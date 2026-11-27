// SPDX-License-Identifier: GPL-2.0-only
/* Q2 PCM topology from stock codec_cs43131.ko. Emulator-qualified only. */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/cs43130.h"

struct q2_audio {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu, platform, codecs[4];
	struct snd_soc_codec_conf conf[4];
	struct gpio_desc *mute, *mode, *boost;
};

static int q2_output_event(struct snd_soc_dapm_widget *widget,
			   struct snd_kcontrol *control, int event)
{
	struct q2_audio *q2 = snd_soc_card_get_drvdata(snd_soc_dapm_to_card(widget->dapm));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		gpiod_set_value_cansleep(q2->mute, 1);
		msleep(200);
		gpiod_set_value_cansleep(q2->mode, 1);
		break;
	case SND_SOC_DAPM_POST_PMU:
		msleep(50);
		gpiod_set_value_cansleep(q2->mute, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		gpiod_set_value_cansleep(q2->mute, 1);
		msleep(300);
		break;
	case SND_SOC_DAPM_POST_PMD:
		gpiod_set_value_cansleep(q2->mode, 0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget q2_widgets[] = {
	SND_SOC_DAPM_OUT_DRV_E("Output", SND_SOC_NOPM, 0, 0, NULL, 0,
		q2_output_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_HP("Headphones", NULL),
};

static const struct snd_soc_dapm_route q2_routes[] = {
	{ "Output", NULL, "DAC0 HPOUTA" },
	{ "Output", NULL, "DAC0 HPOUTB" },
	{ "Output", NULL, "DAC1 HPOUTA" },
	{ "Output", NULL, "DAC1 HPOUTB" },
	{ "Output", NULL, "DAC2 HPOUTA" },
	{ "Output", NULL, "DAC2 HPOUTB" },
	{ "Output", NULL, "DAC3 HPOUTA" },
	{ "Output", NULL, "DAC3 HPOUTB" },
	{ "Headphones", NULL, "Output" },
};

static int q2_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec;
	int i;

	for_each_rtd_codec_dais(rtd, i, codec) {
		snd_soc_dapm_set_idle_bias(snd_soc_component_to_dapm(codec->component), false);
	}
	return 0;
}

static int q2_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec;
	unsigned int slots[2];
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, codec) {
		/* The SoC supplies MCLK; the board has no qualified DAC crystal.
		 * Match the clock family selected by x2000-audio for this stream.
		 */
		ret = snd_soc_component_set_sysclk(codec->component, 0,
			CS43130_MCLK_SRC_EXT, params_rate(params) == 44100 ?
			CS43130_MCLK_22M : CS43130_MCLK_24M, SND_SOC_CLOCK_IN);
		if (ret)
			return ret;
		ret = snd_soc_dai_set_sysclk(codec, 0, params_rate(params) * 64,
					    SND_SOC_CLOCK_IN);
		if (ret)
			return ret;
		/* Stock addresses 0x30/31 copy left; 0x32/33 copy right.
		 * Invert B in each pair (stock PCM_PATH_CTL_2 values 5/7).
		 */
		slots[0] = slots[1] = i >= 2;
		ret = snd_soc_dai_set_channel_map(codec, 0, NULL, 2, slots);
		if (ret)
			return ret;
		ret = snd_soc_component_update_bits(codec->component,
			CS43130_PCM_PATH_CTL_2, BIT(2) | BIT(3), BIT(2));
		if (ret < 0)
			return ret;
	}
	return 0;
}

static const struct snd_soc_ops q2_ops = {
	.hw_params = q2_hw_params,
};

static void q2_put_nodes(void *data)
{
	struct q2_audio *q2 = data;
	int i;

	of_node_put(q2->cpu.of_node);
	for (i = 0; i < ARRAY_SIZE(q2->codecs); i++)
		of_node_put(q2->codecs[i].of_node);
}

static void q2_mute(void *data)
{
	struct q2_audio *q2 = data;

	gpiod_set_value_cansleep(q2->mute, 1);
	gpiod_set_value_cansleep(q2->mode, 0);
	gpiod_set_value_cansleep(q2->boost, 0);
}

static int q2_audio_probe(struct platform_device *pdev)
{
	static const char * const prefixes[] = { "DAC0", "DAC1", "DAC2", "DAC3" };
	struct device *dev = &pdev->dev;
	struct q2_audio *q2;
	int i, ret;

	q2 = devm_kzalloc(dev, sizeof(*q2), GFP_KERNEL);
	if (!q2)
		return -ENOMEM;
	q2->mute = devm_gpiod_get(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(q2->mute))
		return dev_err_probe(dev, PTR_ERR(q2->mute), "Cannot get mute GPIO\n");
	q2->mode = devm_gpiod_get(dev, "mode", GPIOD_OUT_LOW);
	if (IS_ERR(q2->mode))
		return dev_err_probe(dev, PTR_ERR(q2->mode), "Cannot get mode GPIO\n");
	/* PB2 is switched by stock high-volume policy, not the codec VCP
	 * supply. Keep it low until that policy has been qualified.
	 */
	q2->boost = devm_gpiod_get(dev, "boost", GPIOD_OUT_LOW);
	if (IS_ERR(q2->boost))
		return dev_err_probe(dev, PTR_ERR(q2->boost), "Cannot get boost GPIO\n");
	ret = devm_add_action_or_reset(dev, q2_mute, q2);
	if (ret)
		return ret;
	/* Load this machine driver before the codec driver. ASoC retains an
	 * unbound card until its components register, so these GPIOs remain
	 * owned and muted while the codecs acquire reset and power. Stock waits
	 * 200 ms with mute asserted before enabling the PB1 DAC supply.
	 */
	msleep(200);
	ret = devm_add_action_or_reset(dev, q2_put_nodes, q2);
	if (ret)
		return ret;
	q2->cpu.of_node = of_parse_phandle(dev->of_node, "audio-controller", 0);
	if (!q2->cpu.of_node ||
	    of_count_phandle_with_args(dev->of_node, "audio-codecs", NULL) != 4)
		return -EINVAL;
	q2->platform.of_node = q2->cpu.of_node;
	q2->cpu.dai_name = "x2000-baic1";
	for (i = 0; i < ARRAY_SIZE(q2->codecs); i++) {
		q2->codecs[i].of_node = of_parse_phandle(dev->of_node, "audio-codecs", i);
		if (!q2->codecs[i].of_node)
			return -EINVAL;
		q2->codecs[i].dai_name = "cs43130-asp-pcm";
		q2->conf[i].dlc.of_node = q2->codecs[i].of_node;
		q2->conf[i].name_prefix = prefixes[i];
	}
	q2->link.name = "Q2 PCM";
	q2->link.stream_name = "Q2 PCM";
	q2->link.cpus = &q2->cpu;
	q2->link.num_cpus = 1;
	q2->link.platforms = &q2->platform;
	q2->link.num_platforms = 1;
	q2->link.codecs = q2->codecs;
	q2->link.num_codecs = ARRAY_SIZE(q2->codecs);
	q2->link.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC;
	q2->link.init = q2_link_init;
	q2->link.ops = &q2_ops;
	q2->link.playback_only = 1;
	/* Format/clock changes must follow HP/ASP shutdown, not the generic
	 * five-second delayed DAPM stop (CS43131 DS1155F2 section 4.1.9).
	 */
	q2->link.ignore_pmdown_time = 1;
	q2->card.name = "Shanling Q2";
	q2->card.owner = THIS_MODULE;
	q2->card.dev = dev;
	q2->card.dai_link = &q2->link;
	q2->card.num_links = 1;
	q2->card.codec_conf = q2->conf;
	q2->card.num_configs = ARRAY_SIZE(q2->conf);
	q2->card.dapm_widgets = q2_widgets;
	q2->card.num_dapm_widgets = ARRAY_SIZE(q2_widgets);
	q2->card.dapm_routes = q2_routes;
	q2->card.num_dapm_routes = ARRAY_SIZE(q2_routes);
	snd_soc_card_set_drvdata(&q2->card, q2);
	platform_set_drvdata(pdev, q2);
	return devm_snd_soc_register_card(dev, &q2->card);
}

static void q2_audio_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct q2_audio *q2 = snd_soc_card_get_drvdata(card);

	snd_soc_poweroff(&pdev->dev);
	q2_mute(q2);
}

static const struct of_device_id q2_audio_of_match[] = {
	{ .compatible = "shanling,q2-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, q2_audio_of_match);

static struct platform_driver q2_audio_driver = {
	.probe = q2_audio_probe,
	.shutdown = q2_audio_shutdown,
	.driver = {
		.name = "shanling-q2-audio",
		.pm = &snd_soc_pm_ops,
		.of_match_table = q2_audio_of_match,
	},
};
module_platform_driver(q2_audio_driver);

MODULE_DESCRIPTION("Shanling Q2 PCM sound card");
MODULE_LICENSE("GPL");
