// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PCM playback and tones through one Ingenic X2000 PWM channel, such as the
 * Shanling Q2 clicker on PWM1/PC1.
 *
 * Playback: mono S16_LE. Each sample becomes one or more PWM periods whose
 * duty cycle follows the sample, in a looping DMA buffer. Lower sample rates
 * repeat each sample so the PWM frequency stays at 44.1 or 48 kHz, above
 * hearing. The PWM period in input clocks is clock / (rate * repeat), rounded;
 * at 50 MHz, 48 kHz plays at 47984.6 Hz (-0.03%). The stock Q2
 * cmd_pwm_audio tool supplies the word layout and sample-repeat precedent;
 * this signed fixed-point mapping is an implementation choice:
 * high = period / 2 + sample * period / 65536, clamped to 1..period-1.
 *
 * The hardware's read position is not read back. The pointer is computed
 * from elapsed time and the exact PWM period; both clocks derive from the
 * same crystal.
 *
 * Tones: an input device named "pwm-beeper" with SND_TONE and SND_BELL, as
 * the pwm-beeper driver provides, using ordinary PWM output. Tone requests
 * are ignored while the PCM device is open.
 */
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/hrtimer_bases.h>
#include <linux/input.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/pwm-ingenic-x2000.h>
#include <linux/uio.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

/* Largest repeat (6) times this stays within X2000_PWM_DMA_MAX_WORDS. */
#define X2000_AUDIO_MAX_FRAMES 10920

struct x2000_audio {
	struct device *dev;
	struct device *dma_dev;
	struct pwm_device *pwm;
	unsigned long clk;
	unsigned int bell_hz;

	/* Serializes PCM open/close against tone output. */
	struct mutex lock;
	bool pcm_open;
	bool shutting_down;
	bool suspended;
	bool dma_failed;
	struct input_dev *input;
	struct work_struct tone_work;
	unsigned int tone_hz;

	struct snd_card *card;
	struct snd_pcm_substream *substream;
	u32 *words;
	dma_addr_t words_dma;
	size_t words_bytes;
	unsigned int repeat;
	unsigned int counts;	/* PWM input clocks per period */
	u32 silence;
	struct hrtimer timer;
	ktime_t period_time;
	ktime_t start;
	atomic_t running;
	snd_pcm_uframes_t last_pos;
};

static const unsigned int x2000_audio_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 44100, 48000,
};

static const struct snd_pcm_hw_constraint_list x2000_audio_rate_list = {
	.count = ARRAY_SIZE(x2000_audio_rates),
	.list = x2000_audio_rates,
};

static unsigned int x2000_audio_repeat(unsigned int rate)
{
	return (rate % 11025 ? 48000 : 44100) / rate;
}

static void x2000_audio_tone(struct x2000_audio *a, unsigned int hz)
{
	struct pwm_state state;
	int ret;

	lockdep_assert_held(&a->lock);
	pwm_init_state(a->pwm, &state);
	if (hz) {
		state.period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, hz);
		pwm_set_relative_duty_cycle(&state, 50, 100);
		state.enabled = true;
	} else {
		state.enabled = false;
	}
	ret = pwm_apply_might_sleep(a->pwm, &state);
	if (ret)
		dev_err_ratelimited(a->dev, "cannot set tone %u Hz: %d\n", hz, ret);
}

static void x2000_audio_tone_work(struct work_struct *work)
{
	struct x2000_audio *a = container_of(work, struct x2000_audio, tone_work);

	mutex_lock(&a->lock);
	if (!a->pcm_open && !READ_ONCE(a->shutting_down) && !READ_ONCE(a->suspended))
		x2000_audio_tone(a, READ_ONCE(a->tone_hz));
	mutex_unlock(&a->lock);
}

static int x2000_audio_event(struct input_dev *input, unsigned int type,
			     unsigned int code, int value)
{
	struct x2000_audio *a = input_get_drvdata(input);

	if (READ_ONCE(a->shutting_down) || READ_ONCE(a->suspended))
		return -ESHUTDOWN;
	if (type != EV_SND || value < 0)
		return -EINVAL;
	switch (code) {
	case SND_BELL:
		value = value ? a->bell_hz : 0;
		break;
	case SND_TONE:
		if (value > 20000)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	WRITE_ONCE(a->tone_hz, value);
	schedule_work(&a->tone_work);
	return 0;
}

static void x2000_audio_input_close(struct input_dev *input)
{
	struct x2000_audio *a = input_get_drvdata(input);

	cancel_work_sync(&a->tone_work);
	mutex_lock(&a->lock);
	WRITE_ONCE(a->tone_hz, 0);
	if (!a->pcm_open)
		x2000_audio_tone(a, 0);
	mutex_unlock(&a->lock);
}

static u32 x2000_audio_word(const struct x2000_audio *a, s16 sample)
{
	s32 high = a->counts / 2 + (s32)(((s64)sample * a->counts) >> 16);

	high = clamp_t(s32, high, 1, a->counts - 1);
	return x2000_pwm_dma_word(high, a->counts - high);
}

static void x2000_audio_store(struct x2000_audio *a, snd_pcm_uframes_t frame,
			      u32 word)
{
	u32 *dst = a->words + frame * a->repeat;
	unsigned int i;

	for (i = 0; i < a->repeat; i++)
		dst[i] = word;
}

/* Stop DMA and free the buffer. Leak it if DMA might still read it. */
static void x2000_audio_free_words(struct x2000_audio *a)
{
	int ret;

	atomic_set(&a->running, 0);
	hrtimer_cancel(&a->timer);
	if (!a->words)
		return;
	ret = x2000_pwm_dma_release(a->pwm);
	if (ret) {
		a->dma_failed = true;
		dev_err(a->dev, "DMA did not stop; retaining %zu-byte allocation, PCM disabled\n",
			a->words_bytes);
	} else
		dma_free_coherent(a->dma_dev, a->words_bytes, a->words,
				  a->words_dma);
	a->words = NULL;
}

static enum hrtimer_restart x2000_audio_timer(struct hrtimer *timer)
{
	struct x2000_audio *a = container_of(timer, struct x2000_audio, timer);

	if (!atomic_read(&a->running))
		return HRTIMER_NORESTART;
	snd_pcm_period_elapsed(a->substream);
	if (!atomic_read(&a->running))
		return HRTIMER_NORESTART;
	hrtimer_forward_now(timer, a->period_time);
	return HRTIMER_RESTART;
}

static const struct snd_pcm_hardware x2000_audio_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_KNOT,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = X2000_AUDIO_MAX_FRAMES * 2,
	.period_bytes_min = 64 * 2,
	.period_bytes_max = X2000_AUDIO_MAX_FRAMES,
	.periods_min = 2,
	.periods_max = 256,
};

static int x2000_audio_open(struct snd_pcm_substream *ss)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	int ret;

	if (READ_ONCE(a->shutting_down) || READ_ONCE(a->suspended))
		return -ESHUTDOWN;
	if (a->dma_failed)
		return -EIO;
	runtime->hw = x2000_audio_hw;
	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &x2000_audio_rate_list);
	if (ret < 0)
		return ret;
	/* A multiple of four frames keeps the word count a multiple of four. */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 4);
	if (ret < 0)
		return ret;
	mutex_lock(&a->lock);
	a->pcm_open = true;
	a->substream = ss;
	x2000_audio_tone(a, 0);
	mutex_unlock(&a->lock);
	return 0;
}

static int x2000_audio_close(struct snd_pcm_substream *ss)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);

	x2000_audio_free_words(a);
	mutex_lock(&a->lock);
	a->pcm_open = false;
	a->substream = NULL;
	mutex_unlock(&a->lock);
	return 0;
}

static int x2000_audio_hw_params(struct snd_pcm_substream *ss,
				 struct snd_pcm_hw_params *params)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	unsigned int repeat = x2000_audio_repeat(params_rate(params));
	size_t bytes = params_buffer_size(params) * repeat * sizeof(u32);

	x2000_audio_free_words(a);
	if (a->dma_failed)
		return -EIO;
	a->words = dma_alloc_coherent(a->dma_dev, bytes, &a->words_dma,
				      GFP_KERNEL);
	if (!a->words)
		return -ENOMEM;
	a->words_bytes = bytes;
	return 0;
}

static int x2000_audio_hw_free(struct snd_pcm_substream *ss)
{
	x2000_audio_free_words(snd_pcm_substream_chip(ss));
	return 0;
}

static int x2000_audio_prepare(struct snd_pcm_substream *ss)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	unsigned long counts;
	snd_pcm_uframes_t i;
	int ret;

	if (READ_ONCE(a->shutting_down) || READ_ONCE(a->suspended))
		return -ESHUTDOWN;
	if (!a->words)
		return -EINVAL;
	a->repeat = x2000_audio_repeat(runtime->rate);
	counts = DIV_ROUND_CLOSEST(a->clk, runtime->rate * a->repeat);
	if (counts < 2 || counts > U16_MAX)
		return -EINVAL;
	a->counts = counts;
	a->silence = x2000_audio_word(a, 0);
	/* Stop and synchronize before overwriting memory the old loop could read. */
	atomic_set(&a->running, 0);
	hrtimer_cancel(&a->timer);
	ret = x2000_pwm_dma_release(a->pwm);
	if (ret)
		return ret;
	for (i = 0; i < runtime->buffer_size; i++)
		x2000_audio_store(a, i, a->silence);
	a->period_time = ns_to_ktime(div_u64((u64)runtime->period_size *
					     a->repeat * a->counts *
					     NSEC_PER_SEC, a->clk));
	a->last_pos = 0;
	return x2000_pwm_dma_prepare(a->pwm, a->words_dma,
				     runtime->buffer_size * a->repeat);
}

static int x2000_audio_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (READ_ONCE(a->shutting_down) || READ_ONCE(a->suspended))
			return -ESHUTDOWN;
		a->start = ktime_get();
		x2000_pwm_dma_enable(a->pwm);
		atomic_set(&a->running, 1);
		hrtimer_start(&a->timer, a->period_time, HRTIMER_MODE_REL_SOFT);
		return 0;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		atomic_set(&a->running, 0);
		x2000_pwm_dma_disable(a->pwm);
		/* ALSA holds the stream lock; the callback may need that lock. */
		hrtimer_try_to_cancel(&a->timer);
		return 0;
	default:
		return -EINVAL;
	}
}

static int x2000_audio_sync_stop(struct snd_pcm_substream *ss)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);

	hrtimer_cancel(&a->timer);
	return x2000_pwm_dma_release(a->pwm);
}

static snd_pcm_uframes_t x2000_audio_pointer(struct snd_pcm_substream *ss)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	u64 ns, frames;

	if (!atomic_read(&a->running))
		return a->last_pos;
	ns = ktime_to_ns(ktime_sub(ktime_get(), a->start));
	frames = mul_u64_u64_div_u64(ns, a->clk, (u64)NSEC_PER_SEC *
				     a->counts * a->repeat);
	a->last_pos = do_div(frames, ss->runtime->buffer_size);
	return a->last_pos;
}

static int x2000_audio_copy(struct snd_pcm_substream *ss, int channel,
			    unsigned long pos, struct iov_iter *iter,
			    unsigned long bytes)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	snd_pcm_uframes_t frame = pos / 2, count = bytes / 2, n, i;
	__le16 buf[64];

	while (count) {
		n = min_t(snd_pcm_uframes_t, count, ARRAY_SIZE(buf));
		if (copy_from_iter(buf, n * 2, iter) != n * 2)
			return -EFAULT;
		for (i = 0; i < n; i++)
			x2000_audio_store(a, frame + i,
					  x2000_audio_word(a, (s16)le16_to_cpu(buf[i])));
		frame += n;
		count -= n;
	}
	dma_wmb();
	return 0;
}

static int x2000_audio_fill_silence(struct snd_pcm_substream *ss, int channel,
				    unsigned long pos, unsigned long bytes)
{
	struct x2000_audio *a = snd_pcm_substream_chip(ss);
	snd_pcm_uframes_t frame = pos / 2, end = frame + bytes / 2;

	for (; frame < end; frame++)
		x2000_audio_store(a, frame, a->silence);
	dma_wmb();
	return 0;
}

static const struct snd_pcm_ops x2000_audio_ops = {
	.open = x2000_audio_open,
	.close = x2000_audio_close,
	.hw_params = x2000_audio_hw_params,
	.hw_free = x2000_audio_hw_free,
	.prepare = x2000_audio_prepare,
	.trigger = x2000_audio_trigger,
	.sync_stop = x2000_audio_sync_stop,
	.pointer = x2000_audio_pointer,
	.copy = x2000_audio_copy,
	.fill_silence = x2000_audio_fill_silence,
};

static void x2000_audio_cancel(void *data)
{
	struct x2000_audio *a = data;

	cancel_work_sync(&a->tone_work);
	atomic_set(&a->running, 0);
	hrtimer_cancel(&a->timer);
}

static void x2000_audio_shutdown(struct platform_device *pdev)
{
	struct x2000_audio *a = platform_get_drvdata(pdev);
	struct snd_pcm_substream *ss;
	unsigned long flags;
	int ret;

	WRITE_ONCE(a->shutting_down, true);
	cancel_work_sync(&a->tone_work);
	ss = READ_ONCE(a->substream);
	if (ss)
		snd_pcm_stream_lock_irqsave(ss, flags);
	atomic_set(&a->running, 0);
	x2000_pwm_dma_disable(a->pwm);
	if (ss)
		snd_pcm_stream_unlock_irqrestore(ss, flags);
	hrtimer_cancel(&a->timer);
	mutex_lock(&a->lock);
	ret = x2000_pwm_dma_release(a->pwm);
	if (ret)
		dev_err(a->dev, "DMA still active during shutdown: %d\n", ret);
	else
		x2000_audio_tone(a, 0);
	mutex_unlock(&a->lock);
}

static int x2000_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct x2000_audio *a;
	struct snd_pcm *pcm;
	int ret;

	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	a->dev = dev;
	mutex_init(&a->lock);
	INIT_WORK(&a->tone_work, x2000_audio_tone_work);
	hrtimer_setup(&a->timer, x2000_audio_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_SOFT);
	a->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(a->pwm))
		return dev_err_probe(dev, PTR_ERR(a->pwm), "PWM unavailable\n");
	a->dma_dev = x2000_pwm_dma_device(a->pwm);
	a->clk = x2000_pwm_clock_rate(a->pwm);
	if (!a->dma_dev || !a->clk)
		return dev_err_probe(dev, -ENODEV, "not an X2000 PWM channel\n");
	/* Cancel callbacks before devres releases the PWM device. */
	ret = devm_add_action_or_reset(dev, x2000_audio_cancel, a);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, a);
	ret = x2000_pwm_dma_release(a->pwm);
	if (ret)
		return dev_err_probe(dev, ret, "inherited PWM DMA did not stop\n");
	if (device_property_read_u32(dev, "beeper-hz", &a->bell_hz))
		a->bell_hz = 1000;
	if (a->bell_hz < 10 || a->bell_hz > 10000)
		return dev_err_probe(dev, -EINVAL, "invalid beeper frequency\n");
	mutex_lock(&a->lock);
	x2000_audio_tone(a, 0);
	mutex_unlock(&a->lock);

	a->input = devm_input_allocate_device(dev);
	if (!a->input)
		return -ENOMEM;
	/* The pwm-beeper name and events keep existing users working. */
	a->input->name = "pwm-beeper";
	a->input->phys = "pwm/input0";
	a->input->id.bustype = BUS_HOST;
	a->input->id.version = 0x0100;
	input_set_capability(a->input, EV_SND, SND_TONE);
	input_set_capability(a->input, EV_SND, SND_BELL);
	a->input->event = x2000_audio_event;
	a->input->close = x2000_audio_input_close;
	input_set_drvdata(a->input, a);
	ret = input_register_device(a->input);
	if (ret)
		return ret;

	ret = snd_devm_card_new(dev, SNDRV_DEFAULT_IDX1, "Clicker", THIS_MODULE,
				0, &a->card);
	if (ret)
		return ret;
	strscpy(a->card->driver, "X2000-PWM");
	strscpy(a->card->shortname, "Clicker");
	snprintf(a->card->longname, sizeof(a->card->longname),
		 "Clicker on X2000 PWM%u", a->pwm->hwpwm);
	ret = snd_pcm_new(a->card, "Clicker", 0, 1, 0, &pcm);
	if (ret)
		return ret;
	pcm->private_data = a;
	strscpy(pcm->name, "Clicker");
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &x2000_audio_ops);
	return snd_card_register(a->card);
}

static int x2000_audio_suspend(struct device *dev)
{
	struct x2000_audio *a = dev_get_drvdata(dev);
	struct snd_pcm_substream *ss;
	int ret;

	WRITE_ONCE(a->suspended, true);
	cancel_work_sync(&a->tone_work);
	ss = READ_ONCE(a->substream);
	snd_power_change_state(a->card, SNDRV_CTL_POWER_D3hot);
	if (ss)
		snd_pcm_suspend_all(ss->pcm);
	hrtimer_cancel(&a->timer);
	mutex_lock(&a->lock);
	ret = x2000_pwm_dma_release(a->pwm);
	if (!ret) {
		a->tone_hz = 0;
		x2000_audio_tone(a, 0);
	}
	mutex_unlock(&a->lock);
	if (ret) {
		WRITE_ONCE(a->suspended, false);
		snd_power_change_state(a->card, SNDRV_CTL_POWER_D0);
	}
	return ret;
}

static int x2000_audio_resume(struct device *dev)
{
	struct x2000_audio *a = dev_get_drvdata(dev);

	WRITE_ONCE(a->suspended, false);
	snd_power_change_state(a->card, SNDRV_CTL_POWER_D0);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_audio_pm_ops,
			      x2000_audio_suspend, x2000_audio_resume);

static const struct of_device_id x2000_audio_match[] = {
	{ .compatible = "ingenic,x2000-pwm-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_audio_match);

static struct platform_driver x2000_audio_driver = {
	.probe = x2000_audio_probe,
	.shutdown = x2000_audio_shutdown,
	.driver = {
		.name = "x2000-pwm-audio",
		.pm = pm_sleep_ptr(&x2000_audio_pm_ops),
		.of_match_table = x2000_audio_match,
	},
};
module_platform_driver(x2000_audio_driver);

MODULE_DESCRIPTION("PCM playback and tones through an Ingenic X2000 PWM channel");
MODULE_LICENSE("GPL");
