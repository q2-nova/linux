// SPDX-License-Identifier: GPL-2.0-only
/*
 * X2000 MSC SDHCI glue. Register capabilities and quirks follow the Q2
 * soc_msc.ko and the Ingenic driver by Large Dipper
 * <wangquan.shao@ingenic.cn> in Ingenic-community/linux.
 *
 * The CGU supplies the card clock. Use EXCLK for card identification and
 * the boot PLL parent above 400 kHz; keep the SDHCI divider consistent
 * with the actual CGU rate. UHS phase tuning is intentionally not enabled.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>

#include "sdhci-pltfm.h"

struct x2000_msc {
	struct clk *ext;
	struct clk *pll;
	bool pins_asleep;
};

static void x2000_msc_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);
	struct x2000_msc *msc = sdhci_pltfm_priv(pltfm);
	int ret, enable_ret;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	host->mmc->actual_clock = 0;
	if (!clock)
		return;

	clk_disable_unprepare(pltfm->clk);
	ret = clk_set_parent(pltfm->clk, clock <= 400000 ? msc->ext : msc->pll);
	if (!ret)
		ret = clk_set_rate(pltfm->clk, clock);
	enable_ret = clk_prepare_enable(pltfm->clk);
	if (!ret)
		ret = enable_ret;
	if (ret) {
		dev_err(mmc_dev(host->mmc), "cannot set MSC clock to %u: %d\n", clock, ret);
		return;
	}
	host->max_clk = clk_get_rate(pltfm->clk);
	sdhci_set_clock(host, clock);
}

static const struct sdhci_ops x2000_msc_ops = {
	.set_clock = x2000_msc_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data x2000_msc_data = {
	.ops = &x2000_msc_ops,
	.quirks = SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
		  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_32BIT_DMA_SIZE,
};

static int x2000_msc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm;
	struct x2000_msc *msc;
	struct sdhci_host *host;
	u32 caps = 0x276dc898, caps1 = 0;
	int ret;

	host = sdhci_pltfm_init(pdev, &x2000_msc_data, sizeof(*msc));
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm = sdhci_priv(host);
	msc = sdhci_pltfm_priv(pltfm);
	msc->ext = devm_clk_get_enabled(&pdev->dev, "ext");
	if (IS_ERR(msc->ext))
		return dev_err_probe(&pdev->dev, PTR_ERR(msc->ext), "EXCLK unavailable\n");
	pltfm->clk = devm_clk_get_enabled(&pdev->dev, "sdhci");
	if (IS_ERR(pltfm->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(pltfm->clk), "MSC clock unavailable\n");
	msc->pll = clk_get_parent(pltfm->clk);
	if (!msc->pll)
		return -EINVAL;

	/* soc_msc.ko, sdhci_ingenic_probe +0x520: software capability override. */
	__sdhci_read_caps(host, NULL, &caps, &caps1);
	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;
	sdhci_get_property(pdev);

	/* Stock enables v4 mode on both MSC1 and MSC2 before adding the host. */
	sdhci_enable_v4_mode(host);

	return sdhci_add_host(host);
}

#ifdef CONFIG_PM_SLEEP
static int x2000_msc_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct x2000_msc *msc = sdhci_pltfm_priv(sdhci_priv(host));
	int ret, undo;

	ret = sdhci_pltfm_suspend(dev);
	if (ret)
		return ret;
	/* A powered SDIO wake device must keep its active bus pins. Devices
	 * without a sleep state retain their current setup through the helper.
	 */
	if (mmc_card_keep_power(host->mmc))
		return 0;
	ret = pinctrl_pm_select_sleep_state(dev);
	if (!ret) {
		msc->pins_asleep = true;
		return 0;
	}
	/* A failed state selection may have changed a prefix of the pads.
	 * Restore the complete active state before enabling the controller.
	 */
	undo = pinctrl_pm_select_default_state(dev);
	if (!undo)
		undo = sdhci_pltfm_resume(dev);
	if (undo)
		dev_err(dev, "MSC suspend rollback failed: %d\n", undo);
	return ret;
}

static int x2000_msc_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct x2000_msc *msc = sdhci_pltfm_priv(sdhci_priv(host));
	int ret, undo;

	if (msc->pins_asleep) {
		ret = pinctrl_pm_select_default_state(dev);
		if (ret)
			return ret;
	}
	ret = sdhci_pltfm_resume(dev);
	if (ret) {
		if (msc->pins_asleep) {
			undo = pinctrl_pm_select_sleep_state(dev);
			if (undo)
				dev_err(dev, "MSC resume rollback failed: %d\n", undo);
		}
		return ret;
	}
	msc->pins_asleep = false;
	return 0;
}
#endif
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_msc_pm_ops,
			      x2000_msc_suspend, x2000_msc_resume);

static const struct of_device_id x2000_msc_match[] = {
	{ .compatible = "ingenic,x2000-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_msc_match);

static struct platform_driver x2000_msc_driver = {
	.probe = x2000_msc_probe,
	.remove = sdhci_pltfm_remove,
	.driver = {
		.name = "sdhci-ingenic-x2000",
		.of_match_table = x2000_msc_match,
		.pm = pm_sleep_ptr(&x2000_msc_pm_ops),
	},
};
module_platform_driver(x2000_msc_driver);

MODULE_DESCRIPTION("Ingenic X2000 MSC SDHCI driver");
MODULE_LICENSE("GPL");
