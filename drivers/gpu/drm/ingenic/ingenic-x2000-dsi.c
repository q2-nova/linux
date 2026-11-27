// SPDX-License-Identifier: GPL-2.0-or-later
/* X2000 DSI PHY selector from stock soc_fb.ko and Ingenic's X2000 SDK. */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>

struct x2000_dsi {
	void __iomem *phy;
	struct dw_mipi_dsi *dsi;
	struct dw_mipi_dsi_plat_data pdata;
	u32 selector, lanes;
};

static int x2000_phy_init(void *data)
{
	struct x2000_dsi *x = data;

	writel(x->selector, x->phy + 0x64);
	/* X2000 stock host layout/settings, beyond the generic DW defaults.
	 * These writes follow dw_mipi_dsi_mode_set's timing setup and precede
	 * PHY enable. Do not infer X2000 layout from an unverified IP revision.
	 */
	writel(0x607, x->pdata.base + 0x08);
	writel(0x64280fff, x->pdata.base + 0x9c);
	writel(0x1c00 | (x->lanes - 1), x->pdata.base + 0xa4);
	return 0;
}

static int x2000_lane_rate(void *data, const struct drm_display_mode *mode,
			  unsigned long flags, u32 lanes, u32 format,
			  unsigned int *mbps)
{
	static const struct { u32 first, low, high, step; } ranges[] = {
		{ 0, 63750000, 93125000, 312500 },
		{ 95, 93750000, 186250000, 625000 },
		{ 244, 187500000, 372500000, 1250000 },
		{ 393, 375000000, 745000000, 2500000 },
		{ 542, 750000000, 2750000000U, 5000000 },
	};
	struct x2000_dsi *x = data;
	u64 target;
	int i, bpp = mipi_dsi_pixel_format_to_bpp(format);

	if (!lanes || lanes > 2 || bpp < 0)
		return -EINVAL;
	/* Stock uses 20% bandwidth headroom, rounding the selector down. */
	target = div_u64((u64)mode->clock * 1000 * bpp * 6, 5 * lanes);
	for (i = 0; i < ARRAY_SIZE(ranges); i++) {
		u32 step;

		if (target < ranges[i].low || target > ranges[i].high)
			continue;
		step = div_u64(target - ranges[i].low, ranges[i].step);
		x->selector = ranges[i].first + step;
		x->lanes = lanes;
		*mbps = DIV_ROUND_UP(ranges[i].low + step * ranges[i].step, 1000000);
		return 0;
	}
	return -ERANGE;
}

static int x2000_timing(void *data, unsigned int mbps,
			struct dw_mipi_dsi_dphy_timing *timing)
{
	/* Stock PHY_TMR_CFG=0x64280fff, PHY_TMR_LPCLK_CFG=0. */
	*timing = (struct dw_mipi_dsi_dphy_timing) {
		.data_hs2lp = 100, .data_lp2hs = 40,
	};
	return 0;
}

static const struct dw_mipi_dsi_phy_ops x2000_phy_ops = {
	.init = x2000_phy_init,
	.get_lane_mbps = x2000_lane_rate,
	.get_timing = x2000_timing,
};

static int x2000_dsi_probe(struct platform_device *pdev)
{
	struct x2000_dsi *x;

	x = devm_kzalloc(&pdev->dev, sizeof(*x), GFP_KERNEL);
	if (!x)
		return -ENOMEM;
	x->pdata.base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(x->pdata.base))
		return PTR_ERR(x->pdata.base);
	x->phy = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(x->phy))
		return PTR_ERR(x->phy);
	x->pdata.max_data_lanes = 2;
	x->pdata.phy_ops = &x2000_phy_ops;
	x->pdata.priv_data = x;
	x->dsi = dw_mipi_dsi_probe(pdev, &x->pdata);
	if (IS_ERR(x->dsi))
		return dev_err_probe(&pdev->dev, PTR_ERR(x->dsi), "DSI host\n");
	platform_set_drvdata(pdev, x);
	return 0;
}

static void x2000_dsi_remove(struct platform_device *pdev)
{
	struct x2000_dsi *x = platform_get_drvdata(pdev);

	dw_mipi_dsi_remove(x->dsi);
}

static const struct of_device_id x2000_dsi_match[] = {
	{ .compatible = "ingenic,x2000-dsi" }, { }
};
MODULE_DEVICE_TABLE(of, x2000_dsi_match);
static struct platform_driver x2000_dsi_driver = {
	.probe = x2000_dsi_probe,
	.remove = x2000_dsi_remove,
	.driver = { .name = "ingenic-x2000-dsi", .of_match_table = x2000_dsi_match },
};
module_platform_driver(x2000_dsi_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic X2000 DSI bridge");
