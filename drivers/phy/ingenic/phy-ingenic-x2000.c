// SPDX-License-Identifier: GPL-2.0
/* X2000 PHY sequence from Ingenic's phy-ingenic-x2000.c, adapted to
 * the generic PHY API. CPM and the additional analog register bank are
 * both required. Board supply control is left to firmware, as in Q2 stock DT.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define USBPCR		0x3c
#define USBPCR1		0x48
#define SRBC		0xc4
#define POR		BIT(22)
#define PORT_RESET	BIT(21)
#define CORE_RESET	BIT(12)

struct x2000_usb_phy {
	void __iomem *cpm;
	void __iomem *analog;
	struct clk *clk;
};

static void x2000_update(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

static int x2000_power_on(struct phy *phy)
{
	struct x2000_usb_phy *p = phy_get_drvdata(phy);
	int ret;

	x2000_update(p->cpm + SRBC, 0, CORE_RESET);
	x2000_update(p->cpm + USBPCR, GENMASK(29, 28), BIT(31) | POR);
	x2000_update(p->cpm + USBPCR1, 0, PORT_RESET);
	usleep_range(10, 20);
	x2000_update(p->cpm + USBPCR, POR, 0);
	usleep_range(10, 20);
	/* CGU enables OPCR.SPENDN0 and clears GATE_USBPHY_CLK. */
	ret = clk_prepare_enable(p->clk);
	if (ret) {
		x2000_update(p->cpm + USBPCR, 0, POR);
		return ret;
	}
	usleep_range(500, 1000);
	x2000_update(p->cpm + USBPCR1, PORT_RESET, 0);
	usleep_range(10, 20);
	x2000_update(p->cpm + SRBC, CORE_RESET, 0);
	/* Pull-down control and chirp/SE0 resume enable, vendor sequence. */
	x2000_update(p->cpm + USBPCR1, 0, GENMASK(30, 28));
	x2000_update(p->analog + 0x64, GENMASK(6, 3), 2 << 3);
	/* Vendor resume: receiver and internal VBUS detector enabled. */
	x2000_update(p->analog + 0x100, 0, BIT(6));
	x2000_update(p->analog + 0x108, BIT(3), 0);
	return 0;
}

static int x2000_power_off(struct phy *phy)
{
	struct x2000_usb_phy *p = phy_get_drvdata(phy);

	x2000_update(p->analog + 0x108, 0, BIT(3));
	x2000_update(p->analog + 0x100, BIT(6), 0);
	/* Stop the controller before its clock and kernel DMA storage disappear. */
	x2000_update(p->cpm + SRBC, 0, CORE_RESET);
	x2000_update(p->cpm + USBPCR1, 0, PORT_RESET);
	x2000_update(p->cpm + USBPCR, 0, POR);
	clk_disable_unprepare(p->clk);
	return 0;
}

static const struct phy_ops x2000_ops = {
	.power_on = x2000_power_on,
	.power_off = x2000_power_off,
	.owner = THIS_MODULE,
};

static int x2000_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct x2000_usb_phy *p;
	struct phy *phy;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->cpm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->cpm))
		return PTR_ERR(p->cpm);
	p->analog = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(p->analog))
		return PTR_ERR(p->analog);
	p->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(p->clk))
		return dev_err_probe(dev, PTR_ERR(p->clk), "PHY clock\n");
	phy = devm_phy_create(dev, NULL, &x2000_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, p);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id x2000_match[] = {
	{ .compatible = "ingenic,usbphy-x2000" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_match);

static struct platform_driver x2000_driver = {
	.probe = x2000_probe,
	.driver = {
		.name = "ingenic-x2000-usb-phy",
		.of_match_table = x2000_match,
	},
};
module_platform_driver(x2000_driver);
MODULE_DESCRIPTION("Ingenic X2000 CPM and analog USB PHY");
MODULE_LICENSE("GPL");
