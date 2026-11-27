// SPDX-License-Identifier: GPL-2.0-only
/* X2000 accelerator domains: separate power switches from their clock gates.
 * CPU/DDR/TCSM power and peripheral SRAM are deliberately not managed here.
 */
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <dt-bindings/clock/ingenic,x2000-cgu.h>
#include <dt-bindings/power/ingenic,x2000-power.h>
#include "cgu.h"
#include "x2000-power.h"
#include "x2000-power-transaction.h"

#define X2000_LCR 0x04
#define X2000_POWER_TIMEOUT_US 20000
#define X2000_POWER_COUNT 4

static struct ingenic_cgu *power_cgu;

struct x2000_power_domain {
	struct generic_pm_domain genpd;
	struct ingenic_cgu *cgu;
	struct clk *clock;
	struct mutex lock;
	u32 request, acknowledge;
	unsigned int transitions;
	int error, rollback_error;
	bool clock_held, faulted;
};

struct x2000_power_provider {
	struct x2000_power_domain domains[X2000_POWER_COUNT];
	struct generic_pm_domain *pointers[X2000_POWER_COUNT];
	struct genpd_onecell_data onecell;
};

static u32 x2000_power_read(void *context)
{
	struct x2000_power_domain *pd = context;

	return readl(pd->cgu->base + X2000_LCR);
}

static void x2000_power_update(void *context, u32 mask, u32 value)
{
	struct x2000_power_domain *pd = context;
	unsigned long flags;
	u32 word;

	spin_lock_irqsave(&pd->cgu->lock, flags);
	word = readl(pd->cgu->base + X2000_LCR);
	writel((word & ~mask) | (value & mask), pd->cgu->base + X2000_LCR);
	spin_unlock_irqrestore(&pd->cgu->lock, flags);
}

static int x2000_power_wait(void *context, u32 mask, u32 value)
{
	struct x2000_power_domain *pd = context;
	u32 word;

	/* Also used in suspend_noirq: no sleeping timer/interrupt dependency. */
	return readl_poll_timeout_atomic(pd->cgu->base + X2000_LCR, word,
			(word & mask) == value, 1, X2000_POWER_TIMEOUT_US);
}

static int x2000_power_set(struct x2000_power_domain *pd, bool off)
{
	const struct x2000_power_io io = {
		.read = x2000_power_read, .update = x2000_power_update,
		.wait = x2000_power_wait, .context = pd,
	};
	struct x2000_power_result result;
	int error;

	mutex_lock(&pd->lock);
	if (pd->faulted) {
		error = -EIO;
		goto unlock;
	}
	error = clk_prepare_enable(pd->clock);
	if (error)
		goto record;
	pd->clock_held = true;
	result = x2000_power_change(&io, pd->request, pd->acknowledge, off);
	error = result.error;
	pd->rollback_error = result.rollback_error;
	if (!result.state_known) {
		/* Unknown hardware state: leave the parent clock live and refuse
		 * further requests. A reset is required, not optimistic cleanup.
		 */
		pd->faulted = true;
	} else {
		clk_disable_unprepare(pd->clock);
		pd->clock_held = false;
	}
	if (!error && result.changed)
		pd->transitions++;
record:
	pd->error = error;
unlock:
	mutex_unlock(&pd->lock);
	return error;
}

static int x2000_power_on(struct generic_pm_domain *genpd)
{
	return x2000_power_set(container_of(genpd, struct x2000_power_domain,
					  genpd), false);
}

static int x2000_power_off(struct generic_pm_domain *genpd)
{
	return x2000_power_set(container_of(genpd, struct x2000_power_domain,
					  genpd), true);
}

static ssize_t domain_state_show(struct device *dev,
				struct device_attribute *attribute, char *buffer)
{
	struct x2000_power_provider *provider = dev_get_drvdata(dev);
	ssize_t count = sysfs_emit(buffer, "version=1\n");
	unsigned int i;

	for (i = 0; i < X2000_POWER_COUNT; i++) {
		struct x2000_power_domain *pd = &provider->domains[i];
		u32 word;

		mutex_lock(&pd->lock);
		word = x2000_power_read(pd);
		count += sysfs_emit_at(buffer, count,
			"id=%u request=%u acknowledge=%u transitions=%u error=%d rollback=%d clock_held=%u faulted=%u\n",
			i, !!(word & pd->request), !!(word & pd->acknowledge),
			pd->transitions, pd->error, pd->rollback_error,
			pd->clock_held, pd->faulted);
		mutex_unlock(&pd->lock);
	}
	return count;
}
static DEVICE_ATTR_RO(domain_state);
static struct attribute *x2000_power_attrs[] = { &dev_attr_domain_state.attr, NULL };
ATTRIBUTE_GROUPS(x2000_power);

static int x2000_power_probe(struct platform_device *pdev)
{
	static const char *const names[] = { "x2000-isp0", "x2000-isp1", "x2000-felix", "x2000-helix" };
	static const unsigned int clocks[] = { X2000_CLK_ISP0, X2000_CLK_ISP1, X2000_CLK_FELIX, X2000_CLK_HELIX };
	struct x2000_power_provider *provider;
	struct device *dev = &pdev->dev;
	u32 word;
	int i, error;

	if (!power_cgu || power_cgu->np != dev->of_node)
		return -ENODEV;
	if (!of_property_present(dev->of_node, "#power-domain-cells"))
		return -ENODEV;
	provider = devm_kzalloc(dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;
	word = readl(power_cgu->base + X2000_LCR);
	/* No transition is adopted while its request/acknowledgement disagree. */
	if (((word >> 4) ^ word) & GENMASK(27, 24))
		return dev_err_probe(dev, -EBUSY, "incomplete accelerator transition\n");
	for (i = 0; i < X2000_POWER_COUNT; i++) {
		struct x2000_power_domain *pd = &provider->domains[i];

		pd->cgu = power_cgu;
		pd->clock = power_cgu->clocks.clks[clocks[i]];
		if (IS_ERR_OR_NULL(pd->clock)) {
			error = -ENODEV;
			goto remove;
		}
		mutex_init(&pd->lock);
		pd->request = BIT(31 - i);
		pd->acknowledge = BIT(27 - i);
		pd->genpd.name = names[i];
		pd->genpd.power_on = x2000_power_on;
		pd->genpd.power_off = x2000_power_off;
		pd->genpd.flags = GENPD_FLAG_ACTIVE_WAKEUP;
		error = pm_genpd_init(&pd->genpd, NULL, !!(word & pd->acknowledge));
		if (error)
			goto remove;
		provider->pointers[i] = &pd->genpd;
	}
	provider->onecell.domains = provider->pointers;
	provider->onecell.num_domains = X2000_POWER_COUNT;
	error = of_genpd_add_provider_onecell(dev->of_node, &provider->onecell);
	if (error)
		goto remove;
	platform_set_drvdata(pdev, provider);
	return 0;
remove:
	while (--i >= 0)
		pm_genpd_remove(provider->pointers[i]);
	return error;
}

void __init x2000_power_domains_setup(struct ingenic_cgu *cgu)
{
	power_cgu = cgu;
}

static const struct of_device_id x2000_power_match[] = {
	{ .compatible = "ingenic,x2000-cgu" }, { }
};
/* Non-removable: a provider must outlive every attached consumer and callback. */
static struct platform_driver x2000_power_driver = {
	.probe = x2000_power_probe,
	.driver = {
		.name = "ingenic-x2000-power",
		.of_match_table = x2000_power_match,
		.suppress_bind_attrs = true,
		.dev_groups = x2000_power_groups,
	},
};
builtin_platform_driver(x2000_power_driver);
