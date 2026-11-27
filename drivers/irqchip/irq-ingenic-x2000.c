// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Interrupt controller of the Ingenic X2000.
 *
 * The 64 peripheral interrupts have one register bank per core, at
 * 0x100 * cpu. Each bank holds two groups of 32 sources (at 0x00 and 0x20)
 * with status (+0x00), mask (+0x04), mask set (+0x08), mask clear (+0x0c)
 * and pending (+0x10) registers; pending is the status not masked in that
 * bank. A core sees CP0 IP2 while its bank has pending interrupts.
 *
 * An interrupt is unmasked in the bank of exactly one core, which is how
 * its affinity is set.
 */

#include <linux/bitops.h>
#include <linux/cpumask.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of_clk.h>
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/smp.h>

#define X2000_INTC_NIRQ		64
#define X2000_INTC_BANK		0x100

#define INTC_MASK		0x04
#define INTC_MASK_SET		0x08
#define INTC_MASK_CLR		0x0c
#define INTC_PENDING		0x10
#define INTC_GROUP(hw)		(((hw) / 32) * 0x20)

struct x2000_intc {
	void __iomem *base;
	void __iomem *wake_base;
	struct device_node *node;
	struct clk *wake_clk;
	u32 wake_mask[2], saved_wake_mask[2];
	bool wake_clock_ready, wake_active;
	raw_spinlock_t wake_lock;
	struct irq_domain *domain;
	/* core that takes each interrupt */
	u8 target[X2000_INTC_NIRQ];
};

static struct x2000_intc x2000_intc;

static void __iomem *bank_reg(unsigned int cpu, irq_hw_number_t hw,
			      unsigned int reg)
{
	return x2000_intc.base + X2000_INTC_BANK * cpu + INTC_GROUP(hw) + reg;
}

static void x2000_intc_mask(struct irq_data *d)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);

	writel(BIT(hw % 32), bank_reg(x2000_intc.target[hw], hw, INTC_MASK_SET));
}

static void x2000_intc_unmask(struct irq_data *d)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);

	writel(BIT(hw % 32), bank_reg(x2000_intc.target[hw], hw, INTC_MASK_CLR));
}

static int x2000_intc_set_affinity(struct irq_data *d,
				   const struct cpumask *dest, bool force)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned int cpu, old = x2000_intc.target[hw];

	cpu = force ? cpumask_first(dest) : cpumask_first_and(dest, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	if (cpu != old) {
		bool enabled = !irqd_irq_masked(d);

		writel(BIT(hw % 32), bank_reg(old, hw, INTC_MASK_SET));
		x2000_intc.target[hw] = cpu;
		if (enabled)
			writel(BIT(hw % 32), bank_reg(cpu, hw, INTC_MASK_CLR));
	}
	irq_data_update_effective_affinity(d, cpumask_of(cpu));

	return IRQ_SET_MASK_OK_DONE;
}

/* The external, clock-gated bank is independent of the per-core masks.
 * Stock enables gate_intc and programs this bank for system sleep. */
static int x2000_intc_set_wake(struct irq_data *d, unsigned int on)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&x2000_intc.wake_lock, flags);
	if (on)
		x2000_intc.wake_mask[hw / 32] |= BIT(hw % 32);
	else
		x2000_intc.wake_mask[hw / 32] &= ~BIT(hw % 32);
	raw_spin_unlock_irqrestore(&x2000_intc.wake_lock, flags);
	return 0;
}

static int x2000_intc_suspend(void *data)
{
	struct x2000_intc *intc = data;
	int ret;

	/* Clock providers initialize after init_IRQ. Preparation happens later
	 * with sleeping allowed; syscore only uses atomic enable/disable. */
	if (!intc->wake_clock_ready)
		return -ENODEV;
	ret = clk_enable(intc->wake_clk);
	if (ret)
		return ret;
	/* Save both groups before changing either, so every failure can unwind. */
	for (unsigned int group = 0; group < 2; group++)
		intc->saved_wake_mask[group] = readl(intc->wake_base +
						group * 0x20 + INTC_MASK);
	for (unsigned int group = 0; group < 2; group++) {
		void __iomem *reg = intc->wake_base + group * 0x20 + INTC_MASK;

		writel(~intc->wake_mask[group], reg);
		if (readl(reg) != ~intc->wake_mask[group]) {
			for (unsigned int saved = 0; saved < 2; saved++) {
				void __iomem *restore = intc->wake_base + saved * 0x20 + INTC_MASK;

				writel(intc->saved_wake_mask[saved], restore);
				readl(restore);
			}
			clk_disable(intc->wake_clk);
			return -EIO;
		}
	}
	intc->wake_active = true;
	return 0;
}

static void x2000_intc_resume(void *data)
{
	struct x2000_intc *intc = data;

	if (!intc->wake_active)
		return;
	for (unsigned int group = 0; group < 2; group++) {
		void __iomem *reg = intc->wake_base + group * 0x20 + INTC_MASK;

		writel(intc->saved_wake_mask[group], reg);
		readl(reg);
	}
	intc->wake_active = false;
	clk_disable(intc->wake_clk);
}

static const struct syscore_ops x2000_intc_pm_ops = {
	.suspend = x2000_intc_suspend,
	.resume = x2000_intc_resume,
};

static struct syscore x2000_intc_syscore = {
	.ops = &x2000_intc_pm_ops,
	.data = &x2000_intc,
};

static int __init x2000_intc_prepare_wake(void)
{
	struct clk *clk;
	int ret;

	if (!x2000_intc.node)
		return 0;
	clk = of_clk_get_by_name(x2000_intc.node, "wake");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	ret = clk_prepare(clk);
	if (ret) {
		clk_put(clk);
		return ret;
	}
	x2000_intc.wake_clk = clk;
	x2000_intc.wake_clock_ready = true;
	return 0;
}
late_initcall(x2000_intc_prepare_wake);

static struct irq_chip x2000_intc_chip = {
	.name			= "x2000-intc",
	.irq_mask		= x2000_intc_mask,
	.irq_unmask		= x2000_intc_unmask,
	.irq_set_affinity	= x2000_intc_set_affinity,
	.irq_set_wake		= x2000_intc_set_wake,
	.flags			= IRQCHIP_MASK_ON_SUSPEND,
};

static void x2000_intc_cascade(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cpu = smp_processor_id();

	chained_irq_enter(chip, desc);
	for (int group = 0; group < X2000_INTC_NIRQ / 32; group++) {
		unsigned long pending = readl(bank_reg(cpu, group * 32,
						       INTC_PENDING));
		unsigned int bit;

		for_each_set_bit(bit, &pending, 32)
			generic_handle_domain_irq(x2000_intc.domain,
						  group * 32 + bit);
	}
	chained_irq_exit(chip, desc);
}

static int x2000_intc_map(struct irq_domain *d, unsigned int irq,
			  irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &x2000_intc_chip, handle_level_irq);
	irq_set_status_flags(irq, IRQ_LEVEL);
	irq_data_update_effective_affinity(irq_get_irq_data(irq),
					   cpumask_of(0));

	return 0;
}

static const struct irq_domain_ops x2000_intc_domain_ops = {
	.map	= x2000_intc_map,
	.xlate	= irq_domain_xlate_onecell,
};

static int __init x2000_intc_of_init(struct device_node *np,
				     struct device_node *parent)
{
	unsigned int parent_irq;

	x2000_intc.base = of_iomap(np, 0);
	if (!x2000_intc.base)
		return -ENOMEM;

	x2000_intc.wake_base = of_iomap(np, 1);
	if (!x2000_intc.wake_base)
		goto err_unmap;
	raw_spin_lock_init(&x2000_intc.wake_lock);

	/* Everything masked on every core. */
	for (unsigned int cpu = 0; cpu < num_possible_cpus(); cpu++)
		for (int hw = 0; hw < X2000_INTC_NIRQ; hw += 32)
			writel(~0u, bank_reg(cpu, hw, INTC_MASK_SET));

	parent_irq = irq_of_parse_and_map(np, 0);
	if (!parent_irq)
		goto err_unmap;

	x2000_intc.domain = irq_domain_create_linear(of_fwnode_handle(np),
						     X2000_INTC_NIRQ,
						     &x2000_intc_domain_ops,
						     NULL);
	if (!x2000_intc.domain)
		goto err_unmap;

	irq_set_chained_handler(parent_irq, x2000_intc_cascade);
	x2000_intc.node = of_node_get(np);
	register_syscore(&x2000_intc_syscore);

	return 0;

err_unmap:
	if (x2000_intc.wake_base)
		iounmap(x2000_intc.wake_base);
	iounmap(x2000_intc.base);
	return -EINVAL;
}
IRQCHIP_DECLARE(x2000_intc, "ingenic,x2000-intc", x2000_intc_of_init);
