// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Operating system timers (OST) of the Ingenic X2000.
 *
 * The global OST (first register range) is a free-running 64-bit counter,
 * used as clocksource and sched_clock. Each core has its own 32-bit OST
 * (second register range, 0x100 apart) that counts up to a compare value,
 * flags it and restarts from zero; its interrupt reaches only that core,
 * on CP0 IP4. Both count the OST clock (EXCLK, 24 MHz) undivided.
 *
 * vdso (default 1, run-time): the global counter's page is mapped read-only
 * and uncached below every process's VDSO data, and clock_gettime() and
 * gettimeofday() read it there instead of entering the kernel. 0 makes the
 * VDSO fall back to the system call from the next timekeeping update.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/sched_clock.h>

/* global OST */
#define GOST_CCR		0x00	/* clock control: prescale */
#define GOST_ER			0x04	/* bit 0: run */
#define GOST_CR			0x08	/* bit 0: clear */
#define GOST_CNTH		0x0c
#define GOST_CNTL		0x10

/* per-core OST, at 0x100 * cpu */
#define COST_STRIDE		0x100
#define COST_CCR		0x00
#define COST_ER			0x04	/* bit 0: run */
#define COST_CR			0x08	/* bit 0: clear the counter */
#define COST_FR			0x0c	/* bit 0: compare matched */
#define COST_MR			0x10	/* bit 0: mask the interrupt */
#define COST_DFR		0x14	/* compare value */

struct x2000_ost_cpu {
	struct clock_event_device cevt;
	void __iomem *base;
	char name[16];
};

static void __iomem *gost;
static void __iomem *cost;
static unsigned long ost_rate;
static int ost_irq;
static DEFINE_PER_CPU(struct x2000_ost_cpu, x2000_ost_cpu);

/* The global OST page for the VDSO (arch/mips/kernel/vdso.c), or 0. */
unsigned long x2000_ost_vdso_pfn;
static bool vdso = true;

static u64 notrace x2000_ost_read(void)
{
	u32 hi, lo;

	do {
		hi = readl(gost + GOST_CNTH);
		lo = readl(gost + GOST_CNTL);
	} while (hi != readl(gost + GOST_CNTH));

	return (u64)hi << 32 | lo;
}

static u64 x2000_ost_cs_read(struct clocksource *cs)
{
	return x2000_ost_read();
}

static struct clocksource x2000_ost_cs = {
	.name	= "x2000-ost",
	.rating	= 400,
	.read	= x2000_ost_cs_read,
	.mask	= CLOCKSOURCE_MASK(64),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void x2000_ost_vdso_mode(void)
{
#ifdef CONFIG_GENERIC_GETTIMEOFDAY
	WRITE_ONCE(x2000_ost_cs.vdso_clock_mode, READ_ONCE(vdso) && x2000_ost_vdso_pfn ?
		   VDSO_CLOCKMODE_X2000_OST : VDSO_CLOCKMODE_NONE);
#endif
}

static int x2000_ost_set_vdso(const char *val, const struct kernel_param *kp)
{
	int err = param_set_bool(val, kp);

	if (!err)
		x2000_ost_vdso_mode();
	return err;
}

static const struct kernel_param_ops x2000_ost_vdso_ops = {
	.set = x2000_ost_set_vdso,
	.get = param_get_bool,
};
module_param_cb(vdso, &x2000_ost_vdso_ops, &vdso, 0644);
MODULE_PARM_DESC(vdso, "read the clock in the VDSO, without a system call");

static struct x2000_ost_cpu *to_ost_cpu(struct clock_event_device *evt)
{
	return container_of(evt, struct x2000_ost_cpu, cevt);
}

static int x2000_ost_shutdown(struct clock_event_device *evt)
{
	struct x2000_ost_cpu *t = to_ost_cpu(evt);

	/* Stopping the counter does not clear an already latched compare.
	 * Mask and acknowledge it too, or WAIT can wake with Status.IE clear
	 * from a final timer compare racing the clockevent shutdown.
	 */
	writel(0, t->base + COST_ER);
	writel(1, t->base + COST_MR);
	writel(0, t->base + COST_FR);

	return 0;
}

static void x2000_ost_start(struct x2000_ost_cpu *t, u32 ticks)
{
	writel(0, t->base + COST_ER);
	writel(ticks, t->base + COST_DFR);
	writel(1, t->base + COST_CR);
	writel(0, t->base + COST_FR);
	writel(0, t->base + COST_MR);
	writel(1, t->base + COST_ER);
}

static int x2000_ost_set_next_event(unsigned long ticks,
				    struct clock_event_device *evt)
{
	x2000_ost_start(to_ost_cpu(evt), ticks);

	return 0;
}

static int x2000_ost_set_periodic(struct clock_event_device *evt)
{
	x2000_ost_start(to_ost_cpu(evt), DIV_ROUND_CLOSEST(ost_rate, HZ));

	return 0;
}

static irqreturn_t x2000_ost_irq(int irq, void *dev_id)
{
	struct x2000_ost_cpu *t = this_cpu_ptr(&x2000_ost_cpu);

	if (!readl(t->base + COST_FR))
		return IRQ_NONE;
	writel(0, t->base + COST_FR);
	if (clockevent_state_oneshot(&t->cevt))
		writel(0, t->base + COST_ER);
	t->cevt.event_handler(&t->cevt);

	return IRQ_HANDLED;
}

static int x2000_ost_cpu_starting(unsigned int cpu)
{
	struct x2000_ost_cpu *t = per_cpu_ptr(&x2000_ost_cpu, cpu);

	t->base = cost + COST_STRIDE * cpu;
	writel(0, t->base + COST_ER);
	writel(0, t->base + COST_CCR);
	writel(0, t->base + COST_FR);
	writel(0, t->base + COST_MR);

	snprintf(t->name, sizeof(t->name), "x2000-ost%u", cpu);
	t->cevt.name = t->name;
	t->cevt.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC;
	t->cevt.rating = 400;
	t->cevt.irq = ost_irq;
	t->cevt.cpumask = cpumask_of(cpu);
	t->cevt.set_state_shutdown = x2000_ost_shutdown;
	t->cevt.set_state_periodic = x2000_ost_set_periodic;
	t->cevt.set_state_oneshot = x2000_ost_shutdown;
	t->cevt.set_next_event = x2000_ost_set_next_event;
	clockevents_config_and_register(&t->cevt, ost_rate, 4, 0xffffffff);
	enable_percpu_irq(ost_irq, IRQ_TYPE_NONE);

	return 0;
}

static int x2000_ost_cpu_dying(unsigned int cpu)
{
	struct x2000_ost_cpu *t = per_cpu_ptr(&x2000_ost_cpu, cpu);

	disable_percpu_irq(ost_irq);
	writel(0, t->base + COST_ER);
	writel(1, t->base + COST_MR);

	return 0;
}

static int __init x2000_ost_init(struct device_node *np)
{
	struct resource res;
	struct clk *clk;
	int err;

	gost = of_iomap(np, 0);
	cost = of_iomap(np, 1);
	if (!gost || !cost) {
		pr_err("x2000-ost: cannot map registers\n");
		return -ENXIO;
	}

	clk = of_clk_get_by_name(np, "ost");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	err = clk_prepare_enable(clk);
	if (err)
		return err;
	ost_rate = clk_get_rate(clk);

	ost_irq = irq_of_parse_and_map(np, 0);
	if (!ost_irq)
		return -EINVAL;
	err = irq_set_percpu_devid(ost_irq);
	if (err)
		return err;
	irq_set_handler(ost_irq, handle_percpu_devid_irq);

	writel(0, gost + GOST_CCR);
	writel(1, gost + GOST_CR);
	writel(1, gost + GOST_ER);
	/* The VDSO reads the counter at fixed offsets from the page start. */
	if (!of_address_to_resource(np, 0, &res) && !(res.start & ~PAGE_MASK))
		x2000_ost_vdso_pfn = PFN_DOWN(res.start);
	x2000_ost_vdso_mode();
	err = clocksource_register_hz(&x2000_ost_cs, ost_rate);
	if (err)
		return err;
	sched_clock_register(x2000_ost_read, 64, ost_rate);

	err = request_percpu_irq(ost_irq, x2000_ost_irq, "x2000-ost",
				 &x2000_ost_cpu);
	if (err)
		return err;

	return cpuhp_setup_state(CPUHP_AP_INGENIC_X2000_OST_STARTING,
				 "clockevents/ingenic/x2000-ost:starting",
				 x2000_ost_cpu_starting, x2000_ost_cpu_dying);
}
TIMER_OF_DECLARE(x2000_ost, "ingenic,x2000-ost", x2000_ost_init);
