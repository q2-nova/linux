// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * X2000 standalone PWM controller (not the older TCU PWM block).
 * Register layout and prescaler from Ingenic's pwm-ingenic-v2.c by
 * Sihui Liu <sihui.liu@ingenic.com>, Copyright 2015 Ingenic Semiconductor.
 *
 * Reconfiguration stops the channel and may truncate the current period.
 *
 * Looping DMA waveform output follows the register use of the stock Q2
 * soc_pwm.ko (pwm2_dma_init, pwm2_dma_update, pwm2_dma_disable_loop) and
 * the stock MCU click routine: per-channel mode (0x110) and loop (0x114)
 * bits, buffer address (0x140 + 4n) and word count (0x190 + 4n), DMA
 * enable set/clear (0x100/0x104), start (0x120) and FIFO-empty status (0x124).
 *
 * The module clock runs only while a channel needs it: one enabled with a
 * non-zero duty, or one in DMA mode from prepare until a release that saw
 * the DMA stop. Every register access takes the clock first. A channel
 * that is disabled or enabled at zero duty has no high phase and its
 * initial level is low (INITR), so its output is a constant low, which the
 * gated block keeps (Q2: EN 0x1, WCFG0 0x0000249f, CLKGR1 bit 5 set, the
 * backlight dark). The gate follows the last access by a settle time, so
 * the block has clocked the new state to the pin first.
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/pwm-ingenic-x2000.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define PWM_CCFG(n) (4 * ((n) / 8))
#define PWM_ENS 0x10
#define PWM_ENC 0x14
#define PWM_EN 0x18
#define PWM_UP 0x20
#define PWM_BUSY 0x24
#define PWM_INITR 0x30
#define PWM_WCFG(n) (0xb0 + 4 * (n))
#define PWM_DES 0x100
#define PWM_DEC 0x104
#define PWM_DE 0x108
#define PWM_DCR0 0x110		/* DMA mode */
#define PWM_DCR1 0x114		/* DMA loop */
#define PWM_DSTART 0x120
#define PWM_DFIFO_EMPTY 0x124	/* vendor DFER: FIFO empty */
#define PWM_DFSM 0x128
#define PWM_ICLR 0x134
#define PWM_IMASK 0x138
#define PWM_DADDR(n) (0x140 + 4 * (n))
#define PWM_DCNT(n) (0x190 + 4 * (n))
#define PWM_OEN 0x300

struct x2000_pwm {
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	struct device *dev;
	/* Serializes read-modify-write of registers shared by all channels,
	 * and the clock state below.
	 */
	spinlock_t lock;
	/* Channels whose DMA may still read memory. */
	u16 dma_started;
	/* Channels that need the clock: a non-zero duty, or DMA mode. */
	u16 running;
	/* Register accesses in progress. */
	unsigned int users;
	/* This driver's one clk_enable(), dropped by gate_work. */
	bool clk_held;
	/* Earliest time the clock may stop once nothing needs it. */
	unsigned long gate_at;
	struct delayed_work gate_work;
};

static const struct pwm_ops x2000_pwm_ops;

static void x2000_pwm_update(struct x2000_pwm *p, unsigned int reg,
			     u32 mask, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	writel((readl(p->base + reg) & ~mask) | value, p->base + reg);
	spin_unlock_irqrestore(&p->lock, flags);
}

/* Caller holds p->lock. */
static int x2000_pwm_clk_on(struct x2000_pwm *p)
{
	int ret;

	if (p->clk_held)
		return 0;
	ret = clk_enable(p->clk);
	if (ret)
		return ret;
	p->clk_held = true;
	p->gate_at = jiffies;
	return 0;
}

/* Run the clock for a register access. Atomic context is allowed. */
static int x2000_pwm_clk_get(struct x2000_pwm *p)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&p->lock, flags);
	ret = x2000_pwm_clk_on(p);
	if (!ret)
		p->users++;
	spin_unlock_irqrestore(&p->lock, flags);
	return ret;
}

/* End a register access. The clock stops no sooner than settle_ns plus the
 * second tick from now (one to two tick periods, as the current tick is
 * partly spent), and only if no channel needs it. Atomic context is allowed.
 */
static void x2000_pwm_clk_put(struct x2000_pwm *p, u64 settle_ns)
{
	unsigned long flags, settle = nsecs_to_jiffies(settle_ns) + 2;
	unsigned long at = jiffies + settle;

	spin_lock_irqsave(&p->lock, flags);
	p->users--;
	if (time_after(at, p->gate_at))
		p->gate_at = at;
	spin_unlock_irqrestore(&p->lock, flags);
	schedule_delayed_work(&p->gate_work, settle);
}

static void x2000_pwm_set_running(struct x2000_pwm *p, unsigned int n, bool on)
{
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	if (on)
		p->running |= BIT(n);
	else
		p->running &= ~BIT(n);
	spin_unlock_irqrestore(&p->lock, flags);
}

static void x2000_pwm_gate_work(struct work_struct *work)
{
	struct x2000_pwm *p = container_of(to_delayed_work(work),
					   struct x2000_pwm, gate_work);
	unsigned long flags, wait = 0;

	spin_lock_irqsave(&p->lock, flags);
	if (p->clk_held && !p->running && !p->users) {
		if (time_before(jiffies, p->gate_at)) {
			wait = p->gate_at - jiffies;
		} else {
			clk_disable(p->clk);
			p->clk_held = false;
		}
	}
	spin_unlock_irqrestore(&p->lock, flags);
	if (wait)
		schedule_delayed_work(&p->gate_work, wait);
}

/* Stock's release criterion is DEC followed by DFER (FIFO empty).
 * This is not a separately measured DMA bus-idle indication. May sleep.
 */
static int x2000_pwm_dma_wait_stopped(struct x2000_pwm *p, unsigned int n)
{
	unsigned long flags;
	u32 stopped;
	int ret;

	if (!(READ_ONCE(p->dma_started) & BIT(n)))
		return 0;
	ret = readl_poll_timeout(p->base + PWM_DFIFO_EMPTY, stopped,
				 stopped & BIT(n), 10, 100000);
	if (ret) {
		dev_err(p->dev, "channel %u DMA did not stop\n", n);
		return ret;
	}
	spin_lock_irqsave(&p->lock, flags);
	p->dma_started &= ~BIT(n);
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

/* The clock runs. *running: the programmed channel has a high phase. */
static int x2000_pwm_program(struct x2000_pwm *p, unsigned int n,
			     const struct pwm_state *state, bool *running)
{
	unsigned int shift = (n % 8) * 4, scale;
	u64 period, duty;
	u32 busy;
	int ret;

	/* Never reprogram a channel whose DMA may still be running. */
	ret = x2000_pwm_dma_wait_stopped(p, n);
	if (ret)
		return ret;

	if (!state->enabled) {
		writel(BIT(n), p->base + PWM_ENC);
		x2000_pwm_update(p, PWM_INITR, BIT(n) | BIT(n + 16), 0);
		*running = false;
		return 0;
	}

	/* Bound multiplication, and reject periods smaller than one clock. */
	if (state->period > NSEC_PER_SEC)
		return -ERANGE;
	period = mul_u64_u32_div(state->period, p->rate, NSEC_PER_SEC);
	duty = mul_u64_u32_div(state->duty_cycle, p->rate, NSEC_PER_SEC);
	if (!period)
		return -ERANGE;
	for (scale = 0; period > 65535 && scale < 15; scale++) {
		period >>= 1;
		duty >>= 1;
	}
	if (period > 65535)
		return -ERANGE;

	/* Stock uses GPIO for constant output: a zero low phase does not
	 * produce constant high here. Retain PWM pin ownership and use the
	 * nearest representable waveform, with one low clock at full duty.
	 */
	if (duty >= period) {
		if (period < 2)
			return -ERANGE;
		duty = period - 1;
	}

	ret = readl_poll_timeout(p->base + PWM_BUSY, busy,
				!(busy & BIT(n)), 1, 10000);
	if (ret)
		return ret;
	writel(BIT(n), p->base + PWM_ENC);
	writel(BIT(n), p->base + PWM_DEC);
	x2000_pwm_update(p, PWM_DCR0, BIT(n), 0);
	x2000_pwm_update(p, PWM_CCFG(n), 0xf << shift, scale << shift);
	x2000_pwm_update(p, PWM_INITR, BIT(n) | BIT(n + 16), 0);
	writel((duty << 16) | (period - duty), p->base + PWM_WCFG(n));
	x2000_pwm_update(p, PWM_OEN, BIT(n), BIT(n));
	writel(BIT(n), p->base + PWM_UP);
	writel(BIT(n), p->base + PWM_ENS);
	*running = duty != 0;
	return 0;
}

static int x2000_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct x2000_pwm *p = pwmchip_get_drvdata(chip);
	unsigned int n = pwm->hwpwm;
	u64 period = max(pwm->state.period, state->period);
	bool running;
	int ret;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	/* An error leaves the channel, and its need for the clock, as it was. */
	ret = x2000_pwm_program(p, n, state, &running);
	if (!ret)
		x2000_pwm_set_running(p, n, running);
	/* Two periods of the old or new waveform before the clock may stop. */
	x2000_pwm_clk_put(p, 2 * min_t(u64, period, NSEC_PER_SEC));
	return ret;
}

static struct x2000_pwm *x2000_pwm_of(struct pwm_device *pwm)
{
	if (!pwm || pwm->chip->ops != &x2000_pwm_ops)
		return NULL;
	return pwmchip_get_drvdata(pwm->chip);
}

struct device *x2000_pwm_dma_device(struct pwm_device *pwm)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);

	return p ? p->dev : NULL;
}
EXPORT_SYMBOL_GPL(x2000_pwm_dma_device);

unsigned long x2000_pwm_clock_rate(struct pwm_device *pwm)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);

	return p ? p->rate : 0;
}
EXPORT_SYMBOL_GPL(x2000_pwm_clock_rate);

int x2000_pwm_dma_prepare(struct pwm_device *pwm, dma_addr_t addr, u32 words)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);
	unsigned int n, shift;
	unsigned long flags;
	u32 busy, v;
	int ret;

	if (!p)
		return -EINVAL;
	if (!words || words % 4 || words > X2000_PWM_DMA_MAX_WORDS ||
	    addr % 4 || addr + (u64)words * 4 > SZ_4G)
		return -EINVAL;
	n = pwm->hwpwm;
	shift = (n % 8) * 4;
	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	ret = x2000_pwm_dma_wait_stopped(p, n);
	if (ret)
		goto out;
	ret = readl_poll_timeout(p->base + PWM_BUSY, busy,
				 !(busy & BIT(n)), 1, 10000);
	if (ret)
		goto out;
	writel(BIT(n), p->base + PWM_ENC);
	writel(BIT(n), p->base + PWM_DEC);

	spin_lock_irqsave(&p->lock, flags);
	/* Prescaler 1: buffer counts are PWM input clocks. */
	v = readl(p->base + PWM_CCFG(n)) & ~(0xf << shift);
	writel(v, p->base + PWM_CCFG(n));
	/* Initial and idle output low. */
	v = readl(p->base + PWM_INITR) & ~(BIT(n) | BIT(n + 16));
	writel(v, p->base + PWM_INITR);
	writel(readl(p->base + PWM_DCR0) | BIT(n), p->base + PWM_DCR0);
	writel(readl(p->base + PWM_DCR1) | BIT(n), p->base + PWM_DCR1);
	/* Stock masks the channel interrupt in loop mode. */
	writel(readl(p->base + PWM_IMASK) | BIT(n), p->base + PWM_IMASK);
	writel(BIT(n), p->base + PWM_ICLR);
	writel(addr, p->base + PWM_DADDR(n));
	writel(words, p->base + PWM_DCNT(n));
	/* DMA mode keeps the clock until a release that saw the DMA stop. */
	p->running |= BIT(n);
	spin_unlock_irqrestore(&p->lock, flags);
out:
	x2000_pwm_clk_put(p, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(x2000_pwm_dma_prepare);

void x2000_pwm_dma_enable(struct pwm_device *pwm)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);
	unsigned int n;
	unsigned long flags;

	if (!p)
		return;
	n = pwm->hwpwm;
	spin_lock_irqsave(&p->lock, flags);
	/* Prepare left the clock running; a suspend since then has not. */
	if (x2000_pwm_clk_on(p)) {
		spin_unlock_irqrestore(&p->lock, flags);
		return;
	}
	p->running |= BIT(n);
	p->dma_started |= BIT(n);
	/* Publish coherent waveform writes before the device can fetch them. */
	dma_wmb();
	/* Stock order: DMA enable, DMA start, then channel and output enable. */
	writel(BIT(n), p->base + PWM_DES);
	writel(BIT(n), p->base + PWM_DSTART);
	writel(BIT(n), p->base + PWM_ENS);
	writel(readl(p->base + PWM_OEN) | BIT(n), p->base + PWM_OEN);
	spin_unlock_irqrestore(&p->lock, flags);
}
EXPORT_SYMBOL_GPL(x2000_pwm_dma_enable);

void x2000_pwm_dma_disable(struct pwm_device *pwm)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);
	unsigned int n;

	if (!p)
		return;
	n = pwm->hwpwm;
	if (x2000_pwm_clk_get(p))
		return;
	/* Stop fetching first. Keep the channel enabled so its FIFO can drain;
	 * stock waits for DFER before disabling the waveform/freeing memory.
	 * The channel still counts as running, so the clock stays for the drain.
	 */
	writel(BIT(n), p->base + PWM_DEC);
	x2000_pwm_clk_put(p, 0);
}
EXPORT_SYMBOL_GPL(x2000_pwm_dma_disable);

int x2000_pwm_dma_release(struct pwm_device *pwm)
{
	struct x2000_pwm *p = x2000_pwm_of(pwm);
	unsigned int n;
	int ret;

	if (!p)
		return -EINVAL;
	n = pwm->hwpwm;
	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	writel(BIT(n), p->base + PWM_DEC);
	ret = x2000_pwm_dma_wait_stopped(p, n);
	/* Cut output even if the stop timed out; retain memory ownership. */
	writel(BIT(n), p->base + PWM_ENC);
	/* A DMA that did not stop keeps the clock. */
	if (ret)
		goto out;
	x2000_pwm_update(p, PWM_DCR1, BIT(n), 0);
	x2000_pwm_update(p, PWM_DCR0, BIT(n), 0);
	x2000_pwm_set_running(p, n, false);
out:
	x2000_pwm_clk_put(p, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(x2000_pwm_dma_release);

static int x2000_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			       struct pwm_state *state)
{
	struct x2000_pwm *p = pwmchip_get_drvdata(chip);
	unsigned int n = pwm->hwpwm;
	u32 wave, scale;
	u64 high, low;
	int ret;

	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	wave = readl(p->base + PWM_WCFG(n));
	scale = (readl(p->base + PWM_CCFG(n)) >> ((n % 8) * 4)) & 0xf;
	high = wave >> 16;
	low = wave & 0xffff;
	state->enabled = (readl(p->base + PWM_EN) & BIT(n)) &&
			 (readl(p->base + PWM_OEN) & BIT(n));
	x2000_pwm_clk_put(p, 0);
	state->polarity = PWM_POLARITY_NORMAL;
	state->period = DIV_ROUND_UP_ULL(((high + low) << scale) * NSEC_PER_SEC,
				       p->rate);
	state->duty_cycle = DIV_ROUND_UP_ULL((high << scale) * NSEC_PER_SEC,
					   p->rate);
	return 0;
}

static const struct pwm_ops x2000_pwm_ops = {
	.apply = x2000_pwm_apply,
	.get_state = x2000_pwm_get_state,
};

/* Channels a boot stage left needing the clock. The clock runs. */
static void x2000_pwm_inherit(struct x2000_pwm *p)
{
	u32 en = readl(p->base + PWM_EN), mode = readl(p->base + PWM_DCR0);
	u32 initr = readl(p->base + PWM_INITR);
	unsigned long flags;
	unsigned int n;
	u16 running;

	/* A RAM/kexec boot can inherit enabled or still-busy DMA. Do not
	 * treat that memory as quiescent merely because this driver is new.
	 */
	running = readl(p->base + PWM_DE) | readl(p->base + PWM_DFSM);
	p->dma_started = running;
	/* Enabled in DMA mode, with a high phase, or idling high. */
	for (n = 0; n < 16; n++)
		if ((en & BIT(n)) && (((mode | initr) & BIT(n)) ||
				      readl(p->base + PWM_WCFG(n)) >> 16))
			running |= BIT(n);
	spin_lock_irqsave(&p->lock, flags);
	p->running = running;
	spin_unlock_irqrestore(&p->lock, flags);
}

/* Stop the clock now, whatever needs it. May sleep. */
static void x2000_pwm_clk_stop(void *data)
{
	struct x2000_pwm *p = data;
	unsigned long flags;

	cancel_delayed_work_sync(&p->gate_work);
	spin_lock_irqsave(&p->lock, flags);
	if (p->clk_held)
		clk_disable(p->clk);
	p->clk_held = false;
	p->running = 0;
	spin_unlock_irqrestore(&p->lock, flags);
}

static int x2000_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct x2000_pwm *p;
	struct clk *clk;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, 16, sizeof(*p));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	p = pwmchip_get_drvdata(chip);
	p->dev = &pdev->dev;
	spin_lock_init(&p->lock);
	INIT_DELAYED_WORK(&p->gate_work, x2000_pwm_gate_work);
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	/* The DMA address registers hold 32-bit physical addresses. */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	clk = devm_clk_get_prepared(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk), "PWM clock unavailable\n");
	ret = devm_clk_rate_exclusive_get(&pdev->dev, clk);
	if (ret)
		return ret;
	p->clk = clk;
	p->rate = clk_get_rate(clk);
	if (!p->rate || p->rate > NSEC_PER_SEC)
		return -EINVAL;
	ret = devm_add_action(&pdev->dev, x2000_pwm_clk_stop, p);
	if (ret)
		return ret;
	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	x2000_pwm_inherit(p);
	x2000_pwm_clk_put(p, 0);
	platform_set_drvdata(pdev, chip);
	chip->ops = &x2000_pwm_ops;
	return devm_pwmchip_add(&pdev->dev, chip);
}

static int x2000_pwm_suspend(struct device *dev)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct x2000_pwm *p = pwmchip_get_drvdata(chip);
	u32 busy;
	int ret;

	ret = x2000_pwm_clk_get(p);
	if (ret)
		return ret;
	/* PWM consumers have device links to this supplier. Backlight and
	 * PCM/tone PM must have stopped outputs and drained DMA first. Refuse
	 * an active unowned channel instead of hiding a memory-read hazard.
	 */
	busy = (readl(p->base + PWM_EN) | readl(p->base + PWM_DE) |
		readl(p->base + PWM_DFSM) | READ_ONCE(p->dma_started)) & 0xffff;
	x2000_pwm_clk_put(p, 0);
	if (busy)
		return -EBUSY;
	/* Every channel is disabled and no DMA runs. As before this change,
	 * the clock stops here, without the settle time.
	 */
	x2000_pwm_clk_stop(p);
	clk_unprepare(p->clk);
	return 0;
}

static int x2000_pwm_resume(struct device *dev)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct x2000_pwm *p = pwmchip_get_drvdata(chip);

	/* The clock stays gated until a consumer programs a channel. */
	return clk_prepare(p->clk);
}
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_pwm_pm_ops,
			      x2000_pwm_suspend, x2000_pwm_resume);

static const struct of_device_id x2000_pwm_match[] = {
	{ .compatible = "ingenic,x2000-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_pwm_match);

static struct platform_driver x2000_pwm_driver = {
	.probe = x2000_pwm_probe,
	.driver = {
		.name = "ingenic-x2000-pwm",
		.pm = pm_sleep_ptr(&x2000_pwm_pm_ops),
		.of_match_table = x2000_pwm_match,
	},
};
module_platform_driver(x2000_pwm_driver);
MODULE_DESCRIPTION("Ingenic X2000 standalone PWM controller");
MODULE_LICENSE("GPL");
