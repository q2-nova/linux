// SPDX-License-Identifier: GPL-2.0-only
/* X2000 TCU watchdog: RTC counter and CLRZ protocol from Ingenic's SDK. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/ingenic-x2000-watchdog.h>
#include <linux/watchdog.h>
#include "ingenic-x2000-wdt-timing.h"

#define WDT_TDR		0x00
#define WDT_TCER	0x04
#define WDT_TCNT	0x08
#define WDT_TCSR	0x0c
#define TCU_TSR		0x1c
#define TCU_TFCR	0x28
#define TCU_TMSR	0x34
#define TCU_TSCR	0x3c
#define WDT_STOP	BIT(16)
#define WDT_FLAG	BIT(24)
#define WDT_RTC		BIT(1)
#define WDT_CLRZ	BIT(10)
#define WDT_DIV1024	(5 << 3)

struct x2000_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	struct clk *tcu;
	struct clk *rtc;
	unsigned long rtc_rate;
	unsigned int prescale;
	bool suspend_running;
	struct clk *sleep_rtc, *suspend_parent;
	unsigned int suspend_timeout;
	u32 queued_generation, recovery_generation, ready_generation;
	unsigned int recovery_seconds;
	int recovery_fault;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

static bool keep_armed = true;
module_param(keep_armed, bool, 0644);
MODULE_PARM_DESC(keep_armed, "An orderly stop leaves the counter running and the kernel feeding it (default 1)");

/* APIs below are called only under system-sleep serialization, with
 * userspace frozen. The Q2 uses this static SoC watchdog built-in.
 */
static struct x2000_wdt *recovery_wdt;
static void x2000_wdt_drop_recovery(void *data)
{
	struct x2000_wdt *wdt = data;

	WRITE_ONCE(wdt->ready_generation, 0);
	WRITE_ONCE(wdt->queued_generation, 0);
	if (READ_ONCE(recovery_wdt) == wdt)
		WRITE_ONCE(recovery_wdt, NULL);
}

static int x2000_wdt_ping(struct watchdog_device *wdd)
{
	struct x2000_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 control = readl(wdt->base + WDT_TCSR);

	writel(control | WDT_CLRZ, wdt->base + WDT_TCSR);
	return 0;
}

static int x2000_wdt_set_timeout(struct watchdog_device *wdd, unsigned int secs)
{
	struct x2000_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long hz = clk_get_rate(wdt->rtc);
	u32 ticks, enabled;

	/* The selected source can change across a stopped system-PM interval. */
	if (!x2000_wdt_clock_valid(hz))
		return -EINVAL;
	ticks = x2000_wdt_ticks(hz, secs);
	if (!ticks)
		return -EINVAL;
	enabled = readb(wdt->base + WDT_TCER) & 1;

	writeb(0, wdt->base + WDT_TCER);
	/* No pretimeout IRQ is advertised: mask and clear the SDK half flag. */
	writel(WDT_FLAG, wdt->base + TCU_TMSR);
	writel(WDT_FLAG, wdt->base + TCU_TFCR);
	wdt->rtc_rate = hz;
	wdt->prescale = 1024;
	writel(ticks, wdt->base + WDT_TDR);
	writew(WDT_RTC | WDT_DIV1024 | WDT_CLRZ, wdt->base + WDT_TCSR);
	writel(WDT_STOP, wdt->base + TCU_TSCR);
	writeb(enabled, wdt->base + WDT_TCER);
	wdd->timeout = secs;
	return 0;
}

static int x2000_wdt_start(struct watchdog_device *wdd)
{
	struct x2000_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = x2000_wdt_set_timeout(wdd, wdd->timeout);

	if (ret)
		return ret;
	writeb(1, wdt->base + WDT_TCER);
	return 0;
}

/* Stop the counter: the system-sleep path, which re-arms it itself. */
static int x2000_wdt_halt(struct watchdog_device *wdd)
{
	struct x2000_wdt *wdt = watchdog_get_drvdata(wdd);

	writeb(0, wdt->base + WDT_TCER);
	return 0;
}

/*
 * A process gives the watchdog up in order (magic close, WDIOS_DISABLECARD).
 * With keep_armed the counter keeps running and the watchdog core feeds it
 * from its own timer until a process opens the device again
 * (WDOG_HW_RUNNING), so a stop of both CPUs still ends in a reset while no
 * process owns the watchdog. On the Q2 that is every interval in which the
 * supervisor is stopped: a player restart, USB storage mode, an install.
 * A close without the magic character is unchanged: the core leaves the
 * device active, feeds nothing, and the hardware resets.
 */
static int x2000_wdt_stop(struct watchdog_device *wdd)
{
	if (READ_ONCE(keep_armed) && (readb(((struct x2000_wdt *)
	    watchdog_get_drvdata(wdd))->base + WDT_TCER) & 1)) {
		set_bit(WDOG_HW_RUNNING, &wdd->status);
		return 0;
	}
	return x2000_wdt_halt(wdd);
}

static unsigned int x2000_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct x2000_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 full = readl(wdt->base + WDT_TDR) & 0xffff;
	u32 count = readl(wdt->base + WDT_TCNT) & 0xffff;

	return x2000_wdt_seconds_left(wdt->rtc_rate, wdt->prescale, full, count);
}

/* Reboot must also work after an orderly userspace watchdog disarm.
 * This callback runs only on machine restart, never during kexec. */
static int x2000_wdt_restart(struct watchdog_device *wdd,
                            unsigned long action, void *data)
{
	x2000_wdt_set_timeout(wdd, 1);
	x2000_wdt_start(wdd);
	mdelay(1500);
	return 0;
}

static const struct watchdog_info x2000_wdt_info = {
	.identity = "Ingenic X2000 watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops x2000_wdt_ops = {
	.owner = THIS_MODULE,
	.start = x2000_wdt_start,
	.stop = x2000_wdt_stop,
	.ping = x2000_wdt_ping,
	.set_timeout = x2000_wdt_set_timeout,
	.get_timeleft = x2000_wdt_get_timeleft,
	.restart = x2000_wdt_restart,
};

static void x2000_wdt_release_clock(void *data)
{
	struct x2000_wdt *wdt = data;

	/* Probe failure/unbind must not disable an inherited reset deadline. */
	if (!(readb(wdt->base + WDT_TCER) & 1))
		clk_disable_unprepare(wdt->tcu);
}

static int x2000_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct x2000_wdt *wdt;
	u32 control, full, div;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);
	wdt->rtc = devm_clk_get(dev, "rtc");
	if (IS_ERR(wdt->rtc))
		return dev_err_probe(dev, PTR_ERR(wdt->rtc), "watchdog source unavailable\n");
	wdt->sleep_rtc = devm_clk_get_enabled(dev, "sleep");
	if (IS_ERR(wdt->sleep_rtc))
		return dev_err_probe(dev, PTR_ERR(wdt->sleep_rtc), "sleep clock unavailable\n");
	if (clk_get_rate(wdt->sleep_rtc) != 32768)
		return dev_err_probe(dev, -EINVAL, "sleep clock must be32768Hz\n");
	wdt->rtc_rate = clk_get_rate(wdt->rtc);
	/* OPCR[2] selects EXCLK/512 or32768Hz; neither is the RTC seconds gate. */
	if (!x2000_wdt_clock_valid(wdt->rtc_rate))
		return dev_err_probe(dev, -EINVAL, "unsupported watchdog input rate\n");
	wdt->tcu = devm_clk_get(dev, "tcu");
	if (IS_ERR(wdt->tcu))
		return dev_err_probe(dev, PTR_ERR(wdt->tcu), "TCU clock unavailable\n");
	ret = clk_prepare_enable(wdt->tcu);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, x2000_wdt_release_clock, wdt);
	if (ret)
		return ret;

	wdt->prescale = 1024;
	wdt->wdd.info = &x2000_wdt_info;
	wdt->wdd.ops = &x2000_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = x2000_wdt_max_seconds(wdt->rtc_rate);
	wdt->wdd.timeout = 15;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_restart_priority(&wdt->wdd, 128);
	ret = watchdog_init_timeout(&wdt->wdd, 0, dev);
	if (ret)
		return ret;

	if ((readb(wdt->base + WDT_TCER) & 1) &&
	    !(readl(wdt->base + TCU_TSR) & WDT_STOP)) {
		control = readl(wdt->base + WDT_TCSR);
		div = (control >> 3) & 7;
		full = readl(wdt->base + WDT_TDR) & 0xffff;
		if ((control & 7) != WDT_RTC || div > 5 || !full)
			return dev_err_probe(dev, -EINVAL,
					     "unsupported armed watchdog clock/compare\n");
		wdt->prescale = 1U << (2 * div);
		wdt->wdd.timeout = x2000_wdt_inherited_seconds(
			wdt->rtc_rate, wdt->prescale, full);
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		/* Preserve counter/compare: watchdog core owns subsequent policy. */
	}

	platform_set_drvdata(pdev, wdt);
	watchdog_stop_ping_on_suspend(&wdt->wdd);

	/* No stop-on-reboot: an armed watchdog must cover a kexec handoff. */
	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;
	if (cmpxchg(&recovery_wdt, NULL, wdt))
		return -EBUSY;
	return devm_add_action_or_reset(dev, x2000_wdt_drop_recovery, wdt);
}

/* Restore an acknowledged normal state, including after a failed entry.
 * A failed parent restoration is reported even if a valid RTCLK deadline can
 * still be restored; do not silently expose an unarmed running watchdog.
 */
static int x2000_wdt_restore(struct x2000_wdt *wdt)
{
	int ret = 0, arm_ret = 0;

	WRITE_ONCE(wdt->ready_generation, 0);
	if (wdt->suspend_parent) {
		x2000_wdt_halt(&wdt->wdd);
		ret = clk_set_parent(wdt->rtc, wdt->suspend_parent);
		wdt->wdd.timeout = wdt->suspend_timeout;
	}
	if (wdt->suspend_running) {
		arm_ret = x2000_wdt_start(&wdt->wdd);
		if (!arm_ret)
			watchdog_set_last_hw_keepalive(&wdt->wdd, 0);
	}
	if (ret || arm_ret) {
		wdt->recovery_fault = ret ?: arm_ret;
		/* Keep restoration ownership for a bounded cancel/unwind retry. */
		return wdt->recovery_fault;
	}
	wdt->suspend_running = false;
	wdt->suspend_parent = NULL;
	wdt->recovery_generation = 0;
	return 0;
}

int x2000_wdt_recovery_request(u32 generation, unsigned int seconds)
{
	struct x2000_wdt *wdt = READ_ONCE(recovery_wdt);

	if (!wdt)
		return -ENODEV;
	if (!generation || seconds < 10 || seconds > 60)
		return -EINVAL;
	if (wdt->recovery_fault)
		return -EIO;
	if (wdt->queued_generation || wdt->recovery_generation)
		return -EBUSY;
	wdt->recovery_seconds = seconds;
	WRITE_ONCE(wdt->queued_generation, generation);
	return 0;
}
EXPORT_SYMBOL_GPL(x2000_wdt_recovery_request);

bool x2000_wdt_normal_sleep_ready(void)
{
	struct x2000_wdt *wdt = READ_ONCE(recovery_wdt);

	/* Called at platform entry, after ordinary device suspend stopped the
	 * timer. No sleeping clock operation and no implicit recovery request.
	 */
	if (!wdt || wdt->recovery_fault || wdt->queued_generation ||
	    wdt->recovery_generation || wdt->ready_generation || wdt->suspend_parent)
		return false;
	return !(readb(wdt->base + WDT_TCER) & 1);
}
EXPORT_SYMBOL_GPL(x2000_wdt_normal_sleep_ready);

bool x2000_wdt_recovery_ready(u32 generation)
{
	struct x2000_wdt *wdt = READ_ONCE(recovery_wdt);
	u32 control;

	if (!wdt || !generation || wdt->recovery_fault ||
	    smp_load_acquire(&wdt->ready_generation) != generation)
		return false;
	/* No sleeping clock API here: platform entry has IRQs disabled.
	 * The successful CCF transition and rate were checked before publish.
	 */
	control = readl(wdt->base + WDT_TCSR);
	return (readb(wdt->base + WDT_TCER) & 1) &&
	       !(readl(wdt->base + TCU_TSR) & WDT_STOP) &&
	       (control & 0x3f) == (WDT_RTC | WDT_DIV1024) &&
	       (readl(wdt->base + WDT_TDR) & 0xffff) ==
	       x2000_wdt_ticks(32768, wdt->recovery_seconds);
}
EXPORT_SYMBOL_GPL(x2000_wdt_recovery_ready);

int x2000_wdt_recovery_cancel(u32 generation)
{
	struct x2000_wdt *wdt = READ_ONCE(recovery_wdt);
	int ret = 0;

	if (!wdt || !generation)
		return 0;
	if (wdt->queued_generation == generation)
		WRITE_ONCE(wdt->queued_generation, 0);
	if (wdt->recovery_generation == generation) {
		/* A normal successful device resume already retired this state.
		 * Unexpected leftovers need restoration and a latched fault.
		 */
		if (!wdt->recovery_fault)
			wdt->recovery_fault = -EIO;
		ret = x2000_wdt_restore(wdt);
	}
	return ret ?: wdt->recovery_fault;
}
EXPORT_SYMBOL_GPL(x2000_wdt_recovery_cancel);

/* Default policy stops the watchdog while userspace is frozen. An attempt-bound
 * diagnostic request instead uses the independent32kHz source, which
 * remains available when the external oscillator is disabled by deep sleep.
 * CCF owns the mux transition before IRQs/DDR are unavailable. Platform sleep
 * sets the same selector and restores it before this device's resume callback.
 */
static int x2000_wdt_suspend(struct device *dev)
{
	struct x2000_wdt *wdt = dev_get_drvdata(dev);
	u32 generation = xchg(&wdt->queued_generation, 0);
	unsigned int seconds = generation ? wdt->recovery_seconds : 0;
	int ret, restore_ret;

	WRITE_ONCE(wdt->ready_generation, 0);
	wdt->recovery_generation = generation;
	wdt->suspend_running = !!(readb(wdt->base + WDT_TCER) & 1);
	wdt->suspend_timeout = wdt->wdd.timeout;
	if (!seconds) {
		if (wdt->suspend_running)
			x2000_wdt_halt(&wdt->wdd);
		return 0;
	}
	wdt->suspend_parent = clk_get_parent(wdt->rtc);
	if (IS_ERR_OR_NULL(wdt->suspend_parent)) {
		wdt->suspend_parent = NULL;
		return -ENODEV;
	}
	x2000_wdt_halt(&wdt->wdd);
	ret = clk_set_parent(wdt->rtc, wdt->sleep_rtc);
	if (!ret)
		ret = x2000_wdt_set_timeout(&wdt->wdd, seconds);
	if (!ret)
		ret = x2000_wdt_start(&wdt->wdd);
	if (!ret) {
		if (wdt->rtc_rate != 32768)
			ret = -EINVAL;
		else {
			smp_store_release(&wdt->ready_generation, generation);
			return 0;
		}
	}
	restore_ret = x2000_wdt_restore(wdt);
	if (restore_ret)
		dev_err(dev, "diagnostic watchdog unwind failed: %d\n", restore_ret);
	return ret;
}

static int x2000_wdt_resume(struct device *dev)
{
	return x2000_wdt_restore(dev_get_drvdata(dev));
}
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_wdt_pm_ops,
			      x2000_wdt_suspend, x2000_wdt_resume);

static const struct of_device_id x2000_wdt_of_match[] = {
	{ .compatible = "ingenic,x2000-watchdog" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_wdt_of_match);

static struct platform_driver x2000_wdt_driver = {
	.probe = x2000_wdt_probe,
	.driver = {
		.name = "ingenic-x2000-watchdog",
		.pm = pm_sleep_ptr(&x2000_wdt_pm_ops),
		.of_match_table = x2000_wdt_of_match,
	},
};
module_platform_driver(x2000_wdt_driver);

MODULE_DESCRIPTION("Ingenic X2000 TCU watchdog");
MODULE_LICENSE("GPL");
