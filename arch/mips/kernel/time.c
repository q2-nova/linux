// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 * Copyright (c) 2003, 2004  Maciej W. Rozycki
 *
 * Common time service routines for MIPS machines.
 */
#include <linux/bug.h>
#include <linux/clockchips.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/export.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <asm/cpu-features.h>
#include <asm/cpu-type.h>
#include <asm/div64.h>
#include <asm/time.h>

#ifdef CONFIG_CPU_FREQ

static DEFINE_PER_CPU(unsigned long, pcp_lpj_ref);
static DEFINE_PER_CPU(unsigned long, pcp_lpj_ref_freq);
static unsigned long glb_lpj_ref;
static unsigned long glb_lpj_ref_freq;

static int cpufreq_callback(struct notifier_block *nb,
			    unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpumask *cpus = freq->policy->cpus;
	unsigned long rate, actual, lpj;
	int cpu;

	if (freq->flags & CPUFREQ_CONST_LOOPS)
		return NOTIFY_OK;

	if (!glb_lpj_ref) {
		glb_lpj_ref = cpu_data[freq->policy->cpu].udelay_val;
		glb_lpj_ref_freq = freq->old;
	}

	/* A CPU may first participate after earlier transitions while offline. */
	for_each_cpu(cpu, cpus) {
		if (!per_cpu(pcp_lpj_ref_freq, cpu)) {
			per_cpu(pcp_lpj_ref, cpu) = cpu_data[cpu].udelay_val;
			per_cpu(pcp_lpj_ref_freq, cpu) = freq->old;
		}
	}

	/* Never shorten delays before an increase or an uncompleted decrease. */
	if (val == CPUFREQ_PRECHANGE) {
		rate = max(freq->old, freq->new);
	} else if (val == CPUFREQ_POSTCHANGE) {
		rate = freq->new;
		if (freq->policy->clk) {
			/*
			 * Failed transitions also emit the requested POST before the
			 * rollback notifications. Use the achieved hardware rate so
			 * a failed decrease cannot briefly under-calibrate delays.
			 */
			actual = clk_get_rate(freq->policy->clk) / 1000;
			if (WARN_ON_ONCE(!actual))
				rate = max(freq->old, freq->new);
			else
				rate = actual;
		}
	} else {
		return NOTIFY_OK;
	}

	loops_per_jiffy = cpufreq_scale(glb_lpj_ref, glb_lpj_ref_freq, rate);
	for_each_cpu(cpu, cpus) {
		lpj = cpufreq_scale(per_cpu(pcp_lpj_ref, cpu),
				    per_cpu(pcp_lpj_ref_freq, cpu), rate);
		cpu_data[cpu].udelay_val = (unsigned int)lpj;
	}
	return NOTIFY_OK;
}

static struct notifier_block cpufreq_notifier = {
	.notifier_call  = cpufreq_callback,
};

static int __init register_cpufreq_notifier(void)
{
	return cpufreq_register_notifier(&cpufreq_notifier,
					 CPUFREQ_TRANSITION_NOTIFIER);
}
core_initcall(register_cpufreq_notifier);

#endif /* CONFIG_CPU_FREQ */

/*
 * forward reference
 */
DEFINE_SPINLOCK(rtc_lock);
EXPORT_SYMBOL(rtc_lock);

static int null_perf_irq(void)
{
	return 0;
}

int (*perf_irq)(void) = null_perf_irq;

EXPORT_SYMBOL(perf_irq);

/*
 * time_init() - it does the following things.
 *
 * 1) plat_time_init() -
 *	a) (optional) set up RTC routines,
 *	b) (optional) calibrate and set the mips_hpt_frequency
 *	    (only needed if you intended to use cpu counter as timer interrupt
 *	     source)
 * 2) calculate a couple of cached variables for later usage
 */

unsigned int mips_hpt_frequency;
EXPORT_SYMBOL_GPL(mips_hpt_frequency);

static __init int cpu_has_mfc0_count_bug(void)
{
	switch (current_cpu_type()) {
	case CPU_R4000PC:
	case CPU_R4000SC:
	case CPU_R4000MC:
		/*
		 * V3.0 is documented as suffering from the mfc0 from count bug.
		 * Afaik this is the last version of the R4000.	 Later versions
		 * were marketed as R4400.
		 */
		return 1;

	case CPU_R4400PC:
	case CPU_R4400SC:
	case CPU_R4400MC:
		/*
		 * The published errata for the R4400 up to 3.0 say the CPU
		 * has the mfc0 from count bug.  This seems the last version
		 * produced.
		 */
		return 1;
	}

	return 0;
}

void __init time_init(void)
{
	plat_time_init();

	/*
	 * The use of the R4k timer as a clock event takes precedence;
	 * if reading the Count register might interfere with the timer
	 * interrupt, then we don't use the timer as a clock source.
	 * We may still use the timer as a clock source though if the
	 * timer interrupt isn't reliable; the interference doesn't
	 * matter then, because we don't use the interrupt.
	 */
	if (mips_clockevent_init() != 0 || !cpu_has_mfc0_count_bug())
		init_mips_clocksource();
}
