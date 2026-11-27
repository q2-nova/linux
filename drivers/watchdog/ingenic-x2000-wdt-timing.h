/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef INGENIC_X2000_WDT_TIMING_H
#define INGENIC_X2000_WDT_TIMING_H
#ifdef __KERNEL__
#include <linux/math64.h>
#endif
static inline unsigned int x2000_wdt_div(unsigned long long value, unsigned int divisor)
{
#ifdef __KERNEL__
	return div_u64(value, divisor);
#else
	return value / divisor;
#endif
}
static inline int x2000_wdt_clock_valid(unsigned int hz)
{
	return hz == 32768 || hz == 46875;
}
static inline int x2000_wdt_prescale_valid(unsigned int divisor)
{
	return divisor == 1 || divisor == 4 || divisor == 16 || divisor == 64 ||
								divisor == 256 || divisor == 1024;
}
static inline unsigned int x2000_wdt_max_seconds(unsigned int hz)
{
	return x2000_wdt_clock_valid(hz) ? 65535U * 1024U / hz : 0;
}
/* Zero refuses invalid/unrepresentable requests before changing hardware. */
static inline unsigned int x2000_wdt_ticks(unsigned int hz, unsigned int seconds)
{
	if (!seconds || seconds > x2000_wdt_max_seconds(hz))
		return 0;
	return x2000_wdt_div((unsigned long long)hz * seconds + 1023, 1024);
}
static inline unsigned int x2000_wdt_seconds_left(unsigned int hz, unsigned int divisor,
								unsigned int full, unsigned int count)
{
	if (!x2000_wdt_clock_valid(hz) || !x2000_wdt_prescale_valid(divisor) ||
					full > 65535 || count >= full) return 0;
	return x2000_wdt_div((unsigned long long)(full-count)*divisor + hz-1, hz);
}
/* Recover nominal whole seconds from an inherited, quantized compare. */
static inline unsigned int x2000_wdt_inherited_seconds(unsigned int hz,
								unsigned int divisor, unsigned int full)
{
	unsigned int seconds;
	if (!x2000_wdt_clock_valid(hz) || !x2000_wdt_prescale_valid(divisor) ||
					!full || full > 65535) return 0;
	seconds = x2000_wdt_div((unsigned long long)full*divisor + hz/2, hz);
	if (seconds > x2000_wdt_max_seconds(hz))
		seconds = x2000_wdt_max_seconds(hz);
	return seconds ? seconds : 1;
}
#endif
