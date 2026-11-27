/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INGENIC_X2000_WATCHDOG_H
#define _LINUX_INGENIC_X2000_WATCHDOG_H
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>
/* Called under system-sleep serialization; request/cancel may sleep,
 * ready is a non-sleeping entry check. Generation zero grants nothing.
 */
#if IS_REACHABLE(CONFIG_INGENIC_X2000_WDT)
bool x2000_wdt_normal_sleep_ready(void);
int x2000_wdt_recovery_request(u32 generation, unsigned int seconds);
bool x2000_wdt_recovery_ready(u32 generation);
int x2000_wdt_recovery_cancel(u32 generation);
#else
static inline bool x2000_wdt_normal_sleep_ready(void) { return false; }
static inline int x2000_wdt_recovery_request(u32 generation, unsigned int seconds)
{ return -ENODEV; }
static inline bool x2000_wdt_recovery_ready(u32 generation) { return false; }
static inline int x2000_wdt_recovery_cancel(u32 generation) { return 0; }
#endif
#endif
