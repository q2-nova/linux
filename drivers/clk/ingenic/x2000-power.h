/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef INGENIC_X2000_POWER_H
#define INGENIC_X2000_POWER_H
struct ingenic_cgu;
#if IS_ENABLED(CONFIG_INGENIC_X2000_POWER_DOMAINS)
void x2000_power_domains_setup(struct ingenic_cgu *cgu);
#else
static inline void x2000_power_domains_setup(struct ingenic_cgu *cgu) { }
#endif
#endif
