/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_Q2_PM_CHECKPOINT_H
#define _LINUX_Q2_PM_CHECKPOINT_H
/* Inactive diagnostic hook. Other platforms retain the weak no-op. */
struct pt_regs;
#ifdef CONFIG_SUSPEND
void q2_pm_checkpoint(unsigned int phase, unsigned long callback);
void q2_pm_mmu_probe(unsigned long node,unsigned long ops);
void q2_pm_mmu_fault(const struct pt_regs *regs,unsigned long address,unsigned int kind);
void q2_pm_mmu_fault_origin(const struct pt_regs *regs,unsigned long address,unsigned int kind,unsigned int origin);
#else
static inline void q2_pm_checkpoint(unsigned int phase, unsigned long callback) {}
static inline void q2_pm_mmu_probe(unsigned long node,unsigned long ops) {}
static inline void q2_pm_mmu_fault(const struct pt_regs *regs,unsigned long address,unsigned int kind) {}
static inline void q2_pm_mmu_fault_origin(const struct pt_regs *regs,unsigned long address,unsigned int kind,unsigned int origin) {}
#endif
#endif
