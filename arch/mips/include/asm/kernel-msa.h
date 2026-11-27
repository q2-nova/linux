/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MSA (MIPS SIMD) in kernel code.
 *
 *	if (kernel_msa_begin()) {
 *		... MSA instructions, no sleeping ...
 *		kernel_msa_end();
 *	} else {
 *		... scalar fallback ...
 *	}
 *
 * kernel_msa_begin() returns false, and the caller must not use MSA, when
 * the CPU has no MSA, in hard or soft interrupt context, with interrupts
 * disabled, or inside another kernel MSA section. Otherwise it disables
 * bottom halves and preemption, saves the current task's live FP/MSA
 * context to its thread struct (the task reloads it on its next FP or MSA
 * instruction, as after a context switch), and enables MSA with 64-bit FP
 * registers. kernel_msa_end() disables MSA and the FPU again and re-enables
 * preemption and bottom halves. The vector registers' contents are not
 * preserved across a section.
 */
#ifndef _ASM_KERNEL_MSA_H
#define _ASM_KERNEL_MSA_H

#include <linux/types.h>

#ifdef CONFIG_CPU_HAS_MSA
bool kernel_msa_begin(void);
void kernel_msa_end(void);
#else
static inline bool kernel_msa_begin(void)
{
	return false;
}

static inline void kernel_msa_end(void)
{
}
#endif

#endif /* _ASM_KERNEL_MSA_H */
