// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSA in kernel code: kernel_msa_begin() / kernel_msa_end().
 *
 * Model: x86's kernel_fpu_begin() and arm64's kernel_neon_begin(). A
 * section runs with bottom halves and preemption disabled, so no context
 * switch, migration or softirq can happen inside it. The current task's
 * live FP/MSA context is first written back to its thread struct by
 * lose_fpu_inatomic(1, current), the call switch_to() makes when a task is
 * switched out, so the task finds its context exactly as after a context
 * switch: its next FP or MSA instruction traps and
 * enable_restore_fp_context() reloads it. A section then sets
 * Status.CU1 and Status.FR (MSA requires the 64-bit FP register mode) and
 * Config5.MSAEn; kernel_msa_end() clears MSAEn and CU1 and restores FR.
 * On return to user mode the exception return keeps the live CU1 (clear),
 * so user FP state is never exposed to or clobbered by kernel vectors.
 *
 * Not usable (returns false, the caller takes its scalar path) without
 * MSA, in hard or soft interrupt or NMI context, with interrupts disabled,
 * or inside another section on this CPU.
 */
#include <linux/bottom_half.h>
#include <linux/export.h>
#include <linux/hardirq.h>
#include <linux/irqflags.h>
#include <linux/percpu.h>
#include <linux/preempt.h>

#include <asm/cpu-features.h>
#include <asm/fpu.h>
#include <asm/kernel-msa.h>
#include <asm/mipsregs.h>
#include <asm/msa.h>

static DEFINE_PER_CPU(bool, kernel_msa_active);
static DEFINE_PER_CPU(unsigned int, kernel_msa_status_fr);

bool kernel_msa_begin(void)
{
	if (!cpu_has_msa || !in_task() || irqs_disabled())
		return false;

	local_bh_disable();
	preempt_disable();
	if (this_cpu_read(kernel_msa_active)) {
		preempt_enable();
		local_bh_enable();
		return false;
	}

	/* write the task's FP/MSA context back, as a context switch does */
	lose_fpu_inatomic(1, current);

	this_cpu_write(kernel_msa_status_fr, read_c0_status() & ST0_FR);
	change_c0_status(ST0_CU1 | ST0_FR, ST0_CU1 | ST0_FR);
	enable_fpu_hazard();
	if (!(read_c0_status() & ST0_FR)) {
		/* FR=1 refused: no MSA */
		change_c0_status(ST0_CU1 | ST0_FR, this_cpu_read(kernel_msa_status_fr));
		disable_fpu_hazard();
		preempt_enable();
		local_bh_enable();
		return false;
	}
	if (cpu_has_fre)
		clear_c0_config5(MIPS_CONF5_FRE);
	enable_msa();
	this_cpu_write(kernel_msa_active, true);
	return true;
}
EXPORT_SYMBOL_GPL(kernel_msa_begin);

void kernel_msa_end(void)
{
	WARN_ON_ONCE(!this_cpu_read(kernel_msa_active));
	disable_msa();
	change_c0_status(ST0_CU1 | ST0_FR, this_cpu_read(kernel_msa_status_fr));
	disable_fpu_hazard();
	this_cpu_write(kernel_msa_active, false);
	preempt_enable();
	local_bh_enable();
}
EXPORT_SYMBOL_GPL(kernel_msa_end);
