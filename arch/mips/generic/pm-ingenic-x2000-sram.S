/* SPDX-License-Identifier: GPL-2.0-or-later */
/* X2000 retained-SRAM suspend, based on Ingenic's GPL X2000 pm_sleep.c
	* and the Q2 stock kernel. All blob branches are PC-relative; no stack or
	* DRAM access is permitted between DDR self-refresh and its exit.
	*/
#include <asm/asm.h>
#include <asm/cacheops.h>
#include <asm/errno.h>
#include <asm/regdef.h>

#define CTX 0xb2407000
#define CTRL 152
#define DLP 156
#define AUTOSR 160
#define CPCCR 164
#define STAGE 148
#define ERROR 144
#define PROOF_MAGIC 168
#define PROOF_DDR 176
#define PROOF_PHY 180
#define PROOF_STATUS 184
#define PROOF_RTC_ENTER 188
#define PROOF_RTC_EXIT 192
#define JOURNAL_BASE 200
#define JOURNAL_ACTIVE 204
#define ROUNDTRIP 208
#define CANARY_POINTER 212
#define CANARY_SEED 216
#define CPU_ONLY 220
#define RESET_RETURN 236
#define SAVED_EPC 240
#define EPC_RESTORE 244
#define JOURNAL_LAST 0xb2407f0c

 /* No stack/DRAM access; t4/t6/t7/t8 and peripheral bases are preserved. */
	.macro mark_stage value
	li t1, \value
	sw t1, STAGE(k0)
	lw t2, JOURNAL_ACTIVE(k0)
	beqz t2, .Lstage_done_\@
	 nop
	lw t3, JOURNAL_BASE(k0)
	xor t3, t3, t1
	li t5, JOURNAL_LAST
	sw t3, 0(t5)
	sync
.Lstage_done_\@:
	.endm


	/* Distinct codes prevent a lost CPU mode from masquerading as a later
	 * trace checkpoint: legacy0..7 remains legacy, trace16..23 is version1.
	 * Uses only the same caller-clobbered temporaries as mark_stage.
	 */
	.macro mark_phase legacy, trace
	lw t1, CPU_ONLY(k0)
	li t2, 2
	bne t1, t2, .Llegacy_phase_\@
	 nop
	mark_stage \trace
	b .Lphase_done_\@
	 nop
.Llegacy_phase_\@:
	mark_stage \legacy
.Lphase_done_\@:
	.endm

	.macro mark_trace value
	lw t1, CPU_ONLY(k0)
	li t2, 2
	bne t1, t2, .Ltrace_done_\@
	 nop
	mark_stage \value
.Ltrace_done_\@:
	.endm


	.macro poll reg, base, mask, expected
	sync
	li t8, 0xffffff
.Lpoll_\@:	lw t1, \reg(\base)
	li t2, \mask
	and t1, t1, t2
	li t2, \expected
	beq t1, t2, .Ldone_\@
	 addiu t8, t8, -1
	bnez t8, .Lpoll_\@
	 nop
	b x2000_sram_failure
	 nop
.Ldone_\@:
	sync
	.endm

/* RTC writes require WRDY, WENR magic, WEN and a second WRDY. */
	.macro rtc_write value
	li t0, 0xb0003000
	poll 0, t0, 0x80, 0x80
	li t1, 0xa55a
	sw t1, 0x3c(t0)
	poll 0x3c, t0, 0x80000000, 0x80000000
	poll 0, t0, 0x80, 0x80
	sw \value, 0x48(t0)
	poll 0, t0, 0x80, 0x80
	.endm

/* The context is SRAM too. Save only registers writable on XBurst2; Config1
	* is read-only. Include EntryHi/Wired, which stock's older helper omitted.
	*/
	.macro cp_save reg, sel, off
	mfc0 k1, $\reg, \sel
	sw k1, \off(k0)
	.endm
	.macro cp_restore reg, sel, off
	lw k1, \off(k0)
	mtc0 k1, $\reg, \sel
	.endm

	.set push
	.set noreorder
	.set noat
	.set mips32r2
	.balign 32
LEAF(x2000_sram_start)
.Lresume:
	/* Reset entry and fall-through WAIT both use this same return path. */
	li k0, CTX
	/* Prove SRAM execution before any potentially stalled peripheral read. */
	mark_phase 6, 17
	li t0, 0xb2200f00
	li t1, 0xbfc00000
	sw t1, 0(t0)
	mfc0 t1, $12
	sw t1, PROOF_STATUS(k0)
	li t0, 0xb0003000
	lw t1, 4(t0)
	sw t1, PROOF_RTC_EXIT(k0)
	lw t1, CPU_ONLY(k0)
	bnez t1, .Lcheck_canary
	 nop
	li t7, 0xb3011000
	lw t1, 0x84(t7)
	li t2, -3
	and t1, t1, t2
	sw t1, 0x84(t7)
	poll 0xc8, t7, 8, 8
	li t6, 0xb3012000
	lw t1, 0(t6)
	li t2, -9
	and t1, t1, t2
	sw t1, 0(t6)
	poll 4, t6, 1, 1
	li t0, 0xb0003000
	poll 0, t0, 0x80, 0x80
	lw t4, 0x48(t0)
	li t2, 0x200000
	or t4, t4, t2
	rtc_write t4
	li t6, 0xb34f0000
	lw t1, 0x10(t6)
	li t2, -33
	and t1, t1, t2
	sw t1, 0x10(t6)
	poll 0, t6, 4, 0
	/* Stock waits 1000 loop iterations before PHY auto calibration. */
	li t8, 1000
1:	addiu t8, t8, -1
	bnez t8, 1b
	 nop
	lw t1, 8(t7)
	li t2, -3
	and t1, t1, t2
	ori t1, t1, 1
	sw t1, 8(t7)
	poll 0xcc, t7, 3, 3
	sw zero, 8(t7)
	lw t1, AUTOSR(k0)
	sw t1, 0x28(t6)
	lw t1, DLP(k0)
	sw t1, 0x20(t6)
	lw t1, CTRL(k0)
	sw t1, 0x10(t6)
	sync
	/* Acknowledged register bits do not alone prove usable retained RAM.
	 * This owned16-word canary is deliberately read through uncached KSEG1.
	 */
.Lcheck_canary:
	lw t0, CANARY_POINTER(k0)
	lw t4, CANARY_SEED(k0)
	move t8, zero
.Lcanary:
	lw t1, 0(t0)
	xor t2, t4, t8
	bne t1, t2, .Lcanary_failure
	 addiu t0, t0, 4
	addiu t8, t8, 1
	sltiu t1, t8, 16
	bnez t1, .Lcanary
	 nop
	mark_phase 7, 18
	/* Reset has no previous TLB entries. KSEG1 SRAM remains accessible
	 * throughout; restore caches, exception base and context before DRAM.
	 */
	cp_restore 5, 0, 48
	cp_restore 12, 1, 56
	cp_restore 13, 0, 60
	cp_restore 15, 1, 64
	cp_restore 16, 0, 68
	cp_restore 16, 2, 72
	cp_restore 16, 3, 76
	cp_restore 16, 4, 80
	cp_restore 16, 5, 84
	cp_restore 16, 7, 88
	cp_restore 17, 0, 92
	cp_restore 18, 0, 96
	cp_restore 19, 0, 100
	cp_restore 26, 0, 104
	cp_restore 4, 0, 108
	cp_restore 31, 2, 112
	cp_restore 31, 3, 116
	cp_restore 31, 4, 120
	cp_restore 31, 5, 124
	cp_restore 31, 6, 128
	cp_restore 31, 7, 132
	cp_restore 10, 0, 136
	cp_restore 6, 0, 140
	/* Selected context repair preserves the actual pre-sleep EPC. The
	 * control and ERL-return treatment retain their original behavior.
	 */
EXPORT(x2000_sram_epc_restore)
	lw t0, EPC_RESTORE(k0)
	beqz t0, .Lepc_restore_done
	 nop
	cp_restore 14, 0, SAVED_EPC
	ehb
.Lepc_restore_done:
	mark_trace 19
	lw s0, 0(k0)
	lw s1, 4(k0)
	lw s2, 8(k0)
	lw s3, 12(k0)
	lw s4, 16(k0)
	lw s5, 20(k0)
	lw s6, 24(k0)
	lw s7, 28(k0)
	lw gp, 32(k0)
	lw sp, 36(k0)
	lw fp, 40(k0)
	lw ra, 44(k0)
	mark_trace 20
EXPORT(x2000_sram_return)
	/* Mode0 preserves the original JR.HB return. Selected mode3 keeps
	 * ERL until the architectural reset return, leaving EPC untouched.
	 * C rejects saved IE/EXL/ERL and rewrites the choice for every entry.
	 */
	lw t0, RESET_RETURN(k0)
	lw k1, 52(k0)
	beqz t0, .Lreturn_status
	 nop
	mtc0 ra, $30, 0
	ori k1, k1, 4
.Lreturn_status:
	mtc0 k1, $12, 0
	ehb
	mark_trace 21
	lw v0, ERROR(k0)
	beqz t0, .Lreturn_jump
	 nop
	eret
	nop
.Lreturn_jump:
	jr.hb ra
	 nop

EXPORT(x2000_sram_sleep)
	li k0, CTX
	/* Final drain after context save and after all C/cache-wrapper stores.
	 * Geometry is admitted in C: D=128 sets x8 ways x32, L2=512 x16 x64.
	 * Linear index addresses enumerate every set and way, as blast_*cache.
	 * No stack/DRAM writes occur between this drain and DDR restoration.
	 */
	li t0, 0x80000000
	li t1, 0x80008000
.Ldrain_d:
	cache Index_Writeback_Inv_D, 0(t0)
	addiu t0, t0, 32
	bne t0, t1, .Ldrain_d
	 nop
	sync
	li t0, 0x80000000
	li t1, 0x80080000
.Ldrain_l2:
	cache Index_Writeback_Inv_SD, 0(t0)
	addiu t0, t0, 64
	bne t0, t1, .Ldrain_l2
	 nop
	sync
	li t6, 0xb0000000
	lw t1, 0(t6)
	sw t1, CPCCR(k0)
	/* CPU frequency must match L2 frequency before CPU power-down. */
	li t2, 0xffbffff0
	and t3, t1, t2
	srl t2, t1, 4
	andi t2, t2, 15
	or t3, t3, t2
	li t2, 0xffbfffff
	and t1, t1, t2
	sw t1, 0(t6)
	sw t3, 0(t6)
	li t2, 0x400000
	or t3, t3, t2
	sw t3, 0(t6)
	poll 0xd4, t6, 1, 0
	mark_phase 1, 16
	lw t1, CPU_ONLY(k0)
	bnez t1, .Lwait_witness
	 nop
	li t6, 0xb34f0000
	lw t1, 0x10(t6)
	sw t1, CTRL(k0)
	lw t2, 0x20(t6)
	sw t2, DLP(k0)
	lw t2, 0x28(t6)
	sw t2, AUTOSR(k0)
	li t2, 0xffff8fbf
	and t1, t1, t2
	sw t1, 0x10(t6)
	sw zero, 0x20(t6)
	sw zero, 0x28(t6)
	sync
	li t0, 0xa0000000
	lw zero, 0(t0)
	/* No DRAM access beyond this point until DDR is restored. */
	ori t1, t1, 0x20
	sw t1, 0x10(t6)
	poll 0, t6, 4, 4
	mark_stage 2
	li t0, 0xb0003000
	poll 0, t0, 0x80, 0x80
	lw t4, 0x48(t0)
	li t2, 0xffdfffff
	and t4, t4, t2
	rtc_write t4
	li t6, 0xb3012000
	lw t1, 0(t6)
	ori t1, t1, 8
	sw t1, 0(t6)
	.rept 32
	ssnop
	.endr
	mark_stage 3
	li t7, 0xb3011000
	lw t1, 0x84(t7)
	ori t1, t1, 2
	sw t1, 0x84(t7)
	sync
	mark_stage 4
.Lwait_witness:
	/* Retained numeric evidence for the actual physical transition. No
	 * DRAM or stack access: DDR acknowledgement, disabled PLL and RTC edge.
	 */
	li t0, 0xb34f0000
	lw t1, 0(t0)
	sw t1, PROOF_DDR(k0)
	li t7, 0xb3011000
	lw t1, 0x84(t7)
	sw t1, PROOF_PHY(k0)
	li t0, 0xb0003000
	lw t1, 4(t0)
	sw t1, PROOF_RTC_ENTER(k0)
	li t1, 0x51325350
	sw t1, PROOF_MAGIC(k0)
	sync
	mark_phase 5, 16
	/* Same full DDR transition; only the explicit owned mode bypasses WAIT. */
	lw t1, ROUNDTRIP(k0)
	bnez t1, .Lroundtrip_dwell
	 nop
	wait
	nop
	nop
	nop
	b .Lresume
	 nop

.Lroundtrip_dwell:
	/* An immediate return need not let the PHY PLL settle off. Cross two
	 * RTC second boundaries with DDR/PHY still unavailable, while executing
	 * only from SRAM. Unsigned subtraction handles the seconds rollover.
	 * No WAIT, stack or DRAM access; the admitted watchdog remains the
	 * independent deadline if RTC progress or a peripheral read stalls.
	 */
	li t0, 0xb0003000
	lw t3, PROOF_RTC_ENTER(k0)
	li t8, -1
.Lroundtrip_dwell_poll:
	lw t1, 4(t0)
	subu t2, t1, t3
	sltiu t2, t2, 2
	beqz t2, .Lresume
	 addiu t8, t8, -1
	bnez t8, .Lroundtrip_dwell_poll
	 nop
	b x2000_sram_failure
	 nop

.Lcanary_failure:
	li t1, -EIO
	b .Lrecord_failure
	 nop
x2000_sram_failure:
	/* DRAM/PLL failure cannot safely unwind into C. Keep the last stage
	 * in retained SRAM and reset through the known RTC watchdog protocol.
	 * Never report a failed training/retention path as resumed.
	 */
	li k0, CTX
	li t1, -ETIMEDOUT
.Lrecord_failure:
	li k0, CTX
	sw t1, ERROR(k0)
	lw t2, JOURNAL_ACTIVE(k0)
	beqz t2, .Lfailure_journal_done
	 nop
	lw t1, STAGE(k0)
	ori t1, t1, 0x80
	lw t3, JOURNAL_BASE(k0)
	xor t3, t3, t1
	li t5, JOURNAL_LAST
	sw t3, 0(t5)
	sync
.Lfailure_journal_done:
	li t0, 0xb0000000
	lw t1, 0x20(t0)
	li t2, 0xfffbffff
	and t1, t1, t2
	sw t1, 0x20(t0)
	li t0, 0xb0002000
	sb zero, 4(t0)
	li t1, 32
	sw t1, 0(t0)
	li t1, 0x42a
	sh t1, 0x0c(t0)
	li t1, 0x10000
	sw t1, 0x3c(t0)
	li t1, 1
	sb t1, 4(t0)
	sync
	/* A poll/canary failure must not turn a no-WAIT test into deep sleep. */
	lw t1, ROUNDTRIP(k0)
	bnez t1, .Lfailure_spin
	 nop
	lw t1, CPU_ONLY(k0)
	bnez t1, .Lfailure_spin
	 nop
1:	wait
	b 1b
	 nop
.Lfailure_spin:
	b .Lfailure_spin
	 nop
EXPORT(x2000_sram_end)
	END(x2000_sram_start)

/* Called with CPU1 offline and after the generic primary-cache flush.
	* This leaf writes SRAM only; it cannot dirty a post-flush kernel stack.
	*/
LEAF(x2000_suspend_save)
	li k0, CTX
	sw s0, 0(k0)
	sw s1, 4(k0)
	sw s2, 8(k0)
	sw s3, 12(k0)
	sw s4, 16(k0)
	sw s5, 20(k0)
	sw s6, 24(k0)
	sw s7, 28(k0)
	sw gp, 32(k0)
	sw sp, 36(k0)
	sw fp, 40(k0)
	sw ra, 44(k0)
	cp_save 5, 0, 48
	cp_save 12, 0, 52
	cp_save 12, 1, 56
	cp_save 13, 0, 60
	cp_save 15, 1, 64
	cp_save 16, 0, 68
	cp_save 16, 2, 72
	cp_save 16, 3, 76
	cp_save 16, 4, 80
	cp_save 16, 5, 84
	cp_save 16, 7, 88
	cp_save 17, 0, 92
	cp_save 18, 0, 96
	cp_save 19, 0, 100
	cp_save 26, 0, 104
	cp_save 4, 0, 108
	cp_save 31, 2, 112
	cp_save 31, 3, 116
	cp_save 31, 4, 120
	cp_save 31, 5, 124
	cp_save 31, 6, 128
	cp_save 31, 7, 132
	cp_save 10, 0, 136
	cp_save 6, 0, 140
	cp_save 14, 0, SAVED_EPC
	sw zero, ERROR(k0)
	sw zero, STAGE(k0)
	sw zero, PROOF_MAGIC(k0)
	jr.hb a0
	 nop
	END(x2000_suspend_save)
/* Diagnostic matrix: bit0 selects EXL ERET versus ordinary transfer;
 * bit1 selects keep EPC versus restoring its original value. Select distinct
 * continuation labels BEFORE transfer so the restore target still begins
 * immediately with MTC0, without inserting a post-ERET decision/delay.
 */
LEAF(x2000_epc_probe)
	mfc0 t0, $14, 0
	mfc0 t1, $12, 0
	andi t4, a0, 2
	bnez t4, .Lchoose_keep
	 nop
	la t2, q2_pm_exl_resume
	b .Lprobe_target
	 nop
.Lchoose_keep:
	la t2, q2_pm_exl_keep
.Lprobe_target:
	/* Preserve the old write interval. New contrasts are selected before
	 * either CP0 write, using the same final continuation in all keep arms.
	 */
	andi t4, a0, 12
	bnez t4, .Lordered_return
	 nop
	mtc0 t2, $14, 0
	andi t4, a0, 1
	beqz t4, .Lepc_pulse
	 nop
	ori t3, t1, 2
	mtc0 t3, $12, 0
	ehb
	eret
	nop
.Lordered_return:
	ori t3, t1, 2
	andi t4, a0, 4
	bnez t4, .Lstatus_first
	 nop
	/* Equal two-barrier control: EPC first, then resolved Status. */
	mtc0 t2, $14, 0
	ehb
	mtc0 t3, $12, 0
	ehb
	eret
	nop
.Lstatus_first:
	/* Complete software EXL cycle, separate from hardware trap entry. */
	andi t4, a0, 32
	beqz t4, .Lgeneric_mask
	 nop
	mtc0 t3, $12, 0
	ehb
	mtc0 t1, $12, 0
	ehb
.Lgeneric_mask:
	/* Optional generic-mask write, separate from the basic order arm. */
	andi t4, a0, 16
	beqz t4, .Lset_status
	 nop
	ori t5, t1, 31
	xori t5, t5, 31
	mtc0 t5, $12, 0
	ehb
.Lset_status:
	mtc0 t3, $12, 0
	ehb
	mtc0 t2, $14, 0
	ehb
	eret
	nop
.Lepc_pulse:
	ehb
	jr t2
	 nop
EXPORT(q2_pm_exl_resume)
	mtc0 t0, $14, 0
	ehb
	jr.hb ra
	 nop
EXPORT(q2_pm_exl_keep)
	jr.hb ra
	 nop
	END(x2000_epc_probe)

/* The owned early trap fixup returns directly to q2_pm_exl_keep using the
 * original call RA. The ordinary trap handler is never our return mechanism. */
LEAF(q2_pm_precise_trap)
EXPORT(q2_pm_trap_fault)
	teq zero, zero
	nop
	jr.hb ra
	 nop
	END(q2_pm_precise_trap)
	.set pop
