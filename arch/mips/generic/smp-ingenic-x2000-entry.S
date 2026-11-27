/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Secondary core entry for the Ingenic X2000. The CCU starts a released
 * core here (through KSEG1, uncached); it sets up CP0, loads the stack and
 * thread pointers left by x2000_boot_secondary() and enters the kernel.
 *
 * Based on the Ingenic XBurst SMP entry by Paul Burton and Zhou Yanjie.
 * Copyright (c) 2013 Paul Burton <paul.burton@imgtec.com>
 * Copyright (c) 2021 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 */

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>

	.text
	.balign	4096
	.set	noreorder
LEAF(x2000_secondary_entry)
	mtc0	zero, CP0_CAUSE
	li	t0, ST0_CU0
	mtc0	t0, CP0_STATUS

	/* KSEG0 cacheable, non-coherent */
	mfc0	t0, CP0_CONFIG
	li	t1, ~CONF_CM_CMASK
	and	t0, t0, t1
	ori	t0, t0, CONF_CM_CACHABLE_NONCOHERENT
	mtc0	t0, CP0_CONFIG
	mtc0	zero, CP0_PAGEMASK
	ehb

	PTR_LA	t0, x2000_secondary_sp
	lw	sp, 0(t0)
	PTR_LA	t0, x2000_secondary_gp
	lw	gp, 0(t0)

	PTR_LA	t0, smp_bootstrap
	jr	t0
	 nop
	END(x2000_secondary_entry)
