/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995 Waldorf Electronics
 * Written by Ralf Baechle and Andreas Busse
 * Copyright (C) 1995 - 1999 Ralf Baechle
 * Copyright (C) 1996 Paul M. Antoine
 * Modified for DECStation and hence R3000 support by Paul M. Antoine
 * Further modifications by David S. Miller and Harald Koerfgen
 * Copyright (C) 1999 Silicon Graphics, Inc.
 */

#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/addrspace.h>
#include <asm/cacheops.h>
#include <asm/mipsregs.h>

	LEAF(start)
	/* Save boot rom start args */
	move	s0, a0
	move	s1, a1
	move	s2, a2
	move	s3, a3

	/* Clear BSS */
	PTR_LA	a0, _edata
	PTR_LA	a2, _end
1:	PTR_S	zero, 0(a0)
	PTR_ADDIU a0, a0, PTRSIZE
	bne	a2, a0, 1b

	PTR_LA	a0, (.heap)	     /* heap address */
	PTR_LA	sp, (.stack + 8192)  /* stack address */

	PTR_LA	t9, decompress_kernel
	jalr	t9

2:
#ifdef CONFIG_INGENIC_X2000_SMP
	/* X2000: the decompressor has written the kernel through KSEG0.
	 * Execute maintenance uncached, with no stack accesses afterwards.
	 * Match the vendor soc-x2000/include/soc/cache.h sequence exactly:
	 * L1D, L1I, sync + uncached load, L2, sync + uncached load.
	 * SPL holds other CPUs in reset; boot arguments remain in s0..s3.
	 */
	.set push
	.set noreorder
	mfc0	t0, CP0_PRID
	li	t1, 0xffffff00
	and	t0, t0, t1
	li	t1, 0x00132000
	bne	t0, t1, 9f
	nop
	PTR_LA	t9, 4f
	li	t0, 0x20000000
	or	t9, t9, t0
	jr.hb	t9
	nop
4:	li	t0, CKSEG0
	li	t1, CKSEG0 + 32768
5:	cache	Index_Writeback_Inv_D, 0(t0)
	addiu	t0, t0, 32
	bne	t0, t1, 5b
	nop
	li	t0, CKSEG0
6:	cache	Index_Invalidate_I, 0(t0)
	addiu	t0, t0, 32
	bne	t0, t1, 6b
	nop
	li	t2, CKSEG1
	sync
	lw	zero, 0(t2)
	li	t0, CKSEG0
	li	t1, CKSEG0 + (512 * 1024)
7:	cache	Index_Writeback_Inv_SD, 0(t0)
	addiu	t0, t0, 64
	bne	t0, t1, 7b
	nop
	sync
	lw	zero, 0(t2)
9:
	.set pop
#endif
	move	a0, s0
	move	a1, s1
	move	a2, s2
	move	a3, s3
	PTR_LI	t9, KERNEL_ENTRY
#ifdef CONFIG_INGENIC_X2000_SMP
	jalr.hb t9
#else
	jalr	t9
#endif

3:
	b	3b

	END(start)

	.comm .heap,BOOT_HEAP_SIZE,4
	.comm .stack,4096*2,4
