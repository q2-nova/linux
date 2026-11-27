// SPDX-License-Identifier: GPL-2.0
/*
 * Shanling Q2 (Ingenic X2000): checks of the non-coherent DMA cache
 * maintenance, in debugfs under x2000_dma/. Nothing runs until a file is
 * read or written.
 *
 * probe (read): the CCU CPU and memory-subsystem registers, the CPU
 *   topology the kernel uses for cache IPIs, and cache operation probes on
 *   a page of its own. For each trial a word is first set in memory through
 *   KSEG1, then a cached store leaves a dirty L1 line holding another value;
 *   after one cache operation a cached load (or, for l2_wb, a KSEG1 load)
 *   tells what the operation did to the dirty line:
 *     self_inv    Hit_Invalidate_D on this CPU (control: discarded);
 *     remote_inv  Hit_Invalidate_D on the other CPU (discarded: the CPUs
 *                 share the L1 or the operation reaches both; kept: each
 *                 CPU has its own L1);
 *     l2_inv      Hit_Invalidate_SD on this CPU (discarded: an L2
 *                 invalidate also removes the L1 copy);
 *     l2_wb       Hit_Writeback_Inv_SD on this CPU (written: an L2
 *                 write-back also takes the dirty L1 data to memory).
 *   "evicted" counts trials in which memory already held the dirty value
 *   (the line left L1 on its own); those trials say nothing.
 *   remote_icache: a small function is written and made visible with
 *   flush_icache_range(), run on the other CPU (so its I-cache holds it),
 *   rewritten and flushed with local_flush_icache_range() on this CPU
 *   only, and run on the other CPU again: fresh means the other CPU fetched
 *   the new code (one I-cache, or this CPU's flush reaches it), stale that
 *   it ran its cached old copy.
 *
 * run (write): "mode=M iters=N sector=S len=L off=O cpu=C dev=PATH", all
 *   optional. Reads sectors of a block device (read only, through bios,
 *   not the page cache) into kernel pages and checks each 32-byte line of
 *   the read range: the cached view against memory (KSEG1) right after the
 *   read, and memory again after a write-back of the range (a stale line
 *   that was dirty would overwrite the device's data). The reads run on
 *   CPU C; "other" is the other online CPU. Modes:
 *     reuse     the other CPU fills a new page with a pattern through the
 *               cache (dirty lines, no flush); the page is freed, allocated
 *               again on CPU C and read into;
 *     stream    memory holds the pattern; a thread on the other CPU loads
 *               the read range in a loop until the read has completed (a
 *               stand-in for speculative and prefetch refills during the
 *               transfer and its invalidates);
 *     adjacent  as stream, but the thread loads the whole page in
 *               ascending order, which also drives the hardware prefetchers;
 *     idle      as stream, without the thread.
 *   Modes without a device (dev and sector unused), run through the arch
 *   DMA hooks themselves, so they cover every non-coherent DMA user:
 *     sim       memory holds the pattern, the other CPU loads the range in
 *               a loop; arch_sync_dma_for_device(FROM_DEVICE), the "device"
 *               writes the inverted pattern through KSEG1,
 *               arch_sync_dma_for_cpu(FROM_DEVICE), then the line check;
 *     sim-dirty the pattern is written through the cache and left dirty, on
 *               the other CPU (src=other, default) or this one (src=self),
 *               then the same transfer and check;
 *     wback     memory holds the inverted pattern, the pattern is written
 *               through the cache and left dirty (src as above), then
 *               arch_sync_dma_for_device(TO_DEVICE); a line whose memory
 *               does not hold the pattern counts as unwritten;
 *     sim-head  as sim, but the other CPU loads only the first 128 bytes of
 *               the range, so its fills fall into the passes' windows more
 *               often.
 *   Contract-compliant neighbour modes: the other CPU never loads a line of
 *   the buffer. Its thread loads [page base, buffer start) in ascending
 *   order, then writes back and invalidates those lines so that the next
 *   pass misses again (a stream that ends at the line before the buffer,
 *   which a hardware prefetcher may follow into it):
 *     neighbour      card read, memory holds the pattern (as stream);
 *     sim-neighbour  simulated transfer (as sim).
 *   scan=prev (any mode, off >= 512): right before the transfer is
 *   submitted (read_sectors, or arch_sync_dma_for_device), the reading CPU
 *   writes back and invalidates the 512 bytes before the buffer and loads
 *   them word by word in ascending order: exFAT scanning sector N-1 before
 *   it reads sector N.
 *   Simulated modes take len up to 64 KiB (off + len within 64 KiB, in
 *   physically contiguous pages), which reaches the whole-L1 index pass
 *   from 32 KiB when dma_cache.shared is 1.
 *   A run stops early on a signal to the writer or a write to stop.
 * result (read): the counts of the last run, with the dma_cache.order and
 *   dma_cache.shared in force; per 32-byte line position of the range (the
 *   first 16, then the rest), whether the stale lines pair into 64-byte L2
 *   lines, how many iterations had the range's first L2 line stale (and
 *   only that one), and the physical address of the buffer (to match
 *   dma_cache.verify_ring and sdhci records).
 * pfhold (read): the prefetch-hold probe, see pf_trial().
 *
 * prev (card) and sim-prev (simulated) run modes, contract-compliant: the
 * sibling's thread writes back and invalidates, then loads ascending, only
 * the 512 bytes immediately before the buffer, in a loop; it never touches
 * the buffer. Run parameters:
 *   stopmap=1  the thread stops before the map (sim: before
 *              arch_sync_dma_for_device(); card: before the bio is built,
 *              the map follows in the mmc path) after at least one pass;
 *   desc=1     instead, the 512 bytes right after the buffer, descending
 *              (towards the buffer);
 *   off=4096   the buffer at offset 0 of the second of two contiguous
 *              pages, so the scan ends at the first page's end (prev
 *              allocates two pages; the buffer must lie in one page).
 * stop (write): ends the current run after its current iteration.
 * dis_pfb (read/write): CCU MSCR bits 2 (DisPFB1, the L1-L2 prefetcher) and
 *   1 (DisPFB2, the L2-DDR prefetcher). Write 0, 2, 4 or 6. A write sets
 *   dma_cache.dis_pfb (dma-noncoherent.c), which is reapplied after resume.
 *
 * hang (write, supervised tests of the lockup detectors only): "soft N"
 *   spins N seconds on the writing CPU with preemption off and IRQs on
 *   (the soft lockup detector's case); "hard N" spins N seconds with IRQs
 *   off on that CPU (the buddy hard lockup detector's case). N is 1 to 30;
 *   larger values are refused. With the detectors' panic options set, the
 *   kernel panics during the spin and reboots after panic=; otherwise the
 *   write returns after N seconds ("hard" with N of 5 or more then meets
 *   the 5 s hardware watchdog first, if it is armed). One line is logged
 *   before the spin.
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/irqflags.h>
#include <linux/preempt.h>
#include <linux/sched/clock.h>
#include <linux/debugfs.h>
#include <linux/dma-direction.h>
#include <linux/dma-map-ops.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <asm/addrspace.h>
#include <asm/cacheflush.h>
#include <asm/cacheops.h>
#include <asm/cpu-info.h>
#include <asm/io.h>
#include <asm/r4kcache.h>
#include <asm/smp.h>

#define CCU_PHYS	0x12200000
#define CCU_MSCR	0x060
#define CCU_CCR		0x070
#define MSCR_PFB_MASK	0x6

#define L1_LINE		32
#define L2_LINE		64
#define PROBE_TRIALS	256

static DEFINE_MUTEX(check_lock);

static void __iomem *ccu(void)
{
	return (void __iomem *)CKSEG1ADDR(CCU_PHYS);
}

static inline u32 *uncached(void *va)
{
	return (u32 *)CKSEG1ADDR(virt_to_phys(va));
}

/* Write back and invalidate one address in L1, then in L2. */
static void line_flush(unsigned long va)
{
	cache_op(Hit_Writeback_Inv_D, va & ~(L1_LINE - 1UL));
	__sync();
	cache_op(Hit_Writeback_Inv_SD, va & ~(L2_LINE - 1UL));
	__sync();
}

static void range_flush(void *va, size_t len)
{
	unsigned long end = (unsigned long)va + len;
	unsigned long a;

	for (a = (unsigned long)va & ~(L1_LINE - 1UL); a < end; a += L1_LINE)
		cache_op(Hit_Writeback_Inv_D, a);
	__sync();
	for (a = (unsigned long)va & ~(L2_LINE - 1UL); a < end; a += L2_LINE)
		cache_op(Hit_Writeback_Inv_SD, a);
	__sync();
}

static void remote_hit_inv_d(void *info)
{
	cache_op(Hit_Invalidate_D, (unsigned long)info);
	__sync();
}

enum probe_kind { P_SELF, P_REMOTE, P_L2_INV, P_L2_WB, P_KINDS };

struct probe_count {
	unsigned int changed, unchanged, other, evicted;
};

#define PROBE_OLD	0x0bad0000u
#define PROBE_NEW	0x600d0000u

static int probe_one(enum probe_kind kind, u32 *page, struct probe_count *pc)
{
	unsigned int t;

	for (t = 0; t < PROBE_TRIALS; t++) {
		u32 *va = page + (t % (PAGE_SIZE / L2_LINE)) * (L2_LINE / 4);
		u32 *ua = uncached(va);
		u32 old = PROBE_OLD | t, new = PROBE_NEW | t, v, u, before;
		int ret = 0, other;

		preempt_disable();
		other = cpumask_any_but(cpu_online_mask, smp_processor_id());
		line_flush((unsigned long)va);
		WRITE_ONCE(*ua, old);
		__sync();
		WRITE_ONCE(*va, new);
		__sync();
		before = READ_ONCE(*ua);
		switch (kind) {
		case P_SELF:
			cache_op(Hit_Invalidate_D, (unsigned long)va);
			break;
		case P_REMOTE:
			ret = other < nr_cpu_ids ?
				smp_call_function_single(other, remote_hit_inv_d,
							 va, 1) : -ENODEV;
			break;
		case P_L2_INV:
			cache_op(Hit_Invalidate_SD,
				 (unsigned long)va & ~(L2_LINE - 1UL));
			break;
		case P_L2_WB:
			cache_op(Hit_Writeback_Inv_SD,
				 (unsigned long)va & ~(L2_LINE - 1UL));
			break;
		default:
			break;
		}
		__sync();
		v = READ_ONCE(*va);
		u = READ_ONCE(*ua);
		line_flush((unsigned long)va);
		preempt_enable();
		if (ret)
			return ret;
		/* The line left L1 on its own; the trial says nothing. */
		if (before == new || (kind != P_L2_WB && u == new)) {
			pc->evicted++;
			continue;
		}
		if (kind == P_L2_WB) {
			if (u == new)
				pc->changed++;
			else if (u == old)
				pc->unchanged++;
			else
				pc->other++;
			continue;
		}
		if (v == old)
			pc->changed++;
		else if (v == new)
			pc->unchanged++;
		else
			pc->other++;
	}
	return 0;
}

struct icall {
	u32 *code;
	u32 ret;
};

static void icache_write(u32 *code, u32 val)
{
	WRITE_ONCE(code[0], 0x3c020000 | (val >> 16));		/* lui v0, hi */
	WRITE_ONCE(code[1], 0x34420000 | (val & 0xffff));	/* ori v0, v0, lo */
	WRITE_ONCE(code[2], 0x03e00008);			/* jr ra */
	WRITE_ONCE(code[3], 0);					/* nop */
}

static void icache_call(void *info)
{
	struct icall *c = info;
	u32 (*fn)(void) = (u32 (*)(void))c->code;

	c->ret = fn();
}

struct icache_count {
	unsigned int fresh, stale, other, prime_bad;
};

static int probe_icache(u32 *page, struct icache_count *ic)
{
	unsigned int t;

	for (t = 0; t < PROBE_TRIALS; t++) {
		u32 *code = page + (t % (PAGE_SIZE / L2_LINE)) * (L2_LINE / 4);
		unsigned long a = (unsigned long)code, e = a + 16;
		u32 old = PROBE_OLD | t, new = PROBE_NEW | t;
		struct icall c = { code, 0 };
		int other, ret;

		preempt_disable();
		other = cpumask_any_but(cpu_online_mask, smp_processor_id());
		if (other >= nr_cpu_ids) {
			preempt_enable();
			return -ENODEV;
		}
		icache_write(code, old);
		flush_icache_range(a, e);
		ret = smp_call_function_single(other, icache_call, &c, 1);
		if (!ret && c.ret != old)
			ic->prime_bad++;
		icache_write(code, new);
		local_flush_icache_range(a, e);
		c.ret = 0;
		if (!ret)
			ret = smp_call_function_single(other, icache_call, &c, 1);
		preempt_enable();
		if (ret)
			return ret;
		if (c.ret == new)
			ic->fresh++;
		else if (c.ret == old)
			ic->stale++;
		else
			ic->other++;
	}
	/* Leave the page's code current everywhere before it is freed. */
	flush_icache_range((unsigned long)page, (unsigned long)page + PAGE_SIZE);
	return 0;
}

static int probe_show(struct seq_file *m, void *unused)
{
	static const char *const names[P_KINDS] = {
		"self_inv", "remote_inv", "l2_inv", "l2_wb" };
	static const char *const words[P_KINDS][2] = {
		{ "discarded", "kept" }, { "discarded", "kept" },
		{ "discarded", "kept" }, { "written", "not_written" } };
	struct probe_count pc[P_KINDS] = {};
	struct icache_count ic = {};
	int k, ret = 0;
	u32 *page;

	mutex_lock(&check_lock);
	page = (u32 *)__get_free_page(GFP_KERNEL);
	if (!page) {
		mutex_unlock(&check_lock);
		return -ENOMEM;
	}
	seq_printf(m, "ccu ccr 0x%08x mscr 0x%08x dma_cache.order %u shared %d\n",
		   readl(ccu() + CCU_CCR), readl(ccu() + CCU_MSCR),
		   READ_ONCE(mips_dma_cache_order), READ_ONCE(mips_cache_shared));
	cpus_read_lock();
	for_each_online_cpu(k)
		seq_printf(m, "cpu%d core %u foreign %*pbl siblings %*pbl\n", k,
			   cpu_core(&cpu_data[k]),
			   cpumask_pr_args(&cpu_foreign_map[k]),
			   cpumask_pr_args(&cpu_sibling_map[k]));
	for (k = 0; k < P_KINDS; k++) {
		if (k == P_REMOTE && num_online_cpus() < 2) {
			seq_puts(m, "remote_inv skipped: one CPU online\n");
			continue;
		}
		ret = probe_one(k, page, &pc[k]);
		if (ret)
			break;
		seq_printf(m, "%s trials %u %s %u %s %u other %u evicted %u\n",
			   names[k], PROBE_TRIALS, words[k][0], pc[k].changed,
			   words[k][1], pc[k].unchanged, pc[k].other,
			   pc[k].evicted);
	}
	if (!ret && num_online_cpus() >= 2) {
		ret = probe_icache(page, &ic);
		if (!ret)
			seq_printf(m, "remote_icache trials %u fresh %u stale %u other %u prime_bad %u\n",
				   PROBE_TRIALS, ic.fresh, ic.stale, ic.other,
				   ic.prime_bad);
	}
	cpus_read_unlock();
	free_page((unsigned long)page);
	mutex_unlock(&check_lock);
	return ret;
}
DEFINE_SHOW_ATTRIBUTE(probe);

/*
 * pfhold: can a copy of a line survive the hit operations that a DMA
 * unmap runs, if it was fetched while a CPU streamed towards it? Each
 * trial, on a page of its own and with preemption off:
 *   1. memory holds OLD in every word (KSEG1 stores), no line of the page
 *      is cached (write back and invalidate, L1 and L2);
 *   2. cached loads, one per 32-byte line, ascending over 64-byte lines
 *      0..k-1 (k = 4..62), on this CPU or on the sibling (IPI, waited for);
 *   3. KSEG1 stores of NEW into lines k and k+1;
 *   4. -delay: 1 ms;
 *   5. Hit_Invalidate_SD, SYNC, Hit_Invalidate_D on both L1 halves of line
 *      k, SYNC (not in -noinv);
 *   6. -pfbw: CCU MSCR written with both DisPFB bits inverted, then its
 *      old value (a prefetcher switch event after the invalidates, before
 *      the load);
 *   7. cached loads of both L1 halves of line k and of line k+1, and an
 *      uncached load of line k (mem_bad: memory does not hold NEW).
 *   6b. -refresh: dma_cache.refresh of line k (16 uncached loads, then 16
 *       cached stores); -refresh-evict: then Hit_Writeback_Inv_D on both
 *       halves and Hit_Writeback_Inv_SD, so the load misses both levels;
 *       -ucload: 16 uncached loads of line k, no store;
 *   (-pfbw is not run: an MSCR switch while the sibling ran panicked it.)
 * Line k: old (both halves OLD) means a copy survived the hit operations
 * (in -noinv: line k was fetched before the store, by a prefetch or a
 * speculative load); new is the expected result; mixed one half each.
 * Line k+1 is never invalidated: next_old says a copy of it was fetched
 * before step 3, two lines ahead of the stream. The variants run at the
 * dis_pfb in force (the script iterates 0, 2, 4, 6).
 */
#define PF_TRIALS	1024
#define PF_K_MIN	4

enum pf_variant { PF_SELF, PF_SIB, PF_SELF_DELAY, PF_SIB_DELAY, PF_SELF_PFBW,
		  PF_SIB_PFBW, PF_SELF_NOINV, PF_SIB_NOINV, PF_SELF_REFRESH,
		  PF_SIB_REFRESH, PF_SELF_REFEVICT, PF_SIB_REFEVICT,
		  PF_SELF_UCLOAD, PF_SIB_UCLOAD, PF_VARIANTS };
static const char *const pf_names[PF_VARIANTS] = {
	"self", "sibling", "self-delay", "sibling-delay", "self-pfbw",
	"sibling-pfbw", "self-noinv", "sibling-noinv", "self-refresh",
	"sibling-refresh", "self-refresh-evict", "sibling-refresh-evict",
	"self-ucload", "sibling-ucload" };

struct pf_load_args {
	unsigned long start, end;
};

static void pf_load(void *info)
{
	struct pf_load_args *a = info;
	unsigned long p;

	for (p = a->start; p < a->end; p += L1_LINE)
		(void)READ_ONCE(*(volatile u32 *)p);
}

struct pf_count {
	unsigned int old, new, mixed, other, next_old, next_new, next_other;
	unsigned int mem_bad;
};

static int pf_trial(u32 *page, unsigned int t, enum pf_variant v,
		    struct pf_count *pc)
{
	unsigned int k = PF_K_MIN + t % (PAGE_SIZE / L2_LINE - 1 - PF_K_MIN);
	u32 *lk = page + k * (L2_LINE / 4), *ln = lk + L2_LINE / 4;
	u32 *ua = uncached(page), *uk = uncached(lk);
	u32 old = PROBE_OLD | t, new = PROBE_NEW | t, a, b, n, m, mscr;
	struct pf_load_args args = { (unsigned long)page, (unsigned long)lk };
	bool sib = v & 1;	/* the sibling variants are the odd ones */
	unsigned int w;
	int other, ret = 0;

	preempt_disable();
	other = cpumask_any_but(cpu_online_mask, smp_processor_id());
	if (sib && other >= nr_cpu_ids) {
		preempt_enable();
		return -ENODEV;
	}
	for (w = 0; w < PAGE_SIZE / 4; w++)
		WRITE_ONCE(ua[w], old);
	__sync();
	range_flush(page, PAGE_SIZE);
	if (sib)
		ret = smp_call_function_single(other, pf_load, &args, 1);
	else
		pf_load(&args);
	for (w = 0; w < 2 * L2_LINE / 4; w++)
		WRITE_ONCE(uk[w], new);
	__sync();
	if (v == PF_SELF_DELAY || v == PF_SIB_DELAY)
		udelay(1000);
	if (v != PF_SELF_NOINV && v != PF_SIB_NOINV) {
		cache_op(Hit_Invalidate_SD, (unsigned long)lk);
		__sync();
		cache_op(Hit_Invalidate_D, (unsigned long)lk);
		cache_op(Hit_Invalidate_D, (unsigned long)lk + L1_LINE);
		__sync();
	}
	if (v == PF_SELF_PFBW || v == PF_SIB_PFBW) {
		/* the shared MSCR helper; dma_cache.dis_pfb is not changed */
		mscr = readl(ccu() + CCU_MSCR) & MSCR_PFB_MASK;
		x2000_mscr_set_pfb(mscr ^ MSCR_PFB_MASK);
		x2000_mscr_set_pfb(mscr);
		__sync();
	}
	if (v == PF_SELF_REFRESH || v == PF_SIB_REFRESH ||
	    v == PF_SELF_REFEVICT || v == PF_SIB_REFEVICT) {
		/* dma_cache.refresh on line k: all uncached loads, then stores */
		u32 r[L2_LINE / 4];

		for (w = 0; w < L2_LINE / 4; w++)
			r[w] = READ_ONCE(uk[w]);
		for (w = 0; w < L2_LINE / 4; w++)
			WRITE_ONCE(lk[w], r[w]);
		__sync();
	}
	if (v == PF_SELF_REFEVICT || v == PF_SIB_REFEVICT) {
		/* the refreshed line written back and dropped from L1, then L2 */
		cache_op(Hit_Writeback_Inv_D, (unsigned long)lk);
		cache_op(Hit_Writeback_Inv_D, (unsigned long)lk + L1_LINE);
		__sync();
		cache_op(Hit_Writeback_Inv_SD, (unsigned long)lk);
		__sync();
	}
	if (v == PF_SELF_UCLOAD || v == PF_SIB_UCLOAD) {
		/* uncached loads of line k alone, no store */
		for (w = 0; w < L2_LINE / 4; w++)
			(void)READ_ONCE(uk[w]);
		__sync();
	}
	a = READ_ONCE(lk[0]);
	b = READ_ONCE(lk[L1_LINE / 4]);
	n = READ_ONCE(ln[0]);
	m = READ_ONCE(uk[0]);
	range_flush(page, PAGE_SIZE);
	preempt_enable();
	if (ret)
		return ret;
	if (m != new)
		pc->mem_bad++;
	if (a == old && b == old)
		pc->old++;
	else if (a == new && b == new)
		pc->new++;
	else if ((a == old || a == new) && (b == old || b == new))
		pc->mixed++;
	else
		pc->other++;
	if (n == old)
		pc->next_old++;
	else if (n == new)
		pc->next_new++;
	else
		pc->next_other++;
	return 0;
}

static int pfhold_show(struct seq_file *m, void *unused)
{
	struct pf_count pc;
	unsigned int t;
	int v, ret = 0;
	u32 *page;

	mutex_lock(&check_lock);
	page = (u32 *)__get_free_page(GFP_KERNEL);
	if (!page) {
		mutex_unlock(&check_lock);
		return -ENOMEM;
	}
	range_flush(page, PAGE_SIZE);
	cpus_read_lock();
	seq_printf(m, "pfhold mscr 0x%08x dis_pfb %u trials %u k %u-%lu\n",
		   readl(ccu() + CCU_MSCR), readl(ccu() + CCU_MSCR) & MSCR_PFB_MASK,
		   PF_TRIALS, PF_K_MIN, PAGE_SIZE / L2_LINE - 2);
	for (v = 0; v < PF_VARIANTS && !ret; v++) {
		if (v == PF_SELF_PFBW || v == PF_SIB_PFBW) {
			/*
			 * Not run: a CCU MSCR prefetcher switch while the
			 * sibling ran panicked the Q2 (Reserved Instruction
			 * on the sibling, 2026-11-19).
			 */
			seq_printf(m, "pfhold %s skipped: MSCR writes at run time\n", pf_names[v]);
			continue;
		}
		if ((v & 1) && num_online_cpus() < 2) {
			seq_printf(m, "pfhold %s skipped: one CPU online\n", pf_names[v]);
			continue;
		}
		memset(&pc, 0, sizeof(pc));
		for (t = 0; t < PF_TRIALS && !ret; t++) {
			ret = pf_trial(page, t, v, &pc);
			if (!(t & 63))
				cond_resched();
		}
		if (!ret)
			seq_printf(m, "pfhold %s old %u new %u mixed %u other %u next_old %u next_new %u next_other %u mem_bad %u\n",
				   pf_names[v], pc.old, pc.new, pc.mixed, pc.other,
				   pc.next_old, pc.next_new, pc.next_other,
				   pc.mem_bad);
	}
	cpus_read_unlock();
	free_page((unsigned long)page);
	mutex_unlock(&check_lock);
	return ret;
}
DEFINE_SHOW_ATTRIBUTE(pfhold);

/* run */

enum run_mode { M_REUSE, M_STREAM, M_ADJACENT, M_IDLE, M_SIM, M_SIM_DIRTY, M_WBACK,
		M_SIM_HEAD, M_NEIGHBOUR, M_SIM_NEIGHBOUR, M_PREV, M_SIM_PREV };
static const char *const mode_names[] = {
	"reuse", "stream", "adjacent", "idle", "sim", "sim-dirty", "wback", "sim-head",
	"neighbour", "sim-neighbour", "prev", "sim-prev" };

/* Simulated transfers (no device): the arch DMA hooks, KSEG1 stores. */
static bool sim_mode(unsigned int mode)
{
	return mode >= M_SIM && mode != M_NEIGHBOUR && mode != M_PREV;
}

#define SCAN_BYTES	512
#define POS_LINES	16
#define MAX_ITERS	10000000
#define SIM_MAX		(64 * 1024)
#define HEAD_BYTES	128

static int stop_req;

struct run_cfg {
	char dev[64];
	unsigned int mode, iters, len, off;
	int cpu;
	bool self, scan_prev, stopmap, desc;
	u64 sector;
};

struct run_res {
	struct run_cfg cfg;
	int other, ret;
	unsigned int order, done, reused, read_errors;
	int shared;
	unsigned int lines, stale_lines, stale_iters, stale_pattern, mem_lines;
	unsigned int unwritten_lines, unwritten_iters;
	unsigned int buf_order;
	unsigned int stream_passes;
	long first_iter, first_off;
	u32 first_cached, first_mem;
	/* stale 32-byte lines by position in the range: 0..15, then the rest */
	unsigned int stale_pos[POS_LINES], stale_rest;
	/* 64-byte lines with both halves stale, with one half stale */
	unsigned int stale_pairs, stale_single;
	/* iterations with the range's first 64-byte line stale; with only it */
	unsigned int head_iters, head_only_iters;
	unsigned long long buf_phys;
	s64 usecs;
	bool valid;
};

static struct run_res last;

static struct {
	struct task_struct *task;
	wait_queue_head_t wq;
	unsigned long start, end;
	int armed, busy;
	bool flush, scan, preflush, desc;
	/* neighbour: lines flushed after each pass, [start, flush_end) */
	unsigned long flush_end;
	unsigned int passes;
} st;

static int streamer(void *unused)
{
	while (!kthread_should_stop()) {
		unsigned long a;

		if (!READ_ONCE(st.armed)) {
			smp_store_release(&st.busy, 0);
			wait_event_interruptible(st.wq, READ_ONCE(st.armed) ||
						 kthread_should_stop());
			continue;
		}
		/* busy before the second look at armed; stream_disarm() mirrors it */
		WRITE_ONCE(st.busy, 1);
		smp_mb();
		if (!READ_ONCE(st.armed))
			continue;
		/* prev: flush first, so every load of the pass misses */
		if (st.preflush)
			range_flush((void *)st.start, st.end - st.start);
		if (st.desc) {
			for (a = st.end; a > st.start; a -= L1_LINE)
				(void)READ_ONCE(*(volatile u32 *)(a - L1_LINE));
		} else {
			for (a = st.start; a < st.end; a += L1_LINE)
				(void)READ_ONCE(*(volatile u32 *)a);
		}
		/* neighbour: lines outside the buffer only; the next pass misses */
		if (st.flush && st.flush_end > st.start)
			range_flush((void *)st.start, st.flush_end - st.start);
		WRITE_ONCE(st.passes, st.passes + 1);
		cond_resched();
	}
	smp_store_release(&st.busy, 0);
	return 0;
}

static void stream_arm(unsigned long start, unsigned long end)
{
	st.start = start;
	st.end = end;
	/* with scan=prev the reading CPU's 512 bytes are not flushed under it */
	st.flush_end = st.scan ? end - min_t(unsigned long, end - start, SCAN_BYTES) : end;
	smp_store_release(&st.armed, 1);
	wake_up(&st.wq);
}

static void stream_disarm(void)
{
	WRITE_ONCE(st.armed, 0);
	smp_mb();
	while (smp_load_acquire(&st.busy))
		cpu_relax();
}

/* prev, sim-prev: the 512 bytes before the buffer, or after it with desc. */
static void prev_arm(struct run_cfg *c, unsigned long va)
{
	if (c->desc)
		stream_arm(va + c->len, va + c->len + SCAN_BYTES);
	else
		stream_arm(va - SCAN_BYTES, va);
}

/* stopmap: let the thread finish at least one pass, then stop it. */
static void prev_stop(unsigned int p0)
{
	unsigned int n;

	for (n = 0; n < 1000000 && READ_ONCE(st.passes) == p0; n++)
		cpu_relax();
	stream_disarm();
}

static u32 pattern(unsigned int iter, unsigned int word)
{
	return 0x5a000000u | ((iter & 0xff) << 16) | (word & 0xffff);
}

struct fill_args {
	u32 *va;
	unsigned int words, iter;
};

static void fill_cached(void *info)
{
	struct fill_args *f = info;
	unsigned int i;

	for (i = 0; i < f->words; i++)
		WRITE_ONCE(f->va[i], pattern(f->iter, i));
}

static int read_sectors(struct block_device *bdev, struct page *page,
			unsigned int off, unsigned int len, sector_t sector)
{
	struct bio *bio;
	int ret;

	bio = bio_alloc(bdev, 1, REQ_OP_READ | REQ_SYNC, GFP_KERNEL);
	bio->bi_iter.bi_sector = sector;
	__bio_add_page(bio, page, len, off);
	ret = submit_bio_wait(bio);
	bio_put(bio);
	return ret;
}

/* Check the read range of one iteration; snap holds len bytes. */
static void check_range(struct run_res *r, unsigned int iter, struct page *page,
			u32 *snap)
{
	unsigned int off = r->cfg.off, len = r->cfg.len, i, w, stale = 0;
	unsigned int l2_stale = 0, half = 0;
	bool head = false;
	u32 *va = page_address(page) + off;
	u32 *ua = uncached(va);

	for (w = 0; w < len / 4; w++)
		snap[w] = READ_ONCE(ua[w]);
	for (i = 0; i < len / L1_LINE; i++) {
		bool differ = false, pat = true;

		for (w = i * (L1_LINE / 4); w < (i + 1) * (L1_LINE / 4); w++) {
			u32 c = READ_ONCE(va[w]);

			if (c == snap[w])
				continue;
			if (r->first_iter < 0) {
				r->first_iter = iter;
				r->first_off = off + w * 4;
				r->first_cached = c;
				r->first_mem = snap[w];
			}
			differ = true;
			if (c != pattern(iter, off / 4 + w))
				pat = false;
		}
		r->lines++;
		if (differ) {
			stale++;
			if (pat)
				r->stale_pattern++;
			if (i < POS_LINES)
				r->stale_pos[i]++;
			else
				r->stale_rest++;
			half++;
		}
		/* off is a multiple of 512: lines 2j and 2j+1 share an L2 line */
		if (i & 1) {
			if (half == 2)
				r->stale_pairs++;
			else if (half == 1)
				r->stale_single++;
			if (half) {
				l2_stale++;
				if (i == 1)
					head = true;
			}
			half = 0;
		}
	}
	r->stale_lines += stale;
	if (stale)
		r->stale_iters++;
	if (head) {
		r->head_iters++;
		if (l2_stale == 1)
			r->head_only_iters++;
	}

	/* A stale line that was dirty reaches memory on this write-back. */
	range_flush(va, len);
	for (i = 0; i < len / L1_LINE; i++)
		for (w = i * (L1_LINE / 4); w < (i + 1) * (L1_LINE / 4); w++)
			if (READ_ONCE(ua[w]) != snap[w]) {
				r->mem_lines++;
				break;
			}
}

/*
 * scan=prev: the reading CPU writes back and invalidates the 512 bytes
 * before the buffer and loads them word by word, ascending.
 */
static void scan_before(u32 *va)
{
	u32 *p = va - SCAN_BYTES / 4;
	unsigned int w;

	range_flush(p, SCAN_BYTES);
	for (w = 0; w < SCAN_BYTES / 4; w++)
		(void)READ_ONCE(p[w]);
}

/* One simulated transfer through the arch DMA hooks; 0 or an error. */
static int sim_iter(struct run_res *r, unsigned int iter, struct page *page,
		    u32 *snap)
{
	struct run_cfg *c = &r->cfg;
	u32 *base = page_address(page), *va = base + c->off / 4;
	u32 *ua = uncached(va), *ubase = uncached(base);
	phys_addr_t phys = page_to_phys(page) + c->off;
	unsigned int w, words = c->len / 4, i, bad = 0, p0 = 0;
	size_t bytes = PAGE_SIZE << r->buf_order;
	struct fill_args f = { base, bytes / 4, iter };
	int src = c->self ? c->cpu : r->other, err = 0;

	range_flush(base, bytes);
	if (c->mode == M_SIM || c->mode == M_SIM_HEAD) {
		fill_cached(&f);
		range_flush(base, bytes);
		stream_arm((unsigned long)va, (unsigned long)va +
			   (c->mode == M_SIM_HEAD ? min_t(unsigned int, HEAD_BYTES, c->len) : c->len));
	} else if (c->mode == M_SIM_NEIGHBOUR) {
		fill_cached(&f);
		range_flush(base, bytes);
		/* [page base, buffer start): never a line of the buffer */
		stream_arm((unsigned long)va & PAGE_MASK, (unsigned long)va);
	} else if (c->mode == M_SIM_PREV) {
		fill_cached(&f);
		range_flush(base, bytes);
		p0 = READ_ONCE(st.passes);
		prev_arm(c, (unsigned long)va);
	} else {
		if (c->mode == M_WBACK) {
			for (w = 0; w < bytes / 4; w++)
				WRITE_ONCE(ubase[w], ~pattern(iter, w));
			__sync();
		}
		if (src == c->cpu)
			fill_cached(&f);
		else
			err = smp_call_function_single(src, fill_cached, &f, 1);
		if (err)
			return err;
	}

	if (c->mode == M_WBACK) {
		arch_sync_dma_for_device(phys, c->len, DMA_TO_DEVICE);
		for (i = 0; i < c->len / L1_LINE; i++) {
			for (w = i * (L1_LINE / 4); w < (i + 1) * (L1_LINE / 4); w++) {
				u32 m = READ_ONCE(ua[w]);

				if (m == pattern(iter, c->off / 4 + w))
					continue;
				if (r->first_iter < 0) {
					r->first_iter = iter;
					r->first_off = c->off + w * 4;
					r->first_cached = pattern(iter, c->off / 4 + w);
					r->first_mem = m;
				}
				bad++;
				break;
			}
			r->lines++;
		}
		r->unwritten_lines += bad;
		if (bad)
			r->unwritten_iters++;
		range_flush(base, bytes);
		return 0;
	}

	if (c->scan_prev)
		scan_before(va);
	if (c->mode == M_SIM_PREV && c->stopmap)
		prev_stop(p0);
	arch_sync_dma_for_device(phys, c->len, DMA_FROM_DEVICE);
	for (w = 0; w < words; w++)
		WRITE_ONCE(ua[w], ~pattern(iter, c->off / 4 + w));
	__sync();
	arch_sync_dma_for_cpu(phys, c->len, DMA_FROM_DEVICE);
	if (st.task)
		stream_disarm();
	check_range(r, iter, page, snap);
	return 0;
}

static int parse(char *buf, struct run_cfg *c)
{
	char *tok;

	strscpy(c->dev, "/dev/mmcblk0", sizeof(c->dev));
	c->mode = M_STREAM;
	c->iters = 1000;
	c->sector = 0;
	c->len = 512;
	c->off = 2048;
	c->cpu = 0;
	while ((tok = strsep(&buf, " \t\n")) != NULL) {
		unsigned int i;
		char *val;
		int ret = 0;

		if (!*tok)
			continue;
		val = strchr(tok, '=');
		if (!val)
			return -EINVAL;
		*val++ = 0;
		if (!strcmp(tok, "mode")) {
			for (i = 0; i < ARRAY_SIZE(mode_names); i++)
				if (!strcmp(val, mode_names[i]))
					break;
			if (i == ARRAY_SIZE(mode_names))
				return -EINVAL;
			c->mode = i;
		} else if (!strcmp(tok, "iters")) {
			ret = kstrtouint(val, 0, &c->iters);
		} else if (!strcmp(tok, "sector")) {
			ret = kstrtou64(val, 0, &c->sector);
		} else if (!strcmp(tok, "len")) {
			ret = kstrtouint(val, 0, &c->len);
		} else if (!strcmp(tok, "off")) {
			ret = kstrtouint(val, 0, &c->off);
		} else if (!strcmp(tok, "cpu")) {
			ret = kstrtoint(val, 0, &c->cpu);
		} else if (!strcmp(tok, "scan")) {
			if (strcmp(val, "prev") && strcmp(val, "none"))
				return -EINVAL;
			c->scan_prev = !strcmp(val, "prev");
		} else if (!strcmp(tok, "stopmap")) {
			ret = kstrtobool(val, &c->stopmap);
		} else if (!strcmp(tok, "desc")) {
			ret = kstrtobool(val, &c->desc);
		} else if (!strcmp(tok, "src")) {
			if (!strcmp(val, "self"))
				c->self = true;
			else if (strcmp(val, "other"))
				return -EINVAL;
		} else if (!strcmp(tok, "dev")) {
			if (strscpy(c->dev, val, sizeof(c->dev)) < 0)
				return -EINVAL;
		} else {
			return -EINVAL;
		}
		if (ret)
			return ret;
	}
	if (!c->len || c->len % 512 || c->off % 512 ||
	    c->off + c->len > (sim_mode(c->mode) ? SIM_MAX :
			       c->mode == M_PREV ? 2 * PAGE_SIZE : PAGE_SIZE) ||
	    !c->iters || c->iters > MAX_ITERS)
		return -EINVAL;
	if (c->cpu < 0 || c->cpu >= nr_cpu_ids || !cpu_online(c->cpu))
		return -EINVAL;
	/* scan=prev needs the 512 bytes before the buffer; wback has no read */
	if (c->scan_prev && (c->off < SCAN_BYTES || c->mode == M_WBACK ||
			     c->mode == M_REUSE))
		return -EINVAL;
	/*
	 * prev modes: 512 bytes before the buffer (or after it with desc)
	 * inside the allocation; the card buffer in one page.
	 */
	if (c->mode == M_PREV || c->mode == M_SIM_PREV) {
		unsigned int limit = c->mode == M_PREV ? 2 * PAGE_SIZE : SIM_MAX;

		if (c->desc ? c->off + c->len + SCAN_BYTES > limit :
			      c->off < SCAN_BYTES)
			return -EINVAL;
		if (c->mode == M_PREV &&
		    (c->off & ~PAGE_MASK) + c->len > PAGE_SIZE)
			return -EINVAL;
	} else if (c->stopmap || c->desc) {
		return -EINVAL;
	}
	/* neighbour modes need lines before the buffer to load */
	if ((c->mode == M_NEIGHBOUR || c->mode == M_SIM_NEIGHBOUR) &&
	    (!c->off || (c->off & ~PAGE_MASK) == 0))
		return -EINVAL;
	return 0;
}

static int do_run(struct run_res *r)
{
	struct run_cfg *c = &r->cfg;
	struct block_device *bdev;
	struct page *page = NULL;
	cpumask_var_t saved;
	unsigned int span, slots;
	sector_t nr;
	struct file *bf;
	unsigned int i, p0 = 0;
	ktime_t t0;
	u32 *snap;
	int ret = 0;

	r->first_iter = -1;
	r->order = READ_ONCE(mips_dma_cache_order);
	r->shared = READ_ONCE(mips_cache_shared);
	if (!alloc_cpumask_var(&saved, GFP_KERNEL))
		return -ENOMEM;
	snap = kmalloc(sim_mode(c->mode) ? SIM_MAX : PAGE_SIZE, GFP_KERNEL);
	if (!snap) {
		ret = -ENOMEM;
		goto out_mask;
	}
	bf = NULL;
	bdev = NULL;
	span = c->len >> SECTOR_SHIFT;
	slots = 1;
	if (!sim_mode(c->mode)) {
		if (c->iters > 1000000) {
			ret = -EINVAL;
			goto out_snap;
		}
		bf = bdev_file_open_by_path(c->dev, BLK_OPEN_READ, NULL, NULL);
		if (IS_ERR(bf)) {
			ret = PTR_ERR(bf);
			goto out_snap;
		}
		bdev = file_bdev(bf);
		nr = bdev_nr_sectors(bdev);
		if (nr < c->sector + span) {
			ret = -EINVAL;
			goto out_file;
		}
		/* Each iteration reads the next span, wrapping within 4096 spans. */
		slots = min_t(u64, 4096, div_u64(nr - c->sector, span));
	}

	cpumask_copy(saved, current->cpus_ptr);
	ret = set_cpus_allowed_ptr(current, cpumask_of(c->cpu));
	if (ret)
		goto out_file;
	r->other = cpumask_any_but(cpu_online_mask, c->cpu);
	if (r->other >= nr_cpu_ids) {
		ret = -ENODEV;
		goto out_affinity;
	}
	r->buf_order = sim_mode(c->mode) ?
		get_order(c->off + c->len + (c->desc ? SCAN_BYTES : 0)) :
		c->mode == M_PREV ? 1 : 0;
	if (c->mode != M_REUSE) {
		page = alloc_pages(GFP_KERNEL, r->buf_order);
		if (!page) {
			ret = -ENOMEM;
			goto out_affinity;
		}
	}
	if (page)
		r->buf_phys = page_to_phys(page) + c->off;
	if (c->mode == M_STREAM || c->mode == M_ADJACENT || c->mode == M_SIM ||
	    c->mode == M_SIM_HEAD || c->mode == M_NEIGHBOUR ||
	    c->mode == M_SIM_NEIGHBOUR || c->mode == M_PREV ||
	    c->mode == M_SIM_PREV) {
		init_waitqueue_head(&st.wq);
		st.armed = 0;
		st.busy = 0;
		st.flush = c->mode == M_NEIGHBOUR || c->mode == M_SIM_NEIGHBOUR;
		st.scan = c->scan_prev;
		st.preflush = c->mode == M_PREV || c->mode == M_SIM_PREV;
		st.desc = c->desc;
		st.passes = 0;
		st.task = kthread_create(streamer, NULL, "x2000-dma-stream");
		if (IS_ERR(st.task)) {
			ret = PTR_ERR(st.task);
			st.task = NULL;
			goto out_page;
		}
		kthread_bind(st.task, r->other);
		wake_up_process(st.task);
	}

	WRITE_ONCE(stop_req, 0);
	t0 = ktime_get();
	for (i = 0; i < c->iters; i++) {
		sector_t sector = c->sector + (sector_t)(i % slots) * span;
		struct fill_args f;
		int err;

		if (signal_pending(current) || READ_ONCE(stop_req)) {
			ret = -EINTR;
			break;
		}
		if (sim_mode(c->mode)) {
			ret = sim_iter(r, i, page, snap);
			if (ret)
				break;
			r->done++;
			if (!(i & 255))
				cond_resched();
			continue;
		}
		if (c->mode == M_REUSE) {
			struct page *p = alloc_page(GFP_KERNEL);

			if (!p) {
				ret = -ENOMEM;
				break;
			}
			f.va = page_address(p);
			f.words = PAGE_SIZE / 4;
			f.iter = i;
			err = smp_call_function_single(r->other, fill_cached, &f, 1);
			__free_page(p);
			page = alloc_page(GFP_KERNEL);
			if (!page) {
				ret = -ENOMEM;
				break;
			}
			if (page == p)
				r->reused++;
			r->buf_phys = page_to_phys(page) + c->off;
			if (err) {
				ret = err;
				break;
			}
		} else {
			unsigned long base = (unsigned long)page_address(page);

			f.va = page_address(page);
			f.words = (PAGE_SIZE << r->buf_order) / 4;
			f.iter = i;
			fill_cached(&f);
			range_flush(f.va, PAGE_SIZE << r->buf_order);
			if (c->mode == M_STREAM)
				stream_arm(base + c->off, base + c->off + c->len);
			else if (c->mode == M_ADJACENT)
				stream_arm(base, base + PAGE_SIZE);
			else if (c->mode == M_NEIGHBOUR)
				stream_arm(base, base + c->off);
			else if (c->mode == M_PREV) {
				p0 = READ_ONCE(st.passes);
				prev_arm(c, base + c->off);
			}
		}

		if (c->scan_prev)
			scan_before(page_address(page) + c->off);
		if (c->mode == M_PREV && c->stopmap)
			prev_stop(p0);
		/* prev: the buffer may be in the second page */
		err = read_sectors(bdev, page + (c->off >> PAGE_SHIFT),
				   c->off & ~PAGE_MASK, c->len, sector);
		if (st.task)
			stream_disarm();
		if (err)
			r->read_errors++;
		else
			check_range(r, i, page, snap);
		r->done++;
		if (c->mode == M_REUSE) {
			__free_page(page);
			page = NULL;
		}
		cond_resched();
	}
	r->usecs = ktime_us_delta(ktime_get(), t0);
	if (st.task) {
		r->stream_passes = st.passes;
		kthread_stop(st.task);
		st.task = NULL;
	}
 out_page:
	if (page)
		__free_pages(page, c->mode == M_REUSE ? 0 : r->buf_order);
 out_affinity:
	set_cpus_allowed_ptr(current, saved);
 out_file:
	if (bf)
		fput(bf);
 out_snap:
	kfree(snap);
 out_mask:
	free_cpumask_var(saved);
	return ret;
}

static ssize_t run_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct run_res *r;
	char *buf;
	int ret;

	if (count > 255)
		return -EINVAL;
	buf = memdup_user_nul(ubuf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r) {
		kfree(buf);
		return -ENOMEM;
	}
	ret = parse(buf, &r->cfg);
	kfree(buf);
	if (!ret) {
		mutex_lock(&check_lock);
		r->ret = do_run(r);
		r->valid = true;
		last = *r;
		mutex_unlock(&check_lock);
		ret = r->ret;
	}
	kfree(r);
	return ret ? ret : count;
}

static int result_show(struct seq_file *m, void *unused)
{
	struct run_res *r = kmalloc(sizeof(*r), GFP_KERNEL);
	unsigned int i;

	if (!r)
		return -ENOMEM;
	mutex_lock(&check_lock);
	*r = last;
	mutex_unlock(&check_lock);
	if (!r->valid) {
		seq_puts(m, "no run\n");
		kfree(r);
		return 0;
	}
	seq_printf(m, "mode %s order %u shared %d dev %s sector %llu len %u off %u cpu %d other %d src %s ret %d\n",
		   mode_names[r->cfg.mode], r->order, r->shared, r->cfg.dev,
		   (unsigned long long)r->cfg.sector, r->cfg.len, r->cfg.off,
		   r->cfg.cpu, r->other, r->cfg.self ? "self" : "other", r->ret);
	seq_printf(m, "iters %u done %u read_errors %u reused %u usecs %lld stream_passes %u\n",
		   r->cfg.iters, r->done, r->read_errors, r->reused,
		   (long long)r->usecs, r->stream_passes);
	seq_printf(m, "lines %u stale_lines %u stale_iters %u stale_pattern %u mem_lines %u unwritten_lines %u unwritten_iters %u\n",
		   r->lines, r->stale_lines, r->stale_iters, r->stale_pattern,
		   r->mem_lines, r->unwritten_lines, r->unwritten_iters);
	seq_printf(m, "stale_pos");
	for (i = 0; i < POS_LINES; i++)
		seq_printf(m, " %u", r->stale_pos[i]);
	seq_printf(m, " rest %u pairs %u single %u head_iters %u head_only_iters %u\n",
		   r->stale_rest, r->stale_pairs, r->stale_single, r->head_iters,
		   r->head_only_iters);
	seq_printf(m, "buf_phys 0x%llx scan %s stopmap %d desc %d\n", r->buf_phys,
		   r->cfg.scan_prev ? "prev" : "none", r->cfg.stopmap,
		   r->cfg.desc);
	if (r->first_iter >= 0)
		seq_printf(m, "first iter %ld offset %ld cached 0x%08x memory 0x%08x\n",
			   r->first_iter, r->first_off, r->first_cached,
			   r->first_mem);
	kfree(r);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(result);

static const struct file_operations run_fops = {
	.owner = THIS_MODULE,
	.write = run_write,
	.llseek = noop_llseek,
};

static ssize_t stop_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	WRITE_ONCE(stop_req, 1);
	return count;
}

static const struct file_operations stop_fops = {
	.owner = THIS_MODULE,
	.write = stop_write,
	.llseek = noop_llseek,
};

static int dis_pfb_get(void *data, u64 *val)
{
	*val = readl(ccu() + CCU_MSCR) & MSCR_PFB_MASK;
	return 0;
}

/* Through dma_cache.dis_pfb: the requested value, kept across resume. */
static int dis_pfb_set(void *data, u64 val)
{
	int ret;

	if (val & ~(u64)MSCR_PFB_MASK)
		return -EINVAL;
	mutex_lock(&check_lock);
	ret = x2000_dma_set_dis_pfb((unsigned int)val);
	mutex_unlock(&check_lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(dis_pfb_fops, dis_pfb_get, dis_pfb_set, "%llu\n");

#define HANG_MAX_S	30

static ssize_t hang_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	char buf[32], kind[8];
	unsigned int n;
	unsigned long flags;
	u64 t0, ns;
	bool hard;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = 0;
	if (sscanf(buf, "%7s %u", kind, &n) != 2)
		return -EINVAL;
	if (!strcmp(kind, "soft"))
		hard = false;
	else if (!strcmp(kind, "hard"))
		hard = true;
	else
		return -EINVAL;
	if (!n || n > HANG_MAX_S)
		return -EINVAL;
	ns = (u64)n * NSEC_PER_SEC;
	preempt_disable();
	pr_warn("x2000_dma hang: %s %u s on CPU %d\n", kind, n,
		smp_processor_id());
	if (hard)
		local_irq_save(flags);
	t0 = local_clock();
	while (local_clock() - t0 < ns)
		cpu_relax();
	if (hard)
		local_irq_restore(flags);
	preempt_enable();
	return count;
}

static const struct file_operations hang_fops = {
	.owner = THIS_MODULE,
	.write = hang_write,
	.llseek = noop_llseek,
};

static int __init x2000_dma_check_init(void)
{
	struct dentry *dir = debugfs_create_dir("x2000_dma", NULL);

	debugfs_create_file("probe", 0400, dir, NULL, &probe_fops);
	debugfs_create_file("pfhold", 0400, dir, NULL, &pfhold_fops);
	debugfs_create_file("run", 0200, dir, NULL, &run_fops);
	debugfs_create_file("result", 0400, dir, NULL, &result_fops);
	debugfs_create_file("stop", 0200, dir, NULL, &stop_fops);
	debugfs_create_file_unsafe("dis_pfb", 0600, dir, NULL, &dis_pfb_fops);
	debugfs_create_file("hang", 0200, dir, NULL, &hang_fops);
	return 0;
}
late_initcall(x2000_dma_check_init);
