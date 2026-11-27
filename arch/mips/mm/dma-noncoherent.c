// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2000  Ani Joshi <ajoshi@unixbox.com>
 * Copyright (C) 2000, 2001, 06	 Ralf Baechle <ralf@linux-mips.org>
 * swiped from i386, and cloned for MIPS by Geert, polished by Ralf.
 */
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/hardirq.h>
#include <linux/highmem.h>
#include <linux/jump_label.h>
#include <linux/moduleparam.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>

#include <asm/addrspace.h>
#include <asm/cache.h>
#include <asm/cpu-type.h>
#include <asm/io.h>
#include <asm/kernel-msa.h>
#ifdef CONFIG_CPU_R4K_CACHE_TLB
#include <asm/cacheops.h>
#include <asm/r4kcache.h>
#endif

/*
 * The affected CPUs below in 'cpu_needs_post_dma_flush()' can speculatively
 * fill random cachelines with stale data at any time, requiring an extra
 * flush post-DMA.
 *
 * Warning on the terminology - Linux calls an uncached area coherent;  MIPS
 * terminology calls memory areas with hardware maintained coherency coherent.
 *
 * Note that the R14000 and R16000 should also be checked for in this condition.
 * However this function is only called on non-I/O-coherent systems and only the
 * R10000 and R12000 are used in such systems, the SGI IP28 Indigo² rsp.
 * SGI IP32 aka O2.
 */
static inline bool cpu_needs_post_dma_flush(void)
{
	switch (boot_cpu_type()) {
	case CPU_R10000:
	case CPU_R12000:
	case CPU_BMIPS5000:
	case CPU_LOONGSON2EF:
	case CPU_XBURST:
		return true;
	default:
		/*
		 * Presence of MAARs suggests that the CPU supports
		 * speculatively prefetching data, and therefore requires
		 * the post-DMA flush/invalidate.
		 */
		return cpu_has_maar;
	}
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	dma_cache_wback_inv((unsigned long)page_address(page), size);
}

void *arch_dma_set_uncached(void *addr, size_t size)
{
	return (void *)(__pa(addr) + UNCAC_BASE);
}

static inline void dma_sync_virt_for_device(void *addr, size_t size,
		enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		dma_cache_wback((unsigned long)addr, size);
		break;
	case DMA_FROM_DEVICE:
		/* dma_cache.order 2: write back as well (c-r4k.c) */
		if (READ_ONCE(mips_dma_cache_order) == 2)
			dma_cache_wback_inv((unsigned long)addr, size);
		else
			dma_cache_inv((unsigned long)addr, size);
		break;
	case DMA_BIDIRECTIONAL:
		dma_cache_wback_inv((unsigned long)addr, size);
		break;
	default:
		BUG();
	}
}

static inline void dma_sync_virt_for_cpu(void *addr, size_t size,
		enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		dma_cache_inv_after((unsigned long)addr, size);
		break;
	default:
		BUG();
	}
}

/*
 * A single sg entry may refer to multiple physically contiguous pages.  But
 * we still need to process highmem pages individually.  If highmem is not
 * configured then the bulk of this loop gets optimized out.
 */
static inline void dma_sync_phys(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir, bool for_device)
{
	struct page *page = pfn_to_page(paddr >> PAGE_SHIFT);
	unsigned long offset = paddr & ~PAGE_MASK;
	size_t left = size;

	do {
		size_t len = left;
		void *addr;

		if (PageHighMem(page)) {
			if (offset + len > PAGE_SIZE)
				len = PAGE_SIZE - offset;
		}

		addr = kmap_atomic(page);
		if (for_device)
			dma_sync_virt_for_device(addr + offset, len, dir);
		else
			dma_sync_virt_for_cpu(addr + offset, len, dir);
		kunmap_atomic(addr);

		offset = 0;
		page++;
		left -= len;
	} while (left);
}

static inline void dma_edge(phys_addr_t paddr, size_t size,
			    enum dma_data_direction dir, bool after);
static inline void dma_pfb_map(enum dma_data_direction dir);

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir)
{
	dma_pfb_map(dir);
	dma_sync_phys(paddr, size, dir, true);
	dma_edge(paddr, size, dir, false);
}

#ifdef CONFIG_CPU_R4K_CACHE_TLB
/*
 * dma_cache.verify (/sys/module/dma_cache/parameters/verify, boot parameter
 * dma_cache.verify=N): after the cache maintenance of arch_sync_dma_for_cpu()
 * for a device write (FROM_DEVICE, BIDIRECTIONAL), compare the cached view
 * of the range with the same bytes through the uncached KSEG1 alias, one
 * verify line (the larger of the L1 and L2 line, 64 bytes on the X2000) at
 * a time, only the bytes inside the range:
 *
 *   0  off (default; a patched-out branch, static key);
 *   1  the first and the last line of the range;
 *   2  every line of the range.
 *
 * A line that differs is repaired: L2 hit operation, SYNC, L1 hit
 * operation on each L1 line of it, SYNC, then compared again. Lines wholly
 * inside the range are invalidated (Hit_Invalidate_SD, Hit_Invalidate_D):
 * from the map to this point the device owns the range, so no CPU data in
 * it may be kept, and memory holds what the device wrote. Lines that also
 * hold bytes outside the range are written back and invalidated instead
 * (Hit_Writeback_Inv_SD, Hit_Writeback_Inv_D), as mips_sc_inv() does for
 * the L2 edges, so dirty data of a neighbouring object is not lost; a
 * clean line is only invalidated by them. If such a write-back changes the
 * range's bytes in memory (a dirty edge line holding stale range bytes),
 * the record says "memory".
 *
 * Ranges above lowmem or beyond the KSEG1 window are counted as skipped.
 * Runs in the caller's context (also hard IRQ and under spinlocks: sdhci
 * unmaps in post_req and under the host lock): no sleeping; one
 * ratelimited line per mismatch. verify_stats counts the work (sums of
 * per-CPU counters, read without a lock); verify_ring holds the last
 * DMA_VERIFY_RING mismatch records, newest first, from this check and from
 * drivers (sdhci.dma_verify) through mips_dma_verify_report(). Timing (ns,
 * max_ns) is taken only with dma_cache.timing on.
 */
DECLARE_STATIC_KEY_FALSE(dma_cache_timing_key);

static unsigned int dma_cache_verify;
static DEFINE_STATIC_KEY_FALSE(dma_verify_key);

#define DMA_VERIFY_RING		32
#define DMA_VERIFY_LINE_MAX	128
#define DMA_VERIFY_KSEG1_END	0x20000000ULL

struct dma_verify_stat {
	u64 ranges, lines, bytes, mismatches, repaired, unrepaired, memory;
	u64 skipped, timed, ns, max_ns, seen;
};

static DEFINE_PER_CPU(struct dma_verify_stat, dma_verify_stats);

struct dma_verify_rec {
	u64 ns;
	u64 line, range;
	u32 seq, len, first, last, ndiff, mscr;
	u8 src, cpu, order, zero, result, edge;
};

static DEFINE_RAW_SPINLOCK(dma_verify_lock);
static struct dma_verify_rec dma_verify_ring[DMA_VERIFY_RING];
static u32 dma_verify_seq;

static const char *const dma_verify_src_names[] = { "arch", "sdhci" };
static const char *const dma_verify_result_names[] = {
	"-", "repaired", "unrepaired", "memory", "unrepaired+memory", "seen" };

static u32 dma_verify_mscr(void)
{
#ifdef CONFIG_MACH_X2000
	/* CCU MSCR: L2 enable and the prefetcher switches (DisPFB1, DisPFB2) */
	return readl((void __iomem *)CKSEG1ADDR(0x12200000 + 0x060));
#else
	return 0;
#endif
}

static unsigned int dma_verify_line_size(void)
{
	return max(cpu_dcache_line_size(), cpu_scache_line_size());
}

/*
 * One mismatch record. first and last are byte offsets in the range of the
 * first and the last differing byte, ndiff the number of differing bytes,
 * zero whether every differing cached byte is zero, result one of
 * MIPS_DMA_VERIFY_*, edge whether the checked line also holds bytes outside
 * the range (arch records; 0 from drivers).
 */
static void __dma_verify_report(unsigned int src, phys_addr_t range, size_t len,
				size_t first, size_t last, unsigned int ndiff,
				bool zero, unsigned int result, bool edge)
{
	unsigned int lsz = dma_verify_line_size();
	struct dma_verify_rec r = {
		.ns = local_clock(),
		.range = range,
		.line = (range + first) & ~(phys_addr_t)(lsz - 1),
		.len = len,
		.ndiff = ndiff,
		.mscr = dma_verify_mscr(),
		.src = src,
		.cpu = raw_smp_processor_id(),
		.order = READ_ONCE(mips_dma_cache_order),
		.zero = zero,
		.result = result,
		.edge = edge,
	};
	unsigned long flags;

	/* Offsets from the start of the line that holds the first difference. */
	r.first = range + first - r.line;
	r.last = range + last - r.line;
	raw_spin_lock_irqsave(&dma_verify_lock, flags);
	r.seq = ++dma_verify_seq;
	dma_verify_ring[r.seq % DMA_VERIFY_RING] = r;
	raw_spin_unlock_irqrestore(&dma_verify_lock, flags);
	pr_warn_ratelimited("dma_cache verify: %s seq %u line %llx range %llx len %u first %u last %u n %u zero %u edge %u cpu %u order %u mscr %x %s\n",
			    dma_verify_src_names[src % ARRAY_SIZE(dma_verify_src_names)],
			    r.seq, (unsigned long long)r.line,
			    (unsigned long long)r.range, r.len, r.first, r.last,
			    r.ndiff, r.zero, r.edge, r.cpu, r.order, r.mscr,
			    dma_verify_result_names[result % ARRAY_SIZE(dma_verify_result_names)]);
}

void mips_dma_verify_report(unsigned int src, phys_addr_t range, size_t len,
			    size_t first, size_t last, unsigned int ndiff,
			    bool zero, unsigned int result)
{
	__dma_verify_report(src, range, len, first, last, ndiff, zero, result,
			    false);
}
EXPORT_SYMBOL_GPL(mips_dma_verify_report);

struct dma_verify_diff {
	unsigned int first, last, n;
	bool zero;
};

/*
 * Compare bytes [s, e) of one line, cached view c against uncached view u
 * (word loads where whole words lie inside, byte loads at unaligned edges);
 * the uncached bytes are also copied to snap. Returns the differing bytes.
 */
static unsigned int dma_verify_cmp(const u8 *c, const u8 *u, unsigned int s,
				   unsigned int e, u8 *snap,
				   struct dma_verify_diff *d)
{
	unsigned int o, b;

	d->n = 0;
	d->zero = true;
	for (o = s; o < e; ) {
		u8 cb[4], ub[4];
		unsigned int k = 1;

		if (!(o & 3) && o + 4 <= e) {
			u32 cw = READ_ONCE(*(const u32 *)(c + o));
			u32 uw = READ_ONCE(*(const u32 *)(u + o));

			memcpy(snap + o, &uw, 4);
			if (cw == uw) {
				o += 4;
				continue;
			}
			memcpy(cb, &cw, 4);
			memcpy(ub, &uw, 4);
			k = 4;
		} else {
			cb[0] = READ_ONCE(c[o]);
			ub[0] = READ_ONCE(u[o]);
			snap[o] = ub[0];
		}
		for (b = 0; b < k; b++) {
			if (cb[b] == ub[b])
				continue;
			if (!d->n)
				d->first = o + b;
			d->last = o + b;
			d->n++;
			if (cb[b])
				d->zero = false;
		}
		o += k;
	}
	return d->n;
}

/* L2 then L1 hit operations on one verify line at va (aligned to lsz). */
static void dma_verify_repair(unsigned long va, unsigned int lsz,
			      unsigned long rs, unsigned long re)
{
	unsigned int l1 = cpu_dcache_line_size(), l2 = cpu_scache_line_size();
	unsigned long a;

	if (l2) {
		for (a = va; a < va + lsz; a += l2) {
			if (a >= rs && a + l2 <= re)
				cache_op(Hit_Invalidate_SD, a);
			else
				cache_op(Hit_Writeback_Inv_SD, a);
		}
		__sync();
	}
	for (a = va; a < va + lsz; a += l1) {
		if (a >= rs && a + l1 <= re)
			cache_op(Hit_Invalidate_D, a);
		else
			cache_op(Hit_Writeback_Inv_D, a);
	}
	__sync();
}

/* Check (and repair) the verify line at phys line; true if it differed. */
static bool dma_verify_line(phys_addr_t line, phys_addr_t paddr, size_t size,
			    unsigned int lsz, struct dma_verify_stat *st,
			    bool repair)
{
	phys_addr_t end = paddr + size;
	unsigned int s = paddr > line ? paddr - line : 0;
	unsigned int e = end < line + lsz ? end - line : lsz;
	const u8 *c = phys_to_virt(line);
	const u8 *u = (const u8 *)CKSEG1ADDR(line);
	u8 snap[DMA_VERIFY_LINE_MAX], snap2[DMA_VERIFY_LINE_MAX];
	struct dma_verify_diff d, d2;
	unsigned int result;
	bool differ, memory;

	st->lines++;
	st->bytes += e - s;
	if (!dma_verify_cmp(c, u, s, e, snap, &d))
		return false;
	st->mismatches++;
	if (!repair) {
		/* dma_cache.refresh follows and is the repair */
		st->seen++;
		__dma_verify_report(MIPS_DMA_VERIFY_SRC_ARCH, paddr, size,
				    line + d.first - paddr, line + d.last - paddr,
				    d.n, d.zero, MIPS_DMA_VERIFY_SEEN, s || e != lsz);
		return true;
	}
	dma_verify_repair((unsigned long)c, lsz, (unsigned long)phys_to_virt(paddr),
			  (unsigned long)phys_to_virt(paddr) + size);
	differ = dma_verify_cmp(c, u, s, e, snap2, &d2);
	/*
	 * Memory changed over the repair: on an edge line a write-back of a
	 * dirty line; on a line wholly inside the range (only invalidates were
	 * issued) another writer, or the device still writing.
	 */
	memory = memcmp(snap + s, snap2 + s, e - s);
	if (differ) {
		result = memory ? MIPS_DMA_VERIFY_UNREPAIRED_MEMORY :
				  MIPS_DMA_VERIFY_UNREPAIRED;
		st->unrepaired++;
	} else if (memory) {
		result = MIPS_DMA_VERIFY_MEMORY;
		st->memory++;
	} else {
		result = MIPS_DMA_VERIFY_REPAIRED;
		st->repaired++;
	}
	__dma_verify_report(MIPS_DMA_VERIFY_SRC_ARCH, paddr, size,
			    line + d.first - paddr, line + d.last - paddr,
			    d.n, d.zero, result, s || e != lsz);
	return true;
}

static noinline void dma_verify_range(phys_addr_t paddr, size_t size, bool repair)
{
	unsigned int level = READ_ONCE(dma_cache_verify);
	unsigned int lsz = dma_verify_line_size();
	struct dma_verify_stat st = {}, *pc;
	phys_addr_t end = paddr + size, first, last, line;
	unsigned long flags;
	u64 t0 = 0, dt = 0;

	if (!level || !size)
		return;
	if (static_branch_unlikely(&dma_cache_timing_key))
		t0 = local_clock();
	first = paddr & ~(phys_addr_t)(lsz - 1);
	last = (end - 1) & ~(phys_addr_t)(lsz - 1);
	if (lsz > DMA_VERIFY_LINE_MAX || end > DMA_VERIFY_KSEG1_END ||
	    PageHighMem(pfn_to_page(PFN_DOWN(end - 1)))) {
		st.skipped++;
	} else {
		st.ranges++;
		if (level == 1) {
			dma_verify_line(first, paddr, size, lsz, &st, repair);
			if (last != first)
				dma_verify_line(last, paddr, size, lsz, &st, repair);
		} else {
			for (line = first; line <= last; line += lsz)
				dma_verify_line(line, paddr, size, lsz, &st, repair);
		}
	}
	if (t0) {
		dt = local_clock() - t0;
		st.timed = 1;
	}
	local_irq_save(flags);
	pc = this_cpu_ptr(&dma_verify_stats);
	pc->ranges += st.ranges;
	pc->lines += st.lines;
	pc->bytes += st.bytes;
	pc->mismatches += st.mismatches;
	pc->repaired += st.repaired;
	pc->unrepaired += st.unrepaired;
	pc->memory += st.memory;
	pc->skipped += st.skipped;
	pc->seen += st.seen;
	pc->timed += st.timed;
	pc->ns += dt;
	if (dt > pc->max_ns)
		pc->max_ns = dt;
	local_irq_restore(flags);
}

static int dma_verify_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > 2)
		return -EINVAL;
	WRITE_ONCE(dma_cache_verify, v);
	if (v)
		static_branch_enable(&dma_verify_key);
	else
		static_branch_disable(&dma_verify_key);
	return 0;
}

static const struct kernel_param_ops dma_verify_ops = {
	.set = dma_verify_set,
	.get = param_get_uint,
};

static int dma_verify_stats_get(char *buf, const struct kernel_param *kp)
{
	struct dma_verify_stat s = {};
	int cpu;

	for_each_possible_cpu(cpu) {
		struct dma_verify_stat *p = &per_cpu(dma_verify_stats, cpu);

		s.ranges += p->ranges;
		s.lines += p->lines;
		s.bytes += p->bytes;
		s.mismatches += p->mismatches;
		s.repaired += p->repaired;
		s.unrepaired += p->unrepaired;
		s.memory += p->memory;
		s.skipped += p->skipped;
		s.seen += p->seen;
		s.timed += p->timed;
		s.ns += p->ns;
		s.max_ns = max(s.max_ns, p->max_ns);
	}
	return scnprintf(buf, PAGE_SIZE,
			 "verify %u ranges %llu lines %llu bytes %llu mismatches %llu repaired %llu unrepaired %llu memory %llu skipped %llu timed %llu ns %llu max_ns %llu records %u seen %llu\n",
			 READ_ONCE(dma_cache_verify), s.ranges, s.lines, s.bytes,
			 s.mismatches, s.repaired, s.unrepaired, s.memory,
			 s.skipped, s.timed, s.ns, s.max_ns,
			 READ_ONCE(dma_verify_seq), s.seen);
}

static const struct kernel_param_ops dma_verify_stats_ops = {
	.get = dma_verify_stats_get,
};

#define DMA_VERIFY_REC_MAX	192

/* Newest first; only whole records, so a full page drops the oldest. */
static int dma_verify_ring_get(char *buf, const struct kernel_param *kp)
{
	struct dma_verify_rec *ring;
	char one[DMA_VERIFY_REC_MAX];
	unsigned long flags;
	u32 seq, n;
	int len, k;

	ring = kmalloc(sizeof(dma_verify_ring), GFP_KERNEL);
	if (!ring)
		return -ENOMEM;
	raw_spin_lock_irqsave(&dma_verify_lock, flags);
	memcpy(ring, dma_verify_ring, sizeof(dma_verify_ring));
	seq = dma_verify_seq;
	raw_spin_unlock_irqrestore(&dma_verify_lock, flags);
	len = scnprintf(buf, PAGE_SIZE, "records %u ring %u\n", seq, DMA_VERIFY_RING);
	for (n = 0; n < DMA_VERIFY_RING && n < seq; n++) {
		struct dma_verify_rec *r = &ring[(seq - n) % DMA_VERIFY_RING];

		k = scnprintf(one, sizeof(one),
			      "%u %s ns %llu line %llx range %llx len %u first %u last %u n %u zero %u edge %u cpu %u order %u mscr %x %s\n",
			      r->seq, dma_verify_src_names[r->src % ARRAY_SIZE(dma_verify_src_names)],
			      r->ns, (unsigned long long)r->line,
			      (unsigned long long)r->range, r->len, r->first,
			      r->last, r->ndiff, r->zero, r->edge, r->cpu,
			      r->order, r->mscr,
			      dma_verify_result_names[r->result % ARRAY_SIZE(dma_verify_result_names)]);
		if (len + k >= PAGE_SIZE)
			break;
		memcpy(buf + len, one, k);
		len += k;
		buf[len] = 0;
	}
	kfree(ring);
	return len;
}

static const struct kernel_param_ops dma_verify_ring_ops = {
	.get = dma_verify_ring_get,
};

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "dma_cache."
module_param_cb(verify, &dma_verify_ops, &dma_cache_verify, 0644);
MODULE_PARM_DESC(verify, "After a device write: compare cached and uncached views, repair stale lines: 0 off, 1 first and last line, 2 every line (default 0)");
module_param_cb(verify_stats, &dma_verify_stats_ops, NULL, 0444);
MODULE_PARM_DESC(verify_stats, "dma_cache.verify counts");
module_param_cb(verify_ring, &dma_verify_ring_ops, NULL, 0444);
MODULE_PARM_DESC(verify_ring, "Last mismatch records, newest first");

/*
 * With dma_cache.refresh on, the verify runs before the refresh and only
 * counts and records ("seen"): after a refresh the cached view holds the
 * refreshed bytes and a difference would mean memory changed, so an
 * invalidate could discard the only good copy.
 */
static __always_inline void dma_verify(phys_addr_t paddr, size_t size,
				       enum dma_data_direction dir, bool repair)
{
	if (static_branch_unlikely(&dma_verify_key) &&
	    (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL))
		dma_verify_range(paddr, size, repair);
}

/*
 * dma_cache.edge (/sys/module/dma_cache/parameters/edge, boot parameter
 * dma_cache.edge=N): extra maintenance of the edge L2 lines of a range a
 * device writes (FROM_DEVICE, BIDIRECTIONAL), after the normal passes:
 *
 *   0  off (default);
 *   1  after a transfer (arch_sync_dma_for_cpu(), before dma_cache.verify);
 *   2  also at the map (arch_sync_dma_for_device()).
 *
 * On the first two and the last two L2 lines of the range: SYNC,
 * Hit_Writeback_Inv_SD on each, SYNC, Hit_Invalidate_D on each L1 line of
 * them that holds bytes of the range, SYNC. The model is mainline's
 * prefetch_cache_inv() for the BMIPS5000, whose prefetcher leaves stale
 * lines at DMA buffer edges; the Q2's stale events are the first L2 line of
 * a sector. L1 lines wholly outside the range are not touched; L1 lines at
 * the range's edges are invalidated as the normal L1 pass does. With
 * dma_cache.verify on, its repair counts show what the edge pass leaves.
 * edge_stats: ranges done after a transfer, at the map, and skipped
 * (highmem, no L2).
 */
static unsigned int dma_cache_edge;
static DEFINE_STATIC_KEY_FALSE(dma_edge_key);

struct dma_edge_stat {
	u64 after, map, skipped;
};

static DEFINE_PER_CPU(struct dma_edge_stat, dma_edge_stats);

static noinline void dma_edge_range(phys_addr_t paddr, size_t size, bool after)
{
	unsigned int l1 = cpu_dcache_line_size(), l2 = cpu_scache_line_size();
	phys_addr_t end = paddr + size, first, last, cand[4], lines[4];
	unsigned long rs, re, va, a;
	int n = 0, i;

	if (!size || !l2 || l1 > l2 ||
	    PageHighMem(pfn_to_page(PFN_DOWN(end - 1)))) {
		this_cpu_inc(dma_edge_stats.skipped);
		return;
	}
	first = paddr & ~(phys_addr_t)(l2 - 1);
	last = (end - 1) & ~(phys_addr_t)(l2 - 1);
	cand[0] = first;
	cand[1] = first + l2;
	cand[2] = last >= first + l2 ? last - l2 : first;
	cand[3] = last;
	/* ascending, inside [first, last], each once */
	for (i = 0; i < 4; i++)
		if (cand[i] >= first && cand[i] <= last &&
		    (!n || cand[i] > lines[n - 1]))
			lines[n++] = cand[i];
	rs = (unsigned long)phys_to_virt(paddr);
	re = rs + size;
	__sync();
	for (i = 0; i < n; i++)
		cache_op(Hit_Writeback_Inv_SD, (unsigned long)phys_to_virt(lines[i]));
	__sync();
	for (i = 0; i < n; i++) {
		va = (unsigned long)phys_to_virt(lines[i]);
		for (a = va; a < va + l2; a += l1)
			if (a + l1 > rs && a < re)
				cache_op(Hit_Invalidate_D, a);
	}
	__sync();
	if (after)
		this_cpu_inc(dma_edge_stats.after);
	else
		this_cpu_inc(dma_edge_stats.map);
}

static int dma_edge_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > 2)
		return -EINVAL;
	WRITE_ONCE(dma_cache_edge, v);
	if (v)
		static_branch_enable(&dma_edge_key);
	else
		static_branch_disable(&dma_edge_key);
	return 0;
}

static const struct kernel_param_ops dma_edge_ops = {
	.set = dma_edge_set,
	.get = param_get_uint,
};

static int dma_edge_stats_get(char *buf, const struct kernel_param *kp)
{
	struct dma_edge_stat s = {};
	int cpu;

	for_each_possible_cpu(cpu) {
		struct dma_edge_stat *p = &per_cpu(dma_edge_stats, cpu);

		s.after += p->after;
		s.map += p->map;
		s.skipped += p->skipped;
	}
	return scnprintf(buf, PAGE_SIZE, "edge %u after %llu map %llu skipped %llu\n",
			 READ_ONCE(dma_cache_edge), s.after, s.map, s.skipped);
}

static const struct kernel_param_ops dma_edge_stats_ops = {
	.get = dma_edge_stats_get,
};

module_param_cb(edge, &dma_edge_ops, &dma_cache_edge, 0644);
MODULE_PARM_DESC(edge, "Device writes: Hit_Writeback_Inv_SD and Hit_Invalidate_D on the first two and last two L2 lines after the normal passes: 0 off, 1 after a transfer, 2 also at the map (default 0)");
module_param_cb(edge_stats, &dma_edge_stats_ops, NULL, 0444);
MODULE_PARM_DESC(edge_stats, "dma_cache.edge ranges after a transfer, at the map, skipped");

static inline void dma_edge(phys_addr_t paddr, size_t size,
			    enum dma_data_direction dir, bool after)
{
	if (static_branch_unlikely(&dma_edge_key) &&
	    (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL) &&
	    (after || READ_ONCE(dma_cache_edge) == 2))
		dma_edge_range(paddr, size, after);
}

/*
 * dma_cache.refresh (/sys/module/dma_cache/parameters/refresh, boot
 * parameter dma_cache.refresh=N): after the after-transfer passes of a
 * device write (FROM_DEVICE, BIDIRECTIONAL), each byte of the range is
 * loaded through the uncached KSEG1 alias and stored through the cached
 * alias, one L2 line at a time: every load of a line before its first
 * store (aligned words, single bytes at an unaligned start or end):
 *
 *   0  off;
 *   1  ranges shorter than a page in full, larger ranges their first and
 *      last L2 line inside the range (on the Q2 not enough: interior lines
 *      of 4 KiB and 64 KiB simulated transfers stayed stale);
 *   2  every byte (default).
 *
 * On the X2000 a line fetched by a hardware prefetcher can outlive the
 * Hit_Invalidate passes and serve old contents to the first cached load
 * after the transfer. The refresh leaves the prefetchers on and never
 * writes CCU MSCR. After a level-2 refresh the cached view of the range
 * equals memory if:
 *   - the device has finished writing the range and does not write it
 *     again before the next arch_sync_dma_for_device() (otherwise the dirty
 *     lines written back later overwrite the device's data);
 *   - no CPU stores to the range during the refresh (a store between the
 *     loads and the stores of a line is lost);
 *   - cached stores write-allocate and later loads of a stored word return
 *     the stored value, also after the line is written back and evicted
 *     (on the Q2 tested by x2000_dma pfhold refresh-evict);
 *   - the bytes outside the range on an edge line are current in the copy
 *     the store's fill uses: they are written back with the line.
 * With shared or private caches other than the X2000's, a store on one
 * CPU does not replace another CPU's private copy. The lines are left dirty
 * with the device's data. Ranges that end above the KSEG1 window or in
 * highmem are counted as skipped and not refreshed. The cost is one
 * uncached load per word, with IRQs off where the caller has them off
 * (sdhci request completion, dwc2 gadget).
 *
 * dma_cache.refresh_msa (default 1): the 64-byte-aligned L2 lines wholly
 * inside the refreshed bytes are refreshed with MSA when
 * kernel_msa_begin() succeeds: per line four ld.d from the KSEG1 alias,
 * then four st.d to the cached alias, so every load of a line still comes
 * before its first store. An uncached access costs about 80 ns on the Q2
 * whatever its width, so a 16-byte load costs what a 4-byte load costs
 * (64 KiB: 21.3 us per KiB with lw/sw, 5.5 us per KiB with ld.d/st.d). The
 * partial lines at the ends, and the whole range when MSA is refused (hard
 * or soft IRQ context, IRQs off, a kernel MSA section already open, no
 * MSA), take the word and byte path above. An MSA section disables
 * preemption and bottom halves; it is closed and reopened every
 * DMA_REFRESH_MSA_SECTION bytes. 0: the word and byte path for every byte.
 * refresh_stats adds msa_bytes, scalar_bytes and irq_bytes (the scalar
 * bytes of ranges refreshed in IRQ context or with IRQs off).
 *
 * dma_cache.refresh_skip_inv (default 0): with refresh 2, a range that the
 * refresh will cover (lowmem, ending at or below the KSEG1 window end, L2
 * line no larger than DMA_VERIFY_LINE_MAX) skips the after-transfer cache
 * passes (the order's L2 and L1 invalidates and dma_cache.edge) in
 * arch_sync_dma_for_cpu(). The decision and the refresh level are taken
 * once, before the passes, so a parameter write between them cannot leave
 * a range neither invalidated nor refreshed. The map-time passes
 * (arch_sync_dma_for_device()) are unchanged. Why the refresh alone gives
 * the cached view memory's value:
 *   - every byte of the range is stored through the cache with the value
 *     loaded from memory through KSEG1 after the transfer; a store to a
 *     line that is present (a clean stale copy, prefetched during the
 *     transfer) overwrites those bytes in it, and a store to an absent line
 *     allocates it. The invalidate before the refresh only removed copies
 *     the stores overwrite. This rests on the refresh conditions above, and
 *     with the passes skipped also on a stale L2 copy filled during the
 *     transfer being replaced when the dirty L1 line is written back (the
 *     L1 and L2 of the X2000 are not inclusive). The stale-line checks that
 *     read right after the refresh (sdhci.dma_verify, x2000_dma run) hit
 *     the dirty L1 line and do not test this; a check must compare the
 *     cached and KSEG1 views again after the refreshed lines are evicted
 *     from L1 (as x2000_dma pfhold refresh-evict does for one line);
 *   - the bytes of an edge line outside the range are not stored by the
 *     refresh, and the after-transfer passes would not have changed them
 *     either: the map-time pass writes back and invalidates the partial
 *     edge lines (dma_cache.order 1, 0153), and with the passes skipped the
 *     line is not invalidated after the transfer, so a neighbour's dirty
 *     bytes written during the transfer stay in the cache.
 * refresh_stats counts the ranges that skipped the passes (skip_inv).
 * With dma_cache.verify on, its count-only check runs before the refresh
 * also for a range that skipped the passes; its "seen" counts then include
 * lines the passes would have invalidated.
 */
static unsigned int dma_cache_refresh = 2;
static DEFINE_STATIC_KEY_TRUE(dma_refresh_key);
static bool dma_cache_refresh_msa = true;
static bool dma_cache_refresh_skip_inv;

#define DMA_REFRESH_MSA_LINE	64
#define DMA_REFRESH_MSA_SECTION	(16 * 1024)

struct dma_refresh_stat {
	u32 ranges, partial, skipped, timed, skip_inv;
	u64 bytes, ns, max_ns;
	u64 msa_bytes, scalar_bytes, irq_bytes;
};

struct dma_refresh_acct {
	unsigned long msa, scalar, irq;
};
static DEFINE_PER_CPU(struct dma_refresh_stat, dma_refresh_stats);

/*
 * Bytes [va, va + n) within one L2 line (n <= DMA_VERIFY_LINE_MAX): every
 * uncached load before the first cached store, so no store to the line (and
 * a write-back of it) can precede a load of another part of it.
 */
static __always_inline void dma_refresh_chunk(unsigned long va, unsigned long n)
{
	const unsigned long d = CKSEG1ADDR(__pa(va)) - va;
	unsigned long e = va + n, a, w0, w1;
	u32 w[DMA_VERIFY_LINE_MAX / 4];
	u8 hb[3], tb[3];
	unsigned int h, t, k, i;

	w0 = min(ALIGN(va, 4), e);
	w1 = max(w0, e & ~3UL);
	h = w0 - va;
	k = (w1 - w0) / 4;
	t = e - w1;
	for (i = 0; i < h; i++)
		hb[i] = READ_ONCE(*(const volatile u8 *)(va + i + d));
	for (i = 0, a = w0; i < k; i++, a += 4)
		w[i] = READ_ONCE(*(const volatile u32 *)(a + d));
	for (i = 0; i < t; i++)
		tb[i] = READ_ONCE(*(const volatile u8 *)(w1 + i + d));
	for (i = 0; i < h; i++)
		WRITE_ONCE(*(u8 *)(va + i), hb[i]);
	for (i = 0, a = w0; i < k; i++, a += 4)
		WRITE_ONCE(*(u32 *)a, w[i]);
	for (i = 0; i < t; i++)
		WRITE_ONCE(*(u8 *)(w1 + i), tb[i]);
}

static void dma_refresh_bytes(unsigned long va, unsigned long n,
			      unsigned int lsz)
{
	unsigned long e = va + n, c;

	while (va < e) {
		c = min(e, (va | (lsz - 1)) + 1);
		dma_refresh_chunk(va, c - va);
		va = c;
	}
}

#ifdef CONFIG_CPU_HAS_MSA
/* One 64-byte line at va: four uncached ld.d, then four cached st.d. */
static __always_inline void dma_refresh_line_msa(unsigned long va,
						 unsigned long d)
{
	__asm__ __volatile__(
		".set push\n\t.set noat\n\t"
		".set hardfloat\n\t.set fp=64\n\t.set msa\n\t"
		"ld.d $w0, 0(%[u])\n\t"
		"ld.d $w1, 16(%[u])\n\t"
		"ld.d $w2, 32(%[u])\n\t"
		"ld.d $w3, 48(%[u])\n\t"
		"st.d $w0, 0(%[c])\n\t"
		"st.d $w1, 16(%[c])\n\t"
		"st.d $w2, 32(%[c])\n\t"
		"st.d $w3, 48(%[c])\n\t"
		".set pop\n\t"
		: : [u] "r" (va + d), [c] "r" (va) : "memory");
}

/*
 * The whole lines [va, va + n) (va and n multiples of 64) with MSA, one
 * kernel MSA section per DMA_REFRESH_MSA_SECTION bytes. Returns the bytes
 * refreshed, a prefix of the span; 0 if kernel_msa_begin() refused.
 */
static unsigned long dma_refresh_lines_msa(unsigned long va, unsigned long n)
{
	const unsigned long d = CKSEG1ADDR(__pa(va)) - va;
	unsigned long done = 0, end, a;

	while (done < n) {
		if (!kernel_msa_begin())
			break;
		end = min(n, done + DMA_REFRESH_MSA_SECTION);
		for (a = va + done; a < va + end; a += DMA_REFRESH_MSA_LINE)
			dma_refresh_line_msa(a, d);
		kernel_msa_end();
		done = end;
	}
	return done;
}
#else
static unsigned long dma_refresh_lines_msa(unsigned long va, unsigned long n)
{
	return 0;
}
#endif

/*
 * Refresh [va, va + n): the whole 64-byte lines with MSA where allowed, the
 * rest by dma_refresh_bytes(). A line is refreshed by one path only.
 */
static void dma_refresh_span(unsigned long va, unsigned long n,
			     unsigned int lsz, struct dma_refresh_acct *acct)
{
	unsigned long e = va + n, l0, l1, m = 0;
	bool irq = !in_task() || irqs_disabled();

	if (!n)
		return;
	l0 = ALIGN(va, DMA_REFRESH_MSA_LINE);
	l1 = e & ~(DMA_REFRESH_MSA_LINE - 1UL);
	/* every span in IRQ context or with IRQs off is counted, MSA or not */
	if (irq)
		acct->irq += n;
	else if (IS_ENABLED(CONFIG_CPU_HAS_MSA) && lsz == DMA_REFRESH_MSA_LINE &&
		 READ_ONCE(dma_cache_refresh_msa) && l1 > l0)
		m = dma_refresh_lines_msa(l0, l1 - l0);
	if (!m) {
		dma_refresh_bytes(va, n, lsz);
		acct->scalar += n;
		return;
	}
	dma_refresh_bytes(va, l0 - va, lsz);
	dma_refresh_bytes(l0 + m, e - (l0 + m), lsz);
	acct->msa += m;
	acct->scalar += n - m;
}

/* The refresh can cover [paddr, paddr + size): lowmem below the KSEG1 end. */
static bool dma_refresh_covers(phys_addr_t paddr, size_t size)
{
	phys_addr_t end = paddr + size;

	return size && dma_verify_line_size() <= DMA_VERIFY_LINE_MAX &&
	       end <= DMA_VERIFY_KSEG1_END &&
	       !PageHighMem(pfn_to_page(PFN_DOWN(end - 1)));
}

/*
 * level: dma_cache.refresh read once by the caller; skip_inv: the caller
 * skipped the after-transfer passes (only with level 2 and a covered range).
 */
static noinline void dma_refresh_range(phys_addr_t paddr, size_t size,
				       unsigned int level, bool skip_inv)
{
	struct dma_refresh_acct acct = {};
	unsigned int l2 = dma_verify_line_size();
	struct dma_refresh_stat *pc;
	unsigned long flags, va, n, done = 0, head, tail;
	u64 t0 = 0, dt = 0;
	bool partial = false;

	if (!level || !size)
		return;
	if (!dma_refresh_covers(paddr, size)) {
		this_cpu_inc(dma_refresh_stats.skipped);
		return;
	}
	if (static_branch_unlikely(&dma_cache_timing_key))
		t0 = local_clock();
	va = (unsigned long)phys_to_virt(paddr);
	n = size;
	if (level == 1 && size >= PAGE_SIZE) {
		/* through the end of the first L2 line, from the start of the last */
		head = min_t(unsigned long, n, l2 - (va & (l2 - 1)));
		tail = min_t(unsigned long, n - head, ((va + n - 1) & (l2 - 1)) + 1);
		dma_refresh_span(va, head, l2, &acct);
		dma_refresh_span(va + n - tail, tail, l2, &acct);
		done = head + tail;
		partial = true;
	} else {
		dma_refresh_span(va, n, l2, &acct);
		done = n;
	}
	__sync();
	if (t0)
		dt = local_clock() - t0;
	local_irq_save(flags);
	pc = this_cpu_ptr(&dma_refresh_stats);
	pc->ranges++;
	pc->partial += partial;
	pc->skip_inv += skip_inv;
	pc->bytes += done;
	pc->msa_bytes += acct.msa;
	pc->scalar_bytes += acct.scalar;
	pc->irq_bytes += acct.irq;
	if (t0) {
		pc->timed++;
		pc->ns += dt;
		if (dt > pc->max_ns)
			pc->max_ns = dt;
	}
	local_irq_restore(flags);
}

static int dma_refresh_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > 2)
		return -EINVAL;
	WRITE_ONCE(dma_cache_refresh, v);
	if (v)
		static_branch_enable(&dma_refresh_key);
	else
		static_branch_disable(&dma_refresh_key);
	return 0;
}

static const struct kernel_param_ops dma_refresh_ops = {
	.set = dma_refresh_set,
	.get = param_get_uint,
};

static int dma_refresh_stats_get(char *buf, const struct kernel_param *kp)
{
	struct dma_refresh_stat s = {};
	int cpu;

	for_each_possible_cpu(cpu) {
		struct dma_refresh_stat *p = &per_cpu(dma_refresh_stats, cpu);

		s.ranges += p->ranges;
		s.partial += p->partial;
		s.skipped += p->skipped;
		s.skip_inv += p->skip_inv;
		s.timed += p->timed;
		s.bytes += p->bytes;
		s.ns += p->ns;
		if (p->max_ns > s.max_ns)
			s.max_ns = p->max_ns;
		s.msa_bytes += p->msa_bytes;
		s.scalar_bytes += p->scalar_bytes;
		s.irq_bytes += p->irq_bytes;
	}
	return sysfs_emit(buf, "refresh %u ranges %u partial %u skipped %u bytes %llu timed %u ns %llu max_ns %llu msa %u msa_bytes %llu scalar_bytes %llu irq_bytes %llu skip_inv_on %u skip_inv %u\n",
			  READ_ONCE(dma_cache_refresh), s.ranges, s.partial,
			  s.skipped, s.bytes, s.timed, s.ns, s.max_ns,
			  READ_ONCE(dma_cache_refresh_msa), s.msa_bytes,
			  s.scalar_bytes, s.irq_bytes,
			  READ_ONCE(dma_cache_refresh_skip_inv), s.skip_inv);
}

static int dma_refresh_stats_set(const char *val, const struct kernel_param *kp)
{
	int cpu;

	for_each_possible_cpu(cpu)
		memset(&per_cpu(dma_refresh_stats, cpu), 0,
		       sizeof(struct dma_refresh_stat));
	return 0;
}

static const struct kernel_param_ops dma_refresh_stats_ops = {
	.set = dma_refresh_stats_set,
	.get = dma_refresh_stats_get,
};

module_param_cb(refresh, &dma_refresh_ops, &dma_cache_refresh, 0644);
MODULE_PARM_DESC(refresh, "After a device write: load each byte through KSEG1 and store it through the cache: 0 off, 1 sub-page ranges and the edge L2 lines of larger ones, 2 every byte (default)");
module_param_named(refresh_msa, dma_cache_refresh_msa, bool, 0644);
MODULE_PARM_DESC(refresh_msa, "dma_cache.refresh: whole 64-byte lines with MSA ld.d/st.d where kernel_msa_begin() allows (default 1)");
module_param_named(refresh_skip_inv, dma_cache_refresh_skip_inv, bool, 0644);
MODULE_PARM_DESC(refresh_skip_inv, "With refresh 2: skip the after-transfer cache passes of a device write the refresh covers (default 0)");
module_param_cb(refresh_stats, &dma_refresh_stats_ops, NULL, 0644);
MODULE_PARM_DESC(refresh_stats, "dma_cache.refresh counts and time (dma_cache.timing); write anything to clear");

static __always_inline bool dma_refresh_on(enum dma_data_direction dir)
{
	return (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL) &&
	       static_branch_unlikely(&dma_refresh_key);
}

static __always_inline unsigned int dma_refresh_level(void)
{
	return READ_ONCE(dma_cache_refresh);
}

/*
 * Before the after-transfer passes of a device write that will be
 * refreshed at level: may this range skip them?
 */
static __always_inline bool dma_refresh_skips_inv(phys_addr_t paddr,
		size_t size, unsigned int level)
{
	return level == 2 && READ_ONCE(dma_cache_refresh_skip_inv) &&
	       dma_refresh_covers(paddr, size);
}
#else
static inline void dma_verify(phys_addr_t paddr, size_t size,
			      enum dma_data_direction dir, bool repair) { }
static inline bool dma_refresh_on(enum dma_data_direction dir) { return false; }
static inline unsigned int dma_refresh_level(void) { return 0; }
static inline bool dma_refresh_skips_inv(phys_addr_t paddr, size_t size,
		unsigned int level) { return false; }
static inline void dma_refresh_range(phys_addr_t paddr, size_t size,
				     unsigned int level, bool skip_inv) { }
static inline void dma_edge(phys_addr_t paddr, size_t size,
			    enum dma_data_direction dir, bool after) { }
#endif /* CONFIG_CPU_R4K_CACHE_TLB */

#ifdef CONFIG_MACH_X2000
/*
 * dma_cache.dis_pfb (/sys/module/dma_cache/parameters/dis_pfb, boot
 * parameter dma_cache.dis_pfb=N): the X2000 CCU MSCR prefetcher switches,
 * bit 2 DisPFB1 (L1-L2 prefetcher) and bit 1 DisPFB2 (L2-DDR prefetcher),
 * XBurst2 core manual 8.2.4. 0 both on (default, the reset value),
 * 2 DisPFB2, 4 DisPFB1, 6 both off.
 *
 * On the Q2 under dma_cache.order 1, both prefetchers off removed every
 * stale line in the x2000_dma runs, the contract-violating modes included;
 * either one alone did not (docs/validation/dma-cache-order-2026-11-19.md).
 * The default keeps them on; the parameter is for the A/B.
 *
 * Applied at the boot parameter, again at early_initcall (before SMP
 * bring-up and before any driver starts DMA), on every write of the
 * parameter, and after resume from sleep (syscore resume: the deep-sleep
 * path does not save MSCR, and a CCU reset would return it to 0). MSCR is
 * shared by both CPUs: a read-modify-write of bits 1 and 2 only, under a
 * raw spinlock, which x2000_dma (debugfs) uses too. Reading the parameter
 * returns the bits read back from MSCR; dis_pfb_state adds the requested
 * value, the whole register and the number of applications.
 *
 * dma_cache.pfb (default 0, no change), for device writes (FROM_DEVICE,
 * BIDIRECTIONAL):
 *   1  HOLD: both DisPFB bits set at the start of the after-transfer
 *      maintenance in arch_sync_dma_for_cpu(), SYNC, the order's passes
 *      (and dma_cache.edge), then released (before dma_cache.verify);
 *   2  PULSE: set, SYNC, released, then the after-transfer passes;
 *   3  OFF from the map (before its passes) until the unmap's
 *      arch_sync_dma_for_cpu() (after its passes), while any such mapping
 *      is outstanding.
 * A depth counter and the MSCR read-modify-write share one raw spinlock,
 * held only around them: the first holder sets both bits (ORed with the
 * requested dis_pfb, so bits it already requests stay set), the last
 * restores the requested value; nested or concurrent unmaps (dwc2 in hard
 * IRQ, SDIO under the host lock, the other CPU) only change the counter. A
 * release with no holder (mode 3: a dma_sync_*_for_cpu() without a map) is
 * counted as unbalanced and ignored. A long-lived mapping keeps the
 * prefetchers off for its lifetime in mode 3. Writing pfb drops all holds.
 * pfb_stats: holds (1), pulses (2), map holds and unmap releases (3),
 * unbalanced, current depth, MSCR writes, and with dma_cache.timing on
 * their count, total and maximum ns (one locked read-modify-write each).
 */
#define X2000_CCU_MSCR		((void __iomem *)CKSEG1ADDR(0x12200000 + 0x060))
#define X2000_MSCR_PFB		0x6

static unsigned int dma_cache_dis_pfb;
static DEFINE_RAW_SPINLOCK(x2000_mscr_lock);
static unsigned int x2000_pfb_applied, x2000_pfb_resumed;
/* pfb_pulse holders; under x2000_mscr_lock */
static unsigned int x2000_pfb_holds;

static unsigned int dma_cache_pfb;
static DEFINE_STATIC_KEY_FALSE(dma_pfb_key);
static atomic_t x2000_pfb_holds_n, x2000_pfb_pulses, x2000_pfb_maps,
		x2000_pfb_unmaps, x2000_pfb_unbalanced, x2000_pfb_writes;
/* MSCR write timing (dma_cache.timing on); under x2000_mscr_lock */
static u64 x2000_mscr_ns, x2000_mscr_max_ns;
static u32 x2000_mscr_timed;

static u32 __x2000_mscr_set_pfb(u32 bits)
{
	u32 v = (readl(X2000_CCU_MSCR) & ~X2000_MSCR_PFB) | (bits & X2000_MSCR_PFB);

	writel(v, X2000_CCU_MSCR);
	return readl(X2000_CCU_MSCR);
}

/* Prefetcher bits of MSCR to bits (0, 2, 4, 6); returns MSCR read back. */
u32 x2000_mscr_set_pfb(u32 bits)
{
	unsigned long flags;
	u32 v;

	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	v = __x2000_mscr_set_pfb(bits);
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
	return v;
}
EXPORT_SYMBOL_GPL(x2000_mscr_set_pfb);

/* The requested value, unless a pfb holder has the bits set. */
static void x2000_pfb_apply(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	if (!x2000_pfb_holds)
		__x2000_mscr_set_pfb(READ_ONCE(dma_cache_dis_pfb));
	x2000_pfb_applied++;
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
}

/* A pfb MSCR write under the lock, timed with dma_cache.timing on. */
static void __x2000_pfb_write(u32 bits)
{
	u64 t0 = 0, d;

	if (static_branch_unlikely(&dma_cache_timing_key))
		t0 = local_clock();
	__x2000_mscr_set_pfb(bits);
	atomic_inc(&x2000_pfb_writes);
	if (t0) {
		d = local_clock() - t0;
		x2000_mscr_ns += d;
		x2000_mscr_timed++;
		if (d > x2000_mscr_max_ns)
			x2000_mscr_max_ns = d;
	}
}

static void x2000_pfb_hold(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	if (!x2000_pfb_holds++)
		__x2000_pfb_write(READ_ONCE(dma_cache_dis_pfb) | X2000_MSCR_PFB);
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
	__sync();
}

static void x2000_pfb_release(void)
{
	unsigned long flags;
	bool unbalanced = false;

	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	if (!x2000_pfb_holds)
		unbalanced = true;
	else if (!--x2000_pfb_holds)
		__x2000_pfb_write(READ_ONCE(dma_cache_dis_pfb));
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
	if (unbalanced)
		atomic_inc(&x2000_pfb_unbalanced);
}

/*
 * dma_cache.dis_pfb: the requested value, applied now. Only while booting
 * (the boot parameter): on the Q2, CCU MSCR prefetcher switches with both
 * CPUs running corrupted kernel memory (a BUG in __put_cred after about
 * 1,000 toggles, 2026-11-19). A write of the current value is accepted.
 */
int x2000_dma_set_dis_pfb(unsigned int v)
{
	if (v & ~X2000_MSCR_PFB)
		return -EINVAL;
	if (v == READ_ONCE(dma_cache_dis_pfb))
		return 0;
	if (system_state != SYSTEM_BOOTING)
		return -EPERM;
	WRITE_ONCE(dma_cache_dis_pfb, v);
	x2000_pfb_apply();
	return 0;
}
EXPORT_SYMBOL_GPL(x2000_dma_set_dis_pfb);

static int dma_dis_pfb_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	return ret ?: x2000_dma_set_dis_pfb(v);
}

static int dma_dis_pfb_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 readl(X2000_CCU_MSCR) & X2000_MSCR_PFB);
}

static const struct kernel_param_ops dma_dis_pfb_ops = {
	.set = dma_dis_pfb_set,
	.get = dma_dis_pfb_get,
};

static int dma_dis_pfb_state_get(char *buf, const struct kernel_param *kp)
{
	u32 m = readl(X2000_CCU_MSCR);

	return scnprintf(buf, PAGE_SIZE, "requested %u applied %u mscr 0x%08x applications %u resumes %u holds %u\n",
			 READ_ONCE(dma_cache_dis_pfb), m & X2000_MSCR_PFB, m,
			 READ_ONCE(x2000_pfb_applied),
			 READ_ONCE(x2000_pfb_resumed),
			 READ_ONCE(x2000_pfb_holds));
}

static const struct kernel_param_ops dma_dis_pfb_state_ops = {
	.get = dma_dis_pfb_state_get,
};

static int dma_pfb_set(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > 3)
		return -EINVAL;
	/* A switch around every transfer: refused, see x2000_dma_set_dis_pfb(). */
	if (v)
		return -EPERM;
	static_branch_disable(&dma_pfb_key);
	WRITE_ONCE(dma_cache_pfb, v);
	/* Drop every hold: releases of the old mode would never come. */
	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	x2000_pfb_holds = 0;
	__x2000_mscr_set_pfb(READ_ONCE(dma_cache_dis_pfb));
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
	if (v)
		static_branch_enable(&dma_pfb_key);
	return 0;
}

static const struct kernel_param_ops dma_pfb_ops = {
	.set = dma_pfb_set,
	.get = param_get_uint,
};

static int dma_pfb_stats_get(char *buf, const struct kernel_param *kp)
{
	unsigned long flags;
	u64 ns, max_ns;
	u32 timed, depth;

	raw_spin_lock_irqsave(&x2000_mscr_lock, flags);
	ns = x2000_mscr_ns;
	max_ns = x2000_mscr_max_ns;
	timed = x2000_mscr_timed;
	depth = x2000_pfb_holds;
	raw_spin_unlock_irqrestore(&x2000_mscr_lock, flags);
	return scnprintf(buf, PAGE_SIZE, "pfb %u holds %u pulses %u map_holds %u unmap_releases %u unbalanced %u depth %u mscr_writes %u timed %u ns %llu max_ns %llu\n",
			 READ_ONCE(dma_cache_pfb),
			 (unsigned int)atomic_read(&x2000_pfb_holds_n),
			 (unsigned int)atomic_read(&x2000_pfb_pulses),
			 (unsigned int)atomic_read(&x2000_pfb_maps),
			 (unsigned int)atomic_read(&x2000_pfb_unmaps),
			 (unsigned int)atomic_read(&x2000_pfb_unbalanced),
			 depth, (unsigned int)atomic_read(&x2000_pfb_writes),
			 timed, ns, max_ns);
}

static const struct kernel_param_ops dma_pfb_stats_ops = {
	.get = dma_pfb_stats_get,
};

module_param_cb(dis_pfb, &dma_dis_pfb_ops, &dma_cache_dis_pfb, 0644);
MODULE_PARM_DESC(dis_pfb, "CCU MSCR prefetcher switches, boot parameter only: 0 both on (default), 2 DisPFB2 (L2-DDR), 4 DisPFB1 (L1-L2), 6 both off; reads back MSCR");
module_param_cb(dis_pfb_state, &dma_dis_pfb_state_ops, NULL, 0444);
MODULE_PARM_DESC(dis_pfb_state, "dis_pfb requested and applied, MSCR, applications, resumes, pfb depth");
module_param_cb(pfb, &dma_pfb_ops, &dma_cache_pfb, 0644);
MODULE_PARM_DESC(pfb, "Device writes, both prefetchers off: 0 no change (default, the only value accepted: run-time MSCR switches corrupt memory on the Q2); 1 across the after-transfer passes, 2 a pulse before them, 3 from map to unmap");
module_param_cb(pfb_stats, &dma_pfb_stats_ops, NULL, 0444);
MODULE_PARM_DESC(pfb_stats, "dma_cache.pfb counts and MSCR write timing");

static inline bool dma_pfb_dir(enum dma_data_direction dir)
{
	return dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL;
}

static inline void dma_pfb_map(enum dma_data_direction dir)
{
	if (static_branch_unlikely(&dma_pfb_key) && dma_pfb_dir(dir) &&
	    READ_ONCE(dma_cache_pfb) == 3) {
		x2000_pfb_hold();
		atomic_inc(&x2000_pfb_maps);
	}
}

/* Before the after-transfer passes; returns whether a mode 1 hold began. */
static inline bool dma_pfb_begin(enum dma_data_direction dir)
{
	unsigned int mode;

	if (!static_branch_unlikely(&dma_pfb_key) || !dma_pfb_dir(dir))
		return false;
	mode = READ_ONCE(dma_cache_pfb);
	if (mode == 1) {
		x2000_pfb_hold();
		atomic_inc(&x2000_pfb_holds_n);
		return true;
	}
	if (mode == 2) {
		x2000_pfb_hold();
		x2000_pfb_release();
		atomic_inc(&x2000_pfb_pulses);
	}
	return false;
}

/* After the after-transfer passes: end a mode 1 hold or a mode 3 mapping. */
static inline void dma_pfb_end(enum dma_data_direction dir, bool held)
{
	if (held) {
		x2000_pfb_release();
	} else if (static_branch_unlikely(&dma_pfb_key) && dma_pfb_dir(dir) &&
		   READ_ONCE(dma_cache_pfb) == 3) {
		x2000_pfb_release();
		atomic_inc(&x2000_pfb_unmaps);
	}
}

static void x2000_pfb_resume(void *data)
{
	x2000_pfb_apply();
	WRITE_ONCE(x2000_pfb_resumed, x2000_pfb_resumed + 1);
}

static const struct syscore_ops x2000_pfb_syscore_ops = {
	.resume = x2000_pfb_resume,
};

static struct syscore x2000_pfb_syscore = {
	.ops = &x2000_pfb_syscore_ops,
};

static int __init x2000_pfb_init(void)
{
	x2000_pfb_apply();
	register_syscore(&x2000_pfb_syscore);
	pr_info("dma_cache: dis_pfb %u, CCU MSCR 0x%08x\n",
		READ_ONCE(dma_cache_dis_pfb), readl(X2000_CCU_MSCR));
	return 0;
}
early_initcall(x2000_pfb_init);
#else
static inline void dma_pfb_map(enum dma_data_direction dir) { }
static inline bool dma_pfb_begin(enum dma_data_direction dir)
{
	return false;
}
static inline void dma_pfb_end(enum dma_data_direction dir, bool held) { }
#endif /* CONFIG_MACH_X2000 */

#ifdef CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU
void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir)
{
	if (cpu_needs_post_dma_flush()) {
		/*
		 * Read once: a range that skips the passes is refreshed at the
		 * level that allowed the skip, whatever a parameter write does
		 * meanwhile.
		 */
		bool refresh = dma_refresh_on(dir);
		unsigned int level = refresh ? dma_refresh_level() : 0;
		bool skip = refresh && dma_refresh_skips_inv(paddr, size, level);
		bool held = dma_pfb_begin(dir);

		if (!skip) {
			dma_sync_phys(paddr, size, dir, false);
			dma_edge(paddr, size, dir, true);
		}
		dma_pfb_end(dir, held);
		if (refresh) {
			dma_verify(paddr, size, dir, false);
			dma_refresh_range(paddr, size, level, skip);
		} else {
			dma_verify(paddr, size, dir, true);
		}
	}
}
#endif

#ifdef CONFIG_ARCH_HAS_SETUP_DMA_OPS
void arch_setup_dma_ops(struct device *dev, bool coherent)
{
	dev_assign_dma_coherent(dev, coherent);
}
#endif
