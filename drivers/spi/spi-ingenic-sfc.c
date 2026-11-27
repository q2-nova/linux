// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ingenic X2000 serial flash controller, command descriptor table mode.
 *
 * Register and FIFO protocol from the Q2 vendor kernel and Ingenic's
 * ingenic_sfc_v2 driver (Copyright 2015 Ingenic, Xiaoyang Fu).
 * Data phases use interrupt-driven PIO or one DMA descriptor; status
 * polling runs in the controller. Module parameters select each path.
 *
 * Production read clock (read_prod, default on): SPI NAND read sequences
 * (PAGE READ, its status polls and READ FROM CACHE) of every task run at
 * read_hz/read_delay (default 75 MHz, sample delay 4) once a boot check has
 * read the same pages of the check_part partition at the device tree rate
 * and at that pair and found them equal. Sampling errors on the wire come
 * after the flash's on-die ECC and would be silent; until the check passes,
 * and for the rest of the boot after it fails, reads stay at the device
 * tree rate. A production-rate sequence that times out, fails or reports an
 * uncorrectable ECC status turns the fast rate off for the boot and is read
 * again at the device tree rate. The pair is the one read_hz/read_delay held
 * when the check ran (at most 80 MHz: tCS); later writes of them change
 * only the bench rate. The read_check parameter shows the state, the pair
 * and the counts; a failed check and a fallback each log one warning.
 * Every other operation, writes and erases included, runs at the device
 * tree rate with sample delay 0.
 *
 * Read experiments (all off by default, switchable at run time):
 * - read_tgid: read_hz/read_delay at their current values for read
 *   sequences of a row in [read_row_lo, read_row_hi) issued by the kernel
 *   thread read_tgid (the skipbad read bench), whatever the production
 *   state; its other reads follow the production rule.
 * - chain: PAGE READ is issued with the following status poll as one CDT
 *   chain (vendor NAND_QUAD_READ_TO_CACHE + GET_FEATURE entries).
 * - spin_us: busy-wait up to this long for the end of an operation before
 *   sleeping on the interrupt.
 * - dma_direct: DMA reads straight into a cache-line aligned kernel buffer;
 *   refused after any timeout or failed stop since boot.
 * - timing: per-phase timing, read from the stats parameter.
 * - dma_min: data phases shorter than dma_min bytes use PIO (as with dma 0);
 *   the 64-byte floor stays. stats line "path" counts data phases by path.
 * - uc_bounce: a DMA read into the bounce page is unmapped without CPU
 *   cache maintenance and copied to the caller from the page's uncached
 *   KSEG1 alias (MSA ld.d/st.d where kernel_msa_begin() allows and the
 *   destination is 16-byte aligned, else 32-bit loads and stores, single
 *   bytes for a tail); bytes per copy path in stats.
 *
 * Fault counts (timeouts, SR overrun/underrun, a controller still busy after
 * TRIG_STOP) are kept for the boot and shown in the stats parameter; a stats
 * reset does not clear them.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mtd/mtd.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/task_stack.h>
#include <linux/slab.h>
#include <linux/spi/spi-mem.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#ifdef CONFIG_MIPS
#include <linux/dma-direct.h>
#include <asm/addrspace.h>
#include <asm/kernel-msa.h>
#endif

#define SFC_GLB		0x0000
#define SFC_DEV_CONF	0x0004
#define SFC_TRAN_LEN	0x002c
#define SFC_MEM_ADDR	0x0060
#define SFC_TRIG	0x0064
#define SFC_SR		0x0068
#define SFC_SCR		0x006c
#define SFC_INTC	0x0070
#define SFC_CGE		0x0078
#define SFC_CMD_IDX	0x007c
#define SFC_COL_ADDR	0x0080
#define SFC_ROW_ADDR	0x0084
#define SFC_STA_ADDR0	0x0088
#define SFC_DES_ADDR	0x0090
#define SFC_GLB1	0x0094
#define SFC_CDT		0x0800
#define SFC_FIFO	0x1000

/* GLB0 fields: vendor soc-x2000/include/soc/sfc.h. */
#define GLB_POLL_TIME(n)	((n) << 16)
#define GLB_DES_EN	BIT(15)
#define GLB_CDT_EN	BIT(14)
#define GLB_THRESHOLD(n)	((n) << 7)
#define GLB_OP_DMA	BIT(6)
#define GLB_BURST(n)	(n)

/* DEV_CONF: idle CS/HOLD/WP high, maximum hold/setup/CS-high delays
 * (vendor THOLD 11, TSETUP 9, TSH 5 fields); sample delay in 20:16
 * (vendor DEV_CONF_SMP_DELAY: 0-7 are 0-315 degrees in 45-degree steps).
 */
#define DEV_CONF_BASE	((3 << 11) | (3 << 9) | (15 << 5) | 7)
#define DEV_CONF_SMP_DELAY(n)	((n) << 16)
#define SMP_DELAY_MAX	31

/* CDT entry word 0 (link) and word 1 (transfer). */
#define CDT_LINK	BIT(31)
#define CDT_MODE(n)	((n) << 4)
#define CDT_ADDR_COL	0
#define CDT_ADDR_ROW	1
#define CDT_ADDR_STA0	2
#define MODE_STD	0
#define MODE_QUAD_DATA	5
#define XFER_ADDR_BYTES(n)	((n) << 26)
#define XFER_POLL	BIT(25)
#define XFER_CMD	BIT(24)
#define XFER_DUMMY_BITS(n)	((n) << 17)
#define XFER_DATA	BIT(16)
#define IDX_DATA	BIT(31)
#define IDX_WRITE	BIT(30)

#define TRIG_START	BIT(0)
#define TRIG_STOP	BIT(1)
#define TRIG_FLUSH	BIT(2)
#define SR_UNDER	BIT(0)
#define SR_OVER		BIT(1)
#define SR_ERRORS	GENMASK(1, 0)
#define SR_RREQ		BIT(2)
#define SR_TREQ		BIT(3)
#define SR_END		BIT(4)
#define SR_ALL		GENMASK(4, 0)
/* Vendor soc-x2000 sfc.h SFC_BUSY; sfc_stop() polls it after TRIG_STOP. */
#define SR_BUSY		GENMASK(6, 5)
/* Upper bound of that poll (the vendor loops 0xffff register reads). */
#define STOP_TIMEOUT_US	10000
#define FIFO_WORDS	32

#define MAX_DATA	4096U
/* Shorter data phases (status, IDs) stay on PIO. */
#define DMA_MIN_BYTES	64
/* Vendor sfc_poll_time(0x10) and sfc_init() burst mode 2. */
#define POLL_TIME	0x10
#define DMA_BURST	2

/* SPI NAND opcodes (DS35Q1GA Rev.03 Table 1.3). */
#define OP_PAGE_READ	0x13
#define OP_GET_FEATURE	0x0f
/* DS35Q1GA Rev.03 Table 4.7: fC 104 MHz. */
#define READ_HZ_MAX	104000000U
/* DS35Q1GA Rev.03 Table 4.7: tCS (CS# deselect) 100 ns minimum. The longest
 * CS# high time of DEV_CONF is 8 SPI clocks (TSH 15): 107 ns at 75 MHz,
 * 85 ns at 93.75 MHz. Production reads stay at or below 80 MHz.
 */
#define PROD_HZ_MAX	80000000U
/* Status register C0h: OIP bit 0, ECC_S bits 5:4, 10b uncorrectable
 * (Table 3.6; 11b is reserved and treated the same).
 */
#define REG_STATUS	0xc0
#define STATUS_OIP	BIT(0)
#define STATUS_ECC	GENMASK(5, 4)
#define STATUS_ECC_UNCOR	(2 << 4)
/* spinand's SPINAND_WAITRDY_TIMEOUT_MS. */
#define WAITRDY_MS	400
/* Boot check: the first CHECK_HEAD bytes of the partition's good blocks and
 * CHECK_SPREAD pages spread over the rest of its squashfs image; it passes
 * only with at least CHECK_MIN pages compared.
 */
#define CHECK_HEAD	(2U << 20)
#define CHECK_SPREAD	64U
#define CHECK_MIN	512
/* Pages per read of the head: one clock change per run. */
#define CHECK_RUN	16U

static bool sfc_dma = true;
module_param_named(dma, sfc_dma, bool, 0644);
MODULE_PARM_DESC(dma, "DMA for reads of 64 or more bytes (default 1; 0: interrupt-driven PIO)");

static unsigned int sfc_dma_min;
module_param_named(dma_min, sfc_dma_min, uint, 0644);
MODULE_PARM_DESC(dma_min, "Data phases shorter than this many bytes use PIO (default 0: only the 64-byte floor)");

static bool sfc_dma_write;
module_param_named(dma_write, sfc_dma_write, bool, 0644);
MODULE_PARM_DESC(dma_write, "DMA for writes as well, when dma is set (default 0)");

static bool sfc_hw_poll = true;
module_param_named(hw_poll, sfc_hw_poll, bool, 0644);
MODULE_PARM_DESC(hw_poll, "Poll flash status in the controller (default 1; 0: spi-mem polls by command)");

static bool sfc_chain;
module_param_named(chain, sfc_chain, bool, 0644);
MODULE_PARM_DESC(chain, "Issue PAGE READ and its status poll as one CDT chain (needs hw_poll; default 0)");

static unsigned int sfc_spin_us;
module_param_named(spin_us, sfc_spin_us, uint, 0644);
MODULE_PARM_DESC(spin_us, "Busy-wait up to this many microseconds before sleeping on the interrupt (default 0)");

static bool sfc_dma_direct;
static int sfc_dma_direct_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops sfc_dma_direct_ops = {
	.set = sfc_dma_direct_set,
	.get = param_get_bool,
};
module_param_cb(dma_direct, &sfc_dma_direct_ops, &sfc_dma_direct, 0644);
MODULE_PARM_DESC(dma_direct, "DMA reads into cache-line aligned kernel buffers without the bounce copy (default 0; refused after a timeout or failed stop)");

/*
 * uc_bounce: the CPU never reads the bounce page's cached alias after a DMA
 * read, so the unmap skips the cache passes (and dma_cache.refresh) for it.
 * Stale copies of the bounce lines that stay in the cache are clean (the
 * CPU stores to the bounce only for writes, and the TO_DEVICE map writes
 * them back), the next map of the bounce invalidates them, and the cached
 * view is not read in between. The uncached read costs about 80 ns per
 * access whatever its width on the Q2, so MSA ld.d (16 bytes) is the fast
 * path.
 */
static bool sfc_uc_bounce = true;
module_param_named(uc_bounce, sfc_uc_bounce, bool, 0644);
MODULE_PARM_DESC(uc_bounce, "Copy DMA reads from the bounce page's uncached alias, with no cache maintenance at the unmap (default 1)");

static bool sfc_timing;
module_param_named(timing, sfc_timing, bool, 0644);
MODULE_PARM_DESC(timing, "Record per-phase operation times for the stats parameter (default 0)");

static int sfc_read_hz_set(const char *val, const struct kernel_param *kp)
{
	unsigned int hz;
	int ret = kstrtouint(val, 0, &hz);

	if (ret)
		return ret;
	if (hz > READ_HZ_MAX)
		return -EINVAL;
	WRITE_ONCE(*(unsigned int *)kp->arg, hz);
	return 0;
}

static const struct kernel_param_ops sfc_read_hz_ops = {
	.set = sfc_read_hz_set,
	.get = param_get_uint,
};

static unsigned int sfc_read_hz = 75000000;
module_param_cb(read_hz, &sfc_read_hz_ops, &sfc_read_hz, 0644);
MODULE_PARM_DESC(read_hz, "SPI rate for NAND read sequences, at most 104 MHz; production takes the boot check's value, at most 80 MHz (default 75000000; 0: off)");

static int sfc_read_delay_set(const char *val, const struct kernel_param *kp)
{
	unsigned int delay;
	int ret = kstrtouint(val, 0, &delay);

	if (ret)
		return ret;
	if (delay > SMP_DELAY_MAX)
		return -EINVAL;
	WRITE_ONCE(*(unsigned int *)kp->arg, delay);
	return 0;
}

static const struct kernel_param_ops sfc_read_delay_ops = {
	.set = sfc_read_delay_set,
	.get = param_get_uint,
};

static unsigned int sfc_read_delay = 4;
module_param_cb(read_delay, &sfc_read_delay_ops, &sfc_read_delay, 0644);
MODULE_PARM_DESC(read_delay, "DEV_CONF sample delay for NAND read sequences, 0-31 (default 4)");

static bool sfc_read_prod = true;
module_param_named(read_prod, sfc_read_prod, bool, 0644);
MODULE_PARM_DESC(read_prod, "Every task's NAND read sequences at the pair the boot check passed (default 1; 0: device tree rate)");

static char sfc_check_part[32] = "rootfs";
module_param_string(check_part, sfc_check_part, sizeof(sfc_check_part), 0444);
MODULE_PARM_DESC(check_part, "MTD partition the boot check reads (default rootfs)");

static unsigned int sfc_read_row_lo, sfc_read_row_hi;
module_param_named(read_row_lo, sfc_read_row_lo, uint, 0644);
MODULE_PARM_DESC(read_row_lo, "First NAND row (page) of the armed read window");
module_param_named(read_row_hi, sfc_read_row_hi, uint, 0644);
MODULE_PARM_DESC(read_row_hi, "Row after the last of the armed read window");

static int sfc_read_tgid;
module_param_named(read_tgid, sfc_read_tgid, int, 0644);
MODULE_PARM_DESC(read_tgid, "Arm read_hz/read_delay for this kernel thread (the mtdskip bench; default 0: off)");

/* DMA descriptor: vendor struct sfc_desc. */
struct ingenic_sfc_desc {
	__le32 next;	/* physical address of the next descriptor */
	__le32 addr;	/* physical memory address */
	__le32 len;	/* bytes */
	__le32 link;	/* 1: continue at next */
};

/* Rate of an operation. RATE_SAFE is 0: a closed read window. */
enum ingenic_sfc_rate {
	RATE_SAFE,	/* device tree rate, sample delay 0 */
	RATE_BENCH,	/* read_hz/read_delay now: the armed bench thread */
	RATE_PROD,	/* prod_hz/prod_delay: the pair of the boot check */
};

/* Boot check and production state. */
enum ingenic_sfc_check_state {
	CHECK_PENDING,	/* the partition has not appeared */
	CHECK_RUNNING,
	CHECK_PASSED,	/* production reads at prod_hz/prod_delay */
	CHECK_FAILED,
	CHECK_OFF,	/* read_hz 0 or above PROD_HZ_MAX at the check */
	CHECK_FALLBACK,	/* passed, then a production read failed */
};

static const char * const ingenic_sfc_check_names[] = {
	"pending", "running", "passed", "failed", "off", "fallback",
};

enum ingenic_sfc_class {
	CLS_PAGE_READ,	/* 0x13 alone */
	CLS_CHAIN,	/* 0x13 and the status poll, one chain */
	CLS_POLL,	/* status poll in the controller */
	CLS_STATUS,	/* GET FEATURE command (spi-mem poll loop, IDs) */
	CLS_CACHE,	/* READ FROM CACHE */
	CLS_OTHER,	/* writes, erases, SET FEATURE, reset, ID */
	CLS_COUNT,
};

static const char * const ingenic_sfc_class_names[CLS_COUNT] = {
	"pageread", "chain", "poll", "status", "cache", "other",
};

struct ingenic_sfc_class_stats {
	u32 n, fast, spun, errors;
	u64 setup, hw, wake, post, total, wire, max;
};

struct ingenic_sfc_stats {
	struct ingenic_sfc_class_stats c[CLS_COUNT];
	u32 clock_changes, flushes, direct;
	/* bytes copied out of the bounce page after DMA reads, by path */
	u64 bounce_cached, uc_msa, uc_word, uc_byte;
	/* data phases by path; pio_min: PIO because of dma_min */
	u32 path_dma, path_pio, path_pio_min;
};

/* Since boot; the stats reset leaves these. Updated from the interrupt too. */
struct ingenic_sfc_faults {
	atomic_t timeouts, over, under, stop_busy;
	/* Normal ends whose SR still showed BUSY right after TRIG_STOP. */
	atomic_t end_busy;
};

struct ingenic_sfc {
	struct device *dev;
	struct spi_controller *ctlr;
	void __iomem *base;
	struct clk *clk;
	struct completion done;
	/* Serializes FIFO service between the interrupt and a spinning caller. */
	spinlock_t lock;
	bool ended;
	/* PIO data phase: exactly one of in/out is set while pio_len > 0. */
	u8 *pio_in;
	const u8 *pio_out;
	unsigned int pio_len;
	unsigned int pos;
	unsigned int freq;	/* SPI rate last set in the CGU, 0 if none */
	unsigned long actual_hz;	/* CGU rate / 4 at that setting */
	unsigned long fast_hz;	/* actual_hz of the last armed operation */
	unsigned int safe_hz;	/* device tree rate of the last operation */
	unsigned int delay;	/* sample delay of the current operation */
	/* Read window: rate, row and task of the last PAGE READ. */
	enum ingenic_sfc_rate window;
	u32 window_row;
	struct task_struct *window_task;
	/* PAGE READ held for the following status poll (chain). */
	bool deferred;
	u8 deferred_opcode;
	u32 deferred_row;
	/* Production pair, set before the check's first read. */
	unsigned int prod_hz, prod_delay;
	int check_state;
	/* The check thread and the rate of its next read. */
	struct task_struct *check_task;
	bool check_fast;
	struct mtd_info *check_mtd;
	u32 check_pages, check_mismatches, check_errors, check_skipped, check_ms;
	unsigned long check_hz;	/* actual rate of the check's fast reads */
	atomic_t fallbacks, retries;
	/* DMA goes through this cached bounce page unless dma_direct. */
	void *bounce;
	/* The controller did not go idle after TRIG_STOP: no more DMA. */
	bool dma_dead;
	struct ingenic_sfc_desc *desc;
	dma_addr_t desc_dma;
	int irq;
	int error;
	/* Times of the current operation (local_clock ns), when timing. */
	u64 t_trig, t_end, t_wake;
	bool spun;
	spinlock_t stats_lock;
	struct ingenic_sfc_stats stats;
	struct ingenic_sfc_faults faults;
};

/* The Q2 has one SFC; the stats and features parameters report it. */
static struct ingenic_sfc *sfc_instance;

/* Caller holds sfc->lock. Returns true once the operation has ended. */
static bool ingenic_sfc_service(struct ingenic_sfc *sfc, u32 status)
{
	unsigned int words;

	writel(status, sfc->base + SFC_SCR);
	if (status & SR_ERRORS) {
		if (status & SR_OVER)
			atomic_inc(&sfc->faults.over);
		if (status & SR_UNDER)
			atomic_inc(&sfc->faults.under);
		sfc->error = -EIO;
		goto done;
	}
	if (status & (SR_RREQ | SR_TREQ)) {
		if (sfc->pos >= sfc->pio_len) {
			sfc->error = -EIO;
			goto done;
		}
		for (words = 0; words < FIFO_WORDS && sfc->pos < sfc->pio_len;
		     words++) {
			unsigned int n = min(4U, sfc->pio_len - sfc->pos);
			u32 value = 0;
			unsigned int i;

			if (sfc->pio_in) {
				value = readl(sfc->base + SFC_FIFO);
				for (i = 0; i < n; i++)
					sfc->pio_in[sfc->pos + i] = value >> (8 * i);
			} else {
				for (i = 0; i < n; i++)
					value |= (u32)sfc->pio_out[sfc->pos + i] << (8 * i);
				writel(value, sfc->base + SFC_FIFO);
			}
			sfc->pos += n;
		}
	}
	if (!(status & SR_END))
		return false;
	if (sfc->pos != sfc->pio_len)
		sfc->error = -EIO;
 done:
	if (READ_ONCE(sfc_timing))
		sfc->t_end = local_clock();
	sfc->ended = true;
	writel(SR_ALL, sfc->base + SFC_INTC);
	return true;
}

static irqreturn_t ingenic_sfc_irq(int irq, void *data)
{
	struct ingenic_sfc *sfc = data;
	irqreturn_t ret = IRQ_NONE;
	u32 status;

	spin_lock(&sfc->lock);
	status = readl(sfc->base + SFC_SR) &
		 ~readl(sfc->base + SFC_INTC) & SR_ALL;
	if (status && !sfc->ended) {
		ret = IRQ_HANDLED;
		if (ingenic_sfc_service(sfc, status))
			complete(&sfc->done);
	}
	spin_unlock(&sfc->lock);
	return ret;
}

static bool ingenic_sfc_supports_op(struct spi_mem *mem,
				    const struct spi_mem_op *op)
{
	/* Mode 0, SDR, single-bit command/address, single or quad data. */
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1 ||
	    op->addr.nbytes > 4 || (op->addr.nbytes && op->addr.buswidth != 1) ||
	    (op->dummy.nbytes && op->dummy.buswidth != 1) ||
	    op->dummy.nbytes > 7 ||
	    (op->data.nbytes && op->data.buswidth != 1 && op->data.buswidth != 4))
		return false;

	return spi_mem_default_supports_op(mem, op);
}

static int ingenic_sfc_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	op->data.nbytes = min(op->data.nbytes, MAX_DATA);
	return 0;
}

static enum ingenic_sfc_class ingenic_sfc_classify(const struct spi_mem_op *op)
{
	if (op->cmd.opcode == OP_PAGE_READ && op->addr.nbytes == 3 &&
	    !op->data.nbytes)
		return CLS_PAGE_READ;
	if (op->cmd.opcode == OP_GET_FEATURE && op->data.nbytes == 1 &&
	    op->data.dir == SPI_MEM_DATA_IN)
		return CLS_STATUS;
	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN &&
	    (op->cmd.opcode == 0x03 || op->cmd.opcode == 0x0b ||
	     op->cmd.opcode == 0x3b || op->cmd.opcode == 0x6b))
		return CLS_CACHE;
	return CLS_OTHER;
}

/*
 * The calling task may use read_hz/read_delay as they are now: only the
 * kernel thread named by read_tgid (the skipbad bench). A kernel thread's
 * reads land in its own buffer; a user task's could fill a page cache
 * another reader trusts.
 */
static bool ingenic_sfc_armed_task(void)
{
	int tgid = READ_ONCE(sfc_read_tgid);

	if (tgid <= 0 || !READ_ONCE(sfc_read_hz))
		return false;
	return (current->flags & PF_KTHREAD) && task_tgid_nr(current) == tgid;
}

static bool ingenic_sfc_armed_row(u32 row)
{
	return row >= READ_ONCE(sfc_read_row_lo) &&
	       row < READ_ONCE(sfc_read_row_hi);
}

/*
 * Rate of a read sequence the current task opens with a PAGE READ of row:
 * the check thread's as it asks; the bench's own rows; then production.
 */
static enum ingenic_sfc_rate ingenic_sfc_open_rate(struct ingenic_sfc *sfc,
						   u32 row)
{
	if (current == READ_ONCE(sfc->check_task))
		return READ_ONCE(sfc->check_fast) ? RATE_PROD : RATE_SAFE;
	if (ingenic_sfc_armed_task() && ingenic_sfc_armed_row(row))
		return RATE_BENCH;
	if (READ_ONCE(sfc_read_prod) &&
	    READ_ONCE(sfc->check_state) == CHECK_PASSED)
		return RATE_PROD;
	return RATE_SAFE;
}

/* Rate of a status read or cache read: the window's, for the task that
 * opened it.
 */
static enum ingenic_sfc_rate ingenic_sfc_window_rate(struct ingenic_sfc *sfc)
{
	if (current != sfc->window_task)
		return RATE_SAFE;
	if (sfc->window == RATE_BENCH && !ingenic_sfc_armed_task())
		return RATE_SAFE;
	return sfc->window;
}

/* GET FEATURE of the status register, the only one read fast. */
static bool ingenic_sfc_status_op(const struct spi_mem_op *op)
{
	return op->addr.nbytes == 1 && op->addr.val == REG_STATUS;
}

/* A production-rate read failed; the check thread's reads are its own. */
static bool ingenic_sfc_prod(struct ingenic_sfc *sfc, enum ingenic_sfc_rate rate)
{
	return rate == RATE_PROD && current != READ_ONCE(sfc->check_task);
}

/* An error, or a ready status with an uncorrectable ECC status. */
static bool ingenic_sfc_status_failed(int ret, const u8 *status)
{
	return ret || (!(*status & STATUS_OIP) &&
		       (*status & STATUS_ECC) >= STATUS_ECC_UNCOR);
}

/* The fast rate is off for the rest of the boot, the current window
 * included; one warning.
 */
static void ingenic_sfc_fallback(struct ingenic_sfc *sfc, const char *what,
				 int err)
{
	sfc->window = RATE_SAFE;
	atomic_inc(&sfc->fallbacks);
	if (cmpxchg(&sfc->check_state, CHECK_PASSED, CHECK_FALLBACK) ==
	    CHECK_PASSED)
		dev_warn(sfc->dev, "NAND reads at %u Hz back to the device tree rate for this boot: %s (%d) at row %u\n",
			 sfc->prod_hz, what, err, sfc->window_row);
}

/* Set the SPI rate and sample delay for one operation. */
static int ingenic_sfc_clock(struct ingenic_sfc *sfc, enum ingenic_sfc_rate fast)
{
	unsigned int freq = sfc->safe_hz, delay = 0;
	unsigned long rate;
	int ret;

	if (fast == RATE_BENCH) {
		freq = READ_ONCE(sfc_read_hz);
		delay = READ_ONCE(sfc_read_delay);
	} else if (fast == RATE_PROD) {
		freq = sfc->prod_hz;
		delay = sfc->prod_delay;
	}
	if (!freq) {
		freq = sfc->safe_hz;
		delay = 0;
	}
	sfc->delay = delay;
	/* The rate stays set between operations; only a change reprograms it. */
	if (freq != sfc->freq) {
		sfc->freq = 0;
		/* The controller divides its CGU clock by four (vendor timing code). */
		ret = clk_set_rate(sfc->clk, (unsigned long)freq * 4);
		if (ret)
			return ret;
		rate = clk_get_rate(sfc->clk);
		if (!rate || rate > (unsigned long)freq * 4)
			return -EINVAL;
		sfc->freq = freq;
		sfc->actual_hz = rate / 4;
		spin_lock(&sfc->stats_lock);
		sfc->stats.clock_changes++;
		spin_unlock(&sfc->stats_lock);
	}
	if (fast)
		sfc->fast_hz = sfc->actual_hz;
	return 0;
}

static void ingenic_sfc_setup(struct ingenic_sfc *sfc, u32 glb)
{
	writel(TRIG_STOP, sfc->base + SFC_TRIG);
	writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
	writel(SR_ALL, sfc->base + SFC_SCR);
	writel(glb, sfc->base + SFC_GLB);
	writel(DEV_CONF_BASE | DEV_CONF_SMP_DELAY(sfc->delay),
	       sfc->base + SFC_DEV_CONF);
	writel(0, sfc->base + SFC_GLB1);
}

/* Any timeout or failed stop since boot keeps direct DMA off. */
static bool ingenic_sfc_faulted(struct ingenic_sfc *sfc)
{
	return atomic_read(&sfc->faults.timeouts) ||
	       atomic_read(&sfc->faults.stop_busy) || READ_ONCE(sfc->dma_dead);
}

static int sfc_dma_direct_set(const char *val, const struct kernel_param *kp)
{
	struct ingenic_sfc *sfc = READ_ONCE(sfc_instance);
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	if (on && sfc && ingenic_sfc_faulted(sfc))
		return -EBUSY;
	WRITE_ONCE(sfc_dma_direct, on);
	return 0;
}

/*
 * TRIG_STOP. After a timeout or an error: wait for the controller to leave
 * its busy states before the caller unmaps or reuses a DMA buffer (vendor
 * sfc_stop). A controller that stays busy may still write the buffer: DMA
 * is not used again. After a normal end, one status read is counted only.
 */
static void ingenic_sfc_stop(struct ingenic_sfc *sfc, bool wait)
{
	u32 sr;

	writel(TRIG_STOP, sfc->base + SFC_TRIG);
	if (!wait) {
		if (readl(sfc->base + SFC_SR) & SR_BUSY)
			atomic_inc(&sfc->faults.end_busy);
		return;
	}
	if (readl_poll_timeout_atomic(sfc->base + SFC_SR, sr, !(sr & SR_BUSY),
				      1, STOP_TIMEOUT_US)) {
		atomic_inc(&sfc->faults.stop_busy);
		if (!READ_ONCE(sfc->dma_dead))
			dev_err(sfc->dev, "controller busy after stop (SR %08x); DMA off\n", sr);
		WRITE_ONCE(sfc->dma_dead, true);
	}
}

static void ingenic_sfc_cdt(struct ingenic_sfc *sfc, unsigned int entry,
			    u32 link, u32 xfer, u32 expect, u32 mask)
{
	void __iomem *cdt = sfc->base + SFC_CDT + entry * 16;

	writel(link, cdt);
	writel(xfer, cdt + 4);
	writel(expect, cdt + 8);
	writel(mask, cdt + 12);
}

static int ingenic_sfc_run(struct ingenic_sfc *sfc, u32 enabled,
			   unsigned long timeout_ms)
{
	unsigned int spin_us = READ_ONCE(sfc_spin_us);
	bool timing = READ_ONCE(sfc_timing);
	unsigned long flags;
	int ret = 0;

	sfc->pos = 0;
	sfc->error = 0;
	sfc->ended = false;
	sfc->spun = false;
	sfc->t_end = 0;
	reinit_completion(&sfc->done);
	if (timing)
		sfc->t_trig = local_clock();
	if (spin_us) {
		u64 deadline;

		/* Interrupts masked; the caller serves the status itself. */
		writel(SR_ALL, sfc->base + SFC_INTC);
		writel(TRIG_START, sfc->base + SFC_TRIG);
		deadline = local_clock() + (u64)spin_us * NSEC_PER_USEC;
		do {
			u32 status = readl(sfc->base + SFC_SR) & enabled;

			if (status) {
				spin_lock_irqsave(&sfc->lock, flags);
				if (!sfc->ended)
					ingenic_sfc_service(sfc, status);
				spin_unlock_irqrestore(&sfc->lock, flags);
				if (sfc->ended)
					break;
			}
			cpu_relax();
		} while (local_clock() < deadline);
		spin_lock_irqsave(&sfc->lock, flags);
		sfc->spun = sfc->ended;
		/* Pending status raises the (level) interrupt once unmasked. */
		if (!sfc->ended)
			writel(SR_ALL & ~enabled, sfc->base + SFC_INTC);
		spin_unlock_irqrestore(&sfc->lock, flags);
	} else {
		writel(SR_ALL & ~enabled, sfc->base + SFC_INTC);
		writel(TRIG_START, sfc->base + SFC_TRIG);
	}
	if (!sfc->spun)
		ret = wait_for_completion_timeout(&sfc->done,
						  msecs_to_jiffies(timeout_ms) + 1) ?
		      0 : -ETIMEDOUT;
	if (timing)
		sfc->t_wake = local_clock();
	writel(SR_ALL, sfc->base + SFC_INTC);
	/* Completion is the handler's last access to transfer state. Only a
	 * timeout needs to drain a handler that might still be using it.
	 */
	if (ret) {
		synchronize_irq(sfc->irq);
		atomic_inc(&sfc->faults.timeouts);
	}
	ingenic_sfc_stop(sfc, ret || sfc->error);
	writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
	writel(SR_ALL, sfc->base + SFC_SCR);
	/* Disarm: no descriptor or DMA mode survives the operation. */
	writel(readl(sfc->base + SFC_GLB) & ~(GLB_OP_DMA | GLB_DES_EN),
	       sfc->base + SFC_GLB);
	sfc->desc->len = 0;
	sfc->pio_in = NULL;
	sfc->pio_out = NULL;
	sfc->pio_len = 0;
	return ret ?: sfc->error;
}

/* Clocks on the wire for one command at the configured widths. */
static u32 ingenic_sfc_clocks(const struct spi_mem_op *op)
{
	u32 clocks = 8 * (op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes);

	if (op->data.nbytes)
		clocks += op->data.nbytes * 8 / (op->data.buswidth ?: 1);
	return clocks;
}

static void ingenic_sfc_account(struct ingenic_sfc *sfc,
				enum ingenic_sfc_class cls, u64 t0,
				u32 clocks, bool fast, int ret)
{
	struct ingenic_sfc_class_stats *c = &sfc->stats.c[cls];
	u64 t3 = t0 ? local_clock() : 0;

	spin_lock(&sfc->stats_lock);
	c->n++;
	c->fast += fast;
	c->spun += sfc->spun;
	c->errors += !!ret;
	if (t0 && sfc->t_trig >= t0 && sfc->t_wake >= sfc->t_trig) {
		u64 end = sfc->t_end ?: sfc->t_wake;

		c->setup += sfc->t_trig - t0;
		c->hw += end - sfc->t_trig;
		c->wake += sfc->t_wake - end;
		c->post += t3 - sfc->t_wake;
		c->total += t3 - t0;
		c->max = max(c->max, t3 - t0);
		if (sfc->actual_hz)
			c->wire += div_u64((u64)clocks * NSEC_PER_SEC,
					   sfc->actual_hz);
	}
	spin_unlock(&sfc->stats_lock);
}

static bool ingenic_sfc_use_dma(struct ingenic_sfc *sfc,
				const struct spi_mem_op *op)
{
	/* Vendor DMA transfers whole 32-bit words (ingenic_sfcnand_read). */
	if (!READ_ONCE(sfc_dma) || op->data.nbytes < DMA_MIN_BYTES ||
	    op->data.nbytes % 4 || READ_ONCE(sfc->dma_dead))
		return false;
	return op->data.dir == SPI_MEM_DATA_IN || READ_ONCE(sfc_dma_write);
}

/* DMA would be used but dma_min selects PIO. */
static bool ingenic_sfc_below_dma_min(const struct spi_mem_op *op)
{
	return op->data.nbytes < READ_ONCE(sfc_dma_min);
}

/* A read buffer the controller may fill directly: linear-mapped, not on a
 * stack, whole cache lines.
 */
static bool ingenic_sfc_direct(struct ingenic_sfc *sfc,
			       const struct spi_mem_op *op)
{
	unsigned long align = dma_get_cache_alignment();
	const void *buf = op->data.buf.in;

	return READ_ONCE(sfc_dma_direct) && !ingenic_sfc_faulted(sfc) &&
	       op->data.dir == SPI_MEM_DATA_IN &&
	       virt_addr_valid(buf) && !object_is_on_stack(buf) &&
	       IS_ALIGNED((unsigned long)buf, align) &&
	       IS_ALIGNED(op->data.nbytes, align);
}

/*
 * The bounce page has a KSEG1 alias (below 512 MiB physical) and the mapping
 * is the page itself, not a swiotlb slot whose copy-back the skipped sync
 * would omit.
 */
static bool ingenic_sfc_uc_ok(struct ingenic_sfc *sfc, dma_addr_t addr)
{
#ifdef CONFIG_MIPS
	return __pa(sfc->bounce) + MAX_DATA <= 0x20000000UL &&
	       addr == phys_to_dma(sfc->dev, __pa(sfc->bounce));
#else
	return false;
#endif
}

#if defined(CONFIG_MIPS) && defined(CONFIG_CPU_HAS_MSA)
/* n bytes (a multiple of 64): four uncached ld.d, then four st.d, per 64. */
static void ingenic_sfc_copy_msa(void *dst, const void *src, unsigned int n)
{
	unsigned long d = (unsigned long)dst, s = (unsigned long)src;
	unsigned long e = d + n;

	__asm__ __volatile__(
		".set push\n\t.set reorder\n\t.set noat\n\t"
		".set hardfloat\n\t.set fp=64\n\t.set msa\n\t"
		"1:\n\t"
		"ld.d $w0, 0(%[s])\n\t"
		"ld.d $w1, 16(%[s])\n\t"
		"ld.d $w2, 32(%[s])\n\t"
		"ld.d $w3, 48(%[s])\n\t"
		"addiu %[s], %[s], 64\n\t"
		"st.d $w0, 0(%[d])\n\t"
		"st.d $w1, 16(%[d])\n\t"
		"st.d $w2, 32(%[d])\n\t"
		"st.d $w3, 48(%[d])\n\t"
		"addiu %[d], %[d], 64\n\t"
		"sltu $1, %[d], %[e]\n\t"
		"bnez $1, 1b\n\t"
		".set pop\n\t"
		: [d] "+r" (d), [s] "+r" (s)
		: [e] "r" (e)
		: "$1", "memory");
}
#endif

/* len bytes from the bounce page's KSEG1 alias to dst. */
static void ingenic_sfc_copy_uc(struct ingenic_sfc *sfc, u8 *dst,
				unsigned int len)
{
#ifdef CONFIG_MIPS
	const u8 *src = (const u8 *)CKSEG1ADDR(__pa(sfc->bounce));
	unsigned int done = 0, msa = 0, word;

#ifdef CONFIG_CPU_HAS_MSA
	if (len >= 64 && IS_ALIGNED((unsigned long)dst, 16) &&
	    kernel_msa_begin()) {
		msa = len & ~63U;
		ingenic_sfc_copy_msa(dst, src, msa);
		kernel_msa_end();
	}
#endif
	if (IS_ALIGNED((unsigned long)dst, 4)) {
		for (done = msa; done + 4 <= len; done += 4)
			*(u32 *)(dst + done) =
				READ_ONCE(*(const volatile u32 *)(src + done));
	} else {
		for (done = msa; done + 4 <= len; done += 4)
			put_unaligned(READ_ONCE(*(const volatile u32 *)(src + done)),
				      (u32 *)(dst + done));
	}
	word = done - msa;
	for (; done < len; done++)
		dst[done] = READ_ONCE(*(const volatile u8 *)(src + done));
	spin_lock(&sfc->stats_lock);
	sfc->stats.uc_msa += msa;
	sfc->stats.uc_word += word;
	sfc->stats.uc_byte += len - msa - word;
	spin_unlock(&sfc->stats_lock);
#endif
}

static int ingenic_sfc_do_op(struct ingenic_sfc *sfc,
			     const struct spi_mem_op *op, bool may_be_fast,
			     enum ingenic_sfc_rate *used)
{
	enum ingenic_sfc_class cls = ingenic_sfc_classify(op);
	unsigned int len = op->data.nbytes;
	bool out = op->data.dir == SPI_MEM_DATA_OUT;
	enum dma_data_direction dir = out ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	u32 conf = XFER_CMD | op->cmd.opcode | XFER_ADDR_BYTES(op->addr.nbytes) |
		   XFER_DUMMY_BITS(op->dummy.nbytes * 8);
	u32 glb = GLB_CDT_EN | GLB_THRESHOLD(FIFO_WORDS);
	u32 index = 0, enabled = SR_END | SR_ERRORS;
	u64 t0 = READ_ONCE(sfc_timing) ? local_clock() : 0;
	dma_addr_t addr = DMA_MAPPING_ERROR;
	void *target = NULL;
	size_t mapped = 0;
	enum ingenic_sfc_rate fast = RATE_SAFE;
	bool uc = false;
	int ret;

	/* Only reads following a PAGE READ that opened a fast window run fast;
	 * any other operation closes the window.
	 */
	switch (cls) {
	case CLS_PAGE_READ:
		sfc->window = may_be_fast ?
			      ingenic_sfc_open_rate(sfc, op->addr.val) : RATE_SAFE;
		sfc->window_row = op->addr.val;
		sfc->window_task = current;
		fast = sfc->window;
		break;
	case CLS_STATUS:
		if (may_be_fast && ingenic_sfc_status_op(op))
			fast = ingenic_sfc_window_rate(sfc);
		break;
	case CLS_CACHE:
		if (may_be_fast)
			fast = ingenic_sfc_window_rate(sfc);
		break;
	default:
		sfc->window = RATE_SAFE;
		break;
	}
	if (used)
		*used = fast;
	sfc->t_trig = sfc->t_wake = sfc->t_end = 0;
	sfc->spun = false;
	ret = ingenic_sfc_clock(sfc, fast);
	if (ret)
		goto account;

	if (len) {
		conf |= XFER_DATA;
		index |= IDX_DATA;
		if (out)
			index |= IDX_WRITE;
	}
	if (len && ingenic_sfc_use_dma(sfc, op) && ingenic_sfc_below_dma_min(op)) {
		spin_lock(&sfc->stats_lock);
		sfc->stats.path_pio_min++;
		spin_unlock(&sfc->stats_lock);
	} else if (len && ingenic_sfc_use_dma(sfc, op)) {
		if (ingenic_sfc_direct(sfc, op)) {
			target = op->data.buf.in;
			mapped = len;
		} else {
			/* Whole cache lines of the driver's own page:
			 * invalidation cannot discard a neighbour's data, and a
			 * transfer stopped late can reach only this page.
			 */
			target = sfc->bounce;
			mapped = min_t(size_t, ALIGN(len, dma_get_cache_alignment()),
				       MAX_DATA);
			if (out)
				memcpy(sfc->bounce, op->data.buf.out, len);
		}
		addr = dma_map_single(sfc->dev, target, mapped, dir);
		if (dma_mapping_error(sfc->dev, addr))
			addr = DMA_MAPPING_ERROR;
		/* decided once per operation: the unmap and the copy agree */
		uc = !out && target == sfc->bounce && READ_ONCE(sfc_uc_bounce) &&
		     ingenic_sfc_uc_ok(sfc, addr);
	}
	if (len) {
		spin_lock(&sfc->stats_lock);
		if (addr != DMA_MAPPING_ERROR)
			sfc->stats.path_dma++;
		else
			sfc->stats.path_pio++;
		spin_unlock(&sfc->stats_lock);
	}
	if (addr != DMA_MAPPING_ERROR) {
		sfc->desc->next = 0;
		sfc->desc->addr = cpu_to_le32(addr);
		sfc->desc->len = cpu_to_le32(len);
		sfc->desc->link = 0;
		glb |= GLB_DES_EN | GLB_OP_DMA | GLB_BURST(DMA_BURST);
	} else if (len) {
		enabled |= out ? SR_TREQ : SR_RREQ;
		if (out)
			sfc->pio_out = op->data.buf.out;
		else
			sfc->pio_in = op->data.buf.in;
		sfc->pio_len = len;
	}

	/* CDT enabled; PIO, or DMA through one descriptor. */
	ingenic_sfc_setup(sfc, glb);
	if (addr != DMA_MAPPING_ERROR) {
		/* Vendor sfc_sync_cdt_raw: MEM_ADDR 0, descriptor chain at DES_ADDR. */
		writel(0, sfc->base + SFC_MEM_ADDR);
		writel(sfc->desc_dma, sfc->base + SFC_DES_ADDR);
	}
	writel(op->addr.val, sfc->base + SFC_COL_ADDR);
	writel(len, sfc->base + SFC_TRAN_LEN);
	/* Address source COL, no link; mode 5 is single command/quad data. */
	ingenic_sfc_cdt(sfc, 0, CDT_ADDR_COL |
			CDT_MODE(len && op->data.buswidth == 4 ? MODE_QUAD_DATA : MODE_STD),
			conf, 0, 0);
	writel(index, sfc->base + SFC_CMD_IDX);

	ret = ingenic_sfc_run(sfc, enabled, 1000);
	if (addr != DMA_MAPPING_ERROR) {
		if (uc) {
			dma_unmap_single_attrs(sfc->dev, addr, mapped, dir,
					       DMA_ATTR_SKIP_CPU_SYNC);
			if (!ret)
				ingenic_sfc_copy_uc(sfc, op->data.buf.in, len);
		} else {
			dma_unmap_single(sfc->dev, addr, mapped, dir);
			if (!ret && !out && target == sfc->bounce) {
				memcpy(op->data.buf.in, sfc->bounce, len);
				spin_lock(&sfc->stats_lock);
				sfc->stats.bounce_cached += len;
				spin_unlock(&sfc->stats_lock);
			}
		}
		if (target != sfc->bounce) {
			spin_lock(&sfc->stats_lock);
			sfc->stats.direct++;
			spin_unlock(&sfc->stats_lock);
		}
	}
 account:
	ingenic_sfc_account(sfc, cls, t0, ingenic_sfc_clocks(op), fast != RATE_SAFE, ret);
	return ret;
}

/* Issue a PAGE READ held for chaining on its own. */
static int ingenic_sfc_flush(struct ingenic_sfc *sfc)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(sfc->deferred_opcode, 1),
					  SPI_MEM_OP_ADDR(3, sfc->deferred_row, 1),
					  SPI_MEM_OP_NO_DUMMY,
					  SPI_MEM_OP_NO_DATA);

	if (!sfc->deferred)
		return 0;
	sfc->deferred = false;
	spin_lock(&sfc->stats_lock);
	sfc->stats.flushes++;
	spin_unlock(&sfc->stats_lock);
	return ingenic_sfc_do_op(sfc, &op, sfc->window != RATE_SAFE, NULL);
}

/* Reject a suspended controller; record the device tree rate. */
static int ingenic_sfc_enter(struct ingenic_sfc *sfc, struct spi_mem *mem,
			     const struct spi_mem_op *op)
{
	unsigned int freq = mem->spi->max_speed_hz;

	/* spi-mem's direct exec path holds bus_lock_mutex but does not reject
	 * SPI_CONTROLLER_SUSPENDED itself. Check before any clock/MMIO access;
	 * controller_suspend marks this flag under that same mutex.
	 */
	if (READ_ONCE(mem->spi->controller->flags) & SPI_CONTROLLER_SUSPENDED)
		return -ESHUTDOWN;
	if (op->max_freq && op->max_freq < freq)
		freq = op->max_freq;
	sfc->safe_hz = freq;
	return 0;
}

static int ingenic_sfc_poll_status(struct spi_mem *mem,
				   const struct spi_mem_op *op,
				   u16 mask, u16 match,
				   unsigned long initial_delay_us,
				   unsigned long polling_rate_us,
				   unsigned long timeout_ms);
static int ingenic_sfc_poll(struct ingenic_sfc *sfc,
			    const struct spi_mem_op *op, u16 mask, u16 match,
			    unsigned long timeout_ms, bool chain,
			    enum ingenic_sfc_rate fast);

/*
 * A production-rate read sequence failed at its status (fast_ret, or an
 * uncorrectable ECC status): the page is read again at the device tree
 * rate, PAGE READ of the same row and the status poll into the caller's
 * buffer, which then holds the safe-rate status. An error at the fast rate,
 * or an uncorrectable status the safe rate does not repeat, is the wire:
 * fallback. The same uncorrectable status at both rates is the page.
 */
static int ingenic_sfc_reread(struct ingenic_sfc *sfc,
			      const struct spi_mem_op *status, u16 mask,
			      u16 match, unsigned long timeout_ms, int fast_ret,
			      const char *what)
{
	struct spi_mem_op load = SPI_MEM_OP(SPI_MEM_OP_CMD(OP_PAGE_READ, 1),
					    SPI_MEM_OP_ADDR(3, sfc->window_row, 1),
					    SPI_MEM_OP_NO_DUMMY,
					    SPI_MEM_OP_NO_DATA);
	int ret;

	atomic_inc(&sfc->retries);
	ret = ingenic_sfc_do_op(sfc, &load, false, NULL);
	if (!ret)
		ret = ingenic_sfc_poll(sfc, status, mask, match, timeout_ms,
				       false, RATE_SAFE);
	if (fast_ret || !ingenic_sfc_status_failed(ret, status->data.buf.in))
		ingenic_sfc_fallback(sfc, fast_ret ? what :
				     "uncorrectable ECC status only at the fast rate",
				     fast_ret);
	return ret;
}

static int ingenic_sfc_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct ingenic_sfc *sfc = spi_controller_get_devdata(mem->spi->controller);
	enum ingenic_sfc_class cls = ingenic_sfc_classify(op);
	enum ingenic_sfc_rate rate;
	int ret;

	ret = ingenic_sfc_enter(sfc, mem, op);
	if (ret)
		return ret;
	if (sfc->deferred && cls != CLS_STATUS) {
		/* Anything but a status read must not reach a busy chip:
		 * issue the held PAGE READ and wait for OIP to clear.
		 */
		u8 status __aligned(4);
		struct spi_mem_op poll =
			SPI_MEM_OP(SPI_MEM_OP_CMD(OP_GET_FEATURE, 1),
				   SPI_MEM_OP_ADDR(1, 0xc0, 1),
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_IN(1, &status, 1));

		ret = ingenic_sfc_poll_status(mem, &poll, 0x01, 0x00, 0, 0, 400);
		if (ret == -EOPNOTSUPP)
			ret = ingenic_sfc_flush(sfc);
	} else {
		/* A status read after a held PAGE READ: spi-mem's own poll loop. */
		ret = ingenic_sfc_flush(sfc);
	}
	if (ret)
		return ret;
	if (READ_ONCE(sfc_chain) && READ_ONCE(sfc_hw_poll) &&
	    cls == CLS_PAGE_READ && !op->cmd.dtr &&
	    !op->addr.dtr && !op->dummy.nbytes) {
		/* Issued with the status poll spinand sends next. The arming
		 * decision is taken now, in the issuing task.
		 */
		sfc->deferred = true;
		sfc->deferred_opcode = op->cmd.opcode;
		sfc->deferred_row = op->addr.val;
		sfc->window = ingenic_sfc_open_rate(sfc, op->addr.val);
		sfc->window_row = op->addr.val;
		sfc->window_task = current;
		return 0;
	}
	ret = ingenic_sfc_do_op(sfc, op, true, &rate);
	if (!ingenic_sfc_prod(sfc, rate))
		return ret;
	if (cls == CLS_STATUS) {
		/* spi-mem's own poll loop (hw_poll 0): read the page again and
		 * poll to the end at the device tree rate.
		 */
		if (ingenic_sfc_status_failed(ret, op->data.buf.in))
			ret = ingenic_sfc_reread(sfc, op, STATUS_OIP, 0, WAITRDY_MS,
						 ret, "status read error");
	} else if (ret) {
		/* PAGE READ or READ FROM CACHE (which leaves the flash's
		 * cache as it was): the same command again at the safe rate.
		 */
		ingenic_sfc_fallback(sfc, cls == CLS_CACHE ? "cache read error" :
				     "page read error", ret);
		atomic_inc(&sfc->retries);
		ret = ingenic_sfc_do_op(sfc, op, false, NULL);
	}
	return ret;
}

/* One status poll in the controller at the given rate, the held PAGE READ
 * in front when chain.
 */
static int ingenic_sfc_poll(struct ingenic_sfc *sfc,
			    const struct spi_mem_op *op, u16 mask, u16 match,
			    unsigned long timeout_ms, bool chain,
			    enum ingenic_sfc_rate fast)
{
	u32 conf = XFER_CMD | op->cmd.opcode | XFER_ADDR_BYTES(op->addr.nbytes);
	u64 t0 = READ_ONCE(sfc_timing) ? local_clock() : 0;
	enum ingenic_sfc_class cls = chain ? CLS_CHAIN : CLS_POLL;
	unsigned int first = 0;
	u32 clocks;
	int ret;

	sfc->t_trig = sfc->t_wake = sfc->t_end = 0;
	sfc->spun = false;
	clocks = 2 * 8 * (1 + op->addr.nbytes) + 8;
	ret = ingenic_sfc_clock(sfc, fast);
	if (ret)
		goto out;

	ingenic_sfc_setup(sfc, GLB_POLL_TIME(POLL_TIME) | GLB_CDT_EN |
			  GLB_THRESHOLD(FIFO_WORDS));
	writel(op->addr.val, sfc->base + SFC_STA_ADDR0);
	writel(1, sfc->base + SFC_TRAN_LEN);
	if (chain) {
		writel(sfc->deferred_row, sfc->base + SFC_ROW_ADDR);
		ingenic_sfc_cdt(sfc, 0, CDT_LINK | CDT_ADDR_ROW | CDT_MODE(MODE_STD),
				XFER_CMD | sfc->deferred_opcode |
				XFER_ADDR_BYTES(3), 0, 0);
		first = 1;
		clocks += 32;
	}
	ingenic_sfc_cdt(sfc, first, CDT_LINK | CDT_ADDR_STA0 | CDT_MODE(MODE_STD),
			conf | XFER_POLL, match, mask);
	ingenic_sfc_cdt(sfc, first + 1, CDT_ADDR_STA0 | CDT_MODE(MODE_STD),
			conf | XFER_DATA, 0, 0);
	writel(IDX_DATA, sfc->base + SFC_CMD_IDX);
	sfc->pio_in = op->data.buf.in;
	sfc->pio_len = 1;
	ret = ingenic_sfc_run(sfc, SR_END | SR_ERRORS | SR_RREQ, timeout_ms);
 out:
	ingenic_sfc_account(sfc, cls, t0, clocks, fast != RATE_SAFE, ret);
	return ret;
}

/*
 * Status polling in the controller: CDT entry 0 repeats the status command
 * until (status & mask) == match (vendor MK_ST with POLL_EN), then links to
 * entry 1, which reads the same register once more into the caller's buffer
 * by PIO. One start and one completion replace spi-mem's command loop.
 * A held PAGE READ goes in front as one more linked entry (vendor
 * NAND_QUAD_READ_TO_CACHE, row address from ROW_ADDR).
 */
static int ingenic_sfc_poll_status(struct spi_mem *mem,
				   const struct spi_mem_op *op,
				   u16 mask, u16 match,
				   unsigned long initial_delay_us,
				   unsigned long polling_rate_us,
				   unsigned long timeout_ms)
{
	struct ingenic_sfc *sfc = spi_controller_get_devdata(mem->spi->controller);
	enum ingenic_sfc_rate fast = RATE_SAFE;
	bool chain = false;
	int ret;

	if (!READ_ONCE(sfc_hw_poll) || op->cmd.nbytes != 1 ||
	    op->cmd.buswidth != 1 || op->cmd.dtr || op->addr.nbytes > 4 ||
	    (op->addr.nbytes && (op->addr.buswidth != 1 || op->addr.dtr)) ||
	    op->dummy.nbytes || op->data.nbytes != 1 || op->data.buswidth != 1 ||
	    op->data.dtr || op->data.dir != SPI_MEM_DATA_IN || (mask | match) > 0xff)
		return -EOPNOTSUPP;

	ret = ingenic_sfc_enter(sfc, mem, op);
	if (sfc->deferred) {
		/* Consumed whatever happens: spinand fails the read on any error,
		 * and a held PAGE READ must not run later without its poll.
		 */
		chain = true;
		sfc->deferred = false;
	}
	if (ret)
		return ret;
	if (ingenic_sfc_status_op(op))
		fast = ingenic_sfc_window_rate(sfc);
	ret = ingenic_sfc_poll(sfc, op, mask, match, timeout_ms, chain, fast);
	if (ingenic_sfc_prod(sfc, fast) &&
	    ingenic_sfc_status_failed(ret, op->data.buf.in))
		ret = ingenic_sfc_reread(sfc, op, mask, match, timeout_ms, ret,
					 ret == -ETIMEDOUT ? "status poll timeout" :
					 "status poll error");
	return ret;
}

static const struct spi_controller_mem_ops ingenic_sfc_mem_ops = {
	.supports_op = ingenic_sfc_supports_op,
	.adjust_op_size = ingenic_sfc_adjust_op_size,
	.exec_op = ingenic_sfc_exec_op,
	.poll_status = ingenic_sfc_poll_status,
};

static int ingenic_sfc_stats_get(char *buf, const struct kernel_param *kp)
{
	struct ingenic_sfc *sfc = READ_ONCE(sfc_instance);
	struct ingenic_sfc_stats s;
	unsigned long hz, fast_hz;
	unsigned int safe;
	int len, i;

	if (!sfc)
		return scnprintf(buf, PAGE_SIZE, "absent\n");
	spin_lock_irq(&sfc->stats_lock);
	s = sfc->stats;
	hz = sfc->actual_hz;
	fast_hz = sfc->fast_hz;
	safe = sfc->safe_hz;
	spin_unlock_irq(&sfc->stats_lock);
	len = scnprintf(buf, PAGE_SIZE,
			"clock hz=%lu fast_hz=%lu safe_hz=%u changes=%u timeouts=%d flushes=%u direct=%u\n",
			hz, fast_hz, safe, s.clock_changes,
			atomic_read(&sfc->faults.timeouts), s.flushes, s.direct);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "faults timeouts=%d over=%d under=%d stop_busy=%d end_busy=%d dma_dead=%d\n",
			 atomic_read(&sfc->faults.timeouts),
			 atomic_read(&sfc->faults.over),
			 atomic_read(&sfc->faults.under),
			 atomic_read(&sfc->faults.stop_busy),
			 atomic_read(&sfc->faults.end_busy),
			 READ_ONCE(sfc->dma_dead));
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "bounce uc_bounce=%d cached_bytes=%llu uc_msa_bytes=%llu uc_word_bytes=%llu uc_byte_bytes=%llu\n",
			 READ_ONCE(sfc_uc_bounce), s.bounce_cached, s.uc_msa,
			 s.uc_word, s.uc_byte);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "path dma_min=%u dma=%u pio=%u pio_min=%u\n",
			 READ_ONCE(sfc_dma_min), s.path_dma, s.path_pio,
			 s.path_pio_min);
	for (i = 0; i < CLS_COUNT; i++) {
		struct ingenic_sfc_class_stats *c = &s.c[i];

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%s n=%u fast=%u spun=%u errors=%u total_us=%llu setup_us=%llu hw_us=%llu wake_us=%llu post_us=%llu wire_us=%llu max_us=%llu\n",
				 ingenic_sfc_class_names[i], c->n, c->fast, c->spun,
				 c->errors, div_u64(c->total, 1000),
				 div_u64(c->setup, 1000), div_u64(c->hw, 1000),
				 div_u64(c->wake, 1000), div_u64(c->post, 1000),
				 div_u64(c->wire, 1000), div_u64(c->max, 1000));
	}
	return len;
}

static int ingenic_sfc_stats_set(const char *val, const struct kernel_param *kp)
{
	struct ingenic_sfc *sfc = READ_ONCE(sfc_instance);

	if (!sfc)
		return -ENODEV;
	spin_lock_irq(&sfc->stats_lock);
	memset(&sfc->stats, 0, sizeof(sfc->stats));
	spin_unlock_irq(&sfc->stats_lock);
	return 0;
}

static const struct kernel_param_ops ingenic_sfc_stats_ops = {
	.set = ingenic_sfc_stats_set,
	.get = ingenic_sfc_stats_get,
};
module_param_cb(stats, &ingenic_sfc_stats_ops, NULL, 0644);
MODULE_PARM_DESC(stats, "Operation counts and times per class; write anything to reset (fault counts stay)");

/*
 * SPI NAND feature registers (A0 block lock, B0 configuration, C0 status,
 * D0 drive strength; DS35Q1GA Rev.03 Table 3.1) read at the device tree
 * rate, under the controller's own locks, so a test can confirm that no
 * experimental setting changed them.
 */
static int ingenic_sfc_features_get(char *buf, const struct kernel_param *kp)
{
	static const u8 regs[] = { 0xa0, 0xb0, 0xc0, 0xd0 };
	struct ingenic_sfc *sfc = READ_ONCE(sfc_instance);
	struct spi_controller *ctlr;
	u8 val[ARRAY_SIZE(regs)] __aligned(4) = { 0 };
	int ret = 0, i;

	if (!sfc)
		return scnprintf(buf, PAGE_SIZE, "absent\n");
	ctlr = sfc->ctlr;
	mutex_lock(&ctlr->bus_lock_mutex);
	mutex_lock(&ctlr->io_mutex);
	if (READ_ONCE(ctlr->flags) & SPI_CONTROLLER_SUSPENDED)
		ret = -ESHUTDOWN;
	if (!ret)
		ret = ingenic_sfc_flush(sfc);
	sfc->window = RATE_SAFE;
	for (i = 0; !ret && i < ARRAY_SIZE(regs); i++) {
		struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(OP_GET_FEATURE, 1),
				   SPI_MEM_OP_ADDR(1, regs[i], 1),
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_IN(1, &val[i], 1));

		ret = ingenic_sfc_do_op(sfc, &op, false, NULL);
	}
	mutex_unlock(&ctlr->io_mutex);
	mutex_unlock(&ctlr->bus_lock_mutex);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "a0=%02x b0=%02x c0=%02x d0=%02x\n",
			 val[0], val[1], val[2], val[3]);
}

static const struct kernel_param_ops ingenic_sfc_features_ops = {
	.get = ingenic_sfc_features_get,
};
module_param_cb(features, &ingenic_sfc_features_ops, NULL, 0444);
MODULE_PARM_DESC(features, "SPI NAND feature registers A0, B0, C0, D0 at the device tree rate");

static int ingenic_sfc_read_check_get(char *buf, const struct kernel_param *kp)
{
	struct ingenic_sfc *sfc = READ_ONCE(sfc_instance);
	int state;

	if (!sfc)
		return scnprintf(buf, PAGE_SIZE, "absent\n");
	state = READ_ONCE(sfc->check_state);
	return scnprintf(buf, PAGE_SIZE,
			 "state=%s hz=%u delay=%u actual_hz=%lu pages=%u mismatches=%u errors=%u skipped=%u ms=%u fallbacks=%d retries=%d\n",
			 ingenic_sfc_check_names[state], sfc->prod_hz,
			 sfc->prod_delay, sfc->check_hz, sfc->check_pages,
			 sfc->check_mismatches, sfc->check_errors,
			 sfc->check_skipped, sfc->check_ms,
			 atomic_read(&sfc->fallbacks), atomic_read(&sfc->retries));
}

static const struct kernel_param_ops ingenic_sfc_read_check_ops = {
	.get = ingenic_sfc_read_check_get,
};
module_param_cb(read_check, &ingenic_sfc_read_check_ops, NULL, 0444);
MODULE_PARM_DESC(read_check, "Boot check and production read state: state, pair, pages compared, mismatches, errors, fallbacks, retries");

/* len bytes for the check at the safe or the production rate; corrected
 * bitflips are success.
 */
static int ingenic_sfc_check_read(struct ingenic_sfc *sfc, struct mtd_info *mtd,
				  loff_t offset, size_t len, u8 *buf, bool fast)
{
	size_t retlen = 0;
	int ret;

	WRITE_ONCE(sfc->check_fast, fast);
	ret = mtd_read(mtd, offset, len, &retlen, buf);
	if (mtd_is_bitflip(ret))
		ret = 0;
	if (!ret && retlen != len)
		ret = -EIO;
	return ret;
}

/* Good blocks in order, scanned only as far as block index want. */
struct ingenic_sfc_check_map {
	unsigned int *good, ngood, next, blocks;
};

static bool ingenic_sfc_check_good(struct mtd_info *mtd,
				   struct ingenic_sfc_check_map *m,
				   unsigned int want)
{
	while (m->ngood <= want && m->next < m->blocks) {
		if (mtd_block_isbad(mtd, (loff_t)m->next * mtd->erasesize) == 0)
			m->good[m->ngood++] = m->next;
		m->next++;
	}
	return m->ngood > want;
}

/*
 * Runs of pages once at the device tree rate and once at
 * prod_hz/prod_delay, back to back, in this thread's private buffers;
 * nothing enters a cache. Pages: the first CHECK_HEAD bytes of the good
 * blocks in order, CHECK_RUN pages per read so that the clock changes once
 * per run, then CHECK_SPREAD single pages spread over the rest of the
 * squashfs image (bound by the superblock's bytes_used; the whole good
 * area when page 0 holds no superblock). A run that fails at the safe rate,
 * or with the controller suspended, is skipped; any other error at the
 * fast rate (timeout, controller error, uncorrectable ECC status) ends the
 * check. Bad blocks are looked up only as far as the pages read.
 */
static int ingenic_sfc_check(struct ingenic_sfc *sfc, struct mtd_info *mtd)
{
	unsigned int ppb = mtd->erasesize / mtd->writesize;
	struct ingenic_sfc_check_map m = { .blocks = mtd_div_by_eb(mtd->size, mtd) };
	unsigned int used, head, page = 0, k = 0;
	size_t len = (size_t)CHECK_RUN * mtd->writesize;
	u8 *a = NULL, *b = NULL;
	int state = CHECK_FAILED;

	m.good = kcalloc(m.blocks, sizeof(*m.good), GFP_KERNEL);
	a = kvmalloc(len, GFP_KERNEL);
	b = kvmalloc(len, GFP_KERNEL);
	if (!m.good || !a || !b || !ppb || mtd->writesize < 64 ||
	    !ingenic_sfc_check_good(mtd, &m, 0))
		goto out;
	used = m.blocks * ppb;
	/* squashfs superblock: magic "hsqs", bytes_used at byte 40 */
	if (!ingenic_sfc_check_read(sfc, mtd, (loff_t)m.good[0] * mtd->erasesize,
				    mtd->writesize, a, false) &&
	    get_unaligned_le32(a) == 0x73717368) {
		u64 bytes = get_unaligned_le64(a + 40);

		used = min_t(u64, used, DIV_ROUND_UP_ULL(bytes, mtd->writesize));
	}
	head = min(used, CHECK_HEAD / mtd->writesize);
	for (;;) {
		unsigned int n = 1, i;
		loff_t offset;
		int ret;

		if (page < head) {
			n = min3(CHECK_RUN, head - page, ppb - page % ppb);
		} else {
			if (used <= head || k == CHECK_SPREAD)
				break;
			page = head + div_u64((u64)k++ * (used - head), CHECK_SPREAD);
		}
		if (!ingenic_sfc_check_good(mtd, &m, page / ppb))
			break;
		offset = (loff_t)m.good[page / ppb] * mtd->erasesize +
			 (loff_t)(page % ppb) * mtd->writesize;
		page += n;
		len = (size_t)n * mtd->writesize;
		if (ingenic_sfc_check_read(sfc, mtd, offset, len, a, false)) {
			sfc->check_skipped += n;
			continue;
		}
		ret = ingenic_sfc_check_read(sfc, mtd, offset, len, b, true);
		if (ret == -ESHUTDOWN) {
			sfc->check_skipped += n;
			continue;
		}
		if (ret) {
			sfc->check_errors++;
			break;
		}
		sfc->check_pages += n;
		for (i = 0; i < n; i++)
			if (memcmp(a + i * mtd->writesize, b + i * mtd->writesize,
				   mtd->writesize))
				sfc->check_mismatches++;
		cond_resched();
	}
	if (!sfc->check_errors && !sfc->check_mismatches &&
	    sfc->check_pages >= CHECK_MIN)
		state = CHECK_PASSED;
 out:
	kvfree(b);
	kvfree(a);
	kfree(m.good);
	return state;
}

static int ingenic_sfc_check_thread(void *data)
{
	struct ingenic_sfc *sfc = data;
	struct mtd_info *mtd = sfc->check_mtd;
	unsigned int hz = READ_ONCE(sfc_read_hz);
	u64 t0 = local_clock();
	int state = CHECK_OFF;

	if (hz > PROD_HZ_MAX)
		dev_warn(sfc->dev, "read_hz %u above %u (tCS): NAND reads stay at the device tree rate\n",
			 hz, PROD_HZ_MAX);
	if (hz && hz <= PROD_HZ_MAX) {
		/* Read by other tasks only after the state says passed, and
		 * under the bus mutex every operation takes.
		 */
		sfc->prod_hz = hz;
		sfc->prod_delay = READ_ONCE(sfc_read_delay);
		WRITE_ONCE(sfc->check_task, current);
		state = ingenic_sfc_check(sfc, mtd);
		WRITE_ONCE(sfc->check_task, NULL);
		sfc->check_hz = sfc->fast_hz;
	}
	sfc->check_ms = div_u64(local_clock() - t0, NSEC_PER_MSEC);
	put_mtd_device(mtd);
	WRITE_ONCE(sfc->check_state, state);
	if (state == CHECK_PASSED)
		dev_info(sfc->dev, "NAND read check passed: %u pages equal at %lu Hz, sample delay %u, and at the device tree rate (%u ms)\n",
			 sfc->check_pages, sfc->check_hz, sfc->prod_delay,
			 sfc->check_ms);
	else if (state == CHECK_FAILED)
		dev_warn(sfc->dev, "NAND read check failed at %u Hz, sample delay %u: %u pages, %u mismatches, %u errors, %u skipped; reads stay at the device tree rate\n",
			 sfc->prod_hz, sfc->prod_delay, sfc->check_pages,
			 sfc->check_mismatches, sfc->check_errors,
			 sfc->check_skipped);
	return 0;
}

/*
 * The check starts when the check_part partition of this controller's
 * flash appears (spinand's probe, under mtd_table_mutex), before the root
 * is mounted. It runs in its own thread and holds the partition until it
 * ends; reads by anyone else stay at the device tree rate meanwhile.
 */
static void ingenic_sfc_mtd_add(struct mtd_info *mtd)
{
	struct device *parent = mtd_get_master(mtd)->dev.parent;
	struct spi_controller *ctlr;
	struct task_struct *task;
	struct ingenic_sfc *sfc;

	if (strcmp(mtd->name, sfc_check_part) || !mtd_type_is_nand(mtd) ||
	    !mtd->writesize || !parent || parent->bus != &spi_bus_type)
		return;
	ctlr = to_spi_device(parent)->controller;
	if (ctlr->mem_ops != &ingenic_sfc_mem_ops)
		return;
	sfc = spi_controller_get_devdata(ctlr);
	if (READ_ONCE(sfc->check_state) != CHECK_PENDING || __get_mtd_device(mtd))
		return;
	sfc->check_mtd = mtd;
	WRITE_ONCE(sfc->check_state, CHECK_RUNNING);
	task = kthread_run(ingenic_sfc_check_thread, sfc, "sfc-readcheck");
	if (IS_ERR(task)) {
		__put_mtd_device(mtd);
		WRITE_ONCE(sfc->check_state, CHECK_FAILED);
		dev_warn(sfc->dev, "NAND read check not started (%ld); reads stay at the device tree rate\n",
			 PTR_ERR(task));
	}
}

static void ingenic_sfc_mtd_remove(struct mtd_info *mtd)
{
}

static struct mtd_notifier ingenic_sfc_mtd_notifier = {
	.add = ingenic_sfc_mtd_add,
	.remove = ingenic_sfc_mtd_remove,
};

static int ingenic_sfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ingenic_sfc *sfc;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*sfc));
	if (!ctlr)
		return -ENOMEM;
	sfc = spi_controller_get_devdata(ctlr);
	sfc->dev = dev;
	sfc->ctlr = ctlr;
	sfc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sfc->base))
		return PTR_ERR(sfc->base);
	sfc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(sfc->clk))
		return dev_err_probe(dev, PTR_ERR(sfc->clk), "SFC clock\n");
	sfc->irq = platform_get_irq(pdev, 0);
	if (sfc->irq < 0)
		return sfc->irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/* The bounce has pages of its own: page aligned, and no other object
	 * shares its cache lines or its memory.
	 */
	sfc->bounce = (void *)devm_get_free_pages(dev, GFP_KERNEL,
						  get_order(MAX_DATA));
	if (!sfc->bounce)
		return -ENOMEM;
	sfc->desc = dmam_alloc_coherent(dev, sizeof(*sfc->desc), &sfc->desc_dma,
					GFP_KERNEL);
	if (!sfc->desc)
		return -ENOMEM;
	atomic_set(&sfc->faults.timeouts, 0);
	atomic_set(&sfc->faults.over, 0);
	atomic_set(&sfc->faults.under, 0);
	atomic_set(&sfc->faults.stop_busy, 0);
	atomic_set(&sfc->faults.end_busy, 0);
	atomic_set(&sfc->fallbacks, 0);
	atomic_set(&sfc->retries, 0);
	sfc->check_state = CHECK_PENDING;
	platform_set_drvdata(pdev, ctlr);
	init_completion(&sfc->done);
	spin_lock_init(&sfc->lock);
	spin_lock_init(&sfc->stats_lock);
	writel(SR_ALL, sfc->base + SFC_INTC);
	writel(TRIG_STOP, sfc->base + SFC_TRIG);
	writel(SR_ALL, sfc->base + SFC_SCR);
	writel(0, sfc->base + SFC_CGE);
	ret = devm_request_irq(dev, sfc->irq, ingenic_sfc_irq, 0,
			       dev_name(dev), sfc);
	if (ret)
		return ret;
	ctlr->mem_ops = &ingenic_sfc_mem_ops;
	ctlr->mode_bits = SPI_RX_QUAD | SPI_TX_QUAD;
	ctlr->num_chipselect = 1;
	/* Initial bring-up limit; high-speed sampling needs board validation. */
	ctlr->max_speed_hz = 25000000;
	sfc->safe_hz = ctlr->max_speed_hz;
	ctlr->dev.of_node = dev->of_node;
	ret = devm_spi_register_controller(dev, ctlr);
	if (!ret)
		WRITE_ONCE(sfc_instance, sfc);
	return ret;
}

static int ingenic_sfc_suspend(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct ingenic_sfc *sfc = spi_controller_get_devdata(ctlr);
	int ret;

	ret = spi_controller_suspend(ctlr);
	if (ret) {
		/* Core marks suspended even when stopping its queue fails. */
		spi_controller_resume(ctlr);
		return ret;
	}
	writel(SR_ALL, sfc->base + SFC_INTC);
	synchronize_irq(sfc->irq);
	clk_disable_unprepare(sfc->clk);
	return 0;
}

static int ingenic_sfc_resume(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct ingenic_sfc *sfc = spi_controller_get_devdata(ctlr);
	int ret;

	ret = clk_prepare_enable(sfc->clk);
	if (ret)
		return ret;
	writel(SR_ALL, sfc->base + SFC_INTC);
	writel(TRIG_STOP, sfc->base + SFC_TRIG);
	writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
	writel(SR_ALL, sfc->base + SFC_SCR);
	writel(0, sfc->base + SFC_CGE);
	return spi_controller_resume(ctlr);
}
static DEFINE_SIMPLE_DEV_PM_OPS(ingenic_sfc_pm_ops,
			      ingenic_sfc_suspend, ingenic_sfc_resume);

static const struct of_device_id ingenic_sfc_of_match[] = {
	{ .compatible = "ingenic,x2000-sfc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_sfc_of_match);

static struct platform_driver ingenic_sfc_driver = {
	.probe = ingenic_sfc_probe,
	.driver = {
		.name = "ingenic-sfc",
		.pm = pm_sleep_ptr(&ingenic_sfc_pm_ops),
		.of_match_table = ingenic_sfc_of_match,
		/* The check thread uses the controller; no unbind from sysfs. */
		.suppress_bind_attrs = true,
	},
};

/* The MTD notifier first: the flash's partitions appear during probe. */
static int __init ingenic_sfc_init(void)
{
	int ret;

	register_mtd_user(&ingenic_sfc_mtd_notifier);
	ret = platform_driver_register(&ingenic_sfc_driver);
	if (ret)
		unregister_mtd_user(&ingenic_sfc_mtd_notifier);
	return ret;
}
module_init(ingenic_sfc_init);

static void __exit ingenic_sfc_exit(void)
{
	platform_driver_unregister(&ingenic_sfc_driver);
	unregister_mtd_user(&ingenic_sfc_mtd_notifier);
}
module_exit(ingenic_sfc_exit);

MODULE_DESCRIPTION("Ingenic X2000 serial flash controller");
MODULE_LICENSE("GPL");
