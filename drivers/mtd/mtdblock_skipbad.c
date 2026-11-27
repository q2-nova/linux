// SPDX-License-Identifier: GPL-2.0-only
/* Read-only NAND images written with bad blocks omitted (e.g. nandwrite).
 * Q2 stock mtdblock_bbt_ro by wujiao establishes this image layout. This
 * implementation bounds capacity to good blocks and fails closed on scan,
 * short-read and uncorrectable ECC errors. It never remaps a mounted image.
 */
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched/clock.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <crypto/sha2.h>
#include "mtdblock_skipbad.h"

/* The block layer asks for one 512-byte sector at a time; without this cache
 * each sector costs a whole NAND page read.
 */
static bool page_cache = true;
module_param(page_cache, bool, 0644);
MODULE_PARM_DESC(page_cache, "Serve sectors from the last NAND page read (default 1; 0: one page read per sector)");

/* Requests arrive as segments of whole memory pages (the bdev page cache):
 * read the NAND pages they cover straight into the segment, in one MTD read
 * (two 2 KiB NAND pages per 4 KiB segment).
 */
static bool multi;
module_param(multi, bool, 0644);
MODULE_PARM_DESC(multi, "Read whole NAND pages of a request segment into it with one MTD read (default 0: per sector)");

static bool timing;
module_param(timing, bool, 0644);
MODULE_PARM_DESC(timing, "Record time spent in MTD reads for the stats parameter (default 0)");

struct skipbad_stats {
	u64 sectors, hits, reads, pages, direct_bytes, segments, mtd_ns;
	u32 bitflips, errors;
};

static DEFINE_SPINLOCK(stats_lock);
static struct skipbad_stats stats;

struct skipbad_dev {
	struct mtd_blktrans_dev dev;
	unsigned int *map;
	unsigned int good;
	u8 *page;		/* last NAND page read, or NULL if no cache */
	loff_t page_offset;	/* its offset in the partition, -1 if none */
};

/* The bench reads the first image device in the caller's context. */
static DEFINE_MUTEX(bench_mutex);
static struct skipbad_dev *bench_dev;
static char bench_result[192] = "none\n";

static int skipbad_is_bad(void *context, unsigned int block)
{
	struct mtd_info *mtd = context;

	return mtd_block_isbad(mtd, (loff_t)block * mtd->erasesize);
}

/* One MTD read of whole pages or one sector; 0, or -EIO. */
static int skipbad_mtd_read(struct mtd_info *mtd, loff_t offset, size_t len,
			    u8 *buf)
{
	u64 t0 = READ_ONCE(timing) ? local_clock() : 0;
	size_t retlen = 0;
	int ret;

	ret = mtd_read(mtd, offset, len, &retlen, buf);
	spin_lock(&stats_lock);
	stats.reads++;
	stats.pages += len >= mtd->writesize ? len / mtd->writesize : 1;
	if (t0)
		stats.mtd_ns += local_clock() - t0;
	if (mtd_is_bitflip(ret))
		stats.bitflips++;
	if (retlen != len || (ret && !mtd_is_bitflip(ret)))
		stats.errors++;
	spin_unlock(&stats_lock);
	return retlen != len || (ret && !mtd_is_bitflip(ret)) ? -EIO : 0;
}

static int skipbad_readsect(struct mtd_blktrans_dev *dev,
			   unsigned long sector, char *buf)
{
	struct skipbad_dev *priv = container_of(dev, struct skipbad_dev, dev);
	unsigned long long offset;
	int ret;

	ret = skipbad_offset(priv->map, priv->good, dev->mtd->erasesize,
			     sector, &offset);
	if (ret)
		return 1;
	spin_lock(&stats_lock);
	stats.sectors++;
	spin_unlock(&stats_lock);
	if (priv->page && READ_ONCE(page_cache)) {
		/* blktrans serializes requests per device; the image is never
		 * written while mapped, so a cached page stays valid.
		 */
		loff_t page = offset - mtd_mod_by_ws(offset, dev->mtd);

		if (page != priv->page_offset) {
			priv->page_offset = -1;
			if (skipbad_mtd_read(dev->mtd, page, dev->mtd->writesize,
					     priv->page))
				return 1;
			priv->page_offset = page;
		} else {
			spin_lock(&stats_lock);
			stats.hits++;
			spin_unlock(&stats_lock);
		}
		memcpy(buf, priv->page + (offset - page), 512);
		return 0;
	}
	priv->page_offset = -1;
	return !!skipbad_mtd_read(dev->mtd, offset, 512, buf);
}

/*
 * A request segment: whole NAND pages go straight into it, as many as lie
 * in one eraseblock per MTD read; partial pages through readsect. Returns
 * -EOPNOTSUPP while multi is off, so blktrans reads sector by sector.
 */
static int skipbad_readsects(struct mtd_blktrans_dev *dev, unsigned long block,
			     unsigned long nsect, char *buf)
{
	struct skipbad_dev *priv = container_of(dev, struct skipbad_dev, dev);
	struct mtd_info *mtd = dev->mtd;

	if (!READ_ONCE(multi))
		return -EOPNOTSUPP;
	spin_lock(&stats_lock);
	stats.segments++;
	spin_unlock(&stats_lock);
	while (nsect) {
		unsigned long long offset;
		size_t len;

		if (skipbad_offset(priv->map, priv->good, mtd->erasesize, block,
				   &offset))
			return 1;
		len = min_t(unsigned long long, (unsigned long long)nsect * 512,
			    mtd->erasesize - mtd_mod_by_eb(offset, mtd));
		if (!mtd_mod_by_ws(offset, mtd) && len >= mtd->writesize) {
			len = round_down(len, mtd->writesize);
			if (skipbad_mtd_read(mtd, offset, len, buf))
				return 1;
			spin_lock(&stats_lock);
			stats.sectors += len / 512;
			stats.direct_bytes += len;
			spin_unlock(&stats_lock);
		} else {
			if (skipbad_readsect(dev, block, buf))
				return 1;
			len = 512;
		}
		block += len / 512;
		nsect -= len / 512;
		buf += len;
	}
	return 0;
}


/*
 * Bench: read the first BYTES of the image (logical order, bad blocks
 * skipped as for the block device), an eraseblock per MTD read, into a
 * private buffer; nothing enters any cache. SHA-256 of those bytes unless
 * "nohash"; read time is reported apart from hash time. Jobs run in one
 * kernel thread, mtdskip-bench (its PID in bench_pid), which issues no other
 * I/O, so spi_ingenic_sfc.read_tgid can arm exactly the bench's reads.
 */
struct bench_job {
	unsigned long long bytes;
	bool hash;
	int ret;
	struct completion done;
};

static struct task_struct *bench_task;
static struct bench_job *bench_job;
static DEFINE_SPINLOCK(bench_lock);
static DECLARE_WAIT_QUEUE_HEAD(bench_wait);

/* Runs in the bench thread while the submitter holds bench_mutex. */
static int bench_run(struct bench_job *job)
{
	struct skipbad_dev *priv = bench_dev;
	struct mtd_info *mtd = priv->dev.mtd;
	unsigned int shift = ffs(mtd->erasesize) - 1;
	unsigned long long size = (unsigned long long)priv->good << shift;
	unsigned long long bytes = job->bytes, logical;
	unsigned int errors = 0, bitflips = 0;
	u8 digest[SHA256_DIGEST_SIZE];
	u64 read_ns = 0, hash_ns = 0, t;
	struct sha256_ctx ctx;
	u8 *data;
	int n;

	if (!bytes || bytes > size)
		bytes = size;
	data = kvmalloc(mtd->erasesize, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	sha256_init(&ctx);
	for (logical = 0; logical < bytes; logical += mtd->erasesize) {
		size_t want = min_t(unsigned long long, mtd->erasesize,
				    round_up(bytes - logical, mtd->writesize));
		loff_t phys = (loff_t)priv->map[logical >> shift] << shift;
		size_t retlen = 0;
		int err;

		t = local_clock();
		err = mtd_read(mtd, phys, want, &retlen, data);
		read_ns += local_clock() - t;
		if (retlen != want || (err && !mtd_is_bitflip(err))) {
			errors++;
			break;
		}
		bitflips += !!err;
		if (job->hash) {
			t = local_clock();
			sha256_update(&ctx, data,
				      min_t(unsigned long long, want, bytes - logical));
			hash_ns += local_clock() - t;
		}
		cond_resched();
	}
	sha256_final(&ctx, digest);
	kvfree(data);
	n = scnprintf(bench_result, sizeof(bench_result),
		      "bytes=%llu read_us=%llu hash_us=%llu errors=%u bitflips=%u",
		      logical < bytes ? logical : bytes, div_u64(read_ns, 1000),
		      div_u64(hash_ns, 1000), errors, bitflips);
	if (job->hash)
		scnprintf(bench_result + n, sizeof(bench_result) - n,
			  " sha256=%*phN\n", SHA256_DIGEST_SIZE, digest);
	else
		scnprintf(bench_result + n, sizeof(bench_result) - n,
			  " sha256=none\n");
	return errors ? -EIO : 0;
}

static int bench_thread(void *unused)
{
	while (!kthread_should_stop()) {
		struct bench_job *job;

		wait_event_interruptible(bench_wait, READ_ONCE(bench_job) ||
					 kthread_should_stop());
		spin_lock(&bench_lock);
		job = bench_job;
		bench_job = NULL;
		spin_unlock(&bench_lock);
		if (job) {
			job->ret = bench_run(job);
			complete(&job->done);
		}
	}
	return 0;
}

/* Caller holds bench_mutex. */
static int bench_start(void)
{
	struct task_struct *task;

	if (bench_task)
		return 0;
	task = kthread_run(bench_thread, NULL, "mtdskip-bench");
	if (IS_ERR(task))
		return PTR_ERR(task);
	bench_task = task;
	return 0;
}

static ssize_t bench_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	char arg[32], *p = arg, *word;
	struct bench_job job = { .hash = true };
	ssize_t ret;

	if (count >= sizeof(arg))
		return -EINVAL;
	memcpy(arg, buf, count);
	arg[count] = 0;
	word = strsep(&p, " \t\n");
	if (!word || kstrtoull(word, 0, &job.bytes))
		return -EINVAL;
	while ((word = strsep(&p, " \t\n"))) {
		if (!strcmp(word, "nohash"))
			job.hash = false;
		else if (*word)
			return -EINVAL;
	}
	init_completion(&job.done);

	mutex_lock(&bench_mutex);
	ret = bench_start();
	if (!ret && !bench_dev)
		ret = -ENODEV;
	if (!ret) {
		spin_lock(&bench_lock);
		bench_job = &job;
		spin_unlock(&bench_lock);
		wake_up(&bench_wait);
		/* The job lives on this stack: wait whatever happens. */
		wait_for_completion(&job.done);
		ret = job.ret;
	}
	mutex_unlock(&bench_mutex);
	return ret ?: count;
}

static ssize_t bench_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	ssize_t len;

	mutex_lock(&bench_mutex);
	len = sysfs_emit(buf, "%s", bench_result);
	mutex_unlock(&bench_mutex);
	return len;
}

static ssize_t bench_pid_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	ssize_t ret;

	mutex_lock(&bench_mutex);
	ret = bench_start();
	if (!ret)
		ret = sysfs_emit(buf, "%d\n", task_pid_nr(bench_task));
	mutex_unlock(&bench_mutex);
	return ret;
}

static struct kobj_attribute bench_attr = __ATTR(bench, 0600, bench_show, bench_store);
static struct kobj_attribute bench_pid_attr = __ATTR(bench_pid, 0400, bench_pid_show, NULL);
static struct attribute *bench_attrs[] = { &bench_attr.attr, &bench_pid_attr.attr, NULL };
static const struct attribute_group bench_group = { .attrs = bench_attrs };
static struct kobject *bench_kobj;


static int stats_get(char *buf, const struct kernel_param *kp)
{
	struct skipbad_stats s;

	spin_lock(&stats_lock);
	s = stats;
	spin_unlock(&stats_lock);
	return scnprintf(buf, PAGE_SIZE,
			 "sectors=%llu hits=%llu reads=%llu pages=%llu direct_bytes=%llu segments=%llu mtd_us=%llu bitflips=%u errors=%u\n",
			 s.sectors, s.hits, s.reads, s.pages, s.direct_bytes,
			 s.segments, div_u64(s.mtd_ns, 1000), s.bitflips, s.errors);
}

static int stats_set(const char *val, const struct kernel_param *kp)
{
	spin_lock(&stats_lock);
	memset(&stats, 0, sizeof(stats));
	spin_unlock(&stats_lock);
	return 0;
}

static const struct kernel_param_ops stats_ops = {
	.set = stats_set,
	.get = stats_get,
};
module_param_cb(stats, &stats_ops, NULL, 0644);
MODULE_PARM_DESC(stats, "Sectors, page-cache hits, MTD reads, pages and time; write anything to reset");

static void skipbad_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct skipbad_dev *priv;
	unsigned int blocks;
	int good;

	/* Writable userdata and selector partitions are not image devices. */
	if (!mtd_type_is_nand(mtd) || (mtd->flags & MTD_WRITEABLE) ||
	    mtd->erasesize < 512 || !is_power_of_2(mtd->erasesize) ||
	    mtd_mod_by_eb(mtd->size, mtd))
		return;
	blocks = mtd_div_by_eb(mtd->size, mtd);
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return;
	priv->map = kcalloc(blocks, sizeof(*priv->map), GFP_KERNEL);
	if (!priv->map)
		goto free_dev;
	good = skipbad_build_map(priv->map, blocks, skipbad_is_bad, mtd);
	if (good < 0) {
		pr_err("mtdskip: cannot map %s: %d\n", mtd->name, good);
		goto free_map;
	}
	priv->good = good;
	/* Sectors never cross a page of 512 bytes or more, a power of two. */
	priv->page_offset = -1;
	if (mtd->writesize >= 512 && is_power_of_2(mtd->writesize))
		priv->page = kmalloc(mtd->writesize, GFP_KERNEL);
	priv->dev.mtd = mtd;
	priv->dev.devnum = mtd->index;
	priv->dev.size = (u64)good * mtd->erasesize / 512;
	priv->dev.tr = tr;
	priv->dev.readonly = 1;
	if (add_mtd_blktrans_dev(&priv->dev))
		goto free_page;
	mutex_lock(&bench_mutex);
	if (!bench_dev && priv->page)
		bench_dev = priv;
	mutex_unlock(&bench_mutex);
	pr_info("mtdskip: %s: %u good eraseblocks, %u skipped\n",
		mtd->name, priv->good, blocks - priv->good);
	return;
free_page:
	kfree(priv->page);
free_map:
	kfree(priv->map);
free_dev:
	kfree(priv);
}

static void skipbad_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct skipbad_dev *priv = container_of(dev, struct skipbad_dev, dev);
	unsigned int *map = priv->map;
	u8 *page = priv->page;

	mutex_lock(&bench_mutex);
	if (bench_dev == priv)
		bench_dev = NULL;
	mutex_unlock(&bench_mutex);
	del_mtd_blktrans_dev(dev);
	kfree(page);
	kfree(map);
}

static struct mtd_blktrans_ops skipbad_tr = {
	.name = "mtdskip",
	.major = 0,
	.part_bits = 0,
	.blksize = 512,
	.readsect = skipbad_readsect,
	.readsects = skipbad_readsects,
	.add_mtd = skipbad_add_mtd,
	.remove_dev = skipbad_remove_dev,
	.owner = THIS_MODULE,
};

static int __init skipbad_init(void)
{
	int ret;

	bench_kobj = kobject_create_and_add("mtdskip", kernel_kobj);
	if (!bench_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(bench_kobj, &bench_group);
	if (!ret)
		ret = register_mtd_blktrans(&skipbad_tr);
	if (ret) {
		kobject_put(bench_kobj);
		if (bench_task)
			kthread_stop(bench_task);
	}
	return ret;
}

static void __exit skipbad_exit(void)
{
	deregister_mtd_blktrans(&skipbad_tr);
	kobject_put(bench_kobj);
	if (bench_task)
		kthread_stop(bench_task);
}

module_init(skipbad_init);
module_exit(skipbad_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Read-only NAND image with bad eraseblocks omitted");
