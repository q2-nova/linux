// SPDX-License-Identifier: GPL-2.0-only
/*
 * X2000 audio DMA0 -> DSP source 8 / timeslot 9 -> BAIC1.
 * Register/protocol evidence: Q2 soc_audio_dma.ko and soc_aic.ko, with
 * Ingenic as-v2 headers for register names. This intentionally follows the
 * stock single cyclic descriptor and pointer-polling protocol: its LINK
 * encoding differs from the SDK, so no unverified IRQ protocol is assumed.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <asm/kernel-msa.h>
#include "x2000-pointer-record.h"

#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DBA		0x0000
#define DCR		0x0008
#define DSR		0x000c
#define DCM		0x0010
#define DDA		0x0014
#define DGRR		0x0100
#define FAS		0x1000
#define FCR		0x1004
#define FFR		0x1008
#define FSR		0x100c
#define DFCR		0x2000
#define BTSET		0x4000
#define BTCLR		0x4004
#define BFCR0		0x4010
#define BFCR1		0x4014
#define BFCR2		0x4018
#define BST1		0x4020
#define BTT0		0x4028
#define TCTL		0x6010
#define TCFG		0x6014
#define TDIV		0x6018
#define TCGR		0x601c
#define TLCR		0x6020
#define CCR		0x6024
#define POLL_MIN_NS	1000000
#define POLL_MAX_NS	5000000
#define BOUNDARY_GUARD_NS	250000
#define BUFFER_MAX	(256 * 1024)
#define COPY_BOUNCE	4096
#define COPY_LINE	64

/*
 * pcm_copy: how write() samples reach the DMA buffer. The buffer is the
 * uncached (CCA2) mapping from SNDRV_DMA_TYPE_DEV, which the DMA reads.
 * 0: copy_from_iter() straight into it, as ALSA's default write path
 *    (word stores to the uncached mapping);
 * 1: copy_from_iter() into a cached bounce, then 16-byte MSA ld.d/st.d from
 *    the bounce into the buffer, one kernel MSA section per bounce; word
 *    stores where kernel_msa_begin() refuses and for the unaligned ends;
 * 2: as 1 with word stores only (the fallback path, for checks);
 * 3: as 1, then each chunk is read back through the uncached mapping after
 *    wmb() and compared with the bounce (copy_verify_bytes and
 *    copy_verify_mismatch in debugfs): a check of what the DMA will read,
 *    at about 20 us per KiB more.
 * Any other value acts as 0. Every mode writes the buffer only through its
 * uncached mapping; modes 1 to 3 end with wmb(). Read per write() call.
 */
static unsigned int pcm_copy = 1;
module_param(pcm_copy, uint, 0644);
MODULE_PARM_DESC(pcm_copy, "0: ALSA default copy; 1: cached bounce and MSA stores; 2: cached bounce and word stores; 3: as 1 with uncached read-back check");

/*
 * pointer_timer: 0 polls the DMA pointer every quarter period, bounded to
 * 1-5 ms (poll_ns, V0.55 to V0.122). 1 arms the timer for the bytes left
 * to the next period boundary at the byte rate plus BOUNDARY_GUARD_NS,
 * never under POLL_MIN_NS. Read at every timer start and expiry.
 */
static unsigned int pointer_timer = 1;
module_param(pointer_timer, uint, 0644);
MODULE_PARM_DESC(pointer_timer, "0: quarter-period poll (1-5 ms); 1: next period boundary plus 250 us");

struct x2000_audio {
	void __iomem *base;
	struct clk_bulk_data clocks[2];
	__le32 *desc;
	dma_addr_t desc_dma;
	struct hrtimer timer;
	struct snd_pcm_substream *substream;
	bool running, shutting_down, sleep_clock_off;
	u32 poll_ns, poll_count, timer_ns, period_count;
	unsigned int buffer_bytes, period_bytes, last_pos, period_pos;
	unsigned int byte_rate;
	bool timer_resync, sample_fresh;
	struct mutex copy_lock;
	u8 *bounce;
	u64 copy_msa_bytes, copy_word_bytes, copy_direct_bytes;
	u64 copy_verify_bytes, copy_verify_mismatch;
	u32 xrun_prepares;
	unsigned int snapshot_pos, snapshot_dma, snapshot_queued;
	struct q2_ptr_record *diag;
	spinlock_t diag_lock;
	bool diag_enabled, diag_opening;
	u32 diag_arm_id;
};

#include "x2000-pointer-driver.h"

static unsigned int x2000_position(struct x2000_audio *audio, u32 caller)
{
	u32 before, after, queued, fsr;
	u32 previous = audio->snapshot_pos;
	unsigned int tries;

	audio->sample_fresh = false;

	/*
	 * DBA includes prefetched FIFO words. Both registers change while
	 * playback runs; mixing an old DMA address with a refilled FIFO can
	 * move the reported position backwards and cause a false ALSA XRUN.
	 * Only publish a FIFO snapshot bracketed by an unchanged DMA address.
	 */
	for (tries = 0; tries < 8; tries++) {
		before = readl(audio->base + DBA);
		fsr = readl(audio->base + FSR);
		queued = ((fsr >> 16) & 0x1fff) * 4;
		after = readl(audio->base + DBA);
		if (before == after) {
			u32 pos = before - audio->substream->runtime->dma_addr;

			u32 disposition = Q2_PTR_INVALID;

			if (pos < audio->buffer_bytes && queued <= audio->buffer_bytes) {
				u32 advanced = pos >= audio->snapshot_dma ?
					pos - audio->snapshot_dma :
					audio->buffer_bytes - audio->snapshot_dma + pos;

				/* DBA and FIFO level can settle at different times even
				 * when DBA brackets the read unchanged. Do not publish
				 * more new FIFO bytes than DMA has fetched: that would
				 * move consumption backwards and invent an ALSA ring
				 * wrap. Retain the last accepted pair until it catches
				 * up. A real DMA ring wrap is in advanced above.
				 * Each operand is bounded by BUFFER_MAX (256 KiB).
				 */
				if (queued <= advanced + audio->snapshot_queued) {
					audio->snapshot_pos =
						(pos + audio->buffer_bytes - queued) %
						audio->buffer_bytes;
					audio->snapshot_dma = pos;
					audio->snapshot_queued = queued;
					audio->sample_fresh = true;
					disposition = Q2_PTR_ACCEPTED;
				} else {
					disposition = Q2_PTR_REGRESSED;
				}
			}
			if (unlikely(READ_ONCE(audio->diag_enabled)))
				x2000_diag_event(audio, caller, 0, before, fsr, after,
					tries + 1, disposition, previous,
					audio->snapshot_pos, 0);
			return audio->snapshot_pos;
		}
	}
	/*
	 * Keep the last coherent sample if DMA advances throughout the bounded
	 * retries. The next poll catches up without inventing a ring wrap.
	 */
	if (unlikely(READ_ONCE(audio->diag_enabled)))
		x2000_diag_event(audio, caller, 0, before, fsr, after, tries,
			Q2_PTR_RETRY_EXHAUSTED, previous, audio->snapshot_pos, 0);
	return audio->snapshot_pos;
}

/*
 * The next timer interval; the caller holds the stream lock. The timer
 * calls snd_pcm_period_elapsed() only when the accounted position crosses
 * a period boundary, so mode 1 sleeps until that crossing instead of
 * polling between boundaries. period_pos is below period_bytes here, so
 * the interval is at most one period plus the guard: under one buffer
 * (periods_min 4), which keeps the wrap arithmetic in x2000_poll() valid.
 * A callback that finds the boundary not yet crossed (the FIFO-bracketed
 * position can lag by a burst) re-arms for the rest, at least POLL_MIN_NS.
 * period_pos is only as fresh as the last callback: after PAUSE_RELEASE the
 * DMA has played an unaccounted part of the period (between that callback
 * and the pause), so the first interval is POLL_MIN_NS, and the callback's
 * accounting then gives the exact rest. The same holds after a callback
 * whose own position read was not accepted (x2000_position() kept the last
 * snapshot): the rest computed from it could be up to a period long.
 */
static u64 x2000_timer_ns(struct x2000_audio *audio)
{
	u64 ns;
	bool resync = audio->timer_resync;

	audio->timer_resync = false;
	if (!READ_ONCE(pointer_timer) || !audio->byte_rate ||
	    audio->period_pos >= audio->period_bytes)
		ns = audio->poll_ns;
	else if (resync)
		ns = POLL_MIN_NS;
	else
		ns = max_t(u64, POLL_MIN_NS, BOUNDARY_GUARD_NS +
			div_u64((u64)(audio->period_bytes - audio->period_pos) *
				NSEC_PER_SEC, audio->byte_rate));
	audio->timer_ns = ns;
	return ns;
}

static enum hrtimer_restart x2000_poll(struct hrtimer *timer)
{
	struct x2000_audio *audio = container_of(timer, struct x2000_audio, timer);
	struct snd_pcm_substream *substream = audio->substream;
	unsigned long flags;
	unsigned int pos, delta;
	bool running;
	u64 next = 0;

	/* trigger and this callback serialize through ALSA's stream lock.
	 * sync_stop/close cancel outside that lock before buffers are freed.
	 */
	if (WARN_ON_ONCE(!substream))
		return HRTIMER_NORESTART;
	snd_pcm_stream_lock_irqsave(substream, flags);
	if (audio->running) {
		audio->poll_count++;
		pos = x2000_position(audio, Q2_PTR_POSITION_POLL);
		delta = (pos + audio->buffer_bytes - audio->last_pos) %
			audio->buffer_bytes;
		audio->last_pos = pos;
		audio->period_pos += delta;
		if (audio->period_pos >= audio->period_bytes) {
			audio->period_pos %= audio->period_bytes;
			audio->period_count++;
			snd_pcm_period_elapsed_under_stream_lock(substream);
		}
		if (unlikely(READ_ONCE(audio->diag_enabled)))
			x2000_diag_event(audio, Q2_PTR_POLL_END, 0, 0, 0, 0, 0,
				0, 0, pos, delta);
	}
	running = audio->running;
	/*
	 * A trigger START/PAUSE_RELEASE that ran while this callback waited for
	 * the lock has re-armed the timer: forwarding an enqueued timer is not
	 * allowed, and the trigger's expiry stands. Both run under the lock.
	 */
	if (running && hrtimer_is_queued(timer))
		running = false;
	if (running) {
		if (!audio->sample_fresh)
			audio->timer_resync = true;
		next = x2000_timer_ns(audio);
		/* mode 1's interval is measured from this callback's position:
		 * count it from now, not from the expiry (which a late callback
		 * would turn into an early one)
		 */
		if (READ_ONCE(pointer_timer))
			hrtimer_set_expires(timer, ktime_add_ns(ktime_get(), next));
		else
			hrtimer_forward_now(timer, ns_to_ktime(next));
	}
	snd_pcm_stream_unlock_irqrestore(substream, flags);
	return running ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

static const struct snd_pcm_hardware x2000_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_PAUSE,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
	.rate_min = 44100,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = BUFFER_MAX,
	.period_bytes_min = 1024,
	.period_bytes_max = 32768,
	.periods_min = 4,
	.periods_max = 32,
};

static int x2000_open(struct snd_soc_component *component,
		     struct snd_pcm_substream *substream)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	if (READ_ONCE(audio->shutting_down))
		return -ESHUTDOWN;
	runtime->hw = x2000_pcm_hw;
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;
	ret = snd_pcm_hw_constraint_step(runtime, 0,
					SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret < 0)
		return ret;
	{
		unsigned long flags;
		spin_lock_irqsave(&audio->diag_lock, flags);
		audio->diag_opening = true;
		spin_unlock_irqrestore(&audio->diag_lock, flags);
	}
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(audio->clocks), audio->clocks);
	if (ret < 0) {
		unsigned long flags;
		spin_lock_irqsave(&audio->diag_lock, flags);
		audio->diag_opening = false;
		spin_unlock_irqrestore(&audio->diag_lock, flags);
		return ret;
	}
	writel(1, audio->base + DGRR);
	{
		unsigned long flags;
		spin_lock_irqsave(&audio->diag_lock, flags);
		audio->substream = substream;
		audio->diag_opening = false;
		spin_unlock_irqrestore(&audio->diag_lock, flags);
	}
	return 0;
}

static void x2000_stop_data(struct x2000_audio *audio)
{
	audio->running = false;
	writel(0, audio->base + DCM);
	writel(0, audio->base + FCR);
	writel(0, audio->base + DCR);
}

static void x2000_stop(struct x2000_audio *audio)
{
	x2000_stop_data(audio);
	writel(0, audio->base + CCR);
}

static int x2000_close(struct snd_soc_component *component,
		      struct snd_pcm_substream *substream)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);

	hrtimer_cancel(&audio->timer);
	x2000_stop(audio);
	writel(2, audio->base + DCR);
	writel(0, audio->base + DFCR);
	writel(BIT(9), audio->base + BTCLR);
	writel(0, audio->base + BST1);
	writel(0, audio->base + BTT0);
	{
		unsigned long flags;
		spin_lock_irqsave(&audio->diag_lock, flags);
		audio->substream = NULL;
		spin_unlock_irqrestore(&audio->diag_lock, flags);
	}
	clk_bulk_disable_unprepare(ARRAY_SIZE(audio->clocks), audio->clocks);
	return 0;
}

static int x2000_hw_params(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	unsigned long mclk = rate == 44100 ? 22579200 : 24576000;
	unsigned long actual;
	unsigned int width = params_width(params), code;
	int ret;

	if (params_channels(params) != 2)
		return -EINVAL;
	switch (width) {
	case 16:
		code = 3;
		break;
	case 24:
		code = 6;
		break;
	case 32:
		code = 7;
		break;
	default:
		return -EINVAL;
	}
	ret = clk_set_rate(audio->clocks[1].clk, mclk);
	if (ret)
		return ret;
	/* The fractional divider may round by a few Hz (22,579,195 Hz on
	 * Q2 hardware). Reject errors beyond 100 ppm, not integer rounding.
	 */
	actual = clk_get_rate(audio->clocks[1].clk);
	if (abs((long)actual - (long)mclk) > mclk / 10000)
		return -EINVAL;

	audio->buffer_bytes = params_buffer_bytes(params);
	audio->period_bytes = params_period_bytes(params);
	/* Four observations per period where possible, bounded to 1--5 ms.
	 * This is still pointer polling, not a DMA interrupt protocol.
	 * Preserve synchronous pause/stop and the coherent FIFO snapshot.
	 */
	audio->poll_ns = clamp_t(u64,
		div_u64((u64)params_period_size(params) * NSEC_PER_SEC,
			params_rate(params) * 4), POLL_MIN_NS, POLL_MAX_NS);
	audio->byte_rate = params_rate(params) *
		(params_period_bytes(params) / params_period_size(params));
	writel(0, audio->base + CCR);
	writel(1, audio->base + TCTL);
	writel(0, audio->base + TCGR);
	writel(1, audio->base + TLCR);
	/* Stereo I2S, 64 bit clocks per frame, CPU is clock provider. */
	writel(BIT(17) | (code << 8) | (4 << 2) | 1, audio->base + TCFG);
	writel(((mclk / (rate * 64)) << 16) | 3, audio->base + TDIV);
	writel((1 << 12) | (code << 8), audio->base + DFCR);
	/* ALSA clears the buffer after hw_params only for drivers without a
	 * copy op (snd_pcm_hw_params()); keep that clear, with its size.
	 */
	if (substream->runtime->dma_area) {
		size_t size = substream->runtime->dma_bytes;

		if (params->info & SNDRV_PCM_INFO_MMAP)
			size = PAGE_ALIGN(size);
		memset(substream->runtime->dma_area, 0, size);
	}
	return 0;
}

static int x2000_prepare(struct snd_soc_component *component,
			struct snd_pcm_substream *substream)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);

	hrtimer_cancel(&audio->timer);
	/* snd_pcm_recover() after an XRUN prepares from the XRUN state */
	if (substream->runtime->state == SNDRV_PCM_STATE_XRUN)
		audio->xrun_prepares++;
	/* A drop/prepare may be followed by close without another START
	 * (for example, selection of a different PCM width). The old HP
	 * domain still needs its transmitter through DAPM power-down.
	 * Retire/reset data here without taking that clock away. Initial
	 * hw_params already leaves TEN off; final close disables it.
	 */
	x2000_stop_data(audio);
	writel(2, audio->base + DCR);
	writel(BIT(2), audio->base + FCR);
	udelay(100);
	writel(0x3c, audio->base + DSR);
	/* Stock descriptor: incrementing buffer, 32-byte burst, self link. */
	audio->desc[0] = cpu_to_le32(0x120);
	audio->desc[1] = cpu_to_le32(substream->runtime->dma_addr);
	audio->desc[2] = cpu_to_le32(audio->desc_dma);
	audio->desc[3] = cpu_to_le32(audio->buffer_bytes / 32);
	dma_wmb();
	writel(audio->desc_dma, audio->base + DDA);
	writel(384, audio->base + FAS);
	writel(8 << 16, audio->base + FFR);
	writel(0xfff, audio->base + BFCR1);
	writel(0x7fff, audio->base + BFCR2);
	writel(BIT(8) | BIT(17), audio->base + BFCR0);
	writel(9 << 10, audio->base + BST1);
	writel(9 << 5, audio->base + BTT0);
	writel(BIT(9), audio->base + BTSET);
	if (unlikely(READ_ONCE(audio->diag_enabled)))
		x2000_diag_event(audio, Q2_PTR_PREPARE, 0, 0, 0, 0, 0, 0,
			0, audio->snapshot_pos, 0);
	audio->snapshot_pos = 0;
	audio->snapshot_dma = 0;
	audio->snapshot_queued = 0;
	audio->last_pos = 0;
	audio->period_pos = 0;
	return 0;
}

static int x2000_trigger(struct snd_soc_component *component,
			struct snd_pcm_substream *substream, int cmd)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);

	if (unlikely(READ_ONCE(audio->diag_enabled)))
		x2000_diag_event(audio, Q2_PTR_TRIGGER, cmd, 0, 0, 0, 0, 0,
			0, audio->snapshot_pos, 0);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (READ_ONCE(audio->shutting_down))
			return -ESHUTDOWN;
		writel(readl(audio->base + DFCR) | 1, audio->base + DFCR);
		writel(1, audio->base + DCR);
		writel(1, audio->base + FCR);
		writel(8, audio->base + CCR);
		audio->running = true;
		audio->timer_resync = cmd == SNDRV_PCM_TRIGGER_PAUSE_RELEASE;
		hrtimer_start(&audio->timer, ns_to_ktime(x2000_timer_ns(audio)),
			      HRTIMER_MODE_REL);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* The codec still owns its playback clock domain until DAPM
		 * powers HP/ASP down and codec hw_free switches EXT to RCO.
		 * Stop DMA/FIFO immediately, retaining transmitter state through
		 * that sequence (including close after pause). close/hw_params
		 * remain the boundaries that disable/reconfigure transmission.
		 * TEN is not itself a documented external MCLK gate; hardware
		 * qualification must establish this lifetime's effect on PDN.
		 */
		x2000_stop_data(audio);
		return 0;
	default:
		return -EINVAL;
	}
}

static int x2000_sync_stop(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);

	u32 status;
	int ret;

	hrtimer_cancel(&audio->timer);
	if (unlikely(READ_ONCE(audio->diag_enabled))) {
		unsigned long flags;

		snd_pcm_stream_lock_irqsave(substream, flags);
		x2000_diag_event(audio, Q2_PTR_SYNC_STOP, 0, 0, 0, 0, 0, 0,
			0, audio->snapshot_pos, 0);
		snd_pcm_stream_unlock_irqrestore(substream, flags);
	}
	ret = readl_poll_timeout(audio->base + DSR, status, !(status & 2), 10, 10000);
	if (ret) {
		dev_err(component->dev, "DMA stop timed out; resetting channel\n");
		writel(2, audio->base + DCR);
		udelay(100);
	}
	return ret;
}

static snd_pcm_uframes_t x2000_pointer(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);

	return bytes_to_frames(substream->runtime, x2000_position(audio, Q2_PTR_POSITION_ALSA));
}

/* Words, then the bytes left; dst and src are equally aligned modulo 4. */
static void x2000_store_scalar(u8 *dst, const u8 *src, unsigned long n)
{
	for (; n && ((unsigned long)dst & 3); n--)
		WRITE_ONCE(*dst++, *src++);
	for (; n >= 32; n -= 32, dst += 32, src += 32) {
		const u32 *s = (const u32 *)src;
		u32 *d = (u32 *)dst;

		WRITE_ONCE(d[0], s[0]);
		WRITE_ONCE(d[1], s[1]);
		WRITE_ONCE(d[2], s[2]);
		WRITE_ONCE(d[3], s[3]);
		WRITE_ONCE(d[4], s[4]);
		WRITE_ONCE(d[5], s[5]);
		WRITE_ONCE(d[6], s[6]);
		WRITE_ONCE(d[7], s[7]);
	}
	for (; n >= 4; n -= 4, dst += 4, src += 4)
		WRITE_ONCE(*(u32 *)dst, *(const u32 *)src);
	for (; n; n--)
		WRITE_ONCE(*dst++, *src++);
}

#ifdef CONFIG_CPU_HAS_MSA
/* One 64-byte block: four ld.d from the cached bounce, then four st.d. */
static __always_inline void x2000_store_line_msa(u8 *dst, const u8 *src)
{
	__asm__ __volatile__(
		".set push\n\t"
		".set hardfloat\n\t.set fp=64\n\t.set msa\n\t"
		"ld.d $w0, 0(%[s])\n\t"
		"ld.d $w1, 16(%[s])\n\t"
		"ld.d $w2, 32(%[s])\n\t"
		"ld.d $w3, 48(%[s])\n\t"
		"st.d $w0, 0(%[d])\n\t"
		"st.d $w1, 16(%[d])\n\t"
		"st.d $w2, 32(%[d])\n\t"
		"st.d $w3, 48(%[d])\n\t"
		".set pop\n\t"
		: : [d] "r" (dst), [s] "r" (src) : "memory");
}

/*
 * n a multiple of COPY_LINE, dst and src 16-byte aligned. One kernel MSA
 * section (preemption and bottom halves off, at most COPY_BOUNCE bytes).
 * Returns n, or 0 when kernel_msa_begin() refuses and nothing was stored.
 */
static unsigned long x2000_store_msa(u8 *dst, const u8 *src, unsigned long n)
{
	unsigned long i;

	if (!kernel_msa_begin())
		return 0;
	for (i = 0; i < n; i += COPY_LINE)
		x2000_store_line_msa(dst + i, src + i);
	kernel_msa_end();
	return n;
}
#else
static unsigned long x2000_store_msa(u8 *dst, const u8 *src, unsigned long n)
{
	return 0;
}
#endif

/*
 * Store n bytes of src (the bounce) to dst (the uncached DMA buffer); dst
 * and src are equally aligned modulo 16. Scalar up to dst's 16-byte
 * boundary, whole 64-byte blocks with MSA when msa is set and allowed,
 * scalar for the rest. Each byte is stored once; the buffer is not read.
 */
static void x2000_store(struct x2000_audio *audio, u8 *dst, const u8 *src,
			unsigned long n, bool msa)
{
	unsigned long head = min(n, -(unsigned long)dst & 15);
	unsigned long body, done = 0;

	x2000_store_scalar(dst, src, head);
	dst += head;
	src += head;
	n -= head;
	body = n & ~(COPY_LINE - 1UL);
	if (msa && body)
		done = x2000_store_msa(dst, src, body);
	x2000_store_scalar(dst + done, src + done, n - done);
	audio->copy_msa_bytes += done;
	audio->copy_word_bytes += head + n - done;
}

/*
 * Bytes of dst (uncached) that differ from src, read back after wmb(); word
 * loads where aligned (an uncached load costs about 80 ns whatever its
 * width). dst and src are equally aligned modulo 4.
 */
static unsigned long x2000_verify(const u8 *dst, const u8 *src, unsigned long n)
{
	unsigned long bad = 0, i = 0;

	for (; i < n && ((unsigned long)(dst + i) & 3); i++)
		bad += READ_ONCE(dst[i]) != src[i];
	for (; i + 4 <= n; i += 4) {
		u32 x = READ_ONCE(*(const u32 *)(dst + i)) ^ *(const u32 *)(src + i);

		bad += !!(x & 0xff) + !!(x & 0xff00) + !!(x & 0xff0000) + !!(x & 0xff000000);
	}
	for (; i < n; i++)
		bad += READ_ONCE(dst[i]) != src[i];
	return bad;
}

/*
 * write() into the DMA buffer. ALSA calls this outside the stream lock, in
 * task context, for bytes [pos, pos + bytes) of the region the application
 * may fill; it moves appl_ptr over them only after the call returns.
 *
 * Why the DMA cannot read a stale byte: no cached alias of the buffer is
 * made. Every mode stores through runtime->dma_area, the uncached mapping
 * the default path uses, so no cache line ever holds buffer bytes and no
 * cache maintenance is involved. The bounce is cached, but the DMA never
 * reads it, and each chunk is complete in the bounce (copy_from_iter()
 * returned) before any of it is stored out. Modes 1 and 2 issue wmb()
 * (SYNC) before returning, so every store has left the write buffer before
 * ALSA moves appl_ptr; the default path relies on the write buffer
 * draining by itself. The bounce data sits at the destination's offset
 * modulo 16, so every MSA access is 16-byte aligned on both sides.
 */
static int x2000_copy(struct snd_soc_component *component,
		      struct snd_pcm_substream *substream, int channel,
		      unsigned long pos, struct iov_iter *iter,
		      unsigned long bytes)
{
	struct x2000_audio *audio = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u8 *dst = runtime->dma_area + pos +
		channel * (runtime->dma_bytes / runtime->channels);
	unsigned int mode = READ_ONCE(pcm_copy);
	int ret = 0;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return copy_to_iter(dst, bytes, iter) == bytes ? 0 : -EFAULT;
	if (mode < 1 || mode > 3) {
		if (copy_from_iter(dst, bytes, iter) != bytes)
			return -EFAULT;
		audio->copy_direct_bytes += bytes;
		return 0;
	}
	mutex_lock(&audio->copy_lock);
	while (bytes) {
		unsigned long lead = (unsigned long)dst & 15;
		unsigned long n = min(bytes, COPY_BOUNCE - lead);
		u8 *b = audio->bounce + lead;

		if (copy_from_iter(b, n, iter) != n) {
			ret = -EFAULT;
			break;
		}
		x2000_store(audio, dst, b, n, mode != 2);
		if (mode == 3) {
			wmb();
			audio->copy_verify_mismatch += x2000_verify(dst, b, n);
			audio->copy_verify_bytes += n;
		}
		dst += n;
		bytes -= n;
	}
	mutex_unlock(&audio->copy_lock);
	wmb();
	return ret;
}

static int x2000_pcm_new(struct snd_soc_component *component,
			struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       component->dev, BUFFER_MAX, BUFFER_MAX);
	return 0;
}

static const struct snd_soc_component_driver x2000_component = {
	.name = "x2000-audio",
	.open = x2000_open,
	.close = x2000_close,
	.hw_params = x2000_hw_params,
	.prepare = x2000_prepare,
	.trigger = x2000_trigger,
	.sync_stop = x2000_sync_stop,
	.pointer = x2000_pointer,
	.copy = x2000_copy,
	.pcm_new = x2000_pcm_new,
};

static int x2000_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops x2000_dai_ops = {
	.set_fmt = x2000_set_fmt,
};

static struct snd_soc_dai_driver x2000_dai = {
	.name = "x2000-baic1",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &x2000_dai_ops,
};

static void x2000_debug_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int x2000_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct x2000_audio *audio;
	struct dentry *debug;
	int ret;

	audio = devm_kzalloc(dev, sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return -ENOMEM;
	audio->diag = devm_kzalloc(dev, sizeof(*audio->diag), GFP_KERNEL);
	if (!audio->diag)
		return -ENOMEM;
	spin_lock_init(&audio->diag_lock);
	mutex_init(&audio->copy_lock);
	/* kmalloc() aligns a power-of-two size to itself: 16-byte MSA access */
	audio->bounce = devm_kmalloc(dev, COPY_BOUNCE, GFP_KERNEL);
	if (!audio->bounce)
		return -ENOMEM;
	if (WARN_ON((unsigned long)audio->bounce & 15))
		return -EINVAL;
	audio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(audio->base))
		return PTR_ERR(audio->base);
	audio->clocks[0].id = "audio";
	audio->clocks[1].id = "i2s1";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(audio->clocks), audio->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get clocks\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(29));
	if (ret)
		return ret;
	audio->desc = dmam_alloc_coherent(dev, 16, &audio->desc_dma, GFP_KERNEL);
	if (!audio->desc)
		return -ENOMEM;
	hrtimer_setup(&audio->timer, x2000_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	debug = debugfs_create_dir("x2000-audio", NULL);
	if (audio->diag && IS_ERR_OR_NULL(debug))
		return debug ? PTR_ERR(debug) : -ENOMEM;
	if (!IS_ERR_OR_NULL(debug)) {
		debugfs_create_u32("poll_ns", 0444, debug, &audio->poll_ns);
		debugfs_create_u32("poll_count", 0444, debug, &audio->poll_count);
		debugfs_create_u32("timer_ns", 0444, debug, &audio->timer_ns);
		debugfs_create_u32("period_count", 0444, debug, &audio->period_count);
		debugfs_create_u64("copy_msa_bytes", 0444, debug, &audio->copy_msa_bytes);
		debugfs_create_u64("copy_word_bytes", 0444, debug, &audio->copy_word_bytes);
		debugfs_create_u64("copy_direct_bytes", 0444, debug, &audio->copy_direct_bytes);
		debugfs_create_u64("copy_verify_bytes", 0444, debug, &audio->copy_verify_bytes);
		debugfs_create_u64("copy_verify_mismatch", 0444, debug, &audio->copy_verify_mismatch);
		debugfs_create_u32("xrun_prepares", 0444, debug, &audio->xrun_prepares);
		ret = devm_add_action_or_reset(dev, x2000_debug_remove, debug);
		if (ret)
			return ret;
	}
	debugfs_create_bool("pointer_observer_enabled", 0444, debug,
		&audio->diag_enabled);
	{
		struct dentry *f = debugfs_create_file("pointer_observer", 0400,
			debug, audio, &x2000_diag_fops);

		if (IS_ERR_OR_NULL(f))
			return f ? PTR_ERR(f) : -ENOMEM;
		f = debugfs_create_file("pointer_observer_control", 0200,
			debug, audio, &x2000_diag_control_fops);
		if (IS_ERR_OR_NULL(f))
			return f ? PTR_ERR(f) : -ENOMEM;
	}
	platform_set_drvdata(pdev, audio);
	return devm_snd_soc_register_component(dev, &x2000_component, &x2000_dai, 1);
}

static void x2000_audio_shutdown(struct platform_device *pdev)
{
	struct x2000_audio *audio = platform_get_drvdata(pdev);
	struct snd_pcm_substream *substream;
	unsigned long flags;
	u32 status;
	int ret;

	/* A forced reboot can leave an open PCM stream. Prevent another START
	 * and serialize with the trigger/poll path before cancelling its timer.
	 */
	WRITE_ONCE(audio->shutting_down, true);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(audio->clocks), audio->clocks);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable clocks for DMA shutdown: %d\n", ret);
		return;
	}
	substream = READ_ONCE(audio->substream);
	if (substream) {
		snd_pcm_stream_lock_irqsave(substream, flags);
		x2000_stop(audio);
		snd_pcm_stream_unlock_irqrestore(substream, flags);
	} else {
		x2000_stop(audio);
	}
	hrtimer_cancel(&audio->timer);
	ret = readl_poll_timeout(audio->base + DSR, status, !(status & 2), 10, 10000);
	if (ret)
		dev_warn(&pdev->dev, "DMA shutdown timed out; resetting channel\n");
	writel(2, audio->base + DCR);
	writel(0, audio->base + DFCR);
	writel(BIT(9), audio->base + BTCLR);
	clk_bulk_disable_unprepare(ARRAY_SIZE(audio->clocks), audio->clocks);
}

/* The ASoC card's device link suspends PCM/DAPM before its components.
 * Refuse an undrained channel; DDR may become inaccessible immediately
 * after all device callbacks finish. An open suspended PCM is recoverable
 * with normal ALSA prepare; never restart its old DMA on system resume.
 */
static int x2000_audio_suspend(struct device *dev)
{
	struct x2000_audio *audio = dev_get_drvdata(dev);
	u32 status;
	int ret;

	if (!READ_ONCE(audio->substream))
		return 0;
	if (READ_ONCE(audio->running))
		return -EBUSY;
	hrtimer_cancel(&audio->timer);
	ret = readl_poll_timeout(audio->base + DSR, status, !(status & 2), 10, 10000);
	if (ret)
		return ret;
	clk_bulk_disable_unprepare(ARRAY_SIZE(audio->clocks), audio->clocks);
	audio->sleep_clock_off = true;
	return 0;
}

static int x2000_audio_resume(struct device *dev)
{
	struct x2000_audio *audio = dev_get_drvdata(dev);
	int ret;

	if (!audio->sleep_clock_off)
		return 0;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(audio->clocks), audio->clocks);
	if (!ret)
		audio->sleep_clock_off = false;
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_audio_pm_ops,
			      x2000_audio_suspend, x2000_audio_resume);

static const struct of_device_id x2000_audio_of_match[] = {
	{ .compatible = "ingenic,x2000-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_audio_of_match);

static struct platform_driver x2000_audio_driver = {
	.probe = x2000_audio_probe,
	.shutdown = x2000_audio_shutdown,
	.driver = {
		.name = "x2000-audio",
		.pm = pm_sleep_ptr(&x2000_audio_pm_ops),
		.of_match_table = x2000_audio_of_match,
	},
};
module_platform_driver(x2000_audio_driver);

MODULE_DESCRIPTION("Ingenic X2000 BAIC1 playback");
MODULE_LICENSE("GPL");
