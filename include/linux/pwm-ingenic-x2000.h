/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Looping DMA waveform output for one X2000 PWM channel.
 *
 * Each 32-bit word in the buffer describes one PWM period: the high count
 * in bits 31..16 and the low count in bits 15..0, in PWM input clocks
 * (prescaler 1). The buffer must be DMA-coherent memory of
 * x2000_pwm_dma_device(), word-aligned, a multiple of four words long and
 * at most X2000_PWM_DMA_MAX_WORDS words.
 *
 * x2000_pwm_dma_prepare() and x2000_pwm_dma_release() may sleep.
 * x2000_pwm_dma_enable() and x2000_pwm_dma_disable() may be called from
 * atomic context. Disable stops DMA fetching; the waveform can drain until
 * release. After disable, the hardware may still read the buffer until
 * x2000_pwm_dma_release() or the next prepare
 * returns 0; do not free it before then.
 */
#ifndef __LINUX_PWM_INGENIC_X2000_H
#define __LINUX_PWM_INGENIC_X2000_H

#include <linux/types.h>

struct device;
struct pwm_device;

/* Conservative until the count register's width is measured. */
#define X2000_PWM_DMA_MAX_WORDS 65532

static inline u32 x2000_pwm_dma_word(u16 high, u16 low)
{
	return (u32)high << 16 | low;
}

struct device *x2000_pwm_dma_device(struct pwm_device *pwm);
unsigned long x2000_pwm_clock_rate(struct pwm_device *pwm);
int x2000_pwm_dma_prepare(struct pwm_device *pwm, dma_addr_t addr, u32 words);
void x2000_pwm_dma_enable(struct pwm_device *pwm);
void x2000_pwm_dma_disable(struct pwm_device *pwm);
int x2000_pwm_dma_release(struct pwm_device *pwm);

#endif
