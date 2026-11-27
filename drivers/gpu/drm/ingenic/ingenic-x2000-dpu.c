// SPDX-License-Identifier: GPL-2.0-or-later
/* X2000 direct composer, reconstructed from the Q2 stock driver and SDK.
 * Two DMA descriptor sets: never overwrite the set currently scanning out.
 * FRM_CFG_ADDR queues the next set; FRM_CHAIN_SITE confirms hardware uptake.
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_of.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#define DPU_ADDR 0x0000
#define DPU_START 0x0004
#define DPU_CTRL 0x2000
#define DPU_STATUS 0x2004
#define DPU_CLEAR 0x2008
#define DPU_IRQ_ENABLE 0x200c
#define DPU_IRQ 0x2010
#define DPU_SITE 0x2200
#define DPU_DISPLAY_END BIT(17)

struct x2000_desc {
	u32 frame[16];
	u32 layer[16];
};

struct x2000_dpu {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	void __iomem *regs;
	struct clk *clk;
	struct x2000_desc *desc;
	dma_addr_t dma;
	unsigned int queued;
	bool changed, clock_on, event_vblank, stop_failed;
	struct drm_pending_vblank_event *event;
	u32 event_site;
};

static struct x2000_dpu *to_dpu(struct drm_device *drm)
{
	return container_of(drm, struct x2000_dpu, drm);
}

static int x2000_plane_check(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *c;
	int ret;

	if (!p->crtc)
		return 0;
	c = drm_atomic_get_crtc_state(state, p->crtc);
	if (IS_ERR(c))
		return PTR_ERR(c);
	ret = drm_atomic_helper_check_plane_state(p, c, DRM_PLANE_NO_SCALING,
						 DRM_PLANE_NO_SCALING, false, true);
	if (ret || !p->visible)
		return ret;
	if (p->src_x || p->src_y || p->crtc_x || p->crtc_y ||
	    p->crtc_w != 320 || p->crtc_h != 375 || (p->fb->pitches[0] & 3))
		return -EINVAL;
	return 0;
}

static void x2000_plane_update(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct x2000_dpu *d = to_dpu(plane->dev);
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, plane);
	struct x2000_desc *desc;
	dma_addr_t addr;
	unsigned int next = d->queued ^ 1;

	if (!p->crtc || !p->fb || !p->visible)
		return;
	desc = &d->desc[next];
	addr = d->dma + next * sizeof(*desc);
	memset(desc, 0, sizeof(*desc));
	desc->frame[0] = addr;
	desc->frame[1] = 375 << 16 | 320;
	desc->frame[2] = BIT(2); /* direct output, continuous */
	desc->frame[4] = 320;
	desc->frame[5] = addr + offsetof(struct x2000_desc, layer);
	/* Disabled layers still need distinct hardware order values. */
	desc->frame[9] = BIT(4) | (1 << 10) | (2 << 12) | (3 << 14);
	/* Layer 0 enabled at order 0; layers 1/2/3 disabled at orders 1/2/3. */
	desc->frame[10] = DPU_DISPLAY_END;
	desc->layer[0] = desc->frame[1];
	desc->layer[1] = 4 << 16 | 255; /* RGB888 in 32-bit storage */
	desc->layer[2] = drm_fb_dma_get_gem_addr(p->fb, p, 0);
	desc->layer[9] = p->fb->pitches[0] / 4;
	dma_wmb();
	d->queued = next;
	d->changed = true;
}

static void x2000_flush(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct x2000_dpu *d = to_dpu(crtc->dev);
	struct drm_crtc_state *c = drm_atomic_get_new_crtc_state(state, crtc);
	unsigned long flags;
	u32 addr = d->dma + d->queued * sizeof(*d->desc);

	spin_lock_irqsave(&d->drm.event_lock, flags);
	if (c->event) {
		WARN_ON(d->event);
		d->event = c->event;
		d->event_site = addr;
		d->event_vblank = !drm_crtc_vblank_get(crtc);
		c->event = NULL;
	}
	if (d->changed && d->clock_on) {
		writel(addr, d->regs + DPU_ADDR);
		d->changed = false;
	}
	spin_unlock_irqrestore(&d->drm.event_lock, flags);
}

static enum drm_mode_status x2000_mode_valid(struct drm_crtc *crtc,
					    const struct drm_display_mode *m)
{
	/* Only the stock Q2 video mode has been recovered/validated. */
	return m->hdisplay == 320 && m->hsync_start == 340 && m->hsync_end == 360 &&
	       m->htotal == 384 && m->vdisplay == 375 && m->vsync_start == 391 &&
	       m->vsync_end == 393 && m->vtotal == 405 && m->clock == 9331 &&
	       !(m->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN)) ?
	       MODE_OK : MODE_BAD;
}

static void x2000_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct x2000_dpu *d = to_dpu(crtc->dev);
	int ret;

	ret = clk_set_rate(d->clk, crtc->state->adjusted_mode.clock * 1000);
	if (!ret)
		ret = clk_prepare_enable(d->clk);
	if (ret) {
		drm_err(&d->drm, "cannot enable LCD clock: %d\n", ret);
		return;
	}
	d->clock_on = true;
	/* Stock TFT timing and RGB output configuration. */
	writel(1, d->regs + 0x8000);
	writel(20 << 16 | 384, d->regs + 0x9000);
	writel(2 << 16 | 405, d->regs + 0x9004);
	writel(44 << 16 | 364, d->regs + 0x9008);
	writel(14 << 16 | 389, d->regs + 0x900c);
	writel(0, d->regs + 0x9010);
	writel(0, d->regs + 0x2014);
	writel(~0U, d->regs + DPU_CLEAR);
	writel(DPU_DISPLAY_END, d->regs + DPU_IRQ_ENABLE);
	drm_crtc_vblank_on(crtc);
	spin_lock_irq(&d->drm.event_lock);
	if (d->event && !d->event_vblank)
		d->event_vblank = !drm_crtc_vblank_get(crtc);
	spin_unlock_irq(&d->drm.event_lock);
	/* Initial atomic_flush runs before the LCD clock is enabled.
	 * Queue its completed descriptor now, before starting DMA.
	 */
	writel(d->dma + d->queued * sizeof(*d->desc), d->regs + DPU_ADDR);
	d->changed = false;
	writel(1, d->regs + DPU_START);
}

static void x2000_disable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct x2000_dpu *d = to_dpu(crtc->dev);
	struct drm_crtc_state *c = drm_atomic_get_new_crtc_state(state, crtc);
	u32 val;

	d->stop_failed = false;
	writel(8, d->regs + DPU_CTRL);
	if (readl_poll_timeout(d->regs + DPU_STATUS, val, !(val & BIT(0)), 100, 100000)) {
		drm_warn(&d->drm, "composer stop timed out\n");
		writel(1, d->regs + DPU_CTRL);
		d->stop_failed = readl_poll_timeout(d->regs + DPU_STATUS, val,
						 !(val & BIT(0)), 100, 100000);
	}
	writel(0, d->regs + DPU_IRQ_ENABLE);
	drm_crtc_vblank_off(crtc);
	spin_lock_irq(&d->drm.event_lock);
	if (d->event) {
		drm_crtc_send_vblank_event(crtc, d->event);
		d->event = NULL;
		if (d->event_vblank)
			drm_crtc_vblank_put(crtc);
		d->event_vblank = false;
	}
	if (c->event) {
		drm_crtc_send_vblank_event(crtc, c->event);
		c->event = NULL;
	}
	spin_unlock_irq(&d->drm.event_lock);
	if (d->clock_on) {
		clk_disable_unprepare(d->clk);
		d->clock_on = false;
	}
}

static irqreturn_t x2000_irq(int irq, void *data)
{
	struct x2000_dpu *d = data;
	u32 status = readl(d->regs + DPU_IRQ);

	if (!(status & DPU_DISPLAY_END))
		return IRQ_NONE;
	writel(status, d->regs + DPU_CLEAR);
	drm_crtc_handle_vblank(&d->crtc);
	spin_lock(&d->drm.event_lock);
	/* Do not complete a flip for a stale, already pending frame interrupt. */
	if (d->event && readl(d->regs + DPU_SITE) == d->event_site) {
		drm_crtc_send_vblank_event(&d->crtc, d->event);
		d->event = NULL;
		if (d->event_vblank)
			drm_crtc_vblank_put(&d->crtc);
		d->event_vblank = false;
	}
	spin_unlock(&d->drm.event_lock);
	return IRQ_HANDLED;
}

/* Keep display-end enabled for descriptor retirement even without vblank users. */
static int x2000_enable_vblank(struct drm_crtc *crtc) { return 0; }
static void x2000_disable_vblank(struct drm_crtc *crtc) { }

static const struct drm_plane_funcs x2000_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};
static const struct drm_plane_helper_funcs x2000_plane_helpers = {
	.atomic_check = x2000_plane_check,
	.atomic_update = x2000_plane_update,
};
static const struct drm_crtc_funcs x2000_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = x2000_enable_vblank,
	.disable_vblank = x2000_disable_vblank,
};
static int x2000_crtc_check(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *c = drm_atomic_get_new_crtc_state(state, crtc);

	return c->active ? drm_atomic_helper_check_crtc_primary_plane(c) : 0;
}

static const struct drm_crtc_helper_funcs x2000_crtc_helpers = {
	.atomic_check = x2000_crtc_check,
	.mode_valid = x2000_mode_valid,
	.atomic_enable = x2000_enable,
	.atomic_disable = x2000_disable,
	.atomic_flush = x2000_flush,
};
static const struct drm_encoder_funcs x2000_encoder_funcs = { .destroy = drm_encoder_cleanup };
static const struct drm_mode_config_funcs x2000_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};
DEFINE_DRM_GEM_DMA_FOPS(x2000_fops);
static const struct drm_driver x2000_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.name = "x2000-dpu", .desc = "Ingenic X2000 DPU", .major = 1,
	.fops = &x2000_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int x2000_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct x2000_dpu *d;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	static const u32 formats[] = { DRM_FORMAT_XRGB8888 };
	int ret, irq;

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "DSI bridge\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	d = devm_drm_dev_alloc(dev, &x2000_driver, struct x2000_dpu, drm);
	if (IS_ERR(d))
		return PTR_ERR(d);
	platform_set_drvdata(pdev, d);
	d->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->regs))
		return PTR_ERR(d->regs);
	d->clk = devm_clk_get(dev, "lcd");
	if (IS_ERR(d->clk))
		return PTR_ERR(d->clk);
	d->desc = dmam_alloc_coherent(dev, 2 * sizeof(*d->desc), &d->dma, GFP_KERNEL);
	if (!d->desc)
		return -ENOMEM;
	ret = drmm_mode_config_init(&d->drm);
	if (ret)
		return ret;
	d->drm.mode_config.min_width = d->drm.mode_config.max_width = 320;
	d->drm.mode_config.min_height = d->drm.mode_config.max_height = 375;
	d->drm.mode_config.funcs = &x2000_config_funcs;
	ret = drm_universal_plane_init(&d->drm, &d->plane, 1, &x2000_plane_funcs,
				       formats, ARRAY_SIZE(formats), NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&d->plane, &x2000_plane_helpers);
	ret = drm_crtc_init_with_planes(&d->drm, &d->crtc, &d->plane, NULL, &x2000_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&d->crtc, &x2000_crtc_helpers);
	ret = drm_encoder_init(&d->drm, &d->encoder, &x2000_encoder_funcs, DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;
	d->encoder.possible_crtcs = 1;
	ret = drm_bridge_attach(&d->encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;
	connector = drm_bridge_connector_init(&d->drm, &d->encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);
	ret = drm_connector_attach_encoder(connector, &d->encoder);
	if (ret)
		return ret;
	ret = drm_vblank_init(&d->drm, 1);
	if (ret)
		return ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	writel(0, d->regs + DPU_IRQ_ENABLE);
	ret = devm_request_irq(dev, irq, x2000_irq, 0, dev_name(dev), d);
	if (ret)
		return ret;
	drm_mode_config_reset(&d->drm);
	ret = drm_dev_register(&d->drm, 0);
	if (!ret)
		drm_client_setup(&d->drm, NULL);
	return ret;
}

static void x2000_remove(struct platform_device *pdev)
{
	struct x2000_dpu *d = platform_get_drvdata(pdev);

	drm_dev_unregister(&d->drm);
	drm_atomic_helper_shutdown(&d->drm);
}
static void x2000_shutdown(struct platform_device *pdev)
{
	struct x2000_dpu *d = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&d->drm);
}
static int x2000_suspend(struct device *dev)
{
	struct x2000_dpu *d = dev_get_drvdata(dev);

	int ret = drm_mode_config_helper_suspend(&d->drm);

	if (!ret && d->stop_failed) {
		drm_mode_config_helper_resume(&d->drm);
		return -EBUSY;
	}
	return ret;
}

static int x2000_resume(struct device *dev)
{
	struct x2000_dpu *d = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&d->drm);
}
static DEFINE_SIMPLE_DEV_PM_OPS(x2000_pm_ops, x2000_suspend, x2000_resume);

static const struct of_device_id x2000_match[] = { { .compatible = "ingenic,x2000-dpu" }, { } };
MODULE_DEVICE_TABLE(of, x2000_match);
static struct platform_driver x2000_platform_driver = {
	.probe = x2000_probe, .remove = x2000_remove, .shutdown = x2000_shutdown,
	.driver = { .name = "ingenic-x2000-dpu", .of_match_table = x2000_match,
		.pm = pm_sleep_ptr(&x2000_pm_ops) },
};
module_platform_driver(x2000_platform_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic X2000 direct composer DRM driver");
