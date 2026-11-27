// SPDX-License-Identifier: GPL-2.0-or-later
/* H0209S on Shanling Q2. Commands, delays, rails and reset wiring recovered
 * from stock lcd_h0209s.ko / its loader; no guessed panel physical dimensions.
 * Stock initialization packet digest:
 * e726ec822393edd6c1d5df5a84be76f9c1e2fad657228f3990e854fe584e1583
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

struct q2_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
	struct regulator *vdd28, *vdd33;
};

struct q2_command {
	u8 type, len;
	u8 data[33];
};

static const struct q2_command q2_init_commands[] = {
	{ 0x39, 2, { 0xde, 0x00 } },
	{ 0x39, 3, { 0xdf, 0x98, 0x55 } },
	{ 0x39, 2, { 0xb2, 0x2c } },
	{ 0x39, 5, { 0xb7, 0x01, 0x3d, 0x01, 0x65 } },
	{ 0x39, 7, { 0xbb, 0x23, 0x64, 0xe3, 0x34, 0x3e, 0xf3 } },
	{ 0x39, 5, { 0xbc, 0x00, 0x1a, 0xf3, 0xc0 } },
	{ 0x39, 3, { 0xc0, 0x22, 0xc1 } },
	{ 0x39, 12, { 0xc3, 0x00, 0x01, 0xca, 0x0b, 0x08, 0x48, 0x06, 0x04, 0x62, 0x30, 0x30 } },
	{ 0x39, 12, { 0xc4, 0x40, 0x00, 0xa8, 0x77, 0x41, 0x04, 0x07, 0x16, 0x41, 0x04, 0x07 } },
	{ 0x39, 33, { 0xc8, 0x3f, 0x2b, 0x20, 0x1a, 0x1b, 0x1d, 0x19, 0x1a, 0x1a, 0x1a, 0x17, 0x0c, 0x09, 0x05, 0x02, 0x02, 0x3f, 0x2b, 0x20, 0x1a, 0x1b, 0x1d, 0x19, 0x1a, 0x1a, 0x1a, 0x17, 0x0c, 0x09, 0x05, 0x02, 0x02 } },
	{ 0x39, 3, { 0xd3, 0x28, 0x13 } },
	{ 0x39, 7, { 0xd9, 0x00, 0x00, 0xff, 0x00, 0xf0, 0x00 } },
	{ 0x39, 2, { 0xde, 0x01 } },
	{ 0x39, 9, { 0xb7, 0x17, 0xa7, 0x64, 0x3b, 0x06, 0x36, 0x18, 0x18 } },
	{ 0x39, 2, { 0xbe, 0x00 } },
	{ 0x39, 5, { 0xc1, 0x04, 0x40, 0x90, 0x08 } },
	{ 0x39, 5, { 0xc2, 0x00, 0x16, 0xda, 0xe7 } },
	{ 0x39, 3, { 0xc4, 0x72, 0x12 } },
	{ 0x39, 7, { 0xc7, 0x00, 0x00, 0x0c, 0x39, 0x08, 0x39 } },
	{ 0x39, 7, { 0xc8, 0x00, 0x00, 0x0b, 0x39, 0x08, 0x39 } },
	{ 0x39, 6, { 0xc9, 0x00, 0x0a, 0x08, 0x06, 0x04 } },
	{ 0x39, 6, { 0xca, 0x1e, 0x1f, 0x10, 0x17, 0x18 } },
	{ 0x39, 6, { 0xcb, 0x01, 0x0b, 0x09, 0x07, 0x05 } },
	{ 0x39, 6, { 0xcc, 0x1e, 0x1f, 0x11, 0x17, 0x18 } },
	{ 0x39, 6, { 0xcd, 0x31, 0x2b, 0x25, 0x27, 0x29 } },
	{ 0x39, 6, { 0xce, 0x3f, 0x3e, 0x21, 0x37, 0x38 } },
	{ 0x39, 6, { 0xcf, 0x30, 0x2a, 0x24, 0x26, 0x28 } },
	{ 0x39, 6, { 0xd0, 0x3f, 0x3e, 0x20, 0x37, 0x38 } },
	{ 0x39, 6, { 0xd1, 0x0c, 0x30, 0xa5, 0xff, 0x30 } },
	{ 0x39, 7, { 0xd3, 0x3b, 0x0e, 0x00, 0x00, 0x00, 0x00 } },
	{ 0x39, 7, { 0xd4, 0x67, 0x00, 0x00, 0x01, 0x00, 0x01 } },
	{ 0x39, 8, { 0xd5, 0x10, 0x10, 0x07, 0x07, 0x0f, 0x94, 0x26 } },
	{ 0x39, 4, { 0xd6, 0x00, 0x00, 0x40 } },
	{ 0x39, 4, { 0xd7, 0x01, 0x84, 0x20 } },
	{ 0x39, 2, { 0xde, 0x02 } },
	{ 0x39, 2, { 0xb6, 0x1c } },
	{ 0x39, 2, { 0xde, 0x00 } },
	{ 0x39, 2, { 0x4d, 0x00 } },
	{ 0x39, 2, { 0x4e, 0x00 } },
	{ 0x39, 2, { 0x4f, 0x00 } },
	{ 0x39, 2, { 0x4c, 0x01 } },
	{ 0x39, 2, { 0x4c, 0x00 } },
	{ 0x39, 5, { 0x2a, 0x00, 0x00, 0x01, 0x3f } },
	{ 0x39, 5, { 0x2b, 0x00, 0x00, 0x01, 0x76 } },
	{ 0x05, 1, { 0x35 } },
	{ 0x39, 2, { 0x36, 0x00 } },
	{ 0x39, 2, { 0x3a, 0x55 } },
};

static inline struct q2_panel *to_q2(struct drm_panel *panel)
{
	return container_of(panel, struct q2_panel, panel);
}

static int q2_send(struct q2_panel *p, const struct q2_command *cmd)
{
	struct mipi_dsi_msg msg = {
		.channel = p->dsi->channel,
		.type = cmd->type,
		.flags = MIPI_DSI_MSG_USE_LPM,
		.tx_len = cmd->len,
		.tx_buf = cmd->data,
	};
	ssize_t ret;

	/* Preserve stock long packets even for two-byte vendor commands. */
	ret = p->dsi->host->ops->transfer(p->dsi->host, &msg);
	return ret < 0 ? ret : 0;
}

static int q2_prepare(struct drm_panel *panel)
{
	struct q2_panel *p = to_q2(panel);
	unsigned int i;
	int ret;

	ret = regulator_enable(p->vdd28);
	if (ret)
		return ret;
	usleep_range(15000, 16000);
	ret = regulator_enable(p->vdd33);
	if (ret)
		goto disable28;
	gpiod_set_value_cansleep(p->reset, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(p->reset, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(p->reset, 0);
	usleep_range(10000, 11000);
	ret = mipi_dsi_dcs_soft_reset(p->dsi);
	if (ret)
		goto disable;
	usleep_range(10000, 11000);
	for (i = 0; i < ARRAY_SIZE(q2_init_commands); i++) {
		if (i == 41)
			usleep_range(10000, 11000);
		ret = q2_send(p, &q2_init_commands[i]);
		if (ret)
			goto disable;
	}
	ret = mipi_dsi_dcs_exit_sleep_mode(p->dsi);
	if (ret)
		goto disable;
	/* Keep the stock 120 ms sleep-out minimum without jiffy rounding. */
	usleep_range(120000, 121000);
	return 0;
disable:
	gpiod_set_value_cansleep(p->reset, 1);
	regulator_disable(p->vdd33);
disable28:
	regulator_disable(p->vdd28);
	return ret;
}

static int q2_enable(struct drm_panel *panel)
{
	return mipi_dsi_dcs_set_display_on(to_q2(panel)->dsi);
}

static int q2_disable(struct drm_panel *panel)
{
	int ret = mipi_dsi_dcs_set_display_off(to_q2(panel)->dsi);

	usleep_range(10000, 11000);
	return ret;
}

static int q2_unprepare(struct drm_panel *panel)
{
	struct q2_panel *p = to_q2(panel);
	int ret = mipi_dsi_dcs_enter_sleep_mode(p->dsi);

	msleep(20);
	gpiod_set_value_cansleep(p->reset, 1);
	usleep_range(10000, 11000);
	regulator_disable(p->vdd33);
	regulator_disable(p->vdd28);
	return ret;
}

static const struct drm_display_mode q2_mode = {
	.clock = 9331,
	.hdisplay = 320, .hsync_start = 340, .hsync_end = 360, .htotal = 384,
	.vdisplay = 375, .vsync_start = 391, .vsync_end = 393, .vtotal = 405,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int q2_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode = drm_mode_duplicate(connector->dev, &q2_mode);

	if (!mode)
		return -ENOMEM;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	return 1;
}

static const struct drm_panel_funcs q2_panel_funcs = {
	.prepare = q2_prepare,
	.enable = q2_enable,
	.disable = q2_disable,
	.unprepare = q2_unprepare,
	.get_modes = q2_get_modes,
};

static int q2_panel_probe(struct mipi_dsi_device *dsi)
{
	struct q2_panel *p;
	int ret;

	p = devm_drm_panel_alloc(&dsi->dev, struct q2_panel, panel,
				 &q2_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(p))
		return PTR_ERR(p);
	p->dsi = dsi;
	p->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(p->reset))
		return dev_err_probe(&dsi->dev, PTR_ERR(p->reset), "reset GPIO\n");
	p->vdd28 = devm_regulator_get(&dsi->dev, "vdd28");
	if (IS_ERR(p->vdd28))
		return dev_err_probe(&dsi->dev, PTR_ERR(p->vdd28), "2.8 V supply\n");
	p->vdd33 = devm_regulator_get(&dsi->dev, "vdd33");
	if (IS_ERR(p->vdd33))
		return dev_err_probe(&dsi->dev, PTR_ERR(p->vdd33), "3.3 V supply\n");
	dsi->lanes = 1;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;
	mipi_dsi_set_drvdata(dsi, p);
	ret = drm_panel_of_backlight(&p->panel);
	if (ret)
		return ret;
	drm_panel_add(&p->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&p->panel);
	return ret;
}

static void q2_panel_remove(struct mipi_dsi_device *dsi)
{
	struct q2_panel *p = mipi_dsi_get_drvdata(dsi);

	drm_panel_disable(&p->panel);
	drm_panel_unprepare(&p->panel);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&p->panel);
}

static const struct of_device_id q2_panel_match[] = {
	{ .compatible = "shanling,q2-h0209s" },
	{ }
};
MODULE_DEVICE_TABLE(of, q2_panel_match);

static struct mipi_dsi_driver q2_panel_driver = {
	.probe = q2_panel_probe,
	.remove = q2_panel_remove,
	.driver = {
		.name = "panel-shanling-q2",
		.of_match_table = q2_panel_match,
	},
};
module_mipi_dsi_driver(q2_panel_driver);
MODULE_DESCRIPTION("Shanling Q2 H0209S DSI panel");
MODULE_LICENSE("GPL");
