/*
 * OrtusTech COM32H3N89ULC LCD drm_panel driver.
 *
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#define DATA(x)			((x) | BIT(8))
#define CMD_SLPOUT		0x11
#define CMD_INVOFF		0x20
#define CMD_DISPON		0x29
#define CMD_MADCTL		0x36
#define CMD_COLMOD		0x3a
#define CMD_SETPOWER		0xb1
#define CMD_SETRGBIF0		0xb3
#define CMD_SETRGBIF1		0xb4
#define CMD_SETRGBIF2		0xb9
#define CMD_SETDGCLUT		0xcc
#define CMD_SET_SIP_READ_INDEX	0xe0

/* Ortustech COM32H3N89ULC power-up sequence commands. */
static const u16 const ortus_enable_extended_commands[] = {
	CMD_SETRGBIF2,
	DATA(0xff), DATA(0x83), DATA(0x63)
};

static const u16 const ortus_set_power1[] = {
	CMD_SETPOWER,
	DATA(0x81), DATA(0x24), DATA(0x04), DATA(0x02),
	DATA(0x02), DATA(0x03), DATA(0x10), DATA(0x10),
	DATA(0x34), DATA(0x3c), DATA(0x3f), DATA(0x3f)
};

static const u16 const ortus_sleep_out[] = {
	CMD_SLPOUT
};

static const u16 const ortus_inversion_off[] = {
	CMD_INVOFF
};

static const u16 const ortus_memory_access_control[] = {
	CMD_MADCTL, DATA(0x00)
};

static const u16 const ortus_interface_pixel_format[] = {
	CMD_COLMOD, DATA(0x70)
};

static const u16 const ortus_set_power2[] = {
	CMD_SETPOWER,
	DATA(0x78), DATA(0x24), DATA(0x04), DATA(0x02),
	DATA(0x02), DATA(0x03), DATA(0x10), DATA(0x10),
	DATA(0x34), DATA(0x3c), DATA(0x3f), DATA(0x3f)
};

static const u16 const ortus_set_rgb_interface_related_reg[] = {
	CMD_SETRGBIF0, DATA(0x01)
};

static const u16 const ortus_set_ortus_waveform_cycle[] = {
	CMD_SETRGBIF1,
	DATA(0x00), DATA(0x08), DATA(0x56), DATA(0x07),
	DATA(0x01), DATA(0x01), DATA(0x4d), DATA(0x01),
	DATA(0x42)
};

static const u16 const ortus_set_panel[] = {
	CMD_SETDGCLUT, DATA(0x0b)
};

static const u16 const ortus_set_gamma_curve_setting[] = {
	CMD_SET_SIP_READ_INDEX,
	DATA(0x01), DATA(0x48), DATA(0x4d), DATA(0x4e),
	DATA(0x58), DATA(0xf6), DATA(0x0b), DATA(0x4e),
	DATA(0x12), DATA(0xd5), DATA(0x15), DATA(0x95),
	DATA(0x55), DATA(0x8e), DATA(0x11), DATA(0x01),
	DATA(0x48), DATA(0x4d), DATA(0x55), DATA(0x5f),
	DATA(0xfd), DATA(0x0a), DATA(0x4e), DATA(0x51),
	DATA(0xd3), DATA(0x17), DATA(0x95), DATA(0x96),
	DATA(0x4e), DATA(0x11),
};

static const u16 const ortus_panel_on[] = {
	CMD_DISPON
};

struct ortus {
	struct drm_panel	panel;
	struct work_struct	work;
	struct spi_device	*spi;
	struct videomode	vm;
	struct gpio_desc	*reset_gpio;
};

static inline struct ortus *panel_to_ortus(struct drm_panel *panel)
{
	return container_of(panel, struct ortus, panel);
}

static void ortus_spi_init(struct work_struct *work)
{
	struct ortus *ctx = container_of(work, struct ortus, work);
	int ret;

	/* Reset the display. */
	gpiod_set_value(ctx->reset_gpio, 1);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(50);

	ret = spi_write(ctx->spi, ortus_enable_extended_commands,
			sizeof(ortus_enable_extended_commands));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_set_power1, sizeof(ortus_set_power1));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_sleep_out, sizeof(ortus_sleep_out));
	if (ret)
		return;

	mdelay(5);

	ret = spi_write(ctx->spi, ortus_inversion_off,
			sizeof(ortus_inversion_off));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_memory_access_control,
			sizeof(ortus_memory_access_control));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_interface_pixel_format,
			sizeof(ortus_interface_pixel_format));
	if (ret)
		return;

	mdelay(120);

	ret = spi_write(ctx->spi, ortus_set_power2, sizeof(ortus_set_power2));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_set_rgb_interface_related_reg,
			sizeof(ortus_set_rgb_interface_related_reg));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_set_ortus_waveform_cycle,
			sizeof(ortus_set_ortus_waveform_cycle));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_set_panel, sizeof(ortus_set_panel));
	if (ret)
		return;

	ret = spi_write(ctx->spi, ortus_set_gamma_curve_setting,
			sizeof(ortus_set_gamma_curve_setting));
	if (ret)
		return;

	mdelay(5);

	spi_write(ctx->spi, ortus_panel_on, sizeof(ortus_panel_on));
}

static int ortus_disable(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);

	cancel_work_sync(&ctx->work);

	/* Put the display into reset state. */
	gpiod_set_value(ctx->reset_gpio, 1);

	return 0;
}

static int ortus_enable(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);

	cancel_work_sync(&ctx->work);

	return schedule_work(&ctx->work);
}

static const struct drm_display_mode default_mode = {
	.clock		= 25000,
	.hdisplay	= 480,
	.hsync_start	= 480 + 10,
	.hsync_end	= 480 + 10 + 10,
	.htotal		= 480 + 10 + 10 + 15,
	.vdisplay	= 800,
	.vsync_start	= 800 + 3,
	.vsync_end	= 800 + 3 + 3,
	.vtotal		= 800 + 3 + 3 + 3,
	.vrefresh	= 60,
};

static int ortus_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct drm_display_info *info = &connector->display_info;
	struct drm_display_mode *mode;
	u32 format = MEDIA_BUS_FMT_RGB888_1X24;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	info->bpc = 8;
	info->width_mm = 47;
	info->height_mm = 79;
	drm_display_info_set_bus_formats(info, &format, 1);
	info->bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE;

	return 1;
}

static const struct drm_panel_funcs ortus_drm_funcs = {
	.disable	= ortus_disable,
	.enable		= ortus_enable,
	.get_modes	= ortus_get_modes,
};

static int ortus_probe(struct spi_device *spi)
{
	struct ortus *ctx;
	int ret;

	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	INIT_WORK(&ctx->work, ortus_spi_init);

	ctx->spi = spi;

	spi_set_drvdata(spi, ctx);
	spi->bits_per_word = 9;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
		return ret;
	}

	ctx->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(&spi->dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = &spi->dev;
	ctx->panel.funcs = &ortus_drm_funcs;

	return drm_panel_add(&ctx->panel);
}

static int ortus_remove(struct spi_device *spi)
{
	struct ortus *ctx = spi_get_drvdata(spi);

	ortus_disable(&ctx->panel);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id ortus_of_match[] = {
	{ .compatible = "ortustech,com32h3n89ulc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ortus_of_match);

static struct spi_driver ortus_driver = {
	.probe = ortus_probe,
	.remove = ortus_remove,
	.driver = {
		.name = "com32h3n89ulc",
		.of_match_table = ortus_of_match,
	},
};
module_spi_driver(ortus_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("OrtusTech COM32H3N89ULC LCD Driver");
MODULE_LICENSE("GPL v2");
