// SPDX-License-Identifier: GPL-2.0-only
/*
 * OrtusTech COM32H3N89ULC LCD drm_panel driver.
 *
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 * Copyright (C) 2019 Kai Ruhnau <kai.ruhnau@target-sg.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

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
#define CMD_DISPOFF		0x28
#define CMD_SLPIN		0x10

/* Ortustech COM32H3N89ULC power-up sequence commands. */
static const u16 ortus_enable_extended_commands[] = {
	CMD_SETRGBIF2,
	DATA(0xff), DATA(0x83), DATA(0x63)
};

static const u16 ortus_set_power1[] = {
	CMD_SETPOWER,
	DATA(0x81), DATA(0x24), DATA(0x04), DATA(0x02),
	DATA(0x02), DATA(0x03), DATA(0x10), DATA(0x10),
	DATA(0x34), DATA(0x3c), DATA(0x3f), DATA(0x3f)
};

static const u16 ortus_sleep_out[] = {
	CMD_SLPOUT
};

static const u16 ortus_inversion_off[] = {
	CMD_INVOFF
};

static const u16 ortus_memory_access_control[] = {
	CMD_MADCTL, DATA(0x00)
};

static const u16 ortus_interface_pixel_format[] = {
	CMD_COLMOD, DATA(0x70)
};

static const u16 ortus_set_power2[] = {
	CMD_SETPOWER,
	DATA(0x78), DATA(0x24), DATA(0x04), DATA(0x02),
	DATA(0x02), DATA(0x03), DATA(0x10), DATA(0x10),
	DATA(0x34), DATA(0x3c), DATA(0x3f), DATA(0x3f)
};

static const u16 ortus_set_rgb_interface_related_reg[] = {
	CMD_SETRGBIF0, DATA(0x01)
};

static const u16 ortus_set_ortus_waveform_cycle[] = {
	CMD_SETRGBIF1,
	DATA(0x00), DATA(0x08), DATA(0x56), DATA(0x07),
	DATA(0x01), DATA(0x01), DATA(0x4d), DATA(0x01),
	DATA(0x42)
};

static const u16 ortus_set_panel[] = {
	CMD_SETDGCLUT, DATA(0x0b)
};

static const u16 ortus_set_gamma_curve_setting[] = {
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

static const u16 ortus_panel_on[] = {
	CMD_DISPON
};

static const u16 ortus_panel_off[] = {
	CMD_DISPOFF
};

static const u16 ortus_sleep_in[] = {
	CMD_SLPIN
};

struct ortus {
	struct drm_panel	panel;
	struct spi_device	*spi;
	struct videomode	vm;
	struct gpio_desc	*reset_gpio;
	struct regulator	*supply;
	struct backlight_device *backlight;
};

static inline struct ortus *panel_to_ortus(struct drm_panel *panel)
{
	return container_of(panel, struct ortus, panel);
}

static int ortus_prepare(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);
	int err;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 5000);

	err = regulator_enable(ctx->supply);

	if (err < 0) {
		DRM_DEV_ERROR(ctx->panel.drm->dev,
			      "Failed to enable supply %d\n", err);
		return err;
	}

	usleep_range(10000, 20000);
	gpiod_set_value(ctx->reset_gpio, 0);

	usleep_range(10000, 20000);

	return 0;
}

static int ortus_enable(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);
	int ret;

	msleep(50);

	ret = spi_write(ctx->spi, ortus_enable_extended_commands,
			sizeof(ortus_enable_extended_commands));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_set_power1, sizeof(ortus_set_power1));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_sleep_out, sizeof(ortus_sleep_out));
	if (ret)
		return ret;

	usleep_range(5000, 10000);

	ret = spi_write(ctx->spi, ortus_inversion_off,
			sizeof(ortus_inversion_off));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_memory_access_control,
			sizeof(ortus_memory_access_control));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_interface_pixel_format,
			sizeof(ortus_interface_pixel_format));
	if (ret)
		return ret;

	msleep(120);

	ret = spi_write(ctx->spi, ortus_set_power2, sizeof(ortus_set_power2));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_set_rgb_interface_related_reg,
			sizeof(ortus_set_rgb_interface_related_reg));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_set_ortus_waveform_cycle,
			sizeof(ortus_set_ortus_waveform_cycle));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_set_panel, sizeof(ortus_set_panel));
	if (ret)
		return ret;

	ret = spi_write(ctx->spi, ortus_set_gamma_curve_setting,
			sizeof(ortus_set_gamma_curve_setting));
	if (ret)
		return ret;

	usleep_range(5000, 10000);

	ret = spi_write(ctx->spi, ortus_panel_on, sizeof(ortus_panel_on));
	if (ret)
		return ret;

	ret = backlight_enable(ctx->backlight);
	if (ret) {
		DRM_DEV_ERROR(ctx->panel.drm->dev,
			      "Failed to enable backlight %d\n", ret);
		return ret;
	}

	return 0;
}

static int ortus_disable(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);
	int ret;

	backlight_disable(ctx->backlight);

	ret = spi_write(ctx->spi, ortus_panel_off, sizeof(ortus_panel_off));
	if (ret)
		return ret;
	usleep_range(5000, 10000);
	ret = spi_write(ctx->spi, ortus_sleep_in, sizeof(ortus_sleep_in));
	if (ret)
		return ret;
	msleep(50);

	return 0;
}

static int ortus_unprepare(struct drm_panel *panel)
{
	struct ortus *ctx = panel_to_ortus(panel);

	return regulator_disable(ctx->supply);
}

static const struct drm_display_mode default_mode = {
	.clock		= 30720,
	.hdisplay	= 480,
	.hsync_start	= 480 + 8,
	.hsync_end	= 480 + 8 + 10,
	.htotal		= 480 + 8 + 10 + 10,
	.vdisplay	= 800,
	.vsync_start	= 800 + 2,
	.vsync_end	= 800 + 2 + 2,
	.vtotal		= 800 + 2 + 2 + 2,
	.vrefresh	= 75,
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
	info->width_mm = 41;
	info->height_mm = 69;
	drm_display_info_set_bus_formats(info, &format, 1);

	return 1;
}

static const struct drm_panel_funcs ortus_drm_funcs = {
	.prepare	= ortus_prepare,
	.enable		= ortus_enable,
	.disable	= ortus_disable,
	.unprepare	= ortus_unprepare,
	.get_modes	= ortus_get_modes,
};

static int ortus_probe(struct spi_device *spi)
{
	struct ortus *ctx;
	int ret;

	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

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

	ctx->supply = devm_regulator_get_optional(&spi->dev, "power");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		dev_err(&spi->dev, "failed to request regulator: %d\n", ret);
		return ret;
	}

	ctx->backlight = devm_of_find_backlight(&spi->dev);
	if (IS_ERR(ctx->backlight))
		return PTR_ERR(ctx->backlight);

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

static const struct spi_device_id com32_ids[] = {
	{ "com32h3n89ulc", 0 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(spi, com32_ids);

static struct spi_driver ortus_driver = {
	.probe = ortus_probe,
	.remove = ortus_remove,
	.id_table = com32_ids,
	.driver = {
		.name = "com32h3n89ulc",
		.of_match_table = ortus_of_match,
	},
};
module_spi_driver(ortus_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_AUTHOR("Kai Ruhnau <kai.ruhnau@target-sg.com>");
MODULE_DESCRIPTION("OrtusTech COM32H3N89ULC LCD Driver");
MODULE_LICENSE("GPL v2");
