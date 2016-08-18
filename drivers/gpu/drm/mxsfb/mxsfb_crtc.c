/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * This code is based on drivers/video/fbdev/mxsfb.c :
 * Copyright (C) 2010 Juergen Beisert, Pengutronix
 * Copyright (C) 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/of_graph.h>
#include <linux/platform_data/simplefb.h>
#include <video/videomode.h>

#include "mxsfb_drv.h"
#include "mxsfb_regs.h"

static u32 set_hsync_pulse_width(struct mxsfb_drm_private *mxsfb, u32 val)
{
	return (val & mxsfb->devdata->hs_wdth_mask) <<
		mxsfb->devdata->hs_wdth_shift;
}

static struct simplefb_format supported_formats[] = SIMPLEFB_FORMATS;

/*
 * Setup the MXSFB registers for decoding the pixels out of the framebuffer
 */
static int mxsfb_set_pixel_fmt(struct drm_crtc *crtc)
{
	struct drm_device *drm = crtc->dev;
	struct mxsfb_drm_private *mxsfb = crtc_to_mxsfb_priv(crtc);
	struct simplefb_format *format = NULL;
	u32 pixel_format, ctrl, ctrl1;
	int i;

	pixel_format = crtc->primary->state->fb->pixel_format;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (supported_formats[i].fourcc == pixel_format)
			format = &supported_formats[i];
	}

	if (WARN_ON(!format))
		return 0;

	ctrl = CTRL_BYPASS_COUNT | CTRL_MASTER;

	/*
	 * WARNING: The bus width, CTRL_SET_BUS_WIDTH(), is configured to
	 * match the selected mode here. This differs from the original
	 * MXSFB driver, which had the option to configure the bus width
	 * to arbitrary value. This limitation should not pose an issue.
	 */

	/* CTRL1 contains IRQ config and status bits, preserve those. */
	ctrl1 = readl(mxsfb->base + LCDC_CTRL1);
	ctrl1 &= CTRL1_CUR_FRAME_DONE_IRQ_EN | CTRL1_CUR_FRAME_DONE_IRQ;

	switch (format->fourcc) {
	case DRM_FORMAT_RGB565:
		dev_dbg(drm->dev, "Setting up RGB565 mode\n");
		ctrl |= CTRL_SET_BUS_WIDTH(STMLCDIF_16BIT);
		ctrl |= CTRL_SET_WORD_LENGTH(0);
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0xf);
		break;
	case DRM_FORMAT_XRGB8888:
		dev_dbg(drm->dev, "Setting up XRGB8888 mode\n");
		ctrl |= CTRL_SET_BUS_WIDTH(STMLCDIF_24BIT);
		ctrl |= CTRL_SET_WORD_LENGTH(3);
		/* Do not use packed pixels = one pixel per word instead. */
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0x7);
		break;
	default:
		dev_err(drm->dev, "Unhandled color format %s\n",
			format->name);
		return -EINVAL;
	}

	writel(ctrl1, mxsfb->base + LCDC_CTRL1);
	writel(ctrl, mxsfb->base + LCDC_CTRL);
	return 0;
}

static void mxsfb_enable_controller(struct mxsfb_drm_private *mxsfb)
{
	u32 reg;

	if (mxsfb->clk_disp_axi)
		clk_prepare_enable(mxsfb->clk_disp_axi);
	clk_prepare_enable(mxsfb->clk);
	mxsfb_enable_axi_clk(mxsfb);

	/* If it was disabled, re-enable the mode again */
	writel(CTRL_DOTCLK_MODE, mxsfb->base + LCDC_CTRL + REG_SET);

	/* Enable the SYNC signals first, then the DMA engine */
	reg = readl(mxsfb->base + LCDC_VDCTRL4);
	reg |= VDCTRL4_SYNC_SIGNALS_ON;
	writel(reg, mxsfb->base + LCDC_VDCTRL4);

	writel(CTRL_RUN, mxsfb->base + LCDC_CTRL + REG_SET);

	mxsfb->enabled = 1;
}

static void mxsfb_disable_controller(struct mxsfb_drm_private *mxsfb)
{
	u32 reg;

	/*
	 * Even if we disable the controller here, it will still continue
	 * until its FIFOs are running out of data
	 */
	writel(CTRL_DOTCLK_MODE, mxsfb->base + LCDC_CTRL + REG_CLR);

	readl_poll_timeout(mxsfb->base + LCDC_CTRL, reg, !(reg & CTRL_RUN),
			   0, 1000);

	reg = readl(mxsfb->base + LCDC_VDCTRL4);
	reg &= ~VDCTRL4_SYNC_SIGNALS_ON;
	writel(reg, mxsfb->base + LCDC_VDCTRL4);

	mxsfb_disable_axi_clk(mxsfb);

	clk_disable_unprepare(mxsfb->clk);
	if (mxsfb->clk_disp_axi)
		clk_disable_unprepare(mxsfb->clk_disp_axi);

	mxsfb->enabled = 0;
}

static void mxsfb_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mxsfb_drm_private *mxsfb = crtc_to_mxsfb_priv(crtc);
	struct drm_display_mode *m = &crtc->state->adjusted_mode;
	const u32 bus_flags = mxsfb->output.connector.display_info.bus_flags;
	u32 vdctrl0, vsync_pulse_len, hsync_pulse_len;
	bool reenable = false;
	int err;

	/*
	 * It seems, you can't re-program the controller if it is still
	 * running. This may lead to shifted pictures (FIFO issue?), so
	 * first stop the controller and drain its FIFOs.
	 */
	if (mxsfb->enabled) {
		reenable = true;
		mxsfb_disable_controller(mxsfb);
	}

	mxsfb_enable_axi_clk(mxsfb);

	/* Clear the FIFOs */
	writel(CTRL1_FIFO_CLEAR, mxsfb->base + LCDC_CTRL1 + REG_SET);

	err = mxsfb_set_pixel_fmt(crtc);
	if (err)
		return;

	clk_set_rate(mxsfb->clk, m->crtc_clock * 1000);

	writel(TRANSFER_COUNT_SET_VCOUNT(m->crtc_vdisplay) |
	       TRANSFER_COUNT_SET_HCOUNT(m->crtc_hdisplay),
	       mxsfb->base + mxsfb->devdata->transfer_count);

	vsync_pulse_len = m->crtc_vsync_end - m->crtc_vsync_start;

	vdctrl0 = VDCTRL0_ENABLE_PRESENT |	/* Always in DOTCLOCK mode */
		  VDCTRL0_VSYNC_PERIOD_UNIT |
		  VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
		  VDCTRL0_SET_VSYNC_PULSE_WIDTH(vsync_pulse_len);
	if (m->flags & DRM_MODE_FLAG_PHSYNC)
		vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
	if (m->flags & DRM_MODE_FLAG_PVSYNC)
		vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
	if (bus_flags & DRM_BUS_FLAG_DE_HIGH)
		vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
	if (bus_flags & DRM_BUS_FLAG_PIXDATA_NEGEDGE)
		vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

	writel(vdctrl0, mxsfb->base + LCDC_VDCTRL0);

	/* Frame length in lines. */
	writel(m->crtc_vtotal, mxsfb->base + LCDC_VDCTRL1);

	/* Line length in units of clocks or pixels. */
	hsync_pulse_len = m->crtc_hsync_end - m->crtc_hsync_start;
	writel(set_hsync_pulse_width(mxsfb, hsync_pulse_len) |
	       VDCTRL2_SET_HSYNC_PERIOD(m->crtc_htotal),
	       mxsfb->base + LCDC_VDCTRL2);

	writel(SET_HOR_WAIT_CNT(m->crtc_hblank_end - m->crtc_hsync_end) |
	       SET_VERT_WAIT_CNT(m->crtc_vblank_end - m->crtc_vsync_end),
	       mxsfb->base + LCDC_VDCTRL3);

	writel(SET_DOTCLK_H_VALID_DATA_CNT(m->hdisplay),
	       mxsfb->base + LCDC_VDCTRL4);

	mxsfb_disable_axi_clk(mxsfb);

	if (reenable)
		mxsfb_enable_controller(mxsfb);
}

static void mxsfb_crtc_enable(struct drm_crtc *crtc)
{
	struct mxsfb_drm_private *mxsfb = crtc_to_mxsfb_priv(crtc);

	mxsfb_crtc_mode_set_nofb(crtc);
	mxsfb_enable_controller(mxsfb);
}

static void mxsfb_crtc_disable(struct drm_crtc *crtc)
{
	struct mxsfb_drm_private *mxsfb = crtc_to_mxsfb_priv(crtc);

	if (!crtc->state->active)
		return;

	mxsfb_disable_controller(mxsfb);
}

static int mxsfb_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	struct mxsfb_drm_private *mxsfb = crtc_to_mxsfb_priv(crtc);
	struct drm_display_mode *mode = &state->adjusted_mode;
	long rate, clk_rate = mode->clock * 1000;

	if (!mode->clock)
		return 0;

	rate = clk_round_rate(mxsfb->clk, clk_rate);
	/* clock required by mode not supported by hardware */
	if (rate != clk_rate)
		return -EINVAL;

	return 0;
}

static void mxsfb_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_crtc_state *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs mxsfb_crtc_helper_funcs = {
	.mode_set	= drm_helper_crtc_mode_set,
	.mode_set_base	= drm_helper_crtc_mode_set_base,
	.mode_set_nofb	= mxsfb_crtc_mode_set_nofb,
	.enable		= mxsfb_crtc_enable,
	.disable	= mxsfb_crtc_disable,
	.prepare	= mxsfb_crtc_disable,
	.commit		= mxsfb_crtc_enable,
	.atomic_check	= mxsfb_crtc_atomic_check,
	.atomic_begin	= mxsfb_crtc_atomic_begin,
};

static void mxsfb_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *state)
{
	struct mxsfb_drm_private *mxsfb = plane->dev->dev_private;
	struct drm_gem_cma_object *gem;

	if (!plane->state->crtc || !plane->state->fb)
		return;

	gem = drm_fb_cma_get_gem_obj(plane->state->fb, 0);

	mxsfb_enable_axi_clk(mxsfb);
	writel(gem->paddr, mxsfb->base + mxsfb->devdata->next_buf);
	mxsfb_disable_axi_clk(mxsfb);
}

static const struct drm_plane_helper_funcs mxsfb_plane_helper_funcs = {
	.atomic_update	= mxsfb_plane_atomic_update,
};

static void mxsfb_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs mxsfb_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= mxsfb_plane_destroy,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static struct drm_plane *mxsfb_plane_init(struct drm_device *drm)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;
	struct drm_plane *plane = NULL;
	u32 formats[ARRAY_SIZE(supported_formats)], i;
	int ret;

	plane = devm_kzalloc(drm->dev, sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++)
		formats[i] = supported_formats[i].fourcc;

	ret = drm_universal_plane_init(drm, plane, 0xff, &mxsfb_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) {
		devm_kfree(drm->dev, plane);
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(plane, &mxsfb_plane_helper_funcs);
	mxsfb->plane = plane;

	return plane;
}

static void mxsfb_crtc_cleanup(struct drm_crtc *crtc)
{
	mxsfb_crtc_disable(crtc);
	drm_crtc_cleanup(crtc);
}

static const struct drm_crtc_funcs mxsfb_crtc_funcs = {
	.destroy		= mxsfb_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

int mxsfb_setup_crtc(struct drm_device *drm)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;
	struct drm_plane *primary;
	int ret;

	primary = mxsfb_plane_init(drm);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	ret = drm_crtc_init_with_planes(drm, &mxsfb->crtc, primary, NULL,
					&mxsfb_crtc_funcs, NULL);
	if (ret) {
		mxsfb_plane_destroy(primary);
		devm_kfree(drm->dev, primary);
		return ret;
	}

	drm_crtc_helper_add(&mxsfb->crtc, &mxsfb_crtc_helper_funcs);
	return 0;
}
