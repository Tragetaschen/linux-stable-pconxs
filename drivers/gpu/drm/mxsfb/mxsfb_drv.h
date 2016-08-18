/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * i.MX23/i.MX28/i.MX6SX MXSFB LCD controller driver.
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

#ifndef __MXSFB_DRV_H__
#define __MXSFB_DRV_H__

struct mxsfb_devdata {
	unsigned int	 transfer_count;
	unsigned int	 cur_buf;
	unsigned int	 next_buf;
	unsigned int	 debug0;
	unsigned int	 hs_wdth_mask;
	unsigned int	 hs_wdth_shift;
	unsigned int	 ipversion;
};

struct mxsfb_rgb_output {
	struct drm_connector	connector;
	struct drm_encoder	encoder;
	struct drm_panel	*panel;
};

struct mxsfb_drm_private {
	struct clk		*clk;
	struct clk		*clk_axi;
	struct clk		*clk_disp_axi;
	void __iomem		*base;	/* registers */

	struct drm_fbdev_cma	*fbdev;
	struct drm_crtc		crtc;
	struct drm_plane	*plane;
	struct drm_atomic_state	*state;

	struct mxsfb_rgb_output	output;

	const struct mxsfb_devdata *devdata;
	bool enabled;
};

#define crtc_to_mxsfb_priv(x)	container_of(x, struct mxsfb_drm_private, crtc)

int mxsfb_setup_crtc(struct drm_device *dev);
int mxsfb_create_outputs(struct drm_device *dev);
void mxsfb_set_scanout(struct mxsfb_drm_private *mxsfb);

void mxsfb_enable_axi_clk(struct mxsfb_drm_private *mxsfb);
void mxsfb_disable_axi_clk(struct mxsfb_drm_private *mxsfb);

#endif /* __MXSFB_DRV_H__ */
