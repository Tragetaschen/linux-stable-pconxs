/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * This code is based on drivers/gpu/drm/atmel-hlcdc/atmel_hlcdc_output.c :
 * Copyright (C) 2014 Traphandler
 * Copyright (C) 2014 Free Electrons
 * Copyright (C) 2014 Atmel
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

#include <linux/of_graph.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_plane_helper.h>
#include <drm/drmP.h>

#include "mxsfb_drv.h"

static inline struct mxsfb_rgb_output *
drm_connector_to_mxsfb_rgb_output(struct drm_connector *connector)
{
	return container_of(connector, struct mxsfb_rgb_output,
			    connector);
}

static inline struct mxsfb_rgb_output *
drm_encoder_to_mxsfb_rgb_output(struct drm_encoder *encoder)
{
	return container_of(encoder, struct mxsfb_rgb_output, encoder);
}

static void mxsfb_rgb_encoder_enable(struct drm_encoder *encoder)
{
	struct mxsfb_rgb_output *rgb =
			drm_encoder_to_mxsfb_rgb_output(encoder);

	if (rgb->panel) {
		drm_panel_prepare(rgb->panel);
		drm_panel_enable(rgb->panel);
	}
}

static void mxsfb_rgb_encoder_disable(struct drm_encoder *encoder)
{
	struct mxsfb_rgb_output *rgb =
			drm_encoder_to_mxsfb_rgb_output(encoder);

	if (rgb->panel) {
		drm_panel_disable(rgb->panel);
		drm_panel_unprepare(rgb->panel);
	}
}

static const struct
drm_encoder_helper_funcs mxsfb_panel_encoder_helper_funcs = {
	.disable	= mxsfb_rgb_encoder_disable,
	.enable		= mxsfb_rgb_encoder_enable,
};

static void mxsfb_rgb_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
	memset(encoder, 0, sizeof(*encoder));
}

static const struct drm_encoder_funcs mxsfb_panel_encoder_funcs = {
	.destroy = mxsfb_rgb_encoder_destroy,
};

static int mxsfb_panel_get_modes(struct drm_connector *connector)
{
	struct mxsfb_rgb_output *rgb =
			drm_connector_to_mxsfb_rgb_output(connector);

	if (rgb->panel)
		return rgb->panel->funcs->get_modes(rgb->panel);

	return 0;
}

static int mxsfb_rgb_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	return 0;
}

static struct drm_encoder *
mxsfb_rgb_best_encoder(struct drm_connector *connector)
{
	struct mxsfb_rgb_output *rgb =
			drm_connector_to_mxsfb_rgb_output(connector);

	return &rgb->encoder;
}

static const struct
drm_connector_helper_funcs mxsfb_panel_connector_helper_funcs = {
	.get_modes = mxsfb_panel_get_modes,
	.mode_valid = mxsfb_rgb_mode_valid,
	.best_encoder = mxsfb_rgb_best_encoder,
};

static enum drm_connector_status
mxsfb_panel_connector_detect(struct drm_connector *connector, bool force)
{
	struct mxsfb_rgb_output *rgb =
			drm_connector_to_mxsfb_rgb_output(connector);

	if (rgb->panel)
		return connector_status_connected;

	return connector_status_disconnected;
}

static void mxsfb_panel_connector_destroy(struct drm_connector *connector)
{
	struct mxsfb_rgb_output *rgb =
			drm_connector_to_mxsfb_rgb_output(connector);

	if (rgb->panel)
		drm_panel_detach(rgb->panel);

	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs mxsfb_panel_connector_funcs = {
	.dpms			= drm_atomic_helper_connector_dpms,
	.detect			= mxsfb_panel_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= mxsfb_panel_connector_destroy,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static int mxsfb_check_endpoint(struct drm_device *drm,
				const struct of_endpoint *ep)
{
	struct device_node *np;
	void *obj;

	np = of_graph_get_remote_port_parent(ep->local_node);

	obj = of_drm_find_panel(np);
	if (!obj)
		obj = of_drm_find_bridge(np);

	of_node_put(np);

	return obj ? 0 : -EPROBE_DEFER;
}

static int mxsfb_attach_endpoint(struct drm_device *drm,
				 const struct of_endpoint *ep)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;
	struct mxsfb_rgb_output *output = &mxsfb->output;
	struct device_node *np;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	int ret;

	drm_encoder_helper_add(&output->encoder,
			       &mxsfb_panel_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &output->encoder,
			       &mxsfb_panel_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;

	output->encoder.possible_crtcs = 0x1;

	np = of_graph_get_remote_port_parent(ep->local_node);

	ret = -EPROBE_DEFER;

	panel = of_drm_find_panel(np);
	if (panel) {
		of_node_put(np);
		output->connector.dpms = DRM_MODE_DPMS_OFF;
		output->connector.polled = DRM_CONNECTOR_POLL_CONNECT;
		drm_connector_helper_add(&output->connector,
				&mxsfb_panel_connector_helper_funcs);
		ret = drm_connector_init(drm, &output->connector,
					 &mxsfb_panel_connector_funcs,
					 DRM_MODE_CONNECTOR_Unknown);
		if (ret)
			goto err_encoder_cleanup;

		drm_mode_connector_attach_encoder(&output->connector,
						  &output->encoder);

		ret = drm_panel_attach(panel, &output->connector);
		if (ret) {
			drm_connector_cleanup(&output->connector);
			goto err_encoder_cleanup;
		}

		output->panel = panel;

		return 0;
	}

	bridge = of_drm_find_bridge(np);
	of_node_put(np);

	if (bridge) {
		output->encoder.bridge = bridge;
		bridge->encoder = &output->encoder;
		ret = drm_bridge_attach(drm, bridge);
		if (!ret)
			return 0;
	}

err_encoder_cleanup:
	drm_encoder_cleanup(&output->encoder);

	return ret;
}

int mxsfb_create_outputs(struct drm_device *drm)
{
	struct device_node *ep_np = NULL;
	struct of_endpoint ep;
	int ret;

	for_each_endpoint_of_node(drm->dev->of_node, ep_np) {
		ret = of_graph_parse_endpoint(ep_np, &ep);
		if (!ret)
			ret = mxsfb_check_endpoint(drm, &ep);

		if (ret) {
			of_node_put(ep_np);
			return ret;
		}
	}

	for_each_endpoint_of_node(drm->dev->of_node, ep_np) {
		ret = of_graph_parse_endpoint(ep_np, &ep);
		if (!ret)
			ret = mxsfb_attach_endpoint(drm, &ep);

		if (ret) {
			of_node_put(ep_np);
			return ret;
		}
	}

	return 0;
}
