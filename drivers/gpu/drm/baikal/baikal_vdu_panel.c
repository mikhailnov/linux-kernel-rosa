// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

static void baikal_lvds_connector_force(struct drm_connector *connector)
{
	struct baikal_lvds_bridge *bridge = connector_to_baikal_lvds_bridge(connector);
	struct baikal_vdu_private *priv = bridge->vdu;
	u32 cntl = readl(priv->regs + CR1);
	if (connector->force == DRM_FORCE_OFF)
		cntl &= ~CR1_LCE;
	else
		cntl |= CR1_LCE;
	writel(cntl, priv->regs + CR1);
}

static int baikal_lvds_connector_get_modes(struct drm_connector *connector)
{
	struct baikal_lvds_bridge *panel_bridge =
		connector_to_baikal_lvds_bridge(connector);

	return drm_panel_get_modes(panel_bridge->panel, connector);
}

static const struct drm_connector_helper_funcs
baikal_lvds_bridge_connector_helper_funcs = {
	.get_modes = baikal_lvds_connector_get_modes,
};

static const struct drm_connector_funcs baikal_lvds_bridge_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.force = baikal_lvds_connector_force,
};

static int baikal_lvds_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	struct baikal_lvds_bridge *panel_bridge = bridge_to_baikal_lvds_bridge(bridge);
	struct drm_connector *connector = &panel_bridge->connector;
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Missing encoder\n");
		return -ENODEV;
	}

	drm_connector_helper_add(connector,
				 &baikal_lvds_bridge_connector_helper_funcs);

	ret = drm_connector_init(bridge->dev, connector,
				 &baikal_lvds_bridge_connector_funcs,
				 panel_bridge->connector_type);
	if (ret) {
		DRM_ERROR("Failed to initialize connector\n");
		return ret;
	}

	ret = drm_connector_attach_encoder(&panel_bridge->connector,
					  bridge->encoder);
	if (ret < 0)
		return ret;

	return 0;
}

static void baikal_lvds_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct baikal_lvds_bridge *panel_bridge = bridge_to_baikal_lvds_bridge(bridge);

	baikal_vdu_switch_on(panel_bridge->vdu);
	drm_panel_prepare(panel_bridge->panel);
}

static void baikal_lvds_bridge_enable(struct drm_bridge *bridge)
{
	struct baikal_lvds_bridge *panel_bridge = bridge_to_baikal_lvds_bridge(bridge);

	drm_panel_enable(panel_bridge->panel);
}

static void baikal_lvds_bridge_disable(struct drm_bridge *bridge)
{
	struct baikal_lvds_bridge *panel_bridge = bridge_to_baikal_lvds_bridge(bridge);

	drm_panel_disable(panel_bridge->panel);
}

static void baikal_lvds_bridge_post_disable(struct drm_bridge *bridge)
{
	struct baikal_lvds_bridge *panel_bridge = bridge_to_baikal_lvds_bridge(bridge);

	drm_panel_unprepare(panel_bridge->panel);
	baikal_vdu_switch_off(panel_bridge->vdu);
}

static const struct drm_bridge_funcs baikal_lvds_bridge_funcs = {
	.attach = baikal_lvds_bridge_attach,
	.pre_enable = baikal_lvds_bridge_pre_enable,
	.enable = baikal_lvds_bridge_enable,
	.disable = baikal_lvds_bridge_disable,
	.post_disable = baikal_lvds_bridge_post_disable,
};

static struct drm_bridge *baikal_lvds_bridge_add(struct drm_panel *panel,
					u32 connector_type)
{
	struct baikal_lvds_bridge *panel_bridge;

	if (!panel)
		return ERR_PTR(-EINVAL);

	panel_bridge = devm_kzalloc(panel->dev, sizeof(*panel_bridge),
				    GFP_KERNEL);
	if (!panel_bridge)
		return ERR_PTR(-ENOMEM);

	panel_bridge->connector_type = connector_type;
	panel_bridge->panel = panel;

	panel_bridge->bridge.funcs = &baikal_lvds_bridge_funcs;
#ifdef CONFIG_OF
	panel_bridge->bridge.of_node = panel->dev->of_node;
#endif

	drm_bridge_add(&panel_bridge->bridge);

	return &panel_bridge->bridge;
}

static void baikal_lvds_bridge_remove(struct drm_bridge *bridge)
{
	struct baikal_lvds_bridge *panel_bridge;

	if (!bridge)
		return;

	if (bridge->funcs != &baikal_lvds_bridge_funcs)
		return;

	panel_bridge = bridge_to_baikal_lvds_bridge(bridge);

	drm_bridge_remove(bridge);
	devm_kfree(panel_bridge->panel->dev, bridge);
}

static void devm_baikal_lvds_bridge_release(struct device *dev, void *res)
{
	struct drm_bridge **bridge = res;

	baikal_lvds_bridge_remove(*bridge);
}

struct drm_bridge *devm_baikal_lvds_bridge_add(struct device *dev,
					     struct drm_panel *panel,
					     u32 connector_type)
{
	struct drm_bridge **ptr, *bridge;

	ptr = devres_alloc(devm_baikal_lvds_bridge_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	bridge = baikal_lvds_bridge_add(panel, connector_type);
	if (!IS_ERR(bridge)) {
		*ptr = bridge;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return bridge;
}

bool bridge_is_baikal_lvds_bridge(const struct drm_bridge *bridge)
{
	return bridge && (bridge->funcs == &baikal_lvds_bridge_funcs);
}
