/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#ifndef __BAIKAL_VDU_DRM_H__
#define __BAIKAL_VDU_DRM_H__

#include <drm/drm_bridge.h>
#include <drm/drm_gem.h>
#include <drm/drm_simple_kms_helper.h>

#include <drm/bridge/dw_hdmi.h>

#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/backlight.h>

#define CRTC_HDMI	0
#define CRTC_LVDS	1

#define crtc_to_baikal_vdu(x) \
	container_of(x, struct baikal_vdu_private, crtc)
#define bridge_to_baikal_lvds_bridge(x) \
	container_of(x, struct baikal_lvds_bridge, bridge)
#define connector_to_baikal_lvds_bridge(x) \
	container_of(x, struct baikal_lvds_bridge, connector)
#define drm_to_baikal_vdu_crossbar(x) \
	container_of(x, struct baikal_vdu_crossbar, drm)

#define VDU_NAME_LEN 5

struct baikal_vdu_private {
	struct drm_device *drm;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_plane primary;
	void *regs;
	int irq;
	struct clk *clk;
	spinlock_t lock;
	u32 counters[20];
	int mode_override;
	int index;
	char name[VDU_NAME_LEN];
	char irq_name[VDU_NAME_LEN + 4];
	char pclk_name[VDU_NAME_LEN + 5];
	char regs_name[VDU_NAME_LEN + 5];
	int num_lanes;
	int data_mapping;
	int off;
	int ready;

	/* backlight */
	struct gpio_desc *enable_gpio;
	struct backlight_device *bl_dev;
	int min_brightness;
	int brightness_step;
	bool brightness_on;
};

struct baikal_vdu_crossbar {
	struct drm_device drm;
	struct baikal_vdu_private hdmi;
	struct baikal_vdu_private lvds;
	int legacy;
};

struct baikal_lvds_bridge {
	struct baikal_vdu_private *vdu;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_panel *panel;
	u32 connector_type;
};

struct baikal_hdmi_bridge {
	struct dw_hdmi *hdmi;
	struct drm_bridge *bridge;
};

/* Generic functions */
inline void baikal_vdu_switch_on(struct baikal_vdu_private *priv);

inline void baikal_vdu_switch_off(struct baikal_vdu_private *priv);

/* Bridge functions */
bool bridge_is_baikal_lvds_bridge(const struct drm_bridge *bridge);

struct drm_bridge *devm_baikal_lvds_bridge_add(struct device *dev,
					     struct drm_panel *panel,
					     u32 connector_type);

/* CRTC Functions */
u64 baikal_vdu_crtc_get_rate(struct baikal_vdu_private *priv);

int baikal_vdu_crtc_create(struct baikal_vdu_private *priv);

irqreturn_t baikal_vdu_irq(int irq, void *data);

int baikal_vdu_primary_plane_init(struct baikal_vdu_private *priv);

/* Backlight Functions */
int baikal_vdu_backlight_create(struct drm_device *drm);

/* Debugfs functions */
void baikal_vdu_hdmi_debugfs_init(struct drm_minor *minor);

void baikal_vdu_lvds_debugfs_init(struct drm_minor *minor);

#endif /* __BAIKAL_VDU_DRM_H__ */
