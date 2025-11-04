// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal LVDS panel driver
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 *
 * Implementation based on panel-lvds.c
 */

#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#include <video/display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

struct panel_lvds {
	struct drm_panel panel;
	struct device *dev;
	unsigned int width;
	unsigned int height;
	struct drm_display_mode dmode;
	u32 bus_flags;
	unsigned int bus_format;
	enum drm_panel_orientation orientation;
};

static inline struct panel_lvds *to_panel_lvds(struct drm_panel *panel)
{
	return container_of(panel, struct panel_lvds, panel);
}

static int panel_lvds_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &lvds->dmode);
	if (!mode)
		return 0;

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = lvds->dmode.width_mm;
	connector->display_info.height_mm = lvds->dmode.height_mm;
	drm_display_info_set_bus_formats(&connector->display_info,
					 &lvds->bus_format, 1);
	connector->display_info.bus_flags = lvds->bus_flags;

	/*
	 * TODO: Remove once all drm drivers call
	 * drm_connector_set_orientation_from_panel()
	 */
	drm_connector_set_panel_orientation(connector, lvds->orientation);

	return 1;
}

static enum drm_panel_orientation panel_lvds_get_orientation(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);

	return lvds->orientation;
}

static const struct drm_panel_funcs panel_lvds_funcs = {
	.get_modes = panel_lvds_get_modes,
	.get_orientation = panel_lvds_get_orientation,
};

static int panel_lvds_get_data_mapping(struct device *dev)
{
	const char *mapping;
	int ret;

	ret = device_property_read_string(dev, "data-mapping", &mapping);
	if (ret < 0)
		return -ENODEV;

	if (!strcmp(mapping, "jeida-18"))
		return MEDIA_BUS_FMT_RGB666_1X7X3_SPWG;
	if (!strcmp(mapping, "jeida-24"))
		return MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA;
	if (!strcmp(mapping, "vesa-24"))
		return MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;

	return -EINVAL;
}

static int parse_timing_property(struct device *dev, const char *name,
				 struct timing_entry *result)
{
	int count, ret;

	count = device_property_count_u32(dev, name);
	if (count == 1) {
		ret = device_property_read_u32(dev, name, &result->typ);
		result->min = result->typ;
		result->max = result->typ;
	} else if (count == 3) {
		ret = device_property_read_u32_array(dev, name, &result->min, count);
	} else {
		dev_err(dev, "illegal timing specification in %s\n", name);
		return -EINVAL;
	}

	return ret;
}

static int panel_lvds_get_display_timing(struct device *dev,
					 struct display_timing *dt)
{
	u32 val = 0;
	int ret = 0;

	memset(dt, 0, sizeof(*dt));

	ret |= parse_timing_property(dev, "hback-porch", &dt->hback_porch);
	ret |= parse_timing_property(dev, "hfront-porch", &dt->hfront_porch);
	ret |= parse_timing_property(dev, "hactive", &dt->hactive);
	ret |= parse_timing_property(dev, "hsync-len", &dt->hsync_len);
	ret |= parse_timing_property(dev, "vback-porch", &dt->vback_porch);
	ret |= parse_timing_property(dev, "vfront-porch", &dt->vfront_porch);
	ret |= parse_timing_property(dev, "vactive", &dt->vactive);
	ret |= parse_timing_property(dev, "vsync-len", &dt->vsync_len);
	ret |= parse_timing_property(dev, "clock-frequency", &dt->pixelclock);

	dt->flags = 0;
	if (!device_property_read_u32(dev, "vsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_VSYNC_HIGH :
				DISPLAY_FLAGS_VSYNC_LOW;
	if (!device_property_read_u32(dev, "hsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_HSYNC_HIGH :
				DISPLAY_FLAGS_HSYNC_LOW;
	if (!device_property_read_u32(dev, "de-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_DE_HIGH :
				DISPLAY_FLAGS_DE_LOW;
	if (!device_property_read_u32(dev, "pixelclk-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_PIXDATA_POSEDGE :
				DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	if (!device_property_read_u32(dev, "syncclk-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_SYNC_POSEDGE :
				DISPLAY_FLAGS_SYNC_NEGEDGE;
	else if (dt->flags & (DISPLAY_FLAGS_PIXDATA_POSEDGE |
			      DISPLAY_FLAGS_PIXDATA_NEGEDGE))
		dt->flags |= dt->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE ?
				DISPLAY_FLAGS_SYNC_POSEDGE :
				DISPLAY_FLAGS_SYNC_NEGEDGE;

	if (device_property_read_bool(dev, "interlaced"))
		dt->flags |= DISPLAY_FLAGS_INTERLACED;
	if (device_property_read_bool(dev, "doublescan"))
		dt->flags |= DISPLAY_FLAGS_DOUBLESCAN;
	if (device_property_read_bool(dev, "doubleclk"))
		dt->flags |= DISPLAY_FLAGS_DOUBLECLK;

	if (ret) {
		dev_err(dev, "error reading timing properties\n");
		return -EINVAL;
	}

	return 0;
}

static int panel_lvds_get_display_mode(struct device *dev,
				       struct drm_display_mode *dmode,
				       u32 *bus_flags)
{
	u32 width_mm = 0, height_mm = 0;
	struct display_timing timing;
	struct videomode vm;
	int ret;

	ret = panel_lvds_get_display_timing(dev, &timing);
	if (ret)
		return ret;

	videomode_from_timing(&timing, &vm);

	memset(dmode, 0, sizeof(*dmode));
	drm_display_mode_from_videomode(&vm, dmode);
	if (bus_flags)
		drm_bus_flags_from_videomode(&vm, bus_flags);

	ret = device_property_read_u32(dev, "width-mm", &width_mm);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "height-mm", &height_mm);
	if (ret)
		return ret;

	dmode->width_mm = width_mm;
	dmode->height_mm = height_mm;

	drm_mode_debug_printmodeline(dmode);

	return 0;
}

static int panel_lvds_get_panel_orientation(struct device *dev,
					    enum drm_panel_orientation *orientation)
{
	int rotation, ret;

	ret = device_property_read_u32(dev, "rotation", &rotation);
	if (ret == -EINVAL) {
		/* Don't return an error if there's no rotation property. */
		*orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;
		return 0;
	}

	if (ret < 0)
		return ret;

	if (rotation == 0)
		*orientation = DRM_MODE_PANEL_ORIENTATION_NORMAL;
	else if (rotation == 90)
		*orientation = DRM_MODE_PANEL_ORIENTATION_RIGHT_UP;
	else if (rotation == 180)
		*orientation = DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP;
	else if (rotation == 270)
		*orientation = DRM_MODE_PANEL_ORIENTATION_LEFT_UP;
	else
		return -EINVAL;

	return 0;
}

static int panel_lvds_parse(struct panel_lvds *lvds)
{
	struct device *dev = lvds->dev;
	int ret;

	ret = panel_lvds_get_panel_orientation(dev, &lvds->orientation);
	if (ret < 0) {
		dev_err(dev, "failed to get orientation %d\n", ret);
		return ret;
	}

	ret = panel_lvds_get_display_mode(dev, &lvds->dmode, &lvds->bus_flags);
	if (ret < 0) {
		dev_err(dev, "problems parsing panel-timing (%d)\n", ret);
		return ret;
	}

	ret = panel_lvds_get_data_mapping(dev);
	if (ret < 0) {
		dev_err(dev, "invalid or missing data-mapping property\n");
		return ret;
	}

	lvds->bus_format = ret;

	lvds->bus_flags |= device_property_read_bool(dev, "data-mirror") ?
			   DRM_BUS_FLAG_DATA_LSB_TO_MSB :
			   DRM_BUS_FLAG_DATA_MSB_TO_LSB;

	return 0;
}

static int panel_lvds_probe(struct platform_device *pdev)
{
	struct panel_lvds *lvds;
	int ret;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = &pdev->dev;

	ret = panel_lvds_parse(lvds);
	if (ret < 0)
		return ret;

	/* Register the panel. */
	drm_panel_init(&lvds->panel, lvds->dev, &panel_lvds_funcs,
		       DRM_MODE_CONNECTOR_LVDS);

	drm_panel_add(&lvds->panel);

	dev_set_drvdata(lvds->dev, lvds);
	return 0;
}

static void panel_lvds_remove(struct platform_device *pdev)
{
	struct panel_lvds *lvds = platform_get_drvdata(pdev);

	drm_panel_remove(&lvds->panel);

	drm_panel_disable(&lvds->panel);
}

static const struct of_device_id panel_lvds_of_table[] = {
	{ .compatible = "baikal,panel-lvds", },
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, panel_lvds_of_table);

static struct platform_driver panel_baikal_lvds_driver = {
	.probe		= panel_lvds_probe,
	.remove		= panel_lvds_remove,
	.driver		= {
		.name	= "panel-baikal-lvds",
		.of_match_table = panel_lvds_of_table,
	},
};

module_platform_driver(panel_baikal_lvds_driver);

MODULE_AUTHOR("Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal LVDS Panel Driver");
MODULE_LICENSE("GPL");
