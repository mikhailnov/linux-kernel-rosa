// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 * Bugfixes by Alexey Sheplyakov <asheplyakov@altlinux.org>
 *
 */

#include <linux/arm-smccc.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define DRIVER_NAME		"baikal-vdu"
#define DRIVER_DESC		"Baikal VDU DRM driver"
#define DRIVER_DATE		"20230221"

#define BAIKAL_SMC_LOG_DISABLE	0xC2000200
#define IFIFO_SIZE	16384
#define UHD_FIFO_SIZE	16384

int mode_override = 0;
int hdmi_off = 0;
int lvds_off = 0;

#define legacy_LDVS	1
#define legacy_HDMI	2

static struct drm_driver vdu_drm_driver;

static struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

const struct drm_encoder_funcs baikal_vdu_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct drm_bridge *devm_baikal_get_bridge(struct device *dev,
						 u32 port, u32 endpoint)
{
	struct baikal_hdmi_bridge *priv_hdmi;
	struct device *tmp;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct fwnode_handle *fwnode;
	struct drm_device *drm = dev_get_drvdata(dev);
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(drm);
	int ret = 0;

	if (is_of_node(dev->fwnode)) {
		ret = drm_of_find_panel_or_bridge(to_of_node(dev->fwnode),
						  crossbar->legacy ? 0 : port,
						  endpoint, &panel, &bridge);
	} else {
		if (port == CRTC_HDMI) {
			fwnode = fwnode_find_reference(dev->fwnode,
						       "baikal,hdmi-bridge", 0);
			if (IS_ERR_OR_NULL(fwnode))
				return ERR_PTR(-ENODEV);

			tmp = bus_find_device_by_fwnode(&platform_bus_type, fwnode);
			if (IS_ERR_OR_NULL(tmp))
				return ERR_PTR(-ENODEV);

			priv_hdmi = dev_get_drvdata(tmp);
			if (!priv_hdmi)
				return ERR_PTR(-EPROBE_DEFER);

			bridge = priv_hdmi->bridge;
			panel = NULL;
		} else if (port == CRTC_LVDS) {
			fwnode = fwnode_find_reference(dev->fwnode,
						       "baikal,lvds-panel", 0);
			if (IS_ERR_OR_NULL(fwnode))
				return ERR_PTR(-ENODEV);

			panel = fwnode_drm_find_panel(fwnode);
			if (IS_ERR(panel))
				return ERR_CAST(panel);
		} else {
			return ERR_PTR(-ENODEV);
		}
	}

	if (ret)
		return ERR_PTR(ret);

	if (panel) {
		bridge = devm_baikal_lvds_bridge_add(dev, panel, DRM_MODE_CONNECTOR_LVDS);
	}

	return bridge;
}

static int baikal_vdu_remove_efifb(struct drm_device *dev)
{
	int err;
	err = drm_aperture_remove_framebuffers(&vdu_drm_driver);
	if (err)
		dev_warn(dev->dev, "failed to remove firmware framebuffer\n");
	return err;
}

inline void baikal_vdu_switch_off(struct baikal_vdu_private *priv)
{
	u32 cntl = readl(priv->regs + CR1);
	cntl &= ~CR1_LCE;
	writel(cntl, priv->regs + CR1);
}

inline void baikal_vdu_switch_on(struct baikal_vdu_private *priv)
{
	u32 cntl = readl(priv->regs + CR1);
	cntl |= CR1_LCE;
	writel(cntl, priv->regs + CR1);
}

static int baikal_vdu_modeset_init(struct baikal_vdu_private *priv)
{
	struct drm_device *dev = priv->drm;
	struct drm_encoder *encoder;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	ret = baikal_vdu_primary_plane_init(priv);
	if (ret != 0) {
		dev_err(dev->dev, "%s: failed to init primary plane\n", priv->name);
		return ret;
	}

	ret = baikal_vdu_crtc_create(priv);
	if (ret) {
		dev_err(dev->dev, "%s: failed to create CRTC\n", priv->name);
		return ret;
	}

	if (priv->bridge) {
		encoder = &priv->encoder;
		ret = drm_encoder_init(dev, encoder, &baikal_vdu_encoder_funcs,
				       DRM_MODE_ENCODER_NONE, NULL);
		if (ret) {
			dev_err(dev->dev, "%s: failed to create DRM encoder\n", priv->name);
			return ret;
		}
		encoder->crtc = &priv->crtc;
		encoder->possible_crtcs = BIT(drm_crtc_index(encoder->crtc));
		priv->bridge->encoder = encoder;
		ret = drm_bridge_attach(encoder, priv->bridge, NULL, 0);
		if (ret) {
			dev_err(dev->dev, "%s: failed to attach DRM bridge (%d)\n", priv->name, ret);
			return ret;
		}
	} else {
		dev_err(dev->dev, "%s: no bridge or panel attached\n", priv->name);
		return -ENODEV;
	}

	priv->mode_override = mode_override;

	return ret;
}

static int baikal_dumb_create(struct drm_file *file, struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height + IFIFO_SIZE + UHD_FIFO_SIZE;

	return drm_gem_dma_dumb_create_internal(file, dev, args);
}

DEFINE_DRM_GEM_DMA_FOPS(baikal_drm_fops);

static struct drm_driver vdu_drm_driver = {
	.driver_features = DRIVER_GEM |	DRIVER_MODESET | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	.ioctls = NULL,
	.fops = &baikal_drm_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = 2,
	.minor = 0,
	.patchlevel = 0,
	.dumb_create = baikal_dumb_create,
	.dumb_map_offset = drm_gem_dumb_map_offset,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = drm_gem_dma_prime_import_sg_table,
};

static int baikal_vdu_allocate_resources(struct platform_device *pdev,
		struct baikal_vdu_private *priv)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm = priv->drm;
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(drm);
	struct resource *mem;
	int ret;

	if (crossbar->legacy)
		mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	else {
		mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, priv->regs_name);
		if (!mem)
			mem = platform_get_resource(pdev, IORESOURCE_MEM, priv->index);
	}

	if (!mem) {
		dev_err(dev, "%s %s: no MMIO resource specified\n", __func__, priv->name);
		return -EINVAL;
	}

	priv->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(priv->regs)) {
		dev_err(dev, "%s %s: MMIO allocation failed\n", __func__, priv->name);
		return PTR_ERR(priv->regs);
	}

	if (priv->off) {
		baikal_vdu_switch_off(priv);
		return -EPERM;
	} else {
		ret = baikal_vdu_modeset_init(priv);
		if (ret) {
			dev_err(dev, "%s %s: failed to init modeset\n", __func__, priv->name);
			if (ret == -ENODEV) {
				baikal_vdu_switch_off(priv);
			}
			return ret;
		} else {
			writel(MRR_MAX_VALUE, priv->regs + MRR);
			spin_lock_init(&priv->lock);
			return 0;
		}
	}
}

static int baikal_vdu_allocate_irq(struct platform_device *pdev,
	       struct baikal_vdu_private *priv)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm = priv->drm;
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(drm);
	int ret;

	if (crossbar->legacy)
		priv->irq = fwnode_irq_get(dev->fwnode, 0);
	else
		priv->irq = fwnode_irq_get_byname(dev->fwnode, priv->irq_name);
	if (priv->irq < 0) {
		dev_err(dev, "%s %s: no IRQ resource specified\n", __func__, priv->name);
		return -EINVAL;
	}

	/* turn off interrupts before requesting the irq */
	baikal_vdu_set_irq(priv, false, false);
	ret = request_irq(priv->irq, baikal_vdu_irq, IRQF_SHARED, dev->driver->name, priv);
	if (ret != 0)
		dev_err(dev, "%s %s: IRQ %d allocation failed\n", __func__, priv->name, priv->irq);
	return ret;
}

static void baikal_vdu_free_irq(struct baikal_vdu_private *priv)
{
	baikal_vdu_set_irq(priv, false, false);
	free_irq(priv->irq, priv->drm->dev);
}

static int baikal_vdu_allocate_clk(struct baikal_vdu_private *priv)
{
	if (is_of_node(priv->drm->dev->fwnode)) {
		priv->clk = clk_get(priv->drm->dev, priv->pclk_name);
		if (IS_ERR(priv->clk)) {
			dev_err(priv->drm->dev, "%s: unable to get %s, err %ld\n", priv->name, priv->pclk_name, PTR_ERR(priv->clk));
			return PTR_ERR(priv->clk);
		}
	}

	return 0;
}

static void baikal_vdu_set_name(struct baikal_vdu_private *priv, int index, const char *name)
{
	char *c;
	int len = sizeof(priv->name) / sizeof(priv->name[0]) - 1;
	strncpy(priv->name, name, len);
	for (c = priv->name; c < priv->name + len && *c; c++) {
		*c = toupper(*c);
	}
	sprintf(priv->irq_name, "%s_irq", name);
	sprintf(priv->pclk_name, "%s_pclk", name);
	sprintf(priv->regs_name, "%s_regs", name);
	priv->index = index;
}

static int baikal_vdu_bridge_init(struct baikal_vdu_private *priv, struct drm_device *drm) {
	int ret = 0;
	struct device *dev;
	struct drm_bridge *bridge;
	if (!priv || !drm)
		return -ENODEV;
	priv->drm = drm;
	dev = drm->dev;
	bridge = devm_baikal_get_bridge(dev, priv->index, 0);
	if (IS_ERR(bridge)) {
		ret = PTR_ERR(bridge);
		if (ret == -EPROBE_DEFER) {
			dev_info(dev, "%s: bridge probe deferred\n", priv->name);
		}
		priv->bridge = NULL;
	} else {
		priv->bridge = bridge;
	}
	return ret;
}

static int baikal_vdu_resources_init(struct platform_device *pdev, struct baikal_vdu_private *priv)
{
	int ret = baikal_vdu_allocate_resources(pdev, priv);
	if (ret)
		return 0;
	ret = baikal_vdu_allocate_irq(pdev, priv);
	if (ret)
		return 0;
	ret = baikal_vdu_allocate_clk(priv);
	if (ret) {
		baikal_vdu_free_irq(priv);
		return 0;
	} else {
		return 1;
	}
}

static int baikal_vdu_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_lvds_bridge *panel_bridge;
	struct baikal_vdu_crossbar *crossbar;
	struct baikal_vdu_private *hdmi;
	struct baikal_vdu_private *lvds;
	struct drm_device *drm;
	struct drm_mode_config *mode_config;
	struct arm_smccc_res res;
	struct device_node *node;
	int node_count;
	int ret;

	crossbar = devm_drm_dev_alloc(dev, &vdu_drm_driver,
                  struct baikal_vdu_crossbar, drm);
	if (IS_ERR(crossbar))
		return PTR_ERR(crossbar);

	drm = &crossbar->drm;
	platform_set_drvdata(pdev, drm);
	crossbar->legacy = 0;
	hdmi = &crossbar->hdmi;
	baikal_vdu_set_name(hdmi, CRTC_HDMI, "hdmi");
	lvds = &crossbar->lvds;
	baikal_vdu_set_name(lvds, CRTC_LVDS, "lvds");

	node = NULL;
	node_count = 0;
	while ((node = of_find_compatible_node(node, NULL, "baikal,vdu")))
		node_count++;

	if (node_count > 1) {
		dev_info(dev, "Found separate description of VDU devices\n");
		if (device_property_present(dev, "lvds-out")) {
			// LVDS
			crossbar->legacy = legacy_LDVS;
			sprintf(lvds->pclk_name, "pclk");
			lvds->index = CRTC_LVDS;
		} else {
			// HDMI
			crossbar->legacy = legacy_HDMI;
			sprintf(hdmi->pclk_name, "pclk");
			hdmi->index = CRTC_HDMI;
		}
	}

	if (crossbar->legacy != legacy_LDVS) {
		ret = baikal_vdu_bridge_init(hdmi, drm);
		if (ret == -EPROBE_DEFER) {
			goto out_drm;
		}
	}

	if (crossbar->legacy != legacy_HDMI) {
		ret = device_property_read_u32(&pdev->dev, "lvds-lanes",
					       &lvds->num_lanes);
		if (ret) {
			if (crossbar->legacy) {
				lvds->num_lanes = of_graph_get_endpoint_count(dev->of_node);
				if (lvds->num_lanes < 0)
					lvds->num_lanes = 0;
			} else
				lvds->num_lanes = 0;
		}
		if (lvds->num_lanes) {
			ret = baikal_vdu_bridge_init(lvds, drm);
			if (ret == -EPROBE_DEFER) {
				goto out_drm;
			}
		}
	}

	drm_mode_config_init(drm);
	mode_config = &drm->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 8192;
	mode_config->min_height = 1;
	mode_config->max_height = 8192;

	hdmi->off = hdmi_off;
	hdmi->ready = (crossbar->legacy != legacy_LDVS) && baikal_vdu_resources_init(pdev, hdmi);
	if (lvds->num_lanes) {
		lvds->off = lvds_off;
		lvds->ready = (crossbar->legacy != legacy_HDMI) && baikal_vdu_resources_init(pdev, lvds);
	} else {
		lvds->ready = 0;
		lvds->bridge = NULL;
		dev_info(dev, "No 'lvds-lanes' property found\n");
	}
	if (lvds->ready) {
		ret = baikal_vdu_backlight_create(drm);
		if (ret) {
			dev_err(dev, "LVDS: failed to create backlight\n");
		}
		if (bridge_is_baikal_lvds_bridge(lvds->bridge)) {
			panel_bridge = bridge_to_baikal_lvds_bridge(lvds->bridge);
			panel_bridge->vdu = lvds;
		} else {
		// TODO implement handling of third-party bridges
		}
	}
	if (hdmi->bridge) {
		// TODO implement functions specific to HDMI bridge
	}

	hdmi->ready = hdmi->ready & !hdmi->off;
	lvds->ready = lvds->ready & !lvds->off;
	dev_info(dev, "%s output %s\n", hdmi->name, hdmi->ready ? "enabled" : "disabled");
	dev_info(dev, "%s output %s\n", lvds->name, lvds->ready ? "enabled" : "disabled");

	ret = drm_vblank_init(drm, (hdmi->ready ? 1 : 0) +
				   (lvds->ready ? 1 : 0));
	if (ret) {
		dev_err(dev, "failed to init vblank\n");
		goto out_config;
	}

	baikal_vdu_remove_efifb(drm);

	if (hdmi->ready || lvds->ready) {
		/* Disable SCP debug output as it may affect VDU performance */
		arm_smccc_smc(BAIKAL_SMC_LOG_DISABLE, 0, 0, 0, 0, 0, 0, 0, &res);

		drm_mode_config_reset(drm);
		drm_kms_helper_poll_init(drm);
		ret = drm_dev_register(drm, 0);
		if (ret) {
			dev_err(dev, "failed to register DRM device\n");
			goto out_config;
		}

		drm_fbdev_dma_setup(drm, 32);

#if defined(CONFIG_DEBUG_FS)
		if (hdmi->ready)
			baikal_vdu_hdmi_debugfs_init(drm->primary);
		if (lvds->ready)
			baikal_vdu_lvds_debugfs_init(drm->primary);
#endif
		return 0;
	} else {
		dev_err(dev, "no active outputs configured\n");
		ret = -ENODEV;
	}
out_config:
	drm_mode_config_cleanup(drm);
out_drm:
	dev_err(dev, "failed to probe: %d\n", ret);
	return ret;
}

static void baikal_vdu_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(drm);

	drm_dev_unregister(drm);
	drm_mode_config_cleanup(drm);
	if (crossbar->hdmi.irq)
		free_irq(crossbar->hdmi.irq, drm->dev);
	if (crossbar->lvds.irq)
		free_irq(crossbar->lvds.irq, drm->dev);
}

static const struct of_device_id baikal_vdu_of_match[] = {
	{ .compatible = "baikal,vdu" },
	{ },
};
MODULE_DEVICE_TABLE(of, baikal_vdu_of_match);

static int baikal_vdu_pm_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(drm);
}

static int baikal_vdu_pm_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(baikal_vdu_pm_ops, baikal_vdu_pm_suspend, baikal_vdu_pm_resume);

static struct platform_driver baikal_vdu_platform_driver = {
	.probe      = baikal_vdu_drm_probe,
	.remove_new = baikal_vdu_drm_remove,
	.driver     = {
		.name   = DRIVER_NAME,
		.of_match_table = baikal_vdu_of_match,
		.pm = pm_sleep_ptr(&baikal_vdu_pm_ops)
	},
};

module_param(mode_override, int, 0644);
module_param(hdmi_off, int, 0644);
module_param(lvds_off, int, 0644);

module_platform_driver(baikal_vdu_platform_driver);

MODULE_AUTHOR("Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics BE-M1000 Video Display Unit (VDU) DRM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_SOFTDEP("pre: baikal_hdmi");
