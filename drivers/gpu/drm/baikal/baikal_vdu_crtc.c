// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

/**
 * baikal_vdu_crtc.c
 * Implementation of the CRTC functions for Baikal Electronics BE-M1000 VDU driver
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dma-buf.h>
#include <linux/shmem_fs.h>
#include <linux/version.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_vblank.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

irqreturn_t baikal_vdu_irq(int irq, void *data)
{
	struct baikal_vdu_private *priv = data;
	irqreturn_t status = IRQ_NONE;
	u32 raw_stat;
	u32 irq_stat;

	priv->counters[0]++;
	irq_stat = readl(priv->regs + IVR);
	raw_stat = readl(priv->regs + ISR);

	if (raw_stat & INTR_UFU) {
		priv->counters[4]++;
		status = IRQ_HANDLED;
	}

	if (raw_stat & INTR_IFO) {
		priv->counters[5]++;
		status = IRQ_HANDLED;
	}

	if (raw_stat & INTR_OFU) {
		priv->counters[6]++;
		status = IRQ_HANDLED;
	}

	if (irq_stat & INTR_FER) {
		priv->counters[11]++;
		priv->counters[12] = readl(priv->regs + DBAR);
		priv->counters[13] = readl(priv->regs + DCAR);
		priv->counters[14] = readl(priv->regs + MRR);
		status = IRQ_HANDLED;
	}

	priv->counters[3] |= raw_stat;

	/* Clear all interrupts */
	writel(raw_stat, priv->regs + ISR);

	return status;
}

static bool baikal_vdu_crtc_is_clk_enabled(struct baikal_vdu_private *priv)
{
	if (acpi_disabled) {
		return __clk_is_enabled(priv->clk);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj = {
			.type = ACPI_TYPE_INTEGER,
			.integer.type = ACPI_TYPE_INTEGER,
			.integer.value = priv->index
	        };
	        struct acpi_object_list args = {
	                .count = 1,
	                .pointer = &obj
	        };
	        union acpi_object *obj2;
		struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
		acpi_status status;
		u64 val;

		status = acpi_evaluate_object(ACPI_COMPANION(priv->drm->dev)->handle,
					      "CISE", &args, &buffer);
		if (ACPI_FAILURE(status))
			return 0;

		obj2 = buffer.pointer;
		if (!obj2 || obj2->type != ACPI_TYPE_INTEGER) {
			kfree(obj2);
			return 0;
		}

		val = obj2->integer.value;
		kfree(obj2);

		return val;
#endif
	}
}

static void baikal_vdu_crtc_clk_enable(struct baikal_vdu_private *priv)
{
	if (acpi_disabled) {
		clk_prepare_enable(priv->clk);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj = {
			.type = ACPI_TYPE_INTEGER,
			.integer.type = ACPI_TYPE_INTEGER,
			.integer.value = priv->index
	        };
	        struct acpi_object_list args = {
	                .count = 1,
	                .pointer = &obj
	        };

		acpi_evaluate_object(ACPI_COMPANION(priv->drm->dev)->handle,
				     "CEN", &args, NULL);
#endif
	}
}

static void baikal_vdu_crtc_clk_disable(struct baikal_vdu_private *priv)
{
	if (acpi_disabled) {
		clk_disable_unprepare(priv->clk);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj = {
			.type = ACPI_TYPE_INTEGER,
			.integer.type = ACPI_TYPE_INTEGER,
			.integer.value = priv->index
	        };
	        struct acpi_object_list args = {
	                .count = 1,
	                .pointer = &obj
	        };

		acpi_evaluate_object(ACPI_COMPANION(priv->drm->dev)->handle,
				     "CDIS", &args, NULL);
#endif
	}
}

static int baikal_vdu_crtc_set_rate(struct baikal_vdu_private *priv, u32 rate)
{
	if (acpi_disabled) {
		return clk_set_rate(priv->clk, rate);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj[2] = {
			{
				.type = ACPI_TYPE_INTEGER,
				.integer.type = ACPI_TYPE_INTEGER,
				.integer.value = priv->index
			},
			{
				.type = ACPI_TYPE_INTEGER,
				.integer.type = ACPI_TYPE_INTEGER,
				.integer.value = rate
			}
	        };
	        struct acpi_object_list args = {
	                .count = 2,
	                .pointer = obj
	        };

		acpi_evaluate_object(ACPI_COMPANION(priv->drm->dev)->handle,
				     "CSET", &args, NULL);
		return 0;
#endif
	}
}

u64 baikal_vdu_crtc_get_rate(struct baikal_vdu_private *priv)
{
	if (acpi_disabled) {
		return clk_get_rate(priv->clk);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj = {
			.type = ACPI_TYPE_INTEGER,
			.integer.type = ACPI_TYPE_INTEGER,
			.integer.value = priv->index
	        };
	        struct acpi_object_list args = {
	                .count = 1,
	                .pointer = &obj
	        };
	        union acpi_object *obj2;
		struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
		acpi_status status;
		u64 val;

		status = acpi_evaluate_object(ACPI_COMPANION(priv->drm->dev)->handle,
					      "CGET", &args, &buffer);
		if (ACPI_FAILURE(status))
			return 0;

		obj2 = buffer.pointer;
		if (!obj2 || obj2->type != ACPI_TYPE_INTEGER) {
			kfree(obj2);
			return 0;
		}

		val = obj2->integer.value;
		kfree(obj2);

		return val;
#endif
	}
}

static void baikal_vdu_crtc_helper_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct baikal_vdu_private *priv = crtc_to_baikal_vdu(crtc);
	const struct drm_display_mode *mode = &crtc->state->mode;
	unsigned long rate;
	unsigned int ppl, hsw, hfp, hbp;
	unsigned int lpp, vsw, vfp, vbp;
	unsigned int reg;
	int ret = 0;

	drm_mode_debug_printmodeline(mode);

	rate = mode->clock * 1000;

	if (rate != baikal_vdu_crtc_get_rate(priv)) {
		DRM_DEV_DEBUG_DRIVER(dev->dev, "Requested pixel clock is %lu Hz\n", rate);

		/* hold clock domain reset; disable clocking */
		writel(0, priv->regs + PCTR);

		if (baikal_vdu_crtc_is_clk_enabled(priv))
			baikal_vdu_crtc_clk_disable(priv);
		ret = baikal_vdu_crtc_set_rate(priv, rate);

		if (ret >= 0) {
			baikal_vdu_crtc_clk_enable(priv);
			if (!baikal_vdu_crtc_is_clk_enabled(priv))
				ret = -1;
		}

		/* release clock domain reset; enable clocking */
		reg = readl(priv->regs + PCTR);
		reg |= PCTR_PCR + PCTR_PCI;
		writel(reg, priv->regs + PCTR);
	}

	if (ret < 0)
		DRM_ERROR("Cannot set desired pixel clock (%lu Hz)\n", rate);

	ppl = mode->hdisplay / 16;
	if (priv->index == CRTC_LVDS && priv-> num_lanes == 2) {
		hsw = mode->hsync_end - mode->hsync_start;
		hfp = mode->hsync_start - mode->hdisplay - 1;
	} else {
		hsw = mode->hsync_end - mode->hsync_start - 1;
		hfp = mode->hsync_start - mode->hdisplay;
	}
	hbp = mode->htotal - mode->hsync_end;

	lpp = mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	writel((HTR_HFP(hfp) & HTR_HFP_MASK) |
			(HTR_PPL(ppl) & HTR_PPL_MASK) |
			(HTR_HBP(hbp) & HTR_HBP_MASK) |
			(HTR_HSW(hsw) & HTR_HSW_MASK),
			priv->regs + HTR);

	if (mode->hdisplay > 4080 || ppl * 16 != mode->hdisplay)
		writel((HPPLOR_HPPLO(mode->hdisplay) & HPPLOR_HPPLO_MASK) | HPPLOR_HPOE,
				priv->regs + HPPLOR);

	writel((VTR1_VSW(vsw) & VTR1_VSW_MASK) |
			(VTR1_VFP(vfp) & VTR1_VFP_MASK) |
			(VTR1_VBP(vbp) & VTR1_VBP_MASK),
			priv->regs + VTR1);

	writel(lpp & VTR2_LPP_MASK, priv->regs + VTR2);

	writel((HVTER_VSWE(vsw >> VTR1_VSW_LSB_WIDTH) & HVTER_VSWE_MASK) |
			(HVTER_HSWE(hsw >> HTR_HSW_LSB_WIDTH) & HVTER_HSWE_MASK) |
			(HVTER_VBPE(vbp >> VTR1_VBP_LSB_WIDTH) & HVTER_VBPE_MASK) |
			(HVTER_VFPE(vfp >> VTR1_VFP_LSB_WIDTH) & HVTER_VFPE_MASK) |
			(HVTER_HBPE(hbp >> HTR_HBP_LSB_WIDTH) & HVTER_HBPE_MASK) |
			(HVTER_HFPE(hfp >> HTR_HFP_LSB_WIDTH) & HVTER_HFPE_MASK),
			priv->regs + HVTER);

	/* Set polarities */
	reg = readl(priv->regs + CR1);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		reg |= CR1_HSP;
	else
		reg &= ~CR1_HSP;
	reg &= ~CR1_VSP; // always set VSP to active high
	reg |= CR1_DEP; // set DE to active high;
	writel(reg, priv->regs + CR1);
}

static enum drm_mode_status baikal_vdu_mode_valid(struct drm_crtc *crtc,
	                const struct drm_display_mode *mode)
{
	struct baikal_vdu_private *priv = crtc_to_baikal_vdu(crtc);
	if (!priv->mode_override && (mode->hdisplay > 2560 ||
			mode->vdisplay > 1440))
		return MODE_BAD;
	else
		return MODE_OK;
}

static void baikal_vdu_crtc_helper_enable(struct drm_crtc *crtc,
					  struct drm_atomic_state *old_state)
{
	struct baikal_vdu_private *priv = crtc_to_baikal_vdu(crtc);
	const char *data_mapping = NULL;
	u32 cntl, gpio;

	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "enabling pixel clock\n");
	baikal_vdu_crtc_clk_enable(priv);

	/* Set 16-word input FIFO watermark */
	/* Enable and Power Up */
	cntl = readl(priv->regs + CR1);
	cntl &= ~(CR1_FDW_MASK | CR1_OPS_MASK);
	cntl |= CR1_LCE | CR1_FDW_16_WORDS;

	if (priv->index == CRTC_LVDS) {
		if (priv->bridge->of_node) {
			of_property_read_string(priv->bridge->of_node,
						"data-mapping", &data_mapping);
		} else {
			struct fwnode_handle *fwnode;

			fwnode = fwnode_find_reference(priv->bridge->dev->dev->fwnode,
						       "baikal,lvds-panel", 0);
			if (!IS_ERR_OR_NULL(fwnode))
				fwnode_property_read_string(fwnode, "data-mapping",
							    &data_mapping);
		}

		if (!data_mapping) {
			cntl |= CR1_OPS_LCD18;
		} else if (!strncmp(data_mapping, "vesa-24", 7))
			cntl |= CR1_OPS_LCD24;
		else if (!strncmp(data_mapping, "jeida-18", 8))
			cntl |= CR1_OPS_LCD18;
		else {
			dev_warn(crtc->dev->dev, "%s data mapping is not supported, vesa-24 is set\n", data_mapping);
			cntl |= CR1_OPS_LCD24;
		}
		gpio = GPIOR_UHD_ENB;
		if (priv->num_lanes == 4)
			gpio |= GPIOR_UHD_QUAD_PORT;
		else if (priv->num_lanes == 2)
			gpio |= GPIOR_UHD_DUAL_PORT;
		else
			gpio |= GPIOR_UHD_SNGL_PORT;
		writel(gpio, priv->regs + GPIOR);
	} else
		cntl |= CR1_OPS_LCD24;
	writel(cntl, priv->regs + CR1);

	writel(0x3ffff, priv->regs + ISR);
	writel(INTR_FER, priv->regs + IMR);
}

static void baikal_vdu_crtc_helper_disable(struct drm_crtc *crtc)
{
	struct baikal_vdu_private *priv = crtc_to_baikal_vdu(crtc);

	writel(0x3ffff, priv->regs + ISR);
	writel(0, priv->regs + IMR);

	/* Disable clock */
	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "disabling pixel clock\n");
	baikal_vdu_crtc_clk_disable(priv);
}

static void baikal_vdu_crtc_helper_atomic_flush(struct drm_crtc *crtc,
					   struct drm_atomic_state *old_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

const struct drm_crtc_funcs crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_set_nofb = baikal_vdu_crtc_helper_mode_set_nofb,
	.mode_valid = baikal_vdu_mode_valid,
	.atomic_flush = baikal_vdu_crtc_helper_atomic_flush,
	.disable = baikal_vdu_crtc_helper_disable,
	.atomic_enable = baikal_vdu_crtc_helper_enable,
};

int baikal_vdu_crtc_create(struct baikal_vdu_private *priv)
{
	struct drm_device *dev = priv->drm;
	struct drm_crtc *crtc = &priv->crtc;

	drm_crtc_init_with_planes(dev, crtc,
				  &priv->primary, NULL,
				  &crtc_funcs, "primary");
	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "enabling pixel clock\n");
	baikal_vdu_crtc_clk_enable(priv);

	return 0;
}
