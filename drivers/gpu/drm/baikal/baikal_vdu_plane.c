// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

static void baikal_vdu_primary_plane_atomic_update(struct drm_plane *plane,
					      struct drm_atomic_state *old_state)
{
	struct baikal_vdu_private *priv;
	struct drm_plane_state *state = plane->state;
	struct drm_crtc *crtc = state->crtc;
	struct drm_framebuffer *fb = state->fb;
	uint32_t cntl;
	uint32_t addr;
	unsigned long flags;

	if (!fb)
		return;

	priv = crtc_to_baikal_vdu(crtc);
	addr = drm_fb_dma_get_gem_addr(fb, state, 0);

	spin_lock_irqsave(&priv->lock, flags);
	writel(addr, priv->regs + DBAR);
	spin_unlock_irqrestore(&priv->lock, flags);
	priv->counters[16]++;

	cntl = readl(priv->regs + CR1);
	cntl &= ~CR1_BPP_MASK;

	/* Note that the the hardware's format reader takes 'r' from
	 * the low bit, while DRM formats list channels from high bit
	 * to low bit as you read left to right.
	 */
	switch (fb->format->format) {
	case DRM_FORMAT_BGR888:
		cntl |= CR1_BPP24 | CR1_FBP | CR1_BGR;
		break;
	case DRM_FORMAT_RGB888:
		cntl |= CR1_BPP24 | CR1_FBP;
		break;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		cntl |= CR1_BPP24 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		cntl |= CR1_BPP24;
		break;
	case DRM_FORMAT_BGR565:
		cntl |= CR1_BPP16_565 | CR1_BGR;
		break;
	case DRM_FORMAT_RGB565:
		cntl |= CR1_BPP16_565;
		break;
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		cntl |= CR1_BPP16_555 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		cntl |= CR1_BPP16_555;
		break;
	default:
		WARN_ONCE(true, "Unknown FB format 0x%08x, set XRGB8888 instead\n",
				fb->format->format);
		cntl |= CR1_BPP24;
		break;
	}

	writel(cntl, priv->regs + CR1);

	spin_lock_irqsave(&priv->lock, flags);
	writel((fb->pitches[0] / DIV_ROUND_UP(drm_format_info_bpp(fb->format, 0), 8)) |
		HPPLOR_HPOE, priv->regs + HPPLOR);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static const struct drm_plane_helper_funcs baikal_vdu_primary_plane_helper_funcs = {
	.atomic_update = baikal_vdu_primary_plane_atomic_update,
};

static const struct drm_plane_funcs baikal_vdu_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int baikal_vdu_primary_plane_init(struct baikal_vdu_private *priv)
{
	struct drm_device *drm = priv->drm;
	struct drm_plane *plane = &priv->primary;
	static const u32 formats[] = {
		DRM_FORMAT_BGR888,
		DRM_FORMAT_RGB888,
		DRM_FORMAT_ABGR8888,
		DRM_FORMAT_XBGR8888,
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_BGR565,
		DRM_FORMAT_RGB565,
		DRM_FORMAT_ABGR1555,
		DRM_FORMAT_XBGR1555,
		DRM_FORMAT_ARGB1555,
		DRM_FORMAT_XRGB1555,
	};
	int ret;

	ret = drm_universal_plane_init(drm, plane, 0,
				       &baikal_vdu_primary_plane_funcs,
				       formats,
				       ARRAY_SIZE(formats),
				       NULL,
				       DRM_PLANE_TYPE_PRIMARY,
				       NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &baikal_vdu_primary_plane_helper_funcs);

	return 0;
}
