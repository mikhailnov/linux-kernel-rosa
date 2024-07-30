// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#include <linux/seq_file.h>
#include <linux/device.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_device.h>
#include <drm/drm_file.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define REGDEF(reg) { reg, #reg }
static const struct {
	u32 reg;
	const char *name;
} baikal_vdu_reg_defs[] = {
	REGDEF(CR1),
	REGDEF(HTR),
	REGDEF(VTR1),
	REGDEF(VTR2),
	REGDEF(PCTR),
	REGDEF(ISR),
	REGDEF(IMR),
	REGDEF(IVR),
	REGDEF(ISCR),
	REGDEF(DBAR),
	REGDEF(DCAR),
	REGDEF(DEAR),
	REGDEF(PWMFR),
	REGDEF(PWMDCR),
	REGDEF(HVTER),
	REGDEF(HPPLOR),
	REGDEF(GPIOR),
	REGDEF(MRR),
};

#define REGS_HDMI	"regs_hdmi"
#define REGS_LVDS	"regs_lvds"

static int baikal_vdu_debugfs_regs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(dev);
	char *filename = m->file->f_path.dentry->d_iname;
	struct baikal_vdu_private *priv = NULL;
	int i;

	if (!strcmp(REGS_HDMI, filename))
		priv = &crossbar->hdmi;
	if (!strcmp(REGS_LVDS, filename))
		priv = &crossbar->lvds;

	if (!priv || !priv->regs)
		return 0;

	for (i = 0; i < ARRAY_SIZE(baikal_vdu_reg_defs); i++) {
		seq_printf(m, "%s (0x%04x): 0x%08x\n",
			   baikal_vdu_reg_defs[i].name, baikal_vdu_reg_defs[i].reg,
			   readl(priv->regs + baikal_vdu_reg_defs[i].reg));
	}
	for (i = 0; i < ARRAY_SIZE(priv->counters); i++) {
		seq_printf(m, "COUNTER[%d]: 0x%08x\n", i, priv->counters[i]);
	}

	return 0;
}

static const struct drm_info_list baikal_vdu_hdmi_debugfs_list[] = {
	{REGS_HDMI, baikal_vdu_debugfs_regs, 0},
};

static const struct drm_info_list baikal_vdu_lvds_debugfs_list[] = {
	{REGS_LVDS, baikal_vdu_debugfs_regs, 0},
};

void baikal_vdu_hdmi_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(baikal_vdu_hdmi_debugfs_list,
				ARRAY_SIZE(baikal_vdu_hdmi_debugfs_list),
				minor->debugfs_root, minor);
}

void baikal_vdu_lvds_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(baikal_vdu_lvds_debugfs_list,
				ARRAY_SIZE(baikal_vdu_lvds_debugfs_list),
				minor->debugfs_root, minor);
}
