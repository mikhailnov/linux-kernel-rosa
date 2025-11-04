// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare XPCS PMA driver
 *
 * Copyright (C) 2024 Serge Semin
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mdio.h>
#include <linux/pcs/pcs-xpcs.h>

#include "pcs-xpcs.h"

/* DesignWare Gen5 10G PHY can be clocked by an external clock source.
 * It will be enabled instead of the internal one in case if it's specified.
 */
static int xpcs_gen5_10g_ref_clock_select(struct dw_xpcs *xpcs, bool reset)
{
	int ret;

	ret = xpcs_read_vendor(xpcs, MDIO_MMD_PMAPMD,
			       DW_VR_XS_PMA_GEN5_10G_MPLL_CTRL);
	if (ret < 0)
		return ret;

	if (xpcs->clks[DW_XPCS_PAD_CLK].clk || !xpcs->mdiodev->dev.of_node)
		ret &= ~DW_VR_XS_PMA_REF_CLK_SEL_CORE;
	else if (xpcs->clks[DW_XPCS_CORE_CLK].clk)
		ret |= DW_VR_XS_PMA_REF_CLK_SEL_CORE;
	else if (reset)
		goto out_vendor_reset;
	else
		return 0;

	ret = xpcs_write_vendor(xpcs, MDIO_MMD_PMAPMD,
				DW_VR_XS_PMA_GEN5_10G_MPLL_CTRL, ret);
	if (ret < 0)
		return ret;

	/* Vendor reset must be immediately performed. Note it will workout
	 * only if Tx/Rx are stable (Power_Good state).
	 */
out_vendor_reset:
	return xpcs_vendor_reset(xpcs);
}

static int xpcs_10gbaser_gen5_10g_pma_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_read_vendor(xpcs, MDIO_MMD_PMAPMD,
			       DW_VR_XS_PMA_GEN5_10G_GEN_CTRL);
	if (ret < 0)
		return ret;

	/* Select KR-mode of the PHY lanes */
	ret &= ~DW_VR_XS_PMA_LANE_MODE;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LANE_MODE, DW_VR_XS_PMA_LANE_MODE_KR);

	/* Activate 1 lane per link */
	ret &= ~DW_VR_XS_PMA_LINK_WIDTH;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LINK_WIDTH, DW_VR_XS_PMA_LINK_WIDTH_1);

	/* Disable unused 1-3 lanes */
	ret &= ~DW_VR_XS_PMA_LANE_PWR_OFF;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LANE_PWR_OFF, 0xe);

	ret = xpcs_write_vendor(xpcs, MDIO_MMD_PMAPMD,
				DW_VR_XS_PMA_GEN5_10G_GEN_CTRL, ret);
	if (ret < 0)
		return ret;

	/* Select PCS/PMA refclk source: Pad or Core. Vendor-reset the device
	 * to make sure the updates are perceived by the core
	 */
	return xpcs_gen5_10g_ref_clock_select(xpcs, true);
}

int xpcs_10gbaser_pma_config(struct dw_xpcs *xpcs)
{
	switch (xpcs->info.pma) {
	case DW_XPCS_PMA_GEN5_10G_ID:
		return xpcs_10gbaser_gen5_10g_pma_config(xpcs);
	default:
		return 0;
	}
}

static int xpcs_10gbasex_gen5_10g_pma_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_read_vendor(xpcs, MDIO_MMD_PMAPMD,
			       DW_VR_XS_PMA_GEN5_10G_GEN_CTRL);
	if (ret < 0)
		return ret;

	/* Select KX4-mode of the PHY lanes */
	ret &= ~DW_VR_XS_PMA_LANE_MODE;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LANE_MODE, DW_VR_XS_PMA_LANE_MODE_KX4);

	/* Activate 4 lane per link */
	ret &= ~DW_VR_XS_PMA_LINK_WIDTH;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LINK_WIDTH, DW_VR_XS_PMA_LINK_WIDTH_4);

	/* Enable all 4 lanes since it's X4 */
	ret &= ~DW_VR_XS_PMA_LANE_PWR_OFF;
	ret |= FIELD_PREP(DW_VR_XS_PMA_LANE_PWR_OFF, 0x0);

	ret = xpcs_write_vendor(xpcs, MDIO_MMD_PMAPMD,
				DW_VR_XS_PMA_GEN5_10G_GEN_CTRL, ret);
	if (ret < 0)
		return ret;

	/* Select PCS/PMA refclk source: Pad or Core. Vendor-reset the device
	 * to make sure the updates are perceived by the core
	 */
	return xpcs_gen5_10g_ref_clock_select(xpcs, true);
}

int xpcs_10gbasex_pma_config(struct dw_xpcs *xpcs)
{
	switch (xpcs->info.pma) {
	case DW_XPCS_PMA_GEN5_10G_ID:
		return xpcs_10gbasex_gen5_10g_pma_config(xpcs);
	default:
		return 0;
	}
}
