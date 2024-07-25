// SPDX-License-Identifier: GPL-2.0
/*
 * BM1000 PCIe Controller & Phy Gen3 equalization parameters fine tune.
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/property.h>
#include <linux/debugfs.h>

#include "pcie-designware.h"
#include "pcie-baikal.h"

static int gen3_eq_fb_mode = -1;
module_param(gen3_eq_fb_mode, int, 0444);
MODULE_PARM_DESC(gen3_eq_fb_mode, "feedback mode (0 - FOM + DIR, 1 - FOM only)");
static int gen3_eq_psets = -1;
module_param(gen3_eq_psets, int, 0444);
MODULE_PARM_DESC(gen3_eq_psets, "initial presets");
static int phy_rx_agc = -1;
module_param(phy_rx_agc, int, 0444);
MODULE_PARM_DESC(phy_rx_agc,
		 "Phy RX AGC gain ([7:4] - Pre-CTLE gain, [3:0] - Post-CTLE gain)");
static int phy_rx_ctle = -1;
module_param(phy_rx_ctle, int, 0444);
MODULE_PARM_DESC(phy_rx_ctle, "Phy RX CTLE control ([7:4] - zero, [3:0] - pole)");
static int phy_rx_dfe = -1;
module_param(phy_rx_dfe, int, 0444);
MODULE_PARM_DESC(phy_rx_dfe, "Phy RX DFE control (0 - disable, 1 - enable)");
static int phy_tx_gain = -1;
module_param(phy_tx_gain, int, 0444);
MODULE_PARM_DESC(phy_tx_gain, "Phy TX gain value");
static int phy_tx_turbo = -1;
module_param(phy_tx_turbo, int, 0444);
MODULE_PARM_DESC(phy_tx_turbo, "Phy TX turbo mode (0 - disable, 1 - enable)");
static int phy_rx_ctle_pole = -1;
module_param(phy_rx_ctle_pole, int, 0444);
MODULE_PARM_DESC(phy_rx_ctle_pole,
		 "Phy RX CTLE pole range for VMA adaptation ([7:4] - max, [3:0] - min)");
static bool debugfs;
module_param(debugfs, bool, 0444);
MODULE_PARM_DESC(debugfs, "Phy debugfs monitor enable");
static bool notune;
module_param(notune, bool, 0444);
MODULE_PARM_DESC(notune, "Gen3 equalization fine tune disable");

static void bm1000_pcie_tune_debugfs_populate(struct dw_pcie *pci);

/* Baikal-M PCIe PHY registers access */
#define BM1000_PCIE_AXI2MGM_LINENUM			0xd04
#define BM1000_PCIE_AXI2MGM_LINENUM_LANE_SEL_MASK	GENMASK(7, 0)
#define BM1000_PCIE_AXI2MGM_ADDRCTL			0xd08
#define BM1000_PCIE_AXI2MGM_ADDRCTL_BUSY		BIT(31)
#define BM1000_PCIE_AXI2MGM_ADDRCTL_DONE		BIT(30)
#define BM1000_PCIE_AXI2MGM_ADDRCTL_RW_FLAG		BIT(29)
#define BM1000_PCIE_AXI2MGM_ADDRCTL_PHY_ADDR_MASK	GENMASK(20, 0)
#define BM1000_PCIE_AXI2MGM_WRITEDATA			0xd0c
#define BM1000_PCIE_AXI2MGM_WRITEDATA_DATA_MASK		GENMASK(15, 0)
#define BM1000_PCIE_AXI2MGM_READDATA			0xd10
#define BM1000_PCIE_AXI2MGM_READDATA_DATA_MASK		GENMASK(15, 0)

#define BM1000_PCIE_PHY_REG_RETRIES		10
#define BM1000_PCIE_PHY_REG_RETRY_TIMEOUT	100

static int bm1000_pcie_phy_done(struct dw_pcie *pci)
{
	u32 reg;
	int retries;

	for (retries = 0; retries < BM1000_PCIE_PHY_REG_RETRIES; ++retries) {
		reg = dw_pcie_readl_dbi(pci, BM1000_PCIE_AXI2MGM_ADDRCTL);
		if (reg & BM1000_PCIE_AXI2MGM_ADDRCTL_DONE)
			return 0;
		udelay(BM1000_PCIE_PHY_REG_RETRY_TIMEOUT);
	}
	return -ETIMEDOUT;
}

static int bm1000_pcie_phy_read(struct dw_pcie *pci, u8 lane,
				u32 addr, u16 *value)
{
	int ret;

	bm1000_pcie_phy_enable(pci);
	dw_pcie_writel_dbi(pci, BM1000_PCIE_AXI2MGM_LINENUM, lane);
	dw_pcie_writel_dbi(pci, BM1000_PCIE_AXI2MGM_ADDRCTL,
			   addr & BM1000_PCIE_AXI2MGM_ADDRCTL_PHY_ADDR_MASK);
	ret = bm1000_pcie_phy_done(pci);
	if (ret == 0)
		*value = dw_pcie_readl_dbi(pci, BM1000_PCIE_AXI2MGM_READDATA) &
			 BM1000_PCIE_AXI2MGM_READDATA_DATA_MASK;
	bm1000_pcie_phy_disable(pci);
	return ret;
}

static int bm1000_pcie_phy_write(struct dw_pcie *pci, u8 lanes,
				 u32 addr, u16 value)
{
	int ret;

	bm1000_pcie_phy_enable(pci);
	dw_pcie_writel_dbi(pci, BM1000_PCIE_AXI2MGM_LINENUM, lanes);
	dw_pcie_writel_dbi(pci, BM1000_PCIE_AXI2MGM_WRITEDATA, value);
	dw_pcie_writel_dbi(pci, BM1000_PCIE_AXI2MGM_ADDRCTL,
			   (addr & BM1000_PCIE_AXI2MGM_ADDRCTL_PHY_ADDR_MASK) |
			   BM1000_PCIE_AXI2MGM_ADDRCTL_RW_FLAG);
	ret = bm1000_pcie_phy_done(pci);
	bm1000_pcie_phy_disable(pci);
	return ret;
}

/* Baikal-M PCIe RX/TX equalizers fine tune */
#define BM1000_PCIE_GEN3_EQ_CONTROL			0x8a8
#define BM1000_PCIE_GEN3_EQ_FOM_INC_INITIAL_EVAL	BIT(24)
#define BM1000_PCIE_GEN3_EQ_PSET_REQ_VEC_MASK		GENMASK(23, 8)
#define BM1000_PCIE_GEN3_EQ_FB_MODE_MASK		GENMASK(3, 0)

#define BM1000_PCIE_PHY_RX_CFG_2					0x18002
#define BM1000_PCIE_PHY_RX_CFG_2_PCS_SDS_RX_AGC_MVAL			GENMASK(9, 0)
#define BM1000_PCIE_PHY_RX_CFG_5					0x18005
#define BM1000_PCIE_PHY_RX_CFG_5_RX_AGC_MEN_OVRRD_EN			BIT(4)
#define BM1000_PCIE_PHY_RX_CFG_5_RX_AGC_MEN_OVRRD_VAL			BIT(3)
#define BM1000_PCIE_PHY_RX_LOOP_CTRL					0x18009
#define BM1000_PCIE_PHY_RX_LOOP_CTRL_CFG_RX_LCTRL_LCTRL_MEN		BIT(8)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL					0x1800b
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_ZERO_MASK		GENMASK(13, 10)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_RX_CTLE_POLE_OVRRD_EN		BIT(9)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_RX_CTLE_POLE_OVRRD_VAL_MASK	GENMASK(8, 5)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MAX_MASK	GENMASK(4, 3)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MIN_MASK	GENMASK(2, 1)
#define BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_STEP		BIT(0)
#define BM1000_PCIE_PHY_TX_CFG_1					0x18016
#define BM1000_PCIE_PHY_TX_CFG_1_TX_VBOOST_EN_OVRRD_EN			BIT(11)
#define BM1000_PCIE_PHY_TX_CFG_1_TX_TURBO_EN_OVRRD_EN			BIT(10)
#define BM1000_PCIE_PHY_TX_CFG_3					0x18018
#define BM1000_PCIE_PHY_TX_CFG_3_CFG_TX_VBOOST_EN			BIT(14)
#define BM1000_PCIE_PHY_TX_CFG_3_PCS_SDS_TX_GAIN_MASK			GENMASK(6, 4)
#define BM1000_PCIE_PHY_TX_CFG_3_CFG_TX_TURBO_EN			BIT(0)
#define BM1000_PCIE_PHY_RX_PWR_MON_1					0x1802a
#define BM1000_PCIE_PHY_RX_PWR_MON_1_RX_PWRSM_LANE_PWR_OFF		BIT(4)
#define BM1000_PCIE_PHY_TX_PWR_MON_0					0x1802c
#define BM1000_PCIE_PHY_TX_PWR_MON_0_TX_PWRSM_LANE_PWR_OFF		BIT(15)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_0					0x18048
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_1					0x18049
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2					0x1804a
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_OVRD_EN			BIT(5)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C5_MEN_OVRD_VAL		BIT(4)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C4_MEN_OVRD_VAL		BIT(3)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C3_MEN_OVRD_VAL		BIT(2)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C2_MEN_OVRD_VAL		BIT(1)
#define BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C1_MEN_OVRD_VAL		BIT(0)

void bm1000_pcie_tune(struct dw_pcie *pci)
{
	struct device *dev = pci->dev;
	u16 exp_cap_off;
	u8 lane = 0, lanes = 0;
	int override;
	int i, n_lanes;
	int ret;

	if (notune)
		return;

	/* Search for active lanes */
	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	n_lanes = FIELD_GET(PCI_EXP_LNKCAP_MLW,
			    dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKCAP));
	for (i = 0; i < n_lanes; i++) {
		u16 reg;
		u8 mask = 1 << i;

		ret = bm1000_pcie_phy_read(pci, mask,
					   BM1000_PCIE_PHY_RX_PWR_MON_1, &reg);
		if (ret != 0 ||
		    FIELD_GET(BM1000_PCIE_PHY_RX_PWR_MON_1_RX_PWRSM_LANE_PWR_OFF,
			      reg) == 1)
			continue;
		ret = bm1000_pcie_phy_read(pci, mask,
					   BM1000_PCIE_PHY_TX_PWR_MON_0, &reg);
		if (ret != 0 ||
		    FIELD_GET(BM1000_PCIE_PHY_TX_PWR_MON_0_TX_PWRSM_LANE_PWR_OFF,
			      reg) == 1)
			continue;
		lanes |= mask;
		if (lane == 0)
			lane = mask;
	}

	/* Feedback mode */
	override = gen3_eq_fb_mode;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,gen3-eq-fb-mode", &override);
	if (override >= 0) {
		u32 reg;

		dev_dbg(dev, "Gen3 fb_mode = %d\n", override);
		reg = dw_pcie_readl_dbi(pci, BM1000_PCIE_GEN3_EQ_CONTROL);
		reg &= ~BM1000_PCIE_GEN3_EQ_FB_MODE_MASK;
		reg |= FIELD_PREP(BM1000_PCIE_GEN3_EQ_FB_MODE_MASK, override);
		dw_pcie_writel_dbi(pci, BM1000_PCIE_GEN3_EQ_CONTROL, reg);
	}
	/* Initial presets */
	override = gen3_eq_psets;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,gen3-eq-psets", &override);
	if (override >= 0) {
		u32 reg;

		reg = dw_pcie_readl_dbi(pci, BM1000_PCIE_GEN3_EQ_CONTROL);
		dev_dbg(dev, "Gen3 initial presets = 0x%x\n", override);
		reg &= ~(BM1000_PCIE_GEN3_EQ_PSET_REQ_VEC_MASK |
			 BM1000_PCIE_GEN3_EQ_FOM_INC_INITIAL_EVAL);
		reg |= FIELD_PREP(BM1000_PCIE_GEN3_EQ_PSET_REQ_VEC_MASK,
				  override);
		dw_pcie_writel_dbi(pci, BM1000_PCIE_GEN3_EQ_CONTROL, reg);
	}
	/* Phy RX AGC */
	override = phy_rx_agc;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-rx-agc", &override);
	if (override >= 0) {
		u16 reg;

		ret = bm1000_pcie_phy_read(pci, lane, BM1000_PCIE_PHY_RX_CFG_2,
					   &reg);
		if (ret == 0) {
			reg &= ~BM1000_PCIE_PHY_RX_CFG_2_PCS_SDS_RX_AGC_MVAL;
			reg |= FIELD_PREP(BM1000_PCIE_PHY_RX_CFG_2_PCS_SDS_RX_AGC_MVAL,
					  override);
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_RX_CFG_2,
						    reg);
		}
		if (ret == 0)
			ret = bm1000_pcie_phy_read(pci, lane,
						   BM1000_PCIE_PHY_RX_CFG_5,
						   &reg);
		if (ret == 0) {
			reg |= BM1000_PCIE_PHY_RX_CFG_5_RX_AGC_MEN_OVRRD_EN |
			       BM1000_PCIE_PHY_RX_CFG_5_RX_AGC_MEN_OVRRD_VAL;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_RX_CFG_5,
						    reg);
		}
		dev_dbg(dev, "Phy RX AGC = 0x%04x (%d)\n", override, ret);
	}
	/* Rhy RX CTLE */
	override = phy_rx_ctle;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-rx-ctle", &override);
	if (override >= 0) {
		u16 reg;

		ret = bm1000_pcie_phy_read(pci, lane,
					   BM1000_PCIE_PHY_RX_CTLE_CTRL, &reg);
		if (ret == 0) {
			reg &= ~(BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_ZERO_MASK |
				 BM1000_PCIE_PHY_RX_CTLE_CTRL_RX_CTLE_POLE_OVRRD_VAL_MASK);
			reg |= BM1000_PCIE_PHY_RX_CTLE_CTRL_RX_CTLE_POLE_OVRRD_EN |
			       FIELD_PREP(BM1000_PCIE_PHY_RX_CTLE_CTRL_RX_CTLE_POLE_OVRRD_VAL_MASK,
					  override & 0xf) |
			       FIELD_PREP(BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_ZERO_MASK,
					  override >> 4);
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_RX_CTLE_CTRL,
						    reg);
		}
		if (ret == 0)
			ret = bm1000_pcie_phy_read(pci, lane,
						   BM1000_PCIE_PHY_RX_LOOP_CTRL,
						   &reg);
		if (ret == 0) {
			reg |= BM1000_PCIE_PHY_RX_LOOP_CTRL_CFG_RX_LCTRL_LCTRL_MEN;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_RX_LOOP_CTRL,
						    reg);
		}
		dev_dbg(dev, "Phy RX CTLE = 0x%04x (%d)\n", override, ret);
	}
	/* Phy RX DFE */
	override = phy_rx_dfe;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-rx-dfe", &override);
	if (override == 0) { /* enabled by default - disable only */
		u16 reg;

		reg = BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_OVRD_EN;
		reg |= BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C1_MEN_OVRD_VAL |
		       BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C2_MEN_OVRD_VAL |
		       BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C3_MEN_OVRD_VAL |
		       BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C4_MEN_OVRD_VAL |
		       BM1000_PCIE_PHY_RX_AEQ_VALBBD_2_RX_DFE_C5_MEN_OVRD_VAL;
		ret = bm1000_pcie_phy_write(pci, lanes,
					    BM1000_PCIE_PHY_RX_AEQ_VALBBD_2,
					    reg);
		if (ret == 0)
			ret = bm1000_pcie_phy_read(pci, lane,
						   BM1000_PCIE_PHY_RX_LOOP_CTRL,
						   &reg);
		if (ret == 0) {
			reg |= BM1000_PCIE_PHY_RX_LOOP_CTRL_CFG_RX_LCTRL_LCTRL_MEN;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_RX_LOOP_CTRL,
						    reg);
		}
		if (ret == 0) {
			reg = 0;
			bm1000_pcie_phy_write(pci, lanes,
					      BM1000_PCIE_PHY_RX_AEQ_VALBBD_0,
					      reg);
			bm1000_pcie_phy_write(pci, lanes,
					      BM1000_PCIE_PHY_RX_AEQ_VALBBD_1,
					      reg);
		}
		dev_dbg(dev, "Phy RX DFE = %d (%d)\n", override, ret);
	}
	/* Phy TX gain */
	override = phy_tx_gain;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-tx-gain", &override);
	if (override >= 0) {
		u16 reg;

		ret = bm1000_pcie_phy_read(pci, lane, BM1000_PCIE_PHY_TX_CFG_3,
					   &reg);
		if (ret == 0) {
			reg &= ~BM1000_PCIE_PHY_TX_CFG_3_PCS_SDS_TX_GAIN_MASK;
			reg |= BM1000_PCIE_PHY_TX_CFG_3_CFG_TX_VBOOST_EN;
			reg |= FIELD_PREP(BM1000_PCIE_PHY_TX_CFG_3_PCS_SDS_TX_GAIN_MASK,
					  override);
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_TX_CFG_3,
						    reg);
		}
		if (ret == 0)
			ret = bm1000_pcie_phy_read(pci, lane,
						   BM1000_PCIE_PHY_TX_CFG_1,
						   &reg);
		if (ret == 0) {
			reg |= BM1000_PCIE_PHY_TX_CFG_1_TX_VBOOST_EN_OVRRD_EN;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_TX_CFG_1,
						    reg);
		}
		dev_dbg(dev, "Phy TX gain = 0x%x (%d)\n", override, ret);
	}
	/* Phy TX turbo */
	override = phy_tx_turbo;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-tx-turbo", &override);
	if (override >= 0) {
		u16 reg;

		ret = bm1000_pcie_phy_read(pci, lane, BM1000_PCIE_PHY_TX_CFG_3,
					   &reg);
		if (ret == 0) {
			if (override == 0)
				reg &= ~BM1000_PCIE_PHY_TX_CFG_3_CFG_TX_TURBO_EN;
			else
				reg |= BM1000_PCIE_PHY_TX_CFG_3_CFG_TX_TURBO_EN;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_TX_CFG_3,
						    reg);
		}
		if (ret == 0)
			ret = bm1000_pcie_phy_read(pci, lane,
						   BM1000_PCIE_PHY_TX_CFG_1,
						   &reg);
		if (ret == 0) {
			reg |= BM1000_PCIE_PHY_TX_CFG_1_TX_TURBO_EN_OVRRD_EN;
			ret = bm1000_pcie_phy_write(pci, lanes,
						    BM1000_PCIE_PHY_TX_CFG_1,
						    reg);
		}
		dev_dbg(dev, "Phy TX turbo = %d (%d)\n", override, ret);
	}
	/* Phy RX CTLE pole range */
	override = phy_rx_ctle_pole;
	if (override == -1)
		device_property_read_u32(dev, "bm1000,phy-rx-ctle-pole", &override);
	if (override >= 0) {
		u16 reg;
		u8 pole_max = (override >> 4) & 0xf, pole_min = override & 0xf;

		ret = bm1000_pcie_phy_read(pci, lane, BM1000_PCIE_PHY_RX_CTLE_CTRL,
					   &reg);
		if (ret == 0) {
			reg &= ~(BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MAX_MASK |
				 BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MIN_MASK);
			reg |= FIELD_PREP(BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MAX_MASK,
					  pole_max);
			reg |= FIELD_PREP(BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_MIN_MASK,
					  pole_min);
			if (pole_max == pole_min)
				reg &= ~BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_STEP;
			else
				reg |= BM1000_PCIE_PHY_RX_CTLE_CTRL_PCS_SDS_RX_CTLE_POLE_STEP;
			ret = bm1000_pcie_phy_write(pci, lanes, BM1000_PCIE_PHY_RX_CTLE_CTRL,
						    reg);
		}
		dev_dbg(dev, "Phy RX CTLE pole = 0x%04x (%d)\n", override, ret);
	}

	/* debugfs populate */
	if (debugfs)
		bm1000_pcie_tune_debugfs_populate(pci);
}

#ifdef CONFIG_DEBUG_FS

#define BM1000_PCIE_PHY_SDS_PIN_MON_1				0x18027
#define BM1000_PCIE_PHY_SDS_PIN_MON_1_PCS_SDS_TX_SWING		GENMASK(5, 1)
#define BM1000_PCIE_PHY_SDS_PIN_MON_2				0x18028
#define BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_VBOOST_EN	BIT(10)
#define BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_TURBO_EN	BIT(9)
#define BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_POST_CURSOR	GENMASK(8, 4)
#define BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_PRE_CURSOR	GENMASK(3, 0)
#define BM1000_PCIE_PHY_RX_PWR_MON_0				0x18029
#define BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_AGC_EN		BIT(10)
#define BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_DFE_EN		BIT(9)
#define BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_CDR_EN		BIT(8)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_0				0x18050
#define BM1000_PCIE_PHY_RX_AEQ_OUT_0_DFE_TAP5			GENMASK(9, 5)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_0_DFE_TAP4			GENMASK(4, 0)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_1				0x18051
#define BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP3			GENMASK(14, 10)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP2			GENMASK(9, 5)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP1			GENMASK(4, 0)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_2				0x18052
#define BM1000_PCIE_PHY_RX_AEQ_OUT_2_PRE_CTLE_GAIN		GENMASK(7, 4)
#define BM1000_PCIE_PHY_RX_AEQ_OUT_2_POST_CTLE_GAIN		GENMASK(3, 0)
#define BM1000_PCIE_PHY_RX_VMA_STATUS_0				0x18057
#define BM1000_PCIE_PHY_RX_VMA_STATUS_0_CTLE_PEAK		GENMASK(5, 2)
#define BM1000_PCIE_PHY_RX_VMA_STATUS_0_CTLE_POLE		GENMASK(1, 0)

static void print_for_all_lanes(struct dw_pcie *pci, struct seq_file *s,
				u8 n_lanes, u32 addr, u16 mask)
{
	int i;

	for (i = 0; i < n_lanes; i++) {
		u16 reg;
		u8 lane = 1 << i;

		if (bm1000_pcie_phy_read(pci, lane, addr, &reg) == 0)
			seq_put_hex_ll(s, " ", (reg & mask) >> __bf_shf(mask), 0);
		else
			seq_puts(s, " ?");
	}
}

static int bm1000_pcie_dbgfs_phy_mon_show(struct seq_file *s, void *data)
{
	struct dw_pcie *pci = s->private;
	u8 n_lanes;
	u16 exp_cap_off;

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	n_lanes = FIELD_GET(PCI_EXP_LNKCAP_MLW,
			    dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKCAP));

	seq_puts(s, "sds_pin_mon:\n");
	seq_puts(s, " tx_swing:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_SDS_PIN_MON_1,
			    BM1000_PCIE_PHY_SDS_PIN_MON_1_PCS_SDS_TX_SWING);
	seq_puts(s, "\n");
	seq_puts(s, " pre_cursor:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_SDS_PIN_MON_2,
			    BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_PRE_CURSOR);
	seq_puts(s, "\n");
	seq_puts(s, " post_cursor:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_SDS_PIN_MON_2,
			    BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_POST_CURSOR);
	seq_puts(s, "\n");
	seq_puts(s, " vboost_en:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_SDS_PIN_MON_2,
			    BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_VBOOST_EN);
	seq_puts(s, "\n");
	seq_puts(s, " turbo_en:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_SDS_PIN_MON_2,
			    BM1000_PCIE_PHY_SDS_PIN_MON_2_PCS_SDS_TX_TURBO_EN);
	seq_puts(s, "\n");
	seq_puts(s, "rx_vma_status:\n");
	seq_puts(s, " ctle_peak:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_VMA_STATUS_0,
			    BM1000_PCIE_PHY_RX_VMA_STATUS_0_CTLE_PEAK);
	seq_puts(s, "\n");
	seq_puts(s, " ctle_pole:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_VMA_STATUS_0,
			    BM1000_PCIE_PHY_RX_VMA_STATUS_0_CTLE_POLE);
	seq_puts(s, "\n");
	seq_puts(s, "rx_aeq_out:\n");
	seq_puts(s, " pre_ctle_gain:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_2,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_2_PRE_CTLE_GAIN);
	seq_puts(s, "\n");
	seq_puts(s, " post_ctle_gain:");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_2,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_2_POST_CTLE_GAIN);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_tap1:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_1,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP1);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_tap2:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_1,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP2);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_tap3:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_1,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_1_DFE_TAP3);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_tap4:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_0,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_0_DFE_TAP4);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_tap5:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_AEQ_OUT_0,
			    BM1000_PCIE_PHY_RX_AEQ_OUT_0_DFE_TAP5);
	seq_puts(s, "\n");
	seq_puts(s, "pwr_mon:\n");
	seq_puts(s, " tx_pwr_off:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_TX_PWR_MON_0,
			    BM1000_PCIE_PHY_TX_PWR_MON_0_TX_PWRSM_LANE_PWR_OFF);
	seq_puts(s, "\n");
	seq_puts(s, " rx_pwr_off:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_PWR_MON_1,
			    BM1000_PCIE_PHY_RX_PWR_MON_1_RX_PWRSM_LANE_PWR_OFF);
	seq_puts(s, "\n");
	seq_puts(s, " agc_en:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_PWR_MON_0,
			    BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_AGC_EN);
	seq_puts(s, "\n");
	seq_puts(s, " dfe_en:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_PWR_MON_0,
			    BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_DFE_EN);
	seq_puts(s, "\n");
	seq_puts(s, " cdr_en:\t");
	print_for_all_lanes(pci, s, n_lanes, BM1000_PCIE_PHY_RX_PWR_MON_0,
			    BM1000_PCIE_PHY_RX_PWR_MON_0_RX_PWRSM_CDR_EN);
	seq_puts(s, "\n");
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(bm1000_pcie_dbgfs_phy_mon);

static void bm1000_pcie_tune_debugfs_populate(struct dw_pcie *pci)
{
	struct dentry *root_dir;

	root_dir = debugfs_create_dir(dev_name(pci->dev), NULL);
	if (!root_dir) {
		dev_warn(pci->dev, "%s: failed to create debugfs dir\n",
			 __func__);
		return;
	}
	if (!debugfs_create_file("phy_mon", 0444, root_dir, pci,
				 &bm1000_pcie_dbgfs_phy_mon_fops))
		dev_warn(pci->dev, "%s: failed to create phy_mon debugfs file\n",
			 __func__);
}
#else
static void bm1000_pcie_tune_debugfs_populate(struct dw_pcie *pci)
{
}
#endif /* CONFIG_DEBUG_FS */
