// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This is the driver for the GMAC on-chip Ethernet controller for ST SoCs.
  DWC Ether MAC 10/100/1000 Universal version 3.41a  has been used for
  developing this code.

  This only implements the mac core functions for this chip.

  Copyright (C) 2007-2009  STMicroelectronics Ltd


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/crc32.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/phylink.h>
#include "stmmac.h"
#include "stmmac_pcs.h"
#include "dwmac1000.h"
#include "dwmac_dma.h"

static void dwmac1000_core_init(struct mac_device_info *hw,
				struct net_device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_CONTROL);
	int mtu = dev->mtu;

	/* Configure GMAC core */
	value |= GMAC_CORE_INIT;

	/* Frame length limits (giant status(+VLAN):truncated by watchdog) */
	if (mtu > 9000) /* Rx <= 9018(9022):16383 && Tx <= 16383 */
		value |= GMAC_CONTROL_JE | GMAC_CONTROL_WD | GMAC_CONTROL_JD;
	else if (mtu > 1978) /* Rx <= 9018(9022):10240 && Tx <= 10240 */
		value |= GMAC_CONTROL_JE;
	else if (priv->synopsys_id < DWMAC_CORE_3_70 && mtu > 1500)
		value |= GMAC_CONTROL_JE;
	else if (mtu > 1500) /* Rx <= 1996(2000):2048 && Tx <= 2048 */
		value |= GMAC_CONTROL_2K;
	/* else 64 <= Rx <= 1518(1522):2048 && Tx <= 2048 */

	if (hw->ps) {
		value |= GMAC_CONTROL_TE;

		value &= ~hw->link.speed_mask;
		switch (hw->ps) {
		case SPEED_1000:
			value |= hw->link.speed1000;
			break;
		case SPEED_100:
			value |= hw->link.speed100;
			break;
		case SPEED_10:
			value |= hw->link.speed10;
			break;
		}
	}

	writel(value, ioaddr + GMAC_CONTROL);

	/* Over 2K, 3K, ..., 10K, 16K-1 frames will be truncated on Rx */
	if (mtu > 1500 && mtu <= 9000) {
		value = GMAC_WDT_PWE |
			ALIGN(mtu + ETH_HLEN + VLAN_HLEN + ETH_FCS_LEN, SZ_1K);
	} else {
		value = 0;
	}

	/* Over giant frame watchdog fine-tuning (available since v3.70a) */
	writel(value, ioaddr + GMAC_WDT);

	/* Mask GMAC interrupts */
	writel(GMAC_INT_DEFAULT_MASK, ioaddr + GMAC_INT_MASK);

#ifdef STMMAC_VLAN_TAG_USED
	/* Tag detection without filtering */
	writel(0x0, ioaddr + GMAC_VLAN_TAG);
#endif
}

static void dwmac1000_rx_fcs_enable(struct mac_device_info *hw, bool enable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + GMAC_CONTROL);

	if (enable)
		value &= ~GMAC_CONTROL_CST;
	else
		value |= GMAC_CONTROL_CST;

	writel(value, ioaddr + GMAC_CONTROL);
}

static int dwmac1000_rx_fcs_status(struct mac_device_info *hw, int snps_id,
				   int status)
{
	/* CRC Stripping for Type Frames has been supported since v3.50a */
	if (snps_id < DWMAC_CORE_3_50 || unlikely(status & llc_snap))
		return -ENOTSUPP;

	return 0;
}

static int dwmac1000_rx_ipc_enable(struct mac_device_info *hw, bool enable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_CONTROL);

	if (enable)
		value |= GMAC_CONTROL_IPC;
	else
		value &= ~GMAC_CONTROL_IPC;

	writel(value, ioaddr + GMAC_CONTROL);

	value = readl(ioaddr + GMAC_CONTROL);

	return !!(value & GMAC_CONTROL_IPC);
}

static void dwmac1000_dump_regs(struct mac_device_info *hw, u32 *reg_space)
{
	void __iomem *ioaddr = hw->pcsr;
	int i;

	for (i = 0; i < 55; i++)
		reg_space[i] = readl(ioaddr + i * 4);
}

static void dwmac1000_set_umac_addr(struct mac_device_info *hw,
				    const unsigned char *addr,
				    unsigned int reg_n)
{
	void __iomem *ioaddr = hw->pcsr;
	stmmac_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
			    GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_get_umac_addr(struct mac_device_info *hw,
				    unsigned char *addr,
				    unsigned int reg_n)
{
	void __iomem *ioaddr = hw->pcsr;
	stmmac_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
			    GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_set_mchash(void __iomem *ioaddr, u32 *mcfilterbits,
				 int mcbitslog2)
{
	int numhashregs, regs;

	switch (mcbitslog2) {
	case 6:
		writel(mcfilterbits[0], ioaddr + GMAC_HASH_LOW);
		writel(mcfilterbits[1], ioaddr + GMAC_HASH_HIGH);
		return;
	case 7:
		numhashregs = 4;
		break;
	case 8:
		numhashregs = 8;
		break;
	default:
		pr_debug("STMMAC: err in setting multicast filter\n");
		return;
	}
	for (regs = 0; regs < numhashregs; regs++)
		writel(mcfilterbits[regs],
		       ioaddr + GMAC_EXTHASH_BASE + regs * 4);
}

static int dwmac1000_set_vlan_tag(struct mac_device_info *hw, u32 queue, u16 vid)
{
	void __iomem *ioaddr = hw->pcsr;
	bool ready;
	u32 value;
	int ret;

	/* Wait for any transfer being finished so not to corrupt the pending
	 * outbound frames.
	 */
	ret = read_poll_timeout_atomic(dwmac_dma_suspended, ready, ready,
				       100, 50000, false,
				       NULL, ioaddr, queue, DMA_DIR_TX);
	if (ret)
		return ret;

	value = readl(ioaddr + GMAC_VLAN_INCL) & ~GMAC_VLAN_VLT;
	value |= vid;
	writel(value, ioaddr + GMAC_VLAN_INCL);

	return 0;
}

static void dwmac1000_update_vlan_hash(struct mac_device_info *hw, u32 hash,
				       int add_ctags, int add_stags)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	hw->vlan_hash = hash;
	writel(hash, ioaddr + GMAC_VLAN_HASH_TABLE);

	value = readl(ioaddr + GMAC_VLAN_TAG) | GMAC_VLAN_ETV;

	if (hash)
		value |= GMAC_VLAN_VTHM;
	else
		value &= ~GMAC_VLAN_VTHM;

	/* Note DW GMAC VLAN filter doesn't have a filter-specific flag to
	 * toggle the S-Tags support. Instead if ESVL flag is set then both
	 * tag types will be considered as valid. But since the bit also
	 * affects the MAC transmitter it is handled in set_hw_vlan_mode().
	 */

	writel(value, ioaddr + GMAC_VLAN_TAG);
}

static void dwmac1000_write_single_vlan(struct net_device *dev, u32 data)
{
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	u32 val;

	if (!(data & GMAC_VLAN_VID))
		data |= GMAC_VLAN_VID;

	val = readl(ioaddr + GMAC_VLAN_TAG) & ~GMAC_VLAN_VID;
	val |= GMAC_VLAN_ETV | (data & GMAC_VLAN_VID);

	writel(val, ioaddr + GMAC_VLAN_TAG);
}

static int dwmac1000_add_hw_vlan_rx_fltr(struct net_device *dev,
					 struct mac_device_info *hw,
					 __be16 proto, u16 vid)
{
	bool is_stag;
	u32 val;

	if (vid > 4095)
		return -EINVAL;

	is_stag = proto == htons(ETH_P_8021AD);

	/* For single VLAN filter, VID 0 means VLAN promiscuous */
	if (vid == 0)
		return -EINVAL;

	/* Note ESVL is utilized to virtually distinguish the C- and S-Tags
	 * only. The bit is set/cleared in the set_hw_vlan_mode() depending
	 * on the HW-offloaded features requested.
	 */
	val = vid;
	if (is_stag)
		val |= GMAC_VLAN_ESVL;

	if (hw->vlan_filter[0] == val)
		return 0;

	if (hw->vlan_filter[0] & GMAC_VLAN_VID)
		return -ENOSPC;

	dwmac1000_write_single_vlan(dev, val);

	hw->vlan_filter[0] = val;

	return 0;
}

static int dwmac1000_del_hw_vlan_rx_fltr(struct net_device *dev,
					 struct mac_device_info *hw,
					 __be16 proto, u16 vid)
{
	bool is_stag;

	is_stag = proto == htons(ETH_P_8021AD);

	/* Single Rx VLAN Filter */
	if ((hw->vlan_filter[0] & GMAC_VLAN_VID) != vid)
		return -ENOENT;

	if (is_stag != !!(hw->vlan_filter[0] & GMAC_VLAN_ESVL))
		return -ENOENT;

	dwmac1000_write_single_vlan(dev, 0);

	hw->vlan_filter[0] = 0;

	return 0;
}

static void dwmac1000_restore_hw_vlan_rx_fltr(struct net_device *dev,
					      struct mac_device_info *hw)
{
	/* Hash-based Rx VLAN Filter */
	dwmac1000_update_vlan_hash(hw, hw->vlan_hash, 0, 0);

	/* Single Rx VLAN Filter */
	dwmac1000_write_single_vlan(dev, hw->vlan_filter[0]);
}

static void dwmac1000_set_filter(struct mac_device_info *hw,
				 struct net_device *dev)
{
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	unsigned int value = 0;
	u32 mc_filter[8];
	int mcbitslog2 = hw->mcast_bits_log2;

	pr_debug("%s: # mcasts %d, # unicast %d\n", __func__,
		 netdev_mc_count(dev), netdev_uc_count(dev));

	value = readl(ioaddr + GMAC_FRAME_FILTER);
	value &= ~GMAC_FRAME_FILTER_RA;
	value &= ~GMAC_FRAME_FILTER_HPF;
	value &= ~GMAC_FRAME_FILTER_PCF;
	value &= ~GMAC_FRAME_FILTER_PM;
	value &= ~GMAC_FRAME_FILTER_HMC;
	value &= ~GMAC_FRAME_FILTER_PR;

	memset(mc_filter, 0, sizeof(mc_filter));

	if (dev->flags & IFF_PROMISC) {
		value |= GMAC_FRAME_FILTER_RA;
		value |= GMAC_FRAME_FILTER_PR;
		value |= GMAC_FRAME_FILTER_PCF;
	} else if (dev->flags & IFF_ALLMULTI) {
		value = GMAC_FRAME_FILTER_PM;	/* pass all multi */
	} else if (!netdev_mc_empty(dev) && (mcbitslog2 == 0)) {
		/* Fall back to all multicast if we've no filter */
		value |= GMAC_FRAME_FILTER_PM;
	} else if (!netdev_mc_empty(dev) && dev->flags & IFF_MULTICAST) {
		struct netdev_hw_addr *ha;

		/* Hash filter for multicast */
		value |= GMAC_FRAME_FILTER_HMC;

		netdev_for_each_mc_addr(ha, dev) {
			/* The upper n bits of the calculated CRC are used to
			 * index the contents of the hash table. The number of
			 * bits used depends on the hardware configuration
			 * selected at core configuration time.
			 */
			int bit_nr = bitrev32(~crc32_le(~0, ha->addr,
					      ETH_ALEN)) >>
					      (32 - mcbitslog2);
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register.
			 */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
	}

	value |= GMAC_FRAME_FILTER_HPF;
	dwmac1000_set_mchash(ioaddr, mc_filter, mcbitslog2);

	/* Handle multiple unicast addresses (perfect filtering) */
	if (netdev_uc_count(dev) > hw->unicast_filter_entries - 1)
		/* Switch to promiscuous mode if more than unicast
		 * addresses are requested than supported by hardware.
		 */
		value |= GMAC_FRAME_FILTER_PR;
	else {
		int reg = 1;
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, dev) {
			stmmac_set_mac_addr(ioaddr, ha->addr,
					    GMAC_ADDR_HIGH(reg),
					    GMAC_ADDR_LOW(reg));
			reg++;
		}

		for (; reg < hw->unicast_filter_entries; reg++) {
			writel(0, ioaddr + GMAC_ADDR_HIGH(reg));
			writel(0, ioaddr + GMAC_ADDR_LOW(reg));
		}
	}

#ifdef FRAME_FILTER_DEBUG
	/* Enable Receive all mode (to debug filtering_fail errors) */
	value |= GMAC_FRAME_FILTER_RA;
#endif

	/* VLAN filtering */
	dwmac1000_restore_hw_vlan_rx_fltr(dev, hw);

	writel(value, ioaddr + GMAC_FRAME_FILTER);
}

static void dwmac1000_flow_ctrl(struct mac_device_info *hw, unsigned int duplex,
				unsigned int fc, unsigned int pause_time,
				u32 tx_cnt)
{
	void __iomem *ioaddr = hw->pcsr;
	/* Set flow such that DZPQ in Mac Register 6 is 0,
	 * and unicast pause detect is enabled.
	 */
	unsigned int flow = GMAC_FLOW_CTRL_UP;

	pr_debug("GMAC Flow-Control:\n");
	if (fc & FLOW_RX) {
		pr_debug("\tReceive Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_RFE;
	}
	if (fc & FLOW_TX) {
		pr_debug("\tTransmit Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_TFE;
	}

	if (duplex) {
		pr_debug("\tduplex mode: PAUSE %d\n", pause_time);
		flow |= (pause_time << GMAC_FLOW_CTRL_PT_SHIFT);
	}

	writel(flow, ioaddr + GMAC_FLOW_CTRL);
}

static void dwmac1000_pmt(struct mac_device_info *hw, unsigned long mode)
{
	void __iomem *ioaddr = hw->pcsr;
	unsigned int pmt = 0;

	if (mode & WAKE_MAGIC) {
		pr_debug("GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	}
	if (mode & WAKE_UCAST) {
		pr_debug("GMAC: WOL on global unicast\n");
		pmt |= power_down | global_unicast | wake_up_frame_en;
	}

	writel(pmt, ioaddr + GMAC_PMT);
}

static int dwmac1000_irq_status(struct mac_device_info *hw,
				struct stmmac_extra_stats *x)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 intr_status = readl(ioaddr + GMAC_INT_STATUS);
	u32 intr_mask = readl(ioaddr + GMAC_INT_MASK);
	int ret = 0;

	/* Discard masked bits */
	intr_status &= ~intr_mask;

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & GMAC_INT_STATUS_MMCTIS))
		x->mmc_tx_irq_n++;
	if (unlikely(intr_status & GMAC_INT_STATUS_MMCRIS))
		x->mmc_rx_irq_n++;
	if (unlikely(intr_status & GMAC_INT_STATUS_MMCCSUM))
		x->mmc_rx_csum_offload_irq_n++;
	if (unlikely(intr_status & GMAC_INT_DISABLE_PMT)) {
		/* clear the PMT bits 5 and 6 by reading the PMT status reg */
		readl(ioaddr + GMAC_PMT);
		x->irq_receive_pmt_irq_n++;
	}

	/* MAC tx/rx EEE LPI entry/exit interrupts */
	if (intr_status & GMAC_INT_STATUS_LPIIS) {
		/* Clean LPI interrupt by reading the Reg 12 */
		ret = readl(ioaddr + LPI_CTRL_STATUS);

		if (ret & LPI_CTRL_STATUS_TLPIEN)
			x->irq_tx_path_in_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_TLPIEX)
			x->irq_tx_path_exit_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_RLPIEN)
			x->irq_rx_path_in_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_RLPIEX)
			x->irq_rx_path_exit_lpi_mode_n++;
	}

	dwmac_pcs_isr(&hw->mac_pcs, intr_status, x);

	if (intr_status & PCS_RGSMIIIS_IRQ) {
		/* TODO Dummy-read to clear the IRQ status */
		readl(ioaddr + GMAC_RGSMIIIS);
		phylink_pcs_change(&hw->mac_pcs.pcs, false);
		x->irq_rgmii_n++;
	}

	return ret;
}

static void dwmac1000_set_eee_mode(struct mac_device_info *hw,
				   bool en_tx_lpi_clockgating)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	/*TODO - en_tx_lpi_clockgating treatment */

	/* Enable the link status receive on RGMII, SGMII ore SMII
	 * receive path and instruct the transmit to enter in LPI
	 * state.
	 */
	value = readl(ioaddr + LPI_CTRL_STATUS);
	value |= LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA;
	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void dwmac1000_reset_eee_mode(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + LPI_CTRL_STATUS);
	value &= ~(LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA);
	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void dwmac1000_set_eee_pls(struct mac_device_info *hw, int link)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + LPI_CTRL_STATUS);

	if (link)
		value |= LPI_CTRL_STATUS_PLS;
	else
		value &= ~LPI_CTRL_STATUS_PLS;

	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void dwmac1000_set_eee_timer(struct mac_device_info *hw, int ls, int tw)
{
	void __iomem *ioaddr = hw->pcsr;
	int value = ((tw & 0xffff)) | ((ls & 0x7ff) << 16);

	/* Program the timers in the LPI timer control register:
	 * LS: minimum time (ms) for which the link
	 *  status from PHY should be ok before transmitting
	 *  the LPI pattern.
	 * TW: minimum time (us) for which the core waits
	 *  after it has stopped transmitting the LPI pattern.
	 */
	writel(value, ioaddr + LPI_TIMER_CTRL);
}

static int dwmac1000_mii_pcs_enable(struct phylink_pcs *pcs)
{
	struct stmmac_pcs *spcs = phylink_pcs_to_stmmac_pcs(pcs);
	void __iomem *ioaddr = spcs->priv->hw->pcsr;
	u32 intr_mask;

	intr_mask = readl(ioaddr + GMAC_INT_MASK);
	intr_mask &= ~GMAC_INT_DISABLE_PCS;
	writel(intr_mask, ioaddr + GMAC_INT_MASK);

	return 0;
}

static void dwmac1000_mii_pcs_disable(struct phylink_pcs *pcs)
{
	struct stmmac_pcs *spcs = phylink_pcs_to_stmmac_pcs(pcs);
	void __iomem *ioaddr = spcs->priv->hw->pcsr;
	u32 intr_mask;

	intr_mask = readl(ioaddr + GMAC_INT_MASK);
	intr_mask |= GMAC_INT_DISABLE_PCS;
	writel(intr_mask, ioaddr + GMAC_INT_MASK);
}

static void dwmac1000_mii_pcs_get_state(struct phylink_pcs *pcs,
					struct phylink_link_state *state)
{
	struct stmmac_pcs *spcs = phylink_pcs_to_stmmac_pcs(pcs);
	u32 status = readl(spcs->priv->ioaddr + GMAC_RGSMIIIS);

	dwmac_rs_decode_stat(state, FIELD_GET(GMAC_RGSMIIIS_RS_STAT, status));
}

static const struct phylink_pcs_ops dwmac1000_mii_pcs_ops = {
	.pcs_enable = dwmac1000_mii_pcs_enable,
	.pcs_disable = dwmac1000_mii_pcs_disable,
	.pcs_config = dwmac_pcs_config,
	.pcs_get_state = dwmac1000_mii_pcs_get_state,
};

static struct phylink_pcs *
dwmac1000_phylink_select_pcs(struct stmmac_priv *priv,
			     phy_interface_t interface)
{
	if (priv->hw->pcs & STMMAC_PCS_RGMII ||
	    priv->hw->pcs & STMMAC_PCS_SGMII)
		return &priv->hw->mac_pcs.pcs;

	return NULL;
}

static void dwmac1000_debug(struct stmmac_priv *priv, void __iomem *ioaddr,
			    struct stmmac_extra_stats *x,
			    u32 rx_queues, u32 tx_queues)
{
	u32 value = readl(ioaddr + GMAC_DEBUG);

	if (value & GMAC_DEBUG_TXSTSFSTS)
		x->mtl_tx_status_fifo_full++;
	if (value & GMAC_DEBUG_TXFSTS)
		x->mtl_tx_fifo_not_empty++;
	if (value & GMAC_DEBUG_TWCSTS)
		x->mmtl_fifo_ctrl++;
	if (value & GMAC_DEBUG_TRCSTS_MASK) {
		u32 trcsts = (value & GMAC_DEBUG_TRCSTS_MASK)
			     >> GMAC_DEBUG_TRCSTS_SHIFT;
		if (trcsts == GMAC_DEBUG_TRCSTS_WRITE)
			x->mtl_tx_fifo_read_ctrl_write++;
		else if (trcsts == GMAC_DEBUG_TRCSTS_TXW)
			x->mtl_tx_fifo_read_ctrl_wait++;
		else if (trcsts == GMAC_DEBUG_TRCSTS_READ)
			x->mtl_tx_fifo_read_ctrl_read++;
		else
			x->mtl_tx_fifo_read_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_TXPAUSED)
		x->mac_tx_in_pause++;
	if (value & GMAC_DEBUG_TFCSTS_MASK) {
		u32 tfcsts = (value & GMAC_DEBUG_TFCSTS_MASK)
			      >> GMAC_DEBUG_TFCSTS_SHIFT;

		if (tfcsts == GMAC_DEBUG_TFCSTS_XFER)
			x->mac_tx_frame_ctrl_xfer++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_GEN_PAUSE)
			x->mac_tx_frame_ctrl_pause++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_WAIT)
			x->mac_tx_frame_ctrl_wait++;
		else
			x->mac_tx_frame_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_TPESTS)
		x->mac_gmii_tx_proto_engine++;
	if (value & GMAC_DEBUG_RXFSTS_MASK) {
		u32 rxfsts = (value & GMAC_DEBUG_RXFSTS_MASK)
			     >> GMAC_DEBUG_RRCSTS_SHIFT;

		if (rxfsts == GMAC_DEBUG_RXFSTS_FULL)
			x->mtl_rx_fifo_fill_level_full++;
		else if (rxfsts == GMAC_DEBUG_RXFSTS_AT)
			x->mtl_rx_fifo_fill_above_thresh++;
		else if (rxfsts == GMAC_DEBUG_RXFSTS_BT)
			x->mtl_rx_fifo_fill_below_thresh++;
		else
			x->mtl_rx_fifo_fill_level_empty++;
	}
	if (value & GMAC_DEBUG_RRCSTS_MASK) {
		u32 rrcsts = (value & GMAC_DEBUG_RRCSTS_MASK) >>
			     GMAC_DEBUG_RRCSTS_SHIFT;

		if (rrcsts == GMAC_DEBUG_RRCSTS_FLUSH)
			x->mtl_rx_fifo_read_ctrl_flush++;
		else if (rrcsts == GMAC_DEBUG_RRCSTS_RSTAT)
			x->mtl_rx_fifo_read_ctrl_read_data++;
		else if (rrcsts == GMAC_DEBUG_RRCSTS_RDATA)
			x->mtl_rx_fifo_read_ctrl_status++;
		else
			x->mtl_rx_fifo_read_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_RWCSTS)
		x->mtl_rx_fifo_ctrl_active++;
	if (value & GMAC_DEBUG_RFCFCSTS_MASK)
		x->mac_rx_frame_ctrl_fifo = (value & GMAC_DEBUG_RFCFCSTS_MASK)
					    >> GMAC_DEBUG_RFCFCSTS_SHIFT;
	if (value & GMAC_DEBUG_RPESTS)
		x->mac_gmii_rx_proto_engine++;
}

static void dwmac1000_set_mac_loopback(void __iomem *ioaddr, bool enable)
{
	u32 value = readl(ioaddr + GMAC_CONTROL);

	if (enable)
		value |= GMAC_CONTROL_LM;
	else
		value &= ~GMAC_CONTROL_LM;

	writel(value, ioaddr + GMAC_CONTROL);
}

static void dwmac1000_set_hw_vlan_mode(struct mac_device_info *hw, bool rx_strip,
				       bool rx_ctag, bool rx_stag, bool tx_stag)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	/* Activate VLAN Tag Rx filters */
	value = readl(ioaddr + GMAC_FRAME_FILTER);
	if (rx_ctag || rx_stag)
		value |= GMAC_FRAME_FILTER_VTFE;
	else
		value &= ~GMAC_FRAME_FILTER_VTFE;
	writel(value, ioaddr + GMAC_FRAME_FILTER);

	/* Activate S-VLAN feature on MAC Tx and Rx */
	value = readl(ioaddr + GMAC_VLAN_TAG);
	if (rx_stag || tx_stag)
		value |= GMAC_VLAN_ESVL;
	else
		value &= ~GMAC_VLAN_ESVL;
	writel(value, ioaddr + GMAC_VLAN_TAG);

	/* Set Tx VLAN Insertion feature (might be unavailable) */
	value = readl(ioaddr + GMAC_VLAN_INCL);
	if (tx_stag)
		value |= GMAC_VLAN_CSVL;
	else
		value &= ~GMAC_VLAN_CSVL;
	writel(value, ioaddr + GMAC_VLAN_INCL);
}

const struct stmmac_ops dwmac1000_ops = {
	.core_init = dwmac1000_core_init,
	.phylink_select_pcs = dwmac1000_phylink_select_pcs,
	.set_mac = stmmac_set_mac,
	.rx_fcs = dwmac1000_rx_fcs_enable,
	.rx_fcs_status = dwmac1000_rx_fcs_status,
	.rx_ipc = dwmac1000_rx_ipc_enable,
	.dump_regs = dwmac1000_dump_regs,
	.host_irq_status = dwmac1000_irq_status,
	.set_filter = dwmac1000_set_filter,
	.flow_ctrl = dwmac1000_flow_ctrl,
	.pmt = dwmac1000_pmt,
	.set_umac_addr = dwmac1000_set_umac_addr,
	.get_umac_addr = dwmac1000_get_umac_addr,
	.set_eee_mode = dwmac1000_set_eee_mode,
	.reset_eee_mode = dwmac1000_reset_eee_mode,
	.set_eee_timer = dwmac1000_set_eee_timer,
	.set_eee_pls = dwmac1000_set_eee_pls,
	.debug = dwmac1000_debug,
	.pcs_ctrl_ane = dwmac_ctrl_ane,
	.set_mac_loopback = dwmac1000_set_mac_loopback,
	.set_vlan_tag = dwmac1000_set_vlan_tag,
	.update_vlan_hash = dwmac1000_update_vlan_hash,
	.set_hw_vlan_mode = dwmac1000_set_hw_vlan_mode,
	.add_hw_vlan_rx_fltr = dwmac1000_add_hw_vlan_rx_fltr,
	.del_hw_vlan_rx_fltr = dwmac1000_del_hw_vlan_rx_fltr,
};

static u32 dwmac1000_get_num_vlan(void __iomem *ioaddr)
{
	u32 val;

	val = readl(ioaddr + GMAC_VERSION);
	if ((val & GENMASK(7, 0)) >= DWMAC_CORE_3_70)
		return 1;

	return 0;
}

int dwmac1000_setup(struct stmmac_priv *priv)
{
	struct mac_device_info *mac = priv->hw;

	dev_info(priv->device, "\tDWMAC1000\n");

	priv->dev->priv_flags |= IFF_UNICAST_FLT;
	mac->pcsr = priv->ioaddr;
	mac->multicast_filter_bins = priv->plat->multicast_filter_bins;
	mac->unicast_filter_entries = priv->plat->unicast_filter_entries;
	mac->mcast_bits_log2 = 0;

	if (mac->multicast_filter_bins)
		mac->mcast_bits_log2 = ilog2(mac->multicast_filter_bins);

	mac->link.caps = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
			 MAC_10 | MAC_100 | MAC_1000;
	mac->link.duplex = GMAC_CONTROL_DM;
	mac->link.speed10 = GMAC_CONTROL_PS;
	mac->link.speed100 = GMAC_CONTROL_PS | GMAC_CONTROL_FES;
	mac->link.speed1000 = 0;
	mac->link.speed_mask = GMAC_CONTROL_PS | GMAC_CONTROL_FES;
	mac->mii.addr = GMAC_MII_ADDR;
	mac->mii.data = GMAC_MII_DATA;
	mac->mii.addr_shift = 11;
	mac->mii.addr_mask = 0x0000F800;
	mac->mii.reg_shift = 6;
	mac->mii.reg_mask = 0x000007C0;
	mac->mii.clk_csr_shift = 2;
	mac->mii.clk_csr_mask = GENMASK(5, 2);

	mac->mac_pcs.priv = priv;
	mac->mac_pcs.pcs_base = priv->ioaddr + GMAC_PCS_BASE;
	mac->mac_pcs.pcs.ops = &dwmac1000_mii_pcs_ops;
	mac->mac_pcs.pcs.neg_mode = true;

	mac->num_vlan = dwmac1000_get_num_vlan(priv->ioaddr);

	return 0;
}
