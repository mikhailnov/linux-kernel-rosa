// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This contains the functions to handle the enhanced descriptors.

  Copyright (C) 2007-2014  STMicroelectronics Ltd


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/stmmac.h>
#include "common.h"
#include "descs_com.h"

static int enh_desc_get_tx_status(struct stmmac_extra_stats *x,
				  struct dma_desc *p)
{
	unsigned int tdes0 = le32_to_cpu(p->des0);
	int ret = tx_done;

	/* Get tx owner first */
	if (unlikely(tdes0 & ETDES0_OWN))
		return tx_dma_own;

	/* Verify tx error by looking at the last segment. */
	if (likely(!(tdes0 & ETDES0_LAST_SEGMENT)))
		return tx_not_ls;

	if (unlikely(tdes0 & ETDES0_ERROR_SUMMARY)) {
		if (unlikely(tdes0 & ETDES0_JABBER_TIMEOUT))
			x->tx_jabber++;

		if (unlikely(tdes0 & ETDES0_FRAME_FLUSHED))
			x->tx_frame_flushed++;

		if (unlikely(tdes0 & ETDES0_LOSS_CARRIER)) {
			x->tx_losscarrier++;
		}
		if (unlikely(tdes0 & ETDES0_NO_CARRIER)) {
			x->tx_carrier++;
		}
		if (unlikely((tdes0 & ETDES0_LATE_COLLISION) ||
			     (tdes0 & ETDES0_EXCESSIVE_COLLISIONS)))
			x->tx_collision +=
				(tdes0 & ETDES0_COLLISION_COUNT_MASK) >> 3;

		if (unlikely(tdes0 & ETDES0_EXCESSIVE_DEFERRAL))
			x->tx_deferred++;

		if (unlikely(tdes0 & ETDES0_UNDERFLOW_ERROR))
			x->tx_underflow++;

		if (unlikely(tdes0 & ETDES0_IP_HEADER_ERROR))
			x->tx_ip_header_error++;

		if (unlikely(tdes0 & ETDES0_PAYLOAD_ERROR))
			x->tx_payload_error++;

		ret = tx_err;
	}

	if (unlikely(tdes0 & ETDES0_DEFERRED))
		x->tx_deferred++;

#ifdef STMMAC_VLAN_TAG_USED
	if (tdes0 & ETDES0_VLAN_FRAME)
		x->tx_vlan++;
#endif

	return ret;
}

static int enh_desc_rx_ext_status(unsigned int rdes4,
				  struct stmmac_extra_stats *x)
{
	int message_type = (rdes4 & ERDES4_MSG_TYPE_MASK) >> 8;
	int ret = good_frame;

	if (rdes4 & ERDES4_IP_HDR_ERR) {
		x->ip_hdr_err++;
		ret = csum_err;
	}

	if (rdes4 & ERDES4_IP_PAYLOAD_ERR) {
		x->ip_payload_err++;
		ret = csum_err;
	}

	if (rdes4 & ERDES4_IP_CSUM_BYPASSED) {
		x->ip_csum_bypassed++;
		ret = csum_none;
	}

	if (rdes4 & ERDES4_IPV4_PKT_RCVD)
		x->ipv4_pkt_rcvd++;
	else if (rdes4 & ERDES4_IPV6_PKT_RCVD)
		x->ipv6_pkt_rcvd++;
	else /* Not an IP packet or COE is disabled */
		ret = csum_none;

	switch (message_type) {
	case RDES_EXT_NO_PTP:
		x->no_ptp_rx_msg_type_ext++;
		break;
	case RDES_EXT_SYNC:
		x->ptp_rx_msg_type_sync++;
		break;
	case RDES_EXT_FOLLOW_UP:
		x->ptp_rx_msg_type_follow_up++;
		break;
	case RDES_EXT_DELAY_REQ:
		x->ptp_rx_msg_type_delay_req++;
		break;
	case RDES_EXT_DELAY_RESP:
		x->ptp_rx_msg_type_delay_resp++;
		break;
	case RDES_EXT_PDELAY_REQ:
		x->ptp_rx_msg_type_pdelay_req++;
		break;
	case RDES_EXT_PDELAY_RESP:
		x->ptp_rx_msg_type_pdelay_resp++;
		break;
	case RDES_EXT_PDELAY_FOLLOW_UP:
		x->ptp_rx_msg_type_pdelay_follow_up++;
		break;
	case RDES_PTP_ANNOUNCE:
		x->ptp_rx_msg_type_announce++;
		break;
	case RDES_PTP_MANAGEMENT:
		x->ptp_rx_msg_type_management++;
		break;
	case RDES_PTP_PKT_RESERVED_TYPE:
		x->ptp_rx_msg_pkt_reserved_type++;
		break;
	default:
		break;
	}

	if (rdes4 & ERDES4_PTP_FRAME_TYPE)
		x->ptp_frame_type++;
	if (rdes4 & ERDES4_PTP_VER)
		x->ptp_ver++;
	if (rdes4 & ERDES4_TIMESTAMP_DROPPED)
		x->timestamp_dropped++;
	if (rdes4 & ERDES4_AV_PKT_RCVD)
		x->av_pkt_rcvd++;
	if (rdes4 & ERDES4_AV_TAGGED_PKT_RCVD)
		x->av_tagged_pkt_rcvd++;
	if ((rdes4 & ERDES4_VLAN_TAG_PRI_VAL_MASK) >> 18)
		x->vlan_tag_priority_val++;
	if (rdes4 & ERDES4_L3_FILTER_MATCH)
		x->l3_filter_match++;
	if (rdes4 & ERDES4_L4_FILTER_MATCH)
		x->l4_filter_match++;
	if ((rdes4 & ERDES4_L3_L4_FILT_NO_MATCH_MASK) >> 26)
		x->l3_l4_filter_no_match++;

	return ret;
}

static int enh_desc_get_rx_basic_status(unsigned int rdes0,
					struct stmmac_extra_stats *x)
{
	int ret = good_frame;

	if (unlikely(rdes0 & RDES0_OWN))
		return dma_own;

	if (unlikely(!(rdes0 & RDES0_LAST_DESCRIPTOR)))
		return rx_not_ls;

	if (unlikely(!(rdes0 & RDES0_FRAME_TYPE)))
		ret = llc_snap;

	if (unlikely(rdes0 & RDES0_ERROR_SUMMARY)) {
		if (unlikely(rdes0 & RDES0_DESCRIPTOR_ERROR)) {
			x->rx_desc++;
			ret |= cutoff_err;
		}

		if (unlikely(rdes0 & RDES0_OVERFLOW_ERROR)) {
			x->rx_gmac_overflow++;
			ret |= cutoff_err;
		}

		if (unlikely(rdes0 & RDES0_GIANT_FRAME_ERROR)) {
			x->rx_length++;
			ret |= len_err;
		}

		if (unlikely(rdes0 & RDES0_COLLISION)) {
			x->rx_collision++;
			ret |= proto_err;
		}

		if (unlikely(rdes0 & RDES0_RECEIVE_WATCHDOG)) {
			x->rx_watchdog++;
			ret |= cutoff_err;
		}

		if (unlikely(rdes0 & RDES0_MII_ERROR)) {
			x->rx_mii++;
			ret |= proto_err;
		}

		if (unlikely(rdes0 & RDES0_CRC_ERROR)) {
			x->rx_crc_errors++;
			ret |= csum_err;
		}
	}

	if (unlikely(rdes0 & RDES0_DRIBBLING))
		x->dribbling_bit++;

	if (unlikely(rdes0 & RDES0_SA_FILTER_FAIL))
		x->sa_rx_filter_fail++;

	if (unlikely(rdes0 & RDES0_DA_FILTER_FAIL))
		x->da_rx_filter_fail++;

	if (unlikely(rdes0 & RDES0_LENGTH_ERROR))
		x->rx_length++;

#ifdef STMMAC_VLAN_TAG_USED
	if (rdes0 & RDES0_VLAN_TAG)
		x->rx_vlan++;
#endif

	return ret;
}

static int enh_desc_get_rx_status_nocoe(struct stmmac_extra_stats *x,
					struct dma_desc *p)
{
	return enh_desc_get_rx_basic_status(le32_to_cpu(p->des0), x);
}

static int enh_desc_get_rx_status_noext(struct stmmac_extra_stats *x,
					struct dma_desc *p)
{
	unsigned int rdes0 = le32_to_cpu(p->des0);
	int ret;

	ret = enh_desc_get_rx_basic_status(rdes0, x);
	if (ret & (dma_own | rx_not_ls))
		return ret;

	/* Rx COE type 2 has been available since v3.30a. If it's synthesized
	 * into the GMAC the Bits 5, 7, and 0 state reflects the Rx COE
	 * outcome. The bits permutation had been available up to the DW GMAC
	 * v3.50a IP-core release which initially introduced the extended
	 * enhanced descriptors.
	 */
	return ret | com_desc_rx_coe_rdes0(!!(rdes0 & RDES0_IPC_CSUM_ERROR),
					   !!(rdes0 & RDES0_FRAME_TYPE),
					   !!(rdes0 & RDES0_PAYLOAD_CSUM_ERR));
}

static int enh_desc_get_rx_status_ext(struct stmmac_extra_stats *x,
				      struct dma_desc *p)
{
	unsigned int rdes4 = le32_to_cpu(to_dma_extended_desc(p)->des4);
	unsigned int rdes0 = le32_to_cpu(p->des0);
	int ret;

	ret = enh_desc_get_rx_basic_status(rdes0, x);
	if (ret & (dma_own | rx_not_ls))
		return ret;

	/* In DW GMAC v3.50a IP-core the Rx COE type 2 status was moved to
	 * the extended part of the enhanced descriptor (RDES4). The RDES0
	 * 5, 7, and 0 bits permutation has been no longer relevant.
	 */
	if (rdes0 & ERDES0_RX_MAC_ADDR)
		ret |= enh_desc_rx_ext_status(rdes4, x);
	else /* No extended part so no CSUM/Timestamp processed */
		ret |= csum_none;

	return ret;
}

static void enh_desc_init_rx_desc(struct dma_desc *p, int disable_rx_ic,
				  int mode, int end, int bfsize)
{
	p->des0 |= cpu_to_le32(RDES0_OWN);

	if (mode == STMMAC_CHAIN_MODE)
		enh_desc_rx_set_on_chain(p, bfsize);
	else
		enh_desc_rx_set_on_ring(p, end, bfsize);

	if (disable_rx_ic)
		p->des1 |= cpu_to_le32(ERDES1_DISABLE_IC);
}

static void enh_desc_init_tx_desc(struct dma_desc *p, int mode, int end)
{
	memset(p, 0, offsetof(struct dma_desc, des2));
	if (mode == STMMAC_CHAIN_MODE)
		enh_desc_end_tx_desc_on_chain(p, 0, false);
	else
		enh_desc_end_tx_desc_on_ring(p, end);
}

static int enh_desc_get_tx_owner(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & ETDES0_OWN) >> 31;
}

static void enh_desc_set_tx_owner(struct dma_desc *p)
{
	p->des0 |= cpu_to_le32(ETDES0_OWN);
}

static void enh_desc_set_rx_owner(struct dma_desc *p, int disable_rx_ic)
{
	p->des0 |= cpu_to_le32(RDES0_OWN);
}

static unsigned int enh_desc_get_rx_len(int mode)
{
	if (mode == STMMAC_CHAIN_MODE)
		return enh_desc_rx_desc_len_on_chain();
	else
		return enh_desc_rx_desc_len_on_ring();
}

static void enh_desc_release_rx_desc(struct dma_desc *p, int disable_rx_ic,
				     int is_fs, bool rx_own)
{
	unsigned int rdes1 = le32_to_cpu(p->des1);

	if (disable_rx_ic)
		rdes1 |= cpu_to_le32(ERDES1_DISABLE_IC);
	else
		rdes1 &= cpu_to_le32(~ERDES1_DISABLE_IC);

	p->des1 = cpu_to_le32(rdes1);

	/* Before releasing the initial descriptor make sure that all
	 * the previous writes are visible to the controller.
	 */
	if (is_fs && rx_own)
		dma_wmb();

	if (rx_own)
		p->des0 = cpu_to_le32(RDES0_OWN);
}

static void enh_desc_prepare_rx_desc(struct dma_desc *p, int mode,
				     dma_addr_t np, bool hwts_rx, int bfsize)
{
	if (mode == STMMAC_CHAIN_MODE)
		enh_desc_rx_set_buf_on_chain(p, np, hwts_rx);
	else
		enh_desc_rx_set_buf_on_ring(p, bfsize);
}

static int enh_desc_get_tx_ls(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & ETDES0_LAST_SEGMENT) >> 29;
}

static unsigned int enh_desc_get_tx_len(int mode)
{
	if (mode == STMMAC_CHAIN_MODE)
		return enh_desc_tx_desc_len_on_chain();
	else
		return enh_desc_tx_desc_len_on_ring();
}

static void enh_desc_release_tx_desc(struct dma_desc *p, int mode,
				     dma_addr_t np, bool hwts_tx)
{
	int ter = (le32_to_cpu(p->des0) & ETDES0_END_RING) >> 21;

	memset(p, 0, offsetof(struct dma_desc, des2));
	if (mode == STMMAC_CHAIN_MODE)
		enh_desc_end_tx_desc_on_chain(p, np, hwts_tx);
	else
		enh_desc_end_tx_desc_on_ring(p, ter);
}

static void enh_desc_prepare_tx_desc(struct dma_desc *p, int is_fs, int len,
				     bool csum_flag, int mode, bool tx_own,
				     bool ls, unsigned int tot_pkt_len)
{
	unsigned int tdes0 = le32_to_cpu(p->des0);

	if (mode == STMMAC_CHAIN_MODE)
		enh_set_tx_desc_len_on_chain(p, len);
	else
		enh_set_tx_desc_len_on_ring(p, len);

	if (is_fs)
		tdes0 |= ETDES0_FIRST_SEGMENT;
	else
		tdes0 &= ~ETDES0_FIRST_SEGMENT;

	if (likely(csum_flag))
		tdes0 |= (TX_CIC_FULL << ETDES0_CHECKSUM_INSERTION_SHIFT);
	else
		tdes0 &= ~(TX_CIC_FULL << ETDES0_CHECKSUM_INSERTION_SHIFT);

	if (ls)
		tdes0 |= ETDES0_LAST_SEGMENT;

	/* Finally set the OWN bit. Later the DMA will start! */
	if (tx_own)
		tdes0 |= ETDES0_OWN;

	if (is_fs && tx_own)
		/* When the own bit, for the first frame, has to be set, all
		 * descriptors for the same frame has to be set before, to
		 * avoid race condition.
		 */
		dma_wmb();

	p->des0 = cpu_to_le32(tdes0);
}

static void enh_desc_set_tx_ic(struct dma_desc *p)
{
	p->des0 |= cpu_to_le32(ETDES0_INTERRUPT);
}

static int enh_desc_get_rx_frame_len(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & RDES0_FRAME_LEN_MASK) >>
	       RDES0_FRAME_LEN_SHIFT;
}

static void enh_desc_enable_tx_timestamp(struct dma_desc *p)
{
	p->des0 |= cpu_to_le32(ETDES0_TIME_STAMP_ENABLE);
}

static int enh_desc_get_tx_timestamp_status(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & ETDES0_TIME_STAMP_STATUS) >> 17;
}

static void enh_desc_get_timestamp(void *desc, u32 ats, u64 *ts)
{
	u64 ns;

	if (ats) {
		struct dma_extended_desc *p = (struct dma_extended_desc *)desc;
		ns = le32_to_cpu(p->des6);
		/* convert high/sec time stamp value to nanosecond */
		ns += le32_to_cpu(p->des7) * 1000000000ULL;
	} else {
		struct dma_desc *p = (struct dma_desc *)desc;
		ns = le32_to_cpu(p->des2);
		ns += le32_to_cpu(p->des3) * 1000000000ULL;
	}

	*ts = ns;
}

static int enh_desc_get_rx_timestamp_status(void *desc, void *next_desc,
					    u32 ats)
{
	if (ats) {
		struct dma_extended_desc *p = (struct dma_extended_desc *)desc;
		return (le32_to_cpu(p->basic.des0) & RDES0_IPC_CSUM_ERROR) >> 7;
	} else {
		struct dma_desc *p = (struct dma_desc *)desc;
		if ((le32_to_cpu(p->des2) == 0xffffffff) &&
		    (le32_to_cpu(p->des3) == 0xffffffff))
			/* timestamp is corrupted, hence don't store it */
			return 0;
		else
			return 1;
	}
}

static void enh_desc_display_ring(void *head, unsigned int size, bool rx,
				  dma_addr_t dma_rx_phy, unsigned int desc_size)
{
	struct dma_extended_desc *ep = (struct dma_extended_desc *)head;
	dma_addr_t dma_addr;
	int i;

	pr_info("Extended %s descriptor ring:\n", rx ? "RX" : "TX");

	for (i = 0; i < size; i++) {
		u64 x;
		dma_addr = dma_rx_phy + i * sizeof(*ep);

		x = *(u64 *)ep;
		pr_info("%03d [%pad]: 0x%x 0x%x 0x%x 0x%x\n",
			i, &dma_addr,
			(unsigned int)x, (unsigned int)(x >> 32),
			ep->basic.des2, ep->basic.des3);
		ep++;
	}
	pr_info("\n");
}

static void enh_desc_set_addr(struct dma_desc *p, dma_addr_t addr)
{
	p->des2 = cpu_to_le32(addr);
}

static void enh_desc_clear(struct dma_desc *p)
{
	p->des2 = 0;
}

static void enh_desc_set_vlan(struct dma_desc *p, u32 type)
{
	type <<= ETDES0_VLIC_SHIFT;
	p->des0 |= cpu_to_le32(type & ETDES0_VLIC_MASK);
}

const struct stmmac_desc_ops enh_desc_ops = {
	.tx_status = enh_desc_get_tx_status,
	.rx_status = enh_desc_get_rx_status_nocoe,
	.get_rx_len = enh_desc_get_rx_len,
	.get_tx_len = enh_desc_get_tx_len,
	.init_rx_desc = enh_desc_init_rx_desc,
	.init_tx_desc = enh_desc_init_tx_desc,
	.get_tx_owner = enh_desc_get_tx_owner,
	.release_rx_desc = enh_desc_release_rx_desc,
	.prepare_rx_desc = enh_desc_prepare_rx_desc,
	.release_tx_desc = enh_desc_release_tx_desc,
	.prepare_tx_desc = enh_desc_prepare_tx_desc,
	.set_tx_ic = enh_desc_set_tx_ic,
	.get_tx_ls = enh_desc_get_tx_ls,
	.set_tx_owner = enh_desc_set_tx_owner,
	.set_rx_owner = enh_desc_set_rx_owner,
	.get_rx_frame_len = enh_desc_get_rx_frame_len,
	.enable_tx_timestamp = enh_desc_enable_tx_timestamp,
	.get_tx_timestamp_status = enh_desc_get_tx_timestamp_status,
	.get_timestamp = enh_desc_get_timestamp,
	.get_rx_timestamp_status = enh_desc_get_rx_timestamp_status,
	.display_ring = enh_desc_display_ring,
	.set_addr = enh_desc_set_addr,
	.clear = enh_desc_clear,
	.set_vlan = enh_desc_set_vlan,
};

const struct stmmac_desc_ops enh_desc_noext_ops = {
	.tx_status = enh_desc_get_tx_status,
	.rx_status = enh_desc_get_rx_status_noext,
	.get_rx_len = enh_desc_get_rx_len,
	.get_tx_len = enh_desc_get_tx_len,
	.init_rx_desc = enh_desc_init_rx_desc,
	.init_tx_desc = enh_desc_init_tx_desc,
	.get_tx_owner = enh_desc_get_tx_owner,
	.release_rx_desc = enh_desc_release_rx_desc,
	.prepare_rx_desc = enh_desc_prepare_rx_desc,
	.release_tx_desc = enh_desc_release_tx_desc,
	.prepare_tx_desc = enh_desc_prepare_tx_desc,
	.set_tx_ic = enh_desc_set_tx_ic,
	.get_tx_ls = enh_desc_get_tx_ls,
	.set_tx_owner = enh_desc_set_tx_owner,
	.set_rx_owner = enh_desc_set_rx_owner,
	.get_rx_frame_len = enh_desc_get_rx_frame_len,
	.enable_tx_timestamp = enh_desc_enable_tx_timestamp,
	.get_tx_timestamp_status = enh_desc_get_tx_timestamp_status,
	.get_timestamp = enh_desc_get_timestamp,
	.get_rx_timestamp_status = enh_desc_get_rx_timestamp_status,
	.display_ring = enh_desc_display_ring,
	.set_addr = enh_desc_set_addr,
	.clear = enh_desc_clear,
	.set_vlan = enh_desc_set_vlan,
};

const struct stmmac_desc_ops enh_desc_ext_ops = {
	.tx_status = enh_desc_get_tx_status,
	.rx_status = enh_desc_get_rx_status_ext,
	.get_rx_len = enh_desc_get_rx_len,
	.get_tx_len = enh_desc_get_tx_len,
	.init_rx_desc = enh_desc_init_rx_desc,
	.init_tx_desc = enh_desc_init_tx_desc,
	.get_tx_owner = enh_desc_get_tx_owner,
	.release_rx_desc = enh_desc_release_rx_desc,
	.prepare_rx_desc = enh_desc_prepare_rx_desc,
	.release_tx_desc = enh_desc_release_tx_desc,
	.prepare_tx_desc = enh_desc_prepare_tx_desc,
	.set_tx_ic = enh_desc_set_tx_ic,
	.get_tx_ls = enh_desc_get_tx_ls,
	.set_tx_owner = enh_desc_set_tx_owner,
	.set_rx_owner = enh_desc_set_rx_owner,
	.get_rx_frame_len = enh_desc_get_rx_frame_len,
	.enable_tx_timestamp = enh_desc_enable_tx_timestamp,
	.get_tx_timestamp_status = enh_desc_get_tx_timestamp_status,
	.get_timestamp = enh_desc_get_timestamp,
	.get_rx_timestamp_status = enh_desc_get_rx_timestamp_status,
	.display_ring = enh_desc_display_ring,
	.set_addr = enh_desc_set_addr,
	.clear = enh_desc_clear,
	.set_vlan = enh_desc_set_vlan,
};
