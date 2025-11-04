// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This contains the functions to handle the normal descriptors.

  Copyright (C) 2007-2009  STMicroelectronics Ltd


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/stmmac.h>
#include "common.h"
#include "descs_com.h"

static int ndesc_get_tx_status(struct stmmac_extra_stats *x,
			       struct dma_desc *p)
{
	unsigned int tdes0 = le32_to_cpu(p->des0);
	unsigned int tdes1 = le32_to_cpu(p->des1);
	int ret = tx_done;

	/* Get tx owner first */
	if (unlikely(tdes0 & TDES0_OWN))
		return tx_dma_own;

	/* Verify tx error by looking at the last segment. */
	if (likely(!(tdes1 & TDES1_LAST_SEGMENT)))
		return tx_not_ls;

	if (unlikely(tdes0 & TDES0_ERROR_SUMMARY)) {
		if (unlikely(tdes0 & TDES0_UNDERFLOW_ERROR)) {
			x->tx_underflow++;
		}
		if (unlikely(tdes0 & TDES0_NO_CARRIER)) {
			x->tx_carrier++;
		}
		if (unlikely(tdes0 & TDES0_LOSS_CARRIER)) {
			x->tx_losscarrier++;
		}
		if (unlikely((tdes0 & TDES0_EXCESSIVE_DEFERRAL) ||
			     (tdes0 & TDES0_EXCESSIVE_COLLISIONS) ||
			     (tdes0 & TDES0_LATE_COLLISION))) {
			unsigned int collisions;

			collisions = (tdes0 & TDES0_COLLISION_COUNT_MASK) >> 3;
			x->tx_collision += collisions;
		}
		ret = tx_err;
	}

	if (tdes0 & TDES0_VLAN_FRAME)
		x->tx_vlan++;

	if (unlikely(tdes0 & TDES0_DEFERRED))
		x->tx_deferred++;

	return ret;
}

/* This function verifies if each incoming frame has some errors
 * and, if required, updates the multicast statistics.
 * In case of success, it returns good_frame because the GMAC device
 * is supposed to be able to compute the csum in HW. */
static int ndesc_get_rx_basic_status(unsigned int rdes0,
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
			x->overflow_error++;
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
		x->sa_filter_fail++;

	if (unlikely(rdes0 & RDES0_DA_FILTER_FAIL))
		x->da_rx_filter_fail++;

	if (unlikely(rdes0 & RDES0_LENGTH_ERROR))
		x->rx_length++;

#ifdef STMMAC_VLAN_TAG_USED
	if (rdes0 & RDES0_VLAN_TAG)
		x->vlan_tag++;
#endif
	return ret;
}

static int ndesc_get_rx_status_nocoe(struct stmmac_extra_stats *x,
				     struct dma_desc *p)
{
	return ndesc_get_rx_basic_status(le32_to_cpu(p->des0), x);
}

static int ndesc_get_rx_status_coe2(struct stmmac_extra_stats *x,
				    struct dma_desc *p)
{
	unsigned int rdes0 = le32_to_cpu(p->des0);
	int ret;

	ret = ndesc_get_rx_basic_status(rdes0, x);
	if (ret & (dma_own | rx_not_ls))
		return ret;

	/* Rx COE type 2 has been available since v3.30a. If it's synthesized
	 * into the GMAC the Bits 5, 7, and 0 state reflects the Rx COE
	 * outcome.
	 */
	return ret | com_desc_rx_coe_rdes0(!!(rdes0 & RDES0_IPC_CSUM_ERROR),
					   !!(rdes0 & RDES0_FRAME_TYPE),
					   !!(rdes0 & RDES0_PAYLOAD_CSUM_ERR));
}

static void ndesc_init_rx_desc(struct dma_desc *p, int disable_rx_ic, int mode,
			       int end, int bfsize)
{
	p->des0 |= cpu_to_le32(RDES0_OWN);

	if (mode == STMMAC_CHAIN_MODE)
		ndesc_rx_set_on_chain(p, bfsize);
	else
		ndesc_rx_set_on_ring(p, end, bfsize);

	if (disable_rx_ic)
		p->des1 |= cpu_to_le32(RDES1_DISABLE_IC);
}

static void ndesc_init_tx_desc(struct dma_desc *p, int mode, int end)
{
	memset(p, 0, offsetof(struct dma_desc, des2));
	if (mode == STMMAC_CHAIN_MODE)
		ndesc_end_tx_desc_on_chain(p, 0, false);
	else
		ndesc_end_tx_desc_on_ring(p, end);
}

static int ndesc_get_tx_owner(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & TDES0_OWN) >> 31;
}

static void ndesc_set_tx_owner(struct dma_desc *p)
{
	p->des0 |= cpu_to_le32(TDES0_OWN);
}

static void ndesc_set_rx_owner(struct dma_desc *p, int disable_rx_ic)
{
	p->des0 |= cpu_to_le32(RDES0_OWN);
}

static unsigned int ndesc_get_rx_len(int mode)
{
	if (mode == STMMAC_CHAIN_MODE)
		return ndesc_rx_desc_len_on_chain();
	else
		return ndesc_rx_desc_len_on_ring();
}

static void ndesc_release_rx_desc(struct dma_desc *p, int disable_rx_ic,
				  int is_fs, bool rx_own)
{
	unsigned int rdes1 = le32_to_cpu(p->des1);

	if (disable_rx_ic)
		rdes1 |= cpu_to_le32(RDES1_DISABLE_IC);
	else
		rdes1 &= cpu_to_le32(~RDES1_DISABLE_IC);

	p->des1 = cpu_to_le32(rdes1);

	/* Before releasing the initial descriptor make sure that all
	 * the previous writes are visible to the controller.
	 */
	if (is_fs && rx_own)
		dma_wmb();

	if (rx_own)
		p->des0 = cpu_to_le32(RDES0_OWN);
}

static void ndesc_prepare_rx_desc(struct dma_desc *p, int mode,
				  dma_addr_t np, bool hwts_rx, int bfsize)
{
	if (mode == STMMAC_CHAIN_MODE)
		ndesc_rx_set_buf_on_chain(p, np, hwts_rx);
	else
		ndesc_rx_set_buf_on_ring(p, bfsize);
}

static int ndesc_get_tx_ls(struct dma_desc *p)
{
	return (le32_to_cpu(p->des1) & TDES1_LAST_SEGMENT) >> 30;
}

static unsigned int ndesc_get_tx_len(int mode)
{
	if (mode == STMMAC_CHAIN_MODE)
		return ndesc_tx_desc_len_on_chain();
	else
		return ndesc_tx_desc_len_on_ring();
}

static void ndesc_release_tx_desc(struct dma_desc *p, int mode,
				  dma_addr_t np, bool hwts_tx)
{
	int ter = (le32_to_cpu(p->des1) & TDES1_END_RING) >> 25;

	memset(p, 0, offsetof(struct dma_desc, des2));
	if (mode == STMMAC_CHAIN_MODE)
		ndesc_end_tx_desc_on_chain(p, np, hwts_tx);
	else
		ndesc_end_tx_desc_on_ring(p, ter);
}

static void ndesc_prepare_tx_desc(struct dma_desc *p, int is_fs, int len,
				  bool csum_flag, int mode, bool tx_own,
				  bool ls, unsigned int tot_pkt_len)
{
	unsigned int tdes1 = le32_to_cpu(p->des1);

	if (is_fs)
		tdes1 |= TDES1_FIRST_SEGMENT;
	else
		tdes1 &= ~TDES1_FIRST_SEGMENT;

	if (likely(csum_flag))
		tdes1 |= (TX_CIC_FULL) << TDES1_CHECKSUM_INSERTION_SHIFT;
	else
		tdes1 &= ~(TX_CIC_FULL << TDES1_CHECKSUM_INSERTION_SHIFT);

	if (ls)
		tdes1 |= TDES1_LAST_SEGMENT;

	p->des1 = cpu_to_le32(tdes1);

	if (mode == STMMAC_CHAIN_MODE)
		norm_set_tx_desc_len_on_chain(p, len);
	else
		norm_set_tx_desc_len_on_ring(p, len);

	if (tx_own)
		p->des0 |= cpu_to_le32(TDES0_OWN);
}

static void ndesc_set_tx_ic(struct dma_desc *p)
{
	p->des1 |= cpu_to_le32(TDES1_INTERRUPT);
}

static int ndesc_get_rx_frame_len(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & RDES0_FRAME_LEN_MASK) >>
	       RDES0_FRAME_LEN_SHIFT;
}

static void ndesc_enable_tx_timestamp(struct dma_desc *p)
{
	p->des1 |= cpu_to_le32(TDES1_TIME_STAMP_ENABLE);
}

static int ndesc_get_tx_timestamp_status(struct dma_desc *p)
{
	return (le32_to_cpu(p->des0) & TDES0_TIME_STAMP_STATUS) >> 17;
}

static void ndesc_get_timestamp(void *desc, u32 ats, u64 *ts)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	u64 ns;

	ns = le32_to_cpu(p->des2);
	/* convert high/sec time stamp value to nanosecond */
	ns += le32_to_cpu(p->des3) * 1000000000ULL;

	*ts = ns;
}

static int ndesc_get_rx_timestamp_status(void *desc, void *next_desc, u32 ats)
{
	struct dma_desc *p = (struct dma_desc *)desc;

	if ((le32_to_cpu(p->des2) == 0xffffffff) &&
	    (le32_to_cpu(p->des3) == 0xffffffff))
		/* timestamp is corrupted, hence don't store it */
		return 0;
	else
		return 1;
}

static void ndesc_display_ring(void *head, unsigned int size, bool rx,
			       dma_addr_t dma_rx_phy, unsigned int desc_size)
{
	struct dma_desc *p = (struct dma_desc *)head;
	dma_addr_t dma_addr;
	int i;

	pr_info("%s descriptor ring:\n", rx ? "RX" : "TX");

	for (i = 0; i < size; i++) {
		u64 x;
		dma_addr = dma_rx_phy + i * sizeof(*p);

		x = *(u64 *)p;
		pr_info("%03d [%pad]: 0x%x 0x%x 0x%x 0x%x",
			i, &dma_addr,
			(unsigned int)x, (unsigned int)(x >> 32),
			p->des2, p->des3);
		p++;
	}
	pr_info("\n");
}

static void ndesc_set_addr(struct dma_desc *p, dma_addr_t addr)
{
	p->des2 = cpu_to_le32(addr);
}

static void ndesc_clear(struct dma_desc *p)
{
	p->des2 = 0;
}

const struct stmmac_desc_ops ndesc_ops = {
	.tx_status = ndesc_get_tx_status,
	.rx_status = ndesc_get_rx_status_nocoe,
	.get_tx_len = ndesc_get_tx_len,
	.get_rx_len = ndesc_get_rx_len,
	.init_rx_desc = ndesc_init_rx_desc,
	.init_tx_desc = ndesc_init_tx_desc,
	.get_tx_owner = ndesc_get_tx_owner,
	.release_rx_desc = ndesc_release_rx_desc,
	.prepare_rx_desc = ndesc_prepare_rx_desc,
	.release_tx_desc = ndesc_release_tx_desc,
	.prepare_tx_desc = ndesc_prepare_tx_desc,
	.set_tx_ic = ndesc_set_tx_ic,
	.get_tx_ls = ndesc_get_tx_ls,
	.set_tx_owner = ndesc_set_tx_owner,
	.set_rx_owner = ndesc_set_rx_owner,
	.get_rx_frame_len = ndesc_get_rx_frame_len,
	.enable_tx_timestamp = ndesc_enable_tx_timestamp,
	.get_tx_timestamp_status = ndesc_get_tx_timestamp_status,
	.get_timestamp = ndesc_get_timestamp,
	.get_rx_timestamp_status = ndesc_get_rx_timestamp_status,
	.display_ring = ndesc_display_ring,
	.set_addr = ndesc_set_addr,
	.clear = ndesc_clear,
};

const struct stmmac_desc_ops ndesc_rxcoe2_ops = {
	.tx_status = ndesc_get_tx_status,
	.rx_status = ndesc_get_rx_status_coe2,
	.get_rx_len = ndesc_get_rx_len,
	.get_tx_len = ndesc_get_tx_len,
	.init_rx_desc = ndesc_init_rx_desc,
	.init_tx_desc = ndesc_init_tx_desc,
	.get_tx_owner = ndesc_get_tx_owner,
	.release_rx_desc = ndesc_release_rx_desc,
	.prepare_rx_desc = ndesc_prepare_rx_desc,
	.release_tx_desc = ndesc_release_tx_desc,
	.prepare_tx_desc = ndesc_prepare_tx_desc,
	.set_tx_ic = ndesc_set_tx_ic,
	.get_tx_ls = ndesc_get_tx_ls,
	.set_tx_owner = ndesc_set_tx_owner,
	.set_rx_owner = ndesc_set_rx_owner,
	.get_rx_frame_len = ndesc_get_rx_frame_len,
	.enable_tx_timestamp = ndesc_enable_tx_timestamp,
	.get_tx_timestamp_status = ndesc_get_tx_timestamp_status,
	.get_timestamp = ndesc_get_timestamp,
	.get_rx_timestamp_status = ndesc_get_rx_timestamp_status,
	.display_ring = ndesc_display_ring,
	.set_addr = ndesc_set_addr,
	.clear = ndesc_clear,
};
