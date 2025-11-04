/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
  Header File to describe Normal/enhanced descriptor functions used for RING
  and CHAINED modes.

  Copyright(C) 2011  STMicroelectronics Ltd

  It defines all the functions used to handle the normal/enhanced
  descriptors in case of the DMA is configured to work in chained or
  in ring mode.


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#ifndef __DESC_COM_H__
#define __DESC_COM_H__

/* Specific functions used for Ring mode */

/* Enhanced descriptors */
static inline void enh_desc_rx_set_on_ring(struct dma_desc *p, int end,
					   int bfsize)
{
	if (bfsize > STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer1 = STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK);
		unsigned int buffer2 =
			(bfsize - buffer1) << ERDES1_BUFFER2_SIZE_SHIFT;

		p->des1 |= cpu_to_le32((buffer2 & ERDES1_BUFFER2_SIZE_MASK) |
				       buffer1);
	} else {
		p->des1 |= cpu_to_le32(bfsize & ERDES1_BUFFER1_SIZE_MASK);
	}

	if (end)
		p->des1 |= cpu_to_le32(ERDES1_END_RING);
}

static inline void enh_desc_rx_set_buf_on_ring(struct dma_desc *p, int bfsize)
{
	if (bfsize > STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer1 = STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK);

		/* Note DMA will ignore the data-bus unaligned offset of the
		 * non-first Rx buffers.
		 */
		p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + buffer1);
	}
}

static inline unsigned int enh_desc_rx_desc_len_on_ring(void)
{
	return 2 * STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK);
}

static inline void enh_desc_end_tx_desc_on_ring(struct dma_desc *p, int end)
{
	if (end)
		p->des0 |= cpu_to_le32(ETDES0_END_RING);
	else
		p->des0 &= cpu_to_le32(~ETDES0_END_RING);
}

static inline void enh_set_tx_desc_len_on_ring(struct dma_desc *p, int len)
{
	if (unlikely(len > ETDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer2 =
			(len - ETDES1_BUFFER1_SIZE_MASK) << ETDES1_BUFFER2_SIZE_SHIFT;

		p->des1 |= cpu_to_le32((buffer2 & ETDES1_BUFFER2_SIZE_MASK) |
				       ETDES1_BUFFER1_SIZE_MASK);
		p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + ETDES1_BUFFER1_SIZE_MASK);
	} else {
		p->des1 |= cpu_to_le32(len & ETDES1_BUFFER1_SIZE_MASK);
	}
}

static inline unsigned int enh_desc_tx_desc_len_on_ring(void)
{
	return 2 * ETDES1_BUFFER1_SIZE_MASK;
}

/* Normal descriptors */
static inline void ndesc_rx_set_on_ring(struct dma_desc *p, int end, int bfsize)
{
	if (bfsize > STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer1 = STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK);
		unsigned int buffer2 =
			(bfsize - buffer1) << RDES1_BUFFER2_SIZE_SHIFT;

		p->des1 |= cpu_to_le32((buffer2 & RDES1_BUFFER2_SIZE_MASK) |
				       buffer1);
	} else {
		p->des1 |= cpu_to_le32(bfsize & RDES1_BUFFER1_SIZE_MASK);
	}

	if (end)
		p->des1 |= cpu_to_le32(RDES1_END_RING);
}

static inline void ndesc_rx_set_buf_on_ring(struct dma_desc *p, int bfsize)
{
	if (bfsize > STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer1 = STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK);

		/* Note DMA will ignore the data-bus unaligned offset of the
		 * non-first Rx buffers.
		 */
		p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + buffer1);
	}
}

static inline unsigned int ndesc_rx_desc_len_on_ring(void)
{
	return 2 * STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK);
}

static inline void ndesc_end_tx_desc_on_ring(struct dma_desc *p, int end)
{
	if (end)
		p->des1 |= cpu_to_le32(TDES1_END_RING);
	else
		p->des1 &= cpu_to_le32(~TDES1_END_RING);
}

static inline void norm_set_tx_desc_len_on_ring(struct dma_desc *p, int len)
{
	if (unlikely(len > TDES1_BUFFER1_SIZE_MASK)) {
		unsigned int buffer2 =
			(len - TDES1_BUFFER1_SIZE_MASK) << TDES1_BUFFER2_SIZE_SHIFT;

		p->des1 |= cpu_to_le32((buffer2 & TDES1_BUFFER2_SIZE_MASK) |
				       TDES1_BUFFER1_SIZE_MASK);
		p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + TDES1_BUFFER1_SIZE_MASK);
	} else {
		p->des1 |= cpu_to_le32(len & TDES1_BUFFER1_SIZE_MASK);
	}
}

static inline unsigned int ndesc_tx_desc_len_on_ring(void)
{
	return 2 * TDES1_BUFFER1_SIZE_MASK;
}

/* Specific functions used for Chain mode */

/* Enhanced descriptors */
static inline void enh_desc_rx_set_on_chain(struct dma_desc *p, int bfsize)
{
	bfsize = umin(bfsize, STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK));
	p->des1 |= cpu_to_le32((bfsize & ERDES1_BUFFER1_SIZE_MASK) |
			       ERDES1_SECOND_ADDRESS_CHAINED);
}

static inline void enh_desc_rx_set_buf_on_chain(struct dma_desc *p, dma_addr_t np,
						bool hwts_rx_en)
{
	/* des3 is overwritten with timestamp in case of IEEE 1588-2002 */
	if (hwts_rx_en)
		p->des3 = cpu_to_le32(lower_32_bits(np));
}

static inline unsigned int enh_desc_rx_desc_len_on_chain(void)
{
	return STMMAC_RX_BUF_ALIGN(ERDES1_BUFFER1_SIZE_MASK);
}

static inline void enh_desc_end_tx_desc_on_chain(struct dma_desc *p, dma_addr_t np,
						 bool hwts_tx_en)
{
	p->des0 |= cpu_to_le32(ETDES0_SECOND_ADDRESS_CHAINED);
	/* des3 is overwritten with timestamp in case of IEEE 1588-2002 */
	if (hwts_tx_en)
		p->des3 = cpu_to_le32(lower_32_bits(np));
}

static inline void enh_set_tx_desc_len_on_chain(struct dma_desc *p, int len)
{
	p->des1 |= cpu_to_le32(len & ETDES1_BUFFER1_SIZE_MASK);
}

static inline unsigned int enh_desc_tx_desc_len_on_chain(void)
{
	return ETDES1_BUFFER1_SIZE_MASK;
}

/* Normal descriptors */
static inline void ndesc_rx_set_on_chain(struct dma_desc *p, int bfsize)
{
	bfsize = umin(bfsize, STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK));
	p->des1 |= cpu_to_le32((bfsize & RDES1_BUFFER1_SIZE_MASK) |
			       RDES1_SECOND_ADDRESS_CHAINED);
}

static inline void ndesc_rx_set_buf_on_chain(struct dma_desc *p, dma_addr_t np,
					     bool hwts_rx_en)
{
	/* des3 is overwritten with timestamp in case of IEEE 1588-2002 */
	if (hwts_rx_en)
		p->des3 = cpu_to_le32(lower_32_bits(np));
}

static inline unsigned int ndesc_rx_desc_len_on_chain(void)
{
	return STMMAC_RX_BUF_ALIGN(RDES1_BUFFER1_SIZE_MASK);
}

static inline void ndesc_end_tx_desc_on_chain(struct dma_desc *p, dma_addr_t np,
					      bool hwts_tx_en)
{
	p->des1 |= cpu_to_le32(TDES1_SECOND_ADDRESS_CHAINED);
	/* des3 is overwritten with timestamp in case of IEEE 1588-2002 */
	if (hwts_tx_en)
		p->des3 = cpu_to_le32(lower_32_bits(np));
}

static inline void norm_set_tx_desc_len_on_chain(struct dma_desc *p, int len)
{
	p->des1 |= cpu_to_le32(len & TDES1_BUFFER1_SIZE_MASK);
}

static inline unsigned int ndesc_tx_desc_len_on_chain(void)
{
	return TDES1_BUFFER1_SIZE_MASK;
}

/* Functions used for all ring/chain modes, enhanced/normal descriptors */

static inline int com_desc_rx_coe_rdes0(int ipc_err, int type, int payload_err)
{
	u32 status = (type << 2 | ipc_err << 1 | payload_err) & 0x7;

	/* bits 5 7 0 | Frame status
	 * ----------------------------------------------------------
	 *      0 0 0 | IEEE 802.3 Type frame (length < 1536 octects)
	 *      1 0 0 | IPv4/6 No CSUM errorS.
	 *      1 0 1 | IPv4/6 CSUM PAYLOAD error
	 *      1 1 0 | IPv4/6 CSUM IP HR error
	 *      1 1 1 | IPv4/6 IP PAYLOAD AND HEADER errorS
	 *      0 0 1 | IPv4/6 unsupported IP PAYLOAD
	 *      0 1 1 | COE bypassed.. no IPv4/6 frame
	 *      0 1 0 | Reserved.
	 */
	switch (status) {
	case 0x0:
		return llc_snap;
	case 0x1:
		return csum_none;
	case 0x3:
		return csum_none;
	case 0x4:
		return good_frame;
	case 0x5:
		return csum_err;
	case 0x6:
		return csum_err;
	case 0x7:
		return csum_err;
	default:
		return csum_none;
	}
}

#endif /* __DESC_COM_H__ */
