/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * stmmac_pcs.h: Physical Coding Sublayer Header File
 *
 * Copyright (C) 2016 STMicroelectronics (R&D) Limited
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 */

#ifndef __STMMAC_PCS_H__
#define __STMMAC_PCS_H__

#include <linux/slab.h>
#include <linux/io.h>
#include "common.h"

/* PCS registers (AN/TBI/SGMII/RGMII) offsets */
#define STMMAC_PCS_AN_CTRL	0x00		/* AN control */
#define STMMAC_PCS_AN_STATUS	0x04		/* AN status */

/* ADV, LPA and EXP are only available for the TBI and RTBI interfaces */
#define STMMAC_PCS_ANE_ADV	0x08		/* ANE Advertisement */
#define STMMAC_PCS_ANE_LPA	0x0c		/* ANE link partener ability */
#define STMMAC_PCS_ANE_EXP	0x10		/* ANE expansion */
#define STMMAC_PCS_TBI		0x14		/* TBI extend status */

/* AN Configuration defines */
#define STMMAC_PCS_AN_CTRL_RAN		BIT(9)	/* Restart Auto-Negotiation */
#define STMMAC_PCS_AN_CTRL_ANE		BIT(12)	/* Auto-Negotiation Enable */
#define STMMAC_PCS_AN_CTRL_ELE		BIT(14)	/* External Loopback Enable */
#define STMMAC_PCS_AN_CTRL_ECD		BIT(16)	/* Enable Comma Detect */
#define STMMAC_PCS_AN_CTRL_LR		BIT(17)	/* Lock to Reference */
#define STMMAC_PCS_AN_CTRL_SGMRAL	BIT(18)	/* SGMII RAL Control */

/* AN Status defines */
#define STMMAC_PCS_AN_STATUS_LS		BIT(2)	/* Link Status 0:down 1:up */
#define STMMAC_PCS_AN_STATUS_ANA	BIT(3)	/* Auto-Negotiation Ability */
#define STMMAC_PCS_AN_STATUS_ANC	BIT(5)	/* Auto-Negotiation Complete */
#define STMMAC_PCS_AN_STATUS_ES		BIT(8)	/* Extended Status */

/* ADV and LPA defines */
#define STMMAC_PCS_ANE_FD		BIT(5)
#define STMMAC_PCS_ANE_HD		BIT(6)
#define STMMAC_PCS_ANE_PSE		GENMASK(8, 7)
#define STMMAC_PCS_ANE_PSE_SHIFT	7
#define STMMAC_PCS_ANE_RFE		GENMASK(13, 12)
#define STMMAC_PCS_ANE_RFE_SHIFT	12
#define STMMAC_PCS_ANE_ACK		BIT(14)

/* MAC specific status - for RGMII and SGMII. These appear as
 * GMAC_RGSMIIIS[15:0] and GMAC_PHYIF_CONTROL_STATUS[31:16]
 */
#define GMAC_RS_STAT_LNKMOD		BIT(0)
#define GMAC_RS_STAT_SPEED		GENMASK(2, 1)
#define GMAC_RS_STAT_LNKSTS		BIT(3)
#define GMAC_RS_STAT_JABTO		BIT(4)
#define GMAC_RS_STAT_FALSECARDET	BIT(5)

#define GMAC_RS_STAT_SPEED_125		2
#define GMAC_RS_STAT_SPEED_25		1
#define GMAC_RS_STAT_SPEED_2_5		0

/**
 * dwmac_pcs_isr - TBI, RTBI, or SGMII PHY ISR
 * @spcs: pointer to &struct stmmac_pcs
 * @intr_status: GMAC core interrupt status
 * @x: pointer to log these events as stats
 * Description: it is the ISR for PCS events: Auto-Negotiation Completed and
 * Link status.
 */
static inline void dwmac_pcs_isr(struct stmmac_pcs *spcs,
				 unsigned int intr_status,
				 struct stmmac_extra_stats *x)
{
	u32 val = readl(spcs->pcs_base + STMMAC_PCS_AN_STATUS);

	if (intr_status & PCS_ANE_IRQ) {
		x->irq_pcs_ane_n++;
		if (val & STMMAC_PCS_AN_STATUS_ANC)
			pr_info("stmmac_pcs: ANE process completed\n");
	}

	if (intr_status & PCS_LINK_IRQ) {
		x->irq_pcs_link_n++;
		if (val & STMMAC_PCS_AN_STATUS_LS)
			pr_info("stmmac_pcs: Link Up\n");
		else
			pr_info("stmmac_pcs: Link Down\n");
	}
}

static inline bool dwmac_rs_decode_stat(struct phylink_link_state *state,
					uint16_t rs_stat)
{
	unsigned int speed;

	state->link = !!(rs_stat & GMAC_RS_STAT_LNKSTS);
	if (!state->link)
		return false;

	speed = FIELD_GET(GMAC_RS_STAT_SPEED, rs_stat);
	switch (speed) {
	case GMAC_RS_STAT_SPEED_125:
		state->speed = SPEED_1000;
		break;
	case GMAC_RS_STAT_SPEED_25:
		state->speed = SPEED_100;
		break;
	case GMAC_RS_STAT_SPEED_2_5:
		state->speed = SPEED_10;
		break;
	default:
		state->link = false;
		return false;
	}

	state->duplex = rs_stat & GMAC_RS_STAT_LNKMOD ?
			DUPLEX_FULL : DUPLEX_HALF;

	return true;
}

void dwmac_ctrl_ane(struct stmmac_priv *priv, bool ane, bool srgmi_ral,
		    bool loopback);

int dwmac_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
		     phy_interface_t interface,
		     const unsigned long *advertising,
		     bool permit_pause_to_mac);

#endif /* __STMMAC_PCS_H__ */
