// SPDX-License-Identifier: GPL-2.0-only
#include "stmmac.h"
#include "stmmac_pcs.h"

static void __dwmac_ctrl_ane(struct stmmac_pcs *spcs, bool ane, bool srgmi_ral,
			     bool loopback)

{
	u32 val;

	val = readl(spcs->pcs_base + STMMAC_PCS_AN_CTRL);

	/* Enable and restart the Auto-Negotiation */
	if (ane)
		val |= STMMAC_PCS_AN_CTRL_ANE | STMMAC_PCS_AN_CTRL_RAN;
	else
		val &= ~STMMAC_PCS_AN_CTRL_ANE;

	/* In case of MAC-2-MAC connection, block is configured to operate
	 * according to MAC conf register.
	 */
	if (srgmi_ral)
		val |= STMMAC_PCS_AN_CTRL_SGMRAL;

	if (loopback)
		val |= STMMAC_PCS_AN_CTRL_ELE;
	else
		val &= ~STMMAC_PCS_AN_CTRL_ELE;

	writel(val, spcs->pcs_base + STMMAC_PCS_AN_CTRL);
}

/**
 * dwmac_ctrl_ane - To program the AN Control Register.
 * @priv: pointer to &struct stmmac_priv
 * @ane: to enable the auto-negotiation
 * @srgmi_ral: to manage MAC-2-MAC SGMII connections.
 * @loopback: to cause the PHY to loopback tx data into rx path.
 * Description: this is the main function to configure the AN control register
 * and init the ANE, select loopback (usually for debugging purpose) and
 * configure SGMII RAL.
 */
void dwmac_ctrl_ane(struct stmmac_priv *priv, bool ane, bool srgmi_ral,
		    bool loopback)
{
	__dwmac_ctrl_ane(&priv->hw->mac_pcs, ane, srgmi_ral, loopback);
}

int dwmac_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
		     phy_interface_t interface,
		     const unsigned long *advertising,
		     bool permit_pause_to_mac)
{
	struct stmmac_pcs *spcs = phylink_pcs_to_stmmac_pcs(pcs);

	/* The RGMII interface does not have the STMMAC_PCS_AN_CTRL register */
	if (phy_interface_mode_is_rgmii(spcs->priv->plat->mac_interface))
		return 0;

	__dwmac_ctrl_ane(spcs, true, spcs->priv->hw->ps, false);

	return 0;
}
