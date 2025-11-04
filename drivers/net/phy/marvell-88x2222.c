// SPDX-License-Identifier: GPL-2.0+
/*
 * Marvell 88x2222 dual-port multi-speed ethernet transceiver.
 *
 * Supports:
 *	XAUI or 10GBase-R on the host side.
 *	1000Base-X or 10GBase-R on the line side.
 *	SGMII over 1000Base-X.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy.h>
#include <linux/gpio/driver.h>
#include <linux/delay.h>
#include <linux/mdio.h>
#include <linux/marvell_phy.h>
#include <linux/of.h>
#include <linux/sfp.h>
#include <linux/netdevice.h>

#include <dt-bindings/net/mv-phy-88x2222.h>

/* Port PCS Configuration */
#define	MV_PCS_CONFIG		0xF002
#define MV_PCS_HOST_PCS_SELECT	GENMASK(6, 0)
#define	MV_PCS_HOST_10GBR	0x71
#define	MV_PCS_HOST_XAUI	0x73
#define MV_PCS_LINE_PCS_SELECT	GENMASK(14, 8)
#define	MV_PCS_LINE_10GBR	0x71
#define	MV_PCS_LINE_1GBX_AN	0x7B
#define	MV_PCS_LINE_SGMII_AN	0x7F

/* Port Reset and Power Down */
#define	MV_PORT_RST	0xF003
#define	MV_LINE_RST_SW	BIT(15)
#define	MV_HOST_RST_SW	BIT(7)
#define	MV_PORT_RST_SW	(MV_LINE_RST_SW | MV_HOST_RST_SW)

/* GPIO Interrupt Enable */
#define MV_GPIO_INT_EN			0xF010

/* GPIO Interrupt Status */
#define MV_GPIO_INT_STAT		0xF011

/* GPIO Input/Output data */
#define MV_GPIO_DATA			0xF012

/* GPIO Tristate control */
#define MV_GPIO_TRISTATE_CTRL		0xF013

/* GPIO Interrupt type 1 (GPIOs 0 - 3) */
#define MV_GPIO_INT_TYPE1		0xF014
#define MV_GPIO_INT_TYPE_PIN0		GENMASK(2, 0)
#define MV_GPIO_INT_NO_IRQ		0
#define MV_GPIO_INT_LEVEL_LOW		2
#define MV_GPIO_INT_LEVEL_HIGH		3
#define MV_GPIO_INT_EDGE_FALLING	4
#define MV_GPIO_INT_EDGE_RISING		5
#define MV_GPIO_INT_EDGE_BOTH		7
#define MV_GPIO_INT_TYPE_PIN1		GENMASK(6, 4)
#define MV_GPIO_FUNC_TX_FLT_PIN1	BIT(7)
#define MV_GPIO_INT_TYPE_PIN2		GENMASK(10, 8)
#define MV_GPIO_FUNC_RX_LOS_PIN2	BIT(11)
#define MV_GPIO_INT_TYPE_PIN3		GENMASK(14, 12)

/* GPIO Interrupt type 2 (GPIOs 4 - 7) */
#define MV_GPIO_INT_TYPE2		0xF015
#define MV_GPIO_INT_TYPE_PIN4		GENMASK(2, 0)
#define MV_GPIO_FUNC_LED0_PIN4		BIT(3)
#define MV_GPIO_INT_TYPE_PIN5		GENMASK(6, 4)
#define MV_GPIO_FUNC_LED1_PIN5		BIT(7)
#define MV_GPIO_INT_TYPE_PIN6		GENMASK(10, 8)
#define MV_GPIO_FUNC_MPC_PIN6		BIT(11)
#define MV_GPIO_INT_TYPE_PIN7		GENMASK(14, 12)
#define MV_GPIO_FUNC_TOD_PIN7		BIT(15)

/* GPIO Interrupt type 3 (GPIOs 8, 10, 11) */
#define MV_GPIO_INT_TYPE3		0xF016
#define MV_GPIO_INT_TYPE_PIN8		GENMASK(2, 0)
#define MV_GPIO_FUNC_TX_DIS_PIN8	GENMASK(4, 3)
#define MV_GPIO_FUNC_TX_DIS_LED		0
#define MV_GPIO_FUNC_TX_DIS_PIN		1
#define MV_GPIO_FUNC_TX_DIS_REG		2
#define MV_GPIO_INT_TYPE_PIN10		GENMASK(10, 8)
#define MV_GPIO_FUNC_SDA_PIN10		BIT(11)
#define MV_GPIO_INT_TYPE_PIN11		GENMASK(14, 12)
#define MV_GPIO_FUNC_SCL_PIN11		BIT(15)

/* GPIO Interrupt type 1-3 helpers */
#define MV_GPIO_INT_TYPE_REG(_pin)		\
	((_pin) / 4)
#define MV_GPIO_INT_TYPE_PREP(_pin, _val)	\
	((_val) << ((_pin) % 4) * 4)
#define MV_GPIO_INT_TYPE_MASK(_pin)		\
	MV_GPIO_INT_TYPE_PREP(_pin, GENMASK(2, 0))
#define MV_GPIO_FUNC_PREP(_pin, _val)		\
	((_val) << (3 + ((_pin) % 4) * 4))
#define MV_GPIO_FUNC_MASK(_pin)			\
	MV_GPIO_FUNC_PREP(_pin, (_pin) != 8 ? BIT(0) : GENMASK(1, 0))

/* Port Interrupt Status */
#define MV_PORT_INT_STAT	0xF040
#define MV_PORT_INT_PCS_LINE	BIT(0)
#define MV_PORT_INT_PCS_HOST	BIT(2)
#define MV_PORT_INT_GPIO	BIT(3)

/* PMD Receive Signal Detect */
#define	MV_RX_SIGNAL_DETECT		0x000A
#define	MV_RX_SIGNAL_DETECT_GLOBAL	BIT(0)

/* 1000Base-X/SGMII Control Register */
#define	MV_1GBX_CTRL		(0x2000 + MII_BMCR)

/* 1000BASE-X/SGMII Status Register */
#define	MV_1GBX_STAT		(0x2000 + MII_BMSR)

/* 1000Base-X Auto-Negotiation Advertisement Register */
#define	MV_1GBX_ADVERTISE	(0x2000 + MII_ADVERTISE)

/* Two Wire Interface Caching Control/Status Register */
#define MV_TWSI_CACHE_CTRL		0x8000
#define MV_TWSI_CACHE_A0_CTRL		GENMASK(1, 0)
#define MV_TWSI_CACHE_NO_AUTO		0
#define MV_TWSI_CACHE_AT_PLUGIN		1
#define MV_TWSI_CACHE_AT_PLUGIN_POLL	2
#define MV_TWSI_CACHE_MANUAL		3
#define MV_TWSI_CACHE_A0_CMD_STAT	GENMASK(3, 2)
#define MV_TWSI_CACHE_NO_UPDATE		0
#define MV_TWSI_CACHE_UPDATED_ONCE	1
#define MV_TWSI_CACHE_IS_LOADING	2
#define MV_TWSI_CACHE_FAILED		3
#define MV_TWSI_CACHE_A0_VALID		BIT(9)
#define MV_TWSI_RESET			BIT(10)
#define MV_TWSI_CACHE_A2_CTRL		GENMASK(12, 11)
#define MV_TWSI_CACHE_A2_CMD_STAT	GENMASK(14, 13)
#define MV_TWSI_CACHE_A2_VALID		BIT(15)

/* Two Wire Interface Memory Address Register */
#define MV_TWSI_MEM_ADDR		0x8001
#define MV_TWSI_BYTE_ADDR		GENMASK(7, 0)
#define MV_TWSI_BYTE_READ		BIT(8)
#define MV_TWSI_SLV_ADDR		GENMASK(15, 9)

/* Two Wire Interface Memory Read Data and Status Register */
#define MV_TWSI_MEM_READ_STAT		0x8002
#define MV_TWSI_MEM_READ_DATA		GENMASK(7, 0)
#define MV_TWSI_STAT			GENMASK(10, 8)
#define MV_TWSI_READY			0
#define MV_TWSI_CMD_DONE		1
#define MV_TWSI_CMD_IN_PROG		2
#define MV_TWSI_CMD_WDONE_RFAIL		3
#define MV_TWSI_CMD_FAIL		5
#define MV_TWSI_BUSY			7
#define MV_TWSI_CACHE_ECC_UNCOR_ERR	BIT(11)
#define MV_TWSI_CACHE_ECC_COR_ERR	BIT(12)

/* Two Wire Interface Memory Write Data and Control Register */
#define MV_TWSI_MEM_WRITE_CTRL		0x8003
#define MV_TWSI_MEM_WRITE_DATA		GENMASK(7, 0)
#define MV_TWSI_MEM_AUTO_RBACK		BIT(9)
#define MV_TWSI_MEM_WRITE_TIME		GENMASK(15, 12)

/* Two Wire Interface Caching Delay Register */
#define MV_TWSI_CACHE_DELAY		0x8004
#define MV_TWSI_CACHE_A2_ADDR_MSB	BIT(0)
#define MV_TWSI_CACHE_A2_SLV_ADDR	GENMASK(7, 1)
#define MV_TWSI_CACHE_RELOAD_FREQ	GENMASK(10, 9)
#define MV_TWSI_CACHE_RELOAD_250MS	0
#define MV_TWSI_CACHE_RELOAD_500MS	1
#define MV_TWSI_CACHE_RELOAD_1S		2
#define MV_TWSI_CACHE_RELOAD_2S		3
#define MV_TWSI_CACHE_ECC_UNCOR_INT_EN	BIT(11)
#define MV_TWSI_CACHE_ECC_COR_INT_EN	BIT(12)
#define MV_TWSI_CACHE_AUTO_DELAY	GENMASK(15, 13)
#define MV_TWSI_CACHE_NO_DELAY		0
#define MV_TWSI_CACHE_DELAY_250MS	1
#define MV_TWSI_CACHE_DELAY_500MS	2
#define MV_TWSI_CACHE_DELAY_1S		3
#define MV_TWSI_CACHE_DELAY_2S		4
#define MV_TWSI_CACHE_DELAY_4S		5
#define MV_TWSI_CACHE_DELAY_8S		6
#define MV_TWSI_CACHE_AUTO_DIS		7

/* Two Wire Interface EEPROM Cache Page A0 Registers */
#define MV_TWSI_CACHE_EEPROM_A0		0x8007

/* Two Wire Interface EEPROM Cache Page A2 Registers */
#define MV_TWSI_CACHE_EEPROM_A2		0x8087

/* 10GBase-R Interrupt Enable Register */
#define MV_10GBR_INT_EN			0x8000
#define MV_10GBR_INT_BLOCK_LOCK		BIT(0)
#define MV_10GBR_INT_HIGH_BER		BIT(1)
#define MV_10GBR_INT_LINK_STAT		BIT(2)
#define MV_10GBR_INT_LOCAL_RXFAULT	BIT(10)
#define MV_10GBR_INT_LOCAL_TXFAULT	BIT(11)
#define MV_10GBR_INT_MASK		(MV_10GBR_INT_LINK_STAT | \
					 MV_10GBR_INT_LOCAL_RXFAULT | \
					 MV_10GBR_INT_LOCAL_TXFAULT)

/* 10GBase-R Interrupt Status Register */
#define MV_10GBR_INT_STAT		0x8001

/* 1000Base-X Interrupt Enable Register */
#define MV_1GBX_INT_EN			0xA001
#define MV_1GBX_INT_FALSE_CARRIER	BIT(7)
#define MV_1GBX_INT_SYMBOL_ERROR	BIT(8)
#define MV_1GBX_INT_LINK_UP		BIT(9)
#define MV_1GBX_INT_LINK_DOWN		BIT(10)
#define MV_1GBX_INT_AN_COMPLETED	BIT(11)
#define MV_1GBX_INT_PAGE_RECEIVED	BIT(12)
#define MV_1GBX_INT_DUPLEX_CHANGED	BIT(13)
#define MV_1GBX_INT_SPEED_CHANGED	BIT(14)
#define MV_1GBX_INT_MASK		(MV_1GBX_INT_FALSE_CARRIER | \
					 MV_1GBX_INT_SYMBOL_ERROR | \
					 MV_1GBX_INT_LINK_UP | \
					 MV_1GBX_INT_LINK_DOWN | \
					 MV_1GBX_INT_AN_COMPLETED | \
					 MV_1GBX_INT_DUPLEX_CHANGED | \
					 MV_1GBX_INT_SPEED_CHANGED)

/* 1000Base-X Interrupt Status Register */
#define MV_1GBX_INT_STAT		0xA002

/* 1000Base-X PHY Specific Status Register */
#define	MV_1GBX_PHY_STAT		0xA003
#define	MV_1GBX_PHY_STAT_AN_RESOLVED	BIT(11)
#define	MV_1GBX_PHY_STAT_DUPLEX		BIT(13)
#define	MV_1GBX_PHY_STAT_SPEED100	BIT(14)
#define	MV_1GBX_PHY_STAT_SPEED1000	BIT(15)

/* Host-side 10GBase-R PCS Status 1 Register */
#define MV_HOST_10GBR_PCS_STAT1		(0x0 + MDIO_STAT1)

/* Host-side XAUI PCS Status 1 Register */
#define MV_HOST_XAUI_PCS_STAT1		(0x1000 + MDIO_STAT1)

/* Host-side 10GBase-R Interrupt Enable Register */
#define MV_HOST_10GBR_INT_EN		0x8000
#define MV_HOST_10GBR_INT_BLOCK_LOCK	BIT(0)
#define MV_HOST_10GBR_INT_HIGH_BER	BIT(1)
#define MV_HOST_10GBR_INT_LINK_STAT	BIT(2)
#define MV_HOST_10GBR_INT_LOCAL_RXFAULT	BIT(10)
#define MV_HOST_10GBR_INT_LOCAL_TXFAULT	BIT(11)
#define MV_HOST_10GBR_INT_MASK		(MV_HOST_10GBR_INT_LINK_STAT | \
					 MV_HOST_10GBR_INT_LOCAL_RXFAULT | \
					 MV_HOST_10GBR_INT_LOCAL_TXFAULT)

/* Host-side 10GBase-R Interrupt Status Register */
#define MV_HOST_10GBR_INT_STAT		0x8001

/* Host-side XAUI Interrupt Enable 1 Register */
#define MV_HOST_XAUI_INT_EN1		0x9001
#define MV_HOST_XAUI_INT_LINK_UP	BIT(2)
#define MV_HOST_XAUI_INT_LINK_DOWN	BIT(3)
#define MV_HOST_XAUI_INT_MASK1		(MV_HOST_XAUI_INT_LINK_UP | \
					 MV_HOST_XAUI_INT_LINK_DOWN)

/* Host-side XAUI Interrupt Enable 2 Register */
#define MV_HOST_XAUI_INT_EN2		0x9002
#define MV_HOST_XAUI_INT_LANE0_SYNC	BIT(0)
#define MV_HOST_XAUI_INT_LANE1_SYNC	BIT(1)
#define MV_HOST_XAUI_INT_LANE2_SYNC	BIT(2)
#define MV_HOST_XAUI_INT_LANE3_SYNC	BIT(3)
#define MV_HOST_XAUI_INT_LANE0_ENERGY	BIT(4)
#define MV_HOST_XAUI_INT_LANE1_ENERGY	BIT(5)
#define MV_HOST_XAUI_INT_LANE2_ENERGY	BIT(6)
#define MV_HOST_XAUI_INT_LANE3_ENERGY	BIT(7)
#define MV_HOST_XAUI_INT_TXFAULT	BIT(8)
#define MV_HOST_XAUI_INT_RXFAULT	BIT(9)
#define MV_HOST_XAUI_INT_MASK2		(MV_HOST_XAUI_INT_TXFAULT | \
					 MV_HOST_XAUI_INT_RXFAULT)

/* Host-side XAUI Interrupt Status 1 Register */
#define MV_HOST_XAUI_INT_STAT1		0x9003

/* Host-side XAUI Interrupt Status 2 Register */
#define MV_HOST_XAUI_INT_STAT2		0x9004

/* GPIO controller settings (#9 always invalid, #0 and #3 always available) */
#define MV_GPIO_NUM			12
#define MV_GPIO_VAL_MASK		0xDFF
#define MV_GPIO_INT_UPD_FLAG		BIT(31)

/* I2C controller settings */
#define MV_I2C_POLL_NUM			3

#define	AUTONEG_TIMEOUT	3

struct mv2222_data {
	phy_interface_t line_interface;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported);
	bool sfp_link;
};

#ifdef CONFIG_MARVELL_88X2222_GPIO
struct mv2222_gpio {
	struct gpio_chip gc;
	struct phy_device *phydev;
	unsigned int irq;
	u32 cache_val_mask;
	u32 cache_data;
	u32 cache_tri_ctrl;
	u32 cache_int_en;
	u32 cache_int_type[MV_GPIO_INT_TYPE_REG(MV_GPIO_NUM)];
	struct mutex cache_lock;
};
#endif

#ifdef CONFIG_MARVELL_88X2222_I2C
struct mv2222_i2c {
	struct i2c_adapter ia;
	struct phy_device *phydev;
};
#endif

/* SFI PMA transmit enable */
static int mv2222_tx_enable(struct phy_device *phydev)
{
	return phy_clear_bits_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_TXDIS,
				  MDIO_PMD_TXDIS_GLOBAL);
}

/* SFI PMA transmit disable */
static int mv2222_tx_disable(struct phy_device *phydev)
{
	return phy_set_bits_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_TXDIS,
				MDIO_PMD_TXDIS_GLOBAL);
}

static int __mv2222_soft_reset(struct phy_device *phydev, u16 mask)
{
	int val, ret;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_PORT_RST, mask);
	if (ret < 0)
		return ret;

	return phy_read_mmd_poll_timeout(phydev, MDIO_MMD_VEND2, MV_PORT_RST,
					 val, !(val & mask),
					 5000, 1000000, true);
}

static inline int mv2222_host_reset(struct phy_device *phydev)
{
	return __mv2222_soft_reset(phydev, MV_HOST_RST_SW);
}

static inline int mv2222_line_reset(struct phy_device *phydev)
{
	return __mv2222_soft_reset(phydev, MV_LINE_RST_SW);
}

static inline int mv2222_soft_reset(struct phy_device *phydev)
{
	return __mv2222_soft_reset(phydev, MV_PORT_RST_SW);
}

static int mv2222_disable_aneg(struct phy_device *phydev)
{
	int ret = phy_clear_bits_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_CTRL,
				     BMCR_ANENABLE | BMCR_ANRESTART);
	if (ret < 0)
		return ret;

	return mv2222_line_reset(phydev);
}

static int mv2222_enable_aneg(struct phy_device *phydev)
{
	int ret = phy_set_bits_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_CTRL,
				   BMCR_ANENABLE | BMCR_RESET);
	if (ret < 0)
		return ret;

	return mv2222_line_reset(phydev);
}

static int mv2222_set_sgmii_speed(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;

	switch (phydev->speed) {
	default:
	case SPEED_1000:
		if ((linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				       priv->supported) ||
		     linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				       priv->supported)))
			return phy_modify_mmd(phydev, MDIO_MMD_PCS,
					      MV_1GBX_CTRL,
					      BMCR_SPEED1000 | BMCR_SPEED100,
					      BMCR_SPEED1000);

		fallthrough;
	case SPEED_100:
		if ((linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
				       priv->supported) ||
		     linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
				       priv->supported)))
			return phy_modify_mmd(phydev, MDIO_MMD_PCS,
					      MV_1GBX_CTRL,
					      BMCR_SPEED1000 | BMCR_SPEED100,
					      BMCR_SPEED100);
		fallthrough;
	case SPEED_10:
		if ((linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
				       priv->supported) ||
		     linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
				       priv->supported)))
			return phy_modify_mmd(phydev, MDIO_MMD_PCS,
					      MV_1GBX_CTRL,
					      BMCR_SPEED1000 | BMCR_SPEED100,
					      BMCR_SPEED10);

		return -EINVAL;
	}
}

static bool mv2222_is_10g_capable(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;

	return (linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseCR_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseSR_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseLR_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseLRM_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseER_Full_BIT,
				  priv->supported));
}

static bool mv2222_is_1gbx_capable(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;

	return linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
				 priv->supported);
}

static bool mv2222_is_sgmii_capable(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;

	return (linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
				  priv->supported) ||
		linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
				  priv->supported));
}

static int mv2222_config_host(struct phy_device *phydev)
{
	u16 val;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_XAUI:
		val = FIELD_PREP(MV_PCS_HOST_PCS_SELECT, MV_PCS_HOST_XAUI);
		break;
	case PHY_INTERFACE_MODE_10GBASER:
		val = FIELD_PREP(MV_PCS_HOST_PCS_SELECT, MV_PCS_HOST_10GBR);
		break;
	default:
		return -EINVAL;
	}

	return phy_modify_mmd(phydev, MDIO_MMD_VEND2, MV_PCS_CONFIG,
			      MV_PCS_HOST_PCS_SELECT, val);
}

static int mv2222_config_line(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	u16 val;

	switch (priv->line_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		val = FIELD_PREP(MV_PCS_LINE_PCS_SELECT, MV_PCS_LINE_10GBR);
		break;
	case PHY_INTERFACE_MODE_1000BASEX:
		val = FIELD_PREP(MV_PCS_LINE_PCS_SELECT, MV_PCS_LINE_1GBX_AN);
		break;
	case PHY_INTERFACE_MODE_SGMII:
		val = FIELD_PREP(MV_PCS_LINE_PCS_SELECT, MV_PCS_LINE_SGMII_AN);
		break;
	default:
		return -EINVAL;
	}

	return phy_modify_mmd(phydev, MDIO_MMD_VEND2, MV_PCS_CONFIG,
			      MV_PCS_LINE_PCS_SELECT, val);
}

/* Switch between 1G (1000Base-X/SGMII) and 10G (10GBase-R) modes */
static int mv2222_swap_line_type(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	bool changed = false;
	int ret;

	switch (priv->line_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		if (mv2222_is_1gbx_capable(phydev)) {
			priv->line_interface = PHY_INTERFACE_MODE_1000BASEX;
			changed = true;
		}

		if (mv2222_is_sgmii_capable(phydev)) {
			priv->line_interface = PHY_INTERFACE_MODE_SGMII;
			changed = true;
		}

		break;
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
		if (mv2222_is_10g_capable(phydev)) {
			priv->line_interface = PHY_INTERFACE_MODE_10GBASER;
			changed = true;
		}

		break;
	default:
		return -EINVAL;
	}

	if (changed) {
		ret = mv2222_config_line(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int mv2222_setup_forced(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	int ret;

	if (priv->line_interface == PHY_INTERFACE_MODE_10GBASER) {
		if (phydev->speed < SPEED_10000 &&
		    phydev->speed != SPEED_UNKNOWN) {
			ret = mv2222_swap_line_type(phydev);
			if (ret < 0)
				return ret;
		}
	}

	if (priv->line_interface == PHY_INTERFACE_MODE_SGMII) {
		ret = mv2222_set_sgmii_speed(phydev);
		if (ret < 0)
			return ret;
	}

	return mv2222_disable_aneg(phydev);
}

static int mv2222_config_aneg(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	int ret, adv;

	/* SFP is not present, do nothing */
	if (priv->line_interface == PHY_INTERFACE_MODE_NA)
		return 0;

	if (phydev->autoneg == AUTONEG_DISABLE ||
	    priv->line_interface == PHY_INTERFACE_MODE_10GBASER)
		return mv2222_setup_forced(phydev);

	adv = linkmode_adv_to_mii_adv_x(priv->supported,
					ETHTOOL_LINK_MODE_1000baseX_Full_BIT);

	ret = phy_modify_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_ADVERTISE,
			     ADVERTISE_1000XFULL |
			     ADVERTISE_1000XPAUSE | ADVERTISE_1000XPSE_ASYM,
			     adv);
	if (ret < 0)
		return ret;

	return mv2222_enable_aneg(phydev);
}

/* The link state and some other fields in the status registers are latched
 * low/high so that the momentary events could be detected. Do not double-read
 * the status in polling mode to detect such a short flag changes except when
 * it's forced to be required (i.e. when the link was already down).
 */
static int mv2222_read_mmd_latched(struct phy_device *phydev, int devad, u32 reg,
				   bool force)
{
	int ret;

	if (!phy_polling_mode(phydev) || force) {
		ret = phy_read_mmd(phydev, devad, reg);
		if (ret < 0)
			return ret;
	}

	return phy_read_mmd(phydev, devad, reg);
}

static int mv2222_aneg_done(struct phy_device *phydev)
{
	int ret;

	if (mv2222_is_10g_capable(phydev)) {
		ret = mv2222_read_mmd_latched(phydev, MDIO_MMD_PCS, MDIO_STAT1,
					      !phydev->link);
		if (ret < 0)
			return ret;

		if (ret & MDIO_STAT1_LSTATUS)
			return 1;
	}

	ret = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_STAT);
	if (ret < 0)
		return ret;

	return (ret & BMSR_ANEGCOMPLETE);
}

/* Returns negative on error, 0 if link is down, 1 if link is up */
static int mv2222_read_status_10g(struct phy_device *phydev)
{
	static int timeout;
	int val, link = 0;

	val = mv2222_read_mmd_latched(phydev, MDIO_MMD_PCS, MDIO_STAT1,
				      !phydev->link);
	if (val < 0)
		return val;

	if (val & MDIO_STAT1_LSTATUS) {
		link = 1;

		/* 10GBASE-R do not support auto-negotiation */
		phydev->autoneg = AUTONEG_DISABLE;
		phydev->speed = SPEED_10000;
		phydev->duplex = DUPLEX_FULL;
	} else {
		if (phydev->autoneg == AUTONEG_ENABLE) {
			timeout++;

			if (timeout > AUTONEG_TIMEOUT) {
				timeout = 0;

				val = mv2222_swap_line_type(phydev);
				if (val < 0)
					return val;

				return mv2222_config_aneg(phydev);
			}
		}
	}

	return link;
}

/* Returns negative on error, 0 if link is down, 1 if link is up */
static int mv2222_read_status_1g(struct phy_device *phydev)
{
	static int timeout;
	int val, link = 0;

	val = mv2222_read_mmd_latched(phydev, MDIO_MMD_PCS, MV_1GBX_STAT,
				      !phydev->link);
	if (val < 0)
		return val;

	if (phydev->autoneg == AUTONEG_ENABLE &&
	    !(val & BMSR_ANEGCOMPLETE)) {
		timeout++;

		if (timeout > AUTONEG_TIMEOUT) {
			timeout = 0;

			val = mv2222_swap_line_type(phydev);
			if (val < 0)
				return val;

			return mv2222_config_aneg(phydev);
		}

		return 0;
	}

	if (!(val & BMSR_LSTATUS))
		return 0;

	link = 1;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_PHY_STAT);
	if (val < 0)
		return val;

	if (val & MV_1GBX_PHY_STAT_AN_RESOLVED) {
		if (val & MV_1GBX_PHY_STAT_DUPLEX)
			phydev->duplex = DUPLEX_FULL;
		else
			phydev->duplex = DUPLEX_HALF;

		if (val & MV_1GBX_PHY_STAT_SPEED1000)
			phydev->speed = SPEED_1000;
		else if (val & MV_1GBX_PHY_STAT_SPEED100)
			phydev->speed = SPEED_100;
		else
			phydev->speed = SPEED_10;
	}

	return link;
}

static bool mv2222_iface_is_operational(struct phy_device *phydev)
{
	int reg, val;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_XAUI:
		reg = MV_HOST_XAUI_PCS_STAT1;
		break;
	case PHY_INTERFACE_MODE_10GBASER:
		reg = MV_HOST_10GBR_PCS_STAT1;
		break;
	default:
		return false;
	}

	val = mv2222_read_mmd_latched(phydev, MDIO_MMD_PHYXS, reg,
				      !phydev->link);
	if (val < 0 || !(val & MDIO_STAT1_LSTATUS))
		return false;

	return true;
}

static bool mv2222_link_is_operational(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MV_RX_SIGNAL_DETECT);
	if (val < 0 || !(val & MV_RX_SIGNAL_DETECT_GLOBAL))
		return false;

	if (phydev->sfp_bus && !priv->sfp_link)
		return false;

	return true;
}

static int mv2222_read_status(struct phy_device *phydev)
{
	struct mv2222_data *priv = phydev->priv;
	int link;

	phydev->link = 0;
	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;

	if (!mv2222_iface_is_operational(phydev)) {
		phydev_dbg(phydev, "Host interface isn't operational\n");
		return 0;
	}

	if (!mv2222_link_is_operational(phydev)) {
		phydev_dbg(phydev, "Line side isn't operational\n");
		return 0;
	}

	if (priv->line_interface == PHY_INTERFACE_MODE_10GBASER)
		link = mv2222_read_status_10g(phydev);
	else
		link = mv2222_read_status_1g(phydev);

	if (link < 0)
		return link;

	phydev->link = link;

	return 0;
}

static int mv2222_config_intr_host_10gbr(struct phy_device *phydev)
{
	int ret;

	if (phydev->interface == PHY_INTERFACE_MODE_10GBASER &&
	    phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_10GBR_INT_STAT);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_10GBR_INT_EN,
				    MV_HOST_10GBR_INT_MASK);
	} else {
		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_10GBR_INT_EN, 0);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_10GBR_INT_STAT);
	}

	return ret;
}

static int mv2222_config_intr_host_xaui(struct phy_device *phydev)
{
	int ret;

	if (phydev->interface == PHY_INTERFACE_MODE_XAUI &&
	    phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT1);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT2);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_EN1,
				    MV_HOST_XAUI_INT_MASK1);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_EN2,
				    MV_HOST_XAUI_INT_MASK2);
	} else {
		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_EN2, 0);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_EN1, 0);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT2);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT1);
	}

	return ret;
}

static int mv2222_config_intr_10g(struct phy_device *phydev)
{
	int ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		ret = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_10GBR_INT_STAT);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MV_10GBR_INT_EN,
				    MV_10GBR_INT_MASK);
	} else {
		ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MV_10GBR_INT_EN, 0);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_10GBR_INT_STAT);
	}

	return ret;
}

static int mv2222_config_intr_1g(struct phy_device *phydev)
{
	int ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		ret = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_INT_STAT);
		if (ret < 0)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_INT_EN,
				    MV_1GBX_INT_MASK);
	} else {
		ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_INT_EN, 0);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_INT_STAT);
	}

	return ret;
}

static int mv2222_config_intr(struct phy_device *phydev)
{
	int ret;

	ret = mv2222_config_intr_10g(phydev);
	if (ret < 0)
		return ret;

	ret = mv2222_config_intr_1g(phydev);
	if (ret < 0)
		return ret;

	ret = mv2222_config_intr_host_10gbr(phydev);
	if (ret < 0)
		return ret;

	ret = mv2222_config_intr_host_xaui(phydev);
	if (ret < 0)
		return ret;

	return 0;
}

static int mv2222_handle_interrupt_host_10gbr(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_10GBR_INT_STAT);
	if (val < 0)
		return val;

	if (val & MV_HOST_10GBR_INT_LINK_STAT)
		phydev_dbg(phydev, "Host link status changed\n");

	if (val & MV_HOST_10GBR_INT_LOCAL_RXFAULT)
		phydev_dbg(phydev, "Host Rx fault detected\n");

	if (val & MV_HOST_10GBR_INT_LOCAL_TXFAULT)
		phydev_dbg(phydev, "Host Tx fault detected\n");

	return 0;
}

static int mv2222_handle_interrupt_host_xaui(struct phy_device *phydev)
{
	int val1, val2;

	val1 = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT1);
	if (val1 < 0)
		return val1;

	val2 = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_HOST_XAUI_INT_STAT2);
	if (val2 < 0)
		return val2;

	if (val1 & MV_HOST_XAUI_INT_LINK_UP)
		phydev_dbg(phydev, "Host link is up\n");

	if (val1 & MV_HOST_XAUI_INT_LINK_DOWN)
		phydev_dbg(phydev, "Host link is down\n");

	if (val2 & MV_HOST_XAUI_INT_TXFAULT)
		phydev_dbg(phydev, "Host Tx fault detected\n");

	if (val2 & MV_HOST_XAUI_INT_RXFAULT)
		phydev_dbg(phydev, "Host Rx fault detected\n");

	return 0;
}

static int mv2222_handle_interrupt_10g(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_10GBR_INT_STAT);
	if (val < 0)
		return val;

	if (val & MV_10GBR_INT_LINK_STAT)
		phydev_dbg(phydev, "Line link status changed\n");

	if (val & MV_10GBR_INT_LOCAL_RXFAULT)
		phydev_dbg(phydev, "Line Rx fault detected\n");

	if (val & MV_10GBR_INT_LOCAL_TXFAULT)
		phydev_dbg(phydev, "Line Tx fault detected\n");

	return 0;
}

static int mv2222_handle_interrupt_1g(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_1GBX_INT_STAT);
	if (val < 0)
		return val;

	if (val & MV_1GBX_INT_FALSE_CARRIER)
		phydev_dbg(phydev, "Line false carrier detected\n");

	if (val & MV_1GBX_INT_SYMBOL_ERROR)
		phydev_dbg(phydev, "Line symbol error detected\n");

	if (val & MV_1GBX_INT_LINK_UP)
		phydev_dbg(phydev, "Line link is up\n");

	if (val & MV_1GBX_INT_LINK_DOWN)
		phydev_dbg(phydev, "Line link is down\n");

	if (val & MV_1GBX_INT_AN_COMPLETED)
		phydev_dbg(phydev, "Line AN is completed\n");

	if (val & MV_1GBX_INT_DUPLEX_CHANGED)
		phydev_dbg(phydev, "Line link duplex changed\n");

	if (val & MV_1GBX_INT_SPEED_CHANGED)
		phydev_dbg(phydev, "Line link speed changed\n");

	return 0;
}

static irqreturn_t mv2222_handle_interrupt(struct phy_device *phydev)
{
	int val, ret;

	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_PORT_INT_STAT);
	if (val < 0)
		goto err_set_state;

	if (!(val & (MV_PORT_INT_PCS_HOST | MV_PORT_INT_PCS_LINE)))
		return IRQ_NONE;

	if (val & MV_PORT_INT_PCS_HOST) {
		ret = mv2222_handle_interrupt_host_10gbr(phydev);
		if (ret < 0)
			goto err_set_state;

		ret = mv2222_handle_interrupt_host_xaui(phydev);
		if (ret < 0)
			goto err_set_state;
	}

	if (val & MV_PORT_INT_PCS_LINE) {
		ret = mv2222_handle_interrupt_10g(phydev);
		if (ret < 0)
			goto err_set_state;

		ret = mv2222_handle_interrupt_1g(phydev);
		if (ret < 0)
			goto err_set_state;
	}

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;

err_set_state:
	phy_error(phydev);

	return IRQ_NONE;
}

static int mv2222_resume(struct phy_device *phydev)
{
	return mv2222_tx_enable(phydev);
}

static int mv2222_suspend(struct phy_device *phydev)
{
	return mv2222_tx_disable(phydev);
}

static int mv2222_get_features(struct phy_device *phydev)
{
	/* All supported linkmodes are set at probe */

	return 0;
}

static int mv2222_config_init(struct phy_device *phydev)
{
	int ret;

	ret = mv2222_config_host(phydev);
	if (ret)
		return ret;

	return mv2222_host_reset(phydev);
}

#ifdef CONFIG_MARVELL_88X2222_GPIO
/*
 * Marvell 88x2222 GPIO control registers access is implemented by means of the
 * private data caches. It has several reasons. First of all the IRQ chip
 * implementation requires the IRQ-related settings applied in a non-atomic
 * context for the slow asynchronously accessed bus. Thus caching the IRQ-masks
 * and types is neccessary in order to flush the respective CSRs data in the
 * may-sleep context. Second the implementation decreases a bus traffic which
 * improves the accesses performance espcially in case of the bit-banged MDIO
 * bus. Finally it prevents a race condition in the output value setup on the
 * output GPIO-mode settings. The GPIO data register has a multiplexed input
 * output data semantics. Any read from the CSRs returns either the input GPIO
 * state or the output GPIO value. Meanwhile any write to the data register
 * updates the output GPIO value. So the race condition may happen if the
 * output value is updated before the output GPIO-mode is activated and if
 * there is a parallel read-modify-write is performed to the GPIO data
 * register. In that case the input GPIO state will be read and written back as
 * the output GPIO value. Using the private data cache prevents that.
 */

static inline bool mv2222_gpio_offset_is_valid(unsigned int ofs)
{
	return ofs < MV_GPIO_NUM && ofs != 9;
}

static int mv2222_gpio_read_tri_ctrl(struct mv2222_gpio *gpio)
{
	int val;

	mutex_lock(&gpio->cache_lock);

	val = phy_read_mmd(gpio->phydev, MDIO_MMD_VEND2, MV_GPIO_TRISTATE_CTRL);
	if (val < 0)
		goto err_mutex_unlock;

	gpio->cache_tri_ctrl = val;

err_mutex_unlock:
	mutex_unlock(&gpio->cache_lock);

	return val;
}

static int mv2222_gpio_wthru_tri_ctrl(struct mv2222_gpio *gpio, u16 mask, u16 val)
{
	int ret;

	mutex_lock(&gpio->cache_lock);

	gpio->cache_tri_ctrl &= ~mask;
	gpio->cache_tri_ctrl |= val;

	ret = phy_write_mmd(gpio->phydev, MDIO_MMD_VEND2,
			    MV_GPIO_TRISTATE_CTRL, gpio->cache_tri_ctrl);

	mutex_unlock(&gpio->cache_lock);

	return ret;
}

static int mv2222_gpio_read_data(struct mv2222_gpio *gpio)
{
	int val;

	mutex_lock(&gpio->cache_lock);

	val = phy_read_mmd(gpio->phydev, MDIO_MMD_VEND2, MV_GPIO_DATA);
	if (val < 0)
		goto err_mutex_unlock;

	gpio->cache_data = val;

err_mutex_unlock:
	mutex_unlock(&gpio->cache_lock);

	return val;
}

static int mv2222_gpio_wthru_data(struct mv2222_gpio *gpio, u16 mask, u16 val)
{
	int ret;

	mutex_lock(&gpio->cache_lock);

	gpio->cache_data &= ~mask;
	gpio->cache_data |= val;

	ret = phy_write_mmd(gpio->phydev, MDIO_MMD_VEND2,
			    MV_GPIO_DATA, gpio->cache_data);

	mutex_unlock(&gpio->cache_lock);

	return ret;
}

static int mv2222_gpio_cache_int_en(struct mv2222_gpio *gpio,
				    unsigned int ofs, bool int_en)
{
	if (!mv2222_gpio_offset_is_valid(ofs))
		return -EINVAL;

	if (int_en)
		gpio->cache_int_en |= BIT(ofs);
	else
		gpio->cache_int_en &= ~BIT(ofs);

	gpio->cache_int_en |= MV_GPIO_INT_UPD_FLAG;

	return 0;
}

static int mv2222_gpio_cache_func(struct mv2222_gpio *gpio,
				  unsigned int ofs, bool gpio_en)
{
	u16 val;
	int reg;

	if (!mv2222_gpio_offset_is_valid(ofs))
		return -EINVAL;

	/* Pins #0 and #3 always work as GPIOs */
	if (ofs == 0 || ofs == 3)
		return !gpio_en ? -EINVAL : 0;

	if (ofs == 8 && !gpio_en)
		val = MV_GPIO_FUNC_TX_DIS_REG;
	else
		val = !!gpio_en;

	reg = MV_GPIO_INT_TYPE_REG(ofs);
	gpio->cache_int_type[reg] &= ~MV_GPIO_FUNC_MASK(ofs);
	gpio->cache_int_type[reg] |= MV_GPIO_FUNC_PREP(ofs, val);

	gpio->cache_int_type[reg] |= MV_GPIO_INT_UPD_FLAG;

	return 0;
}

static int mv2222_gpio_cache_int_type(struct mv2222_gpio *gpio,
				      unsigned int ofs, unsigned int type)
{
	u16 val;
	int reg;

	if (!mv2222_gpio_offset_is_valid(ofs))
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_NONE:
		val = MV_GPIO_INT_NO_IRQ;
		break;
	case IRQ_TYPE_EDGE_RISING:
		val = MV_GPIO_INT_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val = MV_GPIO_INT_EDGE_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		val = MV_GPIO_INT_EDGE_BOTH;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		val = MV_GPIO_INT_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		val = MV_GPIO_INT_LEVEL_LOW;
		break;
	default:
		return -EINVAL;
	}

	reg = MV_GPIO_INT_TYPE_REG(ofs);
	gpio->cache_int_type[reg] &= ~MV_GPIO_INT_TYPE_MASK(ofs);
	gpio->cache_int_type[reg] |= MV_GPIO_INT_TYPE_PREP(ofs, val);

	gpio->cache_int_type[reg] |= MV_GPIO_INT_UPD_FLAG;

	return 0;
}

static int mv2222_gpio_cache_int_flush(struct mv2222_gpio *gpio)
{
	int i, ret;

	if (gpio->cache_int_en & MV_GPIO_INT_UPD_FLAG) {
		ret = phy_write_mmd(gpio->phydev, MDIO_MMD_VEND2,
				    MV_GPIO_INT_EN, gpio->cache_int_en);
		if (ret < 0)
			return ret;

		gpio->cache_int_en &= ~MV_GPIO_INT_UPD_FLAG;
	}

	for (i = 0; i < ARRAY_SIZE(gpio->cache_int_type); ++i) {
		if (!(gpio->cache_int_type[i] & MV_GPIO_INT_UPD_FLAG))
			continue;

		ret = phy_write_mmd(gpio->phydev, MDIO_MMD_VEND2,
				    MV_GPIO_INT_TYPE1 + i, gpio->cache_int_type[i]);
		if (ret < 0)
			return ret;

		gpio->cache_int_type[i] &= ~MV_GPIO_INT_UPD_FLAG;
	}

	return 0;
}

static int mv2222_gpio_init(struct mv2222_gpio *gpio)
{
	int i, ret;

	/* Setup GPIO function and default IRQs state for all valid GPIOs */
	for (i = 0; i < MV_GPIO_NUM; ++i) {
		mv2222_gpio_cache_int_en(gpio, i, false);

		if (gpio->cache_val_mask & BIT(i))
			mv2222_gpio_cache_func(gpio, i, true);
		else
			mv2222_gpio_cache_func(gpio, i, false);

		mv2222_gpio_cache_int_type(gpio, i, IRQ_TYPE_NONE);
	}

	ret = mv2222_gpio_cache_int_flush(gpio);
	if (ret < 0)
		return ret;

	ret = mv2222_gpio_read_tri_ctrl(gpio);
	if (ret < 0)
		return ret;

	ret = mv2222_gpio_read_data(gpio);
	if (ret < 0)
		return ret;

	return 0;
}

static int mv2222_gpio_get_direction(struct gpio_chip *gc, unsigned int ofs)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	int ret;

	mutex_lock(&gpio->cache_lock);
	ret = !(gpio->cache_tri_ctrl & BIT(ofs));
	mutex_unlock(&gpio->cache_lock);

	return ret;
}

static int mv2222_gpio_direction_input(struct gpio_chip *gc, unsigned int ofs)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	u16 mask = BIT(ofs);

	return mv2222_gpio_wthru_tri_ctrl(gpio, mask, 0);
}

static int mv2222_gpio_direction_output(struct gpio_chip *gc,
					unsigned int ofs, int val)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	u16 mask = BIT(ofs);
	int ret;

	ret = mv2222_gpio_wthru_data(gpio, mask, val ? mask : 0);
	if (ret < 0)
		return ret;

	return mv2222_gpio_wthru_tri_ctrl(gpio, mask, mask);
}

static int mv2222_gpio_get(struct gpio_chip *gc, unsigned int ofs)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	u16 mask = BIT(ofs);
	int val;

	val = mv2222_gpio_read_data(gpio);
	if (val < 0)
		return val;

	return !!(val & mask);
}

static int mv2222_gpio_get_multiple(struct gpio_chip *gc,
				    unsigned long *mask, unsigned long *bits)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	int val;

	val = mv2222_gpio_read_data(gpio);
	if (val < 0)
		return val;

	*bits &= ~*mask;
	*bits |= val & *mask;

	return 0;
}

static void mv2222_gpio_set(struct gpio_chip *gc, unsigned ofs, int val)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	u16 mask = BIT(ofs);
	int ret;

	ret = mv2222_gpio_wthru_data(gpio, mask, val ? mask : 0);
	if (ret < 0)
		phydev_err(gpio->phydev, "Failed to set GPIO %d\n", ofs);
}

static void mv2222_gpio_set_multiple(struct gpio_chip *gc,
				     unsigned long *mask, unsigned long *bits)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	int ret;

	ret = mv2222_gpio_wthru_data(gpio, *mask, *bits);
	if (ret < 0)
		phydev_err(gpio->phydev, "Failed to set GPIOs 0x%04lx\n", *bits);
}

static int mv2222_gpio_set_config(struct gpio_chip *gc, unsigned int ofs,
				  unsigned long cfg)
{
	enum pin_config_param mode = pinconf_to_config_param(cfg);

	/* All output pins operate as open drain */
	if (mode != PIN_CONFIG_DRIVE_OPEN_DRAIN)
		return -ENOTSUPP;

	return 0;
}

static int mv2222_gpio_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	int ret;

	if (ngpios > MV_GPIO_NUM)
		return -EINVAL;

	*valid_mask &= MV_GPIO_VAL_MASK;

	if (!acpi_disabled) {
		int mask;
		fwnode_property_read_u32(gc->parent->fwnode, "gpio-unreserved", &mask);
		*valid_mask &= mask;
	}

	gpio->cache_val_mask = *valid_mask;

	ret = mv2222_gpio_init(gpio);
	if (ret < 0)
		return ret;

	return 0;
}

static void mv2222_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	mv2222_gpio_cache_int_en(gpio, hwirq, false);
	gpiochip_disable_irq(gc, hwirq);
}

static void mv2222_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(gc, hwirq);
	mv2222_gpio_cache_int_en(gpio, hwirq, true);
}

static int mv2222_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	return mv2222_gpio_cache_int_type(gpio, hwirq, type);
}

static int mv2222_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);

	return irq_set_irq_wake(gpio->irq, on);
}

static void mv2222_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);

	mutex_lock(&gpio->cache_lock);
}

static void mv2222_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mv2222_gpio *gpio = gpiochip_get_data(gc);
	int ret;

	ret = mv2222_gpio_cache_int_flush(gpio);
	if (ret < 0)
		phydev_err(gpio->phydev, "Failed to flush GPIO IRQs state\n");

	mutex_unlock(&gpio->cache_lock);
}

static void mv2222_gpio_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	seq_printf(p, dev_name(gc->parent));
}

static const struct irq_chip mv2222_gpio_irq_chip = {
	.name 			= "mv88x2222",
	.irq_mask 		= mv2222_gpio_irq_mask,
	.irq_unmask 		= mv2222_gpio_irq_unmask,
	.irq_set_wake 		= mv2222_gpio_irq_set_wake,
	.irq_set_type 		= mv2222_gpio_irq_set_type,
	.irq_bus_lock 		= mv2222_gpio_irq_bus_lock,
	.irq_bus_sync_unlock 	= mv2222_gpio_irq_bus_sync_unlock,
	.irq_print_chip 	= mv2222_gpio_irq_print_chip,
	.flags 			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static irqreturn_t mv2222_gpio_handle_interrupt(int irq, void *devid)
{
	struct mv2222_gpio *gpio = devid;
	unsigned long pending;
	int i, val;

	val = phy_read_mmd(gpio->phydev, MDIO_MMD_VEND2, MV_GPIO_INT_STAT);
	if (val < 0)
		return IRQ_NONE;

	/* The interrupt status register exports the raw IRQ status */
	mutex_lock(&gpio->cache_lock);
	pending = val & gpio->cache_int_en;
	mutex_unlock(&gpio->cache_lock);

	for_each_set_bit(i, &pending, gpio->gc.ngpio)
		handle_nested_irq(irq_find_mapping(gpio->gc.irq.domain, i));

	return IRQ_RETVAL(pending);
}

static const char * const mv2222_gpio_names[MV_GPIO_NUM] = {
	[MV_88X2222_MOD_ABS]	= "MOD_ABS",
	[MV_88X2222_TX_FAULT]	= "TX_FAULT",
	[MV_88X2222_RX_LOS]	= "RX_LOS",
	[MV_88X2222_GPIO]	= "GPIO",
	[MV_88X2222_LED0]	= "LED0",
	[MV_88X2222_LED1]	= "LED1",
	[MV_88X2222_MPC]	= "MPC",
	[MV_88X2222_TOD]	= "TOD",
	[MV_88X2222_TX_DISABLE]	= "TX_DISABLE",
	[MV_88X2222_UNDEF]	= NULL,
	[MV_88X2222_SDA]	= "SDA",
	[MV_88X2222_SCL]	= "SCL",
};

static int mv2222_gpio_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mv2222_gpio *gpio;
	struct gpio_chip *gc;
	int ret;

	/*
	 * No GPIO-chip registration if the PHY-device isn't marked as a
	 * GPIO-controller. This is another level of protection for the
	 * backward compatibility in case if the platforms rely on the
	 * default/pre-initialized pins functions.
	 */
	if (!device_property_present(dev, "gpio-controller"))
		return 0;

	/*
	 * Marvell 88x2222 GPIO CSRs are tolerant to the soft-resets. They are
	 * marked as "Retain" in the "SW Rst" calumn of the registers
	 * description table defined in the HW-databook. On the contrary the
	 * hard-reset will reset all the GPIO CSRs to their default states which
	 * in its turn will break the driver GPIO functionality for sure.
	 */
	if (phydev->mdio.reset_gpio || phydev->mdio.reset_ctrl) {
		phydev_warn(phydev, "Hard-reset detected, GPIOs unsupported\n");
		return 0;
	}

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->phydev = phydev;

	mutex_init(&gpio->cache_lock);

	gc = &gpio->gc;
	gc->label = "mv88x2222";
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->get_direction = mv2222_gpio_get_direction;
	gc->direction_input = mv2222_gpio_direction_input;
	gc->direction_output = mv2222_gpio_direction_output;
	gc->get = mv2222_gpio_get;
	gc->get_multiple = mv2222_gpio_get_multiple;
	gc->set = mv2222_gpio_set;
	gc->set_multiple = mv2222_gpio_set_multiple;
	gc->set_config = mv2222_gpio_set_config;
	gc->init_valid_mask = mv2222_gpio_init_valid_mask;
	gc->base = -1;
	gc->ngpio = MV_GPIO_NUM;
	gc->names = mv2222_gpio_names;
	gc->can_sleep = true;

	if (phy_interrupt_is_valid(phydev)) {
		struct gpio_irq_chip *girq = &gc->irq;

		gpio->irq = phydev->irq;

		girq->handler = handle_bad_irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->threaded = true;

		gpio_irq_chip_set_chip(girq, &mv2222_gpio_irq_chip);

		ret = devm_request_threaded_irq(dev, gpio->irq, NULL,
						mv2222_gpio_handle_interrupt,
						IRQF_ONESHOT | IRQF_SHARED,
						phydev_name(phydev), gpio);
		if (ret) {
			phydev_err(phydev, "Failed to request GPIO IRQ\n");
			return ret;
		}
	} else {
		gpio->irq = IRQ_NOTCONNECTED;
	}

	ret = devm_gpiochip_add_data(dev, gc, gpio);
	if (ret)
		phydev_err(phydev, "Failed to register GPIO chip\n");

	return ret;
}
#else
static int mv2222_gpio_probe(struct phy_device *phydev)
{
	return 0;
}
#endif /* !CONFIG_MARVELL_88X2222_GPIO */

#ifdef CONFIG_MARVELL_88X2222_I2C
static int mv2222_i2c_init(struct mv2222_i2c *i2c)
{
	struct i2c_timings t;
	u16 val;
	int ret;

	/* The I2C bus is available if the SDA/SCL pins function isn't GPIO */
	ret = phy_read_mmd(i2c->phydev, MDIO_MMD_VEND2, MV_GPIO_INT_TYPE3);
	if (ret < 0)
		return ret;

	if (ret & MV_GPIO_FUNC_SDA_PIN10 || ret & MV_GPIO_FUNC_SCL_PIN11)
		return -EBUSY;

	/* Make sure the only supported bus speed is specified */
	i2c_parse_fw_timings(&i2c->phydev->mdio.dev, &t, true);
	if (t.bus_freq_hz != I2C_MAX_STANDARD_MODE_FREQ)
		return -EINVAL;

	/* Disable the EEPROM caching. It will interfere the I2C-adapter work */
	val = FIELD_PREP(MV_TWSI_CACHE_A0_CTRL, MV_TWSI_CACHE_NO_AUTO) |
	      FIELD_PREP(MV_TWSI_CACHE_A2_CTRL, MV_TWSI_CACHE_NO_AUTO);

	ret = phy_write_mmd(i2c->phydev, MDIO_MMD_PMAPMD, MV_TWSI_CACHE_CTRL, val);
	if (ret < 0)
		return ret;

	/* Disable the Read-back-after-write functionality */
	ret = phy_write_mmd(i2c->phydev, MDIO_MMD_PMAPMD, MV_TWSI_MEM_WRITE_CTRL, 0);
	if (ret < 0)
		return ret;

	/* Once again disable the auto-caching */
	val = FIELD_PREP(MV_TWSI_CACHE_AUTO_DELAY, MV_TWSI_CACHE_AUTO_DIS);

	ret = phy_write_mmd(i2c->phydev, MDIO_MMD_PMAPMD, MV_TWSI_CACHE_DELAY, val);
	if (ret < 0)
		return ret;

	return 0;
}

static inline unsigned long mv2222_i2c_get_udelay(char read_write)
{
	const unsigned long p = USEC_PER_SEC / I2C_MAX_STANDARD_MODE_FREQ;
	unsigned long ud;

	/* S, addr, Ack, cmd, Ack, data, Ack/Nack and P bits together */
	ud = 29 * p;

	/* Additional Sr, addr and Ack bits in case of the read op */
	if (read_write == I2C_SMBUS_READ)
		ud += 10 * p;

	return ud;
}

static inline bool mv2222_i2c_ready(int val)
{
	if (FIELD_GET(MV_TWSI_STAT, val) == MV_TWSI_READY)
		return true;

	return false;
}

static inline bool mv2222_i2c_cmd_done(int val)
{
	if (FIELD_GET(MV_TWSI_STAT, val) != MV_TWSI_CMD_IN_PROG)
		return true;

	return false;
}

static int mv2222_i2c_xfer(struct i2c_adapter *ia,
			   u16 addr, unsigned short flags, char read_write,
			   u8 cmd, int size, union i2c_smbus_data *data)
{
	struct mv2222_i2c *i2c = ia->algo_data;
	unsigned long ud;
	int val, ret;

	if (flags & I2C_CLIENT_TEN || size != I2C_SMBUS_BYTE_DATA)
		return -EOPNOTSUPP;

	/* Make sure the interface is ready to execute the command */
	ud = mv2222_i2c_get_udelay(I2C_SMBUS_READ);

	ret = phy_read_mmd_poll_timeout(i2c->phydev, MDIO_MMD_PMAPMD,
					MV_TWSI_MEM_READ_STAT, val,
					mv2222_i2c_ready(val),
					ud, MV_I2C_POLL_NUM * ud, false);
	if (ret < 0)
		return ret;

	/* Collect the command parameters */
	if (read_write == I2C_SMBUS_WRITE) {
		ret = phy_write_mmd(i2c->phydev, MDIO_MMD_PMAPMD,
				    MV_TWSI_MEM_WRITE_CTRL, data->byte);
		if (ret < 0)
			return ret;

		val = 0;
	} else {
		val = MV_TWSI_BYTE_READ;
	}

	val |= FIELD_PREP(MV_TWSI_BYTE_ADDR, cmd) |
	       FIELD_PREP(MV_TWSI_SLV_ADDR, addr);

	ret = phy_write_mmd(i2c->phydev, MDIO_MMD_PMAPMD, MV_TWSI_MEM_ADDR, val);
	if (ret < 0)
		return ret;

	/* Issue the command and wait until the CMD in-progress status is set */
	ud = mv2222_i2c_get_udelay(read_write);

	ret = phy_read_mmd_poll_timeout(i2c->phydev, MDIO_MMD_PMAPMD,
					MV_TWSI_MEM_READ_STAT, val,
					mv2222_i2c_cmd_done(val),
					ud, MV_I2C_POLL_NUM * ud, true);
	if (ret < 0)
		return ret;

	/* The interface may respond with the READY status on the early stage
	 * of the SFP-module attachement. In that case ask to try again.
	 */
	switch (FIELD_GET(MV_TWSI_STAT, val)) {
	case MV_TWSI_CMD_FAIL:
		return -EIO;
	case MV_TWSI_BUSY:
		return -ENXIO;
	case MV_TWSI_READY:
		return -EAGAIN;
	}

	if (read_write == I2C_SMBUS_READ)
		data->byte = FIELD_GET(MV_TWSI_MEM_READ_DATA, val);

	return 0;
}

static u32 mv2222_i2c_func(struct i2c_adapter *ia)
{
	return I2C_FUNC_SMBUS_BYTE_DATA;
}

static struct i2c_algorithm mv2222_i2c_algo = {
	.smbus_xfer = mv2222_i2c_xfer,
	.functionality = mv2222_i2c_func,
};

static int mv2222_i2c_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mv2222_i2c *i2c;
	struct i2c_adapter *ia;
	int ret;

	/*
	 * Marvell 88x2222 I2C CSRs are tolerant to the soft-resets. Alas that
	 * can be said about the hard-resets. It will halt any communications
	 * with possibly no error reported. So it isn't safe to use the
	 * interface if a sudden hard-reset might be performed.
	 */
	if (phydev->mdio.reset_gpio || phydev->mdio.reset_ctrl) {
		phydev_warn(phydev, "Hard-reset detected, I2C unsupported\n");
		return 0;
	}

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->phydev = phydev;

	/* Make sure the I2C-interface is ready before to proceed */
	ret = mv2222_i2c_init(i2c);
	if (ret == -EBUSY) {
		phydev_warn(phydev, "I2C interface unavailable\n");
		return 0;
	} else if (ret) {
		phydev_err(phydev, "Failed to init I2C-interface\n");
		return ret;
	}

	ia = &i2c->ia;
	strscpy(ia->name, "mv88x2222", sizeof(ia->name));
	ia->dev.parent = dev;
	ia->owner = THIS_MODULE;
	ia->algo = &mv2222_i2c_algo;
	ia->algo_data = i2c;

	ret = devm_i2c_add_adapter(dev, ia);
	if (ret)
		phydev_err(phydev, "Failed to register I2C adapter\n");

	return ret;
}
#else
static int mv2222_i2c_probe(struct phy_device *phydev)
{
	return 0;
}
#endif /* !CONFIG_MARVELL_88X2222_I2C */

static int mv2222_sfp_insert(void *upstream, const struct sfp_eeprom_id *id)
{
	DECLARE_PHY_INTERFACE_MASK(interfaces);
	struct phy_device *phydev = upstream;
	phy_interface_t sfp_interface;
	struct mv2222_data *priv;
	int ret;

	__ETHTOOL_DECLARE_LINK_MODE_MASK(sfp_supported) = { 0, };

	priv = phydev->priv;

	sfp_parse_support(phydev->sfp_bus, id, sfp_supported, interfaces);
	phydev->port = sfp_parse_port(phydev->sfp_bus, id, sfp_supported);
	sfp_interface = sfp_select_interface(phydev->sfp_bus, sfp_supported);

	phydev_info(phydev, "%s SFP module inserted\n", phy_modes(sfp_interface));

	if (sfp_interface != PHY_INTERFACE_MODE_10GBASER &&
	    sfp_interface != PHY_INTERFACE_MODE_1000BASEX &&
	    sfp_interface != PHY_INTERFACE_MODE_SGMII) {
		phydev_err(phydev, "Incompatible SFP module inserted\n");
		return -EINVAL;
	}

	priv->line_interface = sfp_interface;
	linkmode_and(priv->supported, phydev->supported, sfp_supported);

	ret = mv2222_config_line(phydev);
	if (ret < 0)
		return ret;

	if (mutex_trylock(&phydev->lock)) {
		ret = mv2222_config_aneg(phydev);
		mutex_unlock(&phydev->lock);
	}

	return ret;
}

static void mv2222_sfp_remove(void *upstream)
{
	struct phy_device *phydev = upstream;
	struct mv2222_data *priv;

	priv = phydev->priv;

	priv->line_interface = PHY_INTERFACE_MODE_NA;
	linkmode_zero(priv->supported);
	phydev->port = PORT_NONE;
}

static void mv2222_sfp_link_up(void *upstream)
{
	struct phy_device *phydev = upstream;
	struct mv2222_data *priv;

	priv = phydev->priv;
	priv->sfp_link = true;
}

static void mv2222_sfp_link_down(void *upstream)
{
	struct phy_device *phydev = upstream;
	struct mv2222_data *priv;

	priv = phydev->priv;
	priv->sfp_link = false;
}

static const struct sfp_upstream_ops sfp_phy_ops = {
	.module_insert = mv2222_sfp_insert,
	.module_remove = mv2222_sfp_remove,
	.link_up = mv2222_sfp_link_up,
	.link_down = mv2222_sfp_link_down,
	.attach = phy_sfp_attach,
	.detach = phy_sfp_detach,
	.connect_phy = phy_sfp_connect_phy,
	.disconnect_phy = phy_sfp_disconnect_phy,
};

static int mv2222_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mv2222_data *priv = NULL;
	int ret;

	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported) = { 0, };

	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_TP_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseCR_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseSR_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseLR_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseLRM_Full_BIT, supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseER_Full_BIT, supported);

	linkmode_copy(phydev->supported, supported);

	/* In perspective the list can be extended with 1000BASE-X and SGMII */
	__set_bit(PHY_INTERFACE_MODE_XAUI, phydev->possible_interfaces);
	__set_bit(PHY_INTERFACE_MODE_10GBASER, phydev->possible_interfaces);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->line_interface = PHY_INTERFACE_MODE_NA;
	phydev->priv = priv;

	ret = mv2222_gpio_probe(phydev);
	if (ret)
		return ret;

	ret = mv2222_i2c_probe(phydev);
	if (ret)
		return ret;

	return phy_sfp_probe(phydev, &sfp_phy_ops);
}

static struct phy_driver mv2222_drivers[] = {
	{
		.phy_id = MARVELL_PHY_ID_88X2222,
		.phy_id_mask = MARVELL_PHY_ID_MASK,
		.name = "Marvell 88X2222",
		.get_features = mv2222_get_features,
		.soft_reset = mv2222_soft_reset,
		.config_init = mv2222_config_init,
		.config_aneg = mv2222_config_aneg,
		.aneg_done = mv2222_aneg_done,
		.probe = mv2222_probe,
		.suspend = mv2222_suspend,
		.resume = mv2222_resume,
		.read_status = mv2222_read_status,
		.config_intr = mv2222_config_intr,
		.handle_interrupt = mv2222_handle_interrupt,
	},
	{
		.phy_id = MARVELL_PHY_ID_88X2222R,
		.phy_id_mask = MARVELL_PHY_ID_MASK,
		.name = "Marvell 88X2222R",
		.get_features = mv2222_get_features,
		.soft_reset = mv2222_soft_reset,
		.config_init = mv2222_config_init,
		.config_aneg = mv2222_config_aneg,
		.aneg_done = mv2222_aneg_done,
		.probe = mv2222_probe,
		.suspend = mv2222_suspend,
		.resume = mv2222_resume,
		.read_status = mv2222_read_status,
		.config_intr = mv2222_config_intr,
		.handle_interrupt = mv2222_handle_interrupt,
	},
};
module_phy_driver(mv2222_drivers);

static struct mdio_device_id __maybe_unused mv2222_tbl[] = {
	{ MARVELL_PHY_ID_88X2222, MARVELL_PHY_ID_MASK },
	{ MARVELL_PHY_ID_88X2222R, MARVELL_PHY_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, mv2222_tbl);

MODULE_DESCRIPTION("Marvell 88x2222 ethernet transceiver driver");
MODULE_LICENSE("GPL");
