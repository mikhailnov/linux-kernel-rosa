// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal GMAC/xGMAC driver
 *
 * Copyright (C) 2022-2025 BAIKAL ELECTRONICS, JSC
 */

#include <linux/acpi.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/firmware/baikal/baikal-smc.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "dwmac1000.h"
#include "dwmac_dma.h"
#include "stmmac.h"
#include "stmmac_platform.h"

/* General Purpose IO */
#define GMAC_GPIO		0x000000e0
#define GMAC_GPIO_GPIS		BIT(0)
#define GMAC_GPIO_GPO		BIT(8)

struct baikal_dwmac {
	struct device *dev;
	int has_aux_div2;
	struct clk *tx_clk;
};

typedef int (*baikal_dwmac_plat_init)(struct baikal_dwmac *, struct plat_stmmacenet_data *);

static int baikal_gmac_bus_reset(void *bsp_priv)
{
	struct baikal_dwmac *bxg = bsp_priv;
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(bxg->dev));

	writel(0, priv->ioaddr + GMAC_GPIO);
	fsleep(priv->mii->reset_delay_us);
	writel(GMAC_GPIO_GPO, priv->ioaddr + GMAC_GPIO);
	if (priv->mii->reset_post_delay_us > 0)
		fsleep(priv->mii->reset_post_delay_us);

	return 0;
}

/* Clean the basic MAC registers up. Note the MAC interrupts are enabled by
 * default after reset. Let's mask them out so not to have any spurious
 * MAC-related IRQ generated during the cleanup procedure.
 */
static void baikal_gmac_core_clean(struct stmmac_priv *priv)
{
	int i;

	writel(0x7FF, priv->ioaddr + GMAC_INT_MASK);
	writel(0, priv->ioaddr + GMAC_CONTROL);
	writel(0, priv->ioaddr + GMAC_FRAME_FILTER);
	writel(0, priv->ioaddr + GMAC_HASH_HIGH);
	writel(0, priv->ioaddr + GMAC_HASH_LOW);
	writel(0, priv->ioaddr + GMAC_FLOW_CTRL);
	writel(0, priv->ioaddr + GMAC_VLAN_TAG);
	writel(0, priv->ioaddr + GMAC_DEBUG);
	writel(0x80000000, priv->ioaddr + GMAC_PMT);
	writel(0, priv->ioaddr + LPI_CTRL_STATUS);
	writel(0x03e80000, priv->ioaddr + LPI_TIMER_CTRL);
	for (i = 0; i < 15; ++i) {
		writel(0x0000ffff, priv->ioaddr + GMAC_ADDR_HIGH(i));
		writel(0xffffffff, priv->ioaddr + GMAC_ADDR_LOW(i));
	}
	writel(0, priv->ioaddr + GMAC_PCS_BASE);
	writel(0, priv->ioaddr + GMAC_RGSMIIIS);
	writel(0x1, priv->ioaddr + GMAC_MMC_CTRL);
	readl(priv->ioaddr + GMAC_INT_STATUS);
	readl(priv->ioaddr + GMAC_PMT);
	readl(priv->ioaddr + LPI_CTRL_STATUS);
}

/* Clean the basic DMA registers up */
static void baikal_gmac_dma_clean(struct stmmac_priv *priv)
{
	writel(0, priv->ioaddr + DMA_INTR_ENA);
	writel(0x00020100, priv->ioaddr + DMA_BUS_MODE);
	writel(0, priv->ioaddr + DMA_RCV_BASE_ADDR);
	writel(0, priv->ioaddr + DMA_TX_BASE_ADDR);
	writel(0x00100000, priv->ioaddr + DMA_CONTROL);
	writel(0x00110001, priv->ioaddr + DMA_AXI_BUS_MODE);
	writel(0x0001FFFF, priv->ioaddr + DMA_STATUS);
}

static int baikal_gmac_swr_reset(void *bsp_priv)
{
	struct baikal_dwmac *gmac = bsp_priv;
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(gmac->dev));

	baikal_gmac_core_clean(priv);

	baikal_gmac_dma_clean(priv);

	return 0;
}

static struct phylink_pcs *baikal_xgmac_select_pcs(struct stmmac_priv *priv,
						phy_interface_t interface)
{
	return xpcs_to_phylink_pcs(priv->hw->xpcs);
}

static void baikal_gmac_fix_mac_speed(void *bsp_priv, unsigned int speed, unsigned int mode)
{
	struct baikal_dwmac *gmac = bsp_priv;
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(gmac->dev));
	unsigned long rate;
	int ret;

	switch (speed) {
	case SPEED_1000:
		rate = 250000000;
		break;
	case SPEED_100:
		rate = 50000000;
		break;
	case SPEED_10:
		rate = 5000000;
		if (gmac->has_aux_div2)
			rate *= 2;
		break;
	default:
		dev_err(gmac->dev, "Unsupported speed %u\n", speed);
		return;
	}

	/* Baikal-S specific ref clock setting up */
	if (gmac->has_aux_div2) {
		struct arm_smccc_res res;

		arm_smccc_smc(speed == SPEED_10 ? BAIKAL_SMC_GMAC_DIV2_ENABLE : BAIKAL_SMC_GMAC_DIV2_DISABLE,
			(unsigned long)priv->ioaddr, 0, 0, 0, 0, 0, 0, &res);
	}

	/* The clock must be gated to successfully update the rate */
	clk_disable_unprepare(gmac->tx_clk);

	ret = clk_set_rate(gmac->tx_clk, rate);
	if (ret)
		dev_err(gmac->dev, "Failed to update Tx clock rate %lu\n", rate);

	ret = clk_prepare_enable(gmac->tx_clk);
	if (ret)
		dev_err(gmac->dev, "Failed to re-enable Tx clock\n");
}

#ifdef CONFIG_ACPI
static int baikal_add_mdio_phy(struct device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(dev));
	struct fwnode_handle *fwnode = device_get_next_child_node(dev, NULL);
	struct phy_device *phy;
	int ret;

	phy = get_phy_device(priv->mii, priv->plat->phy_addr, 0);
	if (IS_ERR(phy)) {
		return PTR_ERR(phy);
	}

	phy->irq = priv->mii->irq[priv->plat->phy_addr];
	phy->mdio.dev.fwnode = fwnode;

	ret = phy_device_register(phy);
	if (ret) {
		phy_device_free(phy);
		return ret;
	}

	return 0;
}

static struct mac_device_info *baikal_gmac_setup(void *ppriv)
{
	struct mac_device_info *mac, *old_mac;
	struct stmmac_priv *priv = ppriv;
	struct gpio_desc *reset_gpio;
	int err;
	u32 value;

	mac = devm_kzalloc(priv->device, sizeof(*mac), GFP_KERNEL);
	if (!mac) {
		return NULL;
	}

	/* Clear PHY reset */
	value = readl(priv->ioaddr + GMAC_GPIO);
	value |= GMAC_GPIO_GPO;
	writel(value, priv->ioaddr + GMAC_GPIO);
	reset_gpio = devm_gpiod_get_optional(priv->device,
					     "snps,reset",
					     GPIOD_OUT_LOW);

	err = readl_poll_timeout(priv->ioaddr + DMA_BUS_MODE, value,
				 !(value & DMA_BUS_MODE_SFT_RESET),
				 10000, 1000000);

	if (reset_gpio != NULL) {
		devm_gpiod_put(priv->device, reset_gpio);
	}

	if (err) {
		dev_err(priv->device, "SW reset is not cleared: error %d", err);
		return NULL;
	}

	old_mac = priv->hw;
	priv->hw = mac;
	err = dwmac1000_setup(priv);
	priv->hw = old_mac;
	if (err) {
		dev_err(priv->device,
			"%s: dwmac1000_setup failed with error %d",
			__func__, err);
		return NULL;
	}

	return mac;
}
#else
static struct plat_stmmacenet_data *baikal_stmmac_probe_config_acpi(struct device *dev,
							       const char **mac)
{
	return NULL;
}

static int baikal_add_mdio_phy(struct device *dev)
{
	return 0;
}

static struct mac_device_info *baikal_gmac_setup(void *ppriv)
{
	return NULL;
}
#endif

static int baikal_gmac_plat_data_init(struct baikal_dwmac *gmac,
				   struct plat_stmmacenet_data *plat)
{
	plat->has_gmac = 1;
	plat->host_dma_width = 32;
	plat->tx_fifo_size = SZ_16K;
	plat->rx_fifo_size = SZ_16K;
	//plat->enh_desc = 1; /* cap.enh_desc */
	plat->tx_coe = 1;
	plat->rx_coe = 1;
	//plat->pmt = 1; /* cap.pmt_rwk */
	plat->unicast_filter_entries = 8;
	plat->multicast_filter_bins = 0;
	plat->swr_reset = baikal_gmac_swr_reset;
	plat->fix_mac_speed = baikal_gmac_fix_mac_speed;
	plat->mdio_bus_data->needs_reset = true;
	if (gmac->dev->of_node)
		plat->bus_reset = baikal_gmac_bus_reset;
	else
		plat->setup = baikal_gmac_setup;

	return 0;
}

static int baikal_s_gmac_plat_data_init(struct baikal_dwmac *bmac,
				   struct plat_stmmacenet_data *plat)
{
	baikal_gmac_plat_data_init(bmac, plat);

	bmac->has_aux_div2 = 1;

	return 0;
}


static int baikal_xgmac_plat_data_init(struct baikal_dwmac *bxg,
				    struct plat_stmmacenet_data *plat)
{
	/* Note Disabling SPH improves the network performance for about
	 * 300Mbit/s. Might be due to the DMA-syncing for two buffers.
	 * Also SPH consumes twice the normal memory especially if there
	 * is a single page-per-buffer allocated.
	 */
	plat->has_xgmac = 1;
	plat->host_dma_width = 40; /* = Cap */
	plat->tx_fifo_size = SZ_128K; /* = Cap */
	plat->rx_fifo_size = SZ_128K; /* = Cap */
	plat->tx_coe = 1; /* = Cap */
	plat->rx_coe = 1; /* = Cap */
	plat->rss_en = 1; /* & cap.rssen */
	plat->unicast_filter_entries = 31;
	plat->multicast_filter_bins = 256;
	plat->select_pcs = baikal_xgmac_select_pcs;
	plat->flags |= STMMAC_FLAG_TSO_EN | /* & cap.tsoen */
		       STMMAC_FLAG_TSO_FULL | /* all channels TSO-capable */
		       STMMAC_FLAG_SPH_DISABLE | /* & cap.sph */
		       STMMAC_FLAG_MULTI_MSI_EN;

	return 0;
}

static int baikal_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources stmmac_res;
	baikal_dwmac_plat_init plat_init;
	struct baikal_dwmac *bmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret) {
		return ret;
	}

	plat = devm_stmmac_probe_config(pdev, (u8 *)&stmmac_res.mac);
	if (IS_ERR(plat)) {
		dev_err(&pdev->dev, "Configuration failed\n");
		return PTR_ERR(plat);
	}

	bmac = devm_kzalloc(&pdev->dev, sizeof(*bmac), GFP_KERNEL);
	if (!bmac)
		return -ENOMEM;

	bmac->dev = &pdev->dev;

	plat->bsp_priv = bmac;

	bmac->tx_clk = devm_clk_get(&pdev->dev, "tx");
	if (IS_ERR(bmac->tx_clk)) {
		bmac->tx_clk = devm_clk_get(&pdev->dev, "tx2_clk");
	}
	if (IS_ERR(bmac->tx_clk)) {
		dev_warn(&pdev->dev, "couldn't get Tx clock\n");
		bmac->tx_clk = NULL;
	} else {
		ret = clk_prepare_enable(bmac->tx_clk);
		if (ret) {
			dev_err(&pdev->dev, "Failed to pre-enable Tx clock\n");
			bmac->tx_clk = NULL;
		}
	}

	plat_init = device_get_match_data(&pdev->dev);
	if (plat_init) {
		ret = plat_init(bmac, plat);
		if (ret)
			return ret;
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat, &stmmac_res);
	if (ret)
		return ret;

	if (ACPI_HANDLE(&pdev->dev) && plat->has_gmac) {
		ret = baikal_add_mdio_phy(&pdev->dev);
		kfree(plat->phy_node);
		if (ret)
			return ret;
	}

	return 0;
}

static void baikal_dwmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct plat_stmmacenet_data *plat = priv->plat;
	struct baikal_dwmac *bmac = plat->bsp_priv;

	clk_disable_unprepare(bmac->tx_clk);

	stmmac_pltfr_remove(pdev);
}

static const struct of_device_id baikal_dwmac_match[] = {
	{ .compatible = "baikal,bm1000-gmac", .data = (void *)baikal_gmac_plat_data_init },
	{ .compatible = "baikal,bs1000-gmac", .data = (void *)baikal_s_gmac_plat_data_init },
	{ .compatible = "baikal,bm1000-xgmac", .data = (void *)baikal_xgmac_plat_data_init },
	{ }
};
MODULE_DEVICE_TABLE(of, baikal_dwmac_match);

static struct platform_driver baikal_dwmac_driver = {
	.probe  = baikal_dwmac_probe,
	.remove_new = baikal_dwmac_remove,
	.driver = {
		.name           = "baikal-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(baikal_dwmac_match),
	},
};
module_platform_driver(baikal_dwmac_driver);

MODULE_AUTHOR("Serge Semin <Sergey.Semin@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal GMAC/XGMAC glue driver");
MODULE_LICENSE("GPL v2");
