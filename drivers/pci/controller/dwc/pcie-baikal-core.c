// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe controller driver for Baikal Electronics SoCs
 *
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 * Authors: Pavel Parkhomenko <pavel.parkhomenko@baikalelectronics.ru>
 *          Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <acpi/acrestyp.h>

#include "../../pci.h"
#include "../../hotplug/pciehp.h"
#include "pcie-designware.h"
#include "pcie-baikal.h"

#define BAIKAL_PCIE_LTSSM_MASK		0x3f
#define BAIKAL_PCIE_LTSSM_STATE_L0	0x11

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)
#define BAIKAL_EDMA_WR_CH		4
#define BAIKAL_EDMA_RD_CH		4

struct baikal_pcie_acpi_data {
	u64		mem_base;
	phys_addr_t	mem_bus_addr;
	u32		mem_size;
};

static const struct baikal_pcie_of_data bm1000_pcie_rc_of_data;
static const struct baikal_pcie_of_data bs1000_pcie_rc_of_data;
#endif

struct baikal_pcie_of_data {
	enum dw_pcie_device_mode	mode;
	struct dw_pcie_ops		dw_pcie_ops;
	int (*get_resources)(struct platform_device *pdev,
			     struct dw_pcie *pci,
			     const struct baikal_pcie_of_data *data);
	int (*add_pcie_port)(struct platform_device *pdev);
};

static const char *baikal_pcie_link_speed_str(const unsigned int speed)
{
	static const char * const speed_str[] = {"2.5", "5.0", "8.0", "16.0"};

	if (speed > 0 && speed <= ARRAY_SIZE(speed_str))
		return speed_str[speed - 1];

	return "???";
}

static void baikal_pcie_link_print_status(struct dw_pcie *pci)
{
	struct device *dev = pci->dev;
	u16 exp_cap_off;
	u32 reg;

	if (!pci->ops->link_up(pci)) {
		dev_info(dev, "link is down\n");
		return;
	}

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKSTA);

	dev_info(dev, "current link is %s GT/s%s, x%u\n",
		 baikal_pcie_link_speed_str(FIELD_GET(PCI_EXP_LNKSTA_CLS, reg)),
		 FIELD_GET(PCI_EXP_LNKSTA_LT, reg) ? " (training)" : "",
		 FIELD_GET(PCI_EXP_LNKSTA_NLW, reg));
}

static bool baikal_pcie_link_is_training(struct dw_pcie *pci)
{
	u16 exp_cap_off;
	u32 reg;

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKSTA);
	return FIELD_GET(PCI_EXP_LNKSTA_LT, reg);
}

static bool baikal_pcie_link_wait_training_done(struct dw_pcie *pci)
{
	struct device *dev = pci->dev;
	unsigned long start_jiffies = jiffies;

	while (baikal_pcie_link_is_training(pci)) {
		if (time_after(jiffies, start_jiffies + HZ)) {
			dev_err(dev, "link training timeout occurred\n");
			return false;
		}

		udelay(100);
	}

	return true;
}

#define BM1000_PCIE_GPR_RESET_BASE		0x00
#define BM1000_PCIE_GPR_RESET(x)		(((x) * 0x20) + BM1000_PCIE_GPR_RESET_BASE)
#define BM1000_PCIE_PHY_RST			BIT(0)
#define BM1000_PCIE_PIPE_RST			BIT(4) /* x4 controllers only */
#define BM1000_PCIE_PIPE0_RST			BIT(4) /* x8 controller only */
#define BM1000_PCIE_PIPE1_RST			BIT(5) /* x8 controller only */
#define BM1000_PCIE_CORE_RST			BIT(8)
#define BM1000_PCIE_PWR_RST			BIT(9)
#define BM1000_PCIE_STICKY_RST			BIT(10)
#define BM1000_PCIE_NONSTICKY_RST		BIT(11)
#define BM1000_PCIE_HOT_RST			BIT(12)
#define BM1000_PCIE_ADB_PWRDWN			BIT(13)

#define BM1000_PCIE_GPR_STATUS_BASE		0x04
#define BM1000_PCIE_GPR_STATUS(x)		(((x) * 0x20) + BM1000_PCIE_GPR_STATUS_BASE)

#define BM1000_PCIE_GPR_GENCTL_BASE		0x08
#define BM1000_PCIE_GPR_GENCTL(x)		(((x) * 0x20) + BM1000_PCIE_GPR_GENCTL_BASE)
#define BM1000_PCIE_LTSSM_ENABLE		BIT(1)
#define BM1000_PCIE_DBI2_MODE			BIT(2)
#define BM1000_PCIE_PHY_MGMT_ENABLE		BIT(3)

#define BM1000_PCIE_GPR_MSI_TRANS_CTL2		0xf8
#define BM1000_PCIE_MSI_TRANS_EN(x)		BIT(9 + (x))
#define BM1000_PCIE_MSI_TRANS_RCNUM(x)		((x) << (2 * (x)))
#define BM1000_PCIE_MSI_TRANS_RCNUM_MASK(x)	((3) << (2 * (x)))

#define BM1000_PCIE0_DBI_BASE			0x02200000
#define BM1000_PCIE1_DBI_BASE			0x02210000
#define BM1000_PCIE2_DBI_BASE			0x02220000

struct bm1000_pcie {
	struct dw_pcie		*pci;
	unsigned int		num;
	struct regmap		*gpr;
	union {
		struct gpio_desc *reset_gpio;
		struct {
			u8 num : 5;
			u8 polarity : 1;
			u8 is_set : 1;
		} gpio[2];
	};
	char			reset_name[32];
	bool			retrained;
};

void bm1000_pcie_phy_enable(struct dw_pcie *pci)
{
	struct bm1000_pcie *bm = dev_get_drvdata(pci->dev);
	u32 reg;

	regmap_read(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), &reg);
	reg |= BM1000_PCIE_PHY_MGMT_ENABLE | BM1000_PCIE_DBI2_MODE;
	regmap_write(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), reg);
}

void bm1000_pcie_phy_disable(struct dw_pcie *pci)
{
	struct bm1000_pcie *bm = dev_get_drvdata(pci->dev);
	u32 reg;

	regmap_read(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), &reg);
	reg &= ~(BM1000_PCIE_PHY_MGMT_ENABLE | BM1000_PCIE_DBI2_MODE);
	regmap_write(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), reg);
}

static int bm1000_get_resources(struct platform_device *pdev,
				struct dw_pcie *pci,
				const struct baikal_pcie_of_data *data)
{
	struct bm1000_pcie *bm = platform_get_drvdata(pdev);
	struct device *dev = pci->dev;
	struct resource *res;

	bm->pci = pci;
	bm->gpr = syscon_regmap_lookup_by_compatible("baikal,bm1000-pcie-gpr");
	if (IS_ERR(bm->gpr)) {
		dev_err(dev, "failed to find PCIe GPR registers\n");
		return PTR_ERR(bm->gpr);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res) {
		dev_err(dev, "failed to find \"dbi\" region\n");
		return -EINVAL;
	}

	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	if (res->start == BM1000_PCIE0_DBI_BASE) {
		bm->num = 0;
	} else if (res->start == BM1000_PCIE1_DBI_BASE) {
		bm->num = 1;
	} else if (res->start == BM1000_PCIE2_DBI_BASE) {
		bm->num = 2;
	} else {
		dev_err(dev, "incorrect \"dbi\" base\n");
		return -EINVAL;
	}

	pci->max_link_speed = of_pci_get_max_link_speed(dev->of_node);
	if (pci->max_link_speed <= 0 || pci->max_link_speed > 3)
		pci->max_link_speed = 3;

	return 0;
}

static int bm1000_pcie_link_up(struct dw_pcie *pci)
{
	struct bm1000_pcie *bm = dev_get_drvdata(pci->dev);
	u32 reg;

	regmap_read(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), &reg);
	if (!(reg & BM1000_PCIE_LTSSM_ENABLE))
		return 0;

	regmap_read(bm->gpr, BM1000_PCIE_GPR_STATUS(bm->num), &reg);
	return (reg & BAIKAL_PCIE_LTSSM_MASK) == BAIKAL_PCIE_LTSSM_STATE_L0;
}

static int bm1000_pcie_start_link(struct dw_pcie *pci)
{
	struct bm1000_pcie *bm = dev_get_drvdata(pci->dev);
	u32 reg;

	regmap_read(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), &reg);
	reg |= BM1000_PCIE_LTSSM_ENABLE;
	regmap_write(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), reg);
	return 0;
}

static void bm1000_pcie_link_speed_fixup(struct bm1000_pcie *bm,
					 struct pci_bus *bus)
{
	struct pci_dev *dev;
	struct dw_pcie *pci = bm->pci;
	unsigned int dev_lnkcap_speed;
	unsigned int dev_lnkcap_width;
	unsigned int rc_lnkcap_speed;
	unsigned int rc_lnksta_speed;
	unsigned int rc_target_speed;
	u16 exp_cap_off;
	u32 reg;

	/* Return if the bus has already been retrained */
	if (bm->retrained)
		return;

	list_for_each_entry(dev, &bus->devices, bus_list)
		if (dev->subordinate)
			bm1000_pcie_link_speed_fixup(bm, dev->subordinate);

	/* Skip root bridge and devices not directly attached to the RC */
	if (pci_is_root_bus(bus) || !pci_is_root_bus(bus->parent))
		return;

	dev = list_first_entry_or_null(&bus->devices, struct pci_dev, bus_list);
	if (!dev)
		return;

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

	reg = dw_pcie_readl_dbi(pci, exp_cap_off + PCI_EXP_LNKCAP);
	rc_lnkcap_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, reg);

	reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKSTA);
	rc_lnksta_speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, reg);

	if (acpi_disabled) {
		pcie_capability_read_dword(dev, PCI_EXP_LNKCAP, &reg);
	} else {
		struct pci_config_window *cfg;
		int where = pci_pcie_cap(dev) + PCI_EXP_LNKCAP;

		cfg = dev->bus->sysdata;
		if (dev->bus->number == cfg->busr.start) {
			if (PCI_SLOT(dev->devfn) > 0)
				reg = 0;
			else
				reg = readl(pci->dbi_base + where);
		} else {
			unsigned int busn = dev->bus->number - cfg->busr.start;
			unsigned int devfn_shift = cfg->ops->bus_shift - 8;

			reg = readl(cfg->win +
				    (busn << cfg->ops->bus_shift) +
				    (dev->devfn << devfn_shift) +
				    where);
		}
	}

	dev_lnkcap_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, reg);
	dev_lnkcap_width = FIELD_GET(PCI_EXP_LNKCAP_MLW, reg);

	baikal_pcie_link_print_status(pci);

	/*
	 * [2.5 -> 8.0 GT/s] is suitable way of retraining.
	 * [2.5 -> 5.0 GT/s] is used when 8.0 GT/s could not be reached.
	 * [5.0 -> 8.0 GT/s] causes system freezing sometimes.
	 */
	if (rc_lnkcap_speed < dev_lnkcap_speed)
		rc_target_speed = rc_lnkcap_speed;
	else
		rc_target_speed = dev_lnkcap_speed;

	if (rc_target_speed > pci->max_link_speed)
		rc_target_speed = pci->max_link_speed;

	while (rc_lnksta_speed < rc_target_speed) {
		unsigned long start_jiffies;

		/* Try to change link speed */
		dev_info(pci->dev, "retrain link to %s GT/s\n",
			 baikal_pcie_link_speed_str(rc_target_speed));

		/* If link is already training wait for training to complete */
		baikal_pcie_link_wait_training_done(pci);

		/* Set desired speed */
		reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKCTL2);
		reg &= ~PCI_EXP_LNKCTL2_TLS;
		reg |= rc_target_speed;
		dw_pcie_writew_dbi(pci, exp_cap_off + PCI_EXP_LNKCTL2, reg);

		/* Deassert and assert PORT_LOGIC_SPEED_CHANGE bit */
		reg = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		reg &= ~PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, reg);
		reg |=	PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, reg);

		/* Wait for link training */
		start_jiffies = jiffies;
		for (;;) {
			/* Wait for link training begin */
			if (!baikal_pcie_link_is_training(pci)) {
				if (time_after(jiffies, start_jiffies + HZ)) {
					dev_err(pci->dev,
						"link training has not started\n");
					/*
					 * Don't wait for training_done()
					 * if it hasn't started.
					 */
					break;
				}

				udelay(100);
				continue;
			}

			/* Wait for link training end */
			if (!baikal_pcie_link_wait_training_done(pci))
				break;

			if (!dw_pcie_wait_for_link(pci)) {
				/* Wait if link switched to config/recovery */
				baikal_pcie_link_wait_training_done(pci);
				baikal_pcie_link_print_status(pci);
			}

			break;
		}

		/* Check if the link is down after retrain */
		if (!bm1000_pcie_link_up(pci)) {
			/*
			 * Check if the link has already been down and
			 * the link is unable to re-establish at 2.5 GT/s
			 */
			if (rc_lnksta_speed == 0 &&
			    rc_target_speed == PCI_EXP_LNKCTL2_TLS_2_5GT)
				break;

			rc_lnksta_speed = 0;
			if (rc_target_speed > PCI_EXP_LNKCTL2_TLS_2_5GT) {
				/* Try to use lower speed */
				--rc_target_speed;
			}

			continue;
		}

		reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKSTA);
		rc_lnksta_speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, reg);

		/* Check if the targeted speed has not been reached */
		if (rc_lnksta_speed < rc_target_speed &&
		    rc_target_speed > PCI_EXP_LNKCTL2_TLS_2_5GT) {
			/* Try to use lower speed */
			--rc_target_speed;
		}
	}

	bm->retrained = true;
}

static void bm1000_pcie_set_gpio(u8 num, bool enable)
{
	void __iomem *addr;
	u32 val;

	if (num > 31)
		return;

	addr = ioremap(0x20200000, 8);
	if (addr) {
		val = readl(addr);
		if (enable)
			val |= 1 << num;
		else
			val &= ~BIT(num);
		writel(val, addr);

		val = readl(addr + 4);
		val |= 1 << num;
		writel(val, addr + 4);

		iounmap(addr);
	}
}

static int bm1000_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct bm1000_pcie *bm = dev_get_drvdata(pci->dev);
	struct device *dev = pci->dev;
	int err;
	int linkup;
	u16 exp_cap_off;
	u16 ext_cap_err_off;
	u32 reg;

	/* Disable access to PHY registers and DBI2 mode */
	bm1000_pcie_phy_disable(pci);

	bm->retrained = false;
	linkup = bm1000_pcie_link_up(pci);

	/* If link is not established yet, reset the RC */
	if (!linkup) {
		/* Disable link training */
		regmap_read(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), &reg);
		reg &= ~BM1000_PCIE_LTSSM_ENABLE;
		regmap_write(bm->gpr, BM1000_PCIE_GPR_GENCTL(bm->num), reg);

		/* Assert PERST pin */
		if (acpi_disabled) {
			snprintf(bm->reset_name, sizeof(bm->reset_name),
				"pcie%u-reset", bm->num);

			bm->reset_gpio = devm_fwnode_gpiod_get(dev,
				of_fwnode_handle(dev->of_node), "reset",
				GPIOD_OUT_HIGH, bm->reset_name);
			err = PTR_ERR_OR_ZERO(bm->reset_gpio);
			if (err) {
				if (err != -ENOENT) {
					dev_err(dev, "request GPIO failed (%d)\n", err);
					return err;
				}
				/* reset gpio is optional */
				bm->reset_gpio = NULL;
			}
		} else if (!acpi_disabled && bm->gpio[0].is_set) {
			bm1000_pcie_set_gpio(bm->gpio[0].num, ~bm->gpio[0].polarity);
		}

		/* Reset the RC */
		regmap_read(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), &reg);
		reg |= BM1000_PCIE_NONSTICKY_RST |
		       BM1000_PCIE_STICKY_RST	 |
		       BM1000_PCIE_PWR_RST	 |
		       BM1000_PCIE_CORE_RST	 |
		       BM1000_PCIE_PHY_RST;

		/* If the RC is PCIe x8, reset PIPE0 and PIPE1 */
		if (bm->num == 2) {
			reg |= BM1000_PCIE_PIPE0_RST |
			       BM1000_PCIE_PIPE1_RST;
		} else {
			reg |= BM1000_PCIE_PIPE_RST;
		}

		regmap_write(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), reg);

		if (!acpi_disabled && bm->num == 2 && bm->gpio[1].is_set) {
			/* Assert PRSNT pin */
			bm1000_pcie_set_gpio(bm->gpio[1].num, ~bm->gpio[1].polarity);
		}

		usleep_range(20000, 30000);

		/* Deassert PERST pin */
		if (bm->reset_gpio && acpi_disabled)
			gpiod_set_value_cansleep(bm->reset_gpio, 0);
		else if (!acpi_disabled && bm->gpio[0].is_set)
			bm1000_pcie_set_gpio(bm->gpio[0].num, bm->gpio[0].polarity);

		/* Deassert PHY reset */
		regmap_read(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), &reg);
		reg &= ~BM1000_PCIE_PHY_RST;
		regmap_write(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), reg);

		/* Deassert all software controlled resets */
		regmap_read(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), &reg);
		reg &= ~(BM1000_PCIE_ADB_PWRDWN	   |
			 BM1000_PCIE_HOT_RST	   |
			 BM1000_PCIE_NONSTICKY_RST |
			 BM1000_PCIE_STICKY_RST	   |
			 BM1000_PCIE_PWR_RST	   |
			 BM1000_PCIE_CORE_RST	   |
			 BM1000_PCIE_PHY_RST);

		if (bm->num == 2) {
			reg &= ~(BM1000_PCIE_PIPE0_RST |
				 BM1000_PCIE_PIPE1_RST);
		} else {
			reg &= ~BM1000_PCIE_PIPE_RST;
		}

		regmap_write(bm->gpr, BM1000_PCIE_GPR_RESET(bm->num), reg);
	}

	/* Enable error reporting */
	ext_cap_err_off = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_ERR);
	reg = dw_pcie_readl_dbi(pci, ext_cap_err_off + PCI_ERR_ROOT_COMMAND);
	reg |= PCI_ERR_ROOT_CMD_COR_EN	    |
	       PCI_ERR_ROOT_CMD_NONFATAL_EN |
	       PCI_ERR_ROOT_CMD_FATAL_EN;
	dw_pcie_writel_dbi(pci, ext_cap_err_off + PCI_ERR_ROOT_COMMAND, reg);

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL);
	reg |= PCI_EXP_DEVCTL_CERE  |
	       PCI_EXP_DEVCTL_NFERE |
	       PCI_EXP_DEVCTL_FERE  |
	       PCI_EXP_DEVCTL_URRE;
	dw_pcie_writew_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL, reg);

	reg = dw_pcie_readl_dbi(pci, exp_cap_off + PCI_EXP_RTCTL);
	reg |= PCI_EXP_RTCTL_SECEE  |
	       PCI_EXP_RTCTL_SENFEE |
	       PCI_EXP_RTCTL_SEFEE  |
	       PCI_EXP_RTCTL_PMEIE;
	dw_pcie_writel_dbi(pci, exp_cap_off + PCI_EXP_RTCTL, reg);

	if (linkup) {
		dev_info(dev, "link is already up\n");
	} else {
		/* Use 2.5 GT/s rate for link establishing */
		reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_LNKCTL2);
		reg &= ~PCI_EXP_LNKCTL2_TLS;
		reg |=	PCI_EXP_LNKCTL2_TLS_2_5GT;
		dw_pcie_writew_dbi(pci, exp_cap_off + PCI_EXP_LNKCTL2, reg);
	}

	regmap_read(bm->gpr, BM1000_PCIE_GPR_MSI_TRANS_CTL2, &reg);
	reg &= ~BM1000_PCIE_MSI_TRANS_RCNUM_MASK(bm->num);
	reg |= BM1000_PCIE_MSI_TRANS_RCNUM(bm->num);
	reg |= BM1000_PCIE_MSI_TRANS_EN(bm->num);
	regmap_write(bm->gpr, BM1000_PCIE_GPR_MSI_TRANS_CTL2, reg);

	/* RX/TX equalizers fine tune */
	bm1000_pcie_tune(pci);

	return 0;
}

static irqreturn_t bm1000_pcie_aer_irq_handler(int irq, void *arg)
{
	struct bm1000_pcie *bm = arg;
	struct dw_pcie *pci = bm->pci;
	struct device *dev = pci->dev;
	u16 exp_cap_off;
	u16 ext_cap_err_off;
	u16 dev_sta;
	u32 cor_err;
	u32 root_err;
	u32 uncor_err;

	exp_cap_off	= dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	ext_cap_err_off = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_ERR);

	uncor_err = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_UNCOR_STATUS);
	cor_err	  = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_COR_STATUS);
	root_err  = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_ROOT_STATUS);
	dev_sta	  = dw_pcie_readw_dbi(pci,
				      exp_cap_off + PCI_EXP_DEVSTA);

	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_UNCOR_STATUS, uncor_err);
	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_COR_STATUS, cor_err);
	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_ROOT_STATUS, root_err);
	dw_pcie_writew_dbi(pci,
			   exp_cap_off + PCI_EXP_DEVSTA, dev_sta);

	dev_err(dev,
		"DevSta:0x%04x RootErr:0x%x UncorErr:0x%x CorErr:0x%x\n",
		dev_sta, root_err, uncor_err, cor_err);

	return IRQ_HANDLED;
}

static const struct dw_pcie_host_ops bm1000_pcie_host_ops = {
	.init	 = bm1000_pcie_host_init,
};

static int bm1000_add_pcie_port(struct platform_device *pdev)
{
	struct bm1000_pcie *bm = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = bm->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	int ret;

	pp->irq = platform_get_irq_byname(pdev, "aer");
	if (pp->irq < 0) {
		dev_err(dev, "failed to get \"aer\" IRQ\n");
		return pp->irq;
	}

	ret = devm_request_irq(dev, pp->irq, bm1000_pcie_aer_irq_handler,
			       IRQF_SHARED, "bm1000-pcie-aer", bm);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d\n", pp->irq);
		return ret;
	}

	pp->num_vectors = MAX_MSI_IRQS;
	pp->ops = &bm1000_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	bm1000_pcie_link_speed_fixup(bm, pp->bridge->bus);
	return 0;
}

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)
static void bm1000_pcie_prog_outbound_atu(struct dw_pcie *pci, int index,
					  int type, u64 cpu_addr, u64 pci_addr,
					  u32 size, u32 flags)
{
	u32 retries, val;

	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT,
			   PCIE_ATU_REGION_DIR_OB | index);
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_BASE,
			   lower_32_bits(cpu_addr));
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_BASE,
			   upper_32_bits(cpu_addr));
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LIMIT,
			   lower_32_bits(cpu_addr + size - 1));
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_TARGET,
			   lower_32_bits(pci_addr));
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_TARGET,
			   upper_32_bits(pci_addr));
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE, type);
	dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_REGION_CTRL2,
			   PCIE_ATU_ENABLE | flags);

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; ++retries) {
		val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT_BASE +
					     PCIE_ATU_REGION_CTRL2);
		if (val & PCIE_ATU_ENABLE)
			return;

		mdelay(LINK_WAIT_IATU);
	}
	dev_err(pci->dev, "Outbound iATU is not being enabled\n");
}

#define PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE	BIT(28)

static void bm1000_pcie_setup_rc_acpi(struct dw_pcie_rp *pp,
				      const struct baikal_pcie_acpi_data *mem_data)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	acpi_status status;
	u64 lanes;
	u32 val;
	int i;

	/*
	 * Enable DBI read-only registers for writing/updating configuration.
	 * Write permission gets disabled towards the end of this function.
	 */
	dw_pcie_dbi_ro_wr_en(pci);

	val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_FAST_LINK_MODE;
	val |= PORT_LINK_DLL_LINK_EN;
	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	status = acpi_evaluate_integer(to_acpi_device(pci->dev)->handle,
				       "NUML", NULL, &lanes);
	if (ACPI_FAILURE(status)) {
		dev_dbg(pci->dev, "failed to get num-lanes\n");
	} else {
		pci->num_lanes = lanes;

		/* Set the number of lanes */
		val &= ~PORT_LINK_FAST_LINK_MODE;
		val &= ~PORT_LINK_MODE_MASK;
		switch (pci->num_lanes) {
		case 1:
			val |= PORT_LINK_MODE_1_LANES;
			break;
		case 2:
			val |= PORT_LINK_MODE_2_LANES;
			break;
		case 4:
			val |= PORT_LINK_MODE_4_LANES;
			break;
		case 8:
			val |= PORT_LINK_MODE_8_LANES;
			break;
		default:
			dev_err(pci->dev, "NUML %u: invalid value\n", pci->num_lanes);
			goto skip_lanes;
		}
		dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

		/* Set link width speed control register */
		val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
		switch (pci->num_lanes) {
		case 1:
			val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
			break;
		case 2:
			val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
			break;
		case 4:
			val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
			break;
		case 8:
			val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
			break;
		}
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	}

skip_lanes:
	/* Setup RC BARs */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

	/* Setup interrupt pins */
	val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);

	/* Setup bus numbers */
	val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* Setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	pci->atu_base = pci->dbi_base + PCIE_ATU_VIEWPORT_BASE;
	pci->atu_size = PCIE_ATU_VIEWPORT_SIZE;
	pci->num_ob_windows = 4;
	pci->num_ib_windows = 0;

	for (i = 0; i < pci->num_ob_windows; ++i)
		dw_pcie_disable_atu(pci, PCIE_ATU_REGION_DIR_OB, i);

	/* Program ATU */
	bm1000_pcie_prog_outbound_atu(pci, 0, PCIE_ATU_TYPE_CFG0,
				      pp->cfg0_base, 0,
				      SZ_2M,
				      PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE);
	bm1000_pcie_prog_outbound_atu(pci, 1, PCIE_ATU_TYPE_CFG1,
				      pp->cfg0_base, 0,
				      pp->cfg0_size,
				      PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE);
	bm1000_pcie_prog_outbound_atu(pci, 2, PCIE_ATU_TYPE_MEM,
				      mem_data->mem_base, mem_data->mem_bus_addr,
				      mem_data->mem_size, 0);
	bm1000_pcie_prog_outbound_atu(pci, 3, PCIE_ATU_TYPE_IO,
				      pp->io_base, pp->io_bus_addr,
				      pp->io_size, 0);

	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);

	/* Program correct class for RC */
	dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_PCI);

	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	dw_pcie_dbi_ro_wr_dis(pci);
}

struct bm1000_pcie_acpi_gpio {
	struct list_head node;
	struct acpi_resource_gpio *ares;
};

static int bm1000_pcie_acpi_dev_filter_gpio(struct acpi_resource *ares, void *data)
{
	struct list_head *list = data;
	struct bm1000_pcie_acpi_gpio *gpio;

	if (ares->type == ACPI_RESOURCE_TYPE_GPIO) {
		gpio = kzalloc(sizeof(*gpio), GFP_KERNEL);
		if (gpio) {
			INIT_LIST_HEAD(&gpio->node);
			gpio->ares = &ares->data.gpio;
			list_add_tail(&gpio->node, list);
			return 0;
		}
	}

	return 1;
}

static int bm1000_pcie_get_res_acpi(struct acpi_device *adev,
				    struct acpi_device **res_dev,
				    struct bm1000_pcie *bm,
				    struct baikal_pcie_acpi_data *mem_data)
{
	struct device *dev = &adev->dev;
	struct dw_pcie_rp *pp = &bm->pci->pp;
	struct resource_entry *entry;
	struct list_head list, *pos;
	struct fwnode_handle *fwnode;
	struct bm1000_pcie_acpi_gpio *gpio, *tmp;
	int ret;
	unsigned long flags = IORESOURCE_MEM;

	fwnode = fwnode_get_named_child_node(&adev->fwnode, "RES0");
	if (!fwnode) {
		dev_err(dev, "failed to get RES0 subdevice\n");
		return -EINVAL;
	}

	*res_dev = to_acpi_device_node(fwnode);
	if (!*res_dev) {
		dev_err(dev, "RES0 is not an acpi device node\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(*res_dev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse RES0._CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 2) {
		dev_err(dev,
			"invalid number of MEM resources present in RES0._CRS (%i, need 2)\n", ret);
		return -EINVAL;
	}

	/* ECAM */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->cfg0_size = resource_size(entry->res);
	pp->cfg0_base = entry->res->start;

	/* DBI */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	if (entry->res->start == BM1000_PCIE0_DBI_BASE) {
		bm->num = 0;
	} else if (entry->res->start == BM1000_PCIE1_DBI_BASE) {
		bm->num = 1;
	} else if (entry->res->start == BM1000_PCIE2_DBI_BASE) {
		bm->num = 2;
	} else {
		dev_err(dev, "incorrect \"dbi\" base\n");
		return -EINVAL;
	}
	bm->pci->dbi_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(bm->pci->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(bm->pci->dbi_base);
		return ret;
	}

	acpi_dev_free_resource_list(&list);

	/* Non-prefetchable memory */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_MEM;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	mem_data->mem_base = entry->res->start;
	mem_data->mem_size = resource_size(entry->res);
	mem_data->mem_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);

	/* I/O */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_IO;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of IO resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->io_base = entry->res->start;
	pp->io_size = resource_size(entry->res);
	pp->io_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);

	/* GPIO */
	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(*res_dev, &list,
				     bm1000_pcie_acpi_dev_filter_gpio,
				     &list);
	if (ret < 0) {
		dev_err(dev, "failed to parse RES0._CRS method, error code %d\n", ret);
		return ret;
	}

	if (!ret) {
		u8 count = 0;
		u8 i;

		list_for_each_entry(gpio, &list, node) {
			for (i = 0; i < gpio->ares->pin_table_length &&
			     count < ARRAY_SIZE(bm->gpio); ++i) {
				bm->gpio[count].num = gpio->ares->pin_table[i] & 0x1f;
				bm->gpio[count].polarity = 1;
				bm->gpio[count].is_set = 1;
				++count;
			}

			if (count == ARRAY_SIZE(bm->gpio))
				break;
		}
	}

	list_for_each_entry_safe(gpio, tmp, &list, node) {
		list_del(&gpio->node);
		kfree(gpio);
	}

	return 0;
}

static int bm1000_pcie_get_irq_acpi(struct device *dev,
				    struct acpi_device *res_dev,
				    struct bm1000_pcie *bm)
{
	struct dw_pcie_rp *pp = &bm->pci->pp;
	struct resource res;
	int ret;

	memset(&res, 0, sizeof(res));

	ret = acpi_irq_get(res_dev->handle, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get irq %d\n", 0);
		return ret;
	}

	if (res.flags & IORESOURCE_BITS) {
		struct irq_data *irqd;

		irqd = irq_get_irq_data(res.start);
		if (!irqd)
			return -ENXIO;

		irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
	}

	pp->irq = res.start;

	ret = devm_request_irq(dev, pp->irq, bm1000_pcie_aer_irq_handler,
			       IRQF_SHARED, "bm1000-pcie-aer", bm);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", pp->irq);
		return ret;
	}

	return 0;
}

static struct regmap *bm1000_regmap;

static const struct regmap_config bm1000_pcie_syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4
};

static struct regmap *bm1000_pcie_get_gpr_acpi(struct bm1000_pcie *bm)
{
	struct device *dev;
	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status = AE_OK;
	struct list_head list, *pos;
	struct resource *res;
	void __iomem *base;
	struct regmap *regmap = NULL;
	struct regmap_config config = bm1000_pcie_syscon_regmap_config;
	unsigned long flags = IORESOURCE_MEM;
	int ret;

	if (bm1000_regmap)
		return bm1000_regmap;

	status = acpi_get_handle(NULL, "\\_SB.PGPR", &handle);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get PCIe GPR device\n");
		return ERR_PTR(-ENODEV);
	}

	adev = acpi_fetch_acpi_dev(handle);
	if (!adev) {
		dev_err(dev, "failed to process PCIe GPR handle\n");
		return ERR_PTR(-ENODEV);
	}

	dev = &adev->dev;
	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return NULL;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		goto ret;
	}

	pos = list.next;
	res = list_entry(pos, struct resource_entry, node)->res;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base) {
		dev_err(dev, "error with ioremap\n");
		goto ret;
	}

	config.max_register = resource_size(res) - 4;

	regmap = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "regmap init failed\n");
		devm_iounmap(dev, base);
		goto ret;
	}

	dev_dbg(dev, "regmap %pR registered\n", res);

	bm1000_regmap = regmap;

ret:
	acpi_dev_free_resource_list(&list);
	return regmap;
}

static int bm1000_get_acpi_data(struct device *dev, struct bm1000_pcie *bm,
				struct baikal_pcie_acpi_data *mem_data)
{
	struct acpi_device *adev = to_acpi_device(dev), *res_dev;
	int ret;

	bm->gpr = bm1000_pcie_get_gpr_acpi(bm);
	if (IS_ERR_OR_NULL(bm->gpr)) {
		dev_err(dev, "No PCIe GPR specified\n");
		return -EINVAL;
	}

	ret = bm1000_pcie_get_res_acpi(adev, &res_dev, bm, mem_data);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	ret = bm1000_pcie_get_irq_acpi(dev, res_dev, bm);
	if (ret) {
		dev_err(dev, "failed to get irq info\n");
		return ret;
	}

	return 0;
}

static int bm1000_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct bm1000_pcie *bm;
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;
	struct baikal_pcie_acpi_data mem_data = {};
	int ret;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &bm1000_pcie_rc_of_data.dw_pcie_ops;

	bm = devm_kzalloc(dev, sizeof(*bm), GFP_KERNEL);
	if (!bm)
		return -ENOMEM;

	cfg->priv = bm;
	bm->pci = pci;
	dev_set_drvdata(dev, bm);

	ret = bm1000_get_acpi_data(dev, bm, &mem_data);
	if (ret) {
		dev_err(dev, "failed to get data from ACPI\n");
		return ret;
	}

	pp = &pci->pp;
	raw_spin_lock_init(&pp->lock);
	pp->ops = &bm1000_pcie_host_ops;
	pp->va_cfg0_base = devm_pci_remap_cfgspace(dev, pp->cfg0_base,
						   pp->cfg0_size);
	if (!pp->va_cfg0_base) {
		dev_err(dev, "error with ioremap\n");
		return -ENOMEM;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "failed to enable DMA\n");
		return ret;
	}

	ret = bm1000_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	dw_pcie_version_detect(pci);
	bm1000_pcie_setup_rc_acpi(pp, &mem_data);

	if (!bm1000_pcie_link_up(bm->pci))
		bm1000_pcie_start_link(bm->pci);

	/* Link will be retrained by 'bm1000_pcie_map_bus()' */
	bm->retrained = false;
	return 0;
}

static void __iomem *bm1000_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct bm1000_pcie *bm = cfg->priv;
	unsigned int devfn_shift = cfg->ops->bus_shift - 8;
	unsigned int busn = bus->number;
	void __iomem *base;

	if (!bm->retrained) {
		struct pci_host_bridge **root_bridge = &bm->pci->pp.bridge;

		if (!*root_bridge) {
			*root_bridge = to_pci_host_bridge(bus->bridge);
		} else if ((*root_bridge)->bus->is_added) {
			struct acpi_device *adev = to_acpi_device(cfg->parent);
			struct acpi_pci_root *root = adev->driver_data;

			if (root->bus == (*root_bridge)->bus) {
				pci_bus_add_devices(root->bus);
				bm1000_pcie_link_speed_fixup(bm, root->bus);
				bm->retrained = true;
			}
		}
	}

	if (bus->number != cfg->busr.start && !bm1000_pcie_link_up(bm->pci))
		return NULL;

	if (bus->number == cfg->busr.start) {
		/*
		 * The DW PCIe core doesn't filter out transactions to other
		 * devices/functions on the root bus num, so we do this here.
		 */
		if (PCI_SLOT(devfn) > 0)
			return NULL;
		else
			return bm->pci->dbi_base + where;
	}

	if (busn < cfg->busr.start || busn > cfg->busr.end)
		return NULL;

	busn -= cfg->busr.start;
	base = cfg->win + (busn << cfg->ops->bus_shift);
	return base + (devfn << devfn_shift) + where;
}

const struct pci_ecam_ops baikal_m_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= bm1000_pcie_init,
	.pci_ops	= {
		.map_bus	= bm1000_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif

#define BS1000_PCIE_APB_PE_GEN_CTRL3			0x58
#define BS1000_PCIE_APB_PE_GEN_CTRL3_LTSSM_EN		BIT(0)

#define BS1000_PCIE_APB_PE_LINK_DBG2			0xb4
#define BS1000_PCIE_APB_PE_LINK_DBG2_SMLH_LINK_UP	BIT(6)
#define BS1000_PCIE_APB_PE_LINK_DBG2_RDLH_LINK_UP	BIT(7)

#define BS1000_PCIE_APB_PE_ERR_STS			0xe0
#define BS1000_PCIE_APB_PE_INT_STS			0xe8

#define BS1000_PCIE0_P0_DBI_BASE			0x39000000
#define BS1000_PCIE0_P1_DBI_BASE			0x39400000
#define BS1000_PCIE1_P0_DBI_BASE			0x3d000000
#define BS1000_PCIE1_P1_DBI_BASE			0x3d400000
#define BS1000_PCIE2_P0_DBI_BASE			0x45000000
#define BS1000_PCIE2_P1_DBI_BASE			0x45400000

#define BS1000_ADDR_IN_CHIP(base)			((base) & 0xffffffff)

struct bs1000_pcie {
	struct dw_pcie		*pci;
	void __iomem		*apb_base;
	u64			cpu_addr_mask;
#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)
	int			edma_irq[BAIKAL_EDMA_WR_CH + BAIKAL_EDMA_RD_CH];
#endif
};

static void bs1000_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	for (bar = BAR_0; bar <= BAR_5; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
}

static int bs1000_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				    unsigned int type,
				    u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_IRQ_INTX:
		return dw_pcie_ep_raise_intx_irq(ep, func_no);
	case PCI_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	}
	dev_err(pci->dev, "UNKNOWN IRQ type\n");
	return -EINVAL;
}

static const struct pci_epc_features bs1000_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	.bar[BAR_3].type = BAR_RESERVED,
	.bar[BAR_5].type = BAR_RESERVED,
	.align = SZ_64K,
};

static const struct pci_epc_features*
bs1000_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	return &bs1000_pcie_epc_features;
}

static const struct dw_pcie_ep_ops bs1000_pcie_ep_ops = {
	.init		= bs1000_pcie_ep_init,
	.raise_irq	= bs1000_pcie_ep_raise_irq,
	.get_features	= bs1000_pcie_ep_get_features,
};

static int bs1000_get_resources(struct platform_device *pdev,
				struct dw_pcie *pci,
				const struct baikal_pcie_of_data *data)
{
	struct bs1000_pcie *bs = platform_get_drvdata(pdev);
	struct device *dev = pci->dev;
	struct resource *res;
	int ret;

	bs->pci = pci;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res) {
		dev_err(dev, "failed to find \"dbi\" region\n");
		return -EINVAL;
	}

	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	if (BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE0_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE0_P1_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE1_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE1_P1_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE2_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(res->start) == BS1000_PCIE2_P1_DBI_BASE) {
		bs->cpu_addr_mask = 0x7fffffffff;
	} else {
		bs->cpu_addr_mask = 0xffffffffff;
	}

	bs->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(bs->apb_base))
		return PTR_ERR(bs->apb_base);

	if (data->mode == DW_PCIE_EP_TYPE) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
		if (!res)
			return -EINVAL;

		pci->ep.phys_base = res->start;
		pci->ep.addr_size = resource_size(res);
		pci->ep.ops = &bs1000_pcie_ep_ops;
		pci->dbi_base2 = pci->dbi_base + 0x100000;
	}

	return 0;
}

static u64 bs1000_pcie_cpu_addr_fixup(struct dw_pcie *pci, u64 cpu_addr)
{
	struct bs1000_pcie *bs = dev_get_drvdata(pci->dev);

	return cpu_addr & bs->cpu_addr_mask;
}

static int bs1000_pcie_link_up(struct dw_pcie *pci)
{
	struct bs1000_pcie *bs = dev_get_drvdata(pci->dev);
	u32 reg;

	reg = readl(bs->apb_base + BS1000_PCIE_APB_PE_LINK_DBG2);
	return ((reg & BAIKAL_PCIE_LTSSM_MASK) == BAIKAL_PCIE_LTSSM_STATE_L0) &&
		(reg & BS1000_PCIE_APB_PE_LINK_DBG2_SMLH_LINK_UP) &&
		(reg & BS1000_PCIE_APB_PE_LINK_DBG2_RDLH_LINK_UP);
}

static int bs1000_pcie_start_link(struct dw_pcie *pci)
{
	struct bs1000_pcie *bs = dev_get_drvdata(pci->dev);
	u32 reg;

	reg = readl(bs->apb_base + BS1000_PCIE_APB_PE_GEN_CTRL3);
	reg |= BS1000_PCIE_APB_PE_GEN_CTRL3_LTSSM_EN;
	writel(reg, bs->apb_base + BS1000_PCIE_APB_PE_GEN_CTRL3);
	return 0;
}

static int bs1000_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u16 exp_cap_off;
	u16 ext_cap_err_off;
	u32 reg;

	/* Enable error reporting */
	ext_cap_err_off = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_ERR);
	reg = dw_pcie_readl_dbi(pci, ext_cap_err_off + PCI_ERR_ROOT_COMMAND);
	reg |= PCI_ERR_ROOT_CMD_COR_EN	    |
	       PCI_ERR_ROOT_CMD_NONFATAL_EN |
	       PCI_ERR_ROOT_CMD_FATAL_EN;
	dw_pcie_writel_dbi(pci, ext_cap_err_off + PCI_ERR_ROOT_COMMAND, reg);

	exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	reg = dw_pcie_readw_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL);
	reg |= PCI_EXP_DEVCTL_CERE  |
	       PCI_EXP_DEVCTL_NFERE |
	       PCI_EXP_DEVCTL_FERE  |
	       PCI_EXP_DEVCTL_URRE;
	dw_pcie_writew_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL, reg);

	reg = dw_pcie_readl_dbi(pci, exp_cap_off + PCI_EXP_RTCTL);
	reg |= PCI_EXP_RTCTL_SECEE  |
	       PCI_EXP_RTCTL_SENFEE |
	       PCI_EXP_RTCTL_SEFEE  |
	       PCI_EXP_RTCTL_PMEIE;
	dw_pcie_writel_dbi(pci, exp_cap_off + PCI_EXP_RTCTL, reg);

#ifdef CONFIG_HOTPLUG_PCI_PCIE
	/*
	 * BE-S1000 does not support PCIe Hot-Plug interrupts
	 * so enable PCIe Hot-Plug poll mode by default
	 */
	pciehp_poll_mode = true;
#endif

	return 0;
}

static irqreturn_t bs1000_pcie_intr_irq_handler(int irq, void *arg)
{
	struct bs1000_pcie *bs = arg;
	struct dw_pcie *pci = bs->pci;
	struct device *dev = pci->dev;
	u16 exp_cap_off;
	u16 ext_cap_err_off;
	u16 dev_sta;
	u32 cor_err;
	u32 root_err;
	u32 uncor_err;
	u32 apb_pe_err;
	u32 apb_pe_int;

	apb_pe_err = readl(bs->apb_base + BS1000_PCIE_APB_PE_ERR_STS);
	apb_pe_int = readl(bs->apb_base + BS1000_PCIE_APB_PE_INT_STS);

	exp_cap_off	= dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	ext_cap_err_off = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_ERR);

	uncor_err = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_UNCOR_STATUS);
	cor_err	  = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_COR_STATUS);
	root_err  = dw_pcie_readl_dbi(pci,
				      ext_cap_err_off + PCI_ERR_ROOT_STATUS);
	dev_sta	  = dw_pcie_readw_dbi(pci,
				      exp_cap_off + PCI_EXP_DEVSTA);

	writel(apb_pe_err, bs->apb_base + BS1000_PCIE_APB_PE_ERR_STS);
	writel(apb_pe_int, bs->apb_base + BS1000_PCIE_APB_PE_INT_STS);

	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_UNCOR_STATUS, uncor_err);
	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_COR_STATUS, cor_err);
	dw_pcie_writel_dbi(pci,
			   ext_cap_err_off + PCI_ERR_ROOT_STATUS, root_err);
	dw_pcie_writew_dbi(pci,
			   exp_cap_off + PCI_EXP_DEVSTA, dev_sta);

	dev_err(dev,
		"DevSta:0x%04x RootErr:0x%x UncorErr:0x%x CorErr:0x%x ApbErr:0x%x ApbInt:0x%x\n",
		dev_sta, root_err, uncor_err, cor_err, apb_pe_err, apb_pe_int);

	return IRQ_HANDLED;
}

static u64 bs1000_pcie_edma_address(struct device *dev, phys_addr_t cpu_addr)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bs1000_pcie *bs = platform_get_drvdata(pdev);
	struct pci_bus *bus = bs->pci->pp.bridge->bus;
	struct pci_bus_region region;
	struct resource res = {
		.flags = IORESOURCE_MEM,
		.start = cpu_addr,
		.end = cpu_addr,
	};

	pcibios_resource_to_bus(bus, &region, &res);
	return region.start;
}

static const struct dw_edma_plat_ops bs1000_pcie_edma_ops = {
	.irq_vector = dw_pcie_edma_irq_vector,
	.pci_address = bs1000_pcie_edma_address,
};

static const struct dw_pcie_host_ops bs1000_pcie_host_ops = {
	.init = bs1000_pcie_host_init,
};

static int bs1000_add_pcie_port(struct platform_device *pdev)
{
	struct bs1000_pcie *bs = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = bs->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	int ret;

	pp->irq = platform_get_irq_byname_optional(pdev, "intr");
	if (pp->irq < 0) {
		/* optional */
		pp->irq = 0;
	}
	else {
		ret = devm_request_irq(dev, pp->irq, bs1000_pcie_intr_irq_handler,
					       IRQF_SHARED, "bs1000-pcie-intr", bs);
		if (ret) {
			dev_err(dev, "failed to request IRQ %d\n", pp->irq);
			return ret;
		}
	}

	pci->edma.ops = &bs1000_pcie_edma_ops;

	pp->num_vectors = MAX_MSI_IRQS;
	pp->ops = &bs1000_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)
static void dw_pcie_writel_ob_unroll(struct dw_pcie *pci, u32 index, u32 reg,
				     u32 val)
{
	dw_pcie_write(pci->atu_base +
		      PCIE_ATU_UNROLL_BASE(PCIE_ATU_REGION_DIR_OB, index) +
		      reg, 0x4, val);
}

static void bs1000_pcie_prog_outbound_atu(struct dw_pcie *pci, int index,
					  int type, u64 cpu_addr, u64 pci_addr,
					  u32 size, u32 flags)
{
	u32 retries, val;

	cpu_addr = bs1000_pcie_cpu_addr_fixup(pci, cpu_addr);

	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_BASE,
				 lower_32_bits(cpu_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_BASE,
				 upper_32_bits(cpu_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_LIMIT,
				 lower_32_bits(cpu_addr + size - 1));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_TARGET,
				 lower_32_bits(pci_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_TARGET,
				 upper_32_bits(pci_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL1, type);
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL2,
				 PCIE_ATU_ENABLE | flags);

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; ++retries) {
		dw_pcie_read(pci->atu_base +
			     PCIE_ATU_UNROLL_BASE(index, PCIE_ATU_REGION_DIR_OB) +
			     PCIE_ATU_UNR_REGION_CTRL2, 0x4, &val);
		if (val & PCIE_ATU_ENABLE)
			return;

		mdelay(LINK_WAIT_IATU);
	}
	dev_err(pci->dev, "Outbound iATU is not being enabled\n");
}

static void bs1000_pcie_setup_rc_acpi(struct dw_pcie_rp *pp,
				      const struct baikal_pcie_acpi_data *mem_data)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	acpi_status status;
	u64 lanes;
	u32 val;
	int i;

	/*
	 * Enable DBI read-only registers for writing/updating configuration.
	 * Write permission gets disabled towards the end of this function.
	 */
	dw_pcie_dbi_ro_wr_en(pci);

	val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_FAST_LINK_MODE;
	val |= PORT_LINK_DLL_LINK_EN;
	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	status = acpi_evaluate_integer(to_acpi_device(pci->dev)->handle,
				       "NUML", NULL, &lanes);
	if (ACPI_FAILURE(status)) {
		dev_dbg(pci->dev, "failed to get num-lanes\n");
	} else {
		pci->num_lanes = lanes;

		/* Set the number of lanes */
		val &= ~PORT_LINK_FAST_LINK_MODE;
		val &= ~PORT_LINK_MODE_MASK;
		switch (pci->num_lanes) {
		case 1:
			val |= PORT_LINK_MODE_1_LANES;
			break;
		case 2:
			val |= PORT_LINK_MODE_2_LANES;
			break;
		case 4:
			val |= PORT_LINK_MODE_4_LANES;
			break;
		case 8:
			val |= PORT_LINK_MODE_8_LANES;
			break;
		default:
			dev_err(pci->dev, "NUML %u: invalid value\n", pci->num_lanes);
			goto skip_lanes;
		}
		dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

		/* Set link width speed control register */
		val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
		switch (pci->num_lanes) {
		case 1:
			val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
			break;
		case 2:
			val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
			break;
		case 4:
			val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
			break;
		case 8:
			val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
			break;
		}
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	}

skip_lanes:
	/* Setup RC BARs */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

	/* Setup interrupt pins */
	val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);

	/* Setup bus numbers */
	val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* Setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	dw_pcie_cap_set(pci, IATU_UNROLL);
	pci->num_ob_windows = 4;
	pci->num_ib_windows = 0;

	for (i = 0; i < pci->num_ob_windows; ++i)
		dw_pcie_disable_atu(pci, PCIE_ATU_REGION_DIR_OB, i);

	/* Program ATU */
	bs1000_pcie_prog_outbound_atu(pci, 0, PCIE_ATU_TYPE_CFG0,
				      pp->cfg0_base, 0,
				      SZ_2M,
				      PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE);
	bs1000_pcie_prog_outbound_atu(pci, 1, PCIE_ATU_TYPE_CFG1,
				      pp->cfg0_base, 0,
				      pp->cfg0_size,
				      PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE);
	bs1000_pcie_prog_outbound_atu(pci, 2, PCIE_ATU_TYPE_MEM,
				      mem_data->mem_base, mem_data->mem_bus_addr,
				      mem_data->mem_size, 0);
	bs1000_pcie_prog_outbound_atu(pci, 3, PCIE_ATU_TYPE_IO,
				      pp->io_base, pp->io_bus_addr,
				      pp->io_size, 0);

	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);

	/* Set eDMA region */
	pci->edma.reg_base = pci->atu_base + DEFAULT_DBI_DMA_OFFSET;

	/* Program correct class for RC */
	dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_PCI);

	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	dw_pcie_dbi_ro_wr_dis(pci);
}

static int bs1000_pcie_get_res_acpi(struct acpi_device *adev,
				    struct acpi_device **res_dev,
				    struct bs1000_pcie *bs,
				    struct baikal_pcie_acpi_data *mem_data)
{
	struct device *dev = &adev->dev;
	struct dw_pcie_rp *pp = &bs->pci->pp;
	struct resource_entry *entry;
	struct list_head list, *pos;
	struct fwnode_handle *fwnode;
	int ret;
	unsigned long flags = IORESOURCE_MEM;

	fwnode = fwnode_get_named_child_node(&adev->fwnode, "RES0");
	if (!fwnode) {
		dev_err(dev, "failed to get RES0 subdevice\n");
		return -EINVAL;
	}

	*res_dev = to_acpi_device_node(fwnode);
	if (!*res_dev) {
		dev_err(dev, "RES0 is not an acpi device node\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(*res_dev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse RES0._CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 4) {
		dev_err(dev,
			"invalid number of MEM resources present in RES0._CRS (%i, need 4)\n", ret);
		return -EINVAL;
	}

	/* ECAM */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->cfg0_size = resource_size(entry->res);
	pp->cfg0_base = entry->res->start;

	/* DBI */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	if (BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE0_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE0_P1_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE1_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE1_P1_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE2_P0_DBI_BASE ||
	    BS1000_ADDR_IN_CHIP(entry->res->start) == BS1000_PCIE2_P1_DBI_BASE) {
		bs->cpu_addr_mask = 0x7fffffffff;
	} else {
		bs->cpu_addr_mask = 0xffffffffff;
	}
	bs->pci->dbi_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(bs->pci->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(bs->pci->dbi_base);
		return ret;
	}

	/* ATU */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	bs->pci->atu_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(bs->pci->atu_base)) {
		dev_err(dev, "error with atu ioremap\n");
		ret = PTR_ERR(bs->pci->atu_base);
		return ret;
	}
	bs->pci->atu_size = resource_size(entry->res);

	/* APB */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	bs->apb_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(bs->apb_base)) {
		dev_err(dev, "error with apb ioremap\n");
		ret = PTR_ERR(bs->apb_base);
		return ret;
	}

	acpi_dev_free_resource_list(&list);

	/* Non-prefetchable memory */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_MEM;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	mem_data->mem_base = entry->res->start;
	mem_data->mem_size = resource_size(entry->res);
	mem_data->mem_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);

	/* I/O */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_IO;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *)flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of IO resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->io_base = entry->res->start;
	pp->io_size = resource_size(entry->res);
	pp->io_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);
	return 0;
}

static int bs1000_pcie_get_irq_acpi(struct device *dev,
				    struct acpi_device *res_dev,
				    struct bs1000_pcie *bs)
{
	struct dw_pcie_rp *pp = &bs->pci->pp;
	struct resource res;
	int index, ret = 0;

	memset(&res, 0, sizeof(res));

	/* eDMA interrupts */
	for (index = 0; index < BAIKAL_EDMA_WR_CH + BAIKAL_EDMA_RD_CH; index++) {
		ret = acpi_irq_get(res_dev->handle, index, &res);
		if (ret)
			break;
		if (res.flags & IORESOURCE_BITS) {
			struct irq_data *irqd;

			irqd = irq_get_irq_data(res.start);
			if (!irqd)
				return -ENXIO;

			irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
		}
		bs->edma_irq[index] = res.start;
	}

	/* RC interrupts */
	if (ret == 0)
		ret = acpi_irq_get(res_dev->handle, index, &res);
	if (ret) {
		/* optional */
		pp->irq = 0;
		return 0;
	}

	if (res.flags & IORESOURCE_BITS) {
		struct irq_data *irqd;

		irqd = irq_get_irq_data(res.start);
		if (!irqd)
			return -ENXIO;

		irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
	}

	pp->irq = res.start;

	ret = devm_request_irq(dev, pp->irq, bs1000_pcie_intr_irq_handler,
			       IRQF_SHARED, "bs1000-pcie-intr", bs);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d\n", pp->irq);
		return ret;
	}

	return 0;
}

static int bs1000_get_acpi_data(struct device *dev, struct bs1000_pcie *bs,
				struct baikal_pcie_acpi_data *mem_data)
{
	struct acpi_device *adev = to_acpi_device(dev), *res_dev;
	int ret;

	ret = bs1000_pcie_get_res_acpi(adev, &res_dev, bs, mem_data);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	ret = bs1000_pcie_get_irq_acpi(dev, res_dev, bs);
	if (ret) {
		dev_err(dev, "failed to get irq info\n");
		return ret;
	}

	return 0;
}

static int bs1000_pcie_acpi_edma_irq_vector(struct device *dev, unsigned int nr)
{
	struct bs1000_pcie *bs = dev_get_drvdata(dev);

	if (nr >= BAIKAL_EDMA_WR_CH + BAIKAL_EDMA_RD_CH)
		return -EINVAL;

	return bs->edma_irq[nr];
}

static u64 bs1000_pcie_acpi_edma_address(struct device *dev, phys_addr_t cpu_addr)
{
	struct bs1000_pcie *bs = dev_get_drvdata(dev);
	struct pci_bus *bus = bs->pci->pp.bridge->bus;
	struct pci_bus_region region;
	struct resource res = {
		.flags = IORESOURCE_MEM,
		.start = cpu_addr,
		.end = cpu_addr,
	};

	pcibios_resource_to_bus(bus, &region, &res);
	return region.start;
}

static const struct dw_edma_plat_ops bs1000_pcie_acpi_edma_ops = {
	.irq_vector = bs1000_pcie_acpi_edma_irq_vector,
	.pci_address = bs1000_pcie_acpi_edma_address,
};

static int bs1000_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct bs1000_pcie *bs;
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;
	struct baikal_pcie_acpi_data mem_data = {};
	int ret;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &bs1000_pcie_rc_of_data.dw_pcie_ops;

	bs = devm_kzalloc(dev, sizeof(*bs), GFP_KERNEL);
	if (!bs)
		return -ENOMEM;

	cfg->priv = bs;
	bs->pci = pci;
	dev_set_drvdata(dev, bs);

	ret = bs1000_get_acpi_data(dev, bs, &mem_data);
	if (ret) {
		dev_err(dev, "failed to get data from ACPI\n");
		return ret;
	}

	pp = &pci->pp;
	raw_spin_lock_init(&pp->lock);
	pp->ops = &bs1000_pcie_host_ops;
	pp->va_cfg0_base = devm_pci_remap_cfgspace(dev, pp->cfg0_base,
						   pp->cfg0_size);
	if (!pp->va_cfg0_base) {
		dev_err(dev, "error with ioremap\n");
		return -ENOMEM;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "failed to enable DMA\n");
		return ret;
	}

	ret = bs1000_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	dw_pcie_version_detect(pci);
	bs1000_pcie_setup_rc_acpi(pp, &mem_data);

	/* eDMA */
	pci->edma.nr_irqs = BAIKAL_EDMA_WR_CH + BAIKAL_EDMA_RD_CH;
	pci->edma.ops = &bs1000_pcie_acpi_edma_ops;
	ret = dw_pcie_edma_detect(pci);
	if (ret) {
		dev_err(dev, "failed to initialize eDMA\n");
		return ret;
	}

	return 0;
}

static void __iomem *bs1000_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct bs1000_pcie *bs = cfg->priv;
	unsigned int devfn_shift = cfg->ops->bus_shift - 8;
	unsigned int busn = bus->number;
	void __iomem *base;

	if (!bs->pci->pp.bridge)
		bs->pci->pp.bridge = to_pci_host_bridge(bus->bridge);

	if (bus->number != cfg->busr.start && !bs1000_pcie_link_up(bs->pci))
		return NULL;

	if (bus->number == cfg->busr.start) {
		/*
		 * The DW PCIe core doesn't filter out transactions to other
		 * devices/functions on the root bus num, so we do this here.
		 */
		if (PCI_SLOT(devfn) > 0)
			return NULL;
		else
			return bs->pci->dbi_base + where;
	}

	if (busn < cfg->busr.start || busn > cfg->busr.end)
		return NULL;

	busn -= cfg->busr.start;
	base = cfg->win + (busn << cfg->ops->bus_shift);
	return base + (devfn << devfn_shift) + where;
}

const struct pci_ecam_ops baikal_s_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= bs1000_pcie_init,
	.pci_ops	= {
		.map_bus	= bs1000_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif

union baikal_pcie {
	struct bm1000_pcie	bm1000_pcie;
	struct bs1000_pcie	bs1000_pcie;
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	union baikal_pcie *bp;
	const struct baikal_pcie_of_data *data;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	bp = devm_kzalloc(dev, sizeof(*bp), GFP_KERNEL);
	if (!bp)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &data->dw_pcie_ops;

	platform_set_drvdata(pdev, bp);
	ret = data->get_resources(pdev, pci, data);
	if (ret)
		return ret;

	switch (data->mode) {
	case DW_PCIE_RC_TYPE:
		ret = data->add_pcie_port(pdev);
		if (ret)
			return ret;

		break;
	case DW_PCIE_EP_TYPE:
		ret = dw_pcie_ep_init(&pci->ep);
		if (ret) {
			dev_err(dev, "failed to initialize endpoint\n");
			return ret;
		}

		break;
	default:
		dev_err(dev, "INVALID device type %d\n", data->mode);
		return -EINVAL;
	}

	return 0;
}

static const struct baikal_pcie_of_data bm1000_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
	.dw_pcie_ops = {
		.link_up	= bm1000_pcie_link_up,
		.start_link	= bm1000_pcie_start_link,
	},
	.get_resources = bm1000_get_resources,
	.add_pcie_port = bm1000_add_pcie_port,
};

static const struct baikal_pcie_of_data bs1000_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
	.dw_pcie_ops = {
		.cpu_addr_fixup	= bs1000_pcie_cpu_addr_fixup,
		.link_up	= bs1000_pcie_link_up,
		.start_link	= bs1000_pcie_start_link,
	},
	.get_resources = bs1000_get_resources,
	.add_pcie_port = bs1000_add_pcie_port,
};

static const struct baikal_pcie_of_data bs1000_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
	.dw_pcie_ops = {
		.cpu_addr_fixup	= bs1000_pcie_cpu_addr_fixup,
		.link_up	= bs1000_pcie_link_up,
		.start_link	= bs1000_pcie_start_link,
	},
	.get_resources = bs1000_get_resources,
};

static const struct of_device_id baikal_pcie_of_match[] = {
	{
		.compatible = "baikal,bm1000-pcie",
		.data = &bm1000_pcie_rc_of_data,
	},
	{
		.compatible = "baikal,bs1000-pcie",
		.data = &bs1000_pcie_rc_of_data,
	},
	{
		.compatible = "baikal,bs1000-pcie-ep",
		.data = &bs1000_pcie_ep_of_data,
	},
	{ },
};

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name = "baikal-pcie",
		.of_match_table = baikal_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = baikal_pcie_probe,
};
builtin_platform_driver(baikal_pcie_driver);
