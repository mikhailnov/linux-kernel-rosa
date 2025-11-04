// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal Electronics eSPI controller driver
 * Copyright (C) 2020-2023 Baikal Electronics, JSC
 */

#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/io.h>

#define ESPI_FIFO_LEN	256
#define ESPI_DUMMY_DATA	0xff

struct espi_data {
	struct spi_controller	*master;
	/* hardware */
	int			irq;
	void __iomem		*regs;
	struct clk		*clk;
	/* transfer */
	u32			rxlen;
	u32			txlen;
	u8			*tx;
	u8			*rx;
};

#define ESPI_CR1	0x00 /* control 1 */
#define ESPI_CR2	0x04 /* control 2 */
#define ESPI_TX_FIFO	0x0c /* tx fifo */
#define ESPI_TX_FBCAR	0x14 /* tx byte count */
#define ESPI_TX_FAETR	0x18 /* tx almost empty threshold */
#define ESPI_RX_FIFO	0x1c /* rx fifo */
#define ESPI_RX_FBCAR	0x24 /* rx byte count */
#define ESPI_RX_FAFTR	0x28 /* rx almost full threshold */
#define ESPI_ISR	0x30 /* irq status */
#define ESPI_IMR	0x34 /* irq mask */
#define ESPI_IVR	0x38 /* irq vector */

union espi_cr1 {
	u32 val;
	struct {
		u32 scr :1; /* reset */
		u32 sce :1; /* enable */
		u32 mss :1; /* select - master/slave */
		u32 cph :1; /* clock phase */
		u32 cpo :1; /* clock polarity */
	} bits;
};

#define ESPI_CR1_SCR_RESET		0
#define ESPI_CR1_SCR_NORESET		1
#define ESPI_CR1_SCE_CORE_DISABLE	0
#define ESPI_CR1_SCE_CORE_ENABLE	1
#define ESPI_CR1_MSS_MODE_MASTER	0
#define ESPI_CR1_MSS_MODE_SLAVE		1
#define ESPI_CR1_CPH_CLKPHASE_ODD	0
#define ESPI_CR1_CPH_CLKPHASE_EVEN	1
#define ESPI_CR1_CPO_CLKPOLAR_LOW	0
#define ESPI_CR1_CPO_CLKPOLAR_HIGH	1

union espi_cr2 {
	u32 val;
	struct {
		u32 sso :3; /* slave select (ss0-ss7) */
		u32 srd :1; /* read disable */
		u32 sri :1; /* read first byte ignore */
		u32 mlb :1; /* least sign git first */
		u32 mte :1; /* master transfer enable */
	} bits;
};

#define ESPI_CR2_SRD_RX_DISABLE		0
#define ESPI_CR2_SRD_RX_ENABLE		1
#define ESPI_CR2_SRI_FIRST_RESIEV	0
#define ESPI_CR2_SRI_FIRST_IGNORE	1
#define ESPI_CR2_MLB_MSB		0
#define ESPI_CR2_MLB_LSB		1
#define ESPI_CR2_MTE_TX_DISABLE		0
#define ESPI_CR2_MTE_TX_ENABLE		1 /* self clear */

#define ESPI_RX_FAFTR_DISABLE		0xff
#define ESPI_TX_FAETR_DISABLE		0x0
#define ESPI_ISR_CLEAR_ALL		0xff /* clear by writing "1" */
#define ESPI_IMR_DISABLE_ALL		0x0
#define ESPI_IMR_ENABLE_ALL		0xff

/* isr, imr, ivr */
union espi_irq {
	u32 val;
	struct {
		u32 tx_underrun		:1;
		u32 tx_overrun		:1;
		u32 rx_underrun		:1;
		u32 rx_overrun		:1;
		u32 tx_almost_empty	:1;
		u32 rx_almost_full	:1;
		u32 tx_done_master	:1;
		u32 tx_done_slave	:1;
		u32 alert		:8;
		u32 rx_crc_error	:1;
	} bits;
};

#define ESPI_IRQ_DISABLE	0
#define ESPI_IRQ_ENABLE		1

static inline uint32_t espi_readl(struct espi_data *priv, uint32_t offset)
{
	return __raw_readl(priv->regs + offset);
}

static inline void espi_writel(struct espi_data *priv, uint32_t offset, uint32_t val)
{
	__raw_writel(val, priv->regs + offset);
}

static void espi_set_cs(struct spi_device *spi, bool enable);
static int  espi_setup(struct spi_device *spi);
static void espi_writer(struct espi_data *priv);
static void espi_reader(struct espi_data *priv);
static void espi_reset(struct espi_data *priv);
static void espi_enable_tx(struct espi_data *priv);
static void espi_enable_rx(struct espi_data *priv);

static void espi_reset(struct espi_data *priv)
{
	union espi_cr1 cr1;

	cr1.val = espi_readl(priv, ESPI_CR1);
	cr1.bits.scr = ESPI_CR1_SCR_RESET;
	espi_writel(priv, ESPI_CR1, cr1.val);
}

static void espi_enable_rx(struct espi_data *priv)
{
	union espi_cr2 cr2;

	cr2.val = espi_readl(priv, ESPI_CR2);
	cr2.bits.srd = ESPI_CR2_SRD_RX_ENABLE;
	espi_writel(priv, ESPI_CR2, cr2.val);
}

static void espi_enable_tx(struct espi_data *priv)
{
	union espi_cr2 cr2;
	int cnt = espi_readl(priv, ESPI_TX_FBCAR);

	cr2.val = espi_readl(priv, ESPI_CR2);

	if (cnt && cr2.bits.mte != ESPI_CR2_MTE_TX_ENABLE) {
		cr2.bits.mte = ESPI_CR2_MTE_TX_ENABLE;
		espi_writel(priv, ESPI_CR2, cr2.val);
	}
}

static void espi_set_cs(struct spi_device *spi, bool enable)
{
	struct espi_data *priv;
	union espi_cr2 cr2;

	priv = spi_controller_get_devdata(spi->controller);
	cr2.val = espi_readl(priv, ESPI_CR2);
	cr2.bits.sso = spi_get_chipselect(spi, 0);
	espi_writel(priv, ESPI_CR2, cr2.val);
}

static void espi_writer(struct espi_data *priv)
{
	u8 data;
	u32 have_tx = espi_readl(priv, ESPI_TX_FBCAR);
	u32 have_rx = espi_readl(priv, ESPI_RX_FBCAR);
	u32 free = ESPI_FIFO_LEN - have_tx - have_rx;
	u32 cnt = min(free, priv->txlen);

	if (!cnt)
		return;

	priv->txlen -= cnt;

	/* fifo */
	while (cnt--) {
		if (priv->tx)
			data = *priv->tx++;
		else
			data = ESPI_DUMMY_DATA;

		espi_writel(priv, ESPI_TX_FIFO, data);
	}

	espi_enable_tx(priv);
}

static void espi_reader(struct espi_data *priv)
{
	u8 data;
	u32 have = espi_readl(priv, ESPI_RX_FBCAR);
	u32 cnt = min(have, priv->rxlen);

	if (!cnt)
		return;

	priv->rxlen -= cnt;

	/* fifo */
	while (cnt--) {
		data = espi_readl(priv, ESPI_RX_FIFO);
		if (priv->rx)
			*priv->rx++ = data;
	}
}

static irqreturn_t espi_irq_handler(int irq, void *dev)
{
	union espi_irq status;
	struct spi_controller *master = dev;
	struct espi_data *priv = spi_controller_get_devdata(master);
	int ret = -1;

	status.val = espi_readl(priv, ESPI_IVR);
	espi_writel(priv, ESPI_ISR, status.val);

	if (!status.val) {
		dev_err(&master->dev, "irq_none\n");
		return IRQ_NONE;
	}

	/* errors */
	if (status.bits.tx_underrun) {
		dev_err(&master->dev, "tx_underrun\n");
		goto exit;
	}
	if (status.bits.tx_overrun) {
		dev_err(&master->dev, "tx_overrun\n");
		goto exit;
	}
	if (status.bits.rx_underrun) {
		dev_err(&master->dev, "rx_underrun\n");
		goto exit;
	}
	if (status.bits.rx_overrun) {
		dev_err(&master->dev, "rx_overrun\n");
		goto exit;
	}
	if (status.bits.rx_crc_error) {
		dev_err(&master->dev, "rx_crc_error\n");
		goto exit;
	}
	if (status.bits.alert) {
		dev_err(&master->dev, "alert\n");
		goto exit;
	}

	/* transfer */
	espi_reader(priv);
	espi_writer(priv);
	ret = 0;

exit:
	if (ret) {
		master->cur_msg->status = -EIO;
		priv->txlen = 0;
		priv->rxlen = 0;
		espi_reset(priv);
	}

	/* tx done */
	if (priv->txlen == 0) {
		union espi_irq mask;

		mask.val = espi_readl(priv, ESPI_IMR);
		mask.bits.tx_almost_empty = ESPI_IRQ_DISABLE;
		espi_writel(priv, ESPI_IMR, mask.val);
	}

	/* done */
	if (priv->txlen == 0 && priv->rxlen == 0) {
		espi_writel(priv, ESPI_IMR, ESPI_IMR_DISABLE_ALL);
		espi_writel(priv, ESPI_ISR, ESPI_ISR_CLEAR_ALL);
		spi_finalize_current_transfer(master);
	}

	return IRQ_HANDLED;
}

static u32 espi_get_rate(struct espi_data *priv)
{
	if (acpi_disabled) {
		return clk_get_rate(priv->clk);
#ifdef CONFIG_ACPI
	} else {
		struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	        union acpi_object *obj;
		acpi_status status;
		u64 val;

		status = acpi_evaluate_object(ACPI_COMPANION(&priv->master->dev)->handle,
					      "CGET", NULL, &buffer);
		if (ACPI_FAILURE(status))
			return 0;

		obj = buffer.pointer;
		if (!obj || obj->type != ACPI_TYPE_INTEGER) {
			kfree(obj);
			return 0;
		}

		val = obj->integer.value;
		kfree(obj);

		return val;
#endif
	}
}

static void espi_set_rate(struct espi_data *priv, u32 rate)
{
	if (acpi_disabled) {
		clk_set_rate(priv->clk, rate);
#ifdef CONFIG_ACPI
	} else {
	        union acpi_object obj = {
	                .type = ACPI_TYPE_INTEGER,
	                .integer.type = ACPI_TYPE_INTEGER,
	                .integer.value = rate
	        };
	        struct acpi_object_list args = {
	                .count = 1,
	                .pointer = &obj
	        };

		acpi_evaluate_object(ACPI_COMPANION(&priv->master->dev)->handle,
						    "CSET", &args, NULL);
#endif
	}
}

static int espi_transfer_one(struct spi_controller *master,
			     struct spi_device *spi,
			     struct spi_transfer *transfer)
{
	struct espi_data *priv = spi_controller_get_devdata(spi->controller);

	priv->tx = (void *)transfer->tx_buf;
	priv->rx = (void *)transfer->rx_buf;
	priv->txlen = transfer->len;
	priv->rxlen = transfer->len;

	/* clk */
	if (transfer->speed_hz && transfer->speed_hz != spi->max_speed_hz)
		espi_set_rate(priv, transfer->speed_hz);

	/* transfer */
	espi_writer(priv);
	espi_writel(priv, ESPI_IMR, ESPI_IMR_ENABLE_ALL);

	return 1;
}

static int espi_setup(struct spi_device *spi)
{
	struct espi_data *priv = spi_controller_get_devdata(spi->controller);
	union espi_cr1 cr1;
	union espi_cr2 cr2;

	espi_reset(priv);

	/* irq */
	espi_writel(priv, ESPI_IMR, ESPI_IMR_DISABLE_ALL);
	espi_writel(priv, ESPI_ISR, ESPI_ISR_CLEAR_ALL);

	/* control 1 */
	cr1.val = espi_readl(priv, ESPI_CR1);
	cr1.bits.scr = ESPI_CR1_SCR_NORESET;
	cr1.bits.sce = ESPI_CR1_SCE_CORE_ENABLE;
	cr1.bits.mss = ESPI_CR1_MSS_MODE_MASTER;
	cr1.bits.cph = spi->mode & SPI_CPHA ? ESPI_CR1_CPH_CLKPHASE_EVEN :
					      ESPI_CR1_CPH_CLKPHASE_ODD;
	cr1.bits.cpo = spi->mode & SPI_CPOL ? ESPI_CR1_CPO_CLKPOLAR_HIGH :
					      ESPI_CR1_CPO_CLKPOLAR_LOW;
	espi_writel(priv, ESPI_CR1, cr1.val);

	/* control 2 */
	cr2.val = espi_readl(priv, ESPI_CR2);
	cr2.bits.srd = ESPI_CR2_SRD_RX_DISABLE;
	cr2.bits.mte = ESPI_CR2_MTE_TX_DISABLE;
	cr2.bits.sri = ESPI_CR2_SRI_FIRST_RESIEV;
	cr2.bits.mlb = spi->mode & SPI_LSB_FIRST ? ESPI_CR2_MLB_LSB :
						   ESPI_CR2_MLB_MSB;
	espi_writel(priv, ESPI_CR2, cr2.val);

	/* threshold */
	espi_writel(priv, ESPI_TX_FAETR, ESPI_FIFO_LEN / 2);
	espi_writel(priv, ESPI_RX_FAFTR, ESPI_FIFO_LEN / 2);

	/* full duplex */
	espi_enable_rx(priv);

	/* clk */
	if (spi->max_speed_hz != espi_get_rate(priv)) {
		espi_set_rate(priv, spi->max_speed_hz);
		spi->max_speed_hz = espi_get_rate(priv);
	}

	return 0;
}

static int espi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct espi_data *priv;
	struct resource *mem;
	int err;
	struct spi_controller *master;

	/* init */
	priv = devm_kzalloc(dev, sizeof(struct espi_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	/* reg */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	priv->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(priv->regs)) {
		dev_err(dev, "region map failed\n");
		return PTR_ERR(priv->regs);
	}

	/* irq */
	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "no irq resource\n");
		return priv->irq;
	}

	/* clk */
	if (is_of_node(dev->fwnode)) {
		priv->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(priv->clk)) {
			dev_err(dev, "could not get spi clock\n");
			return PTR_ERR(priv->clk);
		}
		err = clk_prepare_enable(priv->clk);
		if (err) {
			dev_err(dev, "could not prepare spi clock\n");
			return err;
		}
	}

	/* master */
	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	priv->master = master;
	master->num_chipselect = 8;
	master->use_gpio_descriptors = true;
	master->mode_bits = SPI_CPHA | SPI_CPOL;
	master->dev.of_node = dev->of_node;
	master->dev.fwnode = dev->fwnode;
	master->transfer_one = espi_transfer_one;
	master->set_cs = espi_set_cs;
	master->setup = espi_setup;
	master->bus_num = of_alias_get_id(master->dev.of_node, "spi");
	master->dev.of_node = dev->of_node;
	master->dev.fwnode = dev->fwnode;
	master->flags = SPI_CONTROLLER_GPIO_SS;

	err = devm_request_irq(dev, priv->irq, espi_irq_handler,
			       IRQF_SHARED, pdev->name, master);
	if (err) {
		dev_err(&pdev->dev, "unable to request irq %d\n", priv->irq);
		return err;
	}

	spi_controller_set_devdata(master, priv);

	err = devm_spi_register_controller(dev, master);
	if (err) {
		dev_err(&master->dev, "problem registering spi master\n");
		spi_controller_put(master);
		return err;
	}

	return 0;
}

static void espi_remove(struct platform_device *pdev)
{
	struct espi_data *priv = platform_get_drvdata(pdev);

	espi_reset(priv);
	clk_disable_unprepare(priv->clk);
}

static const struct of_device_id baikal_espi_of_match[] = {
	{ .compatible = "baikal,bm1000-espi" },
	{ .compatible = "baikal,bs1000-espi" },
	{ }
};
MODULE_DEVICE_TABLE(of, baikal_espi_of_match);

static struct platform_driver baikal_espi_driver = {
	.probe		= espi_probe,
	.remove		= espi_remove,
	.driver		= {
		.name	= "baikal-espi",
		.of_match_table = baikal_espi_of_match
	}
};
module_platform_driver(baikal_espi_driver);

MODULE_DESCRIPTION("Baikal eSPI controller driver");
MODULE_LICENSE("GPL v2");
