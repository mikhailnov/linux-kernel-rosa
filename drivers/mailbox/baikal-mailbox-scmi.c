// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics Mailbox Doorbell driver for SCMI protocol
 *
 * Copyright (C) 2025 Baikal Electronics, JSC
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#define BAIKAL_MBOX_IRB_AP2SCP_BASE	0x0
#define BAIKAL_MBOX_IRB_SCP2AP_BASE	0x8000

#define BAIKAL_MBOX_IRB_STATUS		0x0
#define BAIKAL_MBOX_IRB_SET		0x4
#define BAIKAL_MBOX_IRB_CLEAR		0x8

#define BAIKAL_MBOX_POLL_PERIOD		5	/* ms */

struct baikal_mbox {
	struct device *dev;
	void __iomem *ap2scp, *scp2ap;
	struct mbox_chan chan;
	struct mbox_controller controller;
};

static struct baikal_mbox *to_baikal_mbox(struct mbox_controller *controller)
{
	return container_of(controller, struct baikal_mbox, controller);
}

static irqreturn_t baikal_mbox_irq(int irq, void *data)
{
	struct baikal_mbox *mbox = data;
	u32 status = readl(mbox->scp2ap + BAIKAL_MBOX_IRB_STATUS);

	if (status) {
		mbox_chan_received_data(&mbox->chan, &status);
		writel(0xffffffff, mbox->scp2ap + BAIKAL_MBOX_IRB_CLEAR);
	}

	return IRQ_HANDLED;
}

static int baikal_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct baikal_mbox *mbox = to_baikal_mbox(chan->mbox);
	u32 msg = 1;
	int err = -EBUSY;

	if (!readl(mbox->ap2scp + BAIKAL_MBOX_IRB_STATUS)) {
		writel(msg, mbox->ap2scp + BAIKAL_MBOX_IRB_SET);
		err = 0;
	}
	return err;
}

static bool baikal_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct baikal_mbox *mbox = to_baikal_mbox(chan->mbox);

	return !readl(mbox->ap2scp + BAIKAL_MBOX_IRB_STATUS);
}

static const struct mbox_chan_ops baikal_mbox_ops = {
	.send_data	= baikal_mbox_send_data,
	.last_tx_done	= baikal_mbox_last_tx_done
};

static int baikal_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_mbox *mbox;
	void __iomem *irb;
	int ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	irb = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(irb)) {
		dev_err(dev, "Couldn't map IRB registers (%ld)\n",
			PTR_ERR(irb));
		return PTR_ERR(irb);
	}
	mbox->ap2scp = irb + BAIKAL_MBOX_IRB_AP2SCP_BASE;
	mbox->scp2ap = irb + BAIKAL_MBOX_IRB_SCP2AP_BASE;

	mbox->dev = dev;

	mbox->controller.dev = dev;
	mbox->controller.chans = &mbox->chan;
	mbox->controller.num_chans = 1;
	mbox->controller.ops = &baikal_mbox_ops;
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = BAIKAL_MBOX_POLL_PERIOD;

	platform_set_drvdata(pdev, mbox);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "Couldn't find IRB IRQ (%d)\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, ret, NULL, baikal_mbox_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					pdev->name, mbox);
	if (ret) {
		dev_err(dev, "Couldn't request IRB IRQ (%d)\n", ret);
		return ret;
	}

	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret) {
		dev_err(dev, "Failed to register mailbox (%d)\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id baikal_mbox_of_match[] = {
	{ .compatible = "baikal,baikal-mbox-scmi", },
	{},
};
MODULE_DEVICE_TABLE(of, baikal_mbox_of_match);

static struct platform_driver baikal_mbox_driver = {
	.probe  = baikal_mbox_probe,
	.driver = {
		.name = "baikal-mbox-scmi",
		.of_match_table = baikal_mbox_of_match,
	},
};

module_platform_driver(baikal_mbox_driver);

MODULE_DESCRIPTION("Baikal Mailbox SCMI driver");
MODULE_LICENSE("GPL v2");
