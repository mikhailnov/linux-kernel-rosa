// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics Mailbox SMS protocol driver
 *
 * Copyright (C) 2024 Baikal Electronics, JSC
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define BAIKAL_MBOX_IRB_AP2SCP_BASE	0x0
#define BAIKAL_MBOX_IRB_SCP2AP_BASE	0x8000

#define BAIKAL_MBOX_IRB_STATUS		0x0
#define BAIKAL_MBOX_IRB_SET		0x4
#define BAIKAL_MBOX_IRB_CLEAR		0x8

#define BAIKAL_MBOX_SMS_REQUEST		BIT(31)
#define BAIKAL_MBOX_SMS_SERVICE_ID	GENMASK(30, 24)
#define BAIKAL_MBOX_SMS_PARAMETER	GENMASK(23, 0)

#define BAIKAL_MBOX_CHANNELS		3
#define BAIKAL_MBOX_POLL_PERIOD		5	/* ms */

struct baikal_mbox {
	struct device *dev;
	void __iomem *ap2scp, *scp2ap;
	spinlock_t lock;
	struct mbox_chan chan[BAIKAL_MBOX_CHANNELS];
	struct mbox_controller controller;
	struct list_head list;
};

static struct baikal_mbox *to_baikal_mbox(struct mbox_controller *controller)
{
	return container_of(controller, struct baikal_mbox, controller);
}

static LIST_HEAD(baikal_mboxes);
static DEFINE_MUTEX(baikal_mboxes_mutex);

struct mbox_chan *baikal_mbox_sms_request_channel(struct mbox_client *cl);

struct mbox_chan *baikal_mbox_sms_request_channel(struct mbox_client *cl)
{
	struct device *dev = cl->dev;
	struct fwnode_reference_args args;
	struct fwnode_handle *fwnode = NULL;
	struct baikal_mbox *mbox;
	struct mbox_chan *chan;
	unsigned int idx;
	int ret;

	if (!dev || !dev_fwnode(dev)) {
		pr_debug("%s: No owner device node\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	if (fwnode_property_get_reference_args(dev_fwnode(dev), "sms",
					       NULL, 1, 0, &args)) {
		dev_dbg(dev, "%s: can't parse \"sms\" property\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	fwnode = args.fwnode;
	if (!fwnode) {
		dev_dbg(dev, "%s: mbox device not found\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	idx = (args.nargs > 0) ? args.args[0] : 0;
	if (idx >= BAIKAL_MBOX_CHANNELS) {
		fwnode_handle_put(fwnode);
		dev_dbg(dev, "%s: wrong mbox channel number\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	chan = ERR_PTR(-EPROBE_DEFER);
	mutex_lock(&baikal_mboxes_mutex);
	list_for_each_entry(mbox, &baikal_mboxes, list) {
		if (dev_fwnode(mbox->dev) == fwnode) {
			chan = &mbox->chan[idx];
			if (chan->cl)
				chan = ERR_PTR(-EBUSY);
			break;
		}
	}
	mutex_unlock(&baikal_mboxes_mutex);

	fwnode_handle_put(fwnode);

	if (IS_ERR(chan)) {
		dev_dbg(dev, "%s: mbox device not found\n", __func__);
		return chan;
	}

	ret =  mbox_bind_client(chan, cl);
	if (ret)
		chan = ERR_PTR(ret);

	return chan;
}
EXPORT_SYMBOL(baikal_mbox_sms_request_channel);

static irqreturn_t baikal_mbox_irq(int irq, void *data)
{
	struct baikal_mbox *mbox = data;
	u32 msg = readl(mbox->scp2ap + BAIKAL_MBOX_IRB_STATUS);
	unsigned int id = FIELD_GET(BAIKAL_MBOX_SMS_SERVICE_ID, msg);

	if (msg & BAIKAL_MBOX_SMS_REQUEST) {
		if (id < BAIKAL_MBOX_CHANNELS) {
			msg = FIELD_GET(BAIKAL_MBOX_SMS_PARAMETER, msg);
			mbox_chan_received_data(&mbox->chan[id], &msg);
		}
		writel(0xffffffff, mbox->scp2ap + BAIKAL_MBOX_IRB_CLEAR);
	}

	return IRQ_HANDLED;
}

static int baikal_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct baikal_mbox *mbox = to_baikal_mbox(chan->mbox);
	unsigned int id = chan - mbox->chan;
	u32 msg = BAIKAL_MBOX_SMS_REQUEST |
		  FIELD_PREP(BAIKAL_MBOX_SMS_SERVICE_ID, id) |
		  FIELD_PREP(BAIKAL_MBOX_SMS_PARAMETER, *(u32 *)data);
	int ret = -EBUSY;

	spin_lock(&mbox->lock);
	if (!readl(mbox->ap2scp + BAIKAL_MBOX_IRB_STATUS)) {
		writel(msg, mbox->ap2scp + BAIKAL_MBOX_IRB_SET);
		ret = 0;
	}
	spin_unlock(&mbox->lock);
	return ret;
}

static bool baikal_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct baikal_mbox *mbox = to_baikal_mbox(chan->mbox);
	bool ret;

	spin_lock(&mbox->lock);
	ret = !readl(mbox->ap2scp + BAIKAL_MBOX_IRB_STATUS);
	spin_unlock(&mbox->lock);
	return ret;
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
	spin_lock_init(&mbox->lock);

	mbox->controller.dev = dev;
	mbox->controller.chans = mbox->chan;
	mbox->controller.num_chans = BAIKAL_MBOX_CHANNELS;
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

	mutex_lock(&baikal_mboxes_mutex);
	list_add_tail(&mbox->list, &baikal_mboxes);
	mutex_unlock(&baikal_mboxes_mutex);

	return 0;
}

static void baikal_mbox_remove(struct platform_device *pdev)
{
	struct baikal_mbox *mbox = platform_get_drvdata(pdev);

	mutex_lock(&baikal_mboxes_mutex);
	list_del(&mbox->list);
	mutex_unlock(&baikal_mboxes_mutex);
}

static const struct of_device_id baikal_mbox_of_match[] = {
	{ .compatible = "baikal,baikal-mbox-sms", },
	{},
};
MODULE_DEVICE_TABLE(of, baikal_mbox_of_match);

static struct platform_driver baikal_mbox_driver = {
	.probe  = baikal_mbox_probe,
	.remove_new = baikal_mbox_remove,
	.driver = {
		.name = "baikal-mbox-sms",
		.of_match_table = baikal_mbox_of_match,
	},
};

module_platform_driver(baikal_mbox_driver);

MODULE_DESCRIPTION("Baikal Mailbox SMS driver");
MODULE_LICENSE("GPL v2");
