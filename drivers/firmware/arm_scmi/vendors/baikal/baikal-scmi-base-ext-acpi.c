// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics SCMI Base Extension protocol ACPI driver
 *
 * Copyright (C) 2025 Baikal Electronics, JSC
 */

#include <acpi/pcc.h>
#include <linux/acpi.h>
#include <linux/firmware/baikal/baikal-scmi-base-ext.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/rculist.h>

#include "../../common.h"

#define BAIKAL_PCC_CHANNEL_DEFAULT_LATENCY	200000	/* usecs */

struct baikal_scmi_base_ext_dev;

struct baikal_scmi_base_ext_channel {
	u8 id;
	struct mutex lock;
	struct mbox_client client;
	struct pcc_mbox_chan *pcc_chan;
	struct baikal_scmi_base_ext_dev *bdev;
};

struct baikal_scmi_base_ext_dev {
	struct device *dev;
	struct baikal_scmi_base_ext_channel *a2p, *p2a;
	unsigned int seq;
	struct completion done;
	u32 version;
	struct mutex agent_lock;
	struct list_head agent_list;
};

struct baikal_scmi_base_ext_proto_handle {
	struct baikal_scmi_base_ext_dev *bdev;
	struct list_head list;
};

struct baikal_channel_ctx {
	struct baikal_scmi_base_ext_dev *bdev;
	int err;
};

static void baikal_rx_callback(struct mbox_client *cl, void *m)
{
	struct baikal_scmi_base_ext_channel *ch =
		container_of(cl, struct baikal_scmi_base_ext_channel, client);
	struct baikal_scmi_base_ext_dev *bdev = ch->bdev;
	struct pcc_mbox_chan *pcc_chan = ch->pcc_chan;
	void __iomem *shmem = pcc_chan->shmem;
	u32 scmi_msg_hdr = ioread32(shmem + 12);
	u8 msg_type = MSG_XTRACT_TYPE(scmi_msg_hdr);
	unsigned int seq = MSG_XTRACT_TOKEN(scmi_msg_hdr);

	switch (msg_type) {
	case MSG_TYPE_NOTIFICATION:
		dev_err(bdev->dev, "SCMI Protocol notifications not supported yet\n");
		break;
	case MSG_TYPE_COMMAND:
		if (seq == bdev->seq)
			complete(&bdev->done);
		else
			dev_err(bdev->dev, "Response for %u is not expected (%u)\n",
				seq, bdev->seq);
		break;
	case MSG_TYPE_DELAYED_RESP:
		dev_err(bdev->dev, "SCMI Protocol delayed responses not supported yet\n");
		break;
	default:
		WARN_ONCE(1, "Received unknown msg_type: %d\n", msg_type);
		break;
	}
}

static int baikal_pcc_cmd_send(struct baikal_scmi_base_ext_dev *bdev, u8 cmd,
			       struct scmi_msg *req, struct scmi_msg *resp)
{
	u32 scmi_msg_hdr;
	struct acpi_pcct_ext_pcc_shared_memory pcc_shmem_hdr = {
		.signature = PCC_SIGNATURE | bdev->a2p->id,
		.flags = PCC_CMD_COMPLETION_NOTIFY,
	};
	struct baikal_scmi_base_ext_channel *ch = bdev->a2p;
	struct pcc_mbox_chan *pcc_chan = ch->pcc_chan;
	void __iomem *shmem = pcc_chan->shmem;
	u64 max_payload = pcc_chan->shmem_size - sizeof(pcc_shmem_hdr);
	unsigned long timeout = pcc_chan->latency ? pcc_chan->latency :
						    BAIKAL_PCC_CHANNEL_DEFAULT_LATENCY;
	u32 status;
	size_t len = 0;
	int err;

	if (!bdev)
		return -EINVAL;

	if (req) {
		if (req->len > max_payload) {
			dev_err(bdev->dev, "Message too long\n");
			return -ERANGE;
		}
		len = req->len;
		if (!req->buf && len > 0)
			return -EINVAL;
	}

	mutex_lock(&ch->lock);

	bdev->seq++;
	if (bdev->seq == MSG_TOKEN_MAX)
		bdev->seq = 0;

	/* Fill PCC shmem header */
	scmi_msg_hdr = FIELD_PREP(MSG_ID_MASK, cmd) |
		       FIELD_PREP(MSG_TYPE_MASK, MSG_TYPE_COMMAND) |
		       FIELD_PREP(MSG_TOKEN_ID_MASK, bdev->seq) |
		       FIELD_PREP(MSG_PROTOCOL_ID_MASK, BAIKAL_SCMI_PROTOCOL_BASE_EXT);
	pcc_shmem_hdr.command = scmi_msg_hdr;
	pcc_shmem_hdr.length = len + 4;
	memcpy_toio(shmem, (void *)&pcc_shmem_hdr, sizeof(pcc_shmem_hdr));
	if (req && len > 0) {
		/* Fill message payload */
		memcpy_toio(shmem + sizeof(pcc_shmem_hdr), (void *)req->buf, len);
	}

	reinit_completion(&bdev->done);

	/* Ring doorbell */
	err = mbox_send_message(pcc_chan->mchan, NULL);
	if (err < 0) {
		dev_err(bdev->dev, "Ring doorbell failed (%d).\n", err);
		goto out;
	}

	/* Waiting for command complete */
	if (!wait_for_completion_timeout(&bdev->done, usecs_to_jiffies(timeout))) {
		dev_err(bdev->dev, "Command execute timeout!\n");
		err = -ETIMEDOUT;
		goto out;
	}

	/* Copy response data */
	status = ioread32(shmem + sizeof(pcc_shmem_hdr));
	if (status) {
		err = scmi_to_linux_errno(status);
		dev_err(bdev->dev, "Command execute error (%d)!\n", err);
		goto out;
	}
	if (resp && resp->buf) {
		/* Skip the length of header and status in shmem area i.e 8 bytes */
		memcpy_fromio((void *)&pcc_shmem_hdr, shmem, sizeof(pcc_shmem_hdr));
		len = pcc_shmem_hdr.length;
		resp->len = min_t(size_t, resp->len, len > 8 ? len - 8 : 0);
		/* Take a copy to the rx buffer.. */
		memcpy_fromio(resp->buf, shmem + sizeof(pcc_shmem_hdr) + 4, resp->len);
	}
	err = 0;

out:
	mutex_unlock(&ch->lock);
	mbox_chan_txdone(pcc_chan->mchan, err);
	return err;
}

static int baikal_scmi_cmd_send(struct baikal_scmi_base_ext_proto_handle *ph,
				enum baikal_scmi_base_ext_protocol_cmd cmd,
				struct baikal_scmi_base_ext_req *req,
				struct baikal_scmi_base_ext_resp *resp)
{
	struct scmi_msg tx = {
		.buf = req,
		.len = sizeof(*req),
	};
	struct scmi_msg rx = {
		.buf = resp,
		.len = sizeof(*resp),
	};

	return baikal_pcc_cmd_send(ph->bdev, cmd, &tx, &rx);
}

static unsigned int baikal_scmi_node_id(void *ph)
{
	struct baikal_scmi_base_ext_proto_handle *handle = ph;
	unsigned int node_id;

	node_id = dev_to_node(handle->bdev->dev);

	return node_id == NUMA_NO_NODE ? 0 : node_id;
}

static int baikal_scmi_xcp_config_get(void *ph,
				      struct baikal_scmi_base_ext_req *req,
				      struct baikal_scmi_base_ext_resp *resp)
{
	return baikal_scmi_cmd_send(ph, XCP_CONFIG_GET, req, resp);
}

static int baikal_scmi_soc_status_get(void *ph,
				      struct baikal_scmi_base_ext_req *req,
				      struct baikal_scmi_base_ext_resp *resp)
{
	return baikal_scmi_cmd_send(ph, SOC_STATUS_GET, req, resp);
}

static struct baikal_scmi_base_ext_proto_ops baikal_base_ext_proto_ops = {
	.node_id = baikal_scmi_node_id,
	.xcp_config_get = baikal_scmi_xcp_config_get,
	.soc_status_get = baikal_scmi_soc_status_get,
};

const struct baikal_scmi_base_ext_proto_ops *baikal_scmi_base_ext_proto_get(
		struct device *dev,
		struct baikal_scmi_base_ext_proto_handle **ph)
{
	struct device *scmi_dev;
	struct acpi_device *acpi_dev = ACPI_COMPANION(dev), *acpi_scmi;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *package;
	union acpi_object *element;
	struct baikal_scmi_base_ext_dev *bdev;
	struct baikal_scmi_base_ext_proto_handle *handle;
	acpi_status status;
	int err = 0;

	if (!acpi_dev)
		return ERR_PTR(-ENODEV);

	status = acpi_evaluate_object_typed(acpi_dev->handle, "SCMI", NULL,
					    &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "Failed to get SCMI property\n");
		return ERR_PTR(-ENODEV);
	}

	package = buffer.pointer;
	if (package->package.count != 1) {
		dev_err(dev, "Invalid SCMI property\n");
		err = -EINVAL;
		goto out;
	}

	element = &(package->package.elements[0]);
	if (element->type != ACPI_TYPE_LOCAL_REFERENCE ||
	    !element->reference.handle) {
		dev_err(dev, "Invalid SCMI device reference\n");
		err = -EINVAL;
		goto out;
	}

	acpi_scmi = acpi_fetch_acpi_dev(element->reference.handle);
	if (!acpi_scmi) {
		dev_err(dev, "Failed to fetch SCMI device\n");
		err = -ENODEV;
		goto out;
	}

	scmi_dev = bus_find_device_by_fwnode(&platform_bus_type,
					     acpi_fwnode_handle(acpi_scmi));
	if (!scmi_dev) {
		dev_err(dev, "Failed to get SCMI device\n");
		err = -ENODEV;
		goto out;
	}

	bdev = dev_get_drvdata(scmi_dev);
	if (!bdev) {
		err = -EPROBE_DEFER;
		goto out;
	}

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		dev_err(dev, "Not enough memory\n");
		err = -ENOMEM;
		goto out;
	}
	handle->bdev = bdev;
	mutex_lock(&bdev->agent_lock);
	list_add_tail_rcu(&handle->list, &bdev->agent_list);
	mutex_unlock(&bdev->agent_lock);
	*ph = handle;

	try_module_get(THIS_MODULE);

out:
	acpi_os_free(buffer.pointer);
	if (err)
		return ERR_PTR(err);
	else
		return &baikal_base_ext_proto_ops;
}
EXPORT_SYMBOL(baikal_scmi_base_ext_proto_get);

void baikal_scmi_base_ext_proto_put(struct baikal_scmi_base_ext_proto_handle *ph)
{
	if (!ph)
		return;

	mutex_lock(&ph->bdev->agent_lock);
	list_del_rcu(&ph->list);
	mutex_unlock(&ph->bdev->agent_lock);
	synchronize_rcu();
	kfree(ph);

	module_put(THIS_MODULE);
}
EXPORT_SYMBOL(baikal_scmi_base_ext_proto_put);

static acpi_status baikal_get_channel_cb(struct acpi_resource *ares,
					 void *context)
{
	struct acpi_resource_generic_register *reg;
	struct baikal_channel_ctx *ctx = context;
	struct baikal_scmi_base_ext_dev *bdev = ctx->bdev;
	struct baikal_scmi_base_ext_channel *ch;

	if (ares->type != ACPI_RESOURCE_TYPE_GENERIC_REGISTER)
		return AE_OK;

	reg = &ares->data.generic_reg;
	if (reg->space_id != ACPI_ADR_SPACE_PLATFORM_COMM) {
		dev_err(bdev->dev, "Bad channel resource.\n");
		ctx->err = -EINVAL;
		return AE_ERROR;
	}

	ch = devm_kzalloc(bdev->dev, sizeof(*ch), GFP_KERNEL);
	if (!ch) {
		dev_err(bdev->dev, "Not enough memory.\n");
		ctx->err = -ENOMEM;
		return AE_ERROR;
	}
	ch->id = reg->access_size;

	if (!bdev->a2p)
		bdev->a2p = ch;
	else if (!bdev->p2a)
		bdev->p2a = ch;
	else {
		devm_kfree(bdev->dev, ch);
		dev_err(bdev->dev, "Unknown channel resource.\n");
		ctx->err = -EINVAL;
		return AE_ERROR;
	}

	return AE_OK;
}

static int baikal_get_pcc_channels(struct baikal_scmi_base_ext_dev *bdev)
{
	acpi_handle handle = ACPI_HANDLE(bdev->dev);
	struct baikal_channel_ctx ctx = {0};
	acpi_status status;

	if (!acpi_has_method(handle, METHOD_NAME__CRS)) {
		dev_err(bdev->dev, "No _CRS method.\n");
		return -ENODEV;
	}

	ctx.bdev = bdev;
	status = acpi_walk_resources(handle, METHOD_NAME__CRS,
				     baikal_get_channel_cb, &ctx);
	if (ACPI_FAILURE(status))
		return ctx.err;

	return 0;
}

static int baikal_register_pcc_channel(struct baikal_scmi_base_ext_dev *bdev,
				       struct baikal_scmi_base_ext_channel *ch,
				       bool tx)
{
	struct device *dev = bdev->dev;
	struct mbox_client *cl = &ch->client;
	struct pcc_mbox_chan *pcc_chan;
	int err;

	cl->dev = dev;
	cl->rx_callback = baikal_rx_callback;
	cl->tx_block = false;
	cl->knows_txdone = tx;

	ch->bdev = bdev;
	mutex_init(&ch->lock);

	pcc_chan = pcc_mbox_request_channel(cl, ch->id);
	if (IS_ERR(pcc_chan)) {
		dev_err(dev, "PPC channel request failed.\n");
		return -ENODEV;
	}
	ch->pcc_chan = pcc_chan;

	if (!pcc_chan->mchan->mbox->txdone_irq) {
		dev_err(dev, "PCC IRQ not supported.\n");
		err = -EINVAL;
		goto out_channel_free;
	}

	if (!pcc_chan->shmem_base_addr) {
		dev_err(dev, "PCC shared memory region not defined.\n");
		err = -EINVAL;
		goto out_channel_free;
	}

	err = pcc_mbox_ioremap(pcc_chan->mchan);
	if (err || !pcc_chan->shmem) {
		dev_err(dev, "Channel %hhu shared memory region mapping error.\n",
			ch->id);
		err = -ENOMEM;
		goto out_channel_free;
	}

	dev_dbg(dev, "%s PCC channel: %hhu\n", tx ? "Tx" : "Rx", ch->id);

	return 0;

out_channel_free:
	pcc_mbox_free_channel(pcc_chan);
	return err;
}

static void baikal_unregister_pcc_channel(struct baikal_scmi_base_ext_channel *ch)
{
	if (ch)
		pcc_mbox_free_channel(ch->pcc_chan);
}

static void baikal_scmi_base_ext_acpi_remove(struct platform_device *pdev)
{
	struct baikal_scmi_base_ext_dev *bdev = platform_get_drvdata(pdev);

	baikal_unregister_pcc_channel(bdev->a2p);
	baikal_unregister_pcc_channel(bdev->p2a);
}

static int baikal_scmi_base_ext_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_scmi_base_ext_dev *bdev;
	u32 version;
	struct scmi_msg resp = {
		.buf = &version,
		.len = sizeof(version),
	};
	int err;

	if (acpi_disabled) {
		dev_err(dev, "ACPI is disabled.\n");
		return -ENODEV;
	}

	if (!ACPI_COMPANION(dev))
		return -ENODEV;

	bdev = devm_kzalloc(dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;
	bdev->dev = dev;
	init_completion(&bdev->done);
	mutex_init(&bdev->agent_lock);
	INIT_LIST_HEAD(&bdev->agent_list);
	platform_set_drvdata(pdev, bdev);

	err = baikal_get_pcc_channels(bdev);
	if (err)
		return err;

	if (!bdev->a2p) {
		dev_err(dev, "No PCC channels.\n");
		return -ENODEV;
	}

	err = baikal_register_pcc_channel(bdev, bdev->a2p, true);
	if (err)
		return err;

	if (bdev->p2a) {
		err = baikal_register_pcc_channel(bdev, bdev->p2a, false);
		if (err) {
			baikal_unregister_pcc_channel(bdev->a2p);
			return err;
		}
	}

	/* Get Protocol Version */
	err = baikal_pcc_cmd_send(bdev, PROTOCOL_VERSION, NULL, &resp);
	if (err) {
		dev_err(dev, "Can't get protocol version (%d)\n", err);
		baikal_scmi_base_ext_acpi_remove(pdev);
		return err;
	}

	dev_info(dev, "Baikal SCMI Base Ext Protocol Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	return 0;
}

static const struct acpi_device_id baikal_scmi_base_ext_acpi_match[] = {
	{ "BKLE2001" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, baikal_scmi_base_ext_acpi_match);

static struct platform_driver baikal_scmi_base_ext_acpi = {
	.probe = baikal_scmi_base_ext_acpi_probe,
	.remove_new = baikal_scmi_base_ext_acpi_remove,
	.driver = {
		.name = "baikal_scmi_base_ext_acpi",
		.acpi_match_table = baikal_scmi_base_ext_acpi_match,
	},
};

module_platform_driver(baikal_scmi_base_ext_acpi);

MODULE_DESCRIPTION("Baikal SCMI Base Ext Protocol ACPI driver");
MODULE_LICENSE("GPL");
