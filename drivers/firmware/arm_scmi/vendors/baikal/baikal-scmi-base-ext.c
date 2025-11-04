// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics SCMI Base Extension protocol driver
 *
 * Copyright (C) 2025 Baikal Electronics, JSC
 */

#include <linux/firmware/baikal/baikal-scmi-base-ext.h>
#include <linux/of.h>

#include "../../common.h"
#include "../../protocols.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x30000

struct baikal_scmi_base_ext_priv {
	unsigned int node_id;
};

static int baikal_scmi_base_ext_get(struct scmi_protocol_handle *ph,
				     enum baikal_scmi_base_ext_protocol_cmd cmd,
				     struct baikal_scmi_base_ext_req *req,
				     struct baikal_scmi_base_ext_resp *resp)
{
	struct scmi_xfer *t;
	int ret;

	if (!req || !resp)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, cmd, sizeof(*req), sizeof(*resp), &t);
	if (ret)
		return ret;

	memcpy(t->tx.buf, req, sizeof(*req));

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		memcpy(resp, t->rx.buf, sizeof(*resp));

	ph->xops->xfer_put(ph, t);
	return ret;
}

static unsigned int baikal_scmi_node_id(void *ph)
{
	struct scmi_protocol_handle *handle = ph;
	struct baikal_scmi_base_ext_priv *priv = handle->get_priv(handle);

	return priv->node_id;
}

static int baikal_scmi_xcp_config_get(void *ph,
				       struct baikal_scmi_base_ext_req *req,
				       struct baikal_scmi_base_ext_resp *resp)
{
	return baikal_scmi_base_ext_get(ph, XCP_CONFIG_GET, req, resp);
}

static int baikal_scmi_soc_status_get(void *ph,
				       struct baikal_scmi_base_ext_req *req,
				       struct baikal_scmi_base_ext_resp *resp)
{
	return baikal_scmi_base_ext_get(ph, SOC_STATUS_GET, req, resp);
}

static struct baikal_scmi_base_ext_proto_ops baikal_base_ext_proto_ops = {
	.node_id = baikal_scmi_node_id,
	.xcp_config_get = baikal_scmi_xcp_config_get,
	.soc_status_get = baikal_scmi_soc_status_get,
};

static int baikal_scmi_base_ext_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	struct baikal_scmi_base_ext_priv *priv;
	struct device_node *np;
	u32 nid;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	priv = devm_kzalloc(ph->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = -ENODATA;
	np = of_node_get(ph->dev->of_node);
	while (np) {
		ret = of_property_read_u32(np, "numa-node-id", &nid);
		if (ret != -EINVAL)
			break;
		np = of_get_next_parent(np);
	}
	of_node_put(np);
	priv->node_id = ret ? 0 : nid;

	dev_info(ph->dev, "Baikal SCMI Base Ext Protocol Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	return ph->set_priv(ph, priv, version);
}

static const struct scmi_protocol baikal_scmi_base_ext = {
	.id = BAIKAL_SCMI_PROTOCOL_BASE_EXT,
	.owner = THIS_MODULE,
	.instance_init = &baikal_scmi_base_ext_protocol_init,
	.ops = &baikal_base_ext_proto_ops,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = "Baikal",
	.sub_vendor_id = "BS1000",
};
module_scmi_protocol(baikal_scmi_base_ext);

MODULE_DESCRIPTION("Baikal SCMI Base Ext Protocol driver");
MODULE_LICENSE("GPL");
