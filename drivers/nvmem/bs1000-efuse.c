// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC eFuse driver.
 */

#include <linux/module.h>

#include "bs1000-efuse.h"

static int bs1000_efuse_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct baikal_scmi_base_ext_proto_ops *ops;
	struct scmi_protocol_handle *ph;

	if (!handle)
		return -ENODEV;

	ops = handle->devm_protocol_get(sdev, BAIKAL_SCMI_PROTOCOL_BASE_EXT, &ph);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	return bs1000_efuse_create(&sdev->dev, ops, ph);
}

static void bs1000_efuse_remove(struct scmi_device *sdev)
{
	bs1000_efuse_destroy(&sdev->dev);
}

static const struct scmi_device_id bs1000_efuse_id_table[] = {
	{ BAIKAL_SCMI_PROTOCOL_BASE_EXT, "efuse" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, bs1000_efuse_id_table);

static struct scmi_driver bs1000_efuse_driver = {
	.name = "bs1000-efuse",
	.probe = bs1000_efuse_probe,
	.remove = bs1000_efuse_remove,
	.id_table = bs1000_efuse_id_table,
};
module_scmi_driver(bs1000_efuse_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 SoC eFuse driver");
MODULE_LICENSE("GPL v2");
