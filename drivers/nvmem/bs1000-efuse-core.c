// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC eFuse driver common code.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>

#include "bs1000-efuse.h"

#define BS1000_EFUSE_N_CELLS	7

static const struct nvmem_cell_info bs1000_efuse_cell_info[BS1000_EFUSE_N_CELLS] = {
	{
		.name = "Process",
		.offset = 0,
		.bytes = 5,
	},
	{
		.name = "ProjectName",
		.offset = 5,
		.bytes = 2,
	},
	{
		.name = "ProjectRevision",
		.offset = 7,
		.bytes = 1,
	},
	{
		.name = "LotID",
		.offset = 8,
		.bytes = 6,
	},
	{
		.name = "SerialNumber",
		.offset = 14,
		.bytes = 3,
	},
	{
		.name = "MacOffset",
		.offset = 17,
		.bytes = 3,
	},
	{
		.name = "BinMask",
		.offset = 20,
		.bytes = 4,
	},
};

static const enum baikal_scmi_status_id_efuse bs1000_efuse_cell_id[BS1000_EFUSE_N_CELLS] = {
	EFUSE_Process, EFUSE_ProjectName, EFUSE_ProjectRevision, EFUSE_LotID,
	EFUSE_SerialNumber, EFUSE_MacOffset, EFUSE_BinMask
};

static int bs1000_efuse_reg_read(void *context, unsigned int offset,
				 void *val, size_t bytes)
{
	struct bs1000_efuse_dev *edev = (struct bs1000_efuse_dev *)context;

	if (offset + bytes > BS1000_EFUSE_DATA_SIZE)
		return -ERANGE;

	memcpy(val, edev->data + offset, bytes);

	return 0;
}

int bs1000_efuse_create(struct device *dev,
			const struct baikal_scmi_base_ext_proto_ops *ops,
			void *ph)
{
	struct bs1000_efuse_dev *edev;
	struct baikal_scmi_base_ext_req req;
	struct baikal_scmi_base_ext_resp resp;
	struct nvmem_config econfig = {};
	struct nvmem_device *nvmem;
	int i, ret;

	edev = devm_kzalloc(dev, sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;
	edev->dev = dev;
	edev->ops = ops;
	edev->ph = ph;
	dev_set_drvdata(dev, edev);

	econfig.dev = dev;
	econfig.name = "efuse";
	econfig.id = edev->ops->node_id(edev->ph);
	econfig.cells = bs1000_efuse_cell_info;
	econfig.ncells = BS1000_EFUSE_N_CELLS;
	econfig.read_only = true;
	econfig.reg_read = bs1000_efuse_reg_read;
	econfig.size = BS1000_EFUSE_DATA_SIZE;
	econfig.word_size = 1;
	econfig.stride = 1;
	econfig.priv = edev;

	nvmem = devm_nvmem_register(dev, &econfig);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	req.service_id = SID_EFUSE;
	for (i = 0; i < BS1000_EFUSE_N_CELLS; i++) {
		req.value_id = bs1000_efuse_cell_id[i];
		ret = edev->ops->soc_status_get(edev->ph, &req, &resp);
		if (ret || resp.count != bs1000_efuse_cell_info[i].bytes) {
			dev_err(dev, "Couldn't get eFuse %s data (%d)\n",
				bs1000_efuse_cell_info[i].name,
				ret ? ret : resp.count);
			return ret;
		}
		memcpy(edev->data + bs1000_efuse_cell_info[i].offset,
		      (u8 *)resp.value, bs1000_efuse_cell_info[i].bytes);
	}

	return 0;
}
EXPORT_SYMBOL(bs1000_efuse_create);

void bs1000_efuse_destroy(struct device *dev)
{
	dev_set_drvdata(dev, NULL);
}
EXPORT_SYMBOL(bs1000_efuse_destroy);

MODULE_DESCRIPTION("Baikal BE-S1000 SoC eFuse core driver");
MODULE_LICENSE("GPL v2");
