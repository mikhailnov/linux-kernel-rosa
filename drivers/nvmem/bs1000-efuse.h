// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC eFuse driver definitions.
 */

#ifndef _BS1000_EFUSE_H
#define _BS1000_EFUSE_H

#include <linux/firmware/baikal/baikal-scmi-base-ext.h>
#include <linux/nvmem-consumer.h>

#define BS1000_EFUSE_DATA_SIZE	24

struct bs1000_efuse_dev {
	struct device *dev;
	const struct baikal_scmi_base_ext_proto_ops *ops;
	void *ph;
	u8 data[BS1000_EFUSE_DATA_SIZE];
};

int bs1000_efuse_create(struct device *dev,
			const struct baikal_scmi_base_ext_proto_ops *ops,
			void *ph);
void bs1000_efuse_destroy(struct device *dev);

#endif /* _BS1000_EFUSE_H */
