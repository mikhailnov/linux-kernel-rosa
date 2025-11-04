// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC Process, Voltage, Temperature sensor driver definitions.
 */

#ifndef _BS1000_PVT_H
#define _BS1000_PVT_H

#include <linux/firmware/baikal/baikal-scmi-base-ext.h>

struct pvt_dev;

#define PVT_HWMON_NAME_LEN	14

struct pvt_hwmon {
	char name[PVT_HWMON_NAME_LEN];
	struct device *dev;
	struct mutex mutex;
	struct pvt_dev *pvt;
	struct thermal_zone_device *tzd;
	bool enabled;
};

#define BS1000_PVT_PER_CHIP	28

struct pvt_dev {
	struct device *dev;
	struct mbox_chan *mbox;
	struct pvt_hwmon hwmon[BS1000_PVT_PER_CHIP];
	int processing_mode;
	const struct baikal_scmi_base_ext_proto_ops *ops;
	void *ph;
	int numa_node;
};

int bs1000_pvt_create(struct device *dev,
		      const struct baikal_scmi_base_ext_proto_ops *ops,
		      void *ph);
void bs1000_pvt_destroy(struct device *dev);

#endif /* _BS1000_PVT_H */
