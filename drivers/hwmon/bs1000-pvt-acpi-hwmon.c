// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC Process, Voltage, Temperature sensor ACPI driver.
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>

#include "bs1000-pvt.h"

static int bs1000_pvt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct baikal_scmi_base_ext_proto_ops *ops;
	struct baikal_scmi_base_ext_proto_handle *ph;

	if (acpi_disabled) {
		dev_err(dev, "ACPI is disabled.\n");
		return -ENODEV;
	}

	ops = baikal_scmi_base_ext_proto_get(dev, &ph);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	return bs1000_pvt_create(dev, ops, ph);
}

static void bs1000_pvt_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pvt_dev *pvt = dev_get_drvdata(dev);

	baikal_scmi_base_ext_proto_put(pvt->ph);
	bs1000_pvt_destroy(dev);
}

static const struct acpi_device_id bs1000_pvt_acpi_match[] = {
	{ "BKLE2002" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, bs1000_pvt_acpi_match);

static struct platform_driver bs1000_pvt_driver = {
	.probe = bs1000_pvt_probe,
	.remove_new = bs1000_pvt_remove,
	.driver = {
		.name = "bs1000-pvt-acpi",
		.acpi_match_table = bs1000_pvt_acpi_match,
	},
};

module_platform_driver(bs1000_pvt_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 SoC PVT ACPI driver");
MODULE_LICENSE("GPL v2");
