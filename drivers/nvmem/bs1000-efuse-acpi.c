// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC eFuse ACPI driver.
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>

#include "bs1000-efuse.h"

static int bs1000_efuse_probe(struct platform_device *pdev)
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

	return bs1000_efuse_create(dev, ops, ph);
}

static void bs1000_efuse_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bs1000_efuse_dev *edev = dev_get_drvdata(dev);

	baikal_scmi_base_ext_proto_put(edev->ph);
	bs1000_efuse_destroy(dev);
}

static const struct acpi_device_id bs1000_efuse_acpi_match[] = {
	{ "BKLE2003" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, bs1000_efuse_acpi_match);

static struct platform_driver bs1000_efuse_driver = {
	.probe = bs1000_efuse_probe,
	.remove_new = bs1000_efuse_remove,
	.driver = {
		.name = "bs1000-efuse-acpi",
		.acpi_match_table = bs1000_efuse_acpi_match,
	},
};

module_platform_driver(bs1000_efuse_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 SoC eFuse ACPI driver");
MODULE_LICENSE("GPL v2");
