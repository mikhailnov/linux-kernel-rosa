// SPDX-License-Identifier: GPL-2.0-only
/*
 * Baikal flash partition driver for ACPI
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/device/bus.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>

#include <acpi/actypes.h>

static int baikal_add_mtd_partitions(struct device *dev, void *unused)
{
	struct acpi_device *adev;
	const union acpi_object *obj;
	struct flash_platform_data *data;
	struct mtd_partition *parts;
	int nr_parts, i, ret;

	if (dev) {
		adev = ACPI_COMPANION(dev);
		if (!adev)
			return 0;

		if (!acpi_dev_get_property(adev, "baikal,partitions",
					   ACPI_TYPE_PACKAGE, &obj)) {
			if (!obj->package.count || obj->package.count % 3)
				return 0;
			nr_parts = obj->package.count / 3;

			data = kzalloc(sizeof(*data) + nr_parts * sizeof(*parts),
				       GFP_KERNEL);
			if (!data)
				return -ENOMEM;
			parts = (void *)data + sizeof(*data);

			for (i = 0; i < nr_parts; ++i) {
				if (obj->package.elements[3 * i].type != ACPI_TYPE_STRING ||
				    obj->package.elements[3 * i + 1].type != ACPI_TYPE_INTEGER ||
				    obj->package.elements[3 * i + 2].type != ACPI_TYPE_INTEGER) {
					kfree(data);
					return 0;
				}

				parts[i].name = kmemdup_nul(obj->package.elements[3 * i].string.pointer,
							    obj->package.elements[3 * i].string.length,
							    GFP_KERNEL);
				if (!parts[i].name) {
					kfree(data);
					return -ENOMEM;
				}

				parts[i].offset =
					obj->package.elements[3 * i + 1].integer.value;
				parts[i].size =
					obj->package.elements[3 * i + 2].integer.value;
			}

			data->parts = parts;
			data->nr_parts = nr_parts;
			dev->platform_data = data;

			ret = driver_set_override(dev, &to_spi_device(dev)->driver_override,
						  "spi-nor", strlen("spi-nor"));
			if (ret) {
				dev->platform_data = NULL;
				kfree(data);
				return ret;
			}

			ret = device_reprobe(dev);
			if (ret) {
				dev->platform_data = NULL;
				kfree(data);
				return ret;
			}
		}
	}

	return 0;
}

static int __init baikal_spi_partitions_init(void)
{
	return bus_for_each_dev(&spi_bus_type, NULL, NULL,
				baikal_add_mtd_partitions);
}

late_initcall(baikal_spi_partitions_init);
