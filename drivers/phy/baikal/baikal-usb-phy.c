// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal USB PHY driver
 *
 * Copyright (C) 2022-2023 Baikal Electronics, JSC
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <dt-bindings/phy/phy.h>

struct phy_baikal_desc {
	struct phy		*phy;
	int			index;
	bool			enable;
};

struct phy_baikal_priv {
	struct phy_baikal_desc	**phys;
	int			nphys;
	struct clk_bulk_data	*clocks;
	unsigned int		nclocks;
};

#ifdef CONFIG_ACPI
static int phy_baikal_acpi_get_info(struct fwnode_handle *fwnode,
				    bool *is_usb3, const char **ref_name)
{
	struct acpi_device *adev = to_acpi_device_node(fwnode);
	struct acpi_device *ref_adev;
	struct device *ref_dev;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;
	acpi_status status;
	int ret = 0;

	*is_usb3 = fwnode_property_read_bool(fwnode, "usb3");
	status = acpi_evaluate_object_typed(adev->handle, "CTRL", NULL,
					    &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(&adev->dev, "failed to get CTRL data\n");
		return -ENODEV;
	}

	obj = buffer.pointer;
	if (obj->package.count != 1) {
		dev_err(&adev->dev, "invalid CTRL data\n");
		ret = -EINVAL;
		goto err;
	}

	obj = &obj->package.elements[0];
	if (obj->type != ACPI_TYPE_LOCAL_REFERENCE || !obj->reference.handle) {
		dev_err(&adev->dev, "invalid CTRL reference\n");
		ret = -EINVAL;
		goto err;
	}

	ref_adev = acpi_fetch_acpi_dev(obj->reference.handle);
	if (!ref_adev) {
		dev_err(&adev->dev, "failed to process CTRL reference\n");
		ret = -ENODEV;
		goto err;
	}

	ref_dev = bus_find_device_by_fwnode(&platform_bus_type,
					    acpi_fwnode_handle(ref_adev));
	if (!ref_dev) {
		dev_err(&adev->dev, "failed to get referenced device\n");
		ret = -ENODEV;
		goto err;
	}
	*ref_name = dev_name(ref_dev);

err:
	acpi_os_free(buffer.pointer);
	return ret;
}

static int phy_baikal_acpi_add(struct device *dev)
{
	struct phy_baikal_priv *priv = dev_get_drvdata(dev);
	struct fwnode_handle *child;
	const char **ref_name;
	bool *is_usb3;
	int count = 0, i, ret;

	ref_name = kcalloc(priv->nphys, sizeof(*ref_name), GFP_KERNEL);
	if (!ref_name)
		return -ENOMEM;

	is_usb3 = kcalloc(priv->nphys, sizeof(*is_usb3), GFP_KERNEL);
	if (!is_usb3) {
		kfree(ref_name);
		return -ENOMEM;
	}

	device_for_each_child_node(dev, child) {
		ret = phy_baikal_acpi_get_info(child, &is_usb3[count],
					       &ref_name[count]);
		if (ret)
			goto err;

		ret = phy_create_lookup(priv->phys[count]->phy,
					is_usb3[count] ? "usb3-phy" : "usb2-phy",
					ref_name[count]);
		if (ret)
			goto err;

		++count;
	}

err:
	if (ret) {
		for (i = 0; i < count; ++i)
			phy_remove_lookup(priv->phys[i]->phy,
					  is_usb3[i] ? "usb3-phy" : "usb2-phy",
					  ref_name[i]);
	}

	kfree(ref_name);
	kfree(is_usb3);
	return ret;
}

static void phy_baikal_acpi_remove(struct device *dev)
{
	struct phy_baikal_priv *priv = dev_get_drvdata(dev);
	struct fwnode_handle *child;
	const char *ref_name;
	bool is_usb3;
	int i = 0, ret;

	device_for_each_child_node(dev, child) {
		ret = phy_baikal_acpi_get_info(child, &is_usb3, &ref_name);
		if (ret) {
			++i;
			continue;
		}

		phy_remove_lookup(priv->phys[i++]->phy,
				  is_usb3 ? "usb3-phy" : "usb2-phy",
				  ref_name);
	}
}

static void phy_baikal_acpi_init_clk(struct phy_baikal_desc *desc)
{
	struct device *dev = desc->phy->dev.parent;
	struct phy_baikal_priv *priv = dev_get_drvdata(dev);
	struct fwnode_handle *child;
	int i = 0;

	device_for_each_child_node(dev, child) {
		if (i++ == desc->index) {
			priv->clocks[2 * desc->index].clk =
				devm_clk_get_optional(&to_acpi_device_node(child)->dev, "phy0_clk");
			priv->clocks[2 * desc->index + 1].clk =
				devm_clk_get_optional(&to_acpi_device_node(child)->dev, "phy1_clk");
			break;
		}
	}
}
#else /* CONFIG_ACPI */
static inline int phy_baikal_acpi_add(struct device *dev)
{
	return 0;
}

static inline void phy_baikal_acpi_remove(struct device *dev)
{
}

static inline void phy_baikal_acpi_init_clk(struct phy_baikal_desc *desc)
{
}
#endif

static int phy_baikal_init(struct phy *phy)
{
	struct phy_baikal_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct phy_baikal_desc *desc = phy_get_drvdata(phy);
	int n = 2 * desc->index;

	if (!acpi_disabled)
		phy_baikal_acpi_init_clk(desc);

	if (desc->enable) {
		clk_prepare_enable(priv->clocks[n + 0].clk);
		clk_prepare_enable(priv->clocks[n + 1].clk);
		return 0;
	} else {
		clk_disable_unprepare(priv->clocks[n + 0].clk);
		clk_disable_unprepare(priv->clocks[n + 1].clk);
		return -1;
	}
}

static const struct phy_ops phy_baikal_ops = {
	.init	= phy_baikal_init,
	.owner	= THIS_MODULE,
};

static struct phy *phy_baikal_xlate(struct device *dev, const struct of_phandle_args *args)
{
	int i;
	struct phy_baikal_priv *priv = dev_get_drvdata(dev);

	for (i = 0; i < priv->nphys; i++) {
		if (priv->phys[i]->index == args->args[0])
			break;
	}

	return priv->phys[i]->phy;
}

static int phy_baikal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct phy *phy;
	struct phy_baikal_priv *priv;
	struct phy_baikal_desc *phy_desc;
	int index;
	int i = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->nphys = device_get_child_node_count(dev);
	priv->phys = devm_kcalloc(dev, priv->nphys, sizeof(*priv->phys), GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	if (acpi_disabled) {
		priv->nclocks = devm_clk_bulk_get_all(dev, &priv->clocks);
	} else {
		priv->nclocks = 2 * priv->nphys;
		priv->clocks = devm_kcalloc(dev, priv->nclocks,
					    sizeof(*priv->clocks), GFP_KERNEL);
		if (!priv->clocks)
			return -ENOMEM;
	}

	dev_set_drvdata(dev, priv);
	device_for_each_child_node(dev, child) {
		fwnode_property_read_u32(child, "reg", &index);
		phy = devm_phy_create(dev, NULL, &phy_baikal_ops);
		if (!phy)
			return -ENOMEM;

		phy_desc = devm_kzalloc(dev, sizeof(*phy_desc), GFP_KERNEL);
		if (!phy_desc)
			return -ENOMEM;

		phy_desc->phy = phy;
		phy_desc->index = index;
		phy_desc->enable = fwnode_property_read_bool(child, "enable");
		priv->phys[i++] = phy_desc;
		phy_set_drvdata(phy, phy_desc);
	}

	if (acpi_disabled)
		return PTR_ERR_OR_ZERO(devm_of_phy_provider_register(dev, phy_baikal_xlate));
	else
		return phy_baikal_acpi_add(dev);
}

static void phy_baikal_remove(struct platform_device *pdev)
{
	if (!acpi_disabled)
		phy_baikal_acpi_remove(&pdev->dev);
}

static const struct of_device_id phy_baikal_table[] = {
	{ .compatible = "baikal,bm1000-usb-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_baikal_table);

static struct platform_driver phy_baikal_driver = {
	.probe = phy_baikal_probe,
	.remove = phy_baikal_remove,
	.driver = {
		.name = "baikal,bm1000-usb-phy",
		.of_match_table = phy_baikal_table,
	},
};
module_platform_driver(phy_baikal_driver);

MODULE_DESCRIPTION("Baikal USB PHY driver");
MODULE_LICENSE("GPL v2");
