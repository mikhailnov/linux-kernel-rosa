// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 * Author: Baikal Electronics <support@baikalelectronics.ru>
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/initval.h>

#include "local.h"

static struct snd_soc_dai_link snd_baikal_dai = {
	.dai_fmt = SND_SOC_DAIFMT_I2S	|
		   SND_SOC_DAIFMT_NB_NF	|
		   SND_SOC_DAIFMT_CBM_CFM,
};

static struct snd_soc_card snd_baikal_card = {
	.name		= "Baikal-Snd-Card",
	.owner		= THIS_MODULE,
	.dai_link	= &snd_baikal_dai,
	.num_links	= 1,
};

static int snd_soc_baikal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *codec_node, *baikal_node;
	struct snd_soc_dai_link_component *comp;
	int err;

	comp = devm_kzalloc(dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return -ENOMEM;

	baikal_node = fwnode_find_reference(dev->fwnode, "baikal,cpu-dai", 0);
	if (IS_ERR(baikal_node)) {
		dev_err(dev,
			"'baikal,cpu-dai': missing or invalid!\n");
		return -EINVAL;
	}

	codec_node = fwnode_find_reference(dev->fwnode, "baikal,audio-codec", 0);
	if (IS_ERR(codec_node)) {
		dev_err(dev,
			"'baikal,audio-codec': missing or invalid!\n");
		return -EINVAL;
	}

	snd_baikal_card.dev = dev;
	snd_baikal_dai.cpus = &comp[0];
	snd_baikal_dai.codecs = &comp[1];
	snd_baikal_dai.platforms = &comp[2];

	err = device_property_read_string(dev, "baikal,dai-name",
					  &snd_baikal_dai.name);
	if (err) {
		dev_err(dev,
			"'baikal,dai-name': missing or invalid!\n");
		return -EINVAL;
	}

	err = device_property_read_string(dev, "baikal,stream-name",
					  &snd_baikal_dai.stream_name);
	if (err) {
		dev_err(dev,
			"'baikal,stream-name': missing or invalid!\n");
		return -EINVAL;
	}

	err = device_property_read_string(dev, "baikal,codec-name",
					  &snd_baikal_dai.codecs->dai_name);
	if (err) {
		dev_err(dev,
			"'baikal,codec-name': missing or invalid!\n");
		return -EINVAL;
	}

	snd_baikal_dai.num_cpus = 1;
	snd_baikal_dai.num_codecs = 1;
	snd_baikal_dai.num_platforms = 1;

	if (is_of_node(baikal_node) && is_of_node(codec_node)) {
		snd_baikal_dai.cpus->of_node = to_of_node(baikal_node);
		snd_baikal_dai.codecs->of_node = to_of_node(codec_node);
		snd_baikal_dai.platforms->of_node = to_of_node(baikal_node);
		snd_soc_of_parse_card_name(&snd_baikal_card, "baikal,card-name");
	} else {
		struct device *tmp;

		tmp = bus_find_device_by_fwnode(&platform_bus_type, baikal_node);
		if (IS_ERR_OR_NULL(tmp))
			return -ENODEV;
		snd_baikal_dai.cpus->name = dev_name(tmp);
		snd_baikal_dai.platforms->name = dev_name(tmp);

		tmp = bus_find_device_by_fwnode(&i2c_bus_type, codec_node);
		if (IS_ERR_OR_NULL(tmp))
			return -ENODEV;
		snd_baikal_dai.codecs->name = dev_name(tmp);
	}

	platform_set_drvdata(pdev, &snd_baikal_card);
	snd_soc_card_set_drvdata(&snd_baikal_card, NULL);

	err = snd_soc_register_card(&snd_baikal_card);
	if (err)
		dev_err(dev,
			"snd_soc_register_card failed (%d)!\n", err);

	return err;
}

static void snd_soc_baikal_remove(struct platform_device *pdev)
{
	struct snd_soc_card *snd_baikal_card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(snd_baikal_card);
}

#ifdef CONFIG_OF
static const struct of_device_id snd_soc_baikal_of_match[] = {
	{ .compatible = "baikal,snd-soc-baikal", },
	{},
};

MODULE_DEVICE_TABLE(of, snd_soc_baikal_of_match);
#endif


static struct platform_driver snd_soc_baikal_driver = {
	.probe		= snd_soc_baikal_probe,
	.remove		= snd_soc_baikal_remove,
	.driver		= {
		.name	= "snd-soc-baikal",
		.of_match_table = of_match_ptr(snd_soc_baikal_of_match),
	},
};

module_platform_driver(snd_soc_baikal_driver);

MODULE_AUTHOR("Baikal Electronics <support@baikalelectronics.ru>");
MODULE_DESCRIPTION("BM1000 SoC Sound Card Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bm1000_sound");
