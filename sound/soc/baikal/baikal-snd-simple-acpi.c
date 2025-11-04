// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal ASoC simple sound card support for ACPI
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/simple_card_utils.h>
#include <sound/soc.h>

static struct snd_soc_dapm_widget baikal_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("AUX Out", NULL)
};

static struct snd_soc_dapm_route baikal_routes[] = {
	{ .sink = "Headphones",	.source = "RHP" },
	{ .sink = "Headphones",	.source = "LHP" },
	{ .sink = "AUX Out",	.source = "AUXOUT1" },
	{ .sink = "AUX Out",	.source = "AUXOUT2" },
	{ .sink = "L2",		.source = "Mic Jack" },
	{ .sink = "R2",		.source = "Mic Jack" },
	{ .sink = "LAUX",	.source = "Line In" },
	{ .sink = "RAUX",	.source = "Line In" }
};

static int baikal_snd_soc_card_probe(struct snd_soc_card *card)
{
	struct simple_util_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = simple_util_init_hp(card, &priv->hp_jack, NULL);
	if (ret)
		return ret;

	return simple_util_init_mic(card, &priv->mic_jack, NULL);
}

static const struct snd_soc_ops baikal_snd_soc_ops = {
	.startup	= simple_util_startup,
	.shutdown	= simple_util_shutdown,
	.hw_params	= simple_util_hw_params
};

static int baikal_simple_card_parse(struct simple_util_priv *priv)
{
	struct device *dev = priv->snd_card.dev, *tmp;
	struct snd_soc_dai_link *dai_link = priv->snd_card.dai_link;
	struct fwnode_reference_args args;
	int ret;

	ret = fwnode_property_get_reference_args(dev->fwnode, "baikal,cpu-dai",
						 NULL, 0, 0, &args);
	if (ret) {
		dev_err(dev, "'baikal,cpu': missing or invalid!\n");
		return -EINVAL;
	}

	tmp = bus_find_device_by_fwnode(&platform_bus_type, args.fwnode);
	if (IS_ERR_OR_NULL(tmp))
		return -ENODEV;
	dai_link->cpus->name = dev_name(tmp);
	dai_link->platforms->name = dev_name(tmp);
	priv->dai_props->cpu_dai->sysclk = 100000000;

	ret = fwnode_property_get_reference_args(dev->fwnode, "baikal,audio-codec",
						 NULL, 0, 0, &args);
	if (ret) {
		dev_err(dev, "'baikal,codec': missing or invalid!\n");
		return -EINVAL;
	}

	tmp = bus_find_device_by_fwnode(&i2c_bus_type, args.fwnode);
	if (IS_ERR_OR_NULL(tmp))
		return -ENODEV;
	dai_link->codecs->name = dev_name(tmp);

	ret = device_property_read_string(dev, "baikal,codec-name",
					  &dai_link->codecs->dai_name);
	if (ret) {
		dev_err(dev, "'baikal,codec-name': missing or invalid!\n");
		return -EINVAL;
	}

	dai_link->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM;
	dai_link->init = simple_util_dai_init;
	dai_link->ops = &baikal_snd_soc_ops;

	return simple_util_set_dailink_name(dev, dai_link, "%s-%s",
					    dai_link->cpus->name,
					    dai_link->codecs->dai_name);
}

static int baikal_simple_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simple_util_priv *priv;
	struct snd_soc_card *card;
	struct link_info *li;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card = &priv->snd_card;
	card->owner = THIS_MODULE;
	card->dev = dev;
	card->driver_name = "baikal-simple-card";
	card->name = "MITX-Sound-Card";
	card->dapm_widgets = baikal_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(baikal_widgets);
	card->dapm_routes = baikal_routes;
	card->num_dapm_routes = ARRAY_SIZE(baikal_routes);
	card->probe = baikal_snd_soc_card_probe;

	li = kzalloc(sizeof(*li), GFP_KERNEL);
	if (!li)
		return -ENOMEM;
	li->num[0].cpus = 1;
	li->num[0].codecs = 1;
	li->num[0].platforms = 1;
	li->link = 1;

	ret = simple_util_init_priv(priv, li);
	kfree(li);
	if (ret)
		return ret;

	ret = baikal_simple_card_parse(priv);
	if (ret) {
		dev_err_probe(dev, ret, "parse error\n");
		return ret;
	}

	snd_soc_card_set_drvdata(card, priv);
	simple_util_debug_info(priv);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id baikal_simple_card_of_match[] = {
	{ .compatible = "baikal,simple-audio-card" },
	{}
};
MODULE_DEVICE_TABLE(of, baikal_simple_card_of_match);

static struct platform_driver baikal_simple_card_driver = {
	.driver = {
		.name = "baikal-asoc-simple-card",
		.pm = &snd_soc_pm_ops,
		.of_match_table = baikal_simple_card_of_match
	},
	.probe = baikal_simple_card_probe
};

module_platform_driver(baikal_simple_card_driver);

MODULE_ALIAS("platform:baikal-asoc-simple-card");
MODULE_AUTHOR("Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal ASoC Simple Sound Card");
MODULE_LICENSE("GPL");
