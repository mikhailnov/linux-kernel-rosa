// SPDX-License-Identifier: GPL-2.0-only
/*
 * Baikal-M cpufreq driver
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 *
 * Implementation based on cpufreq-dt.c
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include "../opp/opp.h"

struct private_data {
	struct list_head node;

	cpumask_var_t cpus;
	struct device *cpu_dev;
	struct cpufreq_frequency_table *freq_table;
};

static LIST_HEAD(priv_list);

static struct freq_attr *baikal_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static int baikal_cpufreq_target_index(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct private_data *priv = policy->driver_data;
	unsigned long freq = policy->freq_table[index].frequency;

	return dev_pm_opp_set_rate(priv->cpu_dev, freq * 1000);
}

static struct private_data *baikal_cpufreq_find_data(int cpu)
{
	struct private_data *priv;

	list_for_each_entry(priv, &priv_list, node) {
		if (cpumask_test_cpu(cpu, priv->cpus))
			return priv;
	}

	return NULL;
}

static int baikal_cpufreq_init(struct cpufreq_policy *policy)
{
	struct private_data *priv;
	struct device *cpu_dev;
	struct clk *cpu_clk;
	unsigned int transition_latency;
	int ret;

	priv = baikal_cpufreq_find_data(policy->cpu);
	if (!priv) {
		pr_err("failed to find data for cpu%d\n", policy->cpu);
		return -ENODEV;
	}
	cpu_dev = priv->cpu_dev;

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		dev_err(cpu_dev, "%s: failed to get clk: %d\n", __func__, ret);
		return ret;
	}

	transition_latency = dev_pm_opp_get_max_transition_latency(cpu_dev);
	if (!transition_latency)
		transition_latency = CPUFREQ_ETERNAL;

	cpumask_copy(policy->cpus, priv->cpus);
	policy->driver_data = priv;
	policy->clk = cpu_clk;
	policy->freq_table = priv->freq_table;
	policy->suspend_freq = dev_pm_opp_get_suspend_opp_freq(cpu_dev) / 1000;
	policy->cpuinfo.transition_latency = transition_latency;
	policy->dvfs_possible_from_any_cpu = true;

	return 0;
}

static int baikal_cpufreq_online(struct cpufreq_policy *policy)
{
	return 0;
}

static int baikal_cpufreq_offline(struct cpufreq_policy *policy)
{
	return 0;
}

static void baikal_cpufreq_exit(struct cpufreq_policy *policy)
{
	clk_put(policy->clk);
}

static struct cpufreq_driver baikal_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = baikal_cpufreq_target_index,
	.get = cpufreq_generic_get,
	.init = baikal_cpufreq_init,
	.exit = baikal_cpufreq_exit,
	.online = baikal_cpufreq_online,
	.offline = baikal_cpufreq_offline,
	.name = "baikal-cpufreq",
	.attr = baikal_cpufreq_attr,
	.suspend = cpufreq_generic_suspend,
};

static unsigned long baikal_cpufreq_opp_table_hz[] = {
	500000000,
	600000000,
	700000000,
	800000000,
	900000000,
	1000000000,
	1100000000,
	1200000000,
	1300000000,
	1400000000,
	1500000000
};

static struct dev_pm_opp *baikal_cpufreq_opp_get(struct opp_table *opp_table,
						 int num)
{
	struct dev_pm_opp *opp = NULL, *temp;

	mutex_lock(&opp_table->lock);
	list_for_each_entry(temp, &opp_table->opp_list, node) {
		if (!temp->removed && temp->dynamic &&
		    temp->rates[0] == baikal_cpufreq_opp_table_hz[num]) {
			opp = temp;
			break;
		}
	}
	mutex_unlock(&opp_table->lock);

	return opp;
}

static int baikal_cpufreq_add_opp(struct device *dev)
{
	struct opp_table *opp_table = NULL;
	struct dev_pm_opp *opp;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(baikal_cpufreq_opp_table_hz); ++i) {
		ret = dev_pm_opp_add(dev, baikal_cpufreq_opp_table_hz[i], 0);
		if (ret)
			return ret;

		if (!opp_table) {
			opp_table = dev_pm_opp_get_opp_table(dev);
			if (!IS_ERR_OR_NULL(opp_table))
				opp_table->clock_latency_ns_max = 10000000;
		}

		opp = baikal_cpufreq_opp_get(opp_table, i);
		if (opp) {
			mutex_lock(&opp_table->lock);
			opp->clock_latency_ns = 10000000;
			mutex_unlock(&opp_table->lock);
		}
	}

	if (!IS_ERR_OR_NULL(opp_table))
		dev_pm_opp_put_opp_table(opp_table);

	return 0;
}

static void baikal_dev_pm_opp_remove_all_dynamic(const struct cpumask *cpumask)
{
	struct device *cpu_dev;
	int cpu;

	if (cpumask_empty(cpumask))
		return;

	for_each_cpu(cpu, cpumask) {
		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev)
			continue;

		dev_pm_opp_remove_all_dynamic(cpu_dev);
	}
}

static int baikal_cpufreq_add_opp_table(const struct cpumask *cpumask)
{
	struct device *cpu_dev;
	int cpu, ret;

	if (cpumask_empty(cpumask))
		return -ENODEV;

	for_each_cpu(cpu, cpumask) {
		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev) {
			pr_err("%s: failed to get cpu%d device\n", __func__,
			       cpu);
			ret = -ENODEV;
			goto remove_table;
		}

		ret = baikal_cpufreq_add_opp(cpu_dev);
		if (ret)
			goto remove_table;
	}

	return 0;

remove_table:
	baikal_dev_pm_opp_remove_all_dynamic(cpumask);

	return ret;
}

static int baikal_cpufreq_early_init(struct device *dev, int cpu)
{
	struct private_data *priv;
	struct device *cpu_dev, *tmp;
	int ret, i;

	/* Check if this CPU is already covered by some other policy */
	if (baikal_cpufreq_find_data(cpu))
		return 0;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (!alloc_cpumask_var(&priv->cpus, GFP_KERNEL))
		return -ENOMEM;

	cpumask_set_cpu(cpu, priv->cpus);
	priv->cpu_dev = cpu_dev;

	for_each_possible_cpu(i) {
		if (i == cpu_dev->id)
			continue;

		tmp = get_cpu_device(i);
		if (!tmp) {
			ret = -ENODEV;
			goto free_cpumask;
		}

		if (ACPI_COMPANION(cpu_dev)->dev.parent ==
		    ACPI_COMPANION(tmp)->dev.parent)
			cpumask_set_cpu(i, priv->cpus);
	}

	ret = baikal_cpufreq_add_opp_table(priv->cpus);
	if (ret)
		goto free_cpumask;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &priv->freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out;
	}

	list_add(&priv->node, &priv_list);
	return 0;

out:
	dev_pm_opp_remove_all_dynamic(priv->cpu_dev);
free_cpumask:
	free_cpumask_var(priv->cpus);
	return ret;
}

static void baikal_cpufreq_release(void)
{
	struct private_data *priv, *tmp;

	list_for_each_entry_safe(priv, tmp, &priv_list, node) {
		dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &priv->freq_table);
		baikal_dev_pm_opp_remove_all_dynamic(priv->cpus);
		free_cpumask_var(priv->cpus);
		list_del(&priv->node);
	}
}

static int baikal_cpufreq_probe(struct platform_device *pdev)
{
	int ret, cpu;

	for_each_possible_cpu(cpu) {
		struct device *cpu_dev;
		struct clk *cpu_clk;

		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev)
			return -ENODEV;

		cpu_clk = clk_get(cpu_dev, NULL);
		if (IS_ERR(cpu_clk))
			return -ENODEV;
		else
			clk_put(cpu_clk);
	}

	for_each_possible_cpu(cpu) {
		ret = baikal_cpufreq_early_init(&pdev->dev, cpu);
		if (ret) {
			dev_err(&pdev->dev, "failed to add cpufreq table: %d\n", ret);
			goto err;
		}
	}

	ret = cpufreq_register_driver(&baikal_cpufreq_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed register driver: %d\n", ret);
		goto err;
	}

	return 0;
err:
	baikal_cpufreq_release();
	return ret;
}

static void baikal_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&baikal_cpufreq_driver);
	baikal_cpufreq_release();
}

static struct platform_driver baikal_cpufreq_platform_driver = {
	.driver = {
		.name = "baikal-cpufreq",
		.suppress_bind_attrs = true
	},
	.probe		= baikal_cpufreq_probe,
	.remove		= baikal_cpufreq_remove
};
builtin_platform_driver(baikal_cpufreq_platform_driver);

static int __init baikal_cpufreq_driver_init(void)
{
	if (!acpi_disabled)
		return PTR_ERR_OR_ZERO(platform_device_register_data(NULL, "baikal-cpufreq", -1,
								     NULL, 0));

	return 0;
}
core_initcall(baikal_cpufreq_driver_init);

MODULE_ALIAS("platform:baikal-cpufreq");
MODULE_AUTHOR("Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal-M cpufreq driver");
MODULE_LICENSE("GPL");
