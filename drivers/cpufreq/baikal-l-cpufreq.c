// SPDX-License-Identifier: GPL-2.0-only
/*
 * Baikal-L cpufreq driver
 *
 * Copyright (C) 2024-2025 Baikal Electronics, JSC
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/firmware/baikal/baikal-smc.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include "../opp/opp.h"

#define BAIKAL_MAX_COEF		31

static u32 baikal_regs[] = {
	0x388400E0,
	0x38840148,
	0x38840160,
	0x38840198,
};

struct private_data {
	struct list_head node;

	cpumask_var_t cpumask;
	spinlock_t lock;
	void __iomem *reg;
	struct device *cpu_dev;
	struct cpufreq_frequency_table *freq_table;
	u32 *data;
	int size;
	int num;
	int cpu;
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
	int idx = 2 * ((priv->freq_table[index].driver_data >> 16) & 0xffff);
	u8 count = priv->freq_table[index].driver_data & 0x1f;
	int ret;
	struct arm_smccc_res res;

	spin_lock(&priv->lock);
	ret = dev_pm_opp_set_rate(priv->cpu_dev, priv->data[idx]);
	if (ret) {
		spin_unlock(&priv->lock);
		return ret;
	}
	/* TODO change to writel */
	arm_smccc_smc(BAIKAL_SMC_THROTTLE, 0, priv->cpu, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(BAIKAL_SMC_THROTTLE, 0, priv->cpu,
		      0x80000000 | ((1 << count) - 1), 0, 0, 0, 0, &res);
	spin_unlock(&priv->lock);

	return 0;
}

static struct private_data *baikal_cpufreq_find_data(int cpu)
{
	struct private_data *priv;

	list_for_each_entry(priv, &priv_list, node) {
		if (cpumask_test_cpu(cpu, priv->cpumask))
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

	cpumask_copy(policy->cpus, priv->cpumask);
	policy->driver_data = priv;
	policy->clk = cpu_clk;
	policy->freq_table = priv->freq_table;
	policy->suspend_freq = 0;
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

static u8 baikal_bit_count(const unsigned long int *val)
{
	u8 i, count = 0;

	for_each_set_bit(i, val, 32) {
		++count;
	}

	return count;
}

static unsigned int baikal_cpufreq_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct private_data *priv;
	unsigned long int rate;
	u8 count;
	struct arm_smccc_res res;

	if (!policy || IS_ERR(policy->clk)) {
		pr_err("%s: No %s associated to cpu: %d\n",
		       __func__, policy ? "clk" : "policy", cpu);
		return 0;
	}

	priv = policy->driver_data;
	spin_lock(&priv->lock);
	/* TODO change to readl */
	arm_smccc_smc(BAIKAL_SMC_THROTTLE, 1, priv->cpu, 0, 0, 0, 0, 0, &res);
	rate = res.a0 & ~0xC0000000;
	count = baikal_bit_count(&rate);
	if (count)
		rate = clk_get_rate(policy->clk) / 1000 / BAIKAL_MAX_COEF *
			(BAIKAL_MAX_COEF - count);
	else
		rate = clk_get_rate(policy->clk) / 1000;
	spin_unlock(&priv->lock);

	return rate;
}

static struct cpufreq_driver baikal_l_cpufreq_driver = {
	.flags = CPUFREQ_CONST_LOOPS | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = baikal_cpufreq_target_index,
	.get = baikal_cpufreq_get,
	.init = baikal_cpufreq_init,
	.exit = baikal_cpufreq_exit,
	.online = baikal_cpufreq_online,
	.offline = baikal_cpufreq_offline,
	.name = "baikal-cpufreq",
	.attr = baikal_cpufreq_attr,
};

static struct dev_pm_opp *baikal_cpufreq_opp_get(struct opp_table *opp_table,
						 int num, u32 val)
{
	struct dev_pm_opp *opp = NULL, *temp;

	mutex_lock(&opp_table->lock);
	list_for_each_entry(temp, &opp_table->opp_list, node) {
		if (!temp->removed && temp->dynamic &&
		    temp->rates[0] == val) {
			opp = temp;
			break;
		}
	}
	mutex_unlock(&opp_table->lock);

	return opp;
}

static int baikal_cpufreq_add_opp(struct private_data *priv)
{
	struct opp_table *opp_table = NULL;
	struct dev_pm_opp *opp;
	int i, ret;

	for (i = 0; i < priv->size / 2; ++i) {
		ret = dev_pm_opp_add(priv->cpu_dev, priv->data[i * 2], 0);
		if (ret)
			return ret;

		if (!opp_table) {
			opp_table = dev_pm_opp_get_opp_table(priv->cpu_dev);
			if (!IS_ERR_OR_NULL(opp_table))
				opp_table->clock_latency_ns_max = 10000000;
		}

		opp = baikal_cpufreq_opp_get(opp_table, i, priv->data[i * 2]);
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

static int baikal_cpufreq_early_init(struct device *dev, int cpu)
{
	struct private_data *priv;
	int i, j, ret;
	unsigned long int val;

	if (cpu > 4)
		return 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = devm_ioremap(dev, baikal_regs[cpu], 4);
	if (!priv->reg)
		return -ENOMEM;

	priv->cpu_dev = get_cpu_device(cpu);
	priv->cpu = cpu;
	ret = device_property_read_u32_array(priv->cpu_dev, "baikal,cpu-freqs",
					     NULL, 0);
	if (ret > 0 && ret % 2 == 0) {
		priv->size = ret;
		priv->data = devm_kzalloc(dev, sizeof(*priv->data) * priv->size,
					  GFP_KERNEL);
		if (!priv->data)
			return -ENOMEM;
		device_property_read_u32_array(priv->cpu_dev, "baikal,cpu-freqs",
					       priv->data, priv->size);
	} else {
		return -EINVAL;
	}

	priv->num = 1;
	for (i = 0; i < priv->size / 2; ++i) {
		priv->data[2 * i + 1] &= ~0xC0000000;
		val = priv->data[2 * i + 1];
		priv->num += baikal_bit_count(&val);
	}

	priv->freq_table = devm_kzalloc(dev, sizeof(*priv->freq_table) *
					     (priv->num + 1), GFP_KERNEL);
	if (!priv->freq_table)
		return -ENOMEM;

	ret = baikal_cpufreq_add_opp(priv);
	if (ret) {
		free_cpumask_var(priv->cpumask);
		return ret;
	}

	ret = 0;
	for (i = 0; i < priv->size / 2; ++i) {
		for (j = 0; j < BAIKAL_MAX_COEF - 1; ++j) {
			if (priv->data[2 * i + 1] & BIT(j)) {
				priv->freq_table[ret].driver_data = (i << 16) | (j + 1);
				priv->freq_table[ret].frequency = priv->data[2 * i] /
					1000 / BAIKAL_MAX_COEF * (BAIKAL_MAX_COEF - 1 - j);
				++ret;
			}
		}
		priv->freq_table[ret].driver_data = (i << 16);
		priv->freq_table[ret].frequency = priv->data[2 * i] / 1000;
		++ret;
	}

	priv->freq_table[ret].driver_data = ret;
	priv->freq_table[ret].frequency = CPUFREQ_TABLE_END;

	if (!alloc_cpumask_var(&priv->cpumask, GFP_KERNEL))
		return -ENOMEM;

	cpumask_set_cpu(cpu, priv->cpumask);
	spin_lock_init(&priv->lock);

	list_add(&priv->node, &priv_list);
	return 0;
}

static void baikal_cpufreq_release(void)
{
	struct private_data *priv, *tmp;

	list_for_each_entry_safe(priv, tmp, &priv_list, node) {
		dev_pm_opp_remove_all_dynamic(priv->cpu_dev);
		free_cpumask_var(priv->cpumask);
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

	ret = cpufreq_register_driver(&baikal_l_cpufreq_driver);
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
	cpufreq_unregister_driver(&baikal_l_cpufreq_driver);
	baikal_cpufreq_release();
}

static struct platform_driver baikal_l_cpufreq_platform_driver = {
	.driver = {
		.name = "baikal-l-cpufreq",
		.suppress_bind_attrs = true
	},
	.probe		= baikal_cpufreq_probe,
	.remove		= baikal_cpufreq_remove
};
builtin_platform_driver(baikal_l_cpufreq_platform_driver);

static int __init baikal_l_cpufreq_driver_init(void)
{
	struct device_node *np = of_find_node_by_path("/");

	if (!np)
		return -ENODEV;

	if (of_device_is_compatible(np, "baikal,bl1000")) {
		of_node_put(np);
		return PTR_ERR_OR_ZERO(platform_device_register_data(NULL,
				       "baikal-l-cpufreq", -1, NULL, 0));
	}

	of_node_put(np);
	return -ENODEV;
}
core_initcall(baikal_l_cpufreq_driver_init);

MODULE_DESCRIPTION("Baikal-L cpufreq driver");
MODULE_LICENSE("GPL");
