// SPDX-License-Identifier: GPL-2.0-only
/*
 * Baikal DW APB timer driver for ACPI
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 *
 * Implementation based on dw_apb_timer_of.c
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/dw_apb_timer.h>
#include <linux/platform_device.h>
#include <linux/sched_clock.h>

static int __init timer_get_base_and_rate(struct platform_device *pdev,
					  void __iomem **base, unsigned long *rate)
{
	struct device *dev = &pdev->dev;
	int ret;

	*base = devm_platform_ioremap_resource(pdev, 0);
	if (!*base) {
		dev_err(dev, "Unable to map regs\n");
		return PTR_ERR(*base);
	}

	ret = device_property_read_u32(dev, "clock-frequency", (u32 *)rate);
	if (ret)
		return ret;

	return 0;
}

static int __init add_clockevent(struct platform_device *pdev)
{
	struct dw_apb_clock_event_device *ced;
	void __iomem *iobase;
	unsigned long rate;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ for clock event timer\n");
		return -EINVAL;
	}

	ret = timer_get_base_and_rate(pdev, &iobase, &rate);
	if (ret)
		return ret;

	ced = dw_apb_clockevent_init(-1, pdev->name, 300, iobase, irq, rate);
	if (!ced)
		return -EINVAL;

	dw_apb_clockevent_register(ced);

	return 0;
}

static void __iomem *sched_io_base;
static unsigned long sched_rate;

static int __init add_clocksource(struct platform_device *pdev)
{
	struct dw_apb_clocksource *cs;
	void __iomem *iobase;
	unsigned long rate;
	int ret;

	ret = timer_get_base_and_rate(pdev, &iobase, &rate);
	if (ret)
		return ret;

	cs = dw_apb_clocksource_init(300, pdev->name, iobase, rate);
	if (!cs)
		return -EINVAL;

	dw_apb_clocksource_start(cs);
	dw_apb_clocksource_register(cs);

	sched_io_base = iobase + 0x04;
	sched_rate = rate;

	return 0;
}

static u64 notrace read_sched_clock(void)
{
	return ~readl_relaxed(sched_io_base);
}

static int num_called;

static int __init baikal_dw_apb_timer_probe(struct platform_device *pdev)
{
	int ret;

	switch (num_called) {
	case 1:
		pr_debug("%s: found clocksource timer\n", __func__);
		ret = add_clocksource(pdev);
		if (ret)
			return ret;
		sched_clock_register(read_sched_clock, 32, sched_rate);
		break;
	default:
		pr_debug("%s: found clockevent timer\n", __func__);
		ret = add_clockevent(pdev);
		if (ret)
			return ret;
		break;
	}

	++num_called;

	return 0;
}

static const struct of_device_id baikal_dw_apb_timer_of_match[] = {
	{ .compatible = "baikal,bm1000-dw-apb-timer" },
	{ .compatible = "baikal,bs1000-dw-apb-timer" },
	{ .compatible = "baikal,dw-apb-timer" },
	{}
};

static struct platform_driver baikal_dw_apb_timer = {
	.driver = {
		.name = "baikal-dw-apb-timer",
		.of_match_table = baikal_dw_apb_timer_of_match,
		.suppress_bind_attrs = true
	}
};
builtin_platform_driver_probe(baikal_dw_apb_timer, baikal_dw_apb_timer_probe);
