// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2025 BAIKAL ELECTRONICS, JSC
 *
 * Authors:
 *   Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>
 *   Serge Semin <Sergey.Semin@baikalelectronics.ru>
 *
 * Baikal-T1 Process, Voltage, Temperature sensor driver
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/polynomial.h>

#include "baikal-pvt.h"

/*
 * For the sake of the code simplification we created the sensors info table
 * with the sensor names, activation modes, threshold registers base address
 * and the thresholds bit fields.
 */
static const struct pvt_sensor_info pvt_info[] = {
	PVT_SENSOR_INFO(0, "CPU Core Temperature", hwmon_temp, TEMP, TTHRES),
	PVT_SENSOR_INFO(0, "CPU Core Voltage", hwmon_in, VOLT, VTHRES),
	PVT_SENSOR_INFO(1, "CPU Core Low-Vt", hwmon_in, LVT, LTHRES),
	PVT_SENSOR_INFO(2, "CPU Core High-Vt", hwmon_in, HVT, HTHRES),
	PVT_SENSOR_INFO(3, "CPU Core Standard-Vt", hwmon_in, SVT, STHRES),
};

static const struct hwmon_channel_info * const pvt_channel_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_TYPE | HWMON_T_LABEL |
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
			   HWMON_T_MIN | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX | HWMON_T_MAX_ALARM |
#endif
			   HWMON_T_OFFSET),
	HWMON_CHANNEL_INFO(in,
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM |
#endif
			   HWMON_I_INPUT | HWMON_I_LABEL,
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM |
#endif
			   HWMON_I_INPUT | HWMON_I_LABEL,
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM |
#endif
			   HWMON_I_INPUT | HWMON_I_LABEL,
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM |
#endif
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

/*
 * The original translation formulae of the temperature (in degrees of Celsius)
 * to PVT data and vice-versa are following:
 * N = 1.8322e-8*(T^4) + 2.343e-5*(T^3) + 8.7018e-3*(T^2) + 3.9269*(T^1) +
 *     1.7204e2,
 * T = -1.6743e-11*(N^4) + 8.1542e-8*(N^3) + -1.8201e-4*(N^2) +
 *     3.1020e-1*(N^1) - 4.838e1,
 * where T = [-48.380, 147.438]C and N = [0, 1023].
 * They must be accordingly altered to be suitable for the integer arithmetics.
 * The technique is called 'factor redistribution', which just makes sure the
 * multiplications and divisions are made so to have a result of the operations
 * within the integer numbers limit. In addition we need to translate the
 * formulae to accept millidegrees of Celsius. Here what they look like after
 * the alterations:
 * N = (18322e-20*(T^4) + 2343e-13*(T^3) + 87018e-9*(T^2) + 39269e-3*T +
 *     17204e2) / 1e4,
 * T = -16743e-12*(D^4) + 81542e-9*(D^3) - 182010e-6*(D^2) + 310200e-3*D -
 *     48380,
 * where T = [-48380, 147438] mC and N = [0, 1023].
 */
static const struct polynomial __maybe_unused poly_temp_to_N = {
	.total_divider = 10000,
	.terms = {
		{4, 18322, 10000, 10000},
		{3, 2343, 10000, 10},
		{2, 87018, 10000, 10},
		{1, 39269, 1000, 1},
		{0, 1720400, 1, 1}
	}
};

static const struct polynomial poly_N_to_temp = {
	.total_divider = 1,
	.terms = {
		{4, -16743, 1000, 1},
		{3, 81542, 1000, 1},
		{2, -182010, 1000, 1},
		{1, 310200, 1000, 1},
		{0, -48380, 1, 1}
	}
};

/*
 * Similar alterations are performed for the voltage conversion equations.
 * The original formulae are:
 * N = 1.8658e3*V - 1.1572e3,
 * V = (N + 1.1572e3) / 1.8658e3,
 * where V = [0.620, 1.168] V and N = [0, 1023].
 * After the optimization they looks as follows:
 * N = (18658e-3*V - 11572) / 10,
 * V = N * 10^5 / 18658 + 11572 * 10^4 / 18658.
 */
static const struct polynomial __maybe_unused poly_volt_to_N = {
	.total_divider = 10,
	.terms = {
		{1, 18658, 1000, 1},
		{0, -11572, 1, 1}
	}
};

static const struct polynomial poly_N_to_volt = {
	.total_divider = 10,
	.terms = {
		{1, 100000, 18658, 1},
		{0, 115720000, 1, 18658}
	}
};

static u32 bt1_read_pvt(struct pvt_hwmon *pvt, u32 reg)
{
	return readl(pvt->regs + reg);
}

static int bt1_write_pvt(struct pvt_hwmon *pvt, u32 reg, u32 data)
{
	writel(data, pvt->regs + reg);
	return 0;
}

static u32 bt1_to_pvt(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
		      long val)
{
	const struct polynomial *poly = (type == PVT_TEMP) ?
					&poly_temp_to_N : &poly_volt_to_N;

	return polynomial_calc(poly, val);
}

static long bt1_from_pvt(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			 u32 data)
{
	const struct polynomial *poly = (type == PVT_TEMP) ?
					&poly_N_to_temp : &poly_N_to_volt;

	return polynomial_calc(poly, data);
}

static struct pvt_ops bt1_pvt_ops = {
	.read     = bt1_read_pvt,
	.write    = bt1_write_pvt,
	.to_pvt   = bt1_to_pvt,
	.from_pvt = bt1_from_pvt,
};

static int pvt_probe(struct platform_device *pdev)
{
	return baikal_pvt_create(pdev, &bt1_pvt_ops, pvt_info, pvt_channel_info);
}

static const struct of_device_id pvt_of_match[] = {
	{ .compatible = "baikal,bt1-pvt" },
	{ }
};
MODULE_DEVICE_TABLE(of, pvt_of_match);

static struct platform_driver pvt_driver = {
	.probe  = pvt_probe,
	.driver = {
		.name = "bt1-pvt",
		.of_match_table = pvt_of_match
	}
};
module_platform_driver(pvt_driver);

MODULE_AUTHOR("Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal-T1 PVT driver");
MODULE_LICENSE("GPL v2");
