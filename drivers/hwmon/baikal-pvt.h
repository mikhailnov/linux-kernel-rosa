/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020-2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal SoCs Process, Voltage, Temperature sensor driver
 */
#ifndef __HWMON_BAIKAL_PVT_H__
#define __HWMON_BAIKAL_PVT_H__

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/seqlock.h>
#include <linux/thermal.h>

/* Baikal SoC's PVT registers and their bitfields */
#define PVT_CTRL			0x00
#define PVT_CTRL_EN			BIT(0)
#define PVT_CTRL_MODE_FLD		1
#define PVT_CTRL_MODE_MASK		GENMASK(3, PVT_CTRL_MODE_FLD)
#define PVT_CTRL_MODE_TEMP		0x0
#define PVT_CTRL_MODE_VOLT		0x1
#define PVT_CTRL_MODE_LVT		0x2
#define PVT_CTRL_MODE_HVT		0x4
#define PVT_CTRL_MODE_ULVT		PVT_CTRL_MODE_HVT
#define PVT_CTRL_MODE_SVT		0x6
#define PVT_CTRL_TRIM_FLD		4
#define PVT_CTRL_TRIM_MASK		GENMASK(8, PVT_CTRL_TRIM_FLD)
#define PVT_DATA			0x04
#define PVT_DATA_VALID			BIT(10)
#define PVT_DATA_DATA_FLD		0
#define PVT_DATA_DATA_MASK		GENMASK(9, PVT_DATA_DATA_FLD)
#define PVT_TTHRES			0x08
#define PVT_VTHRES			0x0C
#define PVT_LTHRES			0x10
#define PVT_HTHRES			0x14
#define PVT_UTHRES			PVT_HTHRES
#define PVT_STHRES			0x18
#define PVT_THRES_LO_FLD		0
#define PVT_THRES_LO_MASK		GENMASK(9, PVT_THRES_LO_FLD)
#define PVT_THRES_HI_FLD		10
#define PVT_THRES_HI_MASK		GENMASK(19, PVT_THRES_HI_FLD)
#define PVT_TTIMEOUT			0x1C
#define PVT_INTR_STAT			0x20
#define PVT_INTR_MASK			0x24
#define PVT_RAW_INTR_STAT		0x28
#define PVT_INTR_DVALID			BIT(0)
#define PVT_INTR_TTHRES_LO		BIT(1)
#define PVT_INTR_TTHRES_HI		BIT(2)
#define PVT_INTR_VTHRES_LO		BIT(3)
#define PVT_INTR_VTHRES_HI		BIT(4)
#define PVT_INTR_LTHRES_LO		BIT(5)
#define PVT_INTR_LTHRES_HI		BIT(6)
#define PVT_INTR_HTHRES_LO		BIT(7)
#define PVT_INTR_HTHRES_HI		BIT(8)
#define PVT_INTR_UTHRES_LO		PVT_INTR_HTHRES_LO
#define PVT_INTR_UTHRES_HI		PVT_INTR_HTHRES_HI
#define PVT_INTR_STHRES_LO		BIT(9)
#define PVT_INTR_STHRES_HI		BIT(10)
#define PVT_INTR_ALL			GENMASK(10, 0)
#define PVT_CLR_INTR			0x2C

/*
 * PVT sensors-related limits and default values
 * @PVT_TEMP_MIN: Minimal temperature in millidegrees of Celsius.
 * @PVT_TEMP_MAX: Maximal temperature in millidegrees of Celsius.
 * @PVT_TEMP_CHS: Number of temperature hwmon channels.
 * @PVT_VOLT_MIN: Minimal voltage in mV.
 * @PVT_VOLT_MAX: Maximal voltage in mV.
 * @PVT_VOLT_CHS: Number of voltage hwmon channels.
 * @PVT_DATA_MIN: Minimal PVT raw data value.
 * @PVT_DATA_MAX: Maximal PVT raw data value.
 * @PVT_TRIM_MIN: Minimal temperature sensor trim value.
 * @PVT_TRIM_MAX: Maximal temperature sensor trim value.
 * @PVT_TRIM_DEF: Default temperature sensor trim value (set a proper value
 *		  when one is determined for Baikal SoC).
 * @PVT_TRIM_TEMP: Maximum temperature encoded by the trim factor.
 * @PVT_TRIM_STEP: Temperature stride corresponding to the trim value.
 * @PVT_TOUT_MIN: Minimal timeout between samples in nanoseconds.
 * @PVT_TOUT_DEF: Default data measurements timeout in nanoseconds.
 *		  In case if alarms are activated the PVT IRQ is enabled to be
 *		  raised after each conversion in order to have the thresholds
 *		  checked and the converted value cached. Too frequent
 *		  conversions may cause the system CPU overload. Lets set the
 *		  50ms delay between them by default to prevent this.
 */
#define PVT_TEMP_MIN		-48380L
#define PVT_TEMP_MAX		147438L
#define PVT_TEMP_CHS		1
#define PVT_VOLT_MIN		620L
#define PVT_VOLT_MAX		1168L
#define PVT_LVT_MIN		PVT_VOLT_MIN
#define PVT_LVT_MAX		PVT_VOLT_MAX
#define PVT_HVT_MIN		PVT_VOLT_MIN
#define PVT_HVT_MAX		PVT_VOLT_MAX
#define PVT_ULVT_MIN		PVT_VOLT_MIN
#define PVT_ULVT_MAX		PVT_VOLT_MAX
#define PVT_SVT_MIN		PVT_VOLT_MIN
#define PVT_SVT_MAX		PVT_VOLT_MAX
#define PVT_VOLT_CHS		4
#define PVT_DATA_MIN		0
#define PVT_DATA_MAX		(PVT_DATA_DATA_MASK >> PVT_DATA_DATA_FLD)
#define PVT_TRIM_MIN		0
#define PVT_TRIM_MAX		(PVT_CTRL_TRIM_MASK >> PVT_CTRL_TRIM_FLD)
#define PVT_TRIM_TEMP		7130
#define PVT_TRIM_STEP		(PVT_TRIM_TEMP / PVT_TRIM_MAX)
#define PVT_TRIM_DEF		0
#define PVT_TOUT_MIN		(NSEC_PER_SEC / 3000)
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
# define PVT_TOUT_DEF		250000000
#else
# define PVT_TOUT_DEF		PVT_TOUT_MIN
#endif

/*
 * enum pvt_sensor_type - Baikal SoC PVT sensor types (correspond to each PVT
 *			  sampling mode)
 * @PVT_SENSOR*: helpers to traverse the sensors in loops.
 * @PVT_TEMP: PVT Temperature sensor.
 * @PVT_VOLT: PVT Voltage sensor.
 * @PVT_LVT: PVT Low-Voltage threshold sensor.
 * @PVT_HVT: PVT High-Voltage threshold sensor.
 * @PVT_ULVT: PVT Ultra-Low-Voltage threshold sensor.
 * @PVT_SVT: PVT Standard-Voltage threshold sensor.
 */
enum pvt_sensor_type {
	PVT_SENSOR_FIRST,
	PVT_TEMP = PVT_SENSOR_FIRST,
	PVT_VOLT,
	PVT_LVT,
	PVT_HVT,
	PVT_ULVT = PVT_HVT,
	PVT_SVT,
	PVT_SENSOR_LAST = PVT_SVT,
	PVT_SENSORS_NUM
};

/*
 * enum pvt_clock_type - Baikal SoC PVT clocks.
 * @PVT_CLOCK_APB: APB clock.
 * @PVT_CLOCK_REF: PVT reference clock.
 */
enum pvt_clock_type {
	PVT_CLOCK_APB,
	PVT_CLOCK_REF,
	PVT_CLOCK_NUM
};

/*
 * struct pvt_sensor_info - Baikal SoC PVT sensor informational structure
 * @channel: Sensor channel ID.
 * @label: hwmon sensor label.
 * @mode: PVT mode corresponding to the channel.
 * @thres_base: upper and lower threshold values of the sensor.
 * @thres_sts_lo: low threshold status bitfield.
 * @thres_sts_hi: high threshold status bitfield.
 * @type: Sensor type.
 * @value_min: minimal sensor value.
 * @value_max: maximal sensor value.
 * @attr_min_alarm: Min alarm attribute ID.
 * @attr_min_alarm: Max alarm attribute ID.
 */
struct pvt_sensor_info {
	int channel;
	const char *label;
	u32 mode;
	unsigned long thres_base;
	u32 thres_sts_lo;
	u32 thres_sts_hi;
	enum hwmon_sensor_types type;
	long value_min;
	long value_max;
	u32 attr_min_alarm;
	u32 attr_max_alarm;
};

#define PVT_SENSOR_INFO(_ch, _label, _type, _mode, _thres)	\
	{							\
		.channel = _ch,					\
		.label = _label,				\
		.mode = PVT_CTRL_MODE_ ##_mode,			\
		.thres_base = PVT_ ##_thres,			\
		.thres_sts_lo = PVT_INTR_ ##_thres## _LO,	\
		.thres_sts_hi = PVT_INTR_ ##_thres## _HI,	\
		.type = _type,					\
		.value_min = PVT_ ##_mode## _MIN,		\
		.value_max = PVT_ ##_mode## _MAX,		\
		.attr_min_alarm = _type## _min_alarm,		\
		.attr_max_alarm = _type## _max_alarm,		\
	}

/*
 * struct pvt_cache - PVT sensors data cache
 * @data: data cache in raw format.
 * @thres_sts_lo: low threshold status saved on the previous data conversion.
 * @thres_sts_hi: high threshold status saved on the previous data conversion.
 * @data_seqlock: cached data seq-lock.
 * @conversion: data conversion completion.
 */
struct pvt_cache {
	u32 data;
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
	seqlock_t data_seqlock;
	u32 thres_sts_lo;
	u32 thres_sts_hi;
#else
	struct completion conversion;
#endif
};

struct pvt_hwmon;

/*
 * struct pvt_ops - PVT sensor operations
 * @read_pvt: read PVT register.
 * @write_pvt: write PVT register.
 * @to_pvt: convert temperature/voltage to PVT data.
 * @from_pvt: convert PVT data to temperature/voltage.
 */
struct pvt_ops {
	int  (*init)(struct pvt_hwmon *);
	u32  (*read)(struct pvt_hwmon *, u32);
	int  (*write)(struct pvt_hwmon *, u32, u32);
	u32  (*to_pvt)(struct pvt_hwmon *, enum pvt_sensor_type, long);
	long (*from_pvt)(struct pvt_hwmon *, enum pvt_sensor_type, u32);
};

/*
 * struct pvt_hwmon - Baikal SoC PVT private data
 * @dev: device structure of the PVT platform device.
 * @hwmon: hwmon device structure.
 * @regs: pointer to the Baikal SoC PVT registers region.
 * @irq: PVT events IRQ number.
 * @clks: Array of the PVT clocks descriptor (APB/ref clocks).
 * @iface_mtx: Generic interface mutex (used to lock the alarm registers
 *	       when the alarms enabled, or the data conversion interface
 *	       if alarms are disabled).
 * @sensor: current PVT sensor the data conversion is being performed for.
 * @cache: data cache descriptor.
 * @timeout: conversion timeout cache.
 * @ops: PVT operations.
 * @tzd: thermal zone.
 */
struct pvt_hwmon {
	struct device *dev;
	struct device *hwmon;

	void __iomem *regs;
	int irq;

	struct clk_bulk_data clks[PVT_CLOCK_NUM];

	struct mutex iface_mtx;
	enum pvt_sensor_type sensor;
	struct pvt_cache cache[PVT_SENSORS_NUM];
	ktime_t timeout;

	struct pvt_ops *ops;

	const struct pvt_sensor_info *info;

	struct thermal_zone_device *tzd;
};

int baikal_pvt_create(struct platform_device *pdev, struct pvt_ops *ops,
		      const struct pvt_sensor_info *info,
		      const struct hwmon_channel_info * const *hwmon_info);

#endif /* __HWMON_BAIKAL_PVT_H__ */
