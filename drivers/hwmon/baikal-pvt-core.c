// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2025 BAIKAL ELECTRONICS, JSC
 *
 * Authors:
 *   Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>
 *   Serge Semin <Sergey.Semin@baikalelectronics.ru>
 *
 * Baikal SoCs Process, Voltage, Temperature sensor drivers common code
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seqlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include "baikal-pvt.h"

#define PVT_THERMAL_POLLING_DELAY	5000

#define PVT_THERMAL_CRIT	90000	/* 90 degrees default critical temperature limit */
#define PVT_THERMAL_CRIT_HYST	2000

static inline u32 pvt_update(struct pvt_hwmon *pvt, u32 reg, u32 mask, u32 data)
{
	u32 old;

	old = pvt->ops->read(pvt, reg);
	pvt->ops->write(pvt, reg, (old & ~mask) | (data & mask));

	return old & mask;
}

/*
 * Baikal SoCs PVT mode can be updated only when the controller is disabled.
 * So first we disable it, then set the new mode together with the controller
 * getting back enabled. The same concerns the temperature trim and
 * measurements timeout. If it is necessary the interface mutex is supposed
 * to be locked at the time the operations are performed.
 */
static inline void pvt_set_mode(struct pvt_hwmon *pvt, u32 mode)
{
	u32 old;

	mode = FIELD_PREP(PVT_CTRL_MODE_MASK, mode);

	old = pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_MODE_MASK | PVT_CTRL_EN, mode | old);
}

static inline u32 pvt_calc_trim(long temp)
{
	temp = clamp_val(temp, 0, PVT_TRIM_TEMP);

	return DIV_ROUND_UP(temp, PVT_TRIM_STEP);
}

static inline void pvt_set_trim(struct pvt_hwmon *pvt, u32 trim)
{
	u32 old;

	trim = FIELD_PREP(PVT_CTRL_TRIM_MASK, trim);

	old = pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_TRIM_MASK | PVT_CTRL_EN, trim | old);
}

static inline void pvt_set_tout(struct pvt_hwmon *pvt, u32 tout)
{
	u32 old;

	old = pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt->ops->write(pvt, PVT_TTIMEOUT, tout);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, old);
}

/*
 * This driver can optionally provide the hwmon alarms for each sensor the PVT
 * controller supports. The alarms functionality is made compile-time
 * configurable due to the hardware interface implementation peculiarity
 * described further in this comment. So in case if alarms are unnecessary in
 * your system design it's recommended to have them disabled to prevent the PVT
 * IRQs being periodically raised to get the data cache/alarms status up to
 * date.
 *
 * Baikal SoCs PVT embedded controller is based on the Analog Bits PVT sensor,
 * but is equipped with a dedicated control wrapper. It exposes the PVT
 * sub-block registers space via the APB3 bus. In addition the wrapper provides
 * a common interrupt vector of the sensors conversion completion events and
 * threshold value alarms. Alas the wrapper interface hasn't been fully thought
 * through. There is only one sensor can be activated at a time, for which the
 * thresholds comparator is enabled right after the data conversion is
 * completed. Due to this if alarms need to be implemented for all available
 * sensors we can't just set the thresholds and enable the interrupts. We need
 * to enable the sensors one after another and let the controller to detect
 * the alarms by itself at each conversion. This also makes pointless to handle
 * the alarms interrupts, since in occasion they happen synchronously with
 * data conversion completion. The best driver design would be to have the
 * completion interrupts enabled only and keep the converted value in the
 * driver data cache. This solution is implemented if hwmon alarms are enabled
 * in this driver. In case if the alarms are disabled, the conversion is
 * performed on demand at the time a sensors input file is read.
 */

#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)

#define pvt_hard_isr NULL

static irqreturn_t pvt_soft_isr(int irq, void *data)
{
	const struct pvt_sensor_info *info;
	struct pvt_hwmon *pvt = data;
	struct pvt_cache *cache;
	u32 val, thres_sts, old;

	/*
	 * DVALID bit will be cleared by reading the data. We need to save the
	 * status before the next conversion happens. Threshold events will be
	 * handled a bit later.
	 */
	thres_sts = pvt->ops->read(pvt, PVT_RAW_INTR_STAT);

	/*
	 * Then lets recharge the PVT interface with the next sampling mode.
	 * Lock the interface mutex to serialize trim, timeouts and alarm
	 * thresholds settings.
	 */
	cache = &pvt->cache[pvt->sensor];
	info = &pvt->info[pvt->sensor];
	pvt->sensor = (pvt->sensor == PVT_SENSOR_LAST) ?
		      PVT_SENSOR_FIRST : (pvt->sensor + 1);

	/*
	 * For some reason we have to mask the interrupt before changing the
	 * mode, otherwise sometimes the temperature mode doesn't get
	 * activated even though the actual mode in the ctrl register
	 * corresponds to one. Then we read the data. By doing so we also
	 * recharge the data conversion. After this the mode corresponding
	 * to the next sensor in the row is set. Finally we enable the
	 * interrupts back.
	 */
	mutex_lock(&pvt->iface_mtx);

	old = pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, PVT_INTR_DVALID);

	val = pvt->ops->read(pvt, PVT_DATA);

	pvt_set_mode(pvt, pvt->info[pvt->sensor].mode);

	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, old);

	mutex_unlock(&pvt->iface_mtx);

	/*
	 * We can now update the data cache with data just retrieved from the
	 * sensor. Lock write-seqlock to make sure the reader has a coherent
	 * data.
	 */
	write_seqlock(&cache->data_seqlock);

	cache->data = FIELD_GET(PVT_DATA_DATA_MASK, val);

	write_sequnlock(&cache->data_seqlock);

	/*
	 * While PVT core is doing the next mode data conversion, we'll check
	 * whether the alarms were triggered for the current sensor. Note that
	 * according to the documentation only one threshold IRQ status can be
	 * set at a time, that's why if-else statement is utilized.
	 */
	if ((thres_sts & info->thres_sts_lo) ^ cache->thres_sts_lo) {
		WRITE_ONCE(cache->thres_sts_lo, thres_sts & info->thres_sts_lo);
		hwmon_notify_event(pvt->hwmon, info->type, info->attr_min_alarm,
				   info->channel);
		if (info->type == hwmon_temp)
			thermal_zone_device_update(pvt->tzd, THERMAL_EVENT_UNSPECIFIED);
	} else if ((thres_sts & info->thres_sts_hi) ^ cache->thres_sts_hi) {
		WRITE_ONCE(cache->thres_sts_hi, thres_sts & info->thres_sts_hi);
		hwmon_notify_event(pvt->hwmon, info->type, info->attr_max_alarm,
				   info->channel);
		if (info->type == hwmon_temp)
			thermal_zone_device_update(pvt->tzd, THERMAL_EVENT_UNSPECIFIED);
	}

	return IRQ_HANDLED;
}

static inline umode_t pvt_limit_is_visible(enum pvt_sensor_type type)
{
	return 0644;
}

static inline umode_t pvt_alarm_is_visible(enum pvt_sensor_type type)
{
	return 0444;
}

static int pvt_read_data(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			 long *val)
{
	struct pvt_cache *cache = &pvt->cache[type];
	unsigned int seq;
	u32 data;

	do {
		seq = read_seqbegin(&cache->data_seqlock);
		data = cache->data;
	} while (read_seqretry(&cache->data_seqlock, seq));

	*val = pvt->ops->from_pvt(pvt, type, data);

	return 0;
}

static int pvt_read_limit(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			  bool is_low, long *val)
{
	u32 data;

	/* No need in serialization, since it is just read from MMIO. */
	data = pvt->ops->read(pvt, pvt->info[type].thres_base);

	if (is_low)
		data = FIELD_GET(PVT_THRES_LO_MASK, data);
	else
		data = FIELD_GET(PVT_THRES_HI_MASK, data);

	*val = pvt->ops->from_pvt(pvt, type, data);

	return 0;
}

static int pvt_write_limit(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			   bool is_low, long val)
{
	u32 data, limit, mask;
	int ret;

	val = clamp(val, pvt->info[type].value_min, pvt->info[type].value_max);
	data = pvt->ops->to_pvt(pvt, type, val);

	/* Serialize limit update, since a part of the register is changed. */
	ret = mutex_lock_interruptible(&pvt->iface_mtx);
	if (ret)
		return ret;

	/* Make sure the upper and lower ranges don't intersect. */
	limit = pvt->ops->read(pvt, pvt->info[type].thres_base);
	if (is_low) {
		limit = FIELD_GET(PVT_THRES_HI_MASK, limit);
		data = clamp_val(data, PVT_DATA_MIN, limit);
		data = FIELD_PREP(PVT_THRES_LO_MASK, data);
		mask = PVT_THRES_LO_MASK;
	} else {
		limit = FIELD_GET(PVT_THRES_LO_MASK, limit);
		data = clamp_val(data, limit, PVT_DATA_MAX);
		data = FIELD_PREP(PVT_THRES_HI_MASK, data);
		mask = PVT_THRES_HI_MASK;
	}

	pvt_update(pvt, pvt->info[type].thres_base, mask, data);

	mutex_unlock(&pvt->iface_mtx);

	return 0;
}

static int pvt_read_alarm(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			  bool is_low, long *val)
{
	if (is_low)
		*val = !!READ_ONCE(pvt->cache[type].thres_sts_lo);
	else
		*val = !!READ_ONCE(pvt->cache[type].thres_sts_hi);

	return 0;
}

#else /* !CONFIG_SENSORS_BAIKAL_PVT_ALARMS */

static irqreturn_t pvt_hard_isr(int irq, void *data)
{
	struct pvt_hwmon *pvt = data;
	struct pvt_cache *cache;
	u32 val;

	/*
	 * Mask the DVALID interrupt so after exiting from the handler a
	 * repeated conversion wouldn't happen.
	 */
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, PVT_INTR_DVALID);

	/*
	 * Nothing special for alarm-less driver. Just read the data, update
	 * the cache and notify a waiter of this event.
	 */
	val = pvt->ops->read(pvt, PVT_DATA);
	if (!(val & PVT_DATA_VALID)) {
		dev_err(pvt->dev, "Got IRQ when data isn't valid\n");
		return IRQ_HANDLED;
	}

	cache = &pvt->cache[pvt->sensor];

	WRITE_ONCE(cache->data, FIELD_GET(PVT_DATA_DATA_MASK, val));

	complete(&cache->conversion);

	return IRQ_HANDLED;
}

#define pvt_soft_isr NULL

static inline umode_t pvt_limit_is_visible(enum pvt_sensor_type type)
{
	return 0;
}

static inline umode_t pvt_alarm_is_visible(enum pvt_sensor_type type)
{
	return 0;
}

static int pvt_read_data(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			 long *val)
{
	struct pvt_cache *cache = &pvt->cache[type];
	unsigned long timeout;
	u32 data;
	int ret;

	/*
	 * Lock PVT conversion interface until data cache is updated. The
	 * data read procedure is following: set the requested PVT sensor
	 * mode, enable IRQ and conversion, wait until conversion is finished,
	 * then disable conversion and IRQ, and read the cached data.
	 */
	ret = mutex_lock_interruptible(&pvt->iface_mtx);
	if (ret)
		return ret;

	pvt->sensor = type;
	pvt_set_mode(pvt, pvt->info[type].mode);

	/*
	 * Unmask the DVALID interrupt and enable the sensors conversions.
	 * Do the reverse procedure when conversion is done.
	 */
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, 0);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, PVT_CTRL_EN);

	/*
	 * Wait with timeout since in case if the sensor is suddenly powered
	 * down the request won't be completed and the caller will hang up on
	 * this procedure until the power is back up again. Multiply the
	 * timeout by the factor of two to prevent a false timeout.
	 */
	timeout = 2 * usecs_to_jiffies(ktime_to_us(pvt->timeout));
	ret = wait_for_completion_timeout(&cache->conversion, timeout);

	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, PVT_INTR_DVALID);

	data = READ_ONCE(cache->data);

	mutex_unlock(&pvt->iface_mtx);

	if (!ret)
		return -ETIMEDOUT;

	*val = pvt->ops->from_pvt(pvt, type, data);

	return 0;
}

static int pvt_read_limit(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			  bool is_low, long *val)
{
	return -EOPNOTSUPP;
}

static int pvt_write_limit(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			   bool is_low, long val)
{
	return -EOPNOTSUPP;
}

static int pvt_read_alarm(struct pvt_hwmon *pvt, enum pvt_sensor_type type,
			  bool is_low, long *val)
{
	return -EOPNOTSUPP;
}

#endif /* !CONFIG_SENSORS_BAIKAL_PVT_ALARMS */

static inline bool pvt_hwmon_channel_is_valid(enum hwmon_sensor_types type,
					      int ch)
{
	switch (type) {
	case hwmon_temp:
		if (ch < 0 || ch >= PVT_TEMP_CHS)
			return false;
		break;
	case hwmon_in:
		if (ch < 0 || ch >= PVT_VOLT_CHS)
			return false;
		break;
	default:
		break;
	}

	/* The rest of the types are independent from the channel number. */
	return true;
}

static umode_t pvt_hwmon_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int ch)
{
	if (!pvt_hwmon_channel_is_valid(type, ch))
		return 0;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return 0644;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_type:
		case hwmon_temp_label:
			return 0444;
		case hwmon_temp_min:
		case hwmon_temp_max:
			return pvt_limit_is_visible(PVT_TEMP + ch);
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
			return pvt_alarm_is_visible(PVT_TEMP + ch);
		case hwmon_temp_offset:
			return 0644;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			return 0444;
		case hwmon_in_min:
		case hwmon_in_max:
			return pvt_limit_is_visible(PVT_VOLT + ch);
		case hwmon_in_min_alarm:
		case hwmon_in_max_alarm:
			return pvt_alarm_is_visible(PVT_VOLT + ch);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int pvt_read_trim(struct pvt_hwmon *pvt, long *val)
{
	u32 data;

	data = pvt->ops->read(pvt, PVT_CTRL);
	*val = FIELD_GET(PVT_CTRL_TRIM_MASK, data) * PVT_TRIM_STEP;

	return 0;
}

static int pvt_write_trim(struct pvt_hwmon *pvt, long val)
{
	u32 trim;
	int ret;

	/*
	 * Serialize trim update, since a part of the register is changed and
	 * the controller is supposed to be disabled during this operation.
	 */
	ret = mutex_lock_interruptible(&pvt->iface_mtx);
	if (ret)
		return ret;

	trim = pvt_calc_trim(val);
	pvt_set_trim(pvt, trim);

	mutex_unlock(&pvt->iface_mtx);

	return 0;
}

static int pvt_read_timeout(struct pvt_hwmon *pvt, long *val)
{
	int ret;

	ret = mutex_lock_interruptible(&pvt->iface_mtx);
	if (ret)
		return ret;

	/* Return the result in msec as hwmon sysfs interface requires. */
	*val = ktime_to_ms(pvt->timeout);

	mutex_unlock(&pvt->iface_mtx);

	return 0;
}

static u32 pvt_calc_tout(ktime_t timeout, unsigned long rate)
{
	ktime_t kt;
	u32 tout;

	/*
	 * Subtract a constant lag, which always persists due to the limited
	 * PVT sampling rate. Make sure the timeout is not negative.
	 */
	kt = ktime_sub_ns(timeout, PVT_TOUT_MIN);
	if (ktime_to_ns(kt) < 0)
		kt = ktime_set(0, 0);

	/*
	 * Recalculate the timeout in terms of the reference clock
	 * period.
	 */
	tout = ktime_divns(kt * rate, NSEC_PER_SEC);

	return tout;
}

static int pvt_write_timeout(struct pvt_hwmon *pvt, long val)
{
	unsigned long rate;
	ktime_t kt, cache;
	u32 data;
	int ret;

	rate = clk_get_rate(pvt->clks[PVT_CLOCK_REF].clk);
	if (!rate)
		return -ENODEV;

	/*
	 * If alarms are enabled, the requested timeout must be divided
	 * between all available sensors to have the requested delay
	 * applicable to each individual sensor.
	 */
	cache = kt = ms_to_ktime(val);
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
	kt = ktime_divns(kt, PVT_SENSORS_NUM);
#endif

	data = pvt_calc_tout(kt, rate);

	/*
	 * Update the measurements delay, but lock the interface first, since
	 * we have to disable PVT in order to have the new delay actually
	 * updated.
	 */
	ret = mutex_lock_interruptible(&pvt->iface_mtx);
	if (ret)
		return ret;

	pvt_set_tout(pvt, data);
	pvt->timeout = cache;

	mutex_unlock(&pvt->iface_mtx);

	return 0;
}

static int pvt_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int ch, long *val)
{
	struct pvt_hwmon *pvt = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return pvt_read_timeout(pvt, val);
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return pvt_read_data(pvt, PVT_TEMP + ch, val);
		case hwmon_temp_type:
			*val = 1;
			return 0;
		case hwmon_temp_min:
			return pvt_read_limit(pvt, PVT_TEMP + ch, true, val);
		case hwmon_temp_max:
			return pvt_read_limit(pvt, PVT_TEMP + ch, false, val);
		case hwmon_temp_min_alarm:
			return pvt_read_alarm(pvt, PVT_TEMP + ch, true, val);
		case hwmon_temp_max_alarm:
			return pvt_read_alarm(pvt, PVT_TEMP + ch, false, val);
		case hwmon_temp_offset:
			return pvt_read_trim(pvt, val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return pvt_read_data(pvt, PVT_VOLT + ch, val);
		case hwmon_in_min:
			return pvt_read_limit(pvt, PVT_VOLT + ch, true, val);
		case hwmon_in_max:
			return pvt_read_limit(pvt, PVT_VOLT + ch, false, val);
		case hwmon_in_min_alarm:
			return pvt_read_alarm(pvt, PVT_VOLT + ch, true, val);
		case hwmon_in_max_alarm:
			return pvt_read_alarm(pvt, PVT_VOLT + ch, false, val);
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int pvt_hwmon_read_string(struct device *dev,
				 enum hwmon_sensor_types type,
				 u32 attr, int ch, const char **str)
{
	struct pvt_hwmon *pvt = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
			*str = pvt->info[PVT_TEMP + ch].label;
			return 0;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
			*str = pvt->info[PVT_VOLT + ch].label;
			return 0;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int pvt_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int ch, long val)
{
	struct pvt_hwmon *pvt = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return pvt_write_timeout(pvt, val);
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_min:
			return pvt_write_limit(pvt, PVT_TEMP + ch, true, val);
		case hwmon_temp_max:
			return pvt_write_limit(pvt, PVT_TEMP + ch, false, val);
		case hwmon_temp_offset:
			return pvt_write_trim(pvt, val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_min:
			return pvt_write_limit(pvt, PVT_VOLT + ch, true, val);
		case hwmon_in_max:
			return pvt_write_limit(pvt, PVT_VOLT + ch, false, val);
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops pvt_hwmon_ops = {
	.is_visible = pvt_hwmon_is_visible,
	.read = pvt_hwmon_read,
	.read_string = pvt_hwmon_read_string,
	.write = pvt_hwmon_write
};

static struct hwmon_chip_info pvt_hwmon_info = {
	.ops = &pvt_hwmon_ops,
};

static int pvt_thermal_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pvt_hwmon *pvt = thermal_zone_device_priv(tzd);
	long t;
	int err;

	err = pvt_read_data(pvt, PVT_TEMP, &t);
	if (err)
		return err;

	*temp = t;

	return 0;
}

static int pvt_thermal_set_trips(struct thermal_zone_device *tzd,
				 int low, int high)
{
	struct pvt_hwmon *pvt = thermal_zone_device_priv(tzd);
	int err;

	err = pvt_write_limit(pvt, PVT_TEMP, true, low);
	if (err)
		return err;
	return pvt_write_limit(pvt, PVT_TEMP, false, high);
}

static struct thermal_zone_device_ops pvt_thermal_ops = {
	.get_temp = pvt_thermal_get_temp,
	.set_trips = pvt_thermal_set_trips,
};

static const struct thermal_trip pvt_trips[] = {
	{
		.type = THERMAL_TRIP_CRITICAL,
		.temperature = PVT_THERMAL_CRIT,
		.hysteresis = PVT_THERMAL_CRIT_HYST,
	},
};

static void pvt_thermal_unregister(void *data)
{
	struct thermal_zone_device *tzd = data;

	thermal_zone_device_disable(tzd);
	thermal_zone_device_unregister(tzd);
}

static struct thermal_zone_device *pvt_thermal_register(struct pvt_hwmon *pvt)
{
	struct thermal_zone_device *tzd;
	struct thermal_zone_params *tzp;
	struct thermal_trip *trips;
	int err;

	trips = devm_kmemdup(pvt->dev, pvt_trips, sizeof(pvt_trips),
			     GFP_KERNEL);
	if (!trips)
		return ERR_PTR(-ENOMEM);

	tzp = devm_kzalloc(pvt->dev, sizeof(*tzp), GFP_KERNEL);
	if (!tzp) {
		devm_kfree(pvt->dev, trips);
		return ERR_PTR(-ENOMEM);
	}
	tzp->no_hwmon = true;
	tzp->slope = 1;
	tzp->offset = 0;

	tzd = thermal_zone_device_register_with_trips("pvt_thermal",
		trips, ARRAY_SIZE(pvt_trips), pvt, &pvt_thermal_ops,
		tzp, 0, PVT_THERMAL_POLLING_DELAY);
	if (IS_ERR(tzd)) {
		devm_kfree(pvt->dev, tzp);
		devm_kfree(pvt->dev, trips);
		return tzd;
	}
	err = thermal_zone_device_enable(tzd);
	if (err)
		goto out_unregister;
	err = devm_add_action(pvt->dev, pvt_thermal_unregister, tzd);
	if (err)
		goto out_disable;
	return tzd;

out_disable:
	thermal_zone_device_disable(tzd);
out_unregister:
	thermal_zone_device_unregister(tzd);
	devm_kfree(pvt->dev, tzp);
	devm_kfree(pvt->dev, trips);
	return ERR_PTR(err);
}

static void pvt_clear_data(void *data)
{
	struct pvt_hwmon *pvt = data;
#if !defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
	int idx;

	for (idx = 0; idx < PVT_SENSORS_NUM; ++idx)
		complete_all(&pvt->cache[idx].conversion);
#endif

	mutex_destroy(&pvt->iface_mtx);
}

static struct pvt_hwmon *pvt_create_data(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pvt_hwmon *pvt;
	int ret, idx;

	pvt = devm_kzalloc(dev, sizeof(*pvt), GFP_KERNEL);
	if (!pvt)
		return ERR_PTR(-ENOMEM);

	ret = devm_add_action(dev, pvt_clear_data, pvt);
	if (ret) {
		dev_err(dev, "Can't add PVT data clear action\n");
		return ERR_PTR(ret);
	}

	pvt->dev = dev;
	pvt->sensor = PVT_SENSOR_FIRST;
	mutex_init(&pvt->iface_mtx);

#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
	for (idx = 0; idx < PVT_SENSORS_NUM; ++idx)
		seqlock_init(&pvt->cache[idx].data_seqlock);
#else
	for (idx = 0; idx < PVT_SENSORS_NUM; ++idx)
		init_completion(&pvt->cache[idx].conversion);
#endif

	return pvt;
}

static int pvt_request_regs(struct pvt_hwmon *pvt)
{
	struct platform_device *pdev = to_platform_device(pvt->dev);

	if (!pvt->regs) {
		pvt->regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(pvt->regs))
			return PTR_ERR(pvt->regs);
	}

	return 0;
}

static void pvt_disable_clks(void *data)
{
	struct pvt_hwmon *pvt = data;

	clk_bulk_disable_unprepare(PVT_CLOCK_NUM, pvt->clks);
}

static int pvt_request_clks(struct pvt_hwmon *pvt)
{
	int ret;

	pvt->clks[PVT_CLOCK_APB].id = "pclk";
	pvt->clks[PVT_CLOCK_REF].id = "ref";

	ret = devm_clk_bulk_get_optional(pvt->dev, PVT_CLOCK_NUM, pvt->clks);
	if (ret) {
		dev_err(pvt->dev, "Couldn't get PVT clocks descriptors\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(PVT_CLOCK_NUM, pvt->clks);
	if (ret) {
		dev_err(pvt->dev, "Couldn't enable the PVT clocks\n");
		return ret;
	}

	ret = devm_add_action_or_reset(pvt->dev, pvt_disable_clks, pvt);
	if (ret) {
		dev_err(pvt->dev, "Can't add PVT clocks disable action\n");
		return ret;
	}

	return 0;
}

static int pvt_check_pwr(struct pvt_hwmon *pvt)
{
	unsigned long tout;
	int ret = 0;
	u32 data;

	/*
	 * Test out the sensor conversion functionality. If it is not done on
	 * time then the domain must have been unpowered and we won't be able
	 * to use the device later in this driver.
	 * Note If the power source is lost during the normal driver work the
	 * data read procedure will either return -ETIMEDOUT (for the
	 * alarm-less driver configuration) or just stop the repeated
	 * conversion. In the later case alas we won't be able to detect the
	 * problem.
	 */
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_ALL, PVT_INTR_ALL);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, PVT_CTRL_EN);
	pvt_set_tout(pvt, 0);
	pvt->ops->read(pvt, PVT_DATA);

	tout = PVT_TOUT_MIN / NSEC_PER_USEC;
	usleep_range(tout, 2 * tout);

	data = pvt->ops->read(pvt, PVT_DATA);
	if (!(data & PVT_DATA_VALID)) {
		ret = -ENODEV;
		dev_err(pvt->dev, "Sensor is powered down\n");
	}

	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);

	return ret;
}

static int pvt_init_iface(struct pvt_hwmon *pvt)
{
	unsigned long rate;
	u32 trim, temp, tout;

	rate = clk_get_rate(pvt->clks[PVT_CLOCK_REF].clk);
	if (!rate) {
		dev_err(pvt->dev, "Invalid reference clock rate\n");
		return -ENODEV;
	}

	/*
	 * Make sure all interrupts and controller are disabled so not to
	 * accidentally have ISR executed before the driver data is fully
	 * initialized. Clear the IRQ status as well.
	 */
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_ALL, PVT_INTR_ALL);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt->ops->read(pvt, PVT_CLR_INTR);
	pvt->ops->read(pvt, PVT_DATA);

	/*
	 * Preserve the current ref-clock based delay (Ttotal) between the
	 * sensors data samples in the driver data so not to recalculate it
	 * each time on the data requests and timeout reads. It consists of the
	 * delay introduced by the internal ref-clock timer (N / Fclk) and the
	 * constant timeout caused by each conversion latency (Tmin):
	 *   Ttotal = N / Fclk + Tmin
	 * If alarms are enabled the sensors are polled one after another and
	 * in order to get the next measurement of a particular sensor the
	 * caller will have to wait for at most until all the others are
	 * polled. In that case the formulae will look a bit different:
	 *   Ttotal = 5 * (N / Fclk + Tmin)
	 */
	pvt->timeout = ktime_set(0, PVT_TOUT_DEF);
	tout = pvt_calc_tout(
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
		ktime_divns(pvt->timeout, PVT_SENSORS_NUM)
#else
		pvt->timeout
#endif
		, rate);

	/* Setup default sensor mode, timeout and temperature trim. */
	pvt_set_mode(pvt, pvt->info[pvt->sensor].mode);
	pvt_set_tout(pvt, tout);

	trim = PVT_TRIM_DEF;
	if (!of_property_read_u32(pvt->dev->of_node,
	     "baikal,pvt-temp-offset-millicelsius", &temp))
		trim = pvt_calc_trim(temp);

	pvt_set_trim(pvt, trim);

	return 0;
}

static int pvt_create_hwmon(struct pvt_hwmon *pvt,
			    const struct hwmon_channel_info * const *hwmon_info)
{
	pvt_hwmon_info.info = hwmon_info;
	pvt->hwmon = devm_hwmon_device_register_with_info(pvt->dev, "pvt", pvt,
		&pvt_hwmon_info, NULL);
	if (IS_ERR(pvt->hwmon)) {
		dev_err(pvt->dev, "Couldn't create hwmon device\n");
		return PTR_ERR(pvt->hwmon);
	}

	if (__is_defined(CONFIG_THERMAL_OF) && acpi_disabled) {
		pvt->tzd = devm_thermal_of_zone_register(pvt->dev, 0, pvt,
							 &pvt_thermal_ops);
		if (IS_ERR(pvt->tzd) && PTR_ERR(pvt->tzd) == -ENODEV) {
			dev_info(pvt->dev,
				"temp0_input not attached to any thermal zone\n");
			return 0;
		}
	}
	else {
		pvt->tzd = pvt_thermal_register(pvt);
	}
	if (IS_ERR(pvt->tzd)) {
		dev_err(pvt->dev, "Couldn't register to thermal\n");
		return PTR_ERR(pvt->tzd);
	}

	return 0;
}

static int pvt_request_irq(struct pvt_hwmon *pvt)
{
	struct platform_device *pdev = to_platform_device(pvt->dev);
	int ret;

	pvt->irq = platform_get_irq(pdev, 0);
	if (pvt->irq < 0)
		return pvt->irq;

	ret = devm_request_threaded_irq(pvt->dev, pvt->irq,
					pvt_hard_isr,
					pvt_soft_isr,
#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)
					IRQF_SHARED |
					IRQF_TRIGGER_HIGH |
					IRQF_ONESHOT,
#else
					IRQF_SHARED |
					IRQF_TRIGGER_HIGH,
#endif
					"pvt", pvt);
	if (ret) {
		dev_err(pvt->dev, "Couldn't request PVT IRQ\n");
		return ret;
	}

	return 0;
}

#if defined(CONFIG_SENSORS_BAIKAL_PVT_ALARMS)

static void pvt_disable_iface(void *data)
{
	struct pvt_hwmon *pvt = data;

	mutex_lock(&pvt->iface_mtx);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, 0);
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, PVT_INTR_DVALID);
	mutex_unlock(&pvt->iface_mtx);
}

static int pvt_enable_iface(struct pvt_hwmon *pvt)
{
	int ret;

	ret = devm_add_action(pvt->dev, pvt_disable_iface, pvt);
	if (ret) {
		dev_err(pvt->dev, "Can't add PVT disable interface action\n");
		return ret;
	}

	/*
	 * Enable sensors data conversion and IRQ. We need to lock the
	 * interface mutex since hwmon has just been created and the
	 * corresponding sysfs files are accessible from user-space,
	 * which theoretically may cause races.
	 */
	mutex_lock(&pvt->iface_mtx);
	pvt_update(pvt, PVT_INTR_MASK, PVT_INTR_DVALID, 0);
	pvt_update(pvt, PVT_CTRL, PVT_CTRL_EN, PVT_CTRL_EN);
	mutex_unlock(&pvt->iface_mtx);

	return 0;
}

#else /* !CONFIG_SENSORS_BAIKAL_PVT_ALARMS */

static int pvt_enable_iface(struct pvt_hwmon *pvt)
{
	return 0;
}

#endif /* !CONFIG_SENSORS_BAIKAL_PVT_ALARMS */

int baikal_pvt_create(struct platform_device *pdev, struct pvt_ops *ops,
		      const struct pvt_sensor_info *info,
		      const struct hwmon_channel_info * const *hwmon_info)
{
	struct pvt_hwmon *pvt;
	int ret;

	pvt = pvt_create_data(pdev);
	if (IS_ERR(pvt))
		return PTR_ERR(pvt);

	pvt->ops = ops;
	pvt->info = info;
	platform_set_drvdata(pdev, pvt);

	if (ops->init) {
		ret = ops->init(pvt);
		if (ret)
			return ret;
	}

	ret = pvt_request_regs(pvt);
	if (ret)
		return ret;

	ret = pvt_request_clks(pvt);
	if (ret)
		return ret;

	ret = pvt_check_pwr(pvt);
	if (ret)
		return ret;

	ret = pvt_init_iface(pvt);
	if (ret)
		return ret;

	ret = pvt_create_hwmon(pvt, hwmon_info);
	if (ret)
		return ret;

	ret = pvt_request_irq(pvt);
	if (ret)
		return ret;

	ret = pvt_enable_iface(pvt);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(baikal_pvt_create);
