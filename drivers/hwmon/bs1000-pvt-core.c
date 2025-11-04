// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC Process, Voltage, Temperature sensor driver common code.
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/firmware/baikal/baikal-smc.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "bs1000-pvt.h"

#define PVT_PROCESSING_MODE_DEFAULT	3

static unsigned int pvt_processing_mode = CONFIG_SENSORS_BS1000_PVT_MODE;

static int param_set_processing_mode(const char *val, const struct kernel_param *kp)
{
	return param_set_uint_minmax(val, kp, 0, 4);
}

static const struct kernel_param_ops param_ops_processing_mode = {
	.set = param_set_processing_mode,
	.get = param_get_uint,
};

#define param_check_processing_mode(name, p) \
	__param_check(name, p, unsigned int);

module_param_named(mode, pvt_processing_mode, processing_mode, 0644);

#define BS1000_SCP_CMD_PVT_THR_SET	0x8
#define BS1000_SCP_CMD_PVT_THR_GET	0x9
#define BS1000_SCP_CMD_PVT_READ		0xA
#define BS1000_SCP_CMD_PVT_MODE_SET	0xC
#define BS1000_SCP_CMD_PVT_MODE_GET	0xD

#define BS1000_SCP_PVT_THRLO_MASK	GENMASK(15, 0)
#define BS1000_SCP_PVT_THRHI_MASK	GENMASK(31, 16)

#define BAIKAL_MBOX_SMS_PVT_DATA	GENMASK(15, 0)
#define BAIKAL_MBOX_SMS_PVT_ID		GENMASK(20, 16)
#define BAIKAL_MBOX_SMS_THR_MAX		BIT(21)		/* 0 - min, 1 - max */
#define BAIKAL_MBOX_SMS_THR_CRIT	BIT(22)		/* 0 - Yellow, 1 - Red */
#define BAIKAL_MBOX_SMS_PVT_TYPE	BIT(23)		/* 0 - Temperature, 1 - Voltage */

#define BS1000_PVT_DUMMY_ID	24

#define BS1000_PVT_TEMP_CHS	1
#define BS1000_PVT_VOLT_CHS	4

#define PVT_THR_MIN	0
#define PVT_THR_MAX	GENMASK(15, 0)

#define PVT_THERMAL_POLLING_DELAY	5000

#define PVT_THERMAL_CRIT	85000	/* 85 degrees default critical temperature limit */
#define PVT_THERMAL_CRIT_HYST	2000

enum pvt_mode {
	PVT_TEMP = 0,
	PVT_VOLT = 1,
	PVT_LVT  = 2,
	PVT_ULVT = 4,
	PVT_SVT  = 6
};

#define PVT_VALUE_ID_MASK	GENMASK(7, 0)
#define PVT_VALUE_MODE_MASK	GENMASK(11, 8)

struct pvt_info {
	unsigned int channel;
	const char *label;
	enum pvt_mode mode;
	long (*convert)(long value, bool to_pvt);
};

#define PVT_MASK_CPU	GENMASK(11, 0)
#define PVT_MASK_DDR	GENMASK(17, 12)
#define PVT_MASK_PCIE	GENMASK(22, 18)
#define PVT_MASK_SCP	BIT(23)

static const struct pvt_data {
	unsigned int start;
	unsigned int num;
	const char *format;
	u32 mask;
} pvt_data[] = {
	{ .num = 1,  .format = "pvt%1uscp",        .mask = PVT_MASK_SCP  },
	{ .num = 12, .format = "pvt%1ucluster%1u", .mask = PVT_MASK_CPU  },
	{ .num = 5,  .format = "pvt%1upcie%1u",    .mask = PVT_MASK_PCIE },
	{ .num = 6,  .format = "pvt%1uddr%1u",     .mask = PVT_MASK_DDR  },
	{ .num = 1,  .format = "pvt%1udummy",      .mask = 0             },
	{ .num = 1,  .format = "pvt%1umin",        .mask = 0             },
	{ .num = 1,  .format = "pvt%1umax",        .mask = 0             },
	{ .num = 1,  .format = "pvt%1uavg",        .mask = 0             }
};

static long pvt_temp_convert(long data, bool to_pvt)
{
	return to_pvt ? data / 10 : data * 10;
}

static const struct pvt_info pvt_temp_info[] = {
	{ .channel = 0, .label = "Temperature", .mode = PVT_TEMP, .convert = pvt_temp_convert }
};

static const struct pvt_info pvt_volt_info[] = {
	{ .channel = 0, .label = "Voltage",      .mode = PVT_VOLT, .convert = NULL },
	{ .channel = 1, .label = "Low-Vt",       .mode = PVT_LVT,  .convert = NULL },
	{ .channel = 2, .label = "Ultra-Low-Vt", .mode = PVT_ULVT, .convert = NULL },
	{ .channel = 3, .label = "Standard-Vt",  .mode = PVT_SVT,  .convert = NULL }
};

static int pvt_read_data(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			 long *val)
{
	struct pvt_dev *pvt = hwmon->pvt;
	struct baikal_scmi_base_ext_req req;
	struct baikal_scmi_base_ext_resp resp;
	int id = hwmon - pvt->hwmon;
	int err;

	req.service_id = SID_PVT;
	req.value_id = PVT_Value |
		       FIELD_PREP(PVT_VALUE_MODE_MASK, info->mode) |
		       FIELD_PREP(PVT_VALUE_ID_MASK, id);
	err = pvt->ops->soc_status_get(pvt->ph, &req, &resp);
	if (err || resp.count != 2)
		return err ? err : -EINVAL;

	*val = *(u16 *)resp.value;
	if (info->convert)
		*val = info->convert(*val, false);

	return 0;
}

static int pvt_read_limit(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			  bool is_low, long *val)
{
	struct pvt_dev *pvt = hwmon->pvt;
	struct arm_smccc_res res;
	u32 data;
	int id;

	id = (hwmon - pvt->hwmon) + pvt->numa_node * BS1000_PVT_PER_CHIP;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_GET,
		      id, info->mode << 1, 0, 0, 0, 0, &res);
	if ((long)(res.a0) < 0)
		return res.a0;

	data = res.a0;
	if (is_low)
		*val = FIELD_GET(BS1000_SCP_PVT_THRLO_MASK, data);
	else
		*val = FIELD_GET(BS1000_SCP_PVT_THRHI_MASK, data);
	if (info->convert)
		*val = info->convert(*val, false);

	return 0;
}

static int pvt_write_limit(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			   bool is_low, long val)
{
	struct pvt_dev *pvt = hwmon->pvt;
	struct arm_smccc_res res;
	int id;
	u32 data, limit;
	int err;

	id = (hwmon - pvt->hwmon) + pvt->numa_node * BS1000_PVT_PER_CHIP;

	if (info->convert)
		val = info->convert(val, true);
	val = clamp_val(val, PVT_THR_MIN, PVT_THR_MAX);

	err = mutex_lock_interruptible(&hwmon->mutex);
	if (err)
		return err;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_GET,
		      id, info->mode << 1, 0, 0, 0, 0, &res);
	if ((long)(res.a0) < 0) {
		err = res.a0;
		goto out;
	}

	data = res.a0;
	/* Make sure the upper and lower ranges don't intersect. */
	if (is_low) {
		if (val) {
			limit = FIELD_GET(BS1000_SCP_PVT_THRHI_MASK, data);
			if (limit)
				val = clamp_val(val, PVT_THR_MIN, limit);
		}
		data &= ~BS1000_SCP_PVT_THRLO_MASK;
		data |= FIELD_PREP(BS1000_SCP_PVT_THRLO_MASK, val);
	}
	else {
		if (val) {
			limit = FIELD_GET(BS1000_SCP_PVT_THRLO_MASK, data);
			if (limit)
				val = clamp_val(val, limit, PVT_THR_MAX);
		}
		data &= ~BS1000_SCP_PVT_THRHI_MASK;
		data |= FIELD_PREP(BS1000_SCP_PVT_THRHI_MASK, val);
	}
	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_SET,
		      id, info->mode << 1, data, 0, 0, 0, &res);
	err = res.a0;

out:
	mutex_unlock(&hwmon->mutex);
	return err;
}

static int pvt_read_alarm(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			  bool is_low, long *val)
{
	long limit, data;
	int err;

	err = pvt_read_limit(hwmon, info, is_low, &limit);
	if (err)
		return err;

	if (limit == 0) {
		*val = 0;
		return 0;
	}

	err = pvt_read_data(hwmon, info, &data);
	if (err)
		return err;

	if (is_low)
		*val = (data < limit);
	else
		*val = (data > limit);

	return 0;
}

static inline bool pvt_hwmon_channel_is_valid(enum hwmon_sensor_types type,
					      int ch)
{
	switch (type) {
	case hwmon_temp:
		if (ch < 0 || ch >= BS1000_PVT_TEMP_CHS)
			return false;
		break;
	case hwmon_in:
		if (ch < 0 || ch >= BS1000_PVT_VOLT_CHS)
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
	const struct pvt_dev *pvt = ((const struct pvt_hwmon *)data)->pvt;

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return 0;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_type:
		case hwmon_temp_label:
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
			return 0444;
		case hwmon_temp_min:
		case hwmon_temp_max:
			return 0644;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			if ((pvt->processing_mode > 1 && ch == 0) ||
			    pvt->processing_mode >= 3)
				return 0444;
			break;
		case hwmon_in_min:
		case hwmon_in_max:
			if (pvt->processing_mode > 1 && ch == 0)
				return 0644;
			break;
		case hwmon_in_min_alarm:
		case hwmon_in_max_alarm:
			if (pvt->processing_mode > 1 && ch == 0)
				return 0444;
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int pvt_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int ch, long *val)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return pvt_read_data(hwmon, &pvt_temp_info[ch], val);
		case hwmon_temp_type:
			*val = 1;
			return 0;
		case hwmon_temp_min:
			return pvt_read_limit(hwmon, &pvt_temp_info[ch], true,
					      val);
		case hwmon_temp_max:
			return pvt_read_limit(hwmon, &pvt_temp_info[ch], false,
					      val);
		case hwmon_temp_min_alarm:
			return pvt_read_alarm(hwmon, &pvt_temp_info[ch], true,
					      val);
		case hwmon_temp_max_alarm:
			return pvt_read_alarm(hwmon, &pvt_temp_info[ch], false,
					      val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return pvt_read_data(hwmon, &pvt_volt_info[ch], val);
		case hwmon_in_min:
			return pvt_read_limit(hwmon, &pvt_volt_info[ch], true,
					      val);
		case hwmon_in_max:
			return pvt_read_limit(hwmon, &pvt_volt_info[ch], false,
					      val);
		case hwmon_in_min_alarm:
			return pvt_read_alarm(hwmon, &pvt_volt_info[ch], true,
					      val);
		case hwmon_in_max_alarm:
			return pvt_read_alarm(hwmon, &pvt_volt_info[ch], false,
					      val);
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
	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
			*str = pvt_temp_info[ch].label;
			return 0;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
			*str = pvt_volt_info[ch].label;
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
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_min:
			return pvt_write_limit(hwmon, &pvt_temp_info[ch], true,
					       val);
		case hwmon_temp_max:
			return pvt_write_limit(hwmon, &pvt_temp_info[ch], false,
					       val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_min:
			return pvt_write_limit(hwmon, &pvt_volt_info[ch], true,
					       val);
		case hwmon_in_max:
			return pvt_write_limit(hwmon, &pvt_volt_info[ch], false,
					       val);
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

static const struct hwmon_channel_info * const pvt_channel_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_TYPE | HWMON_T_LABEL |
			   HWMON_T_MIN | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL |
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info pvt_hwmon_info = {
	.ops = &pvt_hwmon_ops,
	.info = pvt_channel_info
};

static const struct hwmon_channel_info * const pvt_misc_channel_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info pvt_hwmon_misc_info = {
	.ops = &pvt_hwmon_ops,
	.info = pvt_misc_channel_info
};

static int pvt_thermal_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pvt_hwmon *hwmon = thermal_zone_device_priv(tzd);
	long t;
	int err;

	err = pvt_read_data(hwmon, &pvt_temp_info[0], &t);
	if (err)
		return err;

	*temp = t;

	return 0;
}

static int pvt_thermal_set_trips(struct thermal_zone_device *tzd,
				 int low, int high)
{
	struct pvt_hwmon *hwmon = thermal_zone_device_priv(tzd);
	int err;

	err = pvt_write_limit(hwmon, &pvt_temp_info[0], true, low);
	if (err)
		return err;
	return pvt_write_limit(hwmon, &pvt_temp_info[0], false, high);
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

static struct thermal_zone_device *pvt_thermal_register(struct pvt_hwmon *hwmon)
{
	struct thermal_zone_device *tzd;
	struct thermal_zone_params *tzp;
	struct thermal_trip *trips;
	int err;

	trips = devm_kmemdup(hwmon->dev, pvt_trips, sizeof(pvt_trips),
			     GFP_KERNEL);
	if (!trips)
		return ERR_PTR(-ENOMEM);

	tzp = devm_kzalloc(hwmon->dev, sizeof(*tzp), GFP_KERNEL);
	if (!tzp) {
		devm_kfree(hwmon->dev, trips);
		return ERR_PTR(-ENOMEM);
	}
	tzp->no_hwmon = true;
	tzp->slope = 1;
	tzp->offset = 0;

	tzd = thermal_zone_device_register_with_trips(hwmon->name,
		trips, ARRAY_SIZE(pvt_trips), hwmon, &pvt_thermal_ops,
		tzp, 0, PVT_THERMAL_POLLING_DELAY);
	if (IS_ERR(tzd)) {
		devm_kfree(hwmon->dev, tzp);
		devm_kfree(hwmon->dev, trips);
		return tzd;
	}
	err = thermal_zone_device_enable(tzd);
	if (err)
		goto out_unregister;
	err = devm_add_action(hwmon->dev, pvt_thermal_unregister, tzd);
	if (err)
		goto out_disable;
	return tzd;

out_disable:
	thermal_zone_device_disable(tzd);
out_unregister:
	thermal_zone_device_unregister(tzd);
	devm_kfree(hwmon->dev, tzp);
	devm_kfree(hwmon->dev, trips);
	return ERR_PTR(err);
}

static void pvt_irq_handler(struct mbox_client *client, void *msg)
{
	struct pvt_dev *pvt = dev_get_drvdata(client->dev);
	u32 reg = *(u32 *)msg;
	unsigned int id = FIELD_GET(BAIKAL_MBOX_SMS_PVT_ID, reg);

	if (id < BS1000_PVT_PER_CHIP) {
		struct pvt_hwmon *hwmon = &pvt->hwmon[id];

		if (!hwmon->enabled)
			return;

		if (reg & BAIKAL_MBOX_SMS_PVT_TYPE) {
			if (reg & BAIKAL_MBOX_SMS_THR_CRIT) {
				if (reg & BAIKAL_MBOX_SMS_THR_MAX)
					hwmon_notify_event(hwmon->dev, hwmon_in,
							   hwmon_in_crit_alarm,
							   0);
				else
					hwmon_notify_event(hwmon->dev, hwmon_in,
							   hwmon_in_lcrit_alarm,
							   0);
			}
			else {
				if (reg & BAIKAL_MBOX_SMS_THR_MAX)
					hwmon_notify_event(hwmon->dev, hwmon_in,
						   hwmon_in_max_alarm, 0);
				else
					hwmon_notify_event(hwmon->dev, hwmon_in,
						   hwmon_in_min_alarm, 0);
			}
		}
		else {
			if (reg & BAIKAL_MBOX_SMS_THR_CRIT) {
				if (reg & BAIKAL_MBOX_SMS_THR_MAX)
					hwmon_notify_event(hwmon->dev, hwmon_temp,
							   hwmon_temp_crit_alarm,
							   0);
				else
					hwmon_notify_event(hwmon->dev, hwmon_temp,
							   hwmon_temp_lcrit_alarm,
							   0);
			}
			else {
				if (reg & BAIKAL_MBOX_SMS_THR_MAX)
					hwmon_notify_event(hwmon->dev, hwmon_temp,
							   hwmon_temp_max_alarm,
							   0);
				else
					hwmon_notify_event(hwmon->dev, hwmon_temp,
							   hwmon_temp_min_alarm,
							   0);
			}
			thermal_zone_device_update(hwmon->tzd,
						   THERMAL_EVENT_UNSPECIFIED);
		}
	}
}

extern struct mbox_chan *baikal_mbox_sms_request_channel(struct mbox_client *cl);

static void bs1000_pvt_hwnon_register(struct pvt_dev *pvt, int id, int index,
				      const struct pvt_data *data)
{
	struct pvt_hwmon *hwmon;
	const struct hwmon_chip_info *info;

	hwmon = &pvt->hwmon[id];
	info = (id >= BS1000_PVT_DUMMY_ID) ? &pvt_hwmon_misc_info :
					     &pvt_hwmon_info;
	devm_mutex_init(pvt->dev, &hwmon->mutex);
	hwmon->pvt = pvt;
	snprintf(hwmon->name, sizeof(hwmon->name), data->format,
		 pvt->numa_node, index);
	hwmon->dev = devm_hwmon_device_register_with_info(pvt->dev,
		hwmon->name, hwmon, info, NULL);
	if (IS_ERR(hwmon->dev)) {
		dev_warn(pvt->dev, "Couldn't create %s hwmon device (%ld)\n",
			 hwmon->name, PTR_ERR(hwmon->dev));
		return;
	}
	hwmon->enabled = true;

	if (id < BS1000_PVT_DUMMY_ID) {
		if (__is_defined(CONFIG_THERMAL_OF) && acpi_disabled) {
			device_set_node(hwmon->dev, pvt->dev->fwnode);
			hwmon->tzd = devm_thermal_of_zone_register(
				hwmon->dev, id, hwmon, &pvt_thermal_ops);
		}
		else {
			hwmon->tzd = pvt_thermal_register(hwmon);
		}
		if (IS_ERR(hwmon->tzd)) {
			dev_warn(hwmon->dev,
				 "Couldn't register to thermal (%ld)\n",
				 PTR_ERR(hwmon->tzd));
		}
	}
}

int bs1000_pvt_create(struct device *dev,
		      const struct baikal_scmi_base_ext_proto_ops *ops,
		      void *ph)
{
	struct pvt_dev *pvt;
	struct mbox_client *client;
	struct arm_smccc_res res;
	struct baikal_scmi_base_ext_req req;
	struct baikal_scmi_base_ext_resp resp;
	u32 pvt_mask = 0;
	int i, id, err;

	pvt = devm_kzalloc(dev, sizeof(*pvt), GFP_KERNEL);
	if (!pvt)
		return -ENOMEM;
	pvt->dev = dev;
	pvt->ops = ops;
	pvt->ph = ph;
	pvt->numa_node = pvt->ops->node_id(pvt->ph);
	dev_set_drvdata(dev, pvt);

	client = devm_kzalloc(dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->dev = dev;
	client->rx_callback = pvt_irq_handler;
	pvt->mbox = baikal_mbox_sms_request_channel(client);
	if (IS_ERR(pvt->mbox)) {
		err = PTR_ERR(pvt->mbox);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Couldn't get mailbox channel (%d)\n", err);
		return err;
	}

	if (pvt_processing_mode) {
		/* set requested processing mode */
		arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_MODE_SET,
			      pvt->numa_node * BS1000_PVT_PER_CHIP, 0,
			      pvt_processing_mode, 0, 0, 0, &res);
		if ((long)(res.a0) < 0) {
			dev_warn(dev,
				 "Couldn't set PVT processing mode %d (%ld) assume default\n",
				 pvt_processing_mode, (long)res.a0);
			pvt->processing_mode = PVT_PROCESSING_MODE_DEFAULT;
		}
		else
			pvt->processing_mode = pvt_processing_mode;
	}
	else {
		/* get current processing mode */
		arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_MODE_GET,
			      pvt->numa_node * BS1000_PVT_PER_CHIP, 0, 0, 0, 0,
			      0, &res);
		if ((long)(res.a0) < 0) {
			dev_warn(dev,
				 "Couldn't get PVT processing mode (%ld) assume default\n",
				 (long)res.a0);
			pvt->processing_mode = PVT_PROCESSING_MODE_DEFAULT;
		}
		else
			pvt->processing_mode = res.a0;
	}

	req.service_id = SID_PVT;
	req.value_id = PVT_ActiveMask;
	err = pvt->ops->soc_status_get(pvt->ph, &req, &resp);
	if (err || resp.count != 4) {
		dev_err(dev, "Couldn't get PVT active mask (%d)\n",
			err ? err : resp.count);
		mbox_free_channel(pvt->mbox);
		return err;
	}
	pvt_mask = resp.value[0];

	for (i = 0, id = 0; i < ARRAY_SIZE(pvt_data); i++) {
		if (!pvt_data[i].mask || (pvt_mask & pvt_data[i].mask)) {
			int j, shift = ffs(pvt_data[i].mask) - 1;

			for (j = 0; j < pvt_data[i].num; j++, id++) {
				if (id == BS1000_PVT_DUMMY_ID)
					continue;
				if (!pvt_data[i].mask ||
				    (pvt_mask & (1 << (shift + j))))
					bs1000_pvt_hwnon_register(pvt, id, j, &pvt_data[i]);
			}
		}
		else
			id += pvt_data[i].num;
	}

	return 0;
}
EXPORT_SYMBOL(bs1000_pvt_create);

void bs1000_pvt_destroy(struct device *dev)
{
	struct pvt_dev *pvt = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);

	mbox_free_channel(pvt->mbox);
}
EXPORT_SYMBOL(bs1000_pvt_destroy);
