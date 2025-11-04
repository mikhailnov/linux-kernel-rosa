// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2023 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

/**
 * baikal_vdu_backlight.c
 * Implementation of backlight functions for
 * Baikal Electronics BE-M1000 SoC's VDU
 */

#include <linux/clk.h>
#include <linux/input.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define BAIKAL_VDU_MIN_BRIGHTNESS	0
#define BAIKAL_VDU_DEFAULT_BRIGHTNESS	50
#define BAIKAL_VDU_BRIGHTNESS_STEP	5
#define BAIKAL_VDU_DEFAULT_PWM_FREQ	10000

static int baikal_vdu_backlight_update_status(struct backlight_device *bl_dev)
{
	struct baikal_vdu_private *priv = bl_get_data(bl_dev);
	int brightness_on = 1;
	int brightness = bl_dev->props.brightness;
	u8 pwmdc;

	if (backlight_is_blank(bl_dev)) {
		brightness_on = 0;
		brightness = priv->min_brightness;
	}

	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, brightness_on);

	pwmdc = brightness ? ((brightness << 6) / 25 - 1) : 0;

	writel(pwmdc, priv->regs + PWMDCR);

	return 0;
}

static const struct backlight_ops baikal_vdu_backlight_ops = {
	.options        = BL_CORE_SUSPENDRESUME,
	.update_status	= baikal_vdu_backlight_update_status,
};

static void baikal_vdu_input_event(struct input_handle *handle,
				   unsigned int type, unsigned int code,
				   int value)
{
	struct baikal_vdu_private *priv = handle->private;
	int brightness;

	if (type != EV_KEY || value == 0)
		return;

	switch (code) {
	case KEY_BRIGHTNESSDOWN:
		brightness = priv->bl_dev->props.brightness -
			     priv->brightness_step;
		if (brightness >= priv->min_brightness)
			backlight_device_set_brightness(priv->bl_dev,
							brightness);
		break;

	case KEY_BRIGHTNESSUP:
		brightness = priv->bl_dev->props.brightness +
			     priv->brightness_step;
		backlight_device_set_brightness(priv->bl_dev, brightness);
		break;

	case KEY_BRIGHTNESS_TOGGLE:
		priv->brightness_on = !priv->brightness_on;
		if (priv->brightness_on)
			backlight_enable(priv->bl_dev);
		else
			backlight_disable(priv->bl_dev);
		break;

	default:
		return;
	}

	backlight_force_update(priv->bl_dev, BACKLIGHT_UPDATE_HOTKEY);
}

static int baikal_vdu_input_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->private = handler->private;
	handle->name = KBUILD_MODNAME;
	handle->dev = dev;
	handle->handler = handler;

	ret = input_register_handle(handle);
	if (ret)
		goto err_free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister_handle;

	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return ret;
}

static void baikal_vdu_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id baikal_vdu_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},

	{ },    /* Terminating entry */
};

MODULE_DEVICE_TABLE(input, baikal_vdu_input_ids);

int baikal_vdu_backlight_create(struct drm_device *drm)
{
	struct baikal_vdu_crossbar *crossbar = drm_to_baikal_vdu_crossbar(drm);
	struct baikal_vdu_private *priv = &crossbar->lvds;
	struct device *dev = drm->dev;
	struct backlight_properties props;
	struct input_handler *handler;
	struct fwnode_handle *node;
	u32 min_brightness = BAIKAL_VDU_MIN_BRIGHTNESS;
	u32 dfl_brightness = BAIKAL_VDU_DEFAULT_BRIGHTNESS;
	u32 brightness_step = BAIKAL_VDU_BRIGHTNESS_STEP;
	u32 pwm_frequency = 0;
	int ret = 0;
	unsigned long rate;
	unsigned int pwmfr = 0;

	priv->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(priv->enable_gpio)) {
		dev_warn(dev, "failed to get ENABLE GPIO\n");
		priv->enable_gpio = NULL;
	}

	if (priv->enable_gpio && gpiod_get_direction(priv->enable_gpio) != 0)
		gpiod_direction_output(priv->enable_gpio, 1);

	node = fwnode_get_named_child_node(dev->fwnode, is_of_node(dev->fwnode) ?
					   "backlight" : "BCKL");
	if (!node)
		return 0;

	fwnode_property_read_u32(node, "min-brightness-level", &min_brightness);
	fwnode_property_read_u32(node, "default-brightness-level", &dfl_brightness);
	fwnode_property_read_u32(node, "brightness-level-step", &brightness_step);
	fwnode_property_read_u32(node, "pwm-frequency", &pwm_frequency);

	if (pwm_frequency == 0) {
		dev_warn(dev, "using default PWM frequency %u\n",
			 BAIKAL_VDU_DEFAULT_PWM_FREQ);
		pwm_frequency = BAIKAL_VDU_DEFAULT_PWM_FREQ;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = 100;
	props.type = BACKLIGHT_RAW;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	if (min_brightness > props.max_brightness) {
		dev_warn(dev, "invalid min brightness level: %u, using %u\n",
			 min_brightness, props.max_brightness);
		min_brightness = props.max_brightness;
	}

	if (dfl_brightness > props.max_brightness ||
	    dfl_brightness < min_brightness) {
		dev_warn(dev,
			 "invalid default brightness level: %u, using %u\n",
			 dfl_brightness, props.max_brightness);
		dfl_brightness = props.max_brightness;
	}

	priv->min_brightness = min_brightness;
	priv->brightness_step = brightness_step;
	priv->brightness_on = true;

	props.brightness = dfl_brightness;
	props.power = FB_BLANK_UNBLANK;

	priv->bl_dev =
		devm_backlight_device_register(dev, dev_name(dev), dev, priv,
					       &baikal_vdu_backlight_ops,
					       &props);
	if (IS_ERR(priv->bl_dev)) {
		dev_err(dev, "failed to register backlight device\n");
		ret = PTR_ERR(priv->bl_dev);
		priv->bl_dev = NULL;
		goto out;
	}

	handler = devm_kzalloc(dev, sizeof(*handler), GFP_KERNEL);
	if (!handler) {
		dev_err(dev, "failed to allocate input handler\n");
		ret = -ENOMEM;
		goto out;
	}

	handler->private = priv;
	handler->event = baikal_vdu_input_event;
	handler->connect = baikal_vdu_input_connect;
	handler->disconnect = baikal_vdu_input_disconnect;
	handler->name = KBUILD_MODNAME;
	handler->id_table = baikal_vdu_input_ids;

	ret = input_register_handler(handler);
	if (ret) {
		dev_err(dev, "failed to register input handler\n");
		goto out;
	}

	/* Hold PWM Clock Domain Reset, disable clocking */
	writel(0, priv->regs + PWMFR);

	rate = baikal_vdu_crtc_get_rate(priv);
	pwmfr |= PWMFR_PWMFCD(rate / pwm_frequency - 1) | PWMFR_PWMFCI;
	writel(pwmfr, priv->regs + PWMFR);

	/* Release PWM Clock Domain Reset, enable clocking */
	writel(pwmfr | PWMFR_PWMPCR | PWMFR_PWMFCE, priv->regs + PWMFR);

	backlight_update_status(priv->bl_dev);
out:
	fwnode_handle_put(node);
	return ret;
}
