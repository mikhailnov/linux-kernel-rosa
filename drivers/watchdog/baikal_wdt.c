// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics watchdog driver
 *
 * Copyright (C) 2024-2025 Baikal Electronics, JSC
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware/baikal/baikal-smc.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/limits.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define BAIKAL_WDT_CMD_SET_PERIOD	1
#define BAIKAL_WDT_CMD_START		2
#define BAIKAL_WDT_CMD_STOP		3

#define BAIKAL_WDT_RESET_MODE_IRQ	BIT(0)

#define BAIKAL_WDT_MIN_TIMEOUT		1
#define BAIKAL_WDT_MAX_TIMEOUT		10
#define BAIKAL_WDT_HEARTBEAT_MS		100

enum baikal_sms_cmd_id_reset {
	RST_WdtPing   = 0x1,
	RST_OsStopped = 0x2,
	RST_OsStarted = 0x3,
};

enum baikal_sms_notify_id_reset {
	RST_WdtEvent      = 0x1,
	RST_StopOsRequest = 0x2,
};

static u32 ping_msg = RST_WdtPing;
static u32 sys_stop_msg = RST_OsStopped;
static u32 sys_start_msg = RST_OsStarted;

struct baikal_wdt {
	struct watchdog_device	wdd;
	struct mbox_chan	*mbox;
	u8			cfg;
	struct input_dev	*sys_stop;
	struct notifier_block	reboot_nb;
};

static void baikal_wdt_cmd(int cmd, u32 arg)
{
	switch (cmd) {
	case BAIKAL_WDT_CMD_SET_PERIOD:
	case BAIKAL_WDT_CMD_START:
	case BAIKAL_WDT_CMD_STOP:
		{
			struct arm_smccc_res res;

			arm_smccc_smc(BAIKAL_SMC_WDT, cmd, arg, 0, 0, 0, 0, 0, &res);
		}
	}
}

static int baikal_wdt_ping(struct watchdog_device *wdd)
{
	struct baikal_wdt *baikal_wdt = watchdog_get_drvdata(wdd);

	return mbox_send_message(baikal_wdt->mbox, &ping_msg);
}

static int baikal_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	struct baikal_wdt *baikal_wdt = watchdog_get_drvdata(wdd);

	if (timeout > BAIKAL_WDT_MAX_TIMEOUT)
		timeout = BAIKAL_WDT_MAX_TIMEOUT;

	wdd->timeout = timeout;
	timeout = DIV_ROUND_UP(timeout, 2);

	if (baikal_wdt->cfg & BAIKAL_WDT_RESET_MODE_IRQ) {
		/* Use half of the timeout if IRQ mode is used */
		wdd->pretimeout = timeout;
	} else {
		wdd->pretimeout = 0;
	}

	baikal_wdt_cmd(BAIKAL_WDT_CMD_SET_PERIOD, timeout * MSEC_PER_SEC);

	if (watchdog_active(wdd))
		baikal_wdt_ping(wdd);

	return 0;
}

static int baikal_wdt_set_pretimeout(struct watchdog_device *wdd, unsigned int timeout)
{
	struct baikal_wdt *baikal_wdt = watchdog_get_drvdata(wdd);

	if (timeout)
		baikal_wdt->cfg |= BAIKAL_WDT_RESET_MODE_IRQ;
	else
		baikal_wdt->cfg &= ~BAIKAL_WDT_RESET_MODE_IRQ;
	baikal_wdt_set_timeout(wdd, wdd->timeout);

	return 0;
}

static int baikal_wdt_start(struct watchdog_device *wdd)
{
	baikal_wdt_cmd(BAIKAL_WDT_CMD_START, 0);

	return 0;
}

static int baikal_wdt_stop(struct watchdog_device *wdd)
{
	baikal_wdt_cmd(BAIKAL_WDT_CMD_STOP, 0);

	return 0;
}

static int baikal_wdt_restart(struct watchdog_device *wdd,
			      unsigned long action, void *data)
{
	baikal_wdt_cmd(BAIKAL_WDT_CMD_STOP, 0);
	baikal_wdt_cmd(BAIKAL_WDT_CMD_SET_PERIOD, 100);
	baikal_wdt_cmd(BAIKAL_WDT_CMD_START, 0);

	mdelay(500);

	return 0;
}

static const struct watchdog_info baikal_wdt_info = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT | WDIOF_MAGICCLOSE,
	.identity	= "Baikal Electronics Watchdog",
};

static const struct watchdog_ops baikal_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= baikal_wdt_start,
	.stop		= baikal_wdt_stop,
	.ping		= baikal_wdt_ping,
	.set_timeout	= baikal_wdt_set_timeout,
	.set_pretimeout	= baikal_wdt_set_pretimeout,
	.restart	= baikal_wdt_restart,
};

static void baikal_wdt_irq(struct mbox_client *client, void *msg)
{
	struct baikal_wdt *baikal_wdt = dev_get_drvdata(client->dev);
	u32 id = *(u32 *)msg;

	switch (id) {
	case RST_WdtEvent:
		if (baikal_wdt->cfg & BAIKAL_WDT_RESET_MODE_IRQ)
			watchdog_notify_pretimeout(&baikal_wdt->wdd);
		break;
	case RST_StopOsRequest:
		dev_info(client->dev, "OS Stop Request...\n");
		input_report_key(baikal_wdt->sys_stop, KEY_POWER, 1);
		input_sync(baikal_wdt->sys_stop);
		input_report_key(baikal_wdt->sys_stop, KEY_POWER, 0);
		input_sync(baikal_wdt->sys_stop);
		break;
	}
}

static int baikal_wdt_reboot_notifier(struct notifier_block *nb,
				      unsigned long reason, void *__unused)
{
	struct baikal_wdt *baikal_wdt = container_of(nb, struct baikal_wdt, reboot_nb);

	if (reason == SYS_HALT || reason == SYS_POWER_OFF || reason == SYS_RESTART) {
		mbox_send_message(baikal_wdt->mbox, &sys_stop_msg);
	}

	return NOTIFY_OK;
}

static void baikal_wdt_drv_remove(struct platform_device *pdev)
{
	struct baikal_wdt *baikal_wdt = platform_get_drvdata(pdev);

	input_unregister_device(baikal_wdt->sys_stop);
	unregister_reboot_notifier(&baikal_wdt->reboot_nb);
	mbox_free_channel(baikal_wdt->mbox);
}

static int baikal_wdt_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct baikal_wdt *baikal_wdt;
	struct input_dev *sys_stop;
	struct mbox_client *client;
	int ret;

	baikal_wdt = devm_kzalloc(dev, sizeof(*baikal_wdt), GFP_KERNEL);
	if (!baikal_wdt)
		return -ENOMEM;

	wdd = &baikal_wdt->wdd;
	wdd->info = &baikal_wdt_info;
	wdd->ops = &baikal_wdt_ops;
	wdd->min_hw_heartbeat_ms = BAIKAL_WDT_HEARTBEAT_MS;
	wdd->max_hw_heartbeat_ms = BAIKAL_WDT_MAX_TIMEOUT * MSEC_PER_SEC;
	wdd->min_timeout = BAIKAL_WDT_MIN_TIMEOUT;
	wdd->timeout = BAIKAL_WDT_MAX_TIMEOUT;
	wdd->parent = dev;

	watchdog_init_timeout(wdd, 0, dev);
	watchdog_set_nowayout(wdd, nowayout);
	watchdog_set_restart_priority(wdd, 128);
	watchdog_stop_on_reboot(wdd);
	watchdog_stop_on_unregister(wdd);
	watchdog_set_drvdata(wdd, baikal_wdt);

	platform_set_drvdata(pdev, baikal_wdt);

	client = devm_kzalloc(dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	client->dev = dev;
	client->rx_callback = baikal_wdt_irq;
	baikal_wdt->mbox = mbox_request_channel_by_fwnode(client, 0);
	if (IS_ERR(baikal_wdt->mbox))
		return PTR_ERR(baikal_wdt->mbox);

	mbox_send_message(baikal_wdt->mbox, &sys_start_msg);

	baikal_wdt->reboot_nb.notifier_call = &baikal_wdt_reboot_notifier;
	baikal_wdt->reboot_nb.priority = INT_MIN;
	ret = register_reboot_notifier(&baikal_wdt->reboot_nb);
	if (ret) {
		mbox_free_channel(baikal_wdt->mbox);
		return ret;
	}

	sys_stop = input_allocate_device();
	if (!sys_stop)
		return -ENOMEM;
	sys_stop->name = "OS Stop Request";
	sys_stop->phys = "pm/button/input0";
	sys_stop->id.bustype = BUS_HOST;
	sys_stop->dev.parent = NULL;
	input_set_capability(sys_stop, EV_KEY, KEY_POWER);
	ret = input_register_device(sys_stop);
	if (ret) {
		input_free_device(sys_stop);
		mbox_free_channel(baikal_wdt->mbox);
		return ret;
	}
	baikal_wdt->sys_stop = sys_stop;

	baikal_wdt_set_timeout(wdd, wdd->timeout);

	ret = devm_watchdog_register_device(dev, wdd);
	if (ret)
		baikal_wdt_drv_remove(pdev);

	return ret;
}

static const struct of_device_id baikal_wdt_of_match[] = {
	{ .compatible = "baikal,wdt", },
	{}
};
MODULE_DEVICE_TABLE(of, baikal_wdt_of_match);

static struct platform_driver baikal_wdt_driver = {
	.probe		= baikal_wdt_drv_probe,
	.remove		= baikal_wdt_drv_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_match_ptr(baikal_wdt_of_match),
	},
};
module_platform_driver(baikal_wdt_driver);

MODULE_DESCRIPTION("Baikal Electronics Watchdog Driver");
MODULE_LICENSE("GPL");
