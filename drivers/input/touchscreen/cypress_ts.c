/*
 *  Cypress Touch Screen Driver
 *
 *  Copyright (c) 2008 CYPRESS
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
 *
 *  Based on touchscreen code from Cypress Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <mach/board.h>
#include <linux/cypress_ts.h>

static struct workqueue_struct *cypress_wq;

struct cypress_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int (*hw_init)(int on);
	int (*power)(int ch);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int prev_points;
	uint abs_x_max;
	uint abs_pressure_max;
	uint points_max;	/* max support points */
	uint y_max;		/* max y, include virtual key */
};

struct _pos {
	uint x;
	uint y;
	uint z;
};

static struct _pos pos[4];

static ssize_t cypress_fw_version_show(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct cypress_ts_data *ts = dev_get_drvdata(device);
	uint8_t wdata[1] = {0};
	uint8_t rdata[4] = {0};

	/* 0x1C, 0x1D, 0x1F: FW version */
	wdata[0] = 0x1C;
	if (1 != i2c_master_send(ts->client, wdata, 1))
		goto i2c_err;

	if (4 != i2c_master_recv(ts->client, rdata, 4))
		goto i2c_err;

	return sprintf(buf, "ver: %x.%x.%x\n", rdata[0], rdata[1], rdata[3]);

i2c_err:
	pr_err("%s: i2c error\n", __func__);
	return 0;
}

static ssize_t ts_sensitivity_store(struct device *device,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cypress_ts_data *ts = dev_get_drvdata(device);
	uint8_t wdata[2] = {0};
	int rc;

	wdata[0] = 0x1B; /* 0x1B: sensitivity*/
	wdata[1] = (uint8_t)simple_strtoul(buf, NULL, 0);
	if (2 != i2c_master_send(ts->client, wdata, 2))
		goto i2c_err;

	msleep(100);
	rc = ts->power(TS_RESET);
	if (rc)
		pr_info("%s: fail to reset tp\n", __func__);

	return count;

i2c_err:
	pr_err("%s: i2c error\n", __func__);
	return 0;
}

static ssize_t ts_sensitivity_show(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct cypress_ts_data *ts = dev_get_drvdata(device);
	uint8_t wdata[1] = {0};
	uint8_t rdata[1] = {0};

	wdata[0] = 0x1B; /* 0x1B: sensitivity*/
	if (1 != i2c_master_send(ts->client, wdata, 1))
		goto i2c_err;

	if (1 != i2c_master_recv(ts->client, rdata, 1))
		goto i2c_err;

	return sprintf(buf, "sensitivity: %d\n", rdata[0]);

i2c_err:
	pr_err("%s: i2c error\n", __func__);
	return 0;
}

static struct device_attribute ts_ver_attrs =
__ATTR(version, S_IRWXUGO, cypress_fw_version_show, NULL);

static struct device_attribute ts_sensitivity_attrs =
__ATTR(sensitivity, S_IRWXUGO, ts_sensitivity_show, ts_sensitivity_store);

static int cypress_set_power_state(int status, struct i2c_client *client)
{
	uint8_t wdata[2] = {0};

	pr_debug("%s: status: %x\n", __func__, status);
	switch (status) {
	case INIT_STATE:
		/* TODO: read fw version in initial state */
		break;
	case SUSPEND_STATE:
		/* set deep sleep mode */
		wdata[0] = 0;
		wdata[1] = 2;
		if (2 != i2c_master_send(client, wdata, 2))
			goto i2c_err;
		break;
	default:
		break;
	}

	return 0;

i2c_err:
	pr_err("%s: i2c error (%d)\n", __func__, status);
	return -ENXIO;
}

static void cypress_work_func(struct work_struct *work)
{
	struct cypress_ts_data *ts =
		container_of(work, struct cypress_ts_data, work);
	uint8_t rdata[32] = {0};
	uint8_t width = 1;
	int points = 0;
	int i;

	if (32 != i2c_master_recv(ts->client, rdata, 32)) {
		pr_err("%s: i2c recv error\n", __func__);
		goto i2c_err;
	}

	points = ((rdata[2] & 0x0F) > 4) ? 4 : (rdata[2] & 0x0F);
	pos[0].x = rdata[3] << 8 | rdata[4];
	pos[0].y = rdata[5] << 8 | rdata[6];
	pos[0].z = rdata[7];
	pos[1].x = rdata[9] << 8 | rdata[10];
	pos[1].y = rdata[11] << 8 | rdata[12];
	pos[1].z = rdata[13];
	pos[2].x = rdata[16] << 8 | rdata[17];
	pos[2].y = rdata[18] << 8 | rdata[19];
	pos[2].z = rdata[19];
	pos[3].x = rdata[22] << 8 | rdata[23];
	pos[3].y = rdata[24] << 8 | rdata[25];
	pos[3].z = rdata[26];

	for (i = 0; i < points; i++) {
		/*
		   pr_info("x%d = %u,  y%d = %u, z%d = %u\n",
		   i, pos[i].x, i, pos[i].y, i, pos[i].z);
		 */
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
			(pos[i].x > ts->abs_x_max) ? 0 : pos[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
			(pos[i].y > ts->y_max) ? 0 : pos[i].y);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			(pos[i].z > ts->abs_pressure_max) ? 0 : pos[i].z);
		input_mt_sync(ts->input_dev);
	}

	for (i = 0; i < ts->prev_points - points; i++) {
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
	}

	ts->prev_points = points;
	input_sync(ts->input_dev);

i2c_err:
	enable_irq(ts->client->irq);
}

static irqreturn_t cypress_ts_interrupt(int irq, void *dev_id)
{
	struct cypress_ts_data *ts = dev_id;

	disable_irq(ts->client->irq);
	queue_work(cypress_wq, &ts->work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void cypress_early_suspend(struct early_suspend *h)
{
	struct cypress_ts_data *ts;

	pr_info("%s: Enter\n", __func__);
	ts = container_of(h, struct cypress_ts_data, early_suspend);

	cypress_set_power_state(SUSPEND_STATE, ts->client);
}

void cypress_early_resume(struct early_suspend *h)
{
	struct cypress_ts_data *ts;

	pr_info("%s: Enter\n", __func__);
	ts = container_of(h, struct cypress_ts_data, early_suspend);

	if (ts->power(TS_RESET))
		pr_err("%s: power on failed\n", __func__);
}
#endif

static int cypress_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cypress_ts_data *ts = NULL;
	struct cypress_i2c_platform_data *pdata;
	int ret = 0;

	pr_info("%s: enter\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct cypress_ts_data), GFP_KERNEL);
	if (!ts) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	memset(pos, 0, sizeof(pos));
	INIT_WORK(&ts->work, cypress_work_func);
	ts->client = client;
	strlcpy(client->name, CYPRESS_TS_DRIVER_NAME,
		strlen(CYPRESS_TS_DRIVER_NAME));
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (pdata) {
		ts->hw_init = pdata->hw_init;
		ts->power = pdata->power;
		ts->abs_x_max = pdata->abs_x_max;
		ts->abs_pressure_max = pdata->abs_pressure_max;
		ts->points_max = pdata->points_max;
		ts->y_max = pdata->y_max;

		ret = ts->hw_init(1);
		if (ret) {
			pr_err("%s: hw init failed\n", __func__);
			goto err_hw_init_failed;
		}

		ret = ts->power(TS_RESET);
		if (ret) {
			pr_err("%s: reset failed\n", __func__);
			goto err_power_on_failed;
		}
	}

	msleep(50);
	if (cypress_set_power_state(INIT_STATE, ts->client) != 0) {
		pr_info("%s: set mode  failed\n", __func__);
		goto err_power_on_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = CYPRESS_TS_DRIVER_NAME;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		pdata->abs_width_min, pdata->abs_width_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min, pdata->abs_y_max, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		pr_err("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (client->irq) {
		ret = request_irq(client->irq, cypress_ts_interrupt,
			pdata->irqflags, client->name, ts);
		if (ret) {
			pr_err("%s: Unable to register %s irq\n",
				__func__, ts->input_dev->name);
			ret = -ENOTSUPP;
			goto err_request_irq;
		}
	}

	if (device_create_file(&client->dev, &ts_ver_attrs))
		pr_err("%s: device_create_file ts_ver_attrs error\n", __func__);

	if (device_create_file(&client->dev, &ts_sensitivity_attrs))
		pr_err("%s: device_create_file ts_sensitivity_attrs error\n", __func__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ts->early_suspend.suspend = cypress_early_suspend;
	ts->early_suspend.resume = cypress_early_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	pr_info("%s: probe done\n", __func__);
	return 0;

err_request_irq:
	free_irq(client->irq, ts);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_power_on_failed:
err_hw_init_failed:
	ts->hw_init(0);
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int cypress_remove(struct i2c_client *client)
{
	struct cypress_ts_data *ts = i2c_get_clientdata(client);

	pr_info("%s: enter\n", __func__);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	if (ts->hw_init(0))
		pr_err("%s: hw deinit failed\n", __func__);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id cypress_id[] = {
	{ CYPRESS_TS_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver cypress_ts_driver = {
	.probe		= cypress_probe,
	.remove		= cypress_remove,
	.id_table	= cypress_id,
	.driver		= {
		.name = CYPRESS_TS_DRIVER_NAME,
	},
};

static int __init cypress_init(void)
{
	pr_info("%s: enter\n", __func__);
	cypress_wq = create_singlethread_workqueue("cypress_wq");
	if (!cypress_wq)
		return -ENOMEM;
	return i2c_add_driver(&cypress_ts_driver);
}

static void __exit cypress_exit(void)
{
	pr_info("%s: enter\n", __func__);
	i2c_del_driver(&cypress_ts_driver);
	if (cypress_wq)
		destroy_workqueue(cypress_wq);
}

module_init(cypress_init);
module_exit(cypress_exit);

MODULE_AUTHOR("Peng Chang <Peng_Chang@acer.com.tw>");
MODULE_DESCRIPTION("CYPRESS driver");
MODULE_LICENSE("GPL v2");

