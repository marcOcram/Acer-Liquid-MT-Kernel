/*
 * Acer Vibrator driver.
 *
 * Copyright (C) 2010 ACER Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>
#include "proc_comm.h"

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned id1 = ACER_SMEM_PROC_CMD_HW_CTRL_VIBRATOR;
	unsigned id2 = value;

	pr_debug("%s: value = %d\n", __func__, value);
	msm_proc_comm(PCOM_CUSTOMER_CMD1, &id1, &id2);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	return 0;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{
	int rc = 0;

	rc = timed_output_dev_register(&pmic_vibrator);
	if (rc)
		printk(KERN_ERR "timed_output_dev register failed!\n");
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

