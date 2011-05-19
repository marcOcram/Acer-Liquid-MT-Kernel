/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */
#define DEBUG  1

#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

#ifdef CONFIG_MACH_ACER_A4
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/delay.h>
#include "../../arch/arm/mach-msm/smd_private.h"
#include "../../arch/arm/mach-msm/proc_comm.h"

#define SMB_BATTERY_DRIVER  "smb-battery"
#define POLLING_TIMER  30000  /* 30s */
#define MAX_VOLT_TABLE_SIZE 21
#define MAX_SIZE 4
#define BASE_SYSTEM_LOADING 80

#define SUSPEND_CHANGE_TIME  5400  /* 5400s */
#define AC_CHANGE_TIME  150  /* 150s */
#define NOT_AC_CHANGE_TIME  300  /* 300s */
#endif

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VER_1_1		0x00010001
#define CHG_RPC_VER_1_3		0x00010003
#define CHG_RPC_VER_2_2		0x00020002

#define BATTERY_REGISTER_PROC                          	2
#define BATTERY_MODIFY_CLIENT_PROC                     	4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_READ_MV_PROC 				12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC 		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC 		1
#define BATTERY_CB_ID_ALL_ACTIV       	1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW            	2800
#define BATTERY_HIGH           	4300

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC 	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC 	0xffffffff

#define BATT_RPC_TIMEOUT    5000	/* 5 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))


#if DEBUG
#define DBG_LIMIT(x...) do {if (printk_ratelimit()) pr_debug(x); } while (0)
#else
#define DBG_LIMIT(x...) do {} while (0)
#endif

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
	/* The charger is good      */
	CHARGER_STATUS_GOOD,
	/* The charger is bad       */
	CHARGER_STATUS_BAD,
	/* The charger is weak      */
	CHARGER_STATUS_WEAK,
	/* Invalid charger status.  */
	CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
	/* The charger is removed                 */
	CHARGER_TYPE_NONE,
	/* The charger is a regular wall charger   */
	CHARGER_TYPE_WALL,
	/* The charger is a PC USB                 */
	CHARGER_TYPE_USB_PC,
	/* The charger is a wall USB charger       */
	CHARGER_TYPE_USB_WALL,
	/* The charger is a USB carkit             */
	CHARGER_TYPE_USB_CARKIT,
	/* Invalid charger hardware status.        */
	CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
	/* The battery is good        */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot    */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad         */
	BATTERY_STATUS_BAD,
	/* The battery is removed     */
	BATTERY_STATUS_REMOVED,		/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status.    */
	BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
	/* The battery voltage is dead/very low (less than 3.2V) */
	BATTERY_LEVEL_DEAD,
	/* The battery voltage is weak/low (between 3.2V and 3.4V) */
	BATTERY_LEVEL_WEAK,
	/* The battery voltage is good/normal(between 3.4V and 4.2V) */
	BATTERY_LEVEL_GOOD,
	/* The battery voltage is up to full (close to 4.2V) */
	BATTERY_LEVEL_FULL,
	/* Invalid battery voltage level. */
	BATTERY_LEVEL_INVALID
};

struct rpc_reply_batt_chg_v1 {
	struct rpc_reply_hdr hdr;
	u32 	more_data;

	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
};

struct rpc_reply_batt_chg_v2 {
	struct rpc_reply_batt_chg_v1	v1;

	u32	is_charger_valid;
	u32	is_charging;
	u32	is_battery_valid;
	u32	ui_event;
};

union rpc_reply_batt_chg {
	struct rpc_reply_batt_chg_v1	v1;
	struct rpc_reply_batt_chg_v2	v2;
};

static union rpc_reply_batt_chg rep_batt_chg;

#ifdef CONFIG_MACH_ACER_A4
typedef struct {
	u16	voltage;
	u16	capacity;
} acer_battery_table_type;

acer_battery_table_type acer_battery_dsg_25C_10mA_table[] = {
	{4205, 100},
	{4144, 95},
	{4096, 90},
	{4059, 85},
	{3998, 80},
	{3971, 75},
	{3942, 70},
	{3915, 65},
	{3889, 60},
	{3843, 55},
	{3817, 50},
	{3801, 45},
	{3790, 40},
	{3782, 35},
	{3777, 30},
	{3769, 25},
	{3750, 20},
	{3718, 15},
	{3692, 10},
	{3673, 5},
	{2963, 0}
};

acer_battery_table_type acer_battery_dsg_25C_100mA_table[] = {
	{4195, 100},
	{4115, 95},
	{4070, 90},
	{4030, 85},
	{3976, 80},
	{3950, 75},
	{3921, 70},
	{3889, 65},
	{3854, 60},
	{3819, 55},
	{3801, 50},
	{3785, 45},
	{3774, 40},
	{3764, 35},
	{3758, 30},
	{3750, 25},
	{3734, 20},
	{3702, 15},
	{3676, 10},
	{3657, 5},
	{3011, 0}
};

acer_battery_table_type acer_battery_dsg_25C_300mA_table[] = {
	{4201, 100},
	{4095, 95},
	{4045, 90},
	{4006, 85},
	{3948, 80},
	{3916, 75},
	{3884, 70},
	{3850, 65},
	{3818, 60},
	{3792, 55},
	{3770, 50},
	{3755, 45},
	{3741, 40},
	{3731, 35},
	{3723, 30},
	{3712, 25},
	{3696, 20},
	{3665, 15},
	{3641, 10},
	{3615, 5},
	{3025, 0}
};

acer_battery_table_type acer_battery_dsg_25C_500mA_table[] = {
	{4148, 100},
	{4066, 95},
	{4016, 90},
	{3974, 85},
	{3918, 80},
	{3876, 75},
	{3844, 70},
	{3813, 65},
	{3784, 60},
	{3760, 55},
	{3739, 50},
	{3720, 45},
	{3704, 40},
	{3694, 35},
	{3683, 30},
	{3670, 25},
	{3654, 20},
	{3625, 15},
	{3601, 10},
	{3562, 5},
	{3031, 0}
};

acer_battery_table_type acer_battery_chg_25C_500mA_table[] = {
	{4212, 100},
	{4188, 95},
	{4159, 90},
	{4138, 85},
	{4095, 80},
	{4069, 75},
	{4051, 70},
	{4024, 65},
	{3995, 60},
	{3969, 55},
	{3948, 50},
	{3929, 45},
	{3913, 40},
	{3900, 35},
	{3889, 30},
	{3881, 25},
	{3866, 20},
	{3831, 15},
	{3797, 10},
	{3776, 5},
	{3377, 0}
};

acer_battery_table_type acer_battery_chg_25C_900mA_table[] = {
	{4203, 100},
	{4189, 95},
	{4176, 90},
	{4160, 85},
	{4139, 80},
	{4128, 75},
	{4115, 70},
	{4088, 65},
	{4059, 60},
	{4035, 55},
	{4016, 50},
	{3998, 45},
	{3984, 40},
	{3971, 35},
	{3955, 30},
	{3942, 25},
	{3923, 20},
	{3891, 15},
	{3851, 10},
	{3819, 5},
	{3359, 0}
};

int module_loading_table[MAX_SIZE] = {
	0,	/* init value */
	0,	/* WiFi */
	0,	/* camera */
	0	/* phone call */
};

static int acer_hw_version;
static int old_enable;
#endif

struct msm_battery_info {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity; /* in percentage */

	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage; /* in millie volts */
	u32 battery_temp;  /* in celsius */

	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;

	struct early_suspend early_suspend;
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_status = CHARGER_STATUS_BAD,
	.charger_type = CHARGER_TYPE_INVALID,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_level = BATTERY_LEVEL_FULL,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
	.battery_temp = 23,
	.vbatt_modify_reply_avail = 0,
};

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

struct msm_batt_get_volt_ret_data {
	u32 battery_voltage;
};

static int msm_batt_get_volt_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_volt_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_volt_ret_data *)data;
	buf_ptr = (struct msm_batt_get_volt_ret_data *)buf;

	data_ptr->battery_voltage = be32_to_cpu(buf_ptr->battery_voltage);

	return 0;
}

#ifdef CONFIG_MACH_ACER_A4
/* USBIN mode */
enum {
	/* not charging */
	NONE,
	/* charging with USB */
	USB1_mode,
	/* charging with USB */
	USB5_mode,
	/* charging with AC */
	HC_mode
};

/* USB5/1/HC control */
enum {
	I2C_control,
	PIN_control
};

/* AICL detection threshold */
enum {
	/* 4.25V */
	voltage_4250,
	/* 4.50V */
	voltage_4500,
	/* 4.75V */
	voltage_4750,
	/* 5.00V */
	voltage_5000
};

enum {
	/* 382 min */
	minute_382,
	/* 764 min */
	minute_764,
	/* 1527 min */
	minute_1527,
	/* Disabled */
	disabled
};

/* different system loading cases */
enum {
	BATT_DSG_25C_10mA,
	BATT_DSG_25C_100mA,
	BATT_DSG_25C_300mA,
	BATT_DSG_25C_500mA,
	BATT_CHG_25C_500mA,
	BATT_CHG_25C_900mA
};

struct smb_reply_batt_chg {
	u32  charger_status;
	u32  charger_type;
	u32  battery_status;
	u32  battery_level;
	u32  battery_voltage;
	u32  battery_temp;
	u32  battery_capacity;
};

/* Data for I2C driver */
struct smb_data {
	struct i2c_client *client;
	struct work_struct work_for_polling;
	struct timer_list polling_timer;
};

static struct i2c_client *private_smb136_client;
static struct i2c_driver smb_batt_driver;
static struct smb_reply_batt_chg smb_rep_batt_chg;
static struct smb_data *smb_pdata;

static u32 base_battery_capacity;
static int Is_enable_fast_charging;
static int Is_full_charging;
static int pre_USBIN_mode;
acer_smem_flag_t *acer_smem_flag;

struct timespec current_time;
struct timespec change_time;

static struct wake_lock main_wlock;
/*
  * if >= +2, voltage increase over 2 runs,
  * if <= -2, voltage discrease over 2 runs
  */
static int change_counter = 0;
/*
  * Invalid : 3
  * AC : 2
  * USB : 0
  */
static u32 g_chg_type = 3;
/*
  * charging: 1
  * not charging: 0
  */
static int smb_chg_status = 0;

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
	if (1 != i2c_master_send(client, buf, 1)) {
		pr_err("SBM_BATT: i2c_read --> Send reg. info error\n");
		return -1;
	}

	if (count != i2c_master_recv(client, buf, count)) {
		pr_err("SBM_BATT: i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

/*
 * client: target client
 * buf: target register with command
 * count: length of transmitting
 */
static int i2c_write(struct i2c_client *client, char *buf, int count)
{
	if (count != i2c_master_send(client, buf, count)) {
		pr_err("SBM_BATT: i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

int batt_module_enable(int module, bool state)
{
	int i = 0;
	int sys_module_loading = 0;

	if (module <= MAX_SIZE - 1) {
		module_loading_table[module] = state;

		for (i = 0; i < MAX_SIZE; i++) {
			if (module_loading_table[i] == 1) {
				sys_module_loading = 300;
				break;
			} else
				sys_module_loading = 100;
		}
	} else {
		pr_err("[Batt]parameter is wrong in batt_module_enable\n");
		return -1;
	}

	return sys_module_loading;
}
EXPORT_SYMBOL(batt_module_enable);

/* detect string: WiFi, Camera, Phone */
static void batt_module_process(const char *buf, bool state)
{
	switch (buf[0])	{
	case 'w':
		batt_module_enable(WIFI, state);
		break;
	case 'c':
		batt_module_enable(CAMERA, state);
		break;
	case 'p':
		batt_module_enable(PHONE, state);
		break;
	}
}

static ssize_t module_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	batt_module_process(buf, true);
	return count;
}

static ssize_t module_disable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	batt_module_process(buf, false);
	return count;
}

static DEVICE_ATTR(module_enable, S_IWUGO, NULL, module_enable_store);
static DEVICE_ATTR(module_disable, S_IWUGO, NULL, module_disable_store);

/* create entry under /sys/devices/platform/msm-battery/power_supply/battery/ */
static void module_use_sys_create(struct power_supply *psy)
{
	int rc = 0;
	rc = device_create_file(psy->dev, &dev_attr_module_enable);
	rc = device_create_file(psy->dev, &dev_attr_module_disable);
	if (rc)
		pr_err(" module_use_sys_create error!\n");
}

static void module_use_sys_delete(struct power_supply *psy)
{
	device_remove_file(psy->dev, &dev_attr_module_enable);
	device_remove_file(psy->dev, &dev_attr_module_disable);
}

static u32 acer_batt_capacity_scheme(u32 pre_chg_status, u32 chg_status,
				u32 pre_batt_capacity, u32 batt_capacity, u32 batt_table)
{
	u32 transfer_batt_capacity = 0;
	unsigned long int delta_time = 0, change_capacity_time = 0;

	if ((chg_status == CHARGER_STATUS_GOOD) && (Is_full_charging == 1)) {
		transfer_batt_capacity = 100;
		base_battery_capacity = transfer_batt_capacity;
		DBG_LIMIT("BATT: Charging full\n");
		goto done;
	}

	if (pre_chg_status != chg_status) {
		base_battery_capacity = pre_batt_capacity;
		transfer_batt_capacity = pre_batt_capacity;

		change_counter = 0;
		goto done;
	} else {
		current_time = current_kernel_time();
		delta_time = current_time.tv_sec - change_time.tv_sec;

		if (delta_time > SUSPEND_CHANGE_TIME) {
			change_time = current_kernel_time();

			base_battery_capacity = batt_capacity;
			transfer_batt_capacity = batt_capacity;
			goto filter;
		} else {
			if (base_battery_capacity == batt_capacity) {
				transfer_batt_capacity = batt_capacity;
				goto filter;
			} else {
				/* for AC, increase 5%: 300s; for USB, increase 5%: 540s;
				get half time: 150s and 300s to avoid gapping */
				if (batt_table == BATT_CHG_25C_900mA)
					change_capacity_time = AC_CHANGE_TIME;
				else
					change_capacity_time = NOT_AC_CHANGE_TIME;

				if ((delta_time > change_capacity_time) && ((change_counter >= 2) || (change_counter <= -2))) {
					change_time = current_kernel_time();

					if (change_counter <= -2)
						transfer_batt_capacity = base_battery_capacity - 5;
					else
						transfer_batt_capacity = base_battery_capacity + 5;

					base_battery_capacity = transfer_batt_capacity;
					change_counter  = 0;
				} else {
					if (pre_batt_capacity > batt_capacity)
						change_counter--;
					else if (pre_batt_capacity < batt_capacity)
						change_counter++;

					transfer_batt_capacity = pre_batt_capacity;
				}
			}
		}
	}

filter:
	/* Avoid capacity increasing/decreasing when not charging/charging */
	if (smb_chg_status == 0) {
		if (transfer_batt_capacity > pre_batt_capacity) {
			transfer_batt_capacity = pre_batt_capacity;
			base_battery_capacity = transfer_batt_capacity;
		}
	} else {
		if (transfer_batt_capacity < pre_batt_capacity) {
			transfer_batt_capacity = pre_batt_capacity;
			base_battery_capacity = transfer_batt_capacity;
		}
	}

done:
	return transfer_batt_capacity;
}

static u32 acer_batt_calculate_capacity(u32 current_voltage, u32 current_case)
{
	u32 current_capacity = 0;
	static u32 shutdown_counter = 0;
	acer_battery_table_type batt_lookup_table[MAX_VOLT_TABLE_SIZE] = { {0, 0} };

	int index;

	if (current_case == BATT_CHG_25C_900mA)
		memcpy(batt_lookup_table, acer_battery_chg_25C_900mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));
	else if(current_case == BATT_CHG_25C_500mA)
		memcpy(batt_lookup_table, acer_battery_chg_25C_500mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));
	else if (current_case == BATT_DSG_25C_500mA)
		memcpy(batt_lookup_table, acer_battery_dsg_25C_500mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));
	else if (current_case == BATT_DSG_25C_300mA)
		memcpy(batt_lookup_table, acer_battery_dsg_25C_300mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));
	else if (current_case == BATT_DSG_25C_100mA)
		memcpy(batt_lookup_table, acer_battery_dsg_25C_100mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));
	else if (current_case == BATT_DSG_25C_10mA)
		memcpy(batt_lookup_table, acer_battery_dsg_25C_10mA_table,
				MAX_VOLT_TABLE_SIZE*sizeof(acer_battery_table_type));

	if (current_voltage > 3500) {
		for (index = 0; index < MAX_VOLT_TABLE_SIZE-1; index++) {
			if (current_voltage >= batt_lookup_table[0].voltage) {
				current_capacity = 100;
				break;
			}

			if ((current_voltage < batt_lookup_table[index].voltage) &&
				(current_voltage >= batt_lookup_table[index+1].voltage)) {
				current_capacity = batt_lookup_table[index].capacity;
				break;
			}
		}

		shutdown_counter = 0;
	} else {
		shutdown_counter = shutdown_counter + 1;
		if (shutdown_counter >= 2)
			current_capacity = 0;
		else
			current_capacity = 5;
	}

	return current_capacity;
}

static int smb_dis_power_detection(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	int rs = 0;

	reg_buf[0] = 0x06;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x06 in smb_dis_power_detection()\n");
		return rs;
	}

	reg_buf[1] = reg_buf[0] | 0x20;
	reg_buf[0] = 0x06;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x06 fail in smb_dis_power_detection()\n");
	}

	return rs;
}

static int smb_batt_chg_type(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[1] = {0};
	uint8_t value = 0;
	int rs = 0;

	reg_buf[0] = 0x34;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read 0x34 fail in smb_batt_chg_type()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value & 0x01;
	if (!value)
		/* USB */
		return 0;
	else
		/* AC */
		return 1;

	return rs;
}

static u32 acer_batt_booting_capacity(u32 pmic_voltage)
{
	u32 init_chg_type = 0;
	u32 real_voltage = 0;
	u32 booting_capacity = 0;

	if (pmic_voltage > 10000) {
		init_chg_type = smb_batt_chg_type();
		real_voltage = pmic_voltage - 10000;

		if (init_chg_type == 0)
			booting_capacity = acer_batt_calculate_capacity(real_voltage, BATT_CHG_25C_500mA);
		else
			booting_capacity = acer_batt_calculate_capacity(real_voltage, BATT_CHG_25C_900mA);
	} else {
		real_voltage = pmic_voltage;
		booting_capacity = acer_batt_calculate_capacity(real_voltage, BATT_DSG_25C_100mA);
	}

	return booting_capacity;
}

static int smb_batt_full_charging(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[1] = {0};
	uint8_t value = 0;
	int rs = 0;

	reg_buf[0] = 0x36;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read 0x36 fail in smb_charging_full()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value & 0x40;
	if (value)
		Is_full_charging = 1;
	else
		Is_full_charging = 0;

	return rs;
}

static int smb_batt_change_USBIN_mode(uint8_t change_mode)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0, USBIN_mode = 0;
	int rs = 0;

	if (pre_USBIN_mode != change_mode) {
		/* USBIN mode */
		reg_buf[0] = 0x31;
		rs = i2c_read(client, reg_buf, 1);
		if (rs == -1) {
			pr_err("[SMB]i2c_read 0x31 fail in smb_batt_change_USBIN_mode()\n");
			return rs;
		} else {
			value = reg_buf[0];
		}

		if (change_mode == USB5_mode)
			/* USB 500mA mode */
			USBIN_mode = value | 0x08;
		else if (change_mode == USB1_mode)
			/* USB 100mA mode */
			USBIN_mode = value | 0x00;
		else
			/*AC mode */
			USBIN_mode = value | 0x04;

		reg_buf[0] = 0x31;
		reg_buf[1] = USBIN_mode;
		rs = i2c_write(client, reg_buf, 2);
		if (rs == -1) {
			pr_err("[SMB]i2c_write 0x31 fail in smb_batt_change_USBIN_mode()\n");
			return rs;
		}

		pre_USBIN_mode = change_mode;
	}

	return rs;
}

void batt_chg_type_notify_callback(uint8_t type)
{
	g_chg_type = type;
	schedule_work(&smb_pdata->work_for_polling);
}
EXPORT_SYMBOL(batt_chg_type_notify_callback);

static int smb_control(int change_control)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int pre_control = PIN_control, rs = 0;

	if (pre_control != change_control) {
		/* Pin control <-> I2C control */
		reg_buf[0] = 0x05;
		rs = i2c_read(client, reg_buf, 1);
		if (rs == -1) {
			pr_err("[SMB]i2c_read fail reg 0x05 in smb_control()\n");
			return rs;
		} else {
			value = reg_buf[0];
		}

		if (change_control == I2C_control)
			value = value & ~(0x10);
		else
			value = value | 0x10;

		reg_buf[0] = 0x05;
		reg_buf[1] = value;
		rs = i2c_write(client, reg_buf, 2);
		if (rs == -1) {
			pr_err("[SMB]i2c_write reg 0x05 fail in smb_control()\n");
			return rs;
		}

		pre_control = change_control;
	}

	return rs;
}

static int smb_batt_is_charging(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0, is_charging = 0;
	int rs = 0;

	/* get charging status */
	reg_buf[0] = 0x36;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x36 in SMB_allow_volatile_write()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	is_charging = value & 0x06;
	if (!is_charging)
		/* not charging */
		return 0;
	else
		/* charging */
		return 1;
}

static int batt_get_batt_chg_status(void)
{
	int rs = 0;

	/* Detect Charge Status */
	if (g_chg_type == 3) {
		wake_unlock(&main_wlock);

		/* when fast charging, plugging out USB */
		if (Is_enable_fast_charging == 1) {
			rs = smb_fast_charging(0);
			if (rs == -1) {
				pr_err("[SMB]smb_fast_charging fail\n");
				return rs;
			}
		}
		smb_control(PIN_control);

		acer_smem_flag->acer_charger_type = ACER_CHARGER_TYPE_NO_CHARGER;
		smb_rep_batt_chg.charger_status = CHARGER_STATUS_INVALID;
		smb_rep_batt_chg.charger_type = CHARGER_TYPE_NONE;

		pre_USBIN_mode = NONE;
	} else
		smb_rep_batt_chg.charger_status = CHARGER_STATUS_GOOD;

	if (smb_rep_batt_chg.charger_status == CHARGER_STATUS_GOOD) {
		/* Detect Charge Type */
		if (g_chg_type == 2) {
			/* prevent from entering suspend when plugging in AC */
			wake_lock(&main_wlock);

			acer_smem_flag->acer_charger_type = ACER_CHARGER_TYPE_IS_AC;
			smb_rep_batt_chg.charger_type = CHARGER_TYPE_WALL;

			/* change current to AC 950mA mode */
			rs = smb_batt_change_USBIN_mode(HC_mode);
			if (rs == -1) {
				pr_err("[SMB]HC_mode fail in batt_get_batt_chg_status()\n");
				return rs;
			}
		} else if (g_chg_type == 0) {
			acer_smem_flag->acer_charger_type = ACER_CHARGER_TYPE_IS_USB;
			smb_rep_batt_chg.charger_type = CHARGER_TYPE_USB_PC;

			if (Is_enable_fast_charging == 0) {
				/* change current to USB 500mA mode */
				rs = smb_batt_change_USBIN_mode(USB5_mode);
				if (rs == -1) {
					pr_err("[SMB]USB5_mode fail in batt_get_batt_chg_status()\n");
					return rs;
				}
			}
		}
		smb_control(I2C_control);

		rs = smb_batt_full_charging();
	}

	return rs;
}
#endif

static u32 msm_batt_get_vbatt_voltage(void)
{
	int rc;

	struct msm_batt_get_volt_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_MV_PROC,
			NULL, NULL,
			msm_batt_get_volt_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get volt. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.battery_voltage;
}

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))
#ifndef CONFIG_MACH_ACER_A4
static int msm_batt_get_batt_chg_status(void)
{
	int rc ;
	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;
	struct rpc_reply_batt_chg_v1 *v1p;

	req_batt_chg.more_data = cpu_to_be32(1);

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	v1p = &rep_batt_chg.v1;
	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_batt_chg, sizeof(rep_batt_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(v1p->more_data)) {
		be32_to_cpu_self(v1p->charger_status);
		be32_to_cpu_self(v1p->charger_type);
		be32_to_cpu_self(v1p->battery_status);
		be32_to_cpu_self(v1p->battery_level);
		be32_to_cpu_self(v1p->battery_voltage);
		be32_to_cpu_self(v1p->battery_temp);
	} else {
		pr_err("%s: No battery/charger data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}
#endif
static void msm_batt_update_psy_status(void)
{
#ifndef CONFIG_MACH_ACER_A4
	static u32 unnecessary_event_count;
#endif
	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
	struct	power_supply	*supp;

#ifdef CONFIG_MACH_ACER_A4
	int ret = 0;
	static int init_calculation = 0;
	static u32 pre_charger_status = 0;
	static u32 pre_battery_capacity = 0;
	int avg_system_loading = 0;
	u32 batt_chg_case = 0;
	u32 battery_capacity = 0;

	ret = batt_get_batt_chg_status();
	charger_status = smb_rep_batt_chg.charger_status;
	charger_type = smb_rep_batt_chg.charger_type;

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	battery_status = rep_batt_chg.v1.battery_status;
	battery_level = rep_batt_chg.v1.battery_level;
	battery_voltage = msm_batt_get_vbatt_voltage();
	battery_temp = rep_batt_chg.v1.battery_temp;
#else
	if (msm_batt_get_batt_chg_status())
		return;

	charger_status = rep_batt_chg.v1.charger_status;
	charger_type = rep_batt_chg.v1.charger_type;
	battery_status = rep_batt_chg.v1.battery_status;
	battery_level = rep_batt_chg.v1.battery_level;
	battery_voltage = rep_batt_chg.v1.battery_voltage;
	battery_temp = rep_batt_chg.v1.battery_temp;
#endif

	/* Make correction for battery status */
	if (battery_status == BATTERY_STATUS_INVALID_v1) {
		if (msm_batt_info.chg_api_version < CHG_RPC_VER_2_2)
			battery_status = BATTERY_STATUS_INVALID;
	}

#ifndef CONFIG_MACH_ACER_A4
	if (charger_status == msm_batt_info.charger_status &&
	    charger_type == msm_batt_info.charger_type &&
	    battery_status == msm_batt_info.battery_status &&
	    battery_level == msm_batt_info.battery_level &&
	    battery_voltage == msm_batt_info.battery_voltage &&
	    battery_temp == msm_batt_info.battery_temp) {
		/* Got unnecessary event from Modem PMIC VBATT driver.
		 * Nothing changed in Battery or charger status.
		 */
		unnecessary_event_count++;
		if ((unnecessary_event_count % 20) == 1)
			DBG_LIMIT("BATT: same event count = %u\n",
				 unnecessary_event_count);
		return;
	}

	unnecessary_event_count = 0;
#endif

	DBG_LIMIT("BATT: rcvd: %d, %d, %d, %d; %d, %d\n",
		 charger_status, charger_type, battery_status,
		 battery_level, battery_voltage, battery_temp);

	if (battery_status == BATTERY_STATUS_INVALID &&
	    battery_level != BATTERY_LEVEL_INVALID) {
		DBG_LIMIT("BATT: change status(%d) to (%d) for level=%d\n",
			 battery_status, BATTERY_STATUS_GOOD, battery_level);
		battery_status = BATTERY_STATUS_GOOD;
	}

	if (msm_batt_info.charger_type != charger_type) {
		if (charger_type == CHARGER_TYPE_USB_WALL ||
		    charger_type == CHARGER_TYPE_USB_PC ||
		    charger_type == CHARGER_TYPE_USB_CARKIT) {
			DBG_LIMIT("BATT: USB charger plugged in\n");
			msm_batt_info.current_chg_source = USB_CHG;
			supp = &msm_psy_usb;
		} else if (charger_type == CHARGER_TYPE_WALL) {
			DBG_LIMIT("BATT: AC Wall changer plugged in\n");
			msm_batt_info.current_chg_source = AC_CHG;
			supp = &msm_psy_ac;
		} else {
			if (msm_batt_info.current_chg_source & AC_CHG)
				DBG_LIMIT("BATT: AC Wall charger removed\n");
			else if (msm_batt_info.current_chg_source & USB_CHG)
				DBG_LIMIT("BATT: USB charger removed\n");
			else
				DBG_LIMIT("BATT: No charger present\n");
			msm_batt_info.current_chg_source = 0;
			supp = &msm_psy_batt;

			/* Correct charger status */
			if (charger_status != CHARGER_STATUS_INVALID) {
				DBG_LIMIT("BATT: No charging!\n");
				charger_status = CHARGER_STATUS_INVALID;
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
	} else
		supp = NULL;

	if (msm_batt_info.charger_status != charger_status) {
		if (charger_status == CHARGER_STATUS_GOOD ||
		    charger_status == CHARGER_STATUS_WEAK) {
			if (msm_batt_info.current_chg_source) {
				DBG_LIMIT("BATT: Charging.\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;

				/* Correct when supp==NULL */
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;
				else
					supp = &msm_psy_usb;
			}
		} else {
			DBG_LIMIT("BATT: No charging.\n");
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			supp = &msm_psy_batt;
		}
	} else {
		/* Correct charger status */
		if (charger_type != CHARGER_TYPE_INVALID &&
		    charger_status == CHARGER_STATUS_GOOD) {
			DBG_LIMIT("BATT: In charging\n");
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_CHARGING;
		}
	}
#ifndef CONFIG_MACH_ACER_A4
	/* Correct battery voltage and status */
	if (!battery_voltage) {
		if (charger_status == CHARGER_STATUS_INVALID) {
			DBG_LIMIT("BATT: Read VBATT\n");
			battery_voltage = msm_batt_get_vbatt_voltage();
		} else
			/* Use previous */
			battery_voltage = msm_batt_info.battery_voltage;
	}
#endif
	if (battery_status == BATTERY_STATUS_INVALID) {
		if (battery_voltage >= msm_batt_info.voltage_min_design &&
		    battery_voltage <= msm_batt_info.voltage_max_design) {
			DBG_LIMIT("BATT: Battery valid\n");
			msm_batt_info.batt_valid = 1;
			battery_status = BATTERY_STATUS_GOOD;
		}
	}

	if (msm_batt_info.battery_status != battery_status) {
		if (battery_status != BATTERY_STATUS_INVALID) {
			msm_batt_info.batt_valid = 1;

			if (battery_status == BATTERY_STATUS_BAD) {
				DBG_LIMIT("BATT: Battery bad.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_DEAD;
			} else if (battery_status == BATTERY_STATUS_BAD_TEMP) {
				DBG_LIMIT("BATT: Battery overheat.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_OVERHEAT;
			} else {
				DBG_LIMIT("BATT: Battery good.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_GOOD;
			}
		} else {
			msm_batt_info.batt_valid = 0;
			DBG_LIMIT("BATT: Battery invalid.\n");
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		}

		if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			if (battery_status == BATTERY_STATUS_INVALID) {
				DBG_LIMIT("BATT: Battery -> unknown\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_UNKNOWN;
			} else {
#ifdef CONFIG_MACH_ACER_A4
				/* battery status only charging and
					not charging for Acer projects*/
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
#else
				DBG_LIMIT("BATT: Battery -> discharging\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_DISCHARGING;
#endif
			}
		}

		if (!supp) {
			if (msm_batt_info.current_chg_source) {
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;
				else
					supp = &msm_psy_usb;
			} else
				supp = &msm_psy_batt;
		}
	}

	msm_batt_info.charger_status 	= charger_status;
	msm_batt_info.charger_type 	= charger_type;
	msm_batt_info.battery_status 	= battery_status;
	msm_batt_info.battery_level 	= battery_level;
	msm_batt_info.battery_temp 	= battery_temp;

	if (msm_batt_info.battery_voltage != battery_voltage) {
		msm_batt_info.battery_voltage = battery_voltage * 1000;
#ifdef CONFIG_MACH_ACER_A4
		smb_chg_status = smb_batt_is_charging();
		if (smb_chg_status == 1) {
			/* select which table */
			if (charger_type == CHARGER_TYPE_WALL)
				batt_chg_case = BATT_CHG_25C_900mA;
			else if (charger_type == CHARGER_TYPE_USB_PC)
				batt_chg_case = BATT_CHG_25C_500mA;
		} else if (smb_chg_status == 0) {
			if (charger_type == CHARGER_TYPE_NONE) {
				/* select proper dsg table*/
				avg_system_loading = batt_module_enable(INIT, false);
				if (avg_system_loading == 300)
					batt_chg_case = BATT_DSG_25C_300mA;
				else
					batt_chg_case = BATT_DSG_25C_100mA;
			}
		} else
			pr_err("[Batt]smb_batt_is_charging fail\n");

		if (init_calculation != 1) {
			battery_capacity = acer_batt_booting_capacity(battery_voltage);
			base_battery_capacity = battery_capacity;
			pre_battery_capacity = battery_capacity;
			init_calculation = 1;

			ret = smb_dis_power_detection();
		} else {
			if (battery_voltage > 10000)
				battery_voltage = battery_voltage - 10000;
			battery_capacity = acer_batt_calculate_capacity(battery_voltage, batt_chg_case);
		}

		if (battery_capacity != 0) {
			msm_batt_info.batt_capacity =
				acer_batt_capacity_scheme(pre_charger_status,
					charger_status, pre_battery_capacity, battery_capacity, batt_chg_case);
			if (msm_batt_info.batt_capacity == 100)
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			msm_batt_info.batt_capacity = 0;
			DBG_LIMIT("BATT: Power off Shutting down\n");
		}

		pre_battery_capacity = msm_batt_info.batt_capacity;
		pre_charger_status = charger_status;
#else
		msm_batt_info.batt_capacity =
			msm_batt_info.calculate_capacity(battery_voltage);
#endif
		DBG_LIMIT("BATT: voltage = %u mV [capacity = %d%%]\n",
			 battery_voltage, msm_batt_info.batt_capacity);

		if (!supp)
			supp = msm_batt_info.current_ps;
	}

	if (supp) {
		msm_batt_info.current_ps = supp;
		DBG_LIMIT("BATT: Supply = %s\n", supp->name);
		power_supply_changed(supp);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
		       __func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, rep.result);
		return -EIO;
	}

	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);
#ifdef CONFIG_MACH_ACER_A4
	acer_smem_flag->acer_os_pwr_state = ACER_OS_SUSPEND_MODE;
#endif
	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);

		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);
#ifdef CONFIG_MACH_ACER_A4
	acer_smem_flag->acer_os_pwr_state = ACER_OS_NORMAL_MODE;
	schedule_work(&smb_pdata->work_for_polling);
#endif
	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;


	*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->more_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_error);
	size += sizeof(u32);

	return size;
}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_registration_rep *)data;
	buf_ptr = (struct batt_client_registration_rep *)buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_rep batt_reg_rep;
	int rc;

	batt_reg_req.desired_batt_voltage = desired_batt_voltage;
	batt_reg_req.voltage_direction = voltage_direction;
	batt_reg_req.batt_cb_id = batt_cb_id;
	batt_reg_req.cb_data = cb_data;
	batt_reg_req.more_data = 1;
	batt_reg_req.batt_error = 0;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, &batt_reg_req,
			msm_batt_register_ret_func, &batt_reg_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	pr_debug("%s: got handle = %d\n", __func__, msm_batt_info.batt_handle);

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}

static int msm_batt_cleanup(void)
{
	int rc = 0;

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			       __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);


	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
			       __func__, rc);
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
	return rc;
}

static u32 msm_batt_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_batt_info.voltage_min_design;
	u32 high_voltage = msm_batt_info.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}


int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			  ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
			       __func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. "
			"The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
		msm_batt_update_psy_status();

	return rc;
}

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	if (!msm_batt_info.calculate_capacity)
		msm_batt_info.calculate_capacity = msm_batt_capacity;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

#ifdef CONFIG_MACH_ACER_A4
	module_use_sys_create(&msm_psy_batt);
#endif

	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif
	msm_batt_update_psy_status();

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   },
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	msm_batt_info.chg_ep =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_2_2, 0);

	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_1;
	} else
		msm_batt_info.chg_api_version =  CHG_RPC_VER_2_2;

	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
			msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_2_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
		       __func__);
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_1_1;
	} else
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_2_1;

	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

	return rc;
}

#ifdef CONFIG_MACH_ACER_A4
static void battery_work_func(struct work_struct *work)
{
	msm_batt_update_psy_status();
}

static void polling_timer_func(unsigned long unused)
{
	schedule_work(&smb_pdata->work_for_polling);
	mod_timer(&smb_pdata->polling_timer, jiffies + msecs_to_jiffies(POLLING_TIMER));
}

static int smb_input_current_limit(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	int rs = 0;

	/* set 1000mA */
	reg_buf[0] = 0x01;
	reg_buf[1] = 0x61;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x01 fail in smb_input_current_limit()\n");
		return rs;
	}

	return rs;
}

static int smb_set_AICL_detection_threshold(int threshold)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int rs = 0;

	/* AICL Detection Threshold */
	reg_buf[0] = 0x01;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x01 in smb_set_AICL_detection_threshold()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value & (~0x03);
	switch (threshold) {
	case voltage_4250:
		value = value | 0x00;		/* set 4.25V */
		break;
	case voltage_4500:
		value = value | 0x01;		/* set 4.50V */
		break;
	case voltage_4750:
		value = value | 0x02;		/* set 4.75V */
		break;
	case voltage_5000:
		value = value | 0x03;		/* set 5.00V */
		break;
	default:
		pr_err("[SMB]Incorrect threshold\n");
		break;
	}

	reg_buf[0] = 0x01;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x01 fail in smb_set_AICL_detection_threshold()\n");
		return rs;
	}

	return rs;
}

int smb_fast_charging(int enable)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int rs = 0;

	if (old_enable == enable)
		return rs;

	if (enable) {
		/* set AICL threshold to 4.25V to implement Acer NB fast charging feature */
		rs = smb_set_AICL_detection_threshold(voltage_4250);
		if (rs == -1) {
			pr_err("[SMB]smb_set_AICL_detection_threshold fail\n");
			return rs;
		}
	} else {
		/* restore default settings */
		rs = smb_set_AICL_detection_threshold(voltage_4500);
		if (rs == -1) {
			pr_err("[SMB]smb_set_AICL_detection_threshold fail\n");
			return rs;
		}
	}

	reg_buf[0] = 0x31;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x31 in smb_fast_charging()\n");
		goto i2cfailed;
	}

	/* 1A charging current setting */
	if (enable) {
		value = reg_buf[0] | 0x04;
		Is_enable_fast_charging = 1;
	} else {
		value = reg_buf[0] & ~0x04;
		value = value | 0x08;	/* USB 500mA mode */
		Is_enable_fast_charging = 0;
	}

	reg_buf[0] = 0x31;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x31 fail in smb_fast_charging()\n");
		goto i2cfailed;
	}

	old_enable = enable;

i2cfailed:
	return rs;
}
EXPORT_SYMBOL(smb_fast_charging);

static int smb_set_complete_charger_timeout(int timeout)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int rs = 0;

	reg_buf[0] = 0x09;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x01 in smb_set_complete_charger_timeout()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value & (~0x0C);
	switch (timeout) {
	case minute_382:
		value = value | 0x00;
		break;
	case minute_764:
		value = value | 0x04;
		break;
	case minute_1527:
		value = value | 0x08;
		break;
	case disabled:
		value = value | 0x0C;
		break;
	default:
		pr_err("[SMB]Incorrect timeout parameter\n");
		break;
	}

	reg_buf[0] = 0x09;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x01 fail in smb_set_complete_charger_timeout()\n");
		return rs;
	}

	return rs;
}

static int smb_set_STAT_pin(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int rs = 0;

	/* set STAT output polarity to low true */
	reg_buf[0] = 0x02;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail in SMB_set_STAT_pin()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value | 0x80;

	reg_buf[0] = 0x02;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x02 fail in SMB_set_STAT_pin()\n");
		return rs;
	}

	/* set Interrupt output to IRQ pulses STAT pin */
	reg_buf[0] = 0x04;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail in SMB_set_STAT_pin()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value & ~(0x03);

	reg_buf[0] = 0x04;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x04 fail in SMB_set_STAT_pin()\n");
		return rs;
	}

	return rs;
}

static int smb_allow_volatile_write(void)
{
	struct i2c_client *client = private_smb136_client;
	uint8_t reg_buf[2] = {0};
	uint8_t value = 0;
	int rs = 0;

	/* allow volatile writes to 00h-09h */
	reg_buf[0] = 0x31;
	rs = i2c_read(client, reg_buf, 1);
	if (rs == -1) {
		pr_err("[SMB]i2c_read fail reg 0x31 in SMB_allow_volatile_write()\n");
		return rs;
	} else {
		value = reg_buf[0];
	}

	value = value | 0x80;

	reg_buf[0] = 0x31;
	reg_buf[1] = value;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[SMB]i2c_write reg 0x31 fail in SMB_allow_volatile_write()\n");
		return rs;
	}

	return rs;
}

static int smb_batt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;

	Is_enable_fast_charging = 0;
	Is_full_charging = 0;
	pre_USBIN_mode = NONE;

	wake_lock_init(&main_wlock, WAKE_LOCK_SUSPEND, "acer_battery");

	current_time = current_kernel_time();
	change_time = current_time;

	smb_pdata = kzalloc(sizeof(*smb_pdata), GFP_KERNEL);
	if (smb_pdata == NULL) {
		pr_err("[SMB]smb_pdata kzalloc error\n");
		return -ENOMEM;
	}

	private_smb136_client = client;

	/* smem_alloc acer_smem_flag_t */
	acer_smem_flag = (acer_smem_flag_t *)(smem_alloc(SMEM_ID_VENDOR0, sizeof(acer_smem_flag_t)));
	if (acer_smem_flag == NULL) {
		pr_err("[SMB]smem_alloc SMEM_ID_VENDOR0 error\n");
		return -1;
	}
	acer_smem_flag->acer_os_pwr_state = ACER_OS_NORMAL_MODE;
	acer_hw_version = acer_smem_flag->acer_hw_version;

	/* check i2c functionality is workable */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[SMB]i2c_check_functionality error\n");
		return -ENOTSUPP;
	}

	res = smb_allow_volatile_write();
	if (res == -1) {
		pr_err("[SMB]SMB_allow_volatile_write error\n");
		return res;
	}

	res = smb_input_current_limit();
	if (res == -1) {
		pr_err("[SMB]smb_input_current_limit error\n");
		return res;
	}

	res = smb_set_complete_charger_timeout(minute_764);
	if (res == -1) {
		pr_err("[SMB]smb_set_complete_charger_timeout error\n");
		return res;
	}

	res = smb_set_AICL_detection_threshold(voltage_4250);
	if (res == -1) {
		pr_err("[SMB]smb_set_AICL_detection_threshold error\n");
		return res;
	}

	if (acer_hw_version == ACER_HW_VERSION_EVT) {
		res = smb_set_STAT_pin();
		if (res == -1) {
			pr_err("[SMB]SMB_set_STAT_pin error\n");
			return res;
		}
	}

	res = msm_batt_init_rpc();
	if (res < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, res);
		msm_batt_cleanup();
		return res;
	}

	/* polling to get battery and charger status */
	INIT_WORK(&smb_pdata->work_for_polling, battery_work_func);

	/* set polling interval */
	setup_timer(&smb_pdata->polling_timer, polling_timer_func, 0);
	mod_timer(&smb_pdata->polling_timer, jiffies + msecs_to_jiffies(POLLING_TIMER));

	return 0;
}

static int smb_batt_remove(struct i2c_client *client)
{
	i2c_del_driver(&smb_batt_driver);
	return 0;
}

static const struct i2c_device_id smb_batt_id[] = {
	{ SMB_BATTERY_DRIVER, 0 },
	{ }
};

static struct i2c_driver smb_batt_driver = {
	.probe		= smb_batt_probe,
	.remove		= __devexit_p(smb_batt_remove),
	.id_table	= smb_batt_id,
	.driver		= {
		.name = SMB_BATTERY_DRIVER,
	},
};
#endif

static int __init msm_batt_init(void)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

#ifdef CONFIG_MACH_ACER_A4
	rc = i2c_add_driver(&smb_batt_driver);
	if (rc) {
		pr_err("%s: Driver Initialization failed\n", __FILE__);
		return rc;
	}
#else
	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
#endif
	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	return 0;
}

static void __exit msm_batt_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
#ifdef CONFIG_MACH_ACER_A4
	i2c_del_driver(&smb_batt_driver);
	module_use_sys_delete(msm_batt_info.msm_psy_batt);
#endif
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");
