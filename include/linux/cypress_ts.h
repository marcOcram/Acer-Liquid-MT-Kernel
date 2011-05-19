/* include/linux/cy8c_tmg_ts.c
 *
 * Copyright (C) 2007-2008 HTC Corporation.
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

#ifndef CYPRESS_I2C_H
#define CYPRESS_I2C_H

#include <linux/types.h>

#define CYPRESS_TS_DRIVER_NAME "cypress-ts"

struct cypress_i2c_platform_data {
	uint abs_x_min;
	uint abs_x_max;
	uint abs_y_min;
	uint abs_y_max;
	uint abs_pressure_min;
	uint abs_pressure_max;
	uint abs_width_min;
	uint abs_width_max;
	uint points_max;	/* max support points */
	uint y_max;		/* max y, include virtual key */
	int irqflags;
	int (*hw_init)(int on);
	int (*power)(int ch);
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_ISSP
	bool (*enable_fw_update)(void);
#endif
};

enum {
	TS_VDD_POWER_OFF,
	TS_VDD_POWER_ON,
	TS_RESET,
};

enum {
	SUSPEND_STATE,
	INIT_STATE,
};

extern void download_firmware_main(char *);

#endif

