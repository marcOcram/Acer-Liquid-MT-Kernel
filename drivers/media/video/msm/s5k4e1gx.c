/*
Copyright (c) 2010, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Alternatively, and instead of the terms immediately above, this
software may be relicensed by the recipient at their option under the
terms of the GNU General Public License version 2 ("GPL") and only
version 2.  If the recipient chooses to relicense the software under
the GPL, then the recipient shall replace all of the text immediately
above and including this paragraph with the text immediately below
and between the words START OF ALTERNATE GPL TERMS and END OF
ALTERNATE GPL TERMS and such notices and license terms shall apply
INSTEAD OF the notices and licensing terms given above.

START OF ALTERNATE GPL TERMS

Copyright (c) 2010, Code Aurora Forum. All rights reserved.

This software was originally licensed under the Code Aurora Forum
Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
under the terms thereof by a recipient under the General Public
License Version 2.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

END OF ALTERNATE GPL TERMS
*/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "s5k4e1gx.h"
#include <mach/vreg.h>
#include "led_adp1650.h"

#define S5K4E1GX_REG_MODEL_ID                0x0000
#define S5K4E1GX_MODEL_ID                    0x4E10

#define S5K4E1GX_REG_GROUP_PARAMETER_HOLD    0x0104
#define S5K4E1GX_GROUP_PARAMETER_HOLD        0x01
#define S5K4E1GX_GROUP_PARAMETER_UNHOLD      0x00

#define REG_TEST_PATTERN_MODE                0x0601

#define S5K4E1GX_AF_I2C_ADDR                 0x18
#define S5K4E1GX_STEPS_NEAR_TO_CLOSEST_INF   50
#define S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR     28
#define S5K4E1GX_SW_DAMPING_STEP             10
#define S5K4E1GX_SW_DAMPING_STEP2            4
#define S5K4E1GX_MAX_FPS                     20

#define S5K4E1GX_MAX_SNAPSHOT_EXP_LC         3961

struct s5k4e1gx_work {
	struct work_struct work;
};
static struct s5k4e1gx_work *s5k4e1gx_sensorw;
static struct i2c_client *s5k4e1gx_client;

struct adp1560_work {
	struct work_struct work;
};
static struct adp1560_work *adp1560_flash;
static struct i2c_client *adp1560_client;
int cam_strobe = -1;

static uint16_t s5k4e1gx_pos_tbl[S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR+1];

struct s5k4e1gx_ctrl {
	const struct msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider; /* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */
	uint16_t curr_step_pos;
	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
};

static struct s5k4e1gx_ctrl *s5k4e1gx_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(s5k4e1gx_wait_queue);
DEFINE_MUTEX(s5k4e1gx_mutex);

static int32_t adp1560_i2c_txdata(unsigned short saddr, unsigned char *txdata,
					int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};

	if (i2c_transfer(adp1560_client->adapter, msg, 1) < 0) {
		pr_err("adp1560_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t adp1560_i2c_write_b(unsigned short saddr, unsigned short baddr,
					unsigned short bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	rc = adp1560_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		pr_err("i2c_write failed, saddr = 0x%x, addr = 0x%x,\
			 val =0x%x!\n", saddr, baddr, bdata);

	return rc;
}

static int32_t set_flashic_current(uint32_t flashc, uint32_t torchc)
{
	int32_t rc = -EIO;

	rc = adp1560_i2c_write_b(adp1560_client->addr,
				 ADP1650_REG_CURRENT_SET,
				 (flashc << ADP1650_I_FL_SHIFT) | torchc);
	if (rc < 0)
		pr_err("%s: Set flash IC current failed\n", __func__);

	return rc;
}

int32_t flash_mode_control(int ctrl)
{
	int32_t rc = -EIO;
	unsigned short reg = 0, data = 0;

	reg = ADP1650_REG_OUTPUT_MODE;
	data = (ADP1650_IL_PEAK_2P75A << ADP1650_IL_PEAK_SHIFT) |
	       (ADP1650_STR_LV_LEVEL_SENSITIVE << ADP1650_STR_LV_SHIFT) |
	       (ADP1650_FREQ_FB_1P5MHZ_NOT_ALLOWED << ADP1650_FREQ_FB_SHIFT) |
	       (ADP1650_STR_MODE_SW << ADP1650_STR_MODE_SHIFT);

	switch (ctrl) {
	case MSM_CAMERA_LED_OFF:
		/* Disable flash light output */
		data |= ADP1650_OUTPUT_EN_OFF << ADP1650_OUTPUT_EN_SHIFT;
		data |= ADP1650_LED_MODE_STANDBY;
		rc = adp1560_i2c_write_b(adp1560_client->addr, reg, data);
		if (rc < 0)
			pr_err("%s: Disable flash light failed\n", __func__);
		break;

	case MSM_CAMERA_LED_HIGH:
		/* Enable flash light output at high current (1000 mA)*/
		set_flashic_current(ADP1650_I_FL_1000mA, ADP1650_I_TOR_100mA);
		data |= ADP1650_OUTPUT_EN_ON << ADP1650_OUTPUT_EN_SHIFT;
		data |= ADP1650_LED_MODE_FLASH;
		rc = adp1560_i2c_write_b(adp1560_client->addr, reg, data);
		if (rc < 0)
			pr_err("%s: Enable flash light failed\n", __func__);
		break;

	case MSM_CAMERA_LED_LOW:
		/* Enable flash light output at low current (300 mA)*/
		set_flashic_current(ADP1650_I_FL_300mA, ADP1650_I_TOR_100mA);
		data |= ADP1650_OUTPUT_EN_ON << ADP1650_OUTPUT_EN_SHIFT;
		data |= ADP1650_LED_MODE_FLASH;
		rc = adp1560_i2c_write_b(adp1560_client->addr, reg, data);
		if (rc < 0)
			pr_err("%s: Enable flash light failed\n", __func__);
		break;

	default:
		pr_err("%s: Illegal flash light parameter\n", __func__);
		break;
	}

	return rc;
}

int32_t torch_mode_control(int ctrl)
{
	int32_t rc = -EIO;
	unsigned short reg = 0, data = 0;

	reg = ADP1650_REG_OUTPUT_MODE;
	data = (ADP1650_IL_PEAK_2P75A << ADP1650_IL_PEAK_SHIFT) |
	       (ADP1650_STR_LV_LEVEL_SENSITIVE << ADP1650_STR_LV_SHIFT) |
	       (ADP1650_FREQ_FB_1P5MHZ_NOT_ALLOWED << ADP1650_FREQ_FB_SHIFT) |
	       (ADP1650_STR_MODE_SW << ADP1650_STR_MODE_SHIFT);

	switch (ctrl) {
	case 0:
		/* Disable torch light output */
		data |= ADP1650_OUTPUT_EN_OFF << ADP1650_OUTPUT_EN_SHIFT;
		data |= ADP1650_LED_MODE_STANDBY;
		rc = adp1560_i2c_write_b(adp1560_client->addr, reg, data);
		if (rc < 0)
			pr_err("%s: Disable torch light failed\n", __func__);
		break;

	case 1:
		/* Enable torch light output */
		set_flashic_current(ADP1650_I_FL_1000mA, ADP1650_I_TOR_100mA);
		data |= ADP1650_OUTPUT_EN_ON << ADP1650_OUTPUT_EN_SHIFT;
		data |= ADP1650_LED_MODE_ASSIST;
		rc = adp1560_i2c_write_b(adp1560_client->addr, reg, data);
		if (rc < 0)
			pr_err("%s: Enable torch light failed\n", __func__);
		break;

	default:
		pr_err("%s: Illegal torch light parameter\n", __func__);
		break;
	}

	return rc;
}

static int s5k4e1gx_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
	int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	if (i2c_transfer(s5k4e1gx_client->adapter, msgs, 2) < 0) {
		pr_err("s5k4e1gx_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4e1gx_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(s5k4e1gx_client->adapter, msg, 1) < 0) {
		pr_err("s5k4e1gx_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4e1gx_i2c_write_b(unsigned short saddr, unsigned short waddr,
	unsigned char bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = s5k4e1gx_i2c_txdata(saddr, buf, 3);
	if (rc < 0)
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);

	return rc;
}

static int32_t s5k4e1gx_i2c_write_table(
	struct s5k4e1gx_i2c_reg_conf *reg_cfg_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			reg_cfg_tbl->waddr, reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}

	return rc;
}

static int32_t s5k4e1gx_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k4e1gx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("s5k4e1gx_i2c_read failed!\n");

	return rc;
}

static int s5k4e1gx_probe_init_done(const struct msm_camera_sensor_info *data)
{
	/* Disable sensor */
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);

	/* Disable vcm */
	gpio_direction_output(data->vcm_pwd, 0);


	/* Disable flashic */
	gpio_direction_output(data->flash_data->flash_src->_fsrc.flashic_src.strobe_gpio, 0);
	gpio_direction_output(data->flash_data->flash_src->_fsrc.flashic_src.flash_en_gpio, 0);

	return 0;
}

static int s5k4e1gx_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	uint16_t chipid = 0;

	/* Enable sensor */
	rc = gpio_request(data->sensor_reset, "s5k4e1gx");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else
		goto init_probe_fail;

	mdelay(50);

	pr_err("s5k4e1gx_sensor_init(): reseting sensor.\n");

	pr_err("S5K4E1GX gen model_id = 0x%x\n", chipid);

	rc = s5k4e1gx_i2c_read_w(s5k4e1gx_client->addr,
		S5K4E1GX_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;

	if (chipid != S5K4E1GX_MODEL_ID) {
		pr_err("S5K4E1GX wrong model_id = 0x%x\n", chipid);
		rc = -ENODEV;
		goto init_probe_fail;
	}else
		pr_err("S5K4E1GX good model_id = 0x%x\n", chipid);

	/* Enable vcm */
	gpio_direction_output(data->vcm_pwd, 1);

	/* Enable adp1560 flash ic */
	gpio_direction_output(data->flash_data->flash_src->_fsrc.flashic_src.strobe_gpio, 0);
	gpio_direction_output(data->flash_data->flash_src->_fsrc.flashic_src.flash_en_gpio, 1);
	cam_strobe = data->flash_data->flash_src->_fsrc.flashic_src.strobe_gpio;

	goto init_probe_done;

init_probe_fail:
	s5k4e1gx_probe_init_done(data);
init_probe_done:
	return rc;
}

static int s5k4e1gx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k4e1gx_wait_queue);
	return 0;
}

static const struct i2c_device_id s5k4e1gx_i2c_id[] = {
	{ "s5k4e1gx", 0},
	{ }
};

static void s5k4e1gx_setup_af_tbl(void)
{
	int i;
	uint16_t s5k4e1gx_nl_region_boundary1 = 4;
	uint16_t s5k4e1gx_nl_region_boundary2 = 5;
	uint16_t s5k4e1gx_nl_region_code_per_step1 = 15;
	uint16_t s5k4e1gx_nl_region_code_per_step2 = 13;
	uint16_t s5k4e1gx_l_region_code_per_step = 9;
	uint16_t s5k4e1gx_initial_dac = 202;

	s5k4e1gx_pos_tbl[0] = 0;

	for (i = 1; i <= S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= 3) {
			s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] + s5k4e1gx_initial_dac/3;
		} else if (i <= s5k4e1gx_nl_region_boundary1) {
			s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] + s5k4e1gx_nl_region_code_per_step1;
		} else if (i <= s5k4e1gx_nl_region_boundary2) {
			s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] + s5k4e1gx_nl_region_code_per_step2;
		} else {
			s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] + s5k4e1gx_l_region_code_per_step;
		}
	}
}

static int s5k4e1gx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("s5k4e1gx_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	s5k4e1gx_sensorw = kzalloc(sizeof(struct s5k4e1gx_work), GFP_KERNEL);
	if (!s5k4e1gx_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k4e1gx_sensorw);
	s5k4e1gx_init_client(client);
	s5k4e1gx_client = client;

	mdelay(50);

	pr_err("s5k4e1gx_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("s5k4e1gx_probe failed! rc = %d\n", rc);
	return rc;
}

static struct i2c_driver s5k4e1gx_i2c_driver = {
	.id_table = s5k4e1gx_i2c_id,
	.probe  = s5k4e1gx_i2c_probe,
	.remove = __exit_p(s5k4e1gx_i2c_remove),
	.driver = {
		.name = "s5k4e1gx",
	},
};

static int adp1560_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("!!!Call adp1560_i2c_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	adp1560_flash = kzalloc(sizeof(struct adp1560_work), GFP_KERNEL);
	if (!adp1560_flash) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, adp1560_flash);
	adp1560_client = client;

	mdelay(50);

	pr_err("adp1560_i2c_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("adp1560_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static const struct i2c_device_id adp1560_i2c_id[] = {
	{"adp1560", 0},
	{}
};

static struct i2c_driver adp1560_i2c_driver = {
	.id_table = adp1560_i2c_id,
	.probe = adp1560_i2c_probe,
	.remove = __exit_p(adp1560_i2c_remove),
	.driver = {
		   .name = "adp1560",
		   },
};

static int32_t s5k4e1gx_test(enum msm_s_test_mode mo)
{
	int32_t rc = 0;

	if (mo == S_TEST_OFF)
		rc = 0;
	else
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			REG_TEST_PATTERN_MODE, (uint16_t)mo);

	return rc;
}

#ifdef CONFIG_MIPI_2LANE
struct s5k4e1gx_i2c_reg_conf tbl_preview[] = {
	{0x0100, 0x00},
	{0x3030, 0x06},

	// CDS timing setting
	{0x3000, 0x05},
	{0x3001, 0x03},
	{0x3002, 0x08},
	{0x3003, 0x09},
	{0x3004, 0x2E},
	{0x3005, 0x06},
	{0x3006, 0x34},
	{0x3007, 0x00},
	{0x3008, 0x3C},
	{0x3009, 0X3C},
	{0x300A, 0X28},
	{0x300B, 0X04},
	{0x300C, 0x0A},
	{0x300D, 0x02},
	{0x300F, 0x82},

	// CDS option setting
	{0x3010, 0x00},
	{0x3011, 0x4C},
	{0x3012, 0x30},
	{0x3013, 0xC0},
	{0x3014, 0x00},
	{0x3015, 0x00},
	{0x3016, 0x2C},
	{0x3017, 0x94},
	{0x3018, 0x78},
	{0x301B, 0x83},
	{0x301D, 0xD4},
	{0x3021, 0x02},
	{0x3022, 0x24},
	{0x3024, 0x40},
	{0x3027, 0x08},
	{0x3029, 0xC6},
	{0x30BC, 0x98},
	{0x302B, 0x01},

	// Pixel option setting
	{0x301C, 0x04},
	{0x30D8, 0x3F},

	// ADLC setting
	{0x3070, 0x5F},
	{0x3071, 0x00},
	{0x3080, 0x04},
	{0x3081, 0x38},

	// MIPI setting
	{0x30BD, 0x00},  //SEL_CCP[0]
	{0x3084, 0x15},  //SYNC Mode
	{0x30BE, 0x1A},  //M_PCLKDIV_AUTO[4], M_DIV_PCLK[3:0]
	{0x30C1, 0x01},  //pack video enable [0]
	{0x30EE, 0x02},  //DPHY enable [1]
	{0x3111, 0x86},  //Embedded data off [5]

	// For MIPI T8 T9 add by greg0809
	{0x30E3, 0x38},  //outif_mld_ulpm_rxinit_limit[15:8]
	{0x30E4, 0x40},  //outif_mld_ulpm_rxinit_limit[7:0]
	{0x3113, 0x70},  //outif_enable_time[15:8]
	{0x3114, 0x80},  //outif_enable_time[7:0]
	{0x3115, 0x7B},  //streaming_enalbe_time[15:8]
	{0x3116, 0xC0},  //streaming_enalbe_time[7:0]
	{0x30EE, 0x1A},  //[5:4]esc_ref_div, [3] dphy_ulps_auto, [1]dphy_enable

	// Integration setting
	{0x0202, 0x01},
	{0x0203, 0xFD},
	{0x0204, 0x00},
	{0x0205, 0x80},

	//Frame Length
	{0x0340, 0x03},
	{0x0341, 0xE0},

	// Line Length
	{0x0342, 0x0A},  //2738
	{0x0343, 0xB2},

	// PLL setting
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0x56},
	{0x30B5, 0x01},
	{0x30E2, 0x02},  //num lanes[1:0] = 2
	{0x30F1, 0x90},

	// MIPI size setting
	// 1304x980
	{0x30A9, 0x02},  //Horizontal Binning On
	{0x300E, 0xEB},  //Vertical Binning On
	{0x0387, 0x03},  //y_odd_inc 03(10b AVG)
	{0x0344, 0x00},  //x_addr_start 0
	{0x0345, 0x00},
	{0x0348, 0x0A},  //x_addr_end 2607
	{0x0349, 0x2F},
	{0x0346, 0x00},  //y_addr_start 0
	{0x0347, 0x00},
	{0x034A, 0x07},  //y_addr_end 1959
	{0x034B, 0xA7},
	{0x0380, 0x00},  //x_even_inc 1
	{0x0381, 0x01},
	{0x0382, 0x00},  //x_odd_inc 1
	{0x0383, 0x01},
	{0x0384, 0x00},  //y_even_inc 1
	{0x0385, 0x01},
	{0x0386, 0x00},  //y_odd_inc 3
	{0x0387, 0x03},
	{0x034C, 0x05},  //x_output_size 1304
	{0x034D, 0x18},
	{0x034E, 0x03},  //y_output_size 980
	{0x034F, 0xd4},
	{0x30BF, 0xAB},  //outif_enable[7], data_type[5:0](2Bh = bayer 10bit)
	{0x30C0, 0xA0},  //video_offset[7:4] 3260%12
	{0x30C8, 0x06},  //video_data_length 1600 = 1304 * 1.25
	{0x30C9, 0x5E}
};

struct s5k4e1gx_i2c_reg_conf tbl_snapshot[] = {
	{0x0100, 0x00},
	{0x3030, 0x06},

	// Analog Setting
	{0x3000, 0x05},
	{0x3001, 0x03},
	{0x3002, 0x08},
	{0x3003, 0x09},
	{0x3004, 0x2E},
	{0x3005, 0x06},
	{0x3006, 0x34},
	{0x3007, 0x00},
	{0x3008, 0x3C},
	{0x3009, 0x3C},
	{0x300A, 0x28},
	{0x300B, 0x04},
	{0x300C, 0x0A},
	{0x300D, 0x02},
	{0x300E, 0xE8},
	{0x300F, 0x82},
	{0x3010, 0x00},
	{0x3011, 0x4C},
	{0x3012, 0x30},
	{0x3013, 0xC0},
	{0x3014, 0x00},
	{0x3015, 0x00},
	{0x3016, 0x2C},
	{0x3017, 0x94},
	{0x3018, 0x78},
	{0x301B, 0x75},
	{0x301C, 0x04},
	{0x301D, 0xD4},
	{0x3021, 0x02},
	{0x3022, 0x24},
	{0x3024, 0x40},
	{0x3027, 0x08},
	{0x3029, 0xC6},
	{0x30BC, 0x98},
	{0x302B, 0x01},
	{0x30D8, 0x3F},

	// ADLC setting
	{0x3070, 0x5F},
	{0x3071, 0x00},
	{0x3080, 0x04},
	{0x3081, 0x38},

	// MIPI setting
	{0x30BD, 0x00},  //SEL_CCP[0]
	{0x3084, 0x15},  //SYNC Mode
	{0x30BE, 0x1A},  //M_PCLKDIV_AUTO[4], M_DIV_PCLK[3:0]
	{0x30C1, 0x01},  //pack video enable [0]
	{0x30EE, 0x02},  //DPHY enable [1]
	{0x3111, 0x86},  //Embedded data off [5]

	// For MIPI T8 T9 add by greg0809
	{0x30E3, 0x38},  //outif_mld_ulpm_rxinit_limit[15:8]
	{0x30E4, 0x40},  //outif_mld_ulpm_rxinit_limit[7:0]
	{0x3113, 0x70},  //outif_enable_time[15:8]
	{0x3114, 0x80},  //outif_enable_time[7:0]
	{0x3115, 0x7B},  //{0xtreaming_enalbe_time[15:8]
	{0x3116, 0xC0},  //streaming_enalbe_time[7:0]
	{0x30EE, 0x1A},  //[5:4]esc_ref_div, [3] dphy_ulps_auto, [1]dphy_enable

	// Integration setting ...
	{0x0202, 0x04},
	{0x0203, 0x12},
	{0x0204, 0x00},
	{0x0205, 0x80},

	// Frame Length
	{0x0340, 0x07},
	{0x0341, 0xB4},

	// Line Length
	{0x0342, 0x0A},  //2738
	{0x0343, 0xB2},

	// PLL setting
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0x56},
	{0x30B5, 0x01},
	{0x30E2, 0x02},  //num lanes[1:0] = 2
	{0x30F1, 0x90},

	// MIPI Size Setting
	{0x30A9, 0x03},  //Horizontal Binning Off
	{0x300E, 0xE8},  //Vertical Binning Off
	{0x0387, 0x01},  //y_odd_inc
	{0x034C, 0x0A},  //x_output size
	{0x034D, 0x30},
	{0x034E, 0x07},  //y_output size
	{0x034F, 0xA8},

	{0x30BF, 0xAB},  //outif_enable[7], data_type[5:0](2Bh = bayer 10bit)
	{0x30C0, 0x80},  //video_offset[7:4] 3260%12
	{0x30C8, 0x0C},  //video_data_length 3260 = 2608 * 1.25
	{0x30C9, 0xBC},

	{0x30e3, 0x19},
	{0x30e4, 0x64},
	{0x30e5, 0xf0},
	{0x30e3, 0x00}
};

static int32_t s5k4e1gx_setting(enum msm_s_reg_update rupdate,
		enum msm_s_setting rt)
{
	int32_t rc = 0;
	struct msm_camera_csi_params *s5k4e1gx_csi_params =
		kzalloc(sizeof(struct msm_camera_csi_params), GFP_KERNEL);

	CDBG("s5k4e1gx_setting: lane number = %d\n", s5k4e1gx_reg_pat[rt].outif_num_of_lanes);
	switch (rupdate) {
	case S_UPDATE_PERIODIC:
		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
			/* config mipi csi controller */
			CDBG("Config csi controller \n");
			s5k4e1gx_csi_params->data_format = CSI_10BIT;
			s5k4e1gx_csi_params->lane_cnt = 2;
			s5k4e1gx_csi_params->lane_assign = 0xe4;
			s5k4e1gx_csi_params->dpcm_scheme = 0;
			s5k4e1gx_csi_params->settle_cnt = 20;
			rc = msm_camio_csi_config(s5k4e1gx_csi_params);
			if (rc < 0)
				CDBG(" config csi controller failed \n");

			if (rt == S_RES_PREVIEW) {
				rc = s5k4e1gx_i2c_write_table(&tbl_preview[0],
							ARRAY_SIZE(tbl_preview));
			} else {
				rc = s5k4e1gx_i2c_write_table(&tbl_snapshot[0],
							ARRAY_SIZE(tbl_snapshot));
			}
			if (rc < 0) {
				CDBG("Write preview/snapshot mode registers failed \n");
				return rc;
			}

			/* Streaming ON */
			rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, S5K4E1GX_REG_MODE_SELECT, S5K4E1GX_MODE_SELECT_STREAM);
			if (rc < 0) {
				CDBG("Write streaming on failed\n");
				return rc;
			}

			rc = s5k4e1gx_test(s5k4e1gx_ctrl->set_test);
			if (rc < 0)
				return rc;
		}
		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
			if (rt == S_RES_PREVIEW) {
				rc = s5k4e1gx_i2c_write_table(&tbl_preview[0],
							ARRAY_SIZE(tbl_preview));
			} else {
				rc = s5k4e1gx_i2c_write_table(&tbl_snapshot[0],
							ARRAY_SIZE(tbl_snapshot));
			}
			if (rc < 0) {
				CDBG("Write initial registers failed\n");
				return rc;
			}

			// Only init sensor, not turn on streaming

			/* reset fps_divider */
			s5k4e1gx_ctrl->fps_divider = 1 * 0x0400;
		}
		break; /* case REG_INIT: */

	default:
		rc = -EINVAL;
		break;
	} /* switch (rupdate) */

	return rc;
}
#else
static int32_t s5k4e1gx_setting(enum msm_s_reg_update rupdate,
		enum msm_s_setting rt)
{
	int32_t rc = 0;

	switch (rupdate) {
	case S_UPDATE_PERIODIC:
		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {

			struct s5k4e1gx_i2c_reg_conf tbl_1[] = {
				{S5K4E1GX_REG_MODE_SELECT,          S5K4E1GX_MODE_SELECT_SW_STANDBY},

				/* CDS timing settings */
				/* Reserved registers */
				{REG_LD_START,                      s5k4e1gx_reg_pat[rt].ld_start},
				{REG_SL_START,                      s5k4e1gx_reg_pat[rt].sl_start},
				{REG_RX_START,                      s5k4e1gx_reg_pat[rt].rx_start},
				{REG_CDS_START,                     s5k4e1gx_reg_pat[rt].cds_start},
				{REG_SMP_WIDTH,                     s5k4e1gx_reg_pat[rt].smp_width},
				{REG_AZ_WIDTH,                      s5k4e1gx_reg_pat[rt].az_width},
				{REG_S1R_WIDTH,                     s5k4e1gx_reg_pat[rt].s1r_width},
				{REG_TX_START,                      s5k4e1gx_reg_pat[rt].tx_start},
				{REG_TX_WIDTH,                      s5k4e1gx_reg_pat[rt].tx_width},
				{REG_STX_WIDTH,                     s5k4e1gx_reg_pat[rt].stx_width},
				{REG_DTX_WIDTH,                     s5k4e1gx_reg_pat[rt].dtx_width},
				{REG_RMP_RST_START,                 s5k4e1gx_reg_pat[rt].rmp_rst_start},
				{REG_RMP_SIG_START,                 s5k4e1gx_reg_pat[rt].rmp_sig_start},
				{REG_RMP_LAT,                       s5k4e1gx_reg_pat[rt].rmp_lat},
				{REG_V_BINNING,                     s5k4e1gx_reg_pat[rt].v_binning_1},
				{REG_300F,                          s5k4e1gx_reg_pat[rt].reg_300F},
				{REG_301B,                          s5k4e1gx_reg_pat[rt].reg_301B},

				{REG_SMP_EN,                        s5k4e1gx_reg_pat[rt].smp_en},
				{REG_RST_MX,                        s5k4e1gx_reg_pat[rt].rst_mx},
				{REG_3029,                          s5k4e1gx_reg_pat[rt].reg_3029},
				{REG_SIG_OFFSET1,                   s5k4e1gx_reg_pat[rt].sig_offset1},
				{REG_RST_OFFSET1,                   s5k4e1gx_reg_pat[rt].rst_offset1},
				{REG_SIG_OFFSET2,                   s5k4e1gx_reg_pat[rt].sig_offset2},
				{REG_RST_OFFSET2,                   s5k4e1gx_reg_pat[rt].rst_offset2},
				{REG_ADC_SAT,                       s5k4e1gx_reg_pat[rt].adc_sat},
				{REG_RMP_INIT,                      s5k4e1gx_reg_pat[rt].rmp_init},
				{REG_RMP_OPTION,                    s5k4e1gx_reg_pat[rt].rmp_option},

				{REG_CLP_LEVEL,                     s5k4e1gx_reg_pat[rt].clp_level},
				{REG_INRUSH_CTRL,                   s5k4e1gx_reg_pat[rt].inrush_ctrl},
				{REG_PUMP_RING_OSC,                 s5k4e1gx_reg_pat[rt].pump_ring_osc},
				{REG_PIX_VOLTAGE,                   s5k4e1gx_reg_pat[rt].pix_voltage},
				{REG_NTG_VOLTAGE,                   s5k4e1gx_reg_pat[rt].ntg_voltage},

				{REG_PIXEL_BIAS,                    s5k4e1gx_reg_pat[rt].pixel_bias},
				{REG_ALL_TX_OFF,                    s5k4e1gx_reg_pat[rt].all_tx_off},
				{REG_302B,                          s5k4e1gx_reg_pat[rt].reg_302B},

				/* ADLC SETTING */
				{REG_L_ADLC_BPR,                    s5k4e1gx_reg_pat[rt].l_adlc_bpr},
				{REG_F_L_ADLC_MAX,                  s5k4e1gx_reg_pat[rt].f_l_adlc_max},
				{REG_F_ADLC_FILTER_A,               s5k4e1gx_reg_pat[rt].f_adlc_filter_a},
				{REG_F_ADLC_FILTER_B,               s5k4e1gx_reg_pat[rt].f_adlc_filter_b},

				{REG_SEL_CCP,                       s5k4e1gx_reg_pat[rt].sel_ccp},
				{REG_SYNC_MODE,                     s5k4e1gx_reg_pat[rt].sync_mode},
				{REG_M_PCLK_DIV,                    s5k4e1gx_reg_pat[rt].m_pclk_div},
				{REG_PACK_VIDEO_ENABLE,             s5k4e1gx_reg_pat[rt].pack_video_enable},
				{REG_DPHY_ENABLE,                   s5k4e1gx_reg_pat[rt].dphy_enable},
				{REG_EMBEDDED_DATA_OFF,             s5k4e1gx_reg_pat[rt].embedded_data_off},

				{REG_COARSE_INTEGRATION_TIME,       s5k4e1gx_reg_pat[rt].coarse_integration_time},
				{REG_COARSE_INTEGRATION_TIME_LSB,   s5k4e1gx_reg_pat[rt].coarse_integration_time_lsb},
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, s5k4e1gx_reg_pat[rt].analogue_gain_code_global_msb},
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, s5k4e1gx_reg_pat[rt].analogue_gain_code_global_lsb},

				/* Frame Fotmat */
				{REG_FRAME_LENGTH_LINES_MSB,        s5k4e1gx_reg_pat[rt].frame_length_lines_msb},
				{REG_FRAME_LENGTH_LINES_LSB,        s5k4e1gx_reg_pat[rt].frame_length_lines_lsb},
				{REG_LINE_LENGTH_PCK_MSB,           s5k4e1gx_reg_pat[rt].line_length_pck_msb},
				{REG_LINE_LENGTH_PCK_LSB,           s5k4e1gx_reg_pat[rt].line_length_pck_lsb},


				/* PLL Registers */
				{REG_PRE_PLL_CLK_DIV,               s5k4e1gx_reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER_MSB,            s5k4e1gx_reg_pat[rt].pll_multiplier_msb},
				{REG_PLL_MULTIPLIER_LSB,            s5k4e1gx_reg_pat[rt].pll_multiplier_lsb},
				{REG_VT_SYS_CLK_DIV,                s5k4e1gx_reg_pat[rt].vt_sys_clk_div},
				{REG_OUTIF_NUM_OF_LANES,            s5k4e1gx_reg_pat[rt].outif_num_of_lanes},
				{REG_DPHY_BAND_CTRL,                s5k4e1gx_reg_pat[rt].dphy_band_ctrl},

				/* Reserved register */
				{REG_H_BINNING,                     s5k4e1gx_reg_pat[rt].h_binning},
				{REG_V_BINNING,                     s5k4e1gx_reg_pat[rt].v_binning_2},
				{REG_Y_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_lsb_1},

				{REG_X_ADDR_START_MSB,              s5k4e1gx_reg_pat[rt].x_addr_start_msb},
				{REG_X_ADDR_START_LSB,              s5k4e1gx_reg_pat[rt].x_addr_start_lsb},
				{REG_X_ADDR_END_MSB,                s5k4e1gx_reg_pat[rt].x_addr_end_msb},
				{REG_X_ADDR_END_LSB,                s5k4e1gx_reg_pat[rt].x_addr_end_lsb},
				{REG_Y_ADDR_START_MSB,              s5k4e1gx_reg_pat[rt].y_addr_start_msb},
				{REG_Y_ADDR_START_LSB,              s5k4e1gx_reg_pat[rt].y_addr_start_lsb},
				{REG_Y_ADDR_END_MSB,                s5k4e1gx_reg_pat[rt].y_addr_end_msb},
				{REG_Y_ADDR_END_LSB,                s5k4e1gx_reg_pat[rt].y_addr_end_lsb},

				/* Binning */
				{REG_X_EVEN_INC_MSB,                s5k4e1gx_reg_pat[rt].x_even_inc_msb},
				{REG_X_EVEN_INC_LSB,                s5k4e1gx_reg_pat[rt].x_even_inc_lsb},
				{REG_X_ODD_INC_MSB,                 s5k4e1gx_reg_pat[rt].x_odd_inc_msb},
				{REG_X_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].x_odd_inc_lsb},
				{REG_Y_EVEN_INC_MSB,                s5k4e1gx_reg_pat[rt].y_even_inc_msb},
				{REG_Y_EVEN_INC_LSB,                s5k4e1gx_reg_pat[rt].y_even_inc_lsb},
				{REG_Y_ODD_INC_MSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_msb},
				{REG_Y_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_lsb_2},

				/* Output Size */
				{REG_X_OUTPUT_SIZE_MSB,             s5k4e1gx_reg_pat[rt].x_output_size_msb},
				{REG_X_OUTPUT_SIZE_LSB,             s5k4e1gx_reg_pat[rt].x_output_size_lsb},
				{REG_Y_OUTPUT_SIZE_MSB,             s5k4e1gx_reg_pat[rt].y_output_size_msb},
				{REG_Y_OUTPUT_SIZE_LSB,             s5k4e1gx_reg_pat[rt].y_output_size_lsb},

				{REG_OUTIF_VIDEO_DATA_TYPE1,        s5k4e1gx_reg_pat[rt].outif_video_data_typel},
				{REG_OUTIF_OFFSET_VIDEO,            s5k4e1gx_reg_pat[rt].outif_offset_video},
				{REG_VIDEO_DATA_LENGTH_MSB,         s5k4e1gx_reg_pat[rt].video_data_length_msb},
				{REG_VIDEO_DATA_LENGTH_LSB,         s5k4e1gx_reg_pat[rt].video_data_length_lsb}
			};

			rc = s5k4e1gx_i2c_write_table(&tbl_1[0],
				ARRAY_SIZE(tbl_1));
			if (rc < 0)
				return rc;

#if defined(A4_APPLY_SENSOR_SHADING)
			/* switch resolution for lens shading correction */
			if (rt == S_RES_PREVIEW)
				rc = s5k4e1gx_i2c_write_table(&sub_sampling_shading[0], ARRAY_SIZE(sub_sampling_shading));
			else if (rt == S_RES_CAPTURE)
				rc = s5k4e1gx_i2c_write_table(&full_size_shading[0], ARRAY_SIZE(full_size_shading));
			if (rc < 0)
				return rc;
#endif

			/* Streaming ON */
			rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, S5K4E1GX_REG_MODE_SELECT, S5K4E1GX_MODE_SELECT_STREAM);
			if (rc < 0)
				return rc;

			rc = s5k4e1gx_test(s5k4e1gx_ctrl->set_test);
			if (rc < 0)
				return rc;
		}
		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {

			struct s5k4e1gx_i2c_reg_conf tbl_3[] = {
				{S5K4E1GX_REG_MODE_SELECT,          S5K4E1GX_MODE_SELECT_SW_STANDBY},
				{S5K4E1GX_REG_SOFTWARE_RESET,       S5K4E1GX_SOFTWARE_RESET},

				/* CDS timing settings */
				/* Reserved registers */
				{REG_LD_START,                      s5k4e1gx_reg_pat[rt].ld_start},
				{REG_SL_START,                      s5k4e1gx_reg_pat[rt].sl_start},
				{REG_RX_START,                      s5k4e1gx_reg_pat[rt].rx_start},
				{REG_CDS_START,                     s5k4e1gx_reg_pat[rt].cds_start},
				{REG_SMP_WIDTH,                     s5k4e1gx_reg_pat[rt].smp_width},
				{REG_AZ_WIDTH,                      s5k4e1gx_reg_pat[rt].az_width},
				{REG_S1R_WIDTH,                     s5k4e1gx_reg_pat[rt].s1r_width},
				{REG_TX_START,                      s5k4e1gx_reg_pat[rt].tx_start},
				{REG_TX_WIDTH,                      s5k4e1gx_reg_pat[rt].tx_width},
				{REG_STX_WIDTH,                     s5k4e1gx_reg_pat[rt].stx_width},
				{REG_DTX_WIDTH,                     s5k4e1gx_reg_pat[rt].dtx_width},
				{REG_RMP_RST_START,                 s5k4e1gx_reg_pat[rt].rmp_rst_start},
				{REG_RMP_SIG_START,                 s5k4e1gx_reg_pat[rt].rmp_sig_start},
				{REG_RMP_LAT,                       s5k4e1gx_reg_pat[rt].rmp_lat},
				{REG_V_BINNING,                     s5k4e1gx_reg_pat[rt].v_binning_1},
				{REG_300F,                          s5k4e1gx_reg_pat[rt].reg_300F},
				{REG_301B,                          s5k4e1gx_reg_pat[rt].reg_301B},


				{REG_SMP_EN,                        s5k4e1gx_reg_pat[rt].smp_en},
				{REG_RST_MX,                        s5k4e1gx_reg_pat[rt].rst_mx},
				{REG_3029,                          s5k4e1gx_reg_pat[rt].reg_3029},
				{REG_SIG_OFFSET1,                   s5k4e1gx_reg_pat[rt].sig_offset1},
				{REG_RST_OFFSET1,                   s5k4e1gx_reg_pat[rt].rst_offset1},
				{REG_SIG_OFFSET2,                   s5k4e1gx_reg_pat[rt].sig_offset2},
				{REG_RST_OFFSET2,                   s5k4e1gx_reg_pat[rt].rst_offset2},
				{REG_ADC_SAT,                       s5k4e1gx_reg_pat[rt].adc_sat},
				{REG_RMP_INIT,                      s5k4e1gx_reg_pat[rt].rmp_init},
				{REG_RMP_OPTION,                    s5k4e1gx_reg_pat[rt].rmp_option},

				{REG_CLP_LEVEL,                     s5k4e1gx_reg_pat[rt].clp_level},
				{REG_INRUSH_CTRL,                   s5k4e1gx_reg_pat[rt].inrush_ctrl},
				{REG_PUMP_RING_OSC,                 s5k4e1gx_reg_pat[rt].pump_ring_osc},
				{REG_PIX_VOLTAGE,                   s5k4e1gx_reg_pat[rt].pix_voltage},
				{REG_NTG_VOLTAGE,                   s5k4e1gx_reg_pat[rt].ntg_voltage},

				{REG_PIXEL_BIAS,                    s5k4e1gx_reg_pat[rt].pixel_bias},
				{REG_ALL_TX_OFF,                    s5k4e1gx_reg_pat[rt].all_tx_off},
				{REG_302B,                          s5k4e1gx_reg_pat[rt].reg_302B},

				/* ADLC SETTING */
				{REG_L_ADLC_BPR,                    s5k4e1gx_reg_pat[rt].l_adlc_bpr},
				{REG_F_L_ADLC_MAX,                  s5k4e1gx_reg_pat[rt].f_l_adlc_max},
				{REG_F_ADLC_FILTER_A,               s5k4e1gx_reg_pat[rt].f_adlc_filter_a},
				{REG_F_ADLC_FILTER_B,               s5k4e1gx_reg_pat[rt].f_adlc_filter_b},

				{REG_SEL_CCP,                       s5k4e1gx_reg_pat[rt].sel_ccp},
				{REG_SYNC_MODE,                     s5k4e1gx_reg_pat[rt].sync_mode},
				{REG_M_PCLK_DIV,                    s5k4e1gx_reg_pat[rt].m_pclk_div},
				{REG_PACK_VIDEO_ENABLE,             s5k4e1gx_reg_pat[rt].pack_video_enable},
				{REG_DPHY_ENABLE,                   s5k4e1gx_reg_pat[rt].dphy_enable},
				{REG_EMBEDDED_DATA_OFF,             s5k4e1gx_reg_pat[rt].embedded_data_off},

				{REG_COARSE_INTEGRATION_TIME,       s5k4e1gx_reg_pat[rt].coarse_integration_time},
				{REG_COARSE_INTEGRATION_TIME_LSB,   s5k4e1gx_reg_pat[rt].coarse_integration_time_lsb},
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, s5k4e1gx_reg_pat[rt].analogue_gain_code_global_msb},
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, s5k4e1gx_reg_pat[rt].analogue_gain_code_global_lsb},

				/* Frame Fotmat */
				{REG_FRAME_LENGTH_LINES_MSB,        s5k4e1gx_reg_pat[rt].frame_length_lines_msb},
				{REG_FRAME_LENGTH_LINES_LSB,        s5k4e1gx_reg_pat[rt].frame_length_lines_lsb},
				{REG_LINE_LENGTH_PCK_MSB,           s5k4e1gx_reg_pat[rt].line_length_pck_msb},
				{REG_LINE_LENGTH_PCK_LSB,           s5k4e1gx_reg_pat[rt].line_length_pck_lsb},

				/* PLL Registers */
				{REG_PRE_PLL_CLK_DIV,               s5k4e1gx_reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER_MSB,            s5k4e1gx_reg_pat[rt].pll_multiplier_msb},
				{REG_PLL_MULTIPLIER_LSB,            s5k4e1gx_reg_pat[rt].pll_multiplier_lsb},
				{REG_VT_SYS_CLK_DIV,                s5k4e1gx_reg_pat[rt].vt_sys_clk_div},
				{REG_OUTIF_NUM_OF_LANES,            s5k4e1gx_reg_pat[rt].outif_num_of_lanes},
				{REG_DPHY_BAND_CTRL,                s5k4e1gx_reg_pat[rt].dphy_band_ctrl},

				/* Reserved register */
				{REG_H_BINNING,                     s5k4e1gx_reg_pat[rt].h_binning},
				{REG_V_BINNING,                     s5k4e1gx_reg_pat[rt].v_binning_2},
				{REG_Y_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_lsb_1},

				{REG_X_ADDR_START_MSB,              s5k4e1gx_reg_pat[rt].x_addr_start_msb},
				{REG_X_ADDR_START_LSB,              s5k4e1gx_reg_pat[rt].x_addr_start_lsb},
				{REG_X_ADDR_END_MSB,                s5k4e1gx_reg_pat[rt].x_addr_end_msb},
				{REG_X_ADDR_END_LSB,                s5k4e1gx_reg_pat[rt].x_addr_end_lsb},
				{REG_Y_ADDR_START_MSB,              s5k4e1gx_reg_pat[rt].y_addr_start_msb},
				{REG_Y_ADDR_START_LSB,              s5k4e1gx_reg_pat[rt].y_addr_start_lsb},
				{REG_Y_ADDR_END_MSB,                s5k4e1gx_reg_pat[rt].y_addr_end_msb},
				{REG_Y_ADDR_END_LSB,                s5k4e1gx_reg_pat[rt].y_addr_end_lsb},

				/* Binning */
				{REG_X_EVEN_INC_MSB,                s5k4e1gx_reg_pat[rt].x_even_inc_msb},
				{REG_X_EVEN_INC_LSB,                s5k4e1gx_reg_pat[rt].x_even_inc_lsb},
				{REG_X_ODD_INC_MSB,                 s5k4e1gx_reg_pat[rt].x_odd_inc_msb},
				{REG_X_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].x_odd_inc_lsb},
				{REG_Y_EVEN_INC_MSB,                s5k4e1gx_reg_pat[rt].y_even_inc_msb},
				{REG_Y_EVEN_INC_LSB,                s5k4e1gx_reg_pat[rt].y_even_inc_lsb},
				{REG_Y_ODD_INC_MSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_msb},
				{REG_Y_ODD_INC_LSB,                 s5k4e1gx_reg_pat[rt].y_odd_inc_lsb_2},

				/* Output Size */
				{REG_X_OUTPUT_SIZE_MSB,             s5k4e1gx_reg_pat[rt].x_output_size_msb},
				{REG_X_OUTPUT_SIZE_LSB,             s5k4e1gx_reg_pat[rt].x_output_size_lsb},
				{REG_Y_OUTPUT_SIZE_MSB,             s5k4e1gx_reg_pat[rt].y_output_size_msb},
				{REG_Y_OUTPUT_SIZE_LSB,             s5k4e1gx_reg_pat[rt].y_output_size_lsb},

				{REG_OUTIF_VIDEO_DATA_TYPE1,        s5k4e1gx_reg_pat[rt].outif_video_data_typel},
				{REG_OUTIF_OFFSET_VIDEO,            s5k4e1gx_reg_pat[rt].outif_offset_video},
				{REG_VIDEO_DATA_LENGTH_MSB,         s5k4e1gx_reg_pat[rt].video_data_length_msb},
				{REG_VIDEO_DATA_LENGTH_LSB,         s5k4e1gx_reg_pat[rt].video_data_length_lsb}
			};

			rc = s5k4e1gx_i2c_write_table(&tbl_3[0],
				ARRAY_SIZE(tbl_3));
			if (rc < 0)
				return rc;

#if defined(A4_APPLY_SENSOR_SHADING)
			/* lens shading correction */
			rc = s5k4e1gx_i2c_write_table(&shading_setting[0], ARRAY_SIZE(shading_setting));
			if (rc < 0)
				return rc;
#endif

			/*Streaming ON*/
			rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
									  S5K4E1GX_REG_MODE_SELECT,
									  S5K4E1GX_MODE_SELECT_STREAM);
			if (rc < 0)
				return rc;

			/* reset fps_divider */
			s5k4e1gx_ctrl->fps_divider = 1 * 0x0400;
		}
		break; /* case REG_INIT: */

	default:
		rc = -EINVAL;
		break;
	} /* switch (rupdate) */

	return rc;
}
#endif

static int s5k4e1gx_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	uint32_t flash_current, torch_current;
	int32_t  rc;
#ifndef CONFIG_MIPI_2LANE
	struct msm_camera_csi_params *s5k4e1gx_csi_params =
		kzalloc(sizeof(struct msm_camera_csi_params), GFP_KERNEL);
#endif
	s5k4e1gx_ctrl = kzalloc(sizeof(struct s5k4e1gx_ctrl), GFP_KERNEL);
	if (!s5k4e1gx_ctrl) {
		pr_err("s5k4e1gx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	s5k4e1gx_ctrl->fps_divider = 1 * 0x00000400;
	s5k4e1gx_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k4e1gx_ctrl->set_test = S_TEST_OFF;
	s5k4e1gx_ctrl->prev_res = S_QTR_SIZE;
	s5k4e1gx_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		s5k4e1gx_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(24000000);
	mdelay(20);

	rc = s5k4e1gx_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail1;

#ifndef CONFIG_MIPI_2LANE
    /* config mipi csi controller */
	CDBG("s5k4e1gx_sensor_open_init: config csi controller \n");
	s5k4e1gx_csi_params->data_format = CSI_10BIT;
	s5k4e1gx_csi_params->lane_cnt = 1;
	s5k4e1gx_csi_params->lane_assign = 0xe4;
	s5k4e1gx_csi_params->dpcm_scheme = 0;
	s5k4e1gx_csi_params->settle_cnt = 7;
	rc = msm_camio_csi_config(s5k4e1gx_csi_params);
	if (rc < 0)
		CDBG(" config csi controller failed \n");
#endif

	if (s5k4e1gx_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k4e1gx_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = s5k4e1gx_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0) {
		pr_err("s5k4e1gx_setting failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* set up lens position table */
	s5k4e1gx_setup_af_tbl();

	/* Set flashic current */
	flash_current = data->flash_data->flash_src->_fsrc.flashic_src.flash_current;
	torch_current = data->flash_data->flash_src->_fsrc.flashic_src.torch_current;
	rc = set_flashic_current(ADP1650_I_FL_1000mA, ADP1650_I_TOR_100mA);
	goto init_done;

init_fail1:
	kfree(s5k4e1gx_ctrl);
init_done:
	return rc;
}

static int32_t s5k4e1gx_power_down(void)
{
	int32_t rc = 0;
	return rc;
}

static int s5k4e1gx_sensor_release(void)
{
	mutex_lock(&s5k4e1gx_mutex);

	s5k4e1gx_power_down();

	/* Disable sensor */
	gpio_direction_output(s5k4e1gx_ctrl->sensordata->sensor_reset, 0);
	gpio_free(s5k4e1gx_ctrl->sensordata->sensor_reset);

	/* Disable vcm */
	gpio_direction_output(s5k4e1gx_ctrl->sensordata->vcm_pwd, 0);


	/* Disable flashic */
	gpio_direction_output(s5k4e1gx_ctrl->sensordata->flash_data->flash_src->_fsrc.flashic_src.strobe_gpio, 0);
	gpio_direction_output(s5k4e1gx_ctrl->sensordata->flash_data->flash_src->_fsrc.flashic_src.flash_en_gpio, 0);

	kfree(s5k4e1gx_ctrl);
	s5k4e1gx_ctrl = NULL;

	pr_err("s5k4e1gx_release completed\n");

	mutex_unlock(&s5k4e1gx_mutex);
	return 0;
}

static void s5k4e1gx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
	uint32_t d1;
	uint32_t d2;

	d1 =
		(uint32_t)(
		((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
		0x00000400) /
		(s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l));

	d2 =
		(uint32_t)(
		((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p) *
		0x00000400) /
		 (s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p));


	divider = (uint32_t) (d1 * d2) / 0x00000400;
	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t)(fps * divider / 0x00000400);
}

static uint16_t s5k4e1gx_get_prev_lines_pf(void)
{
	return s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;
}

static uint16_t s5k4e1gx_get_prev_pixels_pl(void)
{
	return s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p;
}

static uint16_t s5k4e1gx_get_pict_lines_pf(void)
{
	return s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
		s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l;
}

static uint16_t s5k4e1gx_get_pict_pixels_pl(void)
{
	return s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
		s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p;
}

static uint32_t s5k4e1gx_get_pict_max_exp_lc(void)
{
	uint32_t snapshot_lines_per_frame;

	if (s5k4e1gx_ctrl->pict_res == S_QTR_SIZE)
		snapshot_lines_per_frame =
		s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;
	else
		snapshot_lines_per_frame = S5K4E1GX_MAX_SNAPSHOT_EXP_LC * 6;

	return snapshot_lines_per_frame;
}

static int32_t s5k4e1gx_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;

	s5k4e1gx_ctrl->fps_divider = fps->fps_div;

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
		REG_FRAME_LENGTH_LINES_MSB,
		(((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
			s5k4e1gx_ctrl->fps_divider / 0x400) & 0xFF00) >> 8);
	if (rc < 0)
		goto set_fps_done;

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
		REG_FRAME_LENGTH_LINES_LSB,
		(((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
			s5k4e1gx_ctrl->fps_divider / 0x400) & 0x00FF));

set_fps_done:
	return rc;
}

static int32_t s5k4e1gx_write_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	uint16_t max_legal_gain = 0x0200;
	uint32_t ll_ratio; /* Q10 */
	uint32_t ll_pck, fl_lines;
	uint16_t offset = 8;
	uint32_t  gain_msb, gain_lsb;
	uint32_t  intg_t_msb, intg_t_lsb;
	uint32_t  ll_pck_msb, ll_pck_lsb;

	struct s5k4e1gx_i2c_reg_conf tbl[3];

	CDBG("Line:%d s5k4e1gx_write_exp_gain \n", __LINE__);
	if (s5k4e1gx_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

		s5k4e1gx_ctrl->my_reg_gain = gain;
		s5k4e1gx_ctrl->my_reg_line_count = (uint16_t)line;

		fl_lines = s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;

		ll_pck = s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p;

	} else {

		fl_lines = s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l;

		ll_pck = s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* in Q10 */
	/* line = (line * s5k4e1gx_ctrl->fps_divider);*/

	if ((fl_lines-offset) < line)
		ll_ratio = (line * 0x400 / (fl_lines - offset));
	else
		ll_ratio = 0x400;

	/* update gain registers */
	gain_msb = (gain & 0xFF00) >> 8;
	gain_lsb = gain & 0x00FF;
	tbl[0].waddr = S5K4E1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[0].bdata = S5K4E1GX_GROUP_PARAMETER_HOLD;
	tbl[1].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB;
	tbl[1].bdata = gain_msb;
	tbl[2].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB;
	tbl[2].bdata = gain_lsb;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;

	ll_pck = ll_pck * ll_ratio;
	ll_pck_msb = ((ll_pck / 0x400) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck / 0x400) & 0x00FF;
	tbl[0].waddr = REG_LINE_LENGTH_PCK_MSB;
	tbl[0].bdata = ll_pck_msb;
	tbl[1].waddr = REG_LINE_LENGTH_PCK_LSB;
	tbl[1].bdata = ll_pck_lsb;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;

	line = (line * 0x400) / ll_ratio;
	intg_t_msb = (line & 0xFF00) >> 8;
	intg_t_lsb = (line & 0x00FF);
	tbl[0].waddr = REG_COARSE_INTEGRATION_TIME;
	tbl[0].bdata = intg_t_msb;
	tbl[1].waddr = REG_COARSE_INTEGRATION_TIME_LSB;
	tbl[1].bdata = intg_t_lsb;
	tbl[2].waddr = S5K4E1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[2].bdata = S5K4E1GX_GROUP_PARAMETER_UNHOLD;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));

write_gain_done:
	return rc;
}

static int32_t s5k4e1gx_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	pr_info("Line:%d s5k4e1gx_set_pict_exp_gain \n", __LINE__);

	rc =
		s5k4e1gx_write_exp_gain(gain, line);

	return rc;
}

static int32_t s5k4e1gx_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
	case S_QTR_SIZE:
		rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		if (rc < 0)
			return rc;

		CDBG("s5k4e1gx sensor configuration done!\n");
		break;

	case S_FULL_SIZE:
		rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		if (rc < 0)
			return rc;

		break;
	default:
		return 0;
	} /* switch */

	s5k4e1gx_ctrl->prev_res = res;
	s5k4e1gx_ctrl->curr_res = res;
	s5k4e1gx_ctrl->sensormode = mode;
	rc =
		s5k4e1gx_write_exp_gain(s5k4e1gx_ctrl->my_reg_gain,
			s5k4e1gx_ctrl->my_reg_line_count);

	return rc;
}

static int32_t s5k4e1gx_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;

	s5k4e1gx_ctrl->curr_res = s5k4e1gx_ctrl->pict_res;
	s5k4e1gx_ctrl->sensormode = mode;

	return rc;
}

static int32_t s5k4e1gx_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;

	s5k4e1gx_ctrl->curr_res = s5k4e1gx_ctrl->pict_res;
	s5k4e1gx_ctrl->sensormode = mode;

	return rc;
}

static int32_t s5k4e1gx_set_sensor_mode(int mode, int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = s5k4e1gx_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = s5k4e1gx_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = s5k4e1gx_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}
static int32_t s5k4e1gx_go_to_position(uint32_t lens_pos, uint8_t mask)
{
	int32_t rc = 0;
	unsigned char buf[2];
	uint8_t code_val_msb, code_val_lsb;

	code_val_msb = lens_pos >> 4;
	code_val_lsb = (lens_pos & 0x000F) << 4;
	code_val_lsb |= mask;

	buf[0] = code_val_msb;
	buf[1] = code_val_lsb;
	rc = s5k4e1gx_i2c_txdata(S5K4E1GX_AF_I2C_ADDR >> 1, buf, 2);
	if (rc < 0)
		pr_err("i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
			S5K4E1GX_AF_I2C_ADDR >> 1, buf[0], buf[1]);

	return rc;
}

static int32_t s5k4e1gx_move_focus(int direction, int32_t num_steps)
{
	uint8_t s5k4e1gx_mode_mask = 0;
	uint16_t s5k4e1gx_sw_damping_time_wait = 1;
	uint16_t s5k4e1gx_damping_minimum_distance = 80;
	int16_t step_direction;
	int16_t curr_lens_pos;
	int16_t curr_step_pos;
	int16_t dest_lens_pos;
	int16_t dest_step_pos;
	int16_t target_dist;
	int16_t small_step;
	int16_t next_lens_pos;
	int32_t rc = 0, time_wait;

	if (num_steps > S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0)
		return -EINVAL;

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if (direction == MOVE_FAR)
		step_direction = -1;
	else
		return -EINVAL;

	/* need to decide about default position and power supplied
	 * at start up and reset */
	curr_lens_pos = s5k4e1gx_ctrl->curr_lens_pos;
	curr_step_pos = s5k4e1gx_ctrl->curr_step_pos;

	if (curr_lens_pos < s5k4e1gx_ctrl->init_curr_lens_pos)
		curr_lens_pos = s5k4e1gx_ctrl->init_curr_lens_pos;

	dest_step_pos = curr_step_pos + (step_direction * num_steps);

	if (dest_step_pos < 0)
		dest_step_pos = 0;
	else if (dest_step_pos > S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_pos = S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_pos == s5k4e1gx_ctrl->curr_step_pos)
		return rc;

	dest_lens_pos = s5k4e1gx_pos_tbl[dest_step_pos];
	target_dist = step_direction * (dest_lens_pos - curr_lens_pos);

	/* HW damping */
	if (step_direction < 0 && (target_dist >= s5k4e1gx_damping_minimum_distance)) {
		time_wait = 1000000 / S5K4E1GX_MAX_FPS - S5K4E1GX_SW_DAMPING_STEP * s5k4e1gx_sw_damping_time_wait * 1000;
		small_step = (uint16_t)target_dist/S5K4E1GX_SW_DAMPING_STEP;
		s5k4e1gx_sw_damping_time_wait = 2;
	} else {
		time_wait = 1000000 / S5K4E1GX_MAX_FPS;
		small_step = (uint16_t)target_dist/S5K4E1GX_SW_DAMPING_STEP2;
		s5k4e1gx_sw_damping_time_wait = 4;
	}

	for (next_lens_pos = curr_lens_pos + (step_direction * small_step);
			(step_direction * next_lens_pos) <= (step_direction * dest_lens_pos);
			next_lens_pos += (step_direction * small_step)) {
		rc = s5k4e1gx_go_to_position(next_lens_pos, s5k4e1gx_mode_mask);
		if (rc < 0) {
			pr_err("s5k4e1gx_go_to_position Failed in Move Focus!!!\n");
			return rc;
		}
		curr_lens_pos = next_lens_pos;
		mdelay(s5k4e1gx_sw_damping_time_wait);
	}

	if (curr_lens_pos != dest_lens_pos) {
		rc = s5k4e1gx_go_to_position(dest_lens_pos, s5k4e1gx_mode_mask);
		if (rc < 0) {
			pr_err("s5k4e1gx_go_to_position Failed in Move Focus!!!\n");
			return rc;
		}
		mdelay(s5k4e1gx_sw_damping_time_wait);
	}

	s5k4e1gx_ctrl->curr_lens_pos = dest_lens_pos;
	s5k4e1gx_ctrl->curr_step_pos = dest_step_pos;

	return rc;
}

static int32_t s5k4e1gx_set_default_focus(void)
{
	int32_t rc = 0;
	if (s5k4e1gx_ctrl->curr_step_pos != 0) {
		rc = s5k4e1gx_move_focus(MOVE_FAR, s5k4e1gx_ctrl->curr_step_pos);
		if (rc < 0) {
			pr_err("s5k4e1gx_set_default_focus Failed!!!\n");
			return rc;
		}
	} else {
		rc = s5k4e1gx_go_to_position(0, 0x02);
		if (rc < 0) {
			pr_err("s5k4e1gx_go_to_position Failed!!!\n");
			return rc;
		}
	}

	s5k4e1gx_ctrl->curr_lens_pos = 0;
	s5k4e1gx_ctrl->init_curr_lens_pos = 0;
	s5k4e1gx_ctrl->curr_step_pos = 0;

	return rc;
}

static int s5k4e1gx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&s5k4e1gx_mutex);

	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		s5k4e1gx_get_pict_fps(cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = s5k4e1gx_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = s5k4e1gx_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = s5k4e1gx_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = s5k4e1gx_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			s5k4e1gx_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = s5k4e1gx_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc =
			s5k4e1gx_write_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc =
			s5k4e1gx_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc =
			s5k4e1gx_set_sensor_mode(
			cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = s5k4e1gx_power_down();
		break;

	case CFG_MOVE_FOCUS:
		rc =
			s5k4e1gx_move_focus(
			cdata.cfg.focus.dir,
			cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc =
			s5k4e1gx_set_default_focus();
		break;

	case CFG_GET_AF_MAX_STEPS:
	case CFG_SET_EFFECT:
	case CFG_SET_LENS_SHADING:
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&s5k4e1gx_mutex);
	return rc;
}

static int s5k4e1gx_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;

	rc = i2c_add_driver(&s5k4e1gx_i2c_driver);
	if (rc < 0 || s5k4e1gx_client == NULL) {
		pr_err("!!!Add s5k4e1 i2c driver failed\n");
		rc = -ENOTSUPP;
		goto probe_fail;
	}

	rc = i2c_add_driver(&adp1560_i2c_driver);
	if (0 > rc || adp1560_client == NULL) {
		pr_err("!!!Add adp1560 i2c driver failed\n");
	}

	msm_camio_clk_rate_set(24000000);
	mdelay(20);

	rc = s5k4e1gx_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = s5k4e1gx_sensor_open_init;
	s->s_release = s5k4e1gx_sensor_release;
	s->s_config  = s5k4e1gx_sensor_config;
	s5k4e1gx_probe_init_done(info);

	return rc;

probe_fail:
	pr_err("SENSOR PROBE FAILS!\n");
	return rc;
}

static int __s5k4e1gx_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k4e1gx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k4e1gx_probe,
	.driver = {
		.name = "msm_camera_s5k4e1gx",
		.owner = THIS_MODULE,
	},
};

static int __init s5k4e1gx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k4e1gx_init);
