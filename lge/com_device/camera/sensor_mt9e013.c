/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#if 1 /*                                              */
#include <linux/module.h>
#endif
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "sensor_mt9e013.h"

/*                                                             
                                              
                                                              */
#define CAMERA_LOG_DISABLE
#ifdef CAMERA_LOG_DISABLE
 #ifdef CDBG
 #undef CDBG
 #endif
 #define CDBG(fmt, args...) do { } while (0)
#else
 #ifdef CDBG
 #undef CDBG
 #endif
 #define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD	0x0104
#define GROUPED_PARAMETER_HOLD_OFF	0x00
#define GROUPED_PARAMETER_HOLD		0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME	0x3012
/* Gain */
#define REG_GLOBAL_GAIN				0x305E
/* PLL registers */
#define REG_FRAME_LENGTH_LINES		0x0340
/* Test Pattern */
#define REG_TEST_PATTERN_MODE		0x0601

#define REG_VCM_CONTROL				0x30F0
#define REG_VCM_NEW_CODE			0x30F2
#define REG_VCM_STEP_TIME			0x30F4

#define REG_LENS_SHADING	        0x3780
#define LSC_ON						1
#define LSC_OFF 					0


#define MT9E013_EEPROM_SLAVE_ADDR	0xA0>>1 	//EEPROM Slave Address for 5100K(Page #1)

/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define MT9E013_MASTER_CLK_RATE 24000000

/* AF Total steps parameters */
#define MT9E013_TOTAL_STEPS_NEAR_TO_FAR		32

static uint16_t mt9e013_linear_total_step = MT9E013_TOTAL_STEPS_NEAR_TO_FAR;
uint16_t mt9e013_step_position_table[MT9E013_TOTAL_STEPS_NEAR_TO_FAR+1];

//                                                 
uint16_t af_infinity = 30;  // sungmin.woo : at least 64~64+96
uint16_t mt9e013_nl_region_boundary1 = 0;
uint16_t mt9e013_nl_region_code_per_step1 = 0;
uint16_t mt9e013_l_region_code_per_step = 5;
uint16_t mt9e013_vcm_step_time;
uint16_t mt9e013_sw_damping_time_wait;
//                                                 

#define mt9e013_offset 5

struct mt9e013_work_t {
	struct work_struct work;
};

//                                                            
static struct mt9e013_i2c_reg_conf *lsc_data;
//                                                            


static struct mt9e013_work_t *mt9e013_sensorw;
static struct i2c_client *mt9e013_client;

struct mt9e013_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9e013_resolution_t prev_res;
	enum mt9e013_resolution_t pict_res;
	enum mt9e013_resolution_t curr_res;
	enum mt9e013_test_mode_t  set_test;
};


static bool CSI_CONFIG;
static struct mt9e013_ctrl_t *mt9e013_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9e013_wait_queue);

DEFINE_MUTEX(mt9e013_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;
/*=============================================================*/

static int mt9e013_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
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
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9e013_client->adapter, msgs, 2) < 0) {
		printk(KERN_EMERG "mt9e013_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

//                                                 
static int mt9e013_eeprom_i2c_rxdata(uint8_t saddr,
	uint8_t *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};
	
	if (i2c_transfer(mt9e013_client->adapter, msgs, 2) < 0) {
		printk(KERN_EMERG "[CAMERA] %s: mt9e013_i2c_rxdata faild 0x%x\n", __func__, saddr);
		return -EIO;
	}
	return 0;
}
//                                                 

static int32_t mt9e013_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9e013_client->adapter, msg, 1) < 0) {
		printk(KERN_EMERG "mt9e013_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9e013_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9e013_i2c_rxdata(mt9e013_client->addr<<1, buf, rlen);
	if (rc < 0) {
		printk(KERN_EMERG "mt9e013_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("mt9e013_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t mt9e013_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = mt9e013_i2c_txdata(mt9e013_client->addr<<1, buf, 4);
	if (rc < 0) {
		printk(KERN_EMERG "i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t mt9e013_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = mt9e013_i2c_txdata(mt9e013_client->addr<<1, buf, 3);
	if (rc < 0) {
		printk(KERN_EMERG "i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9e013_i2c_write_w_table(struct mt9e013_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = mt9e013_i2c_write_w_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

//                                                 
static int32_t mt9e013_i2c_read_w_eeprom(uint16_t raddr,
	uint16_t *rdata)
{
	int32_t rc = 0;
	unsigned char buf;
	
	if (!rdata)
		return -EIO;

	//Read 2 bytes in sequence 
	//Big Endian address:
	buf = raddr;

	buf = (raddr & 0xFF00) >> 8;
	
	rc = mt9e013_eeprom_i2c_rxdata(MT9E013_EEPROM_SLAVE_ADDR, &buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]1 :mt9e013_i2c_read_eeprom 0x%x failed!\n", buf);
		return rc;
	}
	*rdata = buf;

	buf = (raddr & 0x00FF);
	
	rc = mt9e013_eeprom_i2c_rxdata(MT9E013_EEPROM_SLAVE_ADDR, &buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]2 :mt9e013_i2c_read_eeprom 0x%x failed!\n", buf);
		return rc;
	}
	*rdata = (*rdata<<8)|buf;

	return rc;
}
//                                                 


//                                                                     
//#define EEPROM_CHECK
static int32_t mt9e013_read_5100k_data(void)
{
	int32_t i,j,n,k;
	int32_t rc = 0;
	uint16_t raddr=0x0001;
	uint16_t waddr=0x3600;
	uint16_t eepromdata = 0;
	
	int32_t data_size=20;	// EEPROM data size

	k=0;
	lsc_data = kzalloc(sizeof(struct mt9e013_i2c_reg_conf)*106, GFP_KERNEL);

	printk(KERN_EMERG "[CAMERA][EEPROM] %s : Start : 0x%04x 0x%04x\n", __func__, raddr, waddr);
	n=0x01;
	for(j=0;j<5;j++) {
		for(i=0;i<data_size;i++) {
			rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
			if (rc < 0) {
				printk(KERN_EMERG "[CAMERA]%s : Fail to read!! j=%d,i=%d Source line number : %d\n", __func__, j, i, __LINE__);
				return rc;
			}
#ifdef EEPROM_CHECK
			CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x waddr:0x%04x eepromdata: 0x%04x\n", raddr, waddr, eepromdata);
#endif			
			lsc_data[k].waddr=waddr;
			lsc_data[k].wdata=eepromdata;
			
			raddr++;waddr += 2;
			n += 2;k++;
			raddr = (raddr<<8)|n;
		}
		waddr = waddr + 0x0018;
	}

	// Second Phase
	data_size=2;
	waddr=0x3782;
	for(i=0;i<data_size;i++) {
		rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
		if (rc < 0) {
			printk(KERN_EMERG "[CAMERA]%s : Fail to read!! j=%d,i=%d Source line number : %d\n", __func__, j, i, __LINE__);
			return rc;
		}
#ifdef EEPROM_CHECK
		CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x waddr:0x%04x eepromdata: 0x%04x\n", raddr, waddr, eepromdata);
#endif
		lsc_data[k].waddr=waddr;
		lsc_data[k].wdata=eepromdata;
		
		raddr++;waddr += 2;
		n += 2;k++;
		raddr = (raddr<<8)|n;
	}

	// Third Phase
	data_size=4;
	waddr=0x37C0;
	for(i=0;i<data_size;i++) {
		rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
		if (rc < 0) {
			printk(KERN_EMERG "[CAMERA]%s : Fail to read!! j=%d,i=%d Source line number : %d\n", __func__, j, i, __LINE__);
			return rc;
		}
#ifdef EEPROM_CHECK
		CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x waddr:0x%04x eepromdata: 0x%04x\n", raddr, waddr, eepromdata);
#endif
		lsc_data[k].waddr=waddr;
		lsc_data[k].wdata=eepromdata;

		raddr++;waddr += 2;
		n += 2;k++;
		raddr = (raddr<<8)|n;
	}
	
#ifdef EEPROM_CHECK
	for(i=0;i<110;i++) {
		CDBG("[CAMERA][EEPROM_CHECK]count: %03d waddr:0x%04x eepromdata: 0x%04x\n", i, lsc_data[i].waddr, lsc_data[i].wdata);
	}
#endif

	return rc;
}


static int32_t mt9e013_write_5100k_data(void)
{
	int32_t rc=0;

	rc = mt9e013_i2c_write_w_table(lsc_data, 106);	// 106: LSC data length
	return rc;
}

static int32_t mt9e013_read_awb_data(struct sensor_cfg_data *cfg, bool bresult)
{
	int32_t rc = 0;

	uint16_t raddr=0;
	uint16_t eepromdata = 0;

	raddr=0xD4D5;	// R/G
	rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		cfg->cfg.calib_info.r_over_g = 0x0300;	//0x005E;
	} else {
		cfg->cfg.calib_info.r_over_g = eepromdata;
	}
	CDBG("[CAMERA]%s : line:%d : R/G 0x%04x\n", __func__, __LINE__, cfg->cfg.calib_info.r_over_g);

	raddr=0xD6D7;	// B/G
	rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		cfg->cfg.calib_info.b_over_g = 0x0289;	//0x0051;
	} else {
		cfg->cfg.calib_info.b_over_g = eepromdata;
	}
	CDBG("[CAMERA]%s : line:%d : B/G 0x%04x\n", __func__, __LINE__, cfg->cfg.calib_info.b_over_g);

	raddr=0xD4D5;	// Gr/Gb
	rc = mt9e013_i2c_read_w_eeprom(raddr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		cfg->cfg.calib_info.gr_over_gb = 0x0300;	//0x005E;
	} else {
		cfg->cfg.calib_info.gr_over_gb = eepromdata;
	}
	printk(KERN_EMERG "[CAMERA]%s : line:%d : GR/GB 0x%04x\n", __func__, __LINE__, cfg->cfg.calib_info.gr_over_gb);

	return rc;
}

//                                                                     

static void mt9e013_group_hold_on(void)
{
	CDBG("[CAMERA]mt9e013_group_hold_on\n");

	mt9e013_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void mt9e013_group_hold_off(void)
{
	CDBG("[CAMERA]mt9e013_group_hold_off\n");
	mt9e013_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void mt9e013_start_stream(void)
{
	CDBG("[CAMERA]mt9e013_start_stream\n");

	mt9e013_i2c_write_w_sensor(0x301A, 0x8250);
	mt9e013_i2c_write_w_sensor(0x301A, 0x8650);
	mt9e013_i2c_write_w_sensor(0x301A, 0x8658);
	mt9e013_i2c_write_b_sensor(0x0104, 0x00);
	mt9e013_i2c_write_w_sensor(0x301A, 0x065C);
}

static void mt9e013_stop_stream(void)
{
	CDBG("[CAMERA]mt9e013_stop_stream\n");
	mt9e013_i2c_write_w_sensor(0x301A, 0x0058);		//Start_Streaming
	mt9e013_i2c_write_w_sensor(0x301A, 0x0050);		//Lock_Register
	mt9e013_i2c_write_b_sensor(0x0104, 0x01);		//GROUPED_PARAMETER_HOLD   ==== 8bit 
}

static void mt9e013_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
		* 0x00000400/
		mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	d2 = mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata
		* 0x00000400/
		mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	divider = d1 * d2 / 0x400;
	CDBG("[CAMERA]mt9e013_get_pict_fps: divider = %d, d1 = %d, d2 = %d \n", divider, d1, d2);

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */

	CDBG("[CAMERA]mt9e013_get_pict_fps:fps = %d, pfps = %d\n", fps, *pfps);
}

static uint16_t mt9e013_get_prev_lines_pf(void)
{
	CDBG("[CAMERA]mt9e013_get_prev_lines_pf\n");

	if (mt9e013_ctrl->prev_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->prev_res == FHD_SIZE)
		return mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
}


static uint16_t mt9e013_get_prev_pixels_pl(void)
{
	CDBG("[CAMERA]mt9e013_get_prev_pixels_pl\n");

	if (mt9e013_ctrl->prev_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->prev_res == FHD_SIZE)
		return mt9e013_regs.reg_FHD[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
}

static uint16_t mt9e013_get_pict_lines_pf(void)
{
	CDBG("[CAMERA]mt9e013_get_pict_lines_pf\n");

	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->pict_res == FHD_SIZE)
		return mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9e013_get_pict_pixels_pl(void)
{
	CDBG("[CAMERA]mt9e013_get_pict_pixels_pl\n");

	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->pict_res == FHD_SIZE)
		return mt9e013_regs.reg_FHD[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
}

static uint32_t mt9e013_get_pict_max_exp_lc(void)
{
	CDBG("[CAMERA]mt9e013_get_pict_max_exp_lc\n");

	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata * 24;
	else if (mt9e013_ctrl->pict_res == FHD_SIZE)
		return mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata * 24;
	else
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata * 24;
}

static int32_t mt9e013_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_set_fps mode:%d\n", mt9e013_ctrl->curr_res);
	
	if (mt9e013_ctrl->curr_res == QTR_SIZE)
		total_lines_per_frame =	mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->curr_res == FHD_SIZE)
		total_lines_per_frame =	mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata;
	else
		total_lines_per_frame =	mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;


	mt9e013_ctrl->fps_divider = fps->fps_div;
	mt9e013_ctrl->pict_fps_divider = fps->pict_fps_div;
	CDBG("[CAMERA][1] mt9e013_set_fps : total_lines_per_frame = %d, fps->fps_div = %d, fps->pict_fps_div = %d\n", 
												total_lines_per_frame, fps->fps_div, fps->pict_fps_div);

	if (mt9e013_ctrl->curr_res == FULL_SIZE) {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_ctrl->pict_fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_ctrl->fps_divider/0x400);
	}
	CDBG("[CAMERA][2] mt9e013_set_fps : total_lines_per_frame = %d, fps->fps_div = %d, fps->pict_fps_div = %d\n", 
												total_lines_per_frame, fps->fps_div, fps->pict_fps_div);

	mt9e013_group_hold_on();
	rc = mt9e013_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	mt9e013_group_hold_off();
	return rc;
}

static int32_t mt9e013_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0xE7F;
	//uint16_t frame_length_lines;
    
	int32_t rc = 0;
#if 1
    CDBG("mt9e013_write_exp_gain entering.... \n");
    if (mt9e013_ctrl->curr_res == SENSOR_PREVIEW_MODE) {
        mt9e013_ctrl->my_reg_gain = gain;
        mt9e013_ctrl->my_reg_line_count = (uint16_t) line;
    }
#endif

    if (gain > max_legal_gain) {
        CDBG("Max legal gain Line:%d\n", __LINE__);
        gain = max_legal_gain;
    }
#if 0

    if(mt9e013_ctrl->curr_res == QTR_SIZE)
    {
        frame_length_lines = mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
        frame_length_lines = frame_length_lines * mt9e013_ctrl->fps_divider / 0x400;
		line = (uint32_t) (line * mt9e013_ctrl->fps_divider /  0x00000400);		
    }
    else if(mt9e013_ctrl->curr_res == FHD_SIZE)
    {
        frame_length_lines = mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata;
        frame_length_lines = frame_length_lines * mt9e013_ctrl->fps_divider / 0x400;
		line = (uint32_t) (line * mt9e013_ctrl->fps_divider /  0x00000400);		
    }
    else        //snapshot
    {
        frame_length_lines = mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
        frame_length_lines = frame_length_lines * mt9e013_ctrl->pict_fps_divider / 0x400;
    	mt9e013_ctrl->pict_fps_divider = mt9e013_ctrl->fps_divider * 2;
        line = (uint32_t) (line * mt9e013_ctrl->pict_fps_divider /  0x00000400);		
    }
#else
    if (mt9e013_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
        //mt9e013_ctrl->my_reg_gain = gain;
        //mt9e013_ctrl->my_reg_line_count = (uint16_t) line;
        line = (uint32_t) (line * mt9e013_ctrl->fps_divider /  0x00000400);
    } else {
    	//mt9e013_ctrl->pict_fps_divider = mt9e013_ctrl->fps_divider / 2;
        line = (uint32_t) (line * mt9e013_ctrl->pict_fps_divider /  0x00000400);
    }
#endif

#if 0
if(line > frame_length_lines - mt9e013_offset)
{
	frame_length_lines = line + mt9e013_offset;
}
#endif
	CDBG("[CAMERA][1] mt9e013_write_exp_gain : frame_length_lines = NONE, mt9e013_ctrl->fps_divider = %d, mt9e013_ctrl->pict_fps_divider = %d\n", 
												/*frame_length_lines, */mt9e013_ctrl->fps_divider, mt9e013_ctrl->pict_fps_divider);	
	CDBG("[CAMERA][2] mt9e013_write_exp_gain : line=%d gain=%d\n",line, gain); 

    gain |= 0x1000;

    mt9e013_group_hold_on();
//	rc = mt9e013_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES, frame_length_lines);
    rc = mt9e013_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
    rc = mt9e013_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
    mt9e013_group_hold_off();

#if 0 // test 1012
{
	unsigned short s_data, s_data2, s_data3;
	rc = mt9e013_i2c_read(REG_GLOBAL_GAIN, &s_data, 2);
	rc = mt9e013_i2c_read(REG_COARSE_INTEGRATION_TIME, &s_data2, 2);
	rc = mt9e013_i2c_read(REG_LENS_SHADING, &s_data3, 2);
	printk(KERN_EMERG "[CAMERA][1012] mt9e013_write_exp_gain : line=%d gain=%d REG_GLOBAL_GAIN=%d REG_COARSE_INTEGRATION_TIME=%d REG_LENS_SHADING=0x%04x\n",
			line, gain, s_data, s_data2,s_data3);
}
#endif
	
    return rc;
}


static int32_t mt9e013_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_set_pict_exp_gain : gain = %d,line = %d \n", gain, line);

	rc = mt9e013_write_exp_gain(gain, line);
	mt9e013_i2c_write_w_sensor(0x301A, 0x065C|0x2);
	return rc;
}

static int32_t mt9e013_move_focus(int direction,
	int32_t num_steps)
{
	int16_t step_direction, dest_lens_position, dest_step_position;

	CDBG("[CAMERA]mt9e013_move_focus : direction = %d,num_steps = %d \n", direction, num_steps);

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else
		step_direction = -1;

	dest_step_position = mt9e013_ctrl->curr_step_pos
						+ (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > mt9e013_linear_total_step)
		dest_step_position = mt9e013_linear_total_step;

	if (dest_step_position == mt9e013_ctrl->curr_step_pos)
		return 0;
	CDBG("[CAMERA]__debug:MoveFocus, dest_step_position:%d \n", dest_step_position);
	dest_lens_position = mt9e013_step_position_table[dest_step_position];

	if ((dest_step_position <= 4) && (step_direction == 1)) {
		mt9e013_i2c_write_w_sensor(REG_VCM_STEP_TIME, 0x0000);
		if (num_steps == 4) {
			CDBG("[CAMERA]__debug:MoveFocus, jumpvalue:%d \n",
			mt9e013_nl_region_boundary1 * mt9e013_nl_region_code_per_step1);
			mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
			mt9e013_nl_region_boundary1 * mt9e013_nl_region_code_per_step1);
		} else {
			if (dest_step_position <= mt9e013_nl_region_boundary1) {
				CDBG("[CAMERA]__debug:MoveFocus, fine search:%d \n",
					dest_lens_position);
				mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
					dest_lens_position);
				mt9e013_ctrl->curr_lens_pos = dest_lens_position;
				mt9e013_ctrl->curr_step_pos = dest_step_position;
				return 0;
			}
		}
	}

	if(step_direction < 0) {
		if(num_steps > 20) {
			/*macro to infinity*/
			mt9e013_vcm_step_time = 0x0050;
			mt9e013_sw_damping_time_wait = 5;
		} else if (num_steps <= 4) {
			/*reverse search fine step  dir - macro to infinity*/
			mt9e013_vcm_step_time = 0x0400;
			mt9e013_sw_damping_time_wait = 4;
		} else {
			/*reverse search Coarse Jump ( > 4) dir - macro to infinity*/
			mt9e013_vcm_step_time = 0x96;
			mt9e013_sw_damping_time_wait = 3;
			}
	} else {
		if(num_steps >= 4) {
			/*coarse jump  dir - infinity to macro*/
			mt9e013_vcm_step_time = 0x0200;
			mt9e013_sw_damping_time_wait = 2;
		} else {
			/*fine step  dir - infinity to macro*/
			mt9e013_vcm_step_time = 0x0400;
			mt9e013_sw_damping_time_wait = 4;
		}
	}

	mt9e013_i2c_write_w_sensor(REG_VCM_STEP_TIME,
			mt9e013_vcm_step_time);
	CDBG("[CAMERA]__debug:MoveFocus, mt9e013_vcm_step_time:%d \n", mt9e013_vcm_step_time);
	CDBG("[CAMERA]__debug:MoveFocus, DestLensPosition:%d \n", dest_lens_position);
	if (mt9e013_ctrl->curr_lens_pos != dest_lens_position) {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		dest_lens_position);
		usleep(mt9e013_sw_damping_time_wait * 1000);
	}
	mt9e013_ctrl->curr_lens_pos = dest_lens_position;
	mt9e013_ctrl->curr_step_pos = dest_step_position;
	return 0;
}

static int32_t mt9e013_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_set_default_focus : af_step = %d \n", af_step);

	if (mt9e013_ctrl->curr_step_pos != 0) {
		rc = mt9e013_move_focus(MOVE_FAR,
		mt9e013_ctrl->curr_step_pos);
	} else {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x00);
	}

	mt9e013_ctrl->curr_lens_pos = 0;
	mt9e013_ctrl->curr_step_pos = 0;

	return rc;
}

static void mt9e013_init_focus(void)
{
	uint8_t i;

	CDBG("[CAMERA]mt9e013_init_focus\n");

//                                                 
	mt9e013_step_position_table[0] = af_infinity;
//                                                 

	for (i = 1; i <= mt9e013_linear_total_step; i++) {
		if (i <= mt9e013_nl_region_boundary1) {
			mt9e013_step_position_table[i] =
				mt9e013_step_position_table[i-1]
				+ mt9e013_nl_region_code_per_step1;
		} else {
			mt9e013_step_position_table[i] =
				mt9e013_step_position_table[i-1]
				+ mt9e013_l_region_code_per_step;
		}
		if (mt9e013_step_position_table[i] > 255)
			mt9e013_step_position_table[i] = 255;
	}
	mt9e013_ctrl->curr_lens_pos = 0;
}

static int32_t mt9e013_test(enum mt9e013_test_mode_t mo)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_test\n");

	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (mt9e013_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t mt9e013_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;

	uint16_t total_lines_per_frame;

	struct msm_camera_csi_params mt9e013_csi_params;
	CDBG("[CAMERA]sensor_settings\n");
	printk(KERN_EMERG "[CAMERA]mt9e013_sensor_setting :::: update_type = %d, rt = %d\n",update_type, rt );
	mt9e013_stop_stream();
	msleep(5);	//                                                       
	if (update_type == REG_INIT) {
		printk(KERN_EMERG "[CAMERA]mt9e013_sensor_setting :::: REG_INIT\n");
		mt9e013_i2c_write_w_table(mt9e013_regs.reg_mipi, mt9e013_regs.reg_mipi_size);
		msleep(5);
		mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,	mt9e013_regs.reg_pll_size);
		mt9e013_i2c_write_w_table(mt9e013_regs.rec_settings, mt9e013_regs.rec_size);

		cam_debug_init();
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {

		if (rt == QTR_SIZE) {
			printk(KERN_EMERG "[CAMERA]mt9e013_sensor_setting :::: QTR_SIZE\n");
			//                                                                
			//                                                                                                                       

			//                                                         
			//mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,	mt9e013_regs.reg_pll_size);
			//msleep(1);
			//                                                         
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_prev, mt9e013_regs.reg_prev_size);
			//                                                                
			#if 1
			{
				total_lines_per_frame =	mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
				mt9e013_group_hold_on();
				rc = mt9e013_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,total_lines_per_frame);
				mt9e013_group_hold_off();
			}
			#endif
			//                                                                
		} else if (rt == FHD_SIZE) {
			printk(KERN_EMERG "[CAMERA]mt9e013_sensor_setting :::: FHD_SIZE\n");
			//                                                                
			//                                                                                                                       

			//                                                         
			//mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,	mt9e013_regs.reg_pll_size);
			//msleep(1);
			//                                                         
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_FHD,	mt9e013_regs.reg_FHD_size);
			//                                                                
			#if 1
			{
				total_lines_per_frame =	mt9e013_regs.reg_FHD[E013_FRAME_LENGTH_LINES].wdata;
				mt9e013_group_hold_on();
				rc = mt9e013_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,total_lines_per_frame);
				mt9e013_group_hold_off();
			}
			#endif
			//                                                                
		} else if (rt == FULL_SIZE) {
			printk(KERN_EMERG "[CAMERA]mt9e013_sensor_setting :::: FULL_SIZE\n");
			//                                                         
			//mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,	mt9e013_regs.reg_pll_size);
			//msleep(1);
			//                                                         
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_snap, mt9e013_regs.reg_snap_size);
		} 

		if (!CSI_CONFIG) {
			//                                                         
			if (rt != FULL_SIZE){
				msm_camio_vfe_clk_rate_set(192000000);
				mt9e013_csi_params.data_format = CSI_10BIT;
				mt9e013_csi_params.lane_cnt = 2;
				mt9e013_csi_params.lane_assign = 0xe4;
				mt9e013_csi_params.dpcm_scheme = 0;
				mt9e013_csi_params.settle_cnt = 0x18;
				rc = msm_camio_csi_config(&mt9e013_csi_params);
				msleep(10);
			}
			//                                                         
			CSI_CONFIG = 1;
		}
		mt9e013_start_stream();
	}
	return rc;
}

static int32_t mt9e013_video_config(int mode)
{

	int32_t rc = 0;

	CDBG("[CAMERA]video config\n");
	printk(KERN_EMERG "[CAMERA]mt9e013_video_config :::: mode = %d, prev_res = %d\n",mode, mt9e013_ctrl->prev_res );
	/* change sensor resolution if needed */
	if (mt9e013_sensor_setting(UPDATE_PERIODIC,
			mt9e013_ctrl->prev_res) < 0)
		return rc;
	if (mt9e013_ctrl->set_test) {
		if (mt9e013_test(mt9e013_ctrl->set_test) < 0)
			return  rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->prev_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9e013_snapshot_config(int mode)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_snapshot_config : mode = %d\n", mode);
	/*change sensor resolution if needed */
	if (mt9e013_ctrl->curr_res != mt9e013_ctrl->pict_res) {
		if (mt9e013_sensor_setting(UPDATE_PERIODIC,
				mt9e013_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->pict_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
} /*end of mt9e013_snapshot_config*/

static int32_t mt9e013_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9e013_raw_snapshot_config : mode = %d\n", mode);

	/* change sensor resolution if needed */
	if (mt9e013_ctrl->curr_res != mt9e013_ctrl->pict_res) {
		if (mt9e013_sensor_setting(UPDATE_PERIODIC,
				mt9e013_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->pict_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
} /*end of mt9e013_raw_snapshot_config*/

static int32_t mt9e013_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	printk(KERN_EMERG "[CAMERA][KERNEL]mt9e013_set_sensor_mode ::: mode = %d, res = %d\n", mode, res);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		mt9e013_ctrl->prev_res = res;
		rc = mt9e013_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9e013_ctrl->pict_res = res;
		rc = mt9e013_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		mt9e013_ctrl->pict_res = res;
		rc = mt9e013_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t mt9e013_af_power_down(void)
{
	CDBG("[CAMERA]mt9e013_af_power_down\n");

	if (mt9e013_ctrl->curr_lens_pos != 0)
	{
		mt9e013_set_default_focus(0);
		msleep(40);
	}
	mt9e013_i2c_write_w_sensor(REG_VCM_CONTROL, 0x00);
	return 0;
}

static int32_t mt9e013_power_down(void)
{
	printk(KERN_EMERG "[CAMERA]mt9e013_power_down\n");
	mt9e013_af_power_down();
	return 0;
}

static int mt9e013_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("[CAMERA]probe done\n");
	gpio_free(data->sensor_reset);
#if 0
	//                                          
		printk(KERN_EMERG "[CAMERA/+]mt9e013_probe_init_done\n");
		mt9e013_ctrl->sensordata->pdata->camera_power_off();
		printk(KERN_EMERG "[CAMERA/-]mt9e013_probe_init_done\n");
	//                                          
#endif
	return 0;
}


static int32_t mt9e013_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CDBG("[CAMERA]%s: entered. enable = %d\n", __func__, is_enable);

	mt9e013_group_hold_on();

	rc = mt9e013_i2c_write_w_sensor(REG_LENS_SHADING,
			((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;

	mt9e013_group_hold_off();

	CDBG("[CAMERA]%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}


static int mt9e013_read_eeprom_data(struct sensor_cfg_data *cfg)
{
	int32_t rc = 0;
	uint16_t eepromdata = 0;
	uint16_t addr = 0;
	bool bresult = false;

	CDBG("[CAMERA][MT9E013_EEPROM] Start reading EEPROM\n");	

	/*#1. Model ID for checking EEPROM READ*/
	addr = 0xFEFF;
	rc = mt9e013_i2c_read_w_eeprom(addr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	if(eepromdata == 0x0AFF) bresult = true;
	else bresult = false;
	
	printk(KERN_EMERG "[CAMERA][QCTK_EEPROM] Product version = 0x%x bresult:%d\n", eepromdata,bresult);	

	/*#2. 5100K LSC : Read LSC table Data from EEPROM */
	rc = mt9e013_read_5100k_data();	// read LSC data
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s:%d: Error Reading\n", __func__,__LINE__);
		return rc;
	}
	
	/*#3. 5100K AWB Data from EEPROM */	
	mt9e013_read_awb_data(cfg, bresult);
	CDBG("[CAMERA]%s:%d: AWB: r/g:0x%04x b/g:0x%04x gr/gb:0x%04x\n", __func__,__LINE__,
			cfg->cfg.calib_info.r_over_g,cfg->cfg.calib_info.b_over_g,cfg->cfg.calib_info.gr_over_gb);
	
	/*#4. Write LSC data to sensor - it will be enabled in setting routine*/
	mt9e013_group_hold_on();

	//Write LSC table to the sensor
	rc = mt9e013_write_5100k_data();
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s:%d: Error Writing\n", __func__,__LINE__);
		return rc;
	}

	mt9e013_group_hold_off();

	/*Enable Aptina Lens shading */
	mt9e013_lens_shading_enable(LSC_ON); 

	return rc;
}

static int mt9e013_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	CDBG("[CAMERA]%s: %d\n", __func__, __LINE__);
	rc = gpio_request(data->sensor_reset, "mt9e013");
	CDBG("[CAMERA]mt9e013_probe_init_sensor\n");
	if (!rc) {
		printk(KERN_EMERG "[CAMERA] %s : sensor_reset = %d\n", __func__, rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);

	} else {
		goto init_probe_done;
	}

	CDBG("[CAMERA]mt9e013_probe_init_sensor is called\n");

	rc = mt9e013_i2c_read(0x0000, &chipid, 2);
	printk(KERN_EMERG "[CAMERA] chip ID: %x, rc : %d\n", chipid, rc);
	CDBG("[CAMERA]ID: %d\n", chipid);
	// 4. Compare sensor ID to MT9E013 ID:

	if (chipid != 0x4B00) {
		rc = -ENODEV;
		printk(KERN_EMERG "[CAMERA]mt9e013_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}

	mt9e013_ctrl = kzalloc(sizeof(struct mt9e013_ctrl_t), GFP_KERNEL);
	if (!mt9e013_ctrl) {
		printk(KERN_EMERG "[CAMERA]mt9e013_init failed!\n");
		rc = -ENOMEM;
	}
	mt9e013_ctrl->fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->set_test = TEST_OFF;
	mt9e013_ctrl->prev_res = QTR_SIZE;
	mt9e013_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9e013_ctrl->sensordata = data;

	goto init_probe_done;

init_probe_fail:
	printk(KERN_EMERG "[CAMERA] %s: mt9e013_probe_init_sensor fails\n",__func__);
	gpio_set_value_cansleep(data->sensor_reset, 0);
	mt9e013_probe_init_done(data);

init_probe_done:
	printk(KERN_EMERG "[CAMERA] mt9e013_probe_init_sensor finishes \n");
	return rc;
}
/* camsensor_mt9e013_reset */

int mt9e013_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	printk(KERN_EMERG "[CAMERA]mt9e013_sensor_open_init\n");

	mt9e013_ctrl = kzalloc(sizeof(struct mt9e013_ctrl_t), GFP_KERNEL);
	if (!mt9e013_ctrl) {
		printk(KERN_EMERG "[CAMERA]mt9e013_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	mt9e013_ctrl->fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->set_test = TEST_OFF;
	mt9e013_ctrl->prev_res = QTR_SIZE;
	mt9e013_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9e013_ctrl->sensordata = data;
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]Calling mt9e013_sensor_open_init fail1\n");
		return rc;
	}
	CDBG("[CAMERA]%s: %d\n", __func__, __LINE__);
#if 1
//                                          
	data->pdata->camera_power_on();
	printk(KERN_EMERG "[CAMERA]mt9e013_sensor_open_init(Power ON!)\n");
	if (rc < 0) {
		printk(KERN_ERR "[CAMERA][ERROR]%s:failed to power on!\n", __func__);
		return rc;
	}
//                                          
#endif

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9E013_MASTER_CLK_RATE);
	rc = mt9e013_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

	CDBG("[CAMERA]init settings\n");
	rc = mt9e013_sensor_setting(REG_INIT, mt9e013_ctrl->prev_res);
	mt9e013_ctrl->fps = 30*Q8;
	mt9e013_init_focus();
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	printk(KERN_EMERG "[CAMERA] %s: init_fail\n",__func__);
	kfree(mt9e013_ctrl);
	mt9e013_ctrl = NULL;
	mt9e013_probe_init_done(data);
init_done:
	CDBG("[CAMERA]init_done\n");
	return rc;
} /*endof mt9e013_sensor_open_init*/

static int mt9e013_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	printk(KERN_EMERG "[CAMERA]mt9e013_init_client\n");
	init_waitqueue_head(&mt9e013_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9e013_i2c_id[] = {
	{"mt9e013", 0},
	{ }
};

static int mt9e013_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	printk(KERN_EMERG "[CAMERA]mt9e013_i2c_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_EMERG "[CAMERA]i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9e013_sensorw = kzalloc(sizeof(struct mt9e013_work_t), GFP_KERNEL);
	if (!mt9e013_sensorw) {
		printk(KERN_EMERG "[CAMERA]kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9e013_sensorw);
	mt9e013_init_client(client);
	mt9e013_client = client;


	CDBG("[CAMERA]mt9e013_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	printk(KERN_EMERG "[CAMERA] %s: mt9e013_probe failed! rc = %d\n",__func__, rc);
	return rc;
}


static int mt9e013_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} /*end of mt9e013_snapshot_config*/

static int __exit mt9e013_remove(struct i2c_client *client)
{
	struct mt9e013_work_t_t *sensorw = i2c_get_clientdata(client);

	printk(KERN_EMERG "[CAMERA]mt9e013_remove\n");

	free_irq(client->irq, sensorw);
	mt9e013_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9e013_i2c_driver = {
	.id_table = mt9e013_i2c_id,
	.probe  = mt9e013_i2c_probe,
	.remove = __exit_p(mt9e013_i2c_remove),
	.driver = {
		.name = "mt9e013",
	},
};

int mt9e013_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&mt9e013_mut);
	CDBG("[CAMERA]mt9e013_sensor_config : cdata.cfgtype = %d\n", cdata.cfgtype);
	CDBG("[CAMERA]mt9e013_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			mt9e013_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9e013_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9e013_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9e013_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9e013_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9e013_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9e013_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9e013_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9e013_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9e013_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9e013_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9e013_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9e013_set_default_focus(
				cdata.cfg.focus.steps);
			break;

		case CFG_GET_CALIB_DATA:
			rc = mt9e013_read_eeprom_data(&cdata);
			if (rc < 0)
				break;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(cdata)))
				rc = -EFAULT;			
			break;

		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = mt9e013_linear_total_step;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_EFFECT:
			rc = mt9e013_set_default_focus(
				cdata.cfg.effect);
			break;


		case CFG_SEND_WB_INFO:
			rc = mt9e013_send_wb_info(
				&(cdata.cfg.wb_info));
			break;

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&mt9e013_mut);

	return rc;
}

static int mt9e013_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&mt9e013_mut);
	mt9e013_power_down();
#if 1
	//                                          
	mt9e013_ctrl->sensordata->pdata->camera_power_off();
	printk(KERN_EMERG "[CAMERA]mt9e013_sensor_release(Power OFF!)\n");
	//                                          
#endif
	gpio_set_value_cansleep(mt9e013_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_free(mt9e013_ctrl->sensordata->sensor_reset);
	kfree(mt9e013_ctrl);
	kfree(lsc_data);	//                                            
	mt9e013_ctrl = NULL;
	CDBG("[CAMERA]mt9e013_release completed\n");
	mutex_unlock(&mt9e013_mut);

	return rc;
}

static int mt9e013_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&mt9e013_i2c_driver);
	CDBG("[CAMERA] i2c_add_driver****** rc : %d\n", rc);
	if (rc < 0 || mt9e013_client == NULL) {
		rc = -ENOTSUPP;
		printk(KERN_EMERG "[CAMERA] I2C add driver failed****** rc : %d\n", rc);
		goto probe_fail;
	}

//                                          
//	msm_camio_clk_rate_set(MT9E013_MASTER_CLK_RATE);
//	rc = mt9e013_probe_init_sensor(info);
//                                          
	if (rc < 0)
		goto probe_fail;
	s->s_init = mt9e013_sensor_open_init;
	s->s_release = mt9e013_sensor_release;
	s->s_config  = mt9e013_sensor_config;
	s->s_mount_angle  = 90; //                                                                 
	gpio_set_value_cansleep(info->sensor_reset, 0);
//                                          
//	mt9e013_probe_init_done(info);
//                                          

	return rc;

probe_fail:
	printk(KERN_EMERG "[CAMERA] %s: SENSOR PROBE FAILS!\n",__func__);
	return rc;
}

static int __mt9e013_probe(struct platform_device *pdev)
{
	printk(KERN_EMERG "[CAMERA]__mt9e013_probe\n");
	return msm_camera_drv_start(pdev, mt9e013_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9e013_probe,
	.driver = {
		.name = "msm_camera_mt9e013",
		.owner = THIS_MODULE,
	},
};

static int __init mt9e013_init(void)
{
	printk(KERN_EMERG "[CAMERA]mt9e013_init\n");
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9e013_init);
void mt9e013_exit(void)
{
	i2c_del_driver(&mt9e013_i2c_driver);
}
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int mt9e013_set_af_codestep(void *data, u64 val)
{
	mt9e013_l_region_code_per_step = val;
	mt9e013_init_focus();
	return 0;
}

static int mt9e013_get_af_codestep(void *data, u64 *val)
{
	*val = mt9e013_l_region_code_per_step;
	return 0;
}

static int mt9e013_set_linear_total_step(void *data, u64 val)
{
	mt9e013_linear_total_step = val;
	return 0;
}

static int mt9e013_af_linearity_test(void *data, u64 *val)
{
	int i = 0;

	mt9e013_set_default_focus(0);
	msleep(3000);
	for (i = 0; i < mt9e013_linear_total_step; i++) {
		mt9e013_move_focus(MOVE_NEAR, 1);
		pr_err("__debug:MOVE_NEAR moved to index =[%d]\n", i);
	msleep(1000);
	}

	for (i = 0; i < mt9e013_linear_total_step; i++) {
		mt9e013_move_focus(MOVE_FAR, 1);
		CDBG("__debug:MOVE_FAR moved to index =[%d]\n", i);
		msleep(1000);
	}
	return 0;
}

static uint16_t mt9e013_step_jump = 4;
static uint8_t mt9e013_step_dir = MOVE_NEAR;
static int mt9e013_af_step_config(void *data, u64 val)
{
	mt9e013_step_jump = val & 0xFFFF;
	mt9e013_step_dir = (val >> 16) & 0x1;
	return 0;
}

static int mt9e013_af_step(void *data, u64 *val)
{
	int i = 0;
	int dir = MOVE_NEAR;
	mt9e013_set_default_focus(0);
	if (mt9e013_step_dir == 1)
		dir = MOVE_FAR;

	for (i = 1; i < MT9E013_TOTAL_STEPS_NEAR_TO_FAR; i+=mt9e013_step_jump) {
		mt9e013_move_focus(dir, mt9e013_step_jump);
		msleep(1000);
	}
	mt9e013_set_default_focus(0);
	return 0;
}

static int mt9e013_af_set_slew(void *data, u64 val)
{
	mt9e013_vcm_step_time = val & 0xFFFF;
	return 0;
}

static int mt9e013_af_get_slew(void *data, u64 *val)
{
	*val = mt9e013_vcm_step_time;
	return 0;
}

static int mt9e013_set_sw_damping(void *data, u64 val)
{
	mt9e013_sw_damping_time_wait = val;
	return 0;
}

static int mt9e013_get_sw_damping(void *data, u64 *val)
{
	*val = mt9e013_sw_damping_time_wait;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(af_damping, mt9e013_get_sw_damping,
			mt9e013_set_sw_damping, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_codeperstep, mt9e013_get_af_codestep,
	mt9e013_set_af_codestep, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_linear, mt9e013_af_linearity_test,
	mt9e013_set_linear_total_step, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_step, mt9e013_af_step,
	mt9e013_af_step_config, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_slew, mt9e013_af_get_slew,
	mt9e013_af_set_slew, "%llu\n");

static int mt9e013_focus_test(void *data, u64 *val)
{
	int i = 0;
	mt9e013_set_default_focus(0);

	for (i = 90; i < 256; i++) {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	msleep(5000);
	for (i = 255; i > 90; i--) {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_focus, mt9e013_focus_test,
			NULL, "%lld\n");

static int mt9e013_step_test(void *data, u64 *val)
{
	int i = 0;
	mt9e013_set_default_focus(0);

	for (i = 0; i < MT9E013_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		mt9e013_move_focus(MOVE_NEAR, 1);
		msleep(5000);
	}

	mt9e013_move_focus(MOVE_FAR, MT9E013_TOTAL_STEPS_NEAR_TO_FAR);
	msleep(5000);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_step, mt9e013_step_test,
			NULL, "%lld\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		mt9e013_start_stream();
		streaming = 1;
	} else {
		mt9e013_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");


static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("mt9e013", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("af_codeperstep", S_IRUGO | S_IWUSR, cam_dir,
		NULL, &af_codeperstep))
		return -ENOMEM;
	if (!debugfs_create_file("af_linear", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_linear))
		return -ENOMEM;
	if (!debugfs_create_file("af_step", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_step))
		return -ENOMEM;
	if (!debugfs_create_file("af_slew", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_slew))
		return -ENOMEM;
	if (!debugfs_create_file("af_damping", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_damping))
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;

	return 0;
}
