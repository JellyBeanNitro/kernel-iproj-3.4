/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
 */

#include "msm_sensor.h"
#include "msm_actuator.h"
#define SENSOR_NAME "imx105"
#define PLATFORM_DRIVER_NAME "msm_camera_imx105"
#define imx105_obj imx105_##obj

#ifdef CONFIG_LGE_SENSOR_IMX105

#ifdef CONFIG_MSM_CAMERA_DEBUG_IMX105
  #ifdef CDBG
  #undef CDBG
  #endif
  #define CDBG(fmt, args...) do { } while (0)
#else
  #ifdef CDBG
  #undef CDBG
  #endif
  #define CDBG(fmt, args...) do { } while (0)
#endif

#define 		USE_LG_fast_af
#endif

DEFINE_MUTEX(imx105_mut);
static struct msm_sensor_ctrl_t imx105_s_ctrl;

//                                                                  
#define REG_GROUPED_PARAMETER_HOLD			0x0104
#define GROUPED_PARAMETER_HOLD_OFF			0x00
#define GROUPED_PARAMETER_HOLD				0x01
#define REG_MODE_SELECT						0x0100
#define MODE_SELECT_STANDBY_MODE			0x00
#define MODE_SELECT_STREAM					0x01

/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME_HI		0x0202
#define REG_COARSE_INTEGRATION_TIME_LO		0x0203
/* Gain */
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_HI	0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LO	0x0205
/* mode setting */
#define REG_FRAME_LENGTH_LINES_HI   0x0340
#define REG_FRAME_LENGTH_LINES_LO	0x0341

/* AF Total steps parameters */
#define	IMX105_STEPS_NEAR_TO_CLOSEST_INF		32
#define	IMX105_TOTAL_STEPS_NEAR_TO_FAR			32
#define AF_STEP_RESOLUTION_VAL	0x04
#define AF_STEP_RESOLUTION_SETTING (AF_STEP_RESOLUTION_VAL << 5)
#define AF_SINGLESTEP_TIME_SETTING 0x14 /*1 msec*/
#define AF_SLEW_RATE_SPEED_SETING 0x2
#define AF_RESONANCE_FREQ_VAL 0x15 /*150 Hz*/
#define AF_RESONANCE_FREQ_SETING (AF_RESONANCE_FREQ_VAL << 3)
#define AF_SSTS_RS_VAL (AF_SINGLESTEP_TIME_SETTING | AF_STEP_RESOLUTION_SETTING)
#define AF_SRSS_RFS_VAL (AF_SLEW_RATE_SPEED_SETING | AF_RESONANCE_FREQ_SETING)

/*/*Intelligent slew rate control*/
#define AF_ISRC_MODE_ADDR (0xC4)
/*Slew rate speed setting and resonance frequency setting*/
#define AF_SRSS_RFS_VALUE_ADDR (0xC8)
/*Intelligent slew rate control un-control  DAC code 1*/
#define AF_DIRECT_MODE_ADDR (0xD0)
/*Intelligent slew rate control un-control  DAC code 2*/
#define AF_STEP_MODE_ADDR (0xDC)
/*Single Step Time Setting and Resolution Setting in Step Mode*/
#define AF_SSTS_RS_VALUE_ADDR (0xE0)
#define AF_MECH_INFINITY (0x03)
#define AF_OPTICAL_INFINITY (190)
#define AF_DISABLE (0x00)

//NR Register : shchang@qualcomm.com, 04019
#define REG_2DNR_ENABLE			0x30D0
#define REG_2DNR_GLINKINGSWITCH	0x30CF
#define REG_NR_STRENGTH 		0x3640
#define REG_NR_MFILTER_LEVEL	0x3620

uint16_t imx105_step_position_table[IMX105_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t imx105_l_region_code_per_step = 12;

/* 16bit address - 8 bit context register structure */
#define	IMX105_OFFSET	5
#define	Q8		0x00000100

/* Full	Size */
#define	IMX105_FULL_SIZE_WIDTH      3280
#define	IMX105_FULL_SIZE_HEIGHT		2464
#define	IMX105_HRZ_FULL_BLK_PIXELS	256
#define	IMX105_VER_FULL_BLK_LINES	46	//                                                       
#define	IMX105_FULL_SIZE_DUMMY_PIXELS	0
#define	IMX105_FULL_SIZE_DUMMY_LINES	0

/* Quarter Size	*/
#define	IMX105_QTR_SIZE_WIDTH	1640		/* PREVIEW 0x034C | 0x034D */
#define	IMX105_QTR_SIZE_HEIGHT	1232		/* PREVIEW 0x034D | 0x034F */
//                                                         
#define	IMX105_FHD_SIZE_WIDTH	3280
#define	IMX105_FHD_SIZE_HEIGHT	1232
#define	IMX105_HRZ_FHD_BLK_PIXELS	256
#define	IMX105_VER_FHD_BLK_LINES	36
//                                                       
#define	IMX105_QTR_SIZE_DUMMY_PIXELS	0
#define	IMX105_QTR_SIZE_DUMMY_LINES		0
#define	IMX105_HRZ_QTR_BLK_PIXELS	1896	/* (Preview) Frame width  (0x0342 | 0x0343) - CutOut Setting Size X Direction */
#define	IMX105_VER_QTR_BLK_LINES	38//34, shchang@qualcomm.com ,0419
// Delay for start stream and stop stream
static uint8_t imx105_delay_msecs_stdby = 5;
//static uint16_t imx105_delay_msecs_stream = 10;
//                                                                 

//                                                                  
#define IMX105_EEPROM_SLAVE_ADDR		(0x50 >> 1)
#define IMX105_AF_SLAVE_ADDR	(0x0C >> 1)
//                                                                 

//                                                                                          
static struct msm_camera_i2c_reg_conf imx105_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf imx105_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf imx105_groupon_settings[] = {
	{0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf imx105_groupoff_settings[] = {
	{0x0104, 0x00},
};

static struct msm_camera_i2c_reg_conf prev_settings[] = {
	{0x0101, 0x0003},//read out type : Flip & mirror
	{0x0340, 0x0004},
	{0x0341, 0x00F6},		//                                                           
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0346, 0x0000},
	{0x0347, 0x0024},
	{0x034A, 0x0009},
	{0x034B, 0x00C3},
	{0x034C, 0x0006},
	{0x034D, 0x0068},
	{0x034E, 0x0004},
	{0x034F, 0x00D0},
	{0x0381, 0x0001},
	{0x0383, 0x0003},
	{0x0385, 0x0001},
	{0x0387, 0x0003},
	{0x3033, 0x0000},
	{0x3048, 0x0001}, 
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00D2},
	{0x309B, 0x0028},
	{0x309E, 0x0000},
//	{0x30AA, 0x0002},
	{0x30D5, 0x0009},
	{0x30D6, 0x0001},
	{0x30D7, 0x0001},
	{0x30DE, 0x0002},
	{0x3102, 0x0008},
	{0x3103, 0x0022},
	{0x3104, 0x0020},
	{0x3105, 0x0000},
	{0x3106, 0x0087},
	{0x3107, 0x0000},
	{0x315C, 0x00A5},
	{0x315D, 0x00A4},
	{0x316E, 0x00A6},
	{0x316F, 0x00A5},
	{0x3318, 0x0072},
	{0x0202, 0x0004},
	{0x0203, 0x00ED}
};

static struct msm_camera_i2c_reg_conf snap_settings[] = {
	{0x0101, 0x0003},//read out type : Flip & mirror
	{0x0340, 0x0009},
	{0x0341, 0x00CE},	//                                                                                         
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0346, 0x0000},
	{0x0347, 0x0024},
	{0x034A, 0x0009},
	{0x034B, 0x00C3},
	{0x034C, 0x000C},
	{0x034D, 0x00D0},
	{0x034E, 0x0009},
	{0x034F, 0x00A0},
	{0x0381, 0x0001},
	{0x0383, 0x0001},
	{0x0385, 0x0001},
	{0x0387, 0x0001},
	{0x3033, 0x0000},
	{0x3048, 0x0000},
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00D2},
	{0x309B, 0x0020},
	{0x309E, 0x0000},
//	{0x30AA, 0x0002},
	{0x30D5, 0x0000},
	{0x30D6, 0x0085},
	{0x30D7, 0x002A},
	{0x30DE, 0x0000},
	{0x3102, 0x0008},
	{0x3103, 0x0022},
	{0x3104, 0x0020},
	{0x3105, 0x0000},
	{0x3106, 0x0087},
	{0x3107, 0x0000},
	{0x315C, 0x00A5},
	{0x315D, 0x00A4},
	{0x316E, 0x00A6},
	{0x316F, 0x00A5},
	{0x3318, 0x0062},
	{0x0202, 0x0009},
	{0x0203, 0x00E1}
};

static struct msm_camera_i2c_reg_conf FHD_settings[] = {
	{0x0101, 0x0003},//read out type : Flip & mirror
	{0x0340, 0x0004},
	{0x0341, 0x00F4},
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0344, 0x0000},
	{0x0345, 0x0004},
	{0x0346, 0x0001},
	{0x0347, 0x0058},
	{0x0348, 0x000C},
	{0x0349, 0x00D3},
	{0x034A, 0x0008},
	{0x034B, 0x008F},
	{0x034C, 0x000C},
	{0x034D, 0x00D0},
	{0x034E, 0x0004},
	{0x034F, 0x00D0},
	{0x0381, 0x0001},
	{0x0383, 0x0001},
	{0x0385, 0x0002},
	{0x0387, 0x0003},
	{0x303D, 0x0070},
	{0x303E, 0x0040},
	{0x3048, 0x0000}, 
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00F2},
	{0x309B, 0x0020},
	{0x309C, 0x0034},
//	{0x30AA, 0x0002},
	{0x30D5, 0x0000},
	{0x30D6, 0x0085},
	{0x30D7, 0x002A},
	{0x30D8, 0x0064},
	{0x30D9, 0x0089},
	{0x30DE, 0x0000},
	{0x3318, 0x0062}
};

static struct msm_camera_i2c_reg_conf recommend_settings[] = {
	{0x0305, 0x0001},
	{0x0307, 0x001C},
	{0x303C, 0x004B},
	{0x3031, 0x0010},
	{0x3064, 0x0012},
	{0x3087, 0x0057},
	{0x308A, 0x0035},
	{0x3091, 0x0041},
	{0x3098, 0x0003},
	{0x3099, 0x00C0},
	{0x309A, 0x00A3},
	{0x309C, 0x0034},
	{0x30AB, 0x0001},
	{0x30AD, 0x0008},
	{0x30F3, 0x0003},
	{0x3116, 0x0031},
	{0x3117, 0x0038},
	{0x3138, 0x0028},
	{0x3137, 0x0014},
	{0x3139, 0x002E},
	{0x314D, 0x002A},
	{0x3343, 0x0004},
	{0x3032, 0x0040}
	//{0x0101, 0x0003}
};
//                                                                                        

static struct sensor_extra_t {
	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
//	uint16_t my_reg_gain;
//	uint32_t my_reg_line_count;
//	uint16_t total_lines_per_frame;
} imx105_extra;
static struct sensor_extra_t * imx105_ctrl = &imx105_extra;

#ifdef CONFIG_LGE_SENSOR_IMX105
uint16_t af_optical_infinity_value = (uint16_t)AF_OPTICAL_INFINITY;
#endif

static uint16_t imx105_linear_total_step = IMX105_TOTAL_STEPS_NEAR_TO_FAR;
#if 0  // need to check by john.park
static uint16_t imx105_linear_total_step = IMX105_TOTAL_STEPS_NEAR_TO_FAR;
static uint16_t imx105_step_position_table[IMX105_TOTAL_STEPS_NEAR_TO_FAR+1];
//                                                 
static uint16_t af_infinity = 30;  // sungmin.woo : at least 64~64+96
static uint16_t imx105_nl_region_boundary1 = 0;
static uint16_t imx105_nl_region_code_per_step1 = 0;
static uint16_t imx105_l_region_code_per_step = 5;
static uint16_t imx105_vcm_step_time;
static uint16_t imx105_sw_damping_time_wait;
//                                                 
#endif 

static struct v4l2_subdev_info imx105_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array imx105_init_conf[] = {
	{&recommend_settings[0],
	ARRAY_SIZE(recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}, // MSM_CAMERA_I2C_WORD_DATA
};

static struct msm_camera_i2c_conf_array imx105_confs[] = {
	{&snap_settings[0],
	ARRAY_SIZE(snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&prev_settings[0],
	ARRAY_SIZE(prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&FHD_settings[0],
	ARRAY_SIZE(FHD_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

//                                                                  
static struct msm_sensor_output_info_t imx105_dimensions[] = {
	//(Ext=24MHz, vt_pix_clk=174MHz, op_pix_clk=69.6MHz)*/
	{ //MSM_SENSOR_RES_FULL
		.x_output = IMX105_FULL_SIZE_WIDTH,
		.y_output = IMX105_FULL_SIZE_HEIGHT,
		.line_length_pclk = 0x0DD0, 
		.frame_length_lines = 0x09CE, 
		.vt_pixel_clk = 134400000, // mo2jonghoo.lee 2013.02.01 330000000, //174000000,
		.op_pixel_clk = 134400000, // mo2jonghoo.lee 2013.02.01 69600000,
		.binning_factor = 1,	
	},
	{ //MSM_SENSOR_RES_QTR
		.x_output = IMX105_QTR_SIZE_WIDTH,
		.y_output = IMX105_QTR_SIZE_HEIGHT,
		.line_length_pclk = 0x0DD0, 
		.frame_length_lines = 0x04F6, // Imx111 -> 0x4E6, /* 1254 */
		.vt_pixel_clk = 134400000, //174000000,
		.op_pixel_clk = 67200000, // mo2jonghoo.lee 2013.02.01 69600000,
/*
[IMX105 JB] refer to mt9e013
.vt_pixel_clk = 192000000, //174000000,
.op_pixel_clk = 76800000, // mo2jonghoo.lee 2013.02.01 69600000,

[IMX111 sensor]
.x_output = 0x668, // 1640 
.y_output = 0x4D0, // 1232 
.line_length_pclk = 0xDD0, // 3536 
.frame_length_lines = 0x4E6, // 1254 

.vt_pixel_clk = 134400000,
.op_pixel_clk = 134400000,

[IMX105 ICS] in imx105_process_start()
// -----------------  Anti Banding Config ------------------ 
sensor->sensor.pixel_clock = 67200000;
sensor->sensor.pixel_clock_per_line = 3536;
--> matched with vt_pixel_clk
*/
		.binning_factor = 1,	
	},
	{ //MSM_SENSOR_RES_FHD // MSM_SENSOR_RES_2
		.x_output = IMX105_FHD_SIZE_WIDTH,
		.y_output = IMX105_FHD_SIZE_HEIGHT,
		.line_length_pclk = 0x0DD0, 
		.frame_length_lines = 0x04F4, 
		.vt_pixel_clk = 134400000, //174000000,
		.op_pixel_clk = 134400000, // mo2jonghoo.lee 2013.02.01 69600000,
		.binning_factor = 1,	
	},
};
//                                                                 

static struct msm_camera_csi_params imx105_csi_params = {
	.data_format = CSI_10BIT,
	.lane_cnt    = 2,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14, //                                                            
};

static struct msm_camera_csi_params *imx105_csi_params_array[] = {
	&imx105_csi_params,
	&imx105_csi_params, //NULL, //when res is FULL_SIZE, Nothing 
	&imx105_csi_params,
};

//                                                                                   
static struct msm_sensor_output_reg_addr_t imx105_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};
//                                                                                 

static struct msm_sensor_id_info_t imx105_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x0105,  //                                                            
};

//                                                                  
static struct msm_sensor_exp_gain_info_t imx105_exp_gain_info = {
	.coarse_int_time_addr = 0x0202, 
	.global_gain_addr = 0x0204, 
	.vert_offset = 5,
};
//                                                                


//                                                                  
static int16_t imx105_i2c_write_b_af(struct msm_camera_i2c_client* sensor_i2c_client, 
	uint8_t baddr,uint8_t bdata)
{
	int32_t rc;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	rc = msm_camera_i2c_txdata_manual(sensor_i2c_client->client->adapter, IMX105_AF_SLAVE_ADDR, buf, 2);
	if (rc < 0)
		CDBG("afi2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!",
			IMX105_AF_SLAVE_ADDR, baddr, bdata);
	return rc;
}
//                                                                 

//                                                                  
static int32_t imx105_i2c_read_w_eeprom(struct msm_camera_i2c_client* sensor_i2c_client, 
	uint16_t reg_addr, uint16_t *rdata)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;

	//Read 2 bytes in sequence 
	//Big Endian address:
	buf[0] = (reg_addr & 0x00FF);
	buf[1] = (reg_addr & 0xFF00) >> 8;
	rc = msm_camera_i2c_rxdata_manual(sensor_i2c_client->client->adapter, IMX105_EEPROM_SLAVE_ADDR<<1, buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]1 :imx105_i2c_read_eeprom 0x%x failed!\n", reg_addr);
		return rc;
	}
	*rdata = buf[0];
	reg_addr += 1;

	/* Read Second byte of data */
	buf[0] = (reg_addr & 0x00FF);
	buf[1] = (reg_addr & 0xFF00) >> 8;
	rc = msm_camera_i2c_rxdata_manual(sensor_i2c_client->client->adapter, IMX105_EEPROM_SLAVE_ADDR<<1, buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]2 :imx105_i2c_read_eeprom 0x%x failed!\n", reg_addr);
		return rc;
	}
	*rdata = (*rdata << 8) | buf[0];
	return rc;
}

static int imx105_read_eeprom_data(struct msm_sensor_ctrl_t *s_ctrl,	struct sensor_cfg_data *cfg)
{
	int32_t rc = 0;
	uint16_t eepromdata = 0;
	//uint8_t addr = 0;
	uint16_t addr = 0;
#ifdef USE_LG_fast_af
	uint8_t i;
	uint16_t fastaf_stepsize = 0;
	//uint16_t fastaf_totalstepmargin = 1;
	uint16_t fastaf_infinitymargin = 30;
	uint16_t fastaf_macromargin = 30;
#endif
	CDBG("[QCTK_EEPROM] Start reading EEPROM\n");

	//Start for Read debigging
	addr = 0x0000;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	
	CDBG("[QCTK_EEPROM] Product version = 0x%x\n", eepromdata);	

	//End for Read debugging
	
	addr = 0x0010;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.r_over_g = eepromdata;
	CDBG("[QCTK_EEPROM] r_over_g = 0x%4x\n", cfg->cfg.calib_info.r_over_g);	

	addr = 0x0012;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.b_over_g = eepromdata;
	CDBG("[QCTK_EEPROM] b_over_g = 0x%4x\n", cfg->cfg.calib_info.b_over_g);	


	addr = 0x0014;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.gr_over_gb = eepromdata;

#ifdef USE_LG_fast_af
	// infinity af position
	addr = 0x001A;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.af_pos_inf = eepromdata;
	CDBG("[QCTK_EEPROM] af_pos_inf = %d\n", cfg->cfg.calib_info.af_pos_inf);	
	
	// 1m af position
	addr = 0x1C; // sungmin.woo
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.af_pos_1m = eepromdata;
	CDBG("[QCTK_EEPROM] af_pos_1m = %d\n", cfg->cfg.calib_info.af_pos_1m);	
	
	// 100mm(macro) af position
	addr = 0x1E; // sungmin.woo let's use this for 100mm af position
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.stroke_amt = eepromdata;
	CDBG("[QCTK_EEPROM] macro pos = %d\n", cfg->cfg.calib_info.stroke_amt);	

	//remaining parameter

	cfg->cfg.calib_info.macro_2_inf =  cfg->cfg.calib_info.stroke_amt - cfg->cfg.calib_info.af_pos_inf;
	CDBG("[QCTK_EEPROM] macro_2_inf = %d\n", cfg->cfg.calib_info.macro_2_inf);	
	cfg->cfg.calib_info.inf_2_macro = cfg->cfg.calib_info.macro_2_inf;
	CDBG("[QCTK_EEPROM] inf_2_macro = %d\n", cfg->cfg.calib_info.inf_2_macro);	

	
	CDBG("Reset focus range from eeprom for fast af\n");	
	CDBG("**********************************\n");
	fastaf_infinitymargin = (uint16_t)(cfg->cfg.calib_info.af_pos_inf/10);
	fastaf_macromargin = fastaf_infinitymargin;
	CDBG("fastaf_infinitymargin = %d\n", fastaf_infinitymargin);	
	fastaf_stepsize =  (uint16_t)((cfg->cfg.calib_info.macro_2_inf + fastaf_infinitymargin + fastaf_macromargin )/(IMX105_TOTAL_STEPS_NEAR_TO_FAR));
	imx105_step_position_table[0] = cfg->cfg.calib_info.af_pos_inf - fastaf_infinitymargin;
	imx105_l_region_code_per_step = fastaf_stepsize;
	CDBG("imx105_l_region_code_per_step = %d\n", fastaf_stepsize);	
	
	for (i = 1; i <= IMX105_TOTAL_STEPS_NEAR_TO_FAR; i++)
	{	
		imx105_step_position_table[i] = imx105_step_position_table[i-1] + imx105_l_region_code_per_step;
	}

	for(i = 0; i <= IMX105_TOTAL_STEPS_NEAR_TO_FAR; i++)
	{	
		CDBG("imx105_step_position_table[%d] = [%d]\n",i, imx105_step_position_table[i]);	
	}

	af_optical_infinity_value = (uint16_t)cfg->cfg.calib_info.af_pos_inf - (uint16_t)fastaf_infinitymargin;
	//AF_OPTICAL_INFINITY = 0;
	CDBG("AF_OPTICAL_INFINITY = %d\n", af_optical_infinity_value);	
	
	CDBG("**********************************\n");	

#else
	//: Infinite AF Position
	addr = 0x001A;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.macro_2_inf = eepromdata;

	//: 1m AF Position
	addr = 0x001C;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.inf_2_macro = eepromdata;

	// : 10cm AF Position
	addr = 0x001E;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.stroke_amt = eepromdata;

	// : Start current
	addr = 0x0020;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.af_pos_1m = eepromdata;

	// : Operating sensitivity
	addr = 0x0022;
	rc = imx105_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, addr, &eepromdata);
	if (rc < 0) {
		CDBG("%s: Error Reading EEPROM @ 0x%x\n", __func__, addr);
		return rc;
	}
	cfg->cfg.calib_info.af_pos_inf = eepromdata;
#endif
	return rc;
}
//                                                                 

#if 0  //                                                                                 
static int32_t imx105_get_pict_fps(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	int32_t rc = 0;
	uint32_t divider, d1, d2;

	d1 = prev_settings[E013_FRAME_LENGTH_LINES].reg_data * 0x00000400
		/ snap_settings[E013_FRAME_LENGTH_LINES].reg_data;
	d2 = prev_settings[E013_LINE_LENGTH_PCK].reg_data * 0x00000400
		/ snap_settings[E013_LINE_LENGTH_PCK].reg_data;
	divider = d1 * d2 / 0x400;
	CDBG("[CAMERA]imx105_get_pict_fps: divider = %d, d1 = %d, d2 = %d \n", divider, d1, d2);

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */

	CDBG("[CAMERA]imx105_get_pict_fps:fps = %d, pfps = %d\n", fps, *pfps);
	return rc;
}

static int32_t imx105_get_prev_lines_pf(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_prevl_pf)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_get_prev_lines_pf\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_prevl_pf = prev_settings[E013_FRAME_LENGTH_LINES].reg_data;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_prevl_pf = FHD_settings[E013_FRAME_LENGTH_LINES].reg_data;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_prevl_pf = snap_settings[E013_FRAME_LENGTH_LINES].reg_data;

	return rc;
}

static int32_t imx105_get_prev_pixels_pl(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_prevp_pl)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_get_prev_pixels_pl\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_prevp_pl = prev_settings[E013_LINE_LENGTH_PCK].reg_data;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_prevp_pl = FHD_settings[E013_LINE_LENGTH_PCK].reg_data;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_prevp_pl = snap_settings[E013_LINE_LENGTH_PCK].reg_data;

	return rc;	
}

static int32_t imx105_get_pict_lines_pf(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_pictl_pf)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_get_pict_lines_pf\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pictl_pf = prev_settings[E013_FRAME_LENGTH_LINES].reg_data;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pictl_pf = FHD_settings[E013_FRAME_LENGTH_LINES].reg_data;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pictl_pf = snap_settings[E013_FRAME_LENGTH_LINES].reg_data;

	return rc;		
}

static int32_t imx105_get_pict_pixels_pl(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_pictp_pl)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_get_pict_pixels_pl\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pictp_pl = prev_settings[E013_LINE_LENGTH_PCK].reg_data;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pictp_pl = FHD_settings[E013_LINE_LENGTH_PCK].reg_data;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pictp_pl = snap_settings[E013_LINE_LENGTH_PCK].reg_data;

	return rc;			
}

static int32_t imx105_get_pict_max_exp_lc(struct msm_sensor_ctrl_t *s_ctrl,
	uint32_t *p_pict_max_exp_lc)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_get_pict_max_exp_lc\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pict_max_exp_lc = prev_settings[E013_FRAME_LENGTH_LINES].reg_data * 24;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pict_max_exp_lc = FHD_settings[E013_FRAME_LENGTH_LINES].reg_data * 24;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pict_max_exp_lc = snap_settings[E013_FRAME_LENGTH_LINES].reg_data * 24;

	return rc;			
}
#endif

static int32_t imx105_get_af_max_steps(struct msm_sensor_ctrl_t *s_ctrl,
	uint8_t *pmax_steps)
{
	int32_t rc = 0;

	*pmax_steps = (uint8_t) imx105_linear_total_step;
	
	return rc;			
}

#if 1 //tongting_0311
static int32_t imx105_set_fps(struct msm_sensor_ctrl_t *s_ctrl,
						struct fps_cfg *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	s_ctrl->fps_divider = fps->fps_div;
//	s_ctrl->pict_fps_divider = fps->pict_fps_div;  // john.park 2013.02.27  compile error fix

	if (s_ctrl->curr_res  == MSM_SENSOR_RES_QTR)
		total_lines_per_frame = (uint16_t)(((IMX105_QTR_SIZE_HEIGHT +
		IMX105_VER_QTR_BLK_LINES) *
		s_ctrl->fps_divider) / 0x400);
	else if(s_ctrl->curr_res  == MSM_SENSOR_RES_2)		//shchang@qualcomm.com for FHD fixed FPS
		total_lines_per_frame = (uint16_t)(((IMX105_FHD_SIZE_HEIGHT +
		IMX105_VER_FHD_BLK_LINES) *
		s_ctrl->fps_divider) / 0x400);		
	else
		total_lines_per_frame = (uint16_t)(((IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES) *
			s_ctrl->fps_divider) / 0x400); /* s_ctrl->pict_fps_divider) / 0x400 */// john.park 2013.02.27  compile error fix

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_FRAME_LENGTH_LINES_HI, 
		((total_lines_per_frame & 0xFF00) >> 8), MSM_CAMERA_I2C_BYTE_DATA);	
//	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_HI,((total_lines_per_frame & 0xFF00) >> 8));

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_FRAME_LENGTH_LINES_LO, 
		(total_lines_per_frame & 0x00FF), MSM_CAMERA_I2C_BYTE_DATA);	
//	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_LO,(total_lines_per_frame & 0x00FF));

	return rc;
}
#endif

#if 1 //tongting_0311
static int32_t imx105_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
//	static uint16_t max_legal_gain  = 0x00E0;
	static uint16_t max_legal_gain  = 0x00EA;	
	uint8_t gain_msb, gain_lsb;
	uint8_t intg_time_msb, intg_time_lsb;
	uint8_t frame_length_line_msb, frame_length_line_lsb;
	uint16_t frame_length_lines;
	int32_t rc = -1;

//                                                     
//                                                      
	CDBG("imx105_write_exp_gain : gain = %d line = %d", gain, line);
	if (s_ctrl->curr_res  == MSM_SENSOR_RES_QTR) {
		frame_length_lines = IMX105_QTR_SIZE_HEIGHT +
			IMX105_VER_QTR_BLK_LINES;
		frame_length_lines = frame_length_lines *
			s_ctrl->fps_divider / 0x400;
	}
//                                                               
	else if (s_ctrl->curr_res  == MSM_SENSOR_RES_2) {
		frame_length_lines = IMX105_FHD_SIZE_HEIGHT +
			IMX105_VER_FHD_BLK_LINES;
	frame_length_lines = frame_length_lines *
		s_ctrl->fps_divider / 0x400;
	}
//                                                            
	else {
		frame_length_lines = IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES;
		frame_length_lines = frame_length_lines *
			s_ctrl->fps_divider / 0x400;			
//			s_ctrl->pict_fps_divider / 0x400;  // john.park 2013.02.27  compile error fix
	}
//                                                    
//                                                     

	if (line > (frame_length_lines - IMX105_OFFSET))
		frame_length_lines = line + IMX105_OFFSET;

	CDBG("imx105 setting line = %d\n", line);

	CDBG("imx105 setting frame_length_lines = %d\n", frame_length_lines);

	if (gain > max_legal_gain)
		/* range: 0 to 224 */
		gain = max_legal_gain;

	CDBG("[QCTK]imx105_write_exp_gain : gain = %d line = %d", gain, line);	//shchang@qualcomm.com

	/* update gain registers */
	gain_msb = (uint8_t) ((gain & 0xFF00) >> 8);
	gain_lsb = (uint8_t) (gain & 0x00FF);

	frame_length_line_msb = (uint8_t) ((frame_length_lines & 0xFF00) >> 8);
	frame_length_line_lsb = (uint8_t) (frame_length_lines & 0x00FF);

	/* update line count registers */
	intg_time_msb = (uint8_t) ((line & 0xFF00) >> 8);
	intg_time_lsb = (uint8_t) (line & 0x00FF);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_GROUPED_PARAMETER_HOLD, 
		GROUPED_PARAMETER_HOLD, MSM_CAMERA_I2C_BYTE_DATA);	
//	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_ANALOGUE_GAIN_CODE_GLOBAL_HI = 0x%X\n", gain_msb);
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_ANALOGUE_GAIN_CODE_GLOBAL_HI, 
		gain_msb, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_HI, gain_msb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_ANALOGUE_GAIN_CODE_GLOBAL_LO = 0x%X\n", gain_lsb);
	
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_ANALOGUE_GAIN_CODE_GLOBAL_LO, 
		gain_lsb, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_LO, gain_lsb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_FRAME_LENGTH_LINES_HI = 0x%X\n",frame_length_line_msb);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_FRAME_LENGTH_LINES_HI, 
		frame_length_line_msb, MSM_CAMERA_I2C_BYTE_DATA);	
//	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_HI,frame_length_line_msb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_FRAME_LENGTH_LINES_LO = 0x%X\n", frame_length_line_lsb);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_FRAME_LENGTH_LINES_LO, 
		frame_length_line_lsb, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_LO, frame_length_line_lsb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_COARSE_INTEGRATION_TIME_HI = 0x%X\n", intg_time_msb);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_COARSE_INTEGRATION_TIME_HI, 
		intg_time_msb, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_HI,intg_time_msb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_COARSE_INTEGRATION_TIME_LO = 0x%X\n", intg_time_lsb);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_COARSE_INTEGRATION_TIME_LO, 
		intg_time_lsb, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_LO, intg_time_lsb);
	if (rc < 0)
		return rc;
	//                                                      

	//real_gain = (float)(256.0 / (256.0 - (float)gain));
	CDBG("[QCTK] Gain = %d\n", gain);
	//Gain : 0~234
	//Just use Sony 2DNR option ~15 ~ 127
	if(gain > 220)
	{
		CDBG("Func : NoiseReduction - Writing low light NR Value..\n");

		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_NR_STRENGTH, 
			0x52, MSM_CAMERA_I2C_BYTE_DATA);		
//		rc = imx105_i2c_write_b_sensor(REG_NR_STRENGTH,0x52);	//7f, 82
	}
	else if(gain > 200)
	{
		CDBG("Func : NoiseReduction - Writing indoor NR Value..\n");
		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_NR_STRENGTH, 
			0x29, MSM_CAMERA_I2C_BYTE_DATA);		
//		rc = imx105_i2c_write_b_sensor(REG_NR_STRENGTH,0x29);	//52
	}
	else if(gain > 150)
	{
		CDBG("Func : NoiseReduction - Writing bright2 light NR Value..\n");

		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_NR_STRENGTH, 
			0x1D, MSM_CAMERA_I2C_BYTE_DATA);
//		rc = imx105_i2c_write_b_sensor(REG_NR_STRENGTH,0x1D);	//25, //29
	}
	else if(gain > 80)
	{
		CDBG("Func : NoiseReduction - Writing bright1 light NR Value..\n");

		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_NR_STRENGTH, 
			0x10, MSM_CAMERA_I2C_BYTE_DATA);
//		rc = imx105_i2c_write_b_sensor(REG_NR_STRENGTH,0x10);	//15
	}
	else 
	{
		CDBG("Func : NoiseReduction - Writing bright1 light NR Value..\n");
		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_NR_STRENGTH, 
			0x0F, MSM_CAMERA_I2C_BYTE_DATA);
//		rc = imx105_i2c_write_b_sensor(REG_NR_STRENGTH,0x0F);	//15
	}

	if(rc<0)
	{
		CDBG("Func : NoiseReduction - Writing NR Value is failed..\n");
		return rc;
	}
	//                                                     

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_GROUPED_PARAMETER_HOLD, 
		GROUPED_PARAMETER_HOLD_OFF, MSM_CAMERA_I2C_BYTE_DATA);
//	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD_OFF);
	if (rc < 0)
		return rc;

	return rc;
}
#endif

#if 0
static int32_t imx105_write_exp_snapshot_gain(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CDBG("[CAMERA]imx105_set_pict_exp_gain : gain = %d,line = %d \n", gain, line);

	if (s_ctrl->func_tbl->sensor_write_exp_gain)
		rc = s_ctrl->func_tbl->sensor_write_exp_gain(s_ctrl, gain, line);
		
	return rc;
}
#endif

#if 0  //                                                                                 
//                                                                
static int32_t imx105_move_focus(struct msm_sensor_ctrl_t *s_ctrl,
	int direction, int32_t num_steps)
{

	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	uint8_t codeval_msb, codeval_lsb;
	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if (direction == MOVE_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction\n");
		return -EINVAL;
	}

	dest_step_position = imx105_ctrl->curr_step_pos +
			(step_direction * num_steps);
	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > IMX105_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = IMX105_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == imx105_ctrl->curr_step_pos) {
		CDBG("imx105_move_focus ==  imx105_ctrl->curr_step_pos:exit\n");
		return rc;
	}
	dest_lens_position = imx105_step_position_table[dest_step_position];
	CDBG("%s line %d index =[%d] = value = %d\n",
		 __func__, __LINE__, dest_step_position, dest_lens_position);
	if (imx105_ctrl->curr_lens_pos != dest_lens_position) {
		codeval_msb = (AF_ISRC_MODE_ADDR |
					((dest_lens_position & 0x0300)>>8));
		codeval_lsb = dest_lens_position & 0x00FF;
		rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, codeval_msb, codeval_lsb);
		if (rc < 0) {
			CDBG("imx105 I2C Failed line %d\n", __LINE__);
			return rc;
		}
		//usleep(10);  
		usleep(12); //                                               
	}
	imx105_ctrl->curr_lens_pos = dest_lens_position;
	imx105_ctrl->curr_step_pos = dest_step_position;

	return rc;
}	
//                                                              
#endif

//                                                                
static int32_t imx105_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int direction = move_params->dir;
	int32_t num_steps = move_params->num_steps;

	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	uint8_t codeval_msb, codeval_lsb;
	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if (direction == MOVE_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction\n");
		return -EINVAL;
	}

	CDBG("%s direction = %d, num_steps = %d, curr_step_pos = %d, curr_lens_pos = %d\n",
		 __func__, direction, num_steps, imx105_ctrl->curr_step_pos, imx105_ctrl->curr_lens_pos);

	dest_step_position = imx105_ctrl->curr_step_pos +
			(step_direction * num_steps);
	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > IMX105_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = IMX105_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == imx105_ctrl->curr_step_pos) {
		CDBG("imx105_move_focus ==  imx105_ctrl->curr_step_pos:exit\n");
		return rc;
	}
	dest_lens_position = imx105_step_position_table[dest_step_position];
	CDBG("%s index =[%d] = value = %d, curr_lens_pos = %d\n",
		 __func__, dest_step_position, dest_lens_position, imx105_ctrl->curr_lens_pos);
	if (imx105_ctrl->curr_lens_pos != dest_lens_position) {
		codeval_msb = (AF_ISRC_MODE_ADDR |
					((dest_lens_position & 0x0300)>>8));
		codeval_lsb = dest_lens_position & 0x00FF;
		rc = imx105_i2c_write_b_af(&a_ctrl->i2c_client, codeval_msb, codeval_lsb);
		if (rc < 0) {
			CDBG("imx105 I2C Failed line %d\n", __LINE__);
			return rc;
		}
		//usleep(10);  
		usleep(12); //                                               
	}
	imx105_ctrl->curr_lens_pos = dest_lens_position;
	imx105_ctrl->curr_step_pos = dest_step_position;

	a_ctrl->curr_step_pos = imx105_ctrl->curr_step_pos;

	return rc;
}
//                                                              

#if 0  //                                                                                 
//                                                                
static int32_t imx105_set_default_focus(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	CDBG("%s ==  enter\n", __func__);
	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_DIRECT_MODE_ADDR,
				AF_MECH_INFINITY);
	if (rc < 0)
		return rc;
	usleep(50);
	CDBG("%s defaultfocus directmode\n", __func__);
	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_STEP_MODE_ADDR,
				AF_OPTICAL_INFINITY);
	if (rc < 0)
		return rc;
	msleep(50);
	CDBG("%s defaultfocus step mode\n", __func__);
	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_ISRC_MODE_ADDR,
			AF_OPTICAL_INFINITY);
	if (rc < 0)
		return rc;

	CDBG("%s ==  exit\n", __func__);
	imx105_ctrl->curr_lens_pos = AF_OPTICAL_INFINITY;
	imx105_ctrl->curr_step_pos = 0;

	return rc;
}
//                                                              
#endif

//                                                                
static int32_t imx105_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	CDBG("%s ==  enter\n", __func__);
	rc = imx105_i2c_write_b_af(&a_ctrl->i2c_client, AF_DIRECT_MODE_ADDR,
				AF_MECH_INFINITY);
	if (rc < 0)
		return rc;
	usleep(50);
	CDBG("%s defaultfocus directmode\n", __func__);
	rc = imx105_i2c_write_b_af(&a_ctrl->i2c_client, AF_STEP_MODE_ADDR,
				af_optical_infinity_value);
	if (rc < 0)
		return rc;
	msleep(50);
	CDBG("%s defaultfocus step mode\n", __func__);
	rc = imx105_i2c_write_b_af(&a_ctrl->i2c_client, AF_ISRC_MODE_ADDR,
			af_optical_infinity_value);
	if (rc < 0)
		return rc;

	CDBG("%s ==  exit\n", __func__);
	imx105_ctrl->curr_lens_pos = af_optical_infinity_value;
	imx105_ctrl->curr_step_pos = 0;

	a_ctrl->curr_step_pos = imx105_ctrl->curr_step_pos;

	return rc;
}
//                                                              

struct msm_actuator msm_actuator_table_imx105 = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = imx105_actuator_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus = imx105_actuator_set_default_focus,
		.actuator_init_focus = NULL,
		.actuator_i2c_write = NULL,
	},
};

//                                                                
static int16_t imx105_init_focus(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint8_t i;

	CDBG("[CAMERA]imx105_init_focus\n");
	
	imx105_step_position_table[0] = af_optical_infinity_value;
	for (i = 1; i <= IMX105_TOTAL_STEPS_NEAR_TO_FAR; i++)
			imx105_step_position_table[i] =
			imx105_step_position_table[i-1] +
			imx105_l_region_code_per_step;

	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_SRSS_RFS_VALUE_ADDR,
				AF_SRSS_RFS_VAL);
	if (rc < 0){
		pr_err("[JJONG]imx105_init_focus af i2c failed 1\n");
		return rc;
	}
	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_SSTS_RS_VALUE_ADDR,
				AF_SSTS_RS_VAL);
	if (rc < 0){
		pr_err("[JJONG]imx105_init_focus af i2c failed 2\n");
		return rc;
	}
	
	return rc;	
}
//                                                                 

static int32_t imx105_power_up(struct msm_sensor_ctrl_t *s_ctrl) 
{
	int32_t rc = 0;
	
	memset(imx105_ctrl, 0x00, sizeof(*imx105_ctrl));
	s_ctrl->fps_divider = 1 * 0x00000400;
//No on JB	imx105_ctrl->pict_fps_divider = 1 * 0x00000400;
//No on JB	imx105_ctrl->set_test = TEST_OFF;
//	imx105_ctrl->prev_res = QTR_SIZE;
	s_ctrl->curr_res = MSM_SENSOR_RES_QTR /* QTR_SIZE */;
//No on JB	imx105_ctrl->pict_res = FULL_SIZE;	
	rc = msm_sensor_power_up(s_ctrl);

	rc = imx105_init_focus(s_ctrl);

	if(rc < 0)
		pr_err("[JJONG] imx105_power_up FAILED\n");

	return rc;
}

static int32_t imx105_power_down(struct msm_sensor_ctrl_t * s_ctrl)
{
	int32_t rc = 0;

	printk(KERN_EMERG "[CAMERA]imx105_power_down\n");

//                                                                 
	rc = imx105_i2c_write_b_af(s_ctrl->sensor_i2c_client, AF_DISABLE,
			AF_DISABLE);
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,REG_MODE_SELECT, 
		MODE_SELECT_STANDBY_MODE, MSM_CAMERA_I2C_BYTE_DATA);	
	
	msleep(imx105_delay_msecs_stdby);
//                                                                 

	return msm_sensor_power_down(s_ctrl);
}

static int32_t imx105_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
#if 0		
		case CFG_SET_EFFECT:
			rc = -EFAULT;
			break;
#endif
		case CFG_GET_CALIB_DATA:
//                                                                     
			rc = imx105_read_eeprom_data(s_ctrl, &cdata);
//                                                                 
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;					
			break;
#if 0		//                                                                                 
		case CFG_MOVE_FOCUS:
			rc = imx105_move_focus(s_ctrl, 
				cdata.cfg.focus.dir, 
				cdata.cfg.focus.steps);			
			break;

		case CFG_SET_DEFAULT_FOCUS:		
			rc = imx105_set_default_focus(s_ctrl);			
			break;	
			
		case CFG_GET_PICT_FPS:
			rc = imx105_get_pict_fps(s_ctrl,
				cdata.cfg.gfps.prevfps,
				&cdata.cfg.gfps.pictfps);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;			
			break;			

		case CFG_GET_PREV_L_PF:
			rc = imx105_get_prev_lines_pf(s_ctrl,
				&cdata.cfg.prevl_pf);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;				
			break;

		case CFG_GET_PREV_P_PL:
			rc = imx105_get_prev_pixels_pl(s_ctrl,
				&cdata.cfg.prevp_pl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;						
			break;

		case CFG_GET_PICT_L_PF:
			rc = imx105_get_pict_lines_pf(s_ctrl,
				&cdata.cfg.pictl_pf);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;						
			break;

		case CFG_GET_PICT_P_PL:
			rc = imx105_get_pict_pixels_pl(s_ctrl,
				&cdata.cfg.pictp_pl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;				
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			rc = imx105_get_pict_max_exp_lc(s_ctrl,
				&cdata.cfg.pict_max_exp_lc);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;					
			break;
#endif
		case CFG_GET_AF_MAX_STEPS:
			rc = imx105_get_af_max_steps(s_ctrl,
				&cdata.max_steps);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;					
			break;

		default:
			rc = -EINVAL;
			break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	if (rc == -EINVAL) {
		rc = msm_sensor_config(s_ctrl, argp);
	}
	
	return rc;
}

static const struct i2c_device_id imx105_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&imx105_s_ctrl},
	{ }
};

static struct i2c_driver imx105_i2c_driver = {
	.id_table = imx105_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx105_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&imx105_i2c_driver);
}

static struct v4l2_subdev_core_ops imx105_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops imx105_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops imx105_subdev_ops = {
	.core = &imx105_subdev_core_ops,
	.video  = &imx105_subdev_video_ops,
};

static struct msm_sensor_fn_t imx105_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,//imx105_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,//imx105_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
/*
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
*/
	.sensor_set_fps = imx105_set_fps, //tongting_0311
    .sensor_write_exp_gain = imx105_write_exp_gain,
    .sensor_write_snapshot_exp_gain = imx105_write_exp_gain,//imx105_write_exp_snapshot_gain,
/*
.sensor_set_fps = imx105_set_fps,
.sensor_write_exp_gain = imx105_write_exp_gain,
.sensor_write_snapshot_exp_gain = imx105_write_exp_snapshot_gain,
[Imx091 ºæº≠]
.sensor_set_fps = msm_sensor_set_fps,
.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
*/
//                                                                               
	.sensor_csi_setting = msm_sensor_setting,
//                                                                                                             
//                                                                             
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = imx105_config,
	.sensor_power_up = imx105_power_up,
	.sensor_power_down = imx105_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t imx105_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = imx105_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(imx105_start_settings),
	.stop_stream_conf = imx105_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(imx105_stop_settings),
	.group_hold_on_conf = imx105_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(imx105_groupon_settings),
	.group_hold_off_conf = imx105_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(imx105_groupoff_settings),
	.init_settings = &imx105_init_conf[0],
	.init_size = ARRAY_SIZE(imx105_init_conf),
	.mode_settings = &imx105_confs[0],
	.output_settings = &imx105_dimensions[0],
	.num_conf = ARRAY_SIZE(imx105_confs),
};

static struct msm_sensor_ctrl_t imx105_s_ctrl = {
	.msm_sensor_reg = &imx105_regs,
	.sensor_i2c_client = &imx105_sensor_i2c_client,
	.sensor_i2c_addr = 0x34, //0x34, Real salve address : 0x34>> 1 ( 0x1A ¿”.) 
	.sensor_output_reg_addr = &imx105_reg_addr,
	.sensor_id_info = &imx105_id_info,
	.sensor_exp_gain_info = &imx105_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &imx105_csi_params_array[0],
	.msm_sensor_mutex = &imx105_mut,
	.sensor_i2c_driver = &imx105_i2c_driver,
	.sensor_v4l2_subdev_info = imx105_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx105_subdev_info),
	.sensor_v4l2_subdev_ops = &imx105_subdev_ops,
	.func_tbl = &imx105_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

//module_init(msm_sensor_init_module);
late_initcall(msm_sensor_init_module);
MODULE_DESCRIPTION("Sony 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");


