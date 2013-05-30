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
#define SENSOR_NAME "mt9p017"
#define PLATFORM_DRIVER_NAME "msm_camera_mt9p017"
#define mt9p017_obj mt9p017_##obj

DEFINE_MUTEX(mt9p017_mut);
static struct msm_sensor_ctrl_t mt9p017_s_ctrl;

enum mt9p017_reg_mode {
	E013_X_ADDR_START,
	E013_X_ADDR_END,
	E013_Y_ADDR_START,
	E013_Y_ADDR_END,
	E013_X_OUTPUT_SIZE,
	E013_Y_OUTPUT_SIZE,
	E013_DATAPATH_SELECT,
	E013_READ_MODE,
	E013_ANALOG_CONTROL5,
	E013_DAC_LD_4_5,
	E013_SCALING_MODE,
	E013_SCALE_M,
	E013_LINE_LENGTH_PCK,
	E013_FRAME_LENGTH_LINES,
	E013_COARSE_INTEGRATION_TIME,
	E013_FINE_INTEGRATION_TIME,
	E013_FINE_CORRECTION
};

#define MT9P017_MODEL_ID     		 0x4800 // mo2jonghoo.lee 2012.12.27 

/* AF Total steps parameters */
// mo2jonghoo.lee 2012.12.27 #define MT9E013_TOTAL_STEPS_NEAR_TO_FAR		32
#define MT9P017_TOTAL_STEPS_NEAR_TO_FAR    32 // mo2jonghoo.lee 2012.12.27 

/* PLL registers */
// mo2jonghoo.lee 2012.12.27 #define REG_FRAME_LENGTH_LINES		0x0340
// mo2jonghoo.lee 2012.12.27 #define REG_VCM_CONTROL			0x30F0
// mo2jonghoo.lee 2012.12.27 #define REG_VCM_NEW_CODE			0x30F2
// mo2jonghoo.lee 2012.12.27 #define REG_VCM_STEP_TIME			0x30F4 
// mo2jonghoo.lee 2012.12.27 #define REG_LENS_SHADING	        		0x3780
#define REG_FRAME_LENGTH_LINES  0x300A // mo2jonghoo.lee 2012.12.27 
#define REG_COARSE_INT_TIME     0x3012 // mo2jonghoo.lee 2012.12.27 
#define REG_GLOBAL_GAIN         0x305E // mo2jonghoo.lee 2012.12.27 
#define REG_LENS_SHADING    		0x3780 // mo2jonghoo.lee 2012.12.27

#define REG_RESET_REGISTER      0x301A
#define REG_FAST_TRANS_MODE_ON  0x065E

#define REG_VCM_CONTROL					0x30F0 // mo2jonghoo.lee 2012.12.27 
#define REG_VCM_NEW_CODE				0x30F2 // mo2jonghoo.lee 2012.12.27 
#define REG_VCM_STEP_TIME				0x30F4 // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 


#define LSC_ON						1 // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 
#define LSC_OFF 					0 // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 

#define MT9P017_EEPROM_SLAVE_ADDR	0xA0>>1 	//EEPROM Slave Address for 5100K(Page #1) // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 
#define LSC_FIRST_PHASE1_DATA_SIZE	(5) // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 
#define LSC_FIRST_PHASE2_DATA_SIZE	(20) // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 
#define LSC_SECOND_PHASE_DATA_SIZE	(2) // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 
#define LSC_THIRD_PHASE_DATA_SIZE	(4) // mo2jonghoo.lee 2012.12.27 mt9p017에는 없음. 

/* // mo2jonghoo.lee 2012.12.27 
#define MT9E013_QTR_SIZE_WIDTH 			(1640)
#define MT9E013_QTR_SIZE_HEIGHT 			(1232)
#define MT9E013_QTR_SIZE_DUMMY_PIXELS 	(0)
#define MT9E013_QTR_SIZE_DUMMY_LINES	(0)

#define MT9E013_FULL_SIZE_WIDTH			(3280)
#define MT9E013_FULL_SIZE_HEIGHT			(2464)
#define MT9E013_FULL_SIZE_DUMMY_PIXELS 	(0)
#define MT9E013_FULL_SIZE_DUMMY_LINES	(0)

#define MT9E013_FHD_SIZE_WIDTH 			(2640)
#define MT9E013_FHD_SIZE_HEIGHT 			(1486)
#define MT9E013_FHD_SIZE_DUMMY_PIXELS 	(0)
#define MT9E013_FHD_SIZE_DUMMY_LINES 	(0)
*/

// mo2jonghoo.lee 2012.12.27 
/* Full	Size */
#define	MT9P017_FULL_SIZE_WIDTH      	2608
#define	MT9P017_FULL_SIZE_HEIGHT		1960
#define	MT9P017_FULL_SIZE_DUMMY_PIXELS	0
#define	MT9P017_FULL_SIZE_DUMMY_LINES	0
/* Quarter Size	*/
#define	MT9P017_QTR_SIZE_WIDTH			1300
#define	MT9P017_QTR_SIZE_HEIGHT			980
#define	MT9P017_QTR_SIZE_DUMMY_PIXELS	0
#define	MT9P017_QTR_SIZE_DUMMY_LINES	0
/* FHD Size	*/
#define MT9P017_FHD_SIZE_WIDTH	1936
#define MT9P017_FHD_SIZE_HEIGHT	1096
#define	MT9P017_FHD_SIZE_DUMMY_PIXELS	0
#define	MT9P017_FHD_SIZE_DUMMY_LINES	0

/* Full	Size */
#define	MT9P017_HRZ_FULL_BLK_PIXELS		2672
#define	MT9P017_VER_FULL_BLK_LINES		80
/* Quarter Size	*/
#define	MT9P017_HRZ_QTR_BLK_PIXELS		2104
#define	MT9P017_VER_QTR_BLK_LINES		76
/* FHD Size	*/
#define	MT9P017_HRZ_FHD_BLK_PIXELS		1133
#define	MT9P017_VER_FHD_BLK_LINES		77

#define REG_GROUPED_PARAMETER_HOLD	 0x0104 //                                                                    
#define GROUPED_PARAMETER_HOLD_OFF	 0x00 //                                                                    
#define GROUPED_PARAMETER_HOLD		 0x01 //                                                                    
// mo2jonghoo.lee 2012.12.27 

// mo2jonghoo.lee 2012.12.27
static struct msm_camera_i2c_reg_conf mipi_settings[] = { 
	{0x301A, 0x0018},	 //reset_register
	{0x3064, 0xB800},	 //smia_test_2lane_mipi
	{0x31AE, 0x0202},	 //dual_lane_MIPI_interface
	{0x30F0, 0x8010},	/*VCM CONTROL*/ //                                                                       
	{0x317A, 0x2000},	/*VCM_ANALOG_POWER*/ //                                                                       
};

struct msm_camera_i2c_reg_conf pll_settings[]=
{
	{0x0300, 0x0005},	 //vt_pix_clk_div
	{0x0302, 0x0001},	 //vt_sys_clk_div
	{0x0304, 0x0002},	 //pre_pll_clk_div
	{0x0306, 0x002D},	 //pll_multipler
	{0x0308, 0x000A},	 //op_pix_clk_div
	{0x030A, 0x0001}	 //op_sys_clk_div
};

struct msm_camera_i2c_reg_conf prev_settings[]=
{
	{0x3004, 0x0000},	 //x_addr_start
	{0x3008, 0x0A25},	 //x_addr_end
	{0x3002, 0x0000},	 //y_start_addr
	{0x3006, 0x07A5},	 //y_addr_end
	{0x3040, 0x04C3},	 //read_mode
	{0x034C, 0x0514},	 //x_output_size
	{0x034E, 0x03D4},	 //y_output_size
	{0x300C, 0x0D4C},	 //line_length_pck
	{0x300A, 0x0420},	 //frame_length_lines
	{0x3012, 0x041F},	 //coarse_integration_time
	{0x3014, 0x0A04},	 //fine_integration_time
	{0x3010, 0x0184}	 //fine_correction
};

/* Snapshot register settings */
struct msm_camera_i2c_reg_conf snap_settings[]=
{
	{0x3004, 0x0000},	 //x_addr_start
	{0x3008, 0x0A2F},	 //x_addr_end  //2607
	{0x3002, 0x0000},	 //y_start_addr
	{0x3006, 0x07A7},	 //y_addr_end //1959
	{0x3040, 0x0041},	 //                                                                                          
	{0x034C, 0x0A30},	 //x_output_size //2608
	{0x034E, 0x07A8},	 //y_output_size //1960
	{0x300C, 0x14A0},	 //line_length_pck //5280
	{0x300A, 0x07F8},	 //frame_length_lines //2040
	{0x3012, 0x07F7},	 //coarse_integration_time //2039
	{0x3014, 0x12BE},	 //fine_integration_time //4798
	{0x3010, 0x00A0},	 //fine_correction //160
};

struct msm_camera_i2c_reg_conf FHD_settings[]=
{
	{0x3004, 0x0150},	   //x_addr_start								  
	{0x3008, 0x08DF},	  //x_addr_end										 
	{0x3002, 0x01B0},	  //y_start_addr									 
	{0x3006, 0x05F7},	   //y_addr_end 									  
	{0x3040, 0x0041},	  //read_mode - vertical flip, horizontal mirror	 
	{0x034C, 0x0790},	  //x_output_size	 //1920+16									  
	{0x034E, 0x0448},	   //y_output_size	  //1080+16 								 
	{0x300C, 0x0BFD},	  //line_length_pck 								 
	{0x300A, 0x0495},	  //frame_length_lines								 
	{0x3012, 0x0494},	   //coarse_integration_time						  
	{0x3014, 0x02CE},	  //0x12BE				//fine_integration_time 						   
	{0x3010, 0x00A0}	  //fine_correction
};

static struct msm_camera_i2c_reg_conf recommend_settings[] = {
	//mipi timing setting
	{0x31B0, 0x00C4}, 
	{0x31B2, 0x0064},
	{0x31B4, 0x0E77},
	{0x31B6, 0x0D24},
	{0x31B8, 0x020E},
	{0x31BA, 0x0710},
	{0x31BC, 0x2A0D},	
	{0x31BE, 0xC007}, //  continuous

	//Recommended Settings
	{0x316A, 0x8400}, // RESERVED       
	{0x316C, 0x8400}, // RESERVED       
	{0x316E, 0x8400}, // RESERVED       
	{0x3EFA, 0x1A1F}, // RESERVED       
	{0x3ED2, 0xD965}, // RESERVED       
	{0x3ED8, 0x7F1B}, // RESERVED       
	{0x3EDA, 0xAF11}, // RESERVED       
	{0x3EE2, 0x0060}, // RESERVED       
	{0x3EF2, 0xD965}, // RESERVED       
	{0x3EF8, 0x797F}, // RESERVED       
	{0x3EFC, 0xA8EF}, // RESERVED       
	{0x3EFE, 0x1F0F}, // RESERVED       
	{0x31E0, 0x1F01}, // RESERVED       
	{0x305E, 0x1824},      //  minimum analog gain, 2011.12.21
	{0x3E00, 0x042F},                   
	{0x3E02, 0xFFFF},                   
	{0x3E04, 0xFFFF},                   
	{0x3E06, 0xFFFF},                   
	{0x3E08, 0x8071},                   
	{0x3E0A, 0x7281},                   
	{0x3E0C, 0x4011},                   
	{0x3E0E, 0x8010},                   
	{0x3E10, 0x60A5},                   
	{0x3E12, 0x4080},                   
	{0x3E14, 0x4180},                   
	{0x3E16, 0x0018},                   
	{0x3E18, 0x46B7},                   
	{0x3E1A, 0x4994},                   
	{0x3E1C, 0x4997},                   
	{0x3E1E, 0x4682},                   
	{0x3E20, 0x0018},                   
	{0x3E22, 0x4241},                   
	{0x3E24, 0x8000},                   
	{0x3E26, 0x1880},                   
	{0x3E28, 0x4785},                   
	{0x3E2A, 0x4992},                   
	{0x3E2C, 0x4997},                   
	{0x3E2E, 0x4780},                   
	{0x3E30, 0x4D80},                   
	{0x3E32, 0x100C},                   
	{0x3E34, 0x8000},                   
	{0x3E36, 0x184A},                   
	{0x3E38, 0x8042},                   
	{0x3E3A, 0x001A},                   
	{0x3E3C, 0x9610},                   
	{0x3E3E, 0x0C80},                   
	{0x3E40, 0x4DC6},                   
	{0x3E42, 0x4A80},                   
	{0x3E44, 0x0018},                   
	{0x3E46, 0x8042},                   
	{0x3E48, 0x8041},                   
	{0x3E4A, 0x0018},                   
	{0x3E4C, 0x804B},                   
	{0x3E4E, 0xB74B},                   
	{0x3E50, 0x8010},                   
	{0x3E52, 0x6056},                   
	{0x3E54, 0x001C},                   
	{0x3E56, 0x8211},                   
	{0x3E58, 0x8056},                   
	{0x3E5A, 0x827C},                   
	{0x3E5C, 0x0970},                   
	{0x3E5E, 0x8082},                   
	{0x3E60, 0x7281},                   
	{0x3E62, 0x4C40},                   
	{0x3E64, 0x8E4D},                   
	{0x3E66, 0x8110},                   
	{0x3E68, 0x0CAF},                   
	{0x3E6A, 0x4D80},                   
	{0x3E6C, 0x100C},                   
	{0x3E6E, 0x8440},                   
	{0x3E70, 0x4C81},                   
	{0x3E72, 0x7C5F},                   
	{0x3E74, 0x7000},                   
	{0x3E76, 0x0000},                   
	{0x3E78, 0x0000},                   
	{0x3E7A, 0x0000},                   
	{0x3E7C, 0x0000},                   
	{0x3E7E, 0x0000},                   
	{0x3E80, 0x0000},                   
	{0x3E82, 0x0000},                   
	{0x3E84, 0x0000},                   
	{0x3E86, 0x0000},                   
	{0x3E88, 0x0000},                   
	{0x3E8A, 0x0000},                   
	{0x3E8C, 0x0000},                   
	{0x3E8E, 0x0000},                   
	{0x3E90, 0x0000},                   
	{0x3E92, 0x0000},                   
	{0x3E94, 0x0000},                   
	{0x3E96, 0x0000},                   
	{0x3E98, 0x0000},                   
	{0x3E9A, 0x0000},                   
	{0x3E9C, 0x0000},                   
	{0x3E9E, 0x0000},                   
	{0x3EA0, 0x0000},                   
	{0x3EA2, 0x0000},                   
	{0x3EA4, 0x0000},                   
	{0x3EA6, 0x0000},                   
	{0x3EA8, 0x0000},                   
	{0x3EAA, 0x0000},                   
	{0x3EAC, 0x0000},                   
	{0x3EAE, 0x0000},                   
	{0x3EB0, 0x0000},                   
	{0x3EB2, 0x0000},                   
	{0x3EB4, 0x0000},                   
	{0x3EB6, 0x0000},                   
	{0x3EB8, 0x0000},                   
	{0x3EBA, 0x0000},                   
	{0x3EBC, 0x0000},                   
	{0x3EBE, 0x0000},                   
	{0x3EC0, 0x0000},                   
	{0x3EC2, 0x0000},                   
	{0x3EC4, 0x0000},                   
	{0x3EC6, 0x0000},                   
	{0x3EC8, 0x0000},                   
	{0x3ECA, 0x0000},                   
	{0x3170, 0x2150},                   
	{0x317A, 0x0150},                   
	{0x3ECC, 0x2200},                   
	{0x3174, 0x0000},                   
	{0x3176, 0X0000},                   
	{0x30D4, 0x9200},                   
	{0x30B2, 0xC000},                   
	{0x30BC, 0x0400},                   
	{0x306E, 0xB480},                   
	{0x31B0, 0x00C4},                   
	{0x31B2, 0x0064},                   
	{0x31B4, 0x0E77},                   
	{0x31B6, 0x0D24},                   
	{0x31B8, 0x020E},                   
	{0x31BA, 0x0710},                   
	{0x31BC, 0x2A0D},                   
    {0x31BE, 0xC007},
};


static struct msm_camera_i2c_reg_conf lensrolloff_tbl[] = {
  {0x3780, 0x0000}              //  Poly_sc_enable                
};

static struct msm_camera_i2c_reg_conf nr_settings[] = {
	{0x3100, 0x0002},		// ADACD_CONTROL
	{0x3102, 0x0025},   // ADACD_NOISE_MODEL1
	{0x3104, 0x0B6D},   // ADACD_NOISE_MODEL2
	{0x3106, 0x0201},   // ADACD_NOISE_FLOOR1
	{0x3108, 0x0804},   // ADACD_NOISE_FLOOR2
	{0x310A, 0x002A},   // ADACD_PEDESTAL
	{0x310C, 0x0080},   // ADACD_GAIN_THRESHOLD_0
	{0x310E, 0x0100},   // ADACD_GAIN_THRESHOLD_1
	{0x3110, 0x0200},   // ADACD_GAIN_THRESHOLD_2
};

static struct msm_camera_i2c_reg_conf dpc_settings[] = {
	{0x31E0, 0x1F01}, 	//PIX_DEF_ID
	{0x3F02, 0x0001}, 	//PIX_DEF_2D_DDC_THRESH_HI3
	{0x3F04, 0x0032}, 	//PIX_DEF_2D_DDC_THRESH_LO3
	{0x3F06, 0x015E}, 	//PIX_DEF_2D_DDC_THRESH_HI4
	{0x3F08, 0x0190}, 	//PIX_DEF_2D_DDC_THRESH_LO4
};

// mo2jonghoo.lee 2012.12.27

//                                                            
// mo2jonghoo.lee 2012.12.27 임시로 사용. 차후 분석 필요. static struct msm_camera_i2c_reg_conf lsc_data[LSC_FIRST_PHASE1_DATA_SIZE*LSC_FIRST_PHASE2_DATA_SIZE+LSC_SECOND_PHASE_DATA_SIZE+LSC_THIRD_PHASE_DATA_SIZE] ;
//                                                            

static struct sensor_extra_t {
	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
//	uint16_t my_reg_gain;
//	uint32_t my_reg_line_count;
//	uint16_t total_lines_per_frame;
} mt9p017_extra;
static struct sensor_extra_t * mt9p017_ctrl = &mt9p017_extra;

static uint16_t mt9p017_linear_total_step = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;
static uint16_t mt9p017_step_position_table[MT9P017_TOTAL_STEPS_NEAR_TO_FAR+1];
//                                                 
static uint16_t af_infinity = 32;  // sungmin.woo : at least 64~64+96
static uint16_t mt9p017_nl_region_boundary1 = 0;
static uint16_t mt9p017_nl_region_code_per_step1 = 0;
static uint16_t mt9p017_l_region_code_per_step = 5;
//                                                                                                             
static uint16_t mt9p017_sw_damping_time_wait;
static uint16_t mt9p017_damping_threshold = 10; //                                                                       
static uint16_t mt9p017_sw_damping_time_wait = 1; //                                                                       
//                                                 
static struct msm_camera_i2c_reg_conf mt9p017_start_stream_settings[] = {
	{0x0100, 0x01},   
};
static struct msm_camera_i2c_reg_conf mt9p017_stop_stream_settings[] = {
	{0x0100, 0x00},  
};

static struct msm_camera_i2c_reg_conf mt9p017_groupon_settings[] = {
	{0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf mt9p017_groupoff_settings[] = {
	{0x0104, 0x00},
};

static struct v4l2_subdev_info mt9p017_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array mt9p017_init_conf[] = {
	{&mipi_settings[0],	ARRAY_SIZE(mipi_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&pll_settings[0],	ARRAY_SIZE(pll_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&recommend_settings[0],	ARRAY_SIZE(recommend_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&lensrolloff_tbl[0],	ARRAY_SIZE(lensrolloff_tbl), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&nr_settings[0],	ARRAY_SIZE(nr_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&dpc_settings[0],	ARRAY_SIZE(dpc_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
};

static struct msm_camera_i2c_conf_array mt9p017_confs[] = {
	{&snap_settings[0], ARRAY_SIZE(snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&prev_settings[0], ARRAY_SIZE(prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&FHD_settings[0], ARRAY_SIZE(FHD_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
};

static struct msm_sensor_output_info_t mt9p017_dimensions[] = {
	//(Ext=24MHz, vt_pix_clk=174MHz, op_pix_clk=69.6MHz)*/
	{ //MSM_SENSOR_RES_FULL
		.x_output = MT9P017_FULL_SIZE_WIDTH,
		.y_output = MT9P017_FULL_SIZE_HEIGHT,
		.line_length_pclk = 0x14A0, //0x1370, 
		.frame_length_lines = 0x07F8, //0x0A2F,
		.vt_pixel_clk = 108000000, //174000000,
		.op_pixel_clk = 54000000,
		.binning_factor = 1,	
	},
	{ //MSM_SENSOR_RES_QTR
		.x_output = MT9P017_QTR_SIZE_WIDTH,
		.y_output = MT9P017_QTR_SIZE_HEIGHT,
		.line_length_pclk = 0x0D4C, //0x1018,
		.frame_length_lines = 0x0420, //0x055B,
		.vt_pixel_clk = 108000000, //174000000,
		.op_pixel_clk = 54000000,
		.binning_factor = 1,	
	},
	{ //MSM_SENSOR_RES_2
		.x_output = MT9P017_FHD_SIZE_WIDTH,
		.y_output = MT9P017_FHD_SIZE_HEIGHT,
		.line_length_pclk = 0x0BFD, //0x0FD8,
		.frame_length_lines = 0x0495, //0x065D,
		.vt_pixel_clk = 108000000, //174000000,
		.op_pixel_clk = 54000000,
		.binning_factor = 1,	
	},
};

static struct msm_camera_csi_params mt9p017_csi_params = {
	.data_format = CSI_10BIT,
	.lane_cnt    = 2,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *mt9p017_csi_params_array[] = {
	&mt9p017_csi_params,
	&mt9p017_csi_params, //NULL, //when res is FULL_SIZE, Nothing 
	&mt9p017_csi_params,
};

static struct msm_sensor_output_reg_addr_t mt9p017_reg_addr = { // mo2jonghoo.lee 2012.12.27
	.x_output = 0x034C,
	.y_output = 0x034E,
	.line_length_pclk = 0x300C,
	.frame_length_lines = 0x300A,
};

static struct msm_sensor_id_info_t mt9p017_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = MT9P017_MODEL_ID,
};

static struct msm_sensor_exp_gain_info_t mt9p017_exp_gain_info = {
	.coarse_int_time_addr = REG_COARSE_INT_TIME, //0x3012
	.global_gain_addr = REG_GLOBAL_GAIN, //0x305E
	.vert_offset = 0,
};

#if 0 // mo2jonghoo.lee 2012.12.27 임시로 사용. 차후 분석 필요.
//                                                 
static int32_t mt9p017_i2c_read_w_eeprom(struct msm_camera_i2c_client* sensor_i2c_client, 
	uint16_t reg_addr, uint16_t *rdata)
{
	int32_t rc = 0;
	unsigned char buf;
	
	if (!rdata)
		return -EIO;

	//Read 2 bytes in sequence 
	//Big Endian address:
	buf = reg_addr;
	buf = (reg_addr & 0xFF00) >> 8;
	rc = msm_camera_i2c_rxdata_manual(sensor_i2c_client->client->adapter, MT9P017_EEPROM_SLAVE_ADDR, &buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]1 :mt9p017_i2c_read_eeprom 0x%x failed!\n", buf);
		return rc;
	}
	*rdata = buf;

	buf = (reg_addr & 0x00FF);	
	rc = msm_camera_i2c_rxdata_manual(sensor_i2c_client->client->adapter, MT9P017_EEPROM_SLAVE_ADDR, &buf, 1);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]2 :mt9p017_i2c_read_eeprom 0x%x failed!\n", buf);
		return rc;
	}
	*rdata = (*rdata<<8)|buf;

	return rc;
}

static int32_t mt9p017_read_5100k_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t index_lsc = 0;
	int32_t i,j;
	int32_t n=0x01;	
	uint16_t reg_addr_read = 0x0001;
	uint16_t eepromdata = 0;
	uint16_t reg_addr_write;

	//First Phase
	reg_addr_write = 0x3600;
	printk(KERN_EMERG "[CAMERA][EEPROM] %s : Start : 0x%04x 0x%04x\n", __func__, reg_addr_read, reg_addr_write);
	for( j = 0 ; j < LSC_FIRST_PHASE1_DATA_SIZE ; j++) {
		for( i = 0 ; i < LSC_FIRST_PHASE2_DATA_SIZE ; i++) {
			rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr_read, &eepromdata);
			if (rc < 0) {
				printk(KERN_EMERG "[CAMERA]%s : Fail to read!! First Phase (j=%d,i=%d) Source line number : %d\n", __func__, j, i, __LINE__);
				return rc;
			}
#ifdef EEPROM_CHECK
			CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x waddr:0x%04x eepromdata: 0x%04x\n", reg_addr_read, reg_addr_write, eepromdata);
#endif			
			lsc_data[index_lsc].reg_addr=reg_addr_write;
			lsc_data[index_lsc].reg_data=eepromdata;
			
			reg_addr_read++;
			reg_addr_write += 2;
			n += 2;
			index_lsc++;
			reg_addr_read = (reg_addr_read<<8) |n;
		}
		reg_addr_write = reg_addr_write + 0x0018;
	}

	
	// Second Phase
	reg_addr_write = 0x3782;
	for( i=0 ; i < LSC_SECOND_PHASE_DATA_SIZE ; i++) {
		rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr_read, &eepromdata);
		if (rc < 0) {
			printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Second Phase i=%d Source line number : %d\n", __func__,  i, __LINE__);
			return rc;
		}
#ifdef EEPROM_CHECK
		CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x waddr:0x%04x eepromdata: 0x%04x\n", reg_addr_read, reg_addr_write, eepromdata);
#endif
		lsc_data[index_lsc].reg_addr=reg_addr_write;
		lsc_data[index_lsc].reg_data=eepromdata;
		
		reg_addr_read++;
		reg_addr_write += 2;
		n += 2; 
		index_lsc++;
		reg_addr_write = (reg_addr_write<<8) |n;
	}

	// Third Phase
	reg_addr_write = 0x37C0;
	for( i=0 ; i < LSC_THIRD_PHASE_DATA_SIZE; i++) {
		rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr_read, &eepromdata);
		if (rc < 0) {
			printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Third Phase i=%d Source line number : %d\n", __func__, i, __LINE__);
			return rc;
		}
#ifdef EEPROM_CHECK
		CDBG("[CAMERA][TEST]!!!! GOGO raddr:0x%04x reg_addr:0x%04x eepromdata: 0x%04x\n", reg_addr_read, reg_addr_write, eepromdata);
#endif
		lsc_data[index_lsc].reg_addr=reg_addr_write;
		lsc_data[index_lsc].reg_data=eepromdata;

		reg_addr_read++;
		reg_addr_write += 2;
		n += 2;
		index_lsc++;
		reg_addr_read = (reg_addr_read<<8) |n;
	}	
	
#ifdef EEPROM_CHECK
	for(i = 0; i <ARRAY_SIZE(lsc_data); i++) {
		CDBG("[CAMERA][EEPROM_CHECK]index: %03d reg_addr:0x%04x eepromdata: 0x%04x\n", i, lsc_data[i].reg_addr, lsc_data[i].reg_data);
	}
#endif

	return rc;
}

static int32_t mt9p017_read_awb_data(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_calib_data *sensor_cablib_data, bool bresult)
{
	int32_t rc = 0;

	uint16_t reg_addr=0;
	uint16_t eepromdata = 0;

	reg_addr=0xD4D5;	// R/G
	rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		sensor_cablib_data->r_over_g = 0x0300;	//0x005E;
	} else {
		sensor_cablib_data->r_over_g = eepromdata;
	}
	CDBG("[CAMERA]%s : line:%d : R/G 0x%04x\n", __func__, __LINE__, sensor_cablib_data->r_over_g);

	reg_addr=0xD6D7;	// B/G
	rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		sensor_cablib_data->b_over_g = 0x0289;	//0x0051;
	} else {
		sensor_cablib_data->b_over_g = eepromdata;
	}
	CDBG("[CAMERA]%s : line:%d : B/G 0x%04x\n", __func__, __LINE__, sensor_cablib_data->b_over_g);

	reg_addr=0xD4D5;	// Gr/Gb
	rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s : Fail to read!! Source line number : %d\n", __func__, __LINE__);
		return rc;
	}
	// If there is no data in EEPROM, Apply static value.
	if(!bresult) {
		sensor_cablib_data->gr_over_gb = 0x0300;	//0x005E;
	} else {
		sensor_cablib_data->gr_over_gb = eepromdata;
	}
	printk(KERN_EMERG "[CAMERA]%s : line:%d : GR/GB 0x%04x\n", __func__, __LINE__, sensor_cablib_data->gr_over_gb);

	return rc;
}

static int32_t mt9p017_write_5100k_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc=0;

	if (s_ctrl->func_tbl->sensor_group_hold_on)
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);

	rc = msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client, &lsc_data[0], ARRAY_SIZE(lsc_data), MSM_CAMERA_I2C_WORD_DATA);

	if (s_ctrl->func_tbl->sensor_group_hold_off)
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	
	return rc;
}

static int32_t mt9p017_lens_shading_enable(struct msm_sensor_ctrl_t *s_ctrl, uint8_t is_enable) // mo2jonghoo.lee 2012.12.27 
{
	int32_t rc = 0;

	CDBG("[CAMERA]%s: entered. enable = %d\n", __func__, is_enable);

	if (s_ctrl->func_tbl->sensor_group_hold_on)
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 
			REG_LENS_SHADING, ((uint16_t) is_enable) << 15, MSM_CAMERA_I2C_WORD_DATA);	
	if (rc < 0)
		return rc;

	if (s_ctrl->func_tbl->sensor_group_hold_off)
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);

	CDBG("[CAMERA]%s: exiting. rc = %d\n", __func__, rc);
	
	return rc;
}

static int mt9p017_read_eeprom_data(struct msm_sensor_ctrl_t *s_ctrl,
		struct sensor_calib_data *sensor_cablib_data)
{
	int32_t rc = 0;
	uint16_t eepromdata = 0;
	uint16_t reg_addr = 0;
	bool bresult = false;

	CDBG("[CAMERA][MT9P017_EEPROM] Start reading EEPROM\n");	

	/*#1. Model ID for checking EEPROM READ*/
	reg_addr = 0xFEFF;
	rc = mt9p017_i2c_read_w_eeprom(s_ctrl->sensor_i2c_client, reg_addr, &eepromdata);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s: Error Reading EEPROM @ 0x%x\n", __func__, reg_addr);
		return rc;
	}
	if(eepromdata == 0x0AFF) 
		bresult = true;
	else 
		bresult = false;
	
	printk(KERN_EMERG "[CAMERA][QCTK_EEPROM] Product version = 0x%x bresult:%d\n", eepromdata,bresult); 

	/*#2. 5100K LSC : Read LSC table Data from EEPROM */
	rc = mt9p017_read_5100k_data(s_ctrl); // read LSC data
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s:%d: Error Reading\n", __func__,__LINE__);
		return rc;
	}
	
	/*#3. 5100K AWB Data from EEPROM */ 
	mt9p017_read_awb_data(s_ctrl, sensor_cablib_data, bresult);
	CDBG("[CAMERA]%s:%d: AWB: r/g:0x%04x b/g:0x%04x gr/gb:0x%04x\n", __func__,__LINE__,
			sensor_cablib_data->r_over_g, sensor_cablib_data->b_over_g, sensor_cablib_data->gr_over_gb);
	
	/*#4. Write LSC data to sensor - it will be enabled in setting routine*/
	//Write LSC table to the sensor
	rc = mt9p017_write_5100k_data(s_ctrl);
	if (rc < 0) {
		printk(KERN_EMERG "[CAMERA]%s:%d: Error Writing\n", __func__,__LINE__);
		return rc;
	}
	
	/*Enable Aptina Lens shading */
	mt9p017_lens_shading_enable(s_ctrl, LSC_ON); 

	return rc;
}
#endif // mo2jonghoo.lee 2012.12.27 임시로 사용. 차후 분석 필요.

static int32_t mt9p017_lens_shading_enable(struct msm_sensor_ctrl_t *s_ctrl, uint8_t is_enable) //                                                                    
{
	int32_t rc = 0;

	CDBG("%s: entered. enable = %d\n", __func__, is_enable);

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD, MSM_CAMERA_I2C_BYTE_DATA);	
	if (rc < 0)
		return rc;

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			REG_LENS_SHADING, 0x0000, MSM_CAMERA_I2C_WORD_DATA);	
	if (rc < 0)
		return rc;

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD_OFF, MSM_CAMERA_I2C_BYTE_DATA);	
	if (rc < 0)
		return rc;

	CDBG("%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}

//==========================================================================

static int32_t mt9p017_get_pict_fps(struct msm_sensor_ctrl_t *s_ctrl, uint16_t fps, uint16_t *pfps) // mo2jonghoo.lee 2012.12.27 
{
	int32_t rc = 0;

	/* input fps is preview fps in Q8 format */
	uint16_t preview_frame_length_lines, snapshot_frame_length_lines;
	uint16_t preview_line_length_pck, snapshot_line_length_pck;
	uint32_t divider, d1, d2;
	/* Total frame_length_lines and line_length_pck for preview */
	preview_frame_length_lines = MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES;
	preview_line_length_pck = MT9P017_QTR_SIZE_WIDTH + MT9P017_HRZ_QTR_BLK_PIXELS;
	/* Total frame_length_lines and line_length_pck for snapshot */
	snapshot_frame_length_lines = MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES;
	snapshot_line_length_pck = MT9P017_FULL_SIZE_WIDTH + MT9P017_HRZ_FULL_BLK_PIXELS;

	d1 = preview_frame_length_lines * 0x00000400/snapshot_frame_length_lines;
	d2 = preview_line_length_pck * 0x00000400/snapshot_line_length_pck;
	divider = d1 * d2 / 0x400;
	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */

	return rc;
}

static int32_t mt9p017_get_prev_lines_pf(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_prevl_pf)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_get_prev_lines_pf\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_prevl_pf = MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_prevl_pf = MT9P017_FHD_SIZE_HEIGHT + MT9P017_VER_FHD_BLK_LINES;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_prevl_pf = MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES;

	return rc;
}

static int32_t mt9p017_get_prev_pixels_pl(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_prevp_pl)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_get_prev_pixels_pl\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_prevp_pl = MT9P017_QTR_SIZE_WIDTH + MT9P017_HRZ_QTR_BLK_PIXELS;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_prevp_pl = MT9P017_FHD_SIZE_WIDTH + MT9P017_HRZ_FHD_BLK_PIXELS;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_prevp_pl = MT9P017_FULL_SIZE_WIDTH + MT9P017_HRZ_FULL_BLK_PIXELS;

	return rc;	
}

static int32_t mt9p017_get_pict_lines_pf(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_pictl_pf)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_get_pict_lines_pf\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pictl_pf = MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pictl_pf = MT9P017_FHD_SIZE_HEIGHT + MT9P017_VER_FHD_BLK_LINES;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pictl_pf = MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES;

	return rc;		
}

static int32_t mt9p017_get_pict_pixels_pl(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t *p_pictp_pl)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_get_pict_pixels_pl\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pictp_pl = MT9P017_QTR_SIZE_WIDTH + MT9P017_HRZ_QTR_BLK_PIXELS;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pictp_pl = MT9P017_FHD_SIZE_WIDTH + MT9P017_HRZ_FHD_BLK_PIXELS;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pictp_pl = MT9P017_FULL_SIZE_WIDTH + MT9P017_HRZ_FULL_BLK_PIXELS;

	return rc;			
}

static int32_t mt9p017_get_pict_max_exp_lc(struct msm_sensor_ctrl_t *s_ctrl,
	uint32_t *p_pict_max_exp_lc)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_get_pict_max_exp_lc\n");

	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		*p_pict_max_exp_lc = (MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES)*24;
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		*p_pict_max_exp_lc = (MT9P017_FHD_SIZE_HEIGHT + MT9P017_VER_FHD_BLK_LINES)*24;
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		*p_pict_max_exp_lc = (MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES)*24;

	return rc;			
}

static int32_t mt9p017_get_af_max_steps(struct msm_sensor_ctrl_t *s_ctrl,
	uint8_t *pmax_steps)
{
	int32_t rc = 0;

	*pmax_steps = mt9p017_linear_total_step;
	
	return rc;			
}

static int32_t mt9p017_set_fps(struct msm_sensor_ctrl_t *s_ctrl,
						struct fps_cfg *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_set_fps mode:%d\n", s_ctrl->curr_res);
	
	if (s_ctrl->curr_res == MSM_SENSOR_RES_QTR /* QTR_SIZE */)
		total_lines_per_frame =	(MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES);
	else if (s_ctrl->curr_res == MSM_SENSOR_RES_2 /* FHD_SIZE */)
		total_lines_per_frame =	(MT9P017_FHD_SIZE_HEIGHT + MT9P017_VER_FHD_BLK_LINES);
	else //MSM_SENSOR_RES_FULL /* FULL_SIZE */
		total_lines_per_frame =	(MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES);


	s_ctrl->fps_divider = fps->fps_div;
//No on JB	s_ctrl->pict_fps_divider = fps->pict_fps_div;
	CDBG("[CAMERA][1] mt9p017_set_fps : total_lines_per_frame = %d, fps->fps_div = %d, fps->pict_fps_div = %d\n", 
												total_lines_per_frame, fps->fps_div, fps->pict_fps_div);

//No on JB	if (s_ctrl->curr_res == FULL_SIZE) {
//No on JB		total_lines_per_frame = (uint16_t)
//No on JB		(total_lines_per_frame * mt9p017_ctrl->pict_fps_divider/0x400);
//No on JB	} else {
		total_lines_per_frame = (uint16_t)(total_lines_per_frame * s_ctrl->fps_divider/0x400);
//No on JB	}
	CDBG("[CAMERA][2] mt9p017_set_fps : total_lines_per_frame = %d, fps->fps_div = %d, fps->pict_fps_div = %d\n", 
												total_lines_per_frame, fps->fps_div,  fps->pict_fps_div );

	if (s_ctrl->func_tbl->sensor_group_hold_on)
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, REG_FRAME_LENGTH_LINES, total_lines_per_frame, 
			MSM_CAMERA_I2C_WORD_DATA);

	if (s_ctrl->func_tbl->sensor_group_hold_off)
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	
	return rc;
}

static int32_t mt9p017_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0xE7F;
	//uint16_t frame_length_lines;

	int32_t rc = 0;

	CDBG("mt9p0173_write_exp_gain entering.... \n");
//No on JB	if (mt9p017_ctrl->curr_res == SENSOR_PREVIEW_MODE) {
//No on JB		mt9p017_ctrl->my_reg_gain = gain;
//No on JB		mt9p017_ctrl->my_reg_line_count = (uint16_t) line;
//No on JB	}

	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}

//No on JB	if (mt9p017_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line = (uint32_t) (line * s_ctrl->fps_divider /  0x00000400 /*Q10*/);
//No on JB	} else {
//No on JB		line = (uint32_t) (line * mt9p017_ctrl->pict_fps_divider /  0x00000400);
//No on JB	}

	CDBG("[CAMERA][1] mt9p017_write_exp_gain : frame_length_lines = NONE, mt9p017_ctrl->fps_divider = %d, mt9p017_ctrl->pict_fps_divider = %d\n", 
									/*frame_length_lines, */s_ctrl->fps_divider, -1 /* mt9p017_ctrl->pict_fps_divider */);	
	CDBG("[CAMERA][2] mt9p017_write_exp_gain : line=%d gain=%d\n",line, gain); 

	gain |= 0x1000;

	if (s_ctrl->func_tbl->sensor_group_hold_on)
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,	MSM_CAMERA_I2C_WORD_DATA);
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, line,	MSM_CAMERA_I2C_WORD_DATA);
	
	if (s_ctrl->func_tbl->sensor_group_hold_off)
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);

	return rc;
}

static int32_t mt9p017_write_exp_snapshot_gain(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_set_pict_exp_gain : gain = %d,line = %d \n", gain, line);

	if (s_ctrl->func_tbl->sensor_write_exp_gain)
		rc = s_ctrl->func_tbl->sensor_write_exp_gain(s_ctrl, gain, line);
	
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,	REG_RESET_REGISTER, REG_FAST_TRANS_MODE_ON, MSM_CAMERA_I2C_WORD_DATA);	
	
	return rc;
}

static int32_t mt9p017_move_focus(struct msm_sensor_ctrl_t *s_ctrl,
	int direction, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if(direction == MOVE_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction \n");
		return -EINVAL;
	}
	
	//printk("mt9p017_move_focus calculating dest_step_position \n");
	
	dest_step_position = mt9p017_ctrl->curr_step_pos + (step_direction * num_steps);
	if (dest_step_position < 0){
		dest_step_position = 0;
	}
	else if (dest_step_position > MT9P017_TOTAL_STEPS_NEAR_TO_FAR){
		dest_step_position = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;
	}
	if(dest_step_position == mt9p017_ctrl->curr_step_pos){
		//printk("mt9p017_move_focus ==  mt9p017_ctrl->curr_step_pos No Move exit \n");
		return rc;
	}
	//printk("Cur Step: %hd Step Direction: %hd Dest Step Pos: %hd Num Step: %hd\n", mt9p017_ctrl->curr_step_pos, step_direction, dest_step_position, num_steps);

	dest_lens_position = mt9p017_step_position_table[dest_step_position];
	target_dist = step_direction * (dest_lens_position - mt9p017_ctrl->curr_lens_pos);
	//printk("Target Dist: %hd\n", target_dist);

	if(step_direction < 0 && (target_dist >=
		mt9p017_step_position_table[mt9p017_damping_threshold])){
		small_step = (uint16_t)((target_dist/10));
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 1;
	}
	else{
		small_step = (uint16_t)(target_dist/4);
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 4;
	}
	//printk("mt9p017_move_focus small_step %d ...\n", small_step);

	for (next_lens_position = mt9p017_ctrl->curr_lens_pos + (step_direction * small_step);
	(step_direction * next_lens_position) <= (step_direction * dest_lens_position);
	next_lens_position += (step_direction * small_step)){
		//printk("mt9p017_move_focus next_lens_position %d ...\n", next_lens_position);
		//printk("mt9p017_move_focus writing i2c at line %d ...\n", __LINE__);
		// mo2jonghoo.lee 2013.02.19 rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, next_lens_position);
		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, REG_VCM_NEW_CODE, next_lens_position, MSM_CAMERA_I2C_WORD_DATA); // mo2jonghoo.lee 2013.02.19 
		if (rc < 0){
			//printk("mt9p017_move_focus failed writing i2c at line %d ...\n", __LINE__);
			return rc;
			}
		//printk("mt9p017_move_focus writing Success i2c at line %d ...\n", __LINE__);
		mt9p017_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9p017_sw_damping_time_wait*10);
	}
	if(mt9p017_ctrl->curr_lens_pos != dest_lens_position){
		//printk("mt9p017_move_focus writing i2c at line %d ...\n", __LINE__);
		//printk("mt9p017_move_focus curr_lens_pos = %d  dest_lens_position = %d ...\n", mt9p017_ctrl->curr_lens_pos, dest_lens_position);

		// mo2jonghoo.lee 2013.02.19 rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		// mo2jonghoo.lee 2013.02.19	 dest_lens_position);
		rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, REG_VCM_NEW_CODE, dest_lens_position, MSM_CAMERA_I2C_WORD_DATA); // mo2jonghoo.lee 2013.02.19 
		if (rc < 0){
			//printk("mt9p017_move_focus failed writing i2c at line %d ...\n", __LINE__);
			return rc;
			}
		//printk("mt9p017_move_focus writing Success i2c at line %d ...\n", __LINE__);

		usleep(mt9p017_sw_damping_time_wait*10);
	}
	mt9p017_ctrl->curr_lens_pos = dest_lens_position;
	mt9p017_ctrl->curr_step_pos = dest_step_position;
	//printk("SM: curr_lens_pos/curr_step_pos = %d/%d\n", mt9p017_ctrl->curr_lens_pos, mt9p017_ctrl->curr_step_pos);
	//printk("mt9p017_move_focus exit.... \n");
	return rc;
}

static int32_t mt9p017_set_default_focus(struct msm_sensor_ctrl_t *s_ctrl,
	uint8_t af_step)
{
	int32_t rc = 0;

	CDBG("[CAMERA]mt9p017_set_default_focus : af_step = %d \n", af_step);

	if (mt9p017_ctrl->curr_step_pos != 0) {
		rc = mt9p017_move_focus(s_ctrl,
				MOVE_FAR, mt9p017_ctrl->curr_step_pos);
	} else {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			REG_VCM_NEW_CODE, 0x00, MSM_CAMERA_I2C_WORD_DATA);			
	}

	mt9p017_ctrl->curr_lens_pos = 0;
	mt9p017_ctrl->curr_step_pos = 0;

	return rc;
}

#if 0 // mo2jonghoo.lee 2013.01.07
static int32_t mt9p017_set_effect (struct msm_sensor_ctrl_t *s_ctrl,
	int8_t effect)
{
	return mt9p017_set_default_focus(s_ctrl, (uint8_t)effect);
}
#endif // mo2jonghoo.lee 2013.01.07

static void mt9p017_init_focus(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t i;

	CDBG("[CAMERA]mt9p017_init_focus\n");

//                                                 
	mt9p017_step_position_table[0] = af_infinity;
//                                                 

	for (i = 1; i <= mt9p017_linear_total_step; i++) {
		if (i <= mt9p017_nl_region_boundary1) {
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_nl_region_code_per_step1;
		} else {
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_l_region_code_per_step;
		}
		if (mt9p017_step_position_table[i] > 255)
			mt9p017_step_position_table[i] = 255;
	}
	mt9p017_ctrl->curr_lens_pos = 0;
}

static int32_t mt9p017_af_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("[CAMERA]mt9p017_af_power_down\n");

	if (mt9p017_ctrl->curr_lens_pos != 0)
	{
		mt9p017_set_default_focus(s_ctrl, 0);
		msleep(40);
	}
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		REG_VCM_CONTROL, 0x00, MSM_CAMERA_I2C_WORD_DATA);		
	return 0;
}

static int32_t mt9p017_power_up(struct msm_sensor_ctrl_t *s_ctrl) 
{
	memset(mt9p017_ctrl, 0x00, sizeof(*mt9p017_ctrl));
	s_ctrl->fps_divider = 1 * 0x00000400;
//No on JB	mt9p017_ctrl->pict_fps_divider = 1 * 0x00000400;
//No on JB	mt9p017_ctrl->set_test = TEST_OFF;
//	mt9p017_ctrl->prev_res = QTR_SIZE;
	s_ctrl->curr_res = MSM_SENSOR_RES_QTR /* QTR_SIZE */;
//No on JB	mt9p017_ctrl->pict_res = FULL_SIZE;	
	mt9p017_init_focus(s_ctrl);
	return msm_sensor_power_up(s_ctrl);
}

static int32_t mt9p017_power_down(struct msm_sensor_ctrl_t * s_ctrl)
{
	printk(KERN_EMERG "[CAMERA]mt9p017_power_down\n");
	mt9p017_af_power_down(s_ctrl);
	return msm_sensor_power_down(s_ctrl);
}

static int32_t mt9p017_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
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
		case CFG_SET_EFFECT:
			// mo2jonghoo.lee 2013.01.07 rc = mt9p017_set_effect(s_ctrl, cdata.cfg.effect);
			break;
			
#if 0 // mo2jonghoo.lee 2012.12.27 임시로 사용. 차후 분석 필요.
		case CFG_GET_CALIB_DATA:
			rc = mt9p017_read_eeprom_data(s_ctrl,
				&cdata.cfg.calib_info);
			
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;					
			break;
#endif // mo2jonghoo.lee 2012.12.27 임시로 사용. 차후 분석 필요.

		case CFG_MOVE_FOCUS:
			rc = mt9p017_move_focus(s_ctrl, 
				cdata.cfg.focus.dir, 
				cdata.cfg.focus.steps);			
			break;

		case CFG_SET_DEFAULT_FOCUS:		
			rc = mt9p017_set_default_focus(s_ctrl, 
				cdata.cfg.focus.steps);			
			break;	
			
		case CFG_GET_PICT_FPS:
			rc = mt9p017_get_pict_fps(s_ctrl,
				cdata.cfg.gfps.prevfps,
				&cdata.cfg.gfps.pictfps);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;			
			break;			

		case CFG_GET_PREV_L_PF:
			rc = mt9p017_get_prev_lines_pf(s_ctrl,
				&cdata.cfg.prevl_pf);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;				
			break;

		case CFG_GET_PREV_P_PL:
			rc = mt9p017_get_prev_pixels_pl(s_ctrl,
				&cdata.cfg.prevp_pl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;						
			break;

		case CFG_GET_PICT_L_PF:
			rc = mt9p017_get_pict_lines_pf(s_ctrl,
				&cdata.cfg.pictl_pf);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;						
			break;

		case CFG_GET_PICT_P_PL:
			rc = mt9p017_get_pict_pixels_pl(s_ctrl,
				&cdata.cfg.pictp_pl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;				
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			rc = mt9p017_get_pict_max_exp_lc(s_ctrl,
				&cdata.cfg.pict_max_exp_lc);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;					
			break;

		case CFG_SET_LENS_SHADING: //                                                                    
 			rc = mt9p017_lens_shading_enable(s_ctrl, cdata.cfg.lens_shading);
			break;

		case CFG_GET_AF_MAX_STEPS:
			rc = mt9p017_get_af_max_steps(s_ctrl,
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

static int32_t mt9p017_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
	
	int direction = move_params->dir;
	int32_t num_steps = move_params->num_steps;

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if(direction == MOVE_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction \n");
		return -EINVAL;
	}
	
	//printk("mt9p017_move_focus calculating dest_step_position \n");
	
	dest_step_position = mt9p017_ctrl->curr_step_pos + (step_direction * num_steps);
	if (dest_step_position < 0){
		dest_step_position = 0;
	}
	else if (dest_step_position > MT9P017_TOTAL_STEPS_NEAR_TO_FAR){
		dest_step_position = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;
	}
	if(dest_step_position == mt9p017_ctrl->curr_step_pos){
		//printk("mt9p017_move_focus ==  mt9p017_ctrl->curr_step_pos No Move exit \n");
		return rc;
	}
	//printk("Cur Step: %hd Step Direction: %hd Dest Step Pos: %hd Num Step: %hd\n", mt9p017_ctrl->curr_step_pos, step_direction, dest_step_position, num_steps);

	dest_lens_position = mt9p017_step_position_table[dest_step_position];
	target_dist = step_direction * (dest_lens_position - mt9p017_ctrl->curr_lens_pos);
	//printk("Target Dist: %hd\n", target_dist);

	if(step_direction < 0 && (target_dist >=
		mt9p017_step_position_table[mt9p017_damping_threshold])){
		small_step = (uint16_t)((target_dist/10));
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 1;
	}
	else{
		small_step = (uint16_t)(target_dist/4);
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 4;
	}
	//printk("mt9p017_move_focus small_step %d ...\n", small_step);

	for (next_lens_position = mt9p017_ctrl->curr_lens_pos + (step_direction * small_step);
	(step_direction * next_lens_position) <= (step_direction * dest_lens_position);
	next_lens_position += (step_direction * small_step)){
		//printk("mt9p017_move_focus next_lens_position %d ...\n", next_lens_position);
		//printk("mt9p017_move_focus writing i2c at line %d ...\n", __LINE__);
		// mo2jonghoo.lee 2013.02.19 rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, next_lens_position);
		rc = msm_camera_i2c_write(&a_ctrl->i2c_client, REG_VCM_NEW_CODE, next_lens_position, MSM_CAMERA_I2C_WORD_DATA);	// mo2jonghoo.lee 2013.02.19 
		if (rc < 0){
			//printk("mt9p017_move_focus failed writing i2c at line %d ...\n", __LINE__);
			return rc;
			}
		//printk("mt9p017_move_focus writing Success i2c at line %d ...\n", __LINE__);
		mt9p017_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9p017_sw_damping_time_wait*10);
	}
	if(mt9p017_ctrl->curr_lens_pos != dest_lens_position){
		//printk("mt9p017_move_focus writing i2c at line %d ...\n", __LINE__);
		//printk("mt9p017_move_focus curr_lens_pos = %d  dest_lens_position = %d ...\n", mt9p017_ctrl->curr_lens_pos, dest_lens_position);

		// mo2jonghoo.lee 2013.02.19 rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		// mo2jonghoo.lee 2013.02.19 	 dest_lens_position);
		rc = msm_camera_i2c_write(&a_ctrl->i2c_client, REG_VCM_NEW_CODE, dest_lens_position, MSM_CAMERA_I2C_WORD_DATA);	// mo2jonghoo.lee 2013.02.19 
		if (rc < 0){
			//printk("mt9p017_move_focus failed writing i2c at line %d ...\n", __LINE__);
			return rc;
			}
		//printk("mt9p017_move_focus writing Success i2c at line %d ...\n", __LINE__);

		usleep(mt9p017_sw_damping_time_wait*10);
	}
	mt9p017_ctrl->curr_lens_pos = dest_lens_position;
	mt9p017_ctrl->curr_step_pos = dest_step_position;
	//printk("SM: curr_lens_pos/curr_step_pos = %d/%d\n", mt9p017_ctrl->curr_lens_pos, mt9p017_ctrl->curr_step_pos);
	//printk("mt9p017_move_focus exit.... \n");
	return rc;
}


static int32_t mt9p017_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;

	if (mt9p017_ctrl->curr_step_pos != 0) {
		move_params->dir = MOVE_FAR;
		move_params->num_steps = mt9p017_ctrl->curr_step_pos;
		rc = mt9p017_actuator_move_focus(a_ctrl,
				move_params);
	} else {
	
		msm_camera_i2c_write(&a_ctrl->i2c_client,
			REG_VCM_NEW_CODE, 0x00, MSM_CAMERA_I2C_WORD_DATA);			
	}

	mt9p017_ctrl->curr_lens_pos = 0;
	mt9p017_ctrl->curr_step_pos = 0;

	a_ctrl->curr_step_pos = mt9p017_ctrl->curr_step_pos;

	return rc;
}

struct msm_actuator msm_actuator_table_mt9p017 = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = mt9p017_actuator_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus = mt9p017_actuator_set_default_focus,
		.actuator_init_focus = NULL,
		.actuator_i2c_write = NULL,
	},
};

#if 0
static int32_t mt9p017_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines;
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain | 0x1000,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, line,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static int32_t mt9p017_write_exp_snapshot_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines;
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain | 0x1000,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, line,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, (0x065C|0x2), MSM_CAMERA_I2C_WORD_DATA);

	return 0;
}

#endif

#if 0 // mo2jonghoo.lee 2013.01.07
static void mt9p017_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, 0x8250, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, 0x8658, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x0104, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, 0x065C, MSM_CAMERA_I2C_WORD_DATA);
}

static void mt9p017_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, 0x0058, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x301A, 0x0050, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x0104, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
}
#endif // mo2jonghoo.lee 2013.01.07

static const struct i2c_device_id mt9p017_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&mt9p017_s_ctrl},
	{ }
};

static struct i2c_driver mt9p017_i2c_driver = {
	.id_table = mt9p017_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mt9p017_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&mt9p017_i2c_driver);
}

static struct v4l2_subdev_core_ops mt9p017_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mt9p017_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mt9p017_subdev_ops = {
	.core = &mt9p017_subdev_core_ops,
	.video  = &mt9p017_subdev_video_ops,
};

static struct msm_sensor_fn_t mt9p017_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream, // mo2jonghoo.lee 2013.01.07 mt9p017_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream, // mo2jonghoo.lee 2013.01.07 mt9p017_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = mt9p017_set_fps,
	.sensor_write_exp_gain = mt9p017_write_exp_gain,
	.sensor_write_snapshot_exp_gain = mt9p017_write_exp_snapshot_gain,
	.sensor_csi_setting = msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = mt9p017_config,
	.sensor_power_up = mt9p017_power_up,
	.sensor_power_down = mt9p017_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t mt9p017_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = mt9p017_start_stream_settings, // mo2jonghoo.lee 2013.01.07
	.start_stream_conf_size = ARRAY_SIZE(mt9p017_start_stream_settings), // mo2jonghoo.lee 2013.01.07
	.stop_stream_conf = mt9p017_stop_stream_settings, // mo2jonghoo.lee 2013.01.07
	.stop_stream_conf_size = ARRAY_SIZE(mt9p017_stop_stream_settings), // mo2jonghoo.lee 2013.01.07
	.group_hold_on_conf = mt9p017_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(mt9p017_groupon_settings),
	.group_hold_off_conf = mt9p017_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(mt9p017_groupoff_settings),
	.init_settings = &mt9p017_init_conf[0],
	.init_size = ARRAY_SIZE(mt9p017_init_conf),
	.mode_settings = &mt9p017_confs[0],
	.output_settings = &mt9p017_dimensions[0],
	.num_conf = ARRAY_SIZE(mt9p017_confs),
};

static struct msm_sensor_ctrl_t mt9p017_s_ctrl = {
	.msm_sensor_reg = &mt9p017_regs,
	.sensor_i2c_client = &mt9p017_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.sensor_output_reg_addr = &mt9p017_reg_addr,
	.sensor_id_info = &mt9p017_id_info,
	.sensor_exp_gain_info = &mt9p017_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &mt9p017_csi_params_array[0],
	.msm_sensor_mutex = &mt9p017_mut,
	.sensor_i2c_driver = &mt9p017_i2c_driver,
	.sensor_v4l2_subdev_info = mt9p017_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9p017_subdev_info),
	.sensor_v4l2_subdev_ops = &mt9p017_subdev_ops,
	.func_tbl = &mt9p017_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

//module_init(msm_sensor_init_module);
late_initcall(msm_sensor_init_module);
MODULE_DESCRIPTION("Aptina 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");


