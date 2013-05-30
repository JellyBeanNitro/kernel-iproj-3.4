/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include "sensor_mt9p017.h"

struct mt9p017_i2c_reg_conf const pll_tbl[]=
{
	{0x301A, 0x0018},	 //reset_register
	{0x3064, 0xB800},	 //smia_test_2lane_mipi
	{0x31AE, 0x0202},	 //dual_lane_MIPI_interface
	{0x0300, 0x0005},	 //vt_pix_clk_div
	{0x0302, 0x0001},	 //vt_sys_clk_div
	{0x0304, 0x0002},	 //pre_pll_clk_div
	{0x0306, 0x002D},	 //pll_multipler
	{0x0308, 0x000A},	 //op_pix_clk_div
	{0x030A, 0x0001}	 //op_sys_clk_div

};

struct mt9p017_i2c_reg_conf const init_tbl[] =
{
//[20111220_rev04_recommended setting] 
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

	//mipi set
	{0x31B0, 0x00C4}, 
	{0x31B2, 0x0064},
	{0x31B4, 0x0E77},
	{0x31B6, 0x0D24},
	{0x31B8, 0x020E},
	{0x31BA, 0x0710},
	{0x31BC, 0x2A0D},	
	{0x31BE, 0xC007}, //  continuous
}; 
/* Preview	register settings	*/

/* read out mode register : 15-vert_flip, 14-horiz_mirror, 
					      11-x_bin_en, 10-xy_bin_en, 
					      8~6-x_odd_inc, 5~0-y_odd_inc */

struct mt9p017_i2c_reg_conf const mode_preview_tbl[]=
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

struct mt9p017_i2c_reg_conf const mode_preview_1080_tbl[]=
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

/* Snapshot register settings */
struct mt9p017_i2c_reg_conf const mode_snapshot_tbl[]=
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

struct mt9p017_i2c_reg_conf const lensrolloff_tbl[] = {
  {0x3780, 0x0000}              //  Poly_sc_enable                
  //{0x3780, 0x8000}              //  Poly_sc_enable
};


struct mt9p017_reg mt9p017_regs = {
	.pll_tbl = &pll_tbl[0],
	.plltbl_size = ARRAY_SIZE(pll_tbl),

	.init_tbl = &init_tbl[0],
	.inittbl_size = ARRAY_SIZE(init_tbl),

	.prev_tbl = &mode_preview_tbl[0],
	.prevtbl_size = ARRAY_SIZE(mode_preview_tbl),

	.prev_1080_tbl = &mode_preview_1080_tbl[0],
	.prevtbl_1080_size = ARRAY_SIZE(mode_preview_1080_tbl),

	.snap_tbl = &mode_snapshot_tbl[0],
	.snaptbl_size = ARRAY_SIZE(mode_snapshot_tbl),

	.lensroff_tbl = &lensrolloff_tbl[0],
	.lensroff_size = ARRAY_SIZE(lensrolloff_tbl),
};



