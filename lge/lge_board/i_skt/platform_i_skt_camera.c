/*
 * Copyright (C) 2011 LGE, Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/bootmem.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/camera.h>
#include "board_i_skt.h"
#include "devices_i_skt.h"

#ifdef CONFIG_LGE_DISPLAY_MIPI_LGIT_VIDEO_HD_PT		// js.chang: only LGIT defines
#define LGIT_IEF_SWITCH
#endif

/* MT9P017 Main Camera - 8M Bayer Camera*/
#define GPIO_CAM_MCLK						32
#define GPIO_CAM_I2C_SDA					47
#define GPIO_CAM_I2C_SCL					48
#define GPIO_8M_CAM_RESET_N					157
#define GPIO_CAM_I2C_SLAVE_ADDR					0x1A>>1
#if defined(LG_HW_REV1)
#define GPIO_8M_CAM_VCM_EN					156
#else
#define GPIO_8M_CAM_VCM_EN					94
#endif
#define CAM_I2C_SLAVE_ADDR				0x6C >> 2


/* MT9M114 VT Camera - 1.3M SOC Camera*/
#define GPIO_VT_CAM_RESET_N					57
#define GPIO_VT_OSC_EN						156
#define VT_CAM_I2C_SLAVE_ADDR					0x48>>1

/* LM3559 LED Flash driver*/
#define LED_FLASH_I2C_SLAVE_ADDR				0xA6>>1
#define GPIO_LED_FLASH_EN			    		154

#ifdef LGIT_IEF_SWITCH
extern int mipi_lgit_lcd_ief_off(void);
extern int mipi_lgit_lcd_ief_on(void);
#endif

/* Seperate main/vt camera power*/
//static struct regulator *ldo1_1p2;
/*
 * I disabled camera features.
 * So, to skip compilation error, I blocked the below codes !!!
 */
#if 0 /*                                                                                           */
static struct regulator *ldo9_2p8;
static struct regulator *lvs0_1p8;
static struct regulator *ldo8_2p8;
static struct regulator *vt_ldo4_2p8;
static struct regulator *vt_lvs3_1p8;
static struct regulator *vt_lvs1_1p8;
#endif
//========================================================================

#ifdef CONFIG_LGE_CAMERA
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(GPIO_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(GPIO_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 
	GPIO_CFG(GPIO_8M_CAM_RESET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(GPIO_VT_CAM_RESET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(GPIO_VT_OSC_EN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_LED_FLASH_EN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
};

static uint32_t camera_on_gpio_table_mt9p017[] = {
	GPIO_CFG(GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(GPIO_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 
	GPIO_CFG(GPIO_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 
	GPIO_CFG(GPIO_8M_CAM_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(GPIO_LED_FLASH_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)	//                                               
};

static uint32_t camera_on_gpio_table_mt9m114[] = {
	GPIO_CFG(GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(GPIO_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(GPIO_VT_CAM_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_VT_OSC_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};


#ifndef CONFIG_MSM_CAMERA_V4L2
static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, table[n], rc);
			break;
		}
	}
}

static int config_camera_on_gpios(void)
{
	int rc = 0;

	config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));

	return rc;
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_vt_cam(void)
{
	int rc = 0;

	config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));

	return rc;
}

static void config_camera_off_gpios_vt_cam(void)
{
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

void  main_camera_power_on (void)
{
	printk("%s: \n", __func__);

	gpio_set_value(157, 0); // RESET_BAR

	/* +1V8_5MCAM_VIO*/
	lvs0_1p8 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(lvs0_1p8)) {
		pr_err("%s: LVS0 get failed\n", __func__);
		lvs0_1p8 = NULL;
		return;
	}
	regulator_set_voltage(lvs0_1p8, 1800000, 1800000);
	regulator_enable(lvs0_1p8);

	msm_camio_clk_rate_set(24000000);
	gpio_set_value(157, 1); // RESET_BAR
	msleep(1);

	/* +2V8_5MCAM_AVDD*/
	ldo9_2p8 = regulator_get(NULL, "8058_l9");
	if (IS_ERR(ldo9_2p8)) {
		pr_err("%s: VREG LDO9 get failed\n", __func__);
		ldo9_2p8 = NULL;
		return;
	}
	regulator_set_voltage(ldo9_2p8, 2800000, 2800000);
	regulator_enable(ldo9_2p8);

	/* +2V8_5MCAM_AF*/
	ldo8_2p8 = regulator_get(NULL, "8058_l8");
	if (IS_ERR(ldo8_2p8)) {
		pr_err("%s: VREG LDO8 get failed\n", __func__);
		ldo8_2p8 = NULL;
		return;
	}
	regulator_set_voltage(ldo8_2p8, 2800000, 2800000);
	regulator_enable(ldo8_2p8);

	msleep(1);			
}


void  main_camera_power_off (void)
{
	printk("%s: \n", __func__);

	gpio_set_value(157, 0); // RESET_BAR

	if (lvs0_1p8) {
		regulator_disable(lvs0_1p8);
		regulator_put(lvs0_1p8);
	}
	if (ldo9_2p8) {
		regulator_disable(ldo9_2p8);
		regulator_put(ldo9_2p8);
	}
	if (ldo8_2p8) {
		regulator_disable(ldo8_2p8);
		regulator_put(ldo8_2p8);
	}
}

void  vt_camera_power_on (void)
{

	printk("%s: \n", __func__);

	gpio_set_value(GPIO_VT_OSC_EN, 0);

	/*VT Cam Power*/
	/* +1V8_VTCAM_DVDD*/
	vt_lvs3_1p8 = regulator_get(NULL, "8901_lvs3");
	if (IS_ERR(vt_lvs3_1p8)) {
		pr_err("%s: 8901 LVS3 get failed\n", __func__);
		vt_lvs3_1p8 = NULL;
		return;
	}
	regulator_set_voltage(vt_lvs3_1p8, 1800000, 1800000);
	regulator_enable(vt_lvs3_1p8);
	mdelay(10);

	/* +1V8_VTCAM_IOVDD*/
	vt_lvs1_1p8 = regulator_get(NULL, "8901_lvs1");
	if (IS_ERR(vt_lvs1_1p8)) {
		pr_err("%s: 8901 LVS1 get failed\n", __func__);
		vt_lvs1_1p8 = NULL;
		return;
	}
	regulator_set_voltage(vt_lvs1_1p8, 1800000, 1800000);
	regulator_enable(vt_lvs1_1p8);

	/* +2V8_VTCAM_AVDD*/
	vt_ldo4_2p8 = regulator_get(NULL, "8901_l4");
	if (IS_ERR(vt_ldo4_2p8)) {
		pr_err("%s: 8901 VREG LDO4 get failed\n", __func__);
		vt_ldo4_2p8 = NULL;
		return;
	}
	regulator_set_voltage(vt_ldo4_2p8, 2800000, 2800000);
	regulator_enable(vt_ldo4_2p8);

	mdelay(1);

	gpio_set_value(GPIO_VT_OSC_EN, 1);

	/* +1V8_8MCAM_VIO*/
	lvs0_1p8 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(lvs0_1p8)) {
		pr_err("%s: LVS0 get failed\n", __func__);
		lvs0_1p8 = NULL;
		return;
	}
	regulator_set_voltage(lvs0_1p8, 1800000, 1800000);
	regulator_enable(lvs0_1p8);
	printk("%s: [-]\n", __func__);

}

void  vt_camera_power_off (void)
{

	printk("%s: \n", __func__);

	gpio_set_value(GPIO_VT_OSC_EN, 0);

	if (vt_ldo4_2p8) {
		regulator_disable(vt_ldo4_2p8);
		regulator_put(vt_ldo4_2p8);
	}
	if (vt_lvs3_1p8) {
		regulator_disable(vt_lvs3_1p8);
		regulator_put(vt_lvs3_1p8);
	}
	if (vt_lvs1_1p8) {
		regulator_disable(vt_lvs1_1p8);
		regulator_put(vt_lvs1_1p8);
	}

	if (lvs0_1p8) {
		regulator_disable(lvs0_1p8);
		regulator_put(lvs0_1p8);
	}

}
#endif //#ifndef CONFIG_MSM_CAMERA_V4L2

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 283115520,
		.ib  = 452984832,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 283115520,
		.ib  = 452984832,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 283115520,
		.ib  = 452984832,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 319610880,
		.ib  = 511377408,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 566231040,
		.ib  = 905969664,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 69984000,
		.ib  = 111974400,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 320864256,
		.ib  = 513382810,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 320864256,
		.ib  = 513382810,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 566231040,
		.ib  = 905969664,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 706199040,
		.ib  = 1129918464,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 320864256,
		.ib  = 513382810,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 320864256,
		.ib  = 513382810,
	},
};

static struct msm_bus_vectors cam_stereo_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 212336640,
		.ib  = 339738624,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 25090560,
		.ib  = 40144896,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 239708160,
		.ib  = 383533056,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 79902720,
		.ib  = 127844352,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_stereo_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 300902400,
		.ib  = 481443840,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 230307840,
		.ib  = 368492544,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 245113344,
		.ib  = 392181351,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 106536960,
		.ib  = 170459136,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 106536960,
		.ib  = 170459136,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_video_vectors),
		cam_stereo_video_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_snapshot_vectors),
		cam_stereo_snapshot_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
	cam_bus_client_config,
	ARRAY_SIZE(cam_bus_client_config),
	.name = "msm_camera",
};
#endif

#ifndef CONFIG_MSM_CAMERA_V4L2
struct msm_camera_device_platform_data msm_camera_device_data_main_cam = {
	.camera_power_on   = main_camera_power_on,
	.camera_power_off  = main_camera_power_off,
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
#ifdef CONFIG_MSM_BUS_SCALING
	.cam_bus_scale_table = &cam_bus_client_pdata,
#endif
};
#endif	

struct msm_camera_device_platform_data msm_camera_device_data_vt_cam = {
#ifndef CONFIG_MSM_CAMERA_V4L2
	.camera_power_on   	= vt_camera_power_on,
	.camera_power_off  	= vt_camera_power_off,
	.camera_gpio_on  = config_camera_on_gpios_vt_cam,
	.camera_gpio_off = config_camera_off_gpios_vt_cam,
#endif	
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
#ifdef CONFIG_MSM_BUS_SCALING
	.cam_bus_scale_table = &cam_bus_client_pdata,
#endif
};

struct resource lge_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_MSM_CAMERA_V4L2
static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
		.ioclk = {
			.vfe_clk_rate =	228570000, //                                
		},
	},
	{
		.csid_core = 1,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
		.ioclk = {
			.vfe_clk_rate =	228570000,
		},
	},
};

#ifdef CONFIG_LGE_SENSOR_MT9P017
static int32_t ext_power_ctrl_mt9p017 (int enable)
{
	int32_t rc = 0;

	unsigned extra_gpio = GPIO_8M_CAM_RESET_N;
	if (enable) {
		rc = gpio_request(extra_gpio, "mt9p017_reset_n");

		if (rc < 0) {
			pr_err("%s: gpio_request failed\n", __func__);
			return rc;
		}
		
		gpio_direction_output(extra_gpio, 0);
		msleep(50);
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		msleep(10);

#ifdef LGIT_IEF_SWITCH
		mipi_lgit_lcd_ief_off();
#endif
	}
	else {
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		gpio_free(extra_gpio);
#ifdef LGIT_IEF_SWITCH
		mipi_lgit_lcd_ief_on();
#endif	

	}
		

	return rc;
}

static struct msm_camera_gpio_conf gpio_conf_mt9p017 = {
	.camera_off_table = camera_off_gpio_table,
	.camera_off_table_size = ARRAY_SIZE(camera_off_gpio_table),
	.camera_on_table = camera_on_gpio_table_mt9p017,
	.camera_on_table_size = ARRAY_SIZE(camera_on_gpio_table_mt9p017),
	.gpio_no_mux = 1,
};

static struct camera_vreg_t msm_cam_vreg_mt9p017[] = { 
	{"8058_lvs0", REG_VS, 1800000, 1800000, -1},
	{"8058_l9", REG_LDO, 2800000, 2800000, -1},
	{"8058_l8", REG_LDO, 2800000, 2800000, -1},
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9p017 = {
	.mount_angle	= 90, //270
	.cam_vreg = msm_cam_vreg_mt9p017,
	.num_vreg = ARRAY_SIZE(msm_cam_vreg_mt9p017),
	.gpio_conf = &gpio_conf_mt9p017,
	.ext_power_ctrl = ext_power_ctrl_mt9p017,
};
#endif

#ifdef CONFIG_LGE_SENSOR_MT9M114
static int32_t ext_power_ctrl_mt9m114 (int enable)
{
	int32_t rc = 0;

	unsigned extra_gpio;

	if (enable) {
		extra_gpio = GPIO_VT_OSC_EN;

		rc = gpio_request(extra_gpio, "mt9m114_osc_en");

		if (rc < 0) {
			pr_err("%s: gpio_request failed\n", __func__);
			return rc;
		}

		gpio_direction_output(extra_gpio, 0);
		msleep(10);
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		msleep(10);

		extra_gpio = GPIO_VT_CAM_RESET_N;
		rc = gpio_request(extra_gpio, "mt9m114_reset_n");

		if (rc < 0) {
			pr_err("%s: gpio_request failed\n", __func__);
			return rc;
		}

		gpio_direction_output(extra_gpio, 0);
		msleep(50);
		gpio_direction_output(extra_gpio, enable ? 1 : 0);
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		msleep(10);

#ifdef LGIT_IEF_SWITCH
		mipi_lgit_lcd_ief_off();
#endif		
	}
	else {
		extra_gpio = GPIO_VT_OSC_EN;
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		gpio_free(extra_gpio);

		extra_gpio = GPIO_VT_CAM_RESET_N;
		gpio_set_value(extra_gpio, enable ? 1 : 0);
		gpio_free(extra_gpio);

#ifdef LGIT_IEF_SWITCH
		mipi_lgit_lcd_ief_on();
#endif			
	}

	return rc;
}

static struct msm_camera_gpio_conf gpio_conf_mt9m114 = {
	.camera_off_table = camera_off_gpio_table,
	.camera_off_table_size = ARRAY_SIZE(camera_off_gpio_table),
	.camera_on_table = camera_on_gpio_table_mt9m114,
	.camera_on_table_size = ARRAY_SIZE(camera_on_gpio_table_mt9m114),
	.gpio_no_mux = 1,
};

static struct camera_vreg_t msm_cam_vreg_mt9m114[] = {
	{"8901_lvs3", REG_VS, 1800000, 1800000, -1},
	{"8901_lvs1", REG_VS, 1800000, 1800000, -1},
	{"8901_l4", REG_LDO, 2800000, 2800000, -1},
	{"8058_lvs0", REG_VS, 1800000, 1800000, -1}, // mo2jonghoo.lee 2013.01.14
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9m114 = {
	.mount_angle	= 270,
	.cam_vreg = msm_cam_vreg_mt9m114,
	.num_vreg = ARRAY_SIZE(msm_cam_vreg_mt9m114),
	.gpio_conf = &gpio_conf_mt9m114,
	.ext_power_ctrl = ext_power_ctrl_mt9m114,
};
#endif
#endif //CONFIG_MSM_CAMERA_V4L2

/*                                                                        
                        
                                                                        */
#ifdef CONFIG_LGE_FLASH_LM3559
struct led_flash_platform_data {
	int gpio_en;
};

static struct led_flash_platform_data lm3559_flash_pdata = {
	.gpio_en		= GPIO_LED_FLASH_EN,
};

static struct i2c_board_info cam_i2c_flash_info[] = {
	{
		I2C_BOARD_INFO("lm3559",LED_FLASH_I2C_SLAVE_ADDR),
		.type = "lm3559",
		.platform_data = &lm3559_flash_pdata,
	}
};
#endif

/*                                                                        
                                                                  
                                                                        */
#ifdef CONFIG_LGE_SENSOR_MT9P017
static struct msm_camera_sensor_flash_data flash_mt9p017 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	//.flash_src		= &lge_flash_src      //js.kim
};
#ifdef CONFIG_MSM_ACTUATOR //                        
static struct i2c_board_info msm_act_main_cam_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x11),	
};

static struct msm_actuator_info msm_act_main_cam_3_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_3,
	.bus_id         = MSM_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd 	= 1,
	.vcm_enable     = 1,
};
#endif
static struct msm_camera_sensor_info msm_camera_sensor_mt9p017_data = {
	.sensor_name	= "mt9p017",
#ifdef CONFIG_MSM_CAMERA_V4L2
	.pdata	= &msm_camera_csi_device_data[0],
	.sensor_platform_info = &sensor_board_info_mt9p017,
	.camera_type = BACK_CAMERA_2D,
#else	
	.sensor_reset	= GPIO_8M_CAM_RESET_N,
	.sensor_pwd		= 85,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data_main_cam,
	.resource		= lge_camera_resources,
	.num_resources	= ARRAY_SIZE(lge_camera_resources),
#endif
	.flash_data		= &flash_mt9p017,
	.csi_if			= 1,
#ifdef CONFIG_MSM_ACTUATOR //                         
	.actuator_info = &msm_act_main_cam_3_info,
#endif	
};

#ifndef CONFIG_MSM_CAMERA_V4L2
struct platform_device msm_camera_sensor_mt9p017 = {
	.name	= "msm_camera_mt9p017",
	.dev	= {
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
};
#endif
#endif

#ifdef CONFIG_LGE_SENSOR_MT9M114
static struct msm_camera_sensor_flash_data flash_mt9m114 = {
	.flash_type		= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9m114_data = {
	.sensor_name	= "mt9m114",
#ifdef CONFIG_MSM_CAMERA_V4L2	
	.pdata	= &msm_camera_csi_device_data[1],
	.sensor_platform_info = &sensor_board_info_mt9m114,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
#else			
	.sensor_reset	= GPIO_VT_CAM_RESET_N,
	.sensor_pwd		= 85,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data_vt_cam,
	.resource		= lge_camera_resources,
	.num_resources	= ARRAY_SIZE(lge_camera_resources),
#endif
	.flash_data		= &flash_mt9m114,
	.csi_if			= 1
};

#ifndef CONFIG_MSM_CAMERA_V4L2
struct platform_device msm_camera_sensor_mt9m114 = {
	.id 	= 0,
	.name	= "msm_camera_mt9m114",
	.dev	= {
		.platform_data = &msm_camera_sensor_mt9m114_data,
	},
};
#endif
#endif

#ifdef CONFIG_MSM_CAMERA_V4L2
static struct i2c_board_info lge_camera_boardinfo[] __initdata = {
#ifdef CONFIG_LGE_SENSOR_MT9P017
	{
		I2C_BOARD_INFO("mt9p017", CAM_I2C_SLAVE_ADDR),
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
#endif
#ifdef CONFIG_LGE_SENSOR_MT9M114
	{
		I2C_BOARD_INFO("mt9m114", VT_CAM_I2C_SLAVE_ADDR),
		.platform_data = &msm_camera_sensor_mt9m114_data,
	},
#endif
};
#endif
#endif /*                 */
static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

struct platform_device *camera_devices[] __initdata = {
		&msm_camera_server,
		&msm_device_csic0,
		&msm_device_csic1,
		&msm_device_vfe,
		&msm_device_vpe,
};

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_FLUID (1 << 4)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry camera_i2c_devices[] __initdata = {
#ifdef CONFIG_LGE_FLASH_LM3559 
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		cam_i2c_flash_info,
		ARRAY_SIZE(cam_i2c_flash_info),
	},
#endif
#ifdef CONFIG_LGE_CAMERA
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		lge_camera_boardinfo,
		ARRAY_SIZE(lge_camera_boardinfo),
	},
#endif
};

void __init i2c_register_camera_info(void){

	int i;
	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(camera_i2c_devices); ++i) {
		i2c_register_board_info(camera_i2c_devices[i].bus,
				camera_i2c_devices[i].info,
				camera_i2c_devices[i].len);
	}
}
#endif /*CONFIG_I2C*/

void __init lge_camera_init(void){

	printk(KERN_EMERG "[CAMERA/+]msm_camera_init\n");
	platform_add_devices(camera_devices, ARRAY_SIZE(camera_devices));
	printk(KERN_EMERG "[CAMERA/-]msm_camera_init\n");
}
