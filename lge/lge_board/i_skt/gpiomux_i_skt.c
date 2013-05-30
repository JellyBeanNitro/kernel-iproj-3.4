/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <mach/gpiomux.h>
#include "gpiomux_i_skt.h"
#include "devices_i_skt.h"

#ifdef CONFIG_LGE_PM_CURRENT_CONSUMPTION_FIX
static struct gpiomux_setting msm_gpio81_cfg_suspend2 = /* BOOT_CONFIG_0 */
{
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting msm_gpio84_cfg_suspend2 = /* BOOT_CONFIG_1 */
{
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting msm_gpio76_cfg_suspend2 = /* BOOT_CONFIG_6*/
{
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8x60_current_configs[] __initdata = {
	/* BOOT_CONFIG_0 */
	{
		.gpio      = 81,
		.settings = {
			[GPIOMUX_SUSPENDED] = &msm_gpio81_cfg_suspend2,
		},
	},
	/* BOOT_CONFIG_1 */
	{
		.gpio      = 84,
		.settings = {
			[GPIOMUX_SUSPENDED] = &msm_gpio84_cfg_suspend2,
		},
	},
	/* BOOT_CONFIG_6*/
	{
		.gpio      = 76,
		.settings = {
			[GPIOMUX_SUSPENDED] = &msm_gpio76_cfg_suspend2,
		},
	},
};
//for atnt rock_bottom end
#endif

static struct gpiomux_setting console_uart = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

//                                   
static struct gpiomux_setting wifi_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
//                                   


/* The SPI configurations apply to GSBI1 and GSBI10 */
static struct gpiomux_setting spi_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting spi_suspended_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting spi_suspended_cs_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* This I2C active configuration applies to GSBI3 and GSBI4 */
static struct gpiomux_setting i2c_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#ifdef CONFIG_LGE_FUEL_GAUGE
static struct gpiomux_setting gsbi5 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting i2c_active_gsbi7 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* This I2C suspended configuration applies to GSBI3, GSBI4 and GSBI7 */
static struct gpiomux_setting i2c_suspended_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi10 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_LGE_BROADCAST_TDMB
static struct gpiomux_setting gsbi11 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting bcast_ctrl_pin = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* DMB_INT_N */
static struct gpiomux_setting DMB_INT = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif	//                          

#if defined (CONFIG_LGE_WIRELESS_CHARGER_MAX8971) || defined (CONFIG_LGE_WIRELESS_CHARGER_BQ24160)
static struct gpiomux_setting gsbi11 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting gsbi12 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting ps_hold = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting msm_snddev_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting msm_snddev_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdcc1_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc1_dat_4_7_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc1_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc1_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_dat_4_7_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc2_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc5_dat_0_3_cmd_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc5_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc5_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting aux_pcm_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting aux_pcm_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting uart1dm_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting uart1dm_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_status_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_sync_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_sync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_3_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting pmic_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct gpiomux_setting uart9dm_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA ,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gsbi9 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_vfr_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting mdm2ap_vfr_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_kpdpwr_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_LGE_AUDIO 
#if 0 /* error CAMCORDER_MIC_EN180 greater than max:173 */
static struct gpiomux_setting camcorder_mic_en_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
#endif

static struct gpiomux_setting motor_en_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting lin_motor_pwm_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
//	.dir = GPIOMUX_OUT_LOW,
};

#ifdef CONFIG_LGE_HEADSET_DETECTION_FSA8008
static struct gpiomux_setting ear_mic_en_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting earpole_detect_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
#endif

#endif

#if defined (CONFIG_MACH_LGE_I_BOARD_SKT)
static struct gpiomux_setting fuel_int_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
#endif

static struct gpiomux_setting mdm2ap_vddmin_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_vddmin_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
//                                          
#if defined(CONFIG_LGE_NFC_PN544_C2)
static struct gpiomux_setting nfc_pn544pn65n_ven_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting nfc_pn544pn65n_irq_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting nfc_pn544pn65n_firm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};
#endif
//                                        

#ifdef CONFIG_LGE_MHL_SII9244
static struct gpiomux_setting mhl_detect_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting mhl_reset_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config msm8x60_mhl_configs[] __initdata = {
  /* MHL_INT */
	{
		.gpio      = 30,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_detect_cfg,
		},
	},
	/* USB_MHL_SEL */
	{
		.gpio      = 139,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_reset_n_cfg,
		},
	},
	/* MHL_RESET_N */
	{
		.gpio      = 142,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_reset_n_cfg,
		},
	},
	/* MHL_WAKE_UP */
	{
		.gpio      = 153,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_reset_n_cfg,
		},
	},
};
#endif
static struct msm_gpiomux_config msm8x60_gsbi_configs[] __initdata = {
	{
		.gpio      = 33,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 34,
		.settings = {
			//                                  
			//[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			//[GPIOMUX_ACTIVE]    = &spi_active,
			[GPIOMUX_ACTIVE]    = &wifi_active,
			[GPIOMUX_SUSPENDED] = &wifi_active,
			
		},
	},
	{
		.gpio      = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_cs_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 36,
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE]    = &spi_active,
		},
	},
	{
		.gpio      = 43,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 44,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},			
//                                          
#if defined(CONFIG_LGE_NFC_PN544_C2)
	{
		.gpio      = 46,
		.settings = {
			[GPIOMUX_SUSPENDED] = &nfc_pn544pn65n_firm_cfg,
		},
	},
#endif
//                                        
	{
		.gpio      = 47,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
	{
		.gpio      = 48,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active,
		},
	},
#if defined(CONFIG_LGE_FUEL_GAUGE) || defined(CONFIG_LGE_SWITCHING_CHARGER_MAX8971) || defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
	{
		.gpio      = 51,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
	{
		.gpio      = 52,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
#endif
	{
		.gpio      = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active_gsbi7,
		},
	},
	{
		.gpio      = 60,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspended_config,
			[GPIOMUX_ACTIVE]    = &i2c_active_gsbi7,
		},
	},
	{
		.gpio      = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8,
		},
	},
	{
		.gpio      = 65,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8,
		},
	},

#ifdef CONFIG_LGE_SENSOR_ACCELEROMETER
	{
		.gpio	   = 72,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{
		.gpio	   = 73,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},	
#endif
#ifdef CONFIG_LGE_BROADCAST_TDMB

	/* DMB_RESET */
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_SUSPENDED] = &bcast_ctrl_pin,
		},
	},

	/* DMB EN */
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_SUSPENDED] = &bcast_ctrl_pin,
		},
	},

	/* DMB SPI CLOCK */
	{
		.gpio	   = 103,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},

	/* DMB SPI CS */
	{
		.gpio	   = 104,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},

	/* DMB SPI MISO */
	{
		.gpio	   = 105,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},

	/* DMB SPI MOSI */
	{
		.gpio	   = 106,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},

	/* DMB_INT_N */
	{
		.gpio	   = 107,
		.settings = {
			[GPIOMUX_SUSPENDED] = &DMB_INT,
		},
	},
#endif	//                          

#if defined (CONFIG_LGE_WIRELESS_CHARGER_MAX8971) || defined (CONFIG_LGE_WIRELESS_CHARGER_BQ24160)
	{
		.gpio	   = 103,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},
	{
		.gpio	   = 104,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi11,
		},
	},
#endif
	{
		.gpio	   = 115,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{
		.gpio	   = 116,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},	
};

//                                                                
static struct msm_gpiomux_config msm8x60_ebi2_configs[] __initdata = {
	{
		.gpio      = 123,
		.settings = {
#if defined(CONFIG_LGE_NFC_PN544_C2)
			[GPIOMUX_SUSPENDED] = &nfc_pn544pn65n_irq_cfg,
#else
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
#endif
		},
	},
	{
		.gpio      = 130,
		.settings = {
#if defined(CONFIG_LGE_NFC_PN544_C2)
			[GPIOMUX_SUSPENDED] = &nfc_pn544pn65n_ven_cfg,
#else
			[GPIOMUX_SUSPENDED] = &ebi2_a_d,
#endif
		},
	},
};
//                                                              

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
#endif

static struct msm_gpiomux_config msm8x60_uart_configs[] __initdata = {
	{ /* UARTDM_TX */
		.gpio      = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_RX */
		.gpio      = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_CTS */
		.gpio      = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{ /* UARTDM_RFR */
		.gpio      = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart1dm_active,
			[GPIOMUX_SUSPENDED] = &uart1dm_suspended,
		},
	},
	{
		.gpio      = 115,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
	{
		.gpio      = 116,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
#if !defined(CONFIG_USB_PEHCI_HCD) && !defined(CONFIG_USB_PEHCI_HCD_MODULE)
	/* USB ISP1763 may also use 117 GPIO */
	{
		.gpio      = 117,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
#endif
	{
		.gpio      = 118,
		.settings = {
			[GPIOMUX_SUSPENDED] = &console_uart,
		},
	},
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_gpiomux_config msm8x60_charm_uart_configs[] __initdata = {
	{ /* UART9DM  RX */
		.gpio	   = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart9dm_active,
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
	{ /* UART9DM TX */
		.gpio	   = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &uart9dm_active,
			[GPIOMUX_SUSPENDED] = &gsbi9,
		},
	},
};
#endif

static struct msm_gpiomux_config msm8x60_aux_pcm_configs[] __initdata = {
	{
		.gpio = 111,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
	{
		.gpio = 114,
		.settings = {
			[GPIOMUX_ACTIVE]    = &aux_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &aux_pcm_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_sdc_configs[] __initdata = {
	/* SDCC1 data[0] */
	{
		.gpio = 159,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[1] */
	{
		.gpio = 160,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[2] */
	{
		.gpio = 161,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[3] */
	{
		.gpio = 162,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[4] */
	{
		.gpio = 163,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[5] */
	{
		.gpio = 164,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[6] */
	{
		.gpio = 165,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 data[7] */
	{
		.gpio = 166,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 CLK */
	{
		.gpio = 167,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
	/* SDCC1 CMD */
	{
		.gpio = 168,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc1_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc1_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_charm_sdc_configs[] __initdata = {
	/* SDCC5 cmd */
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[3]*/
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 clk */
	{
		.gpio = 97,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[2]*/
	{
		.gpio = 98,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[1]*/
	{
		.gpio = 99,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* SDCC5 data[0]*/
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc5_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc5_suspend_config,
		},
	},
	/* MDM2AP_SYNC */
	{
		.gpio = 129,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_sync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_sync_suspend_cfg,
		},
	},

	/* MDM2AP_VDDMIN */
	{
		.gpio = 140,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_vddmin_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_vddmin_suspend_cfg,
		},
	},
	/* SDCC2 data[0] */
	{
		.gpio = 143,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[1] */
	{
		.gpio = 144,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[2] */
	{
		.gpio = 145,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[3] */
	{
		.gpio = 146,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[4] */
	{
		.gpio = 147,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[5] */
	{
		.gpio = 148,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[6] */
	{
		.gpio = 149,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 data[7] */
	{
		.gpio = 150,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_4_7_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
	/* SDCC2 CMD */
	{
		.gpio = 151,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_dat_0_3_cmd_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},

	/* SDCC2 CLK */
	{
		.gpio = 152,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_snd_configs[] __initdata = {
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &msm_snddev_active_config,
			[GPIOMUX_SUSPENDED] = &msm_snddev_suspend_config,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &msm_snddev_active_config,
			[GPIOMUX_SUSPENDED] = &msm_snddev_suspend_config,
		},
	},
};

static struct msm_gpiomux_config msm8x60_mdp_vsync_configs[] __initdata = {
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8x60_hdmi_configs[] __initdata = {
	{
		.gpio = 169,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 170,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 171,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 172,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
};

/* Because PMIC drivers do not use gpio-management routines and PMIC
 * gpios must never sleep, a "good enough" config is obtained by placing
 * the active config in the 'suspended' slot and leaving the active
 * config invalid: the suspended config will be installed at boot
 * and never replaced.
 */

static struct msm_gpiomux_config msm8x60_pmic_configs[] __initdata = {
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pmic_suspended_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pmic_suspended_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8x60_common_configs[] __initdata = {
	/* MDM2AP_STATUS */
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdm2ap_status_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_suspend_cfg,
		},
	},
	/* PS_HOLD */
	{
		.gpio      = 92,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ps_hold,
		},
	},
};

static struct msm_gpiomux_config msm8x60_charm_configs[] __initdata = {
	/* AP2MDM_WAKEUP */
	{
		.gpio = 135,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_VFR */
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE] = &mdm2ap_vfr_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_vfr_suspend_cfg,
		}
	},
	/* AP2MDM_STATUS */
	{
		.gpio = 136,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_STATUS */
	{
		.gpio = 134,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	/* MDM2AP_WAKEUP */
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_ERRFATAL */
	{
		.gpio = 133,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
	/* AP2MDM_ERRFATAL */
	{
		.gpio = 93,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* AP2MDM_KPDPWR_N */
	{
		.gpio = 132,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	},
	/* AP2MDM_PMIC_RESET_N */
	{
		.gpio = 131,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	}
};

#ifdef CONFIG_LGE_AUDIO
static struct msm_gpiomux_config msm8x60_audio_configs[] __initdata = {
#if 0 /* error CAMCORDER_MIC_EN180 greater than max:173 */
	/* CAMCORDER_MIC_EN */
	{
		.gpio = GPIO_CAMCORDER_MIC_EN,
		.settings = {
			[GPIOMUX_SUSPENDED] = &camcorder_mic_en_cfg,
		}
	},
#endif
	/* MOTOR_EN*/
	{
		.gpio = GPIO_LIN_MOTOR_EN,
		.settings = {
			[GPIOMUX_SUSPENDED] = &motor_en_cfg,
		}
	},
        /* LIN_MOTOR_PWM*/
	{
		.gpio = GPIO_LIN_MOTOR_PWM,
		.settings = {
			[GPIOMUX_SUSPENDED] = &lin_motor_pwm_cfg,
		}
	},
#ifdef CONFIG_LGE_HEADSET_DETECTION_FSA8008
	/* EAR_MIC_EN */
	{
		.gpio = GPIO_EAR_MIC_EN,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ear_mic_en_cfg,
		}
	},
	/* EARPOL_DETECT */
	{
		.gpio = GPIO_EARPOL_DETECT,
		.settings = {
			[GPIOMUX_SUSPENDED] = &earpole_detect_cfg,
		}
	},
#endif
};
#endif

#if defined (CONFIG_MACH_LGE_I_BOARD_SKT)
static struct msm_gpiomux_config msm8x60_max17040_configs[] __initdata = {
	/* FUEL_INT*/
	{
		.gpio = GPIO_FUEL_INT,
		.settings = {
			[GPIOMUX_SUSPENDED] = &fuel_int_cfg,
		}
	},
};
#endif
#if 0
struct msm_gpiomux_configs
msm8x60_charm_gpiomux_cfgs[] __initdata = {
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
#ifdef CONFIG_MSM_GSBI9_UART
	{msm8x60_charm_uart_configs, ARRAY_SIZE(msm8x60_charm_uart_configs)},
#endif
	{msm8x60_ts_configs, ARRAY_SIZE(msm8x60_ts_configs)},
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mi2s_configs, ARRAY_SIZE(msm8x60_mi2s_configs)},
	{msm8x60_lcdc_configs, ARRAY_SIZE(msm8x60_lcdc_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_cam_configs, ARRAY_SIZE(msm8x60_cam_configs)},
	{msm8x60_tmg200_configs, ARRAY_SIZE(msm8x60_tmg200_configs)},
	{msm8x60_charm_sdc_configs, ARRAY_SIZE(msm8x60_charm_sdc_configs)},
	{msm8x60_charm_configs, ARRAY_SIZE(msm8x60_charm_configs)},
	{NULL, 0},
};
#endif

struct msm_gpiomux_configs
msm8x60_i_skt_gpiomux_cfgs[] __initdata = {
#ifdef CONFIG_LGE_PM_CURRENT_CONSUMPTION_FIX
	{msm8x60_current_configs, ARRAY_SIZE(msm8x60_current_configs)}, //for  rock_bottom
#endif	
#ifdef CONFIG_LGE_MHL_SII9244
	{msm8x60_mhl_configs, ARRAY_SIZE(msm8x60_mhl_configs)},
#endif
	{msm8x60_gsbi_configs, ARRAY_SIZE(msm8x60_gsbi_configs)},
	{msm8x60_uart_configs, ARRAY_SIZE(msm8x60_uart_configs)},
#ifdef CONFIG_MSM_GSBI9_UART
	{msm8x60_charm_uart_configs, ARRAY_SIZE(msm8x60_charm_uart_configs)},
#endif
	{msm8x60_aux_pcm_configs, ARRAY_SIZE(msm8x60_aux_pcm_configs)},
	{msm8x60_sdc_configs, ARRAY_SIZE(msm8x60_sdc_configs)},
	{msm8x60_snd_configs, ARRAY_SIZE(msm8x60_snd_configs)},
	{msm8x60_mdp_vsync_configs, ARRAY_SIZE(msm8x60_mdp_vsync_configs)},
	{msm8x60_hdmi_configs, ARRAY_SIZE(msm8x60_hdmi_configs)},
	{msm8x60_pmic_configs, ARRAY_SIZE(msm8x60_pmic_configs)},
	{msm8x60_common_configs, ARRAY_SIZE(msm8x60_common_configs)},
	{msm8x60_charm_sdc_configs, ARRAY_SIZE(msm8x60_charm_sdc_configs)},
	{msm8x60_charm_configs, ARRAY_SIZE(msm8x60_charm_configs)},
#ifdef CONFIG_LGE_AUDIO
	{msm8x60_audio_configs, ARRAY_SIZE(msm8x60_audio_configs)},
//                                                                
#if defined (CONFIG_MACH_LGE_I_BOARD_SKT)
	{msm8x60_max17040_configs, ARRAY_SIZE(msm8x60_max17040_configs)},
#endif
	{msm8x60_ebi2_configs, ARRAY_SIZE(msm8x60_ebi2_configs)},
//                                                              
#endif
	{NULL, 0},
};

void __init msm8x60_init_gpiomux(struct msm_gpiomux_configs *cfgs)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err("%s failure: %d\n", __func__, rc);
		return;
	}

	while (cfgs->cfg) {
		msm_gpiomux_install(cfgs->cfg, cfgs->ncfg);
		++cfgs;
	}
}
