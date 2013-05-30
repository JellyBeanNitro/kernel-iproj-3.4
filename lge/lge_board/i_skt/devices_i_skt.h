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
 *
 */
#ifndef __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H
#define __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H

#define MSM_GSBI3_QUP_I2C_BUS_ID 0
#define MSM_GSBI4_QUP_I2C_BUS_ID 1
#define MSM_GSBI9_QUP_I2C_BUS_ID 2
#define MSM_GSBI8_QUP_I2C_BUS_ID 3
#define MSM_GSBI7_QUP_I2C_BUS_ID 4
#define MSM_GSBI10_QUP_I2C_BUS_ID 9
#define MSM_GSBI12_QUP_I2C_BUS_ID 5
#define MSM_SSBI1_I2C_BUS_ID     6
#define MSM_SSBI2_I2C_BUS_ID     7
#define MSM_SSBI3_I2C_BUS_ID     8
#ifdef CONFIG_LGE_FUEL_GAUGE
#define MSM_GSBI5_QUP_I2C_BUS_ID 11
#endif
#if defined (CONFIG_LGE_WIRELESS_CHARGER_MAX8971) || defined (CONFIG_LGE_WIRELESS_CHARGER_BQ24160)
#define MSM_GSBI11_QUP_I2C_BUS_ID 12
#endif


#define LGE_GSBI_BUS_ID_AUDIO_AMP_WM9093    MSM_GSBI3_QUP_I2C_BUS_ID
#define LGE_GSBI_BUS_ID_VIB_ISA1200         MSM_GSBI3_QUP_I2C_BUS_ID

/* GPIO related defines */
/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_GPIO_BASE)
#define PM8058_MPP_BASE			(PM8058_GPIO_BASE + PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_MPP_BASE)
#define PM8058_MPP_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_MPP_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define PM8901_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8901_GPIO_BASE)
#define PM8901_IRQ_BASE				(PM8058_IRQ_BASE + \
						NR_PMIC8058_IRQS)

#define PMIC_GPIO_EAR_SENSE_N			19
#define PMIC_GPIO_EAR_KEY_INT			20

/* gpios for audio control */
#define PMIC_GPIO_RCV_AMP_RESET			7
#define PMIC_GPIO_CAMCORDER_MIC_EN		8

#define GPIO_RCV_AMP_RESET				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_RCV_AMP_RESET - 1)
#define GPIO_CAMCORDER_MIC_EN			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAMCORDER_MIC_EN - 1)

/* gpios for ear jack detecion */
#define GPIO_EAR_SENSE_N			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_SENSE_N - 1)
#define GPIO_EAR_MIC_EN				58
#define GPIO_EARPOL_DETECT			126
#define GPIO_EAR_KEY_INT			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_KEY_INT - 1)

/* gpio and clock control for vibrator */
#define GPIO_LIN_MOTOR_EN		        158
#define GPIO_LIN_MOTOR_PWM	        	31

/* MIC BIAS configuration */
#define OTHC_MICBIAS_MAIN     OTHC_MICBIAS_0
#define OTHC_MICBIAS_SUB      OTHC_MICBIAS_1
#define OTHC_MICBIAS_HEADSET  OTHC_MICBIAS_2

#ifdef CONFIG_LGE_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#define MSM_GSBI1_QUP_I2C_BUS_ID  10
#endif

#if defined (CONFIG_MACH_LGE_I_BOARD_SKT)
#define GPIO_FUEL_INT					128  /*gpio for MAX17040 FUEL_INT */
#endif

#ifdef CONFIG_SND_SOC_MSM8660_APQ
extern struct platform_device msm_pcm;
extern struct platform_device msm_multi_ch_pcm;
extern struct platform_device msm_pcm_routing;
extern struct platform_device msm_cpudai0;
extern struct platform_device msm_cpudai1;
extern struct platform_device msm_cpudai_hdmi_rx;
extern struct platform_device msm_cpudai_bt_rx;
extern struct platform_device msm_cpudai_bt_tx;
extern struct platform_device msm_cpudai_fm_rx;
extern struct platform_device msm_cpudai_fm_tx;
extern struct platform_device msm_cpu_fe;
extern struct platform_device msm_stub_codec;
extern struct platform_device msm_voice;
extern struct platform_device msm_voip;
extern struct platform_device msm_lpa_pcm;
extern struct platform_device msm_pcm_hostless;
#endif

#ifdef CONFIG_SPI_QUP
extern struct platform_device msm_gsbi1_qup_spi_device;
extern struct platform_device msm_gsbi10_qup_spi_device;
#ifdef CONFIG_LGE_BROADCAST_TDMB
extern struct platform_device msm_gsbi11_qup_spi_device;	
#endif  //                          
#endif

#ifdef CONFIG_LGE_FUEL_GAUGE
extern struct platform_device msm_gsbi5_qup_i2c_device;
#endif 

#ifdef CONFIG_LGE_SENSOR_ACCELEROMETER  /*                                                       */
extern struct platform_device msm_gsbi10_qup_i2c_device;
#endif

#if defined (CONFIG_LGE_WIRELESS_CHARGER_MAX8971) || defined (CONFIG_LGE_WIRELESS_CHARGER_BQ24160)
extern struct platform_device msm_gsbi11_qup_i2c_device;
#endif /*                                                                                        */

extern struct platform_device msm_bus_apps_fabric;
extern struct platform_device msm_bus_sys_fabric;
extern struct platform_device msm_bus_mm_fabric;
extern struct platform_device msm_bus_sys_fpb;
extern struct platform_device msm_bus_cpss_fpb;
extern struct platform_device msm_bus_def_fab;

extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_gpio;
extern struct platform_device msm_device_vidc;

extern struct platform_device msm_charm_modem;
extern struct platform_device msm_device_tz_log;
#ifdef CONFIG_HW_RANDOM_MSM
extern struct platform_device msm_device_rng;
#endif
extern struct platform_device msm_device_csic0;
extern struct platform_device msm_device_csic1;
extern struct platform_device msm_device_vfe;
extern struct platform_device msm_device_vpe;

void __init msm8x60_init_irq(void);
void __init msm8x60_check_2d_hardware(void);

#ifdef CONFIG_MSM_DSPS
extern struct platform_device msm_dsps_device;
#endif

#ifdef CONFIG_MSM_BUS_SCALING
extern struct msm_bus_scale_pdata rotator_bus_scale_pdata;
#endif

#if defined(CONFIG_MSM_RPM_STATS_LOG)
extern struct platform_device msm_rpm_stat_device;
#endif

#ifdef CONFIG_MSM_RPM   /*                                                       */
extern struct platform_device msm8660_rpm_device;
#endif

#endif

void __init msm_fb_register_device(char *name, void *data);
