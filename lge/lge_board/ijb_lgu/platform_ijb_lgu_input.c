/*
  * Copyright (C) 2009 LGE, Inc.
  * 
  * Author: Joon young Lee <joon0.lee@lge.com>
  *
  * This software is licensed under the terms of the GNU General Public
  * License version 2, as published by the Free Software Foundation, and
  * may be copied, distributed, and modified under those terms.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  */

#include <linux/types.h>
#include <linux/list.h>
#include <linux/err.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <board_lge.h>

#include "devices_ijb_lgu.h"

#include <apds9900.h>

#include <linux/i2c.h>
#include <linux/bootmem.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include "board_ijb_lgu.h"


#ifdef CONFIG_LGE_SENSOR



#ifdef CONFIG_LGE_PMIC8058_REGULATOR
struct regulator *pm8058_l11; // for Sensor, +3V0_SENSOR 
static int power_set_for_8058_l11(unsigned char onoff)
{
	int rc = -EINVAL;
	
	if(!pm8058_l11) {
		pm8058_l11 = regulator_get(NULL, "8058_l11");
		if (IS_ERR(pm8058_l11)) {
			pr_err("%s: line: %d, vreg_get failed (%ld)\n",
			__func__, __LINE__, PTR_ERR(pm8058_l11));
			rc = PTR_ERR(pm8058_l11);
			return rc;
		}
	}	
	if (onoff) 
	{

		
		rc = regulator_set_voltage(pm8058_l11, 3000000, 3000000);
		if (rc) {
			pr_err("%s: line: %d, unable to set pm8058_l11 voltage to 3.0 V\n",__func__,__LINE__);
			goto vreg_l11_fail;
		}
		rc = regulator_enable(pm8058_l11);
		if (rc) {
			pr_err("%s: line: %d, vreg_enable failed %d\n", __func__, __LINE__, rc);
			goto vreg_l11_fail;
		}
	} 
	else 
	{
		rc = regulator_disable(pm8058_l11);
		if (rc) {
			pr_err("%s: line: %d, vreg_disable failed %d\n",__func__, __LINE__, rc);
			goto vreg_l11_fail;
		}
	}
//	printk(KERN_INFO "%s: line: %d\n", __func__, __LINE__);
	return 0;
	
vreg_l11_fail:
	regulator_put(pm8058_l11);
	pm8058_l11 = NULL;
	return rc;	

}
#endif


#if defined (CONFIG_LGE_PMIC8058_REGULATOR) && defined (CONFIG_LGE_SENSOR_PROXIMITY)
static int sensor_power_pm8058_l15 = false;
struct regulator *pm8058_l15; //for Proximity, RPM_VREG_ID_PM8058_L15
static int power_set_for_8058_l15(unsigned char onoff)
{
	int rc = -EINVAL;
	printk(KERN_INFO "%s: prox/als power line: %d, onoff(%d)\n", __func__, __LINE__, onoff);

#if 0
	if(sensor_power_pm8058_l15 == onoff){
		printk(KERN_INFO "don't need to handle %s, onoff; %d", __func__, onoff);
		return 0;
	}	
#endif
	
	if(!pm8058_l15) {
		pm8058_l15 = regulator_get(NULL, "8058_l15");
		if (IS_ERR(pm8058_l15)) {
			pr_err("%s: line: %d, vreg_get failed (%ld)\n",
			__func__, __LINE__, PTR_ERR(pm8058_l15));
			rc = PTR_ERR(pm8058_l15);
			return rc;
		}
	}

	if (onoff) {
		rc = regulator_set_voltage(pm8058_l15, 2850000, 2850000);
		if (rc) {
			pr_err("%s: line: %d, unable to set pm8058_l15 voltage to 2.85 V\n",__func__,__LINE__);
			goto vreg_l15_fail;
		}
		rc = regulator_enable(pm8058_l15);
		if (rc) {
			pr_err("%s: line: %d, vreg_enable failed %d\n", __func__, __LINE__, rc);
			goto vreg_l15_fail;
		}
		sensor_power_pm8058_l15 = true;		
	} else {
		rc = regulator_disable(pm8058_l15);
		if (rc) {
			pr_err("%s: line: %d, vreg_disable failed %d\n",__func__, __LINE__, rc);
			goto vreg_l15_fail;
		}
		sensor_power_pm8058_l15 = false;				
	}

	return 0;

vreg_l15_fail:
	regulator_put(pm8058_l15);
	pm8058_l15 = NULL;
	return rc;	
}
#endif 

#if defined (CONFIG_LGE_MACH_BOARD_REVA)
#define GPIO_AXIS_I2C_SDA			72	
#define GPIO_AXIS_I2C_SCL			73	
#define GPIO_SENSOR_I2C_SDA		116 
#define GPIO_SENSOR_I2C_SCL		115 
#elif defined (CONFIG_LGE_MACH_BOARD_REVB)||defined (CONFIG_LGE_MACH_BOARD_REVC)
#define GPIO_AXIS_I2C_SDA			116	
#define GPIO_AXIS_I2C_SCL			115
#define GPIO_SENSOR_I2C_SDA		72
#define GPIO_SENSOR_I2C_SCL		73
#endif
#define GPIO_3AXIS_INT 			38
#define GPIO_GYRO_INT			37
#define GPIO_COMPASS_INT		39
#define GPIO_PROXIMITY_OUT_INT	41

static unsigned sensor_int_gpio[] = {GPIO_GYRO_INT, GPIO_3AXIS_INT, GPIO_COMPASS_INT};
static unsigned sensor_config_power_on[] = {
	GPIO_CFG(GPIO_GYRO_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_3AXIS_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_COMPASS_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	
};
#if 0
static unsigned sensor_config_power_off[] = {
	GPIO_CFG(GPIO_GYRO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_3AXIS_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),		
	GPIO_CFG(GPIO_COMPASS_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),			
};
#endif

void __init sensor_power_init(void)
{
	int rc;
	int i;

	printk(KERN_INFO "%s, line: %d\n", __func__, __LINE__);	
	rc = gpio_request(GPIO_GYRO_INT, "k3g_irq");
	if (rc)
	{
		printk(KERN_ERR "%s: gyro_int  %d request failed\n",__func__,GPIO_GYRO_INT );
		return;
	}

	rc = gpio_request(GPIO_3AXIS_INT, "k3dh_irq");
	if (rc)
	{
		printk(KERN_ERR "%s: 3axis_int  %d request failed\n",__func__,GPIO_3AXIS_INT );
		return;
	}
	
	rc = gpio_request(GPIO_COMPASS_INT, "ami306_irq");
	if (rc)
	{
		printk(KERN_ERR "%s: compass_int  %d request failed\n",__func__,GPIO_COMPASS_INT );
		return;
	} 

	for (i = 0; i < ARRAY_SIZE(sensor_config_power_on); i++)
	{
		rc = gpio_tlmm_config(sensor_config_power_on[i], GPIO_CFG_ENABLE);
		gpio_direction_input(sensor_int_gpio[i]);
		if (rc) 
		{
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=fg%d\n",__func__, sensor_config_power_on[i], rc);
			return;
		}
	}

}

/*                                                */
#ifdef CONFIG_LGE_SENSOR
uint32_t sensor_pwr_mask = 0;
static int sensor_common_power_set(unsigned char on, int sensor)
{
    int ret = 0;
    printk(KERN_INFO "%s pwr_mask(%d), on(%d), sensor(%d)\n", __func__, sensor_pwr_mask, on, sensor);

    if(on)
    {
        if(!sensor_pwr_mask)
        {
			
			
            ret = power_set_for_8058_l11(on);
            if(ret !=0)
                printk(KERN_ERR "%s, power on, pwr_mask=%d, sensor=%d\n", __func__,sensor_pwr_mask, sensor);
        }
        sensor_pwr_mask |= sensor;
    }
    else
    {
        if(sensor_pwr_mask)
        {
            sensor_pwr_mask &= ~sensor;

            if(!sensor_pwr_mask)
            {
                ret = power_set_for_8058_l11(on);
                if(ret !=0)
                    printk(KERN_ERR "%s, power off, pwr_mask=%d, sensor=%d\n", __func__,sensor_pwr_mask, sensor);
            }
        }
    }

    return ret;
}
/*                                              */

static int sensor_power_on(int sensor)
{
    int ret = 0;
    ret = sensor_common_power_set(1, sensor);
    return ret;
}

static int sensor_power_off(int sensor)
{
    int ret = 0;
    ret = sensor_common_power_set(0, sensor);
    return ret;
}
#endif

#ifdef CONFIG_LGE_SENSOR_ACCELEROMETER
static int k3dh_init(void){return 0;}
static void k3dh_exit(void){}
struct k3dh_acc_platform_data accelerometer_pdata = {
	.poll_interval = 10,
	.min_interval = 0,
	.g_range = 0x00,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
	.init = k3dh_init,
	.exit = k3dh_exit,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,	
	.gpio_int1 = -1,//GPIO_3AXIS_INT,
	.gpio_int2 = -1,//0,
};

#endif //                               

#ifdef CONFIG_LGE_SENSOR_GYROSCOPE
static int k3g_init(void){return 0;}
static void k3g_exit(void){}
struct k3g_platform_data gyroscope_pdata = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
	.init = k3g_init,
	.exit = k3g_exit,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

#endif //                           

#ifdef CONFIG_LGE_SENSOR_DCOMPASS
static int ami306_init(void){return 0;}
static void ami306_exit(void){}
static struct ami306_platform_data dcompss_pdata = {
	.init = ami306_init,
	.exit = ami306_exit,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
	.fdata_mDir = 10,
	.fdata_sign_x = -1,
	.fdata_sign_y = 1,
	.fdata_sign_z = -1,
	.fdata_order0 = 0,
	.fdata_order1 = 1,
	.fdata_order2 = 2,
};
#endif //                          
#ifdef CONFIG_LGE_SENSOR_PROXIMITY
static int sensor_proximity_power_set(unsigned char onoff)
{
	power_set_for_8058_l15(onoff);
	return 0;	
}

static struct apds9900_platform_data proximity_pdata = {
	.irq_num= GPIO_PROXIMITY_OUT_INT,
	.power = sensor_proximity_power_set,
	.prox_int_low_threshold = 0,
	.prox_int_high_threshold = 800, //850,//600,
	.als_threshold_hsyteresis = 30,
	.ppcount = 4,
	.B = 1960, //1939,//1951,
	.C = 690, //687,//747,
	.D = 1250, //1293,//1414,
	.alsit = 146880, //92480,
	.ga_value = 2450, //2555,//2298,
	.df_value = 52,
	.atime = 0xCA, //0xDE,
};
#endif //                           
#if defined (CONFIG_LGE_SENSOR_ACCELEROMETER)||defined (CONFIG_LGE_SENSOR_GYROSCOPE)
static struct i2c_board_info msm_i2c_gsbi12_info[] = {
	{
		I2C_BOARD_INFO("k3dh_acc_misc", 0x19),
		.irq =  -1,//MSM_GPIO_TO_INT(GPIO_3AXIS_INT),
		.platform_data = &accelerometer_pdata,
	},
	{
		I2C_BOARD_INFO("k3g", 0x69),
		.irq =  -1,//MSM_GPIO_TO_INT(GPIO_GYRO_INT),
		.platform_data = &gyroscope_pdata,
	},	
};
#endif

#if defined (CONFIG_LGE_SENSOR_DCOMPASS)|| defined (CONFIG_LGE_SENSOR_PROXIMITY) 
#define APDS9900_ADDRESS 0x39

static struct i2c_board_info msm_i2c_gsbi10_info[] = {
	{
		I2C_BOARD_INFO("ami306", 0x0E),
		.irq =  -1,//MSM_GPIO_TO_INT(GPIO_COMPASS_INT),
		.platform_data = &dcompss_pdata,
	},
	{
		I2C_BOARD_INFO("apds9900", APDS9900_ADDRESS),
		.irq =  MSM_GPIO_TO_INT(GPIO_PROXIMITY_OUT_INT),
		.platform_data = &proximity_pdata,
	},	
};
#endif

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

static struct i2c_registry input_sensor_i2c_devices[] __initdata = {
#if defined (CONFIG_LGE_SENSOR_DCOMPASS)|| defined (CONFIG_LGE_SENSOR_PROXIMITY)
    {
        I2C_SURF | I2C_FFA | I2C_FLUID,
	    MSM_GSBI10_QUP_I2C_BUS_ID,
	    msm_i2c_gsbi10_info,
	    ARRAY_SIZE(msm_i2c_gsbi10_info),
    },
#endif

#if defined (CONFIG_LGE_SENSOR_ACCELEROMETER) || defined (CONFIG_LGE_SENSOR_GYROSCOPE)
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI12_QUP_I2C_BUS_ID,
		msm_i2c_gsbi12_info,
		ARRAY_SIZE(msm_i2c_gsbi12_info),
	},
#endif
};

void __init i2c_register_input_sensor_info(void){

	int i;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(input_sensor_i2c_devices); ++i) {
		i2c_register_board_info(input_sensor_i2c_devices[i].bus,
						input_sensor_i2c_devices[i].info,
						input_sensor_i2c_devices[i].len);
	}
}
#endif /*CONFIG_I2C*/


#endif //                 
