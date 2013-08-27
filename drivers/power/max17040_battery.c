/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>

#ifdef CONFIG_BLX 
#include <linux/blx.h> 
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include "../../lge/include/board_lge.h"
#include "../../lge/include/lg_power_common.h"
#endif

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#ifdef CONFIG_LGE_FUEL_GAUGE
#define MAX17040_OCV_MSB	0x0E
#define MAX17040_OCV_LSB	0x0F
#endif
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	95

/*                     */
#define LGE_DEBUG_FINAL

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
#define INSTANT_POWER_ON_AT_LOW_BATTERY
#endif

#ifdef CONFIG_LGE_FUEL_GAUGE
#define TOLERANCE	15
#define LGE_CHECK_SOC	5
static struct i2c_client *max17040_i2c_client;
int check_soc = 0;
int batt_mvolts_drop_cnt = 3;
int batt_mvolts_compare=3500-250;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/*                                                                                                            */
int batt_therm_raw = 0; 
#endif

int pre_soc=100;
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
int pre_volt=4350;
extern int usb_chg_type;

extern bool bq24160_chg_plug_in(void);
#else
extern int is_chg_plugged_in(void);
#endif
#endif

static struct workqueue_struct *local_workqueue;

#ifdef CONFIG_LGE_FUEL_GAUGE
extern int lge_battery_info;
#ifdef CONFIG_BATMAN_VZW_FUEL_GAUGE
#define AUTO_CHARGING_FUEL_GAUGE_UPPER_CALC(x)	(4350-(x*150))
#define AUTO_CHARGING_FUEL_GAUGE_LOWER_CALC(x)	(3500-(x*100))
#endif
#endif

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
extern uint16_t battery_info_get(void);
extern int usb_cable_info;
int max17040_lt_nobattery=0;
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER_BQ24160
int bq24160_get_online(void);
#endif

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
};

static int max17040_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, battery);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = chip->status;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->online;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = chip->vcell;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = chip->soc;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_LGE_FUEL_GAUGE
static int max17040_write_data(struct i2c_client *client, int reg, const u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_data(struct i2c_client *client, int reg, u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#else
static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#endif

#ifndef CONFIG_LGE_FUEL_GAUGE
static void max17040_reset(struct i2c_client *client)
{
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x54);
	max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);
}
#endif


int max17040_get_mvolts(void)
{
	u8 buf[5];
	int vbatt_mv;

#ifdef CONFIG_MACH_LGE_325_BOARD
	int ret;
#endif

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return 4350;
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
	ret = max17040_read_data(max17040_i2c_client, MAX17040_VCELL_MSB, &buf[0], 2);
	if (ret < 0)
		return 800;

	vbatt_mv = (buf[0] << 4) + (buf[1] >> 4);
	vbatt_mv = (vbatt_mv * 125) / 100;
	pr_info("%s: vbatt_mv is %d\n", __func__, vbatt_mv);

	return vbatt_mv;
#endif

	max17040_read_data(max17040_i2c_client, MAX17040_VCELL_MSB, &buf[0], 2);
	vbatt_mv = (buf[0] << 4) + (buf[1] >> 4);
	vbatt_mv = (vbatt_mv * 125) / 100;
#ifdef LGE_DEBUG
	pr_info("%s: vbatt_mv is %d\n", __func__, vbatt_mv);
#endif

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
	if (!bq24160_chg_plug_in()) 
	{
		if (pre_volt < vbatt_mv)
			vbatt_mv = pre_volt;
		else
			pre_volt = vbatt_mv;
	}
	else
	{
		pre_volt = vbatt_mv;
	}
#endif

	return vbatt_mv;
}

#if 0 /*                                        */
extern void arch_reset(char mode, const char *cmd);
#else
extern void msm_restart(char mode, const char *cmd);
#endif
extern acc_cable_type get_ext_cable_type_value(void);
int max17040_get_capacity_percent(void)
{
	u8 buf[5];
	long batt_soc = 0;
	int vbatt_mv = 0;
	int cur_soc;
	int ret = 0;
#ifdef CONFIG_MACH_LGE_325_BOARD
#else
#ifdef LGE_DEBUG_FINAL
	long debug_soc=0;
#endif
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
	int battery_soc;
#endif

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return 70;
#endif

	ret = max17040_read_data(max17040_i2c_client, MAX17040_SOC_MSB, &buf[0], 2);

	if (ret < 0){
		printk(KERN_ERR "%s(), MAX17040_SOC_MSB Register Read Fail\n",__func__);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		/*                                                                                                            */
		pr_err("===========================================================");
		if (batt_therm_raw > 2100){
			if(TA_CABLE_800MA == get_ext_cable_type_value() || MHL_CABLE_500MA == get_ext_cable_type_value()
					|| TA_CABLE_600MA == get_ext_cable_type_value() || TA_CABLE_DTC_800MA == get_ext_cable_type_value()
					|| TA_CABLE_FORGED_500MA == get_ext_cable_type_value()
			  ) {
				pr_err("%s: arch_reset board rev = %d \n",__func__, lge_bd_rev);
				pr_err("===========================================================");				
#if 0 /*                                        */
				arch_reset(0, NULL);
#else
				msm_restart(0, NULL);
#endif	/*                                               */
			}
		}
#endif
		return pre_soc;
	}

#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	if(lge_battery_info == 0)
#endif
	{
#ifdef CONFIG_BATMAN_VZW_FUEL_GAUGE
		//[0] defualt
		batt_soc = ((((buf[0]*256)+buf[1]) * 3906) / MAX17040_BATTERY_FULL) / 10000;
		//[1] 3.3V cutoff
		//batt_soc = (buf[0] + buf[1]/256. + 0.77) * 100 / 95.77;
		//batt_soc = (buf[0] * 256 + buf[1] + 197) * 100 / 24517;
		//[2] 3.2V cutoff
		//batt_soc = (buf[0] + buf[1]/256. + 0.90) * 100 / 95.90;
		//batt_soc = (buf[0] * 256 + buf[1] + 230) * 100 / 24550;
		//printk(KERN_INFO "%s(), LINE:%d tmp batt_soc : %ld (raw : %d.%d)\n",__func__,__LINE__, batt_soc, buf[0], buf[1]);
#else
		batt_soc = ((buf[0]*256)+buf[1])*19531; /* 0.001953125 */
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
#else
#ifdef LGE_DEBUG
		pr_info("%s: batt_soc is %d(%d:%d):%ld\n", __func__, (int)(batt_soc/10000000), buf[0], buf[1], batt_soc);
#endif
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
#else
#ifdef LGE_DEBUG_FINAL
		debug_soc = batt_soc;
#endif
#endif

#ifdef CONFIG_BATMAN_VZW_FUEL_GAUGE		
		if(batt_soc >= 100)
		{
			batt_soc = 100;
		}
		else if(batt_soc < 0)
		{
			batt_soc = 0;
		}
		vbatt_mv = max17040_get_mvolts();
		if((vbatt_mv > AUTO_CHARGING_FUEL_GAUGE_LOWER_CALC(lge_battery_info)) && ((int)batt_soc == 0))
		{
			batt_soc = 1;
		}

		if (!is_chg_plugged_in()) 
		{
			if (pre_soc < batt_soc)
			{
				batt_soc = pre_soc;
			}
			else
			{
				pre_soc = batt_soc;
			}
		}
		else
		{
			pre_soc = batt_soc;
		}

		pr_info("max17040_get_capacity_percent(line:%d): Battery SOC is %d, Voltage:%d\n",__LINE__, (int)batt_soc, vbatt_mv);

		return (int)batt_soc;
#else
#if defined(CONFIG_MACH_LGE_I_BOARD) && !defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) && !defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) //                                                       
	batt_soc = (batt_soc - 13000000)/98700;  //cutoff voltage 3.3V
    // pr_info("[Fuel Gauge] batt_soc_calculated = %d\n", (int)batt_soc);
    if (batt_soc > 10000)
    batt_soc = 100;
    else if (batt_soc <= 0)
    batt_soc = 0;
    else
    batt_soc /= 100;
#else
#if 0
		batt_soc /= 10000000;
		if (batt_soc >= 100) batt_soc = 100;
#else
		batt_soc /= 96000;
		if (batt_soc > 10000) batt_soc = 100;
		else
			batt_soc /= 100;
#endif
#endif
#endif
		if (batt_soc == 0) {
			vbatt_mv = max17040_get_mvolts();
#if defined(CONFIG_MACH_LGE_I_BOARD) && !defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) && !defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) //                                                       
				if (vbatt_mv > 3300) batt_soc = 1;
#else
			if (vbatt_mv > batt_mvolts_compare) batt_soc = 1;
			if (check_soc < LGE_CHECK_SOC) check_soc++;
			else if (batt_mvolts_drop_cnt != 0) {
				check_soc=0;
				if (batt_mvolts_drop_cnt == 3) batt_mvolts_compare += 100;
				else if (batt_mvolts_drop_cnt == 2) batt_mvolts_compare += 50;
				else if (batt_mvolts_drop_cnt == 1) batt_mvolts_compare += 20;
				batt_mvolts_drop_cnt--;
			}
#endif //                        
#ifdef LGE_DEBUG
			pr_info("%s: count:%d check_soc:%d batt_soc:%d compare(%d:%d)\n", __func__, batt_mvolts_drop_cnt, check_soc,
					(int)batt_soc, batt_mvolts_compare, vbatt_mv);
#endif
		}
#ifdef CONFIG_MACH_LGE_325_BOARD
#else
#ifdef LGE_DEBUG_FINAL
		else
			vbatt_mv = max17040_get_mvolts();
#endif	
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
		battery_soc = ((buf[0]*256)+buf[1])*100/(95*256);

		if (battery_soc >= 100) battery_soc = 100;

#ifdef INSTANT_POWER_ON_AT_LOW_BATTERY
		if (!is_chg_plugged_in()) {
			if ( buf[0] < 11){
				battery_soc = ((((buf[0]*256)+buf[1]) - 768))*10/(7*256);
				if (battery_soc < 0)
					battery_soc = 0;
				else if ((battery_soc == 0) && (buf[0] >= 3))
					battery_soc = 1;
			}
		}
		else{
			if ( battery_soc == 0)
				battery_soc = 0;
			else if ( buf[0] < 11){
				battery_soc = ((((buf[0]*256)+buf[1]) - 768))*10/(7*256);
				if (battery_soc <= 0)
					battery_soc = 1;
			}
		}
#endif

		pre_soc = battery_soc;

		printk("%s: b325 batt_soc = %d (buf[0] = %d buf[1] = %d )\n", __func__, battery_soc, buf[0], buf[1]);
		return battery_soc;
#endif

	}
#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	else if(lge_battery_info == 1)
	{
		batt_soc = buf[0];
#ifdef CONFIG_MACH_LGE_325_BOARD
#else
#ifdef LGE_DEBUG_FINAL
		debug_soc = batt_soc;
#endif
#endif

#if 0
		if (batt_soc >= 100) batt_soc = 100;
#else
		//		batt_soc /= 96;
		batt_soc = batt_soc * 100 / 96;
		if (batt_soc > 100) batt_soc = 100;
#endif
#ifdef LGE_DEBUG_FINAL
		vbatt_mv = max17040_get_mvolts();
#endif	
	}
#endif
#ifdef CONFIG_MACH_LGE_325_BOARD
#else	
#ifdef LGE_DEBUG_FINAL
	pr_info("[FuelGuage battinfo =%d] FG:[org]%d->%ld, FG:[transe96per->100per]%d, MV:%d\n", lge_battery_info,(int)(debug_soc/10000000), debug_soc, (int)batt_soc, vbatt_mv);
#endif /*                 */
#endif

	cur_soc = (int)batt_soc;
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)

	if (!bq24160_chg_plug_in()) 
	{
		if (pre_soc < cur_soc)
			cur_soc = pre_soc;
		else
			pre_soc = cur_soc;
	}
	else
	{
		pre_soc = cur_soc;
	}
#else    
	if (!is_chg_plugged_in()
#ifdef CONFIG_LGE_WIRELESS_CHARGER_BQ24160
			&& !bq24160_get_online()
#endif
	   ) {
		if (pre_soc < cur_soc)
			cur_soc = pre_soc;
		else
			pre_soc = cur_soc;
	}
	else
		pre_soc = cur_soc;
#endif	
	return cur_soc;
}

#if defined(CONFIG_MACH_LGE_I_BOARD) 
#if 0 /* move the SBL3 */
static void max17040_init_model(struct i2c_client *client)
{
	u8 org_rcomp_msb, org_rcomp_lsb;
	u8 org_ocv_msb, org_ocv_lsb;
	u8 reg;
	u8 values[32];
	int len;

	/* 1 : Unlock Model Access */
	values[0] = 0x4A; values[1] = 0x57;
	max17040_write_data(client, 0x3E, &values[0], 2);

	/* 2 : Read Original RCOMP and OCV Register */
	len = max17040_read_data(client, MAX17040_RCOMP_MSB, &values[0], 4);
	org_rcomp_msb = values[0]; org_rcomp_lsb = values[1];
	org_ocv_msb = values[2]; org_ocv_lsb = values[3];

	/* 3 : Write OCV Register */
	values[0] = 0xE1; values[1] = 0x60;
	max17040_write_data(client, MAX17040_OCV_MSB, &values[0], 2);

	/* 4 : Write RCOMP Register */
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
	values[0] = 0x60; values[1] = 0x00;
#else
	values[0] = 0xFF; values[1] = 0x00;
#endif
	max17040_write_data(client, MAX17040_RCOMP_MSB, &values[0], 2);

	/* 5 : Write the Model - Write 64bytes model */
	len = 16;
	reg = 0x40;
	values[0]  = 0x9E; values[1]  = 0x00; values[2]  = 0xB5; values[3]  = 0xA0;
	values[4]  = 0xB8; values[5]  = 0x90; values[6]  = 0xBA; values[7]  = 0x60;
	values[8]  = 0xBA; values[9]  = 0xA0; values[10] = 0xBB; values[11] = 0x00;
	values[12] = 0xBB; values[13] = 0x40; values[14] = 0xBB; values[15] = 0x90;
	max17040_write_data(client, reg, &values[0], len);

	reg = 0x50;
	values[0]  = 0xBB; values[1]  = 0xD0; values[2]  = 0xBC; values[3]  = 0x20;
	values[4]  = 0xBE; values[5]  = 0x50; values[6]  = 0xC1; values[7]  = 0x50;
	values[8]  = 0xC6; values[9]  = 0xB0; values[10] = 0xCD; values[11] = 0x50;
	values[12] = 0xD1; values[13] = 0xE0; values[14] = 0xD7; values[15] = 0x60;
	max17040_write_data(client, reg, &values[0], len);

	reg = 0x60;
	values[0]  = 0x00; values[1]  = 0x20; values[2]  = 0x22; values[3]  = 0x00;
	values[4]  = 0x00; values[5]  = 0x20; values[6]  = 0x53; values[7]  = 0x00;
	values[8]  = 0x36; values[9]  = 0x10; values[10] = 0x74; values[11] = 0x50;
	values[12] = 0x5D; values[13] = 0x10; values[14] = 0x60; values[15] = 0x10;
	max17040_write_data(client, reg, &values[0], len);

	reg = 0x70;
	values[0]  = 0x4C; values[1]  = 0xD0; values[2]  = 0x1B; values[3]  = 0xD0;
	values[4]  = 0x28; values[5]  = 0xF0; values[6]  = 0x11; values[7]  = 0xF0;
	values[8]  = 0x11; values[9]  = 0xF0; values[10] = 0x10; values[11] = 0xF0;
	values[12] = 0x0C; values[13] = 0xF0; values[14] = 0x0C; values[15] = 0xF0;
	max17040_write_data(client, reg, &values[0], len);

	/* 6 : Delay at least 150mS */
	msleep(150);

	/* 7 : Write OCV Register */
	values[0] = 0xE1; values[1] = 0x60;
	max17040_write_data(client, MAX17040_OCV_MSB, &values[0], 2);

	/* 8 : Delay between 150mS and 600mS */
	msleep(150);

	/* 9 : Read SOC Register */
	len = max17040_read_data(client, MAX17040_SOC_MSB, &values[0], 2);

	if (values[0] >= 0xE6 && values[0] <= 0xE8)
		pr_info("%s: Model Guage was loaded successful!!!(%02x)\n", __func__, values[0]);
	else
		pr_info("%s: Model Guage was not loaded successful!!!T.T(%02x)\n", __func__, values[0]);

	/* 10 : Restore RCOMP and OCV */
	values[0] = org_rcomp_msb; values[1] = org_rcomp_lsb;
	values[2] = org_ocv_msb; values[3] = org_ocv_lsb;
	max17040_write_data(client, MAX17040_RCOMP_MSB, &values[0], 4);

	/* 11 : Lock Model Access */
	values[0] = 0x00; values[1] = 0x00;
	max17040_write_data(client, 0x3E, &values[0], 2);
}
static int lge_check_battery(void)
{
	int soc=0;
	int batt_mv=0, tmp_soc=0, tmp_soc_p, tmp_soc_m;
	long slope=0, intercept=0;
	int flat;

	batt_mv = max17040_get_mvolts();
	soc = max17040_get_capacity_percent();
	flat = 0;

	if (batt_mv >= 3948) {                          /* 70% ~ 100% */
		slope = 107303;
		intercept = 31940993;
	}
	else if (batt_mv >= 3833 && batt_mv < 3948) {   /* 56% ~ 70% */
		slope = 80791;
		intercept = 33802497;
	}
	else if (batt_mv >= 3748 && batt_mv < 3833) {   /* 37% ~ 56% */
		slope = 45359;
		intercept = 35785878;
	}
	else if (batt_mv >= 3699 && batt_mv < 3748) {   /* 20% ~ 37% : flat area */
		flat = 1;
		slope = 28291;
		intercept = 36421496;
	}
	else if (batt_mv >= 3596 && batt_mv < 3699) {   /* 4% ~ 20% */
		slope = 62203;
		intercept = 35743009;
	}
	else if (batt_mv >= 3400 && batt_mv < 3596) {   /* 0% ~ 4% */
		slope = 515118;
		intercept = 34144853;
	}
	else
		return 1;

	tmp_soc = (int)(batt_mv*10000 - intercept)/slope;

#ifdef LGE_DEBUG
	pr_info("%s: soc is %d, mvolts is %d, tmp_soc is %d\n", __func__, soc, batt_mv, tmp_soc);
#endif

	if (flat) {
		tmp_soc_p = tmp_soc + TOLERANCE + 10;
		tmp_soc_m = tmp_soc - TOLERANCE - 10;
	}
	else {
		tmp_soc_p = tmp_soc + TOLERANCE;
		tmp_soc_m = tmp_soc - TOLERANCE;
	}

	if (soc < tmp_soc_m || soc > tmp_soc_p) {
		pr_info("Quick Start(%d<%d<%d.............................................\n", tmp_soc_m, soc, tmp_soc_p);
		return 1;
	}

	return 0;
}

void max17040_quick_start(void)
{
	u8 buf[5];

	if (lge_check_battery()) {
		buf[0] = 0x40; buf[1] = 0x00;
		max17040_write_data(max17040_i2c_client, MAX17040_MODE_MSB, &buf[0], 2);
		msleep(300);
	}
}
#endif

void max17040_update_rcomp(int temp)
{
#if defined(CONFIG_MACH_LGE_I_BOARD) && !defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) && !defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) //                         
	u8 startingRcomp = 0x60;
	int tempCoHot = -125;	//-12.5
	int tempCoCold = -3425; //-342.5
#else
	u8 startingRcomp = 0x4D;
	int tempCoHot = -55;
	int tempCoCold = -535;
#endif
	int newRcomp = 0;
	u8 buf[5];

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if(max17040_lt_nobattery)
		return;
#endif

#ifdef LGE_DEBUG
	max17040_read_data(max17040_i2c_client, MAX17040_RCOMP_MSB, &buf[2], 2);
#endif
#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	if(lge_battery_info == 0)
#endif
	{
#if defined(CONFIG_MACH_LGE_I_BOARD) && !defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) && !defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) //                         
                if (temp > 20)
                newRcomp = startingRcomp + (int)((temp - 20)*tempCoHot/1000);
                else if (temp < 20)
                newRcomp = startingRcomp + (int)((temp - 20)*tempCoCold/1000);
                else
                newRcomp = startingRcomp;
#else
		if (temp > 20)
			newRcomp = startingRcomp + (int)((temp - 20)*tempCoHot/100);
		else if (temp < 20)
			newRcomp = startingRcomp + (int)((temp - 20)*tempCoCold/100);
		else
			newRcomp = startingRcomp;
#endif
	}
#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	else if(lge_battery_info == 1)
	{
		startingRcomp = 0xC0;
		tempCoHot = 11;
		tempCoCold = 5;

		if (temp > 20)
			newRcomp = startingRcomp - (int)((temp - 20)*tempCoHot/10);
		else if (temp < 20)
			newRcomp = startingRcomp + (int)((20 - temp)*tempCoCold);
		else
			newRcomp = startingRcomp;
	}
#endif

	if (newRcomp > 0xFF)
		buf[0] = 0xFF;
	else if (newRcomp < 0)
		buf[0] = 0;
	else
		buf[0] = newRcomp;

	if (buf[0] != buf[2] && buf[0] != startingRcomp) {
		buf[1] = 0x00;
		max17040_write_data(max17040_i2c_client, MAX17040_RCOMP_MSB, &buf[0], 2);
#ifdef LGE_DEBUG
		pr_info("RCOMP: new rcomp is %02X(%02X)\n", buf[0], buf[2]);
#endif
	}
}
EXPORT_SYMBOL(max17040_update_rcomp);
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
void max17040_write_rcomp(int rcomp)
{
	u8 buf[2];

	int temp;

	max17040_read_data(max17040_i2c_client, MAX17040_RCOMP_MSB, &buf[0], 2);

	temp = (buf[0]<<8)+buf[1];

	printk("[325_BAT] %s()  Read Rcomp = %X \n", __func__, temp);

	buf[0]= (u8)((rcomp >> 8) & 0xFF);
	buf[1]= (u8)(rcomp);

	printk("[325_BAT] %s()	Write Rcomp: buf[0]= %X  buf[1]= %X \n", __func__, buf[0],buf[1]);

	max17040_write_data(max17040_i2c_client, MAX17040_RCOMP_MSB, &buf[0], 2);

}
#endif


#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
u8 at_cmd_buf[5] = {0xff,0xff,0xff,0xff,0xff};
void max17040_set_battery_atcmd(int flag, int value)
{
	u8 buf[5];

	if (flag == 0) {
		buf[0] = 0x40; buf[1] = 0x00;
		max17040_write_data(max17040_i2c_client, MAX17040_MODE_MSB, &buf[0], 2);
	}
	else if (flag == 1) {
		at_cmd_buf[0] = 1;
		at_cmd_buf[1] = value;
	}
	else if (flag == 2) {
		at_cmd_buf[0] = 0;
	}
}
EXPORT_SYMBOL(max17040_set_battery_atcmd);
#endif


int max17040_get_battery_mvolts(void)
{
#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return 4350;
#endif /*                                   */

	return max17040_get_mvolts();
}
EXPORT_SYMBOL(max17040_get_battery_mvolts);

int max17040_get_battery_capacity_percent(void)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER  
	if (at_cmd_buf[0] == 1) 
		return at_cmd_buf[1];
#endif
#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return 70;
#endif /*                                   */

	return max17040_get_capacity_percent();
}
EXPORT_SYMBOL(max17040_get_battery_capacity_percent);


static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
#ifdef CONFIG_LGE_FUEL_GAUGE
	u8 buf[5];

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return;
#endif /*                                   */

	max17040_read_data(client, MAX17040_VCELL_MSB, &buf[0], 2);

	chip->vcell = (buf[0] << 4) + (buf[1] >> 4);
#else
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	chip->vcell = (msb << 4) + (lsb >> 4);
#endif
}

static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
#ifdef CONFIG_LGE_FUEL_GAUGE
	u8 buf[5];
	long soc = 0;	/*                                        */

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (max17040_lt_nobattery)
		return;
#endif /*                                   */

	max17040_read_data(client, MAX17040_SOC_MSB, &buf[0], 2);

#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	if(lge_battery_info == 0)
#endif
	{
		soc = ((buf[0]*256) + buf[1])*19531; /* 0.001953125 */
		soc /= 10000000;
	}
#if defined(CONFIG_MACH_LGE_325_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
	else if(lge_battery_info == 1)
	{
		soc = buf[0];
	}
#endif
	chip->soc = (int)soc;
	if (chip->soc > 100) chip->soc = 100;
#else
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);

	chip->soc = msb;
#endif
}

static void max17040_get_version(struct i2c_client *client)
{
#ifdef CONFIG_LGE_FUEL_GAUGE
	u8 buf[5];
	int ret;

	ret = max17040_read_data(client, MAX17040_VER_MSB, &buf[0], 2);

#ifdef CONFIG_LGE_PM_FACTORY_FUEL_GAUGE
	if (ret < 0) {
		if ((0 == battery_info_get())&&((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11)))
			max17040_lt_nobattery = 1;
	}
#endif /*                                   */

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", buf[0], buf[1]);
#else
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
#endif
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (!chip->pdata->charger_online || !chip->pdata->charger_enable) {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) {
		if (chip->pdata->charger_enable())
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#ifdef CONFIG_BLX 
        if (chip->soc >= get_charginglimit()) 
#else
	if (chip->soc > MAX17040_BATTERY_FULL)
#endif
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip;

	chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);

#ifdef CONFIG_MACH_LGE_325_BOARD
	//Do not use max17040 work que in run time
#else
	queue_delayed_work(local_workqueue, &chip->work, MAX17040_DELAY);
#endif
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int __devinit max17040_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
#ifdef CONFIG_MACH_LGE_I_BOARD
#else
	int ret = 0;
#endif

#ifdef CONFIG_MACH_LGE_325_BOARD
	int rcomp;
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

#ifdef CONFIG_LGE_FUEL_GAUGE
	max17040_i2c_client = client;
#endif

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

#ifdef CONFIG_LGE_FUEL_GAUGE
	chip->battery.name		= "battery_fuel_gauge";
#else
	chip->battery.name		= "battery";
#endif
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);

#ifdef CONFIG_MACH_LGE_325_BOARD
	//Do nothing in 325 Model
	ret = 0;

#else
#ifdef CONFIG_MACH_LGE_I_BOARD
#else
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
#endif
#endif

#ifdef CONFIG_LGE_FUEL_GAUGE
	//	max17040_quick_start(); /* move the SBL3 */
#else
	max17040_reset(client);
#endif
	max17040_get_version(client);

#ifdef CONFIG_MACH_LGE_325_BOARD
	rcomp = 0xd800;
	max17040_write_rcomp(rcomp);
#endif


#ifdef CONFIG_LGE_FUEL_GAUGE
	// max17040_init_model(client);
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
	//-	schedule_delayed_work(&chip->work, MAX17040_DELAY);
	queue_delayed_work(local_workqueue, &chip->work, MAX17040_DELAY);

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

#if 0 /*                                                          */
	power_supply_unregister(&chip->battery);
#endif
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	queue_delayed_work(local_workqueue, &chip->work, MAX17040_DELAY);
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	local_workqueue = create_workqueue("max17040_fuel_gauge");

	if (!local_workqueue)
		return -ENOMEM;

	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	if (local_workqueue)
		destroy_workqueue(local_workqueue);

	local_workqueue = NULL;
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
