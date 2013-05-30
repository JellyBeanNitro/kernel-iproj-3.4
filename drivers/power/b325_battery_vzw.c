/*
 * 325 Battery Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/bq24160-charger.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>

#if defined(CONFIG_MACH_LGE_325_BOARD_VZW)
#include <linux/gpio.h>
#else
#include <mach/gpio.h>
#endif
#include <linux/msm-charger.h>
#include <linux/msm_adc.h>

#include <linux/delay.h>
#include <linux/time.h>


#include "../../lge/include/board_lge.h"

#define	VZW_POWER_CAMERA_ON_CHECK
#define	VZW_POWER_ACC_ADC_CHECK
#define	VZW_CEC_SPEC

#ifdef	VZW_POWER_CAMERA_ON_CHECK
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
int is_camera(void);
static struct regulator *regulator_2v8;
#endif

#define	VZW_POWER_TEMP_PROP_CHECK

#ifdef	VZW_POWER_TEMP_PROP_CHECK
static int iHighTempOff;
#endif

int	charger_reg_backup[8] = {0x08, 0x01, 0x2C, 0x90, 0x00, 0x00, 0x12, 0x00};

#ifdef	VZW_POWER_ACC_ADC_CHECK
int acc_adc_value = 0;
extern int get_usb_cable_adc(void);
int cable_name = 0;
int iChargingStatus = 0;
#endif

#ifdef	CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
#define	VZW_POWER_SOC_CONTROL
#endif

//#define BATTERY_INFO_SHOW
//#define BATTERY_FUNCTION_DEBUG
//#define CHARGER_INFO_SHOW

//Default disable the charger interrupt
#define CHARGER_INTERRUTP_ENABLE
 
//LOCAL WORK for 325 Battery instead of mod timer
#define LOCAL_WORK_FOR_325_BATTERY

#ifdef BATTERY_INFO_SHOW
#define d_bat(fmt, ...) printk(KERN_INFO fmt, __VA_ARGS__)
#else
#define d_bat(fmt, ...) do {} while (0)
#endif

#ifdef BATTERY_FUNCTION_DEBUG
#define d_func(fmt, ...) printk(KERN_INFO fmt, __VA_ARGS__)
#else
#define d_func(fmt, ...) do {} while (0)
#endif

#ifdef CHARGER_INFO_SHOW
#define c_func(fmt, ...) printk(KERN_INFO fmt, __VA_ARGS__)
#else
#define c_func(fmt, ...) do {} while (0)
#endif



#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define BATTERY_POLLING_INTERVAL      (60 * 1000)   // 60s
#define BATTERY_POLLING_INTERVAL_LOW  (10 * 1000)   // 10s
#define BATTERY_POLLING_IMMEDIATELY   (30)         // 30ms
//#define BATTERY_DATA_UNKNOWN          0x7FFFFFFF

//#define BATTERY_CUSTOM_MODEL //Max17043 Custom Model Setting

#ifdef	VZW_CEC_SPEC
static char chargerLogo[8];
static int chargerDone=0;
#endif

#define I2C_MAX_ERR_COUNT  5
static int switching_charger_i2c_error = 0;

#ifdef LOCAL_WORK_FOR_325_BATTERY
static struct workqueue_struct *local_workqueue;
#endif

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
typedef	enum TEMP_SCENARIO_T	{
	CHAR_TEMP_SCENARIO_SUCCESS = 0,
	CHAR_TEMP_SCENARIO_OVER55,
	CHAR_TEMP_SCENARIO_45TO55_OVER4000,
	CHAR_TEMP_SCENARIO_45TO55_UNDER4000,
	CHAR_TEMP_SCENARIO_UNDERM10,
	CHAR_TEMP_SCENARIO_OVER_CHARGING,
	CHAR_TEMP_SCENARIO_FAIL
}TEMP_SCENARIO;
#endif

/** enum **/
typedef enum {
    Charger_Type_Battery = 0,
    Charger_Type_USB,
    Charger_Type_AC,
    Charger_Type_UNKNOWN,
} Charger_Type;

typedef enum {
    Charge_Control_Charging_Disable = 0,
    Charge_Control_Charging_Enable,
    //Charge_Control_Num,
    //Charge_Control_Force32 = 0x7FFFFFFF
} Charge_Control;

enum {
	CHARGER_UNDONE = 0,
	CHARGER_DONE_100,
	CHARGER_DONE_99,
};

static enum power_supply_property b325_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
	POWER_SUPPLY_PROP_BATTERY_TEMP_ADC,
#ifdef	VZW_CEC_SPEC
	POWER_SUPPLY_PROP_BATTERY_CHARGER_DONE,
#endif
};

static enum power_supply_property b325_power_properties[] = {
    POWER_SUPPLY_PROP_ONLINE,
#ifdef CONFIG_TEMP_VZW_CONFIG
	POWER_SUPPLY_PROP_CABLE_INFO,
#endif
#ifdef	VZW_POWER_ACC_ADC_CHECK
	POWER_SUPPLY_PROP_ACC_ADC,
	POWER_SUPPLY_PROP_CABLE_NAME,
	POWER_SUPPLY_PROP_CHARGING_STATUS,
#endif
};

typedef struct b325_battery_dev
{
	struct  i2c_client  *client;
	struct i2c_driver *driver;
	struct bq24160_platform_data *pdata;

	#ifdef CHARGER_INTERRUTP_ENABLE
	struct  work_struct  update_charger_info_work;
	#endif

	int batteryLifePercent;
	int batteryVoltage;
	int  batteryTemperature;
	int  batteryTemperature_adc;
#ifdef	VZW_CEC_SPEC
	int	batteryChargerDone;
#endif
	bool batteryPresent;
	bool batteryValid;
	bool chg_termination_flag;
	struct  timer_list  battery_poll_timer;
	uint		batt_status_poll_period;
	uint		batt_status_poll_period_normal;

	struct device *dev;

	struct mutex status_lock;
	spinlock_t queue_lock;

} b325_battery_dev;


typedef struct battery_data {
	int		battery_voltage;		/* battery voltage (mV) */
	int		battery_temp;	/* temperature (degrees C) */
	int		battery_soc;	/* battery SOC (%) */

	bool		battery_present;
	bool		battery_valid;
}battery_data;

struct b325_charger_mux {

	struct wake_lock chg_wake_lock;
	bool wake_lock_flag;

	#ifdef LOCAL_WORK_FOR_325_BATTERY
	struct delayed_work work;
	#endif

};

static struct b325_charger_mux b325_chg;


/** Local functions **/
static int b325_power_get_property(struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val);
static int b325_battery_get_property(struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val);
void update_battery_info(void);
int is_chg_plugged_in(void);
int b325_get_battery_temperature(void);

static int b325_charger_control(bool charging_on_off);
bool is_bat_connected_in(void);
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
static TEMP_SCENARIO temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
void temp_scenario_func(int iTemp);
#endif

/** Extern functions **/
extern int max17047_get_battery_capacity_percent(void);
extern int max17047_get_battery_mvolts(void);

extern int max17040_get_battery_capacity_percent(void);
extern int max17040_get_battery_mvolts(void);
extern void max17040_set_battery_atcmd(int flag, int value);


extern int pm8058_batt_alarm_config(void);

extern bool pm8058_pwrkey_led_blink_set(uint led_set);

extern void machine_restart(char *cmd);


/** Local variables **/
static b325_battery_dev *battery_dev;
struct mutex status_lock;
static bool charging_state;
static bool at_cmd_force_control = FALSE;
static bool at_cmd_force_charge_at56k = FALSE;
static bool power_supply_registered = FALSE;
static bool temp_control = FALSE;
static bool battery_check = TRUE;
static bool full_charge_error_flag = FALSE;
u8 charger_state_cnt = 0;
u8 pmic_die_temp_range = 0;
u16 battery_peak_voltage = 0;
bool conv_first_request = TRUE;
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
int g_iTempValue = 0;
int g_iTempScenario = 0;
#define	TEMPERATURE_BUF_MAX		5
#ifdef	VZW_POWER_SOC_CONTROL
static int g_iBatteryControl;
static int g_iBatteryValue;
static int g_iSocControl;
static int g_iSocValue;
#endif
#endif
/** Extern variables **/
extern uint usb_base_reg;
extern int usb_chg_type_for_FG;

extern int usb_cable_info;

extern int lge_bd_rev;

extern u8 pwrkey_led_status;

#if defined(CONFIG_MACH_LGE_325_BOARD_VZW)
#include "../../lge/include/lg_power_common.h"
extern acc_cable_type get_ext_cable_type_value(void);
#endif

int isChargingDone = CHARGER_UNDONE;

int batt_alarm_state=0;
EXPORT_SYMBOL(batt_alarm_state);


// /sys/class/power_supply/battery/device
static struct power_supply b325_supplies[] = {
    {
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = b325_battery_properties,
        .num_properties = ARRAY_SIZE(b325_battery_properties),
        .get_property = b325_battery_get_property,
    },
    {
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        //.supplied_to = supply_list,
        //.num_supplicants = ARRAY_SIZE(supply_list),
        .properties = b325_power_properties,
        .num_properties = ARRAY_SIZE(b325_power_properties),
        .get_property = b325_power_get_property,
    },
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        //.supplied_to = supply_list,
        //.num_supplicants = ARRAY_SIZE(supply_list),
        .properties = b325_power_properties,
        .num_properties = ARRAY_SIZE(b325_power_properties),
        .get_property = b325_power_get_property,
    },
};

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
void temp_scenario_func(int iTemp)
{
	if(!is_bat_connected_in())
	{
		temp_scenario_state = CHAR_TEMP_SCENARIO_FAIL;
	}
	else
	{
		switch(temp_scenario_state)
		{
		case CHAR_TEMP_SCENARIO_OVER55:
			if(iTemp <= 43)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
			}
			break;

		case CHAR_TEMP_SCENARIO_45TO55_OVER4000:
			if(iTemp >= 55)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_OVER55;
			}
			else if(iTemp <= 43)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
			}
			break;

		case CHAR_TEMP_SCENARIO_45TO55_UNDER4000:
			if(iTemp >= 55)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_OVER55;
			}
			else if(iTemp >= 45)
			{
				if(battery_dev->batteryVoltage > 4000)
				{
					temp_scenario_state = CHAR_TEMP_SCENARIO_45TO55_OVER4000;
				}
			}
			else if(iTemp <= 43)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
			}
			break;

		case CHAR_TEMP_SCENARIO_UNDERM10:
			if(iTemp >= -8)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
			}
			break;

		case CHAR_TEMP_SCENARIO_SUCCESS:
		case CHAR_TEMP_SCENARIO_FAIL:
		default:
			if(iTemp >= 55)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_OVER55;
			}
			else if(iTemp >= 45)
			{
				if(battery_dev->batteryVoltage > 4000)
				{
					temp_scenario_state = CHAR_TEMP_SCENARIO_45TO55_OVER4000;
				}
				else
				{	//550mA
					temp_scenario_state = CHAR_TEMP_SCENARIO_45TO55_UNDER4000;
				}
			}
			else if(iTemp <= -10)
			{
				temp_scenario_state = CHAR_TEMP_SCENARIO_UNDERM10;
			}
			break;
		}
	}
}
#endif

#ifdef CHARGER_INTERRUTP_ENABLE
static void charger_irq_init(int gpio_num)
{
	int rc;

	rc = gpio_request(gpio_num, "charger_irq_gpio");
	if(rc < 0){
		printk("[325_BAT]Can't get charger irq gpio\n");
		return ;
	}
	gpio_direction_input(gpio_num);
	gpio_set_value(gpio_num, 1);
}
#endif


static int charger_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	charger_reg_backup[reg] = value;
	//printk("[spiderman] %s : 0x%x[0x%x]\n", __func__, reg, value);

	//msleep(2);

	if (ret < 0)
		printk("[325_BAT]%s: err %d\n", __func__, ret);

	return ret;
}

static int charger_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

//	msleep(2);

//	printk("[charger] %s : 0x%x[0x%x]\n", __func__, reg, ret);
	if (ret < 0)
	{
		printk("[325_BAT]%s: err %d count=%d\n", __func__, ret, switching_charger_i2c_error);

		switching_charger_i2c_error++;

		if(switching_charger_i2c_error > I2C_MAX_ERR_COUNT)
		{
			dev_err(&client->dev, "[325_BAT][SEND CHG REMOVE EVT]%s: err %d count=%d\n", __func__, ret, switching_charger_i2c_error);

			//msm_charger_notify_event(&g_max8971_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
			switching_charger_i2c_error = 0;

			disable_irq(client->irq);
		}
	}

	return ret;
}


void  charger_fault_check(struct i2c_client *client)
{

	int i = 0;
	int ret=0;

	ret = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
	printk("[325_BAT] BQ24160_REG_STAT_CTRL = %X \n", ret);

	ret = ret & BQ24160_FAULT_MASK;

	if ((ret > 0) && (charger_state_cnt == 0))
	{
		printk("[325_BAT] BQ24160_REG_FAULT STATE = %X \n", ret);
		for ( i =1; i <8 ; i++)
		{
			ret = charger_read_reg(client, i);
			printk("[325_BAT] BQ24160_REG address = 0x%d	value  = %X \n", i, ret);
		}
		charger_state_cnt++;
	}
	else if (ret > 0)
	{
		printk("[325_BAT] BQ24160_REG_FAULT STATE = %X \n", ret);
		if(charger_state_cnt == 5)
			charger_state_cnt =0;
		else
			charger_state_cnt++;
	}
	else if (( ret == 0) && (charger_state_cnt == 0))
	{
		ret = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);
		printk("[325_BAT] BQ24160_REG_BATTNPS_STAT = %X \n", ret);
		charger_state_cnt++;
	}
	else
	{
		if(charger_state_cnt == 10)
			charger_state_cnt =0;
		else
			charger_state_cnt++;
	}
}

#ifdef CHARGER_INTERRUTP_ENABLE
static irqreturn_t charger_handler(int irq, void *data)
{
	//struct b325_battery_dev *chip = (struct b325_battery_dev *)data;

//	c_func("[325_BAT] %s()\n", __func__);

	//mdelay(300);

	schedule_work(&battery_dev->update_charger_info_work);

	return IRQ_HANDLED;
}
#endif

bool b325_full_charge_error_handler(void)
{
#if 0
	u8 reg_val;

	reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
	if ( reg_val < 0)
	{
		printk("charger_read_reg error, skip b325_charger_data one time. \n");
		return 1;
	}
	reg_val = reg_val | 0x2; //Disable CE.
	reg_val = reg_val & 0x7F;
	charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);

	printk("[325_BAT] Full charge error occurred. Disable CE. \n");

	charging_state = FALSE;

	full_charge_error_flag = TRUE;
#endif
	msleep(100);

	return 0;

}

bool b325_charger_temp_control(int bat_temp)
{
	u8 reg_val;

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
	if(battery_check == FALSE)
	{
		return TRUE;
	}

#ifdef	VZW_POWER_TEMP_PROP_CHECK
	if(iHighTempOff == 1)
		temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
	else
		temp_scenario_func(bat_temp);
#else
	temp_scenario_func(bat_temp);
#endif

	switch(temp_scenario_state)
	{
	case CHAR_TEMP_SCENARIO_SUCCESS:
	case CHAR_TEMP_SCENARIO_45TO55_UNDER4000: //550mA
		if(charging_state == FALSE)
		{
			if(isChargingDone == CHARGER_DONE_100)	{	//soc 100%
				if((battery_dev->batteryVoltage < 4130) || (battery_dev->batteryLifePercent < 100))	{
					//Enable CE.
					reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
					reg_val |= 0x4;
					reg_val &= 0x7D;
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val); //Enable CE.
					charging_state = TRUE;
					isChargingDone = CHARGER_UNDONE;
#ifdef	VZW_POWER_ACC_ADC_CHECK
					iChargingStatus = 1;
#endif
#ifdef	VZW_CEC_SPEC
					chargerDone = 0;
#endif
				}
				else	{
#ifdef	VZW_POWER_ACC_ADC_CHECK
					iChargingStatus = 0;
#endif
#ifdef	VZW_CEC_SPEC
					chargerDone = 1;
#endif
				}
			}
			else {
				if(isChargingDone == CHARGER_DONE_99)	{	//soc 0~99%
					isChargingDone = CHARGER_UNDONE;
				}
#ifdef	VZW_POWER_ACC_ADC_CHECK
				iChargingStatus = 1;
#endif

				//Enable CE.
				reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				reg_val |= 0x4;
				reg_val &= 0x7D;
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val); //Enable CE.
				charging_state = TRUE;
#ifdef	VZW_CEC_SPEC
				chargerDone = 0;
#endif
			}
			printk(KERN_INFO "[325_BAT] Temp=%d, vbatt=%d, state=%d, DoneFlag=%d Normal temp continue Charging\n", bat_temp, battery_dev->batteryVoltage, temp_scenario_state, isChargingDone);
		}
		return 1;

	case CHAR_TEMP_SCENARIO_OVER55:
	case CHAR_TEMP_SCENARIO_45TO55_OVER4000:
	case CHAR_TEMP_SCENARIO_UNDERM10: // stop charging
		reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		reg_val |= 0x2;
		reg_val &= 0x7F;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val); //Disable CE.
		charging_state = FALSE;
#ifdef	VZW_POWER_ACC_ADC_CHECK
		iChargingStatus = 0;
#endif
		printk(KERN_INFO "[325_BAT] Temp=%d, vbatt=%d, state=%d, DoneFlag=%d  OUT OF NORMAL RANGE  Stop Charging \n", bat_temp, battery_dev->batteryVoltage, temp_scenario_state, isChargingDone);
		return 0;

	case CHAR_TEMP_SCENARIO_FAIL:
	default:
		return 1;
	}

#else
	if ( (bat_temp > 45) || (bat_temp < 0) )
	{
		charging_state = FALSE;

		reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		if ( reg_val < 0)
			{
				printk("charger_read_reg error, skip temp control one time. \n");
				return 0;
			}
		c_func("[325_BAT]%s BQ24160_REG_CTRL  = %X \n", __func__, reg_val);
		reg_val = reg_val | 0x6;
		reg_val = reg_val & 0x7F;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);

		printk(KERN_INFO "[325_BAT] Battery Temp = %d OUT OF NORMAL RANGE  Stop Charging \n", bat_temp);

		return 0;

	}
	else
	{
		//DCCT ( Dynamic Charging Control by Temperature)
		if ( pmic_die_temp_range == 3)
		{
			if ( charging_state == TRUE)
			{
				reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				if ( reg_val < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 0;
				}
				reg_val = reg_val | 0x2; //Disable CE.
				reg_val = reg_val & 0x7F;
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);
				printk("PMIC 8058 die temperature is high. Stop charging to make system temp low. \n");
				charging_state = FALSE;
				return 0;
			}
			else if (  charging_state == FALSE)
				{
				//printk("PMIC 8058 die temperature is high. Still stop charging. \n");
					return 0;
				}
		}
		else if ( pmic_die_temp_range == 2 )
		{
			if ( charging_state == FALSE)
			{
				//printk("PMIC 8058 die temperature is in hysteresis range. Still stop charging. \n");
				return 0;
			}

		}
		else if ( pmic_die_temp_range == 1 )
		{
			if (charging_state == FALSE)
			{
				reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				if ( reg_val < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 0;
				}
				reg_val = reg_val & 0x7D;//Enable CE.
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);
				charging_state = TRUE;
					printk("PMIC 8058 die temperature is in normal range. Start charging again. \n");
				return 1;
			}
			else if (  charging_state == TRUE)
			{
				//printk("PMIC 8058 die temperature  is in normal range. Keep charging. \n");
				return 1;
			}
		}
		else
		{
			if (charging_state == FALSE)
			{
				reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				if ( reg_val < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 0;
				}
				reg_val = reg_val & 0x7D;//Enable CE.
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);
				charging_state = TRUE;
				return 1;
			}
			else if (  charging_state == TRUE)
			{
				//printk("PMIC 8058 die temperature  is in normal range. Keep charging. \n");
				return 1;
			}
		}

	}
#endif
	return 1;
}

extern int32_t pm8058_xoadc_clear_recentQ(void);
static int batt_read_adc(int channel, int *mv_reading)
{
	int ret;
	void *h;
	struct adc_chan_result adc_chan_result;
	struct completion  conv_complete_evt;
	int wait_ret;

	if (conv_first_request == TRUE)
	{
		printk("[bat_info]%s: msm_adc is not calibrated yet. \n", __func__);
		return 27;
	}

	//printk("[bat_info]%s: called for %d\n", __func__, channel);
	ret = adc_channel_open(channel, &h);
	if (ret) {
		printk("[bat_info]%s: couldnt open channel %d ret=%d\n", __func__, channel, ret);
		return 27;
	}
	init_completion(&conv_complete_evt);
	ret = adc_channel_request_conv(h, &conv_complete_evt);
	if (ret) {
		printk("[bat_info]%s: couldnt request conv channel %d ret=%d\n", __func__, channel, ret);
		goto sanity_out;
	}

	wait_ret = wait_for_completion_timeout(&conv_complete_evt, msecs_to_jiffies(2000));
	if(wait_ret <= 0)
	{
		printk("[bat_info] %s: failed to adc wait for completion!===\n",__func__);
		goto sanity_out;
	}

	ret = adc_channel_read_result(h, &adc_chan_result);
	if (ret) {
		printk("[bat_info]%s: couldnt read result channel %d ret=%d\n", __func__, channel, ret);
		goto sanity_out;
	}
	ret = adc_channel_close(h);
	if (ret) {
		printk("[bat_info]%s: couldnt close channel %d ret=%d\n", __func__, channel, ret);
	}
	if (mv_reading)
		*mv_reading = adc_chan_result.measurement;

	return adc_chan_result.physical;

sanity_out:
	pm8058_xoadc_clear_recentQ();
	ret = adc_channel_close(h);
	if (ret) {
		printk("[bat_info] %s: out : couldnt close channel %d ret=%d\n", __func__, channel, ret);
	}

	printk("[bat_info] ============== completion timeout error!!! ===============\n");
	return 27;
}

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
int temperature_sort(int *temp)	
{
	int iVal=0, i=0, j=0, iSum=0;
	int sort[TEMPERATURE_BUF_MAX];

	for(i=0; i<TEMPERATURE_BUF_MAX; i++)	{
		sort[i] = temp[i];
	}

	for(i=0; i<TEMPERATURE_BUF_MAX-1; i++)	{
		for(j=1; j<TEMPERATURE_BUF_MAX-i; j++)	{
			if(sort[j] > sort[j-1])	{
				iVal = sort[j];
				sort[j] = sort[j-1];
				sort[j-1] = iVal;	
			}
		}
	}

	for(i=1; i<TEMPERATURE_BUF_MAX-1; i++)
		iSum += sort[i];

	iSum = iSum/(TEMPERATURE_BUF_MAX-2);

	return iSum;
}
#endif

int b325_get_battery_temperature(void)
{
	int battery_temp;
	int battery_temp_ADC=0;

	if (power_supply_registered == FALSE)
	{
		battery_temp = 27;
		battery_temp_ADC=0;
		//printk("[325_BAT] Battery driver is not initialized yet. Temporary Temp = 25 deg C\n");
	}
	else
	{
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
		int i=0;
		int batt_temp[TEMPERATURE_BUF_MAX];	
		for(i=0; i<TEMPERATURE_BUF_MAX; i++)	{
			batt_temp[i] = batt_read_adc(CHANNEL_ADC_BATT_THERM, &battery_temp_ADC);
			msleep(5);
		}
	
		battery_temp = temperature_sort(batt_temp);
#else
		battery_temp= batt_read_adc(CHANNEL_ADC_BATT_THERM, &battery_temp_ADC);
#endif
	}

	d_func("[325_BAT] %s() : Temp = %d ADC voltage = %d \n", __func__, battery_temp, battery_temp_ADC);

	battery_dev->batteryTemperature_adc = battery_temp_ADC;

	//Battery temp. range is from -40 degree to 99 degree.
	if ( battery_temp < -40 )
		battery_temp = -40; // -40 degree means no connection between NTC and Temp. ADC. (No battery)
	else if ( battery_temp > 99)
		battery_temp = 99;

    return battery_temp;
}

static bool b325_get_battery_present(void)
{
	bool battery_present;

	d_func("[325_BAT] %s()\n", __func__);

	battery_present = TRUE; //Temporary battery present = TRUE.

    return battery_present;
}

static bool b325_get_battery_valid(void)
{
	bool battery_valid;

	d_func("[325_BAT] %s()\n", __func__);

	battery_valid = TRUE; //Temporary battery present = TRUE.

    return battery_valid;
}

static int b325_battery_get_data(battery_data *battery)
{
	int battery_temp_voltage_value = 0;
	//int battery_temp_soc_value = 0;
	//int battery_soc_gap = 0;

	d_func("[325_BAT] %s()\n", __func__);

	if(lge_bd_rev > LGE_REV_C)
	{
#ifdef CONFIG_BATTERY_MAX17040
		battery_temp_voltage_value = max17040_get_battery_mvolts();
#else
		battery_temp_voltage_value = max17047_get_battery_mvolts();
#endif

		if ( battery_temp_voltage_value < 1500 )
			{
				msleep(300); //Before read register, wait 300ms.

#ifdef CONFIG_BATTERY_MAX17040
				battery_temp_voltage_value = max17040_get_battery_mvolts(); //For the I2C error, read once again
#else
				battery_temp_voltage_value = max17047_get_battery_mvolts(); //For the I2C error, read once again
#endif

				if (battery_temp_voltage_value < 1500 )
					{
						battery->battery_voltage = battery_dev->batteryVoltage;
						battery->battery_soc = battery_dev->batteryLifePercent;
						printk(KERN_INFO " [325_BAT] max17040 read error occurred!! battery_temp_value = %d mV\n", battery_temp_voltage_value);
					}
				else
					{
						msleep(200);

#ifdef CONFIG_BATTERY_MAX17040
#ifdef	VZW_POWER_SOC_CONTROL
						battery->battery_voltage = max17040_get_battery_mvolts();
						msleep(200);
						if(g_iSocControl == 1)
							battery->battery_soc = g_iSocValue;
						else
							battery->battery_soc = max17040_get_battery_capacity_percent();
#else
						battery->battery_voltage = max17040_get_battery_mvolts();
						msleep(200);
						battery->battery_soc = max17040_get_battery_capacity_percent();
#endif
						printk(KERN_INFO " [325_BAT] max17040 read error occurred!! battery->batteryVoltage = %d mV\n", battery->battery_voltage);

#else
						battery->battery_voltage = max17047_get_battery_mvolts();
						msleep(200);
						battery->battery_soc = max17047_get_battery_capacity_percent();
						printk(KERN_INFO " [325_BAT] max17040 read error occurred!! battery->batteryVoltage = %d mV\n", battery->battery_voltage);
#endif
					}

			}
		else
			{
				battery->battery_voltage = battery_temp_voltage_value;
#ifdef CONFIG_BATTERY_MAX17040
#ifdef	VZW_POWER_SOC_CONTROL
				if(g_iSocControl == 1)
					battery->battery_soc = g_iSocValue;
				else
					battery->battery_soc = max17040_get_battery_capacity_percent();
#else
				battery->battery_soc = max17040_get_battery_capacity_percent();
#endif
#else
				battery->battery_soc = max17047_get_battery_capacity_percent();
#endif
			}

		}
	else //Under Rev.D
		{
			battery->battery_voltage =  max17040_get_battery_mvolts();	
#ifdef	VZW_POWER_SOC_CONTROL
			if(g_iSocControl == 1)
				battery->battery_soc = g_iSocValue;
			else
				battery->battery_soc = max17040_get_battery_capacity_percent();
#else
			battery->battery_soc = max17040_get_battery_capacity_percent();
#endif
		}

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
	if(g_iTempScenario == 1)
		battery->battery_temp = g_iTempValue;
	else
		battery->battery_temp = b325_get_battery_temperature();
#else
	battery->battery_temp = b325_get_battery_temperature();
#endif

	battery->battery_present= b325_get_battery_present();

	battery->battery_valid= b325_get_battery_valid();

	printk("[325_BAT] %s() battery - %d%%, %dmV, %d'C  Present = %d  Valid = %d\n",__func__,
		battery->battery_soc, battery->battery_voltage, battery->battery_temp,
		battery->battery_present, battery->battery_valid);

	return 0;
}

static int b325_battery_data(void)
{
    battery_data battery = {0};
    static u64 last_gauging_time = 0;

    d_func("[325_BAT] %s()\n", __func__);

	if ( at_cmd_force_control == FALSE )
	{
		if (0 == last_gauging_time || last_gauging_time + 1 * HZ < get_jiffies_64())
			{
				last_gauging_time = get_jiffies_64();
				if (b325_battery_get_data(&battery) < 0)
					{
						last_gauging_time = 0;
						return 0;
					}
			}
		else
			{
				return 0;
			}
	}
	else
	{
		d_func("[325_BAT] %s() AT CMD force update Battery Information\n", __func__);
		at_cmd_force_control = FALSE;
		b325_battery_get_data(&battery);
	}

	//mutex_lock(&status_lock);

	d_func("[325_BAT] %s() Update Battery Information\n", __func__);

	//Add to support Factory Mode
	if ((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11))
	{
		if(battery.battery_soc ==0)
			battery.battery_soc = 1; //Prevent Power off if SOC is "0"
	}
#if defined(CONFIG_MACH_LGE_325_BOARD_VZW)
	else if((usb_chg_type_for_FG == 2) || (usb_chg_type_for_FG == 3))
	{
		if((battery.battery_soc ==0) && (battery.battery_voltage >= 3200))
		{
			battery.battery_soc = 1; //Prevent Power off if SOC is "0"
		}
	}
#endif
#ifdef	VZW_CEC_SPEC
	battery_dev->batteryChargerDone = chargerDone;
#endif
	battery_dev->batteryVoltage = battery.battery_voltage;
	battery_dev->batteryLifePercent= battery.battery_soc;
	if(battery_check == 1)
		battery_dev->batteryTemperature = battery.battery_temp;
	else
		battery_dev->batteryTemperature = 30;
	battery_dev->batteryTemperature *= 10;  // Android use x10 scale
	battery_dev->batteryPresent = battery.battery_present;
	battery_dev->batteryValid= battery.battery_valid;

#ifdef LOCAL_WORK_FOR_325_BATTERY
	if (battery_dev->batteryLifePercent < 5)
		battery_dev->batt_status_poll_period = 10 * MSEC_PER_SEC;
	else
	{
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
		if(g_iTempScenario == 0)
			battery_dev->batt_status_poll_period = battery_dev->batt_status_poll_period_normal;
#else
		battery_dev->batt_status_poll_period = battery_dev->batt_status_poll_period_normal;
#endif
	}
#else

	//Battery Information Polling Timer set
	if (battery_dev->batteryLifePercent > 10)
           battery_dev->batt_status_poll_period = BATTERY_POLLING_INTERVAL;
       else
           battery_dev->batt_status_poll_period = BATTERY_POLLING_INTERVAL_LOW;
#endif

	//mutex_unlock(&status_lock);

//	printk("[325_BAT] %s() battery updated - %d%%, %dmV, %d'C  Present = %d	Valid = %d\n",__func__,
//			battery_dev->batteryLifePercent, battery_dev->batteryVoltage, battery_dev->batteryTemperature,
//			battery_dev->batteryPresent, battery_dev->batteryValid);

    return 0;
}

static int b325_charger_control(bool charging_on_off)
{
	u8 fast_charge_current;

	if ( charging_on_off == TRUE ){
		at_cmd_force_charge_at56k = TRUE;
		charging_state = TRUE;

		fast_charge_current = 0x01;//Charging current set 550mAh, Termination current 100mA
		charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
		if (battery_dev->batteryVoltage==4350){
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x58); //Disable TE
			d_func("[325_BAT] %s() AT CMD force charge control fast_charge_current = %X \n", __func__, fast_charge_current);
		}else{
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x5C); //Enable TE
			d_func("[325_BAT] %s() AT CMD force charge control fast_charge_current = %X \n", __func__, fast_charge_current);
		}
		//Read 0x05 register. Then check the setting. If the setting doesn't need to update, skip write.
		//--------need to update.
	}
	else{
		at_cmd_force_charge_at56k = FALSE;
		charging_state = FALSE;

		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x5A); //Disable the charger
		d_func("[325_BAT] %s() AT CMD force charge control -> Disable the charger  \n", __func__);
	}

	return 0;
}

#ifdef	VZW_POWER_CAMERA_ON_CHECK
int is_camera(void)
{
	int iEnabled = 0;

	regulator_2v8 = regulator_get(NULL, "8058_l9");
	if(IS_ERR(regulator_2v8))	{
		printk("[spiderman] camera regulator 2v8 get failed\n");
		regulator_2v8 = NULL;
		return 0;
	}
	iEnabled = regulator_is_enabled(regulator_2v8);

	regulator_put(regulator_2v8);
	regulator_2v8 = NULL;

	printk("[spiderman] is_camera() enabled : %d\n", iEnabled);
	return iEnabled;
}
#endif

static int b325_charger_data(void)
{
	int i;
	static int first_flag = 1;

	u8 chargerType;
	u8 fast_charge_current=0;
	u8 dc_input_limit = 0;
	int bat_temp = 0;
	u8 reg_read = 0;
	u8 charger_stat = 0;
	charging_state = 0;

	c_func("[325_BAT] %s()\n", __func__);

	chargerType = Charger_Type_Battery; //Default value

	bat_temp = (battery_dev->batteryTemperature)/10;

	reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_SAFETMR_NTCMON);
	if(reg_read & 0x9)
	{
		reg_read &= 0xF6;
		charger_write_reg(battery_dev->client, BQ24160_REG_SAFETMR_NTCMON, reg_read);
	}

	//	EN_NOBATOP bit is always set in the 0x02 Register
	reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);
	reg_read |= 0x01;
	charger_write_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT, reg_read);

	if ( usb_chg_type_for_FG == 2){ //TA
		chargerType = Charger_Type_AC;

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
		b325_charger_temp_control(bat_temp);
#else
		if (b325_charger_temp_control(bat_temp) == 0)
			return chargerType;
#endif

		reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
		if ( reg_read < 0)
		{
			printk("charger_read_reg error, skip b325_charger_data one time. \n");
			return 2;
		}
		c_func("[325_BAT] BQ24160_REG_STAT_CTRL = %X \n", reg_read);

		charger_stat = (reg_read & BQ24160_STAT_MASK) >> BQ24160_STAT_SHFT;

		if (battery_check == TRUE)
		{
			charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, BQ24160_SUPPLY_SEL_MASK);
		}

#ifdef	VZW_CEC_SPEC
		if(chargerLogo[0] == 'a')	{	//half booting
			if(b325_chg.wake_lock_flag == TRUE)
			{
				printk("[charger_data : monitor unlock] half booting\n");
				full_charge_error_flag = FALSE;
				wake_unlock(&b325_chg.chg_wake_lock);
				b325_chg.wake_lock_flag = FALSE;
			}
		}
		else	{	//on booting
			if((charger_stat == 0x3)||(charger_stat == 0x4)) //Charging
			{
				if ( b325_chg.wake_lock_flag == FALSE )
				{
					printk("[charger_data : monitor lock] on booting\n");
					wake_lock(&b325_chg.chg_wake_lock);
					b325_chg.wake_lock_flag = TRUE;
				}
			}
			else if(charger_stat == 0x5) // Charge Done
			{
				if (battery_dev->batteryLifePercent < 100 )
				{
					b325_full_charge_error_handler();
				}
				else
				{
					if (( b325_chg.wake_lock_flag == TRUE ) && ( battery_dev->batteryLifePercent == 100))
					{
						printk("[charger_data : monitor unlock] on booting\n");
						full_charge_error_flag = FALSE;
						wake_unlock(&b325_chg.chg_wake_lock);
						b325_chg.wake_lock_flag = FALSE;
					}
				}
			}
		}
#else	//                  
		if ((charger_stat == 0x3)||(charger_stat == 0x4)	) //Charging
		{
			if ( b325_chg.wake_lock_flag == FALSE )
			{
				wake_lock(&b325_chg.chg_wake_lock);
				b325_chg.wake_lock_flag = TRUE;
			}
		}
		else if(charger_stat == 0x5) // Charge Done
		{
			if (battery_dev->batteryLifePercent < 100 )
			{
				//printk("[325_BAT]b325_full_charge_error_handler call SOC = %d\n  ", battery_dev->batteryLifePercent);
				b325_full_charge_error_handler();
			}
			else
			{
				if (( b325_chg.wake_lock_flag == TRUE ) && ( battery_dev->batteryLifePercent == 100))
				{
					full_charge_error_flag = FALSE;
					wake_unlock(&b325_chg.chg_wake_lock);
					b325_chg.wake_lock_flag = FALSE;
				}
			}
		}
#endif	//                  

		if (full_charge_error_flag == FALSE)
		{
			reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);

			switch(get_ext_cable_type_value())
			{
			case TA_CABLE_600MA:	// 180k, 200k TA => USB800, 700mA charging
				reg_read = (reg_read & 0x0F) | 0x30;
				fast_charge_current = 0x11;
				break;

			case TA_CABLE_800MA:	// 220k TA => USB1.5A, 850mA charging
			#ifdef	VZW_POWER_CAMERA_ON_CHECK
				if(is_camera() == 1)	{	//camera TA => USB800, 700mA charging
					reg_read = (reg_read & 0x0F) | 0x30;
					fast_charge_current = 0x11;
				}
				else	{					//USB1.5A, 850mA charging
					reg_read = (reg_read & 0x0F) | 0x50;
					fast_charge_current = 0x21;
				}
			#else
				reg_read = (reg_read & 0x0F) | 0x50;
				fast_charge_current = 0x21;
			#endif
				break;

			case TA_CABLE_DTC_800MA:// 270k Desktop Cradle => USB500mA, 550mA charging
			default: // others => USB800, 550mA charging
				reg_read = (reg_read & 0x0F) | 0x20;
				fast_charge_current = 0x00;
				break;
			}

		#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
			if(charging_state)
			{
				reg_read &= 0x7D;
			}
			else
			{
				reg_read |= 0x02;
				reg_read &= 0x7F;
			}

			if(temp_scenario_state == CHAR_TEMP_SCENARIO_45TO55_UNDER4000)
			{
				fast_charge_current = 0x00;
				printk(KERN_INFO "[325_BAT] under 4.0V, over 45 degC. limit current 550mA\n");
			}
		#endif
		}

//		c_func("[325_BAT] %s() dc_input_limit = %X, fast_charge_current = %X \n", __func__, dc_input_limit, fast_charge_current);

		//Read 0x03, 0x05 register. Then check the setting. If the setting doesn't need to update, skip write.
		//--------need to update.

		charger_write_reg(battery_dev->client, BQ24160_REG_VINDPM_STAT, 0x1B);

		if (full_charge_error_flag == FALSE)
		{
			if(battery_dev->batteryVoltage > 4240)	{
				//termination current 200mA
				fast_charge_current &= 0xF8;
				fast_charge_current |= 0x03;
			}
			else if(battery_dev->batteryVoltage > 4230)	{
				//termination current 150mA
				fast_charge_current &= 0xF8;
				fast_charge_current |= 0x02;
			}
			else	{
				//termination current 100mA
				fast_charge_current &= 0xF8;
				fast_charge_current |= 0x01;
			}

			dc_input_limit = 0x90;	//Battery Regulation Voltage 4.22V
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);
			msleep(1000);
		}
		msleep(10);
		charger_fault_check(battery_dev->client);

#if 1
		//  [20120724] when connect TA cable when power off - not charging problem.
		if( first_flag == 1 && battery_check == 1)
		{
		//reset charger ic
			reg_read = charger_reg_backup[BQ24160_REG_CTRL];
			reg_read |= 0x1;
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read); //High impedancd mode 'HIGH'

			msleep(100);

			reg_read &= 0xFE;
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read); //High impedance mode "LOW"

			msleep(100);

			//Restore original registor setting
			for(i = 0; i < 8; i++)
			{
				printk("[charger] Restore original registor settings 0x%x[0x%x]\n", i, charger_reg_backup[i]);
				charger_write_reg(battery_dev->client, i, charger_reg_backup[i]);
			}
		}
		first_flag = 0;
#endif

	}
	else if ( usb_chg_type_for_FG == 3) { //USB
		chargerType = Charger_Type_USB;

		fast_charge_current = 0x0;//Charging current set 550mAh, Termination current 50mA

		//Read 0x03, 0x05 register. Then check the setting. If the setting doesn't need to update, skip write.
		//--------need to update.
		if ((usb_cable_info == 7) || (usb_cable_info == 11)){

			if (battery_check == TRUE)//Change the Input source from USB to IN port from Rev.D & Factory support.
			{
				charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, 0);
			}

			if(at_cmd_force_charge_at56k==FALSE){	// When at%charge = 1 is not recived, discharing at factory cable
				if(charging_state == TRUE)
				{
					charging_state = FALSE;
					//b325_charger_termination_control(1);
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x4E); //USB current limit set 900mA, Disable CE.
					printk("[325_BAT] Factory USB mode usb_cable_info = %d \n", usb_cable_info);
				}
			}else{					//When at%charge = 1 is recived, Charing at factory cable
				charging_state = TRUE;
				fast_charge_current = 0x31;//Charging current set 1000mAh, Termination current 100mA
				charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
				printk("[325_BAT] at%%charge = 1 is recived, charging \n");
			}

			dc_input_limit = 0x92;//Input current limit set 2500mAh
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);

		}
		else //Original USB connection part.
		{
			//Disable charging control from temperature
		#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
			b325_charger_temp_control(bat_temp);
		#else
			charging_state = TRUE;

			if (b325_charger_temp_control(bat_temp) == 0)
				return chargerType;
		#endif

			if (battery_check == TRUE)//Change the Input source to USB from Rev.D.
			{
				charger_write_reg(battery_dev->client, BQ24160_REG_VINDPM_STAT, 0x12);
	
				if(usb_cable_info==6)
				{
					//56k LT case

					//supply sel to IN
					charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, 0);

					dc_input_limit = 0x92;//Input current limit set 2500mAh
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);

					//charging current 550mA / 100mA termination
					charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);
				}
				else
				{
					//Normal USB

					//supply sel to USB
					charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, BQ24160_SUPPLY_SEL_MASK);

					dc_input_limit = 0x90;//Input current limit set 1500mAh, Battery Regulation Voltage 4.22V
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);

//					//charging current 550mA
//					charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x0);

					if(battery_dev->batteryVoltage > 4240)	{
						//termination current 200mA, Charge current 550mA
						fast_charge_current = 0x03;
					}
					else if(battery_dev->batteryVoltage > 4230)	{
						//termination current 150mA, Charge current 550mA
						fast_charge_current = 0x02;
					}
					else	{
						//termination current 100mA, Charge current 550mA
						fast_charge_current = 0x01;
					}
					charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
				}

				reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				if(usb_cable_info == 6) //56K cable
					reg_read = ((reg_read & 0x0F)| 0x40);	//USB current limit set 900mA
				else			//Normal USB
					reg_read = ((reg_read & 0x0F)| 0x20);
			#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
				if(charging_state)
				{
					//Enable CE
					reg_read &= 0x7D;
				}
				else
				{
					//Disable CE
					reg_read |= 0x02;
					reg_read &= 0x7F;
				}
			#endif
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);
				msleep(10);
				charger_fault_check(battery_dev->client);
			}
			else{
				if(usb_cable_info == 6) { //If there is no battery, Stop charging.
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x4E);
				}
			}
		}

#if 1
		//  [20120724] when connect TA cable when power off - not charging problem.
		if( first_flag == 1 && battery_check == 1)  {
			//reset charger ic
			reg_read = charger_reg_backup[BQ24160_REG_CTRL];
			reg_read |= 0x1;
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read); //High impedancd mode 'HIGH'

			msleep(100);

			reg_read &= 0xFE;
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read); //High impedance mode "LOW"

			msleep(100);

			//Restore original registor setting
			for(i = 0; i < 8; i++)  {
				printk("[charger] Restore original registor settings 0x%x[0x%x]\n", i, charger_reg_backup[i]);
				charger_write_reg(battery_dev->client, i, charger_reg_backup[i]);
			}
		}
		first_flag = 0;
#ifdef	VZW_CEC_SPEC
		chargerDone = 0;
#endif
#endif

	}
	else
	{
		chargerType = Charger_Type_Battery;
		isChargingDone = CHARGER_UNDONE; 
#ifdef	VZW_POWER_ACC_ADC_CHECK
		iChargingStatus = 0;
#endif
#ifdef	VZW_CEC_SPEC
		chargerDone = 0;
#endif
	#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
		temp_scenario_state = CHAR_TEMP_SCENARIO_SUCCESS;
	#endif
		if ((battery_check == TRUE) && (usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
		{
			//termination current 100mA, Charge current 550mA
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);
			//current limit 500mA, charging enable
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x2C);
			//supply_sel to USB
			charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, BQ24160_SUPPLY_SEL_MASK);
		}

		if ((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11))
		{
			charging_state = TRUE;
		}
		else
		{
			full_charge_error_flag = FALSE;
			battery_peak_voltage = 0;
			charging_state = FALSE;
		}

		if ( b325_chg.wake_lock_flag == TRUE )
		{
			wake_unlock(&b325_chg.chg_wake_lock);
			b325_chg.wake_lock_flag = FALSE;
		}
	}
#ifdef	VZW_CEC_SPEC
	printk("[battery_info] %s() boottype[%s] Battery=0,USB=1,AC=2 ChargerType[%d] BattType[%d] temp[%d] soc[%d][%d] \n",
		__func__, chargerLogo, chargerType, usb_cable_info, battery_dev->batteryTemperature, battery_dev->batteryLifePercent, battery_dev->batteryVoltage);
#endif
	return chargerType;
}

static int b325_power_get_property(struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val)
{
	d_func("[325_BAT] %s() psp:%d usb_chg_type_for_FG = %d \n", __func__, psp, usb_chg_type_for_FG);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
		if (psy->type == POWER_SUPPLY_TYPE_BATTERY)
			val->intval = (usb_chg_type_for_FG == 0);
#endif
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (usb_chg_type_for_FG == 2); /* not fixed */
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (usb_chg_type_for_FG == 3); /* not fixed */
		break;
#ifdef CONFIG_TEMP_VZW_CONFIG
	case POWER_SUPPLY_PROP_CABLE_INFO:
		val->intval = get_ext_cable_type_value();
		break;
#endif
#ifdef	VZW_POWER_ACC_ADC_CHECK
	case POWER_SUPPLY_PROP_ACC_ADC:
		acc_adc_value = get_usb_cable_adc();
		val->intval = acc_adc_value;
//		printk("[325_BAT] acc_adc_value %d\n", acc_adc_value);
		break;
	case POWER_SUPPLY_PROP_CABLE_NAME:
		val->intval = get_ext_cable_type_value();
//		printk("[325_BAT] cable_type %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_STATUS:
		val->intval = iChargingStatus;
		break;
#endif
	default:
			return -EINVAL;
	}
	return 0;
}

static int b325_battery_get_property(struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val)
{
	int batt_tech = 0;

	u8 chargerType;

	d_func("[325_BAT] %s() psp:%d \n", __func__, psp);

	b325_battery_data();

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        chargerType = b325_charger_data();
        if (battery_dev->batteryPresent)
        {
            if (chargerType == Charger_Type_AC || chargerType == Charger_Type_USB)
            {
#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
				if(	(temp_scenario_state == CHAR_TEMP_SCENARIO_OVER55) ||
					(temp_scenario_state == CHAR_TEMP_SCENARIO_UNDERM10))	{
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				}
				else if (battery_dev->batteryLifePercent == 100)
#else
				if (battery_dev->batteryLifePercent == 100)
#endif
				{
                    val->intval = POWER_SUPPLY_STATUS_FULL;

					if(pwrkey_led_status == 2)
					{
						//pm8058_pwrkey_led_blink_set(0); //LED off for the Full charging
					}
				}
                else
                    val->intval = POWER_SUPPLY_STATUS_CHARGING;
            }
            else
            {
                val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				if(pwrkey_led_status !=0)
				{
					//pm8058_pwrkey_led_blink_set(0); //Turn Off LED.
				}
            }
        }
        else
        {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        }
        d_func("[325_BAT] %s() psp:%d -> value:0x%x\n", __func__, psp, val->intval);
        break;

    case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_dev->batteryPresent;
        break;

    case POWER_SUPPLY_PROP_TECHNOLOGY:
        if (battery_dev->batteryPresent)
            batt_tech = POWER_SUPPLY_TECHNOLOGY_LION;
        else
            batt_tech = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        val->intval = batt_tech;
        break;

    case POWER_SUPPLY_PROP_HEALTH:
		if (battery_dev->batteryPresent)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
        break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_BATTERY_TEMP_ADC:
#ifdef	VZW_CEC_SPEC
	case POWER_SUPPLY_PROP_BATTERY_CHARGER_DONE:
#endif
        if (!battery_dev->batteryPresent)
        {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            return 0;
        }
        if (psp == POWER_SUPPLY_PROP_CAPACITY)
		{
			battery_dev->batteryLifePercent = max17040_get_battery_capacity_percent();
			printk("[battery_info] property_soc : %d\n", battery_dev->batteryLifePercent);
			val->intval = battery_dev->batteryLifePercent;

		} else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
			val->intval = battery_dev->batteryVoltage;
		}

        if (psp == POWER_SUPPLY_PROP_TEMP)	{
			val->intval = battery_dev->batteryTemperature;
		}

		if (psp == POWER_SUPPLY_PROP_BATTERY_TEMP_ADC)	{
			val->intval = battery_dev->batteryTemperature_adc;
		}
#ifdef	VZW_CEC_SPEC
		if (psp == POWER_SUPPLY_PROP_BATTERY_CHARGER_DONE)	{
			val->intval = battery_dev->batteryChargerDone;
		}
#endif
        break;

	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
		val->intval = b325_get_battery_valid();
		break;

    default:
        return -EINVAL;
    }
    return 0;
}

#ifdef LOCAL_WORK_FOR_325_BATTERY

static void b325_battery_work_func(struct work_struct *work)
{

    d_func("[325_BAT] %s() \n", __func__);
    power_supply_changed(&b325_supplies[Charger_Type_Battery]);
    //power_supply_changed(&b325_supplies[Charger_Type_USB]);
    //power_supply_changed(&b325_supplies[Charger_Type_AC]);

	queue_delayed_work(local_workqueue, &b325_chg.work, round_jiffies_relative(msecs_to_jiffies
						     (battery_dev->batt_status_poll_period)));

}


#else

static void b325_battery_poll_timer_func(unsigned long unused)
{
    d_func("[325_BAT] %s() \n", __func__);
    power_supply_changed(&b325_supplies[Charger_Type_Battery]);
    //power_supply_changed(&b325_supplies[Charger_Type_USB]);
    //power_supply_changed(&b325_supplies[Charger_Type_AC]);

    mod_timer(&(battery_dev->battery_poll_timer),
        jiffies + msecs_to_jiffies(battery_dev->batt_status_poll_period));
}
#endif

void update_battery_info(void)
{

	//printk("[325_BAT] update_battery_info enter.\n");

	if (power_supply_registered == FALSE)
		{
			printk("[325_BAT] Power supply is not registered yet.\n");
		}
	else
		{
			if(is_chg_plugged_in())
				{
					//if (pm8xxx_batt_alarm_config_lge_325(FALSE))
						//printk("%s: chg plugged-unable to set battery alarm \n", __func__);

					#if 0
					if((pwrkey_led_status == 0) && (battery_dev->batteryLifePercent !=100))
						{
							pm8058_pwrkey_led_blink_set(2); //Always ON LED for the charging status
						}
					else if ((pwrkey_led_status !=0) &&  (battery_dev->batteryLifePercent ==100))
						{
							pm8058_pwrkey_led_blink_set(0); //LED off for FULL Battery.
						}
					#endif

				}
			else
				{
					//pm8xxx_batt_alarm_config_lge_325(TRUE);
					#if 0
					if(pwrkey_led_status !=0)
						{
							pm8058_pwrkey_led_blink_set(0); //Turn Off LED.
						}
					#endif
					}

			power_supply_changed(&b325_supplies[Charger_Type_USB]);
			power_supply_changed(&b325_supplies[Charger_Type_AC]);
		}
}

#ifdef CHARGER_INTERRUTP_ENABLE
static void b325_charger_state_handle(struct work_struct *wq)
{
	int read_reg = 0, i=0, tmp=0;

	c_func("[325_BAT] %s() \n", __func__);

	msleep(30); //Before check the charger register, wait 30ms.

	msleep(1000);

	//Read 0x00 charger register
	read_reg = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
//	c_func("[325_BAT] BQ24160_REG_STAT_CTRL = %X \n", read_reg);
	printk("[spiderman] interrupt : BQ24160_REG_STAT_CTRL = %X \n", read_reg);

	tmp = (read_reg >> 4) & 0x07;

	//fault checking
	read_reg &= 0x07;

	while(read_reg == 7)// FAULT_BATTERY)
	{
		read_reg = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);
		read_reg = (read_reg >> 1) & 0x03;

		if(read_reg != 1)	{
			printk("[lk_charger] not Battery OVP Fault [0x%x]\n", read_reg);
			return;
		}
		
		read_reg = charger_reg_backup[BQ24160_REG_CTRL];
		read_reg |= 0x1;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, read_reg);	//High impedancd mode 'HIGH'

		msleep(1000);

		read_reg &= 0xFE;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, read_reg);	//High impedance mode "LOW"

		msleep(1000);

		//Restore original registor setting
		for(i = 0; i < 8; i++)
		{
			printk("[charger] Restore original registor settings 0x%x[0x%x]\n", i, charger_reg_backup[i]);
			charger_write_reg(battery_dev->client, i, charger_reg_backup[i]);
		}

		//fault check	
		read_reg = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);	//0x00
		read_reg &= 0x07;

		printk("[charger] Battery OVP Error\n");
	}

	//	is charging done?
	if(tmp == 5)	{
		if(battery_dev->batteryLifePercent < 100)	{
			isChargingDone = CHARGER_DONE_99;
		}
		else if(battery_dev->batteryLifePercent == 100)	{
			isChargingDone = CHARGER_DONE_100;
		}
		//charging disable
		read_reg = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		read_reg |= 0x2;
		read_reg &= 0x7F;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, read_reg); //Disable CE.
		charging_state = FALSE;
		printk(KERN_INFO "[spiderman] charging done or ready by interrupt(charging disable) %d \n", tmp);
	}

//	b325_charger_data();

    //power_supply_changed(&b325_supplies[Charger_Type_USB]);
    //power_supply_changed(&b325_supplies[Charger_Type_AC]);
}
#endif

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
//extern int pm8058_start_charging_for_ATCMD(void);
//extern int pm8058_stop_charging_for_ATCMD(void);

extern void max17040_set_battery_atcmd(int flag, int value);
extern int max17040_get_battery_capacity_percent(void);
extern void (*arm_pm_restart)(char str, const char *cmd);


static ssize_t at_chg_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
  bool b_chg_ok = false;

  if(charging_state == TRUE )
  {
    b_chg_ok = true;
	  r = sprintf(buf, "%d\n", b_chg_ok);
  }
  else
  {
    b_chg_ok = false;
	  r = sprintf(buf, "%d\n", b_chg_ok);
  }

	return r;
}


static ssize_t at_chg_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  //int ret;
  unsigned char string[2];

	sscanf(buf, "%s", string);

  if (!count)
		return -EINVAL;

  if(!strncmp(string, "0", 1))
  {
    /* Stop Charging */
	b325_charger_control(0);

  }
  else if(!strncmp(string, "1", 1))
  {
    /* Start Charging */
	b325_charger_control(1);

  }

	return count;
}


static ssize_t at_chg_status_complete_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool b_chg_complete = false;

  if(battery_dev->batteryLifePercent == 100)
  {
    b_chg_complete = true;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }
  else
  {
    b_chg_complete = false;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }

	return r;
}

static ssize_t at_chg_status_complete_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   // int ret;
    unsigned char string[2];

    sscanf(buf, "%s", string);

    if (!count)
      return -EINVAL;

    if(!strncmp(string, "0", 1))
    {
      /* Charging not Complete */
	  d_func("[325_BAT] %s()  Charging not Complete \n", __func__);
	  //Need to check if this function should be implemented.

    }
    else if(!strncmp(string, "1", 1))
    {
      /* Charging Complete */
	  d_func("[325_BAT] %s()  Charging Complete \n", __func__);
	  //Need to check if this function should be implemented.

    }

    return count;

}



static ssize_t at_fuel_guage_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	max17040_set_battery_atcmd(0, 100);  // Reset the fuel guage IC

  r = sprintf(buf, "%d\n", true);

  at_cmd_force_control = TRUE;

	msleep(300);

  b325_battery_data();

	max17040_set_battery_atcmd(2, 100);  // Release the AT command mode

	return r;
}


static ssize_t at_fuel_guage_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int guage_level = 0;

#ifdef CONFIG_BATTERY_MAX17040
	guage_level = max17040_get_battery_capacity_percent();
#else
  guage_level = max17047_get_battery_capacity_percent();
#endif
  r = sprintf(buf, "%d\n", guage_level);

	return r;
}

static ssize_t at_pmic_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	msleep(500); //for waiting return values of testmode

	machine_restart(NULL);

	r = sprintf(buf, "%d\n", true);

	return r;
}


static ssize_t at_batt_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int battery_level = 0;

#ifdef CONFIG_BATTERY_MAX17040
	battery_level = max17040_get_battery_mvolts();
#else
  battery_level = max17047_get_battery_mvolts();
#endif

	printk(KERN_DEBUG "############ [at_batt_level_show] BATT LVL = %d #####################\n", battery_level);

	r = sprintf(buf, "%d\n", battery_level);

	return r;
}

static ssize_t at_tempctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	if(temp_control == TRUE)
	{
		r = sprintf(buf, "%d\n", temp_control);
	}
	else
	{
		r = sprintf(buf, "%d\n", temp_control);
	}
	return r;
}

static ssize_t at_tempctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   // int ret;
    unsigned char string[2];

    sscanf(buf, "%s", string);

    if (!count)
      return -EINVAL;

    if(!strncmp(string, "0", 1))    {
      /* Temp control disable - Always charging even out of temp range */
	  //printk("[325_BAT] %s()  Temp control disable \n", __func__);
	  temp_control = FALSE;
    }
    else if(!strncmp(string, "1", 1))
    {
      /* Temp control enable - Charging controlled by Temp. */
	  //printk("[325_BAT] %s()  Temp control enable  \n", __func__);
	  temp_control = TRUE;
    }
	return count;
}

#ifdef CONFIG_BATMAN_VZW_POWER_BATT_THRM_SCENARIO
static ssize_t test_tempctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	char cHelp[150] = "\r\nex) echo [on/off] [type] [temp]\r\n\ton/off : on[1], off[0]\r\n\ttype : [bat], [ac], [usb]\r\n\ttemp : [temperature value]\r\n";

	r = sprintf(buf, "%s\n", cHelp);
	return r;
}
	
static ssize_t test_tempctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char string[4];
	battery_data battery = {0};

	sscanf(buf, "%d %s %d", &g_iTempScenario, string, &g_iTempValue);

	if(!count)
		return -EINVAL;

	if(g_iTempScenario == 0)	{
		/* Temp control disable - Always charging even out of temp range */
		//printk("[325_BAT] %s()  Temp control disable \n", __func__);
	}
	else	{
		/* Temp control enable - Charging controlled by Temp. */
		//printk("[325_BAT] %s()  Temp control enable \n", __func__);

		//work_queue restart
		cancel_delayed_work(&b325_chg.work);
		battery_dev->batt_status_poll_period = 3000L;
		queue_delayed_work(local_workqueue, &b325_chg.work, round_jiffies_relative(msecs_to_jiffies
															 (battery_dev->batt_status_poll_period)));
	
		if(!strncmp(string, "bat", 1))
			usb_chg_type_for_FG = POWER_SUPPLY_TYPE_BATTERY;	//with battery only

		else if(!strncmp(string, "ac", 1))
			usb_chg_type_for_FG = POWER_SUPPLY_TYPE_MAINS;		//ac

		else if(!strncmp(string, "usb", 1))
			usb_chg_type_for_FG = POWER_SUPPLY_TYPE_USB;		//usb

		battery_dev->batteryTemperature = g_iTempValue*10;
		battery.battery_temp = g_iTempValue;
	}

	printk("## [test_tempctrl_store] enable[%d] temp[%d] type[%s] ##\n", g_iTempScenario, g_iTempValue, string);

	return count;
}
#endif

#ifdef	VZW_POWER_SOC_CONTROL
static ssize_t test_socctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int battery_level = 0;

	char cHelp[150]="\r\nex) echo [on/off] [type] [value]\r\n\ton/off : on[1], off[0]\r\n\ttype : [soc]\r\n\tvalue : [soc value]\r\ncurrent soc level :";

#ifdef CONFIG_BATTERY_MAX17040
	if(g_iSocControl == 1)
		battery_level = g_iSocValue;
	else
		battery_level = max17040_get_battery_mvolts();
#else
	battery_level = max17047_get_battery_mvolts();
#endif

	printk(KERN_DEBUG "############ [test_socctrl_show] BATT LVL = %d #####################\n", battery_level);

	r = sprintf(buf, "%s %d\n", cHelp, battery_level);

	return r;
}


static ssize_t test_socctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char string[4];
	int iValue, iControl;

	//enable, string, value
	sscanf(buf, "%d %s %d", &iControl, string, &iValue);
	if(!count)
		return -EINVAL;

	if(iControl == 1)	{
		cancel_delayed_work(&b325_chg.work);
		battery_dev->batt_status_poll_period = 3000L;
		queue_delayed_work(local_workqueue, &b325_chg.work, round_jiffies_relative(msecs_to_jiffies
														     (battery_dev->batt_status_poll_period)));
		if(!strncmp(string, "bat", 1))	{
			g_iBatteryControl = iControl;	//battery voltage
			g_iBatteryValue = iValue;
		}
		else if(!strncmp(string, "soc", 1))	{
			g_iSocControl = iControl;		//soc
			g_iSocValue = iValue;
		}
		else	{
			g_iSocControl = 0;
			g_iBatteryControl = 0;
			iControl = 0;
		}
	}
	else	{
		g_iSocControl = iControl;
		g_iBatteryControl = iControl;
	}
	
	printk("## [test_socctrl_store] enable[%d] value[%d] type[%s] ##\n", g_iSocControl, g_iSocValue, string);

	return count;	
}
#endif


#ifdef	VZW_POWER_TEMP_PROP_CHECK
static ssize_t high_temp_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = sprintf(buf, "%d\n", iHighTempOff);

	printk("## [high_temp_off_show] iHighTempOff[%d]\n", iHighTempOff);

	return r;
}

static ssize_t high_temp_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	iHighTempOff = 0;

	sscanf(buf, "%d", &iHighTempOff);
	if(!count)
		return -EINVAL;

	printk("## [high_temp_off_store] iHighTempOff[%d]\n", iHighTempOff);

	return count;	
}
#endif


#ifdef	VZW_POWER_ACC_ADC_CHECK
static ssize_t acc_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = sprintf(buf, "%d\n", acc_adc_value);

	printk("## [acc_adc_show] acc_adc_value[%d]\n", acc_adc_value);

	return r;
}

static ssize_t cable_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = sprintf(buf, "%d\n", cable_name);

	printk("## [cable_name_show] cable_name[%d]\n", cable_name);

	return r;
}

static ssize_t charging_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = sprintf(buf, "%d\n", iChargingStatus);

	printk("## [charging_status_show] charging_status[%d]\n", iChargingStatus);

	return r;
}
#endif

#ifdef	VZW_CEC_SPEC
static ssize_t chargerLogo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r=0;

	printk("## [chargerLogo_show] %s \n", chargerLogo);
	r = sprintf(buf, "%s", chargerLogo);

	return r;
}

static ssize_t chargerLogo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	chargerLogo[0] = 'a';

	printk("## [chargerLogo_store] %s\n", chargerLogo);
	sscanf(buf, "%s", chargerLogo);

	return size;
}

static ssize_t chargerDone_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r=0;

	printk("## [chargerDone_show] %d \n", chargerDone);
	r = sprintf(buf, "%d", chargerDone);

	return r;
}

static ssize_t chargerDone_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("## [chargerDone_store] %d\n", chargerDone);
	sscanf(buf, "%d", &chargerDone);

	return size;
}
#endif

DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_status_complete_show, at_chg_status_complete_store);
DEVICE_ATTR(at_fuelrst, 0644, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0644, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_pmrst, 0644, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_batl, 0644, at_batt_level_show, NULL);
DEVICE_ATTR(at_tempctrl, 0660, at_tempctrl_show, at_tempctrl_store);
#endif

#ifdef	VZW_POWER_SOC_CONTROL
DEVICE_ATTR(test_tempctrl, 0660, test_tempctrl_show, test_tempctrl_store);
DEVICE_ATTR(test_socctrl, 0660, test_socctrl_show, test_socctrl_store);
#endif

#ifdef	VZW_POWER_TEMP_PROP_CHECK
DEVICE_ATTR(high_temp_off, 0660, high_temp_off_show, high_temp_off_store);
#endif

#ifdef	VZW_POWER_ACC_ADC_CHECK
DEVICE_ATTR(acc_adc, 0660, acc_adc_show, NULL);
DEVICE_ATTR(cable_name, 0660, cable_name_show, NULL);
DEVICE_ATTR(charging_status, 0660, charging_status_show, NULL);
#endif

#ifdef	VZW_CEC_SPEC
DEVICE_ATTR(chargerLogo, 0660, chargerLogo_show, chargerLogo_store);
DEVICE_ATTR(chargerDone, 0660, chargerDone_show, chargerDone_store);
#endif

//#ifdef LG_FW_WEB_DOWNLOAD
int is_batt_lvl_present(void)
{
#ifdef CONFIG_BATTERY_MAX17040
	return(max17040_get_battery_capacity_percent());
#else
	return(max17047_get_battery_capacity_percent());
#endif
}
//#endif /*LG_FW_WEB_DOWNLOAD*/

#ifdef CONFIG_LGE_PM_CURRENT_CABLE_TYPE
int get_is_battery_present(void)
{
	//Check the battery or not, return value 0 (No battery), 1(Battery)

	return 1; //Temporary return 1
};
EXPORT_SYMBOL(get_is_battery_present);
#endif

#if 1
int is_chg_plugged_in(void)
{

	u8 reg_val;
	u8 dc_check;

	//printk("is_chg_plugged_in enter \n");

	//Read Charger register to know whether External power is plugged in or out.

	reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);

	dc_check = reg_val & BQ24160_STAT_MASK;
	dc_check = dc_check >> BQ24160_STAT_SHFT;

	switch(dc_check)
	{
	case BQ24160_STAT_IN_READY:
	case BQ24160_STAT_USB_READY:
	case BQ24160_STAT_CHARGING_FROM_IN:
	case BQ24160_STAT_CHARGING_FROM_USB:
	case BQ24160_STAT_CHARGE_DONE:
		dc_check = 1;
		break;

	case BQ24160_STAT_NO_VALID_SRC_DETECTED:
		dc_check = 0;
		break;

	default:
		break;
	}

//	printk("[325_BAT]%s external_power_check dc_check = %d \n", __func__, dc_check);

		return dc_check;
}
EXPORT_SYMBOL(is_chg_plugged_in);
#endif

bool is_bat_connected_in(void)
{
	static u8 checked = 0;
	u8 reg_val;
	u8 bat_check;
	u8 en_nobatop;

	if(checked == 1)
		return battery_check;

	c_func("[325_BAT] %s()\n", __func__);

	reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);

	bat_check = (reg_val & BQ24160_BATTSTAT_MASK) >> BQ24160_BATTSTAT_SHFT;

	en_nobatop = reg_val | 0x1;

	c_func("[325_BAT]%s is_bat_connected_in bat_check = %d en_nobatop = %x\n", __func__, bat_check, en_nobatop);

	if ( bat_check == 2)
	{
		//charger_write_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT, en_nobatop);

		reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		reg_val |= 0x2;
		reg_val &= 0x7F;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val); //Disable CE
		battery_check = FALSE;
	}
	else
	{
		battery_check = TRUE;
	}

	checked = 1;
	return battery_check;
}



static const struct i2c_device_id bq24160_id[] = {
	{ "bq24160", 0 },
	{ }
};


static int __devinit b325_battery_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	battery_dev->client = client;

		d_func("[325_BAT]%s\n", __func__);

	i2c_set_clientdata(client, battery_dev);

	battery_dev->chg_termination_flag = TRUE;

	charger_fault_check(client);

    return 0;

}


struct i2c_driver b325_battery_i2c_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = "bq24160",
  },
  .id_table = bq24160_id,
  .probe    = b325_battery_i2c_probe,
};


static int __devinit b325_battery_probe(struct platform_device *pdev)
{
	int i, rc;

	int err;

	int dc_check;

#ifdef LOCAL_WORK_FOR_325_BATTERY
	unsigned int milli_secs;
#endif

#ifdef CHARGER_INTERRUTP_ENABLE
	int ret;
#endif

	battery_dev = kzalloc(sizeof(*battery_dev), GFP_KERNEL);
	if (!battery_dev)
		return -ENOMEM;

	battery_dev->dev = &pdev->dev;

		d_func("[325_BAT]%s\n", __func__);

	battery_dev->pdata = (struct bq24160_platform_data *)pdev->dev.platform_data;


	battery_dev->driver = &b325_battery_i2c_driver;

		err = i2c_add_driver(battery_dev->driver );

	battery_dev->batteryVoltage = 3700;
	battery_dev->batteryLifePercent = 50;
	battery_dev->batteryPresent = 1;      /* Assume battery is present at start */

#ifdef LOCAL_WORK_FOR_325_BATTERY
		milli_secs = 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			printk("%s: update time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		battery_dev->batt_status_poll_period = milli_secs;
		battery_dev->batt_status_poll_period_normal  = milli_secs;
#else

	battery_dev->batt_status_poll_period = BATTERY_POLLING_INTERVAL_LOW;

#endif


	mutex_init(&status_lock);

	#ifdef CHARGER_INTERRUTP_ENABLE
	INIT_WORK(&battery_dev->update_charger_info_work, b325_charger_state_handle);
	#endif

	#ifdef BATTERY_CUSTOM_MODEL
	battery_dev->load_custom_model = false;
	#endif

	dc_check = is_chg_plugged_in();

	is_bat_connected_in();

	wake_lock_init(&b325_chg.chg_wake_lock, WAKE_LOCK_SUSPEND, "b325_charger_monitor");

	#if 0
	//#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_init(&batt_dev->wlock, WAKE_LOCK_SUSPEND, "battery_check");
	#endif

	#ifdef LOCAL_WORK_FOR_325_BATTERY

	INIT_DELAYED_WORK(&b325_chg.work, b325_battery_work_func);
	queue_delayed_work(local_workqueue, &b325_chg.work, BATTERY_POLLING_INTERVAL_LOW);

	#else

	setup_timer(&(battery_dev->battery_poll_timer), b325_battery_poll_timer_func, 0);
	mod_timer(&(battery_dev->battery_poll_timer),
	jiffies + msecs_to_jiffies(battery_dev->batt_status_poll_period));

	#endif


	for (i = 0; i < ARRAY_SIZE(b325_supplies); i++) {
		rc = power_supply_register(battery_dev->dev, &b325_supplies[i]);
		if (rc) {
				printk(KERN_ERR "Failed to register power supply\n");
				while (i--)
					power_supply_unregister(&b325_supplies[i]);
				kfree(battery_dev);
				return rc;
				}
		}

	printk(KERN_INFO "[325_BAT] B325 battery driver registered\n");
	power_supply_registered = TRUE;


#ifdef CHARGER_INTERRUTP_ENABLE

	//Register Charger Interrupt

	charger_irq_init(SWITCHING_CHG_IRQ_N);

	ret = request_threaded_irq(	battery_dev->client->irq,
								NULL,
								charger_handler,
//								(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
								IRQF_TRIGGER_RISING,
								"max8971-irq",
								battery_dev);

	if (unlikely(ret < 0))	{
		printk("[325_BAT]max8971: failed to request IRQ %X\n", ret);
	}

	printk(KERN_INFO "[325_BAT] %s: charger interrupt registered\n", battery_dev->client->name);
#endif

	if ((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11))
		{
			charging_state = TRUE;
		}

	//Charging LED init
	#if 0
	if ( dc_check == 1)
		{
			pm8058_pwrkey_led_blink_set(2); //Always ON LED for the charging
		}
	#endif

	//DCCT valiable init.
	pmic_die_temp_range = 0;

	//Full charge error flag init.
	full_charge_error_flag = FALSE;

	//MSM_ADC_CALIBRATION flag init.
	conv_first_request = TRUE;

	// OTP_ENABLE is default by HW_request.
	temp_control = TRUE;

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  err = device_create_file(&pdev->dev, &dev_attr_at_charge);
  err = device_create_file(&pdev->dev, &dev_attr_at_chcomp);
  err = device_create_file(&pdev->dev, &dev_attr_at_fuelrst);
  err = device_create_file(&pdev->dev, &dev_attr_at_fuelval);
  err = device_create_file(&pdev->dev, &dev_attr_at_pmrst);
  err = device_create_file(&pdev->dev, &dev_attr_at_batl);
  err = device_create_file(&pdev->dev, &dev_attr_at_tempctrl);
#endif

#ifdef	VZW_POWER_SOC_CONTROL
	err = device_create_file(&pdev->dev, &dev_attr_test_tempctrl);
	err = device_create_file(&pdev->dev, &dev_attr_test_socctrl);
#endif

#ifdef	VZW_POWER_TEMP_PROP_CHECK
	err = device_create_file(&pdev->dev, &dev_attr_high_temp_off);
#endif

#ifdef	VZW_POWER_ACC_ADC_CHECK
	err = device_create_file(&pdev->dev, &dev_attr_acc_adc);
	err = device_create_file(&pdev->dev, &dev_attr_cable_name);
	err = device_create_file(&pdev->dev, &dev_attr_charging_status);
#endif
#ifdef	VZW_CEC_SPEC
	err = device_create_file(&pdev->dev, &dev_attr_chargerLogo);
	err = device_create_file(&pdev->dev, &dev_attr_chargerDone);
#endif
	return 0;

}

static int __devexit b325_battery_remove(struct platform_device *pdev)
{

		int i;

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
	device_remove_file(&pdev->dev, &dev_attr_at_chcomp);
	device_remove_file(&pdev->dev, &dev_attr_at_fuelrst);
	device_remove_file(&pdev->dev, &dev_attr_at_fuelval);
	device_remove_file(&pdev->dev, &dev_attr_at_pmrst);
	device_remove_file(&pdev->dev, &dev_attr_at_batl);
	device_remove_file(&pdev->dev, &dev_attr_at_tempctrl);
#endif

#ifdef	VZW_POWER_SOC_CONTROL
	device_remove_file(&pdev->dev, &dev_attr_test_tempctrl);
	device_remove_file(&pdev->dev, &dev_attr_test_socctrl);
#endif

#ifdef	VZW_POWER_TEMP_PROP_CHECK
	device_remove_file(&pdev->dev, &dev_attr_high_temp_off);
#endif

#ifdef	VZW_POWER_ACC_ADC_CHECK
	device_remove_file(&pdev->dev, &dev_attr_acc_adc);
	device_remove_file(&pdev->dev, &dev_attr_cable_name);
	device_remove_file(&pdev->dev, &dev_attr_charging_status);
#endif
#ifdef	VZW_CEC_SPEC
	device_remove_file(&pdev->dev, &dev_attr_chargerLogo);
	device_remove_file(&pdev->dev, &dev_attr_chargerDone);
#endif
	d_func("[325_BAT]%s\n", __func__);

	i2c_del_driver(&b325_battery_i2c_driver);

#ifdef LOCAL_WORK_FOR_325_BATTERY
	cancel_delayed_work(&b325_chg.work);
#endif

	for (i = 0; i < ARRAY_SIZE(b325_supplies); i++) {
		power_supply_unregister(&b325_supplies[i]);
		}

//	free_irq(client->irq, chip);

//	kfree(chip);

    return 0;
}

#if 0
static int b325_battery_shutdown(struct i2c_client *client)
{

	d_func("[325_BAT]%s\n", __func__);

	//del_timer_sync(&(batt_dev->battery_poll_timer));

	//disable_irq(gpio_to_irq(GPIO_AC_DETECT));
	//disable_irq(gpio_to_irq(GPIO_USB_DETECT_N));

	return 0;
}
#endif


#ifdef CONFIG_PM
static int b325_battery_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	d_func("[325_BAT]%s\n", __func__);

	if (!is_chg_plugged_in()) {
		pm8xxx_batt_alarm_config_lge_325(TRUE);
	}
#ifdef LOCAL_WORK_FOR_325_BATTERY
	cancel_delayed_work(&b325_chg.work);
#endif

	return 0;
}


static int b325_battery_resume(struct platform_device *pdev)
{
	d_func("[325_BAT]%s\n", __func__);
	pm8xxx_batt_alarm_config_lge_325(FALSE);

#ifdef LOCAL_WORK_FOR_325_BATTERY
	queue_delayed_work(local_workqueue, &b325_chg.work, BATTERY_POLLING_IMMEDIATELY);
#else

	mod_timer(&(battery_dev->battery_poll_timer),
        jiffies + msecs_to_jiffies(200));
#endif

	return 0;
}
#endif


static struct platform_driver b325_battery_driver = {
	.probe = b325_battery_probe,
	.remove = __devexit_p(b325_battery_remove),
	.suspend = b325_battery_suspend,
	.resume = b325_battery_resume,
	.driver = {
		.name = "b325_battery",
		.owner = THIS_MODULE,
	}
};


static int __init b325_battery_init(void)
{
	d_func("[325_BAT]%s: b325_battery_driver\n", __func__);

#ifdef LOCAL_WORK_FOR_325_BATTERY
	local_workqueue = create_workqueue("b325_battery_update");

	if (!local_workqueue)
		return -ENOMEM;
#endif

	return platform_driver_register(&b325_battery_driver);

}

static void __exit b325_battery_exit(void)
{
#ifdef LOCAL_WORK_FOR_325_BATTERY
	if (local_workqueue)
			destroy_workqueue(local_workqueue);

		local_workqueue = NULL;
#endif

	platform_driver_unregister(&b325_battery_driver);

}

module_init(b325_battery_init);
module_exit(b325_battery_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maxim-ic.com>");
MODULE_DESCRIPTION("Power supply driver for MAX8971");
MODULE_ALIAS("platform:max8971-charger");
