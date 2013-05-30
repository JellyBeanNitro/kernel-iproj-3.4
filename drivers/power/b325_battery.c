/*
 *  325 Battery Driver
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

#include <mach/gpio.h>
#include <linux/msm-charger.h>
#include <linux/msm_adc.h>

#include <linux/delay.h>
#include <linux/time.h>


#include "../../lge/include/board_lge.h"

//#define BATTERY_INFO_SHOW
//#define BATTERY_FUNCTION_DEBUG
//#define CHARGER_INFO_SHOW

//Default disable the charger interrupt
//#define CHARGER_INTERRUTP_ENABLE

//OTP(Over Temperature Protection) Disable for DV test only
//#define OTP_DISABLE

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
//ACC(Adaptive Charging-current Contorl)
#define ACC_ENABLE
#endif

//LOCAL WORK for 325 Battery instead of mod timer
#define LOCAL_WORK_FOR_325_BATTERY

#ifdef BATTERY_INFO_SHOW
#define d_bat(fmt, ...) printk(KERN_INFO fmt, __VA_ARGS__)
#else
#define d_bat(fmt, ...) do {} while (0)
#endif

#define d_bat_ex(fmt, ...) printk(KERN_INFO fmt, __VA_ARGS__)

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

#define BATTERY_LEVEL_MULTIPLIER      (1000)	/*                                                      */

#define BATTERY_POLLING_INTERVAL      (60 * 1000)   // 60s
#define BATTERY_POLLING_INTERVAL_LOW  (10 * 1000)   // 10s
#define BATTERY_POLLING_IMMEDIATELY   (30)         // 30ms
//#define BATTERY_DATA_UNKNOWN          0x7FFFFFFF

//#define BATTERY_CUSTOM_MODEL //Max17043 Custom Model Setting

/*                                                                           */
#define SKIP_INVALID_TEMP
/*                                                                         */

#define I2C_MAX_ERR_COUNT  5
static int switching_charger_i2c_error = 0;

#ifdef LOCAL_WORK_FOR_325_BATTERY
static struct workqueue_struct *local_workqueue;
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

static enum power_supply_property b325_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
#ifdef CONFIG_BATTERY_325_DCM
	POWER_SUPPLY_PROP_CURRENT_NOW,
#endif
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
	POWER_SUPPLY_PROP_BATTERY_TEMP_ADC,
#ifdef CONFIG_BATTERY_325_DCM
	POWER_SUPPLY_PROP_BATTERY_CONDITION,
	POWER_SUPPLY_PROP_BATTERY_AGE,
#endif

};

static enum power_supply_property b325_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
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
	bool batteryPresent;
	bool batteryValid;
	bool chg_termination_flag;
	struct  timer_list  battery_poll_timer;
	uint		batt_status_poll_period;
	uint		batt_status_poll_period_normal;

	struct device *dev;

	struct mutex status_lock;
	spinlock_t queue_lock;

#ifdef CONFIG_BATTERY_325_DCM
	int batteryCurrent;
	int batteryAge;
#endif

} b325_battery_dev;


typedef struct battery_data {
	int		battery_voltage;		/* battery voltage (mV) */
	int		battery_temp;	/* temperature (degrees C) */
	int		battery_soc;	/* battery SOC (%) */

#ifdef CONFIG_BATTERY_325_DCM
	int 		battery_current; /* battery current (mA) */
	int		battery_age; /* battery age (%) */
#endif

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

#ifdef ACC_ENABLE
struct acc_data{

	bool charger_fixed;
	bool acc_started;
	bool pc_connected;
	u8	wait_cnt;
	u8	charger_step;
	u8	dpm_good_cnt;

};

typedef enum {
	Step_1_in_500mA_charging_550mA = 1, //USB default setting.
	Step_2_in_800mA_charging_625mA,
	Step_3_in_800mA_charging_700mA,
	Step_4_in_800mA_charging_775mA,
	Step_5_in_900mA_charging_850mA,
} Charger_Step;

static struct acc_data b325_acc_data;
extern bool usb_pc_connection;
#if 1 /*                                                    *//*                                                        */
extern bool hdmi_mhl_connection;
#endif
#endif

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

/** Extern functions **/
extern int max17047_get_battery_capacity_percent(void);
extern int max17047_get_battery_mvolts(void);
#ifdef CONFIG_BATTERY_325_DCM
extern int max17047_get_battery_current(void);
extern bool max17047_write_battery_temp(int battery_temp);
extern int max17047_get_battery_age(void);
extern int max17047_battery_exchange_program(void);
extern bool max17047_set_battery_atcmd(int flag, int value);
extern bool max17047_battery_full_info_print(void);
#endif

extern int max17040_get_battery_capacity_percent(void);
extern int max17040_get_battery_mvolts(void);
extern void max17040_set_battery_atcmd(int flag, int value);


extern int pm8058_batt_alarm_config(void);

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
extern bool pm8058_pwrkey_led_blink_set(uint led_set);
#endif

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

/** Extern variables **/
extern uint usb_base_reg;
extern int usb_chg_type_for_FG;

extern int usb_cable_info;

extern int lge_bd_rev;

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
extern u8 pwrkey_led_status;
#endif

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

	if (ret < 0)
		printk("[325_BAT]%s: err %d\n", __func__, ret);

	return ret;
}

static int charger_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

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

	c_func("[325_BAT] %s()\n", __func__);

	//mdelay(300);

	schedule_work(&battery_dev->update_charger_info_work);

	return IRQ_HANDLED;
}
#endif


bool b325_full_charge_error_handler(void)
{

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

	msleep(100);

	return 0;

}

bool b325_charger_temp_control(int bat_temp)
{
	u8 reg_val;

#ifdef OTP_DISABLE
	if (( temp_control == FALSE) && ((bat_temp > 45) || (bat_temp < 0)) )
	{
		printk(KERN_INFO "[325_BAT] %s() Temp control is disabled by Hidden menu setting. Always charging. \n", __func__);
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
		charging_state = TRUE;
		return 1;
	}
#endif

#ifdef CONFIG_BATTERY_325_DCM
	if ( (bat_temp > 55) || (bat_temp < 0) )
#else
		if ( (bat_temp > 45) || (bat_temp < 0) )
#endif
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
	return 1;
}

#ifdef ACC_ENABLE
bool b325_charging_step_control(u8 charger_step)
{

	printk("[325_BAT] b325_charging_step_control  charger_step = %d\n", charger_step);

	//Step1. Set the charger input current
	//Step2. Set the charging current

	switch (charger_step) {
		case Step_1_in_500mA_charging_550mA:
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x2C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x0);//Termniation 50mA
			break;
		case Step_2_in_800mA_charging_625mA:
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x3C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x8);//Termniation 50mA
			break;
		case Step_3_in_800mA_charging_700mA:
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x3C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x12);//Termniation 100mA
			break;
		case Step_4_in_800mA_charging_775mA:
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x3C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x1A);//Termniation 100mA
			break;
		case Step_5_in_900mA_charging_850mA:
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x4C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x22); //Termniation 100mA
			break;
		default:	//Input 500 mA, Charging 550 mA
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x2C);
			msleep(30);
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x0);
			break;
	}

	return TRUE;

}

bool b325_acc_control(void)
{
	u8 reg_read;
	u8 dpm_status;

	b325_acc_data.acc_started = TRUE;

	//Step1. Check PC connection flag
	if ( b325_acc_data.pc_connected == TRUE)
	{
		//printk("[325_BAT] b325_acc_control  PC connected. usb_pc_connection = %d\n", usb_pc_connection);
		return TRUE;
	}

	//Step2. Check DPM status
	reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_VINDPM_STAT);
	dpm_status = (reg_read & BQ24160_DPM_STAT_MASK) >> BQ24160_DPM_STAT_SHFT;

	if(dpm_status == 1) //DPM is active. Charger step down.
	{
		printk("[325_BAT] b325_acc_data charger_step = %d dpm_status = %d\n", b325_acc_data.charger_step, dpm_status);
		if (b325_acc_data.charger_step > 1)
		{
			b325_charging_step_control(b325_acc_data.charger_step - 1);
			b325_acc_data.charger_step = b325_acc_data.charger_step -1;
		}

		b325_acc_data.charger_fixed = TRUE;
	}
	//If the DPM is not active. ACC process keep going.

	//Step3. Wait count for PC connection time
	if( b325_acc_data.wait_cnt < 2)
	{
		b325_acc_data.wait_cnt++;
		return TRUE;
	}
	else if ( b325_acc_data.wait_cnt ==2)
		b325_acc_data.wait_cnt = 0;

	//Step4. PC or MHL connection check
#if 1 /*                                                    *//*                                                        */
	if( (usb_pc_connection == TRUE) || (hdmi_mhl_connection == TRUE) )
#else
	if( usb_pc_connection == TRUE)
#endif
	{
		b325_charging_step_control(Step_1_in_500mA_charging_550mA);
		b325_acc_data.pc_connected = TRUE;
		return TRUE;
	}


	//Step5. Check Charger fixed flag
	if (b325_acc_data.charger_fixed == TRUE)
	{
		if(b325_acc_data.dpm_good_cnt == 4)
		{
			b325_acc_data.charger_fixed = FALSE; //Reset charger fixed flag for recheck status.
			b325_acc_data.dpm_good_cnt = 0;
			return TRUE;
		}
		else
		{
			b325_acc_data.dpm_good_cnt++;
			return TRUE;
		}
	}

	//Step6. Charger set-up for new condition
	if ( b325_acc_data.charger_step == 0)
	{
		b325_charging_step_control(Step_1_in_500mA_charging_550mA);
		b325_acc_data.charger_step = Step_1_in_500mA_charging_550mA;
	}

	else if (b325_acc_data.charger_step == Step_5_in_900mA_charging_850mA)
	{
		b325_acc_data.charger_fixed = TRUE;
	}
	else
	{
		b325_acc_data.charger_step++; //Step up the charging current with adaptive control
		b325_charging_step_control(b325_acc_data.charger_step);
	}

	return TRUE;
}

#endif


static int batt_read_adc(int channel, int *mv_reading)
{
	int ret;
	void *h;
	struct adc_chan_result adc_chan_result;
	struct completion  conv_complete_evt;

	if (conv_first_request == TRUE)
	{
		pr_err("%s: msm_adc is not calibrated yet. \n",
				__func__);
		return 27;
	}

	pr_debug("%s: called for %d\n", __func__, channel);
	ret = adc_channel_open(channel, &h);
	if (ret) {
		pr_err("%s: couldnt open channel %d ret=%d\n",
				__func__, channel, ret);
		return 27;
		//goto out;
	}
	init_completion(&conv_complete_evt);
	ret = adc_channel_request_conv(h, &conv_complete_evt);
	if (ret) {
		pr_err("%s: couldnt request conv channel %d ret=%d\n",
				__func__, channel, ret);
		goto out;
	}

	wait_for_completion(&conv_complete_evt);

	ret = adc_channel_read_result(h, &adc_chan_result);
	if (ret) {
		pr_err("%s: couldnt read result channel %d ret=%d\n",
				__func__, channel, ret);
		goto out;
	}
	ret = adc_channel_close(h);
	if (ret) {
		pr_err("%s: couldnt close channel %d ret=%d\n",
				__func__, channel, ret);
	}
	if (mv_reading)
		*mv_reading = adc_chan_result.measurement;

	pr_debug("%s: done for %d\n", __func__, channel);
	return adc_chan_result.physical;
out:
	pr_debug("%s: done for %d\n", __func__, channel);
	return -EINVAL;

}


int b325_get_battery_temperature(void)
{
	int battery_temp;
	int battery_temp_ADC=0;

/*                                                                           */
#ifdef SKIP_INVALID_TEMP
#define MAX_INVALID_BATTERY_TEMP_COUNT	1
#define INVALID_BATTERY_TEMP_GAP		5
static uint prev_valid_battery_temp=27;
static uint invalid_battery_temp_count = 0;
#endif
/*                                                                         */

	if (power_supply_registered == FALSE)
	{
		battery_temp = 27;
		battery_temp_ADC=0;
		//printk("[325_BAT] Battery driver is not initialized yet. Temporary Temp = 25 deg C\n");
	}
	else
	{
		battery_temp= batt_read_adc(CHANNEL_ADC_BATT_THERM, &battery_temp_ADC);

/*                                                                           */
#ifdef SKIP_INVALID_TEMP
		if( prev_valid_battery_temp+INVALID_BATTERY_TEMP_GAP <= battery_temp
		  || prev_valid_battery_temp-INVALID_BATTERY_TEMP_GAP >= battery_temp )
		{
			invalid_battery_temp_count++;

			if( MAX_INVALID_BATTERY_TEMP_COUNT < invalid_battery_temp_count )
			{
				prev_valid_battery_temp = battery_temp;
				invalid_battery_temp_count = 0;
			}
			else
			{
				d_bat_ex("[325_BAT] %s() : skip temp %d[p:%d/c:%d]\n",
				   __func__, invalid_battery_temp_count, prev_valid_battery_temp, battery_temp );

				battery_temp = prev_valid_battery_temp;
			}
		}
		else
		{
			prev_valid_battery_temp = battery_temp;
			invalid_battery_temp_count = 0;
		}
#endif
/*                                                                         */
	}

	d_func("[325_BAT] %s() : Temp = %d ADC voltage = %d \n", __func__, battery_temp, battery_temp_ADC);

	battery_dev->batteryTemperature_adc = battery_temp_ADC;

	//Battery temp. range is from -40 degree to 99 degree.
	if ( battery_temp < -40 )
		battery_temp = -40; // -40 degree means no connection between NTC and Temp. ADC. (No battery)
	else if ( battery_temp > 99)
		battery_temp = 99;

#ifdef OTP_DISABLE
	if (temp_control == FALSE) //OTP default disable by HW request. DV test only.
	{
		if ( battery_temp > 59)
		{
			printk("[325_BAT] %s() OTP is disabled by DV test only. Original Temp = %d C\n", __func__, battery_temp);
			battery_temp = 59;
		}
	}
#endif

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
				battery->battery_voltage = max17040_get_battery_mvolts();
				msleep(200);
				battery->battery_soc = max17040_get_battery_capacity_percent();
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
			battery->battery_soc = max17040_get_battery_capacity_percent();
#else
			battery->battery_soc = max17047_get_battery_capacity_percent();
#endif
		}

	}
	else //Under Rev.D
	{
		battery->battery_voltage =  max17040_get_battery_mvolts();	
		battery->battery_soc = max17040_get_battery_capacity_percent();
	}

#ifdef CONFIG_BATTERY_325_DCM
	if(lge_bd_rev > LGE_REV_C)
	{
		battery->battery_current = max17047_get_battery_current();
		battery->battery_age = max17047_get_battery_age();
	}
#endif

	battery->battery_temp = b325_get_battery_temperature();

#ifdef CONFIG_BATTERY_325_DCM
	if(lge_bd_rev > LGE_REV_C)
		max17047_write_battery_temp(battery->battery_temp);
#endif

	battery->battery_present= b325_get_battery_present();

	battery->battery_valid= b325_get_battery_valid();

	d_bat("[325_BAT] %s() battery - %d%%, %dmV, %d'C  Present = %d  Valid = %d\n",__func__,
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

	battery_dev->batteryVoltage = battery.battery_voltage;
	battery_dev->batteryLifePercent= battery.battery_soc;
	battery_dev->batteryTemperature = battery.battery_temp;
	battery_dev->batteryTemperature *= 10;  // Android use x10 scale
	battery_dev->batteryPresent = battery.battery_present;
	battery_dev->batteryValid= battery.battery_valid;

#ifdef CONFIG_BATTERY_325_DCM
	battery_dev->batteryCurrent = battery.battery_current;
	battery_dev->batteryAge = battery.battery_age;
#endif

#ifdef LOCAL_WORK_FOR_325_BATTERY
	if (battery_dev->batteryLifePercent < 5)
		battery_dev->batt_status_poll_period = 10 * MSEC_PER_SEC;
	else
	{
		battery_dev->batt_status_poll_period = battery_dev->batt_status_poll_period_normal;
	}
#else

	//Battery Information Polling Timer set
	if (battery_dev->batteryLifePercent > 10)
		battery_dev->batt_status_poll_period = BATTERY_POLLING_INTERVAL;
	else
		battery_dev->batt_status_poll_period = BATTERY_POLLING_INTERVAL_LOW;
#endif

	//mutex_unlock(&status_lock);

// start junpyo.jeon 130130 add meminfo log
#if 1
        {
            extern int meminfo_proc_show_simple( void );
            meminfo_proc_show_simple();
        }
#endif
// end junpyo.jeon 130130 add meminfo log

	printk("[325_BAT] %s() battery updated - %d%%, %dmV, %d'C  Present = %d	Valid = %d\n",__func__,
			battery_dev->batteryLifePercent, battery_dev->batteryVoltage, battery_dev->batteryTemperature,
			battery_dev->batteryPresent, battery_dev->batteryValid);

	return 0;
}

static int b325_charger_control(bool charging_on_off)
{
	u8 fast_charge_current;

	if ( charging_on_off == TRUE ){
		at_cmd_force_charge_at56k = TRUE;
		charging_state = TRUE;

		fast_charge_current = 0x31;//Charging current set 1000mAh, Termination current 100mA
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


static int b325_charger_data(void)
{
	u8 chargerType;
	u8 fast_charge_current;
	u8 dc_input_limit = 0;	/*                                        */
	int bat_temp = 0;
	u8 reg_read = 0;
	u8 charger_stat = 0;

	c_func("[325_BAT] %s()\n", __func__);

	chargerType = Charger_Type_Battery; //Default value

	bat_temp = (battery_dev->batteryTemperature)/10;

	if ( usb_chg_type_for_FG == 2){ //TA
		chargerType = Charger_Type_AC;

		if (full_charge_error_flag == TRUE)
		{
			if ( battery_peak_voltage < 4180)
			{
				fast_charge_current = 0x0;//Charging current set 550mAh, Termination current 50mA
				charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);

				dc_input_limit = 0x94;//Regulation Votlage up to 4.24V
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);
			}
			else if ( battery_peak_voltage >= 4180)
			{
				fast_charge_current = 0x0;//Charging current set 550mAh, Termination current 50mA
				charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);

				dc_input_limit = 0x90;//Regulation Votlage up to 4.22V
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);
			}
		}

		if (b325_charger_temp_control(bat_temp) == 0)
			return chargerType;

		reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
		if ( reg_read < 0)
		{
			printk("charger_read_reg error, skip b325_charger_data one time. \n");
			return 2;
		}

		c_func("[325_BAT] BQ24160_REG_STAT_CTRL = %X \n", reg_read);

		charger_stat = (reg_read & BQ24160_STAT_MASK) >> BQ24160_STAT_SHFT;

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
		if ((lge_bd_rev >= LGE_REV_A) && (battery_check == TRUE))//Change the Input source from USB to IN port from Rev.D
		{
			reg_read = reg_read & 0xF7;
			charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, reg_read);
		}
#else
		reg_read = (reg_read | BQ24160_SUPPLY_SEL_MASK);
		charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, reg_read);	

		reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		reg_read = ((reg_read & 0x0F)| 0x40);
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);
#endif

		if ((charger_stat == 0x1)||(charger_stat == 0x2)||(charger_stat == 0x3)||(charger_stat == 0x4)	) //Charging
		{
			if ( b325_chg.wake_lock_flag == FALSE )
			{
				wake_lock(&b325_chg.chg_wake_lock);
				b325_chg.wake_lock_flag = TRUE;
			}
		}
		else if ( charger_stat == 0x5) // Charge Done
		{
			battery_peak_voltage = battery_dev->batteryVoltage;
			printk("[325_BAT] battery_peak_voltage = %d\n  ", battery_peak_voltage);

#ifdef CONFIG_BATTERY_325_DCM
			max17047_battery_full_info_print();
#endif

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

		if (full_charge_error_flag == FALSE)
		{
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
			dc_input_limit = 0x90;//Input current limit set 1500mAh
#endif

#ifdef CONFIG_BATTERY_325_DCM
			if ( (bat_temp > 44) && (bat_temp < 55) ) //DCM high temp charging support.
				fast_charge_current = 0x0;//Charging current set 550mAh, Termination current 50mA
			else
				fast_charge_current = 0x21;//Charging current set 850mAh, Termination current 100mA
#else
			fast_charge_current = 0x21;//Charging current set 850mAh, Termination current 100mA
#endif
		}

		c_func("[325_BAT] %s() dc_input_limit = %X, fast_charge_current = %X \n", __func__, dc_input_limit, fast_charge_current);

		//Read 0x03, 0x05 register. Then check the setting. If the setting doesn't need to update, skip write.
		//--------need to update.
		charger_write_reg(battery_dev->client, BQ24160_REG_VINDPM_STAT, 0xCB);

		if (full_charge_error_flag == FALSE)
		{
			charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);
#endif
		}

		msleep(10);

		charger_fault_check(battery_dev->client);

	}
	else if ( usb_chg_type_for_FG == 3) { //USB
		chargerType = Charger_Type_USB;

		fast_charge_current = 0x0;//Charging current set 550mAh, Termination current 50mA

		c_func("[325_BAT] %s() dc_input_limit = %X, fast_charge_current = %X \n", __func__, dc_input_limit, fast_charge_current);

		//Read 0x03, 0x05 register. Then check the setting. If the setting doesn't need to update, skip write.
		//--------need to update.
		if ((usb_cable_info == 7) || (usb_cable_info == 11)){

			if ((lge_bd_rev >= LGE_REV_A) && (battery_check == TRUE))//Change the Input source from USB to IN port from Rev.D & Factory support.
			{
				reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
				if ( reg_read < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 1;
				}
				reg_read = reg_read & 0xF7;
				charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, reg_read);
			}

			if(at_cmd_force_charge_at56k==FALSE){	// When at%charge = 1 is not recived, discharing at factory cable
				if(charging_state == TRUE)
				{
					charging_state = FALSE;
					//b325_charger_termination_control(1);

					reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
					if ( reg_read < 0)
					{
						printk("charger_read_reg error, skip b325_charger_data one time. \n");
						return 1;
					}
					reg_read = reg_read | 0x2; //Disable CE.
					reg_read = reg_read & 0x7F;
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);

					printk("[325_BAT] Factory USB mode usb_cable_info = %d \n", usb_cable_info);
				}
			}else{					//When at%charge = 1 is recived, Charing at factory cable
				charging_state = TRUE;
				fast_charge_current = 0x31;//Charging current set 1000mAh, Termination current 100mA
				charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, fast_charge_current);
				printk("[325_BAT] at%%charge = 1 is recived, charging \n");
			}
			dc_input_limit = 0x8E;//Input current limit set 2500mAh
			charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);

		}
		else //Original USB connection part.
		{
			//Disable charging control from temperature
			if (b325_charger_temp_control(bat_temp) == 0)
				return chargerType;

			if ((lge_bd_rev >= LGE_REV_A) && (battery_check == TRUE))//Change the Input source to USB from Rev.D.
			{

				reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
				if ( reg_read < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 1;
				}
				if (usb_cable_info != 6) {
					reg_read = (reg_read | BQ24160_SUPPLY_SEL_MASK);
					charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, reg_read);
				}

				dc_input_limit = 0x90;//Battery Regulation Voltage setting to 4.2V
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL_BATVOLT, dc_input_limit);

				charger_write_reg(battery_dev->client, BQ24160_REG_VINDPM_STAT, 0xD0);

#ifdef ACC_ENABLE
				if(usb_cable_info == 6) //56K cable
				{
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x4C);
					charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x0);
				}
				else // Call ACC function
				{
					if(b325_acc_control() == FALSE)
						printk("[325_BAT] ACC setting error. Try next turn again \n");
				}
#else

				reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
				if ( reg_read < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 1;
				}
				if(usb_cable_info == 6) //56K cable
					reg_read = ((reg_read & 0x0F)| 0x40);	//USB current limit set 900mA
				else			//Normal USB
					reg_read = ((reg_read & 0x0F)| 0x20);
				charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);
				charger_write_reg(battery_dev->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x0);
#endif
			}
			else{
				if(usb_cable_info == 6) { //If there is no battery, Stop charging.
					reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
					reg_read = reg_read | 0x2; //Disable CE.
					reg_read = reg_read & 0x7F;
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_read);
				}
			}
		}
		msleep(10);
		charger_fault_check(battery_dev->client);
	}
	else
	{
		chargerType = Charger_Type_Battery;

		if (charging_state == TRUE) //Charger setting change to Default mode.
		{
			if ((lge_bd_rev >= LGE_REV_B) && (battery_check == TRUE))//Change the Input source to USB from Rev.D. by default
			{
				reg_read = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
				if ( reg_read < 0)
				{
					printk("charger_read_reg error, skip b325_charger_data one time. \n");
					return 1;
				}if (usb_cable_info == 7)
				{
					printk("######## usb_cable_info = %d \n", usb_cable_info);
				}
				else if ((usb_cable_info != 6)&&(battery_dev->batteryVoltage > 3300)) //Blocking Port swtiching with 56K cable & Extremely Low battery
				{
					reg_read = (reg_read | BQ24160_SUPPLY_SEL_MASK);
					charger_write_reg(battery_dev->client, BQ24160_REG_STAT_CTRL, reg_read);
				}
				else {
					charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, 0x2C);
				}
#ifdef ACC_ENABLE //ACC (Adaptive Charging-current contol)
				//Default set the ACC varibales.
				if ( b325_acc_data.acc_started == TRUE)
				{
					b325_acc_data.charger_fixed = FALSE;
					b325_acc_data.charger_step = 0;
					b325_acc_data.wait_cnt = 0;
					b325_acc_data.pc_connected = FALSE;
					b325_acc_data.dpm_good_cnt = 0;
					b325_acc_data.acc_started = FALSE;

					usb_pc_connection = FALSE;
#if 1 /*                                                    *//*                                                        */
					hdmi_mhl_connection = FALSE;
#endif
				}
#endif

			}
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

	d_bat_ex("[325_BAT] %s() Disconn=0, etc=1, TA=2, USB=3	  usb_chg_type_for_FG = %d  charging state = %d \n",__func__, usb_chg_type_for_FG, charging_state);

	return chargerType;
}



static int b325_power_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	d_func("[325_BAT] %s() psp:%d usb_chg_type_for_FG = %d \n", __func__, psp, usb_chg_type_for_FG);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS)
				val->intval = (usb_chg_type_for_FG == 2); /* not fixed */
			if (psy->type == POWER_SUPPLY_TYPE_USB)
				val->intval = (usb_chg_type_for_FG == 3); /* not fixed */
			break;
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
					if (battery_dev->batteryLifePercent == 100)
					{
						val->intval = POWER_SUPPLY_STATUS_FULL;
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
						if(pwrkey_led_status == 2)
						{
							pm8058_pwrkey_led_blink_set(0); //LED off for the Full charging
						}
#endif
					}
					else
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
				else
				{
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
					if(pwrkey_led_status !=0)
					{
						pm8058_pwrkey_led_blink_set(0); //Turn Off LED.
					}
#endif
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
			if (!battery_dev->batteryPresent)
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				return 0;
			}
			if (psp == POWER_SUPPLY_PROP_CAPACITY)
			{
				val->intval = battery_dev->batteryLifePercent;

			} else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
				val->intval = battery_dev->batteryVoltage 
				  * BATTERY_LEVEL_MULTIPLIER  /*                                                      */
				; 
			}

			if (psp == POWER_SUPPLY_PROP_TEMP)
				val->intval = battery_dev->batteryTemperature;

			if (psp == POWER_SUPPLY_PROP_BATTERY_TEMP_ADC)
				val->intval = battery_dev->batteryTemperature_adc;
			break;

		case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
			val->intval = b325_get_battery_valid();
			break;

#ifdef CONFIG_BATTERY_325_DCM
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = battery_dev->batteryCurrent ; //Battery Current (mA)
			break;

		case POWER_SUPPLY_PROP_BATTERY_CONDITION:
			if ( battery_dev->batteryAge == 999)
				val->intval = 0; //Error or Uncalculated Battery age.
			else if (battery_dev->batteryAge >= 80)
				val->intval = 1; //Very Good Condition
			else if (battery_dev->batteryAge >= 50)
				val->intval = 2; //Good Condition
			else if (battery_dev->batteryAge >= 0)
				val->intval = 3; //Bad Condition
			else
				val->intval = 0; //Uncalculated Battery age.
			break;

		case POWER_SUPPLY_PROP_BATTERY_AGE:
			val->intval = battery_dev->batteryAge;
			break;
#endif
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

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
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
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
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

	int ret;

	c_func("[325_BAT] %s() \n", __func__);

	msleep(30); //Before check the charger register, wait 30ms.

	//Read 0x00 charger register
	ret = charger_read_reg(battery_dev->client, BQ24160_REG_STAT_CTRL);
	c_func("[325_BAT] BQ24160_REG_STAT_CTRL = %X \n", ret);

	//Read 0x01 charger register
	ret = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);
	c_func("[325_BAT] BQ24160_REG_BATTNPS_STAT = %X \n", ret);

	//If the charger is in Fault mode, let user know the fault.
	//----------need to update

	//charger_read_reg_all(battery_dev->client);

	b325_charger_data();

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
#ifdef CONFIG_BATTERY_325_DCM
	bool ret = 0;
	ret = max17047_set_battery_atcmd(0, 100);  // Reset the fuel guage IC
	if ( ret == 1)
		printk("at_fuel_guage_reset_show error.\n");
#else
	max17040_set_battery_atcmd(0, 100);  // Reset the fuel guage IC
#endif

	r = sprintf(buf, "%d\n", true);

	at_cmd_force_control = TRUE;

#ifdef CONFIG_BATTERY_325_DCM
	msleep(100);
#else
	msleep(300);
#endif

#ifdef CONFIG_BATTERY_325_DCM
	//max17047_set_battery_atcmd(1, 100);  // Reset the fuel guage IC
#else
	b325_battery_data();

	max17040_set_battery_atcmd(2, 100);  // Release the AT command mode
#endif

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

#ifdef CONFIG_BATTERY_325_DCM
		//Battery exchange program call
		max17047_battery_exchange_program();
#endif

	}
	else if(!strncmp(string, "1", 1))
	{
		/* Temp control enable - Charging controlled by Temp. */
		//printk("[325_BAT] %s()  Temp control enable  \n", __func__);
		temp_control = TRUE;
	}
	return count;
}


DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_status_complete_show, at_chg_status_complete_store);
DEVICE_ATTR(at_fuelrst, 0644, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0644, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_pmrst, 0644, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_batl, 0644, at_batt_level_show, NULL);
DEVICE_ATTR(at_tempctrl, 0660, at_tempctrl_show, at_tempctrl_store);
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
		default:	break;
	}

	//	printk("[325_BAT]%s external_power_check dc_check = %d \n", __func__, dc_check);

	return dc_check;
}
EXPORT_SYMBOL(is_chg_plugged_in);
#endif

bool is_bat_connected_in(void)
{

	u8 reg_val;
	u8 bat_check;
	u8 en_nobatop;

	c_func("[325_BAT] %s()\n", __func__);

	reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT);

	bat_check = (reg_val & BQ24160_BATTSTAT_MASK) >> BQ24160_BATTSTAT_SHFT;

	en_nobatop = reg_val | 0x1;

	c_func("[325_BAT]%s is_bat_connected_in bat_check = %d en_nobatop = %x\n", __func__, bat_check, en_nobatop);

	if ( bat_check == 2)
	{
		//charger_write_reg(battery_dev->client, BQ24160_REG_BATTNPS_STAT, en_nobatop);
		reg_val = charger_read_reg(battery_dev->client, BQ24160_REG_CTRL);
		c_func("[325_BAT]%s BQ24160_REG_CTRL  = %X \n", __func__, reg_val);
		reg_val = reg_val | 0x6;
		reg_val = reg_val & 0x7F;
		charger_write_reg(battery_dev->client, BQ24160_REG_CTRL, reg_val);
		battery_check = FALSE;
		return 0; // No battery
	}
	else
	{
		battery_check = TRUE;
		return 1; // Battery connected
	}

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

	ret = request_threaded_irq(battery_dev->client->irq, NULL, charger_handler,
			IRQF_TRIGGER_FALLING, "max8971-irq", battery_dev);

	if (unlikely(ret < 0)){
		printk("[325_BAT]max8971: failed to request IRQ %X\n", ret);
	}

	printk(KERN_INFO "[325_BAT] %s: charger interrupt registered\n", battery_dev->client->name);
#endif

	if ((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11))
	{
		charging_state = TRUE;
	}

	//Charging LED init
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)
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
#ifdef OTP_DISABLE
	temp_control = FALSE; //DV Test only.
#else
	temp_control = TRUE;
#endif

#ifdef ACC_ENABLE //ACC (Adaptive Charging-current contol)
	//Initial the ACC varibales.
	b325_acc_data.charger_fixed = FALSE;
	b325_acc_data.charger_step = 0;
	b325_acc_data.wait_cnt = 0;
	b325_acc_data.pc_connected = FALSE;
	b325_acc_data.dpm_good_cnt = 0;
	b325_acc_data.acc_started = FALSE;

#endif


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
