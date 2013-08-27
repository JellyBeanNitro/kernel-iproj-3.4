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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/msm-charger.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
#include <linux/pmic8058-charger.h>
#include <linux/mfd/pm8xxx/batt-alarm.h> 
#endif

#include <asm/atomic.h>

#include <mach/msm_hsusb.h>

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include <linux/msm_adc.h>
#include "../../lge/include/lg_power_common.h"
#endif
#ifdef CONFIG_LGE_PM_TEMPERATURE_MONITOR
#include <linux/android_alarm.h>
#endif


#define MSM_CHG_MAX_EVENTS		16
#define CHARGING_TEOC_MS		9000000
#ifdef CONFIG_MACH_LGE_120_BOARD //                                                 
#define UPDATE_TIME_MS			30000
#else
#define UPDATE_TIME_MS			60000
#endif
#define RESUME_CHECK_PERIOD_MS		60000

#define DEFAULT_BATT_MAX_V		4200
#define DEFAULT_BATT_MIN_V		3200

#define MSM_CHARGER_GAUGE_MISSING_VOLTS 3500
#define MSM_CHARGER_GAUGE_MISSING_TEMP  35
#if defined(CONFIG_MACH_LGE_I_BOARD_SKT)
//ACC(Adaptive Charging-current Contorl)
#define ACC_ENABLE

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#endif
/**
 * enum msm_battery_status
 * @BATT_STATUS_ABSENT: battery not present
 * @BATT_STATUS_ID_INVALID: battery present but the id is invalid
 * @BATT_STATUS_DISCHARGING: battery is present and is discharging
 * @BATT_STATUS_TRKL_CHARGING: battery is being trickle charged
 * @BATT_STATUS_FAST_CHARGING: battery is being fast charged
 * @BATT_STATUS_JUST_FINISHED_CHARGING: just finished charging,
 *		battery is fully charged. Do not begin charging untill the
 *		voltage falls below a threshold to avoid overcharging
 * @BATT_STATUS_TEMPERATURE_OUT_OF_RANGE: battery present,
					no charging, temp is hot/cold
 */
enum msm_battery_status {
	BATT_STATUS_ABSENT,
	BATT_STATUS_ID_INVALID,
	BATT_STATUS_DISCHARGING,
	BATT_STATUS_TRKL_CHARGING,
	BATT_STATUS_FAST_CHARGING,
	BATT_STATUS_JUST_FINISHED_CHARGING,
	BATT_STATUS_TEMPERATURE_OUT_OF_RANGE,
};


/*                                     */
#ifdef CONFIG_LGE_PM_TEMPERATURE_MONITOR
/* When we're awake or running on wall power, sample the battery
 * gauge every FAST_POLL seconds.  If we're asleep and on battery
 * power, sample every SLOW_POLL seconds
 */
#define TIME_POLL_0P5_MINUTE (1 * 30)
#define TIME_POLL_1_MINUTE	(1 * 60)
#define TIME_POLL_10_MINUTE (10 * 60)
#define TIME_POLL_30_MINUTE (30 * 60)
#define TIME_POLL_60_MINUTE (60 * 60)
#define TIME_POLL_90_MINUTE (90 * 60)
#endif
/*                                     */

#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
//                                                                         
#define FAKE_BATT_ENABLED 1
#define FAKE_BATT_DISABLED 0
//                                                                                                                                                
#define FAKE_BATT_DISABLE_TEMP 68
#endif

#ifdef CONFIG_LGE_TOUCH_CHAGER_CTRL
#include "../../include/linux/input/touch_synaptics.h"
extern void trigger_baseline_state_machine(int plug_in);
#endif
#ifdef CONFIG_LGE_FUEL_GAUGE
#define LGE_DEBUG
#if !defined(CONFIG_MACH_LGE_I_BOARD_DCM)
extern int usb_chg_type_for_FG;
#else
extern int usb_chg_type;
#endif

extern int max17040_get_battery_capacity_percent(void);
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
extern int threshold_mv;
#endif
#endif

#ifdef CONFIG_LGE_BACKLIGHT_LM3530
//                                                                           
extern int lm3530_backlight_status(void);
//                                                                           
#endif

struct msm_hardware_charger_priv {
	struct list_head list;
	struct msm_hardware_charger *hw_chg;
	enum msm_hardware_charger_state hw_chg_state;
	unsigned int max_source_current;
	struct power_supply psy;
};

struct msm_charger_event {
	enum msm_hardware_charger_event event;
	struct msm_hardware_charger *hw_chg;
};

struct msm_charger_mux {
	int inited;
	struct list_head msm_hardware_chargers;
	int count_chargers;
	struct mutex msm_hardware_chargers_lock;

	struct device *dev;

	unsigned int max_voltage;
	unsigned int min_voltage;

#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
// sujin.shin [TEMP_CONTROL]
	u8 temp_control;
#endif	
	int temperature;

	unsigned int safety_time;
	struct delayed_work teoc_work;

	unsigned int update_time;
	unsigned int processing_delay; /*                                   */
	int stop_update;
	struct delayed_work update_heartbeat_work;
	struct delayed_work update_batt_remove_work;  /*                                   */

	struct mutex status_lock;
	enum msm_battery_status batt_status;
	struct msm_hardware_charger_priv *current_chg_priv;
	struct msm_hardware_charger_priv *current_mon_priv;

	unsigned int (*get_batt_capacity_percent) (void);

	struct msm_charger_event *queue;
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
	struct wake_lock wl;

/*                                     */
#ifdef CONFIG_LGE_PM_TEMPERATURE_MONITOR
  u8 slow_poll;
  ktime_t last_poll;

  struct wake_lock temp_wake_lock;
	struct delayed_work monitor_work;
  struct alarm alarm;
#endif
//                                         
	int lcd_on_charge_curr_status;
	int capacity;
	int chg_current_set;
//                                       

/*                                     */
};

static struct msm_charger_mux msm_chg;

static struct msm_battery_gauge *msm_batt_gauge;

#ifdef CONFIG_LGE_PM
static struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
//sujin.shin [TEMP_CONTROL]
	static struct pseudo_batt_info_type booting_set = {
	.mode = 1,
	.id = 1,
	.therm = 1027,
	.temp = 35,
	.volt = 4100,
	.capacity = 80,
	.charging = 1,
	};
#endif
static int block_charging_state = 1; /* 1 : charging , 0: block charging */

static int charging_flow_monitor_enable = 1; /* for debugging log */
struct wake_lock uevent_wake_lock; 
struct wake_lock adc_wake_lock;
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static int last_stop_charging = 0;
/*                                                                                                    */
static int pseudo_ui_charging = 0;

static int chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

extern const struct adc_map_pt adcmap_batttherm[THERM_LAST];
#ifdef CONFIG_LGE_PM_TA_COMPENSATION
extern const struct adc_map_pt adcmap_batttherm_ta[THERM_LAST];
#endif
extern acc_cable_type get_ext_cable_type_value(void);
extern int pm_chg_imaxsel_set(int chg_current);
extern int pm_chg_auto_disable(int value);
#endif

#if defined(CONFIG_LGE_PM_CAYMAN_VZW) || defined(CONFIG_LGE_PM_CAYMAN_MPCS)
static bool b_is_testmode_cmd_on = false;
extern int pm8058_start_charging_for_TESTMODE(void);
extern int pm8058_stop_charging_for_TESTMODE(void);
#endif /*                                                           */

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
static bool b_is_at_cmd_on = false;

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
extern void bq24160_at_cmd_chg_set(bool set_value);
extern void bq24160_set_chg_current(int ichg);
extern void bq24160_determine_the_collect_chg(int start);
#endif
#endif
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
extern int bq24160_charger_get_status(void);
#endif
#ifdef ACC_ENABLE
struct acc_data{
	bool charger_fixed;
	u8	wait_cnt;
	u8	charger_step;
	u8	vchg_good_cnt;
};

typedef enum {
    Step_1_charging_450mA = 1, /* Default USB charging Current */
    Step_2_charging_535mA,
    Step_3_charging_620mA,
    Step_4_charging_700mA,
} Charger_Step;

static struct acc_data f120_acc_data;

extern bool usb_pc_connection;

#endif

static int is_chg_capable_of_charging(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg_state == CHG_READY_STATE
	    || priv->hw_chg_state == CHG_CHARGING_STATE)
		return 1;

	return 0;
}

static int is_batt_status_capable_of_charging(void)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  if(b_is_at_cmd_on)
    return 1;
  else
  {
    if (msm_chg.batt_status == BATT_STATUS_ABSENT
	    || msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
	    || msm_chg.batt_status == BATT_STATUS_ID_INVALID
	    || msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING)
		  return 0;
	  return 1;
  }
#else
	if (msm_chg.batt_status == BATT_STATUS_ABSENT
	    || msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
	    || msm_chg.batt_status == BATT_STATUS_ID_INVALID
	    || msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING)
		return 0;
	return 1;
#endif
}

static int is_batt_status_charging(void)
{
	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING
	    || msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		return 1;
	return 0;
}

static int is_battery_present(void)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  if(b_is_at_cmd_on)
    return 1;
  else
  {
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	/*                                                                   */
	if((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
	  return 1;
	else
#endif
    if (msm_batt_gauge && msm_batt_gauge->is_battery_present)
		  return msm_batt_gauge->is_battery_present();
	  else {
		  pr_err("msm-charger: no batt gauge batt=absent\n");
		  return 0;
	  }
  }
#else
	if (msm_batt_gauge && msm_batt_gauge->is_battery_present)
		return msm_batt_gauge->is_battery_present();
	else {
		pr_err("msm-charger: no batt gauge batt=absent\n");
		return 0;
	}
#endif
}

static int is_battery_temp_within_range(void)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  if(b_is_at_cmd_on)
    return 1;
  else
  {
    if (msm_batt_gauge && msm_batt_gauge->is_battery_temp_within_range)
		  return msm_batt_gauge->is_battery_temp_within_range();
	  else {
		  pr_err("msm-charger no batt gauge batt=out_of_temperatur\n");
		  return 0;
	  }
  }
#else
	if (msm_batt_gauge && msm_batt_gauge->is_battery_temp_within_range)
		return msm_batt_gauge->is_battery_temp_within_range();
	else {
		pr_err("msm-charger no batt gauge batt=out_of_temperatur\n");
		return 0;
	}
#endif
}

static int is_battery_id_valid(void)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  if(b_is_at_cmd_on)
    return 1;
  else
  {
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	/*                                                                   */
	if((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
		return 1;
	else
#endif
    if (msm_batt_gauge && msm_batt_gauge->is_battery_id_valid)
		  return msm_batt_gauge->is_battery_id_valid();
	  else {
		  pr_err("msm-charger no batt gauge batt=id_invalid\n");
		  return 0;
	  }
  }
#else
	if (msm_batt_gauge && msm_batt_gauge->is_battery_id_valid)
		return msm_batt_gauge->is_battery_id_valid();
	else {
		pr_err("msm-charger no batt gauge batt=id_invalid\n");
		return 0;
	}
#endif
}

static int get_prop_battery_mvolts(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_mvolts)
		return msm_batt_gauge->get_battery_mvolts();
	else {
		pr_err("msm-charger no batt gauge assuming 3.5V\n");
		return MSM_CHARGER_GAUGE_MISSING_VOLTS;
	}
}

static int get_battery_temperature(void)
{
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
		return MSM_CHARGER_GAUGE_MISSING_TEMP;
	else
#endif
	if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature)
		return msm_batt_gauge->get_battery_temperature();
	else {
		pr_err("msm-charger no batt gauge assuming 35 deg G\n");
		return MSM_CHARGER_GAUGE_MISSING_TEMP;
	}
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#if !defined(CONFIG_MACH_LGE_I_BOARD_DCM)
static int get_battery_temperature_adc(void)
{
	if((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
		return 1000;
	else
        if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature_adc)
                return msm_batt_gauge->get_battery_temperature_adc();
        else {
                pr_err("msm-charger no batt gauge and no temp adc\n");
                return 1000;
        }
}
#else
int get_battery_temperature_adc(void)
{
	if((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value()) ||(11 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
		return 1000;
	else
	if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature_adc)
		return msm_batt_gauge->get_battery_temperature_adc();
	else {
		pr_err("msm-charger no batt gauge and no temp adc\n");
		return 1000;
	}
}
EXPORT_SYMBOL(get_battery_temperature_adc);
#endif
#endif
#if defined(CONFIG_LGE_PM_TA_COMPENSATION) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
extern int is_chg_plugged_in(void);
#endif

/*                                                                                  */
#if defined(CONFIG_MACH_LGE_I_BOARD_VZW) || defined(CONFIG_MACH_LGE_C1_BOARD_MPS)
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
int is_temp_too_cold(void)
{
		int chg_batt_temp;
		int rtnValue = 0;
		int temp_adc;
		temp_adc = get_battery_temperature_adc();
	
		printk("%s: START \n",__func__);
		
		if(temp_adc < adcmap_batttherm[THERM_55].x)
		  chg_batt_temp = CHG_BATT_TEMP_OVER_55;
		else if(temp_adc < adcmap_batttherm[THERM_45].x)
		  chg_batt_temp = CHG_BATT_TEMP_46_55;
		else if(temp_adc <= adcmap_batttherm[THERM_42].x)
		  chg_batt_temp = CHG_BATT_TEMP_42_45;
		else if(temp_adc < adcmap_batttherm[THERM_M5].x)
		  chg_batt_temp = CHG_BATT_TEMP_M4_41;
		else if(temp_adc <= adcmap_batttherm[THERM_M10].x)
		  chg_batt_temp = CHG_BATT_TEMP_M10_M5;
		else
		  chg_batt_temp = CHG_BATT_TEMP_UNDER_M10;
#ifdef CONFIG_LGE_PM_TA_COMPENSATION
		if (is_chg_plugged_in())
		{
			if(temp_adc < adcmap_batttherm_ta[THERM_55].x)
			  chg_batt_temp = CHG_BATT_TEMP_OVER_55;
			else if(temp_adc < adcmap_batttherm_ta[THERM_45].x)
			  chg_batt_temp = CHG_BATT_TEMP_46_55;
			else if(temp_adc <= adcmap_batttherm_ta[THERM_42].x)
			  chg_batt_temp = CHG_BATT_TEMP_42_45;
			else if(temp_adc < adcmap_batttherm_ta[THERM_M5].x)
			  chg_batt_temp = CHG_BATT_TEMP_M4_41;
			else if(temp_adc <= adcmap_batttherm_ta[THERM_M10].x)
			  chg_batt_temp = CHG_BATT_TEMP_M10_M5;
			else
			  chg_batt_temp = CHG_BATT_TEMP_UNDER_M10;
		}	
#endif
		if(chg_batt_temp == CHG_BATT_TEMP_M10_M5 || chg_batt_temp == CHG_BATT_TEMP_UNDER_M10)
			rtnValue = 1;

		return rtnValue;	
}
#endif
#endif

static int get_prop_batt_capacity(void)
{
	int capacity;

	if (msm_batt_gauge && msm_batt_gauge->get_batt_remaining_capacity)
		capacity = msm_batt_gauge->get_batt_remaining_capacity();
	else
		capacity = msm_chg.get_batt_capacity_percent();

	if (capacity <= 10)
		pr_err("battery capacity very low = %d\n", capacity);

	return capacity;
}

int is_batt_lvl_present(void)
{
	return(get_prop_batt_capacity());   
}

static int get_prop_batt_health(void)
{
	int status = 0;

	if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;
}

static int get_prop_charge_type(void)
{
	int status = 0;

	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
	else
		status = POWER_SUPPLY_CHARGE_TYPE_NONE;

	return status;
}

#if !defined(CONFIG_MACH_LGE_I_BOARD_DCM) && !defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
static int get_prop_batt_status(void)
{
	int status = 0;

	if (msm_batt_gauge && msm_batt_gauge->get_battery_status) {
		status = msm_batt_gauge->get_battery_status();
		if (status == POWER_SUPPLY_STATUS_CHARGING ||
			status == POWER_SUPPLY_STATUS_FULL ||
			status == POWER_SUPPLY_STATUS_DISCHARGING)
			return status;
	}

	if (is_batt_status_charging())
#ifdef CONFIG_LGE_FUEL_GAUGE
	{
		if(get_prop_batt_capacity() >= 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}
#else
		status = POWER_SUPPLY_STATUS_CHARGING;
#endif
	else if (msm_chg.batt_status ==
		 BATT_STATUS_JUST_FINISHED_CHARGING
			 && msm_chg.current_chg_priv != NULL
			 &&(get_prop_batt_capacity() >= 95))
		status = POWER_SUPPLY_STATUS_FULL;
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	/* in case of booting with TA without battery, the battery icon set'*/
	else if (msm_chg.batt_status == BATT_STATUS_ABSENT)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
#endif
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	return status;
}
#endif

 /* This function should only be called within handle_event or resume */
static void update_batt_status(void)
{
	if (is_battery_present()) {
		if (is_battery_id_valid()) {
			if (msm_chg.batt_status == BATT_STATUS_ABSENT
				|| msm_chg.batt_status
					== BATT_STATUS_ID_INVALID) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
		} else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
	 } else
		msm_chg.batt_status = BATT_STATUS_ABSENT;
}

static enum power_supply_property msm_power_props[] = {
#ifndef CONFIG_LGE_FUEL_GAUGE
	POWER_SUPPLY_PROP_PRESENT,
#endif
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
int prev_cable_type = 0;
#endif

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
#ifdef CONFIG_LGE_FUEL_GAUGE
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
    {
      int bq24160_driver = -1;
      
      bq24160_driver = bq24160_charger_get_status();

      if(bq24160_driver == POWER_SUPPLY_STATUS_CHARGING || bq24160_driver == POWER_SUPPLY_STATUS_FULL)
      {
        if(usb_chg_type != 0)
        {
          prev_cable_type = usb_chg_type;
        }
      }
      else
      {
        prev_cable_type = usb_chg_type;
      }

      if(get_ext_cable_type_value() == 0)
      {
        val->intval = 0;
      }
      else
      {
        if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			    val->intval = (prev_cable_type == 2); /* not fixed */
		    if (psy->type == POWER_SUPPLY_TYPE_USB)
			    val->intval = (prev_cable_type == 3); /* not fixed */
      }
    }
#else
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (usb_chg_type_for_FG == 2); /* not fixed */
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (usb_chg_type_for_FG == 3); /* not fixed */
#endif
		break;
	default:
			return -EINVAL;
	}
	return 0;
#else
	struct msm_hardware_charger_priv *priv;

	priv = container_of(psy, struct msm_hardware_charger_priv, psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(priv->hw_chg_state == CHG_ABSENT_STATE);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (priv->hw_chg_state == CHG_READY_STATE)
			|| (priv->hw_chg_state == CHG_CHARGING_STATE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
#endif
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	POWER_SUPPLY_PROP_TEMP,
#endif
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	POWER_SUPPLY_PROP_BATTERY_TEMP_ADC,
#endif
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_PSEUDO_BATT,
	POWER_SUPPLY_PROP_BLOCK_CHARGING,	
	POWER_SUPPLY_PROP_EXT_PWR_CHECK, // for auto run, it will check just cable is usb or not.
        POWER_SUPPLY_PROP_FACTORY_MODE,
	POWER_SUPPLY_PROP_CHARGING_CURRENT_SET, //                                   
        
        POWER_SUPPLY_PROP_SMPL_MODE,
#if 0	
	POWER_SUPPLY_PROP_WLC_STATUS,
#endif	
#endif
};

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
extern uint16_t battery_info_get(void);
#endif
#ifdef CONFIG_LGE_PM_FACTORY_CURRENT_DOWN
extern int usb_cable_info;
#endif
extern unsigned int smpl_on;

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
extern int batt_volt;
extern int batt_temp;
extern int fuel_percent;
extern bool chg_complete;
extern bool chg_resume;
extern bool bq24160_chg_done_once;
extern int now_chg_state;
#endif

#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
extern int set_status_power(int val);
extern int get_status_power(void);
#endif
static int msm_batt_power_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	//printk("%s, \n", __func__);
	switch (psp) {
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		return 1;
#endif
#ifdef CONFIG_LGE_PM	
	//                                         
	case POWER_SUPPLY_PROP_CHARGING_CURRENT_SET:
		return 1;
	//                                       
#endif	
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(msm_batt_power_property_is_writeable);

static int msm_batt_power_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
	char set_val;
	int retval = 0;
#endif
	
	switch (psp) {		
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		printk("!!!!%s, PSEUDO_BATT = %d\n", __func__, val->intval);
		if(val->intval == 1){
			msm_chg.temp_control = val->intval;
			set_val = (char)msm_chg.temp_control;
			retval = set_status_power(set_val);
			if (retval != 0)
				printk("!!!!!%s, set_status_power is failed!!\n", __func__);
			}
		else if (val->intval == 0){
			msm_chg.temp_control = val->intval;
			set_val = (char)msm_chg.temp_control;
			retval = set_status_power(set_val);
			if (retval != 0)
				printk("!!!!!%s, set_status_power is failed!!\n", __func__);
			}
		break;
#endif	
#ifdef CONFIG_LGE_PM			
//                                         
	case POWER_SUPPLY_PROP_CHARGING_CURRENT_SET:
		printk("!!!!%s, CHARGING_CURRENT_SET = %d\n", __func__, val->intval);
		if (val->intval == 1){
			msm_chg.chg_current_set = val->intval;
			printk("Set OVERHEAT fake Charging mode...\n");
		}
		else if(val->intval == 0) { 
			msm_chg.chg_current_set = val->intval;
			printk("Set NORMAL Charging mode...\n");
		}			
		break;
//                                       
#endif		
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(msm_batt_power_set_property);


static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
	int ext_cable_type;
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
#ifdef CONFIG_LGE_PM
/* we comment out below */
/*		if(pseudo_batt_info.mode == 1)
  			val->intval = pseudo_batt_info.charging;
		else
*/		
		if(block_charging_state == 0) // in hidden menu, charging stop. we show animation stop.
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
#endif

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
    if(get_ext_cable_type_value() == 0)
    {
      now_chg_state = 0;
    }
    
    val->intval = bq24160_charger_get_status();
#else
		val->intval = get_prop_batt_status();
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#ifdef CONFIG_LGE_PM
		if(pseudo_batt_info.mode == 1)
		{
			if(pseudo_batt_info.id == 1 || pseudo_batt_info.therm != 0)
				val->intval = 1;
			else
				val->intval = 0;
		}
		else
#endif
		val->intval = !(msm_chg.batt_status == BATT_STATUS_ABSENT);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
#ifdef CONFIG_LGE_PM
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
#else		
		val->intval = POWER_SUPPLY_TECHNOLOGY_NiMH;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_chg.max_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_chg.min_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_LGE_PM		
/* we comment out below */
/*
		if(pseudo_batt_info.mode == 1)
		  val->intval = pseudo_batt_info.volt;
		else
*/		
#endif		
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
    batt_volt = get_prop_battery_mvolts();
#endif
		val->intval = get_prop_battery_mvolts() * 1000; /*                                                                     */ 
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
  if((0 == battery_info_get()) ||(6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value()) ||(11 ==get_ext_cable_type_value())) /* 6,7 = LT cable*/
		val->intval = 70;//for test only
	else if((batt_volt > 4250) && (bq24160_chg_done_once))
	  val->intval = 100;
	else
  {
    bq24160_chg_done_once = false;
	val->intval = get_prop_batt_capacity();
  }
#else

#if defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_120_BOARD)
	if(pseudo_batt_info.mode == 1)
	  val->intval = pseudo_batt_info.capacity; //80
	else
#endif
	if((0 == battery_info_get()) && ((6 ==get_ext_cable_type_value())||(7 ==get_ext_cable_type_value()))) /* 6,7 = LT cable*/
		val->intval = 70;//for test only
	else if (msm_chg.batt_status ==
		 BATT_STATUS_JUST_FINISHED_CHARGING
			 && msm_chg.current_chg_priv != NULL
			 &&(get_prop_batt_capacity() >= 95))
		val->intval = 100;
	else
		val->intval = get_prop_batt_capacity();
#endif
#endif       
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
   fuel_percent = val->intval;
#endif
		break;
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
		if(pseudo_batt_info.mode == 1)
		  val->intval = pseudo_batt_info.id;
		else if((6 ==get_ext_cable_type_value()) || (7 ==get_ext_cable_type_value()))
		  val->intval = 1; /* 6 = LT cable*/
		else
		val->intval = is_battery_id_valid();
	break;

#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	case POWER_SUPPLY_PROP_TEMP: // temp?? ?¥ì? ?? 
#ifdef CONFIG_MACH_LGE_120_BOARD  
	  if((pseudo_batt_info.mode == 1)&& (msm_chg.temperature >= FAKE_BATT_DISABLE_TEMP))
		val->intval = get_battery_temperature()*10;
	  else if ((pseudo_batt_info.mode == 1)&& (msm_chg.temperature < FAKE_BATT_DISABLE_TEMP))
		val->intval = pseudo_batt_info.temp*10;	  
	  else if((6 ==get_ext_cable_type_value()) || (7 ==get_ext_cable_type_value()))
	  	val->intval = 35*10;
	  else
		val->intval = get_battery_temperature()*10;
#else
	  if(pseudo_batt_info.mode == 1)
		val->intval = pseudo_batt_info.temp*10;	  
	  else if((6 ==get_ext_cable_type_value()) || (7 ==get_ext_cable_type_value()))
	  	val->intval = 35*10;
	  else
		val->intval = get_battery_temperature()*10;
#endif
	  break;

	case POWER_SUPPLY_PROP_BATTERY_TEMP_ADC: // ?¥ì? ADC ??. 
#ifdef CONFIG_MACH_LGE_120_BOARD
    if((pseudo_batt_info.mode == 1)&& (msm_chg.temperature >= FAKE_BATT_DISABLE_TEMP))
      val->intval = get_battery_temperature_adc();
	else if ((pseudo_batt_info.mode == 1)&& (msm_chg.temperature < FAKE_BATT_DISABLE_TEMP))
      val->intval = pseudo_batt_info.therm;
    else
      val->intval = get_battery_temperature_adc();
#else
    if(pseudo_batt_info.mode == 1)
      val->intval = pseudo_batt_info.therm;
    else
      val->intval = get_battery_temperature_adc();
#endif

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
    batt_temp = val->intval;
#endif
    break;
#endif

#ifdef CONFIG_LGE_PM
  case POWER_SUPPLY_PROP_PSEUDO_BATT:
    val->intval = pseudo_batt_info.mode;
    break;

  case POWER_SUPPLY_PROP_BLOCK_CHARGING:
    val->intval = block_charging_state;
    break;
    case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
		ext_cable_type = get_ext_cable_type_value();
		if ((ext_cable_type == 10)&&(usb_pc_connection == 0)){
			val->intval = TA_CABLE_NOT_AUTH_700MA;
			}
		else
			val->intval = ext_cable_type; // 8  :USB_CABLE_400MA, 9: USB_CABLE_DTC_500MA, 10:ABNORMAL_USB_CABLE_400MA
#else
  	    val->intval = get_ext_cable_type_value(); // 8  :USB_CABLE_400MA, 9: USB_CABLE_DTC_500MA, 10:ABNORMAL_USB_CABLE_400MA
#endif
  	    break;
#if 0		
    case POWER_SUPPLY_PROP_WLC_STATUS:
	    val->intval = msm_batt_info.wlc_status;
	    break;
#endif
#ifdef CONFIG_LGE_PM_FACTORY_CURRENT_DOWN
    case POWER_SUPPLY_PROP_FACTORY_MODE:
      if((0 == battery_info_get())&&((usb_cable_info == 6) ||(usb_cable_info == 7)||(usb_cable_info == 11)))
	  {
	   // printk(KERN_DEBUG "############ Factory Mode #####################\n");
	    val->intval = 1;
      }
      else
      {
	  //  printk(KERN_DEBUG "############ Normal Mode #####################\n");
        val->intval = 0;
      }

    break;

#endif

 //                                         
    case POWER_SUPPLY_PROP_CHARGING_CURRENT_SET:
	 val->intval = msm_chg.chg_current_set;
	  break;	
//                                       
  case POWER_SUPPLY_PROP_SMPL_MODE:
    val->intval = smpl_on;
    break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
	.set_property = msm_batt_power_set_property, //                        
	.property_is_writeable = msm_batt_power_property_is_writeable, //                        
};

#ifdef CONFIG_LGE_FUEL_GAUGE
static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

#if defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};
#endif
#endif

#ifdef CONFIG_LGE_PM
int pseudo_batt_set(struct pseudo_batt_info_type* info)
{
	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

	power_supply_changed(&msm_psy_batt);
	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set);

void block_charging_set(int block)
{
    if(block)
    {
		pm_chg_auto_disable(0);
    }
    else
    {
    	pm_chg_auto_disable(1);
    }

}
void batt_block_charging_set(int block)
{
	block_charging_state = block;
	block_charging_set(block);
	power_supply_changed(&msm_psy_batt);
}
EXPORT_SYMBOL(batt_block_charging_set);
#endif
static int usb_chg_current;
static struct msm_hardware_charger_priv *usb_hw_chg_priv;
static void (*notify_vbus_state_func_ptr)(int);
static int usb_notified_of_insertion;

/* this is passed to the hsusb via platform_data msm_otg_pdata */
int msm_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = callback;
	return 0;
}

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = NULL;
}

static void notify_usb_of_the_plugin_event(struct msm_hardware_charger_priv
					   *hw_chg, int plugin)
{
	plugin = !!plugin;
	if (plugin == 1 && usb_notified_of_insertion == 0) {
		usb_notified_of_insertion = 1;
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying plugin\n", __func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify plugin\n",
				__func__);
		usb_hw_chg_priv = hw_chg;
	}
	if (plugin == 0 && usb_notified_of_insertion == 1) {
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying unplugin\n",
				__func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify unplugin\n",
				__func__);
		usb_notified_of_insertion = 0;
		usb_hw_chg_priv = NULL;
	}
}

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
int old_soc;
extern int is_chg_plugged_in(void);
extern int max17040_get_battery_capacity_percent(void);

static unsigned int msm_chg_get_batt_capacity_percent(void)
{
#ifdef CONFIG_LGE_FUEL_GAUGE
	int soc = max17040_get_battery_capacity_percent();

	if (!is_chg_plugged_in()) 
  {
		if (old_soc < soc) 
      return old_soc;
		else 
    {
			old_soc = soc;
			return soc;
		}
	}
	else 
  {
		old_soc = soc;
		return soc;
	}
#endif
}
#else
static unsigned int msm_chg_get_batt_capacity_percent(void)
{
#ifdef CONFIG_LGE_FUEL_GAUGE
	return max17040_get_battery_capacity_percent();
#else
	unsigned int current_voltage = get_prop_battery_mvolts();
	unsigned int low_voltage = msm_chg.min_voltage;
	unsigned int high_voltage = msm_chg.max_voltage;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
		    / (high_voltage - low_voltage);
#endif
}
#endif


#ifdef DEBUG
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
	dev_dbg(msm_chg.dev,
		"%s current=(%s)(s=%d)(r=%d) new=(%s)(s=%d)(r=%d) batt=%d En\n",
		func,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->name : "none",
		msm_chg.current_chg_priv ? msm_chg.
		current_chg_priv->hw_chg_state : -1,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->rating : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->name : "none",
		hw_chg_priv ? hw_chg_priv->hw_chg_state : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->rating : -1,
		msm_chg.batt_status);
}
#else
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
}
#endif

static struct msm_hardware_charger_priv *find_best_charger(void)
{
	struct msm_hardware_charger_priv *hw_chg_priv;
	struct msm_hardware_charger_priv *better;
	int rating;

	better = NULL;
	rating = 0;

	list_for_each_entry(hw_chg_priv, &msm_chg.msm_hardware_chargers, list) {
		if (is_chg_capable_of_charging(hw_chg_priv)) {
			if (hw_chg_priv->hw_chg->rating > rating) {
				rating = hw_chg_priv->hw_chg->rating;
				better = hw_chg_priv;
			}
		}
	}

	return better;
}

static int msm_charging_switched(struct msm_hardware_charger_priv *priv)
{
	int ret = 0;

	if (priv->hw_chg->charging_switched)
		ret = priv->hw_chg->charging_switched(priv->hw_chg);
	return ret;
}

static int msm_stop_charging(struct msm_hardware_charger_priv *priv)
{
	int ret;

	ret = priv->hw_chg->stop_charging(priv->hw_chg);
	if (!ret)
		wake_unlock(&msm_chg.wl);
	return ret;
}

static void msm_enable_system_current(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg->start_system_current)
		priv->hw_chg->start_system_current(priv->hw_chg,
					 priv->max_source_current);
}

static void msm_disable_system_current(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg->stop_system_current)
		priv->hw_chg->stop_system_current(priv->hw_chg);
}

/* the best charger has been selected -start charging from current_chg_priv */
static int msm_start_charging(void)
{
	int ret;
	struct msm_hardware_charger_priv *priv;

	priv = msm_chg.current_chg_priv;
	wake_lock(&msm_chg.wl);
	ret = priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
					 priv->max_source_current);
	if (ret) {
		wake_unlock(&msm_chg.wl);
		dev_err(msm_chg.dev, "%s couldnt start chg error = %d\n",
			priv->hw_chg->name, ret);
	} else
		priv->hw_chg_state = CHG_CHARGING_STATE;

	return ret;
}

static void handle_charging_done(struct msm_hardware_charger_priv *priv)
{
	if (msm_chg.current_chg_priv == priv) {
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE)
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		msm_chg.batt_status = BATT_STATUS_JUST_FINISHED_CHARGING;
		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);

		if (msm_batt_gauge && msm_batt_gauge->monitor_for_recharging)
			msm_batt_gauge->monitor_for_recharging();
		else
			dev_err(msm_chg.dev,
			      "%s: no batt gauge recharge monitor\n", __func__);
	}
}

static void teoc(struct work_struct *work)
{
#ifdef CONFIG_LGE_PM
/* 2011-08-16, when the battery fake mode is on, we don't use safety time for AT&T,MTBF and someting else */
  if(pseudo_batt_info.mode == 1)
    return;
#endif

	/* we have been charging too long - stop charging */
	dev_info(msm_chg.dev, "%s: safety timer work expired\n", __func__);

	mutex_lock(&msm_chg.status_lock);
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		handle_charging_done(msm_chg.current_chg_priv);
	}
	mutex_unlock(&msm_chg.status_lock);
}

static void handle_battery_inserted(void)
{
	/* if a charger is already present start charging */
	if (msm_chg.current_chg_priv != NULL &&
	    is_batt_status_capable_of_charging() &&
	    !is_batt_status_charging()) {
		if (msm_start_charging()) {
			dev_err(msm_chg.dev, "%s couldnt start chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
			return;
		}
		msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;

		dev_info(msm_chg.dev, "%s: starting safety timer work\n",
				__func__);
		queue_delayed_work(msm_chg.event_wq_thread,
					&msm_chg.teoc_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (msm_chg.
							      safety_time)));
	}
}

static void handle_battery_removed(void)
{
	/* if a charger is charging the battery stop it */
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		if (msm_stop_charging(msm_chg.current_chg_priv)) {
			dev_err(msm_chg.dev, "%s couldnt stop chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);
	}
}
#ifdef ACC_ENABLE
extern int vchg_read_adc(int channel, int *mv_reading);
#define VCHG_ADC_MIN 4660 //mV

static int acc_charge_control (void)
{
	int vchg_adc_result;

	if (usb_pc_connection == 1){
		f120_acc_data.charger_step = Step_1_charging_450mA;
		
		return f120_acc_data.charger_step;
		}

	if (f120_acc_data.charger_step == 4){
		vchg_adc_result = (int)vchg_read_adc(CHANNEL_ADC_VCHG, NULL);

		if (vchg_adc_result < VCHG_ADC_MIN){
			f120_acc_data.charger_step--;
			f120_acc_data.charger_fixed = TRUE;
			f120_acc_data.vchg_good_cnt++; //Check!!!!!
			}

		return f120_acc_data.charger_step;
		}

	if (f120_acc_data.wait_cnt == 1) {
		vchg_adc_result = (int)vchg_read_adc(CHANNEL_ADC_VCHG, NULL);
		if (vchg_adc_result >= VCHG_ADC_MIN){
			f120_acc_data.charger_step++;
			}
		else { // vchg_adc_result < VCHG_ADC_MIN
			f120_acc_data.charger_step--;
			f120_acc_data.charger_fixed = TRUE;
			f120_acc_data.vchg_good_cnt++; //Check!!!!!

			if (f120_acc_data.charger_step == 0) {
				f120_acc_data.charger_step = Step_1_charging_450mA;
				}
			}
		f120_acc_data.wait_cnt = 0;

		return f120_acc_data.charger_step;
		}

	if (f120_acc_data.charger_fixed == TRUE){
		
		if(f120_acc_data.vchg_good_cnt == 4) {
			f120_acc_data.charger_fixed = FALSE;
			f120_acc_data.vchg_good_cnt = 0;
			f120_acc_data.wait_cnt++; //Check!!!!!

			return f120_acc_data.charger_step;
			}
		else {
			f120_acc_data.vchg_good_cnt++;

			return f120_acc_data.charger_step;
			}
		}	

	f120_acc_data.wait_cnt++;

	return f120_acc_data.charger_step;
	
}

static int acc_current(void)
{
	int acc_curr = 450;

	switch (acc_charge_control()) {
		case Step_1_charging_450mA:
			acc_curr = 450;
			break;

		case Step_2_charging_535mA:
			acc_curr = 535;
			break;

		case Step_3_charging_620mA:
			acc_curr = 620;
			break;

		case Step_4_charging_700mA:
			acc_curr = 700;
			break;

		}
	printk("[%s] : acc_curr = %d \n", __func__, acc_curr);
	return acc_curr;
			
}
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
int g_temp_adc = 1000;
static int chg_is_battery_too_hot_or_too_cold(int temp_adc, int batt_level, int curr_set_status) //                                   
{
    int chg_batt_temp;
    int rtnValue = 0;
#ifdef CONFIG_LGE_BACKLIGHT_LM3530
	int curr = 0;  //                                   
#endif
/*                                                                                                    */
	pseudo_ui_charging = 0;
    if(temp_adc < adcmap_batttherm[THERM_55].x)
      chg_batt_temp = CHG_BATT_TEMP_OVER_55;
    else if(temp_adc < adcmap_batttherm[THERM_45].x)
      chg_batt_temp = CHG_BATT_TEMP_46_55;
    else if(temp_adc <= adcmap_batttherm[THERM_42].x)
      chg_batt_temp = CHG_BATT_TEMP_42_45;
    else if(temp_adc < adcmap_batttherm[THERM_M5].x)
      chg_batt_temp = CHG_BATT_TEMP_M4_41;
    else if(temp_adc <= adcmap_batttherm[THERM_M10].x)
      chg_batt_temp = CHG_BATT_TEMP_M10_M5;
    else
      chg_batt_temp = CHG_BATT_TEMP_UNDER_M10;
#ifdef CONFIG_LGE_PM_TA_COMPENSATION
	if (is_chg_plugged_in()) {
	    if(temp_adc < adcmap_batttherm_ta[THERM_55].x)
	      chg_batt_temp = CHG_BATT_TEMP_OVER_55;
	    else if(temp_adc < adcmap_batttherm_ta[THERM_45].x)
	      chg_batt_temp = CHG_BATT_TEMP_46_55;
	    else if(temp_adc <= adcmap_batttherm_ta[THERM_42].x)
	      chg_batt_temp = CHG_BATT_TEMP_42_45;
	    else if(temp_adc < adcmap_batttherm_ta[THERM_M5].x)
	      chg_batt_temp = CHG_BATT_TEMP_M4_41;
	    else if(temp_adc <= adcmap_batttherm_ta[THERM_M10].x)
	      chg_batt_temp = CHG_BATT_TEMP_M10_M5;
	    else
    	  chg_batt_temp = CHG_BATT_TEMP_UNDER_M10;
	}
#endif

    switch(chg_batt_temp_state)
    {
      case CHG_BATT_NORMAL_STATE:
          if(chg_batt_temp == CHG_BATT_TEMP_OVER_55 || 
            chg_batt_temp == CHG_BATT_TEMP_UNDER_M10 ||
            (chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level > 4000) /*4.0V*/)
          {
            chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
/*                                                                                                    */
			if(chg_batt_temp == CHG_BATT_TEMP_UNDER_M10 ||(chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level > 4000))
			{
				pseudo_ui_charging = 1; // we must show charging image although charging is stopped.
			}
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_NORMAL_STATE,temp_adc,batt_level);
            }
			rtnValue = 1;
          }
          else if(chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level <= 4000)
          {
            chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
			  				__func__,CHG_BATT_NORMAL_STATE, temp_adc,batt_level);
            }
			
			rtnValue = 0;					
			if(get_ext_cable_type_value() == MHL_CABLE_500MA ||
				get_ext_cable_type_value() == TA_CABLE_600MA ||
				get_ext_cable_type_value() == TA_CABLE_800MA ||
				get_ext_cable_type_value() == TA_CABLE_DTC_800MA || 				
				get_ext_cable_type_value() == USB_CABLE_DTC_500MA ||
				get_ext_cable_type_value() == TA_CABLE_FORGED_500MA
				)
				pm_chg_imaxsel_set(450);
          }
          else
          {
            chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
			  				__func__,CHG_BATT_NORMAL_STATE, temp_adc,batt_level);
            }
			
			rtnValue = 0;

			if(get_ext_cable_type_value() == MHL_CABLE_500MA)
				pm_chg_imaxsel_set(500);
			else if(get_ext_cable_type_value() == TA_CABLE_600MA)
				pm_chg_imaxsel_set(600);
			else if(get_ext_cable_type_value() == TA_CABLE_800MA)
			{
#ifdef CONFIG_LGE_BACKLIGHT_LM3530
			  //                                         
			  printk("[%s] CHG_BATT_NORMAL_STATE, backlight=%d, curr_set_status = %d \n"
			  ,__func__, lm3530_backlight_status(), curr_set_status);
			  if ((msm_chg.chg_current_set == 1) && (lm3530_backlight_status() == 1)) //LCD ON, Overheat Fake mode
			  {
			    /*Default charging current is setted to 450mA while LCD is turned ON */
			  	if (msm_chg.capacity <= 10)
				{
					curr = 800;
					curr_set_status = 1; //It means charging current is 800mA while LCD is turned ON.
			  	}
				else if ((msm_chg.capacity > 10) && (msm_chg.capacity <= 20))//10<capacity<=20 
				{
					if (curr_set_status == 0)  //charging current is 450mA
					{
						curr = 450; /*Default Value */
						curr_set_status = 0;
					}
					else if(curr_set_status == 1) // charging current is 800mA 
					{
						curr = 800;
						curr_set_status = 1;
					}
				}
				else //capacity > 20 
				{
					curr = 450; /*Default Value */
					curr_set_status = 0;
				}
				  pm_chg_imaxsel_set(curr); 
				  msm_chg.lcd_on_charge_curr_status = curr_set_status;
				  printk("[%s] LCD ON: pm_chg_imaxsel_set(%d), curr_set_status = %d, capacity = %d \n",
				  	__func__, curr, curr_set_status, msm_chg.capacity);
			  }
			  else //LCD OFF, NORMAL MODE
			  {
			  	  if (msm_chg.chg_current_set == 1) {
				  	msm_chg.chg_current_set = 0;
					msm_chg.lcd_on_charge_curr_status = 0;
					printk("[%s] LCD OFF, so we need to set chg_current_set to 0!! \n",__func__);
			  	  	}
				  pm_chg_imaxsel_set(800);
				  printk("[%s] LCD OFF or Normal Mode: pm_chg_imaxsel_set(800)\n",__func__);
			  }
			  //                                       
#endif
			}
			else if(get_ext_cable_type_value() == TA_CABLE_DTC_800MA)
				pm_chg_imaxsel_set(800);
			else if(get_ext_cable_type_value() == TA_CABLE_FORGED_500MA)
				pm_chg_imaxsel_set(500);			
			else if(get_ext_cable_type_value() == LT_CABLE_56K)
				pm_chg_imaxsel_set(1500);
			else if(get_ext_cable_type_value() == LT_CABLE_130K)
				pm_chg_imaxsel_set(1500);			
			else if(get_ext_cable_type_value() == USB_CABLE_DTC_500MA)
				pm_chg_imaxsel_set(500);
			else
#ifdef ACC_ENABLE
					  pm_chg_imaxsel_set(acc_current());
#else
				pm_chg_imaxsel_set(450);
#endif
          }
        break;

      case CHG_BATT_DC_CURRENT_STATE:
          if(chg_batt_temp == CHG_BATT_TEMP_OVER_55 || 
              chg_batt_temp == CHG_BATT_TEMP_UNDER_M10 ||
              (chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level > 4000) /*4.0V*/)
          {
            chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
/*                                                                                                    */
			if(chg_batt_temp == CHG_BATT_TEMP_UNDER_M10 ||(chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level > 4000))
			{
				pseudo_ui_charging = 1; // we must show charging image although charging is stopped.
			}
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
			 				__func__,CHG_BATT_DC_CURRENT_STATE, temp_adc,batt_level);
			}
			
			rtnValue = 1;
          }
          else if(chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level <= 4000)
          {
            chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_DC_CURRENT_STATE, temp_adc,batt_level);
            }
			rtnValue = 0;
			if(get_ext_cable_type_value() == MHL_CABLE_500MA ||
				get_ext_cable_type_value() == TA_CABLE_600MA ||
				get_ext_cable_type_value() == TA_CABLE_800MA ||
				get_ext_cable_type_value() == TA_CABLE_DTC_800MA ||				
				get_ext_cable_type_value() == TA_CABLE_FORGED_500MA ||
				get_ext_cable_type_value() == USB_CABLE_DTC_500MA
				)
				pm_chg_imaxsel_set(450);			
          }
		  
          else if(chg_batt_temp == CHG_BATT_TEMP_M4_41 || chg_batt_temp == CHG_BATT_TEMP_M10_M5 || chg_batt_temp == CHG_BATT_TEMP_42_45)
          {
            chg_batt_temp_state = CHG_BATT_NORMAL_STATE;			
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_DC_CURRENT_STATE, temp_adc,batt_level);
            }
			rtnValue = 0;	

			if(get_ext_cable_type_value() == MHL_CABLE_500MA)
				pm_chg_imaxsel_set(500);
			else if(get_ext_cable_type_value() == TA_CABLE_600MA)
				pm_chg_imaxsel_set(600);
			else if(get_ext_cable_type_value() == TA_CABLE_800MA)
			{
			  //                                         
#ifdef CONFIG_LGE_BACKLIGHT_LM3530
			  printk("[%s] CHG_BATT_DC_CURRENT_STATE, backlight=%d, curr_set_status = %d \n"
			  ,__func__, lm3530_backlight_status(), curr_set_status);
			  if ((msm_chg.chg_current_set == 1) && (lm3530_backlight_status() == 1)) //LCD ON, Overheat Fake mode
			  {
			    /*Default charging current is setted to 450mA while LCD is turned ON */
			  	if (msm_chg.capacity <= 10)
				{
					curr = 800;
					curr_set_status = 1; //It means charging current is 800mA while LCD is turned ON.
			  	}
				else if ((msm_chg.capacity > 10) && (msm_chg.capacity <= 20))//10<capacity<=20 
				{
					if (curr_set_status == 0)  //charging current is 450mA
					{
						curr = 450; /*Default Value */
						curr_set_status = 0;
					}
					else if(curr_set_status == 1) // charging current is 800mA 
					{
						curr = 800;
						curr_set_status = 1;
					}
				}
				else //capacity > 20 
				{
					curr = 450; /*Default Value */
					curr_set_status = 0;
				}
				  pm_chg_imaxsel_set(curr); 
				  msm_chg.lcd_on_charge_curr_status = curr_set_status;
				  printk("[%s] LCD ON: pm_chg_imaxsel_set(%d), curr_set_status = %d, capacity = %d \n",
				  	__func__, curr, curr_set_status, msm_chg.capacity);
			  }
			  else //LCD OFF, NORMAL MODE
			  {
			  	  if (msm_chg.chg_current_set == 1) {
				  	msm_chg.chg_current_set = 0;
					msm_chg.lcd_on_charge_curr_status = 0;
					printk("[%s] LCD OFF, so we need to set chg_current_set to 0!! \n",__func__);
			  	  	}
				  pm_chg_imaxsel_set(800);
					printk("[%s] LCD OFF or Normal Mode: pm_chg_imaxsel_set(800)\n",__func__);
			  }
			  //                                       
#endif
			}				
			else if(get_ext_cable_type_value() == TA_CABLE_DTC_800MA)
				pm_chg_imaxsel_set(800);
			else if(get_ext_cable_type_value() == TA_CABLE_FORGED_500MA)
				pm_chg_imaxsel_set(500);
			else if(get_ext_cable_type_value() == LT_CABLE_56K)
				pm_chg_imaxsel_set(1500);
			else if(get_ext_cable_type_value() == LT_CABLE_130K)
				pm_chg_imaxsel_set(1500);			
			else if(get_ext_cable_type_value() == USB_CABLE_DTC_500MA)
				pm_chg_imaxsel_set(500);
			else
#ifdef ACC_ENABLE
					  pm_chg_imaxsel_set(acc_current());
#else
				pm_chg_imaxsel_set(450);
#endif
          }
          else
   		  {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP UNREAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_DC_CURRENT_STATE, temp_adc,batt_level);
			}
			
			rtnValue = 0; 				
		  }       
          break;

      case CHG_BATT_STOP_CHARGING_STATE:
        if(chg_batt_temp == CHG_BATT_TEMP_M4_41 /* #ifndef SKW_TEST || chg_batt_temp== CHG_BATT_TEMP_M10_M5*/)
        {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_STOP_CHARGING_STATE, temp_adc,batt_level);
	        }
			
			rtnValue = 0;			

			if(get_ext_cable_type_value() == MHL_CABLE_500MA)
				pm_chg_imaxsel_set(500);
			else if(get_ext_cable_type_value() == TA_CABLE_600MA)
				pm_chg_imaxsel_set(600);
			else if(get_ext_cable_type_value() == TA_CABLE_800MA)
			{
#ifdef CONFIG_LGE_BACKLIGHT_LM3530
			  //                                         
			  printk("[%s] CHG_BATT_STOP_CHARGING_STATE, backlight=%d, curr_set_status = %d \n"
			  ,__func__, lm3530_backlight_status(), curr_set_status);
			  if ((msm_chg.chg_current_set == 1) && (lm3530_backlight_status() == 1)) //LCD ON, Overheat Fake mode
			  {
				/*Default charging current is setted to 450mA while LCD is turned ON */
				if (msm_chg.capacity <= 10)
				{
					curr = 800;
					curr_set_status = 1; //It means charging current is 800mA while LCD is turned ON.
				}
				else if ((msm_chg.capacity > 10) && (msm_chg.capacity <= 20))//10<capacity<=20 
				{
					if (curr_set_status == 0)  //charging current is 450mA
					{
						curr = 450; /*Default Value */
						curr_set_status = 0;
					}
					else if(curr_set_status == 1) // charging current is 800mA 
					{
						curr = 800;
						curr_set_status = 1;
					}
				}
				else //capacity > 20 
				{
					curr = 450; /*Default Value */
					curr_set_status = 0;
				}
				  pm_chg_imaxsel_set(curr); 
				  msm_chg.lcd_on_charge_curr_status = curr_set_status;
				  printk("[%s] LCD ON: pm_chg_imaxsel_set(%d), curr_set_status = %d, capacity = %d \n",
					__func__, curr, curr_set_status, msm_chg.capacity);
			  }
			  else //LCD OFF, NORMAL MODE
			  {
				  if (msm_chg.chg_current_set == 1) {
					msm_chg.chg_current_set = 0;
					msm_chg.lcd_on_charge_curr_status = 0;
					printk("[%s] LCD OFF, so we need to set chg_current_set to 0!! \n",__func__);
					}
				  pm_chg_imaxsel_set(800);
				  printk("[%s] LCD OFF or Normal Mode: pm_chg_imaxsel_set(800)\n",__func__);
			  }
			  //                                       
#endif
			}				
			else if(get_ext_cable_type_value() == TA_CABLE_DTC_800MA)
				pm_chg_imaxsel_set(800);
			else if(get_ext_cable_type_value() == TA_CABLE_FORGED_500MA)
				pm_chg_imaxsel_set(500);
			else if(get_ext_cable_type_value() == LT_CABLE_56K)
				pm_chg_imaxsel_set(1500);
			else if(get_ext_cable_type_value() == LT_CABLE_130K)
				pm_chg_imaxsel_set(1500);
			else if(get_ext_cable_type_value() == USB_CABLE_DTC_500MA)
				pm_chg_imaxsel_set(500);			
			else
#ifdef ACC_ENABLE
					  pm_chg_imaxsel_set(acc_current());
#else
				pm_chg_imaxsel_set(450);
#endif
		}
/*                                                                     
                                                                                                               
         
                                                   
                                        
    
                                                                      
                                                                     
    
   
                     
                                                      
                                                   
                                                   
                                                           
                                                       
     
                            
         
*/        
        else
        {
			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
/*                                                                                                        */
			if(chg_batt_temp == CHG_BATT_TEMP_UNDER_M10 ||(chg_batt_temp == CHG_BATT_TEMP_46_55 && batt_level > 4000))
			{
				pseudo_ui_charging = 1; // we must show charging image although charging is stopped.
			}
			
			if(charging_flow_monitor_enable == 1)
			{
				pr_err("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
							__func__,CHG_BATT_STOP_CHARGING_STATE, temp_adc,batt_level);
			}
			rtnValue = 1;
        }
        break;
    }

	return rtnValue;

}


#endif

static void update_heartbeat(struct work_struct *work)
{
	//int temperature;
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	int temp_adc;
	int battery_level;
	int stop_charging = 0;
#endif

	if (msm_chg.batt_status == BATT_STATUS_ABSENT
		|| msm_chg.batt_status == BATT_STATUS_ID_INVALID) {
		if (is_battery_present())
			if (is_battery_id_valid()) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				handle_battery_inserted();
			}
	} else {
		if (!is_battery_present()) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
		}
		/*
		 * check battery id because a good battery could be removed
		 * and replaced with a invalid battery.
		 */
		if (!is_battery_id_valid()) {
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
			handle_battery_removed();
		}
	}
	pr_debug("msm-charger %s batt_status= %d\n",
				__func__, msm_chg.batt_status);
//sujin.shin [TEMP_CONTROL]
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
	if (msm_chg.temp_control == 0xFF)  //When phone is boot completed, 
	{
		printk("%s : msm_chg.temp_control value is 0xFF\n", __func__);
		msm_chg.temp_control = get_status_power();
		printk("%s : msm_chg.temp_control = %d\n", __func__, msm_chg.temp_control);
		if (msm_chg.temp_control == FAKE_BATT_ENABLED)
		{
			//SET Pseudo_batt ..
			pseudo_batt_set(&booting_set);
		}
		else if (msm_chg.temp_control == FAKE_BATT_DISABLED)
		{
			//default
		}
	}
#endif
	msm_chg.temperature = get_battery_temperature();
	msm_chg.capacity = get_prop_batt_capacity();  //                                   
	if (msm_chg.current_chg_priv
		&& msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) {
		/* TODO implement JEITA SPEC*/
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		pr_err("%s: battery temperature is %d celcius)!.\n",__func__,msm_chg.temperature);
		temp_adc = get_battery_temperature_adc();
        g_temp_adc = temp_adc;
		if(pseudo_batt_info.mode == 1)
		{
#ifdef CONFIG_MACH_LGE_120_BOARD
		  if(msm_chg.temperature < FAKE_BATT_DISABLE_TEMP)
		  {
		    temp_adc = pseudo_batt_info.therm; // kiwone.seo 2011-07-08 charging must be work in case of DV event in high temp
		  }	
#endif
		  temp_adc = pseudo_batt_info.therm; // kiwone.seo 2011-07-08 charging must be work in case of DV event in high temp
		}
		battery_level = get_prop_battery_mvolts();
		//stop_charging = chg_is_battery_too_hot_or_too_cold(temp_adc,battery_level);
		stop_charging = chg_is_battery_too_hot_or_too_cold(temp_adc,battery_level,msm_chg.lcd_on_charge_curr_status); //                                      
/*                                                                                                          */
#if defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
		if ((pseudo_batt_info.mode == 1)&&
			((get_ext_cable_type_value() == USB_CABLE_400MA)||
			(get_ext_cable_type_value() == ABNORMAL_USB_CABLE_400MA))){
			pm_chg_imaxsel_set(700);
		}
#endif
/* [End] USB charging current is set to 800mA when enabling FAKE mode. */
/*                                                                                                                                 
                                                                                                                 
                                                                                                                              
                                                                                                                     */
		if(stop_charging == 1 && last_stop_charging == 0)
		{
			pm_chg_auto_disable(1);
			pr_err("msm-charger stop charing by scenario\n");
			last_stop_charging = stop_charging;
#if 1 // after stop charging, charging is working when TA re-inserted to inform UI
/*                                                                                                    */
			if(pseudo_ui_charging)
				; 
			else
			msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
#endif
		}
		else if(stop_charging == 0 && last_stop_charging == 1)
		{
			pm_chg_auto_disable(0);
			pr_err("msm-charger start charing by scenario\n");
			last_stop_charging = stop_charging;
/*                                                                                                    */
			pseudo_ui_charging = 0;
#if 1 // after stop charging, charging is working when TA re-inserted to inform UI
			msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;
#endif
		}
#endif
	}

	/* notify that the voltage has changed
	 * the read of the capacity will trigger a
	 * voltage read*/
#ifdef CONFIG_LGE_FUEL_GAUGE
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
  if (usb_chg_type == 2) /* not fixed */
		power_supply_changed(&msm_psy_ac);
#else
	if (usb_chg_type_for_FG == 2) /* not fixed */
		power_supply_changed(&msm_psy_ac);
#endif
#if defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
	else if (usb_chg_type == 3)
		power_supply_changed(&msm_psy_usb);
#endif
	else
#endif
	power_supply_changed(&msm_psy_batt);

	if (msm_chg.stop_update) {
		msm_chg.stop_update = 0;
		return;
	}
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
}

/* set the charger state to READY before calling this */
static void handle_charger_ready(struct msm_hardware_charger_priv *hw_chg_priv)
{
	struct msm_hardware_charger_priv *old_chg_priv = NULL;

	debug_print(__func__, hw_chg_priv);
	wake_lock(&uevent_wake_lock);
	if (msm_chg.current_chg_priv != NULL
	    && hw_chg_priv->hw_chg->rating >
	    msm_chg.current_chg_priv->hw_chg->rating) {
		/*
		 * a better charger was found, ask the current charger
		 * to stop charging if it was charging
		 */
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE) {
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
			if (msm_charging_switched(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
		old_chg_priv = msm_chg.current_chg_priv;
		msm_chg.current_chg_priv = NULL;
	}

	if (msm_chg.current_chg_priv == NULL) {
		msm_chg.current_chg_priv = hw_chg_priv;
		dev_info(msm_chg.dev,
			 "%s: best charger = %s\n", __func__,
			 msm_chg.current_chg_priv->hw_chg->name);

		msm_enable_system_current(msm_chg.current_chg_priv);
		/*
		 * since a better charger was chosen, ask the old
		 * charger to stop providing system current
		 */
		if (old_chg_priv != NULL)
			msm_disable_system_current(old_chg_priv);

		if (!is_batt_status_capable_of_charging())
			return;

		/* start charging from the new charger */
		if (!msm_start_charging()) {
			/* if we simply switched chg continue with teoc timer
			 * else we update the batt state and set the teoc
			 * timer */
			if (!is_batt_status_charging()) {
				dev_info(msm_chg.dev,
				       "%s: starting safety timer\n", __func__);
				queue_delayed_work(msm_chg.event_wq_thread,
							&msm_chg.teoc_work,
						      round_jiffies_relative
						      (msecs_to_jiffies
						       (msm_chg.safety_time)));
				msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;
			}
		} else {
			/* we couldnt start charging from the new readied
			 * charger */
			if (is_batt_status_charging())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		}
	}
}

//                                                   
#define SKIP_STOP_CHARGING  1
#define DO_STOP_CHARGING    0
int lge_skip_stop_charging_with_factory_condition( void )
{
    acc_cable_type ext_cable_type;
    printk(KERN_DEBUG "%s: entered\n",__func__);

    if(0 != battery_info_get())
    {
        printk(KERN_DEBUG "%s: battery is present! ignore factory condition\n",__func__);
        return DO_STOP_CHARGING;
    }

    ext_cable_type = get_ext_cable_type_value();
    if( LT_CABLE_56K == ext_cable_type ) {
        printk(KERN_DEBUG "%s: 56K or 910K Factory Cable detected\n",__func__);
        return SKIP_STOP_CHARGING;
    }
    else {
        return DO_STOP_CHARGING;
    }
}
//                                                 

static void handle_charger_removed(struct msm_hardware_charger_priv
				   *hw_chg_removed, int new_state)
{
	struct msm_hardware_charger_priv *hw_chg_priv;

	msm_chg.lcd_on_charge_curr_status = 0; //                                   

#ifdef ACC_ENABLE
	if(usb_pc_connection == 1)
		usb_pc_connection = 0;
#endif

	debug_print(__func__, hw_chg_removed);
//                                                   
    if( lge_skip_stop_charging_with_factory_condition() )
        return;
//                                                 

	if (msm_chg.current_chg_priv == hw_chg_removed) {
		msm_disable_system_current(hw_chg_removed);
		if (msm_chg.current_chg_priv->hw_chg_state
						== CHG_CHARGING_STATE) {
			if (msm_stop_charging(hw_chg_removed)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		}
		msm_chg.current_chg_priv = NULL;
	}

	hw_chg_removed->hw_chg_state = new_state;

	if (msm_chg.current_chg_priv == NULL) {
		hw_chg_priv = find_best_charger();
		if (hw_chg_priv == NULL) {
			dev_info(msm_chg.dev, "%s: no chargers\n", __func__);
			/* if the battery was Just finished charging
			 * we keep that state as is so that we dont rush
			 * in to charging the battery when a charger is
			 * plugged in shortly. */
#ifdef CONFIG_LGE_PM
/*                                                                
                                                                                                  
                                                                                                       */
#else
			if (is_batt_status_charging())
#endif				
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
            pm8xxx_batt_alarm_config_lge();
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
            if (!is_battery_id_valid())
            {
            	pr_err("===========================================================");
            	pr_err("%s: pm_power_off board  \n",__func__);
            	pr_err("===========================================================");
            
                pm_power_off();
            }
#endif
            
		} else {
			msm_chg.current_chg_priv = hw_chg_priv;
			msm_enable_system_current(hw_chg_priv);
			dev_info(msm_chg.dev,
				 "%s: best charger = %s\n", __func__,
				 msm_chg.current_chg_priv->hw_chg->name);

			if (!is_batt_status_capable_of_charging())
				return;

			if (msm_start_charging()) {
				/* we couldnt start charging for some reason */
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
            pm8xxx_batt_alarm_config_lge();
#endif

		}
	}
	wake_lock_timeout(&uevent_wake_lock, 500);
	/* if we arent charging stop the safety timer */
	if (!is_batt_status_charging()) {
		dev_info(msm_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);
	}
}

/*                                     */
extern void pm8058_chg_batt_remove_and_reset(void);
static void update_battery_remove_work(struct work_struct *work)
{
  if(msm_batt_gauge->get_battery_temperature_adc() < 2000)
  {
    printk(KERN_ERR "================== [ update_battery_remove_work ] BATTERY INSERTED!!!! =======================\n");
  }
  else
  {
    printk(KERN_ERR "================== [ update_battery_remove_work ] BATTERY REMOVED!!!! =======================\n");
    pm8058_chg_batt_remove_and_reset();
  }
}
/*                                     */

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
static bool cable_already_inserted = false;
#endif

static void handle_event(struct msm_hardware_charger *hw_chg, int event)
{
	struct msm_hardware_charger_priv *priv = NULL;

	/*
	 * if hw_chg is NULL then this event comes from non-charger
	 * parties like battery gauge
	 */
	if (hw_chg)
		priv = hw_chg->charger_private;

	mutex_lock(&msm_chg.status_lock);

	switch (event) {
	case CHG_INSERTED_EVENT:
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		last_stop_charging = 0;
/*                                                                                                    */
		pseudo_ui_charging = 0;
#endif
		if (priv->hw_chg_state != CHG_ABSENT_STATE) {
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
      cable_already_inserted = true;
#endif
			dev_info(msm_chg.dev,
				 "%s insertion detected when cbl present",
				 hw_chg->name);
			break;
		}
#ifdef CONFIG_LGE_TOUCH_CHAGER_CTRL
		trigger_baseline_state_machine(IC_CHARGER_CTRL_ON);
#endif 				
		update_batt_status();
		if (hw_chg->type == CHG_TYPE_USB) {
			priv->hw_chg_state = CHG_PRESENT_STATE;
			notify_usb_of_the_plugin_event(priv, 1);
			if (usb_chg_current) {
				priv->max_source_current = usb_chg_current;
				usb_chg_current = 0;
				/* usb has already indicated us to charge */
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			}
		} else {
			priv->hw_chg_state = CHG_READY_STATE;
			handle_charger_ready(priv);
		}
		break;
	case CHG_ENUMERATED_EVENT:	/* only in USB types */
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s enum withuot presence\n",
				 hw_chg->name);
			break;
		}
		update_batt_status();
		dev_dbg(msm_chg.dev, "%s enum with %dmA to draw\n",
			 hw_chg->name, priv->max_source_current);
		if (priv->max_source_current == 0) {
			/* usb subsystem doesnt want us to draw
			 * charging current */
			/* act as if the charge is removed */
			if (priv->hw_chg_state != CHG_PRESENT_STATE)
				handle_charger_removed(priv, CHG_PRESENT_STATE);
		} else {
			if (priv->hw_chg_state != CHG_READY_STATE) {
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/*                                                              
                                                                                                              
                                                       */				
				if(priv->hw_chg_state == CHG_CHARGING_STATE)
					; // don't change hw_chg_state
				else
#endif
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			}
		}
		break;
	case CHG_REMOVED_EVENT:
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
    cable_already_inserted = false;
#endif
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s cable already removed\n",
				 hw_chg->name);
			break;
		}
#ifdef CONFIG_LGE_TOUCH_CHAGER_CTRL
		trigger_baseline_state_machine(IC_CHARGER_CTRL_OFF);
#endif 		
		update_batt_status();
#ifdef CONFIG_LGE_FUEL_GAUGE
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
    usb_chg_type = 0;
#else
		usb_chg_type_for_FG = 0;
#endif
#endif
		if (hw_chg->type == CHG_TYPE_USB) {
			usb_chg_current = 0;
			notify_usb_of_the_plugin_event(priv, 0);
		}
		handle_charger_removed(priv, CHG_ABSENT_STATE);
		break;
	case CHG_DONE_EVENT:
#ifdef CONFIG_LGE_TOUCH_CHAGER_CTRL
		trigger_baseline_state_machine(IC_CHARGER_CTRL_NONE);
#endif 		
		if (priv->hw_chg_state == CHG_CHARGING_STATE)
			handle_charging_done(priv);
		break;
	case CHG_BATT_BEGIN_FAST_CHARGING:
		/* only update if we are TRKL charging */
		if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
			msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
		break;
	case CHG_BATT_NEEDS_RECHARGING:
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		priv = msm_chg.current_chg_priv;
		break;
	case CHG_BATT_TEMP_OUTOFRANGE:
		/* the batt_temp out of range can trigger
		 * when the battery is absent */
		 //                                                                   
		if (pseudo_batt_info.mode == 1) {			
			break; //If it does not have charge when fake mode
		}
		if (!is_battery_present()
		    && msm_chg.batt_status != BATT_STATUS_ABSENT) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		handle_battery_removed();
		break;
	case CHG_BATT_TEMP_INRANGE:
		if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		/* check id */
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery
		 * and act as if the battery was inserted
		 * if a charger is present charging will be resumed */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		break;
	case CHG_BATT_INSERTED:
		if (msm_chg.batt_status != BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (!is_battery_present())
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		/* check if a charger is present */
		handle_battery_inserted();
		break;
	case CHG_BATT_REMOVED:
		if (msm_chg.batt_status == BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (is_battery_present())
			break;
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		handle_battery_removed();
		break;
	case CHG_BATT_STATUS_CHANGE:
		/* TODO  battery SOC like battery-alarm/charging-full features
		can be added here for future improvement */
		break;
/*                                     */
//                                  
	case CHG_BATT_REMOVE_EVENT:
		printk(KERN_DEBUG "############ Handle Event: [CHG_BATT_REMOVE_EVENT] #####################\n");
		queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_batt_remove_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.processing_delay)));
		break;
//#endif
/*                                     */

/*                                     */
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
	case CHG_BATT_REFRESH_EVENT:
		break;
#endif
/*                                     */
	}
	dev_dbg(msm_chg.dev, "%s %d done batt_status=%d\n", __func__,
		event, msm_chg.batt_status);

	/* update userspace */
	if (msm_batt_gauge)
		power_supply_changed(&msm_psy_batt);
#ifdef CONFIG_LGE_FUEL_GAUGE
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
  if(!cable_already_inserted)
  {
    if (usb_chg_type == 2) /* not fixed */
	    power_supply_changed(&msm_psy_ac);
	  else if (usb_chg_type == 3) /* not fixed */
	    power_supply_changed(&msm_psy_usb);
  }
#else
	if (priv) {
		if (usb_chg_type_for_FG == 2) /* not fixed */
			power_supply_changed(&msm_psy_ac);
		else
			power_supply_changed(&priv->psy);
	}
#endif
#else
	if (priv)
		power_supply_changed(&priv->psy);
#endif

	mutex_unlock(&msm_chg.status_lock);
}

static int msm_chg_dequeue_event(struct msm_charger_event **event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == 0) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		return -EINVAL;
	}
	*event = &msm_chg.queue[msm_chg.head];
	msm_chg.head = (msm_chg.head + 1) % MSM_CHG_MAX_EVENTS;
	pr_debug("%s dequeueing %d\n", __func__, (*event)->event);
	msm_chg.queue_count--;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static int msm_chg_enqueue_event(struct msm_hardware_charger *hw_chg,
			enum msm_hardware_charger_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == MSM_CHG_MAX_EVENTS) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d\n",
				__func__, event);
		return -EAGAIN;
	}
	pr_debug("%s queueing %d\n", __func__, event);
	msm_chg.queue[msm_chg.tail].event = event;
	msm_chg.queue[msm_chg.tail].hw_chg = hw_chg;
	msm_chg.tail = (msm_chg.tail + 1)%MSM_CHG_MAX_EVENTS;
	msm_chg.queue_count++;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static void process_events(struct work_struct *work)
{
	struct msm_charger_event *event;
	int rc;

	do {
		rc = msm_chg_dequeue_event(&event);
		if (!rc)
			handle_event(event->hw_chg, event->event);
	} while (!rc);
}

/* USB calls these to tell us how much charging current we should draw */
void msm_charger_vbus_draw(unsigned int mA)
{
	if (usb_hw_chg_priv) {
		usb_hw_chg_priv->max_source_current = mA;
/*                                                                                                          */
#if defined(CONFIG_MACH_LGE_IJB_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
		if ((pseudo_batt_info.mode == 1)&&
			((get_ext_cable_type_value() == USB_CABLE_400MA)||
			(get_ext_cable_type_value() == ABNORMAL_USB_CABLE_400MA))){
			usb_hw_chg_priv->max_source_current = 700;
			printk("%s :fake mode - max_source_current = %d \n", __func__, usb_hw_chg_priv->max_source_current);
		}
#endif
/* [End] USB charging current is set to 800mA when enabling FAKE mode */
//                                         
		if((msm_chg.chg_current_set == 1) && (msm_chg.capacity > 10)) {
			usb_hw_chg_priv->max_source_current = 450;
			printk("%s :Overheat fake mode - max_source_current = %d \n", __func__, usb_hw_chg_priv->max_source_current);
		}
//                                       
		msm_charger_notify_event(usb_hw_chg_priv->hw_chg,
						CHG_ENUMERATED_EVENT);
	} else
		/* remember the current, to be used when charger is ready */
		usb_chg_current = mA;
}

static int __init determine_initial_batt_status(void)
{
	/*                  */
	int rc;
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
	msm_chg.temp_control = 0xFF;
#endif
	//                                         
	msm_chg.chg_current_set = 0;
	msm_chg.lcd_on_charge_curr_status = 0;
	//                                       

	if (is_battery_present())
		if (is_battery_id_valid())
			if (is_battery_temp_within_range())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			else
				msm_chg.batt_status
				    = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
	else
		msm_chg.batt_status = BATT_STATUS_ABSENT;

	if (is_batt_status_capable_of_charging())
		handle_battery_inserted();

#ifdef CONFIG_LGE_FUEL_GAUGE
	rc = power_supply_register(msm_chg.dev, &msm_psy_ac);
	if (rc < 0) {
		dev_err(msm_chg.dev, "%s(AC): power_supply_register failed"
			" rc=%d\n", __func__, rc);
		return rc;
	}
#if defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
	rc = power_supply_register(msm_chg.dev, &msm_psy_usb);
	if (rc < 0) {
		dev_err(msm_chg.dev, "%s(USB): power_supply_register failed"
			" rc=%d\n", __func__, rc);
		return rc;
	}
#endif
#endif

	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));

	pr_debug("%s:OK batt_status=%d\n", __func__, msm_chg.batt_status);
	return 0;
}

#ifdef CONFIG_LGE_PM_CURRENT_CABLE_TYPE
/*                                                                                                    */
int get_is_battery_present(void)
{
	return msm_chg.batt_status;
};
EXPORT_SYMBOL(get_is_battery_present);
/* END */
#endif

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
extern int pm8058_start_charging_for_ATCMD(void);
extern int pm8058_stop_charging_for_ATCMD(void);

extern void max17040_set_battery_atcmd(int flag, int value);
extern int max17040_get_battery_capacity_percent(void);
extern void machine_restart(char *cmd);


static ssize_t at_chg_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
  bool b_chg_ok = false;

  if(msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE)
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
  int ret;
  unsigned char string[2];

	sscanf(buf, "%s", string);

  if (!count)
		return -EINVAL;

  if(!strncmp(string, "0", 1))
  {
    /* Stop Charging */
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
    struct msm_hardware_charger_priv *priv;

    bq24160_at_cmd_chg_set(true);

    /*                                                                         */
    if (msm_chg.current_chg_priv != NULL)
    {
      priv = msm_chg.current_chg_priv;
  	  ret = priv->hw_chg->stop_charging(priv->hw_chg);

      msm_chg.current_chg_priv->hw_chg_state = CHG_ABSENT_STATE;
    }
    /*                                                                         */
    
    msm_chg.batt_status = BATT_STATUS_ABSENT;
#else
    if(msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE)
    {
      ret = pm8058_stop_charging_for_ATCMD();
      msm_chg.current_chg_priv->hw_chg_state = CHG_ABSENT_STATE;
      msm_chg.batt_status = BATT_STATUS_ABSENT;
      b_is_at_cmd_on = false;
    }
#endif
  }
  else if(!strncmp(string, "1", 1))
  {
    /* Start Charging */
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
	  struct msm_hardware_charger_priv *priv;

    bq24160_at_cmd_chg_set(true);

    msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
    /*                                                                         */
    if (msm_chg.current_chg_priv != NULL)
    {
      msm_chg.current_chg_priv->hw_chg_state = CHG_CHARGING_STATE;

      priv = msm_chg.current_chg_priv;
      ret = priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
					 priv->max_source_current);
    }
    /*                                                                         */
#else
    if(msm_chg.current_chg_priv->hw_chg_state != CHG_CHARGING_STATE)
    {
      printk(KERN_DEBUG "############ [chg_status_store] START CHARGING #####################\n");

      b_is_at_cmd_on = true;
      msm_chg.current_chg_priv->hw_chg_state = CHG_CHARGING_STATE;
      msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
      ret = pm8058_start_charging_for_ATCMD();

      msm_charger_notify_event(usb_hw_chg_priv->hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);
    }
#endif
  }
	
	return count;
}


static ssize_t at_chg_status_complete_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool b_chg_complete = false;

#if defined(CONFIG_MACH_LGE_I_BOARD_DCM)
  int guage_level = 0;

  guage_level = max17040_get_battery_capacity_percent();

  if(guage_level == 100)
  {
    b_chg_complete = true;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }
  else
  {
    b_chg_complete = false;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }
#else
  if(get_prop_batt_status()==POWER_SUPPLY_STATUS_FULL)
  {
    b_chg_complete = true;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }
  else
  {
    b_chg_complete = false;
	  r = sprintf(buf, "%d\n", b_chg_complete);
  }
#endif
	
	return r;
}

static ssize_t at_chg_status_complete_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;  
    unsigned char string[2];
  
    sscanf(buf, "%s", string);
  
    if (!count)
      return -EINVAL;
  
    if(!strncmp(string, "0", 1))
    {
      /* Charging not Complete */
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
      struct msm_hardware_charger_priv *priv;

      bq24160_at_cmd_chg_set(true);

      msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;

      /*                                                                         */
      if (msm_chg.current_chg_priv != NULL)
      {
        msm_chg.current_chg_priv->hw_chg_state = CHG_CHARGING_STATE;
        priv = msm_chg.current_chg_priv;
        ret = priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
  					 priv->max_source_current);
      }      
      /*                                                                         */
      max17040_set_battery_atcmd(2, 100);
#else
      if(msm_chg.current_chg_priv->hw_chg_state != CHG_CHARGING_STATE)
      {  
        msm_chg.current_chg_priv->hw_chg_state = CHG_CHARGING_STATE;
        msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
        ret = pm8058_start_charging_for_ATCMD();

        msm_charger_notify_event(usb_hw_chg_priv->hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);
        max17040_set_battery_atcmd(2, 100);
        b_is_at_cmd_on = false;
      }
#endif
    }
    else if(!strncmp(string, "1", 1))
    {
      /* Charging Complete */
#if defined(CONFIG_MACH_LGE_I_BOARD_DCM) && defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY)
      struct msm_hardware_charger_priv *priv;

      bq24160_at_cmd_chg_set(true);

      /*                                                                         */
      if (msm_chg.current_chg_priv != NULL)
      {
        priv = msm_chg.current_chg_priv;
  	    ret = priv->hw_chg->stop_charging(priv->hw_chg);
        
        msm_chg.current_chg_priv->hw_chg_state = CHG_ABSENT_STATE;
      }
      /*                                                                         */
      msm_chg.batt_status = BATT_STATUS_ABSENT;

      max17040_set_battery_atcmd(1, 100);
#else
      if((msm_chg.batt_status != BATT_STATUS_JUST_FINISHED_CHARGING))
      {  
        b_is_at_cmd_on = true;

        ret = pm8058_stop_charging_for_ATCMD();
        msm_chg.current_chg_priv->hw_chg_state = CHG_ABSENT_STATE;
        msm_chg.batt_status = BATT_STATUS_ABSENT;

        max17040_set_battery_atcmd(1, 100);
      }
#endif
    }
    
    return count;

}



static ssize_t at_fuel_guage_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	max17040_set_battery_atcmd(0, 100);  // Reset the fuel guage IC

	r = sprintf(buf, "%d\n", true);

	max17040_set_battery_atcmd(2, 100);  // Release the AT command mode
	
	return r;
}


static ssize_t at_fuel_guage_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int guage_level = 0;

	guage_level = max17040_get_battery_capacity_percent();

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
  
  battery_level = get_prop_battery_mvolts();

  printk(KERN_DEBUG "############ [at_batt_level_show] BATT LVL = %d #####################\n", battery_level);

  r = sprintf(buf, "%d\n", battery_level);
	
	return r;
}



DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_status_complete_show, at_chg_status_complete_store);
DEVICE_ATTR(at_fuelrst, 0644, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0644, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_pmrst, 0644, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_batl, 0644, at_batt_level_show, NULL);
#endif

#if defined(CONFIG_LGE_PM_CAYMAN_VZW) || defined(CONFIG_LGE_PM_CAYMAN_MPCS)
void testmode_charging_mode_test(void)
{
  int ret;

  /* Start Charging */
  if(msm_chg.current_chg_priv->hw_chg_state != CHG_CHARGING_STATE)
  {
  	printk(KERN_DEBUG "############ [chg_status_store] START CHARGING #####################\n");

	b_is_testmode_cmd_on = true;
	msm_chg.current_chg_priv->hw_chg_state = CHG_CHARGING_STATE;
	msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
	ret = pm8058_start_charging_for_TESTMODE();

	msm_charger_notify_event(usb_hw_chg_priv->hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);
  }
	
}
EXPORT_SYMBOL(testmode_charging_mode_test);

void testmode_discharging_mode_test(void)
{
  int ret;

 /* Stop Charging */
    if(msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE)
    {
      ret = pm8058_stop_charging_for_TESTMODE();
      msm_chg.current_chg_priv->hw_chg_state = CHG_ABSENT_STATE;
      msm_chg.batt_status = BATT_STATUS_ABSENT;
      b_is_testmode_cmd_on = false;
    }
	
}
EXPORT_SYMBOL(testmode_discharging_mode_test);
#endif /*                                                           */


/*                                     */
#ifdef CONFIG_LGE_PM_TEMPERATURE_MONITOR
static void msm_charger_program_alarm(int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

 // printk(KERN_DEBUG "############ [msm_charger_program_alarm] ALARM TIME = %d #####################\n", seconds);

	next = ktime_add(msm_chg.last_poll, low_interval);

	alarm_start_range(&msm_chg.alarm, next, ktime_add(next, slack));
}


static void msm_charger_temp_work(struct work_struct *work)
{  
  int temp_adc;
  int polling_sec_time;
  
	msm_chg.last_poll = alarm_get_elapsed_realtime();

  /* Check the battery temperature to decide next alarm time */
  temp_adc = g_temp_adc;

  if(temp_adc < adcmap_batttherm[THERM_55].x)
    polling_sec_time = TIME_POLL_0P5_MINUTE;
  else if(temp_adc < adcmap_batttherm[THERM_45].x)
    polling_sec_time = TIME_POLL_10_MINUTE;
  else if(temp_adc <= adcmap_batttherm[THERM_42].x)
    polling_sec_time = TIME_POLL_30_MINUTE;
  else
    polling_sec_time = TIME_POLL_90_MINUTE;
#ifdef CONFIG_LGE_PM_TA_COMPENSATION
  if (is_chg_plugged_in())
  {
	  if(temp_adc < adcmap_batttherm_ta[THERM_55].x)
	    polling_sec_time = TIME_POLL_0P5_MINUTE;
	  else if(temp_adc < adcmap_batttherm_ta[THERM_45].x)
	    polling_sec_time = TIME_POLL_10_MINUTE;
	  else if(temp_adc <= adcmap_batttherm_ta[THERM_42].x)
	    polling_sec_time = TIME_POLL_30_MINUTE;
	  else
	    polling_sec_time = TIME_POLL_90_MINUTE;
  }
#endif
  printk(KERN_DEBUG "############ [msm_charger_temp_work] POLL TIME = %d #####################\n", polling_sec_time);

  /* Set the Next Alarm Time */
	msm_charger_program_alarm(polling_sec_time);

	/* prevent suspend before starting the alarm */
	wake_unlock(&msm_chg.temp_wake_lock);
}


static void msm_charger_temperature_battery_alarm(struct alarm *alarm)
{
  printk(KERN_DEBUG "############ [msm_charger_temperature_battery_alarm] #####################\n");
  
	wake_lock(&msm_chg.temp_wake_lock);
	queue_delayed_work(msm_chg.event_wq_thread, &msm_chg.monitor_work, 0);
}
#endif
/*                                     */

static int __devinit msm_charger_probe(struct platform_device *pdev)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  int err;
#endif
	msm_chg.dev = &pdev->dev;
	if (pdev->dev.platform_data) {
		unsigned int milli_secs;

		struct msm_charger_platform_data *pdata
		    =
		    (struct msm_charger_platform_data *)pdev->dev.platform_data;

		milli_secs = pdata->safety_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.safety_time = milli_secs;

#ifdef CONFIG_MACH_LGE_120_BOARD
		milli_secs = pdata->update_time * 30 * MSEC_PER_SEC;
#else
		milli_secs = pdata->update_time * 60 * MSEC_PER_SEC;
#endif
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.update_time = milli_secs;

		msm_chg.max_voltage = pdata->max_voltage;
		msm_chg.min_voltage = pdata->min_voltage;
		msm_chg.get_batt_capacity_percent =
		    pdata->get_batt_capacity_percent;
	}
	if (msm_chg.safety_time == 0)
		msm_chg.safety_time = CHARGING_TEOC_MS;
	if (msm_chg.update_time == 0)
		msm_chg.update_time = UPDATE_TIME_MS;
	if (msm_chg.max_voltage == 0)
		msm_chg.max_voltage = DEFAULT_BATT_MAX_V;
	if (msm_chg.min_voltage == 0)
		msm_chg.min_voltage = DEFAULT_BATT_MIN_V;
	if (msm_chg.get_batt_capacity_percent == NULL)
		msm_chg.get_batt_capacity_percent =
		    msm_chg_get_batt_capacity_percent;

	msm_chg.processing_delay = 1000;  /*                                   */
	mutex_init(&msm_chg.status_lock);
	INIT_DELAYED_WORK(&msm_chg.teoc_work, teoc);
	INIT_DELAYED_WORK(&msm_chg.update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&msm_chg.update_batt_remove_work, update_battery_remove_work);  /*                                   */

#ifdef ACC_ENABLE
	f120_acc_data.charger_fixed = FALSE;
	f120_acc_data.charger_step = Step_1_charging_450mA;
	f120_acc_data.wait_cnt = 0;
	f120_acc_data.vchg_good_cnt = 0;
#endif

/*                                     */
#ifdef CONFIG_LGE_PM_TEMPERATURE_MONITOR
  INIT_DELAYED_WORK(&msm_chg.monitor_work, msm_charger_temp_work);

	/* init to something sane */
	msm_chg.last_poll = alarm_get_elapsed_realtime();

  wake_lock_init(&msm_chg.temp_wake_lock, WAKE_LOCK_SUSPEND, "msm_temp");

  alarm_init(&msm_chg.alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			msm_charger_temperature_battery_alarm);
  wake_lock(&msm_chg.temp_wake_lock);

  queue_delayed_work(msm_chg.event_wq_thread, &msm_chg.monitor_work, 1000);
#endif
/*                                     */

#ifdef CONFIG_LGE_PM
	 wake_lock_init(&uevent_wake_lock, WAKE_LOCK_SUSPEND, "charge_uevent");

     wake_lock_init(&adc_wake_lock, WAKE_LOCK_SUSPEND, "pmic8058_adc");
#endif

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
  err = device_create_file(&pdev->dev, &dev_attr_at_charge);
  err = device_create_file(&pdev->dev, &dev_attr_at_chcomp);
  err = device_create_file(&pdev->dev, &dev_attr_at_fuelrst);
  err = device_create_file(&pdev->dev, &dev_attr_at_fuelval);
  err = device_create_file(&pdev->dev, &dev_attr_at_pmrst);
  err = device_create_file(&pdev->dev, &dev_attr_at_batl);
#endif

	wake_lock_init(&msm_chg.wl, WAKE_LOCK_SUSPEND, "msm_charger");
	return 0;
}

static int __devexit msm_charger_remove(struct platform_device *pdev)
{
#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
    device_remove_file(&pdev->dev, &dev_attr_at_charge);
    device_remove_file(&pdev->dev, &dev_attr_at_chcomp);
    device_remove_file(&pdev->dev, &dev_attr_at_fuelrst);
    device_remove_file(&pdev->dev, &dev_attr_at_fuelval);
    device_remove_file(&pdev->dev, &dev_attr_at_pmrst);
    device_remove_file(&pdev->dev, &dev_attr_at_batl);
#endif

	wake_lock_destroy(&msm_chg.wl);
	mutex_destroy(&msm_chg.status_lock);
	power_supply_unregister(&msm_psy_batt);
	return 0;
}

int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	msm_chg_enqueue_event(hw_chg, event);
	queue_work(msm_chg.event_wq_thread, &msm_chg.queue_work);
	return 0;
}
EXPORT_SYMBOL(msm_charger_notify_event);

int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;
	int rc = 0;

	if (!msm_chg.inited) {
		pr_err("%s: msm_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (hw_chg->start_charging == NULL
		|| hw_chg->stop_charging == NULL
		|| hw_chg->name == NULL
		|| hw_chg->rating == 0) {
		pr_err("%s: invalid hw_chg\n", __func__);
		return -EINVAL;
	}

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (priv == NULL) {
		dev_err(msm_chg.dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	priv->psy.name = hw_chg->name;
	if (hw_chg->type == CHG_TYPE_USB)
		priv->psy.type = POWER_SUPPLY_TYPE_USB;
	else
		priv->psy.type = POWER_SUPPLY_TYPE_MAINS;

	priv->psy.supplied_to = msm_power_supplied_to;
	priv->psy.num_supplicants = ARRAY_SIZE(msm_power_supplied_to);
	priv->psy.properties = msm_power_props;
	priv->psy.num_properties = ARRAY_SIZE(msm_power_props);
	priv->psy.get_property = msm_power_get_property;

	rc = power_supply_register(NULL, &priv->psy);
	if (rc) {
		dev_err(msm_chg.dev, "%s power_supply_register failed\n",
			__func__);
		goto out;
	}

	priv->hw_chg = hw_chg;
	priv->hw_chg_state = CHG_ABSENT_STATE;
	INIT_LIST_HEAD(&priv->list);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_add_tail(&priv->list, &msm_chg.msm_hardware_chargers);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	hw_chg->charger_private = (void *)priv;
	return 0;

out:
	kfree(priv);
	return rc;
}
EXPORT_SYMBOL(msm_charger_register);

void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge)
{
	int rc;

	if (msm_batt_gauge) {
		msm_batt_gauge = batt_gauge;
		pr_err("msm-charger %s multiple battery gauge called\n",
								__func__);
	} else {
		rc = power_supply_register(msm_chg.dev, &msm_psy_batt);
		if (rc < 0) {
			dev_err(msm_chg.dev, "%s: power_supply_register failed"
					" rc=%d\n", __func__, rc);
			return;
		}

		msm_batt_gauge = batt_gauge;
		determine_initial_batt_status();
	}
}
EXPORT_SYMBOL(msm_battery_gauge_register);

void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge)
{
	msm_batt_gauge = NULL;
}
EXPORT_SYMBOL(msm_battery_gauge_unregister);

int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;

	priv = (struct msm_hardware_charger_priv *)(hw_chg->charger_private);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_del(&priv->list);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	power_supply_unregister(&priv->psy);
	kfree(priv);
	return 0;
}
EXPORT_SYMBOL(msm_charger_unregister);

static int msm_charger_suspend(struct device *dev)
{
	dev_dbg(msm_chg.dev, "%s suspended\n", __func__);
	msm_chg.stop_update = 1;
	cancel_delayed_work(&msm_chg.update_heartbeat_work);
	mutex_lock(&msm_chg.status_lock);
	handle_battery_removed();
#ifdef CONFIG_LGE_PM_BATTERY_ALARM 
	pm8xxx_batt_alarm_config_lge();
#endif
	mutex_unlock(&msm_chg.status_lock);
	return 0;
}

static int msm_charger_resume(struct device *dev)
{
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
    int soc;
#endif
	dev_dbg(msm_chg.dev, "%s resumed\n", __func__);
	msm_chg.stop_update = 0;
	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
	msm_chg.chg_current_set = 0; //                                   
	mutex_lock(&msm_chg.status_lock);
	handle_battery_inserted();

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
	/*                                                     */
	msm_charger_notify_event(NULL, CHG_BATT_REFRESH_EVENT); /*                                  */
	soc = max17040_get_battery_capacity_percent();
#if 0 /*                                          */
	pr_info("resume wake up fuel gauge is %d\n", soc);
	if (soc ==14 || soc == 4 || soc == 0 ) {
#else
	//pr_info("resume wake up fuel gauge is %d\n", soc);
	if (soc ==15 || soc == 5) {
#endif
		pr_info("wake lock.............\n");
	}
#if 1
	pm8xxx_batt_alarm_config_lge();
#else
	else if (!is_chg_plugged_in()) {
		pm8xxx_batt_alarm_config_lge();
	}
	else
		pm8xxx_batt_alarm_enable(PM8XXX_BATT_ALARM_LOWER_COMPARATOR);
#endif	
#endif

	mutex_unlock(&msm_chg.status_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(msm_charger_pm_ops,
		msm_charger_suspend, msm_charger_resume);

static struct platform_driver msm_charger_driver = {
	.probe = msm_charger_probe,
	.remove = __devexit_p(msm_charger_remove),
	.driver = {
		   .name = "msm-charger",
		   .owner = THIS_MODULE,
		   .pm = &msm_charger_pm_ops,
	},
};

static int __init msm_charger_init(void)
{
	int rc;

	INIT_LIST_HEAD(&msm_chg.msm_hardware_chargers);
	msm_chg.count_chargers = 0;
	mutex_init(&msm_chg.msm_hardware_chargers_lock);

	msm_chg.queue = kzalloc(sizeof(struct msm_charger_event)
				* MSM_CHG_MAX_EVENTS,
				GFP_KERNEL);
	if (!msm_chg.queue) {
		rc = -ENOMEM;
		goto out;
	}
	msm_chg.tail = 0;
	msm_chg.head = 0;
	spin_lock_init(&msm_chg.queue_lock);
	msm_chg.queue_count = 0;
	INIT_WORK(&msm_chg.queue_work, process_events);
	msm_chg.event_wq_thread = create_workqueue("msm_charger_eventd");
	if (!msm_chg.event_wq_thread) {
		rc = -ENOMEM;
		goto free_queue;
	}
	rc = platform_driver_register(&msm_charger_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto destroy_wq_thread;
	}
	msm_chg.inited = 1;
	return 0;

destroy_wq_thread:
	destroy_workqueue(msm_chg.event_wq_thread);
free_queue:
	kfree(msm_chg.queue);
out:
	return rc;
}

static void __exit msm_charger_exit(void)
{
	flush_workqueue(msm_chg.event_wq_thread);
	destroy_workqueue(msm_chg.event_wq_thread);
	kfree(msm_chg.queue);
	platform_driver_unregister(&msm_charger_driver);
}

module_init(msm_charger_init);
module_exit(msm_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Abhijeet Dharmapurikar <adharmap@codeaurora.org>");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
