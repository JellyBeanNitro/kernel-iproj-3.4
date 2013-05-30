/* Copyright (c) 2010, 2011, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/leds-pmic8058.h>
#include <linux/module.h>
#include <linux/string.h>

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                             */
#include <linux/delay.h>
#include <linux/device.h>
//                                                                                      
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include "../../lge/include/board_lge.h"
//                                                                                      
#endif /* ==================================================================================== */

#define SSBI_REG_ADDR_DRV_KEYPAD	0x48
#define PM8058_DRV_KEYPAD_BL_MASK	0xf0
#define PM8058_DRV_KEYPAD_BL_SHIFT	0x04

#define SSBI_REG_ADDR_FLASH_DRV0        0x49
#define PM8058_DRV_FLASH_MASK           0xf0
#define PM8058_DRV_FLASH_SHIFT          0x04

#define SSBI_REG_ADDR_FLASH_DRV1        0xFB

#define SSBI_REG_ADDR_LED_CTRL_BASE	0x131
#define SSBI_REG_ADDR_LED_CTRL(n)	(SSBI_REG_ADDR_LED_CTRL_BASE + (n))
#define PM8058_DRV_LED_CTRL_MASK	0xf8
#define PM8058_DRV_LED_CTRL_SHIFT	0x03

#define MAX_FLASH_CURRENT	300
#define MAX_KEYPAD_CURRENT 300
#define MAX_KEYPAD_BL_LEVEL	(1 << 4)
#define MAX_LED_DRV_LEVEL	20 /* 2 * 20 mA */

#define PMIC8058_LED_OFFSET(id) ((id) - PMIC8058_ID_LED_0)

struct pmic8058_led_data {
	struct device		*dev;
	struct led_classdev	cdev;
	int			id;
	enum led_brightness	brightness;
	u8			flags;
	struct work_struct	work;
	struct mutex		lock;
	spinlock_t		value_lock;
#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                             */
	//                                                                                      
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	bool is_kp_bl_enabled;
	//                                                                                      
#endif /* =================================================================================== */
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;
};

#define PM8058_MAX_LEDS		7
static struct pmic8058_led_data led_data[PM8058_MAX_LEDS];

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                       */
static int g_led_off;
static int g_led_brightness=2;
static void led_lc_set(struct pmic8058_led_data *led, enum led_brightness value);
#endif

#ifdef CONFIG_LGE_PM_FACTORY_CURRENT_DOWN	/*                                        */
extern uint16_t battery_info_get(void);
extern int usb_cable_info;
#endif

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)	/*                                        */
extern u8 pwrkey_led_status;
#endif

static void kp_bl_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                             */
	//                                                                                      
	if (value != 0) {
		if (led->is_kp_bl_enabled == false) {
			printk("%s : Key LED is disable now\n", __FUNCTION__);
			return;
		}
	}
	printk("[Touch key backlight] %s value : %d\n", __FUNCTION__, value);
	//                                                                                      
#endif /* =================================================================================== */

#ifdef CONFIG_LGE_PM_FACTORY_CURRENT_DOWN	/*                                        */
	if ((0 == battery_info_get()) && 
		((usb_cable_info == 6) || (usb_cable_info == 7) || (usb_cable_info == 11)))
	{
		pr_err("%s: can't set keypad backlight level in factory mode \n", __func__);
		return;
	}
#endif

#ifdef CONFIG_LGE_PMIC8058_KEYPAD	/*                                        */
	if (value > led->cdev.max_brightness)
		value = led->cdev.max_brightness;
#endif

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_KEYPAD_BL_SHIFT) &
				 PM8058_DRV_KEYPAD_BL_MASK;

	led->reg_kp &= ~PM8058_DRV_KEYPAD_BL_MASK;
	led->reg_kp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_DRV_KEYPAD,
						led->reg_kp);
	if (rc)
		pr_err("%s: can't set keypad backlight level\n", __func__);
}

static enum led_brightness kp_bl_get(struct pmic8058_led_data *led)
{
	if ((led->reg_kp & PM8058_DRV_KEYPAD_BL_MASK) >>
			 PM8058_DRV_KEYPAD_BL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
}

static void led_lc_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	unsigned long flags;
	int rc, offset;
	u8 level, tmp;

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                             */
	//                                                                                      
	if (value != 0) {
		if (led->is_kp_bl_enabled == false) {
			printk("%s : Key LED is disable now\n", __FUNCTION__);
			return;
		}

		value = g_led_brightness; //led->brightness = 5;
	}
	led->brightness = value;
	printk("[Touch key backlight] %s value : %d\n", __FUNCTION__, led->brightness);
	//                                                                                      
#endif /* =================================================================================== */

	spin_lock_irqsave(&led->value_lock, flags);

	level = (led->brightness << PM8058_DRV_LED_CTRL_SHIFT) &
		PM8058_DRV_LED_CTRL_MASK;

	offset = PMIC8058_LED_OFFSET(led->id);
	tmp = led->reg_led_ctrl[offset];

	tmp &= ~PM8058_DRV_LED_CTRL_MASK;
	tmp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)	/*                                        */
	if ((led->id == PMIC8058_ID_LED_0) && ( pwrkey_led_status >= 1)) {
		return;
	}
#endif

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_LED_CTRL(offset),
								tmp);
	if (rc) {
		dev_err(led->cdev.dev, "can't set (%d) led value\n",
				led->id);
		return;
	}

	spin_lock_irqsave(&led->value_lock, flags);
	led->reg_led_ctrl[offset] = tmp;
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static enum led_brightness led_lc_get(struct pmic8058_led_data *led)
{
	int offset;
	u8 value;

	offset = PMIC8058_LED_OFFSET(led->id);
	value = led->reg_led_ctrl[offset];

	if ((value & PM8058_DRV_LED_CTRL_MASK) >>
			PM8058_DRV_LED_CTRL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
}

static void
led_flash_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;
	u8 reg_flash_led;
	u16 reg_addr;

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_FLASH_SHIFT) &
				 PM8058_DRV_FLASH_MASK;

	if (led->id == PMIC8058_ID_FLASH_LED_0) {
		led->reg_flash_led0 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led0 |= level;
		reg_flash_led	    = led->reg_flash_led0;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV0;
	} else {
		led->reg_flash_led1 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led1 |= level;
		reg_flash_led	    = led->reg_flash_led1;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV1;
	}
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8xxx_writeb(led->dev->parent, reg_addr, reg_flash_led);
	if (rc)
		pr_err("%s: can't set flash led%d level %d\n", __func__,
			led->id, rc);
}

int pm8058_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;

	if ((id < PMIC8058_ID_FLASH_LED_0) || (id > PMIC8058_ID_FLASH_LED_1)) {
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	if (mA > MAX_FLASH_CURRENT)
		return -EINVAL;

	led_flash_set(led, mA / 20);

	return 0;
}
EXPORT_SYMBOL(pm8058_set_flash_led_current);

int pm8058_set_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;
	int brightness = 0;

	if ((id < PMIC8058_ID_LED_KB_LIGHT) || (id > PMIC8058_ID_FLASH_LED_1)) {
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	switch (id) {
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		brightness = mA / 2;
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		led_lc_set(led, brightness);
		break;

	case PMIC8058_ID_LED_KB_LIGHT:
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		brightness = mA / 20;
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		if (id == PMIC8058_ID_LED_KB_LIGHT)
			kp_bl_set(led, brightness);
		else
			led_flash_set(led, brightness);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_set_led_current);

static void pmic8058_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pmic8058_led_data *led;
	unsigned long flags;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	spin_lock_irqsave(&led->value_lock, flags);
	led->brightness = value;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static void pmic8058_led_work(struct work_struct *work)
{
	struct pmic8058_led_data *led = container_of(work,
					 struct pmic8058_led_data, work);

#if 1 /*                                                           */
#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
#ifdef CONFIG_MACH_LGE_120_BOARD_SKT
	if (lge_bd_rev > LGE_REV_13) //Rev_13(i) = Rev_B (cayman)
#else
	if (lge_bd_rev > LGE_REV_16) //Rev_16(i) =Rev_E (cayman)
#endif
		led->id = PMIC8058_ID_LED_2;
	else
		led->id = PMIC8058_ID_LED_KB_LIGHT;
#else
/*
                             
                              
     
                                     
*/
#endif
#ifdef CONFIG_MACH_LGE_I_BOARD_LGU
	if (lge_bd_rev > LGE_REV_B)
		led->id = PMIC8058_ID_LED_2;
	else
		led->id = PMIC8058_ID_LED_KB_LIGHT;
#endif
#endif /* ================================================================================= */

	mutex_lock(&led->lock);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		kp_bl_set(led, led->brightness);
		break;
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		led_lc_set(led, led->brightness);
		break;
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		led_flash_set(led, led->brightness);
		break;
	}

	mutex_unlock(&led->lock);
}

static enum led_brightness pmic8058_led_get(struct led_classdev *led_cdev)
{
	struct pmic8058_led_data *led;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		return kp_bl_get(led);
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		return led_lc_get(led);
	}
	return LED_OFF;
}

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                             */
//                                                                                      
static void pmic8058_led_early_suspend(struct early_suspend *es);
static void pmic8058_led_late_resume(struct early_suspend *es);

static ssize_t store_led_off(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	g_led_off = simple_strtoul(buf, NULL, 10);
	return size;
}

static ssize_t show_led_off(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_led_off);
}

static ssize_t store_led_brightness(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	g_led_brightness = simple_strtoul(buf, NULL, 10);
	return size;
}

static ssize_t show_led_brightness(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_led_brightness);
}

static DEVICE_ATTR(led_off, S_IRUGO | S_IWUSR, show_led_off, store_led_off);
static DEVICE_ATTR(led_brightness, S_IRUGO | S_IWUSR, show_led_brightness, store_led_brightness);

static struct attribute *pmic8058_led_attributes[] = {
	&dev_attr_led_off.attr,
	&dev_attr_led_brightness.attr,
	NULL
};

static const struct attribute_group pmic8058_led_group = {
	.attrs = pmic8058_led_attributes,
};
#endif /* =================================================================================== */

static int pmic8058_led_probe(struct platform_device *pdev)
{
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led_dat;
	struct pmic8058_led *curr_led;
	int rc, i = 0;
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -EINVAL;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_DRV_KEYPAD, &reg_kp);
	if (rc) {
		dev_err(&pdev->dev, "can't get keypad backlight level\n");
		goto err_reg_read;
	}

	rc = pm8xxx_read_buf(pdev->dev.parent, SSBI_REG_ADDR_LED_CTRL_BASE,
							reg_led_ctrl, 3);
	if (rc) {
		dev_err(&pdev->dev, "can't get led levels\n");
		goto err_reg_read;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_FLASH_DRV0,
						&reg_flash_led0);
	if (rc) {
		dev_err(&pdev->dev, "can't read flash led0\n");
		goto err_reg_read;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_FLASH_DRV1,
						&reg_flash_led1);
	if (rc) {
		dev_err(&pdev->dev, "can't get flash led1\n");
		goto err_reg_read;
	}

	for (i = 0; i < pdata->num_leds; i++) {
		curr_led	= &pdata->leds[i];
		led_dat		= &led_data[curr_led->id];

		led_dat->cdev.name		= curr_led->name;
		led_dat->cdev.default_trigger   = curr_led->default_trigger;
		led_dat->cdev.brightness_set    = pmic8058_led_set;
		led_dat->cdev.brightness_get    = pmic8058_led_get;
		led_dat->cdev.brightness	= LED_OFF;
		led_dat->cdev.max_brightness	= curr_led->max_brightness;
		led_dat->cdev.flags		= LED_CORE_SUSPENDRESUME;

		led_dat->id		        = curr_led->id;
		led_dat->reg_kp			= reg_kp;
		memcpy(led_data->reg_led_ctrl, reg_led_ctrl,
					 sizeof(reg_led_ctrl));
		led_dat->reg_flash_led0		= reg_flash_led0;
		led_dat->reg_flash_led1		= reg_flash_led1;

		if (!((led_dat->id >= PMIC8058_ID_LED_KB_LIGHT) &&
				(led_dat->id <= PMIC8058_ID_FLASH_LED_1))) {
			dev_err(&pdev->dev, "invalid LED ID (%d) specified\n",
						 led_dat->id);
			rc = -EINVAL;
			goto fail_id_check;
		}

		led_dat->dev			= &pdev->dev;

#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_325_BOARD_LGU)	/*                                        */
		if (led_dat->id == PMIC8058_ID_LED_2)
			led_lc_set(led_dat, 0);
		else
			kp_bl_set(led_dat, 0);
#endif

#if defined(CONFIG_MACH_LGE_325_BOARD_DCM)	/*                                        */
		led_lc_set(led_dat, 0);
#endif

		mutex_init(&led_dat->lock);
		spin_lock_init(&led_dat->value_lock);
		INIT_WORK(&led_dat->work, pmic8058_led_work);

		rc = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (rc) {
			dev_err(&pdev->dev, "unable to register led %d\n",
						 led_dat->id);
			goto fail_id_check;
		}

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                            */
		led_dat->is_kp_bl_enabled = true;
		if (led_dat->id == PMIC8058_ID_LED_KB_LIGHT) {
			//                                                                                      
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
			led_dat->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
			led_dat->early_suspend.suspend = pmic8058_led_early_suspend;
			led_dat->early_suspend.resume = pmic8058_led_late_resume;
			register_early_suspend(&led_dat->early_suspend);
#endif
			//                                                                                      
			if (lge_bd_rev > LGE_REV_D)
				kp_bl_set(led_dat, 0);
		}
#endif /* ================================================================================== */
	}

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                            */
	rc = sysfs_create_group(&pdev->dev.kobj, &pmic8058_led_group);
	if (rc)
		printk("[LED_PMIC8058] sysfs create group error\n");
#endif

	platform_set_drvdata(pdev, led_data);

	return 0;

err_reg_read:
fail_id_check:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--)
			led_classdev_unregister(&led_data[i].cdev);
	}
	return rc;
}

#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                            */
//                                                                                      
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
static void pmic8058_led_early_suspend(struct early_suspend *es)
{
	struct pmic8058_led_data *led = container_of(es, struct pmic8058_led_data, early_suspend);

	printk("ENTER : %s\n",__FUNCTION__);

#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
#ifdef CONFIG_MACH_LGE_120_BOARD_SKT
	if (lge_bd_rev > LGE_REV_13) //Rev_13(i) = Rev_B (cayman)
#else
	if (lge_bd_rev > LGE_REV_16) //Rev_16(i) =Rev_E (cayman)
#endif
		led->id = PMIC8058_ID_LED_2;
	else
		led->id = PMIC8058_ID_LED_KB_LIGHT;
#else
	if (lge_bd_rev > LGE_REV_13)
		led->id = PMIC8058_ID_LED_2;
	else
		led->id = PMIC8058_ID_LED_KB_LIGHT;
#endif

	led->brightness = LED_OFF;
	//  led->is_kp_bl_enabled = false;
	//  schedule_work(&led->work);

	return;
}

static void pmic8058_led_late_resume(struct early_suspend *es)
{
	struct pmic8058_led_data *led = container_of(es, struct pmic8058_led_data, early_suspend);

	//if(!g_led_off)
	//{
	printk("ENTER : %s\n",__FUNCTION__);

#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
#ifdef CONFIG_MACH_LGE_120_BOARD_SKT
	if (lge_bd_rev > LGE_REV_13) //Rev_13(i) = Rev_B (cayman)
#else
	if (lge_bd_rev > LGE_REV_16) //Rev_16(i) =Rev_E (cayman)
#endif
	{
		led->id = PMIC8058_ID_LED_2;
		led->brightness = 2;
	} else {
		led->id = PMIC8058_ID_LED_KB_LIGHT;
		led->brightness = 1;
	}
#else
	if (lge_bd_rev > LGE_REV_13) {
		led->id = PMIC8058_ID_LED_2;
		led->brightness = 5;
	} else {
		led->id = PMIC8058_ID_LED_KB_LIGHT;
		led->brightness = 1;
	}
#endif

	//      led->is_kp_bl_enabled = true;
	//      msleep(100);
	//schedule_work(&led->work);
	//}
	return;
}
#endif
//                                                                                      
#endif /* ================================================================================== */

static int __devexit pmic8058_led_remove(struct platform_device *pdev)
{
	int i;
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&led[led->id].cdev);
		cancel_work_sync(&led[led->id].work);
	}
#ifdef CONFIG_MACH_LGE_I_BOARD /*                                                            */
	sysfs_remove_group(&pdev->dev.kobj,&pmic8058_led_group);
#endif

	return 0;
}

static struct platform_driver pmic8058_led_driver = {
	.probe		= pmic8058_led_probe,
	.remove		= __devexit_p(pmic8058_led_remove),
	.driver		= {
		.name	= "pm8058-led",
		.owner	= THIS_MODULE,
	},
};

static int __init pmic8058_led_init(void)
{
	return platform_driver_register(&pmic8058_led_driver);
}
module_init(pmic8058_led_init);

static void __exit pmic8058_led_exit(void)
{
	platform_driver_unregister(&pmic8058_led_driver);
}
module_exit(pmic8058_led_exit);

MODULE_DESCRIPTION("PMIC8058 LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-led");
