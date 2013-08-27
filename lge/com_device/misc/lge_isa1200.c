/*
 *  lge/com_device/misc/lge_isa1200.c
 *
 *  isa1200 Haptic Motor Device Driver.
 *
 * Copyright (C) 2011 LGE, Inc.
 * Author: Yoon Gi Souk <gisouk.yoon@lge.com>
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
 */

/* #define DEBUG */

/*
  Config for pre initial mode
  1. The added mode will reduce vibrator's latency.
  2. Because, hen/len gpios are on during non-suspended mode,
    the mode may consume power a little bit more.
*/

#define CONFIG_PRE_INITIAL_CODE

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

#include <linux/delay.h>
#include <linux/hrtimer.h>

#include <linux/mutex.h>
#include <linux/debugfs.h>

#include <../../../drivers/staging/android/timed_output.h>


#include "../../include/lge_isa1200.h"

/* Note

  1. currently only one gpio mode is supported, which means HEN and LEN share one line.
  2. LPA motor is assumed.
  3. PWM generated mode is used.
*/

struct lge_isa1200_context {
	struct i2c_client *client;
	struct timed_output_dev dev;
	struct lge_isa1200_platform_data *pdata;
	struct hrtimer timer;
	struct work_struct work;
	atomic_t vibe_level;
	int enable;
/*
	struct mutex lock;
*/
};

static struct workqueue_struct *local_workqueue;

static struct lge_isa1200_context *context_for_debugfs = NULL;

static bool lge_is_one_gpio_mode_or_not(struct lge_isa1200_context *context)
{
	return (context->pdata->gpio_hen == context->pdata->gpio_len);
}

#ifdef CONFIG_DEBUG_FS
static int lge_isa1200_read_reg(struct i2c_client *client, u8 addr)
{
	s32 value = i2c_smbus_read_byte_data (client, addr);

	dev_dbg(&client->dev, "%s %02x:%02x (negative value means error code)\n", __func__, addr, value);

	return value;
}
#endif

static int lge_isa1200_write_reg(struct i2c_client *client, u8 addr, u8 value)
{
	int ret;

	dev_dbg(&client->dev, "%s %02x:%02x\n", __func__, addr, value);

	ret = i2c_smbus_write_byte_data(client, addr, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int lge_isa1200_hw_init(struct lge_isa1200_context *context)
{
#ifdef DEBUG
	unsigned int value;
#endif
	struct i2c_client *client = context->client;

	dev_info(context->dev.dev, "%s()\n", __func__);

#ifdef CONFIG_PRE_INITIAL_CODE
	gpio_set_value_cansleep(context->pdata->gpio_hen, 1);
#endif

#ifdef DEBUG
	value = lge_isa1200_read_reg(client, LGE_ISA1200_HCTRL1);
	dev_dbg(context->dev.dev, "%s() reg[%02x]=0x%02x\n", __func__, LGE_ISA1200_HCTRL1, value);
#endif

	if (context->pdata->init_seq) {
		int i;
		struct isa1200_reg_seq *init_seq=context->pdata->init_seq;

		for (i=0; i<init_seq->number_of_reg_cmd_list; i++) {
			lge_isa1200_write_reg(client, init_seq->reg_cmd_list[i].addr, init_seq->reg_cmd_list[i].data);
		}
	} else {
		dev_err(context->dev.dev, "%s() no initialization sequence\n", __func__);
	}

#ifdef DEBUG
	value = lge_isa1200_read_reg(client, LGE_ISA1200_HCTRL1);
	dev_dbg(context->dev.dev, "%s() reg[%02x]=0x%02x\n", __func__, LGE_ISA1200_HCTRL1, value);

	value = lge_isa1200_read_reg(client, LGE_ISA1200_HCTRL3);
	dev_dbg(context->dev.dev, "%s() reg[%02x]=0x%02x\n", __func__, LGE_ISA1200_HCTRL3, value);
#endif

	return 0;
}

static int lge_isa1200_hw_vib_on_off(struct lge_isa1200_context *context, bool on_off)
{
	struct i2c_client *client = context->client;

	dev_info(context->dev.dev, "%s(%d), level = %d\n", __func__, on_off, atomic_read(&(context->vibe_level)));

	if (on_off)	{
		if (context->pdata->clock) context->pdata->clock(true, atomic_read(&(context->vibe_level)));
#ifndef CONFIG_PRE_INITIAL_CODE
		gpio_set_value_cansleep(context->pdata->gpio_hen, 1);
		lge_isa1200_hw_init(context);
#endif
		lge_isa1200_write_reg(client, LGE_ISA1200_HCTRL5, atomic_read(&(context->vibe_level)) );	/* [7:0] PWM High Duty(PWM Gen) 0-6B-D6 */
		lge_isa1200_write_reg(client, LGE_ISA1200_HCTRL0, 0x10 + (on_off<<7));						/* [7]Haptic Drive Enable Mode */
	} else {
#ifdef CONFIG_PRE_INITIAL_CODE
		lge_isa1200_write_reg(client, LGE_ISA1200_HCTRL0, 0x10 + (on_off<<7));            /* [7]Haptic Drive Enable Mode */
#else
		gpio_set_value_cansleep(context->pdata->gpio_hen, 0);
#endif
		if (context->pdata->clock) context->pdata->clock(false, 0);
	}

	return 0;
}

static void lge_isa1200_vibrator_work_func(struct work_struct *work)
{
	struct lge_isa1200_context *context = container_of(work, struct lge_isa1200_context, work);

	dev_dbg(context->dev.dev, "%s()\n", __func__);

	lge_isa1200_hw_vib_on_off(context, context->enable);
}

static enum hrtimer_restart lge_isa1200_vibrator_timer_func(struct hrtimer *timer)
{
	struct lge_isa1200_context *context = container_of(timer, struct lge_isa1200_context, timer);

	dev_dbg(context->dev.dev, "%s()\n", __func__);

	context->enable = false;
#if 0
	schedule_work(&context->work);
#else
	queue_work(local_workqueue,&context->work);
#endif

	return HRTIMER_NORESTART;
}

static int lge_isa1200_vibrator_get_time(struct timed_output_dev *dev)
{
	struct lge_isa1200_context *context = container_of(dev, struct lge_isa1200_context, dev);

	dev_info(dev->dev, "%s()\n", __func__);

	if (hrtimer_active(&context->timer)) {
		ktime_t r = hrtimer_get_remaining(&context->timer);
		return ktime_to_ms(r);
	} else
		return 0;
}

static void lge_isa1200_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct lge_isa1200_context *context = container_of(dev, struct lge_isa1200_context, dev);
/*
	int ret;
*/

	dev_info(dev->dev, "%s(%d)\n", __func__, value);
/*
	ret = mutex_lock_interruptible(&context->lock);
	if (ret > 0) {
		dev_err(dev->dev, "mutex_lock_interruptible is failed with cause=%d\n", ret);
		return;
	}
*/
	hrtimer_cancel(&context->timer);
	cancel_work_sync(&context->work);

	if (value > 0) {
		if  (value > context->pdata->max_timeout) value = context->pdata->max_timeout;

		hrtimer_start(&context->timer, ktime_set(value / 1000, (value % 1000) * 1000000), HRTIMER_MODE_REL);

		context->enable = true;
	} else {
		context->enable = false;
	}
/*
	mutex_unlock(&context->lock);
*/
#if 0
	schedule_work(&context->work);
#else
	queue_work(local_workqueue,&context->work);
#endif
}

static ssize_t lge_isa1200_vibrator_amp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *dev_ =(struct timed_output_dev *)dev_get_drvdata(dev);
	struct lge_isa1200_context *context = container_of(dev_, struct lge_isa1200_context, dev);

	dev_info(dev, "%s()\n", __func__);

    return sprintf(buf, "%d\n", atomic_read(&(context->vibe_level)));
}

static ssize_t lge_isa1200_vibrator_amp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct timed_output_dev *dev_ =(struct timed_output_dev *)dev_get_drvdata(dev);
	struct lge_isa1200_context *vib = container_of(dev_, struct lge_isa1200_context, dev);
    int gain;

	dev_info(dev, "%s(%s)\n", __func__, buf);

    sscanf(buf, "%d", &gain);

    /* TODO range check */
	dev_info(dev, "vib_gain is set by value=%d\n", gain);

    atomic_set(&vib->vibe_level, gain);

    return size;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <asm/uaccess.h>

static struct dentry *debugfs_timpani_dent;
static struct dentry *debugfs_poke;
static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (strict_strtoul(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
			}
		else
			return -EINVAL;
	}
	return 0;
}


static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc;
	long int param[5];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;
	lbuf[cnt] = '\0';

  if (!strcmp(access_str, "poke")) {
		rc = get_parameters(lbuf, param, 2);

      switch(param[0]){
        case 1:
        {
          //mount -t debugfs debugfs /sys/kernel/debug;echo 1 0 > /sys/kernel/debug/lgvib/poke
          int reg = 0;
          int val = 0;
          struct i2c_client *client = context_for_debugfs->client;

          printk(KERN_INFO "LGE:%s() reg read 0x%x(%d)\n", __func__, (int)param[1], (int)param[1]);

          //debug read
          reg = 0x30; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x31; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x32; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x33; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x34; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x35; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
          reg = 0x36; val = lge_isa1200_read_reg(client,reg); printk(KERN_INFO "LGE:%s() 0x%x:0x%x(%d)\n", __func__, reg, val, val);
        }
        break;
        case 2:
        {
          //lge_isa1200_write_reg(client, 0x35, (unsigned char)param[1]);    // [7:0] PWM High Duty(PWM Gen) 0-6B-D6
        }
        case 3:
        {
          //lge_isa1200_write_reg(client, 0x30, VAL_0x30+(param[1]<<7));    	// [7]Haptic Drive Enable Mode
        }
        break;
      }

	}

	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};
#endif



static DEVICE_ATTR(amp, S_IRUGO | S_IWUSR, lge_isa1200_vibrator_amp_show, lge_isa1200_vibrator_amp_store);

static int __devinit lge_isa1200_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lge_isa1200_context *context;
	struct lge_isa1200_platform_data *pdata;
	int ret = 0;

	dev_info(&client->dev, "%s()\n", __func__);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
		return -EIO;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	context = kzalloc(sizeof(struct lge_isa1200_context), GFP_KERNEL);
	if (!context) {
		ret = -ENOMEM;
		goto fail_1;
	}

	context->client = client;
	context->pdata = pdata;

	i2c_set_clientdata(client, context);

	/* init basic things : mutex, work, timer */
/*
	mutex_init(&context->lock);
*/
	INIT_WORK(&context->work, lge_isa1200_vibrator_work_func);

	hrtimer_init(&context->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	context->timer.function = lge_isa1200_vibrator_timer_func;

	/* gpio initialization */
	if (lge_is_one_gpio_mode_or_not(context)) {

		dev_err(&client->dev, "%s: lge_is_one_gpio_mode_or_not gpio_hen=%d\n", __func__, pdata->gpio_hen);

		ret = gpio_request(pdata->gpio_hen, "gpio_hen");
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d request failed\n",
					__func__, pdata->gpio_hen);
			goto fail_2;
		}
		gpio_direction_output(pdata->gpio_hen, 0);
	} else {
		dev_err(&client->dev, "%s: invalid gpio configure\n", __func__);
		ret = -EINVAL;
		goto fail_2;
	}


    atomic_set(&context->vibe_level, pdata->default_vib_strength);

	/* register timed output device */
	context->dev.name = pdata->vibrator_name;

	context->dev.enable = lge_isa1200_vibrator_enable;
	context->dev.get_time = lge_isa1200_vibrator_get_time;

	ret = timed_output_dev_register(&context->dev);
	if (ret < 0) goto fail_3;

    /* turn off vibrator (initialization) */
#ifdef CONFIG_PRE_INITIAL_CODE
	lge_isa1200_hw_init(context);
#endif
	lge_isa1200_hw_vib_on_off(context, false);

	ret = device_create_file(context->dev.dev, &dev_attr_amp);
	if (ret < 0) goto fail_4;

	context_for_debugfs = context;

#ifdef CONFIG_DEBUG_FS
        debugfs_timpani_dent = debugfs_create_dir("lgvib", 0);
        if (!IS_ERR(debugfs_timpani_dent)) {
          debugfs_poke = debugfs_create_file("poke",
          S_IFREG | S_IRUGO, debugfs_timpani_dent,
          (void *) "poke", &codec_debug_ops);
        }
#endif

	return 0;

fail_4:
	timed_output_dev_unregister(&context->dev);
fail_3:
	gpio_free(context->pdata->gpio_hen);
fail_2:
#if 0 /*                                        */
	mutex_destroy(&context->mutex);
#endif
	i2c_set_clientdata(client, NULL);
	kfree(context);
fail_1:
	return ret;
}

static int __devexit lge_isa1200_remove(struct i2c_client *client)
{
	struct lge_isa1200_context *context = i2c_get_clientdata(client);

	dev_info(context->dev.dev, "%s()\n", __func__);

	context_for_debugfs = NULL;

	device_remove_file(context->dev.dev, &dev_attr_amp);

	hrtimer_cancel(&context->timer);
	cancel_work_sync(&context->work);
	lge_isa1200_hw_vib_on_off(context, false);

	timed_output_dev_unregister(&context->dev);

	gpio_free(context->pdata->gpio_hen);

#if 0 /*                                        */
	mutex_destroy(&context->mutex);
#endif
	i2c_set_clientdata(client, NULL);
	kfree(context);

	return 0;
}

#ifdef CONFIG_PM
static int lge_isa1200_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lge_isa1200_context *context = i2c_get_clientdata(client);

	hrtimer_cancel(&context->timer);
	cancel_work_sync(&context->work);

	lge_isa1200_hw_vib_on_off(context, false);

#ifdef CONFIG_PRE_INITIAL_CODE
	gpio_set_value_cansleep(context->pdata->gpio_hen, 0);
#endif
	return 0;
}

static int lge_isa1200_resume(struct i2c_client *client)
{
#ifdef CONFIG_PRE_INITIAL_CODE
	struct lge_isa1200_context *context = i2c_get_clientdata(client);

	lge_isa1200_hw_init(context);
#endif
	return 0;
}
#endif

static struct i2c_device_id lge_isa1200_idtable[] = {
	{ "lge_isa1200", 0 }, // TODO slave id 0?
	{ },
};

MODULE_DEVICE_TABLE(i2c, lge_isa1200_idtable);

static struct i2c_driver lge_isa1200_driver = {
	.probe = lge_isa1200_probe,
	.remove = __devexit_p(lge_isa1200_remove),
#ifdef CONFIG_PM
	.suspend = lge_isa1200_suspend,
	.resume	 = lge_isa1200_resume,
#endif
	.id_table = lge_isa1200_idtable,
	.driver = {
		.name = "lge_isa1200",
	},
};

static int __init lge_isa1200_init(void)
{
	pr_debug("%s", __func__);

	local_workqueue = create_workqueue("lge_isa1200") ;
	if(!local_workqueue)
		return -ENOMEM;
	return i2c_add_driver(&lge_isa1200_driver);
}

static void __exit lge_isa1200_exit(void)
{
	pr_debug("%s", __func__);
	if(local_workqueue)
		destroy_workqueue(local_workqueue);
	local_workqueue = NULL;

	i2c_del_driver(&lge_isa1200_driver);
}

/* to let init lately */
late_initcall_sync(lge_isa1200_init);
module_exit(lge_isa1200_exit);

MODULE_AUTHOR("Yoon Gi Souk <gisouk.yoon@lge.com>");
MODULE_DESCRIPTION("LGE ISA1200 driver");
MODULE_LICENSE("GPL");

