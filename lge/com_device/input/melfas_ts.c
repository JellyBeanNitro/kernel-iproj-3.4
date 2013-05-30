/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
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


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include "melfas_ts.h"


#define MIP_ENABLE 	1

#define TS_MAX_Z_TOUCH		255
#define TS_MAX_W_TOUCH		255

#define TS_MAX_X_COORD 		768
#define TS_MAX_Y_COORD 		1024
#define FW_VERSION			0x09

#define TS_READ_START_ADDR 	0x10
#define TS_READ_PUBLIC_VERSION_ADDR	0xF5

#if MIP_ENABLE
#define TS_READ_REGS_LEN 		100
#else
#define TS_READ_REGS_LEN 		5
#endif

#define MELFAS_MAX_TOUCH		10



#define I2C_RETRY_CNT			10

#define PRESS_KEY				1
#define RELEASE_KEY				0
#define DEBUG_PRINT 			0
#define	SET_DOWNLOAD_BY_GPIO	1


#if SET_DOWNLOAD_BY_GPIO
//#include <melfas_download.h>
#include "mms100_ISP_download.h"
#endif // SET_DOWNLOAD_BY_GPIO

enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
	int width;
	int posX;
	int posY;
};

struct melfas_ts_data
{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;
	struct delayed_work  work;
	uint32_t flags;
	int (*power)(int on);
	int (*power_enable)(int en, bool log_en);
	struct early_suspend early_suspend;
	char fw_rev;
	char manufcturer_id;
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

int (*g_power_enable) (int en, bool log_en);

static struct workqueue_struct *melfas_wq;

static char tmp_flag[10];

/*static int melfas_init_panel(struct melfas_ts_data *ts)
{
	char buf;
	int ret;

	buf = 0x00;

	ret = i2c_master_send(ts->client, &buf, 1);

	ret = i2c_master_send(ts->client, &buf, 1);

	if(ret <0)
	{
		printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed\n [%d]", ret);
		return 0;
	}


	return true;
}*/
static struct melfas_ts_data *touch_pdev = NULL;
#if 1 // test_mode command
void Send_Touch_Melfas( unsigned int x, unsigned int y)
{
	if(touch_pdev)
	{

		input_report_abs(touch_pdev->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(touch_pdev->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(touch_pdev->input_dev, ABS_MT_PRESSURE, 1);
		input_report_abs(touch_pdev->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(touch_pdev->input_dev, ABS_MT_WIDTH_MINOR, 1);
		input_mt_sync(touch_pdev->input_dev);
		input_sync(touch_pdev->input_dev);

		input_report_abs(touch_pdev->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(touch_pdev->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(touch_pdev->input_dev, ABS_MT_PRESSURE, 0);
		input_report_abs(touch_pdev->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(touch_pdev->input_dev, ABS_MT_WIDTH_MINOR, 0);
		input_mt_sync(touch_pdev->input_dev);
		input_sync(touch_pdev->input_dev);
	}
	else
	{
		printk(KERN_ERR "melfas_ts_data not found\n");
	}
}
EXPORT_SYMBOL(Send_Touch_Melfas);

int get_touch_ts_fw_version_melfas(char *fw_ver)
{

		if(touch_pdev)
		{
			sprintf(fw_ver, "%s.0%d", "Melfas", FW_VERSION);
			return 1;
		}
		else
		{
			return 0;
		}

}

EXPORT_SYMBOL(get_touch_ts_fw_version_melfas);
#endif

static void melfas_ts_work_func(struct work_struct *work)
{
	struct melfas_ts_data *ts = container_of(to_delayed_work(work), struct melfas_ts_data, work);
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	int touchType=0, touchState =0, touchID=0, posX=0, posY=0, width = 0, strength=10, keyID = 0, reportID = 0;
	uint8_t read_num;

	read_num = 0;


#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_work_func\n");

	if(ts ==NULL)
			printk(KERN_ERR "melfas_ts_work_func : TS NULL\n");
#endif

#if !(MIP_ENABLE)
	buf[0] = TS_READ_START_ADDR;
	for(i=0; i<I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, buf, 1);// wrtie
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif
		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN); // read
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);
			printk(KERN_ERR "%d %d %d %d %d\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
#endif
			if(ret >=0)
			{
				break; // i2c success
			}
		}
	}

	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
		return ;
	}
	else
	{
 		touchType  = (buf[0]>>5)&0x03; // MTSY 0.7
		touchState = (buf[0]>>4)&0x01;
		reportID = (buf[0]&0x0f);
		posX = (  ( buf[1]& 0x0F) << (8)  ) +  buf[2];
		posY = (( (buf[1]& 0xF0 ) >> 4 ) << (8)) +  buf[3];
		width = buf[4];
		if(touchType == 2 ) //touch key
		{
			keyID = reportID;
		}
		else
		{
			keyID = reportID;
		}

		touchID = reportID-1;

		if(touchID > MELFAS_MAX_TOUCH-1)
		{
#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d\n",  touchID);
#endif
		return ;
	}

	if(touchType == TOUCH_SCREEN)
	{
		g_Mtouch_info[touchID].posX= posX;
		g_Mtouch_info[touchID].posY= posY;
		g_Mtouch_info[touchID].width= width;

		if(touchState)
			g_Mtouch_info[touchID].strength= strength;
		else {
			g_Mtouch_info[touchID].strength = 0;
			tmp_flag[touchID] = 1;
		}


		for(i=0; i<MELFAS_MAX_TOUCH; i++)
		{
			if(g_Mtouch_info[i].strength== -1)
				continue;

			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength );
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
			input_mt_sync(ts->input_dev);
#if 1	//DEBUG_PRINT
	if ((touchState == 1 && tmp_flag[touchID] ==1) || (touchState ==0 && tmp_flag[touchID] ==1)) {
	printk(KERN_ERR "Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
		i, touchState, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
		if (touchState == 1 )
		 tmp_flag[touchID] = 0;
	}
#endif
			if(g_Mtouch_info[i].strength == 0)
				g_Mtouch_info[i].strength = -1;

		}


	}
	else if(touchType == TOUCH_KEY)
	{
		if (keyID == 0x1)
			input_report_key(ts->input_dev, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);
		if (keyID == 0x2)
			input_report_key(ts->input_dev, KEY_HOME, touchState ? PRESS_KEY : RELEASE_KEY);
		if (keyID == 0x3)
			input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);
		if (keyID == 0x4)
			input_report_key(ts->input_dev, KEY_SEARCH, touchState ? PRESS_KEY : RELEASE_KEY);
#if 1	//DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_work_func: keyID : %d, touchState: %d\n", keyID, touchState);
#endif
	}

	input_sync(ts->input_dev);
}



#else

#define MIP_INPUT_EVENT_PACKET_SIZE	0x0F
#define MIP_INPUT_EVENT_INFORMATION	0x10

	buf[0] = MIP_INPUT_EVENT_PACKET_SIZE;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &read_num, 1);

	if( read_num == 0 ) {
		printk("read number 0 error!!!! \n");
		enable_irq(ts->client->irq);
		return;
	}

	buf[0] = MIP_INPUT_EVENT_INFORMATION;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &buf[0], read_num);

	for (i = 0; i < read_num; i = i + 6) // extract touch information
	{
		if (ret < 0)
		{
			printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
			enable_irq(ts->client->irq);
			return ;
		}
		else
		{
			touchType  =  ((buf[i] & 0x60) >> 5);
			touchState =((buf[i] & 0x80) == 0x80);
			reportID = (buf[i] & 0x0F);
			posX = (uint16_t) (buf[i + 1] & 0x0F) << 8 | buf[i + 2];
			posY = (uint16_t) (buf[i + 1] & 0xF0) << 4 | buf[i + 3];
			width = buf[i + 4];
			if(touchType == 2 ) //touch key
			{
				keyID = reportID;
			}
			else
			{
				keyID = reportID;
			}

			touchID = reportID-1;

			if(touchID > MELFAS_MAX_TOUCH-1)
			{
			   enable_irq(ts->client->irq);
			    return ;
			}

			if(touchType == TOUCH_SCREEN)
			{
				g_Mtouch_info[touchID].posX= posX;
				g_Mtouch_info[touchID].posY= posY;
				g_Mtouch_info[touchID].width= width;

				if(touchState)
					g_Mtouch_info[touchID].strength= strength;
				else {
					g_Mtouch_info[touchID].strength = 0;
					tmp_flag[touchID] = 1;
				}

			}
			else if(touchType == TOUCH_KEY)
			{
				if (keyID == 0x1)
					input_report_key(ts->input_dev, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);
				if (keyID == 0x2)
					input_report_key(ts->input_dev, KEY_HOME, touchState ? PRESS_KEY : RELEASE_KEY);
				if (keyID == 0x3)
					input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);
				if (keyID == 0x4)
					input_report_key(ts->input_dev, KEY_SEARCH, touchState ? PRESS_KEY : RELEASE_KEY);
#if DEBUG_PRINT
				printk(KERN_ERR "melfas_ts_work_func: keyID : %d, touchState: %d\n", keyID, touchState);
#endif
				break;
			}

		}
	}

	if(touchType == TOUCH_SCREEN)
	{
		for(i=0; i<MELFAS_MAX_TOUCH; i++)
		{
			if(g_Mtouch_info[i].strength== -1)
				continue;


			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,  g_Mtouch_info[i].strength);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR, g_Mtouch_info[i].width );
			input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, i);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_mt_sync(ts->input_dev);
#if DEBUG_PRINT
			if ((touchState == 1 && tmp_flag[touchID] ==1) || (touchState ==0 && tmp_flag[touchID] ==1)) {
			printk(KERN_ERR "Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
				i, touchState, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
				if (touchState == 1 )
				 tmp_flag[touchID] = 0;
			}
#endif
			if(g_Mtouch_info[i].strength == 0)
				g_Mtouch_info[i].strength = -1;

		}
	}

	input_sync(ts->input_dev);
#endif

	enable_irq(ts->client->irq);
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_irq_handler\n");
#endif

	disable_irq_nosync(ts->client->irq);

	queue_delayed_work(melfas_wq, &ts->work, 0);

	return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0, i;

	uint8_t buf;

		melfas_wq = create_singlethread_workqueue("melfas_wq");

#if DEBUG_PRINT
			printk(KERN_DEBUG "[TOUCH] Melfas_ts_init \n");
#endif

		if (!melfas_wq) {
			printk(KERN_ERR "[TOUCH]failed to create singlethread workqueue\n");
			return -ENOMEM;
		}

#if 1	//DEBUG_PRINT
	printk(KERN_ERR "Touch : melfas_ts_probe Start!!!\n");
#endif

	memset(&tmp_flag[0],0x01,sizeof(tmp_flag));


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	touch_pdev = ts = kzalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->pdata = client->dev.platform_data;
	if (ts->pdata->power_enable)
		ts->power_enable = ts->pdata->power_enable;
    else
    {
        printk(KERN_ERR "melfas_ts_probe: ts->pdata->power_enable is NULL \n");
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

	ret = ts->pdata->power_enable(1, true);
        msleep(50);
	g_power_enable = ts->pdata->power_enable;

	INIT_DELAYED_WORK(&ts->work, melfas_ts_work_func);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	for(i=0; i<I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, &buf, 1);
		//printk("%d %d %d \n",i, buf,ret);
		if(ret >=0) {
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() ok [%d]\n", ret);
			break;
		}
		else
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed[%d]\n", ret);
	}

	buf = TS_READ_PUBLIC_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf, 1);
	ret = i2c_master_recv(ts->client, &buf, 1);

	printk("[Touch] Public Custom Version :: %d  \n",buf);

#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif



#if SET_DOWNLOAD_BY_GPIO
	buf = TS_READ_PUBLIC_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "melfas_probe : i2c_master_send [%d]\n", ret);
	}

	ret = i2c_master_recv(ts->client, &buf, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "melfas_probe : i2c_master_recv [%d]\n", ret);
	}

	printk(KERN_ERR "melfas_probe : buf %d \n",buf);
	if(buf < FW_VERSION)
	{

		printk(KERN_ERR "melfas_probe : download start \n");
		mms100_download();
	}


	//ret = ts->pdata->power_enable(0, true);
	//mdelay(50);
	//ret = ts->pdata->power_enable(1, true);

#endif // SET_DOWNLOAD_BY_GPIO

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "melfas-ts" ;

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
  ts->input_dev->propbit[0] = BIT_MASK(INPUT_PROP_DIRECT);
#endif
	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);
	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);


//	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
//	__set_bit(EV_ABS,  ts->input_dev->evbit);
//	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TS_MAX_Z_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MINOR, 0, TS_MAX_W_TOUCH, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH-1, 0, 0);

//	__set_bit(EV_SYN, ts->input_dev->evbit);
//	__set_bit(EV_KEY, ts->input_dev->evbit);


	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Failed to register device\n");
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}


	if (ts->client->irq) {
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif
		ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);

		//ret = request_threaded_irq(client->irq, melfas_ts_irq_handler,
		//		synaptics_ts_thread_irq_handler,
		//		ts->pdata->irqflags | IRQF_ONESHOT, client->name, ts);
		if (ret > 0) {
			printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	//schedule_work(&ts->work);

	for (i = 0; i < MELFAS_MAX_TOUCH ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;

#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_probe: succeed to register input device\n");
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif
	return 0;

err_request_irq:
	printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");
//err_detect_failed:
//	printk(KERN_ERR "melfas-ts: err_detect failed\n");
//	kfree(ts);
err_check_functionality_failed:
	printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");

	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;
	for(i=0; i<MELFAS_MAX_TOUCH; i++) {
		if(-1 == g_Mtouch_info[i].strength) {
			g_Mtouch_info[i].posX = 0;
			g_Mtouch_info[i].posY = 0;
			continue;
		}

		g_Mtouch_info[i].strength = 0;

		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength );
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
		input_mt_sync(ts->input_dev);

		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;

		if(0 == g_Mtouch_info[i].strength)
			g_Mtouch_info[i].strength = -1;
	}
}

#if defined(CONFIG_PM)
static int melfas_ts_suspend(struct device *device)
{
	#if DEBUG_PRINT
		printk(KERN_DEBUG "\n");
	#endif
	return 0;
}

static int melfas_ts_resume(struct device *device)
{
	#if DEBUG_PRINT
		printk(KERN_DEBUG "\n");
	#endif
	return 0;

}
#endif

static void melfas_ts_suspend_func(struct melfas_ts_data *ts)
{
	int ret;

	printk(KERN_ERR "melfas_ts_suspend start \n");

	disable_irq_nosync(ts->client->irq);

	ret = cancel_delayed_work_sync(&ts->work);

	ret = i2c_smbus_write_byte_data(ts->client, 0x01, 0x00); /* sleep */

	release_all_fingers(ts);

	ret = ts->pdata->power_enable(0, true);

	if (ret < 0)
		printk(KERN_ERR "melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");

	printk(KERN_ERR "melfas_ts_suspend end \n");

}

static void melfas_ts_resume_func(struct melfas_ts_data *ts)
{
	int ret = 0;
        printk(KERN_ERR "melfas_ts_resume start \n");

	ret = ts->pdata->power_enable(1, true);
	queue_delayed_work(melfas_wq,
			&ts->work,msecs_to_jiffies(ts->pdata->ic_booting_delay));

	enable_irq(ts->client->irq); // scl wave
	printk(KERN_ERR "melfas_ts_resume end \n");

}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend_func(ts);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume_func(ts);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_TS_NAME, 0 },
	{ }
};

#if defined(CONFIG_PM)
static struct dev_pm_ops melfas_ts_pm_ops = {
	.suspend 	= melfas_ts_suspend,
	.resume 	= melfas_ts_resume,
};
#endif

static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= MELFAS_TS_NAME,
		.owner 	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm 	= &melfas_ts_pm_ops,
#endif
	},
	.id_table		= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,

};

static int __devinit melfas_ts_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&melfas_ts_driver);
	if (ret < 0) {
		printk(KERN_ERR "[TOUCH]failed to i2c_add_driver\n");
		destroy_workqueue(melfas_wq);
	}

	return ret;
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);

	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
