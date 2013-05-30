/* lge/audio/wm9093.c
 *
 * Copyright (C) 2010 LGE, Inc.
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

/* #define DEBUG */ /* for printing pr_debug, dev_dbg */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "wm9093.h"

#define MODULE_NAME	"wm9093"

#ifdef CONFIG_LGE_AUDIO_AMP_WM9093_NO_DEBUG_MESSAGE

#undef pr_debug
#undef pr_info
#undef pr_err

#define pr_debug(fmt, args...) do {} while(0)
#define pr_info(fmt, args...) do {} while(0)
#define pr_err(fmt, args...) do {} while(0)

#endif

/* This struct is used to save the context */
struct amp_data {
	struct i2c_client *client;
	struct wm9093_platform_data *pdata;
	struct mutex mutex;
};

static struct amp_data *_data = NULL; /* TODO erase req. need to way to give handle through set_amp_path interface */

static int wm9093_amp_read_register(struct amp_data *data, u8 reg, int* ret)
{
/* just for test code */
#if 0
	return swab16(i2c_smbus_read_word_data(client, reg));
#else
	struct i2c_msg	xfer[2];
	u16				value = 0xffff;
	u16				retval;

	xfer[0].addr = data->client->addr;
	xfer[0].flags = 0;
	xfer[0].len  = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = data->client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 2;
	xfer[1].buf = (u8*)&value;

	retval = i2c_transfer(data->client->adapter, xfer, 2);

	*ret =  (value>>8) | ((value & 0xff) << 8);

	return retval;
#endif
}

static int wm9093_amp_write_register(struct amp_data *data, u8 reg, int value)
{
/* just for test code */
#if 0
	int ret;
	if ((ret = i2c_smbus_write_word_data(client, reg, swab16(value))) < 0)
		pr_info("========== woonrae: i2c error================== ret=%d", ret);
	return ret;
#else
	int				 err;
	unsigned char    buf[3];
	struct i2c_msg	msg = { data->client->addr, 0, 3, &buf[0] };
	
	buf[0] = reg;
	buf[1] = (value & 0xFF00) >> 8;
	buf[2] = value & 0x00FF;

	if ((err = i2c_transfer(data->client->adapter, &msg, 1)) < 0){
		return -EIO;
	} else {
		return 0;
	}
#endif
}

static void wm9093_codec_cmd(struct amp_data *data, wmCodecCmd *amp_codec_cmd)
{
    int err = 0;
#ifdef CONFIG_LGE_AUDIO_AMP_WM9093_CHECK_REGISTER_VALUE_BY_REREADING
	int errRead = 0;
	int rData=0;
#endif

	err = wm9093_amp_write_register(data, amp_codec_cmd->wmaddress, amp_codec_cmd->wmdata);

	if (err == 0) {
		//pr_debug("WM9093 I2C Write OK {0x%02X, 0x%04X}\n",
		//	amp_codec_cmd->wmaddress, amp_codec_cmd->wmdata);
	} else {
		pr_err("WM9093 I2C Write FAIL : reg = 0x%02X, data = 0x%04X\n",
			amp_codec_cmd->wmaddress, amp_codec_cmd->wmdata);
	}

#ifdef CONFIG_LGE_AUDIO_AMP_WM9093_CHECK_REGISTER_VALUE_BY_REREADING
	errRead = wm9093_amp_read_register(data, amp_codec_cmd->wmaddress, &rData);

	if (errRead > 0) {
		//pr_debug(KERN_INFO "WM9093 I2C Read OK : reg = 0x%X, data = 0x%X\n",
		//	amp_codec_cmd->wmaddress, rData&0xFFFF);
	} else {
		pr_err("WM9093 I2C Read FAIL :errRead = %d[%X] reg = 0x%X, data = 0x%X\n",
			errRead, errRead, amp_codec_cmd->wmaddress, rData&0xFFFF);
	}
#endif
}

static void wm9093_cmd_register_sequence(struct amp_data *data, struct wm9093_CodecCmd_data *seq) {
	int idx = 0;

	for(idx = 0; idx < seq->amp_function_size; idx++)
	{
		if (data->pdata->bTuningOnOff)
			wm9093_codec_cmd(data, seq->amp_tuning_function + idx);
		else
			wm9093_codec_cmd(data, seq->amp_function + idx);
	}
}

#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
/* add delay to reduce earphone popup noise for DCM*/
static void wm9093_cmd_register_sequence_add_delay(int icodec_num, struct amp_data *data, struct wm9093_CodecCmd_data *seq) {
	int idx = 0;
	int sleep_point = 0;
	switch(icodec_num) 
	{
		case  ICODEC_HEADSET_ST_RX:
		  sleep_point = 1;
	    break;
		case  ICODEC_SPEAKER_RX:
		  sleep_point = 1;
    	break;
		case  ICODEC_HEADSET_ST_RX_SPEAKER_RX:
		  sleep_point = 1;
     	break;
		case  ICODEC_TTY_RX:
		  sleep_point = 1;
     	break;		  
		case  ICODEC_SPEAKER_PLAYBACK_RX:
		  sleep_point = 1;
     	break;
		case  ICODEC_HEADSET_ST_PLAYBACK_RX:
		  sleep_point = 1;
      	break;
    
    	default :
    	sleep_point = 0;
        break;
	}
	for(idx = 0; idx < seq->amp_function_size; idx++)
	{
		
		if (data->pdata->bTuningOnOff)
			wm9093_codec_cmd(data, seq->amp_tuning_function + idx);
		else
			wm9093_codec_cmd(data, seq->amp_function + idx);
			
		if ((sleep_point != 0)&&(idx == sleep_point))
		  {
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM)
		    msleep(50);
#elif defined(CONFIG_MACH_LGE_325_BOARD_VZW)
			msleep(70);
#endif
		    pr_info("************** wm9093 sleep in 50ms for reducing earphone popup noise *************\n");
		    
		  }
	}		
}

#endif


static void wm9093_cmd_register(wmCodecCmd wmCmd) {
	struct amp_data *data = _data; /* TODO move this variable to function parameter. interface change is required */

	wm9093_codec_cmd(data, &wmCmd);
}

static void wm9093_reg_dump(int icodec_num)
{

	struct amp_data *data = _data; /* TODO move this variable to function parameter. interface change is required */
 //   int rtnWD = 0;
	int i=0;
	int read_result;
	int reg_val;
	pr_info("************** wm regs *************\n");
	
	if (NULL == data) {
		pr_err("wm9093 is not initialized yet\n");
		WARN(1, "wm9093 is not initialized yet\n");
		return;
	}

	switch(icodec_num) {
		case  ICODEC_HANDSET_RX:
        case  ICODEC_AMP_OFF:
			break;
				
		case  ICODEC_HEADSET_ST_RX:
		pr_info("************** wm HDSET regs *************\n");
		for (i = 0; i < data->pdata->hph_on.amp_function_size ; i++) {
			read_result = wm9093_amp_read_register(data, data->pdata->hph_on.amp_function[i].wmaddress, &reg_val);
			if (read_result < 0) {
				pr_info("failed to read codec register\n");
				break;
			} else
			pr_info("WM9093[0x%02X] Oval: 0x%04X Tval: 0x%04X Readval: 0x%04X\n", data->pdata->hph_on.amp_function[i].wmaddress,data->pdata->hph_on.amp_function[i].wmdata ,data->pdata->hph_on.amp_tuning_function[i].wmdata,reg_val);
			
		}
		break;

		case  ICODEC_SPEAKER_RX:
		pr_info("************** wm SPK regs *************\n");
			for (i = 0; i < data->pdata->speaker_on.amp_function_size ; i++) {
				read_result = wm9093_amp_read_register(data, data->pdata->speaker_on.amp_function[i].wmaddress, &reg_val);
				if (read_result < 0) {
					pr_info("failed to read codec register\n");
					break;
				} else
				pr_info("WM9093[0x%02X] Oval: 0x%04X Tval: 0x%04X Readval: 0x%04X\n", data->pdata->speaker_on.amp_function[i].wmaddress,data->pdata->speaker_on.amp_function[i].wmdata ,data->pdata->speaker_on.amp_tuning_function[i].wmdata,reg_val);
				
			}
		break;	

		case ICODEC_HEADSET_ST_RX_SPEAKER_RX:   // simultaneously Ringing Headset and SPK
			pr_info("************** wm HDSET and SPK regs *************\n");
			for (i = 0; i < data->pdata->hph_spk_on.amp_function_size ; i++) {
				read_result = wm9093_amp_read_register(data, data->pdata->hph_spk_on.amp_function[i].wmaddress, &reg_val);
				if (read_result < 0) {
					pr_info("failed to read codec register\n");
					break;
				} else
				pr_info("WM9093[0x%02X] Oval: 0x%04X Tval: 0x%04X Readval: 0x%04X\n", data->pdata->hph_spk_on.amp_function[i].wmaddress,data->pdata->hph_spk_on.amp_function[i].wmdata, data->pdata->hph_spk_on.amp_tuning_function[i].wmdata,reg_val);
				
			}
        break;
		case  ICODEC_HEADSET_ST_PLAYBACK_RX:
		pr_info("************** wm MUSIC HDSET regs *************\n");
		for (i = 0; i < data->pdata->hph_playback_on.amp_function_size ; i++) {
			read_result = wm9093_amp_read_register(data, data->pdata->hph_playback_on.amp_function[i].wmaddress, &reg_val);
			if (read_result < 0) {
				pr_info("failed to read codec register\n");
				break;
			} else
			pr_info("WM9093[0x%02X] Oval: 0x%04X Tval: 0x%04X Readval: 0x%04X\n", data->pdata->hph_playback_on.amp_function[i].wmaddress, data->pdata->hph_playback_on.amp_function[i].wmdata , data->pdata->hph_playback_on.amp_tuning_function[i].wmdata,reg_val);
			
		}
		break;

		case  ICODEC_SPEAKER_PLAYBACK_RX:
		pr_info("************** wm MUSIC SPK regs *************\n");
			for (i = 0; i < data->pdata->speaker_playback_on.amp_function_size ; i++) {
				read_result = wm9093_amp_read_register(data, data->pdata->speaker_playback_on.amp_function[i].wmaddress, &reg_val);
				if (read_result < 0) {
					pr_info("failed to read codec register\n");
					break;
				} else
				pr_info("WM9093[0x%02X] Oval: 0x%04X Tval: 0x%04X Readval: 0x%04X\n", data->pdata->speaker_playback_on.amp_function[i].wmaddress,data->pdata->speaker_playback_on.amp_function[i].wmdata, data->pdata->speaker_playback_on.amp_tuning_function[i].wmdata,reg_val);
				
			}
		break;	
		
		default:
			for (i = 0; i < 0x50 ; i++) {
			read_result = wm9093_amp_read_register(data, i, &reg_val);
			if (read_result < 0) {
				pr_info("failed to read codec register\n");
				break;
			} else
				pr_info("WM9093 reg 0x%02X val 0x%04X\n", i, reg_val);
		}	
		break;	
			
	}
	pr_info("*****************************************\n");
}

static void wm9093_set_amp_path(int icodec_num)
{
	struct amp_data *data = _data; /* TODO move this variable to function parameter. interface change is required */

	if (NULL == data) {
		pr_err("wm9093 is not initialized yet\n");
		WARN(1, "wm9093 is not initialized yet\n");
		return;
	}

	mutex_lock(&data->mutex);
#ifdef CONFIG_LGE_AUDIO_WM9093_POPNOISE
	if( (icodec_num == ICODEC_HEADSET_ST_RX) || (icodec_num == ICODEC_HEADSET_ST_RX_SPEAKER_RX) || (icodec_num == ICODEC_HEADSET_ST_PLAYBACK_RX))
	{
		if(icodec_num == ICODEC_HEADSET_ST_RX_SPEAKER_RX)
		{
			pr_info(KERN_INFO"headset_speaker_pre setting for elimiating AMP pop noise");
			wm9093_cmd_register_sequence(data, &(data->pdata->headset_speaker_pre));
		}
		else
		{
			pr_info(KERN_INFO"headset_pre setting for elimiating AMP pop noise");
			wm9093_cmd_register_sequence(data, &(data->pdata->headset_pre));
		}
		msleep(50);
	}
#endif

	switch(icodec_num) {
		case ICODEC_HANDSET_RX:
			pr_info("AMP ON: voc_codec %d does not use the amp\n", icodec_num);
			break;

        case  ICODEC_AMP_OFF:
			pr_info("AMP OFF: voc_codec %d\n", icodec_num);
			wm9093_cmd_register_sequence(data, &(data->pdata->power_down));
            msleep(50);    //                                            
			break;
				
		case  ICODEC_HEADSET_ST_RX:
			pr_info("AMP ON: voc_codec %d for HEADSET_ST_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
			wm9093_cmd_register_sequence_add_delay(ICODEC_HEADSET_ST_RX, data, &(data->pdata->hph_on));
#else 
			wm9093_cmd_register_sequence(data, &(data->pdata->hph_on));
#endif
		break;

		case  ICODEC_SPEAKER_RX:
			pr_info("AMP ON: voc_codec %d for SPEAKER_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
			wm9093_cmd_register_sequence_add_delay(ICODEC_SPEAKER_RX, data, &(data->pdata->speaker_on));
#else
			wm9093_cmd_register_sequence(data, &(data->pdata->speaker_on));
#endif
		break;

        case ICODEC_HEADSET_ST_RX_SPEAKER_RX:   // simultaneously Ringing Headset and SPK
            pr_info("AMP ON: voc_codec %d for HEADSET_ST_RX_SPEAKER_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
      		wm9093_cmd_register_sequence_add_delay(ICODEC_HEADSET_ST_RX_SPEAKER_RX, data, &(data->pdata->hph_spk_on));
#else
      		wm9093_cmd_register_sequence(data, &(data->pdata->hph_spk_on));
#endif
	 	break;

        case ICODEC_TTY_RX: // TTY
            pr_info("AMP ON: voc_codec %d for TTY_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
     		wm9093_cmd_register_sequence_add_delay(ICODEC_TTY_RX, data, &(data->pdata->tty_on));
#else
			wm9093_cmd_register_sequence( data, &(data->pdata->tty_on));
#endif
	 	break;

        case ICODEC_SPEAKER_PLAYBACK_RX:   // Playback not call
            pr_info("AMP ON: voc_codec %d for SPEAKER_PLAYBACK_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
      		wm9093_cmd_register_sequence_add_delay(ICODEC_SPEAKER_PLAYBACK_RX, data, &(data->pdata->speaker_playback_on));
#else
			wm9093_cmd_register_sequence( data, &(data->pdata->speaker_playback_on));
#endif
	 	break;

        case ICODEC_SPEAKER_PLAYBACK_VZWNAVI:   // Playback not call
            pr_info("AMP ON: voc_codec %d for ICODEC_SPEAKER_VZW_NAVI amp\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
      		wm9093_cmd_register_sequence_add_delay(ICODEC_SPEAKER_PLAYBACK_VZWNAVI, data, &(data->pdata->speaker_vzwnavi_on));
#else
			wm9093_cmd_register_sequence(data, &(data->pdata->speaker_vzwnavi_on));
#endif
	 	break;

        case ICODEC_HEADSET_ST_PLAYBACK_RX:   // Playback not call
            pr_info("AMP ON: voc_codec %d for HEADSET_ST_PLAYBACK_RX\n", icodec_num);
#if defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
      		wm9093_cmd_register_sequence_add_delay(ICODEC_HEADSET_ST_PLAYBACK_RX, data, &(data->pdata->hph_playback_on));
#else
			wm9093_cmd_register_sequence( data, &(data->pdata->hph_playback_on));
#endif
		break;

		default :
			pr_err("voc_icodec %d does not support AMP\n", icodec_num);
			break;
    }

	mutex_unlock(&data->mutex);
}

static int wm9093_amp_ctl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct amp_data *data;
	struct i2c_adapter* adapter = client->adapter;
	int err;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)){
		err = -EOPNOTSUPP;
		return err;
	}

	pr_info("%s()\n", __FUNCTION__);

	data = kzalloc(sizeof(struct amp_data), GFP_KERNEL);
	if (NULL == data) {
		dev_err(&client->dev, "Can not allocate memory\n");
		return -ENOMEM;
	}

	mutex_init(&data->mutex);

	if (client->dev.platform_data) {
		data->pdata = client->dev.platform_data;		
		data->pdata->set_amp_path = wm9093_set_amp_path;
		data->pdata->wm9093_cmd_register = wm9093_cmd_register;
		data->pdata->wm9093_reg_dump = wm9093_reg_dump;
		_data = data;
		data->client = client;
		i2c_set_clientdata(client, data);
	} else {
		dev_err(&client->dev, "No platform data to initialize\n");
		return -EINVAL;
	}

	pr_info("%s chip found\n", client->name);

	err = wm9093_amp_write_register(data, 0x00, 0x9093);

	if (err == 0) {
		pr_info("AMP INIT OK\n");
	} else {
		pr_err("AMP INIT ERR\n");
	}

	msleep(100); /* TODO */

	return 0;
}

static int wm9093_amp_ctl_remove(struct i2c_client *client)
{
	struct amp_data *data = i2c_get_clientdata(client);

	int err;

/*	data->pdata->set_amp_path(ICODEC_AMP_OFF); */

	err = wm9093_amp_write_register(data, 0x00, 0x9093);

	if (err) pr_err("AMP RESET ERR\n");

	data->pdata->set_amp_path = NULL;

	msleep(100);

	_data = NULL;
	kfree (data);
	
	pr_info("%s()\n", __FUNCTION__);
	i2c_set_clientdata(client, NULL);

	return 0;
}


static struct i2c_device_id wm9093_amp_idtable[] = {
	{ "wm9093", 1 },
};

static struct i2c_driver wm9093_amp_ctl_driver = {
	.probe = wm9093_amp_ctl_probe,
	.remove = wm9093_amp_ctl_remove,
	.id_table = wm9093_amp_idtable,
	.driver = {
		.name = MODULE_NAME,
	},
};

static int __init wm9093_amp_ctl_init(void)
{
	return i2c_add_driver(&wm9093_amp_ctl_driver);
}

static void __exit wm9093_amp_ctl_exit(void)
{
	return i2c_del_driver(&wm9093_amp_ctl_driver);
}

module_init(wm9093_amp_ctl_init);
module_exit(wm9093_amp_ctl_exit);

MODULE_DESCRIPTION("WM9093 Audio Subsystem Control");
MODULE_AUTHOR("DongSung SHIN <dongsung.shin@lge.com");
MODULE_LICENSE("GPL");
