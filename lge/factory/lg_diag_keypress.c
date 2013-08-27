#include <linux/module.h>
#include <linux/delay.h>
#include <lg_diagcmd.h>
#include <lg_diag_keypress.h>
#include <linux/input.h>
#include <mach/gpio.h>
/*==========================================================================*/
#define HS_RELEASE_K 0xFFFF
#define KEY_TRANS_MAP_SIZE 77

//#define DEBUG_DIAG_KEYPRESS

/* Virtual Key */
#define V_KEY_CALL_LOG				0x61
#define V_KEY_DELETE_ALL_CALL_LOG	0x62
#define V_KEY_OK_DELETE				0x63
#define V_KEY_DIAL          0x64
#define V_KEY_EDITBOX       0x65
#define V_KEY_STAR	227
#define V_KEY_POUND	228

#define DIAL_TOUCH_X  115	
#define DIAL_TOUCH_Y  170
#define EDITBOX_TOUCH_X  534
#define EDITBOX_TOUCH_Y 524
#define CALL_LOG_TOUCH_X  397
#define CALL_LOG_TOUCH_Y  222
#define DELET_ALL_TOUCH_X 557
//                                                                         
#if defined (CONFIG_MACH_LGE_325_BOARD_VZW)
#define DELET_ALL_TOUCH_Y 1722
#else
#define DELET_ALL_TOUCH_Y 1752
#endif
//                                                                         
#define DELET_OK_TOUCH_X 817
#define DELET_OK_TOUCH_Y 1244

extern struct input_dev* get_ats_input_dev(void);
typedef struct {
	  word LG_common_key_code;
	    unsigned int Android_key_code;
}keycode_trans_type;

keycode_trans_type keytrans_table[KEY_TRANS_MAP_SIZE]={
/* index = 0 */	{0x30, KEY_0},	
/* index = 1 */	{0x31, KEY_1},	
/* index = 2 */	{0x32, KEY_2},	
/* index = 3 */	{0x33, KEY_3},	
/* index = 4 */	{0x34, KEY_4},	
/* index = 5 */	{0x35, KEY_5},	
/* index = 6 */	{0x36, KEY_6},	
/* index = 7 */	{0x37, KEY_7},	
/* index = 8 */	{0x38, KEY_8},	
/* index = 9 */	{0x39, KEY_9},	
/* index = 10 */{0x2A, V_KEY_STAR},	
/* index = 11 */{0x23, V_KEY_POUND},
/* index = 12 */{0x50, KEY_SEND},
/* index = 13 */{0x51, KEY_END},
/* index = 14 */{0x52, KEY_HOME},
/* index = 15 */{0x10, KEY_LEFT},	
/* index = 16 */{0x11, KEY_RIGHT},
/* index = 17 */{0x54, KEY_UP},
/* index = 18 */{0x55, KEY_DOWN},
/* index = 19 */{0x53, KEY_OK},
/* index = 20 */{0x96, KEY_VOLUMEUP},
/* index = 21 */{0x97, KEY_VOLUMEDOWN},
#if defined(CONFIG_MACH_LGE_325_BOARD_SKT) //                                           
/* index = 22 */{0xB6, KEY_POWER},
#else /* BATMAN LGU, CAYMAN SKT, CAYMAN KT */
/* index = 22 */{0x51, KEY_POWER},
#endif
/* index = 23 */{0xA0, KEY_MENU},	
/* index = 24 */{0xA1, KEY_HOME},		
/* index = 25 */{0xA2, KEY_BACK},			
/* index = 26 */{0xB0, KEY_SEND /*V_KEY_DIAL*/}, 
/* index = 27 */{0xB1, V_KEY_CALL_LOG}, 
/* index = 28 */{0xB2, V_KEY_DELETE_ALL_CALL_LOG}, 
/* index = 29 */{0xB3, V_KEY_OK_DELETE}, 
/* index = 30 */{0x09, V_KEY_DIAL},
/* index = 31 */{0x0A, V_KEY_EDITBOX},
/* index = 32 */{0xB5, KEY_VIDEO_CALL},
#if defined(CONFIG_MACH_LGE_325_BOARD_LGU) || defined(CONFIG_MACH_LGE_I_BOARD_LGU) //                                                          
/* index = 33 */{0xB6, KEY_VIDEO_CALL_RECEIVE},
/* index = 34 */{0xB7, KEY_VIDEO_CALL_END},
/* index = 35 */{0xB8, KEY_PS_CALL},
/* index = 36 */{0xB9, KEY_PS_CALL_RECEIVE},
/* index = 37 */{0xBA, KEY_PS_CALL_END},
#endif
//                                                                         
#if defined (CONFIG_MACH_LGE_325_BOARD_VZW)
/* index = 33 */{0xB6, KEY_HIGHLIGHER},
/* index = 34 */{0xB7, KEY_SEARCH},
/* index = 35 */{0xC0, KEY_A},
/* index = 36 */{0xC1, KEY_B},
/* index = 37 */{0xC2, KEY_C},
/* index = 38 */{0xC3, KEY_D},
/* index = 39 */{0xC4, KEY_E},
/* index = 40 */{0xC5, KEY_F},
/* index = 41 */{0xC6, KEY_G},
/* index = 42 */{0xC7, KEY_H},
/* index = 43 */{0xC8, KEY_I},
/* index = 44 */{0xC9, KEY_J},
/* index = 45 */{0xCA, KEY_K},
/* index = 46 */{0xCB, KEY_L},
/* index = 47 */{0xCC, KEY_M},
/* index = 48 */{0xCD, KEY_N},
/* index = 49 */{0xCE, KEY_O},
/* index = 50 */{0xCF, KEY_P},
/* index = 51 */{0xD0, KEY_Q},
/* index = 52 */{0xD1, KEY_R},
/* index = 53 */{0xD2, KEY_S},
/* index = 54 */{0xD3, KEY_T},
/* index = 55 */{0xD4, KEY_U},
/* index = 56 */{0xD5, KEY_V},
/* index = 57 */{0xD6, KEY_W},
/* index = 58 */{0xD7, KEY_X},
/* index = 59 */{0xD8, KEY_Y},
/* index = 60 */{0xD9, KEY_Z},
/* index = 61 */{0xDA, KEY_SPACE},
/* index = 62 */{0xDB, KEY_DOT},
#endif
//                                                                         
};

unsigned int LGF_KeycodeTrans(word input)
{
	int index = 0;
	unsigned int ret = (unsigned int)input;  // if we can not find, return the org value. 
 
	for( index = 0; index < KEY_TRANS_MAP_SIZE ; index++)
	{
		if( keytrans_table[index].LG_common_key_code == input)
		{
			ret = keytrans_table[index].Android_key_code;
			break;
		}
	}  
#ifdef DEBUG_DIAG_KEYPRESS
	printk(KERN_INFO "##DIAG_KEYPRESS## %s, input(%d), key_code(%d)\n", __func__, input, keytrans_table[index].Android_key_code);		
#endif
	return ret;
}

EXPORT_SYMBOL(LGF_KeycodeTrans);
/* ==========================================================================
===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
//extern unsigned int LGF_KeycodeTrans(word input);
extern void Send_Touch( unsigned int x, unsigned int y);
/*==========================================================================*/

static unsigned saveKeycode =0 ;

void SendKey(unsigned int keycode, unsigned char bHold)
{
  struct input_dev *idev = get_ats_input_dev();

  if( keycode != HS_RELEASE_K)
  {
    input_report_key( idev,keycode , 1 ); // press event
    input_sync(idev);
  }

  if(bHold)
  {
    saveKeycode = keycode; 
  }
  else
  {
    if( keycode != HS_RELEASE_K)
      input_report_key( idev,keycode , 0 ); // release  event
    else
      input_report_key( idev,saveKeycode , 0 ); // release  event
    input_sync(idev);
  }
}

void LGF_SendKey(word keycode)
{
	struct input_dev* idev = NULL;

	idev = get_ats_input_dev();

	if(idev == NULL)
		printk("%s: input device addr is NULL\n",__func__);
	
	input_report_key(idev,(unsigned int)keycode, 1);
	input_report_key(idev,(unsigned int)keycode, 0);
	input_sync(idev);

}

EXPORT_SYMBOL(LGF_SendKey);


PACK (void *)LGF_KeyPress (
        PACK (void	*)req_pkt_ptr,			/* pointer to request packet  */
        uint16		pkt_len )		      	/* length of request packet   */
{
  DIAG_HS_KEY_F_req_type *req_ptr = (DIAG_HS_KEY_F_req_type *) req_pkt_ptr;
  DIAG_HS_KEY_F_rsp_type *rsp_ptr;
  unsigned int keycode = 0;
  const int rsp_len = sizeof( DIAG_HS_KEY_F_rsp_type );

  rsp_ptr = (DIAG_HS_KEY_F_rsp_type *) diagpkt_alloc( DIAG_HS_KEY_F, rsp_len );
  if (!rsp_ptr)
  	return 0;

  if((req_ptr->magic1 == 0xEA2B7BC0) && (req_ptr->magic2 == 0xA5B7E0DF))
  {
    rsp_ptr->magic1 = req_ptr->magic1;
    rsp_ptr->magic2 = req_ptr->magic2;
    rsp_ptr->key = 0xff; //ignore byte key code
    rsp_ptr->ext_key = req_ptr->ext_key;

    keycode = LGF_KeycodeTrans((word) req_ptr->ext_key);
  }
  else
  {
    rsp_ptr->key = req_ptr->key;
    keycode = LGF_KeycodeTrans((word) req_ptr->key);

  }

  if( keycode == 0xff)
    keycode = HS_RELEASE_K;  // to mach the size
  
#ifdef DEBUG_DIAG_KEYPRESS		
	printk(KERN_INFO "##DIAG_KEYPRESS## %s, line(%d), keycode(%d), hold(%d)\n", __func__, __LINE__, keycode, req_ptr->hold);			  
#endif
  switch (keycode){
	case V_KEY_CALL_LOG:
	    Send_Touch(CALL_LOG_TOUCH_X, CALL_LOG_TOUCH_Y);
	    break;
				
	case V_KEY_DELETE_ALL_CALL_LOG:
	    Send_Touch(DELET_ALL_TOUCH_X, DELET_ALL_TOUCH_Y);
	    break;
				
	case V_KEY_OK_DELETE:
	    Send_Touch(DELET_OK_TOUCH_X, DELET_OK_TOUCH_Y);
	    break;

  case V_KEY_DIAL:
      Send_Touch(DIAL_TOUCH_X, DIAL_TOUCH_Y);
      break;;
  case V_KEY_EDITBOX:
      Send_Touch(EDITBOX_TOUCH_X, EDITBOX_TOUCH_Y);
      break;

	default:
    	SendKey(keycode , req_ptr->hold);
		break;
  	}
  	
  return (rsp_ptr);
}

EXPORT_SYMBOL(LGF_KeyPress);
