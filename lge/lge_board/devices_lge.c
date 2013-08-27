/*
 * Ref : LG I-Project
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <board_lge.h>
#include <lg_fw_diag_communication.h>

/* for board revision */
int lge_bd_rev;
int lge_bd_target;

static int __init board_revno_setup(char *rev_info)
{
	/* CAUTION: */
#ifdef CONFIG_MACH_LGE_I_BOARD_ATNT
         char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
		"rev_e", "rev_f", "rev_g", "rev_10", "rev_10", "rev_11", "rev_12",
		"rev_13", "rev_14", "rev_15", "rev_16", "rev_17", "rev_18", "rev_19",
			"revserved"};
#else /*                              */
	char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
		"rev_e", "rev_f", "rev_g",  "rev_10", "rev_11", "rev_12",
		"rev_13", "rev_14", "rev_15", "rev_16", "rev_17", "rev_18", "rev_19",
		"revserved"};
#endif /*                              */

	int i;
	
	lge_bd_rev =  LGE_REV_TOT_NUM;

	for(i=0; i< LGE_REV_TOT_NUM; i++)
		if( !strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}

	printk(KERN_INFO "BOARD : LGE %s \n", rev_str[lge_bd_rev]);

#ifdef CONFIG_MACH_LGE_I_BOARD
#ifdef CONFIG_MACH_LGE_I_BOARD_ATNT
  lge_bd_target = LGE_I_BOARD_ATNT;
#endif
#ifdef CONFIG_MACH_LGE_I_BOARD_DCM
  lge_bd_target = LGE_I_BOARD_DCM;
#endif
#if defined(CONFIG_MACH_LGE_I_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
  lge_bd_target = LGE_I_BOARD_SKT;
#endif
#ifdef CONFIG_MACH_LGE_I_BOARD_VZW
  lge_bd_target = LGE_I_BOARD_VZW;
#endif
#if defined(CONFIG_MACH_LGE_I_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
  lge_bd_target = LGE_I_BOARD_LGU;
#endif
	printk(KERN_INFO "BOARD : LGE target %d \n", lge_bd_target);

#endif 

	return 1;
}
__setup("lge.rev=", board_revno_setup);

#ifdef CONFIG_ATCMD_VIRTUAL_KBD
/* virtual key */
#define ATCMD_VIRTUAL_KEYPAD_ROW	8
#define ATCMD_VIRTUAL_KEYPAD_COL	8

#define KEY_STAR 227
#define KEY_SHARP 228

static unsigned short atcmd_virtual_keycode[ATCMD_VIRTUAL_KEYPAD_ROW][ATCMD_VIRTUAL_KEYPAD_COL] = {
	{KEY_1,          KEY_8,           KEY_UNKNOWN/*KEY_Q*/,          KEY_I,          KEY_D,          KEY_HOME,       KEY_B,          KEY_UP},
	{KEY_2,          KEY_9,           KEY_W,          KEY_O,          KEY_F,          KEY_RIGHTSHIFT, KEY_N,          KEY_DOWN},
	{KEY_3,          KEY_0,           KEY_E,          KEY_P,          KEY_G,          KEY_Z,          KEY_M,          KEY_UNKNOWN},
	{KEY_4,          KEY_BACK,        KEY_R,          KEY_SEARCH,     KEY_H,          KEY_X,          KEY_LEFTSHIFT,  KEY_UNKNOWN},
	{KEY_5,          KEY_BACKSPACE,   KEY_T,          KEY_LEFTALT,    KEY_J,          KEY_C,          KEY_REPLY,      KEY_CAMERA},
	{KEY_6,          KEY_ENTER,       KEY_Y,          KEY_A,          KEY_K,          KEY_V,          KEY_RIGHT,      KEY_UNKNOWN},
	{KEY_7,          KEY_MENU,        KEY_U,          KEY_S,          KEY_L,          KEY_SPACE,      KEY_LEFT,       KEY_SEND},
	{KEY_STAR,       KEY_SHARP,       KEY_END,        KEY_UNKNOWN,    KEY_UNKNOWN,    KEY_UNKNOWN,    KEY_UNKNOWN,    KEY_UNKNOWN},
};

struct atcmd_virtual_platform_data atcmd_virtual_pdata = {
	.keypad_row = ATCMD_VIRTUAL_KEYPAD_ROW,
	.keypad_col = ATCMD_VIRTUAL_KEYPAD_COL,
	.keycode = (unsigned char *)atcmd_virtual_keycode,
};

struct platform_device atcmd_virtual_kbd_device = {
	.name = "atcmd_virtual_kbd",
	.id = -1,
	.dev = {
		.platform_data = &atcmd_virtual_pdata,
	},
};
#endif

#ifdef CONFIG_ETA_EVENT_LOG
struct platform_device eta_event_logger_device = {
	.name = "eta_event_logger",
};
#endif

#ifdef CONFIG_LGE_DIAGTEST
struct diagcmd_platform_data lg_fw_diagcmd_pdata = {
	.name = "lg_fw_diagcmd",
};

struct platform_device lg_fw_diagcmd_device = {
	.name = "lg_fw_diagcmd",
	.id = -1,
	.dev = {
		.platform_data = &lg_fw_diagcmd_pdata
	},
};

struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev = {
		.platform_data = 0,
	},
};

struct platform_device lg_diag_input_device = {
	.name = "ats_input",
	.id = -1,
	.dev = {
		.platform_data = 0,
	},
};
#endif
