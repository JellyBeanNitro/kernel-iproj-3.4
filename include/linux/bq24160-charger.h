/*
 * TI BQ24160 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_BQ24160_CHARGER_H
#define __LINUX_BQ24160_CHARGER_H

#ifdef CONFIG_BATTERY_325
// define register map
#define BQ24160_REG_STAT_CTRL 		0x00		/* Status / Control Register (read/write) */

#define BQ24160_WDOG_TMR_MASK		0x80
#define BQ24160_WDOG_TMR_SHFT		7
#define BQ24160_STAT_MASK			0x70
#define BQ24160_STAT_SHFT			4
#define BQ24160_SUPPLY_SEL_MASK		0x08
#define BQ24160_SUPPLY_SEL_SHFT		3
#define BQ24160_FAULT_MASK			0x07
#define BQ24160_FAULT_SHFT			0

#define BQ24160_REG_BATTNPS_STAT	0x01		/* Battery / Supply Status Register (read/write) */

#define BQ24160_INSTAT_MASK			0xC0
#define BQ24160_INSTAT_SHFT			6
#define BQ24160_USBSTAT_MASK		0x30
#define BQ24160_USBSTAT_SHFT		4
#define BQ24160_OTGLOCK_MASK		0x08
#define BQ24160_OTGLOCK_SHFT		3
#define BQ24160_BATTSTAT_MASK		0x06
#define BQ24160_BATTSTAT_SHFT		1

#define BQ24160_REG_CTRL			0x02		/* Control Register (read/write) */

#define BQ24160_RESET_MASK			0x80
#define BQ24160_RESET_SHFT			7
#define BQ24160_IUSB_LMT_MASK		0x70
#define BQ24160_IUSB_LMT_SHFT		4
#define BQ24160_ENSTAT_MASK			0x08
#define BQ24160_ENSTAT_SHFT			3
#define BQ24160_TE_MASK				0x04
#define BQ24160_TE_SHFT				2
#define BQ24160_CE_MASK				0x02
#define BQ24160_CE_SHFT				1
#define BQ24160_HZMODE_MASK			0x01
#define BQ24160_HZMODE_SHFT			0

#define BQ24160_REG_CTRL_BATVOLT	0x03		/* Control / Battery Voltage Register (read/write) */

#define BQ24160_VBATTREG_MASK		0xFC
#define BQ24160_VBATTREG_SHFT		2
#define BQ24160_INLMT_IN_MASK		0x02
#define BQ24160_INLMT_IN_SHFT		1
#define BQ24160_DPDM_EN_MASK		0x01
#define BQ24160_DPDM_EN_SHFT		0

#define BQ24160_REG_VEND_PART_REV	0x04		/* Vendor / Part / Revision (read only) */

#define BQ24160_VENDOR_MASK			0xE0
#define BQ24160_VENDOR_SHFT			5
#define BQ24160_PN_MASK				0x18
#define BQ24160_PN_SHFT				3
#define BQ24160_REV_MASK			0x07
#define BQ24160_REV_SHFT			0

#define BQ24160_REG_BATTTERM_FCHGCRNT	0x05	/* Battery Termination / Fast Charge Current (read/write) */

#define BQ24160_ICHGCRNT_MASK		0xF8
#define BQ24160_ICHGCRNT_SHFT		3
#define BQ24160_ITERMCRNT_MASK		0x07
#define BQ24160_ITERMCRNT_SHFT		0

#define BQ24160_REG_VINDPM_STAT	0x06			/* Vin-dpm Voltage / DPPM Status */

#define BQ24160_MINSYS_STAT_MASK	0x80
#define BQ24160_MINSYS_STAT_SHFT	7
#define BQ24160_DPM_STAT_MASK		0x40
#define BQ24160_DPM_STAT_SHFT		6
#define BQ24160_USB_INDPM_MASK		0x38
#define BQ24160_USB_INDPM_SHFT		3
#define BQ24160_IN_INDPM_MASK		0x07
#define BQ24160_IN_INDPM_SHFT		0

#define BQ24160_REG_SAFETMR_NTCMON		0x07	/* Safety Timer / NTC Monitor (read/write) */

#define BQ24160_2XTMR_EN_MASK		0x80
#define BQ24160_2XTMR_EN_SHFT		7
#define BQ24160_TMR_MASK			0x60
#define BQ24160_TMR_SHFT			5
#define BQ24160_TS_EN_MASK			0x08
#define BQ24160_TS_EN_SHFT			3
#define BQ24160_TS_FAULT_MASK		0x06
#define BQ24160_TS_FAULT_SHFT		1

#define BQ24160_STAT_NO_VALID_SRC_DETECTED		0
#define BQ24160_STAT_IN_READY 					1
#define BQ24160_STAT_USB_READY 					2
#define BQ24160_STAT_CHARGING_FROM_IN 			3
#define BQ24160_STAT_CHARGING_FROM_USB 			4
#define BQ24160_STAT_CHARGE_DONE 				5
#define BQ24160_STAT_NA 						6
#define BQ24160_STAT_FAULT						7

#define BQ24160_CHG_WORK_PERIOD	((HZ) * 10)
#define BQ24160_CHG_DONE_WORK_PERIOD	((HZ) * 3)

#ifdef CONFIG_BATMAN_VZW_KERNEL_GPIO
#define SWITCHING_CHG_IRQ_N 156
#else
#define SWITCHING_CHG_IRQ_N	155
#endif

#endif


struct bq24160_platform_data {
	u8  tmr_rst;		/* watchdog timer reset */
	u8  supply_sel;		/* supply selection */

	u8	reset;			/* reset all reg to default values */
	u8	iusblimit;		/* usb current limit */
	u8 	enstat;			/* enable STAT */
	u8	te;				/* enable charger termination */
	u8 	ce;				/* charger enable : 0 disable : 1 */
	u8	hz_mode;		/* high impedance mode */

	u8 	vbatt_reg;		/* battery regulation voltage */
	u8	inlimit_in;		/* input limit for IN input */
	u8	dpdm_en;		/* D+/D- detention */

	u8	chgcrnt;		/* charge current */
	u8	termcrnt;		/* termination current sense */

	u8	minsys_stat;	/* minimum system voltage mode */
	u8	dpm_stat;		/* Vin-DPM mode */
	u8	vindpm_usb;		/* usb input Vin-dpm voltage */
	u8	vindpm_in;		/* IN input Vin-dpm voltage */

	u8	tmr2x_en;		/* timer slowed by 2x */
	u8	safety_tmr;		/* safety timer */
	u8	ts_en;			/* ts function enable */
	u8	ts_fault;		/* ts fault mode */
	
#ifdef CONFIG_LGE_WIRELESS_CHARGER_BQ24160
	int 	valid_n_gpio;
	int 	(*chg_detection_config) (void);
#endif
};

#endif
