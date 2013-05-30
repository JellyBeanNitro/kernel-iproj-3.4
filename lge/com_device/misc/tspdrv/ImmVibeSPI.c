/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/types.h>
#include <linux/list.h>
#include <linux/err.h>
#include "../../../include/board_lge.h"
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1

#define PWM_DUTY_MAX    579 /* 13MHz / (579 + 1) = 22.4kHz */

static bool g_bAmpEnabled = false;
#define GPIO_LIN_MOTOR_PWM  		29
#define GPIO_LIN_MOTOR_EN		    158
#define VIBE_IC_VOLTAGE         3000000

#define MSM_PDM_BASE_REG		    0x18800040
#define GP_MN_CLK_MDIV_REG		  0xC
#define GP_MN_CLK_NDIV_REG		  0x10
#define GP_MN_CLK_DUTY_REG		  0x14

#define GP_MN_M_DEFAULT			    4
#define GP_MN_N_DEFAULT			    3425
#define GP_MN_D_DEFAULT		      (GP_MN_N_DEFAULT >> 1)

#define PWM_MAX_DUTY            GP_MN_N_DEFAULT - GP_MN_M_DEFAULT
#define PWM_MIN_DUTY            GP_MN_M_DEFAULT

#define GPMN_M_MASK             0x01FF
#define GPMN_N_MASK				      0x1FFF
#define GPMN_D_MASK				      0x1FFF

#define REG_WRITEL(value, reg)	writel(value, (MSM_PDM_BASE+reg))
#define REG_READL(reg)	        readl((MSM_PDM_BASE+reg))

#define VIBE_NO_ERROR           0
#define VIBE_ERROR_1            -1
#define VIBE_ERROR_2            -2

static int vibe_access_status = VIBE_NO_ERROR;

static struct regulator *snddev_reg_l1 = NULL;
static bool snddev_reg_l1_status = false;

static void vibrator_gpio_enable (int enable)
{
  if(vibe_access_status == VIBE_ERROR_1) return;
  
	if (enable)
		gpio_set_value_cansleep(GPIO_LIN_MOTOR_EN, 1);
	else 	
		gpio_set_value_cansleep(GPIO_LIN_MOTOR_EN, 0);
}

static void vibrator_LDO_enable(int enable)
{
  int rc;

//                                                             

  if (NULL == snddev_reg_l1) {
    snddev_reg_l1 = regulator_get(NULL, "8901_l1");

    if (IS_ERR(snddev_reg_l1)) {
      pr_err("LGE: VIB %s: regulator_get(%s) failed (%ld)\n", __func__,
             "l1", PTR_ERR(snddev_reg_l1));

      vibe_access_status = VIBE_ERROR_2;
      return;
    } 
  }
  if(enable == snddev_reg_l1_status) return;
  if (enable) {
    rc = regulator_set_voltage(snddev_reg_l1, VIBE_IC_VOLTAGE, VIBE_IC_VOLTAGE);
    if (rc < 0)
      pr_err("LGE:  VIB %s: regulator_set_voltage(l1) failed (%d)\n",
      __func__, rc);
      
    rc = regulator_enable(snddev_reg_l1);

    if (rc < 0)
      pr_err("LGE: VIB %s: regulator_enable(l1) failed (%d)\n", __func__, rc);
    snddev_reg_l1_status = true;

  } else {
    rc = regulator_disable(snddev_reg_l1);
    if (rc < 0)
      pr_err("%s: regulator_disable(l1) failed (%d)\n", __func__, rc);
    snddev_reg_l1_status = false;
  } 

  return;
}

static void vib_enable(int on )
{
	vibrator_gpio_enable(on);
	vibrator_LDO_enable(on);
}

static void vib_generatePWM(int enable, int amp)
{
	uint M_VAL = GP_MN_M_DEFAULT;
	uint N_VAL = GP_MN_N_DEFAULT;
  uint D_VAL = GP_MN_D_DEFAULT;

	void __iomem *vib_base_ptr = 0;

	vib_base_ptr = ioremap_nocache(MSM_PDM_BASE_REG,0x20 );

	writel((M_VAL & GPMN_M_MASK), vib_base_ptr + GP_MN_CLK_MDIV_REG );
	writel((~( N_VAL - M_VAL )&GPMN_N_MASK), vib_base_ptr + GP_MN_CLK_NDIV_REG);

  if (enable) 
  {
    gpio_tlmm_config(GPIO_CFG(GPIO_LIN_MOTOR_PWM, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);

    D_VAL = (((amp + 128)*PWM_MAX_DUTY) >> 8);

    if (D_VAL > PWM_MAX_DUTY ) D_VAL = PWM_MAX_DUTY;
    if (D_VAL < PWM_MIN_DUTY ) D_VAL = PWM_MIN_DUTY;

    writel(D_VAL & GPMN_D_MASK, vib_base_ptr + GP_MN_CLK_DUTY_REG);
  } 
  else 
  {
    writel(GP_MN_D_DEFAULT & GPMN_D_MASK, vib_base_ptr + GP_MN_CLK_DUTY_REG);
    gpio_tlmm_config(GPIO_CFG(GPIO_LIN_MOTOR_PWM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
  }
  
  iounmap(vib_base_ptr);

  return;
}

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	  DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable start[%d]\n", g_bAmpEnabled ));
    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));
        g_bAmpEnabled = false;

    		vib_enable(false);
    		vib_generatePWM(false ,0);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    if(vibe_access_status != VIBE_NO_ERROR){
        DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_AmpEnable vibe_access_status =  %d \n", vibe_access_status ));
        vib_enable(false);
        return VIBE_E_FAIL;
    }

    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

        g_bAmpEnabled = true;

    		vib_generatePWM(true, 0);
    		vib_enable(true);
    }
    else
    {
        DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnable [%d]\n", g_bAmpEnabled ));
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
	int ret = 0;
  snddev_reg_l1 = NULL;
  vibe_access_status = VIBE_NO_ERROR;

  DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));
    
//	g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

  /* GPIO setting for Motor EN*/
  ret = gpio_request(GPIO_LIN_MOTOR_EN, "lin_motor_en");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for vibrator\n", __func__, GPIO_LIN_MOTOR_EN);
    vibe_access_status = VIBE_ERROR_1;
    goto err_pwm;
	}

//	gpio_direction_output(GPIO_LIN_MOTOR_EN, 0);

//   	ImmVibeSPI_ForceOut_AmpDisable( 0 );	
    vibrator_gpio_enable(0);    
//    vib_generatePWM(false, 0);

    return VIBE_S_SUCCESS;

err_pwm:
    return VIBE_E_FAIL;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

	  ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

bool bInTestMode = 0; /*                                             */

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    VibeInt8 nForce;

    if(vibe_access_status != VIBE_NO_ERROR){
        DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples vibe_access_status =  %d \n", vibe_access_status ));
        return VIBE_E_FAIL;
    }

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1){
                DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
                return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2){
                DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
                return VIBE_E_FAIL;
            }
            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
            return VIBE_E_FAIL;
    }

    DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nForce =  %d \n", nForce ));

    /* Check the Force value with Max and Min force value */
    if (nForce > 127) nForce = 127;
    if (nForce < -127) nForce = -127;

    if (nForce == 0)
    {        
        vibrator_gpio_enable(0);
        vib_generatePWM(false, nForce);
    }    
    else
    {
        vibrator_gpio_enable(1);            
        vib_generatePWM(true, nForce);
    }        

    return VIBE_S_SUCCESS;
}

#if 0
/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    /* This function is not called for ERM device */

    return VIBE_S_SUCCESS;
}
#endif 

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as a sample. Please modify as required. */
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName.\n"));

    strncpy(szDevName, "Generic Linux Device", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}
