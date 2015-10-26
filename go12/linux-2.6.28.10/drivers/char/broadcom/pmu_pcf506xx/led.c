/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/




/*
*
*****************************************************************************
*
*  led.c
*
*  PURPOSE:
*
*     This implements the LED interface for the PCF506xx chips.
*
*  NOTES:
*
*****************************************************************************/



/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/led.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_pcf506xx.h>

#include "led.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define NUM_LEDS    2

/* ---- Private Variables ------------------------------------------------ */

static BCM_PMU_Chip_t gPcfChip = PMU_NUM_CHIPS;

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */
// Calculates best period and pattern values for 'standard' led registers. This is returned
// as a byte with the period in 2:0 and the pattern in 5:3
// This is common code to all 3 PMU LED drivers.
static unsigned char calcPeriodandPattern(unsigned int period, unsigned int pulses, unsigned int ontime)
{
   unsigned char periodval;
   unsigned char patternval;
   if      ( period <= 7 )    { periodval = 0; }
   else if ( period <= 11)    { periodval = 1; }
   else if ( period <= 16)    { periodval = 2; }
   else if ( period <= 23)    { periodval = 3; }
   else if ( period <= 33)    { periodval = 4; }
   else if ( period <= 50)    { periodval = 5; }
   else if ( period <= 70)    { periodval = 6; }
   else                       { periodval = 7; }

   if ( pulses == 1 )
   {
      if      ( ontime <= 75 )   { patternval = 0; }
      else if ( ontime <= 150 )  { patternval = 1; }
      else if ( ontime <= 350 )  { patternval = 2; }
      else                       { patternval = 3; }
   }
   else // if ( pulses >= 2 )
   {
      if      ( ontime <= 75 )   { patternval = 4; }
      else if ( ontime <= 150 )  { patternval = 5; }
      else                       { patternval = 6; }
   }
   return (periodval + (patternval << 3));
}


/****************************************************************************
*
*  led506xx_ioctl
*
***************************************************************************/
static int led506xx_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   led_ctrl_t ctrl;
   unsigned char ledctrlreg;

   switch ( cmd )
   {
      case LED_IOCTL_CTRL:
         if ( copy_from_user( &ctrl, (led_ctrl_t *)arg, sizeof( led_ctrl_t )) != 0 )
         {
            return -EFAULT;
         }
         if ( ctrl.led > NUM_LEDS )
         {
            printk("led_ioctl - led %d not supported\n", ctrl.led);
            return -EINVAL;
         }
         // FIXME - the hardware LED mapping should come from a board specific file
         ledctrlreg = ( ctrl.led == LED_POWER_LIGHT ) ? PCF506XX_REG_LED1C(gPcfChip) : PCF506XX_REG_LED2C(gPcfChip);

         switch ( ctrl.mode )
         {
            case LED_MODE_OFF:
               pmu_i2c_write( ledctrlreg, 0 );
               break;

            case LED_MODE_SOLID_ON:
               pmu_i2c_write( ledctrlreg, (PCF506XX_BIT_LED_PAT7|PCF506XX_BIT_LED_ACT_SET|PCF506XX_BIT_LED_CHG_SET));
               break;

            case LED_MODE_FLASH_PATTERN:
               pmu_i2c_write( ledctrlreg, calcPeriodandPattern(ctrl.period, ctrl.pulses, ctrl.ontime)
                              + PCF506XX_BIT_LED_ACT_SET + PCF506XX_BIT_LED_CHG_SET);
               break;
         }
         break;

      default:
         printk("led_ioctl - Unrecognized ioctl: '0x%x'\n", cmd);
         return -ENOTTY;
   }
   return 0;
}

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations led506xx_fops =
{
   owner:      THIS_MODULE,
   ioctl:      led506xx_ioctl,
};

/****************************************************************************
*
*  led506xx_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int led506xx_init( BCM_PMU_Chip_t chip )
{
   int rc;

   if ((chip != PMU_PCF50603) && (chip != PMU_PCF50611))
   {
      printk( "led506xx: chip ID %d not supported.\n", chip);
      return -ENODEV;
   }

   if (gPcfChip != PMU_NUM_CHIPS)
   {
      printk( "led506xx: driver already loaded for chip %d.\n", gPcfChip);
      return -EBUSY;
   }

   printk( "led506xx: register_chrdev\n");
   if (( rc = register_chrdev( BCM_LED_MAJOR, "led", &led506xx_fops )) < 0 )
   {
       printk( "led506xx: register_chrdev failed for major %d\n", BCM_LED_MAJOR );
       return rc;
   }

   /* Save chip ID */
   gPcfChip = chip;

   /* connect LED1 to gpo1 and put gpo2 in high impedance */
   rc = pmu_i2c_write(PCF506XX_REG_GPOC1(gPcfChip), PCF506XX_BIT_GPO_LED1|(PCF506XX_BIT_GPO_HIGHIMP << 3));

   /* connect LED2 to gpo3 */
   rc |= pmu_i2c_write(PCF506XX_REG_GPOC2(gPcfChip), PCF506XX_BIT_GPO_LED2);

   /* initialize LED2 to off */
   rc |= pmu_i2c_write(PCF506XX_REG_LED2C(gPcfChip), 0 );

   return 0;
}

