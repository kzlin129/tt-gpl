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
*     This implements the LED interface for the BCM59001 chip.
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
#include <linux/broadcom/pmu_bcm59001.h>

#include "led.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define NUM_LEDS    2

/* ---- Private Variables ------------------------------------------------ */

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
*  led59001_ioctl
*
***************************************************************************/
static int led59001_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    led_ctrl_t ctrl;
    unsigned char ledctrlreg;   // control register
    unsigned char ledpatreg;    // pattern register
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
            // The hardware LED mapping is assumed consistent for all 59001 layouts. If this
            // assumption ever changed, then move the led definitions to a board specific file.
            ledctrlreg = ( ctrl.led == LED_POWER_LIGHT ) ? BCM59001_REG_PWMLEDCTRL6 : BCM59001_REG_PWMLEDCTRL1;
            ledpatreg  = ( ctrl.led == LED_POWER_LIGHT ) ? BCM59001_REG_PWMLEDCTRL7 : BCM59001_REG_PWMLEDCTRL2;

            switch ( ctrl.mode )
            {
                case LED_MODE_OFF:
                    pmu_i2c_write( ledctrlreg, 0 );
                    break;

                case LED_MODE_SOLID_ON:
                    pmu_i2c_write( ledctrlreg, 1 );
                    pmu_i2c_write( ledpatreg, BCM59001_VAL_LED_ALWAYS_ON);
                    break;

                case LED_MODE_FLASH_PATTERN:
                    pmu_i2c_write( ledctrlreg, 1 );
                    pmu_i2c_write( ledpatreg, calcPeriodandPattern(ctrl.period, ctrl.pulses, ctrl.ontime));
                    break;

                default:
                    printk("led_ioctl - invalid mode %d\n", ctrl.mode);
                    return -EINVAL;
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

struct file_operations led59001_fops =
{
   owner:      THIS_MODULE,
   ioctl:      led59001_ioctl,
};

/****************************************************************************
*
*  led59001_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
int led59001_init( void )
{
    int rc;
    printk( "led59001: register_chrdev\n");
    if (( rc = register_chrdev( BCM_LED_MAJOR, "led", &led59001_fops )) < 0 )
    {
        printk( "led59001: register_chrdev failed for major %d\n", BCM_LED_MAJOR );
        return rc;
    }
    rc |= pmu_i2c_write(BCM59001_REG_PWMLEDCTRL1, 0);        // LED1 off
    rc |= pmu_i2c_write(BCM59001_REG_PWMLEDCTRL5, 0x16 );    // LED1 60 mA sink, LED2 10 mA sink, powered up
    rc |= pmu_i2c_write(BCM59001_REG_PWMLEDCTRL6, 0);        // LED2 off
    rc |= pmu_i2c_write(BCM59001_REG_PWMLEDCTRL10, 3 );      // LED1/2 mode
    return 0;
}

