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
*  headset.c
*
*  PURPOSE:
*
*     This implements the headset detection interface for the PCF506xx chips.
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
#include <linux/delay.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/headset.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_pcf506xx.h>

#include "pcf506xx.h"
#include "headset.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

static BCM_PMU_Chip_t gPcfChip = PMU_NUM_CHIPS;

static atomic_t gHeadsetAvailable = ATOMIC_INIT(1);

DECLARE_WAIT_QUEUE_HEAD( gHsQueue );

static headset_state gHsState = HEADSET_UNPLUGGED;
static headset_event gHsEvent;

/* ---- Private Function Prototypes -------------------------------------- */
static void headset_wake(headset_event event);

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  rec2_506xx_SampMode
*
*  Turn on/off the 1 second sampling mode for REC2 accessory detection.
*  Also turn on/off mic bias so that its powered correctly.
*
***************************************************************************/
static void rec2_506xx_SampMode( int on )
{
    int rc = 0;
    unsigned char oocc;

    rc = pmu_i2c_read(PCF506XX_REG_OOCC(gPcfChip));
    if ( rc != -1 )
    {
        oocc = (unsigned char)rc;
        if ( on )
        {
            oocc &= ~PCF506XX_BIT_MICB_EN;
            oocc |= PCF506XX_BIT_REC2_SAMP;
        }
        else
        {
            oocc |= PCF506XX_BIT_MICB_EN;
            oocc &= ~PCF506XX_BIT_REC2_SAMP;
        }
        rc = pmu_i2c_write( PCF506XX_REG_OOCC(gPcfChip), oocc );
    }
}

// High Falling Interrupt
static void rec2_506xx_hfISR(PCF506XX_InterruptId_t irq_id)
{
    headset_wake(HEADSET_INSERTED);

    /* use continuous mode for accessory detection */
    rec2_506xx_SampMode( 0 );
}

// Low Falling Interrupt
static void rec2_506xx_lfISR(PCF506XX_InterruptId_t irq_id)
{
    headset_wake(HEADSET_BUTTON);
}

// High Rising Interrupt
static void rec2_506xx_hrISR(PCF506XX_InterruptId_t irq_id)
{
    headset_wake(HEADSET_REMOVED);

    /* turn on the sampling mode for accessory detection */
    rec2_506xx_SampMode( 1 );
}

// Headset Wake Function, callable from different pmu-specific code.
// The PMU isr calls this function when the headset status changes.
static void headset_wake(headset_event event)
{
   gHsEvent = event;

   switch ( event )
   {
      case HEADSET_REMOVED:
         gHsState = HEADSET_UNPLUGGED;
         break;
      case HEADSET_INSERTED:
         gHsState = HEADSET_TOGGLE_A;
         break;
      case HEADSET_BUTTON:
         switch ( gHsState )
         {
            case HEADSET_TOGGLE_A:
               gHsState = HEADSET_TOGGLE_B;
               break;
            case HEADSET_TOGGLE_B:
               gHsState = HEADSET_TOGGLE_A;
               break;
            default:
               gHsState = HEADSET_TOGGLE_A;
               break;
         }
         break;
      default:
         break;
   }

   wake_up_interruptible( &gHsQueue );
}

/****************************************************************************
*
*  headset_506xx_init_regs
*
***************************************************************************/
static int headset_506xx_init_regs(void)
{
   int rc = 0;
   int val;

   /* enable headset recognition on REC2 */
   val = pmu_i2c_read(PCF506XX_REG_OOCC(gPcfChip));
   rc |= pmu_i2c_write(PCF506XX_REG_OOCC(gPcfChip), val | PCF506XX_BIT_REC2_EN );

   /* modify the high threshold for headset recognition to 19/21 * Vmicbias */
   rc |= pmu_i2c_write(PCF506XX_REG_REC2C(gPcfChip), 10 << 4 | 0);

   /* Enable headset interrupts */
   val = pmu_i2c_read(PCF506XX_REG_INT2M(gPcfChip));
   val &= ~(PCF506XX_BIT_REC2HF | PCF506XX_BIT_REC2LF | PCF506XX_BIT_REC2HR);
   rc |= pmu_i2c_write(PCF506XX_REG_INT2M(gPcfChip), val);

   return rc;
}

/****************************************************************************
*
*  headset_506xx_deinit_regs
*
***************************************************************************/
static int headset_506xx_deinit_regs(void)
{
   int rc = 0;
   int val;

   /* Disable headset interrupts */
   val = pmu_i2c_read(PCF506XX_REG_INT2M(gPcfChip));
   val |= (PCF506XX_BIT_REC2HF | PCF506XX_BIT_REC2LF | PCF506XX_BIT_REC2HR);
   rc |= pmu_i2c_write(PCF506XX_REG_INT2M(gPcfChip), val);

   /* Disable headset recognition on REC2 */
   val = pmu_i2c_read(PCF506XX_REG_OOCC(gPcfChip));
   rc |= pmu_i2c_write(PCF506XX_REG_OOCC(gPcfChip), val & (~PCF506XX_BIT_REC2_EN));

   return rc;
}

/****************************************************************************
*
*  headset_open
*
***************************************************************************/

static int headset_open( struct inode *inode, struct file *file )
{
   int rc;
   u8 oocs1;
   int headset_present = 0;

   /* Allow only 1 user */
   if (! atomic_dec_and_test (&gHeadsetAvailable))
   {
      printk("heasdset: already opened\n");
      atomic_inc(&gHeadsetAvailable);
      return -EBUSY;
   }

   /* Initialize registers and interrupts */
   rc = headset_506xx_init_regs();

   /* Set to continuos sampling for reliable headset detection */
   rec2_506xx_SampMode( 0 );
   /* Return error if initialization failed */
   if (rc)
   {
      atomic_inc(&gHeadsetAvailable);
      return rc;
   }

   mdelay(100);
   /* check for the headset */
   printk("Detecting headset presence...");
   rc = pmu_i2c_read(PCF506XX_REG_OOCS1(gPcfChip));

   if ( rc == -1 )
   {
      printk("Unable to read PCF506XX_REG_OOCS1\n");
      return 0;
   }

   oocs1 = (u8)rc;

   if (!( oocs1 & PCF506XX_BIT_REC2H ))
   {
      headset_present = 1;
   }
   else
   {
      headset_present = 0;
   }

   if ( headset_present )
   {
      printk("present\n");
      headset_wake(HEADSET_INSERTED);

      /* use continuous mode for accessory detection */
      rec2_506xx_SampMode( 0 );
   }
   else
   {
      printk("not present\n");
      /* headset is not plugged in */
      headset_wake(HEADSET_REMOVED);

      /* use sampling mode for accessory detection */
      rec2_506xx_SampMode( 1 );
   }
   return 0;

} /* headset_open */


/****************************************************************************
*
*  headset_release
*
***************************************************************************/

static int headset_release( struct inode *inode, struct file *file )
{
   /* Reset registers and interrupts */
   headset_506xx_deinit_regs();

   atomic_inc(&gHeadsetAvailable); /* release the device */
   return 0;
} /* headset_release */


/****************************************************************************
*
*  headset_ioctl
*
***************************************************************************/
static int headset_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   switch ( cmd )
   {
      case HEADSET_IOCTL_GET_STATE:
         if( copy_to_user( (unsigned long *)arg, &gHsState, sizeof( gHsState ) ) != 0 )
         {
            return -EFAULT;
         }
         break;

      default:
         printk("headset_ioctl - Unrecognized ioctl: '0x%x'\n", cmd);
         return -ENOTTY;
   }
   return 0;
}

/****************************************************************************
*
* headset_poll
*
* used to support the select system call
*
***************************************************************************/
static unsigned int headset_poll( struct file *file, struct poll_table_struct *poll_table )
{
   poll_wait( file, &gHsQueue, poll_table );

   if ( gHsEvent > HEADSET_NULL )
   {
      /* an interrupt occurred, set interrupt pending bit back to zero */
      gHsEvent = HEADSET_NULL;

      // Indicate that data is currently available
      return POLLIN | POLLRDNORM;
   }
   return 0;
}

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations headset_fops =
{
   owner:      THIS_MODULE,
   open:       headset_open,
   release:    headset_release,
   ioctl:      headset_ioctl,
   poll:       headset_poll,
};

/****************************************************************************
*
*  headset506xx_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int headset506xx_init( BCM_PMU_Chip_t chip )
{
   int rc;

   if ((chip != PMU_PCF50603) && (chip != PMU_PCF50611))
   {
      printk( "headset: chip ID %d not supported.\n", chip);
      return -ENODEV;
   }

   if (gPcfChip != PMU_NUM_CHIPS)
   {
      printk( "headset: driver already loaded for chip %d.\n", gPcfChip);
      return -EBUSY;
   }

   printk( "headset: register_chrdev\n");
   if (( rc = register_chrdev( BCM_HEADSET_MAJOR, "headset", &headset_fops )) < 0 )
   {
      printk( "headset: register_chrdev failed for major %d\n", BCM_HEADSET_MAJOR );
      return rc;
   }

   /* Save chip ID */
   gPcfChip = chip;

   /* Register interrupt handlers */
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT2REC2HF, rec2_506xx_hfISR);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT2REC2LF, rec2_506xx_lfISR);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT2REC2HR, rec2_506xx_hrISR);

   return 0;
}


