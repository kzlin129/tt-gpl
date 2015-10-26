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
*     This implements the headset detection interface for the BCM59001 chip.
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
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/headset.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59001.h>

#include "bcm59001.h"
#include "headset.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define HS_DEBUG(fmt,args...) printk( "%s: " fmt, __FUNCTION__, ##args )
//#define HS_DEBUG(fmt,args...)

/* ---- Private Variables ------------------------------------------------ */

static atomic_t gHeadsetAvailable = ATOMIC_INIT(1);

DECLARE_WAIT_QUEUE_HEAD( gBcm59001_HsQueue );

static headset_state gHsState = HEADSET_UNPLUGGED;
static headset_event gHsEvent;

/* ---- Private Function Prototypes -------------------------------------- */
static void headset_wake(headset_event event);

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  hs_statestr - convert state enum to string
*
***************************************************************************/
static char * hs_stateStr(headset_state state)
{
   switch (state) 
   {
      case HEADSET_UNPLUGGED:
         return "unplugged";
      case HEADSET_TOGGLE_A:
         return "toggle_a";
      case HEADSET_TOGGLE_B:
         return "toggle_b";
      default:
         return "illegal";
   }
}
/****************************************************************************
*
*  hs_eventStr - convert event enum to string
*
***************************************************************************/
static char * hs_eventStr(headset_event event)
{
   switch (event) 
   {
      case HEADSET_REMOVED:
         return "removed";
      case HEADSET_INSERTED:
         return "inserted";
      case HEADSET_BUTTON:
         return "button";
      default:
         return "illegal";
   }
}

// Headset Inserted
static void headset_inserted_isr(  BCM59001_InterruptId_t irq_id )
{
    headset_wake(HEADSET_INSERTED);
}

// Headset Removed
static void headset_removed_isr( BCM59001_InterruptId_t irq_id )
{
    headset_wake(HEADSET_REMOVED);
}

// Headset Button Press
static void headset_button_isr( BCM59001_InterruptId_t irq_id )
{
    headset_wake(HEADSET_BUTTON);
}

/****************************************************************************
*
*  headset_wake
*
* Headset Wake Function, callable from different pmu-specific code.
* The PMU isr calls this function when the headset status changes.
***************************************************************************/
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
   HS_DEBUG("event = %s, state = %s\n", hs_eventStr(event), hs_stateStr(gHsState));
   wake_up_interruptible( &gBcm59001_HsQueue );
}
/****************************************************************************
*
*  init_regs
*
***************************************************************************/
static int init_regs(void)
{
   int rc = 0;

   /* set the headset detection threshold high voltage */
   rc |= pmu_i2c_write( BCM59001_REG_TSTRECH, BCM59001_VAL_PHFD_HIGH_1_7V );

   // set the headet button debounce times to maximum. This avoids false
   // detections when blowing into the headset microphone.
   rc |= pmu_i2c_write( BCM59001_REG_PHFDDB, ( BCM59001_VAL_PHFDDDB_400MS | BCM59001_VAL_PHFDRDB_400MS ) );

   return rc;
}

/****************************************************************************
*
*  deinit_regs
*
***************************************************************************/
static int deinit_regs(void)
{
   int rc = 0;

   /* Need to do anything ? */

   return rc;
}

/****************************************************************************
*
*  init_interrupts
*
***************************************************************************/
static int init_interrupts(void)
{
    int rc = 0;
    int val;

    // Enable headset interrupts
    val = pmu_i2c_read(BCM59001_REG_INT3M);
    val &= ~(BCM59001_BIT_INT3_PHFDINS | BCM59001_BIT_INT3_PHFDRM 
           | BCM59001_BIT_INT3_PHFDPRS);
    rc |= pmu_i2c_write(BCM59001_REG_INT3M, val);

    return rc;
}

/****************************************************************************
*
*  deinit_interrupts
*
***************************************************************************/
static int deinit_interrupts(void)
{
    int rc = 0;
    int val;

    // Disable headset interrupts
    val = pmu_i2c_read(BCM59001_REG_INT3M);
    val |= (BCM59001_BIT_INT3_PHFDINS | BCM59001_BIT_INT3_PHFDRM 
          | BCM59001_BIT_INT3_PHFDPRS | BCM59001_BIT_INT3_PHFDRLS);
    rc |= pmu_i2c_write(BCM59001_REG_INT3M, val);

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

   /* Allow only 1 user */
   if (! atomic_dec_and_test (&gHeadsetAvailable))
   {
      HS_DEBUG("headsset: already opened\n");
      atomic_inc(&gHeadsetAvailable);
      return -EBUSY;
   }

   /* Initialize registers and interrupts */
   rc = init_regs();

   /* Enable headset interrupts */
   init_interrupts();
   
   /* Return error if initialization failed */
   if (rc)
   {
      atomic_inc(&gHeadsetAvailable);
      return rc;
   }

   /* check for the headset */
   HS_DEBUG("Detecting headset presence...\n");
   rc = pmu_i2c_read(BCM59001_REG_ENV2);
   if ( rc == -1 )
   {
      HS_DEBUG("Unable to read BCM59001_REG_ENV2\n");
      return 0;
   }
   if ( rc & BCM59001_BIT_ENV2_PHFD )
   {
      HS_DEBUG("present\n");
      headset_wake(HEADSET_INSERTED);
   }
   else
   {
      HS_DEBUG("not present\n");
      headset_wake(HEADSET_REMOVED);
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
   deinit_regs();

   /* Disable headset interrupts */
   deinit_interrupts();

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
         HS_DEBUG("state = %s\n", hs_stateStr(gHsState));
         break;

      default:
         HS_DEBUG("Unrecognized ioctl: '0x%x'\n", cmd);
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
   poll_wait( file, &gBcm59001_HsQueue, poll_table );

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

struct file_operations bcm59001_headset_fops =
{
   owner:      THIS_MODULE,
   open:       headset_open,
   release:    headset_release,
   ioctl:      headset_ioctl,
   poll:       headset_poll,
};

/****************************************************************************
*
*  headset59001_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int headset59001_init( void )
{
   int rc;

   HS_DEBUG( "headset: register_chrdev\n");
   if (( rc = register_chrdev( BCM_HEADSET_MAJOR, "headset", &bcm59001_headset_fops )) < 0 )
   {
      HS_DEBUG( "headset: register_chrdev failed for major %d\n", BCM_HEADSET_MAJOR );
      return rc;
   }

   /* Register interrupts */
   bcm59001_irq_register(BCM59001_IRQID_INT3_PHFDPRS,   headset_button_isr);
   bcm59001_irq_register(BCM59001_IRQID_INT3_PHFDRM,    headset_removed_isr);
   bcm59001_irq_register(BCM59001_IRQID_INT3_PHFDINS,   headset_inserted_isr);
   
   return 0;
}


