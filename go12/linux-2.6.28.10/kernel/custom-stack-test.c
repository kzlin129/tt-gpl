/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*  custom-stack-test.c
*
*  PURPOSE:
*
*     This implements some test harnesses for testing the custom stack code.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/ioctls.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define DEBUG_ENABLED   1

#if DEBUG_ENABLED
#   define DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#   define DEBUG( flag, fmt, args... )
#endif  

/* ---- Private Variables ------------------------------------------------ */

static char gBanner[] __initdata = KERN_INFO "Custom Stack Test Module Compiled: " __DATE__ " at " __TIME__ "\n";


static  int         gDebugTrace = 0;
static  int         gStartThread = 0;

static int test_start_thread( ctl_table *table, int write, struct file *filp,
                              void __user *buffer, size_t *lenp, loff_t *ppos );

static struct ctl_table gSysCtlCustomStack[] = {
   {
      .ctl_name         = 1,
      .procname         = "debug-trace",
      .data             = &gDebugTrace,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 2,
      .procname         = "start-thread",
      .data             = &gStartThread,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &test_start_thread
   },
   {}
};

static struct ctl_table gSysCtl[] = {
   {
      .ctl_name         = 517,
      .procname         = "custom-stack",
      .mode             = 0555,
      .child            = gSysCtlCustomStack
   },
   {}
};

static  struct ctl_table_header    *gSysCtlHeader = NULL;


/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  test_thread
*
*     Test thread which consumes a bunch of stack space.
*
***************************************************************************/

static int test_thread( void *data )
{
   u8    bigMem[ 12000 ];
   u64   nextJiffies;
   int   i;

   printk( "test_thread: started &bigMem[0]: 0x%08lx TI:0x%08lx\n", 
           (unsigned long)&bigMem[0], (unsigned long)current_thread_info() );

   memset( bigMem, 0xEE, sizeof( bigMem ));

   printk( "test_thread: sleeping for 5 seconds\n" );

   set_current_state(  TASK_INTERRUPTIBLE );
   schedule_timeout( 5 * HZ );

   printk( "test_thread: busy for 5 seconds\n" );

   nextJiffies = jiffies + 5 * HZ;

   while ( jiffies < nextJiffies )
   {
      ;
   }

   printk( "test_thread: sleeping for 5 seconds\n" );

   set_current_state(  TASK_INTERRUPTIBLE );
   schedule_timeout( 5 * HZ );

   printk( "test_thread: Checking bigMem\n" );

   for ( i = 0; i < sizeof( bigMem ); i++ ) 
   {
      if ( bigMem[ i ] != 0xEE )
      {
         printk( "test_thread: bigMem[ %d ] = 0x%02x, expecting 0xEE\n", i, bigMem[ i ] );
         break;
      }
   }

   printk( "test_thread: Leaving\n" );

   return 0;
   
} // test_thread

/****************************************************************************
*
*  test_start_thread
*
*     Starts a thread test which uses a larger stack
*
***************************************************************************/

static int test_start_thread( ctl_table *table, int write, struct file *filp,
                              void __user *buffer, size_t *lenp, loff_t *ppos )
{
   kernel_thread( test_thread, NULL, CLONE_KERNEL | CLONE_CUSTOM_STACK );

   return proc_dointvec( table, write, filp, buffer, lenp, ppos );

} // test_start_thread

/****************************************************************************
*
*  test_init_drv
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init test_init_drv( void )
{
   DEBUG( Trace, "called\n" );

   printk( gBanner );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
   if ( gSysCtlHeader != NULL )
   {
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif

   return 0;

} // test_init_drv

/****************************************************************************
*
*  test_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit test_exit_drv( void )
{
   DEBUG( Trace, "called\n" );

   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }

} // test_exit_drv

module_init(test_init_drv);
module_exit(test_exit_drv);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Custom Stack Test Harness");
MODULE_LICENSE("GPL");

