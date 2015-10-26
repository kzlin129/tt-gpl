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
*  trample-test.c
*
*  PURPOSE:
*
*     Test code to trample some memory and see what happens.
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
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/ioctls.h>

#include <linux/trample.h>

// The following header files are included so that we can get extern's for 
// the functions we're trying to get addresses of.

#include <linux/zlib.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define DEBUG_ENABLED   1

#if DEBUG_ENABLED
#   define DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#   define DEBUG( flag, fmt, args... )
#endif  

#define  KERNEL_CHECK_COPY 0

/* ---- Private Variables ------------------------------------------------ */

static char gBanner[] __initdata = KERN_INFO "Trample Test Module Compiled: " __DATE__ " at " __TIME__ "\n";

static int do_test( ctl_table *table, int write, struct file *filp,
                    void __user *buffer, size_t *lenp, loff_t *ppos );

static int do_trample( ctl_table *table, int write, struct file *filp,
                       void __user *buffer, size_t *lenp, loff_t *ppos );

#if KERNEL_CHECK_COPY
static int do_checkcopy( ctl_table *table, int write, struct file *filp,
                         void __user *buffer, size_t *lenp, loff_t *ppos );
static  int     gCheckCopy;
static  void   *gKernelCopy = NULL;

#endif

static  int         gInitialized = 0;

static  int         gDebugTrace = 1;
static  int         gTrampleVal = 0xffffffff;
static  int         gDoIt = 0;
static  int         gTest;
static  int         gDoIt2;
static  unsigned    gChecksum;

static struct ctl_table gSysCtlTrample[] = {
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
      .procname         = "value",
      .data             = &gTrampleVal,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 3,
      .procname         = "doit1",
      .data             = &gDoIt,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &do_trample
   },
   {
      .ctl_name         = 4,
      .procname         = "doit2",
      .data             = &gDoIt2,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 5,
      .procname         = "test",
      .data             = &gTest,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &do_test
   },
   {
      .ctl_name         = 6,
      .procname         = "checksum",
      .data             = &gCheckSum,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
#if KERNEL_CHECK_COPY
   {
      .ctl_name         = 7,
      .procname         = "checkcopy",
      .data             = &gCheckCopy,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &do_checkcopy
   },
#endif
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name         = 517,
      .procname         = "trample",
      .mode             = 0555,
      .child            = gSysCtlTrample
   },
   {}
};

static  struct ctl_table_header    *gSysCtlHeader = NULL;

typedef struct
{
   char       *label;
   uint32_t   *testMem;
   size_t      numBytes;

   uint32_t   *goodMem;

} check_t;

extern   unsigned _text;
extern   unsigned _etext;

#define  KERNEL_CODE_SIZE  ((char * )&_etext - (char *)&_text )

void do_ri(struct pt_regs *regs);

static check_t gCheck[] =
{
#if defined( CONFIG_MIPS )
   { "less_than_4units", (uint32_t *)memcpy + 49, 40 },
   { "do_ri",            (uint32_t *)do_ri + ((0x344 - 20 ) / 4), 40 },
#endif
   { "doit2",            (uint32_t *)&gDoIt2, sizeof( gDoIt2 ) },
   { NULL, NULL, 0 }
};

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

#if KERNEL_CHECK_COPY
/****************************************************************************
*
*  Performs a checksum over the entire kernel code area
*
***************************************************************************/

unsigned CompareKernel( void )
{
   unsigned *src = (unsigned *)&_text;
   unsigned *cpy = gKernelCopy;

   if ( cpy == NULL )
   {
      printk( "Unable to compare kernel - no copy found\n" );
      return 1;
   }

   while ( src < (unsigned *)&_etext )
   {
      if ( unlikely( *src != *cpy ))
      {
         printk( "Mismatch detected at 0x%08lx. Found 0x%08x expecting 0x%08x\n",
                 (unsigned long)src, *src, *cpy );
         return 0;
      }

      src++;
      cpy++;
   }

   return 1;

} // CompareKernel
#endif

/****************************************************************************
*
*  Performs a checksum over the entire kernel code area
*
***************************************************************************/

unsigned ChecksumKernel( void )
{
   unsigned sum = 0;
   unsigned *p = &_text;

   while ( p < &_etext )
   {
      sum += *p;

      p++;
   }
   
   return sum;

} // ChecksumKernel

/****************************************************************************
*
*  Tests to see if a trample has been detected.
*
***************************************************************************/

void TestForTrample( const char *fileName, int lineNum )
{
   int   i;

   if ( gInitialized )
   {
      for ( i = 0; gCheck[ i ].label != NULL; i++ ) 
      {
         if ( memcmp( gCheck[ i ].goodMem, gCheck[ i ].testMem, gCheck[ i ].numBytes ) != 0 )
         {
            unsigned testSum;

            console_verbose();

            printk( KERN_ERR "TrampleTest: %s has been corrupted (%s : %d)\n", gCheck[ i ].label, fileName, lineNum );

            // Since we're now checking from within __do_IRQ, setting 
            // gInitialized to zero, causes us not to generate an infinite

            gInitialized = 0;

            // Since we think we've detected a trample, Check the kernel checksum as well

            testSum = ChecksumKernel();

            if ( testSum != gChecksum )
            {
               printk( "Kernel checksum 0x%08x doesn't match 0x%08x\n", testSum, gChecksum );
            }
            else
            {
               printk( "Kernel checksum is fine\n" );
            }

            *(uint32_t *)0 = 0;

         }
      }
   }

} // TestForTrample

EXPORT_SYMBOL(TestForTrample);

#if KERNEL_CHECK_COPY
/****************************************************************************
*
*  do_checkcopy - called in response to:
*
*     echo 1 > /proc/sys/trample/check-copy
*
***************************************************************************/

static int do_checkcopy( ctl_table *table, int write, struct file *filp,
                         void __user *buffer, size_t *lenp, loff_t *ppos )
{
   if ( CompareKernel() )
   {
      printk( "Kernel compared OK\n" );
   }
   else
   {
      printk( "Kernel comparison failed\n" );
   }
   
   return proc_dointvec( table, write, filp, buffer, lenp, ppos );

} // do_checkcopy
#endif

/****************************************************************************
*
*  do_test - called in response to:
*
*     echo 1 > /proc/sys/trample/test
*
***************************************************************************/

static int do_test( ctl_table *table, int write, struct file *filp,
                    void __user *buffer, size_t *lenp, loff_t *ppos )
{
   unsigned testSum;

   TESTFORTRAMPLE();

   testSum = ChecksumKernel();

   if ( testSum != gChecksum )
   {
      printk( "Kernel checksum 0x%08x doesn't match 0x%08x\n", testSum, gChecksum );
   }
   else
   {
      printk( "Kernel checksum is fine\n" );
   }
   
   return proc_dointvec( table, write, filp, buffer, lenp, ppos );

} // do_test

/****************************************************************************
*
*  do_trample - called in response to:
*
*     echo 1 > /proc/sys/trample/doit
*
***************************************************************************/

static int do_trample( ctl_table *table, int write, struct file *filp,
                       void __user *buffer, size_t *lenp, loff_t *ppos )
{
#if defined( CONFIG_MIPS )
   uint32_t *trample;

   if ( write )
   {
      trample = (uint32_t *)memcpy;
      trample += 54;

      *trample = gTrampleVal;
   }
#else
   printk( KERN_ERR "do_trample: only supported on the MIPS\n" );
#endif

   return proc_dointvec( table, write, filp, buffer, lenp, ppos );

} // do_trample

/****************************************************************************
*
*  trample_init_drv
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init trample_init_drv( void )
{
   int   i;

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

   for ( i = 0; gCheck[ i ].label != NULL; i++ ) 
   {
      gCheck[ i ].goodMem = kmalloc( gCheck[ i ].numBytes, GFP_KERNEL );
      memcpy( gCheck[ i ].goodMem, gCheck[ i ].testMem, gCheck[ i ].numBytes );
   }

   gChecksum = ChecksumKernel();

   printk( "Initial Kernel checksum 0x%08x\n", gChecksum );

#if KERNEL_CHECK_COPY
   gKernelCopy = vmalloc( KERNEL_CODE_SIZE );
   if ( gKernelCopy == NULL )
   {
      printk( KERN_ERR "Unable to allocate %d bytes to copy kernel into\n", KERNEL_CODE_SIZE );
   }
   else
   {
      memcpy( gKernelCopy, &_text, KERNEL_CODE_SIZE );
   }
#endif

   gInitialized = 1;

   return 0;

} // trample_init_drv

/****************************************************************************
*
*  trample_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit trample_exit_drv( void )
{
   DEBUG( Trace, "called\n" );

   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }

} // trample_exit_drv

module_init(trample_init_drv);
module_exit(trample_exit_drv);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Trample Test");
MODULE_LICENSE("GPL");

