/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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



/**
*
*  @file    vcassert_notify.c
*
*  @brief   Implementation of the VC02 assert handler on host.
*
****************************************************************************/


#include <linux/broadcom/vc.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/errno.h>


#include "vcgencmd.h"
#include "vchostreq.h"
#include "vcassert_notify.h"
#include "vclogging.h"

#define MAX_ASSERT_LINE_LEN   128
#define MAX_ASSERT_COND_LEN   1024

static void printVC02asserts(void);
static void vc_assert_notify_func(void);

static   uint32_t log_address;
/******************************************************************************
NAME
   vc_assert_notify_init

SYNOPSIS
   int vc_assert_notify_init(void)

FUNCTION

RETURNS
   void
******************************************************************************/

void vc_assert_notify_init(void)
{
   VC_GenCmd_t gencmd;
    if (vc_hostreq_set_notify(VC_HRNOTIFY_ASSERT, (void *)vc_assert_notify_func) != 0)
    {
       printk("Failure to register VC02 assert callback\n");
    }
   sprintf(&gencmd.cmd[0],"%s\n", "get_log_address");
   gencmd.response.err = vc_gencmd( gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );

   vc_gencmd_number_property(gencmd.response.str, "log_address", &log_address);

   vc_assert_notify_func();
}


static void printVC02asserts(void)
{
   static unsigned short num_msgs_retrieved = 0;
   uint32_t assert_time, assert_lineno;
   char *assert_filename;
   char *assert_cond;
   unsigned short seqNum;


   /* Filename (including path) is < 128 bytes */
   assert_filename =  kmalloc( MAX_ASSERT_LINE_LEN, GFP_KERNEL );
   assert_cond = kmalloc(MAX_ASSERT_COND_LEN, GFP_KERNEL);
   // retrieve VC02 ASSERT logs
   while ( vc_retrieve_assert_log( log_address,
                                   &assert_time,
                                   &seqNum,
                                   assert_filename,
                                   MAX_ASSERT_LINE_LEN,
                                   &assert_lineno,
                                   assert_cond,
                                   MAX_ASSERT_COND_LEN   ) )
   {
      if (!assert_lineno)
      {
         // no line number means it is an exception (for exceptions the VC02 sets
         // the filename string to "exception")
         printk("VC02 DBG_INFO: time %u : seq_num = %hu : %s : %s\n", assert_time, seqNum, assert_filename, assert_cond);
      }
      else
      {
         printk( "VC02 DBG_INFO: time %u : seq_num = %hu : file %s : line %u : cond(%s)\n",
                 assert_time, seqNum, assert_filename, assert_lineno, assert_cond);
      }

      num_msgs_retrieved++;
      if ( num_msgs_retrieved != seqNum )
      {
         printk( KERN_ERR "VC02 DBG_INFO : received seq_num = %hu, expected %hu\n", seqNum, num_msgs_retrieved );
         num_msgs_retrieved = seqNum;
      }
   }
   kfree(assert_filename);
   kfree(assert_cond);

}
/******************************************************************************
NAME
   vc_assert_notify_func

SYNOPSIS
   void vc_assert_notify_func

FUNCTION
   Notify function to be called for VC_HRNOTIFY_ASSERT notify events

RETURNS
   void
******************************************************************************/
static void vc_assert_notify_func(void)
{
   printVC02asserts();
}


