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


/*=============================================================================
Project  :  VideoCore Software Host Interface (Host-side functions)
Module   :  Host-specific functions
File     :  $RCSfile: vcil.c,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
This contains the task of the IL component service running on the host
=============================================================================*/

//#include <stdio.h>
#include <asm/current.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vcilcs.h>
#include <linux/broadcom/vc03/vcos.h>

static struct task_struct* vcilcs_task = NULL;
static int vcilcs_task_stopped = 1;

int vc_ilcs_task(void* data)
{
   void *event;

   event = vc_ilcs_read_event();   
   vc_event_blocking();
   vcilcs_task_stopped = 0;   
   
   while(1)
   {
      vc_event_wait(event);
      vc_ilcs_message_handler();
      
      if(vcilcs_task_stopped)
	break;
   }

   vcilcs_task = NULL;
   return 0;
}

void vcilcs_task_func(uint argc, void*argv)
{
  vcilcs_task = get_current();

  vc_ilcs_task(NULL);   
}
   
int vc_identify_vcilcs_task(void)
{
  return get_current() == vcilcs_task;
}

struct task_struct* vc_ilcstask_get(void)
{
  return vcilcs_task;
}

void  vc_ilcstask_stop(void)
{
  vcilcs_task_stopped = 1;
}

/******************************************************************************
NAME
   vcil_iface_init

SYNOPSIS
   int vcil_iface_init()

FUNCTION
   This initializes VC IL interface and starts vc_ilcs_task

RETURNS
   int
******************************************************************************/
int vcil_iface_init(void)
{
  struct task_struct* task;
  
  if(0 >  vc_ilcs_init())
	{
	  printk("VC3 OMX IL: initialization failed\n");
	  return -1;
	}

  task = kthread_run(vc_ilcs_task, NULL, "vc03_ilcs_task");
  if(NULL==task || IS_ERR(task))
	{
	  printk("VC3 OMX IL: initialization failed.\n");
	  return -1;
	}
  
  return 0;
}

/******************************************************************************
NAME
   vcil_iface_exit

SYNOPSIS
   int vcil_iface_exit()

FUNCTION
   This initializes VC IL interface and starts vc_ilcs_task

RETURNS
   int
******************************************************************************/
void vcil_iface_exit(void)
{
  vc_ilcstask_stop();
  while(NULL != vc_ilcstask_get())
	{
	  os_sleep(100);
	}
  
  vc_ilcs_stop();
}

