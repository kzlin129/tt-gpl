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



#ifndef VC_DRV_H
#define VC_DRV_H

#include <linux/sched.h> // struct task_struct
#include <linux/broadcom/vc03/vchi/vchi.h>  // VCHI_INSTANCE & friends

#define VC03VCHICONNECT_MAX 1

typedef struct vc03drv
{
  VCHI_INSTANCE_T instance_vchi;
  VCHI_CONNECTION_T* connection_vchi[VC03VCHICONNECT_MAX]; 
  int initialized;
} vc03drv_t;


inline vc03drv_t* vc03drv_get(void);

#define vc03_sleep_msec(msec)					\
  ({							\
   set_current_state(  TASK_INTERRUPTIBLE );		\
   schedule_timeout( msecs_to_jiffies( msec ));		\
  })

#define vc03_delay_msec(msec) os_sleep(msec)

#endif // #ifdef VC_DRV_H
