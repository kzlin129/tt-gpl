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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/cdev.h>

#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <linux/broadcom/vc03/vchi/vchi.h>


VCHI_SERVICE_HANDLE_T khrn_service_handle;


//used to signal msg arrivals
OS_COUNT_SEMAPHORE_T        khronos_msg_avail;


static void
khronos_vchi_callback( void *callback_param,
                       const VCHI_CALLBACK_REASON_T reason,
                       const void *msg_handle )
{
   OS_SEMAPHORE_T *sem;
   int32_t success;

   switch( reason ) {

   case VCHI_CALLBACK_MSG_AVAILABLE:
      sem = (OS_SEMAPHORE_T *)callback_param;
      if ( sem == NULL )
         return;

      success = os_count_semaphore_release(sem);
      break;

   case VCHI_CALLBACK_BULK_RECEIVED:
   case VCHI_CALLBACK_BULK_SENT:
      sem = (OS_SEMAPHORE_T *)callback_param;
      if ( sem == NULL )
         return;

      success = os_count_semaphore_release(sem);
      assert( success >= 0 );
      break;

   case VCHI_CALLBACK_MSG_SENT:
      assert(0);
      break;

   default:
      break;
   }
}


/*
 * Create a 'KHRN' service on the each of the connections
 */
void
vc_vchi_khronos_init( VCHI_INSTANCE_T initialise_instance,
                      VCHI_CONNECTION_T **connections,
                      uint32_t num_connections )
{
   int32_t success;
   VCHI_SERVICE_HANDLE_T vchi_handle;

   // create various local resources
   success = os_count_semaphore_create( &khronos_msg_avail, 0, OS_SEMAPHORE_TYPE_SUSPEND ); // starts out 'locked'
   assert( success == 0 );

   {
   SERVICE_CREATION_T parameters = { MAKE_FOURCC("KHRN"),    // 4cc service code
                                     connections[0],         // passed in fn ptrs
                                     0,                      // tx fifo size (unused)
                                     0,                      // tx fifo size (unused)
                                     &khronos_vchi_callback, // service callback
                                     &khronos_msg_avail,     // callback parameter 
                                     VC_TRUE,               // Want unaligned bulk rx 
                                     VC_TRUE };              // Want unaligned bulk tx


   // the following assert is just because this hasn't been tested at the mo
   assert( num_connections == 1 );

   success = vchi_service_open( initialise_instance, &parameters, &vchi_handle );
   assert( success == 0 );
   }

   khrn_service_handle = vchi_handle;
}
