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
FILE DESCRIPTION
Host-side interface to VMCS OpenGL/ES proxy service

Linux version  6/9/08

When compiled for user space, provides these functions:
*   vc_ogles_init(void)		Search for the ogles service number (inum), stash it;
    				initialiaze the msgfifo interface for the ogles inum.
**  vc_ogles_start(void)	Start the service (load ogles driver VLL in VC3).
    vc_ogles_inum(void)		Return the ogles service number.

When compied for kernel space, provides all the above functions plus these:
    vc_ogles_stop(void)		Stop the service (unload ogles driver VLL in VC3).
    vc_ogles_exit(void)		Remove ogles event from interrupt event list.

*  In kernel space, this function also creates ogles_event and ogles_lock, and adds
   ogles_event to the interrupt event list.

** This one may be removed from user space if it is not needed there.
=============================================================================*/


#if defined( __KERNEL__ )
#include <linux/string.h>
#else
#include <string.h>
#endif

#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcogles.h"
#include "vcgencmd.h"

/*******************************************************************************
Static data
*******************************************************************************/

static int ogles_inum = -1;

#if defined( __KERNEL__ )
void *ogles_lock = NULL;
void *ogles_event = NULL;
#endif // defined( __KERNEL__ )

/*******************************************************************************
NAME
   vc_ogles_init

SYNOPSIS
   int vc_ogles_init(void)

FUNCTION
   Initialise the opengl es service

RETURNS
   int
*******************************************************************************/
int vc_ogles_init(void)
{
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

#if defined( __KERNEL__ )
   if (ogles_lock == NULL)
      ogles_lock = vc_lock_create();

   vc_lock_obtain(ogles_lock);

   if (ogles_event == NULL)
      ogles_event = vc_event_create();
#endif // __KERNEL__

   // We simply loop through every interface that there is and look for one
   // that claims to be a OpenGL/ES service
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_OGLESSVC) {
            // Gotcha!
            ogles_inum = i;
#if defined( __KERNEL__ )
            vc_interface_register_event_int(ogles_event, (1<<ogles_inum));
#endif // __KERNEL__
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

#if defined( __KERNEL__ )
   vc_lock_release(ogles_lock);
#endif // __KERNEL__
   return ogles_inum;
}

/*******************************************************************************
NAME
   vc_ogles_start

SYNOPSIS
   int vc_ogles_start(void)

FUNCTION
   Start the ogles service (cause the driver to be loaded)

RETURNS
   int 0 on success, -ve on error
*******************************************************************************/
int vc_ogles_start(void)
{
   char response[ 100 ];

   vc_gencmd( response, 100, "ogles_load_driver" );

   if (!strncmp(response, "ok", 2)) return 0;
   return -1;
}

#if defined( __KERNEL__ )
/*******************************************************************************
NAME
   vc_ogles_stop

SYNOPSIS
   int vc_ogles_stop(void)

FUNCTION
   Stop the ogles service (cause the driver to be unloaded)

RETURNS
   int 0 on success, -ve on error
*******************************************************************************/
int vc_ogles_stop(void)
{
   char response[ 100 ];

   vc_gencmd( response, 100, "ogles_unload_driver" );

   if (!strncmp(response, "ok", 2)) return 0;
   return -1;
}

/*******************************************************************************
NAME
   vc_ogles_exit

SYNOPSIS
   void vc_ogles_exit(void)

FUNCTION
   Note that the ogles service has stopped

RETURNS
   -
*******************************************************************************/
void vc_ogles_exit(void)
{
   vc_lock_obtain(ogles_lock);
   ogles_inum = -1;
   vc_interface_register_event_int(ogles_event, 0);
   vc_lock_release(ogles_lock);
}
#endif // __KERNEL__

/*******************************************************************************
NAME
   vc_ogles_inum

SYNOPSIS
   int vc_ogles_inum(void)

FUNCTION
   Return the service number

RETURNS
   int
*******************************************************************************/
int vc_ogles_inum(void)
{
   return ogles_inum;
}
