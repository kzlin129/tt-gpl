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




#include <string.h>

#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcmsgfifo.h"

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

/******************************************************************************
Static data.
******************************************************************************/

static uint32_t touch_vc_addr;

/******************************************************************************
Static functions.
******************************************************************************/

/******************************************************************************
NAME
   vc_touch_init

SYNOPSIS
   int vc_touch_init()

FUNCTION
   Initialise the touchscreen service for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

int vc_touch_init (void) {
   int touch_inum = -1;
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   // We simply loop through every interface that there is and look for one
   // that claims to be a display manager service.
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (generic_interface.stype == VC_STYPE_TOUCHSCREEN) {
            // Gotcha!
            touch_inum = i;
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   touch_vc_addr = vc_interface_base + vc_sharedmem_header.iface[touch_inum] + sizeof(VC_REGBLOCK_INTERFACE_T);
   return touch_inum;
}

/******************************************************************************
NAME
   vc_touch_fetch

SYNOPSIS
   void vc_touch_fetch(VC_TOUCH_PARAMS_T *params)

FUNCTION
   Return current values for touchscreen register block, after re-reading them
   from VideoCore. The passed parameter block memory must be supplied by the caller.

RETURNS
   void
******************************************************************************/

void vc_touch_fetch (VC_TOUCH_PARAMS_T *params) {
   vc_host_read_consecutive((void *)params, touch_vc_addr, sizeof(VC_TOUCH_PARAMS_T), 0);
}
