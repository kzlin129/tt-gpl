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



#ifndef __VCIFACE_H
#define __VCIFACE_H

#include "vcinterface.h"

/* Address of VideoCore shared memory base (set by vc_interface_init). */

extern uint32_t vc_interface_base;

/* VideoCore shared memory header. */

extern VC_SHAREDMEM_HEADER_T vc_sharedmem_header;

/* Initialise the host-side of the interface. */

int vc_interface_init(void);

/* Query which interfaces DO have a requested still pending on the VideoCore side. */

int vc_interface_query_req(void);

/* Request interrupts of VideoCore by setting interface bits. */

int vc_interface_set_req(int mask);

/* Set the interface int_req bits and send an interrupt to VideoCore. */

void vc_interface_send_interrupt(int mask);

/* Register with the interrupt handler an event to be signalled when a vc->host
   interrupt occurs for any service corresponding to the bit(s) set in mask. */

int vc_interface_register_event_int(void *event, int mask);

/* Handle (any) interrupt from VideoCore (must be called at task level) */

int vc_interface_interrupt_handler(void);

/* Return non-zero if we are polling rather than interrupt driven. */

int vc_interface_polling(void);

/* Set us into pollling (pass 1) or interrupt driven mode (pass 0). */

void vc_interface_set_polling(int onoff);

/* Query the application that is running on VideoCore. The int is actually a VC_INTERFACE_APP_T. */

int vc_host_get_app(void);

#endif
