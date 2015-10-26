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
Module   :  OpenMAX IL Component Service (host-side)
File     :  $RCSfile: vcilcs.h,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
OpenMAX IL Component Service API
=============================================================================*/

#ifndef VCILCS_H
#define VCILCS_H

#include "vchost_config.h"
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/connection.h>

/* Initialise IL component service. Returns it's interface number. This initialises
   the host side of the interface, it does not send anything to VideoCore. */
VCHPRE_ void vc_vchi_ilcs_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections);
VCHPRE_ VCHI_SERVICE_HANDLE_T vc_ilcs_vchi_handle( void );
VCHPRE_ void vc_ilcs_obtain_component_lock( void );
VCHPRE_ void vc_ilcs_release_component_lock( void );

VCHPRE_ int VCHPOST_ vc_ilcs_init(void);

/* Stop the service from being used. */
VCHPRE_ void VCHPOST_ vc_ilcs_stop(void);

/* Return the service number (-1 if not running). */
VCHPRE_ int VCHPOST_ vc_ilcs_inum(void);

/* Recieve and handle incoming messages from VideoCore */
VCHPRE_ int VCHPOST_ vc_ilcs_message_handler(void);

/* Return the read event */
VCHPRE_ void * VCHPOST_ vc_ilcs_read_event(void);

/* Return the component lock */
VCHPRE_ void * VCHPOST_ vc_ilcs_component_lock(void);

/* Implemented by the platform layer
 * Returns non-zero if the current task is the host ilcs task */
VCHPRE_ int VCHPOST_ vc_identify_vcilcs_task(void);

// ilcs service init
int vcil_iface_init(void);

#endif
