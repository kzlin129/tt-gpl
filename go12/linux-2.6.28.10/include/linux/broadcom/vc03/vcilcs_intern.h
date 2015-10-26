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
File     :  $RCSfile: vcilcs_intern.h,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
OpenMAX IL Component Service API
Internal functions for the host side IL component service
=============================================================================*/

#ifndef VCILCS_INTERN_H
#define VCILCS_INTERN_H

#include <linux/broadcom/vc03/vc_ilcs_defs.h>
#include <linux/broadcom/vc03/vchost_config.h>
#include <linux/broadcom/vc.h>
#include <linux/broadcom/omx/OMX_Component.h>

// used by host functions that implement the IL component API
// to execute a marshalled function on a VideoCore component
VCHPRE_ void VCHPOST_ vc_ilcs_execute_function(IL_FUNCTION_T func, void *data, int len, void *data2, int len2, void *resp, int resplen);
VCHPRE_ OMX_ERRORTYPE VCHPOST_ vc_ilcs_pass_buffer(IL_FUNCTION_T func, void *reference, OMX_BUFFERHEADERTYPE *pBuffer);
VCHPRE_ OMX_BUFFERHEADERTYPE * VCHPOST_ vc_ilcs_receive_buffer(void *call, int clen, OMX_COMPONENTTYPE **pComp);

// functions that implement incoming functions calls
// from VideoCore components to host based components
VCHPRE_ void VCHPOST_ vcil_in_get_state(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_get_parameter(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_set_parameter(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_get_config(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_set_config(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_use_buffer(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_free_buffer(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_empty_this_buffer(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_fill_this_buffer(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_get_component_version(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_get_extension_index(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_in_component_role_enum(void *call, int clen, void *resp, int *rlen);

// functions that implement callbacks from VideoCore
// components to the host core.
// The prefix is vcil_out since they implement part
// of the API that the host uses out to VideoCore
VCHPRE_ void VCHPOST_ vcil_out_event_handler(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_out_empty_buffer_done(void *call, int clen, void *resp, int *rlen);
VCHPRE_ void VCHPOST_ vcil_out_fill_buffer_done(void *call, int clen, void *resp, int *rlen);


int  vc_ilcs_componentlock_change(int val);
int  vc_ilcs_execute_function_usr(vcilcs_func_out_t* arg);
int  vc_ilcsinb_getusr(vc_ilcsinb_t* msgptr);

#endif // VCILCS_INTERN_H
