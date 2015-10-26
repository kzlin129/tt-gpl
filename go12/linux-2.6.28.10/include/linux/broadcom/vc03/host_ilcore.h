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


Project  :  OpenMAX IL
Module   :
File     :  $RCSfile: host_ilcore.h,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
OpenMAX IL - Host Core implementation
=============================================================================*/

#ifndef HOST_ILCORE_H
#define HOST_ILCORE_H

OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_Init(void);
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_Deinit(void);
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_ComponentNameEnum(
   OMX_OUT OMX_STRING cComponentName,
   OMX_IN  OMX_U32 nNameLength,
   OMX_IN  OMX_U32 nIndex);
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_GetHandle(
   OMX_OUT OMX_HANDLETYPE* pHandle,
   OMX_IN  OMX_STRING cComponentName,
   OMX_IN  OMX_PTR pAppData,
   OMX_IN  OMX_CALLBACKTYPE* pCallBacks);
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_FreeHandle(
   OMX_IN  OMX_HANDLETYPE hComponent);
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_SetupTunnel(
   OMX_IN  OMX_HANDLETYPE hOutput,
   OMX_IN  OMX_U32 nPortOutput,
   OMX_IN  OMX_HANDLETYPE hInput,
   OMX_IN  OMX_U32 nPortInput);
OMX_API OMX_ERRORTYPE host_OMX_GetComponentsOfRole (
   OMX_IN      OMX_STRING role,
   OMX_INOUT   OMX_U32 *pNumComps,
   OMX_INOUT   OMX_U8  **compNames);
OMX_API OMX_ERRORTYPE host_OMX_GetRolesOfComponent (
   OMX_IN      OMX_STRING compName,
   OMX_INOUT   OMX_U32 *pNumRoles,
   OMX_OUT     OMX_U8 **roles);

#define OMX_Init host_OMX_Init
#define OMX_Deinit host_OMX_Deinit
#define OMX_ComponentNameEnum host_OMX_ComponentNameEnum
#define OMX_GetHandle host_OMX_GetHandle
#define OMX_FreeHandle host_OMX_FreeHandle
#define OMX_SetupTunnel host_OMX_SetupTunnel
#define OMX_GetComponentsOfRole host_OMX_GetComponentsOfRole
#define OMX_GetRolesOfComponent host_OMX_GetRolesOfComponent

#endif // HOST_ILCORE_H

