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
File     :  $RCSfile: ilcore.c,v $
Revision :  $Revision: 1.1.2.2 $

FILE DESCRIPTION
OpenMAX IL - Host Core implementation
=============================================================================*/


#include <stdarg.h>

#if defined( __KERNEL__ )

#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <linux/broadcom/vc.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/jiffies.h>
#include <linux/broadcom/vc03/vchi/host_msgfifo_wrapper.h>

#define malloc vmalloc
#define free     vfree

#else

#include <stdio.h>

#endif

#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vcmsgfifo.h>
#include <linux/broadcom/vc03/vc_ilcs_defs.h>
//#include <linux/broadcom/vc03/vcilcs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcos.h>


static int coreInit = 0;
static int nActiveHandles = 0;

/* OMX_Init */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_Init(void)
{
   coreInit++;
   return OMX_ErrorNone;
}

/* OMX_Deinit */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_Deinit(void)
{
   if(coreInit == 0 || (coreInit == 1 && nActiveHandles > 0))
      return OMX_ErrorNotReady;
   coreInit--;
   return OMX_ErrorNone;
}

extern OMX_ERRORTYPE vcil_out_component_name_enum(OMX_STRING cComponentName, OMX_U32 nNameLength, OMX_U32 nIndex);

/* OMX_ComponentNameEnum */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_ComponentNameEnum(
   OMX_OUT OMX_STRING cComponentName,
   OMX_IN  OMX_U32 nNameLength,
   OMX_IN  OMX_U32 nIndex)
{
   return vcil_out_component_name_enum(cComponentName, nNameLength, nIndex);
}

extern OMX_ERRORTYPE vcil_out_create_component(OMX_HANDLETYPE hComponent, OMX_STRING component_name);

/* OMX_GetHandle */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_GetHandle(
   OMX_OUT OMX_HANDLETYPE* pHandle,
   OMX_IN  OMX_STRING cComponentName,
   OMX_IN  OMX_PTR pAppData,
   OMX_IN  OMX_CALLBACKTYPE* pCallBacks)
{
   OMX_ERRORTYPE eError;
   OMX_COMPONENTTYPE *pComp;
   OMX_HANDLETYPE hHandle = 0;

   if (pHandle == NULL || cComponentName == NULL || pCallBacks == NULL)
      return OMX_ErrorBadParameter;

   pComp = (OMX_COMPONENTTYPE *)malloc(sizeof(OMX_COMPONENTTYPE));
   hHandle = (OMX_HANDLETYPE)pComp;
   pComp->nVersion.s.nVersionMajor    = 1;
   pComp->nVersion.s.nVersionMinor    = 1;
   pComp->nVersion.s.nRevision        = 0;
   pComp->nVersion.s.nStep            = 0;
   pComp->nSize = sizeof(OMX_COMPONENTTYPE);
   eError = vcil_out_create_component(hHandle, cComponentName);

   if (eError == OMX_ErrorNone) {
     eError = (pComp->SetCallbacks)(hHandle,pCallBacks,pAppData);
     if (eError != OMX_ErrorNone)
       {
	 (pComp->ComponentDeInit)(hHandle);
       }
   }
   if (eError == OMX_ErrorNone) {
      nActiveHandles++;
      *pHandle = hHandle;
   }
   else {
      *pHandle = NULL;
      free(pComp);
   }
   return eError;
}

/* OMX_FreeHandle */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_FreeHandle(
   OMX_IN  OMX_HANDLETYPE hComponent)
{
   OMX_ERRORTYPE eError = OMX_ErrorNone;
   OMX_COMPONENTTYPE *pComp;

   if (hComponent == NULL)
      return OMX_ErrorBadParameter;

   pComp = (OMX_COMPONENTTYPE*)hComponent;
   eError = (pComp->ComponentDeInit)(hComponent);
   if (eError == OMX_ErrorNone) {
      --nActiveHandles;
      free(pComp);
   }

   vc_assert(nActiveHandles >= 0);

   return eError;
}

/* OMX_SetupTunnel */
OMX_API OMX_ERRORTYPE OMX_APIENTRY host_OMX_SetupTunnel(
   OMX_IN  OMX_HANDLETYPE hOutput,
   OMX_IN  OMX_U32 nPortOutput,
   OMX_IN  OMX_HANDLETYPE hInput,
   OMX_IN  OMX_U32 nPortInput)
{
   OMX_ERRORTYPE eError = OMX_ErrorNone;
   OMX_COMPONENTTYPE *pCompIn, *pCompOut;
   OMX_TUNNELSETUPTYPE oTunnelSetup;

   if (hOutput == NULL && hInput == NULL)
      return OMX_ErrorBadParameter;

   oTunnelSetup.nTunnelFlags = 0;
   oTunnelSetup.eSupplier = OMX_BufferSupplyUnspecified;

   pCompOut = (OMX_COMPONENTTYPE*)hOutput;

   if (hOutput){
      eError = pCompOut->ComponentTunnelRequest(hOutput, nPortOutput, hInput, nPortInput, &oTunnelSetup);
   }

   if (eError == OMX_ErrorNone && hInput) {
      pCompIn = (OMX_COMPONENTTYPE*)hInput;
      eError = pCompIn->ComponentTunnelRequest(hInput, nPortInput, hOutput, nPortOutput, &oTunnelSetup);

      if (eError != OMX_ErrorNone && hOutput) {
         /* cancel tunnel request on output port since input port failed */
         pCompOut->ComponentTunnelRequest(hOutput, nPortOutput, NULL, 0, NULL);
      }
   }
   return eError;
}

/* OMX_GetComponentsOfRole */
OMX_API OMX_ERRORTYPE host_OMX_GetComponentsOfRole (
   OMX_IN      OMX_STRING role,
   OMX_INOUT   OMX_U32 *pNumComps,
   OMX_INOUT   OMX_U8  **compNames)
{
   OMX_ERRORTYPE eError = OMX_ErrorNone;
   (void)role;
   (void)compNames;

   *pNumComps = 0;
   return eError;
}

/* OMX_GetRolesOfComponent */
OMX_API OMX_ERRORTYPE host_OMX_GetRolesOfComponent (
   OMX_IN      OMX_STRING compName,
   OMX_INOUT   OMX_U32 *pNumRoles,
   OMX_OUT     OMX_U8 **roles)
{
   OMX_ERRORTYPE eError = OMX_ErrorNone;
   (void)roles;
   (void)compName;

   *pNumRoles = 0;
   return eError;
}

#if defined( __KERNEL__ )

#include <linux/module.h>
   
EXPORT_SYMBOL (host_OMX_Init);
EXPORT_SYMBOL (host_OMX_Deinit);
EXPORT_SYMBOL (host_OMX_SetupTunnel);

#endif

/* File EOF */
