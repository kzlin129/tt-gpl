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
File     :  $RCSfile: vcilcs_out.c,v $
Revision :  $Revision: 1.1.2.2 $

FILE DESCRIPTION
OpenMAX IL Component service API
Host functions that implements the IL component API
for outgoing function calls to VideoCore components
=============================================================================*/

#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vcmsgfifo.h>
#include <linux/broadcom/vc03/vc_ilcs_defs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vcilclient.h>

#if defined (__KERNEL__)
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vcilcs.h>
#endif

typedef struct {
   OMX_U32 port;
   IL_FUNCTION_T func;
   OMX_BOOL bEGL;
   OMX_U32 numBuffers;
   OMX_DIRTYPE dir;
} VC_PRIVATE_PORT_T;

struct _VC_PRIVATE_COMPONENT_T {
   OMX_COMPONENTTYPE *comp;
   void *reference;
   OMX_U32 numPorts;
   OMX_CALLBACKTYPE callbacks;
   OMX_PTR callback_state;
   VC_PRIVATE_PORT_T *port;
   struct _VC_PRIVATE_COMPONENT_T *next;
};
typedef struct _VC_PRIVATE_COMPONENT_T  VC_PRIVATE_COMPONENT_T;

static VC_PRIVATE_COMPONENT_T *component_list = NULL;

#if defined( __KERNEL__ )

#else

#define vc_ilcs_component_lock() NULL
#define vc_ilcs_obtain_component_lock()  libomx_vc_component_lock_obtain()
#define vc_ilcs_release_component_lock() libomx_vc_component_lock_release()

#endif

#define ilclient_debug_output OMX_DEBUG


OMX_ERRORTYPE vcil_out_component_name_enum(OMX_STRING cComponentName, OMX_U32 nNameLength, OMX_U32 nIndex);
OMX_ERRORTYPE vcil_out_create_component(OMX_HANDLETYPE hComponent, OMX_STRING component_name);


static VC_PRIVATE_PORT_T *find_port(VC_PRIVATE_COMPONENT_T *st, OMX_U32 nPortIndex)
{
   OMX_U32 i=0;
   while (i<st->numPorts && st->port[i].port != nPortIndex)
      i++;

   if (i < st->numPorts)
      return &st->port[i];

   return NULL;
}

static OMX_ERRORTYPE vcil_out_ComponentDeInit(OMX_IN OMX_HANDLETYPE hComponent)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_EXECUTE_HEADER_T exe;
   IL_RESPONSE_HEADER_T resp;

   exe.reference = st->reference;

   vc_ilcs_execute_function(IL_COMPONENT_DEINIT, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   // remove from list, assuming that we successfully managed to deinit
   // this component.  The assumption here is that if the component
   // signalled an error, it still exists.
   if (resp.err == OMX_ErrorNone)
   {
      VC_PRIVATE_COMPONENT_T *list, *prev;
      vc_ilcs_obtain_component_lock();

      list = component_list;
      prev = NULL;

      while (list != NULL && list != st)
      {
         prev = list;
         list = list->next;
      }

      // failing to find this component is not a good sign.
      vc_assert(list);
      if (list)
      {
         if (prev == NULL)
            component_list = list->next;
         else
            prev->next = list->next;
      }

      vc_ilcs_release_component_lock();
      free(st);
   }

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_GetComponentVersion(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_STRING pComponentName,
      OMX_OUT OMX_VERSIONTYPE* pComponentVersion,
      OMX_OUT OMX_VERSIONTYPE* pSpecVersion,
      OMX_OUT OMX_UUIDTYPE* pComponentUUID)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_EXECUTE_HEADER_T exe;
   IL_GET_VERSION_RESPONSE_T resp;

   exe.reference = st->reference;

   vc_ilcs_execute_function(IL_GET_COMPONENT_VERSION, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   strncpy(pComponentName, resp.name, 128);
   *pComponentVersion = resp.component_version;
   *pSpecVersion = resp.spec_version;
   memcpy(pComponentUUID, resp.uuid, sizeof(OMX_UUIDTYPE));

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_SetCallbacks(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_CALLBACKTYPE* pCallbacks,
      OMX_IN  OMX_PTR pAppData)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_SET_CALLBACKS_EXECUTE_T exe;
   IL_RESPONSE_HEADER_T resp;

   st->callbacks = *pCallbacks;
   st->callback_state = pAppData;

   exe.reference = st->reference;
   exe.pAppData = pComp;

   vc_ilcs_execute_function(IL_SET_CALLBACKS, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_GetState(OMX_IN  OMX_HANDLETYPE hComponent,
                                       OMX_OUT OMX_STATETYPE* pState)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_EXECUTE_HEADER_T exe;
   IL_GET_STATE_RESPONSE_T resp;

   exe.reference = st->reference;

   vc_ilcs_execute_function(IL_GET_STATE, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   *pState = resp.state;

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_get(OMX_IN  OMX_HANDLETYPE hComponent,
                                  OMX_IN  OMX_INDEXTYPE nParamIndex,
                                  OMX_INOUT OMX_PTR pComponentParameterStructure,
                                  IL_FUNCTION_T func)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_GET_EXECUTE_T exe;
   IL_GET_RESPONSE_T resp;
   OMX_U32 size;

   if (pComponentParameterStructure == NULL)
      return OMX_ErrorBadParameter;

   exe.reference = st->reference;
   exe.index = nParamIndex;

   size = *((OMX_U32 *) pComponentParameterStructure);
   vc_assert(size <= VC_ILCS_MAX_PARAM_SIZE);
   memcpy(exe.param, pComponentParameterStructure, size);

   vc_ilcs_execute_function(func, &exe, size + IL_GET_EXECUTE_HEADER_SIZE, NULL, 0, &resp, sizeof(resp));

   memcpy(pComponentParameterStructure, resp.param, size);

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_set(OMX_IN  OMX_HANDLETYPE hComponent,
                                  OMX_IN  OMX_INDEXTYPE nParamIndex,
                                  OMX_IN OMX_PTR pComponentParameterStructure,
                                  IL_FUNCTION_T func)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_SET_EXECUTE_T exe;
   IL_RESPONSE_HEADER_T resp;
   OMX_U32 size;

   if (pComponentParameterStructure == NULL)
      return OMX_ErrorBadParameter;

   exe.reference = st->reference;
   exe.index = nParamIndex;

   size = *((OMX_U32 *) pComponentParameterStructure);
   vc_assert(size <= VC_ILCS_MAX_PARAM_SIZE);
   memcpy(exe.param, pComponentParameterStructure, size);

   vc_ilcs_execute_function(func, &exe, size + IL_SET_EXECUTE_HEADER_SIZE, NULL, 0, &resp, sizeof(resp));

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_GetParameter(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_INDEXTYPE nParamIndex,
      OMX_INOUT OMX_PTR pComponentParameterStructure)
{
   return vcil_out_get(hComponent, nParamIndex, pComponentParameterStructure, IL_GET_PARAMETER);
}

static OMX_ERRORTYPE vcil_out_SetParameter(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_INDEXTYPE nParamIndex,
      OMX_IN OMX_PTR pComponentParameterStructure)
{
   return vcil_out_set(hComponent, nParamIndex, pComponentParameterStructure, IL_SET_PARAMETER);
}

static OMX_ERRORTYPE vcil_out_GetConfig(OMX_IN  OMX_HANDLETYPE hComponent,
                                        OMX_IN  OMX_INDEXTYPE nParamIndex,
                                        OMX_INOUT OMX_PTR pComponentParameterStructure)
{
   return vcil_out_get(hComponent, nParamIndex, pComponentParameterStructure, IL_GET_CONFIG);
}

static OMX_ERRORTYPE vcil_out_SetConfig(OMX_IN  OMX_HANDLETYPE hComponent,
                                        OMX_IN  OMX_INDEXTYPE nParamIndex,
                                        OMX_IN OMX_PTR pComponentParameterStructure)
{
   return vcil_out_set(hComponent, nParamIndex, pComponentParameterStructure, IL_SET_CONFIG);
}

static OMX_ERRORTYPE vcil_out_SendCommand(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_COMMANDTYPE Cmd,
      OMX_IN  OMX_U32 nParam1,
      OMX_IN  OMX_PTR pCmdData)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_SEND_COMMAND_EXECUTE_T exe;
   IL_RESPONSE_HEADER_T resp;

   exe.reference = st->reference;
   exe.cmd = Cmd;
   exe.param = nParam1;

   if (Cmd == OMX_CommandMarkBuffer)
   {
      exe.mark = *((OMX_MARKTYPE *) pCmdData);
   }
   else
   {
      exe.mark.hMarkTargetComponent = 0;
      exe.mark.pMarkData = 0;
   }

   vc_ilcs_execute_function(IL_SEND_COMMAND, &exe, sizeof(IL_SEND_COMMAND_EXECUTE_T), NULL, 0, &resp, sizeof(resp));

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_addBuffer(OMX_IN OMX_HANDLETYPE hComponent,
                                        OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
                                        OMX_IN OMX_U32 nPortIndex,
                                        OMX_IN OMX_PTR pAppPrivate,
                                        OMX_IN OMX_U32 nSizeBytes,
                                        OMX_IN OMX_U8* pBuffer,
                                        OMX_IN void *eglImage,
                                        IL_FUNCTION_T func)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_ADD_BUFFER_EXECUTE_T exe;
   IL_ADD_BUFFER_RESPONSE_T resp;
   OMX_BUFFERHEADERTYPE *pHeader;
   VC_PRIVATE_PORT_T *port = find_port(st, nPortIndex);

   if (!port) // bad port index
      return OMX_ErrorBadPortIndex;

   if (port->numBuffers > 0 && port->func != func)
   {
      // inconsistent use of usebuffer/allocatebuffer/eglimage
      // all ports must receive all buffers by exactly one of these methods
      vc_assert(port->func != func);
      return OMX_ErrorInsufficientResources;
   }
   port->func = func;

   if (!VCHI_BULK_ALIGNED(pBuffer))
   {
      // cannot transfer this buffer across the host interface
      return OMX_ErrorBadParameter;
   }

   pHeader = os_malloc(sizeof(*pHeader), VCHI_BULK_ALIGN, "vcout buffer header");

   if (!pHeader)
      return OMX_ErrorInsufficientResources;

   if (func == IL_ALLOCATE_BUFFER)
   {
      pBuffer = os_malloc(nSizeBytes, VCHI_BULK_ALIGN, "vcout mapping buffer");
      if (!pBuffer)
      {
         free(pHeader);
         return OMX_ErrorInsufficientResources;
      }
   }

   exe.reference = st->reference;
   exe.bufferReference = pHeader;
   exe.port = nPortIndex;
   exe.size = nSizeBytes;
   exe.eglImage = eglImage;

   vc_ilcs_execute_function(func, &exe, sizeof(IL_ADD_BUFFER_EXECUTE_T), NULL, 0, &resp, sizeof(resp));

   if (resp.err == OMX_ErrorNone)
   {
      memcpy(pHeader, &resp.bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));
      if (port->dir == OMX_DirOutput)
         pHeader->pOutputPortPrivate = resp.reference;
      else
         pHeader->pInputPortPrivate = resp.reference;

      if (func == IL_USE_EGL_IMAGE)
      {
         pHeader->pBuffer = (OMX_U8*)eglImage;
         port->bEGL = OMX_TRUE;
      }         
      else
      {
      pHeader->pBuffer = pBuffer;
         port->bEGL = OMX_FALSE;
      }
                   
      pHeader->pAppPrivate = pAppPrivate;
      *ppBufferHdr = pHeader;
      port->numBuffers++;
   }
   else
   {
      if (func == IL_ALLOCATE_BUFFER)
         free(pBuffer);
      free(pHeader);
   }

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_UseEGLImage(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
      OMX_IN OMX_U32 nPortIndex,
      OMX_IN OMX_PTR pAppPrivate,
      OMX_IN void* eglImage)
{
   return vcil_out_addBuffer(hComponent, ppBufferHdr, nPortIndex, pAppPrivate, 0, NULL, eglImage, IL_USE_EGL_IMAGE);
}

static OMX_ERRORTYPE vcil_out_UseBuffer(OMX_IN OMX_HANDLETYPE hComponent,
                                        OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
                                        OMX_IN OMX_U32 nPortIndex,
                                        OMX_IN OMX_PTR pAppPrivate,
                                        OMX_IN OMX_U32 nSizeBytes,
                                        OMX_IN OMX_U8* pBuffer)
{
   return vcil_out_addBuffer(hComponent, ppBufferHdr, nPortIndex, pAppPrivate, nSizeBytes, pBuffer, NULL, IL_USE_BUFFER);
}

static OMX_ERRORTYPE vcil_out_AllocateBuffer(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
      OMX_IN OMX_U32 nPortIndex,
      OMX_IN OMX_PTR pAppPrivate,
      OMX_IN OMX_U32 nSizeBytes)
{
   return vcil_out_addBuffer(hComponent, ppBufferHdr, nPortIndex, pAppPrivate, nSizeBytes, NULL, NULL, IL_ALLOCATE_BUFFER);
}

static OMX_ERRORTYPE vcil_out_FreeBuffer(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_U32 nPortIndex,
      OMX_IN  OMX_BUFFERHEADERTYPE* pBufferHdr)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_FREE_BUFFER_EXECUTE_T exe;
   IL_RESPONSE_HEADER_T resp;
   VC_PRIVATE_PORT_T *port = find_port(st, nPortIndex);

   if (!port)
      return OMX_ErrorBadPortIndex;

   if (port->numBuffers == 0)
      return OMX_ErrorIncorrectStateTransition;

   exe.reference = st->reference;
   exe.port = nPortIndex;
   if (port->dir == OMX_DirOutput)
      exe.bufferReference = pBufferHdr->pOutputPortPrivate;
   else
      exe.bufferReference = pBufferHdr->pInputPortPrivate;
   exe.func = port->func;
   exe.inputPrivate = NULL;
   exe.outputPrivate = NULL;

   vc_ilcs_execute_function(IL_FREE_BUFFER, &exe, sizeof(IL_FREE_BUFFER_EXECUTE_T), NULL, 0, &resp, sizeof(resp));

   if (resp.err == OMX_ErrorNone)
   {
      if (port->func == IL_ALLOCATE_BUFFER)
         free(pBufferHdr->pBuffer);
      free(pBufferHdr);
      port->numBuffers--;
   }

   return resp.err;
}

static OMX_ERRORTYPE vcil_out_EmptyThisBuffer(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;

#if !defined(__KERNEL__)
   IL_PASS_BUFFER_EXECUTE_T exe;
   IL_RESPONSE_HEADER_T resp;
   void *data2 = NULL;
   int len2 = 0;

   if (pBuffer->pInputPortPrivate == NULL )
   {
      // return this to pass conformance
      // the actual error is using a buffer that hasn't be registered with usebuffer/allocatebuffer
      return OMX_ErrorIncorrectStateOperation;
   }
   
   exe.reference = st->reference;
   memcpy(&exe.bufferHeader, pBuffer, sizeof(OMX_BUFFERHEADERTYPE));

   vc_ilcs_execute_function(IL_EMPTY_THIS_BUFFER, &exe, sizeof(IL_PASS_BUFFER_EXECUTE_T), data2, len2, &resp, sizeof(resp));
   return resp.err; 
   
#else

   return vc_ilcs_pass_buffer(IL_EMPTY_THIS_BUFFER, st->reference, pBuffer);
#endif
//#endif
}


static OMX_ERRORTYPE vcil_out_FillThisBuffer(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer)
{
#if !defined(__KERNEL__)
     // FIXME, support usermode
     (void) hComponent;
     (void) pBuffer;
     OMX_ERRORTYPE err = OMX_ErrorNoMore;
#else
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   VC_PRIVATE_PORT_T *port;

   // The lower layers will attempt to transfer the bytes specified if we don't
   // clear these - callers should ideally do this themselves, but it is not
   // mandated in the specification.
   pBuffer->nFilledLen = 0;
   pBuffer->nFlags = 0;

   OMX_ERRORTYPE err = vc_ilcs_pass_buffer(IL_FILL_THIS_BUFFER, st->reference, pBuffer);

   if (err == OMX_ErrorNone && (port = find_port(st, pBuffer->nOutputPortIndex)) != NULL &&
       port->bEGL)
{
      // If an output port is marked as an EGL port, we request EGL to notify the IL component 
      // when it's allowed to render into the buffer/EGLImage.
      //eglIntOpenMAXILDoneMarker(st->reference, (EGLImageKHR)pBuffer->pBuffer);
}
#endif
   return err;
}

static OMX_ERRORTYPE vcil_out_ComponentTunnelRequest(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_U32 nPort,
      OMX_IN  OMX_HANDLETYPE hTunneledComp,
      OMX_IN  OMX_U32 nTunneledPort,
      OMX_INOUT  OMX_TUNNELSETUPTYPE* pTunnelSetup)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_TUNNEL_REQUEST_EXECUTE_T exe;
   IL_TUNNEL_REQUEST_RESPONSE_T resp;
   VC_PRIVATE_COMPONENT_T *list;

   exe.reference = st->reference;
   exe.port = nPort;
   exe.tunnel_port = nTunneledPort;
   if (pTunnelSetup)
      exe.setup = *pTunnelSetup;

   // the other component may be on the host or on VC.  Look through our list
   // so we can tell, and tell ILCS on VC the details.
   vc_ilcs_obtain_component_lock();
   list = component_list;
   while (list != NULL && list->comp != (void *) hTunneledComp)
      list = list->next;
   vc_ilcs_release_component_lock();

   if (list == NULL)
   {
      exe.tunnel_ref = hTunneledComp;
      exe.tunnel_host = OMX_TRUE;
   }
   else
   {
      exe.tunnel_ref = list->reference;
      exe.tunnel_host = OMX_FALSE;
   }

   vc_ilcs_execute_function(IL_COMPONENT_TUNNEL_REQUEST, &exe, sizeof(IL_TUNNEL_REQUEST_EXECUTE_T), NULL, 0, &resp, sizeof(resp));

   if (pTunnelSetup)
      *pTunnelSetup = resp.setup;
   return resp.err;
}

static OMX_ERRORTYPE vcil_out_GetExtensionIndex(OMX_IN  OMX_HANDLETYPE hComponent,
      OMX_IN  OMX_STRING cParameterName,
      OMX_OUT OMX_INDEXTYPE* pIndexType)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_GET_EXTENSION_EXECUTE_T exe;
   IL_GET_EXTENSION_RESPONSE_T resp;

   exe.reference = st->reference;
   strncpy(exe.name, cParameterName, 128);

   vc_ilcs_execute_function(IL_GET_EXTENSION_INDEX, &exe, sizeof(IL_GET_EXTENSION_EXECUTE_T), NULL, 0, &resp, sizeof(resp));

   *pIndexType = resp.index;
   return resp.err;
}

static OMX_ERRORTYPE vcil_out_ComponentRoleEnum(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_U8 *cRole,
      OMX_IN OMX_U32 nIndex)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   IL_COMPONENT_ROLE_ENUM_EXECUTE_T exe;
   IL_COMPONENT_ROLE_ENUM_RESPONSE_T resp;

   exe.reference = st->reference;
   exe.index = nIndex;

   vc_ilcs_execute_function(IL_COMPONENT_ROLE_ENUM, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   strncpy((char *) cRole, (char *) resp.role, 128);
   return resp.err;
}

OMX_ERRORTYPE vcil_out_component_name_enum(OMX_STRING cComponentName, OMX_U32 nNameLength, OMX_U32 nIndex)
{
   IL_COMPONENT_NAME_ENUM_EXECUTE_T exe;
   IL_COMPONENT_NAME_ENUM_RESPONSE_T resp;

   exe.index = nIndex;

   vc_ilcs_execute_function(IL_COMPONENT_NAME_ENUM, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   if (sizeof(resp.name) < nNameLength)
      nNameLength = sizeof(resp.name);

   strncpy((char *)cComponentName, (char *) resp.name, nNameLength);
   return resp.err;
}

OMX_ERRORTYPE vcil_out_create_component(OMX_HANDLETYPE hComponent, OMX_STRING component_name)
{
   OMX_COMPONENTTYPE *pComp = (OMX_COMPONENTTYPE *) hComponent;
   IL_CREATE_COMPONENT_EXECUTE_T exe;
   IL_CREATE_COMPONENT_RESPONSE_T resp;
   VC_PRIVATE_COMPONENT_T *st;
   OMX_U32 i;

   if (strlen(component_name) >= sizeof(exe.name))
      return OMX_ErrorInvalidComponent;

   strcpy(exe.name, component_name);
   exe.mark = pComp;

   vc_ilcs_execute_function(IL_CREATE_COMPONENT, &exe, sizeof(exe), NULL, 0, &resp, sizeof(resp));

   if (resp.err != OMX_ErrorNone)
      return resp.err;

   st = malloc(sizeof(VC_PRIVATE_COMPONENT_T) + (sizeof(VC_PRIVATE_PORT_T) * resp.numPorts));
   if (!st)
   {
      IL_EXECUTE_HEADER_T dexe;
      IL_RESPONSE_HEADER_T dresp;

      dexe.reference = resp.reference;

      vc_ilcs_execute_function(IL_COMPONENT_DEINIT, &dexe, sizeof(dexe), NULL, 0, &dresp, sizeof(resp));
      return OMX_ErrorInsufficientResources;
   }

   memset(st, 0, sizeof(VC_PRIVATE_COMPONENT_T) + (sizeof(VC_PRIVATE_PORT_T) * resp.numPorts));

   st->reference = resp.reference;
   st->comp = pComp;
   st->numPorts = resp.numPorts;
   st->port = (VC_PRIVATE_PORT_T *) ((unsigned char *) st + sizeof(VC_PRIVATE_COMPONENT_T));

   for (i=0; i<st->numPorts; i++)
   {
      if (i && !(i&0x1f))
      {
         IL_GET_EXECUTE_T gexe;
         IL_GET_RESPONSE_T gresp;
         OMX_PARAM_PORTSUMMARYTYPE *summary;

         gexe.reference = st->reference;
         gexe.index = OMX_IndexParamPortSummary;

         summary = (OMX_PARAM_PORTSUMMARYTYPE *) &gexe.param;
         summary->nSize = sizeof(OMX_PARAM_PORTSUMMARYTYPE);
         summary->nVersion.nVersion = OMX_VERSION;
         summary->reqSet = i>>5;

         vc_ilcs_execute_function(IL_GET_PARAMETER, &gexe,
                                  sizeof(OMX_PARAM_PORTSUMMARYTYPE)+IL_GET_EXECUTE_HEADER_SIZE,
                                  NULL, 0,
                                  &gresp, sizeof(resp));

         summary = (OMX_PARAM_PORTSUMMARYTYPE *) &gresp.param;
         resp.portDir = summary->portDir;
         memcpy(resp.portIndex, summary->portIndex, sizeof(OMX_U32) * 32);
      }

      st->port[i].port = resp.portIndex[i&0x1f];
      st->port[i].dir = ((resp.portDir >> (i&0x1f)) & 1) ? OMX_DirOutput : OMX_DirInput;
   }

   vc_ilcs_obtain_component_lock();
   // insert into head of list
   st->next = component_list;
   component_list = st;
   vc_ilcs_release_component_lock();

   pComp->pComponentPrivate = st;

   pComp->GetComponentVersion = vcil_out_GetComponentVersion;
   pComp->ComponentDeInit = vcil_out_ComponentDeInit;
   pComp->SetCallbacks = vcil_out_SetCallbacks;
   pComp->GetState = vcil_out_GetState;
   pComp->GetParameter = vcil_out_GetParameter;
   pComp->SetParameter = vcil_out_SetParameter;
   pComp->GetConfig = vcil_out_GetConfig;
   pComp->SetConfig = vcil_out_SetConfig;
   pComp->SendCommand = vcil_out_SendCommand;
   pComp->UseBuffer = vcil_out_UseBuffer;
   pComp->AllocateBuffer = vcil_out_AllocateBuffer;
   pComp->FreeBuffer = vcil_out_FreeBuffer;
   pComp->EmptyThisBuffer = vcil_out_EmptyThisBuffer;
   pComp->FillThisBuffer = vcil_out_FillThisBuffer;
   pComp->ComponentTunnelRequest = vcil_out_ComponentTunnelRequest;
   pComp->GetExtensionIndex = vcil_out_GetExtensionIndex;
   pComp->UseEGLImage = vcil_out_UseEGLImage;
   pComp->ComponentRoleEnum = vcil_out_ComponentRoleEnum;

   return resp.err;
}

/* callbacks */

void vcil_out_event_handler(void *call, int clen, void *resp, int *rlen)
{
   IL_EVENT_HANDLER_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   
   *rlen = 0;
   (void)ret;
   (void)clen;

   vc_assert(st->callbacks.EventHandler);
   st->callbacks.EventHandler(pComp, st->callback_state, exe->event, exe->data1, exe->data2, exe->eventdata);
}

void vcil_out_empty_buffer_done(void *call, int clen, void *resp, int *rlen)
{
   IL_PASS_BUFFER_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   VC_PRIVATE_COMPONENT_T *st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;
   OMX_BUFFERHEADERTYPE *pHeader = exe->bufferHeader.pOutputPortPrivate;
   OMX_U8 *pBuffer = pHeader->pBuffer;
   OMX_PTR *pAppPrivate = pHeader->pAppPrivate;
   OMX_PTR *pPlatformPrivate = pHeader->pPlatformPrivate;
   OMX_PTR *pInputPortPrivate = pHeader->pInputPortPrivate;
   OMX_PTR *pOutputPortPrivate = pHeader->pOutputPortPrivate;

   memcpy(pHeader, &exe->bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));

   pHeader->pBuffer = pBuffer;
   pHeader->pAppPrivate = pAppPrivate;
   pHeader->pPlatformPrivate = pPlatformPrivate;
   pHeader->pInputPortPrivate = pInputPortPrivate;
   pHeader->pOutputPortPrivate = pOutputPortPrivate;

   *rlen = 0;
   (void)ret;
   (void)clen;

   vc_assert(st->callbacks.EmptyBufferDone);
   st->callbacks.EmptyBufferDone(pComp, st->callback_state, pHeader);
}

void vcil_out_fill_buffer_done(void *call, int clen, void *resp, int *rlen)
{
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp;
   VC_PRIVATE_COMPONENT_T *st;
   OMX_BUFFERHEADERTYPE *pHeader;

#if !defined(__KERNEL__)
     // FIXME, support usermode
   (void) call;
   (void) ret;
   (void) clen;
   (void) rlen;
#else
   pHeader = vc_ilcs_receive_buffer(call, clen, &pComp);

   st = (VC_PRIVATE_COMPONENT_T *) pComp->pComponentPrivate;

   *rlen = 0;
#endif

   vc_assert(st->callbacks.FillBufferDone);
   st->callbacks.FillBufferDone(pComp, st->callback_state, pHeader);
}
