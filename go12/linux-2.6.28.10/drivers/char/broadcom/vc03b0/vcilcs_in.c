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
File     :  $RCSfile: vcilcs_in.c,v $
Revision :  $Revision: 1.1.2.2 $

FILE DESCRIPTION
OpenMAX IL Component service API
Host functions that implements the incoming function
calls from VideoCore components to host based components
=============================================================================*/

#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vchostmem.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vcmsgfifo.h>

#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/vc_ilcs_defs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>

#if defined (__KERNEL__)
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vcilcs.h>

#else

#include <linux/broadcom/omx/omx.h>
#define malloc libomx_malloc  
#define free     libomx_free

#endif

void vcil_in_get_state(void *call, int clen, void *resp, int *rlen)
{
   IL_EXECUTE_HEADER_T *exe = call;
   IL_GET_STATE_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   
   (void)clen;

   *rlen = sizeof(IL_GET_STATE_RESPONSE_T);
   ret->func = IL_GET_STATE;
   ret->err = pComp->GetState(pComp, &ret->state);
}

void vcil_in_get_parameter(void *call, int clen, void *resp, int *rlen)
{
   IL_GET_EXECUTE_T *exe = call;
   IL_GET_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp  = exe->reference;
   OMX_U32 size = *((OMX_U32 *) (&exe->param));

   (void)clen;

   vc_assert(size <= VC_ILCS_MAX_PARAM_SIZE);
   *rlen = (size + IL_GET_RESPONSE_HEADER_SIZE + 15) & ~15;
   ret->func = IL_GET_PARAMETER;
   ret->err = pComp->GetParameter(pComp, exe->index, exe->param);
   memcpy(ret->param, exe->param, size);
}

void vcil_in_set_parameter(void *call, int clen, void *resp, int *rlen)
{
   IL_SET_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp  = exe->reference;

   (void)clen;

   *rlen = sizeof(IL_RESPONSE_HEADER_T);
   ret->func = IL_SET_PARAMETER;
   ret->err = pComp->SetParameter(pComp, exe->index, exe->param);
}

void vcil_in_get_config(void *call, int clen, void *resp, int *rlen)
{
   IL_GET_EXECUTE_T *exe = call;
   IL_GET_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp  = exe->reference;
   OMX_U32 size = *((OMX_U32 *) (&exe->param));

   (void)clen;

   vc_assert(size <= VC_ILCS_MAX_PARAM_SIZE);
   *rlen = (size + IL_GET_RESPONSE_HEADER_SIZE + 15) & ~15;
   ret->func = IL_GET_CONFIG;
   ret->err = pComp->GetConfig(pComp, exe->index, exe->param);
   memcpy(ret->param, exe->param, size);
}

void vcil_in_set_config(void *call, int clen, void *resp, int *rlen)
{
   IL_SET_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp  = exe->reference;

   (void)clen;

   *rlen = sizeof(IL_RESPONSE_HEADER_T);
   ret->func = IL_SET_CONFIG;
   ret->err = pComp->SetConfig(pComp, exe->index, exe->param);
}
void vcil_in_use_buffer(void *call, int clen, void *resp, int *rlen)
{
   IL_ADD_BUFFER_EXECUTE_T *exe = call;
   IL_ADD_BUFFER_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   OMX_U8 *buffer;
   OMX_BUFFERHEADERTYPE *bufferHeader;

   (void)clen;

   *rlen = sizeof(IL_ADD_BUFFER_RESPONSE_T);

   buffer = malloc(exe->size);

   if (!buffer)
   {
      ret->err = OMX_ErrorInsufficientResources;
      return;
   }

   //OMX_OSAL_Trace(OMX_OSAL_TRACE_COMPONENT, "hostcomp: use buffer(%p)\n", buffer);
   ret->func = IL_USE_BUFFER;
   ret->err = pComp->UseBuffer(pComp, &bufferHeader, exe->port, exe->bufferReference, exe->size, buffer);

   if (ret->err == OMX_ErrorNone)
   {
      // we're going to pass this buffer to VC
      // initialise our private field in their copy with the host buffer reference
      OMX_PARAM_PORTDEFINITIONTYPE def;
      OMX_ERRORTYPE error;
      def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      def.nVersion.nVersion = OMX_VERSION;
      def.nPortIndex = exe->port;
      error = pComp->GetParameter(pComp, OMX_IndexParamPortDefinition, &def);
      vc_assert(error == OMX_ErrorNone);

      ret->reference = bufferHeader;
      memcpy(&ret->bufferHeader, bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));

      if (def.eDir == OMX_DirInput)
         ret->bufferHeader.pInputPortPrivate = bufferHeader;
      else
         ret->bufferHeader.pOutputPortPrivate = bufferHeader;
   }
   else
      free(buffer);
}

void vcil_in_free_buffer(void *call, int clen, void *resp, int *rlen)
{
   IL_FREE_BUFFER_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   OMX_BUFFERHEADERTYPE *pHeader;
   OMX_U8 *buffer;
   OMX_PARAM_PORTDEFINITIONTYPE def;
   OMX_ERRORTYPE error;

   (void)clen;

   def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   def.nVersion.nVersion = OMX_VERSION;
   def.nPortIndex = exe->port;
   error = pComp->GetParameter(pComp, OMX_IndexParamPortDefinition, &def);
   vc_assert(error == OMX_ErrorNone);
   if (def.eDir == OMX_DirInput)
      pHeader = exe->inputPrivate;
   else
      pHeader = exe->outputPrivate;

   buffer = pHeader->pBuffer;

   *rlen = sizeof(IL_RESPONSE_HEADER_T);
   ret->func = IL_FREE_BUFFER;
   ret->err = pComp->FreeBuffer(pComp, exe->port, pHeader);
   if (ret->err == OMX_ErrorNone)
      free(buffer);
}

void vcil_in_empty_this_buffer(void *call, int clen, void *resp, int *rlen)
{
   IL_PASS_BUFFER_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   OMX_BUFFERHEADERTYPE *pHeader = exe->bufferHeader.pInputPortPrivate;
   OMX_U8 *pBuffer = pHeader->pBuffer;
   OMX_PTR *pAppPrivate = pHeader->pAppPrivate;
   OMX_PTR *pPlatformPrivate = pHeader->pPlatformPrivate;
   OMX_PTR *pInputPortPrivate = pHeader->pInputPortPrivate;
   OMX_PTR *pOutputPortPrivate = pHeader->pOutputPortPrivate;

   (void)clen;

   vc_assert(pHeader);
   memcpy(pHeader, &exe->bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));

   pHeader->pBuffer = pBuffer;
   pHeader->pAppPrivate = pAppPrivate;
   pHeader->pPlatformPrivate = pPlatformPrivate;
   pHeader->pInputPortPrivate = pInputPortPrivate;
   pHeader->pOutputPortPrivate = pOutputPortPrivate;

   // bulk transfer from videocore to host
#if !defined(KERNEL)
   vchostreq_writemem(pBuffer + pHeader->nOffset, exe->bufferHeader.pBuffer + pHeader->nOffset, pHeader->nFilledLen, -1);
#else
   // FIXME: can we do this cleaner?
   extern VCHI_SERVICE_HANDLE_T ilcs_vchi_handle( void );

   //host_vchi_msgfifo_wrapper_bulk_read( pBuffer + pHeader->nOffset, exe->bufferHeader.pBuffer + pHeader->nOffset, pHeader->nFilledLen );
   vchi_bulk_queue_receive( ilcs_vchi_handle(),
                            pBuffer + pHeader->nOffset,
                            pHeader->nFilledLen,
                            VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
                            NULL );
#endif

   *rlen = sizeof(IL_RESPONSE_HEADER_T);
   ret->func = IL_EMPTY_THIS_BUFFER;
   ret->err = pComp->EmptyThisBuffer(pComp, pHeader);
}
void vcil_in_fill_this_buffer(void *call, int clen, void *resp, int *rlen)
{
   IL_PASS_BUFFER_EXECUTE_T *exe = call;
   IL_RESPONSE_HEADER_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;
   OMX_BUFFERHEADERTYPE *pHeader = exe->bufferHeader.pOutputPortPrivate;
   OMX_U8 *pBuffer = pHeader->pBuffer;
   OMX_PTR *pAppPrivate = pHeader->pAppPrivate;
   OMX_PTR *pPlatformPrivate = pHeader->pPlatformPrivate;
   OMX_PTR *pInputPortPrivate = pHeader->pInputPortPrivate;
   OMX_PTR *pOutputPortPrivate = pHeader->pOutputPortPrivate;

   (void)clen;

   vc_assert(pHeader);
   memcpy(pHeader, &exe->bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));

   pHeader->pBuffer = pBuffer;
   pHeader->pAppPrivate = pAppPrivate;
   pHeader->pPlatformPrivate = pPlatformPrivate;
   pHeader->pInputPortPrivate = pInputPortPrivate;
   pHeader->pOutputPortPrivate = pOutputPortPrivate;

   *rlen = sizeof(IL_RESPONSE_HEADER_T);
   ret->func = IL_FILL_THIS_BUFFER;
   ret->err = pComp->FillThisBuffer(pComp, pHeader);
}

void vcil_in_get_component_version(void *call, int clen, void *resp, int *rlen)
{
   IL_EXECUTE_HEADER_T *exe = call;
   IL_GET_VERSION_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;

   (void)clen;

   *rlen = sizeof(IL_GET_VERSION_RESPONSE_T);
   ret->func = IL_GET_COMPONENT_VERSION;
   ret->err = pComp->GetComponentVersion(pComp, ret->name, &ret->component_version, &ret->spec_version, &ret->uuid);
}

void vcil_in_get_extension_index(void *call, int clen, void *resp, int *rlen)
{
   IL_GET_EXTENSION_EXECUTE_T *exe = call;
   IL_GET_EXTENSION_RESPONSE_T *ret = resp;
   OMX_COMPONENTTYPE *pComp = exe->reference;

   (void)clen;

   *rlen = sizeof(IL_GET_EXTENSION_RESPONSE_T);
   ret->func = IL_GET_EXTENSION_INDEX;
   ret->err = pComp->GetExtensionIndex(pComp, exe->name, &ret->index);
}
