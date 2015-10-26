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
Project  :  VMCS-X
Module   :  OpenMAX IL Component Service
File     :  $RCSfile: vc_ilcs_defs.h,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
OpenMAX IL Component Service definitions
=============================================================================*/

#ifndef VC_ILCS_DEFS_H
#define VC_ILCS_DEFS_H

#define OMX_VERSION 0x10101

#include <linux/broadcom/omx/OMX_IVCommon.h>
#include <linux/broadcom/vc03/vchi/vchi.h>

typedef enum {
   IL_RESPONSE,
   IL_CREATE_COMPONENT,

   IL_GET_COMPONENT_VERSION,
   IL_SEND_COMMAND,
   IL_GET_PARAMETER,
   IL_SET_PARAMETER,
   IL_GET_CONFIG,
   IL_SET_CONFIG,
   IL_GET_EXTENSION_INDEX,
   IL_GET_STATE,
   IL_COMPONENT_TUNNEL_REQUEST,
   IL_USE_BUFFER,
   IL_USE_EGL_IMAGE,
   IL_ALLOCATE_BUFFER,
   IL_FREE_BUFFER,
   IL_EMPTY_THIS_BUFFER,
   IL_FILL_THIS_BUFFER,
   IL_SET_CALLBACKS,
   IL_COMPONENT_ROLE_ENUM,

   IL_COMPONENT_DEINIT,

   IL_EVENT_HANDLER,
   IL_EMPTY_BUFFER_DONE,
   IL_FILL_BUFFER_DONE,

   IL_COMPONENT_NAME_ENUM,

   IL_FUNCTION_MAX_NUM,
   IL_FUNCTION_MAX = 0x7fffffff
} IL_FUNCTION_T;


// size of the largest structure passed by get/set
// parameter/config
// this should be calculated at compile time from IL headers
// must be a multiple of VC_INTERFACE_BLOCK_SIZE
#define VC_ILCS_MAX_PARAM_SIZE 288

// size of the largest structure below
#define VC_ILCS_MAX_CMD_LENGTH (sizeof(IL_GET_EXECUTE_T))
#define VC_ILCS_MAX_RESP_LENGTH VC_ILCS_MAX_CMD_LENGTH

#define VC_ILCS_MAX_INLINE (VCHI_MAX_MSG_SIZE-8)

// all structures should be padded to be multiples of
// VC_INTERFACE_BLOCK_SIZE in length (currently 16)
typedef struct {
   void *reference;
} IL_EXECUTE_HEADER_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
} IL_RESPONSE_HEADER_T;

// create instance
typedef struct {
   OMX_PTR mark;
   char name[256];
} IL_CREATE_COMPONENT_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   void *reference;
   OMX_U32 numPorts;
   OMX_U32 portDir;
   OMX_U32 portIndex[32];
} IL_CREATE_COMPONENT_RESPONSE_T;

// set callbacks
typedef struct {
   void *reference;
   void *pAppData;
} IL_SET_CALLBACKS_EXECUTE_T;

// get state
typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_STATETYPE state;
} IL_GET_STATE_RESPONSE_T;

// get parameter & get config
#define IL_GET_EXECUTE_HEADER_SIZE 8
typedef struct {
   void *reference;
   OMX_INDEXTYPE index;
   unsigned char param[VC_ILCS_MAX_PARAM_SIZE];
} IL_GET_EXECUTE_T;

#define IL_GET_RESPONSE_HEADER_SIZE 8
typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   unsigned char param[VC_ILCS_MAX_PARAM_SIZE];
} IL_GET_RESPONSE_T;

// set parameter & set config
#define IL_SET_EXECUTE_HEADER_SIZE 8
typedef struct {
   void *reference;
   OMX_INDEXTYPE index;
   unsigned char param[VC_ILCS_MAX_PARAM_SIZE];
} IL_SET_EXECUTE_T;

// send command
typedef struct {
   void *reference;
   OMX_COMMANDTYPE cmd;
   OMX_U32 param;
   OMX_MARKTYPE mark;
} IL_SEND_COMMAND_EXECUTE_T;

// event handler callback
typedef struct {
   void *reference;
   OMX_EVENTTYPE event;
   OMX_U32 data1;
   OMX_U32 data2;
   OMX_PTR eventdata;
} IL_EVENT_HANDLER_EXECUTE_T;

// use/allocate buffer
typedef struct {
   void *reference;
   OMX_PTR bufferReference;
   OMX_U32 port;
   OMX_U32 size;
   void *eglImage;
} IL_ADD_BUFFER_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_PTR reference;
   OMX_BUFFERHEADERTYPE bufferHeader;
} IL_ADD_BUFFER_RESPONSE_T;

// free buffer
typedef struct {
   void *reference;
   OMX_U32 port;
   OMX_PTR bufferReference;
   IL_FUNCTION_T func;
   OMX_PTR inputPrivate;
   OMX_PTR outputPrivate;
} IL_FREE_BUFFER_EXECUTE_T;

// empty/fill this buffer
typedef enum {
   IL_BUFFER_NONE,
   IL_BUFFER_BULK,
   IL_BUFFER_INLINE,
   IL_BUFFER_MAX = 0x7fffffff
} IL_BUFFER_METHOD_T;

typedef struct {
   OMX_U8 header[VCHI_BULK_ALIGN-1];
   OMX_U8 headerlen;
   OMX_U8 trailer[VCHI_BULK_ALIGN-1];
   OMX_U8 trailerlen;
} IL_BUFFER_BULK_T;

typedef struct {
   OMX_U8 buffer[1];
} IL_BUFFER_INLINE_T;

typedef struct {
   void *reference;
   OMX_BUFFERHEADERTYPE bufferHeader;
   IL_BUFFER_METHOD_T method;
} IL_PASS_BUFFER_EXECUTE_T;

// get component version
typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   char name[128];
   OMX_VERSIONTYPE component_version;
   OMX_VERSIONTYPE spec_version;
   OMX_UUIDTYPE uuid;
} IL_GET_VERSION_RESPONSE_T;

// get extension index
typedef struct {
   void *reference;
   char name[128];
} IL_GET_EXTENSION_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_INDEXTYPE index;
} IL_GET_EXTENSION_RESPONSE_T;

// component role enum
typedef struct {
   void *reference;
   OMX_U32 index;
} IL_COMPONENT_ROLE_ENUM_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_U8 role[128];
} IL_COMPONENT_ROLE_ENUM_RESPONSE_T;

typedef struct {
   void *reference;
   OMX_U32 port;
   OMX_PTR tunnel_ref;       // reference to use in requests - address of host/vc component
   OMX_BOOL tunnel_host;     // whether tunnel_ref is a host component
   OMX_U32 tunnel_port;
   OMX_TUNNELSETUPTYPE setup;
} IL_TUNNEL_REQUEST_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_TUNNELSETUPTYPE setup;
} IL_TUNNEL_REQUEST_RESPONSE_T;

typedef struct {
   int index;
} IL_COMPONENT_NAME_ENUM_EXECUTE_T;

typedef struct {
   IL_FUNCTION_T func;
   OMX_ERRORTYPE err;
   OMX_U8 name[128];
} IL_COMPONENT_NAME_ENUM_RESPONSE_T;


#endif // VC_ILCS_DEFS_H