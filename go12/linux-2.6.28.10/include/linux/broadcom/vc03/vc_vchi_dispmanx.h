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
Module   :  Software host interface (host-side)
File     :  $Id: //software/vc3/REL/interface/vmcs_host/vc_vchi_dispmanx.h#2 $
Revision :  $Revision: #2 $

FILE DESCRIPTION
Display manager service API (VCHI native).
=============================================================================*/

#ifndef VC_VCHI_DISPMANX_H
#define VC_VCHI_DISPMANX_H

#define VC_NUM_HOST_RESOURCES 64
#define DISPMANX_MSGFIFO_SIZE 1024
#define DISPMANX_CLIENT_NAME MAKE_FOURCC("DISP")
#define DISPMANX_NOTIFY_NAME MAKE_FOURCC("UPDH")

//Or with command to indicate we don't need a response
#define DISPMANX_NO_REPLY_MASK (1<<31)

//VCHI dispmanx command code is different from the msgfifo dispmanx
typedef enum  {
   // IMPORTANT - DO NOT ALTER THE ORDER OF COMMANDS IN THIS ENUMERATION
   // NEW FUNCTIONS SHOULD BE ADDED TO THE END, AND MUST ALSO BE ADDED TO
   // THE HOST SIDE FUNCTION TABLE
   
   // No function configured - do not use
   EDispmanNoFunction = 0,
   
   // Dispman pre-configure functions
   EDispmanGetDevices,
   EDispmanGetModes,
   
   // Dispman resource-related functions
   EDispmanResourceCreate,
   EDispmanResourceCreateFromImage,
   EDispmanResourceDelete,
   EDispmanResourceGetData,
   EDispmanResourceGetImage,
   
   // Dispman display-related functions
   EDispmanDisplayOpen,
   EDispmanDisplayOpenMode,
   EDispmanDisplayOpenOffscreen,
   EDispmanDisplayReconfigure,
   EDispmanDisplaySetDestination,
   EDispmanDisplaySetBackground,
   EDispmanDisplayGetInfo,
   EDispmanDisplayClose,
   
   
   // Dispman update-related functions
   EDispmanUpdateStart,
   EDispmanUpdateSubmit,
   EDispmanUpdateSubmitSync,
   
   // Dispman element-related functions
   EDispmanElementAdd,
   EDispmanElementModified,
   EDispmanElementRemove,
   EDispmanElementChangeSource,
   EDispmanElementChangeAttributes,

   //More commands go here...
   EDispmanResourceFill,    //Comes from uideck
   EDispmanQueryImageFormats,
   EDispmanBulkWrite,
   EDispmanDisplayOrientation,
   EDispmanSnapshot,

   EDispmanMaxFunction
} DISPMANX_COMMAND_T;


typedef struct {
   char     description[32];
   uint32_t width;
   uint32_t height;
   uint32_t aspect_pixwidth;
   uint32_t aspect_pixheight;
   uint32_t fieldrate_num;
   uint32_t fieldrate_denom;
   uint32_t fields_per_frame;
   uint32_t transform;        
} GET_MODES_DATA_T;

typedef struct {
   int32_t  response;
   char     description[32];
   uint32_t width;
   uint32_t height;
   uint32_t aspect_pixwidth;
   uint32_t aspect_pixheight;
   uint32_t fieldrate_num;
   uint32_t fieldrate_denom;
   uint32_t fields_per_frame;
   uint32_t transform;        
} GET_INFO_DATA_T;

//Attributes changes flag mask
#define ELEMENT_CHANGE_LAYER          (1<<0);
#define ELEMENT_CHANGE_OPACITY        (1<<1);
#define ELEMENT_CHANGE_DEST_RECT      (1<<2);
#define ELEMENT_CHANGE_SRC_RECT       (1<<3);
#define ELEMENT_CHANGE_MASK_RESOURCE  (1<<4);
#define ELEMENT_CHANGE_TRANSFORM      (1<<5);

#endif
