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
Copyright (c) 2002 Alphamosaic Limited.

Project  :  VideoCore Software Host Interface (Host-side functions)
Module   :  File system (host-side)
File     :  $RCSfile $
Revision :  $Revision $

FILE DESCRIPTION
File service command enumeration.
=============================================================================*/

#ifndef VC_VCHI_FILESERVICE_DEFS_H
#define VC_VCHI_FILESERVICE_DEFS_H

/* Definitions (not used by API) */

/* structure used by both side to communicate */
#define FILESERV_MAX_DATA	1000

#define FILESERV_MAX_BULK  (64*1024) //4096

#define FILESERV_4CC  MAKE_FOURCC("FSRV")

//this has to add up to 1024 bytes 
typedef struct{
	uint32_t xid;		    //4 /* transaction's ID, used to match cmds with response */
   uint32_t cmd_code;    //4
   uint32_t params[4];   //8
   char  data[FILESERV_MAX_DATA];
}FILESERV_MSG_T;

typedef enum
{
   FILESERV_RESP_OK,
   FILESERV_RESP_ERROR,
   FILESERV_BULK_READ,
   FILESERV_BULK_WRITE,
   
} FILESERV_RESP_CODE_T;


/* Protocol (not used by API) version 1.2 */



#endif
