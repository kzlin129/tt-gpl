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

Project  :  VMCS
Module   :  Gencmd Service
Id       :  $Id: //software/vc3/DEV/applications/vmcs/vchi/gencmd_common.h#2 $

FILE DESCRIPTION
Gencmd Service common header
=============================================================================*/


#ifndef GENCMD_COMMON_H
#define GENCMD_COMMON_H
#include "vchi/message.h"
#define GENCMD_4CC  MAKE_FOURCC("GCMD")

#define GENCMDSERVICE_MSGFIFO_SIZE 1024
//Format of reply message is error code followed by a string

#endif
