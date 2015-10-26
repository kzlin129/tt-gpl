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

Project  :  VMCS Host Apps
Module   :  Framework - VMCS
File     :  $RCSfile: vmcs_framework.c,v $
Revision :  $Revision: #9 $

FILE DESCRIPTION
Contains the code to start and run Host applications.
=============================================================================*/

#ifndef _HOST_MSGFIFO_WRAPPER_H
#define _HOST_MSGFIFO_WRAPPER_H

#include <linux/broadcom/vc03/vcos.h>
#include "vchi.h"

//typedef enum {
//   MSGFIFO_WRAPPER_READ = 0,
//   MSGFIFO_WRAPPER_WRITE
//} MSGFIFO_WRAPPER_CMD_T;

void
host_vchi_msgfifo_wrapper_init( VCHI_INSTANCE_T initialise_instance,
                                VCHI_CONNECTION_T **connections,
                                uint32_t num_connections );

void host_vchi_msgfifo_wrapper_exit(void);

void host_vchi_msgfifo_wrapper_read( unsigned char *host_addr, uint32_t vc_addr, int nbytes );

void host_vchi_msgfifo_wrapper_write( uint32_t vc_addr, unsigned char *host_addr, int nbytes );

void host_vchi_msgfifo_wrapper_bulk_read( unsigned char *host_addr, uint32_t vc_addr, int nbytes );

void host_vchi_msgfifo_wrapper_bulk_write( uint32_t vc_addr, unsigned char *host_addr, int nbytes );

#endif // _MSGFIFO_WRAPPER_H
