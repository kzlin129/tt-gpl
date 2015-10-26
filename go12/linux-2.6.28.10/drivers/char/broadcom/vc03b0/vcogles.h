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
FILE DESCRIPTION
Host side interface to OpenGL/ES proxy service
=============================================================================*/

#ifndef  _VC_OGLES_H_
#define _VC_OGLES_H_

extern void* ogles_event;
extern void* ogles_lock;
#define OGLES_STAT_SIZE 32

/* OGLES service code */
enum {
   VC_OGLESCMD_EXECUTE = 1,
   VC_OGLESCMD_DATA,
};

/* Initialise the opengl es service */
int vc_ogles_init(void);

/* Start the service, (load the driver) */
int vc_ogles_start(void);

/* Stop the service, (unload the driver) */
int vc_ogles_stop(void);

/* Stop the service */
void vc_ogles_exit(void);

int vc_ogles_inum(void);
#endif
