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



#ifndef __OMX_H
#define __OMX_H

#include "OMX_Core.h"
#include "OMX_Types.h"
#include "OMX_Component.h"
#include "OMX_Broadcom.h"
#include "OMX_Audio.h"
//#include "vchost_ilcore.h"


#if defined( __KERNEL__ )

#define OMX_DEBUG(fmt, args...) printk("[%s:%d]: " fmt, __FUNCTION__, __LINE__, ##args)

#else

#define OMX_DEBUG(fmt, args...) {fprintf(stdout,  "u[%s:%d]: " fmt, __FUNCTION__ , __LINE__,  ##args); fflush(stdout); }
  
#endif

/*
 * Misc. libomx API exported by libomx in addition to OpenMax API
*/
int   libomx_init(void);
int   libomx_uninit(void);
int   libomx_initialized(void);
void* libomx_malloc(int bytecount);
void  libomx_free(void* ptr);
int   libomx_gencmd(char* response, int resplen, char* cmdstr);
#define libomx_gencmd2(cmdstr) libomx_gencmd(NULL, 0, cmdstr)
void  libomx_delay_msec(int msec);
void  libomx_vc_component_lock_obtain(void);
void  libomx_vc_component_lock_release(void);

#endif
