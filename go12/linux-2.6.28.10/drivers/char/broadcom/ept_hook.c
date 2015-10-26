/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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




/*****************************************************************************
*
*  ept_hook.c
*
*  PURPOSE:
*
*     This driver is just a "hook" for the EPT loadable module.
*     It contains code that muse be compiled into the Linux Kernel
*     before the EPT module can be loaded.
*
*  NOTES:
*
*****************************************************************************/

#include <linux/module.h>

// Global pointer used by ept module (in mips/lccdrv.c) to point to ZSP memory 
// that may not be freed when ept is unloaded.  
void *pZspOverlayPrgMemory = NULL;   
EXPORT_SYMBOL (pZspOverlayPrgMemory);


