/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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



#ifndef _BCM_FUSE_MEMMAP_H_
#define _BCM_FUSE_MEMMAP_H_

/* from nansdram_memmap.h */
#define IPC_BASE				0x81100000     	// 1MB IPC shared RAM
#define IPC_SIZE	 			0x00100000
#define CP_START_ADDR		(0x80300000)

#define CP_BOOT_BASE_PHYS	(0xFFFF0000UL)
#define CP_BOOT_BASE_SIZE	(0x40)

#endif //_BCM_FUSE_MEMMAP_H_

