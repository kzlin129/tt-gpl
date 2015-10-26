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



#ifndef CFG_GLOBAL_DEFINES_H
#define CFG_GLOBAL_DEFINES_H

/* CPU */
#define   ARMEB  1
#define   MIPS32 2
#define   ARM9   3
#define   ARM11  4

/* CHIP */
#define BCM1103 1
#define BCM1161 2
#define BCM1191 4
#define BCM2153 5
#define BCM2820 6

#define BCM2826 8
#define FPGA11107 9
#define BCM11107   10
#define BCM4760 11

/* CFG_GLOBAL_BCM116X_TYPE */
#define CFG_GLOBAL_BCM1160    1
#define CFG_GLOBAL_BCM1161    2
#define CFG_GLOBAL_BCM2153    3

/* CFG_GLOBAL_CHIP_FAMILY types */
#define CFG_GLOBAL_CHIP_FAMILY_NONE        0
#define CFG_GLOBAL_CHIP_FAMILY_BCM116X     2
#define CFG_GLOBAL_CHIP_FAMILY_BCMRING     4
#define CFG_GLOBAL_CHIP_FAMILY_BCM1103     8
#define CFG_GLOBAL_CHIP_FAMILY_BCM476X     16

/* CFG_GLOBAL_ROOT_FILE_SYSTEM */
#define JFFS2_RFS      1
#define CRAMFS_RFS     2
#define INITRAMFS      3

#endif

