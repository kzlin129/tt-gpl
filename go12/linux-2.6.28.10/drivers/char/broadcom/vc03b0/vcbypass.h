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




#ifndef VC_BYPASS_HEADER
#define VC_BYPASS_HEADER

/* Bypass Logic Device Register (VCBYPDEV). */

#define VCBYPDEV_FORCEDATA_BYPDATA  (0<<6)
#define VCBYPDEV_FORCEDATA_BYPFDATA (1<<6)
#define VCBYPDEV_OUTMODE_MODE80     (0<<5)
#define VCBYPDEV_OUTMODE_MODE68     (1<<5)
#define VCBYPDEV_HDEV_MAIN_CS0      (0<<3)
#define VCBYPDEV_HDEV_MAIN_CS1      (1<<3)
#define VCBYPDEV_HDEV_MAIN_CS2      (2<<3)
#define VCBYPDEV_HDEV_SECONDARY     (3<<3)
#define VCBYPDEV_FORCE_RD_NOEFFECT  (0<<1)
#define VCBYPDEV_FORCE_RD_ACTIVE    (1<<1)
#define VCBYPDEV_FORCE_WR_NOEFFECT  (0<<0)
#define VCBYPDEV_FORCE_WR_ACTIVE    (1<<0)

/* Bypass Logic Enable Register (VCBYPENA1/2). */

#define VCBYPENA1_ENABLE (0xA5A5)
#define VCBYPENA2_ENABLE (0x5A5A)

/* Bypass Logic Control Register (VCBYPCTRL). */

#define VCBYPCTRL_RDGLITCH_NONE (0<<12)
#define VCBYPCTRL_RDGLITCH_A    (1<<12)
#define VCBYPCTRL_RDGLITCH_AA   (2<<12)
#define VCBYPCTRL_RDGLITCH_B    (3<<12)
#define VCBYPCTRL_WRGLITCH_NONE (0<<12)
#define VCBYPCTRL_WRGLITCH_A    (1<<10)
#define VCBYPCTRL_WRGLITCH_AA   (2<<10)
#define VCBYPCTRL_WRGLITCH_B    (3<<10)
#define VCBYPCTRL_HRDY          (1<<9)
#define VCBYPCTRL_VBUSPRES      (1<<8)
#define VCBYPCTRL_HBYPASS       (1<<7)

#endif
