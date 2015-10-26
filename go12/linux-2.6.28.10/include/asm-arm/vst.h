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



/*
 * ARM/asm header file for VST (Variable Sleep Time, tick elimination)
*****************************************************************************/

#ifndef ASM_ARM_VST
#define ASM_ARM_VST

extern void vst_sleep_till(unsigned long rjiffies);
extern void vst_wakeup(struct pt_regs *regs, unsigned int irq);

#define vst_stop_jiffie_int() do{}while(0)
#define vst_start_jiffie_int() do{}while(0)

#endif  /* ASM_ARM_VST */

