/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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

#ifndef __BCM476X_HW_TIMER_H
#define __BCM476X_HW_TIMER_H
#include <asm/mach-types.h>
#include <asm/arch/platform.h>

#define MAX_HW_TIMERS           8

#define TIMER_LOAD_OFFSET       0x00                // Timer's load offset
#define TIMER_VALUE_OFFSET      0x04                // Timer's value offset
#define TIMER_CONTROL_OFFSET    0x08                // Timer's control offset
#define TIMER_INTCLR_OFFSET     0x0c                // Timer's interrupt clear offset
#define TIMER_RIS_OFFSET        0x10                // Timer's raw interrupt offset
#define TIMER_MIS_OFFSET        0x14                // Timer's masked interrupt status offset
#define TIMER_BGLOAD_OFFSET     0x18                // Timer's background load offset

#define TIMER_COUNTER_REG_OFFSET 0x20

/* Timer control values */
/*
 *	Timer Control Register Bits
 */
#define TIMER_CTRL_ONESHOTMODE  (1 << 0)        /*One shot mode */
#define TIMER_CTRL_16BIT	    (0 << 1)		/* 16-bit counter mode */
#define TIMER_CTRL_32BIT	    (1 << 1)		/* 32-bit counter mode */
#define TIMER_CTRL_IE		    (1 << 5)		/* Interrupt enable */
#define TIMER_CTRL_PERIODIC     (1 << 6)        /* Periodic mode */
#define TIMER_CTRL_EN		    (1 << 7)		/* Timer enable */
#define TIMER_CTRL_CLK2		    (1 << 9)		/* Clock 2 selected */
#define TIMER_CTRL_CLK1		    (0 << 9)		/* Clock 1 slected */
#define TIMER_CTRL_PREBY16      (1 << 2)		/* prescale divide by 16 */
#define TIMER_CTRL_PREBY256     (2 << 2)		/* prescale divide by 256 */


void sys_timer_init(void);
void sys_timer_resume(void);
uint32_t sys_timer_gettimeoffset(void);
void hw_timer_init(int timer_id, uint32_t reload, uint32_t ctl);
void hw_timer_enable(int timer_id);
void hw_timer_disable(int timer_id);
uint32_t hw_timer_get_counter(int timer_id);
uint32_t hw_timer_base(int32_t timer_id);

#endif
