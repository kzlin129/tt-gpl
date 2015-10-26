/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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
*
*****************************************************************************
*
*  PURPOSE:
*     This file contains definitions for the Power manager registers.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_4760_PWRSEQ_H )
#define __ASM_ARCH_REG_4760_PWRSEQ_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>

#include <linux/suspend.h>

#include <asm/arch/hardware.h>

#define UNLOCK_CMU_SIGNATURE		0xBCBC4760
#define LOCK_CMU_SIGNATURE		0x00000000

/* Wake up masks used in PML_R_PML_WAKEUP_MASK0/1_MEMADDR */
#define PML_WAKEUP_MASK_ALL			0xFFFFFFFF
#define PML_WAKEUP_MASK0_GPIO_0			0x00000001
#define PML_WAKEUP_MASK0_GPIO_1			0x00000002
#define PML_WAKEUP_MASK0_GPIO_2			0x00000004
#define PML_WAKEUP_MASK0_GPIO_3			0x00000008
#define PML_WAKEUP_MASK0_GPIO_4			0x00000010
#define PML_WAKEUP_MASK0_GPIO_5			0x00000020
#define PML_WAKEUP_MASK0_GPIO_6			0x00000040

#define PML_WAKEUP_MASK1_RTC_MATCHED	0x00000001
#define PML_WAKEUP_MASK1_RTC_INT	0x00000002
#define PML_WAKEUP_MASK1_TIM2_COUNTER1	0x00000004
#define PML_WAKEUP_MASK1_TIM2_COUNTER2	0x00000008


/* Suspend delay values used in PML_R_PML_DELAYS0/1_MEMADDR */
#define PML_DELAY_PWRDOWN		0x3F
#define PML_DELAY_ISOLATE		0x3F
#define PML_DELAY_STOPCLOCKS		0x3F
#define PML_DELAY_SREFRESH		0x3F
#define PML_DELAY_ENABLEPOR		0x3F
#define PML_DELAY_ENABLECLKS		0x3F
#define PML_DELAY_ENABLEISO		0x3F
#define PML_DELAY_ENABLEPWR		0x3F

/* power states */
typedef enum
{
    PWRMGR_PWRMODE_0 = 0,
    PWRMGR_PWRMODE_1 = 1,
    PWRMGR_PWRMODE_2 = 2,
    PWRMGR_PWRMODE_3 = 3,
} PWRMGR_PWRMODE_T;

typedef enum 
{
    LOW_WORD = 0 ,
    HIGH_WORD = 1 ,
    NUM_INTR_WORDS
} high_low_t ;

struct PwrSeqRegisterSave
{
	unsigned int	Address;		/* Address of register to save/restore */
	unsigned int	OrValue;		/* Value of register (ORed into actual register */
	unsigned int	Size;			/* Size of register (1,2,4) */
	unsigned int	Mask;			/* Not mask for register, mask for OrValue */
};

int cds_pll_init(uint32_t ref_clk_idx);
int cds_pll_switch(uint32_t ref_index, int alternate);

unsigned int bcm4760_pm_suspend_valid(suspend_state_t state) ;
unsigned int bcm4760_pm_suspend_prepare(void) ;
unsigned int bcm4760_pm_suspend_standby(void) ;
unsigned int bcm4760_pm_suspend_mem(void) ;
void bcm4760_pm_suspend_finish(void) ;

int pwrseq_switch_pm_states_begin(void);
int pwrseq_switch_pm_states_finalize(int new_pm_state);
int pwrseq_get_current_pm_state(void);
void pwrseq_force_pm_state_sw_override(int new_pm_state);
void pwrseq_stop_pm_state_sw_override(int new_pm_state);

#endif
