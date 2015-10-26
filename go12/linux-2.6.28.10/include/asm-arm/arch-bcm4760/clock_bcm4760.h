/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   clock_bcm4760.h 
 * 
 *   @brief  BCM4760 side interface functions for clock frame work.
 * 
 ****************************************************************************/

#ifndef __ARCH_ARM_MACH_BCM4760_CLOCK_H
#define __ARCH_ARM_MACH_BCM4760_CLOCK_H

// Error codes.
#define BCM4760_ERR_CODE_INVALID_CORE_CLOCK      100
#define BCM4760_ERR_CODE_INVALID_PLL_OSCI_CLOCK  101
#define BCM4760_ERR_CODE_INVALID_CLOCK_TYPE      102

#define BCM4760_CF_MAX_CLOCKS 128 

#define FREQ_1_IN_MHZ  100000000
#define FREQ_2_IN_MHZ  200000000
#define FREQ_3_IN_MHZ  286000000
#define FREQ_4_IN_MHZ  400000000
#define FREQ_5_IN_MHZ  500000000

int bcm4760_clk_proc_summary ( char *page, char **start, off_t off, int count, int *eof, void *data) ;
int bcm4760_clk_set_rate(struct clk *clk, unsigned long rate) ;

int __init bcm4760_clk_init(void) ;

typedef enum 
{
    BCM4760_TYPE_CORE_CLOCK = 0,
    BCM4760_TYPE_PLL_CLOCK ,
    BCM4760_TYPE_OSCI 
} core_clk_osci_t ;

#endif
