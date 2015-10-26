/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * Description: Public header of the LED Matrix driver shared between various
 * platforms
 */

#ifndef LEDM_H
#define LEDM_H

#if CONFIG_ARCH_BCM11107
#define LEDM_MAX_ROWS     6
#define LEDM_MAX_COLS     12
/* add future architectures here */
#endif

/*
 * Clock frequency divisor
 */
typedef enum ledm_clk_div {
   LEDM_CLK_DIV_2 = 0,
   LEDM_CLK_DIV_4,
   LEDM_CLK_DIV_8,
   LEDM_CLK_DIV_16,
   LEDM_CLK_DIV_INVALID
} LEDM_CLK_DIV;

#endif /* LEDM_H */
