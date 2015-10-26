/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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
/*  rtc_cpuapi.h - Register access for BCM4760 RTC block.
 */

#ifndef RTC_CPUAPI_H
#define RTC_CPUAPI_H

#include <linux/types.h>

#define RTC_PSRST 		0x00000000
#define RTC_PER 		0x00000001
#define RTC_MATCH		0x00000002
#define RTC_SET_DIV 		0x00000003
#define RTC_SET_RTC 		0x00000004
#define RTC_CLR_INT_MATCH 	0x00000005
#define RTC_DLY1 		0x00000006
#define RTC_DLY2 		0x00000007
#define RTC_TIMEOUT 		0x00000008
#define RTC_MASK 		0x00000009
#define RTC_EVENT0 		0x0000000a
#define RTC_EVENT1 		0x0000000b
#define RTC_EVENT2 		0x0000000c
#define RTC_CLK_DIV_REG 	0x0000000d
#define RTC_RTC_REG 		0x0000000e
#define RTC_TRIG_STAT 		0x0000000f
#define RTC_INTERRUPT_ENABLE 	0x00000010
#define RTC_RESET_ACCESS 	0x00000011

#define RTC_SW_REG00		0x00000012
#define RTC_SW_REG01 		0x00000013
#define RTC_SW_REG02 		0x00000014
#define RTC_SW_REG03 		0x00000015
#define RTC_SW_REG04 		0x00000016
#define RTC_SW_REG05 		0x00000017
#define RTC_SW_REG06 		0x00000018
#define RTC_SW_REG07 		0x00000019
#define RTC_SW_REG08 		0x0000001a
#define RTC_SW_REG09 		0x0000001b
#define RTC_SW_REG10 		0x0000001c
#define RTC_SW_REG11 		0x0000001d
#define RTC_SW_REG12 		0x0000001e
#define RTC_SW_REG13 		0x0000001f
#define RTC_SW_REG14 		0x00000020
#define RTC_SW_REG15 		0x00000021
#define RTC_SW_REG16 		0x00000022
#define RTC_SW_REG17 		0x00000023
#define RTC_SW_REG18 		0x00000024
#define RTC_SW_REG19 		0x00000025
#define RTC_SW_REG20 		0x00000026
#define RTC_SW_REG21 		0x00000027
#define RTC_SW_REG22 		0x00000028
#define RTC_SW_REG23 		0x00000029


uint32_t rtc_read (uint32_t addr);
void rtc_write (uint32_t addr, uint32_t data);
void init_bbl(void* warm_boot_address);
void deinit_bbl(void);
#endif
