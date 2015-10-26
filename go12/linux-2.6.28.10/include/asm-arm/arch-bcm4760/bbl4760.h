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
/*  bbl4760.h - Definitions for BBL logic for BCM4760.
 */

#ifndef BBL4760_H
#define BBL4760_H

#define USE_TMR_TO_CHECK_RTC            0
#define TMR_BASE_INPUT_CLK_32KHz        1

#define RTC_PERIODIC_MS_CLOCK_TICK      125UL                                                               // 125 ms resolution for periodic register
#define RTC_PERIODIC_TICKS_PER_SEC      (1000UL / RTC_PERIODIC_MS_CLOCK_TICK)                               // number of periodic clock tick per second; 8
#define RTC_PERIODIC_TEST_SEC_VALUE     20UL                                                                // total time of rtc periodic test in seconds

#define RTC_MATCH_MS_CLOCK_TICK         1000UL                                                              // 1000 ms resolution for match register
#define RTC_MATCH_TICKS_PER_SEC         (1000UL / RTC_MATCH_MS_CLOCK_TICK)                                  // number of match clock tick per second; 1
#define RTC_MATCH_TEST_SEC_VALUE        10UL                                                                // total time of rtc match test in seconds


#if TMR_BASE_INPUT_CLK_32KHz
#define TMR_USEC_CLOCK_TICK             30UL                                                                // 30 usec resolution for a 32 KHz input clock to timer module
#define TMR_TICKS_PER_SEC               32786                                                               // number of timer clock tick per second
#define TMR_MAX_SECS_VALUE              131072                                                              // maximum number of seconds the timer can count up to based on 32 KHz input clock
#else
#define TMR_NSEC_CLOCK_TICK             41UL                                                                // 41 ns resolution for a 24 MHz input clock to timer module
#define TMR_TICKS_PER_SEC               24000000                                                            // number of timer clock tick per second
#define TMR_MAX_SECS_VALUE              179                                                                 // maximum number of seconds the timer can count up to based on 24 MHz input clock
#endif

#define FIRST_USER_RTC_REG_OFFSET       RTC_SW_REG00                                                        // register offset of the first RTC user register
#define LAST_USER_RTC_REG_OFFSET        RTC_SW_REG23                                                        // register offset of the last RTC user register

/* BBL SW data register offset */

#define BBL_OFFSET_DRAM_MAX_ADDR		0x12
#define BBL_OFFSET_DRAM_MAX_ADDR_LOCK	0x13
#define BBL_OFFSET_ADDR_SCR_BASE		0x14
#define BBL_OFFSET_ADDR_SCR_BASE_MSK	0x15
#define BBL_OFFSET_ADDR_SCR_MSK			0x16
#define BBL_OFFSET_ADDR_KEY				0x17
#define BBL_OFFSET_DATA_KEY_LO			0x18
#define BBL_OFFSET_DATA_KEY_HI			0x19
#define BBL_OFFSET_ADDR_SWZ_CTRL_LO		0x1A
#define BBL_OFFSET_ADDR_SWZ_CTRL_HI		0x1B
#define BBL_OFFSET_DATA_SWZ_CTRL_LO_0	0x1C
#define BBL_OFFSET_DATA_SWZ_CTRL_LO_1	0x1D
#define BBL_OFFSET_EMI_DRAM_TIMING_0	0x1E
#define BBL_OFFSET_EMI_DRAM_TIMING_1	0x1F
#define BBL_OFFSET_SCR_CONTROL			0x20
#define BBL_OFFSET_EMI_CFG_BITS			0x21
#define BBL_OFFSET_DRAM_ADDRESS			0x22
#define BBL_OFFSET_WARM_TAG				0x23
#define BBL_OFFSET_EMI_PAD_CONTROL				0x24
#define BBL_OFFSET_DDR_READ_NCDL_OFFSET		0x25
#define BBL_OFFSET_DDR_WRITE_NCDL_OFFSET	0x26

#define BBL_OFFSET_DDR_LUKEWARM_BOOT_OFFSET 0x28
#define BBL_OFFSET_DDR_TOMTOM_OFFSET 0x29

#define WARM_BOOT_TAG4760              0xAAAA5555
#define LUKEWARM_BOOT_TAG4760          0x5051484A

//void bbl_setup_timer(uint32_t timer, uint32_t time_val);
//void bbl_wait_timer_expire(uint32_t timer);
//void bbl_reg_read(int32_t argc, int8_t **argv);
//void bbl_reg_write(int32_t argc, int8_t **argv);

#endif // #ifndef BBL_H
