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




/*
*
*****************************************************************************
*
*  pmu_pcf50611.h
*
*  PURPOSE:
*
*  This file defines the interface to the Philips PCF50611 PMU chip.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( PMU_PCF50611_H )
#define PMU_PCF50611_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */

#define PCF50611_I2C_BASE_ADDR  0x09


/* Register addresses for direct accesses */

#define PCF50611_REG_ID      0x00
#define PCF50611_REG_INT1    0x01
#define PCF50611_REG_INT2    0x02
#define PCF50611_REG_INT3    0x03
#define PCF50611_REG_INT4    0x04
#define PCF50611_REG_OOCC    0x05
#define PCF50611_REG_OOCS1   0x06
#define PCF50611_REG_OOCS2   0x07
#define PCF50611_REG_RTC1    0x08
#define PCF50611_REG_RTC2    0x09
#define PCF50611_REG_RTC3    0x0A
#define PCF50611_REG_RTC4    0x0B
#define PCF50611_REG_RTC1A   0x0C
#define PCF50611_REG_RTC2A   0x0D
#define PCF50611_REG_RTC3A   0x0E
#define PCF50611_REG_RTC4A   0x0F
#define PCF50611_REG_REC1C   0x10
#define PCF50611_REG_REC2C   0x11
#define PCF50611_REG_INT1M   0x12
#define PCF50611_REG_INT2M   0x13
#define PCF50611_REG_INT3M   0x14
#define PCF50611_REG_INT4M   0x15
#define PCF50611_REG_D1C     0x16
#define PCF50611_REG_D2C     0x17
#define PCF50611_REG_D3C     0x18
#define PCF50611_REG_HCC     0x19
#define PCF50611_REG_IOC     0x1A
#define PCF50611_REG_LPC     0x1B
#define PCF50611_REG_RF1C    0x1C
#define PCF50611_REG_RF2C    0x1D
#define PCF50611_REG_CPC     0x1E
#define PCF50611_REG_DCDC1   0x1F
#define PCF50611_REG_DCDC2   0x20
#define PCF50611_REG_DCDC3   0x21
#define PCF50611_REG_DCDC4   0x22
#define PCF50611_REG_DCDC5   0x23
#define PCF50611_REG_BVMC    0x24
#define PCF50611_REG_PWM1S   0x25
#define PCF50611_REG_PWM1D   0x26
#define PCF50611_REG_LED1C   0x27
#define PCF50611_REG_LED2C   0x28
#define PCF50611_REG_LEDCC   0x29
#define PCF50611_REG_GPOC1   0x2A
#define PCF50611_REG_GPOC2   0x2B
#define PCF50611_REG_SIMC1   0x2C
#define PCF50611_REG_SIMC2   0x2D
#define PCF50611_REG_CBCC1   0x2E
#define PCF50611_REG_CBCC2   0x2F
#define PCF50611_REG_CBCC3   0x30
#define PCF50611_REG_CBCC4   0x31
#define PCF50611_REG_CBCC5   0x32
#define PCF50611_REG_CBCC6   0x33
#define PCF50611_REG_CBCS1   0x34
#define PCF50611_REG_BBCC    0x35
#define PCF50611_REG_DEBC    0x36
#define PCF50611_REG_PSSC    0x37
#define PCF50611_REG_ISRCAL  0x38
#define PCF50611_REG_VMAXCAL 0x39
#define PCF50611_REG_PWM2S   0x3A
#define PCF50611_REG_PWM2D   0x3B
#define PCF50611_REG_DCDC6   0x83
#define PCF50611_REG_LCC     0x84
#define PCF50611_REG_ID_VAL_N1C     (0x13)
#define PCF50611_REG_ID_VAL_N2A     (0x73)

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* PMU_PCF50611_H */

