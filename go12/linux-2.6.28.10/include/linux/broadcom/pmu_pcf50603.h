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
*  pmu_pcf50603.h
*
*  PURPOSE:
*
*  This file defines the interface to the Philips PCF50603 PMU chip.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( PMU_PCF50603_H )
#define PMU_PCF50603_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */

#define PCF50603_I2C_BASE_ADDR	0x09

/* Register addresses for direct accesses */

#define PCF50603_REG_INT1    0x00
#define PCF50603_REG_INT2    0x01
#define PCF50603_REG_INT3    0x02
#define PCF50603_REG_OOCC    0x03
#define PCF50603_REG_OOCS1   0x04
#define PCF50603_REG_OOCS2   0x05
#define PCF50603_REG_RTC1    0x06
#define PCF50603_REG_RTC2    0x07
#define PCF50603_REG_RTC3    0x08
#define PCF50603_REG_RTC4    0x09
#define PCF50603_REG_RTC1A   0x0A
#define PCF50603_REG_RTC2A   0x0B
#define PCF50603_REG_RTC3A   0x0C
#define PCF50603_REG_RTC4A   0x0D
#define PCF50603_REG_REC2C   0x0E
#define PCF50603_REG_INT1M   0x0F
#define PCF50603_REG_INT2M   0x10
#define PCF50603_REG_INT3M   0x11
#define PCF50603_REG_D1C     0x12
#define PCF50603_REG_D2C     0x13
#define PCF50603_REG_D3C     0x14
#define PCF50603_REG_HCC     0x15
#define PCF50603_REG_IOC     0x16
#define PCF50603_REG_LPC     0x17
#define PCF50603_REG_RF1C    0x18
#define PCF50603_REG_RF2C    0x19
#define PCF50603_REG_CPC     0x1A
#define PCF50603_REG_BVMC    0x1B
#define PCF50603_REG_PWM1C   0x1C
#define PCF50603_REG_PWM2C   0x1D
#define PCF50603_REG_LED1C   0x1E
#define PCF50603_REG_LED2C   0x1F
#define PCF50603_REG_GPOC1   0x20
#define PCF50603_REG_GPOC2   0x21
#define PCF50603_REG_SIMC1   0x22
#define PCF50603_REG_SIMC2   0x23
#define PCF50603_REG_CHGC1   0x24
#define PCF50603_REG_CHGC2   0x25
#define PCF50603_REG_CHGS1   0x26
#define PCF50603_REG_BBCC    0x27
#define PCF50603_REG_ID      0x28
#define PCF50603_REG_OCRCAL  0x29
#define PCF50603_REG_CHGCAL  0x2A
#define PCF50603_REG_DEBC    0x2B
#define PCF50603_REG_CHGC3   0x2C

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* PMU_PCF50603_H */

