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
*  pmu_pcf506xx.h
*
*  PURPOSE:
*
*  This file contains common defintions to the Philips PCF506xx series of PMU chips
*
*  NOTES:
*
*****************************************************************************/


#if !defined( PMU_PCF506XX_H )
#define PMU_PCF506XX_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_pcf50603.h>
#include <linux/broadcom/pmu_pcf50611.h>

/* ---- Constants and Types ---------------------------------------------- */

/* Regulator IDs */

typedef enum {
   PCF506XX_REGULATOR_D1 = 0,
   PCF506XX_REGULATOR_D2,
   PCF506XX_REGULATOR_D3,
   PCF506XX_REGULATOR_HC,
   PCF506XX_REGULATOR_CP,
   PCF506XX_REGULATOR_IO,
   PCF506XX_REGULATOR_LP,
   PCF506XX_REGULATOR_RF1,
   PCF506XX_REGULATOR_RF2,
   PCF506XX_REGULATOR_DCD,
   PCF506XX_REGULATOR_LC,
   PCF506XX_NUM_REGULATORS
} PCF506XX_regulators_t;

/* Charger IDs */
typedef enum {
   PCF506XX_CHARGER_MAIN = 0,
   PCF506XX_CHARGER_USB,
   PCF506XX_NUM_CHARGERS
} PCF506XX_chargers_t;

/* Register addresses for direct accesses */

#define PCF506XX_REG_INT1(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_INT1:PCF50611_REG_INT1)
#define PCF506XX_REG_INT2(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_INT2:PCF50611_REG_INT2)
#define PCF506XX_REG_INT3(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_INT3:PCF50611_REG_INT3)
#define PCF506XX_REG_OOCC(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_OOCC:PCF50611_REG_OOCC)
#define PCF506XX_REG_OOCS1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_OOCS1:PCF50611_REG_OOCS1)
#define PCF506XX_REG_OOCS2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_OOCS2:PCF50611_REG_OOCS2)
#define PCF506XX_REG_RTC1(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RTC1:PCF50611_REG_RTC1)
#define PCF506XX_REG_RTC2(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RTC2:PCF50611_REG_RTC2)
#define PCF506XX_REG_RTC3(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RTC3:PCF50611_REG_RTC3)
#define PCF506XX_REG_RTC4(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RTC4:PCF50611_REG_RTC4)
#define PCF506XX_REG_RTC1A(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_RTC1A:PCF50611_REG_RTC1A)
#define PCF506XX_REG_RTC2A(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_RTC2A:PCF50611_REG_RTC2A)
#define PCF506XX_REG_RTC3A(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_RTC3A:PCF50611_REG_RTC3A)
#define PCF506XX_REG_RTC4A(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_RTC4A:PCF50611_REG_RTC4A)
#define PCF506XX_REG_REC2C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_REC2C:PCF50611_REG_REC2C)
#define PCF506XX_REG_INT1M(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_INT1M:PCF50611_REG_INT1M)
#define PCF506XX_REG_INT2M(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_INT2M:PCF50611_REG_INT2M)
#define PCF506XX_REG_INT3M(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_INT3M:PCF50611_REG_INT3M)
#define PCF506XX_REG_D1C(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_D1C:PCF50611_REG_D1C)
#define PCF506XX_REG_D2C(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_D2C:PCF50611_REG_D2C)
#define PCF506XX_REG_D3C(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_D3C:PCF50611_REG_D3C)
#define PCF506XX_REG_HCC(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_HCC:PCF50611_REG_HCC)
#define PCF506XX_REG_IOC(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_IOC:PCF50611_REG_IOC)
#define PCF506XX_REG_LPC(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_LPC:PCF50611_REG_LPC)
#define PCF506XX_REG_RF1C(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RF1C:PCF50611_REG_RF1C)
#define PCF506XX_REG_RF2C(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_RF2C:PCF50611_REG_RF2C)
#define PCF506XX_REG_CPC(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_CPC:PCF50611_REG_CPC)
#define PCF506XX_REG_BVMC(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_BVMC:PCF50611_REG_BVMC)
#define PCF506XX_REG_LED1C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_LED1C:PCF50611_REG_LED1C)
#define PCF506XX_REG_LED2C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_LED2C:PCF50611_REG_LED2C)
#define PCF506XX_REG_GPOC1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_GPOC1:PCF50611_REG_GPOC1)
#define PCF506XX_REG_GPOC2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_GPOC2:PCF50611_REG_GPOC2)
#define PCF506XX_REG_SIMC1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_SIMC1:PCF50611_REG_SIMC1)
#define PCF506XX_REG_SIMC2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_SIMC2:PCF50611_REG_SIMC2)
#define PCF506XX_REG_BBCC(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_BBCC:PCF50611_REG_BBCC)
#define PCF506XX_REG_ID(chip)        ((chip == PMU_PCF50603)?PCF50603_REG_ID:PCF50611_REG_ID)
#define PCF506XX_REG_DEBC(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_DEBC:PCF50611_REG_DEBC)
#define PCF506XX_REG_PWM1C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM1C:PCF50611_REG_PWM1C)
#define PCF506XX_REG_PWM2C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM2C:PCF50611_REG_PWM2C)
#define PCF506XX_REG_CHGC1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CHGC1:PCF50611_REG_CHGC1)
#define PCF506XX_REG_CHGC2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CHGC2:PCF50611_REG_CHGC2)
#define PCF506XX_REG_CHGS1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CHGS1:PCF50611_REG_CHGS1)
#define PCF506XX_REG_OCRCAL(chip)    ((chip == PMU_PCF50603)?PCF50603_REG_OCRCAL:PCF50611_REG_OCRCAL)
#define PCF506XX_REG_CHGCAL(chip)    ((chip == PMU_PCF50603)?PCF50603_REG_CHGCAL:PCF50611_REG_CHGCAL)
#define PCF506XX_REG_CHGC3(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CHGC3:PCF50611_REG_CHGC3)
#define PCF506XX_REG_INT4(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_INT4:PCF50611_REG_INT4)
#define PCF506XX_REG_REC1C(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_REC1C:PCF50611_REG_REC1C)
#define PCF506XX_REG_INT4M(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_INT4M:PCF50611_REG_INT4M)
#define PCF506XX_REG_DCDC1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC1:PCF50611_REG_DCDC1)
#define PCF506XX_REG_DCDC2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC2:PCF50611_REG_DCDC2)
#define PCF506XX_REG_DCDC3(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC3:PCF50611_REG_DCDC3)
#define PCF506XX_REG_DCDC4(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC4:PCF50611_REG_DCDC4)
#define PCF506XX_REG_DCDC5(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC5:PCF50611_REG_DCDC5)
#define PCF506XX_REG_PWM1S(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM1S:PCF50611_REG_PWM1S)
#define PCF506XX_REG_PWM1D(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM1D:PCF50611_REG_PWM1D)
#define PCF506XX_REG_LEDCC(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_LEDCC:PCF50611_REG_LEDCC)
#define PCF506XX_REG_CBCC1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC1:PCF50611_REG_CBCC1)
#define PCF506XX_REG_CBCC2(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC2:PCF50611_REG_CBCC2)
#define PCF506XX_REG_CBCC3(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC3:PCF50611_REG_CBCC3)
#define PCF506XX_REG_CBCC4(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC4:PCF50611_REG_CBCC4)
#define PCF506XX_REG_CBCC5(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC5:PCF50611_REG_CBCC5)
#define PCF506XX_REG_CBCC6(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCC6:PCF50611_REG_CBCC6)
#define PCF506XX_REG_CBCS1(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_CBCS1:PCF50611_REG_CBCS1)
#define PCF506XX_REG_PSSC(chip)      ((chip == PMU_PCF50603)?PCF50603_REG_PSSC:PCF50611_REG_PSSC)
#define PCF506XX_REG_ISRCAL(chip)    ((chip == PMU_PCF50603)?PCF50603_REG_ISRCAL:PCF50611_REG_ISRCAL)
#define PCF506XX_REG_VMAXCAL(chip)   ((chip == PMU_PCF50603)?PCF50603_REG_VMAXCAL:PCF50611_REG_VMAXCAL)
#define PCF506XX_REG_PWM2S(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM2S:PCF50611_REG_PWM2S)
#define PCF506XX_REG_PWM2D(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_PWM2D:PCF50611_REG_PWM2D)
#define PCF506XX_REG_DCDC6(chip)     ((chip == PMU_PCF50603)?PCF50603_REG_DCDC6:PCF50611_REG_DCDC6)
#define PCF506XX_REG_LCC(chip)       ((chip == PMU_PCF50603)?PCF50603_REG_LCC:PCF50611_REG_LCC)

/* bit position definitions within the registers */

/* INT1 */
#define PCF506XX_BIT_LOWBAT      (1 << 0)            // low battery
#define PCF506XX_BIT_ALARM       (1 << 3)            // RTC alarm time is reached
#define PCF506XX_BIT_ONKEYR      (1 << 4)            // on key rising edge
#define PCF506XX_BIT_ONKEYF      (1 << 5)            // on key falling edge
#define PCF506XX_BIT_ONKEY1S     (1 << 6)            // on key for 1 second

/* INT2 */
#define PCF506XX_BIT_REC2LF      (1 << 2)            // REC2 low falling edge
#define PCF506XX_BIT_REC2HF      (1 << 4)            // REC2 high falling edge
#define PCF506XX_BIT_REC2HR      (1 << 5)            // REC2 high rising edge
#define PCF506XX_BIT_CHGEVT      (1 << 6)            // charger status change event
#define PCF506XX_BIT_VMAX        (1 << 6)            // charger switch CC to CV
#define PCF506XX_BIT_CHGWD       (1 << 7)            // charger watchdog event

/* INT3 */
#define PCF506XX_BIT_CHGINS      (1 << 6)            // charger inserted
#define PCF506XX_BIT_CHGRM       (1 << 7)            // charger removed

/* INT4 */
#define PCF506XX_BIT_CHGRES       (1 << 0)            // battery less than Vresume
#define PCF506XX_BIT_BATFUL       (1 << 3)            // battery fully charged
#define PCF506XX_BIT_BATTMFLT     (1 << 4)            // battery temp outside window
#define PCF506XX_BIT_BATTMOK      (1 << 5)            // battery temp inside window
#define PCF506XX_BIT_UCHGRM       (1 << 6)            // battery temp inside window
#define PCF506XX_BIT_UCHGINS      (1 << 7)            // battery temp inside window

/* OOCC */
#define PCF506XX_BIT_GO_STDBY    (1 << 0)            // go to standby mode
#define PCF506XX_BIT_TOT_RST     (1 << 1)            // time out timer reset
#define PCF506XX_BIT_MICB_EN     (1 << 2)            // mic bias enable
#define PCF506XX_BIT_RTC_WAK     (1 << 4)            // RTC alarm
#define PCF506XX_BIT_REC2_EN     (1 << 5)            // REC2 enable
#define PCF506XX_BIT_REC2_SAMP   (1 << 6)            // set REC2 sampling mode

/* 00CS1 */
#define PCF506XX_BIT_REC2L       (1 << 2)            // REC2 low level
#define PCF506XX_BIT_BAT_OK      (1 << 3)            // Battery OK (VBAT > VLOW_BAT)
#define PCF506XX_BIT_CHG_OK      (1 << 5)            // Charger detected
#define PCF506XX_BIT_REC2H       (1 << 7)            // REC2 high level

/* OOCS2 */
#define PCF506XX_BIT_UCHG_OK      (1 << 3)            // USB Charger detected

/* BVMC */
#define PCF506XX_BIT_BVMC_LOWBAT (1 << 0)            // Low battery condition detected
#define PCF506XX_BIT_THRSHLD_300 (3 << 1)            // Low battery threshold = 3.00 V
#define PCF506XX_BIT_THRSHLD_320 (5 << 1)            // Low battery threshold = 3.20 V
#define PCF506XX_BIT_THRSHLD_330 (6 << 1)            // Low battery threshold = 3.30 V
#define PCF506XX_BIT_THRSHLD_340 (7 << 1)            // Low battery threshold = 3.40 V

/* LED control */
#define PCF506XX_BIT_LED_PER04   (0 << 0)            // 0.4 second repetition period
#define PCF506XX_BIT_LED_PER10   (1 << 0)            // 1.0 second repetition period
#define PCF506XX_BIT_LED_PER12   (2 << 0)            // 1.2 second repetition period
#define PCF506XX_BIT_LED_PER20   (3 << 0)            // 2.0 second repetition period
#define PCF506XX_BIT_LED_PER26   (4 << 0)            // 2.6 second repetition period
#define PCF506XX_BIT_LED_PER40   (5 << 0)            // 4.0 second repetition period
#define PCF506XX_BIT_LED_PER60   (6 << 0)            // 6.0 second repetition period
#define PCF506XX_BIT_LED_PER80   (7 << 0)            // 8.0 second repetition period
#define PCF506XX_BIT_LED_PAT0    (0 << 3)            // on 50 ms, off
#define PCF506XX_BIT_LED_PAT1    (1 << 3)            // on 100 ms, off
#define PCF506XX_BIT_LED_PAT2    (2 << 3)            // on 200 ms, off
#define PCF506XX_BIT_LED_PAT3    (3 << 3)            // on 500 ms, off
#define PCF506XX_BIT_LED_PAT4    (4 << 3)            // on 50 ms, off 50 ms, on 50 ms, off
#define PCF506XX_BIT_LED_PAT5    (5 << 3)            // on 100 ms, off 100 ms, on 100 ms, off
#define PCF506XX_BIT_LED_PAT6    (6 << 3)            // on 200 ms, off 200 ms, on 200 ms, off
#define PCF506XX_BIT_LED_PAT7    (7 << 3)            // on continuously
#define PCF506XX_BIT_LED_CHG_SET (1 << 6)            // LED enabled when charger connected
#define PCF506XX_BIT_LED_ACT_SET (1 << 7)            // LED enabled in active state

/* GPO control */
#define PCF506XX_BIT_GPO_ACTLOW  (0)                 // active low output
#define PCF506XX_BIT_GPO_LED1    (1)                 // LED1 output
#define PCF506XX_BIT_GPO_LED2    (2)                 // LED2 output
#define PCF506XX_BIT_GPO_PWM1    (3)                 // PWM1 output
#define PCF506XX_BIT_GPO_PWM1B   (4)                 // !PWM1 output
#define PCF506XX_BIT_GPO_PWM2    (5)                 // PWM2 output
#define PCF506XX_BIT_GPO_PWM2B   (6)                 // !PWM2 output
#define PCF506XX_BIT_GPO_HIGHIMP (7)                 // high impedance output

/* CHGC1 */
#define PCF506XX_BIT_CHGAPE      (1 << 0)            // enable CCCV charging
#define PCF506XX_BIT_CHGMODFST   (3 << 2)            // fast charge most CCCV
#define PCF506XX_BIT_WDTIME      (1 << 6)            // 0: no watchdog, 1: 3 minute watchdog
#define PCF506XX_BIT_BATMAXHYST  (1 << 7)            // end of charge hysteresis

/* CHGC2 */
#define PCF506XX_BIT_WDRST       (1 << 0)            // Charger watchdog reset
#define PCF506XX_BIT_VCHGCON402  (0 << 1)            // Charge control voltage = 4.02 V
#define PCF506XX_BIT_VCHGCON420  (9 << 1)            // Charge control voltage = 4.20 V
#define PCF506XX_BIT_CURRAT05    (0 << 5)            // current = 0.05 times Ifast
#define PCF506XX_BIT_CURRAT10    (1 << 5)            // current = 0.10 times Ifast
#define PCF506XX_BIT_CURRAT20    (2 << 5)            // current = 0.20 times Ifast
#define PCF506XX_BIT_CURRAT40    (3 << 5)            // current = 0.40 times Ifast

/* CHGS1 */
#define PCF506XX_BIT_VBAT_MASK   (3 << 0)            // bits (1:0)
#define PCF506XX_BIT_VBAT_LO     (0 << 0)            // Vchg < Vbatmin
#define PCF506XX_BIT_VBAT_MID    (1 << 0)            // Vbatmin < Vbat < Vchgcon
#define PCF506XX_BIT_VBAT_HI     (3 << 0)            // Vbat > Vchgcon
#define PCF506XX_BIT_WDEXP       (1 << 2)            // Watchdog expired
#define PCF506XX_BIT_VCHG_MASK   (3 << 3)            // bits (4:3)
#define PCF506XX_BIT_VCHGSTAT_LO (0 << 3)            // charger voltage too low
#define PCF506XX_BIT_VCHGSTAT_OK (1 << 3)            // charger voltage within acceptable limits
#define PCF506XX_BIT_VCHGSTAT_HI (3 << 3)            // charger voltage too high
#define PCF506XX_BIT_CHGCUR_MASK (3 << 5)            // bits (6:5)
#define PCF506XX_BIT_CHGCUR_LO   (0 << 5)            // Ichg < currat * Ifst
#define PCF506XX_BIT_CHGCUR_MID  (1 << 5)            // currat * Ifst < Ichg < 2 * Ifst
#define PCF506XX_BIT_CHGCUR_HI   (3 << 5)            // Ichg > 2 * Ifst

/* CBCC1 values*/
#define PCF506XX_BIT_CBCC1_CHGENA        (1 << 0)    // Enable main charging
#define PCF506XX_BIT_CBCC1_USBENA        (1 << 1)    // Enable USB charging
#define PCF506XX_BIT_CBCC1_AUTOCC        (1 << 2)    // Enable fast charge
#define PCF506XX_BIT_CBCC1_AUTOSTOP      (1 << 3)    // Enable autostop for charger
/* Following values of Vresume are only applicable to N1C silicon */
#define PCF506XX_BIT_CBCC1_AUTORES_6_0   (0 << 4)    // Vresume=Vmax - 6%
#define PCF506XX_BIT_CBCC1_AUTORES_4_5   (1 << 4)    // Vresume=Vmax - 4.5%
#define PCF506XX_BIT_CBCC1_AUTORES_7_5   (2 << 4)    // Vresume=Vmax - 7.5%
#define PCF506XX_BIT_CBCC1_WDRST         (1 << 6)    // Charger watchdog reset
#define PCF506XX_BIT_CBCC1_WDTIME_NOLIM  (0 << 7)    // No limit to max charging time
#define PCF506XX_BIT_CBCC1_WDTIME_5HRS   (1 << 7)    // Set watchdog time to 5 hours

/* CBCC2 values*/
#define PCF506XX_BIT_CBCC2_REVERSE       (1 << 0)    // reverse mode enable
#define PCF506XX_BIT_CBCC2_VBATCOND_2_2  (0 << 1)    // Vbatcond = 2.2V
#define PCF506XX_BIT_CBCC2_VBATCOND_3_0  (1 << 1)    // Vbatcond = 3.0V
#define PCF506XX_BIT_CBCC2_VBATCOND_3_3  (2 << 1)    // Vbatcond = 3.3V
#define PCF506XX_BIT_CBCC2_VMAX_4_10V    (0x06 << 3) // VMAX @4.10V
#define PCF506XX_BIT_CBCC2_VMAX_4_20V    (0x0c << 3) // VMAX @4.2V
#define PCF506XX_BIT_CBCC2_VMAX_4_28V    (0x0f << 3) // VMAX @4.28V
#define PCF506XX_BIT_CBCC2_VMAX_4_30V    (0x10 << 3) // VMAX @4.30V
#define PCF506XX_BIT_CBCC2_VMAX_4_65V    (0x11 << 3) // VMAX @4.65V

/* CBCC3 values */
#define PCF506XX_BIT_CBCC3_VAL    (0xff)            // Set to max charging current

/* CBCC4 values */
#define PCF506XX_BIT_CBCC4_VAL    (0xff)            // Set to max charging current

/* CBCC5 values*/
#define PCF506XX_BIT_CBCC5_TRICKLE_MASK  (0x1f << 0)
#define PCF506XX_BIT_CBCC5_TRICKLE_SHIFT 0

/* CBCS1 */
#define PCF506XX_BIT_BATFUL_MASK     (1 << 0)
#define PCF506XX_BIT_WDEXP_MASK      (1 << 2)
#define PCF506XX_BIT_WDEXP           (1 << 2)
#define PCF506XX_BIT_ILMT_MASK       (1 << 3)
#define PCF506XX_BIT_ILMT_LO         (1 << 3)
#define PCF506XX_BIT_ILMT_HI         (0 << 3)
#define PCF506XX_BIT_VLMT_MASK       (1 << 4)
#define PCF506XX_BIT_VLMT_HI         (1 << 4)
#define PCF506XX_BIT_RESSTAT_MASK    (1 << 7)

/* BBCC */
#define PCF506XX_BIT_BBCE        (1 << 0)            // Backup Battery Charger Enable
#define PCF506XX_BIT_BBCR        (1 << 1)            // bypass output resistor
#define PCF506XX_BIT_BBCC50      (0 << 2)            // charging current = 50 microamps
#define PCF506XX_BIT_BBCC100     (1 << 2)            // charging current = 100 microamps
#define PCF506XX_BIT_BBCC200     (2 << 2)            // charging current = 200 microamps
#define PCF506XX_BIT_BBCC400     (3 << 2)            // charging current = 400 microamps
#define PCF506XX_BIT_BBCV        (1 << 4)            // limiting voltage (0=2.5V, 1=3.0V)

/* DEBC */
#define PCF506XX_BIT_ONKEYDB0    (0)                 // no debounce
#define PCF506XX_BIT_ONKEYDB14   (1)                 // 14 ms debounce
#define PCF506XX_BIT_ONKEYDB62   (2)                 // 62 ms debounce
#define PCF506XX_BIT_ONKEYDB500  (3)                 // 500 ms debounce
#define PCF506XX_BIT_ONKEYDB1000 (4)                 // 1000 ms debounce
#define PCF506XX_BIT_ONKEYDB2000 (5)                 // 2000 ms debounce

/* regulator settings (V_OUT and CPV_OUT) */
#define PCF506XX_BIT_REG_VOUT_MASK  (0x1F)           // output voltage mask
#define PCF506XX_BIT_REG_VOUT_SHIFT 0                // output voltage shift

#define PCF506XX_BIT_REGV_1_8    (0x09)              // 1.8 volts
#define PCF506XX_BIT_REGV_2_9    (0x14)              // 2.9 volts
#define PCF506XX_BIT_REGV_3_0    (0x15)              // 3.0 volts
#define PCF506XX_BIT_REGV_3_2    (0x17)              // 3.2 volts

#define PCF506XX_BIT_REG_CPV_MASK  (0x03)            // charge pump output voltage mask
#define PCF506XX_BIT_REG_CPV_SHIFT 0                 // charge pump output voltage shift

#define PCF506XX_BIT_REG_CPV_3_5 (0)                 // 3.5 volts for charge pump
#define PCF506XX_BIT_REG_CPV_4_0 (1)                 // 4.0 volts for charge pump
#define PCF506XX_BIT_REG_CPV_4_5 (2)                 // 4.5 volts for charge pump
#define PCF506XX_BIT_REG_CPV_5_0 (3)                 // 5.0 volts for charge pump

#define PCF506XX_BIT_LCV_2_9     (0x1F)              // 2.9 volts for LC

#define PCF506XX_BIT_REG_OPMOD_MASK   (7 << 5)       // operation mode mask
#define PCF506XX_BIT_REG_OPMOD_SHIFT  (5)            // operation mode shift

#define PCF506XX_BIT_REG_ON      (7 << 5)            // supply always on during active mode
#define PCF506XX_BIT_REG_OFF     (0 << 5)            // turn regulator off
#define PCF506XX_BIT_REG_ECO     (2 << 5)            // use regulator in eco mode
#define PCF506XX_BIT_REG_SLP_OFF (4 << 5)            // turn off supply during sleep
#define PCF506XX_BIT_REG_SLP_ECO (1 << 5)            // use eco mode during sleep
#define PCF506XX_BIT_RST_ACT     (1 << 1)            // reset LP in standby

/* SIMC1 */
#define PCF506XX_BIT_SIMC1_SIMUP  (1 << 0)            // Card supply voltage
#define PCF506XX_BIT_SIMC1_SIMMOD (1 << 1)            // SIM operation mode
#define PCF506XX_BIT_SIMC1_IOEN   (1 << 3)            // Enable communication from SIMIOCD
#define PCF506XX_BIT_SIMC1_STRT   (1 << 4)            // SIM start action according to operation mode
#define PCF506XX_BIT_SIMC1_PD     (1 << 5)            // Power down mode

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* PMU_PCF506XX_H */

