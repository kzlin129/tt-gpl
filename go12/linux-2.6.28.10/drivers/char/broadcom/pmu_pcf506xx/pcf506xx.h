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
*  pcf506xx.h
*
*  PURPOSE:
*
*  This file defines the internal interface to the Philips PCF506xx PMU chips.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( PCF506XX_H )
#define PCF506XX_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_pcf50603.h>
#include <linux/broadcom/pmu_pcf50611.h>

/* ---- Constants and Types ---------------------------------------------- */

#define PCF506XX_NUM_IRQS(chip)   ( ( chip == PMU_PCF50603 ) ? 24 : \
                                  ( ( chip == PMU_PCF50611 ) ? 32 : 0 ) )
#define PCF506XX_MAX_NUM_IRQS     32

#define PCF506XX_NUM_INT_REGS(chip)   ( ( chip == PMU_PCF50603 ) ? 3 : \
                                             ( ( chip == PMU_PCF50611 ) ? 4 : 0 ) )
#define PCF506XX_MAX_NUM_INT_REGS     4

/* interrupt IDs */
typedef enum {
   PCF506XX_IRQID_INT1LOWBAT,
   PCF506XX_IRQID_INT1SECOND,
   PCF506XX_IRQID_INT1MINUTE,
   PCF506XX_IRQID_INT1ALARM,
   PCF506XX_IRQID_INT1ONKEYR,
   PCF506XX_IRQID_INT1ONKEYF,
   PCF506XX_IRQID_INT1ONKEY1S,
   PCF506XX_IRQID_INT1THS,

   PCF506XX_IRQID_INT2REC1R,
   PCF506XX_IRQID_INT2REC1F,
   PCF506XX_IRQID_INT2REC2LF,
   PCF506XX_IRQID_INT2REC2LR,
   PCF506XX_IRQID_INT2REC2HF,
   PCF506XX_IRQID_INT2REC2HR,
   PCF506XX_IRQID_INT2CHGEVT,
   PCF506XX_IRQID_INT2CHGWD,

   PCF506XX_IRQID_INT3SIMUV,
   PCF506XX_IRQID_INT3INSERT,
   PCF506XX_IRQID_INT3EXTRACT,
   PCF506XX_IRQID_INT3MUTE,
   PCF506XX_IRQID_INT3EARLY,
   PCF506XX_IRQID_INT3SIMRDY,
   PCF506XX_IRQID_INT3CHGINS,
   PCF506XX_IRQID_INT3CHGRM,

   // INT4 does not apply to 50603
   PCF506XX_IRQID_INT4CHGRES,
   PCF506XX_IRQID_INT4THLIMON,
   PCF506XX_IRQID_INT4THLIMOFF,
   PCF506XX_IRQID_INT4BATFUL,
   PCF506XX_IRQID_INT4BATTMFLT,
   PCF506XX_IRQID_INT4BATTMOK,
   PCF506XX_IRQID_INT4UCHGRM,
   PCF506XX_IRQID_INT4UCHGINS,

   PCF506XX_TOTAL_IRQ

} PCF506XX_InterruptId_t;

typedef void (*pcf506xx_isr_t)(PCF506XX_InterruptId_t irq_id);

typedef struct
{
   int available;     // boolean flag indicating availability of regulator
   int programmable;  // boolean flag indicating programmability of regulator
   u8  reg_addr;      // address of regulator control register
   u32 min_mV;        // minimum voltage in mV
   u32 max_mV;        // maximum voltage in mV
   u32 mV_step;       // programmable voltage step size in mV
} pcf506xx_regulator_map_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

/* Common initialization routine */
int pcf506xx_common_init(BCM_PMU_Chip_t chip);

/* Interrupt handling functions */
int pcf506xx_irq_init(BCM_PMU_Chip_t chip, u8 *initial_int_status);
int pcf506xx_irq_register(BCM_PMU_Chip_t chip, PCF506XX_InterruptId_t irqId, pcf506xx_isr_t isrFunction);

/* Interrupt service routine */
irqreturn_t pcf506xx_isr(BCM_PMU_Chip_t chip);

/* Reset 8 second watchdog timer */
void pcf506xx_reset8Second(BCM_PMU_Chip_t chip);

/* IOCTL handler */
int pcf506xx_ioctl(BCM_PMU_Chip_t chip, struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
void pcf506xx_poweroff(BCM_PMU_Chip_t chip);

/* Power regulator control */
int pcf506xx_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod);
int pcf506xx_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state);
int pcf506xx_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step);
int pcf506xx_vout_to_mV(int regulatorID, u8 vout, u32 *mV);

#endif  /* PCF506XX_H */

