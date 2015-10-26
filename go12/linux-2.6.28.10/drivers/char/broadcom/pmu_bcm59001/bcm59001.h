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
*  bcm59001.h
*
*  PURPOSE:
*
*  This file defines the internal interface to the Broadcom BCM59001 PMU chip
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM59001_H )
#define BCM59001_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59001.h>

/* ---- Constants and Types ---------------------------------------------- */

/* interrupt IDs from 0 to 40 */
typedef enum {
    BCM59001_IRQID_INT1_PONKEYR,
    BCM59001_IRQID_INT1_PONKEYF,
    BCM59001_IRQID_INT1_PONKEYH,
    BCM59001_IRQID_INT1_RTC60S,
    BCM59001_IRQID_INT1_RTCA1,
    BCM59001_IRQID_INT1_RSVD_BIT5,
    BCM59001_IRQID_INT1_RTCADJ,
    BCM59001_IRQID_INT1_RTC1S,

    BCM59001_IRQID_INT2_CHGINS,
    BCM59001_IRQID_INT2_CHGRM,
    BCM59001_IRQID_INT2_CHGERR,
    BCM59001_IRQID_INT2_CHGEOC,
    BCM59001_IRQID_INT2_USBINS,
    BCM59001_IRQID_INT2_USBRM,
    BCM59001_IRQID_INT2_USBEOC,
    BCM59001_IRQID_INT2_MBCCHGERR,

    BCM59001_IRQID_INT3_ACDINS,
    BCM59001_IRQID_INT3_ACDRM,
    BCM59001_IRQID_INT3_PHFDRLS,
    BCM59001_IRQID_INT3_PHFDINS,
    BCM59001_IRQID_INT3_PHFDRM,
    BCM59001_IRQID_INT3_PHFDPRS,
    BCM59001_IRQID_INT3_LOWBAT,
    BCM59001_IRQID_INT3_RSVD_BIT7,

    BCM59001_IRQID_INT4_A1OVRI,
    BCM59001_IRQID_INT4_A20VRI,
    BCM59001_IRQID_INT4_R1OVRI,
    BCM59001_IRQID_INT4_R2OVRI,
    BCM59001_IRQID_INT4_HOVRI,
    BCM59001_IRQID_INT4_UOVRI,
    BCM59001_IRQID_INT4_IOVRI,
    BCM59001_IRQID_INT4_MOVRI,

    BCM59001_IRQID_INT5_LOVRI,
    BCM59001_IRQID_INT5_LVOVRI,
    BCM59001_IRQID_INT5_SOVRI,
    BCM59001_IRQID_INT5_CSROVRI,
    BCM59001_IRQID_INT5_IOSROVRI,
    BCM59001_IRQID_INT5_USBERR,
    BCM59001_IRQID_INT5_RESV_BIT6,
    BCM59001_IRQID_INT5_RESV_BIT7,

} BCM59001_InterruptId_t;

typedef void (*bcm59001_isr_t)(BCM59001_InterruptId_t irq_id);

typedef struct
{
   int available;     // boolean flag indicating availability of regulator
   int programmable;  // boolean flag indicating programmability of regulator
   u8  reg_addr;      // address of regulator control register
   u32 min_mV;        // minimum voltage in mV
   u32 max_mV;        // maximum voltage in mV
   u32 mV_step;       // programmable voltage step size in mV
} bcm59001_regulator_map_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
int bcm59001_irq_register(BCM59001_InterruptId_t irqId, bcm59001_isr_t isrFunction);

#endif  /* BCM59001_H */

