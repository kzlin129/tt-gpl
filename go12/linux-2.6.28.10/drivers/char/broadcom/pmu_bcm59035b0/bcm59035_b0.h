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
*  bcm59035_b0.h
*
*  PURPOSE:
*
*  This file defines the internal interface to the Broadcom BCM59035 PMU chip
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM59035_B0_H )
#define BCM59035_B0_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59035_b0.h>

/* ---- Constants and Types ---------------------------------------------- */

/* interrupt IDs from 0 to 80 */
typedef enum {
    BCM59035_IRQID_INT1_PONKEYR,					   
    BCM59035_IRQID_INT1_PONKEYF,					   
    BCM59035_IRQID_INT1_PONKEYH,					   
    BCM59035_IRQID_INT1_RTC60S,						   
    BCM59035_IRQID_INT1_RTCA1,						   
    BCM59035_IRQID_INT1_RSVD_BIT5,					   
    BCM59035_IRQID_INT1_RTCADJ,						   
    BCM59035_IRQID_INT1_RTC1S,						   

    BCM59035_IRQID_INT2_CHGINS,			 
    BCM59035_IRQID_INT2_CHGRM,			 
    BCM59035_IRQID_INT2_CHGERR,			 
    BCM59035_IRQID_INT2_CHGEOC,			 
    BCM59035_IRQID_INT2_USBINS,			 
    BCM59035_IRQID_INT2_USBRM,			 
    BCM59035_IRQID_INT2_USBERR,			 
    BCM59035_IRQID_INT2_MBCCHGERR,		 
										 
    BCM59035_IRQID_INT3_ACDINS,			
    BCM59035_IRQID_INT3_ACDRM,			 
    BCM59035_IRQID_INT3_PHFDRLS,					   
    BCM59035_IRQID_INT3_PHFDINS,					   
    BCM59035_IRQID_INT3_PHFDRM,						   
    BCM59035_IRQID_INT3_PHFDPRS,					   
    BCM59035_IRQID_INT3_LOWBAT,						   
    BCM59035_IRQID_INT3_BBLOWB,						   
													   
    BCM59035_IRQID_INT4_A1OVRI,		 
    BCM59035_IRQID_INT4_A20VRI,		 
    BCM59035_IRQID_INT4_R1OVRI,		 
    BCM59035_IRQID_INT4_R2OVRI,		 
    BCM59035_IRQID_INT4_H1OVRI,		 
    BCM59035_IRQID_INT4_H2OVRI,    	 
    BCM59035_IRQID_INT4_M1OVRI,		 
    BCM59035_IRQID_INT4_M2OVRI,		     
													         
    BCM59035_IRQID_INT5_LOVRI,		        
    BCM59035_IRQID_INT5_LV1OVRI,	          
    BCM59035_IRQID_INT5_LV2OVRI,	   
    BCM59035_IRQID_INT5_IOVRI,		      
    BCM59035_IRQID_INT5_SOVRI,		   
    BCM59035_IRQID_INT5_AX1OVRI,	   	   
    BCM59035_IRQID_INT5_AX2OVRI,	   
    BCM59035_IRQID_INT5_RESV_BIT7,	   													   
													   
	BCM59035_IRQID_INT6_IOSROVRI,											   
	BCM59035_IRQID_INT6_CSROVRI, 
	BCM59035_IRQID_INT6_IOSROVRV,									
	BCM59035_IRQID_INT6_CSROVRV, 									        
	BCM59035_IRQID_INT6_FGC,     									        
	BCM59035_IRQID_INT6_TOOWARM, 									        
	BCM59035_IRQID_INT6_RESV_BIT6,											    
	BCM59035_IRQID_INT6_RESV_BIT7,										    
				  													    
	BCM59035_IRQID_INT7_MBTEMPFAULT, 
	BCM59035_IRQID_INT7_MBTEMPLOW,   												       
	BCM59035_IRQID_INT7_MBTEMPHIGH,  												       
	BCM59035_IRQID_INT7_MBRM,        												       
	BCM59035_IRQID_INT7_MBOV,        												       
	BCM59035_IRQID_INT7_NOBAT,       												       
	BCM59035_IRQID_INT7_BATINS,      												       
	BCM59035_IRQID_INT7_LOWBAT,      												       
													       
	BCM59035_IRQID_INT8_VBUSVALID,   
	BCM59035_IRQID_INT8_A_SESSVALID, 												       
	BCM59035_IRQID_INT8_B_SESSEND,   												       
	BCM59035_IRQID_INT8_ID_INSRT,   
	BCM59035_IRQID_INT8_RESV_BIT4,											    
	BCM59035_IRQID_INT8_RESV_BIT5,
	BCM59035_IRQID_INT8_RESV_BIT6,										     												       
	BCM59035_IRQID_INT8_RESUME_VBUS, 												       
													       
	BCM59035_IRQID_INT9_VBUSVALID,   
	BCM59035_IRQID_INT9_A_SESSVALID, 												       
	BCM59035_IRQID_INT9_B_SESSEND,   												       
	BCM59035_IRQID_INT9_ID_RMV,      												       
	BCM59035_IRQID_INT9_RESV_BIT4,											    
	BCM59035_IRQID_INT9_RESV_BIT5,
	BCM59035_IRQID_INT9_RESV_BIT6,										     												       
	BCM59035_IRQID_INT9_RESV_BIT7, 
													       
	BCM59035_IRQID_INT10_VBOVRI,     
	BCM59035_IRQID_INT10_VBOV,       												       
	BCM59035_IRQID_INT10_RESV_BIT2,											    
	BCM59035_IRQID_INT10_RESV_BIT3,
	BCM59035_IRQID_INT10_RESV_BIT4,										     												       
	BCM59035_IRQID_INT10_RESV_BIT5, 
	BCM59035_IRQID_INT10_RESV_BIT6,										     												       
	BCM59035_IRQID_INT10_RESV_BIT7, 

	BCM59035_TOTAL_IRQ

} BCM59035_InterruptId_t;

typedef void (*bcm59035_isr_t)(BCM59035_InterruptId_t irq_id);

typedef struct
{
   int available;     // boolean flag indicating availability of regulator
   int programmable;  // boolean flag indicating programmability of regulator
   u8  reg_addr;      // address of regulator control register for mode control
   u8  reg_addr_volt; // address of control register to change voltage
   u32 min_mV;        // minimum voltage in mV
   u32 max_mV;        // maximum voltage in mV
   u32 mV_step;       // programmable voltage step size in mV
   u32 vout_mask;     // Mask of bits in register
   u32 vout_shift;    // Bit shift in register
   u32 *vout_to_mV_map; // Map for converting register voltage to register value
   u32 map_size;      // Size of register map
} bcm59035_regulator_map_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
int bcm59035_irq_register(BCM59035_InterruptId_t irqId, bcm59035_isr_t isrFunction);

#endif  /* BCM59035_H */

