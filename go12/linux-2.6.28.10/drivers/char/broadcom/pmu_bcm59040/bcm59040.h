/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
*  bcm59040.h
*
*  PURPOSE:
*
*  This file defines the internal interface to the Broadcom BCM59040 PMU chip
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM59040_H )
#define BCM59040_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>

/* ---- Constants and Types ---------------------------------------------- */

/* interrupt IDs from 0 to 80 */
typedef enum {
    BCM59040_IRQID_INT1_PONKEYR,					   
    BCM59040_IRQID_INT1_PONKEYF,					   
    BCM59040_IRQID_INT1_PONKEYH,					   
    BCM59040_IRQID_INT1_PONKEYBHD,						   
    BCM59040_IRQID_INT1_RESTARTH,						   
    BCM59040_IRQID_INT1_HBINT,					   
    BCM59040_IRQID_INT1_PMUTOOWARM,						   
#ifdef PMU_59040_B0
    BCM59040_IRQID_INT1_RESTARTON,
#else
    BCM59040_IRQID_INT1_RSVD_BIT7,						   
#endif

    BCM59040_IRQID_INT2_CHGINS,			 
    BCM59040_IRQID_INT2_CHGRM,			 
    BCM59040_IRQID_INT2_CHGOVERV,			 
    BCM59040_IRQID_INT2_EOC,			 
    BCM59040_IRQID_INT2_USBINS,			 
    BCM59040_IRQID_INT2_USBRM,			 
    BCM59040_IRQID_INT2_USBOVERV,			 
    BCM59040_IRQID_INT2_CHGDET,		 
										 
    BCM59040_IRQID_INT3_VSROVERV,			
    BCM59040_IRQID_INT3_VSROVERI,			 
    BCM59040_IRQID_INT3_VCHGRNOTOK,					   
#ifdef PMU_59040_B0					   
    BCM59040_IRQID_INT3_CHG_WDT_ALARM,
#else
    BCM59040_IRQID_INT3_VCHGRCLSP,					   
#endif					   
    BCM59040_IRQID_INT3_VBUSLOWBND,						   
    BCM59040_IRQID_INT3_CHGERRDIS,					   
    BCM59040_IRQID_INT3_CHGWDTEXP,						   
    BCM59040_IRQID_INT3_IDOVERI,						   
													   
    BCM59040_IRQID_INT4_LDO1OVRI,		 
    BCM59040_IRQID_INT4_LDO20VRI,		 
    BCM59040_IRQID_INT4_LDO3OVRI,		 
    BCM59040_IRQID_INT4_LDO4OVRI,		 
    BCM59040_IRQID_INT4_LDO5OVRI,		 
    BCM59040_IRQID_INT4_LDO6OVRI,    	 
    BCM59040_IRQID_INT4_BBLOW,		 
    BCM59040_IRQID_INT4_FGC,		     
													         
    BCM59040_IRQID_INT5_IOSROVRI,		        
    BCM59040_IRQID_INT5_CSROVRI,	          
    BCM59040_IRQID_INT5_IOSROVRV,	   
    BCM59040_IRQID_INT5_CSROVRV,		      
    BCM59040_IRQID_INT5_RTCADJ,		   
    BCM59040_IRQID_INT5_RTC1S,	   	   
    BCM59040_IRQID_INT5_RTC60S,	   
    BCM59040_IRQID_INT5_RTCA1,	   													   
													   
	BCM59040_IRQID_INT6_MBTEMPFAULT,											   
	BCM59040_IRQID_INT6_MBTEMPLOW, 
	BCM59040_IRQID_INT6_MBTEMPHIGH,									
	BCM59040_IRQID_INT6_MBRM, 									        
	BCM59040_IRQID_INT6_MBOV,     									        
	BCM59040_IRQID_INT6_BATINS, 									        
	BCM59040_IRQID_INT6_LOWBAT,											    
	BCM59040_IRQID_INT6_VERYLOWBAT,										    
				  													    
	BCM59040_IRQID_INT7_VBUS_VALID_F, 
	BCM59040_IRQID_INT7_A_SESSVALID_F,   												       
	BCM59040_IRQID_INT7_B_SESSEND_F,  												       
	BCM59040_IRQID_INT7_ID_INSRT,        												       
	BCM59040_IRQID_INT7_VBUS_VALID_R,        												       
	BCM59040_IRQID_INT7_A_SESSVALID_R,       												       
	BCM59040_IRQID_INT7_B_SESSEND_R,      												       
	BCM59040_IRQID_INT7_ID_RMV,      												       
													       
	BCM59040_IRQID_INT8_SARCONVRDY0,   
	BCM59040_IRQID_INT8_SARCONVRDY1, 												       
	BCM59040_IRQID_INT8_SARCONVRDY2,   												       
	BCM59040_IRQID_INT8_SARCONVRDY3,   
	BCM59040_IRQID_INT8_SARCONVRDY4,											    
	BCM59040_IRQID_INT8_SARCONVRDY5,
	BCM59040_IRQID_INT8_SARCONVRDY6,										     												       
	BCM59040_IRQID_INT8_SARCONVRDY7, 												       
													       
	BCM59040_IRQID_INT9_SARCONVRDY8,   
	BCM59040_IRQID_INT9_SARCONVRDY9, 												       
	BCM59040_IRQID_INT9_SARCONVEND,   												       
	BCM59040_IRQID_INT9_SARCONTCONVFAIL,      												       
	BCM59040_IRQID_INT9_SARASYNCONVOFF,											    
	BCM59040_IRQID_INT9_SARASYNREQFAIL,
	BCM59040_IRQID_INT9_RESUME_VBUS,										     												       
	BCM59040_IRQID_INT9_ID_CHNG, 
													       
	BCM59040_TOTAL_IRQ

} BCM59040_InterruptId_t;

typedef void (*bcm59040_isr_t)(BCM59040_InterruptId_t irq_id);

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
} bcm59040_regulator_map_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
int bcm59040_irq_register(BCM59040_InterruptId_t irqId, bcm59040_isr_t isrFunction);

#endif  /* BCM59040 */

