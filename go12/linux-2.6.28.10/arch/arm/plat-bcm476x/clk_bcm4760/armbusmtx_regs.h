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
/** 
 * 
 *   @file   armbusmtx_regs.h 
 * 
 *   @brief  The file contains the register definitions for ARM BUS Matrix
 * 
 ****************************************************************************/


#ifndef _ARMBUSMTX_REGS_H_
#define _ARMBUSMTX_REGS_H_

/**
    INCLUDE FILES
 */
#include "bcm_basetypes.h"        
#include "bcm_reg.h"

//--------------------------------------------------
/**
     Maximum number of PLL in the system
     @ingroup ARMBUSMTX
*/
//--------------------------------------------------
#define ARMBUSMTX_SYNCTOP_COUNT 5

//--------------------------------------------------
/**
     Maximum number of Timers in the Matrix arbiter
     @ingroup ARMBUSMTX
*/
//--------------------------------------------------
#define ARMBUSMTX_MARB_TIMER_COUNT      5


//--------------------------------------------------
/**
    0x200     Sync Control.    

    @ingroup ARMBUSMTX
*/
//--------------------------------------------------
typedef struct
{                            
                                           /** Bits        Access    POR  */
    uint32_t  prefetch_length     :  4;   /**< Bits 3:0     RW    POR(0) */
    uint32_t  low_watermark       :  4;   /**< Bits 7:4     RW    POR(0) */
    uint32_t  high_watermark      :  4;   /**< Bits 11:8    RW    POR(0) */
    uint32_t  rsvd                :  1;   /**< Bits 12      RW    POR(0) */
    uint32_t  latency_mode        :  1;   /**< Bits 13      RW    POR(0) */
    uint32_t  enable_posted_write :  1;   /**< Bits 14      RW    POR(0) */
    uint32_t  bypass_sync_top     :  1;   /**< Bits 15      RW    POR(1) */
    uint32_t  clock_mode_1ton     :  1;   /**< Bits 16      RW    POR(0) */
    uint32_t  clock_mode_nto1     :  1;   /**< Bits 17      RW    POR(0) */
    uint32_t  rsvd1               : 12;   /**< Bits 29:18   RW    POR(0) */
    uint32_t  clock_change        :  1;   /**< Bits 30      RW    POR(0) */
    uint32_t  soft_reset          :  1;   /**< Bits 31      RW    POR(0) */
} armbusmtx_syncctrl_t;                               
                                                      
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(armbusmtx_syncctrl);


                                                                                         
//--------------------------------------------------
/**
    This structure is the mapping of Sync top registers.

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------

typedef struct
{
                                                 /** Offset     Description                              */
 	bcm_reg32_t marb0_priority;                 /**< 0x00	   	Matrix slave 0 arbiter priority register */
       uint32_t rsvd;   		                /**< 0x04		                                         */
 	bcm_reg32_t marb0_timer[ARMBUSMTX_MARB_TIMER_COUNT];   /**< 0x08	   	Matrix slave 0 arbiter timer register 0  */
                                                /**< 0x0C	   	Matrix slave 0 arbiter timer register 1  */
                                                /**< 0x10	   	Matrix slave 0 arbiter timer register 2  */
                                                /**< 0x14	   	Matrix slave 0 arbiter timer register 3  */
                                                /**< 0x18	   	Matrix slave 0 arbiter timer register 4  */
       uint32_t rsvd1[9];      			        /**< 0x1C-0x3C  Reserved                                 */
 	bcm_reg32_t marb1_priority;                 /**< 0x40	   	Matrix slave 1 arbiter priority register */
       uint32_t rsvd2;  			            /**< 0x44		Reserved                                 */
 	bcm_reg32_t marb1_timer[ARMBUSMTX_MARB_TIMER_COUNT];   /**< 0x48	   	Matrix slave 1 arbiter timer register 0  */
 	                                            /**< 0x4C	   	Matrix slave 1 arbiter timer register 1  */
 	                                            /**< 0x50	   	Matrix slave 1 arbiter timer register 2  */
     	                                        /**< 0x54	   	Matrix slave 1 arbiter timer register 3  */
 	                                            /**< 0x58	   	Matrix slave 1 arbiter timer register 4  */
       uint32_t rsvd3[9];      	                /**< 0x5C-0x7C  Reserved                                 */
 	bcm_reg32_t marb2_priority;                 /**< 0x80	   	Matrix slave 2 arbiter priority register */
       uint32_t rsvd4;		                    /**< 0x84		Reserved                                 */
 	bcm_reg32_t marb2_timer[ARMBUSMTX_MARB_TIMER_COUNT];   /**< 0x88	   	Matrix slave 2 arbiter timer register 0  */
 	                                            /**< 0x8C	   	Matrix slave 2 arbiter timer register 1  */
 	                                            /**< 0x90	   	Matrix slave 2 arbiter timer register 2  */
 	                                            /**< 0x94	   	Matrix slave 2 arbiter timer register 3  */
 	                                            /**< 0x98	   	Matrix slave 2 arbiter timer register 4  */
       uint32_t rsvd5[9];	                    /**< 0x9C-0xBC  Reserved                                 */
 	bcm_reg32_t marb3_priority;                 /**< 0xC0	   	Matrix slave 3 arbiter priority register */
       uint32_t rsvd6;	   	                    /**< 0xC4		Reserved                                 */
 	bcm_reg32_t marb3_timer[ARMBUSMTX_MARB_TIMER_COUNT];   /**< 0xC8	   	Matrix slave 3 arbiter timer register 0  */
 	                                            /**< 0xCC	   	Matrix slave 3 arbiter timer register 1  */
                                                /**< 0xD0	   	Matrix slave 3 arbiter timer register 2  */
                                                /**< 0xD4	   	Matrix slave 3 arbiter timer register 3  */
                                	            /**< 0xD8	   	Matrix slave 3 arbiter timer register 4  */
       uint32_t rsvd7[9];      			        /**< 0xDC-0xFC  Reserved                                 */
 	bcm_reg32_t marb4_priority;	                /**< 0x100	   	Matrix slave 4 arbiter priority register */
       uint32_t rsvd8;      		            /**< 0x104		Reserved                                 */
 	bcm_reg32_t marb4_timer[ARMBUSMTX_MARB_TIMER_COUNT];   /**< 0x108	   	Matrix slave 4 arbiter timer register 0  */
                                                /**< 0x10C	   	Matrix slave 4 arbiter timer register 1  */
                                                /**< 0x110	   	Matrix slave 4 arbiter timer register 2  */
                                                /**< 0x114	   	Matrix slave 4 arbiter timer register 3  */
                                                /**< 0x118	   	Matrix slave 4 arbiter timer register 4  */
       uint32_t rsvd9[57];                      /**< 0x11C-0x1FC Reserved                                
                                                                                                         */   
    armbusmtx_syncctrl_u_t sync_ctrl[ARMBUSMTX_SYNCTOP_COUNT];  
                                                /**< 0x200       Sync Control AI                         */                                                              
                                                /**< 0x204       Sync_Control_AO                         */
                                                /**< 0x208       Sync_Control_PI                         */
                                                /**< 0x20C       Sync_Control_VI                         */
                                                /**< 0x210       Sync_Control_VO                         */                                                                 
} armbusmtx_regs_t;

#endif  //_ARMBUSMTX_REGS_H_

