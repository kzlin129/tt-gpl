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
 *   @file   chipmgr_regs.h
 * 
 *   @brief  Register defs for chip manager.
 * 
 ****************************************************************************/

#ifndef _CHIPMGR_REGS_H_
#define _CHIPMGR_REGS_H_

/**
    INCLUDE FILES
 */
#include "bcm_basetypes.h"        
#include "bcm_reg.h"

//--------------------------------------------------
/**
     Maximum number of PLL in the system
     @ingroup CHIPMGR
*/
//--------------------------------------------------
#define CHIPMGR_PLL_COUNT 4

//--------------------------------------------------
/**
     Maximum number of Pin share controls in the system
     @ingroup CHIPMGR
*/
//--------------------------------------------------
#define CHIPMGR_PIN_SHARE_CTRL_COUNT  10


//--------------------------------------------------
/**
     0x000C         BCM2820 Strap option.  

     @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                           
                            /** Bits           Access     POR  */
    uint32_t    lcd_oe    : 1; /**< Bits 0      RO        N/A  */
    uint32_t    lcd_vsync : 1; /**< Bits 1      RO        N/A  */
    uint32_t    lcd_hsync : 1; /**< Bits 2      RO        N/A  */
    uint32_t    rsvd      : 29;/**< Bits 31-3   RO        N/A  */ 
} chipmgr_strap_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_strap);


//--------------------------------------------------
/**
    0x0014         Chip Peripheral Control 0.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
                                       /** Bits        Access    POR  */
    uint32_t    startnsleep       : 1; /**< Bits 0       RW    POR(0) */
    uint32_t    dmac_wait_cycle0  : 1; /**< Bits 1       RW    POR(0) */    
    uint32_t    dmac_wait_cycle1  : 1; /**< Bits 2       RW    POR(0) */    
    uint32_t    stby_10mbit       : 1; /**< Bits 3       RW    POR(0) */
    uint32_t    enable_10mbit     : 1; /**< Bits 4       RW    POR(0) */
    uint32_t    sms_awt           : 1; /**< Bits 5       RW    POR(0) */  
    uint32_t    xtal24_xtalpd     : 1; /**< Bits 6       RW    POR(0) */  
    uint32_t    xtal27_xtalpd     : 1; /**< Bits 7       RW    POR(0) */ 
    uint32_t    uart_arm_stby     : 4; /**< Bits 11-8    RW    POR(0) */
    uint32_t    spi0_en           : 1; /**< Bits 12      RW    POR(0) */
    uint32_t    spi1_en           : 1; /**< Bits 13      RW    POR(0) */
    uint32_t    tw_spi_en         : 1; /**< Bits 14      RW    POR(0) */
    uint32_t    otp_clk_div_sel   : 1; /**< Bits 15      RW    POR(0) */
    uint32_t    otp_iddq_en       : 1; /**< Bits 16      RW    POR(0) */
    uint32_t    rng_iddq_en       : 1; /**< Bits 17      RW    POR(0) */
    uint32_t    emi_hw_selfref_ent: 1; /**< Bits 18      RW    POR(0) */
    uint32_t    emi_hw_pwrdn_exit : 1; /**< Bits 19      RW    POR(0) */
    uint32_t    clk_emi_eq_dram   : 1; /**< Bits 20      RW    POR(0) */
    uint32_t    clkmode_2by3      : 1; /**< Bits 21      RW    POR(0) */
    
	uint32_t	VREG_1P2_sel      : 4; /**< Bits 22-25   RW    POR(0) */
	uint32_t	VREG_2P5_sel      : 4; /**< Bits 26-29   RW    POR(0) */
	uint32_t    emi_hib_mode      : 1; /**< Bits 21      RW    POR(0) */
	uint32_t    emi_clk_req_extend : 1;/**< Bits 21      RW    POR(0) */
} chipmgr_perictrl0_t;                               
                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_perictrl0);


//--------------------------------------------------
/**
    0x0018         Chip Peripheral Control 1.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
                                       /** Bits        Access    POR  */
    uint32_t    en_boundary_scan  : 1; /**< Bits 0       RW    POR(0) */
    uint32_t    rsvd              :29; /**< Bits 29-1    RW    POR(0) */
    uint32_t    disable_jtag_bsd  : 2; /**< Bits 31-31   RO    POR(0) */
} chipmgr_perictrl1_t;                               
                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_perictrl1);



//--------------------------------------------------
/**
    0x0020         USB Control.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
                                              /** Bits        Access    POR  */
    uint32_t    usb_phy_pll_clksel_i     : 1; /**< Bits 0       RW    POR(0) */
    uint32_t    usb_phy_tp_mux_sel_i     : 5; /**< Bits 5-1     RW    POR(0) */
    uint32_t    usb_phy_tp_phy_sel_i     : 4; /**< Bits 9-6     RW    POR(0) */
    uint32_t    usb_phy_tp_port_sel_i    : 4; /**< Bits 13-10   RW    POR(0) */
    uint32_t    usb_phy_pll_bypass       : 1; /**< Bits 14      RW    POR(0) */
    uint32_t    usb_phy_iddq_en          : 1; /**< Bits 15      RW    POR(0) */
    uint32_t    usb_phy_resetb_i         : 1; /**< Bits 16      RW    POR(1) */
    uint32_t    usb_phy_reset_hi_usb_pll : 1; /**< Bits 17      RW    POR(0) */
    uint32_t    usb_pll_suspend_en_i     : 1; /**< Bits 18      RW    POR(0) */
    uint32_t    usb_host_mode            : 1; /**< Bits 19      RW    POR(0) */
    uint32_t    rsvd                     :12; /**< Bits 31-20   RW    POR(0) */
} chipmgr_usbctrl_t;                               

//--------------------------------------------------
/**
    0x0024         IIS Control Register.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
                                            /** Bits        Access    POR  */
    uint32_t    arm_bclk_ctrl  : 2;         /**< Bits 1-0    RW     POR(0) */
    uint32_t    rsvd           : 1;         /**< Bits 2      RO     POR(0) */
    uint32_t    aud_bclk_ctrl  : 2;         /**< Bits 4-3    RW     POR(0) */
    uint32_t    rsvd1          : 1;         /**< Bits 5      RO     POR(0) */
    uint32_t    bt_bclk_ctrl   : 2;         /**< Bits 7-6    RW     POR(0) */
    uint32_t    rsvd2          : 8;         /**< Bits 15-8   RO     POR(0) */
    uint32_t    emi_dll_rst_counter : 16;   /**< Bits 31-16  RW     N/A    */

    /** Note: The register documentation is incorrect */
    //uint32_t    aud_da_fifo_en : 1; /**< Bits 16     RO     POR(0) */
    //uint32_t    arm_da_fifo_en : 1; /**< Bits 17     RO     POR(0) */
    //uint32_t    bt_da_fifo_en  : 1; /**< Bits 18     RO     POR(0) */
    //uint32_t    rsvd3          :13; /**< Bits 0      RW     POR(0) */                
} chipmgr_iisctrl_t;                               
                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_iisctrl);

                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_usbctrl);


//--------------------------------------------------
/**
    0x0014         Chip Peripheral Control 0.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
                               /** Bits        Access    POR   */
    uint32_t    ps_gpio0  : 2; /**< Bits 1-0    RW    POR(0/1) */
    uint32_t    ps_gpio1  : 2; /**< Bits 3-2    RW    POR(0/1) */
    uint32_t    ps_gpio2  : 2; /**< Bits 5-4    RW    POR(0/1) */
    uint32_t    ps_gpio3  : 2; /**< Bits 7-6    RW    POR(0/1) */
    uint32_t    ps_gpio4  : 2; /**< Bits 9-8    RW    POR(0/1) */
    uint32_t    ps_gpio5  : 2; /**< Bits 11-10  RW    POR(0/1) */
    uint32_t    ps_gpio6  : 2; /**< Bits 13-12  RW    POR(0/1) */
    uint32_t    ps_gpio7  : 2; /**< Bits 15-14  RW    POR(0/1) */
    uint32_t    ps_gpio8  : 2; /**< Bits 17-16  RW    POR(0/1) */
    uint32_t    ps_gpio9  : 2; /**< Bits 19-18  RW    POR(0/1) */
    uint32_t    ps_gpio10 : 2; /**< Bits 21-20  RW    POR(0/1) */
    uint32_t    ps_gpio11 : 2; /**< Bits 23-22  RW    POR(0/1) */
    uint32_t    ps_gpio12 : 2; /**< Bits 25-24  RW    POR(0/1) */
    uint32_t    ps_gpio13 : 2; /**< Bits 27-26  RW    POR(0/1) */
    uint32_t    ps_gpio14 : 2; /**< Bits 29-28  RW    POR(0/1) */
    uint32_t    ps_gpio15 : 2; /**< Bits 31-30  RW    POR(0/1) */
} chipmgr_pin_ctrl_t;                               
                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(chipmgr_pin_ctrl);

/**
  @TODO :  Note that not all registers have been defined as bit field structures.
           Bit field definitions will be added as and when the registers need to
           be accessed.
 */


//--------------------------------------------------
/**
    0x003C-0x48         PLL Contro 0-3.    

    @ingroup CHIPMGR
*/
//--------------------------------------------------
typedef struct
{                            
	uint32_t    rsvd0					: 16; 
	uint32_t    bypen_p					: 2;
    uint32_t    rsvd1					: 14; 
} chipmgr_pll_ctrl_t;                               

AMBA_REG_BLD_UNION32(chipmgr_pll_ctrl);

//--------------------------------------------------
/**
    This structure is the mapping of Chip manager registers.

    @ingroup CHIPMGR
*/
//--------------------------------------------------

typedef struct
{
                                                /** Offset Description                           */               
    bcm_reg32_t                   chip_rev_id;                  
                                                  /**< 0x0000         CHIP_REV_ID                  */
    bcm_reg32_t                   vc2_part_code_version;                    
                                                  /**< 0x0004         VC2 Manuf_Part_code_version  */
    bcm_reg32_t                   vc2_proc_version;                         
                                                  /**< 0x0008         VC2 Proc_version             */
    chipmgr_strap_u_t     bcm2820_strap_option;                     
                                                  /**< 0x000C         BCM2820 Strap option         */
    bcm_reg32_t                   cust_logic_ints;                          
                                                  /**< 0x0010         CUSTOMER LOGIC Interrupts    */
    chipmgr_perictrl0_u_t chip_perip_ctrl0;                         
                                                  /**< 0x0014         Chip Peripheral Control 0    */
    chipmgr_perictrl1_u_t chip_perip_ctrl1;                         
                                                  /**< 0x0018         Chip Peripheral Control 1    */
    bcm_reg32_t                   rsvd;                                     
                                                  /**< 0x001C         Reserved                     */
    chipmgr_usbctrl_u_t   usb_ctrl;                                 
                                                  /**< 0x0020         USB Control                  */
    chipmgr_iisctrl_u_t   iis_ctrl;                                 
                                                  /**< 0x0024         IIS Control Register         */
    bcm_reg32_t                   rsvd1[5];                                 
                                                  /**< 0x0028-0x0038  Reserved                     */
    chipmgr_pll_ctrl_u_t  pll_ctrl[CHIPMGR_PLL_COUNT];                
                                                  /**< 0x003C-0x0048  PLL Control Registers        */
    bcm_reg32_t                   xtal_ctrl;                                
                                                  /**< 0x004C         Xtal Control Register        */
    bcm_reg32_t                   combo_pad_select[2];                      
                                                  /**< 0x0050-0x0054  Combo Pads Select Registers  */
    bcm_reg32_t                   pin_share_ctrl[CHIPMGR_PIN_SHARE_CTRL_COUNT];
                                                  /**< 0x0058-0x007C  Pin Share Control Registers  */
    bcm_reg32_t                   security_ctrl;                             
                                                  /**< 0x0080         Security Control Register    */
    bcm_reg32_t                   security_sts;                              
                                                  /**< 0x0084         Security Status Register     */                                                                   
} chipmgr_regs_t;

//--------------------------------------------------
/**
    Returns the base address for chip manager config registers

    @retval   base address for chip manager registers

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
chipmgr_regs_t * get_chipmgr_base_address(void);

#endif  //_CHIPMGR_REGS_H_

