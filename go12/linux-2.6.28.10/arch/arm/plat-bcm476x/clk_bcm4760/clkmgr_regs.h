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
 *   @file   clkmgr_regs.h
 * 
 *   @brief  Register defs for clock manager.
 * 
 ****************************************************************************/

#ifndef _CLKMGR_REGS_H_
#define _CLKMGR_REGS_H_

/**
    INCLUDE FILES
 */
#include "bcm_basetypes.h"        
#include "bcm_reg.h"

//--------------------------------------------------
/**
     0x0000   System PLL Control/Status Register      (CMPLLS).
     0x0004   AUX1 PLL Control/Status Register        (CMPLL1).
     0x0008   AUX2 PLL Control/Status Register        (CMPLL2).  
     0x000C   AUX3 PLL Control/Status Register        (CMPLL3).  

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct
{                           
                            /** Bits           Access     POR  */
    uint32_t    pdiv  : 5;  /**< Bits 4-0       RW     POR(1)  */  
    uint32_t    rsvd  : 3;  /**< Bits 7-5       RO     POR(0)  */  
    uint32_t    qdiv  : 9;  /**< Bits 16-8      RW     POR(1)  */   
    uint32_t    vcor  : 1;  /**< Bits 17        RW     POR(0)  */  
    uint32_t    enab  : 1;  /**< Bits 18        RW     POR(0)  */  
    uint32_t    lock  : 1;  /**< Bits 19        RO     POR(0)  */  
    uint32_t    rsvd1 : 12; /**< Bits 31-20     RO     POR(0)  */ 
} clkmgr_cmpll_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmpll);


//--------------------------------------------------
/**
    0x0010   System Clock Control Register           (CMSYS)   

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct
{                            
                             /** Bits           Access    POR  */
    uint32_t    div    : 3;  /**< Bits 2-0       RW     POR(1) */    
    uint32_t    rsvd   : 1;  /**< Bits 3         RO     POR(0) */  
    uint32_t    csrc   : 3;  /**< Bits 6-4       RW     POR(0) */  
    uint32_t    rsvd1  : 1;  /**< Bits 7         RO     POR(0) */  
    uint32_t    ahdiv  : 6;  /**< Bits 13-8      RW     POR(1) */  
    uint32_t    rsvd2  : 2;  /**< Bits 15-14     RO     POR(0) */  
    uint32_t    vhdiv  : 6;  /**< Bits 21-16     RW     POR(1) */  
    uint32_t    rsvd3  : 2;  /**< Bits 23-22     RO     POR(0) */ 
    uint32_t    emidiv : 2;  /**< Bits 25-24     RW     POR(1) */ 
    uint32_t    rsvd4  : 2;  /**< Bits 27-26     RO     POR(0) */    
    uint32_t    mtxdiv : 2;  /**< Bits 29-28     RW     POR(1) */
    uint32_t    rsvd5  : 2;  /**< Bits 31-30     RO     POR(0) */ 
} clkmgr_cmsys_t;                               
                                                        
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32_MASK(clkmgr_cmsys, 0x333F3F77)

//--------------------------------------------------
/**
    0x0014   AHB Peripheral Control Register         (CMAHB)   

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                             /** Bits           Access     POR  */
    uint32_t    lmsel : 1;   /**< Bits 0          RW     POR(0) */ 
    uint32_t    smsel : 1;   /**< Bits 1          RW     POR(0) */
    uint32_t    rsvd  : 2;   /**< Bits 3-2        RO     POR(0) */
    uint32_t    apdiv : 2;   /**< Bits 5-4        RW     POR(2) */
    uint32_t    rsvd1 : 2;   /**< Bits 7-6        RO     POR(0) */
    uint32_t    spdiv : 2;   /**< Bits 9-8        RW     POR(2) */
    uint32_t    rsvd2 : 2;   /**< Bits 11-10      RO     POR(0) */
    uint32_t    amdiv : 4;   /**< Bits 15-12      RW     POR(3) */
    uint32_t    nrdiv : 2;   /**< Bits 17-16      RW     POR(1) */
    uint32_t    rsvd3 : 14;  /**< Bits 31-18      RO     POR(0) */
}clkmgr_cmahb_t;                                      
                                                              
/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmahb);

//--------------------------------------------------
/**
    0x0018   Timer Clock 0 Control Register          (CMTIM0).  
    0x001C   Timer Clock 1 Control Register          (CMTIM1).  
    0x0020   PWM Clock Control Register              (CMPWM) .  
                                                             
    @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                             /** Bits           Access     POR  */
    uint32_t    div  : 8;    /**< Bits 7-0        RW     POR(1) */
    uint32_t    csrc : 3;    /**< Bits 10-8       RW     POR(0) */
    uint32_t    rsvd : 21;   /**< Bits 31-11      RO     POR(0) */
}clkmgr_cmtim_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmtim);


//--------------------------------------------------
/**
    0x0024   UART0 Clock Control Register            (CMUART0). 
    0x0028   UART1 Clock Control Register            (CMUART1). 
    0x002C   UART2 Clock Control Register            (CMUART2). 
    0x0030   CVD UART Clock Control Register         (CMUART). 
    xxxxxx Hole here, see next register, cmarm xxxxxxxxxxxxxx.
    0x0038   Crypto Clock Control Register           (CMCRYPT). 
    0x003C   Generic Clock Control Register          (CMGEN). 
    0x0040   ARM I2C Clock Control Register          (CMARMI2C).
    0x0044   VC I2C Clock Control Register           (CMVCI2C). 
    0x0048   VFIR Clock Control Register             (CMVFIR). 
    0x004C   SPI0 Clock Control Register             (CMSPI0). 
    0x0050   SPI1 Clock Control Register             (CMSPI1). 
    0x0054   ARM I2S Clock Control Register          (CMARMI2S).
    0x0058   CVD I2S Clock Control Register          (CMCVDI2S).
    0x005C   VC I2S Clock Control Register           (CMVCI2S).
    0x0060   SDIO0 Clock Control Register            (CMSDIO0).
    0x0064   SDIO1 Clock Control Register            (CMSDIO1). 
    0x0068   LCD Clock Control Register              (CMLCD). 
    0x006C   Camera Clock Control Register           (CMCAM). 
    0x0070   Storage Clock Control Register          (CMSTOR).
    @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                              /** Bits           Access    POR  */
    uint32_t    div   : 5;    /**< Bits 4-0      RW     POR(1)  */
    uint32_t    rsvd  : 3;    /**< Bits 7-5      RO     POR(0)  */
    uint32_t    csrc  : 3;    /**< Bits 10-8     RW     POR(0)  */
    uint32_t    rsvd1 : 21;   /**< Bits 31-11    RO     POR(0)  */
}clkmgr_cmperip_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmperip);


//--------------------------------------------------
/**
    0x0034   ARM CPU Clock Control Register          (CMARM)   

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                              /** Bits         Access    POR  */
    uint32_t    div   : 5;    /**< Bits 4-0     RW     POR(1) */
    uint32_t    rsvd  : 3;    /**< Bits 7-5     RO     POR(0) */
    uint32_t    csrc  : 3;    /**< Bits 10-8    RW     POR(1) */
    uint32_t    rsvd1 : 1;    /**< Bits 11      RO     POR(0) */
    uint32_t    sync  : 1;    /**< Bits 12      RW     POR(0) */
    uint32_t    rsvd2 : 19;   /**< Bits 31-13   RO     POR(0) */
}clkmgr_cmarm_t;


/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmarm);


//--------------------------------------------------
/**
    0x0074   TWSPI Clock Control Register            (CMTWSPI) 

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                              /** Bits         Access    POR  */
    uint32_t    div  : 8;    /**< Bits 7-0     RW     POR(1)  */
    uint32_t    csrc : 3;    /**< Bits 10-8    RW     POR(0)  */
    uint32_t    rsvd : 21;   /**< Bits 31-11   RO     POR(0)  */
}clkmgr_cmtwspi_t;                            
                                   
/*                                 
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmtwspi);


//--------------------------------------------------
/**
    0x007C   Clock Request Control Status Register   (CMREQCS) 

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                              /** Bits         Access    POR   */
    uint32_t    amcreq  : 1;  /**< Bits 0       RW     POR(1)  */
    uint32_t    amcack  : 1;  /**< Bits 1       RW     POR(0)  */
    uint32_t    rsvd    : 2;  /**< Bits 3-2     R      POR(0)  */
    uint32_t    reqmode : 1;  /**< Bits 4       RW     POR(0)  */
    uint32_t    clkreq  : 1;  /**< Bits 5       RW     POR(0)  */
    uint32_t    reqstat : 1;  /**< Bits 6       R      POR(0)  */
    uint32_t    gntstat : 1;  /**< Bits 7       R      POR(0)  */
    uint32_t    dcasync : 1;  /**< Bits 8       RW     POR(0)  */
    uint32_t    rsvd1   : 23; /**< Bits 31-9    RO     POR(0)  */
}clkmgr_cmreqcs_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmreqcs);


//--------------------------------------------------
/**
    0x0080   Clock Enable Control Register 0         (CMCE0).   
    0x0088   Clock Enable Clear Register 0           (CMCEC0)

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                            /** Bits           Access    POR  */
    uint32_t ide   : 1;  /**< Bits 0          RW     POR(en:1)/POR(cl:0) */
    uint32_t ceata : 1;  /**< Bits 1          RW     POR(en:1)/POR(cl:0) */
    uint32_t nand  : 1;  /**< Bits 2          RW     POR(en:1)/POR(cl:0) */
    uint32_t usb   : 1;  /**< Bits 3          RW     POR(en:1)/POR(cl:0) */
    uint32_t sdio0 : 1;  /**< Bits 4          RW     POR(en:1)/POR(cl:0) */
    uint32_t sdio1 : 1;  /**< Bits 5          RW     POR(en:1)/POR(cl:0) */
    uint32_t vfir  : 1;  /**< Bits 6          RW     POR(en:1)/POR(cl:0) */
    uint32_t crpt  : 1;  /**< Bits 7          RW     POR(en:1)/POR(cl:0) */
    uint32_t amc   : 1;  /**< Bits 8          RW     POR(en:1)/POR(cl:0) */
    uint32_t pwm   : 1;  /**< Bits 9          RW     POR(en:1)/POR(cl:0) */
    uint32_t uart0 : 1;  /**< Bits 10         RW     POR(en:1)/POR(cl:0) */
    uint32_t uart1 : 1;  /**< Bits 11         RW     POR(en:1)/POR(cl:0) */
    uint32_t uart2 : 1;  /**< Bits 12         RW     POR(en:1)/POR(cl:0) */
    uint32_t pke   : 1;  /**< Bits 13         RW     POR(en:1)/POR(cl:0) */
    uint32_t otp   : 1;  /**< Bits 14         RW     POR(en:1)/POR(cl:0) */
    uint32_t tim   : 1;  /**< Bits 15         RW     POR(en:1)/POR(cl:0) */
    uint32_t twspi : 1;  /**< Bits 16         RW     POR(en:1)/POR(cl:0) */
    uint32_t spi0  : 1;  /**< Bits 17         RW     POR(en:1)/POR(cl:0) */
    uint32_t spi1  : 1;  /**< Bits 18         RW     POR(en:1)/POR(cl:0) */
    uint32_t intc  : 1;  /**< Bits 19         RW     POR(en:1)/POR(cl:0) */
    uint32_t rng   : 1;  /**< Bits 20         RW     POR(en:1)/POR(cl:0) */
    uint32_t rtc   : 1;  /**< Bits 21         RW     POR(en:1)/POR(cl:0) */
    uint32_t i2c   : 1;  /**< Bits 22         RW     POR(en:1)/POR(cl:0) */
    uint32_t sysm  : 1;  /**< Bits 23         RW     POR(en:1)/POR(cl:0) */
    uint32_t gpio  : 1;  /**< Bits 24         RW     POR(en:1)/POR(cl:0) */
    uint32_t pm    : 1;  /**< Bits 25         RW     POR(en:1)/POR(cl:0) */
    uint32_t i2s   : 1;  /**< Bits 26         RW     POR(en:1)/POR(cl:0) */
    uint32_t rmp   : 1;  /**< Bits 27         RW     POR(en:1)/POR(cl:0) */
    uint32_t rpc   : 1;  /**< Bits 28         RW     POR(en:1)/POR(cl:0) */
    uint32_t wdt   : 1;  /**< Bits 29         RW     POR(en:1)/POR(cl:0) */
    uint32_t rsvd    : 2;  /**< Bits 31-30    RO     POR(en:0)/POR(cl:0) */
}clkmgr_cmcearm_t;                                 

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmcearm);


//--------------------------------------------------
/**
    0x0084   Clock Enable Control Register 1         (CMCE1).   
    0x008C   Clock Enable Clear Register 1           (CMCEC1)

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                           /** Bits         Access    POR   */
    uint32_t mmtx  : 1;  /**< Bits 0       RW     POR(en:1)/POR(cl:0)  */
    uint32_t mdma  : 1;  /**< Bits 1       RW     POR(en:1)/POR(cl:0)  */
    uint32_t mrom  : 1;  /**< Bits 2       RW     POR(en:1)/POR(cl:0)  */
    uint32_t mnor  : 1;  /**< Bits 3       RW     POR(en:1)/POR(cl:0)  */
    uint32_t msram : 1;  /**< Bits 4       RW     POR(en:1)/POR(cl:0)  */
    uint32_t memi  : 1;  /**< Bits 5       RW     POR(en:1)/POR(cl:0)  */
    uint32_t mips  : 1;  /**< Bits 6       RW     POR(en:1)/POR(cl:0)  */
    uint32_t cuart : 1;  /**< Bits 7       RW     POR(en:1)/POR(cl:0)  */
    uint32_t ci2s  : 1;  /**< Bits 8       RW     POR(en:1)/POR(cl:0)  */
    uint32_t cdc   : 1;  /**< Bits 9       RW     POR(en:1)/POR(cl:0)  */
    uint32_t csmi  : 1;  /**< Bits 10      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vcpu  : 1;  /**< Bits 11      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vcam  : 1;  /**< Bits 12      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vbg   : 1;  /**< Bits 13      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vint  : 1;  /**< Bits 14      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vtim  : 1;  /**< Bits 15      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vi2c  : 1;  /**< Bits 16      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vi2s  : 1;  /**< Bits 17      RW     POR(en:1)/POR(cl:0)  */
    uint32_t vtvo  : 1;  /**< Bits 18      RW     POR(en:1)/POR(cl:0)  */
    uint32_t asnd  : 1;  /**< Bits 19      RW     POR(en:1)/POR(cl:0)  */
    uint32_t aside : 1;  /**< Bits 20      RW     POR(en:1)/POR(cl:0)  */
    uint32_t ascea : 1;  /**< Bits 21      RW     POR(en:1)/POR(cl:0)  */
    uint32_t astvo : 1;  /**< Bits 22      RW     POR(en:1)/POR(cl:0)  */
    uint32_t rsvd    : 9;  /**< Bits 31-23 RO     POR(en:0)/POR(cl:0)  */
}clkmgr_cmcevc_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmcevc);


//--------------------------------------------------
/**
    0x0090   Lock Control Register                   (CMLOCK)  

     @ingroup CLKMGR
*/
//--------------------------------------------------
typedef struct 
{
                           /** Bits         Access    POR  */
    uint32_t reglock : 1;  /**< Bits 0       RW     POR(0) */
    uint32_t rsvd    : 15; /**< Bits 15-1    RO     POR(0) */
    uint32_t passwd  : 16; /**< Bits 31-16   WO     POR(0) */
}clkmgr_cmlock_t;

/*
 * Build union data type from bitfield type and basic
 * 32bit word accessor.  <basename>_u_t
 */
AMBA_REG_BLD_UNION32(clkmgr_cmlock);


//--------------------------------------------------
/**
    This structure is the mapping of Clock Manager registers.

    @ingroup CLKMGR
*/
//--------------------------------------------------

typedef struct
{
                                           /** Offset Description                                              */
    clkmgr_cmpll_u_t     cmpll[CLKMGR_PLL_COUNT];     /**< 0x0000   System PLL Control/Status Register      (CMPLLS)      */
                                                             /**< 0x0004   AUX1 PLL Control/Status Register        (CMPLL1)      */
                                                             /**< 0x0008   AUX2 PLL Control/Status Register        (CMPLL2)      */
                                                             /**< 0x000C   AUX3 PLL Control/Status Register        (CMPLL3)      */

    /*
         Note: 

         The following peripheral related registers could 
         have been clubbed into an array. But it is not done 
         because 
         (1) cmarm has a different format and is inserted in the middle and 
         (2) some api's are applicable to both timer control register and
             peripherals and they have to differentiated anyway in the API.
             Hence, they are listed as individual registers. 
    */
    clkmgr_cmsys_u_t      cmsys;   /**< 0x0010   System Clock Control Register           (CMSYS)       */
    clkmgr_cmahb_u_t      cmahb;   /**< 0x0014   AHB Peripheral Control Register         (CMAHB)       */
    clkmgr_cmtim_u_t     cmtim0;   /**< 0x0018   Timer Clock 0 Control Register          (CMTIM0)      */
    clkmgr_cmtim_u_t     cmtim1;   /**< 0x001C   Timer Clock 1 Control Register          (CMTIM1)      */
    clkmgr_cmtim_u_t      cmpwm;   /**< 0x0020   PWM Clock Control Register              (CMPWM)       */
    clkmgr_cmperip_u_t   cmuart0;  /**< 0x0024   UART0 Clock Control Register            (CMUART0)     */
    clkmgr_cmperip_u_t   cmuart1;  /**< 0x0028   UART1 Clock Control Register            (CMUART1)     */
    clkmgr_cmperip_u_t   cmuart2;  /**< 0x002C   UART2 Clock Control Register            (CMUART2)     */
    clkmgr_cmperip_u_t    cmuart;  /**< 0x0030   CVD UART Clock Control Register         (CMUART)      */
    clkmgr_cmarm_u_t       cmarm;  /**< 0x0034   ARM CPU Clock Control Register          (CMARM)       */
    clkmgr_cmperip_u_t   cmcrypt;  /**< 0x0038   Crypto Clock Control Register           (CMCRYPT)     */
    clkmgr_cmperip_u_t     cmgen;  /**< 0x003C   Generic Clock Control Register          (CMGEN)       */
    clkmgr_cmperip_u_t  cmarmi2c;  /**< 0x0040   ARM I2C Clock Control Register          (CMARMI2C)    */
    clkmgr_cmperip_u_t   cmvci2c;  /**< 0x0044   VC I2C Clock Control Register           (CMVCI2C)     */
    clkmgr_cmperip_u_t    cmvfir;  /**< 0x0048   VFIR Clock Control Register             (CMVFIR)      */
    clkmgr_cmperip_u_t    cmspi0;  /**< 0x004C   SPI0 Clock Control Register             (CMSPI0)      */
    clkmgr_cmperip_u_t    cmspi1;  /**< 0x0050   SPI1 Clock Control Register             (CMSPI1)      */
    clkmgr_cmperip_u_t  cmarmi2s;  /**< 0x0054   ARM I2S Clock Control Register          (CMARMI2S)    */
    clkmgr_cmperip_u_t   cmvdi2s;  /**< 0x0058   CVD I2S Clock Control Register          (CMCVDI2S)    */
    clkmgr_cmperip_u_t   cmvci2s;  /**< 0x005C   VC I2S Clock Control Register           (CMVCI2S)     */
    clkmgr_cmperip_u_t   cmsdio0;  /**< 0x0060   SDIO0 Clock Control Register            (CMSDIO0)     */
    clkmgr_cmperip_u_t   cmsdio1;  /**< 0x0064   SDIO1 Clock Control Register            (CMSDIO1)     */
    clkmgr_cmperip_u_t     cmlcd;  /**< 0x0068   LCD Clock Control Register              (CMLCD)       */
    clkmgr_cmperip_u_t     cmcam;  /**< 0x006C   Camera Clock Control Register           (CMCAM)       */
    clkmgr_cmperip_u_t    cmstor;  /**< 0x0070   Storage Clock Control Register          (CMSTOR)      */

    clkmgr_cmtwspi_u_t   cmtwspi;  /**< 0x0074   TWSPI Clock Control Register            (CMTWSPI)     */
    clkmgr_cmtim_u_t     cmvctim;  /**< 0x0078   VC Timer Clock Control Register         (CMVCTIM)     */
    clkmgr_cmreqcs_u_t   cmreqcs;  /**< 0x007C   Clock Request Control Status Register   (CMREQCS)     */

    clkmgr_cmcearm_u_t     cmce0;  /**< 0x0080   Clock Enable Control Register 0         (CMCE0)       */
    clkmgr_cmcevc_u_t      cmce1;  /**< 0x0084   Clock Enable Control Register 1         (CMCE1)       */
    clkmgr_cmcearm_u_t    cmcec0;  /**< 0x0088   Clock Enable Clear Register 0           (CMCEC0)      */
    clkmgr_cmcevc_u_t     cmcec1;  /**< 0x008C   Clock Enable Clear Register 1           (CMCEC1)      */
    clkmgr_cmlock_u_t     cmlock;  /**< 0x0090   Lock Control Register                   (CMLOCK)      */                                 
    clkmgr_cmperip_u_t    cmdasyn;  /**<0x0094   Async DRAM Clock Control Register       (CMDASYN)     */

} clkmgr_regs_t;

#endif  //_CLKMGR_REGS_H_

