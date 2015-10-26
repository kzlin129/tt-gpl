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
 *   @file   clkmgr.h 
 * 
 *   @brief  clock manager low level apis
 * 
 ****************************************************************************/

#ifndef _CLKMGR_IFC_H_
#define _CLKMGR_IFC_H_

#include "bcm_basetypes.h"
#include "bcm_privtypes.h"
#include "bcm_os_support.h"

//--------------------------------------------------
/**
     24MHZ frequency
     @ingroup CLKMGR
*/
//--------------------------------------------------
#define CLKMGR_XTAL_24MHZ_FREQUENCY	(24000000UL)

//--------------------------------------------------
/**
     27MHZ frequency
     @ingroup CLKMGR
*/
//--------------------------------------------------
#define CLKMGR_XTAL_27MHZ_FREQUENCY	(27000000UL)

//--------------------------------------------------
/**
     32MHZ frequency
     @ingroup CLKMGR
*/
//--------------------------------------------------
#define CLKMGR_XTAL_32KHZ_FREQUENCY	(32000UL)

//--------------------------------------------------
/**
    @def  CLKMGR_CLK_BLK_ENBCLR
    @brief  Enables the bit for the block using the block as index into the word
 */
//--------------------------------------------------
#define CLKMGR_CLK_BLK_ENBCLR(cur_reg_val, blk) (cur_reg_val | (0x00000001 << (blk)))


enum 
{
    ECLKMGR_CLK_USE_FEW_SRC = 0,  //Exclude 27MHz crystal and 32KHz crystal
    ECLKMGR_CLK_USE_ALL_SRC
};

#define CLKMGR_DIV_MIN                 1 

#define CLKMGR_PLL_DIV_MIN             CLKMGR_DIV_MIN

#define CLKMGR_ARMCLK_DIV_MAX          32
#define CLKMGR_TIMCLK_DIV_MAX          256
#define CLKMGR_TWSPICLK_DIV_MAX        256
#define CLKMGR_PERICLK_DIV_MAX         32
#define CLKMGR_PLL_PDIV_MAX            32
#define CLKMGR_PLL_QDIV_MAX            512

/* the 2820 PLLs can go as low as 150Mhz, but it seems its better not
 * to run them at so low a frequency. We have arbitrarilt chosen a 
 * cut of of 200 Mhz
 */
#define CLKMGR_PLL_FREQ_MIN				150000000
#define CLKMGR_PLL_VCO_CUTOFF_FREQ		250000000


//--------------------------------------------------
/**
    @enum clkmgr_csrc_t

    Clock source for ARM CPU clock and async peripherals.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_csrc_t
{
	ECLKMGR_CSRC_NONE=0,         /**< 0 */
	ECLKMGR_CSRC_XTAL_24MHZ=1,   /**< 1 */
	ECLKMGR_CSRC_PLLS=4,         /**< 4 */
	ECLKMGR_CSRC_PLL1=5,         /**< 5 */
	ECLKMGR_CSRC_PLL2=6,         /**< 6 */
	ECLKMGR_CSRC_PLL3=7,         /**< 7 */
    CLKMGR_CSRC_MAX=7
} clkmgr_csrc_t;


//--------------------------------------------------
/**
    @enum clkmgr_tcsrc_t

    Clock source for timer clocks and PWM clock.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_tcsrc_t
{
	ECLKMGR_TCSRC_NONE=0,              /**< 0 */
	ECLKMGR_TCSRC_XTAL_24MHZ=1,        /**< 1 */
	ECLKMGR_TCSRC_XTAL_27MHZ=2,        /**< 2 */
	ECLKMGR_TCSRC_XTAL_32KHZ=3,       /**< 3 */
	ECLKMGR_TCSRC_PLLS=4,              /**< 4 */
	ECLKMGR_TCSRC_PLL1=5,              /**< 5 */
	ECLKMGR_TCSRC_PLL2=6,              /**< 6 */
	ECLKMGR_TCSRC_PLL3=7,              /**< 7 */
    ECLKMGR_MAX_TCSRC=7
} clkmgr_tcsrc_t;

#define CLKMGR_TCSRC_COUNT	(ECLKMGR_MAX_TCSRC+1)

// corresponds to CMSYS:EMIDIV field possible values
typedef enum _clkmgr_emidiv_t
{
	ECLKMGR_EMIDIV_NONE=0, //invalid
	ECLKMGR_EMIDIV_1_1=1, //EMI(MTX) clock = SDRAM clock
	ECLKMGR_EMIDIV_3_2=2, //EMI(MTX) clock = 1.5 * SDRAM clock
	ECLKMGR_EMIDIV_2_1=3, //EMI(MTX) clock = 2 * SDRAM clock
} clkmgr_emidiv_t;

            
//--------------------------------------------------
/**
    @enum clkmgr_pll_t

    Different PLLs in the system.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_pll_t
{
    ECLKMGR_CMPLLS = 0,    /**< 0 */
    ECLKMGR_CMPLL1 = 1,    /**< 1 */
    ECLKMGR_CMPLL2 = 2,    /**< 2 */
    ECLKMGR_CMPLL3 = 3,     /**< 3 */
    ECLKMGR_MAX_PLL = 3,
} clkmgr_pll_t;

#define CLKMGR_PLL_COUNT	(ECLKMGR_MAX_PLL+1)

//--------------------------------------------------
/**

    @enum clkmgr_arm_block_t

    Different peripherals on the ARM side whose clock can be enabled or disabled.

    Note: The enum will be used to modify registers and hence do not muck around
          with it. This enum should match the bit fields in the register definition
          file for clock manager, which in turn will map to CMCE0 and CMCEC0 in the
          bcm2820 data sheet.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_arm_block_t
{
      ECLKMGR_CM_IDE = 0x00000001,   /**< 0x0000 0001 */
    ECLKMGR_CM_CEATA = 0x00000002,   /**< 0x0000 0002 */
     ECLKMGR_CM_NAND = 0x00000004,   /**< 0x0000 0004 */
      ECLKMGR_CM_USB = 0x00000008,   /**< 0x0000 0008 */

    ECLKMGR_CM_SDIO0 = 0x00000010,   /**< 0x0000 0010 */
    ECLKMGR_CM_SDIO1 = 0x00000020,   /**< 0x0000 0020 */
     ECLKMGR_CM_VFIR = 0x00000040,   /**< 0x0000 0040 */
     ECLKMGR_CM_CRPT = 0x00000080,   /**< 0x0000 0080 */

      ECLKMGR_CM_AMC = 0x00000100,   /**< 0x0000 0100 */
      ECLKMGR_CM_PWM = 0x00000200,   /**< 0x0000 0200 */
    ECLKMGR_CM_UART0 = 0x00000400,   /**< 0x0000 0400 */
    ECLKMGR_CM_UART1 = 0x00000800,   /**< 0x0000 0800 */

    ECLKMGR_CM_UART2 = 0x00001000,   /**< 0x0000 1000 */
      ECLKMGR_CM_PKE = 0x00002000,   /**< 0x0000 2000 */
      ECLKMGR_CM_OTP = 0x00004000,   /**< 0x0000 4000 */
      ECLKMGR_CM_TIM = 0x00008000,   /**< 0x0000 8000 */

    ECLKMGR_CM_TWSPI = 0x00010000,   /**< 0x0001 0000 */
     ECLKMGR_CM_SPI0 = 0x00020000,   /**< 0x0002 0000 */
     ECLKMGR_CM_SPI1 = 0x00040000,   /**< 0x0004 0000 */
     ECLKMGR_CM_INTC = 0x00080000,   /**< 0x0008 0000 */

      ECLKMGR_CM_RNG = 0x00100000,   /**< 0x0010 0000 */
      ECLKMGR_CM_RTC = 0x00200000,   /**< 0x0020 0000 */
      ECLKMGR_CM_I2C = 0x00400000,   /**< 0x0040 0000 */
     ECLKMGR_CM_SYSM = 0x00800000,   /**< 0x0080 0000 */

     ECLKMGR_CM_GPIO = 0x01000000,   /**< 0x0100 0000 */
       ECLKMGR_CM_PM = 0x02000000,   /**< 0x0200 0000 */
      ECLKMGR_CM_I2S = 0x04000000,   /**< 0x0400 0000 */
      ECLKMGR_CM_RMP = 0x08000000,   /**< 0x0800 0000 */

      ECLKMGR_CM_RPC = 0x10000000,   /**< 0x1000 0000 */
      ECLKMGR_CM_WDT = 0x20000000,   /**< 0x2000 0000 */
}clkmgr_arm_block_t;


//--------------------------------------------------
/**

    @enum clkmgr_vc_block_t

    Different peripherals on the VC side whose clock can be enabled or disabled.

    Note: The enum will be used to modify registers and hence do 
          not muck around with it. This enum should match the bit
          fields in the register definition file for clock manager, 
          which in turn will map to CMCE1 and CMCEC1 registers
          in the bcm2820 data sheet.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_vc_block_t
{
     ECLKMGR_CM_MMTX  = 0x00000001,    /**<0x0000 0001 */
     ECLKMGR_CM_MDMA  = 0x00000002,    /**<0x0000 0002 */
     ECLKMGR_CM_MROM  = 0x00000004,    /**<0x0000 0004 */
     ECLKMGR_CM_MNOR  = 0x00000008,    /**<0x0000 0008 */

     ECLKMGR_CM_MSRAM  = 0x00000010,    /**<0x0000 0010 */
     ECLKMGR_CM_MEMI  = 0x00000020,    /**<0x0000 0020 */
     ECLKMGR_CM_MIPS  = 0x00000040,    /**<0x0000 0040 */
     ECLKMGR_CM_CUART  = 0x00000080,    /**<0x0000 0080 */

     ECLKMGR_CM_CI2S  = 0x00000100,    /**<0x0000 0100 */
     ECLKMGR_CM_CDC   = 0x00000200,    /**<0x0000 0200 */
     ECLKMGR_CM_CSMI  = 0x00000400,    /**<0x0000 0400 */
     ECLKMGR_CM_VCPU  = 0x00000800,    /**<0x0000 0800 */

     ECLKMGR_CM_VCAM  = 0x00001000,    /**<0x0000 1000 */
     ECLKMGR_CM_VBG   = 0x00002000,    /**<0x0000 2000 */
     ECLKMGR_CM_VINT  = 0x00004000,    /**<0x0000 4000 */
     ECLKMGR_CM_VTIM  = 0x00008000,    /**<0x0000 8000 */

     ECLKMGR_CM_VI2C  = 0x00010000,    /**<0x0001 0000 */
     ECLKMGR_CM_VI2S  = 0x00020000,    /**<0x0002 0000 */
     ECLKMGR_CM_VTVO  = 0x00040000,    /**<0x0004 0000 */

//The following three peripherals are ARM related and are spilled over form CMCE(C)0 register
//into CMCE(C)1 registers.
     ECLKMGR_CM_ASND  = 0x00080000,    /**<0x0008 0000 */

     ECLKMGR_CM_ASIDE = 0x00100000,   /**< 0x0010 0000 */
     ECLKMGR_CM_ASCEA = 0x00200000,   /**< 0x0020 0000 */

     ECLKMGR_CM_ASTVO = 0x04000000,   /**< 0x0400 0000 */
    
}clkmgr_vc_block_t;


//--------------------------------------------------
/**
    @enum clkmgr_blk_t

    Enum list of peripherals whose clock frequencies can be modified

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_blk_t
{
    ECLKMGR_CMUART0 = 0,/**<  0 */
    ECLKMGR_CMUART1,    /**<  1 */
    ECLKMGR_CMUART2,    /**<  2 */
    ECLKMGR_CMUART,     /**<  3 */
    ECLKMGR_CMCRYPT,    /**<  4 */
    ECLKMGR_CMGEN,      /**<  5 */
    ECLKMGR_CMARMI2C,   /**<  6 */
    ECLKMGR_CMVCI2C,    /**<  7 */
    ECLKMGR_CMVFIR,     /**<  8 */
    ECLKMGR_CMSPI0,     /**<  9 */
    ECLKMGR_CMSPI1,     /**< 10 */
    ECLKMGR_CMARMI2S,   /**< 11 */
    ECLKMGR_CMCVDI2S,   /**< 12 */
    ECLKMGR_CMVCI2S,    /**< 13 */
    ECLKMGR_CMSDIO0,    /**< 14 */
    ECLKMGR_CMSDIO1,    /**< 15 */
    ECLKMGR_CMLCD,      /**< 16 */
    ECLKMGR_CMCAM,      /**< 17 */
    ECLKMGR_CMSTOR,     /**< 18 */
    ECLKMGR_CMDASYN,    /**< 19 */
    ECLKMGR_CMTIM0,     /**< 20 */
    ECLKMGR_CMTIM1,     /**< 21 */
    ECLKMGR_CMPWM,      /**< 22 */
    ECLKMGR_CMVCTIM,    /**< 23 */  

    ELKMGR_BLK_MAX = ECLKMGR_CMVCTIM
} clkmgr_blk_t;

#define CLKMGR_BLK_COUNT	(ELKMGR_BLK_MAX+1)

//--------------------------------------------------
/**
    @enum clkmgr_pll_t

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_master_src_t
{
    ECLKMGR_SEL_ARM_ACCESS = 0,    /**< 0 */
    ECLKMGR_SEL_VC_ACCESS = 1,    /**< 1 */
} clkmgr_sel_access_t;

//--------------------------------------------------
/**
    @enum clkmgr_dram_mode_t

    DRAM Clock mode. 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
typedef enum _clkmgr_dram_mode_t
{
    ECLKMGR_DRAMCLK_SYNC_TO_EMICLK  = 0,    /**< 0 */
    ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK = 1,    /**< 1 */
} clkmgr_dram_mode_t;

/**
    @struct  clkmgr_pll_settings_t
*/
typedef struct _clkmgr_pll_settings_t
{
    uint32_t      qdiv;   //qdiv
    uint32_t      pdiv;   //pdiv
}clkmgr_pll_settings_t;

/**
    @struct  clkmgr_freq_info_t

    used as input param instead of passing too many vars to a function call
 */
typedef struct _clkmgr_freq_info_t
{
    unsigned long desired_freq;
    uint32_t max_div;
    int_t tolerance_percent; 
}clkmgr_freq_info_t;


#define CLKMGR_CSRC_RANGE(blk, val)        \
        if (blk <= ECLKMGR_CMSTOR)          \
             val = ECLKMGR_CLK_USE_FEW_SRC; \
        else                        \
             val = ECLKMGR_CLK_USE_ALL_SRC; \

/**
    @struct  clkmgr_arm_clk_settings_t
*/
typedef struct _clkmgr_arm_clk_settings_t_
{
    clkmgr_tcsrc_t csrc;
    uint32_t div;    
}clkmgr_arm_clk_settings_t;

/**
    @struct  clkmgr_cmsys_clk_settings_t
*/
typedef struct _clkmgr_cmsys_clk_settings_t_
{ 
    clkmgr_tcsrc_t csrc;
    uint32_t div;
    uint32_t mtxdiv;
    uint32_t emidiv;
    uint32_t ahdiv;
    uint32_t vhdiv;
}clkmgr_cmsys_clk_settings_t;


/**
    @struct  clkmgr_cmsys_freqs_t
*/
typedef struct _clkmgr_cmsys_freqs_t_
{ 
    unsigned long mtx_freq;
    unsigned long vc_freq;
    unsigned long armahb_freq;
}clkmgr_cmsys_freqs_t;


/**
    @struct  clkmgr_cmahb_clk_settings_t
*/
typedef struct _clkmgr_cmahb_clk_settings_t_
{ 
    uint32_t amdiv;
    uint32_t apdiv;
    uint32_t nrdiv;
    uint32_t spdiv;
}clkmgr_cmahb_clk_settings_t;

/**
    @struct  clkmgr_cmahb_freqs_t
*/
typedef struct _clkmgr_cmahb_freqs_t_
{ 
    unsigned long ap_freq;
    unsigned long sp_freq;
    unsigned long am_freq;
    unsigned long nr_freq;
}clkmgr_cmahb_freqs_t;

//--------------------------------------------------
/**
    Initialize clock manager.

    @param  Block virtual address

    @special
        This initialization routine currently initializes the base 
        address for the clock manager block. 

    @retval   None

    @ingroup  CLKMGR
*/
//--------------------------------------------------
void clkmgr_api_init(void);


//--------------------------------------------------
/**
    Enable clock manager register write access.

    @param  None

    @special
        It is assumed that the caller invokes this API before modifying any
        registers in the clock manager module. After completing the writes,
        the caller should invoke the clkmgr_lock_regs function
        which will enable a bit in the register to prevent anyone from changing
        the contents.

        Here is more information from the datasheet:

        The Lock control register provides protection against accidental
        writes to the clock manager registers. When the REGLOCK bit is 
        set to 1, a write from the APB interface to any register in the
        Clock Manager (except the Lock Control Register itself) is blocked.
        
        Read to the registers are not affected by the REGLOCK bit. When 
        the REGLOCK bit is cleared to 0, all the registers can be read and
        written through the APB interface. The REGLOCK bit itself is protected
        by the PASSWD field. All writes to the REGLOCK bit must have "A5A5" 
        in the PASSWD field for the write to take place.

    @see  clkmgr_lock_regs()

    @retval   None

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_unlock_regs(void);


//--------------------------------------------------
/**
     Disable clock manager hardware register writes.

    @param  None

    @special
        None

    @see clkmgr_unlock_regs()

    @retval   None

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_lock_regs(void);


//--------------------------------------------------
/**
    Enable the hardware clock switching mode.

    @param  None

    @special
        Used during clock switching.

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_hw_clock_switch_mode(void);


//--------------------------------------------------
/**
    Wait for completion of clock switching process.
	This is implemented by polling for CMREQCS:REQSTAT bit to be set to 0

    @param  None

    @special
        Used during clock switching.

    @retval   N/A
    
    @ingroup  CLKMGR
*/
//--------------------------------------------------
void clkmgr_wait_for_clock_switch_completion(void);


//--------------------------------------------------
/**
    Set PLL to desired frequency.

    @param  pll             which pll?
    @param  desired_freq    
    @param  tolerance_percent
    @param  actual_freq

    @special
        The function will attempt to return the closest possible frequency. 

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed - Unable to get desired frequency

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_pll_frequency(clkmgr_pll_t pll, uint32_t desired_freq,
                                   int tolerance_percent,
                                   uint32_t *actual_freq);

//--------------------------------------------------
/**
    Enable PLL.

    @param  pll     which pll?
    @param  pll_settings
                vcor    VCO frequency range control, 
                         0 = 150-250 MHz, 1 = 250-450MHz
                qdiv    PLL feedback divider
                pdiv    PLL input divider value

    @special
        The function will ensure that the PLL is locked before returning. 

    @retval   0 - Success
    @retval   1 - Failed - most likely the PLL did not lock

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_enable_pll(clkmgr_pll_t pll, clkmgr_pll_settings_t *ppll_info);


//--------------------------------------------------
/**
    Disable PLL.

    @param  pll    which pll?

    @special
        None.

    @retval   0 - Success
    @retval   1 - Failed - most likely the PLL did not lock

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_disable_pll(clkmgr_pll_t pll);


//--------------------------------------------------
/**
    Check the PLL state, enabled or disabled.

    @param  pll    which pll?

    @special
        None.

    @retval   0 - False
    @retval   1 - True

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_is_pll_enabled(clkmgr_pll_t pll);

//--------------------------------------------------
/**
    Retrieve the PLL frequency.

    @param  pll    which pll?

    @special
        None.

    @retval   PLL frequency in MHz or '0' if PLL is disabled

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_pll_frequency(clkmgr_pll_t pll);

//--------------------------------------------------
/**
    Compute PLL frequency.

    @param  pll settings

    @special
        There is no hardware access in this call. This api computes the pll frequency for
        the given settings.

    @retval   PLL frequency in MHz

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_compute_pll_frequency(const clkmgr_pll_settings_t *ppll_settings);

//--------------------------------------------------
/**
    Retrieve the PLL settings.

    @param  pll    which pll?

    @special
        None.

    @retval   vcor
    @retval   qdiv
    @retval   pdiv

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_pll_settings(clkmgr_pll_t pll, clkmgr_pll_settings_t *ppll_info);

//--------------------------------------------------
/**
    Set the PLL settings.

    @param  pll    which pll?

    @special
        None.

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_pll_settings(clkmgr_pll_t pll, const clkmgr_pll_settings_t *ppll_info);
//--------------------------------------------------
/**
    Set system clock source and divider values.

    @param  clkmgr_cmsys_clk_settings_t - struct containing the following :
                  csrc    clock source
                  div     divider value
                  mtxdiv  Master clock divider
                  emidiv  Clock ratio between EMI clock and EMI SDRAM clock
                  ahdiv   ARM AHB divider value
                  vhdiv   VC AHB  divider value 

    @special
        Th CMSYS register controls the clock dividers and switchers in the 
        AHB sub-system. The sub-system runs synchronously even though
        the clock frequencies can be different for various sections of
        the sub-system. Clock enables are used to ensure proper 
        synchronization between sections that have different frequencies. 
        In order to do so, all the dividers and switchers have to be 
        changed at the same time to maintain proper clock phase relationship.

    @note - From the opmode interface layer, this API can be used to program
            the dividers for MTX, EMI, ARM AHB and Video AHB.

    @retval   BCM_OK    - Success
    @retval   BCM_ERROR - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_cmsys_clk_settings(const clkmgr_cmsys_clk_settings_t *pcmsys_clk);


//--------------------------------------------------
/**
    Get system clock source and divider values.

    @param  clkmgr_cmsys_clk_settings_t - struct will be filled by this API

    @special
        None.


    @retval   BCM_OK    - Success
    @retval   BCM_ERROR - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_cmsys_clk_settings(clkmgr_cmsys_clk_settings_t *pcmsys_clk);

//--------------------------------------------------
/**
    Retrieve VC subsystem clock source.

    @param   None.
    @special
        None.

    @retval   clkmgr_tcsrc_t

    @ingroup  CLKMGR
*/
//--------------------------------------------------
clkmgr_tcsrc_t clkmgr_get_cmsys_csrc(void);

//--------------------------------------------------
/**
    Retrieve the VC subsystem frequencies.

    @param   clkmgr_cmsys_freqs_t - caller allocated memory for struct 
    
    @special
        None.

    @retval   BCM_OK    - Success
    @retvl    BCM_ERROR - Failure

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_cmsys_freqs(clkmgr_cmsys_freqs_t *pcmsys_freqs, const clkmgr_cmsys_clk_settings_t *cmsys);

//--------------------------------------------------
/**
    Compute VC Subsystem frequencies.

    @param  pll settings

    @special
        There is no hardware access in this call. This api computes the VC subsystem frequencies using the
        the given cmsys and pll divider settings.

    @retval   PLL frequency in MHz

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_compute_cmsys_freqs(clkmgr_cmsys_freqs_t *pvcsys_freqs, 
                                      const clkmgr_cmsys_clk_settings_t *pcmsys, 
                                      const clkmgr_pll_settings_t *ppll_settings);


//--------------------------------------------------
/**
    Set ARM Clock source and divider.

    @param  csrc  clock source
    @param  div   divider value

    @special
        csrc selects the clock source. It is recommended that csrc 
        is not switched to a PLL source until the PLL has been 
        enabled and has acquired lock. This API will enable the PLL 
        if it is not already enabled.

    @retval   0 - Success
    @retval   1 - Failed - failed to enable pll lock if not already enabled

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_arm_clk(clkmgr_csrc_t csrc, uint32_t div);


//--------------------------------------------------
/**
    Retrieve ARM clock source and divider.

    @param   csrc -- filled by this api
    @param   div  -- filled by this api

    @special
        None.

    @retval   BCM_OK

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_arm_clk(clkmgr_tcsrc_t *csrc, uint32_t *div);


//--------------------------------------------------
/**
    Retrieve ARM frequency.

    @param   None.
    @special
        None.

    @retval   ARM frequency in MHz

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_arm_frequency(void);


//--------------------------------------------------
/**
    Set AHB peripheral clock source and divider values.

    @param  clkmgr_cmahb_clk_settings_t - struct containing the following :
                  amcdiv  AMCSS clk divider
                  apdiv   AHP clk divider
                  nrdiv   NOR clk divider
                  spdiv   Shared peripheral clk divider

    @special
            None

    @retval   BCM_OK    - Success
    @retval   BCM_ERROR - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_cmahb_clk_settings(const clkmgr_cmahb_clk_settings_t *pahb_clk);


//--------------------------------------------------
/**
    Get system clock source and divider values.

    @param  clkmgr_cmahb_clk_settings_t - struct will be filled by this API

    @special
        None.


    @retval   BCM_OK    - Success
    @retval   BCM_ERROR - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_cmahb_clk_settings(clkmgr_cmahb_clk_settings_t *pahb_clk);


//--------------------------------------------------
/**
    Set peripheral block clock source and divider.

    @param  blk     block whose clock has to change
    @param  csrc    clock source
    @param  div     divider value [ max. 32 ]

    @special
        csrc selects the clock source. It is recommended that csrc 
        is not switched to a PLL source until the PLL has been 
        enabled and has acquired lock. This API will enable the PLL 
        if it is not already enabled.      

    @retval   0 - Success
    @retval   1 - Failed - failed to enable pll lock if not already enabled

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_peripheral_clk(clkmgr_blk_t blk, clkmgr_csrc_t csrc, uint32_t div);


//--------------------------------------------------
/**
    Retrieve the clock source and divider for the given peripheral.

    @param  clkmgr_blk_t   which peripheral?
    @param clkmgr_tcsrc_t  *pcsrc  [ filled by this api ]
    @param uint32_t                *pdiv   [ filled by this api ]

    @special
        None.

    @retval   

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_peripheral_clk(clkmgr_blk_t blk, 
                                           clkmgr_tcsrc_t *pcsrc,
                                           uint32_t *pdiv);

//--------------------------------------------------
/**
    Set touch wheel clock source and divider.

    @param  csrc    clock source
    @param  div     divider value [ max. 256 ]

    @special
        The caller has to ensure that the clock source is already enabled.     

    @retval   BCM_OK - Success
    @retval   BCM_OK - Failed 

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_touchwheel_csrc_and_div(clkmgr_csrc_t csrc, uint32_t div);


//--------------------------------------------------
/**
    Retrieve the touchwheel clock frequency.

    @param  None.

    @special
        None.

    @retval   touchwheel frequency in Hz

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_touchwheel_frequency(void);

//--------------------------------------------------
/**
    Check the ARM block clock state, enabled or disabled.

    @param  arm_blk   which block?

    @special
            None
                                  
    @retval   0 - FALSE
    @retval   1 - TRUE

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_is_armblk_enabled(clkmgr_arm_block_t arm_blk);
                         

//--------------------------------------------------
/**
    Enable or disable ARM side peripheral block's clock.

    @param  arm_blk   Block whose clock has to be enabled or disabled
    @param  stat      enable or disable

    @special
              None

    @retval   None

    @ingroup  CLKMGR
*/
//--------------------------------------------------
void clkmgr_config_arm_block_clk(clkmgr_arm_block_t arm_blk, bcm_cfg_stat_t stat);


//--------------------------------------------------
/**
    Enable or disable VC side peripheral block's clock.

    @param  vc_blk   Block whose clock has to be enabled or disabled
    @param  stat      enable or disable

    @special
              None

    @retval   None

    @ingroup  CLKMGR
*/
//--------------------------------------------------
void clkmgr_config_vc_block_clk(clkmgr_vc_block_t vc_blk, bcm_cfg_stat_t stat);


//--------------------------------------------------
/**
    Check the VC block clock state, enabled or disabled.

    @param  vc_blk   which block?

    @special
            None

    @retval   0 - False
    @retval   1 - True

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_is_vcblk_enabled(clkmgr_vc_block_t vc_blk);


//--------------------------------------------------
/**
    Set shared peripheral master source.

    @param  clkmgr_sel_access_t  ARM or VC?

    @special
            None

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failure

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_shared_perip_master_src(clkmgr_sel_access_t src);


//--------------------------------------------------
/**
    Get shared peripheral master source.

    @param  None.

    @special
            None

    @retval   ECLKMGR_SEL_ARM_ACCESS
    @retval   ECLKMGR_SEL_VC_ACCESS

    @ingroup  CLKMGR
*/
//--------------------------------------------------
clkmgr_sel_access_t clkmgr_get_shared_perip_master_src(void);


//--------------------------------------------------
/**
    Set LCD master source.

    @param  clkmgr_sel_access_t  ARM or VC?

    @special
            None

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failure

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_lcd_master_src(clkmgr_sel_access_t src);

//--------------------------------------------------
/**
    Get LCD master source.

    @param  None.

    @special
            None

    @retval   ECLKMGR_SEL_ARM_ACCESS
    @retval   ECLKMGR_SEL_VC_ACCESS

    @ingroup  CLKMGR
*/
//--------------------------------------------------
clkmgr_sel_access_t clkmgr_get_lcd_master_src(void);

//--------------------------------------------------
/**
    Get DRAM clock mode.

    @param  None.

    @special
            None

    @retval   ECLKMGR_DRAMCLK_SYNC_TO_EMICLK 
    @retval   ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK

    @ingroup  CLKMGR
*/
//--------------------------------------------------
clkmgr_dram_mode_t clkmgr_get_dram_clk_mode(void);

//--------------------------------------------------
/**
    Set DRAM clock mode.

    @param  mode - ECLKMGR_DRAMCLK_SYNC_TO_EMICLK or
                   ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK  

    @special
            None

    @retval   BCM_OK

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_set_dram_clk_mode(clkmgr_dram_mode_t mode);


int		clkmgr_calc_cmsys_src_and_divs(
							unsigned long							mtx_freq_desired, 
							clkmgr_pll_t				pll_desired,
							unsigned long *							mtx_freq_actual, 
							clkmgr_cmsys_clk_settings_t *cmsys,
							int									tolerance_percent);


uint32_t clkmgr_clock_switch(
		const clkmgr_cmsys_clk_settings_t		*cmsys_new,
		const clkmgr_cmahb_clk_settings_t		*cmahb_new);

uint32_t	clkmgr_calc_pll_freq_for_cmsys(
					unsigned long						mtx_freq_desired,
					unsigned long						*pll_freq_actual,
					clkmgr_cmsys_clk_settings_t		*cmsys);

uint32_t clkmgr_calc_cmsys_divs(uint32_t source_freq, 
					unsigned long mtx_freq_desired, 
					unsigned long *mtx_freq_actual, 	
					clkmgr_cmsys_clk_settings_t *cmsys);


//--------------------------------------------------
/**
    Checks to see if specified CMSYS settings are valid or not.

    @param  clkmgr_cmsys_clk_settings_t * pcmsys_clk; value to check
                   

    @special
            None

    @retval   BCM_OK or BCM_ERROR

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_are_cmsys_clk_settings_valid(const clkmgr_cmsys_clk_settings_t *pcmsys_clk);


//--------------------------------------------------
/**
    Checks to see if specified CMAHB settings are valid or not.

    @param  clkmgr_cmahb_clk_settings_t * pcmahb_clk; value to check
                   

    @special
            None

    @retval   BCM_OK or BCM_ERROR

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_are_cmahb_clk_settings_valid(const clkmgr_cmahb_clk_settings_t *pcmahb_clk);


//--------------------------------------------------
/**
    Gets the frequencies for APB, Shared Peripherals, AMCSS and Synchronous NOR

    @param  
                   
    @special
            None

    @retval   BCM_OK or BCM_ERROR

    @ingroup  CLKMGR
*/
//--------------------------------------------------
uint32_t clkmgr_get_cmahb_freqs(clkmgr_cmahb_freqs_t *pcmahb_freqs, 
								const clkmgr_cmsys_clk_settings_t *pcmsys,
								const clkmgr_cmahb_clk_settings_t *pcmahb);


uint32_t clkmgr_round_rate_pll_frequency(clkmgr_pll_t pll, uint32_t desired_freq,
                                   int tolerance_percent,
                                   uint32_t *actual_freq);

#endif // _CLKMGR_IFC_H_


