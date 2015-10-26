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
 *   @file   chipmgr.h
 * 
 *   @brief  Chip manager low level API set.
 * 
 ****************************************************************************/


#ifndef _CHIPMGR_IFC_H_
#define _CHIPMGR_IFC_H_

#include "bcm_basetypes.h"
#include "bcm_privtypes.h"

//--------------------------------------------------
/**
    @enum chipmgr_dram_clkmode_t

    Clock mode indicating the EMI & DRAM frequency ratios.

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chimgr_dram_clkmode_t
{
	ECHIPMGR_EMI_DRAM_ASYNC,
    ECHIPMGR_EMI_DRAM_1by1,
    ECHIPMGR_EMI_DRAM_3by2,
    ECHIPMGR_EMI_DRAM_2by1,
} chipmgr_dram_clkmode_t;


//--------------------------------------------------
/**
    @enum chipmgr_usbmode_t

    USB mode

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_usbmode_t
{
	ECHIPMGR_USBMODE_DEVICE = 0, /**< 0 */
    ECHIPMGR_USBMODE_HOST   = 1, /**< 1 */
    ECHIPMGR_USBMODE_IDDQ   = 2, /**< 2 */
} chipmgr_usbmode_t;

//--------------------------------------------------
/**
    @enum chipmgr_usbblk_t

    USB block 

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_usbblk_t
{
	ECHIPMGR_USBBLK_PHY = 0, /**< 0 */
    ECHIPMGR_USBBLK_PLL = 1, /**< 1 */
} chipmgr_usbblk_t;


//--------------------------------------------------
/**
    @enum chipmgr_spi_port_t

    SPI Port 

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_spi_port_t
{
	ECHIPMGR_SPI_PORT0 = 0, /**< 0 */
    ECHIPMGR_SPI_PORT1 = 1, /**< 1 */
} chipmgr_spi_port_t;

//--------------------------------------------------
/**
    @enum chipmgr_clk_t

    Oscillators

    @see clkmgr_tcsrc_t

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_clk_t
{
	ECHIPMGR_CLK_24MHZ = 1,    /**< 1 */    //not from 0 as it is mapped to tcsrc in clkmgr
    ECHIPMGR_CLK_27MHZ = 2,    /**< 2 */
    ECHIPMGR_CLK_32KHZ = 3,    /**< 3 */    //note: 32Khz crystal can't be turned on or off
    ECHIPMGR_CLK_24_AND_27MHZ = 4, /**< 4 */
} chipmgr_clk_t;


//--------------------------------------------------
/**
    @enum chipmgr_clkstat_t

    Oscillators state - ON or OFF

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_clkstat_t
{
	ECHIPMGR_CLK_OFF  = 0,    /**< 0 */
    ECHIPMGR_CLK_ON = 1,    /**< 1 */
    ECHIPMGR_CLK_RETAIN = 2    /**< 2 */
}chipmgr_clkstat_t;

typedef enum _chipmgr_pll_t
{
	ECHIPMGR_PLL0=0,
	ECHIPMGR_PLL1=1,
	ECHIPMGR_PLL2=2,
	ECHIPMGR_PLL3=3,
} chipmgr_pll_t;

typedef enum _chipmgr_pll_config_t
{
	ECHIPMGR_PLL_CONFIG_NORMAL=0,
	ECHIPMGR_PLL_BYPASS_WITH_24MHZ=1,
	ECHIPMGR_PLL_BYPASS_WITH_32KHZ=2,
} chipmgr_pll_config_t;


//--------------------------------------------------
/**
    @enum chipmgr_pinshare_mode_t

    Note: Don't have all the information abt pin share to come up
          with sensible enums/apis. 

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_pinshare_mode_t
{
	ECHIPMGR_CONNECT_ARM_I2S  = 0,    /**< 0 */
    ECHIPMGR_CONNECT_VC_I2S   = 1,    /**< 1 */
}chipmgr_pinshare_mode_t;

//--------------------------------------------------
/**
    @enum chipmgr_audio_blk_freq_t

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
typedef enum _chipmgr_audio_blk_freq_t
{
	ECHIPMGR_AUDIO_BLK_12MHZ=0,
	ECHIPMGR_AUDIO_BLK_22_579474MHZ=1,
	ECHIPMGR_AUDIO_BLK_12_288MHZ=2,
  	ECHIPMGR_AUDIO_BLK_24_576MHZ=3,
}chipmgr_audio_blk_freq_t;

//--------------------------------------------------
/**
    Initialize clock manager.

    @param  Block virtual address

    @special
        This initialization routine currently initializes the base 
        address for the chip manager block. 

    @retval   None

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
void chipmgr_api_init(void);


//--------------------------------------------------
/**
    Set EMI & DRAM clock modes.

    @param  chipmgr_dram_clkmode_t  mode [ async / 1:1 / 2:1 / 3:2 ]

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_set_emi_dram_clkmode(chipmgr_dram_clkmode_t mode);

//--------------------------------------------------
/**
    Enable or disable the 10MBiit VC SRAM 

    @param  chipmgr_enable_vc_sram  enable [ TRUE / FALSE ]

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_enable_vc_sram(bool_t enable);

//--------------------------------------------------
/**
    Get EMI & DRAM clock modes.

    @param  None

    @retval  chipmgr_dram_clkmode_t  mode [ async / 1:1 / 2:1 / 3:2 ]

    @special

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
chipmgr_dram_clkmode_t chipmgr_get_emi_dram_clkmode(void);


//--------------------------------------------------
/**
    Set the emi_dll reset counter. This is the time to wait 
    DRAM DLL entering lock state after DLL is reset.

    @param  None

    @special - For now the this API programs the lock value
               to 150 obtained from the asic team. If this
               is programmable, then this API can be changed
               to allow the user to feed in a different value. 

    @retval   None

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
void chipmgr_set_emi_dll_rst_counter(void);


//--------------------------------------------------
/**
    Configure USB mode.

    @param  mode   [ device or host ]

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_usb(chipmgr_usbmode_t usb_mode);


//--------------------------------------------------
/**
    Reset USB PHY.

    @param  None

    @Special - NOT IMPLEMENTED

    Note: Reseting the USB phy involves modifying the USB
          control register as well as a different register 
          in the USB core itself. Also, there should be a
          time delay between the writes. Hence, this API 
          has to be implemented at a higher level.

          Here is the information from an email regarding
          USB Phy reset from Philip Chen:

            Usb phy has 3 resets -- reset_hi_pll, resetb and softresetb [1:0]
            reset_hi_pll is high true, resetb is low true, 
                                       softresetb = 2'b00 (reset), 
                                                  = 2'b11 (not reset)
 
            First reset_hi_pll = 1, resetb = 0, softresetb = 2'b00
                after 1 us, reset_hi_pll = 0
                after another 1 us, resetb = 1,
                after another 1 us, softresetb = 2'b11
            
    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_reset_usb(void);

//--------------------------------------------------
/**
    Reset USB PHY or USB PLL.

    @param  usb_blk -- phy or pll

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @see     chipmgr_reset_usb

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_reset_usb_blk(chipmgr_usbblk_t usb_blk);

//--------------------------------------------------
/**
    Enable/disable OTP in IDDQ mode.

    @param  stat -- enable or disable

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_otp_iddq(bcm_cfg_stat_t stat);             


//--------------------------------------------------
/**
    Enable/disable RNG in IDDQ mode.

    @param  stat -- enable or disable

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_rng_iddq(bcm_cfg_stat_t stat);             

//--------------------------------------------------
/**
    Is Oscillator turned on ?

    @param  clk -- 24MHz or 27MHz

    @special
            None

    @retval   TRUE - powered on
    @retval   FALSE - powered off

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
bool_t  chipmgr_is_oscillator_on(chipmgr_clk_t clk);

//--------------------------------------------------
/**
    Turn on or off an Oscillator.

    @param  clk -- 24MHz or 27MHz or both crystal
    @param  clkstat -- ON or OFF
    @param  should_wait  - True, wait for 10ms. 
                           10ms is the time required for the oscillator to be turned on.

                           False - Don't wait.

    @special
            None

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_oscillators(chipmgr_clk_t clk, chipmgr_clkstat_t stat, bool_t should_wait);


//--------------------------------------------------
/**
    Modify pinshare registers for a given mode.

    @param  mode - CONNECT_ARM_I2S or CONNECT_VC_I2S
    
    @special
            The mode can change once there is enough information abt pin share usage in opmode.
    At this time, this api is used to change pin control register #8 to allow amcs audio playback
    mode.

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_pinshare(chipmgr_pinshare_mode_t mode);

//--------------------------------------------------
/**
    Set VC2 Audio Block Control frequency input indicator.

    @param  freq_mode   chipmgr_audio_blk_freq_t

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @special
        Note this register is just a frequency input indicator. The actual
        frequency should be set in the clock manager register.

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_vc2_audioblk_control(chipmgr_audio_blk_freq_t freq_mode);

uint32_t	chipmgr_set_pll_config(chipmgr_pll_t pll, chipmgr_pll_config_t pll_config);

chipmgr_pll_config_t	chipmgr_get_pll_config(chipmgr_pll_t pll);

//--------------------------------------------------
/**
    Configure SPI.

    @param  spi_  
    @param  enable or disable 

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  CHIPMGR
*/
//--------------------------------------------------
uint32_t chipmgr_config_spi(chipmgr_spi_port_t spi_port, bcm_cfg_stat_t stat);


#endif // _CHIPMGR_IFC_H_

