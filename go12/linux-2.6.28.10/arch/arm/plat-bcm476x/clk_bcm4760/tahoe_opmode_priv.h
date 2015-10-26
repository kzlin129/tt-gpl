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
 *   @file   tahoe_opmode_pric.h 
 * 
 *   @brief  Some helper functions.
 * 
 ****************************************************************************/

#ifndef _TAHOE_OPMODE_PRIV_H
#define _TAHOE_OPMODE_PRIV_H


/***********************************************************
*
* Included files
*
***********************************************************/
#include "bcm_basedefs.h"
#include "bcm_basetypes.h"

#include "clkmgr.h"
#include "chipmgr.h"
// #include "pmu.h"
#include "tahoe_opmode.h"
#include "bcm_divide.h"


/**
    These structure use data types defined in the low-level header.
    The opmode layer should not expose the low-level interfaces/datatypes
    to the user of the opmode layer. 
    
    Also, the structure bcm47xx_peripctrl_t is same as tahoe_opmode_corectrl_t
    defined in the tahoe_opmode.h.
    why?
        1. All these three structures (see below) have to reside in one file. 
           Don't want to split them between public and private headers. 
           Splitting will cause a cyclic dependency between the two headers.

        2. Don't want to break the public interface that is already in use. 
           Hence the public api still expects tahoe_opmode_corectrl_t as param and
           internally we use the peripheral struct.
*/

/**
    @struct bcm47xx_csrc_info_t 

*/
typedef struct _bcm47xx_csrc_info_t_
{
    clkmgr_tcsrc_t csrc;
    bcm_cfg_stat_t  stat;
}bcm47xx_csrc_info_t;

/**
    @struct bcm47xx_peripctrl_t 
*/
typedef tahoe_opmode_corectrl_t bcm47xx_peripctrl_t;

/**
    @struct tahoe_opmode_mode_info_t 
*/
typedef struct _tahoe_opmode_mode_info_t_
{
    tahoe_opmode_mode_t mode;
    bcm47xx_csrc_info_t   clk_settings[CLKMGR_TCSRC_COUNT]; //first one in the enum is NONE and hence not included in the list.
    bcm47xx_peripctrl_t  perip_settings[OPMODE_PERIP_CLKCORE_COUNT];
    //Add default frequencies for arm11, arm-ahb, bus-mtx, vc-02 and emi
    clkmgr_arm_clk_settings_t   arm11_clk;
    clkmgr_cmsys_clk_settings_t cmsys_clk;
    clkmgr_cmahb_clk_settings_t cmahb_clk;
}tahoe_opmode_mode_info_t;
//--------------------------------------------------
/**
    @enum core_type_t

    type of desired voltage that needs to be set - ARM/VC .

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
typedef enum core_type_t
{
    OUTPUT_VOLTAGE_CALC_ARM_TYPE = 0,
    OUTPUT_VOLTAGE_CALC_VC_TYPE
} core_type_t ;

#if 0
/**
     @enum tahoe_audio_ps_type_t - Supported Opmodes
 
      @ingroup TAHOE_OPMODE_PRIVATE
*/
 //--------------------------------------------------
typedef enum
{
    NO_AUDIO_PS = 0,
    AUDIO_PS_DK = 1,
    AUDIO_PS_MSI = 2,    
}tahoe_audio_ps_type_t;
#endif 

//--------------------------------------------------
/**
    Enable interrupts

    @param - None

    @special
        None.
    
    @retval  None

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
BCM_INLINE void bcm47xx_enable_interrupts(void)
{
    __asm("cpsie if  @ __sti" : : : "memory", "cc");    
}

/**
   @fn void bcm47xx_get_suitable_divisor(desired_freq, src_freq, div, min_div, max_div)
   @brief - Given a source frequency (such as that from a PLL), this helper function
            calculates a divisor that is needed to result in a desired frequency.
 */
BCM_INLINE void bcm47xx_get_suitable_divisor(unsigned long desired_freq, 
             								 unsigned long source_freq, 
		        						     uint32_t *div,
				        				     uint32_t min_div, 
						        		     uint32_t max_div)
{
	*div = bcm_udivide32(source_freq, desired_freq);
	*div = max(min_div, *div);
	*div = min(max_div, *div);
}

//--------------------------------------------------
/**
    Disable interrupts

    @param - None

    @special
        None.
    
    @retval  None

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
BCM_INLINE void bcm47xx_disable_interrupts(void)
{
    __asm("cpsid if  @ __cli" : : : "memory", "cc");
}

//--------------------------------------------------
/**
    Is the given source a suitable one?

    @param  csrc           clock src
    @param  pfreq          structure containing desired frequency, max divider and tolerance percent
    @param  *pnew_div      New divider
    @param  *actual_freq   actual frequency

    @special
            None.

    @retval   BCM_SUCCESS
    @retval   BCM_ERROR 

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
bool_t bcm47xx_is_suitable_csrc(clkmgr_tcsrc_t source,
                                clkmgr_freq_info_t *pfreq,
							    uint32_t *new_div,
							    unsigned long *actual_freq);


//--------------------------------------------------
/**
    Enable the clock source

    @param  csrc           clock src

    @special
        It is the responsibility of the caller to determine if the clock
        source needs to be enabled before invoking this function.

    @retval   tahoe_stat_t
 
    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_enable_csrc(clkmgr_tcsrc_t csrc);

//--------------------------------------------------
/**
    Disable the clock source

    @param  csrc           clock src

    @special
        It is the responsibility of the caller to determine if the clock
        source needs to be disabled before invoking this function.

    @retval   tahoe_stat_t
 
    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_disable_csrc(clkmgr_tcsrc_t csrc);

//--------------------------------------------------
/**
    Find a suitable clock source.

    @param  pfreq          structure containing desired frequency, max divider and tolerance percent
    @param  use_all        In some cases, 27MHz and 32KHz crystals are not used. Hence use 0 for those.
    @param  *pnew_csrc     New clock source
    @param  *pnew_div      New divider
    @param  *actual_freq   actual frequency

    @special
            None.

    @retval   BCM_SUCCESS
    @retval   BCM_ERROR 

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
uint32_t bcm47xx_find_suitable_source(clkmgr_freq_info_t *pfreq,
                                      uint32_t use_all,
			    				      clkmgr_tcsrc_t *pnew_csrc, 
				    			      uint32_t *pnew_div,
  		    					      unsigned long *actual_freq);


//--------------------------------------------------
/**
    Have the PLL settings changed?

    @param  pll  - which pll?

    @special
        None.
    
    @retval   TRUE = settings have changed
    @retval  FALSE = settings have not changed

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
bool_t bcm47xx_has_pll_setting_changed(clkmgr_pll_t pll);

//--------------------------------------------------
/**
    Update the pll settings.

    Currently, the pll defaults are the p and q dividers for the desired frequency.
    Since the PLL frequency may be set in nvflash, the p and q dividers used to achieve
    the desired frequency is stored as the new defaults. 

    @param  pll  - which pll?

    @special
        None.
    
    @retval   OpOk 

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t tahoe_update_pll_settings(clkmgr_pll_t pll);
/**
    TODO: add doc
 */
tahoe_stat_t bcm47xx_opmode_set_up_pll(clkmgr_pll_t pll, clkmgr_freq_info_t *pfreq, 
                                       uint32_t *pnew_div,
                                       unsigned long *pactual_freq);

//--------------------------------------------------
/**
    Is the given clock source used by any peripheral or vc subsystem or ARM

    @param csrc - which clock source.

    @special
             None
    
    @retval   uint32_t - with flags indicating which component is using the clock source.

    @see tahoe_opmode_csrc_stat_t for information about the flags

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
uint32_t bcm47xx_is_csrc_in_use(clkmgr_tcsrc_t csrc);


//--------------------------------------------------
/**
    Is the given clock source enabled?

    @param  csrc - which clock source?

    @special
        None.
    
    @retval   enable - if csrc is enabled
    @retval   disable - if csrc is disabled

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
uint32_t bcm47xx_is_csrc_enabled(clkmgr_tcsrc_t csrc);

//--------------------------------------------------
/**
    Is the given pll enabled?

    @param  pll - which pll?

    @special
        Note that the PLL config has to be determined from two places, chipmgr and clkmgr blocks.
        First check with chipmgr to see if it's in bypass mode. If not, check with clkmgr to find
        out if it's enabled.
    
    @retval   TRUE - if pll is enabled
    @retval   FALSE - if pll is disabled

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
bool_t tahoe_opmode_is_pll_enabled(int pll);

//--------------------------------------------------
/**
    Find a disable clock source.

    @param  clkmgr_freq_info_t - desired frequency information
    @param  use_all  - should all clock sources be considered
    @param  pnew_csrc - New clk source, filled in by this api.
    @param  pnew_div  - New divider, filled by this api.
    @param  pactual_freq - Actual freq, filled by this api.

    @special
        The param use_all is not currently being used as oscillators
        are not being considered in this case.
    
    @retval   OpOk - success
    @retval   Other Op codes - Failure

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
uint32_t bcm47xx_find_disabled_source(clkmgr_freq_info_t *pfreq,
                                      uint32_t use_all,
                                      clkmgr_tcsrc_t *pnew_csrc,
                                      uint32_t *pnew_div,
                                      unsigned long *pactual_freq);

//--------------------------------------------------
/**
    Switch ARM11 frequency temporarily to a different source.

    @param  old_freq - arm11 frequency before switching
    @param  new_freq - arm11 frequency after switching

    @special
        This function switches arm to the 24MHz crystal.
        A better solution would be to scale up and down 
        the desired frequency and find a clock source 
        closest to the desired frequency. 

    @retval   OpOk or error code

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_opmode_switch_arm11_to_stable_csrc(unsigned long *pold_freq, unsigned long *pnew_freq);

//--------------------------------------------------
/**
    Switch ARM11 frequency back to old frequency.

    @param  csrc - arm11 csrc before switching
    @param  arm_freq - arm11 frequency before switching

    @special
        When a clock source, pll, settings have to be changed,
        ARM is switched to a stable clock source. This API,
        switches ARM back to the frequency that was before
        ARM was switched to a stable source.

    @retval   OpOk or error code

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_opmode_switch_arm11_back_to_old_freq(clkmgr_tcsrc_t csrc, unsigned long arm_freq);

//--------------------------------------------------
/**
    ARM11 frequency for the given mode.

    @param  mode - for which mode?

    @special    
        The arm11 frequency can be computed offline and stuffed into the default opmode
        table. In order to keep the table information consistent, the arm frequency information
        is stored as src/divider just like it is done for the VC sub system. If computing
        this at run-time is expensive, the table struct can be modified to add a frequency
        field instead of or in addition to the clock source and the divider. 

    @retval   arm11_frequency

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
unsigned long bcm47xx_opmode_arm11_default_freq(tahoe_opmode_mode_t mode);

//--------------------------------------------------
/**
    Config the clock sources provided as input.

    @param  csrc_info - clock source info.

    @special
          If the clock source is a PLL, the default PLL settings will be used.
            
    @retval   BCM_OK
    @retval   BCM_FAILED

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
uint32_t bcm47xx_config_csrcs(bcm47xx_csrc_info_t *clk_srcs, uint32_t num_csrcs);


//--------------------------------------------------
/**
    Get the default settings for the given pll.

    @param  pll  - which pll?

    @special
        None.
    
    @retval  clkmgr_pll_settings_t = pll settings

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
clkmgr_pll_settings_t* bcm47xx_get_pll_default(clkmgr_pll_t pll);


//--------------------------------------------------
/**
    Get the default settings for the given mode.

    @param mode  - which mode?

    @special
        None.
    
    @retval  tahoe_opmode_mode_t = mode default settings

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_opmode_mode_info_t* tahoe_get_opmode_default(tahoe_opmode_mode_t mode);


//--------------------------------------------------
/**
    Get EMI frequency.

    @param None

    @special
        None.
    
    @retval  EMI frequency in Hz.

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t	bcm47xx_opmode_get_emi_frequency(unsigned long *freq);


//--------------------------------------------------
/**
    Get peripheral frequency.

    @param clkmgr_blk_t - peripheral clk blk
    @param frequency          [filled in by this api]

    @special
        None.
    
    @retval  Peripheral frequency in Hz.

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_opmode_get_peripheral_freq(clkmgr_blk_t perip_clkblk, unsigned long *pfreq);

//--------------------------------------------------
/**
    Set DRAM to the desired frequency.

    @param  desired frequency
    @param  tolerance_percent
    @param  pactual_freq  -- will be filled in by this function

    @special
        Note that the DRAM has to be configured in ASYNC mode before
    attempting to change its frequency.
            
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t bcm47xx_opmode_set_dram_freq(unsigned long desired_freq,
                                                   int tolerance_percent,
                                                   unsigned long *actual_freq);

//--------------------------------------------------
/**
    Set ARM clock source and divider to the given values.

    @param  clock_src
    @param  divider

    @special
        None.
                    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------

tahoe_stat_t bcm47xx_opmode_set_arm_csrc_div(clkmgr_tcsrc_t csrc, 
                                                      uint32_t div);


//--------------------------------------------------
/**
    Set the arm11 frequency to the desired frequency.

    @param  desired frequency
    @param  tolerance_percent
    @param  pactual_freq  -- will be filled in by this function

    @special
          Depending on the current PLL settings this may 
    need to move dependent peripherals to other PLLS.
    More information to follow...
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_arm11_freq(unsigned long desired_freq,  
                                                    int tolerance_percent, 
                                                    unsigned long *pactual_freq);

//--------------------------------------------------
/**
    Get the arm11 frequency.

    @special
        Wrapper around clkmgr interface
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_arm11_freq(unsigned long * frequency);

//--------------------------------------------------
/**
    Main handler that invokes peripheral specific custom handler.

    @param  core
    @param  stat - used to either initialize or cleanup depending on whether
                   the core is being enabled or disabled.
    @param  data - Any data that might be useful. For future use...

    @special
        None.
                    
    @retval   OpOk - Success or Op erro code

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_perip_custom_handler(tahoe_opmode_core_t core, bcm_cfg_stat_t stat, void *data);

/* returns TRUE if successful. returns FALSE on failure */
int		bcm47xx_get_cmsys_src_and_divs(
							uint32_t			mtx_frequency_desired, 
							clkmgr_tcsrc_t		pll_desired,
							uint32_t *			mtx_frequency_actual, 
							clkmgr_cmsys_clk_settings_t *cmsys);

tahoe_stat_t bcm47xx_opmode_clock_switch(
	const clkmgr_cmsys_clk_settings_t		*cmsys_new,
	const clkmgr_cmahb_clk_settings_t		*cmahb_new,
	const tahoe_opmode_dram_mode_t				*dram_mode_new,
	bool_t											forced_switch);


tahoe_opmode_csrc_t xlate_clkmgr_tcsrc_to_opmode_csrc(clkmgr_tcsrc_t tcsrc);

clkmgr_tcsrc_t xlate_opmode_csrc_to_clkmgr_csrc(tahoe_opmode_csrc_t csrc);


//--------------------------------------------------
/**
    Get the relavent voltage that can be set for setting the desired freq.

    @param  desired_freq - new freq that is desired.
    @param  core_type    - whether the desired freq to be set is ARM/VC type.
    @param  *millivolts  - this is the output parameter for this function.
                           this is the millivolts value that needs to be set for
			   setting desired freq.

    @special
        None.
                    
    @retval   OpOk - Success or Op erro code

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
//--------------------------------------------------
tahoe_stat_t compute_new_output_voltage(uint32_t desired_freq, core_type_t core_type, uint32_t *millivolts);


/**
    Get the primary storage type.

    @param  None.

    @special
        None.
                    
    @retval   CORE_IDE/CORE_CEATA/CORE_NAND/CORE_SDIO1

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
tahoe_opmode_core_t tahoe_get_primary_storage_core(void);

/**
    Configure the PLLs based on the nvflash settings.

    @param  tahoe_opmode_bootmode_info_t contains PLL info as a string in the nvflash.

    @special
        None.
                    
    @retval   OpOk or error value

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
tahoe_stat_t tahoe_boot_plls(tahoe_opmode_bootmode_info_t *pbootmode_info);

/**
    Maps the given opmode in string format to an enum used for internal processing.

    @param  char buff containing the mode string name.

    @special
        None.
                    
    @retval   Valid opmode enum or INVALID_MODE.

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
tahoe_opmode_mode_t map_modestr_to_opmode(char *pmode);

tahoe_pwr_mode_t map_pwrmodestr_to_pwrmode(const char *pmode);


/**
    Maps the given opmode to mode string.

    @param  tahoe_opmode_mode_t.

    @special
        None.
                    
    @retval   Valid opmode string or NULL

    @ingroup  TAHOE_OPMODE_PRIVATE
*/
const char *map_opmode_to_modestr(tahoe_opmode_mode_t mode);

const char *map_pwrmode_to_pwrmodestr(tahoe_pwr_mode_t mode);


/**
    Initialize internal lists
    
    @param None.

    @Return None.

    @ingroup TAHOE_OPMODE_PRIVATE
*/
void tahoe_init_dvfs_lists(void);

/**
    Returns non-zero value if arm_dvfs is enabled or '0' if disabled from nvflash
    
    @param None.

    @return 1 if arm_dvfs is enabled

    @ingroup TAHOE_OPMODE_PRIVATE
*/
unsigned int is_arm_dvfs_enabled(void);

/**
    Returns non-zero value indicating power save mode
    
    @param None.

    @return non-zero for power save mode.

    @special -  0  indicates normal mode, 1 - indicates power save for DK and 2 - power save for MSI

    @ingroup TAHOE_OPMODE_PRIVATE
*/
// unsigned int get_audio_ps(void);

/**
    Returns the minimum core voltage supported by the supported
    
    @param None.

    @return non-zero for power save mode.

    @special -  None

    @ingroup TAHOE_OPMODE_PRIVATE
*/
unsigned int get_min_core_voltage(void);

/**
    Update the freq-voltage table mappings.
    
    @param None.

    @return non-zero for power save mode.

    @special -  None

    @ingroup TAHOE_OPMODE_PRIVATE
*/
void update_voltage_tables(unsigned int min_voltage);


/** Lower power mode related */
// tahoe_stat_t tahoe_opmode_config_pmu_ldos(pmu_ldo_config_t *pmu_ldo_stat_list, int ldo_count);
//
tahoe_stat_t tahoe_opmode_round_rate_arm_freq(unsigned long desired_freq,  
                                                    int tolerance_percent, 
                                                    unsigned long *pactual_freq);


tahoe_stat_t bcm47xx_opmode_round_rate_set_up_pll(clkmgr_pll_t pll, clkmgr_freq_info_t *pfreq, 
                                       uint32_t *pnew_div,
                                       unsigned long *pactual_freq);

#endif //_TAHOE_OPMODE_PRIV_H
