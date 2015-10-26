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
 *   @file   tahoe_opmode.h
 * 
 *   @brief  All interface functions.
 * 
 ****************************************************************************/


#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**

 @file tahoe_opmode.h

    This file contains operational mode interfaces.

 @author Seetharam Samptur

    Copyright 2004 - 2007 Broadcom Corporation.  All rights reserved.

    Unless you and Broadcom execute a separate written software license
    agreement governing use of this software, this software is licensed to you
    under the terms of the GNU General Public License version 2, available at
    http://www.gnu.org/copyleft/gpl.html (the "GPL").

    Notwithstanding the above, under no circumstances may you combine this
    software in any way with any other Broadcom software provided under a
    license other than the GPL, without Broadcom's express prior written
    consent.


*/
//----------------------------------------------------------
#endif // DOXYGEN_IGNORE_EXTERNAL

#ifndef _TAHOE_OPMODE_H_
#define _TAHOE_OPMODE_H_


/***********************************************************
*
* Included files
*
***********************************************************/
#include "bcm_basetypes.h"
#include "tahoe_stat.h"

//tolerance of '0' means get the best possible frequency closest to the desired frequency NOT EXACT
#define DEFAULT_TOLERANCE_PERCENT 0  

#define MAX_MODE_STR_SZ 50

#define MAX_CLNT_ID_SZ  50

//--------------------------------------------------
/**
     @enum tahoe_opmode_mode_t - Supported Opmodes

     @ingroup TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum
{
    OPMODE_HIBERNATE              =  0,  /**<  0  */
    OPMODE_SLEEP                  =  1,  /**<  1  */
    OPMODE_MENU                   =  2,  /**<  2  */
    OPMODE_AUDIO_PLAYBACK         =  3,  /**<  3  */
    OPMODE_VIDEO_PLAYBACK         =  4,  /**<  4  */
    OPMODE_USB_HOST               =  5,  /**<  5  */
    OPMODE_USB_DEV                =  6,  /**<  6  */
    OPMODE_SOFTWARE_IDLE          =  7,  /**<  7  */
    OPMODE_AMCSS_AUDIOPLAYBACK    =  8,  /**<  8  */
    OPMODE_BOOTMODE               =  9,  /**<  9  */
    OPMODE_LP_AUDIO_PLAYBACK     =  10, /**<  10 */

//the following should be the last entry. so add any new entries above this line
    OPMODE_DEFAULT_MODE                               
}tahoe_opmode_mode_t;

//--------------------------------------------------
/**
     @enum tahoe_pwr_mode_t - Supported power modes

     @ingroup TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum
{
    LOW_POWER_MODE = 0,
    NORMAL_POWER_MODE,
}tahoe_pwr_mode_t;

//--------------------------------------------------
/**
    @enum tahoe_opmode_dram_mode_t

    DRAM Clock mode. 

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum _tahoe_opmode_dram_mode_t
{
	OPMODE_DRAM_MODE_ASYNC,  /**< 0 */
    OPMODE_DRAM_MODE_1by1,   /**< 1 */
    OPMODE_DRAM_MODE_3by2,   /**< 2 */
    OPMODE_DRAM_MODE_2by1,   /**< 3 */
    OPMODE_DRAM_MODE_INVALID 
} tahoe_opmode_dram_mode_t;

/**
    @enum tahoe_opmode_csrc_t

    Clock source.

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum _tahoe_opmode_csrc_t
{
	OPMODE_TCSRC_NONE=0,              /**< 0 */
	OPMODE_TCSRC_XTAL_24MHZ=1,        /**< 1 */
	OPMODE_TCSRC_XTAL_27MHZ=2,        /**< 2 */
	OPMODE_TCSRC_XTAL_32KHZ=3,       /**< 3 */
	OPMODE_TCSRC_PLLS=4,              /**< 4 */
	OPMODE_TCSRC_PLL1=5,              /**< 5 */
	OPMODE_TCSRC_PLL2=6,              /**< 6 */
	OPMODE_TCSRC_PLL3=7,              /**< 7 */
    OPMODE_MAX_TCSRC=7
} tahoe_opmode_csrc_t;

#define OPMODE_TCSRC_COUNT  OPMODE_MAX_TCSRC + 1

//--------------------------------------------------
/**
     @enum tahoe_opmode_csrc_stat_t - Clock source users
*/
//--------------------------------------------------
typedef enum 
{
    CSRC_UNUSED        = 0x0,
	CSRC_USED_BY_PERIP = 0x1, 
    CSRC_USED_BY_ARM   = 0x2,
    CSRC_USED_BY_SYS   = 0x4,
    CSRC_USED_BY_ALL   = 0x7,
}tahoe_opmode_csrc_stat_t;

#define VC_CORE_BASE             30
#define ASYNC_ONLY_CORE_BASE     51
#define NON_PERIP_CORE_BASE      52

//--------------------------------------------------
/**
     @enum tahoe_opmode_core_t - Cores in the system.

     'Core' refers peripherals that either on the ARM side or on the VC side.
     @ingroup TAHOE_OPMODEIFC
*/
//--------------------------------------------------

typedef enum _tahoe_opmode_core_t
{
    //ARM SIDE PERIPHERALS 

    CORE_IDE  = 0,     /**<  0 */
    CORE_CEATA,        /**<  1 */
     CORE_NAND,        /**<  2 */
      CORE_USB,        /**<  3 */

    CORE_SDIO0,        /**<  4 */
    CORE_SDIO1,        /**<  5 */
     CORE_VFIR,        /**<  6 */
     CORE_CRPT,        /**<  7 */

      CORE_AMC,        /**<  8 */
      CORE_PWM,        /**<  9 */
    CORE_UART0,        /**< 10 */
    CORE_UART1,        /**< 11 */

    CORE_UART2,        /**< 12 */
      CORE_PKE,        /**< 13 */
      CORE_OTP,        /**< 14 */
      CORE_TIM,        /**< 15 */

    CORE_TWSPI,        /**< 16 */
     CORE_SPI0,        /**< 17 */
     CORE_SPI1,        /**< 18 */
     CORE_INTC,        /**< 19 */

      CORE_RNG,        /**< 20 */
      CORE_RTC,        /**< 21 */
      CORE_I2C,        /**< 22 */
     CORE_SYSM,        /**< 23 */

     CORE_GPIO,        /**< 24 */
       CORE_PM,        /**< 25 */
      CORE_I2S,        /**< 26 */
      CORE_RMP,        /**< 27 */

      CORE_RPC,        /**< 28 */
      CORE_WDT,        /**< 29 */

    //VC SIDE PERIPHERALS 

//#define VC_CORE_BASE 30

     CORE_MMTX,        /**< 30 */
     CORE_MDMA,        /**< 31 */
     CORE_MROM,        /**< 32 */
     CORE_MNOR,        /**< 33 */

    CORE_MSRAM,        /**< 34 */
     CORE_MEMI,        /**< 35 */
     CORE_MIPS,        /**< 36 */
    CORE_CUART,        /**< 37 */

     CORE_CI2S,        /**< 38 */
      CORE_CDC,        /**< 39 */
     CORE_CSMI,        /**< 40 */
     CORE_VCPU,        /**< 41 */

     CORE_VCAM,        /**< 42 */
      CORE_VBG,        /**< 43 */
     CORE_VINT,        /**< 44 */
     CORE_VTIM,        /**< 45 */

     CORE_VI2C,        /**< 46 */
     CORE_VI2S,        /**< 47 */
     CORE_SPDIF,       /**< 48 */
     CORE_VTVO,        /**< 49 */

     CORE_ASTVO,       /**< 50 */

//#define ASYNC_ONLY_CORE_BASE 51
     //The following peripherals have only asynchronous side
     CORE_GEN,         /**< 51 */

     //The following may not be peripherals but are abstracted that way..
//#define NON_PERIP_CORE_BASE 52
     CORE_ARM,         /**< 52 */
     CORE_DRAM,        /**< 53 */

    //The following are not really cores as seen by the clkmgr. These are here
    //to accomodate pin-share cores.
     CORE_BT_CLK,
     CORE_OPEN,
     CORE_CVAPB_PWREN_CTRL, 
     CORE_KP_ROW,        
     CORE_KP_COL,        
     CORE_SYSTIMER_OUT,     
     CORE_CLK_24MHZ, 
     CORE_CLK_27MHZ,

     MAX_CORES  
}tahoe_opmode_core_t;

#define OPMODE_PERIP_CLKCORE_COUNT NON_PERIP_CORE_BASE
#define OPMODE_CORE_COUNT       NON_PERIP_CORE_BASE + 2 //for ARM and DRAM

/**
    @struct tahoe_opmode_corectrl_t 
*/
typedef struct _tahoe_opmode_corectrl_t
{
   tahoe_opmode_core_t core;
   bcm_cfg_stat_t  stat;
}tahoe_opmode_corectrl_t;


//--------------------------------------------------
/**
    @enum tahoe_opmode_synctop_mode_t

   Sychronizer Operational mode.

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum _tahoe_opmode_synctop_mode_t
{
	SYNCTOP_MODE_BYPASS     = 0, /**< 0 */  //SHOULD BE USED ONLY FOR TESTING!!!
    SYNCTOP_MODE_BANDWIDTH  = 1, /**< 1 */
    SYNCTOP_MODE_LATENCY    = 2, /**< 2 */   
} tahoe_opmode_synctop_mode_t;

//--------------------------------------------------
/**
    @enum tahoe_opmode_synctop_clkmode_t

   Sychronizer clock mode.

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum _tahoe_opmode_synctop_clkmode_t
{
	SYNCTOP_CLKMODE_1to1    = 0, 
	SYNCTOP_CLKMODE_Nto1    = 1, /***/  //Master N times faster than the slave
	SYNCTOP_CLKMODE_1toN    = 2, /***/  //Master N times slower than the slave
}tahoe_opmode_synctop_clkmode_t;

//--------------------------------------------------
/**
    @enum tahoe_opmode_synctop_t

    USB mode

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
typedef enum _tahoe_opmode_synctop_t
{
    SYNCTOP_AI  = 0,     /**< 0 */
    SYNCTOP_AO  = 1,     /**< 1 */
    SYNCTOP_PI  = 2,     /**< 2 */
    SYNCTOP_VI  = 3,     /**< 3 */
    SYNCTOP_VO  = 4,     /**< 4 */
    SYNCTOP_INVALID, 
} tahoe_opmode_synctop_t;

#define TAHOE_OPMODE_SYNCTOP_COUNT  5

typedef enum _dvfs_request_type_t
{
    ARM_FREQ_CHANGE_REQUEST = 0,
    ARM_FREQ_CHANGE_UPDATE_REQUEST,
    ARM_FREQ_CHANGE_CANCEL_REQUEST,
    MATRIX_FREQ_CHANGE_REQUEST,
    MATRIX_FREQ_CHANGE_UPDATE_REQUEST,
    MATRIX_FREQ_CHANGE_CANCEL_REQUEST,
    INVALID_REQUEST
}dvfs_request_type_t;

typedef struct _tahoe_freq_req_info_t
{
    dvfs_request_type_t type;
    char client_id[MAX_CLNT_ID_SZ];
    unsigned long desired_freq;
    int tolerance_percent;

    //internal fields not to be used by caller
    int state;
}tahoe_freq_req_info_t;


/**
    @struct tahoe_opmode_synctop_info_t

    @ingroup  TAHOE_OPMODEIFC
*/
typedef struct _tahoe_opmode_synctop_info_t
{
    tahoe_opmode_synctop_mode_t    mode;
    tahoe_opmode_synctop_clkmode_t clkmode;       
    unsigned int  high_watermark;
    unsigned int  low_watermark; 
    unsigned int  prefetch_len;
}tahoe_opmode_synctop_info_t;

/**
    @struct tahoe_opmode_bootmode_info_t

    @ingroup  TAHOE_OPMODEIFC
*/
typedef struct _tahoe_opmode_bootmode_info_t
{
    tahoe_opmode_core_t  primary_storage;
    unsigned int wakeup_events;
    unsigned int cv_val_at_300_arm ;
    const char *pmode_info;
    const char *ppll_info;
    unsigned int arm_dvfs_stat;
    unsigned int min_core_voltage;
    unsigned int run_to_sleep_timer_val ;
	unsigned int prog_gpios ;
	unsigned int pmu_intr_pin ;
        int vid_be_pin ;
	unsigned int interval_to_suspend_sec ;
	unsigned int interval_to_suspend_min ;
	unsigned int suspend_feature_on ;
	const char *rail_ldo_info ;
	const char *rail_dev_info ;
	const char *ldos_ps_sleep ;
	const char *ldos_ps_audio ;
	const char *ldos_ps_video ;
	const char *ldos_normal ;
	const char *core_gpio ;
	const char *core_mnor ;
	const char *core_rtc ;
	const char *core_ceata ;
	const char *core_cuart ;
	const char *core_spi0 ;
	const char *core_uart0 ;
	const char *core_sdio0 ;
	const char *core_sdio1 ;
	const char *core_cdc ;
	const char *core_csmi ;
	const char *core_spi1 ;
	const char *core_ci2s ;
	const char *core_vi2s ;
	const char *core_tim ;
	const char *core_ide ;
	const char *core_clk_27mhz ;
	const char *core_uart1 ;
	const char *core_uart2;
	const char *core_vcam ;
	const char *core_spdif ;
	const char *core_nand ;
	const char *core_pwrmgr ;
	unsigned int emidiv_from_user ;
	const char *core_pwm ;
}tahoe_opmode_bootmode_info_t;

#define MAX_VC_BOOT_FILE_LENGTH 256 
typedef struct _tahoe_vc_boot_file_info_t
{
    int filename_size ;
	char filename[MAX_VC_BOOT_FILE_LENGTH] ;
	unsigned char disptype_primary ;
} tahoe_vc_boot_file_info_t ;

tahoe_stat_t tahoe_opmode_vc_boot_file_info(tahoe_vc_boot_file_info_t *ptr) ;

//--------------------------------------------------
/**
    Opmode interface initialization

    @param  None

    @special
        This interface library is assumed to be used by one client
        in each environment. Hence it does not keep track of clients.
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_init(tahoe_opmode_bootmode_info_t *bootmode_info);

//--------------------------------------------------
/**
    Is UART enabled?

    @param  None

    @special
        None
                    
    @retval   1 - enabled
    @retval   0 - disabled

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
int tahoe_opmode_is_uart_enabled(void);

//--------------------------------------------------
/**
    Enable UART for debugging

    @param  None

    @special
        None
                    
    @retval None

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_enable_uart(void);

//--------------------------------------------------
/**
    Disable UART

    @param  None

    @special
        None
                    
    @retval None

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_disable_uart(void);


//--------------------------------------------------
/**
    Get the current operational mode we are in..

    @param  None

    @special
        Get be used by higher layer to determine the current mode and take
        appropriate actions if required. Can also be used in debugging.
    
    @retval   char buff - filled with the current mode string
              The input size should be MAX_MODE_STR_SZ

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_mode(char *pmode);


//--------------------------------------------------
/**
    Save the current opmode in string format.

    @param  None

    @special
        Note that the application may introduce new opmodes using the flex option. So,
    the string passed to this api cannot be validated at all times. This is used to
    store the current mode.
    
    @retval   char buff - the current mode in string format
              The input size should be MAX_MODE_STR_SZ

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_mode(const char *pmode);


//--------------------------------------------------
/**
    Set system in a low power mode by lowering voltage and switching off
    unused LDOs. 

    @param mode - mode the system needs to be configured.

    @special
        Low power mode settings implemented only for audio_playback mode.
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_set_power_mode(const char *ppwr_mode);


//--------------------------------------------------
/**
    Set system to one of the specified canned modes

    @param mode - mode the system needs to be configured.

    @special
              None.
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_opmode(const char *pmode);


//--------------------------------------------------
/**
    Set the vc02 subsystem frequency to the desired frequency.

    @param  desired frequency
    @param  tolerance_percent
    @param  pactual_freq  -- will be filled in by this function
	@param	always_use_pll -- by default, this should be set to zero. If set to non-zero,
							the VC-subsystem would always use System-PLL even if desired
							frequency is less than or equal to 24MHz.

    @special
        None.
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_vc02_subsys_freq(unsigned long desired_freq,
                                               int tolerance_percent, 
                                               unsigned long *pactual_freq, 
											   int always_use_pll);

//--------------------------------------------------
/**
    Set the ARM/VC frequency to the desired frequency conditionally.

    @param  frequency_request information
    @param  request_handle, filled in up this API and required for type-1 and type-2 operations.

    @special
        None.
    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_dvfs_request(tahoe_freq_req_info_t *pfreq_info,
                                       unsigned int *request_handle);

//--------------------------------------------------
/**
    Enable or disable one or more cores.

    @param  cores  - list of cores
    @param  num_cores  - number of cores

    @special
        A PLL could be turned on if required to drive the core that
        is being enabled. Similarly, it could be turned off if the
        only core that the PLL was driving is being disabled.
            
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_config_core(tahoe_opmode_corectrl_t *core, unsigned int num_cores);

//--------------------------------------------------
/**
    Switch SPDIF mode on/off.

    @param  stat - enable or disable

    @special
        SPDIF and VI2S refer to the same core but run @ different frequencies and use
    different pin shares. We abstract this out for the app and the application call 
    this api to switch in and out of spdif mode. 

    Tahoe remembers if VI2S was being used when SPDIF was switched on so that it can
    go back to the earlier settings. However, this could cause harmless but undesirable
    effect like if there are any opmode changes between turing spdif on and off.
            
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_spdif_mode(bcm_cfg_stat_t stat);

//--------------------------------------------------
/**
    Reset the specified core.

    @param  core - core to be reset

    @special
        None.
                    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_reset_core(tahoe_opmode_core_t core);

//--------------------------------------------------
/**
    Get status of one or more cores.

    @param  cores  - list of cores
    @param  num_cores  - number of cores

    @special
        The cores param is in-out. On the way out it'll have its status set.
                    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_core_status(tahoe_opmode_corectrl_t *cores, unsigned int num_cores);


//--------------------------------------------------
/**
    Get core clock source and divider.

    @param  core 
    @param  clock_src [filled in by this api]
    @param  divider [filled in by this api]

    @special
        None
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_core_csrc_and_div(int core,
                                                tahoe_opmode_csrc_t *pcsrc,
                                                int *pdiv);

//--------------------------------------------------
/**
    Get core frequency.

    @param  core 
    @param  frequency [filled in by this api]

    @special
        None
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_core_freq(int core, unsigned long *freq);


//--------------------------------------------------
/**
    Set core to the desired frequency.

    @param  core 
    @param  desired frequency
    @param  tolerance_percent

    @special
        None.
                    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_core_freq(int core,
                                        unsigned long desired_freq,
                                        int tolerance_percent,
                                        unsigned long *actual_freq);

//--------------------------------------------------
/**
    Set core clock source and divider to the given values.

    @param  core 
    @param  clock_src
    @param  divider

    @special
        None.
                    
    @retval   tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_core_csrc_and_div(int core,
                                                tahoe_opmode_csrc_t csrc,
                                                int div);

//--------------------------------------------------
/**
    Get DRAM mode.

    @param None

    @special
        None
    
    @retval  OPMODE_DRAM_MODE_ASYNC
    @retval  OPMODE_DRAM_MODE_1by1
    @retval  OPMODE_DRAM_MODE_3by2
    @retval  OPMODE_DRAM_MODE_2by1

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_dram_mode(tahoe_opmode_dram_mode_t *mode);


//--------------------------------------------------
/**
    Set DRAM mode.

    @param tahoe_opmode_dram_mode_t

    @special
        SYNC to ASYNC mode -- sets dram at 133Mhz.
        ASYNC to SYNC mode -- sets dram in sync with emi and uses
                              emidiv that was used before switching
                              to ASYNC mode.
    
    @retval  tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_dram_mode(tahoe_opmode_dram_mode_t dram_mode);


//--------------------------------------------------
/**
    Set synctop settings.

    @param tahoe_opmode_synctop_t         - Which synctop?
    @param tahoe_opmode_synctop_info_t    - Synctop settings

    @special
        There are some basic system requirements that have to be met while
        changing synctop settings.

        I. Clock Modes :
                   From BCM2820 Datasheet  9.11.4.3	Synccntrl Registers

           For 5 sync_tops, system requires the following: 
                   Everyone can be 1 to 1 mode. 
                   2 can be in N to 1 mode and 
                   the other 3 can be in 1:N mode. 

        II. Clock ratios :
                The synchronizers do not allow arbitrary ratios on each side.
                
                a. If they are in "bandwidth" mode, the ratio has to be "2**n:1" or "1:2**n"
                   Allowed ratio is 1:2^n where n can be 0-6.
                    
                b. If they are in "latency" mode, the ratio has to be "2:1", "1:1" or "1:2"        
                    
    @retval  tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_synctop(tahoe_opmode_synctop_t synctop, 
                                      const tahoe_opmode_synctop_info_t *psynctop_info);


//--------------------------------------------------
/**
    Get synctop settings.

    @param tahoe_opmode_synctop_t         - Which synctop?
    @param tahoe_opmode_synctop_info_t    - Synctop settings (filled in by this api)

    @special
        None.
                            
    @retval  tahoe_stat_t

    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------

tahoe_stat_t tahoe_opmode_get_synctop(tahoe_opmode_synctop_t synctop, 
                                      tahoe_opmode_synctop_info_t *psynctop_info);

//--------------------------------------------------
/**
    Get the pll frequency.

    @special
        Wrapper around clkmgr interface
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t  tahoe_opmode_get_pll_freq(int pll, unsigned long *freq);


//--------------------------------------------------
/**
    Set the pll frequency.

    @special
        Wrapper around clkmgr interface
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_set_pll_freq(int pll, unsigned long desired_freq,
                                       int tolerance_percent,
                                       unsigned long *actual_freq);


//--------------------------------------------------
/**
    Get the VC subsystem frequency frequency.

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_vcsys_freq(unsigned long *pmtx_freq, unsigned long *pvc_freq, unsigned long *armahb_freq);

//--------------------------------------------------
/**
    Get the various CMSYS dividers and the CSRC

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t	tahoe_opmode_get_cmsys_clk_settings(tahoe_opmode_csrc_t *csrc, unsigned int *div, unsigned int *mtxdiv, unsigned int *ahdiv, unsigned int *vhdiv );

//--------------------------------------------------
/**
    Set the various CMAHB dividers. 

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t	tahoe_opmode_get_cmahb_divs(unsigned int *apdiv, unsigned int *spdiv, unsigned int *amdiv, unsigned int *nrdiv );

//--------------------------------------------------
/**
    Set the various CMSYS dividers. Meant for diagnostic purpose only.

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t	tahoe_opmode_set_cmsys_divs(unsigned int div, unsigned int mtxdiv, unsigned int ahdiv, unsigned int vhdiv );

//--------------------------------------------------
/**
    Set the various CMAHB dividers. Meant for diagnostic purpose only.

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t	tahoe_opmode_set_cmahb_divs(unsigned int apdiv, unsigned int spdiv, unsigned int amdiv, unsigned int nrdiv );

//--------------------------------------------------
/**
    Get the frequencies for APB, Shared Peripherals, AMC and NOR (CMAHB)

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_opmode_get_cmahb_freqs(unsigned long *ap_freq, unsigned long *sp_freq, unsigned long *am_freq, unsigned long *nr_freq);

//--------------------------------------------------
/**
    Enable/Disable shared peripheral ownership

    @special
        Wrapper around low-level interfaces
    
    @ingroup  TAHOE_OPMODEIFC
*/
//--------------------------------------------------
tahoe_stat_t tahoe_arm_shared_perip_owner(unsigned int allow_arm);

//--------------------------------------------------
/**
    Setup the system with the desired boot opmode settings.

    @param  pbootmode_info  - string containing boot mode information 
                              retrieved from nvflash

    @special
        None.
            
    @retval   0 - Success
    @retval   non-zero - Failure     

    @ingroup  BCM28XX_BSP
*/
//--------------------------------------------------
tahoe_stat_t tahoe_set_boot_opmode(tahoe_opmode_bootmode_info_t *pbootmode_info);

tahoe_opmode_core_t opmode_get_core(char *core_str) ;

//--------------------------------------------------
/**
    pass the EMI NCDL values.

    @param  read_ncdl  - read NCDL offset
    @param  write_ncdl  - write NCDL offset
    
    @special
        None.
            
    @retval   0 - Success
    @retval   non-zero - Failure     

    @ingroup  BCM28XX_BSP
*/
//--------------------------------------------------
void tahoe_get_emi_ncdl(unsigned int * read_ncdl, unsigned int * write_ncdl);

void tahoe_set_control_emidiv_from_user(unsigned int val) ;

void tahoe_opmode_get_max_arm_freq(unsigned long *freq ) ;

void tahoe_opmode_set_max_arm_freq(unsigned long freq ) ;

void tahoe_set_max_core_voltage_when_arm_300(unsigned int val) ;

void tahoe_get_max_core_voltage_when_arm_300(unsigned int *val) ;

tahoe_stat_t tahoe_opmode_round_rate_pll_freq(int pll, unsigned long desired_freq,
                                       int tolerance_percent,
                                       unsigned long *actual_freq);


#endif //_TAHOE_OPMODE_H_
