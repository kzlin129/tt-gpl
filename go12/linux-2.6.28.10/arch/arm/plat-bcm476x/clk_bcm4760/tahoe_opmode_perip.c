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
 *   @file   tahoe_opmode_perip.c 
 * 
 *   @brief  Peripheral management functions.
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_divide.h"
#include "bcm_log.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "tahoe_opmode.h"
#include "tahoe_opmode_priv.h"
// #include "tahoe_psh_priv.h"
#include "tahoe_stat.h"

/**
    Notes: Some peripherals have both asynchronous clocks as well as synchronous
           clocks.

           Synchronous clocks are turned on and off by modifying the CMCE/C/0/1 
           registers. 

           Asynchronous clocks are turned on and off by setting the csrc to '0'
           in their respective clock registers.

           Three cases:
             (1) Both sync and async need to be controlled.
             (2) Only async controlled.
             (3) Only sync controlled.

    When the peripheral has only async side, its mask is set to the DEFAULT_CORE_MASK.          
*/
#define DEFAULT_CORE_MASK 0x0

typedef struct 
{
    uint32_t        core_mask;   //peripheral core
    uint32_t        is_async;    //does it need csrc modifications?
    clkmgr_blk_t   blk;  //which control reg to modify csrc? based on is_async 
    clkmgr_tcsrc_t csrc; //clock source
    uint32_t        div;         //divider
}bcm47xx_peripheral_t;

/**
    Note: The order of the elements in the array perip_def has to match the order
          of the tahoe_opmode_core_t enum in opmode_ifc.h which inturn is dependent on the order
          of the enums (clkmgr_arm_block_t) and (clkmgr_vc_block_t) in the clkmgr_ifc.h file.
 */
static const bcm47xx_peripheral_t perip_def[OPMODE_PERIP_CLKCORE_COUNT] =
{
//ARM-SIDE peripherals
    {   ECLKMGR_CM_IDE, 1, ECLKMGR_CMSTOR, ECLKMGR_TCSRC_PLL1,  12        },   //CMSTOR=33.33 MHz. Source=PLL1, DIV=12
    { ECLKMGR_CM_CEATA, 1, ECLKMGR_CMSTOR, ECLKMGR_TCSRC_PLL3,  4,  },         //CMSTOR=48 MHZ.     Source=PLL3, DIV=4
    {  ECLKMGR_CM_NAND, 1, ECLKMGR_CMSTOR, ECLKMGR_TCSRC_PLL1,  2,        },   //CMSTOR=200 MHz.   Source=PLL1, DIV=2
    {   ECLKMGR_CM_USB, 0, 0,      0,          0,                         },

    { ECLKMGR_CM_SDIO0, 1, ECLKMGR_CMSDIO0, ECLKMGR_TCSRC_XTAL_24MHZ, 1,  },   // SDIO=12MHz: CSRC=24MHz, DIV=2 
    { ECLKMGR_CM_SDIO1, 1, ECLKMGR_CMSDIO1, ECLKMGR_TCSRC_XTAL_24MHZ, 1,  },   // SDIO=12MHz: CSRC=24MHz, DIV=2 
    {  ECLKMGR_CM_VFIR, 1, ECLKMGR_CMVFIR,  ECLKMGR_TCSRC_PLL2, 3,        },   // VFIR=72MHz (Accurate): CSRC=PLL2, DIV=3
    {  ECLKMGR_CM_CRPT, 1, ECLKMGR_CMCRYPT, ECLKMGR_TCSRC_PLL3, 2,        },   // Crypto=96 MHz: CSRC=PLL3, DIV=2

    {   ECLKMGR_CM_AMC, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_PWM, 1, ECLKMGR_CMPWM,   ECLKMGR_TCSRC_XTAL_32KHZ, 160, },  // PWM Timer = 1 Mhz, csrc=XTAL_24MHz, DIV=24
    { ECLKMGR_CM_UART0, 1, ECLKMGR_CMUART0, ECLKMGR_TCSRC_XTAL_24MHZ, 1,  },  // Uarts=24 MHz, source=24MHz: DIV=1
    { ECLKMGR_CM_UART1, 1, ECLKMGR_CMUART1, ECLKMGR_TCSRC_XTAL_24MHZ, 1,  },  // Uarts=24 MHz, source=24MHz: DIV=1

    { ECLKMGR_CM_UART2, 1, ECLKMGR_CMUART2, ECLKMGR_TCSRC_XTAL_24MHZ, 1,  },  // Uarts=24 MHz, source=24MHz: DIV=1
    {   ECLKMGR_CM_PKE, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_OTP, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_TIM, 1, ECLKMGR_CMTIM0,  ECLKMGR_TCSRC_XTAL_32KHZ,  0, }, //CMTIM0=32 KHz. CSRC=XTAL_27 MHz. In which case, DIV(bit7-0) field is ignored.

    { ECLKMGR_CM_TWSPI, 0, 0,       0,          0 ,                       },
    {  ECLKMGR_CM_SPI0, 1, ECLKMGR_CMSPI0,  ECLKMGR_TCSRC_PLL3, 10,       },  //SPI=19.2MHz: CSRC=PLL3, DIV=10 
    {  ECLKMGR_CM_SPI1, 1, ECLKMGR_CMSPI1,  ECLKMGR_TCSRC_PLL3, 10,       },  //SPI=19.2MHz: CSRC=PLL3, DIV=10 
    {  ECLKMGR_CM_INTC, 0, 0,       0,           0,                       },

    {   ECLKMGR_CM_RNG, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_RTC, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_I2C, 1, ECLKMGR_CMARMI2C, ECLKMGR_TCSRC_XTAL_24MHZ, 8, }, //ARM I2C=3MHz: CSRC=XTAL_24MHz, DIV=8
    {  ECLKMGR_CM_SYSM, 0, 0,       0,          0,                        },

    {  ECLKMGR_CM_GPIO, 0, 0,       0,          0,                        },
    {    ECLKMGR_CM_PM, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_I2S, 1, ECLKMGR_CMARMI2S, ECLKMGR_TCSRC_XTAL_24MHZ, 2, }, //ARM I2S=12MHz (Accurate): CSRC=XTAL_24MHz, DIV=2
    {   ECLKMGR_CM_RMP, 0, 0,       0,          0,                        },

    {   ECLKMGR_CM_RPC, 0, 0,       0,          0,                        },
    {   ECLKMGR_CM_WDT, 0, 0,       0,          0,                        },

//VC-SIDE peripherals
                                       
    {  ECLKMGR_CM_MMTX, 0, 0,       0,          0,                        },
    {  ECLKMGR_CM_MDMA, 0, 0,       0,          0,                        },
    {  ECLKMGR_CM_MROM, 0, 0,       0,          0,                        },
    {  ECLKMGR_CM_MNOR, 0, 0,       0,          0,                        },
                                         
    { ECLKMGR_CM_MSRAM, 0, 0,       0,          0,                        },
    {  ECLKMGR_CM_MEMI, 0, 0,       0,          0,                        },
    {  ECLKMGR_CM_MIPS, 0, 0,       0,          0,                        },
    { ECLKMGR_CM_CUART, 1, ECLKMGR_CMUART,   ECLKMGR_TCSRC_PLL3, 1,       }, //BT Uart=192 MHz, source=PLL3: DIV=1

    {  ECLKMGR_CM_CI2S, 1, ECLKMGR_CMCVDI2S, ECLKMGR_TCSRC_XTAL_24MHZ, 2, }, //CVD I2S=12MHz (Accurate): CSRC=XTAL_24MHz, DIV=2
    {   ECLKMGR_CM_CDC, 1, ECLKMGR_CMLCD,    ECLKMGR_TCSRC_PLL2,      12, }, //CMLCD=20MHz, source=PLL2, DIV=12
    {  ECLKMGR_CM_CSMI, 0, 0,       0,         0,                         },
    {  ECLKMGR_CM_VCPU, 0, 0,       0,         0,                         },

    {  ECLKMGR_CM_VCAM, 1, ECLKMGR_CMCAM,    ECLKMGR_TCSRC_PLL2, 22,      }, //CMCAM=9.8181 MHz: CSRC=PLL2, DIV=22
    {   ECLKMGR_CM_VBG, 0, 0,       0,            0,                      },
    {  ECLKMGR_CM_VINT, 0, 0,       0,            0,                      },
    {  ECLKMGR_CM_VTIM, 1, ECLKMGR_CMVCTIM,  ECLKMGR_TCSRC_XTAL_24MHZ, 24, }, //CMTIM1=24 MHz. CSRC=XTAL_24 MHz. DIV=24

    {  ECLKMGR_CM_VI2C, 1, ECLKMGR_CMVCI2C,  ECLKMGR_TCSRC_XTAL_24MHZ, 8, }, //VC I2C=8MHz: CSRC=XTAL_24MHz, DIV=8
    {  ECLKMGR_CM_VI2S, 1, ECLKMGR_CMVCI2S,  ECLKMGR_TCSRC_XTAL_24MHZ, 2, }, //VC I2S (aka SPDIF)=12 MHz : CSRC=XTAL_24MHZ, DIV=2

    //Does the clk manager bit VI2S also used for SPDIF?
    //Note: PLL3 base frequency changed to 192MHz to accomodate BT UART. So can't use SPDIF along with BT!!
    {  ECLKMGR_CM_VI2S, 1, ECLKMGR_CMVCI2S,  ECLKMGR_TCSRC_PLL3,      10, }, //VC I2S (aka SPDIF)=24.576 MHz : CSRC=PLL3, DIV=10
    {  ECLKMGR_CM_VTVO, 0, 0,       0,                 0,                 },
                                   
    { ECLKMGR_CM_ASTVO, 0, 0,       0,                 0,                 },

//ARM-VC side peripherals that have only  ASYNC clocks.
    { DEFAULT_CORE_MASK, 1, ECLKMGR_CMGEN, ECLKMGR_TCSRC_PLL3, 10,        }, //CMGEN=19.2MHz, source=PLL3, DIV=10
};                                              


//This is used to turn on/off the UART at runtime. By default, it's off, however for development turn it ON.
static uint32_t g_uart_state = enable;

//used to remember vi2s state when the core is switched to spdif mode.
static bcm_cfg_stat_t g_vi2s_stat = disable;
static bcm_cfg_stat_t g_spdif_mode = disable;
static uint32_t config_blk(tahoe_opmode_core_t core, bcm_cfg_stat_t  stat);
static bcm_cfg_stat_t is_blk_enabled(tahoe_opmode_core_t core);
static uint32_t config_perip_csrc(tahoe_opmode_core_t core, 
                                  clkmgr_tcsrc_t perip_csrc, 
                                  uint32_t perip_div, 
                                  bcm_cfg_stat_t stat);
static uint32_t enable_perip_csrc(tahoe_opmode_core_t core,
                                  clkmgr_tcsrc_t perip_csrc, 
                                  uint32_t perip_div);
static tahoe_stat_t disable_perip_csrc(tahoe_opmode_core_t core);
static uint32_t get_blk_user_cnt(clkmgr_blk_t blk);
static uint32_t get_core_max_div_csrc(clkmgr_blk_t blk, uint32_t *pmax_div, uint32_t *pavailable_csrc);
static tahoe_stat_t get_core_clkblk(int core, clkmgr_blk_t *pclk_blk);
static tahoe_stat_t config_core_clk(tahoe_opmode_core_t this_core, bcm_cfg_stat_t stat);
static tahoe_stat_t enable_core(tahoe_opmode_core_t core);
static tahoe_stat_t disable_core(tahoe_opmode_core_t core);
static tahoe_stat_t config_core_preprocessor(tahoe_opmode_corectrl_t *pcore, int *process_core);

#if 0
static bool_t is_core_sharing_clk_on(int core);
#endif

/**
    @fn int tahoe_opmode_is_uart_enabled(void);
*/
int tahoe_opmode_is_uart_enabled(void)
{
    return(g_uart_state);
}

/**
    @fn tahoe_stat_t tahoe_opmode_enable_uart(void);
*/
tahoe_stat_t tahoe_opmode_enable_uart(void)
{
   g_uart_state = enable;
   return OpOk;
}

/**
    @fn tahoe_stat_t tahoe_opmode_disable_uart(void);
*/
tahoe_stat_t tahoe_opmode_disable_uart(void)
{
   g_uart_state = disable;
   return OpOk;
}

/** 
    @fn tahoe_stat_t tahoe_opmode_config_core(tahoe_opmode_corectrl_t *core, unsigned int num_cores);
*/
tahoe_stat_t tahoe_opmode_config_core(tahoe_opmode_corectrl_t *cores, unsigned int num_cores)
{
    uint32_t idx, ret;
    tahoe_stat_t fret;
    tahoe_opmode_core_t this_core;
    int process_core = FALSE;

	if( num_cores != 0 && cores == NULL )
		return(OpBadParam);

    bcm_log_debug("%s : Entry num_cores[%d]\n", __FUNCTION__, num_cores);

    fret = OpOk;

    for (idx = 0; idx < num_cores; idx++)
    {
        this_core = cores[idx].core;
		if (this_core >= ARRAYSIZE(perip_def))
			return(OpBadParam);

        process_core = TRUE;
        ret = config_core_preprocessor(&cores[idx], &process_core);
        if (ret != OpOk)
            return(ret);

        if (process_core == TRUE)
        {
            if (cores[idx].stat == enable)
                ret = enable_core(this_core);
            else
                ret = disable_core(this_core);

            if (ret != OpOk)
                fret = ret;
        }
    }

    bcm_log_debug("%s: Exit num_cores[%d]\n", __FUNCTION__, num_cores);

    return(fret);
}


/**
   @fn tahoe_stat_t tahoe_opmode_get_core_status(const tahoe_opmode_corectrl_t *cores, unsigned int num_cores);
*/
tahoe_stat_t tahoe_opmode_get_core_status(tahoe_opmode_corectrl_t *cores, unsigned int num_cores)
{
    uint32_t idx;
    tahoe_opmode_core_t this_core;
    clkmgr_tcsrc_t csrc = ECLKMGR_TCSRC_NONE;

	if( num_cores != 0 && cores == NULL )
		return(OpBadParam);
    
    for (idx = 0; idx < num_cores; idx++)
    {
        //which core?
        this_core = cores[idx].core;

        if ((this_core == CORE_ARM) || (this_core == CORE_DRAM))
        {
            //if this code is being executed, then both arm and dram are enabled
            cores[idx].stat = enable;
        }
        else
        {
            //This check is done here because some cores are special peripherals
            //and don't exist in the defaults table. 
    		if (this_core >= ARRAYSIZE(perip_def))
	    		return(OpBadParam);

            //is it one with sync-side with or without async clock?
            if (perip_def[this_core].core_mask)
            {   
                //Is this block enabled? 
                cores[idx].stat = is_blk_enabled(this_core);
            }
            else //we have to look at the peripheral that has async-side only
            {
                (void)clkmgr_get_peripheral_clk(perip_def[this_core].blk, &csrc, NULL);
                cores[idx].stat = (csrc > 0) ? enable : disable;
            }
        }
    }

    return(OpOk);
}


/**
    Note: Ideally, the following functions can be moved to a opmode private implementation file.
          However, these functions use the default peripheral structs defined at the top of this
          file. Hence the implementation stays here and the prototypes go to a private header
          file that is not exposed to the users of the opmode layer.
 */

/**
    @fn uint32_t bcm47xx_is_csrc_in_use(clkmgr_tcsrc_t csrc);
*/
uint32_t bcm47xx_is_csrc_in_use(clkmgr_tcsrc_t csrc)
{
    uint32_t idx, div, ret;
    clkmgr_tcsrc_t arm_csrc, perip_csrc = ECLKMGR_TCSRC_NONE;
    uint32_t users = CSRC_UNUSED;

    //Check arm-side peripherals
    for (idx = 0; idx < VC_CORE_BASE; idx++)
    {
        (void)clkmgr_get_peripheral_clk(perip_def[idx].blk, &perip_csrc, NULL);

        if ((clkmgr_is_armblk_enabled((clkmgr_arm_block_t)perip_def[idx].core_mask)) &&
            (perip_def[idx].is_async)  &&
            (perip_csrc == csrc)) 
        {
            //we found one device that is using the source
            users |= CSRC_USED_BY_PERIP;
        } 
    }

    //Check vc-side peripherals
    for (idx = VC_CORE_BASE; idx < ASYNC_ONLY_CORE_BASE; idx++)
    {
        (void)clkmgr_get_peripheral_clk(perip_def[idx].blk, &perip_csrc, NULL);

        if ((clkmgr_is_vcblk_enabled((clkmgr_vc_block_t)perip_def[idx].core_mask)) &&
            (perip_def[idx].is_async)  &&
            (perip_csrc == csrc)) 
        {
            //we found one device that is using the source
            users |= CSRC_USED_BY_PERIP;
        } 
    }

    //Check if async-only peripherals are using this csrc
    for (idx = ASYNC_ONLY_CORE_BASE; idx < NON_PERIP_CORE_BASE; idx++)
    {
        (void)clkmgr_get_peripheral_clk(perip_def[idx].blk, &perip_csrc, NULL);

        if ((perip_def[idx].is_async) &&
            (perip_csrc == csrc)) 
        {
            //we found one device that is using the source
            users |= CSRC_USED_BY_PERIP;
        } 
    }

    //check the rest
    perip_csrc = ECLKMGR_TCSRC_NONE;
	(void)clkmgr_get_peripheral_clk(ECLKMGR_CMDASYN, &perip_csrc, &div);
    if (perip_csrc == csrc)
    {
        users |= CSRC_USED_BY_PERIP;
    }

    ret = clkmgr_get_arm_clk(&arm_csrc, &div);
    if ((ret == BCM_OK) && (arm_csrc == csrc))
    {
        users |= CSRC_USED_BY_ARM;
    }
    
    if (clkmgr_get_cmsys_csrc() == csrc)
    {
        users |= CSRC_USED_BY_SYS;
    }

    return(users);
}


/**
    @fn tahoe_stat_t tahoe_opmode_get_core_freq(int core, unsigned long *pfreq);
*/
tahoe_stat_t tahoe_opmode_get_core_freq(int core, unsigned long *pfreq)
{
    tahoe_stat_t ret;
    clkmgr_blk_t perip_clkblk;

	if ((pfreq == NULL) || (core >= OPMODE_CORE_COUNT))
		return OpBadParam;
       
    switch(core)
    {
        case CORE_ARM:
            ret = tahoe_opmode_get_arm11_freq(pfreq);
        break;
        
        case CORE_DRAM:
            //DRAM frequency depends on which mode it is configured:ASYNC or SYNC.
            ret = bcm47xx_opmode_get_emi_frequency(pfreq);
        break;

        default:
            ret = get_core_clkblk(core, &perip_clkblk);
            if (ret != OpOk)
                return(ret);

            ret = bcm47xx_opmode_get_peripheral_freq(perip_clkblk, pfreq);           
        break;                          
    }

	return(ret);
}

/**
    @fn tahoe_stat_t tahoe_opmode_set_core_freq(int core,
                                                           unsigned long desired_freq,
                                                           int tolerance_percent,
                                                           unsigned long *pactual_freq);
*/
tahoe_stat_t tahoe_opmode_set_core_freq(int core,
                                        unsigned long desired_freq,
                                        int tolerance_percent,
                                        unsigned long *pactual_freq)
{
    int ret;
    clkmgr_freq_info_t freq;
	uint32_t		new_div;
    uint32_t    	new_csrc;
    uint32_t        available_csrc;
    tahoe_stat_t op_ret;
    clkmgr_blk_t perip_clkblk;

    if ((core >= OPMODE_CORE_COUNT) || (pactual_freq == NULL))
		return(OpBadParam);

    switch(core)
    {
        case CORE_ARM:     
        {
            op_ret = tahoe_opmode_set_arm11_freq(desired_freq,
                                                   tolerance_percent,
                                                   pactual_freq);
            if (op_ret != OpOk)
            {
                bcm_log_debug("%s: set_arm11_freq() failed [0x%x]\n", op_ret);
            }
        }
        break;

        case CORE_DRAM:
        {
            op_ret = bcm47xx_opmode_set_dram_freq(desired_freq,
                                                  tolerance_percent,
                                                  pactual_freq);
            if (op_ret != OpOk)
            {
                bcm_log_debug("%s: set_dram_freq() failed [0x%x]\n", op_ret);
            }
        }
        break;

        default:
        {
            //TODO: create a separate function out of this block.
            op_ret = get_core_clkblk(core, &perip_clkblk);
            if (op_ret != OpOk)
                return(op_ret);

            //get max-divider and available csrc values for the peripheral
            (void)get_core_max_div_csrc(perip_clkblk, &freq.max_div, &available_csrc);
    
            //Set the new frequency
            freq.desired_freq = desired_freq;
            freq.tolerance_percent = tolerance_percent;

            //Find a new clock source
	        ret = bcm47xx_find_suitable_source(&freq, available_csrc, 
	                                           &new_csrc, &new_div, pactual_freq);
            if (ret != BCM_OK)
	        {
		        bcm_log_error("%s: Cannot change core[%d] frequency to %lu. No suitable clock source\n", 
        					        __FUNCTION__, core, (unsigned long)desired_freq);
		        return(OpNoSuitableClkSrc);
	        }

            //we found a new source, change the source and divider
            //set up the new csrc/divider.
            (void)clkmgr_set_peripheral_clk(perip_clkblk, new_csrc, new_div);                            

	        bcm_log_debug("%s: New source for core[%d] is clksrc[%d] div[%d] freq[%lu Hz]\n", 
	                                        __FUNCTION__,
                                            core,
									        new_csrc, 
                                            new_div,
									        (unsigned long)pactual_freq);
        }
        break;
    }
        

    return(op_ret);
}


/**
    @fn tahoe_stat_t bcm47xx_opmode_set_core__csrc_and_div(int core,
                                                                    tahoe_opmode_csrc_t csrc,
                                                                    int div);
*/
tahoe_stat_t tahoe_opmode_set_core_csrc_and_div(int core,
                                                           tahoe_opmode_csrc_t csrc,
                                                           int div)
{
    tahoe_opmode_dram_mode_t dram_mode;
    tahoe_stat_t ret = OpOk;
    uint32_t available_csrc, max_div;
    clkmgr_blk_t perip_clkblk;
	clkmgr_tcsrc_t clkmgr_csrc = xlate_opmode_csrc_to_clkmgr_csrc(csrc);
   
    if (core >= OPMODE_CORE_COUNT)
        return OpBadParam;

    switch(core)
    {
        case CORE_ARM:
        {
            if ((clkmgr_csrc == ECLKMGR_TCSRC_XTAL_27MHZ) || (clkmgr_csrc == ECLKMGR_TCSRC_XTAL_32KHZ))
            {
                return(OpInvalidCsrc);
            }
    
            if (div >= CLKMGR_ARMCLK_DIV_MAX)
            {
                return(OpInvalidDiv);
            }

            ret = bcm47xx_opmode_set_arm_csrc_div(clkmgr_csrc, div);
        }
        break;

        case CORE_DRAM:
        {
            //Ensure dram it is setup in async mode.
	        if (tahoe_opmode_get_dram_mode(&dram_mode) != OpOk)
		        return(OpDramInvalidMode);

            if (dram_mode != OPMODE_DRAM_MODE_ASYNC)
            {
		        bcm_log_warn("%s: Configure dram in async mode before attempting to change its clock settings\n", 
        					        __FUNCTION__);
                return(OpEmiAsyncModeNotSetup);
            }
        }
        //break; //Intentionally removed break. DRAM is similar to other peripherals except that
                 //it should be configured in ASYNC mode to be allowed to changes its csrc/div.
        default:
        {
            //TODO: create a separate function out of this block. If that happens make sure the
            //new function is called at the end for CORE_DRAM.
 
            ret = get_core_clkblk(core, &perip_clkblk);
            if (ret != OpOk)
                return(ret);

            //get max-divider and available csrc values for the peripheral
            (void)get_core_max_div_csrc(perip_clkblk, &max_div, &available_csrc);

            if ((available_csrc == ECLKMGR_CLK_USE_FEW_SRC) &&
                ((clkmgr_csrc == ECLKMGR_TCSRC_XTAL_27MHZ) || (clkmgr_csrc == ECLKMGR_TCSRC_XTAL_32KHZ)))
            {
                return(OpInvalidCsrc);
            }

            if (div >= max_div)
            {
                return(OpInvalidDiv);
            }

            ret = enable_perip_csrc(core, clkmgr_csrc, div);
        }
        break;
    }
    
    return(ret);
}


/**
    @fn tahoe_stat_t bcm47xx_opmode_get_core__csrc_and_div(int core,
                                                                    tahoe_opmode_csrc_t *pcsrc,
                                                                    int *pdiv);
*/
tahoe_stat_t tahoe_opmode_get_core_csrc_and_div(int core,
                                                tahoe_opmode_csrc_t *pcsrc,
                                                int *pdiv)
{
    tahoe_stat_t ret = OpOk;
    clkmgr_blk_t perip_clkblk;
	clkmgr_tcsrc_t clkmgr_csrc;

	if ((core >= OPMODE_CORE_COUNT) || (pcsrc == NULL))
		return OpBadParam;

    if ((pcsrc == NULL) || (pdiv == NULL))
        return(OpBadParam);

    switch(core)
    {
        case CORE_ARM:
            ret = clkmgr_get_arm_clk(&clkmgr_csrc, (uint32_t *)pdiv);
        break;

        case CORE_DRAM:
            (void)clkmgr_get_peripheral_clk(ECLKMGR_CMDASYN, 
                                            &clkmgr_csrc,
                                            (uint32_t *)pdiv);
        break;

        default:
        {
            ret = get_core_clkblk(core, &perip_clkblk);
            if (ret != OpOk)
                return(ret);

            (void)clkmgr_get_peripheral_clk(perip_clkblk, 
                                                    &clkmgr_csrc,
                                                    (uint32_t *)pdiv);
        }
        break;
    }

	*pcsrc = xlate_clkmgr_tcsrc_to_opmode_csrc(clkmgr_csrc);
    return(ret);
}


tahoe_stat_t tahoe_opmode_spdif_mode(bcm_cfg_stat_t spdif_stat)
{
    tahoe_stat_t stat = OpOk;
    bcm_cfg_stat_t vi2s_curr_stat = disable;
    clkmgr_tcsrc_t  csrc = ECLKMGR_TCSRC_NONE;
    uint32_t ret;

    if ((stat != enable) && (stat != disable))
        return(OpBadParam);

    //get current stat: vi2s : on or off?
    //We can't use pinshare to determine this because both spdif and vi2s use same pins.
    (void)clkmgr_get_peripheral_clk(ECLKMGR_CMVCI2S, &csrc, NULL);
    if (csrc == ECLKMGR_TCSRC_XTAL_24MHZ)
        vi2s_curr_stat = enable;

    //If we are switching off spdif and if vi2s was enabled earlier, then switch on vi2s.
    if (spdif_stat == disable)
    {
        if (g_vi2s_stat == disable)
        {
            stat = disable_core(CORE_SPDIF);
            if (stat != OpOk)
                bcm_log_error("%s: disable_core(CORE_SPDIF) failed\n", __FUNCTION__);

            stat = disable_core(CORE_VI2S);
            if (stat != OpOk)
                bcm_log_error("%s: disable_core(CORE_VI2S) failed\n", __FUNCTION__);
        }
        else
        {
            stat = disable_core(CORE_SPDIF);
            if (stat != OpOk)
                bcm_log_error("%s: disable_core(CORE_SPDIF) failed\n", __FUNCTION__);

            stat = enable_core(CORE_VI2S);
            if (stat != OpOk)
                bcm_log_error("%s: enable_core(CORE_VI2S) failed\n", __FUNCTION__);
        }

        g_spdif_mode = disable;
    }
    else if (spdif_stat == enable)
    {
        stat = disable_core(CORE_VI2S);
        if (stat != OpOk)
            bcm_log_error("%s: disable_core(CORE_VI2S) failed\n", __FUNCTION__);

        stat = enable_core(CORE_SPDIF);                     
        if (stat != OpOk)
            bcm_log_error("%s: enable_core(CORE_SPDIF) failed\n", __FUNCTION__);

        g_spdif_mode = enable;
    }

    //save current state
    g_vi2s_stat = vi2s_curr_stat;

    //See if we can disable spdif's clock source as it is disabled.
    if (spdif_stat == disable)
    {
        csrc = perip_def[CORE_SPDIF].csrc;
        ret = bcm47xx_is_csrc_in_use(csrc);
        if (ret == CSRC_UNUSED)
        {
            //Turn off the source
            if (csrc > ECLKMGR_TCSRC_PLLS)
            {
                //it is a pll
                ret = clkmgr_disable_pll((clkmgr_pll_t)(csrc - ECLKMGR_TCSRC_PLLS));
            }
            else
            {
                //it is an oscillator
                ret = chipmgr_config_oscillators((chipmgr_clk_t)csrc, ECHIPMGR_CLK_OFF, FALSE);
            }
        }
    }

    return(stat);
}


/**
    @fn tahoe_stat_t bcm47xx_opmode_get_peripheral_freq(clkmgr_blk_t perip_clkblk, unsigned long *pfreq)
*/
tahoe_stat_t bcm47xx_opmode_get_peripheral_freq(clkmgr_blk_t perip_clkblk, unsigned long *pfreq)
{
    tahoe_stat_t ret = OpOk;
    uint32_t div;
    unsigned long freq;
    clkmgr_tcsrc_t  csrc = ECLKMGR_TCSRC_NONE;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(perip_clkblk < CLKMGR_BLK_COUNT);

    (void)clkmgr_get_peripheral_clk(perip_clkblk, &csrc, &div);

    *pfreq = freq = 0;

	switch(csrc)
	{
        case ECLKMGR_TCSRC_NONE :
        {
		    freq = 0;
        }
        break;

        case ECLKMGR_TCSRC_XTAL_24MHZ :
        {
    		if (chipmgr_is_oscillator_on(ECHIPMGR_CLK_24MHZ))
            {
	    		freq =  CLKMGR_XTAL_24MHZ_FREQUENCY;
            }
        }
 		break;

        case ECLKMGR_TCSRC_XTAL_27MHZ :
        {
    		if (chipmgr_is_oscillator_on(ECHIPMGR_CLK_27MHZ))
            {
	    		freq =  CLKMGR_XTAL_27MHZ_FREQUENCY;
            }
        }
 		break;

        case ECLKMGR_TCSRC_XTAL_32KHZ :
        {
    		if (chipmgr_is_oscillator_on(ECHIPMGR_CLK_32KHZ))
            {
	    		freq =  CLKMGR_XTAL_32KHZ_FREQUENCY;
            }
        }
 		break;

        case ECLKMGR_TCSRC_PLLS :
        case ECLKMGR_TCSRC_PLL1 :
        case ECLKMGR_TCSRC_PLL2 :
        case ECLKMGR_TCSRC_PLL3 :
        {
	        ret = tahoe_opmode_get_pll_freq((csrc - ECLKMGR_TCSRC_PLLS), &freq);
        }
    	break;

		default:
        {
    		BCM_ASSERT(!"peripheral clock source invalid!");
		    ret = OpInvalidCsrc;
        }
        break;
	}

    if (freq)
        *pfreq = bcm_udivide32_round(freq, div);

	return(ret);
}


/******************************************************
             PRIVATE STATIC FUNCTIONS
 ******************************************************/
/**
    @fn static tahoe_stat_t config_core_preprocessor(tahoe_opmode_corectrl_t *pcore, int *process_core) 
*/
static tahoe_stat_t config_core_preprocessor(tahoe_opmode_corectrl_t *pcore, int *process_core)
{
    tahoe_opmode_corectrl_t current_core_info = { 0 };

    BCM_ASSERT(pcore != NULL);
    BCM_ASSERT(process_core != NULL);
    
    if (pcore->core >= MAX_CORES)
    {
		return(OpBadParam);
    }

    //ARM and DRAM are "special" cores and cannot be configured just like other
    //cores.
    if ((pcore->core == CORE_ARM) || (pcore->core == CORE_DRAM))
    {
		return(OpBadParam);
    }

    //If the stat is retain, the caller wants to retain it's current state.
    if (pcore->stat == retain)
    {
        *process_core = FALSE;
        return(OpOk);                
    }

    //Assume we want to process this core, i.e we don't intend to skip it.
    *process_core = TRUE;

    /** 
      SPDIF is same as VI2S and share same sync-clk bit. Cannot distinguish one from
      the other. This will be removed from the core list. DO NOT RETURN here... continue
      as there could be cores following this one in the list.
     
      Also, spdif mode is sticky i.e. once spdif mode is enabled, we should not switch
      to VI2S mode because of canned settings. The spdif mode will be persistent across
      opmode switches until the application switches it off explictly.
    */
    if ((pcore->core == CORE_SPDIF) || ((pcore->core == CORE_VI2S) && (g_spdif_mode == enable)))
    {
        *process_core = FALSE;
		return(OpOk);    
    }

    //For TV-OUT only, if the current state is same as requested state, don't process it.
    //Hence, we do not process the core.
    if (pcore->core == CORE_VTVO)
    {
        current_core_info.core = pcore->core;
        current_core_info.stat = disable;
        if ((tahoe_opmode_get_core_status(&current_core_info, 1) == OpOk) &&
            (current_core_info.stat == pcore->stat))
        {
            *process_core = FALSE;
            return(OpOk);
        }
    }

    return(OpOk);
}


/**
    @fn static tahoe_stat_t enable_core(tahoe_opmode_core_t core);
*/
static tahoe_stat_t enable_core(tahoe_opmode_core_t core)
{
    tahoe_stat_t ret = OpOk;

    //If core is being enabled, set-up pinshare before handling peripheral specific logic.

#if 0	
    //Set up pinshare
    ret = tahoe_psh_setup_pinshare(core) ;
    if (ret != OpOk)
    {
        bcm_log_error("%s: tahoe_psh_setup_pinshare(%d) failed\n", __FUNCTION__, core);
        return(ret);
    }
#endif

    //Enable clock
    ret = config_core_clk(core, enable);
    if (ret != OpOk)
    {
        bcm_log_error("%s: config_core_clk(%d, enable) failed\n", __FUNCTION__, core);
        return(ret);
    }

    //Handle peripheral specific custom logic
    ret = tahoe_opmode_perip_custom_handler(core, enable, NULL);
    if (ret)
    {
        bcm_log_error("%s: tahoe_opmode_perip_custom_handler(%d, enable) failed with error[%d]\n",
                         __FUNCTION__, core, ret);
        return(ret);
    }

    bcm_log_debug("%s: enabled(%d) \n", __FUNCTION__, core);
     
    return(ret);
}

/**
    @fn static tahoe_stat_t disable_core(tahoe_opmode_core_t core);
*/
static tahoe_stat_t disable_core(tahoe_opmode_core_t core)
{
    tahoe_stat_t ret = OpOk;

    //If core is being disabled, tear-down pinshare after handling peripheral specific logic.

    //Handle peripheral specific custom logic
    ret = tahoe_opmode_perip_custom_handler(core, disable, NULL);
    if (ret)
    {
        bcm_log_error("%s: tahoe_opmode_perip_custom_handler(%d, disable) failed with error[%d]\n",
                         __FUNCTION__, core, ret);
        return(ret);
    }

    //Disable clock
    ret = config_core_clk(core, disable);
    if (ret != OpOk)
    {
        bcm_log_error("%s: config_core_clk(%d, disable) failed\n", __FUNCTION__, core);
        return(ret);
    }

#if 0
    //Tear down pinshare
    ret = tahoe_psh_teardown_pinshare(core);
    if (ret != OpOk)
    {
        bcm_log_debug("%s: tahoe_psh_teardown_pinshare(%d) failed\n", __FUNCTION__, core);
        return(ret);
    }
#endif

    bcm_log_debug("%s: disabled(%d) \n", __FUNCTION__, core);

    return(ret);
}

/**
    @fn static tahoe_stat_t config_core_clk(tahoe_opmode_core_t this_core, bcm_cfg_stat_t stat);
*/
static tahoe_stat_t config_core_clk(tahoe_opmode_core_t this_core, bcm_cfg_stat_t stat)
{
    uint32_t ret;
    tahoe_stat_t fret = OpOk;

    //Enable or disable the core in the clock manager module
    if (perip_def[this_core].core_mask) 
    {
        config_blk(this_core, stat);
    }

    //If the core is not linked to is_async side, we are done
    if (perip_def[this_core].is_async)
    {
        //we need to modify the is_async clock side
        ret = config_perip_csrc(this_core, perip_def[this_core].csrc, perip_def[this_core].div, stat);
        if (ret != BCM_OK)
        {
            bcm_log_error("%s : config_perip_csrc() failed for core[%d, %d] ret[%d]\n", 
                          __FUNCTION__, this_core, stat, ret);
            fret = OpUnknownErr;
        }
    }

    return(fret);
}

/**
    @fn static uint32_t get_core_max_div_csrc(int blk, uint32_t *pmax_div, uint32_t *pavailable_csrc);
*/
static uint32_t get_core_max_div_csrc(clkmgr_blk_t blk, uint32_t *pmax_div, uint32_t *pavailable_csrc)
{
    BCM_ASSERT(pmax_div != NULL);
    BCM_ASSERT(pavailable_csrc != NULL);
    BCM_ASSERT(blk < CLKMGR_BLK_COUNT);

    switch(blk)
    {
	    case ECLKMGR_CMTIM0:
		case ECLKMGR_CMTIM1:
		case ECLKMGR_CMPWM:
		case ECLKMGR_CMVCTIM:
            *pmax_div = CLKMGR_TIMCLK_DIV_MAX;
            *pavailable_csrc = ECLKMGR_CLK_USE_ALL_SRC;
        break;

        default :
            *pmax_div = CLKMGR_PERICLK_DIV_MAX;
            *pavailable_csrc = ECLKMGR_CLK_USE_FEW_SRC;
        break;
    }

    return(OpOk);
}

/**
    @fn static tahoe_stat_t get_core_clkblk(int core, clkmgr_blk_t *pclk_blk);
*/
static tahoe_stat_t get_core_clkblk(int core, clkmgr_blk_t *pclk_blk)
{
    
    if (core == CORE_ARM)
        return(OpNoAsyncClkSide);

    if (core == CORE_DRAM)
    {
        *pclk_blk = ECLKMGR_CMDASYN;
        return(OpOk);
    }

    BCM_ASSERT(core < OPMODE_PERIP_CLKCORE_COUNT);

    if (perip_def[core].is_async)
    {
        *pclk_blk = perip_def[core].blk;
        return(OpOk);
    }

    return(OpNoAsyncClkSide);
}

#if 0
/**
    static bool_t is_core_sharing_clk_on(int core);
*/
static bool_t is_core_sharing_clk_on(int core)
{
    uint32_t idx, perip_cnt, csrc, sync_clk = disable, async_clk = disable;
    clkmgr_blk_t blk;

    BCM_ASSERT(core < OPMODE_PERIP_CLKCORE_COUNT);

    //get the core's clk-blk
    blk = perip_def[core].blk;

    perip_cnt = ARRAYSIZE(perip_def);
    for (idx = 0; idx < perip_cnt; idx++)
    {
        //excluding the given core, is the mutually exclusive core on?
        if ((idx != core) && (perip_def[idx].is_async) && (perip_def[idx].blk == blk)) 
        {
            (void)clkmgr_get_peripheral_clk(perip_def[idx].blk, &csrc, NULL);
            async_clk = (csrc > 0) ? enable : disable;

            //does it have a sync side and is the block enabled?
            if (perip_def[idx].core_mask)
            {
                sync_clk = (idx < VC_CORE_BASE) ? clkmgr_is_armblk_enabled((clkmgr_arm_block_t)perip_def[idx].core_mask) :
                                              clkmgr_is_vcblk_enabled((clkmgr_vc_block_t)perip_def[idx].core_mask);            
            }

            //async-side and/or sync-side enabled??
            if ((async_clk == enable) || (sync_clk == enable))
            {
                bcm_log_debug("%s: core[%d] enable/disable, so can't enable core[%d]\n",
                         __FUNCTION__, idx, core);
                return(TRUE);
            }            
        } 
    }

    return(FALSE);
}
#endif

/**
    @fn static uint32_t get_blk_user_cnt(clkmgr_blk_t blk);
*/
static uint32_t get_blk_user_cnt(clkmgr_blk_t blk)
{
    uint32_t idx, perip_cnt, stat;
    uint32_t users = 0;

    /**
        As of today this is mainly to ensure CMSTOR is not turned off only when IDE/CEATA and NAND
        are all turned off. Otherwise, CMSTOR will be left untouched. 
    */
    perip_cnt = ARRAYSIZE(perip_def);
    for (idx = 0; idx < perip_cnt; idx++)
    {
        //does this perip have an async side and does it match the input blk?
        if ((perip_def[idx].is_async) && ((perip_def[idx].blk) == blk)) 
        {
            //does it have a sync side and is the block enabled?
            if (perip_def[idx].core_mask)
            {
                stat = (idx < VC_CORE_BASE) ? clkmgr_is_armblk_enabled((clkmgr_arm_block_t)perip_def[idx].core_mask) :
                                              clkmgr_is_vcblk_enabled((clkmgr_vc_block_t)perip_def[idx].core_mask);
            
                //async-side and sync-side enabled
                if (stat)
                    users++;
            }
            //no need to take into consideration the async-only because it is NOT shared between peripherals.
            //for ex: CMGEN is not shared and so is CMLCD, the two known async-only perips.
        } 
    }

    return(users);
}

/**
    @fn static uint32_t config_blk(tahoe_opmode_core_t core, bcm_cfg_stat_t stat);
    Note: Don't want to move this to clkmgr because different enums are used in the clkmgr.         
*/
static uint32_t config_blk(tahoe_opmode_core_t core, bcm_cfg_stat_t stat)
{
    bcm_cfg_stat_t curr_stat;

    curr_stat = is_blk_enabled(core);
    if (curr_stat == stat)
    {
        bcm_log_debug("%s : BLK [%d] already enabled\n", __FUNCTION__, core);
        return(BCM_OK);
    }

    if (core < VC_CORE_BASE)
    {
        clkmgr_config_arm_block_clk(perip_def[core].core_mask, stat);
    }
    else if ((core >= VC_CORE_BASE) && (core < ASYNC_ONLY_CORE_BASE))
    {
        clkmgr_config_vc_block_clk(perip_def[core].core_mask, stat);
    }

    return(BCM_OK);
}


/**
    @fn static bcm_cfg_stat_t is_blk_enabled(tahoe_opmode_core_t core)
*/
static bcm_cfg_stat_t is_blk_enabled(tahoe_opmode_core_t core)
{
    if (core < VC_CORE_BASE)
    {
        return(clkmgr_is_armblk_enabled(perip_def[core].core_mask));
    }
    else if ((core >= VC_CORE_BASE) && (core < ASYNC_ONLY_CORE_BASE))
    {
        return(clkmgr_is_vcblk_enabled(perip_def[core].core_mask));
    }

    return(BCM_OK);    
}

/**
   @fn static uint32_t config_perip_csrc(tahoe_opmode_core_t core, 
                                         clkmgr_tcsrc_t perip_csrc,
                                         uint32_t perip_div, 
                                         bcm_cfg_stat_t stat);
*/
static uint32_t config_perip_csrc(tahoe_opmode_core_t core, 
                                  clkmgr_tcsrc_t perip_csrc, 
                                  uint32_t perip_div, 
                                  bcm_cfg_stat_t stat)
{
    clkmgr_tcsrc_t curr_csrc = ECLKMGR_TCSRC_NONE;
    tahoe_stat_t op_ret;
    clkmgr_blk_t perip_clkblk;

    BCM_ASSERT((stat == enable) || (stat == disable));

    if (stat == disable)
    {
        bcm_log_debug("%s : Peripheral [%d] has async side and is disabled. check to see if the clock source can be turned off\n",
                       __FUNCTION__, core);
        return(disable_perip_csrc(core));
    }

    op_ret = get_core_clkblk(core, &perip_clkblk);
    if (op_ret != OpOk)
        return(op_ret);

    (void)clkmgr_get_peripheral_clk(perip_clkblk, &curr_csrc, NULL);
    if (curr_csrc && (curr_csrc != perip_csrc))
    {
        bcm_log_debug("%s: changing clock src for peripheral[%d] is not recommended, curr_csrc[%d] new_csrc[%d]\n", 
                     __FUNCTION__, core, curr_csrc, perip_csrc);
    }

    return(enable_perip_csrc(core, perip_csrc, perip_div));
}


/**
    @fn static uint32_t enable_perip_csrc(tahoe_opmode_core_t core,
                                          clkmgr_tcsrc_t perip_csrc, 
                                          uint32_t perip_div);
*/
static uint32_t enable_perip_csrc(tahoe_opmode_core_t core,
                                  clkmgr_tcsrc_t perip_csrc, 
                                  uint32_t perip_div)
{
    uint32_t ret=OpOk, csrc_stat, clk_src_users;
    clkmgr_pll_t pll;
    clkmgr_blk_t perip_clkblk;
    bool_t arm_clk_src_switched = FALSE;
    unsigned long new_arm_freq, old_arm_freq;

    //we use this api for DRAM as well which is not treated as peripheral
    //in this code. Hence can't use assert on opmode_perip_clkcore_count.
    if ((core > OPMODE_PERIP_CLKCORE_COUNT) &&
        (core != CORE_DRAM))
    {
        return(OpBadParam);
    }

    ret = get_core_clkblk(core, &perip_clkblk);
    if (ret != OpOk)
        return(ret);

    //check to see if the required source is enabled.
    csrc_stat = bcm47xx_is_csrc_enabled(perip_csrc);

    //Enable the source PLL if core has to be enabled and if the core is linked to async side
    if (csrc_stat == disable) //easy case, just enable the pll before setting the peripheral clk
    {
        bcm_log_debug("%s : Clock source [%d] used by perip[%d] has to be enabled\n", __FUNCTION__, 
                        perip_csrc,  core);

        //Enable the clock source
        if (perip_csrc > ECLKMGR_TCSRC_PLLS)
        {
            //it is a pll
            pll = perip_csrc - CLKMGR_PLL_COUNT; //needed to match pll enums
            ret = clkmgr_enable_pll((clkmgr_pll_t)pll, bcm47xx_get_pll_default(pll));
            if (ret != BCM_OK)
            {
                return(OpPllEnableFailed);
            }
        }
        else
        {
            //it is an oscillator
            //should we wait for the oscillator to come up? for now YES..
            ret = chipmgr_config_oscillators((chipmgr_clk_t)perip_csrc, ECHIPMGR_CLK_ON, TRUE);
            if (ret != BCM_OK)
            {
                return(OpCrystalEnableFailed);
            }
        }

         ret = clkmgr_set_peripheral_clk(perip_clkblk, perip_csrc, perip_div);

    }// end-if pll was disabled
    else //pll is already enabled
    {
        bcm_log_debug("%s : Peripheral [%d] has async side and has clock source[%d] is already  enabled\n", 
                        __FUNCTION__, core, perip_csrc);

        if (perip_csrc > ECLKMGR_TCSRC_PLLS)// pll?
        {
            /*
                Can this PLL be used for this peripheral? why this question?? 
           
                The PLL frequency might have changed dynamically because it was set for other cores
                and hence make sure this PLL it is still usable, i.e we can achieve the desired 
                peripheral frequency by modifying the divider for this core.
            */                
            //Get current pll parameters
            //If current pll parameters don't match the defaults for the PLL,
            pll = perip_csrc - CLKMGR_PLL_COUNT;
            ret = bcm47xx_has_pll_setting_changed(pll);
            if (ret == TRUE) 
            {  
                clk_src_users = bcm47xx_is_csrc_in_use(perip_csrc);
                if (clk_src_users & CSRC_USED_BY_ARM)
                {
                    //ARM is using this csrc, temporarily switch arm to a different source
                    ret = bcm47xx_opmode_switch_arm11_to_stable_csrc(&old_arm_freq, &new_arm_freq);
                    if (ret != OpOk)
                    {
                        return(ret);
                    }

                    arm_clk_src_switched = TRUE;
                }

                if (clk_src_users & CSRC_USED_BY_PERIP)
                {
                    bcm_log_warn("%s : Peripheral [%d] clock source[%d] defaults have changed! \n", __FUNCTION__, core, perip_csrc);
                    bcm_log_warn("%s : Resetting clock source to its defaults and this may affect other peripherals using this clock source\n", __FUNCTION__);
                }

                ret = clkmgr_enable_pll(pll, bcm47xx_get_pll_default(pll));
                if (ret != BCM_OK)
                {
                    return(OpPllEnableFailed);   
                }

                if (arm_clk_src_switched)
                {
                    ret = bcm47xx_opmode_switch_arm11_back_to_old_freq(perip_csrc, old_arm_freq);
                    if (ret != OpOk)
                    {
                        return(ret);
                    }
                }
            }
            //PLL settings unchanged...
            //Now, go ahead and set the peripheral setting to defaults
        } 
        //else //oscillator?
        //{
        //    //nothing to do
        //}
 
        ret = clkmgr_set_peripheral_clk(perip_clkblk, perip_csrc, perip_div);
        if (ret != BCM_OK)
        {
            return(OpPeripSetClkFailed);
        }
    }//end-else - pll is already enabled

    return(ret);
}


/**
    @fn static uint32_t disable_perip_csrc(tahoe_opmode_core_t core);
*/
static uint32_t disable_perip_csrc(tahoe_opmode_core_t core)
{
    uint32_t ret;
    clkmgr_tcsrc_t  current_source = ECLKMGR_TCSRC_NONE;
    clkmgr_blk_t perip_clkblk;

    BCM_ASSERT(core < OPMODE_PERIP_CLKCORE_COUNT);

    ret = get_core_clkblk(core, &perip_clkblk);
    if (ret != OpOk)
        return(ret);

    //Disable the source if no other code tied to it, except for the system PLL and 32KHZ crystal
    //because the syspll should never be turned off??? and 32KHz crystal cannot be turned off
    (void)clkmgr_get_peripheral_clk(perip_clkblk, &current_source, NULL);

    //some blocks are used by multiple peripherals. For ex: CMSTOR is used by
    //IDE, CEATA and NAND. In this case, we can't turn off the clk src for CMSTOR
    //just because we are disabling say CEATA. This will affect IDE if that is turned
    //ON.
    if (!get_blk_user_cnt(perip_clkblk))
    {
        bcm_log_debug("%s : Clock src [%d] for blk[%d] used by peripheral [%d] will be set to NONE\n", 
                           __FUNCTION__, current_source, perip_clkblk, core);
        //set clock source to none..
        ret = clkmgr_set_peripheral_clk(perip_clkblk,  ECLKMGR_TCSRC_NONE, 0);
        if (ret != BCM_OK)
        {
            bcm_log_error("%s : Failed to turn off clk source for Peripheral [%d]\n", 
                           __FUNCTION__, core, current_source);
            return(OpClkSrcDisableFailed);
        }
    }

    //can the clock source be disabled?
    if ((current_source != ECLKMGR_TCSRC_NONE)       && 
        (current_source != ECLKMGR_TCSRC_PLLS)       && 
        (current_source != ECLKMGR_TCSRC_XTAL_32KHZ))
    {
        ret = bcm47xx_is_csrc_in_use(current_source);
        if (ret == CSRC_UNUSED)
        {
            bcm_log_debug("%s : Peripheral [%d], current clk source[%d] is NOT IN-USE\n", 
                          __FUNCTION__, core, current_source);

            //Turn off the source
            if (current_source > ECLKMGR_TCSRC_PLLS)
            {
                //it is a pll
                ret = clkmgr_disable_pll((clkmgr_pll_t)(current_source - ECLKMGR_TCSRC_PLLS));
            }
            else
            {
                //it is an oscillator
                ret = chipmgr_config_oscillators((chipmgr_clk_t)current_source, ECHIPMGR_CLK_OFF, FALSE);
            }
        }
        else
        {
            bcm_log_debug("%s : Peripheral [%d], current clk source[%d] is IN USE\n", 
                          __FUNCTION__, core, current_source);
        }
    }

    return(OpOk);
}


