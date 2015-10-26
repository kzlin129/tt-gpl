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
 *   @file   tahoe_opmode_dram.c 
 * 
 *   @brief  DRAM related apis
 * 
 ****************************************************************************/

#include "bcm_basedefs.h"
#include "bcm_divide.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "emi.h"

#include "tahoe_opmode_priv.h"
#include "tahoe_opmode.h"

//Note: lowered default freq from 133MHz to 100MHz for testing. Some boards lockup with 133MHz.
#define DRAM_DEFAULT_DIV 4               //assuming pll1 is at 400MHz and we want 100MHz
#define DRAM_ASYNC_DEFAULT_FREQ 100000000UL

static tahoe_stat_t set_dram_async_mode(void);
static tahoe_stat_t set_dram_sync_mode(tahoe_opmode_dram_mode_t dram_mode);
static tahoe_stat_t get_csrc_div_for_dram(clkmgr_tcsrc_t *pnew_csrc, 
                                                   uint32_t *pnew_div);

/**
    @fn tahoe_stat_t bcm47xx_opmode_get_emi_frequency(unsigned long *pfreq);
*/
tahoe_stat_t bcm47xx_opmode_get_emi_frequency(unsigned long *pfreq)
{
    tahoe_stat_t           ret = OpOk;
    tahoe_opmode_dram_mode_t		dram_mode;
    clkmgr_cmsys_freqs_t	cmsys_freqs;

	if (pfreq == NULL)
		return(OpBadParam);

    //Is DRAM configured in async mode?
    tahoe_opmode_get_dram_mode(&dram_mode);

    switch(dram_mode)
    {
        case OPMODE_DRAM_MODE_ASYNC:
    		ret = bcm47xx_opmode_get_peripheral_freq(ECLKMGR_CMDASYN, pfreq);   	
        break;
        
        case OPMODE_DRAM_MODE_INVALID:
            *pfreq = 0;
        break;
        
        default:
            //Get emi-div & mtx_freq values
            cmsys_freqs.mtx_freq = 0;
            if (clkmgr_get_cmsys_freqs(&cmsys_freqs, NULL) != BCM_OK)
			{
				*pfreq = 0;
                return(OpVcSysGetFreqFailed); /* error */
			}

		    *pfreq = cmsys_freqs.mtx_freq;
        break; 
    }

    return(ret);
}


/**
    @fn tahoe_stat_t bcm47xx_opmode_set_dram_freq(unsigned long desired_freq,
                                                   int tolerance_percent,
                                                   unsigned long *pactual_freq);    
*/
tahoe_stat_t bcm47xx_opmode_set_dram_freq(unsigned long desired_freq,
                                                   int tolerance_percent,
                                                   unsigned long *pactual_freq)
{
    int ret;
    clkmgr_freq_info_t freq;
	uint32_t		new_div;
    uint32_t    	new_csrc;
	tahoe_opmode_dram_mode_t	dram_mode;
       
    if( pactual_freq == NULL)
		return OpBadParam;

	if( tahoe_opmode_get_dram_mode(&dram_mode) != OpOk )
		return OpDramInvalidMode;

    if (dram_mode != OPMODE_DRAM_MODE_ASYNC)
    {
		bcm_log_warn("%s: Configure dram in async mode before attempting to change its frequency\n", 
        					__FUNCTION__);
        return(OpEmiAsyncModeNotSetup);
    }

    //Set the new frequency
    freq.desired_freq = desired_freq;
    freq.max_div = CLKMGR_PERICLK_DIV_MAX;
    freq.tolerance_percent = tolerance_percent;

	ret = bcm47xx_find_suitable_source(&freq, ECLKMGR_CLK_USE_FEW_SRC, 
	                                   &new_csrc, &new_div, pactual_freq);
    if (ret != BCM_OK)
	{
		bcm_log_warn("%s: Cannot change DRAM frequency to %lu. No suitable clock source\n", 
        					__FUNCTION__, (unsigned long)desired_freq);
		return(OpNoSuitableClkSrc);
	}

    //we found a new source, change the source and divider
    //set up the new csrc/divider.
    (void)clkmgr_set_peripheral_clk(ECLKMGR_CMDASYN, new_csrc, new_div);                            

	bcm_log_debug("%s: New source is clksrc[%d] freq[%lu Hz]\n", 
	                                __FUNCTION__,
									new_csrc, 
									(unsigned long)pactual_freq);

    return(OpOk);
}

/**
    @fn tahoe_opmode_dram_mode_t tahoe_opmode_get_dram_mode(void);
*/
tahoe_stat_t tahoe_opmode_get_dram_mode(tahoe_opmode_dram_mode_t *dram_mode)
{
    chipmgr_dram_clkmode_t cmode;
    clkmgr_dram_mode_t    clkmode;

	if( dram_mode == NULL )
		return OpBadParam;

    *dram_mode = OPMODE_DRAM_MODE_INVALID;

    /*
        Ideally, we should consider chipmgr emi-dram clkmode as well.
        But we set that info to 0x2 because of a hardware bug. If that
        gets fixed, we need to revisit this part of the code.
    */
    cmode = chipmgr_get_emi_dram_clkmode();
    clkmode = clkmgr_get_dram_clk_mode();

    switch(cmode)
    {
    	case ECHIPMGR_EMI_DRAM_ASYNC :
            BCM_ASSERT(clkmode == ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK);
            if (clkmode == ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK)
                *dram_mode = OPMODE_DRAM_MODE_ASYNC;
        break;

        case ECHIPMGR_EMI_DRAM_1by1 :
            BCM_ASSERT(clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK);
            if (clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK)
                *dram_mode = OPMODE_DRAM_MODE_1by1;                   
        break;

        case ECHIPMGR_EMI_DRAM_3by2 :
            BCM_ASSERT(clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK);
            if (clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK)
                *dram_mode = OPMODE_DRAM_MODE_3by2;
        break;

        case ECHIPMGR_EMI_DRAM_2by1 :       
            BCM_ASSERT(clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK);
            if (clkmode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK)
                *dram_mode = OPMODE_DRAM_MODE_2by1;
        break;

        default:
    		BCM_ASSERT(!"Invalid emi_dram clock mode");
        break;
    }

    return OpOk;
}

tahoe_stat_t tahoe_opmode_set_dram_mode(tahoe_opmode_dram_mode_t dram_mode)
{
    tahoe_stat_t opstat = OpOk;

    BCM_ASSERT((dram_mode == OPMODE_DRAM_MODE_ASYNC) || 
               (dram_mode == OPMODE_DRAM_MODE_1by1)  || 
               (dram_mode == OPMODE_DRAM_MODE_3by2)  || 
               (dram_mode == OPMODE_DRAM_MODE_2by1));

    switch(dram_mode)
    {
        case OPMODE_DRAM_MODE_ASYNC :
            opstat = set_dram_async_mode();
        break;

        case OPMODE_DRAM_MODE_1by1  :
        case OPMODE_DRAM_MODE_3by2  :
        case OPMODE_DRAM_MODE_2by1  :
        default:
            opstat = set_dram_sync_mode(dram_mode);
        break;
    }

    return(opstat);
}

/**
    @fn static tahoe_stat_t set_dram_async_mode(void);

    Notes: On switching to this mode, we always configure dram at 133MHz.
           For this purpose, we have increased the defaults for PLL1 to
           400MHz from 200MHz.

           Once dram is configured in async mode, the user can later 
           change its frequency to any desired frequency.
*/
static tahoe_stat_t set_dram_async_mode(void)
{
    tahoe_stat_t ret = OpOk;
    tahoe_opmode_dram_mode_t dram_mode;
    clkmgr_tcsrc_t csrc;
    uint32_t div;
	clkmgr_cmsys_clk_settings_t cmsys;

	tahoe_opmode_get_dram_mode(&dram_mode);
    if ( dram_mode == OPMODE_DRAM_MODE_ASYNC)
    {
        return(OpOk);
    }

    ret = get_csrc_div_for_dram(&csrc, &div);
    if (ret != OpOk)
    {
        return(ret);
    }

    //we will configure dram at 133Mhz and we will use PLL1.
    //set up the csrc/divider.
    (void)clkmgr_set_peripheral_clk(ECLKMGR_CMDASYN, csrc, div);                            

    //do a forced-clock switch so that emi knows that dram is being configured in asyncmode.
    dram_mode = OPMODE_DRAM_MODE_ASYNC;

	/* Somewhat of an HACK:
	 * When setting SDRAM to async mode, we set the emidiv to "1:1" mode.
	 * This is not really needed by H/W
	 * However, this make life simpler for the clkmgr function is_dram_div_valid()
	 */
	clkmgr_get_cmsys_clk_settings(&cmsys);
	cmsys.emidiv = OPMODE_DRAM_MODE_1by1;

    ret = bcm47xx_opmode_clock_switch(&cmsys, NULL, &dram_mode, TRUE);
    if (ret != OpOk)
    {
        bcm_log_debug("%s : bcm47xx_opmode_clock_switch failed, ret[%d]\n", __FUNCTION__, ret);        
        return(OpClkSwitchFailed);
    }

    return(ret);
}

/**
    @fn static tahoe_stat_t get_csrc_div_for_dram(clkmgr_tcsrc_t *pnew_csrc, 
                                                            uint32_t *pnew_div);
*/
static tahoe_stat_t get_csrc_div_for_dram(clkmgr_tcsrc_t *pnew_csrc, 
                                                   uint32_t *pnew_div)
{
    unsigned long actual_freq=0;
    tahoe_stat_t ret;
    clkmgr_freq_info_t freq;

    BCM_ASSERT(pnew_csrc != NULL);
    BCM_ASSERT(pnew_div != NULL);

    //check if PLL1(the default PLL for dram) is enabled and ready to use
    if (bcm47xx_is_csrc_enabled(ECLKMGR_TCSRC_PLL1) == disable)
    {
        ret = bcm47xx_enable_csrc(ECLKMGR_TCSRC_PLL1);
        if (ret == OpOk)
        {
            *pnew_div =  DRAM_DEFAULT_DIV;
            *pnew_csrc = ECLKMGR_TCSRC_PLL1;
            return(OpOk);
        }
        else
        {
            bcm_log_error("%s: unable to enable clock_src[pll1]\n");
            return(OpClkSrcEnableFailed);
        }
    }

    //ensure pll1's default settings have not changed
    if (!bcm47xx_has_pll_setting_changed((clkmgr_pll_t)(ECLKMGR_TCSRC_PLL1 - ECLKMGR_TCSRC_PLLS)))
    {
        *pnew_div =  DRAM_DEFAULT_DIV;
        *pnew_csrc = ECLKMGR_TCSRC_PLL1;
        return(OpOk);
    }

    freq.desired_freq = DRAM_ASYNC_DEFAULT_FREQ;
    freq.max_div = CLKMGR_PERICLK_DIV_MAX;
    freq.tolerance_percent = 0;

	ret = bcm47xx_find_suitable_source(&freq, ECLKMGR_CLK_USE_FEW_SRC, 
	                                   pnew_csrc, pnew_div, &actual_freq);
    if (ret != BCM_OK)
	{
		bcm_log_warn("%s: Cannot change DRAM frequency to %lu. No suitable clock source\n", 
        					__FUNCTION__, (unsigned long)freq.desired_freq);
		return(OpNoSuitableClkSrc);
	}

    return(OpOk);
}


static tahoe_stat_t set_dram_sync_mode(tahoe_opmode_dram_mode_t dram_mode)
{
    tahoe_opmode_dram_mode_t current_mode;
    clkmgr_cmsys_clk_settings_t cmsys_val;
    tahoe_stat_t ret = OpOk;

	clkmgr_get_cmsys_clk_settings(&cmsys_val);

    tahoe_opmode_get_dram_mode(&current_mode);
    if (current_mode != OPMODE_DRAM_MODE_ASYNC)
    {
        bcm_log_debug("%s: current_mode[%d] new_mode[%d]\n", __FUNCTION__, current_mode, dram_mode);
    }

	switch(dram_mode)
    {
		case OPMODE_DRAM_MODE_1by1:
            cmsys_val.emidiv = ECLKMGR_EMIDIV_1_1;
		break;
		
		case OPMODE_DRAM_MODE_2by1:
            cmsys_val.emidiv = ECLKMGR_EMIDIV_2_1;
		break;

		case OPMODE_DRAM_MODE_3by2:
            cmsys_val.emidiv = ECLKMGR_EMIDIV_3_2;
		break;

		case ECLKMGR_EMIDIV_NONE:
		default:
		    BCM_ASSERT(!"EMIDIV has invalid value");
            cmsys_val.emidiv = ECLKMGR_EMIDIV_1_1;
		break;
    }

    //do the clock switch so that dram is back in-sync with emi
    ret = bcm47xx_opmode_clock_switch(&cmsys_val, NULL, &dram_mode, TRUE);
    if (ret != OpOk)
    {
        bcm_log_debug("%s : bcm47xx_opmode_clock_switch failed, ret[%d]\n", __FUNCTION__, ret);        
        return(OpClkSwitchFailed);
    }

     //Reset csrc/divider as dram is programmed in sync mode.
    (void)clkmgr_set_peripheral_clk(ECLKMGR_CMDASYN, ECLKMGR_CSRC_NONE, 0);

    return(ret);
}

