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
 *   @file   tahoe_opmode_priv.c 
 * 
 *   @brief  Some helper functions.
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_divide.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "tahoe_opmode.h"
#include "tahoe_opmode_priv.h"
#include "tahoe_stat.h"

typedef struct _opmode_strs
{
    tahoe_opmode_mode_t mode;
    char str[MAX_MODE_STR_SZ];
}opmode_strs_t;

static const opmode_strs_t g_opmode_str[] =
{
    { OPMODE_HIBERNATE,            "HIBERNATE"      },
    { OPMODE_SLEEP,                "SLEEP"          },
    { OPMODE_MENU,                 "MENU"           },
    { OPMODE_AUDIO_PLAYBACK,       "AUDIO_PLAYBACK" },
    { OPMODE_VIDEO_PLAYBACK,       "VIDEO_PLAYBACK" },
    { OPMODE_USB_HOST,             "USB_HOST"       },
    { OPMODE_USB_DEV,              "USB_DEV"        },
    { OPMODE_SOFTWARE_IDLE,        "SOFTWARE_IDLE"  },
    { OPMODE_AMCSS_AUDIOPLAYBACK,  "AMCSS_AUDIOPLAYBACK" },
    { OPMODE_BOOTMODE,             "BOOTMODE" },
    { OPMODE_LP_AUDIO_PLAYBACK,   "LP_AUDIO_PLAYBACK" },
    { OPMODE_DEFAULT_MODE,         "DEFAULT_MODE"   }
};

/**
    @fn uint32_t bcm47xx_is_csrc_enabled(clkmgr_tcsrc_t csrc);
*/
uint32_t bcm47xx_is_csrc_enabled(clkmgr_tcsrc_t csrc)
{
    uint32_t csrc_stat = enable;

    BCM_ASSERT(csrc > ECLKMGR_TCSRC_NONE);
    BCM_ASSERT(csrc <= ECLKMGR_MAX_TCSRC);

    if (csrc == ECLKMGR_TCSRC_XTAL_32KHZ)
    {
        return(csrc_stat); //Always enabled
    }

    if (csrc >= ECLKMGR_TCSRC_PLLS)
    {
        //it is a pll
        csrc_stat = tahoe_opmode_is_pll_enabled((csrc - ECLKMGR_TCSRC_PLLS));
        bcm_log_debug("%s : clk source PLL[%d] csrc_stat[%d]\n", __FUNCTION__, csrc, csrc_stat);
    }
    else
    {
        //it is an oscillator
        csrc_stat = chipmgr_is_oscillator_on((chipmgr_clk_t)csrc);
        bcm_log_debug("%s : clk source OSC[%d]  csrc_stat[%d]\n", __FUNCTION__, csrc, csrc_stat);
    }

    return(csrc_stat);
}

/**
    @fn tahoe_stat_t bcm47xx_enable_csrc(clkmgr_tcsrc_t csrc);
*/
tahoe_stat_t bcm47xx_enable_csrc(clkmgr_tcsrc_t csrc)
{
    uint32_t ret;
    clkmgr_pll_t pll;

    //Enable the clock source
    if (csrc > ECLKMGR_TCSRC_PLLS)
    {
        //it is a pll
        pll = csrc - CLKMGR_PLL_COUNT; //needed to match pll enums
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
        ret = chipmgr_config_oscillators((chipmgr_clk_t)csrc, ECHIPMGR_CLK_ON, TRUE);
        if (ret != BCM_OK)
        {
            return(OpCrystalEnableFailed);
        }
    }

    return(OpOk);
}

/**
    @fn tahoe_stat_t bcm47xx_disable_csrc(clkmgr_tcsrc_t csrc);
*/
tahoe_stat_t bcm47xx_disable_csrc(clkmgr_tcsrc_t csrc)
{
    uint32_t ret;
    clkmgr_pll_t pll;

    //Disable the clock source
    if (csrc > ECLKMGR_TCSRC_PLLS)
    {
        //it is a pll
        pll = csrc - CLKMGR_PLL_COUNT; //needed to match pll enums
        ret = clkmgr_disable_pll((clkmgr_pll_t)pll);
        if (ret != BCM_OK)
        {
            return(OpClkSrcDisableFailed);
        }
    }
    else
    {
        //it is an oscillator
        ret = chipmgr_config_oscillators((chipmgr_clk_t)csrc, ECHIPMGR_CLK_OFF, FALSE);
        if (ret != BCM_OK)
        {
            return(OpClkSrcDisableFailed);
        }
    }

    return(OpOk);
}

/*
 * @fn  bool_t bcm47xx_is_suitable_csrc(...)
 */
bool_t bcm47xx_is_suitable_csrc(clkmgr_tcsrc_t source,
                                clkmgr_freq_info_t *pfreq,
                                uint32_t *pnew_div,
							    unsigned long *actual_freq) 
{
	unsigned long  source_freq;
	uint32_t  div;
	uint32_t  deviation_percent;

    BCM_ASSERT((pfreq != NULL) && (pnew_div != NULL) && (actual_freq != NULL));

    if (source == ECLKMGR_TCSRC_NONE)
    {
        return(FALSE);
    }

    //check to see if the source is enabled.
    if (bcm47xx_is_csrc_enabled(source) == disable)
    {
        //If it's not enabled, we can't use it and hence unsuitable
        return(FALSE);
    }

    if (source == ECLKMGR_TCSRC_XTAL_24MHZ)
    {
        source_freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
    }
    else if (source == ECLKMGR_TCSRC_XTAL_27MHZ)
    {
        source_freq = CLKMGR_XTAL_27MHZ_FREQUENCY;
    }
    else if (source == ECLKMGR_TCSRC_XTAL_32KHZ)
    {
        source_freq = CLKMGR_XTAL_32KHZ_FREQUENCY;
    }
    else
    {
    	tahoe_opmode_get_pll_freq(source - ECLKMGR_TCSRC_PLLS, &source_freq);
    }

	bcm47xx_get_suitable_divisor(pfreq->desired_freq, source_freq, &div, CLKMGR_DIV_MIN, pfreq->max_div);
    	
    bcm_log_debug("%s : clk source [%d]  divider[%d] source_freq[%lu]\n", __FUNCTION__, source, div, source_freq);

	*actual_freq = bcm_udivide32_round(source_freq, div);
	if( pfreq->tolerance_percent != 0 )
	{
		deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, *actual_freq);
		if (deviation_percent <= pfreq->tolerance_percent)
		{
			bcm_log_debug("%s: desired_freq[%ld]  actual_freq[%ld] deviation[%d] tolerance[%d] return TRUE\n", 
					__FUNCTION__, pfreq->desired_freq, *actual_freq, deviation_percent, pfreq->tolerance_percent);

			*pnew_div = div;
    		return(TRUE); 
		}
		else
		{
			bcm_log_debug("%s: desired_freq[%ld]  actual_freq[%ld] deviation[%d] tolerance[%d] return FALSE\n", 
					__FUNCTION__, pfreq->desired_freq, *actual_freq, deviation_percent, pfreq->tolerance_percent);

			*actual_freq = 0;
			return(FALSE);
		}
	}
	else
	{
		/* in case tolerance_percent is zero, return TRUE as we are simply looking
		 * for the best match that can  be done
		 */
		bcm_log_debug("%s: desired_freq[%ld]  actual_freq[%ld] tolerance[%d] return TRUE\n", 
				__FUNCTION__, pfreq->desired_freq, *actual_freq, pfreq->tolerance_percent);

    	*pnew_div = div;
		return(TRUE);
	}

}


/*
 * helper function to find if there is a clock source which can be
 * used to generate a desired frequency 
 */
uint32_t bcm47xx_find_suitable_source(clkmgr_freq_info_t *pfreq,
                                      uint32_t use_all,
			    				      clkmgr_tcsrc_t *pnew_csrc, 
				    			      uint32_t *pnew_div,
		    					      unsigned long *pactual_freq)
{
    uint32_t      pll_idx, i, ret=BCM_OK;
	int_t         new_deviation = INT_MAX;
	uint32_t	  div, next_best_div=0;
	int_t   	  deviation;
  	unsigned long source_freq[CLKMGR_TCSRC_COUNT];
  	unsigned long new_source_freq = 0, tmp_freq, next_best_actual_freq=0;
	uint32_t      deviation_percent;
    clkmgr_tcsrc_t next_best_csrc=0;

    BCM_ASSERT((pfreq != NULL) && (pnew_csrc != NULL) && (pnew_div != NULL) && (pactual_freq != NULL));

    for (i=0; i < CLKMGR_TCSRC_COUNT; i++)
        source_freq[i] = 0;

    //gather all source frequencies
    source_freq[ECLKMGR_TCSRC_NONE] = 0;  //TSRC_NONE is 0
    if (bcm47xx_is_csrc_enabled(ECLKMGR_TCSRC_XTAL_24MHZ))
        source_freq[ECLKMGR_TCSRC_XTAL_24MHZ] = CLKMGR_XTAL_24MHZ_FREQUENCY;

    if (use_all && (bcm47xx_is_csrc_enabled(ECLKMGR_TCSRC_XTAL_27MHZ)))  //In some cases, all crystals are used
    {
        source_freq[ECLKMGR_TCSRC_XTAL_27MHZ] = CLKMGR_XTAL_27MHZ_FREQUENCY;
    }

    if (use_all && (bcm47xx_is_csrc_enabled(ECLKMGR_TCSRC_XTAL_32KHZ)))  //In some cases, all crystals are used
    {
        source_freq[ECLKMGR_TCSRC_XTAL_32KHZ] = CLKMGR_XTAL_27MHZ_FREQUENCY;
    }

    //should we leave PLLS out because that could be changing too often and hence not 
    //a stable source?
    //  For now, we'll leave PLLS out.. 
    source_freq[ECLKMGR_TCSRC_PLLS] = 0;
    for (pll_idx = ECLKMGR_TCSRC_PLL1; pll_idx < CLKMGR_TCSRC_COUNT; pll_idx++)
	{
        tahoe_opmode_get_pll_freq(pll_idx - ECLKMGR_TCSRC_PLLS, &(source_freq[pll_idx]) );
    }

    //Find the best of the lot
	for (i = 0; i < CLKMGR_TCSRC_COUNT; i++)
	{
        if (source_freq[i]) //Note: in some cases, the 27MHz and the 32KHz crystals are not used
        {
		    bcm47xx_get_suitable_divisor(pfreq->desired_freq, source_freq[i], &div, CLKMGR_DIV_MIN, pfreq->max_div);
		
		    tmp_freq = bcm_udivide32_round(source_freq[i], div);
		    if (tmp_freq > pfreq->desired_freq)
            {
			    deviation = tmp_freq - pfreq->desired_freq;
            }
	    	else
            {
			    deviation = pfreq->desired_freq - tmp_freq;
            }

	    	if (deviation < new_deviation)
		    {
			    /* remember the "best-suited" PLL so far */
    			new_deviation = deviation;
	    		new_source_freq = source_freq[i];
		    	*pnew_csrc = (clkmgr_tcsrc_t)i;
			    *pnew_div = div;
    		}
	    }
    }

    if (new_deviation != INT_MAX) 
    {
        bcm_log_debug("%s : We might have a new source[%d], new_div[%d]\n",__FUNCTION__, *pnew_csrc, *pnew_div);
	    *pactual_freq = bcm_udivide32_round(new_source_freq, *pnew_div);

		deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, *pactual_freq);

    	bcm_log_debug("%s : desired_freq[%ld]  actual_freq[%ld] deviation[%d] tolerance[%d]\n", 
	    				__FUNCTION__, pfreq->desired_freq, *pactual_freq, deviation_percent, pfreq->tolerance_percent);

		if (((pfreq->tolerance_percent != 0) && (deviation_percent < pfreq->tolerance_percent)) ||
            ((pfreq->tolerance_percent == 0) && (deviation_percent == 0)))
		{
    		return(BCM_OK); 
		}
        else if ((pfreq->tolerance_percent == 0) && (deviation_percent > 0))
        {
            //NEXT_BEST_SOLN
            //If tolerance_percent is '0', the user is expecting the best possible
            //frequency. We have found one but what if there is a disabled PLL
            //that can use use the exact frequency?

            //In order to handle this, we save the current csrc, div etc and if
            //we can't find a disabled source, we fall back on the next best solution.
            next_best_csrc = *pnew_csrc;
            next_best_div = *pnew_div;
            next_best_actual_freq = *pactual_freq;
        }

        //We can't meet the tolerance requirement, see if we can enable a pll
        *pactual_freq = 0;
        ret = bcm47xx_find_disabled_source(pfreq, use_all, pnew_csrc, pnew_div, pactual_freq);
        if (ret == BCM_OK)
        {
            //we found a disabled PLL.
            bcm_log_debug("%s : We found a disabled pll that can be a suitable source\n",__FUNCTION__);
            return(BCM_OK);
        }

        //NEXT_BEST_SOLN
        if ((pfreq->tolerance_percent == 0) && (deviation_percent > 0))
        {
            //switch back to next best values as we could not find 
            *pnew_csrc = next_best_csrc; 
            *pnew_div = next_best_div;
            *pactual_freq = next_best_actual_freq;
            return(BCM_OK);
        }

        return(BCM_ERROR);
    }
    //See if we can use a disabled pll to get to the desired frequency.
    else
    {
        //If there is any PLL (other than the system PLL) that is disabled,
        //we can enable and configure it to its default frequency and use
        //that as the source.
        ret = bcm47xx_find_disabled_source(pfreq, use_all, pnew_csrc, pnew_div, pactual_freq);
        if (ret == BCM_OK)
        {
            //we found a disabled PLL that meets the requirement.
            bcm_log_debug("%s : We found a disabled pll that can be a suitable source\n",__FUNCTION__);
        }
    }

   	return(ret); 
}


/**
   @fn  uint32_t bcm47xx_config_csrcs(bcm47xx_csrc_info_t *clk_srcs, , uint32_t num_csrcs)
*/
uint32_t bcm47xx_config_csrcs(bcm47xx_csrc_info_t *clk_srcs, uint32_t num_csrcs)
{
    uint32_t idx, ret, fret;
    clkmgr_pll_t pll;

    BCM_ASSERT((clk_srcs != NULL) && (num_csrcs != 0));

    fret = BCM_OK;

    for (idx = 0; idx < num_csrcs; idx++)
    {
        BCM_ASSERT(clk_srcs[idx].csrc > ECLKMGR_TCSRC_NONE);
        BCM_ASSERT(clk_srcs[idx].csrc <= ECLKMGR_MAX_TCSRC);

        //Don't touch clock sources that are in use..
        if (bcm47xx_is_csrc_in_use(clk_srcs[idx].csrc))
        {
            bcm_log_debug("%s : clk source [%d] in use, requested cfg[%d]\n", __FUNCTION__, clk_srcs[idx].csrc, clk_srcs[idx].stat);
            continue;
        }

        bcm_log_debug("%s : clk source [%d] csrc_stat[%d]\n", __FUNCTION__, clk_srcs[idx].csrc, clk_srcs[idx].stat);

        if (clk_srcs[idx].csrc < ECLKMGR_TCSRC_PLLS)
        {
            //it is an oscillator
            ret = chipmgr_config_oscillators(clk_srcs[idx].csrc, clk_srcs[idx].stat, TRUE);
        }
        else if (clk_srcs[idx].csrc >= ECLKMGR_TCSRC_PLLS)
        {
            //it is a pll
            //Note: there is no single api to config a pll just like there is one for oscillators
            //      is because enabling a pll needs additional parameters.
            pll = clk_srcs[idx].csrc - ECLKMGR_TCSRC_PLLS;
            if (clk_srcs[idx].stat == enable)
            {
                ret = clkmgr_enable_pll(pll, bcm47xx_get_pll_default(pll));
            }
            else
            {
                ret = clkmgr_disable_pll(pll);
            }
        }

        if (ret != BCM_OK)
        {
            fret = BCM_ERROR;
            bcm_log_error("%s : clk source [%d] could not be configured\n", __FUNCTION__, clk_srcs[idx].csrc);
        }
    }//end for each clock source

    return(fret);
}


tahoe_stat_t bcm47xx_opmode_round_rate_set_up_pll(clkmgr_pll_t pll, clkmgr_freq_info_t *pfreq, 
                                       uint32_t *pnew_div,
                                       unsigned long *pactual_freq)
{
    tahoe_stat_t ret;
    unsigned long pll_freq;
    unsigned long deviation_percent;
    clkmgr_pll_settings_t *pll_defs;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pnew_div != NULL);
    BCM_ASSERT(pactual_freq != NULL);

    //Set up the pll to the desired frequency only of the freq is > min_freq that a pll can be set to..
    //for 32KHz and less, we configure the pll in bypass mode.
    if ((pfreq->desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY) || (pfreq->desired_freq >= CLKMGR_PLL_FREQ_MIN))
    {
        ret = tahoe_opmode_round_rate_pll_freq(pll, 
                                        pfreq->desired_freq, 
    	                                pfreq->tolerance_percent, pactual_freq);

        if (ret == OpOk)
            *pnew_div = 1;

        return(ret);
    }

    //calculate the source frequecy based on default dividers
    pll_defs = bcm47xx_get_pll_default(pll);
    pll_freq = (CLKMGR_XTAL_24MHZ_FREQUENCY * pll_defs->qdiv)/pll_defs->pdiv;

    bcm47xx_get_suitable_divisor(pfreq->desired_freq, pll_freq, pnew_div, CLKMGR_DIV_MIN, pfreq->max_div);

    *pactual_freq = bcm_udivide32_round(pll_freq, *pnew_div);
    deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, *pactual_freq);

    if (pfreq->tolerance_percent != 0)
    {
        if (deviation_percent < pfreq->tolerance_percent)
        {
            //This PLL is good... enable it...
            return(OpOk);
        }
        
        return(OpNoSuitableClkSrc);
    }
    else
    {
        //This PLL is good... enable it...
        return(OpOk);
    }
}


tahoe_stat_t bcm47xx_opmode_set_up_pll(clkmgr_pll_t pll, clkmgr_freq_info_t *pfreq, 
                                       uint32_t *pnew_div,
                                       unsigned long *pactual_freq)
{
    tahoe_stat_t ret;
    unsigned long pll_freq;
    unsigned long deviation_percent;
    clkmgr_pll_settings_t *pll_defs;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pnew_div != NULL);
    BCM_ASSERT(pactual_freq != NULL);

    //Set up the pll to the desired frequency only of the freq is > min_freq that a pll can be set to..
    //for 32KHz and less, we configure the pll in bypass mode.
    if ((pfreq->desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY) || (pfreq->desired_freq >= CLKMGR_PLL_FREQ_MIN))
    {
        ret = tahoe_opmode_set_pll_freq(pll, 
                                        pfreq->desired_freq, 
    	                                pfreq->tolerance_percent, pactual_freq);

        if (ret == OpOk)
            *pnew_div = 1;

        return(ret);
    }

    //calculate the source frequecy based on default dividers
    pll_defs = bcm47xx_get_pll_default(pll);
    pll_freq = (CLKMGR_XTAL_24MHZ_FREQUENCY * pll_defs->qdiv)/pll_defs->pdiv;

    bcm47xx_get_suitable_divisor(pfreq->desired_freq, pll_freq, pnew_div, CLKMGR_DIV_MIN, pfreq->max_div);

    *pactual_freq = bcm_udivide32_round(pll_freq, *pnew_div);
    deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, *pactual_freq);

    if (pfreq->tolerance_percent != 0)
    {
        if (deviation_percent < pfreq->tolerance_percent)
        {
            //This PLL is good... enable it...
            ret = clkmgr_enable_pll(pll, pll_defs);
            if (ret == BCM_ERROR)
            {
                bcm_log_error("%s : Failed to enable PLL[%d]\n", __FUNCTION__, pll);
                return(OpPllSetFreqFailed);
            }

            return(OpOk);
        }
        
        return(OpNoSuitableClkSrc);
    }
    else
    {
        //This PLL is good... enable it...
        ret = clkmgr_enable_pll(pll, pll_defs);
        if (ret == BCM_ERROR)
        {
            bcm_log_error("%s : Failed to enable PLL[%d]\n", __FUNCTION__, pll);
            return(OpPllSetFreqFailed);
        }

        return(OpOk);
    }
}

//Not considering crystals in this function... if required, we could add them as well.
uint32_t bcm47xx_find_disabled_source(clkmgr_freq_info_t *pfreq,
                                      uint32_t use_all,
                                      clkmgr_tcsrc_t *pnew_csrc,
                                      uint32_t *pnew_div,
                                      unsigned long *pactual_freq)
{
    uint32_t      pll_idx, ret=BCM_OK;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pactual_freq != NULL);
    BCM_ASSERT(pnew_csrc != NULL);
    BCM_ASSERT(pnew_div != NULL);

    //Ignore PLLS, the first pll... because it is dynamic and could change frequently.
	for (pll_idx = 1; pll_idx < CLKMGR_PLL_COUNT; pll_idx++)
	{
        if (clkmgr_is_pll_enabled((clkmgr_pll_t)pll_idx) == FALSE)
        {
            ret = bcm47xx_opmode_set_up_pll((clkmgr_pll_t)pll_idx,
                                     pfreq, pnew_div, pactual_freq);
            if (ret == OpOk)
        	{
                *pnew_csrc = (clkmgr_tcsrc_t)pll_idx + ECLKMGR_TCSRC_PLLS;
                return(OpOk);               
            }
        }//endif-pll-disabled
    }//end-for each pll

    //We did not find a any disabled pll or the ones we found were not able to provide us
    //a suitable frequency.
    *pnew_csrc = ECLKMGR_TCSRC_NONE;
    *pnew_div = 0;
    *pactual_freq = 0;

    return(BCM_ERROR);
}

//helper function, private to tahoe
tahoe_opmode_csrc_t xlate_clkmgr_tcsrc_to_opmode_csrc(clkmgr_tcsrc_t tcsrc)
{
	switch( tcsrc )
	{
		case ECLKMGR_TCSRC_XTAL_24MHZ:
		return OPMODE_TCSRC_XTAL_24MHZ;
	
		case ECLKMGR_TCSRC_XTAL_27MHZ:
		return OPMODE_TCSRC_XTAL_27MHZ;
	
		case ECLKMGR_TCSRC_XTAL_32KHZ:
		return OPMODE_TCSRC_XTAL_32KHZ;
	
		case ECLKMGR_TCSRC_PLLS: 
		return OPMODE_TCSRC_PLLS;
	
		case ECLKMGR_TCSRC_PLL1:
		return OPMODE_TCSRC_PLL1;
	
		case ECLKMGR_TCSRC_PLL2:
		return OPMODE_TCSRC_PLL2;

		case ECLKMGR_TCSRC_PLL3:
		return OPMODE_TCSRC_PLL3;

		case ECLKMGR_TCSRC_NONE:
		default:
		return OPMODE_TCSRC_NONE;
	}

	return OPMODE_TCSRC_NONE;
}


clkmgr_tcsrc_t xlate_opmode_csrc_to_clkmgr_csrc(tahoe_opmode_csrc_t csrc)
{
	switch( csrc )
	{
		case OPMODE_TCSRC_XTAL_24MHZ:
		return ECLKMGR_TCSRC_XTAL_24MHZ;
	
		case OPMODE_TCSRC_XTAL_27MHZ:
		return ECLKMGR_TCSRC_XTAL_27MHZ;
	
		case OPMODE_TCSRC_XTAL_32KHZ:
		return ECLKMGR_TCSRC_XTAL_32KHZ;
	
		case OPMODE_TCSRC_PLLS:
		return ECLKMGR_TCSRC_PLLS; 
	
		case OPMODE_TCSRC_PLL1:
		return ECLKMGR_TCSRC_PLL1;
	
		case OPMODE_TCSRC_PLL2:
		return ECLKMGR_TCSRC_PLL2;

		case OPMODE_TCSRC_PLL3:
		return ECLKMGR_TCSRC_PLL3;

		case OPMODE_TCSRC_NONE:
		default:
		return ECLKMGR_TCSRC_NONE;
	}

	return ECLKMGR_TCSRC_NONE;
}

tahoe_opmode_mode_t map_modestr_to_opmode(char *pmode)
{
    int i = 0, sz = 0;

    sz = ARRAY_SIZE(g_opmode_str); 
    for (; i < sz; i++)
    {
        if (bcm_strcmp(g_opmode_str[i].str, pmode) == 0)
        {
            return(g_opmode_str[i].mode);
        }
    }

    return(OPMODE_DEFAULT_MODE);
}

const char *map_opmode_to_modestr(tahoe_opmode_mode_t mode)
{
    int i = 0, sz = 0;

    sz = ARRAY_SIZE(g_opmode_str); 
    for (; i < sz; i++)
    {
        if (g_opmode_str[i].mode == mode)
        {
            return(g_opmode_str[i].str);
        }
    }

    return(NULL);
}



