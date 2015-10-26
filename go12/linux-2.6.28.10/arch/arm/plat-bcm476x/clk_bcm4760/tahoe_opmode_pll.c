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
 *   @file   tahoe_opmode_pll.c 
 * 
 *   @brief  PLL manipulation apis.
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "tahoe_opmode.h"
#include "tahoe_opmode_priv.h"
#include "tahoe_stat.h"

static clkmgr_pll_settings_t pll_def[] =
{
     //qdiv pdiv
    { 150, 12 }, //PLL0 - qdiv=150, pdiv=12, freq=300 MHz

    { 200, 12 }, //PLL1 - qdiv=200, pdiv=12, freq=400 MHz
    { 108, 12 }, //PLL2 - qdiv=108, pdiv=12, freq=216 MHz
    {  96, 12 }, //PLL3 - qdiv=96, pdiv=12, freq=192 MHz   //for BT UART
//    { 204, 20 }  //PLL3 - qdiv=204, pdiv=20, freq=244.8 MHz  -- Required when SPDIF is in use
};


/**
    @fn tahoe_stat_t tahoe_update_pll_settings(clkmgr_pll_t pll)
*/
tahoe_stat_t tahoe_update_pll_settings(clkmgr_pll_t pll)
{
    clkmgr_pll_settings_t pll_info;

    BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll < CLKMGR_PLL_COUNT);

    if (!clkmgr_is_pll_enabled(pll))
        return(OpOk);

    clkmgr_get_pll_settings(pll, &pll_info);    

    pll_def[pll].qdiv = pll_info.qdiv;
    pll_def[pll].pdiv = pll_info.pdiv;

    bcm_log_debug("%s: pll_def[%d].qdiv = [%d]\t pll_def[%d].pdiv = [%d]\n", 
            __FUNCTION__, pll, pll_def[pll].qdiv, pll, pll_def[pll].pdiv); 

    return(OpOk);
}

/**
    @fn bool_t  bcm47xx_has_pll_setting_changed(clkmgr_pll_t pll);
*/
bool_t bcm47xx_has_pll_setting_changed(clkmgr_pll_t pll)
{
    clkmgr_pll_settings_t pll_info;

    BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll < CLKMGR_PLL_COUNT);

    if (!clkmgr_is_pll_enabled(pll))
        return(TRUE);

    clkmgr_get_pll_settings(pll, &pll_info);    

    if ( (pll_info.qdiv != pll_def[pll].qdiv) ||
         (pll_info.pdiv != pll_def[pll].pdiv)
       )
    {
        return(TRUE);
    }

    return(FALSE);
}

clkmgr_pll_settings_t* bcm47xx_get_pll_default(clkmgr_pll_t pll)
{
    BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll < CLKMGR_PLL_COUNT);

    return((clkmgr_pll_settings_t *)&pll_def[pll]);
}


/**
    @fn bool_t tahoe_opmode_is_pll_enabled(int pll)
*/
bool_t tahoe_opmode_is_pll_enabled(int pll)
{
    bool_t ret;

	BCM_ASSERT(pll >= ECHIPMGR_PLL0);
	BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll <= ECHIPMGR_PLL3);
	BCM_ASSERT(pll <= ECLKMGR_CMPLL3);

	switch(chipmgr_get_pll_config(pll + ECHIPMGR_PLL0))
	{
		case ECHIPMGR_PLL_CONFIG_NORMAL:
        {
            uint32_t pll_stat;

            pll_stat = clkmgr_is_pll_enabled((clkmgr_pll_t)pll);
            ret = (pll_stat) ? TRUE : FALSE;
        }
        break;

        case ECHIPMGR_PLL_BYPASS_WITH_24MHZ:
		case ECHIPMGR_PLL_BYPASS_WITH_32KHZ:
        {
            ret = TRUE;
        }
        break;

		default:
        {
		    BCM_ASSERT(!"chipmgr pll configuration is invalid!");
    		ret = FALSE;
        }
        break;
    }

    return(ret);
}


/**
    @fn tahoe_stat_t tahoe_opmode_get_pll_freq(int pll, unsigned long *freq)
*/
tahoe_stat_t tahoe_opmode_get_pll_freq(int pll, unsigned long *freq)
{
	
	BCM_ASSERT(pll >= ECHIPMGR_PLL0);
	BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll <= ECHIPMGR_PLL3);
	BCM_ASSERT(pll <= ECLKMGR_CMPLL3);

	switch( chipmgr_get_pll_config(pll + ECHIPMGR_PLL0))
	{
		case ECHIPMGR_PLL_CONFIG_NORMAL:
		if( chipmgr_is_oscillator_on(ECHIPMGR_CLK_24MHZ) )
			*freq = (clkmgr_get_pll_frequency(pll + ECLKMGR_CMPLLS));
		else
			*freq = 0;
		break;

		case ECHIPMGR_PLL_BYPASS_WITH_24MHZ:
		if( chipmgr_is_oscillator_on(ECHIPMGR_CLK_24MHZ) )
			*freq =  CLKMGR_XTAL_24MHZ_FREQUENCY;
		else
			*freq = 0;
		break;

		case ECHIPMGR_PLL_BYPASS_WITH_32KHZ:
		if( chipmgr_is_oscillator_on(ECHIPMGR_CLK_32KHZ) )
			*freq =  CLKMGR_XTAL_32KHZ_FREQUENCY;
		else
			*freq = 0;
		break;

		default:
		BCM_ASSERT(!"chipmgr pll configuration is invalid!");
		*freq = 0;
		return OpInvalidChipMgrPllConfig;
	}

	return OpOk;
}


/**
    @fn tahoe_stat_t tahoe_opmode_set_pll_freq(int pll, unsigned long desired_freq,
                                           int tolerance_percent,
                                           unsigned long *actual_freq);
*/
tahoe_stat_t tahoe_opmode_set_pll_freq(int pll, unsigned long desired_freq,
                                  int tolerance_percent,
                                  unsigned long *actual_freq)
{
	BCM_ASSERT(pll >= ECHIPMGR_PLL0);
	BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll <= ECHIPMGR_PLL3);
	BCM_ASSERT(pll <= ECLKMGR_CMPLL3);

    //for 32KHz and less, we configure the pll in bypass mode.
    if ((desired_freq > CLKMGR_XTAL_32KHZ_FREQUENCY) && (desired_freq < CLKMGR_PLL_FREQ_MIN))
    {
        return(OpFreqLessThanMinFreq);    
    }

    /*
        Allow change only if pll is unused
     */
    if (bcm47xx_is_csrc_in_use((clkmgr_tcsrc_t)(pll+ECLKMGR_TCSRC_PLLS)))
    {
        return(OpCsrcInUse);    
    }

	/* if ARM11 wants to run at 32K or less, we need to set PLL1 in bypass mode...*/
	if( desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY )
	{
		/* goto PLL "bypass" mode with 32KHz clock 		*/
		if( chipmgr_config_oscillators(ECHIPMGR_CLK_32KHZ, ECHIPMGR_CLK_ON, TRUE) != BCM_OK )
			return(OpCrystalEnableFailed);

		if( chipmgr_set_pll_config(pll + ECHIPMGR_PLL0, ECHIPMGR_PLL_BYPASS_WITH_32KHZ) != BCM_OK )
			return(OpUnknownErr);

		/* TODO: in this case, we are completely ignoring tolerance percent. Which may
		 * not always be the right thing to do?
		 */
		*actual_freq = CLKMGR_XTAL_32KHZ_FREQUENCY;
		return(OpOk);
	}


	/* "normal" mode handling ... */
	if( !chipmgr_is_oscillator_on(ECHIPMGR_CLK_24MHZ) )
	{
		if( chipmgr_config_oscillators(ECHIPMGR_CLK_24MHZ, ECHIPMGR_CLK_ON, TRUE) )
			return(OpCrystalEnableFailed);
	}

	if( chipmgr_set_pll_config(pll + ECHIPMGR_PLL0, ECHIPMGR_PLL_CONFIG_NORMAL) != BCM_OK )
		return(OpUnknownErr);
	
	if( clkmgr_set_pll_frequency(pll, desired_freq, tolerance_percent, (uint32_t *)actual_freq) != BCM_OK )
		return(OpPllSetFreqFailed);

	return(OpOk);
}



tahoe_stat_t tahoe_opmode_round_rate_pll_freq(int pll, unsigned long desired_freq,
                                  int tolerance_percent,
                                  unsigned long *actual_freq)
{
	BCM_ASSERT(pll >= ECHIPMGR_PLL0);
	BCM_ASSERT(pll >= ECLKMGR_CMPLLS);
	BCM_ASSERT(pll <= ECHIPMGR_PLL3);
	BCM_ASSERT(pll <= ECLKMGR_CMPLL3);

    //for 32KHz and less, we configure the pll in bypass mode.
    if ((desired_freq > CLKMGR_XTAL_32KHZ_FREQUENCY) && (desired_freq < CLKMGR_PLL_FREQ_MIN))
    {
        return(OpFreqLessThanMinFreq);    
    }

    /*
        Allow change only if pll is unused
     */
    if (bcm47xx_is_csrc_in_use((clkmgr_tcsrc_t)(pll+ECLKMGR_TCSRC_PLLS)))
    {
        return(OpCsrcInUse);    
    }

	/* if ARM11 wants to run at 32K or less, we need to set PLL1 in bypass mode...*/
	if( desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY )
	{
		*actual_freq = CLKMGR_XTAL_32KHZ_FREQUENCY;
		return(OpOk);
	}

#if 0
	/* "normal" mode handling ... */
	if( !chipmgr_is_oscillator_on(ECHIPMGR_CLK_24MHZ) )
	{
		if( chipmgr_config_oscillators(ECHIPMGR_CLK_24MHZ, ECHIPMGR_CLK_ON, TRUE) )
			return(OpCrystalEnableFailed);
	}

	if( chipmgr_set_pll_config(pll + ECHIPMGR_PLL0, ECHIPMGR_PLL_CONFIG_NORMAL) != BCM_OK )
		return(OpUnknownErr);
#endif

	if( clkmgr_round_rate_pll_frequency(pll, desired_freq, tolerance_percent, (uint32_t *)actual_freq) != BCM_OK )
		return(OpPllSetFreqFailed);

	return(OpOk);
}

