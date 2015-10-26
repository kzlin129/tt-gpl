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
 *   @file   tahoe_opmode_arm11.c 
 * 
 *   @brief  Implements interface apis for setting all arm related functionality.
 * 
 ****************************************************************************/

#include "bcm_os_support.h"
#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_divide.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "tahoe_opmode.h"
#include "tahoe_opmode_priv.h"
#include "tahoe_stat.h"
// #include "tahoe_pmu.h"

#define TAHOE_OPMODE_ARM_DEFAULT_CSRC ECLKMGR_TCSRC_PLL1

static int bcm47xx_opmode_find_better_csrc(clkmgr_freq_info_t *pfreq, 
                                           uint32_t *pbest_div, 
                                           uint32_t *pbest_csrc, 
                                           unsigned long *pbest_freq);

static tahoe_stat_t bcm47xx_opmode_round_rate_change_arm11_csrc_freq(clkmgr_freq_info_t *pfreq, 
                                                          clkmgr_tcsrc_t csrc,
                                                          uint32_t *pnew_div,                                                     
                                                          unsigned long *pactual_freq);

static tahoe_stat_t bcm47xx_opmode_change_arm11_csrc_freq(clkmgr_freq_info_t *pfreq, 
                                                          clkmgr_tcsrc_t csrc,
                                                          uint32_t *pnew_div,                                                     
                                                          unsigned long *pactual_freq);

static tahoe_stat_t bcm47xx_opmode_set_arm11_at_32KHz(clkmgr_freq_info_t *pfreq, 
                                                      unsigned long *pactual_freq);


static tahoe_stat_t bcm47xx_opmode_compute_arm11_freq(clkmgr_tcsrc_t csrc, uint32_t div, 
                                                      unsigned long *pfrequency);

static tahoe_stat_t bcm47xx_opmode_set_arm11_freq(clkmgr_tcsrc_t csrc, uint32_t div);

static tahoe_stat_t bcm47xx_opmode_try_csrc_for_arm11(clkmgr_tcsrc_t csrc, 
                                                      clkmgr_freq_info_t *pfreq,
                                                      unsigned long *pactual_freq);
static tahoe_stat_t bcm47xx_opmode_round_rate_try_csrc_for_arm11(clkmgr_tcsrc_t csrc, 
                                                      clkmgr_freq_info_t *pfreq,
                                                      unsigned long *pactual_freq);
/**
    @fn tahoe_stat_t tahoe_opmode_get_arm11_freq(unsigned long *pfrequency)
*/
tahoe_stat_t tahoe_opmode_get_arm11_freq(unsigned long *pfrequency)
{
    clkmgr_tcsrc_t arm_csrc;
    uint32_t div, ret;

	if(pfrequency == NULL)
		return(OpBadParam);

    //get the arm clk settings
    ret = clkmgr_get_arm_clk(&arm_csrc, &div);
    if (ret != BCM_OK)
    {
		*pfrequency = 0;
        return(OpUnknownErr);
    }

    return(bcm47xx_opmode_compute_arm11_freq(arm_csrc, div, pfrequency));
}

static tahoe_stat_t bcm47xx_opmode_round_rate_try_csrc_for_arm11(clkmgr_tcsrc_t csrc, 
                                                      clkmgr_freq_info_t *pfreq,
                                                      unsigned long *pactual_freq)
{
    uint32_t new_div;
    uint32_t ret;      
    // unsigned long old_freq, new_freq;
    tahoe_stat_t opstat = OpOk;


	//Are there any peripherals using the given clock source?
	ret = bcm47xx_is_csrc_in_use(csrc);
	if ((ret & CSRC_USED_BY_PERIP) || (ret & CSRC_USED_BY_SYS))
	{
		//Yes, there are one or more peripherals using the clock source.
			//ERROR - Unable to change ARM frequency
		bcm_log_debug("%s: Some peripheral/sys is using the given clock source, hence arm11 freq cannot be changed\n",
						__FUNCTION__);
		return(OpClkSrcCannotBchanged);

	}

	//Now, modify the preallocated PLL to obtain the desired frequency.
    opstat = bcm47xx_opmode_round_rate_change_arm11_csrc_freq(pfreq, csrc, &new_div, pactual_freq); 
    if (opstat != OpOk)
        return(opstat);

    return(OpOk);
}


/** 
    @fn tahoe_stat_t tahoe_opmode_set_arm11_freq(unsigned long desired_freq,  int tolerance_percent, unsigned long *pactual_freq);
*/
tahoe_stat_t tahoe_opmode_set_arm11_freq(unsigned long desired_freq,  
                                         int tolerance_percent, 
                                         unsigned long *pactual_freq)
{
    clkmgr_freq_info_t freq = { 0 };
    uint32_t new_div;
    clkmgr_tcsrc_t arm_default_csrc;
    clkmgr_tcsrc_t new_source;
    uint32_t ret;      
  	uint32_t 	  current_deviation_percent;
    int old_div;
    clkmgr_tcsrc_t old_csrc;
	tahoe_opmode_csrc_t	opmode_csrc;
    tahoe_stat_t opstat = OpOk;

    BCM_ASSERT(pactual_freq != NULL);

    freq.desired_freq = desired_freq;
    freq.max_div = CLKMGR_ARMCLK_DIV_MAX;
    freq.tolerance_percent = tolerance_percent;

    if (desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY)
    {
        opstat = bcm47xx_opmode_set_arm11_at_32KHz(&freq, pactual_freq);   
        if ((opstat == OpOk) || ((opstat != OpOk) && (tolerance_percent != 0)))
            return(opstat);

        //else let us try to find the next best possible frequency as tolerance == 0
        //the code following this if will take care of this condition
    }

    //Get the current(going to be old) csrc for arm
    tahoe_opmode_get_core_csrc_and_div(CORE_ARM, &opmode_csrc, &old_div);
	old_csrc = xlate_opmode_csrc_to_clkmgr_csrc(opmode_csrc); 

    /**
        Earlier, we were using the default clock source for arm based on
        the mode's default settings. However, once the user sets the mode,
        there may be a need to tweak arm frequency. In order to accomodate
        this, the code now ignores the mode defaults and sets the clock 
        source based on the desired frequency.

        Also, if the desired frequency can be obtained by an oscillator,
        then use it instead of a PLL. 
    */
    if ((desired_freq >= CLKMGR_XTAL_32KHZ_FREQUENCY) &&
        (desired_freq <= CLKMGR_XTAL_24MHZ_FREQUENCY))
    {
        arm_default_csrc = ECLKMGR_CSRC_XTAL_24MHZ;        
    }
    else
    {
       arm_default_csrc = TAHOE_OPMODE_ARM_DEFAULT_CSRC; //default
    }

    new_div = 0;
    *pactual_freq = 0;
    //Can we use the current preallocated PLL to obtain the desired_freq?
    ret = bcm47xx_is_suitable_csrc(arm_default_csrc, &freq, &new_div, pactual_freq);
        
    //Yes, we can use the preallocated PLL.
    if ((ret == TRUE) && (tolerance_percent != 0))
    {
         bcm_log_debug("%s: Preallocated source [%d] can be used, desired freq[%ld] actual freq[%ld]\n", 
                            __FUNCTION__, arm_default_csrc, desired_freq, *pactual_freq);

         opstat = bcm47xx_opmode_set_arm11_freq(arm_default_csrc, new_div);
         if (opstat != OpOk)
            return(opstat);

         //can we disable the old clock source?
         if (old_csrc != arm_default_csrc)
         {
             ret = bcm47xx_is_csrc_in_use(old_csrc);
             if ((ret == CSRC_UNUSED) || (ret == CSRC_USED_BY_ARM))
             {
                 if (bcm47xx_disable_csrc(old_csrc))
            		     bcm_log_warn("%s: Unable to disable unused csrc [%d]\n", __FUNCTION__, old_csrc);
             }
         }

        return(OpOk);
    }

    //Yes, we can use the preallocated PLL.
    if ((ret == TRUE) && (tolerance_percent == 0))
	{
        new_source = arm_default_csrc;
      	current_deviation_percent = bcm_get_deviation_percent(desired_freq, *pactual_freq);

        if (current_deviation_percent != 0)
        {
		    /* 
                The caller wants a frequency that is equal to or closer to the desired frequency.
                We can use the current clock source. But is there a different clock source that
                can do better?
		    */
            (void)bcm47xx_opmode_find_better_csrc(&freq, &new_div, &new_source, pactual_freq);
        }

        opstat = bcm47xx_opmode_set_arm11_freq(new_source, new_div);
        if (opstat != OpOk)
           return(opstat);

        //can we disable the old clock source?
        if (old_csrc != new_source)
        {
            ret = bcm47xx_is_csrc_in_use(old_csrc);
            if ((ret == CSRC_UNUSED) || (ret == CSRC_USED_BY_ARM))
            {
                if (bcm47xx_disable_csrc(old_csrc))
            		    bcm_log_warn("%s: Unable to disable unused csrc [%d]\n", __FUNCTION__, old_csrc);
            }
        }

        return(OpOk);
	}

    //No, we can't use the preallocated PLL.
    //Find a different source, other than the system PLL to see if we can use.
    bcm_log_debug("%s: Cannot use preallocated source [%d]\n", __FUNCTION__, arm_default_csrc);

	ret = bcm47xx_find_suitable_source(&freq, ECLKMGR_CLK_USE_FEW_SRC, &new_source, &new_div, pactual_freq);
	if (ret == BCM_OK )
	{        
		bcm_log_debug("%s: Found a suitable source [%d], new_divider[%d]\n", __FUNCTION__, new_source, new_div);

		//Yes, we found a source that would provide ARM the desired frequency.
        opstat = bcm47xx_opmode_set_arm11_freq(new_source, new_div);
        if (opstat != OpOk)
           return(opstat);

        //can we disable the old clock source?
        if (old_csrc != new_source)
        {
            ret = bcm47xx_is_csrc_in_use(old_csrc);
            if ((ret == CSRC_UNUSED) || (ret == CSRC_USED_BY_ARM))
            {
                if (bcm47xx_disable_csrc(old_csrc))
            		    bcm_log_warn("%s: Unable to disable unused csrc [%d]\n", __FUNCTION__, old_csrc);
            }       
        }

		return(OpOk);
	}

    //can we modify the current source to obtain the desired frequency?
    // i.e. if no one else is using it.
    opstat =  bcm47xx_opmode_try_csrc_for_arm11(old_csrc, &freq, pactual_freq);
    if (opstat == OpOk)
    {
        return(OpOk);
    }

  	bcm_log_debug("%s: Current clock source[%d] cannot be changed\n", 
    	  			  __FUNCTION__, old_csrc); 

    //As a last resort, can we modify the default source to obtain the desired frequency?
    // i.e. if no one else is using it.
    opstat =  bcm47xx_opmode_try_csrc_for_arm11(arm_default_csrc, &freq, pactual_freq);
    if (opstat != OpOk)
    {
        return(opstat);
    }

    return(OpOk);
}


/** 
    @fn tahoe_stat_t tahoe_opmode_round_rate_arm_freq(unsigned long desired_freq,  int tolerance_percent, unsigned long *pactual_freq);
*/
tahoe_stat_t tahoe_opmode_round_rate_arm_freq(unsigned long desired_freq,  
                                         int tolerance_percent, 
                                         unsigned long *pactual_freq)
{
    clkmgr_freq_info_t freq = { 0 };
    uint32_t new_div;
    clkmgr_tcsrc_t arm_default_csrc;
    clkmgr_tcsrc_t new_source;
    uint32_t ret;      
  	uint32_t 	  current_deviation_percent;
    int old_div;
    clkmgr_tcsrc_t old_csrc;
	tahoe_opmode_csrc_t	opmode_csrc;
    tahoe_stat_t opstat = OpOk;

    BCM_ASSERT(pactual_freq != NULL);

    freq.desired_freq = desired_freq;
    freq.max_div = CLKMGR_ARMCLK_DIV_MAX;
    freq.tolerance_percent = tolerance_percent;

    if (desired_freq <= CLKMGR_XTAL_32KHZ_FREQUENCY)
    {
			*pactual_freq = CLKMGR_XTAL_32KHZ_FREQUENCY ;
            return(opstat);
    }

    //Get the current(going to be old) csrc for arm
    tahoe_opmode_get_core_csrc_and_div(CORE_ARM, &opmode_csrc, &old_div);
	old_csrc = xlate_opmode_csrc_to_clkmgr_csrc(opmode_csrc); 

    /**
        Earlier, we were using the default clock source for arm based on
        the mode's default settings. However, once the user sets the mode,
        there may be a need to tweak arm frequency. In order to accomodate
        this, the code now ignores the mode defaults and sets the clock 
        source based on the desired frequency.

        Also, if the desired frequency can be obtained by an oscillator,
        then use it instead of a PLL. 
    */
    if ((desired_freq >= CLKMGR_XTAL_32KHZ_FREQUENCY) &&
        (desired_freq <= CLKMGR_XTAL_24MHZ_FREQUENCY))
    {
        arm_default_csrc = ECLKMGR_CSRC_XTAL_24MHZ;        
    }
    else
    {
       arm_default_csrc = TAHOE_OPMODE_ARM_DEFAULT_CSRC; //default
    }

    new_div = 0;
    *pactual_freq = 0;
    //Can we use the current preallocated PLL to obtain the desired_freq?
    ret = bcm47xx_is_suitable_csrc(arm_default_csrc, &freq, &new_div, pactual_freq);
        
    //Yes, we can use the preallocated PLL.
    if ((ret == TRUE) && (tolerance_percent != 0)) { return(OpOk); }

    //Yes, we can use the preallocated PLL.
    if ((ret == TRUE) && (tolerance_percent == 0))
	{
        new_source = arm_default_csrc;
      	current_deviation_percent = bcm_get_deviation_percent(desired_freq, *pactual_freq);

        if (current_deviation_percent != 0)
        {
		    /* 
                The caller wants a frequency that is equal to or closer to the desired frequency.
                We can use the current clock source. But is there a different clock source that
                can do better?
		    */
            (void)bcm47xx_opmode_find_better_csrc(&freq, &new_div, &new_source, pactual_freq);
        }

        return(OpOk);
	}

    //No, we can't use the preallocated PLL.
    //Find a different source, other than the system PLL to see if we can use.
    printk("Cannot use preallocated source [%d]\n", arm_default_csrc);

	ret = bcm47xx_find_suitable_source(&freq, ECLKMGR_CLK_USE_FEW_SRC, &new_source, &new_div, pactual_freq);
	if (ret == BCM_OK )
	{        
		printk("Found a suitable source [%d], new_divider[%d]\n", new_source, new_div);

		//Yes, we found a source that would provide ARM the desired frequency.
		return(OpOk);
	}

    //can we modify the current source to obtain the desired frequency?
    // i.e. if no one else is using it.
    opstat =  bcm47xx_opmode_round_rate_try_csrc_for_arm11(old_csrc, &freq, pactual_freq);
    if (opstat == OpOk)
    {
        return(OpOk);
    }

  	bcm_log_debug("%s: Current clock source[%d] cannot be changed\n", 
    	  			  __FUNCTION__, old_csrc); 

    //As a last resort, can we modify the default source to obtain the desired frequency?
    // i.e. if no one else is using it.
    opstat =  bcm47xx_opmode_round_rate_try_csrc_for_arm11(arm_default_csrc, &freq, pactual_freq);
    if (opstat != OpOk)
    {
        return(opstat);
    }

    return(OpOk);
}


#if 0
/**
    @fn unsigned long bcm47xx_opmode_compute_arm11_default(tahoe_opmode_mode_t mode);
*/
unsigned long bcm47xx_opmode_arm11_default_freq(tahoe_opmode_mode_t mode)
{
    unsigned long freq = 0, arm11_freq = 0;
    clkmgr_pll_settings_t* pll_def;
    clkmgr_tcsrc_t arm_default_csrc;
    tahoe_opmode_mode_info_t* pmode_info;

    BCM_ASSERT(mode < OPMODE_DEFAULT_MODE);

    pmode_info = tahoe_get_opmode_default(mode);
    arm_default_csrc = pmode_info->arm11_clk.csrc;  

    if (arm_default_csrc == ECLKMGR_CSRC_NONE)
    {
        return(0);
    }
    else if (arm_default_csrc == ECLKMGR_CSRC_XTAL_24MHZ)
    {
    	freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
    }
    else if ((arm_default_csrc >= ECLKMGR_TCSRC_PLLS) &&     //Is a PLL the source?
             (arm_default_csrc <= ECLKMGR_MAX_TCSRC))
    {
        pll_def = bcm47xx_get_pll_default((clkmgr_pll_t)(arm_default_csrc - ECLKMGR_TCSRC_PLLS));
        //the default PLL for arm11 is always PLL1 according to our static allocation
        freq = (CLKMGR_XTAL_24MHZ_FREQUENCY * pll_def->qdiv)/pll_def->pdiv;
    }

    arm11_freq = freq/(pmode_info->arm11_clk.div);

    return(arm11_freq);
}
#endif

/**
    @fn tahoe_stat_t bcm47xx_opmode_set_arm_csrc_div(clkmgr_tcsrc_t csrc, 
                                                uint32_t div);
*/
tahoe_stat_t bcm47xx_opmode_set_arm_csrc_div(clkmgr_tcsrc_t csrc, 
                                                      uint32_t div)
{
    uint32_t ret, csrc_stat;
    clkmgr_pll_t pll;

    //check to see if the required source is enabled.
    csrc_stat = bcm47xx_is_csrc_enabled(csrc);

    //Enable the source PLL 
    if (csrc_stat == disable) //easy case, just enable the pll before setting the arm clk
    {
        bcm_log_debug("%s : Clock source [%d] required for arm has to be enabled\n", __FUNCTION__, 
                        csrc);

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

        ret = bcm47xx_opmode_set_arm11_freq(csrc, div);
        if (ret != OpOk)
            return(ret);	
    }// end-if pll was disabled
    else //pll is already enabled
    {
        bcm_log_debug("%s : Clock source[%d] is already  enabled\n", 
                        __FUNCTION__, csrc);

        ret = bcm47xx_opmode_set_arm11_freq(csrc, div);
        if (ret != OpOk)
            return(ret);
    }//end-else - pll is already enabled

    return(OpOk);
}


/**
    @fn tahoe_stat_t bcm47xx_opmode_switch_arm11_to_stable_csrc(unsigned long *pold_freq, unsigned long *pnew_freq);
*/
tahoe_stat_t bcm47xx_opmode_switch_arm11_to_stable_csrc(unsigned long *pold_freq, unsigned long *pnew_freq)
{
    tahoe_stat_t ret;

    BCM_ASSERT((pold_freq != NULL) && (pnew_freq != NULL));

    *pnew_freq = 0;

    //get the current ARM frequency
    *pold_freq = (unsigned long)clkmgr_get_arm_frequency();

    //For now temporarlily switch ARM to 24MHz crystal. 
    //A better solution would be to scale up and down the desired frequency 
    //and find a clock source closest to the desired frequency. This optimization
    //will be added later.

    //Is the 24MHz crystal on??
    if (bcm47xx_is_csrc_enabled(ECLKMGR_TCSRC_XTAL_24MHZ) == FALSE)
    {
        //Turn it on... and wait till it's on
        if (chipmgr_config_oscillators(ECHIPMGR_CLK_24MHZ, ECHIPMGR_CLK_ON, TRUE) == BCM_ERROR)
        {
            //we are in trouble..
            bcm_log_debug("%s : Failed to turn ON the 24MHz crystal\n", __FUNCTION__);
            return(OpCrystalEnableFailed);
        }
    }

    ret = bcm47xx_opmode_set_arm11_freq(ECLKMGR_TCSRC_XTAL_24MHZ, 1);
    if (ret != OpOk)
        return(ret);

    *pnew_freq = (unsigned long)CLKMGR_XTAL_24MHZ_FREQUENCY;
    return(OpOk);
}

/**
    @fn tahoe_stat_t bcm47xx_opmode_switch_arm11_back_to_old_freq(clkmgr_tcsrc_t csrc, unsigned long arm_freq);
*/
tahoe_stat_t bcm47xx_opmode_switch_arm11_back_to_old_freq(clkmgr_tcsrc_t csrc, unsigned long arm_freq)
{
    uint32_t ret, new_div;
    clkmgr_freq_info_t freq;
    clkmgr_tcsrc_t new_csrc;
    unsigned long actual_arm_freq;

    //Now that the PLL is stable, switch back ARM to this PLL
    freq.desired_freq = arm_freq;
    freq.tolerance_percent = 0;
    freq.max_div = CLKMGR_ARMCLK_DIV_MAX;

    ret = bcm47xx_is_suitable_csrc(csrc, &freq, &new_div, &actual_arm_freq);
    if (ret == TRUE)
    {
        //The current source itself is suitable, change the divider and arm is set
        ret = bcm47xx_opmode_set_arm11_freq(csrc, new_div);
        if (ret != OpOk)
            return(ret);
    }
    else
    {
        //We need to find a new source that can be used for arm.
        ret = bcm47xx_find_suitable_source(&freq, ECLKMGR_CLK_USE_FEW_SRC, &new_csrc, &new_div, &actual_arm_freq);
        if (ret == BCM_ERROR)
        {
            //Note: ARM was temporarily switched to a different frequency above. Hence
            //      it is set to "some" frequency though not a desirable one.
            bcm_log_error("bcm47xx_opmode: Unable to find suitable source for ARM!!\n");
            return(OpNoSuitableClkSrc);
        }
        else
        {
            //we found a new source, change the source and divider
            ret = bcm47xx_opmode_set_arm11_freq(new_csrc, new_div);
            if (ret != OpOk)
                return(ret);                        
        }
    }

    return(OpOk);
}

/**
    Static APIs local to this file...
*/
static int bcm47xx_opmode_find_better_csrc(clkmgr_freq_info_t *pfreq, 
                                           uint32_t *pbest_div, 
                                           uint32_t *pbest_csrc, 
                                           unsigned long *pbest_freq)
{
	uint32_t      new_div;
	uint32_t	  new_csrc;
	unsigned long new_freq;
  	uint32_t 	  current_deviation_percent;
	uint32_t 	  new_deviation_percent;
    uint32_t      ret;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pbest_div != NULL);
    BCM_ASSERT(pbest_csrc != NULL);
    BCM_ASSERT(pbest_freq != NULL);

	bcm_log_debug("%s: The best so far: best_div[%ld] best_csrc[%ld] desired freq[%ld] best_freq[%ld]\n", 
                        __FUNCTION__, *pbest_div, *pbest_csrc, pfreq->desired_freq, *pbest_freq);

	ret = bcm47xx_find_suitable_source(pfreq, ECLKMGR_CLK_USE_FEW_SRC, &new_csrc, &new_div, &new_freq);
    if (ret != BCM_OK)
    {
        //the current source is the best source...
        return(BCM_OK);
    }

    //looks like we can do better
	current_deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, *pbest_freq);
    new_deviation_percent = bcm_get_deviation_percent(pfreq->desired_freq, new_freq);

    if (new_deviation_percent < current_deviation_percent)
    {
        //We found a better source!!
         *pbest_div = new_div;
        *pbest_csrc = new_csrc;
        *pbest_freq = new_freq;
    }
    //else
    //{
        //the current source is the best source...
    //}

    return(BCM_OK);
}


static tahoe_stat_t bcm47xx_opmode_round_rate_change_arm11_csrc_freq(clkmgr_freq_info_t *pfreq, 
                                                          clkmgr_tcsrc_t csrc,
                                                          uint32_t *pnew_div,                                                       
                                                          unsigned long *pactual_freq)
{
    tahoe_stat_t ret;
    clkmgr_pll_t pll;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pnew_div != NULL);
    BCM_ASSERT(csrc < CLKMGR_TCSRC_COUNT);
    BCM_ASSERT(pactual_freq != NULL);

    if (csrc < ECLKMGR_TCSRC_PLLS)
        return(OpOk);

    pll = (clkmgr_pll_t)(csrc - ECLKMGR_TCSRC_PLLS);

    //Set the csrc, in this case a PLL, to the desired frequency. 
    //Note : in this case arm11 frequency == pll frequency == desired frequency

    ret = bcm47xx_opmode_round_rate_set_up_pll(pll, pfreq, pnew_div, pactual_freq);

    return(ret);
}

static tahoe_stat_t bcm47xx_opmode_change_arm11_csrc_freq(clkmgr_freq_info_t *pfreq, 
                                                          clkmgr_tcsrc_t csrc,
                                                          uint32_t *pnew_div,                                                       
                                                          unsigned long *pactual_freq)
{
    tahoe_stat_t ret;
    clkmgr_pll_t pll;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pnew_div != NULL);
    BCM_ASSERT(csrc < CLKMGR_TCSRC_COUNT);
    BCM_ASSERT(pactual_freq != NULL);

    if (csrc < ECLKMGR_TCSRC_PLLS)
        return(OpOk);

    pll = (clkmgr_pll_t)(csrc - ECLKMGR_TCSRC_PLLS);

    //Set the csrc, in this case a PLL, to the desired frequency. 
    //Note : in this case arm11 frequency == pll frequency == desired frequency

    ret = bcm47xx_opmode_set_up_pll(pll, pfreq, pnew_div, pactual_freq);

    return(ret);
}


static tahoe_stat_t bcm47xx_opmode_set_arm11_at_32KHz(clkmgr_freq_info_t *pfreq, unsigned long *pactual_freq)
{
    tahoe_opmode_csrc_t current_csrc, new_csrc;
	clkmgr_tcsrc_t		current_clkmgr_csrc;
    uint32_t current_div, new_div;
    uint32_t ret;
    clkmgr_pll_t pll;
    unsigned long old_freq, new_freq;

    BCM_ASSERT(pfreq != NULL);
    BCM_ASSERT(pactual_freq != NULL);

    if (pfreq->desired_freq < CLKMGR_XTAL_32KHZ_FREQUENCY)
    {
		bcm_log_debug("%s: ARM @ less than 32KHz is not supported at this time\n", __FUNCTION__);       
        return(OpFreqLessThanMinFreq);
    }

    //Get the current(going to be old) csrc for arm
    tahoe_opmode_get_core_csrc_and_div(CORE_ARM, &current_csrc, &current_div);
	current_clkmgr_csrc = xlate_opmode_csrc_to_clkmgr_csrc(current_csrc);

    //Check if the current src is being used by anyone else?
	ret = bcm47xx_is_csrc_in_use(current_clkmgr_csrc);
	if ((ret & CSRC_USED_BY_PERIP) || (ret & CSRC_USED_BY_SYS))
	{
		//Yes, there are one or more peripherals using the preallocated PLL as a source.
		bcm_log_debug("%s: Some peripheral/sys is using the current arm clk source\n", __FUNCTION__);
	}
    else
    {
        if (ret & CSRC_USED_BY_ARM)
        {
            //switch to oscillator before changing the default csrc frequency
            ret = bcm47xx_opmode_switch_arm11_to_stable_csrc(&old_freq, &new_freq);
            if (ret != OpOk)
            {
                return(ret);
            }
        }

        //Set up pll at 32KHz
        pll = (clkmgr_pll_t)(current_clkmgr_csrc - ECLKMGR_TCSRC_PLLS);
        ret = bcm47xx_opmode_set_up_pll(pll, pfreq, &new_div, pactual_freq);

        //Set up ARM current src
        ret = bcm47xx_opmode_set_arm11_freq(current_clkmgr_csrc, new_div);

        return(ret);
    }

    //Someone else is tied to the clk src used by arm, we need find a different src.
    //Since we need 32KHz, we have to use a PLL. Find a PLL that is not being used at this time.
    *pactual_freq = 0;
    ret = bcm47xx_find_disabled_source(pfreq, ECLKMGR_CLK_USE_FEW_SRC, (clkmgr_tcsrc_t *)&new_csrc, &new_div, pactual_freq);
    if (ret == BCM_OK)
    {
        //We found a disabled PLL that we can use.
        bcm_log_debug("%s : We found a disabled pll that can be a suitable source\n",__FUNCTION__);

         //Set up ARM clock src
        ret = bcm47xx_opmode_set_arm11_freq(new_csrc, new_div);

        return(ret);
    }

    //We did not find any PLL, so can't satisfy the request.
    return(OpNoSuitableClkSrc);
}


static tahoe_stat_t bcm47xx_opmode_compute_arm11_freq(clkmgr_tcsrc_t csrc, uint32_t div, 
                                                      unsigned long *pfrequency)
{
	if(pfrequency == NULL)
		return(OpBadParam);

    if (!bcm47xx_is_csrc_enabled(csrc))
    {
        *pfrequency = 0;
        return(OpOk);
    }

    if (div == 0)
        div = CLKMGR_ARMCLK_DIV_MAX;

    if (csrc == ECLKMGR_TCSRC_XTAL_24MHZ)
    {
        *pfrequency = bcm_udivide32_round(CLKMGR_XTAL_24MHZ_FREQUENCY, div);
    }
    else if ((csrc >= ECLKMGR_TCSRC_PLLS) && (csrc <= ECLKMGR_TCSRC_PLL3))
    {
        unsigned long pll_freq = 0;

        tahoe_opmode_get_pll_freq(csrc - ECLKMGR_TCSRC_PLLS, &pll_freq);
        *pfrequency = bcm_udivide32_round(pll_freq, div);
    }

	return(OpOk);
}


static tahoe_stat_t bcm47xx_opmode_try_csrc_for_arm11(clkmgr_tcsrc_t csrc, 
                                                      clkmgr_freq_info_t *pfreq,
                                                      unsigned long *pactual_freq)
{
    uint32_t new_div;
    uint32_t ret;      
    unsigned long old_freq, new_freq;
    tahoe_stat_t opstat = OpOk;


	//Are there any peripherals using the given clock source?
	ret = bcm47xx_is_csrc_in_use(csrc);
	if ((ret & CSRC_USED_BY_PERIP) || (ret & CSRC_USED_BY_SYS))
	{
		//Yes, there are one or more peripherals using the clock source.
			//ERROR - Unable to change ARM frequency
		bcm_log_debug("%s: Some peripheral/sys is using the given clock source, hence arm11 freq cannot be changed\n",
						__FUNCTION__);
		return(OpClkSrcCannotBchanged);

	}

    //switch to oscillator before changing the clock source frequency
    opstat = bcm47xx_opmode_switch_arm11_to_stable_csrc(&old_freq, &new_freq);
    if (opstat != OpOk)
        return(opstat);

	//No, modify the preallocated PLL to obtain the desired frequency.
    opstat = bcm47xx_opmode_change_arm11_csrc_freq(pfreq, csrc, &new_div, pactual_freq); 
    if (opstat != OpOk)
        return(opstat);

	//Since the clock source frequency has been adjusted to obtain the
	//desired frequency, switch ARM back to the desired clock source.
    opstat = bcm47xx_opmode_set_arm11_freq(csrc, new_div);
    if (opstat != OpOk)
       return(opstat);	 

	bcm_log_debug("%s: Given clock source[%d] reset to change arm11 freq\n", 
					__FUNCTION__, csrc);        

    return(OpOk);
}

// #define VOLTAGE_CHANGE 1

static tahoe_stat_t bcm47xx_opmode_set_arm11_freq(clkmgr_tcsrc_t csrc, uint32_t div)
{
    tahoe_stat_t stat = OpOk;
    unsigned long current_freq = 0 , desired_freq = 0;
#ifdef VOLTAGE_CHANGE
    unsigned int voltage = 0;
#endif
    //get current arm frequency.
    stat = tahoe_opmode_get_core_freq(CORE_ARM, &current_freq);
    if (stat != OpOk)
        goto SET_ARM11_EXIT;

    //compute the new arm frequency with the given csrc and div
    stat = bcm47xx_opmode_compute_arm11_freq(csrc, div, &desired_freq);
    if (stat != OpOk)
        goto SET_ARM11_EXIT;

    //get voltage for new frequency
#ifdef VOLTAGE_CHANGE
    stat = compute_new_output_voltage(desired_freq,OUTPUT_VOLTAGE_CALC_ARM_TYPE , &voltage);
    if (stat != OpOk)
        goto SET_ARM11_EXIT;
#endif

    /**
        Note: If new frequency is more than current frequency, change voltage before changing frequency.
              else change frequency before changing voltage.
    */

#ifdef VOLTAGE_CHANGE
    if (desired_freq > current_freq)
    {
        stat = tahoe_opmode_set_output_voltage(VR_1_2_RAIL, voltage);
        if (stat != OpOk)
            goto SET_ARM11_EXIT;
    }
#endif

    // bcm_cpufreq_prechange_handler(current_freq, desired_freq);

    //Change arm frequency
    stat = clkmgr_set_arm_clk(csrc, div);
    if (stat != BCM_OK)
    {
        // Note: currently there is no way to suspend the freq change under progress.
        //       On the bright side, the clkmgr_set_arm_clk is a primitive function that
        //       modifies the clock source and divider. The chances of that failing are 
        //       pretty low. Hence, we can leave without a suspend handler.
        // bcm_cpufreq_postchange_handler(current_freq, desired_freq); 
        stat = OpArm11SetClkFailed;
        goto SET_ARM11_EXIT;
    }

    // bcm_cpufreq_postchange_handler(current_freq, desired_freq);

#ifdef VOLTAGE_CHANGE
    //change voltage if moving to a lower frequency
    //Note: don't go by the variable names... actually current_freq is previous and desired is current.
    if (desired_freq < current_freq)
    {
        stat = tahoe_opmode_set_output_voltage(VR_1_2_RAIL, voltage);
        if (stat != OpOk)
            goto SET_ARM11_EXIT;
    }
#endif

SET_ARM11_EXIT:

    bcm_log_debug("%s : return status[0x%x]\n",__FUNCTION__, stat);    
    return(stat);    
}

