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
 *   @file   tahoe_opmode_synctops.c 
 * 
 *   @brief  Sync top settings.
 * 
 ****************************************************************************/

#include "bcm_os_support.h"
#include "bcm_basedefs.h"
#include "bcm_privtypes.h"
#include "bcm_divide.h"

#include "tahoe_opmode.h"
#include "tahoe_stat.h"
#include "armbusmtx.h"
#include "clkmgr.h"

#define SYNCTOP_MAX_CLKMODE_N1 2

#define SYNCTOP_MAX_CLKMODE_1N 3

static tahoe_stat_t check_synctop_clkmodes(tahoe_opmode_synctop_t synctop, 
                                                    tahoe_opmode_synctop_clkmode_t clkmode);

static tahoe_stat_t check_synctop_clkratio(tahoe_opmode_synctop_t synctop, 
                                                    tahoe_opmode_synctop_mode_t synctop_mode);


static tahoe_stat_t get_synctop_master_slave_freqs(tahoe_opmode_synctop_t synctop, 
                                                   const clkmgr_cmsys_freqs_t *pcmsys_freqs, 
                                                   const clkmgr_cmahb_clk_settings_t *pcmahb_settings,
                                                   uint32_t *pmaster_freq,
                                                   uint32_t *pslave_freq);

/**
    @fn tahoe_stat_t tahoe_opmode_set_synctop(tahoe_opmode_synctop_t synctop, 
                                              const tahoe_opmode_synctop_info_t *psynctop_info);
*/
tahoe_stat_t tahoe_opmode_set_synctop(tahoe_opmode_synctop_t synctop, 
                                      const tahoe_opmode_synctop_info_t *psynctop_info)
{
    armbusmtx_synctop_info_t synctop_info = {0};
    tahoe_stat_t ret;

    if ((synctop >= TAHOE_OPMODE_SYNCTOP_COUNT) || (psynctop_info == NULL))
    {
        return(OpBadParam);
    }

    //check to see if the clkmodes restrictions are met
    ret = check_synctop_clkmodes(synctop, psynctop_info->clkmode);
    if (ret != OpOk)
    {
        bcm_log_error("%s: check_synctop_clkmodes() failed, ret [0x%x]\n", __FUNCTION__, ret);
        return(ret);
    }

    //check if the synctop can be configured in the requested mode
    ret = check_synctop_clkratio(synctop, psynctop_info->mode);
    if (ret != OpOk)
    {
        bcm_log_error("%s: check_synctop_clkratio() failed, ret [0x%x]\n", __FUNCTION__, ret);
        return(ret);
    }

    //Set the synctop settings
    synctop_info.mode            = psynctop_info->mode;          
    synctop_info.clkmode         = psynctop_info->clkmode;       
    synctop_info.high_watermark  = psynctop_info->high_watermark;
    synctop_info.low_watermark   = psynctop_info->low_watermark; 
    synctop_info.prefetch_len    = psynctop_info->prefetch_len;  

    synctop_info.clk_change = FALSE;

    (void)armbusmtx_set_synctop((armbusmtx_synctop_t)synctop, &synctop_info);

    return(OpOk);
}

/**
    @fn tahoe_stat_t tahoe_opmode_get_synctop(tahoe_opmode_synctop_t synctop, tahoe_opmode_synctop_info_t *psynctop_info);
*/
tahoe_stat_t tahoe_opmode_get_synctop(tahoe_opmode_synctop_t synctop, 
                                       tahoe_opmode_synctop_info_t *psynctop_info)
{
    armbusmtx_synctop_info_t synctop_info = {0};

    if ((synctop >= TAHOE_OPMODE_SYNCTOP_COUNT) ||
        (psynctop_info == NULL))
    {
        return(OpBadParam);
    }

    (void)armbusmtx_get_synctop((armbusmtx_synctop_t)synctop, &synctop_info);

    psynctop_info->mode           = synctop_info.mode;
    psynctop_info->clkmode        = synctop_info.clkmode;
    psynctop_info->high_watermark = synctop_info.high_watermark;
    psynctop_info->low_watermark  = synctop_info.low_watermark; 
    psynctop_info->prefetch_len   = synctop_info.prefetch_len;

    return(OpOk);
}


//Note: this function being re-used from clock switch code. Eventually, all synctop
//related apis from clock-switch will be moved into this file.

/**
    @fn static tahoe_stat_t check_synctop_clkmodes(tahoe_opmode_synctop_t synctop, 
                                                    tahoe_opmode_synctop_clkmode_t clkmode);
*/
static tahoe_stat_t check_synctop_clkmodes(tahoe_opmode_synctop_t synctop, 
                                           tahoe_opmode_synctop_clkmode_t clkmode)
{
    uint32_t idx;
    tahoe_opmode_synctop_clkmode_t cmode;
    armbusmtx_synctop_info_t synctop_info = {0};
    uint32_t clk_11, clk_n1, clk_1n;

    /**
        BCM2820 Datasheet
        9.11.4.3   Synccntrl Registers
                   For 5 sync_tops, system requires the following: 
                   Everyone can be 1 to 1 mode. 
                   2 can be in N to 1 mode and 
                   the other 3 can be in 1:N mode. 
    */

    clk_11 = clk_n1 = clk_1n = 0;
    //get clkmodes for all synctops
    for (idx = 0; idx < TAHOE_OPMODE_SYNCTOP_COUNT; idx++)
    {
        if ((uint32_t)synctop == idx)
        {
            cmode = clkmode;
        }
        else
        {
            (void)armbusmtx_get_synctop((armbusmtx_synctop_t)idx, &synctop_info);
            cmode = (tahoe_opmode_synctop_clkmode_t)synctop_info.clkmode;
        }    

        if (cmode == SYNCTOP_CLKMODE_1to1)
            clk_11++;
        else if (cmode == SYNCTOP_CLKMODE_Nto1)
            clk_n1++;
        else if (cmode == SYNCTOP_CLKMODE_1toN)
            clk_1n++;
        else
            BCM_ASSERT(!"Invalid synctop clockmode");
    }
    
    if ((clk_n1 > SYNCTOP_MAX_CLKMODE_N1) ||
        (clk_1n > SYNCTOP_MAX_CLKMODE_1N))
    {
        return(OpInvalidClockMode);
    }

    return(OpOk);
}


/**
    @fn static tahoe_stat_t check_synctop_clkratio(tahoe_opmode_synctop_t synctop, 
                                                            tahoe_opmode_synctop_mode_t synctop_mode);
*/
static tahoe_stat_t check_synctop_clkratio(tahoe_opmode_synctop_t synctop, 
                                                    tahoe_opmode_synctop_mode_t synctop_mode)
{
    uint32_t master_freq, slave_freq;
	uint32_t hi_freq, lo_freq;

    if (synctop >= TAHOE_OPMODE_SYNCTOP_COUNT)
    {
        return(OpBadParam);
    }

    master_freq = slave_freq = 0;
    //get master-slave frequencies for the given synctop
    if (get_synctop_master_slave_freqs(synctop, NULL, NULL, &master_freq, &slave_freq) != OpOk)
    {
        bcm_log_error("%s: Failed to retrieve master/slave frequencies for synctop[%d]\n", (int)synctop);
        return(OpInvalidClockRatios);
    }

    /* The synchronizers do not allow arbitrary ratios on each side.
     * if they are in "bandwidth" mode, the ratios has to be "2**n:1" or "1:2**n"
     * if they are in "latency" mode, the ratio has to be "2:1", "1:1" or "1:2"
     */

	// make freq2 >= freq1
	if (master_freq < slave_freq)
	{
		hi_freq = slave_freq;
		lo_freq = master_freq;
	}
	else
	{
		hi_freq = master_freq;
		lo_freq = slave_freq;
	}

	// we use division (bit shift to right) instead of multiplication
	// because that will easily handle the case where the "hi_freq" is an 
	// odd number...
	if (synctop_mode == SYNCTOP_MODE_LATENCY)
	{
		if( hi_freq == lo_freq || (hi_freq >> 1) == lo_freq)
		{
			return(OpOk);
		}
		else
		{
			bcm_log_error("synctop(%d) latency mode: master(%lu) and slave(%lu) are not 1:1 nor 1:2 nor 2:1\n", 
				(int)synctop, master_freq, slave_freq);
			return(OpInvalidClockRatios);
		}
	}
	else if (synctop_mode == SYNCTOP_MODE_BANDWIDTH)
	{
		/* Allowed ratio is 1:2^n where n can be 0-6 */
		int			i=0;

		do
		{
			if (lo_freq == hi_freq)
				return(OpOk);

			hi_freq >>= 1;
		} while (++i <= 6);

		bcm_log_error("synctop(%d) bandwidth mode: master_freq(%lu) and slave_freq(%lu) are not 1:2^n\n", 
					(int)synctop, master_freq, slave_freq);
		return(OpInvalidClockRatios);
	}
	else if (synctop_mode == SYNCTOP_MODE_BYPASS)
	{
		return(OpOk);
	}
	else
	{
		BCM_ASSERT(!"Invalid synctop mode");
		return(OpInvalidClockRatios);
	}

	return(OpInvalidClockRatios);
}


/**
    @fn static tahoe_stat_t get_synctop_master_slave_freqs(tahoe_opmode_synctop_t synctop, 
                                                                    const clkmgr_cmsys_freqs_t *pcmsys_freqs, 
                                                                    const clkmgr_cmahb_clk_settings_t *pcmahb_settings,
                                                                    uint32_t *pmaster_freq,
                                                                    uint32_t *pslave_freq);
*/
static tahoe_stat_t get_synctop_master_slave_freqs(tahoe_opmode_synctop_t synctop, 
                                                            const clkmgr_cmsys_freqs_t *pcmsys_freqs, 
                                                            const clkmgr_cmahb_clk_settings_t *pcmahb_settings,
                                                            uint32_t *pmaster_freq,
                                                            uint32_t *pslave_freq)
{
    clkmgr_cmahb_clk_settings_t cmahb_settings;
	clkmgr_cmsys_freqs_t		cmsys_freqs;
    unsigned long arm_apb_freq;

    if ((synctop >= TAHOE_OPMODE_SYNCTOP_COUNT) ||
        (pmaster_freq == NULL) ||
        (pslave_freq == NULL))
    {
        return(OpBadParam);
    }

    *pmaster_freq = *pslave_freq = 0;

    if (pcmsys_freqs == NULL)
    {
	    //get current frequencies of matrix/arm-ahb and vc
    	clkmgr_get_cmsys_freqs(&cmsys_freqs, NULL);
   
        //get ahb settings, needed for computing apb freq
    	clkmgr_get_cmahb_clk_settings(&cmahb_settings);
    }
    else
    {
        //This part of the code was added so that it can reused in clock-switching code, wherein
        //the new values have to be used as against the current values.
        BCM_ASSERT(pcmahb_settings != NULL);
       
        cmsys_freqs = *pcmsys_freqs;
        cmahb_settings = *pcmahb_settings;
    }

    //compute arm-apb frequency
    arm_apb_freq = bcm_udivide32_round(cmsys_freqs.armahb_freq,  cmahb_settings.apdiv);

    /*
     *   Information from clock_switch.c that was originally from an email from suresh.
     *
     *   Synchronizer           Master                Slave
     *      AI                  ARM AHB             ARM MATRIX
     *      AO                  ARM MATRIX          ARM AHB
     *      PI                  ARM APB             ARM Matrix
     *      VI                  VC2 AHB             ARM MATRIX
     *      VO                  ARM MATRIX          VC2 AHB
     *     
	 */
     
    switch(synctop)
    {
        case SYNCTOP_AI :
            *pmaster_freq = cmsys_freqs.armahb_freq;
            *pslave_freq = cmsys_freqs.mtx_freq;
        break;

        case SYNCTOP_AO :
            *pmaster_freq = cmsys_freqs.mtx_freq;
            *pslave_freq = cmsys_freqs.armahb_freq;
        break;

        case SYNCTOP_PI :
            *pmaster_freq = arm_apb_freq;
            *pslave_freq = cmsys_freqs.mtx_freq;
        break;

        case SYNCTOP_VI :
            *pmaster_freq = cmsys_freqs.vc_freq;
            *pslave_freq = cmsys_freqs.mtx_freq;
        break;

        case SYNCTOP_VO :
            *pmaster_freq = cmsys_freqs.mtx_freq;
            *pslave_freq = cmsys_freqs.vc_freq;
        break;

        default:
            BCM_ASSERT(!"Invalid synctop");
            return(OpBadParam);        
    }

    return(OpOk);
}

