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
 *   @file   tahoe_opmode_low_level.c 
 * 
 *   @brief  Wrappers for low level apis.
 * 
 ****************************************************************************/

#include "bcm_basedefs.h"
#include "bcm_divide.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "tahoe_opmode.h"
#include "tahoe_opmode_priv.h"


/**
    The user app will invoke apis in the opmode layer only and the opmode will provide
    wrappers to the low-level apis. 
 */

/**
    tahoe_stat_t tahoe_opmode_get_vcsys_freq(unsigned long *pmtx_freq, unsigned long *pvc_freq, unsigned long *armahb_freq);

    Note: This api implementation will change to not use the clkmgr interface alone because the PLL can be
          in bypass mode and that is available in the chip manager. Eventually, this function will use a 
          combination of the clkmgr and chipmgr low-level api's.
 */

tahoe_stat_t tahoe_opmode_get_vcsys_freq(unsigned long *pmtx_freq, unsigned long *pvc_freq, unsigned long *parmahb_freq)
{
    int rc;
    clkmgr_cmsys_freqs_t vcsys_freqs;

    BCM_ASSERT(pmtx_freq != NULL);    
    BCM_ASSERT(pvc_freq != NULL);
    BCM_ASSERT(parmahb_freq != NULL);

    rc = clkmgr_get_cmsys_freqs(&vcsys_freqs, NULL);
    if (rc == BCM_ERROR)
    {
        return(OpVcSysGetFreqFailed);
    }

    *pmtx_freq    = vcsys_freqs.mtx_freq;
    *pvc_freq     = vcsys_freqs.vc_freq;
    *parmahb_freq = vcsys_freqs.armahb_freq;

    return(OpOk);
}

/**
  tahoe_opmode_mode_t tahoe_opmode_get_cmsys_clk_settings(unsigned int div, unsigned int mtxdiv, unsigned int ahdiv, unsigned int vhdiv );
*/
tahoe_stat_t	tahoe_opmode_get_cmsys_clk_settings(tahoe_opmode_csrc_t *csrc, unsigned int *div, unsigned int *mtxdiv, unsigned int *ahdiv, unsigned int *vhdiv )
{
	clkmgr_cmsys_clk_settings_t	cmsys;

	if( div == NULL || mtxdiv == NULL || ahdiv == NULL || vhdiv == NULL )
		return OpBadParam;

	clkmgr_get_cmsys_clk_settings(&cmsys);

	*csrc = xlate_clkmgr_tcsrc_to_opmode_csrc(cmsys.csrc);

	*div = cmsys.div;
	*mtxdiv = cmsys.mtxdiv;
	*ahdiv = cmsys.ahdiv;
	*vhdiv = cmsys.vhdiv;

	return OpOk;
}

/**
  tahoe_stat_t	tahoe_opmode_get_cmahb_divs(unsigned int *apdiv, unsigned int *spdiv, unsigned int *amdiv, unsigned int *nrdiv );
*/
tahoe_stat_t	tahoe_opmode_get_cmahb_divs(unsigned int *apdiv, unsigned int *spdiv, unsigned int *amdiv, unsigned int *nrdiv )
{
	clkmgr_cmahb_clk_settings_t	cmahb;

	if( apdiv == NULL || spdiv == NULL || amdiv == NULL || nrdiv == NULL )
		return OpBadParam;

	clkmgr_get_cmahb_clk_settings(&cmahb);
	*apdiv = cmahb.apdiv;
	*spdiv = cmahb.spdiv;
	*amdiv = cmahb.amdiv;
	*nrdiv = cmahb.nrdiv;

	return OpOk;
}
/**
  tahoe_opmode_mode_t tahoe_opmode_set_cmsys_divs(unsigned int div, unsigned int mtxdiv, unsigned int ahdiv, unsigned int vhdiv );
*/
tahoe_stat_t	tahoe_opmode_set_cmsys_divs(unsigned int div, unsigned int mtxdiv, unsigned int ahdiv, unsigned int vhdiv )
{
	clkmgr_cmsys_clk_settings_t	cmsys;

	//TODO: need to do some kind of critical section?
	clkmgr_get_cmsys_clk_settings(&cmsys);

	cmsys.div = div;
	cmsys.mtxdiv = mtxdiv;
	cmsys.ahdiv = ahdiv;
	cmsys.vhdiv = vhdiv;

	if( clkmgr_are_cmsys_clk_settings_valid(&cmsys) )
		return OpInvalidCmSysSettings;

	return bcm47xx_opmode_clock_switch(
					&cmsys,
					NULL, /* no change to CMAHB */
					NULL, /* no change to EMI mode */
					FALSE); /* no need for "forced change" -- will only do clock switch if needed */ 
}

/**
  tahoe_stat_t	tahoe_opmode_set_cmahb_divs(unsigned int apdiv, unsigned int spdiv, unsigned int amdiv, unsigned int nrdiv );
*/
tahoe_stat_t	tahoe_opmode_set_cmahb_divs(unsigned int apdiv, unsigned int spdiv, unsigned int amdiv, unsigned int nrdiv )
{
	clkmgr_cmahb_clk_settings_t	cmahb;

	//TODO: need to do some kind of critical section?
	clkmgr_get_cmahb_clk_settings(&cmahb);

	cmahb.apdiv = apdiv;
	cmahb.spdiv = spdiv;
	cmahb.amdiv = amdiv;
	cmahb.nrdiv = nrdiv;

	if( clkmgr_are_cmahb_clk_settings_valid(&cmahb) )
		return OpInvalidCmAhbSettings;

	return bcm47xx_opmode_clock_switch(
					NULL, /* no change to CMSYS */
					&cmahb,
					NULL, /* no change to EMI mode */
					FALSE); /* no need for "forced change" -- will only do clock switch if needed */ 
}


tahoe_stat_t tahoe_opmode_get_cmahb_freqs(unsigned long *ap_freq, unsigned long *sp_freq, unsigned long *am_freq, unsigned long *nr_freq)
{
	clkmgr_cmahb_freqs_t cmahb_freqs;

	if( ap_freq == NULL )
		return OpBadParam;

	if( sp_freq == NULL )
		return OpBadParam;

	if( am_freq == NULL )
		return OpBadParam;

	if( nr_freq == NULL )
		return OpBadParam;

	if( clkmgr_get_cmahb_freqs(&cmahb_freqs, NULL, NULL) != BCM_OK )
	{
        return(OpCmAhbGetFreqFailed);
	}

	*ap_freq = cmahb_freqs.ap_freq;
	*sp_freq = cmahb_freqs.sp_freq;
	*am_freq = cmahb_freqs.am_freq;
	*nr_freq = cmahb_freqs.nr_freq;

	return OpOk;
}
