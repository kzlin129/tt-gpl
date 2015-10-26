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
 *   @file   tahoe_opmode_clock_switch.c 
 * 
 *   @brief  Clock Switching Function, and other helper functions.
 * 
 ****************************************************************************/

#include "bcm_os_support.h"
#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_divide.h"

#include "clkmgr.h"
#include "chipmgr.h"
#include "emi.h"
#include "tahoe_opmode.h"
// #include "tahoe_pmu.h"
#include "tahoe_opmode_priv.h"
#include "tahoe_stat.h"
#include "armbusmtx.h"


static void set_one_synctop(armbusmtx_synctop_t which_synctop, 
								unsigned long master_freq, 
								unsigned long slave_freq)
{
	armbusmtx_synctop_info_t				synctop;

	if( master_freq > slave_freq )
		synctop.clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_Nto1;
	else if (master_freq < slave_freq )
		synctop.clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_1toN;
	else 
		synctop.clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_1to1;


	synctop.clk_change = TRUE;

	armbusmtx_set_synctop(which_synctop, &synctop);

}

/* given two frequency values hi and lo, this function
 * checks to see if it satisfies the following constraints:
 *
 *       hi/lo = 2^N
 * 
 *
 * Where N goes from 0 through "max_power_of_two"
 */
static int check_ratio(uint32_t hi_freq_exact, uint32_t lo_freq_exact, unsigned int max_power_of_two)
{
	int			i=0;
	do
	{
		if( lo_freq_exact == hi_freq_exact )
			return TRUE;

		hi_freq_exact >>= 1;
	} while( ++i <= max_power_of_two );

	return FALSE;
}

/* given two frequency values hi and lo, this function
 * checks to see if it satisfies the following constraints:
 *
 *       hi/lo = 2^N
 *          or
 *      (hi +/- 1)/ (lo +/- 1) = 2^N
 *
 * Where N goes from 0 through "max_power_of_two"
 */
static int check_ratio_all_possible(uint32_t hi_freq, uint32_t lo_freq, unsigned int max_power_of_two)
{
	if( check_ratio(hi_freq+1, lo_freq+1, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq, lo_freq+1, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq-1, lo_freq+1, max_power_of_two) )
		return TRUE;

	if( check_ratio(hi_freq-1, lo_freq, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq, lo_freq, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq-1, lo_freq, max_power_of_two) )
		return TRUE;

	if( check_ratio(hi_freq-1, lo_freq-1, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq, lo_freq-1, max_power_of_two) )
		return TRUE;
	if( check_ratio(hi_freq-1, lo_freq-1, max_power_of_two) )
		return TRUE;

	return FALSE;
}

/* The synchronizers do not allow arbitrary ratios on each side.
 * if hi and lo are the two frequencies, (hi >= lo), then they
 * have to follow the following constraint:
 *
 *      hi/lo = 2^N
 *          or
 *      (hi +/- 1)/ (lo +/- 1) = 2^N
 *
 * In case of "latency" mode, N can be "0" or "1"
 * In case of "bandwidth" mode, N can be "0..6"
 * Exception to this rule -- when synchronizers are in BYPASS mode,
 * the frequencies can be in any ratio.
*/
static bool_t check_one_synctop(armbusmtx_synctop_t synctop, uint32_t master_freq, uint32_t slave_freq)
{
	armbusmtx_synctop_info_t	synctop_info;
	uint32_t				hi_freq, lo_freq;

	if( armbusmtx_get_synctop(synctop, &synctop_info) != BCM_OK )
		return FALSE;

	// make freq2 >= freq1
	if( master_freq < slave_freq )
	{
		hi_freq = slave_freq;
		lo_freq = master_freq;
	}
	else
	{
		hi_freq = master_freq;
		lo_freq = slave_freq;
	}

	if( synctop_info.mode == ARMBUSMTX_SYNCTOP_MODE_LATENCY )
	{
		// in the case of "latency" mode, only "1:2", "1:1" or "2:1" is allowed
		if( check_ratio_all_possible(hi_freq, lo_freq, 1) )
		{
			return TRUE;
		}
		else
		{
			bcm_log_error("synctop(%d) latency mode: master(%lu) and slave(%lu) are not 1:1 nor 1:2 nor 2:1\n", 
				(int)synctop, master_freq, slave_freq);
			return FALSE;
		}
	}
	else if ( synctop_info.mode == ARMBUSMTX_SYNCTOP_MODE_BANDWIDTH )
	{
		/* Allowed ratio is 1:2^n where n can be 0-6 */
		if( check_ratio_all_possible(hi_freq, lo_freq, 6) )
		{
			return TRUE;
		}
		else
		{
			bcm_log_error("synctop(%d) bandwidth mode: master_freq(%lu) and slave__freq(%lu) are not 1:2^n\n", 
						(int)synctop, master_freq, slave_freq);
			return FALSE;
		}
	}
	else if( synctop_info.mode == ARMBUSMTX_SYNCTOP_MODE_BYPASS )
	{
		return TRUE;
	}
	else
	{
		BCM_ASSERT(!"Invalid synctop mode");
		return FALSE;
	}

	return FALSE;
}

static bool_t set_all_synctops(const clkmgr_cmsys_freqs_t * cmsys_freqs, unsigned long arm_apb_freq)
{
	/*
	 * How to identify which synchronizer is which. From email by adsuresh, 6/27/06:
	 * AI - This is the synchronizer between the ARM AHB (master) & ARM Matrix
	 * (slave). 
	 * AO - This is the synchronizer between the the ARM matrix (Master) and
	 * the ARM AHB (slave). 
	 * PI - This is the synchronizer between the Peripheral interface & ARM
	 * Matrix
	 * VI - This is the synchronizer between the VC2 AHB (master) & ARM matrix
	 * (slave)
	 * VO - This is the synchronizer between the ARM Matrix (master) & the VC2
	 * AHB (slave)
	 */


	/* The synchronizers do not allow arbitrary ratios on each side.
	 * if they are in "bandwidth" mode, the ratios has to be "2**n:1" or "1:2**n"
	 * if they are in "latency" mode, the ration has to be "2:1", "1:1" or "1:2"
	 */
	if( check_one_synctop(ARMBUSMTX_SYNC_AI, cmsys_freqs->armahb_freq, cmsys_freqs->mtx_freq) != TRUE)
		return FALSE;

	if( check_one_synctop(ARMBUSMTX_SYNC_AO, cmsys_freqs->mtx_freq, cmsys_freqs->armahb_freq) != TRUE )
		return FALSE;

	if( check_one_synctop(ARMBUSMTX_SYNC_PI, arm_apb_freq, cmsys_freqs->mtx_freq) != TRUE )
		return FALSE;

	if( check_one_synctop(ARMBUSMTX_SYNC_VI, cmsys_freqs->vc_freq, cmsys_freqs->mtx_freq) != TRUE )
		return FALSE;

	if( check_one_synctop(ARMBUSMTX_SYNC_VO, cmsys_freqs->mtx_freq, cmsys_freqs->vc_freq) != TRUE )
		return FALSE;

	if( check_one_synctop(ARMBUSMTX_SYNC_AI, cmsys_freqs->armahb_freq, cmsys_freqs->mtx_freq) != TRUE )
		return FALSE;

	/* Reason why "AO" is setup differently from all of the others, 
	 * with a low-watermark of 0x0 and prefetch-len of 0x0, explained in an 
	 * email from sliu@broadcom.com (dated Apr 25 2006):
	 *   "It is because the AO port connects to peripherals that are IO mapped.
     *   Having a 16 beats of prefetch means that you will read 16 consecutive
     *   address (within 1K boundary). This is not good for IO mapped cores."
	 */
	set_one_synctop(ARMBUSMTX_SYNC_AO, cmsys_freqs->mtx_freq, cmsys_freqs->armahb_freq);

	set_one_synctop(ARMBUSMTX_SYNC_PI, arm_apb_freq, cmsys_freqs->mtx_freq);

	set_one_synctop(ARMBUSMTX_SYNC_VI, cmsys_freqs->vc_freq, cmsys_freqs->mtx_freq);
	set_one_synctop(ARMBUSMTX_SYNC_VO, cmsys_freqs->mtx_freq, cmsys_freqs->vc_freq);

	return TRUE;
}

//This is used for testing and will be removed
// #define VOLTAGE_CHANGE 1

tahoe_stat_t bcm47xx_opmode_clock_switch(
	const clkmgr_cmsys_clk_settings_t		*cmsys_new,
	const clkmgr_cmahb_clk_settings_t		*cmahb_new,
	const tahoe_opmode_dram_mode_t				*dram_mode_new,
	bool_t											forced_switch)
{
	clkmgr_cmsys_clk_settings_t		cmsys_old;
	clkmgr_cmahb_clk_settings_t		cmahb_old;
	chipmgr_dram_clkmode_t			chipmgr_dram_mode;
	emi_dram_mode_t					emi_dram_mode;
	clkmgr_cmsys_freqs_t			cmsys_freqs_new;
	clkmgr_cmsys_freqs_t			cmsys_freqs_old;
	unsigned long							arm_apb_freq_new;
	tahoe_opmode_dram_mode_t				dram_mode_old;
	unsigned long							emi_freq_old;
	unsigned long							emi_freq_new;
    tahoe_stat_t                   ret;
#ifdef VOLTAGE_CHANGE
    unsigned int voltage = 0;
#endif


	// read the current values of cmsys and cmahb
	clkmgr_get_cmsys_clk_settings(&cmsys_old);
	clkmgr_get_cmahb_clk_settings(&cmahb_old);
	tahoe_opmode_get_dram_mode(&dram_mode_old);

	if( cmsys_new == NULL )
		cmsys_new = &cmsys_old;

	if( cmahb_new == NULL )
		cmahb_new = &cmahb_old;

	if( dram_mode_new == NULL )
		dram_mode_new = &dram_mode_old;

	if( !forced_switch && bcm_memcmp(cmsys_new, &cmsys_old, sizeof(cmsys_old))==0 &&
		bcm_memcmp(cmahb_new, &cmahb_old, sizeof(cmahb_old))==0 && 
		*dram_mode_new == dram_mode_old)
	{
		return OpOk; // "trivial" clock switch. Nothing to do.
	}

	//compute the existing mtx_frequency as well as old
	clkmgr_get_cmsys_freqs(&cmsys_freqs_old, &cmsys_old);
	clkmgr_get_cmsys_freqs(&cmsys_freqs_new, cmsys_new);
	arm_apb_freq_new = bcm_udivide32_round(cmsys_freqs_new.armahb_freq,  cmahb_new->apdiv);

	bcm47xx_opmode_get_emi_frequency(&emi_freq_old);

	if( *dram_mode_new == OPMODE_DRAM_MODE_ASYNC )
	{
		chipmgr_dram_mode = ECHIPMGR_EMI_DRAM_ASYNC;
		emi_dram_mode = EEMI_MODE_ASYNC;

		// Note: this code assumes the CMDASYN register has been setup cortrectly.
		ret = bcm47xx_opmode_get_peripheral_freq(ECLKMGR_CMDASYN, &emi_freq_new);
		if( (ret != OpOk) || (emi_freq_new == 0) )
		{
			return OpEmiAsyncModeNotSetup;
		}
	}
	else
	{
		// in "synchronous" mode, EMI frequency is same as Bus-Matrix frequency
		emi_freq_new = cmsys_freqs_new.mtx_freq;

		//Pre-calculate the EMI mode
		switch(cmsys_new->emidiv)
		{
			case ECLKMGR_EMIDIV_1_1:
			chipmgr_dram_mode = ECHIPMGR_EMI_DRAM_1by1;
			emi_dram_mode = EEMI_MODE_1by1;
			break;
			
			case ECLKMGR_EMIDIV_2_1:
			chipmgr_dram_mode = ECHIPMGR_EMI_DRAM_2by1;
			emi_dram_mode = EEMI_MODE_2by1;
			break;

			case ECLKMGR_EMIDIV_3_2:
			chipmgr_dram_mode = ECHIPMGR_EMI_DRAM_3by2;
			emi_dram_mode = EEMI_MODE_3by2;
			break;

			case ECLKMGR_EMIDIV_NONE:
			default:
			BCM_ASSERT(!"EMIDIV has invalid value");
			chipmgr_dram_mode = ECHIPMGR_EMI_DRAM_3by2;
			emi_dram_mode = EEMI_MODE_3by2;
			break;
		}

	}

	//TODO:
	// Compute the voltage_level needed for final state
	// if final reqd. voltage is higher than current voltage,
	// update current voltage to new value
    //get voltage for new frequency
#ifdef VOLTAGE_CHANGE
    ret = compute_new_output_voltage(cmsys_freqs_new.vc_freq, OUTPUT_VOLTAGE_CALC_VC_TYPE , &voltage);
    if (ret != OpOk)
    {
		bcm_log_error("%s: compute_new_output_voltage() failed, ret[%x]\n", __FUNCTION__, ret);
    }
#endif

    /**
        Note: If new frequency is more than current frequency, change voltage before changing frequency.
              else change frequency before changing voltage.
    */
#ifdef VOLTAGE_CHANGE
    if (cmsys_freqs_new.vc_freq > cmsys_freqs_old.vc_freq)
    {
        ret = tahoe_opmode_set_output_voltage(VR_1_2_RAIL, voltage);
        if (ret != OpOk)
        {
			bcm_log_error("%s: Unable to increase voltage for the requested frequency!!\n", __FUNCTION__);
        }
    }
#endif

	//TODO:
	// disable interrupts
    bcm47xx_disable_interrupts();

	//setup sync_tops 
	if( set_all_synctops(&cmsys_freqs_new, arm_apb_freq_new) != TRUE )
	{
		bcm47xx_enable_interrupts();
		return OpInvalidClockRatios;
	}
	
	//set the CMREQCS register to "hardware" mode, by setting bit4
	clkmgr_set_hw_clock_switch_mode();


	// set emi SDRAM mode
	emi_set_dram_mode(emi_dram_mode);

	//setup EMI DLL 
	emi_setup_dll_phase_load_value(emi_freq_new);

	// setup EMI SDRAM modes "3:2"/ "2:1" /"1:1" in CHIP Manager 
	chipmgr_set_emi_dram_clkmode(chipmgr_dram_mode);

	//set  Set EMI_IIS Control Register:emi_dll_rst_counter = 150
    //150 is the time (in terms of SDRAM clock) to wait DRAM DLL
    //entering lock state after DLL is reset.
    //Note1: The number 150 came from zhuang and sathish.kumar .
    //Note2: The DLL is reset only after step #10 below.
    //Note3: In theory, this step is only needed if the SDRAM clock
    //      actually changes. However, there is no harm if this step
    //       is done even otherwise.
	chipmgr_set_emi_dll_rst_counter();

	// setup the EMI sdram refresh frequency, which is derived from the
	// MTX frequency. We set it to the "safer" of the old and new values.
	emi_change_frequency(min(emi_freq_new, emi_freq_old));

	// setup the EMI/DRAM appropritaley in clockmgr (chipmgr and EMI are already
	// setup. This is the register write which would change the clock to DRAM

    //Note the order of invoking the clkmgr clock-switch function is different
    //based on the new dram mode. This is yet to be confirmed with the hardware
    //team.
	if( *dram_mode_new == OPMODE_DRAM_MODE_ASYNC )
	{
	    // we will now enter "SDRAM Dead Zone" The SDRAM
	    // switch completion. This also handles the "SDRAM Dead Zone"
	    clkmgr_clock_switch(cmsys_new, cmahb_new);

    	//We are out of "SDRAM Dead Zone"

	    //Finally, enable DRAM ASYNC mode in clock manager
		clkmgr_set_dram_clk_mode(ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK);
	}
	else
	{
		clkmgr_set_dram_clk_mode(ECLKMGR_DRAMCLK_SYNC_TO_EMICLK);

	    // we will now enter "SDRAM Dead Zone" The SDRAM
	    // switch completion. This also handles the "SDRAM Dead Zone"
	    clkmgr_clock_switch(cmsys_new, cmahb_new);

    	//We are out of "SDRAM Dead Zone"
	}


	//setup SDRAM refresh for correct frequency
	emi_change_frequency(emi_freq_new);

	//enable interrupts
    bcm47xx_enable_interrupts();

#ifdef VOLTAGE_CHANGE
    //change voltage if moving to a lower frequency
    //Note: don't go by the variable names... actually current_freq is previous and desired is current.
    if (cmsys_freqs_new.vc_freq < cmsys_freqs_old.vc_freq)
    {
        ret = tahoe_opmode_set_output_voltage(VR_1_2_RAIL, voltage);
        if (ret != OpOk)
        {
			bcm_log_error("%s: Unable to increase voltage for the requested frequency!!\n", __FUNCTION__);
        }
    }
#endif

    return(OpOk);

}

void tahoe_get_emi_ncdl(unsigned int * read_ncdl, unsigned int * write_ncdl)
{
	emi_get_ncdl(read_ncdl, write_ncdl);
	return;
}
