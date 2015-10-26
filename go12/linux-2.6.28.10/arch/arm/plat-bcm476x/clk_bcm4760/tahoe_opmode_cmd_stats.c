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
 *   @file   tahoe_opmode_cmd_stats.c 
 * 
 *   @brief  proc interface stats information
 * 
 ****************************************************************************/

#include <linux/kernel.h>

#include "tahoe.h"
#include "tahoe_opmode_cmd_params.h"
#include "tahoe_opmode_cmd_stats.h"
#include "tahoe_opmode.h"

#define CMARM_STR   "CMARM"
#define VC_STR      "VC"
#define AHB_STR     "ARM-AHB"
#define DRAM_STR    "DRAM"

static int tahoe_opmode_get_opmode_stats(char *pbuf, int buf_size);
// static int tahoe_opmode_get_output_voltage_stats(char *pbuf, int buf_size) ;
static int tahoe_opmode_get_dram_stats(char *pbuf, int buf_size);
static int tahoe_opmode_get_peripheral_stats(char *pbuf, int buf_size);
// static int tahoe_opmode_get_synctop_stats(char *pbuf, int buf_size);
static int tahoe_opmode_get_vcsys_freq_stats(char *pbuf, int buf_size);
static int tahoe_opmode_get_pll_freq_stats(char *pbuf, int buf_size);
static int tahoe_opmode_get_peripheral_freq_stats(char *pbuf, int buf_size);

/**
    @fn int tahoe_opmode_stats(char *input_buf, int buf_size);
*/
int tahoe_opmode_stats(char *input_buf, int buf_size)
{
	int len, cnt, avail_len;
	char *pbuf;

	pbuf = input_buf;
	len = 0;
	cnt = 0;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len,	"Version: %s\n", TAHOE_OPMODE_VERSION_STR);
    pbuf  += len;
    cnt += len;

#ifdef LOG_SUPPORT
    avail_len = buf_size - cnt;        
    len = snprintf(pbuf, avail_len,	"Current log level: %s\n", tahoe_opmode_log_str(tahoe_get_log_level()));
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len,	"UART debug state : %s\n", tahoe_opmode_stat_str(tahoe_opmode_is_uart_enabled()));
    pbuf  += len;
    cnt += len;
#endif

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "Current OPMODE   :");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_opmode_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "Current DRAM mode:");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_dram_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

#if 0
    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "Ouput Voltage stats : \n");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "+-----+---------------+---------+--------+---------+ \n");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "| SNo |   Rail name   |   LDO   | On/Off | Voltage | \n");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "+-----+---------------+---------+--------+---------+ \n");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_output_voltage_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "+-----+---------------+---------+--------+---------+ \n");
    pbuf  += len;
    cnt += len;
#endif

    avail_len = buf_size - cnt;
    len = snprintf(pbuf, avail_len, "\n==========ARM/VC Peripheral Status==========\n\n");
    pbuf  += len;
	cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_peripheral_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
	len = snprintf(pbuf, avail_len, "\n==========Clock Frequency of all blocks=====\n\n");
	pbuf  += len;
	cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_pll_freq_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_vcsys_freq_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

	// Worked till here.
    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_peripheral_freq_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;

#if 0
    avail_len = buf_size - cnt;
	len = snprintf(pbuf, avail_len, "\n========== SYNCTOP INFORMATION =====\n\n");
    pbuf  += len;
    cnt += len;

    avail_len = buf_size - cnt;
    len = tahoe_opmode_get_synctop_stats(pbuf, avail_len);
    pbuf  += len;
    cnt += len;
#endif

	return (cnt);
}

static int tahoe_opmode_get_opmode_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret = OpOk ;
    char mode[MAX_MODE_STR_SZ] = {"FUN"} ;

    cnt = len = 0;
    // memset(mode, 0, MAX_MODE_STR_SZ);
    // ret = tahoe_opmode_get_mode(mode);
    if (ret == OpOk)
    {
        len = snprintf(pbuf, buf_size - cnt, "%s \n", mode);
        pbuf  += len;
        cnt += len;
    }

    return(cnt);
}

#if 0
Working code, if get_output_voltage is to be called in tahoe stats.
// Ideally following definitions need to be shared by chim/include/pmu.h and this file.
// Ask Seetharam how to do it.
#define MAX_NUM_RAILS_APPS 5
#define MAX_NUM_LDOS_SWITCHES_APPS 13
char rail_names[MAX_NUM_RAILS_APPS][64] = { "1.2 Variable", "1.2 Constant", "1.8" , "2.5", "3.3" } ;
char ldos_switches_names[MAX_NUM_LDOS_SWITCHES_APPS][64] = { 
    "ALDO1", 
    "ALDO2" ,
    "RFLDO1" ,
    "RFLDO2" ,
    "HCLDO" ,
    "USBLDO",
    "IOLDO",
    "MSLDO" ,
    "LCLDO" ,
    "LVLDO" ,
    "SIMLDO", 
    "CSRSWI",
    "IOSRSWI" } ;

static int tahoe_opmode_get_output_voltage_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret;
    int rail = 0 ;
    unsigned int value = 0 ;
    if (pbuf == NULL)
        return(0);

    cnt = len = 0;

    //Get rail voltages
    for (rail = 0; rail < MAX_NUM_RAILS_APPS; rail++)
    {
        ret = tahoe_opmode_get_output_voltage(rail, &value);

        // Bits 24,25,26,27 carry rail value. 	
        // Bits 16 - 23 carry LDO value.
        // Bit 30 tells whether the power source is enabled or disabled.
   
        if (ret == OpOk)
        {
            len = snprintf(pbuf, buf_size - cnt, "| %3d | %13s | %7s | %6d | %7d |\n", 
			    rail, 
			    rail_names[( ( value & 0x0F000000) >> 24 )] ,
                            ldos_switches_names[ ( ( value & 0x00FF0000 ) >> 16 ) ] ,
			    ( ( value & 0x40000000 ) >> 30 ) ,
			    ( ( value & 0x0000FFFF ) ) 
			    ) ;
            pbuf  += len;
            cnt += len;
        }
    }

    return(cnt);
}
#endif


#if 0
static int tahoe_opmode_get_output_voltage_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret;
    unsigned int val;

    cnt = len = 0;
    ret = tahoe_opmode_get_output_voltage(&val);
    if (ret == OpOk)
    {
        len = snprintf(pbuf, buf_size - cnt, "%d \n", val);
        pbuf  += len;
        cnt += len;
    }

    return(cnt);
}
#endif


static int tahoe_opmode_get_dram_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret;
	tahoe_opmode_dram_mode_t dram_mode;	

    cnt = len = 0;
    ret = tahoe_opmode_get_dram_mode(&dram_mode);
    if (ret == OpOk)
    {
  	    len = snprintf(pbuf, buf_size - cnt, "%s \n\n", tahoe_opmode_drammode_str(dram_mode));
        pbuf  += len;
        cnt += len;
    }

    return(cnt);
}


static int tahoe_opmode_get_peripheral_stats(char *pbuf, int buf_size)
{
	int len, cnt;
	int core_idx;
    tahoe_opmode_corectrl_t core_info;
	int enabled_cnt, core_stat[OPMODE_CORE_COUNT];

    if (pbuf == NULL)
        return(0);

    cnt = len = 0;
	enabled_cnt = 0;
    for (core_idx = 0; core_idx < OPMODE_CORE_COUNT; core_idx++)
    {
        core_info.core = core_idx;
        core_info.stat = disable;
        (void)tahoe_opmode_get_core_status(&core_info, 1);

		core_stat[core_idx] = core_info.stat;
		if (core_stat[core_idx])
			enabled_cnt++;
    }

	if (enabled_cnt)
	{
		len = snprintf(pbuf, buf_size - cnt, "ENABLED PERIPHERALS:\n\t");
		pbuf  += len;
		cnt += len;

		for (core_idx = 0; core_idx < OPMODE_CORE_COUNT; core_idx++)
		{
			if (core_stat[core_idx])
			{
				len = snprintf(pbuf, buf_size - cnt, "%s ", tahoe_opmode_core_str(core_idx));
				pbuf  += len;
				cnt += len;
			}
		}

		len = snprintf(pbuf, buf_size - cnt, "\n\n");
		pbuf  += len;
		cnt += len;
	}

	if (enabled_cnt < OPMODE_CORE_COUNT)
	{
		len = snprintf(pbuf, buf_size - cnt, "DISABLED PERIPHERALS:\n\t");
		pbuf  += len;
		cnt += len;

		for (core_idx = 0; core_idx < OPMODE_CORE_COUNT; core_idx++)
		{
			if (!core_stat[core_idx])
			{
				len = snprintf(pbuf, buf_size - cnt, "%s ", tahoe_opmode_core_str(core_idx));
				pbuf  += len;
				cnt += len;
			}
		}

		len = snprintf(pbuf, buf_size - cnt, "\n\n");
		pbuf  += len;
		cnt += len;
	}

    return(cnt);
}


static int tahoe_opmode_get_vcsys_freq_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret1, ret2, ret3, ret4;
	tahoe_opmode_csrc_t	csrc;
    unsigned long mtx_freq, vc_freq, ahb_freq;
	unsigned long ap_freq, sp_freq, am_freq, nr_freq;
	unsigned int div, mtxdiv, ahdiv, vhdiv;
	unsigned int apdiv, spdiv, amdiv, nrdiv;

    if (pbuf == NULL)
        return(0);

    cnt = len = 0;
    mtx_freq = vc_freq = ahb_freq = 0;
    ret1 = tahoe_opmode_get_vcsys_freq(&mtx_freq, &vc_freq, &ahb_freq);
    ret2 = tahoe_opmode_get_cmsys_clk_settings(&csrc, &div, &mtxdiv, &ahdiv, &vhdiv);
    if (ret1 == OpOk && ret2 == OpOk )
    {
		if(cnt + 80 >= buf_size )
			return cnt;

		len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [clock_src: %6s, div: %3u mtxdiv: %3u]\n", 
						MATRIX_STR, mtx_freq, 
						tahoe_opmode_csrc_str(csrc),
						div, mtxdiv);
		pbuf  += len;
		cnt += len;

		if(cnt + 80 >= buf_size )
			return cnt;

		len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [                   div: %3u ahdiv:  %3u]\n", 
						AHB_STR, ahb_freq,
						div, ahdiv);
		pbuf  += len;
		cnt += len;

		if(cnt + 80 >= buf_size )
			return cnt;

		len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [                   div: %3u vhdiv:  %3u]\n", 
						VC_STR, vc_freq,
						div, vhdiv);
		pbuf  += len;
		cnt += len;

		ret3 = tahoe_opmode_get_cmahb_freqs(&ap_freq, &sp_freq, &am_freq, &nr_freq);
	    ret4 = tahoe_opmode_get_cmahb_divs(&apdiv, &spdiv, &amdiv, &nrdiv);
		if( ret3 == OpOk && ret4 == OpOk )
		{
			if(cnt + 80 >= buf_size )
				return cnt;

			len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [                   div: %3u ahdiv:  %3u spdiv: %3u]\n", 
							"APB", ap_freq,
							div, ahdiv, spdiv);
			pbuf  += len;
			cnt += len;

			if(cnt + 80 >= buf_size )
				return cnt;

			len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [                   div: %3u ahdiv:  %3u amdiv: %3u]\n", 
							"AMC", am_freq,
							div, ahdiv, amdiv);
			pbuf  += len;
			cnt += len;

			if(cnt + 80 >= buf_size )
				return cnt;

			len = snprintf(pbuf, buf_size - cnt, "%15s: %10lu Hz   [                   div: %3u ahdiv:  %3u nrdiv: %3u]\n", 
							"NR", nr_freq,
							div, ahdiv, nrdiv);
			pbuf  += len;
			cnt += len;
		}
	}

    return(cnt);
}


static int tahoe_opmode_get_pll_freq_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret;
    int pll_idx;
    unsigned long pll_freq;

    cnt = len = 0;

    //Get PLL frequencies
    for (pll_idx = 0; pll_idx < PLL_COUNT; pll_idx++)
    {
        ret = tahoe_opmode_get_pll_freq(pll_idx, &pll_freq);
        if (ret == OpOk)
        {
            len = snprintf(pbuf, buf_size - cnt, "%15s: %10ld Hz\n", 
                	       tahoe_opmode_pll_str(pll_idx), pll_freq);
            pbuf += len;
	        cnt += len;
        }
    }

    return(cnt);
}


static int tahoe_opmode_get_peripheral_freq_stats(char *pbuf, int buf_size)
{
    int cnt, len;
    tahoe_stat_t ret;
    int core_idx;
    unsigned long perip_freq;

    if (pbuf == NULL)
        return(0);

    cnt = len = 0;

    //Get peripheral frequencies
    for (core_idx = 0; core_idx < OPMODE_CORE_COUNT && cnt < buf_size; core_idx++)
    {
        ret = tahoe_opmode_get_core_freq(core_idx, &perip_freq);
        if (ret == OpOk)
        {
            tahoe_opmode_csrc_t csrc = 0;
            int div;

            //get csrc and div as well
            ret = tahoe_opmode_get_core_csrc_and_div(core_idx, &csrc, &div);
            if (ret == OpOk)
            {
                len = snprintf(pbuf, buf_size - cnt, "%15s: %10ld Hz   [clock_src: %6s, divider: %3d]\n", 
                    	       tahoe_opmode_core_str(core_idx), perip_freq, tahoe_opmode_csrc_str(csrc), div);
                pbuf += len;
	            cnt += len;            
            }
        }
    }
    WARN(cnt >= buf_size, "Not enough space to write all freq stats.\n");

    return(cnt);
}

#if 0
static int tahoe_opmode_get_synctop_stats(char *pbuf, int buf_size)
{
    int synctop_idx;
    int cnt, len;
    tahoe_opmode_synctop_info_t synctop_info = {0};
    tahoe_stat_t ret;

    if (pbuf == NULL)
        return(0);

    cnt = len = 0;

    //Get synctop info
    for (synctop_idx = 0; synctop_idx < TAHOE_OPMODE_SYNCTOP_COUNT; synctop_idx++)
    {
        ret = tahoe_opmode_get_synctop(synctop_idx, &synctop_info);
        if (ret == OpOk)
        {
            if (synctop_info.mode == SYNCTOP_MODE_LATENCY)
            {
                len = snprintf(pbuf, buf_size - cnt, "%3s: mode[%9s] clkmode[%4s] prefetch_len[%d] high_mark[%d] low_mark[%d] \n", 
                    	       tahoe_opmode_synctop_str(synctop_idx),
                               tahoe_opmode_synctop_mode_str(synctop_info.mode),
                               tahoe_opmode_synctop_clkmode_str(synctop_info.clkmode),
                               synctop_info.prefetch_len,
                               synctop_info.high_watermark,
                               synctop_info.low_watermark);
            }
            else
            {
                len = snprintf(pbuf, buf_size - cnt, "%3s: mode[%9s] clkmode[%4s] prefetch_len[%d]\n", 
                    	       tahoe_opmode_synctop_str(synctop_idx),
                               tahoe_opmode_synctop_mode_str(synctop_info.mode),
                               tahoe_opmode_synctop_clkmode_str(synctop_info.clkmode),
                               synctop_info.prefetch_len);
            }

            pbuf += len;
	        cnt += len;
        }
    }

    return(cnt);
}
#endif



  
  


 
 
 
   
   
  
  
  



