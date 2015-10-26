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
 *   @file   clkmgr.c
 * 
 *   @brief  Clock routines.
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_amba_io.h"
#include "bcm_divide.h"
#include "clkmgr.h"
#include "clkmgr_regs.h"
#include <asm-arm/arch-bcm4760/bcm4760_addressmap.h>

/* an invalid pointer value */
#define CLKMGR_POISON           ((clkmgr_regs_t *)0xdeadcafe)

#define CLKMGR_PASSWD           0xA5A5
#define CLKMGR_MAX_PLL_LOCK_COUNT      100000  /* why 100,000??? */
#define CLKMGR_PLL_QDIV_HIGH_VALUE           300
#define CLKMGR_PLL_QDIV_LOW_VALUE            100
#define CLKMGR_PLL_PDIV_HIGH_VALUE           20
#define CLKMGR_PLL_PDIV_LOW_VALUE            10

#define CLKMGR_PLL_VCOR_CUT_OFF_HZ            250000000UL

//TODO: These should be defined at a top level
#define UINT32_MAX	0xFFFFFFFF

/** Function used within this file */
static uint32_t wait_for_pll_lock(clkmgr_pll_t pll);

static uint32_t calc_pll_dividers(uint32_t desired_freq, 
                            uint32_t *best_qdiv, uint32_t *best_pdiv, uint32_t *best_freq);
static uint32_t set_timer_clk(clkmgr_blk_t blk, clkmgr_tcsrc_t csrc, uint32_t div);
static void enable_arm_block_clk(clkmgr_arm_block_t arm_blk);
static void disable_arm_block_clk(clkmgr_arm_block_t arm_blk);

/* Clock manager block base address */
static clkmgr_regs_t *pclkmgr_base_addr = 0;
typedef struct _clkmgr_blk_id_t
{
    clkmgr_blk_t  blk;
    uint32_t       reg_offset;
    unsigned long  addr;
}clkmgr_blk_id_t;
 

//order here has to match the enum _bcm28xx_blk_t
static clkmgr_blk_id_t clkmgr_blks[]=
{
	{ ECLKMGR_CMUART0,  0x0024, 0 },
    { ECLKMGR_CMUART1,  0x0028, 0 },
    { ECLKMGR_CMUART2,  0x002c, 0 },
    { ECLKMGR_CMUART,   0x0030, 0 },
//Hole here forcing us to use this structure to compute address. Using
//switch case in the functions that touch these registers bloat the code
//and can't be used in cldb environment.
    { ECLKMGR_CMCRYPT,  0x0038, 0 },
    { ECLKMGR_CMGEN,    0x003C, 0 },
    { ECLKMGR_CMARMI2C, 0x0040, 0 },
    { ECLKMGR_CMVCI2C,  0x0044, 0 },
    { ECLKMGR_CMVFIR,   0x0048, 0 },
    { ECLKMGR_CMSPI0,   0x004C, 0 },
    { ECLKMGR_CMSPI1,   0x0050, 0 },
    { ECLKMGR_CMARMI2S, 0x0054, 0 },
    { ECLKMGR_CMCVDI2S, 0x0058, 0 },
    { ECLKMGR_CMVCI2S,  0x005C, 0 },
    { ECLKMGR_CMSDIO0,  0x0060, 0 },
    { ECLKMGR_CMSDIO1,  0x0064, 0 },
    { ECLKMGR_CMLCD,    0x0068, 0 },
    { ECLKMGR_CMCAM,    0x006C, 0 },
    { ECLKMGR_CMSTOR,   0x0070, 0 },

	//Another hole in the register definitions in the clk manager
    { ECLKMGR_CMDASYN,  0x0094, 0 },

	//Another hole in the register definitions in the clk manager
    //keep all timers together
    { ECLKMGR_CMTIM0,   0x0018, 0 },
    { ECLKMGR_CMTIM1,   0x001C, 0 },
    { ECLKMGR_CMPWM,    0x0020, 0 },
    { ECLKMGR_CMVCTIM,  0x0078, 0 }
};                                  
                                  
//--------------------------------------------------
/**
    Clock manager public APIs
*/
//--------------------------------------------------
/**
    @fn void clkmgr_api_init(bcm_dev_virtaddr_t virt_addr)
*/
void clkmgr_api_init(void)
{
    uint32_t i, perip_cnt;

	BCM_ASSERT(pclkmgr_base_addr==NULL); // prevent against double-init
#if 0 // DCN incompleted, may not applicable to BCM4760
    pclkmgr_base_addr = (clkmgr_regs_t *)bcm_xlate_to_virt_addr(BCM28XX_CLKMGR_ARM_ADDRBASE0);
#else
    pclkmgr_base_addr = CLKMGR_POISON;
#endif
	//fix the peripheral addresses using the base 
    perip_cnt = sizeof(clkmgr_blks)/sizeof(clkmgr_blk_id_t);

	for (i=0; i < perip_cnt; i++)
    {
        clkmgr_blks[i].addr = (unsigned long)(((uint8_t *)pclkmgr_base_addr + clkmgr_blks[i].reg_offset));       
    }
}

/**
    @fn uint32_t clkmgr_unlock_regs(void)
*/
uint32_t clkmgr_unlock_regs(void)
{
    clkmgr_cmlock_u_t lock;
    lock.w_data = 0;
    lock.bf.passwd = CLKMGR_PASSWD;
    lock.bf.reglock = BCM_BIT_CLR;
    amba_reg_write32(pclkmgr_base_addr, cmlock, lock.w_data);
    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_lock_regs(void);
*/
uint32_t clkmgr_lock_regs(void)
{
	clkmgr_cmlock_u_t lock;

	lock.w_data = 0;
    lock.bf.passwd = CLKMGR_PASSWD;
    lock.bf.reglock = BCM_BIT_SET;
    amba_reg_write32(pclkmgr_base_addr, cmlock, lock.w_data);
    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_set_hw_clock_switch_mode(void)
*/
uint32_t clkmgr_set_hw_clock_switch_mode(void)
{
    clkmgr_cmreqcs_u_t cmreqcs;

	cmreqcs.w_data = amba_reg_read32(pclkmgr_base_addr, cmreqcs); 
    cmreqcs.bf.reqmode = 1; //hw controls the clock change request to the EMI
    amba_reg_write32(pclkmgr_base_addr, cmreqcs, cmreqcs.w_data);    
    return(BCM_OK);
}
/**
    void clkmgr_wait_for_clock_switch_completion(void);
*/
void clkmgr_wait_for_clock_switch_completion(void)
{
    clkmgr_cmreqcs_u_t cmreqcs;

	do
	{
		cmreqcs.w_data = amba_reg_read32(pclkmgr_base_addr, cmreqcs); 
	}    while( cmreqcs.bf.reqstat != 0); //'0' indicates clock switch is done
}
/**
    @fn uint32_t clkmgr_set_pll_frequency(clkmgr_pll_t pll, unsigned long desired_freq,
                                           int tolerance_percent,
                                           unsigned long *actual_freq);
*/
uint32_t clkmgr_set_pll_frequency(clkmgr_pll_t pll, 
								   uint32_t desired_freq,
                                   int tolerance_percent,
                                   uint32_t *actual_freq)
{
    uint32_t ret_val;
    uint32_t deviation_percent;

    clkmgr_pll_settings_t pll_info;
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT) 
	BCM_ASSERT(actual_freq != NULL);

    pll_info.pdiv = pll_info.qdiv;
    ret_val = calc_pll_dividers(desired_freq, &pll_info.qdiv, &pll_info.pdiv, 
										actual_freq);
    if (ret_val == BCM_ERROR)
    {
        bcm_log_error("%s: No suitable qdiv and pdiv for frequency=%u Hz\n", 
			__FUNCTION__, desired_freq);
        return(BCM_ERROR);
    }
    //Are we exceeding the limits?
	if( tolerance_percent != 0 )
	{
		deviation_percent = bcm_get_deviation_percent(desired_freq, *actual_freq);
    if (deviation_percent > tolerance_percent)
	{
        bcm_log_debug("%s: Best available frequency[%ld] deviation[%d]\n", __FUNCTION__, 
                        *actual_freq, deviation_percent);

		return(BCM_ERROR);
    }
	}
    //Now program the register for the desired frequency
    ret_val = clkmgr_enable_pll(pll, &pll_info);
    if (ret_val == BCM_ERROR)
    {
        bcm_log_debug("%s: clkmgr_enable_pll() Failed\n", __FUNCTION__);
        return(BCM_ERROR);
    }
    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_enable_pll(clkmgr_pll_t pll, clkmgr_pll_settings_t *ppll_info)
*/
uint32_t clkmgr_enable_pll(clkmgr_pll_t pll, clkmgr_pll_settings_t *ppll_info) 
{
    clkmgr_cmpll_u_t pll_val;
    BCM_ASSERT((pll < CLKMGR_PLL_COUNT) && (ppll_info != NULL));
    pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
    //Is this PLL already enabled?
    if (pll_val.bf.enab == 0)    
    {
        /**
            The following sequence has to be followed on enabling a PLL
            that has been disabled earlier. This is because of a H/W issue in 
			the 2820 A0
	        workaround listed in email from Daniel Sun, dated May 9 2006 
            PLL power up: 
            1. set ENAB=1 VCO_R=0 PDIV=1 QDIV=6
            2. wait for pll LOCK (vco output 144MHZ)
            3. keep ENAB=1, change VCO_R, PDIV and QDIV 
               values for the target frequency.
            4. wait for pll LOCK.
            5. pll output is ready.
         */
        pll_val.w_data = 0;
        pll_val.bf.enab = BCM_BIT_SET;
        pll_val.bf.vcor = 0;
        pll_val.bf.qdiv = 6;
        pll_val.bf.pdiv = 1;
        amba_reg_write32(pclkmgr_base_addr, cmpll[pll], pll_val.w_data);
		if (wait_for_pll_lock(pll) == BCM_ERROR)
    	{
			/*bcm_log_error("%s: pll lock failed, addr=0x%0x, val=0x%0x\n", 
						__FUNCTION__,
						(unsigned int)bcm_xlate_to_phys_addr(&pclkmgr_base_addr->cmpll[pll], 
						(unsigned int)pll_val.w_data);*/
								
            return(BCM_ERROR);
    	}
    }
    //Now set the values to the desired frequency and wait for the PLL to get locked
	if( clkmgr_set_pll_settings(pll, ppll_info) != BCM_OK )
	{
		return BCM_ERROR;
	}

	if (wait_for_pll_lock(pll) != BCM_OK)
    {
		/*bcm_log_error("%s: pll lock failed, addr=0x%0x, val=0x%0x\n", 
					__FUNCTION__,
					(unsigned int)bcm_xlate_to_phys_addr(&pclkmgr_base_addr->cmpll[pll], 
					(unsigned int)pll_val.w_data);*/
        return(BCM_ERROR);
    }

	return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_disable_pll(clkmgr_pll_t pll);
*/
uint32_t clkmgr_disable_pll(clkmgr_pll_t pll)
{
    clkmgr_cmpll_u_t pll_val;
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT);
	/*
	 * If disabling, always set it to default state PDIV=1, QDIV=1, ENAB=0
	 */
    pll_val.w_data = 0;
    pll_val.bf.qdiv = 1;
    pll_val.bf.pdiv = 1;

    amba_reg_write32(pclkmgr_base_addr, cmpll[pll], pll_val.w_data);

    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_is_pll_enabled(clkmgr_pll_t pll);
*/
uint32_t clkmgr_is_pll_enabled(clkmgr_pll_t pll)
{
    clkmgr_cmpll_u_t pll_val;

    if (pclkmgr_base_addr == CLKMGR_POISON) {
      return 0;
    }

    BCM_ASSERT(pll < CLKMGR_PLL_COUNT);
    pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
    return(pll_val.bf.enab);
}
/**
    @fn uin32_t clkmgr_get_pll_settings(clkmgr_pll_t pll, , clkmgr_pll_settings_t *ppll_info);
*/
uint32_t clkmgr_get_pll_settings(clkmgr_pll_t pll, clkmgr_pll_settings_t *ppll_info)
{
    clkmgr_cmpll_u_t pll_val;
   
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT); 
	BCM_ASSERT(ppll_info != NULL);
    pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
	ppll_info->qdiv = pll_val.bf.qdiv==0 ? CLKMGR_PLL_QDIV_MAX : pll_val.bf.qdiv;
	ppll_info->pdiv = pll_val.bf.pdiv==0 ? CLKMGR_PLL_PDIV_MAX : pll_val.bf.pdiv;                
    return(BCM_OK);
}

/**
    @fn uin32_t clkmgr_set_pll_settings(clkmgr_pll_t pll, , const clkmgr_pll_settings_t *ppll_info);
*/
uint32_t clkmgr_set_pll_settings(clkmgr_pll_t pll, const clkmgr_pll_settings_t *ppll_info)
{
    clkmgr_cmpll_u_t pll_val;
    uint32_t pll_freq;
   
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT); 
	BCM_ASSERT(ppll_info != NULL);

	BCM_ASSERT(ppll_info->qdiv <= CLKMGR_PLL_QDIV_MAX);
	BCM_ASSERT(ppll_info->pdiv <= CLKMGR_PLL_PDIV_MAX);

	pll_freq = bcm_udivide64_round(CLKMGR_XTAL_24MHZ_FREQUENCY * (uint64_t)ppll_info->qdiv, 
						ppll_info->pdiv);
	BCM_ASSERT(pll_freq >= CLKMGR_PLL_FREQ_MIN);
    pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
	pll_val.bf.vcor = (pll_freq > CLKMGR_PLL_VCO_CUTOFF_FREQ) ? 1 : 0;
	pll_val.bf.qdiv = (ppll_info->qdiv == CLKMGR_PLL_QDIV_MAX) ? 0 : ppll_info->qdiv;
	pll_val.bf.pdiv = (ppll_info->pdiv == CLKMGR_PLL_PDIV_MAX) ? 0 : ppll_info->pdiv;

    amba_reg_write32(pclkmgr_base_addr, cmpll[pll], pll_val.w_data);
    return(BCM_OK);
}

/**
    @fn uint32_t clkmgr_get_pll_frequency(clkmgr_pll_t pll);
*/
uint32_t clkmgr_get_pll_frequency(clkmgr_pll_t pll)
{
    clkmgr_pll_settings_t pll_info;
    
	BCM_ASSERT(pll < CLKMGR_PLL_COUNT);

    if (!clkmgr_is_pll_enabled(pll))
        return(0);

	clkmgr_get_pll_settings(pll, &pll_info);
    return bcm_udivide64_round(
			CLKMGR_XTAL_24MHZ_FREQUENCY * (uint64_t)pll_info.qdiv, 
			pll_info.pdiv);
}

/**
    @fn uint32_t clkmgr_compute_pll_frequency(const clkmgr_pll_settings_t *ppll_settings);
*/
uint32_t clkmgr_compute_pll_frequency(const clkmgr_pll_settings_t *ppll_settings)
{       
	BCM_ASSERT(ppll_settings != NULL);

    return bcm_udivide64_round(
			CLKMGR_XTAL_24MHZ_FREQUENCY * (uint64_t)ppll_settings->qdiv, 
			ppll_settings->pdiv);
}


void set_cmsys_clk_settings_helper(const clkmgr_cmsys_clk_settings_t *pcmsys_clk, clkmgr_cmsys_u_t *cmsys_val)
{
    BCM_ASSERT(pcmsys_clk != NULL);
    BCM_ASSERT(cmsys_val != NULL);

	// load existing values
    cmsys_val->w_data = amba_reg_read32(pclkmgr_base_addr, cmsys);   

	BCM_ASSERT(clkmgr_are_cmsys_clk_settings_valid(pcmsys_clk) == BCM_OK );

	// and then load the new values from the cmsys structure...
	cmsys_val->bf.ahdiv  = (pcmsys_clk->ahdiv == 64) ? 0 : pcmsys_clk->ahdiv;
	cmsys_val->bf.csrc   = pcmsys_clk->csrc;
	cmsys_val->bf.div    = (pcmsys_clk->div == 8) ? 0 : pcmsys_clk->div;
    cmsys_val->bf.emidiv = pcmsys_clk->emidiv;
	cmsys_val->bf.mtxdiv = (pcmsys_clk->mtxdiv == 4) ? 0  : pcmsys_clk->mtxdiv;
	cmsys_val->bf.vhdiv  = (pcmsys_clk->vhdiv==64) ? 0 : pcmsys_clk->vhdiv;
}

/**
    @fn uint32_t clkmgr_set_cmsys_clk_settings(clkmgr_cmsys_clk_settings_t *pcmsys_clk)
*/
uint32_t clkmgr_set_cmsys_clk_settings(const clkmgr_cmsys_clk_settings_t *pcmsys_clk)
{
    clkmgr_cmsys_u_t cmsys_val;

	if ( clkmgr_are_cmsys_clk_settings_valid(pcmsys_clk) != BCM_OK )
		return BCM_ERROR;

	set_cmsys_clk_settings_helper(pcmsys_clk, &cmsys_val);
    amba_reg_write32(pclkmgr_base_addr, cmsys, cmsys_val.w_data);   

    return(BCM_OK);
}


// in "sychronous" mode, SDRAM_DIV=MTXDIV * REAL_EMIDIV must be an integer
static uint32_t is_dram_div_valid(uint32_t mtxdiv, uint32_t emidiv)
{
	// SDRAM_DIV = MTXDIV * REAL_EMIDIV
	// (where REAL_EMIDIV is 1, 1.5 or 2)
	// SDRAM_DIV must be an integer

	if( ECLKMGR_EMIDIV_3_2 == emidiv )
	{
		// in "3:2" mode, mtxdiv must be an even number
		return ((mtxdiv % 2)==0) ? BCM_OK : BCM_ERROR;
	}

	// in 1:1 or "2:1"  modes mtxdiv can be anything
	return BCM_OK;
}


/**
    @fn uint32 clkmgr_are_cmsys_clk_settings_valid(const clkmgr_cmsys_clk_settings_t *pcmsys_clk)
*/
uint32_t clkmgr_are_cmsys_clk_settings_valid(const clkmgr_cmsys_clk_settings_t *pcmsys_clk)
{
	BCM_ASSERT(pcmsys_clk != NULL);

	if( !(pcmsys_clk->csrc == ECLKMGR_TCSRC_XTAL_24MHZ ||
		ECLKMGR_TCSRC_XTAL_27MHZ ||
		ECLKMGR_TCSRC_XTAL_32KHZ ||
		ECLKMGR_TCSRC_PLLS ||
		ECLKMGR_TCSRC_PLL1 ||
		ECLKMGR_TCSRC_PLL2 ||
		ECLKMGR_TCSRC_PLL3 ) ) 
	{
		bcm_log_error("%s(): invalid CMSYS:csrc %d\n", __FUNCTION__, (int)pcmsys_clk->csrc);
	}

	if( pcmsys_clk->div > 8 )
	{
		
		bcm_log_error("%s(): invalid CMSYS:div %d\n", __FUNCTION__, (int)pcmsys_clk->div);
		return BCM_ERROR;
	}

	if( pcmsys_clk->mtxdiv > 4 )
	{
		bcm_log_error("%s(): invalid CMSYS:mtxdiv %d\n", __FUNCTION__, (int)pcmsys_clk->mtxdiv);
		return BCM_ERROR;
	}

	if( !(pcmsys_clk->emidiv==ECLKMGR_EMIDIV_1_1 ||
					pcmsys_clk->emidiv==ECLKMGR_EMIDIV_3_2 ||
						pcmsys_clk->emidiv==ECLKMGR_EMIDIV_2_1) )
	{
		bcm_log_error("%s(): invalid CMSYS:emidiv %d\n", __FUNCTION__, (int)pcmsys_clk->emidiv);
		return BCM_ERROR;
	}

	if( pcmsys_clk->ahdiv > 64 )
	{
		bcm_log_error("%s(): invalid CMSYS:ahdiv %d\n", __FUNCTION__, (int)pcmsys_clk->ahdiv);
		return BCM_ERROR;
	}

	if( pcmsys_clk->vhdiv > 64 )
	{
		bcm_log_error("%s(): invalid CMSYS:vhdiv %d\n", __FUNCTION__, (int)pcmsys_clk->vhdiv);
		return BCM_ERROR;
	}

	if( is_dram_div_valid(pcmsys_clk->mtxdiv, pcmsys_clk->emidiv) == BCM_ERROR )
	{
		bcm_log_error("%s(): invalid combination of CMSYS:mtxdiv %d and CMSYS:emidiv %d\n", 
					__FUNCTION__, (int)pcmsys_clk->mtxdiv, (int)pcmsys_clk->emidiv);
		return BCM_ERROR;
	}

	return BCM_OK;
}

/**
    @fn uint32_t clkmgr_get_cmsys_clk_settings(clkmgr_cmsys_clk_settings_t *pcmsys_clk)
*/
uint32_t clkmgr_get_cmsys_clk_settings(clkmgr_cmsys_clk_settings_t *pcmsys_clk)
{
    clkmgr_cmsys_u_t cmsys_val;
    BCM_ASSERT(pcmsys_clk != NULL);

    if (pclkmgr_base_addr == CLKMGR_POISON) {
      memset( pcmsys_clk, 0, sizeof(*pcmsys_clk) );
      return 0;
    }

	cmsys_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmsys);
	pcmsys_clk->ahdiv  =  cmsys_val.bf.ahdiv==0 ? 64 : cmsys_val.bf.ahdiv;
	pcmsys_clk->csrc   =  cmsys_val.bf.csrc==0 ? ECLKMGR_TCSRC_XTAL_24MHZ : cmsys_val.bf.csrc;
	pcmsys_clk->div    =  cmsys_val.bf.div ==0 ? 8 : cmsys_val.bf.div;
    pcmsys_clk->emidiv =  cmsys_val.bf.emidiv;
	pcmsys_clk->mtxdiv =  cmsys_val.bf.mtxdiv==0 ? 4 : cmsys_val.bf.mtxdiv;
	pcmsys_clk->vhdiv  =  cmsys_val.bf.vhdiv==0 ? 64 : cmsys_val.bf.vhdiv;
	return(BCM_OK);
}
/**
    @fn clkmgr_tcsrc_t clkmgr_get_cmsys_csrc(void);
*/
clkmgr_tcsrc_t clkmgr_get_cmsys_csrc(void)
{
    clkmgr_cmsys_u_t cmsys_val;
    cmsys_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmsys);
    return((clkmgr_tcsrc_t)cmsys_val.bf.csrc);
}
/**
    @fn uint32_t clkmgr_get_cmsys_freqs(const clkmgr_cmsys_clk_settings_t *cmsys, clkmgr_cmsys_freqs_t *pvcsys_freqs)
*/
uint32_t clkmgr_get_cmsys_freqs(clkmgr_cmsys_freqs_t *pvcsys_freqs, const clkmgr_cmsys_clk_settings_t *pcmsys)
{
	clkmgr_cmsys_clk_settings_t cmsys;
    uint32_t	div;
    uint32_t	source_freq, inter_freq;
    BCM_ASSERT(pvcsys_freqs != NULL);
    if (pclkmgr_base_addr == CLKMGR_POISON) {
      memset( pvcsys_freqs, 0, sizeof( *pvcsys_freqs) );
      return 0;
    }

    if( pcmsys == NULL )
    {
      clkmgr_get_cmsys_clk_settings(&cmsys);
      pcmsys = &cmsys;
    }
    //get the divider
    div = pcmsys->div;
    //Get the source
	switch(pcmsys->csrc)
	{
		case ECLKMGR_TCSRC_XTAL_24MHZ:
		source_freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
		break;
		case ECLKMGR_TCSRC_XTAL_27MHZ:
		source_freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
		break;
		case ECLKMGR_TCSRC_XTAL_32KHZ:
		source_freq = CLKMGR_XTAL_27MHZ_FREQUENCY;
		break;

		case ECLKMGR_TCSRC_PLLS:
		case ECLKMGR_TCSRC_PLL1:
		case ECLKMGR_TCSRC_PLL2:
		case ECLKMGR_TCSRC_PLL3:
            //enums don't match, hence this fix
        source_freq = clkmgr_get_pll_frequency((clkmgr_pll_t)(pcmsys->csrc - ECLKMGR_TCSRC_PLLS));
		break;

		case ECLKMGR_TCSRC_NONE:
		default:
		BCM_ASSERT(!"Invalid frequency source in CMSYS");
		return BCM_ERROR;
	}
    inter_freq = bcm_udivide32_round(source_freq, pcmsys->div);
    pvcsys_freqs->mtx_freq = bcm_udivide32_round(inter_freq, pcmsys->mtxdiv);
    pvcsys_freqs->vc_freq = bcm_udivide32_round(inter_freq, pcmsys->vhdiv);
    pvcsys_freqs->armahb_freq = bcm_udivide32_round(inter_freq, pcmsys->ahdiv);
    return(BCM_OK);             
}

uint32_t clkmgr_compute_cmsys_freqs(clkmgr_cmsys_freqs_t *pvcsys_freqs, 
                                    const clkmgr_cmsys_clk_settings_t *pcmsys, 
                                    const clkmgr_pll_settings_t *ppll_settings)
{
	clkmgr_cmsys_clk_settings_t cmsys;
    uint32_t	div;
    uint32_t	source_freq, inter_freq;
    
    BCM_ASSERT(pvcsys_freqs != NULL);

	if( pcmsys == NULL )
	{
		clkmgr_get_cmsys_clk_settings(&cmsys);
		pcmsys = &cmsys;
	}
    //get the divider
    div = pcmsys->div;
    //Get the source
	switch(pcmsys->csrc)
	{
		case ECLKMGR_TCSRC_XTAL_24MHZ:
		source_freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
		break;
		case ECLKMGR_TCSRC_XTAL_27MHZ:
		source_freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
		break;
		case ECLKMGR_TCSRC_XTAL_32KHZ:
		source_freq = CLKMGR_XTAL_27MHZ_FREQUENCY;
		break;

		case ECLKMGR_TCSRC_PLLS:
		case ECLKMGR_TCSRC_PLL1:
		case ECLKMGR_TCSRC_PLL2:
		case ECLKMGR_TCSRC_PLL3:
        //enums don't match, hence this fix
        if (ppll_settings != NULL)
        {
            source_freq = clkmgr_compute_pll_frequency(ppll_settings);
        }
        else
        {
            source_freq = clkmgr_get_pll_frequency((clkmgr_pll_t)(pcmsys->csrc - ECLKMGR_TCSRC_PLLS));
        }
		break;

		case ECLKMGR_TCSRC_NONE:
		default:
		BCM_ASSERT(!"Invalid frequency source in CMSYS");
		return BCM_ERROR;
	}
    inter_freq = bcm_udivide32_round(source_freq, pcmsys->div);
    pvcsys_freqs->mtx_freq = bcm_udivide32_round(inter_freq, pcmsys->mtxdiv);
    pvcsys_freqs->vc_freq = bcm_udivide32_round(inter_freq, pcmsys->vhdiv);
    pvcsys_freqs->armahb_freq = bcm_udivide32_round(inter_freq, pcmsys->ahdiv);
    return(BCM_OK);             
}


/**
    @fn uint32_t clkmgr_set_arm_clk(clkmgr_csrc_t csrc, uint32_t div)
*/
uint32_t clkmgr_set_arm_clk(clkmgr_csrc_t csrc, uint32_t div)
{
    clkmgr_cmarm_u_t arm_val;
	BCM_ASSERT((csrc == ECLKMGR_TCSRC_XTAL_24MHZ) ||
		(csrc >= ECLKMGR_TCSRC_PLLS && csrc <= ECLKMGR_TCSRC_PLL3));

	BCM_ASSERT(div <= CLKMGR_ARMCLK_DIV_MAX); 

    arm_val.w_data = 0;
	arm_val.bf.sync = 0; /* always run asynchronously */
    arm_val.bf.csrc = csrc;
	arm_val.bf.div = (div == CLKMGR_ARMCLK_DIV_MAX) ? 0 : div;
    amba_reg_write32(pclkmgr_base_addr, cmarm, arm_val.w_data);
    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_get_arm_clk(clkmgr_tcsrc_t *csrc, uint32_t *div);
*/
uint32_t clkmgr_get_arm_clk(clkmgr_tcsrc_t *csrc, uint32_t *div)
{
    clkmgr_cmarm_u_t cmarm_val;

    BCM_ASSERT(csrc != NULL);
    BCM_ASSERT(div != NULL);

    if (pclkmgr_base_addr == CLKMGR_POISON) {
      *csrc = ECLKMGR_TCSRC_NONE;
      *div = 0;
      return BCM_ERROR;
    }
    cmarm_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmarm);

    *csrc = (clkmgr_tcsrc_t)cmarm_val.bf.csrc;
    *div = (cmarm_val.bf.div == 0) ? CLKMGR_ARMCLK_DIV_MAX : cmarm_val.bf.div;

    return(BCM_OK);
}

/**
    @fn uint64_t clkmgr_get_arm_frequency(void);
*/
uint32_t clkmgr_get_arm_frequency(void)
{
    clkmgr_cmarm_u_t cmarm_val;
    uint32_t div;
    uint32_t freq;
    cmarm_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmarm);
    div = (cmarm_val.bf.div == 0) ? CLKMGR_ARMCLK_DIV_MAX : cmarm_val.bf.div;
    //@todo - Optimize - move this code into a separate function for reuse

	if (cmarm_val.bf.csrc == ECLKMGR_CSRC_XTAL_24MHZ)
    {
    	freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
    }
    else if ((cmarm_val.bf.csrc >= ECLKMGR_CSRC_PLLS) &&     //Is a PLL the source?
             (cmarm_val.bf.csrc <= CLKMGR_CSRC_MAX))
    {
        //enum don't match, hence this fix
        freq = clkmgr_get_pll_frequency((clkmgr_pll_t)(cmarm_val.bf.csrc - ECLKMGR_CSRC_PLLS));
        bcm_log_debug("%s: csrc[%d] div[%d] pll_freq[%lu]\n", 
				__FUNCTION__, 
				cmarm_val.bf.csrc, div, (unsigned long) freq); 
    }
    else
	{
		BCM_ASSERT(!"CMARM has invalid clock source!");
		return 0;
	}
    freq = bcm_udivide32_round(freq, div);
    bcm_log_debug("%s: arm11_freq[%lu] \n", __FUNCTION__, (unsigned long) freq); 
    return freq;
}
static void set_cmahb_clk_settings_helper(const clkmgr_cmahb_clk_settings_t *pahb_clk, clkmgr_cmahb_u_t *cmahb_val)
{
    BCM_ASSERT(pahb_clk != NULL);
    BCM_ASSERT(cmahb_val != NULL);

	// load existing values
    cmahb_val->w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);   

	BCM_ASSERT(clkmgr_are_cmahb_clk_settings_valid(pahb_clk) == BCM_OK);
	// and then load the values from the passed in pahb_clk...

	cmahb_val->bf.amdiv = pahb_clk->amdiv==16 ? 0 : pahb_clk->amdiv;                      
	cmahb_val->bf.apdiv = pahb_clk->apdiv==4 ? 0 : pahb_clk->apdiv;
	cmahb_val->bf.nrdiv = pahb_clk->nrdiv==4 ? 0 : pahb_clk->nrdiv;                      
	cmahb_val->bf.spdiv = pahb_clk->spdiv==4 ? 0 : pahb_clk->spdiv;                       
}

/**
    @fn uint32_t clkmgr_are_cmahb_clk_settings_valid(const clkmgr_cmsys_clk_settings_t *pcmahb_clk);
*/
uint32_t clkmgr_are_cmahb_clk_settings_valid(const clkmgr_cmahb_clk_settings_t *pcmahb_clk)
{
	if( !(pcmahb_clk->apdiv == 1 || pcmahb_clk->apdiv == 2 || pcmahb_clk->apdiv == 4 ) )
	{
		bcm_log_error("%s(): Invalid CMAHB:apdiv %d", __FUNCTION__, (int)pcmahb_clk->apdiv  );
		return BCM_ERROR;
	}

	if( !(pcmahb_clk->spdiv == 1 || pcmahb_clk->spdiv == 2 || pcmahb_clk->spdiv == 4 ) )
	{
		bcm_log_error("%s(): Invalid CMAHB:spdiv %d", __FUNCTION__, (int)pcmahb_clk->spdiv  );
		return BCM_ERROR;
	}

	if( pcmahb_clk->amdiv > 16 )
	{
		bcm_log_error("%s(): Invalid CMAHB:amdiv %d", __FUNCTION__, (int)pcmahb_clk->amdiv  );
		return BCM_ERROR;
	}

	if( pcmahb_clk->nrdiv > 4 )
	{
		bcm_log_error("%s(): Invalid CMAHB:nrdiv %d", __FUNCTION__, (int)pcmahb_clk->nrdiv  );
		return BCM_ERROR;
	}

	return BCM_OK;
}


uint32_t clkmgr_set_cmahb_clk_settings(const clkmgr_cmahb_clk_settings_t *pahb_clk)
{
    clkmgr_cmahb_u_t cmahb_val;

	if ( clkmgr_are_cmahb_clk_settings_valid(pahb_clk) != BCM_OK )
		return BCM_ERROR;

	set_cmahb_clk_settings_helper(pahb_clk, &cmahb_val);
    amba_reg_write32(pclkmgr_base_addr, cmahb, cmahb_val.w_data);   
    return(BCM_OK);

}

/**
    @fn uint32_t clkmgr_get_cmahb_clk_settings(clkmgr_cmahb_clk_settings_t *pahb_clk);
*/
uint32_t clkmgr_get_cmahb_clk_settings(clkmgr_cmahb_clk_settings_t *pahb_clk)
{
    clkmgr_cmahb_u_t cmahb_val;
    BCM_ASSERT(pahb_clk != NULL);
    if (pclkmgr_base_addr == CLKMGR_POISON) {
      memset(pahb_clk, 0, sizeof( *pahb_clk));
      return 0;
    }
    cmahb_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);
	pahb_clk->amdiv = cmahb_val.bf.amdiv==0 ? 16 : cmahb_val.bf.amdiv;
	pahb_clk->apdiv = cmahb_val.bf.apdiv==0 ? 4 : cmahb_val.bf.apdiv;
	pahb_clk->nrdiv = cmahb_val.bf.nrdiv==0 ? 4 : cmahb_val.bf.nrdiv;
	pahb_clk->spdiv = cmahb_val.bf.spdiv==0 ? 4 : cmahb_val.bf.spdiv;
    return(BCM_OK);
}

/**
    @fn uint32_t uint32_t clkmgr_get_cmahb_freqs(clkmgr_cmahb_freqs_t *pcmahb_freqs, 
								const clkmgr_cmsys_clk_settings_t *pcmsys,
								const clkmgr_cmahb_clk_settings_t *pcmahb)
*/
uint32_t clkmgr_get_cmahb_freqs(clkmgr_cmahb_freqs_t *pcmahb_freqs, 
								const clkmgr_cmsys_clk_settings_t *pcmsys,
								const clkmgr_cmahb_clk_settings_t *pcmahb)
{
	clkmgr_cmahb_clk_settings_t cmahb;
	clkmgr_cmsys_freqs_t		cmsys_freq;

    BCM_ASSERT(pcmahb_freqs != NULL);

	if( clkmgr_get_cmsys_freqs(&cmsys_freq, pcmsys) != BCM_OK )
		return BCM_ERROR;

	if( pcmahb == NULL )
	{
		// read the CMAHB and compute frequencies
		if( clkmgr_get_cmahb_clk_settings(&cmahb) != BCM_OK )
			return BCM_ERROR;

		pcmahb = &cmahb;
	}

	pcmahb_freqs->ap_freq = bcm_udivide32(cmsys_freq.armahb_freq, pcmahb->apdiv);
	pcmahb_freqs->sp_freq = bcm_udivide32(cmsys_freq.armahb_freq, pcmahb->spdiv);
	pcmahb_freqs->am_freq = bcm_udivide32(cmsys_freq.armahb_freq, pcmahb->amdiv);
	pcmahb_freqs->nr_freq = bcm_udivide32(cmsys_freq.armahb_freq, pcmahb->nrdiv);

    return(BCM_OK);             
}

/**
    @fn uint32_t clkmgr_set_peripheral_clk(clkmgr_blk_t blk, clkmgr_csrc_t csrc, uint32_t div)
*/
uint32_t clkmgr_set_peripheral_clk(clkmgr_blk_t blk, clkmgr_csrc_t csrc, uint32_t div)
{
    clkmgr_cmperip_u_t perip_val;
    BCM_ASSERT(blk < CLKMGR_BLK_COUNT);
    if (blk >= ECLKMGR_CMTIM0)
    {
        return(set_timer_clk(blk, csrc, div));        
    }
	BCM_ASSERT(div <= CLKMGR_PERICLK_DIV_MAX);
    perip_val.w_data = 0;
    perip_val.bf.csrc = csrc;
	perip_val.bf.div = (div == CLKMGR_PERICLK_DIV_MAX) ? 0 : div;
    amba_write32((uint32_t *)clkmgr_blks[blk].addr, perip_val.w_data);
    return(BCM_OK);
}

/**
    @fn uint32_t clkmgr_get_peripheral_clk(clkmgr_blk_t blk, 
                                           clkmgr_tcsrc_t *pcsrc,
                                           uint32_t *pdiv);
*/
uint32_t clkmgr_get_peripheral_clk(clkmgr_blk_t blk,
                                           clkmgr_tcsrc_t *pcsrc,
                                           uint32_t *pdiv)
{
	clkmgr_cmtim_u_t cmtim_val;
  clkmgr_cmperip_u_t perip_val;

  if (pclkmgr_base_addr == CLKMGR_POISON) {
    if (pcsrc)
      *pcsrc = 0;
    if (pdiv)
      *pdiv = 0;
    return BCM_ERROR;
  }

	//timers are treated differently, according to datasheet
	switch(blk)
	{
	    case ECLKMGR_CMTIM0:
        cmtim_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmtim0);
		break;

		case ECLKMGR_CMTIM1:
        cmtim_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmtim1);
		break;

		case ECLKMGR_CMPWM:
        cmtim_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpwm);
        break;

		case ECLKMGR_CMVCTIM:
        cmtim_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmvctim);
		break;

		default:
		/* non-timer peripheral */
        perip_val.w_data = amba_read32((uint32_t *)clkmgr_blks[blk].addr);

        if (pcsrc != NULL)
            *pcsrc = (clkmgr_tcsrc_t)perip_val.bf.csrc;

        if (pdiv != NULL)
            *pdiv = (perip_val.bf.div == 0) ? CLKMGR_PERICLK_DIV_MAX : perip_val.bf.div;

        return(BCM_OK);
    }

	/* "timer" peripheral */
    if (pcsrc != NULL)
        *pcsrc = (clkmgr_tcsrc_t)cmtim_val.bf.csrc;
                            
    if (pdiv != NULL)
        *pdiv = (cmtim_val.bf.div == 0) ? CLKMGR_TIMCLK_DIV_MAX : cmtim_val.bf.div;

    return(BCM_OK);
}

/**
    @fn uint32_t clkmgr_set_touchwheel_csrc_and_div(clkmgr_csrc_t csrc, uint32_t div)
*/
uint32_t clkmgr_set_touchwheel_csrc_and_div(clkmgr_csrc_t csrc, uint32_t div)
{
    clkmgr_cmtwspi_u_t cmtwspi_val;
   
	BCM_ASSERT(div <= 256);
	BCM_ASSERT((csrc == ECLKMGR_TCSRC_XTAL_24MHZ) || (csrc >= ECLKMGR_TCSRC_PLLS && csrc <= ECLKMGR_TCSRC_PLL3));
    cmtwspi_val.w_data = 0;
    cmtwspi_val.bf.csrc = csrc;
	cmtwspi_val.bf.div = (div==256) ? 0 : div ;
    amba_reg_write32(pclkmgr_base_addr, cmtwspi, cmtwspi_val.w_data);
    return(BCM_OK);
}
/**
    @fn uint32_t clkmgr_get_touchwheel_frequency(void)
*/
uint32_t clkmgr_get_touchwheel_frequency(void)
{
    clkmgr_cmtwspi_u_t cmtwspi_val;
    uint32_t div;
    uint32_t freq;
    cmtwspi_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmtwspi);
    div = (cmtwspi_val.bf.div == 0) ? CLKMGR_TWSPICLK_DIV_MAX : cmtwspi_val.bf.div;

	if (cmtwspi_val.bf.csrc == ECLKMGR_TCSRC_XTAL_24MHZ)
    {
    	freq = CLKMGR_XTAL_24MHZ_FREQUENCY;
    }
    else if ((cmtwspi_val.bf.csrc >= ECLKMGR_TCSRC_PLLS) &&     //Is a PLL the source?
             (cmtwspi_val.bf.csrc <= ECLKMGR_MAX_TCSRC))
    {
		//enum don't match, hence this fix
		freq = clkmgr_get_pll_frequency((clkmgr_pll_t)(cmtwspi_val.bf.csrc - ECLKMGR_TCSRC_PLLS));
    }
    else
    {
		return 0;
    }
    return bcm_udivide32_round(freq, div);
}
/**
    @fn uint32_t clkmgr_is_armblk_enabled(clkmgr_vc_block_t arm_blk);
*/
uint32_t clkmgr_is_armblk_enabled(clkmgr_arm_block_t arm_blk)
{
    uint32_t cmce0_val, cmce1_val;
    uint32_t ret, sync, async;

    if (pclkmgr_base_addr == CLKMGR_POISON)
      return 0;

    //Read current CMCE0 reg value
    cmce0_val = amba_reg_read32(pclkmgr_base_addr, cmce0); 

    //Read current CMCE1 reg value
    cmce1_val = amba_reg_read32(pclkmgr_base_addr, cmce1); 

    //Some peripherals are spilled over to the CMCE1 register.
    //we need to check bits in both cmce0 as well as cmce1 for IDE, CEATA and NAND
    switch(arm_blk)
    {
        case ECLKMGR_CM_IDE :
            //Is the block enabled?
            sync = (cmce0_val & arm_blk) ? BCM_BIT_SET : BCM_BIT_CLR;
            async = (cmce1_val & ECLKMGR_CM_ASIDE) ? BCM_BIT_SET : BCM_BIT_CLR;
            ret = sync & async;
        break;

        case ECLKMGR_CM_CEATA :
            //Is the block enabled?
            sync = (cmce0_val & arm_blk) ? BCM_BIT_SET : BCM_BIT_CLR;
            async = (cmce1_val & ECLKMGR_CM_ASCEA) ? BCM_BIT_SET : BCM_BIT_CLR;
            ret = sync & async;
        break;

        case ECLKMGR_CM_NAND :
            //Is the block enabled?
            sync = (cmce0_val & arm_blk) ? BCM_BIT_SET : BCM_BIT_CLR;
            async = (cmce1_val & ECLKMGR_CM_ASND) ? BCM_BIT_SET : BCM_BIT_CLR;
            ret = sync & async;
        break;

        default:
            //Is the block enabled?
            ret = (cmce0_val & arm_blk) ? BCM_BIT_SET : BCM_BIT_CLR; 
        break;
    }

    return(ret);
}

/**
    @fn void clkmgr_config_arm_block_clk(clkmgr_arm_block_t arm_blk, bcm_cfg_stat_t stat);
*/
void clkmgr_config_arm_block_clk(clkmgr_arm_block_t arm_blk, bcm_cfg_stat_t stat)
{
    //Note: No need to perform read-mod-write because a write '0'
    //      has no effect on the bits in this register. 

	if (stat == enable)
    {
        //Enable the block(s) specified in the input
        enable_arm_block_clk(arm_blk);
    }
    else
    {
        //Disable the block(s) specified in the input
        disable_arm_block_clk(arm_blk);
    }
    return;
}

/**
    @fn uint32_t clkmgr_is_vcblk_enabled(clkmgr_arm_block_t vc_blk);
*/
uint32_t clkmgr_is_vcblk_enabled(clkmgr_vc_block_t vc_blk)
{
    uint32_t reg_val;
    uint32_t ret;

    if (pclkmgr_base_addr == CLKMGR_POISON)
      return 0;

    //Read current reg value
    reg_val = amba_reg_read32(pclkmgr_base_addr, cmce1); 

    //Is the block enabled?
    ret = (reg_val & vc_blk) ? BCM_BIT_SET : BCM_BIT_CLR;
    return(ret);
}
/**
    @fn void clkmgr_config_arm_block_clk(clkmgr_config_vc_block_clk vc_blk, bcm_cfg_stat_t stat);
*/
void clkmgr_config_vc_block_clk(clkmgr_vc_block_t vc_blk, bcm_cfg_stat_t stat)
{
    if (stat == enable)
    {
        //Enable the block(s) specified in the input
        amba_reg_write32(pclkmgr_base_addr, cmce1, vc_blk);
    }
    else
    {
        //Disable the block(s) specified in the input
        amba_reg_write32(pclkmgr_base_addr, cmcec1, vc_blk);
    }

	return;
}
/**
    @fn uint32_t clkmgr_set_shared_perip_master_src(clkmgr_sel_access_t src);
*/
uint32_t clkmgr_set_shared_perip_master_src(clkmgr_sel_access_t src)
{
    clkmgr_cmahb_u_t cmahb_val;
    
	BCM_ASSERT(src == ECLKMGR_SEL_ARM_ACCESS || src==ECLKMGR_SEL_VC_ACCESS);

    cmahb_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);
	cmahb_val.bf.smsel = (src==ECLKMGR_SEL_VC_ACCESS) ? 1 : 0;
    amba_reg_write32(pclkmgr_base_addr, cmahb, cmahb_val.w_data);
    return(BCM_OK);
}

/**
    @fn clkmgr_sel_access_t clkmgr_get_shared_perip_master_src(void);
*/
clkmgr_sel_access_t clkmgr_get_shared_perip_master_src(void)
{
    clkmgr_cmahb_u_t cmahb_val;
  
    cmahb_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);
	return(cmahb_val.bf.smsel == 0 ? ECLKMGR_SEL_ARM_ACCESS : ECLKMGR_SEL_VC_ACCESS);
}
/**
    @fn uint32_t clkmgr_set_lcd_master_src(clkmgr_sel_access_t src);
*/
uint32_t clkmgr_set_lcd_master_src(clkmgr_sel_access_t src)
{
    clkmgr_cmahb_u_t cmahb_val;

    BCM_ASSERT(src == ECLKMGR_SEL_ARM_ACCESS || src==ECLKMGR_SEL_VC_ACCESS);

	cmahb_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);
	cmahb_val.bf.lmsel = (src==ECLKMGR_SEL_VC_ACCESS) ? 1 : 0;
    amba_reg_write32(pclkmgr_base_addr, cmahb, cmahb_val.w_data);
    return(BCM_OK);
}
/**
    @fn clkmgr_sel_access_t clkmgr_get_lcd_master_src(void);
*/
clkmgr_sel_access_t clkmgr_get_lcd_master_src(void)
{
    clkmgr_cmahb_u_t cmahb_val;

	cmahb_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmahb);
    return(cmahb_val.bf.lmsel == 0 ? ECLKMGR_SEL_ARM_ACCESS : ECLKMGR_SEL_VC_ACCESS);
}

/**
    @fn clkmgr_dram_mode_t clkmgr_get_dram_clk_mode(void);
*/
clkmgr_dram_mode_t clkmgr_get_dram_clk_mode(void)
{
    clkmgr_cmreqcs_u_t cmreqcs;

    if (pclkmgr_base_addr == CLKMGR_POISON)
      // possibly this needs to return something different for other units?
      return ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK;
    else {
      cmreqcs.w_data = amba_reg_read32(pclkmgr_base_addr, cmreqcs);
      return(cmreqcs.bf.dcasync);
    }
}


/**
    @fn uint32_t clkmgr_set_dram_clk_mode(clkmgr_dram_mode_t mode);
*/
uint32_t clkmgr_set_dram_clk_mode(clkmgr_dram_mode_t mode)
{
    clkmgr_cmreqcs_u_t cmreqcs;

    BCM_ASSERT((mode == ECLKMGR_DRAMCLK_SYNC_TO_EMICLK) || (mode == ECLKMGR_DRAMCLK_ASYNC_TO_EMICLK));

	cmreqcs.w_data = amba_reg_read32(pclkmgr_base_addr, cmreqcs); 
    cmreqcs.bf.dcasync = mode; 
    amba_reg_write32(pclkmgr_base_addr, cmreqcs, cmreqcs.w_data);    
    return(BCM_OK);
}

/**
    Private functions used within this file  
*/

/**
    @fn static void enable_arm_block_clk(clkmgr_arm_block_t arm_blk);
*/
static void enable_arm_block_clk(clkmgr_arm_block_t arm_blk)
{
    //Some peripherals are spilled over to the CMCE1 register.
    //we need to enable bits in both cmce0 as well as cmce1 for IDE, CEATA and NAND
    switch(arm_blk)
    {
        case ECLKMGR_CM_IDE :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmce0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmce1, ECLKMGR_CM_ASIDE);
        break;

        case ECLKMGR_CM_CEATA :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmce0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmce1, ECLKMGR_CM_ASCEA);
        break;

        case ECLKMGR_CM_NAND :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmce0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmce1, ECLKMGR_CM_ASND);
        break;

        default:
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmce0, arm_blk);
        break;
    }

    return;
}

/**
    @fn static void disable_arm_block_clk(clkmgr_arm_block_t arm_blk);
*/
static void disable_arm_block_clk(clkmgr_arm_block_t arm_blk)
{
    //Some peripherals are spilled over to the CMCE1 register.
    //we need to disable bits in both cmce0 as well as cmce1 for IDE, CEATA and NAND
    switch(arm_blk)
    {
        case ECLKMGR_CM_IDE :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmcec0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmcec1, ECLKMGR_CM_ASIDE);
        break;

        case ECLKMGR_CM_CEATA :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmcec0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmcec1, ECLKMGR_CM_ASCEA);
        break;

        case ECLKMGR_CM_NAND :
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmcec0, arm_blk);
            amba_reg_write32(pclkmgr_base_addr, cmcec1, ECLKMGR_CM_ASND);
        break;

        default:
            //Enable the block(s) specified in the input
            amba_reg_write32(pclkmgr_base_addr, cmcec0, arm_blk);
        break;
    }

    return;
}


//--------------------------------------------------
/**
   Wait for PLL lock.
    @param  pll     which pll?

	@special
        The function will ensure that the PLL is locked before returning. 
    @retval   BCM_OK    - Success
    @retval   BCM_ERROR - Failed - most likely the PLL did not lock
    @ingroup  CLKMGR
*/
//--------------------------------------------------
static uint32_t wait_for_pll_lock(clkmgr_pll_t pll)
{
    clkmgr_cmpll_u_t pll_val;
	volatile int lock_count=0;
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT);
    pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
    //Is this PLL enabled?
    if (pll_val.bf.enab == 0)
	{
        return(BCM_OK);
    }
	/* Yes, it is enabled, spin in a loop until it is locked.*/
	while(pll_val.bf.lock == 0)
    {
		++lock_count;
		if (lock_count > CLKMGR_MAX_PLL_LOCK_COUNT)
		{
            bcm_log_debug("%s: Pll-lock failed, pll[%d][0x%08x]\n", __FUNCTION__, pll, pll_val.w_data);
		    return(BCM_ERROR);
  		}
        pll_val.w_data = amba_reg_read32(pclkmgr_base_addr, cmpll[pll]);
    }

	return(BCM_OK);
}
/**
    @fn static uint32_t set_timer_clk(clkmgr_blk_t blk, clkmgr_tcsrc_t csrc, uint32_t div)
*/
static uint32_t set_timer_clk(clkmgr_blk_t blk, clkmgr_tcsrc_t csrc, uint32_t div)
{
    clkmgr_cmtim_u_t cmtim_val;

	BCM_ASSERT(blk == ECLKMGR_CMTIM0 || blk == ECLKMGR_CMTIM1 || blk==ECLKMGR_CMPWM || blk==ECLKMGR_CMVCTIM);
	BCM_ASSERT(div <= CLKMGR_TIMCLK_DIV_MAX);
    cmtim_val.w_data = 0;
    cmtim_val.bf.csrc = csrc;
	cmtim_val.bf.div = (div==CLKMGR_TIMCLK_DIV_MAX) ? 0 : div;

	if (blk == ECLKMGR_CMTIM0)
    {
        amba_reg_write32(pclkmgr_base_addr, cmtim0, cmtim_val.w_data);
    }
    else if (blk == ECLKMGR_CMTIM1)
    {
        amba_reg_write32(pclkmgr_base_addr, cmtim1, cmtim_val.w_data);
    }
    else if (blk == ECLKMGR_CMPWM)
    {
        amba_reg_write32(pclkmgr_base_addr, cmpwm, cmtim_val.w_data);
    }
    else if (blk == ECLKMGR_CMVCTIM)
    {
        amba_reg_write32(pclkmgr_base_addr, cmvctim, cmtim_val.w_data);
    }
    return(BCM_OK);
}
static uint64_t udivide64_round_up(uint64_t numer, uint64_t denom)
{
	return bcm_udivide64(numer + denom -1, denom);
}
/**
    @fn static int calc_pll_dividers(unsigned long desired_freq, uint32_t *best_qdiv, uint32_t *best_pdiv)
    @brief Without floating point math, we may not get the best divisors. This method attempts
           to find the best divisors that provides a frequency that is equal to or higher than
           the desired frequency.
*/
static uint32_t calc_pll_dividers(
						uint32_t desired_freq, 
						uint32_t *best_qdiv, 
						uint32_t *best_pdiv,
						uint32_t *best_freq)
{
    uint32_t		qdiv, pdiv;
	uint64_t	rhs, lhs;
	uint64_t	best_diff=0xFFFFFFFFFFFFFFFFLL;

	*best_pdiv = 0;    
	*best_qdiv = CLKMGR_PLL_QDIV_HIGH_VALUE;
    for (pdiv=CLKMGR_PLL_PDIV_HIGH_VALUE; pdiv >= CLKMGR_PLL_PDIV_LOW_VALUE; --pdiv)
    {
		rhs = (uint64_t)pdiv * desired_freq;
		qdiv=udivide64_round_up(rhs, CLKMGR_XTAL_24MHZ_FREQUENCY); 

		if( qdiv < CLKMGR_PLL_QDIV_LOW_VALUE )
			qdiv = CLKMGR_PLL_QDIV_LOW_VALUE;
		else if (qdiv > CLKMGR_PLL_QDIV_HIGH_VALUE )
			continue;

		lhs = CLKMGR_XTAL_24MHZ_FREQUENCY * (uint64_t)qdiv;

		if ((lhs >= rhs) && ((lhs - rhs) < best_diff))
        {
			/* this frequency is closest (so far) AND higher than desired frequency */
			best_diff = lhs - rhs;
			*best_pdiv = pdiv;
			*best_qdiv = qdiv;
        }
    }

	*best_freq = bcm_udivide64_round(
		CLKMGR_XTAL_24MHZ_FREQUENCY * (uint64_t)*best_qdiv, 
			*best_pdiv);

	bcm_log_debug("%s, Final: qdiv=%u, pdiv=%u, act_freq=%lu\n", 
				__FUNCTION__, qdiv, pdiv, *best_freq);
	return(BCM_OK);
}
/* divider lookup table, in ordet of ascending values */

const static struct _cmsys_div_lookup
{
	uint8_t   div; /* 1...8 */
	uint8_t   mtxdiv; /* 1...4 */
	uint16_t  effective_div; /* div x mtxdiv */
} cmsys_div_lookup_table[]=
{
	/* sorted in increasing order of effective_div and decreasing order of mtxdiv */
	//{1,1,1,},
	//{2,1,2,},
	{1,2,2,},
	//{1,3,3,},
	//{3,1,3,},
	{1,4,4,},
	{2,2,4,},
	//{4,1,4,},
	//{5,1,5,},
	//{2,3,6,},
	{3,2,6,},
	//{6,1,6,},
	//{7,1,7,},
	{2,4,8,},
	{4,2,8,},
	//{8,1,8,},
	//{3,3,9,},
	{5,2,10,},
	{3,4,12,},
	{6,2,12,},
	{7,2,14,},
	//{5,3,15,},
	{4,4,16,},
	{8,2,16,},
	//{6,3,18,},
	{5,4,20,},
	//{7,3,21,},
	{6,4,24,},
	//{8,3,24,},
	{7,4,28,},
	{8,4,32,},

};

static uint32_t	recalc_cmsys_divs(unsigned int	div_new, unsigned int mtxdiv_new, clkmgr_cmsys_clk_settings_t *cmsys)
{
	unsigned int	ahdiv_old, vhdiv_old, mtxdiv_old, div_old;
	unsigned int	ahdiv_new=0, vhdiv_new=0;

	/* read the old dividers. This is the ratio we want to preserve */
	div_old = cmsys->div;
	mtxdiv_old = cmsys->mtxdiv;
	ahdiv_old = cmsys->ahdiv;
	vhdiv_old = cmsys->vhdiv;
	
	// Now, make sure that the existing ratio of MTX:VC, and MTX:ARM_AHB can be preserved.
	ahdiv_new = bcm_udivide32_round((ahdiv_old * mtxdiv_new),  mtxdiv_old );
	vhdiv_new = bcm_udivide32_round((vhdiv_old * mtxdiv_new),  mtxdiv_old );
	if( ahdiv_new >= 1 && ahdiv_new <=64 && vhdiv_new >=1 && vhdiv_new <=64 )
	{
		// SUCCESS: We have found a working set of dividers which can produce
		// the desired MTX frequency while at the same time preserving the 
		// frequency ratiins MTX:VC and MTX:ARM_AHB
		cmsys->ahdiv = ahdiv_new;
		cmsys->vhdiv = vhdiv_new;
		cmsys->mtxdiv = mtxdiv_new;
		cmsys->div = div_new;
		return BCM_OK;
	}

	bcm_log_error("%s(): With CMSYS:MTXDIV=%u and CMSYS:DIV=%u, cannot get legal AHDIV and VHDIV and still maintain existing ratio  of dividers...\n",
		__FUNCTION__,(unsigned int)mtxdiv_new, (unsigned int)div_new);
	return BCM_ERROR; /* mtx frequency is fine, but the VC02 and AHB frequencies are not fine */
}

/* given the frequency of a "core", calculates the optimal
 * PLL frequency needed for CMSYS */
uint32_t	clkmgr_calc_pll_freq_for_cmsys(
					unsigned long						mtx_freq_desired,
					unsigned long						*pll_freq_actual,
					clkmgr_cmsys_clk_settings_t			*cmsys)
{
	int				i;
	unsigned int	div, mtxdiv;

	BCM_ASSERT(pll_freq_actual != NULL)
	BCM_ASSERT(cmsys != NULL);
	BCM_ASSERT(ARRAYSIZE(cmsys_div_lookup_table) != 0);

	for(i=0; i < ARRAYSIZE(cmsys_div_lookup_table); ++i)
	{
		*pll_freq_actual = mtx_freq_desired * cmsys_div_lookup_table[i].effective_div;
		if( *pll_freq_actual >= CLKMGR_PLL_FREQ_MIN )
		{
			if( is_dram_div_valid(cmsys_div_lookup_table[i].mtxdiv, cmsys->emidiv)==BCM_OK)
				break; /* found it */
			else
				bcm_log_warn("%s(): skipping. *pll_freq_actual=%lu, mtxdiv=%u, emidiv=%u\n",
											__FUNCTION__, *pll_freq_actual, 
											cmsys_div_lookup_table[i].mtxdiv, cmsys->emidiv);
		}
	}

	if( *pll_freq_actual < CLKMGR_PLL_FREQ_MIN )
	{
		bcm_log_warn("%s(): To produce MTX frequency of %lu Hz, need a PLL frequency of %lu Hz, which is too low\n",
							__FUNCTION__, (unsigned long) mtx_freq_desired, 
							(unsigned long)*pll_freq_actual);

	}

	if( i == ARRAYSIZE(cmsys_div_lookup_table) )
	{
		i = ARRAYSIZE(cmsys_div_lookup_table) -1;
		*pll_freq_actual = mtx_freq_desired * cmsys_div_lookup_table[i].effective_div;
	}

	div = cmsys_div_lookup_table[i].div;
	mtxdiv = cmsys_div_lookup_table[i].mtxdiv;
	return recalc_cmsys_divs(div, mtxdiv, cmsys);
}

uint32_t clkmgr_calc_cmsys_divs(uint32_t source_freq, 
					unsigned long mtx_freq_desired, 
					unsigned long *mtx_freq_actual, 	
					clkmgr_cmsys_clk_settings_t *cmsys)
{
	unsigned int	mtx_freq_temp=UINT32_MAX;
	unsigned int	mtxdiv=0, div=0;
	int				i;

	BCM_ASSERT(mtx_freq_actual != NULL);
	BCM_ASSERT(cmsys != NULL);
	BCM_ASSERT(ARRAYSIZE(cmsys_div_lookup_table) != 0);

	*mtx_freq_actual=UINT32_MAX; 

	for(i=0; i < ARRAYSIZE(cmsys_div_lookup_table); ++i)
	{
		mtx_freq_temp = bcm_udivide32_round(source_freq, cmsys_div_lookup_table[i].effective_div);
		
		if( mtx_freq_temp < mtx_freq_desired )
			break; // no luck
		else if( mtx_freq_temp < *mtx_freq_actual && 
					mtx_freq_temp >= mtx_freq_desired)
		{
			// best value so far...
			*mtx_freq_actual = mtx_freq_temp;
			div = cmsys_div_lookup_table[i].div;
			mtxdiv = cmsys_div_lookup_table[i].mtxdiv;
		}
	}
	
	if( *mtx_freq_actual == UINT32_MAX )
	{
		/* request could not be satisfied. The lowest possible
		 *  dividers resulted in a frequency that is lower
		 *  than the desired frequency 
		 */
		return BCM_ERROR; 
	}

	bcm_log_debug("i=%d, ARRAYSIZE(cmsys_div_lookup_table)=%d\n", (int)i, ARRAYSIZE(cmsys_div_lookup_table));
	if( i >= ARRAYSIZE(cmsys_div_lookup_table) )
	{
		bcm_log_warn("%s(): dividers needed are too high to get frequency %lu Hz, but continuing anyway...\n",
			__FUNCTION__,
			(unsigned long)mtx_freq_desired);
		bcm_log_warn("%s(): actual frequency would be %lu Hz\n", __FUNCTION__, (unsigned long)*mtx_freq_actual);

	}

	return recalc_cmsys_divs(div, mtxdiv, cmsys);
}

/* returns TRUE if successful. returns FALSE on failure */
int		clkmgr_calc_cmsys_src_and_divs(
						unsigned long								mtx_freq_desired, 
						clkmgr_pll_t					pll_desired,
						unsigned long *								mtx_freq_actual, 
						clkmgr_cmsys_clk_settings_t *	cmsys,
						int										tolerance_percent)
{
	if( (mtx_freq_desired <= CLKMGR_XTAL_32KHZ_FREQUENCY))
	{
		*mtx_freq_actual = CLKMGR_XTAL_32KHZ_FREQUENCY;
		if( tolerance_percent == 0 ||
			( (*mtx_freq_actual > mtx_freq_desired) && 
				(bcm_get_deviation_percent(mtx_freq_desired, *mtx_freq_actual) <= tolerance_percent) ) )
		{

			/* In this mode, all dividers are set to 1, for simplicity */
			cmsys->csrc = ECLKMGR_TCSRC_XTAL_32KHZ;
			cmsys->div = 1;
			cmsys->vhdiv = 1;
			cmsys->mtxdiv = 1;
			cmsys->ahdiv = 1;

			return TRUE;
		}
		else
			return FALSE;
	}

	if(mtx_freq_desired <= CLKMGR_XTAL_24MHZ_FREQUENCY)
	{
		/* Note:
		 * In this case, we are happy as long as the 24MHz oscillator is able to
		 * just satisfy the request, i.e. as long as the MTX frequency is <=24MHz
		 * and the ratios can be maintained. We do not check if the actual MTX frequency
		 * is "close" enough to the desired frequency or not.
		 */
		 
		if( clkmgr_calc_cmsys_divs(CLKMGR_XTAL_24MHZ_FREQUENCY, 
									mtx_freq_desired, 
									mtx_freq_actual, 
									cmsys) == BCM_OK)
		{
			// This means we are able to generate an actual frequency
			// which is greater than desired, and at the same time, uses
			// just the 24MHz oscillator.

			// Really, this is the best that can be done.

			if( tolerance_percent == 0 )
			{
				/* dont care about tolerance. Just produce the very best */
				cmsys->csrc = ECLKMGR_TCSRC_XTAL_24MHZ;
				return TRUE;
			}
			else if( bcm_get_deviation_percent(mtx_freq_desired, *mtx_freq_actual) <= tolerance_percent ) 
			{
				cmsys->csrc = ECLKMGR_TCSRC_XTAL_24MHZ;
				return TRUE;
			}
		}

		// request cannot be satisfied within tolerance percent...
		return FALSE;
	}


	// Try the desired PLL
	if( clkmgr_calc_cmsys_divs(
				clkmgr_get_pll_frequency(pll_desired), 
				mtx_freq_desired, 
				mtx_freq_actual, 
				cmsys) == BCM_OK )
	{

		if( tolerance_percent==0 )
		{
			if( mtx_freq_desired == *mtx_freq_actual )
			{
				cmsys->csrc = pll_desired + ECLKMGR_CSRC_PLLS;
				return TRUE;
			}
		}
		else if( bcm_get_deviation_percent(mtx_freq_desired, *mtx_freq_actual) <= tolerance_percent ) 
		{
			cmsys->csrc = pll_desired + ECLKMGR_CSRC_PLLS;
			return TRUE;
		}
	}

	return FALSE; /* the current sources and dividers are not enough */

}
uint32_t clkmgr_clock_switch(
		const clkmgr_cmsys_clk_settings_t		*cmsys_new,
		const clkmgr_cmahb_clk_settings_t		*cmahb_new)
{
    clkmgr_cmahb_u_t	cmahb_reg;
    clkmgr_cmsys_u_t	cmsys_reg;
	volatile register uint32_t								cmahb_val;
	volatile register uint32_t								cmsys_val;
	volatile register clkmgr_cmreqcs_u_t			cmreqcs_reg;
	volatile register clkmgr_regs_t					*pclkmgr_base_local_copy=pclkmgr_base_addr;
	register uint32_t	zero=0;

	if ( clkmgr_are_cmahb_clk_settings_valid(cmahb_new) != BCM_OK )
		return BCM_ERROR;

	if ( clkmgr_are_cmsys_clk_settings_valid(cmsys_new) != BCM_OK )
		return BCM_ERROR;

	// set up the values to be written to CMSYS and CMAHB
	set_cmahb_clk_settings_helper(cmahb_new, &cmahb_reg);
	set_cmsys_clk_settings_helper(cmsys_new, &cmsys_reg);

	cmahb_val = cmahb_reg.w_data;
	cmsys_val = cmsys_reg.w_data;

	// getting into SDRAM dead zone. -- after CMSYS gets updated and until
	// CMREQCS:reqstat reads zero, SDRAM cannot be accessed.

	//flush i-cache.
	// The idea is since we are flussing the i-cache, the code following it will
	// be loaded into cache. Therefore, when this code is fetched for execution, it
	// will not have to be fetched from SDRAM. 
	__asm("mcr p15, 0, %0, c7, c5, 0": :"r"(zero));

    amba_reg_write32(pclkmgr_base_local_copy, cmahb, cmahb_val);
    amba_reg_write32(pclkmgr_base_local_copy, cmsys, cmsys_val);
	//wait for clock switch completion. This will wait until CMREQCS:RESTAT is zero
	do
	{
		cmreqcs_reg.w_data = amba_reg_read32(pclkmgr_base_local_copy, cmreqcs); 
	}    while( cmreqcs_reg.bf.reqstat != 0); //'0' indicates clock switch is done

	//We are out of "SDRAM Dead Zone"

	return BCM_OK;
}

uint32_t clkmgr_round_rate_pll_frequency(clkmgr_pll_t pll, 
								   uint32_t desired_freq,
                                   int tolerance_percent,
                                   uint32_t *actual_freq)
{
    uint32_t ret_val;
    uint32_t deviation_percent;

    clkmgr_pll_settings_t pll_info;
    BCM_ASSERT(pll < CLKMGR_PLL_COUNT) 
	BCM_ASSERT(actual_freq != NULL);

    pll_info.pdiv = pll_info.qdiv;
    ret_val = calc_pll_dividers(desired_freq, &pll_info.qdiv, &pll_info.pdiv, 
										actual_freq);
    if (ret_val == BCM_ERROR)
    {
        bcm_log_error("%s: No suitable qdiv and pdiv for frequency=%u Hz\n", 
			__FUNCTION__, desired_freq);
        return(BCM_ERROR);
    }
    //Are we exceeding the limits?
	if( tolerance_percent != 0 )
	{
		deviation_percent = bcm_get_deviation_percent(desired_freq, *actual_freq);
        if (deviation_percent > tolerance_percent)
	    {
            bcm_log_debug("%s: Best available frequency[%ld] deviation[%d]\n", __FUNCTION__, 
                        *actual_freq, deviation_percent);

    		return(BCM_ERROR);
        }
	}
    return(BCM_OK);
}
