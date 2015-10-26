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
 *   @file   armbusmtx.c 
 * 
 *   @brief ARM Bus matrix APIs  
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_privtypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_amba_io.h"

#include "armbusmtx.h"
#include "armbusmtx_regs.h"
#include <asm-arm/arch-bcm4760/bcm4760_addressmap.h>

/* Chip manager block base address */
static armbusmtx_regs_t *parmbusmtx_base_addr = 0;
                                    
//--------------------------------------------------
/**
    Chip manager public APIs
*/
//--------------------------------------------------
/**
    @fn void armbusmtx_init(bcm_dev_virtaddr_t virt_addr)
*/
void armbusmtx_api_init(void)
{
#if 0 // not applicable to 4760
    parmbusmtx_base_addr = (armbusmtx_regs_t *)bcm_xlate_to_virt_addr(BCM28XX_MATRIXARB_ADDRBASE0);
#else
    parmbusmtx_base_addr = 0xdeadbeef; // point to invalid address for BCM4760
#endif
}

/**
    @fn uint32_t armbusmtx_set_synctop(armbusmtx_synctop_t synctop, const armbusmtx_synctop_info_t *psynctop_info);
*/
uint32_t armbusmtx_set_synctop(armbusmtx_synctop_t synctop, const armbusmtx_synctop_info_t *psynctop_info)
{
    armbusmtx_syncctrl_u_t syncctrl_val;

    BCM_ASSERT((synctop < ARMBUSMTX_SYNCTOP_COUNT) && (psynctop_info != NULL));

    syncctrl_val.w_data = amba_reg_read32(parmbusmtx_base_addr, sync_ctrl[synctop]);

    //Set synctop mode
    if (psynctop_info->mode == ARMBUSMTX_SYNCTOP_MODE_BYPASS)
    {
        syncctrl_val.bf.bypass_sync_top = 1;
    }
    else
    {
        syncctrl_val.bf.latency_mode = (psynctop_info->mode == ARMBUSMTX_SYNCTOP_MODE_LATENCY) ?  1 : 0;
    }

    //Set the clk mode
	switch(psynctop_info->clkmode)
	{
		case ARMBUSMTX_SYNCTOP_CLKMODE_1toN:
        syncctrl_val.bf.clock_mode_1ton = 1;
		syncctrl_val.bf.clock_mode_nto1 = 0;
		break;

		case ARMBUSMTX_SYNCTOP_CLKMODE_Nto1:
        syncctrl_val.bf.clock_mode_1ton = 0;
        syncctrl_val.bf.clock_mode_nto1 = 1;
		break;

		case ARMBUSMTX_SYNCTOP_CLKMODE_1to1:
        syncctrl_val.bf.clock_mode_1ton = 0;
        syncctrl_val.bf.clock_mode_nto1 = 0;
		break;

		default:
		BCM_ASSERT(!"(Setting psynctop_info->clkmode to invalid value");
        syncctrl_val.bf.clock_mode_1ton = 0;
        syncctrl_val.bf.clock_mode_nto1 = 0;
		break;
	}

    if (psynctop_info->mode == ARMBUSMTX_SYNCTOP_MODE_LATENCY)
    {
        syncctrl_val.bf.low_watermark  = psynctop_info->low_watermark;
        syncctrl_val.bf.high_watermark = psynctop_info->high_watermark ;
    }

    //Is this update related to a clock change?
    syncctrl_val.bf.clock_change = psynctop_info->clk_change;

    syncctrl_val.bf.enable_posted_write = 1;  //Must be always set to 1. Reference - BCM2820 datasheet

    amba_reg_write32(parmbusmtx_base_addr, sync_ctrl[synctop], syncctrl_val.w_data);

    return(BCM_OK);
}


/**
    @fn uint32_t armbusmtx_get_synctop(armbusmtx_synctop_t synctop, armbusmtx_synctop_info_t *psynctop_info);
*/
uint32_t armbusmtx_get_synctop(armbusmtx_synctop_t synctop, armbusmtx_synctop_info_t *psynctop_info)
{
    armbusmtx_syncctrl_u_t syncctrl_val;

    BCM_ASSERT((synctop < ARMBUSMTX_SYNCTOP_COUNT) && (psynctop_info != NULL));

    syncctrl_val.w_data = amba_reg_read32(parmbusmtx_base_addr, sync_ctrl[synctop]);

    //Get synctop mode
    if (syncctrl_val.bf.bypass_sync_top)
    {
        psynctop_info->mode = ARMBUSMTX_SYNCTOP_MODE_BYPASS;
    }
    else
    {
        psynctop_info->mode = (syncctrl_val.bf.latency_mode == BCM_BIT_SET) ? ARMBUSMTX_SYNCTOP_MODE_LATENCY : ARMBUSMTX_SYNCTOP_MODE_BANDWIDTH;
    }

    //Get the clk mode
    if (syncctrl_val.bf.clock_mode_1ton==1 && syncctrl_val.bf.clock_mode_nto1==0)
    {
        psynctop_info->clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_1toN;                          
    }                                                       
    else if (syncctrl_val.bf.clock_mode_1ton==0 && syncctrl_val.bf.clock_mode_nto1==1)              
    {
        psynctop_info->clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_Nto1;
    }
    else if (syncctrl_val.bf.clock_mode_1ton==0 && syncctrl_val.bf.clock_mode_nto1==0)              
    {
        psynctop_info->clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_1to1;
    }
	else
	{
		BCM_ASSERT("!syncctrl_val.bf: clock_mode_1ton and clock_mode_nto1 are both zeros");
		psynctop_info->clkmode = ARMBUSMTX_SYNCTOP_CLKMODE_1to1;
	}
        
    psynctop_info->prefetch_len = syncctrl_val.bf.prefetch_length;

    if (psynctop_info->mode == ARMBUSMTX_SYNCTOP_MODE_LATENCY)
    {
        psynctop_info->low_watermark  = syncctrl_val.bf.low_watermark;
        psynctop_info->high_watermark = syncctrl_val.bf.high_watermark;
    }

    return(BCM_OK);
}


/**
    @fn uint32_t armbuxmtx_synctop_set_prefetch_len(armbusmtx_synctop_t synctop, uint32_t prefetch_len);
*/
uint32_t armbuxmtx_synctop_set_prefetch_len(armbusmtx_synctop_t synctop, uint32_t prefetch_len)
{
    armbusmtx_syncctrl_u_t syncctrl_val;

    BCM_ASSERT(synctop < ARMBUSMTX_SYNCTOP_COUNT);

    syncctrl_val.w_data = amba_reg_read32(parmbusmtx_base_addr, sync_ctrl[synctop]);

    syncctrl_val.bf.prefetch_length = prefetch_len;

    amba_reg_write32(parmbusmtx_base_addr, sync_ctrl[synctop], syncctrl_val.w_data);

    return(BCM_OK);
}

/**
    @fn uint32_t armbuxmtx_synctop_set_watermarks(armbusmtx_synctop_t synctop, uint32_t low_watermark, uint32_t high_watermark);
*/
uint32_t armbuxmtx_synctop_set_watermarks(armbusmtx_synctop_t synctop, uint32_t low_watermark, uint32_t high_watermark)
{
    armbusmtx_syncctrl_u_t syncctrl_val;

    BCM_ASSERT(synctop < ARMBUSMTX_SYNCTOP_COUNT);

    syncctrl_val.w_data = amba_reg_read32(parmbusmtx_base_addr, sync_ctrl[synctop]);

    syncctrl_val.bf.low_watermark = low_watermark;
    syncctrl_val.bf.high_watermark = high_watermark;

    amba_reg_write32(parmbusmtx_base_addr, sync_ctrl[synctop], syncctrl_val.w_data);

    return(BCM_OK);
}

