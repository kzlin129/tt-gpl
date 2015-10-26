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
 *   @file   emi_regs.h
 * 
 *   @brief  EMI register definitions.
 * 
 ****************************************************************************/

#ifndef _EMI_REGS_H_
#define _EMI_REGS_H_

#include "bcm_basetypes.h"
#include "bcm_reg.h"


//--------------------------------------------------
/**
	EMI_EMI_SOFT_RESET reset register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    emi_clk_reset  : 1;  
    uint32_t    dram_clk_reset  : 1; 
    uint32_t    rsvd0    : 30; 
}emi_emi_soft_reset_t;

AMBA_REG_BLD_UNION32(emi_emi_soft_reset);

//--------------------------------------------------
/**
	EMI_CNTRLR_CONFIG register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    dram_type  : 1;  
    uint32_t    dram_width  : 1;  
    uint32_t    rsvd0    : 2; 
    uint32_t    col_bits    : 4; 
    uint32_t    arm_cntrl_port_config    : 1; 
    uint32_t    arm_cntrl_fifo_path_en    : 1; 
    uint32_t    arm_aux_fifo_path_en    : 1; 
    uint32_t    date_coherency_mode_sel    : 1; 
    uint32_t    data_coherency_en    : 1;
    uint32_t    rsvd1    : 19; 
}emi_cntrlr_config_t;

AMBA_REG_BLD_UNION32(emi_cntrlr_config);


//--------------------------------------------------
/**
	EMI_DRAM_TIMING0 register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    trp_nop		:3;  
    uint32_t    rsvd0		:1;  
    uint32_t    trrd_nop    :2; 
    uint32_t    rsvd1		:2; 
    uint32_t    rd2wr_nop   :3; 
    uint32_t    rsvd2		:1; 
    uint32_t    wr2rd_nop   :3; 
    uint32_t    rsvd3		:17; 
}emi_dram_timing0_t;

AMBA_REG_BLD_UNION32(emi_dram_timing0);

//--------------------------------------------------
/**
	EMI_DRAM_TIMING1 register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    trfc_nop	: 5;  
    uint32_t    rsvd0		: 3;  
    uint32_t    tras_nop    : 4; 
    uint32_t    twr_nop		: 3; 
    uint32_t    rsvd1		: 1; 
    uint32_t    trcd_nop    : 3; 
    uint32_t	rsvd2		: 1; 
    uint32_t    tmrd_nop    : 2; 
    uint32_t    rsvd3		: 2; 
    uint32_t    txsr_nop    : 6; 
    uint32_t    rsvd4		: 2; 
}emi_dram_timing1_t;

AMBA_REG_BLD_UNION32(emi_dram_timing1);


//--------------------------------------------------
/**
	EMI_DRAM_MODE_SET reset register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    cl		: 3;  
    uint32_t    rsvd0	: 1;  
    uint32_t    pasr    : 3; 
    uint32_t    rsvd1   : 1; 
    uint32_t    tcsr    : 2; 
    uint32_t    rsvd2   : 2; 
    uint32_t    ds		: 2; 
    uint32_t    rsvd3   : 17; 
}emi_dram_mode_set_t;

AMBA_REG_BLD_UNION32(emi_dram_mode_set);

//--------------------------------------------------
/**
	EMI_DDR_DLL_PHASE_HIGH, EMI_DDR_DLL_PHASE_LOW and EMI_DDR_DLL_PHASE_FILTERED registers
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    lock_val	: 6;  
    uint32_t    rsvd0		: 26;  
}emi_ddr_dll_phase_t;

AMBA_REG_BLD_UNION32(emi_ddr_dll_phase);

//--------------------------------------------------
/**
	EMI_DDR_DLL_LOCK_BIT register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    dll_lock	: 1;  
    uint32_t    rsvd0	: 31;  
}emi_ddr_dll_lock_bit_t;

AMBA_REG_BLD_UNION32(emi_ddr_dll_lock_bit);


//--------------------------------------------------
/**
	EMI_REFRESH_CNTRL register
	@ingroup EMI
*/
//--------------------------------------------------
typedef struct
{
    uint32_t    ref_period	: 16;  
    uint32_t    ref_clust	: 2;  
	uint32_t	rsvd0		: 14;
}emi_refresh_cntrl_t;

AMBA_REG_BLD_UNION32(emi_refresh_cntrl);

//--------------------------------------------------
/**
	EMI_CLIENT0_ACCESS_TIMEOUT_PERIOD...EMI_CLIENT4_ACCESS_TIMEOUT_PERIOD registers
	@ingroup EMI
*/
//--------------------------------------------------

typedef struct
{
    uint32_t    ref_period	: 16;  
    uint32_t    ref_clust	: 2;  
	uint32_t	rsvd0		: 14;
}emi_client_access_timeout_period_t;

AMBA_REG_BLD_UNION32(emi_client_access_timeout_period);

typedef struct
{
    uint32_t    inact_cnt	    : 12 ;  
    uint32_t    pdn_enter_mode	: 2  ;  
	uint32_t	rsvd0		    : 18 ;
}emi_pdown_mode_t;

AMBA_REG_BLD_UNION32(emi_pdown_mode);


//--------------------------------------------------
/**
    This structure is the mapping of EMI registers.

    @ingroup EMI
*/
//--------------------------------------------------

typedef struct
{
  bcm_reg32_t emi_rev_id;
  bcm_reg32_t emi_emi_soft_reset;
  bcm_reg32_t emi_cntrlr_config;
  bcm_reg32_t emi_dram_timing0;
  bcm_reg32_t emi_dram_timing1;
  bcm_reg32_t emi_dram_mode_set;
  emi_pdown_mode_u_t emi_power_down_mode;
  bcm_reg32_t emi_cntrlr_start_seq;
  bcm_reg32_t emi_ddr_rddqs_gate_cntrl;
  bcm_reg32_t emi_ddr_ncdl_mode;
  bcm_reg32_t emi_ddr_dll_mode;
  bcm_reg32_t emi_ddr_read_ncdl_offset;
  bcm_reg32_t emi_ddr_write_ncdl_offset;
  bcm_reg32_t emi_ddr_dll_phase_load_enable;
  bcm_reg32_t emi_ddr_dll_phase_load_value;
  emi_ddr_dll_phase_u_t						emi_ddr_dll_phase_high;
  emi_ddr_dll_phase_u_t						emi_ddr_dll_phase_low;
  emi_ddr_dll_phase_u_t						emi_ddr_dll_phase_filtered;
  emi_ddr_dll_lock_bit_u_t					emi_ddr_dll_lock_bit;
  bcm_reg32_t										emi_sdr_read_reclock;
  bcm_reg32_t										emi_power_down_status;
  emi_refresh_cntrl_u_t						emi_refresh_cntrl;
  bcm_reg32_t emi_dram_bank_close_timer;
  bcm_reg32_t emi_client_preferred_mode_enable;
  bcm_reg32_t emi_request_mask;
  emi_client_access_timeout_period_u_t	 emi_client_access_timeout_period[5];
  bcm_reg32_t emi_non_pref_client_req_disable;
  bcm_reg32_t emi_emi_block_en;
  bcm_reg32_t emi_soft_commands_client;
  bcm_reg32_t emi_transfer_mode;
  bcm_reg32_t emi_pad_cntl;
  /* We do not really use the following */
  bcm_reg32_t emi_cpu_intr_set;
  bcm_reg32_t emi_cpu_intr_clear;
  bcm_reg32_t emi_cpu_intr_status;
  bcm_reg32_t emi_cpu_intr_mask_set;
  bcm_reg32_t emi_cpu_intr_mask_clear;
  bcm_reg32_t emi_cpu_intr_mask_status;
} emi_regs_t;

#endif /* _EMI_REGS_H_ */

