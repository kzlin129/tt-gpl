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
 *   @file   emi.h
 * 
 *   @brief  EMI apis.
 * 
 ****************************************************************************/

#ifndef _EMI_IFC_H_
#define _EMI_IFC_H_
typedef enum _emi_sdram_mode
{
	EEMI_MODE_1by1,
	EEMI_MODE_2by1,
	EEMI_MODE_3by2,
	EEMI_MODE_ASYNC,
} emi_dram_mode_t;

void emi_api_init(void);
void emi_set_ncdl(uint32_t read_ncdl, uint32_t write_ncdl);
void emi_dram_init(uint32_t mtx_frequency, emi_dram_mode_t emi_dram_mode);
void emi_get_ncdl(uint32_t * read_ncdl, uint32_t * write_ncdl);
void emi_change_frequency(uint32_t emi_frequency);
void emi_setup_dll_phase_load_value(uint32_t emi_frequency);
void emi_wait_for_dll_phase_lock(uint32_t mtx_frequency);
void emi_self_refresh_power_down_set(void);
void emi_self_refresh_power_down_unset(void) ;


void emi_set_dram_mode(emi_dram_mode_t emi_dram_mode);

#endif /* #ifndef _EMI_IFC_H_ */

