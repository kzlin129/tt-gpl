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
 *   @file   tahoe_opmode_cmd_params.h
 * 
 *   @brief  proc interfacing helper functions.
 * 
 ****************************************************************************/

#ifndef _TAHOE_OPMODE_CMD_PARAM_H_
#define _TAHOE_OPMODE_CMD_PARAM_H_

#include "tahoe_opmode.h"

#define PLL_COUNT 4
#define MATRIX_STR  "MATRIX"

//CORES
#define MAX_STR_SZ 50
typedef struct _tahoe_core_strs_t
{
    tahoe_opmode_core_t core;
    char str[MAX_STR_SZ];
}tahoe_core_strs_t;

//user-input string to enum conversion functions
int                            tahoe_opmode_get_cfg(char *cfg);
tahoe_opmode_core_t            tahoe_opmode_get_core(char *core_str);
tahoe_opmode_csrc_t            tahoe_opmode_get_csrc(char *csrc_str);
int                            tahoe_opmode_get_pll(char *pll_str);
tahoe_opmode_mode_t            tahoe_opmode_get_opmode(char *opmode_str);
unsigned long                  tahoe_opmode_get_log(char *log_str);
tahoe_opmode_dram_mode_t       tahoe_opmode_get_drammode(char *dram_mode_str);
tahoe_opmode_synctop_t         tahoe_opmode_get_synctop_idx(char *synctop_str);
tahoe_opmode_synctop_mode_t    tahoe_opmode_get_synctop_mode(char *synctop_mode_str);
tahoe_opmode_synctop_clkmode_t tahoe_opmode_get_synctop_clkmode(char *synctop_clkmode_str);

//display strings
const char *tahoe_opmode_log_str(unsigned long level);
const char *tahoe_opmode_stat_str(int stat);
const char *tahoe_opmode_mode_str(int mode);
const char *tahoe_opmode_drammode_str(int dram_mode);
const char *tahoe_opmode_core_str(int core_idx);
const char *tahoe_opmode_pll_str(int pll);
const char *tahoe_opmode_csrc_str(int csrc);
const char *tahoe_opmode_synctop_str(int synctop);
const char *tahoe_opmode_synctop_mode_str(int synctop_mode);
const char *tahoe_opmode_synctop_clkmode_str(int synctop_clkmode);

#endif //_TAHOE_OPMODE_CMD_PARAM_H_
