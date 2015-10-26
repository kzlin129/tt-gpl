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

#ifndef _TAHOE_OPMODE_CMD_STATS_H_
#define _TAHOE_OPMODE_CMD_STATS_H_

#define TAHOE_OPMODE_VERSION_STR "1.0"

int tahoe_opmode_stats(char *buf, int buf_size);

#endif  //_TAHOE_OPMODE_CMD_STATS_H_

