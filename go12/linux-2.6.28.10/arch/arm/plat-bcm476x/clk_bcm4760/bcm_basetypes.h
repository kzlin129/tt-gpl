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
 *   @file   bcm_basetypes.h
 * 
 *   @brief  This file contains standard base types to be used within the
 *           broadcom MOBM source domain.  Defines in this header should 
 *           be application neutral.
 * 
 ****************************************************************************/

#ifndef __BCM_BASETYPES_H__
#define __BCM_BASETYPES_H__

/*
 * config flags
 */
typedef enum  _bcm_cfg_stat_t
{
    disable = 0,
    enable  = 1,
    retain  = 2,   
} bcm_cfg_stat_t;

#endif /* __BCM_BASETYPES_H__ */
