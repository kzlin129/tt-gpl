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
 *   @file   bcm_privtypes.h
 * 
 *   @brief  Basic types
 * 
 ****************************************************************************/

#ifndef __BCM_PRIVTYPES_H__
#define __BCM_PRIVTYPES_H__

/*
 *  Our boolean type...
 */
typedef enum 
{
   FALSE = 0,
   TRUE  = 1
} bool_t;

/**
 *  Our return values...
 */
typedef enum 
{
   BCM_OK = 0,
   BCM_ERROR,
} stat_t;




/** 
    Reg struct - currently used in CLDB
  */
//typedef struct reg_t
//{
	//volatile uint_ptr_t   	   addr;
	//uint32_t		            val;
//} reg_t;


typedef unsigned long bcm_dev_phyaddr_t;

typedef void * bcm_dev_virtaddr_t;

#endif /* __BCM_BASETYPES_H__ */
