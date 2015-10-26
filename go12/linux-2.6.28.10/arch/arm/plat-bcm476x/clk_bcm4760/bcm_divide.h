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
 *   @file   bcm_divide.h
 * 
 *   @brief  Low level math functions.
 * 
 ****************************************************************************/


#ifndef _BCM_DIVIDE_H_
#define _BCM_DIVIDE_H_

/* To get the standard C-runtime library implentation, turn on
 * The following #define
 */

/*
#define BCM_USE_CRT_DIVISION
*/

#ifndef BCM_USE_CRT_DIVISION
/* the following functions are defined in bcm_divide.c */
uint32_t bcm_udivide32 ( uint32_t  dividend, uint32_t divisor);
uint32_t bcm_umodulo32 ( uint32_t  dividend, uint32_t divisor);
uint64_t bcm_udivide64 ( uint64_t  dividend, uint64_t divisor);
uint64_t bcm_umodulo64 ( uint64_t  dividend, uint64_t divisor);
#else
/* in case we want the standard C-library implementation... */
BCM_INLINE uint32_t bcm_udivide32 ( uint32_t  dividend, uint32_t divisor)
{
	return dividend/divisor;
}

BCM_INLINE uint32_t bcm_umodulo32 ( uint32_t  dividend, uint32_t divisor)
{
	return dividend % divisor;
}

BCM_INLINE uint64_t bcm_udivide64 ( uint64_t  dividend, uint64_t divisor)
{
	return dividend/divisor;
}

BCM_INLINE uint64_t bcm_umodulo64 ( uint64_t  dividend, uint64_t divisor)
{
	return dividend % divisor;
}
#endif /* #ifndef BCM_USE_CRT_DIVISION */


/* instead of truncating, "round" to nearest integer. 32-bit */
BCM_INLINE uint32_t bcm_udivide32_round(uint32_t dividend, uint32_t divisor)
{
	return bcm_udivide32((dividend + (divisor >> 1)), divisor);
}

/* instead of truncating, "round" to nearest integer. 64-bit */
BCM_INLINE uint64_t bcm_udivide64_round(uint64_t dividend, uint64_t divisor)
{
	return bcm_udivide64((dividend + (divisor >> 1)), divisor);
}

/**
   Given two values "desired" and "actual", this function will return, in percentage
   how far the "actual" value differs or deviates from the "desired" value.
   Always outputs the deviation as a positive integer
 */
uint32_t bcm_get_deviation_percent(uint32_t desired, uint32_t actual);

#endif /* #ifndef _BCM_DIVIDE_H_ */
