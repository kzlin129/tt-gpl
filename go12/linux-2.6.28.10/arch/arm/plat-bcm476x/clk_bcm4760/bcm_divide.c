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
 *   @file   bcm_divide.c 
 * 
 *   @brief  Low level math functions.
 * 
 ****************************************************************************/

#include "bcm_basedefs.h"
#include "bcm_basetypes.h"
#include "bcm_divide.h"

#ifndef BCM_USE_CRT_DIVISION
// divide.c
// has simple 32-bit and 64-bit division routines . This is to avoid
// having the compiler throw exceptions when we do not need it to.

/* Simple 32-bit division helper function */
/* returns the quotient */
static uint32_t udivide32_helper ( uint32_t  dividend, uint32_t divisor, uint32_t * remainder_output)
{
	register unsigned int quotient 	= 0;
	register unsigned int tmp 		= divisor;
	register unsigned int remainder	= 0;
  
	if ( divisor && (dividend >= tmp)) {

		quotient = 1;
    
		while ( (dividend >> 1 ) >= tmp) {

			quotient <<= 1;
			tmp <<=1;
		}

		if ( dividend > tmp ) {
			register unsigned int q2 = quotient;
      
			remainder = dividend - tmp;

			do {
        
				tmp >>= 1;
				q2  >>= 1;
        
				if ( remainder >= tmp ) {
					quotient	+= q2;
					remainder 	-= tmp;
				}

			} while ( (tmp > divisor));
    
		}
	}
	*remainder_output = remainder;
	return (quotient);
}

/* helper function for 64-bit division */
/* TODO: For now, this is a copy of the corresponding 32-bit  function.
 * Needs to be optimized, at some later date.
 */
static uint64_t udivide64_helper ( uint64_t  dividend, uint64_t divisor, uint64_t * remainder)
{
	unsigned long long quotient 	= 0;
	unsigned long long tmp 		= divisor;

	*remainder	= 0;
  
	if ( divisor && (dividend >= tmp)) {

		quotient = 1;
    
		while ( (dividend >> 1 ) >= tmp) {

			quotient <<= 1;
			tmp <<=1;
		}

		if ( dividend > tmp ) {
			unsigned long long q2 = quotient;
      
			*remainder = dividend - tmp;

			do {
        
				tmp >>= 1;
				q2  >>= 1;
        
				if ( *remainder >= tmp ) {
					quotient	+= q2;
					*remainder 	-= tmp;
				}

			} while ( (tmp > divisor));
    
		}
	}
  
	return (quotient);
}


uint32_t bcm_udivide32 ( uint32_t  dividend, uint32_t divisor)
{
	uint32_t	modulo;
	return udivide32_helper(dividend, divisor, &modulo);
}

/* public function */
uint32_t bcm_umodulo32 ( uint32_t  dividend, uint32_t divisor)
{
	uint32_t	modulo;
	udivide32_helper(dividend, divisor, &modulo);
	return modulo;
}

uint64_t bcm_udivide64 ( uint64_t  dividend, uint64_t divisor)
{
	uint64_t	modulo;
	return udivide64_helper(dividend, divisor, &modulo);
}

uint64_t bcm_umodulo64 ( uint64_t  dividend, uint64_t divisor)
{
	uint64_t	modulo;
	udivide64_helper(dividend, divisor, &modulo);
	return modulo;
}
#endif /* #ifndef BCM_USE_CRT_DIVISION */

uint32_t bcm_get_deviation_percent(uint32_t desired, uint32_t actual)
{
	uint32_t	deviation;

	if( actual > desired )
	{
		deviation = actual - desired;
	}
	else
	{
		deviation = desired - actual;
	}

	/* take care of possible overflow */
	return bcm_udivide64_round( (uint64_t) deviation*100LL, desired);
}


