/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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




#include "vchost.h"
#include "vcutil.h"

/******************************************************************************
NAME
   vc_swap_endian_32

SYNOPSIS
   uint32_t vc_swap_endian_long(uint32_t val)

FUNCTION
   Host to VideoCore (little-endian) long.

RETURNS
   uint32_t
******************************************************************************/

uint32_t vc_swap_endian_32 (uint32_t val) {
   uint32_t ret;
   char *val_ptr = (char *)&val;
   char *ret_ptr = (char *)&ret;
   ret_ptr[0] = val_ptr[3];
   ret_ptr[1] = val_ptr[2];
   ret_ptr[2] = val_ptr[1];
   ret_ptr[3] = val_ptr[0];
   return ret;
}

/******************************************************************************
NAME
   vc_swap_endian_short

SYNOPSIS
   uint16_t vc_swap_endian_short(uint16_t val)

FUNCTION
   Host to VideoCore (little-endian) short.

RETURNS
   uint16_t
******************************************************************************/

uint16_t vc_swap_endian_16 (uint16_t val) {
   return (val>>8) + ((val&0xff)<<8);
}

/******************************************************************************
NAME
   vc_swap_endian_block

SYNOPSIS
   void vc_swap_endian_block(uint16_t *dest, uint16_t *src)

FUNCTION
   Copy a block of 8 shorts, swapping their endian-nes in the process.

RETURNS
   -
******************************************************************************/

void vc_swap_endian_block (uint16_t *dest, uint16_t *src) {
   unsigned char *d = (unsigned char *)dest, *s = (unsigned char *)src;
   *(d+0) = *(s+1);
   *(d+1) = *(s+0);
   *(d+2) = *(s+3);
   *(d+3) = *(s+2);
   *(d+4) = *(s+5);
   *(d+5) = *(s+4);
   *(d+6) = *(s+7);
   *(d+7) = *(s+6);
   *(d+8) = *(s+9);
   *(d+9) = *(s+8);
   *(d+10) = *(s+11);
   *(d+11) = *(s+10);
   *(d+12) = *(s+13);
   *(d+13) = *(s+12);
   *(d+14) = *(s+15);
   *(d+15) = *(s+14);
}
