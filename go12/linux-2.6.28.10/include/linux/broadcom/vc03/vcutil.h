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




/* Endian-reverse a long. */

uint32_t vc_swap_endian_32(uint32_t value);

/* Endian-reverse a short. */

uint16_t vc_swap_endian_16(uint16_t value);

/* Endian-reverse a block of 8 shorts, copying it in the process. */

void vc_swap_endian_block(uint16_t *dest, uint16_t *src);
