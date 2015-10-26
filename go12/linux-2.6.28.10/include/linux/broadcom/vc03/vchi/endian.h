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




/*=============================================================================

Project  :  VCHI
Module   :  Endian-aware routines to read/write data

FILE DESCRIPTION

=============================================================================*/

#ifndef _VCHI_ENDIAN_H_
#define _VCHI_ENDIAN_H_

int16_t  vchi_readbuf_int16 ( const void *ptr );
uint16_t vchi_readbuf_uint16( const void *ptr );
uint32_t vchi_readbuf_uint32( const void *ptr );
fourcc_t vchi_readbuf_fourcc( const void *ptr );

void vchi_writebuf_uint16( void *ptr, uint16_t value );
void vchi_writebuf_uint32( void *ptr, uint32_t value );
void vchi_writebuf_fourcc( void *ptr, fourcc_t value );

#endif /* _VCHI_ENDIAN_H_ */

/********************************** End of file ******************************************/
