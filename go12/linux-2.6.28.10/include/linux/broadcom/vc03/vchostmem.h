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



#ifndef __VCHOSTMEM
#define __VCHOSTMEM
extern void vchostreq_meminit( void );
extern VC_RESP_CODE_T vchostreq_writemem( void* host_addr, void *vc_addr, int len, int channel );
extern VC_RESP_CODE_T vchostreq_readmem( void* host_addr, void *vc_addr, int len );
extern void*  vchostreq_malloc( size_t bytes);
extern void  vchostreq_free( void *host_addr );
extern VC_RESP_CODE_T  vchostreq_memmove( void  *dest_addr, void *src_addr, int len );
extern VC_RESP_CODE_T vchostreq_readmem_3d(uint32_t dest,
                                           unsigned int tile_width, unsigned int tile_height, unsigned int num_tiles,
                                           unsigned int tile_d_pitch, unsigned int d_pitch,
                                           void *src, int tile_s_pitch, int s_pitch,
                                           uint32_t frac_offset, uint32_t frac_int, int channel);
#endif
