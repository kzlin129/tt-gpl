/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    vc_frmfwd_defs.h
*
*  @brief   frame forward service enumeration & misc types & defines
*
****************************************************************************/
#ifndef BCM_VC_FRMFWD_DEFS_H
#define BCM_VC_FRMFWD_DEFS_H
#include "vc_frmfwd_ext_defs.h"

// bit definitions for flags

#define FRMFWD_KEY_FRAME           0x01   // indicates an I-frame
#define FRMFWD_KEY_FRAME_BITSHIFT  0     // indicates an I-frame
#define FRMFWD_MARKER              0x02   // marks end of the a video frame
#define FRMFWD_MARKER_BITSHIFT     1   // marks end of the a video frame
#define FRMFWD_END_OF_STREAM       0x04   // for host to mark the end of stream
#define FRMFWD_END_OF_STREAM_BITSHIFT  2   // for host to mark the end of stream
#define FRMFWD_HAS_CHECKSUM            0x08  // indicates presence of checksum
#define FRMFWD_HAS_CHECKSUM_BITSHIFT   3     // indicates presence of checksum

#define GET_FRMFWD_KEY_FRAME(x)           (((x & 0xFF) & FRMFWD_KEY_FRAME) >> FRMFWD_KEY_FRAME_BITSHIFT)
#define SET_FRMFWD_KEY_FRAME(x,y)         ((x & 0xFF) | ((y & 0x1) << FRMFWD_KEY_FRAME_BITSHIFT))
#define GET_FRMFWD_MARKER(x)              (((x & 0xFF) & FRMFWD_MARKER) >> FRMFWD_MARKER_BITSHIFT)
#define SET_FRMFWD_MARKER(x,y)            ((x & 0xFF) | ((y & 0x1) << FRMFWD_MARKER_BITSHIFT))
#define GET_FRMFWD_END_OF_STREAM(x)       (((x & 0xFF) & FRMFWD_END_OF_STREAM) >> FRMFWD_END_OF_STREAM_BITSHIFT)
#define SET_FRMFWD_END_OF_STREAM(x,y)     ((x & 0xFF) | ((y & 0x1) << FRMFWD_END_OF_STREAM_BITSHIFT))
#define GET_FRMFWD_HAS_CHECKSUM(x)        (((x & 0xFF) & FRMFWD_HAS_CHECKSUM) >> FRMFWD_HAS_CHECKSUM_BITSHIFT)
#define SET_FRMFWD_HAS_CHECKSUM(x,y)      ((x & 0xFF) | ((y & 0x1) << FRMFWD_HAS_CHECKSUM_BITSHIFT))

// Types and constants common across the both ends of the interface

typedef struct {
   uint8_t stream_num;
   uint8_t flags;       // bit 2: end_of_stream, bit 1: marker, bit 0: key_frame
   uint8_t offset;      // byte offset from end of INFO structure to start of payload
   uint8_t reserved1;    
   uint16_t checksum;   // 8-bit checksum (simple sum over all data only, not CRC)
   uint16_t seq_num;
   uint32_t timestamp;
   uint32_t data_len;  // amount of video data

} FRMFWD_FRAME_INFO_T;  // size should be multiple of 16 bytes

typedef struct {
   FRMFWD_FRAME_INFO_T info;
   uint8_t  data[4];    // pointer to payload.  NOTE: data buffer should be multiple of 4 bytes.
                        // The payload starts at a byte offset from end of INFO structure.

} FRMFWD_FRAME_T;

#endif // BCM_VC_FRMFWD_DEFS_H
