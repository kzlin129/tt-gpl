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
*  @file    vcfrmfwd.h
*
*  @brief   Contains function prototypes for public functions in vcfrmfwd.c
*
****************************************************************************/

#ifndef VCFRMFWD_H
#define VCFRMFWD_H

#include "vc_frmfwd_defs.h"

typedef struct
{
   // for getting a pointer where frame data can be copied to
   FRMFWD_FRAME_T * (* get_frame_buf)(int stream_num, int length_in_bytes);
   // for dispatching the frame buffer after data has been copied to it
   void (* frame_rcv)(FRMFWD_FRAME_T *frame, unsigned int lengthinbyte, int stream_num);
   // for dispatching frame info (NOTE: info is not buffered)
   void (* dec_ack_rcv)(FRMFWD_FRAME_INFO_T *info, int stream_num);
   // for notifying that VC02 is out of buffers and a frame was lost
   void (* dec_out_of_buffers_rcv)(FRMFWD_FRAME_INFO_T *info, int stream_num);
   // for handling audio packets from the VC02
   FRMFWD_FRAME_T * (* get_audio_buf)(int stream_num, int length_in_bytes);
   void (*audio_rcv)(FRMFWD_FRAME_T *frame, unsigned int lengthinbyte, int stream_num);
} VC_FFCALLBACK_T;

typedef struct
{
   int   checksumEnabled;
   int   framesSentWithChecksum;
   int   framesSentNoChecksum;
   int   framesRcvdNoChecksum;
   int   framesRcvdChecksumGood;
   int   framesRcvdChecksumBad;

} VC_FRMFWD_STATS_T;

extern VC_FRMFWD_STATS_T g_vc_frmfwd_stats;

// Set user callback functions
extern int vc_frmfwd_set_cmd_callback(VC_FFCALLBACK_T *callback);

// Send a video frame to the VC02 for decoding
extern int vc_frmfwd_send_frame(FRMFWD_FRAME_T *frame, int blocking);

#endif
