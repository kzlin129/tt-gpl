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
*  @file    vc_frmfwd_ext_defs.h
*
*  @brief   video data buffer size definitions
*
****************************************************************************/
#ifndef VC_FRMFWD_EXT_DEFS_H
#define VC_FRMFWD_EXT_DEFS_H

#ifndef MAX
#define MAX(x,y) (x>y?x:y)
#endif
#ifndef MAX3
#define MAX3(x,y,z) MAX(MAX(x,y),z)
#endif

// encoded data size definitions

#define MAX_H263_ENCODED_DATA_SIZE   14400        // needs to be multiple of 16
#define MAX_MPEG4_ENCODED_DATA_SIZE  20480
#define MAX_H264_ENCODED_DATA_SIZE   33000
//#define VC_FFMAX_CMD_DATA  MAX3(MAX_H263_ENCODED_DATA_SIZE,MAX_MPEG4_ENCODED_DATA_SIZE,MAX_H264_ENCODED_DATA_SIZE)
#define VC_FFMAX_CMD_DATA  64000 // max size = 64k bytes

#endif
