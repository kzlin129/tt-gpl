/* 
 * drivers/media/video/samsung/mfc40/s3c_mfc_common.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 *
 * Jaeryul Oh, Copyright (c) 2009 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _S3C_MFC_COMMON_H_
#define _S3C_MFC_COMMON_H_

#include <plat/regs-mfc.h>

#include "s3c_mfc_interface.h"

#define BUF_L_UNIT (1024)
#define BUF_S_UNIT (32)
#define Align(x, alignbyte) (((x)+(alignbyte)-1)/(alignbyte)*(alignbyte))

typedef enum
{
	MFCINST_STATE_NULL = 0,

	/* Instance is created */
	MFCINST_STATE_OPENED = 10,

	/* channel_set and init_codec is completed */
	MFCINST_STATE_DEC_INITIALIZE = 20,

	MFCINST_STATE_DEC_EXE = 30,
	MFCINST_STATE_DEC_EXE_DONE,
	
	/* Instance is initialized for encoding */
	MFCINST_STATE_ENC_INITIALIZE = 40, 
	MFCINST_STATE_ENC_EXE,
	MFCINST_STATE_ENC_EXE_DONE
} s3c_mfc_inst_state;

typedef enum
{
	MEM_STRUCT_LINEAR = 0,
	MEM_STRUCT_TILE_ENC  = 3  /* 64x32 */
} s3c_mfc_mem_type;

typedef enum
{
	SEQ_HEADER = 1,
	FRAME = 2,
	LAST_FRAME = 3
} s3c_mfc_dec_type;

typedef enum
{
	H2R_CMD_EMPTY = 0,
	H2R_CMD_OPEN_INSTANCE = 1,
	H2R_CMD_CLOSE_INSTANCE = 2,
	H2R_CMD_SYS_INIT = 3
} s3c_mfc_facade_cmd;


typedef enum
{
	R2H_CMD_EMPTY = 0,
	R2H_CMD_OPEN_INSTANCE_RET = 1,	
	R2H_CMD_CLOSE_INSTANCE_RET = 2,
	R2H_CMD_ERROR_RET = 3,
	R2H_CMD_SEQ_DONE_RET = 4,
	R2H_CMD_FRAME_DONE_RET = 5,
	R2H_CMD_SYS_INIT_RET = 8,	
	R2H_CMD_FW_STATUS_RET = 9,
	R2H_CMD_EDFU_INIT_RET = 16,	
	R2H_CMD_DECODE_ERR_RET = 32	
} s3c_mfc_wait_done_type;


typedef enum
{
	DECODING_ONLY = 0,
	DECODING_DISPLAY = 1,
	DISPLAY_ONLY = 2
} s3c_mfc_display_status;

// In case of MPEG4/H.263
typedef enum
{
	MFC_RET_FRAME_NOT_SET = -1,
	MFC_RET_FRAME_I_FRAME = 0,
	MFC_RET_FRAME_P_FRAME = 1,
	MFC_RET_FRAME_B_FRAME = 2	
} s3c_mfc_frame_type;

typedef enum
{
	PORTA = 0,
	PORTB = 1	
} s3c_mfc_port_type;

typedef struct tag_mfc_inst_ctx
{
	unsigned int MfcSfr[S3C_FIMV_REG_COUNT];

	int InstNo;
	unsigned int DPBCnt;
	unsigned int totalDPBCnt;
	unsigned int extraDPB;
	unsigned int displayDelay;
	unsigned int postEnable;
	unsigned int endOfFrame;
	unsigned int forceSetFrameType;
	unsigned int img_width;
	unsigned int img_height;
	unsigned int dwAccess;  // for Power Management.
	unsigned int IsPackedPB;
	int port_no;	
	s3c_mfc_frame_type FrameType;
	MFC_CODEC_TYPE MfcCodecType;
	s3c_mfc_inst_state MfcState;
} s3c_mfc_inst_ctx;

int s3c_mfc_get_frame_buf_size(s3c_mfc_inst_ctx  *mfc_ctx, s3c_mfc_args *args);
//unsigned int s3c_mfc_get_codec_type(MFC_CODEC_TYPE    codec_type);
//int s3c_mfc_get_fw_buf_offset(MFC_CODEC_TYPE codecType);
//int s3c_mfc_get_fw_buf_size(MFC_CODEC_TYPE codecType);
int s3c_mfc_wait_for_done(s3c_mfc_wait_done_type command);
//void s3c_mfc_init_inst_no(void);
void s3c_mfc_return_inst_no(int inst_no);
int s3c_mfc_set_state(s3c_mfc_inst_ctx *ctx, s3c_mfc_inst_state state);

#endif /* _S3C_MFC_COMMON_H_ */

