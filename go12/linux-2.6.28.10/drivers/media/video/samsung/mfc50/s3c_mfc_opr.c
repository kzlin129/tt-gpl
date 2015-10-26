/*
 * drivers/media/video/samsung/mfc40/s3c_mfc_opr.c
 *
 * C file for Samsung MFC (Multi Function Codec - FIMV) driver
 *
 * Jaeryul Oh, Copyright (c) 2009 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <plat/regs-mfc.h>

#include "s3c_mfc_common.h"
#include "s3c_mfc_opr.h"
#include "s3c_mfc_logmsg.h"
#include "s3c_mfc_memory.h"
#include "s3c_mfc_fw.h"
#include "s3c_mfc_buffer_manager.h"
#include "s3c_mfc_interface.h"

extern void __iomem *s3c_mfc_sfr_virt_base;
//extern dma_addr_t s3c_mfc_phys_data_buf;
//extern unsigned char *s3c_mfc_virt_data_buf;

#define READL(offset)		readl(s3c_mfc_sfr_virt_base + (offset))
#define WRITEL(data, offset)	writel((data), s3c_mfc_sfr_virt_base + (offset))

static void s3c_mfc_cmd_reset(void);
static void s3c_mfc_cmd_frame_start(void);
static void s3c_mfc_cmd_sleep(void);
static void s3c_mfc_cmd_wakeup(void);
static void s3c_mfc_backup_context(s3c_mfc_inst_ctx  *mfc_ctx);
static void s3c_mfc_restore_context(s3c_mfc_inst_ctx  *mfc_ctx);
static void s3c_mfc_set_codec_firmware(s3c_mfc_inst_ctx  *mfc_ctx);
static void s3c_mfc_set_encode_init_param(int inst_no, MFC_CODEC_TYPE mfc_codec_type, s3c_mfc_args *args);
static int s3c_mfc_get_inst_no(MFC_CODEC_TYPE codec_type);
static MFC_ERROR_CODE s3c_mfc_set_dec_stream_buffer(int inst_no, int buf_addr, unsigned int buf_size);
static MFC_ERROR_CODE s3c_mfc_set_dec_frame_buffer(s3c_mfc_inst_ctx  *mfc_ctx, s3c_mfc_frame_buf_arg_t buf_addr, s3c_mfc_frame_buf_arg_t buf_size);
static MFC_ERROR_CODE s3c_mfc_set_risc_buffer(MFC_CODEC_TYPE codec_type, int inst_no);
static MFC_ERROR_CODE s3c_mfc_decode_one_frame(s3c_mfc_inst_ctx *mfc_ctx, s3c_mfc_dec_exe_arg_t *dec_arg, unsigned int *consumed_strm_size);


static void s3c_mfc_cmd_reset(void)
{
	WRITEL(0, S3C_FIMV_SW_RESET);
	mdelay(10);
	WRITEL(0x3fe, S3C_FIMV_SW_RESET);	
}

static void s3c_mfc_cmd_host2risc(s3c_mfc_facade_cmd cmd, int arg)
{
	s3c_mfc_facade_cmd cur_cmd;

	/* wait until host to risc command register becomes 'H2R_CMD_EMPTY' */
	do {	
		cur_cmd = READL(S3C_FIMV_HOST2RISC_CMD);		
	} while(cur_cmd != H2R_CMD_EMPTY);

	WRITEL(arg, S3C_FIMV_HOST2RISC_ARG1);
	WRITEL(cmd, S3C_FIMV_HOST2RISC_CMD);
	
}

/*
static void s3c_mfc_cmd_frame_start(void)
{
	WRITEL(1, S3C_FIMV_FRAME_START);
}

static void s3c_mfc_cmd_sleep()
{
	WRITEL(-1, S3C_FIMV_CH_ID);
	WRITEL(MFC_SLEEP, S3C_FIMV_COMMAND_TYPE);
}

static void s3c_mfc_cmd_wakeup()
{
	WRITEL(-1, S3C_FIMV_CH_ID);
	WRITEL(MFC_WAKEUP, S3C_FIMV_COMMAND_TYPE);
	mdelay(100);
}
*/

static void s3c_mfc_backup_context(s3c_mfc_inst_ctx  *mfc_ctx)
{
	memcpy(mfc_ctx->MfcSfr, s3c_mfc_sfr_virt_base, S3C_FIMV_REG_SIZE);
}

static void s3c_mfc_restore_context(s3c_mfc_inst_ctx  *mfc_ctx)
{
	/*
	memcpy(s3c_mfc_sfr_virt_base, mfc_ctx->MfcSfr, S3C_FIMV_REG_SIZE);
	*/
}

static MFC_ERROR_CODE s3c_mfc_set_dec_stream_buffer(int inst_no, int buf_addr, unsigned int buf_size)
{
	unsigned int fw_phybuf;	
	unsigned int risc_phy_buf, aligned_risc_phy_buf;

	mfc_debug("inst_no : %d, buf_addr : 0x%08x, buf_size : %d\n", inst_no, buf_addr, buf_size);
	
	fw_phybuf = Align(s3c_mfc_get_fw_buf_phys_addr(), 128*BUF_L_UNIT);
	
	risc_phy_buf = s3c_mfc_get_risc_buf_phys_addr(inst_no);
	aligned_risc_phy_buf = Align(risc_phy_buf, 2*BUF_L_UNIT);
	

	WRITEL((buf_addr & 0xfffffff8)-fw_phybuf, S3C_FIMV_SI_CH1_SB_ST_ADR);
	WRITEL(buf_size, S3C_FIMV_SI_CH1_SB_FRM_SIZE);	// peter, buf_size is '0' when last frame
	WRITEL(aligned_risc_phy_buf-fw_phybuf, S3C_FIMV_SI_CH1_DESC_ADR);
	WRITEL(CPB_BUF_SIZE, S3C_FIMV_SI_CH1_CPB_SIZE);
	WRITEL(DESC_BUF_SIZE, S3C_FIMV_SI_CH1_DESC_SIZE);	

	return MFCINST_RET_OK;
}


static MFC_ERROR_CODE s3c_mfc_set_dec_frame_buffer(s3c_mfc_inst_ctx  *mfc_ctx, s3c_mfc_frame_buf_arg_t buf_addr, 
							s3c_mfc_frame_buf_arg_t buf_size)
{
	unsigned int width, height, frame_size, dec_dpb_addr;
	unsigned int fw_phybuf;
	s3c_mfc_frame_buf_arg_t dpb_buf_addr;
	unsigned int aligned_width, aligned_height;

	mfc_debug("luma_buf_addr : 0x%08x  luma_buf_size : %d\n", buf_addr.luma, buf_size.luma);
	mfc_debug("chroma_buf_addr : 0x%08x  chroma_buf_size : %d\n", buf_addr.chroma, buf_size.chroma);

	fw_phybuf = Align(s3c_mfc_get_fw_buf_phys_addr(), 128*BUF_L_UNIT);

	width = Align(mfc_ctx->img_width, BUF_S_UNIT/2);
	height = Align(mfc_ctx->img_height, BUF_S_UNIT);
	frame_size = (width*height*3)>>1;

	mfc_debug("width : %d height : %d frame_size : %d mfc_ctx->DPBCnt :%d\n", \
					width, height, frame_size, mfc_ctx->DPBCnt);
	
	if(buf_size < frame_size*mfc_ctx->totalDPBCnt){	// peter, it should be corrected 
		mfc_err("MFCINST_ERR_FRM_BUF_SIZE\n");
		return MFCINST_ERR_FRM_BUF_SIZE;
	}

	aligned_width = Align(mfc_ctx->img_width, 4*BUF_S_UNIT); // 128B align 
	aligned_height = Align(mfc_ctx->img_height, BUF_S_UNIT); // 32B align

	dpb_buf_addr.luma = Align(buf_addr.luma, 2*BUF_S_UNIT);
	dpb_buf_addr.chroma = Align(buf_addr.chroma, 2*BUF_S_UNIT);
	
	mfc_debug("DEC_LUMA_DPB_START_ADR : 0x%08x\n", dpb_buf_addr.luma);
	mfc_debug("DEC_CHROMA_DPB_START_ADR : 0x%08x\n", dpb_buf_addr.chroma);

	if (mfc_ctx->MfcCodecType == H264_DEC) {		
		
		for (i=0; i < mfc_ctx->totalDPBCnt; i++) {		
			/* set Luma address */
			WRITEL((dpb_buf_addr.luma-fw_phybuf)>>11, S3C_FIMV_H264_LUMA_ADR+(4*i));	
			dpb_buf_addr.luma += Align(aligned_width*aligned_height, 64*BUF_L_UNIT)
			/* set Chroma address */	
			WRITEL((dpb_buf_addr.chroma-fw_phybuf)>>11, S3C_FIMV_H264_CHROMA_ADR+(4*i));	
			dpb_buf_addr.chroma += Align(aligned_width*aligned_height/2, 64*BUF_L_UNIT);
			/* set MV address */	
			WRITEL((dpb_buf_addr.chroma-fw_phybuf)>>11, S3C_FIMV_MV_ADR+(4*i));	
			dpb_buf_addr.chroma += Align(aligned_width*aligned_height/4, 64*BUF_L_UNIT);			
		}	

	} else {
	
		for (i=0; i < mfc_ctx->totalDPBCnt; i++) {		
			/* set Luma address */
			WRITEL((dpb_buf_addr.luma-fw_phybuf)>>11, S3C_FIMV_LUMA_ADR+(4*i));	
			dpb_buf_addr.luma += Align(aligned_width*aligned_height, 64*BUF_L_UNIT)
			/* set Chroma address */	
			WRITEL((dpb_buf_addr.chroma-fw_phybuf)>>11, S3C_FIMV_CHROMA_ADR+(4*i));	
			dpb_buf_addr.chroma += Align(aligned_width*aligned_height/2, 64*BUF_L_UNIT);			
		}	

	}

	mfc_debug("DEC_LUMA_DPB_END_ADR : 0x%08x\n", dpb_buf_addr.luma);
	mfc_debug("DEC_CHROMA_DPB_END_ADR : 0x%08x\n", dpb_buf_addr.chroma);


	return MFCINST_RET_OK;
}

// peter, set the desc, motion vector, overlap, bitplane0/1/2, etc
static MFC_ERROR_CODE s3c_mfc_set_risc_buffer(MFC_CODEC_TYPE codec_type, int inst_no)
{
	unsigned int fw_phybuf;	
	unsigned int risc_phy_buf, aligned_risc_phy_buf;
	
	fw_phybuf = Align(s3c_mfc_get_fw_buf_phys_addr(), 128*BUF_L_UNIT);
	
	risc_phy_buf = s3c_mfc_get_risc_buf_phys_addr(inst_no) + DESC_BUF_SIZE;
	aligned_risc_phy_buf = Align(risc_phy_buf, 2*BUF_L_UNIT);

	mfc_debug("inst_no : %d, risc_buf_start : 0x%08x\n", \
			inst_no, aligned_risc_phy_buf);


	switch (codec_type) {
	case H264_DEC:		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_VERT_NB_MV_ADR);
		aligned_risc_phy_buf += 16*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_VERT_NB_IP_ADR);	
		aligned_risc_phy_buf += 32*BUF_L_UNIT;
		break;
		
	case MPEG4_DEC:
	case DIVX_DEC:
	case XVID_DEC:	
	case H263_DEC:	
	case MP4SH_DEC:
	case DIVX311_DEC:	
	case DIVX412_DEC:	
	case DIVX502_DEC:	
	case DIVX503_DEC:		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_NB_DCAC_ADR);		
		aligned_risc_phy_buf += 16*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_UP_NB_MV_ADR);	
		aligned_risc_phy_buf += 68*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_SA_MV_ADR);	
		aligned_risc_phy_buf += 136*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_OT_LINE_ADR);
		aligned_risc_phy_buf += 32*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_SP_ADR);	
		aligned_risc_phy_buf += 68*BUF_L_UNIT;		
		break;
		
	case VC1AP_DEC:	
	case VC1RCV_DEC:		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_UP_NB_MV_ADR);	
		aligned_risc_phy_buf += 68*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_SA_MV_ADR);	
		aligned_risc_phy_buf += 136*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_OT_LINE_ADR);
		aligned_risc_phy_buf += 32*BUF_L_UNIT;	
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_BITPLANE3_ADR);	
		aligned_risc_phy_buf += 2*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_BITPLANE2_ADR);	
		aligned_risc_phy_buf += 2*BUF_L_UNIT;		
		WRITEL((aligned_risc_phy_buf-fw_phybuf)>>11, S3C_FIMV_BITPLANE1_ADR);
		aligned_risc_phy_buf += 2*BUF_L_UNIT;		
		break;
		
	case MPEG1_DEC:	
	case MPEG2_DEC:	
		break;				

	default:	/* encoder case */		
		break;		

	}	
	
	mfc_debug("inst_no : %d, risc_buf_end : 0x%08x\n", \
			inst_no, aligned_risc_phy_buf);

	return MFCINST_RET_OK;
}

#if 0
/* This function sets the MFC SFR values according to the input arguments. */
static void s3c_mfc_set_encode_init_param(int inst_no, MFC_CODEC_TYPE mfc_codec_type, s3c_mfc_args *args)
{
	unsigned int		ms_size;

	s3c_mfc_enc_init_mpeg4_arg_t   *EncInitMpeg4Arg;
	s3c_mfc_enc_init_h264_arg_t    *EncInitH264Arg;

	EncInitMpeg4Arg = (s3c_mfc_enc_init_mpeg4_arg_t *) args;
	EncInitH264Arg  = (s3c_mfc_enc_init_h264_arg_t  *) args;

	mfc_debug("mfc_codec_type : %d\n", mfc_codec_type);

	s3c_mfc_set_vsp_buffer(inst_no);

	/* Set the other SFR */
	WRITEL(EncInitMpeg4Arg->in_dpb_addr, S3C_FIMV_ENC_DPB_ADR);
	WRITEL(EncInitMpeg4Arg->in_width, S3C_FIMV_IMG_SIZE_X);
	WRITEL(EncInitMpeg4Arg->in_height, S3C_FIMV_IMG_SIZE_Y);
	WRITEL(EncInitMpeg4Arg->in_profile_level, S3C_FIMV_PROFILE);
	WRITEL(EncInitMpeg4Arg->in_gop_num, S3C_FIMV_IDR_PERIOD);
	WRITEL(EncInitMpeg4Arg->in_gop_num, S3C_FIMV_I_PERIOD);
	WRITEL(EncInitMpeg4Arg->in_vop_quant, S3C_FIMV_FRAME_QP_INIT);
	WRITEL(0, S3C_FIMV_POST_ON);
	WRITEL(EncInitMpeg4Arg->in_mb_refresh, S3C_FIMV_CIR_MB_NUM);

	/* Rate Control options */
	WRITEL((EncInitMpeg4Arg->in_RC_enable << 8) | (EncInitMpeg4Arg->in_vop_quant & 0x3F), S3C_FIMV_RC_CONFIG);

	if (READL(S3C_FIMV_RC_CONFIG) & 0x0300) {
		WRITEL(EncInitMpeg4Arg->in_RC_framerate, S3C_FIMV_RC_FRAME_RATE);
		WRITEL(EncInitMpeg4Arg->in_RC_bitrate, S3C_FIMV_RC_BIT_RATE);
		WRITEL(EncInitMpeg4Arg->in_RC_qbound, S3C_FIMV_RC_QBOUND);
		WRITEL(EncInitMpeg4Arg->in_RC_rpara, S3C_FIMV_RC_RPARA);
		WRITEL(0, S3C_FIMV_RC_MB_CTRL);
	}

	/* Multi-slice options */
	WRITEL(EncInitMpeg4Arg->in_MS_mode, S3C_FIMV_MSLICE_ENA);

	if (EncInitMpeg4Arg->in_MS_mode) {
		WRITEL(EncInitMpeg4Arg->in_MS_size_mode, S3C_FIMV_MSLICE_SEL);
		if (EncInitMpeg4Arg->in_MS_size_mode == 0) {
			WRITEL(EncInitMpeg4Arg->in_MS_size, S3C_FIMV_MSLICE_MB);
			WRITEL(0, S3C_FIMV_MSLICE_BYTE);
		} else {
			ms_size = (mfc_codec_type == H264_ENC) ? EncInitMpeg4Arg->in_MS_size : 0;
			WRITEL(ms_size, S3C_FIMV_MSLICE_MB);
			WRITEL(EncInitMpeg4Arg->in_MS_size, S3C_FIMV_MSLICE_BYTE);
		}
	}

	switch (mfc_codec_type) {
	case MPEG4_ENC:
		/* MPEG4 encoder */
		WRITEL(0, S3C_FIMV_ENTROPY_CON);
		WRITEL(0, S3C_FIMV_DEBLOCK_FILTER_OPTION);
		WRITEL(0, S3C_FIMV_SHORT_HD_ON);
		break;

	case H263_ENC:
		/* H263 encoder */
		WRITEL(0, S3C_FIMV_ENTROPY_CON);
		WRITEL(0, S3C_FIMV_DEBLOCK_FILTER_OPTION);
		WRITEL(1, S3C_FIMV_SHORT_HD_ON);

		break;

	case H264_ENC:
		/* H.264 encoder */
		WRITEL((EncInitH264Arg->in_symbolmode & 0x1) | (EncInitH264Arg->in_model_number << 2), S3C_FIMV_ENTROPY_CON);
		WRITEL((EncInitH264Arg->in_deblock_filt & 0x3)
				| ((EncInitH264Arg->in_deblock_alpha_C0 & 0x1f) << 7)
				| ((EncInitH264Arg->in_deblock_beta     & 0x1f) << 2), S3C_FIMV_DEBLOCK_FILTER_OPTION);
		WRITEL(0, S3C_FIMV_SHORT_HD_ON);
		break;

	default:
		mfc_err("Invalid MFC codec type\n");
	}

}
#endif

BOOL s3c_mfc_load_firmware()
{
	volatile unsigned char *fw_virbuf;

	mfc_debug("s3c_mfc_load_firmware++\n");

	fw_virbuf = Align(s3c_mfc_get_fw_buf_virt_addr(), 128*BUF_ALIGN_L_UNIT);
	memcpy((void *)fw_virbuf, s3c_mfc_fw_code, sizeof(s3c_mfc_fw_code));
	

	mfc_debug("s3c_mfc_load_firmware--\n");
	
	return TRUE;
}

MFC_ERROR_CODE s3c_mfc_init_hw()
{
	unsigned int fw_phybuf;
	unsigned int dpb_luma_phybuf;
	int fw_buf_size;		

	mfc_debug("s3c_mfc_init_hw++\n");	
	
	fw_phybuf = Align(s3c_mfc_get_fw_buf_phys_addr(), 128*BUF_L_UNIT);
	dpb_luma_phybuf = Align(s3c_mfc_get_dpb_luma_buf_phys_addr(), 128*BUF_L_UNIT);
	
	/*
	 * 0. MFC reset
	 */
	s3c_mfc_cmd_reset();

	/*
	 * 1. Set DRAM base Addr
	 */
	WRITEL(dpb_luma_phybuf, S3C_FIMV_MC_DRAMBASE_ADR_A); 	// channelA 
	WRITEL(fw_phybuf, S3C_FIMV_MC_DRAMBASE_ADR_B); 		// channelB
	WRITEL(1, S3C_FIMV_MC_RS_IBASE); 			// fw location sel : 0->A, 1->B
	WRITEL(1, S3C_FIMV_NUM_MASTER);				// 0->1master, 1->2master
	
	/* peter, set hidden reg.
	mfcInp32(MFC_MR_BUSIF_CTRL,uMrBusIfReg);
	uMrBusIfReg |= (1<<2);
	mfcOutp32(MFC_MR_BUSIF_CTRL, uMrBusIfReg);
	*/
	
	/*
	 * 2. Initialize registers of stream I/F for decoder
	 */
	WRITEL(0xffff, S3C_FIMV_SI_CH1_INST_ID);
	
	WRITEL(0, S3C_FIMV_RISC2HOST_CMD);
	WRITEL(0, S3C_FIMV_HOST2RISC_CMD);

	/*
	 * 3. Release reset signal to the RISC.
	 */
	WRITEL(0x3ff, S3C_FIMV_SW_RESET);	

	if(s3c_mfc_wait_for_done(R2H_CMD_FW_STATUS_RET) == 0){
		mfc_err("MFCINST_ERR_FW_LOAD_FAIL\n");
		return MFCINST_ERR_FW_LOAD_FAIL;
	}

	/*
	 * 4. Initialize firmware
	 */
	fw_buf_size = FIRMWARE_CODE_SIZE + MFC_FW_TOTAL_BUF_SIZE;
	s3c_mfc_cmd_host2risc(H2R_CMD_SYS_INIT, fw_buf_size);

	if(s3c_mfc_wait_for_done(R2H_CMD_SYS_INIT_RET) == 0){
		mfc_err("MFCINST_ERR_FW_INIT_FAIL\n");
		return MFCINST_ERR_FW_INIT_FAIL;
	}	

	mfc_debug("FW_PHY_BUFFER : 0x%08x\n", READL(S3C_FIMV_MC_DRAMBASE_ADR_B));
	mfc_debug("s3c_mfc_init_hw--\n");
	
	return MFCINST_RET_OK;
}

static int s3c_mfc_get_inst_no(MFC_CODEC_TYPE codec_type)
{
	unsigned int codec;

	codec = (unsigned int)codec_type;

	s3c_mfc_cmd_host2risc(H2R_CMD_OPEN_INSTANCE, codec);

	if(s3c_mfc_wait_for_done(R2H_CMD_OPEN_INSTANCE_RET) == 0){
		mfc_err("MFCINST_ERR_FW_INIT_FAIL\n"); // peter, check this status
		return MFCINST_ERR_FW_INIT_FAIL;
	}

	mfc_debug("INSTANCE NO : %d, CODEC_TYPE : %d --\n", READL(S3C_FIMV_RISC2HOST_ARG1), codec);

	// Get the instance no
	return (READL(S3C_FIMV_RISC2HOST_ARG1));
	
/*
	for(i = 0; i < MFC_MAX_INSTANCE_NUM; i++)
		if (s3c_mfc_inst_no[i] == 0) {
			s3c_mfc_inst_no[i] = 1;
			return i;
		}
*/
	
}

void s3c_mfc_return_inst_no(int inst_no)
{
	unsigned int codec;

	codec = (unsigned int)codec_type;

	s3c_mfc_cmd_host2risc(H2R_CMD_CLOSE_INSTANCE, codec);

	if(s3c_mfc_wait_for_done(R2H_CMD_CLOSE_INSTANCE_RET) == 0){
		mfc_err("MFCINST_ERR_FW_INIT_FAIL\n"); // peter, check this status
		return MFCINST_ERR_FW_INIT_FAIL;
	}

	mfc_debug("INSTANCE NO : %d, CODEC_TYPE : %d --\n", READL(S3C_FIMV_RISC2HOST_ARG1), codec);

/*
	// Get the instance no
	return (READL(S3C_FIMV_RISC2HOST_ARG1));
	if ((inst_no >= 0) && (inst_no < MFC_MAX_INSTANCE_NUM))
		s3c_mfc_inst_no[inst_no] = 0;
*/
}

#if 0
MFC_ERROR_CODE s3c_mfc_init_encode(s3c_mfc_inst_ctx  *MfcCtx,  s3c_mfc_args *args)
{
	mfc_debug("++\n");

	MfcCtx->MfcCodecType = ((MFC_CODEC_TYPE *) args)[0];

	/* 3. CHANNEL SET
	 * 	- set codec firmware
	 * 	- set codec_type/channel_id/post_on
	 */
	 
	// peter, MFC_InitProcessForDecoding()
	mfc_ctx->InstNo = s3c_mfc_get_inst_no();
	if (mfc_ctx->InstNo < 0) {
		kfree(MfcCtx);
		mfc_err("MFCINST_INST_NUM_EXCEEDED\n");
		ret = -EPERM;
		goto out_open;
	} 

	s3c_mfc_set_codec_firmware(MfcCtx);

	WRITEL(s3c_mfc_get_codec_type(MfcCtx->MfcCodecType), S3C_FIMV_STANDARD_SEL);
	WRITEL(MFC_CHANNEL_SET, S3C_FIMV_COMMAND_TYPE);
	WRITEL(MfcCtx->InstNo, S3C_FIMV_CH_ID);
	WRITEL(0, S3C_FIMV_POST_ON);
	WRITEL(1, S3C_FIMV_BITS_ENDIAN);

	WRITEL(INT_LEVEL_BIT, S3C_FIMV_INT_MODE);
	WRITEL(0, S3C_FIMV_INT_OFF);
	WRITEL(1, S3C_FIMV_INT_DONE_CLEAR);
	WRITEL((INT_MFC_FRAME_DONE|INT_MFC_FW_DONE), S3C_FIMV_INT_MASK);

	s3c_mfc_cmd_frame_start();

	if(s3c_mfc_wait_for_done(MFC_INTR_FRAME_DONE) == 0){
		mfc_err("MFCINST_ERR_FW_LOAD_FAIL\n");
		return MFCINST_ERR_FW_LOAD_FAIL;
	}

	/* 4. INIT CODEC
	 * 	- change Endian(important!!!)
	 * 	- set Encoder Init SFR
	 */

	s3c_mfc_set_encode_init_param(MfcCtx->InstNo, MfcCtx->MfcCodecType, args);

	WRITEL(s3c_mfc_get_codec_type(MfcCtx->MfcCodecType), S3C_FIMV_STANDARD_SEL);
	WRITEL(MfcCtx->InstNo, S3C_FIMV_CH_ID);
	WRITEL(MFC_INIT_CODEC, S3C_FIMV_COMMAND_TYPE);
	WRITEL(1, S3C_FIMV_BITS_ENDIAN);
	WRITEL(INT_LEVEL_BIT, S3C_FIMV_INT_MODE);
	WRITEL(0, S3C_FIMV_INT_OFF);
	WRITEL(1, S3C_FIMV_INT_DONE_CLEAR);
	WRITEL((INT_MFC_FRAME_DONE|INT_MFC_FW_DONE), S3C_FIMV_INT_MASK);

	s3c_mfc_cmd_frame_start();

	if(s3c_mfc_wait_for_done(MFC_INTR_FRAME_DONE) == 0){
		mfc_err("MFCINST_ERR_FW_LOAD_FAIL\n");
		return MFCINST_ERR_FW_LOAD_FAIL;
	}

	s3c_mfc_backup_context(MfcCtx);
	mfc_debug("--\n");
	return MFCINST_RET_OK;
}


MFC_ERROR_CODE s3c_mfc_exe_encode(s3c_mfc_inst_ctx  *MfcCtx,  s3c_mfc_args *args)
{
	s3c_mfc_enc_exe_arg          *EncExeArg;

	/* 
	 * 5. Encode Frame
	 */

	EncExeArg = (s3c_mfc_enc_exe_arg *) args;
	mfc_debug("++ EncExeArg->in_strm_st : 0x%08x EncExeArg->in_strm_end :0x%08x \r\n", \
								EncExeArg->in_strm_st, EncExeArg->in_strm_end);
	mfc_debug("EncExeArg->in_Y_addr : 0x%08x EncExeArg->in_CbCr_addr :0x%08x \r\n",   \
								EncExeArg->in_Y_addr, EncExeArg->in_CbCr_addr);

	s3c_mfc_restore_context(MfcCtx);

	s3c_mfc_set_vsp_buffer(MfcCtx->InstNo);

	if ((MfcCtx->forceSetFrameType > DONT_CARE) && 		\
		(MfcCtx->forceSetFrameType <= NOT_CODED)) {
		WRITEL(MfcCtx->forceSetFrameType, S3C_FIMV_CODEC_COMMAND);
		MfcCtx->forceSetFrameType = DONT_CARE;
	} else 
		WRITEL(DONT_CARE, S3C_FIMV_CODEC_COMMAND);
	/*
	if((EncExeArg->in_ForceSetFrameType >= DONT_CARE) && (EncExeArg->in_ForceSetFrameType <= NOT_CODED))
		WRITEL(EncExeArg->in_ForceSetFrameType, S3C_FIMV_CODEC_COMMAND);
	*/
	/*
	 * Set Interrupt
	 */

	WRITEL(EncExeArg->in_Y_addr, S3C_FIMV_ENC_CUR_Y_ADR);
	WRITEL(EncExeArg->in_CbCr_addr, S3C_FIMV_ENC_CUR_CBCR_ADR);
	WRITEL(EncExeArg->in_strm_st, S3C_FIMV_EXT_BUF_START_ADDR);
	WRITEL(EncExeArg->in_strm_end, S3C_FIMV_EXT_BUF_END_ADDR);
	WRITEL(EncExeArg->in_strm_st, S3C_FIMV_HOST_PTR);

	WRITEL(MFC_FRAME_RUN, S3C_FIMV_COMMAND_TYPE);
	WRITEL(MfcCtx->InstNo, S3C_FIMV_CH_ID);
	WRITEL(s3c_mfc_get_codec_type(MfcCtx->MfcCodecType), S3C_FIMV_STANDARD_SEL);

	WRITEL(INT_LEVEL_BIT, S3C_FIMV_INT_MODE);
	WRITEL(0, S3C_FIMV_INT_OFF);
	WRITEL(1, S3C_FIMV_INT_DONE_CLEAR);
	WRITEL(1, S3C_FIMV_BITS_ENDIAN);
	WRITEL((INT_MFC_FRAME_DONE|INT_MFC_FW_DONE), S3C_FIMV_INT_MASK);

	s3c_mfc_cmd_frame_start();

	if (s3c_mfc_wait_for_done(MFC_INTR_FRAME_DONE) == 0) {
		mfc_err("MFCINST_ERR_ENC_ENCODE_DONE_FAIL\n");
		return MFCINST_ERR_ENC_ENCODE_DONE_FAIL;
	}

	EncExeArg->out_frame_type = READL(S3C_FIMV_RET_VALUE);
	EncExeArg->out_encoded_size = READL(S3C_FIMV_ENC_UNIT_SIZE);
	EncExeArg->out_header_size  = READL(S3C_FIMV_ENC_HEADER_SIZE);


	mfc_debug("-- frame type(%d) encodedSize(%d)\r\n", \
		EncExeArg->out_frame_type, EncExeArg->out_encoded_size);
	return MFCINST_RET_OK;
}
#endif

MFC_ERROR_CODE s3c_mfc_init_decode(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{
	MFC_ERROR_CODE   ret;
	s3c_mfc_dec_init_arg_t *init_arg;
	s3c_mfc_dec_type dec_type = SEQ_HEADER;
	//unsigned int FWPhyBuf;

	mfc_debug("++\n");
	init_arg = (s3c_mfc_dec_init_arg_t *)args;
	//FWPhyBuf = s3c_mfc_get_fw_buf_phys_addr();

	/* Context setting from input param */
	mfc_ctx->MfcCodecType = init_arg->in_codec_type;
	mfc_ctx->IsPackedPB = init_arg->in_packed_PB;
	
	/* OPEN CHANNEL
	 * 	- set open instance using codec_type
	 * 	- get the instance no
	 */
	// peter, MFC_InitProcessForDecoding()
	mfc_ctx->InstNo = s3c_mfc_get_inst_no(mfc_ctx->MfcCodecType);
	if (mfc_ctx->InstNo < 0) {
		kfree(MfcCtx);
		mfc_err("MFCINST_INST_NUM_EXCEEDED\n");
		ret = -EPERM;
		goto out_open;
	}

	// peter, Does it need to set for decoder ?
	WRITEL(mfc_ctx->postEnable, S3C_FIMV_ENC_LF_CTRL);
	WRITEL(0, S3C_FIMV_ENC_PXL_CACHE_CTRL);		

	/* INIT CODEC
	 * 	- set input stream buffer 
	 * 	- set sequence done command
	 * 	- set input risc buffer
	 * 	- set NUM_EXTRA_DPB
	 */	
	// peter, MFC_ParseStreamHeader() 
	s3c_mfc_set_dec_stream_buffer(mfc_ctx->InstNo, init_arg->in_strm_buf, init_arg->in_strm_size); 
	WRITEL((SEQ_HEADER<<16 & 0x30000)|(mfc_ctx->InstNo), S3C_FIMV_SI_CH1_INST_ID);

	if (s3c_mfc_wait_for_done(R2H_CMD_SEQ_DONE_RET) == 0) {
		mfc_err("MFCINST_ERR_DEC_HEADER_DECODE_FAIL\n");
		return MFCINST_ERR_DEC_HEADER_DECODE_FAIL;
	}
		
	s3c_mfc_set_risc_buffer(mfc_ctx->MfcCodecType, mfc_ctx->InstNo);	

	/* out param & context setting from header decoding result */
	mfc_ctx->img_width = READL(S3C_FIMV_SI_HRESOL);
	mfc_ctx->img_height = READL(S3C_FIMV_SI_VRESOL);	

	init_arg->out_img_width = READL(S3C_FIMV_SI_HRESOL);
	init_arg->out_img_height = READL(S3C_FIMV_SI_VRESOL);

	/* in the case of VC1 interlace, height will be the multiple of 32
	 * otherwise, height and width is the mupltiple of 16
	 */
	init_arg->out_buf_width = (READL(S3C_FIMV_SI_HRESOL)+15)/16*16;
	init_arg->out_buf_height = (READL(S3C_FIMV_SI_VRESOL)+31)/32*32;

	/* peter, It should have extraDPB to protect tearing in the display
	 */
	init_arg->out_dpb_cnt = READL(S3C_FIMV_SI_BUF_NUMBER); 
	mfc_ctx->DPBCnt = READL(S3C_FIMV_SI_BUF_NUMBER);

/*
	switch (mfc_ctx->MfcCodecType) {
	case H264_DEC: 
		init_arg->out_dpb_cnt = (READL(S3C_FIMV_SI_BUF_NUMBER)*3)>>1; 
		mfc_ctx->DPBCnt = READL(S3C_FIMV_SI_BUF_NUMBER);
		break;

	case MPEG4_DEC:
	case MPEG2_DEC: 
	case DIVX_DEC: 
	case XVID_DEC:
		init_arg->out_dpb_cnt = ((NUM_MPEG4_DPB * 3) >> 1) + NUM_POST_DPB + mfc_ctx->extraDPB;
		mfc_ctx->DPBCnt = NUM_MPEG4_DPB;
		break;

	case VC1_DEC:
		init_arg->out_dpb_cnt = ((NUM_VC1_DPB * 3) >> 1)+ MfcCtx->extraDPB;
		mfc_ctx->DPBCnt = NUM_VC1_DPB + mfc_ctx->extraDPB;
		break;

	default:
		init_arg->out_dpb_cnt = ((NUM_MPEG4_DPB * 3) >> 1)+ NUM_POST_DPB + mfc_ctx->extraDPB;
		mfc_ctx->DPBCnt = NUM_MPEG4_DPB;
	}
*/
	mfc_ctx->totalDPBCnt = init_arg->out_dpb_cnt;

	mfc_debug("buf_width : %d buf_height : %d out_dpb_cnt : %d MfcCtx->DPBCnt : %d\n", \
				init_arg->out_img_width, init_arg->out_img_height, init_arg->out_dpb_cnt, mfc_ctx->DPBCnt);
	mfc_debug("img_width : %d img_height : %d\n", \
				init_arg->out_img_width, init_arg->out_img_height);

	s3c_mfc_backup_context(mfc_ctx);

	mfc_debug("--\n");
	
	return MFCINST_RET_OK;
}

static MFC_ERROR_CODE s3c_mfc_decode_one_frame(s3c_mfc_inst_ctx *mfc_ctx,  s3c_mfc_dec_exe_arg_t *dec_arg, 
						unsigned int *consumed_strm_size)
{
	int ret;
	unsigned int frame_type;
	static int count = 0;

	count++;
	
	mfc_debug("++ IntNo%d(%d)\r\n", mfc_ctx->InstNo, count);

	s3c_mfc_restore_context(mfc_ctx);	

	s3c_mfc_set_dec_stream_buffer(dec_arg->in_strm_buf, dec_arg->in_strm_size);
	
	s3c_mfc_set_dec_frame_buffer(mfc_ctx, dec_arg->in_frm_buf, dec_arg->in_frm_size);

	/* Set RISC buffer */
	s3c_mfc_set_risc_buffer(mfc_ctx->MfcCodecType, mfc_ctx->InstNo);

	if(mfc_ctx->endOfFrame) {
		WRITEL((LAST_FRAME<<16 & 0x30000)|(mfc_ctx->InstNo), S3C_FIMV_SI_CH1_INST_ID);
		mfc_ctx->endOfFrame = 0;
	} else {
		WRITEL((FRAME<<16 & 0x30000)|(mfc_ctx->InstNo), S3C_FIMV_SI_CH1_INST_ID);
	}	

	if (s3c_mfc_wait_for_done(R2H_CMD_FRAME_DONE_RET) == 0) {
		mfc_err("MFCINST_ERR_DEC_DECODE_DONE_FAIL\n");
		return MFCINST_ERR_DEC_DECODE_DONE_FAIL;
	}		

	if ((READL(S3C_FIMV_SI_DISPLAY_STATUS) & 0x3) == DECODING_ONLY) {
		dec_arg->out_display_Y_addr = 0;
		dec_arg->out_display_C_addr = 0;
	} else {
		dec_arg->out_display_Y_addr = READL(S3C_FIMV_SI_DISPLAY_Y_ADR);
		dec_arg->out_display_C_addr = READL(S3C_FIMV_SI_DISPLAY_C_ADR);
	}
	
	frame_type = READL(S3C_FIMV_ENC_SI_SLICE_TYPE); // peter, check whether this is for only encoder
	mfc_ctx->FrameType = (s3c_mfc_frame_type)(frame_type & 0x3);

	s3c_mfc_backup_context(mfc_ctx);

	mfc_debug("(Y_ADDR : 0x%08x  C_ADDR : 0x%08x)\r\n", \
		dec_arg->out_display_Y_addr , dec_arg->out_display_C_addr);  
	mfc_debug("(in_strmsize : 0x%08x  consumed byte : 0x%08x)\r\n", \
			dec_arg->in_strm_size, READL(S3C_FIMV_RET_VALUE));      

	*consumed_strm_size = READL(S3C_FIMV_RET_VALUE); // peter, find out related reg. for C110
	return MFCINST_RET_OK;
}


MFC_ERROR_CODE s3c_mfc_exe_decode(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{
	MFC_ERROR_CODE ret;
	s3c_mfc_dec_exe_arg_t *dec_arg;
	unsigned int consumed_strm_size;
	
	/* 6. Decode Frame */
	mfc_debug("++\n");

	dec_arg = (s3c_mfc_dec_exe_arg_t *)args;
	ret = s3c_mfc_decode_one_frame(mfc_ctx, dec_arg, &consumed_strm_size);

	if((mfc_ctx->IsPackedPB) && (mfc_ctx->FrameType == MFC_RET_FRAME_P_FRAME) \
		&& (dec_arg->in_strm_size - consumed_strm_size > 4)) {
		mfc_debug("Packed PB\n");
		dec_arg->in_strm_buf += consumed_strm_size;
		dec_arg->in_strm_size -= consumed_strm_size;

		ret = s3c_mfc_decode_one_frame(mfc_ctx, dec_arg, &consumed_strm_size);
	}
	mfc_debug("--\n");

	return ret; 
}

MFC_ERROR_CODE s3c_mfc_deinit_hw(s3c_mfc_inst_ctx  *mfc_ctx)
{
	s3c_mfc_restore_context(mfc_ctx);

	return MFCINST_RET_OK;
}

MFC_ERROR_CODE s3c_mfc_get_config(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{
	return MFCINST_RET_OK;
}


MFC_ERROR_CODE s3c_mfc_set_config(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{
	s3c_mfc_set_config_arg_t *set_cnf_arg;
	set_cnf_arg = (s3c_mfc_set_config_arg_t *)args;

	switch (set_cnf_arg->in_config_param) {
	case MFC_DEC_SETCONF_POST_ENABLE:
		if (mfc_ctx->MfcState >= MFCINST_STATE_DEC_INITIALIZE) {
			mfc_err("MFC_DEC_SETCONF_POST_ENABLE : state is invalid\n");
			return MFCINST_ERR_STATE_INVALID;
		}

		if((set_cnf_arg->in_config_value[0] == 0) || (set_cnf_arg->in_config_value[0] == 1))
			mfc_ctx->postEnable = set_cnf_arg->in_config_value[0];
		else {
			mfc_warn("POST_ENABLE should be 0 or 1\n");
			mfc_ctx->postEnable = 0;
		}
		break;	
		
	case MFC_DEC_SETCONF_EXTRA_BUFFER_NUM:
		if (mfc_ctx->MfcState >= MFCINST_STATE_DEC_INITIALIZE) {
			mfc_err("MFC_DEC_SETCONF_EXTRA_BUFFER_NUM : state is invalid\n");
			return MFCINST_ERR_STATE_INVALID;
		}
		if ((set_cnf_arg->in_config_value[0] >= 0) || (set_cnf_arg->in_config_value[0] <= MFC_MAX_EXTRA_DPB))
			mfc_ctx->extraDPB = set_cnf_arg->in_config_value[0];
		else {
			mfc_warn("EXTRA_BUFFER_NUM should be between 0 and 5...It will be set 5 by default\n");
			mfc_ctx->extraDPB = MFC_MAX_EXTRA_DPB;
		}
		break;

	// peter, check whether C110 has this function	
	/*	
	case MFC_DEC_SETCONF_DISPLAY_DELAY:
		if (mfc_ctx->MfcState >= MFCINST_STATE_DEC_INITIALIZE) {
			mfc_err("MFC_DEC_SETCONF_DISPLAY_DELAY : state is invalid\n");
			return MFCINST_ERR_STATE_INVALID;
		}
		if (mfc_ctx->MfcCodecType == H264_DEC) {
			if ((set_cnf_arg->in_config_value[0] >= 0) || (set_cnf_arg->in_config_value[0] < 16))
				mfc_ctx->displayDelay = set_cnf_arg->in_config_value[0];
			else {
				mfc_warn("DISPLAY_DELAY should be between 0 and 16\n");
				mfc_ctx->displayDelay = 0;
			}
		} else {
			mfc_warn("MFC_DEC_SETCONF_DISPLAY_DELAY is only valid for H.264\n");
			mfc_ctx->displayDelay = 0;
		}
		break;
	*/	
	case MFC_DEC_SETCONF_IS_LAST_FRAME:
		if (mfc_ctx->MfcState != MFCINST_STATE_DEC_EXE) {
			mfc_err("MFC_DEC_SETCONF_IS_LAST_FRAME : state is invalid\n");
			return MFCINST_ERR_STATE_INVALID;
		}

		if ((set_cnf_arg->in_config_value[0] == 0) || (set_cnf_arg->in_config_value[0] == 1))
			mfc_ctx->endOfFrame = set_cnf_arg->in_config_value[0];
		else {
			mfc_warn("IS_LAST_FRAME should be 0 or 1\n");
			mfc_ctx->endOfFrame = 0;
		}
		break;
			
	case MFC_ENC_SETCONF_FRAME_TYPE:
		if ((mfc_ctx->MfcState < MFCINST_STATE_ENC_INITIALIZE) || (mfc_ctx->MfcState > MFCINST_STATE_ENC_EXE)) {
			mfc_err("MFC_ENC_SETCONF_FRAME_TYPE : state is invalid\n");
			return MFCINST_ERR_STATE_INVALID;
		}

		if ((set_cnf_arg->in_config_value[0] < DONT_CARE) || (set_cnf_arg->in_config_value[0] > NOT_CODED))
			mfc_ctx->forceSetFrameType = set_cnf_arg->in_config_value[0];
		else {
			mfc_warn("FRAME_TYPE should be between 0 and 2\n");
			mfc_ctx->forceSetFrameType = DONT_CARE;
		}
		break;
		
	default:
		mfc_err("invalid config param\n");
		return MFCINST_ERR_SET_CONF;
	}
	
	return MFCINST_RET_OK;
}

