/* 
 * drivers/media/video/samsung/mfc40/s3c_mfc_memory.h
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

#ifndef _S3C_MFC_MEMORY_H_
#define _S3C_MFC_MEMORY_H_

#include "s3c_mfc_common.h"
#include "s3c_mfc_types.h"

#ifdef CONFIG_VIDEO_MFC_MAX_INSTANCE
#define MFC_MAX_INSTANCE_NUM (CONFIG_VIDEO_MFC_MAX_INSTANCE)
#endif


#define MFC_MAX_FW_NUM		(8)
#define MFC_MAX_WIDTH		(1920)
#define MFC_MAX_HEIGHT		(1080)

/* All buffer size have to be aligned to 64 */
#define FIRMWARE_CODE_SIZE	(195964) /* 195,964 byte */
#define MFC_FW_SYSTEM_SIZE	(0x100000) /* 1MB : 1024x1024 */
#define MFC_FW_BUF_SIZE		(0x80000) /* 512KB : 512x1024 size per instance */
#define MFC_FW_TOTAL_BUF_SIZE	(MFC_FW_SYSTEM_SIZE + MFC_MAX_INSTANCE_NUM * MFC_FW_BUF_SIZE) 

#define CPB_BUF_SIZE		(0x400000) /* 4MB : 4x1024x1024 */
#define DESC_BUF_SIZE		(0x20000)  /* 128KB : 128x1024 */

//#define VSP_BUF_SIZE		(393216) /* 393,216 byte */
//#define DB_STT_SIZE		(MFC_MAX_WIDTH*4*32) /* 163,840 byte */
//#define MFC_SFR_BUF_SIZE	sizeof(S5PC100_MFC_SFR)

volatile unsigned char *s3c_mfc_get_fw_buf_virt_addr(void);		/* port1 base address */
volatile unsigned char *s3c_mfc_get_risc_buf_virt_addr(int instNo);
volatile unsigned char *s3c_mfc_get_data_buf_virt_addr(void);
volatile unsigned char *s3c_mfc_get_dpb_luma_buf_virt_addr(void);	/* port0 base address */

unsigned int s3c_mfc_get_fw_buf_phys_addr(void);
unsigned int s3c_mfc_get_risc_buf_phys_addr(int instNo);
unsigned int s3c_mfc_get_data_buf_phys_addr(void);
unsigned int s3c_mfc_get_dpb_luma_buf_phys_addr(void);

#endif /* _S3C_MFC_MEMORY_H_ */
