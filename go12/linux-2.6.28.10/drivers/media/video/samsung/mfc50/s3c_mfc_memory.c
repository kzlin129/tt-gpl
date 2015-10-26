/*
 * drivers/media/video/samsung/mfc40/s3c_mfc_memory.c
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

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <mach/map.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/sizes.h>
#include <asm/memory.h>

#include "s3c_mfc_memory.h"
#include "s3c_mfc_logmsg.h"
#include "s3c_mfc_types.h"
#include "s3c_mfc_interface.h"

extern volatile unsigned char	*s3c_mfc_virt_buf;
//extern void __iomem		*s3c_mfc_sfr_virt_base;
dma_addr_t s3c_mfc_phys_data_buf, s3c_mfc_phys_dpb_luma_buf;
volatile unsigned char *s3c_mfc_virt_data_buf;
volatile unsigned char *s3c_mfc_virt_dpb_luma_buf;


volatile unsigned char *s3c_mfc_get_fw_buf_virt_addr()	
{
	volatile unsigned char *virt_addr;

	virt_addr = s3c_mfc_virt_buf;
	return virt_addr; 
}

// peter, for desc, motion vector, bitplane0/1/2, etc
volatile unsigned char *s3c_mfc_get_risc_buf_virt_addr(int inst_no)	
{
	volatile unsigned char *virt_addr;

	virt_addr = s3c_mfc_get_fw_buf_virt_addr() + FIRMWARE_CODE_SIZE + MFC_FW_TOTAL_BUF_SIZE + inst_no*MFC_FW_BUF_SIZE;
	return virt_addr; 
}

volatile unsigned char *s3c_mfc_get_data_buf_virt_addr()
{
	
	s3c_mfc_virt_data_buf = s3c_mfc_get_risc_buf_virt_addr(MFC_MAX_INSTANCE_NUM);
	
	return s3c_mfc_virt_data_buf; 	
}

volatile unsigned char *s3c_mfc_get_dpb_luma_buf_virt_addr()
{
	
	s3c_mfc_virt_dpb_luma_buf = s3c_get_media_memory(S3C_MDEV_MFC); // peter, it should be changed
	
	return s3c_mfc_virt_dpb_luma_buf; 	
}

unsigned int s3c_mfc_get_fw_buf_phys_addr()
{
	return (unsigned int)__virt_to_phys((unsigned int)s3c_mfc_get_fw_buf_virt_addr()); /* s3c_get_media_memory(S3C_MDEV_MFC); */
}

// peter, for desc, motion vector, bitplane0/1/2, etc
unsigned int s3c_mfc_get_risc_buf_phys_addr(int inst_no)	
{
	return (unsigned int)__virt_to_phys((unsigned int)s3c_mfc_get_risc_buf_virt_addr(inst_no));	
}

unsigned int s3c_mfc_get_data_buf_phys_addr()
{
	s3c_mfc_phys_data_buf = (unsigned int)__virt_to_phys((unsigned int)s3c_mfc_virt_data_buf); 
	
	return (unsigned int)s3c_mfc_phys_data_buf;
}

unsigned int s3c_mfc_get_dpb_luma_buf_phys_addr()
{
	s3c_mfc_phys_dpb_luma_buf = (unsigned int)__virt_to_phys((unsigned int)s3c_mfc_virt_dpb_luma_buf); 
	
	return (unsigned int)s3c_mfc_phys_dpb_luma_buf;
}



