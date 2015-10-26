/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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


/*
*
*****************************************************************************
*
*  REG_NAND.h
*
*  PURPOSE:
*
*     This file contains definitions for the nand registers:
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_NAND_H )
#define __ASM_ARCH_REG_NAND_H

/* ---- Include Files ---------------------------------------------------- */
#include <asm/arch/hardware.h>
#include <asm/arch/reg_sys.h>
#include <asm/arch/reg_umi.h>

/* ---- Constants and Types ---------------------------------------------- */

// DMA accesses by the bootstrap need hard nonvirtual addresses
#define REG_NAND_PHYS_DATA16   (HW_NAND_BASE + 8)

#ifndef __ARMEB__
	#define REG_NAND_PHYS_DATA8    (HW_NAND_BASE + 8 )
#else
	#define REG_NAND_PHYS_DATA8	   (HW_NAND_BASE + 8 + 1 )
#endif

#define REG_NAND_CMD            __REG16( HW_NAND_BASE + 0 )
#define REG_NAND_ADDR           __REG16( HW_NAND_BASE + 4 )
#define REG_NAND_DATA16         __REG16( REG_NAND_PHYS_DATA16 )
#define REG_NAND_DATA8          __REG8 ( REG_NAND_PHYS_DATA8 )

/* Linux DMA requires physical address of the data register */
#define REG_NAND_DATA16_PADDR    HW_IO_VIRT_TO_PHYS( REG_NAND_PHYS_DATA16 )
#define REG_NAND_DATA8_PADDR     HW_IO_VIRT_TO_PHYS( REG_NAND_PHYS_DATA8 )

#define NAND_BUS_16BIT()        ( REG_SYS_SUCR & REG_SYS_SUCR_NAND_BUS_WIDTH_16 )
#define NAND_BUS_8BIT()         ( !NAND_BUS_16BIT() )
#define NAND_WAIT_DONE()        { while ((REG_UMI_NAND_RCSR & REG_UMI_NAND_RCSR_RDY) == 0); }

#endif
