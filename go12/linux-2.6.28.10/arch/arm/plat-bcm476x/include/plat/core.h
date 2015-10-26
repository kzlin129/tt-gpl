/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  linux/arch/arm/mach-versatile/core.h
 *
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 */

#ifndef __BCM476X_CORE_H
#define __BCM476X_CORE_H

#include <linux/amba/bus.h>
#include <asm/arch/bcm4760_addressmap.h>
#include <mach/irqs.h>

#define AMBA_DEVICE(name,busid,base,plat)   \
static struct amba_device name##_device = {     \
   .dev = {                                     \
        .coherent_dma_mask = ~0,                  \
        .bus_id = busid,                          \
        .platform_data = plat                     \
   },                                           \
   .res = {                                     \
        .start = base##_REG_BASE_ADDR,            \
        .end = base##_REG_BASE_ADDR + SZ_4K - 1,  \
        .flags = IORESOURCE_MEM                   \
   },                                           \
   .dma_mask = ~0,                              \
   .irq = {                                     \
        BCM4760_INTR_##base                \
   }                                            \
   /* .dma		= base##_DMA,*/				      \
}

#endif
