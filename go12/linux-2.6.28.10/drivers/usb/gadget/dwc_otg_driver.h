/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_driver.c $
 * $Revision: #76 $
 * $Date: 2009/05/03 $
 * $Change: 1245589 $
 *
 * Copyright (c) 2003-2010 Synopsys, Inc.
 *
 * This software driver reference implementation and other associated
 * documentation (hereinafter, "Software") is an unsupported work of
 * Synopsys, Inc. unless otherwise expressly agreed to in writing between
 * Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. Permission is hereby granted,
 * free of charge, to any person obtaining a copy of thsi software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify
 * merge, publish, distribute sublicense, and/or seel copies of the Software,
 * and to permit persons to whom the Software is furnted to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Unless you and Broadcom execute a separate written software license agreement
 * governing use of this software, this software is licensed to you under the
 * terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written consent.
 * ========================================================================= */
// @DB: 091023 - Added common switch header
#include "../dwc_common_port/dwc_defines.h"

#ifndef __DWC_OTG_DRIVER_H__
#define __DWC_OTG_DRIVER_H__

/** @file
 * This file contains the interface to the Linux driver.
 */
#include "dwc_otg_core_if.h"

/* Type declarations */
struct dwc_otg_pcd;
struct dwc_otg_hcd;

#ifdef  PCI_INTERFACE
#include <linux/pci.h>
#endif



/**
 * This structure is a wrapper that encapsulates the driver components used to
 * manage a single DWC_otg controller.
 */
typedef struct dwc_otg_device {
	/** Base address returned from ioremap() */
	void *base;

#ifdef LM_INTERFACE
	struct lm_device *lmdev;
#elif  PCI_INTERFACE
	int rsrc_start;
	int rsrc_len;
#endif

	/** Pointer to the core interface structure. */
	dwc_otg_core_if_t *core_if;

	/** Register offset for Diagnostic API. */
	uint32_t reg_offset;

	/** Pointer to the PCD structure. */
	struct dwc_otg_pcd *pcd;

	/** Pointer to the HCD structure. */
	struct dwc_otg_hcd *hcd;

	/** Flag to indicate whether the common IRQ handler is installed. */
	uint8_t common_irq_installed;

} dwc_otg_device_t;

// @DB: 100129 - Add a local version of lm_get_drvdata and lm_set_drvdata
// so we can swap a global variable definition version until the common
// one using the lm_dev can be fixed
#if 1
extern dwc_otg_device_t	*g_dwc_otg_device;

#define DWC_OTG_LM_GET_DRVDATA(ignored)		(g_dwc_otg_device)
#define DWC_OTG_LM_SET_DRVDATA(ignored, v)	{g_dwc_otg_device = (v);}
#else
#define DWC_OTG_LM_GET_DRVDATA(d)			lm_get_drvdata(d)
#define DWC_OTG_LM_SET_DRVDATA(d, v)		lm_set_drvdata(d,v)
#endif


extern uint32_t g_dwc_otg_mode;
#define DWC_OTG_NO_MODE				0xFFFFFFFF
#define DWC_OTG_DEVICE_MODE			0x00000000
#define DWC_OTG_HOST_MODE			0x00000001
#endif
