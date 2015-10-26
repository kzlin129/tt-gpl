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


#if !defined(__DWC_OTG_ATTR_H__)
#define __DWC_OTG_ATTR_H__

/** @file
 * This file contains the interface to the Linux device attributes.
 */
extern struct device_attribute dev_attr_regoffset;
extern struct device_attribute dev_attr_regvalue;

extern struct device_attribute dev_attr_mode;
extern struct device_attribute dev_attr_hnpcapable;
extern struct device_attribute dev_attr_srpcapable;
extern struct device_attribute dev_attr_hnp;
extern struct device_attribute dev_attr_srp;
extern struct device_attribute dev_attr_buspower;
extern struct device_attribute dev_attr_bussuspend;
extern struct device_attribute dev_attr_busconnected;
extern struct device_attribute dev_attr_gotgctl;
extern struct device_attribute dev_attr_gusbcfg;
extern struct device_attribute dev_attr_grxfsiz;
extern struct device_attribute dev_attr_gnptxfsiz;
extern struct device_attribute dev_attr_gpvndctl;
extern struct device_attribute dev_attr_ggpio;
extern struct device_attribute dev_attr_guid;
extern struct device_attribute dev_attr_gsnpsid;
extern struct device_attribute dev_attr_devspeed;
extern struct device_attribute dev_attr_enumspeed;
extern struct device_attribute dev_attr_hptxfsiz;
extern struct device_attribute dev_attr_hprt0;
#ifdef CONFIG_USB_DWC_OTG_LPM
extern struct device_attribute dev_attr_lpm_response;
extern struct device_attribute dev_attr_sleep_local_dev;
extern struct device_attribute devi_attr_sleep_status;
#endif

void dwc_otg_attr_create (
#ifdef LM_INTERFACE
	struct lm_device *dev
#elif  PCI_INTERFACE
	struct pci_dev *dev
#endif
	);

void dwc_otg_attr_remove (
#ifdef LM_INTERFACE
	struct lm_device *dev
#elif  PCI_INTERFACE
	struct pci_dev *dev
#endif
	);
#endif
