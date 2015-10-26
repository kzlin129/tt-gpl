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


#ifndef __DWC_OTG_DBG_H__
#define __DWC_OTG_DBG_H__

/** @file
 * This file defines debug levels.
 * Debugging support vanishes in non-debug builds.  
 */

/**
 * The Debug Level bit-mask variable.
 */
extern uint32_t g_dbg_lvl;
/**
 * Set the Debug Level variable.
 */
static inline uint32_t SET_DEBUG_LEVEL(const uint32_t new)
{
	uint32_t old = g_dbg_lvl;
	g_dbg_lvl = new;
	return old;
}

/** When debug level has the DBG_CIL bit set, display CIL Debug messages. */
#define DBG_CIL		(0x2)
/** When debug level has the DBG_CILV bit set, display CIL Verbose debug
 * messages */
#define DBG_CILV	(0x20)
/**  When debug level has the DBG_PCD bit set, display PCD (Device) debug
 *  messages */
#define DBG_PCD		(0x4)
/** When debug level has the DBG_PCDV set, display PCD (Device) Verbose debug
 * messages */
#define DBG_PCDV	(0x40)
/** When debug level has the DBG_HCD bit set, display Host debug messages */
#define DBG_HCD		(0x8)
/** When debug level has the DBG_HCDV bit set, display Verbose Host debug
 * messages */
#define DBG_HCDV	(0x80)
/** When debug level has the DBG_HCD_URB bit set, display enqueued URBs in host
 *  mode. */
#define DBG_HCD_URB	(0x800)

/** When debug level has any bit set, display debug messages */
#define DBG_ANY		(0xFF)

/** All debug messages off */
#define DBG_OFF		0

/** Prefix string for DWC_DEBUG print macros. */
#define USB_DWC "DWC_otg: "

/** 
 * Print a debug message when the Global debug level variable contains
 * the bit defined in <code>lvl</code>.
 *
 * @param[in] lvl - Debug level, use one of the DBG_ constants above.
 * @param[in] x - like printf
 *
 *    Example:<p>
 * <code>
 *      DWC_DEBUGPL( DBG_ANY, "%s(%p)\n", __func__, _reg_base_addr);
 * </code>
 * <br>
 * results in:<br> 
 * <code>
 * usb-DWC_otg: dwc_otg_cil_init(ca867000)
 * </code>
 */
#ifdef DWC_OTG_DEBUG

# define DWC_DEBUGPL(lvl, x...) do{ if ((lvl)&g_dbg_lvl)__DWC_DEBUG(USB_DWC x ); }while(0)
# define DWC_DEBUGP(x...)	DWC_DEBUGPL(DBG_ANY, x )

# define CHK_DEBUG_LEVEL(level) ((level) & g_dbg_lvl)

#else

# define DWC_DEBUGPL(lvl, x...) do{}while(0)
# define DWC_DEBUGP(x...)

# define CHK_DEBUG_LEVEL(level) (0)

#endif /*DWC_OTG_DEBUG*/
#endif
