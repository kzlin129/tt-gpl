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

#ifndef DWC_DEVICE_ONLY

/**
 * @file
 *
 * This file contains the implementation of the HCD. In Linux, the HCD
 * implements the hc_driver API.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <asm/io.h>

#ifdef LM_INTERFACE
// @DB: 091023 - Header seems to no be present or needed
/*#include <asm/arch/regs-irq.h>*/
#include <asm/arch/lm.h>
#include <asm/arch/irqs.h>
#endif

#include <linux/usb.h>
#include <../drivers/usb/core/hcd.h>

#include "dwc_otg_hcd.h"
#include "dwc_otg_hcd_if.h"
#include "dwc_otg_dbg.h"
#include "dwc_otg_driver.h"

#include "dwc_otg_pcd.h"
#include <asm/arch/bcm4760_reg.h>
#include <asm/arch/hardware.h>

#if 0
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pmu_bcm59040.h>
#endif

/**
 * Gets the endpoint number from a _bEndpointAddress argument. The endpoint is
 * qualified with its direction (possible 32 endpoints per device).
 */
#define dwc_ep_addr_to_endpoint(_bEndpointAddress_) ((_bEndpointAddress_ & USB_ENDPOINT_NUMBER_MASK) | \
						     ((_bEndpointAddress_ & USB_DIR_IN) != 0) << 4)

static const char dwc_otg_hcd_name[] = "dwc_otg_hcd";

/** @name Linux HC Driver API Functions */
/** @{ */
// @DB: 091023 - Port code transfered
static int urb_enqueue(struct usb_hcd *hcd,
			   /*struct usb_host_endpoint *ep,*/
		       struct urb *urb, gfp_t mem_flags);

// @DB: 091023 - Port code transfered.  Added status
static int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status);

static void endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *ep);

static irqreturn_t dwc_otg_hcd_irq(struct usb_hcd *hcd);
extern int hcd_start(struct usb_hcd *hcd);
extern void hcd_stop(struct usb_hcd *hcd);
static int get_frame_number(struct usb_hcd *hcd);
extern int hub_status_data(struct usb_hcd *hcd, char *buf);
extern int hub_control(struct usb_hcd *hcd,
		       u16 typeReq,
		       u16 wValue, u16 wIndex, char *buf, u16 wLength);

// @KP: Extern the suspend/resume functions
#ifdef CONFIG_PM
extern int dwc_otg_hcd_suspend(struct usb_hcd *hcd);
extern int dwc_otg_hcd_resume(struct usb_hcd *hcd);
#endif



struct wrapper_priv_data {
	dwc_otg_hcd_t *dwc_otg_hcd;
};

/** @} */

static struct hc_driver dwc_otg_hc_driver = {

	.description = dwc_otg_hcd_name,
	.product_desc = "DWC OTG Controller",
	.hcd_priv_size = sizeof(struct wrapper_priv_data),

	.irq = dwc_otg_hcd_irq,

	.flags = HCD_MEMORY | HCD_USB2,

	//.reset =              
	.start = hcd_start,
	//.suspend =            
	//.resume =             
	.stop = hcd_stop,

	.urb_enqueue = urb_enqueue,
	.urb_dequeue = urb_dequeue,
	.endpoint_disable = endpoint_disable,

	.get_frame_number = get_frame_number,

	.hub_status_data = hub_status_data,
	.hub_control = hub_control,

#ifdef CONFIG_PM
    .bus_suspend =	dwc_otg_hcd_suspend,	
    .bus_resume =	dwc_otg_hcd_resume,	
#endif

};

/** Gets the dwc_otg_hcd from a struct usb_hcd */
static inline dwc_otg_hcd_t *hcd_to_dwc_otg_hcd(struct usb_hcd *hcd)
{
	struct wrapper_priv_data *p;
	p = (struct wrapper_priv_data *)(hcd->hcd_priv);
	return p->dwc_otg_hcd;
}

/** Gets the struct usb_hcd that contains a dwc_otg_hcd_t. */
static inline struct usb_hcd *dwc_otg_hcd_to_hcd(dwc_otg_hcd_t * dwc_otg_hcd)
{
	return dwc_otg_hcd_get_priv_data(dwc_otg_hcd);
}

/** Gets the usb_host_endpoint associated with an URB. */
inline struct usb_host_endpoint *dwc_urb_to_endpoint(struct urb *urb)
{
	struct usb_device *dev = urb->dev;
	int ep_num = usb_pipeendpoint(urb->pipe);

	if (usb_pipein(urb->pipe))
		return dev->ep_in[ep_num];
	else
		return dev->ep_out[ep_num];
}

static int _disconnect(dwc_otg_hcd_t * hcd)
{
	struct usb_hcd *usb_hcd = dwc_otg_hcd_to_hcd(hcd);

	usb_hcd->self.is_b_host = 0;
	return 0;
}

static int _start(dwc_otg_hcd_t * hcd)
{
	struct usb_hcd *usb_hcd = dwc_otg_hcd_to_hcd(hcd);

	usb_hcd->self.is_b_host = dwc_otg_hcd_is_b_host(hcd);
	hcd_start(usb_hcd);

	return 0;
}

static int _hub_info(dwc_otg_hcd_t * hcd, void *urb_handle, uint32_t * hub_addr,
		     uint32_t * port_addr)
{
	struct urb *urb = (struct urb *)urb_handle;
	// @DB: 100208 - Make sure that the hub is not NULL
	if (urb->dev->tt && urb->dev->tt->hub) {
		*hub_addr = urb->dev->tt->hub->devnum;
	} else {
		*hub_addr = 0;
	}
	*port_addr = urb->dev->ttport;
	return 0;
}

static int _speed(dwc_otg_hcd_t * hcd, void *urb_handle)
{
	struct urb *urb = (struct urb *)urb_handle;
	return urb->dev->speed;
}

static int _get_b_hnp_enable(dwc_otg_hcd_t * hcd)
{
	struct usb_hcd *usb_hcd = dwc_otg_hcd_to_hcd(hcd);
	return usb_hcd->self.b_hnp_enable;
}

static void allocate_bus_bandwidth(struct usb_hcd *hcd, uint32_t bw,
				   struct urb *urb)
{
	hcd_to_bus(hcd)->bandwidth_allocated += bw / urb->interval;
	if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
		hcd_to_bus(hcd)->bandwidth_isoc_reqs++;
	} else {
		hcd_to_bus(hcd)->bandwidth_int_reqs++;
	}
}

static void free_bus_bandwidth(struct usb_hcd *hcd, uint32_t bw,
			       struct urb *urb)
{
	hcd_to_bus(hcd)->bandwidth_allocated -= bw / urb->interval;
	if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
		hcd_to_bus(hcd)->bandwidth_isoc_reqs--;
	} else {
		hcd_to_bus(hcd)->bandwidth_int_reqs--;
	}
}

/**
 * Sets the final status of an URB and returns it to the device driver. Any
 * required cleanup of the URB is performed.
 */
static int _complete(dwc_otg_hcd_t * hcd, void *urb_handle,
		     dwc_otg_hcd_urb_t * dwc_otg_urb, int32_t status)
{
	struct urb *urb = (struct urb *)urb_handle;

	if (!urb)
	{
		DWC_PRINTF("dwc_otg_hcd_linux: _complete - no urb passed\n");
		return 0;
	}

#ifdef DWC_OTG_DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		DWC_PRINTF("%s: urb %p, device %d, ep %d %s, status=%d\n",
			   __func__, urb, usb_pipedevice(urb->pipe),
			   usb_pipeendpoint(urb->pipe),
			   usb_pipein(urb->pipe) ? "IN" : "OUT", status);
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
			int i;
			for (i = 0; i < urb->number_of_packets; i++) {
				DWC_PRINTF("  ISO Desc %d status: %d\n",
					   i, urb->iso_frame_desc[i].status);
			}
		}
	}
#endif

	urb->actual_length = dwc_otg_hcd_urb_get_actual_length(dwc_otg_urb);
	/* Convert status value. */
	switch (status) {
	case -DWC_E_PROTOCOL:
		status = -EPROTO;
		break;
	case -DWC_E_IN_PROGRESS:
		status = -EINPROGRESS;
		break;
	case -DWC_E_PIPE:
		status = -EPIPE;
		break;
	case -DWC_E_IO:
		status = -EIO;
		break;
	case -DWC_E_TIMEOUT:
		status = -ETIMEDOUT;
		break;
	case -DWC_E_OVERFLOW:
		status = -EOVERFLOW;
		break;
	default:
		if (status) {
			DWC_PRINTF("Uknown urb status %d\n", status);
		}
	}

	if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
		int i;

		urb->error_count = dwc_otg_hcd_urb_get_error_count(dwc_otg_urb);
		for (i = 0; i < urb->number_of_packets; ++i) {
			urb->iso_frame_desc[i].actual_length =
			    dwc_otg_hcd_urb_get_iso_desc_actual_length
			    (dwc_otg_urb, i);
			urb->iso_frame_desc[i].status =
			    dwc_otg_hcd_urb_get_iso_desc_status
			    (dwc_otg_urb, i);
		}
	}

	urb->status = status;
	urb->hcpriv = NULL;
	if (!status) {
		if ((urb->transfer_flags & URB_SHORT_NOT_OK) &&
		    (urb->actual_length < urb->transfer_buffer_length)) {
			urb->status = -EREMOTEIO;
		}
	}

	if ((usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) ||
	    (usb_pipetype(urb->pipe) == PIPE_INTERRUPT)) {
		struct usb_host_endpoint *ep = dwc_urb_to_endpoint(urb);
		if (ep) {
			free_bus_bandwidth(dwc_otg_hcd_to_hcd(hcd),
					   dwc_otg_hcd_get_ep_bandwidth(hcd,
									ep->
									hcpriv),
					   urb);
		}
	}

	dwc_free(dwc_otg_urb);
	// @DB: 091023 - Call with status for new prototype
	usb_hcd_giveback_urb(dwc_otg_hcd_to_hcd(hcd), urb, status);
	return 0;
}

static struct dwc_otg_hcd_function_ops hcd_fops = {
	.start = _start,
	.disconnect = _disconnect,
	.hub_info = _hub_info,
	.speed = _speed,
	.complete = _complete,
	.get_b_hnp_enable = _get_b_hnp_enable,
};

// @DB: 091023 - Port code transfered
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)	// @KP: 090403: dma_mask is a pointer to a dma mask
u64 g_dma_mask_value = DMA_32BIT_MASK;
u64 g_dma_mask_0 = 0x0000000000000000ULL;
#endif

/**
 * Initializes the HCD. This function allocates memory for and initializes the
 * static parts of the usb_hcd and dwc_otg_hcd structures. It also registers the
 * USB bus with the core and calls the hc_driver->start() function. It returns
 * a negative error on failure.
 */
int hcd_init(
#ifdef LM_INTERFACE
	struct lm_device *_dev
#elif  PCI_INTERFACE
	struct pci_dev *_dev
#endif
	)
{
	struct usb_hcd *hcd = NULL;
	dwc_otg_hcd_t *dwc_otg_hcd = NULL;
#ifdef LM_INTERFACE
	dwc_otg_device_t *otg_dev = DWC_OTG_LM_GET_DRVDATA(_dev);
#elif  PCI_INTERFACE
	dwc_otg_device_t *otg_dev = pci_get_drvdata(_dev);
#endif

	int retval = 0;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD INIT\n");

	/* Set device flags indicating whether the HCD supports DMA. */
	if (dwc_otg_is_dma_enable(otg_dev->core_if)) {
#ifdef LM_INTERFACE
// @DB: 091023 - Port code transfered
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
		_dev->dev.dma_mask = (void*)~0;
#else
		// @KP: dma_mask is a pointer to a dma mask
		_dev->dev.dma_mask = &g_dma_mask_value;
#endif
		_dev->dev.coherent_dma_mask = ~0;
#elif  PCI_INTERFACE
		pci_set_dma_mask(_dev,DMA_32BIT_MASK);		
		pci_set_consistent_dma_mask(_dev,DMA_32BIT_MASK);
#endif

	} else {
#ifdef LM_INTERFACE
// @DB: 091023 - Port code transfered
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
		_dev->dev.dma_mask = (void*)0;
#else
		// @KP: dma_mask is a pointer to a dma mask
		_dev->dev.dma_mask = &g_dma_mask_0;
#endif
		_dev->dev.coherent_dma_mask = 0;
#elif  PCI_INTERFACE
		pci_set_dma_mask(_dev,0);		
		pci_set_consistent_dma_mask(_dev,0);
#endif
	}

	/*
	 * Allocate memory for the base HCD plus the DWC OTG HCD.
	 * Initialize the base HCD.
	 */
	hcd = usb_create_hcd(&dwc_otg_hc_driver, &_dev->dev, _dev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto error1;
	}

	hcd->regs = otg_dev->base;

	/* Initialize the DWC OTG HCD. */
	dwc_otg_hcd = dwc_otg_hcd_alloc_hcd();
	if (!dwc_otg_hcd) {
		goto error2;
	}
	((struct wrapper_priv_data *)(hcd->hcd_priv))->dwc_otg_hcd =
	    dwc_otg_hcd;
	otg_dev->hcd = dwc_otg_hcd;

	if (dwc_otg_hcd_init(dwc_otg_hcd, otg_dev->core_if)) {
		goto error3;
	}

	hcd->self.otg_port = dwc_otg_hcd_otg_port(dwc_otg_hcd);

	/*
	 * Finish generic HCD initialization and start the HCD. This function
	 * allocates the DMA buffer pool, registers the USB bus, requests the
	 * IRQ line, and calls hcd_start method.
	 */
	// @DB: 091023 - Changed SA_SHIRQ to be IRQF_SHARED
	retval = usb_add_hcd(hcd, _dev->irq, IRQF_SHARED);
	if (retval < 0) {
		goto error3;
	}

	dwc_otg_hcd_set_priv_data(dwc_otg_hcd, hcd);
	return 0;

error3:
	dwc_free(dwc_otg_hcd);
error2:
	usb_put_hcd(hcd);
error1:
	return retval;
}

/**
 * Removes the HCD.
 * Frees memory and resources associated with the HCD and deregisters the bus.
 */
void hcd_remove(
#ifdef LM_INTERFACE
	struct lm_device *_dev
#elif  PCI_INTERFACE
	struct pci_dev *_dev
#endif
	)
{
#ifdef LM_INTERFACE
	dwc_otg_device_t *otg_dev = DWC_OTG_LM_GET_DRVDATA(_dev);
#elif  PCI_INTERFACE
	dwc_otg_device_t *otg_dev = pci_get_drvdata(_dev);
#endif

	dwc_otg_hcd_t *dwc_otg_hcd;
	struct usb_hcd *hcd;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD REMOVE\n");

	if (!otg_dev) {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev NULL!\n", __func__);
		return;
	}

	dwc_otg_hcd = otg_dev->hcd;

	if (!dwc_otg_hcd) {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->hcd NULL!\n", __func__);
		return;
	}

	hcd = dwc_otg_hcd_to_hcd(dwc_otg_hcd);

	if (!hcd) {
		DWC_DEBUGPL(DBG_ANY,
			    "%s: dwc_otg_hcd_to_hcd(dwc_otg_hcd) NULL!\n",
			    __func__);
		return;
	}
	usb_remove_hcd(hcd);
	dwc_otg_hcd_set_priv_data(dwc_otg_hcd, NULL);
	dwc_otg_hcd_remove(dwc_otg_hcd);
	usb_put_hcd(hcd);
}

/* =========================================================================
 *  Linux HC Driver Functions
 * ========================================================================= */

/** Initializes the DWC_otg controller and its root hub and prepares it for host
 * mode operation. Activates the root port. Returns 0 on success and a negative
 * error code on failure. */
int hcd_start(struct usb_hcd *hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);
	struct usb_bus *bus;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD START\n");
	bus = hcd_to_bus(hcd);

	hcd->state = HC_STATE_RUNNING;
	if (dwc_otg_hcd_start(dwc_otg_hcd, &hcd_fops)) {
		return 0;
	}

	/* Initialize and connect root hub if one is not already attached */
	if (bus->root_hub) {
		DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD Has Root Hub\n");
		/* Inform the HUB driver to resume. */
		usb_hcd_resume_root_hub(hcd);
	}

	return 0;
}

/**
 * Halts the DWC_otg host mode operations in a clean manner. USB transfers are
 * stopped.
 */
void hcd_stop(struct usb_hcd *hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);

	dwc_otg_hcd_stop(dwc_otg_hcd);
}

/** Returns the current frame number. */
static int get_frame_number(struct usb_hcd *hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);

	return dwc_otg_hcd_get_frame_number(dwc_otg_hcd);
}

#ifdef DWC_OTG_DEBUG
static void dump_urb_info(struct urb *urb, char *fn_name)
{
	DWC_PRINTF("%s, urb %p\n", fn_name, urb);
	DWC_PRINTF("  Device address: %d\n", usb_pipedevice(urb->pipe));
	DWC_PRINTF("  Endpoint: %d, %s\n", usb_pipeendpoint(urb->pipe),
		   (usb_pipein(urb->pipe) ? "IN" : "OUT"));
	DWC_PRINTF("  Endpoint type: %s\n", ( {
					     char *pipetype;
					     switch (usb_pipetype(urb->pipe)) {
case PIPE_CONTROL:
pipetype = "CONTROL"; break; case PIPE_BULK:
pipetype = "BULK"; break; case PIPE_INTERRUPT:
pipetype = "INTERRUPT"; break; case PIPE_ISOCHRONOUS:
pipetype = "ISOCHRONOUS"; break; default:
					     pipetype = "UNKNOWN"; break;};
					     pipetype;}
		   )) ;
	DWC_PRINTF("  Speed: %s\n", ( {
				     char *speed; switch (urb->dev->speed) {
case USB_SPEED_HIGH:
speed = "HIGH"; break; case USB_SPEED_FULL:
speed = "FULL"; break; case USB_SPEED_LOW:
speed = "LOW"; break; default:
				     speed = "UNKNOWN"; break;};
				     speed;}
		   )) ;
	DWC_PRINTF("  Max packet size: %d\n",
		   usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));
	DWC_PRINTF("  Data buffer length: %d\n", urb->transfer_buffer_length);
	DWC_PRINTF("  Transfer buffer: %p, Transfer DMA: %p\n",
		   urb->transfer_buffer, (void *)urb->transfer_dma);
	DWC_PRINTF("  Setup buffer: %p, Setup DMA: %p\n",
		   urb->setup_packet, (void *)urb->setup_dma);
	DWC_PRINTF("  Interval: %d\n", urb->interval);
	if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
		int i;
		for (i = 0; i < urb->number_of_packets; i++) {
			DWC_PRINTF("  ISO Desc %d:\n", i);
			DWC_PRINTF("    offset: %d, length %d\n",
				   urb->iso_frame_desc[i].offset,
				   urb->iso_frame_desc[i].length);
		}
	}
}

#endif

/** Starts processing a USB transfer request specified by a USB Request Block
 * (URB). mem_flags indicates the type of memory allocation to use while
 * processing this URB. */
// @DB: 091023 - Removed ep, change in api
static int urb_enqueue(struct usb_hcd *hcd,
		       struct urb *urb, gfp_t mem_flags)
{
	int retval = 0;
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);
	dwc_otg_hcd_urb_t *dwc_otg_urb;
	int i;
	int alloc_bandwidth = 0;
	uint8_t ep_type = 0;
	uint32_t flags = 0;
	void *buf;
	// @DB: 091210 - Get EP from urb
	struct usb_host_endpoint* ep = dwc_urb_to_endpoint(urb);

#ifdef DWC_OTG_DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		dump_urb_info(urb, "urb_enqueue");
	}
#endif

	if ((usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS)
	    || (usb_pipetype(urb->pipe) == PIPE_INTERRUPT)) {
		if (!dwc_otg_hcd_is_bandwidth_allocated
		    (dwc_otg_hcd, &ep->hcpriv)) {
			alloc_bandwidth = 1;
		}
	}

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		ep_type = USB_ENDPOINT_XFER_CONTROL;
		break;
	case PIPE_ISOCHRONOUS:
		ep_type = USB_ENDPOINT_XFER_ISOC;
		break;
	case PIPE_BULK:
		ep_type = USB_ENDPOINT_XFER_BULK;
		break;
	case PIPE_INTERRUPT:
		ep_type = USB_ENDPOINT_XFER_INT;
		break;
	default:
		DWC_WARN("Wrong ep type\n");
	}

	dwc_otg_urb = dwc_otg_hcd_urb_alloc(dwc_otg_hcd,
					    urb->number_of_packets,
					    mem_flags == GFP_ATOMIC ? 1 : 0);

	dwc_otg_hcd_urb_set_pipeinfo(dwc_otg_urb, usb_pipedevice(urb->pipe),
				     usb_pipeendpoint(urb->pipe), ep_type,
				     usb_pipein(urb->pipe),
				     usb_maxpacket(urb->dev, urb->pipe,
						   !(usb_pipein(urb->pipe))));

	buf = urb->transfer_buffer;
	if (hcd->self.uses_dma) {
		/*
		 * Calculate virtual address from physical address,
		 * because some class driver may not fill transfer_buffer.
		 * In Buffer DMA mode virual address is used,
		 * when handling non DWORD aligned buffers.
		 */
		buf = phys_to_virt(urb->transfer_dma);
	}
	
	if (!(urb->transfer_flags & URB_NO_INTERRUPT))
		flags |= URB_GIVEBACK_ASAP;
	if (urb->transfer_flags & URB_ZERO_PACKET)
		flags |= URB_SEND_ZERO_PACKET;

	dwc_otg_hcd_urb_set_params(dwc_otg_urb, urb, buf,
				   urb->transfer_dma,
				   urb->transfer_buffer_length,
				   urb->setup_packet, 
				   urb->setup_dma,
				   flags,
				   urb->interval);

	for (i = 0; i < urb->number_of_packets; ++i) {
		dwc_otg_hcd_urb_set_iso_desc_params(dwc_otg_urb, i,
						    urb->iso_frame_desc[i].
						    offset,
						    urb->iso_frame_desc[i].
						    length);
	}

	urb->hcpriv = dwc_otg_urb;
	retval = dwc_otg_hcd_urb_enqueue(dwc_otg_hcd, dwc_otg_urb, &ep->hcpriv);
	if (!retval) {
		if (alloc_bandwidth) {
			allocate_bus_bandwidth(hcd,
					       dwc_otg_hcd_get_ep_bandwidth
					       (dwc_otg_hcd, ep->hcpriv), urb);
		}
	} else {
		if (retval == -DWC_E_NO_DEVICE) {
			retval = -ENODEV;
		}
	}

	return retval;
}

/** Aborts/cancels a USB transfer request. Always returns 0 to indicate
 * success.  */
// @DB: 091023 - Added status to pass on to give back routine
static int urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	dwc_otg_hcd_t *dwc_otg_hcd;
	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD URB Dequeue\n");

	dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);

#ifdef DWC_OTG_DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		dump_urb_info(urb, "urb_dequeue");
	}
#endif
	dwc_otg_hcd_urb_dequeue(dwc_otg_hcd, urb->hcpriv);

	dwc_free(urb->hcpriv);
	urb->hcpriv = NULL;

	/* Higher layer software sets URB status. */
	// @DB: 091023 - Include status for new prototype
	usb_hcd_giveback_urb(hcd, urb, status);
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		DWC_PRINTF("Called usb_hcd_giveback_urb()\n");
		DWC_PRINTF("  urb->status = %d status = %d\n", urb->status, status);
	}

	return 0;
}

/* Frees resources in the DWC_otg controller related to a given endpoint. Also
 * clears state in the HCD related to the endpoint. Any URBs for the endpoint
 * must already be dequeued. */
static void endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);

	DWC_DEBUGPL(DBG_HCD,
		    "DWC OTG HCD EP DISABLE: _bEndpointAddress=0x%02x, "
		    "endpoint=%d\n", ep->desc.bEndpointAddress,
		    dwc_ep_addr_to_endpoint(ep->desc.bEndpointAddress));
	dwc_otg_hcd_endpoint_disable(dwc_otg_hcd, ep->hcpriv, 250);
	ep->hcpriv = NULL;
}

/** Handles host mode interrupts for the DWC_otg controller. Returns IRQ_NONE if
 * there was no interrupt to handle. Returns IRQ_HANDLED if there was a valid
 * interrupt.
 *
 * This function is called by the USB core when an interrupt occurs */
static irqreturn_t dwc_otg_hcd_irq(struct usb_hcd *hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);
	int32_t retval = dwc_otg_hcd_handle_intr(dwc_otg_hcd);
	return IRQ_RETVAL(retval);
}

/** Creates Status Change bitmap for the root hub and root port. The bitmap is
 * returned in buf. Bit 0 is the status change indicator for the root hub. Bit 1
 * is the status change indicator for the single root port. Returns 1 if either
 * change indicator is 1, otherwise returns 0. */
int hub_status_data(struct usb_hcd *hcd, char *buf)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);

	buf[0] = 0;
	buf[0] |= (dwc_otg_hcd_is_status_changed(dwc_otg_hcd, 1)) << 1;

	return (buf[0] != 0);
}

/** Handles hub class-specific requests. */
int hub_control(struct usb_hcd *hcd,
		u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	int retval;

	retval = dwc_otg_hcd_hub_control(hcd_to_dwc_otg_hcd(hcd),
					 typeReq, wValue, wIndex, buf, wLength);

	switch (retval) {
	case -DWC_E_INVALID:
		retval = -EINVAL;
		break;
	}

	return retval;
}


// @DB: 100128 - Add suspend/resume functionality back in from KP code
#ifdef CONFIG_PM
int dwc_otg_hcd_suspend(struct usb_hcd *_hcd)
{
	dwc_otg_device_t *otg_dev = DWC_OTG_LM_GET_DRVDATA(NULL);

	DWC_DEBUGPL(DBG_HCDV, "%s(_hcd=0x%p)\n", __func__, _hcd);

	_hcd->state = HC_STATE_SUSPENDED;

	// clean up active and queued request and notify gadget driver
	// of device disconnect
	dwc_otg_pcd_stop( otg_dev->pcd );

#if 0
#ifdef TOMTOM_LEGACY_MODE
	DWC_DEBUGPL(DBG_USR,
				"DWC_USB_CHARGER_TYPE_%s: set current limit to 500\n",
				(g_usb_charger_type == DWC_USB_CHARGER_TYPE_USB)? "USB":"DC");
	pm_submit_event(PM_EVENT_PMU_CHIP,
					PMU_EVENT_CHARGER_SET_CURR_LMT,
					(uint32_t)BCM59040_CHARGER_USB,
					500);
#else
    DWC_DEBUGPL(DBG_USR,
				"DWC_USB_CHARGER_TYPE_USB: set current limit to 100\n" );
    pm_submit_event(PM_EVENT_PMU_CHIP,
					PMU_EVENT_CHARGER_SET_CURR_LMT,
					(uint32_t)BCM59040_CHARGER_USB,
					100);
#endif
#endif

    // soft disconnect from host
	{
		uint32_t dctl = readl(IO_ADDRESS(USB_R_DCTL_MEMADDR));

		dctl |= USB_F_SFTDISCON_MASK;
		writel(dctl, IO_ADDRESS(USB_R_DCTL_MEMADDR));
		DWC_MDELAY(5);
	}
	dwc_otg_disable_global_interrupts(otg_dev->core_if);

	/* Clear any pending OTG Interrupts */
	writel(0xFFFFFFFF, IO_ADDRESS(USB_R_GOTGINT_MEMADDR));

	/* Clear all pending Device Interrupts */
	writel(0xFFFFFFFF, IO_ADDRESS(USB_R_DAINT_MEMADDR));

	/* Clear any pending interrupts */
	writel(0xFFFFFFFF, IO_ADDRESS(USB_R_GINTSTS_MEMADDR));

	/* turn off the USB terminators; there's no PHY driver so need to do this here */
	writel(readl(IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR)) | 0x00001000,
			IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR));

	return 0;
}

int dwc_otg_hcd_resume(struct usb_hcd *_hcd)
{
	dwc_otg_device_t *otg_dev = DWC_OTG_LM_GET_DRVDATA(NULL);		

	DWC_DEBUGPL(DBG_HCDV, "%s(_hcd=0x%p)\n", __func__, _hcd);

	_hcd->state = HC_STATE_RESUMING;

	// Initialize the DWC_otg core.
	GET_CORE_IF(otg_dev->pcd)->core_params->speed = DWC_SPEED_PARAM_HIGH;
	dwc_otg_core_init(GET_CORE_IF(otg_dev->pcd));

	/* Initialize EP structures */
	dwc_otg_pcd_reinit(otg_dev->pcd);

	// config usb broadcom phy (8-bit) for otg mode (default to device mode);
	// reset and unreset pair; reg 0xb0244
	writel(0x004069d5, IO_ADDRESS(CMU_R_USB_PHY0_MEMADDR));
	writel(0x0041a9d5, IO_ADDRESS(CMU_R_USB_PHY0_MEMADDR));

	// Connect back to the USB bus in an orderly manner
	{	
		uint32_t usb_phy1;
		uint32_t dctl = readl(IO_ADDRESS(USB_R_DCTL_MEMADDR));
		
		// soft disconnect from host
		if ((dctl & USB_F_SFTDISCON_MASK) == 0)
		{
			dctl |= USB_F_SFTDISCON_MASK;
			writel(dctl, IO_ADDRESS(USB_R_DCTL_MEMADDR));
			DWC_MDELAY(5);
		}

		// enable DP/DN terminators
		usb_phy1 = readl(IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR));
		if (usb_phy1 & 0x00001000)
		{
			usb_phy1 &= ~0x00001000;
			writel(usb_phy1, IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR));
		}

		// simulate reconnect event to host
		dctl &= ~USB_F_SFTDISCON_MASK;
		writel(dctl, IO_ADDRESS(USB_R_DCTL_MEMADDR));
	}

	dwc_otg_enable_global_interrupts(otg_dev->core_if);

	return 0;
}
#endif  /* CONFIG_PM */
#endif				/* DWC_DEVICE_ONLY */