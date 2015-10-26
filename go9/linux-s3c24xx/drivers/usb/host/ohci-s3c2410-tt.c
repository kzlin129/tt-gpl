/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * USB Bus Glue for Samsung S3C2410
 *
 * Written by Christopher Hoover <ch@hpl.hp.com>
 * Based on fragments of previous driver by Rusell King et al.
 *
 * Modified for S3C2410 from ohci-sa1111.c, ohci-omap.c and ohci-lh7a40.c
 *	by Ben Dooks, <ben@simtec.co.uk>
 *	Copyright (C) 2004 Simtec Electronics
 *
 * This file is licenced under the GPL.
*/

#define DEBUG

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>
#include <asm/arch/usb-control.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#ifdef CONFIG_MACH_TOMTOMGO
	#include <barcelona/gopins.h>
#endif

#ifdef CONFIG_S3C2410
#error "This file is S3C2410 bus glue. CONFIG_S3C2410 must be set"
#endif

#define valid_port(idx) ((idx) == 1 || (idx) == 2)

static struct clk *clk;

/* conversion functions */

struct s3c2410_hcd_info *
to_s3c2410_info(struct usb_hcd *hcd)
{
	return (struct s3c2410_hcd_info *)hcd->self.controller->platform_data;
}

static void s3c2410_start_hc(struct platform_device *dev)
{
	unsigned int i;
	
	i = __raw_readl(S3C2410_CLKCON);
	i|= S3C2410_CLKCON_USBH;
	__raw_writel(i, S3C2410_CLKCON);

	i = __raw_readl(S3C2410_MISCCR);
#if defined CONFIG_MACH_TOMTOMGO && !defined CONFIG_SMDK2440_BOARD
	i |= S3C2410_MISCCR_USBHOST|S3C2410_MISCCR_USBSUSPND0;
	i &= ~S3C2410_MISCCR_USBSUSPND1;
#else /* CONFIG_MACH_TOMTOMGO && !CONFIG_SMDK2440_BOARD */
	i |= S3C2410_MISCCR_USBHOST;
	i &= ~(S3C2410_MISCCR_USBSUSPND0|S3C2410_MISCCR_USBSUSPND1);
#endif /* CONFIG_MACH_TOMTOMGO && !CONFIG_SMDK2440_BOARD */
	__raw_writel(i, S3C2410_MISCCR);
	dev_dbg(&dev->dev, "s3c2410_start_hc:\n");
	clk_enable(clk);
}

static void s3c2410_stop_hc(struct platform_device *dev)
{
	dev_dbg(&dev->dev, "s3c2410_stop_hc:\n");
	clk_disable(clk);
}

/* ohci_s3c2410_hub_status_data
 *
 * update the status data from the hub with anything that
 * has been detected by our system
*/

static int
ohci_s3c2410_hub_status_data (struct usb_hcd *hcd, char *buf)
{
	struct s3c2410_hcd_info *info = to_s3c2410_info(hcd);
	struct s3c2410_hcd_port *port;
	int orig; 
	int portno;

	orig  = ohci_hub_status_data (hcd, buf);

	if (info == NULL)
		return orig;

	port = &info->port[0];

	/* mark any changed port as changed */

	for (portno = 0; portno < 2; port++, portno++) {
		if (port->oc_changed == 1 &&
		    port->flags & S3C_HCDFLG_USED) {
			dev_dbg(hcd->self.controller, 
				"oc change on port %d\n", portno);

			if (orig < 1)
				orig = 1;

			buf[0] |= 1<<(portno+1);
		}
	}
	
	return orig;
}

/* s3c2410_usb_set_power
 *
 * configure the power on a port, by calling the platform device
 * routine registered with the platfrom device
*/

static void s3c2410_usb_set_power(struct s3c2410_hcd_info *info,
				  int port, int to)
{
	if (info == NULL)
		return;

	if (info->power_control != NULL) {
		info->port[port-1].power = to;
		(info->power_control)(port, to);
	}
}

/* ohci_s3c2410_hub_control
 *
 * look at control requests to the hub, and see if we need
 * to take any action or over-ride the results from the
 * request. 
*/

static int ohci_s3c2410_hub_control (
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	struct s3c2410_hcd_info *info = to_s3c2410_info(hcd);
	int ret;

	dev_dbg(hcd->self.controller, 
		"s3c2410_hub_control(%p,0x%04x,0x%04x,0x%04x,%p,%04x)\n",
		 hcd, typeReq, wValue, wIndex, buf, wLength);

	if (info != NULL) {
		if (typeReq == SetPortFeature &&
		    wValue == USB_PORT_FEAT_POWER) {
			dev_dbg(hcd->self.controller, "SetPortFeat: POWER\n");
			s3c2410_usb_set_power(info, wIndex, 1);
			return 0;
		}

		if (typeReq == ClearPortFeature) {
			switch (wValue) {
			case USB_PORT_FEAT_C_OVER_CURRENT:
				dev_dbg(hcd->self.controller,
					"ClearPortFeature: C_OVER_CURRENT\n");

				if (valid_port(wIndex)) {
					info->port[wIndex-1].oc_changed = 0;
					info->port[wIndex-1].oc_status = 0;
				}
				return 0;

			case USB_PORT_FEAT_OVER_CURRENT:
				dev_dbg(hcd->self.controller,
					"ClearPortFeature: OVER_CURRENT\n");
				
				if (valid_port(wIndex)) {
					info->port[wIndex-1].oc_status = 0;
				}

				return 0;

			case USB_PORT_FEAT_POWER:
				dev_dbg(hcd->self.controller,
					"ClearPortFeature: POWER\n");

				if (valid_port(wIndex)) {
					s3c2410_usb_set_power(info, wIndex, 0);
					return 0;
				}
			}
		}
	}

	ret = ohci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

	if (info == NULL)
		return ret;

	if (typeReq == GetHubDescriptor) {
		struct usb_hub_descriptor *desc;

		/* update the hub's descriptor */

		desc = (struct usb_hub_descriptor *)buf;

		if (info->power_control == NULL)
			return ret;

		dev_dbg(hcd->self.controller, "wHubCharacteristics 0x%04x\n",
			desc->wHubCharacteristics);

		/* remove the old configurations for power-switching, and
		 * over-current protection, and insert our new configuration
		 */

		desc->wHubCharacteristics &= ~cpu_to_le16(HUB_CHAR_LPSM);
		desc->wHubCharacteristics |= cpu_to_le16(0x0001);

		if (info->enable_oc) {
			desc->wHubCharacteristics &= ~cpu_to_le16(HUB_CHAR_OCPM);
			desc->wHubCharacteristics |=  cpu_to_le16(0x0008|0x0001);
		}

		dev_dbg(hcd->self.controller, "wHubCharacteristics after 0x%04x\n",
			desc->wHubCharacteristics);

		return ret;
	}

	if (typeReq == GetPortStatus) {
		u32 *data = (u32 *)buf;
		u32 before = *data;
		/* check port status */
		
		dev_dbg(hcd->self.controller, "GetPortStatus(%d)\n", wIndex);

		if (valid_port(wIndex)) {
			dev_dbg(hcd->self.controller,
				"GetPortStatus(%d) oc=%d, changed=%d\n",
				wIndex, info->port[wIndex-1].oc_status,
				info->port[wIndex-1].oc_changed);

			if (info->port[wIndex-1].oc_changed) {
				*data |= cpu_to_le32(RH_PS_OCIC);
			}

			if (info->port[wIndex-1].oc_status) {
				*data |= cpu_to_le32(RH_PS_POCI);
			}

			dev_dbg(hcd->self.controller, "GetPortStatus(%d) 0x%x => 0x%x\n",
				wIndex, before, *data);
		}
	}

	return ret;
}


static irqreturn_t s3c2410_hc_irq(int irq, void *param, struct pt_regs * r)
{
	struct usb_hcd *hcd = (struct usb_hcd *)param;

	usb_hcd_irq(irq, hcd, r);
	return IRQ_HANDLED;
}

/* s3c2410_hcd_oc
 *
 * handle an over-current report
*/

static void s3c2410_hcd_oc(struct s3c2410_hcd_info *info, int port_oc)
{
	struct s3c2410_hcd_port *port;
	struct usb_hcd *hcd;
	unsigned long flags;
	int portno;

	if (info == NULL)
		return;

	port = &info->port[0];
	hcd = info->hcd;

	local_irq_save(flags);

	for (portno = 0; portno < 2; port++, portno++) {
		if (port_oc & (1<<portno) &&
		    port->flags & S3C_HCDFLG_USED) {
			port->oc_status = 1;
			port->oc_changed = 1;
			
			/* ok, once over-current is detected,
			   the port needs to be powered down */
			s3c2410_usb_set_power(info, portno+1, 0);
		}
	}

	local_irq_restore(flags);
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/*
 * usb_hcd_s3c2410_remove - shutdown processing for HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_3c2410_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
*/

void usb_hcd_s3c2410_remove (struct usb_hcd *hcd, struct platform_device *dev)
{
	void *base;

	dev_info(&dev->dev, "remove: %s, state %x",
		 hcd->self.bus_name, hcd->state);

	if (in_interrupt ())
		BUG ();

	hcd->state = USB_STATE_QUIESCING;	
	usb_disconnect (&hcd->self.root_hub);

	hcd->driver->stop (hcd);
	hcd->state = USB_STATE_HALT;

	free_irq (hcd->irq, hcd);
	hcd_buffer_destroy (hcd);

	usb_deregister_bus (&hcd->self);

	base = hcd->regs;

	s3c2410_stop_hc(dev);
	release_mem_region(dev->resource[0].start,
			   dev->resource[0].end
			   - dev->resource[0].start + 1);
}

/**
 * usb_hcd_s3c2410_probe - initialize S3C2410-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_hcd_s3c2410_probe (const struct hc_driver *driver,
			  struct usb_hcd **hcd_out,
			  struct platform_device *dev)
{
	struct s3c2410_hcd_info *info;
	int retval;
	struct usb_hcd *hcd = 0;
	unsigned int *addr = NULL;

	dev_dbg(&dev->dev, "s3c2410_probe in.\n");

	if (!request_mem_region(dev->resource[0].start,
				dev->resource[0].end
				- dev->resource[0].start + 1, hcd_name)) {
		dev_err(&dev->dev, "request_mem_region failed");
		return -EBUSY;
	}

	clk = clk_get(NULL, "usb-host");
	clk_use(clk);
	
	s3c2410_start_hc(dev);

	info = (struct s3c2410_hcd_info *)dev->dev.platform_data;

	addr = ioremap(dev->resource[0].start,
		       dev->resource[0].end
		       - dev->resource[0].start + 1);
	if (!addr) {
		dev_err(&dev->dev, "ioremap failed\n");
		retval = -ENOMEM;
		goto err1;
	}
	

	hcd = driver->hcd_alloc ();
	if (hcd == NULL){
		dev_err(&dev->dev, "hcd_alloc() failed\n");
		retval = -ENOMEM;
		goto err1;
	}

	hcd->irq = platform_get_irq(dev, 0);
	if (hcd->irq <= 0) {
		dev_err(&dev->dev, "no IRQ specified\n");
		retval = -ENOMEM;
		goto err1;
	}

	hcd->driver = (struct hc_driver *) driver;
	hcd->description = driver->description;
	hcd->irq = dev->resource[1].start;
	hcd->regs = addr;
	hcd->self.controller = &dev->dev;

	retval = hcd_buffer_create (hcd);
	if (retval != 0) {
		dev_err(&dev->dev, "hcd_buffer_create() failed\n");
		goto err1;
	}

	retval = request_irq (hcd->irq, s3c2410_hc_irq, SA_INTERRUPT,
			      hcd->description, hcd);
	if (retval != 0) {
		dev_err(&dev->dev, "cannot request irq %d\n", hcd->irq);
		retval = -EBUSY;
		goto err2;
	}

	dev_info (&dev->dev, "%s (S3C2410) at 0x%p, irq %d\n",
		  hcd->description, hcd->regs, hcd->irq);

#ifdef CONFIG_MACH_TOMTOMGO
	IO_Deactivate(USB_PULL_EN);
#endif


	usb_bus_init (&hcd->self);
	hcd->self.op = &usb_hcd_operations;
	hcd->self.hcpriv = (void *) hcd;
	hcd->self.bus_name = "s3c2410";
	hcd->product_desc = "S3C2410 OHCI";

	INIT_LIST_HEAD (&hcd->dev_list);

	usb_register_bus (&hcd->self);

	if (info != NULL) {
		info->hcd= hcd;
		info->report_oc = s3c2410_hcd_oc;

		if (info->enable_oc != NULL) {
			(info->enable_oc)(info, 1);
		}
	}

	if ((retval = driver->start (hcd)) < 0)
	{
		usb_hcd_s3c2410_remove(hcd, dev);
		return retval;
	}

	dev_dbg(&dev->dev, "s3c2410_probe OK.\n");

	*hcd_out = hcd;
	return 0;

 err2:
	hcd_buffer_destroy (hcd);
 err1:
	if (info != NULL) {
		info->report_oc = NULL;
		info->hcd = NULL;

		if (info->enable_oc != NULL) {
			(info->enable_oc)(info, 0);
		}
	}

	s3c2410_stop_hc(dev);
	release_mem_region(dev->resource[0].start,
			   dev->resource[0].end - dev->resource[0].start + 1);
	return retval;
}

/*-------------------------------------------------------------------------*/

static int
ohci_s3c2410_start (struct usb_hcd *hcd)
{
	struct s3c2410_hcd_info *info;
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	info = to_s3c2410_info(hcd);

	ohci->hcca = dma_alloc_coherent (hcd->self.controller,
					 sizeof *ohci->hcca, &ohci->hcca_dma,
					 0);
	if (!ohci->hcca)
		return -ENOMEM;

        memset (ohci->hcca, 0, sizeof (struct ohci_hcca));
	if ((ret = ohci_mem_init (ohci)) < 0) {
		ohci_stop (hcd);
		return ret;
	}
	ohci->regs = hcd->regs;

	if (ohci_init (ohci) < 0) {
		ohci_stop (hcd);
		return -ENODEV;
	}
	
	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", ohci->hcd.self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

/*
	if (ohci_restart (ohci) < 0) {
		err ("can't start %s", ohci->hcd.self.bus_name);
		ohci_stop (hcd);
		return -EBUSY;
	}
*/
	create_debug_files (ohci);

#ifdef	DEBUG
	ohci_dump (ohci, 1);
#endif
	return 0;
}


static const struct hc_driver ohci_s3c2410_hc_driver = {
	.description =		hcd_name,

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_s3c2410_start,
	.stop =			ohci_stop,

	/*
	 * memory lifecycle (except per-request)
	 */
	.hcd_alloc =		ohci_hcd_alloc,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_s3c2410_hub_status_data,
	.hub_control =		ohci_s3c2410_hub_control,
#if defined(CONFIG_USB_SUSPEND) && 0
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/* device driver */

static int ohci_hcd_s3c2410_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = NULL;
	int ret;

	ret = usb_hcd_s3c2410_probe(&ohci_s3c2410_hc_driver, &hcd, pdev);
	if (ret == 0)
		dev_set_drvdata(dev, hcd);

	return ret;
}


static int ohci_hcd_s3c2410_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	usb_hcd_s3c2410_remove(hcd, pdev);
	dev_set_drvdata(dev, NULL);
	return 0;
}

static struct device_driver ohci_hcd_s3c2410_driver = {
	.name		= "s3c2410-ohci",
	.bus		= &platform_bus_type,
	.probe		= ohci_hcd_s3c2410_drv_probe,
	.remove		= ohci_hcd_s3c2410_drv_remove,
	/*.suspend	= ohci_hcd_s3c2410_drv_suspend, */
	/*.resume	= ohci_hcd_s3c2410_drv_resume, */
};

static int __init ohci_hcd_s3c2410_init (void)
{
	return driver_register(&ohci_hcd_s3c2410_driver);
}

static void __exit ohci_hcd_s3c2410_cleanup (void)
{
	driver_unregister(&ohci_hcd_s3c2410_driver);
}

module_init (ohci_hcd_s3c2410_init);
module_exit (ohci_hcd_s3c2410_cleanup);
