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
 * Thanks to basprog@mail.ru for updates to newer kernels
 *
 * This file is licenced under the GPL.
*/

#include <linux/kernel.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>
#include <asm/arch/usb-control.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-udc.h>
#ifdef CONFIG_MACH_TOMTOMGO
	#include <barcelona/gopins.h>
#endif
#include <linux/cpufreq.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;
#endif

/* from s3c24xx_usbmode.c */
extern void s3c2443_resurrect_phy(void);

#define PFX " ohci-s3c2410: "
#define PK_DBG PK_DBG_FUNC

// #define valid_port(idx) ((idx) == 1 || (idx) == 2)

/* clock device associated with the hcd */

static struct clk *clk;

/* forward definitions */

static void s3c2410_hcd_oc(struct s3c2410_hcd_info *info, int port_oc);
static void s3c2410_tomtom_init(void);
static int valid_port(int idx);

/* conversion functions */

struct s3c2410_hcd_info *to_s3c2410_info(struct usb_hcd *hcd)
{
	return hcd->self.controller->platform_data;
}

static int valid_port(int idx)
{
	return (idx == 2) ? 1 : 0;
}

static void s3c2410_tomtom_init()
{
	unsigned long int i;
	
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
	if ((IO_GetCpuType() == GOCPU_S3C2443 || IO_GetCpuType() == GOCPU_S3C2450) && (IO_UsbOhciPortMask() & 0x01)) {
		unsigned long flags;
		/* Enable the power. If not enabled, HC will not work ! DUH */
		IO_Activate( USB_PHY_PWR_EN );
		IO_Activate( USB_PHY_1V2_PWR_EN );

		local_irq_save(flags);
		 __raw_writel(__raw_readl(S3C2443_PWRCFG) | S3C2443_PWRCFG_nSW_PHY_OFF_USB,S3C2443_PWRCFG);
		
		/*use  48 Mhz freq , use USB crystal for PLL, set mode to host */
		i =  S3C2443_PHYCTRL_CLK_SEL_48MHZ | S3C2443_PHYCTRL_INT_PLL_SEL | S3C2443_PHYCTRL_DOWNSTREAM_PORT ;
		
		__raw_writel(i, S3C2443_PHYCTRL);
		
		i = __raw_readl(S3C2443_PHYPWR);
		i &= ~(S3C2443_PHYPWR_COMMON_ON_N | S3C2443_PHYPWR_ANALOG_POWERDOWN_MASK | S3C2443_PHYPWR_PLL_REF_CLK | S3C2443_PHYPWR_XO_ON | S3C2443_PHYPWR_PLL_POWERDOWN  | S3C2443_PHYPWR_FORCE_SUSPEND);
		/* turn on USB2.0 PHY (connects also to USB 1.1 host, XO on, XO off during suspend, no USB suspend */
		i |= S3C2443_PHYPWR_ANALOG_POWERDOWN_UP | S3C2443_PHYPWR_XO_ON;
		
		__raw_writel(i, S3C2443_PHYPWR);
		
		mdelay(1); /* wait until crystal is stable */

		/* reset host controller and phy for at least 10 usec */
		i = __raw_readl(S3C2443_URSTCON);
		i |= S3C2443_URSTCON_HOST_RESET | S3C2443_URSTCON_PHY_RESET;
		__raw_writel(i, S3C2443_URSTCON);
		udelay(10);
		mdelay(1);
		i &= ~(S3C2443_URSTCON_HOST_RESET | S3C2443_URSTCON_PHY_RESET);
		__raw_writel(i, S3C2443_URSTCON);
		
		/* enable USBH clock UCLKCON */
		i = __raw_readl(S3C2443_UCLKCON);
		i &= ~(S3C2443_UCLKCON_DETECT_VBUS| S3C2443_UCLKCON_HOST_CLK_TEST | S3C2443_UCLKCON_FUNC_CLK_EN | S3C2443_UCLKCON_HOST_CLK_EN | S3C2443_UCLKCON_TCLK_EN);
		i |= S3C2443_UCLKCON_HOST_CLK_EN | S3C2443_UCLKCON_HOST_CLK_TEST;
		__raw_writel(i, S3C2443_UCLKCON);
		
		/* route USB PHY DP/DN pins to host controller */
		i = __raw_readl(S3C2443_PHYCTRL);
		i &= ~(S3C2443_PHYCTRL_CLK_SEL_MASK | S3C2443_PHYCTRL_EXT_CLK | S3C2443_PHYCTRL_INT_PLL_SEL | S3C2443_PHYCTRL_DOWNSTREAM_PORT);
		//i |= S3C2443_PHYCTRL_DOWNSTREAM_PORT | S3C2443_PHYCTRL_INT_PLL_SEL;
		i |= S3C2443_PHYCTRL_INT_PLL_SEL;
		__raw_writel(i, S3C2443_PHYCTRL);

		local_irq_restore(flags);
	} else
#endif /* CONFIG_CPU_S3C2443 */
	{
		/* 2410/2440/2442/2412 */
		if (IO_GetCpuType() == GOCPU_S3C2412){
			/* tomtomgo_init configures the upll clock to 24 Mhz and
			use a different divider in this code we reprogram the
			clocks for the 4112. buspower.c also  changed the
			frequency and settings of the usb host/device mode.
			we have tried to keep the changes related to usb host to
			this file, and thus we override both the buspower and
			tomtomgo_init settings*/
			
			/* we never use the EXTCLK but we use a crystal as source
			clock that crystal is 12 Mhz */
			
			/* configure the clock source for the u(sb)pll */
			i =  __raw_readl(S3C2412_CLKSRC);
			i  &= ~(S3C2412_CLKSRC_SELUREF_MASK);/* clean up the upll clock source mask( 3 << 12)*/
			i  |= S3C2412_CLKSRC_SELUREF_EXTOSC;/* configure the external crystal to be clock source (2 << 12) */
			
			
			/* We now have a 12Mhz clock signal to the upll */
			/* Select the upll output to be the source for the USB system clock (USYSCLK = FOUTupll) */
			/* SELUPLL =1 */
			i |= S3C2412_CLKSRC_SELUPLL; /* (1 <<5) */
			
			/* the USYSCLK on it's turn must be used a source for the USBSRCCLK */
			/* SELUSB =0 */
			i &= ~(S3C2412_CLKSRC_SELUSB); /* 1 < 10 */
			
			__raw_writel(i,S3C2412_CLKSRC);
			
			
			
			/*configure the usb divider */
			i =  __raw_readl(S3C2410_CLKDIVN);
			/* 1 = 48Mhz / 2 | 0 = 48Mhz */
			i &= ~(S3C2412_CLKDIVN_USB48DIV);
			__raw_writel(i , S3C2410_CLKDIVN);
			
			
			/* We now need to configure the upll and put it on fire */
			//FOUTupll 48Mhz p = 7 m = 64 (0x40) , s 1
			//[20] =0 (ON)
			//[19:12] = MDIV
			//[9:4] = PDIV
			//[0,1] = SDIV
			i &= ~(S3C2410_PLLCON_MDIVMASK | S3C2410_PLLCON_PDIVMASK | S3C2410_PLLCON_SDIVMASK) ;
			i |= 0x40  << S3C2410_PLLCON_MDIVSHIFT;
			i |=  7   << S3C2410_PLLCON_PDIVSHIFT;
			i |=  1   << S3C2410_PLLCON_SDIVSHIFT;
			__raw_writel(i , S3C2410_UPLLCON);
			
			mdelay(1);
			
			/*enable the usb host */
			i = __raw_readl(S3C2410_CLKCON);
			i |=  S3C2412_CLKCON_USBH;
			i |= S3C2412_CLKCON_USBH48M;
			/*dissable the usb device mode */
			i &=  ~(S3C2412_CLKCON_USBD);
			i &= ~(S3C2412_CLKCON_USBD48M);
			
			__raw_writel(i,S3C2410_CLKCON);
			
//			IO_Deactivate(USB_PULL_EN);
			
			/* Select USBD instead of USBH1, enable USB port 1 */
			//i = __raw_readb(S3C2412_MISCCR);
			//i |= S3C2412_MISCCR_USBHOST; // (1<<3);
			//i &= ~S3C2412_MISCCR_USBSUSPND1; // (1<<13);
			//__raw_writel(i, S3C2412_MISCCR);
			
		} else {
			/* 2410/2440/2442 */
			i = __raw_readl(S3C2410_MISCCR);
			i |= S3C2410_MISCCR_USBSUSPND0; /* disable port 1 */
			i |= S3C2410_MISCCR_USBSUSPND1; /* disable port 2 */
			
			if (IO_UsbOhciPortMask() & 0x01){
				i &= ~(S3C2410_MISCCR_USBSUSPND0);         /* enable port 1 */
			}
			
			if (IO_UsbOhciPortMask() & 0x02) {
				i |= S3C2410_MISCCR_USBSUSPND1;   /* enable port 2 */
				i |= S3C2410_MISCCR_USBHOST;      /* set port 2 to host instead of device */
			} else {
				i &= ~(S3C2410_MISCCR_USBHOST);   /* set port 2 to device, for usb cable detection */
			}
			
			__raw_writel(i, S3C2410_MISCCR);
			
		}
	}
}

static void s3c2410_start_hc(struct platform_device *dev, struct usb_hcd *hcd)
{
//	unsigned int i;
	struct s3c2410_hcd_info *info = dev->dev.platform_data;

#ifdef CONFIG_MACH_TOMTOMGO

	s3c2410_tomtom_init();
#endif

//	/* route UPLL as OHCI clock */
	//i = __raw_readl(S3C2410_CLKCON);
	//i|= S3C2410_CLKCON_USBH;
	//__raw_writel(i, S3C2410_CLKCON);

	/* set DP0/DN0 as host, not device */
	/*kejo:TODO review this code misscr must be changed
	to use misccr */
	/*
	i = __raw_readl(S3C2410_MISCCR);
	i |= S3C2410_MISCCR_USBHOST; 
	__raw_writel(i, S3C2410_MISCCR);
	*/

	dev_dbg(&dev->dev, "s3c2410_start_hc:\n");
	clk_enable(clk);

	if (info != NULL) {
		info->hcd	= hcd;
		info->report_oc = s3c2410_hcd_oc;

		if (info->enable_oc != NULL) {
			(info->enable_oc)(info, 1);
		}
	}
}

static void s3c2410_stop_hc(struct platform_device *dev)
{
	struct s3c2410_hcd_info *info = dev->dev.platform_data;

	dev_dbg(&dev->dev, "s3c2410_stop_hc:\n");

	if (info != NULL) {
		info->report_oc = NULL;
		info->hcd	= NULL;

		if (info->enable_oc != NULL) {
			(info->enable_oc)(info, 0);
		}
	}

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

	/* Check if we need to do the S3C2443 Workaround. S3C2443 hub status will reflect if a device */
	/* is connected even if we are currently in device mode. This would cause the OHCI controller */
	/* to fail during device detection, and cause it to disable the port. */
	/* To work around this, we return as if there is no device connected when we are in device mode. */
	if((IO_GetCpuType() == GOCPU_S3C2443) ||
           (IO_GetCpuType() == GOCPU_S3C2450)) {
		if(! (__raw_readl( S3C2443_PHYCTRL ) & 0x00000001) ) {
			buf[0]&=0x02;
			buf[1]=0;
			return (buf[0] != 0 ? 1 : 0);
		}
	}

	return orig;
}

/* s3c2410_usb_set_power
 *
 * configure the power on a port, by calling the platform device
 * routine registered with the platform device
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
	struct usb_hub_descriptor *desc;
	int ret = -EINVAL;
	u32 *data = (u32 *)buf;

	dev_dbg(hcd->self.controller,
		"s3c2410_hub_control(%p,0x%04x,0x%04x,0x%04x,%p,%04x)\n",
		hcd, typeReq, wValue, wIndex, buf, wLength);

	/* if we are only an humble host without any special capabilities
	 * process the request straight away and exit */

	/* Check if we need to do the S3C2443 Workaround. S3C2443 hub status will reflect if a device */
	/* is connected even if we are currently in device mode. This would cause the OHCI controller */
	/* to fail during device detection, and cause it to disable the port. */
	/* To work around this, we return as if there is no device connected when we are in device mode. */
	if((IO_GetCpuType() == GOCPU_S3C2443) ||
           (IO_GetCpuType() == GOCPU_S3C2450)) {
		if( ((__raw_readl( S3C2443_PHYCTRL ) & 0x00000001) == 0) && (typeReq == GetPortStatus) && (wIndex != 1) ) {
			ret=ohci_hub_control( hcd, typeReq, wValue, wIndex, buf, wLength );
			if( ret >=0 )
				*(__le32 *) buf &= ~(RH_PS_CSC | RH_PS_LSDA | RH_PS_CCS);
			return ret;
		}
	}
	
	if (info == NULL) {
		ret = ohci_hub_control(hcd, typeReq, wValue,
				       wIndex, buf, wLength);
		goto out;
	}

	/* check the request to see if it needs handling */

	switch (typeReq) {
	case SetPortFeature:
		if (wValue == USB_PORT_FEAT_POWER) {
			dev_dbg(hcd->self.controller, "SetPortFeat: POWER\n");
			s3c2410_usb_set_power(info, wIndex, 1);
			goto out;
		}
		break;

	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_C_OVER_CURRENT:
			dev_dbg(hcd->self.controller,
				"ClearPortFeature: C_OVER_CURRENT\n");

			if (valid_port(wIndex)) {
				info->port[wIndex-1].oc_changed = 0;
				info->port[wIndex-1].oc_status = 0;
			}

			goto out;

		case USB_PORT_FEAT_OVER_CURRENT:
			dev_dbg(hcd->self.controller,
				"ClearPortFeature: OVER_CURRENT\n");

			if (valid_port(wIndex)) {
				info->port[wIndex-1].oc_status = 0;
			}

			goto out;

		case USB_PORT_FEAT_POWER:
			dev_dbg(hcd->self.controller,
				"ClearPortFeature: POWER\n");

			if (valid_port(wIndex)) {
				s3c2410_usb_set_power(info, wIndex, 0);
				return 0;
			}
		}
		break;
	}

	ret = ohci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
	if (ret)
		goto out;

	switch (typeReq) {
	case GetHubDescriptor:

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

	case GetPortStatus:
		/* check port status */

		dev_dbg(hcd->self.controller, "GetPortStatus(%d)\n", wIndex);

		if (valid_port(wIndex)) {
			if (info->port[wIndex-1].oc_changed) {
				*data |= cpu_to_le32(RH_PS_OCIC);
			}

			if (info->port[wIndex-1].oc_status) {
				*data |= cpu_to_le32(RH_PS_POCI);
			}
		}
	}

 out:
	return ret;
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
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	usb_remove_hcd(hcd);
	s3c2410_stop_hc(dev);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. OHCI is using HCLK based memory-mapped IO and external UPLL clock, so I think 
 * we are safe besides possible issues with memory bandwidth and MEMCLK has to be > UPLL clock.
 */
static int
usb_hcd_s3c2410_freq_transition(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	return 0;
}

static int
usb_hcd_s3c2410_freq_policy(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
		policy->min = 12000; // we want at least 12 Mhz
		break;
	case CPUFREQ_INCOMPATIBLE:
		/* todo: fill in min/max values */
		break;
	case CPUFREQ_NOTIFY:
		break;
	}
	return 0;
}
#endif

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
			   struct platform_device *dev)
{
	struct usb_hcd *hcd = NULL;
	int retval;

	s3c2410_usb_set_power(dev->dev.platform_data, 0, 1);
	s3c2410_usb_set_power(dev->dev.platform_data, 1, 1);

	hcd = usb_create_hcd(driver, &dev->dev, "s3c24xx");
	if (hcd == NULL)
		return -ENOMEM;

	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len   = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_err(&dev->dev, "request_mem_region failed");
		retval = -EBUSY;
		goto err0;
	}

	clk = clk_get(NULL, "usb-host");
	if (IS_ERR(clk)) {
		dev_err(&dev->dev, "cannot get usb-host clock\n");
		retval = -ENOENT;
		goto err1;
	}

	clk_use(clk);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = usb_hcd_s3c2410_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_OHCI_HOST_PRIO;
	freq_policy.notifier_call = usb_hcd_s3c2410_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	s3c2410_start_hc(dev, hcd);

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&dev->dev, "ioremap failed\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, dev->resource[1].start, SA_INTERRUPT);
	if (retval != 0)
		goto err2;

	return 0;

 err2:
	s3c2410_stop_hc(dev);
	iounmap(hcd->regs);
	clk_unuse(clk);
	clk_put(clk);

 err1:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

 err0:
	usb_put_hcd(hcd);
	return retval;
}

/*-------------------------------------------------------------------------*/

static int
ohci_s3c2410_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}


static const struct hc_driver ohci_s3c2410_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"S3C24XX OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_s3c2410_start,
	.stop =			ohci_stop,

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

#ifdef CONFIG_USB_SUSPEND
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
};

/* device driver */

static int ohci_hcd_s3c2410_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	return usb_hcd_s3c2410_probe(&ohci_s3c2410_hc_driver, pdev);
}

static int ohci_hcd_s3c2410_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	usb_hcd_s3c2410_remove(hcd, pdev);
	return 0;
}

static int ohci_hcd_s3c2410_drv_suspend(struct device *dev, pm_message_t state, u32 level)
{

	if (level == SUSPEND_POWER_DOWN) {
		printk(KERN_INFO PFX " suspend power down \n");
		//unsigned misccr;
		/* set USB pads in suspend mode */
		/*kejo:TODO, enable this code back again
		it needs to be changed to use the misscr from regs-dyn
		it also need to be disabled for the 2443 */
		/*
		misccr = __raw_readl(S3C2410_MISCCR);
		misccr |= S3C2410_MISCCR_USBSUSPND0;
		misccr |= S3C2410_MISCCR_USBSUSPND1;
		__raw_writel(misccr, S3C2410_MISCCR);
		*/
		//struct platform_device *pdev = to_platform_device(dev);
		//struct usb_hcd *hcd = dev_get_drvdata(dev);
		//ohci_stop(hcd);
		//
	}
	return 0;
}

static int ohci_hcd_s3c2410_drv_resume(struct device *dev, u32 level)
{
	if (level == RESUME_ENABLE) {
		printk(KERN_INFO PFX " resume..\n");

		s3c2443_resurrect_phy();

		//struct platform_device *pdev = to_platform_device(dev);
		struct usb_hcd *hcd = dev_get_drvdata(dev);
		struct ohci_hcd * ohci =hcd_to_ohci(hcd);

		s3c2410_tomtom_init();
		ohci_restart(ohci);
	}
	return 0;
}

static struct device_driver ohci_hcd_s3c2410_driver = {
	.name		= "s3c2410-ohci",
	.bus		= &platform_bus_type,
	.probe		= ohci_hcd_s3c2410_drv_probe,
	.remove		= ohci_hcd_s3c2410_drv_remove,
	.suspend	= ohci_hcd_s3c2410_drv_suspend, 
	.resume		= ohci_hcd_s3c2410_drv_resume, 
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
