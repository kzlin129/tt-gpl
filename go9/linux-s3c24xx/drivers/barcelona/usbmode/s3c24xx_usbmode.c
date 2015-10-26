/* drivers/barcelona/usbmode/s3c24xx_usbmode.c

Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
Author: Kees Jongenburger <kees.jongenburger@tomtom.com>
        Balazs Gerofi <balazs.gerofi@tomtom.com>

Methods to perform state changes between the different usb 
modes for s3c24xx platforms.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <barcelona/usbmode.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-hsudc.h>
#include <asm/arch/regs-udc.h>
#include <asm/arch/regs-dyn.h>

// HACK
#include <asm/arch/regs-irq.h>

#include <barcelona/gotype.h>
#include <barcelona/gopins.h>

#include <barcelona/Barc_buspower.h>

#include <asm/arch/regs-usb.h>

#undef ENABLE_USBPOWERSAVE

#define PFX "s3c24xx_usbmode:"
#define ERR " ERROR"
#define PK_DBG PK_DBG_FUNC

#define DRIVER_DESC_LONG "TomTom S3C24XX USB mode driver, (C) 2007 TomTom BV "

/* Time in seconds to assume there is no host present when performing
in device detect mode(Device detect == detect is we need to become device)*/
#define DEVICE_DETECT_TIMEOUT_SECONDS 2

/* Time in seconds to assume there there is no connected (non hub) device
on the bus */
#define HOST_DETECT_TIMEOUT_SECONDS 2

#define HOST_DEVICE_REMOVAL_WATCHER_TIMEOUT_SECONDS 2

struct impl_data_s3c {

	/* timer to trigger a time based timeout when in device dectect mode
	   to detect no activity */
	struct timer_list device_detect_timeout;

	/* work struct to trigger a time based timeout to detect if a device is 
	   attached to the PND */
	struct work_struct host_detect_timeout;

	/* work struct to trigger a time based timeout to detect the last device 
	   was removed and we can take action */
	struct work_struct host_device_removal_watcher_timeout;

};

static struct impl_data_s3c *s3c_impl_data;

/* hack for rebooting prague/krakow USB device */
extern int rds_tmc_hack_enabled;

/* mimic state as left by bootloader */
static int s3c2443_phy_state = 0;
static int s3c2443_usbd_state = 1;
static int s3c2443_usbh_state = 1;
static int still_booting = 1;

int s3c2443_usb_powersave_enabled(void)
{
#ifndef ENABLE_USBPOWERSAVE
	return 0;
#else
	if ((IO_GetCpuType() == GOCPU_S3C2443) ||
	    (IO_GetCpuType() == GOCPU_S3C2450) )
	{
		if (rds_tmc_hack_enabled) {
			printk(KERN_INFO "USB PHY POWERSAVE not disabling USB PHY, usb reboot in progress\n");
			return 0;
		}
		if (still_booting) {
			printk(KERN_INFO "USB PHY POWERSAVE: not disabling USB PHY because we're still booting\n");
			return 0;
		}
		/* always try to save power */
		printk(KERN_INFO "USB PHY POWERSAVE allowed\n");
		return 1;
	}
	else
	{
		printk(KERN_INFO "USB PHY POWERSAVE not allowed because this is only done on S3C2443/S3C2450.\n");
		return 0;
	}
#endif
}

void s3c2443_usb_check_booting_done(void)
{
	if (still_booting) {
		still_booting = 0;
	}
	return;
}

void s3c2443_reset_ints(void)
{
	unsigned long flags;
	unsigned int ints;
	
	local_irq_save(flags);
	__raw_writel(__raw_readl(S3C2443_PWRCFG) |
		     S3C2443_PWRCFG_nSW_PHY_OFF_USB, S3C2443_PWRCFG);

	/* disable & ack pending ints */
	__raw_writel(0, S3C_HSUDC_EP_INT_EN_REG);
	while ( (ints = __raw_readl(S3C_HSUDC_EP_INT_REG)) ) {
		__raw_writel(ints | 0x1ff, S3C_HSUDC_EP_INT_REG);
	}
	// Error status too
	while ( (ints = __raw_readl(S3C_HSUDC_SYS_STATUS_REG)) & ~((1 << 4) | (1 << 5) | (1 << 6)) ) {
		__raw_writel(ints, S3C_HSUDC_SYS_STATUS_REG);
	}
	
	/* disable ohci ints & ack ints */
	// TODO	

	// Ack controller USBD and USBH ints before enabling ints again
	__raw_writel( __raw_readl(S3C2410_INTPND) | (1 << 25) | (1 << 26), S3C2410_INTPND);
	local_irq_restore(flags);
}

void s3c2443_usbd_enable(void)
{
	unsigned long flags;
	
	if (s3c2443_usbd_state)
		return;

	// TODO: use clk_get/enable/put
	__raw_writel(__raw_readl(S3C2443_HCLKCON) 
		| S3C2443_HCLKCON_USBDEV, S3C2443_HCLKCON);

	udelay(1);

	s3c2443_reset_ints();

	// turn on ints
	local_irq_save(flags);
	__raw_writel(__raw_readl(S3C2410_INTPND) 
		| (1 << 25), S3C2410_INTPND);
	__raw_writel(__raw_readl(S3C2410_INTMSK) 
		& ~(1 << 25), S3C2410_INTMSK);
	local_irq_restore(flags);

	// turn on pullup
	__raw_writel(__raw_readl(S3C2443_UCLKCON) 
		| (S3C2443_UCLKCON_DETECT_VBUS | 
		   S3C2443_UCLKCON_FUNC_CLK_EN), S3C2443_UCLKCON);

	s3c2443_usbd_state = 1;
}

void s3c2443_usbd_disable(void)
{
	unsigned long flags;

	if (!s3c2443_usbd_state)
		return;
	// turn off pullup
	__raw_writel(__raw_readl(S3C2443_UCLKCON) 
		& ~(S3C2443_UCLKCON_DETECT_VBUS | 
		    S3C2443_UCLKCON_FUNC_CLK_EN), S3C2443_UCLKCON);

	// turn off ints
	local_irq_save(flags);
	__raw_writel(__raw_readl(S3C2410_INTPND) 
		| (1 << 25), S3C2410_INTPND);
	__raw_writel(__raw_readl(S3C2410_INTMSK) 
		| (1 << 25), S3C2410_INTMSK);
	local_irq_restore(flags);

	// TODO: use clk_get/disable/put
	__raw_writel(__raw_readl(S3C2443_HCLKCON) & 
		~S3C2443_HCLKCON_USBDEV, S3C2443_HCLKCON);

	s3c2443_reset_ints();

	s3c2443_usbd_state = 0;
}

void s3c2443_usbh_enable(void)
{
	if (s3c2443_usbh_state)
		return;

	// turn on ints
	__raw_writel(__raw_readl(S3C2410_INTMSK) &
		~(1 << 26), S3C2410_INTMSK);

	// TODO: use clk_get/enable/put
	__raw_writel(__raw_readl(S3C2443_HCLKCON) | 
		S3C2443_HCLKCON_USBHOST, S3C2443_HCLKCON);
	udelay(1);

	s3c2443_reset_ints();

	s3c2443_usbh_state = 1;
}

void s3c2443_usbh_disable(void)
{
	if (!s3c2443_usbh_state)
		return;

	// turn off ints
	__raw_writel(__raw_readl(S3C2410_INTMSK) &
		~(1 << 26), S3C2410_INTMSK);

	// turn off host clock on func block side
	__raw_writel(__raw_readl(S3C2443_UCLKCON) 
		& ~(S3C2443_UCLKCON_HOST_CLK_EN), S3C2443_UCLKCON);

	// TODO: use clk_get/disable/put
	__raw_writel(__raw_readl(S3C2443_HCLKCON) & 
		~S3C2443_HCLKCON_USBHOST, S3C2443_HCLKCON);

	s3c2443_reset_ints();
	
	s3c2443_usbh_state = 0;
}

void s3c2443_usb_phy_disable(void)
{
	printk(KERN_INFO "POWERSAVE: s3c2443_usb_phy_disable\n");

	/* disable external USB PHY power regulator, saves power */
	IO_Deactivate(USB_PHY_PWR_EN);
	IO_Deactivate(USB_PHY_1V2_PWR_EN);
	__raw_writel(__raw_readl(S3C2443_PWRCFG) &
		     ~S3C2443_PWRCFG_nSW_PHY_OFF_USB, S3C2443_PWRCFG);

	__raw_writel((__raw_readl(S3C2443_PHYPWR)& 
		~(S3C2443_PHYPWR_COMMON_ON_N | 
		  S3C2443_PHYPWR_ANALOG_POWERDOWN_MASK | 
		  S3C2443_PHYPWR_XO_ON))
		| (S3C2443_PHYPWR_ANALOG_POWERDOWN_DOWN | 
		   S3C2443_PHYPWR_PLL_POWERDOWN |
		   S3C2443_PHYPWR_FORCE_SUSPEND), S3C2443_PHYPWR);

	s3c2443_reset_ints();

	s3c2443_phy_state = 0;
}

void s3c2443_usb_phy_enable(void)
{
	printk(KERN_INFO "POWERSAVE: s3c2443_usb_phy_enable()\n");

	if (s3c2443_phy_state) {
		printk(KERN_INFO "POWERSAVE: s3c2443_usb_phy_enable() was already on, resetting\n");
	}

	/* enable external USB PHY power regulator */
	IO_Activate(USB_PHY_PWR_EN);
	IO_Activate(USB_PHY_1V2_PWR_EN);
	mdelay(1);

	s3c2443_reset_ints();

	s3c2443_phy_state = 1;
}

void s3c2443_resurrect_phy(void)
{
	printk(KERN_INFO "POWERSAVE: s3c2443_resurrect_phy()\n");

	s3c2443_usb_phy_enable();
	udelay(1);
	s3c2443_usbd_enable();
	s3c2443_usbh_enable();
}

/* Function used when we use poll based USB_HOST_DETECT monitoring */
int buspower_changed(struct notifier_block *self, unsigned long power_on,
		     void *data_pointer)
{
	struct usbmode_data *data =
	    (struct usbmode_data *)container_of(self, struct usbmode_data,
						buspower_change_listener);
	unsigned int current_input_events = data->input_events;
	printk(KERN_DEBUG PFX " %s POWER EVENT \n", __FUNCTION__);
	if (power_on) {
		if (s3c2443_usb_powersave_enabled()) {
			if ((IO_GetCpuType() == GOCPU_S3C2443) ||
			    (IO_GetCpuType() == GOCPU_S3C2450) )
                        {
				s3c2443_usb_phy_enable();
				s3c2443_usbd_enable();
			}
		}
		
		if (data->input_events & INPUT_BUS_POWER_OFF_EVENT) {
			data->input_events &= ~INPUT_BUS_POWER_OFF_EVENT;
		}
		data->input_events |= INPUT_BUS_POWER_ON_EVENT;
	} else {
		if (data->input_events & INPUT_BUS_POWER_ON_EVENT) {
			data->input_events &= ~INPUT_BUS_POWER_ON_EVENT;
		}
		data->input_events |= INPUT_BUS_POWER_OFF_EVENT;
	}

	/* if we have an input event */
	if (current_input_events != data->input_events) {
		schedule_work(&data->state_changed_work);
	}
	return 0;
}

/* start the power monitor */
void s3c24xx_start_powermon(struct usbmode_data *data)
{
	data->buspower_change_listener.notifier_call = buspower_changed;
	buspower_register_notifier(&data->buspower_change_listener);
};

/* stop the power monitor */
void s3c24xx_stop_powermon(struct usbmode_data *data)
{
	buspower_unregister_notifier(&data->buspower_change_listener);
};


int usb_count_devs(struct usb_device *usbdev)
{
	int count = 0;
	int loop = 0;

	count++;
	if (usbdev->maxchild != 0) {
		for (loop = 0; loop < usbdev->maxchild; loop++)
			if (usbdev->children[loop] != NULL)
				count += usb_count_devs(usbdev->children[loop]);
	}
	return count;
}

/**
 * return the amount of connected devices connected to the root hub in use for OTG (not counting the internal USB port)
 * Note: We ignore the roothub port 0, as this contains the bluetooth device that'll always be there (or not)
 **/
int usb_get_connected_device_count(void)
{
	struct usb_device *roothub;
	unsigned long int shiftval;
	int count = 0;
	int devcount = 0;


	roothub = usb_find_device(0, 0);
	shiftval = IO_UsbOhciPortMask();

	/* Find out which devices are in use on the roothub. IO_UsbOhciPortMask is a bitmask which has all ports */
	/* that are not in use set to 1. */
	if (roothub != NULL) {
		/* Cycle through all ports on the roothub, but exclude any port that is marked as not in use */
		/* These busses don't count for this code, but might contain a bluetooth chip. This is */
		/* irrelevant to this code however. */
		for (count = 0; count < roothub->maxchild; count++)
			if (!(shiftval & (1 << count))
			    && (roothub->children[count] != NULL))
				devcount +=
				    usb_count_devs(roothub->children[count]);
	}
	return devcount;
}

void helper_enable_host_port(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);

	unsigned int i;
	switch (IO_GetCpuType()) {
	case GOCPU_S3C2443:
	case GOCPU_S3C2450:
		/* route USB PHY DP/DN pins to device controller */
		i = __raw_readl(S3C2443_PHYCTRL);
		i |= S3C2443_PHYCTRL_DOWNSTREAM_PORT;
		__raw_writel(i, S3C2443_PHYCTRL);
		break;
	case GOCPU_S3C2410:
	case GOCPU_S3C2412:
	case GOCPU_S3C2440:
	case GOCPU_S3C2442:
		i = __raw_readl(s3c24xx_misccr);
		i |= S3C2412_MISCCR_USBHOST;	// (1<<3);
		i &= ~S3C2412_MISCCR_USBSUSPND1;	// (1<<13);
		i &= ~S3C2412_MISCCR_USBSUSPND0;	// (1<<12);
		if (IO_UsbOhciPortMask() & 0x01) {
			i |= S3C2412_MISCCR_USBSUSPND0;	// (1<<12);
		}
		if (IO_UsbOhciPortMask() & 0x02) {
			i |= S3C2412_MISCCR_USBSUSPND1;	// (1<<13);
		}
		__raw_writel(i, s3c24xx_misccr);
		break;
	default:
		printk(KERN_INFO PFX ERR
		       " %s Does not know how to enable host port on this platform\n",
		       __FUNCTION__);
	}
}

void s3c24xx_suspend_usb(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	unsigned int i;
	switch (IO_GetCpuType()) {
		case GOCPU_S3C2443:
		case GOCPU_S3C2450:
			/* Force suspend USB block */
			i = __raw_readl(S3C2443_PHYPWR);
			i |= S3C2443_PHYPWR_FORCE_SUSPEND;
			__raw_writel(i, S3C2443_PHYPWR);              
			s3c2443_phy_state = 0;
			break;

		case GOCPU_S3C2410:
		case GOCPU_S3C2412:
		case GOCPU_S3C2440:
		case GOCPU_S3C2442:

			i = __raw_readl(s3c24xx_misccr);
			i &= ~(S3C2412_MISCCR_USBHOST | S3C2412_MISCCR_USBSUSPND1);
			i |= S3C2412_MISCCR_USBSUSPND0;	// (1<<3) |(1<<13) ;
			__raw_writel(i, s3c24xx_misccr);

			break;

		default:
			printk(KERN_INFO PFX ERR
					" %s Does not know how to disable host port on this platform\n",
					__FUNCTION__);
	}

	mdelay(500);
}

/* Called when entering state initial, except for the first time this happens(i.e. when the PND was just switched on.) */
int initial_enter(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	s3c24xx_stop_powermon(data);

	return 0; // success
}

void helper_disable_host_port(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);

	unsigned int i;
	switch (IO_GetCpuType()) {
		case GOCPU_S3C2443:
		case GOCPU_S3C2450:

			/* route USB PHY DP/DN pins to host controller */
			i = __raw_readl(S3C2443_PHYCTRL);
			i &= ~(S3C2443_PHYCTRL_DOWNSTREAM_PORT);
			__raw_writel(i, S3C2443_PHYCTRL);

			break;

		case GOCPU_S3C2410:
		case GOCPU_S3C2412:
		case GOCPU_S3C2440:
		case GOCPU_S3C2442:

			i = __raw_readl(s3c24xx_misccr);
			i &= ~(S3C2412_MISCCR_USBHOST | S3C2412_MISCCR_USBSUSPND1);
			i |= S3C2412_MISCCR_USBSUSPND0;	// (1<<3) |(1<<13) ;
			__raw_writel(i, s3c24xx_misccr);

			break;

		default:
			printk(KERN_INFO PFX ERR
					" %s Does not know how to disable host port on this platform\n",
					__FUNCTION__);
	}
}

/* Start usb device detect mode . in this mode we try to detect if we must behave like a usb device */
void device_detect_timeout(unsigned long data_pointer)
{
	struct usbmode_data *data = (struct usbmode_data *)data_pointer;
	if (data->usb_state == USB_STATE_DEVICE_DETECT) {
		printk(KERN_DEBUG PFX " %s timeout\n", __FUNCTION__);
		data->input_events |= INPUT_DEVICE_DETECT_TIMEOUT;
		schedule_work(&data->state_changed_work);
	} else {
		printk(KERN_DEBUG PFX ERR " %s timeout\n", __FUNCTION__);
	}
}

static irqreturn_t
device_detect_irq_handler(int irq, void *data_pointer, struct pt_regs *regs)
{
	struct usbmode_data *data = (struct usbmode_data *)data_pointer;
	int handled;
	int i;

	do {
		int saveIdx = __raw_readl(S3C_UDC_INDEX_REG);
		int usb_status = __raw_readl(S3C_UDC_USB_INT_REG);
		int usbd_status = __raw_readl(S3C_UDC_EP_INT_REG);

		handled = 0;

		/* RESET Interrupt Request - USB reset */
		if (usb_status & S3C_UDC_USBINT_RESET) {
			/* clear interrupt */
			__raw_writel(S3C_UDC_USBINT_RESET, S3C_UDC_USB_INT_REG);

			/* we have the signal we where looking for (a reset) */
			/* we now change the input_events bit and fire the state change tasklet */
			data->input_events |= INPUT_DEVICE_RESET;
			printk(KERN_DEBUG PFX " RESET\n");
			schedule_work(&data->state_changed_work);
			handled = 1;
		}

		/* RESume Interrupt Request */
		if (usb_status & S3C_UDC_USBINT_RESUME) {
			/* clear interrupt */
			printk(KERN_DEBUG PFX " RESUME\n");
			__raw_writel(S3C_UDC_USBINT_RESUME,
				     S3C_UDC_USB_INT_REG);
			handled = 1;
		}

		/* SUSpend Interrupt Request */
		if (usb_status & S3C_UDC_USBINT_SUSPEND) {
			/* clear interrupt */
			printk(KERN_DEBUG PFX " SUSPEND\n");
			__raw_writel(S3C_UDC_USBINT_SUSPEND,
				     S3C_UDC_USB_INT_REG);
			handled = 1;
		}

		/* EP interrupts */
		/*while we did not register for EP interrupts we still get EP0 interrupts */
		if (usbd_status) {
			/* endpoint data transfers */
			for (i = 0; i < 5; i++) {
				u32 tmp = 1 << i;
				if (usbd_status & tmp) {
					/* Clear the interrupt bit by setting it to 1 */
					__raw_writel(tmp, S3C_UDC_EP_INT_REG);
					handled = 1;
				}
			}
			handled = 1;
		}
		/* restore index reg */
		__raw_writel(saveIdx, S3C_UDC_INDEX_REG);
	} while (handled);

	return IRQ_HANDLED;
}

static irqreturn_t usb2_irq_handler(int irq, void *data_pointer,
				    struct pt_regs *regs)
{
	struct usbmode_data *data = (struct usbmode_data *)data_pointer;

	unsigned int s, ep_int, tr;
	unsigned int fnr;

	/* Get interrupt source */
	s = __raw_readl(S3C_HSUDC_SYS_STATUS_REG);
	ep_int = __raw_readl(S3C_HSUDC_EP_INT_REG);
	tr = __raw_readl(S3C_HSUDC_TEST_REG);
	fnr = __raw_readl(S3C_HSUDC_FRAME_NUM_REG);

	/* Ack any endpoint activity */
	if (ep_int) {
		__raw_writel(ep_int, S3C_HSUDC_EP_INT_REG);
	}

	if (s) {
		if (s & S3C_HSUDC_INT_RESET) {
			/* we have the signal we where looking for (a reset) */
			/* we now change the input_events bit and fire the state change tasklet */

			/*TODO perhaps also enable the device reset counter here just like for the
			   non 2442 irq handler */
			data->input_events |= INPUT_DEVICE_RESET;
			schedule_work(&data->state_changed_work);
		}
		if (s & S3C_HSUDC_INT_VBUSON) {
		}
		if (s & S3C_HSUDC_INT_VBUSOFF) {
		}

		__raw_writel(s, S3C_HSUDC_SYS_STATUS_REG);
	}

	if (tr) {
		__raw_writel(tr, S3C_HSUDC_TEST_REG);
	}

	return IRQ_HANDLED;
}

/* check the registers to fix any issue with the clocks */
int setup_clocks_for_device(struct usbmode_data *data)
{
	unsigned long i, flags, tmp;

	switch (IO_GetCpuType()) {
	case GOCPU_S3C2443:
	case GOCPU_S3C2450:
		/* Disable host and device IRQs */
		local_irq_save(flags);
		tmp = __raw_readl(S3C2410_INTMSK);
 		tmp |= (S3C2443_SRCPND_USBD | S3C2443_SRCPND_USBH);
		__raw_writel(tmp, S3C2410_INTMSK);
		/* clear any pending ints */
		tmp = __raw_readl(S3C2410_INTPND);
 		tmp |= (S3C2443_SRCPND_USBD | S3C2443_SRCPND_USBH);
		__raw_writel(tmp, S3C2410_INTPND);
		local_irq_restore(flags);
		/* Are we are waking up ? */
		if (__raw_readl(S3C2443_RSTSTAT) & S3C2443_RSTSTAT_SLEEP) {
			/* restore pad IO (why here ??) */
			__raw_writel(__raw_readl(S3C2443_RSTCON) |
				     S3C2443_RSTCON_PWROFF_SLEEP,
				     S3C2443_RSTCON);
		}
		if (IO_UsbOhciPortMask() & 0x01) {
			/* put USB port in normal mode (not suspend) */
			__raw_writel(__raw_readl(s3c24xx_misccr) &
				     ~(S3C2412_MISCCR_USBSUSPND0),
				     s3c24xx_misccr);
		} else {
			__raw_writel(__raw_readl(s3c24xx_misccr) &
				     ~(S3C2412_MISCCR_USBSUSPND1),
				     s3c24xx_misccr);
		}

		/* enable external USB PHY power */
		s3c2443_usb_phy_enable();

		/* USB device 2.0 must reset like below,
		   1st phy reset and after at least 10us, func_reset & host reset
		   phy reset can reset below registers.
		 */

		__raw_writel(S3C2443_URSTCON_PHY_RESET, S3C2443_URSTCON);
		udelay(20);	/* phy reset must be asserted for at least 10us */

		/*Function 2.0, Host 1.1 S/W reset, deassert PHY reset */
		//__raw_writel( S3C2443_URSTCON_FUNC_RESET | S3C2443_URSTCON_HOST_RESET, S3C2443_URSTCON);
		__raw_writel(S3C2443_URSTCON_FUNC_RESET, S3C2443_URSTCON);

		udelay(20);
		__raw_writel(0, S3C2443_URSTCON);	/* deassert all resets  */

		mdelay(10);

		/* 12Mhz clock on ,PHY2.0 analog block power on
		   XO block power on,XO block power in suspend mode,
		   PHY 2.0 Pll power on ,suspend signal for save mode disable
		 */
		__raw_writel(S3C2443_PHYPWR_COMMON_ON_N, S3C2443_PHYPWR);

		/* D+ pull up disable(VBUS detect), USB2.0 Function clock Enable,
		   USB1.1 HOST disable, USB2.0 PHY test disable */
		__raw_writel(S3C2443_UCLKCON_DETECT_VBUS |	/* detect VBUS */
			     S3C2443_UCLKCON_HOST_CLK_TEST |	/* DISable HOST_CLK_TEST */
			     S3C2443_UCLKCON_FUNC_CLK_EN |	/* USB 2 clock enable */
			     S3C2443_UCLKCON_HOST_CLK_EN, S3C2443_UCLKCON);

		/* error interrupt enable, 16bit bus, Little format,
		   suspend&reset enable
		 */
		__raw_writew(S3C_HSUDC_DTZIEN_EN | S3C_HSUDC_RRD_EN |	/* low byte first NOT supported...      */
			     S3C_HSUDC_SUS_EN | S3C_HSUDC_RST_EN,	/* | S3C2443_UDC_SCR_BIS;       */
			     S3C_HSUDC_SYS_CON_REG);

		__raw_writel(0x00000, S3C_HSUDC_EP0_CON_REG);
		/* Enable host and device IRQ's */
		local_irq_save(flags);
		tmp = __raw_readl(S3C2410_INTMSK);
 		tmp &= ~(S3C2443_SRCPND_USBD | S3C2443_SRCPND_USBH);
		__raw_writel(tmp, S3C2410_INTMSK);
		/* clear any pending ints */
		tmp = __raw_readl(S3C2410_INTPND);
 		tmp |= (S3C2443_SRCPND_USBD | S3C2443_SRCPND_USBH);
		__raw_writel(tmp, S3C2410_INTPND);
		local_irq_restore(flags);
		break;
	case GOCPU_S3C2410:
	case GOCPU_S3C2440:
	case GOCPU_S3C2442:
		/* 2410/2440/2442, not 2412 */
		i = __raw_readb(S3C2410_CLKSLOW);
		/* enable usb the documentation points out that it is
		   required to disable CLKSLOW in order the allow the pll
		   clock */
		i &= ~S3C2410_CLKSLOW_USB_CLK_DISABLE;
		__raw_writeb(i, S3C2410_CLKSLOW);
		/* intentional fallthrough */
		break;		/* kejo:todo: is this break needed? is so remove comment above :p */
	case GOCPU_S3C2412:
		/* tomtomgo_init configures the upll clock to 24 Mhz and
		   use a different divider in this code we reprogram the
		   clocks for the 4112. buspower.c also  changed the
		   frequency and settings of the usb host/device mode.
		   we have tried to keep the changes related to usb host to
		   this file, and thus we override both the buspower and
		   tomtomgo_init settings */

		/* we never use the EXTCLK but we use a crystal as source
		   clock that crystal is 12 Mhz */

		/* configure the clock source for the u(sb)pll */
		i = __raw_readl(S3C2412_CLKSRC);
		i &= ~(S3C2412_CLKSRC_SELUREF_MASK);	/* clean up the upll clock source mask( 3 << 12) */
		i |= S3C2412_CLKSRC_SELUREF_EXTOSC;	/* configure the external crystal to be clock source (2 << 12) */

		/* We now have a 12Mhz clock signal to the upll */
		/* Select the upll output to be the source for the USB system clock (USYSCLK = FOUTupll) */
		/* SELUPLL =1 */
		i |= S3C2412_CLKSRC_SELUPLL;	/* (1 <<5) */

		/* the USYSCLK on it's turn must be used a source for the USBSRCCLK */
		/* SELUSB =0 */
		i &= ~(S3C2412_CLKSRC_SELUSB);	/* 1 < 10 */

		__raw_writel(i, S3C2412_CLKSRC);

		/*configure the usb divider */
		i = __raw_readl(S3C2410_CLKDIVN);
		/* 1 = 48Mhz / 2 | 0 = 48Mhz */
		i &= ~(S3C2412_CLKDIVN_USB48DIV);
		__raw_writel(i, S3C2410_CLKDIVN);

		/* We now need to configure the upll and put it on fire */
		//FOUTupll 48Mhz p = 7 m = 64 (0x40) , s 1
		//[20] =0 (ON)
		//[19:12] = MDIV
		//[9:4] = PDIV
		//[0,1] = SDIV
		i &= ~(S3C2410_PLLCON_MDIVMASK | S3C2410_PLLCON_PDIVMASK |
		       S3C2410_PLLCON_SDIVMASK);
		i |= 0x40 << S3C2410_PLLCON_MDIVSHIFT;
		i |= 7 << S3C2410_PLLCON_PDIVSHIFT;
		i |= 1 << S3C2410_PLLCON_SDIVSHIFT;
		__raw_writel(i, S3C2410_UPLLCON);

		mdelay(1);

		/*enable the clock to usb device */
		i = __raw_readl(S3C2410_CLKCON);
		i &= ~S3C2412_CLKCON_USBH;
		i &= ~S3C2412_CLKCON_USBH48M;
		/* TODO:how do we */
		/* enable the usb device mode */
		i |= S3C2412_CLKCON_USBD;
		i |= S3C2412_CLKCON_USBD48M;

		__raw_writel(i, S3C2410_CLKCON);
		break;
	default:
		printk(KERN_INFO PFX ERR "%s no clocks configured\n",
		       __FUNCTION__);
		break;
	}
	return 0;		/* big success */
}

int setup_clocks_for_host(struct usbmode_data *data)
{
	unsigned long i;
	switch (IO_GetCpuType()) {
	case GOCPU_S3C2443:
	case GOCPU_S3C2450:
		i = __raw_readl(S3C2443_PWRCFG);
		if (!(i & S3C2443_PWRCFG_nSW_PHY_OFF_USB)) {
			printk(KERN_INFO PFX ERR
			       " S3C2443_PWRCFG register is not set to power the usb phy block (%08lx)\n",
			       i);
			return 1;
		}

		/* check the different registers */

		i = __raw_readl(S3C2443_PHYPWR);
		if ((i & S3C2443_PHYPWR_ANALOG_POWERDOWN_MASK) ==
		    S3C2443_PHYPWR_ANALOG_POWERDOWN_DOWN) {
			printk(KERN_INFO PFX ERR
			       " PHYPWR register is not set to power the Analog block power (%08lx)\n",
			       i);
			return 1;
		}

		if (i & S3C2443_PHYPWR_PLL_REF_CLK) {
			printk(KERN_INFO PFX ERR
			       " PHYPWR register is not configured to use the external crystal clock (%08lx)\n",
			       i);
			return 1;
		}

		i = __raw_readl(S3C2443_HCLKCON);	/* check the host clock */
		if (!(i & S3C2443_HCLKCON_USBHOST)) {
			printk(KERN_INFO PFX ERR
			       " HCLKCON is not configured to feed usbhost (%08lx)\n",
			       i);
			return 1;
		}

		i = __raw_readl(S3C2443_PHYCTRL);

		/* we use the a crystal */
		if ((i & S3C2443_PHYCTRL_CLK_SEL_MASK) !=
		    S3C2443_PHYCTRL_CLK_SEL_48MHZ) {
			printk(KERN_INFO PFX ERR
			       " PHYCTRL register is not set to use the 48Mhz clock (%08lx)\n",
			       i);
			return 1;
		}
		/* we use the crystal as clock source */
		if (i & S3C2443_PHYCTRL_EXT_CLK) {
			printk(KERN_INFO PFX ERR
			       " PHYCTRL register is not set to use the crystal as clock (%08lx)\n",
			       i);
			return 1;
		}
		//TODO!!!
		break;
	case GOCPU_S3C2412:
		printk(KERN_INFO PFX " %s setting clocks for usb host\n",
		       __FUNCTION__);
		/* tomtomgo_init configures the upll clock to 24 Mhz and
		   use a different divider in this code we reprogram the
		   clocks for the 4112. buspower.c also  changed the
		   frequency and settings of the usb host/device mode.
		   we have tried to keep the changes related to usb host to
		   this file, and thus we override both the buspower and
		   tomtomgo_init settings */

		/* we never use the EXTCLK but we use a crystal as source
		   clock that crystal is 12 Mhz */

		/* configure the clock source for the u(sb)pll */
		i = __raw_readl(S3C2412_CLKSRC);
		i &= ~(S3C2412_CLKSRC_SELUREF_MASK);	/* clean up the upll clock source mask( 3 << 12) */
		i |= S3C2412_CLKSRC_SELUREF_EXTOSC;	/* configure the external crystal to be clock source (2 << 12) */

		/* We now have a 12Mhz clock signal to the upll */
		/* Select the upll output to be the source for the USB system clock (USYSCLK = FOUTupll) */
		/* SELUPLL =1 */
		i |= S3C2412_CLKSRC_SELUPLL;	/* (1 <<5) */

		/* the USYSCLK on it's turn must be used a source for the USBSRCCLK */
		/* SELUSB =0 */
		i &= ~(S3C2412_CLKSRC_SELUSB);	/* 1 < 10 */

		__raw_writel(i, S3C2412_CLKSRC);

		/*configure the usb divider */
		i = __raw_readl(S3C2410_CLKDIVN);
		/* 1 = 48Mhz / 2 | 0 = 48Mhz */
		i &= ~(S3C2412_CLKDIVN_USB48DIV);
		__raw_writel(i, S3C2410_CLKDIVN);

		/* We now need to configure the upll and put it on fire */
		//FOUTupll 48Mhz p = 7 m = 64 (0x40) , s 1
		//[20] =0 (ON)
		//[19:12] = MDIV
		//[9:4] = PDIV
		//[0,1] = SDIV
		i &= ~(S3C2410_PLLCON_MDIVMASK | S3C2410_PLLCON_PDIVMASK |
		       S3C2410_PLLCON_SDIVMASK);
		i |= 0x40 << S3C2410_PLLCON_MDIVSHIFT;
		i |= 7 << S3C2410_PLLCON_PDIVSHIFT;
		i |= 1 << S3C2410_PLLCON_SDIVSHIFT;
		__raw_writel(i, S3C2410_UPLLCON);

		mdelay(1);

		/*enable the usb host */
		i = __raw_readl(S3C2410_CLKCON);
		i |= S3C2412_CLKCON_USBH;
		i |= S3C2412_CLKCON_USBH48M;

		/*dissable the usb device mode */
		i &= ~(S3C2412_CLKCON_USBD);
		i &= ~(S3C2412_CLKCON_USBD48M);

		__raw_writel(i, S3C2410_CLKCON);

		//                      IO_Deactivate(USB_PULL_EN);

		/* Select USBD instead of USBH1, enable USB port 1 */

	default:
		break;
	}

	return 0;		/* success */
};

/*
Called when we are entering idle mode.
*/
static int idle_enter(struct usbmode_data *data)
{
	// The power may already be on at this point.
	// If so, we won't receive a power_on event so we'll stay in idle mode forever (See also FSBA-2107).
	// Therefore, ask the buspower to drive to generate a state change event regardless whether the status
	// has actually changed.
	buspower_request_state_change_event();

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
}

/*
Called when we are entering the device mode detect.
I this mode we want to look if we are connected to a usb host like a computer.
to perform this task we must have two things in place
-1 The pull(up/down???) resistor must be set so that the computer will detect
that there is a device connected to the usb cable.
-2 We must listen to the usb host's message in order to detect it sends a reset signal.

If we get a signal within 2 seconds we can be sure that we are connected to a usb host.
We have finished the probe , we disable the pull(up/down???) as fast as possible so that
the usb host thinks we already disconnected. and send a message to user land telling
that we have detected that there is a usb host on the other side.

If we do no get a signal withing 2 seconds we know we are not connected to a usb-host.
we are probably connected to a CLA (car lighter adapter) and must possibly start behaving
like a usb host
*/
static int device_detect_enter(struct usbmode_data *data)
{
	struct impl_data_s3c *  impl =(struct impl_data_s3c *) data->u_impl_data;
	printk(KERN_DEBUG PFX " device_detect_enter\n");
	/* check data */
	if (data == NULL) {
		printk(KERN_DEBUG PFX ERR " %s  data == null \n", __FUNCTION__);
	}

	unsigned int test_value;
	unsigned int i;
	struct clk *clk;

	/* TODO: first configure clocks so interrupts etc can be triggered */
	if (setup_clocks_for_device(data)) {
		printk(KERN_DEBUG PFX ERR " Clock setup failed \n");
		return 1;
	}

	switch (IO_GetCpuType()) {
	case GOCPU_S3C2443:
	case GOCPU_S3C2450:
		/*enable the pull-up so we are detected by the host is there is one */
		i = __raw_readl(S3C2443_UCLKCON);
		i |= S3C2443_UCLKCON_DETECT_VBUS;
		__raw_writel(i, S3C2443_UCLKCON);

		/* EP0-2 Interrupt enable */
		__raw_writel(S3C_HSUDC_INT_EP0 | S3C_HSUDC_INT_EP1 |
			     S3C_HSUDC_INT_EP2, S3C_HSUDC_EP_INT_EN_REG);

		__raw_writel(0x0000, S3C_HSUDC_TEST_REG);
		__raw_writel(S3C_HSUDC_INT_RESET | S3C_UDC_USBINT_SUSPEND,
			     S3C_HSUDC_SYS_STATUS_REG);
		test_value =
		    request_irq(IRQ_USBD, usb2_irq_handler, 0,
				"USB device detect IRQ driver II", data);
		if (test_value != 0) {
			printk(KERN_INFO PFX ERR
			       " failed to install irq (%d)\n", test_value);
			return 1;
		}
		break;
	default:
		/* Configure interrupts */
		IO_Activate(USB_PULL_EN);
		mdelay(100);

		/* disable EP0-4 USBD interrupts for now */
		/*kejo:2007-05-09 even if S3C_UDC_USB_INT_EN_REG I still get EP interrupts why? */

		__raw_writel(S3C_UDC_INT_EP0 | S3C_UDC_INT_EP1 |
			     S3C_UDC_INT_EP2, S3C_UDC_EP_INT_EN_REG);
		/* writing to the registers clears the data as stated in the documentation
		   we need to do that because we want to detect a reset signal. but (also in the
		   documentation ) when power (Vbus) is detected a reset signal is also generated
		   we do not want to react on that signal. This is also why we first clear the
		   registers and only after that request the interrupt. We first wait 100 ms to be sure
		   that the vbus power is stable, if this is not the case we could clear the register
		   and still get a reset interrupt because the state has been low */

		__raw_writel(0xff, S3C_UDC_USB_INT_REG);
		__raw_writel(0xff, S3C_UDC_EP_INT_REG);
		/* register an irq handler and use "data" as data_pointer */
		test_value =
		    request_irq(IRQ_USBD, device_detect_irq_handler, 0,
				"USB device detect IRQ driver", data);
		if (test_value != 0) {
			printk(KERN_INFO PFX ERR
			       " failed to install irq (%d)\n", test_value);
			return 1;
		}

	}
	/* get the usb-device clock and increase the use counter */
	clk = clk_get(NULL, "usb-device");
	if (IS_ERR(clk)) {
		printk(KERN_INFO PFX ERR " cannot get usb-device clock\n");
		goto err;
	}
	clk_use(clk);

	init_timer(&impl->device_detect_timeout);	/* only clears the timer */
	impl->device_detect_timeout.expires =
	    jiffies + DEVICE_DETECT_TIMEOUT_SECONDS * HZ;
	impl->device_detect_timeout.data = (unsigned long)data;
	impl->device_detect_timeout.function = device_detect_timeout;
	add_timer(&impl->device_detect_timeout);

	return 0;		/* Success */
      err:
	free_irq(IRQ_USBD, data);
	return 1;		/*failure */
}

/* Called when we are exiting the device mode detect (no matter what mode we wil be in next.
We try to stop anything that we started in device_detect_enter */
static int device_detect_exit(struct usbmode_data *data)
{
	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	//disable_irq(IRQ_USBD);    
	struct clk *clk;
	unsigned int i;
	printk(KERN_DEBUG PFX " %s\n", __FUNCTION__);

	/* get the usb-device clock and decrease the use counter */
	clk = clk_get(NULL, "usb-device");
	if (IS_ERR(clk)) {
		printk(KERN_INFO PFX ERR " cannot get usb-device clock\n");
	} else {
		clk_unuse(clk);
	}
	/* remove the timeout timer so we will not get false positive
	   timeouts */
	del_timer_sync(&impl->device_detect_timeout);

	free_irq(IRQ_USBD, data);

	switch (IO_GetCpuType()) 
	{
		case GOCPU_S3C2443:
		case GOCPU_S3C2450:
			i = __raw_readl(S3C2443_UCLKCON);
			i &= ~(S3C2443_UCLKCON_DETECT_VBUS);
			__raw_writel(i, S3C2443_UCLKCON);
			/* disable USB_PULL_EN */
			break;
		default:
			IO_Deactivate(USB_PULL_EN);
	}

	return 0;		/* success */
}

/*==end device detect mode **/

int s3c24xx_initial_to_idle(struct usbmode_data *data)
{
	unsigned long i;

	s3c2443_phy_state = 0;
	helper_disable_host_port(data); // This is probably superfluous, but it shouldn't harm either.
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);
	if ((IO_GetCpuType() == GOCPU_S3C2443) ||
	    (IO_GetCpuType() == GOCPU_S3C2450) )
	{
#ifdef ENABLE_USBPOWERSAVE
			s3c2443_usbh_disable();
			s3c2443_usbd_disable();
			/* can disable phy as well */
			s3c2443_usb_phy_disable();
#endif
			s3c2443_usb_check_booting_done();  
	} else {
		/* 2410/2412/2440/2442 */
		i = __raw_readl(s3c24xx_misccr);

		/* disable port 0 (put it in suspend mode) and enable port 1 */
		i |= S3C2410_MISCCR_USBSUSPND0;
		i &= ~S3C2410_MISCCR_USBSUSPND1;

		/* There are two ports the s3c2440 documentation states
		   that that SEL_USBPAD (miscr[3]) selects port usb1 as host
		   or device, from that documentation is looks like we could
		   only enable usb1 as usb host. when this value is 0 this port is used
		   as device */
		i &= ~S3C2410_MISCCR_USBHOST;	/* make sure it's 0 to be in device mode */

		__raw_writel(i, s3c24xx_misccr);
	}

	s3c24xx_start_powermon(data);
  
	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;		/* success */
};

int s3c24xx_idle_to_initial(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);

 	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s  while switching from idle to initial mode\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
}

/* start state changes */
int s3c24xx_idle_to_device_detect(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);
	unsigned int i;

	switch (IO_GetCpuType()) {

	case GOCPU_S3C2443:
	case GOCPU_S3C2450:
#ifdef ENABLE_USBPOWERSAVE
		/* skip this sequence in case we're rebooting prague/krakow */
		if (s3c2443_usb_powersave_enabled()) {
			s3c2443_usbd_enable();
			s3c2443_usbh_enable();
			s3c2443_usb_phy_enable();
			udelay(1);
		}
#endif
		s3c2443_usb_phy_enable();

		/* enable the port set [12] to 0  */
		i = __raw_readl(s3c24xx_misccr);
		i &= ~S3C2443_MISCCR_USBSUSPND;	/* SEL_SUSPND in the documentation */
		__raw_writel(i, s3c24xx_misccr);
		udelay(500);

		/* set D+ pullup to signal to host we like to be reset */
		i = __raw_readl(S3C2443_UCLKCON);
		i |= S3C2443_UCLKCON_DETECT_VBUS;
		__raw_writel(i, S3C2443_UCLKCON);

		break;

	default:
		/* check S3C2410_MISCCR register expectation */
		i = __raw_readl(s3c24xx_misccr);

		/* check the port 0 is disabled  {0 = normal mode 1= suspend mode } */
		if (!i & S3C2410_MISCCR_USBSUSPND0) {
			printk(KERN_DEBUG PFX ERR
			       " USB port 0 is not disabled \n");
			//TODO:return error
		}
		/* port 1 must be enabled */
		if (i & S3C2410_MISCCR_USBSUSPND1) {
			printk(KERN_DEBUG PFX ERR
			       " USB port 1 is not enabled \n");
		}
		/* we do no want the pad to be in usb host mode  0 = use USB1 as Device */
		if (!i & S3C2410_MISCCR_USBHOST) {
			printk(KERN_DEBUG PFX ERR
			       " USB port 1 set in host mode \n");
		}
		//printk(KERN_DEBUG PFX " VALUE %08x\n",i);
	}

	if (device_detect_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s  while switching from idle to device detect mode\n",
		       __FUNCTION__);
		return 1;
	}
	/* pull down enable */
	/* register usb device mode interrupt handler */

	/* switch to detect mode?? */
	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
};

int s3c24xx_device_detect_to_initial(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);
	if (device_detect_exit(data)) {
		printk(KERN_DEBUG PFX ERR
		       " %s error in  device_detect_exit while switching from device detect to idle mode\n",
		       __FUNCTION__);
		return 1;
	}

	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering initial mode.\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
};

int s3c24xx_device_detect_to_idle(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);
	if (device_detect_exit(data)) {
		printk(KERN_DEBUG PFX ERR
		       " %s error in  device_detect_exit while switching from device detect to idle mode\n",
		       __FUNCTION__);
		return 1;
	}

	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
};

/* Switch to device mode. This mode is not implemented
in the current kernel, this does via the bootloader.
The only thing we really want to do in this mode is to disable
the pull_enable so the connected host does not try to assign a
usb address to the device.Why? because we will reboot and only after that
be ready to behave like a usb device */
int s3c24xx_device_detect_to_device(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	//exit device detect mode...
	if (device_detect_exit(data)) {
		printk(KERN_DEBUG PFX ERR
		       " %s error in  device_detect_exit while switching from device detect to device mode\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */

};

int s3c24xx_device_to_initial(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
};

int s3c24xx_device_to_idle(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	if (s3c2443_usb_powersave_enabled()) {
		/* skip powersaving if we're rebooting prague/krakow */
		if ((IO_GetCpuType() == GOCPU_S3C2443) ||
		    (IO_GetCpuType() == GOCPU_S3C2450) )
		{
#ifdef ENABLE_USBPOWERSAVE
			s3c2443_usb_phy_disable();
			s3c2443_usbh_disable();
			s3c2443_usbd_disable();
#endif
		}
	}

	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s SUCCESS\n", __FUNCTION__);
	return 0;		/* success */
};

void host_device_removal_watcher(void *data_pointer)
{
	struct usbmode_data *data = (struct usbmode_data *)data_pointer;
	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	if (usb_get_connected_device_count() == 0) {
		printk(KERN_DEBUG PFX
		       " %s no device connected any more, disconnecting\n",
		       __FUNCTION__);
		data->input_events |= INPUT_HOST_DEVICE_DISCONNECT;
		schedule_work(&data->state_changed_work);
		return;
	};

	INIT_WORK(&impl->host_device_removal_watcher_timeout,
		  host_device_removal_watcher, (void *)data);
	schedule_delayed_work(&impl->host_device_removal_watcher_timeout,
			      HOST_DEVICE_REMOVAL_WATCHER_TIMEOUT_SECONDS * HZ);
}

void host_detect_timeout(void *data_pointer)
{
	printk(KERN_DEBUG PFX " %s \n", __FUNCTION__);
	struct usbmode_data *data = (struct usbmode_data *)data_pointer;
	if (data->usb_state == USB_STATE_HOST_DETECT) {
		struct bus_type *usb_bus = find_bus("usb");
		if (!usb_bus) {
			printk(KERN_INFO PFX
			       " failed to get the usb bus. Is usb compiled into the kernel?");
			/* In this situation we don't have usb-host support. just send a timeout event */
			if (!(data->input_events & INPUT_HOST_DETECT_TIMEOUT)) {
				printk(KERN_DEBUG PFX
				       " %s no-host... sending timeout\n",
				       __FUNCTION__);
				data->input_events |= INPUT_HOST_DETECT_TIMEOUT;
				schedule_work(&data->state_changed_work);
			} else {
				printk(KERN_DEBUG PFX ERR
				       " %s INPUT_HOST_DETECT_TIMEOUT already set in no-host mode\n",
				       __FUNCTION__);
			}
			return;
		} else if (usb_get_connected_device_count() > 0) {
			/* In this situation we have a device connected to the pnd */
			if (!(data->input_events & INPUT_HOST_DEVICE_CONNECT)) {
				data->input_events |= INPUT_HOST_DEVICE_CONNECT;
				schedule_work(&data->state_changed_work);
			} else {
				printk(KERN_DEBUG PFX ERR
				       " %s INPUT_HOST_DEVICE_CONNECT already set %02x %i %i %i\n",
				       __FUNCTION__, data->input_events,
				       data->input_events,
				       INPUT_HOST_DEVICE_CONNECT,
				       data->input_events & INPUT_HOST_DEVICE_CONNECT);
			}
		} else {
			/* There is a usb_bus but device is connected to the bus.
			   this just means that no device was detected */
			if (!(data->input_events & INPUT_HOST_DETECT_TIMEOUT)) {
				printk(KERN_DEBUG PFX " %s timeout\n",
				       __FUNCTION__);
				data->input_events |= INPUT_HOST_DETECT_TIMEOUT;
				schedule_work(&data->state_changed_work);
			} else {
				printk(KERN_DEBUG PFX ERR
				       " %s INPUT_HOST_DETECT_TIMEOUT already set\n",
				       __FUNCTION__);
			}
		}
	} else {
		/* we are not in HOST_DETECT mode but still arrive here.
		   when can that happen?. One situation where this happens
		   is when just before the timeout occurs you remove the
		   power from the bus, in that situation it can happen
		   that while you are not in HOST_DETECT mode you still can 
		   receive this event. The sate machine can handle any event in 
		   any situation but we will for the moment not generate the event
		   since we simply are not in HOST_DETECT mode. Still lets write a
		   message because the biggest chance of reaching this code is actually 
		   that somebody forgot to disable the host_detect timeout while switching
		   to a certain mode */
		printk(KERN_DEBUG PFX
		       " %s timeout while not in HOST_DETECT mode\n",
		       __FUNCTION__);
	}
}

int s3c24xx_device_detect_to_host_detect(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	
	/* Initialize work first in order to avoid failing to delete an uninitalized timer
	   in case a POWER_OFF event occurs during the device detect to host detect transition */
	INIT_WORK(&impl->host_detect_timeout, host_detect_timeout, (void *)data);

	/* First exit device detect mode */
	if (device_detect_exit(data)) {
		printk(KERN_DEBUG PFX ERR
		       " %s error in  device_detect_exit while switching from device detect to host detect mode\n",
		       __FUNCTION__);
		return 1;
	}

	if (s3c2443_usb_powersave_enabled()) {
		/* we are not using device mode block in s3c kernels, we would like to turn it off */
		/* but we can't, since it is still attached to phy somehow */
		// DO NOT s3c2443_usbd_disable();
	}

	if (setup_clocks_for_host(data)) {
		printk(KERN_DEBUG PFX ERR
		       " %s failed to setup usb host but ignoring for the moment..\n",
		       __FUNCTION__);

	};
	helper_enable_host_port(data);

	schedule_delayed_work(&impl->host_detect_timeout,
			      HOST_DETECT_TIMEOUT_SECONDS * HZ);

	return 0;		/* success */

};

int s3c24xx_host_detect_to_initial(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	helper_disable_host_port(data);
	cancel_delayed_work(&impl->host_detect_timeout);
	/* This is not yet initialized here!
		cancel_delayed_work(&impl->host_device_removal_watcher_timeout); */

	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering initial mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;		/* success */
}

int s3c24xx_host_detect_to_idle(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	helper_disable_host_port(data);
	cancel_delayed_work(&impl->host_detect_timeout);
	/* This is not yet initialized here!
		cancel_delayed_work(&impl->host_device_removal_watcher_timeout); */

	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;		/* success */
}

int s3c24xx_host_detect_to_cla(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	/* struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data; */
	return 0;		/* success */
};

int s3c24xx_host_detect_to_host(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;

	/* Schedule delayed work */
	INIT_WORK(&impl->host_device_removal_watcher_timeout,
		  host_device_removal_watcher, (void *)data);
	schedule_delayed_work(&impl->host_device_removal_watcher_timeout,
			      HOST_DEVICE_REMOVAL_WATCHER_TIMEOUT_SECONDS * HZ);

	return 0;		/* success */
};

int s3c24xx_cla_to_initial(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	helper_disable_host_port(data);

	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering initial mode.\n",
		       __FUNCTION__);
		return 1;
	}

	printk(KERN_DEBUG PFX " %s After initial_enter \n", __FUNCTION__);

	return 0;
};

int s3c24xx_cla_to_idle(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	helper_disable_host_port(data);

	if (s3c2443_usb_powersave_enabled()) {
		/* skip powersaving if we're rebooting prague/krakow */
		if ((IO_GetCpuType() == GOCPU_S3C2443) ||
		    (IO_GetCpuType() == GOCPU_S3C2450) )
		{
#ifdef ENABLE_USBPOWERSAVE
			s3c2443_usb_phy_disable();
			s3c2443_usbh_disable();
			s3c2443_usbd_disable();
#endif
		}
	}

	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;
};

int s3c24xx_host_to_initial(struct usbmode_data *data)
{
	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	printk(KERN_DEBUG PFX " %s DISABLING PORT\n", __FUNCTION__);
	cancel_delayed_work(&impl->host_device_removal_watcher_timeout);
	helper_disable_host_port(data);

	if (initial_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;
};

int s3c24xx_host_to_idle(struct usbmode_data *data)
{
	printk(KERN_DEBUG PFX " %s  \n", __FUNCTION__);

	struct impl_data_s3c *  impl = (struct impl_data_s3c * )data->u_impl_data;
	printk(KERN_DEBUG PFX " %s DISABLING PORT\n", __FUNCTION__);
	cancel_delayed_work(&impl->host_device_removal_watcher_timeout);
	helper_disable_host_port(data);

	if (s3c2443_usb_powersave_enabled()) {
		/* skip powersaving if we're rebooting prague/krakow */
		if ((IO_GetCpuType() == GOCPU_S3C2443) ||
		    (IO_GetCpuType() == GOCPU_S3C2450) )
		{
#ifdef ENABLE_USBPOWERSAVE
			s3c2443_usb_phy_disable();
			s3c2443_usbh_disable();
			s3c2443_usbd_disable();
#endif
		}
	}

	if (idle_enter(data)) {
		printk(KERN_DEBUG PFX ERR
		       " error in  %s while entering idle mode.\n",
		       __FUNCTION__);
		return 1;
	}

	return 0;
};


struct usbmode_operations s3c24xx_usbmode_ops = {
	.initial_to_idle = s3c24xx_initial_to_idle,
	.idle_to_initial = s3c24xx_idle_to_initial,
	.idle_to_device_detect = s3c24xx_idle_to_device_detect,
	.device_detect_to_initial = s3c24xx_device_detect_to_initial,
	.device_detect_to_idle = s3c24xx_device_detect_to_idle,
	.device_detect_to_device = s3c24xx_device_detect_to_device,
	.device_detect_to_host_detect = s3c24xx_device_detect_to_host_detect,
	.device_to_initial = s3c24xx_device_to_initial,
	.device_to_idle = s3c24xx_device_to_idle,
	.host_detect_to_initial = s3c24xx_host_detect_to_initial,
	.host_detect_to_idle = s3c24xx_host_detect_to_idle,
	.host_detect_to_cla = s3c24xx_host_detect_to_cla,
	.host_detect_to_host = s3c24xx_host_detect_to_host,
	.cla_to_initial = s3c24xx_cla_to_initial,
	.cla_to_idle = s3c24xx_cla_to_idle,
	.host_to_initial = s3c24xx_host_to_initial,
	.host_to_idle = s3c24xx_host_to_idle,
	.suspend_usb = s3c24xx_suspend_usb,
	.start_powermon = s3c24xx_start_powermon,
	.stop_powermon = s3c24xx_stop_powermon
};


static int __init s3c24xx_usbmode_init_module(void)
{
	int ret;
	s3c_impl_data = kmalloc(sizeof(struct impl_data_s3c), GFP_KERNEL);

	/* Allocate implementation spec data */
	if (!s3c_impl_data) {
		printk(KERN_INFO PFX " %s unable to allocated memory for impl_data_s3c\n", __FUNCTION__); 
		return -ENOMEM;
	}

	/* Register driver, usbmode late init will call init_to_idle transition */
	if ((ret = register_usbmode_driver(&s3c24xx_usbmode_ops, (void*)s3c_impl_data))) {
		printk(KERN_INFO PFX " %s error registering s3c24xx usbmode driver\n", __FUNCTION__); 
		kfree(s3c_impl_data);
		return ret;
	}
		
	printk(KERN_INFO DRIVER_DESC_LONG "(" PFX ")\n");
	return 0;	
}

static void __exit s3c24xx_usbmode_exit_module(void)
{
	/* Stop timers */
	cancel_delayed_work(&s3c_impl_data->host_detect_timeout);
	cancel_delayed_work(&s3c_impl_data->host_device_removal_watcher_timeout);
	del_timer_sync(&s3c_impl_data->device_detect_timeout);

	/* Unregister driver */	
	unregister_usbmode_driver();

	kfree(s3c_impl_data);
}

module_init(s3c24xx_usbmode_init_module);
module_exit(s3c24xx_usbmode_exit_module);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR("Kees Jongenburger (kees.jongenburger@tomtom.com), Balazs Gerofi (balazs.gerofi@tomtom.com)");
MODULE_LICENSE("GPL");

