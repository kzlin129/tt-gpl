/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Pepijn de Langen <pepijn.delangen@tomtom.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/mach/arch.h>
#include <plat/tt_vbusmon.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/mendoza.h>
#include <mach/gpio.h>


static int (*vbusdetect_handler)(struct device *, int);
static atomic_t configured = ATOMIC_INIT(0);

static int vbus_request_irq(void);
static void vbus_free_irq(void);

/**
 * Read out the value of the vbus pin
 **/
static irqreturn_t vbus_irq_handler(int irqnumber, void *dev_id)
{
	BUG_ON(!atomic_read(&configured));
	/* on s5p6440 host detect is low active! */
	if (vbusdetect_handler(&mendoza_device_vbus.dev, !gpio_get_value(TT_VGPIO_USB_HOST_DETECT))) {
		printk(KERN_DEBUG "%s: vbus handler failed\n",__FILE__);
	}
	return IRQ_HANDLED;
}

/**
 * Configure the vbus-detect pin and register it as an interrupt
 **/
static int config_vbusdetect(int (*hndlr)(struct device *, int))
{
	int ret;

	ret = gpio_request (TT_VGPIO_USB_HOST_DETECT, "vbus_mon");
	if (ret) {
		printk ("Could not get gpio number: %d\n", TT_VGPIO_USB_HOST_DETECT);
		return ret;
	}


	if (atomic_read(&configured)) {
		printk(KERN_ERR "%s: vbus-detect is already configured\n",__FILE__);
		return 1;
	}

	/* Set up the USB detect pin */
	gpio_direction_input(TT_VGPIO_USB_HOST_DETECT);

	/* Switch off pull-up/down */
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_USB_HOST_DETECT),S3C_GPIO_PULL_NONE);

	/* Set up an IRQ on the USB detect pin */
	vbusdetect_handler = hndlr;

	/* We are configured now, unset this if we fail later */
	atomic_set(&configured, 1);

	if ((ret = vbus_request_irq())) {
		printk(KERN_ERR "%s: could not register interrupt for "
				"vbus-detect\n", __FILE__);
		atomic_set(&configured, 0);
		return ret;
	}

	return 0;
}

/**
 * Unregister the vbus pin's interrupt
 **/
static void cleanup_vbusdetect(void)
{
	if (!atomic_read(&configured)) {
		printk(KERN_ERR "%s: vbus-detect is not configured\n",__FILE__);
		return;
	}

	vbus_free_irq();

	atomic_set(&configured, 0);
}


/**
 * Poll the vbus pin and return its status
 * Return codes:
 *  0             - vbus is not detected
 *  anything else - vbus is detected
 **/
static int poll_vbusdetect(void)
{
	if (!atomic_read(&configured)) {
		printk(KERN_ERR "%s: vbus-detect is not configured\n",__FILE__);
		return 0; /* return vbus not present */
	}
	/* on s5p6440 host detect is low active! */
	return !gpio_get_value(TT_VGPIO_USB_HOST_DETECT);
}

static void vbus_suspend(void)
{
	pr_info("vbus: suspending\n");
	vbus_free_irq();
}

static void vbus_resume(void)
{
	pr_info("vbus: resuming\n");
	vbus_request_irq();
}

static struct vbus_pdata pdata = {
  .name = "gpio-vbus",
	.config_vbusdetect = config_vbusdetect,
	.cleanup_vbusdetect = cleanup_vbusdetect,
	.poll_vbusdetect = poll_vbusdetect,
	.do_suspend = vbus_suspend,
	.do_resume = vbus_resume,
};

struct platform_device mendoza_device_vbus = {
	.name	= "tomtom-vbus",
	.id	= 2,
	.dev	= {
		.platform_data = &pdata
	},
};

static int vbus_request_irq(void)
{
	int ret;
	int int_nr;

	/* Retrieve interrupt number from GPIO pin */
	int_nr = gpio_to_irq(vgpio_to_gpio(TT_VGPIO_USB_HOST_DETECT));

	/* WARNING! There is a design flaw in the s3c external interrupt
	 * wakeup source handling: configuring IRQT_FALLING will resume the
	 * device ONLY if the signal is HIGH during suspend. If signal is
	 * LOW at the time of suspend, device will not resume after the
	 * sequence LOW->HIGH->LOW! */
	if ((ret = request_irq(int_nr, vbus_irq_handler, IRQ_TYPE_EDGE_BOTH, "vbus", &mendoza_device_vbus))) {
		printk(KERN_ERR "%s: could not register interrupt for "
				"vbus-detect\n", __FILE__);
		return ret;
	}

	enable_irq_wake(int_nr);
	return 0;
}

static void vbus_free_irq(void)
{
	int int_nr;

	/* Retrieve interrupt number from GPIO pin */
	int_nr = gpio_to_irq(vgpio_to_gpio(TT_VGPIO_USB_HOST_DETECT));

	/* Disable up the IRQ */
	free_irq(int_nr, &mendoza_device_vbus);
}

