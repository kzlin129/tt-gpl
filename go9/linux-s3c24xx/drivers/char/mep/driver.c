/*
 * Synaptics embedded Touchpad driver for TomTom GO devices.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 *
 */

#include <linux/config.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/irq.h>

#include "mep_bus.h"

#ifdef MEP_CONFIG_POLLED_MODE
	#include <barcelona/Barc_adc.h>
#else
	#include <linux/interrupt.h>
#endif

#include <barcelona/gopins.h>

mep_bus_t mep_bus;

/* Import from device.c */
extern struct file_operations mep_dev_fops;

wait_queue_head_t mep_wait;

/*-------------------------------------------------------------------------*/
static void
mep_bus_rx(mep_err_t error)
{
	mep_packet_t pkt;
	if (error == MEP_NOERR && mep_rx(&pkt) == MEP_NOERR) {
		/* Check packet and store in correct device buffer */
		unsigned long flags;
		int addr = MEP_GET_ADDR(&pkt);
		mep_dev_t* dev = mep_bus.devices[addr];
		if (dev != NULL) {
			spin_lock_irqsave(&dev->lock, flags);
				/* Is it an Absolute Packet? */
				if ( (pkt.rawPkt[0] & 0x07) == 6 && MEP_GET_CTRL(&pkt) == 1) {
					dev->state.w = (pkt.rawPkt[1] >> 4) & 0x0F;
					dev->state.x = pkt.rawPkt[3] + (pkt.rawPkt[2] & 0x1F) * 0x100;
					dev->state.y = pkt.rawPkt[5] + (pkt.rawPkt[4] & 0x1F) * 0x100;
					dev->state.z = pkt.rawPkt[6];
					dev->changed = 1;
					wake_up_interruptible(&mep_wait);
				}
				/* Is it a Button Packet? */
				else if ( (pkt.rawPkt[0] & 0x07) == 1 && MEP_GET_CTRL(&pkt) == 3) {
					dev->curr_buttons = pkt.rawPkt[1] & 0x0F;
					if (dev->changed == 0) {
						dev->state.buttons = dev->curr_buttons;
						dev->changed = 1;
						wake_up_interruptible(&mep_wait);
					}
				}
				/* Ignore all other packets. */
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			dev_warn(mep_bus.dev, "Received packet from unused MEP device %d\n", addr);
		}
	}
}



#ifdef MEP_CONFIG_POLLED_MODE
/*-------------------------------------------------------------------------*/
static void
mep_bus_poll(short buf[], void* arg)
{
	/* Try and receive/handle a single packet */
	mep_bus_rx(MEP_NOERR);
}

#else
/*-------------------------------------------------------------------------*/
/* IRQ related device handling */
static irqreturn_t
mep_bus_irq(int irqno, void *dev_id, struct pt_regs *regs)
{
	mep_bus_t* bus = dev_id;
	
	if (bus) {
		mep_machine_t result;

		mep_pl_intDisable();		/* Disable further MEP ints... */

		result = mep_machine();		/* Invoke the MEP machine */

		if (result == MEP_DISABLE_INTS) {
			mep_pl_intDisable();
		} else {
			/*Must have returned MEP_ENABLE_INTS or MEP_CLK_TIMEOUT. */
			if (result == MEP_CLK_TIMEOUT) {
				/* TODO: We might want to reset the module, if possible. */
			}
        
			/* In either case, we want to reenable MEP interrupts */
			mep_pl_intEnable();
		}
	}

	return IRQ_HANDLED;
}
#endif

static void
mep_bus_sleep(void)
{
//	mep_packet_t mep_sleep = { { 1, 0x0a } };
//	mep_tx(&mep_sleep);
}

static void
mep_bus_active(void)
{
//	mep_packet_t mep_active = { { 1, 0x09 } };
//	mep_tx(&mep_active);
}

/*-------------------------------------------------------------------------*/
/* Module level load/unload/power management functions */

static int
mep_bus_probe(struct device *dev)
{
	mep_packet_t mep_reportmode = { { 0x3, 0x60, 0x00, 0x85 } };
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res = NULL;
	int ret;

	mep_bus.dev = dev;

	dev_err(dev, "Initializing wait queue\n");
	init_waitqueue_head(&mep_wait);
	
	/* find the IRQ for this unit.
	 */

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "cannot find IRQ\n");
		ret = -ENOENT;
		goto error;
	}
	
#ifdef MEP_CONFIG_POLLED_MODE
	ret = adc_register_poll(mep_bus_poll, NULL );
	if (ret) {
		dev_err(dev, "cannot register polling function\n");
		goto error;
	}
#else
	ret = request_irq(res->end, mep_bus_irq, SA_INTERRUPT, pdev->name, &mep_bus);
	if (ret) {
		dev_err(dev, "cannot claim IRQ %lu\n", res->end);
		goto error;
	}
	
	mep_bus.irq = res;
#endif

	/* TODO: Probe the bus for available devices? */
	ret = register_chrdev(MEP_BUS_MAJOR, "synaptics-mep-bus", &mep_dev_fops);
	if (ret) {
		dev_err(dev, "could not register major device number\n");
		goto error;
	}

	mep_init();

#ifndef MEP_CONFIG_POLLED_MODE
	mep_setCallback(MEP_RX_CALLBACK, mep_bus_rx, NULL);
#endif

	dev_set_drvdata(dev, &mep_bus);
	mep_bus_active();
	mep_tx(&mep_reportmode);
	
	return 0;

error:
#ifndef MEP_CONFIG_POLLED_MODE
	if (res != NULL) free_irq(res->end, mep_bus_irq);
#endif

	return ret;
}

/*-------------------------------------------------------------------------*/
static int
mep_bus_remove(struct device *dev)
{
#ifdef MEP_CONFIG_POLLED_MODE
	adc_unregister_poll(mep_bus_poll);
#else	
	mep_bus_t* bus = dev_get_drvdata(dev);

	/* If IRQ was setup, remove it */
	if (bus != NULL && bus->irq != NULL) {
		free_irq(bus->irq->end, &mep_bus);
		bus->irq = NULL;
	}
#endif
	
	return 0;
}

/*-------------------------------------------------------------------------*/
static void
mep_bus_shutdown(struct device *dev)
{
#ifndef MEP_CONFIG_POLLED_MODE
	mep_bus_t* bus;
#endif

	mep_bus_sleep();
#ifdef MEP_CONFIG_POLLED_MODE
        adc_unregister_poll(mep_bus_poll);
#else
	bus = dev_get_drvdata(dev);

	/* If IRQ was setup, remove it */
	if (bus != NULL && bus->irq != NULL) {
		free_irq(bus->irq->start, &mep_bus);
		bus->irq = NULL;
	}
#endif

	dev_set_drvdata(dev, NULL);
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int
mep_bus_suspend(struct device *dev, u32 state, u32 level)
{
	dev_dbg(dev, "state = %u, level = %u\n", state, level);

	if (level == SUSPEND_POWER_DOWN) {
		mep_bus_sleep();
	}

	return 0;
}

static int
mep_bus_resume(struct device *dev, u32 level)
{
	dev_dbg(dev, "level = %u\n", level);

	if (level == RESUME_POWER_ON) {
		mep_bus_active();
	}

	return 0;
}
#else
	#define mep_bus_suspend		NULL
	#define mep_bus_resume		NULL
#endif

/*-------------------------------------------------------------------------*/
static struct device_driver
mep_bus_driver = {
	.owner          = THIS_MODULE,
	.name           = "synaptics-mep-bus",
	.bus            = &platform_bus_type,
	.probe          = mep_bus_probe,
	.remove         = mep_bus_remove,
	.shutdown       = mep_bus_shutdown,
	.suspend        = mep_bus_suspend,
	.resume         = mep_bus_resume,
};

/* --- Module initialisation / cleanup --- */
static int __init
mep_bus_mod_init(void)
{
	int res;

	printk("TomTom GO Touchpad driver, (C) 2006 Tom Tom BV\n");

	if ((res=driver_register(&mep_bus_driver)))
		printk("tomtom-touchpad: Driver registration failed, module not inserted.\n");

	return res;
}

static void
mep_bus_mod_cleanup(void)
{
	driver_unregister(&mep_bus_driver);
}

MODULE_AUTHOR("Ithamar Adema <ithamar.adema@tomtom.com>");
MODULE_DESCRIPTION("Driver for Syntaptics Embedded Touchpad");

module_init(mep_bus_mod_init);
module_exit(mep_bus_mod_cleanup);
