/* drivers/barcelona/dock/dock_cagliari.c
 *
 * Dock driver for cagliari
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <rogier.stam@tomtom.com>
 *         Laurent Gregoire <laurent.gregoire@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uio.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ns73.h>
#include <linux/mcp23008.h>
#include <linux/kdev_t.h>
#include <linux/workqueue.h>
#include <barcelona/gopins.h>
#include <linux/i2c.h>
#include <../arch/arm/mach-s3c2410/tomtomgo-iopins.h>

#include "dock_detect.h"

#define MAX_SUB_DEV 2
#define MODPFX	"DockC: "

#if 0
TODO Link to the SI4703 FM radio tuner driver
/* Empty placeholder to stop the kernel from nagging on a detach. */
static void dock_platform_release(struct device *dev)
{
	return;
}

static struct fm_transmitter_info	tomtomgo_fmx_info = {
	.slave_address			= NS73_I2C_SLAVE_ADDR,
	.device_nr			= MKDEV(NS73_MAJOR, NS73_MINOR),
};

static struct platform_device		tomtomgo_device_fmx = {
        .name				= NS73_DEVNAME,
        .id				= -1,
        .dev =	{
			.platform_data	= &tomtomgo_fmx_info,
			.release	= dock_platform_release,
		}
};
#endif

static struct device *probe_dev = NULL;
static int current_dock = 0;	/* 0 = No dock */

struct generic_dock {
	int det_mask;
	int dock_type;
	struct platform_device *sub_devices[MAX_SUB_DEV];
};

static struct generic_dock DOCK_MATRIX[] = {
	{ .det_mask = 0x00, /* 0 0  0 0 Dock 1 - Home */
	  .dock_type = GODOCK_DESK,
	  .sub_devices = { NULL, NULL }
	},
	{ .det_mask = 0x05, /* 0 1  0 1 Dock 2 - Screen without FM */
	  .dock_type = GODOCK_WINDSCR_WO_FM,
	  .sub_devices = { NULL, NULL }
	},
	{ .det_mask = 0x0A, /* 1 0  1 0 Dock 3 - Screen with FM */
	  .dock_type = GODOCK_WINDSCREEN,
	  .sub_devices = { NULL, NULL }
	},
	{ .det_mask = 0x04, /* 0 1  0 0 Dock 4 - ? */
	  .dock_type = -1,
	  .sub_devices = { NULL, NULL }
	},
	{ .det_mask = 0x08, /* 1 0  0 0 Dock 5 - ? */
	  .dock_type = -1,
	  .sub_devices = { NULL, NULL }
	},
};

static void dock_irq_debounce_handler(void *not_used)
{
	int dock_nr, i, det_mask = 0;

	/* Check the status of our pin with power disabled (bit 3/2). */
	det_mask |= (IO_GetInput(DOCK_SENSE) ? 1 : 0) << 3;
	det_mask |= (IO_GetInput(DOCK_SENSE1) ? 1 : 0) << 2;

	/* Check the status of our pin with power enabled (bit 1/0). */
	IO_Activate(DOCK_PWREN);
	msleep(10); /* Let external signal stabilize */
	det_mask |= (IO_GetInput(DOCK_SENSE) ? 1 : 0) << 1;
	det_mask |= (IO_GetInput(DOCK_SENSE1) ? 1 : 0);

	dock_nr = 0;
	for (i = 0; i < sizeof(DOCK_MATRIX) / sizeof(DOCK_MATRIX[0]);
			i++)
	{	if (det_mask == DOCK_MATRIX[i].det_mask)
		{	dock_nr = i + 1;
			break;
		}
	}
	if (dock_nr != current_dock)
	{
		printk(KERN_INFO MODPFX "Dock change detected: %d => %d\n",
				current_dock, dock_nr);
		if (dock_nr != 0) {
			/* all external dock hardware has a HEADPHONE_DETECT pullup */
			IO_DisablePullResistor(HEADPHONE_DETECT);
		}

		if (current_dock > 0) {
			for (i = 0; i < MAX_SUB_DEV; i++)
			{	struct platform_device *pv =
					DOCK_MATRIX[current_dock - 1]
					.sub_devices[i];
				if (pv) {
					printk(KERN_DEBUG MODPFX
						"Unregistering dev %s\n",
						pv->name);
					platform_device_unregister(pv);
		}	}	}
		current_dock = dock_nr;
		if (current_dock > 0) {
			for (i = 0; i < MAX_SUB_DEV; i++)
			{	struct platform_device *pv =
					DOCK_MATRIX[current_dock - 1]
					.sub_devices[i];
				if (pv) {
					printk(KERN_DEBUG MODPFX
						"Registering dev %s\n",
						pv->name);
					platform_device_register(pv);
		}	}	}
	}

	/* Back to interrupt pin. */
	IO_SetInterruptOnToggle(DOCK_SENSE);
	IO_SetInterruptOnToggle(DOCK_SENSE1);

	return;
}

static struct work_struct dock_irq_wq;

static irqreturn_t dock_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	/* Getting a DOCK_INT interrupt. E.g dock was inserted or removed. */  
	/* schedule a workqueue which can wait for the interrupt to debounce. */
	/* We use workqueue so that even if another DOCK_INT IRQ occurs, */
	/* no new workqueue will be scheduled. The wq itself can do */
	/* the waiting for the DOCK_INT to be debounced. Note that we */
	/* disable the IRQ also to ensure no spurious interrupts. */
	IO_SetInput(DOCK_SENSE);
	IO_SetInput(DOCK_SENSE1);

	/* Schedule the workqueue. */
	schedule_delayed_work(&dock_irq_wq, HZ/10);

	return IRQ_HANDLED;
}

static int dock_cagliari_state_handler(void)
{
	if (current_dock == 0)
		return GODOCK_NONE;
	return DOCK_MATRIX[current_dock - 1].dock_type;
}

static int dock_probe(struct device *dev)
{
	probe_dev = dev;

	dock_set_dock_state_handler(dock_cagliari_state_handler);

	IO_Activate(DOCK_DET_PWREN);

	return 0;
}

static void dock_detect(struct device *dev)
{
	struct platform_device	*pdev;
	struct resource		*res1;
	struct resource		*res2;
	int			det_mask = 0;

	pdev = to_platform_device(dev);
	res1 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	res2 = platform_get_resource(pdev, IORESOURCE_IRQ, 1);

	/* We first detect whether the dock is there on power up. */
	IO_Activate(DOCK_PWREN);
	IO_SetInput(DOCK_SENSE);
	IO_SetInput(DOCK_SENSE1);
	mdelay(1);
	det_mask |= (IO_GetInput(DOCK_SENSE) ? 1 : 0) << 1;
	det_mask |= (IO_GetInput(DOCK_SENSE1) ? 1 : 0);
	IO_Deactivate(DOCK_PWREN);

	/* prevent headphone detect pin floating when not docked */
	IO_SetPullResistor(HEADPHONE_DETECT);

	/* Register the interrupts. */
	if ((res1 == NULL) || request_irq(res1->start,
			dock_irq_handler, SA_INTERRUPT | SA_SAMPLE_RANDOM,
			"dock_irq1", dev))
	{
		printk(KERN_ERR MODPFX "Can't register dock interrupt 1!\n");
		return;
	} 
	if ((res2 == NULL) || request_irq(res2->start,
			dock_irq_handler, SA_INTERRUPT | SA_SAMPLE_RANDOM,
			"dock_irq2", dev))
	{
		printk(KERN_ERR MODPFX "Can't register dock interrupt 2!\n");
		free_irq(res1->start, dev);
		return;
	} 

	/* Create the work queue. */
	INIT_WORK(&dock_irq_wq, dock_irq_debounce_handler, NULL);

	IO_SetInterruptOnToggle(DOCK_SENSE);
	IO_SetInterruptOnToggle(DOCK_SENSE1);

	schedule_work(&dock_irq_wq);

	return;
}

static int __init dock_late_probe(void)
{
	if (probe_dev != NULL)
		dock_detect(probe_dev);

	return 0;
}
late_initcall(dock_late_probe);


static int dock_remove(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct resource		*res1 =
		platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct resource		*res2 = 
		platform_get_resource(pdev, IORESOURCE_IRQ, 1);

	/* Release the 2 dock irq. */
	if (res1 != NULL)
		free_irq(res1->start, dev);
	if (res2 != NULL)
		free_irq(res2->start, dev);
	if (res1 == NULL || res2 == NULL)
	{
		printk(KERN_ERR MODPFX "Can't unregister dock interrupt!\n");
		return -ENODEV;
	}

	/* Configure the irq pin back to input. */
	IO_SetInput(DOCK_SENSE);
	IO_SetInput(DOCK_SENSE1);

	IO_SetPullResistor(HEADPHONE_DETECT);

	/* Ensure no tasks are remaining on the work queue. */
	cancel_delayed_work(&dock_irq_wq);
	flush_scheduled_work();

	probe_dev = NULL;

	return 0;
}

static void dock_shutdown(struct device *dev)
{
	dock_remove(dev);
	return;
}

static int dock_suspend(struct device *dev, pm_message_t state, u32 level)
{
	if (level == SUSPEND_POWER_DOWN) {
		IO_DisablePullResistor(HEADPHONE_DETECT);
		return dock_remove(dev);
	} else {
		return 0;
	}
}

static int dock_resume(struct device *dev, u32 level)
{
        if (level == RESUME_POWER_ON) {
		dock_detect(dev);
	}
	
	return 0;
}

static struct device_driver dock_driver = {
	.owner		= THIS_MODULE,
	.name		= "dock-cagliari-dongle",
	.bus		= &platform_bus_type,
	.probe		= dock_probe,
	.remove		= dock_remove,
	.shutdown	= dock_shutdown,
	.suspend	= dock_suspend,
	.resume		= dock_resume,

};

static int __init dock_cagliari_init(void)
{
	/* Signon message. */
	printk("Cagliari dock driver - v1.0\n");
	return driver_register(&dock_driver);
}

static void __exit dock_cagliari_exit(void)
{
	driver_unregister(&dock_driver);
}

module_init(dock_cagliari_init);
module_exit(dock_cagliari_exit);

MODULE_DESCRIPTION("Dock driver for Cagliari");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Laurent Gregoire <laurent.gregoire@tomtom.com>");

