/*****************************************************************************
* Copyright 2009 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
/* linux/drivers/char/bcm4760_adc.c
 *
 * Implementation of the BCM4760 specific ADC driver.
 *
 * Changelog:
 *
 * 17-Jun-2009  DB   Initial version for refs #811:
 */                  

/* Includes */ 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/broadcom/bcmtypes.h>
#include <linux/broadcom/bcm_adc.h>
#include <asm/io.h>
#include <asm/arch/irq.h>
#include <asm/arch/hw_cfg.h>
#include <asm/arch/bcm4760_reg.h>

static uint32_t	settime_value = 0x7;
#define PFX "bcm4760adc: "

/*
 * Completion Handler used in the registration process
 */
static bcm_adc_handler_t bcm4760_adc_completion_handler	= NULL;

/*
 * Current ADC Request used for tracking
 */
static bcm_adc_request_entry_t* bcm4760_adc_current_req_entry = NULL;

/*
 * Delayed Thread variables
 */
static struct workqueue_struct* bcm4760_adc_workQueue;
static struct work_struct bcm4760_adc_work;

static unsigned int bcm4760_adc_irq;

// no need for atomic because the caller ensures that things are properly ordered
static int suspended = ATOMIC_INIT(0);

/****************************************************************************
 ** BCM4760 ADC Public Interfaces
 */
/*
 * Register the BCM-ADC completion handler
 */
int bcm4760_adc_complete_register(bcm_adc_handler_t adcCompletion)
{
//	printk("BCM4760-ADC: ADC Complete Handler Registration called\n");
	bcm4760_adc_completion_handler = adcCompletion;
	return 0;
}

/*
 * BCM-ADC request
 */
int bcm4760_adc_request(
	bcm_adc_request_entry_t* req_entry)
{
	uint32_t	data;

//	printk("BCM4760-ADC: ADC Request received\n");

	if (suspended)
		return -EAGAIN;

	/*
	 * Save the current request for the response
	 */
	WARN_ON(bcm4760_adc_current_req_entry != NULL);
	bcm4760_adc_current_req_entry = req_entry;

	/*
	 * Configure the AUX control register (10bit ADC module)
	 */
	data = readl(IO_ADDRESS(AXA_R_AUXCR_MEMADDR));

	if(((data & (AXA_F_ADCOUTSEL_MASK | AXA_F_APWR_MASK)) !=
		 (AXA_F_ADCOUTSEL_MASK | AXA_F_APWR_MASK)))
	{
		data |= ((settime_value << AXA_F_SETTIME_R) |
				 AXA_F_ADCOUTSEL_MASK |
				 AXA_F_APWR_MASK);
		writel(data, IO_ADDRESS(AXA_R_AUXCR_MEMADDR));
		udelay(300);

		/*
		* Unmask AXA interrupt
		*/
		data = readl(IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
		data &= AXA_F_AXA_INT_MASK_MASK;
		writel(data, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
	}

	/*
	 * Select channel of interest
	 */
	data = readl(IO_ADDRESS(AXA_R_AUXCR_MEMADDR));
	data |= (req_entry->channel << AXA_F_AMUX_R);
	writel(data, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));

	/*
	 * Start conversion
	 */
	data = readl(IO_ADDRESS(AXA_R_AUXCR_MEMADDR));
	data |= AXA_F_START_MASK;
	writel(data, IO_ADDRESS(AXA_R_AUXCR_MEMADDR));

	return 0;
}


/*****************************************************************************
 ** BCM4760 ADC helpers
 **/
static inline void call_handler(unsigned short sample, int error)
{
	bcm_adc_request_entry_t* req_entry;

	/* the handler may immediately request another sample */
	req_entry = bcm4760_adc_current_req_entry;
	bcm4760_adc_current_req_entry = NULL;

	if(req_entry && bcm4760_adc_completion_handler)
	{
//		printk("BCM4760-ADC: Calling caller's completion handler\n");
		if (bcm4760_adc_completion_handler(req_entry, sample, error))
		{
			printk("BCM4760-ADC: BCM-ADC completion not handled\n");
		}
	}
}

/*****************************************************************************
 ** BCM4760 ADC Delayed Task Work
 ** No need to check here for suspended because there's a sample and otherwise the
 ** user gets an error.
 **/
static inline void bcm4760_adc_do_work(struct work_struct* work)
{
	unsigned short sample;

	/* Read the value for our request */
	sample = AXA_F_AUXD_GET(readl(IO_ADDRESS(AXA_R_AUXDR1_MEMADDR)));
//	printk("BCM4760-ADC: Delayed Response of 0x%x\n", sample);

	call_handler(sample, 0);
}

static inline void clear_interrupt(void)
{
	uint32_t data = readl(IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
	data |= AXA_F_AXA_INT_CLEAR_MASK;
	writel(data, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
}

/*****************************************************************************
 ** BCM4760 ADC Interrupt Service Routine
 **/
irqreturn_t bcm4760_adc_interrupt(
	int irq,
	void* dev_id)
{
//	printk("BCM4760-ADC: Interrupt received\n");

	/*
	 * Check for a work queue
	 */
	if (WARN_ON(!bcm4760_adc_workQueue))
	{
		return IRQ_NONE;
	}

	clear_interrupt();

	/*
	 * Delay queue the work to be done in a task
	 */
	if (queue_work(bcm4760_adc_workQueue, &bcm4760_adc_work) == 0)
	{
		panic("BCM4760-ADC: Work previously queued.\n");
		return IRQ_NONE;
	}

//	printk("BCM4760-ADC: Interrupt Handled\n");
	return IRQ_HANDLED;
}


static int bcm4760adc_probe(struct platform_device *pdev)
{
	uint32_t	data;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		printk(KERN_ERR PFX "no IRQ resource specified\n");
		return -ENOENT;
	}

	bcm4760_adc_irq = res->start;

	/*
	 * Setup a workqueue for sending responses back to bcm_adc
	 */
	bcm4760_adc_workQueue = create_singlethread_workqueue("bcm4760_adc_work");
	if (!bcm4760_adc_workQueue)
	{
		printk("BCM4760_ADC: Initialization error creating work queue\n");
		return 1;
	}

	INIT_WORK(&bcm4760_adc_work, bcm4760_adc_do_work);

	/*
	 * Register the ISR
	 */
	data = AXA_F_AXA_INT_MASK_MASK	| AXA_F_AXA_INT_CLEAR_MASK;
	writel(data, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));

	if (request_irq(bcm4760_adc_irq,
					bcm4760_adc_interrupt,
					0,
					"axa",
					NULL))
	{
		printk("BCM4760_ADC: Initialization error retrieving irq\n");
		return 1;
	}

	/*
	 * Mask AXA interrupt
	 */
	data = readl(IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
	data |= AXA_F_AXA_INT_MASK_MASK;
	writel(data, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));

	/*
	 * Configure the AUX control register (10bit ADC module)
	 */
	data = readl(IO_ADDRESS(AXA_R_AUXCR_MEMADDR));
	data |= ((settime_value << AXA_F_SETTIME_R) |
				AXA_F_ADCOUTSEL_MASK |
				AXA_F_APWR_MASK);
	writel(data, IO_ADDRESS(AXA_R_AUXCR_MEMADDR));

	return 0;
}

static int bcm4760adc_remove(struct platform_device *dev)
{
	printk("BCM4760-ADC: Unloading BCM4760-ADC Driver\n");

	/*
	 * Clean up work queue
	 */
	flush_workqueue(bcm4760_adc_workQueue);
	destroy_workqueue(bcm4760_adc_workQueue);

	/*
	 * Clean up ISR
	 */
	writel(0, IO_ADDRESS(AXA_R_AUXINT_MEMADDR));
	free_irq(bcm4760_adc_irq, NULL);
	return 0;
}

static int bcm4760adc_suspend(struct platform_device *dev, pm_message_t state)
{
	// deny new requests
	suspended = 1;

	// stop the rest of the driver
	disable_irq(bcm4760_adc_irq);
	cancel_work_sync(&bcm4760_adc_work);

	// all requests must have their handler called
	call_handler(0, -EAGAIN);

	return 0;
}

static int bcm4760adc_resume(struct platform_device *dev)
{
	// suppose a request was entered right before the suspend
	// and the IRQ got disabled just before it went off
	clear_interrupt();

	// allow the driver to work properly
	enable_irq(bcm4760_adc_irq);
	suspended = 0;

	return 0;
}

static void bcm4760adc_shutdown(struct platform_device *dev)
{
}

static struct platform_driver bcm4760adc_driver = {
	.probe		= bcm4760adc_probe,
	.remove		= bcm4760adc_remove,
	.shutdown	= bcm4760adc_shutdown,
	.suspend	= bcm4760adc_suspend,
	.resume		= bcm4760adc_resume,
	.driver		= {
		.owner  = THIS_MODULE,
		.name   = "bcm4760-adc",
	},
};

static char banner[] __initdata =
	KERN_INFO PFX "BROADCOM BCM476X ADC driver\n";

/*****************************************************************************
 ** BCM4760 ADC Initialization
 **/
static int __init bcm_adc_module_init(void)
{
	printk(banner);
	return platform_driver_register(&bcm4760adc_driver);
}
static void __exit bcm_adc_module_exit(void)
{
	platform_driver_unregister(&bcm4760adc_driver);
}
rootfs_initcall(bcm_adc_module_init);
module_exit(bcm_adc_module_exit);
