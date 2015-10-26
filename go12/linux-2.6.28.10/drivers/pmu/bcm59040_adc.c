/*****************************************************************************
 * Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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

/** BCM59040 ADC Driver
 *****************************************************************************
 *
 *     This implements the BCM59040 PMU ADC driver layer. This layer is the
 *     low-level ADC hardware driver for the BCM59040. In the Broadcom ADC
 *     stack there is an upper layer driver that provides the API to other
 *     drivers. It is not intended that drivers other than the generic
 *     upper layer Broadcom ADC driver interact directly with this driver.
 *
 *     It functions by scheduling ADC integrations and waiting for the
 *     interrupts from the PMU indicating that the integration has completed.
 *     When it receives such interrupts it schedules a workqueue to trigger
 *     and the workqueue thread then processes the event eventually calling
 *     the callback function to the generic ADC layer.
 *
 ****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#include <linux/platform_device.h>
#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>

#include <linux/broadcom/bcm_adc.h>
#include <linux/broadcom/pmu_bcm59040.h>

/* Include external interfaces */

#include <asm/arch/bcm59040_irqs.h>

//#include <linux/pmu/bcm59040_core.h> // not sure if this is needed yet

/* Include internal interfaces */


/*
 * Structure and macro definitions.
 */

#define BCM59040_ADC_DRIVER				"bcm59040_adc"
#define BCM59040_ADC_MOD_DESCRIPTION	"BCM59040 ADC Driver"
#define BCM59040_ADC_MOD_VERSION		1.0

#undef BCM59040_DEVNAME
#define BCM59040_DEVNAME					BCM59040_ADC_DRIVER
#define PFX									BCM59040_DEVNAME ": "

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#ifdef DEBUG
#define DBG_DEFAULT_LEVEL	DBG_ERROR
#define DBG_DEBUGGING_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)

static const int gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

/** struct bcm59040_adc_config_struct - Driver data for BCM59040 ADC
 * @adc_handler_cb:     Callback function to generic ADC driver stack.
 * @current_req:        Currently active request from generic ADC driver.
 * @dev:                Pointer to associated device.
 * @work:               Work generated from ISR.
 * @workqueue:          Work queue for foreground ISR processing of ADC interrupts.
 *
 */
typedef struct bcm59040_adc_config_struct
{
	bcm_adc_handler_t			adc_handler_cb;
	bcm_adc_request_entry_t			*current_req_entry;
	struct pmu_client			*pclient;
	struct work_struct			work;
	struct workqueue_struct			*workqueue;
	int					suspended;
} bcm59040_adc_config_t;

/*
 * Local variables.
 */

/*
 * Local function declarations.
 */


/** bcm59040_adc_async_mode_control - Put BCM59040 ADC in async measurement mode.
 * @dev:    Pointer to PMU device structure.
 *
 */
static int bcm59040_adc_async_mode_control(struct pmu_client *pclient)
{
	int			rc;
	uint8_t		data;

	/* configure ADCCTRL6 to make sure SS_MASK is disabled so SS requests can be responded */
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_ADCCTRL6, &data, 1)) < 0)
	{
		printk (KERN_ERR PFX "error reading BCM59040_REG_ADCCTRL6 register.\n");
		return rc;
	}

	data &= ~BCM59040_ADCCTRL6_SS_MASK_EN;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_ADCCTRL6, &data, 1)) < 0)
	{
		printk (KERN_ERR PFX "error writing BCM59040_REG_ADCCTRL6 register.\n");
		return rc;
	}

	/* configure ADCCTRL7 */
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_ADCCTRL7, &data, 1)) < 0)
	{
		printk (KERN_ERR PFX "error reading BCM59040_REG_ADCCTRL7 register.\n");
		return rc;
	}
	data |= BCM59040_ADCCTRL7_I2CSS_EN;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_ADCCTRL7, &data, 1 )) < 0)
	{
		printk (KERN_ERR PFX "error writing BCM59040_REG_ADCCTRL7 register.\n");
		return rc;
	}

	return 0;
}

/** bcm59040_adc_latch_data - Latch ADC measurement from integration in readable register.
 * @dev:    Pointer to PMU device structure.
 * @channel:    Channel to latch ADC data for.
 *
 */
static int bcm59040_adc_latch_data(struct pmu_client *pclient, int channel)
{
	int			rc;
	uint8_t		data;

	data = BCM59040_ADCCTRL7_CHAN_SNAPSHOT_MASK & channel;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_ADCCTRL7, &data, 1 )) < 0)
	{
		printk (KERN_ERR PFX "error writing BCM59040_REG_ADCCTRL7 register.\n");
		return rc;
	}
	return 0;
}

/** bcm59040_adc_read_data - Read data from BCM59040 register that has latched copy from ADC.
 * @dev:            Pointer to PMU device structure.
 * @combined_data:  Pointer to location to store copy of ADC data.
 *
 *  Read latched raw data from ADC_DATA registers, which was latched by a prior
 *  i2c command.
 *
 */
static int bcm59040_adc_read_data(struct pmu_client *pclient, unsigned short *combined_data)
{
	int rc;
	u8 adc_data[2];

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_ADCCTRL8, &adc_data, 2)) < 0)
	{
		printk (KERN_ERR PFX "error writing BCM59040_REG_ADCCTRL8 register.\n");
		return rc;
	}
	*combined_data = (u16) (((adc_data[0] & 0x03) << 8) | adc_data[1]);

	return 0;
}

/*
 * Function definitions.
 */

/* bcm59040_adc_request - Start ADC request with BCM59040 hardware.
 * @dev:    Pointer to PMU device structure.
 * @req:    Pointer to ADC request
 *
 * First we 
 */

int bcm59040_adc_request(bcm_adc_request_entry_t *req_entry)
{
	struct device		*dev;
	struct pmu_client	*pclient;
	int 			rc;
	bcm59040_adc_config_t	*adc_config;

	/*
	 * Find device by name using device number passed in req
	 * structure by high level ADC driver.
	 */

	dev = bus_find_device_by_name(&pmu_bus_type, NULL, "bcm59040-adc");

	if(dev)
	{
		PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "found ADC device with lookup.\n"));

		pclient = container_of(dev, struct pmu_client, dev);

		adc_config = pmu_get_drvdata(pclient);
		if(!adc_config)
		{
			printk(KERN_ERR "error -ENOMEM\n");
			return -ENOMEM;
		}
		WARN_ON( adc_config->pclient != pclient );
	
		if (adc_config->suspended) {
			PMU_DEBUG(DBG_ERROR, (KERN_INFO PFX "Denying request because suspended.\n"));
			return -EAGAIN;
		}

		// this happens every now and then; it should not because it means a missed interrupt!!!
		WARN_ON(adc_config->current_req_entry != NULL);

		// better set the request before there's any chance that isr will get called
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "New request %p %p %p.\n", req_entry, pclient, adc_config));
		adc_config->current_req_entry = req_entry;
		if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_ADCCTRL1, &req_entry->channel, 1)) < 0)
		{
			printk(KERN_ERR "error writing to ADCCTRL1 register (%d)\n", rc);
			return rc;
		}
		bcm59040_adc_async_mode_control(pclient);

		return 0;
	}
	else
	{
		printk(KERN_CRIT "lookup of ADC device failed!\n");

		return -ENXIO;
	}
}

/* bcm59040_adc_complete_register - Register callback to generic ADC driver.
 * @dev:                Pointer to PMU device structure.
 * @adc_completion:     Pointer to ADC request
 *
 * First we 
 */

int bcm59040_adc_complete_register(bcm_adc_handler_t adc_completion)
{
	struct device			*dev;
	struct pmu_client		*pclient;
	bcm59040_adc_config_t	*adc_config;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "registering completion function for bcm59040_adc.\n"));

	/*
	 * Find device by name using device number passed in req
	 * structure by high level ADC driver.
	 */

	dev = bus_find_device_by_name(&pmu_bus_type, NULL, "bcm59040-adc");
	if(dev)
	{
		pclient = container_of(dev, struct pmu_client, dev);

		adc_config = pmu_get_drvdata(pclient);
		if(!adc_config)
		{
			printk("ADC ENOMEM\n");
			return -ENOMEM;
		}
	
		adc_config->adc_handler_cb = adc_completion;

		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "registration completed successfully.\n"));
		return 0;
	}
	else
	{
		printk(KERN_CRIT "lookup of ADC device failed!\n");

		return -ENXIO;
	}
}

static void bcm59040_adc_call_callback(bcm59040_adc_config_t *adc_config,
		unsigned short sample,
		int error)
{
	if(adc_config->current_req_entry)
	{
		// cache the current request so the callback is free to put in the next one
		bcm_adc_request_entry_t *tmp_req_entry = adc_config->current_req_entry;
		adc_config->current_req_entry = NULL;

		(*adc_config->adc_handler_cb)(tmp_req_entry, sample, error);
		PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "Callback done (sample %d, error %d).\n", sample, error));
	}
}

/** bcm59040_adc_work - Work function for BCM59040 ADC IRQ
 * @work:   Pointer to work structure.
 *
 * No need to check for ->suspended here because there's a good sample
 * to deliver. The alternative is that the user will get an error.
 */
static void bcm59040_adc_work(struct work_struct *work)
{
	int			rc;
	uint8_t			data, intbit;
	unsigned short		adc_data;
	bcm59040_adc_config_t	*adc_config;
	struct pmu_client	*pclient;

	adc_config = container_of(work, bcm59040_adc_config_t, work);
	pclient = adc_config->pclient;
	
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT9, &data, 1 )) < 0)
	{
		printk(KERN_ERR PFX "error reading BCM59040_REG_VINT9 register.\n");

		/* clear the interrupt */
		pmu_bus_set_bit(pclient, BCM59040_REG_VINT2, BCM59040_VINT2_ADC_BIT );
		return;
	}

	/* clear the interrupt */
	intbit = 1 << BCM59040_VINT2_ADC_BIT;
	pmu_bus_seqwrite(pclient, BCM59040_REG_VINT2, &intbit, 1 );

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Read %d bytes (%02X) from virtual int status registers.\n", rc, data));
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Work for %p %p %p.\n", adc_config->current_req_entry, pclient, adc_config));

	/* apparently it happens that there's an interrupt from CONVEND without a request */
	WARN_ON(adc_config->current_req_entry == NULL);
	if(adc_config->current_req_entry && (data & BCM59040_INT9_SARCONVEND))
	{
		int channel = adc_config->current_req_entry->channel;
		if(!bcm59040_adc_latch_data(pclient,channel) &&
		   !bcm59040_adc_read_data(pclient, &adc_data)) {
			bcm59040_adc_call_callback(adc_config, adc_data, 0);
		}
		else {
			bcm59040_adc_call_callback(adc_config, 0, -EIO);
		}
	}

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Work done.\n"));
}

/** bcm59040_adc_isr - Interrupt service routine for BCM59040 ADC
 * @irq:    IRQ vector number
 * @dev_id: Data structure associated with this interrupt event.
 *
 * This ISR handles events for the ADC in the BCM59040. The primary
 * event we're interested in is the ADC completion interrupt.
 *
 */
static irqreturn_t bcm59040_adc_isr(int irq, void *dev_id)
{
	bcm59040_adc_config_t	*adc_config =(bcm59040_adc_config_t*) dev_id;

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "ISR.\n"));
	queue_work(adc_config->workqueue, &adc_config->work);

	return IRQ_HANDLED;
}

/** bcm59040_adc_probe - BCM59040 adc probe for present.
 *
 * Probe to determine if BCM59040 is really present.
 *
 * The ADC hardware needs to be initialized to a default/known state. We also
 * need to create the workqueue so we can dispatch events to the upper layer
 * generic ADC driver as well. The callback that the upper layer generic
 * ADC driver has registered with us shouldn't really be called at ISR
 * time since we can't predict what the function might eventually do someday.
 * Thus calling it from a workqueue thread is safer.
 *
 * @pclient:		Pointer to PMU Client structure.
 * @id:				id table.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static int bcm59040_adc_probe(struct pmu_client *pclient,
		const struct pmu_device_id *id)
{
	int			rc;
	bcm59040_adc_config_t	*adc_config;

	if(!(adc_config = kmalloc(sizeof(*adc_config), GFP_KERNEL)))
	{
		printk (KERN_ERR PFX "Out of memory allocating adc_config\n");
		return -ENOMEM;
	}

	adc_config->adc_handler_cb = NULL;	/* No callback registered yet */
	adc_config->current_req_entry = NULL;		/* No ADC operation active yet */
	adc_config->suspended = 0;

	adc_config->workqueue = create_workqueue(BCM59040_ADC_DRIVER);
	adc_config->pclient = pclient;

	INIT_WORK(&adc_config->work, &bcm59040_adc_work);

	pmu_set_drvdata(pclient, adc_config);

	if((rc = request_irq(BCM59040_IRQ_ADC,
			bcm59040_adc_isr,
			0,
			"bcm59040_adc",
			adc_config)) < 0) {
        	printk (KERN_ERR PFX "bcm59040_adc_probe failed to attach interrupt, rc = %d\n", rc);
        	kfree(adc_config);
		pmu_set_drvdata(pclient, NULL);
		return rc;
	}

	rc = pmu_bus_clear_bit(pclient, BCM59040_REG_VINT2M, BCM59040_VINT2_ADC_BIT );
	if(rc < 0)
	{
		printk (KERN_ERR PFX "error unmasking interrupt.\n");
		free_irq(BCM59040_IRQ_ADC, adc_config);
        	kfree(adc_config);
		pmu_set_drvdata(pclient, NULL);
		return rc;
	}

	return 0;
}

/** bcm59040_adc_remove - BCM59040 adc remove function.
 *
 * Need to remove BCM59040 adc driver.
 *
 * @pclient:		Pointer to PMU Client structure.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_adc_remove(struct pmu_client *pclient)
{
	bcm59040_adc_config_t			*adc_config;

	adc_config = pmu_get_drvdata(pclient);
	if(adc_config)
	{
		free_irq(BCM59040_IRQ_ADC, adc_config);
		kfree(adc_config);
	}

	return 0;
}

#ifdef CONFIG_PM

/** bcm59040_adc_suspend - Suspend ADC functioning.
 *
 * ADC suspend handler that is called when sysem is entering
 * a suspend state.
 *
 * @pclient:	Pointer to client structure for this device.
 * @state:		PM state being entered.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */
static int bcm59040_adc_suspend(struct pmu_client *pclient, pm_message_t state)
{
	bcm59040_adc_config_t	*adc_config = pmu_get_drvdata(pclient);

	/* deny new requests */
	adc_config->suspended = 1;

	/* stop the rest of the driver */
	disable_irq(BCM59040_IRQ_ADC);
	cancel_work_sync(&adc_config->work);

	bcm59040_adc_call_callback(adc_config, 0, -EAGAIN);

	PMU_DEBUG(DBG_INFO, (KERN_INFO "Suspend processed for bcm59040-%s\n", pclient->name));

	return 0;
}

/** bcm59040_adc_resume - Resume ADC functioning.
 *
 * ADC resume handler that is called when sysem is exiting
 * a suspend state.
 *
 * @pclient:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */
static int bcm59040_adc_resume(struct pmu_client *pclient)
{
	bcm59040_adc_config_t	*adc_config = pmu_get_drvdata(pclient);

	enable_irq(BCM59040_IRQ_ADC);
	adc_config->suspended = 0;

	PMU_DEBUG(DBG_INFO, (KERN_INFO "Resume processed for bcm59040-%s\n", pclient->name));
	return 0;
}

#endif

struct pmu_device_id bcm59040_adc_idtable[] = {
	{ "adc" },
	{ }
};

/** @brief BCM59040 adc driver registration data
 *
 */

struct pmu_driver bcm59040_adc_driver =
{
	.driver	= {
		.name	= BCM59040_ADC_DRIVER,
		.owner	= THIS_MODULE,
	},
	.probe		= &bcm59040_adc_probe,
	.remove		= &bcm59040_adc_remove,
#ifdef CONFIG_PM
	.suspend	= &bcm59040_adc_suspend,
	.resume		= &bcm59040_adc_resume,
#endif
	.id_table	= bcm59040_adc_idtable,
};

/** @brief BCM59040 adc module Initialization
 *
 * Registers the BCM59040 adc driver with the PMU device
 * framework.
 *
 * @return 0 if successful otherwise standard Linux error codes.
 *
 */

static int __init bcm59040_adc_init(void)
{
	return pmu_register_driver(&bcm59040_adc_driver);
}

/** @brief BCM59040 adc module Exit
 *
 * Deregisters the BCM59040 adc driver with the PMU device
 * framework.
 *
 * @return 0 if successful otherwise standard Linux error codes.
 *
 */

static void __exit bcm59040_adc_exit(void)
{
	pmu_unregister_driver(&bcm59040_adc_driver);
}


device_initcall(bcm59040_adc_init);
module_exit(bcm59040_adc_exit);

MODULE_DESCRIPTION(BCM59040_ADC_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM59040_ADC_MOD_VERSION);
