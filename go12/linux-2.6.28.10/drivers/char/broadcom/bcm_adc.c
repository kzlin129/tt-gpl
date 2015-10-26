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
/* linux/drivers/char/bcm_adc.c
 *
 * Implementation of the ADC driver.
 *
 * Changelog:
 *
 * 17-Jun-2009  DB   Initial version for refs #811:
 */                  

/* Includes */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <linux/broadcom/bcmtypes.h>
#include <linux/broadcom/bcm_adc.h>
#include <linux/broadcom/adc.h>
#include <linux/broadcom/bcm_major.h>

#define BCM_ADC_DRIVER			"bcm_adc"
#define BCM_ADC_MOD_DESCRIPTION		"BCM ADC Driver"
#define BCM_ADC_MOD_VERSION		3.0

#define BCM_DEVNAME			BCM_ADC_DRIVER
#define PFX				"BCM_ADC: "

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
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)
#define DBG_DEFAULT_LEVEL	(DBG_ERROR)

static const int gLevel = DBG_DEFAULT_LEVEL;
#	undef ADC_DEBUG

#	define ADC_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define ADC_DEBUG(level,x)
#endif

#define ADC_DRV_DEV_NAME   "user-adc"

static  dev_t           gAdcDrvDevNum = MKDEV(BCM_AUXADC_MAJOR, 0);
static  struct class   *gAdcDrvClass = NULL;
static  struct  cdev    gAdcDrvCDev;

static int bcm_adc_is_initialized = FALSE;

static DEFINE_MUTEX(bcm_adc_mutex);

typedef struct bcm_adc_info_s
{
	struct list_head	head;
	int			min_channel;
	int			max_channel;
	int			busy;
	int (*adc_request)(bcm_adc_request_entry_t* req);
	int (*adc_handler_register)(bcm_adc_handler_t adc_handler);
} bcm_adc_info_t;

bcm_adc_info_t bcm_adc_info[] =
{
	{	// BCM4760
		.head				= { NULL, NULL },
		.min_channel			= BCM4760_ADC_MIN_CHANNEL,
		.max_channel			= BCM4760_ADC_MAX_CHANNEL,
		.adc_request			= bcm4760_adc_request,
		.adc_handler_register		= bcm4760_adc_complete_register
	},
#if defined(CONFIG_PMU_DEVICE_BCM59040) || defined(CONFIG_BCM_PMU_BCM59040)
	{	// BCM59040
		.head				= { NULL, NULL },
		.min_channel			= BCM59040_ADC_MIN_CHANNEL,
		.max_channel			= BCM59040_ADC_MAX_CHANNEL,
		.adc_request			= bcm59040_adc_request,
		.adc_handler_register		= bcm59040_adc_complete_register
	}
#endif
};



static void bcm_adc_start_next_request(bcm_adc_info_t* dev_info)
{
	struct list_head	*tlist, *entry;
	int			error = 0;

	WARN_ON_ONCE(!mutex_is_locked(&bcm_adc_mutex));
	WARN_ON_ONCE(dev_info->busy);

	list_for_each_safe(entry, tlist, &dev_info->head) {
		bcm_adc_request_entry_t	*next_req_entry = list_entry(entry, bcm_adc_request_entry_t, list);
		bcm_adc_request_t	*next_req = next_req_entry->request;

		ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "starting next request %p\n", next_req_entry));

		/* if the request was already canceled then request is not set */
		if (next_req)
			error = dev_info->adc_request(next_req_entry);
		if (!next_req || error) {
			list_del_init(&next_req_entry->list);
			if (next_req) {
				ADC_DEBUG(DBG_ERROR, (KERN_ERR PFX "new request failed, completing bad request!\n"));
				next_req->callback(next_req, 0, error);
			}
			kfree(next_req_entry);
		} else {
			dev_info->busy = 1;
			break;
		}
	}
}

static bcm_adc_request_entry_t *bcm_adc_find_request(bcm_adc_info_t* dev_info, bcm_adc_request_t *req)
{
	struct list_head	*entry;
	bcm_adc_request_entry_t	*next_req_entry;

	WARN_ON_ONCE(!mutex_is_locked(&bcm_adc_mutex));

	__list_for_each(entry, &dev_info->head) {
		next_req_entry = list_entry(entry, bcm_adc_request_entry_t, list);
		if (next_req_entry->request == req)
			return next_req_entry;
	}
	return NULL;
}

/** bcm_adc_complete_handler - Handle ADC completions
 *
 * Handle conversion request completion events. Try to
 * associate them with the proper low-level ADC driver
 * and dispatch to it's callback if identified as one of
 * the ADCs we a responsible for servicing.
 *
 * @req:		ADC Request from caller.
 * @sample		sample value from low-level ADC driver
 * @error:		error code from low-level ADC driver.
 * 
 * Returns: FALSE if the event was not handled
 */
static int bcm_adc_handler(bcm_adc_request_entry_t* req_entry,
			   unsigned short sample,
			   int error)
{
	int			handled;
	bcm_adc_request_t	*req;
	bcm_adc_info_t		*dev_info;

	mutex_lock(&bcm_adc_mutex);

	req = req_entry->request;
	dev_info = req_entry->dev_info;

	ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "completion Handler called with sample of 0x%04X for req %08X\n",
						 sample,
						 (unsigned int)req_entry));

	if (!req) {
		/* it was canceled */
		handled = TRUE;
		dev_info->busy = 0;
	} else {

		/*
		 * If the requested device was working on the responses
		 * request, then this is the right one... complete it.
		 */
		if((!list_empty(&dev_info->head)) &&
		   (list_first_entry(&dev_info->head, bcm_adc_request_entry_t, list) == req_entry)) {

			handled = TRUE;
			dev_info->busy = 0;

			ADC_DEBUG(DBG_TRACE, (KERN_INFO PFX "completion handler calling callback\n"));

			list_del_init(&req_entry->list);
			req->callback(req, sample, error);
			kfree(req_entry);
		} else {
			/* this should NEVER happen  */
			WARN_ON_ONCE(1);
			handled = FALSE;
		}
	}

	if (!dev_info->busy) {
		bcm_adc_start_next_request(dev_info);
	}

	mutex_unlock(&bcm_adc_mutex);

	/* Tell the lower level ADC caller if this was handled */
	return handled;
}

/** bcm_adc_init - Initialize Broadcom ADC driver structures
 *
 * Initialize the ADC lists and devices
 *
 * Returns 0 for success or Linux error code if failure.
 *
 */
static int bcm_adc_init(void)
{
	int adc_device;

	ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "initialization\n"));

	for(adc_device = 0; adc_device < BCM_ADC_DEVICES; adc_device++) {
		bcm_adc_info_t* dev_info = &bcm_adc_info[adc_device];

		INIT_LIST_HEAD(&dev_info->head);

		dev_info->adc_handler_register(bcm_adc_handler);
	}

	bcm_adc_is_initialized = TRUE;
	return 0;
}

/** bcm_adc_request - Make a request to an ADC device/channel
 *
 * Queue up a request to one of the ADC devices.
 *
 * @req:	structure containing ADC request.
 *
 * Return: Error or 0
 */
int bcm_adc_request(bcm_adc_request_t* req)
{
	bcm_adc_request_entry_t	*req_entry;
	bcm_adc_info_t		*dev_info;

	if(!bcm_adc_is_initialized) {
		bcm_adc_init();
	}

	if (!req->callback) {
		return -EINVAL;
	}
	if ((req->device < 0) || (req->device >= BCM_ADC_DEVICES)) {
		return -ENODEV;
	}

	dev_info = &bcm_adc_info[req->device];

	if((req->channel < dev_info->min_channel) ||
	   (req->channel > dev_info->max_channel)) {
		return -ECHRNG;
	}

	/* allocate outside the mutex */
	req_entry = kzalloc(sizeof(*req_entry), GFP_KERNEL);
	if (!req_entry)
		return -ENOMEM;
	req_entry->request = req;
	req_entry->dev_info = dev_info;
	req_entry->channel = req->channel;

	ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "request received (%d/%d/%08lX) is entry %p\n",
						 req->device,
						 req->channel,
						 req->param,
						 req_entry));

	mutex_lock(&bcm_adc_mutex);

	WARN_ON(bcm_adc_find_request(dev_info, req));

	list_add_tail(&req_entry->list, &dev_info->head);
	if(!dev_info->busy) {
		bcm_adc_start_next_request(dev_info);
	}

	mutex_unlock(&bcm_adc_mutex);

	return 0;
}

/** bcm_adc_cancel_request - Cancel a request to an ADC device/channel
 *
 * Dequeue/cancel a request previously made to one of the ADC devices.
 *
 * @req:	structure containing ADC request to be cancelled.
 *
 * Return: 0 if request found and removed, Linux error code otherwise.
 */
int bcm_adc_cancel_request(bcm_adc_request_t* req)
{
	bcm_adc_info_t		*dev_info;
	bcm_adc_request_entry_t	*req_entry;

	ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "cancel received for request %08X\n",
						 (unsigned int)req));

	if(!bcm_adc_is_initialized) {
		bcm_adc_init();
		printk("bcm_adc not initialized\n");
		return -ENOMSG;
	}

	if((req->device < 0) || (req->device >= BCM_ADC_DEVICES)) {
		return -ENODEV;
	}
	dev_info = &bcm_adc_info[req->device];
	if ((req->channel < dev_info->min_channel) || (req->channel > dev_info->max_channel)) {
		return -ECHRNG;
	}

	mutex_lock(&bcm_adc_mutex);

	req_entry = bcm_adc_find_request(dev_info, req);

	/* Either the request got handled between when the client decided to cancel
	 * and now, or problem in the user; how can it cancel something not requested.
	 */
	if (!req_entry) {
		mutex_unlock(&bcm_adc_mutex);
		return -ENOMSG;
	}

	/* the cancel itself */
	req_entry->request = NULL;

	mutex_unlock(&bcm_adc_mutex);

	return 0;
}

/** bcm_adc_ioctl - Perform special operations to Broadcom ADC driver
 *
 * @inode:		inode reference
 * @file:		file path
 * @cmd:		IOCTL command code
 * @arg:		IOCTL argument
 *
 * Returns 0 for successful or Linux error code otherwise.
 *	
 */
static int bcm_adc_ioctl(struct inode *inode,
						 struct file *file,
						 unsigned int cmd,
						 unsigned long arg)
{
	int					ret = -ENOIOCTLCMD;
	int					device;
	brcm_adc_req_t		req;

	ADC_DEBUG(DBG_INFO, (KERN_INFO PFX "ADC driver IOCTL called.\n"));

	switch (cmd) {
	case BRCM_ADC_IOCTL_GET_NUM_DEVICES:
	{
		ret = BCM_ADC_DEVICES;
		break;
	}

	case BRCM_ADC_IOCTL_GET_NUM_CHANNELS:
	{
		device = (int)arg;
		if(device == BCM4760_ADC_DEVICE) {
			ret = BCM4760_ADC_MAX_CHANNEL+1;
#if defined(CONFIG_PMU_DEVICE_BCM59040) || defined(CONFIG_BCM_PMU_BCM59040)
		} else if(device == BCM59040_ADC_DEVICE) {
			ret = BCM59040_ADC_MAX_CHANNEL+1;
#endif
		} else {
			ret = -EINVAL;
		}
		break;
	}

	case BRCM_ADC_IOCTL_GET_CAPS:
	{
		ret = -EINVAL;
		break;
	}

   case BRCM_ADC_IOCTL_REQUEST:
	{
		if(copy_from_user(&req,
						  (void __user *)arg,
						  sizeof(req))) {
			ret = -EFAULT;
		} else {
			ret = bcm_adc_read(req.device, req.channel);
			if(ret >= 0) {
				req.sample = (unsigned short)ret;
				ret = copy_to_user((void __user *)arg,
								   &req,
								   sizeof(req));
			}
		}
		break;
	}

   case BRCM_ADC_IOCTL_CANCEL_REQUEST:
	{
		if(copy_from_user(&req,
						  (void __user *)arg,
						  sizeof(req))) {
			ret = -EFAULT;
		} else {
			ret = -EINVAL;
			if(ret >= 0) {
				req.sample = (unsigned short)ret;
				ret = copy_to_user((void __user *)arg,
								   &req,
								   sizeof(req));
			}
		}
		break;
	}

    default:
      break;

	}
	return ret;
}

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations adc_fops =
{
    owner:      THIS_MODULE,
    ioctl:      bcm_adc_ioctl,
};

/*****************************************************************************
 * Module routines
 */

/** bcm_adc_module_init - ADC module initialization function
 *
 * Returns 0 if successful, Linux error code otherwise.
 *
 */
static int __init bcm_adc_module_init(void)
{
	int			rc=0;

	printk("BCM-ADC: Loading BCM-ADC Driver\n");

	if(MAJOR(gAdcDrvDevNum) == 0)
	{
		// Allocate a major number dynaically

		if(( rc = alloc_chrdev_region(&gAdcDrvDevNum, 0, 1, ADC_DRV_DEV_NAME)) < 0)
		{
			printk(KERN_WARNING PFX "alloc_chrdev_region failed; err: %d\n", rc);
			return rc;
		}
	}
	else
	{
		// Use the statically assigned major number

		if((rc = register_chrdev_region( gAdcDrvDevNum, 1, ADC_DRV_DEV_NAME)) < 0)
		{
			printk(KERN_WARNING PFX "register_chrdev failed for major %d; err: %d\n", BCM_AUXADC_MAJOR, rc);
			return rc;
		}
	}

	cdev_init(&gAdcDrvCDev, &adc_fops);
	gAdcDrvCDev.owner = THIS_MODULE;

	if((rc = cdev_add( &gAdcDrvCDev, gAdcDrvDevNum, 1)) != 0)
	{
		printk( KERN_WARNING PFX "cdev_add failed: %d\n", rc );
		goto out_unregister;
	}

	// Now that we've added the device, create a class, so that udev will make the /dev entry

	gAdcDrvClass = class_create(THIS_MODULE, ADC_DRV_DEV_NAME);
	if(IS_ERR(gAdcDrvClass))
	{
		printk( KERN_WARNING "gpio: Unable to create class\n" );
		rc = -1;
		goto out_cdev_del;
	}

	device_create(gAdcDrvClass, NULL, gAdcDrvDevNum, NULL, ADC_DRV_DEV_NAME);

	goto done;

out_cdev_del:
	cdev_del(&gAdcDrvCDev);

out_unregister:
	unregister_chrdev_region(gAdcDrvDevNum, 1);

done:
	return 0;
}

/** bcm_adc_module_exit - ADC module exit/cleanup function
 *
 */
static void __exit bcm_adc_module_exit(void)
{
	printk("BCM-ADC: Unloading BCM-ADC Driver\n");
}

module_init(bcm_adc_module_init);
module_exit(bcm_adc_module_exit);

MODULE_DESCRIPTION(BCM_ADC_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM_ADC_MOD_VERSION);
