/* drivers/barcelona/usbmode/usbmode.c
Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
Author: Kees Jongenburger <kees.jongenburger@tomtom.com>
        Balazs Gerofi <balazs.gerofi@tomtom.com>

Driver to perform probing usb of mode. This driver has two functionalities.
The first is to be able to detect the current usb mode. the second
is to be able to switch between "idle" , usb device and usb host mode.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.
*/

/* Normal kernel includes */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>

#include <barcelona/usbmode.h>
#include <barcelona/gopins.h>
/*#include "powermon.h"*/

/* Defines */
#define PFX "usbmode:"
#define PK_DBG PK_DBG_FUNC

#define DRIVER_DESC_LONG "TomTom GO Legacy USB mode detect, (C) 2007 TomTom BV "

const char *USB_STATE_STRINGS[] = {
	"USB_STATE_INITIAL",
	"USB_STATE_IDLE",
	"USB_STATE_DEVICE_DETECT",
	"USB_STATE_DEVICE",
	"USB_STATE_HOST_DETECT",
	"USB_STATE_HOST",
	"USB_STATE_CLA"
};

const char *USB_EVENT_STRINGS[] = {
	"USB_EVENT_POWER_ON",
	"USB_EVENT_POWER_OFF",
	"USB_EVENT_DEVICE_RESET",
	"USB_EVENT_DEVICE_DETECT_TIMEOUT",
	"USB_EVENT_HOST_DETECT_TIMEOUT",
	"USB_EVENT_HOST_DEVICE_ADDED",
	"USB_EVENT_HOST_DEVICE_REMOVED",
	"USB_EVENT_KERNEL_SUSPEND"
};

/**
 * sysfs entry attached to the driver to output the usbmode mode
 **/
static ssize_t usbmode_sysfs_read_mode(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct usbmode_data *data = (struct usbmode_data *)dev->driver_data;
	return sprintf(buf, "%s\n", USB_STATE_STRINGS[data->usb_state]);
}

/* define driver_attr_mode variable this will be used to define the "mode"
entry in the sysfs*/
static DEVICE_ATTR(mode, S_IWUGO | S_IRUGO, usbmode_sysfs_read_mode, NULL);

/*==== START change listener data code */
/* data members for the change listener callback */
struct state_change_listener_list_item {
	struct list_head node;
	usb_state_change_listener_func func;
	void *arg;
};
/* create a list for state_change_listeners */
static LIST_HEAD(state_change_listener_list);
/* define a lock to use when performing operations on the change listener list */
static spinlock_t state_change_listener_list_lock = SPIN_LOCK_UNLOCKED;

/*==== START state machine code */

/* Define a worker method used to handle state changes in an asynchronous manner */
void state_changed_work_handler(void *given_data)
{
	struct usbmode_data *data = (struct usbmode_data *)given_data;
	int work_preformed = 0;
	int event = data->input_events;

	if (event & INPUT_DEVICE_RESET) {
		/* we have detected a INPUT_DEVICE_RESET,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_DEVICE_RESET;

		handle_usbmode_state_event(data, USB_EVENT_DEVICE_RESET);
		work_preformed = 1;
	}

	if (event & INPUT_DEVICE_DETECT_TIMEOUT) {
		/* we have detected a INPUT_DEVICE_DETECT_TIMEOUT,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_DEVICE_DETECT_TIMEOUT;

		/*create a USB_EVENT_DEVICE_DETECT_TIMEOUT */
		handle_usbmode_state_event(data,
					   USB_EVENT_DEVICE_DETECT_TIMEOUT);
		work_preformed = 1;
	}

	if (event & INPUT_HOST_DETECT_TIMEOUT) {
		/* we have detected a INPUT_HOST_DETECT_TIMEOUT,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_HOST_DETECT_TIMEOUT;

		/*create a USB_EVENT_DEVICE_DETECT_TIMEOUT */
		handle_usbmode_state_event(data, USB_EVENT_HOST_DETECT_TIMEOUT);
		work_preformed = 1;
	}

	if (event & INPUT_HOST_DEVICE_DISCONNECT) {
		/* we have detected a INPUT_HOST_DEVICE_DISCONNECT,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_HOST_DEVICE_DISCONNECT;

		/* create a USB_EVENT_HOST_DEVICE_REMOVED */
		handle_usbmode_state_event(data, USB_EVENT_HOST_DEVICE_REMOVED);
		work_preformed = 1;
	}

	if (event & INPUT_HOST_DEVICE_CONNECT) {
		/* we have detected a INPUT_HOST_DEVICE_CONNECT,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_HOST_DEVICE_CONNECT;

		/* create a USB_EVENT_HOTPLUG */
		handle_usbmode_state_event(data, USB_EVENT_HOST_DEVICE_ADDED);
		work_preformed = 1;
	}

	if (event & INPUT_BUS_POWER_ON_EVENT) {
		/* we have detected a power on event,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_BUS_POWER_ON_EVENT;

		/* create a USB_EVENT_HOTPLUG */
		handle_usbmode_state_event(data, USB_EVENT_POWER_ON);
		work_preformed = 1;
	}

	if (event & INPUT_BUS_POWER_OFF_EVENT) {
		/* we have detected a POWER OFF EVENT,
		   clear the bit to notify that we still would like to receive
		   new events */
		data->input_events &= ~INPUT_BUS_POWER_OFF_EVENT;

		/* create a USB_EVENT_HOTPLUG */
		handle_usbmode_state_event(data, USB_EVENT_POWER_OFF);
		work_preformed = 1;
	}

	/* we expect to at least handle one event every time we are called
	   if this is not the case I guess that something is not ok (given_data corrupt?
	   log a message */
	if (work_preformed == 0) {
		printk(KERN_INFO PFX
		       " %s called but no event was emitted state %08x\n",
		       __FUNCTION__, event);
	}
}

/* Handle events */
void handle_usbmode_state_event(struct usbmode_data *data, USB_EVENT event)
{
	if (atomic_read(&data->event_barrier) == 1  && event != USB_EVENT_KERNEL_SUSPEND){
		printk(KERN_DEBUG PFX ":%s Skipping event (currently in event barrier) or event is USB_EVENT_KERNEL_SUSPEND\n",__FUNCTION__);
		return;
	}
	int previous_state = data->usb_state;
	switch (data->usb_state) {
	case USB_STATE_INITIAL:
		printk(KERN_DEBUG PFX " USB mode initial setup\n");

		if (data->u_ops->initial_to_idle(data)) {
			break;
		}

		data->usb_state = USB_STATE_IDLE;
		break;
	case USB_STATE_IDLE:
		switch (event) {
		case USB_EVENT_POWER_ON:
			/* if we are in idle mode and get power start detecting if we must behave
			   like a device */
			if (IO_HaveUsbDeviceHostCapable()) {
				if (data->u_ops->idle_to_device_detect(data)) {
					break;
				}
				
				data->usb_state = USB_STATE_DEVICE_DETECT;
			} 
			else {
				printk(KERN_DEBUG PFX
				       " Device does not support usb host going into device\n");
				if (data->u_ops->device_detect_to_device(data)) {
					break;
				}

				data->usb_state = USB_STATE_DEVICE;
			}
			break;
		case USB_EVENT_POWER_OFF:
			/* when power is on we move to at least USB_STATE_DEVICE_DETECT mode
			   so we don't expect to get a USB power off while in this mode.
			   While the above sound reasonable it it not completely true.
			   We have no hard guaranty about the order in with we receive events.
			   therefore I can happen that we first detect that the D signal have stopped
			   and therefore switch to IDLE mode. in that case we could receive a power
			   off event while in idle mode. Still I guess we do not need to do anything here */
			printk(KERN_DEBUG PFX
			       " USB power off while in idle mode. (not expected)\n");
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  in idle mode. (not expected)\n");
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			/* not expected either */
			printk(KERN_DEBUG PFX
			       " Device detect timeout  in idle mode. (not expected)\n");
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			/* not really expected but I guess it can happen if a different event (power down) puts the
			   state machine in idle mode but the host detect timeout is still running */
			printk(KERN_DEBUG PFX
			       " host detect timeout  in idle mode. (not expected)\n");
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/* don't to anything */
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			/* don't to anything */
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			if (data->u_ops->idle_to_initial(data)) {
				break;
			}
				
			data->usb_state = USB_STATE_INITIAL;
			break;
		}
		break;
	case USB_STATE_DEVICE_DETECT:
		switch (event) {
		case USB_EVENT_POWER_ON:
			printk(KERN_DEBUG PFX
			       " USB power on while in device detect detect mode. (not expected) \n");
			break;
		case USB_EVENT_POWER_OFF:
			if (data->u_ops->device_detect_to_idle(data)) {
				break;
			}

			data->usb_state = USB_STATE_IDLE;
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  while in device detect. switching to usb device mode\n");
			if (data->u_ops->device_detect_to_device(data)) {
				break;
			}

			data->usb_state = USB_STATE_DEVICE;
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " Device detect timeout  while in device detect. switching to host detect\n");
			if (data->u_ops->device_detect_to_host_detect(data)) {
				break;
			}
			
			data->usb_state = USB_STATE_HOST_DETECT;
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " Host detect timeout  while in device detect .(not expected)\n");
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/* don't to anything */
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			/* don't to anything */
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			/* upon suspend we want to go back in idle mode to make sure we
			   have no timers running etc */
			if (data->u_ops->device_detect_to_initial(data)) {
				break;
			}

			data->usb_state = USB_STATE_IDLE;
			break;
		}
		break;
	case USB_STATE_DEVICE:
		switch (event) {
		case USB_EVENT_POWER_ON:
			printk(KERN_DEBUG PFX
			       " USB power on while in  usb device mode\n");
			break;
		case USB_EVENT_POWER_OFF:
			if (data->u_ops->device_to_idle(data)) {
				break;
			}

			data->usb_state = USB_STATE_IDLE;
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  while in device mode .(not expected) \n");
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			/* not expected */
			printk(KERN_DEBUG PFX
			       " Device detect timeout  while in device mode .(not expected) \n");
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			/* not expected */
			printk(KERN_DEBUG PFX
			       " host detect timeout  while in device mode .(not expected) \n");
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/* don't to anything */
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			/* don't to anything */
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			/* perhaps we will need to disable interrupts at some point? */
			if (data->u_ops->device_to_initial(data)) {
				break;
			}
				
			data->usb_state = USB_STATE_INITIAL;
			break;

		}
		break;

	case USB_STATE_HOST_DETECT:
		switch (event) {
		case USB_EVENT_POWER_ON:
			printk(KERN_DEBUG PFX
			       " USB power on while in  usb host detect mode .(not expected) \n");
			break;
		case USB_EVENT_POWER_OFF:
			if (data->u_ops->host_detect_to_idle(data)) {
				break;
			}
			
			data->usb_state = USB_STATE_IDLE;
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  while in host detect mode. (not expected)\n");
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			/* not expected */
			printk(KERN_DEBUG PFX
			       " Device detect timeout  while in host detect mode. (not expected) \n");
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " host detect timeout while in host detect mode, we can assume that we are in CLA mode\n");
			if (data->u_ops->host_detect_to_cla(data)) {
				break;
			}
			
			data->usb_state = USB_STATE_CLA;
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/* go in usb host mode */
			printk(KERN_DEBUG PFX
			       " hotplug while in host detect mode,switching to usb host\n");
			if (data->u_ops->host_detect_to_host(data)) {
				break;
			}
			
			data->usb_state = USB_STATE_HOST;
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			/* don't to anything */
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			if (data->u_ops->host_detect_to_initial(data)) {
				break;
			}
			
			data->usb_state = USB_STATE_INITIAL;
			break;
		}
		break;
	case USB_STATE_HOST:
		switch (event) {
		case USB_EVENT_POWER_ON:
			printk(KERN_DEBUG PFX
			       " USB power on while in  usb host mode . \n");
			break;
		case USB_EVENT_POWER_OFF:
			/* TODO: if we are in host mode and we get a power off we do not want
			   to switch to an other mode since we only want to exit host mode if there
			   are no devices any more. I guess we need to look if there are devices left
			   on the bus */
			//printk(KERN_DEBUG PFX " Power off  while in host mode.waiting for device removal\n");
			//host_to_idle(data);
			switch (IO_GetCpuType()) {
			case GOCPU_S3C2440:
			case GOCPU_S3C2410:
				/* there are problems on the devices that are shipped with the 2440 SoC, the
				   usb host on those devices really requires the 2 times 15 k pulldown and they 
				   are not in the hardware. the result is that it's not possible to detect device 
				   removal, therefore in this case when the power goes off we directly swicth to 
				   idle mode */
				if (data->u_ops->host_to_idle(data)) {
					break;
				}

				data->usb_state = USB_STATE_IDLE;
				break;

			default:
				break;
			}
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  while in host  mode. (not expected)\n");
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " Device detect timeout  while in host mode. (not expected) \n");
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " host detect timeout while in host mode. (not expected)\n");
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/* TODO: expand hotplug kind of stuff to detect when the last device gets removed */
			printk(KERN_DEBUG PFX
			       " hotplug while in host detect mode TODO\n");
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			if (data->u_ops->host_to_idle(data)) {
				break;
			}

			data->usb_state = USB_STATE_IDLE;
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			data->u_ops->suspend_usb(data);
			if (data->u_ops->host_to_initial(data)) {
				break;
			}

			data->usb_state = USB_STATE_INITIAL;
			break;
		}
		break;
	case USB_STATE_CLA:
		switch (event) {
		case USB_EVENT_POWER_ON:
			printk(KERN_DEBUG PFX
			       " USB power on while in cla mode .(not expected) \n");
			break;
		case USB_EVENT_POWER_OFF:
			printk(KERN_DEBUG PFX
			       " Power off  while in cla mode. switching to idle mode\n");
			if (data->u_ops->cla_to_idle(data)) {
				break;
			}
	
			data->usb_state = USB_STATE_IDLE;
			break;
		case USB_EVENT_DEVICE_RESET:
			printk(KERN_DEBUG PFX
			       " Device reset  while in cla  mode. (not expected)\n");
			break;
		case USB_EVENT_DEVICE_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " Device detect timeout  while in cla mode. (not expected) \n");
			break;
		case USB_EVENT_HOST_DETECT_TIMEOUT:
			printk(KERN_DEBUG PFX
			       " host detect timeout while in host mode. (not expected)\n");
			break;
		case USB_EVENT_HOST_DEVICE_ADDED:
			/*TODO: switch to host mode? */
			printk(KERN_DEBUG PFX
			       " hotplug while in cla mode. (not expected)\n");
			break;
		case USB_EVENT_HOST_DEVICE_REMOVED:
			/*TODO: switch to idle mode? */
			printk(KERN_DEBUG PFX
			       " hotplug while in cla mode. (not expected)\n");
			break;
		case USB_EVENT_KERNEL_SUSPEND:
			if (data->u_ops->cla_to_initial(data)) {
				break;
			}
	
			data->usb_state = USB_STATE_INITIAL;
			break;
		}
		break;

	};

	if (previous_state != data->usb_state) {
		/* we have performed a state change */
		printk(KERN_INFO PFX "[%i,%i,%i] %s ==( %s )==> %s\n",
		       previous_state,
		       event,
		       data->usb_state,
		       USB_STATE_STRINGS[previous_state],
		       USB_EVENT_STRINGS[event],
		       USB_STATE_STRINGS[data->usb_state]);

		struct state_change_listener_list_item *item;
		unsigned long flags;
		/* Then call all the registered poll functions. */
		spin_lock_irqsave(&state_change_listener_list_lock, flags);
		list_for_each_entry(item, &state_change_listener_list, node) {
			item->func(previous_state, data->usb_state, item->arg);
		}
		spin_unlock_irqrestore(&state_change_listener_list_lock, flags);

		/* notify that the usb mode  has changed */
		kobject_uevent(&data->dev->kobj, KOBJ_CHANGE, &dev_attr_mode.attr);
	} 
	else {
		/* if we are here the event did not result is a state change so output this
		   information in debug mode so we at least know the event did not result
		   in a change */
		printk(KERN_DEBUG PFX "[%i,%i] %s==(%s) NOP\n",
		       previous_state,
		       event,
		       USB_STATE_STRINGS[previous_state],
		       USB_EVENT_STRINGS[event]);
	}

};

/*==== START state change listener  implementation */
int add_usb_state_change_listener(usb_state_change_listener_func function,
				  void *arg)
{
	struct state_change_listener_list_item *item;
	unsigned long flags;

	item =
	    kmalloc(sizeof(struct state_change_listener_list_item), GFP_KERNEL);
	if (item == NULL) {
		printk(KERN_INFO PFX
		       " %s Unable to allocate state_change_listener_list_item node\n",
		       __FUNCTION__);
		return -ENOMEM;
	}

	/* Quite a bit of overhead for a simple pointer... */
	item->func = function;
	item->arg = arg;

	spin_lock_irqsave(&state_change_listener_list_lock, flags);
	list_add_tail(&item->node, &state_change_listener_list);
	spin_unlock_irqrestore(&state_change_listener_list_lock, flags);

	return 0;
}

EXPORT_SYMBOL(add_usb_state_change_listener);

int remove_usb_state_change_listener(usb_state_change_listener_func function,
				     void *arg)
{
	unsigned long flags;
	struct state_change_listener_list_item *item;

	spin_lock_irqsave(&state_change_listener_list_lock, flags);
	list_for_each_entry(item, &state_change_listener_list, node) {
		if (item->func == function) {
			list_del(&item->node);
			kfree(item);
			spin_unlock_irqrestore(&state_change_listener_list_lock,
					       flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&state_change_listener_list_lock, flags);

	printk(KERN_INFO PFX
	       " %s Unable to find the state change listener for function %p to remove \n",
	       __FUNCTION__, function);
	return -EINVAL;
}

EXPORT_SYMBOL(remove_usb_state_change_listener);

static struct device *static_dev;
static struct usbmode_data *static_usbmode_data;

/* Register usbmode implementation and call init to idle transition */
int register_usbmode_driver(struct usbmode_operations *u_ops, void *u_impl_data) 
{
	printk(KERN_DEBUG PFX " %s\n", __FUNCTION__);
	struct usbmode_data *data =
	    (struct usbmode_data *)(static_dev->driver_data);

	/* Is there a driver already using the state machine? */
	if (data->u_ops || data->u_impl_data) {
		printk(KERN_INFO PFX " USBmode driver already registered! \n");
		return -EBUSY;
	}

	/* Otherwise register and call init */
	data->u_ops = u_ops;
	data->u_impl_data = u_impl_data;
	atomic_set(&data->event_barrier,0);
	
	return 0;
}

EXPORT_SYMBOL(register_usbmode_driver);


/* Unregister usbmode driver, note: the actual implementation has to do its cleanup by itself */
void unregister_usbmode_driver(void) 
{
	printk(KERN_DEBUG PFX " %s\n", __FUNCTION__);
	struct state_change_listener_list_item *item;
	unsigned long flags;
	struct usbmode_data *data =
	    (struct usbmode_data *)(static_dev->driver_data);
	USB_STATE previous_state = data->usb_state;

	data->u_ops = NULL;
	data->u_impl_data = NULL;
	data->usb_state = USB_STATE_INITIAL;

	/* Then call all the registered poll functions. */
	spin_lock_irqsave(&state_change_listener_list_lock, flags);
	list_for_each_entry(item, &state_change_listener_list, node) {
		item->func(previous_state, data->usb_state, item->arg);
	}
	spin_unlock_irqrestore(&state_change_listener_list_lock, flags);

	/* Notify that the usb mode has changed */
	kobject_uevent(&data->dev->kobj, KOBJ_CHANGE, &dev_attr_mode.attr);
}

EXPORT_SYMBOL(unregister_usbmode_driver);



/*==== START usbmode module code */

/* probe method for the usbmode device driver */
static int usbmode_probe(struct device *dev)
{
	printk(KERN_DEBUG PFX " Probing\n");

	if (!(static_usbmode_data = kmalloc(sizeof(struct usbmode_data), GFP_KERNEL))) {
		/*make sure we don't have dangling pointers */
		dev->driver_data = NULL;
		printk(KERN_DEBUG PFX
		       " failed to allocate memory for the usbmode_data struct \n");
		return -ENOMEM;
	}

	/* clear the data and initialize */
	memset(static_usbmode_data, 0, sizeof(struct usbmode_data));

	static_usbmode_data->usb_state = USB_STATE_INITIAL;
	static_usbmode_data->input_events = 0x00;

	/* make usbmode_data the driver_data */
	dev->driver_data = static_usbmode_data;
	
	/* initialize a state change work queue */
	INIT_WORK(&static_usbmode_data->state_changed_work, state_changed_work_handler, static_usbmode_data);

	/* and assign the dev back to the structure */
	static_usbmode_data->dev = dev;

	device_create_file(dev, &dev_attr_mode);

	static_dev = dev;
	return 0;		/* success */
};

/**
 * usbmode driver remove method
 **/
static int usbmode_remove(struct device *dev)
{
	struct usbmode_data *data = (struct usbmode_data *)(dev)->driver_data;

	if (data->u_ops) {
		printk(KERN_INFO PFX " USBmode still has a driver registered!\n");
		return EBUSY;
	}

	device_remove_file(dev, &dev_attr_mode);

	/* TODO:also disable the state_changed_work? */
	/* TODO: This has to go into the underlying implementation's cleanup 
	if (data->u_ops && data->u_ops->stop_powermon) data->u_ops->stop_powermon(data);
	*/	

	return 0;
};

static int usbmode_suspend(struct device *dev, u32 state, u32 level)
{
	struct usbmode_data *data = (struct usbmode_data *)dev->driver_data;

	atomic_set(&data->event_barrier,1);

	if (level == SUSPEND_POWER_DOWN) {
		printk(KERN_DEBUG PFX " %s power down \n", __FUNCTION__);
		handle_usbmode_state_event(data, USB_EVENT_KERNEL_SUSPEND);
	}

	return 0;
}

static int usbmode_resume(struct device *dev, u32 level)
{
	struct usbmode_data *data = (struct usbmode_data *)dev->driver_data;
	atomic_set(&data->event_barrier,0);

	if (level == RESUME_ENABLE) {
		printk(KERN_DEBUG PFX " %s power on \n", __FUNCTION__);
		if (data->usb_state != USB_STATE_INITIAL) {
			printk(KERN_WARNING PFX " %s the current state is not USB_STATE_INITIAL skipping initialisation\n", __FUNCTION__);
			return 0;
		}
		data->usb_state = USB_STATE_INITIAL;
		data->input_events = 0x00;
		
		if (!(data->u_ops->initial_to_idle(data))) data->usb_state = USB_STATE_IDLE;
		printk(KERN_DEBUG PFX " %s done\n", __FUNCTION__);
	}
	return 0;
}

/*This driver registers a tomtomgo-usbmode */
static struct device_driver usbmode_driver = {
	.name = "tomtomgo-usbmode",
	.bus = &platform_bus_type,
	.probe = usbmode_probe,
	.remove = usbmode_remove,
	.suspend = usbmode_suspend,
	.resume = usbmode_resume,
};

/**
 * usbmode module initialization routine for the linux kernel.
 * This method registers a bus driver called tomtomgo-usbmode.
 **/
static int __init usbmode_init_module(void)
{
	int ret;

	/* display driver information when the driver is started */
	printk(KERN_INFO DRIVER_DESC_LONG "(" PFX ")\n");

	/* this driver registers itself as driver to get the different changes in
	   software states */
	ret = driver_register(&usbmode_driver);
	if (ret) {
		printk(KERN_ERR PFX " unable to register driver (%d)\n", ret);
		return ret;
	}

	printk(KERN_DEBUG PFX " Initialized\n");
	return 0;
};

static int __init usbmode_late_init(void)
{
	printk(KERN_DEBUG PFX " %s\n", __FUNCTION__);
	struct usbmode_data *data =
	    (struct usbmode_data *)(static_dev->driver_data);
	
	/* Is there a driver registered to the state machine? */
	if (!data->u_ops) {
		printk(KERN_INFO PFX " USBmode driver not registered yet! \n");
		return 1;
	}

	/* Switch from initial mode to idle */
	if (data->u_ops->initial_to_idle(data)) {
		printk(KERN_INFO PFX " Init to Idle transition failed.\n");
		return 1;
	}

	data->usb_state = USB_STATE_IDLE;
	return 0;
}

late_initcall(usbmode_late_init);

static void __exit usbmode_exit_module(void)
{
	driver_unregister(&usbmode_driver);

	kfree(static_usbmode_data);
	printk(KERN_INFO PFX " exit\n");
};

module_init(usbmode_init_module);
module_exit(usbmode_exit_module);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR("Kees Jongenburger (kees.jongenburger@tomtom.com), Balazs Gerofi (balazs.gerofi@tomtom.com)");
MODULE_LICENSE("GPL");
