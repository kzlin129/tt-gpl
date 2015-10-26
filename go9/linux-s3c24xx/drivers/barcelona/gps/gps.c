/* drivers/barcelona/gps/gps.c
 *
 * Implementation of the GPS driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Authors:
 * Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 * Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Includes */ 
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/tomtomgo-irq.h>
#include <barcelona/Barc_gps.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>
#include <../arch/arm/mach-s3c2410/tomtomgo-iopins.h>

/* Defines */
#define PFX "gps: "
#define PK_DBG PK_DBG_FUNC
#define IH_DBG PK_DBG_NONE

/* In order to let read_idx and write_idx freely overlap, 
   buffer size must be power of 2 */
#define GPS_CBUFFER_SIZE 8

#define BUF_LEN	10

/* Kernel resources */
static int gps_irq;

/* Local variables */

struct gps_circular_buf
{
	unsigned int	read_idx;
	unsigned int	write_idx;
	struct timeval	buffer[GPS_CBUFFER_SIZE];
};

struct gps_cbuf_node
{
	struct list_head	node;
	struct inode 		*inode;
	struct file 		*file;
	struct gps_circular_buf gps_cbuf;
};

LIST_HEAD(gps_cbuf_list);
static spinlock_t gps_cbuf_list_lock = SPIN_LOCK_UNLOCKED;

static void gps_add_sample(struct gps_circular_buf *gps_cbuf, struct timeval *gps_stamp )
{
	gps_cbuf->buffer[(gps_cbuf->write_idx) % GPS_CBUFFER_SIZE] = *gps_stamp;
	gps_cbuf->write_idx++;
	
	if((gps_cbuf->write_idx - gps_cbuf->read_idx) == GPS_CBUFFER_SIZE )
		gps_cbuf->read_idx++;
}

static int gps_get_sample(struct gps_circular_buf *gps_cbuf, struct timeval *gps_stamp )
{
	if( gps_cbuf->read_idx == gps_cbuf->write_idx ) {
		return -EAGAIN;
	}

	*gps_stamp = gps_cbuf->buffer[(gps_cbuf->read_idx) % GPS_CBUFFER_SIZE];
	gps_cbuf->read_idx++;
	return 0;
}
				
static void gps_init_cbuf(struct gps_circular_buf *gps_cbuf)
{
	gps_cbuf->write_idx = 0;
	gps_cbuf->read_idx  = 0;
}

irqreturn_t gps_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct timeval		gps_stamp;
	struct gps_cbuf_node	*gps_node;
	unsigned long 		flags;

	do_gettimeofday(&gps_stamp);

	spin_lock_irqsave(&gps_cbuf_list_lock, flags);
	list_for_each_entry(gps_node, &gps_cbuf_list, node) 
	{
		gps_add_sample(&gps_node->gps_cbuf, &gps_stamp);
	}
	spin_unlock_irqrestore(&gps_cbuf_list_lock, flags);

	IH_DBG("%8ld.%06ld\n", gps_stamp.tv_sec, gps_stamp.tv_usec);
	return IRQ_HANDLED;
}

static inline void gps_power(unsigned on)
{
	PK_DBG("Switching GPS device %s\n", on ? "on" : "off");

	if (IO_HasPin(GPS_STANDBY)) {
		PK_DBG("Standby GPS device %s\n", on ? "on" : "off");
		if (on) IO_Deactivate(GPS_STANDBY); else IO_Activate(GPS_STANDBY);
	} else {
		if (on) IO_Activate(GPS_ON); else IO_Deactivate(GPS_ON);
	}
}

static inline void gps_reset(void)
{
	int gpstype = IO_GetGpsType();

	PK_DBG("Resetting GPS device\n");

	IO_Activate(GPS_RESET);	/* will reset BT as well on classc,malaga,atlanta */
	if ( gpstype == GOGPS_ATH_AR1520 ) {
		mdelay(100);
	}
	else {
		mdelay(50);
	}
	IO_Deactivate(GPS_RESET);	
}

static inline void gps_update(unsigned on)
{
	PK_DBG("%sabling GPS firmware update\n", on ? "En" : "Dis");

	if (on)
		IO_Activate(GPS_REPRO);
	else
		IO_Deactivate(GPS_REPRO);
	gps_reset();
}

/*
* The message the device will give when asked
*/
static char Message[BUF_LEN];
static ssize_t gps_reset_write(struct file *file,
  const char __user * buffer, size_t length, loff_t * offset)
{
	int i;
	int to_copy;

//	for (i = 0; i < length && i < BUF_LEN; i++) {
//		get_user(Message[i], buffer + i);
//	}
	to_copy = (length > BUF_LEN ? BUF_LEN : length);
	(void) copy_from_user(&Message[0], buffer, to_copy); 
	if (memcmp(&Message[0],"1" ,1)==0) {
		PK_WARN("WARNING: Going to reset GPS\n");
		gps_reset();
	}
	/*
	 * Return the number of input characters used
	 */
	return to_copy;
}

static int gps_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct timeval 		gps_stamp;
	struct gps_cbuf_node 	*gps_node;
	unsigned long 		flags;
	int ret = 0;

	switch (cmd) {
	case IOW_GPS_PROGRAM_MODE:
		gps_update(1);
		break;
	case IOW_GPS_NORMAL_MODE:
		gps_update(0);
		break;
	case IOW_GPS_ON:
		PK_WARN("WARNING: Ignoring IOW_GPS_ON\n");
		//gps_power(1);
		break;
	case IOW_GPS_OFF:
		PK_WARN("WARNING: Ignoring IOW_GPS_OFF\n");
		//gps_power(0);
		break;
	case IOW_GPS_RESET:
		PK_WARN("WARNING: Not Ignoring IOW_GPS_RESET\n");
		gps_reset();
		break;
	case IOW_GPS_UPDATE_ON:
		PK_WARN("WARNING: Ignoring IOW_GPS_UPDATE_ON\n");
		//gps_update(1);
		break;
	case IOW_GPS_UPDATE_OFF:
		PK_WARN("WARNING: Ignoring IOW_GPS_UPDATE_OFF\n");
		//gps_update(0);
		break;
	case IOR_GET_TIME_STAMP:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg, sizeof gps_stamp)) {
			ret = -EFAULT;
		} else {
			spin_lock_irqsave(&gps_cbuf_list_lock, flags);
			list_for_each_entry(gps_node, &gps_cbuf_list, node) 
			{
				if ((gps_node->inode == inode) && (gps_node->file == file))
				{
					ret = gps_get_sample(&gps_node->gps_cbuf, &gps_stamp);
					if (!ret) {
						ret = __copy_to_user((void __user *) arg, 
								&gps_stamp, 
								sizeof gps_stamp) ? -EFAULT : 0;
					} 
				}
			}
			spin_unlock_irqrestore(&gps_cbuf_list_lock, flags);
		}
		break;
	default:
		//PK_WARN("Invalid ioctl command %u\n", cmd);
		PK_WARN("Invalid ioctl: Type:%d Nr:%d Size:%d dir:%d\n",
			_IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd), _IOC_DIR(cmd));
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int gps_regeister_cbuf(struct inode *inode, struct file *file)
{
	struct gps_cbuf_node *gps_node;
	unsigned long flags;

	gps_node = kmalloc(sizeof(struct gps_cbuf_node), GFP_KERNEL);
	if (gps_node == NULL) {
		PK_ERR("Unable to allocate gps_cbuf node\n");
		return -ENOMEM;
	}

	gps_node->inode = inode;
	gps_node->file  = file;
	gps_init_cbuf(&gps_node->gps_cbuf);
				
	spin_lock_irqsave(&gps_cbuf_list_lock, flags);
	list_add_tail(&gps_node->node, &gps_cbuf_list);
	spin_unlock_irqrestore(&gps_cbuf_list_lock, flags);
	return 0;
}

static int gps_unregister_cbuf(struct inode *inode, struct file *file)
{
	struct gps_cbuf_node *gps_node;
	struct gps_cbuf_node *ptmp;
	unsigned long flags;

	spin_lock_irqsave(&gps_cbuf_list_lock, flags);
	list_for_each_entry_safe(gps_node, ptmp, &gps_cbuf_list, node) 
	{
		if ((gps_node->inode == inode) && (gps_node->file == file))
		{
			list_del(&gps_node->node);
			kfree(gps_node);
			spin_unlock_irqrestore(&gps_cbuf_list_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&gps_cbuf_list_lock, flags);
	PK_WARN("gps_cbuf node for inode:%p and file:%p not found\n", inode, file);
	return -EINVAL;
}

static int gps_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = gps_regeister_cbuf(inode, file);
	if (ret < 0) 
		return ret;
	
	return nonseekable_open(inode, file);
}

static int gps_release(struct inode *inode, struct file *file)
{
	int ret;

	ret = gps_unregister_cbuf(inode, file);
	if (ret < 0)
		return ret;

	return 0;
}

/* Kernel interface */
static struct file_operations gps_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= gps_ioctl,
	.open		= gps_open,
	.write		= gps_reset_write,
	.release	= gps_release,
};

extern unsigned int reset_state;

static void gps_hw_init(void)
{
	PK_DBG(" start");
	static int firsttime = 1;
	int gpstype = IO_GetGpsType();

	/* Config bluetooth UART signals */
	IO_SetFunction(TXD_BT);
	IO_SetFunction(RXD_BT);

	switch (gpstype) {
		case GOGPS_ATH_AR1520:
			/* fallthrough */
		case GOGPS_GL:
		case GOGPS_GL_INT_LNA:
		case GOGPS_GL_BCM4750:

			/* Deactivate signal, i.e. enable RTS  */
			if (IO_GetGpsUartNr() == 0) {
				/* use 1st and only function of GPH8/H9 on S3C2443 */
				IO_SetFunction(RTS_GPS);
				IO_SetFunction(CTS_GPS);
			} else {
				/* use 2nd function of GPG9/GPG10 on S3C2412 */
				IO_SetFunction2(RTS_GPS);     
				IO_SetFunction2(CTS_GPS);
			}

			gps_power(1);

			/* As Cork only has this pin, it will only be activated for it */
			IO_Activate(GPS_POWERON_OUT);

			/* Activate IRQ in case of Barracuda */
			if (IO_GetGpsType() == GOGPS_GL_BCM4750) {
				if (PIN_IS_USED(IO_Pin(GPS_1PPS))) {
					IO_SetInterruptOnActivation(GPS_1PPS);
				}
			}

			break;

		
		/* SiRF chips */
		case GOGPS_SIRF1:
		case GOGPS_SIRF2:
		case GOGPS_SIRF3:

			/* GPS_REPRO == reprogramming is people will reprogram
			   the gps they will use the ioctl calls to program the device */
			IO_Deactivate(GPS_REPRO);
			IO_SetFunction(RTS_BT);
			IO_SetFunction(CTS_BT);

			/* Turn on all GPS pins */
			IO_SetInterruptOnActivation(GPS_1PPS);

			gps_power(1);

			break;

		default:
			return;
	}

	IO_SetFunction(RXD_GPS);
	IO_SetFunction(TXD_GPS);
	
	switch (gpstype) {
		/* GL type chips */
		case GOGPS_GL:
		case GOGPS_GL_INT_LNA:
		case GOGPS_GL_BCM4750: /* Do not reset the barracuda every time */

			if (firsttime) {
				firsttime = 0;

//
// Not resetting the GPS chip is a very bad idea if we are not absolutely 
// sure that the chip is still in a proper state. As long as the reset_state
// boot flag is broken (XIAMEN-240, XIAMEN-253), we cannot rely on it to make
// this decision.
//
//				if (reset_state) {
				if (1) {
					/* bootloader cold boot, reset GPS */
					PK_DBG("%s: RESETTING GPS \n", __FUNCTION__);
					gps_reset();
					PK_DBG(KERN_INFO "%s: the reset button pressed, GPS_RESET is on\n", __FUNCTION__);
				} else {
					/* warm boot */
					if (gpstype == GOGPS_GL_BCM4750) {
						PK_DBG("%s: Deactivate GPS reset \n", __FUNCTION__);
						IO_Deactivate(GPS_RESET);
					}
				}
			} else {
				IO_Deactivate(GPS_RESET);
			}

			break;
		
		case GOGPS_ATH_AR1520:
			/* fallthrough */
		/* SiRF chips */
		case GOGPS_SIRF1:
		case GOGPS_SIRF2:
		case GOGPS_SIRF3:

			gps_reset();

			break;

		default:
			return;
	}
}

static void gps_hw_exit(void)
{
	PK_DBG(" start");

    /* Turn off all GPS pins, exept GPS-nRESET_RF pin,
     * so Trident board is hooked now it neen not resets the
     * Real Time Clock that we have in the chip. */

	switch (IO_GetGpsType()) {
		case GOGPS_ATH_AR1520:
			/* fallthrough */
		/* GL type chips */
		case GOGPS_GL_BCM4750:
			/* fallthrough */
		case GOGPS_GL:
		case GOGPS_GL_INT_LNA:
			IO_Deactivate(GPS_POWERON_OUT);
			IO_Deactivate(RTS_GPS);
			IO_Deactivate(TXD_GPS);

			gps_power(0);
			
			/* Deactivate IRQ in case of Barracuda */
			if (IO_GetGpsType() == GOGPS_GL_BCM4750) {
				if (PIN_IS_USED(IO_Pin(GPS_1PPS))) {
					IO_Deactivate(GPS_1PPS);
				}
			}

			break;
		
		/* SiRF chips */
		case GOGPS_SIRF1:
		case GOGPS_SIRF2:
		case GOGPS_SIRF3:

			IO_Deactivate(GPS_REPRO);
			IO_Deactivate(RTS_GPS);
			IO_Deactivate(TXD_GPS);
			IO_Deactivate(GPS_ON);
			IO_Deactivate(RXD_GPS);
			IO_Deactivate(GPS_1PPS);

			break;

		default:
			return;
	}
}

static int gps_probe(struct device *dev)
{
	int ret;
	gps_hw_init();

	switch (IO_GetGpsType()) {
		case GOGPS_ATH_AR1520:
			/* fallthrough */
		/* GL type chips */
		case GOGPS_GL:
		case GOGPS_GL_INT_LNA:

			gps_irq = 0;
           	PK_DBG("gps_probe: no IRQ!\n");

			break;
		
		/* SiRF chips and Barracuda */
		case GOGPS_SIRF1:
		case GOGPS_SIRF2:
		case GOGPS_SIRF3:
		case GOGPS_GL_BCM4750:

			if (PIN_IS_USED(IO_Pin(GPS_1PPS))) {
				PK_DBG("Requesting GPS_PPS IRQ\n");
				ret = request_irq(IO_GetInterruptNumber(GPS_1PPS), gps_irq_handler, 0, "GPS PPS", dev);
				if (ret != 0) {
					PK_ERR("Failed to request timer IRQ (%d)\n", ret);
					return ret;
				}
				gps_irq = 1;
			}

			break;

		default:
			return -1;
	}

	/* For the Barracuda set GPS_RESET to Input */
	if (IO_GetGpsType() == GOGPS_GL_BCM4750) IO_SetInput(GPS_RESET);	

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(GPS_MAJOR, GPS_DEVNAME, &gps_fops);
	if (ret != 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", GPS_MAJOR, ret);
		return ret;
	}

	PK_DBG("Done\n");
	return 0;
}

static int gps_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(GPS_MAJOR, GPS_DEVNAME);

	if (gps_irq != 0) {
		PK_DBG("Freeing GPS_PPS IRQ\n");
		free_irq(TOMTOMGO_IRQ_GPS_PPS, dev);
		gps_irq = 0;
	}

	gps_hw_exit();

	PK_DBG("Done\n");
	return 0;
}

static void gps_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	gps_hw_exit();
}

#ifdef CONFIG_PM

static int gps_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		gps_hw_exit();
	}
	return 0;
}

static int gps_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		gps_hw_init();
	}
	return 0;
}

#else /* CONFIG_PM */
#define gps_suspend NULL
#define gps_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver gps_driver = {
	.name		= "tomtomgo-gps",
	.bus		= &platform_bus_type,
	.probe		= gps_probe,
	.remove		= gps_remove,
	.shutdown	= gps_shutdown,
	.suspend	= gps_suspend,
	.resume		= gps_resume,
};

static int __init gps_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO GPS Driver, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&gps_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit gps_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&gps_driver);
	PK_DBG("Done\n");
}

module_init(gps_mod_init);
module_exit(gps_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO GPS Driver");
MODULE_LICENSE("GPL");

/* EOF */
