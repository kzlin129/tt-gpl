/* drivers/barcelona/ts/ts.c
 *
 * Implementation of the touchscreen driver.
 *
 * Copyright (C) 2004,2005,2006,2007,2008 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
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
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <barcelona/Barc_ts.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>

/* Defines */
#define PFX "ts: "
#define PK_DBG PK_DBG_FUNC

#define TS_FIFO_SIZE 256
#define PEN_UP 0
#define PEN_DOWN 255

static int clamp_x;
static int clamp_y;

/* Local variables */
static wait_queue_head_t ts_wait;
static int ts_read_index = 0;
static int ts_write_index = 0;
static int pen = -1;
static int forceTasklet = 0;
static TS_EVENT ts_fifo[TS_FIFO_SIZE];
static MATRIX ts_matrix = {
	.An = -363,
	.Bn = 0,
	.Cn = 360416,
	.Dn = 0,
	.En = 258,
	.Fn = -12676,
	.Divider = 1000,
	.xMin = 0,    // Stored on FLASH address 0x10c
	.xMax = 1023, // Stored on FLASH address 0x110
	.yMin = 0,    // Stored on FLASH address 0x104
	.yMax = 1023, // Stored on FLASH address 0x108
};
static int ts_calibrated = 1;

static void ts_tasklet(unsigned long data);
DECLARE_TASKLET(ts_tasklet_handle, ts_tasklet, 0L);

static unsigned short buf[ADC_CHANNELS];

static void ts_fifo_write(short x, short y, short pressure)
{
	if ((ts_read_index == ts_write_index) || (pen == -1))
	{
		if (++ts_write_index >= TS_FIFO_SIZE)
			ts_write_index = 0;
	}

	ts_fifo[ts_write_index].x = x;
	ts_fifo[ts_write_index].y = y;
	ts_fifo[ts_write_index].pressure = pressure;

	/* If fifo is full, bump read index. */
	if (ts_write_index == ts_read_index)
		if (++ts_read_index >= TS_FIFO_SIZE)
			ts_read_index = 0;
}

static void ts_adc_poll(short passedbuf[ADC_CHANNELS], void* arg )
{
	if ((forceTasklet) || (passedbuf[ADC_TS_DOWN] < 200))
	{
		memcpy(buf,passedbuf,sizeof(buf));
		tasklet_schedule(&ts_tasklet_handle);
	}
}

static void ts_tasklet(unsigned long data)
{
	static short lastX = 10000;
	static short lastY = 10000;
	static short stableX;
	static short stableY;
	short x = buf[ADC_TS_X];
	short y = buf[ADC_TS_Y];
	short down = buf[ADC_TS_DOWN];

	// Pen down?
	if (down < 200)
	{
		short difX = lastX - x;
		short difY = lastY - y;

		if (difX < 0)
			difX = -difX;

		if (difY < 0)
			difY = -difY;

		if (difX < 50 && difY < 50)
		{
			stableX = (lastX + x) >> 1;
			stableY = (lastY + y) >> 1;

			// Store pen down
			ts_fifo_write(stableX, stableY, PEN_DOWN);

			wake_up_interruptible(&ts_wait);

			// Pen is down and it must be 2 cycles up before pen up is send
			pen = 2;

			// Make sure tasklet is also called when there is a pen up
			forceTasklet = 1;
		}

		lastX = x;
		lastY = y;
		
	}
	else
	{
		if (pen > 0)
			--pen;

		if (pen == 0)
		{
			pen = -1;

			// Take last stable raw or calibrated values for pen up
			// Store pen up
			ts_fifo_write(stableX, stableY, PEN_UP);

			wake_up_interruptible(&ts_wait);

			// Make next difference high enough to do a second test
			lastX = 10000;
			lastY = 10000;
			
			// Make sure tasklet is not called till there is a pen down
			forceTasklet = 0;
		}
	}
}

static ssize_t ts_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	TS_EVENT ev;

	/* Check user buffer sanity */
	if (len < sizeof(TS_EVENT) || !access_ok(VERIFY_WRITE, data, sizeof(TS_EVENT)))
		return -EINVAL;

	/* Check if data is available in the fifo */
	tasklet_disable(&ts_tasklet_handle);
	if (ts_write_index == ts_read_index) {
		tasklet_enable(&ts_tasklet_handle);
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&ts_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			for (;;) {
				tasklet_disable(&ts_tasklet_handle);
				if (ts_write_index != ts_read_index)
					break;
				tasklet_enable(&ts_tasklet_handle);
				if (signal_pending(current)) {
					set_current_state(TASK_RUNNING);
					remove_wait_queue(&ts_wait, &wait);
					return -ERESTARTSYS;
				}
				schedule();
			}
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&ts_wait, &wait);
		}
	}
	/* Spinlock should still be locked here... */
	if (++ts_read_index >= TS_FIFO_SIZE)
		ts_read_index = 0;
	memcpy(&ev, &ts_fifo[ts_read_index], sizeof(TS_EVENT));
	tasklet_enable(&ts_tasklet_handle);

	if (ts_calibrated) {
		/* Calculate points */
		ev.x = ((ev.x - ts_matrix.xMin) * (clamp_x-30)) / (ts_matrix.xMax - ts_matrix.xMin);
		ev.y = ((ev.y - ts_matrix.yMin) * (clamp_y-30)) / (ts_matrix.yMax - ts_matrix.yMin);

		// Add offset
		ev.x += 15;
		ev.y += 15;
				
		/* Clamping */
		if (ev.x < 0)
			ev.x = 0;
		else if (ev.x > clamp_x)
			ev.x = clamp_x;
		if (ev.y < 0)
			ev.y = 0;
		else if (ev.y > clamp_y)
			ev.y = clamp_y;

		// X plane is connected the wrong way around in hardware
		ev.x = clamp_x - ev.x;
	}

	return __copy_to_user(data, &ev, sizeof(TS_EVENT)) ? -EFAULT : sizeof(TS_EVENT);
}

static unsigned int ts_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int ret;

	poll_wait(file, &ts_wait, wait);

	tasklet_disable(&ts_tasklet_handle);
	ret = (ts_write_index == ts_read_index) ? 0 : (POLLIN | POLLRDNORM);
	tasklet_enable(&ts_tasklet_handle);

	return ret;
}

static int ts_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case TS_GET_CAL:
		ret = copy_to_user((void __user *) arg, &ts_matrix, sizeof ts_matrix) ? -EFAULT : 0;
		break;
	case TS_SET_CAL:
		ret = copy_from_user(&ts_matrix, (void __user *) arg, sizeof ts_matrix) ? -EFAULT : 0;
		break;
	case TS_SET_RAW_ON:
		ts_calibrated = 0;
		ret = 0;
		break;
	case TS_SET_RAW_OFF:
		ts_calibrated = 1;
		ret = 0;
		break;
	case TS_ENABLE:
		// Enable handling of TS
		adc_register_poll(ts_adc_poll, NULL );
		ret = 0;
		break;
	case TS_DISABLE:
		// Disable handling of TS
		adc_unregister_poll(ts_adc_poll);
		
		// Clear FIFO
		ts_read_index = 0;
		ts_write_index = 0;
		ret = 0;
		break;
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ts_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int ts_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations ts_fops = {
	.owner		= THIS_MODULE,
	.read		= ts_read,
	.poll		= ts_poll,
	.ioctl		= ts_ioctl,
	.open		= ts_open,
	.release	= ts_release,
};

static int ts_probe(struct device *dev)
{
	int ret;

	PK_DBG("Initializing wait queue\n");
	init_waitqueue_head(&ts_wait);

	PK_DBG("Registering ADC pollfunc\n");
	ret = adc_register_poll(ts_adc_poll, NULL );
	if (ret != 0) {
		PK_ERR("Unable to register ADC pollfunc (%d)\n", ret);
		return ret;
	}

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(TS_MAJOR, TS_DEVNAME, &ts_fops);
	if (ret != 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", TS_MAJOR, ret);
		return ret;
	}
	
	switch (IO_GetTftType()) {
	case GOTFT_SAMSUNG_LTP400:
	case GOTFT_SAMSUNG_LTE430WQ:
	case GOTFT_SHARP_LQ043T1:
	case GOTFT_SAMSUNG_LMS430HF12:
	case GOTFT_SHARP_LQ043T3DW01:
	case GOTFT_LG_LB043WQ3:	
		clamp_x = 480-1;
		clamp_y = 272-1;
		break;
	case GOTFT_NEC_NL2432HC22:
	case GOTFT_SAMSUNG_LTV350:
	default:
		clamp_x = 320-1;
		clamp_y = 240-1;
		break;
	}

	PK_DBG("Done\n");
	return 0;
}

static int ts_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(TS_MAJOR, TS_DEVNAME);

	PK_DBG("Unregistering ADC poll\n");
	adc_unregister_poll(ts_adc_poll);

	PK_DBG("Done\n");
	return 0;
}

static struct device_driver ts_driver = {
	.name		= "tomtomgo-ts",
	.bus		= &platform_bus_type,
	.probe		= ts_probe,
	.remove		= ts_remove,
};

static int __init ts_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Touchscreen Driver, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&ts_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit ts_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&ts_driver);
	PK_DBG("Done\n");
}

module_init(ts_mod_init);
module_exit(ts_mod_exit);

MODULE_AUTHOR("Koen Martens <kmartens@sonologic.nl>, Jeroen Taverne <jeroen.taverne@tomtom.com> and Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Touchscreen Driver");
MODULE_LICENSE("GPL");

/* EOF */
