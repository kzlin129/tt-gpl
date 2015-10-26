/* drivers/barcelona/acc/acc.c
 *
 * Implementation of the accelerometer driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
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
#include <asm/arch/regs-gpio.h>
#include <barcelona/Barc_acc.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

/* Defines */
#define PFX "acc: "
#define PK_DBG PK_DBG_FUNC

#define ACC_FIFO_SIZE 256
#define ACC_SCALER_MIN (ADC_RATE / ACC_RATE_MAX)
#define ACC_SCALER_DEF (ADC_RATE / ACC_RATE_DEF)
#define ACC_SCALER_MAX (ADC_RATE / ACC_RATE_MIN)

/* Local variables */
static wait_queue_head_t acc_wait;
static volatile int acc_read_index;
static volatile int acc_write_index;
static ACCMETER_DATA acc_fifo[ACC_FIFO_SIZE];
static volatile unsigned acc_scaler = ACC_SCALER_DEF;

static void acc_tasklet(unsigned long data);
DECLARE_TASKLET(acc_tasklet_handle, acc_tasklet, 0L);

static unsigned short buf[ADC_CHANNELS];

extern void mxr2312_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data);
extern void mxr3999_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data);
extern void mxm9301_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data);
extern void mxr9500_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data);
static void (*acc_get_values)(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data) = 0;

static void acc_adc_poll(short passedbuf[ADC_CHANNELS], void* arg)
{
	memcpy(buf,passedbuf,sizeof(buf));
	tasklet_schedule(&acc_tasklet_handle);
}

static void acc_tasklet(unsigned long data)
{
	static unsigned count = 0;
	struct timeval tv;

	if (count > 0) {
		--count;
		return;
	}
	count = acc_scaler - 1;

	do_gettimeofday(&tv);

	/* Store accelerometer data in fifo. */

	/* Get acc values acoording to used type */
	if (acc_get_values) acc_get_values(buf,&acc_fifo[acc_write_index]);
	/* Get gyro data */
	acc_fifo[acc_write_index].u32Gyro        = buf[ADC_GYRO];
	/* Get timestamp */
	acc_fifo[acc_write_index].u32sTimeStamp  = tv.tv_sec;
	acc_fifo[acc_write_index].u32usTimeStamp = tv.tv_usec;

	if (++acc_write_index >= ACC_FIFO_SIZE)
		acc_write_index = 0;
	/* If fifo is full, bump read index. */
	if (acc_write_index == acc_read_index)
		if (++acc_read_index >= ACC_FIFO_SIZE)
			acc_read_index = 0;

	wake_up_interruptible(&acc_wait);
}

static ssize_t acc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	ssize_t ret;

	/* Check user buffer sanity */
	if (len < sizeof(ACCMETER_DATA) || !access_ok(VERIFY_WRITE, data, sizeof(ACCMETER_DATA)))
		return -EFAULT;

	/* Check if data is available in the fifo */
	tasklet_disable(&acc_tasklet_handle);
	if (acc_write_index == acc_read_index) {
		tasklet_enable(&acc_tasklet_handle);
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&acc_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			for (;;) {
				tasklet_disable(&acc_tasklet_handle);
				if (acc_write_index != acc_read_index)
					break;
				tasklet_enable(&acc_tasklet_handle);
				if (signal_pending(current)) {
					set_current_state(TASK_RUNNING);
					remove_wait_queue(&acc_wait, &wait);
					return -ERESTARTSYS;
				}
				schedule();
			}
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&acc_wait, &wait);
		}
	}
	/* Spinlock should still be locked here... */
	ret = __copy_to_user(data, &acc_fifo[acc_read_index], sizeof(ACCMETER_DATA)) ? -EFAULT : sizeof(ACCMETER_DATA);
	if (++acc_read_index >= ACC_FIFO_SIZE)
		acc_read_index = 0;
	tasklet_enable(&acc_tasklet_handle);

	return ret;
}

static unsigned int acc_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int ret;

	poll_wait(file, &acc_wait, wait);

	tasklet_disable(&acc_tasklet_handle);
	ret = (acc_write_index == acc_read_index) ? 0 : (POLLIN | POLLRDNORM);
	tasklet_enable(&acc_tasklet_handle);

	return ret;
}

static int acc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	unsigned rate;
	unsigned scaler;

	switch (cmd) {
	case IOR_FIFOFILLED_SIZE:
		PK_WARN("IOR_FIFOFILLED_SIZE is no longer supported, please fix!\n");
		ret = -EINVAL;
		break;
	case IOW_ACC_SAMPLINGRATE:
		if (copy_from_user(&rate, (void *) arg, sizeof(unsigned))) {
			PK_DBG("Invalid user buffer %p passed\n", (void *) arg);
			ret = -EFAULT;
		} else if (rate < ACC_RATE_MIN || rate > ACC_RATE_MAX) {
			PK_DBG("Invalid rate %u passed\n", rate);
			ret = -EINVAL;
		} else {
			scaler = ADC_RATE / rate;
			if (scaler < ACC_SCALER_MIN || scaler > ACC_SCALER_MAX) {
				PK_DBG("Rate %u causes weird scaler %u\n", rate, scaler);
				ret = -EINVAL;
			} else {
				PK_DBG("New rate %u, scaler %u\n", rate, scaler);
				acc_scaler = scaler;
				ret = 0;
			}
		}
		break;
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int acc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int acc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations acc_fops = {
	.owner		= THIS_MODULE,
	.read		= acc_read,
	.poll		= acc_poll,
	.ioctl		= acc_ioctl,
	.open		= acc_open,
	.release	= acc_release,
};

static void acc_hw_init(void)
{
	PK_DBG("Activating ACC_PWR_ON and GYRO_EN\n");
	IO_Activate(ACC_PWR_ON);
	IO_Activate(GYRO_EN);
}

static void acc_hw_exit(void)
{
	PK_DBG("Deactivating ACC_PWR_ON and GYRO_EN\n");
	IO_Deactivate(ACC_PWR_ON);
	IO_Deactivate(GYRO_EN);
}

static int acc_probe(struct device *dev)
{
	int ret;

	PK_DBG("Initializing wait queue\n");
	init_waitqueue_head(&acc_wait);

	acc_hw_init();

	switch (IO_GetAccType())
	{
		case GOACC_MXR2312:
			acc_get_values = mxr2312_Read;
			break;
		case GOACC_MXR3999:
			acc_get_values = mxr3999_Read;
			break;
		case GOACC_MXM9301:
			acc_get_values = mxm9301_Read;
			break;
		case GOACC_MXR9500:
			acc_get_values = mxr9500_Read;
			break;
		default:
			acc_get_values = 0;
			break;
	}

	PK_DBG("Registering ADC pollfunc\n");
	ret = adc_register_poll(acc_adc_poll, NULL, ADC_RATE_ACC );
	if (ret < 0) {
		PK_ERR("Unable to register ADC pollfunc (%d)\n", ret);
		return ret;
	}

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(ACC_MAJOR, ACC_DEVNAME, &acc_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", ACC_MAJOR, ret);
		return ret;
	}

	PK_DBG("Done\n");
	return 0;
}

static int acc_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(ACC_MAJOR, ACC_DEVNAME);

	PK_DBG("Unregistering ADC poll\n");
	adc_unregister_poll(acc_adc_poll);

	acc_hw_exit();

	PK_DBG("Done\n");
	return 0;
}

static void acc_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	acc_hw_exit();
}

#ifdef CONFIG_PM

static int acc_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		acc_hw_exit();
	}
	return 0;
}

static int acc_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		acc_hw_init();
	}
	return 0;
}

#else /* CONFIG_PM */
#define acc_suspend NULL
#define acc_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver acc_driver = {
	.name		= "tomtomgo-acc",
	.bus		= &platform_bus_type,
	.probe		= acc_probe,
	.remove		= acc_remove,
	.shutdown	= acc_shutdown,
	.suspend	= acc_suspend,
	.resume		= acc_resume,
};

static int __init acc_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Accelerometer Driver, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&acc_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit acc_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&acc_driver);
	PK_DBG("Done\n");
}

module_init(acc_mod_init);
module_exit(acc_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Accelerometer Driver");
MODULE_LICENSE("GPL");

/* EOF */
