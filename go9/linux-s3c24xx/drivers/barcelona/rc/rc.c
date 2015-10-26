/* drivers/barcelona/rc/rc.c
 *
 * Implementation of the remote driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
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
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <barcelona/Barc_rc.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

#define REMOTE_BUTTON_RIGHT_BOTTOM 1
#define REMOTE_BUTTON_KEYBOARD     2
#define REMOTE_BUTTON_UP           3
#define REMOTE_BUTTON_RIGHT        4
#define REMOTE_BUTTON_VOLUME_UP    5
#define REMOTE_BUTTON_LEFT_BOTTOM  6
#define REMOTE_BUTTON_LEFT         7
#define REMOTE_BUTTON_DOWN         8
#define REMOTE_BUTTON_VOLUME_DOWN  9
#define REMOTE_BUTTON_OK           10

#define rUPLLCON    (*(volatile unsigned *)S3C2410_UPLLCON)
#define rMISCCR     (*(volatile unsigned *)s3c24xx_misccr)

/* Defines */
#define PFX "rc: "
#define PK_DBG PK_DBG_FUNC

#define RC_FIFO_SIZE 256

/* Local variables */
static wait_queue_head_t rc_wait;
static int rc_read_index;
static int rc_write_index;
static RC_EVENT rc_fifo[RC_FIFO_SIZE];
static spinlock_t rc_fifo_lock = SPIN_LOCK_UNLOCKED;
static spinlock_t spi_lock = SPIN_LOCK_UNLOCKED;

static void rc_fifo_write(unsigned int id, unsigned char batteryStatus, unsigned char keycode)
{
	unsigned long flags;
#if 0
	static unsigned char toggle = 0;
	
	if (toggle)
	{
		IO_Activate(BACKLIGHT_EN);
		toggle = 0;
	}
	else
	{
		IO_Deactivate(BACKLIGHT_EN);
		toggle = 1;
	}
#endif

	spin_lock_irqsave(&rc_fifo_lock, flags);

	if (++rc_write_index >= RC_FIFO_SIZE)
		rc_write_index = 0;

	rc_fifo[rc_write_index].id = id;
	rc_fifo[rc_write_index].batteryStatus = batteryStatus;
	rc_fifo[rc_write_index].keycode = keycode;

	/* If fifo is full, bump read index. */
	if (rc_write_index == rc_read_index)
		if (++rc_read_index >= RC_FIFO_SIZE)
			rc_read_index = 0;

	spin_unlock_irqrestore(&rc_fifo_lock, flags);
}

static void IA4320_SendCommand(unsigned short command)
{
	unsigned char bit;

	spin_lock(&spi_lock);

	IO_Activate(FSK_EN);
	IO_Deactivate(SPICLK);
	IO_Deactivate(FSK_EN);

	for (bit=0;bit<16;bit++)
	{
		if (command & 0x8000) IO_Activate(SPIMSI); else IO_Deactivate(SPIMSI);
		IO_Activate(SPICLK);
		command <<= 1;
		IO_Deactivate(SPICLK);
	}

	IO_Activate(FSK_EN);

	spin_unlock(&spi_lock);
}

static void IA4320_Init(void)
{
	IO_Activate(FSK_FFS);
	IO_SetInput(FSK_FFE);
	IO_SetInput(SPIMSO);

	IA4320_SendCommand(0x8986);
	IA4320_SendCommand(0xA620);
	IA4320_SendCommand(0xC081);
	IA4320_SendCommand(0xE196);
	IA4320_SendCommand(0xCC08);
	IA4320_SendCommand(0xC200);
	IA4320_SendCommand(0xC6AF);
	IA4320_SendCommand(0xC8AA);
	IA4320_SendCommand(0xC4AC);
	IA4320_SendCommand(0xC080);
	IA4320_SendCommand(0xC081);
	IA4320_SendCommand(0xCE88);
	IA4320_SendCommand(0xCE8B);

	IO_SetInterruptOnActivated(FSK_IRQ);
}

void IA4320_PowerDown(void)
{
	// Disable IRQ
	IO_SetInput(FSK_IRQ);
	// Set IA4320 in power down mode
	IA4320_SendCommand(0x8802);
	IA4320_SendCommand(0xc0c0);
	IA4320_SendCommand(0xc4ac);
}

inline static void IA4320_HandleInterrupt(void)
{
	static unsigned char data;
	static unsigned char bit;
	static unsigned char buffer[8];
	static unsigned char index=0;
	static unsigned char checksum;
	static unsigned int id;
	static unsigned char keycode;
	static int count;
	static int lastCount = -1;
	static unsigned int repeatJiffies = 0;

	spin_lock(&spi_lock);

	IO_Activate(FSK_FFS);
	IO_SetInput(SPIMSO);
	IO_Deactivate(SPICLK);
	IO_Deactivate(SPIMSI);
	IO_Deactivate(FSK_EN);

	// Get bits in FIFO status
	IO_Activate(SPICLK);
	data = IO_GetInput(SPIMSO);
	IO_Deactivate(SPICLK);

	IO_Activate(SPICLK);
	IO_Deactivate(SPICLK);

	IO_Activate(SPICLK);
	IO_Deactivate(SPICLK);

	IO_Activate(SPICLK);
	IO_Deactivate(SPICLK);

	if (data)
	{
		// Get FIFO data
		IO_Deactivate(FSK_FFS);
		data = 0;
		for (bit=0;bit<8;bit++)
		{
			data <<= 1;
			IO_Activate(SPICLK);
			if (IO_GetInput(SPIMSO)) data |= 1;
			IO_Deactivate(SPICLK);
		}

		IO_Activate(FSK_FFS);
		IO_Activate(FSK_EN);

		spin_unlock(&spi_lock);

		if ((index == 0) && (data != 't'))
		{
			// Reset stuff
			IA4320_SendCommand(0xCE88);
			IA4320_SendCommand(0xCE8b);
		}
		else
		{
			buffer[index++] = data;
			if (index == 8)
			{
				// Reset stuff
				IA4320_SendCommand(0xCE88);
				IA4320_SendCommand(0xCE8b);

				// Calculate checksum
				checksum = 0;
				for (index=0;index<7;index++) checksum += buffer[index];
				index = 0;

				// Check checksum
				if (buffer[7] == (checksum & 0xff))
				{
					id = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16);
					count = buffer[5];
					keycode = buffer[6];

					// New button pressed?
					if (count != lastCount)
					{
						lastCount = count;
						// Store data in FIFO and wakeup processes
						rc_fifo_write(id, buffer[4], keycode);
						wake_up_interruptible(&rc_wait);

						// Determine if key should repeat
						switch (keycode)
						{
							case REMOTE_BUTTON_LEFT:        
							case REMOTE_BUTTON_RIGHT:
							case REMOTE_BUTTON_UP:
							case REMOTE_BUTTON_DOWN:
							case REMOTE_BUTTON_VOLUME_UP:
							case REMOTE_BUTTON_VOLUME_DOWN:
								// Start repeating after 800 msec
								repeatJiffies = jiffies + (800*HZ / 1000);
								break;
							default:
								repeatJiffies = 0;
								break;
						}
					}
					else
					{
						if ((repeatJiffies) && (jiffies >= repeatJiffies))
						{
							// Store data in FIFO and wakeup processes
							rc_fifo_write(id, buffer[4], keycode);
							wake_up_interruptible(&rc_wait);
							// Repeat 8 times a second
							repeatJiffies = jiffies + (HZ / 8);
						}
					}
				} 
			}
		}
	} 
	else
	{
		IO_Activate(FSK_EN);

		spin_unlock(&spi_lock);
	}
}

static irqreturn_t rc_interrupt(int irq, void *dev_id, struct pt_regs *fp)
{
	IA4320_HandleInterrupt();
	return IRQ_HANDLED;
}

static ssize_t rc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	unsigned long flags;
	RC_EVENT ev;

	/* Check user buffer sanity */
	if (len < sizeof(RC_EVENT) || !access_ok(VERIFY_WRITE, data, sizeof(RC_EVENT)))
		return -EINVAL;

	/* Check if data is available in the fifo */
	spin_lock_irqsave(&rc_fifo_lock, flags);
	if (rc_write_index == rc_read_index) {
		spin_unlock_irqrestore(&rc_fifo_lock, flags);
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&rc_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			for (;;) {
				spin_lock_irqsave(&rc_fifo_lock, flags);
				if (rc_write_index != rc_read_index)
					break;
				spin_unlock_irqrestore(&rc_fifo_lock, flags);
				if (signal_pending(current)) {
					set_current_state(TASK_RUNNING);
					remove_wait_queue(&rc_wait, &wait);
					return -ERESTARTSYS;
				}
				schedule();
			}
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&rc_wait, &wait);
		}
	}
	/* Spinlock should still be locked here... */
	if (++rc_read_index >= RC_FIFO_SIZE)
		rc_read_index = 0;
	memcpy(&ev, &rc_fifo[rc_read_index], sizeof(RC_EVENT));
	spin_unlock_irqrestore(&rc_fifo_lock, flags);

	return __copy_to_user(data, &ev, sizeof(RC_EVENT)) ? -EFAULT : sizeof(RC_EVENT);
}

static unsigned int rc_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned long flags;
	unsigned int ret;

	poll_wait(file, &rc_wait, wait);

	spin_lock_irqsave(&rc_fifo_lock, flags);
	ret = (rc_write_index == rc_read_index) ? 0 : (POLLIN | POLLRDNORM);
	spin_unlock_irqrestore(&rc_fifo_lock, flags);

	return ret;
}

static int rc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static int rc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int rc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations rc_fops = {
	.owner		= THIS_MODULE,
	.read		= rc_read,
	.poll		= rc_poll,
	.ioctl		= rc_ioctl,
	.open		= rc_open,
	.release	= rc_release,
};

static int rc_probe(struct device *dev)
{
    int ret;

	printk(KERN_INFO "TomTom GO Remote Driver, (C) 2004,2005 TomTom BV\n");
    if (!IO_HaveRemote()) return -1;

    PK_DBG("Initializing wait queue\n");
    init_waitqueue_head(&rc_wait);

    IA4320_Init();

    PK_DBG("Requesting remote IRQ\n");
    ret = request_irq(IO_GetInterruptNumber(FSK_IRQ), rc_interrupt, 0, "rc_interrupt", dev);
    if (ret < 0) {
    	PK_ERR("Failed to request timer IRQ (%d)\n", ret);
	return ret;
    }
    
    PK_DBG("Registering chardev\n");
    ret = register_chrdev(RC_MAJOR, RC_DEVNAME, &rc_fops);
    if (ret != 0) {
	PK_ERR("Unable to register chardev on major=%d (%d)\n", RC_MAJOR, ret);
	return ret;
    }

    PK_DBG("Done\n");

    return 0;
}

static int rc_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(RC_MAJOR, RC_DEVNAME);

	PK_DBG("Unregistering interrupt\n");


	PK_DBG("Done\n");
	return 0;
}

static void rc_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	IA4320_PowerDown();
}

#ifdef CONFIG_PM

static int rc_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		IA4320_PowerDown();
	}
	return 0;
}

static int rc_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		IA4320_Init();
	}
	return 0;
}

#else /* CONFIG_PM */
#define rc_suspend NULL
#define rc_resume NULL
#endif /* CONFIG_PM */

static struct device_driver rc_driver = {
	.name		= "tomtomgo-rc",
	.bus		= &platform_bus_type,
	.probe		= rc_probe,
	.remove		= rc_remove,
	.shutdown	= rc_shutdown,
	.suspend	= rc_suspend,
	.resume		= rc_resume,
};

static int __init rc_mod_init(void)
{
	int ret;
	
	PK_DBG("Registering driver\n");
	ret = driver_register(&rc_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
} 

static void __exit rc_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&rc_driver);
	PK_DBG("Done\n");
}

module_init(rc_mod_init);
module_exit(rc_mod_exit);

MODULE_AUTHOR("Jeroen Taverne <jeroen.taverne@tomtom.com> and Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Remote Driver");
MODULE_LICENSE("GPL");

/* EOF */
