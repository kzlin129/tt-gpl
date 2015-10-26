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
#include <barcelona/debug.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/Barc_ts.h>

#define PFX "ts: "
#define PK_DBG PK_DBG_FUNC

#define IIC_SDALow() IO_Deactivate(HW_IIC_SDA)
#define IIC_SDAHigh() IO_SetInput(HW_IIC_SDA)
#define IIC_GetSDA() IO_GetInput(HW_IIC_SDA)
#define IIC_SCLLow() IO_Deactivate(HW_IIC_SCL)
#define IIC_SCLHigh() IO_SetInput(HW_IIC_SCL)
#define IIC_GetSCL() IO_GetInput(HW_IIC_SCL)

static wait_queue_head_t ts_wait;

static MATRIX ts_matrix = {
	.An = -363,
	.Bn = 0,
	.Cn = 360416,
	.Dn = 0,
	.En = 258,
	.Fn = -12676,
	.Divider = 1000,
	.xMin = 0x40,    // Stored on FLASH address 0x10c
	.xMax = 1023, // Stored on FLASH address 0x110
	.yMin = 0,    // Stored on FLASH address 0x104
	.yMax = 1023, // Stored on FLASH address 0x108
};

static void IIC_Test(void)
{
	while (1)
	{
	IIC_SDAHigh();
	IIC_SDALow();
	}
}

/* IIC master */
static void	IIC_Wait1uSec(void) {
	volatile int i;
	for (i=0; i<300; i++);
}

static void	IIC_Wait2uSec(void) {
	volatile int i;
	for (i=0; i<400; i++);
}

static void	IIC_Wait3uSec(void) {
	volatile int i;
	for (i=0; i<500; i++);
}

static void IIC_Start(void)
{
	// Make SDA low while SCL is high
	IIC_SDAHigh();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait1uSec();
	IIC_SDALow();
	IIC_Wait1uSec();
	IIC_SCLLow();
	IIC_Wait1uSec();
}

static void IIC_Stop(void)
{
	// Make SDA high while SCL is high
	IIC_SDALow();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait1uSec();
	IIC_SDAHigh();
	IIC_Wait1uSec();
}

static int IIC_Transmit(unsigned char data)
{
	int bit;
	int result;
	
	for (bit=0;bit<8;bit++)
	{
		if (data & 0x80) IIC_SDAHigh(); else IIC_SDALow();
		IIC_Wait1uSec();
		IIC_SCLHigh();
		IIC_Wait1uSec();
		while (IIC_GetSCL() == 0);
		IIC_Wait3uSec();
		data <<= 1;
		IIC_SCLLow();
		IIC_Wait1uSec();
	}
	IIC_SDAHigh();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait3uSec();
	if (IIC_GetSDA() == 0) result = 1; else result = 0;
	IIC_SCLLow();
	IIC_Wait1uSec();

	return result;
}

static unsigned char IIC_Receive(int generateAck)
{
	int bit;
	unsigned char data = 0;
	
	IIC_SDAHigh();
	for (bit=0;bit<8;bit++)
	{
		data <<= 1; 
		IIC_SCLHigh();
		IIC_Wait3uSec();
		while (IIC_GetSCL() == 0);
		IIC_Wait3uSec();
		if (IIC_GetSDA()) data |= 1;
		IIC_Wait3uSec();
		IIC_SCLLow();
		IIC_Wait3uSec();
	}
	if (generateAck)
	{
		IIC_SDALow(); /* ack = low */
		IIC_Wait3uSec();
		IIC_SCLHigh();
		IIC_Wait3uSec();
		IIC_SCLLow();
		IIC_Wait3uSec();
	} else {
		IIC_SDAHigh(); /* no ack = high, but still a clock */
		IIC_Wait3uSec();
		IIC_SCLHigh();
		IIC_Wait3uSec();
		IIC_SCLLow();
		IIC_Wait3uSec();
	}
	
	return data;
}

__inline int eventWaiting(void)
{
	IO_SetInput(TOUCHPAD_SW);
	return (IO_GetInput(TOUCHPAD_SW));
}

static ssize_t ts_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	unsigned char eventbuffer[8];
	int eventsize=0, index;
	TS_EVENT tsevent;
	static int x,y,pen=0;

	if (!eventWaiting()) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&ts_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			for (;;) {
				if (eventWaiting())
					break;
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
	
	IIC_Start();
	IIC_Transmit(0x41);
	eventsize = IIC_Receive(1);
	eventbuffer[0] = eventsize;
	eventsize &= 7;
	for (index = 1;index < eventsize;index++)
	{
		eventbuffer[index] = IIC_Receive(1);
	}
	eventbuffer[index] = IIC_Receive(0);
	IIC_Stop();

	if (eventbuffer[6] > ts_matrix.xMin)
	{
		x = (eventbuffer[4] << 8) + eventbuffer[5];
		y = (eventbuffer[2] << 8) + eventbuffer[3];

		x -= 0x0508;
		x = (x * 319) / (0x10c0 - 0x0508);
		x = 319 - x;
		
		y -= 0x0546;
		y = (y * 239) / (0x1266 - 0x0546);
		y = 239 - y;
		
		if (x < 0) x = 0;
		if (y < 0) y = 0;
		if (x > 319) x = 319;
		if (y > 239) y = 239;
		
		pen = 255;

		tsevent.x = x;
		tsevent.y = y;
		tsevent.pressure = pen;
		tsevent.pad = 0;

		return __copy_to_user(data, &tsevent, sizeof(TS_EVENT)) ? -EFAULT : sizeof(TS_EVENT);
	}
	else 
	{
		if (pen)
		{
			pen = 0;

			tsevent.x = x;
			tsevent.y = y;
			tsevent.pressure = pen;
			tsevent.pad = 0;

			return __copy_to_user(data, &tsevent, sizeof(TS_EVENT)) ? -EFAULT : sizeof(TS_EVENT);
		}
		else
		{
			return -EFAULT;
		}
	}
}

static unsigned int ts_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int ret;

	poll_wait(file, &ts_wait, wait);

	ret = (!eventWaiting()) ? 0 : (POLLIN | POLLRDNORM);

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
		ret = 0;
		break;
	case TS_SET_RAW_OFF:
		ret = 0;
		break;
	case TS_ENABLE:
		ret = 0;
		break;
	case TS_DISABLE:
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

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(TS_MAJOR, TS_DEVNAME, &ts_fops);
	if (ret != 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", TS_MAJOR, ret);
		return ret;
	}
	
	PK_DBG("Done\n");
	return 0;
}

static int ts_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(TS_MAJOR, TS_DEVNAME);

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

	printk(KERN_INFO "TomTom GO Capacitive Touchscreen Driver, (C) 2007 TomTom BV\n");
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

MODULE_AUTHOR("Jeroen Taverne <jeroen.taverne@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Capacative Touchscreen Driver");
MODULE_LICENSE("GPL");

/* EOF */
