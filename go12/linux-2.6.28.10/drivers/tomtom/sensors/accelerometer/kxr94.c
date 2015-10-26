/* drivers/barcelona/gadc/kxr94_gadc.c
 *
 * Implementation of the KXR94 Generic ADC driver. 
 *
 * Copyright (C) 2007, 2008 TomTom BV <http://www.tomtom.com/>
 * Authors: Jeroen Taverne <Jeroen.Taverne@tomtom.com>
 *          Rogier Stam <Rogier.Stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * See kxr94test.c for small test app
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/uio.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gadc.h>
#include <linux/delay.h>
#include <linux/kxr94.h>

/* Already have 10 Hz 1st order LPF in hardware, oversample by 1.5x */
#define KXR94_DEFAULT_POLL_RATE		30 /* Hz */
static uint kxr94_poll_rate = KXR94_DEFAULT_POLL_RATE;
module_param(kxr94_poll_rate, uint, S_IRUGO);
static uint kxr94_debug = 0;
module_param(kxr94_debug, bool, S_IRUGO);

#define TO_MILLI_HZ(x)			(1000*x)
#define NEXT_POLL(x)			(jiffies + TO_MILLI_HZ(HZ)/x)
#define NEXT_DRIVER_POLL()		(jiffies + HZ/kxr94_poll_rate)

#define KXR94_GADC_DRIVER_NAME		"tomtom-kxr94-accelerometer"

#define KXR94_NUM_CHANNELS		4

#define KXR94_CHAN_X			(0x00)
#define KXR94_CHAN_Z			(0x01)
#define KXR94_CHAN_Y			(0x02)
#define KXR94_CHAN_AUX			(0x07)

static const int kxr94_channel_map[KXR94_NUM_CHANNELS] = {
	KXR94_CHAN_X,
	KXR94_CHAN_Y,
	KXR94_CHAN_Z,
	KXR94_CHAN_AUX,
};

#define KXR94_ADDR_READ_CTRL		(0x03)
#define KXR94_ADDR_WRITE_CTRL		(0x04)
#define KXR94_CTRL_ENABLE		(1<<2)

#define KXR94_MODE_ON			( KXR94_CTRL_ENABLE )
#define KXR94_MODE_OFF			( 0 )

struct channel_samples {
	unsigned long int running_total;
	int nr_samples;
	struct kxr94_gadc *kxr94_dev;
	struct gadc_buffer buffer;
	struct timer_list timer;
	ulong pollcount;
};

struct kxr94_gadc {
	struct workqueue_struct *work_queue;
	struct work_struct work;
	struct timer_list timer;
	struct spi_device *spi_dev;
	struct channel_samples channel[KXR94_NUM_CHANNELS];
	uint count;
	spinlock_t chanlock;
	struct mutex devlock;
	ulong pollcount;	/* # times timer fired */
	ulong transactions;	/* # times SPI device polled */
};
#define KXR94_LOCK(d) mutex_lock(&d->devlock)
#define KXR94_UNLOCK(d) mutex_unlock(&d->devlock)
#define KXR94_LOCK_CH(ch, f) spin_lock_irqsave(&ch->kxr94_dev->chanlock, f)
#define KXR94_UNLOCK_CH(ch, f) spin_unlock_irqrestore(&ch->kxr94_dev->chanlock, f)

static inline unsigned short kxr94_to_raw_value(u8 * b)
{
	return (b[0] << 4) | ((b[1] & 0xF0) >> 4);
}

/**
 * Kernel timer handler function.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
void kxr94_poll_timer_handler(unsigned long arg)
{
	struct kxr94_gadc *kxr94_dev = (struct kxr94_gadc *)arg;

	if (unlikely(kxr94_debug))
		kxr94_dev->pollcount++;
	queue_work(kxr94_dev->work_queue, &kxr94_dev->work);

	mod_timer(&kxr94_dev->timer, NEXT_DRIVER_POLL());
	return;
}

int kxr94_spi_read(struct spi_device *spi_dev, unsigned char cmd,
		   unsigned short *data)
{
	struct spi_message msg;
	struct spi_transfer transfer[2];

	/* Prepare the data. */
	memset(&msg, 0, sizeof(msg));
	memset(transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	/* Prepare the address cycle. */
	transfer[0].tx_buf = &cmd;
	transfer[0].len = sizeof(cmd);
	transfer[0].delay_usecs = 80;
	spi_message_add_tail(&(transfer[0]), &msg);

	/* Prepare the data cycle. */
	transfer[1].rx_buf =
	    (u8 *) (cmd != KXR94_ADDR_READ_CTRL ? data : (data + 1));
	transfer[1].len = (cmd != KXR94_ADDR_READ_CTRL ? 2 : 1);
	transfer[1].delay_usecs = 80;
	spi_message_add_tail(&(transfer[1]), &msg);

	/* Finalize and transmit. */
	msg.spi = spi_dev;
	msg.is_dma_mapped = 0;
	return spi_sync(spi_dev, &msg);
}

int kxr94_spi_write(struct spi_device *spi_dev, unsigned char addr,
		    unsigned char data)
{
	unsigned char tx_buf[2] = { addr, data };
	struct spi_message msg;
	struct spi_transfer transfer;
	int retval;

	/* Prepare the data. */
	memset(&msg, 0, sizeof(msg));
	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	/* Prepare the address cycle. */
	transfer.tx_buf = tx_buf;
	transfer.len = sizeof(tx_buf);
	transfer.delay_usecs = 80;
	spi_message_add_tail(&transfer, &msg);

	/* Finalize and transmit. */
	msg.spi = spi_dev;
	msg.is_dma_mapped = 0;
	retval = spi_sync(spi_dev, &msg);
	return retval;
}

static int kxr94_init_hw(struct spi_device *spi_dev)
{

	kxr94_pdata_t    * pdata;
	pdata = (kxr94_pdata_t *)spi_dev->dev.platform_data;

	pdata->init();

	kxr94_spi_write(spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_ON);
	kxr94_spi_write(spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_ON);
	return 0;
}

static int kxr94_exit_hw(struct spi_device *spi_dev)
{
	kxr94_spi_write(spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_OFF);
	return 0;
}

static void kxr94_gadc_timer_handler(unsigned long arg)
{
	struct channel_samples *channel = (struct channel_samples *)arg;
	unsigned long int sample_rate; /* In milli-Hz */
	unsigned long int flags;
	u64 hpdiv;

	KXR94_LOCK_CH(channel, flags);
	sample_rate = gadc_get_samplerate(&channel->buffer);

	if (unlikely(kxr94_debug))
		channel->pollcount++;

	if (channel->nr_samples > 0
	    && channel->nr_samples >= TO_MILLI_HZ(kxr94_poll_rate) / sample_rate) {
		hpdiv = ((u64) channel->running_total) << 20;
		do_div(hpdiv, channel->nr_samples);
		gadc_add_sample(&channel->buffer, (unsigned long int)hpdiv);
		channel->running_total = 0;
		channel->nr_samples = 0;
		wake_up_interruptible(&channel->buffer.poll_queue);
	}
	mod_timer(&channel->timer, NEXT_POLL(sample_rate));
	KXR94_UNLOCK_CH(channel, flags);
	return;
}

#ifdef CONFIG_PM
static int kxr94_gadc_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct gadc_cdev *chardev = (struct gadc_cdev *)spi->dev.driver_data;
	struct kxr94_gadc *kxr94_dev = (struct kxr94_gadc *)chardev->private;
	int i;

	if (kxr94_dev->count)
		del_timer(&kxr94_dev->timer);
	/* TODO Clear the work queue */
	kxr94_exit_hw(kxr94_dev->spi_dev);

	for (i = 0; i < KXR94_NUM_CHANNELS; i++) {
		if (kxr94_dev->channel[i].nr_samples >= 0) {
			del_timer(&kxr94_dev->channel[i].timer);
			kxr94_dev->channel[i].running_total = 0;
			kxr94_dev->channel[i].nr_samples = 0;
			gadc_clear_buffer(&kxr94_dev->channel[i].buffer);
		}
	}

	return 0;
}

static int kxr94_gadc_resume(struct spi_device *spi)
{
	struct gadc_cdev *chardev = (struct gadc_cdev *)spi->dev.driver_data;
	struct kxr94_gadc *kxr94_dev = (struct kxr94_gadc *)chardev->private;
	int i;

	/* Startup the hardware. */
	kxr94_init_hw(kxr94_dev->spi_dev);

	/* Start all timers. */
	for (i = 0; i < KXR94_NUM_CHANNELS; i++)
		if (kxr94_dev->channel[i].nr_samples >= 0)
			add_timer(&kxr94_dev->channel[i].timer);

	if (kxr94_dev->count > 0)
		add_timer(&kxr94_dev->timer);

	return 0;
}
#else
#define kxr94_gadc_suspend	NULL
#define kxr94_gadc_resume	NULL
#endif

static void kxr94_gadc_poll_do_work(struct work_struct *work)
{
	struct kxr94_gadc *kxr94_dev = container_of(work, struct kxr94_gadc, work);
	struct channel_samples *channel;
	unsigned long int flags;
	unsigned short receive = 0;
	int i = 0;

	if (unlikely(kxr94_debug))
		kxr94_dev->transactions++;

	/* We just add up the samples and their count here. Averaging will be done from within the timer handler. */
	for (i = 0; i < KXR94_NUM_CHANNELS; i++) {
		channel = &kxr94_dev->channel[i];

		if (kxr94_spi_read
		    (kxr94_dev->spi_dev, kxr94_channel_map[i], &receive)) {
			printk(KERN_ERR "KXR94 Read error\n");
			return;
		}
		receive = kxr94_to_raw_value((u8 *) & receive);

		KXR94_LOCK_CH(channel, flags);
		if (channel->nr_samples >= 0) {
			channel->running_total += (unsigned long int)receive;
			channel->nr_samples++;
		}
		KXR94_UNLOCK_CH(channel, flags);
	}
	return;
}

static void kxr94_gadc_get(struct kxr94_gadc *kxr94_dev)
{
	KXR94_LOCK(kxr94_dev);
	if (kxr94_dev->count++ == 0) {
		printk(KERN_DEBUG "Starting polling of KXR94\n");
		kxr94_dev->timer.expires = NEXT_DRIVER_POLL();
		kxr94_dev->pollcount = 0;
		kxr94_dev->transactions = 0;
		add_timer(&kxr94_dev->timer);
	}

	KXR94_UNLOCK(kxr94_dev);
}

static void kxr94_gadc_put(struct kxr94_gadc *kxr94_dev)
{
	KXR94_LOCK(kxr94_dev);
	BUG_ON(kxr94_dev->count == 0);
	if (--kxr94_dev->count != 0) {
		KXR94_UNLOCK(kxr94_dev);
		return;
	}

	del_timer(&kxr94_dev->timer);
	flush_workqueue(kxr94_dev->work_queue);
	printk(KERN_DEBUG "Stopped polling KXR94\n");
	if (kxr94_debug)
		printk(KERN_DEBUG "Device polled %lu times, %lu transactions\n",
		       kxr94_dev->pollcount, kxr94_dev->transactions);
	KXR94_UNLOCK(kxr94_dev);
}

static int kxr94_gadc_open(struct inode *inode, struct file *file)
{
	struct gadc_cdev *chardev =
	    (struct gadc_cdev *)get_gadc_cdev(inode->i_cdev);
	struct channel_samples *channel;
	unsigned long int sample_rate; /* in milli-Hz */
	struct kxr94_gadc *kxr94_dev;

	kxr94_dev = (struct kxr94_gadc *)chardev->private;
	channel = &(kxr94_dev->channel[iminor(inode) - chardev->minor]);

	/* TODO channels can't be opened more than once */
	if (channel->nr_samples >= 0) {
		printk(KERN_ERR "KXR94: Open failed: device busy\n");
		return -EBUSY;
	}

	/* Initialize data. */
	gadc_init_buffer(&channel->buffer, 1);

	file->private_data = channel;
	channel->running_total = 0;
	channel->nr_samples = 0;
	channel->timer.function = kxr94_gadc_timer_handler;
	channel->timer.data = ((unsigned long int)channel);
	sample_rate = gadc_get_samplerate(&channel->buffer);
	channel->timer.expires = NEXT_POLL(sample_rate);
	gadc_clear_buffer(&channel->buffer);

	channel->pollcount = 0;

	kxr94_gadc_get(kxr94_dev);

	init_timer(&channel->timer);
	add_timer(&channel->timer);

	return 0;
}

static int kxr94_gadc_release(struct inode *inode, struct file *file)
{
	struct channel_samples *channel = file->private_data;

	kxr94_gadc_put(channel->kxr94_dev);

	del_timer(&channel->timer);
	channel->running_total = 0;
	channel->nr_samples = -1;

	if (kxr94_debug)
		printk(KERN_DEBUG "Channel polled %lu times\n",
		       channel->pollcount);

	file->private_data = NULL;
	return 0;
}

static unsigned kxr94_gadc_poll(struct file *file, poll_table * wait)
{
	struct channel_samples *channel = file->private_data;
	int tmpval;

	gadc_poll_wait(file, &channel->buffer, wait);
	if (gadc_get_samplecount(&channel->buffer, &tmpval) > 0) {
		return POLLIN | POLLRDNORM;
	}

	return 0;
}

static int kxr94_gadc_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct channel_samples *channel = file->private_data;
	unsigned long int samplerate;
	struct gadc_format format;
	int ret = 0;

	switch (cmd) {
	case ADC_SET_SAMPLERATE:
		samplerate = (unsigned long int)arg;
		printk(KERN_DEBUG "KXR94: Setting channel sample rate to %lu Hz\n",
			samplerate / 1000);
		if (samplerate > TO_MILLI_HZ(kxr94_poll_rate)) {
			printk(KERN_WARNING "Sample rate %luHz too high, limiting to %uHz\n",
				   samplerate/1000, kxr94_poll_rate);
			samplerate = TO_MILLI_HZ(kxr94_poll_rate);
		}
		if (samplerate == 0) {
			printk(KERN_ERR "KXR94: Bad sample rate 0\n");
			return -EINVAL;
		}

		del_timer(&channel->timer);
		gadc_set_samplerate(&channel->buffer,
				    samplerate);
		gadc_clear_buffer(&channel->buffer);
		channel->timer.expires = NEXT_POLL(samplerate);
		add_timer(&channel->timer);
		break;

	case ADC_GET_SAMPLERATE:
		*((unsigned long int *)arg) =
		    gadc_get_samplerate(&channel->buffer);
		break;

	case ADC_GET_BUFFERSTATUS:
		*((unsigned long int *)arg) = gadc_get_status(&channel->buffer);
		break;

	case ADC_GET_FORMAT:
		format.bitformat = GADC_BF_ULHA;
		format.accuracy = 12;
		ret = copy_to_user((void *)arg, &format, sizeof(format));
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static ssize_t kxr94_gadc_read(struct file *file, char __user * buf,
			       size_t count, loff_t * loff)
{
	struct channel_samples *channel = file->private_data;
	struct gadc_sample sample;
	int samplecount;
	int maxbufsamples;
	int loop = 0;

	/* Determine the amount of samples that'll fit in the buffer. */
	gadc_get_samplecount(&channel->buffer, &samplecount);

	/* If no samples are present block & wait. */
	if (samplecount == 0) {
		if (!(file->f_flags & O_NONBLOCK)) {
			if (gadc_wait_buffer_fill
			    (&(channel->buffer), &samplecount))
				return -ERESTARTSYS;
		} else
			return -EAGAIN;
	}

	/* Determine how many samples to read. */
	maxbufsamples = count / sizeof(sample);
	if (samplecount > maxbufsamples)
		samplecount = maxbufsamples;

	/* Copy sample by sample. Since we need to copy to user space, do this in two steps. */
	for (loop = 0; loop < samplecount; loop++) {
		if (!gadc_get_sample(&channel->buffer, &sample))
			break;
		if (copy_to_user
		    (&(((struct gadc_sample *)buf)[loop]), &sample,
		     sizeof(sample))) {
			printk(KERN_ERR "KXR94: Failed to copy data to userspace!\n");
			return -EFAULT;
		}
	}

	/* Return the read bytes. */
	return loop * sizeof(struct gadc_sample);
}

struct file_operations kxr94_gadc_fops = {
	.owner = THIS_MODULE,
	.read = kxr94_gadc_read,
	.poll = kxr94_gadc_poll,

	.ioctl = kxr94_gadc_ioctl,
	.open = kxr94_gadc_open,
	.release = kxr94_gadc_release,
};

static int kxr94_gadc_probe(struct spi_device *spi)
{
	struct kxr94_gadc *kxr94_dev;
	struct gadc_cdev *chardev;
	int count = 0;

	dev_info(&spi->dev, "TomTom GO KXR94 Generic ADC Driver, (C) 2007-2010 TomTom BV\n");

	/* Allocate memory for the structures. */
	kxr94_dev = kzalloc(sizeof(struct kxr94_gadc), GFP_KERNEL);
	if (kxr94_dev == NULL) {
		dev_err(&spi->dev, "Error allocating memory for private data!\n");
		return -ENOMEM;
	}

	chardev = kzalloc(sizeof(struct gadc_cdev), GFP_KERNEL);
	if (chardev == NULL) {
		dev_err(&spi->dev, "Error allocating memory for cdev!\n");
		kfree(kxr94_dev);
		return -ENOMEM;
	}

	/* Initialize private data. Note that the nr_samples field of each channel is set to -1 to indicate it is not used. */
	for (count = 0; count < KXR94_NUM_CHANNELS; count++) {
		kxr94_dev->channel[count].running_total = 0;
		kxr94_dev->channel[count].nr_samples = -1;
		gadc_init_buffer(&kxr94_dev->channel[count].buffer, 1);
		kxr94_dev->channel[count].kxr94_dev = kxr94_dev;
	}
	chardev->private = kxr94_dev;
	kxr94_dev->spi_dev = spi;
	spin_lock_init(&kxr94_dev->chanlock);
	mutex_init(&kxr94_dev->devlock);
	memset(&kxr94_dev->work, 0, sizeof(kxr94_dev->work));
	memset(&kxr94_dev->timer, 0, sizeof(kxr94_dev->timer));
	kxr94_dev->timer.function = kxr94_poll_timer_handler;
	kxr94_dev->timer.data = ((unsigned long int)kxr94_dev);
	init_timer(&kxr94_dev->timer);
	kxr94_dev->work_queue =
	    create_singlethread_workqueue(KXR94_GADC_DRIVER_NAME);
	INIT_WORK(&kxr94_dev->work, kxr94_gadc_poll_do_work);

	/* Set private device data. */
	spi->dev.driver_data = chardev;

	/* Startup the hardware. */
	kxr94_init_hw(spi);

	/* Register the character device. */
	dev_info(&spi->dev, "Registering %i channels, starting at %u.%u\n", KXR94_NUM_CHANNELS, GADC_MAJOR, 0);
	if (register_gadc_cdev
	    (chardev, GADC_MAJOR, 0, KXR94_NUM_CHANNELS, KXR94_GADC_DRIVER_NAME,
	     &kxr94_gadc_fops)) {
		dev_err(&spi->dev, "Error registering cdev!\n");
		kxr94_exit_hw(spi);
		kfree(kxr94_dev);
		kfree(chardev);
		return -ENODEV;
	}

	return 0;
}

static int kxr94_gadc_remove(struct spi_device *spi)
{
	struct gadc_cdev *chardev = (struct gadc_cdev *)spi->dev.driver_data;
	struct kxr94_gadc *kxr94_dev = (struct kxr94_gadc *)chardev->private;

	flush_workqueue(kxr94_dev->work_queue);
	destroy_workqueue(kxr94_dev->work_queue);

	unregister_gadc_cdev(chardev);

	kfree(chardev->private);
	kfree(chardev);

	dev_info(&spi->dev, "KXR94 driver removed\n");

	return 0;
}

struct spi_driver kxr94_gadc_driver = {
	.driver = {
		   .name = KXR94_GADC_DRIVER_NAME,
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = kxr94_gadc_probe,
	.remove = kxr94_gadc_remove,
	.suspend = kxr94_gadc_suspend,
	.resume = kxr94_gadc_resume,
};

static int __init kxr94_gadc_init(void)
{
	int retval;

	BUG_ON(KXR94_DEFAULT_POLL_RATE > HZ);
	if (kxr94_poll_rate == 0 || kxr94_poll_rate > HZ) {
		printk(KERN_ERR "%u invalid poll rate, using default (%u)\n",
			 kxr94_poll_rate, KXR94_DEFAULT_POLL_RATE);
		kxr94_poll_rate = KXR94_DEFAULT_POLL_RATE;
	}

	retval = spi_register_driver(&kxr94_gadc_driver);
	if (retval)
		return retval;
	return 0;
}

module_init(kxr94_gadc_init);

static void __exit kxr94_gadc_exit(void)
{
	spi_unregister_driver(&kxr94_gadc_driver);
}

module_exit(kxr94_gadc_exit);

MODULE_DESCRIPTION("KXR94 GADC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("Jeroen Taverne <jeroen.taverne@tomtom.com Rogier Stam <rogier.stam@tomtom.com>");
MODULE_PARM_DESC(kxr94_poll_rate, "<poll rate, Hz>");
MODULE_PARM_DESC(kxr94_debug, "<debug = 0 | 1>");

