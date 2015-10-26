/* drivers/barcelona/gadc/kxr94_gadc.c
 *
 * Implementation of the KXR94 Generic ADC driver. 
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <Rogier.Stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
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
#include <barcelona/gacc.h>
#include <barcelona/debug.h>
#include <barcelona/gadc.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/gotype.h>
#include "kxr94_gadc.h"

static int kxr94_init_hw( struct spi_device *spi_dev )
{
        gopin_t chipselect = spi_dev->chip_select;

        IOP_Deactivate( chipselect );
        IOP_SetFunction( chipselect );
        return -EINVAL;
}

static int kxr94_exit_hw( struct spi_device *spi_dev )
{
        gopin_t chipselect = spi_dev->chip_select;

        IOP_SetInput( chipselect );     /* Disable CS   */
        return 0;
}

/**
 * Kernel timer handler function.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
void kxr94_poll_timer_handler( unsigned long  arg )
{
	struct kxr94_gadc	*private=(struct kxr94_gadc *) arg;

        if( ! (private->flags & KXR94_ADC_FLGS_EXIT) )
        {
                /* if we must exit then return and do not reschedule */
                if( queue_work( private->work_queue, &private->work ) )
                {
                        /* There obviously is congestion if we try to schedule work
                         * while the previous scheduled one is not finished yet */
                         private->flags|= KXR94_ADC_FLGS_CONGESTION;
                }

                private->timer.expires = jiffies + ((HZ + KXR94_POLL_RATE - 1)/KXR94_POLL_RATE);
                add_timer( &private->timer );
        }
        return;
}

int kxr94_spi_read( struct spi_device *spi_dev, unsigned char cmd, unsigned short *data )
{
        struct spi_message              msg;
        struct spi_transfer             transfer[2];

        /* Prepare the data. */
        memset( &msg, 0, sizeof( msg ) );
        memset( transfer, 0, sizeof( transfer ) );
        spi_message_init( &msg );

        /* Prepare the address cycle. */
        transfer[0].tx_buf=&cmd;
        transfer[0].len=sizeof( cmd );
        transfer[0].delay_usecs=80;
        spi_message_add_tail( &(transfer[0]), &msg );

        /* Prepare the data cycle. */
        transfer[1].rx_buf=(u8 *) (cmd != KXR94_ADDR_READ_CTRL ? data : (data + 1) );
        transfer[1].len=(cmd != KXR94_ADDR_READ_CTRL ? 2 : 1);
        transfer[1].delay_usecs=80;
        spi_message_add_tail( &(transfer[1]), &msg );

        /* Finalize and transmit. */
        msg.spi=spi_dev;
        msg.is_dma_mapped=0;
        return spi_sync( spi_dev, &msg );
}

int kxr94_spi_write( struct spi_device *spi_dev, unsigned char addr, unsigned char data )
{
        unsigned char                   tx_buf[2]={addr, data};
        struct spi_message              msg;
        struct spi_transfer             transfer;
        int                             retval;

        /* Prepare the data. */
        memset( &msg, 0, sizeof( msg ) );
        memset( &transfer, 0, sizeof( transfer ) );
        spi_message_init( &msg );

        /* Prepare the address cycle. */
        transfer.tx_buf=tx_buf;
        transfer.len=sizeof( tx_buf );
        transfer.delay_usecs=80;
        spi_message_add_tail( &transfer, &msg );

        /* Finalize and transmit. */
        msg.spi=spi_dev;
        msg.is_dma_mapped=0;
        retval=spi_sync( spi_dev, &msg );
        return retval;
}

static void kxr94_gadc_timer_handler( unsigned long  arg )
{
	struct channel_samples	*channel=(struct channel_samples *) arg;
	unsigned long int	sample_rate;
	unsigned long int	flags;
	u64			hpdiv;

	spin_lock_irqsave( &channel->lock, flags );
	sample_rate=gadc_get_samplerate( &channel->buffer );
	if( (channel->nr_samples > 0) && (channel->nr_samples >= (ADC_RATE/sample_rate)) )
	{
		hpdiv=((u64) channel->total_sample) << 20;
		do_div( hpdiv, channel->nr_samples );
		gadc_add_sample( &channel->buffer, (unsigned long int) hpdiv );
		channel->total_sample=0;
		channel->nr_samples=0;
		wake_up_interruptible( &channel->buffer.poll_queue );
	}
	channel->timer.expires = jiffies + ( HZ * 1000 +(sample_rate>>1))/(sample_rate);
	add_timer( &channel->timer );
	spin_unlock_irqrestore( &channel->lock, flags );
        return;
}

#ifdef CONFIG_PM
static int kxr94_gadc_suspend( struct spi_device *spi, pm_message_t mesg )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) spi->dev.driver_data;
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	unsigned long int	flags;
	int			count;
	int			kxr94_registered=0;

	/* Stop all timers, and flush all values out. */
	for( count=0; count < channel_data->nr_channels; count++ )
	{ 
		spin_lock_irqsave( &channel_data->channel[count].lock, flags );
		if( channel_data->channel[count].nr_samples >= 0 )
		{
			kxr94_registered=1;
			del_timer( &channel_data->channel[count].timer );
			channel_data->channel[count].total_sample=0;
			channel_data->channel[count].nr_samples=0;
			gadc_clear_buffer( &channel_data->channel[count].buffer );
		}
		spin_unlock_irqrestore( &channel_data->channel[count].lock, flags );
	}

	if( kxr94_registered )
		del_timer( &channel_data->timer );

	kxr94_spi_write( channel_data->spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_OFF );
	kxr94_exit_hw( channel_data->spi_dev );
	return 0;
}

static int kxr94_gadc_resume( struct spi_device *spi )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) spi->dev.driver_data;
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	unsigned long int	flags;
	int			count;
	int			kxr94_registered=0;

	/* Startup the hardware. */
	kxr94_init_hw( channel_data->spi_dev );
	kxr94_spi_write( channel_data->spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_ON );

	/* Stop all timers, and flush all values out. */
	for( count=0; count < channel_data->nr_channels; count++ )
	{ 
		spin_lock_irqsave( &channel_data->channel[count].lock, flags );
		if( channel_data->channel[count].nr_samples >= 0 )
		{
			kxr94_registered=1;
			kxr94_gadc_timer_handler( (unsigned long) &channel_data->channel[count] );
		}
		spin_unlock_irqrestore( &channel_data->channel[count].lock, flags );
	}

	if( kxr94_registered )
		kxr94_poll_timer_handler( (unsigned long) channel_data );

	return 0;
}
#else
#define kxr94_gadc_suspend	NULL
#define kxr94_gadc_resume	NULL
#endif

static int kxr94_gadc_remove( struct spi_device *spi )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) spi->dev.driver_data;
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;

        unregister_gadc_cdev( chardev );
	
	kfree( channel_data );
	kfree( chardev );

        return 0;
}

static void kxr94_gadc_poll_do_work( void* arg )
{
        struct kxr94_gadc       *channel_data=(struct kxr94_gadc *) arg;
        struct channel_samples  *channel;
        unsigned long int       flags;
        unsigned short          receive=0;
        int                     count=0;

        /* We just add up the samples and their count here. Averaging will be done from within the timer handler. */
	for( count=0; count < channel_data->nr_channels; count++ )
        {
                channel=&channel_data->channel[count];

		if( kxr94_spi_read( channel_data->spi_dev, channel->channel_nr, &receive ) )
			return;
		receive=kxr94_to_raw_value( (u8 *) &receive );

                spin_lock_irqsave( &channel->lock, flags );
                if( channel->nr_samples >= 0 )
                {
                        channel->total_sample+=(unsigned long int) receive;
                        channel->nr_samples+=1;
                }
                spin_unlock_irqrestore( &channel->lock, flags );
        }
        return;
}

static int kxr94_gadc_open( struct inode *inode, struct file *file )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) get_gadc_cdev( inode->i_cdev );
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	struct channel_samples	*channel=&(channel_data->channel[iminor( inode ) - chardev->minor]);
	unsigned long int	flags;
	int			count;
	int			kxr94_registered;
	unsigned long int	sample_rate;

	spin_lock_irqsave( &channel->lock, flags );
	if( channel->nr_samples >= 0 )
	{
		spin_unlock_irqrestore( &channel->lock, flags );
		return -EBUSY;
	}

	/* Check if any of the other registered channels has the adc poll started already. */
	kxr94_registered=0;
	for( count=0; count < channel_data->nr_channels; count++ )
	{
		if( channel_data->channel[count].nr_samples >= 0 )
			kxr94_registered=1;
	}

	if( !kxr94_registered )
	{
		memset( &channel_data->work, 0, sizeof( channel_data->work ) );
		memset( &channel_data->timer, 0, sizeof( channel_data->timer ) );
		for( count=0; count < channel_data->nr_channels; count++ )
		{
			channel_data->channel[count].total_sample=0;
			channel_data->channel[count].nr_samples=-1;
		}
		channel_data->work_queue=NULL;
		channel_data->flags=0;

		kxr94_init_hw( channel_data->spi_dev );
		if( kxr94_spi_write( channel_data->spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_ON ) )
		{
			spin_unlock_irqrestore( &channel->lock, flags );
			return -EINVAL;
		}
		init_timer( &(channel_data->timer) );
		channel_data->timer.function=kxr94_poll_timer_handler;
		channel_data->timer.data=((unsigned long int) channel_data);
		channel_data->timer.expires=jiffies + ((HZ + KXR94_POLL_RATE - 1)/KXR94_POLL_RATE);
		channel_data->work_queue=create_singlethread_workqueue( KXR94_GADC_DRIVER_NAME );
		INIT_WORK( &(channel_data->work), kxr94_gadc_poll_do_work, channel_data );
		add_timer( &(channel_data->timer) );
	}

	/* Initialize data. */
	gadc_init_buffer( &channel->buffer, 1 );
	init_timer( &channel->timer );

	channel->total_sample=0;
	channel->nr_samples=0;
	channel->timer.function=kxr94_gadc_timer_handler;
	channel->timer.data=((unsigned long int) channel);
	sample_rate=gadc_get_samplerate( &channel->buffer );
	channel->timer.expires=jiffies + (HZ * 1000 + (sample_rate >> 1))/sample_rate;
	file->private_data=chardev;

	/* Start timer. */
	add_timer( &channel->timer );
	spin_unlock_irqrestore( &channel->lock, flags );
	return 0;
}

static int kxr94_gadc_release(struct inode *inode, struct file *file )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) get_gadc_cdev( inode->i_cdev );
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	struct channel_samples	*channel=&(channel_data->channel[iminor( inode ) - chardev->minor]);
	unsigned long int	flags;
	int			kxr94_registered;
	int			count;

	spin_lock_irqsave( &channel->lock, flags );
	del_timer( &channel->timer );
	channel->total_sample=0;
	channel->nr_samples=-1;
	kxr94_registered=0;
	gadc_clear_buffer( &channel->buffer );
	for( count=0; count < channel_data->nr_channels; count++ )
	{
		if( channel_data->channel[count].nr_samples >= 0 )
			kxr94_registered=1;
	}

	if( !kxr94_registered )
	{
		channel_data->flags |= KXR94_ADC_FLGS_EXIT;
		flush_workqueue( channel_data->work_queue ); /* make sure nothin' is running anymore */
		destroy_workqueue( channel_data->work_queue );

		del_timer( &channel_data->timer );

		kxr94_spi_write( channel_data->spi_dev, KXR94_ADDR_WRITE_CTRL, KXR94_MODE_OFF );
		kxr94_exit_hw( channel_data->spi_dev );
	}

	spin_unlock_irqrestore( &channel->lock, flags );
	file->private_data=NULL;
        return 0;
}

static unsigned kxr94_gadc_poll(struct file *file, poll_table *wait)
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) file->private_data;
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	int			index=iminor( file->f_dentry->d_inode ) - chardev->minor;
	int			tmpval;
        unsigned int		mask = 0;

	gadc_poll_wait( file, &(channel_data->channel[index].buffer), wait );
	if ( gadc_get_samplecount( &channel_data->channel[index].buffer, &tmpval ) > 0 )
		mask |= POLLIN | POLLRDNORM;

        return mask;
}

static int kxr94_gadc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) file->private_data;
	struct kxr94_gadc	*channel_data=(struct kxr94_gadc *) chardev->private;
	int			index=iminor( inode ) - chardev->minor;
	unsigned long int	samplerate;

	switch( cmd )
	{
		case ADC_SET_SAMPLERATE:
			/* We are limited by HZ and the ADC rate, so make sure the samplerate matches this. */
			samplerate=(unsigned long int) arg;
			if( samplerate > 1000 * HZ ) samplerate=1000 * HZ;
			if( samplerate == 0 )
				return -EINVAL;

			samplerate=1000 * HZ/((HZ * 1000 + (samplerate >> 1))/samplerate);
			gadc_set_samplerate( &channel_data->channel[index].buffer, samplerate );

			/* Now reschedule the timer. Otherwise we will for one tick still run at the old rate. */
			del_timer( &channel_data->channel[index].timer );
			kxr94_gadc_timer_handler( (unsigned long) &channel_data->channel[index] );
			break;

		case ADC_GET_SAMPLERATE:
			*((unsigned long int *) arg)=gadc_get_samplerate( &channel_data->channel[index].buffer );
			break;

		case ADC_GET_BUFFERSTATUS:
			*((unsigned long int *) arg)=gadc_get_status( &channel_data->channel[index].buffer );
			break;

		default :
			return -ENOTTY;
	}

	return 0;
}

static ssize_t kxr94_gadc_read( struct file *file, char __user *buf, size_t count, loff_t * loff)
{
	struct gadc_cdev		*chardev=(struct gadc_cdev *) file->private_data;
	struct kxr94_gadc		*channel_data=(struct kxr94_gadc *) chardev->private;
	struct channel_samples		*channel=&(channel_data->channel[iminor( file->f_dentry->d_inode ) - chardev->minor]);
	struct gadc_sample		sample;
	int				samplecount;
	int				maxbufsamples;
	int				loop=0;

	/* Determine the amount of samples that'll fit in the buffer. */
	gadc_get_samplecount( &channel->buffer, &samplecount );

	/* If no samples are present block & wait. */
	if( samplecount == 0 )
	{
		if( !(file->f_flags & O_NONBLOCK) )
		{
			if( gadc_wait_buffer_fill( &(channel->buffer), &samplecount ) )
				return -ERESTARTSYS;
		}
		else return -EAGAIN;
	}

	/* Determine how many samples to read. */
	maxbufsamples=count/sizeof( sample );
	if( samplecount > maxbufsamples ) samplecount=maxbufsamples;

	/* Copy sample by sample. Since we need to copy to user space, do this in two steps. */
	for( loop=0; loop < samplecount; loop++ )
	{
		if( !gadc_get_sample( &channel->buffer, &sample ) )
			break;
		copy_to_user( &(((struct gadc_sample *) buf)[loop]), &sample, sizeof( sample ) );
	}

	/* Return the read bytes. */
	return loop * sizeof( struct gadc_sample );
}

struct file_operations kxr94_gadc_fops=
{
	.owner	=THIS_MODULE,
	.read	=kxr94_gadc_read,
	.poll	=kxr94_gadc_poll,

	.ioctl	=kxr94_gadc_ioctl,
	.open	=kxr94_gadc_open,
	.release=kxr94_gadc_release,
};

static void kxr94_init_p74(struct kxr94_gadc *channel_data, int nr_channels)
{
	int count = 0;

	/* Initialize private data. Note that the nr_samples field of each channel is set to -1 to indicate it is not used. */
	channel_data->nr_channels = nr_channels;
	for (count = 0; count < nr_channels; count++) {
		channel_data->channel[count].total_sample = 0;
		channel_data->channel[count].nr_samples = -1;
		channel_data->channel[count].channel_nr =
		    kxr94_p74_channels_array[count];
		spin_lock_init(&(channel_data->channel[count].lock));
		gadc_init_buffer(&channel_data->channel[count].buffer, 1);
	}
}

static void kxr94_init_r94(struct kxr94_gadc *channel_data, int nr_channels)
{
	int count = 0;
	/* Initialize private data. Note that the nr_samples field of each channel is set to -1 to indicate it is not used. */
	channel_data->nr_channels = nr_channels;
	for (count = 0; count < nr_channels; count++) {
		channel_data->channel[count].total_sample = 0;
		channel_data->channel[count].nr_samples = -1;
		channel_data->channel[count].channel_nr =
			kxr94_r94_channels_array[count];
		spin_lock_init(&(channel_data->channel[count].lock));
		gadc_init_buffer(&channel_data->channel[count].buffer, 1);
	}
}


static int kxr94_gadc_probe( struct spi_device *spi )
{
	struct gadc_platform_data *pdata = spi->dev.platform_data;
	struct kxr94_gadc *channel_data;
	struct gadc_cdev *chardev;
	int nr_channels;

	printk(KERN_INFO "TomTom GO KXR94 Generic ADC Driver, (C) 2007 TomTom BV\n");
#if 0
	/* Determine the number of channels we'll register for this driver. */
	nr_channels=KXR94_ADC_CHANNELS;
#endif
	/* Allocate memory for the structures. */
	channel_data=(struct kxr94_gadc *) kmalloc( sizeof( struct kxr94_gadc ), GFP_KERNEL );
	if( channel_data == NULL )
	{
		printk( KXR94_GADC_DRIVER_NAME": Error allocating memory for private data!\n" );
		return -ENOMEM;
	}

	chardev=(struct gadc_cdev *) kmalloc( sizeof( struct gadc_cdev ), GFP_KERNEL );
	if( chardev == NULL )
	{
		printk( KXR94_GADC_DRIVER_NAME": Error allocating memory for cdev!\n" );
		kfree( channel_data );
		return -ENOMEM;
	}
#if 0
	/* Initialize private data. Note that the nr_samples field of each channel is set to -1 to indicate it is not used. */
	channel_data->nr_channels=nr_channels;
	for( count=0; count < nr_channels; count++ )
	{
		channel_data->channel[count].total_sample=0;
		channel_data->channel[count].nr_samples=-1;
		channel_data->channel[count].channel_nr=kxr94_sample_addr_array[count];
	        spin_lock_init( &(channel_data->channel[count].lock) );
		gadc_init_buffer( &channel_data->channel[count].buffer, 1 );
#endif
		/* Device specific part depends on ModelId */
	switch (IO_GetModelId()) {
		case GOTYPE_CAGLIARI:
		case GOTYPE_TREVISO:
			nr_channels = KXR94_R94_CHANNELS;
			kxr94_init_r94(channel_data, nr_channels);
			break;
		case GOTYPE_MILAN:
		case GOTYPE_MODENA:
			nr_channels = KXR94_P74_CHANNELS;
			kxr94_init_p74(channel_data, nr_channels);
			break;
		default:
			printk(KXR94_GADC_DRIVER_NAME ": Error - wrong platform !\n");
			kfree(channel_data);
			kfree(chardev);
			return -ENODEV;
			break;
	}



	/* Register the character device. */
	printk( "Registring %i channels, starting at %u.%u\n", nr_channels, pdata[0].major, pdata[0].minor );
//@@@@@ ALLEEN EERSTE KANAAL WERKT. SAMPLERATE MAX IS 40 ????
	if( register_gadc_cdev( chardev, pdata[0].major, pdata[0].minor, nr_channels, KXR94_GADC_DRIVER_NAME, &kxr94_gadc_fops ) )
	{
		printk( KXR94_GADC_DRIVER_NAME": Error registring cdev!\n" );
		kfree( channel_data );
		kfree( chardev );
		return -ENODEV;
	}
	chardev->private=channel_data;
	channel_data->spi_dev=spi;

	/* Set private device data. */
	spi->dev.driver_data=chardev;

	/* Done. */
	return 0;
}

struct spi_driver kxr94_gadc_driver=
{

        .driver=
        {
                .name   = KXR94_GADC_DRIVER_NAME,
                .bus    = &spi_bus_type,
                .owner  = THIS_MODULE,
        },
        .probe          = kxr94_gadc_probe,
        .remove         = kxr94_gadc_remove,
        .suspend        = kxr94_gadc_suspend,
        .resume         = kxr94_gadc_resume,
};

static int __init kxr94_gadc_init( void )
{
	int	retval;

        retval=spi_register_driver(&kxr94_gadc_driver);
	if( retval ) return retval;
	return 0;
}

module_init( kxr94_gadc_init );

static void __exit kxr94_gadc_exit( void )
{
	spi_unregister_driver(&kxr94_gadc_driver);
}
module_exit(kxr94_gadc_exit);

MODULE_DESCRIPTION("KXR94 GADC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");

