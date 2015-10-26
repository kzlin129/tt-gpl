/* drivers/barcelona/gadc/s3c24xx_gadc.c
 *
 * Implementation of the S3C24XX Generic ADC driver. 
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
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <barcelona/debug.h>
#include <barcelona/gadc.h>
#include <barcelona/Barc_adc.h>
#include "s3c24xx_gadc.h"

static void s3c24xx_gadc_timer_handler( unsigned long  arg )
{
	struct channel_samples	*channel=(struct channel_samples *) arg;
	unsigned long int	sample_rate;
	unsigned long int	flags;
	u64			hpdiv;

	spin_lock_irqsave( &channel->lock, flags );
	sample_rate=gadc_get_samplerate( &channel->buffer );
	if( (channel->nr_samples > 0) && (channel->nr_samples >= (ADC_RATE/sample_rate)) )
	{
		hpdiv=((u64) channel->total_sample) << 22;
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
static int s3c24xx_gadc_suspend( struct device *dev, pm_message_t state, u32 level )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) dev->driver_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	unsigned long int	flags;
	int			count;

        if( level != SUSPEND_POWER_DOWN )
		return 0;

	/* Stop all timers, and flush all values out. */
	for( count=0; count < channel_data->nr_channels; count++ )
	{ 
		spin_lock_irqsave( &channel_data->channel[count].lock, flags );
		if( channel_data->channel[count].nr_samples >= 0 )
		{
			del_timer( &channel_data->channel[count].timer );
			channel_data->channel[count].total_sample=0;
			channel_data->channel[count].nr_samples=0;
			gadc_clear_buffer( &channel_data->channel[count].buffer );
		}
		spin_unlock_irqrestore( &channel_data->channel[count].lock, flags );
	}
	return 0;
}

static int s3c24xx_gadc_resume( struct device *dev, u32 level )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) dev->driver_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	unsigned long int	flags;
	int			count;

        if( level != RESUME_POWER_ON )
		return 0;

	/* Stop all timers, and flush all values out. */
	for( count=0; count < channel_data->nr_channels; count++ )
	{ 
		spin_lock_irqsave( &channel_data->channel[count].lock, flags );
		if( channel_data->channel[count].nr_samples >= 0 )
			s3c24xx_gadc_timer_handler( (unsigned long) &channel_data->channel[count] );
		spin_unlock_irqrestore( &channel_data->channel[count].lock, flags );
	}
	return 0;
}
#else
#define s3c24xx_gadc_suspend	NULL
#define s3c24xx_gadc_resume	NULL
#endif

static int s3c24xx_gadc_remove( struct device *dev )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) dev->driver_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;

        unregister_gadc_cdev( chardev );
	
	kfree( channel_data->channel );
	kfree( channel_data );
	kfree( chardev );

        return 0;
}

static void s3c24xx_gadc_adc_poll( short adcBuffer[ADC_CHANNELS], void* arg )
{
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) arg;
	struct channel_samples	*channel;
	unsigned long int	flags;
	int			count=0;

	/* We just add up the samples and their count here. Averaging will be done from within the timer handler. */
	for( count=0; count < channel_data->nr_channels; count++ )
	{
		channel=&channel_data->channel[count];

		spin_lock_irqsave( &channel->lock, flags );
		if( channel->nr_samples >= 0 )
		{
			channel->total_sample+=(unsigned long int) adcBuffer[channel->channel_nr];
			channel->nr_samples+=1;
		}
		spin_unlock_irqrestore( &channel->lock, flags );
	}
	return;
}

static int s3c24xx_gadc_open( struct inode *inode, struct file *file )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) get_gadc_cdev( inode->i_cdev );
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	struct channel_samples	*channel=&(channel_data->channel[iminor( inode ) - chardev->minor]);
	unsigned long int	flags;
	int			count;
	int			adc_registered;
	unsigned long int	sample_rate;

	spin_lock_irqsave( &channel->lock, flags );
	if( channel->nr_samples >= 0 )
	{
		spin_unlock_irqrestore( &channel->lock, flags );
		return -EBUSY;
	}

	/* Check if any of the other registered channels has the adc poll started already. */
	adc_registered=0;
	for( count=0; count < channel_data->nr_channels; count++ )
	{
		if( channel_data->channel[count].nr_samples >= 0 )
			adc_registered=1;
	}

	/* Initialize data. */
	gadc_init_buffer( &channel->buffer, 1 );
	init_timer( &channel->timer );

	channel->total_sample=0;
	channel->nr_samples=0;
	channel->timer.function=s3c24xx_gadc_timer_handler;
	channel->timer.data=((unsigned long int) channel);
	sample_rate=gadc_get_samplerate( &channel->buffer );
	channel->timer.expires=jiffies + (HZ * 1000 + (sample_rate >> 1))/sample_rate;
	file->private_data=chardev;

	/* Register to ADC poll handler. */
	if( !adc_registered )
	{
		if( adc_register_poll(s3c24xx_gadc_adc_poll, channel_data, ADC_RATE_ACC ) )
		{
			spin_unlock_irqrestore( &channel->lock, flags );
			return -ENOMEM;
		}
	}

	/* Start timer. */
	add_timer( &channel->timer );
	spin_unlock_irqrestore( &channel->lock, flags );
	return 0;
}

static int s3c24xx_gadc_release(struct inode *inode, struct file *file )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) get_gadc_cdev( inode->i_cdev );
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	struct channel_samples	*channel=&(channel_data->channel[iminor( inode ) - chardev->minor]);
	unsigned long int	flags;
	int			adc_registered;
	int			count;

	spin_lock_irqsave( &channel->lock, flags );
	del_timer( &channel->timer );
	channel->total_sample=0;
	channel->nr_samples=-1;
	adc_registered=0;
	gadc_clear_buffer( &channel->buffer );
	for( count=0; count < channel_data->nr_channels; count++ )
	{
		if( channel_data->channel[count].nr_samples >= 0 )
			adc_registered=1;
	}

	if( !adc_registered )
		adc_unregister_poll(s3c24xx_gadc_adc_poll );

	spin_unlock_irqrestore( &channel->lock, flags );
	file->private_data=NULL;
        return 0;
}

static unsigned s3c24xx_gadc_poll(struct file *file, poll_table *wait)
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) file->private_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	int			index=iminor( file->f_dentry->d_inode ) - chardev->minor;
	int			tmpval;
        unsigned int		mask = 0;

	gadc_poll_wait( file, &(channel_data->channel[index].buffer), wait );
	if ( gadc_get_samplecount( &channel_data->channel[index].buffer, &tmpval ) > 0 )
		mask |= POLLIN | POLLRDNORM;

        return mask;
}

static int s3c24xx_gadc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) file->private_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	int			index=iminor( inode ) - chardev->minor;
	unsigned long int	samplerate;

	switch( cmd )
	{
		case ADC_SET_SAMPLERATE:
			/* We are limited by HZ and the ADC rate, so make sure the samplerate matches this. */
			samplerate=(unsigned long int) arg;
			if( samplerate > 1000 * HZ ) samplerate=1000 * HZ;
			if( samplerate > 1000 * ADC_RATE ) samplerate=1000 * ADC_RATE;
			if( samplerate == 0 )
				return -EINVAL;

			samplerate=1000 * HZ/((HZ * 1000 + (samplerate >> 1))/samplerate);
			gadc_set_samplerate( &channel_data->channel[index].buffer, samplerate );

			/* Now reschedule the timer. Otherwise we will for one tick still run at the old rate. */
			del_timer( &channel_data->channel[index].timer );
			s3c24xx_gadc_timer_handler( (unsigned long) &channel_data->channel[index] );
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

static ssize_t s3c24xx_gadc_read( struct file *file, char __user *buf, size_t count, loff_t * loff)
{
	struct gadc_cdev	*chardev=(struct gadc_cdev *) file->private_data;
	struct s3c24xx_gadc	*channel_data=(struct s3c24xx_gadc *) chardev->private;
	struct channel_samples	*channel=&(channel_data->channel[iminor( file->f_dentry->d_inode ) - chardev->minor]);
	struct gadc_sample	sample;
	int			samplecount;
	int			maxbufsamples;
	int			loop=0;

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

struct file_operations s3c24xx_gadc_fops=
{
	.owner	=THIS_MODULE,
	.read	=s3c24xx_gadc_read,
	.poll	=s3c24xx_gadc_poll,

	.ioctl	=s3c24xx_gadc_ioctl,
	.open	=s3c24xx_gadc_open,
	.release=s3c24xx_gadc_release,
};

static int s3c24xx_gadc_probe( struct device *dev )
{
	struct s3c24xx_gadc		*channel_data;
	struct gadc_platform_data	*pdata=(struct gadc_platform_data *) dev->platform_data;
	struct gadc_cdev		*chardev;
	int				count=0;
	int				nr_channels;

	printk(KERN_INFO "TomTom GO S3C24XX Generic ADC Driver, (C) 2007 TomTom BV\n");
	if( pdata == NULL )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": Missing platform data!\n" );
		return -ENODEV;
	}

	/* Determine the number of channels we'll register for this driver. */
	for( nr_channels=0; pdata[nr_channels].major != -1; nr_channels++ );
	if( nr_channels == 0 )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": No channels detected!\n" );
		return -ENODEV;
	}

	/* Allocate memory for the structures. */
	channel_data=(struct s3c24xx_gadc *) kmalloc( sizeof( struct s3c24xx_gadc ), GFP_KERNEL );
	if( channel_data == NULL )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": Error allocating memory for private data!\n" );
		return -ENOMEM;
	}

	channel_data->channel=(struct channel_samples *) kmalloc( nr_channels * sizeof( struct channel_samples ), GFP_KERNEL );
	if( channel_data->channel == NULL )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": Error allocating memory for sample buffers!\n" );
		kfree( channel_data );
		return -ENOMEM;
	}

	chardev=(struct gadc_cdev *) kmalloc( sizeof( struct gadc_cdev ), GFP_KERNEL );
	if( chardev == NULL )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": Error allocating memory for cdev!\n" );
		kfree( channel_data->channel );
		kfree( channel_data );
		return -ENOMEM;
	}

	/* Initialize private data. Note that the nr_samples field of each channel is set to -1 to indicate it is not used. */
	channel_data->nr_channels=nr_channels;
	for( count=0; count < nr_channels; count++ )
	{
		channel_data->channel[count].total_sample=0;
		channel_data->channel[count].nr_samples=-1;
		channel_data->channel[count].channel_nr=pdata[count].channel;
	        spin_lock_init( &(channel_data->channel[count].lock) );
		gadc_init_buffer( &channel_data->channel[count].buffer, 1 );
	}

	/* Register the character device. */
	if( register_gadc_cdev( chardev, pdata[0].major, pdata[0].minor, nr_channels, S3C24XX_GADC_DRIVER_NAME, &s3c24xx_gadc_fops ) )
	{
		printk( S3C24XX_GADC_DRIVER_NAME": Error registring cdev!\n" );
		kfree( channel_data->channel );
		kfree( channel_data );
		kfree( chardev );
		return -ENODEV;
	}
	chardev->private=channel_data;

	/* Set private device data. */
	dev->driver_data=chardev;

	/* Done. */
	return 0;
}

static struct device_driver s3c24xx_gadc_driver=
{
	.name		= S3C24XX_GADC_DRIVER_NAME,
	.bus		= &platform_bus_type,
	.probe		= s3c24xx_gadc_probe,
	.remove		= s3c24xx_gadc_remove,
	.suspend	= s3c24xx_gadc_suspend,
	.resume		= s3c24xx_gadc_resume,
};

static int __init s3c24xx_gadc_init( void )
{
	int	retval;

	retval=driver_register(&s3c24xx_gadc_driver);
	if( retval ) return retval;
	return 0;
}

module_init( s3c24xx_gadc_init );

static void __exit s3c24xx_gadc_exit( void )
{
	driver_unregister(&s3c24xx_gadc_driver);
}
module_exit(s3c24xx_gadc_exit);

MODULE_DESCRIPTION("S3C24XX GADC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");

