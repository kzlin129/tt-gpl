/* drivers/barcelona/gacc/kxr94_adc.c
 *
 * Implementation of the Kionix KXP74 accelerometer SPI protocol driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <Mark.Vels@tomtom.com>, Rogier Stam <Rogier.Stam@tomtom.com> (KXR94 part)
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
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
/* Include generic accelerometer interface definition	*/
#include <barcelona/debug.h>

#include <barcelona/gadc.h>
#include <barcelona/Barc_adc.h>
#include "kxp74.h"
#include "kxr94_adc.h"

int kxr94_spi_read( struct spi_device *spi_dev, unsigned char cmd, unsigned short *data )
{
	struct spi_message		msg;
	struct spi_transfer		transfer[2];

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
	transfer[1].rx_buf=(u8 *) (cmd != KXP74_ADDR_READ_CTRL ? data : (data + 1) );
	transfer[1].len=(cmd != KXP74_ADDR_READ_CTRL ? 2 : 1);
	transfer[1].delay_usecs=80;
	spi_message_add_tail( &(transfer[1]), &msg );

	/* Finalize and transmit. */
	msg.spi=spi_dev;
	msg.is_dma_mapped=0;
	return spi_sync( spi_dev, &msg );
}

int kxr94_spi_write( struct spi_device *spi_dev, unsigned char addr, unsigned char data )
{
	unsigned char			tx_buf[2]={addr, data};
	struct spi_message		msg;
	struct spi_transfer		transfer;
	int				retval;

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

static void kxr94_adc_do_work( void* arg )
{
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) arg;
	unsigned short			receive=0;

	if( kxr94_spi_read( private->spi_dev, KXP94_ADDR_CONVERT_AUX, &receive ) )
		return;

	receive=kxp74_to_raw_value( (u8 *) &receive );

	/* To keep the samples exactly the same for any ADC regardless of how many bits per sample */
	/* we shift the sample into the highest 12 bits. */
	gadc_add_sample( private->adc, ((unsigned long int) receive) << 20 );
	wake_up_interruptible( &private->adc->poll_queue );
	return;
}	

/**
 * Kernel timer handler function.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
void kxr94_adc_timer_handler( unsigned long  arg )
{
	unsigned long int		sample_rate;
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) arg;;
	
	if( ! (private->flags & KXR94_ADC_FLGS_EXIT) )
	{
		/* if we must exit then return and do not reschedule */
		sample_rate = gadc_get_samplerate( private->adc );
		if( queue_work( private->work_queue, &private->work ) )
		{
			/* There obviously is congestion if we try to schedule work
			 * while the previous scheduled one is not finished yet	*/	
			 private->flags|= KXR94_ADC_FLGS_CONGESTION;
		}
		
		private->timer.expires = jiffies + (HZ * 1000 +(sample_rate>>1))/(sample_rate);
		add_timer( &private->timer );
	}
	return;
}

static int kxr94_adc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) file->private_data;
	unsigned long int		samplerate;

	switch( cmd )
	{
		case ADC_SET_SAMPLERATE:
			/* We are limited by HZ, so make sure the samplerate matches this. */
			samplerate=(unsigned long int) arg;
			if( samplerate > 1000 * HZ ) samplerate=1000 * HZ;
			if( samplerate == 0 )
				return -EINVAL;
			samplerate=1000*HZ/((1000 * HZ + (samplerate >> 1))/samplerate);
			gadc_set_samplerate( private->adc, samplerate );

			/* Now reschedule the timer. Otherwise we will for one tick still run at the old rate. */
			del_timer( &private->timer );
			flush_workqueue( private->work_queue );
			kxr94_adc_timer_handler( (unsigned long int) private );
			break;

		case ADC_GET_SAMPLERATE:
			*((unsigned long int *) arg)=gadc_get_samplerate( private->adc );
			break;

		case ADC_GET_BUFFERSTATUS:
			*((unsigned long int *) arg)=gadc_get_status( private->adc );
			break;

		default :
			return -ENOTTY;
	}

	return 0;
}

static int kxr94_adc_release(struct inode *inode, struct file *file )
{
	struct gadc_cdev		*adc_cdev=get_gadc_cdev( inode->i_cdev );
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) container_of( adc_cdev, struct kxp74_cdev, kxr94_adc_dev );
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) adc_cdev->private;

	if( private )
	{
		private->flags |= KXR94_ADC_FLGS_EXIT;

		/* Remove any scheduled timer .*/
		del_timer( &private->timer );
		
		flush_workqueue( private->work_queue );	/* make sure nothin' is running anymore	*/
		destroy_workqueue( private->work_queue );
		
		/* shut down the kxp inner circuits	*/
		if( chardev->kxp74_dev.private == NULL )
		{
			kxr94_spi_write( chardev->spi_dev, KXP74_ADDR_WRITE_CTRL, KXR94_MODE_OFF );
		
			/* power off most stuff to save power	*/
			kxp74_exit_hw( chardev->spi_dev );
		}
		
		/* Free up used memory. */
		kfree( private->adc );
		kfree( private );
		adc_cdev->private=NULL;
		file->private_data=NULL;
	}
	return 0;
}

static int kxr94_adc_open( struct inode *inode, struct file *file )
{
	struct gadc_cdev		*adc_cdev=get_gadc_cdev( inode->i_cdev );
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) container_of( adc_cdev, struct kxp74_cdev, kxr94_adc_dev );
	struct spi_device		*spi_dev=chardev->spi_dev;
	struct gadc_buffer		*kxr94_adc=NULL;
	struct kxr94_adc_private	*private=NULL;
	unsigned long int		sample_rate;

	if( chardev->kxr94_adc_dev.private )
		return -EBUSY;

	kxr94_adc=(struct gadc_buffer *) kmalloc( sizeof( *kxr94_adc ), GFP_KERNEL );
	if( kxr94_adc == NULL )
		return -ENOMEM;

	/* Initialize the ADC local fields. */
	gadc_init_buffer( kxr94_adc, 1 );

	/* Now set the private data structure. */
	private=(struct kxr94_adc_private *) kmalloc( sizeof( struct kxr94_adc_private ), GFP_KERNEL );
	if( private == NULL )
	{
		kfree( kxr94_adc );
		return -ENOMEM;
	}
	memset( private, 0, sizeof( struct kxr94_adc_private ) );
	private->adc=kxr94_adc;
	private->spi_dev=spi_dev;
	init_timer( &(private->timer) );
	private->timer.function=kxr94_adc_timer_handler;
	private->timer.data=((unsigned long int) private);
	sample_rate=gadc_get_samplerate( kxr94_adc );
	private->timer.expires=jiffies + (HZ * 1000 +(sample_rate>>1))/(sample_rate);
	private->work_queue=create_singlethread_workqueue( KXR94_DRIVER_NAME );
	INIT_WORK( &(private->work), kxr94_adc_do_work, private );
	file->private_data=private;
	chardev->kxr94_adc_dev.private=private;
	add_timer( &(private->timer) );
	
	/* START THE KXR94 if it wasn't already!!! */
	if( chardev->kxp74_dev.private == NULL )
	{
		/* Chip wasn't started yet. Start it now. */
		kxp74_init_hw( private->spi_dev );
		kxr94_spi_write( private->spi_dev, KXP74_ADDR_WRITE_CTRL, KXR94_MODE_ON );
	}

	return 0;
}

static ssize_t kxr94_adc_read( struct file *file, char __user *buf, size_t count, loff_t * loff)
{
	int				samplecount;
	int				maxbufsamples;
	int				loop=0;
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) file->private_data;
	struct gadc_sample		sample;

	/* Determine the amount of samples that'll fit in the buffer. */
	gadc_get_samplecount( private->adc, &samplecount );

	/* If no samples are present block & wait. */
	if( samplecount == 0 )
	{
		if( !(file->f_flags & O_NONBLOCK) )
		{
			if( gadc_wait_buffer_fill( private->adc, &samplecount ) )
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
		if( !gadc_get_sample( private->adc, &sample ) )
			break;
		copy_to_user( &(((struct gadc_sample *) buf)[loop]), &sample, sizeof( sample ) );
	}

	/* Return the read bytes. */
	return loop * sizeof( struct gadc_sample );
}

static unsigned kxr94_adc_poll(struct file *file, poll_table *wait)
{
	struct kxr94_adc_private	*private=(struct kxr94_adc_private *) file->private_data;
	unsigned int			mask=0;
	int				tmpval;

	gadc_poll_wait(file, private->adc, wait);
	if ( gadc_get_samplecount( private->adc, &tmpval ) > 0 )
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

struct file_operations kxr94_adc_fops=
{
	.owner		= THIS_MODULE,
	.read		= kxr94_adc_read,
	.poll		= kxr94_adc_poll,

	.ioctl		= kxr94_adc_ioctl,
	.open		= kxr94_adc_open,
	.release	= kxr94_adc_release,
};

struct spi_driver kxr94_driver=
{
	
	.driver=
	{
		.name	= KXR94_DRIVER_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= kxp74_probe,
	.remove		= kxp74_remove,
	.suspend	= kxp74_suspend,
	.resume		= kxp74_resume,
};

EXPORT_SYMBOL( kxr94_adc_fops );
EXPORT_SYMBOL( kxr94_driver );
EXPORT_SYMBOL( kxr94_spi_read );
EXPORT_SYMBOL( kxr94_spi_write );
EXPORT_SYMBOL( kxr94_adc_timer_handler );
