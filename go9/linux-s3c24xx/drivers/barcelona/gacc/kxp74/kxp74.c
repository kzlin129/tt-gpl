/* drivers/barcelona/gacc/kxp74.c
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
#include <barcelona/gacc.h>
#include <barcelona/debug.h>

#include <barcelona/gadc.h>
#include <barcelona/Barc_adc.h>
#include "kxr94_adc.h"
#include "kxp74.h"

/*-----------------------------------------------------------------------------*/
/* Global variables															   */
/*-----------------------------------------------------------------------------*/

static accelerometer_settings_t	kxp74_mode_settings = {
		.sensitivity	= 819,	/* typical datasheet rating		*/
		.range		= 0x07ff,
};

/*-----------------------------------------------------------------------------*/

#ifdef SW_TEST

static void test_fill_testbuffers(struct gacc_driver_private_data* private ){
	int i;
	u8*	t = (u8*) (private->head-1);	/* Address of extra element at start of the start	*/
	for(i = 0; i < sizeof(accelerometer_reading_t); i++){
		t[i] = 0xaa;
	}
	t = (u8*) &private->head[private->readings_size]; /* address of extra element at the end	*/	
	for(i = 0; i < sizeof(accelerometer_reading_t); i++){
		t[i] = 0xaa;
	}	
}

static void test_check_buffers( struct gacc_driver_private_data* private ){
	int i;
	u8*	t = (u8*) (private->head-1);	/* Address of extra element at start of the start	*/
	for(i = 0; i < sizeof(accelerometer_reading_t); i++){
		if( t[i] != 0xaa ){
			printk("***** BUFFER CORRUPTION AT START ELEMENT!!	(buffer address = 0x%04x)\n", t );	
		}
	}
	t = (u8*) &private->head[private->readings_size]; /* address of extra element at the end	*/	
	for(i = 0; i < sizeof(accelerometer_reading_t); i++){
		if( t[i] != 0xaa ){
			printk("***** BUFFER CORRUPTION AT END ELEMENT!!	(buffer address = 0x%04x)\n", t );	
		}
	}
}
#endif /* SW_TEST	*/

/*-----------------------------------------------------------------------------*/
int kxp74_init_hw( struct spi_device *spi_dev )
{
#ifndef SW_TEST
	gopin_t chipselect = spi_dev->chip_select;
#endif /* SW_TEST	*/

#ifndef SW_TEST
	IOP_Deactivate( chipselect );
	IOP_SetFunction( chipselect );
#endif /* SW_TEST	*/

	return -EINVAL;
}
EXPORT_SYMBOL( kxp74_init_hw );
 
/*-----------------------------------------------------------------------------*/
int kxp74_exit_hw( struct spi_device *spi_dev ){
	
#ifndef SW_TEST	
	gopin_t chipselect = spi_dev->chip_select;
	
	IOP_SetInput( chipselect );	/* Disable CS	*/
#endif /* SW_TEST	*/	
	return 0;
}
EXPORT_SYMBOL( kxp74_exit_hw );

/*-----------------------------------------------------------------------------*/
/* return positive 8-bit result or negative error */
#if 0
static int kxp74_read_control_register( struct gacc_driver_private_data* private ){

	struct spi_message msg;
	struct spi_transfer txf[2];
	u8* rx_buf;
	u8* tx_buf;
	int result;

	if( private == NULL ){
		return -EINVAL;	
	}

	rx_buf = ((kxp74_private_t*)(private->private))->rx_buf;
	tx_buf = ((kxp74_private_t*)(private->private))->tx_buf;
	
	/* Setup spi message	*/
	memset( rx_buf, 0 , KXP74_BUF_SIZE );	
	memset( tx_buf, 0 , KXP74_BUF_SIZE );	
	memset( &msg, 0 , sizeof( msg ));		/* Explicitly clear all members	*/
	spi_message_init( &msg );
	memset( &txf[0], 0 , sizeof( txf ));	/* Explicitly clear all members	*/
	
	tx_buf[0] = KXP74_ADDR_READ_CTRL;
	txf[0].tx_buf = &tx_buf[0];
	txf[0].len = 1;
	spi_message_add_tail( &txf[0], &msg );		/* add this transfer	*/
	txf[1].rx_buf = &rx_buf[0];
	txf[1].len = 1;
	spi_message_add_tail( &txf[1], &msg );		/* add this transfer	*/
	
	msg.spi	= private->spi_dev;
	msg.is_dma_mapped = 0;
	result = spi_sync( private->spi_dev, &msg );
	if( result ){
		result = (result>0 ? -result : result);
	}
	else {
		result = rx_buf[0];
	}
	return result;	
}
#endif

/*-----------------------------------------------------------------------------*/
/* Write an 8-bit value, return 0 on success  */
static int kxp74_write_control_register( struct gacc_driver_private_data* private, u8 val ){

	struct spi_message msg;
	struct spi_transfer txf;
	u8* tx_buf;
	int result;

	if( private == NULL ){
		return -EINVAL;	
	}

	tx_buf = ((kxp74_private_t*)(private->private))->tx_buf;
	
	/* Setup spi message	*/
	memset( &msg, 0 , sizeof( msg ));	/* Explicitly clear all members	*/
	memset( &txf, 0 , sizeof( txf ));	/* Explicitly clear all members	*/
	spi_message_init( &msg );

	tx_buf[0] = KXP74_ADDR_WRITE_CTRL;
	tx_buf[1] = val;
	txf.tx_buf = &tx_buf[0];
	txf.len = 2;
	spi_message_add_tail( &txf, &msg );		/* add this transfer	*/
	
	msg.spi	= private->spi_dev;
	msg.is_dma_mapped = 0;
	result = spi_sync( private->spi_dev, &msg );
	return result;	
}
/*-----------------------------------------------------------------------------*/

/**
 * Query the KXP74 for a reading. Store the results in r.
 */
static int kxp74_get_reading( struct gacc_driver_private_data* private, accelerometer_reading_t* r ){

	u8* rx_buf;
	u8* tx_buf;
	struct spi_message msg;
	struct spi_transfer txf[ 3 * 2];	/* 3 times (one write cycle and one read cycle) = 6	*/
	struct spi_transfer* temp;
	int result;

	if( (private == NULL) || ( r == NULL) ){
		return -EINVAL;	
	}
	rx_buf = ((kxp74_private_t*)(private->private))->rx_buf;
	tx_buf = ((kxp74_private_t*)(private->private))->tx_buf;
	
	/* Setup spi message	*/
	memset( rx_buf, 0 , KXP74_BUF_SIZE );	
	memset( &msg, 0 , sizeof( msg ));		/* Explicitly clear all members	*/
	spi_message_init( &msg );
	memset( &txf[0], 0 , sizeof( txf ));	/* Explicitly clear all members	*/
	temp = &txf[0];
	
	/* prepare address write cycle transfer X-axis */
	tx_buf[0] = KXP74_ADDR_CONVERT_X;
	temp->tx_buf = &tx_buf[0];
	temp->len = 1;
	temp->delay_usecs = 80; /* TODO: tune this value, spec says: 40 usecs	*/
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/
	temp++;
	/* prepare read transfer	*/
	temp->rx_buf = &rx_buf[0];
	temp->len = 2;
	temp->delay_usecs = 80;
	temp->cs_change = 1;
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/
	temp++;
	
	/* prepare address write cycle transfer Z-axis */
	tx_buf[1] = KXP74_ADDR_CONVERT_Z;
	temp->tx_buf = &tx_buf[1];
	temp->len = 1;
	temp->delay_usecs = 80; /* TODO: tune this value, spec says: 40 usecs	*/
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/
	temp++;
	/* prepare read transfer	*/
	temp->rx_buf = &rx_buf[2];
	temp->len = 2;
	temp->delay_usecs = 80;
	temp->cs_change = 1;
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/
	temp++;
	
	/* prepare address write cycle transfer Y-axis */
	tx_buf[2] = KXP74_ADDR_CONVERT_Y;
	temp->tx_buf = &tx_buf[2];
	temp->len = 1;
	temp->delay_usecs = 80; /* TODO: tune this value, spec says: 40 usecs	*/
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/
	temp++;
	/* prepare read transfer	*/
	temp->rx_buf = &rx_buf[4];
	temp->len = 2;
	/* Don't set cs_change here, at the last trx it will keep the chip select active	*/
	spi_message_add_tail( temp, &msg );		/* add this transfer	*/

	msg.spi	= private->spi_dev;
	msg.is_dma_mapped = 0;
#ifdef SW_TEST
	result = fill_with_fake_data( rx_buf );
#else
	result = spi_sync( private->spi_dev, &msg );
#endif /*	SW_TEST	*/
	//printk("READ_RES = %d, X 0x%02x%02x  Y 0x%02x%02x  Z 0x%02x%02x\n", result, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5] );
	
	do_gettimeofday( &r->timestamp );	/* assmuption: delay after spi trx is shorter than before trx	*/

	if( ! result){
		unsigned short x;
		unsigned short y;
		unsigned short z;
		x = kxp74_to_raw_value( rx_buf );
		z = kxp74_to_raw_value( rx_buf + 2 );
		y = kxp74_to_raw_value( rx_buf + 4 );
#ifdef GACC_DEBUG
		r->x_raw = x;
		r->z_raw = z;
		r->y_raw = y;
#endif /* GACC_DEBUG	*/
		r->x_axis = kxp74_to_signed_short( x );
		r->y_axis = kxp74_to_signed_short( y );
		r->z_axis = kxp74_to_signed_short( z );
		
		//printk("CONVERTED RESULT X %d  Y %d  Z %d\n", r->x_axis, r->y_axis, r->z_axis );
	}
	return result;
}

/*-----------------------------------------------------------------------------*/
/**
 * Kernel timer handler function.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
static void kxp74_timer_handler( unsigned long  arg ){
	
	unsigned short sample_rate;
	struct gacc_driver_private_data* kxp74_private = (struct gacc_driver_private_data*) arg;
	
	if( ! ( kxp74_private->flags & GACC_FLGS_EXIT ) ){ /* if we must exit then return and do not reschedule	*/
		sample_rate = kxp74_private->settings->sample_rate;
		if( queue_work( kxp74_private->work_queue, &kxp74_private->work ) ){
			/* There obviously is congestion if we try to schedule work
			 * while the previous scheduled one is not finished yet	*/	
			 kxp74_private->flags|= GACC_FLGS_CONGESTION;
		}
		
		kxp74_private->timer.expires = jiffies + ( HZ +(sample_rate>>1))/(sample_rate);
		add_timer( &kxp74_private->timer );
	}
}

/*-----------------------------------------------------------------------------*/

static void kxp74_do_work( void* arg ){
	int result;
	accelerometer_reading_t temp;
	struct gacc_driver_private_data* kxp74_private = (struct gacc_driver_private_data*) arg;
	
	/*
	 * We will use a temporary struct to pass to get_reading() so that we don't have to lock the private data too long.
	 * kxp74_get_reading() might sleep.
	 */ 
	result = kxp74_get_reading( kxp74_private,  &temp);
	
	spin_lock( &kxp74_private->lock );
	CHECK_BUFFERS( kxp74_private );
	memcpy( &kxp74_private->head[kxp74_private->readings_writeindex], &temp, sizeof(accelerometer_reading_t) );
#ifdef GACC_DEBUG
	kxp74_private->head[kxp74_private->readings_writeindex].seq = kxp74_private->seq++;
#endif	/*	GACC_DEBUG	*/
	CHECK_BUFFERS( kxp74_private );
	if( !result ){
		if( (kxp74_private->readings_writeindex==kxp74_private->readings_readindex) && (kxp74_private->readings_count > 0 ) ){
			/* We're overwriting something!	*/
			kxp74_private->flags |= GACC_FLGS_QOVERFLOW;
			kxp74_private->readings_readindex++;	/* move read index along	*/
			if( kxp74_private->readings_readindex >= kxp74_private->readings_size){
				kxp74_private->readings_readindex = 0; /* wrap around	*/
			}
		}
		else{
			kxp74_private->readings_count++;	
		}
		
		kxp74_private->readings_writeindex++;
		if( kxp74_private->readings_writeindex >= kxp74_private->readings_size){
			kxp74_private->readings_writeindex = 0; /* wrap around	*/
		}
	}
	spin_unlock( &kxp74_private->lock );
	
	wake_up_interruptible( &kxp74_private->wait_queue );	/* kick any sleeping threads	*/
}

/*-----------------------------------------------------------------------------*/
/*
 * Function to copy the data from the store to user land. 
 * The store can be asymetrical (i.e. more recent data at lower indices ) in case
 * of an overflow situation.
 * Return a 'read()' char operation style return value.
 */
static int kxp74_copy_elements_to_user( struct gacc_driver_private_data* private, char __user *buf, size_t count ){
	int elements2copy;
	int result;
	
	spin_lock( &private->lock );
	elements2copy = min( count/ sizeof(accelerometer_reading_t), private->readings_count);
	
	if(!access_ok(VERIFY_WRITE, buf, elements2copy*sizeof(accelerometer_reading_t) ) ){
		spin_unlock( &private->lock );
		return -EFAULT;
	} 
	
	if( private->readings_readindex  < private->readings_writeindex ){
		result = __copy_to_user( buf, &private->head[private->readings_readindex], elements2copy*sizeof(accelerometer_reading_t) );
		if( result )
			result = -EFAULT;
		else{
			result = elements2copy*sizeof(accelerometer_reading_t);
			private->readings_readindex += elements2copy;
			if( private->readings_writeindex  ==  private->readings_readindex ){
				private->readings_writeindex = private->readings_readindex = 0;	
			}
			private->readings_count -=  elements2copy; 
		}
	}
	else{
		/* two chunks	*/
		int user_offset;
		int elements_from_this_chunk = min( (int)((private->readings_size)-(private->readings_readindex)), elements2copy );
		result = __copy_to_user( buf, &private->head[private->readings_readindex], elements_from_this_chunk*sizeof(accelerometer_reading_t) );
		if( result )
			result = -EFAULT;
		else{
			result = user_offset = elements_from_this_chunk*sizeof(accelerometer_reading_t) ;
			private->readings_count -= elements_from_this_chunk;
			private->readings_readindex += elements_from_this_chunk;
			if( private->readings_readindex >= private->readings_size )
				private->readings_readindex = 0;	/* wrap around	*/
			if( elements_from_this_chunk < elements2copy ){
				/* there's more, do second chunk	*/	
				elements_from_this_chunk = elements2copy - elements_from_this_chunk;
				result = __copy_to_user( (char __user *)(buf+user_offset), &private->head[0], elements_from_this_chunk*sizeof(accelerometer_reading_t));
				if( result )
					result = -EFAULT;
				else{
					private->readings_readindex += elements_from_this_chunk; /* advance read pointer as well */
					result = elements2copy*sizeof(accelerometer_reading_t); /* we successfuly copied everything	*/
					if( private->readings_writeindex  ==  private->readings_readindex ){
						private->readings_writeindex = private->readings_readindex = 0;	
					}
					private->readings_count -= elements_from_this_chunk;
				}
			}
		}
	}	
	spin_unlock( &private->lock );
	return result;
}

/*-----------------------------------------------------------------------------
 * 
 * PART 2 : Implementation of the generic accelerometer character device interface
 * 
 *-----------------------------------------------------------------------------
 */
static ssize_t kxp74_read( struct file *file, char __user *buf, size_t count, loff_t * loff) {
	int result;
	struct gacc_driver_private_data* kxp74_private = file->private_data;
	
	/* We don't need to lock at this point yet	*/
	if( kxp74_private->readings_count <= 0 ){
		if( file->f_flags & O_NONBLOCK ) {
			return -EAGAIN;
		}
		else{
			while( kxp74_private->readings_count <= 0 ){
				DEFINE_WAIT( wait );
				prepare_to_wait( &kxp74_private->wait_queue, &wait, TASK_INTERRUPTIBLE);
				if( kxp74_private->readings_count <= 0 )
					schedule();
				finish_wait( &kxp74_private->wait_queue, &wait );
				if( signal_pending( current ))
					return -ERESTARTSYS;
			}
		}
	}
	/* assume there is data available beyond this point	*/
	result = -EINVAL;
	
	if( (count/ sizeof(accelerometer_reading_t)) > 0 ){
		result = kxp74_copy_elements_to_user( kxp74_private, buf, count );
	}
	
	return result;
}


/*-----------------------------------------------------------------------------*/

static unsigned kxp74_poll(struct file *file, poll_table *wait)
{
	struct gacc_driver_private_data* kxp74_private = file->private_data;
	unsigned int mask = 0;
	/*
	* The buffer is circular; it is considered full
	* if "wp" is right behind "rp" and empty if the
	* two are equal.
	*/
	spin_lock( &kxp74_private->lock );
	poll_wait(file, &kxp74_private->wait_queue, wait);
	if ( kxp74_private->readings_count > 0 )
		mask |= POLLIN | POLLRDNORM; /* readable */

	spin_unlock( &kxp74_private->lock );
	return mask;
}

/*-----------------------------------------------------------------------------*/

static int kxp74_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) { 
	int result;
	struct gacc_driver_private_data* kxp74_private = file->private_data;
	
	switch( cmd ){
//		case GACC_IOCTL_TEST:
//			{
//				accelerometer_reading_t r;
//				printk("Reading...");
//				//result = kxp74_write_control_register( kxp74_private, 0x1c );
//				//printk("result = %d, 0x%02x\n", result, result);
//				result = kxp74_get_reading( kxp74_private, &r );
//				printk("result = %d, 0x%02x\n", result, result);
//				result = 0;
//			}
//			break;
		case GACC_IOCTL_RESET:
			/* drop all data in store	*/
			spin_lock_bh( &kxp74_private->lock );
			kxp74_private->readings_count = 0;
			kxp74_private->readings_writeindex = 0;
			spin_unlock_bh( &kxp74_private->lock );
			result = 0;	/* This stupid thing doesn't have a reset functionality so it cannot fail either then	*/
			break;
		case GACC_IOCTL_BIST:
		    result = -ENOSYS;
			break;
		case GACC_IOCTL_SET_SAMPLE_RATE:
			if( (arg > 0) && (arg <= GACC_MAX_SAMPLERATE) ){
				spin_lock_bh( &kxp74_private->lock );
				kxp74_private->settings->sample_rate = arg;
				spin_unlock_bh( &kxp74_private->lock );
				result = 0;
			}
			else{
				result = -EINVAL;	
			}
			break;
		case GACC_IOCTL_GET_SETTINGS:
			/* Check user buffer sanity */
			if ( !access_ok(VERIFY_WRITE, arg, sizeof(accelerometer_settings_t) ) ){
				result = -EFAULT;
			}
			else{	/* no need to lock settings, we're only reading	*/
				if( __copy_to_user( (char __user *)arg, kxp74_private->settings, sizeof(accelerometer_settings_t) ) ){
					result = -EFAULT;	
				}
				else{
					result = 0;
				}
			}
			break;
		default:
			result = -ENOTTY;	/* posix compliant response	*/
	}
	return result;
}

/*-----------------------------------------------------------------------------*/

static int kxp74_open( struct inode *inode, struct file *file ) {

	struct gadc_cdev		*adc_cdev=get_gadc_cdev( inode->i_cdev );
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) container_of( adc_cdev, struct kxp74_cdev, kxp74_dev );
	struct spi_device		*spi_dev=chardev->spi_dev;
	struct gacc_driver_private_data	*private;

	if( adc_cdev->private )
		return -EBUSY;	

	private = (struct gacc_driver_private_data *) kmalloc( sizeof(struct gacc_driver_private_data), GFP_KERNEL );
	if(! private )
	{
		printk( KXP74_DRIVER_NAME": Could not alloc private data!\n");
		return -ENOMEM;	
	}

	adc_cdev->private = private;
	/* Init private struct	*/
	private->settings = &kxp74_mode_settings;
	private->settings->sample_rate = GACC_DEFAULT_SAMPLERATE;
	private->spi_dev = spi_dev;
	/* Setup fifo for storing readings	*/
	private->readings_writeindex = private->readings_readindex = 0;
	private->readings_count = 0;
#ifdef GACC_DEBUG
	private->seq = 0;	/* reset sequence counter	*/
#endif	/* GACC_DEBUG	*/
 
#ifdef SW_TEST
	/* allocate an element before and after the buffer. We will fill it with a recognizable value
	 * so that we can check if someone has overwritten it.
	 */
	private->head = (accelerometer_reading_t*) kmalloc( (GACC_READING_QUEUE_SIZE+2) * sizeof(accelerometer_reading_t), GFP_KERNEL );
#else
	private->head = (accelerometer_reading_t*) kmalloc( GACC_READING_QUEUE_SIZE * sizeof(accelerometer_reading_t), GFP_KERNEL );
#endif /*	SW_TEST	*/
	if( ! private->head )
	{
		chardev->kxp74_dev.private=NULL;
		kfree( private );
		return -ENOMEM;	
	}
	private->readings_size = GACC_READING_QUEUE_SIZE;
	spin_lock_init( &private->lock );
	
	/* Setup SPI transfer buffers	*/
	private->private=kmalloc( sizeof( kxp74_private_t ), GFP_KERNEL );	/* KXP74 private data, not GACC	*/
	if( private->private != NULL )
	{
		((kxp74_private_t*)(private->private))->rx_buf = kmalloc( KXP74_BUF_SIZE, GFP_KERNEL | __GFP_DMA );
		((kxp74_private_t*)(private->private))->tx_buf = kmalloc( KXP74_BUF_SIZE, GFP_KERNEL | __GFP_DMA );

		if( !( ((kxp74_private_t*)(private->private))->rx_buf && ((kxp74_private_t*)(private->private))->tx_buf) )
		{
			if( ((kxp74_private_t*)(private->private))->rx_buf)
				kfree( ((kxp74_private_t*)(private->private))->rx_buf);
			if( ((kxp74_private_t*)(private->private))->tx_buf)
				kfree( ((kxp74_private_t*)(private->private))->tx_buf );
			adc_cdev->private=NULL;
			kfree( private->private );
			kfree( private->head );
			kfree( private );
			return -ENOMEM;	
		}
	}
	else
	{
		adc_cdev->private=NULL;
		kfree( private->head );
		kfree( private );
		return -ENOMEM;
	}

	/* Setup kernel timer	*/
	init_timer( &private->timer );
	private->timer.function  = kxp74_timer_handler;
	private->timer.data = (unsigned long) private;	/* argument to timer handler is private struct	*/
	private->work_queue = create_singlethread_workqueue(KXP74_DRIVER_NAME);
	private->flags = 0;
	/* setup work/wait queues	*/
	INIT_WORK( &private->work, kxp74_do_work, private );
	init_waitqueue_head( &private->wait_queue );
	
	/* store private data in struct file	*/
	file->private_data = private;
	
	/* start up the kxp inner circuits	*/
	if( chardev->kxr94_adc_dev.private == NULL )
	{
		/* init hardware	*/
		kxp74_init_hw( private->spi_dev );
		kxp74_write_control_register( private, (chardev->is_kxr94 ? KXR94_MODE_ON : KXP74_MODE_ON) ); 
	}
	
	/* start our self-rescheduling timer */
	kxp74_timer_handler( (unsigned long) private ); /* simply call it, it will reschedule itself	*/
#ifdef SW_TEST
	private->head++;	/* we reserved space for one extra element at the start for testing	*/
	test_fill_testbuffers( private );
#endif /*	SW_TEST	*/
	
	return 0;
}

/*-----------------------------------------------------------------------------*/
static int kxp74_release(struct inode *inode, struct file *file ) {
	
	struct gadc_cdev		*adc_cdev=get_gadc_cdev( inode->i_cdev );
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) container_of( adc_cdev, struct kxp74_cdev, kxp74_dev );
	struct gacc_driver_private_data* kxp74_private = file->private_data;

	if( kxp74_private ){
		kxp74_private->flags |= GACC_FLGS_EXIT;
		del_timer( &kxp74_private->timer );	/* remove any scheduled timer	*/
		
		flush_workqueue( kxp74_private->work_queue );	/* make sure nothin' is running anymore	*/
		destroy_workqueue( kxp74_private->work_queue );
		
		if( chardev->kxr94_adc_dev.private == NULL )
		{
			/* shut down the kxp inner circuits	*/
			kxp74_write_control_register( kxp74_private, (chardev->is_kxr94 ? KXR94_MODE_OFF : KXP74_MODE_OFF) ); 
		
			/* power off most stuff to save power	*/
			kxp74_exit_hw( kxp74_private->spi_dev );
		}
		
#ifdef SW_TEST
		kxp74_private->head--;	/* we added one element extra at the beginning	*/
#endif /* SW_TEST */	
		kfree( kxp74_private->head );
		kfree( kxp74_private->private );
#ifdef SW_TEST
		/* clear it so that any use after the kfree() will at least trigger a segmentation fault	*/
		memset( kxp74_private, 0, sizeof(*kxp74_private) ); 
#endif /* SW_TEST */
		kfree( kxp74_private );	
		chardev->kxp74_dev.private = NULL;
		
		file->private_data = NULL;
	}
	return 0;
}

/*-----------------------------------------------------------------------------*/

static struct file_operations kxp74_fops = {
	.owner 		= THIS_MODULE,
	.read		= kxp74_read,
	.poll		= kxp74_poll,
	
	.ioctl		= kxp74_ioctl,
	.open		= kxp74_open,
	.release	= kxp74_release,
};

int kxp74_remove( struct spi_device *spi )
{
	struct kxp74_cdev	*chardev=(struct kxp74_cdev *) spi->dev.driver_data;	

	unregister_gadc_cdev( &chardev->kxp74_dev );
	if( chardev->is_kxr94 )
		unregister_gadc_cdev( &chardev->kxr94_adc_dev );
	kfree( chardev );
	
	return 0;
}
EXPORT_SYMBOL( kxp74_remove );

/*-----------------------------------------------------------------------------*/

#ifdef CONFIG_PM
int kxp74_suspend( struct spi_device *spi, pm_message_t mesg )
{
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) spi->dev.driver_data;
	struct kxr94_adc_private	*kxr94_priv=(struct kxr94_adc_private *) chardev->kxr94_adc_dev.private;
	struct gacc_driver_private_data	*kxp74_priv=(struct gacc_driver_private_data *) chardev->kxp74_dev.private;

	if( kxr94_priv || kxp74_priv )
	{
		if( kxp74_priv )
		{
			kxp74_priv->flags |= GACC_FLGS_EXIT;
			del_timer( &kxp74_priv->timer );	/* remove any scheduled timer	*/
			flush_workqueue( kxp74_priv->work_queue );	/* make sure nothin' is running anymore	*/
		}

		if( kxr94_priv )
		{
			kxr94_priv->flags |= GACC_FLGS_EXIT;
			del_timer( &kxr94_priv->timer );
			flush_workqueue( kxr94_priv->work_queue );
		}

		/* shut down the kxp inner circuits	*/
		kxr94_spi_write( spi, KXP74_ADDR_WRITE_CTRL, (chardev->is_kxr94 ? KXR94_MODE_OFF : KXP74_MODE_OFF) );
		kxp74_exit_hw( spi );
	}
	return 0;
}
EXPORT_SYMBOL( kxp74_suspend );

int kxp74_resume( struct spi_device *spi )
{
	struct kxp74_cdev		*chardev=(struct kxp74_cdev *) spi->dev.driver_data;
	struct kxr94_adc_private	*kxr94_priv=(struct kxr94_adc_private *) chardev->kxr94_adc_dev.private;
	struct gacc_driver_private_data	*kxp74_priv=(struct gacc_driver_private_data *) chardev->kxp74_dev.private;

	if( kxr94_priv || kxp74_priv )
	{
		kxp74_init_hw( spi );

		/* start up the kxp inner circuits	*/
		kxr94_spi_write( spi, KXP74_ADDR_WRITE_CTRL, (chardev->is_kxr94 ? KXR94_MODE_ON : 0x1C) );
		
		if( kxr94_priv )
		{
			gadc_clear_buffer( kxr94_priv->adc );
			kxr94_priv->flags &=~GACC_FLGS_EXIT;

			kxr94_adc_timer_handler( (unsigned long) kxr94_priv );
		}

		if( kxp74_priv )
		{
			kxp74_priv->readings_count = 0;
			kxp74_priv->readings_writeindex = 0;
			kxp74_priv->flags &= ~GACC_FLGS_EXIT;

			/* start our self-rescheduling timer */
			kxp74_timer_handler( (unsigned long) kxp74_priv );
		}
	}

	return 0;
}
EXPORT_SYMBOL( kxp74_resume );
#endif /* CONFIG_PM */
/*-----------------------------------------------------------------------------*/
static struct spi_driver kxp74_driver;

int kxp74_probe( struct spi_device *spi )
{
	struct gacc_platform_data	*pdata = spi->dev.platform_data;
	struct gadc_platform_data	*pdat_adc = (pdata != NULL ? ((struct gadc_platform_data *) pdata->adc) : NULL);
	char				*devicename;
	struct kxp74_cdev		*chardev=NULL;

	if( (spi != NULL) && (spi->dev.driver != NULL) )
	{
		chardev=(struct kxp74_cdev *) kmalloc( sizeof( *chardev ), GFP_KERNEL );
		if( chardev == NULL )
		{
			printk( KXP74_DRIVER_NAME ":Could not allocate platform data!\n" );	
			return -ENODEV;
		}

		devicename=(char *) spi->dev.driver->name;
		if( to_spi_driver( spi->dev.driver ) == &kxp74_driver )
			chardev->is_kxr94=0;
		else
			chardev->is_kxr94=1;
	}
	else
	{
		PK_DBG_FUNC( KXP74_DRIVER_NAME": Missing device driver info!\n" );
		return -ENODEV;
	}

	if( ! pdata )
	{
		PK_DBG_FUNC( KXP74_DRIVER_NAME": Missing platform data!\n" );
		goto kxp74_probe_register_chrregion_error;
	}

	
	printk( "TomTom GO KXP74 SPI accelerometer, (C) 2006 TomTom BV\n"); 	
	if( register_gadc_cdev( &chardev->kxp74_dev, MAJOR( pdata->device_nr ), 0, 1, KXP74_DRIVER_NAME, &kxp74_fops ) )
	{
		printk( KXP74_DRIVER_NAME": Could not add cdev !\n");	
		goto kxp74_probe_cdev_add;
	}

	if( pdat_adc != NULL )
	{
		if( register_gadc_cdev( &chardev->kxr94_adc_dev, pdat_adc->major, pdat_adc->minor, 1, KXR94_DRIVER_NAME, &kxr94_adc_fops ) ) 
		{
			printk( KXR94_DRIVER_NAME": Could not add ADC cdev !\n" );
			goto kxp74_probe_adc_cdev_add;
		}
		printk( "Found KXR94 SPI ADC controller. Adding at major: %u, minor: %u\n",
			chardev->kxr94_adc_dev.major, chardev->kxr94_adc_dev.minor );
	}
	
	spi->bits_per_word = 8;
	spi->irq = -1;
	printk( "TomTom GO KXP74 SPI accelerometer: initialized %s.\n", devicename ); 	
	chardev->spi_dev = spi; /* Store it for use in the cdev open()	*/
	spi->dev.driver_data=chardev;
	chardev->kxp74_dev.private=NULL;
	chardev->kxr94_adc_dev.private=NULL;
	
	return 0;
	
kxp74_probe_adc_cdev_add:
	unregister_gadc_cdev( &chardev->kxp74_dev );
	
kxp74_probe_cdev_add:
kxp74_probe_register_chrregion_error:
	kfree( chardev );
	
	return -ENODEV;
}
EXPORT_SYMBOL( kxp74_probe );


/*-----------------------------------------------------------------------------*/

static struct spi_driver kxp74_driver = {
	
    .driver = {
        .name   = KXP74_DRIVER_NAME,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .probe          = kxp74_probe,
    .remove         = kxp74_remove,
    .suspend        = kxp74_suspend,
    .resume         = kxp74_resume,
};

static int __init kxp74_init(void) {
	int result;
	result = spi_register_driver(&kxp74_driver);
	if( result ) return result;
	result = spi_register_driver(&kxr94_driver);
	return result;
}

module_init(kxp74_init);

/*-----------------------------------------------------------------------------*/

static void __exit kxp74_exit(void){
	
	spi_unregister_driver(&kxp74_driver);
}
module_exit(kxp74_exit);

/*-----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("KXP74 Accelerometer Driver");
MODULE_LICENSE("GPL");

/* EOF	*/
