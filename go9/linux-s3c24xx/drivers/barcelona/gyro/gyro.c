/* drivers/barcelona/gyro/gyro.c
 *
 * Implementation of the Gyro driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <Mark.Vels@tomtom.com>
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
#include <linux/string.h>
#include <linux/delay.h>

#include <barcelona/gyro.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/debug.h>


#if defined(CONFIG_BARCELONA_GYRO_ING300) && defined(CONFIG_BARCELONA_GYRO_FUJITSU)
#error Fujitus Gyro and InvenSense Gyro cannot be configured at the same time!
#endif 
#if defined(CONFIG_BARCELONA_GYRO_ING300_MODULE) && defined(CONFIG_BARCELONA_GYRO_FUJITSU_MODULE)
#error Fujitus Gyro and InvenSense Gyro cannot be configured at the same time!
#endif

#define PFX "gyro:"
#define PK_DBG PK_DBG_FUNC

#define HERE()	{ printk("%s line %d says: HERE!\n", __FILE__, __LINE__ ); }
#define TELL(X)	{ printk("%s line %d says: %s = %d ( 0x%02x) \n", __FILE__, __LINE__, #X, (int) (X), (int) (X)); }

#define GYRO_DRIVER_NAME		"gyro"


/**
 * Explanation:
 * You can use the gyro driver in 3 modes:
 * -1- "real" mode. The timer is used to determine the moments when to sample the latest adc value and put it in the fifo.
 * -2- "GYRO_MAX_RATE_CALCULATE_AVERAGE mode stores SAMPLE_AVERAGE_SIZE samples in a buffer from the adc_poll function. 
 *      If the count is reached, the average is calculated and put into the fifo.
 * -3- "GYRO_MAX_RATE_FULL_DATA" mode stores all the samples in a buffer from the adc_poll function. After a sample count of 
 *	FULL_DATA_RATE_BUFFER_COUNT, all the samples will be put in the fifo. This will result in a real sample rate of
 *	ADC_RATE ( likely to be set to 100 ) so a lot of data is generated.
*/

#define GYRO_MAX_RATE_CALCULATE_AVERAGES __FILE__
//#define GYRO_MAX_RATE_FULL_DATA __FILE__


/* SANITY CHECK */
#if defined(GYRO_MAX_RATE_FULL_DATA) && defined(GYRO_MAX_RATE_CALCULATE_AVERAGES)
#error config error: GYRO_MAX_RATE_CALCULATE_AVERAGE and GYRO_MAX_RATE_FULL_DATA cannot be set at the same time!
#endif /* GYRO_MAX_RATE_FULL_DATA || GYRO_MAX_RATE_CALCULATE_AVERAGES */



#if defined(GYRO_MAX_RATE_CALCULATE_AVERAGES)
/*
	In this mode, we will collect samples from the adc_pollfunc_t implementation in the gyro driver.
	Once we have collected AVERAGE_SIZE samples, we will calculate the result.
	This result will be put in the queue.

	We will NOT use the kernel timer anymore and the sample rate will be fixed. 
	The IOCTL.s for setting sample rate are disabled, the GET_SAMPLE_RATE must return the fixed value.

*/

#define SAMPLE_AVERAGE_SIZE          (8) /* Must be a power of 2 */
#define SAMPLE_AVERAGE_SIZE_2LOG     (3) /* 8 = 2 ^ 3  */

/* average_buf_count has to be global so that we can reset it	*/
static int average_buf_count;

#elif defined(GYRO_MAX_RATE_FULL_DATA)

/* Dump buffer after this number of samples */
#define FULL_DATA_RATE_BUFFER_COUNT	(8)
#define FULL_SIZE_BUFFER_SIZE		(32)
static int buf_count;

static short samples_buffer_0[FULL_SIZE_BUFFER_SIZE];
static short samples_buffer_1[FULL_SIZE_BUFFER_SIZE];
static struct timeval  samples_buffer_ts[FULL_SIZE_BUFFER_SIZE];

#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES */

/*-----------------------------------------------------------------------------*/
/* Global variables															   */
/*-----------------------------------------------------------------------------*/

static gyro_settings_t	gyro_mode_settings = {
		.sensitivity	= 819,	/* typical datasheet rating		*/
		.range		= 0x07ff,
};

static struct cdev			*gyro_cdev;
static int					gyro_busy;
/* Here we store all the private data structs for each channel	*/
static struct gyro_driver_private_data* gyro_private_list[ADC_CHANNELS];

static gopin_t	gyro_enable_pin;
static int 		gyro_adc_channel[2];

/*-----------------------------------------------------------------------------
 * 
 * PART 1 : Implementation of the gyro local functions and ADC plug-in.
 * 
 *-----------------------------------------------------------------------------
 */


static int gyro_init_hw( void ){
	
	IO_SetFunction( DR_GYRO_HPS );
	
#if defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
	IOP_SetInput( gyro_enable_pin );
	IO_Activate( DR_GYRO_HPS );
	msleep( 200 );
	IO_Deactivate( DR_GYRO_HPS );
	
#endif 

#if defined(CONFIG_BARCELONA_GYRO_FUJITSU) || defined(CONFIG_BARCELONA_GYRO_FUJITSU_MODULE)
	IO_Deactivate( DR_GYRO_HPS );
	
	IOP_SetFunction( gyro_enable_pin );
	IOP_Deactivate( gyro_enable_pin );
#endif

	return 0;
}
 
/*-----------------------------------------------------------------------------*/

static int gyro_exit_hw( void ){
	
#if defined(CONFIG_BARCELONA_GYRO_FUJITSU) || defined(CONFIG_BARCELONA_GYRO_FUJITSU_MODULE)
	IOP_Deactivate( gyro_enable_pin );
	IOP_SetInput( gyro_enable_pin );
#endif

	return 0;
}

/*-----------------------------------------------------------------------------*/

#if defined(GYRO_MAX_RATE_CALCULATE_AVERAGES)

/*
 * Calculate the average in this buffer. 
 * It would have been nicer if the size of samples was dynamic but that
 * requires extra calculation and since this function is called from 
 * interrupt context, we retain from it.
*/
inline short gyro_calculate_average( short samples[SAMPLE_AVERAGE_SIZE] ){

	int i;
	int carry = 0;
	int sum   = 0;          // ADC samples are 16 bit so the sum will fit easily in an int
    
	for( i = 0; i < SAMPLE_AVERAGE_SIZE; i++)
		sum += samples[i];

	if( sum & (1<< (SAMPLE_AVERAGE_SIZE_2LOG-1)) ) 
		carry = 1; /* If we shift out bit 2 then we must add one to have a fair rounding policy     */

	return ( sum >> SAMPLE_AVERAGE_SIZE_2LOG) + carry;
}

#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES */


/*-----------------------------------------------------------------------------*/

/**
 * ADC call back. Is run from interrupt context!
 */

static void gyro_adc_poll(short adcBuffer[ADC_CHANNELS], void* arg){
	
#if defined(GYRO_MAX_RATE_FULL_DATA)

	struct gyro_driver_private_data* gyro_private = (struct gyro_driver_private_data*) arg;

	samples_buffer_0[ buf_count ] = adcBuffer[ gyro_private->adc_channel[0] ];
	samples_buffer_1[ buf_count ] = adcBuffer[ gyro_private->adc_channel[1] ];
	do_gettimeofday( &samples_buffer_ts[buf_count] );
	buf_count++;

	/* Check if is time to store the collected samples */
	if( buf_count >= FULL_DATA_RATE_BUFFER_COUNT ){
 		if( ! ( gyro_private->flags & GYRO_FLGS_EXIT ) ){ /* ignore the values if exit flag is set  */
			if( queue_work( gyro_private->work_queue, &gyro_private->work ) ){
				/* There obviously is congestion if we try to schedule work
				 * while the previous scheduled one is not finished yet */
				gyro_private->flags|= GYRO_FLGS_CONGESTION;
			}
		}
	}


#elif defined(GYRO_MAX_RATE_CALCULATE_AVERAGES)
	static short buffer_0[ SAMPLE_AVERAGE_SIZE ];
	static short buffer_1[ SAMPLE_AVERAGE_SIZE ];

	struct gyro_driver_private_data* gyro_private = (struct gyro_driver_private_data*) arg;

	buffer_0[ average_buf_count ] = adcBuffer[ gyro_private->adc_channel[0] ];
	buffer_1[ average_buf_count ] = adcBuffer[ gyro_private->adc_channel[1] ];

	average_buf_count++;
	if( average_buf_count>= SAMPLE_AVERAGE_SIZE){

 		if( ! ( gyro_private->flags & GYRO_FLGS_EXIT ) ){ /* ignore the values if exit flag is set  */
			gyro_private->last_value[0] = gyro_calculate_average( buffer_0 );
#if defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
			gyro_private->last_value[1] = gyro_calculate_average( buffer_1 );
#endif
			do_gettimeofday( &gyro_private->last_timestamp );


                	if( queue_work( gyro_private->work_queue, &gyro_private->work ) ){
                        	/* There obviously is congestion if we try to schedule work
                         	* while the previous scheduled one is not finished yet */
                         	gyro_private->flags|= GYRO_FLGS_CONGESTION;
                	}
		}
		average_buf_count = 0;
	}


#else /* GYRO_MAX_RATE_FULL_DATA / GYRO_MAX_RATE_CALCULATE_AVERAGES */

	struct gyro_driver_private_data* gyro_private = (struct gyro_driver_private_data*) arg;
	
	gyro_private->last_value[0] = adcBuffer[ gyro_private->adc_channel[0] ];
#if defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
	gyro_private->last_value[1] = adcBuffer[ gyro_private->adc_channel[1] ];
#endif
	do_gettimeofday( &gyro_private->last_timestamp );

#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES */
}

/*-----------------------------------------------------------------------------*/
/**
 * Kernel timer handler function.
 * It is also called from the adc poll function when in FIX_TO_MAX_SAMPLERATE mode.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
#if ! (defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
static void gyro_timer_handler( unsigned long  arg ){
	
	unsigned short sample_rate;
	struct gyro_driver_private_data* gyro_private = (struct gyro_driver_private_data*) arg;
	
	if( ! ( gyro_private->flags & GYRO_FLGS_EXIT ) ){ /* if we must exit then return and do not reschedule	*/
		
		if( queue_work( gyro_private->work_queue, &gyro_private->work ) ){
			/* There obviously is congestion if we try to schedule work
			 * while the previous scheduled one is not finished yet	*/	
			 gyro_private->flags|= GYRO_FLGS_CONGESTION;
		}

		sample_rate = gyro_private->settings->sample_rate;
		gyro_private->timer.expires = jiffies + ( HZ +(sample_rate>>1))/(sample_rate);
		add_timer( &gyro_private->timer );	
	}
}
#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES || GYRO_MAX_RATE_FULL_DATA */
/*-----------------------------------------------------------------------------*/

static void gyro_do_work( void* arg ){
	unsigned long flags;
	struct gyro_driver_private_data* gyro_private = (struct gyro_driver_private_data*) arg;

	/* Sync with irq handler and user level threads ( adc callback is run from interrupt context)	*/
	spin_lock_irqsave( &gyro_private->lock, flags );
	{
#if defined(GYRO_MAX_RATE_FULL_DATA)
		//TODO: This is an ugly construction, double fifo handling code here :-(. Write a function for that!

		int i;
		for( i = 0; i < buf_count ; i++ ){
#if defined(CONFIG_BARCELONA_GYRO_FUJITSU) || defined(CONFIG_BARCELONA_GYRO_FUJITSU_MODULE)
			gyro_private->head[gyro_private->readings_writeindex].value = samples_buffer_0[ i ];
#else
			gyro_private->head[gyro_private->readings_writeindex].value = 0;
#endif
#if defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
			gyro_private->head[gyro_private->readings_writeindex].x_value = samples_buffer_0[ i ];
			gyro_private->head[gyro_private->readings_writeindex].y_value = samples_buffer_1[ i ];
#else
			gyro_private->head[gyro_private->readings_writeindex].x_value = 0;
			gyro_private->head[gyro_private->readings_writeindex].y_value = 0;
#endif
			memcpy( &gyro_private->head[gyro_private->readings_writeindex].timestamp, &samples_buffer_ts[ i ], sizeof( struct timeval ) );

			if( (gyro_private->readings_writeindex==gyro_private->readings_readindex) && (gyro_private->readings_count > 0 ) ){
				/* We're overwriting something! */
				gyro_private->flags |= GYRO_FLGS_QOVERFLOW;
				gyro_private->readings_readindex++;     /* move read index along        */
				if( gyro_private->readings_readindex >= gyro_private->readings_size){
					gyro_private->readings_readindex = 0; /* wrap around    */
				}
			}
			else{
				gyro_private->readings_count++;
			}
			gyro_private->readings_writeindex++;
			if( gyro_private->readings_writeindex >= gyro_private->readings_size){
				gyro_private->readings_writeindex = 0; /* wrap around   */
			}
		}
		buf_count = 0; /* emtpied */

#else /* GYRO_MAX_RATE_FULL_DATA */
#if defined(CONFIG_BARCELONA_GYRO_FUJITSU) || defined(CONFIG_BARCELONA_GYRO_FUJITSU_MODULE)
		gyro_private->head[gyro_private->readings_writeindex].value = gyro_private->last_value[0];
#else
		gyro_private->head[gyro_private->readings_writeindex].value = 0;
#endif
#if defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
		gyro_private->head[gyro_private->readings_writeindex].x_value = gyro_private->last_value[0];
		gyro_private->head[gyro_private->readings_writeindex].y_value = gyro_private->last_value[1];
#else
		gyro_private->head[gyro_private->readings_writeindex].x_value = 0;
		gyro_private->head[gyro_private->readings_writeindex].y_value = 0;
#endif
		memcpy( &gyro_private->head[gyro_private->readings_writeindex].timestamp, &gyro_private->last_timestamp, sizeof( struct timeval ) );
#ifdef GYRO_DEBUG
		gyro_private->head[gyro_private->readings_writeindex].seq = gyro_private->seq++;	/* sample equence counter	*/
#endif /* GYRO_DEBUG	*/

		if( (gyro_private->readings_writeindex==gyro_private->readings_readindex) && (gyro_private->readings_count > 0 ) ){
			/* We're overwriting something!	*/
			gyro_private->flags |= GYRO_FLGS_QOVERFLOW;
			gyro_private->readings_readindex++;	/* move read index along	*/
			if( gyro_private->readings_readindex >= gyro_private->readings_size){
				gyro_private->readings_readindex = 0; /* wrap around	*/
			}
		}
		else{
			gyro_private->readings_count++;	
		}
		gyro_private->readings_writeindex++;
		if( gyro_private->readings_writeindex >= gyro_private->readings_size){
			gyro_private->readings_writeindex = 0; /* wrap around	*/
		}
#endif /* GYRO_MAX_RATE_FULL_DATA */
	}
	spin_unlock_irqrestore( &gyro_private->lock, flags );
	
	wake_up_interruptible( &gyro_private->wait_queue );	/* kick any sleeping threads	*/
		
}

/*-----------------------------------------------------------------------------*/
/*
 * Function to copy the data from the store to user land. 
 * The store can be asymetrical (i.e. more recent data at lower indices ) in case
 * of an overflow situation.
 * Return a 'read()' char operation style return value.
 */
static int gyro_copy_elements_to_user( struct gyro_driver_private_data* private, char __user *buf, size_t count ){
	int elements2copy;
	int result;
	int ret;
	
	spin_lock( &private->lock );
	elements2copy = min( count/ sizeof(gyro_reading_t), private->readings_count);
	
	if(!access_ok(VERIFY_WRITE, buf, elements2copy*sizeof(gyro_reading_t) ) ){
		spin_unlock( &private->lock );
		return -EFAULT;
	} 
	
	if( private->readings_readindex  < private->readings_writeindex ){
		result = __copy_to_user( buf, &private->head[private->readings_readindex], elements2copy*sizeof(gyro_reading_t) );
		if( result )
			ret = -EFAULT;
		else{
			ret = elements2copy*sizeof(gyro_reading_t);
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
		result = __copy_to_user( buf, &private->head[private->readings_readindex], elements_from_this_chunk*sizeof(gyro_reading_t) );
		if( result )
			ret = -EFAULT;
		else{
			ret = user_offset = elements_from_this_chunk*sizeof(gyro_reading_t) ;
			private->readings_count -= elements_from_this_chunk;
			private->readings_readindex += elements_from_this_chunk;
			if( private->readings_readindex >= private->readings_size )
				private->readings_readindex = 0;	/* wrap around	*/
				
			if( elements_from_this_chunk < elements2copy ){
				/* there's more, do second chunk	*/	
				elements_from_this_chunk = elements2copy - elements_from_this_chunk;
				result = __copy_to_user( (char __user *)(buf+user_offset), &private->head[0], elements_from_this_chunk*sizeof(gyro_reading_t));
				if( result )
					ret = -EFAULT;
				else{
					private->readings_readindex += elements_from_this_chunk; /* advance read pointer as well */
					ret = elements2copy*sizeof(gyro_reading_t); /* we successfuly copied everything	*/
					if( private->readings_writeindex  ==  private->readings_readindex ){
						private->readings_writeindex = private->readings_readindex = 0;	
					}
					private->readings_count -= elements_from_this_chunk;
				}
			}
		}
	}	
	spin_unlock( &private->lock );
	return ret;
}

/*-----------------------------------------------------------------------------
 * 
 * PART 2 : Implementation of the gyro character device interface
 * 
 *-----------------------------------------------------------------------------
 */
static ssize_t gyro_read( struct file *file, char __user *buf, size_t count, loff_t * loff) {
	int result;
	struct gyro_driver_private_data* gyro_private = file->private_data;
	
	/* We don't need to lock at this point yet	*/
	if( gyro_private->readings_count <= 0 ){
		if( file->f_flags & O_NONBLOCK ) {
			return -EAGAIN;
		}
		else{
			while( gyro_private->readings_count <= 0 ){
				DEFINE_WAIT( wait );
				prepare_to_wait( &gyro_private->wait_queue, &wait, TASK_INTERRUPTIBLE);
				if( gyro_private->readings_count <= 0 )
					schedule();
				finish_wait( &gyro_private->wait_queue, &wait );
				if( signal_pending( current ))
					return -ERESTARTSYS;
			}
		}
	}
	/* assume there is data available beyond this point	*/
	result = -EINVAL;
	
	if( (count/ sizeof(gyro_reading_t)) > 0 ){
		result = gyro_copy_elements_to_user( gyro_private, buf, count );
	}
	
	return result;
}

/*-----------------------------------------------------------------------------*/

static unsigned gyro_poll(struct file *file, poll_table *wait)
{
	struct gyro_driver_private_data* gyro_private = file->private_data;
	unsigned int mask = 0;
	/*
	* The buffer is circular; it is considered full
	* if "wp" is right behind "rp" and empty if the
	* two are equal.
	*/
	spin_lock( &gyro_private->lock );
	poll_wait(file, &gyro_private->wait_queue, wait);
	if ( gyro_private->readings_count > 0 )
		mask |= POLLIN | POLLRDNORM; /* readable */

	spin_unlock( &gyro_private->lock );
	return mask;
}


/*-----------------------------------------------------------------------------*/

static int gyro_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) { 
	int result;
	struct gyro_driver_private_data* gyro_private = file->private_data;
	
	switch( cmd ){
//		case GYRO_IOCTL_TEST:
//			{
//				gyro_reading_t r;
//				printk("Reading...");
//				//result = gyro_write_control_register( gyro_private, 0x1c );
//				//printk("result = %d, 0x%02x\n", result, result);
//				result = gyro_get_reading( gyro_private, &r );
//				printk("result = %d, 0x%02x\n", result, result);
//				result = 0;
//			}
//			break;
		case GYRO_IOCTL_RESET:
			/* drop all data in store	*/
#if defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA)
			/* clear the temp buffer */
			adc_unregister_poll( gyro_adc_poll ); /* is irq synchronized ...  */
#ifdef GYRO_MAX_RATE_FULL_DATA
			buf_count = 0;
#else
			average_buf_count = 0;
#endif /* GYRO_MAX_RATE_FULL_DATA */
			adc_register_poll(gyro_adc_poll, gyro_private, ADC_RATE_GYRO );
#endif

			spin_lock_bh( &gyro_private->lock );
			gyro_private->readings_count = 0;
			gyro_private->readings_writeindex = 0;
			spin_unlock_bh( &gyro_private->lock );
			result = 0;	
			break;
		case GYRO_IOCTL_BIST:
		    result = -ENOSYS;
			break;
#if !(defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
		case GYRO_IOCTL_SET_SAMPLE_RATE:
			if( (arg > 0) && (arg <= GYRO_MAX_SAMPLERATE) ){
				spin_lock_bh( &gyro_private->lock );
				gyro_private->settings->sample_rate = arg;
				spin_unlock_bh( &gyro_private->lock );
				result = 0;
			}
			else{
				result = -EINVAL;	
			}
			break;
#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES || GYRO_MAX_RATE_FULL_DATA */
		case GYRO_IOCTL_GET_SETTINGS:
			/* Check user buffer sanity */
			if ( !access_ok(VERIFY_WRITE, arg, sizeof(gyro_settings_t) ) ){
				result = -EFAULT;
			}
			else{	/* no need to lock settings, we're only reading	*/
				if( __copy_to_user( (char __user *)arg, gyro_private->settings, sizeof(gyro_settings_t) ) ){
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

static int gyro_open( struct inode *inode, struct file *file ) {
	
	int ret;
	struct gyro_driver_private_data* private;

	if( gyro_private_list[0] ){
		PK_ERR("Channel %d is already in use", 0 );
		return -EBUSY;	
	}
	
	private = kmalloc( sizeof(struct gyro_driver_private_data), GFP_KERNEL );
	if(! private ){
		PK_ERR("Could not alloc private data!\n");
		return -ENOMEM;	
	}
	memset( private, 0, sizeof(struct gyro_driver_private_data) ); /* init all members to 0 */
	private->channel = 0;	/* currently only one instance	*/
	gyro_private_list[private->channel] = private; /* store a pointer in global list */
	/* Init private struct	*/
	private->settings = &gyro_mode_settings;
#if defined(GYRO_MAX_RATE_FULL_DATA)
	private->settings->sample_rate = ADC_RATE;
	buf_count = 0;
#elif defined(GYRO_MAX_RATE_CALCULATE_AVERAGES)
	private->settings->sample_rate = ADC_RATE / SAMPLE_AVERAGE_SIZE;
#else
	private->settings->sample_rate = GYRO_DEFAULT_SAMPLERATE;
#endif /* GYRO_MAX_RATE_CALCULATE_AVERAGES */
	
	private->adc_channel[0] = gyro_adc_channel[0];	/* init from temp copy of platform data	*/
	private->adc_channel[1] = gyro_adc_channel[1];	/* init from temp copy of platform data	*/

	private->head = (gyro_reading_t*) kmalloc( GYRO_READING_QUEUE_SIZE * sizeof(gyro_reading_t), GFP_KERNEL );
	if( ! private->head ){
		kfree( private );
		PK_ERR("Could not alloc queue area!\n");
		return -ENOMEM;	
	}
	private->readings_size = GYRO_READING_QUEUE_SIZE;
	spin_lock_init( &private->lock );
	
#if !( defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
	/* Setup kernel timer	*/
	init_timer( &private->timer );
	private->timer.function  = gyro_timer_handler;
	private->timer.data = (unsigned long) private;	/* argument to timer handler is private struct	*/
#endif

	private->flags = 0;
	/* setup work/wait queues	*/
	private->work_queue = create_singlethread_workqueue(GYRO_DRIVER_NAME);
	INIT_WORK( &private->work, gyro_do_work, private );
	init_waitqueue_head( &private->wait_queue );
	
	/* store private data in struct file	*/
	file->private_data = private;
	/* init hardware	*/
	gyro_init_hw(  );
	
#ifdef GYRO_DEBUG
	private->seq = 0;	/* reset sequence counter	*/
#endif /* GYRO_DEBUG	*/
	
	PK_DBG("Registering ADC pollfunc\n");
	ret = adc_register_poll(gyro_adc_poll, private, ADC_RATE_GYRO );
	if (ret < 0) {
		kfree( private );
		PK_ERR("Unable to register ADC pollfunc (%d)\n", ret);
		return ret;
	}
	
#if ! (defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
	/* start our self-rescheduling timer */
	gyro_timer_handler( (unsigned long) private ); /* simply call it, it will reschedule itself	*/
#else
	/* The adc poll will start collecting now because the EXIT flag is cleared */
#endif
	
	return 0;
}

/*-----------------------------------------------------------------------------*/

static int gyro_release(struct inode *inode, struct file *file ) {
	
	int i;
	int last_one;
	int ret;
	
	struct gyro_driver_private_data* gyro_private = file->private_data;
	
	ret = adc_unregister_poll( gyro_adc_poll ); /* is irq synchronized ...	*/
	if( ret ){
		PK_ERR("Unable to deregister ADC pollfunc (%d)\n", ret);
	}
	if( gyro_private ){
		gyro_private->flags |= GYRO_FLGS_EXIT;
#if ! (defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
		del_timer( &gyro_private->timer );	/* remove any scheduled timer	*/
#endif
		
		flush_workqueue( gyro_private->work_queue );	/* make sure nothin' is running anymore	*/
		destroy_workqueue( gyro_private->work_queue );
			
		gyro_private_list[gyro_private->channel] = NULL;

		/* power off most stuff to save power	*/
		last_one = 1; /* assume this is last one until opposite is proven */
		for( i = 0; i < ARRAY_SIZE(gyro_private_list); i++) {
			if( gyro_private_list[i] )
				last_one = 0;
		}
		if( last_one)
			gyro_exit_hw(  );
		
		kfree( gyro_private->head );
		/* clear it so that any use after the kfree() will at least trigger a segmentation fault	*/
		memset( gyro_private, 0, sizeof(*gyro_private) ); 
		kfree( gyro_private );	
		
		file->private_data = NULL;
	}
	gyro_busy = 0;
	return 0;
}

/*-----------------------------------------------------------------------------*/

static struct file_operations gyro_fops = {
	.owner 		= THIS_MODULE,
	.read		= gyro_read,
	.poll		= gyro_poll,
	
	.ioctl		= gyro_ioctl,
	.open		= gyro_open,
	.release	= gyro_release,
};

/*-----------------------------------------------------------------------------
 * 
 * PART 3 : Implementation of the SPI driver interface
 * 
 *-----------------------------------------------------------------------------
 */
static int gyro_remove( struct device *dev ) {
	
	cdev_del( gyro_cdev );
	kfree( gyro_cdev );
	
	return 0;
}

/*-----------------------------------------------------------------------------*/

#ifdef CONFIG_PM
static int gyro_suspend( struct device *dev, u32 state, u32 level ){
	int i;	

	for(i =0 ; i < ARRAY_SIZE(gyro_private_list); i++){
		if( gyro_private_list[i] == NULL)
			continue;
		gyro_private_list[i]->flags |= GYRO_FLGS_EXIT;
#if ! (defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
		del_timer( &gyro_private_list[i]->timer );	/* remove any scheduled timer	*/
#endif
	}		
	gyro_exit_hw( );
	return 0;
}

/*-----------------------------------------------------------------------------*/

static int gyro_resume( struct device *dev, u32 level ) {
	int i;	
	
	/* Check if there is an active channel	*/
	for( i=0; i < ARRAY_SIZE(gyro_private_list); i++){
		if(gyro_private_list[i]){
			/* reset the fifo	*/
			gyro_private_list[i]->readings_count = 0;
			gyro_private_list[i]->readings_writeindex = 0;
			gyro_private_list[i]->flags &= ~GYRO_FLGS_EXIT;	/* clear the exit flag	*/
#if ! (defined(GYRO_MAX_RATE_CALCULATE_AVERAGES) || defined(GYRO_MAX_RATE_FULL_DATA) )
			gyro_timer_handler( (unsigned long) gyro_private_list[i] );
#endif
			gyro_init_hw( );
		}
	}	
			
	return 0;
}

#else
#define gyro_suspend	NULL
#define gyro_resume	NULL
#endif /* CONFIG_PM */
/*-----------------------------------------------------------------------------*/

static int gyro_probe( struct device *dev ){
	
	struct gyro_platform_data    *pdata = dev->platform_data;
	PK_INFO( "TomTom GO gyro driver, (C) 2007 TomTom BV\n"); 	
	if( ! pdata ){
		printk( GYRO_DRIVER_NAME": Missing platform data!\n");
		return -ENODEV;
	}
	
	gyro_cdev = cdev_alloc();
	if(! gyro_cdev){
		PK_ERR( GYRO_DRIVER_NAME": Could not allocate platform data!\n");	
		return -ENODEV;
	}
	
	if( register_chrdev_region( pdata->device_nr, ARRAY_SIZE(gyro_private_list), GYRO_DRIVER_NAME ) ){
		PK_ERR( GYRO_DRIVER_NAME": Could not register cdev major/minor range!\n");	
		goto gyro_probe_register_chrregion_error;
	}
	
	/* Ready for action?	*/
	if( cdev_add( gyro_cdev, pdata->device_nr, ARRAY_SIZE(gyro_private_list)) ){
		PK_ERR( GYRO_DRIVER_NAME": Could not add cdev !\n");	
		goto gyro_probe_cdev_add;
	}
	
	gyro_cdev->ops = &gyro_fops;
	gyro_cdev->owner = THIS_MODULE;

	gyro_enable_pin = pdata->gyro_enable_pin;
	gyro_adc_channel[0] = pdata->adc_channel[0];
	gyro_adc_channel[1] = pdata->adc_channel[1];
	
	PK_INFO( "TomTom GO gyro driver: initialized.\n"); 	
	
	return 0;
	
gyro_probe_cdev_add:
	cdev_del( gyro_cdev );
	
gyro_probe_register_chrregion_error:
	kfree( gyro_cdev );
	
	return -ENODEV;
}


/*-----------------------------------------------------------------------------*/


static struct device_driver gyro_driver = {
	.name		= "tomtomgo-gyro",
	.bus		= &platform_bus_type,
	.probe		= gyro_probe,
	.remove		= gyro_remove,
	.suspend	= gyro_suspend,
	.resume		= gyro_resume,
};

/*-----------------------------------------------------------------------------*/

static int __init gyro_init(void) {
	int ret;
	printk(KERN_INFO "TomTom GO Gyro Driver, (C) 2007 TomTom BV\n");

	PK_DBG("Registering driver\n");
	ret = driver_register(&gyro_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	memset( &gyro_private_list[0], 0, sizeof(gyro_private_list));  /* clear the list	*/
	return 0;
}

module_init(gyro_init);

/*-----------------------------------------------------------------------------*/

static void __exit gyro_exit(void){
	
	PK_DBG("Unregistering driver\n");
	driver_unregister(&gyro_driver);
	PK_DBG("Done\n");
	
}
module_exit(gyro_exit);

/*-----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("Gyro Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mark Vels <Mark.Vels@tomtom.com>");

/* EOF	*/
