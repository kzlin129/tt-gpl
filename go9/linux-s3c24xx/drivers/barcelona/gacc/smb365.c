/* drivers/barcelona/gacc/smb365.c
 *
 * Implementation of the Bosch SMB365 accelerometer SPI protocol driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <mark.vels@tomtom.com>
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
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
/* Include generic accelerometer interface definition	*/
#include <barcelona/gacc.h>
#include <barcelone/debug.h>
#define SMB365_READING_MEM_SIZE	(6) 	/* need to read 6 bytes to get x, y and z	*/
#define SMB365_ADDR_READ		(0x80)		/* highest address bit indicates read or write	*/

//#define SW_TEST					(1)			/* Fake acc data, don't touch any of the hardware features	*/
#undef SW_TEST

#define HERE()	{ printk("%s line %d says: HERE!\n", __FILE__, __LINE__ ); }
#define TELL(X)	{ printk("%s line %d says: %s = %d ( 0x%02x) \n", __FILE__, __LINE__, #X, (int) (X), (int) (X)); }

#define SMB365_DRIVER_NAME	"smb365"

enum { SMB365_MODE_2G = 0, SMB365_MODE_10G };

typedef struct {
	u8* rx_buf;		/* DMA safe membuf for sending/ receiving SPI messages	*/ 
	u8* tx_buf;
} smb365_private_t;

#ifdef SW_TEST
#define CHECK_BUFFERS(X)	test_check_buffers( X )		
#else
#define CHECK_BUFFERS(X)		while( 0 ){}
#endif /*	SW_TEST	*/

/*-----------------------------------------------------------------------------*/
/* Global variables															   */
/*-----------------------------------------------------------------------------*/

static accelerometer_settings_t	smb365_mode_settings[] = {
	[SMB365_MODE_2G] = {
		.sensitivity	= 256,	/* typical datasheet rating		*/
		.range			= 0x01ff,
	},
	[SMB365_MODE_10G] = {
		.sensitivity	= 51,	/* typical datasheet rating		*/
		.range			= 0x01ff,	
	}
};

static struct cdev			*smb365_cdev;
static struct spi_device	*smb365_spi_dev;
static int 					 smb365_busy;
static smb365_private_t		 smb365_private;


/*-----------------------------------------------------------------------------
 * 
 * PART 1 : Implementation of the smb265 local functions
 * 
 * Low level timing is done with a kernel timer. The kernel timer event handler 
 * will schedule a new work item on a private work queue and then reschedule itself
 * according to sample_rate for the next timer event. The rationale behind the work queue 
 * is that the SPI stack calls cannot be called from an atomic context and therefore
 * the kernel timer event handler and tasklets are disqualified. An advantage of that is
 * that synchronization on the data store can now be done with a simple spinlock without
 * worrying about deadlocks because code executed by the work queue magic wand can sleep.
 * 
 *-----------------------------------------------------------------------------
 */
 
static inline unsigned short smb365_to_raw_value( u8* b ){
	 return ( (b[0] & 0x7f) << 3 ) | (b[1] & 0x07);
}

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
/**
 * DO fake data reading. Pretend we are running in square circles :-)
 */
static inline int fill_with_fake_data( u8* mem ){
	short x;
	short y;
	short z;
	unsigned int ms;
	u8*  temp;
	//const int period = (1<<13)-1; /* 0x1fff (8191) milliseconds	*/
	const int period = (1<<10)-1;
	
	temp = mem;
	x = y = 0;
	z = smb365_mode_settings[SMB365_MODE_2G].sensitivity;
	ms = jiffies_to_msecs(jiffies);
	ms &= period;	/* modulus	*/
	if( ms < (period >>2)){		/* 1/4 pi	*/
		x = y = smb365_mode_settings[SMB365_MODE_2G].sensitivity;
	}
	else if( ms < (period >>1) ){		/* 1/2 pi	*/
		x = smb365_mode_settings[SMB365_MODE_2G].sensitivity;
		y = -smb365_mode_settings[SMB365_MODE_2G].sensitivity;
	}
	else if( ms < ((period >>1)+(period>>2)) ){	/* 3/4 pi	*/
		x = y = -smb365_mode_settings[SMB365_MODE_2G].sensitivity;
	}
	else {
		x=-smb365_mode_settings[SMB365_MODE_2G].sensitivity;
		y=smb365_mode_settings[SMB365_MODE_2G].sensitivity;
	}
	
	*temp++ = (u8) ( (x>>3) & 0x7f);
	*temp++ = (u8) ( x & 0x07 );
	*temp++ = (u8) ( (z>>3) & 0x7f);
	*temp++ = (u8) ( z & 0x07 );
	*temp++ = (u8) ( (y>>3) & 0x7f);
	*temp++ = (u8) ( y & 0x07 );
	return 0;
}
#endif /* SW_TEST	*/

/*-----------------------------------------------------------------------------*/

/**
 * The 10 bits reading from the SMB365 is already 2's complement. We need 
 * to convert it to a real signed short. We do this basically by moving
 * the sign bit from bit 9 to bit 15 if it is set and fill the new bits with 1's.
 */
static inline short smb365_to_signed_short( unsigned short v){
	short result = (short) ( v & 0x03ff );
	if( result & (1<<9) ){		/* is sign bit set?	*/
		result |= (~0x01ff);	/* set the new sign bit	and pad with 1's*/
	}
	return result;
} 

/*-----------------------------------------------------------------------------*/

static int smb365_init_hw( struct gacc_driver_private_data* private ){
	struct gacc_platform_data *pdata;
#ifndef SW_TEST
	gopin_t chipselect = private->spi_dev->chip_select;
#endif /* SW_TEST	*/

	pdata = private->spi_dev->dev.platform_data;

#ifndef SW_TEST
	IOP_SetFunction( chipselect );
	IOP_SetInput( pdata->irq_pin );	/* Don't use the IRQ	*/
#endif /* SW_TEST	*/

	return -EINVAL;
}
 
/*-----------------------------------------------------------------------------*/

static int smb365_exit_hw( struct gacc_driver_private_data* private ){
	
#ifndef SW_TEST	
	gopin_t chipselect = private->spi_dev->chip_select;
	
	IOP_SetInput( chipselect );	/* Disable CS	*/
#endif /* SW_TEST	*/	
	return 0;
}

/*-----------------------------------------------------------------------------*/

/**
 * Query the SMB365 for a reading. Store the results in r.
 */
static int smb365_get_reading( struct gacc_driver_private_data* private, accelerometer_reading_t* r ){

	u8* rx_buf;
	u8* tx_buf;
	struct spi_message msg;
	struct spi_transfer txf[SMB365_READING_MEM_SIZE * 2];	/* 6 address cycles (W) and 6 read cycles	*/
	u8 addr;
	int i;
	struct spi_transfer* temp;
	int result;
	
	if( (private == NULL) || ( r == NULL) ){
		return -EINVAL;	
	}
	rx_buf = ((smb365_private_t*)(private->private))->rx_buf;
	tx_buf = ((smb365_private_t*)(private->private))->tx_buf;
	
	/* Setup spi message	*/
	memset( rx_buf, 0 , SMB365_READING_MEM_SIZE );	
	memset( &msg, 0 , sizeof( msg ));		/* Explicitly clear all members	*/
	spi_message_init( &msg );
	memset( &txf[0], 0 , sizeof( txf ));	/* Explicitly clear all members	*/
	addr = 0x1a | SMB365_ADDR_READ;
	temp = &txf[0];
	for( i=0; i < SMB365_READING_MEM_SIZE; i++){
		/* prepare address write cycle transfer */
		tx_buf[i] = addr +i;
		temp->tx_buf = &tx_buf[i];
		temp->len = 1;
		spi_message_add_tail( temp, &msg );		/* add this transfer	*/
		temp++;
		/* prepare read transfer	*/
		temp->rx_buf = &rx_buf[i];
		temp->len = 1;
		spi_message_add_tail( temp, &msg );		/* add this transfer	*/
		temp++;	
	}
	msg.spi	= private->spi_dev;
	msg.is_dma_mapped = 0;
#ifdef SW_TEST
	result = fill_with_fake_data( rx_buf );
#else
	printk("starting spi_sync\n");
	result = spi_sync( private->spi_dev, &msg );
	printk("Finished spi_sync (result = %d)\n", result );
#endif /*	SW_TEST	*/
	r->timestamp = jiffies_to_msecs(jiffies); /* assmuption: delay after spi trx is shorter than before trx	*/
	if( ! result){
		unsigned short x;
		unsigned short y;
		unsigned short z;
		x = smb365_to_raw_value( rx_buf );
		z = smb365_to_raw_value( rx_buf + 2 );
		y = smb365_to_raw_value( rx_buf + 4 );
#ifdef GACC_DEBUG
		r->x_raw = x;
		r->z_raw = z;
		r->y_raw = y;
#endif /* GACC_DEBUG	*/
		r->x_axis = smb365_to_signed_short( x );
		r->y_axis = smb365_to_signed_short( y );
		r->z_axis = smb365_to_signed_short( z );
	}
	
	return result;
}

/*-----------------------------------------------------------------------------*/
/**
 * Kernel timer handler function.
 * Is running from atomic context so it cannot sleep. It will schedule
 * a work queue item to handle the reading and storing of sensor data.
 */
static void smb365_timer_handler( unsigned long  arg ){
	
	unsigned short sample_rate;
	struct gacc_driver_private_data* smb365_private = (struct gacc_driver_private_data*) arg;
	
	if( ! ( smb365_private->flags & GACC_FLGS_EXIT ) ){ /* if we must exit then return and do not reschedule	*/
		sample_rate = smb365_private->settings->sample_rate;
		if( queue_work( smb365_private->work_queue, &smb365_private->work ) ){
			/* There obviously is congestion if we try to schedule work
			 * while the previous scheduled one is not finished yet	*/	
			 smb365_private->flags|= GACC_FLGS_CONGESTION;
		}
		
		smb365_private->timer.expires = jiffies + ( HZ +(sample_rate>>1))/(sample_rate);
		add_timer( &smb365_private->timer );
	}
}

/*-----------------------------------------------------------------------------*/

static void smb365_do_work( void* arg ){
	int result;
	accelerometer_reading_t temp;
	struct gacc_driver_private_data* smb365_private = (struct gacc_driver_private_data*) arg;
	
	/*
	 * We will use a temporary struct to pass to get_reading() so that we don't have to lock the private data too long.
	 * smb365_get_reading() might sleep.
	 */ 
	result = smb365_get_reading( smb365_private,  &temp);
	
	spin_lock( &smb365_private->lock );
	CHECK_BUFFERS( smb365_private );
	memcpy( &smb365_private->head[smb365_private->readings_writeindex], &temp, sizeof(accelerometer_reading_t) );
	CHECK_BUFFERS( smb365_private );
	if( !result ){
		if( smb365_private->readings_count < smb365_private->readings_size ){
			smb365_private->readings_count++;
		}
		else{	/* We're overwriting something!	*/
			smb365_private->flags |= GACC_FLGS_QOVERFLOW;
			smb365_private->readings_readindex++;	/* move read index along	*/
			if( smb365_private->readings_readindex >= smb365_private->readings_size){
				smb365_private->readings_readindex = 0; /* wrap around	*/
			}
		}
		smb365_private->readings_writeindex++;
		if( smb365_private->readings_writeindex >= smb365_private->readings_size){
			smb365_private->readings_writeindex = 0; /* wrap around	*/
		}
	}
	spin_unlock( &smb365_private->lock );
}

/*-----------------------------------------------------------------------------*/
/*
 * Function to copy the data from the store to user land. 
 * The store can be asymetrical (i.e. more recent data at lower indices ) in case
 * of an overflow situation.
 * Return a 'read()' char operation style return value.
 */
static int smb365_copy_elements_to_user( struct gacc_driver_private_data* private, char __user *buf, size_t count ){
	int elements2copy;
	int result;
	
	spin_lock( &private->lock );
	elements2copy = min( count/ sizeof(accelerometer_reading_t), private->readings_count);
	
	TELL( elements2copy );
	TELL( private->readings_readindex );
	TELL( private->readings_writeindex );
	TELL( private->readings_count );
	
	if(!access_ok(VERIFY_WRITE, buf, elements2copy*sizeof(accelerometer_reading_t) ) ){
		spin_unlock( &private->lock );
		return -EFAULT;
	} 
	
	if( private->readings_readindex  < private->readings_writeindex ){
		TELL( elements2copy );
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
		TELL( elements_from_this_chunk );
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
				TELL( elements_from_this_chunk );
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
	TELL( private->readings_readindex );
	TELL( private->readings_writeindex );
	TELL( private->readings_count );
	TELL( result );
	spin_unlock( &private->lock );
	return result;
}

/*-----------------------------------------------------------------------------
 * 
 * PART 2 : Implementation of the generic accelerometer character device interface
 * 
 *-----------------------------------------------------------------------------
 */
static ssize_t smb365_read( struct file *file, char __user *buf, size_t count, loff_t * loff) {
	int result;
	struct gacc_driver_private_data* smb365_private = file->private_data;
	
	/* We don't need to lock at this point yet	*/
	if( smb365_private->readings_count <= 0 ){
		if( file->f_flags & O_NONBLOCK ) {
			return -EAGAIN;
		}
		else{
			while( smb365_private->readings_count <= 0 ){
				DEFINE_WAIT( wait );
				prepare_to_wait( &smb365_private->wait_queue, &wait, TASK_INTERRUPTIBLE);
				if( smb365_private->readings_count <= 0 )
					schedule();
				finish_wait( &smb365_private->wait_queue, &wait );
				if( signal_pending( current ))
					return -ERESTARTSYS;
			}
		}
	}
	/* assume there is data available beyond this point	*/
	result = -EINVAL;
	
	spin_lock( &smb365_private->lock );
	if( (count/ sizeof(accelerometer_reading_t)) > 0 ){
		result = smb365_copy_elements_to_user( smb365_private, buf, count );
	}
	spin_unlock( &smb365_private->lock );
	
	return result;
}


/*-----------------------------------------------------------------------------*/

static int smb365_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) { 
	int result;
	struct gacc_driver_private_data* smb365_private = file->private_data;
	
	switch( cmd ){
		case GACC_IOCTL_RESET:
			/* drop all data in store	*/
			spin_lock_bh( &smb365_private->lock );
			smb365_private->readings_count = 0;
			smb365_private->readings_writeindex = 0;
			spin_unlock_bh( &smb365_private->lock );
			result = 0;	/* This stupid thing doesn't have a reset functionality so it cannot fail either then	*/
			break;
		case GACC_IOCTL_BIST:
		    result = -ENOSYS;
			break;
		case GACC_IOCTL_SET_SAMPLE_RATE:
			if( (arg > 0) && (arg <= GACC_MAX_SAMPLERATE) ){
				spin_lock_bh( &smb365_private->lock );
				smb365_private->settings->sample_rate = arg;
				spin_unlock_bh( &smb365_private->lock );
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
				if( __copy_to_user( (char __user *)arg, smb365_private->settings, sizeof(accelerometer_settings_t) ) ){
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

static int smb365_open( struct inode *inode, struct file *file ) {
	
	struct gacc_driver_private_data* private;
	if( smb365_busy ){
		return -EBUSY;	
	}
	smb365_busy = 1; /* mark this device as in use	*/
	private = kmalloc( sizeof(struct gacc_driver_private_data), GFP_KERNEL );
	if(! private ){
		printk( SMB365_DRIVER_NAME": Could not alloc private data!\n");
		return -ENOMEM;	
	}
	/* Init private struct	*/
	private->settings = &smb365_mode_settings[SMB365_MODE_2G];
	private->settings->sample_rate = GACC_DEFAULT_SAMPLERATE;
	private->spi_dev = smb365_spi_dev;
	/* Setup fifo for storing readings	*/
	private->readings_writeindex = private->readings_readindex = 0;
	private->readings_count = 0;
#ifdef SW_TEST
	/* allocate an element before and after the buffer. We will fill it with a recognizable value
	 * so that we can check if someone has overwritten it.
	 */
	private->head = (accelerometer_reading_t*) kmalloc( (GACC_READING_QUEUE_SIZE+2) * sizeof(accelerometer_reading_t), GFP_KERNEL );
#else
	private->head = (accelerometer_reading_t*) kmalloc( GACC_READING_QUEUE_SIZE * sizeof(accelerometer_reading_t), GFP_KERNEL );
#endif /*	SW_TEST	*/
	if( ! private->head ){
		kfree( private );
		return -ENOMEM;	
	}
	private->readings_size = GACC_READING_QUEUE_SIZE;
	spin_lock_init( &private->lock );
	
	/* Setup SPI transfer buffers	*/
	private->private = &smb365_private; /* SMB365 private data, not GACC	*/
	((smb365_private_t*)(private->private))->rx_buf = kmalloc( SMB365_READING_MEM_SIZE, GFP_KERNEL | __GFP_DMA );
	((smb365_private_t*)(private->private))->tx_buf = kmalloc( SMB365_READING_MEM_SIZE, GFP_KERNEL | __GFP_DMA );
	if( !( ((smb365_private_t*)(private->private))->rx_buf && ((smb365_private_t*)(private->private))->tx_buf) ){
		if( ((smb365_private_t*)(private->private))->rx_buf)
			kfree( ((smb365_private_t*)(private->private))->rx_buf);
		if( ((smb365_private_t*)(private->private))->tx_buf)
			kfree( ((smb365_private_t*)(private->private))->tx_buf );
		kfree( private->head );
		kfree( private );
		return -ENOMEM;	
	}
	
	/* Setup kernel timer	*/
	init_timer( &private->timer );
	private->timer.function  = smb365_timer_handler;
	private->timer.data = (unsigned long) private;	/* argument to timer handler is private struct	*/
	private->work_queue = create_singlethread_workqueue(SMB365_DRIVER_NAME);
	private->flags = 0;
	/* setup work/wait queues	*/
	INIT_WORK( &private->work, smb365_do_work, private );
	init_waitqueue_head( &private->wait_queue );
	
	/* store private data in struct file	*/
	file->private_data = private;
	/* init hardware	*/
	smb365_init_hw( private );
	
	/* start our self-rescheduling timer */
	smb365_timer_handler( (unsigned long) private ); /* simply call it, it will reschedule itself	*/
#ifdef SW_TEST
	private->head++;	/* we reserved space for one extra element at the start for testing	*/
	test_fill_testbuffers( private );
#endif /*	SW_TEST	*/
	
	return 0;
}

/*-----------------------------------------------------------------------------*/
#define TELL(X)	{ printk("%s line %d says: %s = %d ( 0x%02x) \n", __FILE__, __LINE__, #X, (int) (X), (int) (X)); }
static int smb365_release(struct inode *inode, struct file *file ) {
	
	struct gacc_driver_private_data* smb365_private = file->private_data;
	TELL( smb365_private );
	if( smb365_private ){
		smb365_private->flags |= GACC_FLGS_EXIT;
		del_timer( &smb365_private->timer );	/* remove any scheduled timer	*/
		
		TELL( smb365_private->work_queue );
		flush_workqueue( smb365_private->work_queue );	/* make sure nothin' is running anymore	*/
		destroy_workqueue( smb365_private->work_queue );
		
		smb365_exit_hw( smb365_private );
#ifdef SW_TEST
		smb365_private->head--;	/* we added one element extra at the beginning	*/
#endif /* SW_TEST */	
		TELL( smb365_private->head );
		kfree( smb365_private->head );
#ifdef SW_TEST
		/* clear it so that any use after the kfree() will at least trigger a segmentation fault	*/
		memset( smb365_private, 0, sizeof(*smb365_private) ); 
#endif /* SW_TEST */
		kfree( smb365_private );	
		
		file->private_data = NULL;
	}
	smb365_busy = 0;
	return 0;
}

/*-----------------------------------------------------------------------------*/

static struct file_operations smb365_fops = {
	.owner 		= THIS_MODULE,
	.read		= smb365_read,
	
	.ioctl		= smb365_ioctl,
	.open		= smb365_open,
	.release	= smb365_release,
};

/*-----------------------------------------------------------------------------
 * 
 * PART 3 : Implementation of the SPI driver interface
 * 
 *-----------------------------------------------------------------------------
 */
static int smb365_remove( struct spi_device *spi ) {
	
	cdev_del( smb365_cdev );
	kfree( smb365_cdev );
	
	return 0;
}

/*-----------------------------------------------------------------------------*/

static int smb365_suspend( struct spi_device *spi, pm_message_t mesg ){
	
	return -EINVAL;
}

/*-----------------------------------------------------------------------------*/

static int smb365_resume( struct spi_device *spi ) {
	
	return -EINVAL;
}

/*-----------------------------------------------------------------------------*/

static int smb365_probe( struct spi_device *spi ){
	
	struct gacc_platform_data    *pdata = spi->dev.platform_data;

	if( ! pdata ){
		PK_DBG_FUNC( SMB365_DRIVER_NAME": Missing platform data!\n");
		return -ENODEV;
	}
	
	printk( "TomTom GO SMB365 SPI accelerometer, (C) 2006 TomTom BV\n"); 	
	smb365_cdev = cdev_alloc();
	if(! smb365_cdev){
		printk( SMB365_DRIVER_NAME": Could not allocate platform data!\n");	
		return -ENODEV;
	}
	
	
	if( register_chrdev_region( pdata->device_nr, 1, SMB365_DRIVER_NAME ) ){
		printk( SMB365_DRIVER_NAME": Could not register cdev major/minor range!\n");	
		goto smb365_probe_register_chrregion_error;
	}
	
	/* Ready for action?	*/
	if( cdev_add( smb365_cdev, pdata->device_nr, 1) ){
		printk( SMB365_DRIVER_NAME": Could not add cdev !\n");	
		goto smb365_probe_cdev_add;
	}
	
	smb365_cdev->ops = &smb365_fops;
	smb365_cdev->owner = THIS_MODULE;
	spi->bits_per_word = 8;
	spi->irq = -1;
	printk( "TomTom GO SMB365 SPI accelerometer: initialized.\n"); 	
	smb365_spi_dev = spi; /* Store it for use in the cdev open()	*/
	smb365_busy = 0; /* we're ready to rock	*/
	
	return 0;
	
smb365_probe_cdev_add:
	cdev_del( smb365_cdev );
	
smb365_probe_register_chrregion_error:
	kfree( smb365_cdev );
	
	return -ENODEV;
}

/*-----------------------------------------------------------------------------*/

static struct spi_driver smb365_driver = {
	
    .driver = {
        .name   = SMB365_DRIVER_NAME,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .probe          = smb365_probe,
    .remove         = smb365_remove,
    .suspend        = smb365_suspend,
    .resume         = smb365_resume,
};

/*-----------------------------------------------------------------------------*/

static int __init smb365_init(void) {
	return spi_register_driver(&smb365_driver);
}

module_init(smb365_init);

/*-----------------------------------------------------------------------------*/

static void __exit smb365_exit(void){
	
	spi_unregister_driver(&smb365_driver);
}
module_exit(smb365_exit);

/*-----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("SMB365 Accelerometer Driver");
MODULE_LICENSE("GPL");

/* EOF	*/
