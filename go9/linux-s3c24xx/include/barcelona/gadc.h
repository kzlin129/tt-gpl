/* include/barcelona/gadc.h
 *
 * Implementation of the Generic ADC driver.
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <Rogier.Stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_BARCELONA_GADC_H
#define INCLUDE_BARCELONA_GADC_H
#ifdef __KERNEL__
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/wait.h>
#endif

/* Kernel and userspace defines. */
#define ADC_BUFFER_SIZE		30
#define ADC_SAMPLE_OVERFLOW	0x01
#define ADC_SAMPLE_UNDERFLOW	0x02

#define ADC_DRIVER_MAGIC	'R'
#define ADC_SET_SAMPLERATE	_IOW(ADC_DRIVER_MAGIC, 1, sizeof(unsigned long int) )
#define ADC_GET_SAMPLERATE	_IOR(ADC_DRIVER_MAGIC, 2, sizeof(unsigned long int) )
#define ADC_GET_BUFFERSTATUS	_IOR(ADC_DRIVER_MAGIC, 3, sizeof(unsigned long int) )

/* Major number used by the GADC driver. */
#define GADC_MAJOR		123

struct gadc_sample
{
	struct timeval		timestamp;
	unsigned long int	value;
};

#ifdef __KERNEL__
/* Kernel ONLY structures and defines. */
struct gadc_platform_data
{
	int			major;
	int			minor;
	int			channel;
};

struct gadc_cdev
{
	char		*name;
	struct cdev	dev;
	unsigned int	major;
	unsigned int	minor;
	unsigned int	minor_range;
	void		*private;
};

struct gadc_buffer
{
	struct gadc_sample	buffer[ADC_BUFFER_SIZE];
	int			curr_idx;
	int			last_read_idx;
	unsigned short int	flags;
	spinlock_t		lock;
	wait_queue_head_t	poll_queue;
	unsigned long int	sample_rate;
};

extern void unregister_gadc_cdev( struct gadc_cdev *cdev );
extern struct gadc_cdev *get_gadc_cdev( struct cdev *cdev );
extern int register_gadc_cdev( struct gadc_cdev *cdev, unsigned int major, unsigned int minor, unsigned int count,
				const char *name, struct file_operations *fops );
extern void gadc_init_buffer( struct gadc_buffer *adc, unsigned long int samplerate );
extern void gadc_add_sample( struct gadc_buffer *adc, unsigned long int value );
extern void gadc_clear_buffer( struct gadc_buffer *adc );
extern int gadc_get_sample( struct gadc_buffer *adc, struct gadc_sample *target );
extern int gadc_get_samplecount( struct gadc_buffer *adc, int *samplecount );
extern unsigned long int gadc_get_status( struct gadc_buffer *adc );
extern void gadc_set_samplerate( struct gadc_buffer *adc, unsigned long int sample_rate );
extern unsigned long int gadc_get_samplerate( struct gadc_buffer *adc );
extern int gadc_wait_buffer_fill( struct gadc_buffer *adc, int *samplecount );
extern void gadc_poll_wait( struct file *file, struct gadc_buffer *adc, poll_table *wait );

#endif /* #ifdef __KERNEL__ */

#endif /* INCLUDE_BARCELONA_GADC_H */
