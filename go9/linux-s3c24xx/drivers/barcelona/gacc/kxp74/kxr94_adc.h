/* drivers/barcelona/gacc/kxr94_adc.h
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

#include "kxp74.h"

#ifndef DRIVERS_BARCELONA_GACC_KXR94_H
#define DRIVERS_BARCELONA_GACC_KXR94_H

#define KXR94_DRIVER_NAME	"kxr94"
#define KXP94_ADDR_CONVERT_AUX	(0x07)

#define KXR94_MODE_ON		( KXP74_CTRL_ENABLE )
#define KXR94_MODE_OFF		( 0 )

#define KXR94_ADC_FLGS_EXIT		(1<<0)
#define KXR94_ADC_FLGS_QOVERFLOW	(1<<1)
#define KXR94_ADC_FLGS_CONGESTION	(1<<2)

struct kxr94_adc_private
{
	struct gadc_buffer		*adc;
	struct timer_list		timer;
        struct workqueue_struct		*work_queue;
        struct work_struct		work;
	struct spi_device		*spi_dev;
	int				flags;
};

extern struct file_operations kxr94_adc_fops;
extern struct spi_driver kxr94_driver;
extern int kxr94_spi_read( struct spi_device *spi_dev, unsigned char cmd, unsigned short *data );
extern int kxr94_spi_write( struct spi_device *spi_dev, unsigned char addr, unsigned char data );
extern void kxr94_adc_timer_handler( unsigned long  arg );

#endif
