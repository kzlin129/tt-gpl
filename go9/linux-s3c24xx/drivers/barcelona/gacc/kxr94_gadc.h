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

#ifndef DRIVERS_BARCELONA_GACC_KXR94_H
#define DRIVERS_BARCELONA_GACC_KXR94_H

#define KXR94_POLL_RATE			50
#define KXR94_GADC_DRIVER_NAME		"kxr94-gadc"

#define KXR94_P74_ADDR_CONVERT_X      (0x00)
#define KXR94_P74_ADDR_CONVERT_Y      (0x02)
#define KXR94_P74_ADDR_CONVERT_Z      (0x01)

#define KXR94_R94_ADDR_CONVERT_X      (0x00)
#define KXR94_R94_ADDR_CONVERT_Y      (0x01)
#define KXR94_R94_ADDR_CONVERT_Z      (0x02)
#define KXR94_R94_ADDR_CONVERT_AUX    (0x07)

#define KXR94_ADDR_READ_CTRL          (0x03)
#define KXR94_ADDR_WRITE_CTRL         (0x04)
#define KXR94_CTRL_ENABLE             (1<<2)

#define KXR94_MODE_ON                 ( KXR94_CTRL_ENABLE )
#define KXR94_MODE_OFF                ( 0 )

#define KXR94_ADC_FLGS_EXIT           (1<<0)
#define KXR94_ADC_FLGS_QOVERFLOW      (1<<1)
#define KXR94_ADC_FLGS_CONGESTION     (1<<2)

#define KXR94_P74_CHANNELS            3
#define KXR94_R94_CHANNELS            4
#define KXR94_MAX_CHANNELS            4

static const int kxr94_p74_channels_array[KXR94_P74_CHANNELS] =
    { KXR94_P74_ADDR_CONVERT_X,
	KXR94_P74_ADDR_CONVERT_Y,
	KXR94_P74_ADDR_CONVERT_Z
};

static const int kxr94_r94_channels_array[KXR94_R94_CHANNELS] =
    { KXR94_R94_ADDR_CONVERT_X,
	KXR94_R94_ADDR_CONVERT_Y,
	KXR94_R94_ADDR_CONVERT_Z,
	KXR94_R94_ADDR_CONVERT_AUX
};
struct channel_samples {
	unsigned long int total_sample;
	int nr_samples;
	int channel_nr;
	spinlock_t lock;
	struct gadc_buffer buffer;
	struct timer_list timer;
};

struct kxr94_gadc {
	struct workqueue_struct *work_queue;
	struct work_struct work;
	struct timer_list timer;
	int flags;
	struct spi_device *spi_dev;
	struct channel_samples channel[KXR94_MAX_CHANNELS];
	int nr_channels;
};

static inline unsigned short kxr94_to_raw_value( u8* b )
{
	return ( b[0] << 4 ) | ((b[1] & 0xF0) >> 4);
}

#endif
