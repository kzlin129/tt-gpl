/* drivers/barcelona/gadc/s3c24xx_gadc.h
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

#ifndef DRIVERS_BARCELONA_GADC_S3C24XX_GADC_H
#define DRIVERS_BARCELONA_GADC_S3C24XX_GADC_H

#define S3C24XX_GADC_DRIVER_NAME        "s3c24xx-gadc"
struct channel_samples
{
        unsigned short int      total_sample;
        int                     nr_samples;
        int                     channel_nr;
        spinlock_t              lock;

        struct gadc_buffer      buffer;
        struct timer_list       timer;
};

struct s3c24xx_gadc
{
        struct channel_samples  *channel;
        int                     nr_channels;
};

#endif /* DRIVERS_BARCELONA_GADC_S3C24XX_GADC_H */
