/* drivers/barcelona/gacc/kxp74.h
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

#include <barcelona/gadc.h>

#ifndef DRIVERS_BARCELONA_GACC_KXP74_H
#define DRIVERS_BARCELONA_GACC_KXP74_H
//#define SW_TEST	(1)	/* Fake acc data, don't touch any of the hardware features	*/
#undef SW_TEST
#ifdef SW_TEST
#define CHECK_BUFFERS(X)	test_check_buffers( X )		
#else
#define CHECK_BUFFERS(X)		while( 0 ){}
#endif /*	SW_TEST	*/

#define HERE()	{ printk("%s line %d says: HERE!\n", __FILE__, __LINE__ ); }
#define TELL(X)	{ printk("%s line %d says: %s = %d ( 0x%02x) \n", __FILE__, __LINE__, #X, (int) (X), (int) (X)); }

#define KXP74_DRIVER_NAME	"kxp74"
#define KXP74_BUF_SIZE		(6)
#define KXP74_ADDR_CONVERT_X	(0x00)
#define KXP74_ADDR_CONVERT_Z	(0x01)
#define KXP74_ADDR_CONVERT_Y	(0x02)
#define KXP74_ADDR_READ_CTRL	(0x03)
#define KXP74_ADDR_WRITE_CTRL	(0x04)
#define KXP74_CTRL_ENABLE		(1<<2)

enum { KXP74_SPEED_32KHZ = 0, KXP74_SPEED_8KHZ, KXP74_SPEED_4KHZ, KXP74_SPEED_2KHZ };
#define KXP74_MODE_ON		( KXP74_CTRL_ENABLE | (KXP74_SPEED_32KHZ << 3))
#define KXP74_MODE_OFF		( 0 )

typedef struct {
	u8* rx_buf;		/* DMA safe membuf for sending/ receiving SPI messages	*/ 
	u8* tx_buf;
} kxp74_private_t;

struct kxp74_cdev
{
	int				is_kxr94;
	struct gadc_cdev		kxp74_dev;
	struct gadc_cdev		kxr94_adc_dev;
	struct spi_device		*spi_dev;
};

static inline unsigned short kxp74_to_raw_value( u8* b )
{
	return ( b[0] << 4 ) | ((b[1] & 0xF0) >> 4);
}

/*-----------------------------------------------------------------------------*/

/**
 * The zero g offset is 2048 and the raw value is an unsigned 12-bit value.
 */
static inline short kxp74_to_signed_short( unsigned short v)
{
	signed short result;

	result = ( ((signed short) v) - 2048);
	return result;
}

extern int kxp74_exit_hw( struct spi_device *spi_dev );
extern int kxp74_init_hw( struct spi_device *spi_dev );
extern int kxp74_probe( struct spi_device *spi );
extern int kxp74_remove( struct spi_device *spi );

#ifdef CONFIG_PM
extern int kxp74_suspend( struct spi_device *spi, pm_message_t mesg );
extern int kxp74_resume( struct spi_device *spi );
#else
#define kxp74_suspend   NULL
#define kxp74_resume    NULL
#endif /* CONFIG_PM */

#endif
