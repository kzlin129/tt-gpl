/* drivers/barcelona/baro/scp1000_baro.h
 *
 * Implementation of the SCP1000 Barometer driver. 
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <Rogier.Stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DRIVERS_BARCELONA_BARO_SCP1000_BARO_H
#define DRIVERS_BARCELONA_BARO_SCP1000_BARO_H
#include <barcelona/gadc.h>

struct scp1000_baro_private
{
	struct gadc_buffer		temp_cbuf;
	struct gadc_buffer		pres_cbuf;
	struct timer_list		timer;
	struct workqueue_struct		*work_queue;
	struct work_struct		work_start;
	struct work_struct		work_fini;
	struct spi_device		*spi_dev;
	int				flags;
};

//#define SCP1000_MAX_SAMPLERATE		9000
//#define SCP1000_MAX_HP_SAMPLERATE	1800
#define SCP1000_MAX_SAMPLERATE		7900
#define SCP1000_MAX_HP_SAMPLERATE	1600

#define SCP1000_BARO_FLGS_EXIT		(1<<0)
#define SCP1000_BARO_FLGS_FI_CONGESTION	(1<<1)
#define SCP1000_BARO_FLGS_ST_CONGESTION	(1<<2)
#define SCP1000_BARO_FLGS_SPI_ERROR	(1<<3)
#define SCP1000_BARO_FLGS_TMP_OPEN	(1<<4)
#define SCP1000_BARO_FLGS_PRS_OPEN	(1<<5)

#define SCP1000_BARO_REVID		0x03
#define SCP1000_BARO_DRIVER_NAME	"scp1000-baro"

enum scp1000_spi_commands
{
	REVID=0x00,
	DATAWR=0x01,
	ADDPTR=0x02,
	OPERATION=0x03,
	OPSTATUS=0x04,
	RSTR=0x06,
	STATUS=0x07,
	DATARD8=0x1F,
	DATARD16=0x20,
	TEMPOUT=0x21
};

enum scp1000_spi_icommands
{
	CFG=0x00,
	TWIADD=0x05,
	USERDATA1=0x29,
	USERDATA2=0x2A,
	USERDATA3=0x2B,
	USERDATA4=0x2C
};

static int scp1000_baro_hwinit( struct spi_device *spi_dev );
#ifdef CONFIG_PM
static int scp1000_baro_suspend( struct spi_device *spi, pm_message_t mesg );
static int scp1000_baro_resume( struct spi_device *spi );
#else
#define scp1000_baro_suspend	NULL
#define scp1000_baro_resume	NULL
#endif
static int scp1000_baro_remove( struct spi_device *spi_dev );
static int scp1000_baro_open( struct inode *inode, struct file *file );
static int scp1000_baro_release(struct inode *inode, struct file *file );
static unsigned scp1000_baro_poll(struct file *file, poll_table *wait);
static int scp1000_baro_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static ssize_t scp1000_baro_read( struct file *file, char __user *buf, size_t count, loff_t * loff);
static int scp1000_spi_niread( struct spi_device *spi_dev, enum scp1000_spi_commands regidx, void *data );
static int scp1000_spi_niwrite( struct spi_device *spi_dev, enum scp1000_spi_commands regidx, unsigned char data );
#if 0
/* Function is not used. To stop GCC from whining. */
static int scp1000_spi_iread( struct spi_device *spi_dev, enum scp1000_spi_icommands regidx, unsigned char *data );
#endif
static int scp1000_spi_iwrite( struct spi_device *spi_dev, enum scp1000_spi_icommands regidx, unsigned char data );
static void scp1000_baro_wq_trans_start( void *arg );
static void scp1000_baro_wq_trans_fini( void *arg );
static void scp1000_baro_timer_handler( unsigned long  arg );
static irqreturn_t scp1000_baro_drdy_irq_handler( int irq, void *dev_id, struct pt_regs *regs );
static int scp1000_baro_set_samplerate( struct scp1000_baro_private *private, unsigned long int samplerate );
static struct file_operations scp1000_baro_fops;
static int scp1000_baro_probe( struct spi_device *spi_dev );
static struct spi_driver scp1000_baro_driver;

#endif /* DRIVERS_BARCELONA_BARO_SCP1000_BARO_H */
