/* si4710.c
 *
 * Control driver for Silicon Lab 4710 chip.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Ithamar Adema <ithamar.adema@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>
#include <asm/io.h>

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/si4710.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>

#include <asm/arch/regs-dyn.h>
#include <asm-arm/arch-s3c2410/hardware.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/tomtomgo-irq.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>


/* UNDEFINE THIS IF YOU WANT FMPOWER TO DISAPPEAR IF THE FMTRANSMITTER IS DISABLED!!! */
#define I_DONT_WANT_FM_POWER_GONE 1
#define BARCELONA_DEBUG	__FILE__
#define PFX "SI4710: "
#define PK_DBG PK_DBG_FUNC
#include <barcelona/debug.h>

/*
#undef PK_DBG
#define PK_DBG(format, args...)	printk("si4710: " format, ##args)
*/

struct si4710_tuneparam
{
	unsigned short		tunefreq;
	unsigned short		rfdbuv;
	unsigned char		antcap;
	unsigned char		rnl;
};

struct si4710_property
{
	const unsigned short	property;
	unsigned short		value;
};

struct si4710dev {
	struct cdev cdev;
	struct i2c_client* client;
};

/* Forward declarations */
static struct i2c_client client_template;

/* Spec says SI4710 sits on address 0x11 */
static unsigned short normal_i2c[] = { SI4710_I2C_SLAVE_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static gopin_t 	fmX_power_pin;
static gopin_t 	fmX_clock_pin;
static gopin_t 	fmX_reset_pin;
static dev_t	fmX_devno;

static int si4710_attach(struct i2c_adapter* adapter);
static int si4710_detach(struct i2c_client* client);
static int si4710_i2c_probe(struct device *dev);

#ifdef CONFIG_PM
static int si4710_i2c_suspend(struct device * dev, pm_message_t state, u32 level);
static int si4710_i2c_resume(struct device * dev, u32 level);
#else
#define si4710_i2c_suspend	(NULL)
#define si4710_i2c_resume 	(NULL)
#endif /* CONFIG_PM	*/
static int si4710_powersuspend( struct i2c_client *client );
static int si4710_powerresume( struct i2c_client *client );


static struct i2c_driver driver = {
	.id			= I2C_DRIVERID_SI4710,
	.owner			= THIS_MODULE,
	.name			= "si4710_i2c",
	.flags			= I2C_DF_NOTIFY,
	.attach_adapter	= si4710_attach,
	.detach_client	= si4710_detach,
	.driver = {
		.probe		= si4710_i2c_probe,
		
		.suspend	= si4710_i2c_suspend,
		.resume		= si4710_i2c_resume,
		.name		= SI4710_DEVNAME,
		.owner		= THIS_MODULE,
		.bus		= &platform_bus_type,
	},
};

static struct i2c_client client_template = {
        .name 			= "si4710_i2c_client",
        .flags			= I2C_CLIENT_ALLOW_USE,
        .usage_count	= 0,
        .driver			= &driver,
        .addr			= SI4710_I2C_SLAVE_ADDR,
};

/* Big endian short representation of frequency */
static enum fmtrx_state gFMState = fmtrx_off;

static struct si4710_tuneparam	suspend_tuneparam;
/* Note that Property 0x0103 is not in the list. This is because it's a digital property */
/* that seems to cause problems if get_property is used on it while in analog mode. */
static struct si4710_property	suspend_property[]=
{{0x0001, 0x0000}, {0x0002, 0x0000},
 {0x0101, 0x0000}, {0x0201, 0x0000},
 {0x0202, 0x0000}, {0x2100, 0x0000},
 {0x2101, 0x0000}, {0x2102, 0x0000},
 {0x2103, 0x0000}, {0x2104, 0x0000},
 {0x2105, 0x0000}, {0x2106, 0x0000},
 {0x2107, 0x0000}, {0x2200, 0x0000},
 {0x2201, 0x0000}, {0x2202, 0x0000},
 {0x2203, 0x0000}, {0x2204, 0x0000},
 {0x2300, 0x0000}, {0x2301, 0x0000},
 {0x2302, 0x0000}, {0x2303, 0x0000},
 {0x2304, 0x0000}, {0xFFFF, 0xFFFF}};

static int si4710_save_properties( struct i2c_client *client, struct si4710_property *properties )
{
	int		count=0;
	unsigned char	getprop[4]={0x13, 0x00, 0x00, 0x00};
	unsigned char	recvbuf[8];
	int		rc;
	int		retry=5;

	/* Get all entries in the array. */
	while( properties[count].property != 0xFFFF )
	{
		getprop[2]=((unsigned char) ((properties[count].property & 0xFF00) >> 8));
		getprop[3]=((unsigned char) (properties[count].property & 0x00FF));

		retry=5;
		do
		{
			retry--;
			rc=i2c_master_send( client, getprop, sizeof( getprop ) );
			if( rc < 0 ) return rc;
			mdelay( 1 );

			rc=i2c_master_recv( client, recvbuf, sizeof( recvbuf ) );
			if( rc < 0 ) return rc;
			mdelay( 1 );
		}
		while( ((recvbuf[0] & 0xC0) != 0x80) && (retry >= 0) );

		if( (retry < 0) && ((recvbuf[0] & 0xC0) != 0x80) ) return -EINVAL;

		properties[count].value=(((unsigned short int) recvbuf[2]) << 8) | ((unsigned short int) recvbuf[3]);
		count+=1;
	}

	/* Done. */
	return 0;
}

static int si4710_restore_properties( struct i2c_client *client, struct si4710_property *properties )
{
	int		count=0;
	unsigned char	setprop[6]={0x12, 0x00, 0x00, 0x00, 0x00, 0x00};
	int		rc;

	/* Set all entries in the array. */
	while( properties[count].property != 0xFFFF )
	{
		setprop[2]=((unsigned char) ((properties[count].property & 0xFF00) >> 8));
		setprop[3]=((unsigned char) (properties[count].property & 0x00FF));
		setprop[4]=((unsigned char) ((properties[count].value & 0xFF00) >> 8));
		setprop[5]=((unsigned char) (properties[count].value & 0x00FF));

		rc=i2c_master_send( client, setprop, sizeof( setprop ) );
		if( rc < 0 ) return rc;

		count+=1;
	}

	/* Done. */
	return 0;
}

static int si4710_save_tuneparam( struct i2c_client *client, struct si4710_tuneparam *tuneparam )
{
	int		rc;
	unsigned char	recvbuf[8];
	unsigned char	txtunestatus[2]={0x33, 0x00};
	int		retry=5;

	/* Get the frequency params. */	
	do
	{
		retry--;
		rc=i2c_master_send( client, txtunestatus, sizeof( txtunestatus ) );
		if( rc < 0 ) return rc;

		rc=i2c_master_recv( client, recvbuf, sizeof( recvbuf ) );
		if( rc < 0 ) return rc;
	}
	while( ((recvbuf[0] & 0xC0) != 0x80) && (retry >= 0) );

	if( (retry < 0) && ((recvbuf[0] & 0xC0) != 0x80) ) return -EINVAL;

	/* Save them. */
	tuneparam->tunefreq=(((unsigned short) recvbuf[2]) << 8) | ((unsigned short) recvbuf[3]);
	tuneparam->rfdbuv=(((unsigned short) recvbuf[4]) << 8) | ((unsigned short) recvbuf[5]);
	tuneparam->antcap=recvbuf[6];
	tuneparam->rnl=recvbuf[7];
	return 0;
}

static int si4710_restore_tuneparam( struct i2c_client *client, struct si4710_tuneparam *tuneparam )
{
	int		rc;
	unsigned char	txtunefreq[4]={0x33, 0x00, 0x00, 0x00};
	unsigned char	txtunepwr[5]={0x31, 0x00, 0x00, 0x00, 0x00};

	/* Set the frequency params. */	
	txtunefreq[2]=((unsigned char) ((tuneparam->tunefreq & 0xFF00) >> 8));
	txtunefreq[3]=((unsigned char) (tuneparam->tunefreq & 0x00FF));
	txtunepwr[2]=((unsigned char) ((tuneparam->rfdbuv & 0xFF00) >> 8));
	txtunepwr[3]=((unsigned char) (tuneparam->rfdbuv & 0x00FF));
	txtunepwr[4]=0; /* auto */

	rc=i2c_master_send( client, txtunefreq, sizeof( txtunefreq ) );
	if( rc < 0 ) return rc;

	PK_DBG("%s writing txtunepwr %02x %02x %02x %02x %02x\n", __func__, txtunepwr[0], txtunepwr[1], txtunepwr[2], txtunepwr[3], txtunepwr[4]);
	rc=i2c_master_send( client, txtunepwr, sizeof( txtunepwr ) );
	if( rc < 0 ) return rc;

	/* Done. */
	return 0;
}

static int
si4710_powerdown( struct i2c_client *c)
{
	int rc = -ENOTTY;
	unsigned char pwrdown[] = { 0x11 };
	unsigned char settunepwr[] ={ 0x31, 0x00, 0x00, 0x00, 0x00 };

	PK_DBG("%s writing txtunepwr %02x %02x %02x %02x %02x\n", __func__, settunepwr[0], settunepwr[1], settunepwr[2], settunepwr[3], settunepwr[4]);
	rc = i2c_master_send( c, settunepwr, sizeof(settunepwr) );
	mdelay(10);

	switch (IO_GetFMTransmitterType()) {
		case GOFM_SI4710:

			IOP_Deactivate( fmX_clock_pin );
			mdelay(10);
			break;

		case GOFM_SI4711:

			/* SI4711 CLOCKOUT1 disable */
			IO_Deactivate(FM_RTC);
			break;			

		default:
			printk("SI4710(%s): Error: FM Transmitter chip type is not set! \n", __FUNCTION__);
			return -1;
	}

	rc = i2c_master_send( c, pwrdown, sizeof(pwrdown) );
	mdelay(10);
	
	IOP_Activate( fmX_reset_pin );
	mdelay(10);
	
	switch (IO_GetFMTransmitterType()) {
		case GOFM_SI4710:

#ifndef I_DONT_WANT_FM_POWER_GONE
			IOP_Deactivate( fmX_power_pin );
#endif
			break;

		case GOFM_SI4711:

			/* SI4711: no power pin */
			break;			

		default:
			printk("SI4710(%s): Error: FM Transmitter chip type is not set! \n", __FUNCTION__);
			return -1;
	}

	gFMState = fmtrx_off;
	return rc;
}

static int
si4710_settunepwr( struct i2c_client *c, unsigned char power )
{
	int rc;
	unsigned char settunepwr[]	= { 0x31, 0x00, 0x00, power, 0x00 };
	
	PK_DBG("%s writing txtunepwr %02x %02x %02x %02x %02x\n", __func__, settunepwr[0], settunepwr[1], settunepwr[2], settunepwr[3], settunepwr[4]);
	rc = i2c_master_send( c, settunepwr, sizeof(settunepwr) );
	mdelay(50);
	
	return rc;	
}

static int
si4710_powerup( struct i2c_client *c)
{
	int rc = -ENOTTY;
	unsigned char pwrup[]		= { 0x01, 0xc2, 0x50 };
	unsigned char setfreq[]		= { 0x30, 0x00, 0x27, 0x56 };
	unsigned char getstate[]	= { 0x33, 0x01 };
	unsigned char buf[8];
	struct si4710_property		startup_property[]=
					{{0x2100, 0x0003}, {0x2101, 6825}, {0x2102, 675},
					 {0x2104, ((0x3 << 12) | (600 << 0))}, {0x2301, -15},
					 {0x2300, 0x0001}, {0x2302, 5},
					 {0x2204, 20},
/* Undefine this if you want the Audio Dynamic Range Control feature default enabled. */
#if 1
					 {2200, 0x0003},
#endif
					 {0xFFFF, 0xFFFF}};
	unsigned int reg;

	PK_DBG("----------------------%s----------- set freq=[%x][%x]\n", __func__, setfreq[2], setfreq[3]);
	
	IOP_Activate( fmX_reset_pin );	/* enable reset!	*/
#ifndef I_DONT_WANT_FM_POWER_GONE
	udelay( 100 );
	if (fmX_power_pin) IOP_Activate( fmX_power_pin );
#endif
	udelay( 500 );	// 250 will do according to AN
	IOP_Deactivate( fmX_reset_pin );
	udelay( 500 );
	
	switch (IO_GetFMTransmitterType()) {
		case GOFM_SI4710:

			IOP_Activate( fmX_clock_pin );
			mdelay(1);
			break;

		case GOFM_SI4711:

			/* SI4711: set CLKOUT1 source to RTC for the FM transmitter */
			PK_DBG("%s: switching source to RTC on CLOCKOUT1\n", __FUNCTION__);
			IO_SetFunction(FM_RTC);
			reg = __raw_readl(s3c24xx_misccr);
			reg &= ~(7 << 8);
			reg |= S3C2443_MISCCR_CLK1_RTC;
			__raw_writel(reg, s3c24xx_misccr);
			mdelay(1);
			PK_DBG("%s: switching source to RTC on CLOCKOUT1: Done\n", __FUNCTION__);
			break;			

		default:
			printk("SI4710(%s): Error: FM Transmitter chip type is not set! \n", __FUNCTION__);
			return -1;
	}

	rc= i2c_master_send( c, pwrup, sizeof( pwrup) );
	mdelay(50);

	rc = si4710_settunepwr(c, 115);

	rc= i2c_master_send( c, setfreq, sizeof(setfreq) );
	mdelay(100);

	rc= i2c_master_send( c, getstate, sizeof(getstate) );
	rc= i2c_master_recv( c, buf, sizeof(buf) );
	PK_DBG("state=[0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x] \n", buf[0], buf[1],  buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	rc=si4710_restore_properties( c, startup_property );

	gFMState= fmtrx_on;
	return rc;
}

static int si4710_powersuspend( struct i2c_client *client )
{
	int	rc=0;

	if( gFMState == fmtrx_on )
	{
		rc=si4710_save_properties( client, suspend_property );
		if( rc < 0 ) return rc;

		rc=si4710_save_tuneparam( client, &suspend_tuneparam );
		if( rc < 0 ) return rc;

		rc=si4710_powerdown( client );

		gFMState=fmtrx_suspend;
	}
	return rc;
}

static int si4710_powerresume( struct i2c_client *client )
{
	int	rc=0;

	if( gFMState == fmtrx_suspend )
	{
		rc=si4710_powerup( client );
		if( rc < 0 ) return rc;

		rc=si4710_restore_tuneparam( client, &suspend_tuneparam );
		if( rc < 0 ) return rc;

		rc=si4710_restore_properties( client, suspend_property );
	}
	return rc;
}

static int
si4710_SetFreq( struct i2c_client *c, unsigned int u32Freq)
{
	int rc = 0;
	unsigned char setfreq[] = { 0x30, 0x00, 0x27, 0xa6};

	setfreq[2] = (unsigned char)(((u32Freq/10000)&0xff00)>>8);
	setfreq[3] = (unsigned char)((u32Freq/10000)&0x00ff);
	
	if( fmtrx_on == gFMState) {
		rc = i2c_master_send( c, setfreq, sizeof(setfreq) );
	}
	
	return rc;
}

static int
si4710_i2c_probe(struct device *dev)
{	
	struct fm_transmitter_info *pdata = dev->platform_data;
	int ret;

	fmX_devno = pdata->device_nr;
	
	fmX_clock_pin = pdata->fm_clock_pin;
	fmX_power_pin = pdata->fm_power_pin;
	fmX_reset_pin = pdata->fm_reset_pin;	

	
	if (fmX_clock_pin) IOP_Deactivate( fmX_clock_pin );
	
	switch (IO_GetFMTransmitterType()) {
		case GOFM_SI4710:

#ifndef I_DONT_WANT_FM_POWER_GONE
			IOP_Deactivate( fmX_power_pin );
#else
			IOP_Activate( fmX_power_pin );
			udelay( 500 );	// 250 will do according to AN
#endif
			break;

		case GOFM_SI4711:

			/* SI4711: no power pin */
			break;			

		default:
			printk("SI4710(%s): Error: FM Transmitter chip type is not set! \n", __FUNCTION__);
			return -1;
	}

	ret = i2c_add_driver(&driver);
	printk("----------------------%s------------------- DONE \n", __func__);
	return ret;
}

#ifdef CONFIG_PM
static int
si4710_i2c_suspend(struct device * dev, pm_message_t state, u32 level)
{
	struct i2c_client *c = NULL;
	int rc = 0;

	if (level != SUSPEND_POWER_DOWN)
		return 0;

	c=i2c_get_client(I2C_DRIVERID_SI4710, 0, NULL);
	if( c == NULL ) {
		dev_err(dev, "Can't find I2C driver!\n");
		rc = -ENODEV;
	}

	PK_DBG("----------------------%s----------------pm_msg=%d, level=%d\n", __func__, state, level);
	if (!rc)
		rc = si4710_powersuspend( c);
	if( rc ){
		dev_err(dev, "error while doing powersuspend: rc = %d\n", rc);
	}

	/* Never return an error here. It will block the kernel from going into suspend further
	 * and the FM transmitter is not important enough to block that!
	 */
	//return rc;
	return 0;
}

static int
si4710_i2c_resume(struct device * dev, u32 level)
{
	struct i2c_client *c=NULL;
	int rc = 0;

	if (level != RESUME_POWER_ON)
		return 0;

	c=i2c_get_client(I2C_DRIVERID_SI4710, 0, NULL);
	if( c == NULL )
	{
		dev_err(dev, "Can't find I2C driver!\n");
		rc = -ENODEV;
	}

	PK_DBG("----------------------%s----------------level=%d\n", __func__, level);
	if (!rc)
		rc = si4710_powerresume( c);
	if( rc ){
		dev_err(dev, "error while doing powerresume: rc = %d\n", rc);
	}

	return 0;
}
#endif /* CONFIG_PM	*/

static int
si4710_open(struct inode* nodep, struct file* filep)
{
	struct si4710dev *dev = container_of(nodep->i_cdev, struct si4710dev, cdev);

	PK_DBG("----------------------%s-------------------\n", __func__);
	filep->private_data = dev; /* for easy reference */

	return 0;
}

static int
si4710_release(struct inode* nodep, struct file* filep)
{
	PK_DBG("----------------------%s-------------------\n", __func__);
	if (filep && filep->private_data) {
		filep->private_data = NULL;
	}
	return 0;
}

static int
si4710_ioctl(struct inode* nodep, struct file* filep, unsigned int cmd, unsigned long arg)
{
	struct si4710dev	*dev = filep ? filep->private_data : NULL;
	struct si4710_property	dynrange_property_on[]=
				{{0x2201, 0xFFD8}, {0x2202, 0x0000}, {0x2203, 4}, {0x2200, 0x0001},
				 {0xFFFF, 0xFFFF}};
	struct si4710_property	dynrange_property_off[]={{0x2200, 0x0000}, {0xFFFF, 0xFFFF}};
	struct si4710_property	stereo_property_on[]={{0x2100, 0x0003}, {0xFFFF}};
	struct si4710_property	stereo_property_off[]={{0x2100, 0x0000}, {0xFFFF}};
	struct si4710_property	silent_mode_property_on[]={{0x2300, 0x0001}, {0xFFFF}};
	struct si4710_property	silent_mode_property_off[]={{0x2300, 0x0000}, {0xFFFF}};
	struct si4710_property	adrc_property_on[]={{0x2200, 0x0003}, {0xFFFF}};
	struct si4710_property	adrc_property_off[]={{0x2200, 0x0002}, {0xFFFF}};

	int rc = -ENOTTY;

	PK_DBG("----------------------%s----------- cmd=0x%x\n", __func__, cmd);
	if (_IOC_TYPE(cmd) != FMTRANSMITTER_DRIVER_MAGIC){
			PK_ERR("Wrong IOC type! Failing command");
			return -ENOTTY;
	}

	switch( cmd) {
    		case IOW_SET_FM_FREQUENCY:
			{
				unsigned int u32Freq = (unsigned int) arg;
				PK_DBG("------%s----cmd=set frequency %d\n", __func__, (unsigned int) arg);
				rc = si4710_SetFreq( dev->client, u32Freq);
	   		}
			break;
		case IOW_ENABLE:
			{
				unsigned char bEnable = (unsigned char) arg;
				PK_DBG("------%s----cmd=enable/disable=%d\n", __func__, (unsigned int) arg);
				if( bEnable) {
					rc = si4710_powerresume( dev->client);
				} else {
					rc = si4710_powersuspend( dev->client);
				}
			}
			break;

		case IOW_FMTRX_SET_MONO_STEREO:
			{
				unsigned char bEnable = (unsigned char) arg;
				if( bEnable )
					rc=si4710_restore_properties( dev->client,
								      stereo_property_on );
				else
					rc=si4710_restore_properties( dev->client,
								      stereo_property_off );
			}
			break;

		case IOW_FMTRX_TADRC_ENABLE:
			{
				unsigned char bEnable = (unsigned char) arg;
				if( bEnable )
					rc=si4710_restore_properties( dev->client,
								      dynrange_property_on );
				else
					rc=si4710_restore_properties( dev->client,
								      dynrange_property_off );
			}
			break;

		case IOW_FMTRX_SILENT_MODE_ENABLE:
			{
				unsigned char bEnable = (unsigned char) arg;
				if( bEnable )
					rc=si4710_restore_properties( dev->client,
								      silent_mode_property_on );
				else
					rc=si4710_restore_properties( dev->client,
								      silent_mode_property_off );
			}
			break;
		case IOR_FM_GET_STATE:
			/* Save the state. */
			*((int *) arg)=gFMState;
			break;
		
		case IOW_FMTRX_SET_ADRC:
			{
				/* Enable the ADRC. */
				unsigned char bEnable = (unsigned char) arg;
				if( bEnable )
					rc=si4710_restore_properties( dev->client,
								      adrc_property_on );
				else
					rc=si4710_restore_properties( dev->client,
								      adrc_property_off );
			}
			break;

		case IOW_FMTRX_SET_POWER:
			{
				/* Set power mode. */
				unsigned char power = (unsigned char)arg;
				if ((power >= 88) || (power <= 120)) {
					rc = si4710_settunepwr(dev->client, power);  
					PK_DBG("------%s----cmd=powermode=%d %s\n", __func__, power, (rc < 0) ? "FAILED" : "OK");
				}
			}
			break;


#ifdef SI4710_SW_TEST
		case IOW_SUSPEND:
			rc = si4710_powersuspend( dev->client);
			break;
		case IOW_RESUME:
			rc = si4710_powerresume( dev->client);
			break;
#endif /*  SI4710_SW_TEST	*/
		default:
			break;
    	}
	return rc;
}

static unsigned int
si4710_poll(struct file *filep, struct poll_table_struct *pq)
{
	// FM transmitter can neither be written nor read, 
	// the poll() routine should always indicate 
	// that it is neither possible to read nor to write to the driver.
	PK_DBG("----------------------%s-------------------\n", __func__);
	return 0;
}

struct file_operations si4710_fops = {
        .owner		= THIS_MODULE,
        .open		= si4710_open,
        .release	= si4710_release,
        .ioctl		= si4710_ioctl,
        .poll		= si4710_poll,
};


static int
si4710_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct si4710dev *dev;
	struct i2c_client *c;
	int rc = 0;

	PK_DBG("----------------------%s-------------------\n", __func__);
	c = kmalloc(sizeof *c, GFP_KERNEL);
	if (!c) {
		rc = -ENOMEM;
		goto no_client;
	}
	memcpy(c, &client_template, sizeof *c);
	c->adapter = adapter;
	strcpy(c->name, /*"Silicon Labs 4710"*/SI4710_DEVNAME);

	dev = kmalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		rc = -ENOMEM;
		goto no_dev;
	}
	memset(dev, 0, sizeof *dev);
	i2c_set_clientdata(c, dev);

	i2c_attach_client(c);

	i2c_use_client(c);

	dev->client = c;
	
	/* Now setup our character device to the fm */
	cdev_init( &dev->cdev, &si4710_fops );
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &si4710_fops;
	if( (rc = cdev_add(&dev->cdev, fmX_devno, 1) ) )
		goto no_cdev;

	PK_DBG("%s: %p\n", __func__, c);

	rc=si4710_powerup( c );
	if( rc < 0 ) goto no_cdev;

	return si4710_powersuspend( c );

no_cdev:
	i2c_detach_client(c);
no_dev:
	kfree(c);
no_client:
	PK_ERR("%s: ERROR %d!\n", __func__, rc);

	return rc;

}

/* ----------------------------------------------------------------------- */

static int
si4710_attach(struct i2c_adapter *adap)
{
	int ret= 0;
	PK_DBG("----------------------%s-------------------\n", __func__);

	IOP_Activate( fmX_reset_pin ); /* First power then release reset	*/
	udelay( 500 );

	switch (IO_GetFMTransmitterType()) {
		case GOFM_SI4710:

			IOP_Activate( fmX_power_pin );
			udelay( 500 );
			break;

		case GOFM_SI4711:

			/* SI4711: no power pin */
			break;			

		default:
			printk("SI4710(%s): Error: FM Transmitter chip type is not set! \n", __FUNCTION__);
			return -1;
	}

	IOP_Deactivate( fmX_reset_pin );	

	if (fmX_clock_pin) {
		IOP_Activate( fmX_clock_pin ); //Enable Ref. CLK 
	}
	udelay( 100 );

	ret= i2c_probe( adap, &addr_data, si4710_detect_client);
	PK_DBG("id=%d, name=%s, ret=%d \n", adap->id, adap->algo->name, ret);

	if (fmX_clock_pin) {
		IOP_Deactivate( fmX_clock_pin );
	}

	udelay( 100 );
	
	return ret;
}

static int
si4710_detach(struct i2c_client *c)
{
	struct si4710dev* fm = i2c_get_clientdata(c);
	
	PK_DBG("----------------------%s-------------------\n", __func__);
	PK_DBG("%s: %p\n", __func__, c);

	i2c_release_client(c);
	i2c_detach_client(c);

	kfree(fm);
	kfree(c);

	return 0;
}


/* ----------------------------------------------------------------------- */

static int __init 
si4710_init(void)
{
	int result;
	result = driver_register(&driver.driver);
	PK_DBG("------------------%s:result = 0x%02X---------------\n", __func__, result);
	return result;
}

static void __exit
si4710_exit(void)
{
	PK_DBG("----------------------%s-------------------\n", __func__);
	driver_unregister(&driver.driver);
}

MODULE_AUTHOR("Ithamar Adema <ithamar.adema@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected Silicon Labs 4710 FM Transmitter");

module_init(si4710_init);
module_exit(si4710_exit);

