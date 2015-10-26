/* si4705.c
 *
 * Control driver for Silicon Lab 4703 chip.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Xander Hover <Xander.Hover@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/fmreceiver.h>
#include <linux/si4705.h>
#include <linux/device.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>
#include <asm/semaphore.h>
#include <linux/interrupt.h>
#include <asm/irq.h>

#define PFX "SI4705: "
#define PK_DBG(fmt, arg...)    printk(KERN_DEBUG PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_ERR(fmt, arg...)    printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_ENTRY	PK_DBG("----------------------%s-------------------\n", __func__)


#ifndef __bigendian
#    define __bigendian
#endif				/* __bigendian */

#define TUNER_DELAY 2

#warning SI4705 is being compiled!

struct si4705_property_type {
	u16 property;
	u16 value;
} __attribute__ ((__packed__));

/* command types are used to xfer over i2c using the i2c_master_send/recv functions */
struct si4705_set_property_cmd_type {
	u16 __bigendian cmd;
	u16 __bigendian property;
	u16 __bigendian value;
} __attribute__ ((__packed__));

struct si4705_get_property_cmd_type {
	u16 __bigendian cmd;
	u16 __bigendian property;
} __attribute__ ((__packed__));

struct si4705_tune_cmd_type {
	u16 __bigendian tunecmd;
	u16 __bigendian tunefreq;
	u8 antenna;
} __attribute__ ((__packed__));

#if 0
static struct si4705_property_type property_cpu_to_be(struct
						      si4705_property_type
						      t)
{
	t.property = cpu_to_be16(t.property);
	t.value = cpu_to_be16(t.value);
	return t;
}
#endif

/* SI4705 defined commands */
#define CMD_POWER_UP(a,b)		{ (0x01), (a), (b) }	/* a,b:: u8 */
#define CMD_GET_REV			{ (0x10) }
#define CMD_POWER_DOWN       		{ (0x11) }
#define CMD_SET_PROPERTY(a, b)  	{ cpu_to_be16(0x1200), cpu_to_be16(a), cpu_to_be16(b) }	/* a,b:: u16 */
#define CMD_GET_PROPERTY(a)		{ cpu_to_be16(0x1300), cpu_to_be16(a) }	/* a:: u16 (one of PROP_XXXX) */
#define CMD_GET_INT_STATUS		{ (0x14) }
#define CMD_FM_TUNE_FREQ(a,b)		{ cpu_to_be16(0x2000), cpu_to_be16(a), (b) }	/* a:: u16, b:: u8 */
#define CMD_FM_SEEK_START(a)		{ (0x21), (a) }	/* a:: u8 */
#define CMD_FM_TUNE_STATUS(a)		{ (0x22), (a) }	/* a:: u8 */
#define CMD_FM_RSQ_STATUS(a)		{ (0x23), (a) }	/* a:: u8 */
#define CMD_FM_RDS_STATUS(a)		{ (0x24), (a) }	/* a:: u8, si4705/06/21/31/35/37/39/49 only  */
#define CMD_FM_AGC_STATUS		{ (0x27) }
#define CMD_FM_AGC_OVERRIDE(a,b)	{ (0x28), (a), (b) }	/* a,b:: u8 */
#define CMD_GPIO_CTL(a)			{ (0x80), (a) }	/* a:: u8 */
#define CMD_GPIO_SET(a)			{ (0x81), (a) }	/* a:: u8 */

#define SET_PROPERTY_CMD		(0x1200)
#define GET_PROPERTY_CMD		(0x1300)

#if 0
static struct si4705_set_property_cmd_type to_set_property_cmd(struct
							       si4705_property_type
							       t)
{
	return (struct si4705_set_property_cmd_type) CMD_SET_PROPERTY(t.
								      property,
								      t.
								      value);
}

static struct si4705_get_property_cmd_type to_get_property_cmd(u16 prop)
{
	return (struct si4705_get_property_cmd_type)
	    CMD_GET_PROPERTY(prop);
}
#endif

/* SI4705 defined properties in cpu endianness */
/* macros below are used to initialize properties for the si4705_{get,set}_properties() functions */
#define S_PROP_GPIO_IEN(d) 			 { (0x0001), (d) }	/* d:: u16 */
#define S_PROP_DIGITAL_OUTPUT_FORMAT(d)		 { (0x0102), (d) }	/* d:: u16, si4705/21/31/35/37/39 only */
#define S_PROP_DIGITAL_OUTPUT_SAMPLE_RATE(d)	 { (0x0104), (d) }	/* d:: u16, si4705/21/31/35/37/39 only */
#define S_PROP_REFCLK_FREQ(d)			 { (0x0201), (d) }	/* d:: u16 */
#define S_PROP_REFCLK_PRESCALE(d)		 { (0x0202), (d) }	/* d:: u16 */

#define S_PROP_FM_DEEMPHASIS(d)			 { (0x1100), (d) }	/* d:: u16, not in si4706/49 */
#define S_PROP_FM_BLEND_STEREO_THRESHOLD(d)	 { (0x1105), (d) }	/* d:: u16 */
#define S_PROP_FM_BLEND_MONO_THRESHOLD(d)	 { (0x1106), (d) }	/* d:: u16 */
#define S_PROP_FM_ANTENNA_INPUT(d)		 { (0x1107), (d) }	/* d:: u16 */
#define S_PROP_FM_MAX_TUNE_ERROR(d)		 { (0x1108), (d) }	/* d:: u16 */

#define S_PROP_FM_RSQ_INT_SOURCE(d)		 { (0x1200), (d) }	/* d:: u16 */
#define S_PROP_FM_RSQ_SNR_HI_THRESHOLD(d)	 { (0x1201), (d) }	/* d:: u16 */
#define S_PROP_FM_RSQ_SNR_LO_THRESHOLD(d)	 { (0x1202), (d) }	/* d:: u16 */
#define S_PROP_FM_RSQ_RSSI_HI_THRESHOLD(d)	 { (0x1203), (d) }	/* d:: u16 */
#define S_PROP_FM_RSQ_RSSI_LO_THRESHOLD(d)	 { (0x1204), (d) }	/* d:: u16 */
#define S_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD(d)	 { (0x1205), (d) }	/* d:: u16, si4749 only */
#define S_PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD(d)	 { (0x1206), (d) }	/* d:: u16, si4749 only */
#define S_PROP_RSQ_BLEND_THRESHOLD(d)		 { (0x1207), (d) }	/* d:: u16 */

#define S_PROP_FM_SOFT_MUTE_RATE(d)		 { (0x1300), (d) }	/* d:: u16 */
#define S_PROP_FM_SOFT_MUTE_MAX_ATTENUATION(d)	 { (0x1302), (d) }	/* d:: u16, not in si4706/49 */
#define S_PROP_FM_SOFT_MUTE_SNR_THRESHOLD(d)	 { (0x1303), (d) }	/* d:: u16, not in si4706/49 */

#define S_PROP_SEEK_BAND_BOTTOM(d)		 { (0x1400), (d) }	/* d:: u16 */
#define S_PROP_SEEK_BAND_TOP(d)			 { (0x1401), (d) }	/* d:: u16 */
#define S_PROP_SEEK_FREQ_SPACING(d)		 { (0x1402), (d) }	/* d:: u16 */
#define S_PROP_SEEK_TUNE_SNR_THRESHOLD(d)	 { (0x1403), (d) }	/* d:: u16 */
#define S_PROP_SEEK_TUNE_RSSI_THRESHOLD(d)	 { (0x1404), (d) }	/* d:: u16 */

#define S_PROP_RDS_INT_SOURCE(d)		 { (0x1500), (d) }	/* d:: u16 */
#define S_PROP_RDS_INT_FIFO_COUNT(d)		 { (0x1501), (d) }	/* d:: u16, si4705/06/21/31/35/37/39/49 only */
#define S_PROP_RDS_CONFIG(d)			 { (0x1502), (d) }	/* d:: u16, si4705/06/21/31/35/37/39/49 only */

#define S_PROP_RX_VOLUME(d)			 { (0x4000), (d) }	/* d:: u16, not in si4706/49 */
#define S_PROP_RX_HARD_MUTE(d)			 { (0x4001), (d) }	/* d:: u16, not in si4706/49 */

#define S_PROP_LIST_END				 { (0xFFFF), (0xFFFF) }	/* List terminator, not in any device */

#define PROP_GPIO_IEN 				(0x0001)
#define PROP_DIGITAL_OUTPUT_FORMAT		(0x0102)
#define PROP_DIGITAL_OUTPUT_SAMPLE_RATE		(0x0104)
#define PROP_REFCLK_FREQ			(0x0201)
#define PROP_REFCLK_PRESCALE			(0x0202)

#define PROP_FM_DEEMPHASIS			(0x1100)
#define PROP_FM_BLEND_STEREO_THRESHOLD		(0x1105)
#define PROP_FM_BLEND_MONO_THRESHOLD		(0x1106)
#define PROP_FM_ANTENNA_INPUT			(0x1107)
#define PROP_FM_MAX_TUNE_ERROR			(0x1108)

#define PROP_FM_RSQ_INT_SOURCE			(0x1200)
#define PROP_FM_RSQ_SNR_HI_THRESHOLD		(0x1201)
#define PROP_FM_RSQ_SNR_LO_THRESHOLD		(0x1202)
#define PROP_FM_RSQ_RSSI_HI_THRESHOLD		(0x1203)
#define PROP_FM_RSQ_RSSI_LO_THRESHOLD		(0x1204)
#define PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD	(0x1205)
#define PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD	(0x1206)
#define PROP_RSQ_BLEND_THRESHOLD		(0x1207)

#define PROP_FM_SOFT_MUTE_RATE			(0x1300)
#define PROP_FM_SOFT_MUTE_MAX_ATTENUATION	(0x1302)
#define PROP_FM_SOFT_MUTE_SNR_THRESHOLD		(0x1303)

#define PROP_SEEK_BAND_BOTTOM			(0x1400)
#define PROP_SEEK_BAND_TOP			(0x1401)
#define PROP_SEEK_FREQ_SPACING			(0x1402)
#define PROP_SEEK_TUNE_SNR_THRESHOLD		(0x1403)

#define PROP_RDS_INT_SOURCE			(0x1500)
#define PROP_RDS_INT_FIFO_COUNT			(0x1501)
#define PROP_RDS_CONFIG				(0x1502)

#define PROP_RX_VOLUME				(0x4000)
#define PROP_RX_HARD_MUTE			(0x4001)

#define PROP_END				(0xFFFF)
#define PROP_LIST_END	PROP_END

/* FM receiver state definitions */
#define FM_OFF 		0U
#define FM_SUSPEND 	1U
#define FM_ON  		2U

/* FM receiver status bitsettings */
#define BSTAT_CTS	(0x80)
#define BSTAT_ERR	(0x40)
#define BSTAT_RSQINT	(0x08)
#define BSTAT_RDSINT	(0x04)
#define BSTAT_STCINT	(0x01)

#define WSTAT_CTS	(0x8000)
#define WSTAT_ERR	(0x4000)
#define WSTAT_RSQINT	(0x0800)
#define WSTAT_RDSINT	(0x0400)
#define WSTAT_STCINT	(0x0100)

/* FM receiver POWER_UP Argument bitsettings */
#define POWERUP_CTSIEN  (0x80)
#define POWERUP_GPO2OEN (0x40)
#define POWERUP_PATCH 	(0x20)
#define POWERUP_XOSCEN	(0x10)
#define POWERUP_FUNC_FMRX (0x00)
#define POWERUP_FUNC_QLID (0x0F)
#define POWERUP_OPMODE_ANA_OUT (0x05)
#define POWERUP_OPMODE_DIG_OUT (0xB0)

/* FM receiver FM_SEEK_START argument bitsettings */
#define FM_SEEK_START_SEEKUP (0x08)
#define FM_SEEK_START_WRAP   (0x04)

/* FM receiver FM_TUNE_STATUS argument bitsettings */
#define FM_TUNE_STATUS_CANCEL (0x02)
#define FM_TUNE_STATUS_INTACK (0x01)

/* FM receiver FM_RSQ_STATUS argument bitsettings */
#define FM_RSQ_STATUS_INTACK (0x01)

/* FM receiver FM_RDS_STATUS argument bitsettings */
#define FM_RDS_STATUS_MTFIFO (0x02)
#define FM_RDS_STATUS_INTACK (0x01)

/* FM receiver FM_AGC_OVERRIDE argument bitsettings */
#define FM_AGC_OVERRIDE_RFAGCDIS (0x01)
#define FM_AGC_OVERRIDE_LNA_GAIN_INDEX (0x00)

/* FM receiver GPIO_CTL argument bitsettings */
#define GPIO_CTL_GPO3OEN (0x08)
#define GPIO_CTL_GPO2OEN (0x04)
#define GPIO_CTL_GPO1OEN (0x02)

/* FM receiver GPIO_SET argument settings */
#define GPIO_SET_GPO3LEVEL (0x08)
#define GPIO_SET_GPO2LEVEL (0x04)
#define GPIO_SET_GPO1LEVEL (0x02)


struct si4705dev {
	struct cdev cdev;
	struct i2c_client *client;
};

static struct si4705_property_type si4705_suspend_property[] = {
	S_PROP_GPIO_IEN(0x0000),
	S_PROP_DIGITAL_OUTPUT_FORMAT(0x0000),
	S_PROP_DIGITAL_OUTPUT_SAMPLE_RATE(0x0000),
	S_PROP_REFCLK_FREQ(0x0000),
	S_PROP_REFCLK_PRESCALE(0x0000),
	S_PROP_FM_DEEMPHASIS(0x0000),
	S_PROP_FM_BLEND_STEREO_THRESHOLD(0x0000),
	S_PROP_FM_BLEND_MONO_THRESHOLD(0x0000),
	S_PROP_FM_ANTENNA_INPUT(0x0000),
	S_PROP_FM_MAX_TUNE_ERROR(0x0000),
	S_PROP_FM_RSQ_INT_SOURCE(0x0000),
	S_PROP_FM_RSQ_SNR_HI_THRESHOLD(0x0000),
	S_PROP_FM_RSQ_SNR_LO_THRESHOLD(0x0000),
	S_PROP_FM_RSQ_RSSI_HI_THRESHOLD(0x0000),
	S_PROP_FM_RSQ_RSSI_LO_THRESHOLD(0x0000),
#ifdef SI4749			/* only available in si4749 */
	S_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD(0x0000),
	S_PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD(0x0000),
#endif				/* SI4749 */
	S_PROP_RSQ_BLEND_THRESHOLD(0x0000),
	S_PROP_FM_SOFT_MUTE_RATE(0x0000),
	S_PROP_FM_SOFT_MUTE_MAX_ATTENUATION(0x0000),
	S_PROP_FM_SOFT_MUTE_SNR_THRESHOLD(0x0000),
	S_PROP_SEEK_BAND_BOTTOM(0x0000),
	S_PROP_SEEK_BAND_TOP(0x0000),
	S_PROP_SEEK_FREQ_SPACING(0x0000),
	S_PROP_SEEK_TUNE_SNR_THRESHOLD(0x0000),
	S_PROP_SEEK_TUNE_RSSI_THRESHOLD(0x0000),
	S_PROP_RDS_INT_SOURCE(0x0000),
	S_PROP_RDS_INT_FIFO_COUNT(0x0000),
	S_PROP_RDS_CONFIG(0x0000),
	S_PROP_RX_VOLUME(0x0000),
	S_PROP_RX_HARD_MUTE(0x0000),
	S_PROP_LIST_END,
};

/* there shall only be one user of the device */
static atomic_t si4705_available = ATOMIC_INIT(1);

static int gFMState = FM_OFF;

static unsigned int s_device_id = 0;

/* Forward declarations */
static struct i2c_client client_template;

/* Spec says SI4705 sits on address 0x11 or 0x63 */
static unsigned short normal_i2c[] =
    { SI4705_I2C_SLAVE_ADDR, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static dev_t tmcRX_devno;

static int si4705_attach(struct i2c_adapter *adapter);
static int si4705_detach(struct i2c_client *client);
static int si4705_i2c_probe(struct device *dev);

#ifdef CONFIG_PM
static int si4705_i2c_suspend(struct device *dev, pm_message_t state,
			      u32 level);
static int si4705_i2c_resume(struct device *dev, u32 level);
#else
#    define si4710_i2c_suspend  (NULL)
#    define si4710_i2c_resume   (NULL)
#endif				/* CONFIG_PM     */

static int si4705_powersuspend(struct i2c_client *client);
static int si4705_powerresume(struct i2c_client *client);
static int si4705_powerup(struct i2c_client *c);
static int si4705_powerdown(struct i2c_client *c);

static struct i2c_driver driver = {
	.id = I2C_DRIVERID_SI4705,
	.owner = THIS_MODULE,
	.name = "si4705_i2c",
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = si4705_attach,
	.detach_client = si4705_detach,
	.driver = {
		   .probe = si4705_i2c_probe,
		   .suspend = si4705_i2c_suspend,
		   .resume = si4705_i2c_resume,
		   .name = SI4705_DEVNAME,
		   .owner = THIS_MODULE,
		   .bus = &platform_bus_type,
		   },
};

static struct i2c_client client_template = {
	.name = "si4705_i2c_client",
	.flags = I2C_CLIENT_ALLOW_USE,
	.usage_count = 0,
	.driver = &driver,
	.addr = SI4705_I2C_SLAVE_ADDR,
};

static int si4705_save_properties(struct i2c_client *c,
				  struct si4705_property_type *properties);
static int si4705_restore_properties(struct i2c_client *client, struct si4705_property_type
				     *properties);
#define si4705_set_properties(a,b) si4705_restore_properties(a,b)
#define si4705_get_properties(a,b) si4705_save_properties(a,b)
#if 0
static void si4705_print_properties(struct si4705_property_type *p);
#endif


/* prototypes for tuner commands */
static int si4705_get_freq(struct i2c_client *c, __u16 * freq,
			   __u16 * rssi, __u16 * snr);
static int si4705_set_freq(struct i2c_client *c, __u16 freq);
static int si4705_set_volume(struct i2c_client *c, __u16 vol);
static int si4705_set_band(struct i2c_client *c, __u16 band);

static int si4705_seek(struct i2c_client *c,
		       struct fmr_seek_param_type seek_param);

#if 0 
static int si4705_set_seek_bounds(struct i2c_client *c,
				  struct fmr_seek_bound_type bounds);
static int si4705_set_agc(struct i2c_client *c, struct fmr_agc_type agc);

static int si4705_query_rsq(struct i2c_client *c,
			    struct fmr_rsq_type *rsq);
static int si4705_query_rds(struct i2c_client *c,
			    struct fmr_rds_type *rds);

#endif 
/*--------------------------------------------------------*/

static int si4705_get_freq(struct i2c_client *c, __u16 * freq,
			   __u16 * rssi, __u16 * snr)
{
	/* get tune stat */
	u8 stat[] = CMD_FM_TUNE_STATUS(0x00);
	u8 buf[8];
	u16 *wbuf = (u16 *) buf;
	int i, rc;

	PK_ENTRY;

	memset(buf, 0x0, sizeof(buf));
	rc = i2c_master_send(c, stat, sizeof(stat));
	if (rc < 0)
		return -1;
	mdelay(TUNER_DELAY);
	for (i = 0; i < 5; i++) {
		rc = i2c_master_recv(c, buf, sizeof(buf));
		if ((buf[0] & 0xC0) == 0x80)
			break;
	}
	if ((buf[0] & 0xC0) != 0x80)
		return -1;
	*freq = be16_to_cpu(wbuf[1]);
	*rssi = buf[4];
	*snr = buf[5];

	PK_DBG
	    ("FM_TUNE_STAT: Freq = %d, RSSI = %d, SNR = %d, MULT = %d, ReadAntennaCap = %d\n",
	     *freq, *rssi, *snr, buf[6], buf[7]);
	return 0;
}

static int si4705_set_freq(struct i2c_client *c, __u16 freq)
{
	/* tune to freq */
	struct si4705_tune_cmd_type tune_cmd = CMD_FM_TUNE_FREQ(freq, 0U);
	PK_ENTRY;
	if (i2c_master_send
	    (c, (u8 *) & tune_cmd, sizeof(struct si4705_tune_cmd_type)))
		return 0;
	else
		return -1;
}

static int si4705_set_volume(struct i2c_client *c, __u16 vol)
{
	struct si4705_set_property_cmd_type vol_cmd =
	    CMD_SET_PROPERTY(PROP_RX_VOLUME, (4 * vol));
	PK_ENTRY;
	if (i2c_master_send
	    (c, (u8 *) & vol_cmd,
	     sizeof(struct si4705_set_property_cmd_type)))
		return 0;
	else
		return 1;
}

/*
 * band: 0: 87.5-108, 1: 76-108, 2: 76-90 , spacing allways 100 kHz.
 */
#define band_prop(b,t,s) \
		struct si4705_property_type band_prop_array[] = \
		{ \
			S_PROP_SEEK_BAND_BOTTOM(b), \
			S_PROP_SEEK_BAND_TOP(t), \
			S_PROP_SEEK_FREQ_SPACING(s), \
			S_PROP_LIST_END, \
		} \

static int si4705_set_band(struct i2c_client *c, __u16 band)
{
	PK_ENTRY;
	{
		u16 b, t, s;
		switch (band) {
		default:
		case 0:
			b = 8750U;
			t = 10800U;
			s = 10U;
			break;
		case 1:
			b = 7600U;
			t = 10800U;
			s = 10U;
			break;
		case 2:
			b = 7600U;
			t = 9000U;
			s = 10U;
			break;
		}
		{
			band_prop(b, t, s);
			if (si4705_set_properties(c, band_prop_array))
				return -1;
		}
	}
	return 0;
}

static int si4705_seek(struct i2c_client *c,
		       struct fmr_seek_param_type seek_param)
{
	PK_ENTRY;
	{
		u8 seek_arg = (seek_param.up << 3) & 0x08;
		seek_arg |= (seek_param.wrap << 2) & 0x04;
		{
			u8 seek_cmd[] = CMD_FM_SEEK_START(seek_arg);
			int rc = 0;
			rc = i2c_master_send(c, seek_cmd,
					     sizeof(seek_cmd));
			if (rc < 0)
				return -1;
		}
	}
	return 0;
}

#if 0
static int si4705_set_seek_bounds(struct i2c_client *c,
				  struct fmr_seek_bound_type bounds)
{
	PK_ENTRY;
	{
		band_prop(bounds.bottom, bounds.top, bounds.spacing);
		if (si4705_set_properties(c, band_prop_array))
			return -1;
	}
	return 0;
}

static int si4705_set_agc(struct i2c_client *c, struct fmr_agc_type agc)
{
	PK_ENTRY;
	{
		u8 agc_cmd[] = CMD_FM_AGC_OVERRIDE((agc.rfagcdis & 0x01),
						   (agc.
						    lna_gain_index &
						    0x1F));
		if (i2c_master_send(c, (u8 *) & agc_cmd, sizeof(agc_cmd)))
			return 0;
	}
	return 1;
}

static int si4705_query_rsq(struct i2c_client *c, struct fmr_rsq_type *rsq)
{
	PK_ENTRY;
	{
		u8 rsq_cmd[] = CMD_FM_RSQ_STATUS(0x01);
		u8 buf[8];
		int rc = 0;
		int i = 0;
		rc = i2c_master_send(c, (u8 *) & rsq_cmd, sizeof(rsq_cmd));
		if (rc < 0)
			return -1;
		mdelay(TUNER_DELAY);
		for (i = 0; i < 5; i++) {
			rc = i2c_master_recv(c, buf, sizeof(buf));
			if ((buf[0] & 0xC0) == 0x80)
				break;
		}
		if ((buf[0] & 0xC0) != 0x80)
			return -1;

		rsq->blendint = (buf[1] >> 7) & 0x01;
		rsq->multhint = (buf[1] >> 5) & 0x01;
		rsq->multlint = (buf[1] >> 4) & 0x01;
		rsq->snrhint = (buf[1] >> 3) & 0x01;
		rsq->snrlint = (buf[1] >> 2) & 0x01;
		rsq->rssihint = (buf[1] >> 1) & 0x01;
		rsq->rssilint = buf[1] & 0x01;
		rsq->smute = (buf[2] >> 3) & 0x01;
		rsq->afcrl = (buf[2] >> 1) & 0x01;
		rsq->valid = buf[2] & 0x01;
		rsq->pilot = (buf[3] >> 7) & 0x01;
		rsq->stblend = buf[3] & 0x7F;
		rsq->rssi = buf[4];
		rsq->snr = buf[5];
		rsq->multi = buf[6];
		rsq->freqoff = buf[7];
	}

	return 0;
}

static int si4705_query_rds(struct i2c_client *c, struct fmr_rds_type *rds)
{
	PK_ENTRY;
	{
		u8 rds_cmd[] = CMD_FM_RDS_STATUS(0x01);
		u8 buf[13];
		u16 *wbuf = (u16 *) buf;
		int rc = 0;
		int i = 0;
		rc = i2c_master_send(c, (u8 *) & rds_cmd, sizeof(rds_cmd));
		if (rc < 0)
			return -1;
		mdelay(TUNER_DELAY);
		for (i = 0; i < 5; i++) {
			rc = i2c_master_recv(c, buf, sizeof(buf));
			if ((buf[0] & 0xC0) == 0x80)
				break;
		}
		if ((buf[0] & 0xC0) != 0x80)
			return -1;

		//rds->rdssyncfound = (buf[1] >> 2) & 0x01;
		//rds->rdssynclost = (buf[1] >> 1) & 0x01;
		//rds->rdsrecv = buf[1] & 0x01;
		//rds->grplost = (buf[2] >> 2) & 0x01;
		//rds->rdssync = buf[2] & 0x01;
		//rds->rdsfifoused = buf[3];
		rds->blocka = be16_to_cpu(wbuf[2]);
		rds->blockb = be16_to_cpu(wbuf[3]);
		rds->blockc = be16_to_cpu(wbuf[4]);
		rds->blockd = be16_to_cpu(wbuf[5]);
		//rds->blea = (buf[12] >> 6) & 0x03;
		//rds->bleb = (buf[12] >> 4) & 0x03;
		//rds->blec = (buf[12] >> 2) & 0x03;
		//rds->bled = buf[12] & 0x03;

	}
	return 0;
}
#endif

static int si4705_save_properties(struct i2c_client *c,
				  struct si4705_property_type *properties)
{
	int count = 0;
	int rc;

	while (properties[count].property != PROP_LIST_END) {
		u16 getprop[2] =
		    CMD_GET_PROPERTY(properties[count].property);
		u16 recvbuf[2] = { 0x0000, 0x0000 };

		int retry;	/* retry method used instead of interrupt method */
		for (retry = 0; retry < 5; retry++) {
			rc = i2c_master_send(c, (u8 *) getprop,
					     sizeof(getprop) / sizeof(u8));
			if (rc < 0)
				return rc;
			mdelay(1);

			rc = i2c_master_recv(c, (u8 *) recvbuf,
					     sizeof(recvbuf) / sizeof(u8));
			if (rc < 0)
				return rc;
			mdelay(1);

			recvbuf[0] = be16_to_cpu(recvbuf[0]);
			if ((recvbuf[0] & (WSTAT_CTS | WSTAT_ERR)) ==
			    WSTAT_CTS)
				break;
		}
		if ((recvbuf[0] & (WSTAT_CTS | WSTAT_ERR)) != WSTAT_CTS) {
			PK_ERR("Oops! no valid i2c response!\n");
			return -EINVAL;
		}

		properties[count].value = be16_to_cpu(recvbuf[1]);
		count++;
	}
	PK_DBG("Count = %d\n", count);
	return 0;
}

/* Note: properties are stored in cpu endianness */
static int si4705_restore_properties(struct i2c_client *client, struct si4705_property_type
				     *properties)
{
	int count = 0;
	int rc;

	/* Set all entries in the array. */
	while (properties[count].property != PROP_LIST_END) {
		u16 setprop[3] =
		    CMD_SET_PROPERTY(properties[count].property,
				     properties[count].value);
		u8 stat = 0x00;
		int retry = 0;
		rc = i2c_master_send(client, (u8 *) setprop,
				     sizeof(setprop) / sizeof(u8));
		if (rc < 0) {
			PK_ERR("Oops! i2c_master_send has failed!\n");
			return rc;
		}
		msleep(10);
		for (retry = 0; retry < 5; retry++) {
			rc = i2c_master_recv(client, &stat, sizeof(u8));
			if ((stat & 0xC0) == 0x80)
				break;	/* command accepted */
		}
		count++;
	}
	PK_DBG(" %d properties are set\n", count);
	return 0;
}

#if 0

static void si4705_print_properties(struct si4705_property_type *p)
{
	int i = 0;
	while (p[i].property != PROP_LIST_END) {
		PK_DBG("Property = 0x%04X, value = 0x%04X\n",
		       p[i].property, p[i].value);
		i++;
	}
}
#endif

static int si4705_i2c_probe(struct device *dev)
{
	struct fm_receiver_info *pdata = dev->platform_data;

	PK_ENTRY;
	tmcRX_devno = pdata->device_nr;

	return i2c_add_driver(&driver);
}

#ifdef CONFIG_PM
static int
si4705_i2c_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct i2c_client *c = to_i2c_client(dev);
	int rc = 0;

	if (level != SUSPEND_POWER_DOWN)
		return 0;

	if (c == NULL) {
		PK_ERR("Can't find I2C driver!\n");
		rc = -ENODEV;
	}

	PK_DBG
	    ("----------------------%s----------------pm_msg=%d, level=%d\n",
	     __func__, state, level);
	if (!rc)
		rc = si4705_powersuspend(c);
	if (rc) {
		PK_ERR("error while doing powersuspend: rc = %d\n", rc);
	}

	return 0;
}

static int si4705_i2c_resume(struct device *dev, u32 level)
{
	struct i2c_client *c = to_i2c_client(dev);
	int rc = 0;

	if (level != RESUME_POWER_ON)
		return 0;

	if (c == NULL) {
		PK_ERR("Can't find I2C driver!\n");
		rc = -ENODEV;
	}

	PK_DBG("----------------------%s----------------level=%d\n",
	       __func__, level);
	if (!rc) {
		rc = si4705_powerresume(c);
	} else {
		PK_ERR("error while doing powerresume: rc = %d\n", rc);
	}
	return 0;
}

#endif

static int si4705_powersuspend(struct i2c_client *c)
{
	int rc = 0;

	if (gFMState == FM_ON) {
		rc = si4705_get_properties(c, si4705_suspend_property);
		if (rc < 0) {
			return rc;
		} else {
			rc = si4705_powerdown(c);
			gFMState = FM_SUSPEND;
		}
	}
	return rc;
}

static int si4705_powerresume(struct i2c_client *c)
{
	int rc = 0;

	if (gFMState == FM_SUSPEND) {
		rc = si4705_powerup(c);
		if (rc < 0)
			return rc;

		rc = si4705_set_properties(c, si4705_suspend_property);
	}
	return rc;
}

static int si4705_open(struct inode *nodep, struct file *filep)
{
	struct si4705dev *dev =
	    container_of(nodep->i_cdev, struct si4705dev, cdev);

	PK_ENTRY;

	if (!atomic_dec_and_test(&si4705_available)) {
		atomic_inc(&si4705_available);
		return -EBUSY;
	}

	filep->private_data = dev;

	return 0;
}

static int si4705_release(struct inode *nodep, struct file *filep)
{
	PK_ENTRY;

	if (filep && filep->private_data)
		filep->private_data = NULL;

	atomic_inc(&si4705_available);

	return 0;
}

static int
si4705_ioctl(struct inode *nodep, struct file *filep,
	     unsigned int cmd, unsigned long arg)
{
	struct si4705dev *dev = filep ? filep->private_data : NULL;

	PK_DBG("----------------------%s----------- cmd=0x%x\n",
	       __func__, cmd);
	if (_IOC_TYPE(cmd) != FMRECEIVER_DRIVER_MAGIC) {
		PK_ERR("Wrong IOC type! Failing command\n");
		return -ENOTTY;
	}
	switch (cmd) {
	case TMCIOC_G_FREQUENCY:
		{
			__u16 freq = 0;
			__u16 rssi, snr;
			if (si4705_get_freq
			    (dev->client, &freq, &rssi, &snr))
				return -EFAULT;
			if (put_user(freq, (__u16 __user *) arg))
				return -EFAULT;
		}
		break;
	case TMCIOC_S_FREQUENCY:
		{
			__u16 freq = 0;
			if (get_user(freq, (__u16 __user *) arg))
				return -EFAULT;
			if (si4705_set_freq(dev->client, freq))
				return -EFAULT;
		}
		break;
	case TMCIOC_S_BAND:
		{
			__u16 band = 0;
			if (get_user(band, (__u16 __user *) arg))
				return -EFAULT;
			if (si4705_set_band(dev->client, band))
				return -EFAULT;
		}
		break;
	case TMCIOC_S_VOLUME:
		{
			__u16 vol = 0;
			if (get_user(vol, (__u16 __user *) arg))
				return -EFAULT;
			if (si4705_set_volume(dev->client, vol))
				return -EFAULT;
		}
		break;
	#if  0
	case TMCIOC_S_POWERSTATE:
		{
			__u16 power;
			if (get_user(power, (__u16 __user *) arg))
				return -EFAULT;
			if (power > 0) {
				if (si4705_powerup(dev->client))
					return -EFAULT;
			} else {
				if (si4705_powerdown(dev->client))
					return -EFAULT;
			}
		}
		break;
	case TMCIOC_S_SEEK_BOUNDS:
		{
			struct fmr_seek_bound_type bounds;
			if (copy_from_user
			    (&bounds, (void __user *) arg,
			     sizeof(struct fmr_seek_bound_type)))
				return -EFAULT;
			if (si4705_set_seek_bounds(dev->client, bounds))
				return -EFAULT;
		}
		break;
	#endif
	case TMCIOC_SEEK:
		{
			struct fmr_seek_param_type param;
			if (copy_from_user
			    (&param, (void __user *) arg,
			     sizeof(struct fmr_seek_param_type)))
				return -EFAULT;
			if (si4705_seek(dev->client, param))
				return -EFAULT;
		}
		break;
	#if 0
	case TMCIOC_Q_RSQ:
		{
			struct fmr_rsq_type rsq;
			if (si4705_query_rsq(dev->client, &rsq))
				return -EFAULT;
			if (copy_to_user
			    (&rsq, (void __user *) arg,
			     sizeof(struct fmr_rsq_type)))
				return -EFAULT;
		}
		break;
	case TMCIOC_Q_RDS:
		{
			struct fmr_rds_type rds;
			if (si4705_query_rds(dev->client, &rds))
				return -EFAULT;
			if (copy_to_user
			    (&rds, (void *__user *) arg,
			     sizeof(struct fmr_rds_type)))
				return -EFAULT;
		}
		break;
	case TMCIOC_S_AGC:
		{
			struct fmr_agc_type agc;
			if (copy_from_user
			    (&agc, (void __user *) arg,
			     sizeof(struct fmr_agc_type)))
				return -EFAULT;
			if (si4705_set_agc(dev->client, agc))
				return -EFAULT;
		}
		break;
	#endif
	case TMCIOC_QUERY_DEVICE_ID:
		{
			/* s_device_id is static global */
			__u16 id = s_device_id;
			if (put_user(id, (__u16 __user *) arg))
				return -EFAULT;
		}
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static unsigned int
si4705_poll(struct file *filep, struct poll_table_struct *pq)
{
	return 0;
}

struct file_operations si4705_fops = {
	.owner = THIS_MODULE,
	/* .read = si4705_read, */
	.open = si4705_open,
	.release = si4705_release,
	.ioctl = si4705_ioctl,
	.poll = si4705_poll,
};

static int si4705_powerdown(struct i2c_client *c)
{
	u8 pwrdown[] = CMD_POWER_DOWN;
	return i2c_master_send(c, pwrdown, sizeof(pwrdown) / sizeof(u8));
}


static int si4705_powerup(struct i2c_client *c)
{
	/* bad error handling */
	int rc = -ENOTTY;
	PK_ENTRY;
	{
		/* powerup, find out chip type */
		u8 pwrup[] = CMD_POWER_UP(POWERUP_FUNC_QLID,
					  POWERUP_OPMODE_ANA_OUT);
		rc = i2c_master_send(c, pwrup, sizeof(pwrup));
		mdelay(TUNER_DELAY);
	}
	{
		u8 buf[8];
		do {
			rc = i2c_master_recv(c, buf, 8);
		} while ((buf[0] & 0xC0) != 0x80);

		/* is this the right chip ? */
		switch (buf[1]) {
		case 5U:
		case 6U:
		case 49U:
			PK_DBG("Detected SI47%02d chip, good...\n",
			       buf[1]);
			break;
		default:
			PK_DBG("Invalid chip!\n");
			return -ENOTTY;
		}
		s_device_id = 4700U + buf[1];

	}
	si4705_powerdown(c);
	mdelay(50);
	{
		/* powerup, set fm receive mode */
		u8 pwrup[] = CMD_POWER_UP(POWERUP_FUNC_FMRX,
					  POWERUP_OPMODE_ANA_OUT);
		rc = i2c_master_send(c, pwrup, sizeof(pwrup));
		mdelay(TUNER_DELAY);
	}
	{
		/* set some default properties */
		struct si4705_property_type start_properties[] = {
			S_PROP_GPIO_IEN(0x0000),	/* GPIO not used for interrupts */
			S_PROP_FM_DEEMPHASIS(0x0001),	/* deemphasis is 50 uS */
			S_PROP_RX_HARD_MUTE(0x0000),	/* unmute left and right cannels */
			S_PROP_RX_VOLUME(0x0028),	/* default volume */
			S_PROP_LIST_END,	/* end of list */
		};

		rc = si4705_set_properties(c, start_properties);
		mdelay(TUNER_DELAY);
	}
	{
		/* tune to frequency */
		struct si4705_tune_cmd_type tune_cmd =
		    CMD_FM_TUNE_FREQ(8750U, 0U);
		rc = i2c_master_send(c, (u8 *) & tune_cmd,
				     sizeof(struct si4705_tune_cmd_type));
		mdelay(100);
	}
	{
		/* get status on tuning */
		u8 stat[] = CMD_FM_TUNE_STATUS(0x00);
		u8 buf[8];
		rc = i2c_master_send(c, stat, sizeof(stat));
		mdelay(TUNER_DELAY);
		rc = i2c_master_recv(c, buf, 8);
		PK_DBG
		    ("FM_TUNE_STAT response: [0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X]\n",
		     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
		     buf[6], buf[7]);
	}
	return rc;
}

static int
si4705_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct si4705dev *dev;
	struct i2c_client *c;
	int rc = 0;
	PK_ENTRY;

	c = kmalloc(sizeof *c, GFP_KERNEL);
	if (!c) {
		rc = -ENOMEM;
		goto no_client;
	}
	memcpy(c, &client_template, sizeof *c);
	c->adapter = adapter;
	c->addr = address;
	strcpy(c->name, /*"Silicon Labs 4705" */ SI4705_DEVNAME);

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

	/* is this a 4705 kind of device? */

	PK_DBG("Detecting device on adapter: %s, at i2c address: 0x%02X\n",
	       adapter->name, address);

	rc = si4705_powerup(c);

	if (rc < 0)
		goto no_cdev;

	/* Now setup our character device to the fm */
	cdev_init(&dev->cdev, &si4705_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &si4705_fops;
	if ((rc = cdev_add(&dev->cdev, tmcRX_devno, 1)))
		goto no_cdev;

	PK_DBG("%s: %p\n", __func__, c);


	return 0;

      no_cdev:
	i2c_detach_client(c);
      no_dev:
	kfree(c);
      no_client:
	PK_ERR("%s: ERROR %d!\n", __func__, rc);

	return rc;

}

/* ----------------------------------------------------------------------- */

static int si4705_attach(struct i2c_adapter *adap)
{
	int ret = 0;
	PK_ENTRY;

	ret = i2c_probe(adap, &addr_data, si4705_detect_client);
	PK_DBG("id=%d, name=%s, ret=%d \n", adap->id,
	       adap->algo->name, ret);

	return ret;
}

static int si4705_detach(struct i2c_client *c)
{
	struct si4705dev *fm = i2c_get_clientdata(c);

	PK_ENTRY;
	PK_DBG("%s: %p\n", __func__, c);

	i2c_release_client(c);
	i2c_detach_client(c);

	kfree(fm);
	kfree(c);

	return 0;
}

static int __init si4705_init(void)
{
	int result;

	result = driver_register(&driver.driver);
	PK_DBG
	    ("------------------%s:result = 0x%02X---------------\n",
	     __func__, result);

	return result;
}

static void __exit si4705_exit(void)
{
	PK_ENTRY;
	driver_unregister(&driver.driver);
}

MODULE_AUTHOR("Xander Hover <Xander.Hover@tomtom.com>");
MODULE_DESCRIPTION
    ("Driver for I2C connected Silicon Labs 4705/06/49 FM/RDS/TMC Receiver");
module_init(si4705_init);
module_exit(si4705_exit);
