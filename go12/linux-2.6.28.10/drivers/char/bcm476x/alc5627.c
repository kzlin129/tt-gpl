/*
 * alc5627.c - 
 *
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <asm/io.h>

#define ALC5627_DEVNAME "alc5627"
#define PFX ALC5627_DEVNAME ": "

#define ALC5627_ADDR_IND	0
#define ALC5627_DATA_IND	1
#define ALC5627_NB_MSG		2

#define ALC5627_I2C_DATA_MAXLEN	5
#define ALC5627_I2C_MSG_MAXLEN	5
#define ALC5627_I2C_MSG_LEN	3

#define ALC5627_RESET			0x00
#define ALC5627_SPK_OUT_VOL		0x02
#define ALC5627_HP_OUT_VOL		0x04
#define ALC5627_AUXIN_VOL		0x08
#define ALC5627_LINE_IN_VOL		0x0A
#define ALC5627_STEREO_DAC_VOL		0x0C
#define ALC5627_SOFT_VOL_CTRL_TIME	0x16
#define ALC5627_OUTPUT_MIXER_CTRL	0x1C

#define ALC5627_AUDIO_VOL_CTRL1		0x21
#define ALC5627_AUDIO_VOL_CTRL2		0x22
#define ALC5627_AUDIO_VOL_CTRL3		0x23
#define ALC5627_AUDIO_VOL_CTRL4		0x24
#define ALC5627_AUDIO_VOL_CTRL5		0x25

#define ALC5627_AUDIO_DATA_CTRL		0x34
#define ALC5627_DAC_CLK_CTRL		0x38

#define ALC5627_DIGITAL_INTERNAL_REG	0x39

#define ALC5627_PWR_MANAG_ADD1		0x3A
#define ALC5627_PWR_MANAG_ADD2		0x3C
#define ALC5627_PWR_MANAG_ADD3		0x3E
#define ALC5627_GEN_CTRL		0x40
#define ALC5627_GLOBAL_CLK_CTRL		0x42
#define ALC5627_PLL_CTRL		0x44
#define ALC5627_GPIO_INT_STAT_IRQ_CTRL	0x48
#define ALC5627_GPIO_PIN_CTRL		0x4A
#define ALC5627_JACK_DETECT_CTRL	0x5A
#define ALC5627_MISC1_CTRL		0x5C
#define ALC5627_MISC2_CTRL		0x5E
#define ALC5627_AVC_CTRL		0x68
#define ALC5627_PRIVATE_REG_IND		0x6A
#define ALC5627_PRIVATE_REG_DATA	0x6C


#define BIT01	0x0001
#define BIT02	0x0002
#define BIT03	0x0004
#define BIT04	0x0008
#define BIT05	0x0010
#define BIT06	0x0020
#define BIT07	0x0040
#define BIT08	0x0080

#define BIT09	0x01
#define BIT10	0x02
#define BIT11	0x04
#define BIT12	0x08
#define BIT13	0x10
#define BIT14	0x20
#define BIT15	0x40
#define BIT16	0x80

/* ================================== */
/* Speaker Output Volume              */

#define SPK_R_VOL_ATT_0dB				0x00
#define SPK_R_VOL_ATT_46_5dB				BIT01 | BIT02 | BIT03 | BIT04 | BIT05

#define SPK_L_VOL_ATT_0dB				0x00
#define SPK_L_VOL_ATT_46_5dB				BIT09 | BIT10 | BIT11 | BIT12 | BIT13

#define SPK_R_VOL_ATT_3dB				BIT02
#define SPK_L_VOL_ATT_3dB				BIT10

#define SPK_R_VOL_ATT_6dB				BIT03
#define SPK_L_VOL_ATT_6dB				BIT11

#define MUTE_SP_R_ON					BIT08
#define MUTE_SP_R_OFF					0x00

#define MUTE_SP_L_ON					BIT16
#define MUTE_SP_L_OFF					0x00

/* ================================== */
/* HP Output Volume                   */

#define HP_R_VOL_ATT_0dB				0x00
#define HP_R_VOL_ATT_46_5dB				BIT01 | BIT02 | BIT03 | BIT04 | BIT05

#define HP_L_VOL_ATT_0dB				0x00
#define HP_L_VOL_ATT_46_5dB				BIT09 | BIT10 | BIT11 | BIT12 | BIT13

#define MUTE_HP_R_ON					BIT08
#define MUTE_HP_R_OFF					0x00

#define MUTE_HP_L_ON					BIT16
#define MUTE_HP_L_OFF					0x00

/* ================================== */
/* Power Management Addition 1        */

#define EN_HP_OUT_AMP					BIT06
#define EN_HP_ENHANCE_AMP				BIT05

/* ================================== */
/* Power Management Addition 2        */

#define POW_CLSD					BIT15
#define POW_VREF					BIT14

#define POW_PLL						BIT13
#define POW_THERMAL					BIT12

#define POW_MIX_HP_R					BIT05
#define POW_MIX_HP_L					BIT06

#define POW_DAC_R_2_MIXER_DIRECT			BIT07
#define POW_DAC_L_2_MIXER_DIRECT			BIT08

#define POW_DAC_R					BIT09
#define POW_DAC_L					BIT10

/* ================================== */
/* Power Management Addition 3        */

#define POW_LI_R_VOL					BIT07
#define POW_LI_L_VOL					BIT08

#define POW_AUXIN_R_VOL					BIT05
#define POW_AUXIN_L_VOL					BIT06

#define POW_HP_R_VOL					BIT10
#define POW_HP_L_VOL					BIT11

#define POW_SPK_VOL					BIT13
#define POW_MAIN_BIAS					BIT16

/* ================================== */
/* Auxialiary Input Volume            */
#define SEL_AUXIN_R_VOL_0dB				BIT04
#define SEL_AUXIN_L_VOL_0dB				BIT12

#define EN_AUXIN_DIFF					BIT06
#define MUTE_AUXIN_R_2_SPK				BIT07
#define MUTE_AUXIN_R_2_HP				BIT08

#define MUTE_AUXIN_L_2_SPK				BIT15
#define MUTE_AUXIN_L_2_HP				BIT16


/* ================================== */
/* Line Input Volume                  */
#define SEL_LI_R_VOL_0dB				BIT04
#define SEL_LI_L_VOL_0dB				BIT12

#define EN_LI_DIFF					BIT06
#define MUTE_LI_R_2_SPK					BIT07
#define MUTE_LI_R_2_HP					BIT08

#define MUTE_LI_L_2_SPK					BIT15
#define MUTE_LI_L_2_HP					BIT16

/* ================================== */
/* Output Mixer Control               */

#define SEL_HP_R_IN					BIT09
#define SEL_HP_L_IN					BIT10

#define EN_SPK_VOL_DIFF					BIT03
#define SEL_SPK_VOL_IN					BIT11

/* ================================== */
/* General Purpose Control            */
#define SPK_AMP_RATIO_CTRL_1_25Vdd			BIT12
#define SPK_AMP_RATIO_CTRL_1_5Vdd			BIT10 | BIT11

#define STEREO_DAC_HIGH_PASS_FILTER_ON			BIT09
#define STEREO_DAC_HIGH_PASS_FILTER_OFF			0x00


#define		AUDIO_PATH_BOTH		0
#define 	AUDIO_PATH_SPEAKER	1
#define		AUDIO_PATH_HEADPHONE	2

int alsa_card_brcm_register(void);

typedef struct _alc5627_ctxt
{
	struct semaphore	mutex;
	int    			path_select;
} alc5627_ctxt_t;

static struct i2c_client *gclient = NULL;


int alc5627_i2c_write(struct i2c_client *client, unsigned char addr, unsigned char *data, int len)
{
	unsigned char msg[ALC5627_I2C_DATA_MAXLEN];
	int count = sizeof(addr) + len;
	int ret;

	msg[ALC5627_ADDR_IND] = addr;
	memcpy(&msg[ALC5627_DATA_IND], data, len);

	ret = i2c_master_send(client, msg, count);

	if (ret != count) {
		printk(KERN_ERR PFX "i2c write failure!\n");
		ret = -EIO;
	}

	return ret;
}

int alc5627_i2c_read(struct i2c_client *client, unsigned char addr, unsigned char *data, int len)
{
	unsigned char msg[ALC5627_I2C_DATA_MAXLEN];
	int ret;

	msg[ALC5627_ADDR_IND] = addr;

	ret = i2c_master_send(client, msg, sizeof(addr));

	if (ret != sizeof(addr)) {
		printk(KERN_ERR PFX "i2c send read cmd failure!\n");
		ret = -EIO;
	}

	ret = i2c_master_recv(client, data, len);

	if (ret != len) {
		printk(KERN_ERR PFX "i2c read failure!\n");
		ret = -EIO;
	} else  {
		printk("read: %x %x\n", data[0], data[1]);
	}


	return ret;
}

void alc5627_i2c_mute_hp_spkr(void)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	/* HP OUT mute-unmute de-pop mode */
	data[0] = 0x01;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x00;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x5e, data, 2);

	data[0] = 0x00;
	data[1] = 0xe0;
	alc5627_i2c_write(gclient, 0x5e, data, 2);

	data[0] = 0x80;
	data[1] = 0x80;
	alc5627_i2c_write(gclient, 0x04, data, 2);

	data[0] = 0x80;
	data[1] = 0x80;
	alc5627_i2c_write(gclient, 0x02, data, 2);

	msleep(32); /* 30 ms in spec */
}

void alc5627_i2c_unmute_hp_spkr(int output)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	/* HP OUT mute-unmute de-pop mode */
	data[0] = 0x01;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x00;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x5e, data, 2);

	data[0] = 0x00;
	data[1] = 0xe0;
	alc5627_i2c_write(gclient, 0x5e, data, 2);

	data[0] = 0x00;
	data[1] = 0x00;
	switch (output) {
		case 0x06:
		case 0x04:
			alc5627_i2c_write(gclient, 0x04, data, 2);
			if (output == 0x4)
				break;
		case 0x02:
			alc5627_i2c_write(gclient, 0x02, data, 2);
			break;
	}

	msleep(32); /* 30 ms in spec */
}

void alc5627_i2c_startup(int delay)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	/* HP out de-pop mode 2 */
	data[0] = 0x80;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3e, data, 2);

	data[0] = 0x80;
	data[1] = 0x80;
	alc5627_i2c_write(gclient, 0x04, data, 2);

	data[0] = 0x01;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x20;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3c, data, 2);

	data[0] = 0x86;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3e, data, 2);

	data[0] = 0x02;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x5e, data, 2);

	msleep(delay); /* > 400ms for de-pop mode 2 */

	data[0] = 0x00;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x00;
	data[1] = 0x30;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x00;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x5e, data, 2);
}

void alc5627_i2c_enable_paths(void)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	/* enable both hp and speaker amps */
	data[0] = 0x48;
	data[1] = 0x68;
	alc5627_i2c_write(gclient, 0x08, data, 2);

	data[0] = 0x48;
	data[1] = 0x68;
	alc5627_i2c_write(gclient, 0x0a, data, 2);

	data[0] = 0x07;
	data[1] = 0x04;
	alc5627_i2c_write(gclient, 0x1c, data, 2);

	data[0] = 0x07;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x40, data, 2);

	data[0] = 0x80;
	data[1] = 0xf0;
	alc5627_i2c_write(gclient, 0x3e, data, 2);

	data[0] = 0x20;
	data[1] = 0x30;
	alc5627_i2c_write(gclient, 0x3c, data, 2);

	data[0] = 0x00;
	data[1] = 0x20;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x60;
	data[1] = 0x30;
	alc5627_i2c_write(gclient, 0x3c, data, 2);

	data[0] = 0x96;
	data[1] = 0xf0;
	alc5627_i2c_write(gclient, 0x3e, data, 2);
}

void alc5627_i2c_disable_paths(void)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	/* disbale both hp and speaker amps */
	data[0] = 0x80;
	data[1] = 0x04;
	alc5627_i2c_write(gclient, 0x1c, data, 2);

	data[0] = 0xc8;
	data[1] = 0xc8;
	alc5627_i2c_write(gclient, 0x08, data, 2);

	data[0] = 0xc8;
	data[1] = 0xc8;
	alc5627_i2c_write(gclient, 0x0a, data, 2);

	data[0] = 0x80;
	data[1] = 0xf0;
	alc5627_i2c_write(gclient, 0x3e, data, 2);

	data[0] = 0x00;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3a, data, 2);

	data[0] = 0x20;
	data[1] = 0x30;
	alc5627_i2c_write(gclient, 0x3c, data, 2);

	data[0] = 0x20;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3c, data, 2);

	data[0] = 0x80;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3e, data, 2);
}

void alc5627_i2c_powerdown_suspend(void)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];

	alc5627_i2c_disable_paths();

	/* power down */
	data[0] = 0x00;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3e, data, 2);

	data[0] = 0x00;
	data[1] = 0x00;
	alc5627_i2c_write(gclient, 0x3c, data, 2);
}

int alc5627_i2c_mute(void)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_mute_hp_spkr();

	up(&ctxt->mutex);
	
	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_mute);

int alc5627_i2c_unmute_hp(void)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_unmute_hp_spkr(0x04);

	up(&ctxt->mutex);
	
	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_unmute_hp);

int alc5627_i2c_unmute_spkr(void)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_unmute_hp_spkr(0x02);

	up(&ctxt->mutex);
	
	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_unmute_spkr);

int alc5627_i2c_audio_powerdown(void)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_spk_select: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_powerdown_suspend();

	up(&ctxt->mutex);
	return 0;
}

int alc5627_i2c_line_in_gain(int gain)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	data[0] = MUTE_AUXIN_L_2_SPK | gain;
	data[1] = EN_AUXIN_DIFF | MUTE_AUXIN_R_2_SPK | gain;
	alc5627_i2c_write(gclient, ALC5627_AUXIN_VOL, data, 2);

	data[0] = MUTE_LI_L_2_SPK | gain;
	data[1] = EN_LI_DIFF | MUTE_LI_R_2_SPK | gain;
	alc5627_i2c_write(gclient, ALC5627_LINE_IN_VOL, data, 2);

	up(&ctxt->mutex);

	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_line_in_gain);

int alc5627_i2c_spk_gain(int gain)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");		
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	data[0] = gain | MUTE_SP_L_OFF;
	data[1] = gain | MUTE_SP_R_OFF;
	alc5627_i2c_write(gclient, ALC5627_SPK_OUT_VOL, data, 2);

	up(&ctxt->mutex);

	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_spk_gain);

int alc5627_i2c_amp_ratio(int ratio)
{
	unsigned char data[ALC5627_I2C_DATA_MAXLEN];
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");		
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	data[0] = ratio | STEREO_DAC_HIGH_PASS_FILTER_ON;
	data[1] = 0x0;
	alc5627_i2c_write(gclient, ALC5627_GEN_CTRL, data, 2);

	up(&ctxt->mutex);

	return 0;
}
EXPORT_SYMBOL(alc5627_i2c_amp_ratio);

static int alc5627_i2c_init(void)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");		
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_startup(3); /* delay should be > 400 ms but reduced to not slow down boot process */
	alc5627_i2c_enable_paths();
	alc5627_i2c_unmute_hp_spkr(0x6);

	up(&ctxt->mutex);

	return 0;
}

static int alc5627_i2c_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	alc5627_ctxt_t *ctxt = NULL;
	gclient = client;

	ctxt = (alc5627_ctxt_t *) kzalloc(sizeof(alc5627_ctxt_t), GFP_KERNEL);
	if(NULL == ctxt) {
		printk(KERN_ERR PFX "Can't allocate work!\n");
		return -ENOMEM;
	}

	init_MUTEX(&ctxt->mutex);
	i2c_set_clientdata(client, ctxt);

	printk (KERN_INFO "Realtek ALC5627 driver\n");

	return alc5627_i2c_init();
}

static int alc5627_i2c_remove(struct i2c_client *client)
{
	alc5627_ctxt_t *ctxt = i2c_get_clientdata(client);
	BUG_ON(!ctxt);

	alc5627_i2c_audio_powerdown();
	kfree(ctxt);

	return 0;
}

static void alc5627_i2c_shutdown(struct i2c_client *client)
{
	alc5627_ctxt_t *ctxt = i2c_get_clientdata(client);
	BUG_ON(!ctxt);

	alc5627_i2c_audio_powerdown();
}

#ifdef CONFIG_PM
static int alc5627_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	alc5627_i2c_audio_powerdown();
	return 0;
}

static int alc5627_i2c_resume(struct i2c_client *client)
{
	alc5627_ctxt_t * ctxt = NULL;

	if (NULL == gclient) {
		printk(KERN_INFO PFX "alc5627_i2c_audio_init: no i2c client\n");
		return -ENODEV;
	}

	ctxt = i2c_get_clientdata(gclient);
	BUG_ON(!ctxt);

	if (down_interruptible(&ctxt->mutex))
		return -ERESTARTSYS;

	alc5627_i2c_startup(420);
	alc5627_i2c_enable_paths();
	alc5627_i2c_mute_hp_spkr();

	up(&ctxt->mutex);

	return 0;
}

#else
#define alc5627_suspend	NULL
#define alc5627_resume	NULL
#endif /* CONFIG_PM	*/

static const struct i2c_device_id alc5627_id[] = {
	{ ALC5627_DEVNAME, 1, },
	{ }
};

static struct i2c_driver alc5627_i2c_driver =
{
	.id		= 1049,
	.probe		= alc5627_i2c_probe,
	.remove		= alc5627_i2c_remove,
	.shutdown	= alc5627_i2c_shutdown,
	.suspend	= alc5627_i2c_suspend,
	.resume		= alc5627_i2c_resume,
	.driver 	= {
		.name	= ALC5627_DEVNAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= alc5627_id,
};

static int __init alc5627_init(void)
{
	int err;

	printk(KERN_INFO PFX "add i2c driver\n");

	if ((err = i2c_add_driver(&alc5627_i2c_driver))) {
		printk(KERN_ERR PFX "Could Not Be Added. Err Code: [%i]\n", err);
	}

	return err;
}

static void __exit alc5627_exit(void)
{
	printk(KERN_INFO PFX "remove i2c driver\n");
	i2c_del_driver(&alc5627_i2c_driver);
}


MODULE_LICENSE("GPL");

module_init(alc5627_init);
module_exit(alc5627_exit);
