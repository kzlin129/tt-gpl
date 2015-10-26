/*
 * ALSA SoC TLV320ADC3101 codec driver
 *
 * Author:  Sandeep S Prabhu, <sandeepsp@mistralsolutions.com>
 * Copyright:   (C) 2009 Mistral Solutions Pvt Ltd.
 *
 * Based on sound/soc/codecs/tlv320aic3x.c by  Vladimir Barinov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/***************************** INCLUDES ************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320adc3101.h"
#include <plat/tlv320adc3101_pdata.h>

/*
 * **************************************************************************** 
 *  Macros
 * **************************************************************************** 
 *    
 */

#ifdef CONFIG_SND_SOC_TLV320ADC3101_DSP
extern int adc3101_minidsp_program(struct snd_soc_codec *codec);
extern void adc3101_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

/* codec private data */
/*
struct adc3101_priv {
	unsigned int sysclk;
	int master;
	u8 page_no;
};
*/
#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
static struct snd_soc_codec * adc3101_codec;
#endif


static int adc3101_set_bias_level(struct snd_soc_codec *,
				  enum snd_soc_bias_level);

/*
 * ADC3101 register cache
 * We can't read the ADC3101 register space when we are
 * using 2 wire for device control, so we cache them instead.
 * There is no point in caching the reset register
 */
static const u8 adc3101_reg[ADC3101_CACHEREGNUM] = {
	0x00, 0x00, 0x00, 0x00,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x00,	/* 8 */
	0x00, 0x00, 0x00, 0x00,	/* 12 */
	0x00, 0x00, 0x01, 0x01,	/* 16 */
	0x80, 0x80, 0x04, 0x00,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x02, 0x01, 0x00,	/* 28 */
	0x00, 0x10, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x02, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x12, 0x00, 0x00,	/* 52 */
	0x00, 0x00, 0x00, 0x44,	/* 56 */
	0x00, 0x01, 0x00, 0x00,	/* 60 */
	0x00, 0x00, 0x00, 0x00,	/* 64 */
	0x00, 0x00, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0x00,	/* 72 */
	0x00, 0x00, 0x00, 0x00,	/* 76 */
	0x00, 0x00, 0x88, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x7F, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x00, 0x00, 0x00, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x00,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x00, 0x00, 0x00, 0x00,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x00,	/* 156, PAGE1-28 */
	0x00, 0x00, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x00, 0x00, 0x00, 0x00,	/* 164, PAGE1-36 */
	0x00, 0x00, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x00,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00, 0x00,	/* 176, PAGE1-48 */
	0xFF, 0x00, 0x3F, 0xFF,	/* 180, PAGE1-52 */
	0x00, 0x3F, 0x00, 0x80,	/* 184, PAGE1-56 */
	0x80, 0x00, 0x00, 0x00,	/* 188, PAGE1-60 */
};

/* 
 * adc3101 initialization data 
 * This structure initialization contains the initialization required for
 * ADC3101.
 * These registers values (reg_val) are written into the respective ADC3101 
 * register offset (reg_offset) to  initialize ADC3101. 
 * These values are used in adc3101_init() function only. 
 */
struct adc3101_configs {
	u8 reg_offset;
	u8 reg_val;
};

static const struct adc3101_configs adc3101_reg_init[] = {
	/* Choose Processing Block PRN_R1 (Default) */
	{PRB_SELECT, 0x01},
	/* Disable inputs Left ADC to Left PGA */
	{LEFT_PGA_SEL_1, 0xAA},
	/* Select IN2R/IN3R Differential Ended (0dB) inputs Right ADC */
	{RIGHT_PGA_SEL_1, 0x3F},
	/* Select IN2R/IN3R Differential Ended (0dB) inputs Left ADC */
	{LEFT_PGA_SEL_2, 0x33},
	/* Select IN2R/IN3R Differential Ended (0dB) inputs Right ADC */
	{RIGHT_PGA_SEL_2, 0x33},
	/* Unmute Left PGA + default gain 0dB */
	{LEFT_APGA_CTRL, 0x00},
	/* Unmute Right PGA + default gain 0dB */
	{RIGHT_APGA_CTRL, 0x00},
	/* MICBIAS1=2.5V */
	{MICBIAS_CTRL, 0x40},
	/* Use MCLK for clocks */
	{CLKGEN_MUX, USE_MCLK},
	/* BDIV_CLKIN = ADC_CLK */
	{INTERFACE_CTRL_2, 0x02},
	/* Left AGC Maximum Gain to 40db */
	{LEFT_CHN_AGC_3, 0x50},
	/* Right AGC Maximum Gain to 40db */
	{RIGHT_CHN_AGC_3, 0X50},

	/* ADC control, Gain soft-stepping enabled one step/fs */
	{ADC_DIGITAL, 0x02},
	/* Fine Gain 0dB, Left/Right ADC Unmute */
	{ADC_FGA, 0x00},
	/* DMCLK output = ADC_MOD_CLK */
	{GPIO2_CTRL, 0x28},
	/* DMDIN is in Dig_Mic_In mode */
	{GPIO1_CTRL, 0x04},
};

/****************** RATES TABLE FOR ADC3101 ************************/
struct adc3101_rate_divs {
	u32 mclk;
	u32 rate;
	u8 pll_p;
	u8 pll_r;
	u8 pll_j;
	u16 pll_d;
	u8 nadc;
	u8 madc;
	u8 aosr;
	u8 bdiv_n;
	u8 iadc;
	u32 rfs;
};

/*
 * PLL and Clock settings
 */

static const struct adc3101_rate_divs adc3101_divs[] = {
/*  mclk, rate, p, r, j, d, nadc, madc, aosr, bdiv rfs */
	/* 8k rate */
	{12000000, 8000, 1, 1, 7, 1680, 42, 2, 128, 8, 188, 0},
	/* 11.025k rate */
	{12000000, 11025, 1, 1, 6, 8208, 29, 2, 128, 8, 188, 0},
	/* 16k rate */
	{12000000, 16000, 1, 1, 7, 1680, 21, 2, 128, 8, 188, 0},
	/* 22.05k rate */
	{12000000, 22050, 1, 1, 7, 560, 15, 2, 128, 8, 188, 0},
	/* 32k rate */
	{12000000, 32000, 1, 1, 8, 1920, 12, 2, 128, 8, 188, 0},
#ifdef  CONFIG_SND_SOC_TLV320ADC3101_DSP
	/* 44.1k rate */
	{12000000, 44100, 1, 1, 7, 5264, 4, 4, 128, 16, 188, 0},
#else
	/* 44.1k rate */
	{12000000, 44100, 1, 1, 7, 5264, 8, 2, 128, 8, 188, 0},
#endif
	/* 48k rate */
	{12000000, 48000, 1, 1, 7, 1680, 7, 2, 128, 8, 188, 0},
	/* 88.2k rate */
	{12000000, 88200, 1, 1, 7, 5264, 4, 4, 64, 8, 188, 0},
	/* 96k rate */
	{12000000, 96000, 1, 1, 8, 1920, 4, 4, 64, 8, 188, 0},
	/* slave mode rfs=256 rate */
	{       0,     0, 0, 0, 0,    0, 1, 2, 128, 0, 0, 256},
	/* slave mode rfs=384 rate */
	{       0,     0, 0, 0, 0,    0, 2, 2,  96, 0, 0, 384},
	/* slave mode rfs=768 rate */
	{       0,     0, 0, 0, 0,    0, 2, 2, 192, 0, 0, 768},
};

static inline int adc3101_get_divs(int master, int mclk, int rate, int rfs)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adc3101_divs); i++) {
		if (master) {
			if ((adc3101_divs[i].rate == rate)
				&& (adc3101_divs[i].mclk == mclk)) {
				return i;
			}
		} else {
			if (adc3101_divs[i].rfs == rfs) {
				return i;
			}
		}
	}
	if (master) 
		printk("Master clock and sample rate is not supported\n");
	else
		printk("Slave clock and sample rate is not supported\n");

	return -EINVAL;
}

/*
 * Change register page
 */
int adc3101_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct adc3101_priv *adc3101 = codec->private_data;
	u8 data[2];

	data[0] = 0x0;
	data[1] = new_page;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("adc3101_change_page: I2C Wrte Error\n");
		return -1;
	}
	adc3101->page_no = new_page;
	return 0;
}

/*
 * write adc3101 register cache
 */
static inline void adc3101_write_reg_cache(struct snd_soc_codec *codec,
					   unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ADC3101_CACHEREGNUM)
		return;	
	cache[reg] = value & 0xFF;
}

/*
 * read adc3101 register cache
 */
static inline unsigned int adc3101_read_reg_cache(struct snd_soc_codec *codec,
						  unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ADC3101_CACHEREGNUM)
		return -1;
	return cache[reg];
}

static int adc3101_read(struct snd_soc_codec *codec, unsigned int reg);
/*
 * write to the adc3101 register space
 */
int adc3101_write(struct snd_soc_codec *codec, unsigned int reg,
		  unsigned int value)
{
	struct adc3101_priv *adc3101 = codec->private_data;
	u8 data[2];
	u8 page;

	page = reg / ADC3101_PAGE_SIZE;

	if (adc3101->page_no != page) {
		adc3101_change_page(codec, page);
	}

	/* data is
	 *   D15..D8 adc3101 register offset
	 *   D7...D0 register data
	 */
	data[0] = reg % ADC3101_PAGE_SIZE;
	data[1] = value & 0xFF;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("adc3101_write: I2C write Error\n");
		return -EIO;
	}		
#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
	printk ("ADC3101 W: %02X.%02X <-- %02X\n", page, data[0], data[1]);
#endif

	/* Update register cache */
	if ((page == 0) || (page == 1)) {
		adc3101_write_reg_cache(codec, reg, data[1]);
	}
	return 0;
}

#if defined (ADC3101_TEST_READ) || defined (CONFIG_SND_SOC_TLV320ADC3101_DEBUG)
/* Read from register and update register cache, 
 * Used to read status and flags registers
 * Other codec registers are read from cache
 */
/*
 * read from the adc3101 register space
 */
static int adc3101_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct adc3101_priv *adc3101 = codec->private_data;
	u8 value, page;

	page = reg / ADC3101_PAGE_SIZE;
	if (adc3101->page_no != page) {
		adc3101_change_page(codec, page);
	}

	/* write register address */
	reg = reg % ADC3101_PAGE_SIZE;
	if (codec->hw_write(codec->control_data, (char *)&reg, 1) != 1) {
		printk("adc3101_read: I2C write Error\n");
		return -EIO;
	}

	/*  read register value */
	if (codec->hw_read(codec->control_data, &value, 1) != 1) {
		printk("adc3101_read, I2C Read Error\n");
		return -EIO;
	}
#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
//	printk ("ADC3101 R: %02X.%02X --> %02X\n", page, reg, value);
#endif

	/* Update register cache */
	if ((page == 0) || (page == 1)) {
		adc3101_write_reg_cache(codec, reg, value);
	}
	return value;
}
#endif

#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
static ssize_t adc3101_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int reg;
	int count = 0;

	for (reg = 0x00; reg <= ADC3101_CACHEREGNUM; reg++) {
		if (count == 0) {
			len += sprintf(buf + len, "%02X.%02X", 
						reg / ADC3101_PAGE_SIZE, 
						reg % ADC3101_PAGE_SIZE);
		}
		switch (reg) {
			case PAGE_SELECT:
			case RESET:
			case CLKGEN_MUX:
			case PLL_PROG_PR:
			case PLL_PROG_J:
			case PLL_PROG_D_MSB:
			case PLL_PROG_D_LSB:
			case ADC_NADC:
			case ADC_MADC:
			case ADC_AOSR:
			case ADC_IADC:
			case MINIDSP_DECIMATION:
			case CLKOUT_MUX:
			case CLKOUT_M_DIV:
			case INTERFACE_CTRL_1:
			case CH_OFFSET_1:
			case INTERFACE_CTRL_2:
			case BCLK_N_DIV:
			case INTERFACE_CTRL_3:
			case INTERFACE_CTRL_4:
			case INTERFACE_CTRL_5:
			case I2S_SYNC:
			case ADC_FLAG:
			case CH_OFFSET_2:
			case I2S_TDM_CTRL:
			case INTR_FLAG_1:
			case INTR_FLAG_2:
			case INTR_FLAG_ADC1:
			case INTR_FLAG_ADC2:
			case INT1_CTRL:
			case INT2_CTRL:
			case GPIO2_CTRL:
			case GPIO1_CTRL:
			case DOUT_CTRL:
			case SYNC_CTRL_1:
			case SYNC_CTRL_2:
			case CIC_GAIN_CTRL:
			case PRB_SELECT:
			case INST_MODE_CTRL:
			case MIC_POLARITY_CTRL:
			case ADC_DIGITAL:
			case ADC_FGA:
			case LADC_VOL:
			case RADC_VOL:
			case ADC_PHASE_COMP:
			case LEFT_CHN_AGC_1:
			case LEFT_CHN_AGC_2:
			case LEFT_CHN_AGC_3:
			case LEFT_CHN_AGC_4:
			case LEFT_CHN_AGC_5:
			case LEFT_CHN_AGC_6:
			case LEFT_CHN_AGC_7:
			case LEFT_AGC_GAIN:
			case RIGHT_CHN_AGC_1:
			case RIGHT_CHN_AGC_2:
			case RIGHT_CHN_AGC_3:
			case RIGHT_CHN_AGC_4:
			case RIGHT_CHN_AGC_5:
			case RIGHT_CHN_AGC_6:
			case RIGHT_CHN_AGC_7:
			case RIGHT_AGC_GAIN:
			case DITHER_CTRL:
			case MICBIAS_CTRL:
			case LEFT_PGA_SEL_1:
			case LEFT_PGA_SEL_2:
			case RIGHT_PGA_SEL_1:
			case RIGHT_PGA_SEL_2:
			case LEFT_APGA_CTRL:
			case RIGHT_APGA_CTRL:
			case LOW_CURRENT_MODES:
			case ANALOG_PGA_FLAGS:
				len += sprintf(buf + len, " %02X", adc3101_read(adc3101_codec, reg));
				break;

			default:
				len += sprintf(buf + len, " ..");
				break;
		}
		++count;
		if (count > 15) {
			count = 0;
			len += sprintf(buf + len, "\n");
		}
	}
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t adc3101_reg_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	int val;
	int reg;
	int ret;
	int page;
	struct adc3101_priv *adc3101 = adc3101_codec->private_data;

	page = adc3101->page_no;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret != 2) {
		printk(KERN_ERR "%s: wrong arguments\n", __func__);
		return 0;
	}

	if (reg == 0x00) {
		page = adc3101->page_no = val;
	}
	adc3101_write(adc3101_codec, (page * 128) + reg, (unsigned char)val);
	val = adc3101_read(adc3101_codec, (page  * 128) + reg);

	printk("%02X.%02X <-- %02X\n", page, reg, val);
	return count;
}

static DEVICE_ATTR(reg, 0644, adc3101_reg_show, adc3101_reg_store);
#endif /* CONFIG_SND_SOC_TLV320ADC3101_DEBUG */

/*
 * ADC set volume for adc3101
 */

static int snd_soc_adc3101_put_volsw(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	s8 val1, val2;
	u8 reg;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if ((val1 >= ADC_POS_VOL)) {
		if (val1 > ADC_MAX_VOLUME)
			val1 = ADC_MAX_VOLUME;
		val1 = val1 - ADC_POS_VOL;
	} else if ((val1 >= 0) && (val1 <= 23)) {
		val1 = ADC_POS_VOL - val1;
		val1 = 128 - val1;
	} else
		return -EINVAL;

	if (val2 >= ADC_POS_VOL) {
		if (val2 > ADC_MAX_VOLUME)
			val2 = ADC_MAX_VOLUME;
		val2 = val2 - ADC_POS_VOL;
	} else if ((val2 >= 0) && (val2 <= 23)) {
		val2 = ADC_POS_VOL - val2;
		val2 = 128 - val2;
	} else
		return -EINVAL;

	reg = adc3101_read_reg_cache(codec, LADC_VOL) & (~0x7F);
	adc3101_write(codec, LADC_VOL, reg | (val1 << 0));
	reg = adc3101_read_reg_cache(codec, RADC_VOL) & (~0x7F);
	adc3101_write(codec, RADC_VOL, reg | (val2 << 0));

	return 0;
}

/*
 * ADC get volume for adc3101
 */

static int snd_soc_adc3101_get_volsw(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u8 val1;
	u8 val2;

	val1 = adc3101_read_reg_cache(codec, LADC_VOL) & (0x7F);
	if ((val1 >= 0) && (val1 <= 40)) {
		val1 = val1 + ADC_POS_VOL;
	} else if ((val1 >= 104) && (val1 <= 127)) {
		val1 = val1 - 104;
	} else
		return -EINVAL;

	val2 = adc3101_read_reg_cache(codec, RADC_VOL) & (0x7F);
	if ((val2 >= 0) && (val2 <= 40)) {
		val2 = val2 + ADC_POS_VOL;
	} else if ((val2 >= 104) && (val2 <= 127)) {
		val2 = val2 - 104;
	} else
		return -EINVAL;

	ucontrol->value.integer.value[0] = val1;
	ucontrol->value.integer.value[1] = val2;
	return 0;

}

#define SOC_ADC3101_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
        .info = snd_soc_info_volsw_2r, \
        .get = snd_soc_adc3101_get_volsw, .put = snd_soc_adc3101_put_volsw, \
        .private_value = (unsigned long)&(struct soc_mixer_control) \
                {.reg = reg_left, .rreg = reg_right, .shift = xshift, \
                .max = xmax, .invert = xinvert} }

static const char *micbias_voltage[] = { "off", "2V", "2.5V", "AVDD" };
static const char *linein_attenuation[] = { "0db", "-6db" };
static const char *adc_softstepping[] = { "1 step", "2 step", "off" };

#define MICBIAS1_ENUM		0
#define MICBIAS2_ENUM		1
#define ATTLINEL1_ENUM		2
#define ATTLINEL2_ENUM		3
#define ATTLINEL3_ENUM		4
#define ATTLINER1_ENUM		5
#define ATTLINER2_ENUM		6
#define ATTLINER3_ENUM		7
#define ADCSOFTSTEP_ENUM	8

static const struct soc_enum adc3101_enum[] = {
	SOC_ENUM_SINGLE(MICBIAS_CTRL, 5, 4, micbias_voltage),
	SOC_ENUM_SINGLE(MICBIAS_CTRL, 3, 4, micbias_voltage),
	SOC_ENUM_SINGLE(LEFT_PGA_SEL_1, 0, 2, linein_attenuation),
	SOC_ENUM_SINGLE(LEFT_PGA_SEL_1, 2, 2, linein_attenuation),
	SOC_ENUM_SINGLE(LEFT_PGA_SEL_1, 4, 2, linein_attenuation),
	SOC_ENUM_SINGLE(RIGHT_PGA_SEL_1, 0, 2, linein_attenuation),
	SOC_ENUM_SINGLE(RIGHT_PGA_SEL_1, 2, 2, linein_attenuation),
	SOC_ENUM_SINGLE(RIGHT_PGA_SEL_1, 4, 2, linein_attenuation),
	SOC_ENUM_SINGLE(ADC_DIGITAL, 0, 3, adc_softstepping),
};

/* amixer controls */
static const struct snd_kcontrol_new adc3101_snd_controls[] = {
	/* PGA Gain Volume Control */
	SOC_DOUBLE_R("PGA Gain Volume Control (0=0dB 80=40dB)",
		    LEFT_APGA_CTRL, RIGHT_APGA_CTRL, 0, 0x50, 0),
	/* Audio gain control (AGC) */
	SOC_DOUBLE_R("Audio Gain Control (AGC)", LEFT_CHN_AGC_1,
		     RIGHT_CHN_AGC_1, 7, 0x01, 0),
	/* AGC Target level control */
	SOC_DOUBLE_R("AGC Target Level Control", LEFT_CHN_AGC_1,
		     RIGHT_CHN_AGC_1, 4, 0x07, 1),
	/* AGC Maximum PGA applicable */
	SOC_DOUBLE_R("AGC Maximum PGA Control", LEFT_CHN_AGC_3,
		     RIGHT_CHN_AGC_3, 0, 0x50, 0),
	/* AGC Attack Time control */
	SOC_DOUBLE_R("AGC Attack Time control", LEFT_CHN_AGC_4,
		     RIGHT_CHN_AGC_4, 3, 0x1F, 0),
	/* AGC Decay Time control */
	SOC_DOUBLE_R("AGC Decay Time control", LEFT_CHN_AGC_5,
		     RIGHT_CHN_AGC_5, 3, 0x1F, 0),
	/* AGC Noise Bounce control */
	SOC_DOUBLE_R("AGC Noice bounce control", LEFT_CHN_AGC_6,
		     RIGHT_CHN_AGC_6, 0, 0x1F, 0),
	/* AGC Signal Bounce control */
	SOC_DOUBLE_R("AGC Signal bounce control", LEFT_CHN_AGC_7,
		     RIGHT_CHN_AGC_7, 0, 0x0F, 0),
	/* Mic Bias voltage */
	SOC_ENUM("Mic Bias 1 Voltage", adc3101_enum[MICBIAS1_ENUM]),
	SOC_ENUM("Mic Bias 2 Voltage", adc3101_enum[MICBIAS2_ENUM]),
	/* ADC soft stepping */
	SOC_ENUM("ADC soft stepping", adc3101_enum[ADCSOFTSTEP_ENUM]),
	/* Left/Right Input attenuation */
	SOC_ENUM("Left Linein1 input attenuation",
		 adc3101_enum[ATTLINEL1_ENUM]),
	SOC_ENUM("Left Linein2 input attenuation",
		 adc3101_enum[ATTLINEL2_ENUM]),
	SOC_ENUM("Left Linein3 input attenuation",
		 adc3101_enum[ATTLINEL3_ENUM]),
	SOC_ENUM("Right Linein1 input attenuation",
		 adc3101_enum[ATTLINER1_ENUM]),
	SOC_ENUM("Right Linein2 input attenuation",
		 adc3101_enum[ATTLINER2_ENUM]),
	SOC_ENUM("Right Linein3 input attenuation",
		 adc3101_enum[ATTLINER3_ENUM]),
	/* ADC Volume */
	SOC_ADC3101_DOUBLE_R("ADC Volume Control (0=-12dB 64=+20dB)", LADC_VOL,
		 RADC_VOL, 0, 64,0),
	/* ADC Fine Volume */
	SOC_SINGLE("Left ADC Fine Volume (0=-0.4dB 4=0dB)", ADC_FGA, 4, 4, 1),
	SOC_SINGLE("Right ADC Fine Volume (0=-0.4dB 4=0dB)", ADC_FGA, 0, 4, 1),
};

/* add non dapm controls */
static int adc3101_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(adc3101_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&adc3101_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left input selection, Single Ended inputs and Differential inputs */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_L switch", LEFT_PGA_SEL_1, 1, 0x1, 1),
	SOC_DAPM_SINGLE("IN2_L switch", LEFT_PGA_SEL_1, 3, 0x1, 1),
	SOC_DAPM_SINGLE("IN3_L switch", LEFT_PGA_SEL_1, 5, 0x1, 1),
	SOC_DAPM_SINGLE("DIF1_L switch", LEFT_PGA_SEL_1, 7, 0x1, 1),
	SOC_DAPM_SINGLE("DIF2_L switch", LEFT_PGA_SEL_2, 5, 0x1, 1),
	SOC_DAPM_SINGLE("DIF3_L switch", LEFT_PGA_SEL_2, 3, 0x1, 1),
	SOC_DAPM_SINGLE("IN1_R switch", LEFT_PGA_SEL_2, 1, 0x1, 1),
};

/* Right input selection, Single Ended inputs and Differential inputs */
static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_R switch", RIGHT_PGA_SEL_1, 1, 0x1, 1),
	SOC_DAPM_SINGLE("IN2_R switch", RIGHT_PGA_SEL_1, 3, 0x1, 1),
	SOC_DAPM_SINGLE("IN3_R switch", RIGHT_PGA_SEL_1, 5, 0x1, 1),
	SOC_DAPM_SINGLE("DIF1_R switch", RIGHT_PGA_SEL_1, 7, 0x1, 1),
	SOC_DAPM_SINGLE("DIF2_R switch", RIGHT_PGA_SEL_2, 5, 0x1, 1),
	SOC_DAPM_SINGLE("DIF3_R switch", RIGHT_PGA_SEL_2, 3, 0x1, 1),
	SOC_DAPM_SINGLE("IN1_L switch", RIGHT_PGA_SEL_2, 1, 0x1, 1),
};

/* Left Digital Mic input for left ADC */
static const struct snd_kcontrol_new left_input_dmic_controls[] = {
	SOC_DAPM_SINGLE("Left ADC switch", ADC_DIGITAL, 3, 0x1, 0),
};

/* Right Digital Mic input for Right ADC */
static const struct snd_kcontrol_new right_input_dmic_controls[] = {
	SOC_DAPM_SINGLE("Right ADC switch", ADC_DIGITAL, 2, 0x1, 0),
};

/* dapm widgets */
static const struct snd_soc_dapm_widget adc3101_dapm_widgets[] = {

	/* Left Input Selection */
	SND_SOC_DAPM_MIXER("Left Input Selection", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),
	/* Right Input Selection */
	SND_SOC_DAPM_MIXER("Right Input Selection", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),
	/*PGA selection */
	SND_SOC_DAPM_PGA("Left PGA", LEFT_APGA_CTRL, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right PGA", RIGHT_APGA_CTRL, 7, 1, NULL, 0),

	/*Digital Microphone Input Control for Left/Right ADC */
	SND_SOC_DAPM_MIXER("Left DMic Input", SND_SOC_NOPM, 0, 0,
			&left_input_dmic_controls[0],
			ARRAY_SIZE(left_input_dmic_controls)),
	SND_SOC_DAPM_MIXER("Right DMic Input", SND_SOC_NOPM , 0, 0, 
			&right_input_dmic_controls[0],
			ARRAY_SIZE(right_input_dmic_controls)),

	/* Left/Right ADC */
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ADC_DIGITAL, 7, 0),
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ADC_DIGITAL, 6, 0),

	/* Inputs */
	SND_SOC_DAPM_INPUT("IN1_L"),
	SND_SOC_DAPM_INPUT("IN1_R"),
	SND_SOC_DAPM_INPUT("IN2_L"),
	SND_SOC_DAPM_INPUT("IN2_R"),
	SND_SOC_DAPM_INPUT("IN3_L"),
	SND_SOC_DAPM_INPUT("IN3_R"),
	SND_SOC_DAPM_INPUT("DIF1_L"),
	SND_SOC_DAPM_INPUT("DIF2_L"),
	SND_SOC_DAPM_INPUT("DIF3_L"),
	SND_SOC_DAPM_INPUT("DIF1_R"),
	SND_SOC_DAPM_INPUT("DIF2_R"),
	SND_SOC_DAPM_INPUT("DIF3_R"),
	SND_SOC_DAPM_INPUT("DMic_L"),
	SND_SOC_DAPM_INPUT("DMic_R"),
};

/* dapm routes */
static const struct snd_soc_dapm_route intercon[] = {
/* Left input selection from switchs */
	{"Left Input Selection", "IN1_L switch", "IN1_L"},
	{"Left Input Selection", "IN2_L switch", "IN2_L"},
	{"Left Input Selection", "IN3_L switch", "IN3_L"},
	{"Left Input Selection", "DIF1_L switch", "DIF1_L"},
	{"Left Input Selection", "DIF2_L switch", "DIF2_L"},
	{"Left Input Selection", "DIF3_L switch", "DIF3_L"},
	{"Left Input Selection", "IN1_R switch", "IN1_R"},

/* Left input selection to left PGA */
	{"Left PGA", NULL, "Left Input Selection"},

/* Left PGA to left ADC */
	{"Left ADC", NULL, "Left PGA"},

/* Right input selection from switchs */
	{"Right Input Selection", "IN1_R switch", "IN1_R"},
	{"Right Input Selection", "IN2_R switch", "IN2_R"},
	{"Right Input Selection", "IN3_R switch", "IN3_R"},
	{"Right Input Selection", "DIF1_R switch", "DIF1_R"},
	{"Right Input Selection", "DIF2_R switch", "DIF2_R"},
	{"Right Input Selection", "DIF3_R switch", "DIF3_R"},
	{"Right Input Selection", "IN1_L switch", "IN1_L"},

/* Right input selection to right PGA */
	{"Right PGA", NULL, "Right Input Selection"},

/* Right PGA to right ADC */
	{"Right ADC", NULL, "Right PGA"},
	
/* Left DMic Input selection from switch */
	{"Left DMic Input", "Left ADC switch", "DMic_L"},

/* Left DMic to left ADC */
	{"Left ADC", NULL, "Left DMic Input"},

/* Right DMic Input selection from switch */
	{"Right DMic Input", "Right ADC switch", "DMic_R"},

/* Right DMic to right ADC */
	{"Right ADC", NULL, "Right DMic Input"},
};

/* Add dapm widgets */
static int adc3101_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, adc3101_dapm_widgets,
				  ARRAY_SIZE(adc3101_dapm_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/* ---------------------------------------------------------------------
 *   Digital Audio Interface Operations
 */

static int adc3101_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct adc3101_priv *adc3101 = codec->private_data;
	int i, rfs, adc_fs, width = 16;
	u8 data, bdiv;

	/* Calculate rfs */
	adc_fs = params_rate(params);
	rfs = adc3101->sysclk / adc_fs;

	i = adc3101_get_divs(adc3101->master, adc3101->sysclk, params_rate(params), rfs);
	if (i < 0) {
		printk("Clock configuration is not supported\n");
		return i;
	}

	/* select data word length */
	data = adc3101_read_reg_cache(codec, INTERFACE_CTRL_1) & (~WLENGTH_MASK);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		width = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x01 << 4);
		width = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x02 << 4);
		width = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x03 << 4);
		width = 32;
		break;
	default:
		return -EINVAL;
		break;
	}
	adc3101_write(codec, INTERFACE_CTRL_1, data);

	/* BCLK is derived from ADC_CLK */
	if (width == 16) {
		bdiv = adc3101_divs[i].bdiv_n;
	} else {
		bdiv =
		    (adc3101_divs[i].aosr * adc3101_divs[i].madc) / (2 * width);
	}


	if (!adc3101->master) {
		bdiv = adc3101->sysclk / adc3101_divs[i].nadc / (adc_fs * 2 * width);
	} else {
		/* P & R values */
		adc3101_write(codec, PLL_PROG_PR,
				(adc3101_divs[i].pll_p << PLLP_SHIFT) | (adc3101_divs[i].
									pll_r <<
									PLLR_SHIFT));
		/* J value */
		adc3101_write(codec, PLL_PROG_J, adc3101_divs[i].pll_j & PLLJ_MASK);
		/* D value */
		adc3101_write(codec, PLL_PROG_D_LSB,
				adc3101_divs[i].pll_d & PLLD_LSB_MASK);
		adc3101_write(codec, PLL_PROG_D_MSB,
				(adc3101_divs[i].pll_d >> 8) & PLLD_MSB_MASK);
	}

	/* NADC */
	adc3101_write(codec, ADC_NADC, adc3101_divs[i].nadc & NADC_MASK);
	/* MADC */
	adc3101_write(codec, ADC_MADC, adc3101_divs[i].madc & MADC_MASK);
	/* AOSR */
	adc3101_write(codec, ADC_AOSR, adc3101_divs[i].aosr & AOSR_MASK);
	/* BDIV N Value */
	adc3101_write(codec, BCLK_N_DIV, bdiv & BDIV_MASK);

	printk("ADC3101 PLL: NADC=%d MDAC=%d\n AOSR=%d, BDIV_N=%d\n",
	       adc3101_divs[i].nadc, adc3101_divs[i].madc, 
		   adc3101_divs[i].aosr, bdiv);

	return 0;
}

static int adc3101_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc3101_priv *adc3101 = codec->private_data;

	adc3101->sysclk = freq;
	return 0;
}

static int adc3101_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc3101_priv *adc3101 = codec->private_data;
	u8 iface_reg;

	iface_reg =
	    adc3101_read_reg_cache(codec, INTERFACE_CTRL_1) & (~FMT_MASK);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		adc3101->master = 1;
		iface_reg |= BCLK_MASTER | WCLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		adc3101->master = 0;
		break;
	default:
		printk("Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/*
	 * match both interface format and signal polarities since they
	 * are fixed
	 */
	switch (fmt & (SND_SOC_DAIFMT_FORMAT_MASK | SND_SOC_DAIFMT_INV_MASK)) {
	case (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF):
		break;
	case (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF):
		iface_reg |= (0x01 << 6);
		break;
	case (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF):
		iface_reg |= (0x01 << 6);
		break;
	case (SND_SOC_DAIFMT_RIGHT_J | SND_SOC_DAIFMT_NB_NF):
		iface_reg |= (0x02 << 6);
		break;
	case (SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF):
		iface_reg |= (0x03 << 6);
		break;
	default:
		printk("Invalid DAI format\n");
		return -EINVAL;
	}

	/* set iface */
	adc3101_write(codec, INTERFACE_CTRL_1, iface_reg);

	return 0;
}

static int adc3101_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct adc3101_priv *adc3101 = codec->private_data;
	u8 reg;

	/* all power is driven by DAPM system */
	switch (level) {
	case SND_SOC_BIAS_ON:
		if (adc3101->master) {
			/* Enable pll */
			reg = adc3101_read_reg_cache(codec, PLL_PROG_PR);
			adc3101_write(codec, PLL_PROG_PR, reg | ENABLE_PLL);

			/* 10msec delay needed after PLL power-up */
			mdelay(10);
		}

		/* Switch on NADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_NADC);
		adc3101_write(codec, ADC_NADC, reg | ENABLE_NADC);

		/* Switch on MADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_MADC);
		adc3101_write(codec, ADC_MADC, reg | ENABLE_MADC);

		/* Switch on BCLK_N Divider */
		reg = adc3101_read_reg_cache(codec, BCLK_N_DIV);
		adc3101_write(codec, BCLK_N_DIV, reg | ENABLE_BCLK);
		break;

		/* partial On */
	case SND_SOC_BIAS_PREPARE:
		break;

		/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		if (adc3101->master) {
			/* switch off pll */
			reg = adc3101_read_reg_cache(codec, PLL_PROG_PR);
			adc3101_write(codec, PLL_PROG_PR, reg & (~ENABLE_PLL));
		}

		/* Switch off NADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_NADC);
		adc3101_write(codec, ADC_NADC, reg & (~ENABLE_NADC));

		/* Switch off MADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_MADC);
		adc3101_write(codec, ADC_MADC, reg & (~ENABLE_MADC));

		/* Switch off BCLK_N Divider */
		reg = adc3101_read_reg_cache(codec, BCLK_N_DIV);
		adc3101_write(codec, BCLK_N_DIV, reg & (~ENABLE_BCLK));
		break;

		/* Off without power */
	case SND_SOC_BIAS_OFF:
		/* power off Left/Right ADC channels */
		reg = adc3101_read_reg_cache(codec, ADC_DIGITAL);
		adc3101_write(codec, ADC_DIGITAL,
			      reg & ~(LADC_PWR_ON | RADC_PWR_ON));

		/* Turn off PLLs */
		if (adc3101->master) {
			/* switch off pll */
			reg = adc3101_read_reg_cache(codec, PLL_PROG_PR);
			adc3101_write(codec, PLL_PROG_PR, reg & (~ENABLE_PLL));
		}

		/* Switch off NADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_NADC);
		adc3101_write(codec, ADC_NADC, reg & (~ENABLE_NADC));

		/* Switch off MADC Divider */
		reg = adc3101_read_reg_cache(codec, ADC_MADC);
		adc3101_write(codec, ADC_MADC, reg & (~ENABLE_MADC));

		/* Switch off BCLK_N Divider */
		reg = adc3101_read_reg_cache(codec, BCLK_N_DIV);
		adc3101_write(codec, BCLK_N_DIV, reg & (~ENABLE_BCLK));
		break;
	}
	codec->bias_level = level;

	return 0;
}

struct snd_soc_dai adc3101_dai = {
	.name = "adc3101",
	.id = 1,
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = ADC3101_RATES,
		    .formats = ADC3101_FORMATS,
	},
	.ops = {
		.hw_params = adc3101_hw_params,
		/*.digital_mute = adc3101_mute, */
		.set_sysclk = adc3101_set_dai_sysclk,
		.set_fmt = adc3101_set_dai_fmt,
	},
};

EXPORT_SYMBOL_GPL(adc3101_dai);

static int adc3101_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	adc3101_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int adc3101_resume(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec = socdev->codec;
    int i;
    u8 data[2];
    u8 *cache = codec->reg_cache;

    adc3101_change_page(codec,0);
    adc3101_write(codec, RESET, SOFT_RESET);
    /* Sync reg_cache with the hardware */
    for (i = 0; i < codec->reg_cache_size; i++) {
        if(i==PAGE_SELECT||i== RESET)
            continue;

        switch(i)
        {
        case PAGE_SELECT:
        case RESET:
        case CLKGEN_MUX:
        case PLL_PROG_PR:
        case PLL_PROG_J:
        case PLL_PROG_D_MSB:
        case PLL_PROG_D_LSB:
        case ADC_NADC:
        case ADC_MADC:
        case ADC_AOSR:
        case ADC_IADC:
        case MINIDSP_DECIMATION:
        case CLKOUT_MUX:
        case CLKOUT_M_DIV:
        case INTERFACE_CTRL_1:
        case CH_OFFSET_1:
        case INTERFACE_CTRL_2:
        case BCLK_N_DIV:
        case INTERFACE_CTRL_3:
        case INTERFACE_CTRL_4:
        case INTERFACE_CTRL_5:
        case I2S_SYNC:
        case ADC_FLAG:
        case CH_OFFSET_2:
        case I2S_TDM_CTRL:
        case INTR_FLAG_1:
        case INTR_FLAG_2:
        case INTR_FLAG_ADC1:
        case INTR_FLAG_ADC2:
        case INT1_CTRL:
        case INT2_CTRL:
        case GPIO2_CTRL:
        case GPIO1_CTRL:
        case DOUT_CTRL:
        case SYNC_CTRL_1:
        case SYNC_CTRL_2:
        case CIC_GAIN_CTRL:
        case PRB_SELECT:
        case INST_MODE_CTRL:
        case MIC_POLARITY_CTRL:
        case ADC_DIGITAL:
        case ADC_FGA:
        case LADC_VOL:
        case RADC_VOL:
        case ADC_PHASE_COMP:
        case LEFT_CHN_AGC_1:
        case LEFT_CHN_AGC_2:
        case LEFT_CHN_AGC_3:
        case LEFT_CHN_AGC_4:
        case LEFT_CHN_AGC_5:
        case LEFT_CHN_AGC_6:
        case LEFT_CHN_AGC_7:
        case LEFT_AGC_GAIN:
        case RIGHT_CHN_AGC_1:
        case RIGHT_CHN_AGC_2:
        case RIGHT_CHN_AGC_3:
        case RIGHT_CHN_AGC_4:
        case RIGHT_CHN_AGC_5:
        case RIGHT_CHN_AGC_6:
        case RIGHT_CHN_AGC_7:
        case RIGHT_AGC_GAIN:
        case DITHER_CTRL:
        case MICBIAS_CTRL:
        case LEFT_PGA_SEL_1:
        case LEFT_PGA_SEL_2:
        case RIGHT_PGA_SEL_1:
        case RIGHT_PGA_SEL_2:
        case LEFT_APGA_CTRL:
        case RIGHT_APGA_CTRL:
        case LOW_CURRENT_MODES:
        case ANALOG_PGA_FLAGS:			
            data[0] = i;
            data[1] = cache[i];

            // should use this function
            adc3101_write(codec, data[0], data[1]);
            break;
        default:
            break;
        }
    }

    adc3101_set_bias_level(codec, codec->suspend_bias_level);

    return 0;
}

/*
 * initialise the ADC3101 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int adc3101_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int i, ret = 0;

	codec->name = "adc3101";
	codec->owner = THIS_MODULE;
	codec->read = adc3101_read_reg_cache;
	codec->write = adc3101_write;
	codec->set_bias_level = adc3101_set_bias_level;
	codec->dai = &adc3101_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(adc3101_reg);
	codec->reg_cache =
	    kmemdup(adc3101_reg, sizeof(adc3101_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Select Page 0 */
	adc3101_write(codec, PAGE_SELECT, 0);
	/* Issue software reset to adc3101 */
	adc3101_write(codec, RESET, SOFT_RESET);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "adc3101: failed to create pcms\n");
		goto pcm_err;
	}

	for (i = 0;
	     i < sizeof(adc3101_reg_init) / sizeof(struct adc3101_configs);
	     i++) {
		adc3101_write(codec, adc3101_reg_init[i].reg_offset,
			      adc3101_reg_init[i].reg_val);
	}

	adc3101_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	adc3101_add_controls(codec);
	adc3101_add_widgets(codec);

#ifdef CONFIG_SND_SOC_TLV320ADC3101_DSP
	adc3101_add_minidsp_controls(codec);
#endif
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "adc3101: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static struct snd_soc_device *adc3101_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static int adc3101_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = adc3101_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct adc3101_priv *codec_private_data = 
			(struct adc3101_priv *)codec->private_data;
	tlv320adc3101_platform_t	* pdata = 
			(tlv320adc3101_platform_t *)i2c->dev.platform_data;
	int ret;

	if (NULL != pdata) {
		codec_private_data->pdata = pdata;
	} else {
		printk(KERN_ERR "ADC3101: No platform data!\n");
		return -EINVAL;
	}

	/* Switch on both codec and amplifier */
	if ((ret = pdata->request_gpio())) {
		printk(KERN_ERR "ADC3101: Cannot request GPIO\n");
		return ret;
	}
	pdata->config_gpio();
	pdata->mic_reset(1);
	udelay(100);
	pdata->mic_reset(0);

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = adc3101_init(socdev);
	if (ret < 0)
		printk(KERN_ERR "adc3101: failed to initialise ADC3101\n");

#ifdef CONFIG_SND_SOC_TLV320ADC3101_DSP
	/* Program MINI DSP for ADC */
	adc3101_minidsp_program(codec);
	adc3101_change_page(codec, 0x0);
#endif
	return ret;
}

static int adc3101_i2c_remove(struct i2c_client *client)
{
	struct tlv320adc3101 *tlv320adc3101 = i2c_get_clientdata(client);
	struct snd_soc_device *socdev = adc3101_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	tlv320adc3101_platform_t	* pdata = 
			(tlv320adc3101_platform_t *)client->dev.platform_data;

	i2c_set_clientdata(client, NULL);
	codec->control_data = NULL;
	kfree(codec->reg_cache);
	kfree(tlv320adc3101);
	pdata->free_gpio();

	printk("ADC3101: Codec unregistered\n");	
	return 0;
}
static int adc3101_i2c_suspend( struct i2c_client *client, pm_message_t mesg )
{
	tlv320adc3101_platform_t	* pdata = 
			(tlv320adc3101_platform_t *)client->dev.platform_data;

	if (pdata->suspend)
		pdata->suspend();
	return 0;
}

static int adc3101_i2c_resume( struct i2c_client *client )
{
	tlv320adc3101_platform_t	* pdata = 
			(tlv320adc3101_platform_t *)client->dev.platform_data;

	if (pdata->resume)
		pdata->resume();
	return 0;
}

static const struct i2c_device_id adc3101_i2c_id[] = {
	{TLV320ADC3101_DEVNAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adc3101_i2c_id);

/* machine i2c codec control layer */
static struct i2c_driver adc3101_i2c_driver = {
	.driver = {
		   .name = TLV320ADC3101_DEVNAME,
		   .owner = THIS_MODULE,
		   },
	.probe = adc3101_i2c_probe,
	.remove = adc3101_i2c_remove,
	.suspend=   adc3101_i2c_suspend,
	.resume =   adc3101_i2c_resume,
	.id_table = adc3101_i2c_id,
};

#endif

static int adc3101_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct adc3101_priv *adc3101;
	int ret = 0;

	printk(KERN_INFO "ADC3101 Audio Codec %s\n", ADC3101_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	adc3101 = kzalloc(sizeof(struct adc3101_priv), GFP_KERNEL);
	if (adc3101 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	printk("ADC3101 : Setting private defaults\n");
	adc3101->pdev = pdev;

	codec->private_data = adc3101;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	adc3101_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->hw_read = (hw_read_t) i2c_master_recv;
	ret = i2c_add_driver(&adc3101_i2c_driver);

	if (ret != 0)
		printk(KERN_ERR "can't add i2c driver");
#else
	/* Add other interfaces here */
#endif
 
#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
	adc3101_codec = codec;

	ret = device_create_file(&pdev->dev, &dev_attr_reg);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create reg file\n");
	}
#endif

	if (ret != 0) {
		kfree(codec->private_data);
		kfree(codec);
	}
	return ret;
}

static int adc3101_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* power down chip */
	if (codec->control_data)
		adc3101_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&adc3101_i2c_driver);
#endif

#ifdef CONFIG_SND_SOC_TLV320ADC3101_DEBUG
	device_remove_file(&pdev->dev, &dev_attr_reg);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adc3101 = {
	.probe = adc3101_probe,
	.remove = adc3101_remove,
	.suspend = adc3101_suspend,
	.resume = adc3101_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_adc3101);

MODULE_DESCRIPTION("ASoC TLV320ADC3101 codec driver");
MODULE_AUTHOR("Sandeep S Prabhu <sandeepsp@mistralsolutions.com>");
MODULE_LICENSE("GPL");
