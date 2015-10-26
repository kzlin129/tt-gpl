/*
 * File:	sound/soc/codecs/adau1761.c
 * Based on:	wm8753.c by Liam Girdwood && ad1836.c by Luuk van Dijk
 * Author:	John McCarty <john.mccarty@analog.com>
 *
 * Created:      Fri Jan 16 2009
 * Description:  SoC "Codec driver" for ADAU1761 low power dsp codec
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * theory of operation:
 *
 *  The adau1761 is connected via I2C to SoC for portable audio
 *  applications. DAI (Digital Audio Interface) is I2S mode. 
 *  Headphones are connected in capless configuration.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "adau1761.h"
#include "adau1761_fw.h"
#include <plat/adau1761_pdata.h>

#define AUDIO_NAME "ADAU1761"
#define ADAU1761_VERSION "0.20"

#define CAP_MIC  1
#define CAP_LINE 2
#define CAPTURE_SOURCE_NUMBER 2

#if !defined (CONFIG_I2C) && !defined (CONFIG_I2C_MODULE)
#error "The driver requires I2C bus!"
#endif

/*
 * Debug
 */
#ifdef CONFIG_SND_SOC_ADAU1761_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

#ifdef CONFIG_SND_DEBUG
#define snd_printk_marker() snd_printk(KERN_INFO "%s\n", __FUNCTION__)
#else
#define snd_printk_marker()
#endif


static int adau1761_rfs = CONFIG_SND_SOC_ADAU1761_INFREQ;
static int clkctrl_frq_infreq = 0;

struct snd_soc_codec_device soc_codec_dev_adau1761;

/*
 * read register cache
 */
static inline unsigned int adau1761_read_reg_cache(
	struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg < ADAU_FIRSTREG) {
		/* hack, SoC kcontrol callbacks only support 1byte reg add */
		reg = reg + ADAU_FIRSTREG;
	}
	
	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG)) {
		dbg("register out of range: reg=%04X",reg);
		return -EPERM;
	}

#if 0
	dbg("%s %d read: %04X=%02X",__func__, __LINE__, reg,cache[reg - ADAU_FIRSTREG]);
#endif

	return cache[reg - ADAU_FIRSTREG];
}

/*
 * write register cache
 */
static inline int adau1761_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;

	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG)) {
		dbg("register out of range: reg=%04X",reg);
		return -EPERM;
	}

#if 0
	dbg("%s %d: %04X=%02X",__func__, __LINE__, reg,value);
#endif

	cache[reg - ADAU_FIRSTREG] = value;

	return 0;
}

/*
 * read ADAU1761 hw register and update cache
 */
static int adau1761_read_reg_byte(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 addr[2];
	u8 buf[1] = {0};

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;

	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG))
		return -EIO;
	
	addr[0] = (u8)(reg >> 8);
	addr[1] = (u8)(reg & 0xFF);

	/* write the 2byte read address */
	if (codec->hw_write(codec->control_data, addr, 2) != 2) {
		dbg("%s %d:hw_write failed.", __func__, __LINE__);
		return -EIO;
	}

	/* perform the read */
	if (codec->hw_read(codec->control_data, buf, 1) != 1) {
		dbg("%s %d:hw_read failed.", __func__, __LINE__);
		return -EIO;
	}

#if 0
	dbg("R %04X %02X", reg, buf[0]);
#endif
	
	adau1761_write_reg_cache(codec, reg, (unsigned int)buf[0]);
	return 0;
}

#ifdef CONFIG_SND_SOC_ADAU1761_DEBUG
/*
 * read a multi-byte ADAU1761 register (6byte pll reg)
 */
static int adau1761_read_reg_block(struct snd_soc_codec *codec,
	unsigned int reg, u8 len)
{
	u8 buf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 addr[2];
	unsigned int i;

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;

	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG))
		return -EIO;

	addr[0] = (u8)(reg >> 8);
	addr[1] = (u8)(reg & 0xFF);

	/* write the 2byte read address */
	if (codec->hw_write(codec->control_data, addr, 2) != 2) {
		dbg("read_reg_byte:address write failed.");
		return -EIO;
	}

	if (codec->hw_read(codec->control_data, buf, len) != len)
		return -EIO;

#if 0
	dbg("ReadBlk-%04X: [%02X][%02X][%02X][%02X][%02X][%02X][%02X][%02X]",
		reg,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
#endif

	for (i = 0; i < len; i++) {
		adau1761_write_reg_cache(codec, reg+i, (unsigned int)buf[i]);
	}

	return 0;
}
#endif

/*
 * write to a adau1761 hw register
 */
static int adau1761_write_reg_byte(struct snd_soc_codec *codec, 
	unsigned int reg, unsigned int value)
{
	/* 2byte register address, 1byte value */
	u8 buf[3];

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;
	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)(reg & 0xFF);
	buf[2] = (u8)(value & 0xFF);

	if (adau1761_write_reg_cache (codec, reg, value) == -1)
		return -EIO;

#if 0
	dbg("W %04X %02X", reg, buf[2]);
#endif

	if (codec->hw_write(codec->control_data, buf, 3) != 3) {
		dbg("%s %d:hw_write failed.", __func__, __LINE__);
		return -EIO;
	}

	return 0;
}

/*
 * write a multibyte ADAU1761 register (6byte pll reg)
 */
static int adau1761_write_reg_block(struct snd_soc_codec *codec, 
	unsigned int reg, u8 length, u8* values)
{
	int count = length + 2; /*data plus 16bit register address*/
	int i;
	u8 buf[8]= {0,0,0,0,0,0,0,0};

	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)(reg & 0xFF);

	if (length > 0)
		memcpy(&buf[2], values, length);
#if 0	
	dbg("WriteBlk-%04X: [%02X][%02X][%02X][%02X][%02X][%02X][%02X][%02X]",
		reg,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
#endif

	for (i = 0; i < length; i++) {
		if (adau1761_write_reg_cache (codec, reg + i, buf[2 + i]) == -1)
			return -EIO;
	}

	if (codec->hw_write(codec->control_data, buf, count) != count) {
		dbg("%s %d:hw_write failed.", __func__, __LINE__);
		return -EIO;
	}

	return 0;
}

static void dumpRegisters(struct snd_soc_codec *codec)
{
#ifdef CONFIG_SND_SOC_ADAU1761_DEBUG
	int reg;
	int count = 0;
	u8 *cache = codec->reg_cache;
	for (reg = ADAU_FIRSTREG; reg <= ADAU_LASTREG; reg++) {
		if (count == 0) 
			printk("%04X", reg);

		switch (reg) {
			case ADAU_CLKCTRL:	// 0x4000
			case ADAU_PLLCTRL:	// 0x4002
			case ADAU_MICCTRL:	// 0x4008
			case ADAU_RECPWRM:	// 0x4009
			case ADAU_RECMLC0:	// 0x400A
			case ADAU_RECMLC1:	// 0x400B
			case ADAU_RECMRC0:	// 0x400C
			case ADAU_RECMRC1:	// 0x400D
			case ADAU_RECVLCL:	// 0x400E
			case ADAU_RECVLCR:	// 0x400F

			case ADAU_RECMBIA:	// 0x4010
			case ADAU_ALCCTR0:	// 0x4011
			case ADAU_ALCCTR1:	// 0x4012
			case ADAU_ALCCTR2:	// 0x4013
			case ADAU_ALCCTR3:	// 0x4014
			case ADAU_SPRTCT0:	// 0x4015
			case ADAU_SPRTCT1:	// 0x4016
			case ADAU_CONVCT0:	// 0x4017
			case ADAU_CONVCT1:	// 0x4018
			case ADAU_ADCCTL0:	// 0x4019
			case ADAU_ADCCTL1:	// 0x401A
			case ADAU_ADCCTL2:	// 0x401B
			case ADAU_PLBMLC0:	// 0x401C
			case ADAU_PLBMLC1:	// 0x401D
			case ADAU_PLBMRC0:	// 0x401E
			case ADAU_PLBMRC1:	// 0x401F

			case ADAU_PLBMLLO:	// 0x4020
			case ADAU_PLBMRLO:	// 0x4021
			case ADAU_PLBLRMC:	// 0x4022
			case ADAU_PLBHPVL:	// 0x4023
			case ADAU_PLBHPVR:	// 0x4024
			case ADAU_PLBLOVL:	// 0x4025
			case ADAU_PLBLOVR:	// 0x4026
			case ADAU_PLBMNOC:	// 0x4027
			case ADAU_PLBCTRL:	// 0x4028
			case ADAU_PLBPWRM:	// 0x4029

			case ADAU_DACCTL0:	// 0x402A
			case ADAU_DACCTL1:	// 0x402B
			case ADAU_DACCTL2:	// 0x402C
			case ADAU_SERPAD0:	// 0x402D
			case ADAU_SERPAD1:	// 0x402E
			case ADAU_COMPAD0:	// 0x402F
			case ADAU_COMPAD1:	// 0x4030
			case ADAU_MCLKPAD:	// 0x4031

			case ADAU_DEJITTER:	// 0x4036

			/* DSP Core */
			case ADAU_DSP_CRC:	// 0x40C0
			case ADAU_DSP_GPI:	// 0x40C6
			case ADAU_NON_MOD:	// 0x40E9
			case ADAU_WATCGDG:	// 0x40D0
			case ADAU_DSP_SR0:	// 0x40EA
			case ADAU_DSP_SR1:	// 0x40EB

			case ADAU_INP_ROT:	// 0x40F2
			case ADAU_OUT_ROT:	// 0x40F3
			case ADAU_SER_DAT:	// 0x40F4
			case ADAU_DSP_ENA:	// 0x40F5
			case ADAU_DSP_RUN:	// 0x40F6
			case ADAU_DSP_SLW:	// 0x40F7
			case ADAU_SER_SRT:	// 0x40F8
			case ADAU_CLK_EN0:	// 0x40F9
			case ADAU_CLK_EN1:	// 0x40FA
				printk(" %02X", cache[reg - ADAU_FIRSTREG]);
				break;

			default:
				printk(" ..");
				break;
		}
		++count;
		if (count > 15) {
			count = 0;
			printk("\n");
		}
	}
	printk("\n");
	printk("\n");
#endif
}

/*
 * adau1761 controls
 */
static int adau1761_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *privdata = codec->private_data;

	snd_printk_marker();
	if (privdata->in_source & CAP_MIC){
		ucontrol->value.integer.value[0] = 0x0;
		dbg("Current Input is: 0");
	}
	else {
		ucontrol->value.integer.value[0] = 0x1;
		dbg("Current Input is: 1");
	}

	return 0;
}

static int adau1761_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *privdata = codec->private_data;
	int src = ucontrol->value.integer.value[0];
	u8 regvalue=0;

	snd_printk_marker();
	dbg("Src select: %d", src);

	if (src==0) {/* Select Mic */
		privdata->in_source = CAP_MIC;
#ifdef CONFIG_SND_ADAU1761_DIG_MIC
		regvalue = (adau1761_read_reg_cache(codec,ADAU_ADCCTL0) & 0xFB)|0x4;
		adau1761_write_reg_byte(codec, ADAU_ADCCTL0, regvalue);
#else
		adau1761_write_reg_byte(codec, ADAU_RECMBIA, RECMBIA_ENABLE);
		regvalue = (adau1761_read_reg_cache(codec,ADAU_RECVLCL)
				| RECVLC_ENABLE_MASK);
		adau1761_write_reg_byte(codec, ADAU_RECVLCL, regvalue);
		regvalue = (adau1761_read_reg_cache(codec,ADAU_RECVLCR) 
				| RECVLC_ENABLE_MASK);
		adau1761_write_reg_byte(codec, ADAU_RECVLCR, regvalue);
		adau1761_write_reg_byte(codec, ADAU_RECMLC1, RECMLC_MIC_0DB);
		adau1761_write_reg_byte(codec, ADAU_RECMRC1, RECMLC_MIC_0DB);
#endif
	}
	else if (src==1) {/* Select Line */
		privdata->in_source = CAP_LINE;
#ifdef CONFIG_SND_ADAU1761_DIG_MIC
		regvalue = (adau1761_read_reg_cache(codec,ADAU_ADCCTL0) & 0xFB);
		adau1761_write_reg_byte(codec, ADAU_ADCCTL0, regvalue);
#endif
		adau1761_write_reg_byte(codec, ADAU_RECMBIA, RECMBIA_DISABLE);
		regvalue = (adau1761_read_reg_cache(codec,ADAU_RECVLCL) 
				& RECVLC_DISABLE_MASK);
		adau1761_write_reg_byte(codec, ADAU_RECVLCL, regvalue);
		regvalue = (adau1761_read_reg_cache(codec,ADAU_RECVLCR)
				& RECVLC_DISABLE_MASK);
		adau1761_write_reg_byte(codec, ADAU_RECVLCR, regvalue);
		adau1761_write_reg_byte(codec, ADAU_RECMLC1, RECMLC_LINE_0DB);
		adau1761_write_reg_byte(codec, ADAU_RECMRC1, RECMLC_LINE_0DB);
	}

	return 0;
}

static int adau1761_mic_boost_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *privdata = codec->private_data;

	snd_printk_marker();
	if (privdata->in_source & CAP_MIC)
		ucontrol->value.integer.value[0] = 
		(RECMLC_MIC_20DB == 
			adau1761_read_reg_cache(codec,ADAU_RECMLC1));
	else
		ucontrol->value.integer.value[0] = 0x0;

	snd_printk_marker();
	ucontrol->value.integer.value[1] = ucontrol->value.integer.value[0];
	snd_printk_marker();
	return 0;
}

static int adau1761_mic_boost_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *privdata = codec->private_data;
	int val = ucontrol->value.integer.value[0];
	u8 regvalue=0;

	snd_printk_marker();
	if (privdata->in_source & CAP_MIC) {
		regvalue = (val) ? RECMLC_MIC_20DB:RECMLC_MIC_0DB;
		if (adau1761_read_reg_cache(codec,ADAU_RECMLC1) != regvalue) {
			adau1761_write_reg_byte(codec, ADAU_RECMLC1, regvalue);
			adau1761_write_reg_byte(codec, ADAU_RECMRC1, regvalue);
			return 1;
		}
	}

	return 0;
}

static const char *adau1761_input_select[] = {"Mic", "Line"};
static const struct soc_enum adau1761_enums[] = {
	SOC_ENUM_SINGLE(ADAU_RECMLC1-ADAU_FIRSTREG, 0, 2, adau1761_input_select),
};

static const struct snd_kcontrol_new adau1761_snd_controls[] = {
SOC_DOUBLE_R("Master Playback Volume", ADAU_DACCTL1-ADAU_FIRSTREG,
	ADAU_DACCTL2-ADAU_FIRSTREG, 0, 255, 1),
SOC_DOUBLE_R("Capture Volume", ADAU_ADCCTL1-ADAU_FIRSTREG,
	ADAU_ADCCTL2-ADAU_FIRSTREG, 0, 255, 1),
SOC_DOUBLE_R("Capture Switch", ADAU_RECMLC0-ADAU_FIRSTREG,
	ADAU_RECMRC0-ADAU_FIRSTREG, 0, 1, 0),
SOC_ENUM_EXT("Capture Source", adau1761_enums[0],
	adau1761_mux_get, adau1761_mux_put),
SOC_SINGLE_EXT("Mic Boost (+20dB)", ADAU_RECMLC1-ADAU_FIRSTREG, 0, 1, 0,
	adau1761_mic_boost_get, adau1761_mic_boost_put),
SOC_DOUBLE_R("Headphone Playback Volume", ADAU_PLBHPVL-ADAU_FIRSTREG,
	ADAU_PLBHPVR-ADAU_FIRSTREG, 2, 63, 0),
SOC_DOUBLE_R("Line Playback Volume", ADAU_PLBLOVL-ADAU_FIRSTREG,
	ADAU_PLBLOVR-ADAU_FIRSTREG, 2, 63, 0),
SOC_DOUBLE_R("Differential Input Volume", ADAU_RECVLCL-ADAU_FIRSTREG,
	ADAU_RECVLCL-ADAU_FIRSTREG, 2, 63, 0),
};

/* add non dapm controls */
static int adau1761_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(adau1761_snd_controls); i++) {
		if ((err = snd_ctl_add(codec->card,
			snd_soc_cnew(&adau1761_snd_controls[i],
				codec,NULL))) < 0)
			return err;
	}

	return 0;
}

/*
 * _DAPM_
 */

static int adau1761_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = 0;

	snd_printk_marker();
	dbg("adau1761_mute: mute(%d)", mute);

	if (mute) {
		/* mute outputs */
		reg = (adau1761_read_reg_cache(codec,ADAU_PLBMLC0) & 0xFE)|0x0;
 		adau1761_write_reg_byte(codec, ADAU_PLBMLC0, reg);
		reg = (adau1761_read_reg_cache(codec,ADAU_PLBMRC0) & 0xFE)|0x0;
		adau1761_write_reg_byte(codec, ADAU_PLBMRC0, reg);

		/* mute inputs */
		reg = (adau1761_read_reg_cache(codec,ADAU_RECMLC0) & 0xFE)|0x0;
		adau1761_write_reg_byte(codec, ADAU_RECMLC0, reg);
		reg = (adau1761_read_reg_cache(codec,ADAU_RECMRC0) & 0xFE)|0x0;
		adau1761_write_reg_byte(codec, ADAU_RECMRC0, reg);
	} else {
		/* un-mute inputs */
		reg = (adau1761_read_reg_cache(codec,ADAU_RECMLC0) & 0xFE)|0x1;
		adau1761_write_reg_byte(codec, ADAU_RECMLC0, reg);
		reg = (adau1761_read_reg_cache(codec,ADAU_RECMRC0) & 0xFE)|0x1;
		adau1761_write_reg_byte(codec, ADAU_RECMRC0, reg);

		/* un-mute outputs */	
		reg = (adau1761_read_reg_cache(codec,ADAU_PLBMLC0) & 0xFE)|0x1; 
		adau1761_write_reg_byte(codec, ADAU_PLBMLC0, reg);
		reg = (adau1761_read_reg_cache(codec,ADAU_PLBMRC0) & 0xFE)|0x1;
		adau1761_write_reg_byte(codec, ADAU_PLBMRC0, reg);
		
		/* Reset and bypass de-Jitter control */
		adau1761_write_reg_byte(codec, ADAU_DEJITTER, DEJITTER_BYPASS);
		/* Activate de-Jitter control */
		adau1761_write_reg_byte(codec, ADAU_DEJITTER, DEJITTER_ACTIVE);
	}

	return 0;
}

/*
 * Special event for power management changes,  Mute outputs during dapm 
 * up/down to minimize clicks/pops. Disable DSP and Clocks to save power.
 */
static int adau1761_output_event(struct snd_soc_dapm_widget *w, 
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct adau1761_priv *adau1761 = codec->private_data;
	u16 val;
	int mask = 0x03;
	u8 pwr_down = (event & SND_SOC_DAPM_PRE_PMD);

	snd_printk_marker();

	/* mute outputs before power up/down*/
	if (event & (SND_SOC_DAPM_PRE_PMD|SND_SOC_DAPM_PRE_PMU)) {
		dbg("adau1761_output_event: PRE-PWR");

		if (!pwr_down) {
			adau1761_write_reg_byte(codec, ADAU_CLK_EN1, CLK_EN1_SLAVE);
			adau1761_write_reg_byte(codec, ADAU_CLK_EN0, CLK_EN0_ON);
			adau1761_write_reg_byte(codec, ADAU_DSP_ENA, 0x01);
			adau1761_write_reg_byte(codec, ADAU_DSP_RUN, 0x01);
	
			/* enable digital mixer before power up*/
			adau1761_mute(&adau1761_dai,0);
		}

		val = (adau1761_read_reg_cache(codec,ADAU_PLBLOVL));
		adau1761->dapm_lineL = val;
 		adau1761_write_reg_byte(codec, ADAU_PLBLOVL, (val&mask));
		val = (adau1761_read_reg_cache(codec,ADAU_PLBLOVR));
		adau1761->dapm_lineR = val;
 		adau1761_write_reg_byte(codec, ADAU_PLBLOVR, (val&mask));
		val = (adau1761_read_reg_cache(codec,ADAU_PLBHPVL));
		adau1761->dapm_hpL = val|0x01;
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVL, (val&mask));
		val = (adau1761_read_reg_cache(codec,ADAU_PLBHPVR));
		adau1761->dapm_hpR = val|0x01;
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVR, (val&mask));
	}

	/* restore output levels after power up */
	if (event & SND_SOC_DAPM_POST_PMU) {
		dbg("adau1761_output_event: POST-PWRUP");

 		adau1761_write_reg_byte(codec, ADAU_PLBLOVL, adau1761->dapm_lineL);
 		adau1761_write_reg_byte(codec, ADAU_PLBLOVR, adau1761->dapm_lineR);
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVL, adau1761->dapm_hpL);
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVR, adau1761->dapm_hpR);
	}

	/* disable clock generators, restore volumes for alsa mixer */
	if (event & SND_SOC_DAPM_POST_PMD) {
		dbg("adau1761_output_event: POST-PWR-DOWN");

		adau1761_mute(&adau1761_dai,1);
		adau1761_write_reg_byte(codec, ADAU_DSP_RUN, 0x00);
		adau1761_write_reg_byte(codec, ADAU_DSP_ENA, 0x00);
 		adau1761_write_reg_byte(codec, ADAU_PLBLOVL, adau1761->dapm_lineL);
 		adau1761_write_reg_byte(codec, ADAU_PLBLOVR, adau1761->dapm_lineR);
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVL, adau1761->dapm_hpL&0xFE);
 		adau1761_write_reg_byte(codec, ADAU_PLBHPVR, adau1761->dapm_hpR&0xFE);
		adau1761_write_reg_byte(codec, ADAU_CLK_EN0, CLK_EN0_OFF);
		adau1761_write_reg_byte(codec, ADAU_CLK_EN1, CLK_EN1_OFF);
	}

	return 0;
}

/* Left Mixer */
static const struct snd_kcontrol_new adau1761_left_mixer_controls[] = {
SOC_DAPM_SINGLE("LineLeft Bypass Switch", ADAU_PLBLOVL-ADAU_FIRSTREG, 1, 1, 0),
SOC_DAPM_SINGLE("HPLeft Bypass Switch", ADAU_PLBHPVL-ADAU_FIRSTREG, 1, 1, 0),
};

/* Right mixer */
static const struct snd_kcontrol_new adau1761_right_mixer_controls[] = {
SOC_DAPM_SINGLE("LineRight Bypass Switch", ADAU_PLBLOVR-ADAU_FIRSTREG, 1, 1, 0),
SOC_DAPM_SINGLE("HPRight Bypass Switch", ADAU_PLBHPVR-ADAU_FIRSTREG, 1, 1, 0),
};

static const struct snd_soc_dapm_widget adau1761_dapm_widgets[] = {

SND_SOC_DAPM_PRE("Pre Mute", adau1761_output_event),
SND_SOC_DAPM_POST("Post Mute", adau1761_output_event),

SND_SOC_DAPM_MIXER("Left Mixer", ADAU_PLBPWRM-ADAU_FIRSTREG, 2, 1, \
	&adau1761_left_mixer_controls[0], ARRAY_SIZE(adau1761_left_mixer_controls)),
SND_SOC_DAPM_MIXER("Left Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Left Line Mixer", ADAU_PLBMLLO-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("LHPOUT"),

SND_SOC_DAPM_MIXER("Right Mixer", ADAU_PLBPWRM-ADAU_FIRSTREG, 3, 1, \
	&adau1761_right_mixer_controls[0], ARRAY_SIZE(adau1761_right_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 1, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Line Mixer", ADAU_PLBMRLO-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_OUTPUT("RHPOUT"),

//SND_SOC_DAPM_OUTPUT("MONOOUT"),

SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("DAC Enable Left", ADAU_DACCTL0-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("DAC Enable Right", ADAU_DACCTL0-ADAU_FIRSTREG, 1, 0, NULL, 0),
SND_SOC_DAPM_MIXER("HP Bias Left", ADAU_PLBPWRM-ADAU_FIRSTREG, 6, 1, NULL, 0),
SND_SOC_DAPM_MIXER("HP Bias Right", ADAU_PLBLRMC-ADAU_FIRSTREG, 0, 0, NULL, 0),

SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("ADC Left", ADAU_ADCCTL0-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("ADC Right", ADAU_ADCCTL0-ADAU_FIRSTREG, 1, 0, NULL, 0),

#if !defined(CONFIG_SND_ADAU1761_DIG_MIC)
SND_SOC_DAPM_MICBIAS("Mic Bias", SND_SOC_NOPM, 0, 0),
//SND_SOC_DAPM_MIXER("Left Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
//SND_SOC_DAPM_MIXER("Right Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
//SND_SOC_DAPM_MICBIAS("Mic Bias", ADAU_RECMBIA-ADAU_FIRSTREG, 0, 0),
SND_SOC_DAPM_MIXER("Left Mic Mixer", ADAU_RECVLCL-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Mic Mixer", ADAU_RECVLCR-ADAU_FIRSTREG, 0, 0, NULL, 0),
#else
SND_SOC_DAPM_MICBIAS("Mic Bias Left", SND_SOC_NOPM, 1, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias Right", SND_SOC_NOPM, 1, 0),
SND_SOC_DAPM_MIXER("Left Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
#endif

SND_SOC_DAPM_MIXER("Left Input", ADAU_RECPWRM-ADAU_FIRSTREG, 1, 1, NULL, 0),
SND_SOC_DAPM_MIXER("Right Input", ADAU_RECPWRM-ADAU_FIRSTREG, 2, 1, NULL, 0),

SND_SOC_DAPM_INPUT("LMICIN"),
SND_SOC_DAPM_INPUT("RMICIN"),
SND_SOC_DAPM_INPUT("LLINEIN"),
SND_SOC_DAPM_INPUT("RLINEIN"),
};

static const struct snd_soc_dapm_route audio_connections[] = {
	/* DAC */
	{"DAC Enable Left", NULL, "DAC"},
	{"DAC Enable Right", NULL, "DAC"},

	/* mixers */
	{"Left Mixer", NULL, "DAC Enable Left"},
	{"Right Mixer", NULL, "DAC Enable Right"},

	/* outputs */
	{"Left Out", NULL, "Left Mixer"},
	{"Right Out", NULL, "Right Mixer"},

	/* line Out */
	{"Left Line Mixer", NULL, "Left Out"},
	{"Right Line Mixer", NULL, "Right Out"},
	{"LOUT", "LineLeft Bypass Switch", "Left Line Mixer"},
	{"ROUT", "LineRight Bypass Switch", "Right Line Mixer"},

	/* headphone out */
	{"HP Bias Left", NULL, "Left Out"},
	{"HP Bias Right", NULL, "Right Out"},
	{"LHPOUT", "HPLeft Bypass Switch", "HP Bias Left"},
	{"RHPOUT", "HPRight Bypass Switch", "HP Bias Right"},

	/* inputs */
	{"Left Input", NULL, "LLINEIN"},
	{"Right Input", NULL, "RLINEIN"},
	{"Mic Bias", NULL, "LMICIN"},
	{"Mic Bias", NULL, "RMICIN"},
	{"Left Mic Mixer", NULL, "Mic Bias"},
	{"Right Mic Mixer", NULL, "Mic Bias"},
	{"ADC Left", NULL, "Left Input"},
	{"ADC Right", NULL, "Right Input"},
	{"ADC Left", NULL, "Left Mic Mixer"},
	{"ADC Right", NULL, "Right Mic Mixer"},
	{"ADC", NULL, "ADC Left"},
	{"ADC", NULL, "ADC Right"},
};

static int adau1761_add_widgets(struct snd_soc_codec *codec)
{
	int i;
	snd_printk_marker();

#if !defined(ADAU1761_DISABLE_DAPM)
	for(i = 0; i < ARRAY_SIZE(adau1761_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &adau1761_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, audio_connections, 
		ARRAY_SIZE(audio_connections));

	snd_soc_dapm_new_widgets(codec);
#endif
	return 0;
}

static int adau1761_pll_init(struct snd_soc_codec *codec, int enable)
{
	/* Clear dsp Run bit */	
	adau1761_write_reg_byte(codec, ADAU_DSP_RUN, 0x00);

	/* Init ADAU1761 clocking */
	adau1761_write_reg_byte(codec, ADAU_CLKCTRL,
		(CLKCTRL_SRC_MCLK | clkctrl_frq_infreq | CLKCTRL_DISABLE));

	if (enable) {
		adau1761_write_reg_byte(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_MCLK | clkctrl_frq_infreq | CLKCTRL_ENABLE));

		/* restore dsp state */
		adau1761_write_reg_byte(codec, ADAU_DSP_RUN, 0x01);
	}

	return 0;
}

static void adau1761_setprogram(struct snd_soc_codec *codec, int mode)
{
	int i = 0;
	unsigned int address = 0;

	snd_printk_marker();

	/* Set dsp enable bit before programming */
	if (0 < adau1761_mode_programs[mode].size) {
		adau1761_write_reg_byte(codec, ADAU_DSP_ENA, 0x01);
		adau1761_read_reg_byte(codec, ADAU_DSP_ENA);

		/* don't waste time writing program for adau1361 */
		if (0x01 != adau1761_read_reg_cache(codec, ADAU_DSP_ENA))
			return;
	}

 	/* Load Program - program memory is 5Bytes wide*/
	if (0 < adau1761_mode_programs[mode].size) {

		dbg("Writing mode program size = %d",
			 adau1761_mode_programs[mode].size);

		/* Write one address at a time */
		address = ADAU_PROGRAM;
		for (i=0; i<adau1761_mode_programs[mode].size; i+=5) {
			adau1761_write_reg_block(codec,address++,5,
				adau1761_mode_programs[mode].data+i);
		}
	}

	/* Load Parameters */
	if (0 < adau1761_mode_params[mode].size) {

		dbg("Writing mode param size = %d",
			 adau1761_mode_params[mode].size);

		/* Write one address at a time */
		address = ADAU_PARAMS;
		for (i=0; i<adau1761_mode_params[mode].size; i += 4) {
			adau1761_write_reg_block(codec, address++, 4,
				adau1761_mode_params[mode].data+i);
		}
	}
	
	/* Set dsp Run bit */
	if (0 < adau1761_mode_programs[mode].size)
		adau1761_write_reg_byte(codec, ADAU_DSP_RUN, 0x01);
}

static int adau1761_reg_init(struct snd_soc_codec *codec)
{
	adau1761_modereg_t regdata;
	adau1761_modereg_t* registers = 0;
	int i;

	snd_printk_marker();
	
	/* see if we are connected by writing to the chip */
	adau1761_write_reg_byte(codec, ADAU_CLKCTRL, 0x1);
	if (adau1761_read_reg_byte(codec, ADAU_CLKCTRL) != 0
		|| adau1761_read_reg_cache(codec,ADAU_CLKCTRL) != 0x1) {
		printk(KERN_ERR "ADAU1761 Card read test failed.\n");
		return -ENODEV;
	}

	/* initialize the PLL */
	if (adau1761_pll_init(codec, 1) != 0)
		return -ENODEV;

	/* Load deault regsiter settings */
	for (i=0; i<RESET_REGISTER_COUNT; ++i) {
		regdata = adau1761_reset[i];
		adau1761_write_reg_byte(codec, regdata.regaddress, regdata.regvalue);
	}
	
	/* Load mode registers */
#ifdef CONFIG_SND_ADAU1761_DIG_MIC
	registers = adau1761_mode_registers[1];
#else
	registers = adau1761_mode_registers[0];
#endif
	for (i=0; i<MODE_REGISTER_COUNT; ++i) {
		regdata = registers[i];
		adau1761_write_reg_byte(codec, regdata.regaddress, regdata.regvalue);
	}

	/* Load default program */
#ifdef CONFIG_SND_ADAU1761_DIG_MIC
	adau1761_setprogram(codec, 1);
#else
	adau1761_setprogram(codec, 0);
#endif

	/* unmute outputs */
	adau1761_write_reg_byte(codec, ADAU_PLBHPVL, DAPM_HP_DEF);
	adau1761_write_reg_byte(codec, ADAU_PLBHPVR, DAPM_HP_DEF);
	adau1761_write_reg_byte(codec, ADAU_PLBLOVL, DAPM_LINE_DEF);
	adau1761_write_reg_byte(codec, ADAU_PLBLOVR, DAPM_LINE_DEF);

	dumpRegisters(codec);
	return 0;
}

struct _srate_set {
	int fs;
	u8 reg;
};

static const struct _srate_set srate_iface[] = {
	{8000, 0x1},
	{11025, 0x2},
	{12000, 0x2},
	{16000, 0x3},
	{22050, 0x4},
	{24000, 0x4},
	{32000, 0x5},
	{44100, 0x0},
	{48000, 0x0},
	{88200, 0x6},
	{96000, 0x6},
};

static int adau1761_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u8 reg = 0;

	snd_printk_marker();

	adau1761_write_reg_byte(codec, ADAU_SER_SRT, reg);
	dbg("hw_params: ser-rate-reg(%02X)", reg);

	reg = (adau1761_read_reg_cache(codec,ADAU_CONVCT0) & 0xF8) | reg;
	adau1761_write_reg_byte(codec, ADAU_CONVCT0, reg);
	dbg("hw_params: rate-reg(%02X)", reg);

	/* Set DSP Rate to follow the serial rate */
	adau1761_write_reg_byte(codec, ADAU_DSP_SR1, 0x01);
	return 0;
}

static int adau1761_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	snd_printk_marker();

	/* set master/slave audio interface */
	reg = (adau1761_read_reg_cache(codec,ADAU_SPRTCT0) & 0xFE);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:  /*master*/
		dbg("adau1761_set_dai_fmt: Master");
		reg |= 0x1;
		adau1761_write_reg_byte(codec, ADAU_CLK_EN1, CLK_EN1_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /*slave*/
		dbg("adau1761_set_dai_fmt: slave");
		reg |= 0x0;
		adau1761_write_reg_byte(codec, ADAU_CLK_EN1, CLK_EN1_SLAVE);
		break;
	default:
		return 0;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dbg("adau1761_set_dai_fmt: Stereo I2S format");
		break;
	/* TODO: support TDM */
	default:
		return 0;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	/* TODO: support signal inversions */ 
	default:
		return 0;
	}

	/* set I2S iface format*/
	dbg("adau1761_set_dai_fmt: fmt:%d reg:%d", fmt,reg);
	adau1761_write_reg_byte(codec, ADAU_SPRTCT0, reg);
	return 0;
}

static int adau1761_set_dai_iis_bclk(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (div_id != ADAU1761_BCLK_ID)
		return -EINVAL;

	switch (div) {
	case 32:
		/*32bclocks frame, Left-justified*/
		adau1761_write_reg_byte(codec, ADAU_SPRTCT1, 0x21);
		break;
	case 48:
		/*48bclocks frame, Left-justified*/
		adau1761_write_reg_byte(codec, ADAU_SPRTCT1, 0x41);
		break;
	default:
		dbg("adau1761_set_dai_iis_bclk: undefined bclk rate:%d", div);
		return -EINVAL;
	}

	return 0;
}

static int adau1761_set_bias_level(struct snd_soc_codec *codec, 
	enum snd_soc_bias_level level)
{
	snd_printk_marker();

#if !defined(ADAU1761_DISABLE_DAPM)
	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
		dbg("dapm_event: SND_SOC_BIAS_ON (%d)", level);
		break;
	case SND_SOC_BIAS_PREPARE: /* partial On */
		adau1761_pll_init(codec, 1);
		dbg("dapm_event: SND_SOC_BIAS_PREPARE (%d)", level);
		break;
	case SND_SOC_BIAS_STANDBY: /* power save mode */
		adau1761_pll_init(codec, 0);
		/* Disable PLL and use MCLK src while idle */
		adau1761_write_reg_byte(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_MCLK | clkctrl_frq_infreq | CLKCTRL_ENABLE));
		adau1761_write_reg_byte(codec, ADAU_PLBCTRL, PLBCTRL_POP_LPWR);
		dbg("dapm_event: SND_SOC_BIAS_STANDBY (%d)", level);
		break;
	case SND_SOC_BIAS_OFF: /* Off */
		adau1761_write_reg_byte(codec, ADAU_RECPWRM, RECPWRM_LOW_PWR);
		adau1761_write_reg_byte(codec, ADAU_PLBPWRM, PLBPWRM_LOW_PWR);
		adau1761_write_reg_byte(codec, ADAU_PLBCTRL, PLBCTRL_POP_OFF);
		adau1761_write_reg_byte(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_MCLK | clkctrl_frq_infreq | CLKCTRL_DISABLE));
		dbg("dapm_event: SND_SOC_BIAS_OFF (%d)", level);
		break;
	}
#endif

	codec->bias_level = level;
	return 0;
}

#define ADAU1761_RATES (SNDRV_PCM_RATE_8000_96000)

#define ADAU1761_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai adau1761_dai = {
	.name = "ADAU1761",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ADAU1761_RATES,
		.formats = ADAU1761_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ADAU1761_RATES,
		.formats = ADAU1761_FORMATS,},
	.ops = {
		.hw_params = adau1761_hw_params,
		.digital_mute = adau1761_mute,
		.set_fmt = adau1761_set_dai_fmt,
		.set_clkdiv = adau1761_set_dai_iis_bclk,
	}
};
EXPORT_SYMBOL_GPL(adau1761_dai);

static int adau1761_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct adau1761_priv *privdata = codec->private_data;

	snd_printk_marker();
	privdata->dapm_state_suspend = codec->dapm_state;
	
	adau1761_set_bias_level(codec, SND_SOC_BIAS_OFF);
	pdev->dev.power.power_state.event = state.event;

	if (privdata->pdata->suspend)
		privdata->pdata->suspend();

	return 0;
}

static void adau1761_resume_wq_handler(struct work_struct *work)
{
	struct adau1761_priv *privdata = container_of(work, struct adau1761_priv, resume_work);
	struct snd_soc_codec *codec = privdata->codec;
	unsigned int i, v;

	snd_printk_marker();

	pr_info(AUDIO_NAME ": Resuming %s\n", codec->name);

	if (privdata->pdata->resume)
		privdata->pdata->resume();

	adau1761_pll_init(codec, 1);

	/* Load program */
#ifdef CONFIG_SND_ADAU1761_DIG_MIC
	adau1761_setprogram(codec, 1);
#else
	adau1761_setprogram(codec, 0);
#endif

	/* sync reg_cache with the hardware */
	for (i=ADAU_FIRSTREG; i<=ADAU_LASTREG; ++i) {
		/* skip over the 6byte PLL control register */
		if (i >= ADAU_CLKCTRL && i < ADAU_MICCTRL)
			continue;
		/* skip reserved registers */
		if (i > ADAU_MCLKPAD && i < ADAU_DSP_CRC)
			continue;

		v = adau1761_read_reg_cache(codec, i);
		if( adau1761_write_reg_byte(codec, i, v) != 0 ) {
			printk("ERROR WRITING %.4X AT REG %x\n", v, i);
			return;
		}
	}

	adau1761_write_reg_byte(codec, ADAU_PLBCTRL, PLBCTRL_POP_ON);
	adau1761_write_reg_byte(codec, ADAU_RECPWRM, RECPWRM_RUN_PWR);
	adau1761_write_reg_byte(codec, ADAU_PLBPWRM, PLBPWRM_RUN_PWR);

	adau1761_set_bias_level(codec, codec->suspend_dapm_state);
	privdata->pdev->dev.power.power_state.event = PM_EVENT_ON;

	pr_info(AUDIO_NAME ": Resumed %s\n", codec->name);
}

static int adau1761_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct adau1761_priv *privdata = codec->private_data;

	schedule_work(&privdata->resume_work);
	return 0;
}

/*
 * initialise the adau1761 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int adau1761_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	snd_printk_marker();

	codec->name = "ADAU1761";
	codec->owner = THIS_MODULE;
	codec->read = adau1761_read_reg_cache;
	codec->write = adau1761_write_reg_byte;
	codec->set_bias_level = adau1761_set_bias_level;
	codec->dai = &adau1761_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ADAU_NUMCACHEREG;
	codec->reg_cache = kzalloc(ADAU_NUMCACHEREG, GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR AUDIO_NAME ": failed to create pcms\n");
		goto pcm_err;
	}

	adau1761_add_controls(codec);
	adau1761_add_widgets(codec);

	snd_printk_marker();
	ret = adau1761_reg_init(codec);
	if (ret < 0) {
		printk(KERN_ERR AUDIO_NAME": failed initialize registers\n");
		goto card_err;
	}

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR AUDIO_NAME ": failed to register card\n");
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

static struct snd_soc_device *adau1761_socdev;

static int adau1761_i2c_probe(struct i2c_client *i2c, 
	const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = adau1761_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct adau1761_priv *codec_private_data = 
			(struct adau1761_priv *)codec->private_data;
	adau1761_platform_t	* pdata = 
			(adau1761_platform_t *)i2c->dev.platform_data;

	int ret;

	if (NULL != pdata) {
		codec_private_data->pdata = pdata;
	} else {
		printk(KERN_ERR AUDIO_NAME ": No platform data!\n");
		return -EINVAL;
	}

	/* Switch on both codec and amplifier */
	if ((ret = pdata->request_gpio())) {
		printk(KERN_ERR AUDIO_NAME ": Cannot request GPIO\n");
		return ret;
	}
	pdata->config_gpio();
	pdata->codec_pwr_en(1);
	pdata->amp_pwr_en(1);

	snd_printk_marker();
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	codec_private_data->codec = codec;
	INIT_WORK(&codec_private_data->resume_work, adau1761_resume_wq_handler);

	ret = adau1761_init(socdev);
	if (ret < 0)
		printk(KERN_ERR AUDIO_NAME ": Cannot register codec with ASoC\n");
	else
		printk(KERN_INFO AUDIO_NAME ":codec registered with ASoC\n");

	return ret;
}

static int adau1761_i2c_remove(struct i2c_client *client)
{
	struct adau1761 *adau1761 = i2c_get_clientdata(client);
	struct snd_soc_device *socdev = adau1761_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	adau1761_platform_t	* pdata = 
			(adau1761_platform_t *)client->dev.platform_data;

	snd_printk_marker();

	i2c_set_clientdata(client, NULL);
	codec->control_data = NULL;
	kfree(codec->reg_cache);
	kfree(adau1761);
	pdata->free_gpio();

	printk(AUDIO_NAME ": Codec unregistered\n");	
	return 0;
}

static int adau1761_i2c_suspend( struct i2c_client *client, pm_message_t mesg )
{
	adau1761_platform_t	* pdata = 
			(adau1761_platform_t *)client->dev.platform_data;

	if (pdata->suspend)
		pdata->suspend();
	return 0;
}

static int adau1761_i2c_resume( struct i2c_client *client )
{
	adau1761_platform_t	* pdata = 
			(adau1761_platform_t *)client->dev.platform_data;

	if (pdata->resume)
		pdata->resume();
	return 0;
}

static const struct i2c_device_id adau1761_i2c_id[] = {
	{ ADAU1761_DEVNAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1761_i2c_id);

static struct i2c_driver adau1761_i2c_driver = {
	.driver = {
		.name = ADAU1761_DEVNAME,
		.owner = THIS_MODULE,
	},
	.probe  =   adau1761_i2c_probe,
	.remove =   adau1761_i2c_remove,
	.suspend=   adau1761_i2c_suspend,
	.resume =   adau1761_i2c_resume,
	.id_table = adau1761_i2c_id,
};

static int adau1761_add_i2c_device(struct platform_device *pdev,
				 const struct adau1761_setup_data *setup)
{
	int ret;

	ret = i2c_add_driver(&adau1761_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

	return 0;

}

static int adau1761_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct adau1761_setup_data *setup;
	struct snd_soc_codec *codec;
	struct adau1761_priv *adau1761;
	int ret = 0;

	snd_printk_marker();

	info("ADAU1761 Audio Codec %s", ADAU1761_VERSION);

	switch (adau1761_rfs) {
		case 256:
			clkctrl_frq_infreq = CLKCTRL_FRQ_256;
			break;
		case 512:
			clkctrl_frq_infreq = CLKCTRL_FRQ_512;
			break;
		case 768:
			clkctrl_frq_infreq = CLKCTRL_FRQ_768;
			break;
			break;
		default:
			return -EINVAL;
			break;
	}

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	adau1761 = kzalloc(sizeof(struct adau1761_priv), GFP_KERNEL);
	if (adau1761 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	printk(AUDIO_NAME ": Setting private defaults\n");
	adau1761->in_source = CAP_MIC; /*default is mic input*/
	adau1761->dapm_lineL = DAPM_LINE_DEF;
	adau1761->dapm_lineR = DAPM_LINE_DEF;
	adau1761->dapm_hpL = DAPM_HP_DEF;
	adau1761->dapm_hpR = DAPM_HP_DEF;
	adau1761->pdev = pdev;

	codec->private_data = adau1761;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	adau1761_socdev = socdev;

	ret = -ENODEV;

	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->hw_read = (hw_read_t)i2c_master_recv;
	ret = adau1761_add_i2c_device(pdev, setup);

	if (ret != 0) {
		kfree(codec->private_data);
		kfree(codec);
	}
	return ret;
}

/* power down chip */
static int adau1761_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	snd_printk_marker();

	if (codec->control_data)
		adau1761_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	i2c_del_driver(&adau1761_i2c_driver);

	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adau1761 = {
	.probe = 	adau1761_probe,
	.remove = 	adau1761_remove,
	.suspend = 	adau1761_suspend,
	.resume =	adau1761_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_adau1761);

MODULE_DESCRIPTION("ASoC ADAU1761 driver");
MODULE_AUTHOR("John McCarty");
MODULE_LICENSE("GPL");
