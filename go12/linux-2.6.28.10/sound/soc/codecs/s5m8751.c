/*
 * s5m8751.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>

#include <linux/mfd/s5m8751/s5m8751_audio.h>
#include <linux/mfd/s5m8751/s5m8751_core.h>
#include "s5m8751.h"

#define S5M8751_VERSION "0.4"

struct s5m8751_priv {
	struct snd_soc_codec codec;
};

struct sample_rate {
	unsigned int rate;
	uint8_t reg_rate;
};

struct sample_rate s5m8751_rate[] = {
	{ 8000, S5M8751_SAMP_RATE_8K },
	{ 11025, S5M8751_SAMP_RATE_11_025K },
	{ 16000, S5M8751_SAMP_RATE_16K },
	{ 22050, S5M8751_SAMP_RATE_22_05K },
	{ 32000, S5M8751_SAMP_RATE_32K },
	{ 44100, S5M8751_SAMP_RATE_44_1K },
	{ 48000, S5M8751_SAMP_RATE_48K },
	{ 64000, S5M8751_SAMP_RATE_64K },
	{ 88200, S5M8751_SAMP_RATE_88_2K },
	{ 96000, S5M8751_SAMP_RATE_96K },
};

static struct s5m8751 *s5m8751;

static const unsigned int s5m8751_reg[S5M8751_NUMREGS];

static void dumpRegisters(struct snd_soc_codec *codec)
{
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
	int reg;
	int count = 0;
	unsigned int *cache = codec->reg_cache;
	for (reg = 0x00; reg <= S5M8751_NUMREGS; reg++) {
		if (count == 0) 
			printk("%02X", reg);

		switch (reg) {
			case S5M8751_DA_PDB1: // 0x17
			case S5M8751_DA_AMIX1: // 0x18
			case S5M8751_DA_AMIX2: // 0x19
			case S5M8751_DA_ANA: // 0x1A
			case S5M8751_DA_DWA: // 0x1B
			case S5M8751_DA_VOLL: // 0x1C
			case S5M8751_DA_VOLR: // 0x1D
			case S5M8751_DA_DIG1: // 0x1E
			case S5M8751_DA_DIG2: // 0x1F
			case S5M8751_DA_LIM1: // 0x20
			case S5M8751_DA_LIM2: // 0x21
			case S5M8751_DA_LOF: // 0x22
			case S5M8751_DA_ROF: // 0x23
			case S5M8751_DA_MUX: // 0x24
			case S5M8751_DA_LGAIN: // 0x25
			case S5M8751_DA_RGAIN: // 0x26
			case S5M8751_IN1_CTRL1: // 0x27
			case S5M8751_IN1_CTRL2: // 0x28
			case S5M8751_IN1_CTRL3: // 0x29
			case S5M8751_SLOT_L2: // 0x2A
			case S5M8751_SLOT_L1: // 0x2B
			case S5M8751_SLOT_R2: // 0x2C
			case S5M8751_SLOT_R1: // 0x2D
			case S5M8751_TSLOT: // 0x2E

			case S5M8751_SPK_SLOPE: // 0x30
			case S5M8751_SPK_DT: // 0x31
			case S5M8751_SPK_S2D: // 0x32
			case S5M8751_SPK_CM: // 0x33
			case S5M8751_SPK_DUM: // 0x34
			case S5M8751_HP_VOL1: // 0x35
			case S5M8751_HP_VOL2: // 0x36
			case S5M8751_AMP_EN: // 0x37
			case S5M8751_AMP_MUTE: // 0x38
			case S5M8751_AMP_CTRL: // 0x39
			case S5M8751_AMP_VMID: // 0x3A
			case S5M8751_LINE_CTRL: // 0x3B

			case S5M8751_AUDIO_STATUS: // 0x45

				printk(" %02X", cache[reg]);
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
 * read s5m8751 register cache
 */
static inline unsigned int s5m8751_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	unsigned int *cache = codec->reg_cache;
	BUG_ON(reg < S5M8751_DA_PDB1);
	BUG_ON(reg > S5M8751_LINE_CTRL);
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
//	printk("S5M8751 CR %02X %02X\n", reg, cache[reg]);
#endif
	return cache[reg];
}

/* Fills the data in cache and returns only operation status.
 * 1 for success, -EIO for failure.
 */
static int s5m8751_read(struct snd_soc_codec *codec,
				       unsigned int reg)
{
	u8 data;
	unsigned int *cache = codec->reg_cache;

	data = s5m8751_reg_read(s5m8751, reg);
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
//	printk("S5M8751 R  %02X %02X\n", reg, data);
#endif

	cache[reg] = data;	/* Update the cache */
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
//	printk("S5M8751 CW %02X %02X\n", reg, cache[reg]);
#endif

	return 1;
}

/*
 * write s5m8751 register cache
 */
static inline void s5m8751_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	unsigned int *cache = codec->reg_cache;

	if(reg == S5M8751_IN1_CTRL1)
		value &= ~(1<<6);	/* Reset is cleared automatically by the codec */

	cache[reg] = value;
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
//	printk("S5M8751 CW %02X %02X\n", reg, (u8)value);
#endif
}

/*
 * write to the S5M8751 register space
 */
/* Fills the data in cache and returns only operation status.
 * 1 for success, 0 for failure.
 */
static int s5m8751_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	BUG_ON(reg < S5M8751_DA_PDB1);
	BUG_ON(reg > S5M8751_LINE_CTRL);

	if(s5m8751_reg_write(s5m8751, reg, value))
		return 0;
#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
	printk("S5M8751 W  %02X %02X\n", reg, (u8)value);
#endif
	s5m8751_write_reg_cache(codec, reg, value);

#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
	switch(reg) {
		case S5M8751_DA_PDB1: // 0x17
		case S5M8751_DA_AMIX1: // 0x18
		case S5M8751_DA_AMIX2: // 0x19
		case S5M8751_IN1_CTRL1: // 0x27
		case S5M8751_AMP_EN: // 0x37
		case S5M8751_AMP_MUTE: // 0x38
			dumpRegisters(codec);
			break;
		default:
//		case S5M8751_DA_PDB1: // 0x17
//		case S5M8751_DA_AMIX1: // 0x18
//		case S5M8751_DA_AMIX2: // 0x19
		case S5M8751_DA_ANA: // 0x1A
		case S5M8751_DA_DWA: // 0x1B
		case S5M8751_DA_VOLL: // 0x1C
		case S5M8751_DA_VOLR: // 0x1D
		case S5M8751_DA_DIG1: // 0x1E
		case S5M8751_DA_DIG2: // 0x1F
		case S5M8751_DA_LIM1: // 0x20
		case S5M8751_DA_LIM2: // 0x21
		case S5M8751_DA_LOF: // 0x22
		case S5M8751_DA_ROF: // 0x23
		case S5M8751_DA_MUX: // 0x24
		case S5M8751_DA_LGAIN: // 0x25
		case S5M8751_DA_RGAIN: // 0x26
//		case S5M8751_IN1_CTRL1: // 0x27
		case S5M8751_IN1_CTRL2: // 0x28
		case S5M8751_IN1_CTRL3: // 0x29
		case S5M8751_SLOT_L2: // 0x2A
		case S5M8751_SLOT_L1: // 0x2B
		case S5M8751_SLOT_R2: // 0x2C
		case S5M8751_SLOT_R1: // 0x2D
		case S5M8751_TSLOT: // 0x2E

		case S5M8751_SPK_SLOPE: // 0x30
		case S5M8751_SPK_DT: // 0x31
		case S5M8751_SPK_S2D: // 0x32
		case S5M8751_SPK_CM: // 0x33
		case S5M8751_SPK_DUM: // 0x34
		case S5M8751_HP_VOL1: // 0x35
		case S5M8751_HP_VOL2: // 0x36
//		case S5M8751_AMP_EN: // 0x37
//		case S5M8751_AMP_MUTE: // 0x38
		case S5M8751_AMP_CTRL: // 0x39
		case S5M8751_AMP_VMID: // 0x3A
		case S5M8751_LINE_CTRL: // 0x3B
			break;
	}
#endif

	return 1;
}

static int s5m8751_outpga_put_volsw_vu(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int ret;
	u16 val;

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	val = s5m8751_read_reg_cache(codec, reg);
	return s5m8751_write(codec, reg, val);
}

#define SOC_S5M8751_SINGLE_R_TLV(xname, reg, shift, max, invert,\
	 tlv_array) {\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		  SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = s5m8751_outpga_put_volsw_vu, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert) }

static const DECLARE_TLV_DB_LINEAR(out_dac_tlv, 1200, -8400);

static const struct snd_kcontrol_new s5m8751_snd_controls[] = {

	SOC_S5M8751_SINGLE_R_TLV("Left DAC Digital Volume",
		S5M8751_DA_VOLL,
		S5M8751_DA_VOLL_SHIFT,
		S5M8751_DA_VOLL_MASK,
		0,
		out_dac_tlv),

	SOC_S5M8751_SINGLE_R_TLV("Right DAC Digital Volume",
		S5M8751_DA_VOLR,
		S5M8751_DA_VOLR_SHIFT,
		S5M8751_DA_VOLR_MASK,
		0,
		out_dac_tlv),

	SOC_SINGLE("Right Lineout mute control", S5M8751_AMP_MUTE,
		S5M8751_AMP_MUTE_RLINE_SHIFT, 1, 1),
	SOC_SINGLE("Left Lineout mute control", S5M8751_AMP_MUTE,
		S5M8751_AMP_MUTE_LLINE_SHIFT, 1, 1),
	SOC_SINGLE("Right Headphone mute control", S5M8751_AMP_MUTE,
		S5M8751_AMP_MUTE_RHP_SHIFT, 1, 1),
	SOC_SINGLE("Left Headphone mute control", S5M8751_AMP_MUTE,
		S5M8751_AMP_MUTE_LHP_SHIFT, 1, 1),
	SOC_SINGLE("Speaker PGA mute", S5M8751_AMP_MUTE,
		S5M8751_AMP_MUTE_SPK_S2D_SHIFT, 1, 1),

	SOC_DOUBLE_R("HeadPhone Volume", S5M8751_HP_VOL1, S5M8751_HP_VOL1,
								 0, 0x7f, 1),
	SOC_DOUBLE_R("Line-In Gain", S5M8751_DA_LGAIN, S5M8751_DA_RGAIN,
								 0, 0x7f, 1),
	SOC_DOUBLE_R("DAC Volume", S5M8751_DA_VOLL, S5M8751_DA_VOLR,
								 0, 0xff, 1),
	SOC_SINGLE("Speaker Slope", S5M8751_SPK_SLOPE, 0, 0xf, 1),
	SOC_SINGLE("DAC Mute", S5M8751_DA_DIG2, 7, 1, 0),
	SOC_SINGLE("Mute Dither", S5M8751_DA_DIG2, 6, 1, 0),
	SOC_SINGLE("Dither Level", S5M8751_DA_DIG2, 3, 7, 0),
	SOC_SINGLE("Soft Limit", S5M8751_DA_LIM1, 7, 1, 0),
	SOC_SINGLE("Limit Thrsh", S5M8751_DA_LIM1, 0, 0x7f, 1),
	SOC_SINGLE("Attack", S5M8751_DA_LIM2, 4, 0xf, 0),
	SOC_SINGLE("Release", S5M8751_DA_LIM2, 0, 0xf, 0),
	SOC_SINGLE("Speaker Preamp Gain", S5M8751_SPK_S2D, 4, 0x2, 0),
};

/* Add non-DAPM controls */
static int s5m8751_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(s5m8751_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&s5m8751_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/* Mono Analog Mixer */
static const struct snd_kcontrol_new s5m8751_dapm_monomix_controls[] = {
	SOC_DAPM_SINGLE("Mono Analog Mixer Left Mux", S5M8751_DA_AMIX2,
		S5M8751_LBYP_ML_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Mono Analog Mixer Left DAC", S5M8751_DA_AMIX1,
		S5M8751_LDAC_MM_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Mono Analog Mixer Right Mux", S5M8751_DA_AMIX2,
		S5M8751_RBYP_ML_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Mono Analog Mixer Right DAC", S5M8751_DA_AMIX1,
		S5M8751_RDAC_MM_SHIFT, 1, 0),
};

/* Left Analog Mixer */
static const struct snd_kcontrol_new s5m8751_dapm_leftmix_controls[] = {
	SOC_DAPM_SINGLE("Left Analog Mixer Left Mux", S5M8751_DA_AMIX2,
		S5M8751_LBYP_ML_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Left Analog Mixer Left DAC", S5M8751_DA_AMIX1,
		S5M8751_LDAC_ML_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Left Analog Mixer Right Mux", S5M8751_DA_AMIX2,
		S5M8751_RBYP_ML_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Left Analog Mixer Right DAC", S5M8751_DA_AMIX1,
		S5M8751_RDAC_ML_SHIFT, 1, 0),
};

/* Right Analog Mixer */
static const struct snd_kcontrol_new s5m8751_dapm_rightmix_controls[] = {
	SOC_DAPM_SINGLE("Right Analog Mixer Left Mux", S5M8751_DA_AMIX2,
		S5M8751_LBYP_MR_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Right Analog Mixer Left DAC", S5M8751_DA_AMIX1,
		S5M8751_LDAC_MR_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Right Analog Mixer Right Mux", S5M8751_DA_AMIX2,
		S5M8751_RBYP_MR_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("Right Analog Mixer Right DAC", S5M8751_DA_AMIX1,
		S5M8751_RDAC_MR_SHIFT, 1, 0),
};

static const struct snd_soc_dapm_widget s5m8751_dapm_widgets[] = {
	/* Input Side */
	/* Input Lines */
	SND_SOC_DAPM_INPUT("LIN_L1"),
	SND_SOC_DAPM_INPUT("LIN_L2"),
	SND_SOC_DAPM_INPUT("LIN_R1"),
	SND_SOC_DAPM_INPUT("LIN_R2"),

	/* Output Side */
	/* DAC */
	SND_SOC_DAPM_DAC("LDAC", "Left Playback", S5M8751_DA_PDB1,
		S5M8751_PDB_DACL, 0),
	SND_SOC_DAPM_DAC("RDAC", "Right Playback", S5M8751_DA_PDB1,
		S5M8751_PDB_DACR, 0),

	/* Mono Analog Mixer */
	SND_SOC_DAPM_MIXER("AMIX_M", S5M8751_DA_PDB1,
				S5M8751_PDB_MM_SHIFT, 0,
				&s5m8751_dapm_monomix_controls[0],
				ARRAY_SIZE(s5m8751_dapm_monomix_controls)),

	/* Left Analog Mixer */
	SND_SOC_DAPM_MIXER("AMIX_L", S5M8751_DA_PDB1,
				S5M8751_PDB_ML_SHIFT, 0,
				&s5m8751_dapm_leftmix_controls[0],
				ARRAY_SIZE(s5m8751_dapm_leftmix_controls)),

	/* Right Analog Mixer */
	SND_SOC_DAPM_MIXER("AMIX_R", S5M8751_DA_PDB1,
				S5M8751_PDB_MR_SHIFT, 0,
				&s5m8751_dapm_rightmix_controls[0],
				ARRAY_SIZE(s5m8751_dapm_rightmix_controls)),

	/* */
	SND_SOC_DAPM_PGA("Right Lineout Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_RLINE_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Lineout Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_LLINE_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Headphone Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_RHP_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Headphone Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_LHP_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Modulator Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_SPK_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Speaker Amplifier Enable", S5M8751_AMP_EN,
		S5M8751_AMP_EN_SPK_S2D_SHIFT, 0, NULL, 0),

	/* Output Lines */
	SND_SOC_DAPM_OUTPUT("RL"),
	SND_SOC_DAPM_OUTPUT("LL"),
	SND_SOC_DAPM_OUTPUT("RH"),
	SND_SOC_DAPM_OUTPUT("LH"),
	SND_SOC_DAPM_OUTPUT("SPK"),

	SND_SOC_DAPM_OUTPUT("Internal DAC Sink"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Make DACs turn on when playing even if not mixed into any outputs */
	{"Internal DAC Sink", NULL, "LDAC"},
	{"Internal DAC Sink", NULL, "RDAC"},

	//Registers to drive "Left/Right Input Mux" are not
	//in the datasheet of the codec so the widget is not
	//created in the codec driver.
	//To use Line in of the codec, implement the controls
	//{ "Left Input MUX", NULL, "LIN_L1_L2" },
	//{ "Left Input MUX", NULL, "LIN_L1" },
	//{ "Left Input MUX", NULL, "LIN_L2" },

	//{ "Right Input MUX", NULL, "LIN_R1_R2" },
	//{ "Right Input MUX", NULL, "LIN_R1" },
	//{ "Right Input MUX", NULL, "LIN_R2" },

	//{ "Mono Analog Mixer", "Mono Analog Mixer Left Mux",
	//					"Left Input MUX" },
	{ "AMIX_M", "Mono Analog Mixer Left DAC", 
						"LDAC" },
	//{ "Mono Analog Mixer", "Mono Analog Mixer Right Mux",
	//					"Right Input MUX" },
	{ "AMIX_M", "Mono Analog Mixer Right DAC",
						"RDAC" },

	//{ "Left Analog Mixer", "Left Analog Mixer Left Mux",
	//					"Left Input MUX" },
	{ "AMIX_L", "Left Analog Mixer Left DAC",
						"LDAC" },
	//{ "Left Analog Mixer", "Left Analog Mixer Right Mux",
	//					"Right Input MUX" },
	{ "AMIX_L", "Left Analog Mixer Right DAC",
						"RDAC" },

	//{ "Right Analog Mixer", "Right Analog Mixer Left Mux",
	//					"Left Input MUX" },
	{ "AMIX_R", "Right Analog Mixer Left DAC",
						"LDAC" },
	//{ "Right Analog Mixer", "Right Analog Mixer Right Mux",
	//					"Right Input MUX" },
	{ "AMIX_R", "Right Analog Mixer Right DAC",
						"RDAC" },

	{ "Right Lineout Enable", NULL, "AMIX_R" },
	{ "Left Lineout Enable", NULL, "AMIX_L" },
	{ "Left Headphone Enable", NULL, "AMIX_L" },
	{ "Right Headphone Enable", NULL, "AMIX_R" },

	{"Speaker Modulator Enable", NULL, "AMIX_M"},

	{ "Speaker Amplifier Enable", NULL, "Speaker Modulator Enable" },

	{ "RL", NULL,  "Right Lineout Enable" },
	{ "LL", NULL,  "Left Lineout Enable" },
	{ "RH", NULL,  "Right Headphone Enable" },
	{ "LH", NULL,  "Left Headphone Enable" },
	{ "SPK", NULL, "Speaker Amplifier Enable" },
};

static int s5m8751_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, s5m8751_dapm_widgets,
				  ARRAY_SIZE(s5m8751_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_new_widgets(codec);

	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int s5m8751_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	int count, ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	unsigned char in1_ctrl2, in1_ctrl1, amp_mute;

	amp_mute = S5M8751_ALL_AMP_MUTE;
	s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);

	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	in1_ctrl1 &= ~S5M8751_SAMP_RATE1_MASK;	/* Sampling Rate field */

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~S5M8751_I2S_DL1_MASK;	 /* I2S data length field */

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		in1_ctrl2 |= S5M8751_I2S_DL_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		in1_ctrl2 |= S5M8751_I2S_DL_18BIT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		in1_ctrl2 |= S5M8751_I2S_DL_20BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		in1_ctrl2 |= S5M8751_I2S_DL_24BIT;
		break;
	default:
		return -EINVAL;
	}

	for (count = 0 ; count < ARRAY_SIZE(s5m8751_rate) ; count++) {
		if (params_rate(params) == s5m8751_rate[count].rate) {
			in1_ctrl1 |= s5m8751_rate[count].reg_rate;
			ret = 0;
			break;
		} else
			ret = -EINVAL;
	}

	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);

	return ret;
}

static int s5m8751_set_dai_fmt(struct snd_soc_dai *codec_dai,
							unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned char in1_ctrl2, in1_ctrl1, amp_mute;

	amp_mute = S5M8751_ALL_AMP_MUTE;
	s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~S5M8751_I2S_DF1_MASK;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		in1_ctrl2 |= S5M8751_I2S_STANDARD;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		in1_ctrl2 |= S5M8751_I2S_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		in1_ctrl2 |= S5M8751_I2S_RJ;
		break;
	default:
		printk(KERN_ERR "DAIFmt(%d) not supported!\n", fmt
					& SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	in1_ctrl2 &= ~S5M8751_LRCK_POL1_MASK;
	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	in1_ctrl1 &= ~S5M8751_SCK_POL1_MASK;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		in1_ctrl1 &= ~S5M8751_SCK_POL;
		in1_ctrl2 &= ~S5M8751_LRCK_POL;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		in1_ctrl1 |= S5M8751_SCK_POL;
		in1_ctrl2 |= S5M8751_LRCK_POL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		in1_ctrl1 |= S5M8751_SCK_POL;
		in1_ctrl2 &= ~S5M8751_LRCK_POL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		in1_ctrl1 &= ~S5M8751_SCK_POL;
		in1_ctrl2 |= S5M8751_LRCK_POL;
		break;
	default:
		printk(KERN_ERR "Inv-combo(%d) not supported!\n", fmt &
					 SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);
	return 0;
}

static int s5m8751_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int val)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned char in1_ctrl1, in1_ctrl2, amp_mute;

	amp_mute = S5M8751_ALL_AMP_MUTE;
	s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~S5M8751_I2S_XFS1_MASK;

	switch (div_id) {
	case S5M8751_BCLK:
		switch (val) {
		case 32:
			in1_ctrl2 |= S5M8751_I2S_SCK_32FS;
			break;
		case 48:
			in1_ctrl2 |= S5M8751_I2S_SCK_48FS;
			break;
		case 64:
			in1_ctrl2 |= S5M8751_I2S_SCK_64FS;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);

	return 0;
}

static int s5m8751_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	uint8_t reg;

	reg = s5m8751_read_reg_cache(codec, S5M8751_DA_DIG2);
	if (mute)
		reg |= S5M8751_DAC_MUTE;
	else
		reg &= ~S5M8751_DAC_MUTE;

	s5m8751_write(codec, S5M8751_DA_DIG2, reg);
	s5m8751_write(codec, S5M8751_AMP_MUTE, S5M8751_SPK_S2D_MUTEB |
				S5M8751_LHP_MUTEB | S5M8751_RHP_MUTEB);

	return 0;
}

static int s5m8751_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	u8 da_pdb1, amp_en, amp_mute;

	switch (level) {
	case SND_SOC_BIAS_ON:
		amp_mute = (S5M8751_LHP_MUTEB | S5M8751_RHP_MUTEB) |
					S5M8751_SPK_S2D_MUTEB;
		amp_en = S5M8751_SPK_AMP_EN;
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);
		s5m8751_write(codec, S5M8751_AMP_EN, amp_en);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_PREPARE:
		amp_mute = S5M8751_ALL_AMP_MUTE;
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);
		break;
	case SND_SOC_BIAS_OFF:
		amp_en = S5M8751_ALL_AMP_OFF;
		amp_mute = S5M8751_ALL_AMP_MUTE;
		da_pdb1 = S5M8751_ALL_DA_PDB1_OFF | S5M8751_VMID_250K;
		s5m8751_write(codec, S5M8751_DA_PDB1, da_pdb1);
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute);
		s5m8751_write(codec, S5M8751_AMP_EN, amp_en);
		break;
	}

	codec->bias_level = level;
	return 0;
}

#define S5M8751_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai s5m8751_dai = {
	.name = "S5M8751-I2S",
	.id = 0,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000 | SNDRV_PCM_RATE_KNOT,
		.formats = S5M8751_FORMATS,
	},
	.ops = {
		.hw_params = s5m8751_hw_params,
		.set_fmt = s5m8751_set_dai_fmt,
		.set_clkdiv = s5m8751_set_dai_clkdiv,
		.digital_mute = s5m8751_digital_mute,
	},
};
EXPORT_SYMBOL_GPL(s5m8751_dai);

/*
 * initialise the S5M8751 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int s5m8751_init(struct snd_soc_device *socdev)
{
	int ret = 0, val;
	u16 *cache;
	struct snd_soc_codec *codec = socdev->codec;

	codec->name = "S5M8751";
	codec->owner = THIS_MODULE;
	codec->read = s5m8751_read_reg_cache;
	codec->write = s5m8751_write;
	codec->set_bias_level = s5m8751_set_bias_level;
	codec->dai = &s5m8751_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(s5m8751_reg);
	codec->reg_cache = kmemdup(s5m8751_reg, sizeof(s5m8751_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Fill the reg cache */
	cache = codec->reg_cache;
	for(val=S5M8751_DA_PDB1; val<=S5M8751_LINE_CTRL; val++){ /* Don't use Power Mngmnt regs here */
		while(s5m8751_read(codec, val) == -EIO)
			printk(KERN_WARNING "Read failed! ");
	}

	val = s5m8751_read_reg_cache(codec, S5M8751_AMP_CTRL);
	s5m8751_write(codec, S5M8751_AMP_CTRL, val |
						S5M8751_HP_AMP_MUTE_CONTROL_ON);

	s5m8751_write(codec, S5M8751_DA_PDB1, S5M8751_DA_PDB1_SPK |
						S5M8751_VMID_50K);
	s5m8751_write(codec, S5M8751_DA_AMIX1, S5M8751_DA_AMIX1_SPK);

	s5m8751_write(codec, S5M8751_DA_AMIX2, 0x00);

	s5m8751_write(codec, S5M8751_DA_VOLL, S5M8751_DAC_VOLL_DEFAULT); 
	s5m8751_write(codec, S5M8751_DA_VOLR,  S5M8751_DAC_VOLR_DEFAULT);
	s5m8751_write(codec, S5M8751_SPK_SLOPE, S5M8751_SPK_SLOPE_12); 
	s5m8751_write(codec, S5M8751_SPK_DT, S5M8751_SPK_DT_4ns);
	s5m8751_write(codec, S5M8751_SPK_S2D, S5M8751_SPK_S2D_6dB);

	val = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	val &= ~S5M8751_LOGIC_PDN;
	s5m8751_write(codec, S5M8751_IN1_CTRL1, val);
	val |= S5M8751_LOGIC_PDN;
	s5m8751_write(codec, S5M8751_IN1_CTRL1, val);
	s5m8751_write(codec, S5M8751_AMP_EN, S5M8751_SPK_AMP_EN);

	/* TODO XXX What about DA_ANA, DA_DWA? TODO XXX */
	codec->bias_level = SND_SOC_BIAS_OFF;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1,
			       SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "s5m8751: failed to create pcms\n");
		goto pcm_err;
	}

	s5m8751_add_controls(codec);
	s5m8751_add_widgets(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "s5m8751: failed to register card\n");
		goto card_err;
	}
	dumpRegisters(codec);
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
static ssize_t s5m8751_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	static uint8_t old_da_pdb1  ;
	static uint8_t old_da_amix1 ;
	static uint8_t old_da_amix2 ;
	static uint8_t old_da_ana   ;
	static uint8_t old_da_dwa   ;
	static uint8_t old_da_voll  ;
	static uint8_t old_da_volr  ;
	static uint8_t old_da_dig1  ;
	static uint8_t old_da_dig2  ;
	static uint8_t old_da_lim1  ;
	static uint8_t old_da_lim2  ;
	static uint8_t old_da_lof   ;
	static uint8_t old_da_rof   ;
	static uint8_t old_da_mux   ;
	static uint8_t old_da_lgain ;
	static uint8_t old_da_rgain ;
	static uint8_t old_in1_ctrl1;
	static uint8_t old_in1_ctrl2;
	static uint8_t old_in1_ctrl3;
	static uint8_t old_slot_l2  ;
	static uint8_t old_slot_l1  ;
	static uint8_t old_slot_r2  ;
	static uint8_t old_slot_r1  ;
	static uint8_t old_tslot    ;
	static uint8_t old_spk_slope;
	static uint8_t old_spk_dt   ;
	static uint8_t old_spk_s2d  ;
	static uint8_t old_spk_cm   ;
	static uint8_t old_spk_dum  ;
	static uint8_t old_hp_vol1  ;
	static uint8_t old_hp_vol2  ;
	static uint8_t old_amp_en   ;
	static uint8_t old_amp_mute ;
	static uint8_t old_amp_ctrl ;
	static uint8_t old_amp_vmid ;
	static uint8_t old_line_ctrl;

	uint8_t da_pdb1   = s5m8751_reg_read(s5m8751, S5M8751_DA_PDB1   );
	uint8_t da_amix1  = s5m8751_reg_read(s5m8751, S5M8751_DA_AMIX1  );
	uint8_t da_amix2  = s5m8751_reg_read(s5m8751, S5M8751_DA_AMIX2  );
	uint8_t da_ana    = s5m8751_reg_read(s5m8751, S5M8751_DA_ANA    );
	uint8_t da_dwa    = s5m8751_reg_read(s5m8751, S5M8751_DA_DWA    );
	uint8_t da_voll   = s5m8751_reg_read(s5m8751, S5M8751_DA_VOLL   );
	uint8_t da_volr   = s5m8751_reg_read(s5m8751, S5M8751_DA_VOLR   );
	uint8_t da_dig1   = s5m8751_reg_read(s5m8751, S5M8751_DA_DIG1   );
	uint8_t da_dig2   = s5m8751_reg_read(s5m8751, S5M8751_DA_DIG2   );
	uint8_t da_lim1   = s5m8751_reg_read(s5m8751, S5M8751_DA_LIM1   );
	uint8_t da_lim2   = s5m8751_reg_read(s5m8751, S5M8751_DA_LIM2   );
	uint8_t da_lof    = s5m8751_reg_read(s5m8751, S5M8751_DA_LOF    );
	uint8_t da_rof    = s5m8751_reg_read(s5m8751, S5M8751_DA_ROF    );
	uint8_t da_mux    = s5m8751_reg_read(s5m8751, S5M8751_DA_MUX    );
	uint8_t da_lgain  = s5m8751_reg_read(s5m8751, S5M8751_DA_LGAIN  );
	uint8_t da_rgain  = s5m8751_reg_read(s5m8751, S5M8751_DA_RGAIN  );
	uint8_t in1_ctrl1 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL1 );
	uint8_t in1_ctrl2 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL2 );
	uint8_t in1_ctrl3 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL3 );
	uint8_t slot_l2   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_L2   );
	uint8_t slot_l1   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_L1   );
	uint8_t slot_r2   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_R2   );
	uint8_t slot_r1   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_R1   );
	uint8_t tslot     = s5m8751_reg_read(s5m8751, S5M8751_TSLOT     );
	uint8_t spk_slope = s5m8751_reg_read(s5m8751, S5M8751_SPK_SLOPE );
	uint8_t spk_dt    = s5m8751_reg_read(s5m8751, S5M8751_SPK_DT    );
	uint8_t spk_s2d   = s5m8751_reg_read(s5m8751, S5M8751_SPK_S2D   );
	uint8_t spk_cm    = s5m8751_reg_read(s5m8751, S5M8751_SPK_CM    );
	uint8_t spk_dum   = s5m8751_reg_read(s5m8751, S5M8751_SPK_DUM   );
	uint8_t hp_vol1   = s5m8751_reg_read(s5m8751, S5M8751_HP_VOL1   );
	uint8_t hp_vol2   = s5m8751_reg_read(s5m8751, S5M8751_HP_VOL2   );
	uint8_t amp_en    = s5m8751_reg_read(s5m8751, S5M8751_AMP_EN    );
	uint8_t amp_mute  = s5m8751_reg_read(s5m8751, S5M8751_AMP_MUTE  );
	uint8_t amp_ctrl  = s5m8751_reg_read(s5m8751, S5M8751_AMP_CTRL  );
	uint8_t amp_vmid  = s5m8751_reg_read(s5m8751, S5M8751_AMP_VMID  );
	uint8_t line_ctrl = s5m8751_reg_read(s5m8751, S5M8751_LINE_CTRL );

	ssize_t len = 0;

	len += sprintf(buf + len, "------ DAC control registers ---------------\n");
	len += sprintf(buf + len, "DA_PDB1   (0x17) 0x%02x\n", da_pdb1   );
	len += sprintf(buf + len, "DA_AMIX1  (0x18) 0x%02x\n", da_amix1  );
	len += sprintf(buf + len, "DA_AMIX2  (0x19) 0x%02x\n", da_amix2  );
	len += sprintf(buf + len, "DA_ANA    (0x1A) 0x%02x\n", da_ana    );
	len += sprintf(buf + len, "DA_DWA    (0x1B) 0x%02x\n", da_dwa    );
	len += sprintf(buf + len, "DA_VOLL   (0x1C) 0x%02x\n", da_voll   );
	len += sprintf(buf + len, "DA_VOLR   (0x1D) 0x%02x\n", da_volr   );
	len += sprintf(buf + len, "DA_DIG1   (0x1E) 0x%02x\n", da_dig1   );
	len += sprintf(buf + len, "DA_DIG2   (0x1F) 0x%02x\n", da_dig2   );
	len += sprintf(buf + len, "DA_LIM1   (0x20) 0x%02x\n", da_lim1   );
	len += sprintf(buf + len, "DA_LIM2   (0x21) 0x%02x\n", da_lim2   );
	len += sprintf(buf + len, "DA_LOF    (0x22) 0x%02x\n", da_lof    );
	len += sprintf(buf + len, "DA_ROF    (0x23) 0x%02x\n", da_rof    );
	len += sprintf(buf + len, "DA_MUX    (0x24) 0x%02x\n", da_mux    );
	len += sprintf(buf + len, "DA_LGAIN  (0x25) 0x%02x\n", da_lgain  );
	len += sprintf(buf + len, "DA_RGAIN  (0x26) 0x%02x\n", da_rgain  );
	len += sprintf(buf + len, "------ Interface control registers ---------\n");
	len += sprintf(buf + len, "IN1_CTRL1 (0x27) 0x%02x\n", in1_ctrl1 );
	len += sprintf(buf + len, "IN1_CTRL2 (0x28) 0x%02x\n", in1_ctrl2 );
	len += sprintf(buf + len, "IN1_CTRL3 (0x29) 0x%02x\n", in1_ctrl3 );
	len += sprintf(buf + len, "SLOT_L2   (0x2A) 0x%02x\n", slot_l2   );
	len += sprintf(buf + len, "SLOT_L1   (0x2B) 0x%02x\n", slot_l1   );
	len += sprintf(buf + len, "SLOT_R2   (0x2C) 0x%02x\n", slot_r2   );
	len += sprintf(buf + len, "SLOT_R1   (0x2D) 0x%02x\n", slot_r1   );
	len += sprintf(buf + len, "TSLOT     (0x2E) 0x%02x\n", tslot     );
	len += sprintf(buf + len, "------  Audio amp control registers --------\n");
	len += sprintf(buf + len, "SPK_SLOPE (0x30) 0x%02x\n", spk_slope );
	len += sprintf(buf + len, "SPK_DT    (0x31) 0x%02x\n", spk_dt    );
	len += sprintf(buf + len, "SPK_S2D   (0x32) 0x%02x\n", spk_s2d   );
	len += sprintf(buf + len, "SPK_CM    (0x33) 0x%02x\n", spk_cm    );
	len += sprintf(buf + len, "SPK_DUM   (0x34) 0x%02x\n", spk_dum   );
	len += sprintf(buf + len, "HP_VOL1   (0x35) 0x%02x\n", hp_vol1   );
	len += sprintf(buf + len, "HP_VOL2   (0x36) 0x%02x\n", hp_vol2   );
	len += sprintf(buf + len, "AMP_EN    (0x37) 0x%02x\n", amp_en    );
	len += sprintf(buf + len, "AMP_MUTE  (0x38) 0x%02x\n", amp_mute  );
	len += sprintf(buf + len, "AMP_CTRL  (0x39) 0x%02x\n", amp_ctrl  );
	len += sprintf(buf + len, "AMP_VMID  (0x3A) 0x%02x\n", amp_vmid  );
	len += sprintf(buf + len, "LINE_CTRL (0x3B) 0x%02x\n", line_ctrl );
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "------  Registers that changed -------------\n");

	if (old_da_pdb1   != da_pdb1  ) len += sprintf(buf + len, "DA_PDB1   old 0x%02x new 0x%02x\n", old_da_pdb1  , da_pdb1   );
	if (old_da_amix1  != da_amix1 ) len += sprintf(buf + len, "DA_AMIX1  old 0x%02x new 0x%02x\n", old_da_amix1 , da_amix1  );
	if (old_da_amix2  != da_amix2 ) len += sprintf(buf + len, "DA_AMIX2  old 0x%02x new 0x%02x\n", old_da_amix2 , da_amix2  );
	if (old_da_ana    != da_ana   ) len += sprintf(buf + len, "DA_ANA    old 0x%02x new 0x%02x\n", old_da_ana   , da_ana    );
	if (old_da_dwa    != da_dwa   ) len += sprintf(buf + len, "DA_DWA    old 0x%02x new 0x%02x\n", old_da_dwa   , da_dwa    );
	if (old_da_voll   != da_voll  ) len += sprintf(buf + len, "DA_VOLL   old 0x%02x new 0x%02x\n", old_da_voll  , da_voll   );
	if (old_da_volr   != da_volr  ) len += sprintf(buf + len, "DA_VOLR   old 0x%02x new 0x%02x\n", old_da_volr  , da_volr   );
	if (old_da_dig1   != da_dig1  ) len += sprintf(buf + len, "DA_DIG1   old 0x%02x new 0x%02x\n", old_da_dig1  , da_dig1   );
	if (old_da_dig2   != da_dig2  ) len += sprintf(buf + len, "DA_DIG2   old 0x%02x new 0x%02x\n", old_da_dig2  , da_dig2   );
	if (old_da_lim1   != da_lim1  ) len += sprintf(buf + len, "DA_LIM1   old 0x%02x new 0x%02x\n", old_da_lim1  , da_lim1   );
	if (old_da_lim2   != da_lim2  ) len += sprintf(buf + len, "DA_LIM2   old 0x%02x new 0x%02x\n", old_da_lim2  , da_lim2   );
	if (old_da_lof    != da_lof   ) len += sprintf(buf + len, "DA_LOF    old 0x%02x new 0x%02x\n", old_da_lof   , da_lof    );
	if (old_da_rof    != da_rof   ) len += sprintf(buf + len, "DA_ROF    old 0x%02x new 0x%02x\n", old_da_rof   , da_rof    );
	if (old_da_mux    != da_mux   ) len += sprintf(buf + len, "DA_MUX    old 0x%02x new 0x%02x\n", old_da_mux   , da_mux    );
	if (old_da_lgain  != da_lgain ) len += sprintf(buf + len, "DA_LGAIN  old 0x%02x new 0x%02x\n", old_da_lgain , da_lgain  );
	if (old_da_rgain  != da_rgain ) len += sprintf(buf + len, "DA_RGAIN  old 0x%02x new 0x%02x\n", old_da_rgain , da_rgain  );
	if (old_in1_ctrl1 != in1_ctrl1) len += sprintf(buf + len, "IN1_CTRL1 old 0x%02x new 0x%02x\n", old_in1_ctrl1, in1_ctrl1 );
	if (old_in1_ctrl2 != in1_ctrl2) len += sprintf(buf + len, "IN1_CTRL2 old 0x%02x new 0x%02x\n", old_in1_ctrl2, in1_ctrl2 );
	if (old_in1_ctrl3 != in1_ctrl3) len += sprintf(buf + len, "IN1_CTRL3 old 0x%02x new 0x%02x\n", old_in1_ctrl3, in1_ctrl3 );
	if (old_slot_l2   != slot_l2  ) len += sprintf(buf + len, "SLOT_L2   old 0x%02x new 0x%02x\n", old_slot_l2  , slot_l2   );
	if (old_slot_l1   != slot_l1  ) len += sprintf(buf + len, "SLOT_L1   old 0x%02x new 0x%02x\n", old_slot_l1  , slot_l1   );
	if (old_slot_r2   != slot_r2  ) len += sprintf(buf + len, "SLOT_R2   old 0x%02x new 0x%02x\n", old_slot_r2  , slot_r2   );
	if (old_slot_r1   != slot_r1  ) len += sprintf(buf + len, "SLOT_R1   old 0x%02x new 0x%02x\n", old_slot_r1  , slot_r1   );
	if (old_tslot     != tslot    ) len += sprintf(buf + len, "TSLOT     old 0x%02x new 0x%02x\n", old_tslot    , tslot     );
	if (old_spk_slope != spk_slope) len += sprintf(buf + len, "SPK_SLOPE old 0x%02x new 0x%02x\n", old_spk_slope, spk_slope );
	if (old_spk_dt    != spk_dt   ) len += sprintf(buf + len, "SPK_DT    old 0x%02x new 0x%02x\n", old_spk_dt   , spk_dt    );
	if (old_spk_s2d   != spk_s2d  ) len += sprintf(buf + len, "SPK_S2D   old 0x%02x new 0x%02x\n", old_spk_s2d  , spk_s2d   );
	if (old_spk_cm    != spk_cm   ) len += sprintf(buf + len, "SPK_CM    old 0x%02x new 0x%02x\n", old_spk_cm   , spk_cm    );
	if (old_spk_dum   != spk_dum  ) len += sprintf(buf + len, "SPK_DUM   old 0x%02x new 0x%02x\n", old_spk_dum  , spk_dum   );
	if (old_hp_vol1   != hp_vol1  ) len += sprintf(buf + len, "HP_VOL1   old 0x%02x new 0x%02x\n", old_hp_vol1  , hp_vol1   );
	if (old_hp_vol2   != hp_vol2  ) len += sprintf(buf + len, "HP_VOL2   old 0x%02x new 0x%02x\n", old_hp_vol2  , hp_vol2   );
	if (old_amp_en    != amp_en   ) len += sprintf(buf + len, "AMP_EN    old 0x%02x new 0x%02x\n", old_amp_en   , amp_en    );
	if (old_amp_mute  != amp_mute ) len += sprintf(buf + len, "AMP_MUTE  old 0x%02x new 0x%02x\n", old_amp_mute , amp_mute  );
	if (old_amp_ctrl  != amp_ctrl ) len += sprintf(buf + len, "AMP_CTRL  old 0x%02x new 0x%02x\n", old_amp_ctrl , amp_ctrl  );
	if (old_amp_vmid  != amp_vmid ) len += sprintf(buf + len, "AMP_VMID  old 0x%02x new 0x%02x\n", old_amp_vmid , amp_vmid  );
	if (old_line_ctrl != line_ctrl) len += sprintf(buf + len, "LINE_CTRL old 0x%02x new 0x%02x\n", old_line_ctrl, line_ctrl );

	old_da_pdb1   = da_pdb1  ;
	old_da_amix1  = da_amix1 ;
	old_da_amix2  = da_amix2 ;
	old_da_ana    = da_ana   ;
	old_da_dwa    = da_dwa   ;
	old_da_voll   = da_voll  ;
	old_da_volr   = da_volr  ;
	old_da_dig1   = da_dig1  ;
	old_da_dig2   = da_dig2  ;
	old_da_lim1   = da_lim1  ;
	old_da_lim2   = da_lim2  ;
	old_da_lof    = da_lof   ;
	old_da_rof    = da_rof   ;
	old_da_mux    = da_mux   ;
	old_da_lgain  = da_lgain ;
	old_da_rgain  = da_rgain ;
	old_in1_ctrl1 = in1_ctrl1;
	old_in1_ctrl2 = in1_ctrl2;
	old_in1_ctrl3 = in1_ctrl3;
	old_slot_l2   = slot_l2  ;
	old_slot_l1   = slot_l1  ;
	old_slot_r2   = slot_r2  ;
	old_slot_r1   = slot_r1  ;
	old_tslot     = tslot    ;
	old_spk_slope = spk_slope;
	old_spk_dt    = spk_dt   ;
	old_spk_s2d   = spk_s2d  ;
	old_spk_cm    = spk_cm   ;
	old_spk_dum   = spk_dum  ;
	old_hp_vol1   = hp_vol1  ;
	old_hp_vol2   = hp_vol2  ;
	old_amp_en    = amp_en   ;
	old_amp_mute  = amp_mute ;
	old_amp_ctrl  = amp_ctrl ;
	old_amp_vmid  = amp_vmid ;
	old_line_ctrl = line_ctrl;

	return len;
}

static ssize_t s5m8751_reg_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	uint8_t da_pdb1   ;
	uint8_t da_amix1  ;
	uint8_t da_amix2  ;
	uint8_t da_ana    ;
	uint8_t da_dwa    ;
	uint8_t da_voll   ;
	uint8_t da_volr   ;
	uint8_t da_dig1   ;
	uint8_t da_dig2   ;
	uint8_t da_lim1   ;
	uint8_t da_lim2   ;
	uint8_t da_lof    ;
	uint8_t da_rof    ;
	uint8_t da_mux    ;
	uint8_t da_lgain  ;
	uint8_t da_rgain  ;
	uint8_t in1_ctrl1 ;
	uint8_t in1_ctrl2 ;
	uint8_t in1_ctrl3 ;
	uint8_t slot_l2   ;
	uint8_t slot_l1   ;
	uint8_t slot_r2   ;
	uint8_t slot_r1   ;
	uint8_t tslot     ;
	uint8_t spk_slope ;
	uint8_t spk_dt    ;
	uint8_t spk_s2d   ;
	uint8_t spk_cm    ;
	uint8_t spk_dum   ;
	uint8_t hp_vol1   ;
	uint8_t hp_vol2   ;
	uint8_t amp_en    ;
	uint8_t amp_mute  ;
	uint8_t amp_ctrl  ;
	uint8_t amp_vmid  ;
	uint8_t line_ctrl ;
	int val;
	int reg;
	int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret != 2) {
		printk(KERN_ERR "xxx %s: wrong arguments\n", __func__);
		return count;
	}

	printk(KERN_ERR "Updating 0x%02x:0x%02x\n", reg, val);
	s5m8751_reg_write(s5m8751, reg, val);

	da_pdb1   = s5m8751_reg_read(s5m8751, S5M8751_DA_PDB1   );
	da_amix1  = s5m8751_reg_read(s5m8751, S5M8751_DA_AMIX1  );
	da_amix2  = s5m8751_reg_read(s5m8751, S5M8751_DA_AMIX2  );
	da_ana    = s5m8751_reg_read(s5m8751, S5M8751_DA_ANA    );
	da_dwa    = s5m8751_reg_read(s5m8751, S5M8751_DA_DWA    );
	da_voll   = s5m8751_reg_read(s5m8751, S5M8751_DA_VOLL   );
	da_volr   = s5m8751_reg_read(s5m8751, S5M8751_DA_VOLR   );
	da_dig1   = s5m8751_reg_read(s5m8751, S5M8751_DA_DIG1   );
	da_dig2   = s5m8751_reg_read(s5m8751, S5M8751_DA_DIG2   );
	da_lim1   = s5m8751_reg_read(s5m8751, S5M8751_DA_LIM1   );
	da_lim2   = s5m8751_reg_read(s5m8751, S5M8751_DA_LIM2   );
	da_lof    = s5m8751_reg_read(s5m8751, S5M8751_DA_LOF    );
	da_rof    = s5m8751_reg_read(s5m8751, S5M8751_DA_ROF    );
	da_mux    = s5m8751_reg_read(s5m8751, S5M8751_DA_MUX    );
	da_lgain  = s5m8751_reg_read(s5m8751, S5M8751_DA_LGAIN  );
	da_rgain  = s5m8751_reg_read(s5m8751, S5M8751_DA_RGAIN  );
	in1_ctrl1 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL1 );
	in1_ctrl2 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL2 );
	in1_ctrl3 = s5m8751_reg_read(s5m8751, S5M8751_IN1_CTRL3 );
	slot_l2   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_L2   );
	slot_l1   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_L1   );
	slot_r2   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_R2   );
	slot_r1   = s5m8751_reg_read(s5m8751, S5M8751_SLOT_R1   );
	tslot     = s5m8751_reg_read(s5m8751, S5M8751_TSLOT     );
	spk_slope = s5m8751_reg_read(s5m8751, S5M8751_SPK_SLOPE );
	spk_dt    = s5m8751_reg_read(s5m8751, S5M8751_SPK_DT    );
	spk_s2d   = s5m8751_reg_read(s5m8751, S5M8751_SPK_S2D   );
	spk_cm    = s5m8751_reg_read(s5m8751, S5M8751_SPK_CM    );
	spk_dum   = s5m8751_reg_read(s5m8751, S5M8751_SPK_DUM   );
	hp_vol1   = s5m8751_reg_read(s5m8751, S5M8751_HP_VOL1   );
	hp_vol2   = s5m8751_reg_read(s5m8751, S5M8751_HP_VOL2   );
	amp_en    = s5m8751_reg_read(s5m8751, S5M8751_AMP_EN    );
	amp_mute  = s5m8751_reg_read(s5m8751, S5M8751_AMP_MUTE  );
	amp_ctrl  = s5m8751_reg_read(s5m8751, S5M8751_AMP_CTRL  );
	amp_vmid  = s5m8751_reg_read(s5m8751, S5M8751_AMP_VMID  );
	line_ctrl = s5m8751_reg_read(s5m8751, S5M8751_LINE_CTRL );

	printk(KERN_ERR "------ DAC control registers ---------------\n");
	printk(KERN_ERR "DA_PDB1   (0x17) 0x%02x\n", da_pdb1   );
	printk(KERN_ERR "DA_AMIX1  (0x18) 0x%02x\n", da_amix1  );
	printk(KERN_ERR "DA_AMIX2  (0x19) 0x%02x\n", da_amix2  );
	printk(KERN_ERR "DA_ANA    (0x1A) 0x%02x\n", da_ana    );
	printk(KERN_ERR "DA_DWA    (0x1B) 0x%02x\n", da_dwa    );
	printk(KERN_ERR "DA_VOLL   (0x1C) 0x%02x\n", da_voll   );
	printk(KERN_ERR "DA_VOLR   (0x1D) 0x%02x\n", da_volr   );
	printk(KERN_ERR "DA_DIG1   (0x1E) 0x%02x\n", da_dig1   );
	printk(KERN_ERR "DA_DIG2   (0x1F) 0x%02x\n", da_dig2   );
	printk(KERN_ERR "DA_LIM1   (0x20) 0x%02x\n", da_lim1   );
	printk(KERN_ERR "DA_LIM2   (0x21) 0x%02x\n", da_lim2   );
	printk(KERN_ERR "DA_LOF    (0x22) 0x%02x\n", da_lof    );
	printk(KERN_ERR "DA_ROF    (0x23) 0x%02x\n", da_rof    );
	printk(KERN_ERR "DA_MUX    (0x24) 0x%02x\n", da_mux    );
	printk(KERN_ERR "DA_LGAIN  (0x25) 0x%02x\n", da_lgain  );
	printk(KERN_ERR "DA_RGAIN  (0x26) 0x%02x\n", da_rgain  );
	printk(KERN_ERR "------ Interface control registers ---------\n");
	printk(KERN_ERR "IN1_CTRL1 (0x27) 0x%02x\n", in1_ctrl1 );
	printk(KERN_ERR "IN1_CTRL2 (0x28) 0x%02x\n", in1_ctrl2 );
	printk(KERN_ERR "IN1_CTRL3 (0x29) 0x%02x\n", in1_ctrl3 );
	printk(KERN_ERR "SLOT_L2   (0x2A) 0x%02x\n", slot_l2   );
	printk(KERN_ERR "SLOT_L1   (0x2B) 0x%02x\n", slot_l1   );
	printk(KERN_ERR "SLOT_R2   (0x2C) 0x%02x\n", slot_r2   );
	printk(KERN_ERR "SLOT_R1   (0x2D) 0x%02x\n", slot_r1   );
	printk(KERN_ERR "TSLOT     (0x2E) 0x%02x\n", tslot     );
	printk(KERN_ERR "------  Audio amp control registers --------\n");
	printk(KERN_ERR "SPK_SLOPE (0x30) 0x%02x\n", spk_slope );
	printk(KERN_ERR "SPK_DT    (0x31) 0x%02x\n", spk_dt    );
	printk(KERN_ERR "SPK_S2D   (0x32) 0x%02x\n", spk_s2d   );
	printk(KERN_ERR "SPK_CM    (0x33) 0x%02x\n", spk_cm    );
	printk(KERN_ERR "SPK_DUM   (0x34) 0x%02x\n", spk_dum   );
	printk(KERN_ERR "HP_VOL1   (0x35) 0x%02x\n", hp_vol1   );
	printk(KERN_ERR "HP_VOL2   (0x36) 0x%02x\n", hp_vol2   );
	printk(KERN_ERR "AMP_EN    (0x37) 0x%02x\n", amp_en    );
	printk(KERN_ERR "AMP_MUTE  (0x38) 0x%02x\n", amp_mute  );
	printk(KERN_ERR "AMP_CTRL  (0x39) 0x%02x\n", amp_ctrl  );
	printk(KERN_ERR "AMP_VMID  (0x3A) 0x%02x\n", amp_vmid  );
	printk(KERN_ERR "LINE_CTRL (0x3B) 0x%02x\n", line_ctrl );
	return count;
}

static DEVICE_ATTR(reg, 0644, s5m8751_reg_show, s5m8751_reg_store);
#endif /* CONFIG_SND_SOC_S5M8751_DEBUG */

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *s5m8751_socdev;

static int s5m8751_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = s5m8751_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	codec->private_data = pdev->dev.platform_data;

	s5m8751 = dev_to_s5m8751(&pdev->dev);
	if (!s5m8751) {
		dev_err(&pdev->dev, "failed to register audio\n");
		ret = -EINVAL;
		goto err;
	}

	ret = s5m8751_init(socdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialise S5M8751\n");
		goto err;
	}

#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
	ret = device_create_file(&pdev->dev, &dev_attr_reg);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create reg file\n");
		goto err;
	}
#endif
	return 0;
err:
	kfree(codec);
	return ret;
}

#ifdef CONFIG_SND_SOC_S5M8751_DEBUG
static int s5m8751_audio_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_reg);
	return 0;
}
#else
#define s5m8751_audio_remove NULL
#endif

static struct platform_driver s5m8751_audio_driver = {
	.driver		= {
		.name	= "s5m8751-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= s5m8751_audio_probe,
	.remove		= s5m8751_audio_remove,
};

static int s5m8751_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	pr_info("S5M8751 Audio Codec %s\n", S5M8751_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	s5m8751_socdev = socdev;

	ret = platform_driver_register(&s5m8751_audio_driver);
	if (ret != 0)
		printk(KERN_ERR "can't add audio driver");

	return ret;
}

/* power down chip */
static int s5m8751_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (s5m8751)
		s5m8751_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	platform_driver_unregister(&s5m8751_audio_driver);

	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_s5m8751 = {
	.probe = 	s5m8751_probe,
	.remove = 	s5m8751_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_s5m8751);

MODULE_DESCRIPTION("ASoC S5M8751 driver");
MODULE_AUTHOR("Jaswinder Singh <jassi.brar@samsung.com>");
