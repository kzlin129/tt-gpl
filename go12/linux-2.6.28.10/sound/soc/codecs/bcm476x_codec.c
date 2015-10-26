/*
 * bcm476x_codec.c -- BCM476X ALSA SoC audio driver for internal sound
 *
 * Copyright 2009 Broadcom Corp.
 * Author: Dzanh Nguyen <dzanh@broadcom.com>
 *
 * Copyright 2005 Openedhand Ltd.
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8753.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Triple DAI:-
 *
 * This driver support 3 DAI PCM's. This makes the default PCM available for
 * Stereo audio (e.g. MP3, ogg) playback/capture and the other PCM available for
 * voice and bluetooth.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include "asm/arch/hardware.h"
#include <asm/io.h>
#include "bcm476x_codec.h"
#include <linux/broadcom/halaudio.h>

//#include <linux/gpio.h>
//#include <linux/vgpio.h>
//#include <mach/pinmux.h>
//#include <plat/irvine.h>


#define BCM476X_SND_VERSION "1.00"

#define DAC_DAPM_ON     0       // turn off DAC DAPM for working around 1 DAC being disabled durin HF call audio routing.
#define ADC_DAPM_ON     1

//#define BCM476X_CODEC_DEBUG  

/* Virtual register addresses (0xb7e00-0xb7fff) */
#define REG_VIRTUAL_ADDR_BASE   (AUD1_REG_BASE_ADDR+0x0e00)
#define REG_VIRTUAL_LGAIN       (REG_VIRTUAL_ADDR_BASE+0x00)           /* gain for both audio & voice path */
#define REG_VIRTUAL_RGAIN       (REG_VIRTUAL_ADDR_BASE+0x04)           /* gain for both audio & voice path */
#define REG_VIRTUAL_MUTE        (REG_VIRTUAL_ADDR_BASE+0x08)           /* mute for both audio & voice path */
#define REG_VIRTUAL_ADDR_LAST   (REG_VIRTUAL_ADDR_BASE+0x0c)

#define vir_reg_index(reg)      (((reg)-REG_VIRTUAL_ADDR_BASE) >> 2)
static unsigned short aud_vir_regs[vir_reg_index(REG_VIRTUAL_ADDR_LAST)];

#define SPK_GAIN_MAX_DB  0
#define SPK_GAIN_MIN_DB  -79
#define SPK_GAIN_MAX    511
#define SPK_GAIN_MIN    (SPK_GAIN_MAX+(4*SPK_GAIN_MIN_DB))
#define MIC_GAIN_MAX_DB  42
#define MIC_GAIN_MIN_DB  0

#define SOC_DOUBLE_R_EXT_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert, \
                             xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, \
        .invert = xinvert} }

#define SoCPartVersion()           (readl(IO_ADDRESS(CMU_R_CMUPCELLID1_MEMADDR)) & CMU_F_CMUPCELLID1_MASK)

struct bcm476x_codec_priv {
    spinlock_t codec_lock;
};

struct bcm476x_dai_priv {
    spinlock_t dai_lock;
    struct semaphore stream_sync[2];
    unsigned short xfered_len[2];
};


/* used for muting alc5627 before the BCM4760 goes in power down */ 
int alc5627_i2c_mute(void);
#define BCM476X_AMP_NONE (0)
#define BCM476X_AMP_EUA2011 (1)
#define BCM476X_AMP_ALC5627 (2)
static int bcm476x_amp = BCM476X_AMP_NONE;

// Prototypes for codec driver's function interfaces
static void codec_init_if(void);
static void codec_exit_if(void);
static void codec_on_if(void);
static void codec_off_if(void);
static void codec_standby_if(void);
static int codec_analog_gain_get_if(struct snd_pcm_substream *substream, int *cur_db, int *min_db, int *max_db);
static int codec_analog_gain_set_if(struct snd_pcm_substream *substream, int db);
//static int codec_digital_gain_get_if(struct snd_pcm_substream *substream, int *cur_db, int *min_db, int *max_db);
//static int codec_digital_gain_set_if(struct snd_pcm_substream *substream, int db);
static int codec_write_buf_if(struct snd_pcm_substream *substream, short *buffer, int len);
static int codec_read_buf_if(struct snd_pcm_substream *substream, short *buffer, int len);
static int codec_micgain_get(struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol);
static int codec_micgain_set(struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol);
static int codec_virreg_get(struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol);
static int codec_spkrgain_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int codec_spkrgain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int codec_in_digital_gain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int codec_in_digital_gain_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int codec_mute_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static void speaker_mute(int set);

static HAL_AUDIO_FNCS ghalFuncs;  
static HAL_AUDIO_FNCS *codec_ops = NULL;
static bcm476x_codec_xferdone_cb *xferdone_cb = NULL;
void bcm476x_codec_register_stream_xfer_done_nofify(bcm476x_codec_xferdone_cb *cb)
{
    xferdone_cb = cb;
}


/*
 * read audio register
 */
static unsigned int read_aud_reg(unsigned int reg)
{
    u16 value;
    if (reg >= REG_VIRTUAL_ADDR_BASE && reg < REG_VIRTUAL_ADDR_LAST) 
        value = aud_vir_regs[vir_reg_index(reg)]; 
    else
        value = readl((IO_ADDRESS(reg)));
    return value;
}
/*
 * write audio register
 */
static int write_aud_reg(unsigned int reg, unsigned int value)
{
    int rc = 0;
    if (reg >= REG_VIRTUAL_ADDR_BASE && reg < REG_VIRTUAL_ADDR_LAST) 
        aud_vir_regs[vir_reg_index(reg)] = value; 
    else {
        rc = writel(value, (IO_ADDRESS(reg)));
        //rc = 0;
    }
    return rc;
}

/*
 * read sound-device register
 */
static unsigned int bcm476x_codec_read_reg(struct snd_soc_codec *codec,
	unsigned int reg)
{
    unsigned int value = read_aud_reg(reg);
#ifdef BCM476X_CODEC_DEBUG
    printk("bcm476x_codec_read_reg: reg 0x%x = 0x%x\n", reg, value);
#endif
    return value;
}

/*
 * write sound-device register
 */
static int bcm476x_codec_write_reg(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
   //int rc = 0;
   int rc = write_aud_reg(reg, value);
#ifdef BCM476X_CODEC_DEBUG
    printk("bcm476x_codec_write_reg: reg 0x%x = 0x%x\n", reg, value);
#endif
    return rc;
}

/*
 * BCM476X Controls
 */
static const char *dsp_dac_sources[] = {"DAC1", "DAC2"};
static const struct soc_enum mixer_path_enum = SOC_ENUM_SINGLE(REG_MIXER_INPUT_SEL_ADDR, 14, 2, dsp_dac_sources);

//static const char *dsp_in_sources[] = {"ADC1", "ADC2"};
//static const struct soc_enum voice_in_path_enum = SOC_ENUM_SINGLE(REG_VICTRL_ADDR, 1, 2, dsp_in_sources);
//static const struct soc_enum audio_in_path_enum = SOC_ENUM_SINGLE(REG_AICTRL_ADDR, 2, 2, dsp_in_sources);

// #define DECLARE_TLV_DB_SCALE(name, min, step, mute) 
//static const DECLARE_TLV_DB_SCALE(slopgain_tlv, (SPK_GAIN_MIN_DB*100), 25, 0);      // min -78.0dB, step .25dB

static const unsigned int slopgain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	/* 0000000 - 0xc7 = "Analogue mute" */
	0, (SPK_GAIN_MIN-1), TLV_DB_SCALE_ITEM(-12750, 0, 0),
	SPK_GAIN_MIN, SPK_GAIN_MAX, TLV_DB_SCALE_ITEM((SPK_GAIN_MIN_DB*100), 25, 0), 
};


static const DECLARE_TLV_DB_SCALE(virgain_tlv, (SPK_GAIN_MIN_DB*100), 25, 0);      // min -79.0dB, step .25dB
static const DECLARE_TLV_DB_SCALE(vcfg_tlv, 0, 25, 0);      // min 0dB, step .25dB
static const DECLARE_TLV_DB_SCALE(eqgain_tlv, 0, 25, 0);    // min 0dB, step .25dB
static const DECLARE_TLV_DB_SCALE(in_iir_tlv, 0, 300, 0);  // min 0dB, step 3dB
static const DECLARE_TLV_DB_SCALE(in_cic_tlv, 0, 300, 0);  // min 0dB, step 3dB
static const DECLARE_TLV_DB_SCALE(cic_finescale_tlv, 0, 1, 0);  // min 0dB, step .01dB

static const struct snd_kcontrol_new bcm476x_snd_controls[] = {
//SOC_SINGLE("I2S Mode", REG_AUDMOD_ADDR, 3, 1, 0),
SOC_SINGLE_EXT("Speaker Mute", REG_VIRTUAL_MUTE, 1, 1, 0, codec_virreg_get, codec_mute_set),

// DSP_AUDIO_VSLOPGAIN
//SOC_SINGLE("VoiceSlopeGainEnable", REG_VSLOPGAIN_ADDR, 15, 1, 0),
//SOC_SINGLE("VoiceSlopeMod", REG_VSLOPGAIN_ADDR, 11, 0x0f, 0),
//SOC_SINGLE_TLV("Voice Playback Volume", REG_VSLOPGAIN_ADDR, 0, 511, 0, slopgain_tlv),
// DSP_AUDIO_ASLOPGAIN
//SOC_DOUBLE_R("AudioSlopeGainEnable", REG_ALSLOPGAIN_ADDR, REG_ARSLOPGAIN_ADDR, 15, 1, 0),
//SOC_DOUBLE_R("AudioSlopeMod", REG_ALSLOPGAIN_ADDR, REG_ARSLOPGAIN_ADDR, 11, 0x0f, 0),
//SOC_DOUBLE_R_TLV("Audio Playback Volume", REG_ALSLOPGAIN_ADDR, REG_ARSLOPGAIN_ADDR, 0, 511, 0, slopgain_tlv),
SOC_DOUBLE_R_EXT_TLV("Speaker Playback Volume", REG_VIRTUAL_LGAIN, REG_VIRTUAL_RGAIN, 0, -(SPK_GAIN_MIN_DB*4), 0, codec_spkrgain_get, codec_spkrgain_set, virgain_tlv),

// DSP_AUDIO_MIXER_GAIN_ADJUST
SOC_SINGLE("Mixer Playback Volume", REG_MIXER_GAIN_ADJUST_ADDR, 0, 0x2000, 0),

// MIC volume control
SOC_SINGLE_EXT("MIC1 Capture Volume", CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR, 0, MIC_GAIN_MAX_DB, 0, codec_micgain_get, codec_micgain_set),
SOC_SINGLE_EXT("MIC2 Capture Volume", CMU_R_ADM_RXAUDIO_CTRL4_MEMADDR, 0, MIC_GAIN_MAX_DB, 0, codec_micgain_get, codec_micgain_set),

// DSP_AUDIO_VCFGR (Voice compensation filter gain)
SOC_SINGLE_TLV("VoiceFilterGain Playback Volume", REG_VCFGR_ADDR, 0, 24, 0, vcfg_tlv),

// DSP_AUDIO_AEQPATHGAIN1&2 (Audio equalizer gain)
SOC_SINGLE("EQSlopeGainEnable", REG_AEQPATHGAIN1_ADDR, 15, 1, 1),  // active low
SOC_SINGLE("EQSlopeMod", REG_AEQPATHGAIN1_ADDR, 11, 0x0f, 0),
SOC_SINGLE_TLV("EQGain1 Playback Volume", REG_AEQPATHGAIN1_ADDR, 0, 511, 0, eqgain_tlv),
SOC_SINGLE_TLV("EQGain2 Playback Volume", REG_AEQPATHGAIN2_ADDR, 0, 511, 0, eqgain_tlv),
SOC_SINGLE_TLV("EQGain3 Playback Volume", REG_AEQPATHGAIN3_ADDR, 0, 511, 0, eqgain_tlv),
SOC_SINGLE_TLV("EQGain4 Playback Volume", REG_AEQPATHGAIN4_ADDR, 0, 511, 0, eqgain_tlv),
SOC_SINGLE_TLV("EQGain5 Playback Volume", REG_AEQPATHGAIN5_ADDR, 0, 511, 0, eqgain_tlv),

// Digital Input Gain Control
SOC_SINGLE_EXT_TLV("Stereo In IIR Capture Volume", REG_AICTRL_ADDR, 13, 7, 0, codec_in_digital_gain_get, codec_in_digital_gain_set, in_iir_tlv), // up +3dB, down -3dB
SOC_SINGLE_EXT_TLV("Stereo In CIC Capture Volume", REG_AICTRL_ADDR, 11, 3, 0, codec_in_digital_gain_get, codec_in_digital_gain_set, in_cic_tlv), // up +3dB, down -3dB
SOC_SINGLE_EXT_TLV("Stereo In CIC FineScale Capture Volume", REG_AICTRL_ADDR, 3, 255, 0, codec_in_digital_gain_get, codec_in_digital_gain_set, cic_finescale_tlv), // up +.012dB, down -.012dB
SOC_SINGLE_EXT_TLV("Voice In IIR Capture Volume", REG_VICTRL_ADDR, 13, 7, 1, codec_in_digital_gain_get, codec_in_digital_gain_set, in_iir_tlv), // (inverted) up -3dB, down +3dB
SOC_SINGLE_EXT_TLV("Voice In CIC Capture Volume", REG_VICTRL_ADDR, 11, 3, 0, codec_in_digital_gain_get, codec_in_digital_gain_set, in_cic_tlv), // up +3dB, down -3dB
SOC_SINGLE_EXT_TLV("Voice In CIC FineScale Capture Volume", REG_VICTRL_ADDR, 3, 255, 0, codec_in_digital_gain_get, codec_in_digital_gain_set, cic_finescale_tlv), // up +.012dB, down -.012dB
};

/* add non dapm controls */
static int bcm476x_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(bcm476x_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&bcm476x_snd_controls[i],
						codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/*
 * DAPM Controls
 */


#if 0 // Removed, leave here for reference only. The DAPM for these components are controlled in the dapm widgets
/* DAC controls inverted (active low). */
static const struct snd_kcontrol_new bcm476x_cmu_dac_controls[] = {
SOC_DAPM_SINGLE("DAC D2C Converter",CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 0, 1, 1),
SOC_DAPM_SINGLE("Left DAC Pwr",     CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 1, 1, 1),
SOC_DAPM_SINGLE("Right DAC Pwr",    CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 2, 1, 1),
SOC_DAPM_SINGLE("Left DAC Driver",  CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 3, 1, 1),
SOC_DAPM_SINGLE("Right DAC Driver", CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 4, 1, 1),
SOC_DAPM_SINGLE("DAC nref",         CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 5, 1, 1),
};

/* MIC power controls */
static const struct snd_kcontrol_new bcm476x_pml_adc_pwr_controls[] = {
SOC_DAPM_SINGLE("Right ADC Standby", PML_R_PML_SW_PWRDOWN_MEMADDR, 6, 0, 0),
SOC_DAPM_SINGLE("Left ADC Standby",  PML_R_PML_SW_PWRDOWN_MEMADDR, 5, 0, 0),
SOC_DAPM_SINGLE("Right ADC PwrUp", PML_R_PML_SW_PWRDOWN_MEMADDR, 4, 0, 0),
SOC_DAPM_SINGLE("Left ADC PwrUp",  PML_R_PML_SW_PWRDOWN_MEMADDR, 3, 0, 0),
SOC_DAPM_SINGLE("Right MIC PwrUp",  PML_R_PML_SW_PWRDOWN_MEMADDR, 2, 1, 1), // inverted (active low)
SOC_DAPM_SINGLE("Left MIC PwrUp", PML_R_PML_SW_PWRDOWN_MEMADDR, 1, 1, 1),   // inverted (active low)
};

/* Mixer Control */
static const struct snd_kcontrol_new bcm476x_mixer_input_select[] = {
SOC_DAPM_ENUM("DAC Selection", mixer_path_enum),
SOC_DAPM_SINGLE("Voice Left", REG_MIXER_INPUT_SEL_ADDR, 4, 1, 0),
SOC_DAPM_SINGLE("Audio Right", REG_MIXER_INPUT_SEL_ADDR, 1, 1, 0),
SOC_DAPM_SINGLE("Audio Left", REG_MIXER_INPUT_SEL_ADDR, 0, 1, 0),
};
#endif

/* stream domain */
//SND_SOC_DAPM_DAC(wname, stname, wreg, wshift, winvert)
// SND_SOC_DAPM_ADC(wname, stname, wreg, wshift, winvert)
/* path domanin */
//#define SND_SOC_DAPM_PGA(wname, wreg, wshift, winvert, wcontrols, wncontrols)

static struct snd_soc_dapm_widget bcm476x_dac_dapm_widgets[] = {
 #if DAC_DAPM_ON // Turn on/off DAC power management (set to off for Tomtom, something in handfree app causes the Left/Voice DAC to turn off)
    SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 1, 0),   
    SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 2, 0),
    SND_SOC_DAPM_PGA("Voice Speaker",  CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 3, 0, NULL, 0),  // select audio left source
    SND_SOC_DAPM_PGA("Left Speaker",  CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 3, 0, NULL, 0),  // select audio left source
    SND_SOC_DAPM_PGA("Right Speaker", CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 4, 0, NULL, 0), // select audio right source
    SND_SOC_DAPM_DAC("Voice DAC", "Voice Playback", CMU_R_DAC_AUDIO_CTRL0_MEMADDR, 1, 0), // DAC DAPM for voice stream
#else
    SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", SND_SOC_NOPM, 0, 0),   
    SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", SND_SOC_NOPM, 0, 0),
    SND_SOC_DAPM_PGA("Voice Speaker",  SND_SOC_NOPM, 0, 0, NULL, 0),  // select audio left source
    SND_SOC_DAPM_PGA("Left Speaker",  SND_SOC_NOPM, 0, 0, NULL, 0),  // select audio left source
    SND_SOC_DAPM_PGA("Right Speaker", SND_SOC_NOPM, 0, 0, NULL, 0), // select audio right source
    SND_SOC_DAPM_DAC("Voice DAC", "Voice Playback", SND_SOC_NOPM, 0, 0), // DAC DAPM for voice stream
#endif
};

static const struct snd_soc_dapm_widget bcm476x_dapm_widgets[] =  {
#if 0 // Controls conflict with HAL driver, let HAL driver control
    SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0, &bcm476x_mixer_input_select[0], ARRAY_SIZE(bcm476x_mixer_input_select)),
    SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0, &bcm476x_mixer_input_select[0], ARRAY_SIZE(bcm476x_mixer_input_select)),
    SND_SOC_DAPM_MIXER("Voice Mixer", SND_SOC_NOPM, 0, 0, NULL, 0), // Virtual mixer
#else
    SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("Voice Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
#endif
	SND_SOC_DAPM_OUTPUT("VOUT"),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("PCMOUT"),
	SND_SOC_DAPM_OUTPUT("VREF"),

#if ADC_DAPM_ON // turn on/off the ADC power management
    SND_SOC_DAPM_ADC("Left ADC", "Left Hifi Capture", PML_R_PML_SW_PWRDOWN_MEMADDR, 3, 0),
    SND_SOC_DAPM_ADC("Right ADC", "Right Hifi Capture", PML_R_PML_SW_PWRDOWN_MEMADDR, 4, 0),
    SND_SOC_DAPM_ADC("Left Voice ADC", "Left Voice Capture", PML_R_PML_SW_PWRDOWN_MEMADDR, 3, 0),
    SND_SOC_DAPM_ADC("Right Voice ADC", "Right Voice Capture", PML_R_PML_SW_PWRDOWN_MEMADDR, 4, 0),
#else
    SND_SOC_DAPM_ADC("Left ADC", "Left Hifi Capture", SND_SOC_NOPM, 0, 0),
    SND_SOC_DAPM_ADC("Right ADC", "Right Hifi Capture", SND_SOC_NOPM, 0, 0),
    SND_SOC_DAPM_ADC("Left Voice ADC", "Left Voice Capture", SND_SOC_NOPM, 0, 0),
    SND_SOC_DAPM_ADC("Right Voice ADC", "Right Voice Capture", SND_SOC_NOPM, 0, 0),
#endif

    // Don't touch MIC power, causes glitches at the beginning of the record
    SND_SOC_DAPM_PGA("Left MIC", SND_SOC_NOPM, 0, 0, NULL, 0), // select MIC1 pwr up
    SND_SOC_DAPM_PGA("Right MIC", SND_SOC_NOPM, 0, 0, NULL, 0), // select MIC2 pwr up
    SND_SOC_DAPM_PGA("Left Voice MIC", SND_SOC_NOPM, 0, 0, NULL, 0), // select MIC1 pwr up
    SND_SOC_DAPM_PGA("Right Voice MIC", SND_SOC_NOPM, 0, 0, NULL, 0), // select MIC2 pwr up
    SND_SOC_DAPM_MIXER("Capture Stereo Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("Capture Voice Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

    SND_SOC_DAPM_ADC("PCM In", "PCM Capture", SND_SOC_NOPM, 0, 0),
    SND_SOC_DAPM_ADC("PCM Out", "PCM Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("VLINPUT"),
	SND_SOC_DAPM_INPUT("VRINPUT"),
	SND_SOC_DAPM_INPUT("ALINPUT"),
	SND_SOC_DAPM_INPUT("ARINPUT"),
	SND_SOC_DAPM_INPUT("PCMINPUT"),
};

/*
 * Codec routing: sink-dapm, snd-control, source-dapm
 */
static const struct snd_soc_dapm_route audio_map[] = {
#if 0 // Controls conflict with HAL driver, let HAL driver control
	/* Left mixer */
	{"Left Mixer", "Audio Left", "Left DAC"},
    /* Right mixer */
    {"Right Mixer", "Audio Right", "Right DAC"},
    /* Voice mixer */
	{"Voice Mixer", "Voice Left", "Left DAC"},
	{"Voice Mixer", "Voice Left", "Right DAC"},
#else
	/* Left mixer */
	{"Left Mixer", NULL, "Left DAC"},
	/* Right mixer */
    {"Right Mixer", NULL, "Right DAC"},
    /* Voice mixer */
	{"Voice Mixer", NULL, "Voice DAC"},
	{"Voice Mixer", NULL, "Right DAC"},
#endif
    /* left out 1 */
    {"Left Speaker", NULL, "Left DAC"},
 	{"LOUT", NULL, "Left Speaker"},

	/* right out 1 */
    {"Right Speaker", NULL, "Right DAC"},
	{"ROUT", NULL, "Right Speaker"},

	/* mono out */
    {"Voice Speaker", NULL, "Voice DAC"},
    {"Voice Speaker", NULL, "Right DAC"},
	{"VOUT", NULL, "Voice Speaker"},

    /* Capture mixer */
    {"Capture Stereo Mixer", NULL, "Left ADC"},
    {"Capture Stereo Mixer", NULL, "Right ADC"},
    {"Capture Voice Mixer", NULL, "Left Voice ADC"},
    {"Capture Voice Mixer", NULL, "Right Voice ADC"},

    /* input pga */
 	{"Left ADC", NULL, "Left MIC"},
    {"Right ADC", NULL, "Right MIC"},
 	{"Left Voice ADC", NULL, "Left Voice MIC"},
 	{"Right Voice ADC", NULL, "Right Voice MIC"},

    /* input pga */
 	{"Left MIC", NULL, "ALINPUT"},
    {"Right MIC", NULL, "ARINPUT"},
    {"Left Voice MIC", NULL, "VLINPUT"},
    {"Right Voice MIC", NULL, "VRINPUT"},

  	/* PCM Mux */
    {"PCMOUT", NULL, "PCM Out"},
	{"PCM In", NULL, "PCMINPUT"},
};

static int bcm476x_add_widgets(struct snd_soc_codec *codec)
{
    int n;

    if (SoCPartVersion() <= 0xB1) // Part A0,B0 and B1 are active low, need to set inverted bit.
    {
        printk(KERN_INFO "A0/B0 part detected, invert the DAPM DAC bits\n");
        for (n = 0; n < ARRAY_SIZE(bcm476x_dac_dapm_widgets); n++)
        {
            bcm476x_dac_dapm_widgets[n].invert = 1;
        }
    }
	snd_soc_dapm_new_controls(codec, bcm476x_dac_dapm_widgets,
				  ARRAY_SIZE(bcm476x_dac_dapm_widgets));

	snd_soc_dapm_new_controls(codec, bcm476x_dapm_widgets,
				  ARRAY_SIZE(bcm476x_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

#if 0 // removed, not used
static int bcm476x_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;
    u16 reg;
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= (1 << 3); // select I2S mode
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		return -EINVAL;
	}

    reg = codec->read(codec, REG_AUDMOD_ADDR);
    reg |= iface;
	codec->write(codec, REG_AUDMOD_ADDR, iface);
	return 0;
}
#endif

static int bcm476x_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	/*printk(KERN_INFO "bcm476x_set_bias_level: level=%d\n", level);*/

	switch (level) {
	case SND_SOC_BIAS_ON:
        codec_on_if();
		break;

	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
        codec_standby_if();
		break;

	case SND_SOC_BIAS_OFF:
        codec_off_if();
		break;
	}

	codec->bias_level = level;
	return 0;
}

/* Set audio out path frequency */
static int SetAudioFrequency(struct snd_soc_codec *codec, int freq)
{
#if 0 // comment out, use the codec ops api
     int asamp_rate;
     u16 reg;

     switch (freq)
     {
     case 12000:
         asamp_rate = 1;
         break;
     case 16000:
         asamp_rate = 2;
         break;
     case 24000:
         asamp_rate = 3;
         break;
     case 32000:
         asamp_rate = 4;
         break;
     case 48000:
         asamp_rate = 5;
         break;
     case 11025:
         asamp_rate = 6;
         break;
     case 22050:
         asamp_rate = 7;
         break;
     case 44100:
         asamp_rate = 8;
         break;
     default:
         return -EINVAL;
         break;
     }
     reg = bcm476x_codec_read_reg(codec, REG_AUDMOD_ADDR);
     reg &= ~(REG_AUDMOD_ASAMPRATE_MASK | REG_AUDMOD_AMUTE_MASK);
     reg |= (asamp_rate << REG_AUDMOD_ASAMPRATE_BIT_SHIFT);
     bcm476x_codec_write_reg(codec, REG_AUDMOD_ADDR, reg);
     return 0;
#else
    if(codec_ops->selectMode) {
        return codec_ops->selectMode(HAL_AUDIO_MODE_ALSA,freq);
    }
    return -EINVAL;
#endif

}

static int SetVoiceFrequency(struct snd_soc_codec *codec, int freq)
{
#if 0 // comment out, use the codec ops api
    u16 reg = bcm476x_codec_read_reg(codec, REG_AUDMOD_ADDR);
    if (freq == 16000)
        reg |= REG_AMCR_MODE_16K_MASK;
    else if (freq == 8000)
        reg &= ~REG_AMCR_MODE_16K_MASK;
    else
    	return -EINVAL;
    bcm476x_codec_write_reg(codec, REG_AMCR_ADDR, reg); 
    return 0;
#else
    if(codec_ops->selectMode) {
        return codec_ops->selectMode(HAL_AUDIO_MODE_ALSA,freq);
    }
    return -EINVAL;
#endif
}

/*
 * Get gain control for MICs 
 * Offset in CMU to change gain of MIC1:
 * 0xB0204[31:30] =  i_pga_gain[[1:0] 
 * 0xB0208[3:0]   = i_pga_gain[5:2]
 *
 * Offset in CMU to change gain of MIC2:
 * 0xB0220[31:30] =  i_pga_gain[[1:0] 
 * 0xB0224[3:0]   = i_pga_gain[5:2]
 */
static int codec_micgain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
    unsigned long gain_addr_1, gain_addr_2;
    unsigned long gain_ctrl_1, gain_ctrl_2;
    int mic_gain;
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg_addr = mixer_ctl->reg;

    if (CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR == reg_addr) {  /* MIC1 */
        gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR); // 0xB0204
        gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL2_MEMADDR); // 0xB0208
    }
    else { /* MIC2 */
        gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL4_MEMADDR); // 0xB0220
        gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL5_MEMADDR); // 0xB0224
    }
    gain_ctrl_1 = readl(gain_addr_1);
    gain_ctrl_2 = readl(gain_addr_2);
    mic_gain = ((gain_ctrl_1 >> 30) & 3) | ((gain_ctrl_2 & 0x0f) << 2);
	ucontrol->value.integer.value[0] = mic_gain;
    return 0;
}

/*
 * Set gain control for MICs 
 * Offset in CMU to change gain of MIC1:
 * 0xB0204[31:30] =  i_pga_gain[[1:0] 
 * 0xB0208[3:0]   = i_pga_gain[5:2]
 *
 * Offset in CMU to change gain of MIC2:
 * 0xB0220[31:30] =  i_pga_gain[[1:0] 
 * 0xB0224[3:0]   = i_pga_gain[5:2]
 */
static int codec_micgain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
    unsigned long gain_addr_1, gain_addr_2;
    unsigned long gain_ctrl_1, gain_ctrl_2;
    int mic_gain, cur_gain;
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg_addr = mixer_ctl->reg;

	mic_gain = ucontrol->value.integer.value[0];
    if (CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR == reg_addr) {  /* MIC1 */
        gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR); // 0xB0204
        gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL2_MEMADDR); // 0xB0208
    }
    else { /* MIC2 */
        gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL4_MEMADDR); // 0xB0220
        gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL5_MEMADDR); // 0xB0224
    }
    gain_ctrl_1 = readl(gain_addr_1);
    gain_ctrl_2 = readl(gain_addr_2);
    cur_gain = ((gain_ctrl_1 >> 30) & 3) | ((gain_ctrl_2 & 0x0f) << 2);
    if (cur_gain != mic_gain) {
        gain_ctrl_1 &= 0x3fffffff;
        gain_ctrl_1 |= ((mic_gain & 3) << 30);
        gain_ctrl_2 &= 0xfffffff0;
        gain_ctrl_2 |= ((mic_gain & 0x3c) >> 2);
        writel(gain_ctrl_1, gain_addr_1); 
        writel(gain_ctrl_2,gain_addr_2); 
    }
    return 0;
}

static int codec_virreg_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg = mixer_ctl->reg;
    ucontrol->value.integer.value[0] = aud_vir_regs[vir_reg_index(reg)];
    return 0;
}

static int codec_spkrgain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int lreg = mixer_ctl->reg;
    int rreg = mixer_ctl->rreg;
    u16 lgain, rgain;
    /* Set audio gain */
    lgain = read_aud_reg(lreg);
    lgain &= 0x1ff;
    if (lgain) lgain = lgain-SPK_GAIN_MIN;
    rgain = read_aud_reg(rreg);
    rgain &= 0x1ff;
    if (rgain) rgain = rgain-SPK_GAIN_MIN;
	ucontrol->value.integer.value[0] = lgain;
	ucontrol->value.integer.value[1] = rgain;
    return 0;
}

static void set_speaker_gain(int lval, int rval)
{
    u16 gain;
    /* Set audio gain */
    gain = read_aud_reg(REG_ALSLOPGAIN_ADDR);
    gain &= 0xfe00;
    gain |= lval;
    write_aud_reg(REG_ALSLOPGAIN_ADDR, gain);
    gain = read_aud_reg(REG_ARSLOPGAIN_ADDR);
    gain &= 0xfe00;
    gain |= rval;
    write_aud_reg(REG_ARSLOPGAIN_ADDR, gain);
    /* Set voice gain */
    gain = read_aud_reg(REG_VSLOPGAIN_ADDR);
    gain &= 0xfe00;
    gain |= lval;
    write_aud_reg(REG_VSLOPGAIN_ADDR, gain);
}

static int codec_spkrgain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int lreg = mixer_ctl->reg;
    int rreg = mixer_ctl->rreg;
    u16 lset = ucontrol->value.integer.value[0];
    u16 rset= ucontrol->value.integer.value[1];

    if (lset && rset==0) rset = lset;
    if (lset) lset = SPK_GAIN_MIN + lset; // convert to gain value by adding min gain value
    if (rset) rset = SPK_GAIN_MIN + rset; // convert to gain value by adding min gain value
    /* update virtual register */
	write_aud_reg(lreg ,lset); 
	write_aud_reg(rreg ,rset);
    /* update hardware gain */
    set_speaker_gain(lset, rset);
    return 0;
}

/* mute/unmute the speakers */
static void speaker_mute(int set)
{
#if 0
	int reg = REG_VIRTUAL_MUTE; 

	if (set != read_aud_reg(reg)) {
		if (set) { // mute on
			set_speaker_gain(0, 0); // set to digital mute
		} else { // mue off, restore gain
			int lgain = read_aud_reg(REG_VIRTUAL_LGAIN);
			int rgain = read_aud_reg(REG_VIRTUAL_RGAIN);
			set_speaker_gain(lgain, rgain); // restore gain.
		}
		/* update virtual mute register */
		write_aud_reg(reg, set); 
	}
#endif
}

static int codec_mute_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg = mixer_ctl->reg;
    int set = ucontrol->value.integer.value[0];

    if (set != read_aud_reg(reg)) {
        speaker_mute(set);
    }
    return 0;
}

static int codec_in_digital_gain_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg = mixer_ctl->reg;
    //u16 set = ucontrol->value.integer.value[0];
    u16  vadc_reg = read_aud_reg(REG_VADCCTRL_ADDR);

    if (REG_AICTRL_ADDR == reg) { // audio input control
        vadc_reg |= 0x8000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select audio-in right path
        snd_soc_put_volsw(kcontrol, ucontrol);
        vadc_reg &= ~0x8000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select audio-in left path
        snd_soc_put_volsw(kcontrol, ucontrol);
    } else if (REG_VICTRL_ADDR == reg) { // voice input control
        vadc_reg |= 0x4000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select voice-in right path
        snd_soc_put_volsw(kcontrol, ucontrol);
        vadc_reg &= ~0x4000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select voice-in left path
        snd_soc_put_volsw(kcontrol, ucontrol);
    }
    return 0;
}

static int codec_in_digital_gain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mixer_ctl = (struct soc_mixer_control *) kcontrol->private_value;
    int reg = mixer_ctl->reg;
    u16  vadc_reg = read_aud_reg(REG_VADCCTRL_ADDR);

    if (REG_AICTRL_ADDR == reg) { // audio input control
        vadc_reg &= ~0x8000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select audio-in left path
    }
    else if (REG_VICTRL_ADDR == reg) {
        vadc_reg &= ~0x4000;  
        write_aud_reg(REG_VADCCTRL_ADDR, vadc_reg); // select voice-in left path
    }
    return snd_soc_get_volsw(kcontrol, ucontrol);
}

static int bcm476x_codec_stream_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
    struct bcm476x_dai_priv *dai_rtd = dai->private_data;

	spin_lock_init(&dai_rtd->dai_lock);
	sema_init(&dai_rtd->stream_sync[0],0);
	sema_init(&dai_rtd->stream_sync[1],0);
    bcm476x_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
    return 0;
}

static int bcm476x_hifi_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    int currentDB, minDB, maxDB;
    int rc;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
    struct snd_pcm_runtime *runtime = substream->runtime;

    /* printk(KERN_INFO "bcm476x_hifi_prepare: device=%d, stream=%d, rate=%d\n", substream->pcm->device, substream->stream, runtime->rate); */
    SetAudioFrequency(codec, runtime->rate);

    rc = codec_analog_gain_get_if(substream, &currentDB, &minDB, &maxDB);
    if (!rc) {
        if (HAL_AUDIO_GAIN_SLEEP == currentDB) {
            currentDB = HAL_AUDIO_GAIN_MUTE;    // set this to wake block up
        }
        rc  = codec_analog_gain_set_if(substream, currentDB);
    }
    return rc;
}

static void bcm476x_hifi_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	/* printk(KERN_INFO "bcm476x_hifi_shutdown\n"); */

	/* Gracefully shut down the stereo interface. */
	codec_analog_gain_set_if(substream, HAL_AUDIO_GAIN_SLEEP);

	/* power down external amplifier to avoid loud pops from BCM power down */
	switch (bcm476x_amp) {
		case BCM476X_AMP_ALC5627:
			alc5627_i2c_mute();
			break;
		case BCM476X_AMP_EUA2011:
			//gpio_direction_output( TT_VGPIO_AMP_PWR_EN, 0 );
			break;
	}

	/* Power down the DAC */
	bcm476x_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int bcm476x_hifi_mute(struct snd_soc_dai *dai, int mute)
{
    /*printk(KERN_INFO "bcm476x_hifi_mute: mute=%d\n", mute);*/
#if 0 // The mute bit in REG_AUDMOD_ADDR does not work, use the digital gain mute
    {
	struct snd_soc_codec *codec = dai->codec;
	u16 reg = bcm476x_codec_read_reg(codec, REG_AUDMOD_ADDR) & (~0x2);

	if (mute)
		bcm476x_codec_write_reg(codec, REG_AUDMOD_ADDR, reg | 0x2);
	else
		bcm476x_codec_write_reg(codec, REG_AUDMOD_ADDR, reg);
    }
#else
    speaker_mute(mute);
#endif
	return 0;
}

static int bcm476x_voice_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    int currentDB, minDB, maxDB;
    int rc;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
    struct snd_pcm_runtime *runtime = substream->runtime;

    /* printk(KERN_INFO "bcm476x_voice_prepare: device=%d, stream=%d\n", substream->pcm->device, substream->stream); */
    SetVoiceFrequency(codec, runtime->rate);

    rc = codec_analog_gain_get_if(substream, &currentDB, &minDB, &maxDB);
    if (!rc) {
        if (HAL_AUDIO_GAIN_SLEEP == currentDB) {
            currentDB = HAL_AUDIO_GAIN_MUTE;    // set this to wake block up
        }
        rc  = codec_analog_gain_set_if(substream, currentDB);
    }
    return rc;
}

static void bcm476x_voice_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    bcm476x_hifi_shutdown(substream, dai); // same as hifi shutdown
}

static int bcm476x_voice_mute(struct snd_soc_dai *dai, int mute)
{
    /* printk(KERN_INFO "bcm476x_voice_mute: mute=%d\n", mute); */

#if 0 // The mute bit in REG_VMUT_ADDR does not work, use the digital gain mute
    {
	struct snd_soc_codec *codec = dai->codec;
	u16 reg = bcm476x_codec_read_reg(codec, REG_VMUT_ADDR) & (~0x2);

	if (mute)
		bcm476x_codec_write_reg(codec, REG_VMUT_ADDR, reg | 0x2);
	else
		bcm476x_codec_write_reg(codec, REG_VMUT_ADDR, reg);
    }
#else
    speaker_mute(mute);
#endif
	return 0;
}

// bluetooth interface
static int bcm476x_pcm_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
    struct snd_pcm_runtime *runtime = substream->runtime;
    int rc;

    /* printk(KERN_INFO "bcm476x_pcm_prepare: device=%d, stream=%d\n", substream->pcm->device, substream->stream); */
    SetVoiceFrequency(codec, runtime->rate);
    rc  = codec_analog_gain_set_if(substream, 0); // set gain to 0 for enable the PCM path
    return rc;
}

// bluetooth interface
static void bcm476x_pcm_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    bcm476x_hifi_shutdown(substream, dai); // same as hifi shutdown
}

/*
 * read buffer from hw
 */
static int bcm476x_codec_read_buf(void *param, char *buf, int len)
{
    struct snd_pcm_substream  *substream = (struct snd_pcm_substream  *) param;
    return codec_read_buf_if(substream, (short *) buf, len);
}

/*
 * write buffer to hw
 */
static int bcm476x_codec_write_buf(void *param, const char * buf, int len)
{
    struct snd_pcm_substream  *substream =  (struct snd_pcm_substream  *) param;
    return codec_write_buf_if(substream, (short *) buf, len);
}

static struct bcm476x_dai_priv dai_priv[BCM476X_NUM_DAIS];

struct snd_soc_dai bcm476x_dai[3] = {
	{
	    .name = "Music",
	    .playback = {
		    .stream_name = "HiFi Playback",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | 
                      SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 ),
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
            },
	    .capture = {
		    .stream_name = "HiFi Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000),
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
            },
	    .ops = {
       		    .startup = bcm476x_codec_stream_startup,
		    .prepare = bcm476x_hifi_prepare,
		    .shutdown = bcm476x_hifi_shutdown,
		    //.set_fmt = bcm476x_set_dai_fmt,
	            .digital_mute = bcm476x_hifi_mute,
            },
            .private_data = &dai_priv[BCM476X_STEREO_DAI],
	},
	{
	    .name = "Voice",
	    .playback = {
		    .stream_name = "Voice Playback",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
            },
	    .capture = {
		    .stream_name = "Voice Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
            },
	    .ops = {
        	    .startup = bcm476x_codec_stream_startup,
		    .prepare = bcm476x_voice_prepare,
		    .shutdown = bcm476x_voice_shutdown,
	            .digital_mute = bcm476x_voice_mute,
            },
            .private_data = &dai_priv[BCM476X_VOICE_DAI],
        },
	{
	    .name = "PCM",
	    .playback = {
			.stream_name = "PCM Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
	    },
	    .capture = {
			.stream_name = "PCM Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
	    },
	    .ops = {
			.startup = bcm476x_codec_stream_startup,
			.prepare = bcm476x_pcm_prepare,
			.shutdown = bcm476x_pcm_shutdown,
	    },
	    .private_data = &dai_priv[BCM476X_BT_DAI],
	},
};
EXPORT_SYMBOL_GPL(bcm476x_dai);

static void bcm476x_codec_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
    //printk(KERN_INFO "bcm476x_codec_work: set bias_level=%d\n", codec->bias_level);
	bcm476x_set_bias_level(codec, codec->bias_level);
}

static int bcm476x_codec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	printk(KERN_INFO "bcm476x_codec_suspend: \n");
	codec->bias_level = SND_SOC_BIAS_OFF;
	bcm476x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int bcm476x_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	printk(KERN_INFO "bcm476x_codec_resume: \n");
	codec_init_if(); 
	bcm476x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (codec->suspend_bias_level == SND_SOC_BIAS_ON) {
		codec->bias_level = SND_SOC_BIAS_ON;
		schedule_delayed_work(&codec->delayed_work,
					msecs_to_jiffies(10));
	}

	return 0;
}

/*
 * initialise the BCM476X driver
 * register the mixer and dsp interfaces with the kernel
 */
static int bcm476x_codec_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "BCM476X";
	codec->owner = THIS_MODULE;
	codec->read = bcm476x_codec_read_reg;
	codec->write = bcm476x_codec_write_reg;
	codec->read = bcm476x_codec_read_reg;
	codec->hw_write = bcm476x_codec_write_buf;
	codec->hw_read = bcm476x_codec_read_buf;
	codec->set_bias_level = bcm476x_set_bias_level;
	codec->dai = bcm476x_dai;
	codec->num_dai = ARRAY_SIZE(bcm476x_dai);
	codec->reg_cache_size = 0;
	codec->reg_cache = NULL;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "bcm476x: failed to create pcms\n");
		goto pcm_err;
	}
	bcm476x_add_controls(codec);
	bcm476x_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "bcm476x: failed to register card\n");
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

#include <plat/fdt.h>
static void bcm476x_amplifier(void)
{
	const char *amp;

	amp = fdt_get_string ("/features", "amplifier", "ALC5627");
	bcm476x_amp = BCM476X_AMP_ALC5627;

	if (strcmp("ALC5627", amp) == 0) {
		bcm476x_amp = BCM476X_AMP_ALC5627;
	} else if (strcmp("EUA2011", amp) == 0) {
		bcm476x_amp = BCM476X_AMP_EUA2011;
	}
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *bcm476x_socdev;

static int bcm476x_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct bcm476x_codec_priv *bcm476x;
	int ret = 0;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	bcm476x = kzalloc(sizeof(struct bcm476x_codec_priv), GFP_KERNEL);
	if (bcm476x == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	spin_lock_init(&bcm476x->codec_lock);
	codec->private_data = bcm476x;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	bcm476x_socdev = socdev;
    ret = bcm476x_codec_init(bcm476x_socdev);

	bcm476x_amplifier();

	if (ret != 0) {
		kfree(codec->private_data);
		kfree(codec);
	}
    INIT_DELAYED_WORK(&codec->delayed_work, bcm476x_codec_work);
	return ret;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int bcm476x_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		codec->bias_level = SND_SOC_BIAS_OFF;
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec->private_data);
	kfree(codec);
    codec_exit_if();
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_bcm476x = {
	.probe = 	bcm476x_codec_probe,
	.remove = 	bcm476x_codec_remove,
	.suspend = 	bcm476x_codec_suspend,
	.resume =	bcm476x_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_bcm476x);

/******************************************************************************************
 * The following functions are for mapping to the bcm476x codec hal driver's functions.
 ******************************************************************************************/
/****************************************************************************
*
*  halAudio_registerSOCAudio
*
*  This routine is called by the HAL driver when it is loaded to give us
*  a table of function pointers to the functions we need from the HAL.
*
***************************************************************************/
int halAudio_registerSOCAudio(HAL_AUDIO_FNCS *audiop)
{
	int gain, value;
	//printk(KERN_INFO "halAudio_registerAudio:\n");
	memset(aud_vir_regs, 0, sizeof(aud_vir_regs));
	gain = read_aud_reg(REG_ALSLOPGAIN_ADDR) & 0x1ff;
	printk("halaudio registerSOCAudio gain %x\n",gain);
	write_aud_reg(REG_VIRTUAL_LGAIN, gain); // initial virtual gain to hw's audio gain
	gain = read_aud_reg(REG_ARSLOPGAIN_ADDR) & 0x1ff;
	write_aud_reg(REG_VIRTUAL_RGAIN, gain); // initial virtual gain to hw's audio gain
	memcpy( (char *)&ghalFuncs, (char *)audiop, sizeof(ghalFuncs));
	codec_ops = &ghalFuncs;
	//codec_init_if();

	/* set max volume levels for boot up sound */ 
	/* ensure voice channel is at same level as audio channel */
	value = read_aud_reg(REG_ALSLOPGAIN_ADDR);
	value = (value & ~0xffff) | 0;
	write_aud_reg(REG_VCFGR_ADDR, value);

	/* set mixer level to best performance */
	value = read_aud_reg(REG_MIXER_GAIN_CHSEL_ADDR);
	value = (value & ~0xffff) | 0;
	write_aud_reg(REG_MIXER_GAIN_CHSEL_ADDR, value);

	value = read_aud_reg(REG_MIXER_GAIN_ADJUST_ADDR);
	value = (value & ~0xffff) | 0x2000;
	write_aud_reg(REG_MIXER_GAIN_ADJUST_ADDR, value);

	value = read_aud_reg(REG_MIXER_GAIN_CHSEL_ADDR);
	value = (value & ~0xffff) | 1;
	write_aud_reg(REG_MIXER_GAIN_CHSEL_ADDR, value);

	value = read_aud_reg(REG_MIXER_GAIN_ADJUST_ADDR);
	value = (value & ~0xffff) | 0x2000;
	write_aud_reg(REG_MIXER_GAIN_ADJUST_ADDR, value);


	/* set low boot attenuation level for audio left and right channels and voice channel */
	#define BOOT_OUTPUT_ATT (64)
	value = read_aud_reg(REG_ALSLOPGAIN_ADDR);
	value = (value & ~0x7ff) | (0x01ff - BOOT_OUTPUT_ATT);
	write_aud_reg(REG_ALSLOPGAIN_ADDR, value);

	value = read_aud_reg(REG_ARSLOPGAIN_ADDR);
	value = (value & ~0x7ff) | (0x01ff - BOOT_OUTPUT_ATT);
	write_aud_reg(REG_ARSLOPGAIN_ADDR, value);

	value = read_aud_reg(REG_VSLOPGAIN_ADDR);
	value = (value & ~0x7ff) | (0x01ff - BOOT_OUTPUT_ATT);
	write_aud_reg(REG_VSLOPGAIN_ADDR, value);

	return 0;
}

EXPORT_SYMBOL(halAudio_registerSOCAudio);

int bcm476x_codec_set_volume_db(int target_db)
{
	int audio_block;
	int rc = -EINVAL;

	audio_block = 0x11;

	/* printk ("Setting codec volume for block 0x%x to: %d ", audio_block, target_db);*/

	if (codec_ops->gainSetAnalogHardware)
		rc = codec_ops->gainSetAnalogHardware(audio_block, target_db);

        return 1;
}
EXPORT_SYMBOL(bcm476x_codec_set_volume_db);

static void bcm476x_codec_buffer_done_nofify(unsigned short numSamples, short *buffer, void *cb_param)
{
    //printk(KERN_INFO "bcm476x_codec: buffer_done device=%d, stream=%d, nsamples=%d\n", substream->pcm->device, substream->stream, numSamples);
    if (xferdone_cb) {  // callback to the requester to handle the xfer done.
        (xferdone_cb)(cb_param, 0, numSamples * 2);
    }
    else {
        struct snd_pcm_substream *substream = cb_param;
        struct snd_soc_dai *dai = &bcm476x_dai[substream->pcm->device];
        struct bcm476x_dai_priv *dai_rtd = dai->private_data;

        // driver handles the xfer done.
        spin_lock_irq(&dai_rtd->dai_lock);
        dai_rtd->xfered_len[substream->stream] = numSamples;
        up(&dai_rtd->stream_sync[substream->stream]);
        spin_unlock_irq(&dai_rtd->dai_lock);
    }
}

// Wait for buffer done and returns the number of samples transferred.
static int wait_codec_buffer_done(struct snd_pcm_substream *substream)
{
    int numSamples;
    struct snd_soc_dai *dai = &bcm476x_dai[substream->pcm->device];
    struct bcm476x_dai_priv *dai_rtd = dai->private_data;

    if (down_interruptible(&dai_rtd->stream_sync[substream->stream])) {
	printk (KERN_ERR "Unable to take semaphore in %s\n", __FILE__);
	return -ERESTARTSYS;
    }
    spin_lock_irq(&dai_rtd->dai_lock);
    numSamples = dai_rtd->xfered_len[substream->stream];
    dai_rtd->xfered_len[substream->stream] = 0;
    spin_unlock_irq(&dai_rtd->dai_lock);
    return numSamples;
}

static void codec_init_if(void)
{
    if (codec_ops && codec_ops->init)
        codec_ops->init(NULL);
}

static void codec_exit_if(void)
{
    if (codec_ops && codec_ops->exit)
        codec_ops->exit();
}

/* Enable the codec */
static void codec_on_if(void)
{
    if (codec_ops->enable)
        codec_ops->enable(HAL_AUDIO_POWER_RESUME_ALL);
}

/* Disable the codec */
static void codec_off_if(void)
{
    if (codec_ops->enable)
        codec_ops->enable(HAL_AUDIO_POWER_DEEP_SLEEP);
}

/* Set the codec in ready */
static void codec_standby_if(void)
{
    if (codec_ops->enable)
       codec_ops->enable(HAL_AUDIO_POWER_DIGITAL_ONLY);
}

static HAL_AUDIO_CODEC codec_map(struct snd_pcm_substream *substream)
{
    HAL_AUDIO_CODEC codec;
    struct snd_pcm_runtime *runtime = substream->runtime;
    if (BCM476X_BT_DAI == substream->pcm->device)
        codec = HAL_AUDIO_CODEC2;
    else if (runtime->rate <= 16000)
    {
        if (substream->pcm->device == BCM476X_STEREO_DAI && SNDRV_PCM_STREAM_PLAYBACK == substream->stream)   /* Play thru stereo path */
            codec = HAL_AUDIO_CODEC0;
        else
            codec = HAL_AUDIO_CODEC1;
    }
    else
        codec = HAL_AUDIO_CODEC0;
    return codec;
}

static int audio_block_map(struct snd_pcm_substream *substream)
{
    int block;
    struct snd_pcm_runtime *runtime = substream->runtime;
    if (BCM476X_BT_DAI == substream->pcm->device)
    {
        if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
            block = HAL_AUDIO_CODEC2A_SPKR;
        else
            block = HAL_AUDIO_CODEC2A_MIC;
    }
    else if (runtime->rate <= 16000)
    {
        if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
        {
            if (substream->pcm->device == BCM476X_STEREO_DAI)   /* Play thru stereo path */
                block = HAL_AUDIO_CODEC0A_SPKR;
            else
                block = HAL_AUDIO_CODEC1A_SPKR;
        }
        else
            block = HAL_AUDIO_CODEC1A_MIC;  
    }
    else
    {
       if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream)
            block = HAL_AUDIO_CODEC0A_SPKR;
        else
            block = HAL_AUDIO_CODEC0A_MIC;
    }
    return block;
}

static int codec_analog_gain_get_if(struct snd_pcm_substream *substream, int *cur_db, int *min_db, int *max_db)
{
    int rc = -EINVAL;
    int audio_block = audio_block_map(substream);
    if (codec_ops->getCapabilities) {
        HAL_AUDIO_CAPABILITIES cap;
        rc = codec_ops->getCapabilities(audio_block, &cap);
        *cur_db = cap.analog.currentDB;
        *min_db = cap.analog.minDB;
        *max_db = cap.analog.maxDB;
    }
    return rc;
}

static int codec_analog_gain_set_if(struct snd_pcm_substream *substream, int db)
{
    int rc = -EINVAL;
    int audio_block = audio_block_map(substream);
    if (codec_ops->gainSetAnalogHardware)
        rc = codec_ops->gainSetAnalogHardware(audio_block, db);
    return rc;
}

#if 0 // removed, not used
static int codec_digital_gain_get_if(struct snd_pcm_substream *substream, int *cur_db, int *min_db, int *max_db)
{
    int rc = -EINVAL;
    int audio_block = audio_block_map(substream);
    if (codec_ops->getCapabilities) {
        HAL_AUDIO_CAPABILITIES cap;
        rc = codec_ops->getCapabilities(audio_block, &cap);
        *cur_db = cap.digital.currentDB;
        *min_db = cap.digital.minDB;
        *max_db = cap.digital.maxDB;
    }
    return rc;
}
static int codec_digital_gain_set_if(struct snd_pcm_substream *substream, int db)
{
    int rc = -EINVAL;
    int audio_block = audio_block_map(substream);
    if (codec_ops->gainSetDigitalHardware)
        rc  = codec_ops->gainSetDigitalHardware(audio_block, db);
    return rc;
}
#endif

static int codec_write_buf_if(struct snd_pcm_substream *substream, short *buffer, int len)
{
    int n = -1;
    int numSamples = len >> 1;
    struct snd_pcm_runtime *runtime = substream->runtime;
    HAL_AUDIO_CODEC halAudioCodec;

    /* Get halAudio codec mapping */
    halAudioCodec = codec_map(substream);
    if(codec_ops->write) {
        n = codec_ops->write(numSamples, runtime->channels, halAudioCodec, buffer, &bcm476x_codec_buffer_done_nofify, substream);
        if (NULL == xferdone_cb) { // no callback registered
            if (n >=0 && n < numSamples)
                n = wait_codec_buffer_done(substream);
        }
    }
    return (n<<1);
}

static int codec_read_buf_if(struct snd_pcm_substream *substream, short *buffer, int len)
{
    int n = -1;
    int numSamples = len >> 1;
    struct snd_pcm_runtime *runtime = substream->runtime;
    HAL_AUDIO_CODEC halAudioCodec;

    /* Get halAudio codec mapping */
    halAudioCodec = codec_map(substream);
    if(codec_ops->read) {
        n = codec_ops->read(numSamples, runtime->channels, halAudioCodec, buffer, &bcm476x_codec_buffer_done_nofify, substream);
        if (NULL == xferdone_cb) { // no callback registered
            if (n >= 0 && n < numSamples) 
                n = wait_codec_buffer_done(substream);
        }
    }
    return (n<<1);
}

MODULE_DESCRIPTION("ASoC BCM476X driver");
MODULE_VERSION(BCM476X_SND_VERSION);
MODULE_LICENSE("GPL");
