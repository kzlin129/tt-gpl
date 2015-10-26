#include <linux/init.h>
#include <linux/module.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "asm/arch/hardware.h"
#include <asm/io.h>

#include <linux/broadcom/halaudio.h>

#include <plat/scenarii.h>
#include <plat/tt_setup_handler.h>

#include <plat/irvine.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/pinmux.h>
#include <linux/delay.h>            /* for udelay & msleep */
#include <linux/broadcom/gpio.h>

#include "../../../sound/soc/codecs/bcm476x_codec.h"

#define DUMMY_CALLBACK(name)static int dummy_##name(struct tt_scenario_context *c) \
		{							\
			/* Do nothing */				\
			return 0;					\
		}

int alc5627_i2c_line_in_gain(int);
int alc5627_i2c_spk_gain(int);
int alc5627_i2c_amp_ratio(int);
int alc5627_i2c_mute(void);
int alc5627_i2c_unmute_hp(void);
int alc5627_i2c_unmute_spkr(void);

void alc5627_i2c_enable_paths(void);
void alc5627_i2c_disable_paths(void);

#define BCM4760_MINGAIN (0)
#define BCM4760_MAXGAIN (42)
#ifdef PREAMP_ENABLED
	#define PREAMPGAIN (15)
#else
	#define PREAMPGAIN (0)
#endif
#define OUTPUT_ATT (30) /* 481 (-7.5dB) normal max output level */
#define OUTPUT_LINE_OUT_ATT (33) /* ~ 0.900Vrms on 2nd to highest level measured unit */
#define OUTPUT_HF_ATT (26) /*  (-6.50dB) normal max output level without distortion */
#define OUTPUT_HF_LINE_OUT_ATT (42) /* a little below 0.600Vrms */
#define VOLUME_MUTE (90)
#define GPIO_PULLUP 1

//#define HF_OVER_LINE_OUT (1)


static int bcm4760_set_volume_db(struct tt_scenario_context *c)
{
	struct snd_kcontrol	*kcontrol = (struct snd_kcontrol *) c->data;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int output_gain = ((0x1ff - OUTPUT_ATT) - (120 << 2));
	int VCFGR_gain = 0x00;

	if (c->tt_volume_dB < VOLUME_MUTE) {
		switch(c->tt_scenario) {
		default:
		case TT_SCENARIO_AUDIO_INIT:
		case TT_SCENARIO_AUDIO_OFF:
			break;
		case TT_SCENARIO_LINEOUT:
			output_gain = ((0x1ff - c->tt_max_line_out) - (c->tt_volume_dB << 2));
			c->tt_scenario_line_out_volume = output_gain;
			break;
		case TT_SCENARIO_HF_LINEOUT:
			if (c->tt_hf_line_out != 0) {
				/* TODO: measure levels with (HF) Voice path as it may require 4 higher than Audio Path, for now use the AUdio path level */ 
				output_gain = (0x1ff - c->tt_max_line_out);
				break;
			}
		case TT_SCENARIO_HF_SPEAKER:
			output_gain = (0x01ff - OUTPUT_HF_ATT); /* 0 db, fixed line out level */
			break;
		case TT_SCENARIO_SPEAKER:
			output_gain = ((0x1ff - OUTPUT_ATT) - (c->tt_volume_dB << 2));
			c->tt_scenario_speaker_volume = output_gain;
			break;
		}
	}

	/* ensure voice channel is always at same level as audio channel */
	snd_soc_update_bits(codec, REG_VCFGR_ADDR, 0xffff, VCFGR_gain);

	/* ensure mixer level is always set to best performance */
	snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 0);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, 0x2000);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 1);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, 0x2000);	

 	/* set attenuation level for audio left and right channels and voice channel */
	snd_soc_update_bits(codec, REG_ALSLOPGAIN_ADDR, 0x07ff, output_gain);	
	snd_soc_update_bits(codec, REG_ARSLOPGAIN_ADDR, 0x07ff, output_gain);	
	snd_soc_update_bits(codec, REG_VSLOPGAIN_ADDR, 0x07ff, output_gain);	

	return 1;
}

static void bcm4760_mic_gain(int mic_gain)
{
	unsigned long gain_addr_1, gain_addr_2;
	unsigned long gain_ctrl_1, gain_ctrl_2;
	int cur_gain;

	mic_gain -= PREAMPGAIN;
	if (mic_gain < BCM4760_MINGAIN) {
		mic_gain = BCM4760_MINGAIN;
	} else if (mic_gain > BCM4760_MAXGAIN) {
		mic_gain = BCM4760_MAXGAIN;
	}

	gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR); // 0xB0204
	gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL2_MEMADDR); // 0xB0208
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

	gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL4_MEMADDR); // 0xB0220
	gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL5_MEMADDR); // 0xB0224
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
}

static int bcm4760_set_mic_gain(struct tt_scenario_context *c)
{
	bcm4760_mic_gain(c->tt_mic_gain);
	return 1;
}

static int bcm4760_set_line_out_level(struct tt_scenario_context *c)
{
	/* find match for line out level in table of measured 2 highest output devices */

	#define LINE_LEVEL_ELEMENTS (78)
	int line_level [LINE_LEVEL_ELEMENTS] = {
		900, 875, 850, 826, 802, 779, 757, 735, 714, 694, 674, 655, 637, 618, 601, 584, 
		567, 551, 535, 520, 505, 491, 477, 463, 450, 437, 425, 412, 401, 389, 378, 367, 
		357, 347, 337, 327, 318, 309, 300, 292, 283, 275, 268, 260, 253, 245, 238, 232, 
		225, 219, 213, 206, 200, 195, 189, 184, 179, 174, 169, 164, 159, 155, 151, 146, 
		142, 138, 134, 130, 127, 123, 120, 117, 113, 110, 107, 104, 101, 99
	};

	int k = OUTPUT_LINE_OUT_ATT;
	int i = 0;
	for (;i < LINE_LEVEL_ELEMENTS; i++) {
		if (c->tt_line_out_level >= line_level[i])
			break;
		k++;
	}

	c->tt_max_line_out = k;

	return 1;
}

static int bcm4760_alc5627_unmute_amp(struct tt_scenario_context *c)
{
	alc5627_i2c_unmute_spkr();
	alc5627_i2c_unmute_hp();
	return 1;
}

#ifdef INCLUDE_TEST_KCONTROLS
static int bcm4760_set_gain(struct tt_scenario_context *c)
{
	struct snd_kcontrol	*kcontrol = (struct snd_kcontrol *) c->data;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	unsigned long gain_addr_1, gain_addr_2;
	unsigned long gain_ctrl_1, gain_ctrl_2;
	int cur_gain;

	switch (c->tt_test_gain_type) {
		case 1:
			snd_soc_update_bits(codec, REG_ALSLOPGAIN_ADDR, 0x07ff, c->tt_test_gain_value);	
			snd_soc_update_bits(codec, REG_ARSLOPGAIN_ADDR, 0x07ff, c->tt_test_gain_value);	
			snd_soc_update_bits(codec, REG_VSLOPGAIN_ADDR, 0x07ff, c->tt_test_gain_value);
			break;
		case 2:
			snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 0);	
			snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, c->tt_test_gain_value);	
			snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 1);	
			snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, c->tt_test_gain_value);	
			break;
		case 3:
			snd_soc_update_bits(codec, REG_VCFGR_ADDR, 0xffff, c->tt_test_gain_value);	
			break;
		case 11:
			alc5627_i2c_line_in_gain(c->tt_test_gain_value);
			break;
		case 12:
			alc5627_i2c_spk_gain(c->tt_test_gain_value);
			break;
		case 13:
			alc5627_i2c_amp_ratio(c->tt_test_gain_value);
			break;
		case 21:
			gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL1_MEMADDR); // 0xB0204
			gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL2_MEMADDR); // 0xB0208
			gain_ctrl_1 = readl(gain_addr_1);
			gain_ctrl_2 = readl(gain_addr_2);
			cur_gain = ((gain_ctrl_1 >> 30) & 3) | ((gain_ctrl_2 & 0x0f) << 2);
			if (cur_gain != c->tt_test_gain_value) {
				gain_ctrl_1 &= 0x3fffffff;
				gain_ctrl_1 |= ((c->tt_test_gain_value & 3) << 30);
				gain_ctrl_2 &= 0xfffffff0;
				gain_ctrl_2 |= ((c->tt_test_gain_value & 0x3c) >> 2);
				writel(gain_ctrl_1, gain_addr_1); 
				writel(gain_ctrl_2,gain_addr_2); 
			}
			break;
		case 22:
			gain_addr_1 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL4_MEMADDR); // 0xB0220
			gain_addr_2 = IO_ADDRESS(CMU_R_ADM_RXAUDIO_CTRL5_MEMADDR); // 0xB0224
			gain_ctrl_1 = readl(gain_addr_1);
			gain_ctrl_2 = readl(gain_addr_2);
			cur_gain = ((gain_ctrl_1 >> 30) & 3) | ((gain_ctrl_2 & 0x0f) << 2);
			if (cur_gain != c->tt_test_gain_value) {
				gain_ctrl_1 &= 0x3fffffff;
				gain_ctrl_1 |= ((c->tt_test_gain_value & 3) << 30);
				gain_ctrl_2 &= 0xfffffff0;
				gain_ctrl_2 |= ((c->tt_test_gain_value & 0x3c) >> 2);
				writel(gain_ctrl_1, gain_addr_1); 
				writel(gain_ctrl_2,gain_addr_2); 
			}
			break;
		default:
			printk("unknown gain type\n");
			break;
	}

	return 1;
}
#endif

static int bcm4760_set_scenario(struct tt_scenario_context *c)
{
	/* set audio routing scenarion and resore the output gain */
	struct snd_kcontrol	*kcontrol = (struct snd_kcontrol *) c->data;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int output_gain = c->tt_scenario_speaker_volume;
	int VCFGR_gain = 0x00;

	switch(c->tt_scenario) {
	case TT_SCENARIO_LINEOUT:
		output_gain = c->tt_scenario_line_out_volume;
		break;
	default:
	case TT_SCENARIO_AUDIO_INIT:
		/* set default max volume and go to off state */
		output_gain = (0x1ff - OUTPUT_ATT);
		c->tt_scenario_speaker_volume = output_gain;
	case TT_SCENARIO_AUDIO_OFF:
		/* set speaker output to approx next playback volume level as this slightly reduces pops */ 
		if (c->tt_current_scenario == TT_SCENARIO_LINEOUT) {
			output_gain = c->tt_scenario_line_out_volume;
		} else {
			output_gain = c->tt_scenario_speaker_volume;
		}
		break;
	case TT_SCENARIO_ASR:
		break;
	case TT_SCENARIO_HF_LINEOUT:
		if (c->tt_hf_line_out != 0) {
			/* TODO: measure levels with (HF) Voice path as it may require 4 higher than Audio Path, for now use the AUdio path level */ 
			output_gain = (0x1ff - c->tt_max_line_out);
			break;
		}
	case TT_SCENARIO_HF_SPEAKER:
		output_gain = (0x01ff - OUTPUT_HF_ATT); /* 0 db, fixed line out level */
		break;
	case TT_SCENARIO_SPEAKER:
		output_gain = c->tt_scenario_speaker_volume;
		break;
	}

	/* set BCM4760 volume level close to active level before alc5627 is unmuted to reduce pop */
	/* ensure voice channel is always at same level as audio channel */
	snd_soc_update_bits(codec, REG_VCFGR_ADDR, 0xffff, VCFGR_gain);

	/* ensure mixer level is always set to best performance */
	snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 0);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, 0x2000);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_CHSEL_ADDR, 0xffff, 1);	
	snd_soc_update_bits(codec, REG_MIXER_GAIN_ADJUST_ADDR, 0xffff, 0x2000);	

 	/* set attenuation level for audio left and right channels and voice channel */
	snd_soc_update_bits(codec, REG_ALSLOPGAIN_ADDR, 0x07ff, output_gain);	
	snd_soc_update_bits(codec, REG_ARSLOPGAIN_ADDR, 0x07ff, output_gain);	
	snd_soc_update_bits(codec, REG_VSLOPGAIN_ADDR, 0x07ff, output_gain);	

	/* out of spec delay between BCM4760 and ALC5627 to reduce pop */ 
	msleep(32);

	switch(c->tt_scenario) {
	case TT_SCENARIO_HF_LINEOUT:
		if (c->tt_hf_line_out != 0) {
			alc5627_i2c_unmute_hp();
		} else {
			alc5627_i2c_unmute_spkr();
		}
		break;
	case TT_SCENARIO_LINEOUT:
		alc5627_i2c_unmute_hp();
		break;
	default:
	case TT_SCENARIO_AUDIO_INIT:
		/* go to off state */
		alc5627_i2c_mute();
	case TT_SCENARIO_AUDIO_OFF:
		/* alc5627 mute is called from BCM476x codec driver to reduce pop from BCM476x */
		//alc5627_i2c_mute();
		break;
	case TT_SCENARIO_ASR:
		break;
	case TT_SCENARIO_HF_SPEAKER:
		alc5627_i2c_unmute_spkr();
		break;
	case TT_SCENARIO_SPEAKER:
		alc5627_i2c_unmute_spkr();
		break;
	}

	c->tt_current_scenario = c->tt_scenario;

	return 0;
}

DUMMY_CALLBACK(set_volume)		/* Afaik, set_volume is not really used */
DUMMY_CALLBACK(set_scenario_mode)	/* implementing dapm-like functions. */

static int bcm4760_hpdetect_callback(struct tt_scenario_context *cntx)
{
	return !gpio_get_value(TT_VGPIO_DOCK_HPDETECT);
}

static int bcm4760_get_hpdetect_irq(struct tt_scenario_context *cntx)
{
	return gpio_to_irq(TT_VGPIO_DOCK_HPDETECT);
}

static struct tt_scenario_ops bcm4760_scenario_ops = 
{
	.set_volume         = dummy_set_volume,
	.set_volume_db      = bcm4760_set_volume_db,
	.set_scenario       = bcm4760_set_scenario,
	.set_scenario_mode  = dummy_set_scenario_mode,
	.set_mic_gain       = bcm4760_set_mic_gain,
	.set_line_out_level = bcm4760_set_line_out_level,
	.unmute_amp         = bcm4760_alc5627_unmute_amp,
	.hpdetect_callback  = bcm4760_hpdetect_callback,
	.get_hpdetect_irq   = bcm4760_get_hpdetect_irq,
#ifdef INCLUDE_TEST_KCONTROLS
	.set_gain           = bcm4760_set_gain,
#endif
};

static int __init tt_nashville_alc5627_setup(char *str)
{
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_HPDETECT), BCM4760_PIN_MUX_GPIO);

	/* Set pull-up because as of the BonBom project the external pull-up got removed */
	reg_gpio_set_pull_up_down (vgpio_to_gpio (TT_VGPIO_DOCK_HPDETECT), GPIO_PULLUP);
	reg_gpio_set_pull_up_down_enable (vgpio_to_gpio (TT_VGPIO_DOCK_HPDETECT));

	tt_scenario_register(&bcm4760_scenario_ops);

	printk (KERN_INFO "TomTom ALC5627 ALSA Nashville scenario controls, (C) 2009 TomTom BV\n");

	return 0;
}

TT_SETUP_CB(tt_nashville_alc5627_setup, "tomtom-bcm-amplifier-alc5627");

