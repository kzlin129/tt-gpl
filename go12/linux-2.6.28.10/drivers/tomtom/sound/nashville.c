/* drivers/tomtom/sound/nashville.c
 *
 * Implements the ALSA controls that are required by Nashville. Platform-specific code is deferred
 * to other files in this directory.
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Guillaume Ballet <Guillaume.Ballet@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <plat/scenarii.h>
#include <plat/fdt.h>

#define MAXTESTGAINTYPE (65535)
#define MAXTESTGAINVALUE (65535)
#define TESTGAINRESET MAXTESTGAINVALUE

static int default_callback( struct tt_scenario_context *c )
{
	/* Do almost nothing */
	printk( "TTScenario: default callback\n" );
	return 0;
}

static struct tt_scenario_ops default_scenario_ops = {
        .set_volume             = default_callback,
        .set_volume_db          = default_callback,
        .set_scenario           = default_callback,
        .set_scenario_mode      = default_callback
};

/* Ideally, it should be possible to have several sound devices,
 * yet this is not an urgent problem. The interface is meant to
 * be flexible enough to do that if necessary.
 */
static struct tt_scenario_context tt_context=
{
	.ops			= &default_scenario_ops,
};

static const char *tt_scenarii[] = {
	"Init",
	"Off",
	"ASR",
	"VR",
	"Noise",
	"iPod",
	"Speaker",
	"LineOut",
	"FM",
	"iPodSpeaker",
	"iPodLineOut",
	"iPodFM",
	"HFSpeaker",
	"HFLineOut",
	"HFFM",
	"None",
};

static const char *tt_scenario_modes[] = {
	"Off",
	"SPP",
	"TTS",
	"MP3",
	"iPod",
	"HF",
	"ASR",
	"VR",
	"POI",
};

static const struct soc_enum tt_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tt_scenarii),tt_scenarii),
};

static int tt_set_volume(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_volume = ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_volume) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_volume(&tt_context);
	}

	return 1;
}

static int tt_get_volume(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_volume;
	return 0;
}

/* user of this function will pass in a the gain as a 
   possitive value which should be interpreted as negative */
static int tt_set_volume_dB(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_volume_dB = ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_volume_db) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_volume_db(&tt_context);
	}

	return 1;
}

static int tt_get_volume_dB(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_volume_dB;
	return 0;
}

static int tt_set_mic_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_mic_gain = ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_mic_gain) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_mic_gain(&tt_context);
	}

	return 1;
}

static int tt_get_mic_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_mic_gain;
	return 0;
}

static int tt_set_speaker_level(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_speaker_level = ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_speaker_level) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_speaker_level(&tt_context);
	}

	return 1;
}

static int tt_get_speaker_level(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_speaker_level;
	return 0;
}

static int tt_set_line_out_level(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_line_out_level = ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_line_out_level) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_line_out_level(&tt_context);
	}

	return 1;
}

static int tt_get_line_out_level(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_line_out_level;
	return 0;
}


static int tt_hf_line_out;
static int tt_set_hf_line_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_hf_line_out = ucontrol->value.integer.value[0];
	tt_hf_line_out = tt_context.tt_hf_line_out;

	if (tt_context.ops && tt_context.ops->set_hf_line_out) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_hf_line_out(&tt_context);
	}

	return 1;
}

static int tt_get_hf_line_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_hf_line_out;
	return 0;
}

#ifdef INCLUDE_TEST_KCONTROLS
static int tt_set_gain_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_test_gain_type = ucontrol->value.integer.value[0];

	return 1;
}

static int tt_get_gain_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_test_gain_type;
	return 0;
}

static int tt_set_gain_value(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	tt_context.tt_test_gain_value = ucontrol->value.integer.value[0];

	if (tt_context.tt_test_gain_value == TESTGAINRESET) {
		/* reset test gain mechanism */
		tt_context.tt_test_gain_type = -1;
	} else if (tt_context.ops && tt_context.ops->set_gain) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_gain(&tt_context);
	}


	return 1;
}

static int tt_get_gain_value(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_test_gain_value;
	return 0;
}
#endif

static int tt_set_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (tt_context.tt_scenario == ucontrol->value.integer.value[0])
		return 0;
	
	tt_context.tt_scenario	= ucontrol->value.integer.value[0];

	if (tt_context.ops && tt_context.ops->set_scenario) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_scenario(&tt_context);
	}

	return 1;
}

static int tt_get_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_scenario;
	return 0;
}

static int tt_get_scenario_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tt_context.tt_scenario_mode;
	return 0;
}

static int tt_set_scenario_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (tt_context.tt_scenario_mode == ucontrol->value.integer.value[0])
		return 0;

	tt_context.tt_scenario_mode	= ucontrol->value.integer.value[0];

	/* XXX there was a call to set_scenario_endpoints, so check if something can be factorized there */

	if (tt_context.ops && tt_context.ops->set_scenario_mode) {
		tt_context.data	= kcontrol;
		tt_context.ops->set_scenario_mode(&tt_context);
	}

	return 1;
}

static int tt_hpdetect;
static struct snd_card *tt_hpdetect_card = NULL;
static struct snd_ctl_elem_id *tt_hpdetect_elem_id = NULL;

static irqreturn_t tt_hpdetect_irq(int irq, void *dev_id)
{
	if (tt_context.ops && tt_context.ops->hpdetect_callback) {
		tt_hpdetect	=  tt_context.ops->hpdetect_callback(&tt_context);

		if ((tt_hpdetect_card) && (tt_hpdetect_elem_id))
		{
		  snd_ctl_notify(tt_hpdetect_card, SNDRV_CTL_EVENT_MASK_VALUE, tt_hpdetect_elem_id);
		}
	}

	return IRQ_HANDLED;
}

static int tt_hpdetect_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	if (tt_context.ops->hpdetect_callback){
		ucontrol->value.integer.value[0]	= tt_context.ops->hpdetect_callback(&tt_context);
	} else {
		ucontrol->value.integer.value[0]	= tt_hpdetect;
	}
	return 0;
}

static int tt_hpdetect_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return 1;
}

static int tt_set_unmute_amp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (tt_context.ops && tt_context.ops->unmute_amp) {
		tt_context.data	= kcontrol;
		tt_context.ops->unmute_amp(&tt_context);
	}
	return 1;
}

static int tt_get_unmute_amp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}


static const struct soc_enum tt_scenario_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tt_scenario_modes),tt_scenario_modes),
};

static const struct snd_kcontrol_new tt_controls[] = {
	SOC_ENUM_EXT("TT Scenario", tt_scenario_enum[0],
		tt_get_scenario, tt_set_scenario),
	SOC_ENUM_EXT("TT Mode", tt_scenario_mode_enum[0],
		tt_get_scenario_mode, tt_set_scenario_mode),
	SOC_SINGLE_EXT("TT Playback Volume %", 0, 0, 100, 0,
		tt_get_volume, tt_set_volume),
	SOC_SINGLE_EXT("TT Playback Volume dB", 0, 0, 90, 0,
		tt_get_volume_dB, tt_set_volume_dB),
	SOC_SINGLE_EXT("TT Mic Gain", 0, 0, 64, 0,
		tt_get_mic_gain, tt_set_mic_gain),
	SOC_SINGLE_EXT("TT Speaker Level", 0, 0, 2147483647, 0,
		tt_get_speaker_level, tt_set_speaker_level),
	SOC_SINGLE_EXT("TT Line Out Level", 0, 0, 900000, 0,
		tt_get_line_out_level, tt_set_line_out_level),
	SOC_SINGLE_BOOL_EXT("TT BTHF Line Out", &tt_hf_line_out,
		tt_get_hf_line_out, tt_set_hf_line_out),
	SOC_SINGLE_EXT("Unmute Amp", 0, 0, 1, 0,
		tt_get_unmute_amp, tt_set_unmute_amp),
	SOC_SINGLE_BOOL_EXT("TT Headphones Detect", &tt_hpdetect,
		tt_hpdetect_get, tt_hpdetect_put),
	#ifdef INCLUDE_TEST_KCONTROLS
	SOC_SINGLE_EXT("TT Test Gain Type", 0, 0, MAXTESTGAINTYPE, 0,
		tt_get_gain_type, tt_set_gain_type),
	SOC_SINGLE_EXT("TT Test Gain Value", 0, 0, MAXTESTGAINVALUE, 0,
		tt_get_gain_value, tt_set_gain_value),
	#endif
};

int tomtom_add_nashville_controls(struct snd_card *card, void *data)
{
	int	idx, err;
	struct snd_kcontrol *kcontrol;

	for (idx=0; idx<ARRAY_SIZE(tt_controls); idx++) {
	        kcontrol = snd_ctl_new1(&tt_controls[idx], data);
		if ((err = snd_ctl_add(card, kcontrol)) < 0)
			return err;

		if (!strcmp(tt_controls[idx].name, "TT Headphones Detect"))
		{
		  tt_hpdetect_card = card;
		  tt_hpdetect_elem_id = &kcontrol->id;
		}
	}
	return 0;
}

#define DEFAULT_SPEAKER_VOLUME (15)
struct tt_scenario_context *tt_scenario_register(struct tt_scenario_ops *ops)
{
	int ret;

	tt_context.tt_volume = 0;
	tt_context.tt_volume_dB = 90;
	tt_context.tt_scenario = TT_SCENARIO_NONE;
	tt_context.tt_scenario_mode = TT_MODE_INIT;
	tt_context.tt_mic_gain = 0;
	tt_context.tt_speaker_level = 0xffffffff;
	tt_context.tt_line_out_level = 600;
	tt_context.tt_hf_line_out = 0;
	tt_context.tt_scenario_line_out_volume = 0;
	tt_context.tt_scenario_speaker_volume = DEFAULT_SPEAKER_VOLUME;
	tt_context.tt_scenario_iPod_speaker_volume = DEFAULT_SPEAKER_VOLUME;
	#ifdef INCLUDE_TEST_KCONTROLS
	tt_context.tt_test_gain_type = -1;
	tt_context.tt_test_gain_value = -1;
	#endif
	tt_context.dapm_enabled = /* TT_DAPM_ENABLED */ 0;
	tt_context.has_5V_booster = fdt_get_ulong("/options/audio", "has-booster", 2);
	if (tt_context.has_5V_booster == 2) {
		/* for the moment fall back to checking hansfree has-bluetooth as this currently implies 5V booster */
		tt_context.has_5V_booster = fdt_get_ulong("/features", "has-bluetooth", 0);
	}
	tt_context.ops = ops;

	tt_hf_line_out = tt_context.tt_hf_line_out;

	if (ops->get_hpdetect_irq) {
		tt_hpdetect	= ops->hpdetect_callback(&tt_context);
		ret = request_irq(tt_context.ops->get_hpdetect_irq(&tt_context), tt_hpdetect_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hpdetect-irq",
				&tt_context);
		WARN_ON(ret);
	}

	if (ops->set_line_out_level) {
		ops->set_line_out_level(&tt_context);
	}

	if (ops->set_speaker_level) {
		ops->set_speaker_level(&tt_context);
	}

	return &tt_context;
}

void tt_scenario_unregister(struct tt_scenario_context *context)
{
	memset(&tt_context, 0, sizeof(tt_context));
	tt_context.ops=&default_scenario_ops;
}

EXPORT_SYMBOL(tomtom_add_nashville_controls);
