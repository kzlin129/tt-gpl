/* include/asm-arm/plat-tomtom/scenari.h
 *
 * Header for the ALSA controls that are required by Nashville.
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Guillaume Ballet <Guillaume.Ballet@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* TT scenarios */
#define TT_SCENARIO_NONE -1
#define TT_SCENARIO_AUDIO_INIT 0
#define TT_SCENARIO_AUDIO_OFF 1
#define TT_SCENARIO_ASR 2
#define TT_SCENARIO_VR 3
#define TT_SCENARIO_NOISE 4
#define TT_SCENARIO_iPod 5
#define TT_SCENARIO_SPEAKER 6
#define TT_SCENARIO_LINEOUT 7
#define TT_SCENARIO_FM 8
#define TT_SCENARIO_iPod_SPEAKER 9
#define TT_SCENARIO_iPod_LINEOUT 10
#define TT_SCENARIO_iPod_FM 11
#define TT_SCENARIO_HF_SPEAKER 12
#define TT_SCENARIO_HF_LINEOUT 13
#define TT_SCENARIO_HF_FM 14

/* TT scenario modes */
#define TT_MODE_INIT 0
#define TT_MODE_OFF 1
#define TT_MODE_SPP 2
#define TT_MODE_TTS 3
#define TT_MODE_MP3 4
#define TT_MODE_iPod 5
#define TT_MODE_HF 6
#define TT_MODE_ASR 7
#define TT_MODE_VR 8
#define TT_MODE_POI 9

//#define INCLUDE_TEST_KCONTROLS

struct tt_scenario_context;
struct tt_scenario_ops;

typedef int (*tt_scenario_callback_t)(struct tt_scenario_context *);

struct tt_scenario_ops {
	tt_scenario_callback_t	set_volume,
				set_volume_db,
				set_scenario,
				set_scenario_mode,
				#ifdef INCLUDE_TEST_KCONTROLS
				set_gain,
				#endif
				set_mic_gain,
				set_speaker_level,
				set_line_out_level,
				set_hf_line_out,
				unmute_amp,
				get_hpdetect_irq,
				hpdetect_callback;
};

struct tt_scenario_context {
	int			tt_volume;
	int			tt_volume_dB;
	int			tt_scenario;
	int			tt_current_scenario;
	int			tt_scenario_mode;
	int			tt_mic_gain;
	int			tt_hf_line_out;
	unsigned int tt_speaker_level;
	unsigned int tt_max_speaker_level;
	int			tt_line_out_level;
	int			tt_max_line_out;
	int			tt_scenario_line_out_volume;
	int			tt_scenario_speaker_volume;
	int			tt_scenario_iPod_speaker_volume;
	#ifdef INCLUDE_TEST_KCONTROLS
	int			tt_test_gain_type;
	int			tt_test_gain_value;
	#endif
	int			dapm_enabled;
	int			has_5V_booster;
	void			*data;
	struct tt_scenario_ops	*ops;
};

struct tt_scenario_context *tt_scenario_register(struct tt_scenario_ops *ops);
