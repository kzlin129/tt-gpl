/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

#ifndef HALAUDIO_SETTINGS_H_
#define HALAUDIO_SETTINGS_H_

// ---- Include Files ----------------------------------------
// ---- Constants and Types ----------------------------------

/* --------------------------------------------------------------------------
** Hardware codec mappings.
*/
#include <linux/broadcom/halaudio.h>
#define HW_HANDSET_CODEC            HAL_AUDIO_CODEC0
#define HW_HEADSET_CODEC            HAL_AUDIO_CODEC0
#define HW_SPEAKERPHONE_CODEC       HAL_AUDIO_CODEC1
#define HW_HANDFREE_CODEC           HAL_AUDIO_CODEC2

#define HAL_AUDIO_HEADSET_MIC       HAL_AUDIO_CODEC0A_MIC
#define HAL_AUDIO_HANDSET_MIC       HAL_AUDIO_CODEC0B_MIC
#define HAL_AUDIO_SPKRPHONE_MIC     HAL_AUDIO_CODEC1A_MIC

#define HAL_AUDIO_HEADSET_SPKR      HAL_AUDIO_CODEC0A_SPKR
#define HAL_AUDIO_HANDSET_SPKR      HAL_AUDIO_CODEC0B_SPKR
#define HAL_AUDIO_SPKRPHONE_SPKR    HAL_AUDIO_CODEC1A_SPKR

#define HAL_AUDIO_HANDSET_SIDETONE  HAL_AUDIO_SIDETONE_1
#define HAL_AUDIO_HEADSET_SIDETONE  HAL_AUDIO_SIDETONE_0
#define HAL_AUDIO_SPKRPHONE_SIDETONE HAL_AUDIO_SIDETONE_0
#define HAL_AUDIO_SIDETONE          HAL_AUDIO_HANDSET_SIDETONE

/* this is really for legacy support (ALSA driver in brcm_arm.c)*/
#define HAL_AUDIO_EAR_SPKR          HAL_AUDIO_HEADSET_SPKR          // map to audio DAC
#define HAL_AUDIO_EAR_MIC           HAL_AUDIO_HEADSET_MIC           // map to audio ADC
#define HAL_AUDIO_AUX_SPKR          HAL_AUDIO_SPKRPHONE_SPKR        // map to voice DAC
#define HAL_AUDIO_AUX_MIC           HAL_AUDIO_SPKRPHONE_MIC         // map to voice ADC
#define HAL_AUDIO_SIDETONE          HAL_AUDIO_HANDSET_SIDETONE      // not used in BCM476X
#define HAL_AUDIO_HANDFREE_SPKR     HAL_AUDIO_CODEC2A_SPKR          // map to PCM-BlueTooth out
#define HAL_AUDIO_HANDFREE_MIC      HAL_AUDIO_CODEC2A_MIC           // map to PCM-BlueTooth in

/* --------------------------------------------------------------------------
** Audio Gains.
*/
/* Handset - mic */
#define HW_HANDSET_MIC_ANALOG_GAIN_DB                    0
#define HW_HANDSET_MIC_DIGITAL_HARDWARE_GAIN_DB          0
#define HW_HANDSET_MIC_DIGITAL_SOFTWARE_GAIN_DB          0

/* Handset - speaker */
#define HW_HANDSET_SPEAKER_ANALOG_GAIN_DB                0
#define HW_HANDSET_SPEAKER_DIGITAL_HARDWARE_GAIN_DB      0
#define HW_HANDSET_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB      0


/* Headset - mic */
#define HW_HEADSET_MIC_ANALOG_GAIN_DB                    0
#define HW_HEADSET_MIC_DIGITAL_HARDWARE_GAIN_DB          0
#define HW_HEADSET_MIC_DIGITAL_SOFTWARE_GAIN_DB          0

/* Headset - speaker */
#define HW_HEADSET_SPEAKER_ANALOG_GAIN_DB                0
#define HW_HEADSET_SPEAKER_DIGITAL_HARDWARE_GAIN_DB      0
#define HW_HEADSET_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB      0


/* Handsfree - mic */
#define HW_HANDSFREE_MIC_ANALOG_GAIN_DB                  0
#define HW_HANDSFREE_MIC_DIGITAL_HARDWARE_GAIN_DB        0
#define HW_HANDSFREE_MIC_DIGITAL_SOFTWARE_GAIN_DB        0

/* Handsfree - speaker */
#define HW_HANDSFREE_SPEAKER_ANALOG_GAIN_DB              0
#define HW_HANDSFREE_SPEAKER_DIGITAL_HARDWARE_GAIN_DB    0
#define HW_HANDSFREE_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB    0

#define HW_SIDETONE_GAIN_DB                              -24

/*
** Default gains for internal HW codec
*/

/* Handset - mic */
#define HW_HANDSET_MIC_INTERNAL_ANALOG_GAIN_DB                    HW_HANDSET_MIC_ANALOG_GAIN_DB
#define HW_HANDSET_MIC_INTERNAL_DIGITAL_HARDWARE_GAIN_DB          HW_HANDSET_MIC_DIGITAL_HARDWARE_GAIN_DB 
#define HW_HANDSET_MIC_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB          HW_HANDSET_MIC_DIGITAL_SOFTWARE_GAIN_DB 

/* Handset - speaker */
#define HW_HANDSET_SPEAKER_INTERNAL_ANALOG_GAIN_DB                HW_HANDSET_SPEAKER_ANALOG_GAIN_DB 
#define HW_HANDSET_SPEAKER_INTERNAL_DIGITAL_HARDWARE_GAIN_DB      HW_HANDSET_SPEAKER_DIGITAL_HARDWARE_GAIN_DB 
#define HW_HANDSET_SPEAKER_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB      HW_HANDSET_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB 


/* Headset - mic */
#define HW_HEADSET_MIC_INTERNAL_ANALOG_GAIN_DB                    HW_HEADSET_MIC_ANALOG_GAIN_DB 
#define HW_HEADSET_MIC_INTERNAL_DIGITAL_HARDWARE_GAIN_DB          HW_HEADSET_MIC_DIGITAL_HARDWARE_GAIN_DB
#define HW_HEADSET_MIC_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB          HW_HEADSET_MIC_DIGITAL_SOFTWARE_GAIN_DB  

/* Headset - speaker */
#define HW_HEADSET_SPEAKER_INTERNAL_ANALOG_GAIN_DB                HW_HEADSET_SPEAKER_ANALOG_GAIN_DB 
#define HW_HEADSET_SPEAKER_INTERNAL_DIGITAL_HARDWARE_GAIN_DB      HW_HEADSET_SPEAKER_DIGITAL_HARDWARE_GAIN_DB
#define HW_HEADSET_SPEAKER_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB      HW_HEADSET_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB


/* Handsfree - mic */
#define HW_HANDSFREE_MIC_INTERNAL_ANALOG_GAIN_DB                  HW_HANDSFREE_MIC_ANALOG_GAIN_DB
#define HW_HANDSFREE_MIC_INTERNAL_DIGITAL_HARDWARE_GAIN_DB        HW_HANDSFREE_MIC_DIGITAL_HARDWARE_GAIN_DB 
#define HW_HANDSFREE_MIC_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB        HW_HANDSFREE_MIC_DIGITAL_SOFTWARE_GAIN_DB 

/* Handsfree - speaker */
#define HW_HANDSFREE_SPEAKER_INTERNAL_ANALOG_GAIN_DB              HW_HANDSFREE_SPEAKER_ANALOG_GAIN_DB 
#define HW_HANDSFREE_SPEAKER_INTERNAL_DIGITAL_HARDWARE_GAIN_DB    HW_HANDSFREE_SPEAKER_DIGITAL_HARDWARE_GAIN_DB 
#define HW_HANDSFREE_SPEAKER_INTERNAL_DIGITAL_SOFTWARE_GAIN_DB    HW_HANDSFREE_SPEAKER_DIGITAL_SOFTWARE_GAIN_DB

#endif /* HALAUDIO_SETTINGS_H_ */
