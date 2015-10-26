/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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





/*
*
*****************************************************************************
*
*  halmixer_priv.h
*
*  PURPOSE:
*
*     This file contains the private structure definition of the HAL mixer.
*
*  NOTES:
*
*****************************************************************************/

#if !defined( HALMIXER_PRIV_H )
#define HALMIXER_PRIV_H

/* ---- Include Files ---------------------------------------------------- */
#include "halmixer_resamp.h"

/* ---- Constants and Types ---------------------------------------------- */
#define  HALMIXER_DEBUG 0
#if HALMIXER_DEBUG
#  define  HALMIXER_PRINTK(x)   printk x
#else
#  define  HALMIXER_PRINTK(x)
#endif

#define HALMIXER_ERROR(x) printk x

/* allow up to 20 ports to be registered with the HAL mixer */
/* expected list included:  6 HAL, 6 EPT, 1 EPT special (for echo cancelled audio) */
/* possible add-ons : ALSA and 2702 audio */
#define HALMIXER_MAX_PORT  20


/* mixer mode of operation */
typedef enum
{
   HAL_MIXER_IDLE = 0,
   HAL_MIXER_VOICE_MODE,
   HAL_MIXER_MUSIC_MODE,
   HAL_MIXER_HAL_DEBUG,

} HAL_MIXER_STATE;

typedef struct
{
   short * coefficient;
   short decimRatio;
   short interpRatio;
   short filterlen;
   short *resampbufp;

} halMixer_resampler_info;

typedef struct
{
   HAL_MIXER_PORT_INFO info;
   unsigned int inType;
   unsigned int outType;
   int attenuatedB;     /* local variable to keep track of preferred attenuation (src only) */
   int numInput;
   short * inputp;
   int firstInputIsr;
   int numInBytes;
   int numOutBytes;

} halMixer_port_info;

#define halMixer_port            HALMIXER_SWB_port
#define halMixer_Connection_List HALMIXER_SWB_Connection_List
typedef struct
{
   halMixer_port src;
   halMixer_port dst[1];

} halMixer_Connection;


/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */
int halMixer_ConnectPort_simplex( HALMIXER_Connection_t * connectp );
int halMixer_DisconnectPort_simplex( halMixer_Connection * conn, int src, int dst );
void halMixer_switchboardRemovePort( int port );
void halMixer_findPortNumBytes( int idx );



#endif /* HALMIXER_PRIV_H */
