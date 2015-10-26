/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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



/**
*
*  @file    halaudio_external_custom.c
*
*  @brief   HAL Audio External routines for Custom codec.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/proc_fs.h>          /* For /proc/halAudioExternalAK4642 */
#include <linux/sched.h>            /* For schedule_timeout */

#include <linux/broadcom/hw_cfg.h>  /* Hardware configuration of GPIOs */
#include <asm/arch/reg_gpio.h>      /* GPIO register accesses */
#include <linux/broadcom/halaudio.h>/* HALAUDIO API */

/* ---- Public Variables ------------------------------------------------- */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define EXTAUDIO_PROC_NAME          "halAudioExternalCustom"

static int halAudioExternalCustom_init(void);
static int halAudioExternalCustom_exit(void);
static int halAudioExternalCustom_GainSetAnalogHardware( HAL_AUDIO_BLOCK block, short db );
static int halAudioExternalCustom_GainSetDigitalHardware( HAL_AUDIO_BLOCK block, short db );
static int halAudioExternalCustom_GetCapabilities(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *capabilities);
static int halAudioExternalCustom_GetFrequencies(HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *frequencies);
static int halAudioExternalCustom_SetFrequency( unsigned short freq );
static void halAudioExternalCustom_CodecReset(int reset);
static void halAudioExternalCustom_PowerRegs(HAL_AUDIO_POWER_LEVEL level);
static int halAudioExternalCustom_CustomHardware(int cmd, void *data);
static int halAudioExternalCustom_GetCodecInfo(HALAUDIO_CODEC_INFO *codecInfo);

/*
** External Function table
*/
static const HAL_AUDIO_EXTERNAL_FNCS halAudioExternalFncs =
{
   halAudioExternalCustom_init,
   halAudioExternalCustom_exit,
   halAudioExternalCustom_GainSetAnalogHardware,
   halAudioExternalCustom_GainSetDigitalHardware,
   halAudioExternalCustom_GetCapabilities,
   halAudioExternalCustom_GetFrequencies,
   halAudioExternalCustom_SetFrequency,
   halAudioExternalCustom_CodecReset,
   halAudioExternalCustom_PowerRegs,
   halAudioExternalCustom_CustomHardware,
   halAudioExternalCustom_GetCodecInfo,
};

/* ---- Private Variables ------------------------------------------------ */


/* ---- Private Function Prototypes -------------------------------------- */
static int halAudioExternalCustomReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  halAudioExternalGetParams
*
*  @return  0 if not using external codec (no parameters),
*           1 if using external codec (fill in external parameters
*/
int halAudioExternalGetParams(const HAL_AUDIO_EXTERNAL_FNCS **paramp)
{
   {
      printk("USING Custom EXTERNAL CODEC  \n");
      /* External Codec Option selected for this board */
      *paramp = &halAudioExternalFncs;
      return( 1 );
   }
}

/***************************************************************************/
/**
*  External audio block int routine. Alloc resources, initialize hardware.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_init(void)
{
   int error = 0;

   /* Create debug proc entries */
   create_proc_read_entry( EXTAUDIO_PROC_NAME, 0, NULL, halAudioExternalCustomReadProc, NULL );
   return( error );
}

/***************************************************************************/
/**
*  External audio block exit routine. Frees resources.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_exit(void)
{
   remove_proc_entry( EXTAUDIO_PROC_NAME, NULL );
   return( 0 );
}

/***************************************************************************/
/**
*  halAudioExternalCustom_CodecReset - put the codec in reset or not
*
*  @return nothing
*/
static void halAudioExternalCustom_CodecReset(int reset)
{
   if (reset)
   {
      /* Put external codec in reset */
   }
   else
   {
      /* Take it out of reset */
   }
}

/***************************************************************************/
/**
*  halAudioExternalAK4642_PowerRegs - power hardware based on level
*    Sleep or Digital Only (no analog i/o) or Full
*
*  @return  nothing
*/
static void halAudioExternalCustom_PowerRegs(HAL_AUDIO_POWER_LEVEL level)
{
   if (level == HAL_AUDIO_POWER_DEEP_SLEEP)
   {
   }
   else
   {
   }
}

/***************************************************************************/
/**
*  Custom codec hardware configurations
*
*  @return  0 for success, -1 for unsupported
*/
int halAudioExternalCustom_CustomHardware( 
   int cmd,          /**< (in) custom hardware command */
   void *data        /**< (in) pointer to custom data structure */
)
{
   (void)id; (void) data;
   return -1; 
}

/***************************************************************************/
/**
*  HAL audio Get External Codec info.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalCustom_GetCodecInfo(HALAUDIO_CODEC_INFO *codecInfo)
{
   codecInfo->external = 1;
   strncpy( codecInfo->name, "custom", HALAUDIO_CODEC_NAME_LEN);
   codecInfo->name[HALAUDIO_CODEC_NAME_LEN-1] = 0;
   return 0;
}

/***************************************************************************/
/**
*  HAL audio gain set routine used to set hardware gains for the chosen
*  block.  Setting the gain setting to Mute/Sleep powers down the block,
*  setting the gain to a valid setting powers up the block.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_GainSetAnalogHardware(
   HAL_AUDIO_BLOCK block,        /**< (in) HAL block ID */
   short db                        /**< (in) gain in db */
)
{
   int error = 0;
   switch ( block )
   {
      case HAL_AUDIO_AUX_MIC:
      case HAL_AUDIO_EAR_MIC:
      {
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      {
      }
      break;


      case HAL_AUDIO_AUX_SPKR:
      {
      }
      break;

      case HAL_AUDIO_SIDETONE:
      {
      }
      break;

      default:
      {
         error = -1;  /* unrecognized block */
      }
      break;
   }
   return error;
}

/***************************************************************************/
/**
*  HAL Set Digital Hardware gain setting.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_GainSetDigitalHardware(
   HAL_AUDIO_BLOCK block,        /**< (in) HAL block ID */
   short db                        /**< (in) gain in db */
)
{
   switch ( block )
   {
      case HAL_AUDIO_EAR_MIC:
      case HAL_AUDIO_AUX_MIC:
      {
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      case HAL_AUDIO_AUX_SPKR:
      {
      }
      break;

      case HAL_AUDIO_SIDETONE:
      default:
      {
      }
      break;
   }
   return 0;
}

/***************************************************************************/
/**
*  HAL microphone select routine.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_MicSel(
   HAL_AUDIO_BLOCK block         /**< (in) HAL block ID */
)
{
   int error = 0;

   /*
   ** Select between ear and aux mic as we only have one input
   */
   switch ( block )
   {
      case HAL_AUDIO_EAR_MIC:
      {
      }
      break;
      case HAL_AUDIO_AUX_MIC:
      {
      }
      break;
      default:
      {
         error = -1;
      }
      break;
   }
   return error;
}

/***************************************************************************/
/**
*  HAL audio Set frequency
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_SetFrequency( unsigned short freq )
{
   int result = 0;

   /*
   ** Set sampling rate frequency of hardware
   */
   switch (freq)
   {
      case 8000:
      {
      }
      break;

      case 16000:
      {
      }
      break;

      case 22050:
      {
      }
      break;

      case 44100:
      {
      }
      break;

      default:
      {
         result = -1;
      }
      break;
   }
   return( result );
}

/***************************************************************************/
/**
*  HAL audio Get External Capabilities supported for each block.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_GetCapabilities(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *capabilities)
{
   /*
   ** Fill in audio capabilities of requested block
   */
   switch (block)
   {
      case HAL_AUDIO_EAR_MIC:
      case HAL_AUDIO_AUX_MIC:
      {
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      {
      }
      break;

      case HAL_AUDIO_AUX_SPKR:
      {
      }
      break;

      default:
      {
         return -1;
      }
      break;
   }
   return 0;
}

/***************************************************************************/
/**
*  HAL audio Get Frequencies supported for each mode.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalCustom_GetFrequencies(HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *frequencies)
{
   /*
   ** Fill in the frequencies supported for each mode
   */
   switch (mode)
   {
      case HAL_AUDIO_MODE_ALSA:
      {
         /* 8,16,22.05,44.1 kHz supported for playing out Audio.  We could support more if we wanted to */
         frequencies->numSettings = 4;
         frequencies->freqSetting[0] = 8000;
         frequencies->freqSetting[1] = 16000;
         frequencies->freqSetting[2] = 22050;
         frequencies->freqSetting[3] = 44100;
      }
      break;

      case HAL_AUDIO_MODE_POLYRINGER:
      {
         /* 22.05 kHz supported for playing out Polyringer. */
         frequencies->numSettings = 1;
         frequencies->freqSetting[0] = 22050;
      }
      break;

      case HAL_AUDIO_MODE_LDX:
      {
         /* Wideband (16kHz) sampling supported */
         frequencies->numSettings = 1;
         frequencies->freqSetting[0] = 16000;
      }
      break;

      case HAL_AUDIO_MODE_NONE:
      default:
      {
         /* No Frequencies supported */
         frequencies->numSettings = 0;
      }
      break;
   }
   return 0;
}

/***************************************************************************/
/**
*  Proc read callback function
*
*  @return  Number of characters to print
*/
static int halAudioExternalCustomReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;

   (void) start; (void) offset; (void) count; (void) data; /* avoid compiler warning */                         /* Cache volatile buffers before printing */

   len += sprintf( buf+len, "Custom External HAL Support!\n" );

   *eof = 1;
   return len+1;
}

/** @} */
