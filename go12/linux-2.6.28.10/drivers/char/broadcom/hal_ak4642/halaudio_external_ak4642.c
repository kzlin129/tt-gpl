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
*  @file    halaudio_external_ak4642.c
*
*  @brief   HAL Audio External routines for AK4642 codec.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/proc_fs.h>          /* For /proc/halAudioExternalAK4642 */
#include <linux/sched.h>            /* For schedule_timeout */

#include <linux/broadcom/gpio.h>    /* GPIO register accesses */
#include <linux/broadcom/halaudio.h>/* HALAUDIO API */
#include <linux/broadcom/halaudio_settings.h> /* HALAUDIO settings */

#ifdef CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST
#include <linux/sysctl.h>           /* sysctl interface */
#include <linux/timer.h>            /* timer used to control test freq */
#include <asm/atomic.h>             /* atomic variables */
#endif

#include "ak4642_regs.h"            /* AK4642 registers */
#include "ext_codec_ak4642_i2c.h"   /* I2S driver for AK4642 */

/* ---- Public Variables ------------------------------------------------- */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define EXTAUDIO_PROC_NAME          "halAudioExternalAK4642"

static int halAudioExternalAK4642_init(void);
static int halAudioExternalAK4642_exit(void);
static int halAudioExternalAK4642_GainSetAnalogHardware( HAL_AUDIO_BLOCK block, short db );
static int halAudioExternalAK4642_GainSetDigitalHardware( HAL_AUDIO_BLOCK block, short db );
static int halAudioExternalAK4642_GetCapabilities(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *capabilities);
static int halAudioExternalAK4642_GetFrequencies(HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *frequencies);
static int halAudioExternalAK4642_SetFrequency( unsigned short freq );
static void halAudioExternalAK4642_CodecReset(int reset);
static void halAudioExternalAK4642_PowerRegs(HAL_AUDIO_POWER_LEVEL level);
static int halAudioExternalAK4642_CustomHardware(int cmd, void *data);
static int halAudioExternalAK4642_GetCodecInfo(HALAUDIO_CODEC_INFO *codecInfo);

/*
** External Function table
*/
static HAL_AUDIO_EXTERNAL_FNCS halAudioExternalFncs =
{
   halAudioExternalAK4642_init,
   halAudioExternalAK4642_exit,
   halAudioExternalAK4642_GainSetAnalogHardware,
   halAudioExternalAK4642_GainSetDigitalHardware,
   halAudioExternalAK4642_GetCapabilities,
   halAudioExternalAK4642_GetFrequencies,
   halAudioExternalAK4642_SetFrequency,
   halAudioExternalAK4642_CodecReset,
   halAudioExternalAK4642_PowerRegs,
   halAudioExternalAK4642_CustomHardware,
   halAudioExternalAK4642_GetCodecInfo,
};

/* ---- Private Variables ------------------------------------------------ */

/* Default register settings for AK4642 on power up */
static const unsigned char ext_codec_ak4642_DefaultRegs[AK4642_MAX_REGS] =
{
 0,0,1,0,2,0,0,0,0xE1,0xE1,0x18,0,0xE1,0x18,0x11,0x08,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Mask to indicate power register mapping */
static unsigned char ext_codec_ak4642_ShadowPowerRegsMask[AK4642_MAX_REGS] =
{
   (AK4642_REG_PM1_PMBP | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMDAC),
   (AK4642_REG_PM2_HPMTN | AK4642_REG_PM2_PMHPL | AK4642_REG_PM2_PMHPR),
   (AK4642_REG_SS1_SPPSN | AK4642_REG_SS1_DACS | AK4642_REG_SS1_PMMP),
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   ( AK4642_REG_MC4_DACH ),
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static unsigned char ext_codec_ak4642_ShadowRegs[AK4642_MAX_REGS];      /* Shadow registers for AK4642 registers */
static int halUpdateAK4642regs = 0;
static HAL_AUDIO_BLOCK halgMicActive = HAL_AUDIO_EAR_MIC;    /* default mic selected */
static HAL_AUDIO_BLOCK halgSpkActive = HAL_AUDIO_EAR_SPKR;    /* default mic selected */
static HAL_AUDIO_POWER_LEVEL gPowerLevel = HAL_AUDIO_POWER_DEEP_SLEEP;

/* ---- Private Function Prototypes -------------------------------------- */
static int halAudioExternalAK4642_MicSel( HAL_AUDIO_BLOCK block );
static int halAudioExternalAK4642ReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

#ifdef CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST
/* For AK4642 I2C stress test */
static void ak4642TestSysCtlInit( void );
static void ak4642TestSysCtlExit( void );
#endif

/* Function read registers of AK4642 (use shadow regs to speed access) */
static inline unsigned char halReadReg( AK4642_REG reg )
{
   return(ext_codec_ak4642_ShadowRegs[reg]);
}

/* Will update the shadow registers and write to i2c if flag enabled */
static void halSetRegVal( AK4642_REG reg, unsigned char val )
{
   if (ext_codec_ak4642_ShadowRegs[reg] != val)
   {
      /* Update the shadow register with new value but check on flag to write to i2c */
      ext_codec_ak4642_ShadowRegs[reg] = val;
      if (halUpdateAK4642regs)
      {
         /* Do not allow for power pins to rise when in interrupt only mode */
         if (gPowerLevel == HAL_AUDIO_POWER_INTERRUPTS_ONLY)
         {
            val = val & ~(ext_codec_ak4642_ShadowPowerRegsMask[reg]);
         }
         ext_codec_ak4642_i2c_write( reg, val );
      }
   }
}

static inline void halSetRegBits( AK4642_REG reg, unsigned char val )
{
   halSetRegVal( reg, halReadReg(reg) | val );
}

static inline void halClearRegBits( AK4642_REG reg, unsigned char val )
{
   halSetRegVal( reg, halReadReg(reg) & ~val );
}

/* Intended to force a value regardless of current shadow register value.
 * Will not save to shadow register.
 */
static inline void halForceSetRegVal( AK4642_REG reg, unsigned char val )
{
   ext_codec_ak4642_i2c_write( reg, val );
}

static inline void halForceSetRegBits( AK4642_REG reg, unsigned char val )
{
   halForceSetRegVal( reg, halReadReg(reg) | val );
}

static inline void halForceClearRegBits( AK4642_REG reg, unsigned char val )
{
   halForceSetRegVal( reg, halReadReg(reg) & ~val );
}

static inline void halSetPowerRegVal( AK4642_REG reg, unsigned char val )
{
   /* Only allow values that are in the shadow registers */
   unsigned char allowVal = val & halReadReg(reg);
   if (allowVal)
   {
      allowVal |= ext_codec_ak4642_i2c_read(reg);
      ext_codec_ak4642_i2c_write( reg, allowVal );
   }
}

static inline void halSpkrPowerUp( void )
{
   /* Do not allow power up on digital mode.  Sleep mode will not write to I2C but will track changes */
   if (gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY)
   {
      halSetPowerRegVal( AK4642_REG_SS1, AK4642_REG_SS1_DACS );
      halSetPowerRegVal( AK4642_REG_MC4, AK4642_REG_MC4_DACH );

      /* We power up the handset and headset speakers, the DAC, and the sidetone (Beep) */
      halSetPowerRegVal( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );

      halSetPowerRegVal( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL );
      halSetPowerRegVal( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR );

      /* Set the speaker and headphone mute during power up */
      halSetPowerRegVal( AK4642_REG_PM2, AK4642_REG_PM2_HPMTN );
      halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
   }
}

static inline void halSpkrPowerDown( void )
{
   /* Set the speaker to power save mode and mute the headphone speaker */
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
   halForceClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_HPMTN );

   /* Verify if the headphone or the ear speaker is powered or we skip waiting for the fall time from Vdd to ground */
   if ( (halReadReg(AK4642_REG_PM1) & AK4642_REG_PM1_PMSPK)    ||
         (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPL)   ||
         (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPR))
   {
      /* Delay after entering power save mode.  Reduces pop noise.  */
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(50);
   }
   /* We now power down both headset and handset speakers as well as remove paths to DAC */
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );
   halForceClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );
   halForceClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL | AK4642_REG_PM2_PMHPR );
   halForceClearRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );
}

static inline void halMicPowerUp( void )
{
   if ( gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY )
   {
      /* Power up mic power for biasing */
      halSetPowerRegVal( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );

      /* Power up mic amp to work-around AK4642 silicon problem
       * where the speaker path is also powered off if the mic is off.
       */
      halAudioExternalAK4642_MicSel( halgMicActive );
   }
}

static inline void halMicPowerDown( void )
{
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );

   /* Clear PMADL and PMADR bits */
   halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
   halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
}

static inline void halLoadShadowRegs( void )
{
   int i;
   /* Decrementing counter will reduce pop noise as it happens to follow correct order */
   for (i = 0x1f; i >= 0; i--)
   {
      if (ext_codec_ak4642_ShadowRegs[i] != ext_codec_ak4642_DefaultRegs[i])
      {
         unsigned char value = ext_codec_ak4642_ShadowRegs[i] & ~(ext_codec_ak4642_ShadowPowerRegsMask[i]);
         ext_codec_ak4642_i2c_write( i, value );
//         printk("REG:0x%x VAL:0x%x 0x%x\n", i, ext_codec_ak4642_ShadowRegs[i], ext_codec_ak4642_i2c_read( i ) );
      }
      else
      {
//         printk("DEFREG:0x%x VAL:0x%x 0x%x\n", i, ext_codec_ak4642_ShadowRegs[i], ext_codec_ak4642_i2c_read( i ) );
      }
   }
}

/***************************************************************************/
/**
*  halAudioExternalAK4642_PowerRegs - power hardware based on level
*    Sleep or Digital Only (no analog i/o) or Full
*
*  @return  nothing
*/
static void halAudioExternalAK4642_PowerRegs(HAL_AUDIO_POWER_LEVEL level)
{
   gPowerLevel = level;

   if (level == HAL_AUDIO_POWER_DEEP_SLEEP)
   {
      /* Ensure proper order of power down */
      halSpkrPowerDown();
      halMicPowerDown();

      /* Do not allow the I2C to be written from here on */
      halUpdateAK4642regs = 0;
   }
   else
   {
      /* Allow the I2C to be written from here on */
      halUpdateAK4642regs = 1;

      /* Ensure proper order of power up occurs before loading shadow registers */
      halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
      halLoadShadowRegs();

      /* Ensure proper order of power up */
      halMicPowerUp();
      halSpkrPowerUp();
   }
}

/***************************************************************************/
/**
*  halAudioExternalAK4642_CustomHardware - custom codec hardware
*
*  @return  0 for success, -1 for unsupported
*/
int halAudioExternalAK4642_CustomHardware(
   int cmd,          /**< (in) custom hardware command */
   void *data        /**< (in) pointer to custom data structure */
)
{
   /*
    *  The AK4642 may be shared with another processor and as such the
    *  host needs to be able to do various things with it to facilitate the
    *  sharing.  This function has been used as a suitable "catch-all" place
    *  to provide the functionality required.
    */

   HAL_AUDIO_CUSTOM_HARDWARE_MODE mode = (HAL_AUDIO_CUSTOM_HARDWARE_MODE)cmd;
   int result = 0;
   (void) data;

//   printk( "halAudioExternalAK4642_CustomHardware: parameter = %d\n", mode );

   switch (mode)
   {
      case HAL_AUDIO_CUSTOM_HARDWARE_CODEC_PWR_UP:
      {
         halAudioExternalAK4642_CodecReset(0);
      }
      break;

      case HAL_AUDIO_CUSTOM_HARDWARE_CODEC_PWR_DWN:
      {
         halAudioExternalAK4642_CodecReset(1);
      }
      break;

      case HAL_AUDIO_CUSTOM_HARDWARE_CODEC_RESTORE:
      {
         halLoadShadowRegs();
      }
      break;

      default:
      {
//         printk( "halAudioExternalAK4642_CustomHardware: got illegal parameter %d\n", mode );
         result = -1;
      }
   }

   return result;
}

/***************************************************************************/
/**
*  halAudioExternalAK4642_GetCodecInfo - Get codec info
*
*  @return  0 for success, -1 for unsupported
*/
int halAudioExternalAK4642_GetCodecInfo(HALAUDIO_CODEC_INFO *codecInfo)
{
   codecInfo->external = 1;
   strncpy( codecInfo->name, "AK4642", HALAUDIO_CODEC_NAME_LEN);
   codecInfo->name[HALAUDIO_CODEC_NAME_LEN-1] = 0;
   codecInfo->sampleSize = 2;

   /* the frequency of the external codec is depending on run time configuration */
   //codecIndo->frequency = 16000;

   return 0;
}

static short calculateInputDB( unsigned char gain )
{
   short db;

   /* rounding done */
   db = ((((int)gain - 145)*3*2)/8 + 1)/2;

   return(db);
}

static unsigned char calculateInputGain( short db )
{
   if (db > 36)
   {
      /* Maximum is +36 dB */
      db = 36;
   }
   if (db < -54)
   {
      /* Minimum is -54 dB */
      /* Mute */
      return(0x00);
   }
   /* rounding done */
   return( (unsigned char)(((db*8*2)/3 + 1)/2 + 145) );
}

static short calculateOutputDB( unsigned char vol )
{
   short db;

   db = 12 - ((int)vol/2) ;

   return(db);
}

static unsigned char calculateOutputGain( short db )
{
   if (db > 12)
   {
      /* Maximum is +12 dB */
      db = 12;
   }
   if (db < -115)
   {
      /* Minimum is -115 dB */
      /* Mute */
      return(0xFF);
   }
   return( (unsigned char)((12 - db) * 2) );
}

/* ---- Functions -------------------------------------------------------- */

/* Store value of external pinValue so halAudioExternalGetParams can be called more than once */
static int pinValueDetected = -1;
/***************************************************************************/
/**
*  halAudioExternalGetParams
*
*  @return  0 if no parameters, 1 if parameters
*/
int halAudioExternalGetParams(const HAL_AUDIO_EXTERNAL_FNCS **paramp)
{
   int pinValueDetectedFirstTime = 0;

   if (pinValueDetected == -1)
   {
      /* Determine whether we should use the internal or external audio codec */
      gpio_direction_input( HW_GPIO_CODEC_PDN_PIN );
      pinValueDetected = gpio_get_value( HW_GPIO_CODEC_PDN_PIN );
      pinValueDetectedFirstTime = 1;
   }

   /* If pull-down is present then external codec is configured on board */
   if ( pinValueDetected == 0 )
   {
      if (pinValueDetectedFirstTime)
      {
         printk("USING AK4642 EXTERNAL CODEC  \n");
      }
      /* External Codec Option selected for this board */
      *paramp = &halAudioExternalFncs;

#ifdef HW_GPIO_AUDIO_PA_PWR_PIN
      if (pinValueDetectedFirstTime)
      {
         gpio_direction_output( HW_GPIO_AUDIO_PA_PWR_PIN, 1 );
      }
#endif

      return( 1 );
   }
   else
   {
      if (pinValueDetectedFirstTime)
      {
         printk("NO AK4642, USING INTERNAL CODEC  \n");
      }
      /* Internal Codec is used - don't changed the audioFncs */
      return( 0 );
   }
}

/***************************************************************************/
/**
*  External audio block int routine. Alloc resources, initialize hardware.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalAK4642_init(void)
{
   int error = 0;

   /* Configure external codec GPIO as an output */
   gpio_direction_output(HW_GPIO_CODEC_PDN_PIN, 0);

   /* Put external codec in reset for at least tPD=150ns */
   halAudioExternalAK4642_CodecReset(1);

   /* Take it out of reset */
   halAudioExternalAK4642_CodecReset(0);

   /* Initialze the shadow registers to the default values they come up in */
   memcpy( ext_codec_ak4642_ShadowRegs, ext_codec_ak4642_DefaultRegs, sizeof(ext_codec_ak4642_DefaultRegs)/sizeof(ext_codec_ak4642_DefaultRegs[0]) );

   /* Initialize the external codec i2c interface, AK4642 must be initialized first */
   error = ext_codec_ak4642_ic_init();
   if (error != 0)
   {
      printk( KERN_ERR "halAudioExternal: Failed to initialize AK4642\n" );
      return( error );
   }

   /* Put external codec in reset until enabled */
   gpio_set_value( HW_GPIO_CODEC_PDN_PIN, 0 );

   /* Configure all codec registers */

   /* Set PMVCM bit of reg 00 */
   halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMVCM );
   /* Set PMPLL bit, M/S of reg 01 */
   halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMPLL | AK4642_REG_PM2_MS );
   /* Clear default mic gain to 0dB reg 02 */
   halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
   /* Set clocking options in Mode Control 1 register */
   halSetRegVal( AK4642_REG_MC1, 0x6B );
   /* 16 kHz sampling */
   halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_16KHZ );
   /* Set the Lch and Rch Input Digital Volume to 0 dB */
   halSetRegVal( AK4642_REG_IVL, 0x91 );
   halSetRegVal( AK4642_REG_IVR, 0x91 );
   /* Independent gain control for Lch and Rch */
   halClearRegBits( AK4642_REG_MC3, AK4642_REG_DVOLC );
   /* Configure for differential microphone input */
   halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_MDIF1 | AK4642_REG_PM3_MDIF2 );

   /* Create debug proc entries */
   create_proc_read_entry( EXTAUDIO_PROC_NAME, 0, NULL, halAudioExternalAK4642ReadProc, NULL );

#ifdef CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST
   ak4642TestSysCtlInit();
#endif

   return( error );
}

/***************************************************************************/
/**
*  External audio block exit routine. Frees resources.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalAK4642_exit(void)
{
#ifdef CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST
   ak4642TestSysCtlExit();
#endif

   remove_proc_entry( EXTAUDIO_PROC_NAME, NULL );
   ext_codec_ak4642_ic_release();
   return( 0 );
}

/***************************************************************************/
/**
*  halAudioExternalCustom_CodecReset - put the codec in reset or not
*
*  @return nothing
*/
static void halAudioExternalAK4642_CodecReset(int reset)
{
   if (reset)
   {
      /* Put external codec in reset */
      if ( gpio_get_value(HW_GPIO_CODEC_PDN_PIN) != 0 )
      {
         gpio_set_value( HW_GPIO_CODEC_PDN_PIN, 0 );
         set_current_state(TASK_INTERRUPTIBLE);
         schedule_timeout(1);
      }
   }
   else
   {
      /* Take it out of reset */
      if ( gpio_get_value(HW_GPIO_CODEC_PDN_PIN) == 0 )
      {
         gpio_set_value( HW_GPIO_CODEC_PDN_PIN, 1 );
         set_current_state(TASK_INTERRUPTIBLE);
         schedule_timeout(1);
      }
   }
}

/***************************************************************************/
/**
*  HAL audio gain set routine used to set hardware gains for the chosen
*  block.  Setting the gain setting to Mute/Sleep powers down the block,
*  setting the gain to a valid setting powers up the block.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalAK4642_GainSetAnalogHardware(
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
         unsigned char mgain1 = 0;
         unsigned char mgain0 = 0;

         switch( db )
         {
            case 0:
            {
               /* Set MIC gain to 0 dB */
               mgain1 = 0;
               mgain0 = 0;
            }
            break;
            case 20:
            {
               /* Set MIC gain to 20 dB */
               mgain1 = 0;
               mgain0 = 1;
            }
            break;
            case 26:
            {
               /* Set MIC gain to 26 dB */
               mgain1 = 1;
               mgain0 = 0;
            }
            break;
            case 32:
            {
               /* Set MIC gain to 32 dB */
               mgain1 = 1;
               mgain0 = 1;
            }
            break;

            default:
            {
               if (db > HAL_AUDIO_GAIN_MUTE)
               {
                  /* Invalid gain setting */
                  error = -2;
               }
               else
               {
                  /* Power down setting */
               }
            }
            break;
         }
         if ((!error))
         {
            if (db > HAL_AUDIO_GAIN_MUTE)
            {
               /* Write mgain bits to registers */
               if (mgain0)
               {
                  halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
               }
               else
               {
                  halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
               }
               if (mgain1)
               {
                  halSetRegBits( AK4642_REG_SS2, AK4642_REG_SS2_MGAIN1 );
               }
               else
               {
                  halClearRegBits( AK4642_REG_SS2, AK4642_REG_SS2_MGAIN1 );
               }

               halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );
               /* Write ivol setting  */
               halAudioExternalAK4642_MicSel( block );
            }
            else
            {
               if(   halReadReg(AK4642_REG_SS1) & AK4642_REG_SS1_PMMP &&
                     halgMicActive == block
                 )
               {
                  /* Power down the block */
                  halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );
#if 0
                  /* Warning: Do not affect the PMADL and PMADR bits, because
                   * doing so seems to also bring down the speaker path. This
                   * appears to be a silicon issue with the AK4642. The work-
                   * around is when audio is fully powered, either PMADL or
                   * PMADR bits will be set even though the mic is set to
                   * sleep.
                   */
                  /* Clear PMADL and PMADR bits */
                  halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
                  halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
#endif
               }
            }
         }
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      {
         unsigned char spkg = 0;

         switch( db )
         {
            case 4:
            {
               /* Set Speaker gain to 4.43 dB */
               spkg = AK4642_REG_SS2_SPKG_4_43DB;
            }
            break;
            case 6:
            {
               /* Set Speaker gain to 6.43 dB */
               spkg = AK4642_REG_SS2_SPKG_6_43DB;
            }
            break;
            case 10:
            {
               /* Set Speaker gain to 10.65 dB */
               spkg = AK4642_REG_SS2_SPKG_10_65DB;
            }
            break;
            case 12:
            {
               /* Set Speaker gain to 12.65 dB */
               spkg = AK4642_REG_SS2_SPKG_12_65DB;
            }
            break;
            default:
            {
               if (db > HAL_AUDIO_GAIN_MUTE)
               {
                  /* Invalid gain setting */
                  error = -2;
               }
               else
               {
                  /* Power down setting */
               }
            }
            break;
         }

         if (!error)
         {
            if (db > HAL_AUDIO_GAIN_MUTE)
            {
               /* Set SPKG1-0 bits */
               halSetRegVal( AK4642_REG_SS2, (halReadReg( AK4642_REG_SS2 ) & ~AK4642_REG_SS2_SPKG_MASK) | spkg );
               /* Connect the speaker to the DAC */
               halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );

               halgSpkActive = block;

               /*If speaker is unpowered, follow proper power up procedures*/
               if(!(halReadReg(AK4642_REG_SS1) & AK4642_REG_PM1_PMSPK) )
               {
                  halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );

                  /* We power up the handset and headset speakers, the DAC, and the sidetone (Beep) */
                  halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );
                  halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
               }
            }
            else
            {
               /* Power save mode of speaker amp */
               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN);
               /*Delay to allow fall in output */
               if (  gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY &&
                     (halReadReg(AK4642_REG_PM1) & AK4642_REG_PM1_PMSPK) != 0 &&
                     halgSpkActive == HAL_AUDIO_EAR_SPKR )
               {
                  set_current_state(TASK_INTERRUPTIBLE);
                  schedule_timeout(50);
               }
               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );
               if (db == HAL_AUDIO_GAIN_SLEEP)
               {
                  /* Clear PMDAC bit of reg 00 (turn off DAC if aux_spk also not powered) */
                  if ( (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPR) == 0 )
                  {
                     /* Clear PMDAC bit of reg 00 */
                     /* Clear PMSPK bit of reg 00 */
                     halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK );
                  }
                  else
                  {
                     /* Just Clear PMSPK bit of reg 00 */
                     halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMSPK );
                  }
               }
            }
         }
      }
      break;


      case HAL_AUDIO_AUX_SPKR:
      {
         switch( db )
         {
            case 0:
            case 3:
            {
               /* Connect DAC output to headphone amp */
               halSetRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );

               if (db)
               {
                  /* Set HPG bit */
                  /* Set Headphone gain to +3.6 dB */
                  halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_HPG );
               }
               else
               {
                  /* Clear HPG bit */
                  /* Set Headphone gain to 0 dB */
                  halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_HPG );
               }

               /* Power up the block */
               /* Set PMDAC bit of reg 00 */
               if (gPowerLevel == HAL_AUDIO_POWER_RESUME_ALL)
               {
                  halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC );

                  if (halAudio_getMode() == HAL_AUDIO_MODE_ALSA)
                  {
                     /* Set PMHPL and PMHPR bit of reg 01 */
                     halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL );
                  }
                  else
                  {
                     /* Clear PMHPL bit of reg 01 */
                     halClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL );

                     /* Set PMHPR bit of reg 01 */
                     halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR );
                  }

                  /* Set HPMTN bit of reg 01 (Normal - unmute) */
                  halSetRegBits(AK4642_REG_PM2, AK4642_REG_PM2_HPMTN);
               }

               halgSpkActive = block;
            }
            break;

            case HAL_AUDIO_GAIN_MUTE:
            case HAL_AUDIO_GAIN_SLEEP:
            {
               /* Power down the block */
               /* Clear HPMTN bit of reg 01 (MUTE) */
               halClearRegBits(AK4642_REG_PM2, AK4642_REG_PM2_HPMTN);
               /*Delay to allow fall in output */
               if (  gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY  &&
                     (halReadReg(AK4642_REG_PM2) & (AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL)) != 0 &&
                      halgSpkActive == HAL_AUDIO_AUX_SPKR
                  )
               {
                  set_current_state(TASK_INTERRUPTIBLE);
                  schedule_timeout(50);
               }
               /* Clear PMHPR bit of reg 01 (turn off headphone block) */
               halClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL );
               if ( (halReadReg(AK4642_REG_PM1) & (AK4642_REG_PM1_PMSPK)) == 0 )
               {
                  /* Clear PMDAC bit of reg 00 (turn off DAC if ear_spk also not powered) */
                  halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC );
               }
               /* Disconnect DAC output to headphone amp */
               halClearRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );
            }
            break;

            default:
            {
               /* Invalid gain setting */
               error = -3;
            }
            break;
         }
      }
      break;

      case HAL_AUDIO_SIDETONE:
      {
         if (db <= HAL_AUDIO_GAIN_MUTE)
         {
            /* Turn off sidetone */
            /* Clear BEEPS bit of reg 02 */
            halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_BEEPS );
         }
         else
         {
            /* Turn on sidetone */
            halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_BEEPS );
         }
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
static int halAudioExternalAK4642_GainSetDigitalHardware(
   HAL_AUDIO_BLOCK block,        /**< (in) HAL block ID */
   short db                        /**< (in) gain in db */
)
{
   switch ( block )
   {
      case HAL_AUDIO_EAR_MIC:
      case HAL_AUDIO_AUX_MIC:
      {
         /* Only set mic digital gain if block is actively selected. This is
          * to work around the fact that there is a single digital gain block for
          * both primary and aux microphones */
         if ( block == halgMicActive )
         {
            halSetRegVal( AK4642_REG_IVL, calculateInputGain(db) );
            halSetRegVal( AK4642_REG_IVR, calculateInputGain(db) );
         }
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      case HAL_AUDIO_AUX_SPKR:
      {
         /* Only set spk digital gain if block is actively selected. This is
          * to work around the fact that there is a single digital gain block for
          * both primary and aux speakers */
         if ( block == halgSpkActive )
         {
            halSetRegVal( AK4642_REG_DVL, calculateOutputGain(db) );
            halSetRegVal( AK4642_REG_DVR, calculateOutputGain(db) );
         }
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
static int halAudioExternalAK4642_MicSel(
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
         /* Select channel 1 PMADL=1 PMADR=0 */
         halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
         halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
      }
      break;
      case HAL_AUDIO_AUX_MIC:
      {
         /* Select channel 2 PMADL=0 PMADR=1 */
         halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
         halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
      }
      break;
      default:
      {
         error = -1;
      }
      break;
   }
   if ( !error )
   {
      halgMicActive = block;
   }
   return error;
}

/***************************************************************************/
/**
*  HAL audio Set frequency
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalAK4642_SetFrequency( unsigned short freq )
{
   int result = 0;

   /*
   ** Set sampling rate frequency of hardware
   */
   switch (freq)
   {
      case 8000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_8KHZ );
      }
      break;

      case 16000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_16KHZ );
      }
      break;

      case 22050:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_22_05KHZ );
      }
      break;

      case 24000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_24KHZ );
      }
      break;

      case 32000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_32KHZ );
      }
      break;

      case 44100:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_44_1KHZ );
      }
      break;

      case 48000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_48KHZ );
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
static int halAudioExternalAK4642_GetCapabilities(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *capabilities)
{
   /*
   ** Fill in audio capabilities of requested block
   */
   switch (block)
   {
      case HAL_AUDIO_EAR_MIC:
      case HAL_AUDIO_AUX_MIC:
      {
         unsigned char mgain1;
         unsigned char mgain0;

         if (halReadReg(AK4642_REG_SS1) & AK4642_REG_SS1_MGAIN0)
         {
            mgain0 = 1;
         }
         else
         {
            mgain0 = 0;
         }
         if (halReadReg(AK4642_REG_SS2) & AK4642_REG_SS2_MGAIN1)
         {
            mgain1 = 1;
         }
         else
         {
            mgain1 = 0;
         }
         if (mgain0 == 0)
         {
            if (mgain1 == 0)
            {
               capabilities->analog.currentDB = 0;
            }
            else
            {
               capabilities->analog.currentDB = 26;
            }
         }
         else
         {
            if (mgain1 == 0)
            {
               capabilities->analog.currentDB = 20;
            }
            else
            {
               capabilities->analog.currentDB = 32;
            }
         }
         capabilities->analog.maxDB = 32;
         capabilities->analog.minDB = 0;
         capabilities->analog.rangeType = HAL_AUDIO_RANGETYPE_LIST;
         capabilities->analog.range.list.numSettings = 4;
         capabilities->analog.range.list.dbSetting[0] = 0;
         capabilities->analog.range.list.dbSetting[1] = 20;
         capabilities->analog.range.list.dbSetting[2] = 26;
         capabilities->analog.range.list.dbSetting[3] = 32;

         capabilities->digital.currentDB = calculateInputDB( halReadReg(AK4642_REG_IVL) );
         capabilities->digital.maxDB = 36;
         capabilities->digital.minDB = -54;
         capabilities->digital.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->digital.range.fixedStepSize = 1;
      }
      break;

      case HAL_AUDIO_EAR_SPKR:
      {
         if (halReadReg( AK4642_REG_PM1 ) & AK4642_REG_PM1_PMSPK)
         {
            /* Speaker is powered, determine gain setting */
            switch (halReadReg( AK4642_REG_SS2 ) & AK4642_REG_SS2_SPKG_MASK)
            {
               case AK4642_REG_SS2_SPKG_4_43DB:
               {
                  capabilities->analog.currentDB = 4;
               }
               break;
               case AK4642_REG_SS2_SPKG_6_43DB:
               {
                  capabilities->analog.currentDB = 6;
               }
               break;
               case AK4642_REG_SS2_SPKG_10_65DB:
               {
                  capabilities->analog.currentDB = 10;
               }
               break;
               case AK4642_REG_SS2_SPKG_12_65DB:
               {
                  capabilities->analog.currentDB = 12;
               }
               break;
               default:
               {
                  return -2;
               }
               break;
            }
         }
         else
         {
            /* Speaker is off/muted */
            capabilities->analog.currentDB = HAL_AUDIO_GAIN_SLEEP;
         }
         capabilities->analog.maxDB = 12;
         capabilities->analog.minDB = 4;
         capabilities->analog.rangeType = HAL_AUDIO_RANGETYPE_LIST;
         capabilities->analog.range.list.numSettings = 4;
         capabilities->analog.range.list.dbSetting[0] = 4;
         capabilities->analog.range.list.dbSetting[1] = 6;
         capabilities->analog.range.list.dbSetting[2] = 10;
         capabilities->analog.range.list.dbSetting[3] = 12;

         capabilities->digital.currentDB = calculateOutputDB( halReadReg(AK4642_REG_DVL) );
         capabilities->digital.maxDB = 12;
         capabilities->digital.minDB = -115;
         capabilities->digital.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->digital.range.fixedStepSize = 1;
      }
      break;

      case HAL_AUDIO_AUX_SPKR:
      {
         if (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPR)
         {
            /* Headphone Speaker is powered, determine gain setting */
            if (halReadReg(AK4642_REG_PM3) & AK4642_REG_PM3_HPG)
            {
               capabilities->analog.currentDB = 3;
            }
            else
            {
               capabilities->analog.currentDB = 0;
            }
         }
         else
         {
            /* Headphone Speaker is off/muted */
            capabilities->analog.currentDB = HAL_AUDIO_GAIN_SLEEP;
         }
         capabilities->analog.maxDB = 3;
         capabilities->analog.minDB = 0;
         capabilities->analog.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->analog.range.fixedStepSize = 3;

         capabilities->digital.currentDB = calculateOutputDB( halReadReg(AK4642_REG_DVL) );
         capabilities->digital.maxDB = 12;
         capabilities->digital.minDB = -115;
         capabilities->digital.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->digital.range.fixedStepSize = 1;
      }
      break;

      case HAL_AUDIO_SIDETONE:
      {
         if (halReadReg( AK4642_REG_SS1 ) & AK4642_REG_SS1_BEEPS )
         {
            capabilities->analog.currentDB = -12;
         }
         else
         {
            capabilities->analog.currentDB = HAL_AUDIO_GAIN_MUTE;
         }
         capabilities->analog.maxDB = -12;
         capabilities->analog.minDB = -12;
         capabilities->analog.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->analog.range.fixedStepSize = 1;

         capabilities->digital.currentDB = 0;
         capabilities->digital.maxDB = 0;
         capabilities->digital.minDB = 0;
         capabilities->digital.rangeType = HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE;
         capabilities->digital.range.fixedStepSize = 1;
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
static int halAudioExternalAK4642_GetFrequencies(HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *frequencies)
{
   /*
   ** Fill in the frequencies supported for each mode
   */
   switch (mode)
   {
      case HAL_AUDIO_MODE_ALSA:
      {
         /* 8,16,22.05, 24, 32, 44.1, 48 kHz supported for playing out Audio.
          * We could support more if we wanted to */
         frequencies->numSettings = 7;
         frequencies->freqSetting[0] = 8000;
         frequencies->freqSetting[1] = 16000;
         frequencies->freqSetting[2] = 22050;
         frequencies->freqSetting[3] = 24000;
         frequencies->freqSetting[4] = 32000;
         frequencies->freqSetting[5] = 44100;
         frequencies->freqSetting[6] = 48000;
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
static int halAudioExternalAK4642ReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   int i;

   (void) start; (void) offset; (void) count; (void) data; /* avoid compiler warning */                         /* Cache volatile buffers before printing */

   len += sprintf( buf+len, "AK4642 SHADOW REGISTERS:\n" );
   for (i = 0; i < 16; i++)
   {
      len += sprintf( buf+len, "0x%2.2x ", halReadReg(i) );
   }
   len += sprintf( buf+len, "\n" );
   for (i = 16; i < 32; i++)
   {
      len += sprintf( buf+len, "0x%2.2x ", halReadReg(i) );
   }
   len += sprintf( buf+len, "\n" );

   len += sprintf( buf+len, "AK4642 REGISTERS:\n" );
   if (gpio_get_value( HW_GPIO_CODEC_PDN_PIN ))
   {
      for (i = 0; i < 16; i++)
      {
         len += sprintf( buf+len, "0x%2.2x ", ext_codec_ak4642_i2c_read(i) );
      }
      len += sprintf( buf+len, "\n" );
      for (i = 16; i < 32; i++)
      {
         len += sprintf( buf+len, "0x%2.2x ", ext_codec_ak4642_i2c_read(i) );
      }
      len += sprintf( buf+len, "\n" );
   }
   else
   {
      len += sprintf( buf+len, "AK4642 in reset\n" );
   }

   *eof = 1;
   return len+1;
}

#ifdef CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST

/***************************************************************************/
/**
*  Debug sysctl interface data variables used to stress test
*  reading and writing AK4642 register over I2C.
*/

static int ak4642_read_errors;         /* Number of read errors */
static int ak4642_write_errors;        /* Number of write errors */
static int ak4642_test_count;          /* Number of r/w tests executed */
static int ak4642_rw_per_tick;         /* Number of JIFFIES between tests */

/* Copy of registers used for read/write compare */
static unsigned int ak4642_regs_copy[AK4642_MAX_REGS];

static struct timer_list ak4642_timer; /* Timer control variables */
static atomic_t ak4642_stop_timer;

static DECLARE_COMPLETION( gThreadDone );
static int quitNow;
static struct semaphore gI2cTestSem;

static int i2cTestThread( void *arg );

static int proc_do_rwI2CPerTick(ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos);

#define CTL_TABLE_INT(i,varStr,var) \
      ctl_name: i,\
      procname: varStr,\
      data: &var,\
      maxlen: sizeof(int),\
      mode: 0644,\
      proc_handler: &proc_dointvec,

#define CTL_TABLE_INT_ARRAY(i,varStr,var) \
      ctl_name: i,\
      procname: varStr,\
      data: var,\
      maxlen: sizeof(var),\
      mode: 0644,\
      proc_handler: &proc_dointvec,

static struct ctl_table gSysCtlAk4642[] =
{
   { CTL_TABLE_INT(101, "rd_errs", ak4642_read_errors) },
   { CTL_TABLE_INT(102, "wr_errs", ak4642_write_errors) },
   { CTL_TABLE_INT(103, "test_count", ak4642_test_count) },
   { CTL_TABLE_INT_ARRAY(104, "regs", ak4642_regs_copy) },
   {
      ctl_name: 110,
      procname: "rwtest_per_tick",
      data: &ak4642_rw_per_tick,
      maxlen: sizeof(int),
      mode: 0644,
      proc_handler: &proc_do_rwI2CPerTick,
   },
   {}
};
static struct ctl_table gSysCtl[] =
{
   /* Hard-code a sysctl ID of 250 */
   { 250, "ak4642_i2c_test", NULL, 0, 0555, gSysCtlAk4642, 0, 0, 0, 0, 0 },
   {}
};
static struct ctl_table_header *gSysCtlHeader;

/***************************************************************************/
/**
*  Timer expiry routine used to read and write registers over I2C. The timer
*  expires every ak4642_rw_per_tick jiffies.
*/
static void ak4642RWTimedOut( unsigned long data )
{
   (void) data;

   /* Signal thread to run */
   up( &gI2cTestSem );

   /* Re-trigger timer */
   if ( !atomic_read( &ak4642_stop_timer ))
   {
      ak4642_timer.expires = jiffies + ak4642_rw_per_tick;
      add_timer( &ak4642_timer );
   }
}

/***************************************************************************/
/**
*  Cleanup debug timer
*/
static void ak4642DelTimer( void )
{
   if ( !atomic_read( &ak4642_stop_timer ))
   {
      atomic_set( &ak4642_stop_timer, 1 );
      del_timer( &ak4642_timer );   /* Force removal of timer */
   }
}

/***************************************************************************/
/**
*  Sysctl method for starting and stopping I2C test
*/
static int proc_do_rwI2CPerTick(ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos)
{
   int rc;

   /* Process integer operation */
   rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

   if ( write )
   {
      /* Configure timer to read AK4642 registers over I2C */

      if ( ak4642_rw_per_tick == 0 )
      {
         ak4642DelTimer();
      }
      else
      {
         /* Launch timer if not already running */
         if ( atomic_read( &ak4642_stop_timer ))
         {
            int i;

            /* Take AK4642 out of reset */
            halAudioExternalAK4642_CodecReset( 0 );

            /* Get copy of registers as basis for test comparison */
            for ( i = 0; i < AK4642_MAX_REGS; i++ )
            {
               ak4642_regs_copy[i] = ext_codec_ak4642_i2c_read( i );
            }

            init_timer( &ak4642_timer );
            ak4642_timer.function = ak4642RWTimedOut;
            ak4642_timer.data = 0;
            ak4642_timer.expires = jiffies + ak4642_rw_per_tick;
            atomic_set( &ak4642_stop_timer, 0 );
            add_timer( &ak4642_timer );
         }
      }
   }
   return rc;
}

/***************************************************************************/
/**
*  I2C test thread
*
*  @return  0 on success, otherwise failure
*/
static int i2cTestThread( void *arg )
{
   (void)arg;                       /* Unused */

   daemonize( "i2ctest" );

   while ( 1 )
   {
      unsigned int regval;
      int i;
      int err;

      err = down_interruptible( &gI2cTestSem );
      if ( err || quitNow )
      {
         /* Task is interrupted by signal or quitting */
         break;
      }

      ak4642_test_count++;

      /* Read all AK4642 registers and check for correct data */
      for ( i = 0; i < AK4642_MAX_REGS; i++ )
      {
         regval = ext_codec_ak4642_i2c_read( i );
         if ( regval != ak4642_regs_copy[i] )
         {
            ak4642_read_errors++;
            //printk( KERN_ERR "AK4642 I2C Test: reg=%i read 0x%x expect 0x%x\n", i, regval, ak4642_regs_copy[i] );
         }
      }

#define AK4642_TEST_REG       0x09     /* Lch Input Vol control */
#define AK4642_TEST_VALUE     0x5a

      /* Test write */
      ext_codec_ak4642_i2c_write( AK4642_TEST_REG, AK4642_TEST_VALUE );
      regval = ext_codec_ak4642_i2c_read( AK4642_TEST_REG );
      if ( regval != AK4642_TEST_VALUE )
      {
         ak4642_write_errors++;
      }
      /* Restore register */
      ext_codec_ak4642_i2c_write( AK4642_TEST_REG, ak4642_regs_copy[AK4642_TEST_REG] );
   }

   complete_and_exit( &gThreadDone, 0 );
   return 0;
}

/***************************************************************************/
/**
*  Initialize SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void ak4642TestSysCtlInit( void )
{
   int result;

   quitNow = 0;

   gSysCtlHeader = register_sysctl_table( gSysCtl );

   init_completion( &gThreadDone );
   sema_init( &gI2cTestSem, 0 );

   result = kernel_thread( i2cTestThread, NULL, CLONE_KERNEL );
   if ( result < 0 )
   {
      printk( KERN_ERR "AK4642: Error failed to create test thread\n" );
   }

   /* Timer initially stopped */
   atomic_set( &ak4642_stop_timer, 1 );
}

/***************************************************************************/
/**
*  Cleanup SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void ak4642TestSysCtlExit( void )
{
   ak4642DelTimer();

   quitNow = 1;
   up( &gI2cTestSem );
   wait_for_completion( &gThreadDone );

   /* Put AK4642 in reset */
   halAudioExternalAK4642_CodecReset( 1 );

   unregister_sysctl_table( gSysCtlHeader );
}

#endif   /* CONFIG_BCM_HALAUDIO_AK4642_I2C_TEST */

/** @} */
