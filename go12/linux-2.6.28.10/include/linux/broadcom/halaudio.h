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
*  @file    halaudio.h
*
*****************************************************************************/
#if !defined( HALAUDIO_H )
#define HALAUDIO_H

/* ---- Include Files ---------------------------------------------------- */

/*
 * For the special purpose of building WIN32 based RPC client, the IOCTL
 * macros are not available.
 */
#if (!defined( BOS_OS_Win32 ) || !BOS_OS_Win32)
#include <linux/ioctl.h>
#endif /* BOS_OS_Win32 */

#if defined( __KERNEL__ )
#include <linux/broadcom/csx.h>
#endif   /* __KERNEL__ */

/* ---- Constants and Types ---------------------------------------------- */

/* Hardware codec channel numbers
 *
 * The enumerations must be consecutive because they may be used as
 * array indices into mapping tables. 
 */
typedef enum
{
   HAL_AUDIO_CODEC0 = 0,
   HAL_AUDIO_CODEC1,
   HAL_AUDIO_CODEC2,
   HAL_AUDIO_CODEC3,
   HAL_AUDIO_CODEC4,
   HAL_AUDIO_CODEC5,
   HAL_AUDIO_CODEC6,
   HAL_AUDIO_CODEC7,
   HAL_AUDIO_CODEC8,
   HAL_AUDIO_CODEC9,

   HAL_AUDIO_CODEC_MAX_NUM

} HAL_AUDIO_CODEC;

/* Legacy define */
#define HAL_AUDIO_NUM_CODEC   HAL_AUDIO_CODEC_MAX_NUM

/**
* Supported audio blocks
* The following definition are gain block, each gain block is associated with one of the codecs
* defined in the HAL_AUDIO_CODEC enum list
*/

typedef unsigned int HAL_AUDIO_BLOCK;

/**
*  Hal Audio block IDs are formed from the following bit fields: 
*     codec    - Physical codec ID (HAL_AUDIO_CODEC)
*     sidetone - Sidetone functinality
*     hwsel    - HW mux selection (HAL_AUDIO_HWSEL)
*     dir      - Direction (HAL_AUDIO_DIR)
*
*  | codec (b15-b8) | hwsel (b7-b4) | sidetone (b3) | dir (b2-b0) |
*/
#define HAL_AUDIO_BLOCK_CODEC_BMASK             0xFF00
#define HAL_AUDIO_BLOCK_HWSEL_BMASK             0x00F0
#define HAL_AUDIO_BLOCK_SIDETONE_BMASK          0x0008
#define HAL_AUDIO_BLOCK_DIR_BMASK               0x0007

#define HAL_AUDIO_BLOCK_CODEC_BSHIFT            8 
#define HAL_AUDIO_BLOCK_HWSEL_BSHIFT            4
#define HAL_AUDIO_BLOCK_SIDETONE_BSHIFT         3
#define HAL_AUDIO_BLOCK_DIR_BSHIFT              0

#define HAL_AUDIO_CREATE_BLOCK( codec, hwsel, sidetone, dir ) ((( (codec) << HAL_AUDIO_BLOCK_CODEC_BSHIFT ) & HAL_AUDIO_BLOCK_CODEC_BMASK ) | (( (hwsel) << HAL_AUDIO_BLOCK_HWSEL_BSHIFT ) & HAL_AUDIO_BLOCK_HWSEL_BMASK ) | (( (sidetone) << HAL_AUDIO_BLOCK_SIDETONE_BSHIFT ) & HAL_AUDIO_BLOCK_SIDETONE_BMASK )  | (( (dir) << HAL_AUDIO_BLOCK_DIR_BSHIFT ) & HAL_AUDIO_BLOCK_DIR_BMASK ))

/* Helpers */
#define HAL_AUDIO_BLOCK_ID( codec, hwsel, dir )    HAL_AUDIO_CREATE_BLOCK( codec, hwsel, 0, dir )
#define HAL_AUDIO_SIDETONE_ID( codec )             HAL_AUDIO_CREATE_BLOCK( codec, 0, 1, 0 )
#define HAL_AUDIO_BLOCK_GET_CODEC( block )         (( block & HAL_AUDIO_BLOCK_CODEC_BMASK ) >> HAL_AUDIO_BLOCK_CODEC_BSHIFT )
#define HAL_AUDIO_BLOCK_GET_HWSEL( block )         (( block & HAL_AUDIO_BLOCK_HWSEL_BMASK ) >> HAL_AUDIO_BLOCK_HWSEL_BSHIFT )
#define HAL_AUDIO_BLOCK_GET_DIR( block )           (( block & HAL_AUDIO_BLOCK_DIR_BMASK   ) >> HAL_AUDIO_BLOCK_DIR_BSHIFT   )
#define HAL_AUDIO_BLOCK_IS_SIDETONE( block )       ( block & HAL_AUDIO_BLOCK_SIDETONE_BMASK )

/* Pre-defined Hal Audio block IDs */
#define HAL_AUDIO_CODEC0A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC0, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC0B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC0, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC0A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC0, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC0B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC0, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_0                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC0 )

#define HAL_AUDIO_CODEC1A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC1, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC1B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC1, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC1A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC1, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC1B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC1, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_1                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC1 )

#define HAL_AUDIO_CODEC2A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC2, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC2B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC2, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC2A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC2, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC2B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC2, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_2                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC2 )

#define HAL_AUDIO_CODEC3A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC3, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC3B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC3, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC3A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC3, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC3B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC3, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_3                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC3 )

#define HAL_AUDIO_CODEC4A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC4, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC4B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC4, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC4A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC4, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC4B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC4, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_4                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC4 )

#define HAL_AUDIO_CODEC5A_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC5, HAL_AUDIO_HWSEL_A, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC5B_SPKR                     HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC5, HAL_AUDIO_HWSEL_B, HAL_AUDIO_DAC_DIR )
#define HAL_AUDIO_CODEC5A_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC5, HAL_AUDIO_HWSEL_A, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_CODEC5B_MIC                      HAL_AUDIO_BLOCK_ID( HAL_AUDIO_CODEC5, HAL_AUDIO_HWSEL_B, HAL_AUDIO_ADC_DIR )
#define HAL_AUDIO_SIDETONE_5                       HAL_AUDIO_SIDETONE_ID( HAL_AUDIO_CODEC5 )

#define HAL_AUDIO_NUM_BLOCKS (6*5)

typedef enum
{
   HAL_AUDIO_POWER_DEEP_SLEEP,      /* Deep sleep */
   HAL_AUDIO_POWER_DIGITAL_ONLY,    /* Digital blocks enabled only */
   HAL_AUDIO_POWER_RESUME_ALL,      /* Full analog and digital power */
} HAL_AUDIO_POWER_LEVEL;

typedef enum
{
   HAL_AUDIO_MODE_NONE,
   HAL_AUDIO_MODE_LDX,
   HAL_AUDIO_MODE_ALSA,
#if defined(CONFIG_PLAT_BCM476X)
   HAL_AUDIO_MODE_HANDFREE,
#endif
   HAL_AUDIO_MODE_POLYRINGER
} HAL_AUDIO_MODE;

/* Data flow direction 
 *
 * The enumerations must be consecutive because they may be used as
 * array indices into mapping tables. 
 */
typedef enum
{
   HAL_AUDIO_ADC_DIR = 0,  /* microphone to ADC direction */
   HAL_AUDIO_DAC_DIR,      /* DAC to speaker direction */
   HAL_AUDIO_BOTH_DIR,     /* both directions */
}
HAL_AUDIO_DIR;

/* Hardware multiplex select positions.
 *
 * The enumerations must be consecutive because they may be used as
 * array indices into mapping tables. 
 */
typedef enum
{
   HAL_AUDIO_HWSEL_NONE = -1,    /* no path */
   HAL_AUDIO_HWSEL_ALL  = 0,     /* all paths */
   HAL_AUDIO_HWSEL_A,
   HAL_AUDIO_HWSEL_B,
   HAL_AUDIO_HWSEL_C,
   HAL_AUDIO_HWSEL_MAX_NUM
}
HAL_AUDIO_HWSEL;

/**
* Special power down and mute gain mappings. These definitions
* are used in conjunction with halAudioSetGain to power down and mute
* analog blocks, respectively. The value of -1000 is arbitrarily choosen,
* as this value is essentially equal to -infinity in dB.
*/
#define  HAL_AUDIO_GAIN_MUTE           -1000
#define  HAL_AUDIO_GAIN_SLEEP          (HAL_AUDIO_GAIN_MUTE-1)

typedef enum
{
   HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE,
   HAL_AUDIO_RANGETYPE_LIST

} HAL_AUDIO_RANGETYPE;

/**
* Commands that can be sent to the customHardware function.
*/
typedef enum
{
   HAL_AUDIO_CUSTOM_HARDWARE_CODEC_PWR_UP,
   HAL_AUDIO_CUSTOM_HARDWARE_CODEC_PWR_DWN,
   HAL_AUDIO_CUSTOM_HARDWARE_CODEC_RESTORE,
   HAL_AUDIO_CUSTOM_HARDWARE_CODEC_SET_REG1,

} HAL_AUDIO_CUSTOM_HARDWARE_MODE;

#define HAL_AUDIO_MAX_NUM_DB_SETTINGS 128

typedef union
{
   int fixedStepSize;
   struct
   {
      int numSettings;
      int dbSetting[HAL_AUDIO_MAX_NUM_DB_SETTINGS];
   } list;

} HAL_AUDIO_RANGE;

typedef struct
{
   int currentDB;
   int minDB;
   int maxDB;
   HAL_AUDIO_RANGETYPE rangeType;
   HAL_AUDIO_RANGE     range;

} HAL_AUDIO_DB;

typedef struct
{
   HAL_AUDIO_DB analog;
   HAL_AUDIO_DB digital;

} HAL_AUDIO_CAPABILITIES;

#define HAL_AUDIO_MAX_NUM_FREQ_SETTINGS 10
typedef struct
{
   unsigned int numSettings;
   /* Frequency list sorted from low to high rates */
   unsigned int freqSetting[HAL_AUDIO_MAX_NUM_FREQ_SETTINGS];
} HAL_AUDIO_FREQUENCIES;

/* Write and read equalizer coefficients structure */
#define HALAUDIO_EQU_COEFS_MAX_NUM       128
typedef struct HALAUDIO_EQU_COEFS
{
   int dac[HALAUDIO_EQU_COEFS_MAX_NUM];
   int adc[HALAUDIO_EQU_COEFS_MAX_NUM];

} HALAUDIO_EQU_COEFS;

typedef struct HALAUDIO_EQU_ORDER
{
   int dac;          /* DAC equalizer filter order, 0 to disable */
   int adc;          /* ADC equalizer filter order, 0 to disable */

} HALAUDIO_EQU_ORDER;

typedef struct
{
   int dac;          /* DAC direction equalizer gain */
   int adc;          /* ADC direction equalizer gain */

} HALAUDIO_EQU_GAIN;

/* Equalizer parameters structure */
typedef struct HALAUDIO_EQU_PARMS
{
   HAL_AUDIO_CODEC codec;        /* Select codec */
   HALAUDIO_EQU_ORDER order;     /* Filter order configuration. Platform specific
                                    and may not be supported on all platforms */
   HALAUDIO_EQU_GAIN gain;       /* Platform specific gain setting */
   HALAUDIO_EQU_COEFS coefs;     /* Platform specific coefficients */
   
} HALAUDIO_EQU_PARMS;

/* Get Codec Info structure*/
#define HALAUDIO_CODEC_NAME_LEN 32
typedef struct HALAUDIO_CODEC_INFO
{
   unsigned int codecNum;  /* input, target codec number */
   /* return information starts here */
   int external;                      /* 1 = external codec, 0 = internal codec */
   int sampleSize;                    /* width (8 / 16 bits) of each samples in units of bytes */
   int frequency;                     /* sampling frequency of this codec */
   char name[HALAUDIO_CODEC_NAME_LEN]; /* name identifiying codec */

} HALAUDIO_CODEC_INFO;

/* The following definitions are deprecated */
#if 1
typedef enum
{
   HAL_AUDIO_SPKRSOURCE_EAR,
   HAL_AUDIO_SPKRSOURCE_AUX,
   HAL_AUDIO_SPKRSOURCE_HANDFREE_DEVICE,
   HAL_AUDIO_SPKRSOURCE_SIDETONE
} HAL_AUDIO_SPKRSOURCE;

typedef enum
{
   HAL_AUDIO_MICSOURCE_EAR,
   HAL_AUDIO_MICSOURCE_AUX,
   HAL_AUDIO_MICSOURCE_HANDFREE_DEVICE
} HAL_AUDIO_MICSOURCE;
#endif

#define HALAUDIO_MAGIC   'A'

#define HALAUDIO_CMD_FIRST               0x70

#define HALAUDIO_CMD_GET_CONTROL         0x70
#define HALAUDIO_CMD_RELEASE_CONTROL     0x71
#define HALAUDIO_CMD_GET_AUDIO_GAIN      0x72
#define HALAUDIO_CMD_SET_AUDIO_GAIN      0x73
#define HALAUDIO_CMD_ALTER_AUDIO_GAIN    0x74

typedef struct
{
   HAL_AUDIO_BLOCK block;
   int analogGain;
   int digitalGain;

} HALAUDIO_AUDIO_GAIN;

typedef struct
{
   HAL_AUDIO_BLOCK block;
   int bDigital;
   int direction;
   int resultingGain;
} HALAUDIO_AUDIO_ALTER;

#define HALAUDIO_CMD_BLOCK_ENABLE        0x75
#define HALAUDIO_CMD_BLOCK_DISABLE       0x76
#define HALAUDIO_CMD_BLOCK_QUERY         0x77
#define HALAUDIO_CMD_SET_POWER           0x78
#define HALAUDIO_CMD_GET_POWER           0x79
#define HALAUDIO_CMD_SUPERUSER           0x7A
#define HALAUDIO_CMD_WIDEBAND_CHECK      0x7B
#define HALAUDIO_CMD_CUSTOM_HARDWARE     0x7C
#define HALAUDIO_CMD_SET_CHANNELS        0x7D

typedef struct
{
   int cmd;
   void *data;       /* Pointer to optional user data */

} HALAUDIO_CUSTOM_HARDWARE;

#define HALAUDIO_CMD_WRITE_FORMAT        0x91

/* Formats used with halAudio write */
typedef enum
{
   HALAUDIO_FORMAT_S16_BE,       /* signed 16-bit big-endian */
   HALAUDIO_FORMAT_S16_LE,       /* signed 16-bit little-endian */
   HALAUDIO_FORMAT_ULAW,         /* 8-bit G.711 u-law */
   HALAUDIO_FORMAT_ALAW,         /* 8-bit G.711 a-law */

   HALAUDIO_FORMAT_MAX           /* last entry */

} HALAUDIO_WRITE_FORMAT;

#define HALAUDIO_CMD_CONFIG_FREQ         0x92

typedef struct
{
   HAL_AUDIO_MODE mode;    /* audio mode */ 
   int freq;               /* frequency selection */

} HALAUDIO_CONFIG_FREQ;

#define HALAUDIO_CMD_SET_CODEC           0x96
#define HALAUDIO_CMD_GET_CODEC_INFO      0x97
#define HALAUDIO_CMD_GET_EQU_PARMS       0x98
#define HALAUDIO_CMD_SET_EQU_PARMS       0x99

#define HALAUDIO_CMD_MIC_SELECT          0x9a

/** @{ */

/*
 * For the special purpose of building WIN32 based RPC client, the IOCTL
 * macros are not available.
 */
#if (!defined( BOS_OS_Win32 ) || !BOS_OS_Win32)
#define HALAUDIO_IOCTL_GET_CONTROL   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_GET_CONTROL, int )
#define HALAUDIO_IOCTL_RELEASE_CONTROL   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_RELEASE_CONTROL, int )
#define HALAUDIO_IOCTL_GET_AUDIO_GAIN   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_GET_AUDIO_GAIN, HALAUDIO_AUDIO_GAIN )
#define HALAUDIO_IOCTL_SET_AUDIO_GAIN   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SET_AUDIO_GAIN, HALAUDIO_AUDIO_GAIN )
#define HALAUDIO_IOCTL_ALTER_AUDIO_GAIN    _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_ALTER_AUDIO_GAIN, HALAUDIO_AUDIO_ALTER )
#define HALAUDIO_IOCTL_BLOCK_ENABLE  _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_BLOCK_ENABLE, int )
#define HALAUDIO_IOCTL_BLOCK_DISABLE _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_BLOCK_DISABLE, int )
#define HALAUDIO_IOCTL_BLOCK_QUERY   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_BLOCK_QUERY, int )
#define HALAUDIO_IOCTL_SET_POWER     _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SET_POWER, int )
#define HALAUDIO_IOCTL_GET_POWER     _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_GET_POWER, int )
#define HALAUDIO_IOCTL_SUPERUSER     _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SUPERUSER, int )
#define HALAUDIO_IOCTL_WIDEBAND_CHECK _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_WIDEBAND_CHECK, int )
#define HALAUDIO_IOCTL_CUSTOM_HARDWARE _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_CUSTOM_HARDWARE, HALAUDIO_CUSTOM_HARDWARE )
#define HALAUDIO_IOCTL_SET_CHANNELS   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SET_CHANNELS, int )
#define HALAUDIO_IOCTL_WRITE_FORMAT  _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_WRITE_FORMAT, int )
#define HALAUDIO_IOCTL_CONFIG_FREQ   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_CONFIG_FREQ, HALAUDIO_CONFIG_FREQ )
#define HALAUDIO_IOCTL_SET_CODEC          _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SET_CODEC, int )
#define HALAUDIO_IOCTL_GET_CODEC_INFO _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_GET_CODEC_INFO, HALAUDIO_CODEC_INFO )
#define HALAUDIO_IOCTL_GET_EQU_PARMS   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_GET_EQU_PARMS, HALAUDIO_EQU_PARMS )
#define HALAUDIO_IOCTL_SET_EQU_PARMS   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_SET_EQU_PARMS, HALAUDIO_EQU_PARMS )

#define HALAUDIO_IOCTL_MIC_SELECT   _IOWR( HALAUDIO_MAGIC, HALAUDIO_CMD_MIC_SELECT, HAL_AUDIO_BLOCK )

#endif /* BOS_OS_Win32 */

#if defined( __KERNEL__ )

/* ---- Kernel Constants, Types, and Definitions ------------------------- */


typedef void (*HAL_AUDIO_ISR)(void);
typedef int (*HAL_AUDIO_EXIT)(void);
typedef int (*HAL_AUDIO_ENABLE)(HAL_AUDIO_POWER_LEVEL level);

typedef int (*HAL_AUDIO_GAINSET)(HAL_AUDIO_BLOCK block, short db );
typedef int (*HAL_AUDIO_MICSEL)(HAL_AUDIO_BLOCK);
typedef int (*HAL_AUDIO_SELECTPOWER)(HAL_AUDIO_POWER_LEVEL level);

#if defined(CONFIG_PLAT_BCM476X)
typedef void (*HAL_AUDIO_CBACK)(unsigned short numSamples, short *audioBuffer, void *cb_param);
typedef int (*HAL_AUDIO_WRITE)(unsigned short numSamples, unsigned short numChannels, HAL_AUDIO_CODEC codecNum, short *audioBuffer, HAL_AUDIO_CBACK callback, void *cb_param);
typedef int (*HAL_AUDIO_READ)(unsigned short numSamples, unsigned short numChannels, HAL_AUDIO_CODEC codecNum, short *audioBuffer, HAL_AUDIO_CBACK callback, void *cb_param);
#else
typedef void (*HAL_AUDIO_CBACK)(unsigned short numSamples, short *audioBuffer);
typedef int (*HAL_AUDIO_WRITE)(unsigned short numSamples, unsigned short numChannels, HAL_AUDIO_CODEC codecNum, short *audioBuffer, HAL_AUDIO_CBACK callback);
typedef int (*HAL_AUDIO_READ)(unsigned short numSamples, unsigned short numChannels, HAL_AUDIO_CODEC codecNum, short *audioBuffer, HAL_AUDIO_CBACK callback);
#endif

typedef int (*HAL_AUDIO_SELECTMODE)(HAL_AUDIO_MODE mode, unsigned short freq);
typedef HAL_AUDIO_MODE (*HAL_AUDIO_GETMODE)( void );

typedef int (*HAL_AUDIO_GETCAPABILITIES)(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *capabilities);
typedef int (*HAL_AUDIO_GETFREQUENCIES)(HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *frequencies);
typedef int (*HAL_AUDIO_QUERY_POWER)(HAL_AUDIO_POWER_LEVEL *level, int *downRequestPending );

typedef int (*HAL_AUDIO_SETFREQUENCY)( unsigned short freq );
typedef void (*HAL_AUDIO_RESET)(int reset);
typedef void (*HAL_AUDIO_POWERREGS)(HAL_AUDIO_POWER_LEVEL level);
typedef int (*HAL_AUDIO_EXT_INIT)(void);
typedef int (*HAL_AUDIO_EXT_EXIT)(void);

typedef int (*HAL_AUDIO_CUSTOM_HARDWARE)(int, void *);
typedef int (*HAL_AUDIO_EQU_PARMS_FP)(HALAUDIO_EQU_PARMS *);
typedef int (*HAL_AUDIO_GET_CODEC_INFO)(HALAUDIO_CODEC_INFO *codecInfo);

typedef int (*HAL_AUDIO_REGISTER_CSX_IO_POINT)( HAL_AUDIO_CODEC codec, HAL_AUDIO_DIR dir, CSX_IO_POINT_FNCS *fncp, void *data );

typedef struct
{
   HAL_AUDIO_EXT_INIT init;
   HAL_AUDIO_EXT_EXIT exit;

   HAL_AUDIO_GAINSET gainSetAnalogHardware;  /* Set analog hardware gain (block, db) */
   HAL_AUDIO_GAINSET gainSetDigitalHardware; /* Set digital hardware gain (block, db) */
   HAL_AUDIO_GETCAPABILITIES getCapabilities;/* Get Audio capabilities of a block */
   HAL_AUDIO_GETFREQUENCIES getFrequencies;  /* Get Frequencies supported in a mode */

   HAL_AUDIO_SETFREQUENCY setFrequency;      /* Set Frequency supported */
   HAL_AUDIO_RESET reset;                    /* Set reset state */
   HAL_AUDIO_POWERREGS powerRegs;            /* Set power regs based on level */

   HAL_AUDIO_CUSTOM_HARDWARE customHardware; /* Custom hardware configuration */

   HAL_AUDIO_GET_CODEC_INFO getCodecInfo;    /* Get Codec Info */

} HAL_AUDIO_EXTERNAL_FNCS;

typedef int (*HAL_AUDIO_INIT)(const HAL_AUDIO_EXTERNAL_FNCS *audioFncp);

struct hal_audio_pcm_info
{
   /* Current status */
   int               samp_freq;        /* sampling frequency in Hz */
   int               frame_size_bytes; /* frame size in bytes */
   HAL_AUDIO_HWSEL   micsel;           /* microphone mux selection */
   HAL_AUDIO_HWSEL   spksel;           /* speaker mux selection */

   /* Physical characteristics */
   int               sample_width;     /* sample width in bytes */
   int               microphones;      /* number of multiplexed mics */
   int               speakers;         /* number of multiplexed speakers */

   /* Other information */
   char name[32];                      /* name string */
};

struct hal_audio_if_info
{
   int               num_pcm_chans;    /* Number of PCM channels on interface */
   int               is_external;      /* 1 if interface is external, 0 if internal */
   char              name[32];         /* name string */
};

/* HAL Audio interface operations */

typedef void (*HAL_AUDIO_IF_ISR_CB)( void );
typedef int  (*HAL_AUDIO_IF_INIT)( HAL_AUDIO_IF_ISR_CB isrcb );
typedef int  (*HAL_AUDIO_IF_EXIT)( void );
typedef int  (*HAL_AUDIO_IF_ENABLE)( void );
typedef int  (*HAL_AUDIO_IF_PREPARE)( void );
typedef int  (*HAL_AUDIO_IF_DISABLE)( void );
typedef int  (*HAL_AUDIO_IF_ANA_POWERDN)( int powerdn );
typedef int  (*HAL_AUDIO_IF_INFO)( struct hal_audio_if_info *info );

/* HAL Audio PCM operations */

typedef int  (*HAL_AUDIO_PCM_SETFREQ)( int pcm, int freqhz );
typedef int  (*HAL_AUDIO_PCM_ANA_GAINSET)( int pcm, int db, HAL_AUDIO_DIR dir, HAL_AUDIO_HWSEL hwsel );
typedef int  (*HAL_AUDIO_PCM_DIG_GAINSET)( int pcm, int db, HAL_AUDIO_DIR dir );
typedef int  (*HAL_AUDIO_PCM_SIDETONE_ENABLE)( int pcm, int enable );
typedef int  (*HAL_AUDIO_PCM_SIDETONE_GAINSET)( int pcm, int db );
typedef int  (*HAL_AUDIO_PCM_MIC_SELECT)( int pcm, HAL_AUDIO_HWSEL hwsel );
typedef int  (*HAL_AUDIO_PCM_SPEAKER_SELECT)( int pcm, HAL_AUDIO_HWSEL hwsel );
typedef int  (*HAL_AUDIO_PCM_EQU_WRITE_COEFS)( int pcm, HAL_AUDIO_DIR dir, int *data, int length );
typedef int  (*HAL_AUDIO_PCM_EQU_GAINSET)( int pcm, HAL_AUDIO_DIR dir, int db );
typedef int  (*HAL_AUDIO_PCM_EQU_ENABLE)( int pcm, HAL_AUDIO_DIR dir, int enable );
typedef void (*HAL_AUDIO_PCM_IORW_CB)( int bytes, void *data );
typedef int  (*HAL_AUDIO_PCM_READ)( int pcm, int bytes, char *audiobuf, HAL_AUDIO_PCM_IORW_CB cb, void *data );
typedef int  (*HAL_AUDIO_PCM_WRITE)( int pcm, int bytes, int channels, char *audiobuf, HAL_AUDIO_PCM_IORW_CB cb, void *data );
typedef int  (*HAL_AUDIO_PCM_INFO)( int pcm, struct hal_audio_pcm_info *pcminfo );
typedef int  (*HAL_AUDIO_PCM_GAIN_CAPABILITIES)( int pcm, HAL_AUDIO_DIR dir, HAL_AUDIO_HWSEL hwsel, HAL_AUDIO_CAPABILITIES *capabilities);

struct hal_audio_pcm_ops
{
   HAL_AUDIO_PCM_SETFREQ            set_freq;         /* Set sampling frequency */
   HAL_AUDIO_PCM_ANA_GAINSET        ana_gain;         /* Set analog (PGA) gains */
   HAL_AUDIO_PCM_DIG_GAINSET        dig_gain;         /* Set digital gains */
   HAL_AUDIO_PCM_SIDETONE_ENABLE    sidetone_enable;  /* Enable sidetone */
   HAL_AUDIO_PCM_SIDETONE_GAINSET   sidetone_gain;    /* Set sidetone gain */
   HAL_AUDIO_PCM_MIC_SELECT         mic_sel;          /* Select multiplexed microphone path */
   HAL_AUDIO_PCM_SPEAKER_SELECT     spk_sel;          /* Select multiplexed speaker path(s) */
   HAL_AUDIO_PCM_EQU_WRITE_COEFS    equ_coefs;        /* Write filter coefficients */
   HAL_AUDIO_PCM_EQU_GAINSET        equ_gain;         /* Set equalizer gain */
   HAL_AUDIO_PCM_EQU_ENABLE         equ_enable;       /* Enable equalizer filer */
   HAL_AUDIO_PCM_READ               read;             /* Read digital samples from ADC */
   HAL_AUDIO_PCM_WRITE              write;            /* Write digital samples to DAC */
   HAL_AUDIO_PCM_GAIN_CAPABILITIES  gain_cap;         /* Retrieve gain capabilities */
   HAL_AUDIO_PCM_INFO               info;             /* Retrieve codec information */
};

typedef struct hal_audio_if_ops
{
   HAL_AUDIO_IF_INIT                init;             /* Initialize entire interface */
   HAL_AUDIO_IF_EXIT                exit;             /* Terminate entire interface */
   HAL_AUDIO_IF_PREPARE             prepare;          /* Prepare interface before enabling */
   HAL_AUDIO_IF_ENABLE              enable;           /* Enable interface. WARNING: Must not block! */
   HAL_AUDIO_IF_DISABLE             disable;          /* Disable entire interface */
   HAL_AUDIO_IF_ANA_POWERDN         ana_powerdn;      /* Power down analog interface to save power and/or reduce audible glitches */
   HAL_AUDIO_IF_INFO                info;             /* Retrieve audio interface information */

   struct hal_audio_pcm_ops         pcm_ops;          /* PCM ops */

} HAL_AUDIO_IF_OPS;

typedef int (*HAL_AUDIO_ADDONMODULE_INIT)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_EXIT)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_INGRESS)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_EGRESS)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_ENABLE)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_DISABLE)(void);
typedef int (*HAL_AUDIO_ADDONMODULE_RETEGP)( HAL_AUDIO_CODEC codecNum, short ** ptr, int * bufferSize );

typedef struct
{
   HAL_AUDIO_ADDONMODULE_INIT       init;
   HAL_AUDIO_ADDONMODULE_EXIT       exit;
   HAL_AUDIO_ADDONMODULE_ENABLE     enable;
   HAL_AUDIO_ADDONMODULE_DISABLE    disable;
   HAL_AUDIO_ADDONMODULE_INGRESS    ingress;
   HAL_AUDIO_ADDONMODULE_EGRESS     egress;
   HAL_AUDIO_GET_CODEC_INFO         getCodecInfo; 
   HAL_AUDIO_ADDONMODULE_RETEGP     getEgressPtr;
   HAL_AUDIO_GAINSET                gainSetAnalogHardware;  
   HAL_AUDIO_GAINSET                gainSetDigitalHardware; 

   /* Extension to add-on interface to provide a more generic method to 
    * support new audio interfaces; e.g. PCM, I2S, etc. 
    */
   struct hal_audio_if_ops          extended_ops;

} HAL_AUDIO_ADDONMODULE_FNC;

#define ADDONMODULE_NAMELEN    10
typedef struct 
{
   char                       name[ADDONMODULE_NAMELEN];
   int                        numMediaStreams;
   HAL_AUDIO_ADDONMODULE_FNC  *fncp;

} HAL_AUDIO_ADDONMODULE;

typedef int (*HAL_AUDIO_REGISTER_ADDON) (HAL_AUDIO_ADDONMODULE * addon );

typedef struct
{
   HAL_AUDIO_ISR isr;            /* Internal isr - do not call externally */
   HAL_AUDIO_INIT init;          /* Init - do not call externally */
   HAL_AUDIO_EXIT exit;          /* exit -do not call externally */
   HAL_AUDIO_ENABLE enable;      /* enable - do not call externally */

   HAL_AUDIO_GAINSET gainSetAnalogHardware;  /* Set analog hardware gain (block, db) */
   HAL_AUDIO_GAINSET gainSetDigitalHardware; /* Set digital hardware gain (block, db) */
   HAL_AUDIO_MICSEL micSel;                  /* Select Microphone input (obsolete)  */
   HAL_AUDIO_WRITE write;                    /* Write audio output (alsa mode only right now) */
   HAL_AUDIO_READ read;                      /* Read audio input (alsa mode only right now) */
   HAL_AUDIO_GETMODE    getMode;
   HAL_AUDIO_SELECTMODE selectMode;          /* Select audio mode (none, ldx, alsa) */
   HAL_AUDIO_GETCAPABILITIES getCapabilities;/* Get Audio capabilities of a block */
   HAL_AUDIO_GETFREQUENCIES getFrequencies;  /* Get Frequencies supported in a mode */

   HAL_AUDIO_SELECTPOWER selectPower;        /* Select power mode for audio blocks */
   HAL_AUDIO_QUERY_POWER queryPower;         /* Query power mode */

   HAL_AUDIO_CUSTOM_HARDWARE customHardware; /* Custom hardware configuration */

   HAL_AUDIO_EQU_PARMS_FP getEquParms;       /* Get equalizer parameters */
   HAL_AUDIO_EQU_PARMS_FP setEquParms;       /* Set equalizer parameters */

   HAL_AUDIO_GET_CODEC_INFO getCodecInfo;    /* Get Codec Info */

   HAL_AUDIO_REGISTER_ADDON   registerAddOn; /* register an addon module */

   HAL_AUDIO_REGISTER_CSX_IO_POINT registerCsxIoPoint; /* Register CSX I/O point functions */
   
} HAL_AUDIO_FNCS;

/**
* Structure of platform specific callbacks. These operations allow platform
* specific operation to be hooked into the core Hal Audio framework. For example, 
* set GPIOs when activating an external op-amp or LEDs.
*/
typedef struct hal_audio_platform_ops
{
   HAL_AUDIO_GAINSET ana_gain;               /* Analog gain set */
}
HAL_AUDIO_PLATFORM_OPS;


#define HALAUDIO_TRACE( fmt, args... )    do { if ( halaudio_gDbgPrint ) printk( KERN_INFO "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#define HALAUDIO_DEBUG( fmt, args... )    do { if ( halaudio_gDbgPrint >= 2 ) printk( KERN_INFO "%s: " fmt, __FUNCTION__ , ## args ); } while (0)

/* ---- Kernel Variable Externs ------------------------------------------ */
extern int halaudio_gDbgPrint;

/* ---- Kernel Function Prototypes --------------------------------------- */

int halAudio_allocate_client( void );
void halAudio_free_client ( int client );

#if defined(CONFIG_PLAT_BCM476X)
int halAudio_writeDev( int client, int codec, int numChannels, const char *audio, int bytes );
int halAudio_readDev( int client, int codec, int numChannels, const char *audio, int bytes );
#else
int halAudio_writeDev( int client, int numChannels, const char *audio, int bytes );
int halAudio_readDev( int client, int numChannels, const char *audio, int bytes );
#endif

int halAudio_getAnaGain( int client, HAL_AUDIO_BLOCK block );
int halAudio_setAnaGain( int client, HAL_AUDIO_BLOCK block, int gain );
int halAudio_alterGain( int client, int digital, HAL_AUDIO_BLOCK block, int *retGain, int direction);

int halAudio_getDigGain( int client, HAL_AUDIO_BLOCK block );
int halAudio_setDigGain( int client, HAL_AUDIO_BLOCK block, int gain );
int halAudio_blockEnable( int client, HAL_AUDIO_BLOCK block );
int halAudio_blockDisable( int client, HAL_AUDIO_BLOCK block );
int halAudio_blockQueryGain( int client, HAL_AUDIO_BLOCK block );
int halAudio_setActiveCodec( int client, HAL_AUDIO_CODEC codecNum );
int halAudio_getFrequencies( HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *freq );
HAL_AUDIO_MODE halAudio_getMode( void );
int halAudio_selectMode( int client, HAL_AUDIO_MODE mode, int freq );
int halAudio_getControl( int client, HAL_AUDIO_MODE mode, int freq );
int halAudio_releaseControl( int client );
int halAudio_enableAudio( int client, int bFullPower );
int halAudio_disableAudio( int client );
int halAudio_widebandCheck( void );
int halAudio_getGains(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *cap);

int halAudio_registerAudio(HAL_AUDIO_FNCS *audiop);

int halAudioExternalGetParams(const HAL_AUDIO_EXTERNAL_FNCS **paramp);
int halAudio_customHardware( int cmd, void *data );

void halAudio_queryPower( HAL_AUDIO_POWER_LEVEL *level, int *downRequestPending );

int halAudio_getEquParms( HALAUDIO_EQU_PARMS *parms );
int halAudio_setEquParms( HALAUDIO_EQU_PARMS *parms );

int halAudio_getCodecInfo( HALAUDIO_CODEC_INFO *codecInfo );

int halAudio_registerAddOnModule( HAL_AUDIO_ADDONMODULE * addon );

int halAudio_registerCsxIoPoint( HAL_AUDIO_CODEC codec, HAL_AUDIO_DIR dir, CSX_IO_POINT_FNCS *fncp, void *data );

int halAudio_MicSel( HAL_AUDIO_BLOCK block );
int halAudio_RegisterPlatform( const HAL_AUDIO_PLATFORM_OPS *ops );

int halAudio_setSuperUser(int client, int set, HAL_AUDIO_MODE mode);

#endif   /* __KERNEL__ */

/** @} */

#endif /* HALAUDIO_H */

