/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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
*  halaudio_hook.c
*
*  PURPOSE:
*
*     This driver is just a "hook" which controls multiple application
*     access to the HAL audio driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/sysctl.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <linux/broadcom/bcm_sysctl.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/halaudio.h>
#include <asm/io.h>
#include <asm/uaccess.h>

/* ---- Public Variables ------------------------------------------------- */

/* Debug print flag to gate verbose prints. This variable may be set 
*  via sysctl.
*/
int halaudio_gDbgPrint;

/* ---- Private Constants and Types -------------------------------------- */

//  Maximum number of clients
//
#define HAL_AUDIO_MAX_CLIENTS          6

//  Two directions: ADC and DAC
//
#define MAX_DIRECTIONS        2

//  Maximum number of hardware mux select positions
//
#define MAX_HWSEL            (HAL_AUDIO_HWSEL_MAX_NUM-HAL_AUDIO_HWSEL_A)

//  Structure for block gain and enable state
//
typedef struct audio_block
{
   int enable;    /* block is enabled */
   int db;        /* gain in db */
   int id;        /* block identifier */

} audio_block;

//  Structure to hold client state
//
typedef struct
{
   int used;

   //  We keep track of the active audio mode and freq
   //
   HAL_AUDIO_MODE eAudioMode;
   unsigned short usAudioFreq;

   //  Configured sampling frequencies per audio mode
   //
   unsigned int ldxFreq;
   unsigned int alsaFreq;
   unsigned int polyFreq;

   //  And the power level too.
   //
   HAL_AUDIO_POWER_LEVEL ePowerLevel;

   //  Gain settings for analog, digital and sidetone blocks
   //
   audio_block analog_block[HAL_AUDIO_CODEC_MAX_NUM][MAX_DIRECTIONS][MAX_HWSEL];
   audio_block digital_block[HAL_AUDIO_CODEC_MAX_NUM][MAX_DIRECTIONS];
   audio_block sidetone_block[HAL_AUDIO_CODEC_MAX_NUM];

   // Active codec for HAL audio operations
   HAL_AUDIO_CODEC   activeCodec;

   //  If we are not the active client, and an attempt is made to
   //  read/write audio, we need to block the reading/writing thread on this
   //  semaphore until we get control of the audio again.
   //
   struct  semaphore sSem;
   int  blocked;

   //  In our io callback, we record the number of samples read/written here.
   //
   int ioSamples;

   //  This superuser setting was added for those apps who want/need
   //  direct access to the audio (HAL) even if they are not the currently
   //  active application.  The initial user of this will be the callme
   //  debugging tool.
   int  superUser;

   //  Use for device write format
   int writeFormat;
   // Number of io channels
   int channels;
} halaudio_client;

//  Structure used to manage the context switches between clients
//
typedef struct 
{
   int client;
   int freq;
   HAL_AUDIO_MODE mode;

} LIFOSTACK;

//  Structure used to manage the platform specific operations
//
struct hal_audio_platform
{
   struct hal_audio_platform_ops ops;     /* registered operations */
   struct semaphore              mutex;   /* mutex protection */
};

#define WIDEBAND_THRESHOLD 16000

/* ---- Private Variables ------------------------------------------------ */

static halaudio_client gMyClients[HAL_AUDIO_MAX_CLIENTS];

//  This array is used to keep track of the order in which clients take control.
//  We use a LIFO algorithm for determine what client to give control back to
//  when the current client is done playing.
//
static LIFOSTACK gWaitingClients[HAL_AUDIO_MAX_CLIENTS];
static int gLIFOindex = -1;

//  Here is the table of HAL callbacks we will be using once the HAL registers with us.
//
static HAL_AUDIO_FNCS gHalFncs = {0};

static int gActiveClient = -1;
static int gIoClient = -1;
static int gSuperUsers = 0;

//  Here is the platform specific management structure
//  
static struct hal_audio_platform gPlatform;

DECLARE_WAIT_QUEUE_HEAD( gIoQueue );

static char banner[] __initdata = KERN_INFO "HALAUDIO Hook Driver: 1.0\n";

/**
* G.711 a-law and u-law lookup tables. 
*/
static short alawExpandTable[] =
{
   0xea80, 0xeb80, 0xe880, 0xe980, 0xee80, 0xef80, 0xec80, 0xed80,
   0xe280, 0xe380, 0xe080, 0xe180, 0xe680, 0xe780, 0xe480, 0xe580,
   0xf540, 0xf5c0, 0xf440, 0xf4c0, 0xf740, 0xf7c0, 0xf640, 0xf6c0,
   0xf140, 0xf1c0, 0xf040, 0xf0c0, 0xf340, 0xf3c0, 0xf240, 0xf2c0,
   0xaa00, 0xae00, 0xa200, 0xa600, 0xba00, 0xbe00, 0xb200, 0xb600,
   0x8a00, 0x8e00, 0x8200, 0x8600, 0x9a00, 0x9e00, 0x9200, 0x9600,
   0xd500, 0xd700, 0xd100, 0xd300, 0xdd00, 0xdf00, 0xd900, 0xdb00,
   0xc500, 0xc700, 0xc100, 0xc300, 0xcd00, 0xcf00, 0xc900, 0xcb00,
   0xfea8, 0xfeb8, 0xfe88, 0xfe98, 0xfee8, 0xfef8, 0xfec8, 0xfed8,
   0xfe28, 0xfe38, 0xfe08, 0xfe18, 0xfe68, 0xfe78, 0xfe48, 0xfe58,
   0xffa8, 0xffb8, 0xff88, 0xff98, 0xffe8, 0xfff8, 0xffc8, 0xffd8,
   0xff28, 0xff38, 0xff08, 0xff18, 0xff68, 0xff78, 0xff48, 0xff58,
   0xfaa0, 0xfae0, 0xfa20, 0xfa60, 0xfba0, 0xfbe0, 0xfb20, 0xfb60,
   0xf8a0, 0xf8e0, 0xf820, 0xf860, 0xf9a0, 0xf9e0, 0xf920, 0xf960,
   0xfd50, 0xfd70, 0xfd10, 0xfd30, 0xfdd0, 0xfdf0, 0xfd90, 0xfdb0,
   0xfc50, 0xfc70, 0xfc10, 0xfc30, 0xfcd0, 0xfcf0, 0xfc90, 0xfcb0,
   0x1580, 0x1480, 0x1780, 0x1680, 0x1180, 0x1080, 0x1380, 0x1280,
   0x1d80, 0x1c80, 0x1f80, 0x1e80, 0x1980, 0x1880, 0x1b80, 0x1a80,
   0x0ac0, 0x0a40, 0x0bc0, 0x0b40, 0x08c0, 0x0840, 0x09c0, 0x0940,
   0x0ec0, 0x0e40, 0x0fc0, 0x0f40, 0x0cc0, 0x0c40, 0x0dc0, 0x0d40,
   0x5600, 0x5200, 0x5e00, 0x5a00, 0x4600, 0x4200, 0x4e00, 0x4a00,
   0x7600, 0x7200, 0x7e00, 0x7a00, 0x6600, 0x6200, 0x6e00, 0x6a00,
   0x2b00, 0x2900, 0x2f00, 0x2d00, 0x2300, 0x2100, 0x2700, 0x2500,
   0x3b00, 0x3900, 0x3f00, 0x3d00, 0x3300, 0x3100, 0x3700, 0x3500,
   0x0158, 0x0148, 0x0178, 0x0168, 0x0118, 0x0108, 0x0138, 0x0128,
   0x01d8, 0x01c8, 0x01f8, 0x01e8, 0x0198, 0x0188, 0x01b8, 0x01a8,
   0x0058, 0x0048, 0x0078, 0x0068, 0x0018, 0x0008, 0x0038, 0x0028,
   0x00d8, 0x00c8, 0x00f8, 0x00e8, 0x0098, 0x0088, 0x00b8, 0x00a8,
   0x0560, 0x0520, 0x05e0, 0x05a0, 0x0460, 0x0420, 0x04e0, 0x04a0,
   0x0760, 0x0720, 0x07e0, 0x07a0, 0x0660, 0x0620, 0x06e0, 0x06a0,
   0x02b0, 0x0290, 0x02f0, 0x02d0, 0x0230, 0x0210, 0x0270, 0x0250,
   0x03b0, 0x0390, 0x03f0, 0x03d0, 0x0330, 0x0310, 0x0370, 0x0350,
};

static short ulawExpandTable[] =
{
   0x8284, 0x8684, 0x8a84, 0x8e84, 0x9284, 0x9684, 0x9a84, 0x9e84,
   0xa284, 0xa684, 0xaa84, 0xae84, 0xb284, 0xb684, 0xba84, 0xbe84,
   0xc184, 0xc384, 0xc584, 0xc784, 0xc984, 0xcb84, 0xcd84, 0xcf84,
   0xd184, 0xd384, 0xd584, 0xd784, 0xd984, 0xdb84, 0xdd84, 0xdf84,
   0xe104, 0xe204, 0xe304, 0xe404, 0xe504, 0xe604, 0xe704, 0xe804,
   0xe904, 0xea04, 0xeb04, 0xec04, 0xed04, 0xee04, 0xef04, 0xf004,
   0xf0c4, 0xf144, 0xf1c4, 0xf244, 0xf2c4, 0xf344, 0xf3c4, 0xf444,
   0xf4c4, 0xf544, 0xf5c4, 0xf644, 0xf6c4, 0xf744, 0xf7c4, 0xf844,
   0xf8a4, 0xf8e4, 0xf924, 0xf964, 0xf9a4, 0xf9e4, 0xfa24, 0xfa64,
   0xfaa4, 0xfae4, 0xfb24, 0xfb64, 0xfba4, 0xfbe4, 0xfc24, 0xfc64,
   0xfc94, 0xfcb4, 0xfcd4, 0xfcf4, 0xfd14, 0xfd34, 0xfd54, 0xfd74,
   0xfd94, 0xfdb4, 0xfdd4, 0xfdf4, 0xfe14, 0xfe34, 0xfe54, 0xfe74,
   0xfe8c, 0xfe9c, 0xfeac, 0xfebc, 0xfecc, 0xfedc, 0xfeec, 0xfefc,
   0xff0c, 0xff1c, 0xff2c, 0xff3c, 0xff4c, 0xff5c, 0xff6c, 0xff7c,
   0xff88, 0xff90, 0xff98, 0xffa0, 0xffa8, 0xffb0, 0xffb8, 0xffc0,
   0xffc8, 0xffd0, 0xffd8, 0xffe0, 0xffe8, 0xfff0, 0xfff8, 0x0000,
   0x7d7c, 0x797c, 0x757c, 0x717c, 0x6d7c, 0x697c, 0x657c, 0x617c,
   0x5d7c, 0x597c, 0x557c, 0x517c, 0x4d7c, 0x497c, 0x457c, 0x417c,
   0x3e7c, 0x3c7c, 0x3a7c, 0x387c, 0x367c, 0x347c, 0x327c, 0x307c,
   0x2e7c, 0x2c7c, 0x2a7c, 0x287c, 0x267c, 0x247c, 0x227c, 0x207c,
   0x1efc, 0x1dfc, 0x1cfc, 0x1bfc, 0x1afc, 0x19fc, 0x18fc, 0x17fc,
   0x16fc, 0x15fc, 0x14fc, 0x13fc, 0x12fc, 0x11fc, 0x10fc, 0x0ffc,
   0x0f3c, 0x0ebc, 0x0e3c, 0x0dbc, 0x0d3c, 0x0cbc, 0x0c3c, 0x0bbc,
   0x0b3c, 0x0abc, 0x0a3c, 0x09bc, 0x093c, 0x08bc, 0x083c, 0x07bc,
   0x075c, 0x071c, 0x06dc, 0x069c, 0x065c, 0x061c, 0x05dc, 0x059c,
   0x055c, 0x051c, 0x04dc, 0x049c, 0x045c, 0x041c, 0x03dc, 0x039c,
   0x036c, 0x034c, 0x032c, 0x030c, 0x02ec, 0x02cc, 0x02ac, 0x028c,
   0x026c, 0x024c, 0x022c, 0x020c, 0x01ec, 0x01cc, 0x01ac, 0x018c,
   0x0174, 0x0164, 0x0154, 0x0144, 0x0134, 0x0124, 0x0114, 0x0104,
   0x00f4, 0x00e4, 0x00d4, 0x00c4, 0x00b4, 0x00a4, 0x0094, 0x0084,
   0x0078, 0x0070, 0x0068, 0x0060, 0x0058, 0x0050, 0x0048, 0x0040,
   0x0038, 0x0030, 0x0028, 0x0020, 0x0018, 0x0010, 0x0008, 0x0000,
};

/**
* Global frequency sets for the different support modes. Initialized
* on HAL registration
*/
static HAL_AUDIO_FREQUENCIES gHalLdxFreqs;
static HAL_AUDIO_FREQUENCIES gHalAlsaFreqs;
static HAL_AUDIO_FREQUENCIES gHalPolyFreqs;

/* ---- Private Function Prototypes -------------------------------------- */

static int halAudio_updateAudioPower( void );
static int validateFreq( HAL_AUDIO_FREQUENCIES *setp, int freq );
static void sysCtlInit( void );
static void sysCtlExit( void );
static void sysCtlInit( void );
static void sysCtlExit( void );
static int  do_SetAnaGain( HAL_AUDIO_BLOCK block, int gain );
static int  do_BlockDisable( int client, HAL_AUDIO_BLOCK block );

/* ---- Functions -------------------------------------------------------- */

//  Helper routine to access analog block information
//
static inline audio_block *getAnalogBlock( int client, int codec, int dir, int mux )
{
   if ( client < 0 || client >= HAL_AUDIO_MAX_CLIENTS
   ||   codec >= HAL_AUDIO_CODEC_MAX_NUM 
   ||   ( dir != HAL_AUDIO_DAC_DIR && dir != HAL_AUDIO_ADC_DIR )
   ||   mux   >= HAL_AUDIO_HWSEL_MAX_NUM )
   {
      return NULL;
   }

   return &gMyClients[client].analog_block[codec-HAL_AUDIO_CODEC0][dir-HAL_AUDIO_ADC_DIR][mux-HAL_AUDIO_HWSEL_A];
}

//  Helper routine to access digital block information
//
static inline audio_block *getDigitalBlock( int client, int codec, int dir )
{
   if ( client < 0 || client >= HAL_AUDIO_MAX_CLIENTS
   ||   codec >= HAL_AUDIO_CODEC_MAX_NUM 
   ||   ( dir != HAL_AUDIO_DAC_DIR && dir != HAL_AUDIO_ADC_DIR )
   )
   {
      return NULL;
   }

   return &gMyClients[client].digital_block[codec-HAL_AUDIO_CODEC0][dir-HAL_AUDIO_ADC_DIR];
}

//  Helper routine to access sidetone block information
//
static inline audio_block *getSidetoneBlock( int client, int codec )
{
   if ( client < 0 || client >= HAL_AUDIO_MAX_CLIENTS
   ||   codec >= HAL_AUDIO_CODEC_MAX_NUM )
   {
      return NULL;
   }

   return &gMyClients[client].sidetone_block[codec-HAL_AUDIO_CODEC0];
}

/****************************************************************************
*
*  halAudio_allocate_client
*
***************************************************************************/

int halAudio_allocate_client( void )
{
   halaudio_client  *client;
   int               clientidx, i, j, k;

   client = &gMyClients[0];   
   for ( clientidx = 0; clientidx < HAL_AUDIO_MAX_CLIENTS; clientidx++, client++ )
   {
      if ( !client->used )
         break;
   }

   if ( clientidx == HAL_AUDIO_MAX_CLIENTS )
   {
      return -1;
   }

   //  Initialize default frequencies for client. 
   //
   
   if ( gHalLdxFreqs.numSettings )
   {
      client->ldxFreq = gHalLdxFreqs.freqSetting[gHalLdxFreqs.numSettings-1];
   }
   else
   {
      /* Default support */
      client->ldxFreq = 8000;
   }

   if ( gHalAlsaFreqs.numSettings )
   {
      client->alsaFreq = gHalAlsaFreqs.freqSetting[gHalAlsaFreqs.numSettings-1];
   }
   else
   {
      /* Default support */
      client->alsaFreq = 22050;
   }

   if ( gHalPolyFreqs.numSettings )
   {
      client->polyFreq = gHalPolyFreqs.freqSetting[gHalPolyFreqs.numSettings-1];
   }
   else
   {
      /* Default support */
      client->polyFreq = 22050;
   }

   //  Initialize other values for client.
   //

   client->used = 1;
   client->eAudioMode = HAL_AUDIO_MODE_NONE;
   client->usAudioFreq = 0;

   for ( i = 0; i < HAL_AUDIO_CODEC_MAX_NUM; i++ )
   {
      for ( j = 0; j < MAX_DIRECTIONS; j++ )
      {
         for ( k = 0; k < MAX_HWSEL; k++ )
         {
            client->analog_block[i][j][k].enable   = 0;
            client->analog_block[i][j][k].db       = HAL_AUDIO_GAIN_SLEEP;
            client->analog_block[i][j][k].id       = HAL_AUDIO_BLOCK_ID( i+HAL_AUDIO_CODEC0, k+HAL_AUDIO_HWSEL_A, j+HAL_AUDIO_ADC_DIR );
         }
         client->digital_block[i][j].enable        = 0;
         client->digital_block[i][j].db            = HAL_AUDIO_GAIN_SLEEP;
         client->digital_block[i][j].id            = HAL_AUDIO_BLOCK_ID( i+HAL_AUDIO_CODEC0, 0, j+HAL_AUDIO_ADC_DIR );
// [jlh-removed]         gMyClients[i].blockEnabled[j] = 0;
      }
      client->sidetone_block[i].enable             = 0;
      client->sidetone_block[i].db                 = HAL_AUDIO_GAIN_SLEEP;
      client->sidetone_block[i].id                 = HAL_AUDIO_SIDETONE_ID( i+HAL_AUDIO_CODEC0 );
   }

   client->blocked      = 0;
   client->ePowerLevel  = HAL_AUDIO_POWER_DEEP_SLEEP;
   client->ioSamples    = 0;
   client->superUser    = 0;
   client->activeCodec  = -1;

   sema_init( &client->sSem, 0 );

   //  By default, S16_BE format selected
   //

   client->writeFormat = HALAUDIO_FORMAT_S16_BE;

   return clientidx;

} /* halAudio_allocate_client */

/****************************************************************************
*
*  halAudio_free_client
*
***************************************************************************/

void halAudio_free_client ( int client )
{
   if ((client < 0) || (client >= HAL_AUDIO_MAX_CLIENTS))
      return;

   if (!gMyClients[client].used)
      return;

   gMyClients[client].used = 0;
   halAudio_releaseControl( client );

   //  If we are a super user app, decrement the super user counter.
   //
   if (gMyClients[client].superUser)
   {
      gSuperUsers--;
   }

   //  In case we were influencing the power setting, update it.
   //
   halAudio_updateAudioPower();

   if (gMyClients[client].blocked)
   {
      //  Someone is waiting for the audio.  Wake them up.
      //
      up( &gMyClients[client].sSem );
   }
} /* halAudio_free_client */

/****************************************************************************
*
*  halAudio_open
*
***************************************************************************/

static int halAudio_open( struct inode *inode, struct file *file )
{
   int client;

   //  Register this client.  Note: only the phone application opens us from
   //  user space.  If this is alsa-based, they will have to use an IOCTL call
   //  to change things.
   //
   client = halAudio_allocate_client();
   if (client < 0)
   {
      return -EBUSY;
   }
   file->private_data = (void *) client;

   return 0;

} /* halAudio_open */

/****************************************************************************
*
*  halAudio_ioCallback
*
***************************************************************************/
#if defined(CONFIG_PLAT_BCM476X)
static void halAudio_ioCallback( unsigned short numSamples, short *audioBuffer, void *cb_param)
#else
static void halAudio_ioCallback( unsigned short numSamples, short *audioBuffer )
#endif
{
   int client;

#if defined(CONFIG_PLAT_BCM476X)
   client = (int) cb_param;
#else
   client = gIoClient;
#endif
   //  Record the number of samples read/written and wake up the active client's thread.
   //
   //printk("halAudio_ioCallback: %d samples, gIoClient %d\n", numSamples, gIoClient);
   if (client >= 0) {
       gIoClient = -1;
       gMyClients[client].ioSamples = numSamples;
       gMyClients[client].blocked = 0;
       up( &gMyClients[client].sSem );
       wake_up_interruptible( &gIoQueue );
   }

} /* halAudio_ioCallback */

/****************************************************************************
*
*  halAudio_read
*
***************************************************************************/

static ssize_t halAudio_read( struct file *file, char *buffer, size_t count, loff_t *ppos )
{
   int channels = gMyClients[(int) file->private_data].channels;
#if defined(CONFIG_PLAT_BCM476X)
   return halAudio_readDev( (int) file->private_data,  gMyClients[(int) file->private_data].activeCodec, channels, buffer, (int) count );
#else
    return halAudio_readDev( (int) file->private_data, channels, buffer, (int) count );
#endif
} /* halAudio_read */

/****************************************************************************
*
*  halAudio_release
*
***************************************************************************/

static int halAudio_release( struct inode *inode, struct file *file )
{
   halAudio_free_client ( (int) file->private_data );
   return 0;

} /* halAudio_release */

/****************************************************************************
*
*  G.711 uncompress helper
*
***************************************************************************/
void uncompress( short *outbuf, unsigned char *inbuf, const short *table, int count )
{
   int i; 

   /* Uncompress from back to front in the case output buffer 
    * is the same as input buffer to save memory
    */
   for ( i = count - 1; i >= 0; i-- )
   {
      unsigned int offset = inbuf[i];
      outbuf[i] = table[offset];
   }
}

/****************************************************************************
*
*  halAudio_write
*
***************************************************************************/

static ssize_t halAudio_write( struct file *file, const char __user *buffer, size_t count, loff_t *ppos )
{
   unsigned char *kbuf;
   int channels;
   int err;
   HALAUDIO_WRITE_FORMAT writeFormat;
   int uncompBufSize;
#if defined (__LITTLE_ENDIAN)
   int be_flag = 0;
#elif defined (__BIG_ENDIAN)
   int be_flag = 1;
#else
#error No endian defined.
#endif

   writeFormat = gMyClients[(int) file->private_data].writeFormat;
   channels = gMyClients[(int) file->private_data].channels;
   uncompBufSize = count;
   if ( writeFormat == HALAUDIO_FORMAT_ULAW || writeFormat == HALAUDIO_FORMAT_ALAW )
   {
      /* uncompressed size */
      uncompBufSize *=2;
   }

   kbuf = vmalloc( uncompBufSize );
   if ( !kbuf )
   {
      return -ENOMEM;
   }
   if (( err = copy_from_user( kbuf, buffer, count )))
   {
      printk( KERN_ERR "failed to copy data from user buffer rc=%i\n", err );
      vfree( kbuf );
      return -EINVAL;
   }

   if ( writeFormat == HALAUDIO_FORMAT_ULAW ) 
   {
      uncompress( (short *)(void *)kbuf, kbuf, ulawExpandTable, count );
   }
   else if ( writeFormat == HALAUDIO_FORMAT_ALAW ) 
   {
      uncompress( (short *)(void *)kbuf, kbuf, alawExpandTable, count );
   }
   else if (( be_flag && writeFormat != HALAUDIO_FORMAT_S16_BE ) 
   ||       ( be_flag == 0 && writeFormat != HALAUDIO_FORMAT_S16_LE ))
   {
      int i;
      unsigned char *tmpbuf = kbuf;

      if ( count & 1 ) 
      {
         /* must be even number of bytes for 16-bit data */
         vfree( kbuf );
         return -EINVAL;
      }

      /* Byte swap 16-bit data when format inconsistent with machine */ 
      for ( i = 0; i < count; i += 2 )
      {
         unsigned char swap = *tmpbuf;
         tmpbuf[0] = tmpbuf[1];
         tmpbuf++;
         *tmpbuf = swap;
         tmpbuf++;
      }
   }
#if defined(CONFIG_PLAT_BCM476X)
   err = halAudio_writeDev( (int)file->private_data, gMyClients[(int) file->private_data].activeCodec, channels, kbuf, uncompBufSize );
#else
   err = halAudio_writeDev( (int)file->private_data, channels, (char*)kbuf, uncompBufSize );
#endif
   if ( err == uncompBufSize )
   {
      /* Successful write, return original uncompressed buffer size. */
      err = count;
   }

   vfree( kbuf );

   return err;

} /* halAudio_write */


/****************************************************************************
*
*  Query for the mode's configured frequency
*
***************************************************************************/

static int getFreq( HAL_AUDIO_MODE mode, int client )
{
   int freq;
   switch ( mode )
   {
      case HAL_AUDIO_MODE_LDX:
         freq = gMyClients[client].ldxFreq;
         break;
      case HAL_AUDIO_MODE_ALSA:
         freq = gMyClients[client].alsaFreq;
         break;
      case HAL_AUDIO_MODE_POLYRINGER:
         freq = gMyClients[client].polyFreq;
         break;
      default:
         freq = 0;
         break;
   }
   return freq;
}

/****************************************************************************
*
*  halAudio_ioctl
*
***************************************************************************/

static int halAudio_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   int rc = 0;
   int client = (int) file->private_data;
   switch (cmd)
   {
      case HALAUDIO_IOCTL_SUPERUSER:
      {
         HAL_AUDIO_MODE mode = (HAL_AUDIO_MODE)arg;
         halAudio_setSuperUser(client, 1, mode);
      }
      break;
      case HALAUDIO_IOCTL_WIDEBAND_CHECK:
      {
         rc = halAudio_widebandCheck();
      }
      break;
      case HALAUDIO_IOCTL_GET_CONTROL:
      {
         int freq;
         HAL_AUDIO_MODE mode = (HAL_AUDIO_MODE)arg;

         freq = getFreq( mode, client );
         rc = halAudio_getControl( client, mode, freq );
      }
      break;
      case HALAUDIO_IOCTL_RELEASE_CONTROL:
      {
         halAudio_releaseControl( (int) file->private_data );
      }
      break;

      case HALAUDIO_IOCTL_GET_AUDIO_GAIN:
      {
         HALAUDIO_AUDIO_GAIN gain;
         rc = 0;
         if( copy_from_user( &gain, (HALAUDIO_AUDIO_GAIN *)arg, sizeof( gain ) ) )
         {
            rc = -EFAULT;
         }
         else
         {
            gain.analogGain = halAudio_getAnaGain( (int) file->private_data, gain.block );
            gain.digitalGain = halAudio_getDigGain( (int) file->private_data, gain.block );
            if( copy_to_user( (HALAUDIO_AUDIO_GAIN *)arg, &gain, sizeof( gain ) ) )
            {
               rc = -EFAULT;
            }
         }
      }
      break;

      case HALAUDIO_IOCTL_SET_AUDIO_GAIN:
      {
         HALAUDIO_AUDIO_GAIN gain;
         if( copy_from_user( &gain, (HALAUDIO_AUDIO_GAIN *)arg, sizeof( gain ) ) )
         {
            rc = -EFAULT;
         }
         else
         {
            rc = halAudio_setAnaGain( (int) file->private_data, gain.block, gain.analogGain );
            rc |= halAudio_setDigGain( (int) file->private_data, gain.block, gain.digitalGain );
         }
      }
      break;

      case HALAUDIO_IOCTL_ALTER_AUDIO_GAIN:
      {
         HALAUDIO_AUDIO_ALTER alter;
         if( copy_from_user( &alter, (HALAUDIO_AUDIO_ALTER *)arg, sizeof( alter ) ) )
         {
            rc = -EFAULT;
         }
         else
         {
            rc = halAudio_alterGain( (int) file->private_data, 
                  alter.bDigital,
                  alter.block, 
                  &alter.resultingGain, 
                  alter.direction );
            if ( rc == 0 )
            {
               rc = copy_to_user( (void *)arg, &alter, sizeof(alter) );
            }
         }
      }
      break;
      case HALAUDIO_IOCTL_SET_POWER:
      {
         if ((HAL_AUDIO_POWER_LEVEL) arg == HAL_AUDIO_POWER_DEEP_SLEEP)
         {
            rc = halAudio_disableAudio( (int) file->private_data );
         }
         else
         {
            rc = halAudio_enableAudio( (int) file->private_data, ((HAL_AUDIO_POWER_LEVEL)arg == HAL_AUDIO_POWER_RESUME_ALL) );
         }
      }
      break;
      case HALAUDIO_IOCTL_GET_POWER:
      {
         if (gMyClients[(int) file->private_data].superUser)
         {
            HAL_AUDIO_POWER_LEVEL powerLevel = HAL_AUDIO_POWER_DEEP_SLEEP;
            int downPending;

            //  Super user apps just see what the currently active app does.
            //
            if (gHalFncs.queryPower)
            {
               gHalFncs.queryPower( &powerLevel, &downPending );
               rc = (int) powerLevel;
            }
            else
            {
               //  Cannot directly query power level, then super user apps just
               //  sees what the currently active app does.
               //
               rc = (int) gMyClients[(gActiveClient < 0) ? 0 : gActiveClient].ePowerLevel;
            }
         }
         else
         {
            rc = (int) gMyClients[(int) file->private_data].ePowerLevel;
         }
      }
      break;
      case HALAUDIO_IOCTL_WRITE_FORMAT:
      {
         if (  (HALAUDIO_WRITE_FORMAT)arg < HALAUDIO_FORMAT_MAX )
         {
            gMyClients[(int) file->private_data].writeFormat = (HALAUDIO_WRITE_FORMAT)arg;
         }
         else
         {
            rc = EINVAL;
         }
      }
      break;
      case HALAUDIO_IOCTL_CONFIG_FREQ:
      {
         HALAUDIO_CONFIG_FREQ config; 

         if ( copy_from_user( &config, (void *)arg, sizeof(config) ))
         {
            /* Failed to copy parameters from user */
            rc = -EINVAL; 
         }
         else
         {
            int oldmode = gMyClients[client].eAudioMode;
            int oldfreq = 0;
            rc = -EINVAL;
            switch ( config.mode )
            {
               case HAL_AUDIO_MODE_LDX:
                  if ( validateFreq( &gHalLdxFreqs, config.freq ))
                  {
                     oldfreq = gMyClients[client].ldxFreq;
                     gMyClients[client].ldxFreq = config.freq;
                     rc = 0;
                  }
                  break;
               case HAL_AUDIO_MODE_ALSA:
                  if ( validateFreq( &gHalAlsaFreqs, config.freq ))
                  {
                     oldfreq = gMyClients[client].alsaFreq;
                     gMyClients[client].alsaFreq = config.freq;
                     rc = 0;
                  }
                  break;
               case HAL_AUDIO_MODE_POLYRINGER:
                  if ( validateFreq( &gHalPolyFreqs, config.freq ))
                  {
                     oldfreq = gMyClients[client].polyFreq;
                     gMyClients[client].polyFreq = config.freq;
                     rc = 0;
                  }
                  break;
               default:
                  /* invalid mode */
                  rc = -EINVAL;
                  break;
            }
            if ( rc == 0 )
            {
               /* Change mode and frequency only if different
                */
               if ( oldmode != config.mode || oldfreq != config.freq )
               {
                  rc = halAudio_selectMode( client, config.mode, config.freq );
               }
            }
         }
      }
      break;
      case HALAUDIO_IOCTL_GET_EQU_PARMS:
      {
         HALAUDIO_EQU_PARMS parms;
         /* Read codec ID to retrieve equalizer parameters */
         if ( !( rc = copy_from_user( &parms.codec, &((HALAUDIO_EQU_PARMS *)arg)->codec, sizeof(parms.codec) )))
         {
            rc = halAudio_getEquParms( &parms );
            if ( rc == 0 )
            {
               rc = copy_to_user( (void *)arg, &parms, sizeof(parms) );
            }
         }
      }
      break;
      case HALAUDIO_IOCTL_SET_EQU_PARMS:
      {
         HALAUDIO_EQU_PARMS parms;
         if ( !( rc = copy_from_user( &parms, (void *)arg, sizeof(parms) )))
         {
            rc = halAudio_setEquParms( &parms );
         }
      }
      break;

      case HALAUDIO_IOCTL_CUSTOM_HARDWARE:
      {
         HALAUDIO_CUSTOM_HARDWARE parms; 
         if ( !(rc = copy_from_user( &parms, (void *)arg, sizeof(parms) )))
         {
            rc = (int) halAudio_customHardware( (int) parms.cmd, parms.data );
         }
      }
      break;

      case HALAUDIO_IOCTL_BLOCK_ENABLE:
         rc = (int) halAudio_blockEnable( (int)file->private_data, (HAL_AUDIO_BLOCK)arg );
         break;

      case HALAUDIO_IOCTL_BLOCK_DISABLE:
         rc = (int) halAudio_blockDisable( (int)file->private_data, (HAL_AUDIO_BLOCK)arg );
         break;

      case HALAUDIO_IOCTL_BLOCK_QUERY:
         rc = (int) halAudio_blockQueryGain( (int)file->private_data, (HAL_AUDIO_BLOCK)arg );
         break;

      case HALAUDIO_IOCTL_SET_CODEC:
         gMyClients[client].activeCodec = (HAL_AUDIO_CODEC)arg;
         break;

      case HALAUDIO_IOCTL_GET_CODEC_INFO:
      {
         HALAUDIO_CODEC_INFO codecInfo;
         if( copy_from_user( &codecInfo, (HALAUDIO_CODEC_INFO *)arg, sizeof( codecInfo ) ) )
         {
            printk( KERN_ERR "failed to copy input argument from user\n" );
            return -EINVAL;
         }
         rc = halAudio_getCodecInfo( &codecInfo );
         if ( rc == 0 )
         {
            if( copy_to_user( (HALAUDIO_CODEC_INFO *)arg, &codecInfo, sizeof(codecInfo) ) )
            {
               rc = -EFAULT;
            }
         }
      }
      break;
      
      case HALAUDIO_IOCTL_SET_CHANNELS:
         gMyClients[client].channels = arg;
         break;

      case HALAUDIO_IOCTL_MIC_SELECT:
      {
         rc = halAudio_MicSel( (HAL_AUDIO_BLOCK)arg );
      }
      break;
      
      default:
      {
         return -EINVAL;
      }
   }

   return rc;

} /* halAudio_ioctl */

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations halAudio_fops =
{
   owner:      THIS_MODULE,
   open:       halAudio_open,
   release:    halAudio_release,
   read:       halAudio_read,
   write:      halAudio_write,
   ioctl:      halAudio_ioctl
};

/****************************************************************************
*
*  halAudio_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init halAudio_init( void )
{
   int rc;
   int i;

   printk( banner );

   if (( rc = register_chrdev( BCM_HALAUDIO_MAJOR, "alsahook", &halAudio_fops )) < 0 )
   {
      printk( KERN_ERR "alsaHook: register_chrdev failed for major %d\n", BCM_HALAUDIO_MAJOR );
      return rc;
   }

   // Removed, used static initialization instead because the builtin audio driver may register to the 
   // halAudio_hook before this function gets called.
   //  Null out the table of HAL function pointers. 
   //memset( (void *) &gHALFncs, 0, sizeof( gHALFncs ) ); 

   //  Initialize the structures for client information.
   //
   for (i = 0; i < HAL_AUDIO_MAX_CLIENTS; i++)
   {
      gMyClients[i].used = 0;
      gWaitingClients[i].client = -1;
   }

   sysCtlInit();

   //  Initialize platform mutex
   //
   init_MUTEX( &gPlatform.mutex );

   return 0;

} /* halAudio_init */

/****************************************************************************
*
*  halAudio_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit halAudio_exit( void )
{
   sysCtlExit();

} /* halAudio_exit */

/****************************************************************************
*
*  halAudio_writeDev
*
*  Used by other kernel level modules to send samples to the speaker
*
***************************************************************************/
#if defined(CONFIG_PLAT_BCM476X)
int halAudio_writeDev( int client, int codec, int numChannels, const char *audio, int bytes )
#else
int halAudio_writeDev( int client, int numChannels, const char *audio, int bytes )
#endif
{
   HALAUDIO_TRACE( "client=%i numChans=%i bytes=%i\n", client, numChannels, bytes );
     
   if (gHalFncs.write)
   {
      int rc;
      int target_codec;

      //  Wait until we are the active client if we are not the superUser
      //
      while ((client != gActiveClient) && (!gMyClients[client].superUser))
      {
         gMyClients[client].blocked = 1;
         if (down_interruptible( &gMyClients[client].sSem ) < 0 )
         {
            //  We failed on the semaphore.
            //
            gMyClients[client].blocked = 0;
            return -EINVAL;
         }
         if (!gMyClients[client].used)
         {
            //  We have been cancelled.
            //
            return -EINVAL;
         }
      }

#if defined(CONFIG_PLAT_BCM476X)
      target_codec = codec; // BCM476X can have multiple codecs active, use the one selected instead of the global active codec
#else
      if( gMyClients[client].activeCodec < 0 )
      {
         // no proper playout block has been set
         printk( KERN_ERR " no proper playout block has been set for halAudioWrite\n" );
         return -EPERM;
      }
      target_codec = gMyClients[client].activeCodec;
#endif

      gIoClient = client;

      //  As we only support 16bit samples, we need to divide bytes by 2 to get the number
      //  of samples being passed in.
#if defined(CONFIG_PLAT_BCM476X)            
      rc = gHalFncs.write( bytes/2, numChannels, target_codec, (short *) audio, halAudio_ioCallback, (void *) client );
#else
      rc = gHalFncs.write( bytes/2, numChannels, target_codec, (short *) audio, halAudio_ioCallback );
#endif
      if (rc < 0)
      {
         //  There is a problem with writing.
         //
         return rc;
      }

      //  The call to HAL write is non-blocking so we need to use our semaphore to block
      //  until the write operation is done.
      //
      gMyClients[client].blocked = 1;
      if (down_interruptible( &gMyClients[client].sSem ) < 0 )
      {
         //  We failed on the semaphore.
         //
         gMyClients[client].blocked = 0;
         return -EINVAL;
      }

      //  Return bytes
      return(gMyClients[client].ioSamples * 2);
   }

   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_readDev
*
*  Used by other kernel level modules to read samples from the Mic.
*
***************************************************************************/
#if defined(CONFIG_PLAT_BCM476X)
int halAudio_readDev( int client, int codec, int numChannels, const char *audio, int bytes )
#else
int halAudio_readDev( int client, int numChannels, const char *audio, int bytes )
#endif
{
   int rc;

   if (gHalFncs.read)
   {
      //  Wait until we are the active client if we are not the superUser
      //
      while ((client != gActiveClient) && (!gMyClients[client].superUser))
      {
         gMyClients[client].blocked = 1;
         if (down_interruptible( &gMyClients[client].sSem ) < 0 )
         {
            //  We failed on the semaphore.
            //
            gMyClients[client].blocked = 0;
            return -EINVAL;
         }
         if (!gMyClients[client].used)
         {
            //  We have been cancelled.
            //
            return -EINVAL;
         }
      }

      gIoClient = client;
#if defined(CONFIG_PLAT_BCM476X)
      rc = gHalFncs.read( bytes/2, numChannels, codec, (short *) audio, halAudio_ioCallback, (void*) client );
#else
      rc = gHalFncs.read( bytes/2, numChannels, (short *) audio, halAudio_ioCallback );
#endif

      if (rc < 0)
      {
         //  There is a problem with writing.
         //
         return rc;
      }

      //  The call to HAL read is non-blocking so we need to use our semaphore to block
      //  until the read operation is done.
      //
      gMyClients[client].blocked = 1;
      if (down_interruptible( &gMyClients[client].sSem ) < 0 )
      {
         //  We failed on the semaphore.
         //
         gMyClients[client].blocked = 0;
         return -EINVAL;
      }

      return(gMyClients[client].ioSamples * 2);
   }

   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_updateAudioPower
*
*  Local routine to update the current audio power setting taking into
*  account all client's desires.
*
***************************************************************************/
static int halAudio_updateAudioPower( void )
{
   int rc;
   int i;
   HAL_AUDIO_POWER_LEVEL finalLevel = HAL_AUDIO_POWER_DEEP_SLEEP;

   //  This routine is turned off as long as there are super users.
   //
   if (gSuperUsers)
   {
      return 0;
   }

   //  Look over all client's power setting to determine what we should do
   //  for the final setting.
   //
   for (i = 0; i < HAL_AUDIO_MAX_CLIENTS; i++)
   {
      if ((gMyClients[i].used) && (gMyClients[i].ePowerLevel > finalLevel))
      {
         finalLevel = gMyClients[i].ePowerLevel;
      }
   }

   //  Set the audio power to the decided level.
   //
   if (gHalFncs.selectPower)
   {
      if (finalLevel == HAL_AUDIO_POWER_DEEP_SLEEP)
      {
         //  Just to make sure all buffers destined to the DSP are empty, we
         //  delay 300ms here before cutting the power.
         //
         wait_event_interruptible_timeout( gIoQueue, 0, 30);
      }
      rc = gHalFncs.selectPower( finalLevel );
      return rc;
   }
   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_enableAudio
*
*  Used by other kernel level modules to enable the audio block via endpoint
*
***************************************************************************/
int halAudio_enableAudio( int client, int bFullPower )
{
   HALAUDIO_TRACE( "client=%i fullpower=%i\n", client, bFullPower );

   if ((client >= 0) && (client < HAL_AUDIO_MAX_CLIENTS))
   {
      if (gMyClients[client].superUser)
      {
         //  Super user app directly accesses the HAL
         //
         return gHalFncs.selectPower( (bFullPower) ? HAL_AUDIO_POWER_RESUME_ALL : HAL_AUDIO_POWER_DIGITAL_ONLY );
      }
      else
      {
         gMyClients[client].ePowerLevel = (bFullPower) ? HAL_AUDIO_POWER_RESUME_ALL :
                                          HAL_AUDIO_POWER_DIGITAL_ONLY;
      }

      return halAudio_updateAudioPower();
   }
   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_disableAudio
*
*  Used by other kernel level modules to disable the audio block via endpoint
*
***************************************************************************/
int halAudio_disableAudio( int client )
{
   HALAUDIO_TRACE( "client=%i\n", client );

   if ((client >= 0) && (client < HAL_AUDIO_MAX_CLIENTS))
   {
      if (gMyClients[client].superUser)
      {
         //  Super user app directly accesses the HAL
         //
         return gHalFncs.selectPower( HAL_AUDIO_POWER_DEEP_SLEEP );
      }
      else
      {
         //  Change this client's desired power setting.
         //
         gMyClients[client].ePowerLevel = HAL_AUDIO_POWER_DEEP_SLEEP;
      }

      return halAudio_updateAudioPower();
   }
   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_getAnaGain
*
*  Used by other kernel level modules to get the volume level.
*
***************************************************************************/
int halAudio_getAnaGain( int client, HAL_AUDIO_BLOCK block )
{
   HAL_AUDIO_CAPABILITIES  cap;
   audio_block            *blockp;
   int                     dir, codec, mux, db;

   HALAUDIO_TRACE( "client=%d block=0x%x\n", client, block );

   codec    = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir      = HAL_AUDIO_BLOCK_GET_DIR( block );
   mux      = HAL_AUDIO_BLOCK_GET_HWSEL( block );
   blockp   = HAL_AUDIO_BLOCK_IS_SIDETONE( block ) ? getSidetoneBlock( client, codec ) : getAnalogBlock( client, codec, dir, mux );

   if ( !blockp )
   {
      return 0;   // Error, return 0
   }

   db = blockp->db;
   
   //  Only a super user client gets to see the actual hardware settings.  The main
   //  reason for this is that some blocks get muted automatically when we switch
   //  between speaker and aux.  From the client's perspective, this should be
   //  invisible so we should only show the client what is actually set in their
   //  local gain block, not what is implemented in the HAL.
   //
#if !defined(CONFIG_PLAT_BCM476X)
   if (gMyClients[client].superUser)
#endif
   {
      if (gHalFncs.getCapabilities( block, &cap ) < 0)
      {
         return 0;   // FIXME
      }
      db = cap.analog.currentDB;
   }

   return db;
}

/****************************************************************************
*
*  halAudio_getDigGain
*
*  Used by other kernel level modules to get the volume level.
*
***************************************************************************/
int halAudio_getDigGain( int client, HAL_AUDIO_BLOCK block )
{
   HAL_AUDIO_CAPABILITIES  cap;
   audio_block            *blockp;
   int                     dir, codec, db;

   HALAUDIO_TRACE( "client=%d block=0x%x\n", client, block );

   //  Digital sidetone gain is unsupported
   //
   if ( HAL_AUDIO_BLOCK_IS_SIDETONE( block ))
   {
      return 0;
   }

   codec  = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir    = HAL_AUDIO_BLOCK_GET_DIR( block );
   blockp = getDigitalBlock( client, codec, dir );

   if ( blockp == NULL )
   {
      return 0;   // Error, return 0
   }

   db = blockp->db;

   //  Only a super user client gets to see the actual HAL settings.  The main
   //  reason for this is that some blocks get muted automatically when we switch
   //  between speaker and aux.  From the client's perspective, this should be
   //  invisible so we should only show the client what is actually set in their
   //  local gain block, not what is implemented in the HAL.
   //
   if (gMyClients[client].superUser)
   {
      if (gHalFncs.getCapabilities( block, &cap ) < 0)
      {
         return 0;
      }
      db = cap.digital.currentDB;
   }

   return db;
}

/****************************************************************************
*
*  Stateless helper routine used to setup hardware analog gain. 
*  The gain is not saved. 
*
***************************************************************************/
static int do_SetAnaGain( HAL_AUDIO_BLOCK block, int gain )
{
   int err = 0;

   //  Call primary driver
   //
   if ( gHalFncs.gainSetAnalogHardware )
   {
      err = gHalFncs.gainSetAnalogHardware( block, gain );
      if ( err )
      {
         return err;
      }
   }

   //  Call platform callback 
   //
   if ( down_interruptible( &gPlatform.mutex ) < 0 )
   {
      //  Failed on the semaphore. Likely interrupted
      //
      return -EAGAIN;
   }
   if ( gPlatform.ops.ana_gain )
   {
      if ( gPlatform.ops.ana_gain( block, gain ) < 0 )
      {
         return -2;
      }
   }
   up( &gPlatform.mutex );

   return err;
}
/****************************************************************************
*
*  halAudio_setAnaGain
*
*  Used by other kernel level modules to set the analog gain level.
*
***************************************************************************/
int halAudio_setAnaGain( int client, HAL_AUDIO_BLOCK block, int gain)
{
   audio_block *blockp;
   int          dir, codec, mux;
   int          err = 0;

   HALAUDIO_TRACE( "client=%d block=0x%x gain=%d\n", client, block, gain );

   codec    = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir      = HAL_AUDIO_BLOCK_GET_DIR( block );
   mux      = HAL_AUDIO_BLOCK_GET_HWSEL( block );
   blockp   = HAL_AUDIO_BLOCK_IS_SIDETONE( block ) ? getSidetoneBlock( client, codec ) : getAnalogBlock( client, codec, dir, mux );

   if ( !blockp )
   {
      return -EINVAL;
   }
#if !defined(CONFIG_PLAT_BCM476X) // Must sync with HW for BCM4760
   //  Activate analog gain if client is a superuser, or is the active client,
   //  block is enabled and no superuser is registered
   //
   if (((client == gActiveClient) && (gSuperUsers == 0) && (blockp->enable))
   ||  (gMyClients[client].superUser))
#endif
   {
      err = do_SetAnaGain( block, gain );
   }

   if ( !err )
   {
      blockp->db = gain;
   }
   
   return err;
}

/****************************************************************************
*
*  halAudio_setDigGain
*
*  Used by other kernel level modules to set the digital gain level.
*
***************************************************************************/
int halAudio_setDigGain( int client, HAL_AUDIO_BLOCK block, int gain )
{
   audio_block *blockp;
   int          dir, codec;

   HALAUDIO_TRACE( "client=%d block=0x%x gain=%d\n", client, block, gain );

   //  Digital sidetone gain is unsupported
   //
   if ( HAL_AUDIO_BLOCK_IS_SIDETONE( block ))
   {
      return 0;
   }

   codec  = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir    = HAL_AUDIO_BLOCK_GET_DIR( block );
   blockp = getDigitalBlock( client, codec, dir );

   if ( blockp == NULL )
   {
      return -EINVAL;
   }

   if (gHalFncs.gainSetDigitalHardware)
   {
      //  Set digital gain if client is a superuser, or is the active client,
      //  block is enabled and no superuser is registered
      //
      if (( client == gActiveClient && gSuperUsers == 0 ) 
      ||    gMyClients[client].superUser )
      {
         if (gHalFncs.gainSetDigitalHardware( block, gain ) < 0)
            return -1;
      }
   }

   blockp->db = gain;

   return 0;
}

/****************************************************************************
*
*  halAudio_alterGain
*
*  Used by other kernel level modules to alter up or down the analog gain level.
*
***************************************************************************/
int halAudio_alterGain( int client, int digital, HAL_AUDIO_BLOCK block, int *retGain, int direction)
{
   HAL_AUDIO_CAPABILITIES cap;
   HAL_AUDIO_DB *audioDb;
   int db = 0;
   audio_block *blockp;
   int rc = 0;
   int dir, codec, mux;

   HALAUDIO_TRACE( "client=%d block=0x%x digital=%d notches=%d\n", client, block, digital, direction );

   //  Not supported for sidetone
   //
   if ( HAL_AUDIO_BLOCK_IS_SIDETONE( block ))
   {
      return -EINVAL;
   }

   //  Get the audio capabilities for this block.
   //
   if (halAudio_getGains( block, &cap ) < 0)
   {
      return -1;
   }

   codec  = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir    = HAL_AUDIO_BLOCK_GET_DIR( block );
   mux    = HAL_AUDIO_BLOCK_GET_HWSEL( block );

   //  Start off with our current db setting.
   //
   if (digital)
   {
      blockp   = getDigitalBlock( client, codec, dir );
      audioDb  = &cap.digital;
   }
   else
   {
      blockp   = getAnalogBlock( client, codec, dir, mux );
      audioDb  = &cap.analog;
   }

   if ( blockp == NULL )
   {
      return -EINVAL;
   }

   db = blockp->db;

   //  Based on whether we are dealing with a list or a range of steps, we
   //  handle this differently.
   //
   if (audioDb->rangeType == HAL_AUDIO_RANGETYPE_FIXED_STEPSIZE)
   {
      //  Based on the direction, calculate the next gain setting.
      //
      db += audioDb->range.fixedStepSize * direction;
      db = max(min(db, audioDb->maxDB), audioDb->minDB);
   }
   else
   {
      int i;

      //  Find where we are in the list.
      //
      for (i = 0; ((i < (audioDb->range.list.numSettings-1)) && (db > audioDb->range.list.dbSetting[i])); i++);

      //  Adjust for direction of change.
      //
      i += direction;

      //  Make sure i is still valid.
      //
      if (i < 0)
      {
         i = 0;
      }
      if (i >= audioDb->range.list.numSettings)
      {
         i = audioDb->range.list.numSettings - 1;
      }

      //  Get the new db value.
      //
      db = audioDb->range.list.dbSetting[i];
   }

   //  Set the new gain.
   //
   if (digital)
   {
      rc = halAudio_setDigGain( client, block, db );
   }
   else
   {
      rc = halAudio_setAnaGain( client, block, db );
   }

   *retGain = db;

   return rc;
}
/****************************************************************************
*
*  halAudio_blockEnable
*
*  Gaines settings are already stored in local structure.  This block enable
*  function will send the local gain settings to the actual hardware.
*  (Note, this is actually a replacement of the old function 
*  "halAudio_setSpkrSource" and "halAudio_setMicSource"
*
***************************************************************************/
int halAudio_blockEnable( int client, HAL_AUDIO_BLOCK block )
{
   audio_block *blockp;
   int          dir, codec, mux;
   int          err = 0;

   HALAUDIO_TRACE( "client=%d block=0x%x\n", client, block );

   codec    = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir      = HAL_AUDIO_BLOCK_GET_DIR( block );
   mux      = HAL_AUDIO_BLOCK_GET_HWSEL( block );
   blockp   = HAL_AUDIO_BLOCK_IS_SIDETONE( block ) ? getSidetoneBlock( client, codec ) : getAnalogBlock( client, codec, dir, mux );

   if ( !blockp )
   {
      return -EINVAL;
   }

   if (((client == gActiveClient) && (gSuperUsers == 0))
   ||  (gMyClients[client].superUser))
   {
      if (blockp->db == HAL_AUDIO_GAIN_SLEEP) 
        blockp->db = HAL_AUDIO_GAIN_MUTE; // set to mute to get out of sleep mode.
      err  = do_SetAnaGain( block, blockp->db );
   }

   if ( !err )
   {
      blockp->enable  = 1;
   }

   return err;
}

/****************************************************************************
*
*  Stateless helper routine to mute block without saving the block
*  disable state. This is primarily needed by releaseControl whereby 
*  a block that is enabled by the user needs to be muted when control 
*  is relinquished.
*
***************************************************************************/
static int do_BlockDisable( int client, HAL_AUDIO_BLOCK block )
{
   int err = 0;

   if (((client == gActiveClient) && (gSuperUsers == 0))
   ||  (gMyClients[client].superUser))
   {
      err  = do_SetAnaGain( block, HAL_AUDIO_GAIN_SLEEP );
   }

   return err;
}

/****************************************************************************
*
*  halAudio_blockDisable
*
*  Disable (send gain sleep) to the hardware blocks.
*
***************************************************************************/
int halAudio_blockDisable( int client, HAL_AUDIO_BLOCK block )
{
   audio_block *blockp;
   int          dir, codec, mux;
   int          err = 0;

   HALAUDIO_TRACE( "client=%d block=0x%x\n", client, block );

   codec    = HAL_AUDIO_BLOCK_GET_CODEC( block );
   dir      = HAL_AUDIO_BLOCK_GET_DIR( block );
   mux      = HAL_AUDIO_BLOCK_GET_HWSEL( block );
   blockp   = HAL_AUDIO_BLOCK_IS_SIDETONE( block ) ? getSidetoneBlock( client, codec ) : getAnalogBlock( client, codec, dir, mux );

   if ( !blockp )
   {
      return -EINVAL;
   }

   err = do_BlockDisable( client, block );
   if ( !err )
   {
      blockp->enable    = 0;
   }

   return err;
}
/****************************************************************************
*
*  halAudio_blockQueryGain
*
*  Query the actual hardware gain of a particular block (ignoring the current active user)
*
***************************************************************************/
int halAudio_blockQueryGain( int client, HAL_AUDIO_BLOCK block )
{
   HAL_AUDIO_CAPABILITIES cap;

   if ((client < 0) || (client >= HAL_AUDIO_MAX_CLIENTS))
   {
      return 0;
   }

   if (!gHalFncs.getCapabilities || gHalFncs.getCapabilities( block, &cap ) < 0)
   {
      return 0;
   }

   return (cap.analog.currentDB);

}
/****************************************************************************
*
*  halAudio_setActiveCodec
*
*  set the active block of the current client for halAudioWrite and other
*  block dependent operations
*
***************************************************************************/
int halAudio_setActiveCodec( int client, HAL_AUDIO_CODEC codecNum )
{
   if( (client < 0) || (client >= HAL_AUDIO_MAX_CLIENTS) || (codecNum >= HAL_AUDIO_CODEC_MAX_NUM) )
   {
      return 0;
   }
   gMyClients[client].activeCodec = codecNum;

   return (1);
}

/****************************************************************************
*
*  halAudio_selectMode
*
*  Used by other kernel level modules to select mode and sampling frequency
*
***************************************************************************/
int halAudio_selectMode( int client, HAL_AUDIO_MODE mode, int freq )
{
   if ((client >= 0) && (client < HAL_AUDIO_MAX_CLIENTS))
   {
      int rc = 0;
      if ( gHalFncs.selectMode )
      {
         if (((client == gActiveClient) && (gSuperUsers == 0)) || (gMyClients[client].superUser))
         {
            rc = gHalFncs.selectMode( mode, freq );
         }
      }
      if ( rc == 0 )
      {
         gMyClients[client].usAudioFreq = freq;
         gMyClients[client].eAudioMode = mode;
      }
      return rc;
   }
   return -1;
}

/****************************************************************************
*
*  halAudio_getMode
*
*  Retrieve current audio mode 
*
***************************************************************************/
HAL_AUDIO_MODE halAudio_getMode( void )
{
   return (gHalFncs.getMode());
}

/****************************************************************************
*
*  Obtain frequencies support
*
***************************************************************************/
int halAudio_getFrequencies( HAL_AUDIO_MODE mode, HAL_AUDIO_FREQUENCIES *freq )
{
   if (gHalFncs.getFrequencies)
   {
      if ( gHalFncs.getFrequencies(mode, freq) )
      {
         printk( KERN_ERR "Get frequencies failed\n" );
         return -1;
      }
   }
   else
   {
      freq->numSettings = 0;
   }
   return 0;
}

/****************************************************************************
*
*  halAudio_getControl
*
*  Used by other kernel level modules to get control of the audio
*
***************************************************************************/
int halAudio_getControl( int client, HAL_AUDIO_MODE mode, int freq )
{
   int i;
   audio_block *blockp;
   halaudio_client *cp;

   HALAUDIO_TRACE( "client=%i mode=%i freq=%i\n", client, mode, freq );

   if ((client < 0) || (client >= HAL_AUDIO_MAX_CLIENTS))
   {
      //  Invalid client number.
      //
      return -EINVAL;
   }
   cp = &gMyClients[client];

   if ( cp->superUser )
   {
      //  There is no concept of "control" for super user apps.  They
      //  don't have their own settings, rather they see and control the
      //  setting of the current non-super user active app.
      //
      return 0;
   }

   if (client == gActiveClient)
   {
      //  Make sure the HAL is set up for our operational mode and frequency.
      //
      if ( halAudio_selectMode( client, mode, freq ))
      {
         printk( KERN_ERR "selectMode with active client failed with mode=%i freq=%i\n", mode, freq );
         return -EINVAL;
      }
      return 0;
   }

   //  Check if there is another client currently active.  If there is, move their number
   //  to the LIFO list of waiting clients.
   //
   if (gActiveClient >= 0)
   {
      gLIFOindex++;

      gWaitingClients[gLIFOindex].client = gActiveClient;
      gWaitingClients[gLIFOindex].freq = gMyClients[gActiveClient].usAudioFreq;
      gWaitingClients[gLIFOindex].mode = gMyClients[gActiveClient].eAudioMode;
   }

   //  Change the active client settings and update the gains, etc.
   //  If an outstanding IO operation is in effect, wait up to 500ms for it to complete.
   //
   gActiveClient = client;
   wait_event_interruptible_timeout( gIoQueue, ( gIoClient == -1 ), 50);

   if ( halAudio_selectMode( client, mode, freq ))
   {
      printk( KERN_ERR "selectMode with new client failed with mode=%i freq=%i\n", mode, freq );
      return -EINVAL;
   }

   //  If there is a thread waiting for this client to get control, wake them up.
   //
   if (cp->blocked)
   {
      cp->blocked = 0;
      up( &cp->sSem );
   }

   //  Restore all block gain settings.
   //  The order of operation is important. The block disable operations must happen before 
   //  the block enable operations to ensure that 'disable' gains do not overwrite 'enable' gains.
   // 
   //  FIXME: should optimize to only operate on blocks supported by registered hardware
   //
   blockp = &cp->analog_block[0][0][0];
   for ( i = 0; i < sizeof(cp->analog_block)/sizeof(*blockp); i++, blockp++ )
   {
      //  Disable blocks
      //
      if ( !blockp->enable )
      {
         do_BlockDisable( client, blockp->id );
      }
   }
   blockp = &cp->digital_block[0][0];
   for ( i = 0; i < sizeof(cp->digital_block)/sizeof(*blockp); i++, blockp++ )
   {
      //  Restore digital gains
      //
      halAudio_setDigGain( client, blockp->id, blockp->db );     
   }
   blockp = &cp->analog_block[0][0][0];
   for ( i = 0; i < sizeof(cp->analog_block)/sizeof(*blockp); i++, blockp++ )
   {
      //  Enable blocks
      //
      if ( blockp->enable )
      {
         halAudio_blockEnable( client, blockp->id );
      }
   }
   blockp = &cp->sidetone_block[0];
   for ( i = 0; i < sizeof(cp->sidetone_block)/sizeof(*blockp); i++, blockp++ )
   {
      //  Enable sidetone
      //
      if ( blockp->enable )
      {
         halAudio_blockEnable( client, blockp->id );
      }
      else
      {
         do_BlockDisable( client, blockp->id );
      }
   }

   return 0;
}

/****************************************************************************
*
*  halAudio_releaseControl
*
*  Used by other kernel level modules to release control of the audio
*
***************************************************************************/
int halAudio_releaseControl( int client )
{
   int j;
   int waiting_client = -1;
   int freq = 0;
   HAL_AUDIO_MODE mode = HAL_AUDIO_MODE_NONE;
   audio_block *blockp;
   halaudio_client *cp;

   HALAUDIO_TRACE( "client=%i\n", client );

   if ((client < 0) || (client >= HAL_AUDIO_MAX_CLIENTS))
   {
      //  Invalid client number.
      //
      return -EINVAL;
   }
   cp = &gMyClients[client];

   if (cp->superUser)
   {
      //  There is no concept of "control" for super user apps.  They
      //  don't have their own settings, rather they see and control the
      //  setting of the current non-super user active app.
      //
      return 0;
   }

   cp->usAudioFreq = 0;
   cp->eAudioMode = HAL_AUDIO_MODE_NONE;

   //  If we are not the active client, we just need to ensure we are not in the 
   //  waiting list.
   //
   if (client != gActiveClient)
   {

      for (j = 0; j <= gLIFOindex; j++)
      {
         if (client == gWaitingClients[j].client)
         {
            memmove( &gWaitingClients[j], &gWaitingClients[j+1], 
                  sizeof(gWaitingClients[0]) * (gLIFOindex - j) );
            gLIFOindex--;
            return 0;
         }
      }
   }
   else
   {
      //  Check if there are any other clients waiting for the audio channel.
      //
      while (gLIFOindex >= 0)
      {
         if ((gWaitingClients[gLIFOindex].client >= 0) && (gMyClients[gWaitingClients[gLIFOindex].client].used))
         {
            //  This client is waiting.
            //
            waiting_client = gWaitingClients[gLIFOindex].client;
            freq  = gWaitingClients[gLIFOindex].freq;
            mode  = gWaitingClients[gLIFOindex].mode;
            gWaitingClients[gLIFOindex].client  = -1;
            gWaitingClients[gLIFOindex].freq    = 0;
            gWaitingClients[gLIFOindex].mode    = HAL_AUDIO_MODE_NONE;
            gLIFOindex--;
            break;
         }
         gLIFOindex--;
      }

      //  Disable the enabled analog and sidetone blocks
      //
      blockp = &cp->analog_block[0][0][0];
      for ( j = 0; j < sizeof(cp->analog_block)/sizeof(*blockp); j++, blockp++ )
      {
         if ( blockp->enable )
         {
            do_BlockDisable( client, blockp->id );
         }
      }
      blockp = &cp->sidetone_block[0];
      for ( j = 0; j < sizeof(cp->sidetone_block)/sizeof(*blockp); j++, blockp++ )
      {
         if ( blockp->enable )
         {
            do_BlockDisable( client, blockp->id );
         }
      }

      if ( waiting_client < 0 )
      {
         //  Nope, there are no other clients.
         //
         if ( halAudio_selectMode( client, HAL_AUDIO_MODE_NONE, 0 ))
         {
            printk( KERN_ERR "selectMode call to HAL failed\n" );
            return -1;
         }
         gActiveClient = -1;
         return 0;
      }

      //
      //  Remove ourselves as the active client.
      //
      gActiveClient = -1;

      //  Basically, we just call halAudio_getControl for the other client.
      //
      return halAudio_getControl( waiting_client, mode, freq );
   }

   return 0;
}


/****************************************************************************
*
*  halAudio_getGains
*
*  This routine is called by other drivers to see the set of gains
*  a given audio block has.
*
***************************************************************************/
int halAudio_getGains(HAL_AUDIO_BLOCK block, HAL_AUDIO_CAPABILITIES *cap)
{
   if (gHalFncs.getCapabilities)
   {
      return ( gHalFncs.getCapabilities(block, cap) );
   }
   return -1;
}

/****************************************************************************
*
*  Helper routine to check whether frequency is supported in 
*  frequency set. 
*
*  RETURN: 1 if found, 0 otherwise.
*
***************************************************************************/
static int validateFreq( HAL_AUDIO_FREQUENCIES *setp, int freq )
{
   int i;
   for ( i = 0; i < setp->numSettings; i++ )
   {
      if ( freq == setp->freqSetting[i] )
      {
         return 1;
      }
   }
   return 0;
}

/****************************************************************************
*
*  halAudio_registerAudio
*
*  This routine is called by the HAL driver when it is loaded to give us
*  a table of function pointers to the functions we need from the HAL.
*
***************************************************************************/
int halAudio_registerAudio(HAL_AUDIO_FNCS *audiop)
{
   //  Take a copy of the function pointers so we are not relying on another
   //  driver's memory space.
   //
   memcpy( (char *)&gHalFncs, (char *)audiop, sizeof(gHalFncs) );

   if (gHalFncs.getFrequencies)
   {
      if ( gHalFncs.getFrequencies(HAL_AUDIO_MODE_LDX, &gHalLdxFreqs) )
      {
         printk( KERN_ERR "Get LDX frequencies failed\n" );
         return -1;
      }
      if ( gHalFncs.getFrequencies(HAL_AUDIO_MODE_ALSA, &gHalAlsaFreqs) )
      {
         printk( KERN_ERR "Get ALSA frequencies failed\n" );
         return -1;
      }
      if ( gHalFncs.getFrequencies(HAL_AUDIO_MODE_POLYRINGER, &gHalPolyFreqs) )
      {
         printk( KERN_ERR "Get polyringer frequencies failed\n" );
         return -1;
      }
   }

   return 0;
}

/****************************************************************************
*
*  halAudio_widebandCheck
*
*  This routine returns 1 if LDX is capable of 16kHz wideband, otherwise 0
*
***************************************************************************/
int halAudio_widebandCheck( void )
{
   return validateFreq( &gHalLdxFreqs, WIDEBAND_THRESHOLD );
}

/****************************************************************************
*
*  halAudio_customHardware
*
*  This routine is used to implement custom hardware configurations.
*  Returns 0 on success, other -1 or other codes if unsupported or
*  failure.
*
***************************************************************************/
int halAudio_customHardware( int cmd, void *data )
{
   int error = -1;

   HALAUDIO_TRACE( "cmd=0x%x\n", cmd );

   if (gHalFncs.customHardware)
   {
      error = gHalFncs.customHardware(cmd, data);
   }
   return error;
}

/****************************************************************************
*
*  halAudio_queryPower
*
*  This routine is used to allow for querying of power directly
*
***************************************************************************/

void halAudio_queryPower( HAL_AUDIO_POWER_LEVEL *level, int *downRequestPending )
{
   //  Super user apps just see what the currently active app does.
   //
   if (gHalFncs.queryPower)
   {
      gHalFncs.queryPower( level, downRequestPending );
   }
   else
   {
      //  Cannot directly query power level, then super user apps just
      //  sees what the currently active app does.
      //
      level = (HAL_AUDIO_POWER_LEVEL*)gMyClients[(gActiveClient < 0) ? 0 : gActiveClient].ePowerLevel;
   }
}

/****************************************************************************
*
*  halAudio_getEquParms
*
*  This routine is used to get equalizer parameters
*  Returns 0 on success, other -1 or other codes if unsupported or 
*  failure.
*
***************************************************************************/
int halAudio_getEquParms( HALAUDIO_EQU_PARMS *parms )
{
   int error = -1;

   HALAUDIO_TRACE( "codec=%i\n", parms->codec );

   if ( gHalFncs.getEquParms )
   {
      error = gHalFncs.getEquParms( parms );
   }
   return error;
}

/****************************************************************************
*
*  halAudio_setEquParms
*
*  This routine is used to set equalizer parameters
*  Returns 0 on success, other -1 or other codes if unsupported or 
*  failure.
*
***************************************************************************/
int halAudio_setEquParms( HALAUDIO_EQU_PARMS *parms )
{
   int error = -1;

   HALAUDIO_TRACE( "codec=%i\n", parms->codec );

   if ( gHalFncs.setEquParms )
   {
      error = gHalFncs.setEquParms( parms );
   }
   return error;
}

/****************************************************************************
*
*  halAudio_setSuperUser
*
*  This routine is set the client in super user mode
*  Input:
*       client  the client to be set
*       set     1 to set, 0 to clear
*       mode    halaudio mode to set to super user
*  Returns 0 on success, -1 on failure.
*
***************************************************************************/
int halAudio_setSuperUser(int client, int set, HAL_AUDIO_MODE mode)
{
    int rc = -1;

    if (set)
    {
        if (!gMyClients[client].superUser)
        {
            int freq;
            gMyClients[client].superUser = 1;
            gSuperUsers++;

            //  The superuser updates the mode of operation which cannot be later
            //  changed until the superuser de-registers.
            //
            freq = getFreq( mode, client );
            rc = halAudio_selectMode( client, mode, freq );
        }
    }
    else if (gMyClients[client].superUser)
    {
        gMyClients[client].superUser = 0;
        gSuperUsers--;
    }
    return rc;
 }


/****************************************************************************
*
*  halAudio_getCodecInfo
*
*  This routine is used to information about the HW codec being used.
*  Returns 0 on success, other -1 or other codes if unsupported or 
*  failure.
*
***************************************************************************/
int halAudio_getCodecInfo( HALAUDIO_CODEC_INFO *codecInfo )
{
   int error = -1;

   HALAUDIO_TRACE( "codec=%i\n", codecInfo->codecNum );

   if ( gHalFncs.getCodecInfo )
   {
      error = gHalFncs.getCodecInfo( codecInfo );
   }
   return error;
}

/****************************************************************************
*
*  halAudio_MicSel
*
*  This routine is used to select the microphone input
*
***************************************************************************/
int halAudio_MicSel( HAL_AUDIO_BLOCK block )
{
   int rc = 0;

   HALAUDIO_TRACE( "block=0x%x\n", block );

   if ( gHalFncs.micSel )
   {
      rc = gHalFncs.micSel( block );
   }
   return rc;
}

/****************************************************************************
*
*  halAudio_registerAddOnModule
*
*  This routine is used to append additional external audio interfaces 
*  to the primary interface. For example, HSS/PCM and I2S interfaces may
*  be added via this routine. These external interfaces will be started
*  and stopped in sync with the master module. 
*  
*  Returns 0 on success, otherwise -1 or other negative codes if unsupported 
*  or failure.
*
***************************************************************************/
int halAudio_registerAddOnModule( HAL_AUDIO_ADDONMODULE * addon )
{
   int error = -1;
   if ( gHalFncs.registerAddOn )
   {
      error = gHalFncs.registerAddOn( addon );
   }
   return error;
}

/****************************************************************************
*
*  halAudio_registerCsxIoPoint
*
*  This routine is used to register CSX callback functions to inject/capture
*  samples.
*  Returns 0 on success, other -1 or other codes if unsupported or 
*  failure.
*
***************************************************************************/
int halAudio_registerCsxIoPoint( HAL_AUDIO_CODEC codec, HAL_AUDIO_DIR dir, CSX_IO_POINT_FNCS *fncp, void *data )
{
   int error = -1;
   if( gHalFncs.registerCsxIoPoint )
   {
      error = gHalFncs.registerCsxIoPoint( codec, dir, fncp, data );
   }
   return error;
}

/****************************************************************************
*
*  halAudio_RegisterPlatform
*
*  This routine is used to register platform specific callbacks
*
*  Returns 0 on success, otherwise -1 or other codes if unsupported or 
*  failure.
*
***************************************************************************/
int halAudio_RegisterPlatform( const HAL_AUDIO_PLATFORM_OPS *ops )
{
   if ( down_interruptible( &gPlatform.mutex ) < 0 )
   {
      //  Failed on the semaphore. Likely interrupted
      //
      return -EAGAIN;
   }
   memcpy( &gPlatform.ops, ops, sizeof(gPlatform.ops) );
   up( &gPlatform.mutex );

   return 0;
}

/***************************************************************************/
/**
*  SysCtl data structures
*/

static struct ctl_table gSysCtlI2S[] = {
   {
      .ctl_name      = 100,
      .procname      = "dbgprint",
      .data          = &halaudio_gDbgPrint,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name   = CTL_BCM_HALAUDIO,
      .procname   = "halaudio",
      .mode       = 0555,
      .child      = gSysCtlI2S
   },
   {}
};

static struct ctl_table_header *gSysCtlHeader;

/***************************************************************************/
/**
*  Initialize SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void sysCtlInit( void )
{

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif
}

/***************************************************************************/
/**
*  Cleanup SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void sysCtlExit( void )
{
   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
}

/****************************************************************************/

module_init(halAudio_init);
module_exit(halAudio_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("HALAUDIO Hook Driver");
EXPORT_SYMBOL (halAudio_writeDev);
EXPORT_SYMBOL (halAudio_readDev);
EXPORT_SYMBOL (halAudio_getAnaGain);
EXPORT_SYMBOL (halAudio_setAnaGain);
EXPORT_SYMBOL (halAudio_alterGain);
EXPORT_SYMBOL (halAudio_getDigGain);
EXPORT_SYMBOL (halAudio_setDigGain);
EXPORT_SYMBOL (halAudio_blockEnable);
EXPORT_SYMBOL (halAudio_blockDisable);
EXPORT_SYMBOL (halAudio_blockQueryGain);
EXPORT_SYMBOL (halAudio_setActiveCodec);
EXPORT_SYMBOL (halAudio_allocate_client);
EXPORT_SYMBOL (halAudio_free_client);
EXPORT_SYMBOL (halAudio_enableAudio);
EXPORT_SYMBOL (halAudio_disableAudio);
EXPORT_SYMBOL (halAudio_getMode);
EXPORT_SYMBOL (halAudio_selectMode);
EXPORT_SYMBOL (halAudio_getControl);
EXPORT_SYMBOL (halAudio_releaseControl);
EXPORT_SYMBOL (halAudio_registerAudio);
EXPORT_SYMBOL (halAudio_widebandCheck);
EXPORT_SYMBOL (halAudio_getGains);
EXPORT_SYMBOL (halAudioExternalGetParams);
EXPORT_SYMBOL (halAudio_customHardware);
EXPORT_SYMBOL (halAudio_queryPower);
EXPORT_SYMBOL (halAudio_getFrequencies);
EXPORT_SYMBOL (halAudio_getEquParms);
EXPORT_SYMBOL (halAudio_setEquParms);
EXPORT_SYMBOL (halAudio_getCodecInfo);
EXPORT_SYMBOL (halAudio_MicSel);
EXPORT_SYMBOL (halaudio_gDbgPrint);
EXPORT_SYMBOL (halAudio_registerCsxIoPoint);
//EXPORT_SYMBOL (halAudio_RegisterPlatform);
EXPORT_SYMBOL (halAudio_setSuperUser);
