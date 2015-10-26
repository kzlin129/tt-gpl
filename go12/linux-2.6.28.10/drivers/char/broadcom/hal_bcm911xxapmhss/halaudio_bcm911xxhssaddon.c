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
*  @file    halaudio_bcm911xxhssaddon.c
*
*  @brief   HAL Audio routines for the BCM911xx HSS addon module. An example board 
*           that uses this addon module is the BCM91103EVM.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>
#include <stddef.h>                 /* Needed for NULL */

#include <linux/broadcom/halaudio.h>/* HALAUDIO API */
#include <linux/broadcom/halaudio_mixer.h> /* HALAUDIO mixer API */
#include <linux/broadcom/knllog.h>
#include <linux/dmapool.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>

#include <asm/broadcom/bcm1103/bcm1103.h>
#include <dmal.h>
#include <hssl.h>

#include <linux/broadcom/iodump.h>  /* For debugging */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
#define HSS_NUM_CHANNEL 2
#define HSS_NUM_BUF     2     /* Number of buffers per Rx or Tx (2=double buffered) */
#define hssToDmaCh(x)      (x + 7)
#define HSS_INTERVAL    5     /* the HSS DMA is serviced every 5 ms (from internal driver) */

int halAudioHSSCodecOffset = -1;

extern int bcm1103HsslDrvInit( HSSL_DRV **pDrv );
extern int bcm1103DmalDrvInit( DMAL_DRV **drvp );

/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Functions ------------------------------------------------ */
#define  HSS_NUM_MEDIA_STREAM 4  /* maximum number of HSS media stream, 
                                  * each HSS channel can support 2 media stream when 
                                  * configured to run at dual timeslot mode */

/* HSS module function (for signal processing) definitions */
static int halAudioHss_init( void );
static int halAudioHss_exit( void );
static int halAudioHss_enable( void );
static int halAudioHss_disable( void );
static int halAudioHss_ingress( void );
static int halAudioHss_egress( void );
static int halAudioHss_codecInfo( HALAUDIO_CODEC_INFO *codecInfo );
static int halAudioHss_setAnalogGain( HAL_AUDIO_BLOCK block, short db );
static int halAudioHss_setDigitalGain( HAL_AUDIO_BLOCK block, short db );
static int halAudioHSS_getEgressPtr( HAL_AUDIO_CODEC codecNum, short ** ptr, int * size );

static HAL_AUDIO_ADDONMODULE_FNC halAudioHssModule_fncp = 
{
   halAudioHss_init,
   halAudioHss_exit,
   halAudioHss_enable,
   halAudioHss_disable,
   halAudioHss_ingress,
   halAudioHss_egress,
   halAudioHss_codecInfo,
   halAudioHSS_getEgressPtr,
   halAudioHss_setAnalogGain,
   halAudioHss_setDigitalGain,
};

HAL_AUDIO_ADDONMODULE halAudioHssModule = 
{
   "BCM11xxHSS",
   HSS_NUM_MEDIA_STREAM,
   &halAudioHssModule_fncp
};

/* HSS channel configured instance memory */
typedef struct
{
   HSSL_DRV *drvp;

   struct
   {
      int sampleFreq;   /* sampling frequency */
      int sampleSize;   /* sample width (byte 1 or word 2) */
      int packetLen;    /* HSS DMA packet length */
      int numslot;      /* number of time slots supported */

   } cfg[HSS_NUM_CHANNEL];

} HSS_CBLK;


HSS_CBLK hssCblk;

/* HSS modules using HAL Audio mixer */
static int hssMixerPorts[HSS_NUM_MEDIA_STREAM];
char * hssMixerIngressBuf[HSS_NUM_MEDIA_STREAM];
char * hssMixerEgressBuf[HSS_NUM_MEDIA_STREAM];

static char * hssHalMixerCbk_outputPtr( int portIdx, int numBytes );
static char * hssHalMixerCbk_inputPtr( int portIdx, int numBytes );

static HAL_MIXER_CALLBACK_T gHalMixerHssCallback = 
{
   hssHalMixerCbk_outputPtr,
   hssHalMixerCbk_inputPtr,
   NULL,
   NULL
};

static void hssRegisterMixerPorts( void );

/* array to keep track of packet length going to the mixer for each media stream */
/* note this number can differ as compared to the HSS DMA packet length */
int hssMediaPacketLen[HSS_NUM_MEDIA_STREAM];

/* DMA related */
static int dmaInit( void );
static int dmaStart( int ch );
static int dmaStop( int ch );
int allocRxBufReset( int ch );
void *allocRxBufp( int len );

static int mallocInit( void );
static void mallocExit( void );


/*
*  Freq (kHz)  Interval (ms)  Samples  Bytes
*  ----------  -------------  -------  -----
*  16          5              80       160
*  48          5              240      480
*
*  The largest amount of space required for any buffer is 480bytes, so set
*  the allocation pool size to be 1024bytes, which is more than enough to
*  handle sample data and DMA descriptors
*/
/* Note, allocating 1024 bytes as DMA buffer has one advantage that when 2 
 * media streams are supported in each HSS channels, and the media samples for each
 * stream are interleaved, a larger buffer provide more room to prepare the media 
 * stream *in place* */
#define MALLOC_POOL_SIZE      1024

typedef enum
{
   DMA_DIR_RX,
   DMA_DIR_TX,
   DMA_DIR_NUM

} DMA_DIR;

typedef struct
{
   char *bufp;
   dma_addr_t handle;

} MALLOC_DMA;

typedef struct
{
   MALLOC_DMA buf[HSS_NUM_CHANNEL][DMA_DIR_NUM][HSS_NUM_BUF]; /* DMA buffers */
   MALLOC_DMA bufZero;
   MALLOC_DMA bufScratch;
   MALLOC_DMA dma[HSS_NUM_CHANNEL][DMA_DIR_NUM];              /* DMA descriptors */
   struct dma_pool *poolp;

} MALLOC_CBLK;

MALLOC_CBLK mallocCblk;
static int allocRxBufCh;
static int allocRxBufIndex;

/* macro function to toggle the DMA double buffer usage */
#define BUF_INDEX_NEXT(i)     ((i) ^= 1)

static int hssDmaBufferIdx;

/**
* DMA related allocated memory access macros 
*/
#define getDmaDescRxp(ch)        mallocCblk.dma[ch][DMA_DIR_RX].bufp
#define getDmaDescTxp(ch)        mallocCblk.dma[ch][DMA_DIR_TX].bufp
#define getDmaRxBufp(ch,index)   mallocCblk.buf[ch][DMA_DIR_RX][index].bufp
#define getDmaTxBufp(ch,index)   mallocCblk.buf[ch][DMA_DIR_TX][index].bufp
#define getZeroBufp()            mallocCblk.bufZero.bufp

typedef struct
{
   int ch;
   int descLen;
   char *descBufp;
   int burstLen;
} DMA_CH;

typedef struct
{
   DMA_CH rx;
   DMA_CH tx;
}  DMA_CFG;

/** 
* DMA control block structure for all DMA channels. The convention
* used is that APM DMA channel precede HSS DMA channels.
*/
typedef struct
{
   DMAL_DRV *drvp;
   DMA_CFG cfg[HSS_NUM_CHANNEL];
} DMA_CBLK;

DMA_CBLK dmaCblk;

/**
* DMA definitions 
*/
#define DMA_RX_DESC_NUM       HSS_NUM_BUF
#define DMA_RX_DESC_OPT       0x00000000

#define DMA_TX_DESC_NUM       HSS_NUM_BUF
#define DMA_TX_DESC_OPT       0x00000000

/*
*  Memory management
*/

#define CACHE_LINE_SIZE       16
#define CACHE_LINE_MASK       (CACHE_LINE_SIZE - 1)
#define CACHE_ALIGN(n)        (((n) + CACHE_LINE_MASK) & ~CACHE_LINE_MASK)

/* debug information */
#define AUDIO_HSS_PROC_NAME      "audioInternalHss_addon"
#define AUDIO_HW_HSS_PROC_NAME   "audioInternalHwHss_addon"
static int numIngressCalled;
static int numEgressCalled;
static int numIngressError[HSS_NUM_CHANNEL];
static int numEgressError[HSS_NUM_CHANNEL];
/* proc entry returning the number of DMA error for the HSS channels */
static int halAudioAddonHssReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );
/* proc entry returning the HSS configuraion */
static int halAudioAddonHwHssReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

/*************************************************/

/***************************************************************************/
/**
*  halAudioHss_init
*     initialize and configure the HSS channel and the related media streams
*
*  @return 
*/
static int halAudioHss_init( void )
{
   HSSL_DRV * hssDrvp;

   HALAUDIO_TRACE("Hss addon module init called \n");

   memset( hssMediaPacketLen, 0, sizeof(int) * HSS_NUM_MEDIA_STREAM );
   memset( hssMixerPorts, -1, sizeof( int ) * HSS_NUM_MEDIA_STREAM );

   /* enable both HSS channels on the 1103 */
   bcm1103mmr->chipCtl.blkEnables |= MMR1103_CHIPCTL_BLKEN_EHSS0 | MMR1103_CHIPCTL_BLKEN_EHSS1;

   /* Assign lower driver */
   bcm1103HsslDrvInit( &hssCblk.drvp );
   hssDrvp = hssCblk.drvp;
   
   /* config the HSS0 channel */
   {
      hsslReset( hssDrvp,0 );               /* Reset HSS channel */
      hsslModeSet( hssDrvp,0, HSSL_MODE_TDM_MASTER ); /* Master mode - do not change */
      hsslClkSet( hssDrvp,0, 2048 );        /* Set clock */
      hsslClkInvertEnable( hssDrvp,0 );     /* Enabled inverted clock */
      hsslLsbFirstDisable( hssDrvp,0 );     /* Select MSB first */

      hsslFsLongDisable( hssDrvp,0);        /* Long framesync */
      hsslFsInvertDisable( hssDrvp,0);      /* Framesync inversion */
      hsslFsFallingEnable( hssDrvp,0);      /* Edge clocking */
      hsslFsOffsetSet( hssDrvp,0, 1 );      /* Clock offset */

      hsslTimeslotSet( hssDrvp,0, 0 );      /* Time slot to run, 0 indexed */
      hsslTimeslotNumSet( hssDrvp,0, 1 );   /* Number of time slots */

      hsslSampleFreqSet( hssDrvp,0, 8000 );  /* Frame syn frequency */
      hsslSampleSizeSet( hssDrvp,0, 2 );     /* number of byte per sample */

      hssCblk.cfg[0].sampleFreq = 8000;
      hssCblk.cfg[0].sampleSize = 2;
      hssCblk.cfg[0].numslot = 1;

      /* HSS channel 0 support 1 media stream, running at 8kHz */
      hssMediaPacketLen[0] = 80;
      /* define another media stream here if HSS0 support more than 1 media streams */
   }
   /* config the HSS1 channel */
   {
      hsslReset( hssDrvp,1 );               /* Reset HSS channel */
      hsslModeSet( hssDrvp,1, HSSL_MODE_TDM_MASTER ); /* Master mode - do not change */
      hsslClkSet( hssDrvp,1, 2048 );        /* Set clock */
      hsslClkInvertEnable( hssDrvp,1 );     /* Enabled inverted clock */
      hsslLsbFirstDisable( hssDrvp,1 );     /* Select MSB first */

      hsslFsLongDisable( hssDrvp,1);        /* Long framesync */
      hsslFsInvertDisable( hssDrvp,1);      /* Framesync inversion */
      hsslFsFallingEnable( hssDrvp,1);      /* Edge clocking */
      hsslFsOffsetSet( hssDrvp,1, 1 );      /* Clock offset */

      hsslTimeslotSet( hssDrvp,1, 0 );      /* Time slot to run, 0 indexed */
      hsslTimeslotNumSet( hssDrvp,1, 2 );   /* Number of time slots */

      hsslSampleFreqSet( hssDrvp,1, 8000 );  /* Frame syn frequency */
      hsslSampleSizeSet( hssDrvp,1, 2);      /* Number of byte per sample */

      hssCblk.cfg[1].sampleFreq = 8000;
      hssCblk.cfg[1].sampleSize = 2;
      hssCblk.cfg[1].numslot = 2;

      /* HSS channel 1 support 1 media stream, running at 8kHz */
      hssMediaPacketLen[1] = 160;
      /* define another media stream here if HSS0 support more than 1 media streams */      
   }
   /* registering mixer ports for the media streams */
   /* we can register up to 4 mixer ports since up to 4 media streams can be supported */
   /* only 2 mixer ports are being registered now since this example HSS code only support 2 media streams */
   hssRegisterMixerPorts();

   /* initialize dma blocks */
   mallocInit();
   dmaInit();

   /* create proc entry for debug stats */
   create_proc_read_entry( AUDIO_HSS_PROC_NAME, 0, NULL, halAudioAddonHssReadProc,
                           NULL );
   create_proc_read_entry( AUDIO_HW_HSS_PROC_NAME, 0, NULL,
                           halAudioAddonHwHssReadProc, NULL );

   /* clear statistics variables */
   numIngressCalled = 0;
   numEgressCalled = 0;
   numIngressError[0] = 0;
   numIngressError[1] = 0;
   numEgressError[0] = 0;
   numEgressError[1] = 0;
      
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_exit
*
*  @return 
*/
static int halAudioHss_exit( void )
{
   int i;
   HAL_MIXER_PORT_INFO mixerPort;

   HALAUDIO_TRACE("exit of HSS add on module\n");

   mixerPort.usrid = HAL_MIXER_USER_HAL;

   /* de-register the halMixer ports (register in the init function) */
   for( i=0;i < HSS_NUM_MEDIA_STREAM; i++ )
   {
      mixerPort.portIdx = i + halAudioHSSCodecOffset;
      if( hssMixerPorts[i] != -1 )
      {
         halMixer_deregisterPort( hssMixerPorts[i], &mixerPort );
         hssMixerPorts[i] = -1;
      }
   }
   halAudioHSSCodecOffset = -1;
   halAudioHss_disable();
   mallocExit();

   remove_proc_entry( AUDIO_HSS_PROC_NAME, NULL );
   remove_proc_entry( AUDIO_HW_HSS_PROC_NAME, NULL );
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_ingress
*     run the DMA on the ingress direction (from the mic)
*
*  NOTE : this function is being called within the context of the audio ISR
*  @return 
*/
static int halAudioHss_ingress( void )
{
   int rc, i, len;
   DMAL_DRV *drvp;
   DMA_CFG *cfgp;
   char * dmaBufp;

   drvp = dmaCblk.drvp;
   numIngressCalled++;
   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
      cfgp = &dmaCblk.cfg[i];
      /* receive ingress samples from the DMA */
      rc = dmalRx( drvp, cfgp->rx.ch, &dmaBufp, &len );

      if ( unlikely( (rc <= 0) || (len != hssCblk.cfg[i].packetLen) ) )
      {
         HALAUDIO_TRACE( "HSS Ingress (Rx) DMA ch=%i data unavailable %d", i, len );

         /* No data read, so use a zero buffer to keep things
         * happy.  The debug statistics can be queried in /proc to
         * determine if any errors have occurred.
         */
         dmaBufp = getZeroBufp();
         numIngressError[i]++;
      }
      /* Update Rx DMA for next incoming frame */
      dmalRxUpdate( drvp, cfgp->rx.ch, NULL, 0, 1 );

#ifdef CONFIG_BCM_IODUMP_SUPPORT
      /* Capture ingress samples here. Convert number of 16-bit samples into number of bytes. */
      iodump_write( i+halAudioHSSCodecOffset, IODUMP_TDM_INGRESS_WB, hssCblk.cfg[i].packetLen, dmaBufp );
#endif

       /* handle dma samples, convert it to media streams */
      /* since only 1 media stream is supported each HSS channel, and the DMA samples
       * can be used directly as media samples, the mixer can copy directly from the DMA buffer */
      hssMixerIngressBuf[i] = dmaBufp;

#if 0
      /* this shows where we should split the interleaved HSS DMA samples into 2 media stream is necessary */
      {
         int k;
         short * srcp = (short *)dmaBufp;
         short * ptr1 = (short *)dmaBufp;
         short * ptr2 = (short *)(dmaBufp + MALLOC_POOL_SIZE/2);
         hssMixerIngressBuf[i+1] = (char *)ptr2;
         for( k=0; k < (hssCblk.cfg[i].packetLen / 4); k++ )
         {
            ptr1[k] = *srcp++;
            ptr2[k] = *srcp++;
         }
      }
#endif

   }

   /* clear egress DMA buffer for the egress samples */
   /* DMA buffers needs to be cleared before writing to it, since we know that ingress is always 
    * called before the samples are copied from the other sources and the egress DMA, this is the best time to
    * clear the egress DMA buffer, though this is an ingress routine */
   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
      hssMixerEgressBuf[i] = getDmaTxBufp( i, hssDmaBufferIdx  );
      memset( hssMixerEgressBuf[i], 0, hssCblk.cfg[i].packetLen );
      /* when 2 media streams are supported in an HSS channel, the second half of the DMA buffer
       * can be used as well */
#if 0
      {
         hssMixerEgressBuf[i+1] = hssMixerEgressBuf[i] + (MALLOC_POOL_SIZE/2);
         memset( hssMixerEgressBuf[i+1], 0, hssMediaPacketLen[2] );
      }
#endif
   }

   BUF_INDEX_NEXT( hssDmaBufferIdx );
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_egress
*     run the DMA on the egress direction (to the speaker)
*
*  NOTE : this function is being called within the context of the audio ISR
*  @return 
*/

static int halAudioHss_egress( void )
{
   int i, len;
   DMA_CFG *cfgp;
   char * dmaBufp;
   numEgressCalled++;
   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
#if 0
      /* Note this piece of code that commented out is for forming the DMA samples
       * from 2 media streams, where the media samples should be interleaved */
      /* convert media samples to dma samples */
      {
         int k;
         short * ptr1 = (short *)hssMixerEgressBuf[i];
         short * ptr2 = (short *)hssMixerEgressBuf[i+1];

         for( k = ((hssCblk.cfg[i].packetLen / 4)-1); k >= 0; k-- )
         {
            ptr1[2*k] = ptr1[k];
            ptr1[2*k+1] = ptr2[k];
         }
      }
#endif
      /* since the DMA samples are directly being used as media samples, no change is necessary */
      dmaBufp = hssMixerEgressBuf[i];

      /* Write data to Tx DMA */
      cfgp = &dmaCblk.cfg[i];
#ifdef CONFIG_BCM_IODUMP_SUPPORT
      iodump_write( i+halAudioHSSCodecOffset, IODUMP_TDM_EGRESS_WB, hssCblk.cfg[i].packetLen, dmaBufp );
#endif
      
      len = dmalTx( dmaCblk.drvp, cfgp->tx.ch, dmaBufp, hssCblk.cfg[i].packetLen );
      if( unlikely( len != hssCblk.cfg[i].packetLen ) )
      {
         KNLLOG( "HSS Unable to write Tx buffer for codec %i, len %i\n", i, len );
         numEgressError[i]++;
      }
   }
   
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_enable
*     disable the HSS channels
*
*  NOTE : this function is being called when the interrupt has been disabled
*         (by the HAL Audio internal driver)
*  @return 
*/

static int halAudioHss_enable( void )
{
   int i;
   HSSL_DRV *drvp = hssCblk.drvp;
   HALAUDIO_TRACE("calling hss enable\n");

   /* clear DMA buffer index */
   hssDmaBufferIdx = 0;

   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
      dmaStart( i );
      hsslEnable( drvp, i );
      hsslDmaEnable( drvp, i );
   }
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_disable
*     disable the HSS channels
*
*  NOTE : this function is being called when the interrupt has been disabled
*         (by the HAL Audio internal driver)
*  @return 
*/
static int halAudioHss_disable( void )
{
   int i;
   HSSL_DRV *drvp = hssCblk.drvp;
   HALAUDIO_TRACE( "HSS disable\n");

   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
      hsslDmaDisable( drvp, i );
      hsslDisable( drvp, i );
      dmaStop( i );
   }
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_getEgressPtr
*     return the pointer to the egress buffer of an HSS channel, with the size of the buffer
*
*  NOTE : this function is being called in the context of an interrupt
*         (by the HAL Audio internal driver)
*  @return 
*/
static int halAudioHSS_getEgressPtr( HAL_AUDIO_CODEC codecNum, short ** ptr, int * size )
{
   int result = -1;

   if( (codecNum >= halAudioHSSCodecOffset) &&
       (codecNum < (halAudioHSSCodecOffset + HSS_NUM_MEDIA_STREAM) ) )
   {
      *ptr = (short *)hssMixerEgressBuf[codecNum - halAudioHSSCodecOffset];
      *size = hssMediaPacketLen[codecNum - halAudioHSSCodecOffset];
      if( *size )
      {
         result = 0;
      }
   }
   return( result );
}
/***************************************************************************/
/**
*  halAudioHss_codecInfo
*     HAL Audio codec information supported in the HSS module
*
*  @return 
*/

static int halAudioHss_codecInfo( HALAUDIO_CODEC_INFO *codecInfo )
{
   int codecNum = codecInfo->codecNum;
   int result = -1;
   int hssMediaStreamNum;
   if( (codecNum >= halAudioHSSCodecOffset) && 
       (codecNum < (halAudioHSSCodecOffset + HSS_NUM_MEDIA_STREAM)) )
   {
      hssMediaStreamNum = codecNum - halAudioHSSCodecOffset;
      /* we only support 2 media streams (HAL Audio codecs) in this HSS modules */
      switch( hssMediaStreamNum )
      {
         case 0:
            codecInfo->external = 1;
            codecInfo->frequency = 8000;
            codecInfo->sampleSize = 2;
            result = 0;
            break;
         case 1:
            codecInfo->external = 1;
            codecInfo->frequency = 16000;
            codecInfo->sampleSize = 2;
            result = 0;
            break;
      }
   }
   HALAUDIO_TRACE("getting HAL HSS codecInfo %d result %d\n", codecNum, result );
   return result;
}

/***************************************************************************/
/**
*  halAudioHss_setAnalogGain
*     set analog gain for the HSS module
*
*  @return 
*/
static int halAudioHss_setAnalogGain( HAL_AUDIO_BLOCK block, short db )
{
   /* HSS does not support hardware analog gain block, just return here */
   (void) block;
   (void)db;
   HALAUDIO_TRACE( "block %d db %d\n", block, db);

   /* note: 0 is returned instead of -1 (error) just in case the app is not 
    * handling the error correctly */
   return 0;
}

/***************************************************************************/
/**
*  halAudioHss_setDigitalGain
*     set analog gain for the HSS module
*
*  @return 
*/
static int halAudioHss_setDigitalGain( HAL_AUDIO_BLOCK block, short db )
{
   /* HSS does not support hardware digital gain block, just return here */
   (void) block;
   (void)db;

   HALAUDIO_TRACE( "block %d db %d\n", block, db);

   /* note: it is open to the customer if they want to simulate hardware
    * digital gain here */

   /* note: 0 is returned instead of -1 (error) just in case the app is not 
    * handling the error correctly */
   return 0;
}


/***************************************************************************/
/**
*  hssRegisterMixerPorts
*     register the mixer ports, 1 port for each media stream
*
*  @return 
*/
static void hssRegisterMixerPorts( void )
{
   int i;
   HAL_MIXER_PORT_INFO mixerPort;

   mixerPort.usrid = HAL_MIXER_USER_HAL;
   mixerPort.stereo_partner = -1;
   mixerPort.callback = gHalMixerHssCallback;

   for( i=0; i < HSS_NUM_MEDIA_STREAM; i++ )
   {
      if( hssMediaPacketLen[i] != 0 )
      {
         /* default HSS media streams to 8kHz sampling rate (80 bytes per media packet) */
         mixerPort.input_capability = HAL_MIXER_SIGTYPE_8K;
         mixerPort.output_capability = HAL_MIXER_SIGTYPE_8K;
         if( hssMediaPacketLen[i] == 160 )
         {
            /* this media stream is 16kHz */
            mixerPort.input_capability = HAL_MIXER_SIGTYPE_16K;
            mixerPort.output_capability = HAL_MIXER_SIGTYPE_16K;
         }
         mixerPort.portIdx = halAudioHSSCodecOffset + i;
         hssMixerPorts[i] = halMixer_registerPort( &mixerPort );
      }
   }
}
/***************************************************************************/
/**
*  hssHalMixerCblk_outputPtr
*     Hal Audio Mixer callback function for output pointer 
*
*  @return the buffer pointer where the media samples should be copied from (ingress DMA buffer)
*/

static char * hssHalMixerCbk_outputPtr( int portIdx, int numBytes )
{
   char * ptr = NULL;
   if( numBytes == hssMediaPacketLen[portIdx - halAudioHSSCodecOffset] )
   {
      ptr = hssMixerIngressBuf[(portIdx - halAudioHSSCodecOffset )];
   }
   
   return( ptr );
}

/***************************************************************************/
/**
*  hssHalMixerCblk_inputPtr
*     Hal Audio Mixer callback function for input pointer 
*
*  @return the buffer pointer where the media samples should be copied to (egress DMA buffer)
*/
static char * hssHalMixerCbk_inputPtr( int portIdx, int numBytes )
{
   char * ptr = NULL;

   if( numBytes == hssMediaPacketLen[portIdx - halAudioHSSCodecOffset] )
   {
      ptr = hssMixerEgressBuf[(portIdx - halAudioHSSCodecOffset )];
   }

   return( ptr );
}

/***************************************************************************/
/**
*  dmaInit
*     initialize the DMA for the HSS channels
*
*  @return
*/
static int dmaInit( void )
{
   int i;
   DMAL_DRV *drvp;
   DMA_CFG *cfgp;

   bcm1103DmalDrvInit( &dmaCblk.drvp );
   drvp = dmaCblk.drvp;

   for( i=0; i < HSS_NUM_CHANNEL; i++ )
   {
      cfgp = &dmaCblk.cfg[i];

      cfgp->rx.ch = dmalChUpperToLowerRx( drvp, hssToDmaCh( i ) );
      cfgp->tx.ch = dmalChUpperToLowerTx( drvp, hssToDmaCh( i ) );
    
      /* Initalize Rx DMA */
      dmalInit( drvp, cfgp->rx.ch );
      dmalOptionSet( drvp, cfgp->rx.ch, DMA_RX_DESC_OPT );

      cfgp->rx.descLen  = dmalDescLen( drvp, DMA_RX_DESC_NUM );
      cfgp->rx.descBufp = (char *)getDmaDescRxp( i );

      dmalDescInit( drvp, cfgp->rx.ch, cfgp->rx.descBufp, cfgp->rx.descLen,
                    DMA_RX_DESC_NUM );

      /* Initialize Tx DMA */
      dmalInit( drvp, cfgp->tx.ch );
      dmalOptionSet( drvp, cfgp->tx.ch, DMA_TX_DESC_OPT );

      cfgp->tx.descLen = dmalDescLen( drvp, DMA_TX_DESC_NUM );
      cfgp->tx.descBufp = (char *)getDmaDescTxp( i );

      dmalDescInit( drvp, cfgp->tx.ch, cfgp->tx.descBufp, cfgp->tx.descLen,
                    DMA_TX_DESC_NUM );

      dmalSingleTxSet( drvp, cfgp->tx.ch, 1 );      

      hssCblk.cfg[i].packetLen = ( hssCblk.cfg[i].sampleFreq / 1000 * 
                                   HSS_INTERVAL * hssCblk.cfg[i].sampleSize * hssCblk.cfg[i].numslot );
   }

   return 0;
}

/***************************************************************************/
/**
*  dmaStart
*     start the DMA engine for the HSS channel
*
*  @return
*/
static int dmaStart( int ch )
{
   DMAL_DRV *drvp;
   DMA_CFG *cfgp;
   void *bufp;
   int packetLen = 0;
   int burstLen = 0;

   drvp = dmaCblk.drvp;
   cfgp = &dmaCblk.cfg[ch];

   burstLen = 40; /* DMA burst length of 40 bytes */
   packetLen = hssCblk.cfg[ch].packetLen;

   /* Set DMA burst length */
   dmalBurstLenSet( drvp, cfgp->rx.ch, burstLen );
   dmalBurstLenSet( drvp, cfgp->tx.ch, burstLen );

   /* Prime Rx buffers */
   allocRxBufReset( ch );
   dmalRxUpdate( drvp, cfgp->rx.ch, allocRxBufp, packetLen,
                 DMA_RX_DESC_NUM );

   /* Prime Tx buffers */
   bufp = getZeroBufp();
   /* The first priming buffer must be a full buffer */
   dmalTx( drvp, cfgp->tx.ch, bufp, packetLen );
   /* The second priming buffer determines the amount of egress delay.  In
   *  this case, the amount of delay is half the sampling interval
   */
   dmalTx( drvp, cfgp->tx.ch, bufp, packetLen / 2 );

   /* Enable DMA */
   dmalEnable( drvp, cfgp->tx.ch );
   dmalEnable( drvp, cfgp->rx.ch );

   return( 0 );
}

/***************************************************************************/
/**
*  dmaStop
*     stop the DMA engine for the HSS channel
*
*  @return
*/
static int dmaStop( int ch )
{
   DMAL_DRV *drvp;
   DMA_CFG *cfgp;

   drvp = dmaCblk.drvp;
   cfgp = &dmaCblk.cfg[ch];

   /* Disable the DMA interrupt used as the block sampling interval.
   *  Note that there is no explict enable that starts the interrupt
   *  since dmalEnable() starts it automatically.  It is explicitly
   *  disabled here to prevent an interrupt while disabling the DMA
   */
   dmalIrqRxDisable( drvp, cfgp->rx.ch );

   /* Disable DMA */
   dmalDisable( drvp, cfgp->tx.ch );
   dmalDisable( drvp, cfgp->rx.ch );

   /* Reset descriptors */
   dmalRxFree( drvp, cfgp->rx.ch, NULL );
   dmalTxFree( drvp, cfgp->tx.ch, NULL );

   return( 0 );
}

/***************************************************************************/
/**
*  reset dma buffer pointers
*
*  @return
*/

int allocRxBufReset( int ch )
{
   allocRxBufCh = ch;
   allocRxBufIndex = 0;
}

void *allocRxBufp( int len )
{
   void *ptr;
   (void)len;

   ptr = getDmaRxBufp( allocRxBufCh, allocRxBufIndex++ );
   
   return ptr;
}

/***************************************************************************/
/**
*  mallocInit
*     allocate DMA buffers 
*
*  @return
*/
static int mallocInit( void )
{
   int i;
   int j;
   int k;
   MALLOC_CBLK *cblkp;
   MALLOC_DMA *dmap;

   cblkp = &mallocCblk;

   /* Initialize memory control */
   memset( cblkp, 0, sizeof(*cblkp) );

   /* Create memory pool for halAudio */
   cblkp->poolp = dma_pool_create( "halAudio HSS memory pool", NULL,
                                   MALLOC_POOL_SIZE, CACHE_LINE_SIZE, 0 );
   if( !cblkp->poolp )
   {
      /* Unable to allocate memory */
      printk( KERN_ERR "Unable to allocate APM memory pool\n" );
      return( -ENOMEM );
   }

   /* Allocate memory for DMA descriptors */
   for( i=0; i<HSS_NUM_CHANNEL; i++ )
   {
      for( j=0; j<DMA_DIR_NUM; j++ )
      {
         dmap = &cblkp->dma[i][j];
         dmap->bufp = dma_pool_alloc( cblkp->poolp, GFP_KERNEL,
                                      &dmap->handle );
         if( !dmap->bufp )
         {
            /* Unable to allocate memory */
            printk( KERN_ERR "Unable to allocate DMA descriptor memory\n" );
            mallocExit();
            return( -ENOMEM );
         }
      }
   }

   /* Allocate memory for sample buffers */
   for( i=0; i<HSS_NUM_CHANNEL; i++ )
   {
      for( j=0; j<DMA_DIR_NUM; j++ )
      {
         for( k=0; k<HSS_NUM_BUF; k++ )
         {
            dmap = &cblkp->buf[i][j][k];
            dmap->bufp = dma_pool_alloc( cblkp->poolp, GFP_KERNEL,
                                         &dmap->handle );
            if( !dmap->bufp )
            {
               /* Unable to allocate memory */
               printk( KERN_ERR "Unable to allocate sample buffer memory \n" );
               mallocExit();
               return( -ENOMEM );
            }
         }
      }
   }

   /* Allocate memory for zero buffer */
   dmap = &cblkp->bufZero;
   dmap->bufp = dma_pool_alloc( cblkp->poolp, GFP_KERNEL, &dmap->handle );
   if( !dmap->bufp )
   {
      /* Unable to allocate memory */
      printk( KERN_ERR "Unable to allocate zero buffer memory\n" );
      mallocExit();
      return( -ENOMEM );
   }
   memset( dmap->bufp, 0, MALLOC_POOL_SIZE );

   /* Allocate memory for scratch buffer */
   dmap = &cblkp->bufScratch;
   dmap->bufp = dma_pool_alloc( cblkp->poolp, GFP_KERNEL, &dmap->handle );
   if( !dmap->bufp )
   {
      /* Unable to allocate memory */
      printk( KERN_ERR "Unable to allocate scratch buffer memory\n" );
      mallocExit();
      return( -ENOMEM );
   }

   return( 0 );
}
/***************************************************************************/
/**
*  mallocExit
*     free the allocated DMA buffers 
*
*  @return
*/
static void mallocExit( void )
{
   int i;
   int j;
   int k;
   MALLOC_CBLK *cblkp;
   MALLOC_DMA *dmap;

   cblkp = &mallocCblk;

   if( !cblkp->poolp )
   {
      /* Memory pool for halAudio was not created */
      return;
   }

   /* Free memory for DMA descriptors */
   for( i=0; i<HSS_NUM_CHANNEL; i++ )
   {
      for( j=0; j<DMA_DIR_NUM; j++ )
      {
         dmap = &cblkp->dma[i][j];
         if( dmap->bufp )
         {
            dma_pool_free( cblkp->poolp, dmap->bufp, dmap->handle );
         }
      }
   }

   /* Free memory for sample buffers */
   for( i=0; i<HSS_NUM_CHANNEL; i++ )
   {
      for( j=0; j<DMA_DIR_NUM; j++ )
      {
         for( k=0; k<HSS_NUM_BUF; k++ )
         {
            dmap = &cblkp->buf[i][j][k];
            if( dmap->bufp )
            {
               dma_pool_free( cblkp->poolp, dmap->bufp, dmap->handle );
            }
         }
      }
   }

   /* Free memory for zero buffer */
   dmap = &cblkp->bufZero;
   if( dmap->bufp )
   {
      dma_pool_free( cblkp->poolp, dmap->bufp, dmap->handle );
   }

   /* Free memory for scratch buffer */
   dmap = &cblkp->bufScratch;
   if( dmap->bufp )
   {
      dma_pool_free( cblkp->poolp, dmap->bufp, dmap->handle );
   }

   /* Destroy memory pool for halAudio */
   dma_pool_destroy( cblkp->poolp );
}

/***************************************************************************/
/**
*  halAudioAddonHssReadProc
*     Proc read callback function for the HSS channels
*
*  @return  Number of characters to print
*/
int halAudioAddonHssReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   (void) start; (void) offset; (void) count; (void) data; /* avoid compiler warning */

   /* print the number of times the ingress / egress functions are being called and the number of DMA errors */
   len += sprintf( buf+len, "\nStats            \tHSS0 \tHSS1\n");
   len += sprintf( buf+len, "Ingress:\n" );
   len += sprintf( buf+len, "   num         \t%i  \t%i\n", numIngressCalled, numIngressCalled );
   len += sprintf( buf+len, "   error       \t%i  \t%i\n", numIngressError[0], numIngressError[1] );
   len += sprintf( buf+len, "Egress:\n" );
   len += sprintf( buf+len, "   num         \t%i  \t%i\n", numEgressCalled, numEgressCalled );
   len += sprintf( buf+len, "   error       \t%i  \t%i\n", numEgressError[0], numEgressError[1] );

   *eof = 1;
   return len+1;
}

/***************************************************************************/
/**
*  halAudioAddonHwHssReadProc
*     Proc read callback function for the HSS channels
*
*  @return  Number of characters to print
*/
static int halAudioAddonHwHssReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int i;
   int j;
   int len = 0;

   (void) start; (void) offset; (void) count; (void) data; /* avoid compiler warning */

#define REG32(x)           *((volatile int *)(x))
#define REG32_UC(x)        *((volatile int *)(x)) | 0xa0000000
#define DMA_DUMP_STR(x)    "%08x: %08x %08x %08x %08x\n", \
                           (x), \
                           REG32( (x) ), REG32( (x) + 4 ), \
                           REG32( (x) + 8 ), REG32( (x) + 12 )

   len += sprintf( buf+len, "HSS Configuration:\n" );
   for ( i = 0; i < HSS_NUM_CHANNEL; i++ )
   {
      len += sprintf( buf+len, "   CH%i:  %iHz, %ibits, %ibytes\n", i,
            hssCblk.cfg[i].sampleFreq, hssCblk.cfg[i].sampleSize * 8,
            hssCblk.cfg[i].packetLen);

      len += sprintf( buf+len, "      Ingress @ " );
      for( j=0; j< HSS_NUM_BUF; j++ )
      {
         len += sprintf( buf+len, "0x%08x ", (int)getDmaRxBufp( i, j ) );
      }
      len += sprintf( buf+len, "\n" );

      len += sprintf( buf+len, "      Egress  @ " );
      for( j=0; j< HSS_NUM_BUF; j++ )
      {
         len += sprintf( buf+len, "0x%08x ", (int)getDmaTxBufp( i, j ) );
      }
      len += sprintf( buf+len, "\n" );
   }

   len += sprintf( buf+len, "   Zero @ 0x%08x\n", (int)getZeroBufp() );

   /* TODO: Remove hardcoded register references */

   len += sprintf( buf+len, "\nHSS DMA registers:\n" );
   len += sprintf( buf+len, DMA_DUMP_STR( 0xbafe1600 ) );
   len += sprintf( buf+len, DMA_DUMP_STR( 0xbafe1610 ) );
   len += sprintf( buf+len, DMA_DUMP_STR( 0xbafe1a00 ) );
   len += sprintf( buf+len, DMA_DUMP_STR( 0xbafe1a10 ) );

   len += sprintf( buf+len, "\nDMA descriptors:\n" );
   len += sprintf( buf+len, DMA_DUMP_STR( REG32_UC( 0xbafe1600 ) ) );
   len += sprintf( buf+len, DMA_DUMP_STR( REG32_UC( 0xbafe1610 ) ) );
   len += sprintf( buf+len, DMA_DUMP_STR( REG32_UC( 0xbafe1a00 ) ) );
   len += sprintf( buf+len, DMA_DUMP_STR( REG32_UC( 0xbafe1a10 ) ) );

   *eof = 1;
   return len+1;
}


/** @} */
