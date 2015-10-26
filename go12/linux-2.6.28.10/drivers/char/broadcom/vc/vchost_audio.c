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




/**
*
*  @file    vchost_audio.c
*
*  @brief   Audio frames handing host side implementation
*           Note that this file is only needed if the HAL audio mixer is supported
*
****************************************************************************/
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/broadcom/hw_cfg.h>

#include <linux/broadcom/halaudio_mixer.h>
#include <linux/broadcom/knllog.h>
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/vc.h>

#include "vc_frmfwd_defs.h"
#include "vchost_config.h"
#include "vcgencmd.h"
#include "vcfrmfwd.h"

/******************************************************************************
Global data.
******************************************************************************/
int audio_readIdx = 0;
int audio_writeIdx = 0;

int startplay = 0;
int startrecord = 0;

short *samplebufp = NULL;
void * pktbufp = NULL;
int pktused = 0;

int vcHalMixerPort = -1;
int needStereoComb = 1;

char tempplaybuf[1000];

#define SAMPLES_PER_TICK  160    // the number of samples output by the mixer every 5 ms tick, at 32 kHz sampling rate
#define TICKS_PER_PKT      4     // period for audio sample pkts that will be sent to the VC02 (every 20 ms)

// Definition of audio sample pkt used to send audio data to the VC02
typedef struct VC_AUDIO_PKT
{
   FRMFWD_FRAME_INFO_T info;
   unsigned char data[SAMPLES_PER_TICK * TICKS_PER_PKT * 4];

} VC_AUDIO_PKT;

// Audio sample pkt used to send audio data to the VC02
VC_AUDIO_PKT audio_pkt;

static unsigned int recAudioPktTickCntr = 0;
static unsigned int recAudioNumBytesFromMixer = 0;

int numplayed = 0;
int numerror = 0;

#define  VC_AUDIO_TONEDEBUG   0

#if VC_AUDIO_TONEDEBUG
short testData[48] = 
{
   0,   2144,   4251,   6285,   8212,   9997,   11613,   13028,   14222,   15173,
   15863,   16282,   16422,   16282,   15863,   15172,   14223,   13029,   11612,   9997,
   8212,   6285,   4250,   2144,   0,   -2144,   -4251,   -6285,   -8211,   -9997,
   -11613,   -13028,   -14223,   -15172,   -15863,   -16283,   -16423,   -16281,   -15863,   -15173,
   -14222,   -13029,   -11612,   -9997,   -8212,   -6285,   -4250,   -2144,
};

int tempdataIndex = 0;

//Note that byte swapping is not necessary if we using fake data
#undef VC_VTOH16
#define VC_VTOH16

#endif
/******************************************************************************
Local types and defines.
******************************************************************************/
#define MAX_NUM_SAMPLE_PAIRS  15872

/******************************************************************************
Static data.
******************************************************************************/
static int gcmdThreadPid = 0;
spinlock_t num_lock;
struct completion gcmdThreadExited; 
struct semaphore gcmdSem;
static int audio_stream_num = -1;
/******************************************************************************
Static functions.
******************************************************************************/
char * vcAudioHalMixerCbk_outputPtr ( int portIdx, int numBytes );
char * vcAudioHalMixerCbk_inputPtr ( int portIdx, int numBytes );
void vcAudioHalMixerCbk_inputReady ( int portIdx, int numBytes );
void vcAudioHalMixerCbk_outputRead( int portIdx, int numBytes );

HAL_MIXER_CALLBACK_T gvcAudioMixerCallback = 
{
   vcAudioHalMixerCbk_outputPtr,
   vcAudioHalMixerCbk_inputPtr,
   vcAudioHalMixerCbk_inputReady,
   vcAudioHalMixerCbk_outputRead
};

static int cmdThread( void *unused );
/*---------------------------------------------------------------------------*/
/***************************************************************************/
/**
*  Initialise the audio packet handler for use.
*
*  @param1
*
*  @return
*
*  @remarks
*  A negative return value indicates failure
*
*/

int vc_audiohdl_init (void)
{
   int status = 0;

   samplebufp = vmalloc( MAX_NUM_SAMPLE_PAIRS * 2 * 2);
   //samplebufp = testData;
   pktbufp = vmalloc( sizeof( FRMFWD_FRAME_INFO_T ) + (MAX_NUM_SAMPLE_PAIRS * 2 * 2) );
   audio_readIdx = 0;
   audio_writeIdx = 0;
   pktused = 0;

   if( (samplebufp == NULL) || (pktbufp == NULL) )
   {
      status = -1;
   }
   spin_lock_init( &num_lock );
   sema_init( &gcmdSem, 0 );
   init_completion( &gcmdThreadExited );
   gcmdThreadPid = kernel_thread( cmdThread, 0, 0 );
   return status;
}

FRMFWD_FRAME_T * CB_AllocAudioBuffer( int streamNum, int length_in_bytes )
{
   FRMFWD_FRAME_T * bufp = NULL;
   if( pktused == 0 )
   {
      bufp = (FRMFWD_FRAME_T *)pktbufp;
      pktused = 1;
   }
   return bufp;
}

void CB_AudioFrameReceived( FRMFWD_FRAME_T *fwdFrame, unsigned int lengthinbyte, int streamNum )
{
   int i, available_space;
   short * datap, *dstp;
   int flag = 0;

   int numpair = fwdFrame->info.data_len / 4;
   int spaceRequired = numpair;

   if( needStereoComb == 0 )
   {
      spaceRequired = numpair * 2;
   }

   /* senity check */
   if( fwdFrame != (FRMFWD_FRAME_T *)pktbufp )
   {
      printk("the incoming ptr 0x%x doesn't match the local copy 0x%x\n", (unsigned int)fwdFrame, (unsigned int)pktbufp );
   }
   if( startplay == 0 )
   {
      printk("start play not received yet, why are we receiving data %d?\n", numpair);
   }
   datap = (short *)&fwdFrame->data[0];

#if VC_AUDIO_TONEDEBUG
   for( i=0; i < numpair; i++ )
   {
      datap[0] = testData[tempdataIndex];
      datap[1] = testData[tempdataIndex];
      tempdataIndex++;
      if( tempdataIndex >= 48 )
      {
         tempdataIndex = 0;
      }
      datap+= 2;
   }
   datap = (short *)&fwdFrame->data[0];
#endif
   dstp = &samplebufp[audio_writeIdx];

   /* check if we have enought space to receive the message */
   if( audio_writeIdx < audio_readIdx )
   {
      available_space = audio_readIdx - audio_writeIdx;
   }
   else
   {
      available_space = ( ((MAX_NUM_SAMPLE_PAIRS*2) - audio_writeIdx) + audio_readIdx );
   }
   if( available_space < spaceRequired )
   {
      printk("not enough space for incoming packets: ava %d, asked %d\n", 
            available_space, spaceRequired );
   }
   else
   {
      /* convert from stereo to mono and write it so pcm playout buffer */
      int first_part = ((MAX_NUM_SAMPLE_PAIRS*2) - audio_writeIdx);
      int second_part = 0;
      if( first_part > spaceRequired )
      {
         first_part = spaceRequired;
      }
      else
      {
         second_part = spaceRequired - first_part;
      }
      if( needStereoComb )
      {
         for( i = 0; i < first_part; i++ )
         {
            dstp[i] = (short)(( (int)((short)VC_VTOH16(datap[0])) + (int)((short)VC_VTOH16(datap[1])) )/ 2);
            flag |= dstp[i];
            datap += 2;
         }
         if( second_part > 0 )
         {
            dstp = samplebufp;
            for( i = 0; i < second_part; i++ )
            {
               dstp[i] = (short)(( (int)((short)VC_VTOH16(datap[0])) + (int)((short)VC_VTOH16(datap[1])) )/ 2);
               flag |= dstp[i];
               datap += 2;
            }
         }
      }
      else
      {
         /* playing out stereo samples, do not have to combine the left and right channel input */
         for( i=0; i < first_part; i++ )
         {
            dstp[i] = (short)(VC_VTOH16(datap[i]));
         }
         datap+= first_part;
         if( second_part > 0 )
         {
            dstp = samplebufp;
            for( i=0; i < second_part; i++ )
            {
               dstp[i] = (short)(VC_VTOH16(datap[i]));
            }
         }
      }
      /* take care of buffer wrap around */
      audio_writeIdx += spaceRequired;
      if( audio_writeIdx >= (MAX_NUM_SAMPLE_PAIRS*2) )
      {
         audio_writeIdx = second_part;
      }
   }

   /* release the packet for next incoming packet */
   pktused = 0;
}

void vc_audio_start( VC_Audio_t *audiop )
{
   HALMIXER_Connection_t connect;
   /* register mixer port if necessary */
   if( vcHalMixerPort == -1 )
   {
      HAL_MIXER_PORT_INFO info;

      info.usrid = HAL_MIXER_USER_VC02;
      /* only support 1 port for now */
      info.portIdx = 0;
      info.input_capability = HAL_MIXER_SIGTYPE_16K;
      info.output_capability = HAL_MIXER_SIGTYPE_48K;
      info.callback = gvcAudioMixerCallback;
      vcHalMixerPort = halMixer_registerPort( &info );
   }
   connect.duplex = 0;

   if( audiop->record == 0 )
   {
      /* not recording, must be play out */
      numplayed = 0;

      audio_writeIdx = 0;
      audio_readIdx = 0;
#if VC_AUDIO_TONEDEBUG
      tempdataIndex = 0;
#endif
      connect.src = vcHalMixerPort;
      connect.dst_1 = audiop->audioMixerPort_left;
      if( audiop->audioMixerPort_right == -1 )
      {
         connect.dst_2 = -1;
         connect.connType = HALMIXER_CONNECT_MONO;
         needStereoComb = 1;
      }
      else
      {
         connect.dst_2 = audiop->audioMixerPort_right;
         connect.connType = HALMIXER_CONNECT_STEREO_SPLIT;
         needStereoComb = 0;
      }
      halMixer_addConnect( &connect );

      startplay = 1;
   }

   if( audiop->record == 1 )
   {
      // Configure things for recording

      audio_stream_num = audiop->stream_num;
      memset( &audio_pkt, 0, sizeof( audio_pkt ) );
      recAudioPktTickCntr = 0;         
      recAudioNumBytesFromMixer = 0;

      /* note that the recording frequency can change depending on the voice encoder being used */
      /* the playout frequency is kept unchanged at 48 kHz */
      halMixer_changeSampFreq( vcHalMixerPort, audiop->record_freq, HAL_MIXER_SIGTYPE_48K );
      connect.src = audiop->audioMixerPort_left;
      connect.dst_1 = vcHalMixerPort;
      connect.connType = HALMIXER_CONNECT_MONO;
      connect.dst_2 = -1;
      halMixer_addConnect( &connect );

      // Note: leave this last to ensure cmdThread will not run during the initializations
      startrecord = 1;
   }
}

void vc_audio_stop( VC_Audio_t *audiop )
{
   HALMIXER_Connection_t connect;

   connect.duplex = 0;

   if( audiop->record == 0 )
   {
      startplay = 0;
      connect.src = vcHalMixerPort;
      connect.dst_1 = audiop->audioMixerPort_left;
      if(audiop->audioMixerPort_right != -1)
      {
         connect.dst_2 = audiop->audioMixerPort_right;
         connect.connType = HALMIXER_CONNECT_STEREO_SPLIT;
      }
      else
      {
         connect.dst_2 = -1;
         connect.connType = HALMIXER_CONNECT_MONO;
      }
      halMixer_removeConnect( &connect );
      needStereoComb = 1;
   }
   else
   {
      if( startrecord != 1 )
      {
         printk("vc_audio_stop_play : something wrong, record has not been started\n");
      }
      startrecord = 0;
      connect.src = audiop->audioMixerPort_left;
      connect.dst_2 = -1;
      connect.connType = HALMIXER_CONNECT_MONO;
      connect.dst_1 = vcHalMixerPort;
      halMixer_removeConnect( &connect );
      audio_stream_num = -1;
   }
   {
      int rc;
      HAL_MIXER_PORT_INFO info;

      info.usrid = HAL_MIXER_USER_VC02;
      info.portIdx = 0;
      rc = halMixer_deregisterPort( vcHalMixerPort, &info );
      if( rc )
      {
         printk("cannot deregister port %d\n", vcHalMixerPort );
      }
      else
      {
         vcHalMixerPort = -1;
      }
   }

}

int vc_audio_redirect( VC_Audio_reDirect_t * audiop )
{
   HALMIXER_Connection_t connect;
   int rc = -1;
   /* This function is supposed to be called on during video / audio playback
    * this function provides the user capability to switch the audio playout codec */
   if( (vcHalMixerPort == -1) || (startplay == 0) || (startrecord == 1) )
   {
      printk( "vc_audio_redirect: there is currently no audio playout \n" );
      return( rc );
   }
   rc = 0;
   if( (audiop->currMixerPort_left == audiop->newMixerPort_left) && 
       (audiop->currMixerPort_right == audiop->newMixerPort_right) )
   {
      printk( "vc_audio_redirect: current port and new port is the same, nothing needs to be done\n");
      return( rc );
   }
   /* remove the existing mixer connection */
   connect.src = vcHalMixerPort;
   connect.dst_1 = audiop->currMixerPort_left;
   connect.dst_2 = audiop->currMixerPort_right;
   connect.duplex = 0;
   if( needStereoComb == 1 )
   {
      connect.connType = HALMIXER_CONNECT_MONO;
   }
   else
   {
      connect.connType = HALMIXER_CONNECT_STEREO_SPLIT;
   }
   rc = halMixer_removeConnect( &connect );

   /* add new mixer connection */
   connect.dst_1 = audiop->newMixerPort_left;
   connect.dst_2 = audiop->newMixerPort_right;

   /* flush the current samples IF the request new connection is not of the same
    * type as the existing one */
   if( ( (audiop->currMixerPort_right < 0) && (audiop->newMixerPort_right >= 0) ) ||
       (  (audiop->currMixerPort_right >= 0) && (audiop->newMixerPort_right < 0) ) )
   {
      /* changing connection type from mono (stereo) to stereo (mono) */
      /* disable interrupt */
      spin_lock_irq( &num_lock );
      numplayed = 0;
      audio_writeIdx = 0;
      audio_readIdx = 0;
      /* re-enable interrupt */
      spin_unlock_irq( &num_lock );
   }

   if( audiop->newMixerPort_right >= 0 )
   {
      connect.connType = HALMIXER_CONNECT_STEREO_SPLIT;
      needStereoComb = 0;
   }
   else
   {
      connect.connType = HALMIXER_CONNECT_MONO;
      needStereoComb = 1;
   }

   rc |= halMixer_addConnect( &connect );
   return( rc );
}
char *vcAudioHalMixerCbk_outputPtr( int portIdx, int numBytes )
{
   char *bufp = (char *)&samplebufp[audio_readIdx];
   int samplesNeeded = numBytes / 2;
   int samplesAvailable;

   (void)portIdx;
   if( audio_readIdx > audio_writeIdx )
   {
      /* read after write pointer in the buffer, so can read till the end of the buffer */
      samplesAvailable = ((MAX_NUM_SAMPLE_PAIRS*2) - audio_readIdx) + audio_writeIdx;
   }
   else
   {
      /* what is available is only between the read and write pointer */
      samplesAvailable = (audio_writeIdx - audio_readIdx);
   }
   if( samplesAvailable < (numBytes / 2) )
   {
      /* underrun - we do not have enough samples to fill the request */
      if( (numerror == 0) && (samplesAvailable > 0) )
      {
         /* print out an error message if this is the first time we underrun */
         printk("not enough bytes played %d requested %d\n", samplesAvailable, (numBytes/2) );
         numerror = samplesAvailable;
      }
      bufp = NULL;
   }
   else
   {
      numerror = 0;
      if( (audio_readIdx + samplesNeeded) > (MAX_NUM_SAMPLE_PAIRS*2) )
      {
         /* need to wrap over the end of buffer */
         int first_part = (MAX_NUM_SAMPLE_PAIRS*2) - audio_readIdx;

         /* copy the first part - from the read index till the end of buffer */
         memcpy( tempplaybuf, bufp, first_part * 2 );

         /* copy the remaining part - starting from the beginning of the buffer */
         memcpy( &tempplaybuf[first_part*2], samplebufp, ((samplesNeeded-first_part)*2) );

         bufp = tempplaybuf;
      }
   }
   return( bufp );
}
char * vcAudioHalMixerCbk_inputPtr( int portIdx, int numBytes )
{
   char * bufp = tempplaybuf;
   (void)portIdx;

   /* provide the buffer pointer where the recording samples can be written to */
   if( numBytes > (sizeof( audio_pkt.data )/2) )
   {
      printk("wrong size: asked %d, ava %d\n", numBytes,
            (sizeof( audio_pkt.data )/2) );
            
      /* NULL means the request is rejected */
      bufp = NULL;
   }
   return( bufp );
}

void vcAudioHalMixerCbk_inputReady( int portIdx, int numBytes )
{
   (void) portIdx;

   /* input is ready to be processed, fire a semaphore to the worker thread for recording */
   if (startrecord)
   {
      // vc_audio_start has flagged that all initializations are complete, so we can fire the semaphone
      // to signal cmdThread to run
      recAudioNumBytesFromMixer = numBytes;
      up( &gcmdSem );
   }
}

void vcAudioHalMixerCbk_outputRead( int portIdx, int numBytes )
{
   (void) portIdx;
        
   audio_readIdx += (numBytes / 2);
   if( audio_readIdx >= (MAX_NUM_SAMPLE_PAIRS*2) )
   {
      audio_readIdx -= (MAX_NUM_SAMPLE_PAIRS*2);
   }
   /* keep track of the number of samples played */

   if( needStereoComb )
   {
      numplayed += (numBytes / 2);
   }
   else
   {
      numplayed += (numBytes / 4);
   }
   /* every 960 samples played (20ms at 48kHz sampling rate), fire a semaphore, 
    * this will send a message to the VC02 for a sense of time */
   if( numplayed >= 960 )
   {
      up( &gcmdSem );
   }
}

#define CMDTHREAD_RT_PRIORITY    60
static int cmdThread( void *unused )
{
   struct sched_param sparm;
   VC_GenCmd_t gencmd;
   int rc, commandLength, vc_sync;

   daemonize( "vc_audio_cmd");

   /* set real-time-priority */
   sparm.sched_priority = CMDTHREAD_RT_PRIORITY;

   if( ( rc = sched_setscheduler( current, SCHED_FIFO, &sparm )) < 0 )
   {
      printk( KERN_ERR "failed to set the RTP priority %i for vc_audio_cmd thread\n", sparm.sched_priority );
   }
         
   sprintf( &gencmd.cmd[0], "audio_control num_played " );

   commandLength = strlen( gencmd.cmd );

   while( down_interruptible( &gcmdSem ) == 0 )
   {
      if( startplay )
      {
         spin_lock_irq( &num_lock );
         vc_sync = numplayed;
         numplayed = 0;
         spin_unlock_irq( &num_lock );

         sprintf( &gencmd.cmd[commandLength], "%d\n", vc_sync );

         gencmd.response.err = vc_gencmd( gencmd.response.str, 
                                          sizeof( gencmd.response.str ),
                                          "%s",
                                          &gencmd.cmd[0] );
      }
      if( startrecord )
      {
         int i;
         int numSamples;
         short *src = (short *)tempplaybuf;
         short *dst = (short *)(audio_pkt.data + audio_pkt.info.data_len);

         numSamples = recAudioNumBytesFromMixer / sizeof( short );

         if (recAudioNumBytesFromMixer*2 + audio_pkt.info.data_len > sizeof( audio_pkt.data ))
         {
            // Something's gone wrong.  This will result in an out-of-range write, so log an error message
            // and ditch the data.
            printk( "cmdThread: adding %d bytes will result in %d bytes (greater than %d byte buffer). recAudioPktTickCntr=%d\n", recAudioNumBytesFromMixer*2, recAudioNumBytesFromMixer*2 + audio_pkt.info.data_len, sizeof( audio_pkt.data ), recAudioPktTickCntr );
            recAudioPktTickCntr = 0;
            audio_pkt.info.data_len = 0;
         }
         else
         {
            // Create stereo samples for the VC02 and append them to 'VC02-pending' buffer
            for( i=0; i < numSamples; i++ )
            {
               dst[0] = VC_HTOV16(src[i]);
               dst[1] = dst[0];
               dst += 2;
            }
            audio_pkt.info.data_len += numSamples * sizeof( short [2] );
   
            if (++recAudioPktTickCntr == TICKS_PER_PKT)
            {
               // The 'VC02-pending' buffer contains the required amount of data, so send it to the VC02
               audio_pkt.info.flags = 0x04;
               audio_pkt.info.stream_num = audio_stream_num;
               audio_pkt.info.seq_num++;
               vc_frmfwd_send_frame( (FRMFWD_FRAME_T *)(&audio_pkt), 1 );
               recAudioPktTickCntr = 0;
               audio_pkt.info.data_len = 0;
            }
         }
      }
   }
   complete_and_exit( &gcmdThreadExited, 0 );
   return 0;
}
