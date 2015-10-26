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
*  halmixer_api.c
*
*  PURPOSE:
*
*     This file contains the HAL audio mixer api for the user.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/halaudio_mixer.h>
#include <linux/broadcom/knllog.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include "halmixer_priv.h"
#include "halmixer_resamp.h"
#include "halmixer_adder.h"

/* ---- Public Variables ------------------------------------------------- */
/**
 * @addtogroup HAL
 * @{
 */

/* ---- Private Constants and Types -------------------------------------- */
static  char banner[] __initdata = KERN_INFO "Hal Mixer: 0.00\n";
/* ---- Private Variables ------------------------------------------------ */

// Structure to hold the port information
halMixer_port_info gMixerPorts[HALMIXER_MAX_PORT];
static int gMixerNumPortRegistered = 0;

halMixer_Connection_List gMixerConnectionList;

RESAMPFUNC gResampleFunc = mixerResample;
ADDERFUNC gAdderFunc = mixerAdder;
ATTENUFUNC gAttenuateFunc = mixerApplyAtten;

/* run time global that keep track of the rate when the switchboard is being served at */
static int switchBoardFreq = 0;
static int switchBoardBytes = 0;

/* buffers allocated from virtual memory to hold the left and right channel samples */
#define MIXER_MONO_BUFFERSIZE    240   /* 240 samples good for 5 ms worth @ 48kHz */
short * leftInput = NULL;
short * rightInput = NULL;


int leqr = 1;

#define MIXER_RESAMPLER_SCRATCH_SIZE   200 /* 400 bytes (200 samples) good for 5 ms worth @ 40 kHz */
char * resample_scratch = NULL;        

#define MAX_Q16_ATTENUATE  63

/**
* dB to 16-bit Q16 multiplier map. Used in applying software gains.
* (65536 = 0dB)
*/
static const unsigned int q16GainMap[MAX_Q16_ATTENUATE + 1] =
{
   65536,58409,52057,46396,41350,36854,32846,29274,
   26090,23253,20724,18471,16462,14672,13076,11654,
   10387, 9257, 8250, 7353, 6554, 5841, 5206, 4640,
    4135, 3685, 3285, 2927, 2609, 2325, 2072, 1847,
    1646, 1467, 1308, 1165, 1039,  926,  825,  735,
     655,  584,  521,  464,  414,  369,  328,  293,
     261,  233,  207,  185,  165,  147,  131,  117,
     104,   93,   83,   74,   66,   58,   52,   46
};

#define BCM_SYSCTL_MIXER_BASE 1

#define BCM_SYSCTL_MIXER_NODE(ID) {                         \
      .ctl_name         = BCM_SYSCTL_MIXER_BASE + ID,       \
      .procname         = "port"#ID,                        \
      .data             = &gMixerPorts[ID-1],               \
      .maxlen           = sizeof( halMixer_port_info ),     \
      .mode             = 0644,                             \
      .proc_handler     = &proc_dointvec                    \
}

/* note 15 proc entry for 15 mixer port now, we need to add more if HALMIXER_MAX_PORT changes */
static struct ctl_table gSysCtlMixer[] = {
   {
      .ctl_name         = 1,
      .procname         = "swb",
      .data             = &gMixerConnectionList,
      .maxlen           = sizeof( halMixer_Connection_List ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   BCM_SYSCTL_MIXER_NODE(1),
   BCM_SYSCTL_MIXER_NODE(2),
   BCM_SYSCTL_MIXER_NODE(3),
   BCM_SYSCTL_MIXER_NODE(4),
   BCM_SYSCTL_MIXER_NODE(5),
   BCM_SYSCTL_MIXER_NODE(6),
   BCM_SYSCTL_MIXER_NODE(7),
   BCM_SYSCTL_MIXER_NODE(8),
   BCM_SYSCTL_MIXER_NODE(9),
   BCM_SYSCTL_MIXER_NODE(10),
   BCM_SYSCTL_MIXER_NODE(11),
   BCM_SYSCTL_MIXER_NODE(12),
   BCM_SYSCTL_MIXER_NODE(13),
   BCM_SYSCTL_MIXER_NODE(14),
   BCM_SYSCTL_MIXER_NODE(15),
   {
      .ctl_name         = 17,
      .procname         = "numRegistered",
      .data             = &gMixerNumPortRegistered,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 18,
      .procname         = "leqr",
      .data             = &leqr,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name   = CTL_BCM_MIXER,
      .procname   = "mixer",
      .mode       = 0555,
      .child      = gSysCtlMixer
   },
   {}
};

static struct ctl_table_header *gSysCtlHeader = NULL;

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  halMixer_findPortNumBytes - 
*     determin the number of bytes needs for input and output for the 
*     port specified (base on sampling rate and the rate when the switchboard
*     is being serviced)
*
***************************************************************************/
void halMixer_findPortNumBytes( int idx )
{
   int sampFreq = 0;
   int portFreq = 0;
   halMixer_port_info * portInfop = &gMixerPorts[idx];

   portInfop->numInBytes = 0;
   portInfop->numOutBytes = 0;

   switch( switchBoardFreq )
   {
      /* determine the sample frequency of the switchboard, when resampler is involved */
      case HAL_MIXER_SIGTYPE_8K:
         sampFreq = 8000;
      break;
      case HAL_MIXER_SIGTYPE_16K:
         sampFreq = 16000;
      break;
      case HAL_MIXER_SIGTYPE_32K:
         sampFreq = 32000;
      break;
      case HAL_MIXER_SIGTYPE_40K:
         sampFreq = 40000;
      break;
      /* note that other frequency have not been listed because we do not support that particular resampler */
   };

   if( portInfop->inType )
   {
      /* determine the number of samples for input (to the port) */
      if( portInfop->inType == switchBoardFreq )
      {
         portInfop->numInBytes = switchBoardBytes;
      }
      else
      {
         /* the port is running at a different sampling rate as compared to the switchboard */
         portInfop->numInBytes = 0;
         portFreq = 0;

         /* note that not frequencies are listed here because our resampler set is limited */
         switch( portInfop->inType )
         {
            case HAL_MIXER_SIGTYPE_8K:
               portFreq = 8000;
            break;
            case HAL_MIXER_SIGTYPE_16K:
               portFreq = 16000;
            break;
            case HAL_MIXER_SIGTYPE_32K:
               portFreq = 32000;
            break;
            case HAL_MIXER_SIGTYPE_40K:
               portFreq = 40000;
            break;
         };
         if( (sampFreq != 0) && (portFreq != 0) )
         {
            portInfop->numInBytes = (portFreq * switchBoardBytes) / sampFreq;
         }
      }
   }
   if( portInfop->outType )
   {
      /* determine the number of samples for output (from the port) */
      if( portInfop->outType == switchBoardFreq )
      {
         portInfop->numOutBytes = switchBoardBytes;
      }
      else
      {
         /* the port is running at a different sampling rate as compared to the switchboard */
         portFreq = 0;
         portInfop->numOutBytes = 0;
         switch( portInfop->outType )
         {
            case HAL_MIXER_SIGTYPE_8K:
               portFreq = 8000;
            break;
            case HAL_MIXER_SIGTYPE_16K:
               portFreq = 16000;
            break;
            case HAL_MIXER_SIGTYPE_32K:
               portFreq = 32000;
            break;
            case HAL_MIXER_SIGTYPE_40K:
               portFreq = 40000;
            break;
            case HAL_MIXER_SIGTYPE_48K:
               portFreq = 48000;
            break;
         };
         if( (sampFreq != 0) && (portFreq != 0) )
         {
            portInfop->numOutBytes = (portFreq * switchBoardBytes) / sampFreq;
         }
      }
   }
}
/****************************************************************************
*
*  halMixer_init
*
***************************************************************************/
int halMixer_init( void )
{
   int rc = 0;
   int i;
   HALMIXER_PRINTK( ("halaudio_mixer : initializing HAL mixer \n") );
   /* initialize the registered port list */
   memset( gMixerPorts, 0, sizeof( gMixerPorts ) );
   /* mark all ports to be unused for now */
   for( i=0; i < HALMIXER_MAX_PORT; i++ )
   {
      gMixerPorts[i].info.usrid = HAL_MIXER_USER_NONE;
   }
   gMixerNumPortRegistered = 0;

   /* initialize the mixer connection list */
   memset( &gMixerConnectionList, 0, sizeof( gMixerConnectionList ) );
   for( i=0; i < HALMIXER_SWBARRAY_LEN; i++ )
   {
      gMixerConnectionList.list[i].numConn = -1;
   }

   /* allocate the buffers for the left and right channel (for stereo playout) */
   leftInput = vmalloc( sizeof(short) * MIXER_MONO_BUFFERSIZE );
   rightInput = vmalloc( sizeof(short) * MIXER_MONO_BUFFERSIZE );

   resample_scratch = vmalloc( sizeof(short) * MIXER_RESAMPLER_SCRATCH_SIZE );
   return rc;
}

/****************************************************************************
*
*  halMixer_registerPort - register a halmixer user
*
***************************************************************************/
int halMixer_registerPort( HAL_MIXER_PORT_INFO *info )
{
   int rc = -1;
   int i;

   HALMIXER_PRINTK( ("halaudio_mixer : trying to register port ID %d\n", info->usrid) );

   /* check the gMixerNumPortRegistered, make sure that the list is not full yet */
   if( gMixerNumPortRegistered == HALMIXER_MAX_PORT )
   {
      HALMIXER_ERROR( ("halaudio_mixer : ERROR! List Full! Cannot take new ports\n") );
      return rc;
   }
   /* search for an empty spot */
   for( i=0; i < HALMIXER_MAX_PORT; i++ )
   {
      if( gMixerPorts[i].info.usrid == HAL_MIXER_USER_NONE )
      {
         /* the port returned is the first empty spot in the list */
         rc = i;
         break;
      }
   }
   memcpy( &gMixerPorts[rc].info, info, sizeof(HAL_MIXER_PORT_INFO) );
   gMixerNumPortRegistered++;

   return rc;
}
/****************************************************************************
*
*  halMixer_deregisterPort - free up the resource hold up by the specified port
*
***************************************************************************/
int halMixer_deregisterPort( int portId, HAL_MIXER_PORT_INFO *info )
{
   int rc = -1; /* assuming the port is not being registered */
   /* this is just for senity checking */
   if( (info->usrid != gMixerPorts[portId].info.usrid) ||
       (info->portIdx != gMixerPorts[portId].info.portIdx) )
   {
      HALMIXER_ERROR( ("halaudio_mixer : ERROR! trying to de-register the wrong port %d\n", portId ) );
   }
   else
   {
      /* remove all connections from the switchboard that involved the target port */
      halMixer_switchboardRemovePort( portId );
      gMixerNumPortRegistered--;
      memset( &gMixerPorts[portId], 0, sizeof( halMixer_port_info ) );
      gMixerPorts[portId].info.usrid = HAL_MIXER_USER_NONE;
      rc = 0; 
   }

   return rc;
}

/****************************************************************************
*
*  halMixer_addConnect - add a connection to the switchboard
*
***************************************************************************/
int halMixer_addConnect( HALMIXER_Connection_t * connectp )
{
   unsigned long flags = 0;
   int rc = -1;
   HALMIXER_Connection_t connectCopy;

   if( (connectp->src == -1) || (connectp->dst_1 == -1) )
   {
      HALMIXER_ERROR( ("halaudio_mixer : invalid port number, src %d, dst %d\n", 
               connectp->src, connectp->dst_1) );
      return rc;
   }
   local_irq_save( flags );
   rc = halMixer_ConnectPort_simplex( connectp );

   if( (rc == 0) && connectp->duplex )
   {
      /* only continue on if first connection succeed, and duplex connection is required */
      connectCopy.dst_1 = connectp->src;
      connectCopy.src = connectp->dst_1;
      connectCopy.connType = connectp->connType;
      connectCopy.dst_2 = connectp->dst_2;
      connectCopy.duplex = 1;

      rc |= halMixer_ConnectPort_simplex( &connectCopy );
   }
   /* quit if any of the connection failed */
   if( rc != 0 )
   {
      HALMIXER_ERROR( ("halaudio_mixer : connection failed src %d, dst %d\n", 
               connectp->src, connectp->dst_1) );
   }
   local_irq_restore(flags);

   return rc;
}

/****************************************************************************
*
*  halMixer_removeConnect - remove a connection from the switchboard
*
***************************************************************************/
int halMixer_removeConnect( HALMIXER_Connection_t * connectp )
{
   int rc = -1;
   unsigned long flags;
   halMixer_Connection * conn;
   if( (connectp->src == -1) || (connectp->dst_1 == -1) )
   {
      HALMIXER_ERROR( ("halaudio_mixer : invalid port number, src %d, dst %d\n", 
               connectp->src, connectp->dst_1) );
      return rc;
   }

   if( connectp->duplex && (connectp->connType != HALMIXER_CONNECT_MONO) )
   {
      HALMIXER_ERROR( ("halaudio_mixer : we do not support stereo duplex connection\n") );
      return rc;
   }
   if( (connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT) && (connectp->dst_2 < 0) )
   {
      HALMIXER_ERROR( ("halaudio_mixer : second channel not specified for stereo connection \n") );
      return rc;
   }

   local_irq_save( flags );
   conn = (halMixer_Connection *)gMixerConnectionList.list;
   /* traverse the swb connection list to check if such a connection exist */
   while( conn->src.numConn != -1 )
   {
      if( (conn->src.share.srcPortId == connectp->src) && (conn->src.connType == connectp->connType) )
      {
         HALMIXER_PRINTK(( "connection found %d num %d type %d\n", conn->src.share.srcPortId, conn->src.numConn, conn->src.connType ) );
         break;
      }
      conn = (halMixer_Connection *)&conn->dst[conn->src.numConn];
   }
   if( conn->src.numConn != -1 )
   {
      /* matching connection type for the source port found, carry on */
      rc = halMixer_DisconnectPort_simplex( conn, connectp->src, connectp->dst_1 );
      if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
      {
         rc |= halMixer_DisconnectPort_simplex( conn, connectp->src, connectp->dst_2 );
      }
   }
   if( connectp->duplex )
   {
      /* remove the reverse direction */
      conn = (halMixer_Connection *)gMixerConnectionList.list;
      while( conn->src.numConn != -1 )
      {
         if( (conn->src.share.srcPortId == connectp->dst_1) && (conn->src.connType == connectp->connType) )
         {
            break;
         }
         conn = (halMixer_Connection *)&conn->dst[conn->src.numConn];
      }
      if( conn->src.numConn != -1 )
      {
         rc |= halMixer_DisconnectPort_simplex( conn, connectp->dst_1, connectp->src );
      }
   }

   /* update the number of input/output samples that has to be moved when the switchboard runs */
   halMixer_findPortNumBytes( connectp->src );
   halMixer_findPortNumBytes( connectp->dst_1 );
   if( connectp->dst_2 >= 0)
   {
      halMixer_findPortNumBytes( connectp->dst_2 );
   }

   local_irq_restore(flags);
   return rc;
}

/****************************************************************************
*
*  halMixer_handleMonoSamples - write mono samples from source to destination when
*                               the switchboard is being serviced
*
***************************************************************************/
void halMixer_handleMonoSamples( char * src_bufp, halMixer_port * dstGroup, halMixer_port_info *srcPortp )
{
   halMixer_resampler_info * resamplerInfo;
   halMixer_port_info * dstPortp;
   int srcReqNumBytes, k;
   int firstDstPortNum, numBytesToDst;
   char * bufp = src_bufp;

   resamplerInfo = (halMixer_resampler_info *)dstGroup->share.dst_grp.resampler_ptr;
   firstDstPortNum = dstGroup->share.dst_grp.dst_lst[0];
   numBytesToDst = gMixerPorts[firstDstPortNum].numInBytes;

   if( numBytesToDst == 0 )
   {
      /* skip sample handling if there is nothing to be done */
      return;
   }

   if( resamplerInfo != NULL )
   {
      /* safety check to retrieve the number of bytes needed from the source */
      /* probably can omit */
      srcReqNumBytes = numBytesToDst;
      if( resamplerInfo->decimRatio != 0 )
      {
         srcReqNumBytes *= resamplerInfo->decimRatio;
      }
      if( resamplerInfo->interpRatio != 0 )
      {
         srcReqNumBytes /= resamplerInfo->interpRatio;
      }
      if( (numBytesToDst > (MIXER_RESAMPLER_SCRATCH_SIZE * 2)) || (srcReqNumBytes != srcPortp->numOutBytes) )
      {
         HALMIXER_ERROR( ("resample error, the stack buffer is not big enough %d %d %d %d\n",
                           numBytesToDst, (MIXER_RESAMPLER_SCRATCH_SIZE * 2) , 
                           srcReqNumBytes, srcPortp->numOutBytes) );
         bufp = NULL;
      }
      else
      {
         short filterlen = resamplerInfo->filterlen;
         short * resamplebufp = resamplerInfo->resampbufp;

         memcpy( &resamplebufp[filterlen], bufp, srcReqNumBytes );

         gResampleFunc( &resamplebufp[filterlen], (short *)resample_scratch, (numBytesToDst/2),
                        resamplerInfo->coefficient,
                        filterlen, 
                        resamplerInfo->interpRatio,
                        resamplerInfo->decimRatio );

         bufp += (srcReqNumBytes - (filterlen*2) );
         /* copy the history for the resampler */
         memcpy( resamplebufp, bufp, (filterlen * 2) );
         bufp = resample_scratch;
      }
   }
   if( bufp )
   {
      short * ptr = (short *)bufp;
      if( srcPortp->attenuatedB != 0 )
      {
         unsigned int gainVal = q16GainMap[srcPortp->attenuatedB];
         /* apply atteunation */
         gAttenuateFunc( ptr, ptr, (numBytesToDst/2), gainVal );
      }
      for( k=0; k < dstGroup->numConn; k++ )
      {
         dstPortp = &gMixerPorts[dstGroup->share.dst_grp.dst_lst[k]];

         if( dstPortp->inputp != NULL )
         {
            if( dstPortp->firstInputIsr )
            {
               /* just copy the samples over if this is the first input to this source */
               memcpy( dstPortp->inputp, bufp, dstPortp->numInBytes );
               dstPortp->firstInputIsr = 0;
            }
            else
            {
               /* not the first one to write to this source, have to add to the existing samples */
               gAdderFunc( (short *)dstPortp->inputp, (short *)dstPortp->inputp, (short *)bufp, (dstPortp->numInBytes/2) );
            }
         }
      }
   }
}
/****************************************************************************
*
*  halMixer_handleStereoInSamples - write stereo (interleaved) samples from source
*      to destination.  Currently resampling for interleaved stereo samples is not
*      supported (so just a straight copy will be involved).
*
***************************************************************************/
void halMixer_handleStereoInSamples( char * src_bufp, halMixer_port * dstGroup, halMixer_port_info *srcPortp )
{
   int k;
   halMixer_port_info * dstPortp;
   halMixer_resampler_info * resamplerInfo;
   resamplerInfo = (halMixer_resampler_info *)dstGroup->share.dst_grp.resampler_ptr;

   if( resamplerInfo == NULL )
   {
      /* currently resampling for stereo connection is not supported */
      for( k=0; k < dstGroup->numConn; k++ )
      {
         dstPortp = &gMixerPorts[dstGroup->share.dst_grp.dst_lst[k]];
                        
         if( dstPortp->firstInputIsr )
         {
            /* 2 times the samples needed to be copied for stereo connection */
            memcpy( dstPortp->inputp, src_bufp, (dstPortp->numInBytes * 2) );
            dstPortp->firstInputIsr = 0;
         }
         else
         {
            gAdderFunc( (short *)dstPortp->inputp, (short *)dstPortp->inputp, (short *)src_bufp, dstPortp->numInBytes );
         }
      }
   }
}
/****************************************************************************
*
*  halMixer_runSwitchboard
*
***************************************************************************/
void halMixer_runSwitchboard( int numBytes, short sampleFrequency )
{
   int i;
   char *src_bufp;
   /* go through SWB connection list, call callback to provide samples */
   halMixer_Connection *conn;
   halMixer_port_info *srcPortp, *dstPortp;
   int numOutputBytes;

   if( (sampleFrequency == 0) || (numBytes == 0) )
   {
      /* the swtichboad sample frequency is 0, meaning off, jut return here */
      return;
   }

   /* recalculate the number of input / output samples required if the switchboard is being 
    * called for different sampling frequency */
   if( (sampleFrequency != switchBoardFreq) || (numBytes != switchBoardBytes) )
   {
      switchBoardFreq = sampleFrequency;
      switchBoardBytes = numBytes;
      for( i=0; i < HALMIXER_MAX_PORT; i++ )
      {
         if( gMixerPorts[i].info.usrid != HAL_MIXER_USER_NONE )
         {
            halMixer_findPortNumBytes( i );
         }
      }
   }

   /* go through the callbacks of all the destination ports for input pointers */
   for( i=0; i < HALMIXER_MAX_PORT; i++ )
   {
      if( gMixerPorts[i].numInput )
      {
         dstPortp = &gMixerPorts[i];
         /* callback to get the destination buffer */
         dstPortp->inputp = (short *)dstPortp->info.callback.halmixer_get_inputptr( dstPortp->info.portIdx, dstPortp->numInBytes );
         dstPortp->firstInputIsr = 1;
      }
   }

   conn = (halMixer_Connection *)gMixerConnectionList.list;
   while( conn->src.numConn != -1 )
   {
      /* get the source buffer */
      srcPortp = &gMixerPorts[conn->src.share.srcPortId];
      numOutputBytes = srcPortp->numOutBytes;
      if( conn->src.connType >= HALMIXER_CONNECT_STEREO_SPLIT )
      {
         numOutputBytes *= 2;
      }
      /* get the output port from which samples can be copied from */
      src_bufp = srcPortp->info.callback.halmixer_get_outputptr( srcPortp->info.portIdx, numOutputBytes );
      if( src_bufp )
      {
         short * bufp;
         int k;
         i = 0;
         while( i < conn->src.numConn )
         {
            if( conn->src.connType == HALMIXER_CONNECT_MONO )
            {
               halMixer_handleMonoSamples( src_bufp, &conn->dst[i], srcPortp );
               i++;
            }
            else
            {
               /* stereo connection */
               if( conn->src.connType == HALMIXER_CONNECT_STEREO_SPLIT )
               {
                  if( srcPortp->numOutBytes <= (MIXER_MONO_BUFFERSIZE * 2)  )
                  {
                     bufp = (short *)src_bufp;
                     for(k=0; k < (srcPortp->numOutBytes/2); k++ )
                     {
                        leftInput[k] = bufp[0];
                        rightInput[k] = bufp[1];
                        if( leftInput[k] != rightInput[k] )
                        {
                           leqr = 0;
                        }
                        bufp+= 2;
                     }
                     /* playout the left and right channel */
                     halMixer_handleMonoSamples( (char *)leftInput, &conn->dst[i], srcPortp );
                     halMixer_handleMonoSamples( (char *)rightInput, &conn->dst[i+1], srcPortp );
                  }
                  i += 2;
               }
               else
               {
                  /* both the input and output type in stereo format, left and right channel samples are interleaved */
                  halMixer_handleStereoInSamples( src_bufp, &conn->dst[i], srcPortp );
                  i++;
               }
            }
         }
         if( srcPortp->info.callback.halmixer_outputRead != NULL )
         {
            srcPortp->info.callback.halmixer_outputRead( srcPortp->info.portIdx, numOutputBytes );
         }  
      }
      conn = (halMixer_Connection *)( &conn->dst[conn->src.numConn] );
   }

   /* all copying is done, tell all destination ports that input samples are ready */
   dstPortp = &gMixerPorts[0];
   for( i=0; i < HALMIXER_MAX_PORT; i++, dstPortp++ )
   {
      if( dstPortp->info.callback.halmixer_inputReady )
      {
         /* samples have been written */
         dstPortp->info.callback.halmixer_inputReady( dstPortp->info.portIdx, dstPortp->numInBytes );
      }
   }
}
/****************************************************************************
*
*  halMixer_queryPort - return the list of port numbers belong to the target
*     mixer user type to the caller
*
***************************************************************************/
int halMixer_queryPort( unsigned id, short *portList )
{
   int i, numPort = 0;

   for( i=0; i < HALMIXER_MAX_PORT; i++ )
   {
      if( gMixerPorts[i].info.usrid == id )
      {
         portList[numPort] = i;
         numPort++;
      }
      if( numPort >= HAL_MIXER_MAXNUM_PTYPE )
      {
         /* we have reached the maximum number of ports that can fit into this list
          * just return? */
         break;
      }
   }
   return( numPort );
}
/****************************************************************************
*
*  halMixer_queryPortInfo - return the port information of the specific port 
*     to the caller
*
***************************************************************************/
int halMixer_queryPortInfo( int id, HAL_MIXER_PORT_INFO * info )
{
   int rc = -1;
   /* make sure this port is valid */
   if( gMixerPorts[id].info.usrid != HAL_MIXER_USER_NONE )
   {
      /* return the port index of the query port */
      memcpy( info, &gMixerPorts[id].info, sizeof( HAL_MIXER_PORT_INFO ) );
      rc = 0;
   }
   return( rc );
}
/****************************************************************************
*
*  halMixer_changeSampFreq - change the input/output capability of the target port
*
***************************************************************************/
void halMixer_changeSampFreq( int id, unsigned int inFreq, unsigned int outFreq )
{
   gMixerPorts[id].info.input_capability = inFreq;
   gMixerPorts[id].info.output_capability = outFreq;
   halMixer_findPortNumBytes( id );
}

/****************************************************************************
*
*  halMixer_registerResampler - let the caller to register an efficient resampler
*
***************************************************************************/
void halMixer_registerResampler( RESAMPFUNC funcp )
{
   if( funcp != NULL )
   {
      gResampleFunc = funcp;
   }
}
/****************************************************************************
*
*  halMixer_deregisterResampler
*
***************************************************************************/
void halMixer_deregisterResampler( RESAMPFUNC funcp )
{
   if( funcp == gResampleFunc )
   {
      gResampleFunc = mixerResample;
   }
}
/****************************************************************************
*
*  halMixer_registerAdder - let the caller to register an efficient adder
*
***************************************************************************/
void halMixer_registerAdder( ADDERFUNC funcp )
{
   if( funcp != NULL )
   {
      gAdderFunc = funcp;
   }
}
/****************************************************************************
*
*  halMixer_deregisterAdder
*
***************************************************************************/
void halMixer_deregisterAdder( ADDERFUNC funcp )
{
   if( funcp == gAdderFunc )
   {
      gAdderFunc = mixerAdder;
   }
}
/****************************************************************************
*
*  halMixer_registerAttenuator - let the caller to register an efficient attenuator
*
***************************************************************************/
void halMixer_registerAttenuator( ATTENUFUNC funcp )
{
   if( funcp != NULL )
   {
      gAttenuateFunc = funcp;
   }
}
/****************************************************************************
*
*  halMixer_deregisterAttenuator
*
***************************************************************************/
void halMixer_deregisterAttenuator( ATTENUFUNC funcp )
{
   if( funcp == gAttenuateFunc )
   {
      gAttenuateFunc = mixerApplyAtten;
   }
}

/****************************************************************************
*
*  halMixer_registerStereoPartner
*
***************************************************************************/
int halMixer_registerStereoPartner( int self, int partner )
{
   int rc = -1;
   if( (self >= 0) && (partner >= 0))
   {
      /* check and make sure that these 2 ports are of the same type */
      if( (gMixerPorts[self].info.usrid != HAL_MIXER_USER_NONE) && 
          (gMixerPorts[partner].info.usrid != HAL_MIXER_USER_NONE) && 
          (gMixerPorts[partner].info.usrid == gMixerPorts[self].info.usrid) )
      {
         /* register each other as partner (left and right channel) for stereo playout */
         gMixerPorts[self].info.stereo_partner = partner;
         gMixerPorts[partner].info.stereo_partner = self;
         rc = 0;
      }
      else
      {
         HALMIXER_ERROR( ("stereo partner registration failed %d %d\n", self, partner ) );
      }
   }
   return rc;
}

/****************************************************************************
*
*  halMixer_open
*
***************************************************************************/
int halMixer_open( struct inode *inode, struct file *file )
{
   /* each client can directly access the switchboard connection list
    * or each port maintained by the mixer, 
    * so no client specific information needs to be stored */
   (void) inode;
   (void) file;

   return 0;
}

/****************************************************************************
*
*  halAudio_read
*
***************************************************************************/
ssize_t halMixer_read( struct file*file, char *buffer, size_t count, loff_t *ppos )
{
   /* not supported */
   return -EINVAL;
}

/****************************************************************************
*
*  halAudio_release
*
***************************************************************************/
int halMixer_release( struct inode *inode, struct file *file )
{
   (void) inode;
   (void) file;

   return 0;
}

/****************************************************************************
*
*  halAudio_write
*
***************************************************************************/
ssize_t halMixer_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
   /* note supported */
   return -EINVAL;
}
/****************************************************************************
*
*  halAudio_ioctl
*
***************************************************************************/
int halMixer_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   if( ( _IOC_TYPE( cmd ) != HALMIXER_MAGIC ) ||
       ( _IOC_NR( cmd ) < HALMIXER_CMD_FIRST ) ||
       ( _IOC_NR( cmd ) > HALMIXER_CMD_LAST ) )
   {
      return -ENOTTY;
   }

   switch( cmd )
   {
      case HALMIXER_IOCTL_QUERYPORT:
      {
         int numPorts;
         HALMIXER_QueryPort_t queryPort;

         if( copy_from_user( &queryPort, (HALMIXER_QueryPort_t *)arg, sizeof(queryPort) ) )
         {
            return -EFAULT;
         }
         numPorts = halMixer_queryPort( queryPort.userType, &queryPort.list[0] );
         queryPort.numPorts = numPorts;
         if( copy_to_user( (void *)arg, &queryPort, sizeof( queryPort ) ) )
         {
            return -EFAULT;
         }
      }
      break;

      case HALMIXER_IOCTL_ADDCONNECT:
      {
         HALMIXER_Connection_t connect;
         if( copy_from_user( &connect, (HALMIXER_Connection_t *)arg, sizeof( connect ) ) )
         {
            return -EFAULT;
         }
         halMixer_addConnect( &connect );
      }
      break;

      case HALMIXER_IOCTL_REMOVECONNECT:
      {
         HALMIXER_Connection_t connect;
         if( copy_from_user( &connect, (HALMIXER_Connection_t *)arg, sizeof( connect ) ) )
         {
            return -EFAULT;
         }
         halMixer_removeConnect( &connect );
      }
      break;

      case HALMIXER_IOCTL_ATTENUATE:
      {
         HALMIXER_Attenuate_t att;
         int attenuatedB;
         int rc = 0;
         if( copy_from_user( &att, (HALMIXER_Attenuate_t *)arg, sizeof(att) ) )
         {
            return -EFAULT;
         }
         if( gMixerPorts[att.src].info.usrid == HAL_MIXER_USER_NONE )
         {
            return EINVAL;
         }
         attenuatedB = gMixerPorts[att.src].attenuatedB + att.dB;

         if( (attenuatedB < 0) || (attenuatedB > MAX_Q16_ATTENUATE) )
         {
            HALMIXER_ERROR( ("intended attenuation exceded maximum %d\n", attenuatedB ) );
            rc = -1;
         }
         if( rc == 0 )
         {
            gMixerPorts[att.src].attenuatedB = attenuatedB;
         }
         att.resultingdB = gMixerPorts[att.src].attenuatedB;
         if( copy_to_user( (void *)arg, &att, sizeof( att ) ) )
         {
            return -EFAULT;
         }
         return rc;
      }
      break;

      case HALMIXER_IOCTL_QUERYSWB:
      {
         if( copy_to_user( (void *)arg, &gMixerConnectionList, sizeof( gMixerConnectionList ) ) )
         {
            return -EFAULT;
         }
      }
      break;

      case HALMIXER_IOCTL_QUERYPORTINFO:
      {
         HALMIXER_QueryPortInfo_t queryInfo;

         if( copy_from_user( &queryInfo, (HALMIXER_QueryPortInfo_t *)arg, sizeof( queryInfo ) ) )
         {
            return -EFAULT;
         }

         if( gMixerPorts[queryInfo.portNum].info.usrid != HAL_MIXER_USER_NONE )
         {
            memcpy( &queryInfo.portInfo, &gMixerPorts[queryInfo.portNum].info, sizeof( HAL_MIXER_PORT_INFO ) );
            if( copy_to_user( (void *)arg, &queryInfo, sizeof( queryInfo ) ) )
            {
               return -EFAULT;
            }
         }
      }
      break;

      default:
      {
         return -EFAULT;
      }
      break;
   }

   return 0;
}

/****************************************************************************
*
*  File operations (these are the device driver entry points)
*
***************************************************************************/
struct file_operations halMixer_fops =
{
   owner:   THIS_MODULE,
   open:    halMixer_open,
   release: halMixer_release,
   read:    halMixer_read,
   write:   halMixer_write,
   ioctl:   halMixer_ioctl
};

int __init halMixer_init_drv( void )
{
   int rc;
   printk( banner );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
   if ( gSysCtlHeader != NULL )
   {
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif
   if( ( rc = register_chrdev( BCM_HALMIXER_MAJOR, "halmixer", &halMixer_fops )) < 0 )
   {
      printk( KERN_ERR "halMixer: register_chrdev failed for major %d\n", BCM_HALMIXER_MAJOR );
      return rc;
   }

   halMixer_init( );
   return 0;
}

void __exit halMixer_exit_drv( void )
{
   printk( KERN_ERR "halMixer exit function\n" );
   if( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
   /* free all the buffers allocated */
   vfree(leftInput);
   vfree(rightInput);
   vfree(resample_scratch); 

   leftInput = NULL;
   rightInput = NULL;
   resample_scratch = NULL;
}
/***************************************************************************/

EXPORT_SYMBOL (halMixer_registerPort);
EXPORT_SYMBOL (halMixer_addConnect);
EXPORT_SYMBOL (halMixer_runSwitchboard);
EXPORT_SYMBOL (halMixer_removeConnect);
EXPORT_SYMBOL (halMixer_queryPort);
EXPORT_SYMBOL (halMixer_queryPortInfo);
EXPORT_SYMBOL (halMixer_changeSampFreq);
EXPORT_SYMBOL (halMixer_registerResampler);
EXPORT_SYMBOL (halMixer_deregisterResampler);
EXPORT_SYMBOL (halMixer_registerAdder);
EXPORT_SYMBOL (halMixer_deregisterAdder);
EXPORT_SYMBOL (halMixer_registerAttenuator);
EXPORT_SYMBOL (halMixer_deregisterAttenuator);
EXPORT_SYMBOL (halMixer_deregisterPort );
EXPORT_SYMBOL (halMixer_registerStereoPartner );

module_init( halMixer_init_drv );
module_exit( halMixer_exit_drv );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "Hal Mixer driver" );
