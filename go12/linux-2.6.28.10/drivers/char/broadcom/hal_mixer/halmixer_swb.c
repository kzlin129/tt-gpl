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
*  halmixer_swb.c
*
*  PURPOSE:
*
*     This file contains the basic switchboard code for the HAL audio mixer
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/broadcom/halaudio_mixer.h>
#include <linux/broadcom/knllog.h>
#include "halmixer_priv.h"


/* ---- Public Variables ------------------------------------------------- */
/**
 * @addtogroup HAL
 * @{
 */
extern halMixer_port_info gMixerPorts[HALMIXER_MAX_PORT];
extern halMixer_Connection_List gMixerConnectionList;

/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */
/****************************************************************************
*
*  set_capabilities - determine the input/output sampling frequencies of the
*     destination and source
*
***************************************************************************/
int set_capabilities( unsigned int *in_cap, unsigned int *out_cap, halMixer_resampler_info *resamp )
{
   int rc = 0;
   unsigned short cap = (*in_cap & *out_cap);

   if( cap == 0 )
   {
      /* no common sampling frequencies */
      /* find out if they can match by down sampling */
      if( *in_cap & HAL_MIXER_SIGTYPE_16K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_16K;

         if( *out_cap & HAL_MIXER_SIGTYPE_48K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_48K;
            resamp->decimRatio = 3;
            resamp->interpRatio = 1;
            resamp->coefficient = decim48to16fir;
            resamp->filterlen = DECIM48TO16TAPLEN;
            rc = 1;
         }
         else if( *out_cap & HAL_MIXER_SIGTYPE_32K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_32K;
            resamp->decimRatio = 2;
            resamp->interpRatio = 1;
            resamp->coefficient = decim32to16fir;
            resamp->filterlen = DECIM32TO16TAPLEN;
            rc = 1;
         }
         else if( *out_cap & HAL_MIXER_SIGTYPE_8K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_8K;
            resamp->decimRatio = 1;
            resamp->interpRatio = 2;
            resamp->coefficient = resamp8to16fir;
            resamp->filterlen = RESAMP8TO16FILTLEN;
            rc = 1;
         }
         else
         {
            rc = -1;
         }
      }
      else if( *in_cap & HAL_MIXER_SIGTYPE_32K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_32K;
         if( *out_cap & HAL_MIXER_SIGTYPE_48K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_48K;
            resamp->decimRatio = 3;
            resamp->interpRatio = 2;
            resamp->coefficient = resamp48to32fir;
            resamp->filterlen = RESAMP48TO32FILTLEN;
            rc = 1;
         }
         else if( *out_cap & HAL_MIXER_SIGTYPE_16K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_16K;
            resamp->decimRatio = 1;
            resamp->interpRatio = 2;
            resamp->coefficient = resamp8to16fir;
            resamp->filterlen = RESAMP8TO16FILTLEN;
            rc = 1;
         }
         else
         {
            rc = -1;
         }
      }
      else if( *in_cap & HAL_MIXER_SIGTYPE_40K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_40K;
         if( *out_cap & HAL_MIXER_SIGTYPE_48K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_48K;
            resamp->decimRatio = 6;
            resamp->interpRatio = 5;
            resamp->coefficient = resamp48to40fir;
            resamp->filterlen = RESAMP48TO40FILTLEN;
            rc = 1;
         }
         else if( *out_cap & HAL_MIXER_SIGTYPE_32K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_32K;
            resamp->decimRatio = 4;
            resamp->interpRatio = 5;
            resamp->coefficient = resamp32to40fir;
            resamp->filterlen = RESAMP32TO40FILTLEN;
            rc = 1;
         }
         else
         {
            rc = -1;
         }
      }
      else if( *in_cap & HAL_MIXER_SIGTYPE_8K )
      {
         /* note, we are reusing the 32 to 16K down sampler for the 16 to 8K down sampler */
         *in_cap = HAL_MIXER_SIGTYPE_8K;
         if( *out_cap & HAL_MIXER_SIGTYPE_16K )
         {
            *out_cap = HAL_MIXER_SIGTYPE_16K;
            resamp->decimRatio = 2;
            resamp->interpRatio = 1;
            resamp->coefficient = decim32to16fir;
            resamp->filterlen = DECIM32TO16TAPLEN;
            rc = 1;
         }
         else
         {
            rc = -1;
         }
      }
      else
      {
         rc = -1;
      }
   }
   else
   {
      /* if there is a matching capability, just use it, no resampler will be involved */
      if( cap & HAL_MIXER_SIGTYPE_48K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_48K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_44K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_44K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_40K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_40K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_32K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_32K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_22K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_22K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_16K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_16K;
      }
      else if( cap & HAL_MIXER_SIGTYPE_8K )
      {
         *in_cap = HAL_MIXER_SIGTYPE_8K;
      }
      else
      {
         rc = -1;
      }
      *out_cap = *in_cap;
   }
   return rc;
}
/****************************************************************************
*
*  halMixer_writeDstGrp - fill in the info of the destination group(s) in a
*     switchboard connection
*
***************************************************************************/
void halMixer_writeDstGrp( halMixer_port * insertPort, 
                           HALMIXER_Connection_t *connectp, 
                           halMixer_resampler_info * resampInfo,
                           int decimNeeded,
                           int dst_cap )
{
   halMixer_port * insertPort_2 = NULL;
               
   insertPort->share.dst_grp.resampler_ptr = NULL;
   insertPort->numConn = 1;
   insertPort->share.dst_grp.dst_lst[0] = connectp->dst_1;
   insertPort->connType = dst_cap;
   gMixerPorts[connectp->dst_1].inType = dst_cap;
   gMixerPorts[connectp->dst_1].numInput++;

   if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
   {
      insertPort_2 = &insertPort[1];

      insertPort_2->numConn = 1;
      insertPort_2->share.dst_grp.resampler_ptr = NULL;
      insertPort_2->share.dst_grp.dst_lst[0] = connectp->dst_2;
      insertPort_2->connType = dst_cap;
      gMixerPorts[connectp->dst_2].inType = dst_cap;
      gMixerPorts[connectp->dst_2].numInput++;
   }

   if( decimNeeded > 0 )
   {  
      /* resampler needed in this connection */
      /* allocate the resampler buffer */
      char * tmp = vmalloc( sizeof(halMixer_resampler_info) + 2000 );

      resampInfo->resampbufp = (short *)(tmp + sizeof( halMixer_resampler_info ) );
      memset( resampInfo->resampbufp, 0, 2000 );
      memcpy( tmp, resampInfo, sizeof( halMixer_resampler_info ) );

      insertPort->share.dst_grp.resampler_ptr = tmp;
      
      if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
      {
         tmp = vmalloc( sizeof( halMixer_resampler_info) + 2000 );

         resampInfo->resampbufp = (short *)(tmp + sizeof( halMixer_resampler_info ));
         memset( resampInfo->resampbufp, 0, 2000 );
         memcpy( tmp, resampInfo, sizeof( halMixer_resampler_info ) );
         insertPort_2->share.dst_grp.resampler_ptr = tmp;
      }  
   }
}
/****************************************************************************
*
*  halMixer_ConnectPort_simplex - make a simplex switchboard connection
*
***************************************************************************/
int halMixer_ConnectPort_simplex( HALMIXER_Connection_t * connectp )
{
   halMixer_Connection * conn = NULL;
   int i, k, rc = -1, decimNeeded, newConnNeeded;
   int src, dst_1, dst_2;
   unsigned int src_cap, dst_cap; 
   halMixer_resampler_info resampler_tmp;

   src = connectp->src;
   dst_1 = connectp->dst_1;
   dst_2 = connectp->dst_2;

   src_cap = gMixerPorts[src].info.output_capability;
   dst_cap = gMixerPorts[dst_1].info.input_capability;

   if( gMixerPorts[dst_1].inType != 0 )
   {
      /* this port is already busy as a dst, use the existing input frequency */
      dst_cap = gMixerPorts[dst_1].inType;
   }
   if( gMixerPorts[src].outType != 0 )
   {
      /* src port already in use, so cannot change the sampling rate */
      src_cap = gMixerPorts[src].outType;
   }
   /* dst 2 is only being used in a stereo connection. so it can be negative number (for mono) */
   if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
   {
      if( (dst_2 >= 0) && (gMixerPorts[dst_1].info.stereo_partner == dst_2) )
      {
         /* have to make sure that dst 1 and dst 2 cani be used as the left and right channel together */
         if( gMixerPorts[dst_2].inType )
         {
            dst_cap = ( dst_cap & gMixerPorts[dst_2].inType );
         }
         else
         {
            dst_cap = ( dst_cap & gMixerPorts[dst_2].info.input_capability );
         }
      }
      else
      {
         HALMIXER_ERROR( ("halaudio_mixer : ERROR right channel not specified for stereo connection %d %d\n",
                  dst_1, dst_2) );
         return rc;
      }
   }
   else if( connectp->connType == HALMIXER_CONNECT_STEREO_INTERLEAVE )
   {
      /* we do not support resampling for stereo interleave type connection */
      dst_cap = (dst_cap & src_cap);
   }

   HALMIXER_PRINTK( ("halaudio_mixer : connect simplex %d %d %d cap %d %d\n", src, dst_1, dst_2, src_cap, dst_cap) );
   decimNeeded = set_capabilities( &dst_cap, &src_cap, &resampler_tmp );

   if( decimNeeded < 0 )
   {
      HALMIXER_ERROR( ("halaudio_mixer : ERROR, src and dst capability doesn't match\n") );
      return rc;
   }
   /* safety check only */
   if( connectp->connType == HALMIXER_CONNECT_STEREO_INTERLEAVE )
   {
      if( decimNeeded > 0 )
      {
         HALMIXER_ERROR( ("halaudio_mixer: ERROR, resampling not supported for stereo interleave connection\n") );
         return rc;
      }
   }
   newConnNeeded = 1;

   /* protect critical section (SWB) */ 
   if( gMixerPorts[src].outType != 0 )
   {
      /* the source is already in the swb connection list, look it up from the swb connection list */
      conn = (halMixer_Connection *)gMixerConnectionList.list;
      while( conn->src.numConn != -1 )
      {
         if( conn->src.share.srcPortId == src )
         {
            if( conn->src.connType != connectp->connType )
            {
               if( (conn->src.connType == HALMIXER_CONNECT_MONO) ||
                   (connectp->connType == HALMIXER_CONNECT_MONO) )
               {
                  HALMIXER_ERROR( ("halaudio_mixer : ERROR, trying to use the same port %d for mono and stereo output\n", src) );
                  /* maybe we should return an error here */
               }
            }
            else
            {
               /* matching found, just add to the existing list */
               newConnNeeded = 0;
               break;
            }
         }
         conn = (halMixer_Connection *)( &conn->dst[conn->src.numConn] );
      }
   }
   if( newConnNeeded == 0 )
   {
      halMixer_port * connDstGp;
      int * dstPortIdp;
      int found = -1;
      /* check if there is already the same type of connection with another destination (broadcasting) */
      for( i=0; i < conn->src.numConn; i++ )
      {
         connDstGp = &conn->dst[i];
         dstPortIdp = connDstGp->share.dst_grp.dst_lst;
         /* make sure that we are not already in the connection list */
         /* this for loop is actually just for safety sake to avoid same connection being added twice */
         for( k=0; k < connDstGp->numConn; k++ )
         {
            if( dstPortIdp[k] == dst_1 )
            {
               HALMIXER_ERROR( ("halaudio_mixer : ERROR, connection already in place\n") );
               return rc;
            }
         }
         /* each dst group can only hold 3 port number, have to create a new group if the existing list is already full */
         if( (found <0) && (connDstGp->connType == dst_cap) && (connDstGp->numConn < HALMIXER_NUMDSTP_PERGP) )
         {
            found = i;
         }
      }
      if( found >= 0 )
      {
         halMixer_port * connDstGp = &conn->dst[found];
         /* the same connection type (resampler ratio) has found */
         /* now if this is a stereo connection, there must be another dst group with the same connection type */
         if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
         {
            if( conn->dst[found+1].connType != dst_cap )
            {
               /* cannot find the right channel */
               HALMIXER_ERROR( ("halaudio_mixer : ERROR, right channel not found for stereo connection, src %d connType = %d cap %d\n", src, connDstGp->connType, dst_cap) );
               return rc;
            }
            else
            {
               /* matching right channel found */
               halMixer_port * connDstGp_right = &conn->dst[found+1];
               connDstGp_right->share.dst_grp.dst_lst[connDstGp_right->numConn] = dst_2;
               connDstGp_right->numConn++;
               gMixerPorts[dst_2].inType = dst_cap;
               gMixerPorts[dst_2].numInput++;
            }
         }

         /* add to the current group */
         connDstGp->share.dst_grp.dst_lst[connDstGp->numConn] = dst_1;
         connDstGp->numConn++;
         HALMIXER_PRINTK(("found in swb src port %d, dst port there %d total now %d\n", conn->src.share.srcPortId, 
               connDstGp->share.dst_grp.dst_lst[connDstGp->numConn-2], connDstGp->numConn));

         /* connection succeed */
         gMixerPorts[dst_1].inType = dst_cap;
         gMixerPorts[dst_1].numInput++;
         rc = 0;
      }
      else
      {
         /* no existing connection type share the same resampler, create a new dst group */
         /* insert this new destination group in the middle of the connection list */
         halMixer_Connection_List * connectionList = &gMixerConnectionList;
         int numDstGp = 1;
         if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
         {
            numDstGp ++;
         }
         if( (connectionList->portsUsed + numDstGp) < HALMIXER_SWBARRAY_LEN )
         {
            halMixer_port * insertPort = &conn->dst[conn->src.numConn];
            int portsToMove = &connectionList->list[connectionList->portsUsed + numDstGp] - insertPort;

            /* create a space for the new dst group */
            memmove( insertPort+numDstGp, insertPort, portsToMove * sizeof( halMixer_port ) );

            conn->src.numConn += numDstGp;
            /* fill in the dst group information */
            halMixer_writeDstGrp( insertPort, connectp, &resampler_tmp,decimNeeded, dst_cap );

            connectionList->portsUsed += numDstGp;
               
            /* connection added successfully */
            rc = 0;
         }
      }
   }
   else
   {
      /* first connection being made for this particular source port */
      /* make sure the SWB list has space */
      halMixer_Connection_List * connectionList = &gMixerConnectionList;
      int numDstGp = 2;

      if( connectp->connType == HALMIXER_CONNECT_STEREO_SPLIT )
      {
         numDstGp ++;
      }
      if( (connectionList->portsUsed + numDstGp) < HALMIXER_SWBARRAY_LEN )
      {
         conn = (halMixer_Connection *)&connectionList->list[connectionList->portsUsed];

         conn->src.share.srcPortId = src;
         conn->src.numConn = numDstGp-1;
         conn->src.connType = connectp->connType;

         /* fill in the dst group information */
         halMixer_writeDstGrp( &conn->dst[0], connectp, &resampler_tmp, decimNeeded, dst_cap );


         conn->dst[numDstGp-1].numConn = -1;

         connectionList->portsUsed += numDstGp;
         rc = 0;
         gMixerPorts[src].outType = src_cap;
      }
   }
   /* update the number of input/output samples that has to be moved when the switchboard runs */
   halMixer_findPortNumBytes( connectp->src );
   halMixer_findPortNumBytes( connectp->dst_1 );
   if( connectp->dst_2 >= 0 )
   {
      halMixer_findPortNumBytes( connectp->dst_2 );
   }

   /* restore critical section */
   return rc;
}
/****************************************************************************
*
*  halMixer_DisconnectPort_simplex - remove a simplex connection from the switchboard
*     connection list
*
***************************************************************************/
int halMixer_DisconnectPort_simplex( halMixer_Connection * conn, int src, int dst )
{
   int i, rc = -1;
   halMixer_port *dstPortGp = NULL;
   halMixer_Connection_List *connectionList = &gMixerConnectionList;
   
   HALMIXER_PRINTK( ("halaudio_mixer : disconnect simplex %d %d\n", src, dst) );
   /* the connection has been found from the switchboard connection list before this
    * function is being called, so just return if this is not the right connection */
   if( conn->src.share.srcPortId != src )
   {
      HALMIXER_ERROR(( "halaudio_mixer : disconnect incorrect connection specified\n") );
      return rc;
   }
   /* connection found, remove the connection */
   /* now have to find which dst group this dst port belongs to */
   for( i=0; i < conn->src.numConn; i++ )
   {
      if( conn->dst[i].connType == gMixerPorts[dst].inType )
      {
         dstPortGp = &conn->dst[i];
         break;
      }
   }
   if( dstPortGp )
   {
      halMixer_port * dstp = NULL, * srcp = NULL;
      /* the dst group is found, check if this is the only one in this group */
      if( dstPortGp->numConn == 1 )
      {
         /* only 1 port in this group, remove the whole group */
         if( dstPortGp->share.dst_grp.resampler_ptr )
         {
            halMixer_resampler_info * ptr = (halMixer_resampler_info *)dstPortGp->share.dst_grp.resampler_ptr;
            /* free the resampler buffers */
            vfree( ptr );
         }

         if( conn->src.numConn == 1 )
         {
            /* only 1 dst group linked to this src, remove the src port as well */
            gMixerPorts[src].outType = 0;
            connectionList->portsUsed -= 2;
            dstp = &conn->src;
            srcp = &dstp[2];
         }
         else
         {
            /* just remove this dst group */
            connectionList->portsUsed --;
            dstp = dstPortGp;
            srcp = &dstPortGp[1];
            conn->src.numConn--;
         }
         memmove( dstp, srcp, (&connectionList->list[ connectionList->portsUsed + 1 ] - dstp ) * sizeof(*dstp) );
      }
      else
      {
         int * listrp = dstPortGp->share.dst_grp.dst_lst;
         int * listwp = listrp;
         int writeidx = 0;
         /* there are other destination in this group, only have to remove from the dst list */
         for( i=0; i < dstPortGp->numConn; i++ )
         {
            if( listrp[i] != dst )
            {
               listwp[writeidx++] = listrp[i];
            }
         }
         dstPortGp->numConn--;
      }
      gMixerPorts[dst].numInput--;
      if( gMixerPorts[dst].numInput == 0 )
      {
         gMixerPorts[dst].inType = 0;
      }
      rc = 0;
   }
   else
   {
      HALMIXER_ERROR( ("halaudio_mixer : connection for src %d and dst %d not found\n", src, dst) );
   }
   return rc;
}
/****************************************************************************
*
*  halMixer_switchboardRemovePort - remove a port from the switchboard
*     all connections associated with this port has to be removed
*
***************************************************************************/
void halMixer_switchboardRemovePort( int port )
{
   halMixer_Connection *conn;
   int i, k;
   int dst;

   if( gMixerPorts[port].outType != 0 )
   {
      /* this port is still busy as a src port */
      conn = (halMixer_Connection *)gMixerConnectionList.list;
      while( conn->src.numConn != -1 )
      {
         if( conn->src.share.srcPortId == port )
         {
            for( i=0; i < conn->src.numConn; i++ )
            {
               for( k=0; k < conn->dst[i].numConn; k++ )
               {
                  dst = conn->dst[i].share.dst_grp.dst_lst[k];
                  halMixer_DisconnectPort_simplex( conn, port, dst );
               }
            }
         }
         conn = (halMixer_Connection *)&conn->dst[conn->src.numConn];
      }
   }
   if( gMixerPorts[port].inType != 0 )
   {
      /* this port is still busy as a dst port */
      conn = (halMixer_Connection *)gMixerConnectionList.list;
      while( conn->src.numConn != -1 )
      {
         for( i=0; i < conn->src.numConn; i++ )
         {
            if( conn->dst[i].connType == gMixerPorts[port].inType )
            {
               for( k=0; k < conn->dst[i].numConn; k++ )
               {
                  if( port == conn->dst[i].share.dst_grp.dst_lst[k] )
                  {
                     halMixer_DisconnectPort_simplex( conn, conn->src.share.srcPortId, port );
                  }
               }
            }
         }
         conn = (halMixer_Connection *)&conn->dst[conn->src.numConn];
      }
   }
} 

