/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    vcfrmfwd.c
*
*  @brief   Frame forwarding Service Host Side implementation
*
****************************************************************************/

#include <linux/string.h>
//#include <ctype.h>

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

#include "vc_frmfwd_defs.h"
#include "vcfrmfwd.h"
#include "vclogging.h"
#include <linux/broadcom/vc.h>

/******************************************************************************
Global data.
******************************************************************************/

VC_FRMFWD_STATS_T g_vc_frmfwd_stats;

/******************************************************************************
Local types and defines.
******************************************************************************/

typedef struct
{
   int               initialized;
   int               inum;       /* fifo reference number */
   volatile uint32_t xid;        /* Received message ID */
   void              *in_ievent;
   void              *out_ievent;
   int               resp_code;
   void              *response_lock;
   VC_FFCALLBACK_T   callback;

} VC_FFGLOBALS_T;

/******************************************************************************
Static data.
******************************************************************************/

static VC_FFGLOBALS_T vc_ffg = {0, -1};
static void *frmfwd_lock = NULL;
static VC_ERRORlog_t gDiscardIngressError;

/******************************************************************************
Static functions.
******************************************************************************/

static int vc_ff_message_handler(void);

/*---------------------------------------------------------------------------*/

/***************************************************************************/
/**
*  Calculates a checksum of some data.
*/

uint16_t CalcChecksum( uint8_t *data, int length )
{
   uint32_t checksum = 0;
   int i;

   for ( i = 0; i < length; i++)
   {
      checksum += data[i];
   }

   return (uint16_t)checksum;

} // CalcChecksum

/***************************************************************************/
/**
*  Initialise the frame forward system for use.
*
*  @param1
*
*  @return   interface number
*
*  @remarks
*  A negative return value indicates failure (which may mean it has not been
*  started on VideoCore). Command call back function pointers must be reinitialized
*  after this is called.
*
*/

int vc_frmfwd_init (void)
{
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (frmfwd_lock == NULL)
      frmfwd_lock = vc_lock_create();

   vc_lock_obtain(frmfwd_lock);

   vc_ffg.inum = -1;
   vc_ffg.xid = 0;

   // call back functions for the two command messages
   vc_ffg.callback.frame_rcv = NULL;
   vc_ffg.callback.dec_ack_rcv = NULL;
   vc_ffg.callback.get_frame_buf = NULL;

   if (!vc_ffg.initialized) {
      vc_ffg.response_lock = vc_lock_create();
      vc_lock_obtain(vc_ffg.response_lock);
      vc_ffg.in_ievent = vc_event_create();
      vc_assert(vc_ffg.in_ievent);
      vc_ffg.out_ievent = vc_event_create();
      vc_assert(vc_ffg.out_ievent);

      vc_ffg.initialized = 1;
   }

   // We simply loop through every interface that there is and look for one
   // that claims to be a frame forward service.
   for (i = 0; i < VC_NUM_INTERFACES; i++)
   {
      if (vc_sharedmem_header.iface[i])
      {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface,
                                              vc_interface_base+vc_sharedmem_header.iface[i],
                                                                  sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_FRMFWD)
         {
            // Gotcha!
            vc_ffg.inum = i;
            vc_interface_register_event_int(vc_ffg.in_ievent, (1<<vc_ffg.inum));
            vc_interface_register_event_int(vc_ffg.out_ievent, (1<<vc_ffg.inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   memset( &gDiscardIngressError, 0, sizeof( VC_ERRORlog_t ) );
   vc_lock_release(frmfwd_lock);
   return vc_ffg.inum;
}

/***************************************************************************/
/**
*  Stop the frame forward service.
*
*  @param1
*
*  @return
*
*  @remarks
*  This tells us that the frmfwd service has stopped, thereby preventing
*  any of the functions from doing anything.
*
*/

void vc_frmfwd_stop (void) {
   // Don't want anyone to be using this when we zap it.
   vc_lock_obtain(frmfwd_lock);
   vc_ffg.inum = -1;
   vc_interface_register_event_int(vc_ffg.in_ievent, 0);
   vc_interface_register_event_int(vc_ffg.out_ievent, 0);
   vc_lock_release(frmfwd_lock);
}

/***************************************************************************/
/**
*  Return the frmfwd service number.
*
*  @param1
*
*  @return           interface number (-1 if not running).
*
*  @remarks
*
*/

int vc_frmfwd_inum (void) {
   return vc_ffg.inum;
}

/***************************************************************************/
/**
*  Set user callback functions.
*
*  @param1   callback   (int) pointer to callback function structure
*
*  @return   0 on success, < 0 on failure.
*
*  @remarks
*  This sets the user callback functions to be called whenever a send_frame or
*  decode_ack command is received.
*/

int vc_frmfwd_set_cmd_callback(VC_FFCALLBACK_T *callback)
{
   if (!vc_ffg.initialized)
      return -1;

   vc_lock_obtain(frmfwd_lock);
   vc_ffg.callback = *callback;
   vc_lock_release(frmfwd_lock);

   return 0;
}

/***************************************************************************/
/**
*  Send encoded video frame to the VC02 to be decoded
*
*  @param1   frame       (in)  points to frame structure
*  @param1   blocking    (in)  1 - if to block on not enough room in FIFO,
*                              0 - if to exit without blocking if not enough room
*
*  @return   response code: VC_RESP_OK or VC_RESP_ERROR
*
*  @remarks
*  Caller is responsible for releasing the frame buffer if necessary
*  This message does not require VC02 to generate a response
*  VC_RESP_ERROR indicates message was not sent successfully
*
*/

int vc_frmfwd_send_frame(FRMFWD_FRAME_T *frame, int blocking)
{
   int i;
   VC_MSGFIFO_CMD_HEADER_T hdr;
   int ael, ext_len, space_avail;
   int retval = VC_RESP_ERROR;
   uint8_t temp[16];
   int remainder;

   if (vc_ffg.inum < 0)
      return retval;

   // This lock stops the host task from maybe writing to our fifos until the
   // previous sent message has been read (e.g. to avoid contention with an
   // incoming message that must be answered).  Also, protects access to vc_ffg
   // to allow this function to be re-entrant.
   vc_lock_obtain(frmfwd_lock);

   i = vc_ffg.xid + 1;
   i &= 0x7fffffffUL;
   vc_ffg.xid = i; /* XXX assume this is atomic */

   ext_len = sizeof(frame->info)+frame->info.data_len+frame->info.offset;
   hdr.sync = VC_HTOV32(VC_CMD_SYNC);
   hdr.xid  = VC_HTOV32(vc_ffg.xid);
   hdr.cmd_code = VC_HTOV32(VC_FRMFWD_SEND_FRAME);
   hdr.ext_length = VC_HTOV16(ext_len);
   hdr.timestamp = VC_HTOV16(0);
   ael = (ext_len+15) & ~15;
   remainder=16-(ael-ext_len);

   if ( g_vc_frmfwd_stats.checksumEnabled )
   {
      frame->info.flags |= FRMFWD_HAS_CHECKSUM;
      frame->info.checksum = CalcChecksum( &frame->data[ frame->info.offset ], frame->info.data_len );

      g_vc_frmfwd_stats.framesSentWithChecksum++;
   }
   else
   {
      frame->info.flags &= ~FRMFWD_HAS_CHECKSUM;
      frame->info.checksum = 0;

      g_vc_frmfwd_stats.framesSentNoChecksum++;
   }

   vc_msgfifo_write_refresh(vc_ffg.inum);
   space_avail = vc_msgfifo_output_space_available(vc_ffg.inum);
   if (!blocking && space_avail < ((int)sizeof(hdr)+ael)) {
      printk("space avail = %d\t required = %d\n", space_avail, ((int)sizeof(hdr)+ael));
   }
   else
   {
      vc_msgfifo_write_blocking(vc_ffg.inum, &hdr, sizeof(hdr), vc_ffg.out_ievent);
#if defined( VC_HOST_IS_BIG_ENDIAN )
      {
          FRMFWD_FRAME_INFO_T   info = frame->info; // copy single byte fields
         //printk("E=%d,%d,%d,%d,%d,%d,%d,%d\n", frame->info.stream_num,frame->info.flags,frame->info.offset,  frame->info.reserved1, frame->info.reserved2, frame->info.seq_num, frame->info.timestamp,frame->info.data_len);

          //printk(  "FS: TS= %d\n",(unsigned int)frame->info.timestamp );
         info.seq_num           = VC_HTOV16( frame->info.seq_num );
         info.timestamp         = VC_HTOV32( frame->info.timestamp );
         info.data_len          = VC_HTOV32( frame->info.data_len );
         info.checksum          = VC_HTOV16( frame->info.checksum );

          vc_msgfifo_write_blocking( vc_ffg.inum, &info, sizeof( info ), vc_ffg.out_ievent);
          vc_msgfifo_write_blocking( vc_ffg.inum, frame->data, ael - sizeof( info)-16 , vc_ffg.out_ievent);
          memcpy(temp, frame->data+ael-sizeof(info)-16,remainder);
          vc_msgfifo_write_blocking( vc_ffg.inum, temp, 16, vc_ffg.out_ievent);
      }
#else
      vc_msgfifo_write_blocking(vc_ffg.inum, frame, ael-16, vc_ffg.out_ievent);
      memcpy(temp, frame->data+ael-16,remainder);
      vc_msgfifo_write_blocking( vc_ffg.inum, temp, 16, vc_ffg.out_ievent);
#endif
      vc_msgfifo_write_flush(vc_ffg.inum);
      retval = VC_RESP_OK;
   }

   // Give this so that the reading task can go.
   vc_lock_release(frmfwd_lock);

   return retval;
}

/***************************************************************************/
/**
*  Handle a frame forward message from the VC02
*
*  @param1
*
*  @return   1 if message is found, 0 if none, -1 if error obtaining buffer
*
*  @remarks
*
*/

void show_vc02( void );

static int vc_ff_message_handler(void)
{
   int got_one = 0;
   int isAudio = 0;
   FRMFWD_FRAME_INFO_T info;
   int stream_num;
   FRMFWD_FRAME_T *framep;

   if (vc_ffg.inum < 0)
     return 0;

   vc_msgfifo_read_refresh(vc_ffg.inum);

   if (vc_msgfifo_input_bytes_available(vc_ffg.inum) >= sizeof(VC_MSGFIFO_CMD_HEADER_T))
   {
      VC_MSGFIFO_CMD_HEADER_T hdr;
      uint32_t ael;

      got_one = 1;

      // Read the message header
      vc_msgfifo_read_blocking(vc_ffg.inum, &hdr,
                               sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_ffg.in_ievent);

      hdr.sync = VC_VTOH32( hdr.sync );

      if ( hdr.sync != VC_CMD_SYNC )
      {
         printk( KERN_ERR "*** vcfrmfwd: hdr.sync = 0x%08x, expecting VC_CMD_SYNC (0x%08x)\n",
                 hdr.sync, VC_CMD_SYNC );
         vc_dump_mem( "vcfrmfwd hdr", 0, &hdr, sizeof( hdr ));
         show_vc02();
      }
      vc_assert(hdr.sync == VC_CMD_SYNC);

      hdr.cmd_code = VC_VTOH32(hdr.cmd_code);
      hdr.ext_length = VC_VTOH16(hdr.ext_length);
      hdr.timestamp = VC_VTOH16(hdr.timestamp);
      hdr.xid = VC_VTOH32(hdr.xid);

      ael = (hdr.ext_length + 15) & ~15;
      //vc_assert(ael <= VC_FFMAX_CMD_DATA+sizeof(FRMFWD_FRAME_INFO_T));
      if (ael > VC_FFMAX_CMD_DATA+sizeof(FRMFWD_FRAME_INFO_T))
      {
         printk("Bad packet: packet larger than max supported packet size\n");
      }


      if (hdr.cmd_code == VC_FRMFWD_SEND_FRAME || hdr.cmd_code == VC_FRMFWD_ACK_FRAME || hdr.cmd_code == VC_FRMFWD_OUT_OF_BUFFERS)
      {
         // First read in the frame info to extract stream number
         vc_msgfifo_read_blocking(vc_ffg.inum, &info, sizeof(FRMFWD_FRAME_INFO_T), vc_ffg.in_ievent);

#if defined( VC_HOST_IS_BIG_ENDIAN )

         info.seq_num           = VC_VTOH16( info.seq_num );
         info.timestamp         = VC_VTOH32( info.timestamp );
         info.data_len          = VC_VTOH32( info.data_len );
         info.checksum          = VC_VTOH16( info.checksum );

#endif
         if( info.flags & 0x04 )
         {
            isAudio = 1;
         }
         //printk("I=%d,%d,%d,%d,%d,%d,%d,%d\n", info.stream_num,info.flags,info.offset,  info.reserved1, info.reserved2, info.seq_num, info.timestamp,info.data_len);
         stream_num = info.stream_num;

         if (hdr.cmd_code == VC_FRMFWD_SEND_FRAME)
         {
            // Process send frame message

            // Get a buffer and dispatch frame through callback function
            if( isAudio )
            {
#if defined( CONFIG_BCM_HALAUDIO_MIXER )
               framep = (vc_ffg.callback.get_audio_buf)(stream_num, ael-sizeof(info));
#else
               framep = NULL;
#endif
            }
            else
            {
               framep = (vc_ffg.callback.get_frame_buf)(stream_num, ael-sizeof(info));
            }
            if (framep == NULL)
            {
               vc_msgfifo_read_blocking(vc_ffg.inum, NULL, ael-sizeof(info), vc_ffg.in_ievent);
               vc_hostdrv_errlog( "Discard ingress", &gDiscardIngressError );
               got_one = -1;
            }
            else
            {
               uint8_t *next_write_pos;
               int bytes_to_read = 0;
               uint8_t temp_buf[32];

               VC_DEBUG( Trace, "framep = 0x%08x\n", (uint32_t)framep );

			   // first copy frame info
               memcpy(framep, &info, sizeof(info));
               next_write_pos = (uint8_t *)framep->data;

               /* offset should be less than 32 since the temp buffer has
                * a size of 32 bytes. If it is ever greater than 32,
                * the stack would get corrupted */

               if (info.offset > 32)
               {
                  printk("**********************Bad offset = %d ***************************\n", info.offset);
                  info.offset=0;

               }

               // discard bytes between end frame_info and start of payload
               // this is ugly stuff. number of bytes that can be read by msgfifo_read each time has to be multiples of VC_INTERFACE_BLOCK_SIZE
               if (info.offset > 0)
               {
                  // round up to multiple of VC_INTERFACE_BLOCK_SIZE bytes to be read
                  bytes_to_read =  (info.offset + (VC_INTERFACE_BLOCK_SIZE-1)) & ~(VC_INTERFACE_BLOCK_SIZE-1);
                  vc_msgfifo_read_blocking(vc_ffg.inum, temp_buf, bytes_to_read, vc_ffg.in_ievent);
                  // keep only those additional bytes that had to be read in order to round up to VC_INTERFACE_BLOCK_SIZE
                  memcpy(next_write_pos, &temp_buf[info.offset], bytes_to_read-info.offset);
                  next_write_pos += (bytes_to_read-info.offset);
               }

               framep->info.offset = 0;

               // read and copy remaining data into buffer and send to application
               // the app is responsible for releasing the buffer if necessary
               if (ael-sizeof(info)-bytes_to_read > 0)
               {
                  vc_msgfifo_read_blocking(vc_ffg.inum, next_write_pos, ael-sizeof(info)-bytes_to_read, vc_ffg.in_ievent);
               }
               //printk( "FR: TS = %d\n", framep->info.timestamp );
#if 0
               if( (info.data_len+sizeof(info)) < sizeof(info) )
               {
                  //debug check to ensure the length of the packet is valid
                  printk("frame error: data_len %d, sizeof(info) %d\n", info.data_len, sizeof(info) );
               }
               else
#endif
               {
                  if ( GET_FRMFWD_HAS_CHECKSUM( info.flags ))
                  {
                     uint16_t checksum;

                     // However, the data has already had info.offset bytes stripped out of
                     // it, so we need to use the adjusted framep->info.offset when indexing
                     // into the data.

                     checksum = CalcChecksum( &framep->data[ framep->info.offset ], framep->info.data_len );

                     if ( checksum == info.checksum )
                     {
                        g_vc_frmfwd_stats.framesRcvdChecksumGood++;
                     }
                     else
                     {
                        g_vc_frmfwd_stats.framesRcvdChecksumBad++;

                        printk( KERN_ERR "frmfwd: checksum failed; rcvd 0x%04x, expecting 0x%04x\n",
                                checksum, info.checksum );
                     }
                  }
                  else
                  {
                     g_vc_frmfwd_stats.framesRcvdNoChecksum++;
                  }
                  if( isAudio )
                  {
                     (vc_ffg.callback.audio_rcv)(framep, info.data_len + sizeof( info ), stream_num);
                  }
                  else
                  {
                     (vc_ffg.callback.frame_rcv)(framep, info.data_len + sizeof(info), stream_num);
                  }
               }
            }
         }
         else if (hdr.cmd_code == VC_FRMFWD_ACK_FRAME)
         {
            // Process decode ack message
            uint8_t junk[16];

            // read remaining bytes and throw away
            // henry: do we even need to do this?
            vc_assert(ael-sizeof(info) <= 16);
            vc_msgfifo_read_blocking(vc_ffg.inum, junk, ael-sizeof(info), vc_ffg.in_ievent);

            // dispatch frame info through callback function
            (vc_ffg.callback.dec_ack_rcv)(&info, stream_num);
         }
         else if (hdr.cmd_code == VC_FRMFWD_OUT_OF_BUFFERS)
         {
            // Process out of buffers message
            uint8_t junk[16];

            // read remaining bytes and throw away
            // henry: do we even need to do this?
            if ((ael-sizeof(info)) > 16)
            {
               vc_assert(0);
               //printk("OOFlen=%d\n", (ael-sizeof(info)));
            }
            if (ael-sizeof(info) > 0)
            {
               vc_msgfifo_read_blocking(vc_ffg.inum, junk, ael-sizeof(info), vc_ffg.in_ievent);
            }

            // dispatch frame info through callback function
            (vc_ffg.callback.dec_out_of_buffers_rcv)(&info, stream_num);
            got_one = 0;
         }
         else
         {
            /* Unknown message */
            vc_assert(0);
         }

         // finished with message, so update VC02's read pointer
         // NOTE: VC02 does not expect responses from host
         vc_msgfifo_read_flush(vc_ffg.inum);
      }
      else
      {
         // don't expect any other message coming from VC02
         // henry: instead of asserting could also just flush it
         vc_assert(0);
      }
   }
  return got_one;
}

/***************************************************************************/
/**
*  Public interface of message handler...
*
*  @param1
*
*  @return   0 if success, -1 if error obtaining buffer
*
*  @remarks
*
*/

int vc_frmfwd_poll_message_fifo(void)
{
   int retval=0;

   if (vc_ffg.inum < 0)
      return 0;

   do {
      retval = vc_ff_message_handler();
   } while (retval > 0);

   return retval;
}


/***************************************************************************/
/**
*  Get read event object
*
*  @param1
*
*  @return   pointer to read event object for the frame forwarding service
*
*  @remarks
*
*/

void *vc_frmfwd_read_event (void)
{
   return vc_ffg.in_ievent;
}

