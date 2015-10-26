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




//#include <stdlib.h>

#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcmsgfifo.h"

#include <linux/broadcom/vc.h>

/******************************************************************************
Global data.
*****************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

typedef struct {
   // "in" and "out" here still refer to VideoCore's view of things.
   int in_size;
   int out_size;
   int bytes_in_infifo;
   int vin_fwptr;
   int vout_frptr;
} VC_MSGFIFO_DATA_T;

/******************************************************************************
Static data.
******************************************************************************/

VC_MSGFIFO_INTERFACE_T msgfifo_interface[VC_NUM_INTERFACES];
static VC_MSGFIFO_DATA_T msgfifo_data[VC_NUM_INTERFACES];
static int msgfifo_xid[VC_NUM_INTERFACES];

/******************************************************************************
Static functions.
******************************************************************************/

static int compute_bytes_in_fifo(int fmin, int fmax, int frptr, int fwptr);

/******************************************************************************
NAME
   vc_msgfifo_init

SYNOPSIS
   int vc_msgfifo_init(int inum)

FUNCTION
   Initialise the numbered msgfifo interface for use. Return non-zero for failure.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_init (int inum) {
   int status;
   VC_MSGFIFO_INTERFACE_T *msgfifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   msgfifo = &msgfifo_interface[inum];

   // No particular harm in reading the whole lot.
   status = vc_host_read_consecutive(  msgfifo, vc_interface_base + vc_sharedmem_header.iface[inum], sizeof(VC_MSGFIFO_INTERFACE_T), 0);
   vc_assert(status == 0);

   VC_DEBUG( MsgFifo, "Interface: %d\n", inum );
   VC_DEBUG( MsgFifo, "     itype: 0x%04x\n", VC_VTOH16( msgfifo->itype ));
   VC_DEBUG( MsgFifo, "      sver: 0x%04x\n", VC_VTOH16( msgfifo->sver ));
   VC_DEBUG( MsgFifo, "     stype: 0x%04x\n", VC_VTOH16( msgfifo->stype ));
   VC_DEBUG( MsgFifo, "  vin_fmin: 0x%04x\n", VC_VTOH16( msgfifo->vin_fmin ));
   VC_DEBUG( MsgFifo, "  vin_fmax: 0x%04x\n", VC_VTOH16( msgfifo->vin_fmax ));
   VC_DEBUG( MsgFifo, " vout_fmin: 0x%04x\n", VC_VTOH16( msgfifo->vout_fmin ));
   VC_DEBUG( MsgFifo, " vout_fmax: 0x%04x\n", VC_VTOH16( msgfifo->vout_fmax ));
   VC_DEBUG( MsgFifo, " vin_frptr: 0x%04x\n", VC_VTOH16( msgfifo->vin_frptr ));
   VC_DEBUG( MsgFifo, " vin_fwptr: 0x%04x\n", VC_VTOH16( msgfifo->vin_fwptr ));
   VC_DEBUG( MsgFifo, "vout_frptr: 0x%04x\n", VC_VTOH16( msgfifo->vout_frptr ));
   VC_DEBUG( MsgFifo, "vout_fwptr: 0x%04x\n", VC_VTOH16( msgfifo->vout_fwptr ));

   // Initialise VC_MSGFIFO_DATA_T for this interface.
   msgfifo_data[inum].out_size = VC_VTOH16(msgfifo->vout_fmax) - VC_VTOH16(msgfifo->vout_fmin) - VC_INTERFACE_BLOCK_SIZE;
   msgfifo_data[inum].in_size = VC_VTOH16(msgfifo->vin_fmax) - VC_VTOH16(msgfifo->vin_fmin) - VC_INTERFACE_BLOCK_SIZE;
   msgfifo_data[inum].bytes_in_infifo =
      compute_bytes_in_fifo(VC_VTOH16(msgfifo->vin_fmin), VC_VTOH16(msgfifo->vin_fmax),
                            VC_VTOH16(msgfifo->vin_frptr), VC_VTOH16(msgfifo->vin_fwptr));

   // Must initialise vin_fwptr.
   vc_msgfifo_read_refresh(inum);
   vc_msgfifo_write_refresh(inum);
   msgfifo_data[inum].vin_fwptr = VC_VTOH16(msgfifo_interface[inum].vin_fwptr);
   msgfifo_data[inum].vout_frptr = VC_VTOH16(msgfifo_interface[inum].vout_frptr);

   return 0;
}

/******************************************************************************
NAME
   vc_msgfifo_input_bytes_available

SYNOPSIS
   int vc_msgfifo_input_bytes_available(int inum)

FUNCTION
   Return the number of bytes available in our input fifo (VideoCore's output fifo).
   Does not reload the fifo pointers.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_input_bytes_available (int inum) {
   VC_MSGFIFO_INTERFACE_T *msgfifo = &msgfifo_interface[inum];
   int bytes_available;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   bytes_available = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vout_fmin), VC_VTOH16(msgfifo->vout_fmax),
                                           VC_VTOH16(msgfifo->vout_frptr), VC_VTOH16(msgfifo->vout_fwptr));
   return bytes_available;
}

/******************************************************************************
NAME
   vc_msgfifo_input_space_available

SYNOPSIS
   int vc_msgfifo_input_space_available(int inum)

FUNCTION
   Return the number of bytes available space in our input fifo (VideoCore's output fifo).
   Does not reload the fifo pointers.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_input_space_available (int inum) {
   VC_MSGFIFO_INTERFACE_T *msgfifo = &msgfifo_interface[inum];
   int bytes_available;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   bytes_available = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vout_fmin), VC_VTOH16(msgfifo->vout_fmax),
                                           VC_VTOH16(msgfifo->vout_frptr), VC_VTOH16(msgfifo->vout_fwptr));
   return msgfifo_data[inum].out_size - bytes_available;
}

/******************************************************************************
NAME
   vc_msgfifo_output_bytes_available

SYNOPSIS
   int vc_msgfifo_output_bytes_available(int inum)

FUNCTION
   Return the number of bytes in our output fifo (VideoCore's input fifo) unread by
   VideoCore. Does not reload the fifo pointers.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_output_bytes_available (int inum) {
   VC_MSGFIFO_INTERFACE_T *msgfifo = &msgfifo_interface[inum];
   int bytes_in_fifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   bytes_in_fifo = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vin_fmin), VC_VTOH16(msgfifo->vin_fmax),
                                         VC_VTOH16(msgfifo->vin_frptr), VC_VTOH16(msgfifo->vin_fwptr));
   return bytes_in_fifo;
}

/******************************************************************************
NAME
   vc_msgfifo_output_space_available

SYNOPSIS
   int vc_msgfifo_output_space_available(int inum)

FUNCTION
   Return the number of bytes available space in our output fifo (VideoCore's input fifo).
   Does not reload the fifo pointers.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_output_space_available (int inum) {
   VC_MSGFIFO_INTERFACE_T *msgfifo = &msgfifo_interface[inum];
   int bytes_in_fifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   bytes_in_fifo = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vin_fmin), VC_VTOH16(msgfifo->vin_fmax),
                                         VC_VTOH16(msgfifo->vin_frptr), VC_VTOH16(msgfifo->vin_fwptr));
   return msgfifo_data[inum].in_size - bytes_in_fifo;
}

/******************************************************************************
NAME
   vc_msgfifo_read_refresh

SYNOPSIS
   int vc_msgfifo_read_refresh(int inum)

FUNCTION
   Reload the read fifo pointers that get set by VideoCore. Return non-zero for failure.
   (i.e. read VideoCore's vout_fwptr)

RETURNS
   int
******************************************************************************/

int vc_msgfifo_read_refresh (int inum) {
   int status;
   int offset;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   // Re-read vout_fwptr.
   offset = (uint32_t)&(msgfifo_interface[inum].vout_fwptr) - (uint32_t)&msgfifo_interface[inum];
   status = vc_host_read_consecutive(&(msgfifo_interface[inum].vout_fwptr),
                                     vc_interface_base + vc_sharedmem_header.iface[inum] + offset, VC_INTERFACE_BLOCK_SIZE, 0);
   vc_assert(status == 0);

   VC_DEBUG( MsgFifo, "vout_fwptr: 0x%04x\n", VC_VTOH16( msgfifo_interface[inum].vout_fwptr ));

   return 0;
}

/******************************************************************************
NAME
   vc_msgfifo_write_refresh

SYNOPSIS
   int vc_msgfifo_write_refresh(int inum)

FUNCTION
   Reload VideoCore's pointer for our output fifo (vin_frptr). Return non-zero for failure.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_write_refresh (int inum) {
   int status;
   int offset;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   // Re-read vin_frptr.
   offset = (uint32_t)&(msgfifo_interface[inum].vin_frptr) - (uint32_t)&msgfifo_interface[inum];
   status = vc_host_read_consecutive(&(msgfifo_interface[inum].vin_frptr),
                                     vc_interface_base + vc_sharedmem_header.iface[inum] + offset, VC_INTERFACE_BLOCK_SIZE, 0);
   vc_assert(status == 0);

   VC_DEBUG( MsgFifo, "vin_frptr: 0x%04x\n", VC_VTOH16( msgfifo_interface[inum].vin_frptr ));

   return 0;
}

/******************************************************************************
NAME
   vc_msgfifo_read_consecutive

SYNOPSIS
   int vc_msgfifo_read_consecutive(int inum, void *host_addr, int nbytes)

FUNCTION
   Read up to nbytes from our input fifo (VideoCore's output fifo). The number of
   bytes read is returned, which may be fewer than nbytes, or even 0 (the function
   does not block waiting for input to arrive). Negative return values indicate
   failure. Note that this function does not refresh the fifo pointers, nor does
   it "flush" the read pointer back to VideoCore.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_read_consecutive (int inum, void *host_addr, int nbytes) {
   int status;
   int bytes_to_read;
   int total_bytes_read = 0;
   VC_MSGFIFO_INTERFACE_T *msgfifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   msgfifo = &msgfifo_interface[inum];

   if (VC_VTOH16(msgfifo->vout_fwptr) < VC_VTOH16(msgfifo->vout_frptr)) {
      // Read bytes from frptr to fmax.
      bytes_to_read = VC_VTOH16(msgfifo->vout_fmax) - VC_VTOH16(msgfifo->vout_frptr);
      if (bytes_to_read > nbytes)
         bytes_to_read = nbytes;

      VC_DEBUG( MsgFifo,  "vout_frptr: 0x%04x bytes_to_read: %d\n", VC_VTOH16(msgfifo->vout_frptr), bytes_to_read );

      if (host_addr != NULL)
      {
         status = vc_host_read_consecutive(host_addr, vc_interface_base + VC_VTOH16(msgfifo->vout_frptr), bytes_to_read, 0);
         vc_assert(status == 0);
         host_addr = (void *)((unsigned char *)host_addr + bytes_to_read);
      }
      total_bytes_read += bytes_to_read;
      nbytes -= bytes_to_read;
      msgfifo->vout_frptr = VC_HTOV16( ((VC_VTOH16(msgfifo->vout_frptr)) + bytes_to_read) );
      if (msgfifo->vout_frptr == msgfifo->vout_fmax) {
         // We got to the end of the fifo. There may be more to read.
         msgfifo->vout_frptr = msgfifo->vout_fmin;
      }
   }

   if (VC_VTOH16(msgfifo->vout_fwptr) > VC_VTOH16(msgfifo->vout_frptr)) {
      // Read from frptr to fwptr.
      bytes_to_read = VC_VTOH16(msgfifo->vout_fwptr) - VC_VTOH16(msgfifo->vout_frptr);
      if (bytes_to_read > nbytes)
         bytes_to_read = nbytes;

      VC_DEBUG( MsgFifo,  "vout_frptr: 0x%04x bytes_to_read: %d\n", VC_VTOH16(msgfifo->vout_frptr), bytes_to_read );

      if( host_addr != NULL )
      {
         status = vc_host_read_consecutive(host_addr, vc_interface_base + VC_VTOH16(msgfifo->vout_frptr), bytes_to_read, 0);
         vc_assert(status == 0);
      }
      msgfifo->vout_frptr = VC_HTOV16( ( (VC_VTOH16(msgfifo->vout_frptr)) + bytes_to_read ) );
      total_bytes_read += bytes_to_read;
   }

   return total_bytes_read;
}

/******************************************************************************
NAME
   vc_msgfifo_write_consecutive

SYNOPSIS
   int vc_msgfifo_write_consecutive(int inum, void *host_addr, int nbytes)

FUNCTION
   Write up to nbytes to our output fifo (VideoCore's input fifo). The number of
   bytes written is returned, which may be fewer than nbytes, or even 0 (the function
   does not block waiting for space to be available). Negative return values indicate
   failure. Note that this function does not refresh the fifo pointers, nor does
   it "flush" the write pointer back to VideoCore.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_write_consecutive (int inum, void *host_addr, int nbytes) {
   int status;
   int bytes_to_write;
   int total_bytes_written = 0;
   int prev_vin_frptr;
   VC_MSGFIFO_INTERFACE_T *msgfifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   msgfifo = &msgfifo_interface[inum];

   prev_vin_frptr = VC_VTOH16(msgfifo->vin_frptr);
   if (prev_vin_frptr == VC_VTOH16(msgfifo->vin_fmin))
      prev_vin_frptr = VC_VTOH16(msgfifo->vin_fmax);
   prev_vin_frptr -= VC_INTERFACE_BLOCK_SIZE;

   if (VC_VTOH16(msgfifo->vin_fwptr) > prev_vin_frptr) {
      // Write bytes from fwptr to fmax.
      bytes_to_write = VC_VTOH16(msgfifo->vin_fmax) - VC_VTOH16(msgfifo->vin_fwptr);
      if (bytes_to_write > nbytes)
         bytes_to_write = nbytes;
      status = vc_host_write_consecutive(vc_interface_base + VC_VTOH16(msgfifo->vin_fwptr), host_addr, bytes_to_write, 0);
      vc_assert(status == 0);
      total_bytes_written += bytes_to_write;
      nbytes -= bytes_to_write;
      host_addr = (void *)((unsigned char *)host_addr + bytes_to_write);
      msgfifo->vin_fwptr = VC_HTOV16(( (VC_VTOH16(msgfifo->vin_fwptr)) + bytes_to_write) );
      if (msgfifo->vin_fwptr == msgfifo->vin_fmax) {
         // We got to the end of the fifo. There may be more to read.
         msgfifo->vin_fwptr = msgfifo->vin_fmin;
      }
   }

   if (nbytes && (VC_VTOH16(msgfifo->vin_fwptr) < prev_vin_frptr))
   {
      // Write from fwptr to previous frptr.
      bytes_to_write = prev_vin_frptr - VC_VTOH16(msgfifo->vin_fwptr);
      if (bytes_to_write > nbytes)
         bytes_to_write = nbytes;

      VC_DEBUG( MsgFifo, "Offset: 0x%04x NumBytes: %d\n", VC_VTOH16( msgfifo->vin_fwptr ), bytes_to_write );

      status = vc_host_write_consecutive(vc_interface_base + VC_VTOH16(msgfifo->vin_fwptr), host_addr, bytes_to_write, 0);
      vc_assert(status == 0);
      msgfifo->vin_fwptr = VC_HTOV16(( (VC_VTOH16(msgfifo->vin_fwptr)) + bytes_to_write) );
      total_bytes_written += bytes_to_write;
   }

   return total_bytes_written;
}

/******************************************************************************
NAME
   vc_msgfifo_read_flush

SYNOPSIS
   int vc_msgfifo_read_flush(int inum)

FUNCTION
   Sends our copy of vout_frptr back to VideoCore. Generates an interrupt for this
   service if necessary. Non-zero indicates failure.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_read_flush (int inum) {
   int status;
   int offset;
   int old_bytes_in_fifo, new_bytes_in_fifo;
   int size4;
   VC_MSGFIFO_INTERFACE_T *msgfifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   msgfifo = &msgfifo_interface[inum];
   if (msgfifo_data[inum].vout_frptr == VC_VTOH16(msgfifo->vout_frptr))
      // Has not changed. There is nothing to do.
      return 0;

   offset = (uint32_t)&msgfifo->vout_frptr - (uint32_t)msgfifo;
   status = vc_host_write_consecutive(vc_interface_base + vc_sharedmem_header.iface[inum] + offset,
                                         &msgfifo->vout_frptr, VC_INTERFACE_BLOCK_SIZE, 0);

   VC_DEBUG( MsgFifo, "vout_frptr: 0x%08x\n", VC_VTOH16(msgfifo->vout_frptr) );

   vc_assert(status == 0);

   // Now update the note of how many bytes were in the fifo the last time
   // we flushed it. This is used to decide whether an interrupt needs to be sent.
   old_bytes_in_fifo = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vout_fmin), VC_VTOH16(msgfifo->vout_fmax),
      msgfifo_data[inum].vout_frptr, VC_VTOH16(msgfifo->vout_fwptr));
   new_bytes_in_fifo = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vout_fmin), VC_VTOH16(msgfifo->vout_fmax),
      VC_VTOH16(msgfifo->vout_frptr), VC_VTOH16(msgfifo->vout_fwptr));

   // Generate interrupt if fifo goes from more to less than 1/4 full.
   size4 = msgfifo_data[inum].out_size >> 2;
   if (old_bytes_in_fifo >= size4 && new_bytes_in_fifo < size4) {
      vc_interface_send_interrupt(inum /*1<<inum*/);
   }
   msgfifo_data[inum].vout_frptr = VC_VTOH16(msgfifo->vout_frptr);

   return 0;
}

/******************************************************************************
NAME
   vc_msgfifo_write_flush

SYNOPSIS
   int vc_msgfifo_write_flush(int inum)

FUNCTION
   Sends our copy of vin_fwptr back to VideoCore. Generates an interrupt for this
   service if necessary, either if the fifo becomes more than 3/4 full, or if the
   fifo started empty.

RETURNS
   int
******************************************************************************/

typedef struct {
   int vin_frptr;
   int vin_fwptr;
   int old_vin_fwptr;
   int sent_interrupt;
} VC_MSGFIFO_DEBUG_T;
static VC_MSGFIFO_DEBUG_T vc_msgfifo_debug[8];

int vc_msgfifo_write_flush (int inum) {
   int status;
   int offset;
   int old_bytes_in_fifo, new_bytes_in_fifo;
   int size34;
   VC_MSGFIFO_INTERFACE_T *msgfifo;
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   msgfifo = &msgfifo_interface[inum];
   if (msgfifo_data[inum].vin_fwptr == VC_VTOH16(msgfifo->vin_fwptr))
      // Has not changed. There is nothing to do.
      return 0;

   offset = (uint32_t)&msgfifo->vin_fwptr - (uint32_t)msgfifo;

   VC_DEBUG( MsgFifo,  "vin_fwptr: 0x%08x\n", VC_VTOH16(msgfifo->vin_fwptr ) );

   status = vc_host_write_consecutive(vc_interface_base + vc_sharedmem_header.iface[inum] + offset,
                                      &msgfifo->vin_fwptr, VC_INTERFACE_BLOCK_SIZE, 0);
   vc_assert(status == 0);

   // Now update the note of how many bytes were in the fifo the last time
   // we flushed it. This is used to decide whether an interrupt needs to be sent.
   old_bytes_in_fifo = msgfifo_data[inum].bytes_in_infifo;
   new_bytes_in_fifo = compute_bytes_in_fifo(VC_VTOH16(msgfifo->vin_fmin), VC_VTOH16(msgfifo->vin_fmax),
      VC_VTOH16(msgfifo->vin_frptr), VC_VTOH16(msgfifo->vin_fwptr));
   msgfifo_data[inum].bytes_in_infifo = new_bytes_in_fifo;

   // Generate interrupt if fifo was initially empty, goes from less to more
   // than 3/4 full, or the force flag is set.
   size34 = (3*msgfifo_data[inum].in_size) >> 2;
   if ((old_bytes_in_fifo == 0 && new_bytes_in_fifo > 0) ||
      (old_bytes_in_fifo < size34 && new_bytes_in_fifo >= size34)) {
      // Fifo has become more than 3/4 full.
      vc_interface_send_interrupt(inum /*1<<inum*/);
   }
   else {
      // The other way an interrupt happens is if the fifo started empty. We now
      // re-read vin_fptr and check it against the former value of vin_fwptr before
      // we flushed the output. We must do it in this slightly convoluted way to
      // avoid a possible race hazard.
      offset = (uint32_t)&msgfifo->vin_frptr - (uint32_t)msgfifo;
      vc_host_read_consecutive(&msgfifo->vin_frptr,
                               vc_interface_base + vc_sharedmem_header.iface[inum] + offset, VC_INTERFACE_BLOCK_SIZE, 0);

      VC_DEBUG( MsgFifo, "vin_frptr: 0x%08x\n", VC_VTOH16(msgfifo->vin_fwptr ) );

      vc_msgfifo_debug[inum].vin_frptr = VC_VTOH16(msgfifo->vin_frptr);
      vc_msgfifo_debug[inum].vin_fwptr = VC_VTOH16(msgfifo->vin_fwptr);
      vc_msgfifo_debug[inum].old_vin_fwptr = msgfifo_data[inum].vin_fwptr;
      vc_msgfifo_debug[inum].sent_interrupt = 0;

      if (VC_VTOH16(msgfifo->vin_frptr) == msgfifo_data[inum].vin_fwptr) {
         vc_msgfifo_debug[inum].sent_interrupt = 1;
        // The fifo started empty. Signal an interrupt.
         vc_interface_send_interrupt(inum /*1<<inum*/);
      }
   }
   msgfifo_data[inum].vin_fwptr = VC_VTOH16(msgfifo->vin_fwptr);

   return 0;
}

/******************************************************************************
NAME
   vc_msgfifo_read_blocking

SYNOPSIS
   void vc_msgfifo_read_blocking(int inum, void *host_addr, int nbytes, void *event)

FUNCTION
   Read nbytes from the fifo. This function must be passed an event that can
   be waited on if the function needs to block (waiting for bytes to become
   available). This function only flushes the read pointers back to VideoCore
   when it needs VideoCore to provide more data. Otherwise it does not. It will
   also refresh the fifo pointers only if it needs to.

RETURNS
   -
******************************************************************************/

void vc_msgfifo_read_blocking (int inum, void *host_addr, int nbytes, void *event) {

   VC_DEBUG( Trace, "inum: %d host_addr: 0x%08x nbytes: 0x%04x\n",
             inum, (uint32_t)host_addr, nbytes );

   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   while (1) {
      int bytes_read;
      bytes_read = vc_msgfifo_read_consecutive(inum, host_addr, nbytes);
      nbytes -= bytes_read;
      if (nbytes == 0)
         break;
      // Not enough data in the fifo, we shall go round again.
      if (host_addr != NULL)
      {
         host_addr = (void *)((unsigned char *)host_addr + bytes_read);
      }
      vc_msgfifo_read_flush(inum);
      // It's safe to clear the interrupt before refreshing the fifo.
      vc_event_clear(event);
      vc_msgfifo_read_refresh(inum);
      if (vc_msgfifo_input_bytes_available(inum) == 0)
         // VideoCore will signal an interrupt when it writes to empty fifo.
         vc_event_wait(event);
   }
}

/******************************************************************************
NAME
   vc_msgfifo_write_blocking

SYNOPSIS
   void vc_msgfifo_write_blocking(int inum, void *host_addr, int nbytes, void *event)

FUNCTION
   Write nbytes to the fifo. This function must be passed an event that can
   be waited on if the function needs to block (waiting for space to become
   available). This function only flushes the write pointers back to VideoCore
   when it needs VideoCore to empty the fifo. Otherwise it does not. It will
   also refresh the fifo pointers only if it needs to.

RETURNS
   -
******************************************************************************/

void vc_msgfifo_write_blocking (int inum, void *host_addr, int nbytes, void *event) {
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);
   while (1) {
      int bytes_written;
      bytes_written = vc_msgfifo_write_consecutive(inum, host_addr, nbytes);
      nbytes -= bytes_written;
      if (nbytes == 0)
         break;
      // Not enough space in the fifo, we shall go round again.
      host_addr = (void *)((unsigned char *)host_addr + bytes_written);
      // First of all, just re-read the fifo ptrs to see if there really
      // isn't the space. If possible we would like to continue the write without
      // doing a flush.
      vc_msgfifo_write_refresh(inum);
      if (vc_msgfifo_output_space_available(inum))
         continue;
      vc_msgfifo_write_flush(inum);
      // It's safe to clear the interrupt before refreshing the fifo.
      vc_event_clear(event);
      vc_msgfifo_write_refresh(inum);
      if (vc_msgfifo_output_space_available(inum) == 0)
         // VideoCore will signal an interrupt when fifo < 1/4 full.
         vc_event_wait(event);
   }
}

/******************************************************************************
NAME
   vc_msgfifo_send_command

SYNOPSIS
   int vc_msgfifo_send_command(int inum, int cmd_code, int ext_length, void *data)

FUNCTION
   Send a command, along with sync, xid etc. to the fifo. If insufficient space in
   the fifo for the whole message, sends nothing and returns non-zero.

RETURNS
   int
******************************************************************************/

int vc_msgfifo_send_command (int inum, int cmd_code, int ext_length, void *data) {
   VC_MSGFIFO_CMD_HEADER_T cmd;
   int status = 0;
   int bytes_written = 0;
   int data_bytes_to_write = (ext_length + 15) & ~15; // must send a multiple of 16 bytes
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   vc_msgfifo_write_refresh(inum);
   if (vc_msgfifo_output_space_available(inum) >= (int)sizeof(VC_MSGFIFO_CMD_HEADER_T) + data_bytes_to_write) {
      ++msgfifo_xid[inum];
      msgfifo_xid[inum] &= 0x7fffffffUL; /* H->VC xid has top bit clear */

      VC_DEBUG( MsgFifo, "sync      = 0x%08x\n", VC_CMD_SYNC );
      VC_DEBUG( MsgFifo, "xid       = 0x%08x\n", msgfifo_xid[inum] );
      VC_DEBUG( MsgFifo, "cmd_code  = 0x%08x\n", cmd_code );
      VC_DEBUG( MsgFifo, "ext_len   = 0x%04x\n", ext_length );
      VC_DEBUG( MsgFifo, "timestamp = 0x%04x\n", 0 );

      cmd.sync = VC_HTOV32(VC_CMD_SYNC);
      cmd.xid = VC_HTOV32(msgfifo_xid[inum]);
      cmd.cmd_code = VC_HTOV32(cmd_code);
      cmd.ext_length = VC_HTOV16(ext_length);
      cmd.timestamp = VC_HTOV16(0);
      // Write the command header.
      bytes_written = vc_msgfifo_write_consecutive(inum, &cmd, sizeof(VC_MSGFIFO_CMD_HEADER_T));
      vc_assert(bytes_written == sizeof(VC_MSGFIFO_CMD_HEADER_T));
      if (ext_length) {
          VC_DEBUG( MsgFifo, "Sending %d bytes of ext data\n", data_bytes_to_write );
         // We may write some garbage bytes at the end to pad the data to 16 bytes, but
         // this should be harmless.
         bytes_written = vc_msgfifo_write_consecutive(inum, data, data_bytes_to_write);
         vc_assert(bytes_written == data_bytes_to_write);
      }
      // Flush the output pointer, forcing an interrupt if the fifo had started empty.
      vc_msgfifo_write_flush(inum);
   }
   else
      status = VC_MSGFIFO_FULL;

   return status;
}

/******************************************************************************
NAME
   vc_msgfifo_send_command_blocking

SYNOPSIS
   void vc_msgfifo_send_command_blocking(int inum, int cmd_code, int ext_length, void *data, void *event)

FUNCTION
   Send a command, along with sync, xid etc. to the fifo. If insufficient space in
   the fifo for the whole message, sends nothing and returns non-zero.

RETURNS
   -
******************************************************************************/

void vc_msgfifo_send_command_blocking (int inum, int cmd_code, int ext_length, void *data, void *event) {
   VC_MSGFIFO_CMD_HEADER_T cmd;
   int data_bytes_to_write = (ext_length + 15) & ~15; // must send a multiple of 16 bytes
   vc_assert(inum >= 0 && inum < VC_NUM_INTERFACES);

   ++msgfifo_xid[inum];
   msgfifo_xid[inum] &= 0x7fffffffUL; /* H->VC xid has top bit clear */

   VC_DEBUG( MsgFifo, "sync      = 0x%08x\n", VC_CMD_SYNC );
   VC_DEBUG( MsgFifo, "xid       = 0x%08x\n", msgfifo_xid[inum] );
   VC_DEBUG( MsgFifo, "cmd_code  = 0x%08x\n", cmd_code );
   VC_DEBUG( MsgFifo, "ext_len   = 0x%04x\n", ext_length );
   VC_DEBUG( MsgFifo, "timestamp = 0x%04x\n", 0 );

   cmd.sync = VC_HTOV32(VC_CMD_SYNC);
   cmd.xid = VC_HTOV32(msgfifo_xid[inum]);
   cmd.cmd_code = VC_HTOV32(cmd_code);
   cmd.ext_length = VC_HTOV16(ext_length);
   cmd.timestamp = VC_HTOV16(0);
   // Write the command header.
   vc_msgfifo_write_blocking(inum, &cmd, sizeof(VC_MSGFIFO_CMD_HEADER_T), event);
   if (ext_length) {
      VC_DEBUG( MsgFifo, "Sending %d bytes of ext data\n", data_bytes_to_write );
      // We may write some garbage bytes at the end to pad the data to 16 bytes, but
      // this should be harmless.
      vc_msgfifo_write_blocking(inum, data, data_bytes_to_write, event);
   }
   // Flush the output pointer, forcing an interrupt if the fifo had started empty.
   vc_msgfifo_write_flush(inum);
}

/******************************************************************************
Static function definitions.
******************************************************************************/

int compute_bytes_in_fifo (int fmin, int fmax, int frptr, int fwptr) {
   // fifo is full if next(fwptr)==frptr, empty if fwptr==frptr.
   int nbytes;
   if (fwptr >= frptr)
      nbytes = fwptr - frptr;
   else
      nbytes = (fmax - frptr) + (fwptr - fmin);
   return nbytes;
}
