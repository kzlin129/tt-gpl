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




#ifndef VCMSGFIFO_H
#define VCMSGFIFO_H

/* Some convenient return codes: */

#define VC_MSGFIFO_FULL 1
#define VC_MSGFIFO_EMPTY 2

/* Header structure for commands. */
typedef struct {
   uint32_t sync;
   uint32_t xid;
   uint32_t cmd_code;
   uint16_t ext_length;
   uint16_t timestamp;
} VC_MSGFIFO_CMD_HEADER_T;

/* Header structure for responses. */
typedef struct {
   uint32_t sync;
   uint32_t xid;
   uint32_t resp_code;
   uint16_t ext_length;
   uint16_t timestamp;
} VC_MSGFIFO_RESP_HEADER_T;

/* Initialise a msgfifo interface for use. Non-zero return indicates failure. */

int vc_msgfifo_init(int inum);

/* Return the number of bytes known to be available to read in our input fifo. */

int vc_msgfifo_input_bytes_available(int inum);

/* Return the number of bytes space known to be available in our input fifo. */

int vc_msgfifo_input_space_available(int inum);

/* Return the number of bytes present and unread by the VideoCore in our output fifo. */

int vc_msgfifo_output_bytes_available(int inum);

/* Return the number of bytes space available in our output fifo. */

int vc_msgfifo_output_space_available(int inum);

/* Reload our input fifo pointer that VideoCore sets (vout_fwptr). */

int vc_msgfifo_read_refresh(int inum);

/* Reload our output fifo pointer that VideoCore sets (vin_frptr). */

int vc_msgfifo_write_refresh(int inum);

/* Read bytes from our input fifo. Returns the number read (which may be zero). */

int vc_msgfifo_read_consecutive(int inum, void *host_addr, int nbytes);

/* Write bytes to our output fifo. Returns the number written (which may be zero). */

int vc_msgfifo_write_consecutive(int inum, void *host_addr, int nbytes);

/* "Flush" the input by updating our input fifo's read pointer. */

int vc_msgfifo_read_flush(int inum);

/* "Flush" the output by updating our output fifo's write pointer. */

int vc_msgfifo_write_flush(int inum);

/* Read nbytes from the fifo. This blocks as necessary but does not normally
   flush our input read pointer back to VideoCore. */

void vc_msgfifo_read_blocking(int inum, void *host_addr, int nbytes, void *event);

/* Write nbytes to the fifo. This blocks as necessary but does not normally
   flush our output write pointer back to VideoCore. */

void vc_msgfifo_write_blocking(int inum, void *host_addr, int nbytes, void *event);

/* Send a command and ext_length bytes of parameters to the fifo. If insufficient
   space in the fifo, writes nothing at all, and returns VC_MSGFIFO_FULL. USE NOW DEPRECATED. */

int vc_msgfifo_send_command(int inum, int cmd_code, int ext_length, void *data);

/* Send a command and ext_length bytes of parameters to the fifo, blocking if necessary. */

void vc_msgfifo_send_command_blocking(int inum, int cmd_code, int ext_length, void *data, void *event);

#endif
