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




#if defined( __KERNEL__ )
#   include <linux/string.h>
#else
#   include <string.h>
#endif

#include <linux/broadcom/vc.h>
//#include <stdio.h>
#include <stdarg.h>
//#include <ctype.h>

#define isspace(c)  (( (c) == ' ') || (( (c) >= '\t' ) && ( (c) <= '\r' )))

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcgencmd.h"
#include "vcutil.h"
#include "vcstate.h"

#ifndef min
   #define min(x,y) ((x) < (y) ? (x) : (y))
#endif

#ifndef max
   #define max(x,y) ((x) > (y) ? (x) : (y))
#endif

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

typedef enum {
   RRP_NO_RESP = 0,
   RRP_IN_RESP,
   RRP_WAIT_END_HEADER,
   RRP_FINISHED_RESP
} RRP_STATE;

/******************************************************************************
Static data.
******************************************************************************/

static int gencmd_inum = -1;
static void *gencmd_lock = NULL;
static void *gencmd_ievent = NULL;
static int gencmd_lock_held = 0;

static RRP_STATE resp_read_state = RRP_NO_RESP;
static int resp_read_length = 0;
static int resp_code = 0;

static void vcg_lock_grab (void) {
   vc_lock_obtain(gencmd_lock);
   gencmd_lock_held = 1;
}
static void vcg_lock_release (void) {
   gencmd_lock_held = 0;
   vc_lock_release(gencmd_lock);
}
static int vcg_lock_avail (void) {
   return !gencmd_lock_held;
}

/******************************************************************************
Static functions.
******************************************************************************/

/******************************************************************************
NAME
   vc_gencmd_init

SYNOPSIS
   int vc_gencmd_init()

FUNCTION
   Initialise the general command service for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

int vc_gencmd_init (void) {
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (gencmd_lock == NULL)
      gencmd_lock = vc_lock_create();

   vcg_lock_grab();

   if (gencmd_ievent == NULL)
      gencmd_ievent = vc_event_create();

   // We simply loop through every interface that there is and look for one
   // that claims to be a general command service.
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_GENCMD) {
            // Gotcha!
            gencmd_inum = i;
            vc_interface_register_event_int(gencmd_ievent, (1<<gencmd_inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   vcg_lock_release();
   return gencmd_inum;
}

/******************************************************************************
NAME
   vc_gencmd_stop

SYNOPSIS
   int vc_gencmd_stop()

FUNCTION
   This tells us that the generak command service has stopped, thereby preventing
   any of the functions from doing anything.

RETURNS
   int
******************************************************************************/

void vc_gencmd_stop () {
   // Assume a "power down" gencmd has been sent and the lock is held. There will
   // be no response so this should be called instead.
   if (vcg_lock_avail()) {
      // Also allow this to be called spontaneously just to suspend use of the service.
      vcg_lock_grab();
   }
   gencmd_inum = -1;
   vc_interface_register_event_int(gencmd_ievent, 0);
   vcg_lock_release();
}

/******************************************************************************
NAME
   vc_gencmd_inum

SYNOPSIS
   int vc_gencmd_inum()

FUNCTION
   Return the gencmd service number (-1 if not running)

RETURNS
   int
******************************************************************************/

int vc_gencmd_inum () {
   // Nothing to be gained by trying to grap the lock.
   return gencmd_inum;
}

/******************************************************************************
NAME
   vc_gencmd_send_string

SYNOPSIS
   int vc_gencmd_send_string(char *command)

FUNCTION
   Send a string to general command service.
   This will return immediately, though possibly with
   a VC_GENCMD_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   int
******************************************************************************/

static int vc_gencmd_send_string(char *command) {
   VC_DEBUG( MsgFifo, "command = '%s' len = %d\n", command, strlen( command ));
   vc_msgfifo_send_command_blocking(gencmd_inum, VC_GENCMD_EXECUTE, strlen(command)+1, command,
      gencmd_ievent);
   resp_code = 0;
   return 0;
}

/******************************************************************************
NAME
   vc_gencmd_send

SYNOPSIS
   int vc_gencmd_send( const char *format, ... )

FUNCTION
   Behaves exectly as vc_gencmd_send_string, except allows varargs.

RETURNS
   int
******************************************************************************/
#define GENCMD_MAX_LENGTH 512

// original function broken into two parts so that a higher level interface
// can be implemented and re-use the code
int vc_gencmd_send_list ( const char *format, va_list a )
{
   char command[GENCMD_MAX_LENGTH];
   int length;

   // Obtain the lock and keep it so no one else can butt in while we await the response.
   vcg_lock_grab();

   if (gencmd_inum < 0) {
      vcg_lock_release();
      return -1;
   }

   length = vsprintf ( command, format, a );

   if(length < 0)
   {
      vc_assert(0);
      vcg_lock_release();
      return -1;
   }
   VC_DEBUG( MsgFifo, "format = '%s' command = '%s' len = %d\n", format, command, strlen( command ));

   /* send message back to caller */
   return vc_gencmd_send_string (command);
}

int vc_gencmd_send ( const char *format, ... )
{
   va_list a;
   int     rv;

   va_start ( a, format );
   rv = vc_gencmd_send_list( format, a );
   va_end ( a );
   return rv;
}

/******************************************************************************
NAME
   vc_gencmd_read_response

SYNOPSIS
   int vc_gencmd_read_response

FUNCTION
   Read a response containing the transfer buffer. This function returns immediately.
   If there was not enough in the input fifo a non-zero value is returned.
   The response will be null terminated.

RETURNS
   int
******************************************************************************/
int vc_gencmd_read_response (char *response, int maxlen) {
   int lenread, accum = 0;

   do {
      accum += (lenread = vc_gencmd_read_response_partial(response+accum, maxlen-accum));
   } while(lenread && accum < maxlen);

   vc_gencmd_close_response_partial();

   // If we read anything, return the VideoCore code. Error codes < 0 mean we failed to
   // read anything...
   return (accum?resp_code:-1);
}

/******************************************************************************
NAME
   vc_gencmd

SYNOPSIS
   int vc_gencmd(char *response, int maxlen, const char *format, ...)

FUNCTION
   Send a gencmd and receive the response as per vc_gencmd read_response.

RETURNS
   int
******************************************************************************/
int vc_gencmd(char *response, int maxlen, const char *format, ...) {
   va_list args;
   int ret;

   va_start(args, format);

   ret = vc_gencmd_send_list(format, args);

   VC_DEBUG( MsgFifo, "format = '%s'\n", format );
   //vc_gencmd_send_list(format, args);
   va_end (args);

   if(ret < 0)
      return ret;

   return vc_gencmd_read_response(response, maxlen);
}

int vc_gencmd_read_partial_state(void) {
   return resp_read_state;
}

/******************************************************************************
NAME
   vc_gencmd_read_response_partial

SYNOPSIS
   int vc_gencmd_read_response_partial(char *response, int nbytes)

FUNCTION
   Read nbytes of string from the gencmd message fifo, if a string has previously
   been found on the fifo. If not, read 16 bytes and check if this is a valid
   start of string header; if not return an error, otherwise read string. When
   a string has been finshed the caller *must* call vc_gencmd_close_response
   before trying to read another string.

RETURNS
	int - number of bytes read, 0 if end of string reached, -ve if no valid input found
******************************************************************************/

int vc_gencmd_read_response_partial(char *response, int nbytes)
{
   VC_MSGFIFO_RESP_HEADER_T header;
   int   fifo_rlen;
   char  *nt;

   vc_assert(gencmd_inum >= 0);

   // we've finished reading the response, the next thing that we'll hit will be the header
   if(resp_read_state == RRP_FINISHED_RESP)
      return 0;

   // if possible, try to read out the header
   if(resp_read_state == RRP_NO_RESP || resp_read_state == RRP_WAIT_END_HEADER) {
      vc_msgfifo_read_blocking(gencmd_inum, &header, sizeof(VC_MSGFIFO_RESP_HEADER_T), gencmd_ievent);
      // Flush the input pointer. Use bytes available to check enough data is
      vc_msgfifo_read_flush(gencmd_inum);
      header.sync = VC_VTOH32(header.sync);
	  header.xid = VC_VTOH32(header.xid);
	  header.resp_code = VC_VTOH32(header.resp_code);
     resp_code = header.resp_code;
	  header.ext_length = VC_VTOH16(header.ext_length);
	  header.timestamp = VC_VTOH16(header.timestamp);
      if(header.sync != VC_CMD_SYNC) {
         VC_DEBUG( MsgFifo, "Found header.sync: 0x%08x, expecting: 0x%08x\n",
                   header.sync, VC_CMD_SYNC );
         vc_assert(header.sync == VC_CMD_SYNC);
         return -1;
      }
      if(resp_read_state == RRP_WAIT_END_HEADER) {
         resp_read_state = RRP_FINISHED_RESP;
         return 0;
      }
      resp_read_length = (header.ext_length == 0xffff) ? -1 : (header.ext_length + 15) &~ 15;
      resp_read_state = RRP_IN_RESP;
   }

   if(resp_read_length > 0) {
      // if string is fixed length, read the min of read length and nbytes
      fifo_rlen = min(resp_read_length, (nbytes & ~15));
   } else {
      // if variable length, read min of nbytes, data_in_fifo, subject to min allowable of 16 bytes
      fifo_rlen = vc_msgfifo_input_bytes_available(gencmd_inum) & ~15;
      fifo_rlen = min(fifo_rlen, nbytes);
      fifo_rlen = max(fifo_rlen, 16);
   }

   // don't do anything silly here
   vc_assert(nbytes >= fifo_rlen);

   // read the alloted number of bytes
   vc_msgfifo_read_blocking(gencmd_inum, response, fifo_rlen, gencmd_ievent);
   // Flush the input pointer. Use bytes available to check enough data is
   vc_msgfifo_read_flush(gencmd_inum);

   // for fixed length strings, indicate how-much we have left to read
   if(resp_read_length>=0)
      resp_read_length = max(resp_read_length-fifo_rlen, 0);

   // take a second bite at the apple :- if more data is waiting, and we have space read it
   if( vc_msgfifo_input_bytes_available(gencmd_inum) > 15 && nbytes-fifo_rlen > 15 ) {
      int r2len = vc_msgfifo_input_bytes_available(gencmd_inum);
      r2len = min(nbytes-fifo_rlen, r2len) & ~15;
      if(resp_read_length>0)
         r2len = min(r2len, resp_read_length);
      vc_msgfifo_read_blocking(gencmd_inum, response+fifo_rlen, r2len, gencmd_ievent);
      // Flush the input pointer. Use bytes available to check enough data is
      vc_msgfifo_read_flush(gencmd_inum);
      if(resp_read_length>0)
         resp_read_length = max(resp_read_length-r2len, 0);
      fifo_rlen += r2len;
   }

   // did var length response string have a null terminator ?
   nt = (char*)memchr(response, 0, fifo_rlen);
   if( resp_read_length < 0 && nt ) {
      int   rlen = (int)(nt-response)+1; // strlen(response)+1;
      // see if we have actually read the header into the buffer, if so check it and set the state appropriately
      if(fifo_rlen == ((rlen+15)&~15)+16) {
         VC_MSGFIFO_RESP_HEADER_T *resp = (VC_MSGFIFO_RESP_HEADER_T*)(response+((rlen+15)&~15));
		 resp->sync = VC_VTOH32(resp->sync);
         vc_assert(resp->sync == VC_CMD_SYNC);
         resp_code = VC_VTOH32(resp->resp_code);
         resp_read_state = RRP_FINISHED_RESP;
      } else {
         // we reached the end of the actual string, but no luck with the header
         resp_read_state = RRP_WAIT_END_HEADER;
      }
      return rlen;
   }

   // for fixed length strings, if we have read the whole string in, change our state to
   // RRP_FINISHED_RESP
   if( resp_read_length == 0 ) {
      resp_read_state = RRP_FINISHED_RESP;
   }

   // might return a few bytes representing junk when we get to the end of the file
   return fifo_rlen;
}


/******************************************************************************
NAME
   vc_gencmd_close_response_partial

SYNOPSIS
   int vc_gencmd_close_response_partial(void)

FUNCTION
   Finish reading a string off of the gencmd fifo, if there are any remanents
   of the string still on the fifo consume them buffer, until the header
   has been found

RETURNS
   0 to indicate success
******************************************************************************/

int vc_gencmd_close_response_partial(void)
{
   char temp[128];

   // consume the string on the fifo until we've read the whole response
   while( resp_read_state != RRP_FINISHED_RESP && vc_gencmd_read_response_partial(temp,128) ) ;

   // reset the state to waiting for a new response
   resp_read_state = RRP_NO_RESP;
   resp_read_length = 0;

   // Flush the input pointer. Use bytes available to check enough data is
   vc_msgfifo_read_flush(gencmd_inum);

   // We've got all the response we expected, so give up the gencmd lock.
   vcg_lock_release();

   // Return the VideoCore return code.
   return resp_code;
}


/******************************************************************************
NAME
   vc_gencmd_string_property

SYNOPSIS
   int vc_gencmd_string_property(char *text, char *property, char **value, int *length)

FUNCTION
   Given a text string, containing items of the form property=value,
   look for the named property and return the value. The start of the value
   is returned, along with its length. The value may contain spaces, in which
   case it is enclosed in double quotes. The double quotes are not included in
   the return parameters. Return non-zero if the property is found.

RETURNS
   int
******************************************************************************/

int vc_gencmd_string_property(char *text, char *property, char **value, int *length) {
#define READING_PROPERTY 0
#define READING_VALUE 1
#define READING_VALUE_QUOTED 2
   int state = READING_PROPERTY;
   int delimiter = 1, match = 0, len = strlen(property);
   char *prop_start=text, *value_start=text;
   for (; *text; text++) {
      int ch = *text;
      switch (state) {
      case READING_PROPERTY:
         if (delimiter) prop_start = text;
         if (isspace(ch)) delimiter = 1;
         else if (ch == '=') {
            delimiter = 1;
            match = (text-prop_start==len && strncmp(prop_start, property, text-prop_start)==0);
            state = READING_VALUE;
         }
         else delimiter = 0;
         break;
      case READING_VALUE:
         if (delimiter) value_start = text;
         if (isspace(ch)) {
            if (match) goto success;
            delimiter = 1;
            state = READING_PROPERTY;
         }
         else if (delimiter && ch == '"') {
            delimiter = 1;
            state = READING_VALUE_QUOTED;
         }
         else delimiter = 0;
         break;
      case READING_VALUE_QUOTED:
         if (delimiter) value_start = text;
         if (ch == '"') {
            if (match) goto success;
            delimiter = 1;
            state = READING_PROPERTY;
         }
         else delimiter = 0;
         break;
      }
   }
   if (match) goto success;
   return 0;
 success:
   *value = value_start;
   *length = text - value_start;
   return 1;
}

/******************************************************************************
NAME
   vc_gencmd_number_property

SYNOPSIS
   int vc_gencmd_number_property(char *text, char *property, int *number)

FUNCTION
   Given a text string, containing items of the form property=value,
   look for the named property and return the numeric value. If such a numeric
   value is successfully found, return 1; otherwise return 0.

RETURNS
   int
******************************************************************************/

int vc_gencmd_number_property(char *text, char *property, int *number) {
   char *value, temp;
   int length, retval;
   if (vc_gencmd_string_property(text, property, &value, &length) == 0)
      return 0;
   temp = value[length];
   value[length] = 0;
   retval = sscanf(value, "0x%x", number);
   if (retval != 1)
      retval = sscanf(value, "%d", number);
   value[length] = temp;
   return retval;

}
