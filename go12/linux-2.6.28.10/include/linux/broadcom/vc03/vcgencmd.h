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




#ifndef GENCMD_H
#define GENCMD_H

/* Initialise general command service. Returns it's interface number. This initialises
   the host side of the interface, it does not send anything to VideoCore. */

int vc_gencmd_init(void);

/* Stop the service from being used. */

void vc_gencmd_stop(void);

/* Return the service number (-1 if not running). */
int vc_gencmd_inum(void);

/******************************************************************************
Send commands to VideoCore.
These all return 0 for success. They return VC_MSGFIFO_FIFO_FULL if there is
insufficient space for the whole message in the fifo, and none of the message is
sent.
******************************************************************************/

int vc_gencmd(void * instp, char *response, int maxlen, const char *format, ...);

/*  send command to general command serivce */
int vc_gencmd_send( void * instp, const char *format, ... );

/*  get resonse from general command serivce */
int vc_gencmd_read_response( void * instp, char *response, int maxlen);

/* read part of a response from the general command service */
int vc_gencmd_read_response_partial(char *response, int nbytes);

/* if reading with vc_gencmd_read_response_partial end response reads with this */
int vc_gencmd_close_response_partial(void);

/* get state of reading of response */
int vc_gencmd_read_partial_state(void);

/******************************************************************************
Utilities to help interpret the responses.
******************************************************************************/

/* Read the value of a property=value type pair from a string (typically VideoCore's
   response to a general command). Return non-zero if found. */
int vc_gencmd_string_property(char *text, char *property, char **value, int *length);

/* Read the numeric value of a property=number field from a response string. Return
   non-zero if found. */
int vc_gencmd_number_property(char *text, char *property, int *number);

#endif
