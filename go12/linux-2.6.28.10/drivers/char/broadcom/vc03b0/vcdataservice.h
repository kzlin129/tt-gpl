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




#ifndef VC_DATASERVICE_H
#define VC_DATASERVICE_H

/* Only a single thread should call any of the following at one time!       */


/* vc_dserv_init() should be called once at startup, after VideoCore boots.
   This function does NOT initialize the platform-specific part.            */

int vc_dserv_init(void);


/* Stop the service from being used. */

void vc_dserv_stop(void);


/* Call this when it is known if vc_dsos_start() has completed successfully
   or not [XXX start and stop completions should eventually become Async,
   but for the time being we hack it with a deferred response].             */

int vc_dserv_startresponse(int resp_code);


/* Call this when FIFO data is avaialable, or periodically if unknown.
   Returns 1 if a request was processed.                                    */

int vc_dserv_handle_request(void);


/* Call this when one of the registered sockets has becom readable or
   writable or both; this function generates an Async message to VideoCore.
   The socket should be unregistered by the platform-specific part.         */

void vc_dserv_async(int fd, int rwflags);


/* Nonzero if the data service has any open sockets that require attention. */

int vc_dserv_active(void);

/* get the dataservice event flag (used for blocking) */

void *vc_dserv_read_event(void);

/* Send PPP data to VC */

int vc_dserv_ppp_to_vc(void *ppp_data, int size);

/* Inform VC that data network is down */

void vc_dserv_network_down(void);

#endif
