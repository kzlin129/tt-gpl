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





/*** Operating-system dependent parts of the Data Service Server.
     These must be implemented for each host platform that we will run on. */

/* vchost_config.h actually defines the host platform's socket type. */

#include "vchost_config.h"


/* Called from the main program -- Win32 specific!
   If multi-threaded, call with a lock held and set timeout to zero.       */

int vc_dsos_idle(int timeout);

int vc_dsos_dump(int off_bin_ascii, const char * filename);



/* Everything below this point is platform-independent and does not vary.  */

/* One-time initialization. Call this only at startup. */

void vc_dsos_init(void);


/* Query the capabilities of the implementation (after vc_dsos_init()).    */

int vc_dsos_cap_listen();
int vc_dsos_cap_local_addr();


/* Per-session initialization and teardown. All the calls listed below
   vc_dsos_start() require vc_dsos_start() to have been called before.
   vc_dsos_start() returns 0 on immediate success, VC_SOCK_WOULDBLOCK
   on pending success, in which case vc_dserv_startresponse() *must*
   be called back, or an error code on failure. vc_dsos_stop() always
   unregisters and closes all open sockets, and should generate an
   immediate response.                                                    */

int vc_dsos_start(void);

void vc_dsos_stop(void);


/* Create a UDP socket. If localPort is nonzero, try to bind to that port.
   Port number is specified in host (numeric) byte order.
   Returns a valid socket or returns VCDSOS_INVALID_SOCKET and sets
   errno to one of the VC_SOCK_* error codes.                              */

VCDSOS_SOCK_T vc_dsos_udpsocket(uint16_t localPort, int * errno);


/* Create a TCP socket and attempt to connect it to the following remote
   address. Connect should be a non-blocking operation where possible.
   Returns a valid socket or returns VCDSOS_INVALID_SOCKET and sets
   errno to one of the VC_SOCK_* error codes.                              */

VCDSOS_SOCK_T vc_dsos_connect(const uint8_t remoteAddr[4],
                              uint16_t remotePort, int * errno);


/* Create a TCP socket, bind it to an address, and listen for connections.
   Only supported on platforms that have the "listen" capability.          */

VCDSOS_SOCK_T vc_dsos_listen(uint16_t localPort, int qlen, int * errno);


/* Accept a TCP connection. Non-blocking implementation where possible.    */

VCDSOS_SOCK_T vc_dsos_accept(VCDSOS_SOCK_T listener,
                             uint8_t * remoteAddr, uint16_t * remotePort,
                             int * errno);


/* Determine the local IP address. Only supported on platforms that have
   the "local addr" capability.                                            */

int vc_dsos_local_addr(uint8_t * localAddr);


/* Close a socket. Non-blocking implementation where possible.
   Returns VC_SOCK_OK or VC_SOCK_NOTSOCK.                                  */

int vc_dsos_close(VCDSOS_SOCK_T);



/* Returns nonzero if there are any open sockets */

int vc_dsos_active(void);


/* UDP recvfrom and sendto. Non-blocking implementation where possible.
   Returns a positive byte count or a negative VC_SOCK_* error code.       */

int vc_dsos_recvfrom(VCDSOS_SOCK_T fd, void * ptr, int maxbytes,
                      uint8_t * remoteAddr, uint16_t * remotePort);

int vc_dsos_sendto(VCDSOS_SOCK_T fd, const void * ptr, int bytes,
                   const uint8_t remoteAddr[4], uint16_t remotePort);


/* TCP read/recv and write/send. Non-blocking implementation where possible.
   Out-of-band data should be inlined with regular data.
   Returns a positive byte count or a negative VC_SOCK_* error code.       */

int vc_dsos_read(VCDSOS_SOCK_T fd, void * ptr, int maxbytes);

int vc_dsos_write(VCDSOS_SOCK_T fd, const void * ptr, int maxbytes);


/* Determine if read/recv/recvfrom or write/send/sendto would return
   anything other than VC_SOCK_WOULDBLOCK. If there is no way of telling,
   return "safe" defaults: 0 if there is an async polling mecahnism, or
   1 (as a last resort) if there is no async polling mechanism but calls
   are blocking (this is acceptable for writes, if not for reads).         */

int vc_dsos_readable(VCDSOS_SOCK_T fd);

int vc_dsos_writable(VCDSOS_SOCK_T fd);


/* We maintain two sets of sockets, "read-registered" and "write-
   registered". Membership must be assigned by explicit API calls;
   but when a socket becomes known to be readable or writable and
   vc_dserv_async() is called back, it is automatically unregistered.      */

void vc_dsos_register_r(VCDSOS_SOCK_T fd);

void vc_dsos_register_w(VCDSOS_SOCK_T fd);

void vc_dsos_unregister_r(VCDSOS_SOCK_T fd);

void vc_dsos_unregister_w(VCDSOS_SOCK_T fd);


/* Return the local IP address, on platforms supporting this feature.
   XXX The Win32 platform claims to support this but the current
   implementation sometimes fails with VC_SOCK_UNSUPPORTED anyway.         */

int vc_dsos_local_addr(uint8_t * addr);


int vc_dsos_ppp_from_vc(void *ppp_data, int size);

int vc_dsos_ppp_returned(void *ppp_data, int size);

int vc_dsos_ppp_to_vc(void *ppp_data, int size);

void vc_dsos_request_ppp(void);

void vc_dsos_refuse_ppp(void);

int vcdsos_get_ppp_nego_info(void *data);
