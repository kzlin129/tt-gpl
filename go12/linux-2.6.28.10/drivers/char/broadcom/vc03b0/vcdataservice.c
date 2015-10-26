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




#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#ifdef WIN32
int tprintf(const char *format, ...);
#endif

#include "vchost_config.h"
#include "vcdataservice.h"
#include "vc_dataservice_defs.h"
#include "vcdsos.h"

#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

//#include "vcdispman.h"

//#include "ds4vod.h"

static int ds_inum = -1;
static void *ds_ievent = NULL;
static void *ds_oevent = NULL;
#ifdef VC_DATASERVICE_PPP
static int ppp_request_mode=0;
#endif /* VC_DATASERVICE_PPP */

void *vc_dserv_read_event(void) {
   return ds_ievent;
}
#ifdef WIN32
#define printf tprintf
#define DEBUG_LEVEL 1
#endif

#ifdef DEBUG_LEVEL
#if DEBUG_LEVEL > 1
#define DEBUG_MINOR(x) x  /* Debug level 2, enable major and minor debug */
#define DEBUG_MAJOR(x) x
#else
#define DEBUG_MAJOR(x) x  /* Debug level 1, enable only major debug stmts */
#define DEBUG_MINOR(x)
#endif
#else
#define DEBUG_MAJOR(x)    /* Debug level 0, do not compile debug statements */
#define DEBUG_MINOR(x)
#endif


/*****************************************************************************
NAME
   vc_dserv_init

SYNOPSIS
   int vc_dserv_init()

FUNCTION
   Initialise the data service (platform-independent part). This tries
   to find a data service FIFO on VideoCore, which must already be up.

RETURNS
   The interface number, or negative on failure.
*****************************************************************************/

int vc_dserv_init()
{
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (ds_ievent == NULL) {
      ds_ievent = vc_event_create();
      ds_oevent = vc_event_create();
   }

   /* We simply loop through every interface that there is and look for one
    that claims to be a data service. */
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
       stype = VC_VTOH16(generic_interface.stype);
         if (generic_interface.stype == VC_STYPE_DATASERVICE) {
            /* Gotcha! */
            ds_inum = i;
            vc_interface_register_event_int(ds_ievent, 1<<ds_inum);
            vc_interface_register_event_int(ds_oevent, 1<<ds_inum);
            status = vc_msgfifo_init(i);
            //vc_dsos_init(); xxx dc4 may want this in sometimes
            assert(status == 0);
            break;
         }
      }
   }

   return ds_inum;
}

#define DS_MAX_DATA 1760

struct data_service_msg_body {
   uint32_t params[4];
   uint8_t  data[DS_MAX_DATA];
};

static struct data_service_msg_body body;

#ifdef DEBUG_LEVEL
#if DEBUG_LEVEL > 1
/* Debug related function, prints out the contents of a message */
static void showmsg(VC_MSGFIFO_CMD_HEADER_T const * head, struct data_service_msg_body const * body)
{
   unsigned int ael = (head->ext_length + 15) & ~15;
   printf("Sync=%08x XID=%08x Code=%08x Extlen=%08x (%d bytes follow)\n",
          head->sync, head->xid, head->cmd_code, head->ext_length, ael);
   if (ael) {
      unsigned int i;
      printf("Content:");
      if (head->cmd_code != VC_DSPRO_PPP_FROM_VC && head->cmd_code != VC_DSPRO_PPP_RETURNED) {
         for(i = 0; i < 4 && i*4 < head->ext_length; ++i) printf(" %08x", body->params[i]);
            if (head->ext_length > 16) printf(" plus %d bytes", head->ext_length);
      } else {
         unsigned char *p = (char *)body->params;
         for(i = 0; i < head->ext_length; ++i) printf(" %02x", p[i]);
      }
      printf("\n");
   }
}
#endif
#endif


/******************************************************************************
NAME
   vc_dserv_stop

SYNOPSIS
   void vc_dserv_stop()

FUNCTION
   This stops the dataservice, thereby preventing any of the functions from
   doing anything.

RETURNS
   void
******************************************************************************/

void vc_dserv_stop () {
   ds_inum = -1;
   vc_interface_register_event_int(ds_ievent, 0);
   vc_interface_register_event_int(ds_oevent, 0);
}

/******************************************************************************
NAME
   vc_dserv_inum

SYNOPSIS
   int vc_dserv_inum()

FUNCTION
   Return the dataservice service number (-1 if not running).

RETURNS
   int
******************************************************************************/

int vc_dserv_inum () {
   return ds_inum;
}


/*****************************************************************************
NAME
   vc_dserv_handle_request

SYNOPSIS
   int vc_dserv_handle_request()

FUNCTION
   When called because of an interrupt, or on polling; checks the FIFO
   to see if there is an incoming request; services one or more requests.

RETURNS
   1 if a request was handled, 0 if no request was present.
*****************************************************************************/
int vc_dserv_handle_request()
{
   int rr = 0;

   if (ds_inum < 0)
      return 0;

   vc_msgfifo_read_refresh(ds_inum);
   vc_msgfifo_write_refresh(ds_inum);
   while (vc_msgfifo_input_bytes_available(ds_inum) > 0) {
      VC_MSGFIFO_CMD_HEADER_T req;
      uint32_t ael, rlen;
      int fd, ret;
      unsigned char ipaddr[4];
      uint16_t ush;
      int response_required = 1;
      rr = 1;

      // Read the request header
      vc_msgfifo_read_blocking(ds_inum, &req, sizeof(VC_MSGFIFO_CMD_HEADER_T), ds_ievent);
      assert(req.sync == VC_CMD_SYNC);

      req.cmd_code = VC_VTOH32(req.cmd_code);
      req.ext_length = VC_VTOH16(req.ext_length);
      req.timestamp = VC_VTOH16(0);

      ael = (req.ext_length + 15) & ~15;

      if (ael > 0) {
         assert(ael < sizeof(struct data_service_msg_body));

         vc_msgfifo_read_blocking(ds_inum, &body, ael, ds_ievent);

         body.params[0] = VC_VTOH32(body.params[0]);
         body.params[1] = VC_VTOH32(body.params[1]);
         body.params[2] = VC_VTOH32(body.params[2]);
         body.params[3] = VC_VTOH32(body.params[3]);
      }

      vc_msgfifo_read_flush(ds_inum);
      DEBUG_MINOR(printf("Data service request:\n"));
      DEBUG_MINOR(showmsg(&req, &body));

      rlen = 0;
      ret = VC_SOCK_OK;
      fd = body.params[0];
      switch (req.cmd_code) {
#ifdef VC_DATASERVICE_PPP
      case VC_OPEN_PPP: /* send PPP data to VC */
         DEBUG_MAJOR(printf("PPP open requested\n"));
         vc_dsos_start();
         response_required = 0;
         break;
      case VC_REQUEST_PPP: /* send PPP data to VC */
         DEBUG_MAJOR(printf("PPP data requested by VC\n"));
         vc_dsos_request_ppp();
         response_required = 0;
         break;
      case VC_REFUSE_PPP: /* stop sending PPP data to VC */
         DEBUG_MAJOR(printf("PPP data refused by VC\n"));
         // we will need to be careful here, there may still be returned packets from VC due
         // so we need to ensure these are in correct sequence compared to later packets not
         // routed to VC and back
         vc_dsos_refuse_ppp();
         response_required = 0;
         break;
      case VC_DSPRO_PPP_FROM_VC: /* PPP data */
      {
         DEBUG_MINOR(printf("Got PPP block of size %d from VC\n", req.ext_length));
         vc_dsos_ppp_from_vc(&body, req.ext_length);
         response_required = 0;
         break;
      }
      case VC_DSPRO_PPP_RETURNED: /* PPP data returned */
      {
         DEBUG_MINOR(printf("Returned PPP block of size %d from VC\n", req.ext_length));
         vc_dsos_ppp_returned(&body, req.ext_length);
         response_required = 0;
         break;
      }
#endif /* VC_DATASERVICE_PPP */

      case VC_DSPRO_LOCAL_ADDR:
         DEBUG_MAJOR(printf("Data service get local address\n"));
         body.params[0] = 0;
         if (vc_dsos_cap_local_addr() &&
             vc_dsos_local_addr(ipaddr) == VC_SOCK_OK) {
            body.params[0] = (ipaddr[0]<<24) | (ipaddr[1]<<16) | (ipaddr[2]<<8) | ipaddr[3]; /* sic */
            DEBUG_MAJOR(printf("My IP address seems to be %u.%u.%u.%u\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]));
         }
         rlen = 4;
         break;

      case VC_DSPRO_START: /* XXX check for repeats? */
         DEBUG_MAJOR(printf("Data service START\n"));
         ret = vc_dsos_start();
         body.params[0] = VC_DSPRO_VERSION;

         if (vc_dsos_cap_listen()) body.params[0] |= VC_DSPRO_HOPT_LISTEN;
         if (vc_dsos_cap_local_addr() &&
             vc_dsos_local_addr(ipaddr) == VC_SOCK_OK) {
            body.params[0] |= VC_DSPRO_HOPT_LOCALADDR;
            body.params[2] = (ipaddr[0]<<24) | (ipaddr[1]<<16) | (ipaddr[2]<<8) | ipaddr[3]; /* sic */
            DEBUG_MAJOR(printf("My IP address seems to be %u.%u.%u.%u\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]));
         }
         rlen = 16+vcdsos_get_ppp_nego_info(body.data);
         break;

      case VC_DSPRO_STOP: /* XXX check for sockets still open? */
         DEBUG_MAJOR(printf("Data service STOP\n"));
         vc_dsos_stop();
         break;

      case VC_DSPRO_CLOSE:
         DEBUG_MAJOR(printf("Data service CLOSE\n"));
         vc_dsos_unregister_w(fd);
         vc_dsos_unregister_r(fd);
         ret = vc_dsos_close(body.params[0]);
         DEBUG_MAJOR(printf("Closed socket %d\n", fd));
         break;

      case VC_DSPRO_UDPSOCK:
        DEBUG_MAJOR(printf("Data service UDPSOCK\n"));
         fd = vc_dsos_udpsocket((uint16_t)body.params[1], &ret);
         if (!VCDSOS_IS_INVALID(fd)) {
            DEBUG_MAJOR(printf("Created UDP socket %d at port %d\n", fd, body.params[1]));
            ret = 0;
            /* Assume we start off writable but not readable */
            vc_dsos_register_r(fd);
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_CREATE | VC_DSPRO_WFLAG;
            rlen = 8;
         }
         break;

      case VC_DSPRO_CONNECT:
         DEBUG_MAJOR(printf("Data service CONNECT\n"));
         ipaddr[0] = (uint8_t)(body.params[0] >> 24);
         ipaddr[1] = (uint8_t)(body.params[0] >> 16);
         ipaddr[2] = (uint8_t)(body.params[0] >> 8);
         ipaddr[3] = (uint8_t)(body.params[0]);
         fd = vc_dsos_connect(ipaddr, (uint16_t)body.params[1], &ret);
         if (!VCDSOS_IS_INVALID(fd)) {
            DEBUG_MAJOR(printf("Connected TCP socket %d to %d.%d.%d.%d port %d\n",
                               fd, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], body.params[1]));
            ret = 0;
            /* Assume neither writable nor readable, until we next poll it. */
            vc_dsos_register_r(fd);
            vc_dsos_register_w(fd);
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_CREATE;
            rlen = 8;
         }
       else DEBUG_MAJOR(printf("Failed to connect\n"));
         break;

      case VC_DSPRO_RECVFROM:
         DEBUG_MINOR(printf("Data service RECVFROM\n"));
         ret = body.params[1];
         if (ret > DS_MAX_DATA) ret = DS_MAX_DATA;
         ret = vc_dsos_recvfrom(fd, body.data, ret, ipaddr, &ush);
         if (ret >= 0) {
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_RRESP;
            body.params[2] = (ipaddr[0] << 24) | (ipaddr[1] << 16) |
               (ipaddr[2] << 8) | ipaddr[3];
            body.params[3] = ush;
            if (vc_dsos_readable(fd)) {
               body.params[1] |= VC_DSPRO_RFLAG;
               vc_dsos_unregister_r(fd);
            }
            else {
               vc_dsos_register_r(fd);
            }
            rlen = ret + 16;
            ret = VC_SOCK_OK;
         }
         else  { /* Recvfrom failed, assume it's unreadable */
            vc_dsos_register_r(fd);
         }
         break;

      case VC_DSPRO_READ:
         DEBUG_MINOR(printf("Data service READ\n"));
         ret = body.params[1];
         if (ret > DS_MAX_DATA) ret = DS_MAX_DATA;
         ret = vc_dsos_read(fd, body.data, ret);
         if (ret >= 0) {
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_RRESP;
            if (vc_dsos_readable(fd)) {
               body.params[1] |= VC_DSPRO_RFLAG;
               vc_dsos_unregister_r(fd);
            }
            else {
               vc_dsos_register_r(fd);
            }
            rlen = ret + 16;
            ret = VC_SOCK_OK;
         }
         else { /* Read failed, assume it's unreadable */
            vc_dsos_register_r(fd);
         }
         break;

      case VC_DSPRO_SENDTO:
         DEBUG_MINOR(printf("Data service SENDTO\n"));
         ret = body.params[1];
         if (ret > DS_MAX_DATA) ret = DS_MAX_DATA;
         ipaddr[0] = (uint8_t)(body.params[2] >> 24);
         ipaddr[1] = (uint8_t)(body.params[2] >> 16);
         ipaddr[2] = (uint8_t)(body.params[2] >> 8);
         ipaddr[3] = (uint8_t)(body.params[2]);
         ret = vc_dsos_sendto(fd, body.data, ret, ipaddr, (uint16_t)(body.params[3]));
         if (ret >= 0) {
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_WRESP | VC_DSPRO_WFLAG; /* guess */
            body.params[2] = ret;
            rlen = 12;
            vc_dsos_unregister_w(fd); /* guess */
            ret = VC_SOCK_OK;
         }
         else {
            vc_dsos_register_w(fd); /* guess until next poll() */
         }
         break;

      case VC_DSPRO_WRITE:
         DEBUG_MINOR(printf("Data service WRITE\n"));
         ret = req.ext_length - 16;
         if (ret > DS_MAX_DATA) ret = DS_MAX_DATA;
         ret = vc_dsos_write(fd, body.data, ret);
         if (ret >= 0) {
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_WRESP;
            if (vc_dsos_writable(fd)) {
               body.params[1] |= VC_DSPRO_WFLAG;
               vc_dsos_unregister_w(fd);
            }
            else {
               vc_dsos_register_w(fd);
            }
            body.params[2] = ret;
            rlen = 12;
            ret = VC_SOCK_OK;
         }
         else {
            vc_dsos_register_w(fd); /* guess until next poll() */
         }
         break;

      case VC_DSPRO_LISTEN:
         DEBUG_MAJOR(printf("Data service LISTEN\n"));
         fd = vc_dsos_listen((uint16_t)body.params[1], body.params[2], &ret);
         if (!VCDSOS_IS_INVALID(fd)) {
            DEBUG_MAJOR(printf("Created TCP socket %d listening at port %d\n",
                               fd, body.params[1]));
            ret = 0;
            vc_dsos_register_r(fd); /* accept is like read */
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_CREATE;
            rlen = 8;
         }
         break;

      case VC_DSPRO_ACCEPT:
         DEBUG_MAJOR(printf("Data service ACCEPT\n"));
         fd = vc_dsos_accept(fd, ipaddr, &ush, &ret);
         if (fd >= 0) {
            DEBUG_MAJOR(printf("Accepted TCP connection (socket %d) from %u.%u.%u.%u port %u\n",
                               fd, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], ush));
            vc_dsos_register_r(body.params[0]);  /* old socket */
            vc_dsos_register_w(fd); /* new socket */
            vc_dsos_register_r(fd);
            body.params[0] = fd;
            body.params[1] = VC_DSPRO_CREATE;
            body.params[2] = (ipaddr[0] << 24) | (ipaddr[1] << 16) |
               (ipaddr[2] << 8) | ipaddr[3];
            body.params[3] = ush;
            ret = 0;
            rlen = 16;
         }
         else {
            /* ?? under what circumstances should we re-register listener? */
         }
         break;
      }

      if (response_required) {
         body.params[0] = VC_HTOV32(body.params[0]);
         body.params[1] = VC_HTOV32(body.params[1]);
         body.params[2] = VC_HTOV32(body.params[2]);
         body.params[3] = VC_HTOV32(body.params[3]);
         req.cmd_code = VC_HTOV32(ret);
         req.ext_length = VC_HTOV16(rlen);
         req.timestamp = VC_HTOV16(0);
         ael = (rlen + 15) & ~15;

         DEBUG_MINOR(printf("Data service response:\n"));
         DEBUG_MINOR(showmsg(&req, &body));

         vc_msgfifo_write_blocking(ds_inum, &req, sizeof(VC_MSGFIFO_CMD_HEADER_T), ds_oevent);
         if (ael) vc_msgfifo_write_blocking(ds_inum, &body, ael, ds_oevent);
         vc_msgfifo_write_flush(ds_inum);
      }
   }
   return rr;
}


/*****************************************************************************
NAME
   vc_dserv_active

SYNOPSIS
   int vc_dserv_active()

RETURNS
   1 if open sockets exist, 0 if not.
*****************************************************************************/

int vc_dserv_active()
{
   return vc_dsos_active();
}


/*****************************************************************************
NAME
   vc_dserv_async

SYNOPSIS
   void vc_dserv_async()

FUNCTION
   Sends an asynchronous notification to VideoCore
*****************************************************************************/

void vc_dserv_async(int fd, int rwflags)
{
   if (ds_inum < 0)
      return;

   DEBUG_MINOR(printf("Data service ASYNC socket=%d flags=%d\n", fd, rwflags));
   body.params[0] = fd;
   body.params[1] = rwflags;
   body.params[0] = VC_HTOV32(body.params[0]);
   body.params[1] = VC_HTOV32(body.params[1]);
   body.params[2] = 0;
   body.params[3] = 0;
   vc_msgfifo_send_command_blocking(ds_inum, VC_DSPRO_ASYNC, 8, &body, ds_oevent);
}

/*****************************************************************************
NAME
   vc_dserv_network_down

SYNOPSIS
   void vc_dserv_networkd_down()

FUNCTION
   Informs VideoCore that host side network is no longer available.
   Network operations should abort quickly, rather than wait for timeouts
*****************************************************************************/

void vc_dserv_network_down(void)
{
   if (ds_inum < 0)
      return;

   DEBUG_MINOR(printf("Data service network down\n"));
   vc_msgfifo_send_command_blocking(ds_inum, VC_DSPRO_NETWORK_DOWN, 0, NULL, ds_oevent);
}

/*****************************************************************************
NAME
   vc_dserv_ppp_to_vc

SYNOPSIS
   int vc_dserv_ppp_to_vc(void *ppp_data, int size)

FUNCTION
   Sends PPP data to VideoCore

RETURNS
   0 if sent okay.
*****************************************************************************/
int vc_dserv_ppp_to_vc(void *ppp_data, int size)
{
   if (ds_inum < 0)
      return 0;

   //if (!ppp_request_mode)
   //   return VC_PPP_REFUSED;

   DEBUG_MINOR(printf("Got [%.*s]\n", size, ppp_data));
   vc_msgfifo_send_command_blocking(ds_inum, VC_DSPRO_PPP_TO_VC, size, ppp_data, ds_oevent);
   return VC_SOCK_OK;
}

