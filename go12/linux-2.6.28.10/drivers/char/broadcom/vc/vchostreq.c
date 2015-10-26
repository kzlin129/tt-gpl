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




#include <linux/string.h>
//#include <stdio.h>
#include <linux/ctype.h>

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

#include "vc_hostreq_defs.h"
#include "vchostreq.h"
#include "vcassert_notify.h"
#include "vchostreq_int.h"
#include "vchostmem.h"

#include <linux/broadcom/vc.h>


/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

#define VC_HRMAX_RDAREA    1

typedef struct
{
   void *startaddr;
   void *endaddr;
} VC_HRMEMAREA_T;

typedef struct
{
   int initted;
   int               inum;       /* fifo reference number */
   volatile uint32_t keys_xid;   /* Outstanding key event ID */

   volatile keys_t   keysend_enabled;
   volatile keys_t   prev_keys;
   volatile keys_t   curr_keys;
   volatile keys_t   keys_mask;
   void              *in_ievent;
   void              *out_ievent;
   int               num_rdareas;
   VC_HRNOTIFY_CALLBACK_T notify_callbacks[ VC_HRNOTIFY_END ];
   VC_HRMEMAREA_T    rdarea[VC_HRMAX_RDAREA];

   /* Storage both request data and its response */
   int               cmd_data[VC_HRMAX_VIBRATOR_SEQ];
} VC_HRGLOBALS_T;

/******************************************************************************
Static data.
******************************************************************************/

static VC_HRGLOBALS_T vc_hrg = {0, -1};
static void *hostreq_lock = NULL;

/******************************************************************************
Static functions.
******************************************************************************/

static int vc_hr_message_handler(void);

/*---------------------------------------------------------------------------*/

/******************************************************************************
NAME
   vc_hostreq_init

SYNOPSIS
   int vc_hostreq_init(void)

FUNCTION
   Initialise the host request system for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

int vc_hostreq_init (void)
{
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (hostreq_lock == NULL)
      hostreq_lock = vc_lock_create();

   vc_lock_obtain(hostreq_lock);

   vc_hrg.inum = -1;
   vc_hrg.keys_xid = 0;

   vc_hrg.prev_keys = 0;
   vc_hrg.curr_keys = 0;
   vc_hrg.keys_mask = 0;
   vc_hrg.keysend_enabled = 1;

   if (!vc_hrg.initted) {
      vc_hrg.in_ievent = vc_event_create();
      vc_assert(vc_hrg.in_ievent);
      vc_hrg.out_ievent = vc_event_create();
      vc_assert(vc_hrg.out_ievent);

      for (i = 0; i < VC_HRMAX_RDAREA; i++) {
         vc_hrg.rdarea[i].startaddr = NULL;
         vc_hrg.rdarea[i].endaddr = NULL;
      }

      for (i = 0; i < VC_HRNOTIFY_END; i++)
      {
         vc_hrg.notify_callbacks[i] = NULL;
      }
      /* ### For now... ### */
      vc_hrg.num_rdareas = 1;
      vc_hrg.rdarea[0].startaddr = (void *)4;
      vc_hrg.rdarea[0].endaddr = (void *)0xfffffffc;

      vc_hrg.initted = 1;
   }
   /* Initialize the assert handler */
   vc_assert_notify_init();

   // We simply loop through every interface that there is and look for one
   // that claims to be a host request service.
   for (i = 0; i < VC_NUM_INTERFACES; i++)
   {
      if (vc_sharedmem_header.iface[i])
      {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface,
                                              vc_interface_base+vc_sharedmem_header.iface[i],
                                                                  sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_HOSTREQ)
         {
            // Gotcha!
            vc_hrg.inum = i;
            vc_interface_register_event_int(vc_hrg.in_ievent, (1<<vc_hrg.inum));
            vc_interface_register_event_int(vc_hrg.out_ievent, (1<<vc_hrg.inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   vc_lock_release(hostreq_lock);
   return vc_hrg.inum;
}

/******************************************************************************
NAME
   vc_hostreq_stop

SYNOPSIS
   void vc_hostreq_stop()

FUNCTION
   This tells us that the hostreq service has stopped, thereby preventing
   any of the functions from doing anything.

RETURNS
   void
******************************************************************************/

void vc_hostreq_stop () {
   // Don't want anyone to be using this when we zap it.
   vc_lock_obtain(hostreq_lock);
   vc_hrg.inum = -1;
   // Ensure the host stops sending keys
   vc_hostreq_capturekeys(0);
   vc_interface_register_event_int(vc_hrg.in_ievent, 0);
   vc_interface_register_event_int(vc_hrg.out_ievent, 0);
   vc_lock_release(hostreq_lock);
}

/******************************************************************************
NAME
   vc_hostreq_inum

SYNOPSIS
   int vc_hostreq_inum()

FUNCTION
   Return the hostreq service number (-1 if not running).

RETURNS
   int
******************************************************************************/

int vc_hostreq_inum () {
   return vc_hrg.inum;
}

/******************************************************************************
NAME
   vc_hostreq_set_notify

SYNOPSIS
   int32_t vc_hostreq_set_notify( const VC_HRNOTIFY_T notify_event, VC_HRNOTIFY_CALLBACK_T notify_callback )

FUNCTION
   Sets a user notify function to be called for the given notify event

RETURNS
   0 on success, < 0 on failure
******************************************************************************/
int32_t vc_hostreq_set_notify( const VC_HRNOTIFY_T notify_event, VC_HRNOTIFY_CALLBACK_T notify_callback )
{
   if (!vc_hrg.initted)
      return -2;

   if ( (notify_event > VC_HRNOTIFY_START) && (notify_event < VC_HRNOTIFY_END) )
   {
      vc_hrg.notify_callbacks[ notify_event ] = notify_callback;
   }
   else
      return -1;
   return 0;
}

/******************************************************************************
NAME
   vc_hostreq_keychange

SYNOPSIS
   void vc_hostreq_keychange(keys_t keys)

FUNCTION
   Host/Application calls this function when there is any change in
   the 'grabbed' key status.

RETURNS
   Successful completion: Response code of reply
   Otherwise: -
******************************************************************************/

void vc_hostreq_keychange(keys_t keys)
{
   /* Grab lock to check status and possibly write to fifo */
   vc_lock_obtain(hostreq_lock);

   if (vc_hrg.inum < 0) {
      vc_lock_release(hostreq_lock);
      return;
   }

   vc_hrg.curr_keys = keys;
   if (vc_hrg.keysend_enabled && ((vc_hrg.prev_keys ^ vc_hrg.curr_keys) & vc_hrg.keys_mask))
   {
      VC_MSGFIFO_CMD_HEADER_T hdr;
      int wdata[4];

      vc_hrg.keysend_enabled = 0;   /* Prevent more sends until acknowledged */
      vc_hrg.keys_xid++;

      hdr.sync = VC_HTOV32(VC_CMD_SYNC);
      hdr.xid  = VC_HTOV32(vc_hrg.keys_xid);
      hdr.cmd_code = VC_HTOV32(VC_HOSTREQ_KEYEVENT);
      hdr.ext_length = VC_HTOV16(sizeof(keys_t));
      hdr.timestamp = VC_HTOV16(0);

      /* Only send the keys that are of interest */
      wdata[0] = VC_HTOV32( vc_hrg.curr_keys & vc_hrg.keys_mask );

      /* There will always be enough space in the fifo for full messages */
//      vc_assert(vc_msgfifo_output_space_available(vc_hrg.inum) >= (sizeof(VC_MSGFIFO_CMD_HEADER_T)+16));
//    OJW This assert may go off as we don't refresh the write pointers until the line below.

      vc_msgfifo_write_blocking(vc_hrg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_hrg.out_ievent);
      vc_msgfifo_write_blocking(vc_hrg.inum, wdata, 16, vc_hrg.out_ievent);
      vc_msgfifo_write_flush(vc_hrg.inum);

      vc_hrg.prev_keys = vc_hrg.curr_keys;
   }

   /* Release fifo write lock */
   vc_lock_release(hostreq_lock);
}

/******************************************************************************
NAME
   vc_hr_message_handler

SYNOPSIS
   int vc_hr_message_handler(void)

FUNCTION
   Handle a host request message from the co-processor.

RETURNS
   int
******************************************************************************/

static int vc_hr_message_handler(void)
{
  int gotone = 0;

  if (vc_hrg.inum < 0)
    return 0;

  vc_msgfifo_read_refresh(vc_hrg.inum);

  if (vc_msgfifo_input_bytes_available(vc_hrg.inum) >= sizeof(VC_MSGFIFO_CMD_HEADER_T))
  {
    VC_MSGFIFO_CMD_HEADER_T hdr;
    uint32_t ael;

    gotone = 1;

    /* Read the request header, or key event ack */
    vc_msgfifo_read_blocking(vc_hrg.inum, &hdr,
                             sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_hrg.in_ievent);
    vc_assert(VC_VTOH32(hdr.sync) == VC_CMD_SYNC);

    hdr.cmd_code = VC_VTOH32(hdr.cmd_code);
    hdr.ext_length = VC_VTOH16(hdr.ext_length);
    hdr.timestamp = VC_VTOH16(hdr.ext_length);
    hdr.xid = VC_VTOH32(hdr.xid);

    ael = (hdr.ext_length + 15) & ~15;

    /* Read in the message body */
    if (ael > 0)
    {
      vc_assert(ael <= VC_HRMAX_CMD_DATA);
      vc_msgfifo_read_blocking(vc_hrg.inum, &vc_hrg.cmd_data, ael, vc_hrg.in_ievent);
    }
    vc_msgfifo_read_flush(vc_hrg.inum);

    if (hdr.cmd_code == VC_HOSTREQ_KEYACK)
    {
      /* Process key event ack message */

      vc_assert(vc_hrg.keys_xid == hdr.xid);

      /* Grab lock to check status and possibly write to fifo */
      vc_lock_obtain(hostreq_lock);

      if ((vc_hrg.prev_keys ^ vc_hrg.curr_keys) & vc_hrg.keys_mask)
      {
        /* Key state has already changed, so send a new key event */
        VC_MSGFIFO_CMD_HEADER_T hdr;
        int wdata[4];

        vc_hrg.keys_xid++;

        hdr.sync = VC_HTOV32(VC_CMD_SYNC);
        hdr.xid  = VC_HTOV32(vc_hrg.keys_xid);
        hdr.cmd_code = VC_HTOV32(VC_HOSTREQ_KEYEVENT);
        hdr.ext_length = VC_HTOV16(sizeof(keys_t));
        hdr.timestamp = VC_HTOV16(0);

        /* Only send the keys that are of interest */
        wdata[0] = VC_HTOV32( vc_hrg.curr_keys & vc_hrg.keys_mask );

        vc_msgfifo_write_blocking(vc_hrg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_hrg.out_ievent);
        vc_msgfifo_write_blocking(vc_hrg.inum, wdata, 16, vc_hrg.out_ievent);
        vc_msgfifo_write_flush(vc_hrg.inum);

        vc_hrg.prev_keys = vc_hrg.curr_keys;
      }
      else
      {
        /* Otherwise re-enable sending of key events */
        vc_hrg.keysend_enabled = 1;
      }

      /* Release fifo write lock */
      vc_lock_release(hostreq_lock);
    }
    else
    {
      /* Process host request command messages */
      int retval = VC_RESP_OK;
      int rlen = 0;
      void *rdata = &vc_hrg.cmd_data;
      void *rdstart;
      void *rdend;
      int i,tmplen,fnd;
      int cmdd0;

         /* All data except for text strings is 32-bits, so endian reformat it now if necessary */
         switch (hdr.cmd_code) {
            // These host request types are all strings and so do not endian reformat
            case VC_HOSTREQ_CONFIGTEXT:
            case VC_HOSTREQ_RENDERTEXT:
            case VC_HOSTREQ_LINKDATA:
               break;

            // By default endian reformat
            default:
               for (i = 0; i < (int)(hdr.ext_length>>2); i++)
                  vc_hrg.cmd_data[i] = VC_VTOH32(vc_hrg.cmd_data[i]);
               break;
      }
      cmdd0 = vc_hrg.cmd_data[0];

      switch (hdr.cmd_code)
      {
      case VC_HOSTREQ_RESET:
        /* What is there to reset ? */
        break;
      case VC_HOSTREQ_CAPTUREKEYS:
        {
          keys_t keymask = cmdd0;
          /* I don't think we need to worry about locks? */
          /* Zero our prev keys if we are starting capture afresh */
          if (vc_hrg.keys_mask == 0 && keymask != 0)
            vc_hrg.prev_keys = 0;
          vc_hrg.keys_mask = keymask;
          /* Following is informative rather than mandatory */
          vc_hostreq_capturekeys(vc_hrg.keys_mask);
        }
        break;
      case VC_HOSTREQ_VIBRATORPLAY:
        vc_hostreq_vibratorplay(vc_hrg.cmd_data);
        break;
      case VC_HOSTREQ_VIBRATORSTOP:
        vc_hostreq_vibratorstop();
        break;
      case VC_HOSTREQ_KEYLIGHT:
        vc_hostreq_keylight(cmdd0);
        break;
      case VC_HOSTREQ_SETLEDS:
        vc_hostreq_setleds(cmdd0);
        break;
      case VC_HOSTREQ_TIME:
        vc_hrg.cmd_data[0] = vc_hostreq_time();
        vc_hrg.cmd_data[0] = VC_VTOH32( vc_hrg.cmd_data[0] );
        rlen = sizeof(int);
        break;
      case VC_HOSTREQ_CONFIGTEXT:
        vc_hostreq_configtext((char *)vc_hrg.cmd_data, hdr.ext_length);
        break;
      case VC_HOSTREQ_RENDERTEXT:
        vc_hostreq_rendertext((char *)vc_hrg.cmd_data, hdr.ext_length);
        break;
      case VC_HOSTREQ_LINKDATA:
         vc_hostreq_linkdata((char *)vc_hrg.cmd_data, hdr.ext_length);
         break;
      case VC_HOSTREQ_DMB_FIC:
         vc_hostreq_dmb_fic((char *)vc_hrg.cmd_data, hdr.ext_length);
         break;
      case VC_HOSTREQ_DMB_PAD:
         vc_hostreq_dmb_pad((char *)vc_hrg.cmd_data, hdr.ext_length);
         break;
      case VC_HOSTREQ_DMB_DATA:
         vc_hostreq_dmb_data((char *)vc_hrg.cmd_data, hdr.ext_length);
         break;
      case VC_HOSTREQ_KEYIN:
         vc_hostreq_keyin();
         break;
      case VC_HOSTREQ_NOTIFY:
        /* NOTE: calls user loaded functions AND generic function */
        if (cmdd0 >= 0 && cmdd0 < VC_HRNOTIFY_END)
        {
          if (vc_hrg.notify_callbacks[cmdd0])
          {
            (*vc_hrg.notify_callbacks[ cmdd0 ])(cmdd0, vc_hrg.cmd_data[1]);
          }
        }
        vc_hostreq_notify(cmdd0, vc_hrg.cmd_data[1]);
        break;
      case VC_HOSTREQ_WRITEMEM:
        /* Read data from VC address into host address */
        /*    msg[0] = hostaddr;   msg[1] = vc_buf;  msg[2] = len; */
        {
           int channel = (int)vc_hrg.cmd_data[0];
          void *host_addr = (void*)vc_hrg.cmd_data[1];
          void *vc_addr   = (void*)vc_hrg.cmd_data[2];
          int   len       = (int)vc_hrg.cmd_data[3];

          //tprintf("\n%s%:%d writemem( %08x ) len %d from VC &0x%08x\n", __FILE__, __LINE__, host_addr, len, vc_addr );

          vc_hrg.cmd_data[0] = vchostreq_writemem( host_addr, vc_addr, len, channel );

          /*{
            int i;
            for( i=0; i< len;i++ ) tprintf(" 0x%02x ", ((char*)host_addr)[i]);
          }*/

          rlen = sizeof(int);
        }
        break;

      case VC_HOSTREQ_MALLOC:
        //tprintf("\nMalloc: %d bytes\n",vc_hrg.cmd_data[0] );
        /* Malloc the requested number of bytes. Return pointer to memory area */
        vc_hrg.cmd_data[0] = (int) vchostreq_malloc( (size_t) vc_hrg.cmd_data[0] );
        //tprintf("Result %x \n",vc_hrg.cmd_data[0] );
        rlen = sizeof(int);
        break;

      case VC_HOSTREQ_FREE:
        //tprintf("\nFree: %x \n",vc_hrg.cmd_data[0] );
        /* Free the requested area of memory, no response to vc */
        if( vc_hrg.cmd_data[0] != 0 )
        {
          vchostreq_free( (void*)vc_hrg.cmd_data[0] );
          rlen = 0;
        }

        break;

      case VC_HOSTREQ_MEMMOVE:
        /* Move host side memory */
        {
          void *dest_addr = (void*)vc_hrg.cmd_data[0];
          void *src_addr  = (void*)vc_hrg.cmd_data[1];
          int   len       = (int)vc_hrg.cmd_data[2];
          retval = vchostreq_memmove( dest_addr, src_addr, len );
          rlen = sizeof(int);

        }
        break;
            case VC_HOSTREQ_READMEM:
               /* Read directly from memory */
               /*
                  Protect this by returning an error if access is not inside a
                  pre-designated 'nonprotected' area of memory (could have several of these)
               */
               rdstart = (void *)(cmdd0);
               tmplen = (vc_hrg.cmd_data[1] + 15) & (~15);  /* Round up */
               rdend = (void *)(((char *)rdstart)+tmplen);
               fnd = 0;
               for (i = 0; i < vc_hrg.num_rdareas; i++)
               {
                  VC_HRMEMAREA_T *rdarea = &vc_hrg.rdarea[i];
                  if (rdstart >= rdarea->startaddr && rdstart < rdarea->endaddr
                                                   && rdend <= rdarea->endaddr)
                  {
                     fnd = 1;
                     break;
                  }
               }
               if (fnd)
               {
                  /* Write directly to user buffer on videocore */
                  vc_host_write_consecutive(vc_hrg.cmd_data[2], rdstart, tmplen, 0);
                  rlen = sizeof(int);
                  vc_hrg.cmd_data[0] = VC_HRERR_OK;
               }
               else
               {
                  /* Not in any non-protected area */
                  retval = VC_RESP_ERROR;
                  rlen = sizeof(int);
                  vc_hrg.cmd_data[0] = VC_HRERR_MEMADDR;
               }
               break;

      case VC_HOSTREQ_READMEM_3D:
         {
            int channel = vc_hrg.cmd_data[0];
            uint32_t dest = vc_hrg.cmd_data[1];
            unsigned int tile_width = vc_hrg.cmd_data[2];
            unsigned int tile_height = vc_hrg.cmd_data[3];
            unsigned int num_tiles = vc_hrg.cmd_data[4];
            unsigned int tile_d_pitch = vc_hrg.cmd_data[5];
            unsigned int d_pitch = vc_hrg.cmd_data[6];
            void *src = (void *)vc_hrg.cmd_data[7];
            int tile_s_pitch = vc_hrg.cmd_data[8];
            int s_pitch = vc_hrg.cmd_data[9];
            uint32_t frac_offset = vc_hrg.cmd_data[10];
            uint32_t frac_int = vc_hrg.cmd_data[11];

            VC_DEBUG( Info, "src(0x%lx) dst(0x%x) tile_width(%d) tile_height(%d) "
                      "num_tiles(%d) tile_d_pitch(%d) d_pitch(%d) tile_s_pitch(%d) "
                      "s_pitch(%d) frac_o(%d) frac_int(%d) channel(%d)\n",
                      (unsigned long)src, dest, tile_width, tile_height, num_tiles, tile_d_pitch,
                      d_pitch, tile_s_pitch, s_pitch, frac_offset, frac_int, channel );

            vc_hrg.cmd_data[0] =
               vchostreq_readmem_3d(dest, tile_width, tile_height, num_tiles, tile_d_pitch, d_pitch,
                                    src, tile_s_pitch, s_pitch, frac_offset, frac_int, channel);
            vc_hrg.cmd_data[0] = VC_HTOV32(vc_hrg.cmd_data[0]);
            rlen = sizeof(int);
         }
         break;

            default:;
               retval = VC_RESP_ERROR;
               rlen = sizeof(int);
               vc_hrg.cmd_data[0] = VC_HRERR_UNSUPPORTED;
      }

      hdr.cmd_code = VC_HTOV32(retval);
      hdr.ext_length = VC_HTOV16(rlen);
      hdr.timestamp = VC_HTOV16(0);
      hdr.xid = VC_HTOV32(hdr.xid);

      ael = (rlen + 15) & ~15;

      /* Grab lock to write to fifo */
      vc_lock_obtain(hostreq_lock);

      vc_msgfifo_write_blocking(vc_hrg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_hrg.out_ievent);
      if (ael)
      {
        vc_msgfifo_write_blocking(vc_hrg.inum, rdata, ael, vc_hrg.out_ievent);
      }
      vc_msgfifo_write_flush(vc_hrg.inum);

      /* Release fifo write lock */
      vc_lock_release(hostreq_lock);
    }
    /* Need to deal with stuff in fifo when host is reset? - NO */
  }

  return gotone;
}


/******************************************************************************
NAME
   vc_hostreq_poll_message_fifo

SYNOPSIS
   int vc_hostreq_poll_message_fifo(void)

FUNCTION
   Public interface of message handler...

RETURNS
   0 if successful
******************************************************************************/

int vc_hostreq_poll_message_fifo(void)
{
   int retval;

   if (vc_hrg.inum < 0)
      return 0;

   do {
      retval = vc_hr_message_handler();
   } while (retval);

   return retval;
}


/******************************************************************************
NAME
   vc_hostreq_read_event

SYNOPSIS
   void *vc_hostreq_read_event(void)

FUNCTION
   Return the read event object for the host request service.

RETURNS
   void *
******************************************************************************/
void *vc_hostreq_read_event (void)
{
   return vc_hrg.in_ievent;
}

