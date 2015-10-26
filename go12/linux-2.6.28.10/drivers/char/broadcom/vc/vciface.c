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




#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"

#include <linux/broadcom/vc.h>

// TODO: remove chip specific code - this is only required for debug code.
#if defined( CONFIG_ARCH_BCM116X )&& defined( __KERNEL__ )
   #include <linux/broadcom/hw_cfg.h>
   #include <asm/arch/reg_umi.h>
#endif


#if !defined( __KERNEL__ )
#include <string.h>
#else
#include <linux/broadcom/gpio.h>
#endif

/******************************************************************************
Global data.
******************************************************************************/

uint32_t vc_interface_base;
VC_SHAREDMEM_HEADER_T vc_sharedmem_header;

/******************************************************************************
Local types and defines.
******************************************************************************/

#define VC_INTERFACE_MAX_REG_EVENTS 16

typedef struct
{
   void *event;
   int mask;
} REGISTERED_EVENT_T;

/******************************************************************************
Static data.
******************************************************************************/

static void *interface_lock = NULL;

static REGISTERED_EVENT_T registered_events[VC_INTERFACE_MAX_REG_EVENTS];
static int num_reg_events = 0;

static unsigned char testPattern[ 14 ] =
{
   0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0xAA, 0xAA, 0x55, 0x55, 0xFF, 0xFF
};


/******************************************************************************
Static functions.
******************************************************************************/

static int read_header(void);
static int read_int_bits(void);
static int write_int_bits(void);

void show_vc02( void );

/******************************************************************************
NAME
   vc_interface_init

SYNOPSIS
   int vc_interface_init()

FUNCTION
   Initialise the host side of the interface. Return non-zero for failure.

RETURNS
   int
******************************************************************************/

int vc_interface_init (void) {
   int status;
   uint32_t buffer[4];
   int      i;

   status = vc_host_init();
   if (status != 0) {
       printk( "vc_interface_init: vc_host_init failed\n" );
      // Perhaps VideoCore was not there? Tell our caller there is a problem.
      return 1;
   }

#if defined( __KERNEL__ ) && defined( HW_CFG_GPIO_BAD_READ )
   gpio_set_pin_type( HW_CFG_GPIO_BAD_READ, REG_GPIO_IOTR_GPIO_OUTPUT );
   gpio_set_pin_val( HW_CFG_GPIO_BAD_READ, 0 );
#endif

   // Read address of interface area.
   status = vc_host_read_consecutive(buffer, VC_SHAREDPTR_ADDR & ~15, 16, 0);
   if ( status != 0 )
   {
       printk( "vc_interface_init: read address of intf area failed\n" );
       return status;
   }

   if(VC_VTOH32(buffer[2]) != 1) { // Read from VC_SHAREDPTR_ADDR - VMCS sets this to 1
      printk( "vc_interface_init: VMCS is not running %x!\n", VC_VTOH32(buffer[2]) );
      return 1;   // VMCS is not running
   }

   vc_interface_base = VC_VTOH32(buffer[3]);

   VC_DEBUG( Verbose, "vc_interface_base = 0x%08x\n", vc_interface_base );

   if ((vc_interface_base == 0) || ((vc_interface_base & 3) != 0))
   {
      printk( KERN_ERR "vc_interface_base is NULL - Is your program running on the VC02\n" );
      return 1;   // Sanity check - pointer must be non-null and aligned to a 4-byte address
   }

   // Read shared memory header.
   status = read_header();
   if ( status != 0 )
   {
       printk( "vc_interface_init: read_header failed\n" );
       return status;
   }

   // Probably advisable to have a lock to protect multiple threads trying to
   // set interface bits at once. We protect reads too, just to be sure another
   // thread doesn't use half updated values.
   if (interface_lock == NULL)
      interface_lock = vc_lock_create();

   // Start us off in polling mode. The host can put us into interrupt mode once
   // they have done enough work to set up and process the interrupts.
   // (NOTE: I think this now does nothing. DAP.)
   vc_interface_set_polling(1);

   // Very hookey hack. Write some bytes into the dummy values after the vin_int_ack
   // so we can use them to tell if we're doing something valid.

   {
      int offset = (uint32_t)&vc_sharedmem_header.vin_int_req - (uint32_t)&vc_sharedmem_header;

      vc_host_read_consecutive(&vc_sharedmem_header.vin_int_req, vc_interface_base + offset, sizeof(VC_SHAREDMEM_HEADER_T) - offset, 0);

      memcpy( vc_sharedmem_header.dummy1, testPattern, sizeof( testPattern ));
      memcpy( vc_sharedmem_header.dummy2, testPattern, sizeof( testPattern ));

      vc_host_write_consecutive( vc_interface_base + offset, &vc_sharedmem_header.vin_int_req, sizeof(VC_SHAREDMEM_HEADER_T) - offset, 0);
   }

   // Do not set this as we get reinitialised after being powered down and up again
   // and we do not expect services to recreate their events.
   //num_reg_events = 0;

   for (i = 0; i < VC_NUM_INTERFACES; i++)
   {
       VC_GENERIC_INTERFACE_T   intf;
       char *stypeStr;

       if ( vc_sharedmem_header.iface[i] == 0 )
       {
           VC_DEBUG( Verbose, "vc_interface_init: iface[%d]: 0x%04x\n",
                   i, vc_sharedmem_header.iface[i] );
       }
       else
       {
           vc_host_read_consecutive(&intf, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
           intf.stype = VC_VTOH16(intf.stype);

           switch ( intf.stype )
           {
               case VC_STYPE_GENCMD:        stypeStr = "gencmd";    break;
               case VC_STYPE_DISPMAN:       stypeStr = "dispman";   break;
               case VC_STYPE_TOUCHSCREEN:   stypeStr = "touch";     break;
               case VC_STYPE_DATASERVICE:   stypeStr = "dataSvc";   break;
               case VC_STYPE_FILESERVICE:   stypeStr = "fileSvc";   break;
               case VC_STYPE_HOSTREQ:       stypeStr = "hostReq";   break;
               case VC_STYPE_FRMFWD:        stypeStr = "frmfwd";    break;
               default:                     stypeStr = "***";       break;
           }

           if ( intf.itype == VC_ITYPE_MSGFIFO )
           {
              VC_MSGFIFO_INTERFACE_T   msgFifo;

              VC_HOST_READ_BYTESWAPPED_16( &msgFifo, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_MSGFIFO_INTERFACE_T), 0);


              VC_DEBUG( Verbose, "vc_interface_init: iface[%d]: 0x%04x stype:%d '%s' msgFifo %d bytes (to-vc) %d bytes (from-vc)\n",
                      i, vc_sharedmem_header.iface[i], intf.stype, stypeStr,
                      msgFifo.vin_fmax  - msgFifo.vin_fmin  - VC_INTERFACE_BLOCK_SIZE,
                      msgFifo.vout_fmax - msgFifo.vout_fmin - VC_INTERFACE_BLOCK_SIZE );
           }
           else
           {
              VC_DEBUG( Verbose, "vc_interface_init: iface[%d]: 0x%04x stype:%d '%s'\n",
                      i, vc_sharedmem_header.iface[i], intf.stype, stypeStr );
           }
       }
   }

   return 0;
}

/******************************************************************************
NAME
   vc_interface_query_req

SYNOPSIS
   int vc_interface_query_req()

FUNCTION
   Return a bitmask with a 1 for every interface where we HAVE already requested
   an interrupt that has not been acknowledge. This function reads the int_req/ack
   bits in the shared memory header. Negative return values indicate failure.

RETURNS
   int
******************************************************************************/

int vc_interface_query_req (void) {
   int status, retval;

   vc_lock_obtain(interface_lock);
   status = read_int_bits();
   vc_assert(status == 0);

   retval = vc_sharedmem_header.vin_int_req ^ vc_sharedmem_header.vin_int_ack;
   vc_lock_release(interface_lock);
   return retval;
}

/******************************************************************************
NAME
   vc_interface_set_req

SYNOPSIS
   int vc_interface_set_req(int mask)

FUNCTION
   Request an interrupt for every service that has a 1 set in the passed mask.
   This function reads the int_req/ack bits, and writes back the appropriately
   modified values. The current state of the requested interrupts is returned.
   Does not actually send the interrupt. Negative return values indicate failure.

RETURNS
   int
******************************************************************************/

#if defined( __KERNEL__ )
typedef struct {
   int vin_int_req;
   int vin_int_ack;
   int new_vin_int_req;
} VC_INTERFACE_DEBUG_T;
static VC_INTERFACE_DEBUG_T vc_interface_debug[8];

int vc_interface_set_req (int inum) {
   int status;
   int already_requested;
   int retval;
   int mask = 1<<inum;

   vc_lock_obtain(interface_lock);

   status = read_int_bits();
   vc_assert(status == 0);

   vc_interface_debug[inum].vin_int_req = vc_sharedmem_header.vin_int_req;
   vc_interface_debug[inum].vin_int_ack = vc_sharedmem_header.vin_int_ack;

   // Request interrupts only where they are not already requested.
   already_requested = vc_sharedmem_header.vin_int_req ^ vc_sharedmem_header.vin_int_ack;
   mask &= ~already_requested;
   vc_sharedmem_header.vin_int_req ^= mask;
   vc_interface_debug[inum].new_vin_int_req = vc_sharedmem_header.vin_int_req;
   status = write_int_bits();
   vc_assert(status == 0);

   retval = vc_sharedmem_header.vin_int_req ^ vc_sharedmem_header.vin_int_ack;

   vc_lock_release(interface_lock);

   return retval;
}

/******************************************************************************
NAME
   vc_interface_send_interrupt

SYNOPSIS
   void vc_interface_send_interrupt(int mask)

FUNCTION
   Request an interrupt for every service that has a 1 set in the passed mask.
   This function reads the int_req/ack bits, and writes back the appropriately
   modified values. The current state of the requested interrupts is returned.
   This version does actually send the interrupt.

RETURNS
   int
******************************************************************************/

void vc_interface_send_interrupt (int mask)
{
   VC_DEBUG( MsgFifo, "Mask: 0x%08x\n", mask );

   vc_interface_set_req(mask);
   vc_host_send_interrupt(0);
}
#endif

#if defined( __KERNEL__ )


/******************************************************************************
NAME
   vc_interface_register_event_int

SYNOPSIS
   int vc_interface_register_event_int(void *event, int mask)

FUNCTION
   Register with the interrupt handler an event to be signalled when a vc->host
   interrupt occurs for any service corresponding to the bit(s) set in mask.
   A single event can be signalled by (any of) multiple interrupt sources,
   though each event can only be registered once, and multiple events can
   be signalled from one interrupt source. We assume this is done at start-up,
   with no contention.

RETURNS
   0 if event is successfully registered
******************************************************************************/
int vc_interface_register_event_int(void *event, int mask)
{
   int i;

   /* Check to see if the event has already been registered. If so, simply
      update the mask. */
   for (i = 0; i < num_reg_events; i++) {
      if (registered_events[i].event == event) {
         registered_events[i].mask = mask;
         return 0;
      }
   }

   // Not yet registered. Add a new entry to the registered_events table.
   if (num_reg_events >= VC_INTERFACE_MAX_REG_EVENTS)
      return 1;

   VC_DEBUG( HostIrq, "Registering event 0x%08lx, mask 0x%08x\n", (unsigned long)event, mask );

   registered_events[num_reg_events].event = event;
   registered_events[num_reg_events].mask = mask;
   num_reg_events++;

   return 0;
}

void vc_interface_unregister_event_int(void *event)
{
    int                 i;
    unsigned long       flags;
    static spinlock_t   lock = SPIN_LOCK_UNLOCKED;

    spin_lock_irqsave( &lock, flags );

    i = 0;

    while ( i < num_reg_events )
    {
        if ( registered_events[i].event == event )
        {
            num_reg_events--;
            if ( i < num_reg_events )
            {
                registered_events[ i ].event = registered_events[ num_reg_events ].event;
                registered_events[ i ].mask  = registered_events[ num_reg_events ].mask;

                // Don't increment i in this case, which will cause the "new"
                // event that we moved in from the end to be checked.

                continue;
            }
        }
        i++;
    }
    spin_unlock_irqrestore( &lock, flags );
}

#endif

/******************************************************************************
NAME
   vc_interface_interrupt_handler

SYNOPSIS
   int vc_interface_interrupt_handler(int mask)

FUNCTION
   Handle (any) interrupts from VideoCore.
   To be called at task/thread level (not in an ISR)
   Reads and acks any service interrupt requests from VideoCore, and
   then signals the individual interrupt(s) to the handling task(s).

RETURNS
   Non-zero if any interrupt was present
******************************************************************************/

int vc_interface_interrupt_handler(void)
{
   int status;
   int host_requests;
   int i;

   vc_lock_obtain(interface_lock);

   /* Read request and ack bits from the interface */
   status = read_int_bits();
   vc_assert(status == 0);
   host_requests = vc_sharedmem_header.vout_int_req ^ vc_sharedmem_header.vout_int_ack;
   if (host_requests == 0) {
      vc_lock_release(interface_lock);
      return 0;
   }

   /* Acknowledge all requests immediately */
   vc_sharedmem_header.vout_int_ack ^= host_requests;
   status = write_int_bits();
   vc_assert(status == 0);

   vc_lock_release(interface_lock);

   VC_DEBUG( HostIrq, "host_requests:0x%02x\n", host_requests );

   if (host_requests)
   {
      /* Signal all registered events that have a mask matching any request */
      REGISTERED_EVENT_T *reg_event = registered_events;
      for (i = 0; i < num_reg_events; i++)
      {
         if (host_requests & reg_event->mask)
         {
             VC_DEBUG( HostIrq, "Setting event 0x%08lx, mask 0x%02x\n", (unsigned long)reg_event->event, reg_event->mask );
             vc_event_set(reg_event->event);
         }
         reg_event++;
      }
   }
   return (host_requests);
}

/******************************************************************************
NAME
   vc_interface_polling

SYNOPSIS
   int vc_interface_polling(void)

FUNCTION
   Return non-zero if we are in polling and not in interrupt driven mode.

RETURNS
   Non-zero if in polling mode.
******************************************************************************/

int vc_interface_polling (void) {
   // Deprecated function
   return 0;
}

/******************************************************************************
NAME
   vc_interface_set_polling

SYNOPSIS
   void vc_interface_polling(int onoff)

FUNCTION
   Set us into polling mode (pass 1) or interrupt driven mode (0).

RETURNS
   -
******************************************************************************/

void vc_interface_set_polling (int onoff) {
   // Deprecated function
   vc_host_interrupts(!onoff);
}

/******************************************************************************
NAME
   vc_host_get_app

SYNOPSIS
   int vc_host_get_app()

FUNCTION
   Return the application that is running on VideoCore. Zero means no app is running.

RETURNS
   int
******************************************************************************/

int vc_host_get_app (void) {
   int status;
   uint32_t buffer;
   // Read app id.
   status = vc_host_read_consecutive(&buffer, VC_APP_ADDRESS, 4, 0);
   vc_assert(status == 0);
   return VC_VTOH32(buffer);
}

/******************************************************************************
Static functions definitions.
******************************************************************************/

int read_header (void) {
   // Read up to vin_int_req.
   int status;
   unsigned int i;
   int offset = (uint32_t)&vc_sharedmem_header.vin_int_req - (uint32_t)&vc_sharedmem_header;
   status = vc_host_read_consecutive(&vc_sharedmem_header, vc_interface_base, offset, 0);
   vc_assert(status == 0);
   for (i = 0; i < sizeof(vc_sharedmem_header.iface)/sizeof(uint16_t); i++)
      vc_sharedmem_header.iface[i] = VC_VTOH16(vc_sharedmem_header.iface[i]);
   return status;
}


int read_int_bits (void) {
   // Read from vin_int_req to the end.
   int status;
   int offset = (uint32_t)&vc_sharedmem_header.vin_int_req - (uint32_t)&vc_sharedmem_header;
   int   attempt = 0;

   static int reportCount = 0;

   if ( reportCount < 6 )
   {
      if ( vc_sharedmem_header.vin_int_ack >= 0x20 )
      {
         printk( KERN_ERR "*****\n" );
         printk( KERN_ERR "***** vin_int_ack: 0x%02x (before read)\n", vc_sharedmem_header.vin_int_ack );
         printk( KERN_ERR "*****\n" );
         reportCount++;

         // Dump shared memory data structs.
         #if defined( __KERNEL__ )
         {
            show_vc02();
         }
         #endif
      }
   }

   do
   {
#if defined( __KERNEL__ )
      if ( attempt >= 1 )
      {
#if defined( HW_CFG_GPIO_BAD_READ )
         gpio_set_pin_val( HW_CFG_GPIO_BAD_READ, 1 );
#endif

         vc_dump_mem( "Bad  Data", vc_interface_base + offset, &vc_sharedmem_header.vin_int_req, sizeof(VC_SHAREDMEM_HEADER_T) - offset );
      }
#endif
      status = vc_host_read_consecutive(&vc_sharedmem_header.vin_int_req, vc_interface_base + offset, sizeof(VC_SHAREDMEM_HEADER_T) - offset, 0);
      vc_assert(status == 0);

      if ( attempt++ > 10 )
      {
         printk( KERN_ERR "*****\n" );
         printk( KERN_ERR "***** read_int_bits failed after %d attempts\n", attempt );
         printk( KERN_ERR "*****\n" );
         status = -1;

         // Dump shared memory data structs.
         #if defined( __KERNEL__ )
         {
            show_vc02();
         }
         #endif

         break;
      }

   } while (( memcmp( vc_sharedmem_header.dummy1, testPattern, sizeof( testPattern )) != 0 )
        ||  ( memcmp( vc_sharedmem_header.dummy2, testPattern, sizeof( testPattern )) != 0 ));

#if defined( __KERNEL__ ) && defined( HW_CFG_GPIO_BAD_READ )
   gpio_set_pin_val( HW_CFG_GPIO_BAD_READ, 0 );
   if ( attempt > 1 )
   {
      gpio_set_pin_val( HW_CFG_GPIO_BAD_READ, 1 );

      vc_dump_mem( "Good Data", vc_interface_base + offset, &vc_sharedmem_header.vin_int_req, sizeof(VC_SHAREDMEM_HEADER_T) - offset );
   }
#endif

   if ( attempt > 1 )
   {
      printk( KERN_ERR "*****\n" );
      printk( KERN_ERR "***** read_int_bits took %d attempts\n", attempt );
      printk( KERN_ERR "*****\n" );

      // Dump shared memory data structs.
      #if defined( __KERNEL__ )
      {
         show_vc02();
      }
      #endif


      // TODO: remove chip specific code.
#if defined( CONFIG_ARCH_BCM116X )
   #if defined( __KERNEL__ )
      printk( KERN_ERR "***** NAND_TCR = 0x%08x, VC02_TCR = 0x%08x\n", REG_UMI_NAND_TCR, REG_UMI_VC02_TCR );
   #else
      printk( KERN_ERR "***** NAND_TCR = n/a from usermode, VC02_TCR = n/a from usermode\n" );
   #endif
#endif
   }

   if ( reportCount < 6 )
   {
      if ( vc_sharedmem_header.vin_int_ack >= 0x20 )
      {
         printk( KERN_ERR "*****\n" );
         printk( KERN_ERR "***** vin_int_ack: 0x%02x (after read)\n", vc_sharedmem_header.vin_int_ack );
         printk( KERN_ERR "*****\n" );

         // Dump shared memory data structs.
         #if defined( __KERNEL__ )
         {
            show_vc02();
         }
         #endif
      }
   }

   VC_DEBUG( MsgFifo, "vin_int_req:  0x%02x\n", vc_sharedmem_header.vin_int_req );
   VC_DEBUG( MsgFifo, "vout_int_ack: 0x%02x\n", vc_sharedmem_header.vout_int_ack );
   VC_DEBUG( MsgFifo, "vin_int_ack:  0x%02x\n", vc_sharedmem_header.vin_int_ack );
   VC_DEBUG( MsgFifo, "vout_int_req: 0x%02x\n", vc_sharedmem_header.vout_int_req );

   return status;
}

int write_int_bits (void) {
   // Write from vin_int_req to vin_int_ack.
   int status;
   int offset1 = (uint32_t)&vc_sharedmem_header.vin_int_req - (uint32_t)&vc_sharedmem_header;
   int offset2 = (uint32_t)&vc_sharedmem_header.vin_int_ack - (uint32_t)&vc_sharedmem_header.vin_int_req;

   VC_DEBUG( MsgFifo, "vin_int_req:  0x%02x\n", vc_sharedmem_header.vin_int_req );
   VC_DEBUG( MsgFifo, "vout_int_ack: 0x%02x\n", vc_sharedmem_header.vout_int_ack );

#if defined( __KERNEL__ )
   if (( memcmp( vc_sharedmem_header.dummy1, testPattern, sizeof( testPattern )) != 0 )
   ||  ( memcmp( vc_sharedmem_header.dummy2, testPattern, sizeof( testPattern )) != 0 ))
   {
      printk( KERN_ERR "write_int_bits - Host restoring test pattern data\n" );

      memcpy( vc_sharedmem_header.dummy1, testPattern, sizeof( testPattern ));
      memcpy( vc_sharedmem_header.dummy2, testPattern, sizeof( testPattern ));
   }
#endif

   status = vc_host_write_consecutive(vc_interface_base + offset1, &vc_sharedmem_header.vin_int_req, offset2, 0);
   vc_assert(status == 0);
   return status;
}
