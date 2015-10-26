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




/*=============================================================================

Project  :  VCHI
Module   :  MPHI videocore message driver

FILE DESCRIPTION:

The vcfw mphi driver provides a set of low-level routines for talking
to the hardware.

Of particular note is how message receiving is handled: the vcfw
driver needs to be passed a set of 'slots' into which incoming
messages are placed.  Callbacks are 'slot has completed' and 'position
within slot has advanced' (this latter is message-end-interrupt driven).
This module is responsible for translating these into a set of 'message
received' callbacks to the next layer up.

For transmitting messages, the abstraction is easier: again, we pass
'slots' to the vcfw driver.
=============================================================================*/

#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/mphi.h>
#include <linux/broadcom/vc03/vchi/vchi_cfg.h>
#include <linux/broadcom/vc03/vchi/message.h>
#include <linux/broadcom/vc03/vchi/non_wrap_fifo.h>
#include <linux/broadcom/vc03/vchi/control_service.h>
#include <linux/broadcom/vc03/vchi/multiqueue.h>
#include <linux/broadcom/vc03/vchi/endian.h>



/******************************************************************************
  Local typedefs
 *****************************************************************************/

// define NULL-terminated list of messages we're interested in dumping
//#define DUMP_MESSAGES { "GCMD", "ILCS", "DISP", "KHRN", "HREQ", "FSRV", NULL }
//#define DUMP_MESSAGES { "KHRN", NULL }

// Mapping of public channel indexes to hardware
#define MESSAGE_TX_CHANNEL_MPHI_CONTROL MESSAGE_TX_CHANNEL_MESSAGE
#define MESSAGE_TX_CHANNEL_MPHI_DATA    MESSAGE_TX_CHANNEL_BULK
#define MESSAGE_TX_CHANNEL_CCP2(n)      (MESSAGE_TX_CHANNEL_BULK+1+(n))
#define IS_MPHI_TX_CHANNEL(n)           ((n) == MESSAGE_TX_CHANNEL_MPHI_CONTROL || (n) == MESSAGE_TX_CHANNEL_MPHI_DATA)
#define IS_CCP2_TX_CHANNEL(n)           ((n) >= MESSAGE_TX_CHANNEL_CCP2(0) && (n) <= MESSAGE_TX_CHANNEL_CCP2(7))

#define MESSAGE_RX_CHANNEL_MPHI_CONTROL MESSAGE_RX_CHANNEL_MESSAGE
#define MESSAGE_RX_CHANNEL_MPHI_DATA    MESSAGE_RX_CHANNEL_BULK

// how many times can we call ->open
#define MPHI_MAX_INSTANCES 3

// maximum number of messages we can queue up
#define NUMBER_OF_MESSAGES 128

// parameters of the protocol
#define VCHI_MSG_SYNC      0xAF05
#define VCHI_LEN_ALIGN         16

// offsets for the message format
#define SYNC_OFFSET             0
#define LENGTH_OFFSET           2
#define FOUR_CC_OFFSET          4
#define TIMESTAMP_OFFSET        8
#define SLOT_COUNT_OFFSET      12
#define PAYLOAD_LENGTH_OFFSET  14
#define PAYLOAD_ADDRESS_OFFSET 16

typedef struct {
   int   inuse;
   MESSAGE_TX_CHANNEL_T channel;
   void *handle;
   void *addr;
   uint32_t len;
} FIFO_ENTRY_T;
typedef struct {
   int sync;
   int addr_align;
   int len_align;
   int rx_ctrl_slots;
   int rx_data_slots;
   int tx_slots;
} VCHI_MESSAGE_DRIVER_INFO_T;


typedef struct {

   // index into array of state (handles)
   int instance;


   // handles to mphi driver
   const MPHI_DRIVER_T *mphi_driver;
   DRIVER_HANDLE_T      handle_control;   // control rx
   DRIVER_HANDLE_T      handle_data;      // bulk rx
   DRIVER_HANDLE_T      handle_out;       // control and bulk tx

   // callback to the next higher level
   void (*event_callback)( void *cb_data );
   void *event_callback_data;

   // There are 3 identical 16 deep DMA FIFOs in the MPHI peripheral;
   // we need to keep track of what is in them so that we can pass back
   // the correct information when each finishes.
   RX_MSG_SLOTINFO_T  *rx_msg_slot [ MPHI_RX_MSG_SLOTS  ];
   RX_BULK_SLOTINFO_T *rx_bulk_slot[ MPHI_RX_BULK_SLOTS ];
   FIFO_ENTRY_T        tx_mphi_slot[ MPHI_TX_SLOTS      ];

   // To confirm that things are happening in order we will use an 8-bit
   // id which is returned upon completion (except for the TX side where
   // we just see an interrupt)
   uint8_t rx_msg_slot_id;
   uint8_t rx_bulk_id;
   uint8_t tx_mphi_id;

   // parameters of the driver
   int    rx_addr_align;
   int    tx_addr_align;
   bool_t tx_supports_terminate;

   // index into tx_ccp2_slot, empty when head == tail
   uint8_t tx_ccp2_id_head;
   uint8_t tx_ccp2_id_tail;

   OS_HISR_T hisr;

} MPHI_MSG_DRIVER_STATE_T;


/******************************************************************************
 Extern functions
 *****************************************************************************/


/******************************************************************************
 Static functions
 *****************************************************************************/

// API functions
static VCHI_MDRIVER_HANDLE_T *mphi_msg_open( VCHI_MESSAGE_DRIVER_OPEN_T *params, void *state );
static int32_t mphi_msg_add_msg_slot( VCHI_MDRIVER_HANDLE_T *handle, RX_MSG_SLOTINFO_T *slot );
static int32_t mphi_msg_add_bulk_rx( VCHI_MDRIVER_HANDLE_T *handle, void *data, uint32_t len, RX_BULK_SLOTINFO_T *slot );
static int32_t mphi_msg_send_message( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel, const void *data, uint32_t len, VCHI_MSG_FLAGS_T flags, void *send_handle );
static void    mphi_msg_next_event( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_EVENT_T *slot );
static int32_t mphi_msg_enable( VCHI_MDRIVER_HANDLE_T *handle );
static int32_t mphi_msg_form_message( VCHI_MDRIVER_HANDLE_T *handle, fourcc_t service_id, VCHI_MSG_VECTOR_T *vector, uint32_t count, void *address, uint32_t available_length, uint32_t max_total_length, bool_t pad_to_fill, bool_t allow_partial );

static int32_t mphi_msg_update_message( VCHI_MDRIVER_HANDLE_T *handle, void *dest, int16_t *slot_count );
static int32_t mphi_msg_buffer_aligned( VCHI_MDRIVER_HANDLE_T *handle, int tx, const void *address, const uint32_t length );
static void *  mphi_msg_allocate_buffer( VCHI_MDRIVER_HANDLE_T *handle, uint32_t *length );
static void    mphi_msg_free_buffer( VCHI_MDRIVER_HANDLE_T *handle, void *address );
static int     mphi_msg_slot_size( VCHI_MDRIVER_HANDLE_T *handle, int msg_size );

static bool_t  mphi_msg_tx_supports_terminate( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel );
static bool_t  mphi_msg_tx_alignment( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel );
static bool_t  mphi_msg_rx_alignment( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_RX_CHANNEL_T channel );
static void    mphi_msg_form_bulk_aux( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel, const void *data, uint32_t len, uint32_t chunk_size, const void **aux_data, int32_t *aux_len );

// helper functions
static void    mphi_msg_callback( const void *callback_data );
static void    mphi_hisr0( void );
static void    mphi_hisr1( void );
static void    mphi_hisr2( void );
static int32_t mphi_parse( RX_MSG_SLOTINFO_T *slot, MESSAGE_EVENT_T *event );
static void    mphi_msg_debug( VCHI_MDRIVER_HANDLE_T *handle );

#if defined(DUMP_MESSAGES)
static void dump_message( const char *reason, const unsigned char *addr, int len );
#else
#define dump_message(a,b,c) do { ; } while(0)
#endif


/******************************************************************************
 Static data
 *****************************************************************************/

//The driver function table
static const VCHI_MESSAGE_DRIVER_T mphi_msg_fops =
{
   mphi_msg_open,
   mphi_msg_add_msg_slot,
   mphi_msg_add_bulk_rx,
   mphi_msg_send_message,
   mphi_msg_next_event,
   mphi_msg_enable,
   mphi_msg_form_message,

   mphi_msg_update_message,
   mphi_msg_buffer_aligned,
   mphi_msg_allocate_buffer,
   mphi_msg_free_buffer,
   mphi_msg_slot_size,

   mphi_msg_tx_supports_terminate,
   mphi_msg_tx_alignment,
   mphi_msg_rx_alignment,
   mphi_msg_form_bulk_aux,

   mphi_msg_debug
};

static const VCHI_MESSAGE_DRIVER_INFO_T mphi_msg_info =
{
   0xaf05,             // sync preamble
   16,                 // address alignment                      (required by hardware)
   16,                 // length alignment                       (required by hardware)
   MPHI_RX_MSG_SLOTS,  // number of control rx slots             (hardware limit)
   MPHI_RX_BULK_SLOTS, // number of data (=bulk) rx slots        (hardware limit)
   MPHI_TX_SLOTS       // number of TX (data and control) slots  (hardware limit)
};

static MPHI_MSG_DRIVER_STATE_T mphi_msg_state[ MPHI_MAX_INSTANCES ];
static int mphi_num_instances = 0;

// vcfw driver open params
//static const MPHI_OPEN_T mphi_open_control = { MPHI_CHANNEL_IN_CONTROL, mphi_msg_callback };
//static const MPHI_OPEN_T mphi_open_data    = { MPHI_CHANNEL_IN_DATA,    mphi_msg_callback };
//static const MPHI_OPEN_T mphi_open_out     = { MPHI_CHANNEL_OUT,        mphi_msg_callback };


/******************************************************************************
 Global Functions
 *****************************************************************************/

/* ----------------------------------------------------------------------
 * return pointer to the mphi message driver function table
 * -------------------------------------------------------------------- */
const VCHI_MESSAGE_DRIVER_T *
vchi_mphi_message_driver_func_table( void )
{
   return &mphi_msg_fops;
}


/******************************************************************************
 Static  Functions
 *****************************************************************************/

/* ----------------------------------------------------------------------
 * open the vcfw mphi driver (several times; rx control, rx data and tx)
 *
 * can be called multiple times (eg. local host apps)
 *
 * returns handle to this instance on success; NULL on failure
 * -------------------------------------------------------------------- */
static VCHI_MDRIVER_HANDLE_T *
mphi_msg_open( VCHI_MESSAGE_DRIVER_OPEN_T *params, void *callback_data )
{
   int32_t success = -1; // fail by default
   MPHI_MSG_DRIVER_STATE_T *state;
   MPHI_OPEN_T open_params;
   const char *driver_name;
   uint32_t driver_version_major, driver_version_minor;
   DRIVER_FLAGS_T driver_flags;
   //static int32_t mphi_num_instances;

   // get new state handle
   if ( mphi_num_instances >= MPHI_MAX_INSTANCES ) 
   	return NULL;
   //printk(KERN_ERR "mphi_msg_open %d\n",mphi_num_instances); //zrl

   // could get called from more than one thread
   success = os_semaphore_obtain_global();
   os_assert(success == 0);

   state = &mphi_msg_state[ mphi_num_instances ];
   memset( state, 0, sizeof(MPHI_MSG_DRIVER_STATE_T) );
   state->instance = mphi_num_instances++;

   success = os_semaphore_release_global();
   os_assert(success == 0);

   // record the callbacks
   state->event_callback = params->event_callback;
   state->event_callback_data = callback_data;

   // open (vcfw) mphi driver and get handle
#ifdef VCHI_LOCAL_HOST_PORT
{
   extern const MPHI_DRIVER_T *local_mphi_get_func_table( void );
   state->mphi_driver = local_mphi_get_func_table();
}
#else
   state->mphi_driver = mphi_get_func_table();
#endif

   success = state->mphi_driver->info(&driver_name,
                                      &driver_version_major,
                                      &driver_version_minor,
                                      &driver_flags);
   if ( success != 0) return NULL;

   // The master transmitter supports terminate. A slave transmitter does not, but signals length up front.
   state->tx_supports_terminate = strstr(driver_name, "_slave") == 0;

   open_params.channel       = MPHI_CHANNEL_IN_CONTROL;
   open_params.callback      = mphi_msg_callback;
   open_params.callback_data = state;

   // open the driver for the control channel
   success = state->mphi_driver->open( &open_params, &state->handle_control );
   if ( success != 0 ) return NULL;

   // open the driver for the data (bulk) channel
   open_params.channel = MPHI_CHANNEL_IN_DATA;
   success = state->mphi_driver->open( &open_params, &state->handle_data );
   if ( success != 0 ) return NULL;

   // open the driver for the output channel
   open_params.channel = MPHI_CHANNEL_OUT;
   success = state->mphi_driver->open( &open_params, &state->handle_out );
   if ( success != 0 ) return NULL;

   // set current (12mA) and slew (off in this case)
   success = state->mphi_driver->set_power( state->handle_control, 12, 0 );
   if ( success != 0 ) return NULL;

   // get the alignment requirements
   state->rx_addr_align = OS_MAX( state->mphi_driver->alignment( state->handle_control ),
                                  state->mphi_driver->alignment( state->handle_data ) );
   state->tx_addr_align = state->mphi_driver->alignment( state->handle_out );

   // create the HISR that is used to signal the next higher level that an event has occured
   if ( state->instance == 0 )
      success = os_hisr_create( &state->hisr, mphi_hisr0, "MPHI HISR0" );
   else if ( state->instance == 1 )
      success = os_hisr_create( &state->hisr, mphi_hisr1, "MPHI HISR1" );
   else if ( state->instance == 2 )
      success = os_hisr_create( &state->hisr, mphi_hisr2, "MPHI HISR2" );
   else
      success = -1;
   if ( success != 0 ) return NULL;

   // various parts of this message driver will fail if this condition isn't met -
   // will need more care on message formation.
   os_assert( state->tx_addr_align <= VCHI_LEN_ALIGN );

   return (VCHI_MDRIVER_HANDLE_T *)state;
}

/* ----------------------------------------------------------------------
 * add a slot to the rx message (= control) fifo
 *
 * return 0 if the given slot was successfully added, non-0 otherwise
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_add_msg_slot( VCHI_MDRIVER_HANDLE_T *handle, RX_MSG_SLOTINFO_T *slot )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   uint8_t id = state->rx_msg_slot_id;
   int32_t success = -1; // fail by default

   if ( state->rx_msg_slot[id] != NULL ) {
      //os_logging_message("mphi_msg_add_msg_slot: no free slots");
      return -1; // no free slots
   }
   os_assert( slot->active == 0 );
   // we need to setup, in advance, for vcfw success case (avoid lisr race)
   state->rx_msg_slot[ id ] = slot;
   slot->active = 1;
   slot->read_ptr = 0;
   success = state->mphi_driver->add_recv_slot( state->handle_control, slot->addr, slot->len, id );
   if ( success != 0 ) {
      // vcfw call didn't succeed; undo setup
      slot->active = 0;
      state->rx_msg_slot[ id ] = NULL;
      //os_logging_message( "mphi_msg_add_msg_slot failed" );
   } else {
      state->rx_msg_slot_id = (id + 1) % MPHI_RX_MSG_SLOTS;
      //os_logging_message( "mphi_msg_add_msg_slot: id = %d", id );
   }
   return success;
}

/* ----------------------------------------------------------------------
 * add a slot to the rx bulk (= data) fifo
 *
 * return 0 if the given slot was successfully added, non-0 otherwise
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_add_bulk_rx( VCHI_MDRIVER_HANDLE_T *handle, void *addr, uint32_t len, RX_BULK_SLOTINFO_T *slot )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   uint8_t id = state->rx_bulk_id;
   uint32_t success = -1; // fail by default

   if ( state->rx_bulk_slot[id] != NULL )
      return -1; // no free slots
   // we need to setup, in advance, for vcfw success case (avoid lisr race)
   state->rx_bulk_slot[ id ] = slot;
   success = state->mphi_driver->add_recv_slot( state->handle_data, addr, len, id );
   if ( success != 0 ) {
      // vcfw call didn't succeed; undo setup
      state->rx_bulk_slot[ id ] = NULL;
      os_logging_message( "mphi_msg_add_msg_slot (bulk) failed" );
   } else {
      state->rx_bulk_id = (id + 1) % MPHI_RX_BULK_SLOTS;
      //os_logging_message( "mphi_msg_add_bulk_rx: id = %d", id );
   }
   return success;
}

/* ----------------------------------------------------------------------
 * add a slot to the tx fifo
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_send_message( VCHI_MDRIVER_HANDLE_T * const handle,
                       const MESSAGE_TX_CHANNEL_T channel,
                       const void * const addr,
                       const uint32_t len,
                       const VCHI_MSG_FLAGS_T flags,
                       void * const send_handle )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   uint8_t id;
   int32_t success = -1; // fail by default

   //os_logging_message("mphi_msg_send_message %x:%d", addr, len);

   if( IS_MPHI_TX_CHANNEL(channel) )
   {
      id = state->tx_mphi_id;

      if ( state->tx_mphi_slot[id].inuse ) {
      // no room for message
   //      os_assert(0);
         return -1;
      }

      // we need to setup, in advance, for vcfw success case (avoid lisr race)
      state->tx_mphi_slot[ id ].inuse = 1;
      state->tx_mphi_slot[ id ].handle = send_handle;
      state->tx_mphi_slot[ id ].channel = channel;
      state->tx_mphi_slot[ id ].addr = (void *)addr;
      state->tx_mphi_slot[ id ].len = len;

      //dump_message( "mphi_msg_send_message", addr, len );
      success = state->mphi_driver->out_queue_message( state->handle_out,
                                                       channel == MESSAGE_TX_CHANNEL_MPHI_CONTROL,
                                                       addr,
                                                       len,
                                                       id,
                                                       flags & VCHI_MSG_FLAGS_TERMINATE_DMA ?
                                                                 MPHI_FLAGS_TERMINATE_DMA :
                                                                 MPHI_FLAGS_NONE );
      if ( success != 0 ) {
         // vcfw call didn't succeed; undo setup
         state->tx_mphi_slot[ id ].inuse = 0;
         os_logging_message( "mphi_msg_send_message: mphi tx failed (%d), id = %d", success, id );
//         os_assert(0);
      } else {
         state->tx_mphi_id = (id + 1) % MPHI_TX_SLOTS;
         //os_logging_message( "mphi_msg_send_message: id = %d", id );
      }
   }

#ifdef LOGGING_VCHI
   dump_message( "mphi_msg_send_message", slot->addr, slot->len );
#endif // LOGGING_VCHI

   else
      os_assert(0);

   return success;
}

/* ----------------------------------------------------------------------
 * at this point, we get a callback from the lisr to say there are
 * some events (at the vcfw level) which need handling.
 *
 * the processing chain is to wake up our local hisr...
 * -------------------------------------------------------------------- */
static void
mphi_msg_callback( const void *callback_data )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)callback_data;
   os_hisr_activate( &state->hisr );
}

/* ----------------------------------------------------------------------
 * ...which simply does a callback (in hisr context) to the next level
 * up (= vchi connection driver)...
 * -------------------------------------------------------------------- */
static void
mphi_hisr0( void )
{
   MPHI_MSG_DRIVER_STATE_T *state = &mphi_msg_state[0];
   state->event_callback( state->event_callback_data );
}

static void
mphi_hisr1( void )
{
   MPHI_MSG_DRIVER_STATE_T *state = &mphi_msg_state[1];
   state->event_callback( state->event_callback_data );
}

static void
mphi_hisr2( void )
{
   MPHI_MSG_DRIVER_STATE_T *state = &mphi_msg_state[2];
   state->event_callback( state->event_callback_data );
}

/* ----------------------------------------------------------------------
 * ...at which point, the connection driver will call into us, in
 * task context, to find out exactly which event(s) happened.
 *
 * the idea is that the task should spin in a loop, calling this
 * function until it returns MESSAGE_EVENT_NONE.  closely coupled
 * to the vcfw driver ->next_event method, see that for low-level
 * details.
 *
 * return values is set in event->type as follows:
 *
 *    MESSAGE_EVENT_NONE - no more events pending.
 *
 *    MESSAGE_EVENT_NOP - no event on this call, because only part
 *       of a message was received.  call again.
 *
 *    MESSAGE_EVENT_MESSAGE - a message was received.  information
 *       about the payload (address, len, service 4cc, slot) will
 *       be stored in event->message.*
 *
 *    MESSAGE_EVENT_SLOT_COMPLETE - a message receive slot completed.
 *       pointer to the slot stored in event->rx_msg.  note that we
 *       guarantees we will return MESSAGE_EVENT_MESSAGE for all
 *       messages in a completed slot before returning
 *       MESSAGE_EVENT_SLOT_COMPLETE
 *
 *    MESSAGE_EVENT_RX_BULK_PAUSED - FIXME: do we even need this?
 *
 *    MESSAGE_EVENT_RX_BULK_COMPLETE - a bulk receive slot completed.
 *       pointer to the slot stored in event->rx_bulk
 *
 *    MESSAGE_EVENT_TX_COMPLETE - a transmit (message) slot
 *       completed.  pointer to the slot stored in event->tx
 *
 *    MESSAGE_EVENT_TX_BULK_COMPLETE - a transmit bulk slot
 *       completed.  pointer to the slot stored in event->tx
 *
 *    MESSAGE_EVENT_MSG_DISCARDED - FIXME: do we need this?
 *
 * -------------------------------------------------------------------- */
static void
mphi_msg_next_event( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_EVENT_T *event )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   uint8_t slot_id;
   uint32_t pos;
   MPHI_EVENT_TYPE_T reason;
   static RX_MSG_SLOTINFO_T *slot = NULL;
   static int slot_completed = 0;

   event->type = MESSAGE_EVENT_NONE;

   // if we are busy parsing a slot, keep returning more messages
   // until we run out of data
   if ( slot ) {
      if ( mphi_parse(slot,event) )
         return;
      if ( slot_completed ) {
         slot_completed = 0;
         event->type = MESSAGE_EVENT_SLOT_COMPLETE;
         event->rx_msg = slot;
         slot = NULL;
         return;
      }
      slot = NULL;
   }

   // fetch the next event from vcfw driver
   // should get cleverer, and only check on channels that have signalled. Note
   // that original code, which checked events only on handle_control, relied on
   // a bogus implementation of next_event(). It's been broken from the start,
   // returning any event on any channel. If the channels mean anything, they have
   // to return independent event streams. local.c now has independent channels,
   // mphi.c still has coupled channels. For now, this works with either.
   reason = state->mphi_driver->next_event( state->handle_control, &slot_id, &pos );
   if (reason == MPHI_EVENT_NONE)
      reason = state->mphi_driver->next_event( state->handle_out, &slot_id, &pos );
   if (reason == MPHI_EVENT_NONE)
      reason = state->mphi_driver->next_event( state->handle_data, &slot_id, &pos );

   switch( reason ) {

   default:
      os_assert(0);
   case MPHI_EVENT_NONE:
      //assert(0); _nop();
      event->type = MESSAGE_EVENT_NONE;
      return; // finished

   case MPHI_EVENT_IN_CONTROL_POSITION:
      // the given slot_id has received some data
      //os_logging_message("IN_POS: %d:%d", slot_id, pos);
      os_assert( slot_id < MPHI_RX_MSG_SLOTS );
      slot = state->rx_msg_slot[ slot_id ];
      slot->write_ptr = pos;

      if ( mphi_parse(slot,event) )
         return; // we will loop looking for more messages

      slot = NULL; // stop parsing
      event->type = MESSAGE_EVENT_NOP;
      return;

   case MPHI_EVENT_IN_CONTROL_DMA_COMPLETED:
      // the given slot_id has completed
      //os_logging_message("IN_COMP: %d:%d", slot_id, pos);
      os_assert( slot_id < MPHI_RX_MSG_SLOTS );
      slot = state->rx_msg_slot[ slot_id ];
      slot->write_ptr = pos;
      state->rx_msg_slot[ slot_id ] = NULL;

      slot_completed = 1;
      if ( mphi_parse(slot,event) )
         return; // we will loop looking for more messages

      slot_completed = 0;
      event->type = MESSAGE_EVENT_SLOT_COMPLETE;
      event->rx_msg = slot;
      slot = NULL; // stop parsing
      return;

   case MPHI_EVENT_IN_DATA_POSITION:
      // nothing required at the moment
      // must likely to happen during long transfers when the host
      // needs to check the VC->Host direction
      event->type = MESSAGE_EVENT_RX_BULK_PAUSED;
      return;

   case MPHI_EVENT_IN_DATA_DMA_COMPLETED:
      // a transmission from the host has completed
      os_assert( slot_id < MPHI_RX_BULK_SLOTS );
      event->rx_bulk = state->rx_bulk_slot[ slot_id ];
      state->rx_bulk_slot[ slot_id ] = NULL;
      event->type = MESSAGE_EVENT_RX_BULK_COMPLETE;
      return;

   case MPHI_EVENT_OUT_DMA_COMPLETED:
      // a transmission from VideoCore has completed
      //os_logging_message( "mphi_hisr : dma out complete slot_id = %d", slot_id );
      event->tx_handle = state->tx_mphi_slot[ slot_id ].handle;
      event->tx_channel = state->tx_mphi_slot[ slot_id ].channel;
      event->message.addr = state->tx_mphi_slot[ slot_id ].addr;
      event->message.len = state->tx_mphi_slot[ slot_id ].len;
      state->tx_mphi_slot[ slot_id ].inuse = 0;
      event->type = MESSAGE_EVENT_TX_COMPLETE;
      return;

   case MPHI_EVENT_IN_MESSAGE_DISCARDED:
      event->type = MESSAGE_EVENT_MSG_DISCARDED;
      return;

   }
}

static int32_t
mphi_parse( RX_MSG_SLOTINFO_T *slot, MESSAGE_EVENT_T *event )
{
   int32_t available;
   uint8_t *addr;
   int32_t len;

   // trap badness from the hardware
   if ( slot->write_ptr > slot->len ) {
      os_assert(0);
      return 0; // ignore any further messages in this slot
   }

   available = slot->write_ptr - slot->read_ptr;
   os_assert( available >= 0 ); // this would be bad

   // we require to see at least the header:
   if ( available < PAYLOAD_ADDRESS_OFFSET )
      return 0;

   addr = slot->addr + slot->read_ptr;
   os_assert( vchi_readbuf_uint16(addr) == VCHI_MSG_SYNC );
   len = vchi_readbuf_uint16( addr + 2 );
   //uint32_t len = vchi_readbuf_uint16( addr + PAYLOAD_LENGTH_OFFSET );
   //len += 31; len &= ~15;

   // confirm that we have seen the whole message
   if ( available < len )
      return 0;

   // verify that the values we have read are valid
   if ( len < PAYLOAD_ADDRESS_OFFSET || slot->read_ptr + len > slot->len ) {
      os_assert(0); // must have valid 'length' and 'service id' fields
      return 0; // ignore any further messages in this slot
   }

#ifdef LOGGING_VCHI
   dump_message( "mphi_parse", addr, len );
#endif // LOGGING_VCHI

   os_semaphore_obtain( &slot->sem );
   // also need to record how many messages there are in the slot so we know when it comes free
   slot->messages++; // FIXME: put this into next layer?
   os_semaphore_release( &slot->sem );

   // synchronise clocks with the other side
   vchi_control_update_time( vchi_readbuf_uint32(addr + TIMESTAMP_OFFSET) );

   // fill in ->event fields
   event->type = MESSAGE_EVENT_MESSAGE;
   event->message.addr        = addr + PAYLOAD_ADDRESS_OFFSET;
   event->message.slot_delta  = vchi_readbuf_uint16( addr + SLOT_COUNT_OFFSET );
   event->message.len         = vchi_readbuf_uint16( addr + PAYLOAD_LENGTH_OFFSET );
   event->message.slot        = slot;
   event->message.service     = vchi_readbuf_fourcc( addr + FOUR_CC_OFFSET );
   event->message.tx_timestamp= vchi_readbuf_uint32( addr + TIMESTAMP_OFFSET );
   event->message.rx_timestamp= vchi_control_get_time();

   {
       fourcc_t service_id = event->message.service;

       VC_DEBUG( MphiTrace, "RcvdMsg: %c%c%c%c len: %d, slot_delta:%d pay len:%d, slot# %d rPtr:%d wPtr:%d", 
                 ( service_id >> 24 ) & 0xff,
                 ( service_id >> 16 ) & 0xff,
                 ( service_id >>  8 ) & 0xff,
                 ( service_id >>  0 ) & 0xff,
                 len,
                 event->message.slot_delta,
                 event->message.len,
                 slot->slot_no, slot->read_ptr, slot->write_ptr );
   }

   // ready to parse any further messages in this slot
   slot->read_ptr += len;

   return 1; // we found a message!
}

/* ----------------------------------------------------------------------
 * the routine is called during initialisation, after the connection
 * driver has queued all rx slots.
 *
 * we simply need to call the vcfw ->enable method
 *
 * return 0 on success, non-0 otherwise
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_enable( VCHI_MDRIVER_HANDLE_T *handle )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   return state->mphi_driver->enable( state->handle_control );
}

/* ----------------------------------------------------------------------
 * Calculate the total length of vector v[n].
 * -------------------------------------------------------------------- */
static size_t vec_length(const VCHI_MSG_VECTOR_T *v, int n)
{
   size_t length=0;
   for ( ; n; v++, n-- )
      if (v->vec_len < 0)
          length += vec_length(v->vec_base, -v->vec_len);
      else
          length += v->vec_len;

   return length;
}

/* ----------------------------------------------------------------------
 * Copy vector v[n] to dest. If limit is non-NULL, it will stop at limit,
 * and will update v to indicate consumed data.
 * -------------------------------------------------------------------- */
static void *vec_copy(void *dest, VCHI_MSG_VECTOR_T *v, int n, const void *limit)
{
   uint8_t *p = dest;
   const uint8_t *e = limit;
   for ( ; n && p != e; v++, n--)
   {
      if (v->vec_len < 0)
         p = vec_copy(p, (VCHI_MSG_VECTOR_T *) v->vec_base, -v->vec_len, e);
      else if (v->vec_len > 0)
      {
         size_t len = (e == NULL || e - p > v->vec_len) ? v->vec_len : e - p;
         memcpy(p, v->vec_base, len);
         p += len;
         if (e)
{
            v->vec_base = (uint8_t*)v->vec_base + len;
            v->vec_len -= len;
         }
}
}

   return p;
}
/* ----------------------------------------------------------------------
 * copy the message to the local memory with the correct formating,
 * padding etc.
 *
 * return -1 on error (message is too long for a slot);
 *         0 if fifo is full (ie. caller needs to wait for space to
 *                            become free)
 *        >0 returns the number of bytes written
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_form_message( VCHI_MDRIVER_HANDLE_T *handle,
                       fourcc_t service_id,
                       VCHI_MSG_VECTOR_T *vector,
                       uint32_t count,
                       void *address,
                       uint32_t available_length,
                       uint32_t max_total_length,
                       bool_t pad_to_fill,
                       bool_t allow_partial )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   int32_t success = -1; // fail by default
   int padding;
   uint8_t *ptr;

   // calculate length of all vectors
   uint16_t msg_length, len;
   msg_length = vec_length(vector, count);
   len = msg_length;

   // add space for payload and alignment (if required)
   len += PAYLOAD_ADDRESS_OFFSET;
   padding = len % VCHI_LEN_ALIGN;
   if ( padding )
      padding = VCHI_LEN_ALIGN - padding;
   len += padding;

   if ( allow_partial && len > available_length ) {
      os_assert(available_length % VCHI_LEN_ALIGN == 0);
      len = available_length;
      msg_length = len - PAYLOAD_ADDRESS_OFFSET;
      padding = 0;
   }

   // it would be bad if this didn't fit in a slot
   if ( len > max_total_length )
      return -1;

   // not enough room in this slot
   if ( len > available_length )
      return 0; // calling function needs to find another slot

   if ( pad_to_fill ) {
      padding += available_length - len;
      len = available_length;
   }

   ptr = address;
   os_assert( (((size_t)ptr) % state->tx_addr_align) == 0 );

   VC_DEBUG( MphiTrace, "SendMsg: %c%c%c%c len: %d pay len: %d", 
             ( service_id >> 24 ) & 0xff,
             ( service_id >> 16 ) & 0xff,
             ( service_id >>  8 ) & 0xff,
             ( service_id >>  0 ) & 0xff,
             len, msg_length );

   vchi_writebuf_uint16( ptr + SYNC_OFFSET,    VCHI_MSG_SYNC );
   vchi_writebuf_uint16( ptr + LENGTH_OFFSET,  len );
   vchi_writebuf_fourcc( ptr + FOUR_CC_OFFSET, service_id );
   vchi_writebuf_uint16( ptr + PAYLOAD_LENGTH_OFFSET, msg_length );
   // write the message (if present)
   ptr += PAYLOAD_ADDRESS_OFFSET;
   ptr = vec_copy( ptr, vector, count, allow_partial ? ptr + msg_length : NULL );
   // memset to 0 any padding required
   if ( padding )
      memset( ptr, 0, padding );

   return len;
}

/* ----------------------------------------------------------------------
 * Add slot info and timestamp to a message
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_update_message( VCHI_MDRIVER_HANDLE_T *handle, void *dest, int16_t *slot_count )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   uint8_t *ptr = (uint8_t *)dest;
   uint32_t timestamp = vchi_control_get_time();
   uint16_t delta;
   
   // Negative deltas can exist transiently - cope with this
   if ( *slot_count > 0 )
   {
      delta = *slot_count;
      *slot_count = 0;
   }
   else
      delta = 0;

   vchi_writebuf_uint32( ptr + TIMESTAMP_OFFSET,  timestamp  );
   vchi_writebuf_uint16( ptr + SLOT_COUNT_OFFSET, delta );

   //os_logging_message( "Outgoing timestamp = %u", timestamp );

   return vchi_readbuf_uint16( ptr + LENGTH_OFFSET );
}

/* ----------------------------------------------------------------------
 * Return VC_TRUE if buffer is suitably aligned for this connection
 * -------------------------------------------------------------------- */
static int32_t
mphi_msg_buffer_aligned( VCHI_MDRIVER_HANDLE_T *handle, int tx, const void *address, const uint32_t length )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   int align = tx ? state->tx_addr_align : state->rx_addr_align;
   if ((size_t)address % align != 0 || length % VCHI_LEN_ALIGN != 0)
      return VC_FALSE;
   else
      return VC_TRUE;
}

/* ----------------------------------------------------------------------
 * Allocate memory with suitable alignment and length for this connection
 * -------------------------------------------------------------------- */
static void *
mphi_msg_allocate_buffer( VCHI_MDRIVER_HANDLE_T *handle, uint32_t *length )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   void *addr;
   int align = OS_MAX( state->tx_addr_align, state->rx_addr_align );
   *length = (*length + (VCHI_LEN_ALIGN-1)) & ~(VCHI_LEN_ALIGN-1);

   // check that the alignments are powers of 2
   os_assert(OS_COUNT(align) == 1);
   os_assert(OS_COUNT(VCHI_LEN_ALIGN) == 1);
   os_assert(align > 0);
   os_assert((*length % VCHI_LEN_ALIGN) == 0);

   addr = os_malloc( *length, align, "MPHI_BUFFER" );
   memset( addr, 0, *length );
   return addr;
}

/* ----------------------------------------------------------------------
 * Free previously allocated memory
 * -------------------------------------------------------------------- */
static void
mphi_msg_free_buffer( VCHI_MDRIVER_HANDLE_T *handle, void *address )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   os_free( address );
}

/* ----------------------------------------------------------------------
 * How much space does a given message size take up in a slot?
 * -------------------------------------------------------------------- */
static int
mphi_msg_slot_size( VCHI_MDRIVER_HANDLE_T *handle, int msg_size )
{
   return ((PAYLOAD_ADDRESS_OFFSET + msg_size) + (VCHI_LEN_ALIGN-1)) &~ (VCHI_LEN_ALIGN-1);
}

/* ----------------------------------------------------------------------
 * Returns VC_TRUE if our peer will automatically handle advancing to the
 * next slot, and we don't send "terminate DMA" messages.
 * Returns VC_FALSE if our peer won't automatically advance to the next
 * slot, so we have to send "terminate DMA" messages.
 * -------------------------------------------------------------------- */
static bool_t
mphi_msg_tx_supports_terminate( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;

   if (IS_MPHI_TX_CHANNEL(channel))
      return state->tx_supports_terminate;
   else // CCP2
      return VC_TRUE; // in fact we ONLY support terminate at the moment
   }

/* ----------------------------------------------------------------------
 * Returns alignment requirement for transmit buffers.
 * -------------------------------------------------------------------- */
static bool_t
mphi_msg_tx_alignment( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;

   return state->tx_addr_align;
}

/* ----------------------------------------------------------------------
 * Returns alignment requirement for receive buffers.
 * -------------------------------------------------------------------- */
static bool_t
mphi_msg_rx_alignment( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_RX_CHANNEL_T channel )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;

   return state->rx_addr_align;
 }



/* ----------------------------------------------------------------------
 * Return the message-driver-specific bulk auxiliary data for this message.
 * Message driver is responsible for allocation. The data will be copied
 * away before this call is made a second time, so a single static
 * buffer can be used.
 * -------------------------------------------------------------------- */
static void
mphi_msg_form_bulk_aux( VCHI_MDRIVER_HANDLE_T *handle,
                        MESSAGE_TX_CHANNEL_T channel,
                        const void *addr,
                        uint32_t len,
                        uint32_t chunk_size,
                        const void **aux_data,
                        int32_t *aux_len )
{
   *aux_data = NULL;
   *aux_len = 0;

}

static void
mphi_msg_debug( VCHI_MDRIVER_HANDLE_T *handle )
{
   MPHI_MSG_DRIVER_STATE_T *state = (MPHI_MSG_DRIVER_STATE_T *)handle;
   os_logging_message( "  mphi_msg_debug:" );
   state->mphi_driver->debug( state->handle_control );
}

#if defined(DUMP_MESSAGES)
static void
dump_message( const char *reason, const unsigned char *addr, int len )
{
   static char msg[256];
   int i, m = OS_MIN( 60, len - 16 );
   char *ptr = msg;

   // NULL-terminated list of services we're interested in
   const char *filter[] = DUMP_MESSAGES;
   for (i=0; filter[i]; i++)
      if ( memcmp(filter[i],addr+4,4) == 0 )
         break;
   if ( !filter[i] ) return;

   ptr += sprintf( ptr, "{%p:%d} %04x %04x (%c%c%c%c) %d (sc=%d len=%d)",
                   addr, len,
                   vchi_readbuf_uint16(addr),
                   vchi_readbuf_uint16(addr+2),
                   addr[4], addr[5], addr[6], addr[7],
                   vchi_readbuf_uint32(addr+8),
                   vchi_readbuf_uint16(addr+12),
                   vchi_readbuf_uint16(addr+14) );

   for (i=0; i<m; i++)
      ptr += sprintf( ptr, " %02x", addr[i + 16] );
   if ( len > m )
      sprintf( ptr, " ..." );
   os_logging_message( "%s: %s", reason, msg );
}
#endif

/********************************** End of file ******************************************/
