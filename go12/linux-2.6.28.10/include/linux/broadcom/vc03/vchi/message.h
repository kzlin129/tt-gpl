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
=============================================================================*/

#ifndef _VCHI_MESSAGE_H_
#define _VCHI_MESSAGE_H_

#include <linux/broadcom/vc03/vcos.h>
#include "vchi.h"

#define MAKE_FOURCC(x) ( (x[0] << 24) | (x[1] << 16) | (x[2] << 8) | x[3] )
#define FOURCC_TO_CHAR(x) (x >> 24) & 0xFF,(x >> 16) & 0xFF,(x >> 8) & 0xFF, x & 0xFF

/*
 * Define vector struct for scatter-gather (vector) operations
 * Vectors can be nested - if a vector element has negative length, then
 * the data pointer is treated as pointing to another vector array, with
 * '-vec_len' elements. Thus to append a header onto an existing vector,
 * you can do this:
 *
 * void foo(const VCHI_MSG_VECTOR_T *v, int n)
 * {
 *    VCHI_MSG_VECTOR_T nv[2];
 *    nv[0].vec_base = my_header;
 *    nv[0].vec_len = sizeof my_header;
 *    nv[1].vec_base = v;
 *    nv[1].vec_len = -n;
 *    ...
 *
 */
typedef struct vchi_msg_vector {
   const void *vec_base;
   int32_t vec_len;
} VCHI_MSG_VECTOR_T;


typedef enum message_event_type {
   MESSAGE_EVENT_NONE,
   MESSAGE_EVENT_NOP,
   MESSAGE_EVENT_MESSAGE,
   MESSAGE_EVENT_SLOT_COMPLETE,
   MESSAGE_EVENT_RX_BULK_PAUSED,
   MESSAGE_EVENT_RX_BULK_COMPLETE,
   MESSAGE_EVENT_TX_COMPLETE,
   MESSAGE_EVENT_MSG_DISCARDED
} MESSAGE_EVENT_TYPE_T;

typedef enum vchi_msg_flags
{
   VCHI_MSG_FLAGS_NONE                  = 0x0,
   VCHI_MSG_FLAGS_TERMINATE_DMA         = 0x1
} VCHI_MSG_FLAGS_T;

typedef enum message_tx_channel
{
   MESSAGE_TX_CHANNEL_MESSAGE           = 0,
   MESSAGE_TX_CHANNEL_BULK              = 1 // drivers may provide multiple bulk channels, from 1 upwards
} MESSAGE_TX_CHANNEL_T;

// Macros used for cycling through bulk channels
#define MESSAGE_TX_CHANNEL_BULK_PREV(c) (MESSAGE_TX_CHANNEL_BULK+((c)-MESSAGE_TX_CHANNEL_BULK+VCHI_MAX_BULK_TX_CHANNELS_PER_CONNECTION-1)%VCHI_MAX_BULK_TX_CHANNELS_PER_CONNECTION)
#define MESSAGE_TX_CHANNEL_BULK_NEXT(c) (MESSAGE_TX_CHANNEL_BULK+((c)-MESSAGE_TX_CHANNEL_BULK+1)%VCHI_MAX_BULK_TX_CHANNELS_PER_CONNECTION)

typedef enum message_rx_channel
{
   MESSAGE_RX_CHANNEL_MESSAGE           = 0,
   MESSAGE_RX_CHANNEL_BULK              = 1 // drivers may provide multiple bulk channels, from 1 upwards
} MESSAGE_RX_CHANNEL_T;

// Message receive slot information
typedef struct rx_msg_slot_info {

   struct rx_msg_slot_info *next;
   //struct slot_info *prev;

   //zrl
   //struct opaque_os_semaphore_t *sem;
   OS_SEMAPHORE_T sem; 

   uint8_t           *addr;               // base address of slot
   uint32_t           len;                // length of slot in bytes

   uint32_t           write_ptr;          // hardware causes this to advance
   uint32_t           read_ptr;           // this module does the reading
   int                active;             // is this slot in the hardware dma fifo?
   uint32_t           messages;           // count how many messages are in this slot
   uint32_t           slot_no;            // need to be able to identify slots by number to allow tracking for Xon / Xoff purposes
   void              *state;              // connection state information
} RX_MSG_SLOTINFO_T;

// The message driver no longer needs to know about the fields of RX_BULK_SLOTINFO_T - sort this out.
// In particular, it mustn't use addr and len - they're the client buffer, but the message
// driver will be tasked with sending the aligned core section.
typedef struct rx_bulk_slotinfo_t {
   struct rx_bulk_slotinfo_t *next;

   //zrl
   //struct opaque_os_semaphore_t *blocking;
   OS_SEMAPHORE_T *blocking; 

   // needed by DMA
   void        *addr;
   uint32_t     len;

   // needed for the callback
   void        *service;
   void        *handle;
   VCHI_FLAGS_T flags;
} RX_BULK_SLOTINFO_T;


/* ----------------------------------------------------------------------
 * each connection driver will have a pool of the following struct.
 *
 * the pool will be managed by vchi_qman_*
 * this means there will be multiple queues (single linked lists)
 * a given struct message_info will be on exactly one of these queues
 * at any one time
 * -------------------------------------------------------------------- */
typedef struct rx_message_info {

   struct message_info *next;
   //struct message_info *prev;

   uint8_t    *addr;
   uint32_t   len;
   RX_MSG_SLOTINFO_T *slot; // points to whichever slot contains this message
   fourcc_t sid; // which service this message is destined for.  FIXME: remove!
   uint32_t   tx_timestamp;
   uint32_t   rx_timestamp;

} RX_MESSAGE_INFO_T;


typedef struct {
   MESSAGE_EVENT_TYPE_T type;

   struct {
      // for messages
      void    *addr;           // address of message
      uint16_t slot_delta;     // whether this message indicated slot delta
      uint16_t len;            // length of message
      RX_MSG_SLOTINFO_T *slot; // slot this message is in
      fourcc_t service;        // service id this message is destined for
      uint32_t tx_timestamp;   // timestamp from the header
      uint32_t rx_timestamp;   // timestamp when we parsed it
   } message;

   // FIXME: cleanup slot reporting...
   RX_MSG_SLOTINFO_T *rx_msg;
   RX_BULK_SLOTINFO_T *rx_bulk;
   void *tx_handle;
   MESSAGE_TX_CHANNEL_T tx_channel;

} MESSAGE_EVENT_T;


// callbacks
typedef void VCHI_MESSAGE_DRIVER_EVENT_CALLBACK_T( void *state );

typedef struct {
   VCHI_MESSAGE_DRIVER_EVENT_CALLBACK_T *event_callback;
} VCHI_MESSAGE_DRIVER_OPEN_T;


// handle to this instance of message driver (as returned by ->open)
typedef struct opaque_mhandle_t *VCHI_MDRIVER_HANDLE_T;

typedef struct {
   VCHI_MDRIVER_HANDLE_T *(*open)( VCHI_MESSAGE_DRIVER_OPEN_T *params, void *state );
   int32_t (*add_msg_rx_slot)( VCHI_MDRIVER_HANDLE_T *handle, RX_MSG_SLOTINFO_T *slot );      // rx message
   int32_t (*add_bulk_rx)( VCHI_MDRIVER_HANDLE_T *handle, void *data, uint32_t len, RX_BULK_SLOTINFO_T *slot );  // rx data (bulk)
   int32_t (*send)( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel, const void *data, uint32_t len, VCHI_MSG_FLAGS_T flags, void *send_handle );      // tx (message & bulk)
   void    (*next_event)( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_EVENT_T *event );     // get the next event from message_driver
   int32_t (*enable)( VCHI_MDRIVER_HANDLE_T *handle );
   int32_t (*form_message)( VCHI_MDRIVER_HANDLE_T *handle, fourcc_t service_id, VCHI_MSG_VECTOR_T *vector, uint32_t count, void *address, uint32_t length_avail, uint32_t max_total_length, bool_t pad_to_fill, bool_t allow_partial );

   int32_t (*update_message)( VCHI_MDRIVER_HANDLE_T *handle, void *dest, int16_t *slot_count );
   int32_t (*buffer_aligned)( VCHI_MDRIVER_HANDLE_T *handle, int tx, const void *address, const uint32_t length );
   void *  (*allocate_buffer)( VCHI_MDRIVER_HANDLE_T *handle, uint32_t *length );
   void    (*free_buffer)( VCHI_MDRIVER_HANDLE_T *handle, void *address );
   int     (*slot_size)( VCHI_MDRIVER_HANDLE_T *handle, int msg_size );

   bool_t  (*tx_supports_terminate)( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel );
   int     (*tx_alignment)( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel );
   int     (*rx_alignment)( const VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_RX_CHANNEL_T channel );
   void    (*form_bulk_aux)( VCHI_MDRIVER_HANDLE_T *handle, MESSAGE_TX_CHANNEL_T channel, const void *data, uint32_t len, uint32_t chunk_size, const void **aux_data, int32_t *aux_len );
   void    (*debug)( VCHI_MDRIVER_HANDLE_T *handle );
} VCHI_MESSAGE_DRIVER_T;

// declare all message drivers here
const VCHI_MESSAGE_DRIVER_T *vchi_mphi_message_driver_func_table( void );


#endif // _VCHI_MESSAGE_H_

/****************************** End of file ***********************************/
