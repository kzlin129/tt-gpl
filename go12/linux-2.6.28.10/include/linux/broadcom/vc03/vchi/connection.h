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

Project  :  Powerman
Module   :  Interface to the power manager
File     :  $RCSfile: powerman.h,v $
Revision :  $Revision: #3 $

FILE DESCRIPTION:
Contains the protypes for the interface functions.
=============================================================================*/

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include <linux/broadcom/vc03/vcos.h>
#include "message.h"

/******************************************************************************
 Global defs
 *****************************************************************************/

//callback reasons when an event occurs on a service
typedef enum
{
   VCHI_CALLBACK_REASON_MIN,

   //This indicates that there is data available
   //handle is the msg id that was transmitted with the data
   //    When a message is received and there was no FULL message available previously, send callback
   //    Tasks get kicked by the callback, reset their event and try and read from the fifo until it fails
   VCHI_CALLBACK_MSG_AVAILABLE,
   VCHI_CALLBACK_MSG_SENT,
   VCHI_CALLBACK_MSG_SPACE_AVAILABLE, // XXX not yet implemented

   // This indicates that a transfer from the other side has completed
   VCHI_CALLBACK_BULK_RECEIVED,
   //This indicates that data queued upto be sent has now gone
   //handle is the msg id that was used when sending the data
   VCHI_CALLBACK_BULK_SENT,
   VCHI_CALLBACK_BULK_RX_SPACE_AVAILABLE, // XXX not yet implemented
   VCHI_CALLBACK_BULK_TX_SPACE_AVAILABLE, // XXX not yet implemented

   VCHI_CALLBACK_SERVICE_CLOSED, // XXX not yet implemented

   // this side has sent XOFF to peer due to lack of data consumption by service
   // (suggests the service may need to take some recovery action if it has
   // been deliberately holding off consuming data)
   VCHI_CALLBACK_SENT_XOFF,
   VCHI_CALLBACK_SENT_XON,

   // indicates that a bulk transfer has finished reading the source buffer
   VCHI_CALLBACK_BULK_DATA_READ,

   VCHI_CALLBACK_REASON_MAX
} VCHI_CALLBACK_REASON_T;

//Calback used by all services / bulk transfers
typedef void (*VCHI_CALLBACK_T)( void *callback_param, //my service local param
                                 VCHI_CALLBACK_REASON_T reason,
                                 void *handle ); //for transmitting msg's only


// Opaque handle for a connection / service pair
typedef struct opaque_vchi_connection_connected_service_handle_t *VCHI_CONNECTION_SERVICE_HANDLE_T;

// opaque handle to the connection state information
typedef struct opaque_vchi_connection_info_t VCHI_CONNECTION_STATE_T;
/******************************************************************************
 API
 *****************************************************************************/

// Routine to init a connection with a particular low level driver
typedef VCHI_CONNECTION_STATE_T * (*VCHI_CONNECTION_INIT_T)( const VCHI_MESSAGE_DRIVER_T * driver );

// Routine to control CRC enabling at a connection level
typedef int32_t (*VCHI_CONNECTION_CRC_CONTROL_T)( VCHI_CONNECTION_STATE_T *state_handle,
                                                  VCHI_CRC_CONTROL_T control );

// Routine to create a service
typedef int32_t (*VCHI_CONNECTION_SERVICE_CONNECT_T)( VCHI_CONNECTION_STATE_T *state_handle,
                                                      fourcc_t service_id,
                                                      uint32_t rx_fifo_size,
                                                      uint32_t tx_fifo_size,
                                                      int server,
                                                      VCHI_CALLBACK_T callback,
                                                      void *callback_param,
                                                      bool_t want_crc,
                                                      bool_t want_unaligned_bulk_rx,
                                                      bool_t want_unaligned_bulk_tx,
                                                      VCHI_CONNECTION_SERVICE_HANDLE_T *service_handle );

// Routine to close a service
typedef int32_t (*VCHI_CONNECTION_SERVICE_DISCONNECT_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle );

// Routine to queue a message
typedef int32_t (*VCHI_CONNECTION_SERVICE_QUEUE_MESSAGE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                            const void *data,
                                                            uint32_t data_size,
                                                            VCHI_FLAGS_T flags,
                                                            void *msg_handle );

// scatter-gather (vector) message queueing
typedef int32_t (*VCHI_CONNECTION_SERVICE_QUEUE_MESSAGEV_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                             VCHI_MSG_VECTOR_T *vector,
                                                             uint32_t count,
                                                             VCHI_FLAGS_T flags,
                                                             void *msg_handle );

// Routine to dequeue a message
typedef int32_t (*VCHI_CONNECTION_SERVICE_DEQUEUE_MESSAGE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                              void *data,
                                                              uint32_t max_data_size_to_read,
                                                              uint32_t *actual_msg_size,
                                                              VCHI_FLAGS_T flags );

// Routine to peek at a message
typedef int32_t (*VCHI_CONNECTION_SERVICE_PEEK_MESSAGE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                           void **data,
                                                           uint32_t *msg_size,
                                                           VCHI_FLAGS_T flags );

// Routine to hold a message
typedef int32_t (*VCHI_CONNECTION_SERVICE_HOLD_MESSAGE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                           void **data,
                                                           uint32_t *msg_size,
                                                           VCHI_FLAGS_T flags,
                                                           void **message_handle );

// Routine to initialise a received message iterator
typedef int32_t (*VCHI_CONNECTION_SERVICE_LOOKAHEAD_MESSAGE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                                VCHI_MSG_ITER_T *iter,
                                                                VCHI_FLAGS_T flags );

// Routine to release a held message
typedef int32_t (*VCHI_CONNECTION_HELD_MSG_RELEASE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                             void *message_handle );

// Routine to get info on a held message
typedef int32_t (*VCHI_CONNECTION_HELD_MSG_INFO_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                    void *message_handle,
                                                    void **data,
                                                    int32_t *msg_size,
                                                    uint32_t *tx_timestamp,
                                                    uint32_t *rx_timestamp );

// Routine to check whether the iterator has a next message
typedef bool_t (*VCHI_CONNECTION_MSG_ITER_HAS_NEXT_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service,
                                                       const VCHI_MSG_ITER_T *iter );

// Routine to advance the iterator
typedef int32_t (*VCHI_CONNECTION_MSG_ITER_NEXT_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service,
                                                    VCHI_MSG_ITER_T *iter,
                                                    void **data,
                                                    uint32_t *msg_size );

// Routine to remove the last message returned by the iterator
typedef int32_t (*VCHI_CONNECTION_MSG_ITER_REMOVE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service,
                                                      VCHI_MSG_ITER_T *iter );

// Routine to hold the last message returned by the iterator
typedef int32_t (*VCHI_CONNECTION_MSG_ITER_HOLD_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service,
                                                    VCHI_MSG_ITER_T *iter,
                                                    void **msg_handle );

// Routine to transmit bulk data
typedef int32_t (*VCHI_CONNECTION_BULK_QUEUE_TRANSMIT_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                          const void *data_src,
                                                          uint32_t data_size,
                                                          VCHI_FLAGS_T flags,
                                                          void *bulk_handle );

// Routine to receive data
typedef int32_t (*VCHI_CONNECTION_BULK_QUEUE_RECEIVE_T)( VCHI_CONNECTION_SERVICE_HANDLE_T service_handle,
                                                         void *data_dst,
                                                         uint32_t data_size,
                                                         VCHI_FLAGS_T flags,
                                                         void *bulk_handle );

// Routine to report if a server is available
typedef int32_t (*VCHI_CONNECTION_SERVER_PRESENT)( VCHI_CONNECTION_STATE_T *state, fourcc_t service_id, int32_t peer_flags );

// Routine to report the number of RX slots available
typedef int (*VCHI_CONNECTION_RX_SLOTS_AVAILABLE)( const VCHI_CONNECTION_STATE_T *state );

// Routine to report the RX slot size
typedef uint32_t (*VCHI_CONNECTION_RX_SLOT_SIZE)( const VCHI_CONNECTION_STATE_T *state );

// Callback to indicate that the other side has added a buffer to the rx bulk DMA FIFO
typedef void (*VCHI_CONNECTION_RX_BULK_BUFFER_ADDED)(VCHI_CONNECTION_STATE_T *state,
                                                     fourcc_t service,
                                                     uint32_t length,
                                                     MESSAGE_TX_CHANNEL_T channel,
                                                     uint32_t channel_params,
                                                     uint32_t data_length,
                                                     uint32_t data_offset);

// Callback to inform a service that a Xon or Xoff message has been received
typedef void (*VCHI_CONNECTION_FLOW_CONTROL)(VCHI_CONNECTION_STATE_T *state, fourcc_t service_id, int32_t xoff);

// Callback to inform a service that a server available reply message has been received
typedef void (*VCHI_CONNECTION_SERVER_AVAILABLE_REPLY)(VCHI_CONNECTION_STATE_T *state, fourcc_t service_id, uint32_t flags);

// Callback to indicate that bulk auxiliary messages have arrived
typedef void (*VCHI_CONNECTION_BULK_AUX_RECEIVED)(VCHI_CONNECTION_STATE_T *state);

// Callback to indicate that bulk auxiliary messages have arrived
typedef void (*VCHI_CONNECTION_BULK_AUX_TRANSMITTED)(VCHI_CONNECTION_STATE_T *state, void *handle);

// Callback with all the connection info you require
typedef void (*VCHI_CONNECTION_INFO)(VCHI_CONNECTION_STATE_T *state, uint32_t protocol_version, uint32_t slot_size, uint32_t num_slots, uint32_t min_bulk_size);

// allocate memory suitably aligned for this connection
typedef void * (*VCHI_BUFFER_ALLOCATE)(VCHI_CONNECTION_SERVICE_HANDLE_T service_handle, uint32_t * length);

// free memory allocated by buffer_allocate
typedef void   (*VCHI_BUFFER_FREE)(VCHI_CONNECTION_SERVICE_HANDLE_T service_handle, void * address);


/******************************************************************************
 System driver struct
 *****************************************************************************/

typedef struct
{
   // Routine to init the connection
   VCHI_CONNECTION_INIT_T                    init;

   // Connection-level CRC control
   VCHI_CONNECTION_CRC_CONTROL_T               crc_control;

   // Routine to connect to or create service
   VCHI_CONNECTION_SERVICE_CONNECT_T         service_connect;

   // Routine to disconnect from a service
   VCHI_CONNECTION_SERVICE_DISCONNECT_T      service_disconnect;

   // Routine to queue a message
   VCHI_CONNECTION_SERVICE_QUEUE_MESSAGE_T   service_queue_msg;

   // scatter-gather (vector) message queue
   VCHI_CONNECTION_SERVICE_QUEUE_MESSAGEV_T  service_queue_msgv;

   // Routine to dequeue a message
   VCHI_CONNECTION_SERVICE_DEQUEUE_MESSAGE_T service_dequeue_msg;

   // Routine to peek at a message
   VCHI_CONNECTION_SERVICE_PEEK_MESSAGE_T    service_peek_msg;

   // Routine to hold a message
   VCHI_CONNECTION_SERVICE_HOLD_MESSAGE_T      service_hold_msg;

   // Routine to initialise a received message iterator
   VCHI_CONNECTION_SERVICE_LOOKAHEAD_MESSAGE_T service_look_ahead_msg;

   // Routine to release a message
   VCHI_CONNECTION_HELD_MSG_RELEASE_T          held_msg_release;

   // Routine to get information on a held message
   VCHI_CONNECTION_HELD_MSG_INFO_T             held_msg_info;

   // Routine to check for next message on iterator
   VCHI_CONNECTION_MSG_ITER_HAS_NEXT_T         msg_iter_has_next;

   // Routine to get next message on iterator
   VCHI_CONNECTION_MSG_ITER_NEXT_T             msg_iter_next;

   // Routine to remove the last message returned by iterator
   VCHI_CONNECTION_MSG_ITER_REMOVE_T           msg_iter_remove;

   // Routine to hold the last message returned by iterator
   VCHI_CONNECTION_MSG_ITER_HOLD_T             msg_iter_hold;

   // Routine to transmit bulk data
   VCHI_CONNECTION_BULK_QUEUE_TRANSMIT_T     bulk_queue_transmit;

   // Routine to receive data
   VCHI_CONNECTION_BULK_QUEUE_RECEIVE_T      bulk_queue_receive;

   // Routine to report the available servers
   VCHI_CONNECTION_SERVER_PRESENT            server_present;

   // Routine to report the number of RX slots available
   VCHI_CONNECTION_RX_SLOTS_AVAILABLE          connection_rx_slots_available;

   // Routine to report the RX slot size
   VCHI_CONNECTION_RX_SLOT_SIZE                connection_rx_slot_size;

   // Callback to indicate that the other side has added a buffer to the rx bulk DMA FIFO
   VCHI_CONNECTION_RX_BULK_BUFFER_ADDED      rx_bulk_buffer_added;

   // Callback to inform a service that a Xon or Xoff message has been received
   VCHI_CONNECTION_FLOW_CONTROL              flow_control;

   // Callback to inform a service that a server available reply message has been received
   VCHI_CONNECTION_SERVER_AVAILABLE_REPLY      server_available_reply;

   // Callback to indicate that bulk auxiliary messages have arrived
   VCHI_CONNECTION_BULK_AUX_RECEIVED           bulk_aux_received;

   // Callback to indicate that a bulk auxiliary message has been transmitted
   VCHI_CONNECTION_BULK_AUX_TRANSMITTED        bulk_aux_transmitted;

   // Callback to provide information about the connection
   VCHI_CONNECTION_INFO                      connection_info;

   // allocate memory suitably aligned for this connection
   VCHI_BUFFER_ALLOCATE                      buffer_allocate;

   // free memory allocated by buffer_allocate
   VCHI_BUFFER_FREE                          buffer_free;

} VCHI_CONNECTION_API_T;

typedef struct {
   const VCHI_CONNECTION_API_T *api;
   VCHI_CONNECTION_STATE_T *state;
} VCHI_CONNECTION_T;


// function prototypes for the different mid layers (the state info gives the different physical connections)
extern const VCHI_CONNECTION_API_T *single_get_func_table( void );
//extern const VCHI_CONNECTION_API_T *local_server_get_func_table( void );
//extern const VCHI_CONNECTION_API_T *local_client_get_func_table( void );

#endif /* CONNECTION_H_ */

/****************************** End of file **********************************/
