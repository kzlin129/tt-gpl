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

Project  :  vchi
Module   :  vchi
File     :  $RCSfile:  $
Revision :  $Revision: #9 $

FILE DESCRIPTION:
Contains the protypes for the vchi functions.
=============================================================================*/

#ifndef VCHI_H_
#define VCHI_H_

#include <linux/broadcom/vc03/vcos.h>
#include "vchi_cfg.h"

/******************************************************************************
 Global defs
 *****************************************************************************/

#define VCHI_BULK_ROUND_UP(x)     ((((unsigned long)(x))+VCHI_BULK_ALIGN-1) & ~(VCHI_BULK_ALIGN-1))
#define VCHI_BULK_ROUND_DOWN(x)   (((unsigned long)(x)) & ~(VCHI_BULK_ALIGN-1))
#define VCHI_BULK_ALIGNED(x)      (((unsigned long)(x) & (VCHI_BULK_ALIGN-1)) == 0)

// msgfifo wrapper commands - this will go away eventually once all services converted to native vchi
typedef enum {
   MSGFIFO_WRAPPER_READ = 0,
   MSGFIFO_WRAPPER_WRITE,
   MSGFIFO_WRAPPER_READ_SPECIFIC_VC_APP,
   MSGFIFO_WRAPPER_READ_SPECIFIC_VC_SHAREDPTR,
   MSGFIFO_WRAPPER_BULK_READ,
   MSGFIFO_WRAPPER_BULK_WRITE,
   MSGFIFO_WRAPPER_READ_DATA,
   MSGFIFO_WRAPPER_HOST_ATTENTION_REQUEST
} MSGFIFO_WRAPPER_CMD_T;


//flags used when sending messages (must be bitmapped)
typedef enum
{
   VCHI_FLAGS_NONE                      = 0x0,
   VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE   = 0x1,   // waits for message to be received, or sent (NB. not the same as being seen on other side)
   VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE = 0x2,   // run a callback when message sent
   VCHI_FLAGS_BLOCK_UNTIL_QUEUED        = 0x4,   // return once the transfer is in a queue ready to go
   VCHI_FLAGS_ALLOW_PARTIAL             = 0x8,
   VCHI_FLAGS_BLOCK_UNTIL_DATA_READ     = 0x10,
   VCHI_FLAGS_CALLBACK_WHEN_DATA_READ   = 0x20,

   VCHI_FLAGS_ALIGN_SLOT            = 0x000080,  // internal use only
   VCHI_FLAGS_BULK_AUX_QUEUED       = 0x010000,  // internal use only
   VCHI_FLAGS_BULK_AUX_COMPLETE     = 0x020000,  // internal use only
   VCHI_FLAGS_BULK_DATA_QUEUED      = 0x040000,  // internal use only
   VCHI_FLAGS_BULK_DATA_COMPLETE    = 0x080000,  // internal use only
   VCHI_FLAGS_INTERNAL              = 0xFF0000
} VCHI_FLAGS_T;

// constants for vchi_crc_control()
typedef enum {
   VCHI_CRC_NOTHING = -1,
   VCHI_CRC_PER_SERVICE = 0,
   VCHI_CRC_EVERYTHING = 1,
} VCHI_CRC_CONTROL_T;

// Opaque service information
struct opaque_vchi_service_t;

// Descriptor for a held message. Allocated by client, initialised by vchi_msg_hold,
// vchi_msg_iter_hold or vchi_msg_iter_hold_next. Fields are for internal VCHI use only.
typedef struct
{
   struct opaque_vchi_service_t *service;
   void *message;
} VCHI_HELD_MSG_T;

// Iterator structure for reading ahead through received message queue. Allocated by client,
// initialised by vchi_msg_look_ahead. Fields are for internal VCHI use only.
// Iterates over messages in queue at the instant of the call to vchi_msg_lookahead -
// will not proceed to messages received since. Behaviour is undefined if an iterator
// is used again after messages for that service are removed/dequeued by any
// means other than vchi_msg_iter_... calls on the iterator itself.
typedef struct {
   struct opaque_vchi_service_t *service;
   void *last;
   void *next;
   void *remove;
} VCHI_MSG_ITER_T;

#include "linux/broadcom/vc03/vchi/connection.h"

// structure used to provide the information needed to open a server or a client
typedef struct {
   fourcc_t service_id;
   VCHI_CONNECTION_T *connection;
   uint32_t rx_fifo_size;
   uint32_t tx_fifo_size;
   VCHI_CALLBACK_T callback;
   void *callback_param;
   bool_t want_unaligned_bulk_rx;    // client intends to receive bulk transfers of odd lengths or into unaligned buffers
   bool_t want_unaligned_bulk_tx;    // client intends to transmit bulk transfers of odd lengths or out of unaligned buffers
   bool_t want_crc;                  // client wants to check CRCs on (bulk) transfers. Only needs to be set at 1 end - will do both directions.
} SERVICE_CREATION_T;

// Opaque handle for a VCHI instance
typedef struct opaque_vchi_instance_handle_t *VCHI_INSTANCE_T;

// Opaque handle for a server or client
typedef struct opaque_vchi_service_handle_t *VCHI_SERVICE_HANDLE_T;

// helper functions
extern void * vchi_allocate_buffer(VCHI_SERVICE_HANDLE_T handle, uint32_t *length);
extern void vchi_free_buffer(VCHI_SERVICE_HANDLE_T handle, void *address);
extern uint32_t vchi_current_time(VCHI_INSTANCE_T instance_handle);

// Service registration & startup
typedef void (*VCHI_SERVICE_INIT)(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections);

typedef struct service_info_tag {
   const char * const vll_filename; /* VLL to load to start this service. This is an empty string if VLL is "static" */
   VCHI_SERVICE_INIT init;          /* Service initialisation function */
   void *vll_handle;                /* VLL handle; NULL when unloaded or a "static VLL" in build */
} SERVICE_INFO_T;

/******************************************************************************
 Global funcs - implementation is specific to which side you are on (local / remote)
 *****************************************************************************/

extern VCHI_CONNECTION_T * vchi_create_connection( const VCHI_CONNECTION_API_T * function_table,
                                                   const VCHI_MESSAGE_DRIVER_T * low_level);


// Routine used to initialise the vchi on both local + remote connections
extern int32_t vchi_initialise( VCHI_INSTANCE_T *instance_handle );

extern int32_t vchi_exit( void );

extern int32_t vchi_connect( VCHI_CONNECTION_T **connections,
                             const uint32_t num_connections,
                             VCHI_INSTANCE_T instance_handle );

//When this is called, ensure that all services have no data pending.
//Bulk transfers can remain 'queued'
extern int32_t vchi_disconnect( VCHI_INSTANCE_T instance_handle );

// Global control over bulk CRC checking
extern int32_t vchi_crc_control( VCHI_CONNECTION_T *connection,
                                 VCHI_CRC_CONTROL_T control );

/******************************************************************************
 Global service API
 *****************************************************************************/
// Routine to create a named service
extern int32_t vchi_service_create( VCHI_INSTANCE_T instance_handle,
                                    SERVICE_CREATION_T *setup,
                                    VCHI_SERVICE_HANDLE_T *handle );

// Routine to destory a service
extern int32_t vchi_service_destroy( const VCHI_SERVICE_HANDLE_T handle );

// Routine to open a named service
extern int32_t vchi_service_open( VCHI_INSTANCE_T instance_handle,
                                  SERVICE_CREATION_T *setup,
                                  VCHI_SERVICE_HANDLE_T *handle);

// Routine to close a named service
extern int32_t vchi_service_close( const VCHI_SERVICE_HANDLE_T handle );

// Routine to send a message accross a service
extern int32_t vchi_msg_queue( VCHI_SERVICE_HANDLE_T handle,
                               const void *data,
                               uint32_t data_size,
                               VCHI_FLAGS_T flags,
                               void *msg_handle );

// scatter-gather (vector) and send message
int32_t vchi_msg_queuev( VCHI_SERVICE_HANDLE_T handle,
                         VCHI_MSG_VECTOR_T *vector,
                         uint32_t count,
                         VCHI_FLAGS_T flags,
                         void *msg_handle );

// Routine to receive a msg from a service
// Dequeue is equivalent to hold, copy into client buffer, release
extern int32_t vchi_msg_dequeue( VCHI_SERVICE_HANDLE_T handle,
                                 void *data,
                                 uint32_t max_data_size_to_read,
                                 uint32_t *actual_msg_size,
                                 VCHI_FLAGS_T flags );

// Routine to look at a message in place.
// The message is not dequeued, so a subsequent call to peek or dequeue
// will return the same message.
extern int32_t vchi_msg_peek( VCHI_SERVICE_HANDLE_T handle,
                              void **data,
                              uint32_t *msg_size,
                              VCHI_FLAGS_T flags );

// Routine to remove a message after it has been read in place with peek
// The first message on the queue is dequeued.
extern int32_t vchi_msg_remove( VCHI_SERVICE_HANDLE_T handle );

// Routine to look at a message in place.
// The message is dequeued, so the caller is left holding it; the descriptor is
// filled in and must be released when the user has finished with the message.
extern int32_t vchi_msg_hold( VCHI_SERVICE_HANDLE_T handle,
                              void **data,        // } may be NULL, as info can be
                              uint32_t *msg_size, // } obtained from HELD_MSG_T
                              VCHI_FLAGS_T flags,
                              VCHI_HELD_MSG_T *message_descriptor );

// Initialise an iterator to look through messages in place
extern int32_t vchi_msg_look_ahead( VCHI_SERVICE_HANDLE_T handle,
                                    VCHI_MSG_ITER_T *iter,
                                    VCHI_FLAGS_T flags );

/******************************************************************************
 Global service support API - operations on held messages and message iterators
 *****************************************************************************/

// Routine to get the address of a held message
extern void *vchi_held_msg_ptr( const VCHI_HELD_MSG_T *message );

// Routine to get the size of a held message
extern int32_t vchi_held_msg_size( const VCHI_HELD_MSG_T *message );

// Routine to get the transmit timestamp as written into the header by the peer
extern uint32_t vchi_held_msg_tx_timestamp( const VCHI_HELD_MSG_T *message );

// Routine to get the reception timestamp, written as we parsed the header
extern uint32_t vchi_held_msg_rx_timestamp( const VCHI_HELD_MSG_T *message );

// Routine to release a held message after it has been processed
extern int32_t vchi_held_msg_release( VCHI_HELD_MSG_T *message );

// Indicates whether the iterator has a next message.
extern bool_t vchi_msg_iter_has_next( const VCHI_MSG_ITER_T *iter );

// Return the pointer and length for the next message and advance the iterator.
extern int32_t vchi_msg_iter_next( VCHI_MSG_ITER_T *iter,
                              void **data,
                                   uint32_t *msg_size );

// Remove the last message returned by vchi_msg_iter_next.
// Can only be called once after each call to vchi_msg_iter_next.
extern int32_t vchi_msg_iter_remove( VCHI_MSG_ITER_T *iter );

// Hold the last message returned by vchi_msg_iter_next.
// Can only be called once after each call to vchi_msg_iter_next.
extern int32_t vchi_msg_iter_hold( VCHI_MSG_ITER_T *iter,
                                   VCHI_HELD_MSG_T *message );

// Return information for the next message, and hold it, advancing the iterator.
extern int32_t vchi_msg_iter_hold_next( VCHI_MSG_ITER_T *iter,
                                        void **data,        // } may be NULL
                                        uint32_t *msg_size, // }
                                        VCHI_HELD_MSG_T *message );


/******************************************************************************
 Global bulk API
 *****************************************************************************/

// Routine to prepare interface for a transfer from the other side
extern int32_t vchi_bulk_queue_receive( VCHI_SERVICE_HANDLE_T handle,
                                        void *data_dst,
                                        uint32_t data_size,
                                        VCHI_FLAGS_T flags,
                                        void *transfer_handle );


// Routine to queue up data ready for transfer to the other (once they have signalled they are ready)
extern int32_t vchi_bulk_queue_transmit( VCHI_SERVICE_HANDLE_T handle,
                                         const void *data_src,
                                         uint32_t data_size,
                                         VCHI_FLAGS_T flags,
                                         void *transfer_handle );

#endif /* VCHI_H_ */

/****************************** End of file **********************************/
