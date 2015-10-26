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

Project  :  vcfw
Module   :  chip driver
File     :  $RCSfile: mphi.h,v $
Revision :  $Revision: #0 $

FILE DESCRIPTION
Chip driver for the MPHI peripheral
=============================================================================*/

#ifndef MPHI_H_
#define MPHI_H_

#include <linux/broadcom/vc03/vcos.h>
#include "driver.h"


/******************************************************************************
 Global types
 *****************************************************************************/

// number of slots in the DMA FIFOs
// FIXME: should these be in hardware.h ?
#define MPHI_TX_SLOTS      16
#define MPHI_RX_MSG_SLOTS  16
#define MPHI_RX_BULK_SLOTS 16

// channels supported by the interface
typedef enum {
   MPHI_CHANNEL_OUT = 0,
   MPHI_CHANNEL_IN_CONTROL,
   MPHI_CHANNEL_IN_DATA,
   MPHI_CHANNEL_IN_HOST_SPECIFIED,
   MPHI_CHANNEL_MAX
} MPHI_CHANNEL_T;

typedef enum {
   MPHI_EVENT_NONE,                  // no more events to be processed
   MPHI_EVENT_IN_CONTROL_POSITION,      // IN control channel: the input pointer has changed
   MPHI_EVENT_IN_CONTROL_DMA_COMPLETED, // IN control channel: the DMA has completed
   MPHI_EVENT_IN_DATA_POSITION,         // IN data channel: the input pointer has changed
   MPHI_EVENT_IN_DATA_DMA_COMPLETED, // IN data channel: the DMA has completed
   MPHI_EVENT_IN_MESSAGE_DISCARDED,  // IN channels: an incoming message for this channel was discarded due to empty DMA FIFO
   MPHI_EVENT_OUT_DMA_COMPLETED,        // OUT channel: the specified message has been read from memory
   MPHI_EVENT_IN_FIFO_OVERFLOW,         // IN channels: incoming FIFO overflow
   MPHI_EVENT_OUT_FIFO_UNDERFLOW        // OUT channel: host read faster than output FIFO could supply data
} MPHI_EVENT_TYPE_T;

enum {
   MPHI_ERROR_GENERAL = -1,
   MPHI_ERROR_FIFO_FULL = -2,
};

//flags used when sending messages (must be bitmapped)
typedef enum
{
   MPHI_FLAGS_NONE                      = 0x0,
   MPHI_FLAGS_TERMINATE_DMA             = 0x1
} MPHI_FLAGS_T;

// event callback
typedef void MPHI_EVENT_CALLBACK_T( const void *callback_data );

typedef struct {
   MPHI_CHANNEL_T channel;
   MPHI_EVENT_CALLBACK_T *callback;
   const void *callback_data;
} MPHI_OPEN_T;


/******************************************************************************
 API
 *****************************************************************************/

// add slots for incoming message(s)
typedef int32_t (*MPHI_ADD_RECV_SLOT)( const DRIVER_HANDLE_T handle, const void *addr, uint32_t len, uint8_t slot_id );

// Outgoing
typedef int32_t (*MPHI_OUT_QUEUE_MESSAGE)( const DRIVER_HANDLE_T handle, uint8_t control, const void *addr, size_t len, uint8_t msg_id, const MPHI_FLAGS_T flags );

// General
typedef int (*MPHI_SLOTS_AVAILABLE)( const DRIVER_HANDLE_T handle );

// retrieve next event
typedef MPHI_EVENT_TYPE_T (*MPHI_NEXT_EVENT)( const DRIVER_HANDLE_T handle, uint8_t *slot_id, uint32_t *pos );

typedef int32_t (*MPHI_ENABLE)( const DRIVER_HANDLE_T handle );

// Set drive power (mA) and slew (0=high power, 1=slew, low power)
typedef int32_t (*MPHI_SET_POWER)( const DRIVER_HANDLE_T handle, int drive_mA, int slew );

// Get pointer alignment requirement
typedef int (*MPHI_ALIGNMENT) ( const DRIVER_HANDLE_T handle );

typedef void (*MPHI_DEBUG)( const DRIVER_HANDLE_T handle );

// wait for I/O to finish
typedef void (*MPHI_FINISH_IO)( const int purge );


/******************************************************************************
 System driver struct
 *****************************************************************************/

typedef struct
{
   //include the common driver definitions - this is a macro
   COMMON_DRIVER_API( MPHI_OPEN_T const *params )

   MPHI_ADD_RECV_SLOT            add_recv_slot;

   MPHI_OUT_QUEUE_MESSAGE        out_queue_message;
   MPHI_SLOTS_AVAILABLE          slots_available;

   MPHI_NEXT_EVENT               next_event;

   MPHI_ENABLE                   enable;

   MPHI_SET_POWER                set_power;
   
   MPHI_ALIGNMENT                alignment;
   
   MPHI_DEBUG                    debug;

   MPHI_FINISH_IO		 finish_io;

} MPHI_DRIVER_T;

/******************************************************************************
 Global Functions
 *****************************************************************************/

const MPHI_DRIVER_T *mphi_get_func_table( void );


#endif /*MPHI_H_*/
