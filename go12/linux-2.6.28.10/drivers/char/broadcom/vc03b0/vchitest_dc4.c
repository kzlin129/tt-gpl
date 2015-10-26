/*****************************************************************************
*  Copyright 2001 - 2007 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

/*=============================================================================
Copyright (c) 2008 Broadcom Europe Limited.
All rights reserved.

Project  :  VCHI
Module   :  DC4
File     :  $RCSfile: $
Revision :  $Revision: $

FILE DESCRIPTION
Server and Client implementations of the DC4 test service
=============================================================================*/
#if 0
#include <assert.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "vcinclude/common.h"
#include "interface/vchi/os/os.h"
#include "interface/vchi/vchi.h"
#include "interface/vchi/common/endian.h"

#include "test.h"
#endif

#if defined(__KERNEL__)

#include <linux/string.h>

#else // __KERNEL__

#include <string.h>
#include <stdio.h>

#endif

#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/message.h>
#include <linux/broadcom/vc03/vchi/endian.h>
#include <linux/broadcom/vc03/vchi/mphi.h>
#include <linux/broadcom/vc03/vchi/control_service.h>
#include <linux/broadcom/vc03/vchi/host_msgfifo_wrapper.h>
#include <linux/broadcom/vc03/vcos.h>
#include "vcgencmd.h"
#include "vc_drv.h"

#define printf(fmt, args...) printk(KERN_ERR fmt, ##args)

#define CS "H"
#define OTHER_CS "V"
//#define SPEED_TEST

#define BUFFER_SIZE (1024)
#define NUM_CLIENTS 16
#define NUM_SERVERS 4 

typedef struct {
   VCHI_SERVICE_HANDLE_T open_handle[NUM_SERVERS];
   int errors;
} TEST_SERVER_T;

typedef struct {
   VCHI_SERVICE_HANDLE_T open_handle[NUM_SERVERS];
   int errors;
} TEST_CLIENT_T;

typedef struct _header_s
{
   uint16_t client;
   uint16_t flags;
   uint32_t size;
   uint32_t seqnum;
   uint32_t checksum;
} HEADER_T;

enum {TEST_FLAGS_DELAY = 1<<0};

static TEST_SERVER_T dc4_server_state;
static TEST_CLIENT_T dc4_client_state;

// task structures
static OS_THREAD_T dc4_server_task[NUM_SERVERS];
static OS_THREAD_T dc4_client_task[NUM_CLIENTS];

// message available eventgroups
static OS_EVENTGROUP_T dc4_server_message_available[NUM_SERVERS];

// function prototypes
// server
static void dc4_server_recv( unsigned argc, void *argv );
static void dc4_server_callback( void *callback_param, VCHI_CALLBACK_REASON_T reason, void *msg_handle );
static void dc4_server_process_message( int conection_number, uint8_t * address, int32_t length, int server_num );
// client
static void dc4_client_callback( void *callback_param, VCHI_CALLBACK_REASON_T reason, void *msg_handle );
static void dc4_client_send( unsigned argc, void *argv );
// common

// Simple (FAST) Linear-Congruential PRNG - (BCPL 2^32 period version)
static unsigned int seed = 1234;
#define RANMULT   2147001325
#define RANADD    715136305
#define RAND() (seed = seed * RANMULT + RANADD)
#define drand(m) (((uint32_t)RAND()) % ((uint32_t)m))

static uint32_t checksum(const uint8_t *p, int size)
{
   int i;
   uint32_t checksum = 0xdeadbeef;
   for (i=0; i<size; i++)
      checksum = ((checksum << 1) | (checksum >> 31)) ^ p[i];      
   return checksum;
}

/******************************************************************************
 * SERVER
 ******************************************************************************/

/******************************************************************************
NAME
   dc4_server_func

SYNOPSIS
   void dc4_server_recv( UNSIGNED argc, VOID *argv )

FUNCTION
   Waits to be signalled that messages have arrived and then processes them

RETURNS
   -
******************************************************************************/
static void dc4_server_recv( unsigned argc, void *argv )
{
   int32_t success;
   uint8_t message_buffer[BUFFER_SIZE];
   uint32_t message_size;
   int i;
   int server_num = (int)argv;

   assert((unsigned)server_num < NUM_SERVERS);
   printf(CS": dc4_server_recv %d\n",server_num);
   while(1)
   {
      // wait for the event to say that there is a message
      uint32_t events;
      int32_t status;
//printf(CS": dc4_server_recv: wait for event\n");
      status = os_eventgroup_retrieve( &dc4_server_message_available[server_num], &events );
      assert(events == 1 && status == 0 );
//printf(CS": dc4_server_recv: got event 0x%x\n", events);

      while(1)
      {
         assert((unsigned)server_num < NUM_SERVERS);
         // read the message
         success = vchi_msg_dequeue( dc4_server_state.open_handle[server_num], message_buffer, sizeof(message_buffer), &message_size, VCHI_FLAGS_NONE );

         // if there was no message then we need to wait for the signal that there is another one
         if (success != 0) {
//printf(CS": No message %d\n", status);
            break;
         }

         // process the message contents
         dc4_server_process_message( 0, message_buffer, message_size, server_num );
      }
   }
}

static void dc4_client_send( unsigned argc, void *argv )
{
   int32_t success;
   uint8_t message_buffer[BUFFER_SIZE];
   int j;
   uint32_t sequence_number[NUM_SERVERS] = {0};
   uint32_t client = (int)argv;

#ifdef SPEED_TEST
{
   int size = -sizeof(HEADER_T);//(1024-sizeof(HEADER_T));
   HEADER_T *h = (HEADER_T *)message_buffer;
   uint8_t *p = message_buffer + sizeof(HEADER_T);
   for (j=0; j<size; j++)
      p[j] = drand(256);
   // send dc4 message
   h->flags = 0;
   h->client = client;
   h->size = size;
   h->checksum = checksum(p, size);
   while (1) {
      int server = drand(NUM_SERVERS);
      assert((unsigned)server_num < NUM_SERVERS);
      h->seqnum = sequence_number[server]++;
      success = vchi_msg_queue( dc4_client_state.open_handle[server],
                                message_buffer,
                                size + sizeof(HEADER_T),
                                VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
                                NULL );
      assert( success == 0 );
   }
}
#endif
  printf(CS": dc4_client_send (%d)\n", client);
   while(1)
   {
      switch (drand(256))
      {
         default:
         {
            int size = drand(1024-sizeof(HEADER_T));
            int server_num = drand(NUM_SERVERS);
            HEADER_T *h = (HEADER_T *)message_buffer;
            uint8_t *p = message_buffer + sizeof(HEADER_T);
            for (j=0; j<size; j++)
               p[j] = drand(256);
            // send dc4 message
            assert((unsigned)server_num < NUM_SERVERS);
            h->client = client;
            h->flags = (drand(256) ==0) ? TEST_FLAGS_DELAY:0;
            h->size = size;
            h->seqnum = sequence_number[server_num]++;
            h->checksum = checksum(p, size);
            //printf(CS": dc4_client_send vchi_msg_queue: client=%d size=%d, seq no=%d checksum=0x%08X, sent=0x%02X,0x%02X,0x%02X,0x%02X\n", h->client, h->size, h->seqnum, h->checksum, p[0], p[1], p[2], p[3]);
            //__asm{int 3};
            success = vchi_msg_queue( dc4_client_state.open_handle[server_num],
                                      message_buffer,
                                      size + sizeof(HEADER_T),
                                      VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
                                      NULL );
            assert( success == 0 );
            break;
         }
         case 0:
         {
            msleep(10);
            //os_sleep(100);
            //printf("os_sleep()\n");
            break;
         }
      }
   }
}

/******************************************************************************
NAME
   dc4_server_callback

SYNOPSIS
void dc4_server_callback( void *callback_param,
                                     VCHI_CALLBACK_REASON_T reason,
                                     void *msg_handle )

FUNCTION
   This is passed to the VCHI layer and is called when there is data available
   to read or when a message has been transmitted

RETURNS
   -
******************************************************************************/
static void dc4_server_callback( void *callback_param,
                                  VCHI_CALLBACK_REASON_T reason,
                                  void *msg_handle )
{
   int server_num = (int)callback_param;
//printf(CS": dc4_server_callback (%d, %d)\n", server_num, reason);
   assert((unsigned)server_num < NUM_SERVERS);

   if( reason == VCHI_CALLBACK_MSG_AVAILABLE )
   {
      int32_t status;
//printf(CS": dc4_server_callback: signal index=0x%x\n", index);
      //success = vchi_msg_peek( dc4_server_state.open_handle[i], message_buffer, sizeof(message_buffer), &message_size, VCHI_FLAGS_NONE );
      //if (success == 0)
      {
         status = os_eventgroup_signal( &dc4_server_message_available[server_num], 0 );
         assert( status == 0 );
      }
   }
}

/******************************************************************************
NAME
   dc4_server_process_message

SYNOPSIS
   void dc4_server_process_message( int connection_number, uint8_t * address, int32_t length )

FUNCTION
   Handle the dc4 message we have just read

RETURNS
   -
******************************************************************************/
static void dc4_server_process_message( int connection_number, uint8_t * address, int32_t length, int server_num )
{
   static uint32_t static_dc4_server_seq_no[NUM_SERVERS][NUM_CLIENTS];
   uint32_t *dc4_server_seq_no = static_dc4_server_seq_no[server_num];
   uint8_t message_buffer[BUFFER_SIZE];
   int32_t success;
   HEADER_T *h = (HEADER_T *)address;
   uint8_t *p = address + sizeof(HEADER_T);

//printf(CS": dc4_server_process_message: client/server=%d/%d size=%d, seq no=%d checksum=0x%08X, recv=0x%02X,0x%02X,0x%02X,0x%02X\n", h->client, server_num, h->size, h->seqnum, h->checksum, p[0], p[1], p[2], p[3]);
   assert((unsigned)server_num < NUM_SERVERS);

   if (length >= offsetof(HEADER_T, client))
      assert((unsigned)h->client < NUM_CLIENTS);

   if (length >= offsetof(HEADER_T, seqnum) && dc4_server_seq_no[h->client] != h->seqnum)
   {
      printf(CS": dc4_server_process_message: (%d/%d) expected seq %d, got %d\n", h->client, server_num, dc4_server_seq_no[h->client], h->seqnum);
      dc4_server_state.errors++;
      dc4_server_seq_no[h->client] = h->seqnum;
      //assert(0);
   }
   if (length >= offsetof(HEADER_T, size) && h->size + sizeof(HEADER_T) != length)
   {
      printf(CS": dc4_server_process_message: (%d/%d) expected size %d, got %d\n", h->client, server_num, h->size + sizeof(HEADER_T), length);
      dc4_server_state.errors++;
      assert(0);
   }
#ifndef SPEED_TEST
   if (length >= offsetof(HEADER_T, checksum) && checksum(p, h->size) != h->checksum)
   {
      printf(CS": dc4_server_process_message: (%d/%d) expected checksum 0x%08X, got 0x%08X\n", h->client, server_num, checksum(p, h->size), h->checksum);
      dc4_server_state.errors++;
      assert(0);
   }
#endif

   #if USE_DISPLAY
   {
      char fourcc[] = {'D', 'C', '0'+server_num, CS[0], '\0'};
      update_display_info( MAKE_FOURCC(fourcc), dc4_server_state.errors, VCHI_TEST_SERVER );
   }
   #endif

   if (h->flags & TEST_FLAGS_DELAY)
      msleep(10);//os_sleep(1);

   if (length >= offsetof(HEADER_T, client))
      dc4_server_seq_no[h->client]++;
}


/******************************************************************************
 * CLIENT
 ******************************************************************************/

/******************************************************************************
NAME
   dc4_client_callback

SYNOPSIS
void dc4_client_callback( void *callback_param,
                           VCHI_CALLBACK_REASON_T reason,
                           void *msg_handle )

FUNCTION
   This is passed to the VCHI layer and is called when there is data available
   to read or when a message has been transmitted

RETURNS
   -
******************************************************************************/
static void dc4_client_callback( void *callback_param,
                                  VCHI_CALLBACK_REASON_T reason,
                                  void *msg_handle )
{
   uint32_t event = 1;
   printf(CS": dc4_client_callback (%d)\n", reason);
   assert(0);
}

/******************************************************************************
NAME
   vchi_dc4_client_init

SYNOPSIS
void vchi_dc4_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )

FUNCTION
   Creates the dc4 client and starts the task that handles the entire test (runs continuously)
   the message handler

RETURNS
   -
******************************************************************************/
static void vchi_dc4_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
   int32_t success;
   int i;

   printf(CS":vchi_dc4_client_init\n");
   memset( &dc4_client_state, 0, sizeof(TEST_CLIENT_T) );
   // make sure the display is set up
#if USE_DISPLAY
   vchi_test_display_init();
#endif
   for (i=0; i<NUM_SERVERS; i++)
   {
      char fourcc[] = {'D', 'C', '0'+i, OTHER_CS[0], '\0'};
      SERVICE_CREATION_T service_parameters = { MAKE_FOURCC(fourcc),                              // 4cc service code
                                                connections[0],                                          // passed in fn ptrs
                                                0,                                                       // tx fifo size (unused)
                                                0,                                                       // tx fifo size (unused)
                                                &dc4_client_callback,                                    // service callback
                                                (void *)i}; // callback parameter

      // Create a 'dc4' service on the each of the connections
      success = vchi_service_open(initialise_instance, &service_parameters, &dc4_client_state.open_handle[i]);
      assert( success == 0 );
   }
   for (i=0; i<NUM_CLIENTS; i++)
   {
      char fourcc[] = {'D', 'C', '0'+i, OTHER_CS[0], '\0'};
      success = os_thread_start( &dc4_client_task[i], dc4_client_send, (void *)i, 4000, "DC4_C"CS );
      assert( success == 0 );
   }
}


/******************************************************************************
NAME
   vchi_dc4_server_init

SYNOPSIS
void vchi_dc4_server_init( VCHI_INSTANCE_T initialise_instance )

FUNCTION
   Does the creation of the dc4 server and starts the task that acts as
   the message handler

RETURNS
   -
******************************************************************************/
static void vchi_dc4_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
int32_t success;
int i;

   printf(CS":vchi_dc4_server_init\n");
   memset( &dc4_server_state, 0, sizeof(TEST_SERVER_T) );
   // make sure the display is set up
#if USE_DISPLAY
   vchi_test_display_init();
#endif
   // now the task is running we can register the service with the VCHI interface
   for( i=0; i<NUM_SERVERS; i++ )
   {
      char fourcc[] = {'D', 'C', '0'+i, CS[0], '\0'};
      SERVICE_CREATION_T service_parameters = { MAKE_FOURCC(fourcc),                                    // 4cc service code
                                                connections[0],                                          // passed in fn ptrs
                                                0,                                                       // tx fifo size (unused)
                                                0,                                                       // tx fifo size (unused)
                                                &dc4_server_callback,                                   // service callback
                                                (void *)i};               // callback parameter

      success = os_eventgroup_create( &dc4_server_message_available[i], fourcc );
      assert( success == 0 );

      // Create a 'dc4' service on the each of the connections
      success = vchi_service_create( initialise_instance, &service_parameters, &dc4_server_state.open_handle[i] );
      assert( success == 0 );
   }

   for( i=0; i<NUM_SERVERS; i++ )
   {
      char fourcc[] = {'D', 'C', '0'+i, CS[0], '\0'};
      // create the dc4 task2
      success = os_thread_start( &dc4_server_task[i], dc4_server_recv, (void *)i, 4000, fourcc );
      assert( success == 0 );
   }
}

void vc_dc4_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
   vchi_dc4_client_init(initialise_instance, connections, num_connections );
}
void vc_dc4_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
   vchi_dc4_server_init(initialise_instance, connections, num_connections );
}
