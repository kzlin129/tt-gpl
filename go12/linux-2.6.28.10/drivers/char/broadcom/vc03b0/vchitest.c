/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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

// LONG and SHRT client test
typedef struct {
   int num_connections;
   VCHI_SERVICE_HANDLE_T open_handle[VCHI_MAX_NUM_CONNECTIONS];
   int errors;
} TEST_CLIENT_T;

static OS_THREAD_T    long_thread;
static OS_THREAD_T    short_thread;
static OS_THREAD_T    tstd_thread;

static OS_SEMAPHORE_T long_client_message_available_semaphore;
static OS_SEMAPHORE_T short_client_message_available_semaphore;

static uint8_t long_client_payload[4][1020];
static uint8_t long_server_payload[4][1020];
static uint8_t short_client_payload[4][47];
static uint8_t short_server_payload[4][47];
static TEST_CLIENT_T short_client_state;
static TEST_CLIENT_T long_client_state;

// CLIn client test
typedef struct {
   VCHI_SERVICE_HANDLE_T open_handle[VCHI_MAX_NUM_CONNECTIONS];
   uint32_t              msg_flag[VCHI_MAX_NUM_CONNECTIONS];
   uint16_t              seq_no;
   uint32_t              total_errors;
   fourcc_t              service_id;
   int                   num_connections;
} CLI_SERVICE_T;

#define NSERVICES 10
static CLI_SERVICE_T   cli_client_state[NSERVICES];
static OS_EVENTGROUP_T cli_client_event_group;
static OS_THREAD_T     Cli_client_task;

// TSTD client test
static TEST_CLIENT_T tstd_client_state;
static OS_COUNT_SEMAPHORE_T sema_tstdblk;
#define TSTDPAYLOAD_MAX (1*1024*1024 + 0x10)
static uint8_t* tstd_tx_payload = NULL;
static uint8_t* tstd_rx_payload = NULL;
#define RANDSEED_TSTD 99
int random_number_tstd[2] = {RANDSEED_TSTD, RANDSEED_TSTD};

// function prototypes
static void     cli_client_callback( void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle );
static int cli_client_task( void *argv );
void vchi_cli_client_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections);/*
 *
 */
int  vchi_test0(vc03drv_t* drv);
int  vchi_test1(vc03drv_t* drv);
int  vchi_test2(vc03drv_t* drv);
int  vc_do_vchitest(vc03drv_t* drv, char* cmd);
int  long_client_task(void *argv);
void long_client_callback(void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle);
void vchi_long_client_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections);

void vchi_short_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections );
int  short_client_task(void *argv );
void short_client_callback( void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle );

void vchi_tstd_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections );
int  tstd_client_task(void *argv );
void tstd_client_callback(void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle);

extern void vc_dc4_client_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections);
extern void vc_dc4_server_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections);

// TSTB server test
typedef struct {
   VCHI_SERVICE_HANDLE_T open_handle[VCHI_MAX_NUM_CONNECTIONS];
   int                   server_number;
   uint32_t              connection_number[VCHI_MAX_NUM_CONNECTIONS];
   uint32_t              total_errors[VCHI_MAX_NUM_CONNECTIONS];
   int                   num_connections;
} TSTB_SERVER_T;

#define RANDSEED_TSTB   97
#define MAXPAYLOAD_TSTB (1024*1024)

static OS_EVENTGROUP_T  tstb_server_event_group;
static OS_THREAD_T      Tstb_server_task;
static TSTB_SERVER_T    tstb_server_state;
static int              random_number_tstb[2] = {RANDSEED_TSTB, RANDSEED_TSTB};

static void vchi_tstb_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections );
static int  tstb_server_task(void *argv );
static void tstb_server_callback(void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle);

// common
static void initialise_test_data_long( void );
static void initialise_test_data_short( void );
static void initialise_test_data_tstd( void );


int longclient_done = 0;
int shortclient_done = 0;
int cliclient_done = 0;
int tstdclient_done = 0;
#define ITERMAX (1024*128)

/*
 *
 */
int vc_do_vchitest(vc03drv_t* drv, char* cmd)
{ 
  int testnum;

  if(1 != sscanf(cmd, "%d", &testnum))
    {
      VC_DEBUG(Trace, "error: invalid command: cmd=%s\n", cmd);
      return -1;
    }
  
  if(1 == testnum)
    {
      vchi_test1(drv);
    }
  else if(2 == testnum)
    {
      vchi_test2(drv);      
    }
  else if (0 == testnum)
    {
      vchi_test0(drv);
    }
  else
    {
      VC_DEBUG(Trace, "error: invalid command: cmd=%s\n", cmd);
      return -1;
    }
  return 0;
}

/*
 *
 */
int vchi_test0(vc03drv_t* drv)
{
    
    vc_dc4_client_init(drv->instance_vchi, drv->connection_vchi + 0, 1);
    return 0;
}

int vchi_test1(vc03drv_t* drv)
{
  // vchi init is in vc03_msgfifo_init, at driver reset
  
  assert(drv->initialized == 1);

  vchi_long_client_init(drv->instance_vchi, drv->connection_vchi + 0, 1);
  //vchi_short_client_init(drv->instance_vchi, drv->connection_vchi + 0, 1);
  //vchi_cli_client_init(drv->instance_vchi, drv->connection_vchi + 0, 1);
  vchi_tstd_client_init(drv->instance_vchi, drv->connection_vchi + 0, 1);
  //vchi_tstb_server_init(drv->instance_vchi, drv->connection_vchi + 0, 1);

  return 0;
}


/******************************************************************************
NAME
   vchi_long_client_init

SYNOPSIS
void vchi_long_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )

FUNCTION
   Creates the LONG client and starts the task that handles the entire test (runs continuously)
   the message handler

RETURNS
   -
******************************************************************************/
void vchi_long_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
  int32_t success;
  int i;
  int ret;  
  
  memset( &long_client_state, 0, sizeof(TEST_CLIENT_T) );
  // record the number of connections
  long_client_state.num_connections = num_connections;
  // set up our fixed pattern payloads
  initialise_test_data_long();
  // make sure the display is set up
  // set up the semaphore that will be used to signal incoming messages
  success = os_semaphore_create( &long_client_message_available_semaphore, OS_SEMAPHORE_TYPE_BUSY_WAIT );
  assert( success == 0 );
  // we must have obtained it otherwise we will not see messages
  success = os_semaphore_obtain( &long_client_message_available_semaphore );
  assert( success == 0 );
  
  for( i=0; i<num_connections; i++ )
    {
      SERVICE_CREATION_T service_parameters = { MAKE_FOURCC("LONG"),                                     // 4cc service code
                                                connections[i],                                          // passed in fn ptrs
                                                0,                                                       // tx fifo size (unused)
                                                0,                                                       // tx fifo size (unused)
                                                &long_client_callback,                                   // service callback
                                                (const void *)&long_client_message_available_semaphore}; // callback parameter
      
      // Create a 'LONG' service on the each of the connections
      success = vchi_service_open(initialise_instance, &service_parameters, &long_client_state.open_handle[i]);
      assert( success == 0 );
   }
  
  // allocate the stack for the long client task
   // start the long task
  ret = os_thread_start(&long_thread, long_client_task, NULL, 0, "long_client");
  
  
}

//copy from single.c
// This should come from message driver...
#define VCHI_SLOT_SIZE (VCHI_MAX_MSG_SIZE+16)

/******************************************************************************
NAME
   long_client_task

SYNOPSIS
   void long_client_task(UNSIGNED argc, VOID *argv)

FUNCTION
   Sends a message, waits for reply, checks for errors, repeat over each connection
   and repeat continuously

RETURNS
   -
******************************************************************************/
int long_client_task(void *argv)
{
int32_t success;
uint8_t message_buffer[VCHI_SLOT_SIZE];
uint32_t message_size;
int i;
uint32_t sequence_number = 0;
uint16_t comparison_result[VCHI_MAX_NUM_CONNECTIONS];
uint16_t long_vc_seq_no;
uint16_t payload_index;
uint16_t expected_vc_seq_no = 0;
uint16_t host_error;

 longclient_done = 0;
 for( i=0; i<VCHI_MAX_NUM_CONNECTIONS; i++ )
   comparison_result[i] = 1;
 
 while(1)
   {
      for( i=0; i<long_client_state.num_connections; i++ )
      {
         // form the message
         vchi_writebuf_uint16( &message_buffer[0], sequence_number );
         vchi_writebuf_uint16( &message_buffer[2], comparison_result[i] );
         memcpy( &message_buffer[4], long_client_payload[sequence_number & 0x3], 1020 );

         // send it
         success = vchi_msg_queue( long_client_state.open_handle[i], message_buffer, 1024, /*VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE*/
				    VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE | VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);
        if( success != 0 )
         {
            long_client_state.errors++;
	    assert(0);
         }

         // wait for the reply
         success = os_semaphore_obtain( &long_client_message_available_semaphore );
         assert( success == 0 );

         // read the message
         success = vchi_msg_dequeue( long_client_state.open_handle[i], message_buffer, sizeof(message_buffer), &message_size,VCHI_FLAGS_NONE);

         // verify that the message is valid
         long_vc_seq_no = vchi_readbuf_uint16( message_buffer );
         payload_index = long_vc_seq_no & 0x3;
         host_error = vchi_readbuf_uint16( &message_buffer[2] );
         if(host_error == 0)
         {
            long_client_state.errors++;
	    //  assert(0);
         }

         if(expected_vc_seq_no != long_vc_seq_no)
         {
            long_client_state.errors++;
	    //    assert(0);
         }

         if(( message_size == 1024 ) && ( memcmp( &message_buffer[4], long_server_payload[payload_index], 1020 ) == 0 ))
            comparison_result[i] = 1;
         else
         {
            long_client_state.errors++;
	    //  assert(0);
         }
	 //	 os_delay(10);
	 msleep(500);
      }
      // move on to the next pattern in the sequence
      sequence_number++;
      expected_vc_seq_no--;


   }

   VC_DEBUG(Trace, "long client task complete, iter=%d errors=%d\n", sequence_number, long_client_state.errors);
   
   for( i=0; i<long_client_state.num_connections; i++ )
     {
       VC_DEBUG(Trace, "i=%d comp_res=%d\n", i, comparison_result[i]);
       vchi_service_close(long_client_state.open_handle[i]);
     }

   longclient_done = 1;
   return 0;
}


/******************************************************************************
NAME
   long_client_callback

SYNOPSIS
void long_client_callback( void *callback_param,
                           const VCHI_CALLBACK_REASON_T reason,
                           const void *msg_handle )

FUNCTION
   This is passed to the VCHI layer and is called when there is data available
   to read or when a message has been transmitted

RETURNS
   -
******************************************************************************/
void long_client_callback( void *callback_param,
                                  const VCHI_CALLBACK_REASON_T reason,
                                  const void *msg_handle )
{
  OS_SEMAPHORE_T *semaphore = (OS_SEMAPHORE_T *)callback_param;
  
  if( reason == VCHI_CALLBACK_MSG_AVAILABLE )
    {
      if( semaphore != NULL )
	{
	  os_semaphore_release( semaphore );
	}
    }
}

static void initialise_test_data_long( void )
{
int i,j;
 static int initialised = VC_FALSE;
 
 if( initialised == VC_FALSE )
   {
      initialised = VC_TRUE;
      for( i=0; i<4; i++ )
	{
	  for( j=0; j<1020; j++ )
	    {
	      long_client_payload[i][j] = (uint8_t)(i+j+64);
	      long_server_payload[i][j] = (uint8_t)(i+j);
         }
	}
   }
}


/******************************************************************************
 * CLIENT
 ******************************************************************************/

/******************************************************************************
NAME
   vchi_short_client_init

SYNOPSIS
void vchi_short_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )

FUNCTION
   Creates the SHRT client and starts the task that handles the entire test (runs continuously)
   the message handler

RETURNS
   -
******************************************************************************/
void vchi_short_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
int32_t success;
int i;

   memset( &short_client_state, 0, sizeof(TEST_CLIENT_T) );
   // record the number of connections
   short_client_state.num_connections = num_connections;
   // set up our fixed pattern payloads
   initialise_test_data_short();
   // make sure the display is set up
   // set up the semaphore that will be used to signal incoming messages
   success = os_semaphore_create( &short_client_message_available_semaphore, OS_SEMAPHORE_TYPE_BUSY_WAIT );
   assert( success == 0 );
   // we must have obtained it otherwise we will not see messages
   success = os_semaphore_obtain( &short_client_message_available_semaphore );
   assert( success == 0 );

   for( i=0; i<num_connections; i++ )
   {
      SERVICE_CREATION_T service_parameters = { MAKE_FOURCC("SHRT"),                                      // 4cc service code
                                                connections[i],                                           // passed in fn ptrs
                                                0,                                                        // tx fifo size (unused)
                                                0,                                                        // tx fifo size (unused)
                                                &short_client_callback,                                   // service callback
                                                (const void *)&short_client_message_available_semaphore}; // callback parameter

      // Create a 'SHRT' service on the each of the connections
      success = vchi_service_open( initialise_instance, &service_parameters, &short_client_state.open_handle[i] );
      assert( success == 0 );
   }

  os_thread_start(&short_thread, short_client_task, NULL, 0, "short_client");
}


/******************************************************************************
NAME
   short_client_task

SYNOPSIS
   void short_client_task(UNSIGNED argc, VOID *argv)

FUNCTION
   Sends a message, waits for reply, checks for errors, repeat over each connection
   and repeat continuously

RETURNS
   -
******************************************************************************/
int short_client_task(void *argv)
{
  int32_t success;
  uint8_t message_buffer[VCHI_SLOT_SIZE];
  uint32_t message_size;
  int i;
  uint32_t sequence_number = 0;
  uint16_t comparison_result[VCHI_MAX_NUM_CONNECTIONS];
  uint16_t short_vc_seq_no;
  uint16_t payload_index;
  uint16_t expected_vc_seq_no = 0;
  uint16_t host_error;
  
  shortclient_done = 0;
  for( i=0; i<VCHI_MAX_NUM_CONNECTIONS; i++ )
      comparison_result[i] = 1;

   while(1)
   {
      for( i=0; i<short_client_state.num_connections; i++ )
      {
         // form the message
         vchi_writebuf_uint16( &message_buffer[0], sequence_number );
         vchi_writebuf_uint16( &message_buffer[2], comparison_result[i] );
         memcpy( &message_buffer[4], short_client_payload[sequence_number & 0x3], 47 );

         // send it
         success = vchi_msg_queue( short_client_state.open_handle[i], message_buffer, 51, 
				   VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE | VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
         if( success != 0 )
         {
            short_client_state.errors++;
	    assert(0);
         }

         // wait for the reply
         success = os_semaphore_obtain( &short_client_message_available_semaphore );
         assert( success == 0 );

         // read the message
         success = vchi_msg_dequeue( short_client_state.open_handle[i], message_buffer, sizeof(message_buffer), &message_size,VCHI_FLAGS_NONE  );

         // verify that the message is valid
         short_vc_seq_no = vchi_readbuf_uint16( message_buffer );
         payload_index = short_vc_seq_no & 0x3;
         host_error = vchi_readbuf_uint16( &message_buffer[2] );
         if( host_error == 0 )
         {
            short_client_state.errors++;
	    //      assert(0);
         }

         if( expected_vc_seq_no != short_vc_seq_no )
         {
            short_client_state.errors++;
	    //      assert(0);
         }

         if(( message_size == 47 ) && ( memcmp( &message_buffer[4], short_server_payload[payload_index], 47 ) == 0 ))
            comparison_result[i] = 1;
         else
         {
            short_client_state.errors++;
	    //       assert(0);
         }
      }
      // move on to the next pattern in the sequence
      sequence_number++;
      expected_vc_seq_no--;
      //  os_delay(10);
	 
   }

   VC_DEBUG(Trace, "short client task complete, iter=%d errors=%d\n", sequence_number, long_client_state.errors);
   
   for( i=0; i<long_client_state.num_connections; i++ )
     {
       VC_DEBUG(Trace, "i=%d comp_res=%d\n", i, comparison_result[i]);
       vchi_service_close(short_client_state.open_handle[i]);
     }

   shortclient_done = 1;
   return 0;
}


/******************************************************************************
NAME
   short_client_callback

SYNOPSIS
void short_client_callback( void *callback_param,
                            const VCHI_CALLBACK_REASON_T reason,
                            const void *msg_handle )

FUNCTION
   This is passed to the VCHI layer and is called when there is data available
   to read or when a message has been transmitted

RETURNS
   -
******************************************************************************/
void short_client_callback( void *callback_param,
                                   const VCHI_CALLBACK_REASON_T reason,
                                   const void *msg_handle )
{
  OS_SEMAPHORE_T *semaphore = (OS_SEMAPHORE_T *)callback_param;

   if( reason == VCHI_CALLBACK_MSG_AVAILABLE )
   {
      if( semaphore != NULL )
      {
	 os_semaphore_release( semaphore );
      }
   }
}


/******************************************************************************
NAME
   initialise_test_data

SYNOPSIS
   void initialise_test_data( void )

FUNCTION
   Initialise the payload data for both ends of the connection

RETURNS
   -
******************************************************************************/
static void initialise_test_data_short( void )
{
int i,j;
static int initialised = VC_FALSE;

   if( initialised == VC_FALSE )
   {
      initialised = VC_TRUE;
      for( i=0; i<4; i++ )
      {
         for( j=0; j<47; j++ )
         {
            short_client_payload[i][j] = (uint8_t)(i+j+23);
            short_server_payload[i][j] = (uint8_t)(i+j);
         }
      }
   }
}


/*
 *
 */

static int random_number[2*NSERVICES];

/******************************************************************************
 * CLIENTS
 ******************************************************************************/
static uint16_t get_random( int sequence )
{
   random_number[sequence] = random_number[sequence] * 1103515245 + 12345;
   return (uint16_t)(random_number[sequence]/65536) % 32768;
}


/******************************************************************************
NAME
   vchi_cli_server_init

SYNOPSIS
void vchi_cli_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )

FUNCTION
   Creates the 10 clients

RETURNS
   -
******************************************************************************/
void vchi_cli_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
  int32_t success;
  int i,j;
  char service_name[5];
  
   memset( &cli_client_state[0], 0, sizeof(CLI_SERVICE_T)*NSERVICES );
   for( i=0; i<NSERVICES; i++ )
   {
      // record the number of connections
      cli_client_state[i].num_connections = num_connections;
      for( j=0; j<num_connections; j++ )
	{
	  //	  cli_client_state[i].msg_flag[j] = 1 << (i*VCHI_MAX_NUM_CONNECTIONS + j);
	  cli_client_state[i].msg_flag[j] =  (i*VCHI_MAX_NUM_CONNECTIONS + j);
	}
      // seed the random number generators
      random_number[i] = i+99;
      random_number[i+NSERVICES] = i+99;
   }

   // make sure the display is set up

   // create the event group that will tell us which service has sent us a message
   os_eventgroup_create( &cli_client_event_group, "CLI_CLIENT_EVT" );


   // register the NSERVICES client with the VCHI interface
   for( j=0; j<NSERVICES; j++ )
   {
      sprintf( service_name,"CLI%d", j );
      cli_client_state[j].service_id = MAKE_FOURCC(service_name);
      for( i=0; i<num_connections; i++ )
      {
         SERVICE_CREATION_T service_parameters = { cli_client_state[j].service_id,                  // 4cc service code
                                                   connections[i],                                  // passed in fn ptrs
                                                   0,                                               // tx fifo size (unused)
                                                   0,                                               // tx fifo size (unused)
                                                   &cli_client_callback,                            // service callback
                                                   (const void *)&cli_client_state[j].msg_flag[i]}; // callback parameter

         // Create a 'LONG' service on the each of the connections
         success = vchi_service_open( initialise_instance, &service_parameters, &cli_client_state[j].open_handle[i] );
         assert( success == 0 );
      }
   }

   // allocate the stack for the client task
   os_thread_start(&Cli_client_task, cli_client_task, NULL, 0, (char*)"CLI_CLIENT");
}


/******************************************************************************
NAME
   cli_client_callback

SYNOPSIS
   void cli_client_callback( void *callback_param,
                             const VCHI_CALLBACK_REASON_T reason,
                             const void *msg_handle )

FUNCTION
   Callback for the clients

RETURNS
   -
******************************************************************************/
static void cli_client_callback( void *callback_param,
                                 const VCHI_CALLBACK_REASON_T reason,
                                 const void *msg_handle )
{
   uint32_t *event = (uint32_t *)callback_param;

   // if there is a message set the passed in event
   if( reason == VCHI_CALLBACK_MSG_AVAILABLE )
   {
     os_eventgroup_signal( &cli_client_event_group, *event ); 
   }
}


/******************************************************************************
NAME
   cli_client_task

SYNOPSIS
   void cli_client_task( UNSIGNED argc, VOID *argv )

FUNCTION
   Task to handle all the required client activity

RETURNS
   -
******************************************************************************/
static int cli_client_task(void *argv )
{
  int i,j,k;
  int32_t success;
  uint8_t message_buffer[VCHI_SLOT_SIZE];
  int msg_length;
  uint8_t * msg_address;
  uint32_t actual_msg_size;
  void * message_handle;
  uint32_t events;
  int  iter;
  
  cliclient_done = 0;
  for(iter=0; iter < ITERMAX / NSERVICES; ++iter)
    {
      for( i=0; i<NSERVICES; i++ )
	{
	  for( j=0; j<cli_client_state[i].num_connections; j++ )
	    {
	      //            os_delay(10);
	      // form the message
	      memset( message_buffer, 0, VCHI_SLOT_SIZE );
	      msg_length = get_random(i+10)*1022 / 32767;
	      assert( msg_length <= 1022 );
	      vchi_writebuf_uint16( message_buffer, cli_client_state[i].seq_no );
	      // move on our expected sequence number
	      cli_client_state[i].seq_no++;
	      for( k=0; k<msg_length; k+=2 )
		vchi_writebuf_uint16( &message_buffer[2+k], get_random(i+10) );
	      // send a message
	      success = vchi_msg_queue( cli_client_state[i].open_handle[j],
                                      message_buffer,
					msg_length+2,
					VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE | VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
					NULL );
	      // wait for the reply
	      os_eventgroup_retrieve( &cli_client_event_group, &events);
	      
	      // make sure that we got the message we expected
	      if( events & (1 << (i*VCHI_MAX_NUM_CONNECTIONS + j )))
		{
		  // verify the data is correct
		  success = vchi_msg_peek( cli_client_state[i].open_handle[j],
					   (void **)&msg_address,
					   &actual_msg_size,
					    VCHI_FLAGS_NONE );
		  assert(success == 0);
		  // check the message length matches
		  if( actual_msg_size != msg_length+2 )
		    {
		      // record the error
		      cli_client_state[i].total_errors++;
		      assert(0);
		    }
		  // check the data matches, record total failures
		  for( k=0; k<msg_length+2; k++ )
		    {
		      if( msg_address[k] != message_buffer[k] )
			{
			  // record the error
			  cli_client_state[i].total_errors++;
			  assert(0);
			}
		    }
		  // remove the message
		  success = vchi_msg_remove( cli_client_state[i].open_handle[j]);
		  assert(success == 0);
		}
	      else
		{
		  assert(0);
		  // record the erro
		  cli_client_state[i].total_errors++;
		  assert(0);
		}
	    }
	}
      
      //   os_delay(10);
      
    }

  for(i = 0; i < NSERVICES; i++)
    {
      vchi_service_close(cli_client_state[i].open_handle[0]);
    }
  
  cliclient_done = 1;
  return 0;
}

/*
 *
 */
static uint16_t get_random_tstd(int sequence)
{
   random_number_tstd[sequence] = random_number_tstd[sequence] * 1103515245 + 12345;
   return (uint16_t)(random_number_tstd[sequence]/65536) % 32768;
}

void initialise_test_data_tstd(void)
{
  int i;
  uint16_t* ptr;
 
  if(NULL == tstd_tx_payload)
    {
      tstd_tx_payload = vmalloc(TSTDPAYLOAD_MAX);
      assert(tstd_tx_payload);
    }

  ptr = (uint16_t*)tstd_tx_payload;
  
  for(i = 0; i < TSTDPAYLOAD_MAX / 2; i++)
    {
      ptr[i] = get_random_tstd(0);
    }

  if(NULL == tstd_rx_payload)
    {
      tstd_rx_payload = vmalloc(TSTDPAYLOAD_MAX);
      assert(tstd_rx_payload);
    }

  memset(tstd_rx_payload, 0, TSTDPAYLOAD_MAX);

}

/*
 *
 */
void vchi_tstd_client_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
  int32_t success;
  int i;
  int ret;  
  
  memset( &tstd_client_state, 0, sizeof(TEST_CLIENT_T) );
  // record the number of connections
  tstd_client_state.num_connections = num_connections;
  // set up our fixed pattern payloads
  initialise_test_data_tstd();
  // make sure the display is set up
  // set up the semaphore that will be used to signal incoming messages
  success = os_count_semaphore_create( &sema_tstdblk, 0, OS_SEMAPHORE_TYPE_BUSY_WAIT );
  assert( success == 0 );
  
  for( i=0; i<num_connections; i++ )
    {
      SERVICE_CREATION_T service_parameters = { MAKE_FOURCC("TSTD"),                                     // 4cc service code
                                                connections[i],                                          // passed in fn ptrs
                                                0,                                                       // tx fifo size (unused)
                                                0,                                                       // tx fifo size (unused)
                                                &tstd_client_callback,                                   // service callback
                                                0}; // callback parameter
      
      // Create a 'LONG' service on the each of the connections
      success = vchi_service_open(initialise_instance, &service_parameters, &tstd_client_state.open_handle[i]);
      assert( success == 0 );
   }
  
  // allocate the stack for the long client task
   // start the long task
  ret = os_thread_start(&tstd_thread, tstd_client_task, NULL, 0, "tstd_client");
  
}

/*
 *
 */
void tstd_client_callback( void *callback_param,
			   const VCHI_CALLBACK_REASON_T reason,
			   const void *msg_handle )
{
  int ret;
  
  switch( reason ) 
    {
      
    case VCHI_CALLBACK_MSG_AVAILABLE:
      {
	uint8_t msg[32];
	uint32_t msgsz;

	ret = vchi_msg_dequeue(tstd_client_state.open_handle[0], msg, sizeof(msg), &msgsz, VCHI_FLAGS_NONE );
	break;
      }
      
    case VCHI_CALLBACK_BULK_RECEIVED:
    case VCHI_CALLBACK_BULK_SENT:
      {
	OS_COUNT_SEMAPHORE_T * sem = (OS_COUNT_SEMAPHORE_T *)msg_handle;
	ret = os_count_semaphore_release( sem );
	assert( ret >= 0 );
	break;
      }

    default:
      assert(0);
      break;
    }
}

/*
 *
 */
int tstd_client_task(void *argv)
{
  unsigned char header[14];
  int iter;
  int txlen, rxlen;

  txlen = TSTDPAYLOAD_MAX ;
  rxlen = TSTDPAYLOAD_MAX - 0x10;
  iter = 1;
  
  while(1)
    {
      // transmit
      //   VC_DEBUG(Trace, "%d) len=0x%08X\n", iter, len); 
      memset(header, 0, sizeof(header));
      vchi_writebuf_uint16(header + 0, 0 ); // vc3 cmd -- BULK RX
      vchi_writebuf_uint32(header + 2, RANDSEED_TSTD); //  rand seed
      vchi_writebuf_uint32(header + 6, txlen); // len   
      vchi_msg_queue(tstd_client_state.open_handle[0], &header[0], sizeof(header), VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
      vchi_bulk_queue_transmit(tstd_client_state.open_handle[0],
			       tstd_tx_payload,
			       txlen,
			       /*VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE,*/
			       VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
			       &sema_tstdblk);
      //os_count_semaphore_obtain( &sema_tstdblk, 1);     
      //os_sleep(100);
      msleep(10);

#if 1     
      // receive      
      memset(header, 0, sizeof(header));
      vchi_writebuf_uint16(header + 0, 2); // vc3 cmd -- BULK TX
      vchi_writebuf_uint32(header + 2, RANDSEED_TSTD); //  rand seed
      vchi_writebuf_uint32(header + 6, rxlen);
      vchi_msg_queue(tstd_client_state.open_handle[0], &header[0], sizeof(header), VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
      vchi_bulk_queue_receive(tstd_client_state.open_handle[0], 
			      tstd_rx_payload, rxlen,
			      /*VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE,*/
			      VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
			      &sema_tstdblk);
      //os_count_semaphore_obtain( &sema_tstdblk, 1);
      //os_sleep(100);
      msleep(10);
#endif
      
      ++iter;
    }

  tstdclient_done = 1;
  
  VC_DEBUG(Trace, "TSTD client finished\n");
  
  return 0;
}

/*
 *
 */
/******************************************************************************
NAME
   vchi_tstb_server_init

SYNOPSIS
   void vchi_tstb_server_init( VCHI_INSTANCE_T initialise_instance,
                               VCHI_CONNECTION_T ** connections,
                               uint32_t num_connections)

FUNCTION
   Start up routine for the bulk receive service

RETURNS
   -
******************************************************************************/
void vchi_tstb_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections )
{
  int32_t success;
  int i;

   memset( &tstb_server_state, 0, sizeof(TSTB_SERVER_T) );

   // record the number of connections
   tstb_server_state.num_connections = num_connections;

   // create the event group that will tell us which service has sent us a message
   os_eventgroup_create(&tstb_server_event_group, "TSTB_EVT" );

   // now the task is running we can register the service with the VCHI interface
   for( i=0; i<num_connections; i++ )
   {
     tstb_server_state.connection_number[i] = i;

     {
         SERVICE_CREATION_T service_parameters = { MAKE_FOURCC("TSTB"),                             // 4cc service code
                               connections[i],                                  // passed in fn ptrs
                               0,                                               // tx fifo size (unused)
                               0,                                               // tx fifo size (unused)
                               &tstb_server_callback,                                  // service callback
                               (const void *)&tstb_server_state.connection_number[i]}; // callback parameter
    
         // Create a 'TSTB' service on the each of the connections
         success = vchi_service_create( initialise_instance, &service_parameters, &tstb_server_state.open_handle[i] );
         assert( success == 0 );
     }
   }

   // allocate the stack for the tstb server task
   // start the server message handling task
   os_thread_start(&Tstb_server_task, tstb_server_task, NULL, 0, "TSTB_SERVER");

}

/*
 *
 */
static uint16_t get_random_tstb(int sequence)
{
   random_number_tstb[sequence] = random_number_tstb[sequence] * 1103515245 + 12345;
   return (uint16_t)(random_number_tstb[sequence]/65536) % 32768;
}

/******************************************************************************
NAME
   tstb_callback

SYNOPSIS
   void tstb_callback( void *callback_param,
                       const VCHI_CALLBACK_REASON_T reason,
                       const void *msg_handle )

FUNCTION
   Callback for the tstb service

RETURNS
   -
******************************************************************************/
static void tstb_server_callback( void *callback_param,
                           const VCHI_CALLBACK_REASON_T reason,
                           const void *msg_handle )
{
  uint32_t connection_number = *(uint32_t *)callback_param;
  uint32_t *handle = (uint32_t *)msg_handle;

  switch(reason)
    {
    case VCHI_CALLBACK_MSG_AVAILABLE:
      {
	// inform the task that a message has arrived
	os_eventgroup_signal(&tstb_server_event_group, connection_number+0*VCHI_MAX_NUM_CONNECTIONS);
	break;
      }

    case VCHI_CALLBACK_BULK_RECEIVED:
   // inform the task that the requested data has now arrived
      {
	os_eventgroup_signal(&tstb_server_event_group, connection_number+1*VCHI_MAX_NUM_CONNECTIONS);
	assert( *handle == connection_number );
	break;
      }
    default:
      assert(0);
      break;
    }
}


/******************************************************************************
NAME
   tstb_task

SYNOPSIS
   void tstb_task( UNSIGNED argc, VOID *argv )

FUNCTION
   Task to handle all the required tstb activity

RETURNS
   -
******************************************************************************/
static int tstb_server_task(void *argv)
{
  uint8_t * msg_address;
  uint32_t actual_msg_size;// TSTB server test
  void * message_handle;
  int32_t success;
  int i,j;
  int seed[VCHI_MAX_NUM_CONNECTIONS];
  int tstb_length[VCHI_MAX_NUM_CONNECTIONS];
  int rx_handle[VCHI_MAX_NUM_CONNECTIONS];
  uint8_t *buffer[VCHI_MAX_NUM_CONNECTIONS];
  uint8_t message[VCHI_SLOT_SIZE];
  uint32_t events;

  for( i=0; i<tstb_server_state.num_connections; i++ )
    {
      buffer[i] = vmalloc(MAXPAYLOAD_TSTB);
      rx_handle[i] = i;
    }

  // signal the server has started
  memset(message, 0, sizeof(message));
  vchi_writebuf_uint16(message + 0, 0/*2*/); // command
  vchi_writebuf_uint32(message + 2, 0);
  vchi_writebuf_uint32(message + 6, MAXPAYLOAD_TSTB/*0*/);
  // send it
  for( i=0; i<tstb_server_state.num_connections; i++ )
    {
      success = vchi_msg_queue(tstb_server_state.open_handle[i], 
			       message, 10, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
    }

  for(;;)
    {
      os_eventgroup_retrieve(&tstb_server_event_group, &events);
      
      while(events)
	{
	  // check for messages
	  for( i=0; i<tstb_server_state.num_connections; i++ )
	    {
	      if( events & (1<<i) )
		{
		  // remove this bit from the events variable
		  events &= ~(1<<i);
		  // get the address of the message
		  success = vchi_msg_peek( tstb_server_state.open_handle[i], (void **)&msg_address, &actual_msg_size,  VCHI_FLAGS_NONE );
		  assert(success == 0);
		  // check that it is command 0
		  assert(vchi_readbuf_uint16( msg_address ) == 0);
		  // read the seed for the random number generator for the transfer
		  seed[i] = vchi_readbuf_uint32( msg_address+2 );
		  tstb_length[i] = vchi_readbuf_uint32( msg_address+6 );
		  assert( tstb_length[i] <= MAXPAYLOAD_TSTB );
		  // remove the message
		  success = vchi_msg_remove( tstb_server_state.open_handle[i]);
		  assert(success == 0);
		  // set up the bulk rx
		  success = vchi_bulk_queue_receive(tstb_server_state.open_handle[i], 
						    buffer[i], 
						    tstb_length[i], 
						    VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE, 
						    &rx_handle[i]);
		  assert(success == 0);
		  // os_delay(100);
		  
		}
	    }
	  
	  // check for data received
	  for( i=0; i<VCHI_MAX_NUM_CONNECTIONS; i++ )
	    {
	      if( events & (1<< (i+VCHI_MAX_NUM_CONNECTIONS)) )
		{
		  events &= ~(1<< (i+VCHI_MAX_NUM_CONNECTIONS));
		  
		  // the reception of the buffer has completed so now we must check it
		  random_number_tstb[i] = seed[i];
		  for( j=0; j<tstb_length[i]; j+=2 )
		    {
		      uint16_t val1, val2;
		      val1 = vchi_readbuf_uint16(&buffer[i][j]);
		      val2 = get_random_tstb(0);

		      if(val1 != val2)
			{
			  tstb_server_state.total_errors[i]++;
			}
		    }
		  vchi_writebuf_uint16( &message[0],1 );
		  vchi_writebuf_uint32( &message[2],tstb_server_state.total_errors[i] );
		  // send the message indicating that we have processed the data and how many errors were found
		  success = vchi_msg_queue( tstb_server_state.open_handle[i], message, 6, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);
		  assert(success == 0);
		  // display the error count
		}
	    }
	}
    }
}



/*
 * Gencmd test
 */
int vchi_test2(vc03drv_t* drv)
{
  //int ret;
  //char response[0xff];
  //const char* cmd = "version";
  //int i;
  (void)drv;

  assert(drv->initialized == 1);
  printk("vchi_test2 not implemented\n");
#if 0
  for(i = 0; i < 1000; i++)
    {
      ret = vc_gencmd(response, 255, cmd);

      if(ret<0)
	{
	  printk("%03d) vc_gencmd error, cmd=%s\n", i, cmd);
	}
      else
	{
	  printk("%03d) cmd=%s response=%s\n", i, cmd, response);
	}
    }
#endif
  return 0;
}

