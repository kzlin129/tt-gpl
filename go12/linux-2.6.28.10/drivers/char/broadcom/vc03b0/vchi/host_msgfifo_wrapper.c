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

Project  :  VMCS Host Apps
Module   :  Framework - VMCS
File     :  $RCSfile: vmcs_framework.c,v $
Revision :  $Revision: #9 $

FILE DESCRIPTION
Contains the code to start and run Host applications.
=============================================================================*/

#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/endian.h>
#include <linux/broadcom/vc03/vchi/host_msgfifo_wrapper.h>

/******************************************************************************
Defines
******************************************************************************/



/******************************************************************************
Static data
******************************************************************************/

// info on msgfifo thread
static OS_THREAD_T host_msgfifo_wrapper_thread;
static VCHI_SERVICE_HANDLE_T host_msgfifo_wrapper_handle;
static OS_SEMAPHORE_T host_msgfifo_wrapper_sem;
static OS_SEMAPHORE_T host_msgfifo_dequeue_sem;
static OS_SEMAPHORE_T host_msgfifo_bulk_sem;
static OS_SEMAPHORE_T host_msgfifo_wrapper_read_sem;
static OS_SEMAPHORE_T host_msgfifo_read_in_progress_sem;


/******************************************************************************
Static functions.
******************************************************************************/

static void host_msgfifo_wrapper_callback( void *callback_param, const VCHI_CALLBACK_REASON_T reason, const void *msg_handle );
static int host_msgfifo_wrapper_task(void *argv );

/******************************************************************************
 * CLIENT
 ******************************************************************************/

/* ----------------------------------------------------------------------
 * call this once from the host app to create the long and short
 * services, host-side
 * -------------------------------------------------------------------- */
void
host_vchi_msgfifo_wrapper_init( VCHI_INSTANCE_T initialise_instance,
                                VCHI_CONNECTION_T **connections,
                                uint32_t num_connections )
{
   int32_t success;

   // existing msgfifo is 1:1
   assert( num_connections == 1 );

   // create semaphore to provide atomic reads
   success = os_semaphore_create( &host_msgfifo_wrapper_sem, OS_SEMAPHORE_TYPE_SUSPEND );
   success = os_semaphore_create( &host_msgfifo_dequeue_sem, OS_SEMAPHORE_TYPE_SUSPEND );
   success = os_semaphore_create( &host_msgfifo_bulk_sem,    OS_SEMAPHORE_TYPE_SUSPEND );
   success = os_semaphore_create( &host_msgfifo_read_in_progress_sem,    OS_SEMAPHORE_TYPE_SUSPEND );
   success = os_semaphore_create( &host_msgfifo_wrapper_read_sem, OS_SEMAPHORE_TYPE_SUSPEND );
   os_semaphore_obtain( &host_msgfifo_wrapper_sem );
   os_semaphore_obtain( &host_msgfifo_dequeue_sem );
   os_semaphore_obtain( &host_msgfifo_bulk_sem );

   // Create a 'FIFO' service on the each of the connections
   SERVICE_CREATION_T parameters = { MAKE_FOURCC("FIFO"),            // 4cc service code
                                     connections[0],                 // passed in fn ptrs
                                     0,                              // tx fifo size (unused)
                                     0,                              // tx fifo size (unused)
                                     &host_msgfifo_wrapper_callback, // service callback
                                     &host_msgfifo_wrapper_sem };    // callback parameter

   success = vchi_service_open( initialise_instance, &parameters, &host_msgfifo_wrapper_handle );
   assert( success == 0 );

   success = os_thread_start( &host_msgfifo_wrapper_thread, host_msgfifo_wrapper_task, NULL, 4000, "FIFO_HOST" );
   assert( success == 0 );
}

void host_vchi_msgfifo_wrapper_exit(void)
{
  os_thread_stop(host_msgfifo_wrapper_thread);
  vchi_service_close(host_msgfifo_wrapper_handle);
}

/* ----------------------------------------------------------------------
 * called from the vchi layer whenever an event happens.
 * here, we are only interested in the 'message available' callback
 * -------------------------------------------------------------------- */
static void
host_msgfifo_wrapper_callback( void *callback_param,
                               const VCHI_CALLBACK_REASON_T reason,
                               const void *msg_handle )
{
   int32_t success;
   OS_SEMAPHORE_T *sem;

   switch( reason ) {

   case VCHI_CALLBACK_MSG_AVAILABLE:
     {
       sem = (OS_SEMAPHORE_T *)callback_param;
       if (sem == NULL )
	 {
	   assert(0);
	   break;
	 }
       
       if ( os_semaphore_obtained(sem) ) 
	 {
	   success = os_semaphore_release( sem );
	   assert( success >= 0 );
	 }
      break;
     }

   case VCHI_CALLBACK_BULK_RECEIVED:
   case VCHI_CALLBACK_BULK_SENT:
     {
       sem = (OS_SEMAPHORE_T *)msg_handle;
       if(sem == NULL )
	 {
	   assert(0);
	   break;
	 }
       success = os_semaphore_release( sem );
       assert( success >= 0 );
       break;
     }

   default:
     assert(0);
     break;
   }
   
   return;
}

/* ----------------------------------------------------------------------
 * all this does is sit around wait for the 
 * MSGFIFO_WRAPPER_HOST_ATTENTION_REQUEST message and release the 
 * host message fifo services thread
 * -------------------------------------------------------------------- */

extern OS_SEMAPHORE_T *vcih_obtain_msgfifo_wait_sem (void);

uint8_t host_msgfifo_msg[VCHI_SLOT_SIZE];
//static unsigned char *host_msgfifo_msg;
static uint32_t host_msgfifo_msg_len = 0;
static void *host_msgfifo_msg_handle;

static int host_msgfifo_wrapper_task(void *argv)
{
   int32_t success;
   uint16_t cmd;
   uint8_t message_buffer[VCHI_SLOT_SIZE];
   uint32_t message_size;

   /* For now, use polling mode */
   OS_SEMAPHORE_T *vcih_msgfifo_wait_sem = vcih_obtain_msgfifo_wait_sem();

   for(;;) 
     {
       if(os_thread_should_stop())
	 break;
       
       // wait to receive one or more messages
       os_semaphore_obtain( &host_msgfifo_wrapper_sem );
       
       for (;;) 
	 {
	   // make sure a read is not in progress
	   os_semaphore_obtain( &host_msgfifo_read_in_progress_sem );
	   os_semaphore_release( &host_msgfifo_read_in_progress_sem );
	   
	   memset(message_buffer, 0, sizeof(message_buffer)); 
	   message_size = 0;
	   success = vchi_msg_dequeue(host_msgfifo_wrapper_handle, message_buffer, sizeof(message_buffer), &message_size );

	   //	   success = vchi_msg_peek( host_msgfifo_wrapper_handle, (void *)&(host_msgfifo_msg), &host_msgfifo_msg_len, &host_msgfifo_msg_handle );
	   if ( success != 0 )
	     {
	       break;
	     }
	   
	   //	   cmd = vchi_readbuf_uint16(host_msgfifo_msg);
	   cmd = vchi_readbuf_uint16(message_buffer);
	   
	   switch( cmd ) 
	     {
	     case MSGFIFO_WRAPPER_HOST_ATTENTION_REQUEST:
	       {
		 // release the host message fifo thread
		 //	 vchi_msg_remove( host_msgfifo_wrapper_handle, host_msgfifo_msg_handle );
		 if ( os_semaphore_obtained( vcih_msgfifo_wait_sem ) )
		   success = os_semaphore_release( vcih_msgfifo_wait_sem );
		 
		 break;
	       }
	       
	     case MSGFIFO_WRAPPER_READ_DATA:
	       {
		 memcpy(host_msgfifo_msg, message_buffer, message_size);
		 host_msgfifo_msg_len = message_size;
		 if ( os_semaphore_obtained(&host_msgfifo_dequeue_sem) ) 
		   {
		     success = os_semaphore_obtain( &host_msgfifo_read_in_progress_sem );
		     assert( success >= 0 );
		     success = os_semaphore_release( &host_msgfifo_dequeue_sem );
		     assert( success >= 0 );
		   }
		 
		 break;
	       }
	       
	     default:
		 assert(0);
		 break;
	     }
	 }
     }

   return 0;
}

/* ----------------------------------------------------------------------
 * host has issued a read request; fetch data from videocore
 * -------------------------------------------------------------------- */
void
host_vchi_msgfifo_wrapper_read( unsigned char *host_addr, uint32_t vc_addr, int nbytes )
{
   int32_t success;

   if(vc_addr != VC_SHAREDPTR_ADDR && vc_addr != VC_APP_ADDRESS)
     {
       assert( vc_addr < 65536 );
       assert( vc_addr + nbytes <= 65536 );
     }
   
   memset(host_addr, 0, nbytes);

   //os_logging_message( "HOST: read 0x%p, d'%d", vc_addr, nbytes );
   os_semaphore_obtain( &host_msgfifo_wrapper_read_sem );
   while( nbytes ) {
      unsigned char header[6];
      uint16_t cmd;

      int copy = OS_MIN( nbytes, VCHI_SLOT_SIZE - 16 );

      if(vc_addr != VC_SHAREDPTR_ADDR && vc_addr != VC_APP_ADDRESS)
         vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_READ );
      else if (vc_addr == VC_APP_ADDRESS)
         vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_READ_SPECIFIC_VC_APP );
      else if (vc_addr == VC_SHAREDPTR_ADDR)
         vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_READ_SPECIFIC_VC_SHAREDPTR );
      vchi_writebuf_uint16( &header[2], vc_addr              );
      vchi_writebuf_uint16( &header[4], copy                 );

      vchi_msg_queue( host_msgfifo_wrapper_handle, header, sizeof(header), VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );

      // now wait for response
      os_semaphore_obtain( &host_msgfifo_dequeue_sem );

      // because vchi_msg_peek() doesn't seem to correctly peek at messages 
      // while leaving the messages in the queue, we have to naughtily use 
      // the values obtained by the first peek call in host_msgfifo_wrapper_task() 
      // here.
      //   assert(success==0);
      //      assert(host_msgfifo_msg != 0);
      //      if(success != 0 )
      //return;
      cmd = vchi_readbuf_uint16(host_msgfifo_msg);

      if(MSGFIFO_WRAPPER_READ_DATA == cmd)
      {
         memcpy( host_addr, host_msgfifo_msg+2, host_msgfifo_msg_len - 2);
	 //         success = vchi_msg_remove( host_msgfifo_wrapper_handle, host_msgfifo_msg_handle );
      }
      else
         assert(0);
      
      //      assert( success == 0 );
      assert( host_msgfifo_msg_len - 2 == copy );

      vc_addr += copy;
      host_addr += copy;
      nbytes -= copy;
   }

   memset(host_msgfifo_msg, 0, host_msgfifo_msg_len);
   host_msgfifo_msg_len = 0;
   if ( os_semaphore_obtained( &host_msgfifo_read_in_progress_sem ) ) {
      success = os_semaphore_release( &host_msgfifo_read_in_progress_sem );
      assert( success >= 0 );
   }

   os_semaphore_release( &host_msgfifo_wrapper_read_sem );
}

/* ----------------------------------------------------------------------
 * host is issuing write commands to videocore
 * -------------------------------------------------------------------- */
void
host_vchi_msgfifo_wrapper_write( uint32_t vc_addr, unsigned char *host_addr, int nbytes )
{
   int32_t success;

   assert( vc_addr < 65536 );
   assert( vc_addr + nbytes <= 65536 );

   //os_logging_message( "HOST: write 0x%p -> 0x%p, d'%d", host_addr, vc_addr, nbytes );
   while( nbytes ) {
      VCHI_MSG_VECTOR_T vec[2];
      unsigned char header[6];

      int copy = OS_MIN( nbytes, VCHI_SLOT_SIZE - 16 - sizeof(header) );

      vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_WRITE );
      vchi_writebuf_uint16( &header[2], vc_addr               );
      vchi_writebuf_uint16( &header[4], copy                  );

      vec[0].vec_base = &header[0];
      vec[0].vec_len  = sizeof(header);
      vec[1].vec_base = host_addr;
      vec[1].vec_len  = copy;

      success = vchi_msg_queuev( host_msgfifo_wrapper_handle, &vec[0], 2, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
      assert( success == 0 );

      vc_addr += copy;
      host_addr += copy;
      nbytes -= copy;
   }
}

/* ----------------------------------------------------------------------
 * host is issuing bulk read (data transferred from videocore to host)
 * -------------------------------------------------------------------- */
void
host_vchi_msgfifo_wrapper_bulk_read( unsigned char *host_addr, uint32_t vc_addr, int nbytes )
{
   char tempbuf[16];
   int  overlap;
   int  ael;
   unsigned char *end_addr;

   assert( (((int32_t)host_addr) & 0xf) == 0 );
   assert( (vc_addr & 0xf) == 0 );

   // Can only copy multiples of 16 bytes. So round up, but preserve the
   // area that would otherwise be corrupted.
   ael = (nbytes + 15) & ~15;
   end_addr = host_addr + nbytes;
   overlap = ael - nbytes;

   if ( overlap != 0 )
      memcpy(tempbuf, end_addr, overlap); // preserve overlap

   unsigned char header[14];
   vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_BULK_READ );
   vchi_writebuf_uint16( &header[2], 0 ); // offset = 0
   vchi_writebuf_uint16( &header[4], 0 ); // len = 0

   vchi_writebuf_uint32( &header[6],  vc_addr );
   vchi_writebuf_uint32( &header[10], ael );

   vchi_msg_queue( host_msgfifo_wrapper_handle, &header[0], sizeof(header), VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
   vchi_bulk_queue_receive( host_msgfifo_wrapper_handle,
                            host_addr,
                            ael,
                            VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE,
                            &host_msgfifo_bulk_sem );

   // wait for bulk to be transmitted
   os_semaphore_obtain( &host_msgfifo_bulk_sem );

   // put back overlap
   if ( overlap )
      memcpy(end_addr, tempbuf, overlap);

}

/* ----------------------------------------------------------------------
 * host is issuing bulk write (data transferred from host to videocore)
 * -------------------------------------------------------------------- */
void
host_vchi_msgfifo_wrapper_bulk_write( uint32_t vc_addr, unsigned char *host_addr, int nbytes )
{
   assert( (((int32_t)host_addr) & 0xf) == 0 );
   assert( (vc_addr & 0xf) == 0 );
   assert( (nbytes & 0xf) == 0 );

   unsigned char header[14];
   vchi_writebuf_uint16( &header[0], MSGFIFO_WRAPPER_BULK_WRITE );
   vchi_writebuf_uint16( &header[2], 0 ); // offset = 0
   vchi_writebuf_uint16( &header[4], 0 ); // len = 0

   vchi_writebuf_uint32( &header[6],  vc_addr );
   vchi_writebuf_uint32( &header[10], nbytes );

   vchi_msg_queue( host_msgfifo_wrapper_handle, &header[0], sizeof(header), VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
   vchi_bulk_queue_transmit( host_msgfifo_wrapper_handle,
                             host_addr,
                             nbytes,
                             VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE,
                             &host_msgfifo_bulk_sem );

   // wait for bulk to be transmitted
   os_semaphore_obtain( &host_msgfifo_bulk_sem );
}
