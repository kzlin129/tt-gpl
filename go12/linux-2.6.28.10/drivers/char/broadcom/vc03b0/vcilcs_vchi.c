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

Project  :  VMCS-X
Module   :  OpenMAX IL Component Service
File     :  $RCSfile: ilcs.c,v $
Revision :  $Revision: 1.1.2.1 $

FILE DESCRIPTION
OpenMAX IL Component Service functions
=============================================================================*/

/* System includes */

#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vcmsgfifo.h>
#include <linux/broadcom/omx/OMX_Core.h>
#include <linux/broadcom/vc03/vc_ilcs_defs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/endian.h>
#include <linux/broadcom/vc03/vcilcs.h>


/******************************************************************************
Private types and defines.
******************************************************************************/

// maximum number of threads that can be waiting at once
// we grab this number of events, so some coordination
// with VC_EVENT_MAX_NUM from vchost.h is required
#define VC_ILCS_MAX_WAITING 20
#define VC03_ILCSMSQ_MAX 1
#define VC03_OMXLOG_MAX (32*1024)

typedef struct {
   int xid;
   void *resp;
   int resplen; 
   OS_SEMAPHORE_T sem;
} VC_ILCS_WAIT_T;

typedef struct {
   OS_SEMAPHORE_T rxmsg_sem; // for waking up ilcs thread
   VCHI_SERVICE_HANDLE_T vchi_handle;
   OS_THREAD_T thread;

   OS_SEMAPHORE_T component_lock;

   OS_SEMAPHORE_T wait_sem; // for protecting ->wait and ->next_xid
   VC_ILCS_WAIT_T wait[VC_ILCS_MAX_WAITING];
   int next_xid;

  //
  struct semaphore msg_prod;
  struct semaphore msg_cons;
  vc_ilcsinb_t msg[VC03_ILCSMSQ_MAX];
  
  char* omxlog;
  int  omxlog_rindex;
  int  omxlog_windex;
  struct semaphore omxlog_lock;

} VC_ILCS_GLOBALS_T;

typedef void (*VCIL_FN_T)( void *call, int clen, void *resp, int *rlen );


/******************************************************************************
Private functions in this file.
Define as static.
******************************************************************************/

static void vc_ilcs_callback( void *callback_param, VCHI_CALLBACK_REASON_T reason, void *msg_handle );
static void vc_ilcs_task( unsigned argc, void *argv );
static int  vc_ilcs_process_message( void );
static void vc_ilcs_response( uint32_t xid, unsigned char *msg, int len );
static void vc_ilcs_transmit( uint32_t cmd, uint32_t xid, unsigned char *msg, int len, unsigned char *msg2, int len2 );
static void vc_ilcs_command( uint32_t cmd, uint32_t xid, unsigned char *msg, int len );


/******************************************************************************
Static data.
******************************************************************************/

static VC_ILCS_GLOBALS_T vc_ilcsg;
static void *vc_ilcs_lock = NULL;

static VCIL_FN_T vcilcs_fns[] = {NULL, // response
                                 NULL, // create component

                                 vcil_in_get_component_version,
                                 NULL, // send command
                                 vcil_in_get_parameter,
                                 vcil_in_set_parameter,
                                 vcil_in_get_config,
                                 vcil_in_set_config,
                                 vcil_in_get_extension_index,
                                 vcil_in_get_state,
                                 NULL, // tunnel request
                                 vcil_in_use_buffer,
                                 NULL, // use egl image
                                 NULL, // allocate buffer
                                 vcil_in_free_buffer,
                                 vcil_in_empty_this_buffer,
                                 vcil_in_fill_this_buffer,
                                 NULL, // set callbacks
                                 NULL, // component role enum

                                 NULL, // deinit

                                 vcil_out_event_handler,
                                 vcil_out_empty_buffer_done,
                                 vcil_out_fill_buffer_done,
                                 NULL              // setup camera pools
                                };

static int log_func(char* str, IL_FUNCTION_T func, char* data, int len, char* fmt);
static int log_add(IL_FUNCTION_T func, char* data, int len, char* fmt);
static int component_in_kernel(void* data);
static void vc_ilcs_command_usr( uint32_t cmd, uint32_t xid, unsigned char *msg, int msglen );
static int vcilcs_inb_put(uint32_t cmd, uint32_t xid, uint8_t* data, int len, uint8_t* resp, uint32_t* rlen);


/* ----------------------------------------------------------------------
 * initialise host-side OpenMAX IL component service
 * -------------------------------------------------------------------- */
void
vc_vchi_ilcs_init( VCHI_INSTANCE_T initialise_instance,
                   VCHI_CONNECTION_T **connections,
                   uint32_t num_connections )
{
   int32_t success;

   memset( &vc_ilcsg, 0, sizeof(VC_ILCS_GLOBALS_T) );

   // 
   sema_init(&vc_ilcsg.msg_prod, 0);
   sema_init(&vc_ilcsg.msg_cons, 1);
   sema_init(&vc_ilcsg.omxlog_lock, 1);
   vc_ilcsg.omxlog_rindex = 0;
   vc_ilcsg.omxlog_windex = 0;
   vc_ilcsg.omxlog = vmalloc(VC03_OMXLOG_MAX);
   memset(vc_ilcsg.omxlog, 0, VC03_OMXLOG_MAX);

   // create thread semaphore for blocking
   os_semaphore_create( &vc_ilcsg.component_lock, OS_SEMAPHORE_TYPE_SUSPEND );
   os_semaphore_create( &vc_ilcsg.rxmsg_sem, OS_SEMAPHORE_TYPE_SUSPEND );
   os_semaphore_obtain( &vc_ilcsg.rxmsg_sem );

   // create semaphore for protecting wait/xid structures
   os_semaphore_create( &vc_ilcsg.wait_sem, OS_SEMAPHORE_TYPE_SUSPEND );

   // open 'ILCS' service
   SERVICE_CREATION_T parameters = { MAKE_FOURCC("ILCS"),   // 4cc service code
                                     connections[0],        // passed in fn ptrs
                                     0,                     // tx fifo size (unused)
                                     0,                     // tx fifo size (unused)
                                     &vc_ilcs_callback,     // service callback
                                     &vc_ilcsg.rxmsg_sem }; // callback parameter

   success = vchi_service_open( initialise_instance, &parameters, &vc_ilcsg.vchi_handle );
   assert( success == 0 );

   success = os_thread_start( &vc_ilcsg.thread, vc_ilcs_task, NULL, 4000, "ILCS_HOST" );
   assert( success == 0 );
}

VCHI_SERVICE_HANDLE_T
vc_ilcs_vchi_handle( void )
{
   return vc_ilcsg.vchi_handle;
}

/* ----------------------------------------------------------------------
 * called from the vchi layer whenever an event happens.
 * here, we are only interested in the 'message available' callback
 * -------------------------------------------------------------------- */
static void
vc_ilcs_callback( void *callback_param,
                  const VCHI_CALLBACK_REASON_T reason,
                  void *msg_handle )
{
   int32_t success;
   OS_SEMAPHORE_T *sem;

   switch( reason ) {

   case VCHI_CALLBACK_MSG_AVAILABLE:
      sem = (OS_SEMAPHORE_T *)callback_param;
      if ( sem == NULL )
         break;;
      if ( os_semaphore_obtained(sem) ) {
         success = os_semaphore_release( sem );
         assert( success >= 0 );
      }
      break;

#if 0
   case VCHI_CALLBACK_BULK_RECEIVED:
   case VCHI_CALLBACK_BULK_SENT:
      sem = (OS_SEMAPHORE_T *)msg_handle;
      success = os_semaphore_release( sem );
      assert( success >= 0 );
      break;
#endif
   }
}

/* ----------------------------------------------------------------------
 * send a message and wait for reply.
 * repeats continuously, on each connection
 * -------------------------------------------------------------------- */
static void
vc_ilcs_task( unsigned argc, void *argv )
{
   // FIXME: figure out why initial connect() doesn't block...
   //os_sleep( 50 );

   for (;;) {

      // wait to receive one or more messages
      os_semaphore_obtain( &vc_ilcsg.rxmsg_sem );

      while( vc_ilcs_process_message() )
         ;

   }

   // FIXME: once we implement service close
   //err = OMX_Deinit();
   //assert( err == OMX_ErrorNone );
   //filesys_deregister();
}

/* ----------------------------------------------------------------------
 * check to see if there are any pending messages
 *
 * if there are no messages, return 0
 *
 * otherwise, fetch and process the first queued message (which will
 * be either a command or response from host)
 * -------------------------------------------------------------------- */
static int
vc_ilcs_process_message( void )
{
   int32_t success;
   void *ptr;
   unsigned char *msg;
   VCHI_HELD_MSG_T msg_handle;
   uint32_t msg_len;

   success = vchi_msg_hold( vc_ilcsg.vchi_handle, &ptr, &msg_len, VCHI_FLAGS_NONE, &msg_handle );
   if ( success != 0 ) return 0; // no more messages

   msg = ptr;
   uint32_t cmd = vchi_readbuf_uint32( msg );
   uint32_t xid = vchi_readbuf_uint32( msg + 4 );
   log_add(cmd, msg + 8, msg_len - 8, "<-");

   if ( cmd == IL_RESPONSE )
       vc_ilcs_response( xid, msg + 8, msg_len - 8 );
   else
      //vc_ilcs_command( cmd, xid, msg + 8, msg_len - 8 );
   {
       msg += 8;
       msg_len -= 8;
       if(component_in_kernel(msg)) 
	   {
	      vc_ilcs_command( cmd, xid, msg, msg_len);
	   }
       else
	   {
	      vc_ilcs_command_usr( cmd, xid, msg, msg_len);
       }
    }

   success = vchi_held_msg_release( &msg_handle );
   vc_assert(success == 0);

   return 1;
}

/* ----------------------------------------------------------------------
 * received response to an ILCS command
 * -------------------------------------------------------------------- */
static void
vc_ilcs_response( uint32_t xid, unsigned char *msg, int len )
{
   VC_ILCS_WAIT_T *wait;
   int i;

   // atomically retrieve given ->wait entry
   os_semaphore_obtain( &vc_ilcsg.wait_sem );
   for (i=0; i<VC_ILCS_MAX_WAITING; i++) {
      wait = &vc_ilcsg.wait[i];
      if ( wait->resp && wait->xid == xid )
         break;
   }
   os_semaphore_release( &vc_ilcsg.wait_sem );

   if ( i == VC_ILCS_MAX_WAITING ) {
      // something bad happened
      assert(0);
      return;
   }

   // extract command from fifo and place in response buffer.
   memcpy( wait->resp, msg, len );

   //os_logging_message( "ilcs_poll_fifo: waking waiter %d", i );
   os_semaphore_release( &wait->sem );
}

/* ----------------------------------------------------------------------
 * helper function to transmit an ilcs command/response + payload
 * -------------------------------------------------------------------- */
static void
vc_ilcs_transmit( uint32_t cmd, uint32_t xid, unsigned char *msg, int len, unsigned char *msg2, int len2 )
{
   // could do this in 3 vectors, but hey
   VCHI_MSG_VECTOR_T vec[4];
   int32_t result;
   
   vec[0].vec_base = &cmd;
   vec[0].vec_len  = sizeof(cmd);
   vec[1].vec_base = &xid;
   vec[1].vec_len  = sizeof(xid);
   vec[2].vec_base = msg;
   vec[2].vec_len  = len;
   vec[3].vec_base = msg2;
   vec[3].vec_len  = len2;
   log_add(cmd, msg, len, "->");
   
   result = vchi_msg_queuev( vc_ilcsg.vchi_handle, vec, 4, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
   //result = vchi_msg_queuev( vc_ilcsg.vchi_handle, vec, 4, VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE|VCHI_FLAGS_CALLBACK_WHEN_OP_COMPLETE, NULL );
   vc_assert(result == 0);
}

/* ----------------------------------------------------------------------
 * received response to an ILCS command
 * -------------------------------------------------------------------- */
static void
vc_ilcs_command( uint32_t cmd, uint32_t xid, unsigned char *msg, int len )
{
   // execute this function call
   unsigned char resp[VC_ILCS_MAX_CMD_LENGTH];
   int rlen = -1;

   if ( cmd >= IL_FUNCTION_MAX_NUM ) {
      assert(0);
      return;
   }

   VCIL_FN_T *fn = &vcilcs_fns[ cmd ];
   if ( !fn ) {
      assert(0);
      return;
   }

   //logging_message( LOGGING_VMCS_VERBOSE, "ilcs_poll_fifo: executing xid:%x", xid );

   // at this point we are executing in ILCS task context (videocore side).
   // FIXME: can this cause ilcs_execute_function() calls from within bowels of openmaxil?
   (*fn)( msg, len, resp, &rlen );

   // make sure rlen has been initialised by the function
   assert( rlen != -1 );

   // if rlen is zero, then we don't send a response
   if ( rlen )
      vc_ilcs_transmit( IL_RESPONSE, xid, resp, rlen, NULL, 0 );
}

/* ----------------------------------------------------------------------
 * send a string to the host side IL component service.  if resp is NULL
 * then there is no response to this call, so we should not wait for one.
 *
 * returns response, written to 'resp' pointer
 * -------------------------------------------------------------------- */
void
vc_ilcs_execute_function( IL_FUNCTION_T func, void *data, int len, void *data2, int len2, void *resp, int resplen )
{
   VC_ILCS_WAIT_T *wait;
   int num;

   // the host MUST receive a response
   assert( resp );

   // need to atomically find free ->wait entry
   os_semaphore_obtain( &vc_ilcsg.wait_sem );

   for (;;) {
      num = 0;

      while( num < VC_ILCS_MAX_WAITING && vc_ilcsg.wait[num].resp )
         num++;

      if ( num < VC_ILCS_MAX_WAITING )
         break;

      assert( num < VC_ILCS_MAX_WAITING );
      // might be a fatal error if another thread is relying
      // on this call completing before it can complete
      // we'll pause until we can carry on and hope that's sufficient.
      os_semaphore_release( &vc_ilcsg.wait_sem );
      os_sleep( 10 ); // 10 msec
      os_semaphore_obtain( &vc_ilcsg.wait_sem );
   }
   wait = &vc_ilcsg.wait[num];

   wait->resp = resp;
   wait->xid = vc_ilcsg.next_xid++;
   os_semaphore_create( &wait->sem, OS_SEMAPHORE_TYPE_SUSPEND );
   os_semaphore_obtain( &wait->sem );

   // at this point, ->wait is exclusively ours ()
   os_semaphore_release( &vc_ilcsg.wait_sem );

   // write the command header.
   vc_ilcs_transmit( func, wait->xid, data, len, data2, len2 );

   if ( !os_thread_current(vc_ilcsg.thread) ) {

      os_semaphore_obtain( &wait->sem );

   } else {

      // we're the vcilcs task, so wait for, and handle, incoming
      // messages while we're not completed
      for (;;) {

         // wait->sem will not be released until we process the response message
         if ( vc_ilcs_process_message() == 0 ) {
            // there were no more messages in the fifo; need to wait
            os_semaphore_obtain( &vc_ilcsg.rxmsg_sem );
            continue;
         }

         // did the last message release wait->sem ?
         if ( !os_semaphore_obtained(&wait->sem) )
            break;
      }
   }

   // safe to do the following - the assignment of NULL is effectively atomic
   os_semaphore_destroy( &wait->sem );
   wait->resp = NULL;
}

// send a buffer to VideoCore either by writing the buffer data in a control
// message or by sending an aligned bulk transfer with fixup information in the
// control message.
OMX_ERRORTYPE vc_ilcs_pass_buffer(IL_FUNCTION_T func, void *reference, OMX_BUFFERHEADERTYPE *pBuffer)
{
   IL_PASS_BUFFER_EXECUTE_T exe;
   IL_BUFFER_BULK_T fixup;
   IL_RESPONSE_HEADER_T resp;
   void *data2 = NULL;
   int len2 = 0;

   if ((func == IL_EMPTY_THIS_BUFFER && pBuffer->pInputPortPrivate == NULL) ||
         (func == IL_FILL_THIS_BUFFER && pBuffer->pOutputPortPrivate == NULL))
   {
      // return this to pass conformance
      // the actual error is using a buffer that hasn't be registered with usebuffer/allocatebuffer
      return OMX_ErrorIncorrectStateOperation;
   }

   exe.reference = reference;
   memcpy(&exe.bufferHeader, pBuffer, sizeof(OMX_BUFFERHEADERTYPE));

   if(pBuffer->nFilledLen)
   {
      if(pBuffer->nFilledLen + sizeof(IL_PASS_BUFFER_EXECUTE_T) <= VC_ILCS_MAX_INLINE)
      {
         exe.method = IL_BUFFER_INLINE;
         data2 = pBuffer->pBuffer + pBuffer->nOffset;
         len2 = pBuffer->nFilledLen;
      }
      else
      {
         const uint8_t *start = pBuffer->pBuffer + pBuffer->nOffset;
         const uint8_t *end   = start + pBuffer->nFilledLen;
         const uint8_t *round_start = (const OMX_U8*)VCHI_BULK_ROUND_UP(start);
         const uint8_t *round_end   = (const OMX_U8*)VCHI_BULK_ROUND_DOWN(end);
         int bulk_len = round_end-round_start;
         int32_t result;

         exe.method = IL_BUFFER_BULK;

         result = vchi_bulk_queue_transmit( vc_ilcsg.vchi_handle,
                                            round_start,
                                            round_end-round_start,
                                            VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
                                            NULL );
         // when IL_EMPTY_THIS_BUFFER executes on videocore, there is a
         // corresponding vchi_bulk_queue_receive, which blocks until
         // complete (at which point, the above vchi_bulk_queue_transmit
         // must by definition have completed)
         
         vc_assert(result == 0);

         if((fixup.headerlen = round_start - start) > 0)
            memcpy(fixup.header, start, fixup.headerlen);

         if((fixup.trailerlen = end - round_end) > 0)
            memcpy(fixup.trailer, round_end, fixup.trailerlen);

         data2 = &fixup;
         len2 = sizeof(IL_BUFFER_BULK_T);
      }
   }
   else
   {
      exe.method = IL_BUFFER_NONE;
   }

   vc_ilcs_execute_function(func, &exe, sizeof(IL_PASS_BUFFER_EXECUTE_T), data2, len2, &resp, sizeof(resp));

   return resp.err;
}

// receive a buffer from VideoCore either from the message bytes
// or by a bulk transfer receieve
OMX_BUFFERHEADERTYPE *vc_ilcs_receive_buffer(void *call, int clen, OMX_COMPONENTTYPE **pComp)
{
   IL_PASS_BUFFER_EXECUTE_T *exe = call;
   OMX_BUFFERHEADERTYPE *pHeader = exe->bufferHeader.pInputPortPrivate;
   OMX_U8 *pBuffer = pHeader->pBuffer;
   OMX_PTR *pAppPrivate = pHeader->pAppPrivate;
   OMX_PTR *pPlatformPrivate = pHeader->pPlatformPrivate;
   OMX_PTR *pInputPortPrivate = pHeader->pInputPortPrivate;
   OMX_PTR *pOutputPortPrivate = pHeader->pOutputPortPrivate;

   vc_assert(pHeader);
   memcpy(pHeader, &exe->bufferHeader, sizeof(OMX_BUFFERHEADERTYPE));

   *pComp = exe->reference;

   pHeader->pBuffer = pBuffer;
   pHeader->pAppPrivate = pAppPrivate;
   pHeader->pPlatformPrivate = pPlatformPrivate;
   pHeader->pInputPortPrivate = pInputPortPrivate;
   pHeader->pOutputPortPrivate = pOutputPortPrivate;
   
   if(exe->method == IL_BUFFER_BULK)
   {
      vc_assert(VCHI_BULK_ALIGNED(pHeader->pBuffer));

      IL_BUFFER_BULK_T *fixup = (IL_BUFFER_BULK_T *) (exe+1);

      // bulk transfer from videocore to host
      uint8_t *start = pHeader->pBuffer + pHeader->nOffset;
      uint8_t *end   = start + pHeader->nFilledLen;
      int32_t bulk_len = pHeader->nFilledLen - fixup->headerlen - fixup->trailerlen;
      int32_t result;
      
      vc_assert(clen == sizeof(IL_PASS_BUFFER_EXECUTE_T) + sizeof(IL_BUFFER_BULK_T));

      result = vchi_bulk_queue_receive( vc_ilcsg.vchi_handle,
                                        start + fixup->headerlen,
                                        bulk_len,
                                        VCHI_FLAGS_BLOCK_UNTIL_QUEUED | VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
                                        NULL );
      vc_assert(result == 0);
      
      if (fixup->headerlen)
         memcpy(start, fixup->header, fixup->headerlen);
      if (fixup->trailerlen)
         memcpy(end-fixup->trailerlen, fixup->trailer, fixup->trailerlen);
   }
   else if(exe->method == IL_BUFFER_INLINE)
   {
      IL_BUFFER_INLINE_T *buffer = (IL_BUFFER_INLINE_T *) (exe+1);

      vc_assert(clen == sizeof(IL_PASS_BUFFER_EXECUTE_T) + pHeader->nFilledLen);

      memcpy(pBuffer+pHeader->nOffset, buffer->buffer, pHeader->nFilledLen);
   }
   else if(exe->method == IL_BUFFER_NONE)
   {
      vc_assert(clen == sizeof(IL_PASS_BUFFER_EXECUTE_T));
   }
   else if(exe->method == IL_BUFFER_MAX)
   {
      vc_assert(0);
   }

   return pHeader;
}

void
vc_ilcs_obtain_component_lock( void )
{
   os_semaphore_obtain( &vc_ilcsg.component_lock );
}

void
vc_ilcs_release_component_lock( void )
{
   os_semaphore_release( &vc_ilcsg.component_lock );
}



/*
 *  added for Linux driver 
 */
static int vcilcs_inb_put(uint32_t cmd, uint32_t xid, uint8_t* data, int len, uint8_t* resp, uint32_t* rlen)
{
  vc_ilcsinb_t* ptr;

  // lock msg queue
  if(down_interruptible(&vc_ilcsg.msg_cons))
    {
      ptr->msglen = 0;
      ptr->cmd = 0;
      return -1;
    }

  // put msg
  ptr = vc_ilcsg.msg + 0;  
  memcpy(ptr->msg, data, len);
  ptr->msglen = len;
  ptr->cmd = cmd;
  ptr->xid = xid;

  // unlock, return
  up(&vc_ilcsg.msg_prod);  
  *rlen = 0;  
  return 0;
}

int vc_ilcsinb_getusr(vc_ilcsinb_t* msgptr)
{
  vc_ilcsinb_t* ptr;
  
  if(down_interruptible(&vc_ilcsg.msg_prod))
	{
	  return -1;
	}

  ptr = vc_ilcsg.msg + 0;  

  if(copy_to_user(msgptr, ptr, sizeof(*ptr)) != 0)
	{
	  return -EFAULT;
	}
  
  up(&vc_ilcsg.msg_cons);
  return 0;
}

int vc_ilcsinb_sendresp(vc_ilcsinb_t* msgptr)
{
  vc_ilcs_transmit(IL_RESPONSE, msgptr->xid, msgptr->resp, msgptr->resplen,NULL,0);
  return 0;
}

/*
 *
 */
static int component_in_kernel(void* data)
{
  IL_EXECUTE_HEADER_T* comp=(IL_EXECUTE_HEADER_T*)data;
  if((int)comp->reference >= PAGE_OFFSET)
    return 1;
  else
    return 0;
}

/*
 *
 */
void *vc_ilcs_component_lock(void)
{
  return (void*)&vc_ilcsg.component_lock;
}


/*
 *
 */
int vc_ilcs_componentlock_change(int val)
{
  OS_SEMAPHORE_T *lock = (OS_SEMAPHORE_T*)vc_ilcs_component_lock();
  
  if(NULL == lock)
	{
	  VC_DEBUG( Trace, "Error: invalid lock\n");
	  return -1;
	}
  if(val)
    {
      os_semaphore_obtain(lock);
    }
  else 
    {
      os_semaphore_release(lock);
    }
  return 0;
  
}


/******************************************************************************
NAME
   vc_ilcs_execute_function

SYNOPSIS
   void vc_ilcs_execute_function(IL_FUNCTION_T func, void *data, int len, void *resp)

FUNCTION
   Executes this function on VideoCore via the IL component service

RETURNS
   Response bytes and length
******************************************************************************/
static char* ilfunc2str(IL_FUNCTION_T func)
{
  char* str[] = 
	{
	  "IL_RESPONSE",
	  "IL_CREATE_COMPONENT",
	  "IL_GET_COMPONENT_VERSION",
	  "IL_SEND_COMMAND",               // 4
	  "IL_GET_PARAMETER",               
	  "IL_SET_PARAMETER",              // 6
	  "IL_GET_CONFIG",
	  "IL_SET_CONFIG",                 // 8
	  "IL_GET_EXTENSION_INDEX",
	  "IL_GET_STATE",
	  "IL_COMPONENT_TUNNEL_REQUEST",
	  "IL_USE_BUFFER",                 // 12
	  "IL_USE_EGL_IMAGE",              // 13
	  "IL_ALLOCATE_BUFFER",            // 14
	  "IL_FREE_BUFFER",                // 15
	  "IL_EMPTY_THIS_BUFFER",          // 16
	  "IL_FILL_THIS_BUFFER",           // 17
	  "IL_SET_CALLBACKS",              // 18
	  "IL_COMPONENT_ROLE_ENUM",
	  "IL_COMPONENT_DEINIT",
	  "IL_EVENT_HANDLER",              // 21
	  "IL_EMPTY_BUFFER_DONE",
	  "IL_FILL_BUFFER_DONE",
	  "IL_COMPONENT_NAME_ENUM",
	  
	  "IL_FUNCTION_MAX_NUM",
	};

  vc_assert(func < IL_FUNCTION_MAX_NUM);
  return str[func];
}
  
static char* ilcmd2str(OMX_COMMANDTYPE val)
{
  char* str[] =
	{
	  "OMX_CommandStateSet",    /**< Change the component state */
	  "OMX_CommandFlush",       /**< Flush the data queue(s) of a component */
	  "OMX_CommandPortDisable", /**< Disable a port on a component. */
	  "OMX_CommandPortEnable",  /**< Enable a port on a component. */
	  "OMX_CommandMarkBuffer",  /**< Mark a component/buffer for observation */
	  "OMX_CommandMax",
	};
	  
  vc_assert(val < OMX_CommandMax);
  return str[val];
}

static char* ilerr2str(OMX_ERRORTYPE val)
{
  char* str[] =
	{
	  "OMX_ErrorNone",
	  "OMX_ErrorInsufficientResources",  
	  "OMX_ErrorUndefined",
	  "OMX_ErrorInvalidComponentName",
	  "OMX_ErrorComponentNotFound",
	  "OMX_ErrorInvalidComponent",
	  "OMX_ErrorBadParameter",
	  "OMX_ErrorNotImplemented",
	  "OMX_ErrorUnderflow",
	  "OMX_ErrorOverflow",
	  "OMX_ErrorHardware",
	  "OMX_ErrorInvalidState",
	  "OMX_ErrorStreamCorrupt",
	  "OMX_ErrorPortsNotCompatible",
	  "OMX_ErrorResourcesLost",
	  "OMX_ErrorNoMore",
	  "OMX_ErrorVersionMismatch",
	  "OMX_ErrorNotReady",
	  "OMX_ErrorTimeout",
	  "OMX_ErrorSameState",
	  "OMX_ErrorResourcesPreempted",
	  "OMX_ErrorPortUnresponsiveDuringAllocation",
	  "OMX_ErrorPortUnresponsiveDuringDeallocation",
	  "OMX_ErrorPortUnresponsiveDuringStop",
	  "OMX_ErrorIncorrectStateTransition",
	  "OMX_ErrorIncorrectStateOperation",
	  "OMX_ErrorUnsupportedSetting",
	  "OMX_ErrorUnsupportedIndex",
	  "OMX_ErrorBadPortIndex",
	  "OMX_ErrorPortUnpopulated",
	  "OMX_ErrorComponentSuspended",
	  "OMX_ErrorDynamicResourcesUnavailable",
	  "OMX_ErrorMbErrorsInFrame",
	  "OMX_ErrorFormatNotDetected",
	  "OMX_ErrorContentPipeOpenFailed",
	  "OMX_ErrorContentPipeCreationFailed",
	  "OMX_ErrorSeperateTablesUsed",
	  "OMX_ErrorTunnelingUnsupported"
	  "OMX_ErrorMax",
	};

  val &= 0xfff;
  vc_assert(val < OMX_ErrorMax);
  val = min((int)val, (int)OMX_ErrorMax);
  return str[val];
}

static char* ilevent2str(OMX_EVENTTYPE val)
{
  char* str[] =
	{
	  "OMX_EventCmdComplete",
	  "OMX_EventError",               /**< component has detected an error condition */
	  "OMX_EventMark",                /**< component has detected a buffer mark */
	  "OMX_EventPortSettingsChanged", /**< component is reported a port settings change */
	  "OMX_EventBufferFlag",          /**< component has detected an EOS */ 
	  "OMX_EventResourcesAcquired",   /**< component has been granted resources and is
                                       automatically starting the state change from
                                       OMX_StateWaitForResources to OMX_StateIdle. */
	  "OMX_EventComponentResumed",     /**< Component resumed due to reacquisition of resources */
	  "OMX_EventDynamicResourcesAvailable", /**< Component has acquired previously unavailable dynamic resources */
	  "OMX_EventPortFormatDetected",      /**< Component has detected a supported format. */
	  "OMX_EventMax"
	};
	  
  vc_assert(val < OMX_EventMax);
  val = min((int)val, (int)OMX_EventMax);
  return str[val];
}

static char* ilstate2str(OMX_STATETYPE val)
{
  char* str[] =
	{
	  "OMX_StateInvalid",      /**< component has detected that it's internal data 
								  structures are corrupted to the point that
								  it cannot determine it's state properly */
	  "OMX_StateLoaded",      /**< component has been loaded but has not completed
								 initialization.  The OMX_SetParameter macro
								 and the OMX_GetParameter macro are the only 
								 valid macros allowed to be sent to the 
								 component in this state. */
	  "OMX_StateIdle",        /**< component initialization has been completed
								 successfully and the component is ready to
								 to start. */
	  "OMX_StateExecuting",   /**< component has accepted the start command and
								 is processing data (if data is available) */
	  "OMX_StatePause",       /**< component has received pause command */
	  "OMX_StateWaitForResources", /**< component is waiting for resources, either after 
									  preemption or before it gets the resources requested.
									  See specification for complete details. */
	  "OMX_StateMax"
	};
  vc_assert(val < OMX_StateMax);
  val = min((int)val, (int)OMX_EventMax);
  return str[val];
}

#define KEYSTR_MAX 256
typedef struct
{
  int  mark;
  int  ref;
  char str[KEYSTR_MAX];
} key_str_t;

#define COMPNAME_MAX 20

typedef struct
{
  int count;
  key_str_t name[COMPNAME_MAX];
} namelist_t;

namelist_t compname =
  {
	0,
	{},
  };

static char* log_getname_mark(int mark)
{
  int i;
  
  for(i = 0; i < compname.count; i++)
	{
	  if(compname.name[i].mark == mark)
		{
		  return compname.name[i].str;
		}
	}

  return NULL;  
}

static char* log_getname_ref(int ref)
{
  int i;
  for(i = 0; i < compname.count; i++)
	{
	  if(compname.name[i].ref == ref)
		{
		  return compname.name[i].str;
		}
	}

  return NULL;  
}

static int log_delname_ref(int ref)
{
  int i;
  key_str_t* ptr;

  // find
  ptr = NULL;
  for(i = 0; i < compname.count; i++)
	{
	  if(compname.name[i].ref == ref)
		{
		  ptr =  compname.name + i;
		  break;
		}
	}

  if(NULL==ptr)
	{
	  // not found
	  return -1;
	}

  // delete
  for(; i < compname.count - 1; i++)
	{
	  compname.name[i] = compname.name[i+1];		
	}

  compname.name[compname.count].mark = 0; 
  compname.name[compname.count].ref = 0; 
  compname.name[compname.count].str[0] = 0; 
  --compname.count;
  
  return 0;  
}

static int log_addname_mark(int mark, char* str)
{
  key_str_t* entry;
  
  if(log_getname_mark(mark))
	{
	  return -1;
	}

  entry = compname.name + compname.count;  
  strncpy(entry->str, str, KEYSTR_MAX);
  entry->str[KEYSTR_MAX] = 0;
  entry->mark = mark;
  return 0;
}

static int log_addref(int ref)
{
  key_str_t* entry;
  
  entry = compname.name + compname.count;
  entry->ref = ref;
  compname.count = (compname.count + 1) % COMPNAME_MAX;
  return 0;
}  

static int log_func_resp(char* str, IL_FUNCTION_T func, char* data)
{
  int strlen = 0;
  IL_RESPONSE_HEADER_T* r = (IL_RESPONSE_HEADER_T*)data;
  
  switch(r->func)
	{
	  case IL_CREATE_COMPONENT:		
		{
		  IL_CREATE_COMPONENT_RESPONSE_T* resp = (IL_CREATE_COMPONENT_RESPONSE_T*)data;
		  char* compstr;

		  log_addref((int)resp->reference);
		  compstr = log_getname_ref((int)resp->reference);
		  strlen = sprintf(str, "%s ref=0x%08X r=%s(%d) err=%s(%d) numports=%d portdir=%d",
						   compstr, (int)resp->reference,
						   ilfunc2str(resp->func), resp->func,
						   ilerr2str(resp->err), resp->err,
						   (int)resp->numPorts, (int)resp->portDir);
		  break;
		}
		  
	  case IL_GET_STATE:
		{
		  IL_GET_STATE_RESPONSE_T* resp = (IL_GET_STATE_RESPONSE_T*)data;
		  strlen = sprintf(str, "r=%s(%d) err=%s(%d) state=%s(%d)",
						   ilfunc2str(resp->func), resp->func,
						   ilerr2str(resp->err), resp->err,
						   ilstate2str(resp->state), resp->state
						   );
		  break;
		  
		}
		
	  case IL_USE_BUFFER:
		{
		  IL_ADD_BUFFER_RESPONSE_T* resp = (IL_ADD_BUFFER_RESPONSE_T*)data;
		  OMX_BUFFERHEADERTYPE* bh = (OMX_BUFFERHEADERTYPE*) &resp->bufferHeader;
				 
		  strlen = sprintf(str,
						   "r=%s(%d) err=%s(%d) ref=0x%08X BH:pBuf=0x%08X nAll=0x%08X pApp=0x%08X pPla=0x%08X \n"
						   "          pInp=0x%08X pOut=0x%08X pMark=0x%08X",
						   ilfunc2str(resp->func), resp->func,
						   ilerr2str(resp->err), resp->err,
						   (int)resp->reference,
						   (int)bh->pBuffer, (int)bh->nAllocLen,
						   (int)bh->pAppPrivate, (int)bh->pPlatformPrivate,
						   (int)bh->pInputPortPrivate, (int)bh->pOutputPortPrivate,
						   (int)bh->pMarkData);
						   
		  break;
		}
		
	  default:
		{
		  IL_RESPONSE_HEADER_T* resp = (IL_RESPONSE_HEADER_T*)data;
		  strlen = sprintf(str, "r=%s(%d) err=%s(0x%08X)",
						   ilfunc2str(resp->func), resp->func,
						   ilerr2str(resp->err), resp->err);
		  break;
		}
		
	}
  
  return strlen;
}

static int log_func(char* str, IL_FUNCTION_T func, char* data, int len, char* fmt)
{
  int strlen;

  strlen = sprintf(str, "%s%s(%d) ", fmt, ilfunc2str(func), func);
  str += strlen;
  
  switch(func)
	{
	  case IL_SEND_COMMAND:
		{
		  IL_SEND_COMMAND_EXECUTE_T* exe = (IL_SEND_COMMAND_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  if(exe->cmd == OMX_CommandStateSet)
			{
			  strlen += sprintf(str, "%s cmd=%s(%d) param=%s(%d)",
								compstr,
								ilcmd2str(exe->cmd), (int)exe->cmd,
								ilstate2str(exe->param), (int)exe->param);
			}
		  else
			{
			  strlen += sprintf(str, "%s cmd=%s(%d) param=%d",
								compstr,
								ilcmd2str(exe->cmd), (int)exe->cmd,
								(int)exe->param);		  
			}
		  break;
		}

	  case IL_SET_PARAMETER:
	  case IL_SET_CONFIG:
		{
		  IL_SET_EXECUTE_T* exe = (IL_SET_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);

		  if(OMX_IndexParamContentURI==exe->index)
			{
			  OMX_PARAM_CONTENTURITYPE* uri = (OMX_PARAM_CONTENTURITYPE*) exe->param;
			  char maxpath[255];
			  strncpy(maxpath, (char*)uri->contentURI, 254); maxpath[254] = 0;			  
			  strlen += sprintf(str, "%s index=OMX_IndexParamContentURI contentURI=%s",
								compstr,
								maxpath);
			}
		  else
			{
			  strlen += sprintf(str, "%s index=0x%08X param=%02X %02X %02X %02X",
								compstr, 
								(int)exe->index, 
								exe->param[0], exe->param[1], exe->param[2], exe->param[3]);
			}
		  break;
		}

	  case IL_GET_PARAMETER:
	  case IL_GET_CONFIG:
		{
		  IL_GET_EXECUTE_T* exe = (IL_GET_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s index=0x%08X param=%02X %02X %02X %02X",
							compstr, 
							(int)exe->index,
							exe->param[0], exe->param[1], exe->param[2], exe->param[3]);  
		  break;
		}

	  case IL_EVENT_HANDLER:
		{
		  IL_EVENT_HANDLER_EXECUTE_T *exe = (IL_EVENT_HANDLER_EXECUTE_T*)data;
		  char* compstr = log_getname_mark((int)exe->reference);
		  if(exe->event == OMX_EventError)
			{
			  strlen += sprintf(str, "%s event=%s(%d) data1=0x%08X(%s) data2=0x%08X eventdata=0x%08X",
								compstr,  
								ilevent2str(exe->event), (int)exe->event,
								(int)exe->data1, ilerr2str(exe->data1),
								(int)exe->data2,(int)exe->eventdata);
			}
		  else

			{
			  strlen += sprintf(str, "%s event=%s(%d) data1=%d data2=%d eventdata=%d",
								compstr,
								ilevent2str(exe->event), (int)exe->event,
								(int)exe->data1, (int)exe->data2,(int)exe->eventdata);
			}
		  break;
		}
	  case IL_COMPONENT_TUNNEL_REQUEST:
		{
		  IL_TUNNEL_REQUEST_EXECUTE_T* exe = (IL_TUNNEL_REQUEST_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str,
							"%s port=%d tunnel_ref=0x%08X tunnel_host=%d tunnel_port=%d "
							"setup:nTunnelFlags=0x%08X eSupplier=0x%08X",
							compstr, 
							(int)exe->port,
							(int)exe->tunnel_ref, (int)exe->tunnel_host, (int)exe->tunnel_port,
							(int)exe->setup.nTunnelFlags, (int)exe->setup.eSupplier);		  
		  break;
		}

	  case IL_USE_BUFFER:
		{
		  IL_ADD_BUFFER_EXECUTE_T *exe = (IL_ADD_BUFFER_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s buf_ref=0x%08X port=%d size=%d",
							compstr,
							(int)exe->bufferReference, (int)exe->port, (int)exe->size);
		  break;
		}

	  case IL_EMPTY_THIS_BUFFER:
		{
		  IL_PASS_BUFFER_EXECUTE_T *exe = (IL_PASS_BUFFER_EXECUTE_T*)data;
		  OMX_BUFFERHEADERTYPE* bh = (OMX_BUFFERHEADERTYPE*) &exe->bufferHeader;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s BH:pBuf=0x%08X nAll=0x%08X pApp=0x%08X pPla=0x%08X pInp=0x%08X pOut=0x%08X pMark=0x%08X",
							compstr,
							(int)bh->pBuffer, (int)bh->nAllocLen,
							(int)bh->pAppPrivate, (int)bh->pPlatformPrivate,
							(int)bh->pInputPortPrivate, (int)bh->pOutputPortPrivate,
							(int)bh->pMarkData);
		  break;
		}
		
	  case IL_EMPTY_BUFFER_DONE:
		{
		  IL_PASS_BUFFER_EXECUTE_T *exe = (IL_PASS_BUFFER_EXECUTE_T*)data;
		  OMX_BUFFERHEADERTYPE* bh = (OMX_BUFFERHEADERTYPE*) &exe->bufferHeader;
		  char* compstr = log_getname_mark((int)exe->reference);
		  strlen += sprintf(str, "%s BH:pBuf=0x%08X nAll=0x%08X pApp=0x%08X pPla=0x%08X pInp=0x%08X pOut=0x%08X pMark=0x%08X",
							compstr,
							(int)bh->pBuffer, (int)bh->nAllocLen,
							(int)bh->pAppPrivate, (int)bh->pPlatformPrivate,
							(int)bh->pInputPortPrivate, (int)bh->pOutputPortPrivate,
							(int)bh->pMarkData);
		  break;
		}
		
	  case IL_CREATE_COMPONENT:
		{
		  IL_CREATE_COMPONENT_EXECUTE_T* exe = ( IL_CREATE_COMPONENT_EXECUTE_T*) data;
		  strlen += sprintf(str, "mark=0x%08X name=%s",
							(int)exe->mark, exe->name);
		  log_addname_mark((int)exe->mark, exe->name);
		  		  break;
		}
	  case IL_SET_CALLBACKS:		
		{
		  IL_SET_CALLBACKS_EXECUTE_T* exe = (IL_SET_CALLBACKS_EXECUTE_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s pAppData=0x%08X",
							compstr, 
							(int)exe->pAppData);
		  
		  break;
		}

	  case IL_RESPONSE:
		{		  
		  strlen += log_func_resp(str, func, data);
		  break;
		}

	  case IL_COMPONENT_DEINIT:
		{
		  IL_EXECUTE_HEADER_T* exe = (IL_EXECUTE_HEADER_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s data=0x%08X len=%d", compstr, (int)data, len);
		  log_delname_ref((int)exe->reference);
		}
		
	  case IL_COMPONENT_NAME_ENUM:
		{
		  break;
		}

	  default:
		{		  
		  IL_EXECUTE_HEADER_T* exe = (IL_EXECUTE_HEADER_T*)data;
		  char* compstr = log_getname_ref((int)exe->reference);
		  strlen += sprintf(str, "%s data=0x%08X len=%d", compstr, (int)data, len);
		  break;
		}
	}  

  return strlen;
}

/*
int log_add(IL_FUNCTION_T func, char* data, int len, char* fmt)
{
  (void)func; (void)data; (void)len; (void)fmt;
  return 0;
}
*/

static int log_add(IL_FUNCTION_T func, char* data, int len, char* fmt)
{
  #define MAXSTR 500
  char str[MAXSTR+1];
  int strlen, nbytes;
  char prefix[50];
  unsigned long  j;
  static unsigned long jstart = 0;

  if(jstart == 0)
	jstart = jiffies;

  j = jiffies - jstart;
  sprintf(prefix, "%04lu.%02lu %s", (j / HZ), (j % HZ), fmt);
  
  strlen = log_func(str, func, data, len, prefix);
  sprintf(str + strlen, "\n");
  strlen += 1;
  //  delay_msec(1);
  VC_PRINTK( IlcsTrace, "%s", str );
  //  printk(KERN_ERR"%s", str);   
  
  if(down_interruptible(&vc_ilcsg.omxlog_lock))
	return -1;

  nbytes = min(VC03_OMXLOG_MAX - vc_ilcsg.omxlog_windex, strlen);
  memcpy(vc_ilcsg.omxlog + vc_ilcsg.omxlog_windex, str, nbytes);

  if(vc_ilcsg.omxlog_windex < vc_ilcsg.omxlog_rindex)
	{
	  vc_ilcsg.omxlog_windex += nbytes;
	  if(vc_ilcsg.omxlog_windex >= (vc_ilcsg.omxlog_rindex - 1))		
		{
		  vc_ilcsg.omxlog_rindex = (vc_ilcsg.omxlog_windex + 1) % VC03_OMXLOG_MAX;
		}
	}
  else
	{	  
	  vc_ilcsg.omxlog_windex += nbytes;
	}
  
  if(nbytes < strlen)
	{
	  vc_assert(vc_ilcsg.omxlog_windex >= VC03_OMXLOG_MAX);
	  memcpy(vc_ilcsg.omxlog + 0, str + nbytes, strlen - nbytes);
	  vc_ilcsg.omxlog_windex = strlen - nbytes;
	  vc_ilcsg.omxlog_rindex = max(vc_ilcsg.omxlog_rindex, vc_ilcsg.omxlog_windex + 1);
	}
  
  up(&vc_ilcsg.omxlog_lock);
  return 0;
}

int omxlog_read(char* dest, int maxbytes)
{
  char* src;
  int bytecount;
  
  if(down_interruptible(&vc_ilcsg.omxlog_lock))
	return 0;

  // calc offset
  src = vc_ilcsg.omxlog + vc_ilcsg.omxlog_rindex;
  if(vc_ilcsg.omxlog_rindex  <= vc_ilcsg.omxlog_windex)
	{
	  bytecount = vc_ilcsg.omxlog_windex - vc_ilcsg.omxlog_rindex - 1;
	}
  else
	{
	  bytecount = VC03_OMXLOG_MAX - vc_ilcsg.omxlog_rindex;
	}

  // copy
  bytecount = min(bytecount, maxbytes);
  memcpy(dest, vc_ilcsg.omxlog + vc_ilcsg.omxlog_rindex, bytecount);

  // advance read index
  vc_ilcsg.omxlog_rindex += bytecount;
  if( VC03_OMXLOG_MAX == vc_ilcsg.omxlog_rindex)
	{
	  vc_ilcsg.omxlog_rindex = 0;
	}

  //
  up(&vc_ilcsg.omxlog_lock);
  return bytecount;
}

static void omx_buffer_fixup(IL_PASS_BUFFER_EXECUTE_T* data, void**data2, int*len2,IL_BUFFER_BULK_T *fixup)
{
  OMX_BUFFERHEADERTYPE* headptr;
  void* uaddr;
  vc_malloc_t desc;

   OMX_BUFFERHEADERTYPE *pBuffer;
   IL_RESPONSE_HEADER_T *resp;
   
   *data2 = NULL;
   *len2 = 0;
   
  if(NULL==data || NULL == data->bufferHeader.pBuffer)
	{
	  OMX_DEBUG("error: invalid / corrupt memory\n");
	  return;
	}
  
  headptr = &data->bufferHeader;
  uaddr = (void*)headptr->pBuffer;
  uaddr -= sizeof(desc);
  memcpy(&desc, uaddr, sizeof(desc));

  //
  if(desc.key != VC_MAGIC || NULL == desc.kaddr || NULL == desc.addr)
	{
	  OMX_DEBUG("error: invalid / corrupt memory\n");
	  return;
	}

  headptr->pPlatformPrivate =  headptr->pBuffer;
  headptr->pBuffer = desc.kaddr + sizeof(desc);
#if 0
  vchi_bulk_queue_transmit(vc_ilcs_vchi_handle(),
			   headptr->pBuffer + headptr->nOffset,
			   headptr->nFilledLen,
			   VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
			   NULL );
#else
  pBuffer = headptr;
   if(pBuffer->nFilledLen)
   {
      if(pBuffer->nFilledLen + sizeof(IL_PASS_BUFFER_EXECUTE_T) <= VC_ILCS_MAX_INLINE)
      {
         data->method = IL_BUFFER_INLINE;
         *data2 = pBuffer->pBuffer + pBuffer->nOffset;
         *len2 = pBuffer->nFilledLen;
      }
      else
      {
         const uint8_t *start = pBuffer->pBuffer + pBuffer->nOffset;
         const uint8_t *end   = start + pBuffer->nFilledLen;
         const uint8_t *round_start = (const OMX_U8*)VCHI_BULK_ROUND_UP(start);
         const uint8_t *round_end   = (const OMX_U8*)VCHI_BULK_ROUND_DOWN(end);
         int bulk_len = round_end-round_start;
         int32_t result;

         data->method = IL_BUFFER_BULK;

         result = vchi_bulk_queue_transmit( vc_ilcsg.vchi_handle,
                                            round_start,
                                            round_end-round_start,
                                            VCHI_FLAGS_BLOCK_UNTIL_QUEUED,
                                            NULL );
         // when IL_EMPTY_THIS_BUFFER executes on videocore, there is a
         // corresponding vchi_bulk_queue_receive, which blocks until
         // complete (at which point, the above vchi_bulk_queue_transmit
         // must by definition have completed)
         
         vc_assert(result == 0);

         if((fixup->headerlen = round_start - start) > 0)
            memcpy(fixup->header, start, fixup->headerlen);

         if((fixup->trailerlen = end - round_end) > 0)
            memcpy(fixup->trailer, round_end, fixup->trailerlen);

         *data2 = fixup;
         *len2 = sizeof(IL_BUFFER_BULK_T);
      }
   }
   else
   {
      data->method = IL_BUFFER_NONE;
   }
#endif


}

int vc_ilcs_execute_function_usr(vcilcs_func_out_t* arg)
{
  vcilcs_func_out_t omxdesc;
  uint8_t cmd[VC_ILCS_MAX_CMD_LENGTH];
  uint8_t resp[VC_ILCS_MAX_CMD_LENGTH];
  void *data2 = NULL;
  int len2 = 0;
  IL_BUFFER_BULK_T fixup;

  //
  if(copy_from_user( &omxdesc, (void *)arg, sizeof(omxdesc)) != 0)
    {
      VC_DEBUG( Trace, "error, arg=0x%08X\n", (int)arg);
      return -EFAULT;
    }
  
  assert(omxdesc.len <= VC_ILCS_MAX_CMD_LENGTH);
  assert(omxdesc.resplen <= VC_ILCS_MAX_RESP_LENGTH);
  if(copy_from_user(cmd, omxdesc.data, omxdesc.len) != 0)
    {
      VC_DEBUG( Trace, "copy_from_user error, arg=0x%08X\n", (int)arg);
      return -EFAULT;
    }

  //  memset(resp, 0, omxdesc.resplen);
  if(IL_EMPTY_THIS_BUFFER == omxdesc.func)
    {
      omx_buffer_fixup((IL_PASS_BUFFER_EXECUTE_T*)cmd,&data2,&len2,&fixup);
    }
  
  //
  vc_ilcs_execute_function((IL_FUNCTION_T)omxdesc.func, cmd, omxdesc.len, data2, len2, resp, omxdesc.resplen);
  
  //
  if(copy_to_user((void *)omxdesc.resp, resp, omxdesc.resplen) != 0)
    {
      VC_DEBUG( Trace, "vc03: error, copy_to_user failed\n" );
      return -EFAULT;
    }
  
  return 0;
}

static void vc_ilcs_command_usr( uint32_t cmd, uint32_t xid, unsigned char *msg, int msglen )
{
  int rlen;
  uint8_t resp[VC_ILCS_MAX_CMD_LENGTH];
  
  vcilcs_inb_put(cmd, xid, msg, msglen, resp, &rlen); // no responce is expected iommediately, it'll be sent asynch.
  if ( rlen )
    vc_ilcs_transmit( IL_RESPONSE, xid, resp, rlen,NULL,0 );
}

