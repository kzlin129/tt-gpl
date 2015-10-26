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

Project  :  VideoCore Software Host Interface (Host-side functions)
Module   :  Host Request Service (host-side)
File     :  $Id: //software/vc3/REL/interface/vmcs_host/vc_vchi_gencmd.c#2 $
Revision :  $Revision: #2 $

FILE DESCRIPTION
General command service using VCHI
=============================================================================*/
#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vc_vchi_gencmd.h>
#include <linux/broadcom/vc03/vcutil.h>
#include <linux/broadcom/vc03/vcstate.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/endian.h>
#include <linux/broadcom/vc03/gencmd_common.h>


/******************************************************************************
Local types and defines.
******************************************************************************/
#define GENCMD_MAX_LENGTH 512
typedef struct {
   VCHI_SERVICE_HANDLE_T open_handle[VCHI_MAX_NUM_CONNECTIONS];
   uint32_t              msg_flag[VCHI_MAX_NUM_CONNECTIONS];
   char                  command_buffer[GENCMD_MAX_LENGTH+1];
   char                  response_buffer[GENCMDSERVICE_MSGFIFO_SIZE];
   uint32_t              response_length;  //Length of response minus the error code
   int                   num_connections;
   OS_SEMAPHORE_T        sema;
} GENCMD_SERVICE_T;

//static GENCMD_SERVICE_T gencmd_client;
static OS_SEMAPHORE_T gencmd_message_available_semaphore;


/******************************************************************************
Static function.
******************************************************************************/
static void gencmd_callback( void *callback_param,
                             VCHI_CALLBACK_REASON_T reason,
                             void *msg_handle );

static void gencmd_lock_obtain ( OS_SEMAPHORE_T * sema ) {
   //assert(!os_semaphore_obtained(&gencmd_client.sema));
   int success = os_semaphore_obtain( sema );
   assert(success >= 0);
}
static void gencmd_lock_release (OS_SEMAPHORE_T * sema) {
   //assert(os_semaphore_obtained(&gencmd_client.sema));
   int32_t success = os_semaphore_release( sema );
   assert( success >= 0 );
}

//call vc_vchi_gencmd_init to initialise
int vc_gencmd_init() {
   assert(0); 
   return 0;
}

/******************************************************************************
NAME
   vc_vchi_gencmd_init

SYNOPSIS
   void vc_vchi_gencmd_init(VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections )

FUNCTION
   Initialise the general command service for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

void * vc_vchi_gencmd_init (VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections ) {
   int32_t success;
   int i;
   GENCMD_SERVICE_T * instp = vmalloc( sizeof( GENCMD_SERVICE_T ) );

   // record the number of connections
   memset( instp, 0, sizeof(GENCMD_SERVICE_T) );
   instp->num_connections = num_connections;
   success = os_semaphore_create( &instp->sema, OS_SEMAPHORE_TYPE_SUSPEND );
   assert( success == 0 );
   success = os_semaphore_create( &gencmd_message_available_semaphore, OS_SEMAPHORE_TYPE_BUSY_WAIT );
   assert( success == 0 );
   success = os_semaphore_obtain( &gencmd_message_available_semaphore );
   assert( success == 0 );
 
   for (i=0; i<instp->num_connections; i++) {

      // Create a 'LONG' service on the each of the connections
      SERVICE_CREATION_T gencmd_parameters = { MAKE_FOURCC("GCMD"),      // 4cc service code
                                               connections[i],           // passed in fn ptrs
                                               0,                        // tx fifo size (unused)
                                               0,                        // tx fifo size (unused)
                                               &gencmd_callback,         // service callback
                                               &gencmd_message_available_semaphore }; // callback parameter
   
      success = vchi_service_open( initialise_instance, &gencmd_parameters, &instp->open_handle[i] );
      assert( success == 0 );
   }

   return( (void *)instp );

}

/******************************************************************************
NAME
   gencmd_callback

SYNOPSIS
   void gencmd_callback( void *callback_param,
                     const VCHI_CALLBACK_REASON_T reason,
                     const void *msg_handle )

FUNCTION
   VCHI callback 

RETURNS
   int
******************************************************************************/
static void gencmd_callback( void *callback_param,
                             const VCHI_CALLBACK_REASON_T reason,
                             void *msg_handle ) {

   OS_SEMAPHORE_T *sem = (OS_SEMAPHORE_T *)callback_param;

   if ( reason != VCHI_CALLBACK_MSG_AVAILABLE )
      return;
   
   if ( sem == NULL )
      return;
   
   if ( os_semaphore_obtained(sem) ) {
      int32_t success = os_semaphore_release( sem );
      assert( success >= 0 );
   }

}

/******************************************************************************
NAME
   vc_gencmd_stop

SYNOPSIS
   int vc_gencmd_stop()

FUNCTION
   This tells us that the generak command service has stopped, thereby preventing
   any of the functions from doing anything.

RETURNS
   int
******************************************************************************/

void vc_gencmd_stop ( void * instp ) {
   // Assume a "power down" gencmd has been sent and the lock is held. There will
   // be no response so this should be called instead.
   int32_t success,i;
   GENCMD_SERVICE_T * clientp = (GENCMD_SERVICE_T *)instp;


   for(i = 0; i<clientp->num_connections; i++) {
      success = vchi_service_close( clientp->open_handle[i]);
      assert(success == 0);
   }
   /* free the service memory associated */
   vfree( instp );
}

/******************************************************************************
NAME
   vc_gencmd_send_string

SYNOPSIS
   int vc_gencmd_send_string(char *command)

FUNCTION
   Send a string to general command service.
   Return 

RETURNS
   int
******************************************************************************/

static int vc_gencmd_send_string( GENCMD_SERVICE_T * clientp )
{
   int success = 0, i;
   for( i=0; i<clientp->num_connections; i++ ) {
      success += vchi_msg_queue( clientp->open_handle[i],
                                 clientp->command_buffer,
                                 strlen(clientp->command_buffer)+1,
                                 VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL );
      assert(!success);
      if(success != 0)
         break;
   }
   gencmd_lock_release( &clientp->sema );
   return success;
}

/******************************************************************************
NAME
   vc_gencmd_send

SYNOPSIS
   int vc_gencmd_send( const char *format, ... )

FUNCTION
   Behaves exectly as vc_gencmd_send_string, except allows varargs.

RETURNS
   int
******************************************************************************/
// original function broken into two parts so that a higher level interface
// can be implemented and re-use the code
int vc_gencmd_send_list ( void * instp, const char *format, va_list a )
{
   int length;
   GENCMD_SERVICE_T * clientp = (GENCMD_SERVICE_T *)instp;

   // Obtain the lock and keep it so no one else can butt in while we await the response.
   gencmd_lock_obtain( &clientp->sema );

   if (clientp->open_handle[0] == 0) {
      gencmd_lock_release( &clientp->sema );
     return -1;
   }
   length = vsnprintf( clientp->command_buffer, GENCMD_MAX_LENGTH, format, a );

   if ( (length < 0) || (length >= GENCMD_MAX_LENGTH ) )
   {
      assert(0);
      gencmd_lock_release( &clientp->sema );
      return -1;
   }

   /* send message back to caller */
   return vc_gencmd_send_string ( clientp );
}

int vc_gencmd_send (  void * instp, const char *format, ... )
{
   va_list a;
   int     rv;

   va_start ( a, format );
   rv = vc_gencmd_send_list( instp, format, a );
   va_end ( a );
   return rv;
}

/******************************************************************************
NAME
   vc_gencmd_read_response

SYNOPSIS
   int vc_gencmd_read_response

FUNCTION
   Block until something comes back

RETURNS
   Error code from dequeue message
******************************************************************************/
int vc_gencmd_read_response (void * instp, char *response, int maxlen) {
   int i = 0;
   int success = 0;
   int ret_code = 0;
   int sem_ok = 0;
   GENCMD_SERVICE_T * clientp = (GENCMD_SERVICE_T *)instp;

   //Note this will ALWAYS reset response buffer and overwrite any partially read responses

   do {
      //TODO : we need to deal with messages coming through on more than one connections properly
      //At the moment it will always try to read the first connection if there is something there
      for(i = 0; i < clientp->num_connections; i++) {
         //Check if there is something in the queue, if so return immediately
         //otherwise wait for the semaphore and read again
         success = vchi_msg_dequeue( clientp->open_handle[i], clientp->response_buffer, sizeof(clientp->response_buffer), &clientp->response_length, VCHI_FLAGS_NONE);
         if(success == 0) {
            ret_code = VC_VTOH32( *(int *)clientp->response_buffer );
            break;
         } else {
            clientp->response_length = 0;
         }
      }
   } while(!clientp->response_length && (sem_ok = os_semaphore_obtain( &gencmd_message_available_semaphore)) == 0);

   if(clientp->response_length && sem_ok == 0) {
      clientp->response_length -= sizeof(int); //first word is error code
      memcpy(response, clientp->response_buffer+sizeof(int), OS_MIN(clientp->response_length, maxlen));
   }
   // If we read anything, return the VideoCore code. Error codes < 0 mean we failed to
   // read anything...
   //How do we let the caller know the response code of gencmd?
   //return ret_code;
   return success;
}

/******************************************************************************
NAME
   vc_gencmd

SYNOPSIS
   int vc_gencmd(char *response, int maxlen, const char *format, ...)

FUNCTION
   Send a gencmd and receive the response as per vc_gencmd read_response.

RETURNS
   int
******************************************************************************/
int vc_gencmd(void * instp, char *response, int maxlen, const char *format, ...) {
   va_list args;
   int ret;

   va_start(args, format);
   ret = vc_gencmd_send_list(instp, format, args);
   va_end (args);

   if (ret < 0)
      return ret;

   return vc_gencmd_read_response(instp, response, maxlen);
}

/******************************************************************************
NAME
   vc_gencmd_string_property

SYNOPSIS
   int vc_gencmd_string_property(char *text, char *property, char **value, int *length)

FUNCTION
   Given a text string, containing items of the form property=value,
   look for the named property and return the value. The start of the value
   is returned, along with its length. The value may contain spaces, in which
   case it is enclosed in double quotes. The double quotes are not included in
   the return parameters. Return non-zero if the property is found.

RETURNS
   int
******************************************************************************/

int vc_gencmd_string_property(char *text, const char *property, char **value, int *length) {
#define READING_PROPERTY 0
#define READING_VALUE 1
#define READING_VALUE_QUOTED 2
   int state = READING_PROPERTY;
   int delimiter = 1, match = 0, len = (int)strlen(property);
   char *prop_start=text, *value_start=text;
   for (; *text; text++) {
      int ch = *text;
      switch (state) {
      case READING_PROPERTY:
         if (delimiter) prop_start = text;
         if (isspace(ch)) delimiter = 1;
         else if (ch == '=') {
            delimiter = 1;
            match = (text-prop_start==len && strncmp(prop_start, property, text-prop_start)==0);
            state = READING_VALUE;
         }
         else delimiter = 0;
         break;
      case READING_VALUE:
         if (delimiter) value_start = text;
         if (isspace(ch)) {
            if (match) goto success;
            delimiter = 1;
            state = READING_PROPERTY;
         }
         else if (delimiter && ch == '"') {
            delimiter = 1;
            state = READING_VALUE_QUOTED;
         }
         else delimiter = 0;
         break;
      case READING_VALUE_QUOTED:
         if (delimiter) value_start = text;
         if (ch == '"') {
            if (match) goto success;
            delimiter = 1;
            state = READING_PROPERTY;
         }
         else delimiter = 0;
         break;
      }
   }
   if (match) goto success;
   return 0;
success:
   *value = value_start;
   *length = text - value_start;
   return 1;
}

/******************************************************************************
NAME
   vc_gencmd_number_property

SYNOPSIS
   int vc_gencmd_number_property(char *text, char *property, int *number)

FUNCTION
   Given a text string, containing items of the form property=value,
   look for the named property and return the numeric value. If such a numeric
   value is successfully found, return 1; otherwise return 0.

RETURNS
   int
******************************************************************************/

int vc_gencmd_number_property(char *text, const char *property, int *number) {
   char *value, temp;
   int length, retval;
   if (vc_gencmd_string_property(text, property, &value, &length) == 0)
      return 0;
   temp = value[length];
   value[length] = 0;
   retval = sscanf(value, "0x%x", number);
   if (retval != 1)
      retval = sscanf(value, "%d", number);
   value[length] = temp;
   return retval;

}

/******************************************************************************
NAME
   vc_gencmd_until

SYNOPSIS
   int vc_gencmd_until(const char *cmd, const char *error_string, int timeout);

FUNCTION
   Sends the command repeatedly, until one of the following situations occurs:
   The specified response string is found within the gencmd response.
   The specified error string is found within the gencmd response.
   The timeout is reached.

   The timeout is a rough value, do not use it for precise timing.

RETURNS
   0 if the requested response was detected.
   1 if the error string is detected or the timeout is reached.
******************************************************************************/
int vc_gencmd_until( void        *instp,
                     char        *cmd,
                     const char  *property,
                     char        *value,
                     const char  *error_string,
                     int         timeout) {
   char response[128];
   int length;
   char *ret_value;

   for (;timeout > 0; timeout -= 10) {
      vc_gencmd(instp, response, sizeof(response), cmd);
      if (strstr(response,error_string)) {
         return 1;
      }
      else if (vc_gencmd_string_property(response, property, &ret_value, &length) &&
               strncmp(value,ret_value,length)==0) {
         return 0;
      }
      os_sleep(10);
   }
   // Timed out
   return 1;
}

