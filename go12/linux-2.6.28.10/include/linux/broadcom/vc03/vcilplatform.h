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
Module   :  Framework
File     :  $RCSfile: platform.h,v $
Revision :  $Revision: #9 $

FILE DESCRIPTION
Platform API for VMCS host apps.
=============================================================================*/

#ifndef PLATFORM_H
#define PLATFORM_H

#include "vchost.h"

/******************************************************************************
Global typedefs, macros and constants
******************************************************************************/

/* the definition of all the message types that a
   platform can send to the message handler */

typedef enum
{
   /* A message sent at the start of the day

      Param 1 = 0
      Param 2 = 0 */
   PLATFORM_MSG_INIT,

   /*A message sent at the end of the day

      Param 1 = 0
      Param 2 = 0 */
   PLATFORM_MSG_END,

   /*A message sent when a button action occurs

      Param 1 = the button ID
      Param 2 = the hold time - only valid for repeat button presses */
   PLATFORM_MSG_BUTTON_PRESS,
   PLATFORM_MSG_BUTTON_REPEAT,
   PLATFORM_MSG_BUTTON_RELEASE,

   /*A message sent when a timer event occurs

      Param 1 = the timer ID
      Param 2 = 0 */
   PLATFORM_MSG_TIMER_TICK,

   /*A message sent when a hostreq notify is sent from VMCS

      Param 1 = the event id
      Param 2 = event param */
   PLATFORM_MSG_HOSTREQ_NOTIFY,

   /* user messages go here */

   PLATFORM_MSG_USER

} PLATFORM_MSG_T;


/* the definitions of the buttons in the system
   Current these are mapped to the VC02DK */

typedef enum
{
   //The push buttons
   PLATFORM_BUTTON_A = 0x001,
   PLATFORM_BUTTON_B = 0x002,
   PLATFORM_BUTTON_C = 0x004,
   PLATFORM_BUTTON_D = 0x008,
   PLATFORM_BUTTON_E = 0x010,

   //The joystick moves
   PLATFORM_BUTTON_DOWN = 0x020,
   PLATFORM_BUTTON_LEFT = 0x040,
   PLATFORM_BUTTON_UP = 0x080,
   PLATFORM_BUTTON_RIGHT = 0x100,
   PLATFORM_BUTTON_Z = 0x200,

   PLATFORM_BUTTON_PAGE_UP = 0x400,
   PLATFORM_BUTTON_PAGE_DOWN = 0x800

   //UPDATE THE NUMBER OF BUTTONS BELOW IF YOU EDIT THESE

} PLATFORM_BUTTON_T;

//the max number of buttons definition
#define PLATFORM_NUMBER_BUTTONS  12

//define the max file path
#ifndef MAX_PATH
#define MAX_PATH  260
#endif

//The default buffer size of the general command response
#define PLATFORM_DEFAULT_RESPONSE_BUF_SIZE   (uint32_t)(128)

#define PLATFORM_DEFAULT_REQUEST_BUF_SIZE    (uint32_t)(128)

//the max number of timers which an app can use
#define PLATFORM_MAX_NUM_TIMERS 4

//the starting id for the timers
#define PLATFORM_TIMER_START_ID 0


/* Definition for a platfor message handler - returning FALSE means that the app wishes to quit */
typedef int32_t (*PLATFORM_MESSAGE_HANDLER_T)(  const uint16_t msg,
                                                const uint32_t param1,
                                                const uint32_t param2 );

/******************************************************************************
Global Message funcs definitions
******************************************************************************/

// routines used to post a message to the platform message handle
VCHPRE_ int32_t VCHPOST_ platform_post_message( const uint16_t msg,
                                                const uint32_t param1,
                                                const uint32_t param2);

/******************************************************************************
Global Task funcs definitions
******************************************************************************/

// Definition for a platform thread function
typedef int32_t (*PLATFORM_THREAD_FUNC_T)( void *param );

// Definition for a platform thread handle
typedef void * PLATFORM_THREAD_T;

// routine used to create a new thread. This thread will be passed the void *param when created
// NOTE: the 'thread_func' MUST call the platform_thread_exit function before leaving the context of the function
VCHPRE_ PLATFORM_THREAD_T VCHPOST_ platform_thread_create(  PLATFORM_THREAD_FUNC_T thread_func,
                                                            void *param );

//Routine to set a threads priority
VCHPRE_ int32_t VCHPOST_ platform_thread_set_priority(PLATFORM_THREAD_T thread,
                                                      const uint32_t priority );

//Routine to exit a thead (called from within the thread)
//NOTE! You must call this function before exiting a thread.
VCHPRE_ void VCHPOST_ platform_thread_exit( void );

//Routine used to wait until a thread has exited
VCHPRE_ int32_t VCHPOST_ platform_thread_join( PLATFORM_THREAD_T thread );

/******************************************************************************
Semaphore funcs definitions
******************************************************************************/

//definition of a semaphore
#if defined( __KERNEL__ )
typedef struct semaphore PLATFORM_SEMAPHORE_T;
#else
typedef void *PLATFORM_SEMAPHORE_T;
#endif

//Routine to create a semaphore. Seeds the semaphore with an initial count
VCHPRE_ int32_t VCHPOST_ platform_semaphore_create( PLATFORM_SEMAPHORE_T *semaphore, const uint32_t inital_count );

//Routine to delete a semaphore
VCHPRE_ int32_t VCHPOST_ platform_semaphore_delete( PLATFORM_SEMAPHORE_T *semaphore );

//Routine used to wait for a semaphore to be free (will suspend the calling thread if not available)
VCHPRE_ int32_t VCHPOST_ platform_semaphore_wait( PLATFORM_SEMAPHORE_T *semaphore );

//Routine used to post (increment) a semaphore. used to release any tasks waiting on the semaphore
VCHPRE_ int32_t VCHPOST_ platform_semaphore_post( PLATFORM_SEMAPHORE_T *semaphore );

/******************************************************************************
Global Event funcs definitions
******************************************************************************/

// Definition of an event group
struct PLATFORM_EVENTGROUP;
typedef struct PLATFORM_EVENTGROUP *PLATFORM_EVENTGROUP_T;

//Event group flags
typedef enum
{
   PLATFORM_EVENTGROUP_OPERATION_MIN,

   PLATFORM_EVENTGROUP_OPERATION_AND,
   PLATFORM_EVENTGROUP_OPERATION_OR,

   PLATFORM_EVENTGROUP_OPERATION_AND_CONSUME,
   PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME,

   PLATFORM_EVENTGROUP_OPERATION_MAX

} PLATFORM_EVENTGROUP_OPERATION_T;

#define PLATFORM_EVENTGROUP_SUSPEND -1
#define PLATFORM_EVENTGROUP_NO_SUSPEND 0

//Routines used to create an event group
VCHPRE_ int32_t VCHPOST_ platform_eventgroup_create( PLATFORM_EVENTGROUP_T *eventgroup );

//Routines used to delete an event group
VCHPRE_ int32_t VCHPOST_ platform_eventgroup_delete( PLATFORM_EVENTGROUP_T *eventgroup );

//Routines used to set a bit in an eventgroup
VCHPRE_ int32_t VCHPOST_ platform_eventgroup_set(  PLATFORM_EVENTGROUP_T *eventgroup,
                                                   const uint32_t bitmask,
                                                   const PLATFORM_EVENTGROUP_OPERATION_T operation );

//Routine used to wait for bits in an eventgroup to be set
VCHPRE_ int32_t VCHPOST_ platform_eventgroup_get(  PLATFORM_EVENTGROUP_T *eventgroup,
                                                   const uint32_t bitmask,
                                                   const PLATFORM_EVENTGROUP_OPERATION_T operation,
                                                   const uint32_t suspend,
                                                   uint32_t *retrieved_bits );

/******************************************************************************
Global Timer func definitions
******************************************************************************/

// Routines used to start / stop a single timer for the host app platform
VCHPRE_ int32_t VCHPOST_ platform_timer_start(  const uint32_t timer_id,
                                                const uint32_t timer_period );

VCHPRE_ int32_t VCHPOST_ platform_timer_stop( const uint32_t timer_id );

// Routine to change the timer resolution
VCHPRE_ int32_t VCHPOST_ platform_timer_reschedule(const uint32_t timer_id,
                                                   const uint32_t timer_period );

/******************************************************************************
Global C library style func definitions
******************************************************************************/

// routine to set the button repeat rate
VCHPRE_ void VCHPOST_ platform_set_button_repeat_time( const uint32_t repeat_rate_in_ms );

// routine to enable / disable a host req message
VCHPRE_ int32_t VCHPOST_ platform_enable_hostreq_notify( const uint32_t notify_event, const uint32_t enable );

// routine to sleep the platform
VCHPRE_ void VCHPOST_ platform_sleep( const uint32_t sleep_time_in_ms );

// routine to get the local resource path
VCHPRE_ char * VCHPOST_ platform_get_local_resource_path( void );

// routine perform a str comparison with case sensitivity
VCHPRE_ int32_t VCHPOST_ platform_stricmp( const char *str1, const char *str2 );

// routine to return the number of ticks since the start of day, in ms
VCHPRE_ int32_t VCHPOST_ platform_get_ticks( void );

// routine to send out debug print messages to the platform host
VCHPRE_ void VCHPOST_ platform_debug_print( const char *string, ... );

//Routine to malloc a section of memory - has addition line and file parameters for debug
VCHPRE_ void * VCHPOST_ platform_malloc(  const uint32_t size_in_bytes,
                                          const int line,
                                          const char *file );

//Routine to malloc and clear a section of memory - has addition line and file parameters for debug
VCHPRE_ void * VCHPOST_ platform_calloc(  const uint32_t num,
                                          const uint32_t size_in_bytes,
                                          const int line,
                                          const char *file );

// Routine to free memory previously malloc'd
VCHPRE_ void VCHPOST_ platform_free( void *buffer );

//Routine to query if we are connected to VMCS or not
VCHPRE_ uint32_t VCHPOST_ platform_get_vmcs_connected_status( void );

//Routine to create + return the hosts native debug window handle
VCHPRE_ uint32_t VCHPOST_ platform_create_and_get_debug_window_handle( void );

//Routine to get the platforms screen size
VCHPRE_ int32_t VCHPOST_ platform_get_screen_size( uint32_t *width,
                                                   uint32_t *height );

#endif /* PLATFORM_H */

/**************************** End of file ************************************/
