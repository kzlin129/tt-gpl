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



/*=============================================================================
Project  :  VMCS Host Apps
Module   :  Framework - VMCS
File     :  $RCSfile: platform.c,v $
Revision :  $Revision: #11 $

FILE DESCRIPTION
Platform defs for the host app platform
VMCS runtime image manipulation.
=============================================================================*/

#if defined( __KERNEL__ )

#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#   include <linux/semaphore.h>
#else
#   include <asm/semaphore.h>
#endif

#else // __KERNEL__

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "omxctl.h"

#warning "critical section protection code is not yet implemented in user space"

#endif

#include "vcilplatform.h"
#include <linux/broadcom/vc03/vcos.h>

/******************************************************************************
Private Types
******************************************************************************/

#define PLATFORM_THREAD_DEFAULT_STACK_SIZE   3000

#define BOOL unsigned char
#define TRUE 1
#define FALSE 0

typedef struct
{

} PLATFORM_INTERNAL_THREAD_T;

typedef struct
{

} PLATFORM_TIMER_INFO_T;

#ifndef strcmp_case_insens

#ifdef __HIGHC__
#define strcmp_case_insens _stricmp
#else
#define strcmp_case_insens strcasecmp
#endif

#endif

typedef struct
{
   //the events (bits)
   uint32_t events;
   
} LIN_PLATFORM_EVENTGROUP_T;

#if defined( __KERNEL__ )
#define delay_msec(val) vc_host_delay_msec(val)
#else
#define delay_msec(val) libomx_delay_msec(val)
#endif // KERNEL

/******************************************************************************
Static data
******************************************************************************/

/******************************************************************************
Static function forwards
******************************************************************************/

/******************************************************************************
Global functions
******************************************************************************/

/***********************************************************
 * Name: platform_sleep
 * 
 * Arguments: 
 *       const uint32_t sleep_time_in_ms - sleep time
 *
 * Description: Sleeps the platform for x msecs
 *
 * Returns: void
 *
 ***********************************************************/
void platform_sleep(const uint32_t msec)
{
#if defined( __KERNEL__ )
  
  set_current_state(  TASK_INTERRUPTIBLE );
  schedule_timeout( msecs_to_jiffies( msec ));

#else
  usleep(1000* msec);

#endif

}

/***********************************************************
 * Name: platform_get_local_resource_path
 * 
 * Arguments: 
 *       void
 *
 * Description: Returns the local resource path
 *
 * Returns: char * - pointer to the patrh
 *
 ***********************************************************/
char *platform_get_local_resource_path( void )
{
   return "/mfs/sd/";
}
/***********************************************************
 * Name: platform_malloc
 * 
 * Arguments: 
 *       const uint32_t size_in_bytes, const int line, const char *file
 *
 * Description: Mallocs some data from the heap
 *
 * Returns: void * - pointer to data allocated. NULL if failure.
 *
 ***********************************************************/
void *platform_malloc( const uint32_t size_in_bytes, const int line, const char *file )
{
  (void)line; (void)file;
  
  return malloc(size_in_bytes);
}

/***********************************************************
 * Name: platform_calloc
 * 
 * Arguments: 
 *       const uint32_t num, const uint32_t size_in_bytes, const int line, const char *file
 *
 * Description: Mallocs and clears some data from the heap
 *
 * Returns: void * - pointer to data allocated. NULL if failure.
 *
 ***********************************************************/
void *platform_calloc( const uint32_t num, const uint32_t size_in_bytes, const int line, const char *file )
{
  void* ptr;
  (void)line; (void)file;
  (void)num;
  
  ptr = malloc(size_in_bytes);
  memset(ptr, 0, size_in_bytes);
  return ptr;
}

/***********************************************************
 * Name: platform_free
 * 
 * Arguments: 
 *       void *buffer - pointer to the memory to free
 *
 * Description: Frees data previously malloc'd from the heap.
 *
 * Returns: none
 *
 ***********************************************************/
void platform_free( void *buffer )
{
  free( buffer );
}

/***********************************************************
 * Name: platform_get_vmcs_connected_status
 * 
 * Arguments: 
 *       void
 *
 * Description: Routine to get the status of if we are connected to vmcs or not
 *
 * Returns: uint32_t: > 0 if we are connected
 *
 ***********************************************************/
uint32_t platform_get_vmcs_connected_status( void )
{
   return 1; /* always return that we are connected */
}


/***********************************************************
 * Name: platform_thread_create
 * 
 * Arguments: 
 *       PLATFORM_THREAD_FUNC_T *thread_func,
         void *param
 *
 * Description: routine used to create a new thread. This thread will be passed the void *param when created
 *
 * Returns: PLATFORM_THREAD_T - thread handle
 *
 ***********************************************************/
PLATFORM_THREAD_T VCHPOST_ platform_thread_create(  PLATFORM_THREAD_FUNC_T thread_func, void *param )
{
  (void)thread_func;
  (void)param;
  return 0;
}

/***********************************************************
 * Name: platform_thread_exit
 * 
 * Arguments: 
 *       void
 *
 * Description: Routine to exit a thead (called from within the thread)
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
void platform_thread_exit( void )
{
   //do nothing?
}

/***********************************************************
 * Name: platform_thread_join
 * 
 * Arguments: 
 *       void *thread 
 *
 * Description: Routine used to wait until a thread has exited
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_thread_join( PLATFORM_THREAD_T thread )
{
  (void)thread;
   return 0;
}


/***********************************************************
 * Name: platform_semaphore_create
 * 
 * Arguments: 
 *       const uint32_t inital_count
 *
 * Description: Routine to create a semaphore. Seeds the semaphore with an initial count
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_semaphore_create(  PLATFORM_SEMAPHORE_T *semaphore,
                                    const uint32_t inital_count )
{
  (void)semaphore;
  (void)inital_count;
#if defined( __KERNEL__ )
  sema_init( semaphore, inital_count );
#endif
  
   return 0;
}

                                                   
/***********************************************************
 * Name: platform_semaphore_delete
 * 
 * Arguments: 
 *       PLATFORM_SEMAPHORE_T semaphore
 *
 * Description: Routine to delete a semaphore
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_semaphore_delete( PLATFORM_SEMAPHORE_T *semaphore )
{
  (void)semaphore;
   return 0;
}

/***********************************************************
 * Name: platform_semaphore_wait
 * 
 * Arguments: 
 *       PLATFORM_SEMAPHORE_T semaphore
 *
 * Description: Routine to wait for a semaphore to become free
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_semaphore_wait( PLATFORM_SEMAPHORE_T *semaphore )
{
#if defined( __KERNEL__ )
   down( semaphore );
#endif

  (void)semaphore;
   return 0;
}


/***********************************************************
 * Name: platform_semaphore_post
 * 
 * Arguments: 
 *       PLATFORM_SEMAPHORE_T semaphore
 *
 * Description: Routine to post a semaphore
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_semaphore_post( PLATFORM_SEMAPHORE_T *semaphore )
{
#if defined( __KERNEL__ )
   up( semaphore );
#endif
  (void)semaphore;
   return 0;
}

/***********************************************************
 * Name: platform_eventgroup_create
 * 
 * Arguments: 
 *       PLATFORM_EVENTGROUP_T *eventgroup
 *
 * Description: Routine used to create an event group
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_eventgroup_create( PLATFORM_EVENTGROUP_T *eventgroup )
{
  int32_t success = -1;

   if( NULL != eventgroup )
   {
      LIN_PLATFORM_EVENTGROUP_T *lin_eventgroup  = malloc(sizeof( LIN_PLATFORM_EVENTGROUP_T ) );
	  memset(lin_eventgroup, 0, sizeof(LIN_PLATFORM_EVENTGROUP_T));

      if( NULL != lin_eventgroup )
      {
		//return the event group
		*eventgroup = (PLATFORM_EVENTGROUP_T)lin_eventgroup;
		
		//success!
		success = 0;
      }
      else
      {
         vc_assert( 0 );
      }      
   }

   return success;
}


/***********************************************************
 * Name: platform_eventgroup_delete
 * 
 * Arguments: 
 *       PLATFORM_EVENTGROUP_T eventgroup
 *
 * Description: Routine used to delete an event group
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_eventgroup_delete( PLATFORM_EVENTGROUP_T *eventgroup )
{
  
   int32_t success = -1;
   
   if( NULL != eventgroup )
   {
      LIN_PLATFORM_EVENTGROUP_T *lin_eventgroup  = (LIN_PLATFORM_EVENTGROUP_T *)*eventgroup;

      if( NULL != lin_eventgroup )
      {
         //free the event group
         free( lin_eventgroup );

         //clear the event group
         *eventgroup = (PLATFORM_EVENTGROUP_T)NULL;

         //success!
         success = 0;
      }
      else
      {
         vc_assert( 0 );
      }
   }

   return success;
}

/***********************************************************
 * Name: platform_eventgroup_set
 * 
 * Arguments: 
 *       PLATFORM_EVENTGROUP_T eventgroup,
         const uint32_t bitmask
 *
 * Description: Routines used to set a bit in an eventgroup
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_eventgroup_set( PLATFORM_EVENTGROUP_T *eventgroup,
                                 const uint32_t bitmask,
                                 const PLATFORM_EVENTGROUP_OPERATION_T operation )
{
 int32_t success = -1;
   
   if( NULL != eventgroup )
   {
      LIN_PLATFORM_EVENTGROUP_T *lin_eventgroup  = (LIN_PLATFORM_EVENTGROUP_T *)*eventgroup;
	  
      if( NULL != lin_eventgroup )
		{
		  //get the structure semaphore
		  //set the bits according to the operation
		  if( PLATFORM_EVENTGROUP_OPERATION_OR == operation )
			{
			  //or in the bits
			  lin_eventgroup->events |= bitmask;          
			  
			  //success!
			  success = 0;
            }
		  else
            {
			  //invalid operation
			  vc_assert( 0 );
            }                        
		  
		}
      else
		{
		  vc_assert( 0 );
		}      
   }
   
   return success;
}


/***********************************************************
 * Name: platform_eventgroup_get
 * 
 * Arguments: 
 *       PLATFORM_EVENTGROUP_T eventgroup,
         const uint32_t bits,
         const PLATFORM_EVENTGROUP_OPERATION_T operation,
         const uint32_t timeout,
         uint32_t *retrieved_bits
 *
 * Description: Routines used to get bits in an eventgroup
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t platform_eventgroup_get( PLATFORM_EVENTGROUP_T *eventgroup,
                                 const uint32_t bitmask,
                                 const PLATFORM_EVENTGROUP_OPERATION_T operation,
                                 const uint32_t suspend,
                                 uint32_t *retrieved_bits )
{
  int32_t success = -1;
  LIN_PLATFORM_EVENTGROUP_T *lin_eventgroup;
  BOOL ok_to_exit = (0 == suspend) ? TRUE : FALSE;
  
  if((NULL == eventgroup) || (NULL == retrieved_bits))
	{
	  vc_assert(0);
	  return -1;
	}
  
  lin_eventgroup  = (LIN_PLATFORM_EVENTGROUP_T *)*eventgroup;
  
  if( NULL == lin_eventgroup)
	{
	  vc_assert(0);
	  return -1;
	}
 
  //default retrieved bits to 0
  *retrieved_bits = 0;
  
  do
	{
	  switch( operation )
		{
		  case PLATFORM_EVENTGROUP_OPERATION_AND:
			{
			  if( (lin_eventgroup->events & bitmask) == bitmask )
				{
				  //set the retrieved bits
				  *retrieved_bits = bitmask;                     
				  
				  // success!
				  success = 0;
				  ok_to_exit = TRUE;
				}
			}
			break;
			
		  case PLATFORM_EVENTGROUP_OPERATION_OR:
			{
			  if(lin_eventgroup->events & bitmask )
				{
				  //set the retrieved bits
				  *retrieved_bits = lin_eventgroup->events & bitmask;                        
				  
				  // success!
				  success = 0;
				  ok_to_exit = TRUE;
				}                  
			}
			break;                  

		  case PLATFORM_EVENTGROUP_OPERATION_AND_CONSUME:
			{
			  if( (lin_eventgroup->events & bitmask) == bitmask )
				{
				  ok_to_exit = TRUE;
				  
				  //set the retrieved bits
				  *retrieved_bits = bitmask;
				  
				  //consume the bits
				  lin_eventgroup->events &= ~bitmask;
				  
				  // success!
				  success = 0;
				}
			}
			break;                  
			
		  case PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME:                 
			{
			  if(lin_eventgroup->events & bitmask )
				{
				  ok_to_exit = TRUE;
				  
				  //set the retrieved bits
				  *retrieved_bits = lin_eventgroup->events & bitmask;                        
				  
				  //consume the bits
				  lin_eventgroup->events &= ~(*retrieved_bits);
				  
				  // success!
				  success = 0;
				}
			}
			break;    
			
		  default:
			{
			  vc_assert( 0 );
			}
			break;              
		}
	  
	  
	  //if we still need to wait, suspend on the wait semaphore
	  if( !ok_to_exit && suspend )
		{
		  /*
		  if( WAIT_OBJECT_0 != WaitForSingleObject( win32_eventgroup->semaphore_wait,
													suspend == PLATFORM_EVENTGROUP_SUSPEND ? INFINITE : suspend) )
			{
			  // timeout
			  // if we get woken up frequently by the wrong bits being set, we could end up waiting
			  // for a lot longer than requested - as is, it's a minimum wait time.
			  // could keep track of current time and subtract off the suspend period.
			  ok_to_exit = TRUE;
			}
		  */
		  
		}

#if defined (__KERNEL__)
		   os_sleep(1);
#endif

	  
	}
  while( !ok_to_exit );
  
  return success;
}

/******************************************************************************
Static functions
******************************************************************************/


/**************************** End of file ************************************/
