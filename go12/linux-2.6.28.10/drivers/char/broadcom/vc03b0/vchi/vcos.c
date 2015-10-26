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



#include <linux/broadcom/vc03/vcos.h>

///////////////////////////////////////////////////////
// Condition variables
#define MAX_COND 200

/******************************************************************************
Static data
******************************************************************************/
static OS_SEMAPHORE_T os_semaphore_global;

typedef struct cond_waiter
{
   struct cond_waiter *next;
   OS_SEMAPHORE_T latch;
} COND_WAITER_T;

struct opaque_os_cond_t
{
   COND_WAITER_T *waiters;
   OS_SEMAPHORE_T *semaphore;
};

// sem_latch protects the os_cond structure and os_cond_next
static OS_SEMAPHORE_T cond_latch;
static uint32_t os_cond_next = 0;
static int latch_init = 0;
static struct opaque_os_cond_t os_cond[ MAX_COND ];

 int32_t os_init( void )
{
   int32_t success;
   
   success = os_semaphore_create(&os_semaphore_global, OS_SEMAPHORE_TYPE_BUSY_WAIT);
   os_assert(success == 0);

   return 0;
}

//TODO:
int32_t os_thread_set_priority( OS_THREAD_T thread, uint32_t priority )
{
   return 0;
}

/***********************************************************
 * Name: os_semaphore_obtain_global
 *
 * Arguments: void
 *
 * Description: obtain a global semaphore.
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_semaphore_obtain_global( void )
{
   return os_semaphore_obtain(&os_semaphore_global);
}

/***********************************************************
 * Name: os_semaphore_release_global
 *
 * Arguments: void
 *
 * Description: release a global semaphore.
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_semaphore_release_global( void )
{
   return os_semaphore_release(&os_semaphore_global);
}


 
/***********************************************************
 * Name: os_cond_create
 *
 * Arguments:  OS_COND_T *condition
 *             OS_SEMAPHORE_T *semaphore
 *
 * Description: Routine to create a condition variable
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_cond_create( OS_COND_T *condition, OS_SEMAPHORE_T *semaphore )
{
   int32_t success = -1;
   int i;
   uint32_t n;
   
   if( condition && semaphore )
   {
      if (!latch_init)
      {
         os_semaphore_create(&cond_latch, OS_SEMAPHORE_TYPE_SUSPEND);
         latch_init = 1;
      }

      os_semaphore_obtain( &cond_latch );

      // start searching from os_cond_next
      n = os_cond_next;
      for (i=0; i<MAX_COND; i++)
      {
         if ( os_cond[n].semaphore == NULL )
            break;
         if ( ++n == MAX_COND )
            n = 0;
      }

      if ( i < MAX_COND )
      {
         OS_COND_T cond = &os_cond[n];
         cond->waiters = NULL;
         cond->semaphore = semaphore;

         *condition = cond;

         success = 0;
         if ( ++n == MAX_COND )
            n = 0;
         os_cond_next = n;
      }

      os_assert( success == 0 );
      os_semaphore_release( &cond_latch );
   }
   
   return success;
}

/***********************************************************
 * Name: os_cond_destroy
 *
 * Arguments:  OS_COND_T *cond
 *
 * Description: Routine to destroy a condition variable
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_cond_destroy( OS_COND_T *cond )
{
   int32_t success = 0;
   
   if (cond)
   {
      os_assert((*cond)->waiters == NULL);
      
      if (!latch_init)
      {
         os_semaphore_create(&cond_latch, OS_SEMAPHORE_TYPE_SUSPEND);
         latch_init = 1;
      }
      
      os_semaphore_obtain( &cond_latch );

      (*cond)->semaphore = NULL;
      os_cond_next = (*cond) - os_cond;
      
      os_assert(os_cond_next < MAX_COND);
      success = 0;
      
      os_assert( success == 0 );
      os_semaphore_release( &cond_latch );
   }

   return success;
}

/***********************************************************
 * Name: os_cond_signal
 *
 * Arguments:  OS_COND_T *cond
 *
 * Description: Routine to signal at least one thread
 *              waiting on a condition variable. The
 *              caller must say whether they have obtained
 *              the semaphore.
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_cond_signal( OS_COND_T *cond, bool_t sem_claimed )
{
   int32_t success = -1;
   
   if (cond)
   {
      COND_WAITER_T *w;
      
      // Ensure the condvar's semaphore is claimed for thread-safe access
      if (!sem_claimed)
      {
         success = os_semaphore_obtain((*cond)->semaphore);
         os_assert(success == 0);
         if (success != 0)
            return success;
      }
      w = (*cond)->waiters;
      if (w)
      {
         // Wake the first person waiting
         os_semaphore_release(&w->latch);
         (*cond)->waiters = w->next;
         
      }
      success = 0;
      if (!sem_claimed)
         success = os_semaphore_release((*cond)->semaphore);
      os_assert(success == 0);
   }

   return success;
}

/***********************************************************
 * Name: os_cond_broadcast
 *
 * Arguments:  OS_COND_T *cond
 *             bool_t sem_claimed
 *
 * Description: Routine to signal all threads waiting on
 *              a condition variable. The caller must
 *              say whether they have obtained the semaphore.
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_cond_broadcast( OS_COND_T *cond, bool_t sem_claimed )
{
   int32_t success = -1;
   
   if (cond)
   {
      COND_WAITER_T *w;
      
      // Ensure the condvar's semaphore is claimed for thread-safe access
      if (!sem_claimed)
      {
         success = os_semaphore_obtain((*cond)->semaphore);
         if (success != 0)
            return success;
      }
      for (w = (*cond)->waiters; w; w = w->next)
      {
         os_semaphore_release(&w->latch);
      }
      (*cond)->waiters = NULL;
      
      success = 0;
      if (!sem_claimed)
         success = os_semaphore_release((*cond)->semaphore);
      os_assert(success == 0);
   }
   
   return success;
}

/***********************************************************
 * Name: os_cond_wait
 *
 * Arguments:  OS_COND_T *cond,
 *             OS_SEMAPHORE_T *semaphore
 *
 * Description: Routine to wait for a condition variable
 *              to be signalled. Semaphore is released
 *              while waiting. The same semaphore must
 *              always be used.
 *
 * Returns: int32_t - success == 0
 *
 ***********************************************************/
int32_t os_cond_wait( OS_COND_T *cond, OS_SEMAPHORE_T *semaphore )
{
   int32_t success = -1;
    
   if (cond && semaphore)
   {
      COND_WAITER_T w;
      COND_WAITER_T **prev;
      COND_WAITER_T *p;
      
      
      // Check the API is being followed
      os_assert((*cond)->semaphore == semaphore && os_semaphore_obtained(semaphore));
      
      // Fill in a new waiter structure allocated on our stack
      w.next = NULL;
      os_semaphore_create(&(w.latch), OS_SEMAPHORE_TYPE_SUSPEND);
      os_semaphore_obtain(&w.latch);
      
      // Add it to the end of the condvar's wait queue (we wake first come, first served)
      prev = &(*cond)->waiters;
      p = (*cond)->waiters;
      
      while (p)
         prev = &p->next, p = p->next;
      *prev = &w;

      // Ready to go to sleep now
      success = os_semaphore_release(semaphore);
      os_assert(success == 0);
      
      os_semaphore_obtain(&w.latch);
      
      success = os_semaphore_obtain(semaphore);
      os_assert(success == 0);
      
      os_semaphore_destroy(&w.latch);
   }
   
   return success;
}


