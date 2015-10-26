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




#include <linux/sched.h>  /* current and everything */
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/types.h>  /* size_t */
#include <linux/list.h>
#include <linux/sem.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/broadcom/vc.h>
#include <linux/broadcom/timer.h>

#include "vchost.h"
#include "vchost_int.h"

void vc_interface_unregister_event_int(void *event);

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

#define EVENT_MAGIC     0x45766e74  // Evnt in hex
#define LOCK_MAGIC      0x4c6f636b  // Lock in hex

#define  VCOS_EVENT_LOGGING   0
#define  VCOS_EVENT_PRINTING  0


#include <asm/atomic.h>

typedef struct
{
   uint32_t timestamp;
   uint8_t  action;
   Event_t *event;

} Log_t;

#if VCOS_EVENT_LOGGING

Log_t gLog[ 1024 ];
int   gLogIdx;

void Log( uint8_t action, Event_t *event )
{
   Log_t *entry = &gLog[ ( gLogIdx++ ) & 1023 ];

   entry->timestamp = timer_get_msec();
   entry->action = action;
   entry->event = event;
}

void LogStr( const char *str )
{
   Log( 'M', (Event_t*)str );
}

void LogIrq( int x )
{
   Log( 'I', (Event_t *)x );
}

#elif VCOS_EVENT_PRINTING

void Log( uint8_t action, Event_t *event )
{
   printk( "%c: %u\n", action, event->index );
}

void LogStr( const char *str )
{
   //printk( "%8u %s\n", timer_get_msec(), str );
}

void LogIrq( int x )
{
   printk( "Irq: 0x%08x\n", x );
}

#else

void Log( uint8_t action, Event_t *event ) { (void)action; (void)event; }
void LogStr( const char *str ) { (void)str; }
void LogIrq( int x ) { (void)x; }

#endif

/******************************************************************************
Static data.
******************************************************************************/

static LIST_HEAD( gEventList );
static LIST_HEAD( gLockList );

static int        gEventsBlocking = 0;

static spinlock_t gListLock = SPIN_LOCK_UNLOCKED;
static spinlock_t gEventLock = SPIN_LOCK_UNLOCKED;

void DumpLog( void )
{
#if VCOS_EVENT_LOGGING
   int i;
   Event_t  *event;
   struct list_head *node = gEventList.next;
   const char  *fileName;
   const char  *createFileName;

   printk( "***** Dumping Log *****\n" );
   for ( i = 0; i < ( gLogIdx & 1023 ); i++ )
   {
      event = gLog[ i ].event;

      if ( gLog[ i ].action == 'M' )
      {
         printk( "%8u %s\n", gLog[ i ].timestamp, (char *)event );
      }
      else
      if ( gLog[ i ].action == 'I' )
      {
         printk( "%8u Irq: 0x%08x\n", gLog[ i ].timestamp, (int)event );
      }
      else
      {
         if (( fileName = strrchr( event->fileName, '/' )) == NULL )
         {
            fileName = event->fileName;
         }
         else
         {
            fileName++;
         }
         if (( createFileName = strrchr( event->createFileName, '/' )) == NULL )
         {
            createFileName = event->createFileName;
         }
         else
         {
            createFileName++;
         }

         printk( "%8u %c: %u C:%s:%d W:%s:%d\n",
                 gLog[ i ].timestamp, gLog[ i ].action, event->index,
                 createFileName, event->createLineNum, fileName, event->lineNum );
      }
   }
   printk( "***** End of Log *****\n" );

   while ( node != &gEventList )
   {
      event = list_entry( node, Event_t, node );

      if (( fileName = strrchr( event->fileName, '/' )) == NULL )
      {
         fileName = event->fileName;
      }
      else
      {
         fileName++;
      }
      if (( createFileName = strrchr( event->createFileName, '/' )) == NULL )
      {
         createFileName = event->createFileName;
      }
      else
      {
         createFileName++;
      }

      printk( "%u C:%s:%d W:%s:%d\n", event->index, createFileName, event->createLineNum, fileName, event->lineNum );

      node = node->next;
   }

   printk( "***** End of Events *****\n" );
#endif
}
void show_vc02_log( void )
{
   DumpLog();
}

/******************************************************************************
Static functions.
******************************************************************************/
/*---------------------------------------------------------------------------*/
/* Lock related functions */
/*---------------------------------------------------------------------------*/
/******************************************************************************
NAME
   vc_lock_create

SYNOPSIS
   void *vc_lock_create()

FUNCTION
   Create a lock. Returns the pointer to the lock. NULL if fails.
   We expect locks to be made at startup, without the worry of contention here.

RETURNS
   void
******************************************************************************/

void *vc_lock_create_on( struct list_head *list )
{
    unsigned long flags;

    Lock_t  *lock = kcalloc( 1,  sizeof( Lock_t ), GFP_KERNEL );

    if ( lock != NULL )
    {
        lock->magic = LOCK_MAGIC;
        INIT_LIST_HEAD(  &lock->node );
        init_MUTEX( &lock->sem );

#if VC_LOCK_DEBUG
        lock->fileName = "";
        lock->lineNum  = 0;
#endif
        spin_lock_irqsave( &gListLock, flags );
        list_add_tail( &lock->node, list );
        spin_unlock_irqrestore( &gListLock, flags );
    }

    return lock;

} // vc_lock_create_on

void *vc_lock_create( void )
{
    return vc_lock_create_on( &gLockList );

} // vc_lock_create

void vc_lock_free_list( struct list_head *list )
{
    unsigned long   flags;

    spin_lock_irqsave( &gListLock, flags );

    while ( !list_empty( list ))
    {
        struct list_head *node = list->next;
        Lock_t           *lock = list_entry( node, Lock_t, node );

        // Remove the lock from the list

        list_del( node );

        memset( lock, 0xEE, sizeof( *lock ));

        // Release the memory for the lock

        kfree( lock );
    }

    spin_unlock_irqrestore( &gListLock, flags );

} // vc_lock_free_list

/******************************************************************************
NAME
   vc_lock_obtain

SYNOPSIS
   int vc_lock_obtain(VC_LOCK_T *lock)

FUNCTION
   Obtain the given lock, blocking until we get it.

RETURNS
   void
******************************************************************************/

#if VC_LOCK_DEBUG
void vc_lock_obtain_dbg( void *voidLock, const char *fileName, int lineNum )
#else
void vc_lock_obtain( void *voidLock )
#endif
{
    Lock_t  *lock = (Lock_t *)voidLock;

    // Check to make sure we're not being passed a NULL lock. vcgencmd uses the
    // following sequence:
    //
    //   lock_grab();
    //
    //   if (gencmd_inum < 0) {
    //      lock_release();
    //      return -1;
    //   }
    //
    // and lock_grab calls lock_obtain with an uninitialized lock, so the only
    // way for the above to work is if this function allows for NULL locks.

    if ( lock != NULL )
    {
       BUG_ON( lock->magic != LOCK_MAGIC );

       down( &lock->sem );

#if VC_LOCK_DEBUG
       lock->fileName = fileName;
       lock->lineNum  = lineNum;
#endif
    }

} // vc_obtain_lock

/******************************************************************************
NAME
   vc_release_lock

SYNOPSIS
   void vc_release_lock(VC_LOCK_T *lock)

FUNCTION
   Release a lock.

RETURNS
   void
******************************************************************************/

void vc_lock_release( void *voidLock )
{
    Lock_t  *lock = (Lock_t *)voidLock;

    if ( lock != NULL )
    {
       BUG_ON( lock->magic != LOCK_MAGIC );

#if VC_LOCK_DEBUG
       lock->fileName = "";
       lock->lineNum  = 0;
#endif

       up( &lock->sem );
    }

} // vc_release_lock

/*---------------------------------------------------------------------------*/
/* Event related functions */
/*---------------------------------------------------------------------------*/
/******************************************************************************
NAME
   vc_event_create

SYNOPSIS
  void *vc_event_create(void)

FUNCTION
   Create (and clear) an event.  Returns a pointer to the event object.
   We expect events to be made at startup, without the worry of contention here.

RETURNS
   NULL if fails
******************************************************************************/

#if VC_EVENT_DEBUG
void *vc_event_create_on_dbg( struct list_head *list, const char *fileName, int lineNum )
#else
void *vc_event_create_on( struct list_head *list )
#endif
{
    unsigned long   flags;

    Event_t *event = kcalloc( 1, sizeof( Event_t ), GFP_KERNEL );

    if ( event != NULL )
    {
        static int eventNum = 0;

        event->index = ++eventNum;

        event->magic = EVENT_MAGIC;
        INIT_LIST_HEAD( &event->node );

        init_waitqueue_head( &event->waitQ );
        event->flag = 0;

#if VC_EVENT_DEBUG
        event->createFileName = fileName;
        event->createLineNum  = lineNum;
        event->fileName = "";
        event->lineNum  = 0;
#endif
        spin_lock_irqsave( &gListLock, flags );
        list_add_tail( &event->node, list );
        spin_unlock_irqrestore( &gListLock, flags );
    }

    return event;

} // vc_event_create_on

#if VC_EVENT_DEBUG
void *vc_event_create_dbg( const char *fileName, int lineNum )
{
    return vc_event_create_on_dbg( &gEventList, fileName, lineNum );

} // vc_event_create
#else
void *vc_event_create( void )
{
    return vc_event_create_on( &gEventList );

} // vc_event_create
#endif

void vc_event_free_list( struct list_head *list )
{
    unsigned long   flags;

    spin_lock_irqsave( &gEventLock, flags );

    while ( !list_empty( list ))
    {
        struct list_head *node = list->next;
        Event_t          *event = list_entry( node, Event_t, node );

        // Release any interrupt registrations for the event

        VC_DEBUG( Trace, "About to call vc_interface_unregister_event_int\n" );
        vc_interface_unregister_event_int( event );
        VC_DEBUG( Trace, "Back from vc_interface_unregister_event_int\n" );

        // Remove the event from the list

        list_del( node );

        // Nobody should be waiting for the event

#if VC_EVENT_DEBUG
        if ( waitqueue_active( &event->waitQ ))
        {
            printk( "event being waited on from: %s %d\n", event->fileName, event->lineNum );
        }
#endif
        BUG_ON( waitqueue_active( &event->waitQ ));

        memset( event, 0xEE, sizeof( *event ));

        // Release the memory for the event

        kfree( event );
    }

    spin_unlock_irqrestore( &gListLock, flags );

} // vc_event_free_list

/******************************************************************************
NAME
   vc_event_wait

SYNOPSIS
   void vc_event_wait(void *event)

FUNCTION
   Wait for an event to be set, blocking until it is set.
   Only one thread may be waiting at any one time.
   The event is automatically cleared on leaving this function.
   If the event is already set before this call, the function
   returns immediately.

RETURNS
   void
******************************************************************************/

#if VC_EVENT_DEBUG
void vc_event_wait_dbg( void *voidEvent, const char *fileName, int lineNum )
#else
void vc_event_wait( void *voidEvent )
#endif
{
    Event_t    *event = (Event_t *)voidEvent;

    BUG_ON( event->magic != EVENT_MAGIC );

Log( 'W', event );

    if ( gEventsBlocking )
    {
#if VC_EVENT_DEBUG
        event->fileName = fileName;
        event->lineNum  = lineNum;
#endif
        wait_event( event->waitQ, event->flag > 0 );

#if VC_EVENT_DEBUG
        event->fileName = "";
        event->lineNum  = 0;
#endif
    }
    else
    {
        set_current_state( TASK_INTERRUPTIBLE );
        schedule_timeout( 10 );
    }
    event->flag = 0;

Log( 'w', event );

} // vc_event_wait

/******************************************************************************
NAME
   vc_event_status

SYNOPSIS
   int vc_event_status(void *event)

FUNCTION
   Reads the state of an event (for polling systems)

RETURNS
   1 if event is set, 0 if clear
******************************************************************************/

int vc_event_status( void *voidEvent )
{
    Event_t *event = (Event_t *)voidEvent;

    BUG_ON( event->magic != EVENT_MAGIC );

    return event->flag != 0;

} // vc_event_status

/******************************************************************************
NAME
   vc_event_clear

SYNOPSIS
   void vc_event_clear(void *event)

FUNCTION
   Forcibly clears any pending event

RETURNS
   void
******************************************************************************/

void vc_event_clear( void *voidEvent )
{
    Event_t *event = (Event_t *)voidEvent;

    BUG_ON( event->magic != EVENT_MAGIC );

Log( 'C', event );

    event->flag = 0;

} // vc_event_clear

/******************************************************************************
NAME
   vc_event_set

SYNOPSIS
   void vc_event_set(void *event)

FUNCTION
   Sets a event - can be called from any thread.

RETURNS
   void
******************************************************************************/

void vc_event_set( void *voidEvent )
{
    Event_t *event = (Event_t *)voidEvent;

    BUG_ON( event->magic != EVENT_MAGIC );

Log( 'S', event );

    event->flag = 1;

    wake_up( &event->waitQ );

} // vc_event_set

/******************************************************************************
NAME
   vc_event_blocking

SYNOPSIS
   void vc_event_blocking(void)

FUNCTION
   Sets events to block until they are set.

RETURNS
   void
******************************************************************************/

void vc_event_blocking( void )
{
   gEventsBlocking = 1;

} // vc_event_blocking

