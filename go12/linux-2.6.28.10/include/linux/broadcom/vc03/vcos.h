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



#ifndef VCHIOS_H
#define VCHIOS_H
#include <linux/version.h>
#if defined( __KERNEL__ )

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <asm/bitops.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vc03b0_hostport.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>

#else // __KERNEL__

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <semaphore.h>

#endif



/*******************************************************************
 *
 *  OS primitives common to user- and kernel mode
 *
 *******************************************************************/
typedef enum
{
   OS_SEMAPHORE_TYPE_MIN,

   OS_SEMAPHORE_TYPE_SUSPEND,
   OS_SEMAPHORE_TYPE_BUSY_WAIT,

   OS_SEMAPHORE_TYPE_MAX

} OS_SEMAPHORE_TYPE_T;

typedef uint32_t fourcc_t; 
typedef uint32_t FOURCC_T; 

typedef struct tag_VC_RECT_T {
   int32_t x;
   int32_t y;
   int32_t width;
   int32_t height;
} VC_RECT_T;

typedef enum bool_e
{
   VC_FALSE = 0,
   VC_TRUE = 1,
} bool_t;

// Broadcom types not defined as standard in Linux
typedef int32_t            intptr_t;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26) 
typedef uint32_t           uintptr_t;
#endif

#if defined( __KERNEL__ )

/*******************************************************************
 *
 *  Kernel mode OS layer
 *
 * os_thread_start tasks have a prototype of
 *
 *  void thread_func( unsigned int argc, void **argv )
 *
 * while kernel functions have a prototype of:
 *
 *  int thread_func( void *data )
 *
 *******************************************************************/

#define OS_THREAD_T struct task_struct*
/*
typedef void (*OS_TASK_FUNCPTR)(unsigned int argc, char **argv );

int os_thread_start_helper( void *data );

#define os_thread_start(threadptr, task, arg, stack_size, name) \
  ({							   \
    *(threadptr) = kthread_run(os_thread_start_helper, (void *)task, name);	   \
    0;                                                     \
  })

#define os_thread_start_param(threadptr, task, name, param)		   \
  ({							   \
    *(threadptr) = kthread_run(task, param, name);	   \
    0;                                                     \
  })

#define os_thread_stop(threadptr) \
  ({				  \
    kthread_stop(threadptr);	  \
    0;                            \
  })
*/
extern int os_thread_start(OS_THREAD_T *threadptr, void *task, int arg, int stack_size, char *name);
extern int os_thread_start(OS_THREAD_T *threadptr, void *task, int arg, int stack_size, char *name);
extern int os_thread_stop(OS_THREAD_T *threadptr) ;


#define os_thread_should_stop() kthread_should_stop()

#define os_thread_current(threadptr) (get_current() == (threadptr))

#define OS_SEMAPHORE_T struct semaphore 

// vchi assumes the semaphore is created "unlocked" 
static inline int  os_semaphore_create(OS_SEMAPHORE_T* sem_ptr, OS_SEMAPHORE_TYPE_T type)
{	
  sema_init(sem_ptr, 1);                  
  return 0;                                      
}

static inline int os_semaphore_obtain(OS_SEMAPHORE_T* sema)
{
  int ret = down_interruptible(sema) ? -1 : 0; 
  if ( ret )
  {
      printk( KERN_ERR "os_semaphore_obatin: down_interruptible failed\n" );
  }
  return ret;
}

#define os_semaphore_obtained(sema)  1

static inline int os_semaphore_release(OS_SEMAPHORE_T* sema) 
{ 
  up(sema); 
  return 0; 
}

#define os_semaphore_destroy(sema) (void)0

#define OS_COUNT_SEMAPHORE_T struct semaphore

#define os_count_semaphore_create(sem_ptr, count, type)	\
  ({					                \
   sema_init(sem_ptr, count);                           \
   0;                                                   \
  }) 

#define os_count_semaphore_destroy(sem_ptr, count, type) 0

#define os_count_semaphore_obtain(sema, block)  \
  ({                                            \
    int _ret;                                   \
    if(block)					\
      _ret = down_interruptible(sema);	        \
    else                                        \
      _ret = down_trylock(sema);                \
    _ret;                                       \
  })

#define os_count_semaphore_release(sema) \
  ({                                     \
    up(sema);                            \
    0;                                   \
  })


/*
 * HISR's
 */ 
typedef void (*hisr_fn)(void);

typedef struct OS_HISR
{
  struct task_struct* taskptr;
  wait_queue_head_t wq; 
  hisr_fn taskfn;
  int run;

} OS_HISR_T;

static int vchi_lin_hisrloop(void* param)
{
  OS_HISR_T* ptr;

  ptr = (OS_HISR_T*) param;
  
  for(;;) 
    {       
      wait_event_interruptible((ptr->wq), ptr->run);
      if(kthread_should_stop())
	    break;

      {
          unsigned long flags;
          int           runWasSignalled;

          local_irq_save( flags );
          runWasSignalled = ptr->run;
          ptr->run = 0;
          local_irq_restore( flags );

          if ( runWasSignalled )
          {
              (*(ptr->taskfn))(); 
          }
      }
    }

  return 0;
}

static inline int os_hisr_create(OS_HISR_T* hisr, hisr_fn taskfn, const char* name)    
{			
  init_waitqueue_head(&hisr->wq);
  hisr->taskfn = taskfn;
  hisr->run = 0;
  hisr->taskptr = kthread_run(vchi_lin_hisrloop, hisr, name);
  if(IS_ERR(hisr->taskptr))
    {
      return -1;
    }

  return 0;
} 
  
static inline int os_hisr_activate(OS_HISR_T* hisr)
{
  hisr->run = 1;
  wake_up_all(&hisr->wq);
  return 0;
}

/*
*
*/
#define os_time_init()
#define os_time_term()
#define os_get_time_in_usecs() jiffies_to_usecs(jiffies)
#define os_sleep(msec)					\
  ({							\
   set_current_state(  TASK_INTERRUPTIBLE );		\
   schedule_timeout( msecs_to_jiffies( msec ));		\
  })

#define os_delay(msec) os_sleep(msec)

/*
 *
 */
typedef struct  OS_EVENTGROUP
{
  wait_queue_head_t wq;
  volatile int bits;
} OS_EVENTGROUP_T;
 
#define os_eventgroup_create(eventgroup, name)		\
  ({							\
    (eventgroup)->bits = 0;				\
    init_waitqueue_head(&(eventgroup)->wq);		\
    0;							\
  })

#define os_eventgroup_signal(eventgroup, bit) \
  ({ \
    unsigned long flags;					\
    assert((bit) < 32);                     \
    local_irq_save( flags );                \
    (eventgroup)->bits |= (1 << (bit));	    \
    local_irq_restore( flags );             \
    wake_up_all(&(eventgroup)->wq);			\
    0; \
  })

#define os_eventgroup_signalmask(eventgroup, bitmask) \
  ({ \
    unsigned long flags;				\
    assert((bit) < 32);                 \
    local_irq_save( flags );            \
    (eventgroup)->bits |= (bitmask);	\
    local_irq_restore( flags );         \
    wake_up_all(&(eventgroup)->wq);		\
    0; \
  })

// wait for any events 
#define os_eventgroup_retrieve(eventgroup, events)              \
  ({ \
    unsigned long flags;                                        \
    wait_event((eventgroup)->wq, ((eventgroup)->bits != 0));	\
    local_irq_save( flags );                                    \
    *(events) = (eventgroup)->bits;				                \
    (eventgroup)->bits = 0;                                     \
    local_irq_restore( flags );                                 \
    0; \
  })

// wait for any events 
#define os_eventgroup_retrieve_to(eventgroup, events, timeout)                          	\
  ({                                                                                            \
    long _os_ret = wait_event_timeout((eventgroup)->wq, ((eventgroup)->bits != 0), timeout);	\
    if(_os_ret > 0)                         \
    { \
        unsigned long flags;                \
        local_irq_save( flags );            \
        *(events) = (eventgroup)->bits;		\
	    (eventgroup)->bits = 0;             \
        local_irq_restore( flags );         \
    } \
    else \
    { \
	    *(events) = 0;                      \
    } \
    _os_ret;                                \
  });

#define os_event_wait os_eventgroup_retrieve
#define os_event_wait_to os_eventgroup_retrieve_to
#define os_event_signal os_eventgroup_signal

//#define os_init() ({0;})

void *vcos_alloc( size_t size, const char *name, const char *function, const char *fileName, int lineNum );
void  vcos_free( void *mem );

#define os_malloc(size, align, name) vcos_alloc(size, name, __FUNCTION__, __FILE__, __LINE__ )
#define os_free(addr) vcos_free(addr)
#define malloc(size) vcos_alloc(size, "malloc", __FUNCTION__, __FILE__, __LINE__ )
#define free(addr) vcos_free(addr) 

#define vc_assert( expr ) \
  ((void)((expr) || ( printk( KERN_ERR "k[%s:%d] ***** Assertion '%s' failed\n", __FUNCTION__, __LINE__, #expr), 0 )))

//#define os_logging_message(fmt, args...) printk("k[%s:%d]: " fmt "\n", __FUNCTION__, __LINE__, ##args)
#if (CFG_GLOBAL_CHIP == BCM2153)
#define timer_get_tick_count get_gptimer_usec
#endif
extern u32 timer_get_tick_count( void );
#define VMCSLOG_ENTRY_LENGTH 128

extern char logdata[];
extern uint32_t vmcslog_count;
extern int vmcslog_add(char* data, int len);
#if 0
#define os_logging_message(fmt, args...) \
  { snprintf(logdata, VMCSLOG_ENTRY_LENGTH, \
            "[%d:%u] k[%s:%d]: " fmt "\n", \
            vmcslog_count++, timer_get_tick_count(), \
            __FUNCTION__, __LINE__, ##args); \
     vmcslog_add(logdata,  strlen(logdata));}
#else
#define os_logging_message(fmt, args...)    VC_DEBUG( Trace, fmt, ##args )
#endif

#define os_assert vc_assert
#define assert vc_assert
#define OS_ALIGN_DEFAULT   0
#define OS_COUNT(arg) 1


#define OS_MIN( a, b )     (( (a) < (b) ) ? (a) : (b) )
#define OS_MAX( a, b )     (( (a) > (b) ) ? (a) : (b) )
#define _min OS_MIN 
#define OS_NOP() 

#define rand() get_random_int()
#define RAND_MAX 0x7fffffff


typedef struct opaque_os_cond_t *OS_COND_T;
//typedef struct opaque_os_semaphore_t OS_SEMAPHORE_T;

extern int32_t os_init(void);
extern int32_t os_thread_set_priority( OS_THREAD_T thread, uint32_t priority );
extern int os_semaphore_obtain_global(void);
extern int os_semaphore_release_global(void);

extern int32_t os_cond_create( OS_COND_T *condition, OS_SEMAPHORE_T *semaphore );
extern int32_t os_cond_wait( OS_COND_T *cond, OS_SEMAPHORE_T *semaphore );
extern int32_t os_cond_broadcast( OS_COND_T *cond, bool_t sem_claimed );
extern int32_t os_cond_destroy( OS_COND_T *cond );
extern int32_t os_cond_signal( OS_COND_T *cond, bool_t sem_claimed );

#else  // KERNEL

/*******************************************************************
 *
 *  User mode OS layer
 *
 *******************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>


#define os_malloc(size, align, name) malloc(size)
#define OS_SEMAPHORE_T sem_t 

#endif // KERNEL

#endif // VCHIOS_H
