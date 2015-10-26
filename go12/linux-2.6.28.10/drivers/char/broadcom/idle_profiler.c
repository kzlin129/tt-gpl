/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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



#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/broadcom/idle_prof.h>
#include <linux/broadcom/knllog.h>
#include <linux/module.h>

#include <linux/proc_fs.h>
#include <linux/sysctl.h>

spinlock_t idle_lock = SPIN_LOCK_UNLOCKED;
u32 idle_count = 0;

#define DEFAULT_UPDATE_INTERVAL_MSEC 500
#define HISTORY_SIZE  10
typedef struct
{
   unsigned int minCount;
   unsigned int maxCount;
   unsigned int avgCount;
   unsigned int currCount;
   unsigned int countHistory[HISTORY_SIZE];
}
HOST_ICP_STATS;


typedef struct
{
   unsigned int enable;  /* CPU monitor on of off. Initialized at startup, read-only by kernel and write/read by user via proc entry */
   unsigned int reset;    /* Forced reset of stats, Intiailized at startup, read-only by kernel and write/read by user via proc entry */
   unsigned int updateIntervalMsec; /* Window size over which measurement is made. Initialized at startup, readonly by kernel, write/read by user via proc entry */
   unsigned int alphaFactor; /* Weightage of current value to the average (in %). Initialized at startup, readonly by kernel, write/read by user via proc entry */
   unsigned int resetInterval;  /* Resets the stats after this interval expires . Initialized at startup, readonly by kernel, write/read by user via proc entry */
   unsigned int resetTimer;  /* local timer to reset the stats, read/write by kernel, read only by user via proc entry */
   unsigned int iterations; /* read/write by lernel, read only by user via proc entry */
   HOST_ICP_STATS stats; /* Read/write by kernel, read only by user via proc entry */
}
HOST_ICP_STATE;

typedef struct
{
   unsigned int enable;         /* Load test on of off. Initialized at startup, read-only by kernel and write/read by user via proc entry */
   unsigned int loadPeriod;     /* Period of when the load test is run, in mSec */
   unsigned int loadPercentage; /* Load percentage (in %). */
   unsigned int useThread;      /* Test load using thread context. */
   unsigned int threadPriority; /* Priority of load testing thread */
   unsigned int timeSlices;     /* Number of time slices to spread the load across */
   unsigned int timerStarted;   /* Flag indicating whether kernel timer used for CPU loading has started */

   /* Load testing thread controls */
   int threadPid;
   struct semaphore threadSemaphore;
   struct completion threadCompletion;
}
HOST_ICP_LOAD_TEST;

typedef struct
{
    idle_handle_t   idle_handle;
    u32 idle;
    u32 total;
} ICPDATA;

static HOST_ICP_STATE gHostIcpState;
static HOST_ICP_STATS gHostIcpStats;
static HOST_ICP_LOAD_TEST gHostIcpLoadTest;
static ICPDATA gHostIcpData;
static  struct task_struct  *gHostIcpTask = NULL;

/* sysctl */
static  int     hostIcpThread( void *data );

static struct ctl_table_header *gSysCtlHeader;

static int proc_do_hosticp_intvec_clear(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );

static int proc_do_hosticp_loadtest_intvec_enable(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );

static struct ctl_table gSysCtlIcpStats[] = {
   {
      .ctl_name   = 1,
      .procname   = "minCpu",
      .data       = &gHostIcpStats.minCount,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 2,
      .procname   = "maxCpu",
      .data       = &gHostIcpStats.maxCount,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 3,
      .procname   = "avgCpu",
      .data       = &gHostIcpStats.avgCount,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 4,
      .procname   = "currCpu",
      .data       = &gHostIcpStats.currCount,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 5,
      .procname   = "history",
      .data       = &gHostIcpStats.countHistory,
      .maxlen     = HISTORY_SIZE * sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 6,
      .procname   = "summary",
      .data       = &gHostIcpStats,
      .maxlen     = sizeof( HOST_ICP_STATS ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtlIcpLoadTest[] = {
   {
      .ctl_name   = 1,
      .procname   = "enable",
      .data       = &gHostIcpLoadTest.enable,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_do_hosticp_loadtest_intvec_enable
   },
   {
      .ctl_name   = 2,
      .procname   = "loadPeriod",
      .data       = &gHostIcpLoadTest.loadPeriod,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 3,
      .procname   = "loadPercentage",
      .data       = &gHostIcpLoadTest.loadPercentage,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 4,
      .procname   = "useThread",
      .data       = &gHostIcpLoadTest.useThread,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 5,
      .procname   = "threadPriority",
      .data       = &gHostIcpLoadTest.threadPriority,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 6,
      .procname   = "timeSlices",
      .data       = &gHostIcpLoadTest.timeSlices,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtlIcp[] = {
   {
      .ctl_name   = 1,
      .procname   = "enable",
      .data       = &gHostIcpState.enable,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 2,
      .procname   = "reset",
      .data       = &gHostIcpState.reset,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_do_hosticp_intvec_clear
   },
   {
      .ctl_name   = 3,
      .procname   = "alpha-factor",
      .data       = &gHostIcpState.alphaFactor,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 4,
      .procname   = "resetInterval",
      .data       = &gHostIcpState.resetInterval,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 5,
      .procname   = "stats",
      .child      = gSysCtlIcpStats,
      .mode       = 0555,
   },
   {
      .ctl_name   = 6,
      .procname   = "state",
      .data       = &gHostIcpState,
      .maxlen     = sizeof( HOST_ICP_STATE ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 7,
      .procname   = "loadTest",
      .child      = gSysCtlIcpLoadTest,
      .mode       = 0555,
   },
   {}
};
static ctl_table gSysCtl[] = {
	{
		.ctl_name	= 1,
		.procname	= "hostUsage",
		.mode		   = 0555,
		.child		= gSysCtlIcp
	},
	{}
};

// Actually create (and remove) the /proc file(s).
static void icp_create_proc(void)
{
   struct proc_dir_entry *entry;
   entry = create_proc_entry("hostUsage", 0, NULL);
}
static void icp_remove_proc(void)
{
   /* no problem if it was not registered */
   remove_proc_entry("hostUsage", NULL);
}

// If we are writing the clear field, we reset the stats and start logging
static int proc_do_hosticp_intvec_clear(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      memset(&gHostIcpStats, 0, sizeof(HOST_ICP_STATS));
      memset(&gHostIcpState.stats, 0, sizeof(HOST_ICP_STATS));
      gHostIcpState.stats.minCount = -1;
      gHostIcpState.alphaFactor = (1<< 4);   /* 1/16 in Q8 number */
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}

#define MAX_LOAD_PERIOD 1000
#define MAX_LOAD_PERCENTAGE 98
#define MAX_TIME_SLICES 50
static void ipc_load( void )
{
   static int load = 0;
   static int slice = 0;
   int load_this_slice;
   int loops_this_slice;

   /*
    * The test load is implemented in fixed length udelay loops.  The target load percentage
    * is specifed by loadPercentage, and can be spread across multiple time slots in the following
    * manner:
    *
    * 1 time slot, 60% load:
    *
    * | 60% | 60% | 60%| ...
    *
    * 5 time slots, 60% load:
    *
    * | 98% | 98% | 98% |  6% |  0% | 98% | 98% | 98% |  6% |  0%| ...
    *
    * The maximum load of 98% per slot is enforced to prevent system deadlocks.
    *
    */
   if (slice == 0)
   {
      slice = gHostIcpLoadTest.timeSlices;
      load = gHostIcpLoadTest.loadPercentage << 4;
   }
   load_this_slice = (MAX_LOAD_PERCENTAGE << 4) / gHostIcpLoadTest.timeSlices;
   if (load < load_this_slice)
      load_this_slice = load;
   load -= load_this_slice;
   load_this_slice = (load_this_slice * gHostIcpLoadTest.timeSlices) >> 4;

   /*
    * Convert load percentage for this slot into number of 10uSec loops, which is conveniently
    * just multiplying the load percentage to the loop period in mSec.
    */
   loops_this_slice = (load_this_slice * gHostIcpLoadTest.loadPeriod);
   while (loops_this_slice)
   {
      udelay( 10 );
      loops_this_slice--;
   }

   slice--;
}

static int ipc_load_test_thread(void *data)
{
   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   if (gHostIcpLoadTest.threadPriority > 0)
   {
      struct sched_param param;
      param.sched_priority = (gHostIcpLoadTest.threadPriority < MAX_RT_PRIO)?
                              gHostIcpLoadTest.threadPriority:(MAX_RT_PRIO-1);
      sched_setscheduler(current, SCHED_FIFO, &param);
   }
   daemonize("loadTest");
   allow_signal(SIGKILL);
   allow_signal(SIGTERM);

   /* Run until signal received */
   while (1) {
      if ( down_interruptible (&gHostIcpLoadTest.threadSemaphore) == 0 )
      {
         ipc_load();
      }
      else
         break;
   }
   complete_and_exit(&gHostIcpLoadTest.threadCompletion, 0);
}

static void ipc_load_timer_func(ulong data)
{
   struct timer_list *timer;

   timer = (struct timer_list *)data;

   if (gHostIcpLoadTest.timerStarted)
   {
      if (gHostIcpLoadTest.loadPeriod > MAX_LOAD_PERIOD)
         gHostIcpLoadTest.loadPeriod = MAX_LOAD_PERIOD;

      timer->expires += msecs_to_jiffies(gHostIcpLoadTest.loadPeriod);
      add_timer(timer);


      if (gHostIcpLoadTest.timeSlices < 1)
         gHostIcpLoadTest.timeSlices = 1;
      if (gHostIcpLoadTest.timeSlices > MAX_TIME_SLICES)
         gHostIcpLoadTest.timeSlices = MAX_TIME_SLICES;
      if (gHostIcpLoadTest.loadPercentage > MAX_LOAD_PERCENTAGE)
         gHostIcpLoadTest.loadPercentage = MAX_LOAD_PERCENTAGE;

      if (gHostIcpLoadTest.useThread)
         up(&gHostIcpLoadTest.threadSemaphore);
      else
         ipc_load();
   }
}

// If we are writing the enable field, we start/stop the kernel timer
static int proc_do_hosticp_loadtest_intvec_enable(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   static struct timer_list timer = TIMER_INITIALIZER(ipc_load_timer_func, 0, (ulong)&timer);
   int rc;

   if ( !table || !table->data )
       return -EINVAL;

   if ( write )
   {
      // use generic int handler to get input value
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

      if (rc < 0)
         return rc;

      if (!gHostIcpLoadTest.timerStarted && gHostIcpLoadTest.enable)
      {
         if (gHostIcpLoadTest.useThread)
         {
            sema_init (&gHostIcpLoadTest.threadSemaphore, 0);
            init_completion(&gHostIcpLoadTest.threadCompletion);
            gHostIcpLoadTest.threadPid = kernel_thread(ipc_load_test_thread, NULL, 0);
         }
         if (gHostIcpLoadTest.loadPeriod > MAX_LOAD_PERIOD)
            gHostIcpLoadTest.loadPeriod = MAX_LOAD_PERIOD;
         timer.expires = jiffies + msecs_to_jiffies(gHostIcpLoadTest.loadPeriod);
         add_timer(&timer);
         gHostIcpLoadTest.timerStarted = 1;
      }
      else if (gHostIcpLoadTest.timerStarted && !gHostIcpLoadTest.enable)
      {
         gHostIcpLoadTest.timerStarted = 0;
         // Kill load testing thread
         if (gHostIcpLoadTest.useThread)
         {
            if (gHostIcpLoadTest.threadPid >= 0)
            {
               kill_proc_info(SIGTERM, SEND_SIG_PRIV, gHostIcpLoadTest.threadPid);
               wait_for_completion(&gHostIcpLoadTest.threadCompletion);
            }
            gHostIcpLoadTest.threadPid = -1;
         }
      }
   }
   else
   {
      // nothing special for read
      return proc_dointvec( table, write, filp, buffer, lenp, ppos );
   }
   return rc;
}


// Initialize by setting up the sysctl and proc/knllog entries, allocating
// default storage and resetting the variables.
static int __init host_cpu_usage_init(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   /* register sysctl table */
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
   if ( gSysCtlHeader != NULL )
   {
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif

   icp_create_proc();

   memset(&gHostIcpState, 0, sizeof(HOST_ICP_STATE));   // zero entries, idx, wrap, and enable
   memset(&gHostIcpStats, 0, sizeof(HOST_ICP_STATS));   // zero entries, idx, wrap, and enable
   gHostIcpState.stats.minCount = -1;
   gHostIcpState.enable = 1;
   gHostIcpState.updateIntervalMsec = DEFAULT_UPDATE_INTERVAL_MSEC;
   gHostIcpState.alphaFactor = (1<< 4);   /* 1/16 in Q8 number */
   arch_init_idle_profile(&gHostIcpData.idle_handle);

   /* Launch a kernel thread */
   if (( gHostIcpTask == NULL ) || IS_ERR( gHostIcpTask ))
    {
        gHostIcpTask = kthread_run( hostIcpThread, NULL, "hosticp" );
        if ( IS_ERR( gHostIcpTask ))
        {
            printk( KERN_ERR "Init: failed to start host ICP thread: %ld\n", PTR_ERR( gHostIcpTask ));
            return -1;
        }
    }

   gHostIcpLoadTest.enable = 0;
   gHostIcpLoadTest.loadPeriod = 10;
   gHostIcpLoadTest.loadPercentage = 0;
   gHostIcpLoadTest.useThread = 1;
   gHostIcpLoadTest.threadPriority = 99;
   gHostIcpLoadTest.timeSlices = 1;
   gHostIcpLoadTest.timerStarted = 0;
   gHostIcpLoadTest.threadPid = -1;
   printk("calling host_cpu_usage_init\n");
   return 0;
}
subsys_initcall(host_cpu_usage_init);

// Exit and cleanup (probably not done)
int __exit host_cpu_usage_exit(void)
{
   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
   icp_remove_proc();
   kthread_stop(gHostIcpTask);

   return 0;
}

//subsys_exitcall(host_cpu_usage_exit);


int hostIcpThread(void *data)
{
   printk("*************Starting host ICP thread **************   \n");
   while( 1 )
   {

      set_current_state(  TASK_INTERRUPTIBLE );
      schedule_timeout( ( (HZ * gHostIcpState.updateIntervalMsec) / 1000 ) + 1 );


      if (!(gHostIcpState.enable))
      {
         continue;
      }
      {
         unsigned int temp[HISTORY_SIZE-1];
         memcpy(&temp[0], &gHostIcpStats.countHistory[0], HISTORY_SIZE*sizeof(unsigned int) );
         memcpy(&gHostIcpStats.countHistory[1], &temp[0], (HISTORY_SIZE-1)*sizeof(unsigned int));
         gHostIcpStats.countHistory[0]=gHostIcpStats.currCount;
      }
      arch_get_idle_profile( &gHostIcpData.idle_handle, &gHostIcpData.idle, &gHostIcpData.total );

      if (gHostIcpData.total < (0xFFFFFFFF / 1000))
      {
         gHostIcpState.stats.currCount =  (1000 * gHostIcpData.idle)/(gHostIcpData.total);
      }
      else
      {
         gHostIcpState.stats.currCount =  (gHostIcpData.idle)/(gHostIcpData.total / 1000);
      }
      if (gHostIcpState.stats.minCount > gHostIcpState.stats.currCount)
      {
         gHostIcpState.stats.minCount = gHostIcpState.stats.currCount;
      }
      if (gHostIcpState.stats.maxCount < gHostIcpState.stats.currCount)
      {
         gHostIcpState.stats.maxCount = gHostIcpState.stats.currCount;
      }

      gHostIcpState.stats.avgCount = ((gHostIcpState.stats.currCount*gHostIcpState.alphaFactor) \
                     +(gHostIcpState.stats.avgCount * ((1 << 8) - gHostIcpState.alphaFactor))) >> 8;
      gHostIcpStats.currCount = 1000 - gHostIcpState.stats.currCount;
      gHostIcpStats.minCount = 1000 - gHostIcpState.stats.maxCount;
      gHostIcpStats.maxCount = 1000 - gHostIcpState.stats.minCount;
      gHostIcpStats.avgCount = 1000 - gHostIcpState.stats.avgCount;
      gHostIcpState.iterations++;

      gHostIcpState.resetTimer += gHostIcpState.updateIntervalMsec;

      if ((gHostIcpState.resetTimer > gHostIcpState.resetInterval) && (gHostIcpState.resetInterval))
      {
         memset(&gHostIcpState.stats, 0, sizeof(HOST_ICP_STATS));
         gHostIcpState.stats.minCount = -1;
         gHostIcpState.resetTimer = 0;

      }

   }
   return 0;
}

