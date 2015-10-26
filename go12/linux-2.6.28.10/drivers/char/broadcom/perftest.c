/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
*****************************************************************************
*
*  perftest.c
*
*  PURPOSE:
*  performance tests
*
*  NOTES:
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/string.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/version.h>
#include <linux/broadcom/perfcnt.h>
#include <linux/interrupt.h>
#include <linux/broadcom/timer.h>
#include <linux/mm.h>
#include <linux/completion.h>

/* ---- Constants and Types ---------------------------------------------- */


// The perfcnt object
typedef struct
{
   int verbose;
   int memcpysize;         // memcpy size
   int enableint;          // enable interrupt
   int disableint;         // disable interrupt
   int memcpy;             // memcpy test
   int nop;                // nop test
   int noploop;            // nop at lowest prio
   int aramtest;           // aramtest at lowest prio
}
PERFTEST_OBJ;

// storage for the perfcnt object
static PERFTEST_OBJ perftest;

#define nop1() asm(" nop");
#define nop2() nop1() nop1();
#define nop4() nop2() nop2();
#define nop8() nop4() nop4();
#define nop16() nop8() nop8();
#define nop32() nop16() nop16();
#define nop64() nop32() nop32();
#define nop128() nop64() nop64();
#define nop256() nop128() nop128();
#define nop512() nop256() nop256();
#define nop1024() nop512() nop512();
#define nop2048() nop1024() nop1024();
#define nop4096() nop2048() nop2048();
#define nop8192() nop4096() nop4096();
#define nop16k() nop8192() nop8192();
#define nop32k() nop16k() nop16k();
#define nop64k() nop32k() nop32k();

static uint32_t tmrStart;
static uint32_t elapsedTimeTotal;

void *mymemcpy( void *dest, const void *src, unsigned int n);

static int nop_pid = -1;
static int aram_pid = -1;

/* sysctl */
static  struct ctl_table_header    *gSysCtlHeader;

static int proc_do_perftest_intvec_memcpy(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_nop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_enableint(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_disableint(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_memcpysize(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_noploop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perftest_intvec_aramtest(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );

static struct ctl_table gSysCtlPerftest[] = {

   {
      .ctl_name      = 1,
      .procname      = "memcpy",
      .data          = &perftest.memcpy,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_memcpy
   },
   {
      .ctl_name      = 2,
      .procname      = "nop",
      .data          = &perftest.nop,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_nop
   },
   {
      .ctl_name      = 3,
      .procname      = "enableint",
      .data          = &perftest.enableint,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_enableint
   },
   {
      .ctl_name      = 4,
      .procname      = "disableint",
      .data          = &perftest.disableint,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_disableint
   },
   {
      .ctl_name      = 5,
      .procname      = "memcpysize",
      .data          = &perftest.memcpysize,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_memcpysize
   },
   {
      .ctl_name      = 6,
      .procname      = "verbose",
      .data          = &perftest.verbose,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 7,
      .procname      = "noploop",
      .data          = &perftest.noploop,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_noploop
   },
   {
      .ctl_name      = 8,
      .procname      = "aramtest",
      .data          = &perftest.aramtest,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perftest_intvec_aramtest
   },
   {}
};

static struct ctl_table gSysCtl[] = {
   {
      .ctl_name = CTL_BCM_PERFTEST,
      .procname = "perftest",
      .mode     = 0555,
      .child    = gSysCtlPerftest
   },
   {}
};

static inline void clean_invalidate_dcache(void)
{
   uint32_t val = 0;
   asm volatile("mcr p15, 0, %[val], c7, c14, 0" : : [val] "r" (val));
}

/****************************************************************************/
/**
*  @brief   initialize performance counters and timer
*
*  @return
*/
/****************************************************************************/
static void perf_init_start( ARM_PERF_EVT evt0, ARM_PERF_EVT evt1 )
{
   arm_perf_init_start(evt0, evt1);
   tmrStart  = timer_get_tick_count();
}

/****************************************************************************/
/**
*  @brief   initialize performance counters and timer
*
*  @return
*/
/****************************************************************************/
static void perf_clear( void )
{
   arm_perf_clear();
   elapsedTimeTotal = 0;
}

/****************************************************************************/
/**
*  @brief   initialize performance counters and timer
*
*  @return
*/
/****************************************************************************/
static void perf_stop_save( void )
{
   int elapsed;
   int tmrEnd;

   arm_perf_stop_save();
   tmrEnd = timer_get_tick_count();
   elapsed  = tmrEnd - tmrStart;
   if (elapsed < 0)
   {
      elapsed = tmrEnd - (int)tmrStart;
   }
   elapsedTimeTotal += (uint32_t)elapsed;
}

/****************************************************************************/
/**
*  @brief   initialize performance counters and timer
*
*  @return
*/
/****************************************************************************/
static void perf_print_bus_busy( int (*fpPrint) (const char *, ...) )   //<  [ IN ] Print callback function
{
   unsigned int idle;
   unsigned int timer;

#if defined(CONFIG_ARCH_FPGA11107)
   // fpga uses 150MHz free running timer; idle counter counts at 10MHz
   idle = 1;
   timer = 15;
#else
   // asic uses 150MHz free running timer; idle counter counts at 300MHz
   idle = 2;
   timer = 1;
#endif

   arm_perf_print_bus_busy(idle, timer, elapsedTimeTotal, fpPrint);
}

/****************************************************************************/
/**
*  @brief   Simple profiled nop test of a certain block size and a number of iterations.
*
*  @return
*/
/****************************************************************************/
static void nop_perf_test(int iterations, int verbose, char *banner)
{
   int i = 0;
   int j;
   perf_clear();
   while (arm_perf_evt_pair_tbl[i].evt0 != (ARM_PERF_EVT)~0)
   {
      perf_init_start(arm_perf_evt_pair_tbl[i].evt0, arm_perf_evt_pair_tbl[i].evt1);
      for (j=0; j<iterations; j++)
      {
         nop64k();
      }
      perf_stop_save();
      if (verbose >= 2)
      {
         arm_perf_print_mon_cnts(printk, verbose);
      }
      i++;
   }
   perf_print_bus_busy(printk);
   arm_perf_print_summary(printk, banner, verbose);
}
/****************************************************************************/
/**
*  @brief   Simple profiled memcpy test of a certain block size and a number of iterations.
*
*  @return
*/
/****************************************************************************/
static void memcpy_perf_test(char *da, char *sa, size_t n, uint32_t iterations, int verbose, char *banner)
{
   uint32_t iter;
   int i = 0;
   char modbanner[80];

   perf_clear();
   clean_invalidate_dcache();
   while (arm_perf_evt_pair_tbl[i].evt0 != (ARM_PERF_EVT)~0)
   {
      perf_init_start(arm_perf_evt_pair_tbl[i].evt0, arm_perf_evt_pair_tbl[i].evt1);
      for (iter = 0; iter < iterations; iter++)
      {
         mymemcpy(da, sa, n);
      }
      perf_stop_save();
      if (verbose >= 2)
      {
         arm_perf_print_mon_cnts(printk, verbose);
      }
      i++;
   }
   perf_print_bus_busy(printk);
   sprintf(modbanner, "%s size = 0x%05x, iterations = %d : ", banner, n, iterations);
   arm_perf_print_summary(printk, modbanner, verbose);
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >memcpy - perform memcpy test
*
*  @return
*/
/****************************************************************************/
static int proc_do_perftest_intvec_memcpy(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;
   char *da;
   char *sa;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      da = kmalloc(perftest.memcpysize, GFP_KERNEL);
      sa = kmalloc(perftest.memcpysize, GFP_KERNEL);
      if (!da || !sa)
      {
         printk("memcpy test: failed to allocate memory\n");
         if (da)
         {
            kfree(da);
         }
         if (sa)
         {
            kfree(sa);
         }
         return -1;
      }
      printk("da = 0x%08x, sa = 0x%08x\n", (unsigned int)da, (unsigned int)sa);
      memcpy_perf_test(da, sa, perftest.memcpysize, perftest.memcpy, perftest.verbose, "mempcy test ddr ");
      kfree(sa);
      kfree(da);

   }

   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >nop - run 64k nop test
*
*  @return
*/
/****************************************************************************/
static int proc_do_perftest_intvec_nop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      nop_perf_test(perftest.nop, perftest.verbose, "64k nop test");
   }

   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 0x1000 >memcpysize - configure memcpy size
*
*  @return
*/
/****************************************************************************/
static int proc_do_perftest_intvec_memcpysize(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      if (perftest.memcpysize > 0x10000)
      {
         printk("memcpy size too large (must be 0x10000 or less)\n");
         perftest.memcpysize = 0x1000;
      }
   }

   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >enableint - re-enable interrupts (irq and fiq)
*
*  @return
*/
/****************************************************************************/
static int proc_do_perftest_intvec_enableint(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      local_irq_enable();
   }

   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >disableint - disable interrupts (irq and fiq)
*
*  @return
*/
/****************************************************************************/
static int proc_do_perftest_intvec_disableint(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      local_irq_disable();
   }

   return rc;
}


/****************************************************************************/
/**
*  @brief   low priority noploop to avoid CPU sleeping and stress i-cache
*
*  @return
*/
/****************************************************************************/
static int noploop(void *data)
{
   // Set this thread to idle policy, and it's priority is hence is below all other
   // default policy threads

   struct sched_param param = { .sched_priority = 0 };

   int rc = sched_setscheduler(current, SCHED_IDLE, &param);
   if (rc)
   {
      printk(KERN_ERR "noploop sched_setscheduler failed, rc=%d\n", rc);
      return rc;
   }

   daemonize("noploop");

   while (1)
   {
      nop64k();
   }
   return 0;
}
/****************************************************************************/
/**
*  @brief   Run low priority noploop to avoid CPU sleeping and stress i-cache
*
*  @return
*/
/****************************************************************************/

static int proc_do_perftest_intvec_noploop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      if (perftest.noploop > 0)
      {
         if (nop_pid == -1)
         {
            nop_pid = kernel_thread(noploop, 0, 0);
            printk("PID %d created for nop loop\n", nop_pid);
         }
         else
         {
            printk(KERN_ERR "noploop already running with PID %d\n", nop_pid);
         }
      }
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   low priority aramtest to stress aram
*
*  @return
*/
/****************************************************************************/
static unsigned int pattern = 0;
static int aramtest(void *data)
{
   // Set this thread to idle policy, and it's priority is hence is below all other
   // default policy threads
   struct sched_param param = { .sched_priority = 0 };

   int rc = sched_setscheduler(current, SCHED_IDLE, &param);
   if (rc)
   {
      printk(KERN_ERR "aramtest sched_setscheduler failed, rc=%d\n", rc);
      return rc;
   }
   daemonize("aramtest");
   printk("aramtest starting, aramtest=%d\n", perftest.aramtest);
   while (1)
   {
      // Write incrementing pattern to ARAM, validate, then increment starting word.
      unsigned int val = pattern;
      unsigned int *dst = (unsigned int *)MM_IO_BASE_ARAM;
      unsigned int size = 0x10000;     // 64 KB
      unsigned int i;
      for (i = 0; i<size; i += sizeof(*dst))
      {
         *dst++ = val++;
      }
      val = pattern;
      dst = (unsigned int *)MM_IO_BASE_ARAM;
      for (i = 0; i<size; i += sizeof(*dst), dst++, val++)
      {
         if (*dst != val)
         {
            printk(KERN_ERR "ERROR: %s: dst=0x%p, *dst=0x%x, good=0x%x\n", __FUNCTION__, dst, *dst, val);
         }
      }
      pattern++;
   }
}
/****************************************************************************/
/**
*  @brief   low priority aramtest to stress aram
*
*  @return
*/
/****************************************************************************/

static int proc_do_perftest_intvec_aramtest(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
   if (rc < 0)
   {
      return rc;
   }

   if ( write )
   {
      if (perftest.aramtest > 0)
      {
         if (aram_pid == -1)
         {
            aram_pid = kernel_thread(aramtest, 0, 0);
            printk("PID %d created for aram test\n", aram_pid);
         }
         else
         {
            printk(KERN_ERR "aramtest already running with PID %d\n", aram_pid);
         }
      }
   }
   return rc;
}

/****************************************************************************/
/**
*  @brief   Initialize by setting up the sysctl and proc/perfcnt entries, allocating
*           default storage if any, and setting variables to defaults.
*
*  @return
*/
/****************************************************************************/
static int __init perftest_init(void)
{
   /* register sysctl table */
   printk("%s\n", __FUNCTION__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif
   if ( gSysCtlHeader != NULL )
   {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
#endif
   }
   else
   {
      printk("%s: could not register sysctl table\n", __FUNCTION__);
   }

   memset(&perftest, 0, sizeof(PERFTEST_OBJ));   // zero entries, idx, wrap, and enable
   perftest.memcpysize = 0x1000;

   arm_perf_init_start(ARM_PERF_I_EXEC, ARM_PERF_I_STALL);
   return 0;
}
subsys_initcall(perftest_init);
/****************************************************************************/
/**
*  @brief      Exit and cleanup (probably not done)
*
*  @return
*/
/****************************************************************************/
void perftest_exit(void)
{
   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
}





