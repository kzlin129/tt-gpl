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



/**
*
*  @file    iodump.c
*
*  @brief   iodump implementation.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/in.h>
#include <linux/ctype.h>
#include <linux/net.h>
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/inet.h>
#include <linux/jiffies.h>
#include <linux/broadcom/knllog.h>
#include <linux/broadcom/iodump.h>
#include <linux/broadcom/timer.h>
#include <asm/uaccess.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
static char gBanner[] __initdata = KERN_INFO "BROADCOM IoDump Driver: 0.01\n";

/* ---- Private Variables ------------------------------------------------ */
static IODUMP_OBJ IoDump;
static long IoDumpThreadPid = 0;
static struct completion IoDumpExited;
static spinlock_t IoDumpLogBufLock = SPIN_LOCK_UNLOCKED;
static struct timer_list IoDumpTimer;
DECLARE_WAIT_QUEUE_HEAD(IoDumpWakeQ);
static int IoDumpWake;

// private functions referenced below in tables
static int proc_do_iodump_intvec_logfile(ctl_table * table, int write, struct file *filp,
                                         void __user * buffer, size_t * lenp, loff_t * ppos);
static int proc_do_iodump_intvec_bufsize(ctl_table * table, int write, struct file *filp,
                                         void __user * buffer, size_t * lenp, loff_t * ppos);
static int proc_do_iodump_intvec_dumpstate(ctl_table * table, int write, struct file *filp,
                                           void __user * buffer, size_t * lenp, loff_t * ppos);

static int iodump_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static struct ctl_table_header *gSysCtlHeader;

static int procIoDumpOn(ctl_table * table, int write, struct file *filp, void *buffer, size_t * lenp, loff_t * ppos);

#define BCM_SYSCTL_IODUMP_STREAM_STRUCT(ID) {                        \
   {                                                                 \
      .ctl_name         = 1,                                         \
      .procname         = "ingress",                                 \
      .data             = &IoDump.stream[ID].ingress,                \
      .maxlen           = sizeof( int ),                             \
      .mode             = 0644,                                      \
      .proc_handler     = &proc_dointvec                             \
   },                                                                \
   {                                                                 \
      .ctl_name         = 2,                                         \
      .procname         = "egress",                                  \
      .data             = &IoDump.stream[ID].egress,                 \
      .maxlen           = sizeof( int ),                             \
      .mode             = 0644,                                      \
      .proc_handler     = &proc_dointvec                             \
   },                                                                \
   {}                                                                \
}

#define BCM_SYSCTL_IODUMP_STREAM(NUM) {                              \
   .ctl_name            = NUM + 1,                                   \
   .procname            = #NUM,                                      \
   .child               = gSysCtlIoDumpStream##NUM,                  \
   .mode                = 0555                                       \
}

static struct ctl_table gSysCtlIoDumpStream0[] = BCM_SYSCTL_IODUMP_STREAM_STRUCT(0);
static struct ctl_table gSysCtlIoDumpStream1[] = BCM_SYSCTL_IODUMP_STREAM_STRUCT(1);
static struct ctl_table gSysCtlIoDumpStream2[] = BCM_SYSCTL_IODUMP_STREAM_STRUCT(2);
static struct ctl_table gSysCtlIoDumpStream3[] = BCM_SYSCTL_IODUMP_STREAM_STRUCT(3);

static struct ctl_table gSysCtlIoDumpStreams[] = {
   BCM_SYSCTL_IODUMP_STREAM(0),
   BCM_SYSCTL_IODUMP_STREAM(1),
   BCM_SYSCTL_IODUMP_STREAM(2),
   BCM_SYSCTL_IODUMP_STREAM(3),
   {}
};

static struct ctl_table gSysCtlIoDumpTdm[] = {
   {
      .ctl_name      = 1,
      .procname      = "ingress",
      .data          = &IoDump.tdm.ingress,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 2,
      .procname      = "egress",
      .data          = &IoDump.tdm.egress,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};

static struct ctl_table gSysCtlIoDump[] = {
   {
      .ctl_name      = 1,
      .procname      = "ipaddr",
      .data          = &IoDump.ipaddr,
      .maxlen        = sizeof( IoDump.ipaddr ),
      .mode          = 0644,
      .proc_handler  = &proc_dostring
   },
   {
      .ctl_name      = 2,
      .procname      = "ipport",
      .data          = &IoDump.port,
      .maxlen        = sizeof( IoDump.port ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 3,
      .procname      = "enable",
      .data          = &IoDump.enable,
      .maxlen        = sizeof( IoDump.enable ),
      .mode          = 0644,
      .proc_handler  = &procIoDumpOn
   },
   {
      .ctl_name      = 4,
      .procname      = "net",
      .child         = gSysCtlIoDumpStreams,
      .mode          = 0555
   },
   {
      .ctl_name      = 5,
      .procname      = "tdm",
      .child         = gSysCtlIoDumpTdm,
      .mode          = 0555
   },
   {
      .ctl_name      = 6,
      .procname      = "filename",
      .data          = &IoDump.filename,
      .maxlen        = sizeof( IoDump.filename ),
      .mode          = 0644,
      .proc_handler  = &proc_dostring
   },
   {
      .ctl_name      = 7,
      .procname      = "logfile",
      .data          = &IoDump.logfile,
      .maxlen        = sizeof( IoDump.logfile ),
      .mode          = 0644,
      .proc_handler  = &proc_do_iodump_intvec_logfile
   },
   {
      .ctl_name      = 8,
      .procname      = "bufsize",
      .data          = &IoDump.bufsize,
      .maxlen        = sizeof( IoDump.bufsize ),
      .mode          = 0644,
      .proc_handler  = &proc_do_iodump_intvec_bufsize
   },
   {
      .ctl_name      = 9,
      .procname      = "dump-state",
      .data          = &IoDump.dumpstate,
      .maxlen        = sizeof( IoDump.dumpstate ),
      .mode          = 0644,
      .proc_handler  = &proc_do_iodump_intvec_dumpstate
   },
   {
      .ctl_name      = 10,
      .procname      = "read_timeout",
      .data          = &IoDump.read_timeout,
      .maxlen        = sizeof( IoDump.read_timeout ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};

static struct ctl_table gSysCtl[] = {
   {
      .ctl_name   = CTL_BCM_IODUMP,
      .procname   = "iodump",
      .mode       = 0555,
      .child      = gSysCtlIoDump
   },
   {}
};

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations iodump_fops = {
 owner:THIS_MODULE,
 ioctl:iodump_ioctl,
};

/* ---- Private Function Prototypes -------------------------------------- */
static int iodump_thread(void *data);
static int iodump_gettime(void);

/* ---- Functions -------------------------------------------------------- */
/****************************************************************************
*
*  iodump_circbuf
*
* Write iodump data to a circular capture buffer in ram.
*
***************************************************************************/
static void iodump_circbuf(void *bufp, int size)
{
   char *data = (char *) bufp;

   if ((IoDump.bufwritep + size) <= IoDump.bufendp)
   {
      memcpy(IoDump.bufwritep, data, size);
      IoDump.bufwritep += size;
      if (IoDump.bufwritep >= IoDump.bufendp)
      {
         IoDump.bufwritep = IoDump.bufp;
      }
   }
   else                         // data wraps end of buffer
   {
      size_t lenRemaining = IoDump.bufendp - IoDump.bufwritep;
      memcpy(IoDump.bufwritep, data, lenRemaining);
      memcpy(IoDump.bufp, &data[lenRemaining], size - lenRemaining);
      IoDump.bufwritep = IoDump.bufp + (size - lenRemaining);
   }
}

/****************************************************************************
*
*  iodump_write
*
* Write iodump entry (hdr + data).
*
***************************************************************************/
void iodump_write(int streamId, IODUMP_TYPE type, short length, void *bufp)
{
   unsigned long flags;
   IODUMP_HDR hdr;


   if ( !IoDump.enable )
   {
      return;
   }

   switch (type)
   {
      case IODUMP_TDM_EGRESS:    // fall through
      case IODUMP_TDM_EGRESS_WB:
         if (!IoDump.tdm.egress)
            return;
         break;

      case IODUMP_TDM_INGRESS:   // fall through
      case IODUMP_TDM_INGRESS_WB:
         if (!IoDump.tdm.ingress)
            return;
         break;

      case IODUMP_NET_INGRESS:
         if (!IoDump.stream[streamId].ingress)
            return;
         break;

      case IODUMP_NET_EGRESS:
         if (!IoDump.stream[streamId].egress)
            return;
         break;

      default:
         break;
   }

   memset(&hdr, 0, sizeof(hdr));
   hdr.magic = htonl(IODUMP_MAGIC);
   hdr.timestamp = htonl(iodump_gettime());
   hdr.length = htons(length);
   hdr.stream = streamId;

   // write egress data
   hdr.type = type;
   spin_lock_irqsave(&IoDumpLogBufLock, flags);
   iodump_circbuf(&hdr, sizeof(hdr));
   iodump_circbuf(bufp, length);
   spin_unlock_irqrestore(&IoDumpLogBufLock, flags);
}

EXPORT_SYMBOL(iodump_write);

/****************************************************************************
*
*  iodump_write2
*
* Write iodump entry (hdr + data). data is volatile short * format
* This function exists because the ept is compiled to treat warnings
* as errors, and the cast of the volatile short * becomes an error there.
* The cast here in kernel mode isn't a problem.
*
***************************************************************************/
void iodump_write2(int streamId, IODUMP_TYPE type, short length, volatile short *bufp)
{
   iodump_write(streamId, type, length, (void *)bufp);
}

EXPORT_SYMBOL(iodump_write2);

/****************************************************************************
*
*  iodump_inuse
*
* Get amount of circular capture buffer currently in use.
*
***************************************************************************/
unsigned int iodump_inuse(void)
{
   if (IoDump.bufwritep >= IoDump.bufreadp)
   {
      return (IoDump.bufwritep - IoDump.bufreadp);
   }
   return (IoDump.bufsize - (IoDump.bufreadp - IoDump.bufwritep));
}

/****************************************************************************
*
*  iodump_inuse
*
* Get amount of circular capture buffer currently free.
*
***************************************************************************/
unsigned int iodump_room(void)
{
   return IoDump.bufsize - iodump_inuse();
}

/****************************************************************************
*
*  iodump_ioctl
*
* Allow user mode to use ioctl to write log messages to the kernel buffer.
* This avoids mmap'ing shared memory and devising a strategy to handle
* write pointer contention since everything is now done from kernel mode.
*
***************************************************************************/
int iodump_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
   int err = 0;

   if (!IoDump.enable)
   {
      return err;               // No logging done until iodump is enabled.
   }
   switch (cmd)
   {
   case IODUMP_IOCTL_LOGMSG:
   {
      char msg[IODUMP_MAXMSGLEN];

      if (iodump_room() > (IODUMP_MAXMSGLEN + sizeof(IODUMP_HDR)))
      {
         if (copy_from_user(&msg, (char *) arg, IODUMP_MAXMSGLEN) != 0)
         {
            return -EFAULT;
         }
         msg[IODUMP_MAXMSGLEN - 1] = '\0';
         iodump_write(0, IODUMP_LOG, strlen(msg), msg);
      }
      else
      {
         IoDump.bufwritep = IoDump.bufp;
         IoDump.bufreadp = IoDump.bufp;
         strcpy(msg, "IoDump Buffer Overrun, data lost!");
         iodump_write(0, IODUMP_LOG, strlen(msg), msg);
      }
      break;
   }

   default:
   {
      printk(KERN_WARNING "Unrecognized ioctl: '0x%x'\n", cmd);
      return -ENOTTY;
   }
   }
   return (err);
}

/****************************************************************************
*
*  iodump_timer
*
* timer callback to keep waking reader thread.
*
***************************************************************************/
static void iodump_timer(unsigned long dummy)
{
   (void) dummy;

   if (IoDump.enable)
   {
      // wakeup kernel thread and rearm timer as long as iodump is enabled.
      IoDumpWake = 1;
      wake_up_interruptible(&IoDumpWakeQ);

      IoDumpTimer.expires = jiffies + IoDump.read_timeout;
      add_timer(&IoDumpTimer);
   }
}

/****************************************************************************
*
*  iodump_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int __init iodump_init(void)
{
   int rc;

   printk(gBanner);

   // Register our device with Linux

   if ((rc = register_chrdev(BCM_IODUMP_MAJOR, "iodump", &iodump_fops)) < 0)
   {
      printk(KERN_WARNING "iodump: register_chrdev failed for major %d\n", BCM_IODUMP_MAJOR);
      return rc;
   }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table(gSysCtl, 0);
   if (gSysCtlHeader != NULL)
   {
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gSysCtlHeader = register_sysctl_table(gSysCtl);
#endif

   memset(&IoDump, 0, sizeof(IoDump));
   IoDump.bufsize = IODUMP_DEFAULT_BUFSIZE;
   IoDump.bufp = vmalloc(IoDump.bufsize);
   if (!IoDump.bufp)
   {
      return -ENOMEM;
   }
   memset(IoDump.bufp, 0, IoDump.bufsize);
   IoDump.bufwritep = IoDump.bufp;
   IoDump.bufreadp = IoDump.bufp;
   IoDump.bufendp = IoDump.bufp + IoDump.bufsize;

   strcpy(IoDump.filename, "/tmp/iodump.bin"); // default local file name
   IoDump.logfile = 0;         // log to socket by default
   strcpy(IoDump.ipaddr, "192.168.1.99");      // currently USB over ethernet host IP
   IoDump.port = 4000;         // default logging ip:port
   IoDump.enable = 0;          // dumping enabled flag
   IoDump.stream[0].ingress = 1;       // Main audio stream
   IoDump.stream[0].egress = 1;
   IoDump.stream[1].ingress = 0;
   IoDump.stream[1].egress = 0;
   IoDump.stream[2].ingress = 0;
   IoDump.stream[2].egress = 0;
   IoDump.stream[3].ingress = 0;       // Video stream (doesn't hurt to enable on audio device)
   IoDump.stream[3].egress = 0;
   IoDump.tdm.ingress = 1;     // TDM directions to log
   IoDump.tdm.egress = 1;
   IoDump.read_timeout = msecs_to_jiffies(50);
   IoDump.netDumpLock = SPIN_LOCK_UNLOCKED;

   // Create dumping to file or socket thread
   init_completion(&IoDumpExited);
   IoDumpThreadPid = kernel_thread(iodump_thread, 0, 0);

   // Create timer to tickle thread at regular intervals
   init_timer(&IoDumpTimer);
   IoDumpTimer.function = iodump_timer;

   return 0;
}

/****************************************************************************
*
*  iodump_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

void __exit iodump_exit(void)
{
   del_timer(&IoDumpTimer);

   if (gSysCtlHeader != NULL)
   {
      unregister_sysctl_table(gSysCtlHeader);
   }
   if (IoDump.bufp != NULL)
   {
      vfree(IoDump.bufp);
   }
   if (gSysCtlHeader != NULL)
   {
      unregister_sysctl_table(gSysCtlHeader);
   }
   if (IoDumpThreadPid >= 0)
   {
      kill_proc_info(SIGTERM, SEND_SIG_PRIV, IoDumpThreadPid);
      wait_for_completion(&IoDumpExited);
   }
}

/****************************************************************************
*
*  proc_do_iodump_intvec_dumpstate
*
* Dump iodump state information.
*
***************************************************************************/
static int proc_do_iodump_intvec_dumpstate(ctl_table * table, int write, struct file *filp,
                                           void __user * buffer, size_t * lenp, loff_t * ppos)
{
   int rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);

   if (IoDump.dumpstate)
   {
      printk(KERN_NOTICE "\n");
      printk(KERN_NOTICE "bufp        = 0x%08x\n", (int) IoDump.bufp);
      printk(KERN_NOTICE "bufendp     = 0x%08x\n", (int) IoDump.bufendp);
      printk(KERN_NOTICE "bufreadp    = 0x%08x\n", (int) IoDump.bufreadp);
      printk(KERN_NOTICE "bufwritep   = 0x%08x\n", (int) IoDump.bufwritep);
      printk(KERN_NOTICE "bufsize     = 0x%08x\n", (int) IoDump.bufsize);
      printk(KERN_NOTICE "socket      = 0x%08x\n", (int) IoDump.sock);
      printk(KERN_NOTICE "file        = 0x%08x\n", (int) IoDump.file);

      IoDump.dumpstate = 0;
   }
   return rc;
}

/****************************************************************************
*
*  proc_do_iodump_intvec_logfile
*
* Change the logging mode - must not be running iodump when this occurs
* or sockets will not be cleaned up for example.
*
***************************************************************************/
static int proc_do_iodump_intvec_logfile(ctl_table * table, int write, struct file *filp,
                                         void __user * buffer, size_t * lenp, loff_t * ppos)
{
   int rc = 0;

   if (!table || !table->data)
      return -EINVAL;

   if (write)
   {
      /* get value from buffer into knllog.entries */
      int prevLogfile = IoDump.logfile;
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
      if (rc < 0)
      {
         return rc;
      }
      if ((IoDump.logfile != prevLogfile) && IoDump.enable)
      {
         printk(KERN_WARNING "Cannot change logging mode while iodump enabled.\n");
         IoDump.logfile = prevLogfile;
         return -EINVAL;
      }
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);       // No special processing for read.
   }
   return rc;
}
/****************************************************************************
*
*  proc_do_iodump_intvec_bufsize
*
* Change the circular capture buffer size.
*
***************************************************************************/
static int proc_do_iodump_intvec_bufsize(ctl_table * table, int write, struct file *filp,
                                         void __user * buffer, size_t * lenp, loff_t * ppos)
{
   unsigned long flags;
   int rc;

   if (!table || !table->data)
      return -EINVAL;

   if (write)
   {
      if (IoDump.enable)
      {
         printk(KERN_WARNING "Cannot resize log buffer while iodump enabled\n");
         return -EINVAL;
      }

      spin_lock_irqsave(&IoDumpLogBufLock, flags);

      if (IoDump.bufp != NULL)
      {
         vfree(IoDump.bufp);
         IoDump.bufp = NULL;
      }

      /* get value from buffer into knllog.entries */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
      if (rc < 0)
      {
         spin_unlock_irqrestore(&IoDumpLogBufLock, flags);
         return rc;
      }

      if (IoDump.bufsize == 0)
      {
         printk(KERN_WARNING "Cannot set 0 size, restoring size to default\n");
         IoDump.bufsize = IODUMP_DEFAULT_BUFSIZE;
      }

      IoDump.bufp = vmalloc(IoDump.bufsize);
      if (IoDump.bufp == NULL)
      {
         printk(KERN_WARNING "%s: Cannot allocate memory for iodump buffer size=%u\n", __FUNCTION__, IoDump.bufsize);
         spin_unlock_irqrestore(&IoDumpLogBufLock, flags);
         return -ENOMEM;;
      }
      IoDump.bufwritep = IoDump.bufp;
      IoDump.bufreadp = IoDump.bufp;
      IoDump.bufendp = IoDump.bufp + IoDump.bufsize;

      spin_unlock_irqrestore(&IoDumpLogBufLock, flags);
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);       // No special processing for read.
   }
   return rc;
}

/****************************************************************************
*
*  incrReadPtr
*
*   increment read pointer.
*
***************************************************************************/
static void incrReadPtr(int size)
{
   IoDump.bufreadp += size;
   if (IoDump.bufreadp >= IoDump.bufendp)
   {
      IoDump.bufreadp = IoDump.bufp + (IoDump.bufreadp - IoDump.bufendp);
   }
}

#if CONFIG_NET
/****************************************************************************
*
*  socksend
*
*   send on a socket.
*
***************************************************************************/
int socksend(char *bufp, int size)
{
   struct msghdr msg;
   struct iovec iov;
   int rc = -1;
   iov.iov_base = bufp;
   iov.iov_len = size;
   rc = kernel_sendmsg(IoDump.sock, &msg, (struct kvec *) &iov, 1, size);
   //KNLLOG("size=%d, rc=%d", size, rc);
   if (rc > 0)
   {
      incrReadPtr(rc);      // This much data was successfully sent
   }
   else if (rc < 0 && rc != -EAGAIN)
   {
      printk(KERN_WARNING "sock send, rc=%d\n", rc);
   }
   return rc;
}
#endif

/****************************************************************************
*
*  iodump_thread
*
*   Worker thread to transfer data to the file or socket.
*
***************************************************************************/
static int iodump_thread(void *data)
{
   daemonize("iodump");

   while (1)
   {
      // wait for a batch of tdm, network ingress/egress data before writing */
      if (0 == wait_event_interruptible(IoDumpWakeQ, IoDumpWake))
      {
         int size;
         char *writeSnapshot;   // We don't want to worry about the write pointer changing under us

         IoDumpWake = 0;

         writeSnapshot = IoDump.bufwritep;
         if (writeSnapshot >= IoDump.bufreadp)
         {
            size = writeSnapshot - IoDump.bufreadp;
         }
         else
         {
            size = IoDump.bufsize - (IoDump.bufreadp - writeSnapshot);
         }
         //KNLLOG("enable = %d, size = %d\n", IoDump.enable, size);
         if (IoDump.enable && size)
         {
            if (writeSnapshot >= IoDump.bufreadp)
            {
#if CONFIG_NET
               if (IoDump.sock)
               {
                  int rc = socksend(IoDump.bufreadp, size);
                  if ((rc == -EAGAIN) || (rc == 0))
                  {
                     continue;  // Nothing sent, retry next timer tick
                  }
               }
               else
#endif
               if (IoDump.file)
               {
                  mm_segment_t old_fs = get_fs();
                  set_fs(KERNEL_DS);
                  IoDump.file->f_op->write(IoDump.file, IoDump.bufreadp, size, &IoDump.file->f_pos);
                  set_fs(old_fs);
                  incrReadPtr(size);
               }
            }
            else
            {
               int size1 = IoDump.bufendp - IoDump.bufreadp;  // size to end of buffer
               int size2 = writeSnapshot - IoDump.bufp;        // size from start to write
#if CONFIG_NET
               if (IoDump.sock)
               {
                  int rc = socksend(IoDump.bufreadp, size1);
                  if ((rc == -EAGAIN) || (rc == 0))
                  {
                     continue;  // Nothing sent, retry next timer tick
                  }

                  if (size2)
                  {
                     int rc = socksend(IoDump.bufreadp, size2);
                     if ((rc == -EAGAIN) || (rc == 0))
                     {
                        continue;       // Nothing sent, retry next timer tick
                     }
                  }
               }
               else
#endif
               if (IoDump.file)
               {
                  mm_segment_t old_fs = get_fs();
                  set_fs(KERNEL_DS);
                  IoDump.file->f_op->write(IoDump.file, IoDump.bufreadp, size1, &IoDump.file->f_pos);
                  if (size2)
                  {
                     IoDump.file->f_op->write(IoDump.file, IoDump.bufp, size2, &IoDump.file->f_pos);
                  }
                  set_fs(old_fs);
                  incrReadPtr(size);
               }
            }
         }
      }
   }
   complete_and_exit(&IoDumpExited, 0);
}

/****************************************************************************
*
*  procIoDumpOn
*
*  IoDump on/off handling.
*
***************************************************************************/
static int procIoDumpOn(ctl_table * table,   /**< Control Table */
                        int write,           /**< write flag */
                        struct file *filp,   /**< file pointer */
                        void *buffer,        /**< buffer start */
                        size_t * lenp,       /**< length */
                        loff_t * ppos        /**< offset */
   )
{
   int rc;

   if (!table || !table->data)
      return -EINVAL;

   if (write)
   {
      int prev_enable = IoDump.enable;
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
      if (rc < 0)
      {
         return rc;
      }
      if (IoDump.enable && !prev_enable)
      {
         // 0 to 1 transition - enable iodump by opening socket

         printk(KERN_NOTICE "Enabling IoDump\n");

         // Start with empty buffer.
         IoDump.bufwritep = IoDump.bufp;
         IoDump.bufreadp = IoDump.bufp;

#if CONFIG_NET
         if (!IoDump.logfile)
         {
            struct sockaddr_in serverAddr;
#if 0
            struct linger LingerVal;
            mm_segment_t oldfs;
#endif

            rc = sock_create_kern(AF_INET, SOCK_STREAM, IPPROTO_TCP, &IoDump.sock);
            if (rc < 0)
            {
               printk(KERN_ERR "iodump: socket create failed rc=%d\n", rc);
               goto cleanup;
            }
#if 0
            /*
             * turn on the LINGER option so that on close, we'll send out
             * any pending messages
             */
            LingerVal.l_onoff = 1;
            LingerVal.l_linger = 120;
            oldfs = get_fs();
            set_fs(KERNEL_DS);
            IoDump.sock->ops->setsockopt(IoDump.sock, SOL_SOCKET, SO_LINGER, (char *) &LingerVal, sizeof(LingerVal));
            set_fs(oldfs);

            /* set the session socket to deliver message immediately */
            {
               int sockOptVal = 1;
               oldfs = get_fs();
               set_fs(KERNEL_DS);
               // Including <linux/tcp.h> causes many many warnings, which now cause errors with -Werr
#define TCP_NODELAY     1       /* Turn off Nagle's algorithm. */
               IoDump.sock->ops->setsockopt(IoDump.sock, IPPROTO_TCP, TCP_NODELAY, (char *) &sockOptVal, sizeof(sockOptVal));
               set_fs(oldfs);
            }
#endif

            // connect to the server
            memset(&serverAddr, 0, sizeof(serverAddr));
            serverAddr.sin_family = AF_INET;
            serverAddr.sin_port = htons((short) IoDump.port);
            if ((u32) - 1 == (serverAddr.sin_addr.s_addr = in_aton(IoDump.ipaddr)))
            {
               printk(KERN_WARNING "iodump:  Unknown server name '%s'.\n", IoDump.ipaddr);
               sock_release(IoDump.sock);
               IoDump.sock = NULL;
               return (-1);
            }

            rc = IoDump.sock->ops->connect(IoDump.sock, (struct sockaddr *) &serverAddr, sizeof(serverAddr), 0);
            if (rc < 0)
            {
               printk(KERN_WARNING "iodump: Can't connect to server. rc = %d\n", rc);
               sock_release(IoDump.sock);
               IoDump.sock = NULL;
               return rc;
            }
            printk(KERN_NOTICE "iodump: connected to server. rc = %d\n", rc);
         }
         else                   // log to file
#endif
         {
            mm_segment_t old_fs = get_fs();
            set_fs(KERNEL_DS);
            printk(KERN_NOTICE "Calling open on %s\n", IoDump.filename);
            IoDump.file = filp_open(IoDump.filename, O_TRUNC | O_WRONLY | O_CREAT, 0644);
            set_fs(old_fs);
            if (!IoDump.file || !IoDump.file->f_op || !IoDump.file->f_op->write)
            {
               printk(KERN_WARNING "Could not open %s\n", IoDump.filename);
               return (-1);
            }
         }
         IoDumpTimer.expires = jiffies + IoDump.read_timeout; // about 40 msec between file or socket operations
         add_timer(&IoDumpTimer);
      }
      else if (!IoDump.enable && prev_enable)
      {
         // 1 to 0 transition - disable iodump by closing socket
         printk(KERN_NOTICE "Disabling IoDump\n");
         goto cleanup;
      }
      else
      {
         printk(KERN_WARNING "On->On or Off->Off transition ignored\n");
      }
      return rc;
   }
   else
   {
      /* nothing special for read, just use generic int handler */
      return proc_dointvec(table, write, filp, buffer, lenp, ppos);
   }
 cleanup:
#if CONFIG_NET
   if (IoDump.sock)
   {
      printk(KERN_NOTICE "Calling sock_release\n");
      sock_release(IoDump.sock);
      IoDump.sock = NULL;
   }
#endif
   if (IoDump.file)
   {
      mm_segment_t old_fs = get_fs();
      set_fs(KERNEL_DS);
      printk(KERN_NOTICE "Calling close on %s\n", IoDump.filename);
      filp_close(IoDump.file, current->files);
      set_fs(old_fs);
      IoDump.file = NULL;
   }
   return rc;
}

/****************************************************************************
*
*  iodump_gettime
*
*  Get timestamp for info being logged.
*
***************************************************************************/
static int iodump_gettime(void)
{
   return ( timer_get_tick_count() );
}

/****************************************************************************/

module_init(iodump_init);
module_exit(iodump_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM116x IoDump Driver");
