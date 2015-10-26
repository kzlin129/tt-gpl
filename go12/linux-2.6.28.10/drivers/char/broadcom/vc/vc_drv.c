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




/*
*
*****************************************************************************
*
*  vco2_drv.c
*
*  PURPOSE:
*
*     This implements the driver for the VCO2.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>

#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc_frmfwd_defs.h>
#include <linux/broadcom/vc_gpio.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/debug_pause.h>
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/bcm_major.h>

#include <linux/broadcom/gpio.h>

#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "vchost.h"
#include "vchost_int.h"
#include "vchostreq_int.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcgencmd.h"
#include "vcfrmfwd.h"
#include "vcfrmfwd_int.h"
#include "vcmsgfifo.h"
#include "vc_dispman2.h"
#include "vchostreq.h"
#include "vclogging.h"
#include "vchost_audio.h"
#include "vcstate_intern.h"

#include <cfg_global.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
// Setting VC_UNLOAD_VC_IMAGE to 1 will cause the "vmcs" data structure, and
// some of the functions that use it to be declared as __initdata (and __init
// for the functions).

#define  VC_UNLOAD_VC_IMAGE   1

#if VC_UNLOAD_VC_IMAGE
#  define   VC_INIT     __init
#  define   VC_INITDATA __initdata
#else
#  define   VC_INIT
#  define   VC_INITDATA
#endif

#define USE_IRQ 1
#define USE_VC02_STATIC 0

#if CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL
#if USE_VC02_STATIC
#define VC02_BIN_FNAME     vmcs_vengine
#include "vmcs_vengine.h"
#else
#define VC02_BIN_FNAME     vmcs
#include "vmcs.h"
#endif
#endif  // CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL

#define NUM_FRAME_BUFFERS  4
#define NUM_BUFFERS_ON_VC02  4
#define NUM_MAX_FRAMES  8
#define NUM_MAX_MSEC  250
#define VC_EGRESS_PKT_BASED_FLOWCTRL 0

typedef struct
{
    int                 bLoopbackEnabled;
    int                 decodeStream;
    VC_HOSTCALLBACK_T  *prevCallbacks;

} LoopbackArgs_t;



#define VC_FILE_IO_SIZE 1024

typedef struct
{
    struct list_head    eventList;
    struct list_head    lockList;

    struct semaphore    lock;
    void               *ioBuf;

} VC_FileData_t;

/* ---- Private Variables ------------------------------------------------ */

static  char banner[] __initdata = KERN_INFO "VC02 Driver: 0.00\n";

static  int                     gVC_InterruptsEnabled = 0;
static  void                   *gVC_IrqEvent = NULL;

static  VC_FrameBufferList_t   gVC_FramesToVCList;
static unsigned int gMaxFramesInEgressQ=0;
static unsigned int gAveFramesInEgressQ=1;
static unsigned int gCurrFramesInEgressQ=0;
int gVcFramesLooped = 0;
int gVcFramesLost   = 0;
int gVcFramesRcvd   = 0;
int gVcFramesSent   = 0;
int gVcFramesSendErr= 0;
int gVcFramesAcked  = 0;
int gVcFramesNaked  = 0;

// Higher number = Higher priority
// min is 1, max is MAX_USER_RT_PRIO-1 (99)

int gVcPriorityIrq      = 95;
int gVcPriorityReadRegs = 93;
int gVcPriorityRecv     = 91;
int gVcPrioritySend     = 91;
int gVcHostReqPriority  = 60;

int gVcLoopbackDelay = 50;

int gVcUseIrq = USE_IRQ;

#if VC_DEBUG_ENABLED
int gVcDebugInfo           = 0;
int gVcDebugFrameLoopback  = 1;
int gVcDebugHostIrq        = 0;
int gVcDebugVerbose        = 1;
int gVcDebugDisplayRegion  = 0;
#endif

int gErrorPerPeriod = 2; /* error print out period is set to 2 second by default */
int gErrorPeriodLenSec = 2; /* 2 errors will be printed out every period (2sec) */
int gErrorPrtEnable = 1;   /* error print out is enabled by default */

int gVcGmtOffset  = 0;  // Offset from GMT for the vc-fuse program. This gets
                        // added to the timeofday to come up with local time
                        // for files being stored on the SD card.

static struct task_struct *gIrqTask       = NULL;
static struct task_struct *gFrameRecvTask = NULL;
static struct task_struct *gReadRegsTask  = NULL;
static struct task_struct *gFrameSendTask = NULL;
static struct task_struct *gHostReqTask   = NULL;

static  int                 gInitialized = 0;
#if CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL
static  int                 gBootloaded = 0;
#endif
static  int                 gVcCodeCheckCrcs;
static  int                 gVcUse32bitIOTemp;

//spinlock_t gVC02lock;

static VC_FrameBufferList_t   gLoopbackFreeFramesList;
static VC_FrameBufferList_t   gLoopbackFramesFromVCList;

static LoopbackArgs_t         gLoopbackArgs;

static VC_ERRORlog_t          gInAllocBuffError;
static VC_ERRORlog_t          gEgOutOfBuffError;
static VC_ERRORlog_t          gDiscardEgressError;

static int vc_code_check_crcs( ctl_table *table, int write, struct file *filp,
                               void __user *buffer, size_t *lenp, loff_t *ppos );

static int vc_use_32_bit_io( ctl_table *table, int write, struct file *filp,
                             void __user *buffer, size_t *lenp, loff_t *ppos );

#define BCM_SYSCTL_VC_NODE(NUM, NAME, DATA) {            \
   .ctl_name      = NUM,                                 \
   .procname      = #NAME,                               \
   .data          = &DATA,                               \
   .maxlen        = sizeof( int ),                       \
   .mode          = 0644,                                \
   .proc_handler  = &proc_dointvec                       \
}

static struct ctl_table gSysCtlVc[] = {
   BCM_SYSCTL_VC_NODE(1, frames-looped, gVcFramesLooped),
   BCM_SYSCTL_VC_NODE(2, frames-lost, gVcFramesLost),
   BCM_SYSCTL_VC_NODE(3, frames-rcvd, gVcFramesRcvd),
   BCM_SYSCTL_VC_NODE(4, frames-sent, gVcFramesSent),
   BCM_SYSCTL_VC_NODE(5, frames-send-err, gVcFramesSendErr),
   BCM_SYSCTL_VC_NODE(6, frames-acked, gVcFramesAcked),
   BCM_SYSCTL_VC_NODE(7, frames-naked, gVcFramesNaked),

   BCM_SYSCTL_VC_NODE(8, loopback-delay, gVcLoopbackDelay),
   BCM_SYSCTL_VC_NODE(9, use-irq, gVcUseIrq),

   BCM_SYSCTL_VC_NODE(10, priority-send, gVcPrioritySend),
   BCM_SYSCTL_VC_NODE(11, priority-recv, gVcPriorityRecv),
   BCM_SYSCTL_VC_NODE(12, priority-irq, gVcPriorityIrq),
   BCM_SYSCTL_VC_NODE(13, priority-hostreq, gVcHostReqPriority),
   BCM_SYSCTL_VC_NODE(14, priority-readVCregs, gVcPriorityReadRegs),
   BCM_SYSCTL_VC_NODE(15, egress-queue-max-frames, gMaxFramesInEgressQ),
   BCM_SYSCTL_VC_NODE(16, egress-queue-ave-frames, gAveFramesInEgressQ),
   BCM_SYSCTL_VC_NODE(17, egress-queue-curr-frames, gCurrFramesInEgressQ),

   BCM_SYSCTL_VC_NODE(20, hostport-rled, gVcHostPortRLED),
   BCM_SYSCTL_VC_NODE(21, hostport-wled, gVcHostPortWLED),

   BCM_SYSCTL_VC_NODE(30, frmfwd-checksum-enabled, g_vc_frmfwd_stats.checksumEnabled),
   BCM_SYSCTL_VC_NODE(31, frmfwd-sent-checksum, g_vc_frmfwd_stats.framesSentWithChecksum),
   BCM_SYSCTL_VC_NODE(32, frmfwd-sent-no-checksum, g_vc_frmfwd_stats.framesSentNoChecksum),
   BCM_SYSCTL_VC_NODE(33, frmfwd-rcvd-no-checksum, g_vc_frmfwd_stats.framesRcvdNoChecksum),
   BCM_SYSCTL_VC_NODE(34, frmfwd-rcvd-checksum-good, g_vc_frmfwd_stats.framesRcvdChecksumGood),
   BCM_SYSCTL_VC_NODE(35, frmfwd-rcvd-checksum-bad, g_vc_frmfwd_stats.framesRcvdChecksumBad),

   {
      .ctl_name         = 40,
      .procname         = "vc-code-check-crcs",
      .data             = &gVcCodeCheckCrcs,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &vc_code_check_crcs
   },
   {
      .ctl_name         = 50,
      .procname         = "use-32-bit-io",
      .data             = &gVcUse32bitIOTemp,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &vc_use_32_bit_io
   },
   {
      .ctl_name         = 51,
      .procname         = "addressing-mode",
      .data             = &gVcAddrModeStr,
      .maxlen           = sizeof( gVcAddrModeStr ),
      .mode             = 0644,
      .proc_handler     = &proc_dostring
   },

   BCM_SYSCTL_VC_NODE(52, display-stats, gLcdPrintStats),
   BCM_SYSCTL_VC_NODE(53, use-fast-xfer, gVcUseFastXfer),

#if (CFG_GLOBAL_CPU == MIPS32)
   BCM_SYSCTL_VC_NODE(54, use-dma, gVcUseDma),
#endif

   BCM_SYSCTL_VC_NODE(55, knllog-profile, gVcProfileTransfers),
   BCM_SYSCTL_VC_NODE(60, gmt-offset, gVcGmtOffset),

#if VC_DEBUG_ENABLED
   BCM_SYSCTL_VC_NODE(101, debug-read, gVcDebugRead),
   BCM_SYSCTL_VC_NODE(102, debug-write, gVcDebugWrite),
   BCM_SYSCTL_VC_NODE(103, debug-trace, gVcDebugTrace),
   BCM_SYSCTL_VC_NODE(104, debug-info, gVcDebugInfo),
   BCM_SYSCTL_VC_NODE(105, debug-msgfifo, gVcDebugMsgFifo),
   BCM_SYSCTL_VC_NODE(106, debug-frame-loopback, gVcDebugFrameLoopback),
   BCM_SYSCTL_VC_NODE(107, debug-host-irq, gVcDebugHostIrq),
   BCM_SYSCTL_VC_NODE(108, debug-verbose, gVcDebugVerbose),
   BCM_SYSCTL_VC_NODE(109, debug-display-region, gVcDebugDisplayRegion),
#endif

   BCM_SYSCTL_VC_NODE(120, errprt_perperiod, gErrorPerPeriod),
   BCM_SYSCTL_VC_NODE(121, error_periodlen, gErrorPeriodLenSec),
   BCM_SYSCTL_VC_NODE(122, error_prtenable, gErrorPrtEnable),

   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_VC,
      .procname      = "vc",
      .mode          = 0555,
      .child         = gSysCtlVc
   },
   {}
};

static  struct ctl_table_header    *gSysCtlHeader = NULL;

/* ---- Private Function Prototypes -------------------------------------- */

static  int     vc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static  int     vc_open( struct inode *inode, struct file *file );
static  ssize_t vc_read( struct file *file, char *buffer, size_t count, loff_t *ppos );
static  int     vc_release( struct inode *inode, struct file *file );
static  ssize_t vc_write( struct file *file, const char *buffer, size_t count, loff_t *ppos );

        void    vc_adjust_thread_priority( int *requestedPriority );

#if CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL
static  void    VC_INIT Bootload( void );
#endif

static  int     Init( void );
static  int     GencmdInit( void );
static  void    Dump( void );
static  int     FrameReceiveThread( void *data );
static  int     FrameSendThread( void *data );

static  FRMFWD_FRAME_T *CB_AllocFrameBuffer( int streamNum, int length_in_bytes );
static void CB_FrameReceived( FRMFWD_FRAME_T *fwdFrame, unsigned int lengthinbyte, int streamNum );
static void CB_FrameSent( FRMFWD_FRAME_INFO_T *frameInfo, int streamNum );
static void CB_FrameNotSentOutOfBuffers( FRMFWD_FRAME_INFO_T *frameInfo, int streamNum );


static VC_FrameBuffer_t *VC_DequeueFrameBuffer( VC_FrameBufferList_t *fList );
int VC_GetEgressQDepth( VC_FrameBufferList_t *fList );
static  VC_FrameBuffer_t   *InternalDequeueFrameBuffer( VC_FrameBufferList_t *fList );
static void VC_EnqueueFrameBuffer( VC_FrameBufferList_t *fList, VC_FrameBuffer_t *frameBuf );
static  void                InitFrameBufferList( VC_FrameBufferList_t *fList );


static  void    StartFrameLoopback( int decode_stream );
static  void    StopFrameLoopback( void );

static VC_FrameBuffer_t* loopbackGetFrameBuf
(
   unsigned int stream_num,
   int length_in_bytes
);

static void loopbackEnqueueFrameBuf
(
   VC_FrameBuffer_t *frame,
   unsigned int      lengthinbyte,
   int               stream_num
);

static VC_FrameBuffer_t* loopbackDequeueFrameBuf( int stream_num );
static void loopbackFreeFrameBuf( VC_FrameBuffer_t *frame );



VC_FFCALLBACK_T gFrameForwarderCallbacks =
{
    CB_AllocFrameBuffer,
    CB_FrameReceived,
    CB_FrameSent,
    CB_FrameNotSentOutOfBuffers,
#if defined( CONFIG_BCM_HALAUDIO_MIXER )
    CB_AllocAudioBuffer,
    CB_AudioFrameReceived,
#else
    NULL,
    NULL
#endif
};


static volatile VC_HOSTGLOBALS_T vc_hostg = {0};
extern VC_Clock_t gVCClock;

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  vc_code_check_crcs - called in response to:
*
*     echo 1 > /proc/sys/vc/vc-code-check-crcs
*
***************************************************************************/

static int vc_code_check_crcs( ctl_table *table, int write, struct file *filp,
                               void __user *buffer, size_t *lenp, loff_t *ppos )
{
   vc_host_check_crcs( 0 );

   gVcCodeCheckCrcs = 0;

   return proc_dointvec( table, write, filp, buffer, lenp, ppos );
}

/****************************************************************************
*
*  vc_use_32_bit_io - called in response to:
*
*     echo 1 > /proc/sys/vc/vuse-32-bit-io
*
***************************************************************************/

static int vc_use_32_bit_io( ctl_table *table, int write, struct file *filp,
                             void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int   rc;

   gVcUse32bitIOTemp = gVcUse32bitIO;

   rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

   if ( write && !gVc32BitIOAvail && gVcUse32bitIOTemp )
   {
      printk( KERN_ERR "32-bit I/O isn't available - ignoring\n" );
      rc = -EINVAL;
   }

   if ( rc == 0 )
   {
      gVcUse32bitIO = gVcUse32bitIOTemp;
   }

   return rc;
}

/****************************************************************************
*
*  vc_host_set_cmd_callback
*
*   Initializes callbacks to be used for dealing with video packets from 
*   the VC02
*
***************************************************************************/

VC_HOSTCALLBACK_T *vc_host_set_cmd_callback( VC_HOSTCALLBACK_T *callback )
{
   VC_HOSTCALLBACK_T *oldCallback = vc_hostg.callback;

   vc_hostg.callback = callback;

   return oldCallback;
}

/****************************************************************************
*
*  vc_ioctl
*
***************************************************************************/

// static
int vc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    int err;

    VC_DEBUG( Trace, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    if (( _IOC_TYPE( cmd ) != VC_MAGIC )
    ||  ( _IOC_NR( cmd ) < VC_CMD_FIRST )
    ||  ( _IOC_NR( cmd ) > VC_CMD_LAST ))
    {
        return -ENOTTY;
    }

    // Note that Read/Write is from the perspective of userland. access_ok
    // is from the perspective of kernelland.

    err = 0;
    if (( _IOC_DIR( cmd ) & _IOC_READ ) != 0 )
    {
        err |= !access_ok( VERIFY_WRITE, (void *)arg, _IOC_SIZE( cmd ));
    }
    if (( _IOC_DIR( cmd ) & _IOC_WRITE ) != 0 )
    {
        err |= !access_ok( VERIFY_READ, (void *)arg, _IOC_SIZE( cmd ));
    }
    if ( err )
    {
        printk( KERN_ERR "vc_ioctl: arg pointer is invalid\n" );
        return -EFAULT;
    }

    switch ( cmd )
    {
       
        case VC_IOCTL_INIT:
        {
#if VC_UNLOAD_VC_IMAGE
           printk( KERN_ERR "VC_IOCTL_INIT unsupported\n" );
           return -ENOTTY;
#else
           Init();
           break;
#endif
        }

        case VC_IOCTL_INIT_GENCMD:
        {
            GencmdInit();
            break;
        }

        case VC_IOCTL_DUMP:
        {
            Dump();
            break;
        }

        case VC_IOCTL_GET_STC:
        {
           vc_host_read_VC02_STC_reg();
           printk( KERN_INFO " STC=0x%x\n",gVCClock.dspclock);
           break;
        }


        case VC_IOCTL_GENCMD:
        {
            VC_GenCmd_t gencmd;

            if ( !gInitialized )
            {
                printk( KERN_ERR "vc: Not Initialized\n" );
                return -EPERM;
            }

            if ( copy_from_user( gencmd.cmd, ((VC_GenCmd_t *)arg)->cmd, sizeof( gencmd.cmd )) != 0 )
            {
                return -EFAULT;
            }

            VC_DEBUG( Trace, "Calling vc_gencmd with '%s', len = %d\n", &gencmd.cmd[0], strlen( gencmd.cmd ));

            gencmd.response.err = vc_gencmd( gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );

#if VC_DEBUG_ENABLED
            VC_DEBUG( Trace, "vc_gencmd returned err = %d\n", gencmd.response.err );
            if ( err == 0 )
            {
                VC_DEBUG( Trace, "vc_gencmd returned '%s'\n", gencmd.response.str );
            }
#endif

            if ( copy_to_user( &((VC_GenCmd_t *)arg)->response, &gencmd.response, sizeof( gencmd.response )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }

        case VC_IOCTL_START_FRAME_LOOPBACK:
        {
            int decode_stream = (int)arg;

            if ( !gInitialized )
            {
                printk( KERN_ERR "vc: Not Initialized\n" );
                return -EPERM;
            }

            StartFrameLoopback( decode_stream );
            break;
        }

        case VC_IOCTL_STOP_FRAME_LOOPBACK:
        {
            StopFrameLoopback();
            break;
        }

        case VC_IOCTL_BOOT:
        {
#if VC_UNLOAD_VC_IMAGE
            printk( KERN_ERR "VC_IOCTL_BOOT unsupported\n" );
            return -ENOTTY;
#else
#if 0
            // for some unknown reason, bootload works only on the first and then every other times
            if ( gBootloaded )
            {
               Bootload();
            }
#endif
            Bootload();
            break;
#endif
        }

        case VC_IOCTL_INIT_GPIO:
        {
            vc_host_init_pins();
            break;
        }

        case VC_IOCTL_RUN:
        {
            if (( arg == 0 ) || ( arg == 1 ))
            {
                vc_host_set_run_pin( arg );
            }
            else
            {
                return -EINVAL;
            }
            break;
        }

        case VC_IOCTL_HIB:
        {
            if (( arg == 0 ) || ( arg == 1 ))
            {
                vc_host_set_hibernate_pin( arg );
            }
            else
            {
                return -EINVAL;
            }
            break;
        }

        case VC_IOCTL_RESET:
        {
            if (( arg >= 0 ) && ( arg <= 2 ))
            {
                vc_host_set_reset_pin( arg );
            }
            else
            {
                return -EINVAL;
            }
            break;
        }

        case VC_IOCTL_LOCK_CREATE:
        {
            VC_FileData_t  *fileData = (VC_FileData_t *)file->private_data;
            VC_Lock_t       lock;

            if ( copy_from_user( &lock, (void *)arg, sizeof( lock )) != 0 )
            {
                return -EFAULT;
            }

            lock.lock = vc_lock_create_on( &fileData->lockList );

            if ( copy_to_user( (void *)arg, &lock, sizeof( lock )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }

        case VC_IOCTL_LOCK_OBTAIN:
        {
            VC_Lock_t   lock;
            Lock_t     *realLock;

            if ( copy_from_user( &lock, (void *)arg, sizeof( lock )) != 0 )
            {
                return -EFAULT;
            }

            realLock = (Lock_t *)lock.lock;
            if ( copy_from_user( realLock->fileNameBuf, lock.fileName, sizeof( realLock->fileNameBuf )) != 0 )
            {
                return -EFAULT;
            }
            realLock->fileNameBuf[ sizeof( realLock->fileNameBuf ) - 1 ] = '\0';


            vc_lock_obtain_dbg( lock.lock, realLock->fileNameBuf, lock.lineNum );
            break;
        }

        case VC_IOCTL_LOCK_RELEASE:
        {
            VC_Lock_t       lock;

            if ( copy_from_user( &lock, (void *)arg, sizeof( lock )) != 0 )
            {
                return -EFAULT;
            }
            vc_lock_release( lock.lock );
            break;
        }

        case VC_IOCTL_EVENT_CREATE:
        {
            VC_FileData_t   *fileData = (VC_FileData_t *)file->private_data;
            VC_Event_t       event;

            if ( copy_from_user( &event, (void *)arg, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }

            event.event = vc_event_create_on( &fileData->eventList );

            if ( copy_to_user( (void *)arg, &event, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }

        case VC_IOCTL_EVENT_WAIT:
        {
            VC_Event_t  event;
            Event_t    *realEvent;

            if ( copy_from_user( &event, (void *)arg, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }

            realEvent = (Event_t *)event.event;
            if ( copy_from_user( realEvent->fileNameBuf, event.fileName, sizeof( realEvent->fileNameBuf )) != 0 )
            {
                return -EFAULT;
            }
            realEvent->fileNameBuf[ sizeof( realEvent->fileNameBuf ) - 1 ] = '\0';

            vc_event_wait_dbg( event.event, realEvent->fileNameBuf, event.lineNum );
            break;
        }

        case VC_IOCTL_EVENT_STATUS:
        {
            VC_Event_t  event;

            if ( copy_from_user( &event, (void *)arg, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }

            event.status = vc_event_status( event.event );

            if ( copy_to_user( (void *)arg, &event, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }

        case VC_IOCTL_EVENT_CLEAR:
        {
            VC_Event_t  event;

            if ( copy_from_user( &event, (void *)arg, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }
            vc_event_clear( event.event );
            break;
        }

        case VC_IOCTL_EVENT_SET:
        {
            VC_Event_t  event;

            if ( copy_from_user( &event, (void *)arg, sizeof( event )) != 0 )
            {
                return -EFAULT;
            }
            vc_event_set( event.event );
            break;
        }

        case VC_IOCTL_HOST_READ:
        case VC_IOCTL_HOST_READ_SWAPPED:
        {
            VC_HostIO_t     hostIO;
            VC_FileData_t  *fileData = (VC_FileData_t *)file->private_data;
            int             bytesRemaining;
            uint8_t        *userBuf;
            uint32_t        vcAddr;
            int             rc = 0;

            if ( copy_from_user( &hostIO, (VC_HostIO_t *)arg, sizeof( hostIO )) != 0 )
            {
                return -EFAULT;
            }
            if ( down_interruptible( &fileData->lock ) != 0 )
            {
                return -ERESTARTSYS;
            }

            bytesRemaining = hostIO.numBytes;
            userBuf        = hostIO.buffer;
            vcAddr         = hostIO.vc_addr;

            while ( bytesRemaining > 0 )
            {
                int numBytes = bytesRemaining;
                if ( numBytes > VC_FILE_IO_SIZE )
                {
                    numBytes = VC_FILE_IO_SIZE;
                }
                if ( cmd == VC_IOCTL_HOST_READ )
                {
                    hostIO.result = vc_host_read_consecutive( fileData->ioBuf, vcAddr, numBytes, hostIO.channel );
                }
                else
                {
                    hostIO.result = vc_host_read_byteswapped( fileData->ioBuf, vcAddr, numBytes, hostIO.channel );
                }
                if ( hostIO.result != 0 )
                {
                    break;
                }

                if ( copy_to_user( userBuf, fileData->ioBuf, numBytes ) != 0 )
                {
                    rc = -EFAULT;
                    break;
                }

                userBuf        += numBytes;
                vcAddr         += numBytes;
                bytesRemaining -= numBytes;
            }
            up( &fileData->lock );

            if ( rc == 0 )
            {
                if ( copy_to_user( &((VC_HostIO_t *)arg)->result, &hostIO.result, sizeof( hostIO.result )) != 0 )
                {
                    rc = -EFAULT;
                }
            }
            return rc;
        }

        case VC_IOCTL_HOST_WRITE:
        case VC_IOCTL_HOST_WRITE_SWAPPED:
        {
            VC_HostIO_t     hostIO;
            VC_FileData_t  *fileData = (VC_FileData_t *)file->private_data;
            int             bytesRemaining;
            uint8_t        *userBuf;
            uint32_t        vcAddr;
            int             rc = 0;

            if ( copy_from_user( &hostIO, (VC_HostIO_t *)arg, sizeof( hostIO )) != 0 )
            {
                return -EFAULT;
            }
            if ( down_interruptible( &fileData->lock ) != 0 )
            {
                return -ERESTARTSYS;
            }

            bytesRemaining = hostIO.numBytes;
            userBuf        = hostIO.buffer;
            vcAddr         = hostIO.vc_addr;

            while ( bytesRemaining > 0 )
            {
                int numBytes = bytesRemaining;
                if ( numBytes > VC_FILE_IO_SIZE )
                {
                    numBytes = VC_FILE_IO_SIZE;
                }

                if ( copy_from_user( fileData->ioBuf, userBuf, numBytes ) != 0 )
                {
                    rc = -EFAULT;
                    break;
                }

                if ( cmd == VC_IOCTL_HOST_WRITE )
                {
                    hostIO.result = vc_host_write_consecutive( vcAddr, fileData->ioBuf, numBytes, hostIO.channel );
                }
                else
                {
                    hostIO.result = vc_host_write_byteswapped( vcAddr, fileData->ioBuf, numBytes, hostIO.channel );
                }
                if ( hostIO.result != 0 )
                {
                    break;
                }

                userBuf        += numBytes;
                vcAddr         += numBytes;
                bytesRemaining -= numBytes;
            }
            up( &fileData->lock );

            if ( rc == 0 )
            {
                if ( copy_to_user( &((VC_HostIO_t *)arg)->result, &hostIO.result, sizeof( hostIO.result )) != 0 )
                {
                    rc = -EFAULT;
                }
            }
            return rc;
        }

        case VC_IOCTL_HOST_SEND_INT:
        {
            vc_host_send_interrupt( (int)arg );
            break;
        }

        case VC_IOCTL_INTF_SEND_INT:
        {

            vc_interface_send_interrupt( (int )arg );
            break;
        }

        case VC_IOCTL_INTF_REG_EVENT_INT:
        {
            VC_RegEventInt_t    regEvent;

            if ( copy_from_user( &regEvent, (VC_RegEventInt_t *)arg, sizeof( regEvent )) != 0 )
            {
                return -EFAULT;
            }

            regEvent.result = vc_interface_register_event_int( regEvent.event, regEvent.mask );

            if ( copy_to_user( &((VC_RegEventInt_t *)arg)->result, &regEvent.result, sizeof( regEvent.result )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }


        case VC_IOCTL_SLEEP_MODE:
        {
#ifdef CONFIG_BCM_SLEEP_MODE

           VC_GenCmd_t gencmd;
           int core_freq = ( arg ) ? VC_MIN_VC02_CORE_FREQ:VC_MAX_VC02_CORE_FREQ; /* Set to 5Mhz in standby and 150Mhz in active mode */

           if ( !gInitialized )
           {
              printk( KERN_ERR "vc: Not Initialized\n" );
              return -EPERM;
           }

           sprintf(gencmd.cmd,"set_core_freq %d",core_freq);

           VC_DEBUG( Trace, "Calling vc_gencmd with '%s', len = %d\n", &gencmd.cmd[0], strlen( gencmd.cmd ));

           gencmd.response.err = vc_gencmd( gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );

           VC_DEBUG( Trace, "vc_gencmd returned err = %d\n", gencmd.response.err );
           if ( err != 0 )
           {
              return -EFAULT;
           }

#endif
           break;
        }

        case VC_IOCTL_BOOT_VC02:
        {
            VC_Boot_t   boot;
            void       *firmware;
            int         rc;

            printk( KERN_INFO "VC_IOCTL_BOOT_VC02\n" );

            if ( copy_from_user( &boot, (VC_Boot_t *)arg, sizeof( boot )) != 0 )
            {
                return -EFAULT;
            }

            printk( KERN_INFO "Allocating %u bytes\n", boot.len );

            if (( firmware = (void *)vmalloc( boot.len )) == NULL )
            {
               printk( KERN_ERR "Allocation failed\n" );
               return -ENOMEM;
            }
            rc = 0;

            if ( copy_from_user( firmware, boot.firmware, boot.len ) != 0 )
            {
                printk( KERN_ERR "copy_from_user failed\n" );
                rc = -EFAULT;
            }
            else
            {
               // need to byte swap 16-bit words in bin file
               int i;
               uint16_t *src16 = firmware;
               for (i=0; i<boot.len/2; i++)
               {
                  src16[i] = (src16[i] << 8) + (src16[i] >> 8);
               }
               printk( KERN_INFO "Reset VC02\n" );
               vc_host_reset();
               printk( KERN_INFO "vc_host_boot\n" );
               if ( vc_host_boot( NULL, firmware, boot.len, 0 ) < 0 )
               {
                  printk( KERN_ERR "vc_host_boot of %u bytes failed\n", boot.len );
                  rc = -EIO;
               }

               printk( KERN_INFO "vc_host_boot done\n" );
            }
            vfree( firmware );

            if ( rc != 0 )
            {
               return rc;
            }

            break;
        }

        case VC_IOCTL_INIT_SDRAM:
        {
            printk( KERN_INFO "Initializing VC02 SDRAM\n" );
            vc_host_init_sdram();
            break;
        }
#if defined( CONFIG_BCM_HALAUDIO_MIXER )
        case VC_IOCTL_START_AUDIO:
        {
           VC_Audio_t audio;
           if ( copy_from_user( &audio, (VC_Audio_t *)arg, sizeof( audio )) != 0 )
           {
               printk( KERN_ERR "copy_from_user failed\n" );
               return -EFAULT;
           }
           printk( KERN_INFO "VC02 audio start record %d, stream num %d\n", 
                  audio.record, audio.stream_num);
           vc_audio_start( &audio );
           break;
        }

        case VC_IOCTL_STOP_AUDIO:
        {
           VC_Audio_t audio;
           if ( copy_from_user( &audio, (VC_Audio_t *)arg, sizeof( audio )) != 0 )
           {
               printk( KERN_ERR "copy_from_user failed\n" );
               return -EFAULT;
           }

           printk( KERN_INFO "VC02 audio done record %d, streamnum %d \n", 
                 audio.record, audio.stream_num );
           vc_audio_stop( &audio );
           break;
        }

        case VC_IOCTL_REDIRECT_AUDIO:
        {
           VC_Audio_reDirect_t redirect;
           if ( copy_from_user( &redirect, (VC_Audio_reDirect_t *)arg, sizeof( redirect )) != 0 )
           {
               printk( KERN_ERR "copy_from_user failed\n" );
               return -EFAULT;
           }

           if( vc_audio_redirect( &redirect ) != 0 )
           {
              return EFAULT;
           }
           break;
        }
#endif

        case VC_IOCTL_SET_GMTOFF:
        {
           gVcGmtOffset = arg;
           break;
        }

        case VC_IOCTL_HOST_REG:
	{
    	   VC_HostReg_t hostreg;

           if ( copy_from_user( &hostreg, (void *)arg, sizeof( hostreg )) != 0 )
           {
               return -EFAULT;
           }
    	   if (( err = vc_host_reg( &hostreg )) != 0 )
           {
               return err;
           }
           if ( hostreg.op == VC_HOSTREG_OP_READ )
           {
               if ( copy_to_user( (void *)arg, &hostreg, sizeof( hostreg )) != 0 )
               {
                   return -EFAULT;
               }
           }
    	   break;
	}

        default:
        {
            printk( KERN_WARNING "vc: Unrecognized ioctl: '0x%x'\n", cmd );
            return -ENOTTY;
        }
    }

    return 0;

} // vc_ioctl

/****************************************************************************
*
*  vc_open
*
***************************************************************************/

// static
int vc_open( struct inode *inode, struct file *file )
{
    VC_FileData_t   *fileData;

    VC_DEBUG( Trace, "vc_open called\n" );

    if (( fileData = kcalloc( 1, sizeof( VC_FileData_t ), GFP_KERNEL )) == NULL )
    {
        return -ENOMEM;
    }

    if (( fileData->ioBuf = kcalloc( 1, VC_FILE_IO_SIZE, GFP_KERNEL )) == NULL )
    {
        kfree( fileData );
        return -ENOMEM;
    }

    init_MUTEX( &fileData->lock );

    INIT_LIST_HEAD( &fileData->eventList );
    INIT_LIST_HEAD( &fileData->lockList );

    file->private_data = fileData;

    return 0;

} // vc_open

/****************************************************************************
*
*  vc_read
*
***************************************************************************/

// static
ssize_t vc_read( struct file *file, char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} // vc_read

/****************************************************************************
*
*  vc_release
*
***************************************************************************/

// static
int vc_release( struct inode *inode, struct file *file )
{
    VC_FileData_t   *fileData;

    VC_DEBUG( Trace, "vc_release called\n" );

    if (( fileData = (VC_FileData_t *)file->private_data ) != NULL )
    {
        VC_DEBUG( Trace, "About to call vc_lock_free_list\n" );
        vc_lock_free_list( &fileData->lockList );
        VC_DEBUG( Trace, "About to call vc_event_free_list\n" );
        vc_event_free_list( &fileData->eventList );
        VC_DEBUG( Trace, "Back from call vc_event_free_list\n" );

        kfree( fileData->ioBuf );
        kfree( fileData );
    }

    VC_DEBUG( Trace, "vc_release: leaving\n" );

    return 0;

} // vc_release

/****************************************************************************
*
*  vc_write
*
***************************************************************************/

// static
ssize_t vc_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} // vc_write

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations vc_fops =
{
    owner:      THIS_MODULE,
    open:       vc_open,
    release:    vc_release,
    read:       vc_read,
    write:      vc_write,
    ioctl:      vc_ioctl,
};

/****************************************************************************
*
*  vc_irq
*
*       Interrupt handler for transfer to/from the VC02
*
***************************************************************************/

#if USE_IRQ
irqreturn_t vc_irq( int irq, void *devId )
{
    (void)irq;
    (void)devId;

    if ( gVC_InterruptsEnabled )
    {
        VC_DEBUG( HostIrq, "triggered\n" );

        vc_event_set( gVC_IrqEvent );
    }

    return IRQ_HANDLED;

} // vc_irq

#endif  // USE_IRQ

/****************************************************************************
*
*  vc_irq_task
*
*       Task handler for the IRQ
*
***************************************************************************/

#if USE_IRQ
int vc_irq_task( void *data )
{
    int     rc;

    VC_DEBUG( Trace, "Starting\n" );

    gVC_IrqEvent = vc_event_create();

    // Use VC02 in interrupt driven mode, not polling mode

    gVC_InterruptsEnabled = 1;

    gpio_request( HW_GPIO_VID_INT_PIN, "vc_irq" );

    if (( rc = request_irq( gpio_to_irq( HW_GPIO_VID_INT_PIN ), vc_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING, "vc_drv", NULL )) != 0 )
    {
        printk( KERN_ERR "request_irq for GPIO %d failed: %d\n", HW_GPIO_VID_INT_PIN, rc );
        return rc;
    }
    vc_event_blocking();

    while ( 1 )
    {
        vc_adjust_thread_priority( &gVcPriorityIrq );

        vc_event_wait( gVC_IrqEvent );
        vc_interface_interrupt_handler();
    }

   return 0;
} // vc_irq_task

#endif  // USE_IRQ

/****************************************************************************
*
*  vc_init_drv
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init vc_init_drv( void )
{
   int rc;

   printk( banner );

   memset( &gLoopbackArgs, 0, sizeof( gLoopbackArgs ) );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
   if ( gSysCtlHeader != NULL )
   {
      gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif
   if (( rc = register_chrdev( BCM_VC02_MAJOR, "vc", &vc_fops )) < 0 )
   {
      printk( KERN_WARNING "vc: register_chrdev failed for major %d\n", BCM_VC02_MAJOR );
      return rc;
   }

   InitFrameBufferList( &gVC_FramesToVCList );

#if CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL
   {
      int bPaused = 0;

      #if CONFIG_BCM_DEBUG_PAUSE_VC
      {
         // This provides an opportunity for the user to pause the initialization
         // sequence and load the VC image via the JTAG (instead of the normal
         // host bootload sequence).
         bPaused = debug_pause( "Load VC image via JTAG\n" );
      }
      #endif

      if ( !bPaused )
      {
          Bootload();

          // Sleep for 3 seconds while the VC02 initializes
          set_current_state(  TASK_INTERRUPTIBLE );
          schedule_timeout( 3 * HZ );
      }
   }
#endif

   if ( Init() != 0 )
   {
      printk( KERN_ERR "vc_init_drv: VC02 interface initialization failed\n" );
   }

   memset( &gInAllocBuffError, 0, sizeof( VC_ERRORlog_t ) );
   memset( &gEgOutOfBuffError, 0, sizeof( VC_ERRORlog_t ) );
   memset( &gDiscardEgressError, 0, sizeof( VC_ERRORlog_t ) );

   vc_init_gpio();

   return 0;

} // vc_init_drv

/****************************************************************************
*
*  vc_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit vc_exit_drv( void )
{
    VC_DEBUG( Trace, "vc_exit_drv called\n" );

    free_irq( gpio_to_irq( HW_GPIO_VID_INT_PIN ), NULL );

    if ( gSysCtlHeader != NULL )
    {
        unregister_sysctl_table( gSysCtlHeader );
    }

} // vc_exit_drv

/****************************************************************************
*
*  VC_DequeueFrameBuffer
*
*   Removes a buffer from the queue. If the queue is empty, then it will
*   block.
*
***************************************************************************/
//static
VC_FrameBuffer_t *VC_DequeueFrameBuffer( VC_FrameBufferList_t *fList )
{
    // First of wait for a buffer to be available

    if ( down_interruptible( &fList->availSem ) != 0 )
    {
        return NULL;
    }

    return InternalDequeueFrameBuffer( fList );

} // VC_DequeueFrameBuffer

/****************************************************************************
*
*  VC_GetEgressQDepth
*
*   Removes a buffer from the queue. If the queue is empty, then it will
*   block.
*
***************************************************************************/
//static
int VC_GetEgressQDepth( VC_FrameBufferList_t *fList )
{
    VC_FrameBuffer_t    *headFrame, *tailFrame;
    VC_FrameBufferHdr_t    *frameBufHdr;
    int timestampdiff=0;

    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        if ( !(list_empty( &fList->list )))
        {
           /* Get Head node */
            struct list_head *node = fList->list.next;

            frameBufHdr = list_entry( node, VC_FrameBufferHdr_t, node );
            headFrame = list_entry( frameBufHdr, VC_FrameBuffer_t, hdr );
           /* Get Tail node */
            node = fList->list.prev;
            frameBufHdr = list_entry( node, VC_FrameBufferHdr_t, node );
            tailFrame = list_entry( frameBufHdr, VC_FrameBuffer_t, hdr );
            if( (headFrame != NULL) && (tailFrame!= NULL) && ( headFrame!= tailFrame) )
            {
               /* if any of the timestamp is 0, or if the frames belong to a different stream, do not compare that */
               if( (headFrame->hdr.frame.timestamp == 0) || 
                   (headFrame->hdr.frame.stream_num != tailFrame->hdr.frame.stream_num) )
               {
                  node = fList->list.next;
                  while( (headFrame != NULL) && 
                         (headFrame != tailFrame) && 
                         ( (headFrame->hdr.frame.timestamp == 0) || 
                           (headFrame->hdr.frame.stream_num != tailFrame->hdr.frame.stream_num) ) )
                  {
                     node = node->next;
                     frameBufHdr = list_entry( node, VC_FrameBufferHdr_t, node );
                     headFrame = list_entry( frameBufHdr, VC_FrameBuffer_t, hdr );
                  }
               }
            }
            if ((headFrame != tailFrame) && (headFrame != NULL) && (tailFrame != NULL))
            {
               timestampdiff= (tailFrame->hdr.frame.timestamp - headFrame->hdr.frame.timestamp);
            }
        }
        spin_unlock_irqrestore( &fList->lock, flags );
    }
    return timestampdiff;

} // VC_GetEgressQDepth


/****************************************************************************
*
*  DequeueFrameBufferNoBlock
*
*   Removes a buffer from the queue. If the queue is empty, then it will
*   return NULL. This function is suitable for calling from within an IRQ.
*
***************************************************************************/

#if 0
//static
VC_FrameBuffer_t *VC_DequeueFrameBufferNoBlock( VC_FrameBufferList_t *fList )
{
    // First of wait for a buffer to be available

    if ( down_trylock( &fList->availSem ) != 0 )
    {
        return NULL;
    }

    return InternalDequeueFrameBuffer( fList );

} // VC_DequeueFrameBufferNoBlock
#endif

/****************************************************************************
*
*  InternalDequeueFrameBuffer
*
*   Removes a buffer from the queue. This function assumes that the caller
*   has already downed the semaphore.
*
***************************************************************************/

// static
VC_FrameBuffer_t *InternalDequeueFrameBuffer( VC_FrameBufferList_t *fList )
{
    VC_FrameBuffer_t    *frameBuf;
    VC_FrameBufferHdr_t    *frameBufHdr;

    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        if ( list_empty( &fList->list ))
        {
            frameBuf = NULL;
        }
        else
        {
            struct list_head *deqNode = fList->list.next;
            list_del_init( deqNode );

            frameBufHdr = list_entry( deqNode, VC_FrameBufferHdr_t, node );
            frameBuf = list_entry( frameBufHdr, VC_FrameBuffer_t, hdr );
        }
        spin_unlock_irqrestore( &fList->lock, flags );
    }

    VC_DEBUG( Trace, "returning frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    return frameBuf;

} // InternalDequeueFrameBuffer

/****************************************************************************
*
*  VC_EnqueueFrameBuffer
*
*   Adds a buffer to the queue.
*
***************************************************************************/
//static
void VC_EnqueueFrameBuffer( VC_FrameBufferList_t *fList, VC_FrameBuffer_t *frameBuf )
{
    VC_DEBUG( Trace, "frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    // Add the frame to the end of the list

    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        list_add_tail( &frameBuf->hdr.node, &fList->list );

        spin_unlock_irqrestore( &fList->lock, flags );
    }

    // Wakeup anybody waiting for it.

    up( &fList->availSem );

} // VC_EnqueueFrameBuffer

// static
void InitFrameBufferList( VC_FrameBufferList_t *fList )
{
    INIT_LIST_HEAD( &fList->list );

    spin_lock_init(  &fList->lock );
    sema_init( &fList->availSem, 0 );

} // InitFrameBufferList


/****************************************************************************
*
*  CB_AllocFrameBuffer
*
*     Retrieve an available buffer.
*
***************************************************************************/

// static
static  FRMFWD_FRAME_T *CB_AllocFrameBuffer( int streamNum, int length_in_bytes )
{
   if ( vc_hostg.callback == NULL )
   {
      printk( KERN_ERR "CB_AllocFrameBuffer: unable to allocate frame - no vc_get_framebuf callback registered\n" );
   }
   else
   {
      VC_FrameBuffer_t *frameBuf = vc_hostg.callback->vc_get_framebuf(streamNum, length_in_bytes );

      if ( frameBuf != NULL )
      {
         return (FRMFWD_FRAME_T *)&frameBuf->hdr.frame;
      }

      vc_hostdrv_errlog( "CB_GetFrameBuffer allocation failed", &gInAllocBuffError );
   }

   return NULL;

} // CB_AllocFrameBuffer

/****************************************************************************
*
*  CB_FrameReceived
*
*     Called when we receive a frame.
*
***************************************************************************/

// static
static void CB_FrameReceived( FRMFWD_FRAME_T *fwdFrame, unsigned int lengthinbyte, int streamNum )
{
    VC_FrameBufferHdr_t *frameBufHdr = container_of( &(fwdFrame->info), VC_FrameBufferHdr_t, frame );
    VC_FrameBuffer_t *frameBuf = container_of( frameBufHdr, VC_FrameBuffer_t, hdr );

    if ( vc_hostg.callback == NULL )
    {
       printk( KERN_ERR "CB_FrameReceived but no vc_enqueue_framebuf registered\n" );
    }
    else
    {
       vc_hostg.callback->vc_enqueue_framebuf( frameBuf, lengthinbyte+sizeof(VC_FrameBufferHdr_t)-sizeof(FRMFWD_FRAME_INFO_T), streamNum );
       gVcFramesRcvd++;
    }

} // CB_FrameReceived

/****************************************************************************
*
*  CB_FrameSent
*
*     Called when a frame has been sent successfully to the VC02
*
***************************************************************************/

// static
static void CB_FrameSent( FRMFWD_FRAME_INFO_T *frameInfo, int streamNum )
{
    gVcFramesAcked++;

    /* Semaphore count can be more than number of buffers available
     * in the following scenario:
     * Host sends three frames to VC02 and so the host freebuf is equal to
     * 0. Now VC02 has only read the first packet from interface and sends the
     * number of buffers available to 4. So, host updates its semaphore to 4-1=3. After that the VC02 has read the remaining two packets and sends a num of buffers as 2. So, in this case host copy of semaphore is 3 while the VC02 free buffers is 2. */
#if 0
    if (semCount >= frameInfo->num_buf_available)
    {
       printk(KERN_INFO "Host Semaphore temp out of sync %d %d\n", semCount, (frameInfo->num_buf_available-1));
    }
#endif

} // CB_FrameSent

/****************************************************************************
*
*  CB_GetFrameBuffer
*
*     Retrieve an available buffer.
*
***************************************************************************/

// static
static void CB_FrameNotSentOutOfBuffers( FRMFWD_FRAME_INFO_T *frameInfo, int streamNum )
{
    gVcFramesLost++;
    vc_hostdrv_errlog( "Egress Out of buffers", &gEgOutOfBuffError );

} // CB_FrameNotSentOutOfBuffers

/****************************************************************************
*
*  Dump
*
*     Dumps some information about the VC02 interface.
*
***************************************************************************/

extern VC_MSGFIFO_INTERFACE_T msgfifo_interface[VC_NUM_INTERFACES];

// static
void Dump( void )
{
    int i;

    printk( KERN_INFO "vc_interface_base: %08x\n", vc_interface_base );

    printk( KERN_INFO "vc_sharedmem_header inR:0x%02x inA:0x%02x inX:0x%02x outR:0x%02x outA:0x%02x outX:0x%02x\n",
            vc_sharedmem_header.vin_int_req,
            vc_sharedmem_header.vin_int_ack,
            vc_sharedmem_header.vin_int_req ^ vc_sharedmem_header.vin_int_ack,
            vc_sharedmem_header.vout_int_req,
            vc_sharedmem_header.vout_int_ack,
            vc_sharedmem_header.vout_int_req ^ vc_sharedmem_header.vout_int_ack );

    for (i = 0; i < VC_NUM_INTERFACES; i++)
    {
        uint16_t                 iface_offset;
        VC_MSGFIFO_INTERFACE_T   *msgfifo;
        uint8_t                  buffer[ 64 ];
        int                      bytes_avail;

        vc_msgfifo_read_refresh( i );
        vc_msgfifo_write_refresh( i );

        iface_offset = vc_sharedmem_header.iface[ i ];

        msgfifo = &msgfifo_interface[ i ];

        if ( VC_VTOH16( msgfifo->itype ) == VC_ITYPE_MSGFIFO )
        {
           printk( KERN_INFO "iface[%d]: 0x%04x I:%04x-%04x O:%04x-%04x Ir%04x Iw%04x Or%04x Ow%04x\n"
                " VC --> Host Avail: %4d Space: %4d\n"
                " Host --> VC Avail: %4d Space: %4d\n",
                   i, iface_offset,
                   VC_VTOH16(msgfifo->vin_fmin),  VC_VTOH16(msgfifo->vin_fmax),  VC_VTOH16(msgfifo->vout_fmin),  VC_VTOH16(msgfifo->vout_fmax),
                   VC_VTOH16(msgfifo->vin_frptr), VC_VTOH16(msgfifo->vin_fwptr), VC_VTOH16(msgfifo->vout_frptr), VC_VTOH16(msgfifo->vout_fwptr),
                vc_msgfifo_input_bytes_available( i ),
                vc_msgfifo_input_space_available( i ),
                vc_msgfifo_output_bytes_available( i ),
                vc_msgfifo_output_space_available( i ));

           bytes_avail = vc_msgfifo_input_bytes_available( i );
           if ( bytes_avail > 0 )
           {
              if ( bytes_avail > sizeof( buffer ))
              {
                 bytes_avail = sizeof( buffer );
              }

              vc_host_read_consecutive( buffer, vc_interface_base + VC_VTOH16( msgfifo->vout_frptr ), bytes_avail, 0 );
              vc_dump_mem( "", msgfifo->vout_frptr, buffer, bytes_avail );
           }
        }
        else
        {
           printk( KERN_INFO "vc_sharedmem_header.iface[%d]: 0x%04x - Not a msgfifo\n",
                   i, iface_offset );
        }
    }

} // Dump

/****************************************************************************
*
*  show_vc02
*
*     Function called in response to a SysReq-V key sequence.
*
***************************************************************************/

void show_vc02( void )
{
   Dump();
}

/****************************************************************************
*
*  vc_adjust_thread_priority
*
*     Adjusts the thread priority, if required
*
***************************************************************************/

void vc_adjust_thread_priority( int *requestedPriority )
{
    int rc;

    if (( current->policy != SCHED_FIFO ) || ( current->rt_priority != *requestedPriority ))
    {
        struct sched_param param;

        param.sched_priority = *requestedPriority;

        if (( rc = sched_setscheduler( current, SCHED_FIFO, &param )) == 0 )
        {
            printk( KERN_INFO "%s priority set to %u\n", current->comm, current->rt_priority );
        }
        else
        {
            printk( KERN_ERR "sched_setscheduler failed: %d\n", rc );
        }
    }

} // vc_adjust_thread_priority

/****************************************************************************
*
*  FrameReceiveThread
*
*     copies frames from the VC02 into the gVC_FramesFromVCList
*
***************************************************************************/

// static
int FrameReceiveThread( void *data )
{
    printk( KERN_INFO "******************FRAMERECEIVETHREAD: STARTING******************\n" );

    while ( 1 )
    {
        vc_adjust_thread_priority( &gVcPriorityRecv );

        // The following function will call the appropriate callback based
        // on what it receives.

#if USE_IRQ
        vc_event_wait( vc_frmfwd_read_event() );
#else
        set_current_state(  TASK_INTERRUPTIBLE );

        schedule_timeout( 2 );
#endif

        vc_frmfwd_poll_message_fifo();
    }

    return 0;

} // FrameReceiveThread

/****************************************************************************
*
*  ReadVCRegisters
*
*     copies frames from the VC02 into the gVC_FramesFromVCList
*
***************************************************************************/

// static
int ReadVCRegisters( void *data )
{
    printk( KERN_INFO "******************ReadVCRegisters: STARTING******************\n" );



    while ( 1 )
    {
        vc_adjust_thread_priority( &gVcPriorityReadRegs );

        // The following function will call the appropriate callback based
        // on what it receives.

        set_current_state(  TASK_INTERRUPTIBLE );

        schedule_timeout( 100 );

        vc_host_read_VC02_STC_reg();
    }

    return 0;

} // FrameReceiveThread


/****************************************************************************
*
*  vc_sendframe
*
*     Dequeues frame form hausware queue and sends it to VC02. Currently,
*     the data is enqueued in a list and the FrameSendThread reads it out
*     from this queue and frees the buffer. Once we move th ISR model,
*     this function would change so that we send the buffer directly to the
*     VC02 and free the buffer at the end
*
***************************************************************************/
#define MONITOR_EGRESS_VIDEO_TS  0
#if MONITOR_EGRESS_VIDEO_TS
#include <linux/broadcom/timer.h>


static unsigned int lastTick = 0;
static unsigned int lastframeTS = 0;
#endif
void vc_sendframe ( int devhdl )
{
   VC_FrameBuffer_t   *frameBuf;
   char errorStr[100];
   int semaCount;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
   /* FIXME: we should not access the structure semaphore member directly */
   semaCount = gVC_FramesToVCList.availSem.count;
#else
   semaCount = atomic_read(&gVC_FramesToVCList.availSem.count);
#endif
   unsigned int begin_frameTSdiff=VC_GetEgressQDepth( &gVC_FramesToVCList);

   if ( vc_hostg.callback == NULL )
   {
      // Nothing we can do - drop the frame

      printk( KERN_ERR "vc_sendframe: Can't send frame; vc_dequeue_framebuf not registered\n" );

      return;
   }

   while (( frameBuf = vc_hostg.callback->vc_dequeue_framebuf( devhdl )) != NULL )
   {
#if VC_EGRESS_PKT_BASED_FLOWCTRL
      if ( semaCount < NUM_MAX_FRAMES)
#else
      unsigned int frameTSdiff=VC_GetEgressQDepth( &gVC_FramesToVCList);
      if ((begin_frameTSdiff == 0) || (frameTSdiff < NUM_MAX_MSEC))
#endif
      {
#if MONITOR_EGRESS_VIDEO_TS
         unsigned int clk=timer_get_tick_count();
         unsigned int diff = clk-lastTick;
         unsigned int frameTSdiff = frameBuf->hdr.frame.timestamp - lastframeTS;
         lastframeTS = frameBuf->hdr.frame.timestamp;
         lastTick = clk;
         printk( KERN_INFO "FS %d, %d\n", diff, frameTSdiff);
#endif
         VC_EnqueueFrameBuffer( &gVC_FramesToVCList, frameBuf );

         /* Add 2 to sempahore count since semaphore count is -1 when waiting. */
         gCurrFramesInEgressQ = semaCount +2;
         //printk("semcnt=%d\n", sema_count(&gVC_FramesToVCList.availSem));
         /* Update the max  packets in queue stat  */
         if (gMaxFramesInEgressQ < gCurrFramesInEgressQ)
         {
            gMaxFramesInEgressQ = gCurrFramesInEgressQ;
         }

         gAveFramesInEgressQ = (gAveFramesInEgressQ + gCurrFramesInEgressQ)>> 1;

         VC_DEBUG( Trace, "FrameBuffer %d, %d, %d, %d,%d, %d\n", *(int *)frameBuf, *((int *) frameBuf + 1),*((int *) frameBuf + 2), *((int *) frameBuf + 3), *((int *) frameBuf + 4), *((int *) frameBuf + 5)  );
      }
      else
      {
        gVcFramesLost++;

        snprintf( &errorStr[0], sizeof( errorStr ), "Discard egress, start depth %u, current depth %u\n", 
              begin_frameTSdiff, frameTSdiff );
        
        vc_hostdrv_errlog( &errorStr[0], &gDiscardEgressError );

        /* Free the original buffer */
        vc_hostg.callback->vc_free_framebuf( frameBuf );
      }
   }
}
/****************************************************************************
*
*  FrameSendThread
*
*     Copies frames from the gVC_FramesToVCList into the VC02
*
***************************************************************************/

// static
int FrameSendThread( void *data )
{
    VC_FrameBuffer_t   *frameBuf;

    printk( KERN_INFO "*******************FRAMESENDTHREAD: STARTING*******************\n" );
    while ( (frameBuf = VC_DequeueFrameBuffer( &gVC_FramesToVCList )) != NULL  )
    {

       vc_adjust_thread_priority( &gVcPrioritySend );
       if ( vc_frmfwd_send_frame( container_of( &frameBuf->hdr.frame, FRMFWD_FRAME_T, info ), 1) == VC_RESP_OK )
       {
          gVcFramesSent++;
       }
       else
       {
          gVcFramesSendErr++;

          printk( KERN_ERR "FrameSendThread: Error calling vc_frmfwd_send_frame\n" );
       }

       //printk( "s" );

       /* Free the original buffer */

       if ( vc_hostg.callback == NULL )
       {
          printk( KERN_ERR "FrameSendThread: No vc_free_framebuf registered\n" ); 
       }
       else
       {
          vc_hostg.callback->vc_free_framebuf( frameBuf );
       }

    }
    printk( KERN_INFO "FrameSendThread: exiting\n" );

    return 0;

} // FrameSendThread

/****************************************************************************
*
*  HostReqThread
*
*     Services requests from the VideoCore. e.g. the VC display manager
*     uses the host-req service to pull frame-buffer data from the host.
*
***************************************************************************/

// static
int HostReqThread( void *data )
{
    while ( 1 )
    {
        vc_adjust_thread_priority( &gVcHostReqPriority );

#if USE_IRQ
        vc_event_wait( vc_hostreq_read_event() );
#else
        set_current_state(  TASK_INTERRUPTIBLE );

        schedule_timeout( 2 );
#endif

        vc_hostreq_poll_message_fifo();
    }

    return 0;

} // HostReqThread



/****************************************************************************
*
*  Init
*
*     Initializes the VC02 interface
*
***************************************************************************/

// static
int GencmdInit( void )
{
    int rc;

    VC_DEBUG( Trace, "vc: About to call vc_interface_init\n" );
    rc = vc_interface_init();
    VC_DEBUG( Trace, "vc: - vc_interface_init returned %d\n", rc );
    if ( rc != 0 )
    {
        return rc;
    }
    VC_DEBUG( Trace, "vc: About to call vc_gencmd_init\n" );
    rc = vc_gencmd_init();
    VC_DEBUG( Trace, "vc: vc_gencmd_init returned %d\n", rc );
    if ( rc < 0 )
    {
        return rc;
    }

    gInitialized = 1;

    return 0;
}

/****************************************************************************
*
*  Init
*
*     Initializes the VC02 interface
*
***************************************************************************/

// static
int Init( void )
{
    int rc;

#if USE_IRQ

    // Start the VC02 IRQ Handler thread

    if (( gIrqTask == NULL ) || IS_ERR( gIrqTask ))
    {
        gIrqTask = kthread_run( vc_irq_task, NULL, "vcIrq" );
        if ( IS_ERR( gIrqTask ))
        {
            printk( KERN_ERR "Init: failed to start vc_irq_task: %ld\n", PTR_ERR( gIrqTask ));
            return -1;
        }
    }
#endif

    VC_DEBUG( Trace, "vc: About to call vc_interface_init\n" );
    rc = vc_interface_init();
    VC_DEBUG( Trace, "vc: - vc_interface_init returned %d\n", rc );
    if ( rc != 0 )
    {
        return rc;
    }
    VC_DEBUG( Trace, "vc: About to call vc_gencmd_init\n" );
    rc = vc_gencmd_init();
    VC_DEBUG( Trace, "vc: vc_gencmd_init returned %d\n", rc );
    if ( rc < 0 )
    {
        return rc;
    }

    VC_DEBUG( Trace, "vc: About to call vc_hostreq_init\n" );
    rc = vc_hostreq_init();
    VC_DEBUG( Trace, "vc: vc_hostreq_init returned %d\n", rc );
    if ( rc < 0 )
    {
        return rc;
    }

    VC_DEBUG( Trace, "vc: About to call vc_dispman_init\n" );
    rc = vc_dispman_init();
    VC_DEBUG( Trace, "vc: vc_dispman_init returned %d\n", rc );
    if ( rc < 0 )
    {
        return rc;
    }

    VC_DEBUG( Trace, "vc: About to call vc_frmfwd_init\n" );
    rc = vc_frmfwd_init();
    VC_DEBUG( Trace, "vc: vc_frmfwd_init returned %d\n", rc );
    if ( rc < 0 )
    {
        return rc;
    }

#if defined( CONFIG_BCM_HALAUDIO_MIXER )
    VC_DEBUG( Trace, "vc: About to call vc_audiohdl_init\n" );
    rc = vc_audiohdl_init();
    VC_DEBUG( Trace, "vc: vc_audiohdl_init returned %d\n", rc );
    if( rc < 0 )
    {
       return rc;
    }
#endif

    // Start the threads which will be used to deliver frames to/from the VC02

    if (( gFrameSendTask == NULL ) || IS_ERR( gFrameSendTask ))
    {
        vc_frmfwd_set_cmd_callback( &gFrameForwarderCallbacks );

        gFrameSendTask = kthread_run( FrameSendThread, NULL, "vcSend" );
        if ( IS_ERR( gFrameSendTask ))
        {
            printk( KERN_ERR "Init: failed to start FrameSendThread: %ld\n", PTR_ERR( gFrameSendTask ));
            return -1;
        }
    }

    if (( gFrameRecvTask == NULL ) || IS_ERR( gFrameRecvTask ))
    {
        gFrameRecvTask = kthread_run( FrameReceiveThread, NULL, "vcRecv" );
        if ( IS_ERR( gFrameRecvTask ))
        {
            printk( KERN_ERR "Init: failed to start FrameRecvThread: %ld\n", PTR_ERR( gFrameRecvTask ));
            return -1;
        }
    }

    if (( gReadRegsTask == NULL ) || IS_ERR( gReadRegsTask ))
    {
        gReadRegsTask = kthread_run( ReadVCRegisters, NULL, "vcReadRegs" );
        if ( IS_ERR( gReadRegsTask ))
        {
            printk( KERN_ERR "Init: failed to start ReadVCRegisters: %ld\n", PTR_ERR( gReadRegsTask ));
            return -1;
        }
    }

    if (( gHostReqTask == NULL ) || IS_ERR( gHostReqTask ))
    {
        gHostReqTask = kthread_run( HostReqThread, NULL, "vcHostReq" );
        if ( IS_ERR( gHostReqTask ))
        {
            printk( KERN_ERR "Init: failed to start HostReqThread: %ld\n", PTR_ERR( gHostReqTask ));
            return -1;
        }
    }


    gInitialized = 1;

    return 0;

} // Init

/****************************************************************************
*
*  Bootload
*
*     Load code and start VC02.  vc_host_reset should be performed prior to this.
*
***************************************************************************/

#if CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL
// static
void VC_INIT Bootload( void )
{
   /* boot VMCS */

   vc_host_init_pins();
   vc_host_reset();
   vc_host_init();

   printk( KERN_INFO "Loading code into VC02 ...\n" );
   if ( vc_host_boot(0, (void *)VC02_BIN_FNAME, sizeof(VC02_BIN_FNAME), 0) >= 0 )
   {
      printk( KERN_INFO "VC02 Bootload complete\n" );

      // Note in our record of our state that we're running.
      vc_state.power_state = VC_POWER_STATE_OFF;
      vc_state_init();
      vc_state.power_state = VC_POWER_STATE_RUNNING;

      gBootloaded = 1;
   }
   else
   {
      printk( KERN_ERR "VC02 Bootload failed\n" );
   }
}
#endif // CFG_GLOBAL_VIDEO_BOOT_FROM_KERNEL

/****************************************************************************
*
*  vc_delay_msec
*
*   Delays for at least the specified number of milliseconds
*
***************************************************************************/

void vc_host_delay_msec( unsigned msec )
{
    set_current_state(  TASK_INTERRUPTIBLE );
    schedule_timeout( msecs_to_jiffies( msec ));

} // vc_host_delay_msec

/****************************************************************************
*
*  vc_is_initialized.
*
***************************************************************************/

int vc_is_initialized( void )
{
   return ( gInitialized );

} // vc_is_initialized

/****************************************************************************
*
*  StartFrameLoopback
*
*     Loops back frames received from the VC02
*
***************************************************************************/

static void StartFrameLoopback( int decode_stream )
{
    VC_HOSTCALLBACK_T   loopbackCallbacks =
    {
        loopbackGetFrameBuf,
        loopbackEnqueueFrameBuf,
        loopbackDequeueFrameBuf,
        loopbackFreeFrameBuf
    };


    if ( gLoopbackArgs.bLoopbackEnabled )
    {
        // We are already in loopback mode.
        return;
    }

    // Allocate the frame buffers.

    InitFrameBufferList( &gLoopbackFreeFramesList );
    InitFrameBufferList( &gLoopbackFramesFromVCList );

    {
        int i;

        VC_DEBUG( Trace, "sizeof( VC_FrameBuffer_t ) = %d\n", sizeof( VC_FrameBuffer_t ));

        for ( i = 0; i < NUM_FRAME_BUFFERS; i++ )
        {
            VC_FrameBuffer_t *frameBuf = kmalloc( sizeof( VC_FrameBuffer_t ) +
                                                  VC_FFMAX_CMD_DATA,
                                                  GFP_KERNEL );

            if ( frameBuf == NULL )
            {
                printk( KERN_ERR "vc: Unable to allocate loopback buffers\n" );
                break;
            }

            VC_EnqueueFrameBuffer( &gLoopbackFreeFramesList, frameBuf );
        }
    }

    gLoopbackArgs.decodeStream   = decode_stream;

    // Install loopback callbacks.

    vc_frmfwd_set_cmd_callback( &gFrameForwarderCallbacks );
    gLoopbackArgs.prevCallbacks = vc_host_set_cmd_callback( &loopbackCallbacks ); 

    gLoopbackArgs.bLoopbackEnabled = 1;

} // StartFrameLoopback

/****************************************************************************
*
*  StopFrameLoopback
*
*     Stops the frame loopback
*
***************************************************************************/

static void StopFrameLoopback( void )
{
    VC_FrameBuffer_t *frameBuf;

    if ( !gLoopbackArgs.bLoopbackEnabled )
    {
        // We're not in loopback mode, just bail...
        return;
    }


    // Assume that the encoder has been stopped prior to calling this function.
    // Allow things to settle down, and all packets to be cleared from the
    // message fifos.
    ssleep( 1 );

    // Free allocated memory.
    while ( ( frameBuf = InternalDequeueFrameBuffer( &gLoopbackFreeFramesList ) ) != NULL )
    {
        kfree( frameBuf );
    }

    // Re-install old callbacks.
    vc_frmfwd_set_cmd_callback( &gFrameForwarderCallbacks );
    vc_host_set_cmd_callback( gLoopbackArgs.prevCallbacks );

    gLoopbackArgs.bLoopbackEnabled = 0;

} // StopFrameLoopback

/****************************************************************************
*
*  loopbackGetFrameBuf
*
*   Allocates a buffer for packets received from VC, to be looped-back to VC.
*
***************************************************************************/
static VC_FrameBuffer_t* loopbackGetFrameBuf
(
   unsigned int streamNum,
   int lengthInBytes
)
{
    VC_FrameBuffer_t *frameBuf;

    (void) streamNum;

    if ( lengthInBytes > VC_FFMAX_CMD_DATA )
    {
        printk( KERN_ERR "loopbackGetFrameBuf: packet too big (%d)\n", lengthInBytes );
        return ( NULL );
    }

    frameBuf = VC_DequeueFrameBuffer( &gLoopbackFreeFramesList );
    if ( frameBuf == NULL )
    {
        printk( KERN_ERR "loopbackGetFrameBuf allocation failed\n" );
    }

    return ( frameBuf );
}


/****************************************************************************
*
*  loopbackEnqueueFrameBuf
*
*     Enqueue the packet to the received-from-VC list, and then send it
*     back to VC.
*
***************************************************************************/
static void loopbackEnqueueFrameBuf
(
   VC_FrameBuffer_t *frame,
   unsigned int      lengthInBytes,
   int               streamNum
)
{
    // This was already validated when the buffer was allocated.
    (void) lengthInBytes;

    frame->hdr.frame.timestamp += gVcLoopbackDelay;
    frame->hdr.frame.stream_num = gLoopbackArgs.decodeStream;

    // Enqueue the packet to the received-from-VC list, and then send it
    // back to VC.
    VC_EnqueueFrameBuffer( &gLoopbackFramesFromVCList, frame );
    vc_sendframe( streamNum );
}


/****************************************************************************
*
*  loopbackDequeueFrameBuf
*
*     Dequeue packet from the received-from-VC list.
*
***************************************************************************/
static VC_FrameBuffer_t* loopbackDequeueFrameBuf( int streamNum )
{
   return ( VC_DequeueFrameBuffer( &gLoopbackFramesFromVCList ) );
}


/****************************************************************************
*
*  loopbackFreeFrameBuf
*
*     Release packet back to free buffer pool.
*
***************************************************************************/
static void loopbackFreeFrameBuf( VC_FrameBuffer_t *frame )
{
   VC_EnqueueFrameBuffer( &gLoopbackFreeFramesList, frame );

}


/****************************************************************************/
EXPORT_SYMBOL(vc_host_set_cmd_callback);
EXPORT_SYMBOL(vc_gencmd);
EXPORT_SYMBOL(vc_gencmd_number_property);
EXPORT_SYMBOL(vc_host_set_clock);
EXPORT_SYMBOL(vc_host_get_clock);
EXPORT_SYMBOL(vc_host_read_VC02_STC_reg);
EXPORT_SYMBOL(vc_sendframe);

module_init(vc_init_drv);
module_exit(vc_exit_drv);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("VCO2 Driver");
