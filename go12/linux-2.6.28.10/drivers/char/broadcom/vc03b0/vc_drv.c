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




/*******************************************************************************
*
*  vc_drv.c
*
*  PURPOSE:
*
*     This implements the driver for the VCO3.
*
*  NOTES:
*
*****************************************************************************/

#define CAMERA_PREVIEW_RENDERER 1

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
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/videocore_settings.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/gpio_irq.h>
#if 0
#include <asm/arch/reg_gpio.h>
#include <asm/arch/reg_irq.h>
#endif
#include <linux/broadcom/gpio.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#if 0
#include <asm/arch/reg_lcd.h>
#include <asm/arch/reg_dma.h>
#include <asm/arch/dma.h>
#endif

#include <linux/broadcom/debug_pause.h>

#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vcilcs.h>
#include <linux/broadcom/vc03/vchost.h>
#include <linux/broadcom/vc03/vchost_int.h>
#include <linux/broadcom/vc03/vchostreq_int.h>
#include <linux/broadcom/vc03/vcinterface.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vc_vchi_gencmd.h>
#include <linux/broadcom/vc03/vc_vchi_dispmanx.h>
#include <linux/broadcom/vc03/vc_vchi_filesys.h>
//#include <linux/broadcom/vc03/vc_vchi_logserver.h>
#include <linux/broadcom/vc03/vc_dispmanx.h>
#include <linux/broadcom/vc03/vcmsgfifo.h>
#include <linux/broadcom/vc03/vchostreq.h>
#include <linux/broadcom/vc03/vc03b0_hostport.h>
#include <linux/broadcom/omx/OMX_Core.h>
#include <linux/broadcom/vc03/vc_ilcs_defs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vchi/mphi.h>
#include <linux/broadcom/vc03/vchi/vchi.h> // vchi_msg_queue() etc.
#include <linux/broadcom/vc03/vc03_frmfwd_defs.h>
#include "vc_drv.h"
#include "vc_omxencdec.h"

#define VMCSPATH_MAX 255
#define VMCSPATH_DEFAULT "/vc03/vcvmcs.bin"

#define MAX_MSG_BUF_SIZE (4 * 1024 * 1024)	// reasonableness limit: 4 MB

//#define LOG_VCIO_STATS		// Log largest msg size for queue, dequeue, bulk transmit, bulk receive

//#define VCHI_TEST 

/*
 * Resources to be released for this file connection.
 */
typedef struct
{
    VC_msg_buffer_t	*gfxBufferList;		// List of kernel graphics buffers allocated.

} VC_FileData_t;


/* ---- Private Variables ------------------------------------------------ */


// VC_IOCTL_INIT_SERVICE returns one of these, based on the requested service.
extern VCHI_SERVICE_HANDLE_T khrn_service_handle;
extern VCHI_SERVICE_HANDLE_T disp_service_handle;
extern VCHI_SERVICE_HANDLE_T updh_service_handle;

extern OS_COUNT_SEMAPHORE_T khronos_msg_avail;
extern OS_COUNT_SEMAPHORE_T dispmanx_message_available_semaphore;
extern OS_COUNT_SEMAPHORE_T dispmanx_notify_available_semaphore;


// Need these two temporarily, until kernel buffering is fully functional and buffer packing is fixed.
#define DMODE_USER_SPACE_BUFFER	0x01	// Allocate buffer in user space instead of kernel space.
#define DMODE_USER_NOPACK	0x02	// Don't pack multiple command lists in kernel buffer.

#define DMODE_KERNEL_DEBUG1	0x10
#define DMODE_KERNEL_DEBUG2	0x20
#define DMODE_KERNEL_DEBUG4	0x40
#define DMODE_KERNEL_DEBUG8	0x80
#define DMODE_KERNEL_DEBUG16	0x100

// Debugging flags for: use kernel buffer, no buffer packing, slow mode, debug all VCHI output (but not input).
//static unsigned int gfx_debug_mode = DMODE_USER_NOPACK | DMODE_KERNEL_DEBUG1 | DMODE_KERNEL_DEBUG2 | DMODE_KERNEL_DEBUG4;

// Debugging flags for: use kernel buffer, no buffer packing, slow mode, no VCHI debug printks.
//static unsigned int gfx_debug_mode = DMODE_USER_NOPACK | DMODE_KERNEL_DEBUG1;

// Debugging flags for: Production version: use kernel buffer, full buffer packing, no VCHI debug printks.
static unsigned int gfx_debug_mode = 0;
                                                                                                                                         

static  char banner[] __initdata = KERN_INFO "VC03_b0 Driver: 0.01\n";
#define VC03DRV_MAX 1

vc03drv_t vc03drv[VC03DRV_MAX] =
  {
    {
      .initialized = 0,
    },
  };

#define VC_DEBUG_ENABLED 1

#if VC_DEBUG_ENABLED
int gVcDebugInfo           = 1;
int gVcDebugHostIrq        = 0;
int gVcDebugVerbose        = 0;
int gVcDebugDisplayRegion  = 0;
int gVcDebugTrace          = 0;
int gVcDebugMsgFifo        = 0;
int gVcDebugMphiTrace      = 0;
int gVcDebugMphiDumpMsg    = 0;
int gVcDebugSingleTrace    = 0;
int gVcDebugVchiTrace      = 0;
int gVcDebugVchiTraceData  = 0;
int gVcDebugVchiTraceDataSize  = 64;
int gVcDebugIlcsTrace      = 0;
int gVcDebugRwPerf         = 0;
int gVcLogServer           = 0;
int gVcSingleState         = 0;
int gVcDebugBootTrace      = 0;
int gVcDebugIrqTrace       = 0;
int gVcDebugMemAllocTrace  = 0;
int gVcDebugPifReadMsgBlock = 1;
int gVcDebugAlignDma        = 1;
int gVcDebugServiceInfo     = 1;
#endif
int gVcDmaDisabled = 1;
int gVcDmaMinSize = 200;
int gVcGmtOffset  = 0;

// On the MIPS, we can't take advantage of the kmalloc'd memory for DMA, and 
// since the MIPS platforms have limited memory, they can't see to allocate the larger
// buffers. So we default to using vmalloc under mips, and kmalloc under ARM.

typedef enum
{
    MEM_ALLOC_TYPE_VMALLOC  = 0,
    MEM_ALLOC_TYPE_KMALLOC  = 1,

} MEM_AllocType_t;

#if defined( CONFIG_MIPS )
#define DEFAULT_ALLOC   MEM_ALLOC_TYPE_VMALLOC
#else
#define DEFAULT_ALLOC   MEM_ALLOC_TYPE_KMALLOC
#endif

int gVcOsKmalloc            = DEFAULT_ALLOC;
int gVcAllocBufferKmalloc   = DEFAULT_ALLOC;
int gVcMallocKmalloc        = DEFAULT_ALLOC;
int gVcMsgQueueKmalloc      = DEFAULT_ALLOC;
int gVcBulkQueueKmalloc     = DEFAULT_ALLOC;

#if 0
void logserver_send_test_msg( int msgNum );

static int vc_logserver_proc( ctl_table *table, int write, struct file *filp,
                               void __user *buffer, size_t *lenp, loff_t *ppos )
{
    int rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

    if ( write )
    {
        logserver_send_test_msg( gVcLogServer );
    }

    return rc;
}
#endif

void single_state_debug( void );

static int vc_single_state_proc( ctl_table *table, int write, struct file *filp,
                                 void __user *buffer, size_t *lenp, loff_t *ppos )
{
    int rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

    if ( write )
    {
        single_state_debug();
    }

    return rc;
}

extern int queueDepth;
extern int maxQueueDepth;

#define BCM_SYSCTL_VC_NODE(NUM, NAME, DATA) {            \
   .ctl_name      = NUM,                                 \
   .procname      = NAME,                                \
   .data          = &DATA,                               \
   .maxlen        = sizeof( int ),                       \
   .mode          = 0644,                                \
   .proc_handler  = &proc_dointvec                       \
}

#if VC_DEBUG_ENABLED
static struct ctl_table gSysCtlVcDebug[] = 
{
    BCM_SYSCTL_VC_NODE(101, "vchi-trace",       gVcDebugVchiTrace),
    BCM_SYSCTL_VC_NODE(102, "trace",            gVcDebugTrace),
    BCM_SYSCTL_VC_NODE(103, "info",             gVcDebugInfo),
    BCM_SYSCTL_VC_NODE(104, "host-irq",         gVcDebugHostIrq),
    BCM_SYSCTL_VC_NODE(105, "verbose",          gVcDebugVerbose),
    BCM_SYSCTL_VC_NODE(106, "rw-perf",          gVcDebugRwPerf),
    BCM_SYSCTL_VC_NODE(107, "vchi-trace-data",  gVcDebugVchiTraceData),
    BCM_SYSCTL_VC_NODE(108, "queueDepth",       queueDepth),
    BCM_SYSCTL_VC_NODE(109, "maxQueueDepth",    maxQueueDepth),
    BCM_SYSCTL_VC_NODE(110, "ilcs-trace",       gVcDebugIlcsTrace),
    BCM_SYSCTL_VC_NODE(111, "boot-trace",       gVcDebugBootTrace),
    BCM_SYSCTL_VC_NODE(111, "irq-trace",        gVcDebugIrqTrace),
    BCM_SYSCTL_VC_NODE(112, "dma-disabled",     gVcDmaDisabled),
    BCM_SYSCTL_VC_NODE(113, "dma-min-size",     gVcDmaMinSize),
    BCM_SYSCTL_VC_NODE(114, "vcos-kmalloc",     gVcOsKmalloc),
    BCM_SYSCTL_VC_NODE(115, "alloc-buffer-kmalloc", gVcAllocBufferKmalloc),
    BCM_SYSCTL_VC_NODE(116, "malloc-kmalloc",       gVcMallocKmalloc),
    BCM_SYSCTL_VC_NODE(117, "msg-queue-kmalloc",    gVcMsgQueueKmalloc),
    BCM_SYSCTL_VC_NODE(118, "bulk-queue-kmalloc",    gVcBulkQueueKmalloc),
    BCM_SYSCTL_VC_NODE(119, "mphi-dump-msg",    gVcDebugMphiDumpMsg),
    BCM_SYSCTL_VC_NODE(120, "mphi-trace",       gVcDebugMphiTrace),
    BCM_SYSCTL_VC_NODE(121, "mem-alloc-trace",  gVcDebugMemAllocTrace),
    BCM_SYSCTL_VC_NODE(122, "vchi-trace-data-size",  gVcDebugVchiTraceDataSize),
    BCM_SYSCTL_VC_NODE(123, "pif-read-msg-block",  gVcDebugPifReadMsgBlock),
    BCM_SYSCTL_VC_NODE(124, "dma-align",        gVcDebugAlignDma),
    BCM_SYSCTL_VC_NODE(125, "service-info",     gVcDebugServiceInfo),
    BCM_SYSCTL_VC_NODE(126, "single-trace",     gVcDebugSingleTrace),
#if 0
    {
       .ctl_name         = 200,
       .procname         = "logserver",
       .data             = &gVcLogServer,
       .maxlen           = sizeof( int ),
       .mode             = 0644,
       .proc_handler     = &vc_logserver_proc
    },
#endif
    {
        .ctl_name       = 210,
        .procname       = "single-state",
        .data           = &gVcSingleState,
        .maxlen         = sizeof( int ),
        .mode           = 0644,
        .proc_handler   = &vc_single_state_proc,
    },
    {}
};
#endif

static struct ctl_table gSysCtlVc[] = 
{
#if VC_DEBUG_ENABLED
   {
      .ctl_name      = 1,
      .procname      = "debug",
      .mode          = 0555,
      .child         = gSysCtlVcDebug
   },
#endif
   {}
};

static struct ctl_table gSysCtl[] = 
{
   {
      .ctl_name      = CTL_BCM_VC,
      .procname      = "vc",
      .mode          = 0555,
      .child         = gSysCtlVc
   },
   {}
};

static  struct ctl_table_header    *gSysCtlHeader = NULL;
static  struct proc_dir_entry      *gProcVcDir;
static  struct proc_dir_entry      *gProcVcDebugDir;

extern int vcstage2_size;
extern unsigned char vcstage2_bin[];
//extern int vcvmcs_size;
//extern unsigned char vcvmcs_bin[];
typedef struct
{
  uint32_t header;
  uint32_t id_size;
  uint32_t signature[5];
  uint32_t footer;
  
} boot_message2_t;
//char vcvmcs_bin[1];
//int vcvmcs_size = 0;

typedef struct VCHI_SERVICE_HDLS
{
   void * gencmd_handle;

} VCHI_SERVICE_HDLS;

static VCHI_SERVICE_HDLS gVideoCoreVCHIHdls[2];

/* ---- External function prototypes ------------------------------------- */

int            vc_ilcsinb_getusr(vc_ilcsinb_t* msgptr);
int            vc_ilcsinb_sendresp(vc_ilcsinb_t* msgptr);
void           lcd_colorbars(int num);
int            lcd_init_2( void );
int            omxlog_read(char* dest, int maxbytes);
VC_RESP_CODE_T vchostreq_writemem_usr(void* host_addr, void *vc_addr, int len, int channel);


/* ---- Private Function Prototypes -------------------------------------- */

static  int     vc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static  int     vc_open( struct inode *inode, struct file *file );
static  ssize_t vc_read( struct file *file, char *buffer, size_t count, loff_t *ppos );
static  int     vc_release( struct inode *inode, struct file *file );
static  ssize_t vc_write( struct file *file, const char *buffer, size_t count, loff_t *ppos );
extern  int     vcil_iface_init(void);
static  int     vc03_reset_bootmem(int delay);
static  int     vc03_reset_bootfile(int delay, const char* vcvmcs_path);
static  int     vchost_sendfile(const char* vcvmcs_path);
static  int     vc03_exit(vc03drv_t* drv);

#if USE_DMA   
struct semaphore gDmaSem;
#endif   

#if (CFG_GLOBAL_CHIP == BCM2153)
extern void start_gptimer(void);
#endif


#define VC03_VMCSLOG_MAX (32*1024)
static uint32_t vmcslog_rindex,vmcslog_windex;
static char *vmcslog;
uint32_t vmcslog_count;
char logdata[VMCSLOG_ENTRY_LENGTH];

struct semaphore vmcslog_lock;

int vmcslog_add(char* data, int len)
{
  uint32_t str_len, nbytes;

  if(down_interruptible(&vmcslog_lock))
	return -1;
  
  str_len = len;

  nbytes = min(VC03_VMCSLOG_MAX - vmcslog_windex, str_len);
  memcpy(vmcslog + vmcslog_windex, data, nbytes);

  if(vmcslog_windex < vmcslog_rindex)
	{
	  vmcslog_windex += nbytes;
	  if(vmcslog_windex >= (vmcslog_rindex - 1))		
		{
		  vmcslog_rindex = (vmcslog_windex + 1) % VC03_VMCSLOG_MAX;
		}
	}
  else
	{	  
	  vmcslog_windex += nbytes;
	}
  
  if(nbytes < str_len)
	{
	  vc_assert(vmcslog_windex >= VC03_VMCSLOG_MAX);
	  memcpy(vmcslog + 0, data + nbytes, str_len - nbytes);
	  vmcslog_windex = str_len - nbytes;
	  vmcslog_rindex = max(vmcslog_rindex, vmcslog_windex + 1);
	}
  
  up(&vmcslog_lock);
  return 0;



}


int vmcslog_read(char* dest, int maxbytes)
{
  char* src;
  int bytecount;
  
  if(down_interruptible(&vmcslog_lock))
	return 0;

  // calc offset
  src = vmcslog + vmcslog_rindex;
  if(vmcslog_rindex  <= vmcslog_windex)
	{
	  bytecount = vmcslog_windex - vmcslog_rindex - 1;
	}
  else
	{
	  bytecount = VC03_VMCSLOG_MAX - vmcslog_rindex;
	}

  // copy
  bytecount = min(bytecount, maxbytes);
  memcpy(dest, vmcslog + vmcslog_rindex, bytecount);

  // advance read index
  vmcslog_rindex += bytecount;
  if( VC03_VMCSLOG_MAX == vmcslog_rindex)
	{
	  vmcslog_rindex = 0;
	}

  //
  up(&vmcslog_lock);
  return bytecount;
}

/*
*
*
*
*/
int vc3if_proc_vmcslog_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;
  int ret;

  *start = buf;// + offset;
  while(len < count)
	{
	  ret = vmcslog_read(buf + len /*+ offset*/, count - len);
	  if(ret < 1)
		{
		  break;
		}
	  len += ret;
	}

  /// buffer is empty
  if(ret == 0  && len > 0)
	{
	  len += sprintf(buf + len, "\n");
	}

  if(len == 0)
	{
	  *eof = 1;
	}
  return len;
}



/*
 *
 */
inline vc03drv_t* vc03drv_get(void)
{
  return vc03drv + 0; // FIXME, use minor num
}


#if ( CFG_GLOBAL_GFX_SERVICE_SUPPORT == 1 )
OS_COUNT_SEMAPHORE_T *getSemaphoreFromServiceHandler(VCHI_SERVICE_HANDLE_T hService)
{
	OS_COUNT_SEMAPHORE_T *pSem = 0;

	if (hService == khrn_service_handle)
	{
		pSem = &khronos_msg_avail;
	}
	else
    if (hService == disp_service_handle)
	{
		pSem = &dispmanx_message_available_semaphore;
	}
	else if (hService == updh_service_handle)
	{
		pSem = &dispmanx_notify_available_semaphore;
	}

	return pSem;
}


// Don't know what the limit is, but 65536 is definitely too big.
#define MAX_COPY_FROM_USER (16 * 1024)	// This works for 2.6 kernels.

/*
 * used to perform the transfer of a LARGE amount of data from user mode
 */
int blockCopyFromUserBuffer(void *pDst, const void *pSrc, uint32_t len)
{
	if (0 == access_ok(VERIFY_READ, pSrc, len))
	{
	    printk(KERN_ERR "%s failed to access user memory at addr 0x%08lx, size %d\n",
						__func__, (unsigned long)pSrc, len); 
	    return -EFAULT;
	}

	do 
	{
		uint32_t blockSize = (len < MAX_COPY_FROM_USER) ? len : MAX_COPY_FROM_USER;
		//printk(KERN_ERR "copy ubuf 0x%08x\n",pSrc);
		__copy_from_user(pDst, pSrc, blockSize);
		pSrc += blockSize;
		pDst += blockSize;
		len -= blockSize;
	}
	while (len > 0);

	return 0;
}
#endif // CFG_GLOBAL_GFX_SERVICE_SUPPORT

static void *mem_alloc( MEM_AllocType_t allocType, size_t numBytes )
{
    if ( allocType == MEM_ALLOC_TYPE_KMALLOC )
    {
        return kmalloc( numBytes, GFP_KERNEL );
    }

    return vmalloc( numBytes );
}

static void mem_free( void *mem )
{
    if ((( (unsigned long)mem >= VMALLOC_START ) ) && ( (unsigned long)mem < VMALLOC_END ))
    {
        vfree( mem );
    }
    else
    {
        kfree( mem );
    }
}

/****************************************************************************
*
*  vc_ioctl
*
***************************************************************************/
int vc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
  int err;

  //    VC_DEBUG( Trace, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));
  
  if (( _IOC_TYPE( cmd ) != VC_MAGIC )
      ||  ( _IOC_NR( cmd ) < VC_CMD_FIRST )
      ||  ( _IOC_NR( cmd ) > VC_CMD_LAST ))
    {
      VC_DEBUG(Trace, "invalid cmd: cmd=0x%04X f=0x%04X last=0x%04X type=%c\n",
                  cmd, VC_CMD_FIRST,VC_CMD_LAST, _IOC_TYPE(cmd));
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
      printk( KERN_ERR "vc_ioctl: arg pointer is invalid: cmd=0x%08X arg=0x%08X 0x%08X 0x%08X\n", (int)cmd, (int)arg, (int)_IOC_SIZE(cmd), (int)_IOC_DIR(cmd));
      return -EFAULT;
    }
  
  switch ( cmd )
    {
      
    case VC_IOCTL_ILCS_EXECUTE_FUNC:
      {		  
	err = vc_ilcs_execute_function_usr((void*)arg);
	return err;		  
      }
      
    case VC_IOCTL_ILCS_COMPONENT_LOCK:
      {
	if(0 == vc_ilcs_componentlock_change((int)arg))
	  return 0;
	else
	  return -EFAULT;			
      }

    case VC_IOCTL_HOSTREQ_WRITEMEM:
      {
	vc_hostreq_memaccess desc;
	if(copy_from_user(&desc, (void *)arg, sizeof(desc)) != 0 )
	  {
	    return -EFAULT;
	  }

	//vchostreq_writemem_usr(desc.host_addr, desc.vc_addr, desc.len, desc.channel);
	return 0;
      }
      
    case VC_IOCTL_ILCSINB:
      {
	if(0 != vc_ilcsinb_getusr((vc_ilcsinb_t*)arg))
	  {
	    return -EFAULT;
	  }
	
	return 0;
      }

     case VC_IOCTL_ILCSINBRESP:
     {
	    //printk( "  Before  calling VC_IOCTL_ILCSINBRESP  \n" );

		if(0 != vc_ilcsinb_sendresp((vc_ilcsinb_t*)arg))
	    {
	    	return -EFAULT;
	    }

	    //printk( " After calling VC_IOCTL_ILCSINBRESP  \n" );
            return 0;
     }
      
    case VC_IOCTL_MALLOC:
    {
	vc_malloc_t param;
	
	if(copy_from_user(&param, (void *)arg, sizeof(param)) != 0 )
	  {
	    return -EFAULT;
	  }
	
	param.addr = 0;
    param.kaddr = mem_alloc( gVcMallocKmalloc, param.len );
	
	if(NULL == param.kaddr)
	  {
	    return -EFAULT;
	  }
	
	if(copy_to_user((void *)arg, &param, sizeof(param)) != 0)
	  {
	    return -EFAULT;
	  }
	
	return 0;
    }
      
    case VC_IOCTL_FREE:
    {
	if(0 == arg)
	  {
	    return -EFAULT;
	  }
         
	mem_free((void*)arg);
	return 0;
    }


#if ( CFG_GLOBAL_GFX_SERVICE_SUPPORT == 1 )

    // Return the handle for a VC3 service
    //
    // on entry, 'arg' points to a word containg the name of the service.
    // on exit, that same word in user space is updated to be the handle for that service.
    //
    case VC_IOCTL_INIT_SERVICE:
    {
	uint32_t service_name;
    VCHI_SERVICE_HANDLE_T service_handle = 0;

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG16)
	{
		printk("VC_IOCTL_INIT_SERVICE: arg address is 0x%x\n", (uint32_t)arg);
	}
	
	get_user(service_name, (uint32_t *)arg);

	if (service_name == MAKE_FOURCC("KHRN"))
	{
		service_handle = khrn_service_handle;
	} 
	else if (service_name == DISPMANX_CLIENT_NAME)
	{
		service_handle = disp_service_handle;
	}
	else if (service_name == DISPMANX_NOTIFY_NAME)
	{
		service_handle = updh_service_handle;
	}
	else
	{
	    printk(KERN_ERR "Invalid service ID =  0x%08x\n", (uint32_t)arg);
	    return -EFAULT;
	}

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG16)
	{
		printk("VC_IOCTL_INIT_SERVICE: requested service is 0x%08x, handle is 0x%08lx\n",
								service_name, (unsigned long)service_handle);
	}

	__put_user(service_handle, (VCHI_SERVICE_HANDLE_T *)arg);

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG16)
	{
		printk("VC_IOCTL_INIT_SERVICE: returned 0\n");
	}
	return 0;
    } // case VC_IOCTL_INIT_SERVICE:


    // Enqueue a message to the VC3
    case VC_IOCTL_MSG_QUEUE:
    {
	msg_queue_t msg_queue_desc;
	uint32_t    *srcptr, *buffer, *dstptr;
	uint32_t    bufsize, copysize;
#ifdef LOG_VCIO_STATS
	static uint32_t largest_queue = 0;	// initial value
#endif // LOG_VCIO_STATS
	
	if (copy_from_user(&msg_queue_desc, (void *)arg, sizeof(msg_queue_desc)) != 0 )
	{
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUE failed to access msg_queue desc at addr 0x%08x\n", (int)arg);
	    return -EFAULT;
	}

	// Verify that kernel can access specified user space source data.
	if (0 == access_ok(VERIFY_READ, msg_queue_desc.data, msg_queue_desc.data_size))
	{
	    printk(KERN_ERR "VC_IOCTL_MSG_QUEUE failed to access user memory at addr 0x%08x, size %d\n",
						(uint32_t)msg_queue_desc.data, msg_queue_desc.data_size); 
	    return -EFAULT;
	}
	
	// VCHI interface code prohibits breaking up a transfer into multiple pieces, so we must allocate a buffer
	// for the full transfer length, and output the data in a single vchi call.
	bufsize = msg_queue_desc.data_size;
	buffer = kmalloc(bufsize, GFP_KERNEL);
	if (buffer == NULL)
	{
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUE unable to allocate msg_queue buffer, size=%d\n",
							msg_queue_desc.data_size);
	    return -EFAULT;
	}

#ifdef LOG_VCIO_STATS
	if (bufsize > largest_queue)
	{
	    largest_queue = bufsize;
	    printk(KERN_INFO " ===== Largest msg queue is now %d =====\n", bufsize);
	}
#endif // LOG_VCIO_STATS

	// But Linux limits the maximum length of a copy from/to user space.
	copysize = bufsize;
	if (copysize > MAX_COPY_FROM_USER)
		copysize = MAX_COPY_FROM_USER;

	srcptr = msg_queue_desc.data;
	dstptr = buffer;

	while (msg_queue_desc.data_size > 0)
	{
            // If last block, send only the bytes remaining.
            if (msg_queue_desc.data_size < copysize)
               copysize = msg_queue_desc.data_size;

	    // The call to access_ok() above has already checked validity of user space access, so we use "__copy_...()" here.
	    // Driver may sleep here while faulting in the user pages.
	    if (__copy_from_user(dstptr, srcptr, copysize) != 0)
	    {
                printk(KERN_ERR "VC_IOCTL_MSG_QUEUE failed to copy from user buffer at 0x%08x\n",
			      							(uint32_t)srcptr);
		kfree(buffer);
		return -EFAULT;
	    }

	    msg_queue_desc.data_size -= copysize;
	    srcptr += copysize;
	    dstptr += copysize;
	}

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
	{
	    int i;
	    uint32_t *word_ptr = (uint32_t *)buffer;

	    printk(KERN_INFO "VC_IOCTL_MSG_QUEUE: handle 0x%08x  addr 0x%08x  size %d  flags 0x%08x\n",
			(uint32_t)msg_queue_desc.handle, (uint32_t)buffer, copysize, msg_queue_desc.flags);

            for (i = 0; i < (copysize >> 2); i++)
            {
               printk(KERN_INFO " x%08x", word_ptr[i]);
               if ((i % 8) == 7) printk(KERN_INFO "\n");
            }
            if ((i % 8) != 0) printk(KERN_INFO"\n");		// i incremented again when falling out of loop
        }

	// Driver may sleep here while feeding the fifo.
	vchi_msg_queue(msg_queue_desc.handle, buffer, bufsize, /*msg_queue_desc.flags |*/ VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE, msg_queue_desc.msg_handle);

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG1)
	{
	   // Make sure that mphi_rxtx_task() has a chance to call mphi_pumptx() to start the transfer.
	   msleep(0);

           // Make sure all PIO or DMA output is finished before freeing the buffer.
           mphi_get_func_table()->finish_io(0);
	}
	kfree(buffer);
#if 1
        if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
	    printk("VC_IOCTL_MSG_QUEUE returning 0\n");
#endif
	return 0;
    } // case VC_IOCTL_MSG_QUEUE:


    // Enqueue a vector of messages to the VC3
    case VC_IOCTL_MSG_QUEUEV:
    {
	msg_queuev_t msg_queuev_desc;
	VCHI_MSG_VECTOR_T *pMsgVecLt;
        int ret;

	if (copy_from_user(&msg_queuev_desc, (void *)arg, sizeof(msg_queuev_desc)) != 0 )
	{
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUEV failed to access msg_queuev desc at addr 0x%08x\n", (int)arg);
	    return -EFAULT;
	}

	// VCHI interface code prohibits breaking up a transfer into multiple pieces, so we must allocate a buffer
	// for the full transfer length, and output the data in a single vchi call.
	pMsgVecLt = kmalloc(msg_queuev_desc.count * sizeof(VCHI_MSG_VECTOR_T), GFP_KERNEL);
	if (pMsgVecLt == NULL)
	{
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUEV unable to allocate msg vector list buffer, size=%d\n",
				msg_queuev_desc.count * sizeof(VCHI_MSG_VECTOR_T));

	    return -EFAULT;
	}

	ret = blockCopyFromUserBuffer(pMsgVecLt, msg_queuev_desc.vector, msg_queuev_desc.count * sizeof(VCHI_MSG_VECTOR_T));
	if (!ret)
	{
	    int msgIdx = 0;
	    uint32_t totalMsgSize = 0;
	    unsigned char *pMsgPool;

	    for (;msgIdx<msg_queuev_desc.count;msgIdx++)
	    {
		totalMsgSize += pMsgVecLt[msgIdx].vec_len;
	    }

        pMsgPool = mem_alloc( gVcMsgQueueKmalloc, totalMsgSize );
	    if (pMsgPool)
	    {
		unsigned char *pCurrMsgPool = pMsgPool;
		for (msgIdx=0;msgIdx<msg_queuev_desc.count;msgIdx++)
		{
		    ret = blockCopyFromUserBuffer(pCurrMsgPool, pMsgVecLt[msgIdx].vec_base, pMsgVecLt[msgIdx].vec_len);
		    if (ret) 
		    {
			printk(KERN_ERR "VC_IOCTL_MSG_QUEUEV fail at msg index = %d\n", msgIdx);
			break;
		    }
					
		    pMsgVecLt[msgIdx].vec_base = pCurrMsgPool;
		    pCurrMsgPool += pMsgVecLt[msgIdx].vec_len;
		}
               
		// Driver may sleep here while feeding the fifo.
		if (!ret)
		{
		    //int j;
		    vchi_msg_queuev(msg_queuev_desc.handle, pMsgVecLt, msg_queuev_desc.count,
			/*msg_queuev_desc.flags |*/ VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE, msg_queuev_desc.msg_handle);

#if 0
		    printk(KERN_ERR "queuev: handle = 0x%08x, count = %d, flags = 0x%08x, msg handle = 0x%08x\n", 
					msg_queuev_desc.handle, msg_queuev_desc.count, msg_queuev_desc.flags, msg_queuev_desc.msg_handle);

		    printk(KERN_ERR "Queuev\n");
		    for (j=0;j<totalMsgSize;j++)
		    {
			printk(KERN_ERR "%02x ", (unsigned int)(pMsgPool[j]));
		    }
		    printk(KERN_ERR "\n");

		    for (j=0;j<msg_queuev_desc.count;j++)
		    {
			printk(KERN_ERR "%08x ", pMsgVecLt[j].vec_base);
		    }

		    printk(KERN_ERR "\n");
#endif
		}

                if (gfx_debug_mode & DMODE_KERNEL_DEBUG1)
	        {
        	   // Make sure that mphi_rxtx_task() has a chance to call mphi_pumptx() to start the transfer.
        	   msleep(0);

                   // Make sure all PIO or DMA output is finished before freeing the buffer.
                   mphi_get_func_table()->finish_io(0);
        	}
		mem_free(pMsgPool);
	    }
	    else
	    {
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUEV unable to allocate msg pool buffer, size=%d\n", totalMsgSize);
				ret = -EFAULT;
	    }
	}

	kfree(pMsgVecLt);
	return ret;
    } // case VC_IOCTL_MSG_QUEUEV:


    // Dequeue a message from the VC3
    case VC_IOCTL_MSG_DEQUEUE:
    {
	msg_dequeue_t msg_dequeue_desc;
	uint32_t      *buffer;
	int32_t       err;
	uint32_t      bufsize, actual_size;
         OS_COUNT_SEMAPHORE_T *pSem;
#ifdef LOG_VCIO_STATS
	static uint32_t largest_dequeue;
#endif // LOG_VCIO_STATS
	
	if (copy_from_user(&msg_dequeue_desc, (void *)arg, sizeof(msg_dequeue_desc)) != 0 )
	{
	    printk(KERN_ERR "VC_IOCTL_MSG_DEQUEUE failed to access msg_dequeue desc at addr 0x%08x\n", (uint32_t)arg);
	    return -EFAULT;
	}

	// We presume that kmalloc returns a base address that is 16-byte aligned.
	// We enforce a 16-byte length multiple here, so as to eliminate redundant copying in the bowels of VCHI / MPHI.
	//
	bufsize = (msg_dequeue_desc.max_data_size_to_read + 0xf) & (~0xf);
	buffer = kmalloc(bufsize, GFP_KERNEL);
	if (buffer == NULL)
	{
	    printk(KERN_ERR "VC_IOCTL_MSG_DEQUEUE unable to allocate msg_dequeue buffer, size=%d\n",
		 						msg_dequeue_desc.max_data_size_to_read);
	    return -EFAULT;
	}

#ifdef LOG_VCIO_STATS
	if (bufsize > largest_dequeue)
	{
	    largest_dequeue = bufsize;
	    printk(KERN_INFO " ===== Largest msg dequeue is now %d =====\n", bufsize);
	}
#endif // LOG_VCIO_STATS

#if 1
         if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
	{
	    printk("VC_IOCTL_MSG_DEQUEUE: handle 0x%08x  data 0x%08x  max size %d\n",
		(uint32_t)msg_dequeue_desc.handle, (uint32_t)msg_dequeue_desc.data,
	       	msg_dequeue_desc.max_data_size_to_read);
	}
#endif
	// Wait for some input.

	pSem = getSemaphoreFromServiceHandler(msg_dequeue_desc.handle);

	// Driver may sleep here while waiting for data to be available.
        // This fails and returns nonzero if the app. receives a signal.
        if ( os_count_semaphore_obtain( pSem , 1 ))
           return -EINTR;
            
	// So make sure there really is a message before continuing.
	err = vchi_msg_dequeue(msg_dequeue_desc.handle, buffer, bufsize, &actual_size,
			/*msg_dequeue_desc.flags |*/ VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE);

	if (err)
	{
	    kfree(buffer);
	    printk(KERN_ERR "VC_IOCTL_MSG_DEQUEUE vchi_msg_dequeue failed, returned %d\n", err);
	    // This is probably the wrong thing to do.  Hopefully we never hit this case.
	    return -EFAULT;
	}
#if 0
	{

	unsigned char *pTmpBuf = (unsigned char *)buffer;
	int j;

	printk(KERN_ERR "Dequeue\n");
	for (j=0;j<actual_size;j++)
	{
		printk(KERN_ERR "%02x ", (unsigned int)(pTmpBuf[j]));
	}
	
	printk(KERN_ERR "\n");
	}
#endif
   
#if 1
         if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
	{
	    int i;
	    uint32_t *bufP = buffer;

	    printk(">>> vchi_msg_dequeue received %d bytes:", actual_size);
	    for (i = 0; i < actual_size; i += 4)
	    {
                if ((i % 32) == 16)
                    printk("\n");
                printk(" x%08x", *bufP++);
	    }
            if ((i % 32) != 16) printk("\n");
	}
#endif
         
	// Driver may sleep here while faulting in the user pages.
	if (copy_to_user(msg_dequeue_desc.data, buffer, actual_size) != 0)
	{
            printk(KERN_ERR "VC_IOCTL_MSG_DEQUEUE failed to copy %d bytes to user buffer at 0x%08X\n",
					actual_size, (uint32_t)msg_dequeue_desc.data);
	    kfree(buffer);
	    return -EFAULT;
	}

	// Store the actual size of the transfer, and copy it to user space.
	msg_dequeue_desc.actual_msg_size = actual_size;
	__copy_to_user((void *)arg, &msg_dequeue_desc, sizeof(msg_dequeue_desc));

	kfree(buffer);
#if 1
         if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
	    printk("VC_IOCTL_MSG_DEQUEUE returning 0\n");
#endif
	return 0;
    } // case VC_IOCTL_MSG_DEQUEUE:


    // Transmit bulk data to the VC3
    case VC_IOCTL_BULK_QUEUE_TRANSMIT:
    {
	bulk_queue_transmit_t transmit_desc;
	void                  *buffer, *user_src, *knl_dst;
	uint32_t              bufsize, data_size, copysize;
#ifdef LOG_VCIO_STATS
	static uint32_t largest_transmit = 0;
#endif // LOG_VCIO_STATS
	
	if (copy_from_user(&transmit_desc, (void *)arg, sizeof(transmit_desc)) != 0 )
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_TRANSMIT failed to access I/O descriptor at addr 0x%08x\n",
			   								 (int)arg);
	    return -EFAULT;
	}

	// Verify that kernel can access specified user space source data.
	if (0 == access_ok(VERIFY_READ, transmit_desc.data_src, transmit_desc.data_size))
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_TRANSMIT failed to access user memory at addr 0x%08x, size %d\n",
							(uint32_t)transmit_desc.data_src, transmit_desc.data_size); 
	    return -EFAULT;
	}
	
#if 1
         if (gfx_debug_mode & DMODE_KERNEL_DEBUG4)
         {
	    printk("VC_IOCTL_BULK_QUEUE_TRANSMIT: handle 0x%08x  data_src 0x%08x  data_size %d  flags 0x%08x\n",
						(uint32_t)transmit_desc.handle, (uint32_t)transmit_desc.data_src,
						transmit_desc.data_size, (uint32_t)transmit_desc.flags);
         }            
#endif

	// VCHI interface code prohibits breaking up a transfer into multiple pieces, so we must allocate a buffer
	// for the full transfer length, and output the data in a single vchi call.
	//
	// We presume that vmalloc returns a base address that is 16-byte aligned.
	// We enforce a 16-byte length multiple here, to eliminate redundant copying in the bowels of VCHI / MPHI.
	bufsize = transmit_desc.data_size;
    buffer = mem_alloc( gVcBulkQueueKmalloc, bufsize );
	if (buffer == NULL)
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_TRANSMIT unable to allocate vchi_bulk_queue_transmit buffer, size=%d\n",
									transmit_desc.data_size);
	    return -EFAULT;
	}

#ifdef LOG_VCIO_STATS
	if (bufsize > largest_transmit)
	{
	    largest_transmit = bufsize;
	    printk(KERN_INFO " ===== Largest bulk transmit is now %d =====\n", bufsize);
	}
#endif // LOG_VCIO_STATS

	user_src = transmit_desc.data_src;
	knl_dst = buffer;
	data_size = transmit_desc.data_size;

	copysize = transmit_desc.data_size;
	if (copysize > MAX_COPY_FROM_USER)
		copysize = MAX_COPY_FROM_USER;

	while (data_size > 0)
	{
            // If last block, send only the bytes remaining.
            if (data_size < copysize)
               copysize = data_size;
            
#if 1
            if (gfx_debug_mode & DMODE_KERNEL_DEBUG4)
		printk("VC_IOCTL_BULK_QUEUE_TRANSMIT __copy_from_user addr 0x%08x to knl buffer 0x%08x nb %d\n",
							(uint32_t)user_src, (uint32_t)knl_dst, copysize);
#endif
	    // Driver may sleep here while faulting in the user pages.
	    if (__copy_from_user(knl_dst, user_src, copysize) != 0)
	    {
		printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_TRANSMIT failed to copy from user buffer at 0x%08x  size %d\n",
										(uint32_t)user_src, copysize);
		mem_free(buffer);
		return -EFAULT;
	    }

            if (gfx_debug_mode & DMODE_KERNEL_DEBUG4)
	    {
		printk(KERN_INFO "VC_IOCTL_BULK_QUEUE_TRANSMIT: handle 0x%08x  addr 0x%08x  size %d  flags 0x%08x\n",
					(uint32_t)transmit_desc.handle, (uint32_t)buffer, copysize, transmit_desc.flags);
	    }

	    user_src += copysize;
	    knl_dst += copysize;
	    data_size -= copysize;
	}

	// Driver may sleep here while feeding the fifo.
	// We enforce a 16-byte length multiple here, to eliminate redundant copying in the bowels of VCHI / MPHI.
	vchi_bulk_queue_transmit(transmit_desc.handle, buffer, bufsize,
			/*transmit_desc.flags |*/ VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE, transmit_desc.bulk_handle);

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG1)
	{
	   // Make sure that mphi_rxtx_task() has a chance to call mphi_pumptx() to start the transfer.
	   msleep(0);

           // Make sure all PIO or DMA output is finished before freeing the buffer.
           mphi_get_func_table()->finish_io(0);
	}
	mem_free(buffer);
#if 1
        if (gfx_debug_mode & DMODE_KERNEL_DEBUG4)
	    printk("VC_IOCTL_BULK_QUEUE_TRANSMIT returning 0\n");
#endif
	return 0;
    } // case VC_IOCTL_BULK_QUEUE_TRANSMIT:


    // Receive bulk data from VC3
    case VC_IOCTL_BULK_QUEUE_RECEIVE:
    {
	bulk_queue_receive_t receive_desc;
	void                 *buffer, *knl_src, *user_dst;
	uint32_t             bufsize, data_size;
#ifdef LOG_VCIO_STATS
	static uint32_t largest_receive = 0;
#endif // LOG_VCIO_STATS
	
	if (copy_from_user(&receive_desc, (void *)arg, sizeof(receive_desc)) != 0 )
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_RECEIVE failed to access I/O descriptor at addr 0x%08x\n",
												(int)arg);
	    return -EFAULT;
	}

	// Verify that kernel can access specified user space destination buffer.
	if (0 == access_ok(VERIFY_WRITE, receive_desc.data_dst, receive_desc.data_size))
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_RECEIVE failed to access user memory at addr 0x%08x, size %d\n",
							(uint32_t)receive_desc.data_dst, receive_desc.data_size); 
	    return -EFAULT;
	}
	
#if 1
         if (gfx_debug_mode & DMODE_KERNEL_DEBUG8)
         {
	    printk("VC_IOCTL_BULK_QUEUE_RECEIVE: handle 0x%08x  data_dst 0x%08x  data_size %d  flags 0x%08x\n",
						(uint32_t)receive_desc.handle, (uint32_t)receive_desc.data_dst,
						receive_desc.data_size, (uint32_t)receive_desc.flags);
         }
#endif

	// VCHI interface code prohibits breaking up a transfer into multiple pieces, so we must allocate a
	// buffer big enough for the full transfer length, and transfer the data in a single vchi call.
	//
	// We presume that kmalloc returns a base address that is 16-byte aligned.
	// We enforce a 16-byte length multiple here, so as to eliminate redundant copying in the bowels of VCHI / MPHI.
	bufsize = receive_desc.data_size;
    buffer = mem_alloc( gVcBulkQueueKmalloc, bufsize );
	if (buffer == NULL)
	{
	    printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_RECEIVE unable to allocate buffer, size=%d\n",
									receive_desc.data_size);
	    return -EFAULT;
	}

#ifdef LOG_VCIO_STATS
	if (bufsize > largest_receive)
	{
	    largest_receive = bufsize;
	    printk(KERN_INFO " ===== Largest bulk receive is now %d =====\n", bufsize);
	}
#endif // LOG_VCIO_STATS

	knl_src = buffer;
	user_dst = receive_desc.data_dst;
	data_size = receive_desc.data_size;

#if 1
            if (gfx_debug_mode & DMODE_KERNEL_DEBUG8)
		printk("VC_IOCTL_BULK_QUEUE_RECEIVE __copy_to_user addr 0x%08x from knl buffer 0x%08x nb %d\n",
							(uint32_t)user_dst, (uint32_t)buffer, bufsize);
#endif

	// Driver may sleep here while receiving data.
	vchi_bulk_queue_receive(receive_desc.handle, buffer, bufsize,
			/*receive_desc.flags |*/ VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE, receive_desc.bulk_handle);

	// Reset the expected input size back to the size that we used in mapping
	// the user buffer in the above call to access_ok().
	bufsize = receive_desc.data_size;

	// Limit the max transfer to user space to MAX_COPY_TO_USER.
	while (data_size > 0)
	{
	    // Driver may sleep here while faulting in the user pages.
	    if (0 != __copy_to_user(user_dst, knl_src, bufsize))
	    {
		printk(KERN_ERR "VC_IOCTL_BULK_QUEUE_RECEIVE failed to copy to user buffer at 0x%08x  size %d\n",
											(uint32_t)user_dst, bufsize);
		mem_free(buffer);
		return -EFAULT;
	    }

            if (gfx_debug_mode & DMODE_KERNEL_DEBUG8)
	    {
		printk(KERN_INFO "VC_IOCTL_BULK_QUEUE_RECEIVE: handle 0x%08x  addr 0x%08x  size %d  flags 0x%08x\n",
					(uint32_t)receive_desc.handle, (uint32_t)buffer, bufsize, receive_desc.flags);
	    }

	    knl_src += bufsize;
	    user_dst += bufsize;
	    data_size -= bufsize;

	    // Last block: read only the bytes remaining.
	    if (data_size < bufsize)
       		bufsize = data_size;
	}

	mem_free(buffer);

#if 1
        if (gfx_debug_mode & DMODE_KERNEL_DEBUG8)
	    printk("VC_IOCTL_BULK_QUEUE_RECEIVE returning 0\n");
#endif
        return 0;
    } // case VC_IOCTL_BULK_QUEUE_RECEIVE:


    // Allocate and manage a kernel buffer.  Manage means to free the buffer when the connection
    // closes, in the case that the user process forgets to free it or terminates abnormally.

            /*
      	     * On entry, arg points to this word:
      	     * -------------------------------------------------- *
             *               Requested buffer size                *
             * -------------------------------------------------- *

             * On exit, arg points to this word:
             * -------------------------------------------------- *
             *     Kernel virtual address of allocated buffer     *
             * -------------------------------------------------- */

    // Buffer is allocated by vmalloc() because kmalloc() buffers can't be mmap()'ed by user code.
    // vmalloc() guarantees page boundary alignment.
    //
    // The first 16 bytes of the buffer are reserved for use by the kernel, and to specify the
    // service handle to VCHI; the net result is that the data area is 16 bytes shorter, and is
    // guaranteed to start on a 16-byte boundary.

    case VC_IOCTL_ALLOC_BUFFER: 
    {
        VC_FileData_t  *fileData = (VC_FileData_t *)file->private_data;
        int __user *argp = (int __user *)arg;
        int bufferSize;

        if (0 != get_user(bufferSize, argp))
        {
            printk(KERN_ERR "VC_IOCTL_ALLOC_BUFFER failed to get buffer size word\n");
            return -EFAULT;
        }

        if ((bufferSize < 0) || (bufferSize > MAX_MSG_BUF_SIZE))
        {
            printk(KERN_ERR "VC_IOCTL_ALLOC_BUFFER buffer size %d out of range.\n", bufferSize);
            return -EFAULT;
        }
        else
        {
            VC_msg_buffer_t *newBuffer;

            newBuffer = mem_alloc( gVcAllocBufferKmalloc, bufferSize );

            if (NULL == newBuffer)
            {
                printk(KERN_ERR "VC_IOCTL_ALLOC_BUFFER failed to allocate kernel buffer of size %d\n", bufferSize);
                return -EFAULT;
            }
            newBuffer->next_buffer = fileData->gfxBufferList;
            fileData->gfxBufferList = newBuffer;
            newBuffer->total_size = bufferSize;

	    // Send the graphics debugging mode word back to the user program.
            newBuffer->vchi_handle = gfx_debug_mode;

            __put_user((void *)newBuffer, (void **)arg);

            printk(KERN_DEBUG "VC_IOCTL_ALLOC_BUFFER allocated kernel buffer at 0x%x  size %d  debug 0x%x\n",
			 (unsigned int)newBuffer, newBuffer->total_size, newBuffer->vchi_handle);
            return 0;
        }
    } // case VC_IOCTL_ALLOC_BUFFER:


    // Free a managed buffer, and purge it from the list of buffers to be freed when the connection closes.
    case VC_IOCTL_FREE_BUFFER:
    {
        VC_msg_buffer_t *kvaddr = (VC_msg_buffer_t *)(arg);
        VC_FileData_t  *fileData = (VC_FileData_t *)file->private_data;
        VC_msg_buffer_t *this_buffer = fileData->gfxBufferList;
        VC_msg_buffer_t *prev_buffer = NULL;

        // Find the buffer in the list of allocated buffers.
        while (this_buffer)
        {
            if (kvaddr == this_buffer)
            {
                // Purge it from the list.
                void *free_this_one = this_buffer;
                printk(KERN_INFO "VC_IOCTL_FREE_BUFFER: purging VC3 buffer at 0x%x.\n", (unsigned int)this_buffer);
                if (NULL == prev_buffer)
                {
                    fileData->gfxBufferList = this_buffer->next_buffer;
                }
                else
                {
                    prev_buffer->next_buffer = this_buffer->next_buffer;
                }

                // Free the buffer.
                mem_free(free_this_one);

                // There should be only one match in the list, so we are done.
                return 0;
            }
            prev_buffer = this_buffer;
            this_buffer = this_buffer->next_buffer;
        }

        // Buffer not found in list.
        return -EFAULT;
    } // case VC_IOCTL_FREE_BUFFER:


    // Enqueue one or more messages to the VC3, from control and/or bulk streams
    // in a pre-allocated kernel buffer.
    // The buffer header format is defined in vc.h as VC_msg_buffer_t
    case VC_IOCTL_MSG_QUEUE_FROM_BUFFER:
    {
        VC_msg_buffer_t *bufferP = (VC_msg_buffer_t *)arg;
        VC_FileData_t *fileData = (VC_FileData_t *)file->private_data;
        VC_msg_buffer_t *this_buffer = fileData->gfxBufferList;
        uint8_t *data_ptr = (uint8_t *)&bufferP->msg_data[0];
        int32_t count, data_size;

        // Find the buffer in the list of allocated buffers.
        while (this_buffer)
        {
            if (bufferP == this_buffer)
            break;
 
            this_buffer = this_buffer->next_buffer;
        }

        if (NULL == this_buffer)
        {
            printk(KERN_ERR "VC_IOCTL_MSG_QUEUE_FROM_BUFFER: invalid kernel buffer address %p\n", (void *)bufferP);
            return -EFAULT;
        }

#if 1
        if (gfx_debug_mode & (DMODE_KERNEL_DEBUG2 | DMODE_KERNEL_DEBUG4))
        {
            printk(KERN_INFO "\nVC_IOCTL_MSG_QUEUE_FROM_BUFFER: handle 0x%08x  addr 0x%08x  size %d\n",
                 (uint32_t)bufferP->vchi_handle, (uint32_t)&bufferP->msg_data[0], bufferP->data_size);
        }
#endif

        // Peel off and dispatch each stream.
        data_size = bufferP->data_size;
		//printk(KERN_ERR "BUFFER data_size: %d, buffer start: 0x%08x\n",data_size, &bufferP->msg_data[0]);
        data_ptr = (uint8_t *)&bufferP->msg_data[0];
        while (data_size > 0)
        {
	    // First word of each chunk is a byte count.
	    // Fetch the count word from each chunk, skip over the count word.
	    count = *(uint32_t *)data_ptr;
	    data_ptr  += sizeof(uint32_t);
	    data_size -= sizeof(uint32_t);

	    if (count > 0)		// Data destination is the control channel.
	    {
               if (gfx_debug_mode & DMODE_KERNEL_DEBUG2)
                  printk("CTRL  addr %p  count %d\n", data_ptr, count);

// ???	       // Round up the size of each transfer to 16-byte boundary,
// ???	       // in order to prevent VCHI low level code from copying the buffer.
// ???               vchi_msg_queue((void *)bufferP->vchi_handle, data_ptr, (count + 0xf) & ~0xf,
               vchi_msg_queue((void *)bufferP->vchi_handle, data_ptr, count,
			VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
		       	/*msg handle*/ NULL);
	    }
	    else if (count == 0)
	    {
		// This should not occur.
		printk( "ERROR   addr %p stream count is 0, but buffer not exhausted\n", data_ptr);
		count = 32;		// Set to print a line of data.
		data_size = 0;		// Set to terminate loop after printing stream contents.
	    }
	    else			// Data destination is the bulk channel.
	    {
	       count = -count;

               if (gfx_debug_mode & DMODE_KERNEL_DEBUG4) 
                  printk("BULK  addr %p  count %d\n", data_ptr, count);

// ???	       // Round up the size of each transfer to 16-byte boundary,
// ???	       // in order to prevent VCHI low level code from copying the buffer.
// ???              vchi_bulk_queue_transmit((void *)bufferP->vchi_handle,  data_ptr, (count + 0xf) & ~0xf,
               vchi_bulk_queue_transmit((void *)bufferP->vchi_handle,  data_ptr, count,
			VCHI_FLAGS_BLOCK_UNTIL_OP_COMPLETE,
		       	/*msg handle*/ NULL);
	    }

            if (gfx_debug_mode & (DMODE_KERNEL_DEBUG2 | DMODE_KERNEL_DEBUG4))
            {
	        int i;
		uint32_t *word_ptr = (uint32_t *)data_ptr;

                for (i = 0; i < (count >> 2); i++)
                {
                   printk(" x%08x", word_ptr[i]);
                   if ((i % 8) == 7) printk("\n");
                }
                if ((i % 8) != 0) printk("\n");		// i incremented again when falling out of loop
            }

	    // Round up the size of each chunk to a word boundary.
	    // Skip to next chunk.
	    data_ptr  += count;
	    data_size -= count;
        }

        if (gfx_debug_mode & DMODE_KERNEL_DEBUG1)
	{
	   // Make sure that mphi_rxtx_task() has a chance to call mphi_pumptx() to start the transfer.
	   msleep(0);

#if 0	// Don't need this, because we are not freeing the kernel buffer any time soon.
           // Make sure all PIO or DMA output is finished before freeing the buffer.
           mphi_get_func_table()->finish_io(0);
#endif
	}

#if 1
        if (gfx_debug_mode & (DMODE_KERNEL_DEBUG2 | DMODE_KERNEL_DEBUG4))
            printk("VC_IOCTL_MSG_QUEUE_FROM_BUFFER returning 0\n");
#endif
        return 0;
    } // case VC_IOCTL_MSG_QUEUE_FROM_BUFFER:

#endif // CFG_GLOBAL_GFX_SERVICE_SUPPORT


    case VC_IOCTL_GENCMD:
    {
        VC_GenCmd_t gencmd;
        if ( copy_from_user( gencmd.cmd, ((VC_GenCmd_t *)arg)->cmd, sizeof( gencmd.cmd )) != 0 )
        {
            return -EFAULT;
        }

        /* assuming doing only one videocore for the time being */
        gencmd.response.err = vc_gencmd( gVideoCoreVCHIHdls[0].gencmd_handle, gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );

        if ( copy_to_user( &((VC_GenCmd_t *)arg)->response, &gencmd.response, sizeof( gencmd.response )) != 0 )
        {
            return -EFAULT;
        }
        return 0;
    }
      
    default:
      {
	printk( KERN_WARNING "vc03: Unrecognized ioctl: '0x%x'\n", cmd );
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
   VC_FileData_t *fileData;

  (void)inode;

   if (( fileData = kcalloc( 1, sizeof( VC_FileData_t ), GFP_KERNEL )) == NULL )
   {
      return -ENOMEM;
   }

   fileData->gfxBufferList = NULL;

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
   VC_msg_buffer_t  *this_buffer;

   if (gfx_debug_mode & DMODE_KERNEL_DEBUG1)
   {
      // Make sure that mphi_rxtx_task() has a chance to call mphi_pumptx() to start any pending transfer.
      msleep(10);

      // Make sure all PIO or DMA output is finished before freeing the buffer.
      mphi_get_func_table()->finish_io(1);
   }

   if (( fileData = (VC_FileData_t *)file->private_data ) != NULL )
   {
      this_buffer = fileData->gfxBufferList;
      while (this_buffer)
      {
         void *free_this_one = this_buffer;

#if 1
         printk(KERN_DEBUG "vc_release: freeing kernel buffer at: %p  next %p.\n",
						this_buffer, this_buffer->next_buffer);
#endif
         this_buffer = this_buffer->next_buffer;
         mem_free(free_this_one);
      }

      kfree( fileData );
   }
    
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
static int vc_mmap(struct file *filp, struct vm_area_struct *vma)
{  
    int ret;
    long length = (long)(vma->vm_end - vma->vm_start);
    unsigned long start = vma->vm_start;
    unsigned long pfn;
    unsigned long virtAddr = vma->vm_pgoff << PAGE_SHIFT;

    if (( virtAddr >= VMALLOC_START ) && ( virtAddr < VMALLOC_END ))
    {
        while (length > 0)
        {
            pfn = vmalloc_to_pfn( (void *)virtAddr );
            if ((ret = remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED)) < 0)
            {
                return ret;
            }
            start += PAGE_SIZE;
            virtAddr += PAGE_SIZE;
            length -= PAGE_SIZE;
        }
    }
    else
    {
        pfn = virt_to_phys( (void *)virtAddr ) >> PAGE_SHIFT;
        if ((ret = remap_pfn_range(vma, start, pfn, length, PAGE_SHARED )) < 0)
        {
            return ret;
        }
    }
  
    return 0;
}

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
    mmap:       vc_mmap,	
};

/*
 *
 */
int vc3if_proc_rwtest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  *eof = 1;
  return count;
}

extern int vc_do_vchitest(vc03drv_t* drv, char* cmd);

int vc3if_proc_rwtest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  const int maxstr = 50;
  char test_str[maxstr + 1];
  
  count = count > maxstr ? maxstr : count;
  if(copy_from_user(&test_str, buffer, count))
	{
	  return -EFAULT;
	}
  vc_do_vchitest(vc03drv_get(), test_str);
  return count;  
}

/*
 *
 */
int vc3if_proc_dispmantest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  *eof = 1;
  return count;
}

/*
 *
 */
int vc_do_dispmanxtest(int testnum);

int vc3if_proc_dispmantest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  const int maxstr = 50;
  char test_str[maxstr + 1];
  int testnum = 0;
  
  count = count > maxstr ? maxstr : count;
  if(copy_from_user(&test_str, buffer, count))
	{
	  return -EFAULT;
	}
  if( 1 != sscanf( test_str, "%d", &testnum ) )
  {
     VC_DEBUG( Trace, "vcif_prof_dispmantest_write: invalid parameter %s", test_str );
     return count;
  }
  vc_do_dispmanxtest(testnum);
  return count;
}

/*
 *
 */
int vc3if_proc_fbtest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;
  
  if (offset > 0)
	{	
	  *eof = 1;	  
	  return 0;	  
	}
  len += sprintf(buf + len," vc3if_proc_fbtest_read\n");
  *eof = 1;
  return len;
}

int vc3if_proc_fbtest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  char testnum_str[21];
  int testnum = 0;

  count = count> 20 ? 20 : count;
  if(copy_from_user(&testnum_str, buffer, count))
	{
	  return -EFAULT;
	}
  if(1 != sscanf(testnum_str, "%d", &testnum))
	{
	  VC_DEBUG( Trace, "vc3if_proc_fbtest_write: invalid parameter: %s", testnum_str);
	  return count;
	}
    
  lcd_colorbars(testnum);
  VC_DEBUG( Trace, "testnum=%d\n", testnum);
  
  return count;
  
}

/*
 *
 */
void vc_do_stripes(void); // exported by vcstripes.c
void vc_do_play(char* filename, int disp);
void vc_do_cam(int disp, int start);

#if CAMERA_PREVIEW_RENDERER
void vc_do_play_cam(char* filename, int disp);
#endif
int  vc_do_digicam2(char* cmd);
int  vc_do_recorder2(char* cmd);

int vc3if_proc_omxtest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;
  
  if (offset > 0)
	{	
	  *eof = 1;	  
	  return 0;	  
	}

  len += sprintf(buf + len," vc3if_proc_omxtest_read\n");
  *eof = 1;
  return len;
}

int vc3if_proc_omxtest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  const int maxlen = 255;
  char test_str[maxlen+1];
  char* filename;

  count = min((long)count, (long)maxlen);
  memset(test_str, 0, maxlen+1);
  if(copy_from_user(&test_str, buffer, count))
    {
      return -EFAULT;
    }
  test_str[count-1] = 0;
  VC_DEBUG(Trace, "test_str=%s count=%d\n", test_str, (int)(count-1));
  
  // display stripes test
  if(0 == strcmp(test_str, "stripes"))
    {
      VC_DEBUG(Trace, "do_stripes \n");
      vc_do_stripes();
    }
#if CAMERA_PREVIEW_RENDERER
  // playback_cam test
  else if(0 == strncmp(test_str, "play_cam", 8))
    {
      if(strlen(test_str) <=9)
	{
	  filename = "/mfs/sd/mediafiles/trans.mp4";
	}
      else
	{
	  filename = test_str + 9;
	}
      VC_DEBUG(Trace, "do_play_cam filename=%s len=%d\n", filename, strlen(filename));
      vc_do_play_cam(filename, 0);
    }
#endif
  // playback test
  else if(0 == strncmp(test_str, "play", 4))
    {
      if(strlen(test_str) <=5)
	{
	  filename = "/mfs/sd/mediafiles/trans.mp4";
	}
      else
	{
	  filename = test_str + 5;
	}
      VC_DEBUG(Trace, "do_play filename=%s len=%d\n", filename, strlen(filename));
      vc_do_play(filename, 0);
    }
  // camera test
  else if(0 == strncmp(test_str, "cam", 3))
    {      
       char* cmd;
       if( strlen( test_str ) <= 4 )
       {
         vc_do_cam(0, 1);
       }
       else
       {
          cmd = test_str + 4;
          if( 0 == strncmp( cmd, "stop", 4 ) )
          {
             vc_do_cam( 0, 0 );
          }
       }
    }
#if 0
  else if(0 == strncmp(test_str, "enctest", 7))
    {      
       char* cmd;
       if( strlen( test_str ) <= 8 )
       {
            vc_do_camenc(0, 1);
       }
       else
       {
          cmd = test_str + 8;
          if( 0 == strncmp( cmd, "stop", 4 ) )
          {
             vc_do_camenc( 0, 0 );
          }
       }
    }
#endif
  else if(0 == strncmp(test_str, "digicam", 7))
    {      
      //      vc_do_digicam2(test_str + 8);
    }
  else if(0 == strncmp(test_str, "recorder", 8))
    {      
      //      vc_do_recorder2(test_str + 9);
    }
  else
    {
      VC_DEBUG(Trace, "invalid command: %s\n", test_str);
   }
  return count;
  
}

/*
 *
 */

int vc03_init(vc03drv_t* drv, int mask);

int vc3if_proc_reset_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int ret;
  int len = 0;
  
  if (offset > 0)
	{	
	  *eof = 1;	  
	  return 0;	  
	}

  ret = vc03_init(vc03drv_get(), (1 << 0) | (1 << 1));
  //  ret = vc03_reset_bootmem(10);
  if(0 == ret)
	{
	  VC_DEBUG(Trace, "VC03 initialized\n");
	}
  else
	{
	  VC_DEBUG(Trace, "VC03 init failed\n");
	}	
	  
  *eof = 1;
  return len;
}

int vc3if_proc_reset_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  int delay;
  char*  reset_str;
  char* path;
  char* path_end;

  reset_str = kmalloc(VMCSPATH_MAX, GFP_KERNEL);
  if(0 == reset_str)
    return -ENOMEM;

  memset(reset_str, 0, VMCSPATH_MAX);
  count = count> VMCSPATH_MAX ? VMCSPATH_MAX : count;
  if(copy_from_user(reset_str, buffer, count))
    {
      return -EFAULT;
    }

  reset_str[VMCSPATH_MAX-1] = 0;
  if(1 != sscanf(reset_str, "%d", &delay))
    {
      delay = 2000;
    }
  path = strchr(reset_str, ' ');
  if(NULL==path)    
    {
      path = VMCSPATH_DEFAULT;
    }
  else
    {
      ++path;
    }

  path_end = path + strlen(path) - 1;
  if(*path_end == '\n')
    *path_end = 0;

  VC_DEBUG(Trace, "%d msec delay after reset, booting %s\n", delay, path);	  
  
  if(delay >= 0)
    {
      if(0 == vc03_reset_bootfile(delay, path))
	{
	  VC_DEBUG(Trace, "VC03 interface initialized\n");
	}
      else
	{
	  VC_DEBUG(Trace, "VC03 interface init failed\n");
	}
    }
  else
    {
     vchost_pininit();
     vchost_gpio_set_value(HW_VC03_RUN_GPIO, 1);
  	 mdelay(500);
    }

  kfree(reset_str);
  return count;
}

/*
 *
 */
int vc3if_proc_omxlog_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;
  int ret;

  *start = buf;// + offset;
  while(len < count)
	{
	  ret = omxlog_read(buf + len /*+ offset*/, count - len);
	  if(ret < 1)
		{
		  break;
		}
	  len += ret;
	}

  /// buffer is empty
  if(ret == 0  && len > 0)
	{
	  len += sprintf(buf + len, "\n");
	}

  if(len == 0)
	{
	  *eof = 1;
	}
  return len;
}

/*
 *
 */
int vc3if_proc_gcmdtest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  (void)buf;
  (void)start;
  (void)offset;
  (void)count;
  (void)eof;
  (void)data;
  *eof = 1;	  
  return 0;	  
}

int vc3if_proc_gcmdtest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  VC_GenCmd_t vc3gencmd;
  memset(&vc3gencmd, 0, sizeof(vc3gencmd));
  if(copy_from_user(vc3gencmd.cmd, buffer, count))
	{
	  return -EFAULT;
	}
  VC_DEBUG( Trace, "send gencmd=%s\n", vc3gencmd.cmd);
  /* assuming we are supporting one videocore for the time being */
  vc3gencmd.response.err = vc_gencmd(gVideoCoreVCHIHdls[0].gencmd_handle, vc3gencmd.response.str, sizeof( vc3gencmd.response.str ), "%s", &vc3gencmd.cmd[0]);
  if (vc3gencmd.response.err == 0 )
	{
	  printk(/*VC_DEBUG( Trace, */"vc_gencmd cmd=\"%s\" resp=\"%s\"\n", vc3gencmd.cmd, vc3gencmd.response.str);
	}
  else
	{
	  printk(/*VC_DEBUG( Trace, */"vc_gencmd cmd=\"%s\" err=%d\n", vc3gencmd.cmd, vc3gencmd.response.err);
	}

  return count;
}

/*
 *
 */
int vc3if_proc_gfxtest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;

  if (offset > 0)
	{
	  *eof = 1;
	  return 0;
	}
  len += sprintf(buf + len," vc3if_proc_fbtest_read\n");
  *eof = 1;
  return len;
}

/*
 * Set the gfx_debug_mode control variable.
 */
int vc3if_proc_gfxtest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  char testnum_str[21];
  int testnum = 0;

  count = count> 20 ? 20 : count;
  if(copy_from_user(&testnum_str, buffer, count))
    {
	  return -EFAULT;
    }
  if(1 != sscanf(testnum_str, "%d", &testnum))
    {
      VC_DEBUG( Trace, "vc3if_proc_fbtest_write: invalid parameter: %s", testnum_str);
      return count;
    }

  gfx_debug_mode = testnum;

  return count;
}
/*
 *
 */
int vc3if_proc_dectest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;

  if (offset > 0)
	{
	  *eof = 1;
	  return 0;
	}
  len += sprintf(buf + len," vc3if_proc_dectest_read\n");
  *eof = 1;
  return len;
}

int vc3if_proc_dectest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  int framenum;
  char*  reset_str;
  char* path;
  char* path_end;
  
  reset_str = vmalloc(VMCSPATH_MAX);
  if(0 == reset_str)
    return -EFAULT;
  memset(reset_str, 0, VMCSPATH_MAX);
  count = count> VMCSPATH_MAX ? VMCSPATH_MAX : count;
  if(copy_from_user(reset_str, buffer, count))
    {
      return -EFAULT;
    }
  reset_str[VMCSPATH_MAX] = 0;
  if(1 != sscanf(reset_str, "%d", &framenum))
    {
      framenum = 2000;
    }
  path = strchr(reset_str, ' ');
  if(NULL==path)    
    {
      path = VMCSPATH_DEFAULT;
    }
  else
    {
      ++path;
    }

  path_end = path + strlen(path) - 1;
  if(*path_end == '\n')
    *path_end = 0;

  VC_DEBUG(Trace, "playing %d frames file %s\n", framenum, path);

  /* start the decoder thread for 2 way video */
   vc_start_decThd();

  vc_do_dectest( framenum, 40, path );
  
  vfree(reset_str);
  return count;
}

int vc3if_proc_vidconftest_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len = 0;
  
  if (offset > 0)
	{	
	  *eof = 1;	  
	  return 0;	  
	}

  len += sprintf(buf + len," vc3if_proc_vidconftest_read\n");
  *eof = 1;
  return len;
}
void vc_start_enctest( char * test_str );
void vc_set_encdisp( char * test_str );
void vc_stop_enctest( void );
void vc_start_dectest( char * test_str );
void vc_set_decdisp( char * test_str );
void vc_stop_dectest( void );

int vc3if_proc_vidconftest_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
  const int maxlen = 255;
  char test_str[maxlen+1];
  //char* filename;
  char * path;
//   VC03_VIDEOCONF_CMD cmd;

  count = min((long)count, (long)maxlen);
  memset(test_str, 0, maxlen+1);
  if(copy_from_user(&test_str, buffer, count))
    {
      return -EFAULT;
    }
  test_str[count-1] = 0;
  printk("video conference test, trace = %d str=%s\n", gVcDebugTrace, test_str );
  VC_DEBUG(Trace, "test_str=%s count=%d\n", test_str, (int)(count-1));

  if( 0 == strncmp( test_str, "enc", 3) )
  {
     path = strchr( test_str, ' ');
     path++;
     vc_start_enctest( path );
  }
  else if( 0 == strncmp( test_str, "dec", 3 ) )
  {
     path = strchr( test_str, ' ');
     path++;
     vc_start_dectest( path );     
  }
  else if( 0 == strncmp( test_str, "disp_enc", 8 ) )
  {
     path = strchr( test_str, ' ');
     path++;
     vc_set_encdisp( path );     
  }
  else if( 0 == strncmp( test_str, "stop_enc", 8 ))
  {
     vc_stop_enctest( );
  }
  else if( 0 == strncmp( test_str, "disp_dec", 8 ) )
  {
     path = strchr( test_str, ' ');
     path++;
     vc_set_decdisp( path );
  }
  else if( 0 == strncmp( test_str, "stop_dec", 8 ))
  {
     vc_stop_dectest( );
  }
  
  else
  {
     VC_DEBUG(Trace, "test not supported (enc or dec)\n");
  }
  
  return count;
}

/*
 *
 */

int vc03_proc_init(void)
{
  struct proc_dir_entry *vc3if_procdir_rwtest;
  struct proc_dir_entry *vc3if_procdir_fbtest;
  struct proc_dir_entry *vc3if_procdir_dispmantest;
  struct proc_dir_entry *vc3if_procdir_omxtest;
  struct proc_dir_entry *vc3if_procdir_omxlog;
  struct proc_dir_entry *vc3if_procdir_gcmdtest;
  struct proc_dir_entry *vc3if_procdir_gfxtest;
  struct proc_dir_entry *vc3if_procdir_dectest;
  struct proc_dir_entry *vc3if_procdir_vidconftest;
  
  //zrl
  struct proc_dir_entry *vc3if_procdir_vmcslog;

#if (CFG_GLOBAL_CHIP == BCM2153)
  start_gptimer();
#endif

  
  sema_init(&vmcslog_lock, 1);
  vmcslog_rindex = 0;
  vmcslog_windex = 0;
  vmcslog = vmalloc(VC03_VMCSLOG_MAX);
  memset(vmcslog, 0, VC03_VMCSLOG_MAX);
  
  vc3if_procdir_vmcslog = create_proc_entry("VC03_vmcslog", 0644, NULL);
  //
  // /proc/VC03_vmcslog
  //
  if (vc3if_procdir_vmcslog == NULL)
	{
	  remove_proc_entry("VC03_vmcslog", NULL);
	  printk(KERN_ALERT "could not initialize /proc/VC03_vmcslog");
	  return -ENOMEM;
	}
  
  vc3if_procdir_vmcslog->read_proc  = vc3if_proc_vmcslog_read;
  vc3if_procdir_vmcslog->write_proc = NULL;
  vc3if_procdir_vmcslog->owner 	  = THIS_MODULE;
  vc3if_procdir_vmcslog->mode 	  = S_IFREG | S_IRUGO;
  vc3if_procdir_vmcslog->uid 	  = 0;
  vc3if_procdir_vmcslog->gid 	  = 0;
  vc3if_procdir_vmcslog->size 	  = 37;

  
  //
  // /proc/VC03_rwtest
  //
  vc3if_procdir_rwtest = create_proc_entry("VC03_rwtest", 0644, NULL);
  
  if (vc3if_procdir_rwtest == NULL)
	{
	  remove_proc_entry("VC03_rwtest", NULL);
	  printk(KERN_ALERT "could not initialize /proc/VC03_rwtest");
	  return -ENOMEM;
	}
  
  vc3if_procdir_rwtest->read_proc  = vc3if_proc_rwtest_read;
  vc3if_procdir_rwtest->write_proc = vc3if_proc_rwtest_write;
  vc3if_procdir_rwtest->owner 	  = THIS_MODULE;
  vc3if_procdir_rwtest->mode 	  = S_IFREG | S_IRUGO;
  vc3if_procdir_rwtest->uid 	  = 0;
  vc3if_procdir_rwtest->gid 	  = 0;
  vc3if_procdir_rwtest->size 	  = 37;
  
  //
  // /proc/VC03_fbtest
  //
  vc3if_procdir_fbtest =   create_proc_entry("VC03_fbtest", 0644, NULL);
  
  if (vc3if_procdir_fbtest == NULL)
	{
	  remove_proc_entry("VC03_fbtest", NULL);
	  printk(KERN_ALERT "could not initialize /proc/VC03_fbtest");
	  return -ENOMEM;
	}
  
  vc3if_procdir_fbtest->read_proc  = vc3if_proc_fbtest_read;
  vc3if_procdir_fbtest->write_proc = vc3if_proc_fbtest_write;
  vc3if_procdir_fbtest->owner 	  = THIS_MODULE;
  vc3if_procdir_fbtest->mode 	  = S_IFREG | S_IRUGO;
  vc3if_procdir_fbtest->uid 	  = 0;
  vc3if_procdir_fbtest->gid 	  = 0;
  vc3if_procdir_fbtest->size 	  = 37;
  
  //
  // /proc/VC03_dispmantest
  //
  vc3if_procdir_dispmantest =   create_proc_entry("VC03_dispmanxtest", 0644, NULL);
  
  if (vc3if_procdir_dispmantest == NULL)
	{
	  remove_proc_entry("VC03_dispmantest", NULL);
	  printk(KERN_ALERT "could not initialize /proc/VC03_dispmanxtest");
	  return -ENOMEM;
	}
  
  vc3if_procdir_dispmantest->read_proc  = vc3if_proc_dispmantest_read;
  vc3if_procdir_dispmantest->write_proc = vc3if_proc_dispmantest_write;
  vc3if_procdir_dispmantest->owner 	  = THIS_MODULE;
  vc3if_procdir_dispmantest->mode 	  = S_IFREG | S_IRUGO;
  vc3if_procdir_dispmantest->uid 	  = 0;
  vc3if_procdir_dispmantest->gid 	  = 0;
  vc3if_procdir_dispmantest->size 	  = 37;
  
   //
   // /proc/VC03_omxtest
   //
   vc3if_procdir_omxtest = create_proc_entry("VC03_omxtest", 0644, NULL);
	
   if (vc3if_procdir_omxtest == NULL)
	 {
	   remove_proc_entry("VC03_omxtest", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_omxtest");
	   return -ENOMEM;
	 }

   vc3if_procdir_omxtest->read_proc  = vc3if_proc_omxtest_read;
   vc3if_procdir_omxtest->write_proc = vc3if_proc_omxtest_write;
   vc3if_procdir_omxtest->owner 	  = THIS_MODULE;
   vc3if_procdir_omxtest->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_omxtest->uid 	  = 0;
   vc3if_procdir_omxtest->gid 	  = 0;
   vc3if_procdir_omxtest->size 	  = 37;
      

   // /proc/VC03_omxlog
   //
   vc3if_procdir_omxlog = create_proc_entry("VC03_omxlog", 0644, NULL);
	
   if (vc3if_procdir_omxlog == NULL)
	 {
	   remove_proc_entry("VC03_omxlog", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_omxlog");
	   return -ENOMEM;
	 }

   vc3if_procdir_omxlog->read_proc  = vc3if_proc_omxlog_read;
   vc3if_procdir_omxlog->write_proc = NULL;
   vc3if_procdir_omxlog->owner 	  = THIS_MODULE;
   vc3if_procdir_omxlog->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_omxlog->uid 	  = 0;
   vc3if_procdir_omxlog->gid 	  = 0;
   vc3if_procdir_omxlog->size 	  = 37;

   //
   // /proc/VC03_gcmdtest
   //
   vc3if_procdir_gcmdtest = create_proc_entry("VC03_gcmdtest", 0644, NULL);
	
   if (vc3if_procdir_gcmdtest == NULL)
	 {
	   remove_proc_entry("VC03_gcmdtest", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_gcmdtest");
	   return -ENOMEM;
	 }

   vc3if_procdir_gcmdtest->read_proc  = vc3if_proc_gcmdtest_read;
   vc3if_procdir_gcmdtest->write_proc = vc3if_proc_gcmdtest_write;
   vc3if_procdir_gcmdtest->owner 	  = THIS_MODULE;
   vc3if_procdir_gcmdtest->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_gcmdtest->uid 	  = 0;
   vc3if_procdir_gcmdtest->gid 	  = 0;
   vc3if_procdir_gcmdtest->size   = 37;

   //
   // /proc/VC03_gfxtest
   //
   vc3if_procdir_gfxtest = create_proc_entry("VC03_gfxtest", 0644, NULL);

   if (vc3if_procdir_gfxtest == NULL)
	 {
	   remove_proc_entry("VC03_gfxtest", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_gfxtest");
	   return -ENOMEM;
	 }

   vc3if_procdir_gfxtest->read_proc  = vc3if_proc_gfxtest_read;
   vc3if_procdir_gfxtest->write_proc = vc3if_proc_gfxtest_write;
   vc3if_procdir_gfxtest->owner 	  = THIS_MODULE;
   vc3if_procdir_gfxtest->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_gfxtest->uid 	  = 0;
   vc3if_procdir_gfxtest->gid 	  = 0;
   vc3if_procdir_gfxtest->size 	  = 37;
 
   //
   // /proc/VC03_dectest
   //
   vc3if_procdir_dectest = create_proc_entry("VC03_dectest", 0644, NULL);

   if (vc3if_procdir_dectest == NULL)
	 {
	   remove_proc_entry("VC03_dectest", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_dectest");
	   return -ENOMEM;
	 }

   vc3if_procdir_dectest->read_proc  = vc3if_proc_dectest_read;
   vc3if_procdir_dectest->write_proc = vc3if_proc_dectest_write;
   vc3if_procdir_dectest->owner 	  = THIS_MODULE;
   vc3if_procdir_dectest->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_dectest->uid 	  = 0;
   vc3if_procdir_dectest->gid 	  = 0;
   vc3if_procdir_dectest->size 	  = 37;

   //
   // /proc/VC03_vidconftest
   //
   vc3if_procdir_vidconftest = create_proc_entry("VC03_vidconftest", 0644, NULL);
	
   if (vc3if_procdir_vidconftest == NULL)
	 {
	   remove_proc_entry("VC03_vidconftest", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_vidconftest");
	   return -ENOMEM;
	 }

   vc3if_procdir_vidconftest->read_proc  = vc3if_proc_vidconftest_read;
   vc3if_procdir_vidconftest->write_proc = vc3if_proc_vidconftest_write;
   vc3if_procdir_vidconftest->owner 	  = THIS_MODULE;
   vc3if_procdir_vidconftest->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_vidconftest->uid 	  = 0;
   vc3if_procdir_vidconftest->gid 	  = 0;
   vc3if_procdir_vidconftest->size 	  = 37;

   return 0;
}

/*
 *
 */
int vc03_procreset_init(void)
{
  struct proc_dir_entry *vc3if_procdir_reset;
  
   //
   // /proc/VC03_inttest
   //
   vc3if_procdir_reset = create_proc_entry("VC03_reset", 0644, NULL);
	
   if (vc3if_procdir_reset == NULL)
	 {
	   remove_proc_entry("VC03_reset", NULL);
	   printk(KERN_ALERT "could not initialize /proc/VC03_reset");
	   return -ENOMEM;
	 }

   vc3if_procdir_reset->read_proc  = vc3if_proc_reset_read;
   vc3if_procdir_reset->write_proc = vc3if_proc_reset_write;
   vc3if_procdir_reset->owner 	  = THIS_MODULE;
   vc3if_procdir_reset->mode 	  = S_IFREG | S_IRUGO;
   vc3if_procdir_reset->uid 	  = 0;
   vc3if_procdir_reset->gid 	  = 0;
   vc3if_procdir_reset->size 	  = 37;

   return 0;
}


/*
 *
*/
int vc03_vchi_exit(vc03drv_t* drv)
{

  assert(drv);
  assert(drv->initialized == 1);
  drv->initialized = 0;
  //  host_vchi_msgfifo_wrapper_exit();
  vchi_disconnect((VCHI_INSTANCE_T)&drv->instance_vchi);

  //  vchi_exit(); // FIXME, vchi_exit is not implemented

  mphi_get_func_table()->exit();

  return 0;
}

/****************************************************************************
*
*  Init
*
*     Initializes the VC03 interface
*
***************************************************************************/

//??? TODO: These externs should come from an appropriate header file for the server in question and not be here.
extern void vc_vchi_hostreq_init (VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections );

#ifdef KHRONOS_GFX
extern void vc_vchi_khronos_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T **connections, uint32_t num_connections );
#endif // KHRONOS_GFX

#ifdef VCHI_TEST
extern void vc_dc4_server_init( VCHI_INSTANCE_T initialise_instance, VCHI_CONNECTION_T ** connections, uint32_t num_connections );
#endif

int vc03_vchi_init(vc03drv_t* drv)
{
  int ret;
#if 0 // see below
  uint32_t hostportcfg;
#endif
  
  // get driver ptr
  assert(drv);
  assert(drv->initialized == 0);


  //  remove pininit call as this will reset the VC
  //  vchost_pininit();
  //  hostportcfg = 1;
  //  vchost_portinit(&hostportcfg);
  
  os_init();
  // connect
  mphi_get_func_table()->init();
  ret = vchi_initialise(&drv->instance_vchi);
  assert( ret == 0 );
  drv->connection_vchi[0] = vchi_create_connection( single_get_func_table(),
						    vchi_mphi_message_driver_func_table() );

#ifdef VCHI_TEST
  vc_dc4_server_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
#endif

  ret = vchi_connect( &drv->connection_vchi[0], 1, (VCHI_INSTANCE_T)&drv->instance_vchi);

  gVideoCoreVCHIHdls[0].gencmd_handle = vc_vchi_gencmd_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
  vc_vchi_dispmanx_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
  vc_vchi_ilcs_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
  vc_vchi_hostreq_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
  vc_vchi_filesys_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
  //vc_vchi_logserver_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
#ifdef KHRONOS_GFX
  vc_vchi_khronos_init(drv->instance_vchi, &drv->connection_vchi[0], 1);
#endif // KHRONOS_GFX

  drv->initialized = 1;

  return 0;
  
} // Init

/****************************************************************************
*
*  vc_delay_msec
*
*   Delays for at least the specified number of milliseconds
*
***************************************************************************/

void vc_host_delay_msec( unsigned msec )
{
    // Make sure that we're not called from interrupt context
    might_sleep();

    if ( msec < 20 )
    {
        mdelay( msec );
    }
    else
    {
        msleep( msec );
    }

} // vc_host_delay_msec

/****************************************************************************
*
*  vc_is_initialized.
*
***************************************************************************/

int vc_is_initialized( void )
{
  vc03drv_t* drv;
  drv = vc03drv + 0; // FIXME: use dev minor
  assert(drv);
  return (drv->initialized);

} // vc_is_initialized


/*
 *
 */
int vchost_sendfile(const char* vcvmcs_path)
{
    const int bufsize_max = 64 * 1024;
    uint8_t* buf;
    int fd, nbytes, flags, totalbytes;
    mm_segment_t old_fs;
    
    // open the file
    flags = O_RDONLY;

    old_fs = get_fs();
    set_fs(get_ds());
    fd = sys_open(vcvmcs_path, flags, 0777);
    set_fs(old_fs);

    if(fd < 0)
    {
        printk( KERN_ERR "Unable to open videocore firmware file: %s (%d)\n", vcvmcs_path, fd );
        set_fs(old_fs);
        return fd;
    }

    if (( buf = kmalloc(bufsize_max, GFP_KERNEL)) == NULL )
    {
        old_fs = get_fs();
        set_fs(get_ds());
        sys_close(fd);
        set_fs(old_fs);

        printk( KERN_ERR "Unable to allocate %d bytes for videocore firmware file\n", bufsize_max );
        return -ENOMEM;
    }

    // indicate start of data transfer 
    vchost_writemsg(NULL, 0, 1);
    
    totalbytes = 0;
    do
    {
        memset(buf, 0, bufsize_max);

        old_fs = get_fs();
        set_fs(get_ds());
        nbytes = sys_read(fd, buf, bufsize_max);
        set_fs(old_fs);

        if ( nbytes > 0 )
        {
            nbytes = (nbytes + 0xF) & ~0xF;
            nbytes = vchost_writedata(buf, nbytes);
            totalbytes += nbytes;
        }
    } while ( nbytes == bufsize_max );


    // finish message
    //  vchost_writeparam(0);
    old_fs = get_fs();
    set_fs(get_ds());
    sys_close(fd);
    set_fs(old_fs);
    
    kfree(buf);
    return 0;
}

/*
 *
 */

#if 0
static int vc03_bootmem(uint8_t* stage2_ptr, uint32_t stage2_size, uint8_t* vcvmcs_ptr, uint32_t vcvmcs_size)
{
  uint32_t msg[8];
  uint32_t length;
  boot_message2_t msg2;
  int iter;

  memset(msg, 0, sizeof(msg));

  iter = 500;
  while(iter--)
    {
      // read out message 1 "boot rom running"
      length = vchost_readmsg(msg, sizeof(msg), 0);
      if(length == 32 && msg[0] == 0xAFE45251 && (msg[1] & (1 << 24)))
	break;
    }
  
  VC_DEBUG(Trace, "length=%d msg[0..1]=0x%08X %08X\n", length, msg[0], msg[1]);
  if(0 == iter)
    {
      VC_DEBUG(Trace, "error: boot failed, invalid message from 2727\n");
      return -1;
    }

  // write stage 2 loader, use channel 1
  vchost_writemsg(stage2_ptr, stage2_size, 1);   

  
  // write ctrl message 2 "host finished sending stage 2 loader", use channel 0
  memset(&msg2, 0, sizeof(msg2));
  msg2.header = 0xAFE45251;
  msg2.id_size = (2 << 24) | (stage2_size & 0xffffff);
  msg2.footer = 0xF001BC4E;
  vchost_writemsg(&msg2, sizeof(msg2), 0);
  // finish
  vchost_writeparam(0);

  // poll 2727 for message 3 "boot rom received and verified stage 2 loader" 
  iter = 3000;
  do
    {
      memset(msg, 0, sizeof(msg));
      length = vchost_readmsg(msg, sizeof(msg), 0);
      if(length == 32 && 
	 msg[0] ==0xAFE45251 && 
	 (msg[1] >> 24) == 1)
	{
	  break;
	}
    }
  while(--iter);
  
  VC_DEBUG(Trace, "message 3: length=%d msg[0..1]=0x%08X %08X\n", length, msg[0], msg[1]);
  if(iter < 1)
    {
      VC_DEBUG(Trace, "message 3 invalid\n");
      return -1;
    }

  // load vcvmcs.bin
  // write stage 2 loader, use channel 1
  vchost_writemsg(vcvmcs_ptr, vcvmcs_size, 1);

  memset(&msg2, 0, sizeof(msg2));
  msg2.header = 0xAFE45251;
  msg2.id_size = (2 << 24) | (stage2_size & 0xffffff);
  msg2.footer = 0xF001BC4E;
  vchost_writemsg(&msg2, sizeof(msg2), 0);
  // finish
  vchost_writeparam(0);
  
  return 0;
}
#endif

/*
 *
 */
static int vc03_bootfile(uint8_t* stage2_ptr, uint32_t stage2_size, const char* vcvmcs_path)
{
    uint32_t msg[8];
    int      length;
    boot_message2_t msg2;
    int iter;
    int rc;
    
    memset(msg, 0, sizeof(msg));
    
    VC_DEBUG( BootTrace, "Waiting for BootROM running message..." );

    vchost_set_interface_for_boot();

    // read out message 1 "boot rom running"
    iter = 500;
    while(iter--)
    {
        // read out message 1 "boot rom running"
        length = vchost_readmsg(msg, sizeof(msg), 0);

#if 0
        if (( length > 0 ) && gVcDebugBootTrace )
        {
            printk( KERN_INFO "Msg1 Length=%d\n", length );
            vc_dump_mem( "  msg:", 0, msg, length );
        }
#endif
        
        if ((length == 32 ) && ( msg[0] == 0xAFE45251 ) && (( msg[1] & (1 << 24)) != 0 ))
        {
            break;
        }
    }
    if ( 0 == iter )
    {
        printk( KERN_ERR "Error: Timeout waiting for VideoCore BootROM running message\n" );
        return -EIO;
    }

    VC_DEBUG( BootTrace, "Writing 2nd stage BootLoader..." );

    vchost_writemsg( stage2_ptr, stage2_size, 1 );
    
    VC_DEBUG( BootTrace, "Finished sending 2nd stage BootLoader..." );

    // write ctrl message 2 "host finished sending stage 2 loader", use channel 0
    memset(&msg2, 0, sizeof(msg2));
    msg2.header = 0xAFE45251;
    msg2.id_size = (2 << 24) | (stage2_size & 0xffffff);
    msg2.footer = 0xF001BC4E;
    vchost_writemsg(&msg2, sizeof(msg2), 0);
    // finish
    vchost_writeparam(0);

    VC_DEBUG( BootTrace, "Waiting for 2nd stage BootLoader verification..." );

    // poll 2727 for message 3 "boot rom received and verified stage 2 loader" 
    iter = 3000;
    do
    {
        memset(msg, 0, sizeof(msg));
        length = vchost_readmsg(msg, sizeof(msg), 0);
        if ( length > 0 )
        {
            VC_DEBUG( BootTrace, "Msg2 Length=%d msg[0..1]=0x%08X %08X\n", length, msg[0], msg[1]);
        }
        if(length == 32 && msg[0] ==0xAFE45251 && (msg[1] >> 24) == 1)
            break;
    } while(--iter);
    
    if(iter < 1)
    {
        printk( KERN_ERR "Error: Timeout waiting for VideoCore 2nd stage BootLoader verification\n " );
        return -EIO;
    }

    VC_DEBUG( BootTrace, "Sending '%s' to VideoCore...\n", vcvmcs_path );
    
    // load vcvmcs.bin
    if (( rc = vchost_sendfile(vcvmcs_path)) < 0 )
    {
        return rc;
    }

    VC_DEBUG( BootTrace, "Waiting for firmware verification..." );
    
    memset(&msg2, 0, sizeof(msg2));
    msg2.header = 0xAFE45251;
    msg2.id_size = (2 << 24) | (stage2_size & 0xffffff);
    msg2.footer = 0xF001BC4E;
    vchost_writemsg(&msg2, sizeof(msg2), 0);
    // finish
    vchost_writeparam(0);
    
    // poll 2727 for message 3 "boot rom received and verified stage 2 loader" 
    iter = 3000;
    do
    {
        memset(msg, 0, sizeof(msg));
        length = vchost_readmsg(msg, sizeof(msg), 0);
        if ( length > 0 )
        {
            VC_DEBUG( BootTrace, "Msg3 Length=%d msg[0..1]=0x%08X %08X\n", length, msg[0], msg[1]);
        }
        if(length == 32 && msg[0] ==0xAFE45251 && (msg[1] >> 24) == 3)
            break;
    } while(--iter);
    
    if(iter < 1)
    {
        printk( KERN_ERR "Error: Timeout waiting for VideoCore 2nd stage BootLoader verification\n " );
        return -EIO;
    }

    vchost_set_interface_for_run();

    VC_DEBUG( BootTrace, "VideoCore firmware booted successfully" );
    
    return 0;
}

/****************************************************************************
*
* Uninitialize VC03 driver
*
*
***************************************************************************/
int vc03_exit(vc03drv_t* drv)
{
  assert(drv);
  assert(drv->initialized == 1);

  // FIXME: uninit LCD

  //
  vc03_vchi_exit(drv);

  return 0;
}

/****************************************************************************
*
* Initialize VC03 driver
*
* mask:
*  bit0 -- initialize vc host port and services
*  bit1 --  initialize vc lcd and framebuffer
*
***************************************************************************/
int vc03_init(vc03drv_t* drv, int mask)
{
    int rc;

    if ( mask & (1 << 0))
    {
        if (( rc = vc03_vchi_init( drv )) != 0)
        {
            printk( KERN_ERR "vc03_init: VC03 interface initialization failed: %d\n", rc );
            return rc;
        }
        VC_DEBUG( Trace, "VC03 fifos initialized\n");
    }
    
    if(mask & (1 << 1))
    {
        if(( rc = lcd_init_2()) != 0 )
        {
            printk( KERN_ERR "vc03_init: VC03 lcd interface initialization failed: %d\n", rc );
            return rc;
        }
        VC_DEBUG( Trace, "VC03 lcd interface initialized\n");
    }
    
    /* start the frame forward threads, which is only being used by 2 way video conference calls */
    /* comment it out for the time being */
    //vc03_startfrmfwd_thds();
    
    return 0;
}

/*
 *
 */
int vc03_reset_bootfile(int delay_msec, const char* vmcs_path)
{
    int ret = 0;
    vc03drv_t* drv;

    drv = vc03drv_get(); 
    if(drv->initialized)
    {
        vc03_exit(drv);
    }
  
    vchost_reset();  
    if ( delay_msec > 0 )
    {
        vc_host_delay_msec( delay_msec );
    }

    if (( ret = vc03_bootfile(vcstage2_bin, vcstage2_size, vmcs_path)) != 0 )
    {
        printk( KERN_ERR "VC03 boot failed: %d.\n", ret );
        return ret;
    }

    // Delay for a small amount of time to allow the videocore to get the MPHI interface initialized before
    // we send down the initial CONNECT message.
    vchost_delay_msec( 500 );

    // initializez fifos and lcd framebuffer
    ret = vc03_init(drv, (1 << 0) | (1 << 1));
  
    return ret;
}

int vc03_reset_bootmem(int delay_msec)

{
  int ret=0;
#if 0
  vc03drv_t* drv;

  //
  drv = vc03drv_get(); 
  if(drv->initialized)
    {
      vc03_exit(drv);
    }
  //
  vchost_reset();  
  if(delay_msec > 0)
    {
      vc_host_delay_msec(delay_msec);
    }

  //
  ret = vc03_bootmem(vcstage2_bin, vcstage2_size, vcvmcs_bin, vcvmcs_size);

  if(0 != ret)
	{
	  printk( KERN_ERR "VC03 boot failed.\n" );
	  return -EFAULT;
	}

  //
   ret = vc03_init(drv, (1 << 0) | (1 << 1));
 #endif
  return ret;
}


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

#if defined( CONFIG_BCM_DEBUG_PAUSE )
  if ( debug_pause( "Press any key to abort VideoCore initialization" ))
  {
      return 1;
  }
#endif

  memset(vc03drv, 0, sizeof(vc03drv));

  // Setup the debug stuff using sysctl (because it's so simple)
  // and then symlink /proc/vc/debug to /proc/sys/vc/debug

  gSysCtlHeader = register_sysctl_table( gSysCtl );
  gProcVcDir = create_proc_entry( "vc", S_IFDIR | S_IRUGO | S_IXUGO, NULL );
#if VC_DEBUG_ENABLED
  gProcVcDebugDir = proc_symlink( "debug", gProcVcDir, "../sys/vc/debug" );
#endif

  if (( rc = register_chrdev( BCM_VC03_MAJOR, "vc03", &vc_fops )) < 0 )
    {
      printk( KERN_WARNING "VC03: register_chrdev failed for major %d\n", BCM_VC03_MAJOR );
      return rc;
    }

   vc03_procreset_init();
   vc03_proc_init();
   
#if USE_DMA   
   sema_init(&gDmaSem, 1);
#endif   

   if(0)
   if(0 != vc03_reset_bootmem(10))
     {
       printk(KERN_ALERT "Error: VC03 interface init failed\n");
       return -EFAULT;
     }

   printk(KERN_INFO "VC03 host port driver init OK.\n");

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

#if VC_DEBUG_ENABLED
    if ( gProcVcDebugDir != NULL )
    {
        remove_proc_entry( "debug", gProcVcDir );
    }
#endif
    if ( gProcVcDir != NULL )
    {
        remove_proc_entry( "vc", NULL );
    }
    if ( gSysCtlHeader != NULL )
    {
        unregister_sysctl_table( gSysCtlHeader );
    }

} // vc_exit_drv

module_init(vc_init_drv);
module_exit(vc_exit_drv);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("VCO2 Driver");
