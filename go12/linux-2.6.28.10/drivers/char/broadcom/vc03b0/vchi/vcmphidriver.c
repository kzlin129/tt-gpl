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

Project  :  VCFW
Module   :  Message-Passing Host Interface driver
File     :  $RCSfile: mphi.c,v $
Revision :  $Revision: #7 $
 
FILE DESCRIPTION

=============================================================================*/

#include <linux/broadcom/gpio.h>
#include <linux/broadcom/gpio_irq.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/control_service.h>
#include <linux/broadcom/vc03/vchi/mphi.h>
#include <linux/proc_fs.h>
#include <linux/module.h>

/******************************************************************************
  Register fields
 *****************************************************************************/

/******************************************************************************
  Local typedefs
 *****************************************************************************/
#define MPHIDRV_LOGLEVEL 1

const int gVcDebugMPHI = 0;
const int gVcDebug_TXRXlen = 0;
//const int gVcDebug_SID = ('T'<< 0) | ('S' << 8) | ('T' << 16) | ('D' << 24);
const int gVcDebug_SID = 0; // set to 0 to dump all services
const int gVcDebugStatsInterval = 0;   // in msec, set to 0 not to print stats, otherwise time in msec

#define VC03_MPHILOG_MAX (1024 * 64)

typedef enum 
  {
    RXTX_STOPPED = 0,
    RXTX_STOPPING = 1,
    RXTX_RUNNING = 2,

  } RXTX_STATE_T;

#define MPHISLOT_MAX 17

typedef struct mphi_slot
{
  uint32_t addr;
  uint32_t pos;
  uint32_t len;
  uint32_t id;
  uint8_t ctrl;
  MPHI_FLAGS_T  flags;

} mphi_slot_t;

typedef struct mphi_driver mphi_driver_t;

typedef struct 
{
  uint8_t refcnt;
  uint8_t channel;
  void (*event_callback)( const void *cb_data );
  const void *event_callback_data; 

  mphi_slot_t slotin[MPHISLOT_MAX];          // slots, pending for tx (or empty for rx). completed slot is removed from "slotin"
  uint32_t in_cnt, in_r, in_w;  
  mphi_slot_t slotout[MPHISLOT_MAX];         // completed slots
  uint32_t out_cnt, out_r, out_w;  
  struct semaphore fifolock;                
  
  mphi_driver_t* drv;

  // stats
  int msgcnt;
  int msgcnt_skipped;                        // skipepd count: no slot was available to read the message, skipped != 0 asks for optimization 
  uint64_t bytecnt;
  int st;
  
} MPHI_HANDLE_T;


struct mphi_driver
{
  MPHI_HANDLE_T mphi[MPHI_CHANNEL_MAX];
  struct semaphore buslock;                 // lock for tx/rx hw  
  OS_EVENTGROUP_T rxtxavail;
  int rxtx_state;

#ifdef MPHIDRV_LOGLEVEL
  char* mphilog;
  int   mphilog_rindex;
  int   mphilog_windex;
  struct semaphore mphilog_lock;
  struct proc_dir_entry* mphilog_proc;
#endif
  
};

mphi_driver_t mphidrv;

/******************************************************************************
 Extern functions
 *****************************************************************************/


/******************************************************************************
 Static functions
 *****************************************************************************/

//Routine used to initilise the driver
static int32_t mphi_init( void );

//Routine used to exit the driver
static int32_t mphi_exit( void );

//Routine to get the driver info
static int32_t mphi_info( const char **driver_name,
                          uint32_t *version_major,
                          uint32_t *version_minor,
                          DRIVER_FLAGS_T *flags );

//Routine used to open the driver (returns a handle)
static int32_t mphi_open( const MPHI_OPEN_T *params,
                          DRIVER_HANDLE_T *handle );

//Routine used to close the driver
static int32_t mphi_close( const DRIVER_HANDLE_T handle );

static int32_t  mphi_add_recv_slot( const DRIVER_HANDLE_T handle, const void *addr, uint32_t len, uint8_t slot_id );
static int32_t  mphi_out_queue_message( const DRIVER_HANDLE_T handle, uint8_t control, const void *addr, size_t len, uint8_t msg_id, const MPHI_FLAGS_T flags );

static int      mphi_slots_available( DRIVER_HANDLE_T handle );
static MPHI_EVENT_TYPE_T mphi_next_event( const DRIVER_HANDLE_T handle, uint8_t *slot_id, uint32_t *pos );

// Routine used to enable the logic and interrupts
static int32_t mphi_enable( const DRIVER_HANDLE_T handle );

// Set drive power (mA) and slew (0=high power, 1=slew, low power)
static int32_t mphi_set_power( const DRIVER_HANDLE_T handle, int drive_mA, int slew );

// Get address alignment requirement
static int mphi_alignment( const DRIVER_HANDLE_T handle );

// dump debug info to logging
static void     mphi_debug( const DRIVER_HANDLE_T handle );

// wait for I/O to finish
static void mphi_finish_io(int purge);


#ifdef MPHIDRV_LOGLEVEL
int mphi_dumpmsg(mphi_driver_t* drv, const char* str, mphi_slot_t* slotptr);
#else
#define mphi_dumpmsg(drv, str, slotptr) do {(void)drv; (void)str; (void)slotptr;} while (0)
#endif

#ifdef MPHIDRV_LOGLEVEL

int mphi_omxlog_procread(char *buf, char **start, off_t offset, int count, int *eof, void *data);
int mphi_logread(struct mphi_driver* drv, char* dest, int maxbytes);
int mphi_logadd(struct mphi_driver* drv, char* str, int len, int lock);

#define MPHI_LOGSTR(drv, str, len)                     \
	({                                                \
       mphi_logadd(drv, str, len, 1);                    \
	}) 

#define MPHI_LOGSTR_TS(drv, fmt, args...)                      \
	({                                                \
	   char str[255];                                 \
       int len;                                       \
       len = sprintf(str, "[t=%08u]" fmt,  timer_get_tick_count(),  ## args); \
       mphi_logadd(drv, str, len, 1);                    \
	}) 

#define MPHI_LOGSTR_IRQTS(drv, fmt, args...)     \
	({                                                \
	   char str[255];                                 \
       int len;                                       \
       len = sprintf(str, "[t=%08u]" fmt,  timer_get_tick_count(),  ## args); \
       mphi_logadd(drv, str, len, 0);                    \
	}) 

#else    // MPHIDRV_LOGLEVEL

#define MPHI_LOGSTR(drv, str, len)                          
#define MPHI_LOGSTR_TS(drv, fmt, args...)                      
#define MPHI_LOGSTR_IRQTS(drv, fmt, args...)                      

#endif  // MPHIDRV_LOGLEVEL




#if (CFG_GLOBAL_CHIP == BCM2153)
extern uint32_t get_gptimer_usec(void);
#define timer_get_tick_count get_gptimer_usec
#endif

extern u32 timer_get_tick_count( void );

//logging
static mphi_slot_t m_irq_slot; 
static mphi_slot_t m_out_slot;


/******************************************************************************
 Static data
 *****************************************************************************/

//The driver function table
static const MPHI_DRIVER_T mphi_func_table =
{
   // common driver api
   &mphi_init,
   &mphi_exit,
   &mphi_info,
   &mphi_open,
   &mphi_close,

   // mphi-specific api
   &mphi_add_recv_slot,
   &mphi_out_queue_message,
   &mphi_slots_available,
   &mphi_next_event,
   &mphi_enable,
   &mphi_set_power,
   &mphi_alignment,
   &mphi_debug,
   &mphi_finish_io,
};

static const MPHI_DRIVER_T *mphi_func_table_ptr = &mphi_func_table;


/******************************************************************************
 Global Functions
 *****************************************************************************/

/* ----------------------------------------------------------------------
 * return pointer to the mphi driver function table
 * -------------------------------------------------------------------- */
const MPHI_DRIVER_T * mphi_get_func_table( void )
{
  return mphi_func_table_ptr;
}

/*
 *
 */
inline void lock_mphififo(MPHI_HANDLE_T* mphi)
{
  down_interruptible(&mphi->fifolock);
}

inline void unlock_mphififo(MPHI_HANDLE_T* mphi)
{
   up(&mphi->fifolock);
}



static inline void lock_bus(void)
{
   down(&mphidrv.buslock);
}

static inline void unlock_bus(void)
{
   up(&mphidrv.buslock);
}



static mphi_slot_t* get_slotin_write(MPHI_HANDLE_T* mphi)
{
   // writes don't overrun reads
   if(mphi->in_w + 1 + MPHISLOT_MAX == mphi->in_r + MPHISLOT_MAX)
   {
      //      VC_DEBUG(Trace, "No in_w slots: in_cnt=%d in_r=%d in_w=%d\n", mphi->in_cnt, mphi->in_r, mphi->in_w);
      return NULL;
   }

   mphi->in_w = (mphi->in_w + 1) % MPHISLOT_MAX;
   return mphi->slotin + mphi->in_w;  
}

static int slotin_readavail(MPHI_HANDLE_T* mphi)
{
   return !(mphi->in_r == mphi->in_w);
}

static mphi_slot_t* get_slotin_read(MPHI_HANDLE_T* mphi)
{
   if(mphi->in_r == mphi->in_w)
   {
      //VC_DEBUG(Trace, "No in_r slots, in_cnt=%d in_r=%d in_w=%d\n", mphi->in_cnt, mphi->in_r, mphi->in_w);
      return NULL;
   }

   mphi->in_r = (mphi->in_r + 1) % MPHISLOT_MAX;
   return mphi->slotin + mphi->in_r;  
}

static mphi_slot_t* get_slotout_write(MPHI_HANDLE_T* mphi)
{
   if(mphi->out_w + 1 + MPHISLOT_MAX == mphi->out_r + MPHISLOT_MAX)
   {
      //VC_DEBUG(Trace, "No out_w slots: in_cnt=%d in_r=%d in_w=%d\n", mphi->out_cnt, mphi->out_r, mphi->out_w);
      return NULL;
   }

   mphi->out_w = (mphi->out_w + 1) % MPHISLOT_MAX;
   return mphi->slotout + mphi->out_w;  
}

static mphi_slot_t* get_slotout_read(MPHI_HANDLE_T* mphi)
{
   if(mphi->out_r == mphi->out_w)
   {
      //VC_DEBUG(Trace, "No out_r slots: in_cnt=%d in_r=%d in_w=%d\n", mphi->out_cnt, mphi->out_r, mphi->out_w);
      return NULL;
   }

   mphi->out_r = (mphi->out_r + 1) % MPHISLOT_MAX;
   return mphi->slotout + mphi->out_r;  
}

static int add_slotin(MPHI_HANDLE_T* mphi, mphi_slot_t* slotin)
{
   mphi_slot_t* slot;

   assert(mphi); 
   lock_mphififo(mphi);
   slot = get_slotin_write(mphi);
   if(NULL == slot)
   {
      unlock_mphififo(mphi);
      //    VC_DEBUG(Trace, "error: no free slots\n");
      return -1;
   }
   memcpy(slot, slotin, sizeof(*slot));
   ++mphi->in_cnt;
   unlock_mphififo(mphi);

   // VC_DEBUG(Trace, "id=0x%02X addr=0x%08X len=0x%08X chan=%02d, in_cnt=%d\n", 
   // 	   slot->id, slot->addr, slot->len, mphi->channel, mphi->in_cnt);
  
   return 0;
}

static int del_slotin(MPHI_HANDLE_T* mphi, mphi_slot_t* slotin)
{
   mphi_slot_t* slot;
  
   memset(slotin, 0, sizeof(*slotin));
   lock_mphififo(mphi);
   slot = get_slotin_read(mphi);
   if(NULL == slot)
   {
      unlock_mphififo(mphi);
      // VC_DEBUG(Trace, "error: no avail slots\n");
      //   VC_DEBUG(Trace, "chan=%02d, in_cnt=%d\n", mphi->channel, mphi->in_cnt);
      return -1;
   }
  
   memcpy(slotin, slot, sizeof(*slotin));
   --mphi->in_cnt;
   unlock_mphififo(mphi);

   //  VC_DEBUG(Trace, "id=0x%02X addr=0x%08X len=0x%08X chan=%02d, in_cnt=%d\n", 
   // 	   slot->id, slot->addr, slot->len, mphi->channel, mphi->in_cnt);
  
  return 0;
}

static int add_slotout(MPHI_HANDLE_T* mphi, mphi_slot_t* slotout)
{
   mphi_slot_t* slot;

   assert(mphi); 
   lock_mphififo(mphi);
   slot = get_slotout_write(mphi);
   if(NULL == slot)
   {
      unlock_mphififo(mphi);
      //    VC_DEBUG(Trace, "error: no free slots\n");
      return -1;
   }
   memcpy(slot, slotout, sizeof(*slot));
   ++mphi->out_cnt;
   unlock_mphififo(mphi);

   // VC_DEBUG(Trace, "id=0x%02X addr=0x%08X len=0x%08X chan=%02d, out_cnt=%d\n", 
   //	   slot->id, slot->addr, slot->len, mphi->channel, mphi->out_cnt);
  
   return 0;
}

static int del_slotout(MPHI_HANDLE_T* mphi, mphi_slot_t* slotout)
{
   mphi_slot_t* slot;
  
   memset(slotout, 0, sizeof(*slotout));
   lock_mphififo(mphi);
   slot = get_slotout_read(mphi);
   if(NULL == slot)
   {
      unlock_mphififo(mphi);
      //     VC_DEBUG(Trace, "error: no avail slots\n");
      return -1;
   }
  
   memcpy(slotout, slot, sizeof(*slotout));
   assert(mphi->out_cnt);
   --mphi->out_cnt;
   unlock_mphififo(mphi);

   // VC_DEBUG(Trace, "id=0x%02X addr=0x%08X len=0x%08X chan=%02d, in_cnt=%d\n", 
   // 	   slot->id, slot->addr, slot->len, mphi->channel, mphi->out_cnt);
  
   return 0;
}


/*
 * Wait until current output operation completes.
 * Optionally purge queued output.
 * XXX
 * XXX May want to purge input slots also?
 * XXX
 */
static void mphi_finish_io(int purge)
{
   if (purge)
   {
      // Purge all slots queued for output.
      mphi_slot_t slot;

      MPHI_HANDLE_T* mphi = (MPHI_HANDLE_T*)(mphidrv.mphi + MPHI_CHANNEL_OUT);
      while(0 == del_slotin(mphi, &slot));
   }

   // Wait for any pending output to finish.
   lock_bus();
   unlock_bus();
}


/*
 *
 */
irqreturn_t vc_gpioirq(int irq, void *devId)  
{
   mphi_driver_t* mphidrv;

   m_irq_slot.ctrl = 0;  //data
   m_irq_slot.len = timer_get_tick_count();
   
   mphidrv = (mphi_driver_t*) devId;
   assert(mphidrv);
   if(vchost_gpioirq_handle(irq, devId))
   {
      os_event_signal(&mphidrv->rxtxavail, 0);
   }
   else
   {
      //	  VC_DEBUG(Trace, "vc_gpioirq: ignoring gpio irq\n");
   }
  
  return IRQ_HANDLED;
}
	  

static int mphi_pumptx(mphi_driver_t* mphidrv)
{
   MPHI_HANDLE_T* mphi;
   mphi_slot_t slot;
   int ret;
   int8_t   msgType;

   mphi = (MPHI_HANDLE_T*)(mphidrv->mphi + MPHI_CHANNEL_OUT);
   ret = del_slotin(mphi, &slot);
   if(ret != 0)
   { 
      // no free slots
      return -1;
   }      
  
   mphi_dumpmsg(mphidrv, "TX", &slot);  

   msgType = !slot.ctrl;
   if (( slot.flags & MPHI_FLAGS_TERMINATE_DMA ) != 0 )
   {
       msgType |= VC_HOSTPORT_PARAM_TERMINATE_DMA;
   }

   // transmit
   lock_bus();
   vchost_writemsg((void*)slot.addr, slot.len, msgType);
   vchost_writeparam(0);
   unlock_bus();
  
   ++mphi->msgcnt;
   mphi->bytecnt += slot.len;

   // put it to "out" fifo 
   add_slotout(mphi, &slot);
  
   if(mphi->event_callback) 
   {
      // signal above that something happened
      mphi->event_callback(mphi->event_callback_data);	  	      
   }
      
  
  return 0;
}

static int mphi_pumprx(mphi_driver_t* mphidrv)
{
   mphi_slot_t slot;
   MPHI_HANDLE_T* mphi;
   int ret;
   int8_t ctrl;
 
   assert(mphidrv);

   // check if message is available for read
   ctrl = vchost_msgavail();
   if(ctrl < 0)
      return -1; 
  
   // get correct driver state
   assert(ctrl < 2);
   mphi = ctrl ? (mphidrv->mphi + MPHI_CHANNEL_IN_CONTROL) : (mphidrv->mphi + MPHI_CHANNEL_IN_DATA);
   assert(mphi);

   // get next available slot
   ret = del_slotin(mphi, &slot);
   if(ret != 0)
   {
      VC_DEBUG(MPHI, "error: no empty slot, ch=%d\n", mphi->channel);
      ++mphi->msgcnt_skipped;

      return -1;
   }

   // read the message
   lock_bus();
   memset((void*)slot.addr, 0, slot.len);
   slot.len = vchost_readmsg((void*)slot.addr, slot.len, &(slot.ctrl));
   unlock_bus();

   // sanity check 
   assert(ctrl == slot.ctrl);
   if(slot.len <= 0)
   {
      assert(0); // message should've been available
      return -1;
   }

   // update message stats
   ++mphi->msgcnt;
   mphi->bytecnt += slot.len;
   add_slotout(mphi, &slot);      
  
   mphi_dumpmsg(mphidrv, "RX", &slot);

   if(mphi->event_callback) 
   {      
      // signal above that something happened
      mphi->event_callback(mphi->event_callback_data);	  	      
   }
      
   return 0;
}


static const char* mphi_ch2str(int ch)
{
   const char* str[MPHI_CHANNEL_MAX] = 
   {
      "CH_OUT",
      "CH_IN_CONTROL", 
      "CH_IN_DATA",
      "CH_IN_HOST_SPEC",
   };
   return str[ch % MPHI_CHANNEL_MAX];
}

static int mphi_dumpstats(mphi_driver_t* mphidrv)
{
   int i;
   MPHI_HANDLE_T* mphi;
   uint32_t dt, et, kbyte, mbyte_sec;
 
   et = jiffies_to_msecs(jiffies);
   for(i = 0; i < MPHI_CHANNEL_MAX - 1; i++)
   {
      mphi = mphidrv->mphi + i;

      if(et < mphi->st)
      {
	  printk("MPHI ch=%d(%14s) stats reset\n", i, mphi_ch2str(i));
	  mphi->msgcnt = 0; 
	  mphi->msgcnt_skipped = 0; 
	  mphi->bytecnt = 0;
	  mphi->st = jiffies_to_msecs(jiffies);
	  continue;
      }

      dt = et - mphi->st;

      if(0 == dt)
    	return 0;

      kbyte = mphi->bytecnt / 1024 ;      
      mbyte_sec = (kbyte * 1000) / dt;
      if(mbyte_sec >= 1024)
      {
	  mbyte_sec /= 1024;
	  printk("MPHI ch=%d(%14s) msg cnt=%6d skipped=%4d %8d msg/sec %d MByte/sec \n", 
		 i, mphi_ch2str(i), 
		 mphi->msgcnt, 
		 mphi->msgcnt_skipped, 
		 (mphi->msgcnt * 1000) / dt, 
		 mbyte_sec);
      }
      else
      {
	  printk("MPHI ch=%d(%14s) msg cnt=%6d skipped=%4d %8d msg/sec %d KByte/sec \n", 
		 i, mphi_ch2str(i), 
		 mphi->msgcnt, 
		 mphi->msgcnt_skipped, 
		 (mphi->msgcnt * 1000) / dt, 
		 mbyte_sec);
      }
   }
						
   printk("\n");

   return 0;
}

/*					    
 *
 */
const char* mphi_ctrlcmd2str(uint32_t cmd)
{
   static const char* str[5] = 
   {
      "CONNECT",             // used to signal opening of the comms
      "SERVER_AVAILABLE",    // queries if a server is available
      "BULK_TRANSFER_RX",    // informs the other side about an item added to the RX BULK FIFO
      "XON",                 // restore comms on a particular service
      "XOFF",                // pause comms on a particular service
   };

   return str[cmd %5];
}

//copy from control_service.c
typedef enum {
   CONNECT,               // used to signal opening of the comms
   SERVER_AVAILABLE,      // queries if a server is available
   BULK_TRANSFER_RX,      // informs the other side about an item added to the RX BULK FIFO
   XON,                   // restore comms on a particular service
   XOFF,                  // pause comms on a particular service
   SERVER_AVAILABLE_REPLY // response to a SERVER_AVAILABLE query
} VCHI_COMMANDS_T;

#ifdef MPHIDRV_LOGLEVEL
int mphi_dumpmsg(mphi_driver_t* drv, const char* prefix, mphi_slot_t* slotptr)
{
   const uint32_t ctrlid = MAKE_FOURCC("LRTC");
   uint8_t* addr =  (uint8_t*)slotptr->addr;
   uint32_t cmd, paylen, i;
   static int cnt = 0;
   char str[0x100];
   int len;

   if(slotptr->ctrl && gVcDebug_SID != 0)
	 // if(gVcDebug_SID != 0)
   {
      if(*(uint32_t*) (addr + 4) != gVcDebug_SID)
      {
	  return 0;
      }
   }

   ++cnt;
   len = 0;
   len += sprintf(str + len, "[%04d %08u] %s %c tlen=%u ", 
  		         cnt,  timer_get_tick_count(),  
  		         prefix, slotptr->ctrl ? 'c' : 'd', slotptr->len);
   if(slotptr->ctrl)
   {
      paylen = *(uint16_t*)(addr + 0xE);
      len += sprintf(str + len, "sid=%c%c%c%c slotcnt=0x%04X paylen=0x%04X ",
		     addr[4], addr[5], addr[6], addr[7],
		     *(uint16_t*)(addr + 0xC),
    	             paylen);
      if(*(uint32_t*)(addr + 4) == ctrlid && paylen > 0)
      {
	  addr += 0x10;
	  cmd = *(uint32_t*)(addr);
	  switch(cmd)
	  {
	    case BULK_TRANSFER_RX:
	    {
               len += sprintf(str + len, "cmd=%s(%d) sid=%c%c%c%c(%02X%02X%02X%02X) sz=0x%08X",
			       mphi_ctrlcmd2str(cmd), cmd,
			       addr[4], addr[5], addr[6], addr[7],
			       addr[4], addr[5], addr[6], addr[7],
			       *(uint32_t*)(addr + 0x8));		     
               break;
	    }
	    case XON:
	    case XOFF:
	    {
               len += sprintf(str + len, "cmd=%s(%d) sid=%c%c%c%c",
			       mphi_ctrlcmd2str(cmd), cmd,
			       addr[4], addr[5], addr[6], addr[7]
			       );
               break;
	    }
	    default:
	    {
               len += sprintf(str + len, "cmd=%s(%d) 0x%08X 0x%08X 0x%08X",
			       mphi_ctrlcmd2str(cmd), cmd,
			       *(uint32_t*)(addr + 0x4),
			       *(uint32_t*)(addr + 0x8),
			       *(uint32_t*)(addr + 0xC));	     
               break;
           }
         }
      }
      else
      {
         addr += 0x10;
         for(i = 0; i < paylen && i < 0x18; i +=2)
         {
             len += sprintf(str + len, "%04X ", *(uint16_t*) addr);
             addr += 2;
         }
      }
   }

   len += sprintf(str + len, "\n");

   if ( gVcDebugMphiDumpMsg )
   {
       printk( KERN_INFO "%s", str );
   }

   MPHI_LOGSTR(drv, str, len);
  
   return 0;	 
}
#endif

static int mphi_rxtx_task( void *data )
{
   int event;
   int timeout;
   mphi_driver_t* mphidrv;
   int ret;
   uint32_t st, et;  
   
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26) // Don't set sched priority in newer kernels.
   {
       struct sched_param param;

       param.sched_priority = 95;
       if ( sched_setscheduler(current, SCHED_FIFO, &param) == 0 )
       {
           VC_DEBUG(MPHI, "[%s] pri changed to %d\n", __FUNCTION__, param.sched_priority);
       }
       else
       {
           VC_DEBUG(MPHI, "error\n");
       }
   }
#endif
   
   mphidrv = (mphi_driver_t*) data;
   //vchost_gpioirq_setup();
   ret = -1;
   st = jiffies_to_msecs(jiffies);
  
   assert(mphidrv->rxtx_state == RXTX_STOPPED);
   mphidrv->rxtx_state = RXTX_RUNNING;
   while(mphidrv->rxtx_state == RXTX_RUNNING) 
   {
      // wait for a message, wait on mphi rx status only if nothing to receive 
      if(ret < 0)
      {
	  timeout = os_eventgroup_retrieve_to(&mphidrv->rxtxavail, &event, 1);
      }

      if (event & 1)  //recv event
        mphi_dumpmsg(mphidrv,"IR", &m_irq_slot);      

      if (event & 2)
        mphi_dumpmsg(mphidrv, "TQ", &m_out_slot);

      
      // transmit, receive
      ret = mphi_pumptx(mphidrv);
      ret = mphi_pumprx(mphidrv);

      // print stats
      et = jiffies_to_msecs(jiffies);
      et = (et >= st) ? (et - st) : (~0 - st + et); 
      if(gVcDebugStatsInterval > 0 && et > gVcDebugStatsInterval)
      {
    	  mphi_dumpstats(mphidrv);
	  st = jiffies_to_msecs(jiffies);
      }
   }

   mphidrv->rxtx_state = RXTX_STOPPED;
   VC_DEBUG(MPHI, "exit\n"); 
   return 0;
} 


/* ----------------------------------------------------------------------
 * initialise the mphi driver
 * this is part of the common driver api
 * return 0 on success; all other values are failures
 * -------------------------------------------------------------------- */
static int32_t mphi_init( void )
{
   int i, ret;
   struct task_struct* t;

   memset(&mphidrv, 0, sizeof(mphidrv));
   for(i = 0; i < MPHI_CHANNEL_MAX; i++)
   {
      mphidrv.mphi[i].channel = i;
      mphidrv.mphi[i].drv = &mphidrv;
      sema_init(&mphidrv.mphi[i].fifolock, 1);
   }

   sema_init(&mphidrv.buslock, 1);
   os_eventgroup_create(&mphidrv.rxtxavail, "rxtxavail");

   t = kthread_run(mphi_rxtx_task,  &mphidrv, "mphi_rxtx_task");
   if(IS_ERR(t))
   {
      VC_DEBUG(MPHI, "Error starting mphi_rx_task\n");
      return -1;
   }

#ifdef MPHIDRV_LOGLEVEL
   sema_init(&mphidrv.mphilog_lock, 1);
   mphidrv.mphilog_rindex = 0;
   mphidrv.mphilog_windex = 0;
   mphidrv.mphilog = vmalloc(VC03_MPHILOG_MAX);
   memset(mphidrv.mphilog, 0, VC03_MPHILOG_MAX);

   mphidrv.mphilog_proc = create_proc_entry("VC03_mphilog", 0644, NULL);
	  
   if (mphidrv.mphilog_proc == NULL)
   {
      remove_proc_entry("VC03_mphilog", NULL);
      printk(KERN_ALERT "could not initialize /proc/VC03_mphilog");
   }
   else
   {
      mphidrv.mphilog_proc->read_proc  = mphi_omxlog_procread;
      mphidrv.mphilog_proc->write_proc = NULL;
      mphidrv.mphilog_proc->owner	= THIS_MODULE;
      mphidrv.mphilog_proc->mode 	= S_IFREG | S_IRUGO;
      mphidrv.mphilog_proc->uid		= 0;
      mphidrv.mphilog_proc->gid		= 0;
      mphidrv.mphilog_proc->size 	= 37;
   }
#endif //MPHIDRV_LOGLEVEL

   if (( ret = vchost_request_irq( vc_gpioirq, &mphidrv )) != 0 )
   {
       return ret;
   }

   return 0;
}


/* ----------------------------------------------------------------------
 * exit the mphi driver
 * this is part of the common driver api
 * return 0 on success; all other values are failures
 * -------------------------------------------------------------------- */
static int32_t mphi_exit( void )
{
   int iter;
   VC_DEBUG(MPHI, "\n");

   mphidrv.rxtx_state = RXTX_STOPPING;

   vchost_free_irq( &mphidrv );

   iter = 100;
   while(iter > 0 && mphidrv.rxtx_state != RXTX_STOPPED)
   {
      --iter;
      os_sleep(10);
   }
   assert(mphidrv.rxtx_state == RXTX_STOPPED);
   memset(&mphidrv, 0, sizeof(mphidrv));
   return 0;
}


/* ----------------------------------------------------------------------
 * get the mphi driver info
 * this is part of the common driver api
 * return 0 on success; all other values are failures
 * -------------------------------------------------------------------- */
static int32_t mphi_info( const char **driver_name,
			  uint32_t *version_major,
			  uint32_t *version_minor,
			  DRIVER_FLAGS_T *flags )
{
   int32_t success = -1; // fail by default
  
   VC_DEBUG(MPHI, "\n");
    // return the driver name
   if ( driver_name && version_major && version_minor && flags ) {
    
      *driver_name = "mphi";
      *version_major = 0;
      *version_minor = 1;
      *flags = 0;
    
      // success!
      success = 0;
   }
  
   return success;
}


/* ----------------------------------------------------------------------
 * open the driver and obtain a handle
 * this is part of the common driver api
 * return 0 on success; all other values are failures
 * -------------------------------------------------------------------- */
static int32_t mphi_open( const MPHI_OPEN_T *params, DRIVER_HANDLE_T *handle )
{
   int channel;
   MPHI_HANDLE_T* mphi;

   VC_DEBUG(MPHI, "channel=%d\n", params->channel);
   channel = params->channel;  
   if(channel >= MPHI_CHANNEL_MAX)    
   {
      assert(0);
      return -1;
   }
  
   mphi = mphidrv.mphi + channel;
   ++mphi->refcnt;
  
   if ( mphi->event_callback || mphi->event_callback_data ) 
   {
       assert(mphi->event_callback == params->callback); // can have only one
       assert(mphi->event_callback_data == params->callback_data);
   } 
   else 
   {
       mphi->event_callback = params->callback;
       mphi->event_callback_data = params->callback_data;
   }
  
   //
   mphi->msgcnt = 0;
   mphi->msgcnt_skipped = 0;
   mphi->bytecnt = 0;
   mphi->st = jiffies_to_msecs(jiffies);
   *handle = (DRIVER_HANDLE_T) mphi;

   return 0;
}


/* ----------------------------------------------------------------------
 * close a handle to the driver
 * this is part of the common driver api
 *
 * return 0 on success; all other values are failures
 * -------------------------------------------------------------------- */
static int32_t mphi_close( const DRIVER_HANDLE_T handle )
{
   (void)handle;
   VC_DEBUG(MPHI, "\n");
   return 0;
}
/* ----------------------------------------------------------------------
 * each of the two input channels has its own dma fifo, each
 * containing a number of slots.  as incoming messages are received,
 * they will be placed in the next available slot from this fifo.
 *
 * this routine tries to fill in a slot with the given address/len
 * conbination.  if there are no free slots, this routine returns
 * immediately with MPHI_ERROR_FIFO_FULL.
 *
 * when a message is received, if no entry is available in the FIFO,
 * the message is dropped, and a MPHI_EVENT_IN_MESSAGE_DISCARDED
 * is generated.
 *
 * if a DMA entry is available, then the message is placed into memory
 * and MPHI_EVENT_IN_DMA_COMPLETED is generated.
 * -------------------------------------------------------------------- */
static int32_t mphi_add_recv_slot( const DRIVER_HANDLE_T handle, const void *addr, uint32_t len, uint8_t slot_id )
{
   int ret;
   MPHI_HANDLE_T *mphi;
   mphi_slot_t slot;

   mphi = (MPHI_HANDLE_T *) handle;
   slot.addr = (uint32_t)addr;
   slot.len = len;
   slot.id = slot_id;
   slot.pos = 0;
   slot.ctrl = 0;
   slot.flags = MPHI_FLAGS_NONE;

   ret = add_slotin((MPHI_HANDLE_T*)handle, &slot);

   //VC_DEBUG(Trace, "ch=%d len=%d id=%d\n", mphi->channel, len, slot_id);
   if(0 != ret)
   {
      return MPHI_ERROR_FIFO_FULL;
   }
 
   return 0;
}


/* ----------------------------------------------------------------------
 * queue a message for transmission.  the two output channels share a
 * single fifo.
 *
 * if the output fifo is full, this routine returns immediately with
 * MPHI_ERROR_FIFO_FULL.
 *
 * when the message has been fully read from memory,
 * MPHI_EVENT_OUT_DMA_COMPLETED is generated.  note that this does
 * not mean that the host has read the message; the message is placed
 * in a message fifo and held until the host responds to its interrupt
 * and issues the appropriate number of read cycles.
 * -------------------------------------------------------------------- */
static int32_t mphi_out_queue_message(const DRIVER_HANDLE_T handle, uint8_t control, 
				      const void *addr, size_t len, uint8_t slot_id, const MPHI_FLAGS_T flags )
{
   int ret;
   MPHI_HANDLE_T *mphi;
   mphi_slot_t slot;

   mphi = (MPHI_HANDLE_T *) handle;
   slot.addr = (uint32_t)addr;
   slot.len = len;
   slot.id = slot_id;
   slot.ctrl = control;
   slot.pos = 0;
   slot.flags = flags;

   if(1)
   {
      //mphi_dumpmsg(mphi->drv, "TQ", &slot);
     m_out_slot.ctrl = 0;  //data
     m_out_slot.len = timer_get_tick_count();
   }

   ret = add_slotin((MPHI_HANDLE_T*)handle, &slot);
   //zrl
   if (ret==0)
    {
    //VC_DEBUG(Trace, "mphi_out_queue_message slot\n");
    os_event_signal(&(mphi->drv->rxtxavail), 1); //0
    }

   if(0 != ret)
   {
      return MPHI_ERROR_FIFO_FULL;
   }
 
   return 0;
}

/* ----------------------------------------------------------------------
 * how many free slots are there on this channel?
 *
 * the first function is publically exported via the ->ops table
 *
 * the other two functions are internal
 * -------------------------------------------------------------------- */
static int mphi_slots_available( DRIVER_HANDLE_T handle )
{
   MPHI_HANDLE_T *mphi = (MPHI_HANDLE_T *) handle;
  
   assert(mphi);
   return MPHISLOT_MAX - (mphi->out_cnt + mphi->in_cnt + 1);
}


/* ----------------------------------------------------------------------
 * look at stored copy of mphi interrupt status register, and return
 * the next event that is indicated.
 *
 * re-enable interrupts if no more events.
 *
 * intended to be called from hisr context: keep calling this function
 * until it returns MPHI_EVENT_NONE (at which point, mphi interrupts
 * will be re-enabled, below)
 *
 * !!!note this is no longer called from the HISR context, but from the
 * slot handling task instead!!! this is of particular interest if you
 * are writing a host-side port of VCHI and are tempted to assume
 * that synchronous LISR-like-entity -> HISR implies synchronous
 * LISR-like-entity -> mphi_next_event
 * -------------------------------------------------------------------- */
static MPHI_EVENT_TYPE_T mphi_next_event( const DRIVER_HANDLE_T handle, uint8_t *slot_id, uint32_t *pos )
{
   int i;
   mphi_slot_t slot;
   MPHI_HANDLE_T *mphi;
   int ret;
   mphi_driver_t* drv;
  
   mphi = (MPHI_HANDLE_T *) handle;  
   assert(mphi);
   drv = mphi->drv;
   assert(drv);

   for(i = 0; i < MPHI_CHANNEL_MAX; i++)
   {
      mphi = drv->mphi + i;
      ret = del_slotout(mphi, &slot);
      if(ret == 0)
      {
	  break;
      }
   }
  
   if(i == MPHI_CHANNEL_MAX)
      return MPHI_EVENT_NONE;
  
   mphi = drv->mphi + i;
   assert(mphi->channel == i);

   // 
   *slot_id = slot.id;
   *pos = slot.len;

   switch(mphi->channel)
   {
    case MPHI_CHANNEL_OUT:
      return MPHI_EVENT_OUT_DMA_COMPLETED;

    case MPHI_CHANNEL_IN_CONTROL:
      return MPHI_EVENT_IN_CONTROL_DMA_COMPLETED;

    case MPHI_CHANNEL_IN_DATA:
      return MPHI_EVENT_IN_DATA_DMA_COMPLETED;
   }
 
   return MPHI_EVENT_NONE;
}


/* ----------------------------------------------------------------------
 * Once we have primed the DMA FIFOs we can turn on the interrupts.
 * The MPHI does not have a 'transfer enable' so the only thing we can
 * do is enable the interrupts.  This may result in an initial (spurious)
 * discarded data interrupt etc.
 *
 * return 0 on success, non-0 otherwise
 * -------------------------------------------------------------------- */
static int32_t mphi_enable( const DRIVER_HANDLE_T handle )
{
   (void)handle;
   return 0;
}


/* ----------------------------------------------------------------------
 * MPHI peripheral defaults to a fairly low power state (4mA, slew on).
 * Expose an API to speed it up.
 *
 * return 0 on success, non-0 otherwise
 * -------------------------------------------------------------------- */
static int32_t mphi_set_power( const DRIVER_HANDLE_T handle, int drive_mA, int slew )
{
   (void)handle;
   (void) drive_mA;
   (void)slew;
   return 0;
}


/* ----------------------------------------------------------------------
 * all addresses need to be 16-byte aligned for the VC3 MPHI peripheral
 * -------------------------------------------------------------------- */
static int
mphi_alignment( const DRIVER_HANDLE_T handle )
{
   return 16;
}


/* ----------------------------------------------------------------------
 * dump debug info
 * -------------------------------------------------------------------- */
void mphi_debug( const DRIVER_HANDLE_T handle )
{
   (void)handle;
}

#ifdef MPHIDRV_LOGLEVEL


int mphi_logadd(struct mphi_driver* drv, char* str, int len, int lock)
{
   int nbytes;
	
   if(lock)
      if(down_interruptible(&drv->mphilog_lock))
         return -1;

   nbytes = min(VC03_MPHILOG_MAX - drv->mphilog_windex, len);
   memcpy(drv->mphilog + drv->mphilog_windex, str, nbytes);

   if(drv->mphilog_windex < drv->mphilog_rindex)
   {
      drv->mphilog_windex += nbytes;
      if(drv->mphilog_windex >= (drv->mphilog_rindex - 1))		
      {
         drv->mphilog_rindex = (drv->mphilog_windex + 1) % VC03_MPHILOG_MAX;
      }
   }
   else
   {	  
         drv->mphilog_windex += nbytes;
   }
  
   if(nbytes < len)
   {
      vc_assert(drv->mphilog_windex >= VC03_MPHILOG_MAX);
      memcpy(drv->mphilog + 0, str + nbytes, len - nbytes);
      drv->mphilog_windex = len - nbytes;
      drv->mphilog_rindex = max(drv->mphilog_rindex, drv->mphilog_windex + 1);
   }

   if(lock)
      up(&drv->mphilog_lock);

   return 0;
	  
}


int mphi_logread(struct mphi_driver* drv, char* dest, int maxbytes)
{
   char* src;
   int bytecount;
  
   if(down_interruptible(&drv->mphilog_lock))
	return 0;

   // calc offset
   src = drv->mphilog + drv->mphilog_rindex;
   if(drv->mphilog_rindex  <= drv->mphilog_windex)
   {
      bytecount = drv->mphilog_windex - drv->mphilog_rindex - 1;
   }
   else
   {
      bytecount = VC03_MPHILOG_MAX - drv->mphilog_rindex;
   }

   // copy
   bytecount = min(bytecount, maxbytes);
   memcpy(dest, drv->mphilog + drv->mphilog_rindex, bytecount);

   // advance read index
   drv->mphilog_rindex += bytecount;
   if( VC03_MPHILOG_MAX == drv->mphilog_rindex)
   {
      drv->mphilog_rindex = 0;
   }

   //
   up(&drv->mphilog_lock);
   return bytecount;
}

 
int mphi_omxlog_procread(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
   int len = 0;
   int ret;

   *start = buf;// + offset;
   while(len < count)
   {
      ret = mphi_logread(&mphidrv, buf + len /*+ offset*/, count - len);
      if(ret < 1)
      {
         break;
      }
      len += ret;
   }

   // buffer is empty
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

#endif //#ifdef MPHIDRV_LOGLEVEL

