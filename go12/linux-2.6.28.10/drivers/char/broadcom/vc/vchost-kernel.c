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

/******************************************************************************

Notes:

the vc_host_read_consecutive and vc_host_write consecutive functions were
profiled on MMC hardware. It was found that the only optimisation that helped
was to read/write the data on the host using indicies as into an array (instead
of have a pointer that is incremented after every r/w). The other optimisations
that were tried include:

 + unrolling loops

 + writing larger blocks of data before checking the FIFO status (via the
   threshold mechanism) to check for sufficient space being avaible.

These are platform dependant, and there is likely to be some mileage in trying
these optimisations on other platforms.

******************************************************************************/

#include <linux/version.h>
#include <linux/broadcom/timer.h>
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/knllog.h>

#include <linux/broadcom/hw.h>
#include <linux/kernel.h>
#include <linux/broadcom/vc.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include <linux/vmalloc.h>

#include <linux/dma-mapping.h>
#include <linux/elf.h>

#include "vchost.h"
#include "vchost_int.h"
#include <linux/broadcom/vchost-port.h>
#include "vcinterface.h"
#include "vciface.h"

#include <cfg_global.h>

#if (CFG_GLOBAL_CPU == MIPS32)
#include <asm/io.h>
#include <asm/broadcom/bcm1103/bcm1103.h>
#include <asm/broadcom/bcm1103/dma1103.h>
#endif

/******************************************************************************
Static data.
******************************************************************************/

static void *host_interface_locks[2] = { NULL, NULL };
static int host_interface_initted = 0;
static spinlock_t       gReadVCRegsSpinLock=SPIN_LOCK_UNLOCKED;

#if !defined( CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE )
#   define  CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE  0
#endif
#if CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE
// defined in vclcd.c for Kluge Qtopia fix!
extern int gVcAlphaBitsOverrideOn; 
extern int gVcAlphaBits;
#endif

#define  PRINT_ERR(  fmt, args... )  printk( KERN_ERR  fmt, ## args )

#define REG_VCO2_STC        (0x10001804)

VC_Clock_t gVCClock;

/******************************************************************************
Static functions.
******************************************************************************/

static void *lock_create( int chan_num ) ;

/******************************************************************************
NAME
   vc_host_init

SYNOPSIS
   int vc_host_init()

FUNCTION
   Perform platform dependent initialisations. Return non-zero for failure.

RETURNS
   int
******************************************************************************/
#if (CFG_GLOBAL_CPU == MIPS32)
static uint32_t *uncachedSwapMemp[2] = { NULL, NULL };
static uint32_t *uncachedSwapReadMemp[2] = { NULL, NULL };
static dma_addr_t physp[4];
#endif

int vc_host_init (void)
{

#if (CFG_GLOBAL_CPU == MIPS32)
   /* Initialize uncached buffers needed to swap data before dma */
   if (uncachedSwapReadMemp[1] == NULL )
   {
      uncachedSwapMemp[0] = dma_alloc_coherent( NULL, 1024, &physp[0], GFP_KERNEL );
      if (uncachedSwapMemp[0] == NULL)
      {
         printk( KERN_ERR "kernel: Cannot allocate shared memory page\n" );
         return 1;
      }
      uncachedSwapMemp[1] = dma_alloc_coherent( NULL, 1024, &physp[1], GFP_KERNEL );
      if (uncachedSwapMemp[1] == NULL)
      {
         printk( KERN_ERR "kernel: Cannot allocate shared memory page\n" );
         return 1;
      }
      uncachedSwapReadMemp[0] = dma_alloc_coherent( NULL, 1024, &physp[2], GFP_KERNEL );
      if (uncachedSwapReadMemp[0] == NULL)
      {
         printk( KERN_ERR "kernel: Cannot allocate shared memory page\n" );
         return 1;
      }
      uncachedSwapReadMemp[1] = dma_alloc_coherent( NULL, 1024, &physp[3], GFP_KERNEL );
      if (uncachedSwapReadMemp[1] == NULL)
      {
         printk( KERN_ERR "kernel: Cannot allocate shared memory page\n" );
         return 1;
      }
   }
#endif

   if ( vc_host_port_init() != 0 )
   {
      printk( KERN_ERR "vc_host_port_init failed.\n" );
      return 1;
   }

   // do initialization once only
   if ( !host_interface_initted )
   {
      host_interface_initted = 1;

      // initialise locks for use with each of the host interface channels
      if( host_interface_locks[0] == NULL )
          host_interface_locks[0] = lock_create(0);

      if( host_interface_locks[1] == NULL )
          host_interface_locks[1] = lock_create(1);
   }

	// return zero to indicate success
	return 0;
}

int vc_host_reg( void *hostReg )
{
    VC_HostReg_t *reg = (VC_HostReg_t *)hostReg;
    int     regNum  = reg->reg % 4;
    int     channel = reg->reg / 4;

    if (( channel != 0 ) && ( channel != 1 ))
    {
        return -EINVAL;
    }

    switch ( reg->op )
    {
        case VC_HOSTREG_OP_READ:
        {
            reg->val = vc_host_read_reg(  regNum, channel );
            break;
        }

        case VC_HOSTREG_OP_WRITE:
        {
            vc_host_write_reg( regNum, channel, (uint16_t)reg->val );
            break;
        }

        default:
        {
            return -EINVAL;
        }
    }

    return 0;
}

/* Need a dummy definition of this to link... */

void vc_host_interrupts( int onoff )
{

}


#if VC_HOST_PORT_USE_DMA

#if (CFG_GLOBAL_CPU == MIPS32)
/******************************************************************************
NAME
   vcHostDmaWrite

SYNOPSIS
static int vcHostDmaWrite( uint16_t *host_addr, volatile uint16_t *vcdata, volatile uint16_t *vchcs, int nblocks, int swap16bits )

FUNCTION
   Write nblocks consecutive 16-bit words to VideoCore host port channel channel. nblocks
   must be a multiple of 2, host_addr must be aligned to 4 bytes
   Returns non-zero for failure

RETURNS
   int
******************************************************************************/
#define MAX_VC02_FIFO_TX_SIZE_IN_16_BITS 16
int vcHostDmaWrite( uint16_t *host_addr, volatile uint16_t *vcdata, volatile uint16_t *vchcs, int nblocks, int swap16bits )
{
   uint16_t *src;
   uint32_t *src32;
   int rc = 0;
   int numSwap;
   int uncachedSwapBufNum = 0;
   uint32_t dummy32;
   uint8_t *dummy8p;

   uint32_t src_tx, dst_tx, size_tx;

   dummy8p = (void *)&dummy32;
   src = host_addr;
   if ( !swap16bits )
   {
      /* Flush the cache */
      dma_cache_wback( (unsigned long)src, nblocks << 1 );
   }

   dma_request_channel(DMA_CHANNEL_TX);

   dst_tx= ((unsigned int)vcdata) & 0x1fffffff;
   size_tx = MAX_VC02_FIFO_TX_SIZE_IN_16_BITS;

   numSwap = MAX_VC02_FIFO_TX_SIZE_IN_16_BITS >> 1;

   while ( nblocks > 0 )
   {
      if ( swap16bits )
      {
         int i;

         src32 = (void *)src;
         if ( nblocks < MAX_VC02_FIFO_TX_SIZE_IN_16_BITS )
         {
            numSwap = nblocks >> 1;
         }

         if ( swap16bits == 1 )
         {
            /* Swap 16 bits within 32 bit word example: ABCD = CDAB */
            for (i = 0; i < numSwap; i++)
            {
               dummy32 = src32[i];
               uncachedSwapMemp[uncachedSwapBufNum][i] = (dummy32 >> 16) | (dummy32 << 16);
            }
#if CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE
            if( gVcAlphaBitsOverrideOn ) 
            {
               for( i=0; i < numSwap; i++)
               {
                  // Override the alpha bits which are now in the 3rd byte of each 32bit pixel
                  ((uint8_t*)(uncachedSwapMemp[uncachedSwapBufNum]+i))[2] = (uint8_t)gVcAlphaBits;
               }
            }
#endif
         }
         else
         {
            /* Swap 8 bits within 16 bit word example: ABCD = BADC */
            for (i = 0; i < numSwap; i++)
            {

               dummy32 = src32[i];
               uncachedSwapMemp[uncachedSwapBufNum][i] = ((uint32_t)(dummy8p[1]) << 24) +
                                                         ((uint32_t)(dummy8p[0]) << 16) +
                                                         ((uint32_t)(dummy8p[3]) << 8) +
                                                         dummy8p[2];
            }
         }
      }

      /* Move along src pointer and size */
      if ( swap16bits )
      {
         src_tx = ((unsigned int)uncachedSwapMemp[uncachedSwapBufNum]) & 0x1fffffff;
         uncachedSwapBufNum ^= 1;
      }
      else
      {
         src_tx = ((unsigned int)src) & 0x1fffffff;
      }

      if ( nblocks < MAX_VC02_FIFO_TX_SIZE_IN_16_BITS )
      {
         size_tx = nblocks;
      }

      /* Wait for VC02 to have room available */
      LIMITED_WAIT_FOR( *vchcs & WFE );

      /* kick off polling based DMA, polling is more efficient in this case */
      dma_transfer(DMA_CHANNEL_TX, DMA_DEVICE_TYPE_FIFO, DMA_MODE_POLLING,
            src_tx, dst_tx, size_tx * 2);
      

      src += MAX_VC02_FIFO_TX_SIZE_IN_16_BITS;
      nblocks -= MAX_VC02_FIFO_TX_SIZE_IN_16_BITS;
   }

   end:

   dma_free_channel(DMA_CHANNEL_TX);
   return( rc );
}

/******************************************************************************
NAME
   vcHostDmaRead

SYNOPSIS
static int vcHostDmaRead( uint16_t *host_addr, volatile uint16_t *vcdata, volatile uint16_t *vchcs, int nblocks, int swap16bits )

FUNCTION
   Read nblocks consecutive 16-bit words to VideoCore host port channel channel. nblocks
   must be a multiple of 2, host_addr must be aligned to 4 bytes
   Returns non-zero for failure

RETURNS
   int
******************************************************************************/
#define MAX_VC02_FIFO_RX_SIZE_IN_16_BITS 16
int vcHostDmaRead( uint16_t *host_addr, volatile uint16_t *vcdata, volatile uint16_t *vchcs, int nblocks, int swap16bits )
{
   uint16_t *dest;
   uint32_t *dest32;
   int rc = 0;
   int uncachedSwapBufNum = 0;
   uint32_t dummy32;
   uint8_t *dummy8p;

   uint32_t src_rx, dst_rx, size_rx;

   dummy8p = (void *)&dummy32;
   dest = host_addr;
   if ( !swap16bits )
   {
      /* Invalidate the cache */
      dma_cache_inv( (unsigned long)dest, nblocks << 1 );
   }

   dma_request_channel(DMA_CHANNEL_RX);

   src_rx = ((unsigned int)vcdata) & 0x1fffffff;
   size_rx = MAX_VC02_FIFO_RX_SIZE_IN_16_BITS;

   while ( nblocks > 0 )
   {
      /* Move along dest pointer and size */
      if ( swap16bits )
      {
         dst_rx = ((unsigned int)uncachedSwapReadMemp[uncachedSwapBufNum]) & 0x1fffffff;
         uncachedSwapBufNum ^= 1;
      }
      else
      {
         dst_rx = ((unsigned int)dest) & 0x1fffffff;
         dest += MAX_VC02_FIFO_RX_SIZE_IN_16_BITS;
      }

      if ( nblocks < MAX_VC02_FIFO_RX_SIZE_IN_16_BITS )
      {
         size_rx = nblocks;
      }

      /* Wait for VC02 to have room available */
      LIMITED_WAIT_FOR( *vchcs & RFF );

      /* kick off polling based DMA, polling is more efficient in this case */
      dma_transfer(DMA_CHANNEL_RX, DMA_DEVICE_TYPE_FIFO, DMA_MODE_POLLING,
            src_rx, dst_rx, size_rx * 2);

      /* Swap previous data while DMA is completing */
      if ( swap16bits )
      {
         int i;
         int numToSwap;
         int bufToSwap = uncachedSwapBufNum;

         dest32 = (void *)dest;
         dest += MAX_VC02_FIFO_RX_SIZE_IN_16_BITS;
         if ( nblocks < MAX_VC02_FIFO_RX_SIZE_IN_16_BITS )
         {
            numToSwap = nblocks >> 1;
            bufToSwap ^= 1;
         }
         else
         {
            numToSwap = MAX_VC02_FIFO_RX_SIZE_IN_16_BITS >> 1;
            bufToSwap ^= 1;
         }

         if ( swap16bits == 1 )
         {
            /* Swap 16 bits within 32 bit word example: ABCD = CDAB */
            for (i = 0; i < numToSwap; i++)
            {
               dummy32 = uncachedSwapReadMemp[bufToSwap][i];
               dest32[i] = (dummy32 >> 16) | (dummy32 << 16);
            }
         }
         else
         {
            /* Swap 8 bits within 16 bit word example: ABCD = BADC */
            for (i = 0; i < numToSwap; i++)
            {

               dummy32 = uncachedSwapReadMemp[bufToSwap][i];
               dest32[i] = ((uint32_t)(dummy8p[1]) << 24) +
                           ((uint32_t)(dummy8p[0]) << 16) +
                           ((uint32_t)(dummy8p[3]) << 8) +
                                       dummy8p[2];
            }
         }
      }
      nblocks -= MAX_VC02_FIFO_RX_SIZE_IN_16_BITS;
   }

   end:

   dma_free_channel(DMA_CHANNEL_RX);
   return( rc );
}
#endif  // (CFG_GLOBAL_CPU == MIPS32)

#endif  // VC_HOST_PORT_USE_DMA

/******************************************************************************
NAME
   vc_host_wait_interrupt

SYNOPSIS
   int vc_host_wait_interrupt()

FUNCTION
   Wait from an interrupt from VideoCore. This function is allowed to return
   immediately, in which case the host must busy-wait. Non-zero indicates an error.

RETURNS
   int
******************************************************************************/

int vc_host_wait_interrupt (void) {
   return 0;
}

/******************************************************************************
NAME
   vc_host_read_VC02_STC_reg

SYNOPSIS
   int vc_host_read_VC02_STC_reg()

FUNCTION
   Reads the STC regsiter
   RETURNS
   int
******************************************************************************/

void vc_host_read_VC02_STC_reg ( void) {
   uint32_t clock;
   unsigned long flags;

   vc_host_read_consecutive(&clock, REG_VCO2_STC, 4, 1);
   clock = VC_VTOH32(clock);
   spin_lock_irqsave( &gReadVCRegsSpinLock, flags );
   gVCClock.dspclock = clock;
   gVCClock.hostclock = timer_get_tick_count();
   spin_unlock_irqrestore( &gReadVCRegsSpinLock, flags );
}

/******************************************************************************
NAME
   vc_host_read_clock

SYNOPSIS

FUNCTION
   Reads the STC regsiter
   RETURNS
   int
******************************************************************************/

void vc_host_get_clock ( VC_Clock_t *clock) {

   unsigned long flags;
   spin_lock_irqsave( &gReadVCRegsSpinLock, flags );
   *clock = gVCClock;
   spin_unlock_irqrestore( &gReadVCRegsSpinLock, flags );
}

/******************************************************************************
NAME
   vc_host_write_clock

SYNOPSIS

FUNCTION
   Reads the STC regsiter
   RETURNS
   int
******************************************************************************/

void vc_host_set_clock ( VC_Clock_t *clock) {

   unsigned long flags;
   spin_lock_irqsave( &gReadVCRegsSpinLock, flags );
   gVCClock.time_s=clock->time_s;
   gVCClock.time_frac = clock->time_frac;
   gVCClock.last_stc = clock->last_stc;
   gVCClock.stcfreq = clock->stcfreq;
   gVCClock.speed = clock->speed;
   spin_unlock_irqrestore( &gReadVCRegsSpinLock, flags );
}

/*---------------------------------------------------------------------------*/
/* Lock related functions */
/*---------------------------------------------------------------------------*/

/******************************************************************************
*   
*   lock_create
*
*   Create a lock. Returns the pointer to the lock. NULL if fails.
*   We expect locks to be made at startup, without the worry of contention 
*   here.
*
******************************************************************************/

static void *lock_create( int chan_num )
{
   return vc_lock_create();

} // lock_create

/******************************************************************************
*
*   vc_host_lock_obtain
*
*   Obtain the lock for the indicated channel, blocking until we get it.
*
******************************************************************************/

void vc_host_lock_obtain( int channel )
   {
    vc_lock_obtain( host_interface_locks[ channel ]);

} // vc_obtain_lock

/******************************************************************************
*
*   vc_host_release_lock
*
*   Release the lock for the indicated channel.
*
******************************************************************************/

void vc_host_lock_release( int channel )
{
    vc_lock_release( host_interface_locks[ channel ]);

} // release_lock

