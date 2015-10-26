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

/*
 * Description: The Display Director DMA mechanism
 */

#include <linux/completion.h>
#include <asm/arch/dma.h>
#include "dd_dma.h"

/*
 * DMA configurations
 */
typedef struct dma_cfg {
   DMA_Handle_t handle;
   struct semaphore lock;
   struct completion complete;
} DMA_CFG_T;

static DMA_CFG_T g_dma;

/*
 * DMA IRQ hanlder. This routine is invoked when DMA completes
 */
static void dma_isr(DMA_Device_t dev, int reason, void *data)
{
   DMA_CFG_T *dma = (DMA_CFG_T *)data;
   complete(&dma->complete);
}

DD_STATUS_T dd_dma_init(void)
{
   DMA_CFG_T *dma = &g_dma;

   init_MUTEX(&dma->lock); /* unlocked */
   init_completion(&dma->complete); /* incomplete */

   if (dma_set_device_handler(DMA_DEVICE_CLCD_MEM_TO_MEM, dma_isr,
            dma) != 0) {
      printk(KERN_ERR "DD_DMA: dma_set_device_handler failed\n");
      return DD_FAIL;
   }

   /*
    * Since a DMA channel is dedicated to the DD, might as well reserve it
    * here to save some overhead
    */
   dma->handle = dma_request_channel(DMA_DEVICE_CLCD_MEM_TO_MEM);
   if (dma->handle < 0) {
      printk(KERN_ERR "DD_DMA: dma_request_channel failed\n");
      return DD_FAIL;
   }

   return DD_SUCCESS;
}

DD_STATUS_T dd_dma_term(void)
{
   DMA_CFG_T *dma = &g_dma;

   if (dma_free_channel(dma->handle) != 0) {
      printk(KERN_ERR "DD_DMA: dma_free_channel failed\n");
      return DD_FAIL;
   }
   return DD_SUCCESS;
}

DD_STATUS_T dd_dma_transfer(dma_addr_t src_addr, dma_addr_t dst_addr,
      uint32_t len)
{
   DMA_CFG_T *dma = &g_dma;

   if (down_interruptible(&dma->lock) != 0) {
      printk(KERN_ERR "DD_DMA: Lock acquire interrupted or failed\n");
      goto dma_err;
   }

   DD_LOG("tstart [DD dd_dma_transfer]\n");

   /* mark as incomplete before DMA starts */
   INIT_COMPLETION(dma->complete);

   if (dma_transfer_mem_to_mem(dma->handle, src_addr, dst_addr, len) != 0) {
      printk(KERN_ERR "DD_DMA: dma_transfer_mem_to_mem failed\n");
      goto dma_err_complete;
   }

   /* block wait until DMA completes */
   wait_for_completion(&dma->complete);

   DD_LOG("tstop [DD dd_dma_transfer]\n");

   up(&dma->lock);
   return DD_SUCCESS;

dma_err_complete:
   complete(&dma->complete);
   DD_LOG("tstop [DD dd_dma_transfer]\n");
   up(&dma->lock);
dma_err:
   return DD_FAIL;
}
EXPORT_SYMBOL(dd_dma_transfer);
