/*****************************************************************************
*  Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

#ifndef _BCM_SPU_H_
#define _BCM_SPU_H_

//#include <mach/dma.h>
#include <asm/arch/dma.h>

#define SPU_DMA_FIFO_SIZE            (32 * 1024)     /* 32K FIFO size */
#define SPU_DMA_CMD_BUFFER_LENGTH    1024
#define SPU_DMA_DATA_BUFFER_LENGTH   (SPU_DMA_FIFO_SIZE)

/* SPU buffer structure */
typedef struct {
   void *virt;
   dma_addr_t phys;
} spu_data_buf;

/* SPU DMA configuration information */
typedef struct {
   DMA_Device_t device;
   DMA_Handle_t handle;
   dma_addr_t fifo_addr;
} spu_dma_cfg;

typedef struct {
   spu_data_buf crypto_cmd;     /* SPU command buffer */
   spu_data_buf crypto_in;      /* SPU crypto input */
   spu_data_buf crypto_out;     /* SPU crypto output */
   spu_dma_cfg dma_tx_cfg;
   spu_dma_cfg dma_rx_cfg;
   struct semaphore dma_tx_lock;
   struct semaphore dma_rx_lock;
} spu_dma_context;

void spu_request( unsigned int force_init );
void spu_release( void );

int spu_dma_context_init( spu_dma_context *spu_dma );
int spu_dma_set_device_handlers( spu_dma_context *spu_dma );
int spu_dma_alloc( spu_dma_context *spu_dma );
void spu_dma_dealloc( spu_dma_context *spu_dma );

void spu_dma_reserve(spu_dma_context *spu_dma);
void spu_dma_free(spu_dma_context *spu_dma);

void spu_dma_config(spu_dma_context *spu_dma, uint32_t cmd_len, uint32_t in_len, uint32_t out_len);
void spu_dma_wait(spu_dma_context *spu_dma);

#endif
