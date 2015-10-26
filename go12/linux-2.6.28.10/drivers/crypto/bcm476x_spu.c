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

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/broadcom/bcm_spu.h>

#include <asm/byteorder.h>
#include <linux/semaphore.h>

#include <asm/arch/spuHw.h>
#include <asm/arch/spuHw_inline.h>
//#include <mach/csp/cap.h>

/* ---- Private Variables ------------------------------------ */
static struct semaphore spu_lock;
static unsigned int spu_initialized = 0;
static unsigned int lock_initialized = 0;

/* ---- Private Function Prototypes -------------------------- */
static void spu_dma_irq_tx(void *data, int reason);
static void spu_dma_irq_rx(void *data, int reason);

/* ---- Functions -------------------------------------------- */
/*****************************************************************************
* FUNCTION:   spu_dma_irq_tx
*
* PURPOSE:    IRQ handler for DMA in TX direction (Memory to SPU-M)
*
* PARAMETERS: dev - [IN] Dma device descriptor
*             reason - [IN] Unused
*             data - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
static void spu_dma_irq_tx( void *data, int reason)
{
   spu_dma_context *spu_dma = (spu_dma_context *)data;
   (void)reason;   
   up(&spu_dma->dma_tx_lock);
}

/*****************************************************************************
* FUNCTION:   spu_dma_irq_rx
*
* PURPOSE:    IRQ handler for DMA in RX direction (SPU-M to Memory)
*
* PARAMETERS: dev - [IN] Dma device descriptor
*             reason - [IN] Unused
*             data - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
static void spu_dma_irq_rx(void *data, int reason)
{
   spu_dma_context *spu_dma = (spu_dma_context *)data;
   (void)reason;
   up(&spu_dma->dma_rx_lock);
}

/*****************************************************************************
* FUNCTION:   spu_request
*
* PURPOSE:    Request for access to SPU-M.  Will initialize SPU-M if not done
*
* PARAMETERS: force_init - [IN] Force the SPU-M to initialize or reset
*
* RETURNS:    None
*****************************************************************************/
void spu_request( unsigned int force_init )
{
   if ( !lock_initialized )
   {
      init_MUTEX(&spu_lock);
      lock_initialized = 1;
   }
   
   if ( !spu_initialized || force_init )
   {
      /* Initialize SPU-M block */
      spuHw_initDevice();

#ifdef __LITTLE_ENDIAN
      /* Configure the device for little endian input and output */
      spuHw_configDevice ( spuHw_DEV_CONFIG_OPEN |
            spuHw_DEV_CONFIG_INPUT_LITTLE |
            spuHw_DEV_CONFIG_OUTPUT_LITTLE );
#else
      /* Configure the device for big endian input and output */
      spuHw_configDevice ( spuHw_DEV_CONFIG_OPEN );
#endif
      spu_initialized = 1;
   }
   
   down(&spu_lock);
}

/*****************************************************************************
* FUNCTION:   spu_release
*
* PURPOSE:    Release access to SPU-M.
* 
* PARAMETERS: None
*
* RETURNS:    None
*****************************************************************************/
void spu_release( void )
{
   up(&spu_lock);
}

/*****************************************************************************
* FUNCTION:   spu_dma_context_init
*
* PURPOSE:    Initialize settings in the spu_dma_context struct
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_context_init( spu_dma_context *spu_dma )
{
   int rc=0;

   init_MUTEX_LOCKED( &spu_dma->dma_tx_lock);
   init_MUTEX_LOCKED( &spu_dma->dma_rx_lock);

   spu_dma->dma_tx_cfg.device = DMA_DEVICE_SPUM_MEM_TO_DEV;
   spu_dma->dma_tx_cfg.fifo_addr = spuHw_IN_FIFO_PHYS_ADDR;
   //spu_dma->dma_tx_cfg.fifo_addr = MM_IO_VIRT_TO_PHYS(spuHw_getInputFifoAddress());
   
   spu_dma->dma_rx_cfg.device = DMA_DEVICE_SPUM_DEV_TO_MEM;
   spu_dma->dma_rx_cfg.fifo_addr = spuHw_OUT_FIFO_PHYS_ADDR;
   //spu_dma->dma_rx_cfg.fifo_addr = MM_IO_VIRT_TO_PHYS(spuHw_getOutputFifoAddress());

   rc = spu_dma_set_device_handlers(spu_dma);

   return rc;   	 
}

/*****************************************************************************
* FUNCTION:   spu_dma_set_device_handlers
*
* PURPOSE:    Associates the DMA device handler to the spu_dma_context
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_set_device_handlers( spu_dma_context *spu_dma )
{
   int rc = 0;
	 
#if(0)
   rc = dma_set_device_handler( spu_dma->dma_tx_cfg.device, spu_dma_irq_tx, spu_dma );
   if (rc != 0) 
   {
      printk("spu_dma_set_device_handlers: TX dma_set_device_handler failed\n");
      return (rc);
   }
   
   rc = dma_set_device_handler(spu_dma->dma_rx_cfg.device, spu_dma_irq_rx, spu_dma);
   if (rc != 0) 
   {
      printk("spu_dma_set_device_handlers: RX dma_set_device_handler failed\n");
      return (rc);
   }
	 
#else /* 4760-OCF */
   bcm476x_register_dma_handler(spu_dma->dma_tx_cfg.handle, (u32) spu_dma, spu_dma_irq_tx );
   //printk("spu_dma_set_device_handlers: TX dma_set_device_handler\n");
   
   bcm476x_register_dma_handler(spu_dma->dma_rx_cfg.handle, (u32) spu_dma, spu_dma_irq_rx);
   //printk("spu_dma_set_device_handlers: RX dma_set_device_handler\n");

#endif

   return rc;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_alloc
*
* PURPOSE:    Allocate memory for DMA buffers
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_alloc( spu_dma_context *spu_dma )
{
   spu_dma->crypto_cmd.virt = dma_alloc_writecombine(  NULL, SPU_DMA_CMD_BUFFER_LENGTH, &spu_dma->crypto_cmd.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_cmd.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_cmd buffer\n");
      return (-ENOMEM);
   }
   
   spu_dma->crypto_in.virt = dma_alloc_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, &spu_dma->crypto_in.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_in.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_in buffer\n");
      return (-ENOMEM);
   }
   
   spu_dma->crypto_out.virt = dma_alloc_writecombine(  NULL, SPU_DMA_DATA_BUFFER_LENGTH, &spu_dma->crypto_out.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_out.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_out buffer\n");
      return (-ENOMEM);
   }
   return 0;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_dealloc
*
* PURPOSE:    Free DMA buffer memory
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
void spu_dma_dealloc( spu_dma_context *spu_dma )
{
   if (spu_dma->crypto_out.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, spu_dma->crypto_out.virt, spu_dma->crypto_out.phys);
   }
   spu_dma->crypto_out.phys = 0;

   if (spu_dma->crypto_in.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, spu_dma->crypto_in.virt, spu_dma->crypto_in.phys);
   }
   spu_dma->crypto_in.phys = 0;   
   
   if (spu_dma->crypto_cmd.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_CMD_BUFFER_LENGTH, spu_dma->crypto_cmd.virt, spu_dma->crypto_cmd.phys);
   }
   spu_dma->crypto_cmd.phys = 0;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_reserve
*
* PURPOSE:    Reserve DMA channels for use
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_reserve(spu_dma_context *spu_dma)
{
#if(0)
   /* reserve TX/RX DMA channels */
   spu_dma->dma_tx_cfg.handle = dma_request_channel(spu_dma->dma_tx_cfg.device);
   if (spu_dma->dma_tx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: TX dma_request_channel failed\n");
   }
   
   spu_dma->dma_rx_cfg.handle = dma_request_channel(spu_dma->dma_rx_cfg.device);
   if (spu_dma->dma_rx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: RX dma_request_channel failed\n");
   }

#else /* 4760-OCF */
   /* reserve TX/RX DMA channels */
   spu_dma->dma_tx_cfg.handle = (DMA_Handle_t) bcm476x_request_dma_channel();
   if (spu_dma->dma_tx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: TX dma_request_channel failed\n");
   }
   
   spu_dma->dma_rx_cfg.handle = (DMA_Handle_t) bcm476x_request_dma_channel();
   if (spu_dma->dma_rx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: RX dma_request_channel failed\n");
   }

#endif	 	 
}

/*****************************************************************************
* FUNCTION:   spu_dma_free
*
* PURPOSE:    Free acquired DMA channels
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_free(spu_dma_context *spu_dma)
{
   int rc;
   
#if(0)
   rc = dma_free_channel(spu_dma->dma_tx_cfg.handle);
   if (rc != 0)
   {
      printk("spu_dma_free: TX dma_free_channel failed\n");
   }
   
   rc = dma_free_channel(spu_dma->dma_rx_cfg.handle);
   if (rc != 0)
   {
      printk("spu_dma_free: RX dma_free_channel failed\n");
   }  
	 
#else /* 4760-OCF */
   rc = bcm476x_release_dma_channel((int) spu_dma->dma_tx_cfg.handle);
   if (rc != (int) spu_dma->dma_tx_cfg.handle)
   {
      printk("spu_dma_free: TX dma_free_channel failed\n");
   }
   
   rc = bcm476x_release_dma_channel((int) spu_dma->dma_rx_cfg.handle);
   if (rc != (int) spu_dma->dma_rx_cfg.handle)
   {
      printk("spu_dma_free: RX dma_free_channel failed\n");
   }  
	 
#endif	  
}

/*****************************************************************************
* FUNCTION:   spu_dma_config
*
* PURPOSE:    Configure DMA to setup for transfer
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*             cmd_len - [IN] length of command in bytes
*             in_len  - [IN] length of input data in bytes
*             out_len - [IN] length of expected output data in bytes
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_config(spu_dma_context *spu_dma, uint32_t cmd_len, uint32_t in_len, uint32_t out_len)
{
   int rc = 0;
	 
#if (1) /* 4760-OCF */

   int rval;
   u32 tx_out_tf_size[2], tx_in_tf_size[2], tx_src_list[2], tx_dest_list[2];
	 u32 rx_tf_size[1], rx_src_list[1], rx_dest_list[1];
   DMAC_CTRL_REG tx_in, rx_in;
   DMAC_CTRL_REG tx_out, rx_out;
   DMAC_CFG_REG  tx_cfg, rx_cfg;

   /* setup tx lli - mem2spu*/
   tx_in_tf_size[0] = cmd_len/4; // Divided by 4 because transfer width is 32 bits
   tx_in_tf_size[1] = (((((in_len+3) / sizeof(uint32_t)) * sizeof(uint32_t))+4)/4); /*32 bit align + extra 4 bytes for status; /4 because transfer width is 32 bits */   	 
	 tx_out_tf_size[0] = tx_in_tf_size[0];
	 tx_out_tf_size[1] = tx_in_tf_size[1];
	 tx_src_list[0] = spu_dma->crypto_cmd.phys;
   tx_src_list[1] = spu_dma->crypto_in.phys;
   tx_dest_list[0] = spu_dma->dma_tx_cfg.fifo_addr;
   tx_dest_list[1] = spu_dma->dma_tx_cfg.fifo_addr;
   //prepare the DMA register data
   tx_in.burst_sz = BCM476X_DMAC_BURST_SIZE_16; // 16 byte burst
   tx_in.incr = 1;
   tx_in.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   tx_in.addr = tx_src_list;
   tx_in.tfr_size = tx_in_tf_size;
   tx_in.n_addr = 2;
   tx_in.flags = BCM476X_EN_DMA_LAST_TC_INT;

   tx_out.burst_sz = BCM476X_DMAC_BURST_SIZE_16;
   tx_out.incr = 1;
   tx_out.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   tx_out.addr = tx_dest_list;
   tx_out.tfr_size = tx_out_tf_size;
   tx_out.n_addr = 2;

   tx_cfg.src_id = 0; /* memory */ 
   tx_cfg.dst_id = spu_dma->dma_tx_cfg.device;
   tx_cfg.trans_type = BCM476X_DMAC_MEM2PERI_CTL; /* DMAC control flow */
	 
   rval = bcm476x_setup_dma_chain(spu_dma->dma_tx_cfg.handle, &tx_in, &tx_out, &tx_cfg); 
   if (rval <= 0) {
      printk(KERN_ERR "bcm476x_spu: spu_dma_config: bcm476x_setup_dma_chain failed tx chain\n");
   }

   /* setup rx lli - spu2mem */
   rx_tf_size[0] = ((((out_len+3) / sizeof(uint32_t)) * sizeof(uint32_t))/4); /*32 bit align, /4 because transfer width is 32 bits */   	 
   rx_src_list[0] = spu_dma->dma_rx_cfg.fifo_addr;
   rx_dest_list[0] = spu_dma->crypto_out.phys;;
   //prepare the DMA register data
   rx_in.burst_sz = BCM476X_DMAC_BURST_SIZE_16; // 16 byte burst
   rx_in.incr = 1;
   rx_in.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   rx_in.addr = rx_src_list;
   rx_in.tfr_size = rx_tf_size;
   rx_in.n_addr = 1;
   rx_in.flags = BCM476X_EN_DMA_LAST_TC_INT;

   rx_out.burst_sz = BCM476X_DMAC_BURST_SIZE_16;
   rx_out.incr = 1;
   rx_out.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   rx_out.addr = rx_dest_list;
   rx_out.tfr_size = rx_tf_size;
   rx_out.n_addr = 1;

   rx_cfg.src_id = spu_dma->dma_rx_cfg.device; 
   rx_cfg.dst_id = 0; /* memory */ 
   //rx_cfg.trans_type = BCM476X_DMAC_PERI2MEM_PERI_CTL; /* SPUM control flow */
   rx_cfg.trans_type = BCM476X_DMAC_PERI2MEM_CTL; /* SPUM control flow (same as bootrom setting) */
	 
   rval = bcm476x_setup_dma_chain(spu_dma->dma_rx_cfg.handle, &rx_in, &rx_out, &rx_cfg); 
   if (rval <= 0) {
      printk(KERN_ERR "bcm476x_spu: spu_dma_config: bcm476x_setup_dma_chain failed rx chain\n");
   }

   /* Start the dma */
   bcm476x_enable_dma_channel(spu_dma->dma_tx_cfg.handle);
   bcm476x_enable_dma_channel(spu_dma->dma_rx_cfg.handle);
	 	 
#else	 
   unsigned int desc_cnt; /* descriptor count */
   DMA_DescriptorRing_t tx_dma_ring;

   /*
    * Set up TX dma so it can transfer both the SPU command and data in
    * one shot
    */

   /* number of desc needed for SPU command */
   desc_cnt = dma_calculate_descriptor_count(spu_dma->dma_tx_cfg.device,
            spu_dma->crypto_cmd.phys, spu_dma->dma_tx_cfg.fifo_addr, cmd_len);

   /* add number of desc needed for SPU data */
   desc_cnt += dma_calculate_descriptor_count(spu_dma->dma_tx_cfg.device,
            spu_dma->crypto_in.phys, spu_dma->dma_tx_cfg.fifo_addr,
            in_len);

   /* allocate the descriptor ring */
   rc = dma_alloc_descriptor_ring(&tx_dma_ring, desc_cnt);
   
   if (rc != 0) 
   {
      printk("spu_dma_config: dma_alloc_descriptor_ring failed\n");
   }

   /* add desc for SPU command */
   rc = dma_add_descriptors(&tx_dma_ring,
         spu_dma->dma_tx_cfg.device,
         spu_dma->crypto_cmd.phys,
         spu_dma->dma_tx_cfg.fifo_addr,
         cmd_len);
   
   if (rc != 0) 
   {
      printk("spu_dma_config: dma_add_descriptors failed\n");
   }
   
   /* add desc for SPU data */
   rc = dma_add_descriptors(&tx_dma_ring,
         spu_dma->dma_tx_cfg.device,
         spu_dma->crypto_in.phys,
         spu_dma->dma_tx_cfg.fifo_addr,
         ( ((in_len + 3) / sizeof(uint32_t)) * sizeof(uint32_t)) + 4); /*32 bit align and add extra 4 bytes for status */
   
   if (rc != 0) 
   {
      printk("spu_dma_config: dma_add_descriptors failed\n");
   }

   /* associate the device with the descriptor ring data structure */
   rc = dma_set_device_descriptor_ring(spu_dma->dma_tx_cfg.device,
         &tx_dma_ring);
   
   if (rc != 0) 
   {
      printk("spu_dma_config: dma_set_device_descriptor_ring failed\n");
   }

   /* tell the DMA ready for transfer */
   rc = dma_start_transfer(spu_dma->dma_tx_cfg.handle);
   
   if (rc != 0)
   {
      printk("spu_dma_config: dma_start_transfer failed\n");
   }
   
   rc = dma_transfer_from_device(spu_dma->dma_rx_cfg.handle,
         spu_dma->dma_rx_cfg.fifo_addr, spu_dma->crypto_out.phys, out_len);				 
#endif /* 4760-OCF */
				    
   if (rc != 0)
   {
      printk("spu_dma_config: dma_transfer_from_device failed\n");   
   }   
}

/*****************************************************************************
* FUNCTION:   spu_dma_wait
*
* PURPOSE:    Wait for DMA transfer to complete
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_wait(spu_dma_context *spu_dma)
{
   down(&spu_dma->dma_tx_lock);
   down(&spu_dma->dma_rx_lock);   
}

EXPORT_SYMBOL(spu_request);
EXPORT_SYMBOL(spu_release);
EXPORT_SYMBOL(spu_dma_alloc);
EXPORT_SYMBOL(spu_dma_dealloc);
EXPORT_SYMBOL(spu_dma_context_init);
EXPORT_SYMBOL(spu_dma_set_device_handlers);
EXPORT_SYMBOL(spu_dma_reserve);
EXPORT_SYMBOL(spu_dma_free);
EXPORT_SYMBOL(spu_dma_config);
EXPORT_SYMBOL(spu_dma_wait);

static int spu_probe(struct platform_device *pdev)
{
   return 0;
}

static int spu_remove(struct platform_device *pdev)
{
   return 0;
}

#ifdef CONFIG_PM
static int spu_suspend(struct platform_device *pdev, pm_message_t state)
{
   down(&spu_lock);
   return 0;
}

static int spu_resume(struct platform_device *pdev)
{
   up(&spu_lock);
   return 0;
}
#else
#define spu_suspend   NULL
#define spu_resume    NULL
#endif

static struct platform_driver spu_driver = {
   .driver = {
      .name = "bcm476x-spu",
      .owner = THIS_MODULE,
   },
   .probe = spu_probe,
   .remove = spu_remove,
   .suspend = spu_suspend,
   .resume = spu_resume,
};

static char __initdata banner[] = "BCM476X SPU Driver, (c) 2009 Broadcom Corporation\n";
static int __init spu_init(void)
{   

#if 0 /* 4760-OCF Port: */
   if (cap_isPresent(CAP_SPU,0) == CAP_NOT_PRESENT ) {
      printk (KERN_WARNING "SPU is not supported\n");
      return -EFAULT;
   }
#endif
	 
   printk( banner );
   return platform_driver_register(&spu_driver);
}

static void __exit spu_exit(void)
{
   platform_driver_unregister(&spu_driver);
}

module_init(spu_init);
module_exit(spu_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom BCM476X SPU Driver");
MODULE_LICENSE("GPL");
