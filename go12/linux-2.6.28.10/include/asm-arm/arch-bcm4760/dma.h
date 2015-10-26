/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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
#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <asm/arch/platform.h>
#include <asm/arch/hardware.h>

#define MAX_DMA_ADDRESS		0xffffffff
////*****************************************************
//THIS SHOULD BE ZERO...OTHERWISE GENERAL LINUX CODE WILL GET COMPILED IN
#define MAX_DMA_CHANNELS	0
////*****************************************************

// DCN, 4760reg.h does not define these DMA definitions, define them here.
#define MAX_BCM476X_DMA_CHANNELS         8
#define BCM476X_DMAC_CH_BASEADDR         (DMA_REG_BASE_ADDR + 0x100)
#define BCM476X_DMAC_CH_BASEADDR_MULT    0x00000020

#define BCM476X_DMAC_CH_SRCADDR_OFFSET   0x00000000
#define BCM476X_DMAC_CH_DESTADDR_OFFSET  0x00000004
#define BCM476X_DMAC_CH_LLIREG_OFFSET    0x00000008
#define BCM476X_DMAC_CH_CONTROL_OFFSET   0x0000000c
#define BCM476X_DMAC_CH_CONFIG_OFFSET    0x00000010

/* transfer width, also used as function parameters */
#define BCM476X_DMAC_TRANSFER_WIDTH_8BIT    0 /* 8 bit   */
#define BCM476X_DMAC_TRANSFER_WIDTH_16BIT   1 /* 16 bits */
#define BCM476X_DMAC_TRANSFER_WIDTH_32BIT   2 /* 32 bits */

/* burst size, also used as function parameters */
#define BCM476X_DMAC_BURST_SIZE_1           0
#define BCM476X_DMAC_BURST_SIZE_4           1
#define BCM476X_DMAC_BURST_SIZE_8           2
#define BCM476X_DMAC_BURST_SIZE_16          3
#define BCM476X_DMAC_BURST_SIZE_32          4
#define BCM476X_DMAC_BURST_SIZE_64          5
#define BCM476X_DMAC_BURST_SIZE_128         6
#define BCM476X_DMAC_BURST_SIZE_256         7

/* flow control and transfer type values, values refer to PL081 TRM P3-22 Table 3-24 */
#define BCM476X_DMAC_MEM2MEM_CTL            0 /* PrimeCell DMAC as the flow controller */
#define BCM476X_DMAC_MEM2PERI_CTL           1
#define BCM476X_DMAC_PERI2MEM_CTL           2
#define BCM476X_DMAC_PERI2PERI_CTL          3
#define BCM476X_DMAC_PERI2PERI_DST_CTL      4 /* destination periphral as flow controller */
#define BCM476X_DMAC_MEM2PERI_PERI_CTL      5
#define BCM476X_DMAC_PERI2MEM_PERI_CTL      6
#define BCM476X_DMAC_PERI2PERI_SRC_CTL      7 /* source periphral as flow controller */

/* PL080 DMA peripheral IDs */
#define    BCM476X_DMAC_PERIPHERAL_UART0_Rx    0   
#define    BCM476X_DMAC_PERIPHERAL_UART1_Rx    1   
#define    BCM476X_DMAC_PERIPHERAL_UART2_Rx    2   
#define    BCM476X_DMAC_PERIPHERAL_UART3_Rx    3   
#define    BCM476X_DMAC_PERIPHERAL_UART0_Tx    4   
#define    BCM476X_DMAC_PERIPHERAL_UART1_Tx    5   
#define    BCM476X_DMAC_PERIPHERAL_UART2_Tx    6   
#define    BCM476X_DMAC_PERIPHERAL_UART3_Tx    7   
#define    BCM476X_DMAC_PERIPHERAL_SPI0_Rx     8
#define    BCM476X_DMAC_PERIPHERAL_SPI1_Rx     9
#define    BCM476X_DMAC_PERIPHERAL_SPI0_Tx     10
#define    BCM476X_DMAC_PERIPHERAL_SPI1_Tx     11
#define    BCM476X_DMAC_PERIPHERAL_I2S_Rx      12   
#define    BCM476X_DMAC_PERIPHERAL_I2S_Tx      13   
#define    BCM476X_DMAC_PERIPHERAL_SPM_Tx      14   
#define    BCM476X_DMAC_PERIPHERAL_SPM_Rx      15   

#define    BCM476X_DMAC_PERIPHERAL_MEMORY      24 /* not a real ID used by HW */

typedef enum
{
    DMA_DEVICE_UART0_Rx,
		DMA_DEVICE_UART1_Rx,
		DMA_DEVICE_UART2_Rx,
		DMA_DEVICE_UART3_Rx,
		DMA_DEVICE_UART0_Tx,
		DMA_DEVICE_UART1_Tx,
		DMA_DEVICE_UART2_Tx,
		DMA_DEVICE_UART3_Tx,
		DMA_DEVICE_SPI0_Rx,
		DMA_DEVICE_SPI1_Rx,
		DMA_DEVICE_SPI0_Tx,
		DMA_DEVICE_SPI1_Tx,
		DMA_DEVICE_I2S_Rx,
		DMA_DEVICE_I2S_Tx,
		DMA_DEVICE_SPUM_DEV_TO_MEM,
		DMA_DEVICE_SPUM_MEM_TO_DEV,

    // Add new entries before this line.

    DMA_NUM_DEVICE_ENTRIES,
    DMA_DEVICE_NONE = 0xff,    // Special value to indicate that no device is currently assigned.
} DMA_Device_t;

typedef enum {
	BCM476X_DMA_ASSIGNED,
	BCM476X_DMA_IN_USE,
	BCM476X_DMA_FREE,
	BCM476X_DMA_INTERRUPTED,
	BCM476X_DMA_BAD_CH,
	BCM476X_DMA_UNKNOWN
} BCM476X_DMA_STATE;

typedef void  (*DMAC_IRQ_HANDLER)(unsigned long tag, unsigned long error);

typedef struct {
	uint src_addr; 
	uint dst_addr; 
	uint next_lli; 
	uint control; 	
} DMAC_LINKED_LIST;

typedef struct {
	uint src_addr; 
	uint dst_addr; 
	uint lli; 
	uint control;
	uint config;
} DMAC_CH_REG;


#define BCM476X_EN_DMA_ALL_TC_INT		0x01
#define BCM476X_EN_DMA_LAST_TC_INT		0x02
typedef struct {
	uint*     addr; // address list
	uint*     tfr_size; // address list
	unsigned short    n_addr; //# of address entry 
	unsigned char    incr; //increment or not
	unsigned char    width; //width of tfer
	unsigned char    burst_sz; //burst size
	// bit 0 - enable interrupt after every Link List node
	// bit 1 - enable interrupt for the last node only
	unsigned char    flags;
} DMAC_CTRL_REG;

typedef struct {
	unsigned char    src_id;
	unsigned char    dst_id;
	unsigned short    trans_type;
} DMAC_CFG_REG;

typedef struct {
	/* channel state flags and information */
	DMAC_LINKED_LIST *		ch_link_list; //virt address from dma_pool_alloc for ARM
	DMAC_LINKED_LIST *		ch_link_list_phy; //correspoding physical address for DMA
//	uint 						n_ll;
	/* channel state */
	BCM476X_DMA_STATE		state;

	void __iomem				*base_addr;        /* channels registers */

	/* driver handles */
	DMAC_IRQ_HANDLER	    irq_handler;  /* buffer done callback */
	unsigned long 			tag;

	spinlock_t				lock;
	struct dma_pool*			pool; //for dma_pool_create
	unsigned char				n_ll_elements; // # link nodes allocated
	/* client details */
	//char *					client_name;
	unsigned char				ch_number;      /* number of this dma channel */
	unsigned char				error;      /* error flag */
}DMAC_CHANNEL ;

//typedef void *DMA_Handle_t;
typedef int DMA_Handle_t;

extern int bcm476x_setup_dma_chain
( 
	const int ch, 

	DMAC_CTRL_REG *src, // src side config data
	DMAC_CTRL_REG *dst, // dst side config data
	DMAC_CFG_REG *cfg //config data
);
// to release the channel
extern int bcm476x_release_dma_channel
( 
	int ch 
);
//start transfer
extern int bcm476x_enable_dma_channel
( 
	int ch 
);
//disable the transfer..call setup again to use the channel
// channel is still assigned to the caller
extern int bcm476x_disable_dma_channel
( 
	int ch, int immediately
 
);
//disable any further interrupts
extern int bcm476x_set_dma_halt_bit
( 
	int ch, int halt 
);
//returns the state of the channel
extern int bcm476x_get_dma_channel_state
( 
	int ch 
);

// returns the available channel number and magic number
// returns -1 if no channel is available
extern int bcm476x_request_dma_channel(void);

//pre-allocate buffer for linklist. Try to pass the maximum value driver may ask DMA lib
// to transfer. If this function is not called then allocation will happen when driver call
// setup_dma_chain

// this function should be called after channel is assigned and before bcm476x_setup_dma_chain 
extern int allocate_dma_link_list
(
	int ch, //channel number
	int n_bytes_to_tfr, // max # of bytes to be transfered
	int width //width to be used in DMA control register
);

//register the irq handler
extern void bcm476x_register_dma_handler
(
	const int ch,
	unsigned long tag,
	void (*callback_fn)(unsigned long tag, unsigned long error)
);

extern void bcm476x_dma_fast_setup( 
	const int ch,
	DMAC_CH_REG *ch_reg
);

extern int bcm476x_initialize_dma_chain( 
	const int ch, 

	DMAC_CTRL_REG *src, // src side config data
	DMAC_CTRL_REG *dst, // dst side config data
	DMAC_CFG_REG *cfg, //config data

	DMAC_CH_REG *ch_reg
);
#endif /* _ASM_ARCH_DMA_H */

