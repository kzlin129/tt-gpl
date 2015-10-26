/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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


#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/arch/dma.h>
#include <asm/arch/platform.h>

//#define BCM_DMA_DEBUG

#ifdef BCM_DMA_DEBUG
#define PRINT_DBG(x...) printk(x)
#else
#define PRINT_DBG(x...)
#endif

//taskelet related variables
static struct tasklet_struct	dma_tlet;
static unsigned int  dma_tasklet_data;


uint get_ch_base_addr(int channel);
static int width_in_bytes(unsigned int width);
static int n_ll_required(DMAC_CTRL_REG *src);
static int create_dma_link_list
(
	int ch,
	DMAC_CTRL_REG *src, 
	DMAC_CTRL_REG *dst
);

/////////////////////////////////////////////////////////////////////////////
// Description:
// Enable DMAC and set endianess (1 for Big Endian and 0 for Little Endian)
// Example of usage: EnableDMA(0)  -- enable DMA for Little endian mode
/////////////////////////////////////////////////////////////////////////////
#define DMA_LITTLE_ENDIAN 0

#define MAX_TFR_SIZE 0x0fff

DMAC_CHANNEL dma_ch[MAX_BCM476X_DMA_CHANNELS];
//static DMAC_LINKED_LIST *global_link_list[MAX_BCM476X_DMA_CHANNELS];

/******************************************************************************
NAME
   get_ch_base_addr

SYNOPSIS
   uint get_ch_base_addr(int channel)
   
FUNCTION
   returns the base address of the "channel"
   
RETURNS
   - 0 if channel # is not valid otherwise retuens the address
******************************************************************************/

uint get_ch_base_addr(int channel)
{
	if (channel >= MAX_BCM476X_DMA_CHANNELS)
	{
		return 0;
	}
	return (uint)((channel) * BCM476X_DMAC_CH_BASEADDR_MULT + IO_ADDRESS(BCM476X_DMAC_CH_BASEADDR));
}

/******************************************************************************
NAME
   allocate_dma_link_list_nodes

SYNOPSIS
   void allocate_dma_link_list_nodes(int channel, int n_elements)
   
FUNCTION
   Allocate buffer for link list node if requested elements  are greater than current
   Rest all pointer to NULL if allocation failed
   
RETURNS
   - 
******************************************************************************/
void allocate_dma_link_list_nodes(int ch, int n_elements)
{
	dma_addr_t addr;
	if(dma_ch[ch].n_ll_elements >= n_elements)
	{
		//if current buffer is big enough to hold n_elements then use it
		PRINT_DBG("channel %d had enough link list mem\n", ch);
		return;
	}
	else
	{
		if(dma_ch[ch].ch_link_list != NULL)
		{
			PRINT_DBG("freeing memory for - %d\n", ch);
			dma_pool_free(dma_ch[ch].pool, dma_ch[ch].ch_link_list, (unsigned int)dma_ch[ch].ch_link_list_phy);
			dma_pool_destroy(dma_ch[ch].pool);
		}
		PRINT_DBG("allocating memory for - %d\n", ch);
		//create pool on 32 bytes aligned, 
		// no boundary limitations
		dma_ch[ch].pool = dma_pool_create("bcm476x_dma", NULL, sizeof(DMAC_LINKED_LIST)*n_elements, 32, 0);
		
		if(dma_ch[ch].pool)
		{
			PRINT_DBG("pool is at  - %08x\n", (unsigned int)dma_ch[ch].pool);
			//allocate mem from pool
			dma_ch[ch].ch_link_list = (DMAC_LINKED_LIST *) dma_pool_alloc(dma_ch[ch].pool, GFP_KERNEL, &addr);
			if(dma_ch[ch].ch_link_list == NULL)
			{
				//reset all pointers
				printk(KERN_ERR "can't allocate dma pool memory\n");
				dma_ch[ch].n_ll_elements = 0;
				dma_ch[ch].ch_link_list_phy = 0;

				//free the pool since pool_alloc failed
				dma_pool_destroy(dma_ch[ch].pool);
				dma_ch[ch].pool = 0;
			}
			else
			{
				dma_ch[ch].n_ll_elements = n_elements;
				PRINT_DBG("nodes are at  - %08x or %08X\n", (unsigned int)dma_ch[ch].ch_link_list, addr);
				//assign the physical address 
				dma_ch[ch].ch_link_list_phy = (DMAC_LINKED_LIST *)addr;
			}
		}
		else
		{
			//reset all pointers
			printk(KERN_ERR "can't create dma pool\n");
			dma_ch[ch].n_ll_elements = 0;
			dma_ch[ch].ch_link_list = 0;
			dma_ch[ch].ch_link_list_phy = 0;
		}		
	}
}


/******************************************************************************
NAME
   dma_tasklet_func

SYNOPSIS
   static void dma_tasklet_func(unsigned long data)
   
FUNCTION
   bottom half of interrupt handler. Calls the registered callback 
   
RETURNS
   -
******************************************************************************/
static void dma_tasklet_func(unsigned long data)
{
	
	int ch = 0;

	PRINT_DBG("tasklet function called \n");
	//process higher priority interrupt first
	for(ch = 0; ch < MAX_BCM476X_DMA_CHANNELS; ch++)
	{
		if(dma_ch[ch].state == BCM476X_DMA_INTERRUPTED)
		{
			//reset the  state
			dma_ch[ch].state = BCM476X_DMA_ASSIGNED;
			PRINT_DBG("calling int handler for ch - %d, with error code- %d\n", ch, dma_ch[ch].error);
			//callback with error status
			dma_ch[ch].irq_handler(dma_ch[ch].tag, dma_ch[ch].error);
			//reset error flag
			dma_ch[ch].error = 0;
			
		}
	}
	return;
}


/******************************************************************************
NAME
   bcm476x_tc_dma_interrupt

SYNOPSIS
   static irqreturn_t bcm476x_tc_dma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
   
FUNCTION
   Top half of DMA complete interrupt 
   
RETURNS
   -
******************************************************************************/

static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
bcm476x_tc_dma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
bcm476x_tc_dma_interrupt(int irq, void *dev_id)
#endif
{
	unsigned int mask = 1;
	int ch = 0;

	//read the source of interrupt
	volatile unsigned int tc_status = readl(IO_ADDRESS(DMA_R_DMACINTTCSTATUS_MEMADDR)) & 0x0ff;
	volatile unsigned int  err_status = readl(IO_ADDRESS(DMA_R_DMACINTERRORSTATUS_MEMADDR)) & 0x0ff;

	PRINT_DBG("tc interrupt: tc_status - %08X, err_status - %08X\n", tc_status, err_status);
    /* Need to flush here ?? */
	if(tc_status)
	{
		tasklet_schedule(&dma_tlet);

        while(tc_status)
        {
	        //check whether this channel had the interrupt and set state
	        if(mask & tc_status)
	        {
		        //clear the interrupt
		        writel(mask, IO_ADDRESS(DMA_R_DMACINTTCCLEAR_MEMADDR));
		        writel(mask, IO_ADDRESS(DMA_R_DMACINTERRCLR_MEMADDR));		
		        dma_ch[ch].state = BCM476X_DMA_INTERRUPTED;
		        //set the error flag
		        if(mask & err_status) dma_ch[ch].error = 1;
		        //set the bit for this channel to '0'
		        tc_status = tc_status & ~mask;
		        err_status = err_status & ~mask;
	        }
        	
	        //next channel
	        mask = mask << 1;
	        ch++;
        }
    }
    /*
     * clear the VIC interrupt
     */
	writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));
	PRINT_DBG("returning from TC interrupt\n");
	return IRQ_HANDLED;
}

/******************************************************************************
NAME
   bcm476x_err_dma_interrupt

SYNOPSIS
   static irqreturn_t bcm476x_err_dma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
   
FUNCTION
   Top half of DMA error interrupt 
   
RETURNS
   -
******************************************************************************/
static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
bcm476x_err_dma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
bcm476x_err_dma_interrupt(int irq, void *dev_id)
#endif
{
	unsigned int mask = 1;
	int ch = 0;
	//read the error status
	volatile unsigned int err_status = readl(IO_ADDRESS(DMA_R_DMACINTERRORSTATUS_MEMADDR)) & 0x0ff;
	PRINT_DBG("err_dma - err - %08X\n", err_status);
	
	if(err_status)
	{
		tasklet_schedule(&dma_tlet);
	}

	while(err_status)
	{
		//check whether this channel had the interrupt and set state
		if(mask & err_status)
		{
			//clear the interrupt
			writel(mask, IO_ADDRESS(DMA_R_DMACINTERRCLR_MEMADDR));		
			dma_ch[ch].state = BCM476X_DMA_INTERRUPTED;
			//set the error flag
			dma_ch[ch].error = 1;
			//set the bit for this channel to '0'
			err_status = err_status & ~mask;
		}
		//next channel
		mask = mask << 1;
		ch++;
	}

    /*
     * clear the VIC interrupt
     */
	writel(0, IO_ADDRESS(VIC1_R_VICADDRESS_MEMADDR));
	writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR)); // Clear VIC0 because VIC1 is daisy chained to VIC0
	
	return IRQ_HANDLED;
}

static struct irqaction bcm476x_tc_dma_irq = {
	.name		= "BCM476X DMA TC",
	.flags		= 0,
	.handler	= bcm476x_tc_dma_interrupt
};

static struct irqaction bcm476x_err_dma_irq = {
	.name		= "BCM476X DMA ERR",
	.flags		= 0,
	.handler	= bcm476x_err_dma_interrupt
};

/******************************************************************************
NAME
   bcm476x_enable_dma

SYNOPSIS
   static int __init bcm476x_enable_dma(void)
   
FUNCTION
   enables the DMA block.
   
RETURNS
   -
******************************************************************************/
static int __init bcm476x_enable_dma(void)
{
	int endianess = DMA_LITTLE_ENDIAN;
	int i;
	//enable DMA 
	writel((endianess << 1) | // endianness. 0 = Little Endian
			(1 << 0), IO_ADDRESS(DMA_R_DMACCONFIGURATION_MEMADDR));// Enable DMAC. 

	//Clear any pending intrrupt
	writel( 0x000000ff, IO_ADDRESS(DMA_R_DMACINTTCCLEAR_MEMADDR));   
	writel( 0x000000ff, IO_ADDRESS(DMA_R_DMACINTERRCLR_MEMADDR));

	//initialize the data structure
	for(i = 0; i < MAX_BCM476X_DMA_CHANNELS; i++)
	{
		//initialize
		dma_ch[i].ch_link_list = 0;
		dma_ch[i].ch_link_list_phy = 0;
		dma_ch[i].base_addr = (void *)get_ch_base_addr(i);
		dma_ch[i].state = BCM476X_DMA_FREE;
		dma_ch[i].ch_number = i;
		dma_ch[i].irq_handler = 0;
		dma_ch[i].tag = 0xffffffff;
		dma_ch[i].error = 0;
		dma_ch[i].pool = NULL;
		dma_ch[i].n_ll_elements = 0;
	}
	//register interrupts
	setup_irq(BCM4760_INTR_DMA_DONE, &bcm476x_tc_dma_irq);
	setup_irq(BCM4760_INTR_DMA_ERR, &bcm476x_err_dma_irq);

	//initialize the tasklet for this library
	tasklet_init(&dma_tlet, dma_tasklet_func, (unsigned long)&dma_tasklet_data);
	
	return 0;
}

/******************************************************************************
NAME
   bcm476x_disable_dma

SYNOPSIS
   static void __exit bcm476x_disable_dma(void)
   
FUNCTION
   disable the DMA block.
   
RETURNS
   -
******************************************************************************/
static void __exit bcm476x_disable_dma(void)
{
	//stop the tasklet
	tasklet_kill(&dma_tlet);
	//disable the DMA block
	writel( 0, IO_ADDRESS(DMA_R_DMACCONFIGURATION_MEMADDR));
}

/******************************************************************************
NAME
   width_in_bytes

SYNOPSIS
   static int width_in_bytes(unsigned int width)
   
FUNCTION
   return the # of bytes in "width"
   
RETURNS
   - width in bytes
******************************************************************************/
static int width_in_bytes(unsigned int width)
{
	int n_bytes = 0;

	//returns the # of bytes of data 
	switch(width)
	{
	case 0:
		n_bytes = 1;
		break;
	case 1:
		n_bytes = 2;
		break;
	case 2:
		n_bytes = 4;
		break;
	default:
		n_bytes = 0;
		break;
	}

	PRINT_DBG("n_bytes - %d, width - %d\n", n_bytes, width);

	return n_bytes;
}

/******************************************************************************
NAME
   n_ll_required

SYNOPSIS
   static int n_ll_required(DMAC_CTRL_REG *src)
   
FUNCTION
   calculates total # of Link list nodes required for the current transfer. Max transfer size for 
   any node is 4095. If user requests more than 4095 then break the transfer in smaller 
   chunks of size less than or equal to 4095
   
RETURNS
   - # of link list nodes
******************************************************************************/

static int n_ll_required
(
	DMAC_CTRL_REG *src
)
{
	int n_ll = 0, i = 0;

	for(i = 0; i < src->n_addr; i++)
	{
		//max of MAX_TFR_SIZE unit can be transfered in one Link List node
		n_ll += (src->tfr_size[i] + MAX_TFR_SIZE)/MAX_TFR_SIZE;

		PRINT_DBG("n_ll - %d, index - %d\n", n_ll, i);

	}
	return n_ll;
}

/******************************************************************************
NAME
   create_dma_link_list

SYNOPSIS
   static int create_dma_link_list
	(
		DMAC_CTRL_REG *src, 
		DMAC_CTRL_REG *dst,
		DMAC_LINKED_LIST * link_list
	)
   
FUNCTION
   Creates scatter list for DMA 
   
RETURNS
   - # of link list buffers
******************************************************************************/
static int create_dma_link_list
(
	int ch,
	DMAC_CTRL_REG *src, 
	DMAC_CTRL_REG *dst
)
{
	int i = 0, n_src_bytes, n_dst_bytes, ll_index = 0;
	int src_n_list_items = 0;
	int dst_n_list_items = 0;
	uint control = 0;
	DMAC_LINKED_LIST * link_list = dma_ch[ch].ch_link_list;
	DMAC_LINKED_LIST * link_list_phy = dma_ch[ch].ch_link_list_phy;
		

	if(!link_list)
	{
		printk(KERN_ERR "can't allocate memory for dma link list buffer\n");
		return -1;
	}
	//# of link list items
	src_n_list_items = src->n_addr;
	dst_n_list_items = dst->n_addr;
	
	PRINT_DBG("src_Addr - %d, dst_addr - %d\n", src->n_addr, dst->n_addr);

	//make control word

	//src and dst data width
	control |= ( dst->width     << DMA_F_DWIDTH_R ) | 
				( src->width     << DMA_F_SWIDTH_R );
	n_src_bytes = width_in_bytes(src->width);
	n_dst_bytes = width_in_bytes(dst->width);
	// Destination burst size. This is the number of bytes that will be transferred when the 
	// peripheral requests a burst of data. (Set to 256 bytes)
	// Source burst size. (Memory boundary size when transferring from memory). 

	//src and dst burst size
	control |= ( dst->burst_sz << DMA_F_DBSIZE_R )  | 
				( src->burst_sz << DMA_F_SBSIZE_R );

	//address increment

	control |=  (src->incr ?  DMA_F_SI_MASK : 0) | //src increment settings 
				(dst->incr ?  DMA_F_DI_MASK : 0); //dst increment settings	

	PRINT_DBG("control before loop - %08X\n", control);


	for(i = 0; i < src_n_list_items; i++)
	{
		int j = 0, size = 0;
		for(j = 0; j < (src->tfr_size[i] + MAX_TFR_SIZE)/MAX_TFR_SIZE; j++)
		{

			PRINT_DBG("ll_index - %d\n", ll_index);

			//copy the byte address
			if(src->incr)
			{
				link_list[ll_index].src_addr = src->addr[i] + j*MAX_TFR_SIZE*n_src_bytes;
			}
			else
			{
				link_list[ll_index].src_addr = src->addr[i];
			}

			PRINT_DBG("src_addr - %08X\n", link_list[ll_index].src_addr);
			//dst address
			if(dst->incr)
			{
				//possible that only one dst addr is specified
				unsigned int base = (dst_n_list_items< i) ? dst->addr[0] : dst->addr[i];
				link_list[ll_index].dst_addr = base + j*MAX_TFR_SIZE*n_dst_bytes;
			}
			else
			{
				link_list[ll_index].dst_addr = (dst_n_list_items - 1 < i) ? dst->addr[0] : dst->addr[i];
			}

			PRINT_DBG("dst_addr - %08X\n", link_list[ll_index].dst_addr);
			
			if((i == src_n_list_items -1) && 
				(j == (src->tfr_size[i] + MAX_TFR_SIZE)/MAX_TFR_SIZE -1))
			{
				//to stop the DMA..should be only for the last node
				link_list[ll_index].next_lli = 0;
			}
			else
			{
				//assign the physical address of the next node
				link_list[ll_index].next_lli = (uint)(&link_list_phy[ll_index+1]);
			}
			PRINT_DBG("next_lli - %08X\n", link_list[ll_index].next_lli);

			//for the last node size is the remainder otherwise size is MAX_TFR_SIZE
			size = (j == (src->tfr_size[i] + MAX_TFR_SIZE)/MAX_TFR_SIZE - 1) ?
							(src->tfr_size[i] - j*MAX_TFR_SIZE) : MAX_TFR_SIZE;

			PRINT_DBG("size - %d\n", size);

			//transfer size...12 bits for transfer size
			link_list[ll_index].control = control | (size << DMA_F_TRANSFERSIZE_R);

			PRINT_DBG("control - %08X\n", link_list[ll_index].control);
			ll_index++;
		}
		//interupt setting..don't enable interrupts for the nodes created locally due to size constraints
		link_list[ll_index -1].control |= (src->flags & BCM476X_EN_DMA_ALL_TC_INT) ? DMA_F_I_MASK : 0;
	}
	//set the interrupt for the last LLI
	if(src->flags & BCM476X_EN_DMA_LAST_TC_INT)
	{
		link_list[ll_index -1].control |= DMA_F_I_MASK;
		PRINT_DBG("last control - %08X\n", link_list[ll_index - 1].control);
	}
	
	return ll_index;
}

/******************************************************************************
NAME
   allocate_dma_link_list

SYNOPSIS
   int allocate_dma_link_list(int ch, int n_bytes, int width)
   
FUNCTION
   allocate memory in advance to avoid alloc & free while setting up DMA control registers or link list
   
RETURNS
   - 1 if allocation is sucessful; 0 if failed to allocate
******************************************************************************/
int allocate_dma_link_list(int ch, int n_bytes_to_tfr, int width)
{
	//do the allocation only when channel is assigned and not in use
	if(dma_ch[ch].state == BCM476X_DMA_ASSIGNED)
	{
		//# of bytes in the width
		int n_bytes = width_in_bytes(width);
		//# of width units to be trabsfered
		int n_units = (n_bytes_to_tfr + n_bytes)/n_bytes;
		//# of link nodes required
		int n_ll = (n_units + MAX_TFR_SIZE)/MAX_TFR_SIZE;
		PRINT_DBG("channel %d pre-allocated %d nodes\n", ch, n_ll);
		//allocate nodes
		allocate_dma_link_list_nodes(ch, n_ll);

		//return 1 if allocation is successful
		if(dma_ch[ch].ch_link_list)
			return 1;
	}
	return 0;
}

/******************************************************************************
NAME
   bcm476x_initialize_dma_chain

SYNOPSIS
   int bcm476x_initialize_dma_chain( 
		const int ch, 

		DMAC_CTRL_REG *src, // src side config data
		DMAC_CTRL_REG *dst, // dst side config data
		DMAC_CFG_REG *cfg, //config data

		DMAC_CH_REG *ch_reg
	)
   
FUNCTION
    Initalize the data for DMAC_CH_REG. DMAC_CH_REG Can be used to call 
    light weight fast_dma_setup. Values are initalized in the ch_reg but not copied in the MAC
    register. User should call fast_dma_setup to start the DMA transfer
   
RETURNS
   - # of link list buffers
******************************************************************************/
int bcm476x_initialize_dma_chain( 
	const int ch, 

	DMAC_CTRL_REG *src, // src side config data
	DMAC_CTRL_REG *dst, // dst side config data
	DMAC_CFG_REG *cfg, //config data

	DMAC_CH_REG *ch_reg
)
{

	int ll_count = 0, n_ll_req;

	// 'ITC'            - Terminal Count Interrupt mask. Masks TC interrupt when cleared.
	 // 'IE'             - Interrupt error mask. Masks error interrupt when cleared.
	uint cfg_reg = DMA_F_ITC_MASK | DMA_F_IE_MASK;

	if((ch >= MAX_BCM476X_DMA_CHANNELS) ||
			(dma_ch[ch].state == BCM476X_DMA_IN_USE))
	{
	
		printk(KERN_INFO "setpdma , channel number is wrong - %d, %d\n", ch, dma_ch[ch].state);
		return -1;
	}

	n_ll_req = n_ll_required(src);
	PRINT_DBG("l_ll_req - %08X\n", n_ll_req);

	if(n_ll_req < 1)
	{
		printk(KERN_INFO "no data to transfer in the channel - %d\n", ch);
		return -1;
	}
	allocate_dma_link_list_nodes(ch, n_ll_req);
	if(!dma_ch[ch].ch_link_list)
	{
		printk(KERN_INFO "can't allocate memory for dma link list buffer\n");
		return -ENOMEM;
	}

	ll_count = create_dma_link_list(ch, src, dst);
	if(ll_count != n_ll_req)
	{
		printk(KERN_INFO "# of link buffers created is not equal to required nodes\n");
		return -1;
	}


	/* Set config word */
	// 'FlowCntrl'      - Flow control method.
	cfg_reg |= cfg->trans_type << DMA_F_FLOWCNRTL_R;
	// 'DestPeripheral' - Destination peripheral number to associate with this channel.Ignored for 'from memory' transfers.
	cfg_reg |= cfg->dst_id     << DMA_F_DESTPERIPHERAL_R;
	// 'SrcPeripheral'  - Source peripheral number to associate with this channel. Ignored for 'from memory' transfers.
	cfg_reg |= cfg->src_id     << DMA_F_SRCPERIPHERAL_R;

	/* set up data for the first node...Ideally only LLI and config 
	should be set but need to set src, dst & control to get it work*/
	ch_reg->config = cfg_reg;
	ch_reg->src_addr = dma_ch[ch].ch_link_list[0].src_addr;
	ch_reg->dst_addr = dma_ch[ch].ch_link_list[0].dst_addr;
	ch_reg->control = dma_ch[ch].ch_link_list[0].control;
	ch_reg->lli = dma_ch[ch].ch_link_list[0].next_lli;
	return ll_count;	
}

/******************************************************************************
NAME
   bcm476x_dma_fast_setup

SYNOPSIS
   void bcm476x_dma_fast_setup( 
		const int ch,
		DMAC_CH_REG *ch_reg
	)
   
FUNCTION
    light weight fast_dma_setup. Added to call DMA_SETUP form the ISR. bcm476x_setup_dma_chain
    is very expensive to use in ISR for real time playback
    Data is copied from the ch_reg, so that caller don't need to re-initialize the data for the next setup
    If data (like src or dst address) needs to be changed for the next transfer then caller should 
    update the values and call this function
   
RETURNS
   - 
******************************************************************************/
void bcm476x_dma_fast_setup( 
	const int ch,
	DMAC_CH_REG *ch_reg
)
{
	//set channel registers
	uint *chBaseAddr = (uint *)dma_ch[ch].base_addr;
	writel(ch_reg->lli, (uint *)(chBaseAddr + BCM476X_DMAC_CH_LLIREG_OFFSET/sizeof(uint)));
	writel(ch_reg->src_addr, (uint *)(chBaseAddr + BCM476X_DMAC_CH_SRCADDR_OFFSET/sizeof(uint)));
	writel(ch_reg->dst_addr, (uint *)(chBaseAddr + BCM476X_DMAC_CH_DESTADDR_OFFSET/sizeof(uint)));
	writel(ch_reg->control, (uint *)(chBaseAddr + BCM476X_DMAC_CH_CONTROL_OFFSET/sizeof(uint)));
	//update the state before enabling the channel to avoid race condition
	dma_ch[ch].state = BCM476X_DMA_IN_USE;
	//enable the channel too
	writel(ch_reg->config | 0x01, (uint *)(chBaseAddr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));

}

/******************************************************************************
NAME
   bcm476x_setup_dma_chain

SYNOPSIS
   int bcm476x_setup_dma_chain( 
		const int ch, 

		DMAC_CTRL_REG *src, // src side config data
		DMAC_CTRL_REG *dst, // dst side config data
		DMAC_CFG_REG *cfg //config data
	)
   
FUNCTION
    Setup DMA register data and link/scattter list
   
RETURNS
   - # of link list buffers
******************************************************************************/
int bcm476x_setup_dma_chain( 
	const int ch, 

	DMAC_CTRL_REG *src, // src side config data
	DMAC_CTRL_REG *dst, // dst side config data
	DMAC_CFG_REG *cfg //config data
)
{

	uint *chBaseAddr = (uint *)dma_ch[ch].base_addr;
	int ll_count = 0, n_ll_req;

	// 'ITC'            - Terminal Count Interrupt mask. Masks TC interrupt when cleared.
	 // 'IE'             - Interrupt error mask. Masks error interrupt when cleared.
	uint cfg_reg = DMA_F_ITC_MASK | DMA_F_IE_MASK;

	if((ch >= MAX_BCM476X_DMA_CHANNELS) ||
			(dma_ch[ch].state == BCM476X_DMA_IN_USE))
	{
	
		printk(KERN_INFO "setpdma , channel number is wrong - %d, %d\n", ch, dma_ch[ch].state);
		return -1;
	}

	n_ll_req = n_ll_required(src);
	PRINT_DBG("l_ll_req - %08X\n", n_ll_req);

	if(n_ll_req < 1)
	{
		printk(KERN_INFO "no data to transfer in the channel - %d\n", ch);
		return -1;
	}
	allocate_dma_link_list_nodes(ch, n_ll_req);
	if(!dma_ch[ch].ch_link_list)
	{
		printk(KERN_INFO "can't allocate memory for dma link list buffer\n");
		return -ENOMEM;
	}

	ll_count = create_dma_link_list(ch, src, dst);
	if(ll_count != n_ll_req)
	{
		printk(KERN_INFO "# of link buffers created is not equal to required nodes\n");
		return -1;
	}



	/* set up data for the first node...Ideally only LLI and config 
	should be set but need to set src, dst & control to get it work*/
	
	writel(dma_ch[ch].ch_link_list[0].next_lli, (uint *)(chBaseAddr + BCM476X_DMAC_CH_LLIREG_OFFSET/sizeof(uint)));
	writel(dma_ch[ch].ch_link_list[0].src_addr, (uint *)(chBaseAddr + BCM476X_DMAC_CH_SRCADDR_OFFSET/sizeof(uint)));
	writel(dma_ch[ch].ch_link_list[0].dst_addr, (uint *)(chBaseAddr + BCM476X_DMAC_CH_DESTADDR_OFFSET/sizeof(uint)));
	writel(dma_ch[ch].ch_link_list[0].control, (uint *)(chBaseAddr + BCM476X_DMAC_CH_CONTROL_OFFSET/sizeof(uint)));

	/* Set control word */
	
	/* Set config word */
	// 'FlowCntrl'      - Flow control method.
	cfg_reg |= cfg->trans_type << DMA_F_FLOWCNRTL_R;
	// 'DestPeripheral' - Destination peripheral number to associate with this channel.Ignored for 'from memory' transfers.
	cfg_reg |= cfg->dst_id     << DMA_F_DESTPERIPHERAL_R;
	// 'SrcPeripheral'  - Source peripheral number to associate with this channel. Ignored for 'from memory' transfers.
	cfg_reg |= cfg->src_id     << DMA_F_SRCPERIPHERAL_R;
	writel(cfg_reg, (uint *)(chBaseAddr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));

	return ll_count;	
}

/******************************************************************************
NAME
   bcm476x_release_dma_channel

SYNOPSIS
   int bcm476x_release_dma_channel(int ch)
   
FUNCTION
    release the DMA channel or assign the state to be free
   
RETURNS
   - ch #
******************************************************************************/
int bcm476x_release_dma_channel(int ch)
{
	uint* chBaseAddr = dma_ch[ch].base_addr;
	
	if(ch >= MAX_BCM476X_DMA_CHANNELS)
	{
		printk(KERN_INFO "release, channel number is wrong - %d\n", ch);
		return -1;
	}
	//cleanup time..
    if (dma_ch[ch].pool)
    {
	    dma_pool_free(dma_ch[ch].pool, dma_ch[ch].ch_link_list, (unsigned int)dma_ch[ch].ch_link_list_phy);
	    dma_pool_destroy(dma_ch[ch].pool);
    }

	dma_ch[ch].ch_link_list = 0;
	dma_ch[ch].ch_link_list_phy = 0;
	dma_ch[ch].pool = NULL;
	dma_ch[ch].n_ll_elements = 0;
	dma_ch[ch].irq_handler = 0;
	dma_ch[ch].tag = 0xffffffff;


	//clear any interrupts 
	writel( 1 << ch, IO_ADDRESS(DMA_R_DMACINTTCCLEAR_MEMADDR));  
	writel( 1 << ch, IO_ADDRESS(DMA_R_DMACINTERRCLR_MEMADDR));
	writel(0, (volatile uint *)(chBaseAddr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));	
	
	//mark the state to be free
	dma_ch[ch].state = BCM476X_DMA_FREE;
	return ch;
}

/******************************************************************************
NAME
   bcm476x_set_dma_halt_bit

SYNOPSIS
   int bcm476x_set_dma_halt_bit( int ch, int halt )
   
FUNCTION
    disable any furhter interrupts on this channel
   
RETURNS
   - ch #
******************************************************************************/
int bcm476x_set_dma_halt_bit( int ch, int halt )
{
	uint* ch_base_addr = dma_ch[ch].base_addr;
	volatile uint data;
	unsigned long irq_flags;
	if(ch >= MAX_BCM476X_DMA_CHANNELS)
	{
		printk(KERN_INFO "enable, channel number is wrong - %d\n", ch);
		return -1;
	}
	//check whether there is any activity on this channel
	spin_lock_irqsave(&dma_ch[ch].lock, irq_flags);
	if(dma_ch[ch].state != BCM476X_DMA_IN_USE)
	{
		//not enabled so need to halt
		spin_unlock_irqrestore(&dma_ch[ch].lock, irq_flags);
		return -1;
	}

	spin_unlock_irqrestore(&dma_ch[ch].lock, irq_flags);
	
	//set the halt bit in the channel config register
	data = *(volatile uint *)((uint)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint))); 
	data = ((halt == 0) ? (data & ~ DMA_F_H_MASK) : (data | DMA_F_H_MASK));
	writel(data, (volatile uint *)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));

	return ch;
}

/******************************************************************************
NAME
   bcm476x_disable_dma_channel

SYNOPSIS
   int bcm476x_disable_dma_channel( int ch, int immediately)
   
FUNCTION
    if immediately == 1, then set the enable bit to 0 immediately
    if immediately == 0, then first set the halt bit to further diable any interrupts. Wait for 
    data in the fifo to be flushed then set the enable bit to 0
   
RETURNS
   - ch #
******************************************************************************/
int bcm476x_disable_dma_channel( int ch, int immediately)
{
	uint* ch_base_addr = dma_ch[ch].base_addr;
	volatile uint data;
	unsigned long irq_flags;
	if(ch >= MAX_BCM476X_DMA_CHANNELS)
	{
		printk(KERN_INFO "enable, channel number is wrong - %d\n", ch);
		return -1;
	}
	//check whether caller wants to disable the channel cleanly
	if(!immediately)
	{
		//disable any furhter interrupts on this channel 
		if(bcm476x_set_dma_halt_bit(ch, 1) == ch)
		{
			//start assuming there is some data in the fifo
			data =  DMA_F_A_MASK;
			//get the value of channle config reg
			while (data & DMA_F_A_MASK)
			{
				data = *(volatile uint *)((uint)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));
			}
		}
	}
	
	//reset the enable bit in the channel config register
	data = *(volatile uint *)((uint)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint))); 
	writel(data & ~DMA_F_E_MASK , (volatile uint *)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));

	//update the state
	spin_lock_irqsave(&dma_ch[ch].lock, irq_flags);
	dma_ch[ch].state = BCM476X_DMA_ASSIGNED;
	spin_unlock_irqrestore(&dma_ch[ch].lock, irq_flags);
	
	return ch;
}

/******************************************************************************
NAME
   bcm476x_enable_dma_channel

SYNOPSIS
   int bcm476x_enable_dma_channel( int ch )
   
FUNCTION
    Enable the DMA channel. DMA trasnsfer starts right away after the bit is set.
   
RETURNS
   - ch #
******************************************************************************/
int bcm476x_enable_dma_channel( int ch )
{
	uint* ch_base_addr = dma_ch[ch].base_addr;
	volatile uint data;

	if(ch >= MAX_BCM476X_DMA_CHANNELS)
	{
		printk(KERN_INFO "enable, channel number is wrong - %d\n", ch);
		return -1;
	}
	
	//update the state before enabling the channel to avoid race condition
	dma_ch[ch].state = BCM476X_DMA_IN_USE;
	//set the enable bit in the channel config register
	data = *(volatile uint *)((uint)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint))); 
	writel(data | DMA_F_E_MASK , (volatile uint *)(ch_base_addr + BCM476X_DMAC_CH_CONFIG_OFFSET/sizeof(uint)));

	return ch;
}

/******************************************************************************
NAME
   bcm476x_get_dma_channel_state

SYNOPSIS
   int bcm476x_get_dma_channel_state( int ch )
   
FUNCTION
    return the state of the channel.
   
RETURNS
   - state
******************************************************************************/
int bcm476x_get_dma_channel_state( int ch )
{
	if(ch >= MAX_BCM476X_DMA_CHANNELS)
	{
		printk(KERN_INFO "get_channel, channel number is wrong - %d\n", ch);
		return BCM476X_DMA_BAD_CH;
	}

	return dma_ch[ch].state;
}

/******************************************************************************
NAME
   bcm476x_request_dma_channel

SYNOPSIS
   int bcm476x_request_dma_channel( void)
   
FUNCTION
    return a free channel or state is BCM476X_DMA_FREE
   
RETURNS
   - return -1 if no channel is free
******************************************************************************/
int bcm476x_request_dma_channel(void)
{
	int i = 0;
	int ch = -1;
	unsigned long irq_flags;
	for(i = 0; i < MAX_BCM476X_DMA_CHANNELS; i++)
	{
		spin_lock_irqsave(&dma_ch[i].lock, irq_flags);
		if(dma_ch[i].state == BCM476X_DMA_FREE)
		{
			//mark it assigned
			dma_ch[i].state = BCM476X_DMA_ASSIGNED;
			ch = i;
			PRINT_DBG("assigned new ch - %d\n", ch);
			spin_unlock_irqrestore(&dma_ch[i].lock, irq_flags);
			break;
		}
		spin_unlock_irqrestore(&dma_ch[i].lock, irq_flags);
	}
	//return -1 if no channel is free
	return ch;
}

/******************************************************************************
NAME
   bcm476x_register_dma_handler

SYNOPSIS
   void bcm476x_register_dma_handler
	(
		const int ch,
		unsigned long tag,
		void (*callback_fn)(unsigned long, unsigned long)
	)
   
FUNCTION
    register the callback and tag. ISR will call the callback function upon receiving the interrupt
   
RETURNS
   - 
******************************************************************************/
void bcm476x_register_dma_handler
(
	const int ch,
	unsigned long tag,
	void (*callback_fn)(unsigned long tag, unsigned long error))
{
	/* register the callback and TAG. 
	Tag can be used as the ID, if a module is using multiple DMA channels and has
	one common handler. Tag can also be used as the memory address where debug 
	info is getting stored
	*/
	if((ch < MAX_BCM476X_DMA_CHANNELS) && 
			(dma_ch[ch].state != BCM476X_DMA_IN_USE))
	{
		dma_ch[ch].tag = tag;
		dma_ch[ch].irq_handler = callback_fn;
	}
}

EXPORT_SYMBOL(bcm476x_enable_dma_channel);
EXPORT_SYMBOL(bcm476x_set_dma_halt_bit);
EXPORT_SYMBOL(bcm476x_disable_dma_channel);
EXPORT_SYMBOL(bcm476x_request_dma_channel);
EXPORT_SYMBOL(bcm476x_release_dma_channel);
EXPORT_SYMBOL(bcm476x_setup_dma_chain);
//EXPORT_SYMBOL(setup_dma);
EXPORT_SYMBOL(bcm476x_register_dma_handler);

EXPORT_SYMBOL(bcm476x_get_dma_channel_state);

EXPORT_SYMBOL(bcm476x_dma_fast_setup);
EXPORT_SYMBOL(bcm476x_initialize_dma_chain);
module_init(bcm476x_enable_dma);
module_exit(bcm476x_disable_dma);

