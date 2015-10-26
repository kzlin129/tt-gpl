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

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <asm/semaphore.h>
#include <asm/arch/dma.h>
#include <linux/dma-mapping.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

typedef struct
{
    void       *virtPtr;
    dma_addr_t  physPtr;
    size_t      numBytes;

} dma_mem_t;

/* ---- Private Variables ------------------------------------------------ */

int   gDmaHandle[ MAX_BCM476X_DMA_CHANNELS ];
int   gDmaDone = 0;
struct semaphore    gDmaDoneSem;

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*   Allocates a block of memory
*
***************************************************************************/

static void *alloc_mem( dma_mem_t *mem, size_t numBytes )
{
    mem->numBytes = numBytes;
    mem->virtPtr = dma_alloc_writecombine( NULL, numBytes, &mem->physPtr, GFP_KERNEL );

    if ( mem->virtPtr == NULL )
    {
        printk( KERN_ERR "dma_alloc_writecombine of %d bytes failed\n", numBytes );
    }

    printk( "dma_alloc_writecombine of %d bytes returned virtPtr: 0x%08lx physPstr: %08x\n",
            numBytes, (unsigned long)mem->virtPtr, mem->physPtr );

    return mem->virtPtr;
}

static void free_mem( dma_mem_t *mem )
{
    dma_free_writecombine(NULL, mem->numBytes, mem->virtPtr, mem->physPtr);
}

/****************************************************************************
*
*   Handler called when the DMA finishes.
*
***************************************************************************/

static void dma_handler( int channel, int error )
{
    printk( "dma_handler called: channel: %d, error = 0x%x\n", channel, error );
    gDmaDone |= (1 << channel);
    up( &gDmaDoneSem );
}

/****************************************************************************
*
*   Called to perform module initialization when the module is loaded
*
***************************************************************************/

#define ALLOC_SIZE  1024
#define START_COUNT 0xA000

static int __init dma_test_init( void )
{
    int         rc;
    int         i;
    dma_mem_t   src;
    dma_mem_t   dst;
    uint16_t   *srcInt;
    uint16_t   *dstInt;
    int         numWords;
    uint16_t    counter;
	int         ch;
    int  tf_size[1], src_list[1], dest_list[1];
	DMAC_CTRL_REG src_ctrl; // src side config data
	DMAC_CTRL_REG dst_ctrl; // dst side config data
	DMAC_CFG_REG cfg_reg; //config data

    printk( "\n========== Starting DMA Test ==========\n\n" );


    if ( alloc_mem( &src, ALLOC_SIZE ) == NULL )
    {
        return -ENOMEM;
    }
    if ( alloc_mem( &dst, ALLOC_SIZE ) == NULL )
    {
        free_mem(&src);
        return -ENOMEM;
    }

    for ( i = 0; i < MAX_BCM476X_DMA_CHANNELS; i++ )
    {
        ch = bcm476x_request_dma_channel();
        gDmaHandle[ i ] = ch;
        if ( ch < 0 )
            printk("Failed to allocate dma channel %d\n", i);
        else 
        {
            printk("Allocated handle: 0x%x\n", gDmaHandle[ i ] );
            bcm476x_register_dma_handler( ch, ch, dma_handler);
        }
    }

    for ( ch = 0; ch < MAX_BCM476X_DMA_CHANNELS; ch++ )
    {
        gDmaDone = 0;

        numWords = ALLOC_SIZE / sizeof( *srcInt );

        srcInt = src.virtPtr;
        dstInt = dst.virtPtr;
        counter = START_COUNT + (ch << 12);
        for ( i = 0; i < numWords; i++ )
        {
            *srcInt++ = counter++;
            *dstInt++ = 0;
        }

        sema_init( &gDmaDoneSem, 0 );

        /* Setup for transfer */
        tf_size[0] = ALLOC_SIZE;
        src_list[0] = src.physPtr;
        dest_list[0] = dst.physPtr;
        //prepare the DMA register data
        src_ctrl.burst_sz = BCM476X_DMAC_BURST_SIZE_4;
        src_ctrl.incr = 1;
        src_ctrl.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
        src_ctrl.addr = src_list;
        src_ctrl.tfr_size = tf_size;
        src_ctrl.n_addr = 1;
        src_ctrl.flags = BCM476X_EN_DMA_LAST_TC_INT;

        dst_ctrl.burst_sz = BCM476X_DMAC_BURST_SIZE_4;
        dst_ctrl.incr = 1;
        dst_ctrl.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
        dst_ctrl.addr = dest_list;
        dst_ctrl.tfr_size = tf_size;
        dst_ctrl.n_addr = 1;
        dst_ctrl.flags = BCM476X_EN_DMA_LAST_TC_INT;

        cfg_reg.src_id = 0; 
        cfg_reg.dst_id = 0;
        cfg_reg.trans_type = BCM476X_DMAC_MEM2MEM_CTL;

        if (( rc = bcm476x_setup_dma_chain( ch, &src_ctrl, &dst_ctrl, &cfg_reg)) == 0 )
        {
            printk( KERN_ERR "bcm476x_setup_dma_chain failed: %d\n", rc );
            goto free_resource;
        }

        /* Enable the dma */
        bcm476x_enable_dma_channel(ch);

        printk( "Waiting for DMA to complete\n" );
        if (( rc = down_interruptible( &gDmaDoneSem )) != 0 )
        {
            printk( KERN_ERR "down_interruptible failed: %d\n", rc );
            goto free_resource;
        }
        printk( "DMA completed\n" );

        dstInt = dst.virtPtr;
        counter = START_COUNT + (ch << 12);
        for ( i = 0; i < numWords; i++ )
        {
            if ( *dstInt != counter )
            {
                printk( "Destination: i=%d, found 0x%4.4x, expecting 0x%4.4x\n",
                        i, *dstInt, counter );
                return -EIO;
            }
            dstInt++;
            counter++;
        }

        printk( "dma test succeeded for channel %d\n",ch);
    } /* Test all channels */

free_resource:
    free_mem(&src);
    free_mem(&dst);
    for ( i = 0; i < MAX_BCM476X_DMA_CHANNELS; i++ )
    {
        if (gDmaHandle[ i ] >= 0)
            bcm476x_release_dma_channel(gDmaHandle[ i ]);
    }

    printk( "\n========== Finished DMA Test ==========\n\n" );


    return rc;
}

/****************************************************************************
*
*   Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit dma_test_exit( void )
{
}

/****************************************************************************/

module_init( dma_test_init );
module_exit( dma_test_exit );

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION( "Broadcom DMA Test" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( "1.0" );

