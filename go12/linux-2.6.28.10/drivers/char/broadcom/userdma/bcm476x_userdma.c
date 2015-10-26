/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
 * linux/drivers/userdma/bcm476x_userdma.c
 *
 * Broadcom BCM476X userdma specific driver.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/memory.h>
#include <asm/io.h>
#include <asm/arch/dma.h>
#include <asm/cacheflush.h>
#include <linux/broadcom/userdma.h>
#include "bcm476x_userdma.h"

extern struct miscdevice g_userdma_dev;
static struct completion userdma_comp[MAX_BCM476X_DMA_CHANNELS];
static int userdma_err[MAX_BCM476X_DMA_CHANNELS];

/* dma fnish interrupt handler
 *
 */

void userdma_handler (int dma_ch, unsigned long err_code)
{
    if (err_code) printk("userdma_handler: dma op failed 0x%x\n", (u32)err_code);
    userdma_err[dma_ch] = err_code;
    //printk("userdma_handler: completed, ch=%d\n", dma_ch);
    complete(&userdma_comp[dma_ch]);
}

/* setup the dma structure required by the dma library
 *
 */

static int bcm476x_userdma_setup( userdma_ioctl_t * p, dma_addr_t *src_addrs, dma_addr_t *dest_addrs, DMAC_CH_REG * ch_reg)
{
    DMAC_CTRL_REG mem_src;
    DMAC_CTRL_REG mem_dst;
    DMAC_CFG_REG mem_cfg;
    int dma_ch, i;

    //  mem_src.burst_sz = BCM28XX_DMAC_BurstSize_4;
    mem_src.burst_sz = p->burst;
    mem_src.incr = p->src.incr;
    mem_src.width = p->src.width;
    mem_src.n_addr = p->src.n_addrs;
    mem_src.flags = BCM476X_EN_DMA_LAST_TC_INT;
    mem_src.addr = (int *) src_addrs;
    mem_src.tfr_size = p->src.sizes;

    if (p->verbose) {
        printk("dma source address and len:\n");
        for (i =0; i < p->src.n_addrs; i++) {
            printk("   0x%x, %d\n",  (u32) src_addrs[i], p->src.sizes[i]);
        }
    }

    // mem_dst.burst_sz = BCM28XX_DMAC_BurstSize_4;
    mem_dst.burst_sz = p->burst;
    mem_dst.incr = p->dest.incr;
    mem_dst.width = p->dest.width;
    mem_dst.n_addr = p->dest.n_addrs;
    mem_dst.flags = BCM476X_EN_DMA_LAST_TC_INT;
    mem_dst.addr = (int *) dest_addrs;
    mem_dst.tfr_size = p->dest.sizes;
    //mem_cfg.src_id = 0;
    //mem_cfg.dst_id = 0;
    mem_cfg.src_id = 1; // use slave bus for faster transfer
    mem_cfg.dst_id = 1; // use slave bus for faster transfer
    mem_cfg.trans_type = BCM476X_DMAC_MEM2MEM_CTL;

    if (p->verbose) {
        printk("dma destination address and len:\n");
        for (i =0; i < p->dest.n_addrs; i++) {
            printk("   0x%x, %d\n", (u32) dest_addrs[i], p->dest.sizes[i]);
        }
    }

    if (p->dma_ch < 0)
    {
        bcm476x_userdma_alloc_channel(&dma_ch);
    }
    else
    {
        dma_ch = p->dma_ch;
    }

    if (dma_ch <0 || dma_ch > MAX_BCM476X_DMA_CHANNELS) {
        printk("dma request got channel number wrong\n");
        return -1;
    }

    init_completion (&userdma_comp[dma_ch]);

#if 0
    if (bcm476x_setup_dma_chain(
		          dma_ch,
		          &(mem_src),
		          &(mem_dst),
		          &(mem_cfg)
		          ) <= 0 ) {
        printk("dma chain setup failed\n");

    }
    bcm476x_enable_dma_channel(dma_ch) ;
#else
    if (bcm476x_initialize_dma_chain(
		          dma_ch,
		          &(mem_src),
		          &(mem_dst),
		          &(mem_cfg),
		          ch_reg
		          ) < 0 ) {
        printk("dma chain setup failed\n");
    }
#endif

    return dma_ch;
}

/*
 * Verify Data after each write
 */
static int userdma_verify(userdma_mem_t *from_mem, userdma_mem_t *to_mem, int sz, int verbose)
{
    u32 *to_virt, *from_virt;

    if (verbose) 
        printk("userdma_verify: from: (phys %p, virt %p) to: (phys %p, virt %p)\n", 
            from_mem->phys, from_mem->virt, to_mem->phys, to_mem->virt);
    from_virt = from_mem->virt;
    to_virt = to_mem->virt;
	while (sz)
	{
		if (*to_virt == *from_virt) {
			from_virt++; to_virt++; sz -= 4;
		} else {
			printk("Verify Failed : Data at 0x%08x is 0x%08x, Data at 0x%08x is 0x%08x\n", (u32)from_virt, *from_virt, (u32)to_virt, *to_virt);
			return -1;
		}
	}
	return 0;
}

/* kick start the dma operation
 *
 */

static int userdma_start_dma(int dma_ch, DMAC_CH_REG * ch_reg, int verbose)
{
    userdma_err[dma_ch] = 0;
    bcm476x_dma_fast_setup(dma_ch, ch_reg);

    if (verbose)  printk("DMA start for channel - %d\n", dma_ch);

    wait_for_completion(&userdma_comp[dma_ch]);

    if (verbose)  printk("DMA complete for channel - %d\n", dma_ch);

    bcm476x_disable_dma_channel(dma_ch, 1);
    //printk("dma completed\n");

    return userdma_err[dma_ch];
}

/* worker function to do the dma operation
 *
 */

int bcm476x_userdma_xfer(  userdma_ioctl_t * p )
{
    int dma_ch, i;
    DMAC_CH_REG mem_ch_reg;
    int rc=0;
    dma_addr_t src_dma[12];
    dma_addr_t dest_dma[12];
    
    if (p->verbose) printk("bcm476x_userdma_xfer: ch=%d\n", p->dma_ch);
    if (p->src.n_addrs > 12 || p->dest.n_addrs > 12)
        return -1;
    for (i = 0 ; i < p->src.n_addrs; i++)
        copy_from_user(&src_dma[i], &(p->src.addrs[i]->phys), sizeof(dma_addr_t));
    for (i = 0 ; i < p->dest.n_addrs; i++)
        copy_from_user(&dest_dma[i], &(p->dest.addrs[i]->phys), sizeof(dma_addr_t));

    dma_ch = bcm476x_userdma_setup(p, src_dma, dest_dma, &mem_ch_reg);
    if (dma_ch != -1)
    {
        if (p->loop == 0)
            printk("MEMDMA UNLIMTED NUMBER OF TIMES : From %p To %p\n", p->src.addrs[0]->phys, p->dest.addrs[0]->phys);
        do 
        {
            rc = userdma_start_dma(dma_ch, &mem_ch_reg, p->verbose);
            if (rc) 
                break;
            if (p->verify) 
            {
                for (i = 0; i < p->dest.n_addrs; i++) 
                {
                    rc = userdma_verify(p->src.addrs[i], p->dest.addrs[i], p->dest.sizes[i], p->verbose);
                    if (rc == -1) {
                        printk("Verify failed for dma-transfer in loop %d\n", p->loop);
                        return rc;
                    }
                }
            }
            schedule();
        } while (--p->loop);


        //p->rate = (mem_get_rpl() - rplc_cur) / RPL_CLK;
        //printk("rate is %x %d\n", p->rate , p->rate);
        if (p->dma_ch < 0)  // dynamic channel ?
            bcm476x_release_dma_channel(dma_ch);
    }

    return rc;
}
EXPORT_SYMBOL(bcm476x_userdma_xfer);

int bcm476x_userdma_alloc_channel(int *ch)
{
    int dma_ch;
    int try = 5;
    while ( try-- > 0 && (dma_ch = bcm476x_request_dma_channel()) == -1) {
        schedule_timeout(10 * HZ);
        printk("dma chan request failed\n");
    }
    if (dma_ch != -1)
    {
         bcm476x_register_dma_handler(
                           dma_ch,
                           dma_ch,
                           userdma_handler
                           );
    }
    *ch = dma_ch;
    return (dma_ch < 0) ? -1 : 0;
}

int bcm476x_userdma_release_channel(int ch)
{
    ch = bcm476x_release_dma_channel(ch);
    return (ch < 0) ? -1 : 0;
}

/* dma memory from source(s) to destination(s)
 *
 */
int    bcm476x_userdma_do_dma( userdma_ioctl_t *arg )
{
    userdma_ioctl_t param;

    if (access_ok( VERIFY_READ, (void __user *) arg, sizeof(userdma_ioctl_t) ))
        copy_from_user( &param, (void *)arg,  sizeof(userdma_ioctl_t) ) ;
    else
    {
        printk("access read failed\n"); return -1; 
    }
 
    return  bcm476x_userdma_xfer(&param);
}

