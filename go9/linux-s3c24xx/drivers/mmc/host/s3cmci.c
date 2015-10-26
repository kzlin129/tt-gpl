/*
 *  linux/drivers/mmc/s3c2413mci.h - Samsung S3C2413 SDI Interface driver
 *
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include <asm/dma.h>
#include <asm/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/sizes.h>
#include <asm/mach/mmc.h>

#include <asm/arch/registers.h>
#include <asm/hardware/clock.h>
#include <asm/arch/dma.h>
#include "../core/sysfs.h"

#if CONFIG_MACH_TOMTOMGO
#include <barcelona/gopins.h>
#endif

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <barcelona/cpufreq_order.h>
#include <asm/arch/regs-clock.h>
#endif

#ifdef CONFIG_S3CMCI_DEBUG
#define DBG(x...)       printk(x)
#else
#define DBG(x...)       do { } while (0)
#endif

#include "../core/core.h"
#include "s3cmci.h"

#ifndef MHZ
#define MHZ (1000*1000)
#endif

#define DRIVER_NAME "s3c-sdi"
#define PFX DRIVER_NAME ": "


#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

typedef enum {
	DMAP_READ,
	DMAP_WRITE,
} eDMAPurpose_t;

#ifdef CONFIG_S3CMCI_DEBUG
uint cmd_rec[40];
uint cmd_idx = 0;
#endif

/* parameter for suicide */
struct s3c_sdi_host *s3c_sdi_host_poweroff = 0;

static struct s3c_dma_client s3c_sdi_dma_client = {
	.name		= "s3c-sdi-dma",
};

static void s3c_sdi_poweroff(struct s3c_sdi_host *host);

void s3c_sdi_emergency_poweroff(void)
{
	if (s3c_sdi_host_poweroff) {
		s3c_sdi_poweroff(s3c_sdi_host_poweroff);
	}
}

/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	tells the ISR when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
 */

static irqreturn_t s3c_sdi_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c_sdi_host *host;
	struct mmc_request *mrq;
	u32 sdi_csta, sdi_dsta, sdi_fsta;
	u32 sdi_cclear, sdi_dclear;
	unsigned long iflags;

	host = (struct s3c_sdi_host *)dev_id;

	/* Check for things not supposed to happen */
	if (!host)
		return IRQ_HANDLED;

	spin_lock_irqsave( &host->complete_lock, iflags);

	if ( host->complete_what==COMPLETION_NONE ) {
		goto clear_imask;
	}

	if (!host->mrq) {
		goto clear_imask;
	}

	mrq = host->mrq;

	sdi_csta = readl(host->base + S3C_SDICSTA);
	sdi_dsta = readl(host->base + S3C_SDIDSTA);
	sdi_fsta = readl(host->base + S3C_SDIFSTA);
	sdi_cclear = 0;
	sdi_dclear = 0;

	if (sdi_csta & S3C_SDICSTA_CMDTOUT) {
		mrq->cmd->error = MMC_ERR_TIMEOUT;
		DBG("cmd timeout\n");
		goto transfer_closed;
	}

	if (sdi_csta & S3C_SDICSTA_CMDSENT) {
		if (host->complete_what == COMPLETION_CMDSENT) {
			mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		sdi_cclear |= S3C_SDICSTA_CMDSENT;
	}

	if (sdi_csta & S3C_SDICSTA_CRCFAIL) {
		if( (mrq->cmd->flags & MMC_RSP_CRC) && ( (((mrq->cmd->flags & MMC_RSP_136) == 0) && host->ena_2410_workaround) || !host->ena_2410_workaround) ) {
			DBG("cmd badcrc\n");
			mrq->cmd->error = MMC_ERR_BADCRC;
			goto transfer_closed;
		}

		sdi_cclear |= S3C_SDICSTA_CRCFAIL;
	}

	if (sdi_csta & S3C_SDICSTA_RSPFIN) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			mrq->cmd->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_XFERFINISH;
		}

		sdi_cclear |= S3C_SDICSTA_RSPFIN;
	}

	if( ((sdi_fsta & S3C_SDIFSTA_FIFOFAIL) && !host->ena_2410_workaround) ||
	    ((sdi_dsta & S3C_SDIDSTA_FIFOFAIL) && host->ena_2410_workaround) )  {
			mrq->cmd->error = MMC_ERR_NONE;
		printk(PFX "Unchartererd Waters ....\n");
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_FIFO;
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_RXCRCFAIL) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_BADCRC;
		printk(PFX "crc error - RXCRCFAIL\n");
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_CRCFAIL) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_BADCRC;
		printk(PFX "crc error - TXCRCFAIL\n");
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_DATATIMEOUT) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_TIMEOUT;
#ifdef CONFIG_S3CMCI_DEBUG
		printk(PFX "timeout(%d) - DSTA: %08x\n", mrq->cmd->opcode, readl(host->base + S3C_SDIDSTA));
		{
			int i;
			printk("cmd_idx: %d\n", cmd_idx);
			for (i=0; i<40; i++) {
				printk("%02d: %d\n", i, cmd_rec[i]);
			}
		}
#endif
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_XFERFINISH) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			mrq->cmd->error = MMC_ERR_NONE;
			mrq->data->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			mrq->data->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_RSPFIN;
		}

		sdi_dclear |= S3C_SDIDSTA_XFERFINISH;
	}

	writel(sdi_cclear, host->base + S3C_SDICSTA);
	writel(sdi_dclear, host->base + S3C_SDIDSTA);

	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ still waiting.\n");

	return IRQ_HANDLED;


transfer_closed:
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_request);
	writel(0, host->base + S3C_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ transfer closed.\n");
	return IRQ_HANDLED;

clear_imask:
	writel(0, host->base + S3C_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ clear imask.\n");
	return IRQ_HANDLED;
}

static void s3c_sdi_check_status(unsigned long data)
{
	struct s3c_sdi_host *host = (struct s3c_sdi_host *)data;

	s3c_sdi_irq(0, host, NULL);
}

static void s3c24xx_irq_cd_handler( void *param )
{
	struct mmc_host	*host=(struct mmc_host *) param;

	/* We want the MMC/SD stack to redetect if this IRQ goes off. To do this */
	/* we remove the bus when this IRQ occurs. mmc_detect_change will then */
	/* make sure the bus is reenumerated. */
	mmc_flush_scheduled_work();
	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		if (host->bus_ops->remove)
			host->bus_ops->remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
	mmc_bus_put(host);

	mmc_detect_change( host, HZ/10 );
	return;
}

/*
 * ISR for the CardDetect Pin
 */
static irqreturn_t s3c_sdi_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c_sdi_host *host = (struct s3c_sdi_host *)dev_id;

	schedule_work( &host->irq_cd_wq );

	DBG(PFX "mmc device mode is changed.\n");
	return IRQ_HANDLED;
}

void s3c_sdi_dma_done_callback ( 

#ifdef CONFIG_ARCH_MDIRAC3
s3c_dma_subchan_t 
#else
s3c_dma_chan_t
#endif 
*dma_ch, void *buf_id,
 int size, s3c_dma_buffresult_t result
)
{	unsigned long iflags;
	uint sdi_dcnt;
	struct s3c_sdi_host *host = (struct s3c_sdi_host *)buf_id;

	spin_lock_irqsave( &host->complete_lock, iflags);

	if (!host->mrq)
		goto out;
	if (!host->mrq->data)
		goto out;

	sdi_dcnt = readl(host->base + S3C_SDIDCNT);

	if (result!=S3C_RES_OK)
		goto fail_request;

	if (host->mrq->data->flags & MMC_DATA_READ) {
		if (sdi_dcnt > 0)
			goto fail_request;
	}

out:
	complete(&host->complete_dma);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	return;


fail_request:
	host->mrq->data->error = MMC_ERR_FAILED;
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_dma);
	complete(&host->complete_request);
	writel(0, host->base + S3C_SDIIMSK);
	DBG(PFX "dma fail\n");
	goto out;

}


void s3c_sdi_dma_setup(struct s3c_sdi_host *host, eDMAPurpose_t purpose)
{
#ifdef CONFIG_ARCH_MDIRAC3
	int flowctrl,dest_per,src_per;
#else
	s3c_dmasrc_t source = 0;
#endif
	
	switch(purpose) {
	case DMAP_READ:
#ifdef CONFIG_ARCH_MDIRAC3
		dest_per = 0;
		src_per  = S3C_DMA3_SDMMC;
		//source= S3C_DMASRC_MEM;
		flowctrl=S3C_DMA_PER2MEM;
#else
		source = S3C_DMASRC_HW;
#endif
		break;
	case DMAP_WRITE:
#ifdef CONFIG_ARCH_MDIRAC3
		dest_per= S3C_DMA3_SDMMC;
		src_per = 0;
		flowctrl=S3C_DMA_MEM2PER;
#else
		source=S3C_DMASRC_MEM;
#endif
		break;
	default:
		//add something for other type of transfers
	//	printk("coming to default statement dma transfer\n");

		break;
	}


#ifdef CONFIG_ARCH_MDIRAC3
	//s3c_dma_devconfig(host->dma,host->subchannel,flowctrl,source,dest_per, host->mem->start + S3C_SDIDAT);
	s3c_dma_devconfig(host->dma,host->subchannel,flowctrl,src_per,dest_per, host->mem->start + S3C_SDIDAT);
	s3c_dma_config(host->dma,host->subchannel, 4,4);
	s3c_dma_set_buffdone_fn(host->dma,host->subchannel, s3c_sdi_dma_done_callback);
	s3c_dma_setflags(host->dma,host->subchannel, S3C_DMAF_AUTOSTART);
#else
	s3c_dma_devconfig(host->dma, source, S3C_DISRCC_INC | S3C_DISRCC_APB, host->mem->start + S3C_SDIDAT );
	s3c_dma_config(host->dma, 4, 0, S3C_REQSEL_SDMMC | S3C_REQSEL_HWTRIG );
	s3c_dma_set_buffdone_fn(host->dma, s3c_sdi_dma_done_callback);
	s3c_dma_setflags(host->dma, S3C_DMAF_AUTOSTART);
#endif
}

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))

static void s3c_sdi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
 	struct s3c_sdi_host *host = mmc_priv(mmc);
	u32 sdi_carg, sdi_ccon, sdi_timer, sdi_fsta;
	u32 sdi_bsize, sdi_dcon = 0, sdi_imsk;
	u32 dma_dir = 0;
	unsigned long complete_timeout = msecs_to_jiffies( 1000 ); // 1000 msec timeout on wait for command completion
	WARN_ON(host->mrq != NULL);
	DBG("#############request: [CMD] opcode:0x%02x arg:0x%08x flags:%x retries:%u\n",
		mrq->cmd->opcode, mrq->cmd->arg, mrq->cmd->flags, mrq->cmd->retries);

	host->mrq = mrq;

	sdi_ccon = mrq->cmd->opcode & S3C_SDICCON_INDEX;
	sdi_ccon|= (S3C_SDICCON_SENDERHOST | S3C_SDICCON_CMDSTART);

#ifdef CONFIG_S3CMCI_DEBUG
	{
		cmd_rec[cmd_idx] = mrq->cmd->opcode;
		cmd_idx++;
		cmd_idx %= 40;
	}
#endif
	sdi_carg = mrq->cmd->arg;

	/* XXX: Timer value ?! */
	/* If the workaround for read is enabled, change the timer value. */
	if( host->ena_2410_workaround )
		sdi_timer=0xFFFF;
	else
		sdi_timer= 0x7fffff;

	sdi_bsize= 0;

	/* enable interrupts for transmission errors */
	sdi_imsk = (S3C_SDIIMSK_RESPONSEND | S3C_SDIIMSK_CRCSTATUS);

	host->complete_what = COMPLETION_CMDSENT;

	if (RSP_TYPE(mmc_resp_type(mrq->cmd))) {
		host->complete_what = COMPLETION_RSPFIN;

		sdi_ccon |= S3C_SDICCON_WAITRSP;
		sdi_imsk |= S3C_SDIIMSK_CMDTIMEOUT;
	} else {
		/* We need the CMDSENT-Interrupt only if we want are not waiting
		 * for a response
		 */
		sdi_imsk |= S3C_SDIIMSK_CMDSENT;
	}

	if (mrq->cmd->flags & MMC_RSP_136) {
		sdi_ccon|= S3C_SDICCON_LONGRSP;
	}

	if (mrq->cmd->flags & MMC_RSP_CRC) {
		sdi_imsk |= S3C_SDIIMSK_RESPONSECRC;
	}

	if (mrq->data) {
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
		sdi_ccon|= S3C_SDICCON_WITHDATA;

		sdi_bsize = mrq->data->blksz;

		sdi_dcon  = (mrq->data->blocks & S3C_SDIDCON_BLKNUM_MASK);
		sdi_dcon |= S3C_SDIDCON_DMAEN;

		if( !host->ena_2410_workaround )
			sdi_dcon |= S3C_SDIDCON_WORDTX;

		if (mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
			sdi_dcon |= S3C_SDIDCON_WIDEBUS;
		}

		sdi_imsk |= 0xFFFFFFE0;

		DBG(PFX "request: [DAT] bsize:%u blocks:%u bytes:%u\n",
			sdi_bsize, mrq->data->blocks, mrq->data->blocks * sdi_bsize);

		if (!(mrq->data->flags & MMC_DATA_STREAM)) {
			sdi_dcon |= S3C_SDIDCON_BLOCKMODE;
		}

		if (mrq->data->flags & MMC_DATA_WRITE) {
			sdi_dcon |= S3C_SDIDCON_TXAFTERRESP;
			sdi_dcon |= S3C_SDIDCON_XFER_TX;
			if( !host->ena_2410_workaround )
	 			sdi_dcon |= S3C_SDIDCON_DSTART;
			s3c_sdi_dma_setup(host, DMAP_WRITE);
			dma_dir = DMA_TO_DEVICE;
		} else { 

			sdi_dcon |= S3C_SDIDCON_RXAFTERCMD;
			sdi_dcon |= S3C_SDIDCON_XFER_RX;
			if( !host->ena_2410_workaround )
				sdi_dcon |= S3C_SDIDCON_DSTART;
			s3c_sdi_dma_setup(host, DMAP_READ);
			dma_dir = DMA_FROM_DEVICE;
		}

		/* Clear FAIL bits also for the fifo. */
		sdi_fsta = S3C_SDIFSTA_FRST | S3C_SDIFSTA_FIFOFAIL;
		__raw_writel(sdi_fsta,host->base + S3C_SDIFSTA);

		/* start DMA */
		dma_map_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);
#ifdef CONFIG_ARCH_MDIRAC3
		s3c_dma_enqueue(host->dma,host->subchannel,(void *) host,
			sg_dma_address(mrq->data->sg),
			(mrq->data->blocks * mrq->data->blksz) );
#else 
		s3c_dma_enqueue(host->dma, (void *) host,
			sg_dma_address(mrq->data->sg),
			(mrq->data->blocks * mrq->data->blksz) );
#endif
		/* Check if we should enable the workaround for timeouts in the 2410 soc. */
		/* Since we want to go as fast as possible, don't use the maximum divider.*/
		/* dividing by 90 will give a clock of roughly 553MHz. This should be safe*/
		/* enough. */
		host->prescaler=readl( host->base + S3C_SDIPRE );
		if( host->ena_2410_workaround && (mrq->data->flags & MMC_DATA_READ) )
			writel( (clk_get_rate( host->clk )/533000), host->base + S3C_SDIPRE );
	}

	host->mrq = mrq;
	init_completion(&host->complete_request);
	init_completion(&host->complete_dma);
	
	/* Clear command and data status registers */
	writel(0xFFFFFFFF, host->base + S3C_SDICSTA);
	writel(0xFFFFFFFF, host->base + S3C_SDIDSTA);

	/* Setup SDI controller */
	writel(sdi_bsize,host->base + S3C_SDIBSIZE);
	writel(sdi_timer,host->base + S3C_SDITIMER);

	writel(sdi_imsk,host->base + S3C_SDIIMSK);

	/* Setup SDI command argument and data control */
	writel(sdi_carg, host->base + S3C_SDICARG);
	writel(sdi_dcon, host->base + S3C_SDIDCON);

	/* This initiates transfer */
	writel(sdi_ccon, host->base + S3C_SDICCON);

	/* Workaround for S3C2410. When the receive transfer has started we can write the */
	/* original prescaler back to transfer at maximum speed. Talk about a dirty hack...*/
	if( host->ena_2410_workaround && (mrq->data != NULL) && (mrq->data->flags & MMC_DATA_READ) )
	{
		/* Start polling if the receive transfer has started.... */
		while( ((readl( host->base + S3C_SDIFSTA ) & 0x7F) == 0) && ((readl( host->base + S3C_SDIDSTA ) & 0x30) == 0) )
		{
			/* Ensure that if an error occurs, we can still exit. */
			if( readl( host->base + S3C_SDIDSTA ) & (S3C_SDIDSTA_FIFOFAIL | S3C_SDIDSTA_CRCFAIL |
								 S3C_SDIDSTA_RXCRCFAIL | S3C_SDIDSTA_DATATIMEOUT |
								 S3C_SDIDSTA_SBITERR) )
				break;
		}
		writel( host->prescaler, host->base + S3C_SDIPRE );
	}

	/* this wait is very important to sd/mmc run correctly.
	 * Without this blocking code, operation sequence may be crashed.
	 * by scsuh.
	 */
	/* Wait for transfer to complete */
	wait_for_completion_timeout(&host->complete_request, complete_timeout);
	if (mrq->data && (host->mrq->data->error == MMC_ERR_NONE)) {
		if (wait_for_completion_timeout(&host->complete_dma, complete_timeout) == 0)
		{
#ifdef CONFIG_ARCH_MDIRAC3
			s3c_dma_ctrl(host->dma,host->subchannel, S3C_DMAOP_FLUSH);
#else 
			s3c_dma_ctrl(host->dma, S3C_DMAOP_FLUSH);
#endif
		}
		DBG("[DAT] DMA complete.\n");
		sdi_fsta = readl(host->base + S3C_SDIFSTA);
		writel(sdi_fsta,host->base + S3C_SDIFSTA);
	}

	/* Cleanup controller */
	writel(0, host->base + S3C_SDICARG);
	writel(0, host->base + S3C_SDIDCON);
	writel(0, host->base + S3C_SDICCON);
	writel(0, host->base + S3C_SDIIMSK);

	/* Read response */
	mrq->cmd->resp[0] = readl(host->base + S3C_SDIRSP0);
	mrq->cmd->resp[1] = readl(host->base + S3C_SDIRSP1);
	mrq->cmd->resp[2] = readl(host->base + S3C_SDIRSP2);
	mrq->cmd->resp[3] = readl(host->base + S3C_SDIRSP3);

	host->mrq = NULL;

	DBG(PFX "request done.\n");

	if (mrq->data) {
		dma_unmap_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);

		/* Calculate the about of bytes transfer, but only if there was
		 * no error
		 */
		if (mrq->data->error == MMC_ERR_NONE)
			mrq->data->bytes_xfered = (mrq->data->blocks * mrq->data->blksz);
		else
			mrq->data->bytes_xfered = 0;

		/* If we had an error while transferring data we flush the
		 * DMA channel to clear out any garbage
		 */
		if (mrq->data->error != MMC_ERR_NONE) {
#ifdef CONFIG_ARCH_MDIRAC3
			s3c_dma_ctrl(host->dma,host->subchannel, S3C_DMAOP_FLUSH);
#else 
			s3c_dma_ctrl(host->dma, S3C_DMAOP_FLUSH);
#endif
			DBG(PFX "flushing DMA.\n");
		}
		/* Issue stop command */
		if (mrq->data->stop)
			mmc_wait_for_cmd(mmc, mrq->data->stop, 3);
	}

	mrq->done(mrq);
}

static void s3c_sdi_poweroff(struct s3c_sdi_host *host)
{
	u32 sdi_con = S3C_SDICON_RESET;
	writel(sdi_con, host->base + S3C_SDICON);
	while(readl(host->base + S3C_SDICON) & S3C_SDICON_RESET);
#if CONFIG_MACH_TOMTOMGO
	IO_Deactivate(SDCLK);
	IO_Deactivate(SDCMD);
	IO_Deactivate(SDDATA0);
	IO_Deactivate(SDDATA1);
	IO_Deactivate(SDDATA2);
	IO_Deactivate(SDDATA3);
	IO_Deactivate(PULLUP_SD);
	IO_Deactivate(SD_PWR_ON);
	IO_Deactivate(MOVI_PWR_ON);
	IO_Deactivate(EN_SD);
#elif CONFIG_ARCH_S3C2460
	s3c_gpio_cfgpin(S3C_GPA13, S3C_GPA13_OUTP);
#elif defined CONFIG_ARCH_S3C2412
	s3c_gpio_cfgpin(S3C_GPE5, S3C_GPE5_OUTP);
#elif defined CONFIG_ARCH_MDIRAC3
	//`s3c_gpio_cfgpin(S3C_GPA13,S3C_GPA13_OUTP);
#endif
}

static void s3c_sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s3c_sdi_host *host = mmc_priv(mmc);
	u32 sdi_psc, sdi_con = 0, sdi_fsta;
	u32 original_sdi_con;

	original_sdi_con=readl(host->base + S3C_SDICON);
	
	/* Set power */
	switch(ios->power_mode) {
	case MMC_POWER_UP:
#if CONFIG_MACH_TOMTOMGO
		/* rememeber mmchost->base for suicide circuit */
		if (s3c_sdi_host_poweroff == 0) 
			s3c_sdi_host_poweroff = host;


		IO_SetInput(CD_SD);
		if (IO_GetInput(CD_SD)) {
			IO_Deactivate(SW_SD);
			mmc->removable = 1;
		} else {
			IO_Activate(SW_SD);
			mmc->removable = 0;
		}

		IO_SetInterruptOnToggle(CD_SD);

		IO_Activate(EN_SD);
		IO_Deactivate(SDCLK);
		IO_Activate(SD_PWR_ON);

		IO_SetFunction(SDCMD);
		IO_SetFunction(SDDATA0);
		IO_SetFunction(SDDATA1);
		IO_SetFunction(SDDATA2);
		IO_SetFunction(SDDATA3);

		if (IO_HaveMovinandSoftPoweron()) {
			/* slowly charge 2.2uF cap to prevent 1.8V supply dip */
			IO_SetInput(MOVI_PWR_ON);
			msleep(3);
		}
		IO_Activate(MOVI_PWR_ON);
		msleep(1); // wait for power ramp-up
		IO_Activate(PULLUP_SD);
		msleep(1);
		
		/* Disable clock oscillation before "SetFunction(SDCLK)" to avoid spikes.  */
		writel(original_sdi_con & ~S3C_SDICON_CLKENABLE ,host->base + S3C_SDICON);
		IO_SetFunction(SDCLK); /* Clock oscillation will be started if needed later in the function */

		msleep ( 10 ) ; /* Give the card some time to init. Acc. spec 1 ms is minimum */

#elif CONFIG_ARCH_S3C2460
		s3c_gpio_cfgpin(S3C_GPA8, S3C_GPA8_SDIO_CMD);
		s3c_gpio_cfgpin(S3C_GPA9, S3C_GPA9_SDIO_DAT0);
		s3c_gpio_cfgpin(S3C_GPA10, S3C_GPA10_SDIO_DAT1);
		s3c_gpio_cfgpin(S3C_GPA11, S3C_GPA11_SDIO_DAT2);
		s3c_gpio_cfgpin(S3C_GPA12, S3C_GPA12_SDIO_DAT3);
		s3c_gpio_cfgpin(S3C_GPA13, S3C_GPA13_SDIO_CLK);
		sdi_con = S3C_SDICON_CLKTYPE_MMC | (1 << 6);

#elif defined(CONFIG_ARCH_S3C2412)
		s3c_gpio_cfgpin(S3C_GPE5, S3C_GPE5_SDCLK);
		s3c_gpio_cfgpin(S3C_GPE6, S3C_GPE6_SDCMD);
		s3c_gpio_cfgpin(S3C_GPE7, S3C_GPE7_SDDAT0);
		s3c_gpio_cfgpin(S3C_GPE8, S3C_GPE8_SDDAT1);
		s3c_gpio_cfgpin(S3C_GPE9, S3C_GPE9_SDDAT2);
		s3c_gpio_cfgpin(S3C_GPE10, S3C_GPE10_SDDAT3);


#elif defined CONFIG_ARCH_MDIRAC3
		s3c_gpio_cfgpin(S3C_GPA13,S3C_GPA13_SDIO_CLK);
		s3c_gpio_cfgpin(S3C_GPA8, S3C_GPA8_SDIO_CMD);
		s3c_gpio_cfgpin(S3C_GPA9,S3C_GPA9_SDIO_DAT0);
		s3c_gpio_cfgpin(S3C_GPA10, S3C_GPA10_SDIO_DAT1);
		s3c_gpio_cfgpin(S3C_GPA11,S3C_GPA11_SDIO_DAT2);
		s3c_gpio_cfgpin(S3C_GPE12,S3C_GPA12_SDIO_DAT3);
#endif
		

#if defined(__BIG_ENDIAN)
		sdi_con |= S3C_SDICON_BYTEORDER;
#endif
		break;

		
	case MMC_POWER_ON:
		if (mmc->mode == MMC_MODE_MMC) {
			sdi_con = S3C_SDICON_CLKTYPE_MMC;
		} 
		break;

	case MMC_POWER_OFF:
		s3c_sdi_poweroff(host);
		break;
		
	default:
		printk( KERN_ERR "s3c_sdi_set_ios: !!!!!! powermode %x is unknown. Defaulting to OFF.\n",ios->power_mode );
		/* fall through */
		break;
	}

	sdi_fsta = S3C_SDIFSTA_FRST;
	writel(sdi_fsta,host->base + S3C_SDIFSTA);

	/* Set clock */
	for(sdi_psc=0;sdi_psc<254;sdi_psc++) {
		if ( (clk_get_rate(host->clk) / (sdi_psc+1)) <= ios->clock)
			break;
	}

	if (sdi_psc > 255)
		sdi_psc = 255;
	writel(sdi_psc, host->base + S3C_SDIPRE);

#ifdef CONFIG_S3CMCI_DEBUG
	if (sdi_psc < 10)
		printk("  clock is: %lu.%luMHz of %uMHz\n",
			clk_get_rate(host->clk)/(sdi_psc+1)/1000000,
			(clk_get_rate(host->clk)/(sdi_psc+1)/1000)%1000,
			(ios->clock)/1000000);
#endif

	/* Only SD clock can support more than 20MHz clock.
	 * by scsuh
	 */
	if (clk_get_rate(host->clk) / (sdi_psc+1) > 20 * MHZ)
		sdi_con &= ~S3C_SDICON_CLKTYPE_MMC;

	/* Set CLOCK_ENABLE */
	if (ios->clock)
		sdi_con |= S3C_SDICON_CLKENABLE;
	else
		sdi_con &=~S3C_SDICON_CLKENABLE;

	writel(sdi_con, host->base + S3C_SDICON);
	
	if ( ((original_sdi_con & S3C_SDICON_CLKENABLE)==0) && (sdi_con & S3C_SDICON_CLKENABLE))
	{  	/* Make sure that 74 clock cycles have past before sending first command after clock-enable*/
		msleep(5);
	}
	else
	{
		udelay(1000);
	}	
}

static struct mmc_host_ops s3c_sdi_ops = {
	.request	= s3c_sdi_request,
	.set_ios	= s3c_sdi_set_ios,
};

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. Change sd clock, or refuse policy.
 */
static int
s3c24xxsdi_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs		*f = data;
 	struct s3c_sdi_host		*host=container_of( nb, struct s3c_sdi_host, freq_transition );
	unsigned long int		new_pclk;
	unsigned long int		old_pclk;
	static unsigned long int	old_prescaler=0;
	unsigned long int		prescaler;
	unsigned long int		new_clock;
	unsigned long int		old_clock;

	/* First thing to do. Get the old prescaler if we haven't yet. */
	if( old_prescaler == 0 )
		old_prescaler=__raw_readl( host->base + S3C_SDIPRE ) + 1; 

	/* We get CPUFREQ. Work back from there to PCLK. */
	f->trans2pclk( f, &old_pclk, &new_pclk );

	/* Calculate the new rate, and determine the prescaler. */
	for( prescaler=1; (prescaler <= 256) && (host->mmc->f_max <= ((new_pclk*1000)/prescaler)); prescaler++ );

	/* Calculate the values of the clock. */
	old_clock=1000*old_pclk/old_prescaler;
	new_clock=1000*new_pclk/prescaler;

	switch (val) {
		case CPUFREQ_PRECHANGE:
			/* Disable the clock. We don't want it running while we change things. */
			clk_disable( host->clk );

			/* Check if the current selected clock would give a legal value. */
			if( (new_clock < host->mmc->f_min) || (new_clock > host->mmc->f_max) )
			{
				/* Check if we go from a legal to an illegal clock. */
				if( (old_clock >= host->mmc->f_min) && (old_clock <= host->mmc->f_max) )
				{
					/* From a legal to an illegal clock. Free the IRQs and stop the clock. */
					/* Also disable the SDcard insert detection. */
					IO_Deactivate( CD_SD );
					free_irq(host->irq_cd, host);
					free_irq(host->irq, host);
					disable_irq(host->irq);
				}
				else
				{
					/* From an illegal to an illegal clock. No need to do anything. */
				}

			}
			else
			{
				/* From illegal to legal clock ? Note that if the card is not inserted, the clock */
				/* we go from might be illegal. This should not be a problem as it is not in use. */
				/* Pretend like the clock is legal then. */
				if( ((old_clock < host->mmc->f_min) || (old_clock > host->mmc->f_max)) &&
				    (host->mmc->bus_ops && !host->mmc->bus_dead) )
				{
					/* New clock is legal. Check if the old clock was not. We need to enable the */
					/* IRQs again. */
					/* Request the IRQs again. */
					if(request_irq(host->irq, s3c_sdi_irq, 0, DRIVER_NAME, host))
					{
						/* Can't get IRQ! */
						printk( "s3c2410sdi: Can't request SDI interface IRQ! Expect problems!\n" );
						break;
					}

					if (request_irq(host->irq_cd, s3c_sdi_irq_cd, SA_INTERRUPT, DRIVER_NAME, host))
					{
						printk( "s3c2410sdi: Can't request Card Detect SDI IRQ! Expect problems!\n" );
						free_irq( host->irq, host );
						break;
					}

					/* Reenable the CD_SD line. */
					IO_SetInterruptOnToggle(CD_SD);

					/* Reenable the IRQ. */
					enable_irq(host->irq);
				}
				else
				{
					/* From a legal to a legal clock. No need to do anything. */
				}
			}
			break;

		case CPUFREQ_POSTCHANGE:
			/* Save the newly discovered prescaler, regardless of what it is. */
			old_prescaler=prescaler;

			/* Check if this is a legal clock. */
			if( (new_clock >= host->mmc->f_min) && (new_clock <= host->mmc->f_max) )
			{
				/* Legal clock. */
				/* Verify the prescaler found is correct. */
				clk_set_rate( host->clk, new_pclk );

				/* Fill in the remaining fields. */
				clk_enable( host->clk );

				/* Program prescaler. */
				writel((prescaler - 1), host->base + S3C_SDIPRE);
			}
			else
			{
				/* Illegal clock. Leave it disabled. */
			}
			break;
	}
	return 0;
}

static int
s3c24xxsdi_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy	*policy = data;
	struct s3c_sdi_host	*host=container_of( nb, struct s3c_sdi_host, freq_policy );
	unsigned long int	min_rec_freq=(1 * host->mmc->f_min) / 1000;
	unsigned long int	max_freq=(255 * host->mmc->f_max) / 1000;
	unsigned long int	max_pclk;
	unsigned long int	min_pclk;

	policy->policy2pclk( policy, &min_pclk, &max_pclk );

	switch (val) {
		case CPUFREQ_ADJUST:
			/* Maximum values depend on the card inserted. The speed of the card is in host->mmc->f_max/f_min. */
			/* In principle, no minimums exists, although we'd like to keep the card on the speedit recommends. */
			policy->pclk2policy( policy, min_rec_freq, max_freq );
			break;

		case CPUFREQ_INCOMPATIBLE:
			/* Check if the maximum frequency is exceeded. */
			if( (min_pclk >= min_rec_freq) && (min_pclk <= max_freq) )
				min_rec_freq=min_pclk;

			if( (max_pclk >= min_rec_freq) && (max_pclk <= max_freq) )
				max_freq=max_pclk;

			if( (max_pclk != max_freq) || (min_pclk != min_rec_freq) )
				policy->pclk2policy( policy, min_rec_freq, max_freq );
			break;

		case CPUFREQ_NOTIFY:
			/* Even if the maximum divider is used, at the maximum frequency possible for PCLK, the clock would */
			/* not be much more than 1MHz. It is very unlikely this will ever occur. The code that does this is */
			/* therefore more for the form. It can be found in the transition routine. */
			break;
	}
	return 0;
}
#endif

static int s3c_sdi_probe(struct device *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct mmc_host *mmc;
	struct s3c_sdi_host *host;

	int ret;
#ifdef CONFIG_S3C2443_EVT1
	/* EXTINT0 S3C2443 EVT1 workaround */
	u32 tmp;
#endif

	mmc = mmc_alloc_host(sizeof(struct s3c_sdi_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);
	spin_lock_init(&host->complete_lock);
	host->complete_what 	= COMPLETION_NONE;
	host->mmc 		= mmc;
#if CONFIG_MACH_TOMTOMGO
	host->irq_cd		= IO_GetInterruptNumber(CD_SD);
	mmc->removable		= 1;
#elif CONFIG_ARCH_S3C2460
	host->irq_cd		= IRQ_EINT3;
#elif defined(CONFIG_MACH_SMDK2443)
	host->irq_cd		= IRQ_EINT1;
#elif defined(CONFIG_ARCH_MDIRAC3)
	host->subchannel	=S3C_DMA3_SDMMC;
//	host->irq_cd		= IRQ_EINT7;
#elif defined CONFIG_ARCH_S3C2412
	host->irq_cd		= IRQ_EINT18;
#endif
	host->dma		= S3C_SDI_DMA;

	host->mem = platform_get_resource(pdev, IORESOURCE_MEM ,0);
	if (!host->mem) {
		printk("failed to get io memory region resource.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
		RESSIZE(host->mem), pdev->name);
	
	if (!host->mem) {
		printk("failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	/* if there is an error here, check your SoC dependent code.
	 * You must have iotable that contains SDI in it.
	 * by scsuh.
	 */
	host->base = S3C24XX_VA_SDI;
	host->irq = platform_get_irq(pdev, 0);
	
	if (host->irq == 0) {
		printk("failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto release_memory;
	}

	if (request_irq(host->irq, s3c_sdi_irq, 0, DRIVER_NAME, host)) {
		printk("failed to request sdi interrupt.\n");
		ret = -ENOENT;
		goto release_memory;
	}

#if defined(CONFIG_MACH_SMDK2443)
#ifdef CONFIG_S3C2443_EVT1
	/* EXTINT0 S3C2443 EVT1 workaround */
	tmp = __raw_readl(S3C_EXTINT0);
	s3c_swap_4bit(tmp);
	__raw_writel(tmp | (1<<7), S3C_EXTINT0);
#endif
	s3c_gpio_cfgpin(S3C_GPF1, S3C_GPF1_EINT1);
#elif defined(CONFIG_ARCH_S3C2460)
	s3c_gpio_cfgpin(S3C_GPJ3, S3C_GPJ3_EXT_INT3);
#elif defined CONFIG_ARCH_S3C2412
	s3c_gpio_cfgpin(S3C_GPG10, S3C_GPG10_EINT18);
#elif defined CONFIG_ARCH_MDIRAC3
	;
#endif

#ifdef CONFIG_ARCH_MDIRAC3	
	if (s3c_dma_request(host->dma,host->subchannel, &s3c_sdi_dma_client,NULL)) {
		printk("unable to get DMA channel.\n" );
		ret = -EBUSY;
		goto probe_free_irq_cd;
	}
#else 
	INIT_WORK( &host->irq_cd_wq, s3c24xx_irq_cd_handler, mmc );
	set_irq_type(host->irq_cd, IRQT_BOTHEDGE);
	
	if (host->irq_cd > 0) {
		if (request_irq(host->irq_cd, s3c_sdi_irq_cd, SA_INTERRUPT, DRIVER_NAME, host)) {
			printk("failed to request card detect interrupt.\n" );
			ret = -ENOENT;
			goto probe_free_irq;
		}
	}
	if (s3c_dma_request(S3C_SDI_DMA, &s3c_sdi_dma_client, NULL)) {
		printk("unable to get DMA channel.\n" );
		ret = -EBUSY;
		goto probe_free_irq_cd;
	}
	
#endif
	host->clk = clk_get(&pdev->dev, "sdi");
	if (IS_ERR(host->clk)) {
		printk("failed to find clock source.\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_host;
	}

	if ((ret = clk_enable(host->clk))) {
		printk("failed to enable clock source.\n");
		goto clk_free;
	}

	mmc->ops = &s3c_sdi_ops;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	mmc->f_min = clk_get_rate(host->clk) / 512;
	/* you must make sure that our sdmmc block can support
	 * up to 25MHz. by scsuh
	 */
	mmc->f_max = 25 * MHZ;
	mmc->caps = MMC_CAP_4_BIT_DATA;

	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 */
	mmc->max_req_size = 65535;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */

	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Both block size and block count use 12 bit registers.
	 */
	mmc->max_blk_size = 4095;
	mmc->max_blk_count = 4095;

	printk(KERN_INFO PFX "probe: mapped sdi_base=%p irq=%u irq_cd=%u dma=%u.\n",
		host->base, host->irq, host->irq_cd, host->dma);
	platform_set_drvdata(pdev, mmc);

	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = s3c_sdi_check_status;
	host->timer.expires = jiffies + HZ;
	host->ena_2410_workaround=(IO_GetCpuType( ) == GOCPU_S3C2410);

	if ((ret = mmc_add_host(mmc))) {
		printk(KERN_INFO PFX "failed to add mmc host.\n");
		goto free_dmabuf;
	}

	/* Do CPUFREQ registration. */
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	host->freq_transition.notifier_call = s3c24xxsdi_freq_transition;
	host->freq_transition.priority = CPUFREQ_ORDER_S3C24XX_SDCARD_PRIO;
	host->freq_policy.notifier_call = s3c24xxsdi_freq_policy;
	cpufreq_register_notifier(&host->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&host->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	printk(KERN_INFO PFX "initialization done.\n");
	return 0;

free_dmabuf:
	clk_disable(host->clk);

clk_free:
	clk_put(host->clk);

probe_free_irq_cd:
#ifndef CONFIG_ARCH_MDIRAC3
 	free_irq(host->irq_cd, host);
#endif
probe_free_irq:
 	free_irq(host->irq, host);

release_memory:
	release_mem_region(host->mem->start, RESSIZE(host->mem));

probe_free_host:
	mmc_free_host(mmc);

probe_out:
	return ret;
}

static int s3c_sdi_remove(struct device *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct mmc_host *mmc  = platform_get_drvdata(pdev);
	struct s3c_sdi_host *host = mmc_priv(mmc);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&host->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&host->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	del_timer_sync(&host->timer);
	mmc_remove_host(mmc);

	clk_disable(host->clk);
	clk_put(host->clk);
 	free_irq(host->irq_cd, host);
 	free_irq(host->irq, host);
	mmc_free_host(mmc);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_sdi_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct s3c_sdi_host *host = mmc_priv(mmc);
	int ret = 0;

	if (host && (level == SUSPEND_POWER_DOWN)) {
		if (mmc)
			ret = mmc_suspend_host(mmc, state);

		s3c_dma_free(host->dma, &s3c_sdi_dma_client);
	}

	return ret;
}

static int s3c_sdi_resume(struct device *dev, u32 level)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct s3c_sdi_host *host = mmc_priv(mmc);
	int ret = 0;

	if (host && (level == RESUME_POWER_ON)) {
		s3c_dma_request(host->dma,&s3c_sdi_dma_client,NULL);
		if (mmc)
			ret = mmc_resume_host(mmc);
	}

	return ret;
}
#else
#define s3c_sdi_suspend	NULL
#define s3c_sdi_resume	NULL
#endif

static struct device_driver s3c_sdi_driver =
{
	.name		= "s3c-sdi",
	.bus            = &platform_bus_type,
	.probe          = s3c_sdi_probe,
	.remove         = s3c_sdi_remove,
	.suspend	= s3c_sdi_suspend,
	.resume		= s3c_sdi_resume,
};

static int __init s3c_sdi_init(void)
{
	return driver_register(&s3c_sdi_driver);
}

static void __exit s3c_sdi_exit(void)
{
	driver_unregister(&s3c_sdi_driver);
}
module_init(s3c_sdi_init);
module_exit(s3c_sdi_exit);

MODULE_DESCRIPTION("S3C Multimedia Card I/F Driver");
MODULE_LICENSE("GPL");

