/*
 *  linux/drivers/mmc/s3c-hsmmc.c - Samsung S3C24XX HS-MMC driver
 *
 *  Copyright (C) 2006 Samsung Electronics, All Rights Reserved.
 *  by Seung-Chull, Suh <sc.suh@samsung.com>
 *
 *  This driver is made for High Speed MMC interface. This interface
 *  is adopted and implemented since s3c2443 was made.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Modified by Ryu,Euiyoul <steven.ryu@samsung.com>
 *
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

#if defined(CONFIG_MACH_TOMTOMGO)
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <../arch/arm/mach-s3c2410/tomtomgo-iopins.h>
#endif

#define PK(l,f,x...)	do { printk(KERN_##l "%s: " f, __func__ ,##x); } while (0)

#ifdef CONFIG_S3CMCI_DEBUG
#define DBG(x...)	PK(DEBUG, x)
#else
#define DBG(x...)	do { } while (0)
#endif

#define INFO(x...)	PK(INFO, x)
#define WARN(x...)	PK(WARNING, x)
#define ERR(x...)	PK(ERR, x)


#include "s3c-hsmmc.h"

#ifdef CONFIG_MMC_TRANSFER_LOGGING
#include "mmc-transfer-logger.h"
#endif

#define DRIVER_NAME "s3c-hsmmc"
#define PFX DRIVER_NAME ": "

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

#define SetEPLL( Mdiv, Pdiv, Sdiv)      writel(( (Mdiv<<16) | (Pdiv<<8) | Sdiv) , S3C_EPLLCON )

#define CONTROL3_TXDELAY1	((0 << 31) | (1 << 23))
#define CONTROL3_TXDELAY2	((1 << 31) | (1 << 23))
#define CONTROL3_TXDELAY3	((0 << 31) | (0 << 23))
#define CONTROL3_TXDELAY4	((1 << 31) | (0 << 23))

#define CONTROL3_RXDELAY1	((0 << 15) | (1 << 7))
#define CONTROL3_RXDELAY2	((1 << 15) | (1 << 7))
#define CONTROL3_RXDELAY3	((0 << 15) | (0 << 7))
#define CONTROL3_RXDELAY4	((1 << 15) | (0 << 7))

#if defined(CONFIG_MACH_TOMTOMGO)
static int hsmmc_hw_init( void )
{
        if (IO_HaveHsMmcInterface()) {
	        IO_SetFunction(HS_SDCMD);
        	IO_SetFunction(HS_SDDATA0);
	        IO_SetFunction(HS_SDDATA1);
        	IO_SetFunction(HS_SDDATA2);
	        IO_SetFunction(HS_SDDATA3);
        	IO_SetFunction(HS_SDDATA4);
	        IO_SetFunction(HS_SDDATA5);
        	IO_SetFunction(HS_SDDATA6);
	        IO_SetFunction(HS_SDDATA7);
#if 0
        	IO_SetInput(HS_MOVI_PWR_ON);
	        mdelay(3);
#endif
        	IO_Activate(HS_MOVI_PWR_ON);
	        mdelay(1);
        	IO_SetFunction(HS_SDCLK);
		msleep(200);
	}
	return 0;
}

void hsmmc_hw_exit( void )
{
        if (IO_HaveHsMmcInterface()) {
	        IO_SetInput( HS_SDCMD );
        	IO_SetInput( HS_SDDATA0 );
	        IO_SetInput( HS_SDDATA1 );
	        IO_SetInput( HS_SDDATA2 );
	        IO_SetInput( HS_SDDATA3 );
	        IO_SetInput( HS_SDDATA4 );
	        IO_SetInput( HS_SDDATA5 );
	        IO_SetInput( HS_SDDATA6 );
	        IO_SetInput( HS_SDDATA7 );
        	mdelay(500);
	        IO_Deactivate(HS_MOVI_PWR_ON);
        	IO_SetInput( HS_SDCLK );
	}
}

static int hsmmc0_hw_init(void)
{
	IO_SetFunction(HS0_SDCMD);
	IO_SetFunction(HS0_SDDATA0);
	IO_SetFunction(HS0_SDDATA1);
	IO_SetFunction(HS0_SDDATA2);
	IO_SetFunction(HS0_SDDATA3);

	//IO_Activate(HS0_MOVI_PWR_ON);
	mdelay(1);
	IO_SetFunction(HS0_SDCLK);

	switch(IO_GetModelId()) {
		case GOTYPE_TREVISO:
			/* On Treviso movinand is attached to hsmmc0 */
			msleep( 200 );
			break;
		default:
			msleep( 200 );
			break;
	}
	return 0;
}

static int hsmmc1_hw_init(void)
{
	IO_SetFunction(HS1_SDCMD);
	IO_SetFunction(HS1_SDDATA0);
	IO_SetFunction(HS1_SDDATA1);
	IO_SetFunction(HS1_SDDATA2);
	IO_SetFunction(HS1_SDDATA3);

	IO_Activate(SD_PWR_ON);
	mdelay(1);
	IO_SetFunction(HS1_SDCLK);
	
	switch(IO_GetModelId()) {
		case GOTYPE_TREVISO:
			/* On Treviso mmc is attached to hsmmc1 */
			msleep( 10 );
			break;
		default:
			msleep( 200 );
			break;
	}

	return 0;
}

void hsmmc0_hw_exit( void )
{
	IO_SetInput( HS0_SDCMD );
	IO_SetInput( HS0_SDDATA0 );
	IO_SetInput( HS0_SDDATA1 );
	IO_SetInput( HS0_SDDATA2 );
	IO_SetInput( HS0_SDDATA3 );
	mdelay(500);
	//IO_Deactivate(HS0_MOVI_PWR_ON);
	IO_SetInput( HS0_SDCLK );
}

void hsmmc1_hw_exit( void )
{
	IO_SetInput( HS1_SDCMD );
	IO_SetInput( HS1_SDDATA0 );
	IO_SetInput( HS1_SDDATA1 );
	IO_SetInput( HS1_SDDATA2 );
	IO_SetInput( HS1_SDDATA3 );
	mdelay(500);
	IO_Deactivate(SD_PWR_ON);
	IO_SetInput( HS1_SDCLK );
}

static void hsmmc_set_usb48clk(struct s3c_hsmmc_host *host, unsigned int clock);
static void hsmmc_set_epllclk(struct s3c_hsmmc_host *host, unsigned int clock);
static void hsmmc_set_hclk(struct s3c_hsmmc_host *host, unsigned int clock);

static void hsmmc_set_gpio(struct s3c_hsmmc_host *host, struct platform_device* pdev)
{
	switch (pdev->id)
	{
		case 0:
			host->hsmmc_hw_init = hsmmc0_hw_init;
			host->hsmmc_hw_exit = hsmmc0_hw_exit;
			break;
		case 1:
			host->hsmmc_hw_init = hsmmc1_hw_init;
			host->hsmmc_hw_exit = hsmmc1_hw_exit;
			break;
		default:
			host->hsmmc_hw_init = hsmmc_hw_init;
			host->hsmmc_hw_exit = hsmmc_hw_exit;
			break;
	}

	switch (IO_GetHsMmcClockType())
	{
		case GOHSMMC_USB48CLK:
			host->hsmmc_set_clock = hsmmc_set_usb48clk;
			break;
		case GOHSMMC_EPLL:
			host->hsmmc_set_clock = hsmmc_set_epllclk;
			break;
		case GOHSMMC_HCLK:
		default:
			/* By default we use HCLK */
			host->hsmmc_set_clock = hsmmc_set_hclk;
			break;
	}
}

/*
* ISR for the CardDetect Pin
*/
static irqreturn_t hsmmc_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c_hsmmc_host *host = (struct s3c_hsmmc_host *)dev_id;

	DBG("hsmmc device mode is changed.\n");
	
	tasklet_schedule(&host->card_tasklet);
	return IRQ_HANDLED;
}
#endif /* CONFIG_MACH_TOMTOMGO  */


/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void s3c_hsmmc_reset(struct s3c_hsmmc_host *host, u8 mask)
{
	unsigned long timeout;

	s3c_hsmmc_writeb(mask, HM_SWRST);

	if (mask & S3C_HSMMC_RESET_ALL)
		host->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (s3c_hsmmc_readb(HM_SWRST) & mask) {
		if (timeout == 0) {
			ERR("%s: Reset 0x%x never completed. \n",
				mmc_hostname(host->mmc), (int)mask);
			return;
		}
		timeout--;
		mdelay(1);
	}

}

static void s3c_hsmmc_ios_init(struct s3c_hsmmc_host *host)
{
	u32 intmask;

	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

	intmask = S3C_HSMMC_INT_BUS_POWER | S3C_HSMMC_INT_DATA_END_BIT |
		S3C_HSMMC_INT_DATA_CRC | S3C_HSMMC_INT_DATA_TIMEOUT | S3C_HSMMC_INT_INDEX |
		S3C_HSMMC_INT_END_BIT | S3C_HSMMC_INT_CRC | S3C_HSMMC_INT_TIMEOUT |
		S3C_HSMMC_INT_CARD_REMOVE | S3C_HSMMC_INT_CARD_INSERT |
		S3C_HSMMC_INT_DATA_AVAIL | S3C_HSMMC_INT_SPACE_AVAIL |
		S3C_HSMMC_INT_DMA_END | S3C_HSMMC_INT_DATA_END | S3C_HSMMC_INT_RESPONSE;

	s3c_hsmmc_writel(intmask, HM_NORINTSTSEN);
	s3c_hsmmc_writel(intmask, HM_NORINTSIGEN);
}
#ifndef CONFIG_MACH_TOMTOMGO
static void s3c_hsmmc_set_gpio(void)
{
	s3c_gpio_cfgpin(S3C_GPL0, S3C_GPL0_SD0DAT0);
	s3c_gpio_cfgpin(S3C_GPL1, S3C_GPL1_SD0DAT1);

	s3c_gpio_cfgpin(S3C_GPL2, S3C_GPL2_SD0DAT2);
	s3c_gpio_cfgpin(S3C_GPL3, S3C_GPL3_SD0DAT3);

	s3c_gpio_cfgpin(S3C_GPL4, S3C_GPL4_SD0DAT4);
	s3c_gpio_cfgpin(S3C_GPL5, S3C_GPL5_SD0DAT5);

	s3c_gpio_cfgpin(S3C_GPL6, S3C_GPL6_SD0DAT6);
	s3c_gpio_cfgpin(S3C_GPL7, S3C_GPL7_SD0DAT7);

	s3c_gpio_cfgpin(S3C_GPL8, S3C_GPL8_SD0CMD);
	s3c_gpio_cfgpin(S3C_GPL9, S3C_GPL9_SD0CLK);

	s3c_gpio_cfgpin(S3C_GPJ14, S3C_GPJ14_nSD0CD);
#if 0
	s3c_gpio_cfgpin(S3C_GPJ15, S3C_GPJ15_OUTP);
#endif
	s3c_gpio_cfgpin(S3C_GPJ15, S3C_GPJ15_nSD0WP); /* write protect enable */


	s3c_gpio_setpin(S3C_GPJ15, 1);

	DBG("register read: GPLCON   = 0x%08x\n", readl(S3C_GPLCON));
	DBG("register read: GPJCON   = 0x%08x\n", readl(S3C_GPJCON));
	DBG("register read: GPJCON   = 0x%08x\n", readl(S3C_GPJDAT));
}
#endif /*  MACH_TOMTOMGO	*/


/*****************************************************************************\
 *                                                                           *
 * Tasklets                                                                  *
 *                                                                           *
\*****************************************************************************/

static void s3c_hsmmc_tasklet_card(unsigned long param)
{
	struct s3c_hsmmc_host *host;
	unsigned long iflags;

	host = (struct s3c_hsmmc_host*)param;
	spin_lock_irqsave( &host->complete_lock, iflags);

	if (!(s3c_hsmmc_readl(HM_PRNSTS) & S3C_HSMMC_CARD_PRESENT)) {
		if (host->mrq) {
			ERR("%s: Card removed during transfer!\n",
				mmc_hostname(host->mmc));
			ERR("%s: Resetting controller.\n",
				mmc_hostname(host->mmc));
			/* Controller will be reset in finish_tasklet */
			host->mrq->cmd->error = MMC_ERR_FAILED;
			tasklet_schedule(&host->finish_tasklet);
		}
	}

	spin_unlock_irqrestore( &host->complete_lock, iflags);

	mmc_detect_change(host->mmc, msecs_to_jiffies(500));
}

static void s3c_hsmmc_activate_led(struct s3c_hsmmc_host *host)
{
#if defined(CONFIG_MACH_TOMTOMGO)
#else
#if 0
	u8 ctrl;

	ctrl = s3c_hsmmc_readb(HM_HOSTCTL);
	ctrl |= S3C_HSMMC_CTRL_LED;
	s3c_hsmmc_writeb(ctrl, HM_HOSTCTL);
#endif

	s3c_gpio_cfgpin(S3C_GPJ13, S3C_GPJ13_SD0LED);
#endif
}

static void s3c_hsmmc_deactivate_led(struct s3c_hsmmc_host *host)
{
#if defined(CONFIG_MACH_TOMTOMGO)
#else
#if 0
	u8 ctrl;

	ctrl = s3c_hsmmc_readb(HM_HOSTCTL);
	ctrl &= ~S3C_HSMMC_CTRL_LED;
	s3c_hsmmc_writeb(ctrl, HM_HOSTCTL);
#endif

	s3c_gpio_cfgpin(S3C_GPJ13, S3C_GPJ13_INP);
#endif
}

static void s3c_hsmmc_tasklet_finish(unsigned long param)
{
	struct s3c_hsmmc_host *host;
	unsigned long iflags;
	struct mmc_request *mrq;

	host = (struct s3c_hsmmc_host*)param;

	spin_lock_irqsave(&host->complete_lock, iflags);

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if ((mrq->cmd->error != MMC_ERR_NONE) ||
		(mrq->data && ((mrq->data->error != MMC_ERR_NONE) ||
		(mrq->data->stop && (mrq->data->stop->error != MMC_ERR_NONE))))) {
		s3c_hsmmc_reset(host, S3C_HSMMC_RESET_CMD);
		s3c_hsmmc_reset(host, S3C_HSMMC_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	s3c_hsmmc_deactivate_led(host);

	spin_unlock_irqrestore(&host->complete_lock, iflags);

	mmc_request_done(host->mmc, mrq);
}


/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
\*****************************************************************************/

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

static irqreturn_t s3c_hsmmc_irq (int irq, void *dev_id, struct pt_regs *regs)
{
	irqreturn_t result = 0;
	struct s3c_hsmmc_host *host = dev_id;
	struct mmc_request *mrq;
	u16	intsts = 0, errint = 0;

	spin_lock(&host->complete_lock);

	mrq = host->mrq;

	intsts = s3c_hsmmc_readw(HM_NORINTSTS);

	if (!intsts) {
		result = IRQ_NONE;
		goto out;
	}

	if (host->mrq == NULL) {
		/* If we get here; we're here because of the reset-after-error-state. just
		 * acknowledge and forget about it.
		 */
		ERR("Unanticipated interrupt! host->mrq == NULL! (norintsts=%08x)\n", intsts);
		goto irq_handled;
	}
	
	DBG("got interrupt = 0x%08x\n", intsts);

	if (intsts & (S3C_HSMMC_INT_CARD_INSERT | S3C_HSMMC_INT_CARD_REMOVE)){

		if(intsts & S3C_HSMMC_INT_CARD_INSERT)
			INFO("card inserted.\n");
		else if(intsts & S3C_HSMMC_INT_CARD_REMOVE)
			INFO("card removed.\n");

		s3c_hsmmc_writel(intsts & (S3C_HSMMC_INT_CARD_INSERT |
			S3C_HSMMC_INT_CARD_REMOVE),HM_NORINTSTS);

		tasklet_schedule(&host->card_tasklet);
		goto insert;
	}

	intsts &= ~(S3C_HSMMC_INT_CARD_INSERT | S3C_HSMMC_INT_CARD_REMOVE);

	if (!(intsts & S3C_HSMMC_NIS_ERR)) {
		if (intsts & S3C_HSMMC_NIS_CMDCMP) {
			mrq->cmd->error = MMC_ERR_NONE;
		}

		if (intsts & S3C_HSMMC_NIS_TRSCMP) {
			mrq->cmd->error = MMC_ERR_NONE;
			complete(&host->complete_dma);
		}
	} else {
		errint = s3c_hsmmc_readw(HM_ERRINTSTS);
		if (errint & S3C_HSMMC_EIS_CMDTIMEOUT) {
			mrq->cmd->error = MMC_ERR_TIMEOUT;
			WARN("cmd timeout\n");
			complete(&host->complete_dma);
			goto transfer_closed;
		}

		if (errint & S3C_HSMMC_EIS_CMDERR) {
			ERR("cmd badcrc\n");
			mrq->cmd->error = MMC_ERR_BADCRC;
			complete(&host->complete_dma);
			goto transfer_closed;
		}

		if (errint & S3C_HSMMC_EIS_DATATIMEOUT) {
			ERR("data timeout\n");
			mrq->data->error = MMC_ERR_TIMEOUT;
			complete(&host->complete_dma);
			goto transfer_closed;
		}

		if (errint & S3C_HSMMC_EIS_DATAERR) {
			ERR("data badcrc\n");
			mrq->data->error = MMC_ERR_BADCRC;
			complete(&host->complete_dma);
			goto transfer_closed;
		}

		if (errint & S3C_HSMMC_EIS_CMD12ERR) {
			ERR("cmd12 error\n");
			mrq->cmd->error = MMC_ERR_INVALID;
			complete(&host->complete_dma);
			goto transfer_closed;
		}

	}

transfer_closed:
	complete(&host->complete_request);
irq_handled:
	s3c_hsmmc_writew(intsts, HM_NORINTSTS);
	s3c_hsmmc_writew(errint, HM_ERRINTSTS);

insert:
	result = IRQ_HANDLED;

out:
	spin_unlock(&host->complete_lock);

	return result;
}

static void s3c_hsmmc_check_status(unsigned long data)
{
        struct s3c_hsmmc_host *host = (struct s3c_hsmmc_host *)data;

	s3c_hsmmc_irq(0, host, NULL);
}

/*****************************************************************************\
 *                                                                           *
 * MMC callbacks                                                             *
 *                                                                           *
\*****************************************************************************/

static void s3c_hsmmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
 	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	u32 i, val, dma_dir;
	u16 cmd_val = 0;
	unsigned long iflags;
	u16 blocks;

	spin_lock_irqsave(&host->complete_lock, iflags);

	WARN_ON(host->mrq != NULL);

	DBG("hsmmc request: [CMD] opcode:%d arg:0x%08x flags:0x%02x retries:%u\n",
		mrq->cmd->opcode, mrq->cmd->arg, mrq->cmd->flags, mrq->cmd->retries);

	s3c_hsmmc_activate_led(host);

	host->mrq = mrq;

	/* Check CommandInhibit_CMD */
	for (i=0; i<1000000; i++) {
		val = s3c_hsmmc_readl(HM_PRNSTS);
		if (!(val & 0x1)) break;
	}
	if (i == 1000000) {
		ERR("HM_PRNSTS: %08x\n", val);
	}

	/* Check CommandInhibit_DAT */
	if (mrq->cmd->flags & MMC_RSP_BUSY) {
		for (i=0; i<1000000; i++) {
			val = s3c_hsmmc_readl(HM_PRNSTS);
			if (!(val & 0x2)) break;
		}
		if (i == 1000000) {
			ERR("HM_PRNSTS: %08x\n", val);
		}
	}

	s3c_hsmmc_writew(0x00ff, HM_NORINTSTSEN);
	s3c_hsmmc_writew(0x00ff, HM_ERRINTSTSEN);
	s3c_hsmmc_writew(0x00ff, HM_NORINTSIGEN);
	s3c_hsmmc_writew(0x00ff, HM_ERRINTSIGEN);

/*	if (!(s3c_hsmmc_readl(HM_PRNSTS) & S3C_HSMMC_CARD_PRESENT)) {
		ERR("Card not present!\n");
		mrq->cmd->error = MMC_ERR_TIMEOUT;
		tasklet_schedule(&host->finish_tasklet);
	} else*/ {


#if defined(CONFIG_MACH_TOMTOMGO)
		/* 
		 *  Because there is no reliable card detection on any of our hardware
		 *  we must relay on the state of CD_SD. If it is low we shouldn't
		 *  finalize any request even though it might be still possible.
		 */

		if ((mmc->removable) && PIN_IS_USED(IO_Pin(CD_SD)))
		{
			IO_SetInput(CD_SD);
			if (!IO_GetInput(CD_SD)) 
			{
				ERR("Card not present!\n");

				mrq->cmd->error = MMC_ERR_TIMEOUT;
				tasklet_schedule(&host->finish_tasklet);
				IO_SetInterruptOnToggle(CD_SD);

				spin_unlock_irqrestore(&host->complete_lock, iflags);
				return;
		        } 
			IO_SetInterruptOnToggle(CD_SD);
		}
#endif

		s3c_hsmmc_writel(mrq->cmd->arg, HM_ARGUMENT);

		cmd_val = ((mrq->cmd->opcode & S3C_SDICCON_INDEX) << 8);
		if (cmd_val == (12<<8))
			cmd_val |= (3 << 6);

		if (mrq->cmd->flags & MMC_RSP_136)	/* Long RSP */
			cmd_val |= S3C_HSMMC_CMD_RESP_LONG;
		else if (mrq->cmd->flags & MMC_RSP_BUSY)	/* R1B */
			cmd_val |= S3C_HSMMC_CMD_RESP_SHORT_BUSY;
		else if (mrq->cmd->flags & MMC_RSP_PRESENT)	/* Normal RSP */
			cmd_val |= S3C_HSMMC_CMD_RESP_SHORT;

		if (mrq->cmd->flags & MMC_RSP_OPCODE)
			cmd_val |= S3C_HSMMC_CMD_INDEX;

		if (mrq->cmd->flags & MMC_RSP_CRC)
			cmd_val |= S3C_HSMMC_CMD_CRC;

		if (mrq->data) {

			/* We set DMA enable for the trans mode */
			u16 trans_mode = S3C_HSMMC_TRNS_DMA;

			cmd_val |= S3C_HSMMC_CMD_DATA;

			/* DMA buffer size 111b(7) = 512K Byte */
			s3c_hsmmc_writew(S3C_HSMMC_MAKE_BLKSZ(7, mrq->data->blksz), HM_BLKSIZE);

			s3c_hsmmc_writew(mrq->data->blocks, HM_BLKCNT);

			DBG("request: [DAT] blocks:%u \n", mrq->data->blocks);

			if (mrq->data->flags & MMC_DATA_MULTI)
				trans_mode |= ( S3C_HSMMC_TRNS_MULTI |
#ifdef CONFIG_MMC_PREDEFINED_TRANSFER
					 S3C_HSMMC_TRNS_BLK_CNT_EN );
#else
					 S3C_HSMMC_TRNS_BLK_CNT_EN | S3C_HSMMC_TRNS_ACMD12 );
#endif

			if (mrq->data->flags & MMC_DATA_WRITE) {
				dma_dir = DMA_TO_DEVICE;
			} else { /* if (mrq->data->flags & MMC_DATA_READ) */
				trans_mode |= S3C_HSMMC_TRNS_READ;
				dma_dir = DMA_FROM_DEVICE;
			}

			/* start DMA */
			dma_map_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);
			s3c_hsmmc_writel(sg_dma_address(mrq->data->sg), HM_SYSAD);
			s3c_hsmmc_writew(trans_mode, HM_TRNMOD);
		}

		init_completion(&host->complete_request);
		init_completion(&host->complete_dma);

		/* This initiates transfer */
		s3c_hsmmc_writew(cmd_val, HM_CMDREG);

		/* this wait is very important to sd/mmc run correctly.
		 * Without this blocking code, operation sequence may be crashed.
		 * by scsuh.
		 */
		/* Wait for transfer to complete */
		spin_unlock_irqrestore(&host->complete_lock, iflags);

		wait_for_completion(&host->complete_request);

		if (mrq->data && (mrq->data->error == MMC_ERR_NONE)
				&& (mrq->cmd->error == MMC_ERR_NONE)) {
			wait_for_completion(&host->complete_dma);
		}

		spin_lock_irqsave(&host->complete_lock, iflags);

		if (mrq->cmd->flags & MMC_RSP_PRESENT) {
			if (mrq->cmd->flags & MMC_RSP_136) {
				/* CRC is stripped so we need to do some shifting. */
				for (i = 0;i < 4;i++) {
					mrq->cmd->resp[i] = s3c_hsmmc_readl(
							HM_RSPREG0+ (3-i)*4) << 8;
					if (i != 3)
						mrq->cmd->resp[i] |=
							s3c_hsmmc_readb(
								HM_RSPREG0 + (3-i)*4-1);
				}
			} else {
				mrq->cmd->resp[0] = s3c_hsmmc_readl(HM_RSPREG0);
			}
		}

		if (mrq->cmd->opcode == 0x09) {
			DBG("### %08x, %08x, %08x, %08x\n",
				mrq->cmd->resp[0], mrq->cmd->resp[1],
				mrq->cmd->resp[2], mrq->cmd->resp[3]);
		}

		host->mrq = NULL;
		DBG("request done.\n");

		if (mrq->data) {
			dma_unmap_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);
#if 0
			/* Calulate the amout of bytes transfer, but only if there was
			 * no error
			 */
			if (mrq->data->error == MMC_ERR_NONE)
				mrq->data->bytes_xfered = (mrq->data->blocks << mrq->data->blksz_bits);
			else
				mrq->data->bytes_xfered = 0;
#endif

			/*
			 * Controller doesn't count down when in single block mode.
			 */
			if ((mrq->data->blocks == 1) && (mrq->data->error == MMC_ERR_NONE))
				blocks = 0;
			else
				blocks = s3c_hsmmc_readw(HM_BLKCNT);
			mrq->data->bytes_xfered = mrq->data->blksz * (mrq->data->blocks - blocks);

			if ((mrq->data->error == MMC_ERR_NONE) && blocks) {
				ERR("%s: Controller signalled completion even though there were blocks left.\n",
				    mmc_hostname(host->mmc));
				mrq->data->error = MMC_ERR_FAILED;
			} else if (host->size != 0) {
				ERR("%s: %d bytes were left untransferred.\n",
				    mmc_hostname(host->mmc), host->size);
				mrq->data->error = MMC_ERR_FAILED;
			}

			DBG("Ending data transfer (%d bytes)\n", mrq->data->bytes_xfered);

			spin_unlock_irqrestore(&host->complete_lock, iflags);
			if (mrq->data->stop)
				mmc_wait_for_cmd(mmc, mrq->data->stop, 3);
			spin_lock_irqsave(&host->complete_lock, iflags);
#ifdef CONFIG_MMC_TRANSFER_LOGGING
			transfer_logger_log_transfer(host->mmc, sg_dma_address(mrq->data->sg),
			mrq->data->blksz, mrq->data->bytes_xfered, (dma_dir == DMA_TO_DEVICE)?1:0);
#endif
		}
		mrq->done(mrq);
	}
	spin_unlock_irqrestore(&host->complete_lock, iflags);
}


static void clock_onoff (struct s3c_hsmmc_host *host, int onoff)
{
	u16 val = readw(host->base + HM_CLKCON);

	if (onoff == 0) {
		writew(val & ( ~S3C_HSMMC_CLOCK_CARD_EN	), host->base + HM_CLKCON);
	} else {
		writew(val | S3C_HSMMC_CLOCK_CARD_EN , host->base + HM_CLKCON);
		while (1) {
			val = readw(host->base + HM_CLKCON);
			if (val & S3C_HSMMC_CLOCK_EXT_STABLE) break;
		}
	}
}

static void set_hostctl_speed(struct s3c_hsmmc_host *host, u8 speed)
{
	u8 reg;

	if (IO_GetCpuType() == GOCPU_S3C2450)
		speed = 0; // HOSTCTL:2 does not control speed but clock polarity on this SoC

	reg = s3c_hsmmc_readb(HM_HOSTCTL);
	reg &= ~S3C_HSMMC_CTRL_HIGHSPEED;
	s3c_hsmmc_writeb(reg | (speed << 2), HM_HOSTCTL);
}

/* return 0: OK
 * return -1: error
 */
static int set_bus_width (struct s3c_hsmmc_host *host, uint width)
{
	u8 reg;

	reg = s3c_hsmmc_readb(HM_HOSTCTL);

	switch (width) {
	case MMC_BUS_WIDTH_1:
		DBG("bus width: 1 bit\n");
		break;
	case MMC_BUS_WIDTH_4:
		DBG("bus width: 4 bit\n");
		reg |= S3C_HSMMC_CTRL_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		reg |= S3C_HSMMC_CTRL_8BIT;
		DBG("bus width: 8 bit\n");
		break;
	default:
		DBG("bus width: Error\n");
		return -1;
	}

	s3c_hsmmc_writeb(reg, HM_HOSTCTL);

	DBG("HOSTCTL(0x28) = 0x%02x\n", s3c_hsmmc_readb(HM_HOSTCTL));

	return 0;
}

static void hsmmc_set_clkcon(struct s3c_hsmmc_host *host, u16 div, u32 control2)
{
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		s3c_hsmmc_writel(CONTROL3_RXDELAY2, HM_CONTROL3);
		control2 &= ~S3C2450_HM_CONTROL2_ENFBCLKTX;
	} else {
		s3c_hsmmc_writel(CONTROL3_TXDELAY1|CONTROL3_RXDELAY1, HM_CONTROL3);
	}

	s3c_hsmmc_writel(control2, HM_CONTROL2);
	s3c_hsmmc_writew(((div<<8) | 0x0001), HM_CLKCON);
}

static void hsmmc_set_clkcon_slow(struct s3c_hsmmc_host *host)
{
	hsmmc_set_clkcon(host, 0x20, S3C2450_HM_CONTROL2_ENFBCLKTX
					| S3C2450_HM_CONTROL2_ENFBCLKRX
					| S3C2450_HM_CONTROL2_ENCLKOUTHOLD
					| S3C2450_HM_CONTROL2_SELBASECLK_EXT );
}


static void hsmmc_set_usb48clk(struct s3c_hsmmc_host *host, unsigned int clock)
{
	unsigned long clksrc;
	u16 div = 1;

	/* find a divider that gives us a clock below the max reported by the media */
	for ( div = 1; div < 256 && clock < (48000000 / div); div <<= 1 )
		;

	clksrc = readl(S3C2450_CLKSRC)           |
		 S3C2450_CLKSRC_SELHSMMC0_EXTCLK |
		 S3C2450_CLKSRC_SELHSMMC1_EXTCLK;
	writel(clksrc, S3C2450_CLKSRC);

	hsmmc_set_clkcon(host, div >> 1,  S3C2450_HM_CONTROL2_ENFBCLKTX
					| S3C2450_HM_CONTROL2_ENFBCLKRX
					| S3C2450_HM_CONTROL2_ENCLKOUTHOLD
					| S3C2450_HM_CONTROL2_SELBASECLK_SCLK);
}

static void hsmmc_set_epllclk(struct s3c_hsmmc_host *host, unsigned int clock)
{
#if 0
	writel(0x800, S3C_LOCKTIME1);
	writel( (readl( S3C_CLKSRC ) | (1 << 6) ), S3C_CLKSRC);  // EPLL Output
	SetEPLL(40, 1, 1);      //96MHz
	
	writel( readl(S3C_EPLLCON) & (~(1<<24)) , S3C_EPLLCON );
	writel((readl(S3C_CLKDIV1) & (~(0x3<<6 ))) | (0x0 << 6) , S3C_CLKDIV1);
	writel((readl(S3C_MISCCR)  & (~(0x7<<8 ))) | (0x1 << 8) , S3C_MISCCR);
	writel((readl(S3C_GPHCON)  & (~(0x3<<28))) | (0x1 << 29), S3C_GPHCON);
	
	hsmmc_set_clkcon(host, 0x1, S3C2450_HM_CONTROL2_ENFBCLKTX    |\
			            S3C2450_HM_CONTROL2_ENFBCLKRX    |\
				    S3C2450_HM_CONTROL2_ENCLKOUTHOLD |\
				    S3C2450_HM_CONTROL2_SELBASECLK_SCLK);
	
	DBG("CLKCON  = 0x%08x\n", readl(S3C_CLKSRC));
	DBG("CLKDIV1 = 0x%08x\n", readl(S3C_CLKDIV1));
	DBG("EPLLCON = 0x%08x\n", readl(S3C_EPLLCON));
	DBG("MISCCR  = 0x%08x\n", readl(S3C_MISCCR));
	DBG("EPLL : hsmmc clock \n");
#endif
}

static void hsmmc_set_hclk(struct s3c_hsmmc_host *host, unsigned int clock)
{
	u16 div;

	for (div=0x1; div<0x80; div <<= 1) {
		if ( (clk_get_rate(host->clk) / (div<<1)) <= clock)
			break;
	}
			
	DBG("HCLK : hsmmc clock = %ld MHz \n", (clk_get_rate(host->clk)/(div<<1))/1000000);
	hsmmc_set_clkcon(host, div, S3C2450_HM_CONTROL2_ENFBCLKTX    |\
				    S3C2450_HM_CONTROL2_ENFBCLKRX    |\
				    S3C2450_HM_CONTROL2_ENCLKOUTHOLD |\
				    S3C2450_HM_CONTROL2_SELBASECLK_HCLK);
}

static void hsmmc_set_clock(struct s3c_hsmmc_host *host, unsigned int clock)
{
	u32 val;
	int i;

	if (clock < 0x100000) {
		hsmmc_set_clkcon_slow(host);
	} else {
		host->hsmmc_set_clock(host, clock);
	}

	for (i=0; i<0x1000000; i++) {
		val = s3c_hsmmc_readw(HM_CLKCON);
		if (val & (1<<1)) break;
	}
	if (i == 0x1000000)
		ERR("error in clock stabilization\n");

}

static void s3c_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	unsigned long iflags;

	spin_lock_irqsave(&host->complete_lock, iflags);

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {

		s3c_hsmmc_writew(S3C_HSMMC_INT_MASK_ALL, HM_NORINTSIGEN);

		spin_unlock_irqrestore(&host->complete_lock, iflags);

		s3c_hsmmc_ios_init(host);

#ifdef CONFIG_MACH_TOMTOMGO
		host->hsmmc_hw_exit();
#endif
		spin_lock_irqsave(&host->complete_lock, iflags);

	} else {

#ifdef CONFIG_MACH_TOMTOMGO
		host->hsmmc_hw_init();
#else
		s3c_hsmmc_set_gpio();
#endif /* CONFIG_MACH_TOMTOMGO  */

		s3c_hsmmc_writeb(S3C_HSMMC_RESET_ALL | S3C_HSMMC_RESET_CMD, HM_SWRST);

		/* disable clock controller */
		clock_onoff(host, S3C_HSMMC_CLOCK_OFF);

		s3c_hsmmc_writeb(S3C_HSMMC_TIMEOUT_MAX, HM_TIMEOUTCON);

		set_hostctl_speed(host, SPEED_HIGH);

		hsmmc_set_clock(host, ios->clock);
		set_bus_width(host, ios->bus_width);

		clock_onoff(host, S3C_HSMMC_CLOCK_ON);

		DBG("HM_CONTROL2(0x80) = 0x%08x\n", s3c_hsmmc_readl(HM_CONTROL2));
		DBG("HM_CONTROL3(0x84) = 0x%08x\n", s3c_hsmmc_readl(HM_CONTROL3));
		DBG("HM_CLKCON  (0x2c) = 0x%04x\n", s3c_hsmmc_readw(HM_CLKCON));
	}

	if (ios->power_mode == MMC_POWER_OFF)
		s3c_hsmmc_writeb(S3C_HSMMC_POWER_OFF, HM_PWRCON);
	else
		s3c_hsmmc_writeb(S3C_HSMMC_POWER_ON_ALL, HM_PWRCON);

	udelay(1000);
	spin_unlock_irqrestore(&host->complete_lock, iflags);
}

static struct mmc_host_ops s3c_hsmmc_ops = {
	.request	= s3c_hsmmc_request,
	.set_ios	= s3c_hsmmc_set_ios,
};

static int s3c_hsmmc_probe(struct device *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct s3c_hsmmc_host *host;
	struct mmc_host *mmc;

	int ret;

	mmc = mmc_alloc_host(sizeof(struct s3c_hsmmc_host), &pdev->dev);
	if (!mmc)
		ret = -ENOMEM;

	host = mmc_priv(mmc);

#ifdef CONFIG_MACH_TOMTOMGO
	hsmmc_set_gpio(host, pdev);
	host->hsmmc_hw_init();
#endif

	spin_lock_init(&host->complete_lock);

	host->complete_what 	= COMPLETION_NONE;
	host->mmc 		= mmc;
//	host->dma		= S3C_SDI_DMA;

	host->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mem) {
		ERR("failed to get io memory region resouce.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
		RESSIZE(host->mem), pdev->name);
	if (!host->mem) {
		ERR("failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		ERR("failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto untasklet;
	}

	host->base = (void __iomem *) host->mem->start;

	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

	host->clk = clk_get(&pdev->dev, "hsmmc");
	if (IS_ERR(host->clk)) {
		ERR("failed to find clock source.\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_host;
	}

	if ((ret = clk_enable(host->clk))) {
		ERR("failed to enable clock source.\n");
		goto clk_free;
	}

	host->clk_ext = clk_get(&pdev->dev, "hsmmc-ext");
	if (IS_ERR(host->clk_ext)) {
		ERR("failed to find clock source: hsmmc-ext\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_host;
	}

	if ((ret = clk_enable(host->clk_ext))) {
		ERR("failed to enable clock source: hsmmc-ext\n");
		goto clk_free;
	}

	mmc->ops = &s3c_hsmmc_ops;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	mmc->f_min = 2000;

	/* you must make sure that our sdmmc block can support
	 * up to 25MHz. by scsuh
	 */
	mmc->f_max = 100 * MHZ;

	/* Set the capabilities. Enable MULTIWRITE for higher performance. */
//#ifdef CONFIG_MMC_8_BIT_TRANSFERS
#if 0
	mmc->caps = MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_MULTIWRITE;
#else
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_MULTIWRITE | MMC_CAP_SD_HIGHSPEED;
#endif

	/* disable high speed on port 1 on Treviso */
	if (pdev->id == 1)
		mmc->caps &= ~(MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED);
	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 * Choose 64 (512-byte) sectors as the limit.
	 */
	mmc->max_req_size = 65535;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */

	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Block count is stored in a 16-bit register.
	 */
	mmc->max_blk_count = 65535;

	platform_set_drvdata(pdev, mmc);

	init_timer(&host->timer);
        host->timer.data = (unsigned long)host;
        host->timer.function = s3c_hsmmc_check_status;
        host->timer.expires = jiffies + HZ;

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->card_tasklet,
		s3c_hsmmc_tasklet_card, (unsigned long)host);
	tasklet_init(&host->finish_tasklet,
		s3c_hsmmc_tasklet_finish, (unsigned long)host);

	ret = request_irq(host->irq, s3c_hsmmc_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto untasklet;

	mmc_add_host(mmc);

#ifdef CONFIG_MACH_TOMTOMGO
	if (pdev->id == 1)
	{
		host->irq_cd = IO_GetInterruptNumber(CD_SD);
		set_irq_type(host->irq_cd, IRQT_BOTHEDGE);
		mmc->removable = 1;

		if (host->irq_cd > 0) {
			if (request_irq(host->irq_cd, hsmmc_irq_cd, SA_INTERRUPT, DRIVER_NAME, host)) {
				ERR("failed to request card detect interrupt.\n" );
				ret = -ENOENT;
				goto remove_host;
			}
		}
	}
#endif

	INFO("%s: irq = %d, base = %p \n", mmc_hostname(mmc), host->irq , host->base);
	INFO("initialization done.\n");
#ifdef CONFIG_MMC_TRANSFER_LOGGING
	transfer_logger_add(host->mmc);
#endif
	return 0;

#ifdef CONFIG_MACH_TOMTOMGO
remove_host:
	mmc_remove_host(mmc);
	free_irq(host->irq, host);
#endif

untasklet:
	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

clk_free:
	clk_put(host->clk);

probe_free_host:
	mmc_free_host(mmc);

	return ret;
}

static int s3c_hsmmc_remove(struct device *_dev)
{
	struct platform_device* dev = to_platform_device(_dev);
	struct mmc_host *mmc  = platform_get_drvdata(dev);
	struct s3c_hsmmc_host *host = mmc_priv(mmc);

#ifdef CONFIG_MACH_TOMTOMGO
	if (dev->id == 1)
		free_irq(host->irq_cd, host);
#endif
	mmc = host->mmc;
#ifdef CONFIG_MMC_TRANSFER_LOGGING
	transfer_logger_remove(host->mmc);
#endif

	mmc_remove_host(mmc);

	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

	clk_disable(host->clk);
	clk_put(host->clk);

	free_irq(host->irq, host);

	del_timer_sync(&host->timer);

	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	mmc_free_host(mmc);

	return 0;
}

#ifdef CONFIG_PM

static int s3c_hsmmc_suspend(struct device *dev, pm_message_t state, u32 level)
{
	DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		struct mmc_host *mmc = dev_get_drvdata(dev);
		return mmc ? mmc_suspend_host(mmc, state) : -ENODEV;
	}
	return 0;
}

static int s3c_hsmmc_resume(struct device *dev, u32 level)
{
	DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		struct mmc_host *mmc = dev_get_drvdata(dev);
#if defined(CONFIG_MACH_TOMTOMGO)
		struct s3c_hsmmc_host *host = mmc_priv(mmc);
		host->hsmmc_hw_init();
#else
		s3c_hsmmc_set_gpio();
#endif
		return mmc ? mmc_resume_host(mmc) : -ENODEV;
	}
	return 0;
}

#else /* CONFIG_PM */
#define s3c_hsmmc_suspend NULL
#define s3c_hsmmc_resume NULL
#endif /* CONFIG_PM */

static struct device_driver s3c_hsmmc_driver =
{
	.name		= "s3c-hsmmc",
	.bus		= &platform_bus_type,
	.probe		= s3c_hsmmc_probe,
	.remove		= s3c_hsmmc_remove,
	.suspend	= s3c_hsmmc_suspend,
	.resume		= s3c_hsmmc_resume,
};

static int __init s3c_hsmmc_init(void)
{
	return driver_register(&s3c_hsmmc_driver);
}

static void __exit s3c_hsmmc_exit(void)
{
	driver_unregister(&s3c_hsmmc_driver);
}

module_init(s3c_hsmmc_init);
module_exit(s3c_hsmmc_exit);

MODULE_DESCRIPTION("S3C SD HOST I/F 1.0 Driver");
MODULE_LICENSE("GPL");
