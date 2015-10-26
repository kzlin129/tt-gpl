/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <asm/io.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>

#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>
#include <asm/arch/spi.h>
#include <asm/arch/spi_ioctl.h>

//#include "tahoe_opmode.h"

#define SPI_DRV_DEV_NAME   "spi"
#define NUM_SPI_DEVICES    2

#ifndef ARRAYSIZE
#define ARRAYSIZE(a)	(sizeof(a)/sizeof((a)[0]))
#endif

#define SPI_MAJOR	        153
#define SPI_MINOR	        0

#define SPI_CMD_BUF_SIZE	256		// in bytes
#define SPI_DATA_BUF_SIZE	4096	// in bytes

#define SPI_WAIT_DMA_CHAN	100

/* SSPSR status register */
#define SPI_SSPSR_BSY	(0x10) /* Busy */
#define SPI_SSPSR_RFF	(0x08) /* Rx FIFO 1: full */
#define SPI_SSPSR_RNE	(0x04) /* Rx FIFO 1: not empty */
#define SPI_SSPSR_TNF	(0x02) /* Tx FIFO 1: not full */
#define SPI_SSPSR_TFE	(0x01) /* Tx FIFO 1: not empty */

typedef struct
{
	u16 sspcr0;
	u16 dummy0;
	u8 sspcr1;
	u8 dummy1[3];
	u16 sspdr;
	u16 dummy2;
	u8 sspsr;
	u8 dummy3[3];
	u8 sspcpsr;
	u8 dummy4[3];
	u8 sspimsc;
	u8 dummy5[3];
	u8 sspris;
	u8 dummy6[3];
	u8 sspmis;
	u8 dummy7[3];
	u8 sspicr;
	u8 dummy8[3];
	u8 sspdmacr;
	u8 dummy9[3];
} bcm476x_spi_control_regs_t;

typedef struct {
	int ch;
	DMAC_CTRL_REG src;
	DMAC_CTRL_REG dst;
	DMAC_CFG_REG cfg;
	DMAC_CH_REG ch_reg;
	u32 src_addr[2];
	u32 dst_addr[2];
	u32 transfer_size[2];
	struct completion dma_complete;
} bcm476x_spi_dma_ch_t;

typedef struct {
	u32 addrbase;
	u32 sspdr_phys_addr;
	u16 dma_requestor_id_tx; // write to spi tx fifo
	u16 dma_requestor_id_rx; // read from spi rx fifo
	u32 in_use;
	volatile bcm476x_spi_control_regs_t *control;
	bcm476x_spi_dma_ch_t tx_dma_ch; // mem->spi
	bcm476x_spi_dma_ch_t rx_dma_ch; // spi->mem
	spi_addr_t const_0;
	spi_callback_t spi_callback;
	spi_addr_t *cmd_buf;
	spi_addr_t *data_buf;
	uint cmd_length;
	spi_addr_t cmd_tmp_buf;
	spi_addr_t data_tmp_buf;
	uint data_frame_size;	// in bits
	struct semaphore lock;
} bcm476x_spi_dev_t;

static bcm476x_spi_dev_t spi_dev[] =
{
{SPI0_REG_BASE_ADDR, SPI0_R_SSPDR_MEMADDR, BCM476X_DMAC_PERIPHERAL_SPI0_Tx, BCM476X_DMAC_PERIPHERAL_SPI0_Rx, 0, NULL},
{SPI1_REG_BASE_ADDR, SPI1_R_SSPDR_MEMADDR, BCM476X_DMAC_PERIPHERAL_SPI1_Tx, BCM476X_DMAC_PERIPHERAL_SPI1_Rx, 0, NULL}
};

static struct cdev spi_cdev;

static int spi_is_ready(bcm476x_spi_dev_t *spi);
static void spi_clean_fifo(bcm476x_spi_dev_t *spi);
static int spi_dma_read(bcm476x_spi_dev_t *spi, dma_addr_t buf_phys, uint length);
static int spi_dma_write(bcm476x_spi_dev_t *spi, dma_addr_t buf_phys, uint length);
static void spi_dma_handler(unsigned long tag, unsigned long err_code);

static inline void spi_set_local_dma_enable(bcm476x_spi_dev_t *spi, int tx, int rx)
{
	spi->control->sspdmacr = (tx&1)<<1|(rx&1);
}

static void spi_init_dma_channel(bcm476x_spi_dev_t *spi);

int bcm476x_spi_read(int spi_select, spi_addr_t *rx_buf, int length)
{
	bcm476x_spi_dev_t *spi = &spi_dev[spi_select];

// set up cmd buffer
	if (spi->spi_callback) {
		(*spi->spi_callback)(&spi->cmd_buf, &spi->cmd_length);
	} else {
		spi->cmd_length = 0;
	}

	if (!(spi->cmd_length||length)) {
		return 0;
	}

	if ((spi->cmd_length + length) <= SPI_FIFO_DEPTH) {
		// only support data frame size of 8 or 16 bits
		if (spi->data_frame_size > 8) {

			volatile bcm476x_spi_control_regs_t *control;
			u16 *ptr;
			int i;

			while (!spi_is_ready(spi));
			spi_clean_fifo(spi);

			control = spi->control;
			ptr = (u16 *)spi->cmd_buf->virt;
			for (i = 0; i < spi->cmd_length; i++) {
				control->sspdr = ptr[i];
			}
			for (i = 0; i < length; i++) {
				control->sspdr = 0;
			}

			while (!spi_is_ready(spi));

			for (i = 0; i < spi->cmd_length; i++) {
				ptr[i] = control->sspdr;
			}

			if (length && rx_buf) {
				ptr = (u16 *)rx_buf->virt;
				for (i = 0; i < length; i++) {
					ptr[i] = control->sspdr;
				}
			}

		} else {

			volatile bcm476x_spi_control_regs_t *control;
			u8 *ptr;
			int i;

			while (!spi_is_ready(spi));
			spi_clean_fifo(spi);

			control = spi->control;
			ptr = (u8 *)spi->cmd_buf->virt;
			for (i = 0; i < spi->cmd_length; i++) {
				control->sspdr = ptr[i];
			}
			for (i = 0; i < length; i++) {
				control->sspdr = 0;
			}

			while (!spi_is_ready(spi));

			for (i = 0; i < spi->cmd_length; i++) {
				ptr[i] = control->sspdr;
			}

			if (length && rx_buf) {
				ptr = (u8 *)rx_buf->virt;
				for (i = 0; i < length; i++) {
					ptr[i] = control->sspdr;
				}
			}
		}
		return length;

	} else {
		return spi_dma_read(spi, rx_buf->phys, length);
	}
}

int bcm476x_spi_write(int spi_select, spi_addr_t *tx_buf, int length)
{
	bcm476x_spi_dev_t *spi = &spi_dev[spi_select];

// set up cmd buffer
	if (spi->spi_callback) {
		(*spi->spi_callback)(&spi->cmd_buf, &spi->cmd_length);
	} else {
		spi->cmd_length = 0;
	}

	if (!(spi->cmd_length||length)) {
		return 0;
	}

	if ((spi->cmd_length + length) <= SPI_FIFO_DEPTH) {
		// only support data frame size of 8 or 16 bits
		if (spi->data_frame_size > 8) {

			volatile bcm476x_spi_control_regs_t *control = spi->control;
			u16 *ptr;
			int i;

			while (!spi_is_ready(spi));

			ptr = (u16 *)spi->cmd_buf->virt;
			for (i = 0; i < spi->cmd_length; i++) {
				control->sspdr = ptr[i];
			}

			if (length && tx_buf) {
				ptr = (u16 *)tx_buf->virt;
				for (i = 0; i < length; i++) {
					control->sspdr = ptr[i];
				}
			}

		} else {

			volatile bcm476x_spi_control_regs_t *control = spi->control;
			u8 *ptr;
			int i;

			while (!spi_is_ready(spi));

			ptr = (u8 *)spi->cmd_buf->virt;
			for (i = 0; i < spi->cmd_length; i++) {
				control->sspdr = ptr[i];
			}

			if (length && tx_buf) {
				ptr = (u8 *)tx_buf->virt;
				for (i = 0; i < length; i++) {
					control->sspdr = ptr[i];
				}
			}
		}
		return length;

	} else {
		return spi_dma_write(spi, tx_buf->phys, length);
	}
}

// Tx dummy data in order to generate clock for Rx
static int spi_dma_read(bcm476x_spi_dev_t *spi, dma_addr_t buf_phys, uint length)
{
	int i;
	int tx_dma_ch_num, rx_dma_ch_num;
	bcm476x_spi_dma_ch_t *tx_dma_ch, *rx_dma_ch;

	for (i = 0; i < SPI_WAIT_DMA_CHAN; i++) {
		if ((tx_dma_ch_num = bcm476x_request_dma_channel()) != -1) {
			break;
		} else {
			schedule_timeout(10 * HZ);
		}
	}

	if (tx_dma_ch_num < 0) {
		printk(KERN_ERR "SPI: no DMA channel available\n");
		return -EIO;
	}

	for (i = 0; i < SPI_WAIT_DMA_CHAN; i++) {
		if ((rx_dma_ch_num = bcm476x_request_dma_channel()) != -1) {
			break;
		} else {
			schedule_timeout(10 * HZ);
		}
	}

	if (rx_dma_ch_num < 0) {
		bcm476x_release_dma_channel(tx_dma_ch_num);
		printk(KERN_ERR "SPI: no DMA channel available\n");
		return -EIO;
	}

// set up DMA
	tx_dma_ch = &spi->tx_dma_ch;
	rx_dma_ch = &spi->rx_dma_ch;

	tx_dma_ch->ch = tx_dma_ch_num;
	rx_dma_ch->ch = rx_dma_ch_num;

	i = 0;
	tx_dma_ch->src.n_addr = 0;
	rx_dma_ch->src.n_addr = 0;
	if (spi->cmd_length&&spi->cmd_buf) {
// set up mem->spi DMA
		tx_dma_ch->src_addr[i] = spi->cmd_buf->phys;
		tx_dma_ch->transfer_size[i] = spi->cmd_length;
		tx_dma_ch->src.n_addr++;

// set up spi->mem DMA
		rx_dma_ch->transfer_size[i] = spi->cmd_length;
		rx_dma_ch->dst_addr[i] = spi->cmd_buf->phys;
		rx_dma_ch->src.n_addr++;
		i++;
	}
	if (length&&buf_phys) {
// set up mem->spi DMA
		tx_dma_ch->src_addr[i] = (u32)spi->const_0.phys;
		tx_dma_ch->transfer_size[i] = length;
		tx_dma_ch->src.n_addr++;

// set up spi->mem DMA
		rx_dma_ch->transfer_size[i] = length;
		rx_dma_ch->dst_addr[i] = buf_phys;
		rx_dma_ch->src.n_addr++;

		i++;
	}
	tx_dma_ch->src.incr = 0;
	tx_dma_ch->src.flags = 0;   // disable Tx DMA interrupt
	tx_dma_ch->dst.n_addr = tx_dma_ch->src.n_addr;
	rx_dma_ch->dst.n_addr = rx_dma_ch->src.n_addr;

	bcm476x_initialize_dma_chain(tx_dma_ch->ch, &tx_dma_ch->src, &tx_dma_ch->dst, &tx_dma_ch->cfg, &tx_dma_ch->ch_reg);
	if (spi->cmd_length && spi->cmd_buf) {
		tx_dma_ch->ch_reg.control |= DMA_F_SI_MASK; // set flag to increment source address after transfer
	}
	bcm476x_setup_dma_chain(rx_dma_ch->ch, &rx_dma_ch->src, &rx_dma_ch->dst, &rx_dma_ch->cfg);

	init_completion(&tx_dma_ch->dma_complete);
	bcm476x_register_dma_handler(tx_dma_ch->ch, (unsigned long)tx_dma_ch, spi_dma_handler);
	init_completion(&rx_dma_ch->dma_complete);
	bcm476x_register_dma_handler(rx_dma_ch->ch, (unsigned long)rx_dma_ch, spi_dma_handler);

// wait for last command to finish and then empty Rx fifo
	while (!spi_is_ready(spi));
	spi_clean_fifo(spi);

// kick off DMA
	spi_set_local_dma_enable(spi, 1, 1);
	bcm476x_enable_dma_channel(rx_dma_ch->ch);
	bcm476x_dma_fast_setup(tx_dma_ch->ch, &tx_dma_ch->ch_reg);

//  wait_for_completion(&tx_dma_ch->dma_complete);
	wait_for_completion(&rx_dma_ch->dma_complete);

	bcm476x_disable_dma_channel(tx_dma_ch->ch, 0);
	bcm476x_disable_dma_channel(rx_dma_ch->ch, 0);
	spi_set_local_dma_enable(spi, 0, 0);

	bcm476x_release_dma_channel(tx_dma_ch->ch);
	bcm476x_release_dma_channel(rx_dma_ch->ch);

	return length;
}

static int spi_dma_write(bcm476x_spi_dev_t *spi, dma_addr_t buf_phys, uint length)
{
	int i;
	int tx_dma_ch_num;
	bcm476x_spi_dma_ch_t *tx_dma_ch;

	for (i = 0; i < SPI_WAIT_DMA_CHAN; i++) {
		if ((tx_dma_ch_num = bcm476x_request_dma_channel()) != -1) {
			break;
		} else {
			schedule_timeout(10 * HZ);
		}
	}

	if (tx_dma_ch_num < 0) {
		printk(KERN_ERR "SPI: no DMA channel available\n");
		return -EIO;
	}

// set up DMA
	tx_dma_ch = &spi->tx_dma_ch;
	tx_dma_ch->ch = tx_dma_ch_num;

// set up mem->spi DMA
	i = 0;
	tx_dma_ch->src.n_addr = 0;
	if (spi->cmd_length&&spi->cmd_buf) {
		tx_dma_ch->src_addr[i] = spi->cmd_buf->phys;
		// Only source transfer size in DMAC_CTRL_REG src is used by DMA device driver.
		tx_dma_ch->transfer_size[i] = spi->cmd_length;
		tx_dma_ch->src.n_addr++;
		i++;
	}
	if (length&&buf_phys) {
		tx_dma_ch->src_addr[i] = buf_phys;
		// Only source transfer size in DMAC_CTRL_REG src is used by DMA device driver.
		tx_dma_ch->transfer_size[i] = length;
		tx_dma_ch->src.n_addr++;
		i++;
	}
	tx_dma_ch->src.incr = 1;
	tx_dma_ch->src.flags = BCM476X_EN_DMA_LAST_TC_INT;  // enable Tx DMA interrupt
	tx_dma_ch->dst.n_addr = tx_dma_ch->src.n_addr;

	bcm476x_setup_dma_chain(tx_dma_ch->ch, &tx_dma_ch->src, &tx_dma_ch->dst, &tx_dma_ch->cfg);

	init_completion(&tx_dma_ch->dma_complete);
	bcm476x_register_dma_handler(tx_dma_ch->ch, (unsigned long)tx_dma_ch, spi_dma_handler);

// wait for last command to finish
	while (!spi_is_ready(spi));

// kick off DMA
	spi_set_local_dma_enable(spi, 1, 0);
	bcm476x_enable_dma_channel(tx_dma_ch->ch);

	wait_for_completion(&tx_dma_ch->dma_complete);

	bcm476x_disable_dma_channel(tx_dma_ch->ch, 0);
	spi_set_local_dma_enable(spi, 0, 0);

	bcm476x_release_dma_channel(tx_dma_ch->ch);

	return length;
}

static void spi_dma_handler(unsigned long tag, unsigned long err_code)
{
	bcm476x_spi_dma_ch_t *spi_dma_ch = (bcm476x_spi_dma_ch_t *)tag;
	complete(&spi_dma_ch->dma_complete);
}

static int spi_is_ready(bcm476x_spi_dev_t *spi)
{
	return (spi->control->sspsr & 0x10) ? 0:1;
}

static void spi_clean_fifo(bcm476x_spi_dev_t *spi)
{
	volatile bcm476x_spi_control_regs_t *control = spi->control;
	while (control->sspsr & 0x4) {
		control->sspdr;
	}
}

static void spi_init_dma_channel(bcm476x_spi_dev_t *spi)
{
	bcm476x_spi_dma_ch_t *tx_dma_ch, *rx_dma_ch;
	tx_dma_ch = &spi->tx_dma_ch;
	rx_dma_ch = &spi->rx_dma_ch;

// tx channel: mem2spi
	tx_dma_ch->src_addr[0] = (u32)spi->const_0.phys; // change at run time
	tx_dma_ch->src_addr[1] = (u32)spi->const_0.phys; // change at run time
	tx_dma_ch->src.addr      = tx_dma_ch->src_addr;
	tx_dma_ch->transfer_size[0] = 1; // change at run time
	tx_dma_ch->transfer_size[1] = 1; // change at run time
	tx_dma_ch->src.tfr_size  = tx_dma_ch->transfer_size;
	tx_dma_ch->src.n_addr    = 1;   // change at run time
	tx_dma_ch->src.incr      = 0;   // change at run time
	// only support data frame size of 8 or 16 bits
	if (spi->data_frame_size > 8) {
		tx_dma_ch->src.width     = BCM476X_DMAC_TRANSFER_WIDTH_16BIT;
	} else {
		tx_dma_ch->src.width     = BCM476X_DMAC_TRANSFER_WIDTH_8BIT;
	}
//    tx_dma_ch->src.burst_sz  = BCM476X_DMAC_BURST_SIZE_4;
	tx_dma_ch->src.burst_sz  = BCM476X_DMAC_BURST_SIZE_1;
	tx_dma_ch->src.flags     = 0;   // change at run time

	tx_dma_ch->dst_addr[0]   = spi->sspdr_phys_addr;
	tx_dma_ch->dst_addr[1]   = spi->sspdr_phys_addr;
	tx_dma_ch->dst.addr      = tx_dma_ch->dst_addr;
	tx_dma_ch->dst.tfr_size  = tx_dma_ch->transfer_size;
	tx_dma_ch->dst.n_addr    = 1;
	tx_dma_ch->dst.incr      = 0;
	// only support data frame size of 8 or 16 bits
	if (spi->data_frame_size > 8) {
		tx_dma_ch->dst.width     = BCM476X_DMAC_TRANSFER_WIDTH_16BIT;
	} else {
		tx_dma_ch->dst.width     = BCM476X_DMAC_TRANSFER_WIDTH_8BIT;
	}
//    tx_dma_ch->dst.burst_sz  = BCM476X_DMAC_BURST_SIZE_4;
	tx_dma_ch->dst.burst_sz  = BCM476X_DMAC_BURST_SIZE_1;
	tx_dma_ch->dst.flags     = 0;

	tx_dma_ch->cfg.src_id    = 0; /* memory */
	tx_dma_ch->cfg.dst_id    = spi->dma_requestor_id_tx;
	tx_dma_ch->cfg.trans_type = BCM476X_DMAC_MEM2PERI_CTL;

// rx channel: spi2mem
	rx_dma_ch->src_addr[0]   = spi->sspdr_phys_addr;
	rx_dma_ch->src_addr[1]   = spi->sspdr_phys_addr;
	rx_dma_ch->src.addr      = rx_dma_ch->src_addr;
	rx_dma_ch->transfer_size[0] = 1; // change at run time
	rx_dma_ch->transfer_size[1] = 1; // change at run time
	rx_dma_ch->src.tfr_size  = rx_dma_ch->transfer_size;
	rx_dma_ch->src.n_addr    = 1;    // change at run time
	rx_dma_ch->src.incr      = 0;
	// only support data frame size of 8 or 16 bits
	if (spi->data_frame_size > 8) {
		rx_dma_ch->src.width     = BCM476X_DMAC_TRANSFER_WIDTH_16BIT;
	} else {
		rx_dma_ch->src.width     = BCM476X_DMAC_TRANSFER_WIDTH_8BIT;
	}
//    rx_dma_ch->src.burst_sz  = BCM476X_DMAC_BURST_SIZE_4;
	rx_dma_ch->src.burst_sz  = BCM476X_DMAC_BURST_SIZE_1;
	rx_dma_ch->src.flags     = BCM476X_EN_DMA_LAST_TC_INT;

	// rx_dma_ch->dst_addr[0] change at run time
	// rx_dma_ch->dst_addr[1] change at run time
	rx_dma_ch->dst.addr      = rx_dma_ch->dst_addr;
	rx_dma_ch->dst.tfr_size  = rx_dma_ch->transfer_size;
	rx_dma_ch->dst.n_addr    = 1;    // change at run time
	rx_dma_ch->dst.incr      = 1;
	// only support data frame size of 8 or 16 bits
	if (spi->data_frame_size > 8) {
		rx_dma_ch->dst.width     = BCM476X_DMAC_TRANSFER_WIDTH_16BIT;
	} else {
		rx_dma_ch->dst.width     = BCM476X_DMAC_TRANSFER_WIDTH_8BIT;
	}
//    rx_dma_ch->dst.burst_sz  = BCM476X_DMAC_BURST_SIZE_4;
	rx_dma_ch->dst.burst_sz  = BCM476X_DMAC_BURST_SIZE_1;
	rx_dma_ch->dst.flags     = BCM476X_EN_DMA_LAST_TC_INT;

	rx_dma_ch->cfg.src_id = spi->dma_requestor_id_rx;
	rx_dma_ch->cfg.dst_id = 0; /* memory */
	rx_dma_ch->cfg.trans_type = BCM476X_DMAC_PERI2MEM_CTL;
}

void spi_set_config(int spi_select, spi_control_t *spi_ctrl)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;

	spi = &spi_dev[spi_select];
	control = spi->control;

	// Disable serial port operation
	control->sspcr1 = 0;

	// Set serial clock rate, phase, polarity, frame format and data size
	// bit rate = Fsspclk / (CPSDVR * (1 + SCR)
	// where CPSDVR is an even value from 2 to 254 programmed through SSPCPSR register
	// and SCR is a value from 0 to 255
	control->sspcr0 = spi_ctrl->regs.cr0;
	spi->data_frame_size = (control->sspcr0 & 0xf) + 1;

	// Clock prescale register set to 2, with SCR =0 gives fastest possible clock
	// Must be an even number (2-254)
	control->sspcpsr = spi_ctrl->regs.cpsdvsr;

	// Mask all FIFO/IRQ interrupts
	control->sspimsc = 0;

	// Set master/slave mode and enable serial port operation
	control->sspcr1 = spi_ctrl->regs.cr1 | 0x2;
}

static int spi_set_clk_prescaler(int spi_select, uint cpsdvr)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;

	if (cpsdvr & 1) {
		return -1;
	}

	spi = &spi_dev[spi_select];
	control = spi->control;
	control->sspcpsr = (cpsdvr & 0xfe);
	return 0;
}

static int spi_set_clk_rate(int spi_select, uint scr)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u16 sspcr0;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr0 = control->sspcr0;
	sspcr0 &= 0xff;
	sspcr0 |= (scr << 8);
	control->sspcr0 = sspcr0;
	return 0;
}

static int spi_set_clk_phase(int spi_select, uint sph)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u16 sspcr0;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr0 = control->sspcr0;
	sspcr0 &= ~(1 << 7);
	sspcr0 |= ((sph & 1) << 7);
	control->sspcr0 = sspcr0;
	return 0;
}

static int spi_set_clk_polarity(int spi_select, uint spo)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u16 sspcr0;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr0 = control->sspcr0;
	sspcr0 &= ~(1 << 6);
	sspcr0 |= ((spo & 1) << 6);
	control->sspcr0 = sspcr0;
	return 0;
}

static int spi_set_data_frame_format(int spi_select, uint frf)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u16 sspcr0;

	frf &= 3;
	if (frf == 3) {
		return -1;
	}

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr0 = control->sspcr0;
	sspcr0 &= ~(3 << 4);
	sspcr0 |= (frf << 4);
	control->sspcr0 = sspcr0;
	return 0;
}

static int spi_set_data_frame_size(int spi_select, uint dss)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u16 sspcr0;

	spi = &spi_dev[spi_select];

	if ((dss < 3) || (dss > 15)) {
		return -1;
	}
	spi->data_frame_size = dss + 1;

	control = spi->control;
	sspcr0 = control->sspcr0 & ~0xf;
	sspcr0 |= dss;
	control->sspcr0 = sspcr0;

	spi_init_dma_channel(spi);
	return 0;
}

static int spi_set_slave_output_disable(int spi_select, uint sod)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u8 sspcr1;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr1 = control->sspcr1;
	sspcr1 &= ~(1 << 3);
	sspcr1 |= ((sod & 1) << 3);
	control->sspcr1 = sspcr1;
	return 0;
}

static int spi_set_master_slave_mode_select(int spi_select, uint ms)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u8 sspcr1;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr1 = control->sspcr1;
	sspcr1 &= ~(1 << 2);
	sspcr1 |= ((ms & 1) << 2);
	control->sspcr1 = sspcr1;
	return 0;
}

static int spi_set_ssp_enable(int spi_select, uint sse)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u8 sspcr1;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr1 = control->sspcr1;
	sspcr1 &= ~(1 << 1);
	sspcr1 |= ((sse & 1) << 1);
	control->sspcr1 = sspcr1;
	return 0;
}

static int spi_set_loop_back_mode(int spi_select, uint lbm)
{
	bcm476x_spi_dev_t *spi;
	volatile bcm476x_spi_control_regs_t *control;
	u8 sspcr1;

	spi = &spi_dev[spi_select];
	control = spi->control;
	sspcr1 = control->sspcr1;
	sspcr1 &= ~1;
	sspcr1 |= (lbm & 1);
	control->sspcr1 = sspcr1;
	return 0;
}

void bcm476x_spi_register_callback(int spi_select, spi_callback_t spi_callback)
{
	bcm476x_spi_dev_t *spi = &spi_dev[spi_select];
	spi->spi_callback = spi_callback;
}

void bcm476x_spi_unregister_callback(int spi_select)
{
	bcm476x_spi_dev_t *spi = &spi_dev[spi_select];
	spi->spi_callback = NULL;
}

int bcm476x_set_spi_enable(int spi_select, int flag)
{
/*	tahoe_opmode_corectrl_t	core_info =	{ 0	};
	tahoe_stat_t stat;
	char stat_str[256];

	if (spi_select >= ARRAYSIZE(spi_dev)) {
		return -1;
	}

	if (spi_select == 0) {
		core_info.core = CORE_SPI0;
	} else {
		core_info.core = CORE_SPI1;
	}

	core_info.stat = (flag) ? enable : disable;

	stat = tahoe_opmode_config_core(&core_info,	1);
	if(	stat !=	OpOk ) {
		tahoe_get_stat_str(stat, stat_str, sizeof(stat_str));
		printk(KERN_ERR	"Error:	%s(): tahoe_opmode_config_core failed: %s\n", __FUNCTION__,	stat_str);
		return -1;
	}
	*/
	return 0;
}

int bcm476x_spi_open(int spi_select, spi_control_t *spi_ctrl)
{
	bcm476x_spi_dev_t *spi;
	int ret = 0;

	if (spi_select >= sizeof(spi_dev)/sizeof(bcm476x_spi_dev_t)) {
		return -ENODEV;
	}

	spi = &spi_dev[spi_select];

	down_interruptible(&spi->lock);

	if (spi->in_use) {
		ret = -EBUSY;
		goto done;
	}

	if (!spi->const_0.virt) {
		spi->const_0.virt = dma_alloc_coherent(NULL,
					sizeof(u16),
					(dma_addr_t *)&spi->const_0.phys,
					GFP_KERNEL | GFP_DMA);
		if (!spi->const_0.virt) {
			ret = -ENOMEM;
			goto done;
		}
	}
	*(u16 *)spi->const_0.virt = 0;
	spi->control = (volatile bcm476x_spi_control_regs_t *)(ioremap_nocache(spi_dev[spi_select].addrbase, 0x28));
	spi->in_use = 1;
	spi->spi_callback = NULL;
	spi->cmd_length = 0;

	spi_set_config(spi_select, spi_ctrl);
	spi_init_dma_channel(spi);

done:
	up(&spi->lock);

	return ret;
}

void bcm476x_spi_close(int spi_select)
{
	bcm476x_spi_dev_t *spi;

	if (spi_select >= sizeof(spi_dev)/sizeof(bcm476x_spi_dev_t)) {
		return;
	}

	spi = &spi_dev[spi_select];

	down_interruptible(&spi->lock);

	if (spi->const_0.virt) {
		dma_free_coherent(NULL, sizeof(u16), spi->const_0.virt, spi->const_0.phys);
		spi->const_0.virt = NULL;
	}
	spi->spi_callback = NULL;
	spi->in_use = 0;
	iounmap((void *)(spi->control));

	up(&spi->lock);
}

u32 spi_default_callback(spi_addr_t **cmd_buf, uint *cmd_length)
{
	return 0;
}

static int spi_dev_open(struct inode * inode, struct file * file)
{
	int spi_minor;
	spi_control_t spi_ctrl;
	bcm476x_spi_dev_t *spi;
	int ret;

	spi_minor = MINOR(file->f_dentry->d_inode->i_rdev);

	// Master, Motorola format, phase/polarity (0,0), 16-bit data
	spi_ctrl.ctrl_reg = 0;
	spi_ctrl.bits.frame_format = SPI_SSPCR0_FRF_MOT;
	spi_ctrl.bits.clk_phase = 0;
	spi_ctrl.bits.clk_polarity = 0;
	spi_ctrl.bits.data_size = SPI_SSPCR0_DSS_8;	// default to use 8-bit data frame
//	spi_ctrl.bits.data_size = SPI_SSPCR0_DSS_16;
	spi_ctrl.bits.clk_rate = (2&SPI_SSPCR0_SCR_MASK);
	spi_ctrl.bits.cpsdvsr = (2&SPI_SSPCPSR_MASK);

	ret = bcm476x_spi_open(spi_minor, &spi_ctrl);
	if (ret < 0) {
		return ret;
	}

	spi = &spi_dev[spi_minor];

	spi->cmd_buf = &spi->cmd_tmp_buf;
	spi->data_buf = &spi->data_tmp_buf;

	spi->data_buf->virt = dma_alloc_coherent(NULL,
		SPI_DATA_BUF_SIZE,
		(dma_addr_t *)&spi->data_buf->phys,
		GFP_KERNEL | GFP_DMA);

	if (!spi->data_buf->virt) {
		goto error;
	}

	spi->cmd_buf->virt = dma_alloc_coherent(NULL,
		SPI_CMD_BUF_SIZE,
		(dma_addr_t *)&spi->cmd_buf->phys,
		GFP_KERNEL | GFP_DMA);

	if (!spi->cmd_buf->virt) {
		goto error;
	}

	return 0;

error:
	if (spi->data_buf->virt) {
		dma_free_coherent(NULL, SPI_DATA_BUF_SIZE, spi->data_buf->virt, spi->data_buf->phys);
		spi->data_buf->virt = NULL;
	}

	if (spi->cmd_buf->virt) {
		dma_free_coherent(NULL, SPI_CMD_BUF_SIZE, spi->cmd_buf->virt, spi->cmd_buf->phys);
		spi->cmd_buf->virt = NULL;
	}

	bcm476x_spi_close(spi_minor);
	return -1;
}

static int spi_dev_release(struct inode *inode, struct file * file)
{
	int spi_minor;
	bcm476x_spi_dev_t *spi;

	spi_minor = MINOR(file->f_dentry->d_inode->i_rdev);
	spi = &spi_dev[spi_minor];

	if (spi->data_buf->virt) {
		dma_free_coherent(NULL, SPI_DATA_BUF_SIZE, spi->data_buf->virt, spi->data_buf->phys);
		spi->data_buf->virt = NULL;
	}

	if (spi->cmd_buf->virt) {
		dma_free_coherent(NULL, SPI_CMD_BUF_SIZE, spi->cmd_buf->virt, spi->cmd_buf->phys);
		spi->cmd_buf->virt = NULL;
	}

	bcm476x_spi_close(spi_minor);
	return 0;
}

static ssize_t spi_dev_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	int spi_minor;
	bcm476x_spi_dev_t *spi;
	int cnt;

	spi_minor = MINOR(file->f_dentry->d_inode->i_rdev);
	spi = &spi_dev[spi_minor];

	cnt = (count <= SPI_DATA_BUF_SIZE) ? count : SPI_DATA_BUF_SIZE;

	if (spi->data_frame_size > 8) {
		// in 16-bit words
		cnt = (cnt + 1) >> 1;
	}
	cnt = bcm476x_spi_read(spi_minor, spi->data_buf, cnt);
	if (spi->data_frame_size > 8) {
		// in bytes
		cnt <<= 1;
	}

	cnt = (cnt <= count) ? cnt : count;

	if (access_ok(VERIFY_WRITE, buf, cnt)) {
		copy_to_user(buf, spi->data_buf->virt, cnt);
	} else {
		cnt = -EFAULT;
	}

	bcm476x_spi_unregister_callback(spi_minor);
	spi->cmd_length = 0;

	return cnt;
}

static ssize_t spi_dev_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int spi_minor;
	bcm476x_spi_dev_t *spi;
	int cnt;

	spi_minor = MINOR(file->f_dentry->d_inode->i_rdev);
	spi = &spi_dev[spi_minor];

	cnt = (count <= SPI_DATA_BUF_SIZE) ? count : SPI_DATA_BUF_SIZE;

	if (access_ok(VERIFY_READ, (void __user *)buf, cnt)) {
		copy_from_user(spi->data_buf->virt, buf, cnt);
	} else {
		cnt = -EFAULT;
		goto error;
	}

	if (spi->data_frame_size > 8) {
		// in 16-bit words
		cnt = (cnt + 1) >> 1;
	}
	cnt = bcm476x_spi_write(spi_minor, spi->data_buf, cnt);
	if (spi->data_frame_size > 8) {
		// in bytes
		cnt <<= 1;
	}

error:
	bcm476x_spi_unregister_callback(spi_minor);
	spi->cmd_length = 0;

	return cnt;
}

static int spi_dev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int spi_minor;
	bcm476x_spi_dev_t *spi;
	int ret = 0;
	uint length;

	spi_minor = MINOR(file->f_dentry->d_inode->i_rdev);

	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) >= SPI_IOC_MAXNR) return -ENOTTY;

	spi = &spi_dev[spi_minor];

	switch (cmd) {
		case SPI_SET_CMD_LENGTH:
			length = (uint)arg;
			if (length > SPI_CMD_BUF_SIZE) {
				ret = -EINVAL;
				break;
			}
			if (spi->data_frame_size > 8) {
				// in 16-bit words
				length = (length + 1) >> 1;
			}
			spi->cmd_length = length;
			break;

		case SPI_SET_CMD:
			length = spi->cmd_length;
			if (spi->data_frame_size > 8) {
				length <<= 1;
			}
			// in bytes
			if (access_ok(VERIFY_READ, (void __user *)arg, length)) {
				copy_from_user(spi->cmd_buf->virt, (const void __user *)arg, length);
				bcm476x_spi_register_callback(spi_minor, spi_default_callback);
			} else {
				ret = -EFAULT;
				spi->cmd_length = 0;
			}
			break;

		case SPI_SET_CLK_PRESCALER:
			if (spi_set_clk_prescaler(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_CLK_RATE:
			if (spi_set_clk_rate(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_CLK_PHASE:
			if (spi_set_clk_phase(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_CLK_POLARITY:
			if (spi_set_clk_polarity(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_DATA_FRAME_FORMAT:
			if (spi_set_data_frame_format(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_DATA_FRAME_SIZE:
			if (spi_set_data_frame_size(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_SLAVE_OUTPUT_DISABLE:
			if (spi_set_slave_output_disable(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_MASTER_SLAVE_MODE_SELECT:
			if (spi_set_master_slave_mode_select(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_SSP_ENABLE:
			if (spi_set_ssp_enable(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		case SPI_SET_LOOP_BACK_MODE:
			if (spi_set_loop_back_mode(spi_minor, (uint)arg) < 0) {
				ret = -EINVAL;
			}
			break;

		default:
			break;
	}

	return ret;
}

static struct file_operations spi_fops =
{
	.owner = THIS_MODULE,
	.open = &spi_dev_open,
	.release = &spi_dev_release,
	.read = &spi_dev_read,
	.write = &spi_dev_write,
	.ioctl = &spi_dev_ioctl
};

static int __init bcm476x_spi_init(void)
{
	int i;
	cdev_init(&spi_cdev, &spi_fops);
	spi_cdev.owner = THIS_MODULE;
	spi_cdev.ops = &spi_fops;

	cdev_add(&spi_cdev, MKDEV(SPI_MAJOR, SPI_MINOR), ARRAYSIZE(spi_dev));
    for (i = 0; i < ARRAYSIZE(spi_dev); i++) {
		sema_init(&spi_dev[i].lock, 1);
	}


    printk ("SPI: BCM476X spi driver\n");
	return 0;
}

static void __exit bcm476x_spi_exit(void)
{
	cdev_del(&spi_cdev);
}

module_init(bcm476x_spi_init);
module_exit(bcm476x_spi_exit);

EXPORT_SYMBOL(bcm476x_spi_open);
EXPORT_SYMBOL(bcm476x_spi_close);
EXPORT_SYMBOL(bcm476x_spi_read);
EXPORT_SYMBOL(bcm476x_spi_write);
EXPORT_SYMBOL(bcm476x_spi_register_callback);
EXPORT_SYMBOL(bcm476x_spi_unregister_callback);
EXPORT_SYMBOL(bcm476x_set_spi_enable);

MODULE_LICENSE("GPL");
