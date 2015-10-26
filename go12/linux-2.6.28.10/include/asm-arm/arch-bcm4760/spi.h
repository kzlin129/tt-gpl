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
/*
 *  linux/include/asm-arm/arch-bcm47xx/spi.h
 *
 *  SPI header file.
 */

#ifndef _SPI_H_
#define _SPI_H_

#include <linux/types.h>

#define SPI_FIFO_DEPTH  8

/* SSPCR0 bit shift   */
#define SPI_SSPCR0_DSS_SHIFT    0
#define SPI_SSPCR0_FRF_SHIFT    4
#define SPI_SSPCR0_SPO_SHIFT    6
#define SPI_SSPCR0_SPH_SHIFT    7
#define SPI_SSPCR0_SCR_SHIFT    8

/* SSPCR0 */
#define SPI_SSPCR0_SPH          (1 << SPI_SSPCR0_SPH_SHIFT) /* Motorola SPI phase selector */
#define SPI_SSPCR0_SPO          (1 << SPI_SSPCR0_SPO_SHIFT) /* Motorola SPI clock polarity selector */
#define SPI_SSPCR0_FRF_MOT      (0 << SPI_SSPCR0_FRF_SHIFT) /* Frame format: Motorola */
#define SPI_SSPCR0_FRF_TI       (1 << SPI_SSPCR0_FRF_SHIFT) /* Frame format: TI */
#define SPI_SSPCR0_FRF_NMW      (2 << SPI_SSPCR0_FRF_SHIFT) /* Frame format: National Microwire */
#define SPI_SSPCR0_FRF_RSRV     (3 << SPI_SSPCR0_FRF_SHIFT) /* Frame format: reserved */
#define SPI_SSPCR0_DSS_16       (0xf) /* 16-bit data */
#define SPI_SSPCR0_DSS_8        (0x7) /* 8-bit data */
#define SPI_SSPCR0_SCR_MASK     (0xff)
#define SPI_SSPCPSR_MASK        (0xff)

/* SSPCR1 */
#define SPI_SSPSCR1_SOD         (0x08)    /* Slave output disable                */
#define SPI_SSPSCR1_MS          (0x04)    /* Master/Slave 1:slave 0:master        */
#define SPI_SSPSCR1_SSE         (0x02)    /* Interafce Enable                */
#define SPI_SSPSCR1_LBM         (0x01)    /* Loopback select: 1:loopback, 0:normal    */

/* SSPSR status register    */
#define SPI_SSPSR_BSY           (0x10)     /* Busy            */
#define SPI_SSPSR_RFF           (0x08)     /* Rx FIFO 1: full     */
#define SPI_SSPSR_RNE           (0x04)     /* Rx FIFO 1: not empty    */
#define SPI_SSPSR_TNF           (0x02)     /* Tx FIFO 1: not full    */
#define SPI_SSPSR_TFE           (0x01)     /* Tx FIFO 1: not empty    */
#define SPI_SSPSR_BSY           (0x10)     /* Busy            */


typedef union spi_control_s {
    struct {
    // control register 0
        u16 data_size:4;    /* 0xF = 16-bit data; 0x7 = 8-bit data; */
        u16 frame_format:2; /* 00 = MOT; 01 = TI; 10 = NMW; 11 = reserved; */
        u16 clk_polarity:1; /* Motorola SPI frame format only */
        u16 clk_phase:1;    /* Motorola SPI frame format only */
        u16 clk_rate:8;     /* aka scr = 0 ~ 255 */
    // control register 1
        u8 loopback_mode:1; /* 0 = normal SSP operation; 1 = internal loopback; */
        u8 spi_enable:1;            /* 0 = SSP disabled; 1 = SSP enabled; */
        u8 slave_mode:1;            /* 0 = master; 1 = slave; */
        u8 slave_output_disable:1;  /* 1 = disable slave output */
        u8 reserved:4;
    // clock prescale register
        u8 cpsdvsr; /* even value from 2 ~ 254; bit rate=Fsspclk/(cpsdvsr*(1+scr)) */
    } bits;

    struct {
        u16 cr0;
        u8 cr1;
        u8 cpsdvsr;
    } regs;

    u32 ctrl_reg;
} spi_control_t;

typedef struct spi_addr_s {
    void *virt;
    dma_addr_t phys;
} spi_addr_t;


struct bcm476x_spi_master {
        u16 num_chipselect;
};

struct bcm476x_spi_device {
    int enable_dma;
    int bits_per_word;
};

typedef u32 (*spi_callback_t)(spi_addr_t **cmd_buf, uint *cmd_length);

/*
 * The bcm476x_spi_open function initializes the selected SPI port
 * and returns 0 for success or the corresponding error code.
 * Currently there are two SPI ports and thus spi_select can be 0 or 1.
 * This is the first function to call.
 */
extern int bcm476x_spi_open(int spi_select, spi_control_t *spi_ctrl);

/*
 * The bcm476x_spi_close function releases the selected SPI port.
 */
extern void bcm476x_spi_close(int spi_select);

/*
 * The bcm476x_spi_set_config function allows user to change SPI port configuration.
 * SPI FIFO is 16-bit wide and thus the driver is written in 16-bit fashion.
 */
extern void bcm476x_spi_set_config(int spi_select, spi_control_t *spi_ctrl);

/*
 * The bcm476x_spi_register_callback function allows user to register a callback
 * function if needed. It's usually called after bcm47xx_spi_open.
 * The callback function returns both the virtual and physical address to
 * command buffer, as well as its length to SPI device driver.
 * SPI driver sends the data in command buffer first and the data received
 * during this time is stored back to the command buffer.
 */
extern void bcm476x_spi_register_callback(int spi_select, spi_callback_t spi_callback);

/*
 * The spi_unregister_callback function allows user to unregister the callback
 * function previously registered.
 */
extern void bcm476x_spi_unregister_callback(int spi_select);

/*
 * The bcm476x_set_spi_enable function enables or disables a SPI port.
 *
 * Input:
 * spi_select - 0 for SPI port 0; 1 for SPI port 1.
 * flag - 0: disable; 1: enable
 *
 * Output: error code.
 */
extern int bcm476x_set_spi_enable(int spi_select, int flag);

/*
 * The sbcm476x_spi_read function reads data from the selected SPI port and
 * returns the amount of data read or error code.
 *
 * Input:
 * spi_select - 0 for SPI port 0; 1 for SPI port 1.
 * rx_buf - provides both the virtual and physical addresses to a DMA-safe
 *          receive buffer for SPI driver to write to.
 * length - the number of u16 data to receive. If the sum of command length
 *          and data length is more than SPI_FIFO_DEPTH(8), SPI driver uses
 *          DMA for SPI operations.
 *
 * Output: The amount of data received or error code.
 */
extern int bcm476x_spi_read(int spi_select, spi_addr_t *rx_buf, int length);

/*
 * The bcm476x_spi_write function writes data to the selected SPI port and
 * returns the amount of data written or error code.
 *
 * Input:
 * spi_select - 0 for SPI port 0; 1 for SPI port 1.
 * rx_buf - provides both the virtual and physical addresses to a DMA-safe
 *          transmit buffer for SPI driver to read from.
 * length - the number of u16 data to transmit. If the sum of command length
 *          and data length is more than SPI_FIFO_DEPTH(8), SPI driver uses
 *          DMA for SPI operations.
 *
 * Output: The amount of data transmit or error code.
 */
extern int bcm476x_spi_write(int spi_select, spi_addr_t *tx_buf, int length);

#endif
