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
 *  linux/include/asm-arm/arch-bcm47xx/spi_ioctl.h
 *
 *  SPI driver IOCTL header file.
 */

#ifndef _SPI_IOCTL_H_
#define _SPI_IOCTL_H_

/* Use 'S' as magic number */
#define SPI_IOC_MAGIC  'S'
#define SPI_IOC_MAXNR	E_SPI_IOC_MAXNR

typedef enum E_SPI_IOCTL_OP_ID
{
	E_SPI_SET_CLK_PRESCALER = 0,
	E_SPI_SET_CLK_RATE,
	E_SPI_SET_CLK_PHASE,
	E_SPI_SET_CLK_POLARITY,
	E_SPI_SET_DATA_FRAME_FORMAT,
	E_SPI_SET_DATA_FRAME_SIZE,	// in bits; only support 8 and 16 bits
	E_SPI_SET_SLAVE_OUTPUT_DISABLE,
	E_SPI_SET_MASTER_SLAVE_MODE_SELECT,
	E_SPI_SET_SSP_ENABLE,
	E_SPI_SET_LOOP_BACK_MODE,
	E_SPI_SET_CMD_LENGTH,
	E_SPI_SET_CMD,
	E_SPI_IOC_MAXNR
} E_SPI_IOCTL_OP_ID;

// Refer to SPI section in datasheet
// bit rate = Fsspclk / (CPSDVR * (1 + SCR))

// Set CPSDVR (2 ~ 254 even value)
#define	SPI_SET_CLK_PRESCALER	_IO(SPI_IOC_MAGIC, E_SPI_SET_CLK_PRESCALER)

// Set SCR (0 ~ 255)
#define	SPI_SET_CLK_RATE	_IO(SPI_IOC_MAGIC, E_SPI_SET_CLK_RATE)

// applicable to Motorola SPI frame format only
#define	SPI_SET_CLK_PHASE	_IO(SPI_IOC_MAGIC, E_SPI_SET_CLK_PHASE)

// applicable to Motorola SPI frame format only
#define	SPI_SET_CLK_POLARITY	_IO(SPI_IOC_MAGIC, E_SPI_SET_CLK_POLARITY)

// 00 = Motorola SPI; 01 = TI sync serial; 10 = National Microwire; 11 = reserved
#define	SPI_SET_DATA_FRAME_FORMAT	_IO(SPI_IOC_MAGIC, E_SPI_SET_DATA_FRAME_FORMAT)

// Set the data frame size for the SPI port
// 0000 ~ 0010 = reserved; 0011 = 4-bit data; 0100 = 5-bit; ... 1111 = 16-bit
// e.g. ioctl(fd, SPI_SET_DATA_FRAME_SIZE, 7)	// 8-bit
// e.g. ioctl(fd, SPI_SET_DATA_FRAME_SIZE, 15)	// 16-bit
#define	SPI_SET_DATA_FRAME_SIZE	_IO(SPI_IOC_MAGIC, E_SPI_SET_DATA_FRAME_SIZE)

// 0 = can drive SSPTXD output in slave mode; 1 = must not drive output in slave mode
#define	SPI_SET_SLAVE_OUTPUT_DISABLE	_IO(SPI_IOC_MAGIC, E_SPI_SET_SLAVE_OUTPUT_DISABLE)

// 0 = device in master mode (default); 1 = device in slave mode
#define	SPI_SET_MASTER_SLAVE_MODE_SELECT	_IO(SPI_IOC_MAGIC, E_SPI_SET_MASTER_SLAVE_MODE_SELECT)

// 0 = SSP operation disabled; 1 = SSP operation enabled;
#define	SPI_SET_SSP_ENABLE	_IO(SPI_IOC_MAGIC, E_SPI_SET_SSP_ENABLE)

// 0 = normal serial port operation enabled; 1 = output connected to input internally
#define	SPI_SET_LOOP_BACK_MODE	_IO(SPI_IOC_MAGIC, E_SPI_SET_LOOP_BACK_MODE)

// Set the length (in byte) of command buffer before passing the actual command buffer.
// e.g. ioctl(fd, SPI_SET_CMD_LENGTH, cmd_length)
#define	SPI_SET_CMD_LENGTH	_IO(SPI_IOC_MAGIC, E_SPI_SET_CMD_LENGTH)

// Pass in the pointer to the command buffer. Command length needs to be set before hand.
// e.g. ioctl(fd, SPI_SET_CMD, cmd_buf);
#define	SPI_SET_CMD			_IO(SPI_IOC_MAGIC, E_SPI_SET_CMD)

#endif /* _SPI_IOCTL_H_ */
