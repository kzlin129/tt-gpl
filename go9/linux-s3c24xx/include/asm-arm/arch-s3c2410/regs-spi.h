/* linux/include/asm-arm/arch-s3c2410/regs-spi.h
 *
 * Copyright (c) 2004 Fetron GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 SPI register definition
 *
 *  Changelog:
 *    20-04-2004     KF      Created file
 *    04-10-2004     BJD     Removed VA address (no longer mapped)
 *			     tidied file for submission
 *    03-04-2005     LCVR    Added S3C2400_SPPIN_nCS definition
 *    02-11-2006     MJB     Added reg macros
 */

#ifndef __ASM_ARCH_REGS_SPI_H
#define __ASM_ARCH_REGS_SPI_H

#define S3C24XX_VA_SPI0      (S3C24XX_VA_SPI)
#define S3C24XX_VA_SPI1      (S3C24XX_VA_SPI + 0x0100 )

#define S3C2410_SPIREG0(x) ((x) + S3C24XX_VA_SPI0)
#define S3C2410_SPIREG1(x) ((x) + S3C24XX_VA_SPI1)

#define S3C2410_SPCON	              (0x00)
#define S3C2410_SPCON_RXFIFORB_14   (3<<14)	/* RX FIFO - Almost full control - 14 byte */
#define S3C2410_SPCON_RXFIFORB_12   (2<<14)	/* RX FIFO - Almost full control - 12 byte */
#define S3C2410_SPCON_RXFIFORB_4    (1<<14)	/* RX FIFO - Almost full control - 4 byte */
#define S3C2410_SPCON_RXFIFORB_2    (0<<14)	/* RX FIFO - Almost full control - 2 byte */
#define S3C2410_SPCON_TXFIFORB_14   (3<<12)	/* TX FIFO - Almost empty control - 14 byte */
#define S3C2410_SPCON_TXFIFORB_12   (2<<12)	/* TX FIFO - Almost empty control - 12 byte */
#define S3C2410_SPCON_TXFIFORB_4    (1<<12)	/* TX FIFO - Almost empty control - 4 byte */
#define S3C2410_SPCON_TXFIFORB_2    (0<<12)	/* TX FIFO - Almost empty control - 2 byte */
#define S3C2410_SPCON_RXFIFOEN      (1<<9)	/* RX FIFO Enable */
#define S3C2410_SPCON_TXFIFOEN      (1<<8)	/* TX FIFO Enable */
#define S3C2410_SPCON_SMOD_DMA      (2<<5)	/* DMA mode */
#define S3C2410_SPCON_SMOD_INT      (1<<5)	/* interrupt mode */
#define S3C2410_SPCON_SMOD_POLL     (0<<5)	/* polling mode */
#define S3C2410_SPCON_ENSCK         (1<<4)	/* Enable SCK */
#define S3C2410_SPCON_MSTR          (1<<3)	/* Master/Slave select 0: slave, 1: master */
#define S3C2410_SPCON_CPOL_HIGH     (1<<2)	/* Clock polarity select */
#define S3C2410_SPCON_CPOL_LOW      (0<<2)	/* Clock polarity select */
#define S3C2410_SPCON_CPHA_FMTB     (1<<1)	/* Clock Phase Select */
#define S3C2410_SPCON_CPHA_FMTA     (0<<1)	/* Clock Phase Select */
#define S3C2410_SPCON_TAGD          (1<<0)	/* Tx auto garbage data mode */

#define S3C2410_SPSTA               (0x04)
#define S3C2410_SPSTA_RXFWAV_MASK   (31<<24)	/* RX FIFO read available byte count */
#define S3C2410_SPSTA_TXFWAV_MASK   (31<<16)	/* TX FIFO write available byte count */
#define S3C2410_SPSTA_TBTISTS       (1<<12)		/* RX FIFO trailing bytes timeout */
#define S3C2410_SPSTA_RXFIFOAF      (1<<11)		/* RX FIFO almost full */
#define S3C2410_SPSTA_TXFIFOAE      (1<<10)		/* TX FIFO almost empty */
#define S3C2410_SPSTA_RXFIFOFERR    (1<<9)		/* RX FIFO full error  */
#define S3C2410_SPSTA_TXFIFOEERR    (1<<8)		/* TX FIFO empty error */
#define S3C2410_SPSTA_RXFIFOFULL    (1<<7)		/* RX FIFO full */
#define S3C2410_SPSTA_RXFIFONEMPTY  (1<<6)		/* RX FIFO not empty */
#define S3C2410_SPSTA_TXFIFONFULL   (1<<5)		/* TX FIFO not full */
#define S3C2410_SPSTA_TXFIFOEMPTY   (1<<4)		/* TX FIFO empty */
#define S3C2410_SPSTA_READY_ORG     (1<<3)		/* Data Pre Tx/Rx ready */
#define S3C2410_SPSTA_DCOL          (1<<2)		/* Data Collision Error */
#define S3C2410_SPSTA_MULD          (1<<1)		/* Multi Master Error */
#define S3C2410_SPSTA_READY         (1<<0)		/* Data Tx/Rx ready */

#define S3C2410_SPPIN               (0x08)
#define S3C2410_SPPIN_ENMUL         (1<<2)	/* Multi Master Error detect */
#define S3C2410_SPPIN_RESERVED      (1<<1)
#define S3C2400_SPPIN_nCS           (1<<1)	/* SPI Card Select */
#define S3C2410_SPPIN_KEEP          (1<<0)	/* Master Out keep */

#define S3C2410_SPFIC               (0x24)
#define S3C2410_SPFIC_TOCIE         (1<<7)	/* RX FIFO timeout counter interrupt enable */
#define S3C2410_SPFIC_RXFIFONEMIE   (1<<6)	/* RX FIFO not empty interrupt enable */
#define S3C2410_SPFIC_RXFIFOAFIE    (1<<5)	/* RX FIFO almost full interrupt enable */
#define S3C2410_SPFIC_TXFIFOAEIE    (1<<4)	/* TX FIFO almost empty interrupt enable */
#define S3C2410_SPFIC_RXFIFOFEIE    (1<<3)	/* RX FIFO full error interrupt enable */
#define S3C2410_SPFIC_TXFIFOEEIE    (1<<2)	/* TX FIFO empty error interrupt enable */
#define S3C2410_SPFIC_RXFIFOFLIE    (1<<1)	/* RX FIFO full interrupt enable */
#define S3C2410_SPFIC_TXFIFOEMIE    (1<<0)	/* TX FIFO empty interrupt enable */

#define S3C2410_SPPRE               (0x0C)
#define S3C2410_SPTDAT              (0x10)
#define S3C2410_SPRDAT              (0x14)
#define S3C2410_SPTXFIFO            (0x18)
#define S3C2410_SPRXFIFO            (0x1c)
#define S3C2410_SPRDATB             (0x20)

#define S3C2410_SPTOV             (0x28)
#define S3C2410_SPTOV_TOV         (0x1e) /* RX FIFO Timeout counter value register */

#define S3C2410_SPCON0   S3C2410_SPIREG0(S3C2410_SPCON)
#define S3C2410_SPSTA0   S3C2410_SPIREG0(S3C2410_SPSTA)
#define S3C2410_SPPIN0   S3C2410_SPIREG0(S3C2410_SPPIN)
#define S3C2410_SPPRE0   S3C2410_SPIREG0(S3C2410_SPPRE)
#define S3C2410_SPTDAT0  S3C2410_SPIREG0(S3C2410_SPTDAT)
#define S3C2410_SPRDAT0  S3C2410_SPIREG0(S3C2410_SPRDAT)

#define S3C2410_SPCON1   S3C2410_SPIREG1(S3C2410_SPCON)
#define S3C2410_SPSTA1   S3C2410_SPIREG1(S3C2410_SPSTA)
#define S3C2410_SPPIN1   S3C2410_SPIREG1(S3C2410_SPPIN)
#define S3C2410_SPPRE1   S3C2410_SPIREG1(S3C2410_SPPRE)
#define S3C2410_SPTDAT1  S3C2410_SPIREG1(S3C2410_SPTDAT)
#define S3C2410_SPRDAT1  S3C2410_SPIREG1(S3C2410_SPRDAT)

#endif /* __ASM_ARCH_REGS_SPI_H */
