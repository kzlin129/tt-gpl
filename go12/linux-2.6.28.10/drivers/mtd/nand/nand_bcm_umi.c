/*
 * Copyright (c) 2005 Broadcom Corporation
 *
 *    This driver was based on the sharpl NAND driver written by
 *    Richard Purdie.  Their copyright is below.
 *    As such, this driver falls under the GPL license also below.
 *
 *  Copyright (C) 2004 Richard Purdie
 *
 *  Based on Sharp's NAND driver sharp_sl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand_ecc512.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/reg_nand.h>
#include <asm/arch/reg_umi.h>

#if !defined( CONFIG_ARCH_BCM116X )
#include <linux/broadcom/memory_settings.h>
#endif

#if defined( CONFIG_BCM116X_DMA ) || defined( CONFIG_ARCH_BCMRING )
#define USE_DMA 1
#include <asm/arch/dma.h>
#include <linux/dma-mapping.h>
#else
#define USE_DMA 0
#endif

static char gBanner[] __initdata = KERN_INFO "BCM UMI MTD NAND Driver: 1.00\n";

//  Controls whether we are using hardware ecc or software.
//
#ifdef CONFIG_MTD_NAND_BCM_UMI_HWECC
static int hardware_ecc = 1;
#else
static int hardware_ecc = 0;
#endif

//  Register offsets
//
#define REG_NAND_CMD_OFFSET     ( 0 )
#define REG_NAND_ADDR_OFFSET    ( 4 )
#define REG_NAND_DATA16_OFFSET  ( 8 )
#ifndef __ARMEB__
#define REG_NAND_DATA8_OFFSET   ( 8 )
#else
#define REG_NAND_DATA8_OFFSET   ( 8 + 1 )
#endif

#if USE_DMA
#if defined( CONFIG_BCM116X_DMA )

#define NAND_DMA_CHAN   3

// Global config is common for both directions.
#define DMA_CFG                                 \
    ( REG_DMA_CHAN_CFG_TC_INT_ENABLE            \
    | REG_DMA_CHAN_CFG_ERROR_INT_ENABLE         \
    | REG_DMA_CHAN_CFG_FLOW_CTL_MEM_TO_MEM_DMA  \
    | REG_DMA_CHAN_CFG_ENABLE)

// Common transfer widths in bits (typically 8, 16, or 32)
#define DMA_WIDTH(dstwidth, srcwidth)                        \
    ( REG_DMA_CHAN_CTL_DEST_WIDTH_##dstwidth       \
    | REG_DMA_CHAN_CTL_SRC_WIDTH_##srcwidth )

// Common burst sizes - typically 4
#define DMA_BURST(width)                        \
    ( REG_DMA_CHAN_CTL_DEST_BURST_SIZE_##width  \
    | REG_DMA_CHAN_CTL_SRC_BURST_SIZE_##width )

// DMA settings for copying from NAND to SDRAM
#define DMA_CTRL_NAND_TO_SDRAM(dstwidth, srcwidth, bytes)    \
    ( REG_DMA_CHAN_CTL_TC_INT_ENABLE            \
    | REG_DMA_CHAN_CTL_DEST_INCR                \
    | DMA_BURST(4)                              \
    | DMA_WIDTH(dstwidth, srcwidth)             \
    | ( bytes * 8 / srcwidth ))

// DMA settings for copying from SDRAM to NAND
#define DMA_CTRL_SDRAM_TO_NAND(dstwidth, srcwidth, bytes)    \
    ( REG_DMA_CHAN_CTL_TC_INT_ENABLE            \
    | REG_DMA_CHAN_CTL_SRC_INCR                 \
    | DMA_BURST(4)                              \
    | DMA_WIDTH(dstwidth, srcwidth)             \
    | ( bytes * 8 / srcwidth ))

#endif  // CONFIG_BCM116X_DMA
#endif  // USE_DMA

/****************************************************************************
*
*  nand_hw_eccoob
*
*   New oob placement block for use with hardware ecc generation.
*
***************************************************************************/

static struct nand_ecclayout nand_hw_eccoob_512 = {
	.eccbytes	= 3,
	.eccpos		= {6, 7, 8 },
    // Reserve 0/1 and 10/11 as BI indicators for 16-bit flash
    // Reserve 5 for 8-bit BI
    // 6/7/8 are for ecc so this is all that's left
	.oobfree	= {
        { .offset = 2,      .length = 3},
        { .offset = 9,      .length = 1},
        { .offset = 12,     .length = 4 }}
};

// We treat the OOB for a 2K page as if it were 4 512 byte oobs, except that the ECC offset if 8 rather than 6.

static struct nand_ecclayout nand_hw_eccoob_2048 = {
	.eccbytes	= 12,
	.eccpos		= {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58 },
    // Reserve 0/1 as BI indicators for 8/16-bit flash
    // 8/9/10 are for ecc so this is all that's left
	.oobfree	= {
        { .offset = 2,      .length = 6},
        { .offset = 11,     .length = 13 },
        { .offset = 27,     .length = 13 },
        { .offset = 43,     .length = 13 },
        { .offset = 59,     .length = 5 }}
};

// We treat the OOB for a 4K page as if it were 8 512 byte oobs, except that the ECC offset if 8 rather than 6.

static struct nand_ecclayout nand_hw_eccoob_4096 = {
	.eccbytes	= 24,
	.eccpos		= {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58, 72, 73, 74, 88, 89, 90, 104, 105, 106, 120, 121, 122 },
    // Reserve 0/1 as BI indicators for 8/16-bit flash
    // 8/9/10 are for ecc so this is all that's left
	.oobfree	= {
        { .offset = 2,      .length = 6},
        { .offset = 11,     .length = 13 },
        { .offset = 27,     .length = 13 },
        { .offset = 43,     .length = 13 },
        { .offset = 59,     .length = 13 },
        { .offset = 75,     .length = 13 },
        { .offset = 91,     .length = 13 },
        { .offset = 107,    .length = 13 }}
        //{ .offset = 123,    .length = 5 }}    It turns out nand_ecclayout only has space for 8 entries
};

/*
 * MTD structure for BCM UMI
 */
static struct mtd_info *board_mtd = NULL;
static void __iomem *bcm_umi_io_base;

// Preallocate a buffer to avoid having to do this every dma operation.
// This is the size of the preallocated coherent DMA buffer.
#if USE_DMA
#define DMA_MIN_BUFLEN          512
#define DMA_MAX_BUFLEN          PAGE_SIZE
#define USE_DIRECT_IO(len)      (((len) < DMA_MIN_BUFLEN ) || ((len) > DMA_MAX_BUFLEN ))
#if defined( CONFIG_ARCH_BCMRING )
/*
 * TODO: The current NAND data space goes from 0x80001900 to 0x80001FFF,
 * which is only 0x700 = 1792 bytes long. This is too small for 2K, 4K page
 * size NAND flash. Need to break the DMA down to multiple 1Ks.
 *
 * Need to make sure REG_NAND_DATA_PADDR + DMA_MAX_LEN < 0x80002000 
 */
#define DMA_MAX_LEN             1024
#endif
static void *virtPtr;
static dma_addr_t physPtr;
#else
#define DMA_MIN_BUFLEN          0
#define DMA_MAX_BUFLEN          0   // disable linux nand dma
#define USE_DIRECT_IO(len)      1
#endif

#if USE_DMA

#if defined( CONFIG_ARCH_BCMRING )

struct semaphore    gDmaDoneSem;

/****************************************************************************
*
*   Handler called when the DMA finishes.
*
***************************************************************************/

static void nand_dma_handler( DMA_Device_t dev, int reason, void *userData )
{
    up( &gDmaDoneSem );
}

#endif

/****************************************************************************
*
*   Function to perform DMA initialization
*
***************************************************************************/

static int nand_dma_init( void )
{
#if defined( CONFIG_BCM116X_DMA )

    dma_request_chan(NAND_DMA_CHAN, "nand");

#elif defined( CONFIG_ARCH_BCMRING )

    int     rc;

    sema_init( &gDmaDoneSem, 0 );

    if (( rc = dma_set_device_handler( DMA_DEVICE_NAND_MEM_TO_MEM, nand_dma_handler, NULL )) != 0 )
    {
        printk( KERN_ERR "dma_set_device_handler failed: %d\n", rc );
        return rc;
    }

#else
#   error   Unrecognized DMA platform
#endif

    virtPtr = dma_alloc_coherent( NULL, DMA_MAX_BUFLEN, &physPtr, GFP_KERNEL );
    if ( virtPtr == NULL )
    {
        printk("NAND - Failed to allocate memory for DMA buffer\n");
        return -ENOMEM;
    }

    return 0;
}

#ifdef MODULE
/****************************************************************************
*
*   Function to perform DMA termination
*
***************************************************************************/

static void nand_dma_term( void )
{
#ifdef CONFIG_BCM116X_DMA
   dma_free_chan(NAND_DMA_CHAN);
#elif defined( CONFIG_ARCH_BCMRING )
   // Nothing to do
#else
#   error   Unrecognized DMA platform
#endif


   if ( virtPtr != NULL )
   {
       dma_free_coherent( NULL, DMA_MAX_BUFLEN, virtPtr, physPtr );
   }

}
#endif  // MODULE

/****************************************************************************
*
*   Performs a read via DMA
*
***************************************************************************/

static void nand_dma_read( void *buf, int len )
{
#ifdef CONFIG_BCM116X_DMA
      int dmactrl;
      dmactrl = NAND_BUS_16BIT() ? DMA_CTRL_NAND_TO_SDRAM(32, 16, len) : DMA_CTRL_NAND_TO_SDRAM(32, 8, len);
      dma_init_chan(NAND_DMA_CHAN);
      dma_setup_chan(NAND_DMA_CHAN,
                     NAND_BUS_16BIT() ? REG_NAND_DATA16_PADDR : REG_NAND_DATA8_PADDR,
                     (int)physPtr,
                     0,
                     dmactrl,
                     DMA_CFG);
      dma_poll_chan(NAND_DMA_CHAN);
      if ( buf != NULL )
      {
          memcpy(buf, virtPtr, len);
      }
#elif defined( CONFIG_ARCH_BCMRING )
      int offset = 0;
      int tmp_len = 0;
      int len_left = len;
      DMA_Handle_t  hndl;

      if ( virtPtr == NULL )
      {
          panic( "nand_dma_read: virtPtr == NULL\n" );
      }
      if ( (void *)physPtr == NULL )
      {
          panic( "nand_dma_read: physPtr == NULL\n" );
      }
      if (( hndl = dma_request_channel( DMA_DEVICE_NAND_MEM_TO_MEM )) < 0 )
      {
          printk( KERN_ERR "nand_dma_read: unable to allocate dma channel: %d\n", (int)hndl );
          panic( "\n" );
      }
      
      while ( len_left > 0 )
      {
          if ( len_left > DMA_MAX_LEN )
          {
              tmp_len = DMA_MAX_LEN;
              len_left -= DMA_MAX_LEN;
          }
          else
          {
              tmp_len = len_left;
              len_left = 0;
          }
          
          dma_transfer_mem_to_mem( hndl, REG_NAND_DATA_PADDR, physPtr + offset, tmp_len );
          down( &gDmaDoneSem );

          offset += tmp_len;
      }

      dma_free_channel( hndl );

      if ( buf != NULL )
      {
          memcpy(buf, virtPtr, len);
      }
#else
#   error   Unrecognized DMA platform
#endif
}

/****************************************************************************
*
*   Performs a write via DMA
*
***************************************************************************/

static void nand_dma_write( const void *buf, int len )
{
#if defined( CONFIG_BCM116X_DMA )
      int dmactrl;
      memcpy(virtPtr, buf, len);
      dmactrl = NAND_BUS_16BIT() ? DMA_CTRL_SDRAM_TO_NAND(16, 32, len): DMA_CTRL_SDRAM_TO_NAND(8, 32, len);
      dma_init_chan(NAND_DMA_CHAN);
      dma_setup_chan(NAND_DMA_CHAN,
                     (int)physPtr,
                     NAND_BUS_16BIT() ? REG_NAND_DATA16_PADDR : REG_NAND_DATA8_PADDR,
                     0,
                     dmactrl,
                     DMA_CFG);
      dma_poll_chan(NAND_DMA_CHAN);

#elif defined( CONFIG_ARCH_BCMRING )
      int offset = 0;
      int tmp_len = 0;
      int len_left = len;
      DMA_Handle_t  hndl;

      if ( buf == NULL )
      {
          panic( "nand_dma_write: buf == NULL\n" );
      }
      if ( virtPtr == NULL )
      {
          panic( "nand_dma_write: virtPtr == NULL\n" );
      }
      if ( (void *)physPtr == NULL )
      {
          panic( "nand_dma_write: physPtr == NULL\n" );
      }
      memcpy( virtPtr, buf, len );

      if (( hndl = dma_request_channel( DMA_DEVICE_NAND_MEM_TO_MEM )) < 0 )
      {
          printk( KERN_ERR "nand_dma_write: unable to allocate dma channel: %d\n", (int)hndl );
          panic( "\n" );
      }

      while ( len_left > 0 )
      {
          if ( len_left > DMA_MAX_LEN )
          {
              tmp_len = DMA_MAX_LEN;
              len_left -= DMA_MAX_LEN;
          }
          else
          {
              tmp_len = len_left;
              len_left = 0;
          }

          dma_transfer_mem_to_mem( hndl, physPtr + offset, REG_NAND_DATA_PADDR, tmp_len );
          down( &gDmaDoneSem );

          offset += tmp_len;
      }

      dma_free_channel( hndl );
#else
#   error   Unrecognized DMA platform
#endif
}

#endif

/****************************************************************************
*
*  nand_dev_raedy
*
*   Routine to check if nand is ready.
*
***************************************************************************/
static int nand_dev_ready(struct mtd_info* mtd)
{
    return (REG_UMI_NAND_RCSR & REG_UMI_NAND_RCSR_RDY);
}

/****************************************************************************
*
*  bcm_umi_nand_inithw
*
*   This routine does the necessary hardware (board-specific)
*   initializations.  This includes setting up the timings, etc.
*
***************************************************************************/

int bcm_umi_nand_inithw( void )
{
   // Configure nand timing parameters
#if defined( CONFIG_ARCH_BCM116X )
   // waiten = 0
   // lowfreq = 0
   // memtype async r/w, no page mode = 000
   // pgsz = 000
   // tprc_twlc = 00000     Page read access cycle / Burst write latency (n+2 / n+1)
   // tbta = 001            Bus turnaround cycle (n)
   // twp = 00100           Write pulse width cycle (n+1)
   // twr = 00              Write recovery cycle (n+1)
   // tas = 00              Write address setup cycle (n+1)
   // toe = 00              Output enable delay cycle (n)
   // trc_tlc = 00011       Read access cycle / Burst read latency (n+2 / n+1)
   REG_UMI_FLASH0_TCR = 0x00012003;
#endif

   // Configure nand timing parameters
   REG_UMI_NAND_TCR &= ~0x7ffff;
#if defined( CONFIG_ARCH_BCM116X )
   // tbta = 001            Bus turnaround cycle (n)
   // twp = 00100           Write pulse width cycle (n+1)
   // twr = 00              Write recovery cycle (n+1)
   // tas = 00              Write address setup cycle (n+1)
   // toe = 00              Output enable delay cycle (n)
   // trc_tlc = 00011       Read access cycle / Burst read latency (n+2 / n+1)
   REG_UMI_NAND_TCR |=  0x72003;
#else
   REG_UMI_NAND_TCR |= HW_CFG_NAND_TCR;
#endif

   REG_UMI_NAND_TCR  |= REG_UMI_NAND_TCR_CS_SWCTRL;        // enable software control of CS
   REG_UMI_NAND_RCSR |= REG_UMI_NAND_RCSR_CS_ASSERTED;     // keep NAND chip select asserted

   if (NAND_BUS_16BIT())
   {
      REG_UMI_NAND_TCR |= REG_UMI_NAND_TCR_WORD16;
   }
   else
   {
      REG_UMI_NAND_TCR &= ~REG_UMI_NAND_TCR_WORD16;
   }
   REG_UMI_MMD_ICR |= REG_UMI_MMD_ICR_FLASH_WP;        //enable writes to flash

   writel( NAND_CMD_RESET, bcm_umi_io_base + REG_NAND_CMD_OFFSET );
   NAND_WAIT_DONE();

	return 0;
}

/****************************************************************************
*
*  bcm_umi_nand_hwcontrol
*
*   Used to turn latch the proper register for access.
*
***************************************************************************/

static void bcm_umi_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
   /* TODO: send command to hardware */
   struct nand_chip *chip = mtd->priv;
   if (ctrl & NAND_CTRL_CHANGE) {
      if (ctrl & NAND_CLE) {
         chip->IO_ADDR_W    = bcm_umi_io_base + REG_NAND_CMD_OFFSET;
         goto CMD;
      }
      if (ctrl & NAND_ALE) {
         chip->IO_ADDR_W    = bcm_umi_io_base + REG_NAND_ADDR_OFFSET;
         goto CMD;
      }
      chip->IO_ADDR_W    = bcm_umi_io_base +
         ((NAND_BUS_16BIT()) ? REG_NAND_DATA16_OFFSET : REG_NAND_DATA8_OFFSET);
   }

CMD:
   /* Send command to chip directly */
   if (cmd != NAND_CMD_NONE)
      writeb(cmd, chip->IO_ADDR_W);

}

/****************************************************************************
*
*  bcm_umi_nand_get_hw_ecc
*
*   Used to get the hardware ECC.
*
***************************************************************************/

static int bcm_umi_nand_get_hw_ecc(struct mtd_info *mtd,
				      const u_char *dat, u_char *ecc_code)
{
   unsigned long ecc = REG_UMI_NAND_ECC_DATA;
   ecc_code[2] = (ecc >> 16) & 0xff;
   ecc_code[1] = (ecc >>  8) & 0xff;
   ecc_code[0] = (ecc >>  0) & 0xff;

   return 0;
}

/****************************************************************************
*
*  bcm_umi_nand_enable_hwecc
*
*   Used to turn on hardware ECC.
*
***************************************************************************/

static void bcm_umi_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
   REG_UMI_NAND_ECC_CSR &= ~REG_UMI_NAND_ECC_CSR_ECC_ENABLE;   // disable and reset ECC
   REG_UMI_NAND_ECC_CSR |= REG_UMI_NAND_ECC_CSR_ECC_ENABLE;    // enable ECC
   REG_UMI_NAND_ECC_CSR &= ~REG_UMI_NAND_ECC_CSR_256BYTE;      // 512 byte page
}

/**
 * nand_write_buf - write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 8bit buswith
 */
static void bcm_umi_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      // Do it the old way if the buffer is small or too large. Probably quicker than starting and checking dma.
      int i;
      struct nand_chip *this = mtd->priv;

      for (i=0; i<len; i++)
      {
         writeb(buf[i], this->IO_ADDR_W);
      }
   }
#if USE_DMA
   else
   {
      nand_dma_write( buf, len );
   }
#endif
}

/**
 * nand_read_buf - read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 */

static void bcm_umi_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      int i;
      struct nand_chip *this = mtd->priv;

      for (i=0; i<len; i++)
      {
         buf[i] = readb(this->IO_ADDR_R);
      }
   }
#if USE_DMA
   else
   {
      nand_dma_read( buf, len );
   }
#endif
}

/**
 * bcm_umi_nand_verify_buf - Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int bcm_umi_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      int i;
      struct nand_chip *this = mtd->priv;
      for (i=0; i<len; i++)
      {
         if (buf[i] != readb(this->IO_ADDR_R))
         {
            return -EFAULT;
         }
      }
   }
#if USE_DMA
   else
   {
      nand_dma_read( NULL, len );
      if ( memcmp( buf, virtPtr, len ) != 0 )
      {
         return -EFAULT;
      }
   }
#endif
   return 0;
}

/**
 * bcm_umi_nand_write_buf16 - write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 16bit buswith
 */
static void bcm_umi_nand_write_buf16(struct mtd_info *mtd, const u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      int i;
      struct nand_chip *this = mtd->priv;
      u16 *p = (u16 *) buf;
      len >>= 1;

      for (i=0; i<len; i++)
         writew(p[i], this->IO_ADDR_W);
   }
#if USE_DMA
   else
   {
      nand_dma_write( buf, len );
   }
#endif
}

/**
 * bcm_umi_nand_read_buf16 - read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 16bit buswith
 */
static void bcm_umi_nand_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      int words;
      int i;
      struct nand_chip *this = mtd->priv;
      u16 *p = (u16 *) buf;

      words = (len + 1) / 2;   // e.g len may be odd. 1 byte must do 1 word read

      for (i=0; i<words; i++)
      {
         p[i] = readw(this->IO_ADDR_R);
      }
   }
#if USE_DMA
   else
   {
      nand_dma_read( buf, len );
   }
#endif
}

/**
 * bcm_umi_nand_verify_buf16 - Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 16bit buswith
 */
static int bcm_umi_nand_verify_buf16(struct mtd_info *mtd, const u_char *buf, int len)
{
   if ( USE_DIRECT_IO( len ))
   {
      int i;
      struct nand_chip *this = mtd->priv;
      u16 *p = (u16 *) buf;
      len >>= 1;

      for (i=0; i<len; i++)
         if (p[i] != readw(this->IO_ADDR_R))
            return -EFAULT;
   }
#if USE_DMA
   else
   {
      nand_dma_read( NULL, len );
      if ( memcmp( buf, virtPtr, len ) != 0 )
      {
         return -EFAULT;
      }
   }
#endif
   return 0;
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif


/****************************************************************************
*
*  bcm_umi_nand_init
*
***************************************************************************/

static int __init bcm_umi_nand_init(void)
{
   struct nand_chip *this;
   int err = 0;
   int busw;

   printk( gBanner );

	/* Allocate memory for MTD device structure and private data */
	board_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	if (!board_mtd) {
		printk (KERN_WARNING "Unable to allocate NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* map physical adress */
#ifdef CONFIG_ARCH_BCMRING
	bcm_umi_io_base = ioremap(MM_ADDR_IO_NAND, 0x1000);
#else
	bcm_umi_io_base = ioremap(0x08000000, 0x1000);
#endif

	if(!bcm_umi_io_base){
		printk("ioremap to access BCM UMI NAND chip failed\n");
		kfree(board_mtd);
		return -EIO;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&board_mtd[1]);

	/* Initialize structures */
	memset((char *) board_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	board_mtd->priv = this;

   /* Initialize the NAND hardware.  */
   if (bcm_umi_nand_inithw() < 0)
   {
      printk("BCM UMI NAND chip could not be initialized\n");
      iounmap(bcm_umi_io_base);
      kfree(board_mtd);
      return -EIO;
   }

   /* Set address of NAND IO lines */
   this->IO_ADDR_W    = bcm_umi_io_base + ((NAND_BUS_16BIT()) ? REG_NAND_DATA16_OFFSET : REG_NAND_DATA8_OFFSET);
   this->IO_ADDR_R    = bcm_umi_io_base + ((NAND_BUS_16BIT()) ? REG_NAND_DATA16_OFFSET : REG_NAND_DATA8_OFFSET);

   /* Set command delay time, see datasheet for correct value */
   this->chip_delay   = 0;
   /* Assign the device ready function, if available */
   this->dev_ready = nand_dev_ready;
   this->options = (NAND_BUS_16BIT()) ? NAND_BUSWIDTH_16 : 0;

   busw = this->options & NAND_BUSWIDTH_16;
   this->write_buf = busw ? bcm_umi_nand_write_buf16 : bcm_umi_nand_write_buf;
   this->read_buf = busw ? bcm_umi_nand_read_buf16 : bcm_umi_nand_read_buf;
   this->verify_buf = busw ? bcm_umi_nand_verify_buf16 : bcm_umi_nand_verify_buf;

   this->cmd_ctrl = bcm_umi_nand_hwcontrol;
   if (hardware_ecc) {
      this->ecc.mode = NAND_ECC_HW;
      this->ecc.size = 512;
      this->ecc.bytes = 3;
      this->ecc.correct = nand_correct_data512;
      this->ecc.calculate = bcm_umi_nand_get_hw_ecc;
      this->ecc.hwctl = bcm_umi_nand_enable_hwecc;
   } else {
      this->ecc.mode = NAND_ECC_SOFT;
   }

#if USE_DMA
   if (( err = nand_dma_init() ) != 0 )
   {
      return err;
   }
#endif

   /* Figure out the size of the device that we have. We need to do this to figure out which ECC
    * layout we'll be using.                                                                       */

   err = nand_scan_ident( board_mtd, 1 );
   if (err)
   {
      printk( KERN_ERR "nand_scan failed: %d\n", err );
      iounmap(bcm_umi_io_base);
      kfree(board_mtd);
      return err;
   }

   // Now that we know the nand size, we can setup the ECC layout

   if ( hardware_ecc ) 
   {
      switch ( board_mtd->writesize )  // writesize is the pagesize
      {
         case 4096:  this->ecc.layout = &nand_hw_eccoob_4096;    break;
         case 2048:  this->ecc.layout = &nand_hw_eccoob_2048;    break;
         case 512:   this->ecc.layout = &nand_hw_eccoob_512;     break;
         default:
         {
            printk( KERN_ERR "NAND - Unrecognized pagesize: %d\n", board_mtd->writesize );
            return -EINVAL;
         }
      }
   }

   /* Now finish off the scan, now that ecc.layout has been initialized. */

   err = nand_scan_tail( board_mtd );
   if (err)
   {
      printk( KERN_ERR "nand_scan failed: %d\n", err );
      iounmap(bcm_umi_io_base);
      kfree(board_mtd);
      return err;
   }

   /* Register the partitions */
{
   int nr_partitions;
   struct mtd_partition* partition_info;

   board_mtd->name = "bcm_umi-nand";
   nr_partitions = parse_mtd_partitions(board_mtd, part_probes, &partition_info, 0);

   if (nr_partitions <= 0)
   {
      printk("BCM UMI NAND: Too few partitions - %d\n", nr_partitions);
      iounmap(bcm_umi_io_base);
      kfree(board_mtd);
      return -EIO;
	}
   add_mtd_partitions( board_mtd, partition_info, nr_partitions );
}

   /* Return happy */
   return 0;
}

/****************************************************************************/

module_init(bcm_umi_nand_init);

#ifdef MODULE
/****************************************************************************
*
*  bcm_umi_nand_exit
*
***************************************************************************/
static void __exit bcm_umi_nand_exit(void)
{
#if USE_DMA
   nand_dma_term();
#endif

   /* Release resources, unregister device */
   nand_release(board_mtd);

   /* unmap physical adress */
   iounmap(bcm_umi_io_base);

   /* Free the MTD device structure */
   kfree(board_mtd);
}
module_exit(bcm_umi_nand_exit);
#endif


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM UMI MTD NAND driver");
