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
//#include <linux/config.h>
#include <asm/arch/hw_cfg.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/mtd/compatmac.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/arch/spi.h>

//#if (1)
//#include <asm/arch/gpio.h>
//#endif

//#include "nvram.h"

#ifndef ARRAYSIZE
#define ARRAYSIZE(a)	(sizeof(a) / sizeof(a[0]))
#endif

/* these block of commands are exactly the same for ATMEL and ST flashes */
#define SFLASH_CMD_WRSR (0x01) // write status register
#define SFLASH_CMD_PROG (0x02) // page program
#define SFLASH_CMD_READ (0x03) // read data array
#define SFLASH_CMD_WRDI (0x04) // write disable
#define SFLASH_CMD_RDSR (0x05) // read status register
#define SFLASH_CMD_WREN (0x06) // write enable
#define SFLASH_CMD_FARD	(0x0B) // AT25FS040 fast read			 

#define SFLASH_CMD_SECTOR_ERASE_ST (0xD8)
#define SFLASH_CMD_CHIP_ERASE_ST   (0xC7)

// Cmd Read ID 
#define SFLASH_CMD_RDID_ST       (0xAB)   
#define SFLASH_CMD_RDID_ATMEL1   (0x15)
//AT25FS040 has two RDID opcodes (0x9F and 0xAB). Either will execute the instruction. 
#define SFLASH_CMD_RDID_ATMEL2   (0x9F)   

// Select ATMEL cmd set 1 or 2 based on ID.
// ATMEL 1. for AT25F1024,AT25F4096
#define SFLASH_CMD_SECTOR_ERASE1 (0x52)
#define SFLASH_CMD_CHIP_ERASE1   (0x62)

// ATMEL 2. for AT25FS040
// AT25FS040 has two opcodes for each of the following. Either will execute the instruction.
#define SFLASH_CMD_SECTOR_ERASE2 (0x20)	// 4k sector erase (0x20 or 0xD7)  
//#define SFLASH_CMD_BLOCK_ERASE2  (0x52)	// 64k block erase (0x52 or 0xD8)  
#define SFLASH_CMD_CHIP_ERASE2	(0xC7)	// chip erase (0x60 or 0xC7)  

// Status register bits 
// NOTE: they bear the same meaning    
// but different names for both     
// supported flash types - I use ATMEL  
#define SFLASH_STATUS_WPEN	(0x80) // write protect enable
#define SFLASH_STATUS_BP4	(0x40) // block protect 4	
#define SFLASH_STATUS_BP3	(0x20) // block protect 3	
#define SFLASH_STATUS_BP2	(0x10) // block protect 2	
#define SFLASH_STATUS_BP1	(0x08) // block protect 1
#define SFLASH_STATUS_BP0	(0x04) // block protect 0
#define SFLASH_STATUS_WEN	(0x02) // write enable
#define SFLASH_STATUS_WIP	(0x01) // write in progress

// Manufacturer ID followed by two device ID bytes where:
// For example AT25FS040, 3byte full ID=0x1F6604
// Manufacturer ID assigned by JEDEC is 0x1F for Atmel
// memory type=0x66=AT25FS040, memory capacity=0x04  
#define SFLASH_ID_AT25F1024			(0x1F60)
#define SFLASH_MAX_SIZE_AT25F1024	(SZ_1M >> 3)	// 1Mb for boot image

#define SFLASH_ID_AT25F4096			(0x1F64)
#define SFLASH_MAX_SIZE_AT25F4096	(SZ_4M >> 3)	// 1Mb for boot image

#define SFLASH_ID_AT25FS040	      (0x1F66)    
#define SFLASH_MAX_SIZE_AT25FS040	(SZ_4M >> 3)	// 1Mb for boot image

typedef struct {
	ulong	   id;			   // combined device & manufacturer code
	ulong	   sector_size;   // erase sector size
	ushort	sector_count;  // number of erase units
	ushort	page_size;	   // program page size
   u32      rd_id_cmd;     // Read ID cmd specific to device
} sflash_info_t;

static sflash_info_t sflash_info[] =
{
	{ SFLASH_ID_AT25F1024, SZ_16K*2, (SFLASH_MAX_SIZE_AT25F1024 / (SZ_16K*2)), 256, SFLASH_CMD_RDID_ATMEL1},
	{ SFLASH_ID_AT25F4096, SZ_64K,   (SFLASH_MAX_SIZE_AT25F4096 / SZ_64K),     256, SFLASH_CMD_RDID_ATMEL1},
	{ SFLASH_ID_AT25FS040, SZ_64K,   (SFLASH_MAX_SIZE_AT25FS040 / SZ_64K),     256, SFLASH_CMD_RDID_ATMEL2}
};

static u16 sflash_rd_id = 0;   // The Read ID. id=0 means not yet read or read but not recognised 


#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition bcm476x_mtd_sflash_parts_AT25F1024[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25F1024, },
	//{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25F1024   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};

static struct mtd_partition bcm476x_mtd_sflash_parts_AT25F4096[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25F4096, },
//	{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25F4096   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};

static struct mtd_partition bcm476x_mtd_sflash_parts_AT25FS040[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25FS040, },
//	{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25FS040   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};
/* OLD PARTITION INFORMATION CONTAINING NVFLASH_SECTOR_SIZE

static struct mtd_partition bcm476x_mtd_sflash_parts_AT25F1024[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25F1024 - NVFLASH_SECTOR_SIZE, },
	{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25F1024   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};

static struct mtd_partition bcm476x_mtd_sflash_parts_AT25F4096[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25F4096 - NVFLASH_SECTOR_SIZE, },
	{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25F4096   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};

static struct mtd_partition bcm476x_mtd_sflash_parts_AT25FS040[] =
{
	{ name: "serial-boot",  offset: 0x000000, size: SFLASH_MAX_SIZE_AT25FS040 - NVFLASH_SECTOR_SIZE, },
	{ name: "serial-nvram", offset: SFLASH_MAX_SIZE_AT25FS040   - NVFLASH_SECTOR_SIZE, size: NVFLASH_SECTOR_SIZE, },
	{ name: NULL, },
};
*/

static struct mtd_partition *bcm476x_mtd_sflash_parts_info[] =
{
	bcm476x_mtd_sflash_parts_AT25F1024, bcm476x_mtd_sflash_parts_AT25F4096, bcm476x_mtd_sflash_parts_AT25FS040
};

static struct mtd_partition *bcm476x_mtd_sflash_parts;

#endif /* CONFIG_MTD_PARTITIONS */

#define SFLASH_DMA_BUF_BYTE_SIZE	512
#define SFLASH_CMD_BUF_BYTE_SIZE	(sizeof(u16)*2)

typedef enum {
	STATUS_OK,
	STATUS_FAILED
} STATUS;

typedef struct  {
	struct semaphore lock;
	struct mtd_info mtd;
	struct mtd_erase_region_info region;
	u32 size;
	u32 pagesize;   // page program limit
	u32 blocksize;
	u32 numblocks;
	int spi_select;
	spi_addr_t data_buf;
	spi_addr_t cmd_buf;
	uint cmd_length;
} sflash_mtd;

/* Private global state */
static sflash_mtd sflash;

static u8 sflash_get_status(sflash_mtd *sflash);
static int sflash_is_ready(sflash_mtd *sflash);
static int sflash_enable_wr(sflash_mtd *sflash);
/* static int sflash_disable_wr(sflash_mtd *sflash); */
static int sflash_mtd_poll(sflash_mtd *sflash, unsigned long timeout);
static sflash_mtd * sflash_init(void);
static uint sflash_make_cmd(u32 cmd, u32 addr, u16 *cmd_buf);
static u32 sflash_callback(spi_addr_t **cmd_buf, uint *cmd_length);

/* Returns 0 for success or the corresponding error code. */
static int sflash_mtd_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	sflash_mtd *sflash = (sflash_mtd *)mtd->priv;
	int bytes, bytes_read, ret = 0;

	/* Check address range */
	if (!len) {
		return 0;
	}
	
	if ((from + len) > mtd->size) {
		return -EINVAL;
	}
	
	if (len & 1) {
		printk(KERN_ERR "alignment error\n");
		return -EINVAL;
	}
	
	down(&sflash->lock);

	while(!sflash_is_ready(sflash));

	*retlen = 0;
	while (len > 0) {
		sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_READ, from, (u16 *)sflash->cmd_buf.virt);
#if (defined CONFIG_BCM476X_SPI_DEBUG && CONFIG_BCM476X_SPI_DEBUG)
		bytes = SPI_FIFO_DEPTH - sflash->cmd_length;
#else
		bytes = SFLASH_DMA_BUF_BYTE_SIZE;
#endif
		if (len < bytes) {
			bytes = len;
		}
		if ((bytes_read = bcm476x_spi_read(sflash->spi_select, &sflash->data_buf, (bytes+1)/2)) < 0) {
			ret = bytes_read;
			break;
		}
		bytes_read *= 2;
		if (bytes_read < bytes) {
			bytes = bytes_read;
		}
		memcpy((void *)buf, (void *)sflash->data_buf.virt, bytes);

		from += (loff_t)bytes;
		len -= bytes;
		buf += bytes;
		*retlen += bytes;
	}

	up(&sflash->lock);

	return ret;
}

/* Returns 0 for success or the corresponding error code. */
static int sflash_mtd_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	sflash_mtd *sflash = (sflash_mtd *) mtd->priv;
	int bytes, bytes_written, ret = 0;

	/* Check address range */
	if (!len) {
		return 0;
	}
	
	if ((to + len) > mtd->size) {
		return -EINVAL;
	}
	
	if (len & 1) {
		printk(KERN_ERR "alignment error\n");
		return -EINVAL;     
	}
	
	down(&sflash->lock);

	*retlen = 0;
	while (len > 0) {
		while(!sflash_is_ready(sflash));
		sflash_enable_wr(sflash);
		sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_PROG, to, (u16 *)sflash->cmd_buf.virt);
#if (defined CONFIG_BCM476X_SPI_DEBUG && CONFIG_BCM476X_SPI_DEBUG)
		bytes = SPI_FIFO_DEPTH - sflash->cmd_length;
#else
		bytes = sflash->pagesize;
#endif
		if (len < bytes) {
			bytes = len;
		}
		memcpy((void *)sflash->data_buf.virt, (void *)buf, bytes);
		if ((bytes_written = bcm476x_spi_write(sflash->spi_select, &sflash->data_buf, ((bytes+1)/2))) < 0) {
			ret = bytes_written;
			break;
		}
		bytes_written *= 2;
		if (bytes_written < bytes) {
			bytes = bytes_written;
		}
		
		if ((ret = sflash_mtd_poll(sflash, HZ/10))) {
			break;
		}
		
		to += (loff_t) bytes;
		len -= bytes;
		buf += bytes;
		*retlen += bytes;
	}

	up(&sflash->lock);

	return ret;
}

/* Returns 0 for success or the corresponding error code. */
static int sflash_mtd_erase(struct mtd_info *mtd, struct erase_info *erase)
{
	sflash_mtd *sflash = (sflash_mtd *) mtd->priv;
	int i, j, ret = 0;
	unsigned int addr, len;
   u32 erase_cmd=0; 

   // Check address range 
	if (!erase->len)
		return 0;

	if ((erase->addr + erase->len) > mtd->size) {
		printk(KERN_ERR "sflash_mtd_erase: size problem \n");
		return -EINVAL;
	}

   // cmd select (probably better ways to do this but ok for now)
   switch (sflash_rd_id)
   {
      case 0: 
		   printk(KERN_ERR "sflash_mtd_erase: invalid flash id \n");
		   return -EINVAL;
      case SFLASH_ID_AT25F1024: 
      case SFLASH_ID_AT25F4096: 
         erase_cmd = SFLASH_CMD_SECTOR_ERASE1;
         break;
      case SFLASH_ID_AT25FS040: 
         erase_cmd = SFLASH_CMD_SECTOR_ERASE1;
         break;
      default: 
         erase_cmd = SFLASH_CMD_SECTOR_ERASE_ST;
         break; // ST
   }
   
	addr = erase->addr;
	len = erase->len;

	down(&sflash->lock);

	/* Ensure that requested region is aligned */
	for (i = 0; i < mtd->numeraseregions; i++) {
		for (j = 0; j < mtd->eraseregions[i].numblocks; j++) {
			if (addr == mtd->eraseregions[i].offset + mtd->eraseregions[i].erasesize * j &&
				len >= mtd->eraseregions[i].erasesize) {

				sflash_enable_wr(sflash);
             
				sflash->cmd_length = sflash_make_cmd(erase_cmd, addr, (u16 *)sflash->cmd_buf.virt);
				if ((ret = bcm476x_spi_write(sflash->spi_select, NULL, 0)) < 0) {
					printk(KERN_ERR "sflash erase failed: %d\n", ret);
					break;
				}
				if ((ret = sflash_mtd_poll(sflash, 10 * HZ))) {
					printk(KERN_ERR "sflash_mtd_poll failed: %d\n", ret);
					break;
				}

				addr += mtd->eraseregions[i].erasesize;
				len -= mtd->eraseregions[i].erasesize;
			}
		}
		if (ret)
			break;
	}

	up(&sflash->lock);

	/* Set erase status */
	if (ret) {
		erase->state = MTD_ERASE_FAILED;
	} else {
		erase->state = MTD_ERASE_DONE;
	}

	/* Call erase callback */
	if (erase->callback) {
		erase->callback(erase);
	}

	return ret;
}

static int sflash_mtd_poll(sflash_mtd *sflash, unsigned long timeout)
{
	unsigned long now = jiffies;
	int ret = 0;

	for (;;) {
		if (sflash_is_ready(sflash)) {
			ret = 0;
			break;
		}

		if (time_after(jiffies, now + timeout)) {
			printk(KERN_ERR "sflash: timeout\n");
			ret = -ETIMEDOUT;
			break;
		}

		if (need_resched()) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(timeout / 10);
		} else {
			udelay(1);
		}
	}

	return ret;
}


static u16 sflash_get_id(sflash_mtd *sflash)
{
	u16 id=0, *ptr;
	spi_addr_t data_buf;
    int atmel_count=0;

   printk(KERN_NOTICE "SFlash: get ID\n");
   
   // Check for ATMEL variants
   for ( ; atmel_count < ARRAYSIZE(sflash_info); atmel_count++)
   {
      u32 rd_id_cmd = sflash_info[atmel_count].rd_id_cmd;      
   
	   sflash->cmd_length = sflash_make_cmd(rd_id_cmd, 0, (u16 *)sflash->cmd_buf.virt);  
	   data_buf.virt = sflash->cmd_buf.virt + sizeof(u16);
	   data_buf.phys = sflash->cmd_buf.phys + sizeof(u16);
	   bcm476x_spi_read(sflash->spi_select, &data_buf, 1);
	   ptr = (u16 *)sflash->cmd_buf.virt;
	   id = (ptr[0]<<8)|(ptr[1]>>8);
   
      if ( (u16)sflash_info[atmel_count].id == id)
      {  // Got sflash id
         break;
      }
   } // for ATMEL
   
   if ( ARRAYSIZE(sflash_info) == atmel_count)
   {
      // Check for ST flash (AT25FS040 has two RDID opcodes (0x9F and 0xAB), where 0xAB is ame as ST RDID.    
	   sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_RDID_ST, 0, (u16 *)sflash->cmd_buf.virt);
	   data_buf.virt = sflash->cmd_buf.virt + sizeof(u16);
	   data_buf.phys = sflash->cmd_buf.phys + sizeof(u16);
	   bcm476x_spi_read(sflash->spi_select, &data_buf, 1);
	   ptr = (u16 *)sflash->cmd_buf.virt;
	   id = (ptr[0]<<8)|(ptr[1]>>8);

      if ( 0 != id) // TBD 0 or 0xFFFF for not pressent
      {
         id = 0;  // no match
         printk(KERN_NOTICE "SFlash: no ID\n");
      }
   }
   
	printk(KERN_NOTICE "SFlash: ID = 0x%04x\n", id);
   // Save as global sflash id
   sflash_rd_id = id;
	return id;
}



static u8 sflash_get_status(sflash_mtd *sflash)
{
	u16 *ptr;
	sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_RDSR, 0, (u16 *)sflash->cmd_buf.virt);
	bcm476x_spi_read(sflash->spi_select, NULL, 0);
	ptr = (u16 *)sflash->cmd_buf.virt;
	return (u8)ptr[0];
}

static int sflash_is_ready(sflash_mtd *sflash)
{
	return (sflash_get_status(sflash) & SFLASH_STATUS_WIP) ? 0:1;
}

static int sflash_enable_wr(sflash_mtd *sflash)
{
	while(!sflash_is_ready(sflash));
	sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_WREN, 0, (u16 *)sflash->cmd_buf.virt);
	bcm476x_spi_write(sflash->spi_select, NULL, 0);
	return 0;
}

/*
static int sflash_disable_wr(sflash_mtd *sflash)
{
	while(!sflash_is_ready(sflash));
	sflash->cmd_length = sflash_make_cmd(SFLASH_CMD_WRDI, 0, (u16 *)sflash->cmd_buf.virt);
	spi_write(sflash->spi_select, NULL, 0);
	return 0;
}
*/

static uint sflash_make_cmd(u32 cmd, u32 addr, u16 *cmd_buf)
{
#if (0)
   printk(KERN_NOTICE "SFlash: cmd=%0x\n", cmd);
#endif   
	switch (cmd) 
   {
	case SFLASH_CMD_PROG:
	case SFLASH_CMD_READ:
   case SFLASH_CMD_SECTOR_ERASE_ST:
	case SFLASH_CMD_SECTOR_ERASE1:
	case SFLASH_CMD_SECTOR_ERASE2:
		cmd_buf[0] = (u16)(((cmd&0xff)<<8)|((addr>>16)&0xFF));
		cmd_buf[1] = (u16)addr&0xffff;
		return 2;
	case SFLASH_CMD_RDSR:
		cmd_buf[0] = (u16)(((cmd&0xff)<<8)|(addr&0xFF));
		return 1;
	case SFLASH_CMD_WRSR:
	case SFLASH_CMD_WRDI:
	case SFLASH_CMD_WREN:
	case SFLASH_CMD_RDID_ST: 
	case SFLASH_CMD_RDID_ATMEL1:
   case SFLASH_CMD_RDID_ATMEL2:
		cmd_buf[0] = (u16)((cmd&0xff)<<8);
		return 1;
#if 0 // same as ST sector erase?
   case SFLASH_CMD_BLOCK_ERASE2:
      if (  0 != sflash_rd_id &&  SFLASH_ID_AT25FS040 == sflash_rd_id)
      {
		   cmd_buf[0] = (u16)(((cmd&0xff)<<8)|((addr>>16)&0xFF));
		   cmd_buf[1] = (u16)addr&0xffff;
		   return 2;
      }
      else
         return 0;
#endif
	default:
		return 0;
	}
}

static u32 sflash_callback(spi_addr_t **cmd_buf, uint *cmd_length)
{
	*cmd_buf = &sflash.cmd_buf;
	*cmd_length = sflash.cmd_length;
	return 0;
}

/* Initialize serial flash access */
static sflash_mtd *sflash_init(void)
{
	spi_control_t spi_ctrl;
	u16 id;
	int i, j;

#if (1)
      // YP-T11 SPI_NOR_MISO (output from sflash) needs pull-up. BCM2820 pin AE16, GPIO #59.
      // TBD. Don't hardwire it, use name from platform specific GPIO pin name "GPIO_SPI_NOR_MISO".
		//gpio_set_pull_up_down_enable(59, 1);
		//gpio_set_pull_up_down_is_up(59, 1);
      printk(KERN_NOTICE "SFlash: sflash_init\n");
#endif
   
   // Init global id var which will be setup by sflash_get_id(). 
   sflash_rd_id = 0;
   sflash.data_buf.virt = dma_alloc_coherent(NULL,
			SFLASH_DMA_BUF_BYTE_SIZE,
			(dma_addr_t *)&sflash.data_buf.phys,
			GFP_KERNEL | GFP_DMA);
	printk (KERN_NOTICE "dma_alloc_coherent returned %x", (unsigned int)sflash.data_buf.virt);
   if (!sflash.data_buf.virt) {
		goto error;
	}
   sflash.cmd_buf.virt = dma_alloc_coherent(NULL,
			SFLASH_CMD_BUF_BYTE_SIZE,
			(dma_addr_t *)&sflash.cmd_buf.phys,
			GFP_KERNEL | GFP_DMA);
	printk (KERN_NOTICE "dma_alloc_coherent returned %x", (unsigned int)sflash.cmd_buf.virt);
   	if (!sflash.cmd_buf.virt) {
		goto error;
	}
   	// Master, Motorola format, phase/polarity (0,0), 16-bit data
	spi_ctrl.ctrl_reg = 0;
   	spi_ctrl.bits.frame_format = SPI_SSPCR0_FRF_MOT;
   	spi_ctrl.bits.clk_phase = 0;
   	spi_ctrl.bits.clk_polarity = 0;
   	spi_ctrl.bits.data_size = SPI_SSPCR0_DSS_16;
	spi_ctrl.bits.clk_rate = HW_SPI_SERIAL_CLOCK_DIVISOR;
	spi_ctrl.bits.cpsdvsr = HW_SPI_PRESCALE_DIVISOR;

	for (i = 0, id = 0; i < 2; i++) 
	{
		if (!bcm476x_spi_open(i, &spi_ctrl)) 
		{
			sflash.spi_select = i;
			bcm476x_spi_register_callback(sflash.spi_select, sflash_callback);
			
			id = sflash_get_id(&sflash);
			
			for (j = 0; j < ARRAYSIZE(sflash_info); j++) 
			{
				if (id == sflash_info[j].id) {
					sflash.blocksize = sflash_info[j].sector_size;
					sflash.numblocks = sflash_info[j].sector_count;
					sflash.pagesize = sflash_info[j].page_size;
#ifdef CONFIG_MTD_PARTITIONS
					bcm476x_mtd_sflash_parts = bcm476x_mtd_sflash_parts_info[j];
#endif
					goto done;
				}
			}
			bcm476x_spi_unregister_callback(sflash.spi_select);
			bcm476x_spi_close(sflash.spi_select);			
		}
	}

error:
	if (sflash.data_buf.virt) {
		dma_free_coherent(NULL, SFLASH_DMA_BUF_BYTE_SIZE, sflash.data_buf.virt, sflash.data_buf.phys);
		sflash.data_buf.virt = NULL;
	}
	if (sflash.cmd_buf.virt) {
		dma_free_coherent(NULL, SFLASH_CMD_BUF_BYTE_SIZE, sflash.cmd_buf.virt, sflash.cmd_buf.phys);
		sflash.cmd_buf.virt = NULL;
	}
	return NULL;

done:
	printk(KERN_NOTICE "SFlash: detected Serial Flash (Manufacturer ID = 0x%04x) at SPI %d\n", id, sflash.spi_select);
	sflash.size = sflash.blocksize * sflash.numblocks;
	return sflash.size ? &sflash : NULL;
}

#if LINUX_VERSION_CODE < 0x20212 && defined(MODULE)
#define sflash_mtd_init init_module
#define sflash_mtd_exit cleanup_module
#endif

static int __init sflash_mtd_init(void)
{
	int ret = 0;
	sflash_mtd *info;
	int spi_port = 0;
#ifdef CONFIG_MTD_PARTITIONS
	int i;
#endif

	// enable SPI port for serial flash
	if (bcm476x_set_spi_enable(spi_port, 1))    {
		goto fail;
	}
	
	memset(&sflash, 0, sizeof(sflash_mtd));
	init_MUTEX(&sflash.lock);

	/* Initialize serial flash access */
	info = sflash_init();

	if (!info) {
		printk(KERN_ERR "SFlash: found no supported devices\n");
		ret = -ENODEV;
		goto fail;
	}

	/* Setup region info */
	sflash.region.offset = 0;
	sflash.region.erasesize = info->blocksize;
	sflash.region.numblocks = info->numblocks;
	if (sflash.region.erasesize > sflash.mtd.erasesize) {
		sflash.mtd.erasesize = sflash.region.erasesize;
	}
	sflash.mtd.size = info->size;
	sflash.mtd.numeraseregions = 1;

	/* Register with MTD */
	sflash.mtd.name = "BCM47XX SFLASH-MTD";
	sflash.mtd.type = MTD_DATAFLASH;
	sflash.mtd.flags = MTD_CAP_NORFLASH;
	sflash.mtd.eraseregions = &sflash.region;
	sflash.mtd.owner = THIS_MODULE;
	sflash.mtd.erase = sflash_mtd_erase;
	sflash.mtd.read = sflash_mtd_read;
	sflash.mtd.write = sflash_mtd_write;
	sflash.mtd.priv = &sflash;

#ifdef CONFIG_MTD_PARTITIONS
	for (i = 0; bcm476x_mtd_sflash_parts[i].name; i++);
	ret = add_mtd_partitions(&sflash.mtd, bcm476x_mtd_sflash_parts, i);
	if (ret) {
		printk(KERN_ERR "SFLASH-MTD: add_mtd_partitions failed\n");
		goto fail;
	}
#else
	ret = add_mtd_device(&sflash.mtd);
	if (ret) {
		printk(KERN_ERR "SFLASH-MTD: add_mtd failed\n");
		goto fail;
	}
#endif

	return STATUS_OK;

fail:
	// disable SPI port for serial flash
	bcm476x_set_spi_enable(spi_port, 0);
	return -STATUS_FAILED;
}

static void __exit sflash_mtd_exit(void)
{
	int spi_port = 0;
	
	if (sflash.data_buf.virt) {
		dma_free_coherent(NULL, SFLASH_DMA_BUF_BYTE_SIZE, sflash.data_buf.virt, sflash.data_buf.phys);
		sflash.data_buf.virt = NULL;
	}
	if (sflash.cmd_buf.virt) {
		dma_free_coherent(NULL, SFLASH_CMD_BUF_BYTE_SIZE, sflash.cmd_buf.virt, sflash.cmd_buf.phys);
		sflash.cmd_buf.virt = NULL;
	}
	bcm476x_spi_unregister_callback(sflash.spi_select);
	bcm476x_spi_close(sflash.spi_select);

#ifdef CONFIG_MTD_PARTITIONS
	del_mtd_partitions(&sflash.mtd);
#else
	del_mtd_device(&sflash.mtd);
#endif

	// disable SPI port for serial flash
	bcm476x_set_spi_enable(spi_port, 0);
}

module_init(sflash_mtd_init);
module_exit(sflash_mtd_exit);

MODULE_LICENSE("GPL");
