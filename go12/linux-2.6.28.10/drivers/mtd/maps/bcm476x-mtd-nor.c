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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/string.h>
#include <asm/arch/platform.h>
#include <asm/arch/hardware.h>

//#include "tahoe_opmode.h"
//#include "nvflash.h"
#define BCM47XX_NOR_BASE    0x00000000          // For now

#define NOR_WINDOW_SIZE     (SZ_1M)
#define NOR_WINDOW_ADDR     BCM47XX_NOR_BASE
#define NOR_BUSWIDTH        2 /* NOR flash */

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition bcm476x_mtd_nor_parts[] = {
	{ name: "nor-boot", offset: 0x000000, size: NOR_WINDOW_SIZE-SZ_64K, },
	{ name: "nor-nvram", offset: NOR_WINDOW_SIZE-SZ_64K, size: SZ_64K, },
	{ name: NULL, },
};
#endif /* CONFIG_MTD_PARTITIONS */

static struct map_info bcm476x_mtd_nor_map = {
	name: "BCM476X NOR-MTD",
	size: NOR_WINDOW_SIZE,
	phys: NOR_WINDOW_ADDR,
	bankwidth: NOR_BUSWIDTH,
};
static struct mtd_info *bcm476x_mtd_nor;

static int __init bcm476x_mtd_nor_map_init(void)
{
	uint window_addr = 0, window_size = 0;
	size_t size;
	int ret = 0;
#ifdef CONFIG_MTD_PARTITIONS
	int i;
#endif
	const char *flash_type;

	flash_type = "nor";
	if (flash_type) {
		if (strncmp(flash_type, "nor", strlen("nor"))) {
			return -1;
		}
	}


	bcm476x_mtd_nor_map.map_priv_1 = 0;
	bcm476x_mtd_nor_map.map_priv_2 = 0;
	window_addr = bcm476x_mtd_nor_map.phys;
	window_size = bcm476x_mtd_nor_map.size;

	bcm476x_mtd_nor_map.virt = ioremap(window_addr, window_size);

	// enable NOR interface and set pin share for NOR
/*	{
		tahoe_opmode_corectrl_t core_info = { 0 };
		tahoe_stat_t stat;
		char stat_str[256];

		core_info.core = CORE_MNOR;
		core_info.stat = enable;
		stat = tahoe_opmode_config_core(&core_info, 1);
		if( stat != OpOk ) {
			tahoe_get_stat_str(stat, stat_str, sizeof(stat_str));
			printk(KERN_ERR "Error: %s(): tahoe_opmode_config_core(CORE_MNOR) failed: %s\n", __FUNCTION__, stat_str);
		}
	}
*/
	//Enable pin share
	
	//Enable clock
	
	if (!bcm476x_mtd_nor_map.virt) {
		printk(KERN_ERR "bcm476x_mtd_nor_map_init: ioremap failed\n");
		ret = -EIO;
		goto probe_nor_fail;
	}

	if (!(bcm476x_mtd_nor = do_map_probe("cfi_probe", &bcm476x_mtd_nor_map))) {
		if (!(bcm476x_mtd_nor = do_map_probe("jedec_probe", &bcm476x_mtd_nor_map))) {
			// printk(KERN_ERR "init_bcm476x_mtd_map: CFI and JEDEC probe failed\n");
			ret = -ENXIO;
			goto probe_nor_fail;
		}
	}

	bcm476x_mtd_nor->owner = THIS_MODULE;
	size = bcm476x_mtd_nor->size;
	printk(KERN_NOTICE "BCM476X NOR-MTD device: 0x%x at 0x%x\n", size, window_addr);

#ifdef CONFIG_MTD_PARTITIONS
	for (i = 0; bcm476x_mtd_nor_parts[i].name; i++);
	ret = add_mtd_partitions(bcm476x_mtd_nor, bcm476x_mtd_nor_parts, i);
	if (!ret) goto probe_nor_end;

	printk(KERN_ERR "NOR-MTD: add_mtd_partitions failed\n");
#else
	ret = add_mtd_device(bcm476x_mtd_nor);
	if (!ret) goto probe_nor_end;

	printk(KERN_ERR "NOR-MTD: add_mtd failed\n");
#endif

probe_nor_fail:
	if (bcm476x_mtd_nor)
		map_destroy(bcm476x_mtd_nor);
	if (bcm476x_mtd_nor_map.virt)
		iounmap((void *) bcm476x_mtd_nor_map.virt);
	bcm476x_mtd_nor_map.virt = 0;

probe_nor_end:
	// disable NOR interface and set pin share for LCD
/*	{
		tahoe_opmode_corectrl_t core_info = { 0 };
		tahoe_stat_t stat;
		char stat_str[256];

		core_info.core = CORE_MNOR;
		core_info.stat = disable;
		stat = tahoe_opmode_config_core(&core_info, 1);
		if( stat != OpOk ) {
			tahoe_get_stat_str(stat, stat_str, sizeof(stat_str));
			printk(KERN_ERR "Error: %s(): tahoe_opmode_config_core(CORE_MNOR) failed: %s\n", __FUNCTION__, stat_str);
		}
	}
*/	

	//Disable clock
	//*()	
	return ret;
}

static void __exit bcm476x_mtd_nor_map_exit(void)
{
#ifdef CONFIG_MTD_PARTITIONS
	del_mtd_partitions(bcm476x_mtd_nor);
#else /* CONFIG_MTD_PARTITIONS */
	del_mtd_device(bcm476x_mtd_nor);
#endif
	map_destroy(bcm476x_mtd_nor);
	iounmap((void *)bcm476x_mtd_nor_map.virt);
	bcm476x_mtd_nor_map.virt = 0;
}

module_init(bcm476x_mtd_nor_map_init);
module_exit(bcm476x_mtd_nor_map_exit);

MODULE_LICENSE("GPL");
