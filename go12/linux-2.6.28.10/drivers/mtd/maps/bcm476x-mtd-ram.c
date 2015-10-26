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
#include <asm/arch/platform.h>

#define RAM_BUSWIDTH        4 /* RAM */

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition bcm476x_mtd_ram_parts[] = {
	{ name: "ram-rootfs", offset: 0x00000, mask_flags: MTD_WRITEABLE, },
	{ name: NULL, },
};
#endif /* CONFIG_MTD_PARTITIONS */

static struct map_info bcm476x_mtd_ram_map = {
	name: "BCM476X RAM-MTD",
	bankwidth: RAM_BUSWIDTH,
};
static struct mtd_info *bcm476x_mtd_ram;

static int __init bcm476x_mtd_ram_map_init(void)
{
	uint window_addr = 0, window_size = 0;
	size_t size;
	int ret = 0;
#ifdef CONFIG_MTD_PARTITIONS
	int i;
#endif

	char * ram_mtd_size_start;
	char ram_mtd_size_str[32];
	unsigned long ram_mtd_size;
	char * linux_mem_size_start;
	char linux_mem_size_str[32];
	unsigned long linux_mem_size;
	int cmd_size;

	/* Parse kernel command line to figure out ram_mtd_size
	 */
	ram_mtd_size_start = strstr(saved_command_line, "ram_mtd_size=");

	if (ram_mtd_size_start == NULL)
		goto probe_ram_fail;

	i = 0;
	cmd_size = strlen("ram_mtd_size=");
	while (*(ram_mtd_size_start + cmd_size + i) != ' ')	{
		ram_mtd_size_str[i] = * (ram_mtd_size_start + cmd_size + i);
		i++;
	}
	ram_mtd_size_str[i] = '\0';
	
	ram_mtd_size = simple_strtoul(ram_mtd_size_str, NULL, 10);
		
	if (ram_mtd_size == 0)
		goto probe_ram_fail;
	

	/* Parse kernel command line to figure out mem size
	 */
	linux_mem_size_start = strstr(saved_command_line, "mem=");

	if (linux_mem_size_start == NULL) {
		printk(KERN_ERR "memory size not specified in kernel command line!\n");
		BUG();
	}

	i = 0;
	cmd_size = strlen("mem=");
	while (*(linux_mem_size_start + cmd_size + i) != ' ') {
		linux_mem_size_str[i] = * (linux_mem_size_start + cmd_size + i);
		i++;
	}
	linux_mem_size_str[i] = '\0';


	linux_mem_size = simple_strtoul(linux_mem_size_str, NULL, 10);
		
//////// MTD RAM ////////
	bcm476x_mtd_ram_map.map_priv_1 = 0;
	bcm476x_mtd_ram_map.map_priv_2 = 0;
	bcm476x_mtd_ram_map.phys = BCM47XX_ARM_DRAM + (linux_mem_size * SZ_1M);
	bcm476x_mtd_ram_map.size = ram_mtd_size * SZ_1M;
	window_addr = bcm476x_mtd_ram_map.phys;
	window_size = bcm476x_mtd_ram_map.size;

	bcm476x_mtd_ram_map.virt = ioremap(window_addr, window_size);
	if (!bcm476x_mtd_ram_map.virt) {
		printk(KERN_ERR "bcm476x_mtd_ram_map_init: ioremap failed\n");
		ret = -EIO;
		goto probe_ram_fail;
	}

	if (!(bcm476x_mtd_ram = do_map_probe("map_ram", &bcm476x_mtd_ram_map))) {
		printk(KERN_ERR "init_bcm476x_mtd_map: RAM probe failed\n");
		ret = -ENXIO;
		goto probe_ram_fail;
	}

	bcm476x_mtd_ram->owner = THIS_MODULE;
	size = bcm476x_mtd_ram->size;
	printk(KERN_NOTICE "BCM476X RAM-MTD device: size 0x%x at 0x%x\n", size, window_addr);

#ifdef CONFIG_MTD_PARTITIONS
	bcm476x_mtd_ram_parts[0].size = window_size;
	for (i = 0; bcm476x_mtd_ram_parts[i].name; i++);
	ret = add_mtd_partitions(bcm476x_mtd_ram, bcm476x_mtd_ram_parts, i);
	if (!ret) goto probe_ram_end;

	printk(KERN_ERR "RAM-MTD: add_mtd_partitions failed\n");
#else
	ret = add_mtd_device(bcm476x_mtd_ram);
	if (!ret) goto probe_ram_end;

	printk(KERN_ERR "RAM-MTD: add_mtd failed\n");
#endif

probe_ram_fail:
	if (bcm476x_mtd_ram)
		map_destroy(bcm476x_mtd_ram);
	if (bcm476x_mtd_ram_map.virt)
		iounmap((void *) bcm476x_mtd_ram_map.virt);
	bcm476x_mtd_ram_map.virt = 0;

probe_ram_end:
	return ret;
}

static void __exit bcm476x_mtd_ram_map_exit(void)
{
#ifdef CONFIG_MTD_PARTITIONS
	del_mtd_partitions(bcm476x_mtd_ram);
#else /* CONFIG_MTD_PARTITIONS */
	del_mtd_device(bcm476x_mtd_ram);
#endif

	map_destroy(bcm476x_mtd_ram);
	iounmap((void *)bcm476x_mtd_ram_map.virt);
	bcm476x_mtd_ram_map.virt = 0;
}

module_init(bcm476x_mtd_ram_map_init);
module_exit(bcm476x_mtd_ram_map_exit);
