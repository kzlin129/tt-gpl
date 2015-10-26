/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition xboard_mtd_onenand_parts[] =
{
	{
		.name = "boot1",
		.offset = 0x0,
		.size = 0x4000,
		.mask_flags = MTD_NO_ERASE
	},
	{
		.name = "boot2",
		.offset = 0x4000,
		.size = 0x10000
	},
	{
		.name = "CP",
		.offset = 0x14000,
		.size = 0x1fec000,
		.mask_flags = MTD_NO_ERASE
	},
	{
		.name = "kernel",
		.offset = 0x2000000,
		.size = 0x200000,
		.mask_flags = MTD_NO_ERASE
	},
	{
		.name = "root0",
		.offset = 0x2200000,
		.size = 0x400000,
	},
	{
		.name = "root1",
		.offset = 0x2600000,
		.size = 0x1400000,
	},
	{
		.name = "root2",
		.offset = 0x3a00000,
		.size = 0x400000,
	},
	{
		.name = NULL,
		
	}
};
#endif /* CONFIG_MTD_PARTITIONS */

struct onenand_info {
	struct mtd_info *mtd;
	struct onenand_chip	*onenand;
};

static struct onenand_info *info;

static void xboard_onenand_cleanup (void)
{
	if (info)
	{
		if (info->mtd)
		{
#ifdef CONFIG_MTD_PARTITIONS
			del_mtd_partitions (info->mtd);
#else  /* CONFIG_MTD_PARTITIONS */
#ifdef CONFIG_CMDLINE
	// FIXME?
#endif /* CONFIG_CMDLINE */
#endif /* no CONFIG_MTD_PARTITIONS */
			del_mtd_device (info->mtd);

			kfree (info->mtd);
			info->mtd = NULL;
		}
		if (info->onenand)
		{
			if (info->onenand->base)
			{
				iounmap ((void *) info->onenand->base);
				info->onenand->base = NULL;
			}

			kfree (info->onenand);
			info->onenand = NULL;
		}
		kfree (info);
		info = NULL;
	}
}

static int __init xboard_onenand_init (void)
{
#ifdef CONFIG_MTD_PARTITIONS
	int i, ret;
#endif /* CONFIG_MTD_PARTITIONS */

	printk ( KERN_INFO "BCM OneNAND: Loading X-Board OneNAND driver...\n");

	info = kzalloc (sizeof (struct onenand_info), GFP_KERNEL);
	if (!info)
	{
		printk ("BCM OneNAND: Unable to allocate X-Board OneNAND device structure\n");
		return (-ENOMEM);
	}
	memset (info, 0, sizeof (struct onenand_info));
	
	info->mtd = kzalloc (sizeof (struct mtd_info), GFP_KERNEL);
	if (!info->mtd)
	{
		printk ("BCM OneNAND: Unable to allocate X-Board OneNAND MTD device structure\n");
		xboard_onenand_cleanup ();
		return (-ENOMEM);
	}	
	memset (info->mtd, 0, sizeof (struct mtd_info));
	
	info->onenand = kzalloc (sizeof (struct onenand_chip), GFP_KERNEL);
	if (!info->onenand)
	{
		printk ("BCM OneNAND: Unable to allocate X-Board OneNAND chip device structure\n");
		xboard_onenand_cleanup ();
		return (-ENOMEM);
	}
	memset (info->onenand, 0, sizeof (struct onenand_chip));

	/* set OneNAND base address explicitly */
	info->onenand->base =
		ioremap (ONENAND_MEMORY_MAP (0x0000),
				 (SZ_64K<<1)); /* Max. Bus Addr. is 16bit but 1bit shifted by HW */
	// info->onenand->mmcontrol = ???;
	// info->onenand->irq = ???;
	/* onenand_probe function may fill onenand_chip structure automatically in onenand_scan */

	info->mtd->name = "BCM2153 OneNAND";
	info->mtd->priv = info->onenand;
	info->mtd->owner = THIS_MODULE;

	printk ("BCM OneNAND: scanning X-Board OneNAND!\n");

	if (onenand_scan (info->mtd, 1))
	{
		printk ("BCM OneNAND: Unable to scan X-Board OneNAND!\n");
		xboard_onenand_cleanup ();
		return (-ENXIO);
	}

#ifdef CONFIG_MTD_PARTITIONS
	/* Calculates how many partition in the partition structure  */
	for (i = 0; xboard_mtd_onenand_parts[i].name; i++);
	
	ret = add_mtd_partitions(info->mtd, xboard_mtd_onenand_parts, i);

	if (ret != 0)
	{
		printk ("BCM OneNAND: Unable to add OneNAND Partition, Err: %d.\n", ret);
		xboard_onenand_cleanup ();
		return (ret);
	}
	
#else  /* CONFIG_MTD_PARTITIONS */
#ifdef CONFIG_CMDLINE
	// FIXME?
#endif /* CONFIG_CMDLINE */
#endif /* no CONFIG_MTD_PARTITIONS */

	printk ("BCM OneNAND: Adding X-Board OneNAND MTD device\n");

	add_mtd_device(info->mtd);

	printk ( KERN_INFO "BCM OneNAND: Loading X-Board OneNAND driver done\n");

	return (0);
}

static void __exit xboard_onenand_exit (void)
{
	xboard_onenand_cleanup ();
}

module_init(xboard_onenand_init);
module_exit(xboard_onenand_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Broadcom"); 
MODULE_DESCRIPTION("XBOARD specific OneNAND device driver"); 