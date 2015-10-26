/*
 *  linux/drivers/ide/arm/tomtom.c - TomTomGo IDE bus glue
 *
 *  Copyright (C) 2004 TomTom BV
 *  Author: Thomas Kleffel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Major TODOs:
 *   - proper disconnecting from generic ide driver
 *   - DMA data transfer using memory to memory DMA
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ide.h>

#include <asm/hardware.h>
#include <asm/io.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/tomtomgo-wake.h>

#include <barcelona/gopins.h>
#include <barcelona/debug.h>

#define PFX "ide-tomtom: "
#define PK_DBG PK_DBG_FUNC

static unsigned long tomtom_ide_drive_va;

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

int ide_tomtom_getslot(hw_regs_t *hw, ide_hwif_t **hwifp) {
	ide_hwif_t *hwif;
	int index;

	for (index = 0; index < MAX_HWIFS; ++index) {
		hwif = &ide_hwifs[index];
		if (hwif->hw.io_ports[IDE_DATA_OFFSET] == hw->io_ports[IDE_DATA_OFFSET]) {
			if(hwifp) (*hwifp)=hwif;
			return index;
		}
	}

	for (index = 0; index < MAX_HWIFS; ++index) {
		hwif = &ide_hwifs[index];
		if (hwif->hold)
			continue;

		if (!hwif->hw.io_ports[IDE_DATA_OFFSET]) {
			if(hwifp) (*hwifp)=hwif;
			return index;
		}
	}

	if(hwifp) (*hwifp) = 0;
	return -1;
}

int ide_tomtom_register(hw_regs_t *hw, ide_hwif_t **hwifp) {
	ide_hwif_t *hwif;
	int index;

	index = ide_tomtom_getslot(hw, &hwif);
	if(!hwif) return -2;

	memcpy(&hwif->hw, hw, sizeof(*hw));
	memcpy(hwif->io_ports, hwif->hw.io_ports, sizeof(hwif->hw.io_ports));

	hwif->irq 	= hw->irq;
	hwif->noprobe 	= 0;
	hwif->chipset 	= hw->chipset;

	//We have no access to the CONTROL register and thus cannot do lba48!
	hwif->no_lba48	= 1;

	default_hwif_mmiops(hwif);

	probe_hwif_init_with_fixup(hwif, NULL);
	create_proc_ide_interfaces();

	if (hwifp) *hwifp = hwif;

	return (hwif->present) ? index : -1;
}

static void ide_tomtom_hw_init(void)
{

	__raw_writel((__raw_readl(S3C2410_BWSCON) & (~(7 << 12))) | (1 << 12) | (0 << 14), S3C2410_BWSCON);
	__raw_writel((0 << 4) | (2 << 6) | (IO_GetHarddiskTiming() << 8) | (2 << 11) | (0 << 13), S3C2410_BANKCON3);

	IO_SetInput(HDD_DRQ);
	IO_SetInput(HDD_DACK);
	IO_Activate(HDD_PWR_ON);
	IO_Activate(SD_PWR_ON); // Temporary for Valencia
	IO_SetInterruptOnActivation(HDD_IRQ);
	IO_SetInput(HDD_LED);
	IO_Activate(HDD_BUF_EN);
	IO_SetFunction(HDD_CS);

	/* Now give the HDD a hard reset 	*/
	IO_Activate(HDD_RST);
	msleep(50);
	IO_Deactivate(HDD_RST);
	msleep( 50 ); /* Wait a while, wait until the firmware of HDD booted	*/ 
}

static void ide_tomtom_hw_exit(void)
{
	IO_Activate(HDD_RST);
	msleep(500);
	IO_Deactivate(HDD_CS);
	IO_Deactivate(HDD_BUF_EN);
	IO_Deactivate(SD_PWR_ON); // Temporary for Valencia
	IO_Deactivate(HDD_PWR_ON);
	IO_Deactivate(HDD_IRQ);
	IO_Deactivate(HDD_LED);
	IO_Deactivate(HDD_DRQ);
	IO_Deactivate(HDD_DACK);

#if 0
	// Activate buffer again to save power in sleep mode
	IO_Activate(HDD_BUF_EN);
	IO_Activate(HDD_CS);
#endif
}

static int ide_tomtom_probe(struct device *dev)
{
	struct resource *mem;
	void __iomem *base;
	int irq;
	hw_regs_t ide_hw;
	ide_hwif_t *ide_hwif;

	int index, p;

	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		PK_ERR("failed to get io memory region resource.\n");
		ret = -ENOENT;
		goto probe_out;
	}

	mem = request_mem_region(mem->start, RESSIZE(mem), pdev->name);
	if (!mem) {
		PK_ERR("failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_out;
	}

	base = ioremap(mem->start, RESSIZE(mem));
	if (base == 0) {
		PK_ERR("failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region;
	}

	/** Store the base virtual address for later use	*/
	tomtom_ide_drive_va = (unsigned long) base;
#if 1
	irq = IO_GetInterruptNumber(HDD_IRQ);
#else
	irq = platform_get_irq(pdev, 0);
#endif
	if (irq == 0) {
		PK_ERR("failed to get interrupt resource.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	//We don't need to wait for hd powerup as linux's ide code
	//will wait up to 35 seconds.

	PK_DBG("mapped mem:[%p]=>[%p] irq:%u\n", (void *) mem->start, base, irq);

	//prepare ide_hw structure
	memzero(&ide_hw, sizeof(ide_hw));

	for(p=0;p<8;p++) {
		ide_hw.io_ports[p] = (unsigned long) (base + (4*p));
	}

	// Also set base address for HDD temperature check
	tomtom_hddtemp_setaddress((unsigned long) base);

	ide_hw.chipset	= ide_generic;
	ide_hw.dma	= NO_DMA;
	ide_hw.irq	= irq;

	// Init the hardware
	ide_tomtom_hw_init();

	// Register device with IDE driver
	index = ide_tomtom_register(&ide_hw, &ide_hwif);
	if(index < 0) {
		PK_ERR("failed to register ide interface (%i).\n",index);
		ret = -EINVAL;
		goto probe_shutdown;
	}

	PK_DBG("initialisation done. Index:%i\n",index);
	return 0;

probe_shutdown:
	// Deinit hardware
	ide_tomtom_hw_exit();

probe_iounmap:
	iounmap(base);

probe_free_mem_region:
	release_mem_region(mem->start, RESSIZE(mem));

probe_out:
	return ret;
}

static int ide_tomtom_remove(struct device *dev)
{
	ide_tomtom_hw_exit();
	return 0;
}

static void ide_tomtom_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	ide_tomtom_hw_exit();
}

#ifdef CONFIG_PM

static int ide_tomtom_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if(level == SUSPEND_POWER_DOWN) {
		ide_tomtom_hw_exit();
	}
	return 0;
}

static int ide_tomtom_resume(struct device *dev, u32 level)
{
	unsigned int stat;
	int timeout;

	PK_DBG("dev = %p, level = %u\n", dev, level);
	if(level == RESUME_POWER_ON) {
		ide_tomtom_hw_init();
		/* Go wait until the spin up is complete before we issue commands to it	*/
		timeout = 0;
		do {
		msleep( 50 );
		stat = ioread8(tomtom_ide_drive_va + (7 << 2) ); /* Go fetch IDE status register (offset 7, A0 and A1 are not connected) */
		} while(!(stat & READY_STAT) && timeout++ < 5);
	}
	return 0;
}

#else /* CONFIG_PM */
#define ide_tomtom_suspend NULL
#define ide_tomtom_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver tomtomide_driver =
{
	.name           = "tomtomgo-ide",
	.bus            = &platform_bus_type,
	.probe          = ide_tomtom_probe,
	.remove         = ide_tomtom_remove,
	.shutdown       = ide_tomtom_shutdown,
	.suspend        = ide_tomtom_suspend,
	.resume         = ide_tomtom_resume,
};

static int __init ide_tomtom_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO IDE Driver, (C) 2004,2005 TomTom BV\n");

	if (!IO_HaveHarddisk()) {
		PK_DBG("No harddisk in this model.\n");
		return -ENODEV;
	}

	PK_DBG("Registering driver\n");
	ret = driver_register(&tomtomide_driver);
	if (ret)
	{
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit ide_tomtom_exit(void)
{
	if (!IO_HaveHarddisk())
		return;

	PK_DBG("Unregistering driver\n");
	driver_unregister(&tomtomide_driver);
	PK_DBG("Done\n");
}

module_init(ide_tomtom_init);
module_exit(ide_tomtom_exit);

MODULE_DESCRIPTION("TomTom GO IDE bus glue");
MODULE_LICENSE("GPL");
