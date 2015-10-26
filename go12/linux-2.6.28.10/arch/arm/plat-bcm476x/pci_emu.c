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
/* BCM47XX BSP emulated PCI layer */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <mach/hw_cfg.h>
#include <linux/version.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/delay.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>
#include <asm/arch/pci_emu.h>
#include <asm/arch/platform.h>
#include <asm/arch/cm.h>

extern struct pci_bus *pci_scan_bus(int, struct pci_ops*, void*);

/* Function Prototypes */
#ifdef CONFIG_MMC
static void sdio_init  (const struct pci_emu_dev_t * );
#endif
#if defined(CONFIG_MACH_BCM4760_TOMTOM)	
static void set_mmc_max_speed( int mmc, unsigned long speed );
int bcm476x_get_boot_sdmmc_device(void);
static unsigned long local_mmc_clock[]={BCM47XX_DEFAULT_BASE_CLOCK, BCM47XX_DEFAULT_BASE_CLOCK, BCM47XX_DEFAULT_BASE_CLOCK};
#endif
static int emu_dev_readcfg (u32 bus, u32 dev, u32 func, int where, int size, u32 *val);
static int emu_dev_writecfg (u32 bus, u32 dev, u32 func, int where, int size, u32 val);

/*
 * Device specific configurations
 */
#if 0   // KP: defined(CONFIG_ARCH_BCM4760) && defined(CONFIG_USB_SUPPORT)
static pci_emu_cfg_t	usb2h_cfg[2] =
{
	/* Function 0, USB Host OHCI Controller */
	{
		.device		= PCI_DEVICE_ID_BCM47XX_USB,
		.prog_if	= 0x10,
		.class		= PCI_CLASS_SERIAL_USB,
		.header		= PCI_HEADER_TYPE_MULTFUNC,
		.int_line	= BCM4760_INTR_USB,
		.int_pin	= 0,
		.bars[0]	=
		{
			.mapping	= PCI_BASE_ADDRESS_SPACE_MEMORY,
			.size 		= 0x800,
		},

		.read		= emu_dev_readcfg,
		.write		= emu_dev_writecfg,
	},

	/* Function 1, USB Host EHCI Controller */
	{
		.device		= PCI_DEVICE_ID_BCM47XX_USB,
		.prog_if	= 0x20,
		.class		= PCI_CLASS_SERIAL_USB,
		.header		= PCI_HEADER_TYPE_MULTFUNC,
		.int_line	= BCM4760_INTR_USB,
		.int_pin	= 0,
		.bars[0]	=
		{
			.mapping	= PCI_BASE_ADDRESS_SPACE_MEMORY,
			.size 		= 0x800,
		},

		.read		= emu_dev_readcfg,
		.write		= emu_dev_writecfg,
	},
};
#endif


#ifdef CONFIG_MMC
static pci_emu_cfg_t	sdio_s0_cfg[1] =
{
	/* ARASAN SDIO Host Controller */
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		.device		= PCI_DEVICE_ID_BCM_SD, /*default, will be changed based on nvflash */
#else
		.device		= PCI_DEVICE_ID_BCM47XX_SD, /*default, will be changed based on nvflash */
#endif
		.prog_if	= 0x01,    /* SDHCI driver sees 0x01 as the DMA capable. */
		.class		= PCI_CLASS_SYSTEM_SDHCI,
		.header		= PCI_HEADER_TYPE_NORMAL,
        // @KP: TODO: need to verify int_line and int_pin is set up correctly
		.int_line	= BCM4760_INTR_SDM0,
		.int_pin	= 0,
		.bars[0]	=
		{
			.mapping	= PCI_BASE_ADDRESS_SPACE_MEMORY,
			.size 		= 0x100, /* SDHCI driver takes 0x100 exact. */

		},

		.read		= emu_dev_readcfg,
		.write		= emu_dev_writecfg,
	}
};

static pci_emu_cfg_t	sdio_s1_cfg[1] =
{
	/* ARASAN SDIO Host Controller */
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		.device		= PCI_DEVICE_ID_BCM_SD, /*default, will be changed based on nvflash */
#else
		.device		= PCI_DEVICE_ID_BCM47XX_SD, /*default, will be changed based on nvflash */
#endif
		.prog_if	= 0x01,    /* SDHCI driver sees 0x01 as the DMA capable. */
		.class		= PCI_CLASS_SYSTEM_SDHCI,
		.header		= PCI_HEADER_TYPE_NORMAL,
        // @KP: TODO: need to verify int_line and int_pin is set up correctly
		.int_line	= BCM4760_INTR_SDM1,
		.int_pin	= 0,
		.bars[0]	=
		{
			.mapping	= PCI_BASE_ADDRESS_SPACE_MEMORY,
			.size 		= 0x100, /* SDHCI driver takes 0x100 exact. */

		},

		.read		= emu_dev_readcfg,
		.write		= emu_dev_writecfg,
	}
};

static pci_emu_cfg_t	sdio_s2_cfg[1] =
{
	/* ARASAN SDIO Host Controller */
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		.device		= PCI_DEVICE_ID_BCM_SD, /*default, will be changed based on nvflash */
#else
		.device		= PCI_DEVICE_ID_BCM47XX_SD, /*default, will be changed based on nvflash */
#endif
		.prog_if	= 0x01,    /* SDHCI driver sees 0x01 as the DMA capable. */
		.class		= PCI_CLASS_SYSTEM_SDHCI,
		.header		= PCI_HEADER_TYPE_NORMAL,
        // @KP: TODO: need to verify int_line and int_pin is set up correctly
		.int_line	= BCM4760_INTR_SDM2,
		.int_pin	= 0,
		.bars[0]	=
		{
			.mapping	= PCI_BASE_ADDRESS_SPACE_MEMORY,
			.size 		= 0x100, /* SDHCI driver takes 0x100 exact. */

		},

		.read		= emu_dev_readcfg,
		.write		= emu_dev_writecfg,
	}
};

#endif


/*
 * BCM47xx emulated PCI device table
 */

static pci_emu_dev_t	bcm47xx_pci_emu_tbl[] =
{
#if 0   // KP: defined(CONFIG_ARCH_BCM4760) && defined(CONFIG_USB_SUPPORT)
	{  /* USB 2.0 Host Device */
		.dev_base		= USB_REG_BASE_ADDR,
		.init			= usb2h_init,
		.num_funcs		= sizeof(usb2h_cfg)/sizeof(pci_emu_cfg_t),
		.pci_emu_cfgs		= usb2h_cfg,
	},
#endif
};


static pci_emu_dev_t *bcm47xx_pci_emu_dynamic_tbl = NULL;

static int num_of_devs = sizeof(bcm47xx_pci_emu_tbl)/sizeof(pci_emu_dev_t);
static int num_of_dynamic_devs = 0;

/* Emulated PCI bus spin lock
 */
static spinlock_t bcm47xx_pci_emu_lock;


/*
 * Device Initialization Functions
 */

#define USB_UTMI_CTRL1		0x0910
#define USB_POWER_OFF_PHY_VAL	0x1300

#if 0   // KP: defined(CONFIG_ARCH_BCM4760) && defined(CONFIG_USB_SUPPORT)
/* USB 2.0 host controller initialization function */
static void usb2h_init (const struct pci_emu_dev_t * dev)
{
	u16			vendor;
	pci_config_regs *	cfg;
	u32			func;
	u32			baseaddr0;

	vendor	= PCI_VENDOR_ID_BROADCOM;

	/* Reset and restart the device
	 */
	bcm476x_reset_device(USB_RESET_ID);

	/* Turn the USB PHY off
	 * See IMS Issue #41784
	 * The bcm47xx_reset_device(USB_RESET_ID) function above resets
	 * the USB PHY, which in turn sets the register below to the
	 * power-on state of 0x300
	 * We need to, instead, set it to 0x1300. Otherwise,
	 * The host PC will complain, saying "USB device has malfunctioned"
	 */
	writel(USB_POWER_OFF_PHY_VAL, IO_ADDRESS(BCM47XX_USBH0_UTMI_CTRL1_PHY));

	/* USB 2.0 host core has two functions: OHCI and EHCI. The
	 * 1st 2K of the host configuration address space is reserved
	 * for the OHCI function and the 2nd half space is reserved
	 * for the EHCI function.
	 */
	baseaddr0 = dev->dev_base;

	for (func = 0; func < 2; func++) {
		cfg			= &(dev->pci_emu_cfgs[func].pci_cfg_regs);
		cfg->vendor		= cpu_to_le16(vendor);
		cfg->device		= cpu_to_le16(dev->pci_emu_cfgs[func].device);
		cfg->rev_id		= 0;
		cfg->prog_if		= dev->pci_emu_cfgs[func].prog_if;
		cfg->class		= dev->pci_emu_cfgs[func].class;
		cfg->header_type	= dev->pci_emu_cfgs[func].header;
		cfg->base[0]	= cpu_to_le32(((func == 0) ? baseaddr0 : (baseaddr0 + 0x800)) | dev->pci_emu_cfgs[func].bars[0].mapping);
		cfg->base[1]		= 0;
		cfg->base[2]		= 0;
		cfg->base[3]		= 0;
		cfg->base[4]		= 0;
		cfg->base[5]		= 0;
		cfg->int_line		= dev->pci_emu_cfgs[func].int_line;
		cfg->int_pin		= dev->pci_emu_cfgs[func].int_pin;

		*((u32 *) &cfg->sprom_control) = 0xffffffff;
	}
}
#endif


#ifdef CONFIG_MMC

#define SDIO_NSLOT_BAROFFSET	0x40

/*
 * SDIO Host Controller Initialization Functions
 */
static void sdio_init (const struct pci_emu_dev_t * dev)
{
	u16 		vendor;
	pci_config_regs *	cfg;
	u32 		dev_base;

	vendor	= PCI_VENDOR_ID_BROADCOM;

	/* Reset and restart the device
	 */
	bcm476x_reset_device(SDIO_1_RESET_ID);

	dev_base = dev->dev_base;

	cfg			    = &(dev->pci_emu_cfgs[0].pci_cfg_regs);
	cfg->vendor		= cpu_to_le16(vendor);
	cfg->device		= cpu_to_le16(dev->pci_emu_cfgs[0].device);
	cfg->rev_id		= 0x0;
	cfg->prog_if	= dev->pci_emu_cfgs[0].prog_if;
	cfg->class		= dev->pci_emu_cfgs[0].class;
	cfg->header_type	= dev->pci_emu_cfgs[0].header;
	cfg->base[0]		= cpu_to_le32(dev_base | dev->pci_emu_cfgs[0].bars[0].mapping);
	cfg->base[1]		= 0;
	cfg->base[2]		= 0;
	cfg->base[3]		= 0;
	cfg->base[4]		= 0;
	cfg->base[5]		= 0;
	cfg->subsys_vendor	= cpu_to_le16(PCI_VENDOR_ID_BROADCOM);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	cfg->subsys_id		= cpu_to_le16(PCI_DEVICE_ID_BCM_SD);
#else
	cfg->subsys_id		= cpu_to_le16(PCI_DEVICE_ID_BCM47XX_SD);
#endif
	cfg->int_line		= dev->pci_emu_cfgs[0].int_line;
	cfg->int_pin		= dev->pci_emu_cfgs[0].int_pin;

	/* power manager support */
	cfg->status		    = PCI_STATUS_CAP_LIST;
	cfg->rsvd_a[0]		= 0x48; //pointer to PM capabilities register
	cfg->dev_dep[0 + 8]	= PCI_CAP_ID_PM; //power manager capability
	cfg->dev_dep[1 + 8]	= 0; //no more capabilities
	cfg->dev_dep[2 + 8]	= 0;
	cfg->dev_dep[3 + 8]	= 0;
	cfg->dev_dep[4 + 8]	= 0;
	cfg->dev_dep[5 + 8]	= 0;
	// cfg->dev_dep[2 + 8]	=  PCI_D1| PCI_D0;
	cfg->dev_dep[3 + 8]	= ((PCI_PM_CAP_D1 | PCI_PM_CAP_D2) >> 8);

	*((u32 *) &cfg->sprom_control) = 0xffffffff;

	/* Preset the num of slots and bar offset information to be 0x0 and 0x0 to match
	 * what is expected in the SDIO driver stack.
	 */
	*((u8 *)((u32)cfg + SDIO_NSLOT_BAROFFSET)) = 0x00;

}

#endif


static int emu_dev_readcfg(u32 bus, u32 dev, u32 func, int where, int size, u32 *val)
{
	pci_config_regs *cfg;

	if ((where + size) > sizeof(pci_config_regs))
		return -1;

	cfg = &bcm47xx_pci_emu_dynamic_tbl[dev].pci_emu_cfgs[func].pci_cfg_regs;

	if (size == 4)
		*((u32 *) val) = le32_to_cpu(*((u32 *)((u32) cfg + where)));
	else if (size == 2)
		*((u16 *) val) = le16_to_cpu(*((u16 *)((u32) cfg + where)));
	else if (size == 1)
		*((u8 *) val) = *((u8 *)((u32) cfg + where));
	else
		return -1;

	return 0;
}

static int emu_dev_writecfg(u32 bus, u32 dev, u32 func, int where, int size, u32 val)
{
	pci_config_regs *cfg;

	if ((where + size) > sizeof(pci_config_regs))
		return -1;

	cfg = &bcm47xx_pci_emu_dynamic_tbl[dev].pci_emu_cfgs[func].pci_cfg_regs;

	/* Emulate BAR sizing */
	if (where >= PCI_BASE_ADDRESS_0 && where <= PCI_BASE_ADDRESS_5 &&
			size == 4 && val == ~0)
	{
		int bar_index;
		u32 size;
		bar_info_t *bar_info;

		bar_index = (where - PCI_BASE_ADDRESS_0)/0x04;
		bar_info = &bcm47xx_pci_emu_dynamic_tbl[dev].pci_emu_cfgs[func].bars[bar_index];

		size = ~(bar_info->size - 1);
		size |= (bar_info->mapping == PCI_BASE_ADDRESS_SPACE_MEMORY) ?  PCI_BASE_ADDRESS_SPACE_MEMORY : PCI_BASE_ADDRESS_SPACE_IO;
		cfg->base[bar_index] = size;

		return 0;
	}

	if (size == 4)
		*((u32 *)((u32) cfg + where)) = cpu_to_le32((u32) val);
	else if (size == 2)
		*((u16 *)((u32) cfg + where)) = cpu_to_le16((u16) val);
	else if (size == 1)
		*((u8 *)((u32) cfg + where)) = (u8) val;
	else
		return -1;

	return 0;
}


static int bcm47xx_pci_readcfg(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	unsigned long flags;
	u32 bus_num = bus->number;
	u32 dev_num = PCI_SLOT(devfn);
	u32 func_num = PCI_FUNC(devfn);
	int ret;

	spin_lock_irqsave(&bcm47xx_pci_emu_lock, flags);

	if (bus_num == 0)
	{
		if ((dev_num >= num_of_dynamic_devs) || (func_num >= bcm47xx_pci_emu_dynamic_tbl[dev_num].num_funcs))
			ret = -1;
		else
			ret = bcm47xx_pci_emu_dynamic_tbl[dev_num].pci_emu_cfgs[func_num].read(bus_num, dev_num, func_num, where, size, val);
	}
	else
		ret = -1;

	spin_unlock_irqrestore(&bcm47xx_pci_emu_lock, flags);

	return (ret ? PCIBIOS_DEVICE_NOT_FOUND : PCIBIOS_SUCCESSFUL);
};


static int bcm47xx_pci_writecfg(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	unsigned long flags;
	u32 bus_num = bus->number;
	u32 dev_num = PCI_SLOT(devfn);
	u32 func_num = PCI_FUNC(devfn);
	int ret;

	spin_lock_irqsave(&bcm47xx_pci_emu_lock, flags);

	if (bus_num == 0)
	{
		if ((dev_num >= num_of_dynamic_devs) || (func_num >= bcm47xx_pci_emu_dynamic_tbl[dev_num].num_funcs))
			ret = -1;
		else
			ret = bcm47xx_pci_emu_dynamic_tbl[dev_num].pci_emu_cfgs[func_num].write(bus_num, dev_num, func_num, where, size, val);
	}
	else
		ret = -1;

	spin_unlock_irqrestore(&bcm47xx_pci_emu_lock, flags);

	return (ret ? PCIBIOS_DEVICE_NOT_FOUND : PCIBIOS_SUCCESSFUL);
};

static struct pci_ops bcm47xx_pci_emu_ops = {
	.read	= bcm47xx_pci_readcfg,
	.write	= bcm47xx_pci_writecfg,
};

static int bcm47xx_pci_emu_setup_resources(struct resource **resource)
{
	/*
	 * bus->resource[0] is the IO resource for this bus
	 * bus->resource[1] is the mem resource for this bus
	 * bus->resource[2] is the prefetch mem resource for this bus
	 */
	resource[0] = &ioport_resource;
	resource[1] = &iomem_resource;
	resource[2] = NULL;

	return 1;
}



static int __init bcm47xx_pci_emu_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	u8 irq;

	pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &irq);
	dev->irq = irq;

	return ((int) dev->irq);
}

static int __init bcm47xx_pci_emu_setup(int nr, struct pci_sys_data *sys)
{
	int ret = 0;

	if (nr == 0) {
		ret = bcm47xx_pci_emu_setup_resources(sys->resource);
	}

	return ret;
}


struct pci_bus *bcm47xx_pci_emu_scan_bus(int nr, struct pci_sys_data *sys)
{
	return pci_scan_bus(sys->busnr, &bcm47xx_pci_emu_ops, sys);
}

static struct hw_pci bcm47xx_pci_emu __initdata = {
	.swizzle			= pci_std_swizzle,
	.map_irq			= bcm47xx_pci_emu_map_irq,
	.setup				= bcm47xx_pci_emu_setup,
	.nr_controllers		= 1,
	.scan				= bcm47xx_pci_emu_scan_bus,
	.preinit			= NULL,
	.postinit			= NULL,
};

static int __init get_bcm47xx_pci_ids(char *str_from_nvflash, unsigned int* primary_device)
{
	int id = 0;
	*primary_device = 0;
	if(str_from_nvflash == NULL)
	{
		/* set a invalid ID so that SD driver ignore this device */
		id = 0;
	}
	else if(strcmp(str_from_nvflash, "primary_sd") == 0)
	{
		/* primary boot device SD */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		id = PCI_DEVICE_ID_BCM_SD;
#else
		id = PCI_DEVICE_ID_BCM47XX_SD;
#endif
		*primary_device = 1;
	}
	else if(strcmp(str_from_nvflash, "secondary_sd") == 0)
	{
		/* secondry device SD*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		id = PCI_DEVICE_ID_BCM_SD;
#else
		id = PCI_DEVICE_ID_BCM47XX_SD;
#endif
	}
	else if(strcmp(str_from_nvflash, "tertiary_sd") == 0)
	{
		/* secondry device SD*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		id = PCI_DEVICE_ID_BCM_SD;
#else
		id = PCI_DEVICE_ID_BCM47XX_SD;
#endif
	}
	else
	{
		/* set a invalid ID so that SD driver ignore this device */
		id = 0;
		printk(KERN_ERR "Illegal nvflash value for SD/CSATA core - %s\n", str_from_nvflash);
	}

	return id;
}

/* create pci table absed on sd devcies in nvflash and static USB & IDE devices */
static void __init create_pci_tbl(void)
{
	int mmc_pci_ids[4] = {0,0,0,0};
	int n_sd_devices = 0;
	unsigned int primary_device = 0;
	int primary_device_index = 0;
	int dev_index = 0, temp_index = num_of_devs - 1;
	char *p;

#ifdef CONFIG_MMC
	char *sd_s1, *sd_s2, *sd_s3;

	/*read all four variables from hw_cfg */

	sd_s1 = HW_SD_B_S1;
	sd_s2 = HW_SD_B_S2;
	sd_s3 = HW_SD_B_S3;

	/*get the corresponding PCI ids for every string*/
	mmc_pci_ids[0] = sdio_s0_cfg[0].device;
//	mmc_pci_ids[0] = get_bcm47xx_pci_ids(sd_s1, &primary_device);
	if(mmc_pci_ids[0] != 0)
	{
		if (primary_device != 0)
		{
			primary_device_index = 0;
		}
		n_sd_devices++;
	}

	mmc_pci_ids[1] = sdio_s1_cfg[0].device;
//	mmc_pci_ids[1] = get_bcm47xx_pci_ids(sd_s2, &primary_device);
	if(mmc_pci_ids[1] != 0)
	{
		if(primary_device != 0)
		{
			if (primary_device_index != -1)
			{
				panic("Multiple SD boot devices specified in nvflash\n");
			}
			primary_device_index = 1;
		}
		n_sd_devices++;
	}

	mmc_pci_ids[2] = sdio_s2_cfg[0].device;
//	mmc_pci_ids[2] = get_bcm47xx_pci_ids(sd_s3, &primary_device);
	if(mmc_pci_ids[2] != 0)
	{
		if(primary_device != 0)
		{
			if (primary_device_index != -1)
			{
				panic("Multiple SD boot devices specified in nvflash\n");
			}
			primary_device_index = 2;
		}
		n_sd_devices++;
	}

	p = HW_PRIMARY_STORAGE_TYPE;
	//make sure "primary_sd" is specified if primary storage type is SD
	if (p && (strcmp(p, "SD") == 0) && (primary_device_index == -1))
	{
		panic("primary_storage_type is SD but missing primary_sd slot nvflash entry\n");
	}
#endif

	/* create new table because later in code we assume that first device with mmc (mmcblk0)
	   is the boot device; so based on nvflash entries we have to put device with "primary_sd"
	   as first mmc device */
	bcm47xx_pci_emu_dynamic_tbl =
		(pci_emu_dev_t *)kmalloc(sizeof(pci_emu_dev_t)*(num_of_devs + n_sd_devices), GFP_KERNEL);

	if(bcm47xx_pci_emu_dynamic_tbl == NULL)
	{
		panic("can't allocate pci table in pci_emu.c\n");
		num_of_dynamic_devs = -1;
		return;
	}

	/* copy USB devices first */
	for (dev_index = 0; dev_index < num_of_devs; dev_index++)
	{
		bcm47xx_pci_emu_dynamic_tbl[dev_index].dev_base		= bcm47xx_pci_emu_tbl[dev_index].dev_base;
		bcm47xx_pci_emu_dynamic_tbl[dev_index].init		    = bcm47xx_pci_emu_tbl[dev_index].init;
		bcm47xx_pci_emu_dynamic_tbl[dev_index].num_funcs	= bcm47xx_pci_emu_tbl[dev_index].num_funcs;
		bcm47xx_pci_emu_dynamic_tbl[dev_index].pci_emu_cfgs	= bcm47xx_pci_emu_tbl[dev_index].pci_emu_cfgs;
	}

	//"num_of_devs" index is reserved for primary SD boot device
	if(primary_device_index != -1)	dev_index++;

#if defined(CONFIG_MACH_BCM4760_TOMTOM)
	primary_device_index = bcm476x_get_boot_sdmmc_device();
	
	// set SD boot device (0) to low speed clock rate 
	if( primary_device_index )
	{
		set_mmc_max_speed(0, 25000000);
	}	
#endif
  
#ifdef CONFIG_MMC
	if(mmc_pci_ids[0])
	{
		/* SD Host Device, Slot 1 */
		//sdio_s0_cfg[0].device = mmc_pci_ids[0];
		temp_index = (primary_device_index == 0) ? num_of_devs : dev_index++;

		bcm47xx_pci_emu_dynamic_tbl[temp_index].dev_base	= SDM0_REG_BASE_ADDR;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].init	    	= sdio_init;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].num_funcs	= sizeof(sdio_s0_cfg)/sizeof(pci_emu_cfg_t);
		bcm47xx_pci_emu_dynamic_tbl[temp_index].pci_emu_cfgs	= sdio_s0_cfg;
	}

	if(mmc_pci_ids[1])
	{
		/* SD Host Device, Slot 2 */
		//sdio_s1_cfg[0].device = mmc_pci_ids[1];
		temp_index = (primary_device_index == 1) ? num_of_devs : dev_index++;

		bcm47xx_pci_emu_dynamic_tbl[temp_index].dev_base	= SDM1_REG_BASE_ADDR;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].init	    	= sdio_init;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].num_funcs	= sizeof(sdio_s1_cfg)/sizeof(pci_emu_cfg_t);
		bcm47xx_pci_emu_dynamic_tbl[temp_index].pci_emu_cfgs 	= sdio_s1_cfg;
	}

	if(mmc_pci_ids[2])
	{
		/* SD Host Device, Slot 3 */
		//sdio_s2_cfg[0].device = mmc_pci_ids[2];
		temp_index = (primary_device_index == 2) ? num_of_devs : dev_index++;

		bcm47xx_pci_emu_dynamic_tbl[temp_index].dev_base	= SDM2_REG_BASE_ADDR;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].init	    	= sdio_init;
		bcm47xx_pci_emu_dynamic_tbl[temp_index].num_funcs	= sizeof(sdio_s2_cfg)/sizeof(pci_emu_cfg_t);
		bcm47xx_pci_emu_dynamic_tbl[temp_index].pci_emu_cfgs 	= sdio_s2_cfg;
	}
#endif
	//error checking
#if !defined(CONFIG_MACH_BCM4760_TOMTOM)	
	if (temp_index != n_sd_devices + num_of_devs -1) {
		panic ("PCI devices don't match up\n");
	}
#endif	
	num_of_dynamic_devs = n_sd_devices + num_of_devs;
}

void bcm476x_set_pci_emu_devid (int sd, int devid)
{
	switch (sd) 
	{
		case BCM_SD0:
			sdio_s0_cfg[0].device = devid;
			break;
		case BCM_SD1:
			sdio_s1_cfg[0].device = devid;
			break;
		case BCM_SD2:
			sdio_s2_cfg[0].device = devid;
			break;
		default:
			printk ("Unsupported SD controller :%d\n", sd);
	};
}

static int __init bcm47xx_pci_emu_init(void)
{
	int dev_index;

	/* Initialize the bcm47xx emulated PCI bus lock  */
	spin_lock_init(&bcm47xx_pci_emu_lock);

	/* Populate IDs for SD/CEATA cores from nvflash  */
	create_pci_tbl();

	if((bcm47xx_pci_emu_dynamic_tbl == NULL) || (num_of_dynamic_devs == -1))
	{
		panic("NULL pointer for pci table\n");
	}
	/* Initialize the emulated PCI devices */
	for (dev_index = 0; dev_index < num_of_dynamic_devs; dev_index++){
		if (bcm47xx_pci_emu_dynamic_tbl[dev_index].init != NULL)
			bcm47xx_pci_emu_dynamic_tbl[dev_index].init(&bcm47xx_pci_emu_dynamic_tbl[dev_index]);
	}

#ifdef CONFIG_BCM47XX_SDIOB_S2_CLOCK_WORKAROUND

	/*
	 * If the first slot on the SDIOB is not used but we need to use the SDIOB slot2,
	 * we will go ahead enabling the clock on the slot1.
	 */
	if (!HW_SD_B_S1 && HW_SD_B_S2)
	{
		/* FIXME: These two macros are in some private header file of MMC driver directory,
		 * which is very device specific knowledge; but we do want to make this workaround
		 * in a higher level. So it is done this way...
		 */
#define SDHCI_CLOCK_CARD_EN 0x0004
#define SDHCI_CLOCK_CONTROL 0x2C
		writew(SDHCI_CLOCK_CARD_EN,  IO_ADDRESS(BCM47XX_SDIOB_ADDRBASE0 + SDHCI_CLOCK_CONTROL));
	}

#endif /* CONFIG_BCM47XX_SDIOB_S2_CLOCK_WORKAROUND */


	/* Register the PCI controller
	 */
	pci_common_init(&bcm47xx_pci_emu);

	return 0;
}

void sdhci_BCM_SD_init(void)
{
}

void sdhci_BCM_SD_exit(void)
{
}


unsigned long sdhci_BCM_SD_getBaseClk(int index)
{
	return BCM47XX_DEFAULT_BASE_CLOCK;
}
EXPORT_SYMBOL( sdhci_BCM_SD_getBaseClk );

#if defined(CONFIG_MACH_BCM4760_TOMTOM)	
unsigned long sdhci_BCM_SD_getMaxClk(int index)
{
	return local_mmc_clock[index & 0x3];
}
EXPORT_SYMBOL( sdhci_BCM_SD_getMaxClk );

static void set_mmc_max_speed( int mmc, unsigned long speed )
{
	if( mmc > ARRAY_SIZE( local_mmc_clock ) )
		return;
	local_mmc_clock[mmc] = speed;	
}

int bcm476x_get_boot_sdmmc_device(void)
{
	int v = readl(IO_ADDRESS(CMU_R_CMU_CTL0_MEMADDR));
	
	 /* Check which register is used to determine the boot device */
	if ((v | ~0xf) == ~0) {
		/* OPT_STRAP[10:11] is the reference */
		v = readl(CMU_R_OTP_STRAP_MEMADDR) >> 10;
	} /* Otherwise, CMU_CTL0[0:1] is the reference */
	
	
	return (v & 0x3);
}

EXPORT_SYMBOL(bcm476x_get_boot_sdmmc_device);
#endif

#define SDHCI1_TOP4_PINMUX_MASK (CMU_F_GPIO_18_MXSEL_MASK | CMU_F_GPIO_19_MXSEL_MASK | CMU_F_GPIO_20_MXSEL_MASK | CMU_F_GPIO_21_MXSEL_MASK)
#define SDHCI1_TOP4_PINMUX_VAL (( 1 << CMU_F_GPIO_18_MXSEL_R) | ( 1 << CMU_F_GPIO_19_MXSEL_R) | ( 1 << CMU_F_GPIO_20_MXSEL_R) | ( 1 << CMU_F_GPIO_21_MXSEL_R))

unsigned long sdhci_BCM_SD_getBusWidthEightCapable(int index)
{
	if (index == 0)
		return 1;
	else if (index == 2)
		return 0;
	else if ((index == 1) &&
	    ((readl(IO_ADDRESS(CMU_R_CHIP_PIN_MUX6_MEMADDR)) & SDHCI1_TOP4_PINMUX_MASK) == SDHCI1_TOP4_PINMUX_VAL))
		return 1;
	return 0;
}

subsys_initcall(bcm47xx_pci_emu_init);
