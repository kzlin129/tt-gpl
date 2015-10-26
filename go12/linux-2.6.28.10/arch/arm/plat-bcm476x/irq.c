/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  Copyright (C) 1999 ARM Limited
 */

#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>
#include <asm/arch/reg_pwrseq.h>

/*
 * Structure and macro definitions.
 */

#define VIC_MOD_DESCRIPTION	"BCM4760 VIC Driver"
#define VIC_MOD_VERSION		2.1

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
#define DBG_DEFAULT_LEVEL	DBG_INFO | DBG_TRACE

#if DEBUG
#	define VIC_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define VIC_DEBUG(level,x)
#endif

/*
 * @brief bcm4760 VIC driver suspend/resume state save structure 
 *
 * There is one of these structures per VIC controller in the system
 * which the suspend code will initialize with the current state of the
 * VIC controller. The resume code will of course restore these values
 * along with any other fixed programming that must be done to re-enable
 * the VIC controller to operational status after resuming from the
 * power managed state.
 */  
struct bcm4760_vic_data
{
	uint32_t	base;					/* base address of this VIC block */
	uint32_t	int_select;				/* IRQ/FIQ interrupt selection */
	uint32_t	int_enable;				/* interrupts enabled (unmasked) */
	uint32_t	protection;				/* user mode protection enable */
	uint32_t	sw_priority_mask;		/* sw priority masking */
	uint32_t	vector_addresses[32];	/* 32 interrupt vector addresses */
	uint32_t	vector_priority[32];	/* 32 interrupt priorities */
	uint32_t	vector_priority_daisy;	/* priority of daisy chain interrupt */
};

/*
 * Local variables.
 */

static int 		gLevel = DBG_DEFAULT_LEVEL;

/*
 * Function definitions.
 */

static void bcm4760_vic_mask_irq0(unsigned int irq)
{
	writel(1 << (irq - IRQ_INTC0_START), IO_ADDRESS(VIC0_R_VICINTENCLEAR_MEMADDR));
}
static void bcm4760_vic_unmask_irq0(unsigned int irq)
{
	writel(1 << (irq - IRQ_INTC0_START), IO_ADDRESS(VIC0_R_VICINTENABLE_MEMADDR));
}
static void bcm4760_vic_mask_irq1(unsigned int irq)
{
	writel(1 << (irq - IRQ_INTC1_START), IO_ADDRESS(VIC1_R_VICINTENCLEAR_MEMADDR));
}
static void bcm4760_vic_unmask_irq1(unsigned int irq)
{
	writel(1 << (irq - IRQ_INTC1_START), IO_ADDRESS(VIC1_R_VICINTENABLE_MEMADDR));
}

/****************************************************************************
*
*  bcm4760_vic_set_wake
*
*     Enables or disables wake from suspend triggering for the specified GPIO.
*
***************************************************************************/

static int bcm4760_vic_set_wake( unsigned irq, unsigned on )
{
	uint32_t	temp,on_off_mask;
	
	if ( on )
	{
		on_off_mask = 0;
	}
	else
	{
		on_off_mask = 0xFFFFFFFF;
	}

	if (irq == BCM4760_INTR_RTC_MTCH)
	{
		temp = readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
		temp &= ~PML_WAKEUP_MASK1_RTC_MATCHED;
		temp |= (PML_WAKEUP_MASK1_RTC_MATCHED & on_off_mask);
		writel(temp, IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
	}
	else if (irq == BCM4760_INTR_RTC)
	{
		temp = readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
		temp &= ~PML_WAKEUP_MASK1_RTC_INT;
		temp |= (PML_WAKEUP_MASK1_RTC_INT & on_off_mask);
		writel(temp, IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
	}
	else if (irq == BCM4760_INTR_TIM2_CNTR1)
	{
		temp = readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
		temp &= ~PML_WAKEUP_MASK1_TIM2_COUNTER1;
		temp |= (PML_WAKEUP_MASK1_TIM2_COUNTER1 & on_off_mask);
		writel(temp, IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
	}
	else if (irq == BCM4760_INTR_TIM2_CNTR2)
	{
		temp = readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
		temp &= ~PML_WAKEUP_MASK1_TIM2_COUNTER2;
		temp |= (PML_WAKEUP_MASK1_TIM2_COUNTER2 & on_off_mask);
		writel(temp, IO_ADDRESS(PML_R_PML_WAKEUP_MASK1_MEMADDR));
	}
	else
	{
		return -EINVAL;
	}

	return 0;
} // bcm4760_vic_set_wake


static struct irq_chip bcm4760_irq0_chip = {
	.typename	= "BCM4760_VIC0",
	.ack		= bcm4760_vic_mask_irq0,
	.mask		= bcm4760_vic_mask_irq0,     /* mask a specific interrupt, blocking its delivery. */
	.unmask		= bcm4760_vic_unmask_irq0, /* unmaks an interrupt */
	.set_wake	= bcm4760_vic_set_wake,
};

static struct irq_chip bcm4760_irq1_chip = {
   .typename	= "BCM4760_VIC1",
   .ack			= bcm4760_vic_mask_irq1,
   .mask		= bcm4760_vic_mask_irq1,     
   .unmask		= bcm4760_vic_unmask_irq1, 
   .set_wake	= bcm4760_vic_set_wake,
};


static void vic_init(void __iomem *base, struct irq_chip *chip, unsigned int irq_start, unsigned int vic_sources)
{
    unsigned int i;
    for ( i = 0; i < 31; i++ )
    {
        unsigned int irq = irq_start + i;
        set_irq_chip( irq, chip );
		set_irq_chip_data(irq, base);
        
		if (vic_sources & (1 << i)) 
        {
            set_irq_handler( irq, handle_level_irq );
            set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
        }
    }
	/* Disable all interrupts initially. */

    writel(0, base + VIC0_R_VICINTSELECT_SEL);
    writel(0, base + VIC0_R_VICINTENABLE_SEL);
    writel(~0, base + VIC0_R_VICINTENCLEAR_SEL);
    writel(0, base + VIC0_R_VICIRQSTATUS_SEL);
    writel(~0, base + VIC0_R_VICSOFTINTCLEAR_SEL);
	/*
	 * Make sure we clear all existing interrupts
	 */
	writel(0, base + VIC0_R_VICADDRESS_SEL);
	for (i = 0; i < 31; i++) {
		unsigned int value;
		value = readl(base + VIC0_R_VICADDRESS_SEL);
		writel(value, base + VIC0_R_VICADDRESS_SEL);
	}

#if 0

   // FUTURE - we may want to enable the auto vector
   // functionality some day and this means that we
   // could write an interrupt number into the address
   // registers, and get_irqnr_and_base could use
   // that information.

	/*
	 * Make sure we clear all existing interrupts
	 */
	writel(0, base + VIC0_R_ADDRESS_SEL);
	for (i = 0; i < 36; i++) 
   {
		unsigned int value = readl(base + VIC0_R_ADDRESS_SEL);
		writel(value, base + VIC0_R_ADDRESS_SEL);
	}

   // Instead of vector addresses, store the irq number (from 0) in the address 
   // registers, and when an irq happens, read the number from the sole address
   // register that tells us the source of the interrupt. This avoids complicated
   // code in the low level assember macro get_irqnr_and base.
	for (i = 0; i < 32; i++) 
   {
		void __iomem *reg = base + VIC0_R_VICVECTADDR0_SEL + (i * 4);
		writel(i, reg);
	}
#endif
}


/* @brief bcm4760_vic_probe
 *
 * This probe function is pretty simplistic for this driver. It is
 * here partially as a place holder for functionality we may need.
 * Also provides some indications that installation has happened ok.
 */
static int bcm4760_vic_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct bcm4760_vic_data *vic_driver_data;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL)
    {
        dev_err(&pdev->dev, "cannot find IO resource 0\n");
        return -ENOENT;
    }

	vic_driver_data = kmalloc(sizeof(*vic_driver_data), GFP_KERNEL);
    if (vic_driver_data == NULL)
    {
        dev_err(&pdev->dev, "Failed to obtain vic_driver_data structure!\n");
        return -ENOENT;
    }
	
    platform_set_drvdata(pdev, vic_driver_data);
	vic_driver_data->base = IO_ADDRESS( res->start );

    return 0;
}

/* @brief bcm4760_vic_remove
 *
 * This remove function is pretty simplistic for this driver. It doesn't
 * do anything for now but free up allocated memory.
 */
static int bcm4760_vic_remove(struct platform_device *pdev)
{
    kfree(platform_get_drvdata(pdev));

    platform_set_drvdata(pdev, NULL);   /* mark driver private data as gone */
	
	VIC_DEBUG(DBG_INFO, (KERN_INFO "BCM4760 VIC driver removed.")); 

	return 0;
}

#ifdef CONFIG_PM

static int bcm4760_vic_suspend(struct device *dev)
{
	uint32_t				i,temp;
	struct platform_device	*pdev;
    struct bcm4760_vic_data *vic_driver_data;

	pdev = to_platform_device(dev);
    vic_driver_data = platform_get_drvdata(pdev);

	VIC_DEBUG(DBG_TRACE, (KERN_INFO "Suspend processed for %s.\n", pdev->name)); 

	vic_driver_data->int_select = readl(vic_driver_data->base + VIC0_R_VICINTSELECT_SEL);
	vic_driver_data->int_enable = readl(vic_driver_data->base + VIC0_R_VICINTENABLE_SEL);
	vic_driver_data->protection = readl(vic_driver_data->base + VIC0_R_VICPROTECTION_SEL);
	vic_driver_data->sw_priority_mask = readl(vic_driver_data->base + VIC0_R_VICSWPRIORITYMASK_SEL);
	vic_driver_data->vector_priority_daisy = readl(vic_driver_data->base + VIC0_R_VICPRIORITYDAISY_SEL);

	for (i=0 ; i<32 ; i++)
	{
		temp = readl(vic_driver_data->base + VIC0_R_VICVECTADDR0_SEL + (i*4));
		vic_driver_data->vector_addresses[i] = temp;
		temp = readl(vic_driver_data->base + VIC0_R_VICVECTPRIORITY0_SEL + (i*4));
		vic_driver_data->vector_priority[i] = temp;
	}

	return 0;
} /* bcm4760_vic_suspend */

static int bcm4760_vic_resume(struct device *dev)
{
	unsigned int			i;
	struct platform_device	*pdev;
    struct bcm4760_vic_data *vic_driver_data;

	pdev = to_platform_device(dev);
    vic_driver_data = platform_get_drvdata(pdev);

	VIC_DEBUG(DBG_TRACE, ("Resume processed for %s.\n", pdev->name)); 

	writel(vic_driver_data->int_select, vic_driver_data->base + VIC0_R_VICINTSELECT_SEL);
	writel(vic_driver_data->int_enable, vic_driver_data->base + VIC0_R_VICINTENABLE_SEL);
	writel(vic_driver_data->protection, vic_driver_data->base + VIC0_R_VICPROTECTION_SEL);
	writel(vic_driver_data->sw_priority_mask, vic_driver_data->base + VIC0_R_VICSWPRIORITYMASK_SEL);
	writel(vic_driver_data->vector_priority_daisy, vic_driver_data->base + VIC0_R_VICPRIORITYDAISY_SEL);

	for (i=0 ; i<32 ; i++)
	{
		writel(vic_driver_data->vector_addresses[i],
			   vic_driver_data->base + VIC0_R_VICVECTADDR0_SEL + (i*4));
		writel(vic_driver_data->vector_priority[i],
			   vic_driver_data->base + VIC0_R_VICVECTPRIORITY0_SEL + (i*4));
	}

	return 0;
} /* bcm4760_vic_resume */

#else
#define bcm4760_vic_suspend NULL
#define bcm4760_vic_resume NULL
#endif

static struct pm_ext_ops bcm4760_vic_pm_ops =
{
	.base =
	{
		.suspend	= bcm4760_vic_suspend,
		.resume		= bcm4760_vic_resume
	}
};

static struct platform_driver bcm4760_vic0_driver = {
	.probe		= bcm4760_vic_probe,
	.remove		= bcm4760_vic_remove,
	.pm			= &bcm4760_vic_pm_ops,
	.driver		= {
		.name		= "bcm4760_vic0",
		.owner		= THIS_MODULE,
	},
 };

static struct platform_driver bcm4760_vic1_driver = {
	.probe		= bcm4760_vic_probe,
	.remove		= bcm4760_vic_remove,
	.pm			= &bcm4760_vic_pm_ops,
	.driver		= {
		.name		= "bcm4760_vic1",
		.owner		= THIS_MODULE,
	},
 };

void __init bcm476x_init_irq( void )
{
    vic_init((void __iomem *)IO_ADDRESS(VIC0_REG_BASE_ADDR), &bcm4760_irq0_chip, IRQ_INTC0_START, IRQ_INTC0_VALID_MASK);
    vic_init((void __iomem *)IO_ADDRESS(VIC1_REG_BASE_ADDR), &bcm4760_irq1_chip, IRQ_INTC1_START, IRQ_INTC1_VALID_MASK);
}

/****
Args : None.

Returns : 0 on success or -ve number.
****/
static int __init bcm4760_vic_modinit(void)
{
    int ret;

    ret = platform_driver_register(&bcm4760_vic0_driver);
    if (ret == 0)
    {
        ret = platform_driver_register(&bcm4760_vic1_driver);
        if (ret)
        {
            platform_driver_unregister(&bcm4760_vic0_driver);
        }
    }

	if (ret)
		VIC_DEBUG(DBG_ERROR, (KERN_ERR "BCM4760 VIC driver failed to register!")); 

	return ret;
}

/*
 * Module exit function
 */
static void bcm4760_vic_modexit( void )
{
	platform_driver_unregister(&bcm4760_vic0_driver);
	platform_driver_unregister(&bcm4760_vic1_driver);
}

module_init(bcm4760_vic_modinit);
module_exit(bcm4760_vic_modexit);

MODULE_DESCRIPTION(VIC_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(VIC_MOD_VERSION);
