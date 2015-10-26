/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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
/**
 * 
 *   @file   bcm4760_gfx_pm.c 
 * 
 *   @brief  BCM4760 3D Graphics kernel power management driver.
 * 
 ****************************************************************************/

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/memory.h>
#include <asm/io.h>

#include <asm/arch/hardware.h>
#include <asm/arch/platform.h>

/*
 * Structure and macro definitions.
 */

#define GFX_MOD_DESCRIPTION	"BCM4760 Graphics PM Driver"
#define GFX_MOD_VERSION		1.0

#define ASM_STOPHERE    __asm("1: nop; b 1b")

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
#	define GFX_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define GFX_DEBUG(level,x)
#endif

struct gfx_register_save
{
	unsigned int	address;		/* Address of register to save/restore */
	unsigned int	or_value;		/* Value of register (ORed into actual register */
	unsigned int	mask;			/* Not mask for register, mask for OrValue */
};

/*
 * Local variables.
 */

static int 		gLevel = DBG_DEFAULT_LEVEL;

/* power management state save area */
static struct gfx_register_save gfx_state_save_regs[] =
{
	/* Graphics Frame registers */

	{ GFX_R_REND_LIST_ADDR_MEMADDR, 0, GFX_R_REND_LIST_ADDR_MASK },
	{ GFX_R_REND_RSW_BASE_MEMADDR, 0, GFX_R_REND_RSW_BASE_MASK },
	{ GFX_R_REND_VERTEX_BASE_MEMADDR, 0, GFX_R_REND_VERTEX_BASE_MASK },

	{ GFX_R_FRAMEBUFF_FORMAT_MEMADDR, 0, GFX_R_STENCIL_CLEAR_VALUE_MASK },

	{ GFX_R_RGB_CLEAR_VALUE_MEMADDR, 0, GFX_R_RGB_CLEAR_VALUE_MASK },
	{ GFX_R_Z_CLEAR_VALUE_MEMADDR, 0, GFX_R_Z_CLEAR_VALUE_MASK },
	{ GFX_R_STENCIL_CLEAR_VALUE_MEMADDR, 0, GFX_R_STENCIL_CLEAR_VALUE_MASK },

	{ GFX_R_FEATURE_ENABLE_MEMADDR, 0, GFX_R_FEATURE_ENABLE_MASK },

	{ GFX_R_Z_WRITE_ENABLE_MEMADDR, 0, GFX_R_Z_WRITE_ENABLE_MASK },
	{ GFX_R_Z_WRITE_OFFSET_MEMADDR, 0, GFX_R_Z_WRITE_OFFSET_MASK },

	{ GFX_R_STENCIL_WRITE_ENABLE_MEMADDR, 0, GFX_R_STENCIL_WRITE_ENABLE_MASK },
	{ GFX_R_STENCIL_WRITE_OFFS_MEMADDR, 0, GFX_R_STENCIL_WRITE_OFFS_MASK },

	{ GFX_R_RGB_READ_IN_ENABLE_MEMADDR, 0, GFX_R_RGB_READ_IN_ENABLE_MASK },
	{ GFX_R_RGB_READ_IN_OFFS_MEMADDR, 0, GFX_R_RGB_READ_IN_OFFS_MASK },

	{ GFX_R_HIGH_CLAMP_S_MEMADDR, 0, GFX_R_HIGH_CLAMP_S_MASK },
	{ GFX_R_HIGH_CLAMP_T_MEMADDR, 0, GFX_R_HIGH_CLAMP_T_MASK },

	/* Graphics Management registers */

	{ GFX_R_INT_MASK_MEMADDR, 0, GFX_R_INT_MASK_MASK },

	{ GFX_R_WRITE_BOUNDARY_ENABLE_MEMADDR, 0, GFX_R_WRITE_BOUNDARY_ENABLE_MASK },
	{ GFX_R_WRITE_BOUNDARY_LOW_MEMADDR, 0, GFX_R_WRITE_BOUNDARY_LOW_MASK },
	{ GFX_R_WRITE_BOUNDARY_HIGH_MEMADDR, 0, GFX_R_WRITE_BOUNDARY_HIGH_MASK },

	{ GFX_R_PERF_CNT_0_ENABLE_MEMADDR, 0, GFX_R_PERF_CNT_0_ENABLE_MASK },
	{ GFX_R_PERF_CNT_0_SRC_MEMADDR, 0, GFX_R_PERF_CNT_0_SRC_MASK },
	{ GFX_R_PERF_CNT_0_LIMIT_MEMADDR, 0, GFX_R_PERF_CNT_0_LIMIT_MASK },
	{ GFX_R_PERF_CNT_0_VALUE_MEMADDR, 0, GFX_R_PERF_CNT_0_VALUE_MASK },

	{ GFX_R_PERF_CNT_1_ENABLE_MEMADDR, 0, GFX_R_PERF_CNT_1_ENABLE_MASK },
	{ GFX_R_PERF_CNT_1_SRC_MEMADDR, 0, GFX_R_PERF_CNT_1_SRC_MASK },
	{ GFX_R_PERF_CNT_1_LIMIT_MEMADDR, 0, GFX_R_PERF_CNT_1_LIMIT_MASK },
	{ GFX_R_PERF_CNT_1_VALUE_MEMADDR, 0, GFX_R_PERF_CNT_1_VALUE_MASK },

	/* Graphics MMU registers */

	{ GFX_R_MMU_DTE_ADDR_MEMADDR, 0, GFX_R_MMU_DTE_ADDR_MASK },

	{ GFX_R_MMU_INT_MASK_MEMADDR, 0, GFX_R_MMU_INT_MASK_MASK },
};

/*
 * Function definitions.
 */

/** @brief bcm4760_gfx_probe
 *
 * This probe function is pretty simplistic for this driver. It is
 * here partially as a place holder for functionality we may need.
 * Also provides some indications that installation has happened ok.
 */
static int bcm4760_gfx_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource 0\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IRQ resource 0\n");
		return -ENOENT;
	}

	return 0;
}

/** @brief bcm4760_gfx_remove
 *
 * This remove function is pretty simplistic for this driver. It doesn't
 * do anything for now.
 */
static int bcm4760_gfx_remove(struct platform_device *pdev)
{
	GFX_DEBUG(DBG_INFO, (KERN_INFO "BCM4760 Graphics PM driver removed.")); 

	return 0;
}

#ifdef CONFIG_PM

/** @brief bcm4760_gfx_suspend
 *
 * The suspend function basically saves all read/write configuration registers
 * for the graphics core into a RAM based structure.
 */
static int bcm4760_gfx_suspend(struct device *dev)
{
	unsigned int i;
	struct platform_device *pdev;

	pdev = to_platform_device(dev);
	GFX_DEBUG(DBG_TRACE, (KERN_INFO "Suspend processed for %s.\n", pdev->name)); 

	for (i=0 ; i<(sizeof(gfx_state_save_regs) / sizeof(gfx_state_save_regs[0])) ; i++)
	{
		gfx_state_save_regs[i].or_value = readl(IO_ADDRESS(gfx_state_save_regs[i].address)) & gfx_state_save_regs[i].mask;
	}

	return 0;
} /* bcm4760_gfx_suspend */

/** @brief bcm4760_gfx_resume
 *
 * The resume function restores the graphics core read/write configuration
 * registers from the RAM based structure they were stored into by the suspend
 * function. It then writes the start command to the MMU command register. This
 * register is write-only but we only need to write 0 (zero) to it to get it
 * running again based upon the pointer to the table we restored from the RAM
 * data structure. The interrupt mask register for the MMU interrupt was also
 * saved and restored. The MMU will eventually reload everything else from the
 * RAM based structures itself so we don't need to do that here.
 */
static int bcm4760_gfx_resume(struct device *dev)
{
	struct platform_device *pdev;
	unsigned int	i,j;

	pdev = to_platform_device(dev);
	GFX_DEBUG(DBG_TRACE, ("Resume processed for %s.\n", pdev->name)); 

	for (i=0 ; i<(sizeof(gfx_state_save_regs) / sizeof(gfx_state_save_regs[0])) ; i++)
	{
		j = readl(IO_ADDRESS(gfx_state_save_regs[i].address)) & ~gfx_state_save_regs[i].mask;
		writel(j | gfx_state_save_regs[i].or_value,
			   IO_ADDRESS(gfx_state_save_regs[i].address));
	}

	/* Turn MMU back on */

	writel(0, IO_ADDRESS(GFX_R_MMU_COMMAND_MEMADDR));

	return 0;
} /* bcm4760_gfx_resume */

#else
#define bcm4760_gfx_suspend NULL
#define bcm4760_gfx_resume NULL
#endif

static struct pm_ext_ops bcm4760_gfx_pm_ops =
{
	.base =
	{
		.suspend	= bcm4760_gfx_suspend,
		.resume		= bcm4760_gfx_resume
	}
};

static struct platform_driver bcm4760_gfx_driver = 
{
	.probe		= bcm4760_gfx_probe,
	.remove		= bcm4760_gfx_remove,
	.pm		= &bcm4760_gfx_pm_ops,
	.driver		= {
		.name	= "bcm4760_gfx_pm",
		.owner	= THIS_MODULE,
	},
 };

/****
Args : None.

Returns : 0 on success or -ve number.
****/
static int __init bcm4760_gfx_modinit(void)
{
	int ret;

	/* Register as a platform driver */
	ret = platform_driver_register(&bcm4760_gfx_driver);

	if (ret) {
		GFX_DEBUG(DBG_ERROR, (KERN_ERR "BCM4760 Graphics PM driver failed to register!")); 
	}

	return ret;
}

/*
 * Module exit function
 */
static void bcm4760_gfx_modexit( void )
{
	platform_driver_unregister(&bcm4760_gfx_driver);
}

module_init(bcm4760_gfx_modinit);
module_exit(bcm4760_gfx_modexit);

MODULE_DESCRIPTION(GFX_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(GFX_MOD_VERSION);
