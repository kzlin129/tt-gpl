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
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/broadcom/bcm_sysctl.h>

#include <asm/arch/hw_timer.h>
#include <asm/arch/rtcHw.h>
#include <asm/arch/spi.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>

#include <plat/fdt.h>
#include <plat/irvine-flipflop.h>
#include <plat/irvine-lcd.h>
#include <plat/irvine.h>

HW_DECLARE_SPINLOCK(Arch)

/* sysctl */
int gArchHaltOnOops = 0; // reboot on oops by default
EXPORT_SYMBOL(gArchHaltOnOops);

static struct ctl_table_header *gSysCtlHeader;

static struct ctl_table gSysCtlOops[] = {
   {
	  .ctl_name	= BCM_SYSCTL_REBOOT_WARM,
	  .procname	= "halt",
	  .data		= &gArchHaltOnOops,
	  .maxlen	= sizeof( int ),
	  .mode		= 0644,
	  .proc_handler = &proc_dointvec
   },
   {}
};

static struct ctl_table gSysCtl[] = 
{
   {
	  .ctl_name	= CTL_BCM_REBOOT,
	  .procname	= "oops",
	  .mode		= 0555,
	  .child	= gSysCtlOops
   },
   {}
};

extern void __init bcm476x_amba_init(void);
extern int __init bcm4760_clk_init(void);

static struct resource rtc0_resources[] = 
{
	[e_irq_one_shot] = 
	{
		.start  = BCM4760_INTR_RTC_MTCH,
		.end	= BCM4760_INTR_RTC_MTCH,
		.flags  = IORESOURCE_IRQ,
	},
	[e_irq_periodic] = 
	{
		.start  = BCM4760_INTR_RTC,
		.end	= BCM4760_INTR_RTC,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device bcm476x_device_rtc0 = 
{
	.name 		= "bcm476x-rtc0",
	.id 		= -1,
	.resource	= rtc0_resources,
	.num_resources  = ARRAY_SIZE(rtc0_resources),
};

static struct platform_device bcm59040_device_rtc0 = 
{
	.name 	= "bcm59040_476x-rtc0",
	.id 	= -1,
};

static struct resource bcm476x_resources_wdt[] = 
{
	{
		.start  = BCM4760_INTR_WDT,
		.end	= BCM4760_INTR_WDT,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_wdt = 
{
	.name 		= "bcm4760-wdt",
	.id 		= -1,
	.resource	= bcm476x_resources_wdt,
	.num_resources  = ARRAY_SIZE(bcm476x_resources_wdt),
};

static struct resource bcm476x_resources_adc[] = 
{
	{
		.start  = BCM4760_INTR_AXA,
		.end	= BCM4760_INTR_AXA,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_adc = 
{
	.name 		= "bcm4760-adc",
	.id 		= -1,
	.resource	= bcm476x_resources_adc,
	.num_resources  = ARRAY_SIZE(bcm476x_resources_adc),
};

static struct platform_device bcm476x_device_rng = 
{
	.name 		= "bcm476x-rng",
	.id 		= -1,
};
/* I2C devices */

static struct resource bcm4760_i2c0_resource[] = 
{
	[0] = 
	{
		.start 	= I2C0_REG_BASE_ADDR,
		.end   	= (I2C0_REG_BASE_ADDR + SZ_4K)-1,
		.flags 	= IORESOURCE_MEM,
	},
	[1] = 
	{
		.start 	= BCM4760_INTR_I2C0,
		.end   	= BCM4760_INTR_I2C0,
		.flags 	= IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_i2c0 = 
{
	.name		= "bcm4760-i2c-0",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_i2c0_resource),
	.resource	= bcm4760_i2c0_resource,
};

static struct resource bcm4760_i2c1_resource[] = 
{
	[0] = 
	{
		.start 	= I2C1_REG_BASE_ADDR,
		.end   	= (I2C1_REG_BASE_ADDR + SZ_4K)-1,
		.flags 	= IORESOURCE_MEM,
	},
	[1] = 
	{
		.start 	= BCM4760_INTR_I2C1,
		.end   	= BCM4760_INTR_I2C1,
		.flags 	= IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_i2c1 = 
{
	.name		= "bcm4760-i2c-1",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_i2c1_resource),
	.resource	= bcm4760_i2c1_resource,
};

/* GPIO device */
static struct resource bcm4760_gpio_resource[] = 
{
	[0] = {
		.start 	= GIO0_REG_BASE_ADDR,
		.end   	= (GIO0_REG_BASE_ADDR + SZ_4K)-1,
		.flags 	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= GIO1_REG_BASE_ADDR,
		.end   	= (GIO1_REG_BASE_ADDR + SZ_4K)-1,
		.flags 	= IORESOURCE_MEM,
	},
	[2] = {
		.start 	= BCM4760_INTR_GPIO00,
		.end   	= BCM4760_INTR_GPIO12,
		.flags 	= IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_gpio = 
{
	.name		= "bcm4760-gpio",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_gpio_resource),
	.resource	= bcm4760_gpio_resource,
};

/* PWM device */
static struct platform_device bcm_pwm_device = 
{
	.name   = "bcm4760-pwm",
};


static struct platform_device bcm476x_device_pwrseq = 
{
	.name 	= "bcm4760_pwrseq",
	.id   	= -1
};

#ifdef CONFIG_BCM476X_SPI
static struct bcm476x_spi_master spi_bcm476x_master_platform_info_0 = 
{
	.num_chipselect = 4
};

static struct bcm476x_spi_master spi_bcm476x_master_platform_info_1 = 
{
	.num_chipselect = 4
};

static struct platform_device spi_bcm476x_master_device_0 = {
	.name 	= "spi",	
	.id 	= 0, 		/* Bus number */
	.dev 	= {
		.platform_data = &spi_bcm476x_master_platform_info_0, /* Passed to driver */
	}
};

static struct platform_device spi_bcm476x_master_device_1 = {
	.name 	= "spi",
	.id 	= 1, 		/* Bus number */
	.dev 	= {
		.platform_data = &spi_bcm476x_master_platform_info_1, /* Passed to driver */
	}
};
#endif //CONFIG_BCM476X_SPI

/* 3D Graphics device */

static struct resource bcm4760_gfx_resources[] = 
{
	[0] = {
		.start 	= GFX_REG_BASE_ADDR,
		.end 	= (GFX_REG_BASE_ADDR + (4 * SZ_4K))-1,
		.flags 	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= BCM4760_INTR_GFX_CORE,
		.end   	= BCM4760_INTR_GFX_MMU,
		.flags 	= IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476x_device_gfx = 
{
	.name		= "bcm4760_gfx_pm",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_gfx_resources),
	.resource	= bcm4760_gfx_resources,
};

/* BCM4760 Vectored Interrupt Controller devices */
static struct resource bcm4760_vic0_resources[] = 
{
	[0] = {
		.start 	= VIC0_REG_BASE_ADDR,
		.end   	= (VIC0_REG_BASE_ADDR + SZ_4K)-1,
		.flags 	= IORESOURCE_MEM,
	},
};

static struct platform_device bcm476x_device_vic0 = 
{
	.name		= "bcm4760_vic0",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_vic0_resources),
	.resource	= bcm4760_vic0_resources,
};

static struct resource bcm4760_vic1_resources[] = 
{
	[0] = {
		.start = VIC1_REG_BASE_ADDR,
		.end   = (VIC1_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device bcm476x_device_vic1 = 
{
	.name		= "bcm4760_vic1",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(bcm4760_vic1_resources),
	.resource	= bcm4760_vic1_resources,
};

static struct platform_device bcm476x_device_spu = 
{
	.name 		= "bcm476x-spu",
	.id   		= -1
};

static struct platform_device apm_emulation = {
	.name 		= "apm_emulation",
	.id 		= -1,
};

static struct platform_device *irvine_devices[] __initdata = 
{
#ifndef CONFIG_BCM4760_FLASHER
	&tomtom_device_libfdt,	/* TomTom Flattened Device Tree. */
#endif

	&bcm476x_device_vic0,	/* always need vectored interrupt controllers */
	&bcm476x_device_vic1,	/* and probably earlier than later */
	&bcm476x_device_gpio,	/* no reason not to setup GPIOs early */
	&bcm476x_device_i2c0,	/* I2C must be early up in init order! */
	&bcm476x_device_i2c1,	/* I2C must be early up in init order! */
	&bcm59040_device_rtc0,
	&bcm476x_device_rtc0,

	&bcm476x_device_pwrseq,	/* Should be low in init order! */

	&bcm476x_device_wdt,
	&bcm476x_device_adc,

#ifdef CONFIG_BACKLIGHT_PWM
	&bcm_pwm_device,
#endif
	&bcm476x_device_rng,
	&bcm476x_device_spu,

	&bcm476x_device_gfx,	/* Only for power mangement, pretty low in init/resume ordering */
#ifdef CONFIG_TOMTOM_FLIPFLOP
	&irvine_device_flipflop,
#endif
	&tomtom_device_ttsetup,
	&apm_emulation,
};

static void irvine_pm_power_off(void)
{
	/* Add a delay for movinands. */
	mdelay( 500 );

	gpio_direction_output(TT_VGPIO_KILL_PWRn, 0);
}

void __init irvine_init_machine (void)
{
	pm_power_off  = irvine_pm_power_off;
	gSysCtlHeader = register_sysctl_table(gSysCtl);

	platform_add_devices(irvine_devices, ARRAY_SIZE(irvine_devices));

	bcm4760_clk_init() ;
	bcm476x_amba_init();
}

/****************************************************************************
*
*   Timer related information. sys_timer_init is called from
*   time_init (in arch/arm/kernel/time.c).
*
*****************************************************************************/

struct sys_timer bcm476x_timer =
{
	.init	= sys_timer_init,
#ifndef CONFIG_GENERIC_TIME
	.offset	= sys_timer_gettimeoffset,
#endif
};

