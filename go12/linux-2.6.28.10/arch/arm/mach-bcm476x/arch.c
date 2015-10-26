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

#include <linux/autoconf.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/bcm_adc.h>

#if defined(CONFIG_BCM_PMU_BCM59040)
	#include <linux/broadcom/pmu_chip.h>
	#include <linux/regulator/bcm59040-regulators.h>
	#include <linux/broadcom/pmu_bcm59040.h>
	#include <linux/regulator/bcm59040-regulators.h>
#else
	#if defined(CONFIG_PMU_DEVICE_BCM59040)
		#include <linux/pmu_device.h>
		#include <asm/arch/pmu_device_bcm59040.h>
		#include <linux/broadcom/pmu_bcm59040.h>
		#include <linux/regulator/bcm59040-regulators.h>
	#endif
	#if defined(CONFIG_PMU_DEVICE_BCM59002)
		#include <linux/pmu_device.h>
		#include <asm/arch/pmu_device_bcm59002.h>
		#include <linux/broadcom/pmu_bcm59002.h>
		#include <linux/regulator/bcm59002-regulators.h>
	#endif
#endif

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>

#include <asm/mach/arch.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <asm/arch/hw_cfg.h>

#ifdef CONFIG_SPI
#include <linux/spi/spi.h>
#endif

#ifdef CONFIG_BCM476X_SPI
#include <asm/arch/spi.h>
#endif

#ifdef CONFIG_MTD_M25P80
#include <linux/spi/flash.h>
#endif

#ifdef CONFIG_TOMTOM_FLIPFLOP
#include "tomtom_flipflop.h"
#endif

#include <asm/clock_fw.h>
#include <asm/arch/clock_bcm4760.h>
#include <asm/arch/hw_timer.h>
#include <asm/arch/rtcHw.h>

#include <asm/arch/bcm_gen_battery.h>
#include <asm/arch/bcm59040.h>
#include <linux/vgpio.h>
#include <linux/broadcom/ts.h>

#ifdef CONFIG_KEYBOARD_BCM4760	// @KP: 091209: added
#include <linux/input.h>
#include <linux/bcm_gpio_keypad.h>
#endif

HW_DECLARE_SPINLOCK(Arch)

struct bcm_ll_adc_req {
    bcm_adc_request_t	adc;			/* ADC request structure for request function */
    unsigned short		sample;			/* sample response from ADC integration */
    struct completion	cmp;			/* completion structure for this request */
};	

/* sysctl */
int gArchHaltOnOops = 0; // reboot on oops by default
EXPORT_SYMBOL(gArchHaltOnOops);

static struct ctl_table_header *gSysCtlHeader;

static struct ctl_table gSysCtlOops[] = {
   {
      .ctl_name      = BCM_SYSCTL_REBOOT_WARM,
      .procname      = "halt",
      .data          = &gArchHaltOnOops,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_REBOOT,
      .procname      = "oops",
      .mode          = 0555,
      .child         = gSysCtlOops
   },
   {}
};

extern void bcm476x_map_io(void);
extern void bcm476x_init_irq(void);
extern void __init bcm476x_amba_init(void);
extern int __init bcm4760_clk_init(void);

#include <plat/irvine.h>

static struct vgpio_pin bcm476x_vgpio0_pins[] = {
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_LCD_BACKLIGHT_PWM,		13),	/* GUBA replace numbers with macros out of bcm4760_reg.h */
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_LCD_BACKLIGHT_ENABLE, 	12),
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_LCD_ON,		 	14),
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_PMU_IRQ,			HW_GPIO_PMU_IRQ_PIN),
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_WL_RST,			4),
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_REG_ON,			5),
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_BT_RST,			6),
#if 0
	VGPIO_DEF_PIN(TT_VGPIO_BASE, TT_VGPIO_INTR_USB,		BCM4760_INTR_USB),
#endif
};

#endif

static struct vgpio_platform_data bcm476x_vgpio_pdata[] = {
	[0] = {
        	.gpio_base      = TT_VGPIO_BASE,
        	.gpio_number    = ARRAY_SIZE(bcm476x_vgpio0_pins),
        	.pins           = bcm476x_vgpio0_pins,
	},
};

static struct platform_device bcm476x_vgpio[] = {
	[0] = {
        	.name           = "vgpio",
        	.id             = 0,
        	.dev            = {
                	.platform_data  = &bcm476x_vgpio_pdata[0],
        	},
	},
};

static struct resource rtc0_resources[] = 
{
	[e_irq_one_shot] = {
		.start  = BCM4760_INTR_RTC_MTCH,
		.end    = BCM4760_INTR_RTC_MTCH,
		.flags  = IORESOURCE_IRQ,
	},
	[e_irq_periodic] = {
		.start  = BCM4760_INTR_RTC,
		.end    = BCM4760_INTR_RTC,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device rtc0_device = {
    .name = "bcm476x-rtc0",
    .id = -1,
	.resource       = rtc0_resources,
	.num_resources  = ARRAY_SIZE(rtc0_resources),
};

static struct platform_device rtc0_device9040 = {
    .name = "bcm59040_476x-rtc0",
    .id = -1,
};

static struct resource bcm476xwdt_resources[] = 
{
	{
		.start  = BCM4760_INTR_WDT,
		.end    = BCM4760_INTR_WDT,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476xwdt_device = {
	.name = "bcm4760-wdt",
	.id = -1,
	.resource       = bcm476xwdt_resources,
	.num_resources  = ARRAY_SIZE(bcm476xwdt_resources),
};

static struct resource bcm476xadc_resources[] = 
{
	{
		.start  = BCM4760_INTR_AXA,
		.end    = BCM4760_INTR_AXA,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm476xadc_device = {
	.name = "bcm4760-adc",
	.id = -1,
	.resource       = bcm476xadc_resources,
	.num_resources  = ARRAY_SIZE(bcm476xadc_resources),
};

static struct platform_device rng_device = {
   .name = "bcm476x-rng",
   .id = -1,
};
/* I2C devices */

static struct resource bcm4760_i2c0_resource[] = {
	[0] = {
		.start = I2C0_REG_BASE_ADDR,
		.end   = (I2C0_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = BCM4760_INTR_I2C0,
		.end   = BCM4760_INTR_I2C0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm4760_device_i2c0 = {
    .name           = "bcm4760-i2c-0",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_i2c0_resource),
    .resource       = bcm4760_i2c0_resource,
};

static struct resource bcm4760_i2c1_resource[] = {
	[0] = {
		.start = I2C1_REG_BASE_ADDR,
		.end   = (I2C1_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = BCM4760_INTR_I2C1,
		.end   = BCM4760_INTR_I2C1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm4760_device_i2c1 = {
    .name           = "bcm4760-i2c-1",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_i2c1_resource),
    .resource       = bcm4760_i2c1_resource,
};

/* GPIO device */

static struct resource bcm4760_gpio_resource[] = {
	[0] = {
		.start = GIO0_REG_BASE_ADDR,
		.end   = (GIO0_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = GIO1_REG_BASE_ADDR,
		.end   = (GIO1_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = BCM4760_INTR_GPIO00,
		.end   = BCM4760_INTR_GPIO12,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm4760_device_gpio = {
    .name           = "bcm4760-gpio",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_gpio_resource),
    .resource       = bcm4760_gpio_resource,
};

/* PWM device */

static struct platform_device bcm_pwm_device = {
    .name   = "bcm4760-pwm",
};


static struct platform_device bcm4760_pwrseq_device = {
    .name = "bcm4760_pwrseq",
    .id   = -1
};

#ifdef CONFIG_BCM476X_SPI
static struct bcm476x_spi_master spi_bcm476x_master_platform_info_0 = {
   .num_chipselect = 4
};
static struct bcm476x_spi_master spi_bcm476x_master_platform_info_1 = {
   .num_chipselect = 4
};

static struct platform_device spi_bcm476x_master_device_0 = {
    .name = "spi",          // must match name of platform driver bcm476x_driver in spi_kernel_bcm476x for driver binding
    .id = 0, /* Bus number */
    .dev = {
        .platform_data = &spi_bcm476x_master_platform_info_0, /* Passed to driver */
    }
};

static struct platform_device spi_bcm476x_master_device_1 = {
    .name = "spi",        // must match name of platform driver bcm476x_driver in spi_kernel_bcm476x for driver binding
    .id = 1, /* Bus number */
    .dev = {
        .platform_data = &spi_bcm476x_master_platform_info_1, /* Passed to driver */
    }
};

static struct platform_device *bcm476x_spi_master_devices[] __initdata = {
   &spi_bcm476x_master_device_0,
   &spi_bcm476x_master_device_1
};

struct bcm476x_spi_device spi_sample_info = {
    .enable_dma = 0,         /* enable/disable DMA */
    .bits_per_word = 8,
};

#ifdef CONFIG_ENC28J60
struct bcm476x_spi_device enc28j60_chip_info = {
	.enable_dma = 1,         /* use dma transfer with this chip*/
	.bits_per_word = 8
};
#endif

#ifdef CONFIG_MTD_M25P80
struct bcm476x_spi_device m25p80_chip_info = {
    .enable_dma = 1,         /* use dma transfer with this chip*/
    .bits_per_word = 8
};

struct flash_platform_data flash_data= {
    .name       = "spi-flash",
    .type       = "at25f4096",
//    .nr_parts    = 1,
//    .parts      = flash_partition,
};
#endif

struct spi_board_info const bcm476x_spi_board_info[] = {

#ifdef CONFIG_MTD_M25P80
    {
        .modalias       = "m25p80",
        .platform_data  = &flash_data,
        .mode           = SPI_MODE_0,
        .max_speed_hz   = 120000 * 16,
        .bus_num        = 1,
        .chip_select    = 0,
        .controller_data = &m25p80_chip_info,
    },
#endif
};

#endif //CONFIG_BCM476X_SPI

/* 3D Graphics device */

static struct resource bcm4760_gfx_resources[] = {
    [0] = {
        .start = GFX_REG_BASE_ADDR,
        .end   = (GFX_REG_BASE_ADDR + (4 * SZ_4K))-1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = BCM4760_INTR_GFX_CORE,
        .end   = BCM4760_INTR_GFX_MMU,
        .flags = IORESOURCE_IRQ,
    }
};

static struct platform_device bcm4760_device_gfx = {
    .name           = "bcm4760_gfx_pm",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_gfx_resources),
    .resource       = bcm4760_gfx_resources,
};

/* BCM4760 Vectored Interrupt Controller devices */

static struct resource bcm4760_vic0_resources[] = {
	[0] = {
		.start = VIC0_REG_BASE_ADDR,
		.end   = (VIC0_REG_BASE_ADDR + SZ_4K)-1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device bcm4760_device_vic0 = {
    .name           = "bcm4760_vic0",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_vic0_resources),
    .resource       = bcm4760_vic0_resources,
};

static struct resource bcm4760_vic1_resources[] = {
    [0] = {
        .start = VIC1_REG_BASE_ADDR,
        .end   = (VIC1_REG_BASE_ADDR + SZ_4K)-1,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device bcm4760_device_vic1 = {
    .name           = "bcm4760_vic1",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(bcm4760_vic1_resources),
    .resource       = bcm4760_vic1_resources,
};


#if defined(CONFIG_REGULATOR_BCM59040) || defined(CONFIG_PMU_DEVICE_BCM59040)

static struct regulator_init_data bcm59040_LDO1_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO1_CONSTRAINTS",
        .min_uV             = 1800000,
        .max_uV             = 1800000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 1800000,
            .enabled        = 1
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_LDO2_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO2_CONSTRAINTS",
        .min_uV             = 2500000,
        .max_uV             = 2500000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_LDO3_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO3_CONSTRAINTS",
        .min_uV             = 3000000,
        .max_uV             = 3000000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_LDO4_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO4_CONSTRAINTS",
        .min_uV             = 3200000,
        .max_uV             = 3200000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_LDO5_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO5_CONSTRAINTS",
        .min_uV             = 3200000,
        .max_uV             = 3200000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 3200000,
            .enabled        = 1
        },
        .state_standby  =
        {
            .uV             = 3200000,
            .enabled        = 1
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_LDO6_data = {
    .constraints =
    {
        .name               = "BCM59040_LDO6_CONSTRAINTS",
        .min_uV             = 3200000,
        .max_uV             = 3200000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_CSR_data = {
    .constraints =
    {
        .name               = "BCM59040_CSR_CONSTRAINTS",
        .min_uV             =  900000,
        .max_uV             = 1340000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_standby  =
        {
            .uV             = 1200000,
            .enabled        = 1
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct regulator_init_data bcm59040_IOSR_data = {
    .constraints =
    {
        .name               = "BCM59040_IOSR_CONSTRAINTS",
        .min_uV             = 1710000,
        .max_uV             = 1890000,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        .state_disk =
        {
            .uV             = 0,
            .enabled        = 0
        },
        .state_mem  =
        {
            .uV             = 1800000,
            .enabled        = 1
        },
        .state_standby  =
        {
            .uV             = 1800000,
            .enabled        = 1
        },
        .initial_state  =   PM_SUSPEND_MEM,
        .boot_on           = 1
    }
};

static struct platform_device bcm59040_regulator_devices[] = {
    [0] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO1_ID,
        .dev =
        {
            .platform_data	= &bcm59040_LDO1_data,
        },
    },
    [1] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO2_ID,
        .dev =
        {
            .platform_data  = &bcm59040_LDO2_data,
        },
    },
    [2] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO3_ID,
        .dev =
        {
            .platform_data  = &bcm59040_LDO3_data,
        },
    },
    [3] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO4_ID,
        .dev =
        {
            .platform_data  = &bcm59040_LDO4_data,
        },
    },
    [4] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO5_ID,
        .dev =
        {
            .platform_data  = &bcm59040_LDO5_data,
        },
    },
    [5] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_LDO6_ID,
        .dev =
        {
            .platform_data  = &bcm59040_LDO6_data,
        },
    },
    [6] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_CSR_ID,
        .dev =
        {
            .platform_data  = &bcm59040_CSR_data,
        },
    },
    [7] =
    {
        .name           = "bcmpmu_regulator",
        .id             = BCM59040_IOSR_ID,
        .dev =
        {
            .platform_data  = &bcm59040_IOSR_data,
        },
    },
};

#endif

/****************************************************************************
*
*   Called from the customize_machine function in arch/arm/kernel/setup.c
*
*   The customize_machine function is tagged as an arch_initcall
*   (see include/linux/init.h for the order that the various init sections
*   are called in.
*
*****************************************************************************/
#if  defined(CONFIG_USB_BCM476X_UDC2) || defined(CONFIG_USB_BCM476X_UDC2_MODULE)
static u64 bcm476x_udc_dmamask = ~(u64)0;

static struct platform_device bcm476x_usb_device = {
     .name    = "bcm476x_udc2",
     .id      = 0,
     .dev = {
        .dma_mask      = &bcm476x_udc_dmamask,
        .coherent_dma_mask   = 0xffffffff,
     },
     .num_resources = 0,
     .resource   = NULL,

};
#endif

static struct platform_device bcm476x_spu_device = {
    .name = "bcm476x-spu",
    .id   = -1
};

static struct resource bcm4760_ts_resources[] = 
{
	{
		.start  = BCM4760_INTR_TSC,
		.end    = BCM4760_INTR_TSC,
		.flags  = IORESOURCE_IRQ,
	}
};

static tsc_control_table bcm4760_ts_control_table =
{
   .sample_rate         = 976 * 1,
   .data_threshold      = 6,
   .debounce            = 512*1,
   .settling            = 2560,
   .data_point_average  = 8,
   .wire_mode           = TSC_MODE_4WIRE,
   .xres		= 480,
   .yres		= 272,
   .tscMaxX		= 0xF8E0,	// max X touch value
   .tscMaxY		= 0xF405,	// max Y touch value
   .tscMinX		= 0x06B0,	// min X touch value
   .tscMinY		= 0x0920,	// min Y touch value
   .ABSxy		= 1,		// Coordinates are given by tslib.
};

static struct platform_device bcm4760_ts_device =
{
   .name	= "bcm4760_ts",
   .id	= 0,
     .num_resources = ARRAY_SIZE(bcm4760_ts_resources),
     .resource   = bcm4760_ts_resources,
};

static struct platform_device *bcm476x_devices[] __initdata = 
{
	&bcm4760_device_vic0,	/* always need vectored interrupt controllers */
	&bcm4760_device_vic1,	/* and probably earlier than later */
	&bcm4760_device_gpio,	/* no reason not to setup GPIOs early */
	&bcm4760_device_i2c0,	/* I2C must be early up in init order! */
	&bcm4760_device_i2c1,	/* I2C must be early up in init order! */
	&rtc0_device9040,
	&rtc0_device,
	&bcm476xwdt_device,
	&bcm476xadc_device,
	&bcm_pwm_device,
#if  defined(CONFIG_USB_BCM476X_UDC2) || defined(CONFIG_USB_BCM476X_UDC2_MODULE)
	&bcm476x_usb_device,
#endif
#ifdef CONFIG_PM
	&bcm4760_pwrseq_device,	/* Should be low in init order! */
#endif
	&rng_device,
	&bcm476x_spu_device,
	&bcm4760_ts_device,
#ifdef CONFIG_PM                /* only for power management so only include it then */
	&bcm4760_device_gfx,	/* should be pretty low in init/resume ordering */
#endif
#ifdef CONFIG_TOMTOM_FLIPFLOP
	&bcm4760_rtc_flipflop_pdev,
#endif
};

static void __init bcm476x_init_machine( void )
{
#if defined(BCM476X_ENABLE_DAFCA_TRACE) && (BCM476X_ENABLE_DAFCA_TRACE == 1)
	writel(0x0009249, IO_ADDRESS(CMU_R_CHIP_PIN_MUX9_MEMADDR));
#endif
	platform_device_register(&bcm476x_vgpio[0]);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	/* register sysctl table */
	gSysCtlHeader = register_sysctl_table(gSysCtl, 0);
	if (gSysCtlHeader != NULL)
	{
		gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
	}
#else
	gSysCtlHeader = register_sysctl_table(gSysCtl);
#endif

	platform_add_devices(bcm476x_devices, ARRAY_SIZE(bcm476x_devices));

	bcm4760_clk_init() ;
	bcm476x_amba_init();

	bcm476x_clcd_lookup_register (bcm476x_clcd_lookup);

	/* Enable I2S MCLK out on GPIO27 pin */
	writel((readl(IO_ADDRESS(CMU_R_CHIP_PIN_MUX6_MEMADDR)) & ~CMU_F_GPIO_27_MXSEL_MASK) | (3 << CMU_F_GPIO_27_MXSEL_R),
           IO_ADDRESS(CMU_R_CHIP_PIN_MUX6_MEMADDR));
	/* Also force I2S to 12Mhz */
	writel(readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_CKG_I2S_SEL_12MHZ_MASK,
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)); 

#endif

#if defined(CONFIG_REGULATOR_BCM59040) || defined(CONFIG_PMU_DEVICE_BCM59040)
    platform_device_register(&bcm59040_regulator_devices[0]);
    platform_device_register(&bcm59040_regulator_devices[1]);
    platform_device_register(&bcm59040_regulator_devices[2]);
    platform_device_register(&bcm59040_regulator_devices[3]);
    platform_device_register(&bcm59040_regulator_devices[4]);
    platform_device_register(&bcm59040_regulator_devices[5]);
    platform_device_register(&bcm59040_regulator_devices[6]);
    platform_device_register(&bcm59040_regulator_devices[7]);
#endif
}

/****************************************************************************
*
*   Timer related information. sys_timer_init is called from
*   time_init (in arch/arm/kernel/time.c).
*
*****************************************************************************/

static struct sys_timer bcm476x_timer =
{
    .init		= sys_timer_init,
#ifndef CONFIG_GENERIC_TIME
    .offset     = sys_timer_gettimeoffset,
#endif
#ifdef CONFIG_PM
//	.resume     = sys_timer_resume
#endif
};

#ifdef CONFIG_BCM476X_SPI
static int __init init_bcm476x_spi( void )
{
    platform_add_devices(bcm476x_spi_master_devices, ARRAY_SIZE(bcm476x_spi_master_devices));
    return spi_register_board_info( bcm476x_spi_board_info, ARRAY_SIZE(bcm476x_spi_board_info) );
}

arch_initcall_sync( init_bcm476x_spi );
#endif // CONFIG_BCM476X_SPI

#if defined(CONFIG_PMU_DEVICE_BCM59002) || \
    defined(CONFIG_BATTERY_BCM59040) || \
    defined(CONFIG_PMU_DEVICE_BCM59040)
/****************************************************************************
*
*   Machine Description
*
*****************************************************************************/

MACHINE_START(BCM4760, "BCM4760")
    /* Maintainer: Broadcom Corporation */
    .phys_io 		= BCM47XX_ARM_PERIPH_BASE,
    .io_pg_offst 	= IO_ADDRESS(BCM47XX_ARM_PERIPH_BASE),
    .boot_params 	= (BCM47XX_ARM_DRAM + 0x100),
    .map_io 		= bcm476x_map_io,
    .init_irq 		= bcm476x_init_irq,
    .timer 		= &bcm476x_timer,
    .init_machine 	= bcm476x_init_machine
MACHINE_END

