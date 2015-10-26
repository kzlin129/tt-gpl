/* linux/arch/arm/mach-s3c2410/s3c2450.c
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author 0: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author 1: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 * Samsung S3C2450 Mobile CPU support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *	25-Oct-2006 DA   Start of S3C2450 CPU support
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware/clock.h>

#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/regs-dsc.h>
#include <asm/arch/pm.h>

#include "s3c2450.h"
#include "cpu.h"
#include "devs.h"
#include "clock.h"

/* Initial IO mappings */

static struct map_desc s3c2450_iodesc[] __initdata = {
	IODESC_ENT(USBHOST),
	IODESC_ENT(CLKPWR),
	IODESC_ENT(LCD),
	IODESC_ENT(TIMER),
	IODESC_ENT(ADC),
	IODESC_ENT(WATCHDOG),
};

static struct resource s3c_uart0_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART0,
		.end   = S3C2410_PA_UART0 + 0x4000 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3CUART_RX0,
		.end   = IRQ_S3CUART_ERR0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource s3c_uart1_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART1,
		.end   = S3C2410_PA_UART1 + 0x4000 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3CUART_RX1,
		.end   = IRQ_S3CUART_ERR1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource s3c_uart2_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART2,
		.end   = S3C2410_PA_UART2 + 0x4000 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3CUART_RX2,
		.end   = IRQ_S3CUART_ERR2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource s3c_uart3_resource[] = {
	[0] = {
		.start = S3C2450_PA_UART3,
		.end   = S3C2450_PA_UART3 + 0x4000 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3C2450_RX3,
		.end   = IRQ_S3C2450_ERR3,
		.flags = IORESOURCE_IRQ,
	},
};

/* our uart devices */

static struct platform_device s3c_uart0 = {
	.name		  = "s3c2443-uart",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_uart0_resource),
	.resource	  = s3c_uart0_resource,
};

static struct platform_device s3c_uart1 = {
	.name		  = "s3c2443-uart",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_uart1_resource),
	.resource	  = s3c_uart1_resource,
};

static struct platform_device s3c_uart2 = {
	.name		  = "s3c2443-uart",
	.id		  = 2,
	.num_resources	  = ARRAY_SIZE(s3c_uart2_resource),
	.resource	  = s3c_uart2_resource,
};

static struct platform_device s3c_uart3 = {
	.name		  = "s3c2443-uart",
	.id		  = 3,
	.num_resources	  = ARRAY_SIZE(s3c_uart3_resource),
	.resource	  = s3c_uart3_resource,
};

static struct platform_device *uart_devices[] __initdata = {
	&s3c_uart0,
	&s3c_uart1,
	&s3c_uart2,
	&s3c_uart3,
};

/* uart initialisation */

void __init s3c2450_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	struct platform_device *platdev;
	int uart, count;

	for (uart = count = 0; uart < no; uart++, cfg++) {
		if (cfg->hwport >= ARRAY_SIZE(uart_devices))
			continue;

		platdev = uart_devices[cfg->hwport];

		s3c24xx_uart_devs[count++] = platdev;
		platdev->dev.platform_data = cfg;
	}

	s3c24xx_uart_count = count;
}


#ifdef CONFIG_PM

struct sleep_save s3c2450_sleep[] = {
#if 0	/* XXX TODO */
	SAVE_ITEM(S3C2450_DSC0),
	SAVE_ITEM(S3C2450_DSC1),
	SAVE_ITEM(S3C2450_DSC2),
#endif	/* XXX TODO */
};

static int s3c2450_suspend(struct sys_device *dev, pm_message_t state)
{
	s3c_pm_do_save(s3c2450_sleep, ARRAY_SIZE(s3c2450_sleep));
	return 0;
}

static int s3c2450_resume(struct sys_device *dev)
{
	s3c_pm_do_restore(s3c2450_sleep, ARRAY_SIZE(s3c2450_sleep));
	return 0;
}

#else
#define s3c2450_suspend NULL
#define s3c2450_resume  NULL
#endif

struct sysdev_class s3c2450_sysclass = {
	set_kset_name("s3c2450-core"),
	.suspend	= s3c2450_suspend,
	.resume		= s3c2450_resume
};

static struct sys_device s3c2450_sysdev = {
	.cls		= &s3c2450_sysclass,
};

void __init s3c2450_map_io(struct map_desc *mach_desc, int size)
{
	/* register our io-tables */

	iotable_init(s3c2450_iodesc, ARRAY_SIZE(s3c2450_iodesc));
	iotable_init(mach_desc, size);

	/* rename any peripherals used differing from the s3c2410 */

	s3c_device_i2c.name  = "s3c2450-i2c";
	s3c_device_nand.name = "s3c2450-nand";

	/* change irq for watchdog */

	s3c_device_wdt.resource[1].start = IRQ_S3C2450_WDT;
	s3c_device_wdt.resource[1].end   = IRQ_S3C2450_WDT;
}

void __init s3c2450_init_clocks(int xtal)
{
	unsigned long msysclk;
	unsigned long clkdiv0;
	unsigned long armclk;
	unsigned long prediv;
	unsigned long hclkdiv;
	unsigned long hclk;
	unsigned long pclk;

	/* now we've got our machine bits initialised, work out what
	 * clocks we've got */

	msysclk = s3c2450_get_mpll(__raw_readl(S3C2450_MPLLCON), xtal);

	clkdiv0 = __raw_readl(S3C2450_CLKDIV0);

	/* work out clock scalings */

	switch (clkdiv0 & S3C2450_CLKDIV0_ARMDIV_MASK) {
	case S3C2450_CLKDIV0_ARMDIV_1_1:
		armclk = msysclk;
		break;
	case S3C2450_CLKDIV0_ARMDIV_1_2:
		armclk = msysclk / 2;
		break;
	case S3C2450_CLKDIV0_ARMDIV_1_3:
		armclk = msysclk / 3;
		break;
	case S3C2450_CLKDIV0_ARMDIV_1_4:
		armclk = msysclk / 4;
		break;
	case S3C2450_CLKDIV0_ARMDIV_1_6:
		armclk = msysclk / 6;
		break;
	case S3C2450_CLKDIV0_ARMDIV_1_8:
		armclk = msysclk / 8;
		break;
	default:
		panic("Impossible CLKDIV0 value 0x%08lx\n", clkdiv0);
		break;
	}

	prediv = (clkdiv0 >> S3C2450_CLKDIV0_PREDIV_SHIFT) & S3C2450_CLKDIV0_PREDIV_MASK;
	hclkdiv = (clkdiv0 >> S3C2450_CLKDIV0_HCLKDIV_SHIFT) & S3C2450_CLKDIV0_HCLKDIV_MASK;
	hclk = msysclk / ((prediv + 1) * (hclkdiv + 1));
	pclk = (clkdiv0 & S3C2450_CLKDIV0_PCLKDIV) ? hclk / 2 : hclk;

	/* print brief summary of clocks, etc */

	printk("S3C2450: msys %lu.%03lu MHz, core %lu.%03lu MHz, memory %lu.%03lu MHz, peripheral %lu.%03lu MHz\n",
	       print_mhz(msysclk), print_mhz(armclk), print_mhz(hclk), print_mhz(pclk));

	/* initialise the clocks here, to allow other things like the
	 * console to use them, and to add new ones after the initialisation
	 */

	s3c24xx_setup_clocks(xtal, armclk, hclk, pclk);
}

/* need to register class before we actually register the device, and
 * we also need to ensure that it has been initialised before any of the
 * drivers even try to use it (even if not on an s3c2450 based system)
 * as a driver which may support both 2410 and 2450 may try and use it.
*/

int __init s3c2450_core_init(void)
{
	return sysdev_class_register(&s3c2450_sysclass);
}

core_initcall(s3c2450_core_init);

int __init s3c2450_init(void)
{
	int ret;

	printk("S3C2450: Initialising architecture\n");

	ret = sysdev_register(&s3c2450_sysdev);
	if (ret != 0)
		printk(KERN_ERR "failed to register sysdev for s3c2450\n");
	else
		ret = platform_add_devices(s3c24xx_uart_devs, s3c24xx_uart_count);

	return ret;
}
