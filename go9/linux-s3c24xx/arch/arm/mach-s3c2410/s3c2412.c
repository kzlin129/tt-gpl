/* linux/arch/arm/mach-s3c2410/s3c2412.c
 *
 * Copyright (C) 2006 TomTom BV. All rights reserved.
 * Author 0: Koen Martens <kmartens@sonologic.nl>
 * Author 1: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * Samsung S3C2412 Mobile CPU support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *	27-Feb-2006 KM   Start of S3C2412 CPU support
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

#include <asm/arch/idle.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/regs-dsc.h>
#include <asm/arch/pm.h>
#include <linux/bootmem.h>

#include <barcelona/gopins.h>
#include "s3c2412.h"
#include "cpu.h"
#include "devs.h"
#include "clock.h"

/* Initial IO mappings */

static struct map_desc s3c2412_iodesc[] __initdata = {
	IODESC_ENT(USBHOST),
	IODESC_ENT(CLKPWR),
	IODESC_ENT(LCD),
	IODESC_ENT(TIMER),
	IODESC_ENT(ADC),
	IODESC_ENT(WATCHDOG),
};

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/* Uart clock. Needs to be registered early for console port. */
static struct clk uartclk_p =
{
	.name    = "uartclk",
	.id      = -1,
	.parent  = NULL,
	.enable  = s3c24xx_clkcon_enable,
	.ctrlbit = S3C2412_CLKCON_UARTCLK
};
#endif

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

/* our uart devices */

static struct platform_device s3c_uart0 = {
	.name		  = "s3c2412-uart",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_uart0_resource),
	.resource	  = s3c_uart0_resource,
};

static struct platform_device s3c_uart1 = {
	.name		  = "s3c2412-uart",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_uart1_resource),
	.resource	  = s3c_uart1_resource,
};

static struct platform_device s3c_uart2 = {
	.name		  = "s3c2412-uart",
	.id		  = 2,
	.num_resources	  = ARRAY_SIZE(s3c_uart2_resource),
	.resource	  = s3c_uart2_resource,
};

static struct platform_device *uart_devices[] __initdata = {
	&s3c_uart0,
	&s3c_uart1,
	&s3c_uart2,
};

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
static struct s3c24xx_uart_clksrc s3c2412_uart_clocks[] =
{
/* PCLK is commented out due to corruption issues when using it. */
/*      {"pclk",        1,      0,      115200, 0}, */
        {"uartclk",     1,      0,      115200, 0},
};
#endif

/* uart initialisation */
void __init s3c2412_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	struct platform_device *platdev;
	int uart, count;
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	int gps_idx;

	gps_idx=s3c24xx_get_uart_index( IO_GetGpsDevice( ) );
#endif

	for (uart = count = 0; uart < no; uart++, cfg++) {
		if (cfg->hwport >= ARRAY_SIZE(uart_devices))
			continue;

		platdev = uart_devices[cfg->hwport];

		s3c24xx_uart_devs[count++] = platdev;
		platdev->dev.platform_data = cfg;

/*#ifdef CONFIG_CPU_FREQ */
/* Commented out as PCLK gives occasional corrupted serial characters even on the console. */
#if 0
		/* Use only UARTCLK as source for console/rdstmc. Otherwise too much corruption. */
		if( uart == gps_idx )
		{
			cfg->clocks=&(s3c2412_uart_clocks[1]);
			cfg->clocks_size=1;
		}
		else
		{
			cfg->clocks=s3c2412_uart_clocks;
			cfg->clocks_size=(sizeof( s3c2412_uart_clocks )/sizeof( s3c2412_uart_clocks[0] ));
		}
#endif
/* Temporary replacement as PCLK isn't used anyway. */
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
		cfg->clocks=s3c2412_uart_clocks;
		cfg->clocks_size=(sizeof( s3c2412_uart_clocks )/sizeof( s3c2412_uart_clocks[0] ));
#endif
	}

	s3c24xx_uart_count = count;
}


#ifdef CONFIG_PM

struct sleep_save s3c2412_sleep[] = {
	SAVE_ITEM(S3C2412_DSC0),
	SAVE_ITEM(S3C2412_DSC1)
};

static int s3c2412_suspend(struct sys_device *dev, pm_message_t state)
{
	s3c_pm_do_save(s3c2412_sleep, ARRAY_SIZE(s3c2412_sleep));
	return 0;
}

static int s3c2412_resume(struct sys_device *dev)
{
	s3c_pm_do_restore(s3c2412_sleep, ARRAY_SIZE(s3c2412_sleep));
	return 0;
}

#else
#define s3c2412_suspend NULL
#define s3c2412_resume  NULL
#endif

struct sysdev_class s3c2412_sysclass = {
	set_kset_name("s3c2412-core"),
	.suspend	= s3c2412_suspend,
	.resume		= s3c2412_resume
};

static struct sys_device s3c2412_sysdev = {
	.cls		= &s3c2412_sysclass,
};

static void s3c2412_idle(void)
{
	s3c2412_gen_stbywfi(S3C2412_PWRCFG_STBYWFI_IDLE);
}

void __init s3c2412_map_io(struct map_desc *mach_desc, int size)
{
	/* register our io-tables */

	iotable_init(s3c2412_iodesc, ARRAY_SIZE(s3c2412_iodesc));
	iotable_init(mach_desc, size);

	/* rename any peripherals used differing from the s3c2410 */
	s3c_device_i2c.name  = "s3c2412-i2c";
	s3c_device_nand.name = "s3c2412-nand";

	/* change irq for sdi */
	s3c_device_sdi.resource[1].start = IRQ_S3C2412_SDI;
	s3c_device_sdi.resource[1].end   = IRQ_S3C2412_SDI;

	/* change irq for spi1 */
	s3c_device_spi1.resource[1].start = IRQ_S3C2412_SPI1;
	s3c_device_spi1.resource[1].end   = IRQ_S3C2412_SPI1;

	/* Adjust memory area of camif */
#ifdef CONFIG_VIDEO_S3C241X_CAMIF
	s3c_device_i2c_camif.name = "s3c2412-i2c-camif";
	s3c_device_camif.name  = "s3c2412-camif";
	s3c_device_camif.resource[0].start = S3C2413_PA_CAMIF;
	s3c_device_camif.resource[0].end   = S3C2413_PA_CAMIF + S3C24XX_SZ_CAMIF - 1;
#endif
	
	/* set idle function */
	s3c24xx_idle = s3c2412_idle;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
static void __init s3c2412_setup_uart_clk( int xtal )
{
	unsigned long int	clkdivn;
	unsigned long int	clksrc, clksrc_sav;

	/* Register the clock. */
	if (s3c24xx_register_clock(&uartclk_p) < 0)
		printk(KERN_ERR "failed to register uartclk\n");

	/* Set right rate. */
	uartclk_p.rate=xtal;

	/* Read registers and save CLKSRC. We'll need it later. */
	clkdivn=__raw_readl( S3C2410_CLKDIVN );
	clksrc=__raw_readl( S3C2412_CLKSRC );
	clksrc_sav=clksrc;

	/* Set divider to 1. */
	clkdivn&=~S3C2412_CLKDIVN_UARTCLKDIV_MASK;

	/* Set source to EREFCLK (12 MHz) */
	/* First set SELUART AND SELIIS so that they don't use EREFCLK. Otherwise it won't swap bits. */
	/* Also make sure the original state of SELIIS is saved. */
	clksrc|=S3C2412_CLKSRC_SELUART | S3C2412_CLKSRC_SELIIS;
	__raw_writel( clksrc, S3C2412_CLKSRC );

	/* Now mask out SELEREF. We'll force it to external oscillator selection. */
	clksrc&=~S3C2412_CLKSRC_SELEREF_MASK;
	__raw_writel( clksrc, S3C2412_CLKSRC );
	clksrc|=S3C2412_CLKSRC_SELEREF_EXTOSC;
	__raw_writel( clksrc, S3C2412_CLKSRC );

	/* Now set the SELIIS back. */
	clksrc&=~(S3C2412_CLKSRC_SELUART | S3C2412_CLKSRC_SELIIS);
	clksrc|=(clksrc_sav & S3C2412_CLKSRC_SELIIS);
	__raw_writel( clksrc, S3C2412_CLKSRC );

	/* Set CLKDIVN. */
	__raw_writel( clkdivn, S3C2410_CLKDIVN );
	return;
}
#endif

void __init s3c2412_init_clocks(int xtal)
{
	unsigned long clkdiv;
	unsigned long hclk, fclk, pclk;
	int hdiv = 1;

	/* now we've got our machine bits initialised, work out what
	 * clocks we've got */

	fclk = s3c2410_get_pll(__raw_readl(S3C2410_MPLLCON), xtal);

	clkdiv = __raw_readl(S3C2410_CLKDIVN);

	/* work out clock scalings */

	switch (clkdiv & S3C2412_CLKDIVN_HCLKDIV_MASK) {
	case S3C2412_CLKDIVN_HCLKDIV_1_2:
		hdiv = 1;
		break;

	case S3C2412_CLKDIVN_HCLKDIV_2_4:
		hdiv = 2;
		break;

	case S3C2412_CLKDIVN_HCLKDIV_3_6:
		hdiv = 3;
		break;

	case S3C2412_CLKDIVN_HCLKDIV_4_8:
		hdiv = 4;
		break;
	}

	if (!(clkdiv & S3C2412_CLKDIVN_ARMDIV))
		fclk *= 2;

	hclk = fclk / hdiv;
	pclk = hclk / ((clkdiv & S3C2412_CLKDIVN_PCLKDIV)? 2 : 1);
	if (clkdiv & S3C2412_CLKDIVN_DVSEN)
		fclk = hclk;

	/* print brief summary of clocks, etc */

	printk("S3C2412: core %lu.%03lu MHz, memory %lu.%03lu MHz, peripheral %lu.%03lu MHz\n",
		   print_mhz(fclk), print_mhz(hclk), print_mhz(pclk));

	/* initialise the clocks here, to allow other things like the
	 * console to use them, and to add new ones after the initialisation
	 */

	s3c24xx_setup_clocks(xtal, fclk, hclk, pclk);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	/* Setup the uartclk. Needs to be done this early so the console can use it. */
	s3c2412_setup_uart_clk( xtal );
#endif
}

/* need to register class before we actually register the device, and
 * we also need to ensure that it has been initialised before any of the
 * drivers even try to use it (even if not on an s3c2412 based system)
 * as a driver which may support both 2410 and 2412 may try and use it.
*/

int __init s3c2412_core_init(void)
{
	return sysdev_class_register(&s3c2412_sysclass);
}

core_initcall(s3c2412_core_init);

int __init s3c2412_init(void)
{
	int ret;

	printk("S3C2412: Initialising architecture\n");

	ret = sysdev_register(&s3c2412_sysdev);
	if (ret != 0)
		printk(KERN_ERR "failed to register sysdev for s3c2412\n");
	else
		ret = platform_add_devices(s3c24xx_uart_devs, s3c24xx_uart_count);

	return ret;
}
