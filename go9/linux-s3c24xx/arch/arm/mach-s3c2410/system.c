/* arch/arm/mach-s3c2410/system.c
 *
 * System function definitions.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/io.h>

#include <asm/arch/map.h>
#include <asm/arch/idle.h>
#include <asm/arch/system.h>

#include <asm/arch/regs-watchdog.h>
#include <asm/arch/regs-clock.h>

#ifdef CONFIG_MACH_TOMTOMGO
#include <asm/arch/regs-dyn.h>
#include <asm/arch/tomtomgo-wake.h>
#include <barcelona/gopins.h>
#endif /* CONFIG_MACH_TOMTOMGO */

void (*s3c24xx_idle)(void);

void s3c24xx_default_idle(void)
{
	void __iomem *reg = S3C2410_CLKCON;
	unsigned long tmp;
	int i;

	/* idle the system by using the idle mode which will wait for an
	 * interrupt to happen before restarting the system.
	 */

	/* Warning: going into idle state upsets jtag scanning */

	__raw_writel(__raw_readl(reg) | S3C2410_CLKCON_IDLE, reg);

	/* the samsung port seems to do a loop and then unset idle.. */
	for (i = 0; i < 50; i++) {
		tmp += __raw_readl(reg); /* ensure loop not optimised out */
	}

	/* this bit is not cleared on re-start... */

	__raw_writel(__raw_readl(reg) & ~S3C2410_CLKCON_IDLE, reg);
}

void arch_idle(void)
{
	if (s3c24xx_idle != NULL)
		(s3c24xx_idle)();
	else
		s3c24xx_default_idle();
}


void arch_reset(char mode)
{
	if (mode == 's') {
		cpu_reset(0);
	}

#ifdef CONFIG_MACH_TOMTOMGO
#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412)
		tomtomgo_reboot_2412(S3C2412_INFORM0_NEUTRAL);
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443)
		tomtomgo_reboot_2443(S3C2412_INFORM0_NEUTRAL);
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450)
		tomtomgo_reboot_2450(S3C2412_INFORM0_NEUTRAL);
#endif /* CONFIG_CPU_S3C2450 */
#endif /* CONFIG_MACH_TOMTOMGO */

	printk("arch_reset: attempting watchdog reset\n");

	__raw_writel(0, S3C2410_WTCON);	  /* disable watchdog, to be safe  */

	/* put initial values into count and data */
	__raw_writel(0x100, S3C2410_WTCNT);
	__raw_writel(0x100, S3C2410_WTDAT);

	/* set the watchdog to go and reset... */
	__raw_writel(S3C2410_WTCON_ENABLE|S3C2410_WTCON_DIV16|S3C2410_WTCON_RSTEN |
				 S3C2410_WTCON_PRESCALE(0x80), S3C2410_WTCON);

	/* wait for reset to assert... */
	mdelay(5000);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

/* EOF */
