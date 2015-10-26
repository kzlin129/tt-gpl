/* linux/arch/arm/plat-s5p64xx/s5p6440-init.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5P6440 - CPU initialisation (common with other S5P64XX chips)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>

#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/regs-serial.h>
#include <plat/s5p6440.h>

static struct s3c24xx_uart_clksrc s5p6440_uartclks[] = {
	[0] = {
		.name		= "pclk_low",
		.divisor	= 1,
	},
};

/* uart registration process */

void __init s5p6440_common_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	int i;

	for (i = 0; i < no; i++) {
		if (!cfg[i].clocks || !cfg[i].clocks_size) {
			cfg[i].clocks = s5p6440_uartclks;
			cfg[i].clocks_size = ARRAY_SIZE(s5p6440_uartclks);
		}
	}

	s3c24xx_init_uartdevs("s3c6400-uart", s5p64xx_uart_resources, cfg, no);
}
