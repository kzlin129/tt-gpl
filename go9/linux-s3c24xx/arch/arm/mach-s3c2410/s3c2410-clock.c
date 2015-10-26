/* linux/arch/arm/mach-s3c2410/s3c2410-clock.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/sysdev.h>

#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <asm/hardware.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>

#include "clock.h"
#include "cpu.h"

/* S3C2410 clock support */
#define CLK_H   ((struct clk *)1)
#define CLK_P   ((struct clk *)2)

static struct clk s3c2410_init_clocks[] = {
	{ .name    = "nand",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_NAND
	},
	{ .name    = "lcd",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_LCDC
	},
	{ .name    = "usb-host",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_USBH
	},
	{ .name    = "usb-device",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_USBD
	},
	{ .name    = "timers",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_PWMT
	},
	{ .name    = "sdi",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_SDI
	},
	{ .name    = "uart",
	  .id	   = 0,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_UART0
	},
	{ .name    = "uart",
	  .id	   = 1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_UART1
	},
	{ .name    = "uart",
	  .id	   = 2,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_UART2
	},
	{ .name    = "gpio",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_GPIO
	},
	{ .name    = "rtc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_RTC
	},
	{ .name    = "adc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_ADC
	},
	{ .name    = "i2c",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_IIC
	},
	{ .name    = "iis",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_IIS
	},
	{ .name    = "spi",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2410_CLKCON_SPI
	},
	{ .name    = "watchdog",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .ctrlbit = 0
	},
};


static int s3c2410_clk_add(struct sys_device *sysdev)
{
	struct clk *clk_h;
	struct clk *clk_p;
	int i;
	struct clk *clkp;

	printk("S3C2410: Clock Support\n");

	clk_h = clk_get(NULL, "hclk");
	clk_p = clk_get(NULL, "pclk");

	if (IS_ERR(clk_h) || IS_ERR(clk_p)) {
		printk(KERN_ERR "S3C2410: Failed to get parent clocks\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(s3c2410_init_clocks); i++) {
		clkp = &s3c2410_init_clocks[i];
		switch ((int)clkp->parent) {
		case (int)CLK_H: clkp->parent = clk_h; break;
		case (int)CLK_P: clkp->parent = clk_p; break;
		default:         clkp->parent = NULL;  break;
		}
		s3c24xx_register_clock(clkp);
	}

	return 0;
}

static struct sysdev_driver s3c2410_clk_driver = {
	.add	= s3c2410_clk_add,
};

static __init int s3c24xx_clk_driver(void)
{
	return sysdev_driver_register(&s3c2410_sysclass, &s3c2410_clk_driver);
}

arch_initcall(s3c24xx_clk_driver);
