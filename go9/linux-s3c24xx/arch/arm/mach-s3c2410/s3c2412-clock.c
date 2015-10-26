/* linux/arch/arm/mach-s3c2410/s3c2412-clock.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Copyright (c) 2006 TomTom BV
 *	Thomas Kleffel <tk@maintech.de>
 *
 * S3C2412 Clock support
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

#define CLK_H   ((struct clk *)1)
#define CLK_P   ((struct clk *)2)

static struct clk s3c2412_clk_upll = {
	.name		= "upll",
	.id		= -1,
};

static struct clk s3c2412_init_clocks[] = {
	{ .name    = "dma",
	  .id	   = 0,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_DMA0
	},
	{ .name    = "dma",
	  .id	   = 1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_DMA1
	},
	{ .name    = "dma",
	  .id	   = 2,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_DMA2
	},
	{ .name    = "dma",
	  .id	   = 3,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_DMA3
	},
	{ .name    = "nand",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_NAND
	},
	{ .name    = "lcd",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_LCDC
	},
	{ .name    = "usb-host",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_USBH
	},
	{ .name    = "usb-device",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_USBD
	},
	{ .name    = "timers",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_PWMT
	},
	{ .name    = "cam",
	  .id      = -1,
	  .parent  = CLK_H,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_CAMCLK
	},
	{ .name    = "sdi",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_SDI
	},
	{ .name    = "uart",
	  .id	   = 0,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_UART0
	},
	{ .name    = "uart",
	  .id	   = 1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_UART1
	},
	{ .name    = "uart",
	  .id	   = 2,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_UART2
	},
	{ .name    = "gpio",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_GPIO
	},
	{ .name    = "rtc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_RTC
	},
	{ .name    = "adc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_ADC
	},
	{ .name    = "i2c",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_IIC
	},
	{ .name    = "iis",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_IIS
	},
	{ .name    = "spi",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_SPI
	},
	{ .name    = "watchdog",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c24xx_clkcon_enable,
	  .ctrlbit = S3C2412_CLKCON_WDT
	},
};

static int s3c2412_clk_add(struct sys_device *sysdev)
{
	struct clk *clk_xtal;
	struct clk *clk_h;
	struct clk *clk_p;
	int i;
	struct clk *clkp;

	clk_xtal = clk_get(NULL, "xtal");
	if (IS_ERR(clk_xtal)) {
		printk(KERN_ERR "S3C2412: Failed to get clk_xtal\n");
		return -EINVAL;
	}

	s3c2412_clk_upll.rate = s3c2410_get_pll(__raw_readl(S3C2410_UPLLCON), clk_xtal->rate);

	printk("S3C2412: Clock Support, UPLL %lu.%03lu MHz\n",
	       print_mhz(s3c2412_clk_upll.rate));

	clk_h = clk_get(NULL, "hclk");
	clk_p = clk_get(NULL, "pclk");

	if (IS_ERR(clk_h) || IS_ERR(clk_p)) {
		printk(KERN_ERR "S3C2412: Failed to get parent clocks\n");
		return -EINVAL;
	}

	s3c24xx_register_clock(&s3c2412_clk_upll);

	for (i = 0; i < ARRAY_SIZE(s3c2412_init_clocks); i++) {
		clkp = &s3c2412_init_clocks[i];
		switch ((int)clkp->parent) {
		case (int)CLK_H: clkp->parent = clk_h; break;
		case (int)CLK_P: clkp->parent = clk_p; break;
		default:         clkp->parent = NULL;  break;
		}
		s3c24xx_register_clock(clkp);
	}

	return 0;
}

static struct sysdev_driver s3c2412_clk_driver = {
	.add	= s3c2412_clk_add,
};

static __init int s3c24xx_clk_driver(void)
{
	return sysdev_driver_register(&s3c2412_sysclass, &s3c2412_clk_driver);
}

arch_initcall(s3c24xx_clk_driver);
