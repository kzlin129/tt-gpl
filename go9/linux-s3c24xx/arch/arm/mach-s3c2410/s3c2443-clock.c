/* linux/arch/arm/mach-s3c2410/s3c2443-clock.c
 *
 * Copyright (C) 2004,2005, 2006 TomTom BV <http://www.tomtom.com/>
 * Author 0: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * Samsung S3C2443 clock support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

/* S3C2443 clock support */
#define CLK_H   ((struct clk *)1)
#define CLK_P   ((struct clk *)2)
#define CLK_S	((struct clk *)3)
#define CLK_E	((struct clk *)4)

static struct clk s3c2443_clk_epll = {
	.name = "epll",
	.id   = -1,
};

static int epll_clk_null_enable(struct clk *clk, int enable){
	return 0;	
}

static struct clk s3c2443_init_clocks[] = {
	{ .name    = "dma",
	  .id	   = 0,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA0
	},
	{ .name    = "dma",
	  .id	   = 1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA1
	},
	{ .name    = "dma",
	  .id	   = 2,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA2
	},
	{ .name    = "dma",
	  .id	   = 3,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA3
	},
	{ .name    = "dma",
	  .id	   = 4,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA4
	},
	{ .name    = "dma",
	  .id	   = 5,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DMA5
	},
	{ .name    = "camif",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_CAMIF
	},
	{ .name    = "dispcon",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DISPCON
	},
	{ .name    = "lcd",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_LCDCON
	},
	{ .name    = "usb-host",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_USBHOST
	},
	{ .name    = "usb-device",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_USBDEV
	},
	{ .name    = "hsmmc",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_HSMMC
	},
        { .name    = "hsmmc-ext",
          .id      = -1,
          .parent  = CLK_S,
          .enable  = s3c2443_sclkcon_enable,
          .ctrlbit = S3C2443_SCLKCON_HSMMCCLK_EXT
        },
	{ .name    = "cfc",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_CFC
	},
	{ .name    = "ssmc",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_SSMC
	},
	{ .name    = "dramc",
	  .id	   = -1,
	  .parent  = CLK_H,
	  .enable  = s3c2443_hclkcon_enable,
	  .ctrlbit = S3C2443_HCLKCON_DRAMC
	},

	{ .name    = "uart",
	  .id	   = 0,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_UART0
	},
	{ .name    = "uart",
	  .id	   = 1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_UART1
	},
	{ .name    = "uart",
	  .id	   = 2,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_UART2
	},
	{ .name    = "uart",
	  .id	   = 3,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_UART3
	},
	{ .name    = "i2c",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_IIC
	},
	{ .name    = "sdi",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_SDI
	},
	{ .name    = "spi-hs",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_SPI_HS
	},
	{ .name    = "adc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_TSADC
	},
	{ .name    = "ac97",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_AC97
	},
//	{ .name    = "iis",
//	  .id	   = -1,
//	  .parent  = CLK_P,
//	  .enable  = s3c2443_pclkcon_enable,
//	  .ctrlbit = S3C2443_PCLKCON_IIS
//	},
	{ .name    = "iis",
	  .id	   = -1,
	  .parent  = CLK_E,
	  .enable  = epll_clk_null_enable,
	},
	{ .name    = "timers",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_PWM
	},
	{ .name    = "watchdog",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_WDT
	},
	{ .name    = "rtc",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_RTC
	},
	{ .name    = "gpio",
	  .id	   = -1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_GPIO
	},
	{ .name    = "spi",
	  .id	   = 0,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_SPI_0
	},
	{ .name    = "spi",
	  .id	   = 1,
	  .parent  = CLK_P,
	  .enable  = s3c2443_pclkcon_enable,
	  .ctrlbit = S3C2443_PCLKCON_SPI_1
	},

	{ .name    = "usbhostclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_USBHOST
	},
	{ .name    = "uartclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_UARTCLK
	},
	{ .name    = "iisclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_IISCLK
	},
	{ .name    = "dispclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_DISPCLK
	},
	{ .name    = "camclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_CAMCLK
	},
	{ .name    = "hsmmcclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_HSMMCCLK
	},
	{ .name    = "spiclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_SPICLK
	},
	{ .name    = "ssmcclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_SSMCCLK
	},
	{ .name    = "ddrclk",
	  .id	   = -1,
	  .parent  = CLK_S,
	  .enable  = s3c2443_sclkcon_enable,
	  .ctrlbit = S3C2443_SCLKCON_DDRCLK
	},
};


static int s3c2443_clk_add(struct sys_device *sysdev)
{
	struct clk *clk_xtal;
	struct clk *clk_h;
	struct clk *clk_p;
	struct clk *clk_e;
	int i;
	struct clk *clkp;

	clk_xtal = clk_get(NULL, "xtal");
	if (IS_ERR(clk_xtal)) {
		printk(KERN_ERR "S3C2443: Failed to get clk_xtal\n");
		return -EINVAL;
	}

	s3c2443_clk_epll.rate = s3c2443_get_epll(__raw_readl(S3C2443_EPLLCON), clk_xtal->rate);

	printk("S3C2443: Clock Support, EPLL %lu.%03lu MHz\n",
	       print_mhz(s3c2443_clk_epll.rate));

	clk_h = clk_get(NULL, "hclk");
	clk_p = clk_get(NULL, "pclk");

	if (IS_ERR(clk_h) || IS_ERR(clk_p)) {
		printk(KERN_ERR "S3C2443: Failed to get parent clocks\n");
		return -EINVAL;
	}

	clk_e = &s3c2443_clk_epll;
	s3c24xx_register_clock( clk_e );

	for (i = 0; i < ARRAY_SIZE(s3c2443_init_clocks); ++i) {
		clkp = &s3c2443_init_clocks[i];
		switch ((int)clkp->parent) {
		case (int)CLK_H: clkp->parent = clk_h; break;
		case (int)CLK_P: clkp->parent = clk_p; break;
		case (int)CLK_E: clkp->parent = clk_e; break;
		default:         clkp->parent = NULL;  break;
		}
		
		//enable the dma clocking foir channels used by audio, 50411 bl is known to forget to do this
		if( clkp->ctrlbit == S3C2443_HCLKCON_DMA1 || clkp->ctrlbit == S3C2443_HCLKCON_DMA2 )
		  clkp->enable( clkp, 1 );
		  
		s3c24xx_register_clock(clkp);
		
	}

	return 0;
}

static struct sysdev_driver s3c2443_clk_driver = {
	.add	= s3c2443_clk_add,
};

static __init int s3c24xx_clk_driver(void)
{
	return sysdev_driver_register(&s3c2443_sysclass, &s3c2443_clk_driver);
}

arch_initcall(s3c24xx_clk_driver);
