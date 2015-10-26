/*
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Benoit Leffray <benoit.leffray@tomtom.com>
 *         Martin Jackson <martin.jackson@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

/* Two Flipflop platform devices: B0 has a real flipflop, A0 uses some unused
 * RTC registers.
 */
 
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>
#include <asm/arch/rtc_cpuapi4760.h>

#include <plat/flipflop.h>

/* Bit 30 of this register is our flipflop (on B0 only). The rest of the
 * register doesn't belong to us, so it isn't registered as a resource :P
 */
#define FLIPFLOP_ADDR IO_ADDRESS(PML_REG_BASE_ADDR)
#define FLIPFLOP_BIT 30

static int  irvine_flipflop_init(struct platform_device *pdev);
static void irvine_flipflop_set(int v);
static int  irvine_flipflop_get(void);

/* Some unused bits in the RTC registers can be used on a0 as a 'flipflop'
 * Note there are race conditions between this and the RTC driver, so this is
 * not suitable for production code
 */
static struct flipflop_info irvine_flipflop_info = {
	.init 	   = irvine_flipflop_init,
	.set_value = irvine_flipflop_set,
	.get_value = irvine_flipflop_get
};

struct platform_device irvine_device_flipflop =
{
	.name   = "flipflop",
	.id	= -1,
	.dev = {
		.platform_data = &irvine_flipflop_info,
	},
};

static int irvine_flipflop_init(struct platform_device *pdev)
{
	return 0;
}

static void irvine_flipflop_set(int v)
{
	unsigned long flags, t;

	local_irq_save(flags);
	t = readl(FLIPFLOP_ADDR);
	v? __set_bit(FLIPFLOP_BIT, &t) : __clear_bit(FLIPFLOP_BIT, &t);
	writel(t, FLIPFLOP_ADDR);
	local_irq_restore(flags);
}

static int irvine_flipflop_get(void)
{
	unsigned long flags, t;
	int r;

	local_irq_save(flags);
	t = readl(FLIPFLOP_ADDR);
	r = test_bit(FLIPFLOP_BIT, &t);
	local_irq_restore(flags);
	return r;
}

