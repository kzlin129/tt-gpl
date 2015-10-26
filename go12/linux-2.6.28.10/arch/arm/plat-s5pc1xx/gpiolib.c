/* arch/arm/plat-s5pc1xx/gpiolib.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5PC1XX - GPIOlib support 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio-core.h>

#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/regs-gpio.h>

#define OFF_GPCON	(0x00)
#define OFF_GPDAT	(0x04)

#define con_4bit_shift(__off) ((__off) * 4)

#if 1
#define gpio_dbg(x...) do { } while(0)
#else
#define gpio_dbg(x...) printk(KERN_DEBUG ## x)
#endif

/* The s5pc1xx_gpiolib routines are to control the gpio banks where
 * the gpio configuration register (GPxCON) has 4 bits per GPIO, as the
 * following example:
 *
 * base + 0x00: Control register, 4 bits per gpio
 *	        gpio n: 4 bits starting at (4*n)
 *		0000 = input, 0001 = output, others mean special-function
 * base + 0x04: Data register, 1 bit per gpio
 *		bit n: data bit n
 *
 * Note, since the data register is one bit per gpio and is at base + 0x4
 * we can use s3c_gpiolib_get and s3c_gpiolib_set to change the state of
 * the output.
*/

static int s5pc1xx_gpiolib_input(struct gpio_chip *chip, unsigned offset)
{
	struct s3c_gpio_chip *ourchip = to_s3c_gpio(chip);
	void __iomem *base = ourchip->base;
	unsigned long con;

	con = __raw_readl(base + OFF_GPCON);
	con &= ~(0xf << con_4bit_shift(offset));
	__raw_writel(con, base + OFF_GPCON);

	gpio_dbg("%s: %p: CON now %08lx\n", __func__, base, con);

	return 0;
}

static int s5pc1xx_gpiolib_output(struct gpio_chip *chip,
				       unsigned offset, int value)
{
	struct s3c_gpio_chip *ourchip = to_s3c_gpio(chip);
	void __iomem *base = ourchip->base;
	unsigned long con;
	unsigned long dat;

	con = __raw_readl(base + OFF_GPCON);
	con &= ~(0xf << con_4bit_shift(offset));
	con |= 0x1 << con_4bit_shift(offset);

	dat = __raw_readl(base + OFF_GPDAT);
	if (value)
		dat |= 1 << offset;
	else
		dat &= ~(1 << offset);

	__raw_writel(dat, base + OFF_GPDAT);
	__raw_writel(con, base + OFF_GPCON);
	__raw_writel(dat, base + OFF_GPDAT);

	gpio_dbg("%s: %p: CON %08lx, DAT %08lx\n", __func__, base, con, dat);

	return 0;
}

static struct s3c_gpio_cfg gpio_cfg = {
	.cfg_eint	= 0xf,
	.set_config	= s3c_gpio_setcfg_s5pc1xx,
	.set_pull	= s3c_gpio_setpull_updown,
	.get_pull	= s3c_gpio_getpull_updown,
	.set_pin	= s3c_gpio_setpin_updown,
};

static struct s3c_gpio_cfg gpio_cfg_noint = {
	.set_config	= s3c_gpio_setcfg_s5pc1xx,
	.set_pull	= s3c_gpio_setpull_updown,
	.get_pull	= s3c_gpio_getpull_updown,
	.set_pin	= s3c_gpio_setpin_updown,
};

static struct s3c_gpio_chip gpio_chips[] = {
	{
		.base	= S5PC1XX_GPA0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPA0(0),
			.ngpio	= S5PC1XX_GPIO_A0_NR,
			.label	= "GPA0",
		},
	}, {
		.base	= S5PC1XX_GPA1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPA1(0),
			.ngpio	= S5PC1XX_GPIO_A1_NR,
			.label	= "GPA1",
		},
	}, {
		.base	= S5PC1XX_GPB_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPB(0),
			.ngpio	= S5PC1XX_GPIO_B_NR,
			.label	= "GPB",
		},
	}, 
#if defined(CONFIG_CPU_S5PC100)
	{
		.base	= S5PC1XX_GPC_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPC(0),
			.ngpio	= S5PC1XX_GPIO_C_NR,
			.label	= "GPC",
		},
	}, {
		.base	= S5PC1XX_GPD_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPD(0),
			.ngpio	= S5PC1XX_GPIO_D_NR,
			.label	= "GPD",
		},
	}, 
#elif defined(CONFIG_CPU_S5PC110)
	{
		.base	= S5PC1XX_GPC0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPC0(0),
			.ngpio	= S5PC1XX_GPIO_C0_NR,
			.label	= "GPC0",
		},
	}, {
		.base	= S5PC1XX_GPC1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPC1(0),
			.ngpio	= S5PC1XX_GPIO_C1_NR,
			.label	= "GPC1",
		},
	}, {
		.base	= S5PC1XX_GPD0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPD0(0),
			.ngpio	= S5PC1XX_GPIO_D0_NR,
			.label	= "GPD0",
		},
	}, {
		.base	= S5PC1XX_GPD1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPD1(0),
			.ngpio	= S5PC1XX_GPIO_D1_NR,
			.label	= "GPD1",
		},
	}, 

#else
#error "CPU type should be defined(S5PC100, S5PC110)"
#endif

	{
		.base	= S5PC1XX_GPE0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPE0(0),
			.ngpio	= S5PC1XX_GPIO_E0_NR,
			.label	= "GPE0",
		},
	}, {
		.base	= S5PC1XX_GPE1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPE1(0),
			.ngpio	= S5PC1XX_GPIO_E1_NR,
			.label	= "GPE1",
		},
	}, {
		.base	= S5PC1XX_GPF0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPF0(0),
			.ngpio	= S5PC1XX_GPIO_F0_NR,
			.label	= "GPF0",
		},
	}, {
		.base	= S5PC1XX_GPF1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPF1(0),
			.ngpio	= S5PC1XX_GPIO_F1_NR,
			.label	= "GPF1",
		},
	}, {
		.base	= S5PC1XX_GPF2_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPF2(0),
			.ngpio	= S5PC1XX_GPIO_F2_NR,
			.label	= "GPF2",
		},
	}, {
		.base	= S5PC1XX_GPF3_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPF3(0),
			.ngpio	= S5PC1XX_GPIO_F3_NR,
			.label	= "GPF3",
		},
	}, {
		.base	= S5PC1XX_GPG0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPG0(0),
			.ngpio	= S5PC1XX_GPIO_G0_NR,
			.label	= "GPG0",
		},
	}, {
		.base	= S5PC1XX_GPG1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPG1(0),
			.ngpio	= S5PC1XX_GPIO_G1_NR,
			.label	= "GPG1",
		},
	}, {
		.base	= S5PC1XX_GPG2_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPG2(0),
			.ngpio	= S5PC1XX_GPIO_G2_NR,
			.label	= "GPG2",
		},
	}, {
		.base	= S5PC1XX_GPG3_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPG3(0),
			.ngpio	= S5PC1XX_GPIO_G3_NR,
			.label	= "GPG3",
		},
	}, {
		.base	= S5PC1XX_GPH0_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPH0(0),
			.ngpio	= S5PC1XX_GPIO_H0_NR,
			.label	= "GPH0",
		},
	}, {
		.base	= S5PC1XX_GPH1_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPH1(0),
			.ngpio	= S5PC1XX_GPIO_H1_NR,
			.label	= "GPH1",
		},
	}, {
		.base	= S5PC1XX_GPH2_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPH2(0),
			.ngpio	= S5PC1XX_GPIO_H2_NR,
			.label	= "GPH2",
		},
	}, {
		.base	= S5PC1XX_GPH3_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPH3(0),
			.ngpio	= S5PC1XX_GPIO_H3_NR,
			.label	= "GPH3",
		},
	}, {
		.base	= S5PC1XX_GPI_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPI(0),
			.ngpio	= S5PC1XX_GPIO_I_NR,
			.label	= "GPI",
		},
	}, {
		.base	= S5PC1XX_GPJ0_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPJ0(0),
			.ngpio	= S5PC1XX_GPIO_J0_NR,
			.label	= "GPJ0",
		},
	}, {
		.base	= S5PC1XX_GPJ1_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPJ1(0),
			.ngpio	= S5PC1XX_GPIO_J1_NR,
			.label	= "GPJ1",
		},
	}, {
		.base	= S5PC1XX_GPJ2_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPJ2(0),
			.ngpio	= S5PC1XX_GPIO_J2_NR,
			.label	= "GPJ2",
		},
	}, {
		.base	= S5PC1XX_GPJ3_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPJ3(0),
			.ngpio	= S5PC1XX_GPIO_J3_NR,
			.label	= "GPJ3",
		},
	}, {
		.base	= S5PC1XX_GPJ4_BASE,
		.config	= &gpio_cfg,
		.chip	= {
			.base	= S5PC1XX_GPJ4(0),
			.ngpio	= S5PC1XX_GPIO_J4_NR,
			.label	= "GPJ4",
		},
	}, 
#if defined(CONFIG_CPU_S5PC100)
	{
		.base	= S5PC1XX_GPK0_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPK0(0),
			.ngpio	= S5PC1XX_GPIO_K0_NR,
			.label	= "GPK0",
		},
	}, {
		.base	= S5PC1XX_GPK1_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPK1(0),
			.ngpio	= S5PC1XX_GPIO_K1_NR,
			.label	= "GPK1",
		},
	}, {
		.base	= S5PC1XX_GPK2_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPK2(0),
			.ngpio	= S5PC1XX_GPIO_K2_NR,
			.label	= "GPK2",
		},
	}, {
		.base	= S5PC1XX_GPK3_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_GPK3(0),
			.ngpio	= S5PC1XX_GPIO_K3_NR,
			.label	= "GPK3",
		},
	}, {
		.base	= S5PC1XX_MP00_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP00(0),
			.ngpio	= S5PC1XX_GPIO_MP00_NR,
			.label	= "MP00",
		},
	}, 
#endif
	{
		.base	= S5PC1XX_MP01_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP01(0),
			.ngpio	= S5PC1XX_GPIO_MP01_NR,
			.label	= "MP01",
		},
	}, {
		.base	= S5PC1XX_MP02_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP02(0),
			.ngpio	= S5PC1XX_GPIO_MP02_NR,
			.label	= "MP02",
		},
	}, {
		.base	= S5PC1XX_MP03_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP03(0),
			.ngpio	= S5PC1XX_GPIO_MP03_NR,
			.label	= "MP03",
		},
	}, {
		.base	= S5PC1XX_MP04_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP04(0),
			.ngpio	= S5PC1XX_GPIO_MP04_NR,
			.label	= "MP04",
		},
	}, 
#if defined(CONFIG_CPU_S5PC110)
	{
		.base	= S5PC1XX_MP05_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP05(0),
			.ngpio	= S5PC1XX_GPIO_MP05_NR,
			.label	= "MP05",
		},
	}, {
		.base	= S5PC1XX_MP06_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP06(0),
			.ngpio	= S5PC1XX_GPIO_MP06_NR,
			.label	= "MP06",
		},
	}, {
		.base	= S5PC1XX_MP07_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP07(0),
			.ngpio	= S5PC1XX_GPIO_MP07_NR,
			.label	= "MP07",
		},
	}, {
		.base	= S5PC1XX_MP10_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP10(0),
			.ngpio	= S5PC1XX_GPIO_MP10_NR,
			.label	= "MP10",
		},
	}, {
		.base	= S5PC1XX_MP11_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP11(0),
			.ngpio	= S5PC1XX_GPIO_MP11_NR,
			.label	= "MP11",
		},
	}, {
		.base	= S5PC1XX_MP12_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP12(0),
			.ngpio	= S5PC1XX_GPIO_MP12_NR,
			.label	= "MP12",
		},
	}, {
		.base	= S5PC1XX_MP13_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP13(0),
			.ngpio	= S5PC1XX_GPIO_MP13_NR,
			.label	= "MP13",
		},
	}, {
		.base	= S5PC1XX_MP14_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP14(0),
			.ngpio	= S5PC1XX_GPIO_MP14_NR,
			.label	= "MP14",
		},
	}, {
		.base	= S5PC1XX_MP15_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP15(0),
			.ngpio	= S5PC1XX_GPIO_MP15_NR,
			.label	= "MP15",
		},
	}, {
		.base	= S5PC1XX_MP16_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP16(0),
			.ngpio	= S5PC1XX_GPIO_MP16_NR,
			.label	= "MP16",
		},
	}, {
		.base	= S5PC1XX_MP17_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP17(0),
			.ngpio	= S5PC1XX_GPIO_MP17_NR,
			.label	= "MP17",
		},
	}, {
		.base	= S5PC1XX_MP18_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP18(0),
			.ngpio	= S5PC1XX_GPIO_MP18_NR,
			.label	= "MP18",
		},
	}, {
		.base	= S5PC1XX_MP20_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP20(0),
			.ngpio	= S5PC1XX_GPIO_MP20_NR,
			.label	= "MP120",
		},
	}, {
		.base	= S5PC1XX_MP21_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP21(0),
			.ngpio	= S5PC1XX_GPIO_MP21_NR,
			.label	= "MP21",
		},
	}, {
		.base	= S5PC1XX_MP22_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP22(0),
			.ngpio	= S5PC1XX_GPIO_MP22_NR,
			.label	= "MP22",
		},
	}, {
		.base	= S5PC1XX_MP23_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP23(0),
			.ngpio	= S5PC1XX_GPIO_MP23_NR,
			.label	= "MP23",
		},
	}, {
		.base	= S5PC1XX_MP24_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP24(0),
			.ngpio	= S5PC1XX_GPIO_MP24_NR,
			.label	= "MP24",
		},
	}, {
		.base	= S5PC1XX_MP25_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP25(0),
			.ngpio	= S5PC1XX_GPIO_MP25_NR,
			.label	= "MP25",
		},
	}, {
		.base	= S5PC1XX_MP26_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP26(0),
			.ngpio	= S5PC1XX_GPIO_MP26_NR,
			.label	= "MP26",
		},
	}, {
		.base	= S5PC1XX_MP27_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP27(0),
			.ngpio	= S5PC1XX_GPIO_MP27_NR,
			.label	= "MP27",
		},
	}, {
		.base	= S5PC1XX_MP28_BASE,
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= S5PC1XX_MP28(0),
			.ngpio	= S5PC1XX_GPIO_MP28_NR,
			.label	= "MP28",
		},
	},

#endif
};

static __init void s5pc1xx_gpiolib_link(struct s3c_gpio_chip *chip)
{
	chip->chip.direction_input = s5pc1xx_gpiolib_input;
	chip->chip.direction_output = s5pc1xx_gpiolib_output;
}

static __init void s5pc1xx_gpiolib_add(struct s3c_gpio_chip *chips,
				       int nr_chips,
				       void (*fn)(struct s3c_gpio_chip *))
{
	for (; nr_chips > 0; nr_chips--, chips++) {
		if (fn)
			(fn)(chips);
		s3c_gpiolib_add(chips);
	}
}

static __init int s5pc1xx_gpiolib_init(void)
{
	s5pc1xx_gpiolib_add(gpio_chips, ARRAY_SIZE(gpio_chips),
			    s5pc1xx_gpiolib_link);

	return 0;
}

arch_initcall(s5pc1xx_gpiolib_init);
