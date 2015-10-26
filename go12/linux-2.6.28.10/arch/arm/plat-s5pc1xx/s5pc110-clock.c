/* linux/arch/arm/plat-s5pc1xx/s5pc100-clock.c
 *
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/cpu-freq.h>

#include <plat/regs-clock.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/pll.h>

/* For S5PC100 EVT0 workaround
 * When we modify DIVarm value to change ARM speed D0_BUS parent clock is also changed
 * If we prevent from unwanted changing of bus clock, we should modify DIVd0_bus value also.
 */
#define PREVENT_BUS_CLOCK_CHANGE

extern void ChangeClkDiv0(unsigned int val);

/* fin_apll, fin_mpll and fin_epll are all the same clock, which we call
 * ext_xtal_mux for want of an actual name from the manual.
*/
static unsigned long s5pc1xx_roundrate_clksrc(struct clk *clk, unsigned long rate);

struct clk clk_ext_xtal_mux = {
	.name		= "ext_xtal",
	.id		= -1,
};

#define clk_fin_apll	clk_ext_xtal_mux
#define clk_fin_mpll	clk_ext_xtal_mux
#define clk_fin_epll	clk_ext_xtal_mux
#define clk_fin_vpll	clk_ext_xtal_mux

#define clk_fout_mpll	clk_mpll

struct clk_sources {
	unsigned int	nr_sources;
	struct clk	**sources;
};

struct clksrc_clk {
	struct clk		clk;
	unsigned int		mask;
	unsigned int		shift;

	struct clk_sources	*sources;

	unsigned int		divider_shift;
	void __iomem		*reg_divider;
	void __iomem		*reg_source;
};

struct clk clk_srclk = {
	.name		= "srclk",
	.id		= -1,
};

struct clk clk_fout_apll = {
	.name		= "fout_apll",
	.id		= -1,
};

static struct clk *clk_src_apll_list[] = {
	[0] = &clk_fin_apll,
	[1] = &clk_fout_apll,
};

static struct clk_sources clk_src_apll = {
	.sources	= clk_src_apll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_apll_list),
};

struct clksrc_clk clk_mout_apll = {
	.clk	= {
		.name		= "mout_apll",
		.id		= -1,
	},
	.shift		= S5P_CLKSRC0_APLL_SHIFT,
	.mask		= S5P_CLKSRC0_APLL_MASK,
	.sources	= &clk_src_apll,
	.reg_source	= S5P_CLK_SRC0,
};


static unsigned long s5pc1xx_clk_doutapll_get_rate(struct clk *clk)
{
  	unsigned long rate = clk_get_rate(clk->parent);

	rate /= (((__raw_readl(S5P_CLK_DIV0) & S5P_CLKDIV0_APLL_MASK) >> S5P_CLKDIV0_APLL_SHIFT) + 1);

	return rate;
}

int s5pc1xx_clk_doutapll_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *temp_clk = clk;
	unsigned int div;
	u32 val;

	rate = clk_round_rate(temp_clk, rate);
	div = clk_get_rate(temp_clk->parent) / rate;

	val = __raw_readl(S5P_CLK_DIV0);
	val &=~ S5P_CLKDIV0_APLL_MASK;
	val |= (div - 1) << S5P_CLKDIV0_APLL_SHIFT;
	__raw_writel(val, S5P_CLK_DIV0);

	return 0;
}

struct clk clk_dout_apll = {
	.name = "dout_apll",
	.id = -1,
	.parent = &clk_mout_apll.clk,
	.get_rate = s5pc1xx_clk_doutapll_get_rate,
	.set_rate = s5pc1xx_clk_doutapll_set_rate,
};

static unsigned long s5pc1xx_clk_doutarm_get_rate(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	rate /= (((__raw_readl(S5P_CLK_DIV0) & S5P_CLKDIV0_APLL_MASK) >> S5P_CLKDIV0_APLL_SHIFT) + 1);

	return rate;
}

static unsigned long s5pc1xx_doutarm_roundrate(struct clk *clk,
					      unsigned long rate)
{
	unsigned long parent_rate = clk_get_rate(clk->parent);
	int div;

	if (rate > parent_rate)
		rate = parent_rate;
	else {
		div = parent_rate / rate;

		div ++;
		
		rate = parent_rate / div;
	}

	return rate;
}

int s5pc1xx_clk_doutarm_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *temp_clk = clk;
	unsigned int div_arm;
	unsigned int val;
#ifdef PREVENT_BUS_CLOCK_CHANGE
	unsigned int d0_bus_ratio, arm_ratio_old, ratio;
	val = __raw_readl(S5P_CLK_DIV0);
	d0_bus_ratio = (val & S5P_CLKDIV0_HCLK200_MASK) >> S5P_CLKDIV0_HCLK200_SHIFT;
	arm_ratio_old = (val & S5P_CLKDIV0_APLL_MASK) >> S5P_CLKDIV0_APLL_SHIFT;
	ratio = (arm_ratio_old + 1) * (d0_bus_ratio + 1);
#endif
	div_arm = clk_get_rate(temp_clk->parent) / rate;
	
#ifndef PREVENT_BUS_CLOCK_CHANGE
	val = __raw_readl(S5P_CLK_DIV0);
	val &=~ S5P_CLKDIV0_APLL_MASK;
	val |= (div_arm - 1) << S5P_CLKDIV0_APLL_SHIFT;
#else	
	d0_bus_ratio = (ratio / div_arm) -1;
	val &=~ (S5P_CLKDIV0_APLL_MASK | S5P_CLKDIV0_HCLK200_MASK);
	val |= (div_arm - 1) << S5P_CLKDIV0_APLL_SHIFT;
	val |= d0_bus_ratio << S5P_CLKDIV0_HCLK200_SHIFT;
	//printk(KERN_INFO "d0_bus_ratio : %08d ,arm_ratio: %08d\n",d0_bus_ratio, (div_arm-1));
	
#endif

#ifdef PREVENT_BUS_CLOCK_CHANGE

#if 0
	iter = 0x4000;
	flag = __raw_readl(S5P_CLK_DIV0);
	__asm__  __volatile__ ("mcr p15, 0, %0, c7, c10, 4" \
				    : : "r" (flag) : "memory");
	__asm__  __volatile__ ("mcr p15, 0, %0, c7, c10, 5" \
				    : : "r" (flag) : "memory");
	
	do {
		iter--;
		if(iter == 0x2000) {
			__asm__  __volatile__ ("mcr p15, 0, %0, c7, c10, 4" \
						    : : "r" (flag) : "memory");
			__asm__  __volatile__ ("mcr p15, 0, %0, c7, c10, 5" \
						    : : "r" (flag) : "memory");
			__raw_writel(val, S5P_CLK_DIV0);			
		}
		if(iter <= 0)
			break;
		
	} while(1);
#else
	/* Clock Down */
	if(arm_ratio_old < (div_arm - 1)) {
		val = __raw_readl(S5P_CLK_DIV0);
		val &=~ S5P_CLKDIV0_APLL_MASK;
		val |= (div_arm - 1) << S5P_CLKDIV0_APLL_SHIFT;
		__raw_writel(val, S5P_CLK_DIV0);

		val = __raw_readl(S5P_CLK_DIV0);
		val &=~ S5P_CLKDIV0_HCLK200_MASK;
		val |= d0_bus_ratio << S5P_CLKDIV0_HCLK200_SHIFT;
		__raw_writel(val, S5P_CLK_DIV0);
		
	} else {
		val = __raw_readl(S5P_CLK_DIV0);
		val &=~ S5P_CLKDIV0_HCLK200_MASK;
		val |= d0_bus_ratio << S5P_CLKDIV0_HCLK200_SHIFT;
		__raw_writel(val, S5P_CLK_DIV0);

		val = __raw_readl(S5P_CLK_DIV0);
		val &=~ S5P_CLKDIV0_APLL_MASK;
		val |= (div_arm - 1) << S5P_CLKDIV0_APLL_SHIFT;
		__raw_writel(val, S5P_CLK_DIV0);
	}

#endif

#else
	__raw_writel(val, S5P_CLK_DIV0);
#endif
	return 0;
}

struct clk clk_dout_arm = {
	.name = "dout_arm",
	.id = -1,
	.parent = &clk_dout_apll,
	.get_rate = s5pc1xx_clk_doutarm_get_rate,
	.set_rate = s5pc1xx_clk_doutarm_set_rate,
	.round_rate	= s5pc1xx_doutarm_roundrate,
};

struct clk clk_fout_epll = {
	.name		= "fout_epll",
	.id		= -1,
};

static struct clk *clk_src_epll_list[] = {
	[0] = &clk_fin_epll,
	[1] = &clk_fout_epll,
};

static struct clk_sources clk_src_epll = {
	.sources	= clk_src_epll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_epll_list),
};

struct clksrc_clk clk_mout_epll = {
	.clk	= {
		.name		= "mout_epll",
		.id		= -1,
	},
	.shift		= S5P_CLKSRC0_EPLL_SHIFT,
	.mask		= S5P_CLKSRC0_EPLL_MASK,
	.sources	= &clk_src_epll,
	.reg_source	= S5P_CLK_SRC0,
};

static struct clk *clk_src_vpll_list[] = {
	[0] = &clk_27m,
	[1] = &clk_srclk,
};

static struct clk_sources clk_src_vpll = {
	.sources	= clk_src_vpll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_vpll_list),
};

struct clksrc_clk clk_mout_vpll = {
	.clk	= {
		.name		= "mout_vpll",
		.id		= -1,
	},
	.shift		= S5P_CLKSRC0_VPLL_SHIFT,
	.mask		= S5P_CLKSRC0_VPLL_MASK,
	.sources	= &clk_src_vpll,
	.reg_source	= S5P_CLK_SRC0,
};

static struct clk *clk_src_mpll_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_fout_mpll,
};

static struct clk_sources clk_src_mpll = {
	.sources	= clk_src_mpll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_mpll_list),
};

struct clksrc_clk clk_mout_mpll = {
	.clk = {
		.name		= "mout_mpll",
		.id		= -1,
	},
	.shift		= S5P_CLKSRC0_MPLL_SHIFT,
	.mask		= S5P_CLKSRC0_MPLL_MASK,
	.sources	= &clk_src_mpll,
	.reg_source	= S5P_CLK_SRC0,
};

static unsigned long s5pc1xx_clk_doutmpll_get_rate(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	return rate;
}

struct clk clk_dout_ck166 = {
	.name		= "dout_ck166",
	.id		= -1,
	.parent		= &clk_mout_mpll.clk,
	.get_rate	= s5pc1xx_clk_doutmpll_get_rate,
};

static unsigned long s5pc1xx_clk_sclk_hdmi_get_rate(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	return rate;
}

struct clk clk_sclk_hdmi = {
	.name		= "sclk_hdmi",
	.id		= -1,
	.parent		= &clk_mout_vpll.clk,
	.get_rate	= s5pc1xx_clk_sclk_hdmi_get_rate,
};

static struct clk *clkset_spi_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	&clk_fin_epll,
	&clk_mout_vpll.clk,
};

static struct clk_sources clkset_spi = {
	.sources	= clkset_spi_list,
	.nr_sources	= ARRAY_SIZE(clkset_spi_list),
};

static struct clk *clkset_uart_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	NULL,
	NULL,
};

static struct clk_sources clkset_uart = {
	.sources	= clkset_uart_list,
	.nr_sources	= ARRAY_SIZE(clkset_uart_list),
};

static struct clk *clkset_mmc0_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	&clk_fin_epll,
	NULL,
};

static struct clk_sources clkset_mmc0 = {
	.sources	= clkset_mmc0_list,
	.nr_sources	= ARRAY_SIZE(clkset_mmc0_list),
};

static struct clk *clkset_mmc1_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	&clk_fin_epll,
	&clk_mout_vpll.clk,
};

static struct clk_sources clkset_mmc1 = {
	.sources	= clkset_mmc1_list,
	.nr_sources	= ARRAY_SIZE(clkset_mmc1_list),
};

static struct clk *clkset_mmc2_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	&clk_fin_epll,
	&clk_mout_vpll.clk,
};

static struct clk_sources clkset_mmc2 = {
	.sources	= clkset_mmc2_list,
	.nr_sources	= ARRAY_SIZE(clkset_mmc2_list),
};

static struct clk *clkset_lcd_list[] = {
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	&clk_mout_vpll.clk,
};

static struct clk_sources clkset_lcd = {
	.sources	= clkset_lcd_list,
	.nr_sources	= ARRAY_SIZE(clkset_lcd_list),
};


static struct clk *clkset_pwi_list[] = {
	&clk_srclk,
	&clk_mout_epll.clk,
	&clk_mout_mpll.clk,
	NULL,
};

static struct clk_sources clkset_pwi = {
	.sources	= clkset_pwi_list,
	.nr_sources	= ARRAY_SIZE(clkset_pwi_list),
};

/* The peripheral clocks are all controlled via clocksource followed
 * by an optional divider and gate stage. We currently roll this into
 * one clock which hides the intermediate clock from the mux.
 *
 * Note, the JPEG clock can only be an even divider...
 *
 * The scaler and LCD clocks depend on the S3C64XX version, and also
 * have a common parent divisor so are not included here.
 */

static inline struct clksrc_clk *to_clksrc(struct clk *clk)
{
	return container_of(clk, struct clksrc_clk, clk);
}

static unsigned long s5pc1xx_getrate_clksrc(struct clk *clk)
{
	struct clksrc_clk *sclk = to_clksrc(clk);
	unsigned long rate = clk_get_rate(clk->parent);
	u32 clkdiv = __raw_readl(sclk->reg_divider);

	clkdiv >>= sclk->divider_shift;
	clkdiv &= 0xf;
	clkdiv++;

	rate /= clkdiv;
	return rate;
}

static int s5pc1xx_setrate_clksrc(struct clk *clk, unsigned long rate)
{
	struct clksrc_clk *sclk = to_clksrc(clk);
	void __iomem *reg = sclk->reg_divider;
	unsigned int div;
	u32 val;

	rate = clk_round_rate(clk, rate);
	div = clk_get_rate(clk->parent) / rate;

	val = __raw_readl(reg);
	val &= ~sclk->mask;
	val |= (div - 1) << sclk->shift;
	__raw_writel(val, reg);

	return 0;
}

static int s5pc1xx_setparent_clksrc(struct clk *clk, struct clk *parent)
{
	struct clksrc_clk *sclk = to_clksrc(clk);
	struct clk_sources *srcs = sclk->sources;
	u32 clksrc = __raw_readl(sclk->reg_source);
	int src_nr = -1;
	int ptr;

	for (ptr = 0; ptr < srcs->nr_sources; ptr++)
		if (srcs->sources[ptr] == parent) {
			src_nr = ptr;
			break;
		}

	if (src_nr >= 0) {
		clksrc &= ~sclk->mask;
		clksrc |= src_nr << sclk->shift;

		__raw_writel(clksrc, sclk->reg_source);
		return 0;
	}

	return -EINVAL;
}

static unsigned long s5pc1xx_roundrate_clksrc(struct clk *clk,
					      unsigned long rate)
{
	unsigned long parent_rate = clk_get_rate(clk->parent);
	int div;

	if (rate > parent_rate)
		rate = parent_rate;
	else {
		div = rate / parent_rate;

		if (div == 0)
			div = 1;
		if (div > 16)
			div = 16;

		rate = parent_rate / div;
	}

	return rate;
}

static struct clksrc_clk clk_mmc0 = {
	.clk	= {
		.name		= "mmc_bus",
		.id		= 0,
		.ctrlbit        = S5P_CLKGATE_SCLK0_MMC0,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC4_MMC0_SHIFT,
	.mask		= S5P_CLKSRC4_MMC0_MASK,
	.sources	= &clkset_mmc0,
	.divider_shift	= S5P_CLKDIV4_MMC0_SHIFT,
	.reg_divider	= S5P_CLK_DIV4,
	.reg_source	= S5P_CLK_SRC4,
};

static struct clksrc_clk clk_mmc1 = {
	.clk	= {
		.name		= "mmc_bus",
		.id		= 1,
		.ctrlbit        = S5P_CLKGATE_SCLK0_MMC1,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC4_MMC1_SHIFT,
	.mask		= S5P_CLKSRC4_MMC1_MASK,
	.sources	= &clkset_mmc1,
	.divider_shift	= S5P_CLKDIV4_MMC1_SHIFT,
	.reg_divider	= S5P_CLK_DIV4,
	.reg_source	= S5P_CLK_SRC4,
};

static struct clksrc_clk clk_mmc2 = {
	.clk	= {
		.name		= "mmc_bus",
		.id		= 2,
		.ctrlbit        = S5P_CLKGATE_SCLK0_MMC2,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC4_MMC2_SHIFT,
	.mask		= S5P_CLKSRC4_MMC2_MASK,
	.sources	= &clkset_mmc2,
	.divider_shift	= S5P_CLKDIV4_MMC2_SHIFT,
	.reg_divider	= S5P_CLK_DIV4,
	.reg_source	= S5P_CLK_SRC4,
};

static struct clksrc_clk clk_uart_uclk1 = {
	.clk	= {
		.name		= "uclk1",
		.id		= -1,
		.ctrlbit        = S5P_CLKGATE_SCLK0_UART0,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC4_UART0_SHIFT,
	.mask		= S5P_CLKSRC4_UART0_MASK,
	.sources	= &clkset_uart,
	.divider_shift	= S5P_CLKDIV4_UART0_SHIFT,
	.reg_divider	= S5P_CLK_DIV4,
	.reg_source	= S5P_CLK_SRC4,
};

static struct clksrc_clk clk_spi0 = {
	.clk	= {
		.name		= "spi-bus",
		.id		= 0,
		.ctrlbit        = S5P_CLKGATE_SCLK0_SPI0,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC5_SPI0_SHIFT,
	.mask		= S5P_CLKSRC5_SPI0_MASK,
	.sources	= &clkset_spi,
	.divider_shift	= S5P_CLKDIV5_SPI0_SHIFT,
	.reg_divider	= S5P_CLK_DIV5,
	.reg_source	= S5P_CLK_SRC5,
};

static struct clksrc_clk clk_spi1 = {
	.clk	= {
		.name		= "spi-bus",
		.id		= 1,
		.ctrlbit        = S5P_CLKGATE_SCLK0_SPI1,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC5_SPI1_SHIFT,
	.mask		= S5P_CLKSRC5_SPI1_MASK,
	.sources	= &clkset_spi,
	.divider_shift	= S5P_CLKDIV5_SPI1_SHIFT,
	.reg_divider	= S5P_CLK_DIV5,
	.reg_source	= S5P_CLK_SRC5,
};

static struct clksrc_clk clk_spi2 = {
	.clk	= {
		.name		= "spi-bus",
		.id		= 2,
		.ctrlbit        = S5P_CLKGATE_SCLK0_SPI2,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC5_SPI2_SHIFT,
	.mask		= S5P_CLKSRC5_SPI2_MASK,
	.sources	= &clkset_spi,
	.divider_shift	= S5P_CLKDIV5_SPI2_SHIFT,
	.reg_divider	= S5P_CLK_DIV5,
	.reg_source	= S5P_CLK_SRC5,
};

static struct clksrc_clk clk_pwi = {
	.clk	= {
		.name		= "sclk_pwi",
		.id		= -1,
		.ctrlbit        = S5P_CLKGATE_SCLK0_PWI,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC6_PWI_SHIFT,
	.mask		= S5P_CLKSRC6_PWI_MASK,
	.sources	= &clkset_pwi,
	.divider_shift	= S5P_CLKDIV6_PWI_SHIFT,
	.reg_divider	= S5P_CLK_DIV6,
	.reg_source	= S5P_CLK_SRC6,
};

static struct clksrc_clk clk_lcd = {
	.clk	= {
		.name		= "sclk_lcd",
		.id		= -1,
		.ctrlbit        = S5P_CLKGATE_SCLK0_FIMD,
		.enable		= s5pc1xx_sclk0_ctrl,
		.set_parent	= s5pc1xx_setparent_clksrc,
		.get_rate	= s5pc1xx_getrate_clksrc,
		.set_rate	= s5pc1xx_setrate_clksrc,
		.round_rate	= s5pc1xx_roundrate_clksrc,
	},
	.shift		= S5P_CLKSRC1_FIMD_SHIFT,
	.mask		= S5P_CLKSRC1_FIMD_MASK,
	.sources	= &clkset_lcd,
	.divider_shift	= S5P_CLKDIV1_FIMD_SHIFT,
	.reg_divider	= S5P_CLK_DIV1,
	.reg_source	= S5P_CLK_SRC1,
};

/* Clock initialisation code */

static struct clksrc_clk *init_parents[] = {
	&clk_mout_apll,
	&clk_mout_epll,
	&clk_mout_mpll,
	&clk_mout_vpll,
	&clk_mmc0,
	&clk_mmc1,
	&clk_mmc2,
	&clk_uart_uclk1,
	&clk_spi0,
	&clk_spi1,
	&clk_spi2,
	&clk_pwi,
	&clk_lcd,
};

static void __init_or_cpufreq s5pc1xx_set_clksrc(struct clksrc_clk *clk)
{
	struct clk_sources *srcs = clk->sources;
	u32 clksrc = __raw_readl(clk->reg_source);

	clksrc &= clk->mask;
	clksrc >>= clk->shift;

	if (clksrc > srcs->nr_sources || !srcs->sources[clksrc]) {
		printk(KERN_ERR "%s: bad source %d\n",
		       clk->clk.name, clksrc);
		return;
	}

	clk->clk.parent = srcs->sources[clksrc];

	printk(KERN_INFO "%s: source is %s (%d), rate is %ld\n",
	       clk->clk.name, clk->clk.parent->name, clksrc,
	       clk_get_rate(&clk->clk));
}

#define GET_DIV(clk, field) ((((clk) & field##_MASK) >> field##_SHIFT) + 1)

void __init_or_cpufreq s5pc110_setup_clocks(void)
{
	struct clk *xtal_clk;
	unsigned long xtal;
	unsigned long armclk;
	unsigned long hclk200;
	unsigned long hclk166;
	unsigned long hclk133;
	unsigned long pclk100;
	unsigned long pclk83;
	unsigned long pclk66;
	unsigned long apll;
	unsigned long mpll;
	unsigned long vpll;
	unsigned long epll;
	unsigned int ptr;
	u32 clkdiv0, clkdiv1;

	printk(KERN_DEBUG "%s: registering clocks\n", __func__);

	clkdiv0 = __raw_readl(S5P_CLK_DIV0);
	clkdiv1 = __raw_readl(S5P_CLK_DIV1);

	printk(KERN_DEBUG "%s: clkdiv0 = %08x, clkdiv1 = %08x\n", __func__, clkdiv0, clkdiv1);

	xtal_clk = clk_get(NULL, "xtal");
	BUG_ON(IS_ERR(xtal_clk));

	xtal = clk_get_rate(xtal_clk);
	clk_put(xtal_clk);

	printk(KERN_DEBUG "%s: xtal is %ld\n", __func__, xtal);

	apll = s5pc1xx_get_pll(xtal, __raw_readl(S5P_APLL_CON));
	mpll = s5pc1xx_get_pll(xtal, __raw_readl(S5P_MPLL_CON));
	epll = s5pc1xx_get_pll(xtal, __raw_readl(S5P_EPLL_CON));
	vpll = s5pc1xx_get_pll(xtal, __raw_readl(S5P_VPLL_CON));

	printk(KERN_INFO "S5PC100: PLL settings, A=%ld, M=%ld, E=%ld, H=%ld\n",
	       apll, mpll, epll, vpll);

	armclk = apll / GET_DIV(clkdiv0, S5P_CLKDIV0_APLL);
	if(__raw_readl(S5P_CLK_SRC0)&(1<<S5P_CLKSRC0_MUX200_SHIFT)) {
		hclk200 = mpll / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK200);
	} else {
		hclk200 = armclk / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK200);
	}
	if(__raw_readl(S5P_CLK_SRC0)&(1<<S5P_CLKSRC0_MUX166_SHIFT)) {
		hclk166 = apll / GET_DIV(clkdiv0, S5P_CLKDIV0_A2M);
		hclk166 = hclk166 / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK166);
	} else {
		hclk166 = mpll / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK166);
	}
	if(__raw_readl(S5P_CLK_SRC0)&(1<<S5P_CLKSRC0_MUX133_SHIFT)) {
		hclk133 = apll / GET_DIV(clkdiv0, S5P_CLKDIV0_A2M);
		hclk133 = hclk133 / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK133);
	} else {
		hclk133 = mpll / GET_DIV(clkdiv0, S5P_CLKDIV0_HCLK133);
	}
		
	pclk100 = hclk200 / GET_DIV(clkdiv0, S5P_CLKDIV0_PCLK100);
	pclk83 = hclk166 / GET_DIV(clkdiv0, S5P_CLKDIV0_PCLK83);
	pclk66 = hclk133 / GET_DIV(clkdiv0, S5P_CLKDIV0_PCLK66);


	printk(KERN_INFO "S5PC110: ARMCLK=%ld, HCLK200=%ld, HCLK166=%ld, HCLK133=%ld, PCLK100=%ld, PCLK83=%ld, PCLK66=%ld\n",
	       armclk, hclk200, hclk166, hclk133, pclk100, pclk83, pclk66);
#if 0
	clk_fout_apll.rate = apll;
	clk_fout_mpll.rate = mpll;
	clk_fout_epll.rate = epll;
	clk_mout_vpll.clk.rate = vpll;

	clk_f.rate = armclk;
	clk_h200.rate = hclk200;
	clk_p100.rate = pclk100;
	clk_h166.rate = hclk166;
	clk_p83.rate = pclk83;
	clk_h133.rate = hclk133;
	clk_p66.rate = pclk66;
	clk_h.rate = hclk133;
	clk_p.rate = pclk66;	
#else
	clk_fout_apll.rate = xtal;
	clk_fout_mpll.rate = xtal;
	clk_fout_epll.rate = xtal;
	clk_mout_vpll.clk.rate = xtal;

	clk_f.rate = xtal;
	clk_h200.rate = xtal;
	clk_p100.rate = xtal;
	clk_h166.rate = xtal;
	clk_p83.rate = xtal;
	clk_h133.rate = xtal;
	clk_p66.rate = xtal;
	clk_h.rate = xtal;
	clk_p.rate = xtal;	
#endif
	for (ptr = 0; ptr < ARRAY_SIZE(init_parents); ptr++)
		s5pc1xx_set_clksrc(init_parents[ptr]);
}

static struct clk *clks[] __initdata = {
	&clk_ext_xtal_mux,
	&clk_mout_epll.clk,
	&clk_fout_epll,
	&clk_mout_mpll.clk,
	&clk_dout_ck166,
	&clk_mout_vpll.clk,
	&clk_sclk_hdmi,
	&clk_srclk,
	&clk_mmc0.clk,
	&clk_mmc1.clk,
	&clk_mmc2.clk,
	&clk_uart_uclk1.clk,
	&clk_spi0.clk,
	&clk_spi1.clk,
	&clk_spi2.clk,
	&clk_pwi.clk,
	&clk_lcd.clk,
	&clk_dout_apll,
	&clk_dout_arm,
};

void __init s5pc110_register_clocks(void)
{
	struct clk *clkp;
	int ret;
	int ptr;

	for (ptr = 0; ptr < ARRAY_SIZE(clks); ptr++) {
		clkp = clks[ptr];
		ret = s3c24xx_register_clock(clkp);
		if (ret < 0) {
			printk(KERN_ERR "Failed to register clock %s (%d)\n",
			       clkp->name, ret);
		}
	}
}
