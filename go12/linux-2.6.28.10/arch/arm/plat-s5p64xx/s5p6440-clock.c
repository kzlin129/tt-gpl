/* linux/arch/arm/plat-s5p64xx/s5p6440-clock.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S5P6440 based common clock support
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

/* fin_apll, fin_mpll and fin_epll are all the same clock, which we call
 * ext_xtal_mux for want of an actual name from the manual.
*/

struct clk clk_ext_xtal_mux = {
	.name		= "ext_xtal",
	.id		= -1,
	.rate		= XTAL_FREQ,
};

#define clk_fin_apll clk_ext_xtal_mux
#define clk_fin_mpll clk_ext_xtal_mux
#define clk_fin_epll clk_ext_xtal_mux

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
	.shift		= S3C_CLKSRC_APLL_MOUT_SHIFT,
	.mask		= S3C_CLKSRC_APLL_MOUT,
	.sources	= &clk_src_apll,
};

static inline struct clksrc_clk *to_clksrc(struct clk *clk)
{
	return container_of(clk, struct clksrc_clk, clk);
}

static int fout_enable(struct clk *clk, int enable)
{
	unsigned int ctrlbit = clk->ctrlbit;
	unsigned int epll_con0 = __raw_readl(S3C_EPLL_CON) & ~ ctrlbit;

	if(enable)
	   __raw_writel(epll_con0 | ctrlbit, S3C_EPLL_CON);
	else
	   __raw_writel(epll_con0, S3C_EPLL_CON);

	return 0;
}

static unsigned long fout_get_rate(struct clk *clk)
{
	return clk->rate;
}

static int fout_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int epll_con0, epll_con1;

	if(clk->rate == rate)	/* Return if nothing changed */
		return 0;

	epll_con0 = __raw_readl(S3C_EPLL_CON);
	epll_con1 = __raw_readl(S3C_EPLL_CON_K);

	epll_con0 &= ~(S3C_EPLL_CON_M_MASK | S3C_EPLL_CON_P_MASK | S3C_EPLL_CON_S_MASK);
	epll_con1 &= ~(S3C_EPLL_CON_K_MASK);

	switch(rate){
	case 36000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (48 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (4 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 48000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (32 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 60000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (40 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 72000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (48 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 84000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (28 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (2 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 96000000:
			epll_con1 |= (0 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (32 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (2 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 32768000:
			epll_con1 |= (45264 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (43 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (4 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 45158000:
			epll_con1 |= (6903 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (30 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 49152000:
			epll_con1 |= (50332 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (32 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 67738000:
			epll_con1 |= (10398 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (45 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	case 73728000:
			epll_con1 |= (9961 << S3C_EPLL_CON_K_SHIFT);
			epll_con0 |= (49 << S3C_EPLL_CON_M_SHIFT) |
					(1 << S3C_EPLL_CON_P_SHIFT) | (3 << S3C_EPLL_CON_S_SHIFT);
			break;
	default:
			printk(KERN_ERR "Invalid Clock Freq!\n");
			return -EINVAL;
	}

	__raw_writel(epll_con0, S3C_EPLL_CON);
	__raw_writel(epll_con1, S3C_EPLL_CON_K);

	clk->rate = rate;

	return 0;
}

struct clk clk_fout_epll = {
	.name		= "fout_epll",
	.id		= -1,
	.ctrlbit	= (1<<31),
	.enable		= fout_enable,
	.get_rate	= fout_get_rate,
	.set_rate	= fout_set_rate,
};

int mout_set_parent(struct clk *clk, struct clk *parent)
{
	int src_nr = -1;
	int ptr;
	u32 clksrc;
	struct clksrc_clk *sclk = to_clksrc(clk);
	struct clk_sources *srcs = sclk->sources;

	clksrc = __raw_readl(S3C_CLK_SRC0);

	for (ptr = 0; ptr < srcs->nr_sources; ptr++)
		if (srcs->sources[ptr] == parent) {
			src_nr = ptr;
			break;
		}

	if (src_nr >= 0) {
		clksrc &= ~sclk->mask;
		clksrc |= src_nr << sclk->shift;
		__raw_writel(clksrc, S3C_CLK_SRC0);
		clk->parent = parent;
		return 0;
	}

	return -EINVAL;
}

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
		.set_parent	= mout_set_parent,
	},
	.shift		= S3C_CLKSRC_EPLL_MOUT_SHIFT,
	.mask		= S3C_CLKSRC_EPLL_MOUT,
	.sources	= &clk_src_epll,
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
	.shift		= S3C_CLKSRC_MPLL_MOUT_SHIFT,
	.mask		= S3C_CLKSRC_MPLL_MOUT,
	.sources	= &clk_src_mpll,
};

static unsigned long s5p64xx_clk_doutapll_get_rate(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	//printk(KERN_DEBUG "%s: parent is %ld\n", __func__, rate);

	if (__raw_readl(S3C_CLK_DIV0) & S3C_CLKDIV0_ARM_MASK)
		rate /= 2;

	return rate;
}

static unsigned long s5p64xx_clk_doutmpll_get_rate(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	//printk(KERN_DEBUG "%s: parent is %ld\n", __func__, rate);

	if (__raw_readl(S3C_CLK_DIV0) & S3C_CLKDIV0_MPLL_MASK)
		rate /= 2;

	return rate;
}

struct clk clk_dout_apll = {
	.name		= "dout_apll",
	.id		= -1,
	.parent		= &clk_mout_apll.clk,
	.get_rate	= s5p64xx_clk_doutapll_get_rate,
};

struct clk clk_dout_mpll = {
	.name		= "dout_mpll",
	.id		= -1,
	.parent		= &clk_mout_mpll.clk,
	.get_rate	= s5p64xx_clk_doutmpll_get_rate,
};

static struct clk clk_iis_cd_v40 = {
	.name		= "iis_cdclk_v40",
	.id		= -1,
};

static struct clk clk_pcm_cd = {
	.name		= "pcm_cdclk",
	.id		= -1,
};

static struct clk *clkset_fimgvg_list[] = {
	[0] = &clk_dout_apll,
	[1] = &clk_dout_apll, /* NILA: This entry does not exists, just to set nr_sources to correct value (mkaehlc: couldn't it be NULL?) */ 
	[2] = &clk_mout_mpll.clk,
};

static struct clk_sources clkset_fimgvg = {
	.sources	= clkset_fimgvg_list,
	.nr_sources	= ARRAY_SIZE(clkset_fimgvg_list),
};

static struct clk *clkset_audio2_list[] = {
	[0] = &clk_mout_epll.clk,
	[1] = &clk_dout_mpll,
	[2] = &clk_fin_epll,
	[3] = &clk_iis_cd_v40,
	[4] = &clk_pcm_cd,
};

static struct clk_sources clkset_audio2 = {
	.sources	= clkset_audio2_list,
	.nr_sources	= ARRAY_SIZE(clkset_audio2_list),
};

static struct clk *clkset_spi_mmc_list[] = {
	&clk_mout_epll.clk,
	&clk_dout_mpll,
	&clk_fin_epll,
};

static struct clk_sources clkset_spi_mmc = {
	.sources	= clkset_spi_mmc_list,
	.nr_sources	= ARRAY_SIZE(clkset_spi_mmc_list),
};

static struct clk *clkset_uart_list[] = {
	&clk_mout_epll.clk,
	&clk_dout_mpll,
	NULL,
	NULL
};

static struct clk_sources clkset_uart = {
	.sources	= clkset_uart_list,
	.nr_sources	= ARRAY_SIZE(clkset_uart_list),
};

/* The peripheral clocks are all controlled via clocksource followed
 * by an optional divider and gate stage. We currently roll this into
 * one clock which hides the intermediate clock from the mux.
 *
 * Note, the JPEG clock can only be an even divider...
 *
 * The scaler and LCD clocks depend on the S5P64XX version, and also
 * have a common parent divisor so are not included here.
 */

static unsigned long s5p64xx_getrate_clksrc(struct clk *clk)
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

static int s5p64xx_setrate_clksrc(struct clk *clk, unsigned long rate)
{
	struct clksrc_clk *sclk = to_clksrc(clk);
	void __iomem *reg = sclk->reg_divider;
	unsigned int div;
	u32 val;

	rate = clk_round_rate(clk, rate);
	div = clk_get_rate(clk->parent) / rate;

	val = __raw_readl(reg);
	val &= ~(0xf << sclk->divider_shift);
	val |= ((div - 1) << sclk->divider_shift);
	__raw_writel(val, reg);

	return 0;
}

static struct clksrc_clk clk_audio2;
static struct clksrc_clk clk_fimgvg;

static int s5p64xx_setparent_clksrc(struct clk *clk, struct clk *parent)
{
	int ptr;
	int src_nr = -1;
	u32 clksrc;
	void __iomem *reg_clk_src;
	struct clksrc_clk *sclk = to_clksrc(clk);
	struct clk_sources *srcs = sclk->sources;

	// determine the register used to set the parent
	if ((sclk == &clk_audio2) ||
	    (sclk == &clk_fimgvg))
		reg_clk_src = S3C_CLK_SRC1;
	else
		reg_clk_src = S3C_CLK_SRC0;

	// read the current configuration
	clksrc = __raw_readl(reg_clk_src);

	// determine the index of the parent in the list of possible clock sources
	for (ptr = 0; ptr < srcs->nr_sources; ptr++) {
		if (srcs->sources[ptr] == parent) {
			src_nr = ptr;
			break;
		}
	}

	if (src_nr >= 0) {
		// the parent is in the list of possible clock sources

		clksrc &= ~sclk->mask;
		clksrc |= src_nr << sclk->shift;

		__raw_writel(clksrc, reg_clk_src);

		clk->parent = parent;

		return 0;
	}

	return -EINVAL;
}

static unsigned long s5p64xx_roundrate_clksrc(struct clk *clk,
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
		.ctrlbit        = S3C_CLKCON_SCLK0_MMC0,
		.enable		= s5p64xx_sclk0_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_MMC0_SHIFT,
	.mask		= S3C_CLKSRC_MMC0_MASK,
	.sources	= &clkset_spi_mmc,
	.divider_shift	= S3C_CLKDIV1_MMC0_SHIFT,
	.reg_divider	= S3C_CLK_DIV1,
};

static struct clksrc_clk clk_mmc1 = {
	.clk	= {
		.name		= "mmc_bus",
		.id		= 1,
		.ctrlbit        = S3C_CLKCON_SCLK0_MMC1,
		.enable		= s5p64xx_sclk0_ctrl,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.set_parent	= s5p64xx_setparent_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_MMC1_SHIFT,
	.mask		= S3C_CLKSRC_MMC1_MASK,
	.sources	= &clkset_spi_mmc,
	.divider_shift	= S3C_CLKDIV1_MMC1_SHIFT,
	.reg_divider	= S3C_CLK_DIV1,
};

static struct clksrc_clk clk_mmc2 = {
	.clk	= {
		.name		= "mmc_bus",
		.id		= 2,
		.ctrlbit        = S3C_CLKCON_SCLK0_MMC2,
		.enable		= s5p64xx_sclk0_ctrl,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.set_parent	= s5p64xx_setparent_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_MMC2_SHIFT,
	.mask		= S3C_CLKSRC_MMC2_MASK,
	.sources	= &clkset_spi_mmc,
	.divider_shift	= S3C_CLKDIV1_MMC2_SHIFT,
	.reg_divider	= S3C_CLK_DIV1,
};

static struct clksrc_clk clk_uart_uclk1 = {
	.clk	= {
		.name		= "uclk1",
		.id		= -1,
		.ctrlbit        = S3C_CLKCON_SCLK0_UART,
		.enable		= s5p64xx_sclk0_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_UART_SHIFT,
	.mask		= S3C_CLKSRC_UART_MASK,
	.sources	= &clkset_uart,
	.divider_shift	= S3C_CLKDIV2_UART_SHIFT,
	.reg_divider	= S3C_CLK_DIV2,
};

static struct clksrc_clk clk_fimgvg = {
	.clk	= {
		.name		= "fimgvg",
		.id		= -1,
		.ctrlbit        = S3C_CLKCON_SCLK1_FIMGVG,
		.enable		= s5p64xx_sclk1_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC1_FIMGVG_SHIFT,
	.mask		= S3C_CLKSRC1_FIMGVG_MASK,
	.sources	= &clkset_fimgvg,
	.divider_shift	= S3C_CLKDIV3_FIMGVG_SHIFT,
	.reg_divider	= S3C_CLK_DIV3,
};

static struct clksrc_clk clk_audio2 = {
	.clk	= {
		.name		= "audio-bus",
		.id		= 0,
		.ctrlbit        = S3C_CLKCON_SCLK0_AUDIO2,
		.enable		= s5p64xx_sclk0_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC1_AUDIO2_SHIFT,
	.mask		= S3C_CLKSRC1_AUDIO2_MASK,
	.sources	= &clkset_audio2,
	.divider_shift	= S3C_CLKDIV2_AUDIO2_SHIFT,
	.reg_divider	= S3C_CLK_DIV2,
};

/* Where does UCLK0 come from? */

static struct clksrc_clk clk_spi0 = {
	.clk	= {
		.name		= "spi_epll",
		.id		= 0,
		.ctrlbit        = S3C_CLKCON_SCLK0_SPI0,
		.enable		= s5p64xx_sclk0_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_SPI0_SHIFT,
	.mask		= S3C_CLKSRC_SPI0_MASK,
	.sources	= &clkset_spi_mmc,
	.divider_shift	= S3C_CLKDIV2_SPI0_SHIFT,
	.reg_divider	= S3C_CLK_DIV2,
};

static struct clksrc_clk clk_spi1 = {
	.clk	= {
		.name		= "spi_epll",
		.id		= 1,
		.ctrlbit        = S3C_CLKCON_SCLK0_SPI1,
		.enable		= s5p64xx_sclk0_ctrl,
		.set_parent	= s5p64xx_setparent_clksrc,
		.get_rate	= s5p64xx_getrate_clksrc,
		.set_rate	= s5p64xx_setrate_clksrc,
		.round_rate	= s5p64xx_roundrate_clksrc,
	},
	.shift		= S3C_CLKSRC_SPI1_SHIFT,
	.mask		= S3C_CLKSRC_SPI1_MASK,
	.sources	= &clkset_spi_mmc,
	.divider_shift	= S3C_CLKDIV2_SPI1_SHIFT,
	.reg_divider	= S3C_CLK_DIV2,
};

/* Clock initialisation code */

static struct clksrc_clk *init_parents[] = {
	&clk_mout_apll,
	&clk_mout_epll,
	&clk_mout_mpll,
	&clk_mmc0,
	&clk_mmc1,
	&clk_mmc2,
	&clk_uart_uclk1,
	&clk_spi0,
	&clk_spi1,
	&clk_audio2,
	&clk_fimgvg,
};

static void __init_or_cpufreq s5p6440_set_clksrc(struct clksrc_clk *clk)
{
	u32 clksrc;
	struct clk_sources *srcs = clk->sources;

	// determine the register used to set the parent
	if ((clk == &clk_audio2) ||
	    (clk == &clk_fimgvg))
		clksrc = __raw_readl(S3C_CLK_SRC1);
	else
		clksrc = __raw_readl(S3C_CLK_SRC0);

	clksrc &= clk->mask;
	clksrc >>= clk->shift;

	if (clksrc > srcs->nr_sources || !srcs->sources[clksrc]) {
		printk(KERN_ERR "%s: bad source %d\n",
		       clk->clk.name, clksrc);
		return;
	}

	clk->clk.parent = srcs->sources[clksrc];

	printk(KERN_INFO "%s: source is %s (%d), rate is %ld.%ldMHz\n",
	       clk->clk.name, clk->clk.parent->name, clksrc,
	       print_mhz(clk_get_rate(&clk->clk)));
}

#define GET_DIV(clk, field) ((((clk) & field##_MASK) >> field##_SHIFT) + 1)

void __init_or_cpufreq s5p6440_setup_clocks(void)
{
	struct clk *xtal_clk;
	unsigned long xtal;
	unsigned long fclk;
	unsigned long hclk;
	unsigned long hclk_low;
	unsigned long pclk;
	unsigned long pclk_low;
	unsigned long epll;
	unsigned long apll;
	unsigned long mpll;
	unsigned int ptr;
	u32 clkdiv0;
	u32 clkdiv3;
	u32 clkdiv1;

	printk(KERN_DEBUG "%s: registering clocks\n", __func__);

	clkdiv0 = __raw_readl(S3C_CLK_DIV0);
	printk(KERN_DEBUG "%s: clkdiv0 = %08x\n", __func__, clkdiv0);

	clkdiv3 = __raw_readl(S3C_CLK_DIV3);
	printk(KERN_DEBUG "%s: clkdiv3 = %08x\n", __func__, clkdiv3);

	/* Set default fimgvg clock route */
	clk_set_parent(&clk_mout_mpll.clk, &clk_fout_mpll);
	clk_set_parent(&clk_fimgvg.clk, &clk_mout_mpll.clk);

	/* Set default audio-bus clock route */
	clk_set_parent(&clk_mout_epll.clk, &clk_fout_epll);
	clk_set_parent(&clk_audio2.clk, &clk_mout_epll.clk);

	/* init mmc_clock source */
	s5p64xx_setparent_clksrc(&clk_mmc0.clk, &clk_dout_mpll);
	s5p64xx_setparent_clksrc(&clk_mmc1.clk, &clk_dout_mpll);
	s5p64xx_setparent_clksrc(&clk_mmc2.clk, &clk_dout_mpll);

	/* init mmc_clock divider */
	clkdiv1 = __raw_readl(S3C_CLK_DIV1);
	clkdiv1 &= ~0x00000fff;
	writel(clkdiv1 | 0x777, S3C_CLK_DIV1);
	clkdiv1 = __raw_readl(S3C_CLK_DIV1);

	xtal_clk = clk_get(NULL, "xtal");
	BUG_ON(IS_ERR(xtal_clk));

	xtal = clk_get_rate(xtal_clk);
	clk_put(xtal_clk);

	printk(KERN_DEBUG "%s: xtal is %ld\n", __func__, xtal);

	epll = s5p6440_get_epll(xtal);
	mpll = s5p6440_get_pll(xtal, __raw_readl(S3C_MPLL_CON));
	apll = s5p6440_get_pll(xtal, __raw_readl(S3C_APLL_CON));

	printk(KERN_INFO "S5P64XX: PLL settings, A=%ld.%ldMHz, M=%ld.%ldMHz," \
							" E=%ld.%ldMHz\n",
	       print_mhz(apll), print_mhz(mpll), print_mhz(epll));

	fclk = apll / GET_DIV(clkdiv0, S3C_CLKDIV0_ARM);
	hclk = fclk / GET_DIV(clkdiv0, S3C_CLKDIV0_HCLK);
	pclk = hclk / GET_DIV(clkdiv0, S3C_CLKDIV0_PCLK);

	if(__raw_readl(S3C_OTHERS) & S3C_OTHERS_HCLK_LOW_SEL_MPLL) {
		/* Asynchronous mode */
		hclk_low = mpll / GET_DIV(clkdiv3, S3C_CLKDIV3_HCLK_LOW);
	} else {
		/* Synchronous mode */
		hclk_low = apll / GET_DIV(clkdiv3, S3C_CLKDIV3_HCLK_LOW);
	}

	pclk_low = hclk_low / GET_DIV(clkdiv3, S3C_CLKDIV3_PCLK_LOW);

	printk(KERN_INFO "S5P64XX: HCLK=%ld.%ldMHz, HCLK_LOW=%ld.%ldMHz," \
				" PCLK=%ld.%ldMHz, PCLK_LOW=%ld.%ldMHz\n",
			       print_mhz(hclk), print_mhz(hclk_low),
			       print_mhz(pclk), print_mhz(pclk_low));

	clk_fout_mpll.rate = mpll;
	clk_fout_epll.rate = epll;
	clk_fout_apll.rate = apll;

	clk_f.rate = fclk;
	clk_h.rate = hclk;
	clk_p.rate = pclk;
	clk_h_low.rate = hclk_low;
	clk_p_low.rate = pclk_low;

	for (ptr = 0; ptr < ARRAY_SIZE(init_parents); ptr++)
		s5p6440_set_clksrc(init_parents[ptr]);
}

static struct clk *clks[] __initdata = {
	&clk_ext_xtal_mux,
	&clk_mout_epll.clk,
	&clk_fout_epll,
	&clk_mout_mpll.clk,
	&clk_dout_mpll,
	&clk_iis_cd_v40,
	&clk_pcm_cd,
	&clk_mmc0.clk,
	&clk_mmc1.clk,
	&clk_mmc2.clk,
	&clk_uart_uclk1.clk,
	&clk_spi0.clk,
	&clk_spi1.clk,
	&clk_audio2.clk,
	&clk_fimgvg.clk,
};

void __init s5p6440_register_clocks(void)
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

//	clk_mpll.parent = &clk_mout_mpll.clk;
	clk_epll.parent = &clk_mout_epll.clk;
}
