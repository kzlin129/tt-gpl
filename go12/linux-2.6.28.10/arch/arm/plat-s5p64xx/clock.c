/* linux/arch/arm/plat-s5p64xx/clock.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S5P64XX Base clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/regs-clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/clock.h>
#include <plat/cpu-freq.h>


struct clk clk_27m = {
	.name		= "clk_27m",
	.id		= -1,
	.rate		= 27000000,
};

static int clk_48m_ctrl(struct clk *clk, int enable)
{
	unsigned long flags;
	u32 val;

	/* can't rely on clock lock, this register has other usages */
	local_irq_save(flags);

	val = __raw_readl(S3C_OTHERS);
	if (enable)
		val |= S3C_OTHERS_USB_SIG_MASK;
	else
		val &= ~S3C_OTHERS_USB_SIG_MASK;

	__raw_writel(val, S3C_OTHERS);
	local_irq_restore(flags);

	return 0;
}

struct clk clk_48m = {
	.name		= "clk_48m",
	.id		= -1,
	.rate		= 48000000,
	.enable		= clk_48m_ctrl,
};

unsigned long s3c_fclk_get_rate(void)
{
	unsigned long apll_con;
	unsigned long clk_div0_tmp;
	unsigned long m = 0;
	unsigned long p = 0;
	unsigned long s = 0;
	unsigned long ret;

	apll_con = __raw_readl(S3C_APLL_CON);
	clk_div0_tmp = __raw_readl(S3C_CLK_DIV0) & 0xf;

	m = (apll_con >> 16) & 0x3ff;
	p = (apll_con >> 8) & 0x3f;
	s = apll_con & 0x3;

	ret = (m * (INIT_XTAL / (p * (1 << s))));

	return (ret / (clk_div0_tmp + 1));
}

unsigned long s3c_fclk_round_rate(struct clk *clk, unsigned long rate)
{
	u32 iter;

	for(iter = 1 ; iter < ARRAY_SIZE(s3c_cpu_clock_table) ; iter++){
		if(rate > s3c_cpu_clock_table[iter][0])
			return s3c_cpu_clock_table[iter-1][0];
	}

	return s3c_cpu_clock_table[ARRAY_SIZE(s3c_cpu_clock_table) - 1][0];
}

int s3c_fclk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 round_tmp;
	u32 iter;
	u32 clk_div0_tmp;
	u32 cur_clk = s3c_fclk_get_rate();
	unsigned long flags;

	round_tmp = s3c_fclk_round_rate(clk,rate);

	if(round_tmp == cur_clk)
		return 0;


	for (iter = 0 ; iter < ARRAY_SIZE(s3c_cpu_clock_table) ; iter++){
		if(round_tmp == s3c_cpu_clock_table[iter][0])
			break;
	}

	if(iter >= ARRAY_SIZE(s3c_cpu_clock_table))
		iter = ARRAY_SIZE(s3c_cpu_clock_table) - 1;

	local_irq_save(flags);
	if(cur_clk > round_tmp) {
		/* Frequency Down */
		clk_div0_tmp = __raw_readl(ARM_CLK_DIV) & ~(ARM_DIV_MASK);
		clk_div0_tmp |= s3c_cpu_clock_table[iter][1];
		__raw_writel(clk_div0_tmp, ARM_CLK_DIV);

		clk_div0_tmp = __raw_readl(ARM_CLK_DIV) & ~(HCLK_DIV_MASK);
		clk_div0_tmp |= s3c_cpu_clock_table[iter][2];
		__raw_writel(clk_div0_tmp, ARM_CLK_DIV);


	} else {
		/* Frequency Up */
		clk_div0_tmp = __raw_readl(ARM_CLK_DIV) & ~(HCLK_DIV_MASK);
		clk_div0_tmp |= s3c_cpu_clock_table[iter][2];
		__raw_writel(clk_div0_tmp, ARM_CLK_DIV);

		clk_div0_tmp = __raw_readl(ARM_CLK_DIV) & ~(ARM_DIV_MASK);
		clk_div0_tmp |= s3c_cpu_clock_table[iter][1];
		__raw_writel(clk_div0_tmp, ARM_CLK_DIV);
			
	}
	local_irq_restore(flags);

	clk->rate = s3c_cpu_clock_table[iter][0];

	return 0;
}

struct clk clk_cpu = {
	.name		= "clk_cpu",
	.id		= -1,
	.rate		= 0,
	.parent		= &clk_mpll,
	.ctrlbit	= 0,
	.set_rate	= s3c_fclk_set_rate,
	.round_rate	= s3c_fclk_round_rate,
};

static int inline s5p64xx_gate(void __iomem *reg,
				struct clk *clk,
				int enable)
{
	unsigned int ctrlbit = clk->ctrlbit;
	u32 con;

	con = __raw_readl(reg);

	if (enable)
		con |= ctrlbit;
	else
		con &= ~ctrlbit;

	__raw_writel(con, reg);
	return 0;
}

static int s5p64xx_pclk_ctrl(struct clk *clk, int enable)
{
	return s5p64xx_gate(S3C_CLK_GATE_PCLK, clk, enable);
}

static int s5p64xx_hclk0_ctrl(struct clk *clk, int enable)
{
	return s5p64xx_gate(S3C_CLK_GATE_HCLK0, clk, enable);
}

static int s5p64xx_hclk1_ctrl(struct clk *clk, int enable)
{
	return s5p64xx_gate(S3C_CLK_GATE_HCLK1, clk, enable);
}

int s5p64xx_sclk0_ctrl(struct clk *clk, int enable)
{
	return s5p64xx_gate(S3C_CLK_GATE_SCLK0, clk, enable);
}

int s5p64xx_sclk1_ctrl(struct clk *clk, int enable)
{
	return s5p64xx_gate(S3C_CLK_GATE_SCLK1, clk, enable);
}

static struct clk init_clocks_disable[] = {
	{
		.name		= "nand",
		.id		= -1,
		.parent		= &clk_h,
	}, {
		.name		= "adc",
		.id		= -1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_TSADC,
	}, {
		.name		= "i2c",
		.id		= -1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_IIC0,
	}, {
		.name		= "iis",
		.id		= 0,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_IIS2,
	}, {
		.name		= "pcm",
		.id		= 0,
		.parent		= &clk_p,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_PCM0,
	}, {
		.name		= "spi",
		.id		= 0,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_SPI0,
	}, {
		.name		= "spi",
		.id		= 1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_SPI1,
	}, {
		.name		= "sclk_spi_48",
		.id		= 0,
		.parent		= &clk_48m,
		.enable		= s5p64xx_sclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK0_SPI0_48,
	}, {
		.name		= "sclk_spi_48",
		.id		= 1,
		.parent		= &clk_48m,
		.enable		= s5p64xx_sclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK0_SPI1_48,
	}, {
		.name		= "48m",
		.id		= 0,
		.parent		= &clk_48m,
		.enable		= s5p64xx_sclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK0_MMC0_48,
	}, {
		.name		= "48m",
		.id		= 1,
		.parent		= &clk_48m,
		.enable		= s5p64xx_sclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK0_MMC1_48,
	}, {
		.name		= "48m",
		.id		= 2,
		.parent		= &clk_48m,
		.enable		= s5p64xx_sclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK0_MMC2_48,
	}, {
		.name    	= "otg",
		.id	   	= -1,
		.parent  	= &clk_h_low,
		.enable  	= s5p64xx_hclk0_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK0_USB
	}, {
		.name    	= "post",
		.id	   	= -1,
		.parent  	= &clk_h_low,
		.enable  	= s5p64xx_hclk0_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK0_POST0
	},

};

static struct clk init_clocks[] = {
	{
		.name		= "lcd",
		.id		= -1,
		.parent		= &clk_h_low,
		.enable		= s5p64xx_hclk1_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK1_DISPCON,
	}, {
		.name		= "gpio",
		.id		= -1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_GPIO,
	}, {
		.name		= "hsmmc",
		.id		= 0,
		.parent		= &clk_h_low,
		.enable		= s5p64xx_hclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK0_HSMMC0,
	}, {
		.name		= "hsmmc",
		.id		= 1,
		.parent		= &clk_h_low,
		.enable		= s5p64xx_hclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK0_HSMMC1,
	}, {
		.name		= "hsmmc",
		.id		= 2,
		.parent		= &clk_h_low,
		.enable		= s5p64xx_hclk0_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK0_HSMMC2,
	}, {
		.name		= "timers",
		.id		= -1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_PWM,
	}, {
		.name		= "uart",
		.id		= 0,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART0,
	}, {
		.name		= "uart",
		.id		= 1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART1,
	}, {
		.name		= "uart",
		.id		= 2,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART2,
	}, {
		.name		= "uart",
		.id		= 3,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART3,
	}, {
		.name		= "rtc",
		.id		= -1,
		.parent		= &clk_p_low,
		.enable		= s5p64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_RTC,
	}, {
		.name		= "watchdog",
		.id		= -1,
		.parent		= &clk_p_low,
		.ctrlbit	= S3C_CLKCON_PCLK_WDT,
	}
};

static struct clk *clks[] __initdata = {
	&clk_ext,
	&clk_epll,
	&clk_27m,
	&clk_48m,
	&clk_cpu,
};

void __init s5p64xx_register_clocks(void)
{
	struct clk *clkp;
	int ret;
	int ptr;

	s3c24xx_register_clocks(clks, ARRAY_SIZE(clks));

	clkp = init_clocks;
	for (ptr = 0; ptr < ARRAY_SIZE(init_clocks); ptr++, clkp++) {
		ret = s3c24xx_register_clock(clkp);
		if (ret < 0) {
			printk(KERN_ERR "Failed to register clock %s (%d)\n",
			       clkp->name, ret);
		}
	}

	clkp = init_clocks_disable;
	for (ptr = 0; ptr < ARRAY_SIZE(init_clocks_disable); ptr++, clkp++) {

		ret = s3c24xx_register_clock(clkp);
		if (ret < 0) {
			printk(KERN_ERR "Failed to register clock %s (%d)\n",
			       clkp->name, ret);
		}

		(clkp->enable)(clkp, 0);
	}
}
