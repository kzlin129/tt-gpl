/*
 * linux/arch/arm/mach-s3c2410/clock.h
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	http://www.simtec.co.uk/products/SWLINUX/
 *	Written by Ben Dooks, <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

struct clk {
	struct list_head      list;
	struct module        *owner;
	struct clk           *parent;
	const char           *name;
	int		      id;
	atomic_t              used;
	unsigned long         rate;
	unsigned long         ctrlbit;
	int		    (*enable)(struct clk *, int enable);
};

/* other clocks which may be registered by board support */

extern struct clk s3c24xx_dclk0;
extern struct clk s3c24xx_dclk1;
extern struct clk s3c24xx_clkout0;
extern struct clk s3c24xx_clkout1;
extern struct clk s3c24xx_uextclk;

/* exports for arch/arm/mach-s3c2410
 *
 * Please DO NOT use these outside of arch/arm/mach-s3c2410
*/

extern int s3c24xx_clkcon_enable(struct clk *clk, int enable);
#ifdef CONFIG_CPU_S3C2443
extern int s3c2443_hclkcon_enable(struct clk *clk, int enable);
extern int s3c2443_pclkcon_enable(struct clk *clk, int enable);
extern int s3c2443_sclkcon_enable(struct clk *clk, int enable);

/* Set EPLL clock based on rate lookup table for constants to poke	*/
extern int s3c2443_epll_set_rate( unsigned long rate );
#endif /* CONFIG_CPU_S3C2443 */

#ifdef CONFIG_CPU_S3C2450
extern int s3c2450_hclkcon_enable(struct clk *clk, int enable);
extern int s3c2450_pclkcon_enable(struct clk *clk, int enable);
extern int s3c2450_sclkcon_enable(struct clk *clk, int enable);

/* Set EPLL clock based on rate lookup table for constants to poke	*/
extern int s3c2450_epll_set_rate( unsigned long rate );
#endif /* CONFIG_CPU_S3C2450 */

extern int s3c24xx_register_clock(struct clk *clk);

extern int s3c24xx_setup_clocks(unsigned long xtal,
				unsigned long fclk,
				unsigned long hclk,
				unsigned long pclk);
