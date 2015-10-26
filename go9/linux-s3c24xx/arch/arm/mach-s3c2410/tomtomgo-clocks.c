/* arch/arm/mach-s3c2410/tomtomgo-clocks.c
 *
 * Additional clock type definitions
 *
 * Copyright (C) 2004,2005, 2006 TomTom BV <http://www.tomtom.com/>
 * Authors:
 * Mark Vels <Mark.Vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>

#include <asm/arch/regs-gpio.h>

#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include "tomtomgo-iopins.h"
#include "clock.h"

#define CLK_U   ((struct clk *)1)
#define CLK_P   ((struct clk *)2)

static int s3c24xx_dclk_enable(struct clk *clk, int enable);
static int s3c24xx_clkout_enable(struct clk *clk, int enable);
static int tomtomgo_clk_enable(struct clk *clk, int enable);

/**
 * "sub class" from struct clk to contain the extra divider info.
 */
typedef struct dclk_clk {
	struct list_head      list;
	struct module        *owner;
	struct clk           *parent;
	const char           *name;
	int                   id;
	atomic_t              used;
	unsigned long         rate;
	unsigned long         ctrlbit;
	int                 (*enable)(struct clk *, int enable);
	int                 (*set_rate) (struct clk *clk, unsigned long rate);

	/* additional fields: */
	int                   dclkdiv;  /* frequency = source clock / (DCLK1DIV + 1) */
	int                   dclkcmp;  /* If the DCLKCMP is n, Low level duration is (n + 1), */
	                                /* High level duration is ((DCLK1DIV + 1) (n + 1)). */
} dclk_clk;


static struct dclk_clk s3c24xx_tomtom_dclks[] = {

	[0] = {
		.name    = "dclk0",
		.parent  = CLK_P,			/* Default source */
		.id      = 0,				/* DCLOCK 0 */
		.rate    = -1,				/* calculated from divider */
		.dclkdiv = 3,
		.dclkcmp = 1,
		.enable  = s3c24xx_dclk_enable,
	},
	[1] ={
		.name    = "dclk1",
		.parent  = CLK_P,			/* Default source */
		.id      = 1,				/* DCLOCK 1 */
		.rate    = -1,				/* calculated from divider */
		.dclkdiv = 3,
		.dclkcmp = 1,
		.enable  = s3c24xx_dclk_enable,
	},
};

static struct clk s3c24xx_tomtom_clkout[] = {
	[0] = {
		.name   = "clkout0",
		.parent = (struct clk*) &s3c24xx_tomtom_dclks[0],	/* attach to DCLK0 */
		.id     = 0,
		.rate   = 0,						/* defined by parent */
		.enable = s3c24xx_clkout_enable,
	},
	[1] = {
		.name   = "clkout1",
		.parent = (struct clk*) &s3c24xx_tomtom_dclks[1],	/* attach to DCLK1 */
		.id     = 1,
		.rate   = 0,						/* defined by parent */
		.enable = s3c24xx_clkout_enable,
	},
};


enum {
	INDEX_XUART_CLOCK = 0,
	INDEX_12MHZ_IIS_CLOCK,
};

static struct clk tomtomgo_clks[] = {
	[INDEX_XUART_CLOCK] = {
		.name   = "xuart",
		.parent = &s3c24xx_tomtom_clkout[0],	/* default attach to CLKOUT0 */
		.id     = -1,				/* only one xuart clock */
		.rate   = 0,				/* defined by parent */
		.enable = tomtomgo_clk_enable,
	},
	[INDEX_12MHZ_IIS_CLOCK] = {
		.name   = "12mhz_iis",
		.parent = &s3c24xx_tomtom_clkout[0],	/* default attach to CLKOUT0 */
		.id     = -1,				/* only one 12mhz_iis clock */
		.rate   = 0,				/* defined by parent */
		.enable = tomtomgo_clk_enable,
	},
};

int tomtomgo_init_clocks(void)
{
	struct clk *clk_u;
	struct clk *clk_p;
	struct clk *clk_xtal;
	int i;
	struct dclk_clk *dclkp;
	int needs_iis_clock;
	int needs_uart_clock;
#if defined(CONFIG_BARCELONA_SOUND) || defined(CONFIG_BARCELONA_SOUND_MODULE)
	int iis_clkout;
#endif /* CONFIG_BARCELONA_SOUND */
#if defined(CONFIG_BARCELONA_EXTERNAL_UART) || defined(CONFIG_BARCELONA_EXTERNAL_UART_MODULE)
	int uart_clkout;
#endif /* BARCELONA_EXTERNAL_UART */

	clk_u = clk_get(NULL, "upll");
	clk_p = clk_get(NULL, "pclk");
	clk_xtal = clk_get( NULL, "xtal");
	
	if (IS_ERR(clk_p) ) {
		printk(KERN_ERR "tomtomgo_init_clocks: Failed to get parent: pclk !\n");
		return -EINVAL;
	}
	if (IS_ERR(clk_u)  && 
           (IO_GetCpuType() != GOCPU_S3C2443 ) && 
           (IO_GetCpuType() != GOCPU_S3C2450 ) && 
           (IO_GetCpuType() != GOCPU_S3C2410) ) 
        {
		printk(KERN_ERR "tomtomgo_init_clocks: Failed to get parent: uclk !\n");
		return -EINVAL;
	}
	
	if (IS_ERR(clk_xtal)  && 
            ((IO_GetCpuType() == GOCPU_S3C2443) || 
            (IO_GetCpuType() == GOCPU_S3C2450))) {
		printk(KERN_ERR "tomtomgo_init_clocks: Failed to get parent: xtal !\n");
		return -EINVAL;
	}

	/* Set up parents of dclks */
	for (i = 0; i < ARRAY_SIZE(s3c24xx_tomtom_dclks); ++i) {
		dclkp = &s3c24xx_tomtom_dclks[i];
		switch ((int)dclkp->parent) {
		case (int)CLK_U: dclkp->parent = clk_u; break;
		case (int)CLK_P: dclkp->parent = clk_p; break;
		default:         dclkp->parent = NULL;  break;
		}
	}

	/* First find out which clocks need to be and which one */
#if defined(CONFIG_BARCELONA_SOUND) || defined(CONFIG_BARCELONA_SOUND_MODULE)
	iis_clkout = 0;
	/* IO_GetCodecMaster() returns possibly 3 values:
	 * - GOCODECCFG_SLAVE			: Codec is not master and no iis_12mhz clock is needed;
	 * - GOCODECCFG_EXTERNAL_MASTER	: Codec is EXTERNAL MASTER and we need the 12MHZ clock;
	 * - GOCODECCFG_INTERNAL_MASTER	: Codec is INTERNAL MASTER but we don't need the external 12 MHZ clock.
	 */
	if ( IO_GetCodecMaster() == GOCODECCFG_EXTERNAL_MASTER  ) {
		struct dclk_clk* dclk;

		iis_clkout = IO_GetCodecClkOut();

		/* Only on Milan, the iis 12MHZ clock is fed with xtal clock source */
		if( IO_GetModelId() != GOTYPE_MILAN ){
			
			dclk = &s3c24xx_tomtom_dclks[iis_clkout]; /* ClKOUTx means that we have to use DCLKx as well */
			dclk->dclkdiv  = 3;
			dclk->dclkcmp  = 1;
	
			/* Select source for DCLOCKx */
			dclk->parent = clk_u;
	
			/* DCLKc frequency = source clock / (DCLKxDIV + 1) */
			dclk->rate = clk_get_rate(clk_u) / (dclk->dclkdiv + 1);
	
			
		}
		else{	/* Milan exception	*/
			if( iis_clkout != 0 ){
				printk("CONFIG ERROR! Clkout1 cannot be attached to xtal clock source on S3c2443!\n");	
			}
			s3c24xx_tomtom_clkout[iis_clkout].parent = clk_xtal;
		}
		
		/* Set which CLKOUT to use for clock "12mhz_iis" */
		tomtomgo_clks[INDEX_12MHZ_IIS_CLOCK].parent = &s3c24xx_tomtom_clkout[iis_clkout];
		tomtomgo_clks[INDEX_12MHZ_IIS_CLOCK].rate = s3c24xx_tomtom_clkout[iis_clkout].rate;
		printk("TomTom GO Clocks: attaching clock \"%s\" to clock %s\n",
		       tomtomgo_clks[INDEX_12MHZ_IIS_CLOCK].name, s3c24xx_tomtom_clkout[iis_clkout].name);
			       
		needs_iis_clock = 1;
	} else
#endif /* CONFIG_BARCELONA_SOUND */
	{
		needs_iis_clock = 0;
	}

#if defined(CONFIG_BARCELONA_EXTERNAL_UART) || defined(CONFIG_BARCELONA_EXTERNAL_UART_MODULE)
	uart_clkout = 0;
	if (IO_HasPin(UART_CLK)) {
		struct dclk_clk* dclk;

		uart_clkout = IO_GetExternalUartClkOut();

		/* Find out if this CLKOUT is shared with the IIS driver. */
		/* IF so, IIS settings are master, UART is slave and we   */
		/* will not change settings	for xuart clock				 */
		 
#if defined(CONFIG_BARCELONA_SOUND) || defined(CONFIG_BARCELONA_SOUND_MODULE)
		if (!(needs_iis_clock && (uart_clkout == iis_clkout))) 
#endif/* CONFIG_BARCELONA_SOUND */
		{
			/**
			 * Optimize for baudrate 115200. Accoring to calculations the optimum (i.e. the smallest baudrate error)
			 * lies in a DCLK diver of 5 and a uart divider of 5
			 */
			dclk = &s3c24xx_tomtom_dclks[uart_clkout]; /* ClKOUTx means that we have to use DCLKx as well */
			dclk->dclkdiv  = 4;	/* WARNING: maximum external clk freq. for XR16L570 @ 1.8V is rated at 12 MHZ */
			dclk->dclkcmp  = 2; /* 50%  duty cycle */

			/* Select source for DCLOCKx */
			dclk->parent = clk_p;

			/* DCLKc frequency = source clock / (DCLKxDIV + 1) */
			dclk->rate = clk_get_rate(clk_p) / (dclk->dclkdiv + 1);
		}

		/* Set which CLKOUT to use for clock "xuart" */
		tomtomgo_clks[INDEX_XUART_CLOCK].parent = &s3c24xx_tomtom_clkout[uart_clkout];
		tomtomgo_clks[INDEX_XUART_CLOCK].rate = s3c24xx_tomtom_clkout[uart_clkout].rate;

		printk("TomTom GO Clocks: attaching clock \"%s\" to clock %s\n",
		       tomtomgo_clks[INDEX_XUART_CLOCK].name, s3c24xx_tomtom_clkout[uart_clkout].name);

		needs_uart_clock = 1;
	}
	else
#endif /* BARCELONA_EXTERNAL_UART */
	{
		needs_uart_clock = 0;
	}

#ifdef CONFIG_VIDEO_S3C241X_CAMIF
	if( IO_HaveCamera( ) )
	{
                struct clk		*cam_clk;
		struct clk		*hclk;
		unsigned long int	divider;
		unsigned long int	rate;

                /* Get the clock to set the right rates. */
                cam_clk=clk_get( NULL, "cam" );
                if( IS_ERR( cam_clk ) )
                {
			printk( KERN_ERR "tomtomgo_init_clocks: Failed to get camera clock\n");
			return -EINVAL;
		}

		/* Program the clock properly. Also make sure the rate matches what it should be. */
		/* Set the correct rate. Note officially this should be 24MHz. To save 1mA power, we divide from HCLK. */
		/* HCLK/5=26.4MHz. If OVCLOCK_REALLY_24MHZ is defined, the CAMCLK is derived from USYSCLK/4=24MHz.  */
#ifdef OVCLOCK_REALLY_24MHZ
		/* CAMCLK=USYSCLK/4 */
		divider=4;
		__raw_writel( (__raw_readl( S3C2412_CLKSRC ) & ~S3C2412_CLKSRC_SELCAM), S3C2412_CLKSRC );

		/* Set the right rate. When using UPLL this depends on how it's programmed, so take this into account. */
		if( __raw_readl( S3C2412_CLKSRC ) & S3C2412_CLKSRC_SELUPLL )
			rate=clk_get_rate( clk_u );
		else
			rate=s3c2410_get_pll( __raw_readl(S3C2410_UPLLCON), get_clk_rate( clk_u ) );
#else
		/* CAMCLK=HCLK/5 */
		divider=5;
		__raw_writel( (__raw_readl( S3C2412_CLKSRC ) | S3C2412_CLKSRC_SELCAM), S3C2412_CLKSRC );

		/* Derived from HCLK. Get the rate for HCLK. */
		hclk=clk_get( NULL, "hclk" );
                if( IS_ERR( hclk ) )
                {
			printk( KERN_ERR "tomtomgo_init_clocks: Failed to get host clock for camera\n");
			clk_put( cam_clk );
			return -EINVAL;
		}

		/* Set the rate. */
		rate=clk_get_rate( hclk );

		/* Release the clock. */
		clk_put( hclk );
#endif
		/* Program the remaining registers, and set the rate. */
		__raw_writel( (__raw_readl( S3C2410_CLKDIVN ) & ~S3C2412_CLKDIVN_CAMCLKDIV_MASK) | 
		              ((divider - 1) << 16), S3C2410_CLKDIVN );
		clk_set_rate( cam_clk, rate / divider );

		/* Release the camera clock. */
		clk_put( cam_clk );

		/* Message... */
		printk("TomTom GO Clocks: setting camera clock to %lu\n", rate/divider );
	}
#endif /* CONFIG_VIDEO_S3C241X_CAMIF */

	/* Now register the basic stuff */
	if (s3c24xx_register_clock((struct clk*) &s3c24xx_tomtom_dclks[0]))
		printk(KERN_ERR "failed to register clock DCLK0\n");
	if (s3c24xx_register_clock((struct clk*) &s3c24xx_tomtom_dclks[1]))
		printk(KERN_ERR "failed to register clock DCLK1\n");
	if (s3c24xx_register_clock((struct clk*) &s3c24xx_tomtom_clkout[0]))
		printk(KERN_ERR "failed to register clock CLKOUT0\n");
	if (s3c24xx_register_clock((struct clk*) &s3c24xx_tomtom_clkout[1]))
		printk(KERN_ERR "failed to register clock CLKOUT1\n");

	if (needs_uart_clock) {
		if (s3c24xx_register_clock(&tomtomgo_clks[INDEX_XUART_CLOCK]))
			printk(KERN_ERR "failed to register clock xuart\n");
	}
	if (needs_iis_clock) {
		if (s3c24xx_register_clock(&tomtomgo_clks[INDEX_12MHZ_IIS_CLOCK]))
			printk(KERN_ERR "failed to register clock 12mhz_iis\n");
	}

	return 0;
}
EXPORT_SYMBOL(tomtomgo_init_clocks);


static int s3c24xx_dclk_enable(struct clk *clk, int enable)
{
	u32 reg;
	u16 creg;
	unsigned long flags;
	unsigned long parent_rate;
	struct dclk_clk *dclk = (struct dclk_clk*) clk;
	struct clk *parent;

	if (IS_ERR(dclk))
		return -EINVAL;
	if (IS_ERR(dclk->parent))
		return -EINVAL;

	/*
	 * We will set the settings as if they are for DCLK0.
	 * In case we are modifying DCLK1, we will shift the settings
	 * 16 bits to the right when setting DCLKCON register in s3c24XX
	 */
	creg = 0;
	parent = clk_get_parent(clk);
	if (enable) {
		/* Find out the desired clock source */
		if (0 == strcmp(parent->name, "upll")) {
			creg |= S3C2410_DCLKCON_DCLK0_UCLK;
		} else {
			if (0 != strcmp(parent->name, "pclk")) {
				printk(KERN_WARNING "TomTom GO Clock driver: Cannot set source for dclk%d to %s\n", dclk->id, parent->name);
				return EINVAL;
			}
			/* else keep creg = 0 to select pclk as source */
		}

		/* specify divisor and 50% duty cycle */
		creg |= (dclk->dclkdiv << 4);
		creg |= (dclk->dclkcmp << 8);

		/* enable */
		creg |= S3C2410_DCLKCON_DCLK0EN;
	}
	/* else disable, use creg = 0 */

	local_irq_save(flags);
	{
		reg = ioread32(S3C2410_DCLKCON);
		if (dclk->id == 0) {
			/* DCLCK 0 settings */
			reg &= ~(0xffff);	/* clear all DCLK0 settings */
			reg |= creg;
		} else {
			/* DCLCK1 settings, shift up 16 bits */
			reg &= ~((0xffff) << 16); 	/* clear all DCLK1 settings */
			reg |= (creg << 16);
		}
		iowrite32(reg, S3C2410_DCLKCON);
	}
	local_irq_restore(flags);

	if (enable)
		clk_use(parent);
	else
		clk_unuse(parent);

	/* now set our own clock rate */
	parent_rate = clk_get_rate(dclk->parent);
	/* DCLKc frequency = source clock / (DCLKxDIV + 1) */
	dclk->rate = parent_rate/(dclk->dclkdiv +1);
	return 0;
}


static int s3c24xx_clkout_enable(struct clk *clk, int enable)
{
	u32 reg;
	u16 creg;
	unsigned long flags;
	struct clk *parent;

	if (IS_ERR(clk))
		return -EINVAL;
	if (IS_ERR(clk->parent))
		return -EINVAL;

	creg = 0;
	parent = clk_get_parent(clk);
	if (enable) {
		/* Find out the desired clock source */

		/**
		 * 000 = MPLL CLK 001 = UPLL CLK 010 = RTC clock/ FCLK
		 * 011 = HCLK 100 = PCLK 101 = DCLK0
		 */
		if (0 == strcmp(parent->name, "xtal")) {
			if(clk->id != 0){
				printk(KERN_WARNING "TomTom GO Clock driver: cannot set \"xtal\" as source for CLKOUT %d\n", clk->id );	
			}
			creg = 0;
		} else if (0 == strcmp(parent->name, "hclk")) {
			creg = 3;
		} else if (0 == strcmp(parent->name, "pclk")) {
			creg = 4;
		} else if ((clk->id == 1) && (0 == strcmp(parent->name, "fclk"))) {
			creg = 2;
		} else if ((clk->id == 0) && (0 == strcmp(parent->name, "rtcclk"))) {
			creg = 2;
		} else {
			char srcname[128];
			snprintf(srcname, sizeof(srcname), "dclk%i", clk->id);
			srcname[sizeof(srcname) -1]= '\0';
			if (0 == strcmp(parent->name, srcname)) {
				creg = 5;
			} else{
				printk(KERN_WARNING "TomTom GO Clock driver : Cannot set source for clkout%d to %s\n", clk->id, parent->name);
				return -EINVAL;
			}
		}

		/* write the resolved clock source to S3C2410_MISCCR */
		/* We also have to set the specific pin to output */

		local_irq_save(flags);
		{
			reg = ioread32(S3C2410_MISCCR);
			if (clk->id == 0) {
				reg &= ~(7 << 4); 	/* clear any previous settings */
				reg |= (creg << 4);
			} else {
				reg &= ~(7 << 8); 	/* clear any previous settings */
				reg |= (creg << 8);
			}
			iowrite32(reg, S3C2410_MISCCR);

			reg = ioread32(S3C2410_GPHCON);
			if( (IO_GetCpuType() == GOCPU_S3C2443 ) ||
			    (IO_GetCpuType() == GOCPU_S3C2450 ))
                        {
				if (clk->id == 0) {
					reg &= ~ (S3C2443_GPH13_OUTP | S3C2443_GPH13_CLKOUT0); 	/* clear any present GPH9 config */
					reg |= S3C2443_GPH13_CLKOUT0; 		/* configure to CLOCKOUT0	%10*/
				} else {
					reg &= ~ (S3C2443_GPH14_OUTP | S3C2443_GPH14_CLKOUT1);
					reg |=  S3C2443_GPH14_CLKOUT1;
				}
			} else {
				if (clk->id == 0) {
					reg &= ~ (S3C2410_GPH9_OUTP | S3C2410_GPH9_CLKOUT0); 	/* clear any present GPH9 config */
					reg |= S3C2410_GPH9_CLKOUT0; 		/* configure to CLOCKOUT0	%10*/
				} else {
					reg &= ~ (S3C2410_GPH10_OUTP | S3C2410_GPH10_CLKOUT1);
					reg |=  S3C2410_GPH10_CLKOUT1;
				}
			}
			iowrite32(reg, S3C2410_GPHCON);
		}
		local_irq_restore(flags);

		/* register use at parent */
		/* Watch out, dirty hack but needed to inspect value of used,
		 * should have used clk_use() but that doesn't return parent->used
		 */
		if (1 == atomic_inc_return(&parent->used)) {
			/* we are the first user, enable it */
			parent->enable(parent, 1);
		}
	} else {
		/* DISABLE */
		local_irq_save(flags);
		reg = ioread32(S3C2410_GPHCON);
		if (clk->id == 0) {
			reg &= ~ (S3C2410_GPH9_OUTP | S3C2410_GPH9_CLKOUT0); 	/* set it as input, lowest power consumption? */
		} else {
			reg &= ~ (S3C2410_GPH10_OUTP | S3C2410_GPH10_CLKOUT1); /* set it as input, lowest power consumption? */
		}
		iowrite32(reg, S3C2410_GPHCON);
		local_irq_restore(flags);

		/* deregister use at parent */
		if (0 == atomic_dec_return(&parent->used)) {	// Watch out, dirty hack but needed to inspect value of used
			/* we are the last user, disable it! */
			parent->enable(clk->parent, 0);
		}
	}
	return 0;
}

static int tomtomgo_clk_enable(struct clk *clk, int enable)
{
	struct clk* parent;
	if (IS_ERR(clk))
		return -EINVAL;

	parent = clk_get_parent(clk);
	if (enable) {
		if (clk->parent) {
			/* register use at parent */

			/* Watch out, dirty hack but needed to inspect value of used,
			 * should have used clk_use() but that doesn't return parent->used
			 */

			if (1 == atomic_inc_return(&parent->used)) {
				/* we are the first user, enable it */
				clk_enable(clk->parent);
			}
		}
	} else {
		/* DISABLE */
		if (clk->parent) {
			/* deregister use at parent */

			/* Watch out, dirty hack but needed to inspect value of used,
			 * should have used clk_unuse() but that doesn't return parent->used
			 */
			if (0 == atomic_dec_return(&parent->used)) {
				/* we are the last user, disable it! */
				clk_disable(clk->parent);
			}
		}
	}
	return 0;
}
