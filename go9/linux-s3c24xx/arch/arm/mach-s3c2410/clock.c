/* linux/arch/arm/mach-s3c2410/clock.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Clock control support
 *
 * Based on, and code from linux/arch/arm/mach-versatile/clock.c
 **
 **  Copyright (C) 2004 ARM Limited.
 **  Written by Deep Blue Solutions Limited.
 *
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
#include <linux/proc_fs.h>
#include <linux/delay.h>

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

/* clock information */

static LIST_HEAD(clocks);
static DECLARE_MUTEX(clocks_sem);

/* old functions */

static void inline s3c24xx_clk_enable(void __iomem *reg, unsigned long clocks, int enable)
{
	unsigned long flags;
	unsigned long clkcon;

	local_irq_save(flags);

	clkcon = __raw_readl(reg);

	if (enable)
		clkcon |= clocks;
	else
		clkcon &= ~clocks;

	__raw_writel(clkcon, reg);

	local_irq_restore(flags);
}

/* enable and disable calls for use with the clk struct */

static int clk_null_enable(struct clk *clk, int enable)
{
	return 0;
}

int s3c24xx_clkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2410_CLKCON, clk->ctrlbit, enable);
	return 0;
}

#ifdef CONFIG_CPU_S3C2443
int s3c2443_hclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2443_HCLKCON, clk->ctrlbit, enable);
	return 0;
}

int s3c2443_pclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2443_PCLKCON, clk->ctrlbit, enable);
	return 0;
}

int s3c2443_sclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2443_SCLKCON, clk->ctrlbit, enable);
	return 0;
}

static inline void s3c2443_setEPLL( unsigned char mdiv, unsigned char pdiv, unsigned char sdiv )
{
	writel(( (mdiv<<16) | (pdiv<<8) | sdiv) , S3C2443_EPLLCON );
}

int s3c2443_epll_set_rate( unsigned long rate )
{	
	switch( rate )
	{		
		case 32769231:
			s3c2443_setEPLL( 134, 11, 2);
			break;

		case 67714286:
			s3c2443_setEPLL( 71, 5, 1);
            break;
		case 73714286:
			s3c2443_setEPLL( 78, 5, 1);
		default:
			printk( KERN_ERR "EPLL rate %lu is not supported in the handling code!\n", rate );
			return -1;
	}
	mdelay( 10 );	/* TODO: use the internal LOCKCON1 for waiting to stabilize	*/
	return 0;
}

#endif /* CONFIG_CPU_S3C2443 */

#ifdef CONFIG_CPU_S3C2450
int s3c2450_hclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2450_HCLKCON, clk->ctrlbit, enable);
	return 0;
}

int s3c2450_pclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2450_PCLKCON, clk->ctrlbit, enable);
	return 0;
}

int s3c2450_sclkcon_enable(struct clk *clk, int enable)
{
	s3c24xx_clk_enable(S3C2450_SCLKCON, clk->ctrlbit, enable);
	return 0;
}

static inline void s3c2450_setEPLL( unsigned char mdiv, unsigned char pdiv, unsigned char sdiv )
{
	writel(( (mdiv<<16) | (pdiv<<8) | sdiv) , S3C2450_EPLLCON );
}

int s3c2450_epll_set_rate( unsigned long rate )
{	
	switch( rate )
	{		
		case 32769231:
			s3c2450_setEPLL( 142, 13, 2);
			break;

		case 67714286:
			s3c2450_setEPLL( 79, 7, 1);
            break;
		case 73714286:
			s3c2450_setEPLL( 86, 7, 1);
		default:
			printk( KERN_ERR "EPLL rate %lu is not supported in the handling code!\n", rate );
			return -1;
	}
	mdelay( 10 );	/* TODO: use the internal LOCKCON1 for waiting to stabilize	*/
	return 0;
}

#endif /* CONFIG_CPU_S3C2450 */

/* Clock API calls */

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p;
	struct clk *clk = ERR_PTR(-ENOENT);
	int idno;

	idno = (dev == NULL) ? -1 : to_platform_device(dev)->id;

	down(&clocks_sem);

	list_for_each_entry(p, &clocks, list) {
		if (p->id == idno &&
		    strcmp(id, p->name) == 0 &&
		    try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}

	/* check for the case where a device was supplied, but the
	 * clock that was being searched for is not device specific */

	if (IS_ERR(clk)) {
		list_for_each_entry(p, &clocks, list) {
			if (p->id == -1 && strcmp(id, p->name) == 0 &&
			    try_module_get(p->owner)) {
				clk = p;
				break;
			}
		}
	}

	up(&clocks_sem);
	return clk;
}

void clk_put(struct clk *clk)
{
	module_put(clk->owner);
}

int clk_enable(struct clk *clk)
{
	if (IS_ERR(clk))
		return -EINVAL;

	return (clk->enable)(clk, 1);
}

void clk_disable(struct clk *clk)
{
	if (!IS_ERR(clk))
		(clk->enable)(clk, 0);
}


int clk_use(struct clk *clk)
{
	atomic_inc(&clk->used);
	return 0;
}


void clk_unuse(struct clk *clk)
{
	atomic_dec(&clk->used);
}

unsigned long clk_get_rate(struct clk *clk)
{
	if (IS_ERR(clk))
		return 0;

	if (clk->rate != 0)
		return clk->rate;

	while (clk->parent != NULL && clk->rate == 0)
		clk = clk->parent;

	return clk->rate;
}

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return rate;
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (IS_ERR(clk))
		return -1;	
	clk->rate = rate;
	return  0;
}

struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}

EXPORT_SYMBOL(clk_get);
EXPORT_SYMBOL(clk_put);
EXPORT_SYMBOL(clk_enable);
EXPORT_SYMBOL(clk_disable);
EXPORT_SYMBOL(clk_use);
EXPORT_SYMBOL(clk_unuse);
EXPORT_SYMBOL(clk_get_rate);
EXPORT_SYMBOL(clk_round_rate);
EXPORT_SYMBOL(clk_set_rate);
EXPORT_SYMBOL(clk_get_parent);

/* base clocks */

static struct clk clk_xtal = {
	.name		= "xtal",
	.id		= -1,
	.rate		= 0,
	.parent		= NULL,
	.ctrlbit	= 0,
};

static struct clk clk_f = {
	.name		= "fclk",
	.id		= -1,
	.rate		= 0,
	.parent		= NULL,
	.ctrlbit	= 0,
};

static struct clk clk_h = {
	.name		= "hclk",
	.id		= -1,
	.rate		= 0,
	.parent		= NULL,
	.ctrlbit	= 0,
};

static struct clk clk_p = {
	.name		= "pclk",
	.id		= -1,
	.rate		= 0,
	.parent		= NULL,
	.ctrlbit	= 0,
};

static struct clk clk_uext = {
	.name		= "uextclk",
	.id		= -1,
	.rate		= 0,
	.parent		= NULL,
	.ctrlbit	= 0,
};

/* initialise the clock system */

int s3c24xx_register_clock(struct clk *clk)
{
	clk->owner = THIS_MODULE;
	atomic_set(&clk->used, 0);

	if (clk->enable == NULL)
		clk->enable = clk_null_enable;

	/* add to the list of available clocks */

	down(&clocks_sem);
	list_add(&clk->list, &clocks);
	up(&clocks_sem);

	return 0;
}

/* initalise all the clocks */

int __init s3c24xx_setup_clocks(unsigned long xtal,
				unsigned long fclk,
				unsigned long hclk,
				unsigned long pclk)
{
	printk(KERN_INFO "S3C24xx Clocks, (c) 2004 Simtec Electronics\n");

	/* initialise the main system clocks */

	clk_xtal.rate = xtal;

	clk_h.rate = hclk;
	clk_p.rate = pclk;
	clk_f.rate = fclk;
	clk_uext.rate = 0;

	/* register our clocks */
	if (s3c24xx_register_clock(&clk_xtal) < 0)
		printk(KERN_ERR "failed to register master xtal\n");

	if (s3c24xx_register_clock(&clk_f) < 0)
		printk(KERN_ERR "failed to register cpu fclk\n");

	if (s3c24xx_register_clock(&clk_h) < 0)
		printk(KERN_ERR "failed to register cpu hclk\n");

	if (s3c24xx_register_clock(&clk_p) < 0)
		printk(KERN_ERR "failed to register cpu pclk\n");

	if (s3c24xx_register_clock(&clk_uext) < 0)
		printk(KERN_ERR "failed to register uart extclk\n");

	return 0;
}

/* /proc/clocks entry support */

#ifdef CONFIG_PROC_FS

static int s3c24xx_clocks_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct clk *c;
	char *p = page;
	int len;

	down(&clocks_sem);
	list_for_each_entry(c, &clocks, list) {
		p += sprintf(p, "%s: %lu\n", c->name, clk_get_rate(c));
	}
	up(&clocks_sem);

	len = (p - page) - off;
	if (len < 0)
		len = 0;
	*eof = len <= count;
	*start = page + off;

	return len;
}

static int __init s3c24xx_init_clocks_proc(void)
{
	return create_proc_read_entry("clocks", S_IRUGO, NULL, s3c24xx_clocks_read_proc, 0) == NULL ? -1 : 0;
}

__initcall(s3c24xx_init_clocks_proc);

#endif /* CONFIG_PROC_FS */
