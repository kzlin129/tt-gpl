/*
 * cpufreq.c: clock and memorybus scaling for s3c24xx family
 *
 * Copyright (C) 2006, TomTom International B.V.
 *
 * Author: Mark-Jan Bastian mark-jan.bastian@tomtom.com
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <asm/hardware/clock.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-mem.h>
#include <barcelona/gopins.h>

/* normal: 266/133/66 */
#define S3C2412_HDIVN	1
#define S3C2412_PDIVN	1
#define S3C2412_MDIV	125
#define S3C2412_PDIV	4
#define S3C2412_SDIV	1
#define S3C2412_HCLKDIV	2
#define S3C2412_PCLKDIV	4
#define S3C2412_ARMDIV  0
/* variant: 266/66/66 */
#define S3C2412_ARMDIV_SLOW 0
#define S3C2412_HDIVN_SLOW  3
#define S3C2412_PDIVN_SLOW	0

#define S3C2412_REFRESH_133 1037
#define S3C2412_REFRESH_66  514
#define S3C2412_REFRESH_12  94

typedef struct {
	u32 freq;				/* PLL frequency */
	u32 pll_mdiv;
	u32 pll_pdiv;
	u32 pll_sdiv;
	u32 div_arm;			/* divide factor pll to arm clock */
	u32 div_hclk;			/* divide factor pll to memorybus clock */
	u32 div_pclk;			/* divide factor pll to peripheral clock */
	u32 reg_armdiv;			/* register value */		
	u32 reg_hdivn;			/* register value */
	u32 reg_pdivn;			/* register value */
} s3c24xx_settings_t;

static struct cpufreq_driver s3c24xx_driver;

/* constraints for armdiv on 2443 - do not pick 0x0 */
const unsigned int armdiv_2443_table[16] = {0x0,0x8,0x3,0x9, 0x0,0xa,0x0,0xb, 0x0,0x0,0x0,0xd, 0x0,0x0,0x0,0xf};

#define NR_FREQS	49

/* Table should be ordered, low to high. */
static s3c24xx_settings_t s3c24xx_settings[NR_FREQS] =
{
	/* freq,       pll_mdiv, pll_pdiv, pll_sdiv, div_arm, div_hclk, div_pclk, reg_armdiv, reg_hdivn, reg_pdivn */
	{  12000,	  0,		0,	0,	1,	1,		1,	0,	0,		0 }, /*  12.0 MHz */
	{  24000,	 64,		7,	3,	1,	1,		1,	0,	0,		0 }, /*  24.0 MHz */
	{  33000,	 69,		5,	2,	1,	1,		1,	1,	0,		0 }, /*  33.0 MHz */
	{  40000,	 52,		7,	2,	1,	1,		1,	0,	0,		0 }, /*  40.0 MHz */
	{  45000,	 22,		2,	2,	1,	1,		1,	0,	0,		0 }, /*  45.0 MHz */
	{  50000,	 42,		4,	2,	1,	1,		1,	0,	0,		0 }, /*  50.0 MHz */
	{  55000,	 47,		4,	2,	1,	1,		1,	0,	0,		0 }, /*  55.0 MHz */
	{  60000,	 72,		6,	2,	1,	1,		1,	0,	0,		0 }, /*  60.0 MHz */
	{  66000,	 69,		5,	2,	1,	1,		1,	0,	0,		0 }, /*  66.0 MHz */
	{  70000,	 62,		4,	2,	1,	1,		2,	0,	0,		1 }, /*  70.0 MHz */
	{  75000,	 17,		2,	1,	1,	1,		2,	0,	0,		1 }, /*  75.0 MHz */
	{  80000,	 52,		7,	1,	1,	1,		2,	0,	0,		1 }, /*  80.0 MHz */
	{  85000,	 77,		4,	2,	1,	1,		2,	0,	0,		1 }, /*  85.0 MHz */
	{  90000,	 52,		6,	1,	1,	1,		2,	0,	0,		1 }, /*  90.0 MHz */
	{  95000,	 87,		4,	2,	1,	1,		2,	0,	0,		1 }, /*  95.0 MHz */
	{ 100000,	 42,		1,	1,	1,	1,		2,	1,	0,		1 }, /* 100.0 MHz */
	{ 105000,	 62,		2,	1,	1,	1,		2,	1,	0,		1 }, /* 105.0 MHz */
	{ 110000,	 47,		1,	1,	1,	1,		2,	1,	0,		1 }, /* 110.0 MHz */
	{ 115000,	107,		4,	2,	1,	1,		2,	0,	0,		1 }, /* 115.0 MHz */
	{ 120000,	242,		23,	1,	1,	1,		2,	0,	0,		1 }, /* 120.0 MHz */
	{ 125000,	242,		22,	1,	1,	1,		2,	0,	0,		1 }, /* 125.0 MHz */
	{ 132000,	 80,		2,	1,	1,	2,		2,	1,	1,		0 }, /* 132.0 MHz */
	{ 136000,	 60,		4,	1,	1,	2,		4,	0,	1,		1 }, /* 136.0 MHz */
	{ 140000,	 62,		1,	1,	1,	2,		4,	1,	1,		1 }, /* 140.0 MHz */
	{ 145000,	137,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 145.0 MHz */
	{ 150000,	 67,		1,	1,	1,	2,		4,	1,	1,		1 }, /* 150.0 MHz */
	{ 155000,	147,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 155.0 MHz */
	{ 160000,	 72,		4,	0,	1,	2,		4,	1,	1,		1 }, /* 160.0 MHz */
	{ 164000,	 33,		1,	1,	1,	2,		4,	0,	1,		1 }, /* 164.0 MHz */
	{ 168000,	 48,		2,	0,	1,	2,		4,	1,	1,		1 }, /* 168.0 MHz */
	{ 175000,	167,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 175.0 MHz */
	{ 180000,	 52,		2,	0,	1,	2,		4,	1,	1,		1 }, /* 180.0 MHz */
	{ 185000,	177,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 185.0 MHz */
	{ 190000,	 87,		4,	0,	1,	2,		4,	1,	1,		1 }, /* 190.0 MHz */
	{ 195000,	 57,		2,	1,	1,	2,		4,	0,	1,		1 }, /* 195.0 MHz */
	{ 200000,	 42,		1,	1,	1,	2,		4,	0,	1,		1 }, /* 200.0 MHz */
	{ 205000,	197,		10,	1,	1,	2,		4,	0,	1,		1 }, /* 205.0 MHz */
	{ 210000,	 62,		2,	1,	1,	2,		4,	0,	1,		1 }, /* 210.0 MHz */
	{ 215000,	207,		10,	1,	1,	2,		4,	0,	1,		1 }, /* 215.0 MHz */
	{ 220000,	 47,		1,	1,	1,	2,		4,	0,	1,		1 }, /* 220.0 MHz */
	{ 225000,	 67,		2,	1,	1,	2,		4,	0,	1,		1 }, /* 225.0 MHz */
	{ 230000,	107,		4,	1, 	1,	2,		4,	0,	1,		1 }, /* 230.0 MHz */
	{ 235000,	227,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 235.0 MHz */
	{ 240000,	 72,		2,	1,	1,	2,		4,	0,	1,		1 }, /* 240.0 MHz */
	{ 245000,	237,		4,	2,	1,	2,		4,	0,	1,		1 }, /* 245.0 MHz */
	{ 250000,	117,		4,	1,	1,	2,		4,	0,	1,		1 }, /* 250.0 MHz */
	{ 255000,	 77,		2,	1,	1,	2,		4,	0,	1,		1 }, /* 255.0 MHz */
	{ 260000,	 57,		1,	1,	1,	2,		4,	0,	1,		1 }, /* 260.0 MHz */
	{ 264000,	 80,		2,	1,	1,	2,		4,	0,	1,		1 }  /* 264.0 MHz */
};

struct cpufreq_frequency_table s3c24xx_valid_freq[NR_FREQS+1];

static int cur_config = 0;
extern void s3c2410_freq_change( u32 mpllcon_val, u32 clkdivn_val, u32 new_freq, u32 old_freq, u32 new_hclk, u32 old_hclk );
extern void asm_s3c2412_freq_change( u32 mpllcon_val, u32 clkdivn_val, u32 new_freq, u32 old_freq, u32 new_hclk, u32 old_hclk );
void s3c2412_freq_change( u32 mpllcon_val, u32 clkdivn_val, s3c24xx_settings_t *n, s3c24xx_settings_t *o )
{
	asm volatile(
		"	stmfd	sp!, {r0-r5, r14}\n"
		"	mov	r0, %0\n" 
		"	mov	r1, %1\n" 
		"	mov	r2, %2\n" 
		"	mov	r3, %3\n" 
		"	mov	r4, %4\n" 
		"	mov	r5, %5\n" 
		"	bl	asm_s3c2412_freq_change\n"
		"	ldmfd	sp!, {r0-r5, r14}\n" : :
			"r" (mpllcon_val), "r" (clkdivn_val), "r" (n->freq), "r" (o->freq), "r" (n->freq/n->div_hclk),
			"r" (o->freq/o->div_hclk) :
			"cc", "r0", "r1", "r2", "r3", "r4", "r5", "memory" );
	return;
}

static void s3c24xx_set_freqs(s3c24xx_settings_t *n, s3c24xx_settings_t *o)
{
	register unsigned long int	mpllcon, clkdivn, clksrc;
	register unsigned long int	old_hclk, new_hclk;
	
	/* Calculate the current and new hclk. */
	old_hclk=o->freq/o->div_hclk;
	new_hclk=n->freq/n->div_hclk;

	/* Determine the value to be stored in the MPLLCON register. */
	if (n->pll_mdiv | n->pll_pdiv | n->pll_sdiv) {
		/* use pll */
		mpllcon = (n->pll_mdiv << 12) | (n->pll_pdiv << 4) | (n->pll_sdiv);
	} else {
		/* use crystal */
		mpllcon = 0;
	}

	/* Handle the CPU specific transition. */
	switch (IO_GetCpuType()) {
		case GOCPU_S3C2410:
		case GOCPU_S3C2440:
		case GOCPU_S3C2442:
			clkdivn =__raw_readl(S3C2410_CLKDIVN);
			clkdivn &= ~(0x7); /* pdivn / hdivn */
			if (IO_GetCpuType() == GOCPU_S3C2410) {
				clkdivn |= ((n->reg_hdivn & 0x1) << 1) | ((n->reg_pdivn & 0x1) << 0);
			} else {
				/* 2440/2442 has more choice for hdivn */
				clkdivn = ((n->reg_hdivn & 0x3) << 1) | ((n->reg_pdivn & 0x1) << 0);
			}

			/* Change frequency. */
			s3c2410_freq_change( mpllcon, clkdivn, n->freq, o->freq, new_hclk, old_hclk );
			break;

		case GOCPU_S3C2412:
			clkdivn =__raw_readl(S3C2410_CLKDIVN);
			clkdivn &= ~(0xf);
			clkdivn |= ((n->reg_armdiv & 0x1) <<3) | ((n->reg_pdivn & 0x1) <<2) | ((n->reg_hdivn & 0x3) << 0);

			/* Change frequency. */
			s3c2412_freq_change( mpllcon, clkdivn, n, o );
			break;

		case GOCPU_S3C2443:
			/* WARNING! THIS CODE IS INCOMPLETE, AND LIKELY NON-FUNCTIONAL!!!! TODO: REPLACE BY ASM!!! */
			printk( "WARNING! MALFUNCTIONING CHANGEOVER CODE. REPLACE!!!\n" );
			if (mpllcon) {
				unsigned int clkdiv0;
				
				/* start or continue using MPLL */
				clkdiv0 = __raw_readl(S3C2443_CLKDIV0);
				clkdiv0 &= ~((0xf << 9) | (0xf << 0));
				/* use lookup-table for armdiv (0=1:1 etc), use prediv instead of hdivn */
				clkdiv0 |= (armdiv_2443_table[n->reg_armdiv] << 9) | ((n->reg_pdivn & 0x1) <<2) | ((n->reg_hdivn & 0x3) << 4);
				if (n->div_hclk == 1) {
					clkdiv0 |= (1 << 13); /* armclk = hclk, enable DVS */
				}
				mpllcon = (n->pll_mdiv << 16) | (n->pll_pdiv << 8) | (n->pll_sdiv); /* 2443 layout is different... */
				if (n->freq > o->freq) {
					__raw_writel(clkdiv0, S3C2443_CLKDIV0); /* gearing up, set clock dividers first */
				}
				__raw_writel(mpllcon, S3C2443_MPLLCON); /* MPLL configuration, start PLL */
				mdelay(1); /* wait until PLL locks */
				clksrc = __raw_readl(S3C2443_CLKSRC);
				clksrc |= (1 << 4); /* select MPLL */
				__raw_writel(clksrc, S3C2443_CLKSRC); /* select crystal as clocksource */
				
				__raw_writel(clkdiv0, S3C2443_CLKDIV0); /* set clock dividers (again) */
			} else {
				/* select crystal */
				clksrc = __raw_readl(S3C2443_CLKSRC);
				clksrc &= ~(1 << 4); /* select crystal */
				__raw_writel(clksrc, S3C2443_CLKSRC); /* select crystal as clocksource */
				mpllcon = __raw_readl(S3C2443_MPLLCON);
				mpllcon |= (1 << 24);
				__raw_writel(mpllcon, S3C2443_MPLLCON); /* turn off MPLL */
			}
			break;
		default:
			break;
	}
}

static unsigned int s3c24xx_freq_to_idx(unsigned int khz, int up)
{
	int i;
	for (i=(up ? 0 : (NR_FREQS - 1)); ((i < NR_FREQS) && up) || ((i >= 0) && !up); i=(up ? (i+1) : (i-1)) )
	{
		if( ((s3c24xx_settings[i].freq >= khz) && up) || ((s3c24xx_settings[i].freq <= khz) && !up) )
			break;
	}
	return i;
}

static unsigned int s3c24xx_clkfreq_to_idx(unsigned int khz, unsigned long int (*getdiv)( s3c24xx_settings_t * ), int up )
{
	int			i;
	s3c24xx_settings_t	*curr;
	unsigned int		curr_freq;
	for (i=(up ? 0 : (NR_FREQS - 1)); ((i < NR_FREQS) && up) || ((i >= 0) && !up); i=(up ? (i+1) : (i-1)) )
	{
		curr=&(s3c24xx_settings[i]);
		curr_freq=(curr->freq * curr->div_arm)/getdiv( curr );
		if( ((curr_freq >= khz) && up) || ((curr_freq <= khz) && down) )
			break;
	}
	return i;
}

static void s3c24xx_cpu2clk( unsigned long int min_cpu, unsigned long int max_cpu,
			     s3c24xx_settings_t **min, s3c24xx_settings_t **max )
{
	int			index;

	/* Get the right array index. Note that freq_to_idx will always choose the higher index if */
	/* the value is not exactly equal. Correct for this. We want to choose the higher index for*/
	/* the lower boundary, but the lower index for the higher boundary. */
	index=s3c24xx_freq_to_idx( min_cpu, 1 );
	if( index >= NR_FREQS )
		index=NR_FREQS-1;
	*min=&(s3c24xx_settings[index]);

	index=s3c24xx_freq_to_idx( max_cpu, 0 );
	if( (s3c24xx_settings[index].freq != max_cpu) && (index != 0) )
		index-=1;
	else if( index >= NR_FREQS )
		index=NR_FREQS-1;
	*max=&(s3c24xx_settings[index]);
	return;
}

/* This routine calculates the HCLK (min and max) from the given policy or freq structure. */
static void s3c24xx_cpu2hclk( unsigned long int min_cpu, unsigned long int max_cpu,
			      unsigned long int *min_hclk, unsigned long int *max_hclk )
{
	s3c24xx_settings_t	*min;
	s3c24xx_settings_t	*max;

	/* Get the right index. */
	s3c24xx_cpu2clk( min_cpu, max_cpu, &min, &max );

	/* Now calculate back to the HCLK using these values. */
	*min_hclk=min->div_arm * min_cpu/min->div_hclk;
	*max_hclk=max->div_arm * max_cpu/max->div_hclk;
	return;
}

static void s3c24xx_cpu2pclk( unsigned long int min_cpu, unsigned long int max_cpu,
			      unsigned long int *min_pclk, unsigned long int *max_pclk )
{
	s3c24xx_settings_t	*min;
	s3c24xx_settings_t	*max;

	/* Get the right index. */
	s3c24xx_cpu2clk( min_cpu, max_cpu, &min, &max );

	/* Now calculate back to the PCLK using these values. */
	*min_pclk=min->div_arm * min_cpu/min->div_pclk;
	*max_pclk=max->div_arm * max_cpu/max->div_pclk;
	return;
}

unsigned long int s3c24xx_freq2pclk( s3c24xx_settings_t *curr )
{
	return curr->div_pclk;
}

unsigned long int s3c24xx_freq2hclk( s3c24xx_settings_t *curr )
{
	return curr->div_hclk;
}

/* This routine sets the values specified in src_min and src_max, assuming they are within the ranges */
/* specified by *min_cpu and *max_cpu. Otherwise the values are ignored. */
static void s3c24xx_clk2cpu( unsigned long int (*getdiv)( s3c24xx_settings_t * ),
			     unsigned long int src_min, unsigned long int src_max,
			     unsigned long int *min_cpu, unsigned long int *max_cpu )
{
	int			index;
	s3c24xx_settings_t	*min;
	s3c24xx_settings_t	*max;
	unsigned long int	div_min;
	unsigned long int	div_max;
	unsigned long int	new_min;
	unsigned long int	new_max;

	/* Get the right array index. Note that hclkfreq_to_idx will always choose the higher index */
	/* if the value is not exactly equal. Correct for this. We want to choose the higher index */
	/* for the lower boundary, but the lower index for the higher boundary. */
	index=s3c24xx_clkfreq_to_idx( src_min, getdiv, 1 );
	if( index >= NR_FREQS )
		index=NR_FREQS-1;
	min=&(s3c24xx_settings[index]);
	div_min=getdiv( min );

	index=s3c24xx_clkfreq_to_idx( src_max, getdiv, 0 );
	if( index < 0 ) index=0;
	div_max=getdiv( &(s3c24xx_settings[index]) );
	if( (s3c24xx_settings[index].freq*s3c24xx_settings[index].div_arm/div_max != src_max ) && (index != 0) )
		index-=1;
	else if( index >= NR_FREQS )
		index=NR_FREQS-1;
	max=&(s3c24xx_settings[index]);
	div_max=getdiv( max );

	/* Now figure out if the original value, or the new one fits better. */
	new_min=src_min * div_min/min->div_arm;
	new_max=src_max * div_max/max->div_arm;

	/* Minimum shoud not be out of the range specified by *min_cpu ~ *max_cpu. If it is, ignore the value. */
	if( (new_min < *min_cpu) || (new_min > *max_cpu) )
		new_min=*min_cpu;

	if( (new_max < *min_cpu) || (new_max > *max_cpu) )
		new_max=*max_cpu;

	/* Save the minimum. */
	*min_cpu=new_min;
	*max_cpu=new_max;
	return;
}

/* This routine sets the values specified in min_pclk and max_pclk, assuming they are within the ranges */
/* specified by *min_cpu and *max_cpu. Otherwise the values are ignored. */
static void s3c24xx_pclk2cpu( unsigned long int min_pclk, unsigned long int max_pclk,
			      unsigned long int *min_cpu, unsigned long int *max_cpu )
{
	s3c24xx_clk2cpu( s3c24xx_freq2pclk, min_pclk, max_pclk, min_cpu, max_cpu );
	return;
}

/* This routine sets the values specified in min_hclk and max_hclk, assuming they are within the ranges */
/* specified by *min_cpu and *max_cpu. Otherwise the values are ignored. */
static void s3c24xx_hclk2cpu( unsigned long int min_hclk, unsigned long int max_hclk,
			      unsigned long int *min_cpu, unsigned long int *max_cpu )
{
	s3c24xx_clk2cpu( s3c24xx_freq2hclk, min_hclk, max_hclk, min_cpu, max_cpu );
	return;
}

/* This routine sets the values specified in min_hclk and max_hclk, assuming they are within the ranges */
/* specified in policy. Otherwise the values are ignored. */
/* For use in the policy routine. Conversion back and forth from policy structure to hclk or pclk is needed. */
static void s3c24xx_hclk2policy( struct cpufreq_policy *policy, unsigned long int min_hclk, unsigned long int max_hclk )
{
	s3c24xx_hclk2cpu( min_hclk, max_hclk, (unsigned long int *) &(policy->min), (unsigned long int *) &(policy->max) );
	return;
}

static void s3c24xx_policy2hclk( struct cpufreq_policy *policy, unsigned long int *min_hclk, unsigned long int *max_hclk )
{
	s3c24xx_cpu2hclk( policy->min, policy->max, min_hclk, max_hclk );
	return;
}

/* This routine sets the values specified in min_pclk and max_pclk, assuming they are within the ranges */
/* specified in policy. Otherwise the values are ignored. */
static void s3c24xx_pclk2policy( struct cpufreq_policy *policy, unsigned long int min_pclk, unsigned long int max_pclk )
{
	s3c24xx_pclk2cpu( min_pclk, max_pclk, (unsigned long int *) &(policy->min), (unsigned long int *) &(policy->max) );
	return;
}

static void s3c24xx_policy2pclk( struct cpufreq_policy *policy, unsigned long int *min_pclk, unsigned long int *max_pclk )
{
	s3c24xx_cpu2pclk( policy->min, policy->max, min_pclk, max_pclk );
	return;
}

/* For use in the transition routine. Only converting from the cpufreqs structure to hclk/pclk is needed. */
static void s3c24xx_trans2hclk( struct cpufreq_freqs *freqs, unsigned long int *min_hclk, unsigned long int *max_hclk )
{
	s3c24xx_cpu2hclk( freqs->old, freqs->new, min_hclk, max_hclk );
	return;
}

static void s3c24xx_trans2pclk( struct cpufreq_freqs *freqs, unsigned long int *min_pclk, unsigned long int *max_pclk )
{
	s3c24xx_cpu2pclk( freqs->old, freqs->new, min_pclk, max_pclk );
	return;
}

static int s3c24xx_target(struct cpufreq_policy *policy,
			 unsigned int target_freq,
			 unsigned int relation)
{
	struct device *dev = NULL;
	struct clk *pclk, *hclk, *fclk;
	int new_config = 0;
	s3c24xx_settings_t *old_cfg, *new_cfg;

	struct cpufreq_freqs freqs;
	switch(relation){
	case CPUFREQ_RELATION_L:
		new_config = s3c24xx_freq_to_idx(target_freq, 1);
		if (s3c24xx_settings[new_config].freq > policy->max)
			new_config++;
		break;
	case CPUFREQ_RELATION_H:
		new_config = s3c24xx_freq_to_idx(target_freq, 1);
		if (new_config > 0) {
			if ((s3c24xx_settings[new_config].freq > target_freq) &&
			    (s3c24xx_settings[new_config - 1].freq >= policy->min))
				new_config--;
		}
		break;
	}

	/* Ensure that we don't exceed the boundaries. */
	if( new_config >= NR_FREQS ) new_config = NR_FREQS - 1;

	/* Check if we're actually changing something. If not, don't do anything. */
	if( cur_config == new_config )
		return 0;
		
	old_cfg = &s3c24xx_settings[cur_config];
	new_cfg = &s3c24xx_settings[new_config];
	freqs.cpu = 0;
	freqs.old = old_cfg->freq/old_cfg->div_arm;
	freqs.new = new_cfg->freq/new_cfg->div_arm;
	freqs.trans2hclk=s3c24xx_trans2hclk;
	freqs.trans2pclk=s3c24xx_trans2pclk;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	pclk = clk_get(dev, "pclk");
	hclk = clk_get(dev, "hclk");
	fclk = clk_get(dev, "fclk");

	/* set clockdividers, set PLL, let it lock, set clockdividers, set memory refresh */
	s3c24xx_set_freqs(new_cfg, old_cfg);

	clk_set_rate(fclk, (new_cfg->freq * 1000) / new_cfg->div_arm);
	clk_set_rate(hclk, (new_cfg->freq * 1000) / new_cfg->div_hclk);
	clk_set_rate(pclk, (new_cfg->freq * 1000) / new_cfg->div_pclk);

	clk_put(fclk);
	clk_put(hclk);
	clk_put(pclk);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	cur_config = new_config;

	return 0;
}

/* While this is dirty, we need to fill in the policy hclk fields without using the set_policy routine */
/* since set_policy is meant only for CPUs which can set their own clocks at will. To work around this */
/* if the policy is accepted, we set the hclk settings in the policy structure from this routine. */
static int s3c24xx_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu)
		return -EINVAL;

	/* Verify if the speed is in the allowed limits. */
	return cpufreq_frequency_table_verify( policy, s3c24xx_valid_freq );
}

static unsigned int s3c24xx_getspeed(unsigned int cpu)
{
	return s3c24xx_settings[cur_config].freq; // TODO
}

/* NOTE: Since everything (well, almost everything) is derived from HCLK, our 'CPU' clock is HCLK. */
static int __init s3c24xx_cpu_init(struct cpufreq_policy *policy)
{
	struct clk *fclk;
	struct device *dev = NULL;
	unsigned long int	index=0;

	printk(KERN_INFO "S3C24xx CpuFreq, (c) 2006 TomTom B.V.\n");

	// only support single CPU
	if (policy->cpu != 0)
		return -EINVAL;

	fclk = clk_get(dev, "fclk");
	clk_put(fclk);

	policy->cur = clk_get_rate(fclk) / 1000;
	policy->min = 12000;
	policy->max = policy->cur;

	/* save the pointers to the conversion routines. */
	policy->policy2hclk=s3c24xx_policy2hclk;
	policy->policy2pclk=s3c24xx_policy2pclk;
	policy->hclk2policy=s3c24xx_hclk2policy;
	policy->pclk2policy=s3c24xx_pclk2policy;

	// find current config
	cur_config = s3c24xx_freq_to_idx(policy->cur, 1);
	if (s3c24xx_settings[cur_config].freq != policy->cur) {
		printk(KERN_INFO "WARNING: current CPUFREQ speed %ud not found in table, other settings disabled\n", policy->cur);
		policy->cpuinfo.min_freq = policy->cur;
		policy->cpuinfo.max_freq = policy->cur;
	} else {
		policy->cpuinfo.min_freq = policy->min;
		policy->cpuinfo.max_freq = policy->max;
	}

	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.transition_latency = 10000;

	/* Initialize the valid freq table, to be used by the verify routine. */
	for( index=0;index < (sizeof( s3c24xx_settings )/sizeof( s3c24xx_settings[0] )); index++ )
	{
		s3c24xx_valid_freq[index].index=index;
		s3c24xx_valid_freq[index].frequency=s3c24xx_settings[index].freq;
	}
	s3c24xx_valid_freq[index].index=index;
	s3c24xx_valid_freq[index].frequency=CPUFREQ_TABLE_END;
	return 0;
}

static struct cpufreq_driver s3c24xx_driver = {
	.flags		= CPUFREQ_STICKY,
	.init		= s3c24xx_cpu_init,
	.verify		= s3c24xx_verify_speed,
	.target		= s3c24xx_target,
	.setpolicy	= NULL,
	.get		= s3c24xx_getspeed,
	.name		= "s3c24xx",
};

static int __init s3c24xx_dram_init(void)
{
	// TODO: test archtype
	return cpufreq_register_driver(&s3c24xx_driver);
}

arch_initcall(s3c24xx_dram_init);
