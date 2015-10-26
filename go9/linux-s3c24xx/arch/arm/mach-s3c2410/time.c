/* linux/arch/arm/mach-s3c2410/time.c
 *
 * Copyright (C) 2003-2005 Simtec Electronics
 *	Ben Dooks, <ben@simtec.co.uk>
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

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>

#include <asm/system.h>
#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/map.h>
#include <asm/arch/regs-timer.h>
#include <asm/arch/regs-irq.h>
#include <asm/mach/time.h>
#include <asm/hardware/clock.h>

#ifdef CONFIG_MACH_TOMTOMGO
#include <barcelona/timer.h>
#endif /* CONFIG_MACH_TOMTOMGO */

#include "clock.h"

static unsigned long timer_startval;
static unsigned long timer_usec_ticks;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#include <barcelona/cpufreq_order.h>

static struct notifier_block    time_freq_transition;
static struct notifier_block    time_freq_policy;
#endif

#define TIMER_USEC_SHIFT 16

/* we use the shifted arithmetic to work out the ratio of timer ticks
 * to usecs, as often the peripheral clock is not a nice even multiple
 * of 1MHz.
 *
 * shift of 14 and 15 are too low for the 12MHz, 16 seems to be ok
 * for the current HZ value of 200 without producing overflows.
 *
 * Original patch by Dimitry Andric, updated by Ben Dooks
*/

/* Updated macro's to calculate the right scaler. */
#define SYS_TIMER4_SCALER(timer4_div)	((SYS_TIMER234_PRESCALER + 1) * timer4_div)
#define SYS_TIMER4_HZSCALER(timer4_div)	(SYS_TIMER4_SCALER(timer4_div) * HZ)

/* calc_divider
 *
 * given a pclk it will calculate the nearest divider possible to get it
 * as near to 65535 as possible. This would make the most of the 16 bit
 * value usable in the timer counter.
 */
static inline unsigned long calc_divider( unsigned long pclk )
{
	unsigned long	den=(SYS_TIMER234_PRESCALER + 1) * HZ * 65536;
	unsigned long	divider;
	unsigned long	real_div;

	/* Calculate ideal divider. */
	divider=(pclk+den - 1)/den;

	/* From the ideal divider we need to decuce a (less than) ideal divider, */
	/* as the MUX only allows 2, 4, 8 and 16. */
	real_div=2;
	while( real_div < divider ) real_div<<=1;

	/* Check that the divider is within range. */
	if( real_div > 16 ) real_div=16;

	/* This is the best we can do. Return it. */
	return real_div;
}
	
/* timer_mask_usec_ticks
 *
 * given a clock and divisor, make the value to pass into timer_ticks_to_usec
 * to scale the ticks into usecs
*/

static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}

/* timer_ticks_to_usec
 *
 * convert timer ticks to usec.
*/

static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	unsigned long res;

	res = ticks * timer_usec_ticks;
	res += 1 << (TIMER_USEC_SHIFT - 4);	/* round up slightly */

	return res >> TIMER_USEC_SHIFT;
}

/***
 * Returns microsecond  since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 * IRQs are disabled before entering here from do_gettimeofday()
 */

#define SRCPND_TIMER4 (1<<(IRQ_TIMER4 - IRQ_EINT0))

static unsigned long s3c2410_gettimeoffset (void)
{
	unsigned long tdone;
	unsigned long irqpend;
	unsigned long tval;

	/* work out how many ticks have gone since last timer interrupt */

        tval =  __raw_readl(S3C2410_TCNTO(4));
	tdone = timer_startval - tval;

	/* check to see if there is an interrupt pending */

	irqpend = __raw_readl(S3C2410_SRCPND);
	if (irqpend & SRCPND_TIMER4) {
		/* re-read the timer, and try and fix up for the missed
		 * interrupt. Note, the interrupt may go off before the
		 * timer has re-loaded from wrapping.
		 */

		tval =  __raw_readl(S3C2410_TCNTO(4));
		tdone = timer_startval - tval;

		if (tval != 0)
			tdone += timer_startval;
	}

	return timer_ticks_to_usec(tdone);
}


/*
 * IRQ handler for the timer
 */
static irqreturn_t
s3c2410_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	write_seqlock(&xtime_lock);
	timer_tick(regs);
	write_sequnlock(&xtime_lock);
	return IRQ_HANDLED;
}

static struct irqaction s3c2410_timer_irq = {
	.name		= "S3C2410 Timer Tick",
	.flags		= SA_INTERRUPT | SA_TIMER,
	.handler	= s3c2410_timer_interrupt,
};

/* get_pclk
 *
 * This routine is to obtain the pclk in the same way that the old code used to do
 * during bootup. This code is split off from the setup routine, so the setup
 * routine can be used when changing frequencies with CPUFREQ.
 */
static unsigned long get_pclk( void )
{
	unsigned long	pclk;
	struct clk	*clk;

	if (machine_is_bast() || machine_is_vr1000())
	{
		pclk=12000000;
	}
#ifdef CONFIG_MACH_TOMTOMGO
	else if (machine_is_tomtomgo())
	{
		clk = clk_get(NULL, "pclk");
		if (IS_ERR(clk))
			panic("failed to get pclk");
		pclk = clk_get_rate(clk);
		clk_put(clk);

	}
#endif
	else
	{
		clk = clk_get(NULL, "timers");
		if (IS_ERR(clk))
			panic("failed to get clock for system timer");

		clk_use(clk);
		clk_enable(clk);

		pclk = clk_get_rate(clk);
	}

	return pclk;
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 *
 * Currently we only use timer4, as it is the only timer which has no
 * other function that can be exploited externally
 */
static void s3c2410_timer_setup( unsigned long pclk )
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcnt_mod;
	unsigned long tcfg1;
	unsigned long tcfg0;

	tcnt = 0xffff;  /* default value for tcnt */
	tcnt_mod = 0;

	/* read the current timer configuration bits */

	tcon = __raw_readl(S3C2410_TCON);
	tcfg1 = __raw_readl(S3C2410_TCFG1);
	tcfg0 = __raw_readl(S3C2410_TCFG0);

	/* configure the system for whichever machine is in use */

	if (machine_is_bast() || machine_is_vr1000()) {
		/* timer is at 12MHz, scaler is 1 */
		timer_usec_ticks = timer_mask_usec_ticks(1, 12000000);
		tcnt = 12000000 / HZ;

		tcfg1 &= ~S3C2410_TCFG1_MUX4_MASK;
		tcfg1 |= S3C2410_TCFG1_MUX4_TCLK1;
	}
#ifdef CONFIG_MACH_TOMTOMGO
	else if (machine_is_tomtomgo()) {
		unsigned long timer4_div;

		/* For TomTom GO, we use a divider of 2, and a prescaler of 2,
		 * yielding the following clock count (assuming a PCLK of 50.7
		 * MHz, and HZ = 200):
		 *
		 * 50,700,000 Hz / 2 / (2 + 1) / 200 = 42250 ticks
		 */

		/* Now get the nearest divider. */
		timer4_div=calc_divider( pclk );
		
		timer_usec_ticks = timer_mask_usec_ticks(SYS_TIMER4_SCALER(timer4_div), pclk);

		tcfg1 &= ~S3C2410_TCFG1_MUX4_MASK;
		/* Convert the divider to the right bits... */
		tcfg1 |= (timer4_div >> 2) << 16;

		tcfg1 &= ~S3C2410_TCFG1_MUX1_MASK;
		tcfg1 |= S3C2410_TCFG1_MUX1_DIV2;

		tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;

                tcfg1 &= ~S3C2410_TCFG1_MUX3_MASK;
                tcfg1 |= S3C2410_TCFG1_MUX3_DIV2;

		tcfg0 &= ~S3C2410_TCFG_PRESCALER1_MASK;
		tcfg0 |= SYS_TIMER234_PRESCALER << S3C2410_TCFG_PRESCALER1_SHIFT;

		tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		tcfg0 |= SYS_TIMER01_PRESCALER;

		tcnt = pclk / SYS_TIMER4_HZSCALER( timer4_div );
		tcnt_mod = pclk % SYS_TIMER4_HZSCALER( timer4_div );
	}
#endif /* CONFIG_MACH_TOMTOMGO */
	else {
		/* for the h1940 (and others), we use the pclk from the core
		 * to generate the timer values. since values around 50 to
		 * 70MHz are not values we can directly generate the timer
		 * value from, we need to pre-scale and divide before using it.
		 *
		 * for instance, using 50.7MHz and dividing by 6 gives 8.45MHz
		 * (8.45 ticks per usec)
		 */

		/* this is used as default if no other timer can be found */

		/* configure clock tick */

		timer_usec_ticks = timer_mask_usec_ticks(6, pclk);

		tcfg1 &= ~S3C2410_TCFG1_MUX4_MASK;
		tcfg1 |= S3C2410_TCFG1_MUX4_DIV2;

		tcfg0 &= ~S3C2410_TCFG_PRESCALER1_MASK;
		tcfg0 |= ((6 - 1) / 2) << S3C2410_TCFG_PRESCALER1_SHIFT;

		tcnt = (pclk / 6) / HZ;
	}

	/* timers reload after counting zero, so reduce the count by 1 */

	tcnt--;

	/* check to see if timer is within 16bit range... */
	if( (tcnt > 0xffff) || (tcnt == 0) )
	{
		panic("setup_timer: HZ is out of range small, cannot configure timer!");
		return;
	}

	__raw_writel(tcfg1, S3C2410_TCFG1);
	__raw_writel(tcfg0, S3C2410_TCFG0);

	timer_startval = tcnt;
	__raw_writel(tcnt, S3C2410_TCNTB(4));

	/* ensure timer is stopped... */

	tcon &= ~(7<<20);
	tcon |= S3C2410_TCON_T4RELOAD;
	tcon |= S3C2410_TCON_T4MANUALUPD;

	__raw_writel(tcon, S3C2410_TCON);
	__raw_writel(tcnt, S3C2410_TCNTB(4));

	/* start the timer running */
	tcon |= S3C2410_TCON_T4START;
	tcon &= ~S3C2410_TCON_T4MANUALUPD;
	__raw_writel(tcon, S3C2410_TCON);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. Change timer freq, or refuse policy.
 */
static int time_freq_transition_handler(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs	*f = data;
	unsigned long int	pclk_new;
	unsigned long int	pclk_old;

	f->trans2pclk( f, &pclk_old, &pclk_new );

	/* Not much to do in the way of error handling. If this one goes out of bounds, it's deep shit time. */
	switch( val )
	{
		case CPUFREQ_PRECHANGE:
			if( pclk_new > pclk_old )
			{
				/* New clock is higher than old clock, so make the divider changes here. */
				/* this will temporarily cause the timing to be much slower, which is */
				/* still better than it being waaaaay too fast. */
				s3c2410_timer_setup( pclk_new * 1000 );
			}
			break;

		case CPUFREQ_POSTCHANGE:
			if( pclk_new < pclk_old )
			{
				/* New clock is lower than old clock, so set the divider now. At least */
				/* the clock didn't run too fast for a while. */
				s3c2410_timer_setup( pclk_new * 1000 );
			}
			break;
	}
	return 0;
}

static int time_freq_policy_handler(struct notifier_block *nb, unsigned long val, void *data)
{
	unsigned long int	pclk_low;
	unsigned long int	pclk_high;
	unsigned long int	pclk_min=1*((SYS_TIMER234_PRESCALER + 1) * 2 * HZ);
	unsigned long int	pclk_max=65535*((SYS_TIMER234_PRESCALER + 1) * 16 * HZ);
	struct cpufreq_policy	*policy = data;

	policy->policy2pclk( policy, &pclk_low, &pclk_high );

	switch( val )
	{
		case CPUFREQ_ADJUST:
			policy->pclk2policy( policy, pclk_min, pclk_max );
			break;

		case CPUFREQ_INCOMPATIBLE:
			if( (pclk_low >= pclk_min) && (pclk_low <= pclk_max) )
				pclk_min=pclk_low;

			if( (pclk_high >= pclk_min) && (pclk_high <= pclk_max) )
				pclk_max=pclk_high;

			if( (pclk_min != pclk_low) || (pclk_max != pclk_high) )
				policy->pclk2policy( policy, pclk_min, pclk_max );
	 		break;

		case CPUFREQ_NOTIFY:
			/* If we get here, we are in very deep shit. We're going outside of a legal */
			/* boundary of the system timer. Print an error, but we're likely going down*/
			if( (pclk_low < pclk_min) || (pclk_high > pclk_max) )
				printk( KERN_WARNING "WARNING! system timer is going out of legal bounds. Expect unstability!\n" );
	 		break;
	}
	return 0;
}
#endif

/* s3c2410_timer_resume
 *
 * This routine is created to replace the setup routine's being used as resume code. This since the
 * determining of the pclk rate has been split off from the main routine.
 */
static void s3c2410_timer_resume( void )
{
	s3c2410_timer_setup( get_pclk( ) );
	return;
}

static void __init s3c2410_timer_init (void)
{
	s3c2410_timer_setup( get_pclk( ) );

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	time_freq_transition.notifier_call = time_freq_transition_handler;
	time_freq_transition.priority=CPUFREQ_ORDER_S3C24XX_TIME_PRIO;
	time_freq_policy.notifier_call = time_freq_policy_handler;
	cpufreq_register_notifier(&time_freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&time_freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	setup_irq(IRQ_TIMER4, &s3c2410_timer_irq);
}

struct sys_timer s3c24xx_timer = {
	.init		= s3c2410_timer_init,
	.offset		= s3c2410_gettimeoffset,
	.resume		= s3c2410_timer_resume
};
