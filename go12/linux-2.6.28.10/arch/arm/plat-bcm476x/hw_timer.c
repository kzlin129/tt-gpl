/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <mach/hw_cfg.h>
#include <linux/broadcom/timer.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>
#include <asm/leds.h>
#include <asm/io.h>
#include <asm/arch/hw_timer.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/in.h>
#include <linux/ctype.h>
#include <linux/net.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/inet.h>

typedef struct {
    uint8_t vic;                // VIC controller
    uint8_t tim_ctl;            // timer controller (0-1)
    uint8_t tim_counter;        // timer counter (0-3)
} timer_map_t;

static int __init hw_clocksource_init(void);

/* Mapping from logical timer to physical timer */
static const timer_map_t timers_map[MAX_HW_TIMERS] = {
    // VIC0 timers 
    {0,0,0},
    {0,0,1},    
    {0,1,0},    
    // VIC1 timers        
    {1,1,1},    
    {1,0,2},    
    {1,1,2},    
    {1,0,3}, 
    {1,1,3},            
};
      

static unsigned int tick_count = 0;
//static uint32_t sys_timer_reload;
static int hw_timer0_set_next_event(unsigned long evt,struct clock_event_device *unused);
static void hw_timer0_set_mode(enum clock_event_mode mode, struct clock_event_device *evt);
//static void init_system_timer(uint32_t reload);

 /* Convert timer id to timer counter. */
static int32_t hw_timer_counter(int32_t timer_id)
{
    return timers_map[timer_id].tim_counter;
}

/* Convert timer id to timer controller id. */
static int32_t hw_timer_ctl(int32_t timer_id)
{
    return timers_map[timer_id].tim_ctl; 
}

/* Returns registers' base address of the time. */
uint32_t hw_timer_base(int32_t timer_id)
{
    uint32_t base_addr;

    base_addr = (hw_timer_ctl(timer_id) == 0) ? TIM0_REG_BASE_ADDR : TIM1_REG_BASE_ADDR;
    base_addr += hw_timer_counter(timer_id) * TIMER_COUNTER_REG_OFFSET;
    return base_addr;
}

void hw_timer_init(int timer_id, uint32_t reload, uint32_t ctl)
{
    uint32_t timer_base, timer_reload;
    uint32_t timer_ctrl = ctl;

    timer_base = hw_timer_base(timer_id);
	/* Disable the timer */
	writel(0, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));

    timer_reload = reload;
    if (timer_reload > 0x100000) {
		timer_reload >>= 8;
		timer_ctrl |= 0x08; /* /256 */
	} else if (timer_reload > 0x010000) {
		timer_reload >>= 4;
		timer_ctrl |= 0x04; /* /16 */
	}

    /*
     * Enable timer 
     */
	writel(timer_reload, IO_ADDRESS(timer_base + TIMER_VALUE_OFFSET));
	writel(timer_reload, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
	timer_ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_CLK2;
	writel(timer_ctrl, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
}

void hw_timer_enable(int timer_id)
{
    uint32_t timer_base, timer_ctrl;

    timer_base = hw_timer_base(timer_id);
    timer_ctrl = readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
    timer_ctrl |= TIMER_CTRL_EN;
	writel(timer_ctrl, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
}

void hw_timer_disable(int timer_id)
{
    uint32_t timer_base, timer_ctrl;

    timer_base = hw_timer_base(timer_id);
    timer_ctrl = readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
    timer_ctrl &= ~TIMER_CTRL_EN;
	writel(timer_ctrl, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
    /* Clear pending interrupt */
	writel(1, IO_ADDRESS(timer_base + TIMER_INTCLR_OFFSET));
}

/* Returns timer's counter */
uint32_t hw_timer_get_counter(int timer_id)
{
    uint32_t counter_addr;

    counter_addr = hw_timer_base(timer_id) + TIMER_VALUE_OFFSET;
    return readl(IO_ADDRESS(counter_addr));
}

static cycle_t hw_get_cycles(void)   
{
	return ~hw_timer_get_counter(1);
}

static void hw_clocksource_resume(void)
{
    uint32_t timer_base;

    timer_base = hw_timer_base(1);
	/* Disable the timer */
	writel(0, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));

    /*
     * Enable timer 
     */

	writel(0xFFFFFFFFU, IO_ADDRESS(timer_base + TIMER_VALUE_OFFSET));
	writel(0xFFFFFFFFU, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
	writel(TIMER_CTRL_PERIODIC | TIMER_CTRL_32BIT | TIMER_CTRL_PREBY16, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	writel(TIMER_CTRL_PERIODIC | TIMER_CTRL_32BIT | TIMER_CTRL_CLK2 | TIMER_CTRL_PREBY16, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	writel(TIMER_CTRL_EN | TIMER_CTRL_PERIODIC | TIMER_CTRL_32BIT | TIMER_CTRL_CLK2 | TIMER_CTRL_PREBY16, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	}

static struct clocksource clocksource_bcm476x = {
	.name       = "hw_source_timer1",
	.rating     = 250,
	.read       = (cycle_t (*)(void))hw_get_cycles,
#ifdef CONFIG_PM
	.resume     = (void (*)(void))hw_clocksource_resume,
#endif
	.mask       = CLOCKSOURCE_MASK(32),
	.shift      = 20,
	.flags      = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init hw_clocksource_init(void)
{
	clocksource_bcm476x.mult =   
		clocksource_hz2mult(CLOCK_TICK_RATE, clocksource_bcm476x.shift);

	hw_clocksource_resume();

	clocksource_register(&clocksource_bcm476x);
	return 0;
}
static struct clock_event_device clockevent_bcm476x = {
	.name       = "hw_event_timer0",
	.features   = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift      = 32,
	.set_mode   = hw_timer0_set_mode,
	.set_next_event = hw_timer0_set_next_event,
	.rating     = 300,
};

static int32_t hw_set_next_event(int timer_id, uint32_t evt,struct clock_event_device *unused)
{
    uint32_t load_addr;

    load_addr =  hw_timer_base(timer_id) + TIMER_LOAD_OFFSET;
	writel(evt,IO_ADDRESS(load_addr));
	return 0;
}

static int hw_timer0_set_next_event(unsigned long evt, struct clock_event_device *unused)
{
    return hw_set_next_event(0, (uint32_t)evt, unused);
}

#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#if TIMER_INTERVAL >= 0x100000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 8)
#define TIMER_DIVISOR	(TIMER_CTRL_PREBY256)
#define TICKS2USECS(x)	(256 * (x) / TICKS_PER_uSEC)
#elif TIMER_INTERVAL >= 0x10000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 4)		/* Divide by 16 */
#define TIMER_DIVISOR	(TIMER_CTRL_PREBY16)
#define TICKS2USECS(x)	(16 * (x) / TICKS_PER_uSEC)
#else
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TIMER_DIVISOR	(TIMER_CTRL_DIV1)
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)
#endif

static void hw_timer_set_mode(int timer_id, enum clock_event_mode mode, struct clock_event_device *evt) 
{
	unsigned long flags;
    uint32_t timer_base;

    timer_base = hw_timer_base(timer_id);

//    writel(0, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		local_irq_save(flags);
		writel(TIMER_RELOAD, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
		writel(TIMER_DIVISOR | TIMER_CTRL_CLK2 | TIMER_CTRL_PERIODIC | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		local_irq_save(flags);
		writel(TIMER_CTRL_PREBY16 | TIMER_CTRL_CLK2 | TIMER_CTRL_ONESHOTMODE | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_RESUME:
#if 0
		local_irq_save(flags);
		writel(TIMER_RELOAD, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
		writel(TIMER_DIVISOR | TIMER_CTRL_CLK2 | TIMER_CTRL_PERIODIC | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
#endif
		break;
	case CLOCK_EVT_MODE_UNUSED:
//		printk(KERN_ERR "hw_set_mode: mode 0x%d is not supported for BCM476x\n", mode);
		break;
	}
}

static void hw_timer0_set_mode(enum clock_event_mode mode, struct clock_event_device *evt) 
{
    hw_timer_set_mode(0, mode, evt);
}

static int __init hw_clockevent_init(void)
{
	clockevent_bcm476x.irq = BCM4760_INTR_TIM0_CNTR1;
	clockevent_bcm476x.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC,clockevent_bcm476x.shift);
	clockevent_bcm476x.max_delta_ns =clockevent_delta2ns(0xFFFFFFFF, &clockevent_bcm476x);
	clockevent_bcm476x.min_delta_ns =clockevent_delta2ns(0x0000000F, &clockevent_bcm476x);
	clockevent_bcm476x.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&clockevent_bcm476x);
    return 0;
}

#if 0
uint32_t sys_timer_gettimeoffset(void)
{
	uint32_t ticks1, ticks2, status;
    /*
     * Get the current number of ticks.  Note that there is a race
     * condition between us reading the timer and checking for
     * an interrupt.  We get around this by ensuring that the
     * counter has not reloaded between our two reads.
     */
	ticks2 = readl(IO_ADDRESS(TIM0_R_TIMER2VALUE_MEMADDR));
	do {
		ticks1 = ticks2;
		status = readl(IO_ADDRESS(TIM0_R_TIMER2RIS_MEMADDR));
		ticks2 = readl(IO_ADDRESS(TIM0_R_TIMER2VALUE_MEMADDR));
	} while (ticks2 > ticks1);
    /*
     * Number of ticks since last interrupt.
     */
	ticks1 = sys_timer_reload - ticks2;
    /*
     * Interrupt pending?  If so, we've reloaded once already.
     * check for the first bit in the register
     */
	if (status & 0x1)
		ticks1 += sys_timer_reload;
    /*
     * Convert the ticks to usecs
     */
	return TICKS2USECS(ticks1);
}
#endif

/*
 * IRQ handler for the timer
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t vic0_hw_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t vic0_hw_timer_interrupt(int irq, void *dev_id)
#endif
{
	struct clock_event_device *evt = &clockevent_bcm476x;
    int timer_id = (int) dev_id;
    uint32_t base_addr = hw_timer_base(timer_id);

	tick_count++;

	/*
     * clear the interrupt
     */

	writel(1, IO_ADDRESS(base_addr + TIMER_INTCLR_OFFSET));
    /*
     * clear the VIC interrupt
     */
	writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));
    if (evt->event_handler)
	    evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction hw_timer0_irq =
{
	.name       = "BCM476X Timer Tick",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	.flags      = SA_INTERRUPT | SA_TIMER,
#else
	.flags      = IRQF_DISABLED | IRQF_TIMER,
#endif
	.handler    = vic0_hw_timer_interrupt,
    .dev_id     = (void *) 0  // timer_id 0
};

void __init sys_timer_init( void )
{
    int n;
    for (n = 0; n < MAX_HW_TIMERS; n++)
        hw_timer_disable(n);

	setup_irq(BCM4760_INTR_TIM0_CNTR1, &hw_timer0_irq);

	hw_clocksource_init();
	hw_clockevent_init();
}

void sys_timer_resume( void )
{
	hw_clocksource_init();
	hw_clockevent_init();
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
//static void __init init_system_timer(uint32_t reload)
//{
    /*
     * Timer 1 is selected as the system timer.
     */
//	setup_irq(BCM4760_INTR_TIM0_CNTR1, &hw_timer0_irq);
//    hw_timer_init(1, 0xFFFFFFFFFF, (TIMER_CTRL_PERIODIC | TIMER_CTRL_EN));
//    hw_timer_init(0, 0xffffffff,(TIMER_CTRL_ONESHOTMODE | TIMER_CTRL_IE | TIMER_CTRL_EN));
//}

timer_tick_count_t timer_get_tick_count( void )
{
	return ((tick_count * SYS_TIMER_FREQ_IN_KHZ));
}
EXPORT_SYMBOL(timer_get_tick_count);

timer_tick_rate_t timer_get_tick_rate( void )
{
    return SYS_TIMER_FREQ_IN_KHZ;
}
EXPORT_SYMBOL(timer_get_tick_rate);

timer_msec_t timer_ticks_to_msec(timer_tick_count_t ticks)
{
	timer_tick_rate_t tickrate;
	timer_msec_t msec;

	tickrate = timer_get_tick_rate();

	msec = (ticks * 1000)/tickrate;
	return msec;
}
EXPORT_SYMBOL(timer_ticks_to_msec);

timer_msec_t timer_get_msec(void)
{
	return timer_ticks_to_msec(timer_get_tick_count());
}
EXPORT_SYMBOL(timer_get_msec);

EXPORT_SYMBOL(hw_timer_base);
EXPORT_SYMBOL(hw_timer_init);
EXPORT_SYMBOL(hw_timer_enable);
EXPORT_SYMBOL(hw_timer_disable);
EXPORT_SYMBOL(hw_timer_get_counter);
