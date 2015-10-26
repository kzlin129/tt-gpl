/*
 * kernel/trace/trace-clock-32-to-64.c
 *
 * (C) Copyright	2006,2007,2008 -
 * 		Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Extends a 32 bits clock source to a full 64 bits count, readable atomically
 * from any execution context.
 *
 * notes :
 * - trace clock 32->64 bits extended timer-based clock cannot be used for early
 *   tracing in the boot process, as it depends on timer interrupts.
 * - The timer is only on one CPU to support hotplug.
 * - We have the choice between schedule_delayed_work_on and an IPI to get each
 *   CPU to write the heartbeat. IPI has been chosen because it is considered
 *   faster than passing through the timer to get the work scheduled on all the
 *   CPUs.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/timex.h>
#include <linux/bitops.h>
#include <linux/trace-clock.h>
#include <linux/smp.h>
#include <linux/sched.h> /* FIX for m68k local_irq_enable in on_each_cpu */

/*
 * Number of hardware clock bits. The higher order bits are expected to be 0.
 * If the hardware clock source has more than 32 bits, the bits higher than the
 * 32nd will be truncated by a cast to a 32 bits unsigned. Range : 1 - 32.
 * (too few bits would be unrealistic though, since we depend on the timer to
 * detect the overflows).
 */
#define HW_BITS				32

#define HW_BITMASK			((1ULL << HW_BITS) - 1)
#define HW_LSB(hw)			((hw) & HW_BITMASK)
#define SW_MSB(sw)			((sw) & ~HW_BITMASK)

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define EXPECTED_INTERRUPT_LATENCY	30

atomic_t trace_clock;
EXPORT_SYMBOL(trace_clock);

static struct timer_list stsc_timer;
static unsigned int precalc_expire;

struct synthetic_tsc_struct {
	union {
		u64 val;
		struct {
#ifdef __BIG_ENDIAN
			u32 msb;
			u32 lsb;
#else
			u32 lsb;
			u32 msb;
#endif
		} sel;
	} tsc[2];
	unsigned int index;	/* Index of the current synth. tsc. */
};

static DEFINE_PER_CPU(struct synthetic_tsc_struct, synthetic_tsc);

/* Called from IPI : either in interrupt or process context */
static void update_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u32 tsc;

	preempt_disable();
	cpu_synth = &per_cpu(synthetic_tsc, smp_processor_id());
	tsc = trace_clock_read32();		/* Hardware clocksource read */

	if (tsc < HW_LSB(cpu_synth->tsc[cpu_synth->index].sel.lsb)) {
		unsigned int new_index = 1 - cpu_synth->index; /* 0 <-> 1 */
		/*
		 * Overflow
		 * Non atomic update of the non current synthetic TSC, followed
		 * by an atomic index change. There is no write concurrency,
		 * so the index read/write does not need to be atomic.
		 */
		cpu_synth->tsc[new_index].val =
			(SW_MSB(cpu_synth->tsc[cpu_synth->index].val)
				| (u64)tsc) + (1ULL << HW_BITS);
		cpu_synth->index = new_index;	/* atomic change of index */
	} else {
		/*
		 * No overflow : We know that the only bits changed are
		 * contained in the 32 LSBs, which can be written to atomically.
		 */
		cpu_synth->tsc[cpu_synth->index].sel.lsb =
			SW_MSB(cpu_synth->tsc[cpu_synth->index].sel.lsb) | tsc;
	}
	preempt_enable();
}

/* Called from buffer switch : in _any_ context (even NMI) */
u64 notrace trace_clock_read_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u64 ret;
	unsigned int index;
	u32 tsc;

	preempt_disable_notrace();
	cpu_synth = &per_cpu(synthetic_tsc, smp_processor_id());
	index = cpu_synth->index;		/* atomic read */
	tsc = trace_clock_read32();		/* Hardware clocksource read */

	/* Overflow detection */
	if (unlikely(tsc < HW_LSB(cpu_synth->tsc[index].sel.lsb)))
		ret = (SW_MSB(cpu_synth->tsc[index].val) | (u64)tsc)
			+ (1ULL << HW_BITS);
	else
		ret = SW_MSB(cpu_synth->tsc[index].val) | (u64)tsc;
	preempt_enable_notrace();
	return ret;
}
EXPORT_SYMBOL_GPL(trace_clock_read_synthetic_tsc);

static void synthetic_tsc_ipi(void *info)
{
	update_synthetic_tsc();
}

/* We need to be in process context to do an IPI */
static void synthetic_tsc_work(struct work_struct *work)
{
	on_each_cpu(synthetic_tsc_ipi, NULL, 1);
}
static DECLARE_WORK(stsc_work, synthetic_tsc_work);

/*
 * stsc_timer : - Timer function synchronizing synthetic TSC.
 * @data: unused
 *
 * Guarantees at least 1 execution before low word of TSC wraps.
 */
static void stsc_timer_fct(unsigned long data)
{
	PREPARE_WORK(&stsc_work, synthetic_tsc_work);
	schedule_work(&stsc_work);

	mod_timer(&stsc_timer, jiffies + precalc_expire);
}

/*
 * precalc_stsc_interval: - Precalculates the interval between the clock
 * wraparounds.
 */
static int __init precalc_stsc_interval(void)
{
	precalc_expire =
		(HW_BITMASK / ((trace_clock_frequency() / HZ
			* trace_clock_freq_scale()) << 1)
			- 1 - (EXPECTED_INTERRUPT_LATENCY * HZ / 1000)) >> 1;
	WARN_ON(precalc_expire == 0);
	printk(KERN_DEBUG "Synthetic TSC timer will fire each %u jiffies.\n",
		precalc_expire);
	return 0;
}

/*
 * 	hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *
 *	Sets the new CPU's current synthetic TSC to the same value as the
 *	currently running CPU.
 *
 * 	Returns the success/failure of the operation. (NOTIFY_OK, NOTIFY_BAD)
 */
static int __cpuinit hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	unsigned int hotcpu = (unsigned long)hcpu;
	struct synthetic_tsc_struct *cpu_synth;
	u64 local_count;

	switch (action) {
	case CPU_UP_PREPARE:
		cpu_synth = &per_cpu(synthetic_tsc, hotcpu);
		local_count = trace_clock_read_synthetic_tsc();
		cpu_synth->tsc[0].val = local_count;
		cpu_synth->index = 0;
		smp_wmb();	/* Writing in data of CPU about to come up */
		break;
	case CPU_ONLINE:
		/* As we are preemptible, make sure it runs on the right cpu */
		smp_call_function_single(hotcpu, synthetic_tsc_ipi, NULL, 0);
		break;
	}
	return NOTIFY_OK;
}

/* Called from one CPU, before any tracing starts, to init each structure */
static int __init init_synthetic_tsc(void)
{
	hotcpu_notifier(hotcpu_callback, 3);
	precalc_stsc_interval();
	init_timer(&stsc_timer);
	stsc_timer.function = stsc_timer_fct;
	stsc_timer.expires = jiffies + precalc_expire;
	add_timer(&stsc_timer);
	return 0;
}

__initcall(init_synthetic_tsc);
