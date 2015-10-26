#ifndef _ASM_GENERIC_TRACE_CLOCK_H
#define _ASM_GENERIC_TRACE_CLOCK_H

/*
 * include/asm-generic/trace-clock.h
 *
 * Copyright (C) 2007 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Generic tracing clock for architectures without TSC.
 */

#include <linux/param.h>	/* For HZ */
#include <asm/atomic.h>

#define TRACE_CLOCK_SHIFT 13

u64 trace_clock_read_synthetic_tsc(void);

extern atomic_t trace_clock;

static inline u32 trace_clock_read32(void)
{
	return atomic_add_return(1, &trace_clock);
}

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline void trace_clock_add_timestamp(unsigned long ticks)
{
	int old_clock, new_clock;

	do {
		old_clock = atomic_read(&trace_clock);
		new_clock = (old_clock + (ticks << TRACE_CLOCK_SHIFT))
			& (~((1 << TRACE_CLOCK_SHIFT) - 1));
	} while (atomic_cmpxchg(&trace_clock, old_clock, new_clock)
			!= old_clock);
}

static inline unsigned int trace_clock_frequency(void)
{
	return HZ << TRACE_CLOCK_SHIFT;
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}
#endif /* _ASM_GENERIC_TRACE_CLOCK_H */
