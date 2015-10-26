/*
 * Statically trace early boot.
 *
 * Copyright (C) 2008 Tom Rix <Tom.Rix@windriver.com>
 */
#include <linux/kallsyms.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/ftrace.h>
#include <linux/fs.h>
#include <linux/sort.h>

#include <linux/irq.h>
#include <linux/stacktrace.h>
#include <asm/sections.h>

#include "trace.h"

/* Uncomment for debug output */
/* #define EARLY_TRACE_DB */

extern ftrace_func_t ftrace_trace_function;
extern int ftrace_function_enabled;

static struct trace_array *early_trace;
static unsigned long *itable;
static unsigned long itable_size;

extern initcall_t __initcall_start[], __initcall_end[], __early_initcall_end[];

/* This routine determines which kernel functions are recorded and which
   are ignored.  */
static int skip(unsigned long ip)
{
	/* Functions in the kernel init section are OK */
	if (ip >= (unsigned long)_sinittext &&
	    ip <= (unsigned long)_einittext)
		return 0;
	/* Functions in the initcall tables are OK
	   The itable holds these calls and is presorted */
	if (itable &&
	    ip >= itable[0] &&
	    ip <= itable[itable_size - 1]) {
		unsigned long s, e, m, v;

		s = m = 0;
		e = itable_size - 1;
		for (;;) {
			m = s + ((e - s) >> 1);
			v = itable[m];
			if (unlikely(ip == v))
				return 0;
			if (likely(e <= (s+1)))
				break;
			if (ip > v)
				s = m;
			else /* (ip < v) */
				e = m;
		}
	}
	/* Most functions we do not care to measure these are skipped */
	return 1;
}

static void
early_trace_func(unsigned long ip, unsigned long parent_ip)
{
	struct trace_array *tr = early_trace;
	struct trace_array_cpu *data;
	long disabled;
	int cpu;

	if (skip(ip))
		return;
	/*
	 * Does not matter if we preempt. We test the flags
	 * afterward, to see if irqs are disabled or not.
	 * If we preempt and get a false positive, the flags
	 * test will fail.
	 */
	cpu = raw_smp_processor_id();

	data = tr->data[cpu];
	disabled = atomic_inc_return(&data->disabled);

	if (likely(disabled == 1))
		trace_function(tr, data, ip, parent_ip, 0);

	atomic_dec(&data->disabled);
}

static struct ftrace_ops early_trace_ops __read_mostly =
{
	.func = early_trace_func,
};

static int compare(const void *a, const void *b)
{
	unsigned long *la = (unsigned long *) a;
	unsigned long *lb = (unsigned long *) b;

	if (*la > *lb)
		return 1;
	else if (*lb > *la)
		return -1;
	else
		return 0;
}

static void swap(void *a, void *b, int size)
{
	unsigned long *la = (unsigned long *)a;
	unsigned long *lb = (unsigned long *)b;
	unsigned long t = *la;
	*la = *lb;
	*lb = t;
}

static void early_trace_init(struct trace_array *tr)
{
#ifdef EARLY_TRACE_DB
	pr_info("early_trace_init\n");
#endif

	ftrace_function_enabled = 0;

	if (!ftrace_enabled)
		ftrace_enabled = 1;

	if (!itable) {
		unsigned long s;

		s = __initcall_end - __initcall_start;

#ifdef EARLY_TRACE_DB
		pr_info("early_trace_init init call table size %lu\n", s);
#endif
		if (s) {
			itable = kzalloc(s, GFP_KERNEL);
			if (itable) {
				unsigned long l;
				initcall_t *call = __initcall_start;
				itable_size = s / sizeof(unsigned long);

				BUG_ON(!itable_size);

				for (l = 0; l < itable_size; l++)
					itable[l] = (unsigned long) call[l];

				sort(itable, itable_size, sizeof(unsigned long),
				     compare, swap);
			}
		}
	}
	early_trace = tr;
	if (tr->ctrl) {
		int cpu;
		for_each_online_cpu(cpu)
			tracing_reset(tr->data[cpu]);

		register_ftrace_function(&early_trace_ops);
	}
}

static void early_trace_reset(struct trace_array *tr)
{
	int status = -1;

	if (itable) {
		kfree(itable);
		itable = NULL;
		itable_size = 0;
	}
	if (tr->ctrl)
		status = unregister_ftrace_function(&early_trace_ops);
}

static void early_trace_ctrl_update(struct trace_array *tr)
{
	if (tr->ctrl)
		register_ftrace_function(&early_trace_ops);
	else
		unregister_ftrace_function(&early_trace_ops);
}

#ifndef CONFIG_DYNAMIC_FTRACE
static struct tracer early_tracer __read_mostly =
{
	.name	        = "early",
	.init           = early_trace_init,
	.reset          = early_trace_reset,
	.ctrl_update	= early_trace_ctrl_update,
};
#endif

__init int init_early_trace(void)
{
	int ret = -1;
#ifndef CONFIG_DYNAMIC_FTRACE
	ret = register_tracer(&early_tracer);

	if (5 != tracing_early_set_trace_write("early", 5))
		pr_warning("Could not enable early trace\n");
	else
		pr_info("Early trace enabled\n");

	ftrace_function_enabled = 1;
#endif
	return ret;
}

