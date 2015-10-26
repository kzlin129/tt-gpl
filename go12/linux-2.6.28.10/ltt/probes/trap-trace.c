/*
 * ltt/probes/trap-trace.c
 *
 * Trap tracepoint probes.
 */

#include <linux/module.h>
#include <trace/trap.h>

#include "ltt-type-serializer.h"

/* kernel_trap_entry specialized tracepoint probe */

void probe_trap_entry(struct pt_regs *regs, long id);

DEFINE_MARKER_TP(kernel_trap_entry, trap_entry,
	probe_trap_entry, "ip #p%ld trap_id %d");

notrace void probe_trap_entry(struct pt_regs *regs, long id)
{
	struct marker *marker;
	struct serialize_long_int data;

	if (likely(regs))
		data.f1 = instruction_pointer(regs);
	else
		data.f1 = 0UL;
	data.f2 = (unsigned int)id;

	marker = &GET_MARKER(kernel_trap_entry);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

/* kernel_syscall_exit specialized tracepoint probe */

void probe_trap_exit(void);

DEFINE_MARKER_TP(kernel_trap_exit, trap_exit,
	probe_trap_exit, MARK_NOARGS);

notrace void probe_trap_exit(void)
{
	struct marker *marker;

	marker = &GET_MARKER(kernel_trap_exit);
	ltt_specialized_trace(marker->single.probe_private,
		NULL, 0, 0);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Trap Tracepoint Probes");
