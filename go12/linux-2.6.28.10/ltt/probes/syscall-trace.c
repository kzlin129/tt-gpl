/*
 * ltt/probes/syscall-trace.c
 *
 * System call tracepoint probes.
 */

#include <linux/module.h>
#include <trace/syscall.h>

#include "ltt-type-serializer.h"


/* kernel_syscall_entry specialized tracepoint probe */

void probe_syscall_entry(struct pt_regs *regs, long id);

DEFINE_MARKER_TP(kernel_syscall_entry, syscall_entry,
	probe_syscall_entry, "ip #p%ld syscall_id %d");

notrace void probe_syscall_entry(struct pt_regs *regs, long id)
{
	struct marker *marker;
	struct serialize_long_int data;

	data.f1 = instruction_pointer(regs);
	data.f2 = (int)id;

	marker = &GET_MARKER(kernel_syscall_entry);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

/* kernel_syscall_exit specialized tracepoint probe */

void probe_syscall_exit(long ret);

DEFINE_MARKER_TP(kernel_syscall_exit, syscall_exit,
	probe_syscall_exit, "ret %ld");

notrace void probe_syscall_exit(long ret)
{
	struct marker *marker;

	marker = &GET_MARKER(kernel_syscall_exit);
	ltt_specialized_trace(marker->single.probe_private,
		&ret, sizeof(ret), sizeof(ret));
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("syscall Tracepoint Probes");
