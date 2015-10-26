#ifndef _TRACE_SYSCALL_H
#define _TRACE_SYSCALL_H

#include <linux/tracepoint.h>

DEFINE_TRACE(syscall_entry,
	TPPROTO(struct pt_regs *regs, long id),
	TPARGS(regs, id));
DEFINE_TRACE(syscall_exit,
	TPPROTO(long ret),
	TPARGS(ret));


#endif
