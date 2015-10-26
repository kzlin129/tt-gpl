#ifndef _TRACE_TRAP_H
#define _TRACE_TRAP_H

#include <linux/tracepoint.h>

DEFINE_TRACE(trap_entry,
	TPPROTO(struct pt_regs *regs, long id),
	TPARGS(regs, id));
DEFINE_TRACE(trap_exit,
	TPPROTO( void ),
	TPARGS());

#endif
