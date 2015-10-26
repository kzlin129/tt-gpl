#ifndef _TRACE_LOCKDEP_H
#define _TRACE_LOCKDEP_H

#include <linux/tracepoint.h>

/*
 * lockdep tracing must be very careful with respect to reentrancy.
 *
 * It should not use immediate values for activation because it involves
 * traps called when the code patching is done.
 */
DEFINE_TRACE(lockdep_hardirqs_on,
	TPPROTO(unsigned long retaddr),
		TPARGS(retaddr));
DEFINE_TRACE(lockdep_hardirqs_off,
	TPPROTO(unsigned long retaddr),
		TPARGS(retaddr));
DEFINE_TRACE(lockdep_softirqs_on,
	TPPROTO(unsigned long retaddr),
		TPARGS(retaddr));
DEFINE_TRACE(lockdep_softirqs_off,
	TPPROTO(unsigned long retaddr),
		TPARGS(retaddr));
DEFINE_TRACE(lockdep_lock_acquire,
	TPPROTO(unsigned long retaddr, unsigned int subclass,
			struct lockdep_map *lock, int trylock, int read,
			int hardirqs_off),
		TPARGS(retaddr, subclass, lock, trylock, read, hardirqs_off));
DEFINE_TRACE(lockdep_lock_release,
	TPPROTO(unsigned long retaddr, struct lockdep_map *lock, int nested),
		TPARGS(retaddr, lock, nested));

#endif
