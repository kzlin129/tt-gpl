#ifndef _LINUX_PSRWLOCK_H
#define _LINUX_PSRWLOCK_H

/*
 * Priority Sifting Reader-Writer Lock
 *
 * Priority Sifting Reader-Writer Lock (psrwlock) excludes reader execution
 * contexts one at a time, thus increasing the writer priority in stages. It
 * favors writers against reader threads, but lets higher priority readers in
 * even when there are subscribed writers waiting for the lock at a given lower
 * priority. Very frequent writers could starve reader threads.
 *
 * See psrwlock-types.h for types definitions.
 * See psrwlock.c for algorithmic details.
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * August 2008
 */

#include <linux/hardirq.h>
#include <linux/wait.h>
#include <linux/psrwlock-types.h>

#include <asm/atomic.h>

#define NR_PREEMPT_BUSY_LOOPS	100

/*
 * Uncontended word bits (32 bits)
 *
 * Because we deal with overflow by busy-looping waiting for the counter to
 * decrement, make sure the maximum allowed for lower-priority execution
 * contexts is lower than the maximum for higher priority execution contexts.
 * Therefore, all contexts use the same counter bits, but they reach their
 * overflow capacity one bit apart from each other (only used in the slow path).
 *
 * 3 bits for status
 * 29 bits for reader count
 *   reserve 1 high bit for irqs
 *   reserve 1 high bit for bh
 *   reserve 1 high bit for non-preemptable threads
 *   26 bits left for preemptable readers count
 */
#define UC_READER_MAX		(1U << 29)
#define UC_HARDIRQ_READER_MAX	UC_READER_MAX
#define UC_SOFTIRQ_READER_MAX	(UC_HARDIRQ_READER_MAX >> 1)
#define UC_NPTHREAD_READER_MAX	(UC_SOFTIRQ_READER_MAX >> 1)
#define UC_PTHREAD_READER_MAX	(UC_NPTHREAD_READER_MAX >> 1)

#define UC_WRITER		(1U << 0)
#define UC_SLOW_WRITER		(1U << 1)
#define UC_WQ_ACTIVE		(1U << 2)
#define UC_READER_OFFSET	(1U << 3)
#define UC_HARDIRQ_READER_MASK	((UC_HARDIRQ_READER_MAX - 1) * UC_READER_OFFSET)
#define UC_SOFTIRQ_READER_MASK	((UC_SOFTIRQ_READER_MAX - 1) * UC_READER_OFFSET)
#define UC_NPTHREAD_READER_MASK		\
	((UC_NPTHREAD_READER_MAX - 1) * UC_READER_OFFSET)
#define UC_PTHREAD_READER_MASK	((UC_PTHREAD_READER_MAX - 1) * UC_READER_OFFSET)
#define UC_READER_MASK		UC_HARDIRQ_READER_MASK


/*
 * Writers in slow path count and mutexes (32 bits)
 *
 * 1 bit for WS_WQ_MUTEX (wait queue mutex, always taken with irqs off)
 * 1 bit for WS_COUNT_MUTEX (protects writer count and UC_SLOW_WRITER updates,
 *                           taken in initial writer context).
 * 1 bit for WS_LOCK_MUTEX (single writer in critical section)
 * 29 bits for writer count.
 */
#define WS_WQ_MUTEX		(1U << 0)
#define WS_COUNT_MUTEX		(1U << 1)
#define WS_LOCK_MUTEX		(1U << 2)

#define WS_MAX			(1U << 29)
#define WS_OFFSET		(1U << 3)
#define WS_MASK			((WS_MAX - 1) * WS_OFFSET)


/*
 * Per-context slow path reader and writer count maximum, offset and mask.
 * unsigned long type. Used to atomically detect that there is no contention in
 * a given slow path context and subscribe a writer or let a reader take the
 * slow path context lock.
 */
#define CTX_WOFFSET		(1UL << 0)
#define CTX_WMAX		(1UL << (BITS_PER_LONG/2))
#define CTX_WMASK		((CTX_WMAX - 1) * CTX_WOFFSET)

#define CTX_ROFFSET		CTX_WMAX
#define CTX_RMAX		(1UL << (BITS_PER_LONG/2))
#define CTX_RMASK		((CTX_RMAX - 1) * CTX_ROFFSET)


/*
 * Internal slow paths.
 */
extern asmregparm
void _psread_lock_slow_irq(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _psread_trylock_slow_irq(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
void _psread_lock_slow_bh(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _psread_trylock_slow_bh(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
void _psread_lock_slow_inatomic(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _psread_trylock_slow_inatomic(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
void _psread_lock_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _psread_lock_interruptible_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _psread_trylock_slow(unsigned int uc, psrwlock_t *rwlock);

extern asmregparm
void _pswrite_lock_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _pswrite_lock_interruptible_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
int _pswrite_trylock_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
void _pswrite_unlock_slow(unsigned int uc, psrwlock_t *rwlock);
extern asmregparm
void _psrwlock_wakeup(unsigned int uc, psrwlock_t *rwlock);

#ifdef CONFIG_HAVE_PSRWLOCK_ASM_CALL
#include <asm/call_64.h>
#else
#define psread_lock_slow_irq		_psread_lock_slow_irq
#define psread_trylock_slow_irq		_psread_trylock_slow_irq
#define psread_lock_slow_bh		_psread_lock_slow_bh
#define psread_trylock_slow_bh		_psread_trylock_slow_bh
#define psread_lock_slow_inatomic	_psread_lock_slow_inatomic
#define psread_trylock_slow_inatomic	_psread_trylock_slow_inatomic
#define psread_lock_slow		_psread_lock_slow
#define psread_lock_interruptible_slow	_psread_lock_interruptible_slow
#define psread_trylock_slow		_psread_trylock_slow

#define pswrite_lock_slow		_pswrite_lock_slow
#define pswrite_lock_interruptible_slow	_pswrite_lock_interruptible_slow
#define pswrite_trylock_slow		_pswrite_trylock_slow
#define pswrite_unlock_slow		_pswrite_unlock_slow
#define psrwlock_wakeup			_psrwlock_wakeup
#endif

/*
 * psrwlock-specific latency tracing, maps to standard macros by default.
 */
#ifdef CONFIG_PSRWLOCK_LATENCY_TEST
#include <linux/psrwlock-latency-trace.h>
#else
static inline void psrwlock_profile_latency_reset(void)
{ }
static inline void psrwlock_profile_latency_print(void)
{ }

#define psrwlock_irq_save(flags)		local_irq_save(flags)
#define psrwlock_irq_restore(flags)		local_irq_restore(flags)
#define psrwlock_irq_disable()			local_irq_disable()
#define psrwlock_irq_enable()			local_irq_enable()
#define psrwlock_bh_disable()			local_bh_disable()
#define psrwlock_bh_enable()			local_bh_enable()
#define psrwlock_bh_enable_ip(ip)		local_bh_enable_ip(ip)
#define psrwlock_preempt_disable()		preempt_disable()
#define psrwlock_preempt_enable()		preempt_enable()
#define psrwlock_preempt_enable_no_resched()	preempt_enable_no_resched()
#endif

/*
 * Internal preemption/softirq/irq disabling helpers. Optimized into simple use
 * of standard local_irq_disable, local_bh_disable, preempt_disable by the
 * compiler since wctx and rctx are constant.
 */

static inline void write_context_disable(enum psrw_prio wctx, u32 rctx)
{
	if (wctx != PSRW_PRIO_IRQ && (rctx & PSR_IRQ))
		psrwlock_irq_disable();
	else if (wctx != PSRW_PRIO_BH && (rctx & PSR_BH))
		psrwlock_bh_disable();
	else if (wctx != PSRW_PRIO_NP && (rctx & PSR_NPTHREAD))
		psrwlock_preempt_disable();
}

static inline void write_context_enable(enum psrw_prio wctx, u32 rctx)
{
	if (wctx != PSRW_PRIO_IRQ && (rctx & PSR_IRQ))
		psrwlock_irq_enable();
	else if (wctx != PSRW_PRIO_BH && (rctx & PSR_BH))
		psrwlock_bh_enable();
	else if (wctx != PSRW_PRIO_NP && (rctx & PSR_NPTHREAD))
		psrwlock_preempt_enable();
}

/*
 * psrwlock_preempt_check must have a uc parameter read with a memory
 * barrier making sure the slow path variable writes and the UC_WQ_ACTIVE flag
 * read are done in this order (either a smp_mb() or a atomic_sub_return()).
 */
static inline void psrwlock_preempt_check(unsigned int uc, psrwlock_t *rwlock)
{
	if (unlikely(uc & UC_WQ_ACTIVE))
		psrwlock_wakeup(uc, rwlock);
}


/*
 * API
 */

/* Reader lock */

/*
 * many readers, from irq/softirq/non preemptable and preemptable thread
 * context. Protects against writers.
 *
 * Read lock fastpath :
 *
 * A cmpxchg is used here and _not_ a simple add because a lower-priority reader
 * could block the writer while it is waiting for readers to clear the
 * uncontended path. This would happen if, for instance, the reader gets
 * interrupted between the add and the moment it gets to the slow path.
 */

/*
 * Called from any context.
 * Statically check for preemptable writer to compile-out the check if all the
 * contexts accessing the lock are non-preemptable.
 */
static inline void psread_unlock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc = atomic_sub_return(UC_READER_OFFSET, &rwlock->uc);
	if (wctx == PSRW_PRIO_P || (rctx & PSR_PTHREAD))
		psrwlock_preempt_check(uc, rwlock);
}

/*
 * Called from interrupt disabled or interrupt context.
 */
static inline void psread_lock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_irq(uc, rwlock);
}

static inline int psread_trylock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
	return psread_trylock_slow_irq(uc, rwlock);
}

/*
 * Called from softirq context.
 */

static inline void psread_lock_bh(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_BH));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_bh(uc, rwlock);
}

static inline int psread_trylock_bh(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_BH));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
	return psread_trylock_slow_bh(uc, rwlock);
}


/*
 * Called from non-preemptable thread context.
 */

static inline void psread_lock_inatomic(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_NPTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_inatomic(uc, rwlock);
}

static inline int psread_trylock_inatomic(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_NPTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
	return psread_trylock_slow_inatomic(uc, rwlock);
}


/*
 * Called from preemptable thread context.
 */

static inline void psread_lock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow(uc, rwlock);
}

static inline int psread_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 0;
	return psread_lock_interruptible_slow(uc, rwlock);
}

static inline int psread_trylock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
	return psread_trylock_slow(uc, rwlock);
}


/* Writer Lock */

/*
 * ctx is the context map showing which contexts can take the read lock and
 * which context is using the write lock.
 *
 * Write lock use example, where the lock is used by readers in interrupt,
 * preemptable context and non-preemptable context. The writer lock is taken in
 * preemptable context.
 *
 * static DEFINE_PSRWLOCK(lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 * CHECK_PSRWLOCK_MAP(lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 *
 *  pswrite_lock(&lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 *  ...
 *  pswrite_unlock(&lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 */
static inline
void pswrite_lock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return;
	else
		pswrite_lock_slow(uc, rwlock);
}

static inline
int pswrite_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return 0;
	else
		return pswrite_lock_interruptible_slow(uc, rwlock);
}

static inline
int pswrite_trylock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return 1;
	else
		return pswrite_trylock_slow(uc, rwlock);
}

static inline
void pswrite_unlock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	/*
	 * atomic_cmpxchg makes sure we commit the data before reenabling
	 * the lock. Will take the slow path if there are active readers, if
	 * UC_SLOW_WRITER is set or if there are threads in the wait queue.
	 */
	uc = atomic_cmpxchg(&rwlock->uc, UC_WRITER, 0);
	if (likely(uc == UC_WRITER)) {
		write_context_enable(wctx, rctx);
		/*
		 * no need to check preempt because all wait queue masks
		 * were 0. An active wait queue would trigger the slow path.
		 */
		return;
	}
	/*
	 * Go through the slow unlock path to check if we must clear the
	 * UC_SLOW_WRITER bit.
	 */
	pswrite_unlock_slow(uc, rwlock);
}

#endif /* _LINUX_PSRWLOCK_H */
