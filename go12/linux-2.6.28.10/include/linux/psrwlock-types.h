#ifndef _LINUX_PSRWLOCK_TYPES_H
#define _LINUX_PSRWLOCK_TYPES_H

/*
 * Priority Sifting Reader-Writer Lock types definition
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * August 2008
 */

#include <linux/wait.h>
#include <asm/atomic.h>

/*
 * This table represents which is the lowest read priority context can be used
 * given the highest read priority context and the context in which the write
 * lock is taken.
 *
 * e.g. given the highest priority context from which we take the read lock is
 * interrupt context (IRQ) and the context where the write lock is taken is
 * non-preemptable (NP), we should never have a reader in context lower than
 * NP.
 *
 * X means : don't !
 *
 * X axis : Priority of writer
 * Y axis : Max priority of reader
 * Maps to :  Minimum priority of a reader.
 *
 * Highest Read Prio / Write Prio    | P     NP    BH    IRQ
 * ------------------------------------------------------------------------
 * P                                 | P     X     X     X
 * NP                                | P     NP    X     X
 * BH                                | P     NP    BH    X
 * IRQ                               | P     NP    BH    IRQ
 *
 * This table is verified by the CHECK_PSRWLOCK_MAP macro.
 */

enum psrw_prio {
	PSRW_PRIO_P,
	PSRW_PRIO_NP,
	PSRW_PRIO_BH,
	PSRW_PRIO_IRQ,
	PSRW_NR_PRIO,
};

/*
 * Possible execution contexts for readers.
 */
#define PSR_PTHREAD	(1U << PSRW_PRIO_P)
#define PSR_NPTHREAD	(1U << PSRW_PRIO_NP)
#define PSR_BH		(1U << PSRW_PRIO_BH)
#define PSR_IRQ		(1U << PSRW_PRIO_IRQ)
#define PSR_NR		PSRW_NR_PRIO
#define PSR_MASK	(PSR_PTHREAD | PSR_NPTHREAD | PSR_BH | PSR_IRQ)

typedef struct psrwlock {
	atomic_t uc;			/* Uncontended word	*/
	atomic_t ws;			/* Writers in the slow path count */
	atomic_long_t prio[PSRW_NR_PRIO]; /* Per priority slow path counts */
	u32 rctx_bitmap;		/* Allowed read execution ctx */
	enum psrw_prio wctx;		/* Allowed write execution ctx */
	wait_queue_head_t wq_read;	/* Preemptable readers wait queue */
	wait_queue_head_t wq_write;	/* Preemptable writers wait queue */
} psrwlock_t;

#define __PSRWLOCK_UNLOCKED(x, _wctx, _rctx)				\
	{								\
		.uc = { 0 },						\
		.ws = { 0 },						\
		.prio[0 ... (PSRW_NR_PRIO - 1)] = { 0 },		\
		.rctx_bitmap = (_rctx),					\
		.wctx = (_wctx),					\
		.wq_read = __WAIT_QUEUE_HEAD_INITIALIZER((x).wq_read),	\
		.wq_write = __WAIT_QUEUE_HEAD_INITIALIZER((x).wq_write),\
	}

#define DEFINE_PSRWLOCK(x, wctx, rctx)					\
	psrwlock_t x = __PSRWLOCK_UNLOCKED(x, wctx, rctx)

/*
 * Statically check that no reader with priority lower than the writer is
 * possible.
 */
#define CHECK_PSRWLOCK_MAP(x, wctx, rctx)				\
	static inline void __psrwlock_bad_context_map_##x(void)		\
	{								\
		BUILD_BUG_ON((~(~0UL << (wctx))) & (rctx));		\
	}

#endif /* _LINUX_PSRWLOCK_TYPES_H */
