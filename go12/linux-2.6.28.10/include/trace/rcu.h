#ifndef _TRACE_RCU_H
#define _TRACE_RCU_H

#include <linux/tracepoint.h>
#include <linux/rcupdate.h>

DEFINE_TRACE(rcu_classic_callback,
	TPPROTO(struct rcu_head *head),
	TPARGS(head));

DEFINE_TRACE(rcu_classic_call_rcu,
	TPPROTO(struct rcu_head *head, unsigned long ip),
	TPARGS(head, ip));

DEFINE_TRACE(rcu_classic_call_rcu_bh,
	TPPROTO(struct rcu_head *head, unsigned long ip),
	TPARGS(head, ip));

DEFINE_TRACE(rcu_preempt_callback,
	TPPROTO(struct rcu_head *head),
	TPARGS(head));

DEFINE_TRACE(rcu_preempt_call_rcu,
	TPPROTO(struct rcu_head *head, unsigned long ip),
	TPARGS(head, ip));

DEFINE_TRACE(rcu_preempt_call_rcu_sched,
	TPPROTO(struct rcu_head *head, unsigned long ip),
	TPARGS(head, ip));

#endif
