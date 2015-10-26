/*
 * ltt/probes/rcu-trace.c
 *
 * RCU tracepoint probes.
 */

#include <linux/module.h>
#include <trace/rcu.h>

#ifdef CONFIG_CLASSIC_RCU
void probe_rcu_classic_callback(struct rcu_head *head)
{
	trace_mark_tp(rcu_classic_callback, rcu_classic_callback,
		probe_rcu_classic_callback, "func %p", head->func);
}

void probe_rcu_classic_call_rcu(struct rcu_head *head, unsigned long ip)
{
	trace_mark_tp(rcu_classic_call_rcu, rcu_classic_call_rcu,
		probe_rcu_classic_call_rcu, "func %p ip 0x%lX", head->func, ip);
}

void probe_rcu_classic_call_rcu_bh(struct rcu_head *head, unsigned long ip)
{
	trace_mark_tp(rcu_classic_call_rcu_bh, rcu_classic_call_rcu_bh,
		probe_rcu_classic_call_rcu_bh, "func %p ip 0x%lX",
		head->func, ip);
}
#endif

#ifdef CONFIG_PREEMPT_RCU
void probe_rcu_preempt_callback(struct rcu_head *head)
{
	trace_mark_tp(rcu_preempt_callback, rcu_preempt_callback,
		probe_rcu_preempt_callback, "func %p", head->func);
}

void probe_rcu_preempt_call_rcu(struct rcu_head *head, unsigned long ip)
{
	trace_mark_tp(rcu_preempt_call_rcu, rcu_preempt_call_rcu,
		probe_rcu_preempt_call_rcu, "func %p ip 0x%lX", head->func, ip);
}

void probe_rcu_preempt_call_rcu_sched(struct rcu_head *head, unsigned long ip)
{
	trace_mark_tp(rcu_preempt_call_rcu_sched, rcu_preempt_call_rcu_sched,
		probe_rcu_preempt_call_rcu_sched, "func %p ip 0x%lX",
		head->func, ip);
}
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("RCU Tracepoint Probes");
