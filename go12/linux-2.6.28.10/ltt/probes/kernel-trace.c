/*
 * ltt/probes/kernel-trace.c
 *
 * kernel tracepoint probes.
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/ltt-tracer.h>
#include <trace/irq.h>
#include <trace/sched.h>
#include <trace/timer.h>
#include <trace/kernel.h>

#include "ltt-type-serializer.h"

/*
 * FIXME :
 * currently, the specialized tracepoint probes cannot call into other marker
 * probes, such as ftrace enable/disable. Given we want them to be as fast as
 * possible, it might not be so bad to lose this flexibility. But that means
 * such probes would have to connect to tracepoints on their own.
 */

/* kernel_irq_entry specialized tracepoint probe */

void probe_irq_entry(unsigned int id, struct pt_regs *regs);

DEFINE_MARKER_TP(kernel_irq_entry, irq_entry, probe_irq_entry,
	"irq_id %u kernel_mode %u ip %lu");

notrace void probe_irq_entry(unsigned int id, struct pt_regs *regs)
{
	struct marker *marker;
	struct serialize_int_int_long data;

	if (unlikely(!regs))
		regs = get_irq_regs();
	data.f1 = id;
	if (likely(regs)) {
		data.f2 = !user_mode(regs);
		data.f3 = instruction_pointer(regs);
	} else {
		data.f2 = 1;
		data.f3 = 0UL;
	}

	marker = &GET_MARKER(kernel_irq_entry);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

/* kernel_irq_exit specialized tracepoint probe */

void probe_irq_exit(irqreturn_t retval);

DEFINE_MARKER_TP(kernel_irq_exit, irq_exit, probe_irq_exit,
	"handled #1u%u");

notrace void probe_irq_exit(irqreturn_t retval)
{
	struct marker *marker;
	unsigned char data;

	data = IRQ_RETVAL(retval);

	marker = &GET_MARKER(kernel_irq_exit);
	ltt_specialized_trace(marker->single.probe_private,
		&data, sizeof(data), sizeof(data));
}

/* kernel_softirq_entry specialized tracepoint probe */

void probe_irq_softirq_entry(struct softirq_action *h,
	struct softirq_action *softirq_vec);

DEFINE_MARKER_TP(kernel_softirq_entry, irq_softirq_entry,
	probe_irq_softirq_entry, "softirq_id %lu");

notrace void probe_irq_softirq_entry(struct softirq_action *h,
	struct softirq_action *softirq_vec)
{
	struct marker *marker;
	unsigned long data;

	data = ((unsigned long)h - (unsigned long)softirq_vec) / sizeof(*h);

	marker = &GET_MARKER(kernel_softirq_entry);
	ltt_specialized_trace(marker->single.probe_private,
		&data, sizeof(data), sizeof(data));
}

/* kernel_softirq_exit specialized tracepoint probe */

void probe_irq_softirq_exit(struct softirq_action *h,
	struct softirq_action *softirq_vec);

DEFINE_MARKER_TP(kernel_softirq_exit, irq_softirq_exit,
	probe_irq_softirq_exit, "softirq_id %lu");

notrace void probe_irq_softirq_exit(struct softirq_action *h,
	struct softirq_action *softirq_vec)
{
	struct marker *marker;
	unsigned long data;

	data = ((unsigned long)h - (unsigned long)softirq_vec) / sizeof(*h);

	marker = &GET_MARKER(kernel_softirq_exit);
	ltt_specialized_trace(marker->single.probe_private,
		&data, sizeof(data), sizeof(data));
}

/* kernel_softirq_raise specialized tracepoint probe */

void probe_irq_softirq_raise(unsigned int nr);

DEFINE_MARKER_TP(kernel_softirq_raise, irq_softirq_raise,
	probe_irq_softirq_raise, "softirq_id %u");

notrace void probe_irq_softirq_raise(unsigned int nr)
{
	struct marker *marker;
	unsigned int data;

	data = nr;

	marker = &GET_MARKER(kernel_softirq_raise);
	ltt_specialized_trace(marker->single.probe_private,
		&data, sizeof(data), sizeof(data));
}

/* Standard probes */
void probe_irq_tasklet_low_entry(struct tasklet_struct *t)
{
	trace_mark_tp(kernel_tasklet_low_entry, irq_tasklet_low_entry,
		probe_irq_tasklet_low_entry, "func %p data %lu",
		t->func, t->data);
}

void probe_irq_tasklet_low_exit(struct tasklet_struct *t)
{
	trace_mark_tp(kernel_tasklet_low_exit, irq_tasklet_low_exit,
		probe_irq_tasklet_low_exit, "func %p data %lu",
		t->func, t->data);
}

void probe_irq_tasklet_high_entry(struct tasklet_struct *t)
{
	trace_mark_tp(kernel_tasklet_high_entry, irq_tasklet_high_entry,
		probe_irq_tasklet_high_entry, "func %p data %lu",
		t->func, t->data);
}

void probe_irq_tasklet_high_exit(struct tasklet_struct *t)
{
	trace_mark_tp(kernel_tasklet_high_exit, irq_tasklet_high_exit,
		probe_irq_tasklet_high_exit, "func %p data %lu",
		t->func, t->data);
}

void probe_sched_kthread_stop(struct task_struct *t)
{
	trace_mark_tp(kernel_kthread_stop, sched_kthread_stop,
		probe_sched_kthread_stop, "pid %d", t->pid);
}

void probe_sched_kthread_stop_ret(int ret)
{
	trace_mark_tp(kernel_kthread_stop_ret, sched_kthread_stop_ret,
		probe_sched_kthread_stop_ret, "ret %d", ret);
}

void probe_sched_wait_task(struct rq *rq, struct task_struct *p)
{
	trace_mark_tp(kernel_sched_wait_task, sched_wait_task,
		probe_sched_wait_task, "pid %d state %ld",
		p->pid, p->state);
}

/* kernel_sched_try_wakeup specialized tracepoint probe */

void probe_sched_wakeup(struct rq *rq, struct task_struct *p);

DEFINE_MARKER_TP(kernel_sched_try_wakeup, sched_wakeup,
	probe_sched_wakeup, "pid %d cpu_id %u state %ld");

notrace void probe_sched_wakeup(struct rq *rq, struct task_struct *p)
{
	struct marker *marker;
	struct serialize_int_int_long data;

	data.f1 = p->pid;
	data.f2 = task_cpu(p);
	data.f3 = p->state;

	marker = &GET_MARKER(kernel_sched_try_wakeup);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

void probe_sched_wakeup_new(struct rq *rq, struct task_struct *p)
{
	trace_mark_tp(kernel_sched_wakeup_new_task, sched_wakeup_new,
		probe_sched_wakeup_new, "pid %d state %ld cpu_id %u",
		p->pid, p->state, task_cpu(p));
}

/* kernel_sched_schedule specialized tracepoint probe */

void probe_sched_switch(struct rq *rq, struct task_struct *prev,
		struct task_struct *next);

DEFINE_MARKER_TP(kernel_sched_schedule, sched_switch, probe_sched_switch,
	"prev_pid %d next_pid %d prev_state %ld");

notrace void probe_sched_switch(struct rq *rq, struct task_struct *prev,
		struct task_struct *next)
{
	struct marker *marker;
	struct serialize_int_int_long data;

	data.f1 = prev->pid;
	data.f2 = next->pid;
	data.f3 = prev->state;

	marker = &GET_MARKER(kernel_sched_schedule);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

void probe_sched_migrate_task(struct rq *rq, struct task_struct *p,
		int dest_cpu)
{
	trace_mark_tp(kernel_sched_migrate_task, sched_migrate_task,
		probe_sched_migrate_task, "pid %d state %ld dest_cpu %d",
		p->pid, p->state, dest_cpu);
}

void probe_sched_signal_send(int sig, struct task_struct *p)
{
	trace_mark_tp(kernel_send_signal, sched_signal_send,
		probe_sched_signal_send, "pid %d signal %d", p->pid, sig);
}

void probe_sched_process_free(struct task_struct *p)
{
	trace_mark_tp(kernel_process_free, sched_process_free,
		probe_sched_process_free, "pid %d", p->pid);
}

void probe_sched_process_exit(struct task_struct *p)
{
	trace_mark_tp(kernel_process_exit, sched_process_exit,
		probe_sched_process_exit, "pid %d", p->pid);
}

void probe_sched_process_wait(struct pid *pid)
{
	trace_mark_tp(kernel_process_wait, sched_process_wait,
		probe_sched_process_wait, "pid %d", pid_nr(pid));
}

void probe_sched_process_fork(struct task_struct *parent,
		struct task_struct *child)
{
	trace_mark_tp(kernel_process_fork, sched_process_fork,
		probe_sched_process_fork,
		"parent_pid %d child_pid %d child_tgid %d",
		parent->pid, child->pid, child->tgid);
}

void probe_sched_kthread_create(void *fn, int pid)
{
	trace_mark_tp(kernel_kthread_create, sched_kthread_create,
		probe_sched_kthread_create,
		"fn %p pid %d", fn, pid);
}

void probe_timer_itimer_expired(struct signal_struct *sig)
{
	trace_mark_tp(kernel_timer_itimer_expired, timer_itimer_expired,
		probe_timer_itimer_expired, "pid %d",
		pid_nr(sig->leader_pid));
}

void probe_timer_itimer_set(int which, struct itimerval *value)
{
	trace_mark_tp(kernel_timer_itimer_set,
		timer_itimer_set, probe_timer_itimer_set,
		"which %d interval_sec %ld interval_usec %ld "
		"value_sec %ld value_usec %ld",
		which,
		value->it_interval.tv_sec,
		value->it_interval.tv_usec,
		value->it_value.tv_sec,
		value->it_value.tv_usec);
}

/* kernel_timer_set specialized tracepoint probe */

void probe_timer_set(struct timer_list *timer);

DEFINE_MARKER_TP(kernel_timer_set, timer_set, probe_timer_set,
	"expires %lu function %p data %lu");

notrace void probe_timer_set(struct timer_list *timer)
{
	struct marker *marker;
	struct serialize_long_long_long data;

	data.f1 = timer->expires;
	data.f2 = (unsigned long)timer->function;
	data.f3 = timer->data;

	marker = &GET_MARKER(kernel_timer_set);
	ltt_specialized_trace(marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(long));
}

void probe_timer_update_time(struct timespec *_xtime,
		struct timespec *_wall_to_monotonic)
{
	trace_mark_tp(kernel_timer_update_time, timer_update_time,
		probe_timer_update_time,
		"jiffies #8u%llu xtime_sec %ld xtime_nsec %ld "
		"walltomonotonic_sec %ld walltomonotonic_nsec %ld",
		(unsigned long long)jiffies_64, _xtime->tv_sec, _xtime->tv_nsec,
		_wall_to_monotonic->tv_sec, _wall_to_monotonic->tv_nsec);
}

void probe_timer_timeout(struct task_struct *p)
{
	trace_mark_tp(kernel_timer_timeout, timer_timeout,
		probe_timer_timeout, "pid %d", p->pid);
}

void probe_kernel_printk(unsigned long retaddr)
{
	trace_mark_tp(kernel_printk, kernel_printk,
		probe_kernel_printk, "ip %lu", retaddr);
}

void probe_kernel_vprintk(unsigned long retaddr, char *buf, int len)
{
	if (len > 0) {
		unsigned int loglevel;
		int mark_len;
		char *mark_buf;
		char saved_char;

		if (buf[0] == '<' && buf[1] >= '0' &&
		   buf[1] <= '7' && buf[2] == '>') {
			loglevel = buf[1] - '0';
			mark_buf = &buf[3];
			mark_len = len - 3;
		} else {
			loglevel = default_message_loglevel;
			mark_buf = buf;
			mark_len = len;
		}
		if (mark_buf[mark_len - 1] == '\n')
			mark_len--;
		saved_char = mark_buf[mark_len];
		mark_buf[mark_len] = '\0';
		trace_mark_tp(kernel_vprintk, kernel_vprintk,
			probe_kernel_vprintk, "loglevel %c string %s ip %lu",
			loglevel, mark_buf, retaddr);
		mark_buf[mark_len] = saved_char;
	}
}

#ifdef CONFIG_MODULES
void probe_kernel_module_free(struct module *mod)
{
	trace_mark_tp(kernel_module_free, kernel_module_free,
		probe_kernel_module_free, "name %s", mod->name);
}

void probe_kernel_module_load(struct module *mod)
{
	trace_mark_tp(kernel_module_load, kernel_module_load,
		probe_kernel_module_load, "name %s", mod->name);
}
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("kernel Tracepoint Probes");
