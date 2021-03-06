/*
 * ltt/ltt-tracer.c
 *
 * (C) Copyright	2005-2008 -
 * 		Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Tracing management internal kernel API. Trace buffer allocation/free, tracing
 * start/stop.
 *
 * Author:
 *	Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Inspired from LTT :
 *  Karim Yaghmour (karim@opersys.com)
 *  Tom Zanussi (zanussi@us.ibm.com)
 *  Bob Wisniewski (bob@watson.ibm.com)
 * And from K42 :
 *  Bob Wisniewski (bob@watson.ibm.com)
 *
 * Changelog:
 *  22/09/06, Move to the marker/probes mechanism.
 *  19/10/05, Complete lockless mechanism.
 *  27/05/05, Modular redesign and rewrite.
 */

#include <linux/time.h>
#include <linux/ltt-tracer.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/cpu.h>
#include <linux/kref.h>
#include <linux/delay.h>
#include <asm/atomic.h>

static void async_wakeup(unsigned long data);

static DEFINE_TIMER(ltt_async_wakeup_timer, async_wakeup, 0, 0);

/* Default callbacks for modules */
notrace int ltt_filter_control_default(enum ltt_filter_control_msg msg,
		struct ltt_trace_struct *trace)
{
	return 0;
}

int ltt_statedump_default(struct ltt_trace_struct *trace)
{
	return 0;
}

/* Callbacks for registered modules */

int (*ltt_filter_control_functor)
	(enum ltt_filter_control_msg msg, struct ltt_trace_struct *trace) =
					ltt_filter_control_default;
struct module *ltt_filter_control_owner;

/* These function pointers are protected by a trace activation check */
struct module *ltt_run_filter_owner;
int (*ltt_statedump_functor)(struct ltt_trace_struct *trace) =
					ltt_statedump_default;
struct module *ltt_statedump_owner;

/**
 * ltt_module_register : LTT module registration
 * @name : module type
 * @function : callback to register
 * @owner : module which owns the callback
 */
int ltt_module_register(enum ltt_module_function name, void *function,
		struct module *owner)
{
	int ret = 0;

	switch (name) {
	case LTT_FUNCTION_RUN_FILTER:
		if (ltt_run_filter_owner != NULL) {
			ret = -EEXIST;
			goto end;
		}
		ltt_filter_register((ltt_run_filter_functor)function);
		ltt_run_filter_owner = owner;
		break;
	case LTT_FUNCTION_FILTER_CONTROL:
		if (ltt_filter_control_owner != NULL) {
			ret = -EEXIST;
			goto end;
		}
		ltt_filter_control_functor =
			(int (*)(enum ltt_filter_control_msg,
			struct ltt_trace_struct *))function;
		break;
	case LTT_FUNCTION_STATEDUMP:
		if (ltt_statedump_owner != NULL) {
			ret = -EEXIST;
			goto end;
		}
		ltt_statedump_functor =
			(int (*)(struct ltt_trace_struct *))function;
		ltt_statedump_owner = owner;
		break;
	}

end:

	return ret;
}
EXPORT_SYMBOL_GPL(ltt_module_register);

/**
 * ltt_module_unregister : LTT module unregistration
 * @name : module type
 */
void ltt_module_unregister(enum ltt_module_function name)
{
	switch (name) {
	case LTT_FUNCTION_RUN_FILTER:
		ltt_filter_unregister();
		ltt_run_filter_owner = NULL;
		/* Wait for preempt sections to finish */
		synchronize_sched();
		break;
	case LTT_FUNCTION_FILTER_CONTROL:
		ltt_filter_control_functor = ltt_filter_control_default;
		ltt_filter_control_owner = NULL;
		break;
	case LTT_FUNCTION_STATEDUMP:
		ltt_statedump_functor = ltt_statedump_default;
		ltt_statedump_owner = NULL;
		break;
	}

}
EXPORT_SYMBOL_GPL(ltt_module_unregister);

static LIST_HEAD(ltt_transport_list);

/**
 * ltt_transport_register : LTT transport registration
 * @transport : transport structure
 *
 * Registers a transport which can be used as output to extract the data out of
 * LTTng.
 */
void ltt_transport_register(struct ltt_transport *transport)
{
	ltt_lock_traces();
	list_add_tail(&transport->node, &ltt_transport_list);
	ltt_unlock_traces();
}
EXPORT_SYMBOL_GPL(ltt_transport_register);

/**
 * ltt_transport_unregister : LTT transport unregistration
 * @transport : transport structure
 */
void ltt_transport_unregister(struct ltt_transport *transport)
{
	ltt_lock_traces();
	list_del(&transport->node);
	ltt_unlock_traces();
}
EXPORT_SYMBOL_GPL(ltt_transport_unregister);

static inline int is_channel_overwrite(enum ltt_channels chan,
	enum trace_mode mode)
{
	switch (mode) {
	case LTT_TRACE_NORMAL:
		return 0;
	case LTT_TRACE_FLIGHT:
		switch (chan) {
		case LTT_CHANNEL_METADATA:
			return 0;
		default:
			return 1;
		}
	case LTT_TRACE_HYBRID:
		switch (chan) {
		case LTT_CHANNEL_CPU:
			return 1;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

/**
 * ltt_write_trace_header : Write trace header
 * @trace : Trace information
 * @header : Memory address where the information must be written to
 */
void notrace ltt_write_trace_header(struct ltt_trace_struct *trace,
		struct ltt_subbuffer_header *header)
{
	header->magic_number = LTT_TRACER_MAGIC_NUMBER;
	header->major_version = LTT_TRACER_VERSION_MAJOR;
	header->minor_version = LTT_TRACER_VERSION_MINOR;
	header->arch_size = sizeof(void *);
	header->alignment = ltt_get_alignment();
	header->start_time_sec = trace->start_time.tv_sec;
	header->start_time_usec = trace->start_time.tv_usec;
	header->start_freq = trace->start_freq;
	header->freq_scale = trace->freq_scale;
}
EXPORT_SYMBOL_GPL(ltt_write_trace_header);

static void trace_async_wakeup(struct ltt_trace_struct *trace)
{
	/* Must check each channel for pending read wakeup */
	trace->ops->wakeup_channel(trace->channel.metadata);
	trace->ops->wakeup_channel(trace->channel.interrupts);
	trace->ops->wakeup_channel(trace->channel.processes);
	trace->ops->wakeup_channel(trace->channel.modules);
	trace->ops->wakeup_channel(trace->channel.network);
	trace->ops->wakeup_channel(trace->channel.cpu);
}

/* Timer to send async wakeups to the readers */
static void async_wakeup(unsigned long data)
{
	struct ltt_trace_struct *trace;

	/*
	 * PREEMPT_RT does not allow spinlocks to be taken within preempt
	 * disable sections (spinlock taken in wake_up). However, mainline won't
	 * allow mutex to be taken in interrupt context. Ugly.
	 * A proper way to do this would be to turn the timer into a
	 * periodically woken up thread, but it adds to the footprint.
	 */
#ifndef CONFIG_PREEMPT_RT
	rcu_read_lock_sched();
#else
	ltt_lock_traces();
#endif
	list_for_each_entry_rcu(trace, &ltt_traces.head, list) {
		trace_async_wakeup(trace);
	}
#ifndef CONFIG_PREEMPT_RT
	rcu_read_unlock_sched();
#else
	ltt_unlock_traces();
#endif

	mod_timer(&ltt_async_wakeup_timer, jiffies + LTT_PERCPU_TIMER_INTERVAL);
}

/* _ltt_trace_find :
 * find a trace by given name.
 *
 * Returns a pointer to the trace structure, NULL if not found. */
static struct ltt_trace_struct *_ltt_trace_find(const char *trace_name)
{
	struct ltt_trace_struct *trace;

	list_for_each_entry(trace, &ltt_traces.head, list)
		if (!strncmp(trace->trace_name, trace_name, NAME_MAX))
			return trace;

	return NULL;
}

/* This function must be called with traces semaphore held. */
static int _ltt_trace_create(const char *trace_name, enum trace_mode mode,
				struct ltt_trace_struct *new_trace)
{
	int err = EPERM;

	if (_ltt_trace_find(trace_name) != NULL) {
		printk(KERN_ERR "LTT : Trace %s already exists\n", trace_name);
		err = EEXIST;
		goto traces_error;
	}
	if (list_empty(&ltt_traces.head)) {
		probe_id_defrag();
		mod_timer(&ltt_async_wakeup_timer,
				jiffies + LTT_PERCPU_TIMER_INTERVAL);
		set_kernel_trace_flag_all_tasks();
	}
	list_add_rcu(&new_trace->list, &ltt_traces.head);
	synchronize_sched();
	/* Everything went fine, finish creation */
	return 0;

	/* Error handling */
traces_error:
	return err;
}

/**
 * ltt_release_transport : Release an LTT transport
 * @kref : reference count on the transport
 */
void ltt_release_transport(struct kref *kref)
{
	struct ltt_trace_struct *trace = container_of(kref,
			struct ltt_trace_struct, ltt_transport_kref);
	trace->ops->remove_dirs(trace);
}
EXPORT_SYMBOL_GPL(ltt_release_transport);

/**
 * ltt_release_trace : Release a LTT trace
 * @kref : reference count on the trace
 */
void ltt_release_trace(struct kref *kref)
{
	struct ltt_trace_struct *trace = container_of(kref,
			struct ltt_trace_struct, kref);
	kfree(trace);
}
EXPORT_SYMBOL_GPL(ltt_release_trace);

static inline void prepare_chan_size_num(unsigned *subbuf_size,
	unsigned *n_subbufs, unsigned default_size, unsigned default_n_subbufs)
{
	if (*subbuf_size == 0)
		*subbuf_size = default_size;
	if (*n_subbufs == 0)
		*n_subbufs = default_n_subbufs;
	*subbuf_size = 1 << get_count_order(*subbuf_size);
	*n_subbufs = 1 << get_count_order(*n_subbufs);

	/* Subbuf size and number must both be power of two */
	WARN_ON(hweight32(*subbuf_size) != 1);
	WARN_ON(hweight32(*n_subbufs) != 1);
}

static int ltt_trace_create(const char *trace_name, const char *trace_type,
		enum trace_mode mode,
		unsigned subbuf_size_low, unsigned n_subbufs_low,
		unsigned subbuf_size_med, unsigned n_subbufs_med,
		unsigned subbuf_size_high, unsigned n_subbufs_high)
{
	int err = 0;
	struct ltt_trace_struct *new_trace, *trace;
	unsigned long flags;
	struct ltt_transport *tran, *transport = NULL;

	prepare_chan_size_num(&subbuf_size_low, &n_subbufs_low,
		LTT_DEFAULT_SUBBUF_SIZE_LOW, LTT_DEFAULT_N_SUBBUFS_LOW);

	prepare_chan_size_num(&subbuf_size_med, &n_subbufs_med,
		LTT_DEFAULT_SUBBUF_SIZE_MED, LTT_DEFAULT_N_SUBBUFS_MED);

	prepare_chan_size_num(&subbuf_size_high, &n_subbufs_high,
		LTT_DEFAULT_SUBBUF_SIZE_HIGH, LTT_DEFAULT_N_SUBBUFS_HIGH);

	new_trace = kzalloc(sizeof(struct ltt_trace_struct), GFP_KERNEL);
	if (!new_trace) {
		printk(KERN_ERR
			"LTT : Unable to allocate memory for trace %s\n",
			trace_name);
		err = ENOMEM;
		goto traces_error;
	}

	kref_init(&new_trace->kref);
	kref_init(&new_trace->ltt_transport_kref);
	init_waitqueue_head(&new_trace->kref_wq);
	new_trace->active = 0;
	strncpy(new_trace->trace_name, trace_name, NAME_MAX);
	new_trace->mode = mode;
	new_trace->freq_scale = trace_clock_freq_scale();

	ltt_lock_traces();
	list_for_each_entry(tran, &ltt_transport_list, node) {
		if (!strcmp(tran->name, trace_type)) {
			transport = tran;
			break;
		}
	}

	if (!transport) {
		err = EINVAL;
		printk(KERN_ERR	"LTT : Transport %s is not present.\n",
			trace_type);
		ltt_unlock_traces();
		goto trace_error;
	}

	if (!try_module_get(transport->owner)) {
		err = ENODEV;
		printk(KERN_ERR	"LTT : Can't lock transport module.\n");
		ltt_unlock_traces();
		goto trace_error;
	}

	trace = _ltt_trace_find(trace_name);
	if (trace) {
		printk(KERN_ERR	"LTT : Trace name %s already used.\n",
			trace_name);
		err = EEXIST;
		goto trace_error;
	}

	new_trace->transport = transport;
	new_trace->ops = &transport->ops;

	err = new_trace->ops->create_dirs(new_trace);
	if (err)
		goto dirs_error;

	local_irq_save(flags);
	new_trace->start_freq = trace_clock_frequency();
	new_trace->start_tsc = trace_clock_read64();
	do_gettimeofday(&new_trace->start_time);
	local_irq_restore(flags);

	/*
	 * Always put the metadata channel in non-overwrite mode :
	 * This is a very low traffic channel and it can't afford to have its
	 * data overwritten : this data (marker info) is necessary to be
	 * able to read the trace.
	 */
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.control_root,
			LTT_METADATA_CHANNEL,
			&new_trace->channel.metadata, subbuf_size_low,
			n_subbufs_low,
			is_channel_overwrite(LTT_CHANNEL_METADATA, mode));
	if (err != 0)
		goto metadata_error;
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.control_root,
			LTT_INTERRUPTS_CHANNEL,
			&new_trace->channel.interrupts, subbuf_size_low,
			n_subbufs_low,
			is_channel_overwrite(LTT_CHANNEL_INTERRUPTS, mode));
	if (err != 0)
		goto interrupts_error;
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.control_root,
			LTT_PROCESSES_CHANNEL,
			&new_trace->channel.processes, subbuf_size_med,
			n_subbufs_med,
			is_channel_overwrite(LTT_CHANNEL_PROCESSES, mode));
	if (err != 0)
		goto processes_error;
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.control_root,
			LTT_MODULES_CHANNEL,
			&new_trace->channel.modules, subbuf_size_low,
			n_subbufs_low,
			is_channel_overwrite(LTT_CHANNEL_MODULES, mode));
	if (err != 0)
		goto modules_error;
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.control_root,
			LTT_NETWORK_CHANNEL,
			&new_trace->channel.network, subbuf_size_low,
			n_subbufs_low,
			is_channel_overwrite(LTT_CHANNEL_NETWORK, mode));
	if (err != 0)
		goto network_error;
	err = new_trace->ops->create_channel(trace_name, new_trace,
			new_trace->dentry.trace_root,
			LTT_CPU_CHANNEL,
			&new_trace->channel.cpu, subbuf_size_high,
			n_subbufs_high,
			is_channel_overwrite(LTT_CHANNEL_CPU, mode));
	if (err != 0)
		goto cpu_error;

	err = _ltt_trace_create(trace_name, mode, new_trace);

	if (err != 0)
		goto lock_create_error;
	ltt_unlock_traces();
	return err;

lock_create_error:
	new_trace->ops->remove_channel(new_trace->channel.cpu);
cpu_error:
	new_trace->ops->remove_channel(new_trace->channel.network);
network_error:
	new_trace->ops->remove_channel(new_trace->channel.modules);
modules_error:
	new_trace->ops->remove_channel(new_trace->channel.processes);
processes_error:
	new_trace->ops->remove_channel(new_trace->channel.interrupts);
interrupts_error:
	new_trace->ops->remove_channel(new_trace->channel.metadata);
metadata_error:
	kref_put(&new_trace->ltt_transport_kref, ltt_release_transport);
dirs_error:
	module_put(transport->owner);
trace_error:
	kref_put(&new_trace->kref, ltt_release_trace);
	wake_up_interruptible(&new_trace->kref_wq);
	ltt_unlock_traces();
traces_error:
	return err;
}

/* Must be called while sure that trace is in the list. */
static int _ltt_trace_destroy(struct ltt_trace_struct	*trace)
{
	int err = EPERM;

	if (trace == NULL) {
		err = ENOENT;
		goto traces_error;
	}
	if (trace->active) {
		printk(KERN_ERR
			"LTT : Can't destroy trace %s : tracer is active\n",
			trace->trace_name);
		err = EBUSY;
		goto active_error;
	}
	/* Everything went fine */
	list_del_rcu(&trace->list);
	synchronize_sched();
	if (list_empty(&ltt_traces.head)) {
		clear_kernel_trace_flag_all_tasks();
		/*
		 * We stop the asynchronous delivery of reader wakeup, but
		 * we must make one last check for reader wakeups pending
		 * later in __ltt_trace_destroy.
		 */
		del_timer_sync(&ltt_async_wakeup_timer);
	}
	return 0;

	/* error handling */
active_error:
traces_error:
	return err;
}

/* Sleepable part of the destroy */
static void __ltt_trace_destroy(struct ltt_trace_struct	*trace)
{
	trace->ops->finish_channel(trace->channel.metadata);
	trace->ops->finish_channel(trace->channel.interrupts);
	trace->ops->finish_channel(trace->channel.processes);
	trace->ops->finish_channel(trace->channel.modules);
	trace->ops->finish_channel(trace->channel.network);
	trace->ops->finish_channel(trace->channel.cpu);

	flush_scheduled_work();

	/*
	 * The currently destroyed trace is not in the trace list anymore,
	 * so it's safe to call the async wakeup ourself. It will deliver
	 * the last subbuffers.
	 */
	trace_async_wakeup(trace);

	trace->ops->remove_channel(trace->channel.metadata);
	trace->ops->remove_channel(trace->channel.interrupts);
	trace->ops->remove_channel(trace->channel.processes);
	trace->ops->remove_channel(trace->channel.modules);
	trace->ops->remove_channel(trace->channel.network);
	trace->ops->remove_channel(trace->channel.cpu);

	kref_put(&trace->ltt_transport_kref, ltt_release_transport);

	module_put(trace->transport->owner);

	/*
	 * Wait for lttd readers to release the files, therefore making sure
	 * the last subbuffers have been read.
	 */
	if (atomic_read(&trace->kref.refcount) > 1) {
		int ret = 0;
		__wait_event_interruptible(trace->kref_wq,
			(atomic_read(&trace->kref.refcount) == 1), ret);
	}
	kref_put(&trace->kref, ltt_release_trace);
}

static int ltt_trace_destroy(const char *trace_name)
{
	int err = 0;
	struct ltt_trace_struct *trace;

	ltt_lock_traces();
	trace = _ltt_trace_find(trace_name);
	err = _ltt_trace_destroy(trace);
	if (err)
		goto error;
	ltt_unlock_traces();
	__ltt_trace_destroy(trace);
	return err;

	/* Error handling */
error:
	ltt_unlock_traces();
	return err;
}

/* must be called from within a traces lock. */
static int _ltt_trace_start(struct ltt_trace_struct *trace)
{
	int err = 0;

	if (trace == NULL) {
		err = ENOENT;
		goto traces_error;
	}
	if (trace->active)
		printk(KERN_INFO "LTT : Tracing already active for trace %s\n",
				trace->trace_name);
	if (!try_module_get(ltt_run_filter_owner)) {
		err = ENODEV;
		printk(KERN_ERR "LTT : Can't lock filter module.\n");
		goto get_ltt_run_filter_error;
	}
	trace->active = 1;
	/* Read by trace points without protection : be careful */
	ltt_traces.num_active_traces++;
	return err;

	/* error handling */
get_ltt_run_filter_error:
traces_error:
	return err;
}

static int ltt_trace_start(const char *trace_name)
{
	int err = 0;
	struct ltt_trace_struct *trace;

	ltt_lock_traces();

	trace = _ltt_trace_find(trace_name);
	err = _ltt_trace_start(trace);
	if (err)
		goto no_trace;

	ltt_unlock_traces();

	/*
	 * Call the kernel state dump.
	 * Events will be mixed with real kernel events, it's ok.
	 * Notice that there is no protection on the trace : that's exactly
	 * why we iterate on the list and check for trace equality instead of
	 * directly using this trace handle inside the logging function.
	 */

	ltt_dump_marker_state(trace);

	if (!try_module_get(ltt_statedump_owner)) {
		err = ENODEV;
		printk(KERN_ERR
			"LTT : Can't lock state dump module.\n");
	} else {
		ltt_statedump_functor(trace);
		module_put(ltt_statedump_owner);
	}

	return err;

	/* Error handling */
no_trace:
	ltt_unlock_traces();
	return err;
}

/* must be called from within traces lock */
static int _ltt_trace_stop(struct ltt_trace_struct *trace)
{
	int err = EPERM;

	if (trace == NULL) {
		err = ENOENT;
		goto traces_error;
	}
	if (!trace->active)
		printk(KERN_INFO "LTT : Tracing not active for trace %s\n",
				trace->trace_name);
	if (trace->active) {
		trace->active = 0;
		ltt_traces.num_active_traces--;
		synchronize_sched(); /* Wait for each tracing to be finished */
	}
	module_put(ltt_run_filter_owner);
	/* Everything went fine */
	return 0;

	/* Error handling */
traces_error:
	return err;
}

static int ltt_trace_stop(const char *trace_name)
{
	int err = 0;
	struct ltt_trace_struct *trace;

	ltt_lock_traces();
	trace = _ltt_trace_find(trace_name);
	err = _ltt_trace_stop(trace);
	ltt_unlock_traces();
	return err;
}

/**
 * ltt_control : Trace control in-kernel API
 * @msg : Action to perform
 * @trace_name : Trace on which the action must be done
 * @trace_type : Type of trace (normal, flight, hybrid)
 * @args : Arguments specific to the action
 */
int ltt_control(enum ltt_control_msg msg, const char *trace_name,
		const char *trace_type, union ltt_control_args args)
{
	int err = EPERM;

	printk(KERN_ALERT "ltt_control : trace %s\n", trace_name);
	switch (msg) {
	case LTT_CONTROL_START:
		printk(KERN_DEBUG "Start tracing %s\n", trace_name);
		err = ltt_trace_start(trace_name);
		break;
	case LTT_CONTROL_STOP:
		printk(KERN_DEBUG "Stop tracing %s\n", trace_name);
		err = ltt_trace_stop(trace_name);
		break;
	case LTT_CONTROL_CREATE_TRACE:
		printk(KERN_DEBUG "Creating trace %s\n", trace_name);
		err = ltt_trace_create(trace_name, trace_type,
			args.new_trace.mode,
			args.new_trace.subbuf_size_low,
			args.new_trace.n_subbufs_low,
			args.new_trace.subbuf_size_med,
			args.new_trace.n_subbufs_med,
			args.new_trace.subbuf_size_high,
			args.new_trace.n_subbufs_high);
		break;
	case LTT_CONTROL_DESTROY_TRACE:
		printk(KERN_DEBUG "Destroying trace %s\n", trace_name);
		err = ltt_trace_destroy(trace_name);
		break;
	}
	return err;
}
EXPORT_SYMBOL_GPL(ltt_control);

/**
 * ltt_filter_control : Trace filter control in-kernel API
 * @msg : Action to perform on the filter
 * @trace_name : Trace on which the action must be done
 */
int ltt_filter_control(enum ltt_filter_control_msg msg, const char *trace_name)
{
	int err;
	struct ltt_trace_struct *trace;

	printk(KERN_DEBUG "ltt_filter_control : trace %s\n", trace_name);
	ltt_lock_traces();
	trace = _ltt_trace_find(trace_name);
	if (trace == NULL) {
		printk(KERN_ALERT
			"Trace does not exist. Cannot proxy control request\n");
		err = ENOENT;
		goto trace_error;
	}
	if (!try_module_get(ltt_filter_control_owner)) {
		err = ENODEV;
		goto get_module_error;
	}
	switch (msg) {
	case LTT_FILTER_DEFAULT_ACCEPT:
		printk(KERN_DEBUG
			"Proxy filter default accept %s\n", trace_name);
		err = (*ltt_filter_control_functor)(msg, trace);
		break;
	case LTT_FILTER_DEFAULT_REJECT:
		printk(KERN_DEBUG
			"Proxy filter default reject %s\n", trace_name);
		err = (*ltt_filter_control_functor)(msg, trace);
		break;
	default:
		err = EPERM;
	}
	module_put(ltt_filter_control_owner);

get_module_error:
trace_error:
	ltt_unlock_traces();
	return err;
}
EXPORT_SYMBOL_GPL(ltt_filter_control);

static void __exit ltt_exit(void)
{
	struct ltt_trace_struct *trace;
	struct list_head *pos, *n;
	struct list_head tmp;

	ltt_lock_traces();
	/* Stop each trace, currently being read by RCU read-side */
	list_for_each_entry_rcu(trace, &ltt_traces.head, list)
		_ltt_trace_stop(trace);
	tmp = ltt_traces.head;
	/* empty the original list head */
	INIT_LIST_HEAD(&ltt_traces.head);
	/* Wait for quiescent state. Readers have preemption disabled. */
	synchronize_sched();
	/* Safe iteration is now permitted. It does not have to be RCU-safe
	 * because no readers are left. */
	list_for_each_safe(pos, n, &tmp) {
		trace = container_of(pos, struct ltt_trace_struct, list);
		/* _ltt_trace_destroy does a synchronize_sched() */
		_ltt_trace_destroy(trace);
		__ltt_trace_destroy(trace);
	}
	ltt_unlock_traces();
}

module_exit(ltt_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Next Generation Tracer Kernel API");
