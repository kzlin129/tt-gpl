/**
 * ltt-type-serializer.c
 *
 * LTTng specialized type serializer.
 *
 * Copyright Mathieu Desnoyers, 2008.
 */
#include <linux/module.h>
#include "ltt-type-serializer.h"

notrace void _ltt_specialized_trace(void *probe_data,
		void *serialize_private, unsigned int data_size,
		unsigned int largest_align)
{
	int ret;
	struct ltt_active_marker *pdata;
	uint16_t eID;
	size_t slot_size;
	int channel_index;
	struct ltt_channel_struct *channel;
	struct ltt_trace_struct *trace;
	struct rchan_buf *buf;
	void *transport_data;
	uint64_t tsc;
	long buf_offset;
	int cpu;
	unsigned int rflags;

	/*
	 * If we get here, it's probably because we have useful work to do.
	 */
	if (unlikely(ltt_traces.num_active_traces == 0))
		return;

	rcu_read_lock_sched_notrace();
	cpu = smp_processor_id();
	__get_cpu_var(ltt_nesting)++;

	pdata = (struct ltt_active_marker *)probe_data;
	eID = pdata->id;
	channel_index = pdata->channel;

	/* Iterate on each trace */
	list_for_each_entry_rcu(trace, &ltt_traces.head, list) {
		if (unlikely(!trace->active))
			continue;
		if (unlikely(!ltt_run_filter(trace, eID)))
			continue;
#ifdef CONFIG_LTT_DEBUG_EVENT_SIZE
		rflags = LTT_RFLAG_ID_SIZE;
#else
		if (unlikely(eID >= LTT_FREE_EVENTS))
			rflags = LTT_RFLAG_ID;
		else
			rflags = 0;
#endif
		channel = ltt_get_channel_from_index(trace, channel_index);
		/* reserve space : header and data */
		ret = ltt_reserve_slot(trace, channel, &transport_data,
					data_size, &slot_size, &buf_offset,
					&tsc, &rflags,
					largest_align, cpu);
		if (unlikely(ret < 0))
			continue; /* buffer full */

		/* FIXME : could probably encapsulate transport better. */
		buf = ((struct rchan *)channel->trans_channel_data)->buf[cpu];
		/* Out-of-order write : header and data */
		buf_offset = ltt_write_event_header(trace,
					channel, buf, buf_offset,
					eID, data_size, tsc, rflags);
		if (data_size) {
			buf_offset += ltt_align(buf_offset, largest_align);
			ltt_relay_write(buf, buf_offset, serialize_private,
				data_size);
			buf_offset += data_size;
		}
		/* Out-of-order commit */
		ltt_commit_slot(channel, &transport_data, buf_offset,
				slot_size);
	}
	__get_cpu_var(ltt_nesting)--;
	rcu_read_unlock_sched_notrace();
}
EXPORT_SYMBOL_GPL(_ltt_specialized_trace);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("LTT type serializer");
