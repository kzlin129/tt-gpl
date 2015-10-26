/*
 * ltt/probes/ipc-trace.c
 *
 * IPC tracepoint probes.
 */

#include <linux/module.h>
#include <trace/ipc.h>

void probe_ipc_msg_create(long id, int flags)
{
	trace_mark_tp(ipc_msg_create, ipc_msg_create, probe_ipc_msg_create,
		"id %ld flags %d", id, flags);
}

void probe_ipc_sem_create(long id, int flags)
{
	trace_mark_tp(ipc_sem_create, ipc_sem_create, probe_ipc_sem_create,
		"id %ld flags %d", id, flags);
}

void probe_ipc_shm_create(long id, int flags)
{
	trace_mark_tp(ipc_shm_create, ipc_shm_create, probe_ipc_shm_create,
		"id %ld flags %d", id, flags);
}

void probe_ipc_call(unsigned int call, unsigned int first)
{
	trace_mark_tp(ipc_call, ipc_call, probe_ipc_call,
		"call %u first %d", call, first);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("IPC Tracepoint Probes");
