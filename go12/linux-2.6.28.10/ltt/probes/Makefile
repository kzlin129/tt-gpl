# LTTng tracing probes

ifdef CONFIG_FTRACE
CFLAGS_REMOVE_kernel-trace.o = -pg
CFLAGS_REMOVE_mm-trace.o = -pg
CFLAGS_REMOVE_fs-trace.o = -pg
CFLAGS_REMOVE_ipc-trace.o = -pg
CFLAGS_REMOVE_lockdep-trace.o = -pg
CFLAGS_REMOVE_rcu-trace.o = -pg
CFLAGS_REMOVE_syscall-trace.o = -pg
CFLAGS_REMOVE_trap-trace.o = -pg
endif

obj-$(CONFIG_LTT_TRACEPROBES)	+= ltt-type-serializer.o

obj-$(CONFIG_LTT_TRACEPROBES)	+= kernel-trace.o mm-trace.o fs-trace.o \
				ipc-trace.o lockdep-trace.o rcu-trace.o \
				syscall-trace.o trap-trace.o

ifeq ($(CONFIG_NET),y)
ifdef CONFIG_FTRACE
CFLAGS_REMOVE_net-trace.o = -pg
endif
obj-$(CONFIG_LTT_TRACEPROBES)	+= net-trace.o
endif
