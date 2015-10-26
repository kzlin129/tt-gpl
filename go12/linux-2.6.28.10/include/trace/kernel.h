#ifndef _TRACE_KERNEL_H
#define _TRACE_KERNEL_H

#include <linux/tracepoint.h>

DEFINE_TRACE(kernel_printk,
	TPPROTO(unsigned long retaddr),
	TPARGS(retaddr));
DEFINE_TRACE(kernel_vprintk,
	TPPROTO(unsigned long retaddr, char *buf, int len),
	TPARGS(retaddr, buf, len));
DEFINE_TRACE(kernel_module_free,
	TPPROTO(struct module *mod),
	TPARGS(mod));
DEFINE_TRACE(kernel_module_load,
	TPPROTO(struct module *mod),
	TPARGS(mod));

#endif
