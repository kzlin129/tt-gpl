/*
 * This header specifies the early debug hooks for the kgdb8250
 * driver, which can be implemented per arch and per board.
 *
 * Author: Jason Wessel <jason.wessel@windriver.com>
 *
 * 2008 (c) Wind River System, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef _KGDB8250_H_
#define _KGDB8250_H_

#include <linux/kgdb.h>

/**
 * kgdb8250_early_debug_ready - indicates when it safe to issue uart I/O
 *
 * This function should return 1 when it is safe to read and write
 * to the uart.  By default this is a weak alias which always returns
 * true, unless overriden by an architecture or board specific
 * implementation.  This is overriden in case that uart I/O is only
 * available some time after early kernel parameter processing.
 */
extern int kgdb8250_early_debug_ready(void);

/**
 * kgdb8250_arch_init - active early debugging, if configured
 *
 * This function must be implemented and called in the architecture or
 * board specific code if kgdb8250_early_debug_ready() is implemented.
 * This function should be called as soon as the board is able to
 * process exceptions and the uart is setup for reading and writing.
 * This function will invoke the kgdb I/O driver registration routines
 * and immediately wait for the debugger if kgdbwait was specified on
 * the kernel command line.
 */
extern void kgdb8250_arch_init(void);

#endif /* _KGDB8250_H_ */
