/*
* hwtimer.h, generic interface to access hardware timers in the system
*
* Copyright (c) 2006 Wind River Systems, Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <linux/list.h>
#include <linux/ptrace.h>
#include <linux/spinlock.h>

/*
 * The unchanging information visible to the users of the timers
 */

struct hwtimer_data {
	char* name;		/* Name for human consumption. */
	char* desc;		/* Description for human consumption. */
	unsigned int def_freq;	/* Default timer frequency (Hz). */
	unsigned int min_freq;	/* Minimum frequency API can set (Hz) */
	unsigned int max_freq;	/* Maximum frequency API can set (Hz) */
};

/*
 * Functions for the users of the timer(s).
 */
extern int max_hwtimer_index(void);
extern int get_hwtimer_info(unsigned int index, struct hwtimer_data **data);
extern int get_hwtimer_freq(unsigned int index);
extern int set_hwtimer_freq(unsigned int index, int new_freq);
extern int start_hwtimer(unsigned int index);
extern int stop_hwtimer(unsigned int index);
extern int add_hwtimer_hook(unsigned int index,
	void (*hook_function) (void *), void *hook_data);
extern int del_hwtimer_hook(unsigned int index,
	void (*hook_function) (void *));

/*
 * Functions etc. for the suppliers of the timer resource.  Not to be
 * monkeyed with by the consumers of the timer resource.
 */

struct hwtimer {
	struct hwtimer_data *data;
	int (*set_freq)(int freq);
	int (*get_freq)(void);
	int (*start)(void);
	int (*stop)(void);
	void (*hook)(void *data);
	void *hook_data;
#ifdef CONFIG_PREEMPT_RT
	raw_spinlock_t *lock;
#else
	spinlock_t *lock;
#endif
	int index;
	struct list_head list;
};

#ifdef CONFIG_PREEMPT_RT
#define DECLARE_HWTIMER_LOCK(x) raw_spinlock_t x = RAW_SPIN_LOCK_UNLOCKED(x)
#else
#define DECLARE_HWTIMER_LOCK(x) spinlock_t x = SPIN_LOCK_UNLOCKED
#endif

int register_hwtimer(struct hwtimer *hwtimer);
int unregister_hwtimer(struct hwtimer *hwtimer);
