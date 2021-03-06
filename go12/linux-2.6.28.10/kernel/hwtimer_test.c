
/*
* hwtimer_test.c, interface test of hardware timers in the system
*
* Copyright (c) 2007 Wind River Systems, Inc.
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
*
*/

#include <linux/module.h>
#include <linux/hwtimer.h>
#include <asm/irq_regs.h>

static void *test_data = (void *) 0xbadf00d;
static int data_corrupted = false;
static int fail = 0;
static int timer_index = 0;

static unsigned long hw_ticks = 0;
static unsigned long old_jiffies;

static int index = 0;
static int frequency = 0;

module_param(index, int, 0);
module_param(frequency, int, 0);

void scope_timer_interrupt(void * data)
{

	struct pt_regs *regs;

	hw_ticks++;

	/* validate data pointer */
	if (data != test_data)
		data_corrupted = true;

	regs = get_irq_regs();

	if (regs == NULL)
		fail++;
}

int scan_for_timers(void)
{

	int i, ret, freq, found = 0;
	int maxindex = max_hwtimer_index();

	if (maxindex == 0) {
		printk(KERN_INFO "ERROR: No timers registered with HWclock API\n");
		fail++;
		return -ENODEV;
	}
	printk(KERN_INFO "Scanning for registered timer devices...\n");

	for (i=0; i<maxindex; i++) {
		struct hwtimer_data *tdata;

		ret = get_hwtimer_info(i, &tdata);

		/* Register @ 1, 2, 3 and then de-register 2 is in theory legit. */
		if (ret != 0) {
			printk(KERN_INFO "Warning: no timer registered at index %d\n", i);
			continue;
		}

		found++;

		freq = get_hwtimer_freq(i);

		printk(KERN_INFO "Timer at index %d:\n"
			   "\tName: %s\n"
			   "\tDescription: %s\n"
			   "\tMin Hz: %u\n"
			   "\tMax Hz: %u\n"
			   "\tCurrent Hz: %u\n",
			   i, tdata->name, tdata->desc,
			   tdata->min_freq, tdata->max_freq, freq);
	}

	if (found == 0) {
		printk(KERN_INFO "ERROR: No useable timers between index 1 and %d\n", maxindex);
		fail++;
		return -ENODEV;
	}

	return 0;
}


int init_module(void)
{
	int ret, freq;
	struct hwtimer_data *tdata;

	printk(KERN_INFO "Hardware Timer API test started\n");

	old_jiffies = jiffies;

	ret = scan_for_timers();

	if (ret != 0) {
		printk(KERN_ERR "ERROR: Scan for timers failed\n");
		fail++;
		return ret;
	}

	printk(KERN_INFO "Timer scan PASS\n");

	if (index != 0) {
		printk(KERN_INFO "timer index %d specified as a module parameter\n", index);
		timer_index = index;
	}

	printk(KERN_INFO "Testing timer at index %d\n", timer_index);

	ret = get_hwtimer_info(timer_index, &tdata);
	if (ret != 0) {
		printk(KERN_INFO "ERROR: no timer available at index %d\n", timer_index);
		fail++;
		return -EINVAL;
	}

	if (frequency != 0) {
		printk(KERN_INFO "frequency %d specified as a module parameter\n", frequency);
		ret = set_hwtimer_freq(timer_index, frequency);
		if (ret != 0) {
			printk(KERN_INFO "ERROR: Failed to set new frequency value (%d)\n", ret);
			fail++;
		}
	}

	freq = get_hwtimer_freq(timer_index);

	if (frequency != 0 && freq != frequency) {
		printk(KERN_INFO "ERROR: Requested frequency (%d) does not match current frequency (%d)\n",frequency, freq);
		fail++;
	}

	if (freq == 0) {
		printk(KERN_INFO "Timer frequency currently set to zero - which won't work so well\n");
		printk(KERN_INFO "Specify a non-zero value as a module argument\n");
		fail++;
	}

	old_jiffies = jiffies;

	ret = add_hwtimer_hook(timer_index, scope_timer_interrupt, test_data);
	if (ret != 0) {
		printk(KERN_INFO "ERROR: Failed to register callback with timer %u: %d\n",
		       timer_index, ret);

		if ((ret != -EEXIST) && (ret != -EINVAL) && (ret != -EFAULT)) {
		    printk(KERN_INFO "add_hwtimer_hook() returned unexpected value: %d\n", ret);
		    return -EINVAL;
		}
	}

	ret = start_hwtimer(timer_index);

	if (ret == 0)
		printk(KERN_INFO "Timer started\n");

	if (ret == -EINVAL)
		printk(KERN_INFO "Timer has no start function, assuming already running\n");

	if (ret != 0 && ret != -EINVAL) {
			printk(KERN_INFO "ERROR: Timer start function returned unexpected value (%d)\n", ret);
			fail++;
	}

	printk(KERN_INFO "Attached to timer OK, test started.\n");
	printk(KERN_INFO "Unload module in 30s or more to end test\n");

	return 0;
}


void cleanup_module(void)
{
	int ret, freq;
	unsigned long current_jiffies, delta_jiffies;

	current_jiffies = jiffies;

	ret = stop_hwtimer(timer_index);

	if (ret == 0)
		printk(KERN_INFO "Stopped timer OK.\n");

	if (ret != 0 && ret != -EINVAL) {
			printk(KERN_INFO "ERROR: Timer stop function returned unexpected value (%d)\n", ret);
			fail++;
	}

	freq = get_hwtimer_freq(timer_index);

	ret = del_hwtimer_hook(timer_index, scope_timer_interrupt);

	if (ret != 0) {
		printk(KERN_INFO "ERROR: unregister hwtimer hook failed (%d).\n", ret);
		fail++;
	}

	printk(KERN_INFO "Hardware Timer API test finished\n");

	delta_jiffies = current_jiffies - old_jiffies;

	printk(KERN_INFO "jiffy frequency: %d\n", HZ);
	printk(KERN_INFO "hwtimer frequency: %d\n", freq);
	printk(KERN_INFO "Elapsed jiffies ticks: %lu\n", delta_jiffies);
	printk(KERN_INFO "Elapsed hwtimer ticks: %lu\n", hw_ticks);

	/* Time elapsed according to hwtimer is hw_ticks/freq.
	 * Or delta_jiffies/HZ.  Mult both items by (freq*HZ) to get
	 * a comparable quantity without division. Allow a 1/2 second
	 * either way for allowable error.
	 */
	printk(KERN_INFO "Using a common base ((timer hz) * (jiffy hz))/(sec^2):\n");
	printk(KERN_INFO "\trebased elapsed jiffies: %lu\n", delta_jiffies * freq);
	printk(KERN_INFO "\trebased elapsed hwtimer ticks: %lu\n", hw_ticks * HZ);

	if ( (hw_ticks * HZ >  (delta_jiffies + HZ/2) * freq)
		 || (hw_ticks * HZ < (delta_jiffies - HZ/2) * freq) ) {
		printk(KERN_INFO "ERROR: Expected counted ticks (%lu) to be between "
		       "[%lu, %lu]\n", hw_ticks * HZ,
		       (delta_jiffies - HZ/2) * freq, (delta_jiffies + HZ/2) * freq);
		fail++;
	}

	if (data_corrupted) {
		printk(KERN_INFO "ERROR: Data pointer corrupted during run.\n");
		fail++;
	}

	printk(KERN_INFO "Overall test status: %s.\n", fail? "FAIL" : "PASS");
	printk("\n");
}

MODULE_AUTHOR("Wind River");
MODULE_DESCRIPTION("Hardware Timer API test");
MODULE_LICENSE("GPL");

