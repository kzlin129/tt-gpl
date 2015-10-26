/*
* hwtimer.c, generic interface to access hardware timers in the system
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


#include <linux/hwtimer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>


static LIST_HEAD(hwtimer_list);
static spinlock_t hwtimer_list_lock = SPIN_LOCK_UNLOCKED;

/*
 * Assumes there will never be more than MAXINT timer devices installed
 * on a system....
 */

static int next_index = 0;

static int assign_hwtimer_index(void)
{
	return next_index++;
}

static struct hwtimer *find_hwtimer(int index)
{
	struct list_head *pos;
	struct hwtimer *hwt;

	spin_lock(&hwtimer_list_lock);
	list_for_each(pos, &hwtimer_list) {
		hwt = list_entry(pos, struct hwtimer, list);
		if (hwt->index == index) {
			spin_unlock(&hwtimer_list_lock);
			return hwt;
		}
	}

	spin_unlock(&hwtimer_list_lock);
	return NULL;
}

/*********************************************************************
 *  Functions that are exported for the users of the timers
 *********************************************************************/

int max_hwtimer_index(void)
{
	return next_index;
}
EXPORT_SYMBOL(max_hwtimer_index);

int get_hwtimer_info(unsigned int index, struct hwtimer_data **data)
{
	struct hwtimer *hwtimer;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	*data = hwtimer->data;

	return 0;
}
EXPORT_SYMBOL(get_hwtimer_info);

int get_hwtimer_freq(unsigned int index)
{
	struct hwtimer *hwtimer;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	return hwtimer->get_freq();
}
EXPORT_SYMBOL(get_hwtimer_freq);

int set_hwtimer_freq(unsigned int index, int freq)
{
	struct hwtimer *hwtimer;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	if (hwtimer->set_freq == NULL)
		return -ENOSYS;

	if (freq > hwtimer->data->max_freq || freq < hwtimer->data->min_freq)
		return -EINVAL;

	return hwtimer->set_freq(freq);
}
EXPORT_SYMBOL(set_hwtimer_freq);

/*
 * Start a timer device creating periodic events.  May not be supported
 * on all devices.
 */
int start_hwtimer(unsigned int index)
{
	struct hwtimer *hwtimer;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	if (hwtimer->start == NULL)
		return -EINVAL;

	return hwtimer->start();
}
EXPORT_SYMBOL(start_hwtimer);

/*
 * Stop a timer device creating periodic events.  May not be supported
 * on all devices.
 */
int stop_hwtimer(unsigned int index)
{
	struct hwtimer *hwtimer;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	if (hwtimer->stop == NULL)
		return -EINVAL;

	return hwtimer->stop();
}
EXPORT_SYMBOL(stop_hwtimer);

int add_hwtimer_hook(unsigned int index,
       	void (*hook_function) (void *), void *hook_data)
{
	struct hwtimer *hwtimer;
	unsigned long flags;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	spin_lock_irqsave(hwtimer->lock, flags);

	if (hwtimer->hook != NULL) {
		spin_unlock_irqrestore(hwtimer->lock, flags);
		return -EBUSY;
	}

	hwtimer->hook_data = hook_data;
	mb();
	hwtimer->hook = hook_function;

	spin_unlock_irqrestore(hwtimer->lock, flags);

	return 0;
}
EXPORT_SYMBOL(add_hwtimer_hook);

int del_hwtimer_hook(unsigned int index,
	       void (*hook_function) (void *data))
{
	struct hwtimer *hwtimer;
	unsigned long flags;

	hwtimer = find_hwtimer(index);

	if (hwtimer == NULL)
		return -ENODEV;

	spin_lock_irqsave(hwtimer->lock, flags);

	if (hwtimer->hook == NULL) {
		spin_unlock_irqrestore(hwtimer->lock, flags);
		return -EINVAL;
	}

	hwtimer->hook = NULL;
	spin_unlock_irqrestore(hwtimer->lock, flags);

	return 0;
}
EXPORT_SYMBOL(del_hwtimer_hook);

/*********************************************************************
 * Functions that the providers of the timer resources make use of
 *********************************************************************/

/*
 * Called by a timer device that is willing to share its facilities
 */
int register_hwtimer(struct hwtimer* hwtimer)
{

	if (hwtimer == NULL) {
		printk(KERN_WARNING "hwtimer: Timer to reg is NULL\n");
		return -EINVAL;
	}

	if (hwtimer->data == NULL) {
		printk(KERN_WARNING "Timer has no data\n");
		return -EINVAL;
	}

	if (hwtimer->data->name == NULL) {
		printk(KERN_WARNING "Timer has no name\n");
		return -EINVAL;
	}

	if (hwtimer->data->desc == NULL) {
		printk(KERN_WARNING "Timer has no description\n");
		return -EINVAL;
	}

	spin_lock(&hwtimer_list_lock);
	hwtimer->index = assign_hwtimer_index();
	list_add(&(hwtimer->list), &hwtimer_list);
	spin_unlock(&hwtimer_list_lock);

	printk(KERN_INFO "hwtimer: Added %s (%s) at index=%d\n",
	       	hwtimer->data->name, hwtimer->data->desc, hwtimer->index);

	return 0;
}
EXPORT_SYMBOL(register_hwtimer);

int unregister_hwtimer(struct hwtimer *hwtimer)
{
	if (hwtimer == NULL) {
		printk(KERN_WARNING "hwtimer: Timer to unreg is NULL\n");
		return -EINVAL;
	}

	/* Warn on any remaining hook functions here */

	if (hwtimer->hook != NULL)
		printk(KERN_WARNING "hwtimer: hook present on dead timer %s\n",
				hwtimer->data->name);

	printk(KERN_INFO "hwtimer: removing timer %s\n", hwtimer->data->name);

	spin_lock(&hwtimer_list_lock);
	list_del(&(hwtimer->list));
	spin_unlock(&hwtimer_list_lock);

	return 0;
}
EXPORT_SYMBOL(unregister_hwtimer);

