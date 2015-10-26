/*
 * Copyright Â© International Business Machines  Corp., 2008
 *
 * Author: Balbir Singh <balbir@linux.vnet.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Provide memory resource limits for tasks in a control group. A lot of code is
 * duplicated from the memory controller (this code is common to almost
 * all controllers). TODO: Consider writing a tool that can generate this
 * code.
 */
#include <linux/cgroup.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/res_counter.h>
#include <linux/memrlimitcgroup.h>

struct cgroup_subsys memrlimit_cgroup_subsys;

struct memrlimit_cgroup {
	struct cgroup_subsys_state css;
	struct res_counter as_res;	/* address space counter */
};

static struct memrlimit_cgroup init_memrlimit_cgroup;

static struct memrlimit_cgroup *memrlimit_cgroup_from_cgrp(struct cgroup *cgrp)
{
	return container_of(cgroup_subsys_state(cgrp,
				memrlimit_cgroup_subsys_id),
				struct memrlimit_cgroup, css);
}

static struct memrlimit_cgroup *
memrlimit_cgroup_from_task(struct task_struct *p)
{
	return container_of(task_subsys_state(p, memrlimit_cgroup_subsys_id),
				struct memrlimit_cgroup, css);
}

/*
 * Charge the cgroup for address space usage - mmap(), malloc() (through
 * brk(), sbrk()), stack expansion, mremap(), etc - called with
 * mmap_sem held.
 */
int memrlimit_cgroup_charge_as(struct mm_struct *mm, unsigned long nr_pages)
{
	struct memrlimit_cgroup *memrcg;

	/*
	 * Null pointer is not found from callers up to now, BUG_ON
	 * is added here to report a potential bug; and there is a
	 * return value required by caller.
	 */	
	BUG_ON(!(mm->owner));
	memrcg = memrlimit_cgroup_from_task(mm->owner);
	return res_counter_charge(&memrcg->as_res, (nr_pages << PAGE_SHIFT));
}

/*
 * Uncharge the cgroup, as the address space of one of the tasks is
 * decreasing - called with mmap_sem held.
 */
void memrlimit_cgroup_uncharge_as(struct mm_struct *mm, unsigned long nr_pages)
{
	struct memrlimit_cgroup *memrcg;

	/*
	 * it looks like the caller does not care what happens in
	 * here, so there is no return since mm->owner is null.
	 */
	if(mm->owner){
		memrcg = memrlimit_cgroup_from_task(mm->owner);
		res_counter_uncharge(&memrcg->as_res, (nr_pages << PAGE_SHIFT));
	}
}

static struct cgroup_subsys_state *
memrlimit_cgroup_create(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	struct memrlimit_cgroup *memrcg;

	if (unlikely(cgrp->parent == NULL))
		memrcg = &init_memrlimit_cgroup;
	else {
		memrcg = kzalloc(sizeof(*memrcg), GFP_KERNEL);
		if (!memrcg)
			return ERR_PTR(-ENOMEM);
	}
	res_counter_init(&memrcg->as_res);
	return &memrcg->css;
}

static void memrlimit_cgroup_destroy(struct cgroup_subsys *ss,
					struct cgroup *cgrp)
{
	kfree(memrlimit_cgroup_from_cgrp(cgrp));
}

static int memrlimit_cgroup_reset(struct cgroup *cgrp, unsigned int event)
{
	struct memrlimit_cgroup *memrcg;

	memrcg = memrlimit_cgroup_from_cgrp(cgrp);
	switch (event) {
	case RES_FAILCNT:
		res_counter_reset_failcnt(&memrcg->as_res);
		break;
	}
	return 0;
}

static u64 memrlimit_cgroup_read(struct cgroup *cgrp, struct cftype *cft)
{
	return res_counter_read_u64(&memrlimit_cgroup_from_cgrp(cgrp)->as_res,
					cft->private);
}

static int memrlimit_cgroup_write_strategy(char *buf, unsigned long long *tmp)
{
	*tmp = memparse(buf, &buf);
	if (*buf != '\0')
		return -EINVAL;

	*tmp = PAGE_ALIGN(*tmp);
	return 0;
}

static ssize_t memrlimit_cgroup_write(struct cgroup *cgrp, struct cftype *cft,
					struct file *file,
					const char __user *userbuf,
					size_t nbytes,
					loff_t *ppos)
{
	return res_counter_write(&memrlimit_cgroup_from_cgrp(cgrp)->as_res,
					cft->private, userbuf,
					memrlimit_cgroup_write_strategy);
}

static struct cftype memrlimit_cgroup_files[] = {
	{
		.name = "usage_in_bytes",
		.private = RES_USAGE,
		.read_u64 = memrlimit_cgroup_read,
	},
	{
		.name = "limit_in_bytes",
		.private = RES_LIMIT,
		.write = memrlimit_cgroup_write,
		.read_u64 = memrlimit_cgroup_read,
	},
	{
		.name = "failcnt",
		.private = RES_FAILCNT,
		.trigger = memrlimit_cgroup_reset,
		.read_u64 = memrlimit_cgroup_read,
	},
};

static int memrlimit_cgroup_populate(struct cgroup_subsys *ss,
					struct cgroup *cgrp)
{
	return cgroup_add_files(cgrp, ss, memrlimit_cgroup_files,
				ARRAY_SIZE(memrlimit_cgroup_files));
}

static void memrlimit_cgroup_move_task(struct cgroup_subsys *ss,
					struct cgroup *cgrp,
					struct cgroup *old_cgrp,
					struct task_struct *p)
{
	struct mm_struct *mm;
	struct memrlimit_cgroup *memrcg, *old_memrcg;

	mm = get_task_mm(p);
	if (mm == NULL)
		return;

	/*
	 * Hold mmap_sem, so that total_vm does not change underneath us
	 */
	down_read(&mm->mmap_sem);

	rcu_read_lock();
	if (p != rcu_dereference(mm->owner))
		goto out;

	memrcg = memrlimit_cgroup_from_cgrp(cgrp);
	old_memrcg = memrlimit_cgroup_from_cgrp(old_cgrp);

	if (memrcg == old_memrcg)
		goto out;

	if (res_counter_charge(&memrcg->as_res, (mm->total_vm << PAGE_SHIFT)))
		goto out;
	res_counter_uncharge(&old_memrcg->as_res, (mm->total_vm << PAGE_SHIFT));
out:
	rcu_read_unlock();
	up_read(&mm->mmap_sem);
	mmput(mm);
}

/*
 * This callback is called with mmap_sem held
 */
static void memrlimit_cgroup_mm_owner_changed(struct cgroup_subsys *ss,
						struct cgroup *cgrp,
						struct cgroup *old_cgrp,
						struct task_struct *p)
{
	struct memrlimit_cgroup *memrcg, *old_memrcg;
	struct mm_struct *mm = NULL;

	if(p == NULL)
		return;
	if(p->flags & PF_KTHREAD) /* not user space mm */
		return;
	if((mm = p->mm) == NULL) /* insanity test */
		return;

	if(cgrp) {
		memrcg = memrlimit_cgroup_from_cgrp(cgrp);
		if (res_counter_charge(&memrcg->as_res, (mm->total_vm << PAGE_SHIFT)))
			return;
	}

	if(old_cgrp){
		old_memrcg = memrlimit_cgroup_from_cgrp(old_cgrp);
		res_counter_uncharge(&old_memrcg->as_res, (mm->total_vm << PAGE_SHIFT));
	}

	return ;
}

struct cgroup_subsys memrlimit_cgroup_subsys = {
	.name = "memrlimit",
	.subsys_id = memrlimit_cgroup_subsys_id,
	.create = memrlimit_cgroup_create,
	.destroy = memrlimit_cgroup_destroy,
	.populate = memrlimit_cgroup_populate,
	.attach = memrlimit_cgroup_move_task,
	.mm_owner_changed = memrlimit_cgroup_mm_owner_changed,
	.early_init = 0,
};
