/* biocontrol.c - Block I/O Controller
 *
 * Copyright IBM Corporation, 2007
 * Author Balbir Singh <balbir@linux.vnet.ibm.com>
 *
 * Copyright 2007 OpenVZ SWsoft Inc
 * Author: Pavel Emelianov <xemul@openvz.org>
 *
 * Copyright VA Linux Systems Japan, 2008
 * Author Hirokazu Takahashi <taka@valinux.co.jp>
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
 */

#include <linux/module.h>
#include <linux/cgroup.h>
#include <linux/mm.h>
#include <linux/blkdev.h>
#include <linux/smp.h>
#include <linux/bit_spinlock.h>
#include <linux/idr.h>
#include <linux/err.h>
#include <linux/biocontrol.h>

/* return corresponding bio_cgroup object of a cgroup */
static inline struct bio_cgroup *cgroup_bio(struct cgroup *cgrp)
{
	return container_of(cgroup_subsys_state(cgrp, bio_cgroup_subsys_id),
			    struct bio_cgroup, css);
}

static struct idr bio_cgroup_id;
static DEFINE_SPINLOCK(bio_cgroup_idr_lock);

static struct cgroup_subsys_state *
bio_cgroup_create(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	struct bio_cgroup *biog;
	struct io_context *ioc;
	int error;

	if (!cgrp->parent) {
		static struct bio_cgroup default_bio_cgroup;
		static struct io_context default_bio_io_context;

		biog = &default_bio_cgroup;
		ioc = &default_bio_io_context;
		init_io_context(ioc);

		idr_init(&bio_cgroup_id);
		biog->id = 0;

		page_cgroup_init();
	} else {
		biog = kzalloc(sizeof(*biog), GFP_KERNEL);
		ioc = alloc_io_context(GFP_KERNEL, -1);
		if (!ioc || !biog) {
			error = -ENOMEM;
			goto out;
		}
retry:
		if (unlikely(!idr_pre_get(&bio_cgroup_id, GFP_KERNEL))) {
			error = -EAGAIN;
			goto out;
		}
		spin_lock_irq(&bio_cgroup_idr_lock);
		error = idr_get_new_above(&bio_cgroup_id,
						(void *)biog, 1, &biog->id);
		spin_unlock_irq(&bio_cgroup_idr_lock);
		if (error == -EAGAIN)
			goto retry;
		else if (error)
			goto out;
	}

	ioc->id = biog->id;
	biog->io_context = ioc;

	INIT_LIST_HEAD(&biog->page_list);
	spin_lock_init(&biog->page_list_lock);

	/* Bind the cgroup to bio_cgroup object we just created */
	biog->css.cgroup = cgrp;

	return &biog->css;
out:
	if (ioc)
		put_io_context(ioc);
	kfree(biog);
	return ERR_PTR(error);
}

#define FORCE_UNCHARGE_BATCH	(128)
static void bio_cgroup_force_empty(struct bio_cgroup *biog)
{
	struct page_cgroup *pc;
	struct page *page;
	int count = FORCE_UNCHARGE_BATCH;
	struct list_head *list = &biog->page_list;
	unsigned long flags;

	spin_lock_irqsave(&biog->page_list_lock, flags);
	while (!list_empty(list)) {
		pc = list_entry(list->prev, struct page_cgroup, blist);
		page = pc->page;
		get_page(page);
		spin_unlock_irqrestore(&biog->page_list_lock, flags);
		mem_cgroup_uncharge_page(page);
		put_page(page);
		if (--count <= 0) {
			count = FORCE_UNCHARGE_BATCH;
			cond_resched();
		}
		spin_lock_irqsave(&biog->page_list_lock, flags);
	}
	spin_unlock_irqrestore(&biog->page_list_lock, flags);
	return;
}

static void bio_cgroup_pre_destroy(struct cgroup_subsys *ss,
							struct cgroup *cgrp)
{
	struct bio_cgroup *biog = cgroup_bio(cgrp);
	bio_cgroup_force_empty(biog);
}

static void bio_cgroup_destroy(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	struct bio_cgroup *biog = cgroup_bio(cgrp);

	put_io_context(biog->io_context);

	spin_lock_irq(&bio_cgroup_idr_lock);
	idr_remove(&bio_cgroup_id, biog->id);
	spin_unlock_irq(&bio_cgroup_idr_lock);

	kfree(biog);
}

struct bio_cgroup *find_bio_cgroup(int id)
{
	struct bio_cgroup *biog;
	spin_lock_irq(&bio_cgroup_idr_lock);
	biog = (struct bio_cgroup *)
	idr_find(&bio_cgroup_id, id);
	spin_unlock_irq(&bio_cgroup_idr_lock);
	get_bio_cgroup(biog);
	return biog;
}

struct io_context *get_bio_cgroup_iocontext(struct bio *bio)
{
	struct io_context *ioc;
	struct page_cgroup *pc;
	struct bio_cgroup *biog;
	struct page *page = bio_iovec_idx(bio, 0)->bv_page;

	lock_page_cgroup(page);
	pc = page_get_page_cgroup(page);
	if (pc)
		biog = pc->bio_cgroup;
	else
		biog = bio_cgroup_from_task(rcu_dereference(init_mm.owner));
	ioc = biog->io_context;	/* default io_context for this cgroup */
	atomic_inc(&ioc->refcount);
	unlock_page_cgroup(page);
	return ioc;
}
EXPORT_SYMBOL(get_bio_cgroup_iocontext);

static u64 bio_id_read(struct cgroup *cgrp, struct cftype *cft)
{
	struct bio_cgroup *biog = cgroup_bio(cgrp);

	return (u64) biog->id;
}


static struct cftype bio_files[] = {
	{
		.name = "id",
		.read_u64 = bio_id_read,
	},
};

static int bio_cgroup_populate(struct cgroup_subsys *ss, struct cgroup *cont)
{
	if (bio_cgroup_disabled())
		return 0;
	return cgroup_add_files(cont, ss, bio_files, ARRAY_SIZE(bio_files));
}

static void bio_cgroup_move_task(struct cgroup_subsys *ss,
				struct cgroup *cont,
				struct cgroup *old_cont,
				struct task_struct *p)
{
	/* do nothing */
}


struct cgroup_subsys bio_cgroup_subsys = {
	.name           = "bio",
	.subsys_id      = bio_cgroup_subsys_id,
	.create         = bio_cgroup_create,
	.destroy        = bio_cgroup_destroy,
	.pre_destroy	= bio_cgroup_pre_destroy,
	.populate       = bio_cgroup_populate,
	.attach		= bio_cgroup_move_task,
	.early_init	= 0,
};

/*
 * Change the owner of a given page.
 */
void bio_cgroup_recharge(struct page *page, struct mm_struct *mm)
{
	struct page_cgroup *pc;
	struct bio_cgroup *biog;

	if (bio_cgroup_disabled() || !mm)
		return;
	if (PageSwapCache(page) || PageAnon(page))
		return;
	/* Check if the owner should be changed without any lock. */
	pc = page_get_page_cgroup(page);
	if (unlikely(!pc))
		return;
	rcu_read_lock();
	biog = bio_cgroup_from_task(rcu_dereference(mm->owner));
	rcu_read_unlock();
	/*
	 * This won't cause any trouble even when the page_cgroup has been
	 * released since its memory still exists where it was.
	 */
	if (biog == pc->bio_cgroup)
		return;

	/* Re-check if the owner should be changed with the lock. */
	lock_page_cgroup(page);
	pc = page_get_page_cgroup(page);
	if (unlikely(!pc))
		goto out;
	rcu_read_lock();
	biog = mm_get_bio_cgroup(mm);
	rcu_read_unlock();
	if (biog == pc->bio_cgroup) {
		put_bio_cgroup(biog);
		goto out;
	}

	/* Move the page into the bio_cgroup associating with "mm." */
	bio_cgroup_remove_page(pc);
	clear_bio_cgroup(pc);
	set_bio_cgroup(pc, biog);
	bio_cgroup_add_page(pc);
out:
	unlock_page_cgroup(page);
}

