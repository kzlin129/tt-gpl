/* memcontrol.h - Memory Controller
 *
 * Copyright IBM Corporation, 2007
 * Author Balbir Singh <balbir@linux.vnet.ibm.com>
 *
 * Copyright 2007 OpenVZ SWsoft Inc
 * Author: Pavel Emelianov <xemul@openvz.org>
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

#ifndef _LINUX_MEMCONTROL_H
#define _LINUX_MEMCONTROL_H

#include <linux/rcupdate.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/bit_spinlock.h>

struct mem_cgroup;
struct page_cgroup;
struct page;
struct mm_struct;

#ifdef CONFIG_CGROUP_PAGE
/*
 * We use the lower bit of the page->page_cgroup pointer as a bit spin
 * lock.  We need to ensure that page->page_cgroup is at least two
 * byte aligned (based on comments from Nick Piggin).  But since
 * bit_spin_lock doesn't actually set that lock bit in a non-debug
 * uniprocessor kernel, we should avoid setting it here too.
 */
#define PAGE_CGROUP_LOCK_BIT    0x0
#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK) || defined(CONFIG_PREEMPT_RT)
#define PAGE_CGROUP_LOCK        (1 << PAGE_CGROUP_LOCK_BIT)
#else
#define PAGE_CGROUP_LOCK        0x0
#endif

/*
 * A page_cgroup page is associated with every page descriptor. The
 * page_cgroup helps us identify information about the cgroup
 */
struct page_cgroup {
#ifdef CONFIG_CGROUP_MEM_RES_CTLR
	struct list_head lru;		/* per cgroup LRU list */
	struct mem_cgroup *mem_cgroup;
#endif /* CONFIG_CGROUP_MEM_RES_CTLR */
#ifdef CONFIG_CGROUP_BIO
	struct list_head blist;		/* for bio_cgroup page list */
	struct bio_cgroup *bio_cgroup;
#endif
	struct page *page;
	int flags;
};
#define PAGE_CGROUP_FLAG_CACHE	(0x1)	/* charged as cache */
#define PAGE_CGROUP_FLAG_ACTIVE	(0x2)	/* page is active in this cgroup */
#define PAGE_CGROUP_FLAG_FILE	(0x4)	/* page is file system backed */
#define PAGE_CGROUP_FLAG_UNEVICTABLE (0x8)	/* page is unevictableable */

static inline void lock_page_cgroup(struct page *page)
{
	bit_spin_lock(PAGE_CGROUP_LOCK_BIT, &page->page_cgroup);
}

static inline int try_lock_page_cgroup(struct page *page)
{
	return bit_spin_trylock(PAGE_CGROUP_LOCK_BIT, &page->page_cgroup);
}

static inline void unlock_page_cgroup(struct page *page)
{
	bit_spin_unlock(PAGE_CGROUP_LOCK_BIT, &page->page_cgroup);
}

extern int mem_cgroup_charge(struct page *page, struct mm_struct *mm,
				gfp_t gfp_mask);
extern int mem_cgroup_cache_charge(struct page *page, struct mm_struct *mm,
					gfp_t gfp_mask);
extern void mem_cgroup_move_lists(struct page *page, enum lru_list lru);
extern void mem_cgroup_uncharge_page(struct page *page);
extern void mem_cgroup_uncharge_cache_page(struct page *page);

extern int
mem_cgroup_prepare_migration(struct page *page, struct page *newpage);
extern void mem_cgroup_end_migration(struct page *page);
extern void page_cgroup_init(void);

#else /* CONFIG_CGROUP_PAGE */

static inline int mem_cgroup_charge(struct page *page,
					struct mm_struct *mm, gfp_t gfp_mask)
{
	return 0;
}

static inline int mem_cgroup_cache_charge(struct page *page,
					struct mm_struct *mm, gfp_t gfp_mask)
{
	return 0;
}

static inline void mem_cgroup_uncharge_page(struct page *page)
{
}

static inline void mem_cgroup_uncharge_cache_page(struct page *page)
{
}

static inline int
mem_cgroup_prepare_migration(struct page *page, struct page *newpage)
{
	return 0;
}

static inline void mem_cgroup_end_migration(struct page *page)
{
}
#endif /* CONFIG_CGROUP_PAGE */

#ifdef CONFIG_CGROUP_MEM_RES_CTLR

void put_cgroup_from_page(struct page *page);
struct cgroup *get_cgroup_from_page(struct page *page);

extern void mem_cgroup_move_lists(struct page *page,  bool active);
extern int mem_cgroup_shrink_usage(struct mm_struct *mm, gfp_t gfp_mask);

unsigned long mem_cgroup_isolate_pages(unsigned long nr_to_scan,
					struct list_head *dst,
					unsigned long *scanned, int order,
					int mode, struct zone *z,
					struct mem_cgroup *mem_cont,
					int active);
extern void mem_cgroup_out_of_memory(struct mem_cgroup *mem, gfp_t gfp_mask);
int task_in_mem_cgroup(struct task_struct *task, const struct mem_cgroup *mem);

extern struct mem_cgroup *mem_cgroup_from_task(struct task_struct *p);

#define mm_match_cgroup(mm, cgroup)	\
	((cgroup) == mem_cgroup_from_task((mm)->owner))

/*
 * For memory reclaim.
 */
extern int mem_cgroup_calc_mapped_ratio(struct mem_cgroup *mem);
extern long mem_cgroup_reclaim_imbalance(struct mem_cgroup *mem);

extern int mem_cgroup_get_reclaim_priority(struct mem_cgroup *mem);
extern void mem_cgroup_note_reclaim_priority(struct mem_cgroup *mem,
							int priority);
extern void mem_cgroup_record_reclaim_priority(struct mem_cgroup *mem,
							int priority);
extern long mem_cgroup_calc_reclaim_active(struct mem_cgroup *mem,
				   struct zone *zone, int priority);

extern long mem_cgroup_calc_reclaim_inactive(struct mem_cgroup *mem,
				struct zone *zone, int priority);

#else /* CONFIG_CGROUP_MEM_RES_CTLR */

static inline int mem_cgroup_shrink_usage(struct mm_struct *mm, gfp_t gfp_mask)
{
	return 0;
}

static inline void mem_cgroup_move_lists(struct page *page, bool active)
{
}

static inline int mm_match_cgroup(struct mm_struct *mm, struct mem_cgroup *mem)
{
	return 1;
}

static inline int task_in_mem_cgroup(struct task_struct *task,
				     const struct mem_cgroup *mem)
{
	return 1;
}

static inline int mem_cgroup_calc_mapped_ratio(struct mem_cgroup *mem)
{
	return 0;
}

static inline int mem_cgroup_reclaim_imbalance(struct mem_cgroup *mem)
{
	return 0;
}

static inline int mem_cgroup_get_reclaim_priority(struct mem_cgroup *mem)
{
	return 0;
}

static inline void mem_cgroup_note_reclaim_priority(struct mem_cgroup *mem,
						int priority)
{
}

static inline void mem_cgroup_record_reclaim_priority(struct mem_cgroup *mem,
						int priority)
{
}

static inline long mem_cgroup_calc_reclaim(struct mem_cgroup *mem,
					struct zone *zone, int priority,
					enum lru_list lru)
{
	return 0;
}

#endif /* CONFIG_CGROUP_MEM_RES_CTLR */

#endif /* _LINUX_MEMCONTROL_H */

