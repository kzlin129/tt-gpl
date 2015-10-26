#include <linux/cgroup.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>

#ifndef _LINUX_BIOCONTROL_H
#define _LINUX_BIOCONTROL_H

#ifdef	CONFIG_CGROUP_BIO

struct io_context;
struct block_device;

struct bio_cgroup {
	struct cgroup_subsys_state css;
	int id;
	struct io_context *io_context;	/* default io_context */
/*	struct radix_tree_root io_context_root; per device io_context */
	spinlock_t		page_list_lock;
	struct list_head	page_list;
};

static inline int bio_cgroup_disabled(void)
{
	return bio_cgroup_subsys.disabled;
}

static inline struct bio_cgroup *bio_cgroup_from_task(struct task_struct *p)
{
	return container_of(task_subsys_state(p, bio_cgroup_subsys_id),
				struct bio_cgroup, css);
}

static inline void __bio_cgroup_add_page(struct page_cgroup *pc)
{
	struct bio_cgroup *biog = pc->bio_cgroup;
	list_add(&pc->blist, &biog->page_list);
}

static inline void bio_cgroup_add_page(struct page_cgroup *pc)
{
	struct bio_cgroup *biog = pc->bio_cgroup;
	unsigned long flags;
	spin_lock_irqsave(&biog->page_list_lock, flags);
	__bio_cgroup_add_page(pc);
	spin_unlock_irqrestore(&biog->page_list_lock, flags);
}

static inline void __bio_cgroup_remove_page(struct page_cgroup *pc)
{
	list_del_init(&pc->blist);
}

static inline void bio_cgroup_remove_page(struct page_cgroup *pc)
{
	struct bio_cgroup *biog = pc->bio_cgroup;
	unsigned long flags;
	spin_lock_irqsave(&biog->page_list_lock, flags);
	__bio_cgroup_remove_page(pc);
	spin_unlock_irqrestore(&biog->page_list_lock, flags);
}

static inline void get_bio_cgroup(struct bio_cgroup *biog)
{
	css_get(&biog->css);
}

static inline void put_bio_cgroup(struct bio_cgroup *biog)
{
	css_put(&biog->css);
}

static inline void set_bio_cgroup(struct page_cgroup *pc,
					struct bio_cgroup *biog)
{
	pc->bio_cgroup = biog;
}

static inline void clear_bio_cgroup(struct page_cgroup *pc)
{
	struct bio_cgroup *biog = pc->bio_cgroup;
	pc->bio_cgroup = NULL;
	put_bio_cgroup(biog);
}

static inline struct bio_cgroup *get_bio_page_cgroup(struct page_cgroup *pc)
{
	struct bio_cgroup *biog = pc->bio_cgroup;
	css_get(&biog->css);
	return biog;
}

/* This sould be called in an RCU-protected section. */
static inline struct bio_cgroup *mm_get_bio_cgroup(struct mm_struct *mm)
{
	struct bio_cgroup *biog;
	biog = bio_cgroup_from_task(rcu_dereference(mm->owner));
	get_bio_cgroup(biog);
	return biog;
}

extern struct io_context *get_bio_cgroup_iocontext(struct bio *bio);

extern void bio_cgroup_recharge(struct page *page, struct mm_struct *mm);

#else	/* CONFIG_CGROUP_BIO */

struct bio_cgroup;

static inline int bio_cgroup_disabled(void)
{
	return 1;
}

static inline void bio_cgroup_add_page(struct page_cgroup *pc)
{
}

static inline void bio_cgroup_remove_page(struct page_cgroup *pc)
{
}

static inline void get_bio_cgroup(struct bio_cgroup *biog)
{
}

static inline void put_bio_cgroup(struct bio_cgroup *biog)
{
}

static inline void set_bio_cgroup(struct page_cgroup *pc,
					struct bio_cgroup *biog)
{
}

static inline void clear_bio_cgroup(struct page_cgroup *pc)
{
}

static inline struct bio_cgroup *get_bio_page_cgroup(struct page_cgroup *pc)
{
	return NULL;
}

static inline struct bio_cgroup *mm_get_bio_cgroup(struct mm_struct *mm)
{
	return NULL;
}

static inline int get_bio_cgroup_id(struct page *page)
{
	return 0;
}

static inline struct io_context *get_bio_cgroup_iocontext(struct bio *bio)
{
	return NULL;
}

static inline void bio_cgroup_recharge(struct page *page, struct mm_struct *mm)
{
}

#endif	/* CONFIG_CGROUP_BIO */

#endif /* _LINUX_BIOCONTROL_H */
