/*
 * Copyright (C) 2008 VA Linux Systems Japan K.K.
 * Authors: Hirokazu Takahashi <taka@valinux.co.jp>
 *          Ryo Tsuruta <ryov@valinux.co.jp>
 *
 *  I/O bandwidth control
 *
 * This file is released under the GPL.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/raid/md.h>
#include <linux/rbtree.h>
#include "dm.h"
#include "dm-bio-list.h"
#include "dm-ioband.h"

#define DM_MSG_PREFIX "ioband"
#define POLICY_PARAM_START 6
#define POLICY_PARAM_DELIM "=:,"

static LIST_HEAD(ioband_device_list);
/* to protect ioband_device_list */
static DEFINE_SPINLOCK(ioband_devicelist_lock);

static void suspend_ioband_device(struct ioband_device *, unsigned long, int);
static void resume_ioband_device(struct ioband_device *);
static void ioband_conduct(struct work_struct *);
static void ioband_hold_bio(struct ioband_group *, struct bio *);
static struct bio *ioband_pop_bio(struct ioband_group *);
static int ioband_set_param(struct ioband_group *, char *, char *);
static int ioband_group_attach(struct ioband_group *, int, char *);
static int ioband_group_type_select(struct ioband_group *, char *);

long ioband_debug;	/* just for debugging */

static void do_nothing(void) {}

static int policy_init(struct ioband_device *dp, char *name,
						int argc, char **argv)
{
	struct policy_type *p;
	struct ioband_group *gp;
	unsigned long flags;
	int r;

	for (p = dm_ioband_policy_type; p->p_name; p++) {
		if (!strcmp(name, p->p_name))
			break;
	}
	if (!p->p_name)
		return -EINVAL;

	spin_lock_irqsave(&dp->g_lock, flags);
	if (dp->g_policy == p) {
		/* do nothing if the same policy is already set */
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return 0;
	}

	suspend_ioband_device(dp, flags, 1);
	list_for_each_entry(gp, &dp->g_groups, c_list)
		dp->g_group_dtr(gp);

	/* switch to the new policy */
	dp->g_policy = p;
	r = p->p_policy_init(dp, argc, argv);
	if (!dp->g_hold_bio)
		dp->g_hold_bio = ioband_hold_bio;
	if (!dp->g_pop_bio)
		dp->g_pop_bio = ioband_pop_bio;

	list_for_each_entry(gp, &dp->g_groups, c_list)
		dp->g_group_ctr(gp, NULL);
	resume_ioband_device(dp);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static struct ioband_device *alloc_ioband_device(char *name,
					int io_throttle, int io_limit)

{
	struct ioband_device *dp, *new;
	unsigned long flags;

	new = kzalloc(sizeof(struct ioband_device), GFP_KERNEL);
	if (!new)
		return NULL;

	spin_lock_irqsave(&ioband_devicelist_lock, flags);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		if (!strcmp(dp->g_name, name)) {
			dp->g_ref++;
			spin_unlock_irqrestore(&ioband_devicelist_lock, flags);
			kfree(new);
			return dp;
		}
	}
	spin_unlock_irqrestore(&ioband_devicelist_lock, flags);

	/*
	 * Prepare its own workqueue as generic_make_request() may
	 * potentially block the workqueue when submitting BIOs.
	 */
	new->g_ioband_wq = create_workqueue("kioband");
	if (!new->g_ioband_wq) {
		kfree(new);
		return NULL;
	}

	INIT_DELAYED_WORK(&new->g_conductor, ioband_conduct);
	INIT_LIST_HEAD(&new->g_groups);
	INIT_LIST_HEAD(&new->g_list);
	spin_lock_init(&new->g_lock);
	mutex_init(&new->g_lock_device);
	bio_list_init(&new->g_urgent_bios);
	new->g_io_throttle = io_throttle;
	new->g_io_limit[0] = io_limit;
	new->g_io_limit[1] = io_limit;
	new->g_issued[0] = 0;
	new->g_issued[1] = 0;
	new->g_blocked = 0;
	new->g_ref = 1;
	new->g_flags = 0;
	strlcpy(new->g_name, name, sizeof(new->g_name));
	new->g_policy = NULL;
	new->g_hold_bio = NULL;
	new->g_pop_bio = NULL;
	init_waitqueue_head(&new->g_waitq);
	init_waitqueue_head(&new->g_waitq_suspend);
	init_waitqueue_head(&new->g_waitq_flush);

	spin_lock_irqsave(&ioband_devicelist_lock, flags);
	list_add_tail(&new->g_list, &ioband_device_list);
	spin_unlock_irqrestore(&ioband_devicelist_lock, flags);

	return new;
}

static void release_ioband_device(struct ioband_device *dp)
{
	unsigned long flags;

	spin_lock_irqsave(&ioband_devicelist_lock, flags);
	dp->g_ref--;
	if (dp->g_ref > 0) {
		spin_unlock_irqrestore(&ioband_devicelist_lock, flags);
		return;
	}
	list_del(&dp->g_list);
	spin_unlock_irqrestore(&ioband_devicelist_lock, flags);
	destroy_workqueue(dp->g_ioband_wq);
	kfree(dp);
}

static int is_ioband_device_flushed(struct ioband_device *dp,
						int wait_completion)
{
	struct ioband_group *gp;

	if (wait_completion && dp->g_issued[0] + dp->g_issued[1] > 0)
		return 0;
	if (dp->g_blocked || waitqueue_active(&dp->g_waitq))
		return 0;
	list_for_each_entry(gp, &dp->g_groups, c_list)
		if (waitqueue_active(&gp->c_waitq))
			return 0;
	return 1;
}

static void suspend_ioband_device(struct ioband_device *dp,
				unsigned long flags, int wait_completion)
{
	struct ioband_group *gp;

	/* block incoming bios */
	set_device_suspended(dp);

	/* wake up all blocked processes and go down all ioband groups */
	wake_up_all(&dp->g_waitq);
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (!is_group_down(gp)) {
			set_group_down(gp);
			set_group_need_up(gp);
		}
		wake_up_all(&gp->c_waitq);
	}

	/* flush the already mapped bios */
	spin_unlock_irqrestore(&dp->g_lock, flags);
	queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	flush_workqueue(dp->g_ioband_wq);

	/* wait for all processes to wake up and bios to release */
	spin_lock_irqsave(&dp->g_lock, flags);
	wait_event_lock_irq(dp->g_waitq_flush,
			is_ioband_device_flushed(dp, wait_completion),
			dp->g_lock, do_nothing());
}

static void resume_ioband_device(struct ioband_device *dp)
{
	struct ioband_group *gp;

	/* go up ioband groups */
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (group_need_up(gp)) {
			clear_group_need_up(gp);
			clear_group_down(gp);
		}
	}

	/* accept incoming bios */
	wake_up_all(&dp->g_waitq_suspend);
	clear_device_suspended(dp);
}

static struct ioband_group *ioband_group_find(
					struct ioband_group *head, int id)
{
	struct rb_node *node = head->c_group_root.rb_node;

	while (node) {
		struct ioband_group *p =
			container_of(node, struct ioband_group, c_group_node);

		if (p->c_id == id || id == IOBAND_ID_ANY)
			return p;
		node = (id < p->c_id) ? node->rb_left : node->rb_right;
	}
	return NULL;
}

static void ioband_group_add_node(struct rb_root *root,
						struct ioband_group *gp)
{
	struct rb_node **new = &root->rb_node, *parent = NULL;
	struct ioband_group *p;

	while (*new) {
		p = container_of(*new, struct ioband_group, c_group_node);
		parent = *new;
		new = (gp->c_id < p->c_id) ?
					&(*new)->rb_left : &(*new)->rb_right;
	}

	rb_link_node(&gp->c_group_node, parent, new);
	rb_insert_color(&gp->c_group_node, root);
}

static int ioband_group_init(struct ioband_group *gp,
    struct ioband_group *head, struct ioband_device *dp, int id, char *param)
{
	unsigned long flags;
	int r;

	INIT_LIST_HEAD(&gp->c_list);
	bio_list_init(&gp->c_blocked_bios);
	bio_list_init(&gp->c_prio_bios);
	gp->c_id = id;	/* should be verified */
	gp->c_blocked = 0;
	gp->c_prio_blocked = 0;
	memset(gp->c_stat, 0, sizeof(gp->c_stat));
	init_waitqueue_head(&gp->c_waitq);
	gp->c_flags = 0;
	gp->c_group_root = RB_ROOT;
	gp->c_banddev = dp;

	spin_lock_irqsave(&dp->g_lock, flags);
	if (head && ioband_group_find(head, id)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		DMWARN("ioband_group: id=%d already exists.", id);
		return -EEXIST;
	}

	list_add_tail(&gp->c_list, &dp->g_groups);

	r = dp->g_group_ctr(gp, param);
	if (r) {
		list_del(&gp->c_list);
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return r;
	}

	if (head) {
		ioband_group_add_node(&head->c_group_root, gp);
		gp->c_dev = head->c_dev;
		gp->c_target = head->c_target;
	}

	spin_unlock_irqrestore(&dp->g_lock, flags);

	return 0;
}

static void ioband_group_release(struct ioband_group *head,
						struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;

	list_del(&gp->c_list);
	if (head)
		rb_erase(&gp->c_group_node, &head->c_group_root);
	dp->g_group_dtr(gp);
	kfree(gp);
}

static void ioband_group_destroy_all(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	struct ioband_group *group;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	while ((group = ioband_group_find(gp, IOBAND_ID_ANY)))
		ioband_group_release(gp, group);
	ioband_group_release(NULL, gp);
	spin_unlock_irqrestore(&dp->g_lock, flags);
}

static void ioband_group_stop_all(struct ioband_group *head, int suspend)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *p;
	struct rb_node *node;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	for (node = rb_first(&head->c_group_root); node; node = rb_next(node)) {
		p = rb_entry(node, struct ioband_group, c_group_node);
		set_group_down(p);
		if (suspend) {
			set_group_suspended(p);
			dprintk(KERN_ERR "ioband suspend: gp(%p)\n", p);
		}
	}
	set_group_down(head);
	if (suspend) {
		set_group_suspended(head);
		dprintk(KERN_ERR "ioband suspend: gp(%p)\n", head);
	}
	spin_unlock_irqrestore(&dp->g_lock, flags);
	queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	flush_workqueue(dp->g_ioband_wq);
}

static void ioband_group_resume_all(struct ioband_group *head)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *p;
	struct rb_node *node;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	for (node = rb_first(&head->c_group_root); node;
							node = rb_next(node)) {
		p = rb_entry(node, struct ioband_group, c_group_node);
		clear_group_down(p);
		clear_group_suspended(p);
		dprintk(KERN_ERR "ioband resume: gp(%p)\n", p);
	}
	clear_group_down(head);
	clear_group_suspended(head);
	dprintk(KERN_ERR "ioband resume: gp(%p)\n", head);
	spin_unlock_irqrestore(&dp->g_lock, flags);
}

static int split_string(char *s, long *id, char **v)
{
	char *p, *q;
	int r = 0;

	*id = IOBAND_ID_ANY;
	p = strsep(&s, POLICY_PARAM_DELIM);
	q = strsep(&s, POLICY_PARAM_DELIM);
	if (!q) {
		*v = p;
	} else {
		r = strict_strtol(p, 0, id);
		*v = q;
	}
	return r;
}

/*
 * Create a new band device:
 *   parameters:  <device> <device-group-id> <io_throttle> <io_limit>
 *     <type> <policy> <policy-param...> <group-id:group-param...>
 */
static int ioband_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct ioband_group *gp;
	struct ioband_device *dp;
	struct dm_dev *dev;
	int io_throttle;
	int io_limit;
	int i, r, start;
	long val, id;
	char *param;

	if (argc < POLICY_PARAM_START) {
		ti->error = "Requires " __stringify(POLICY_PARAM_START)
							" or more arguments";
		return -EINVAL;
	}

	if (strlen(argv[1]) > IOBAND_NAME_MAX) {
		ti->error = "Ioband device name is too long";
		return -EINVAL;
	}
	dprintk(KERN_ERR "ioband_ctr ioband device name:%s\n", argv[1]);

	r = strict_strtol(argv[2], 0, &val);
	if (r || val < 0) {
		ti->error = "Invalid io_throttle";
		return -EINVAL;
	}
	io_throttle = (val == 0) ? DEFAULT_IO_THROTTLE : val;

	r = strict_strtol(argv[3], 0, &val);
	if (r || val < 0) {
		ti->error = "Invalid io_limit";
		return -EINVAL;
	}
	io_limit = val;

	r = dm_get_device(ti, argv[0], 0, ti->len,
				dm_table_get_mode(ti->table), &dev);
	if (r) {
		ti->error = "Device lookup failed";
		return r;
	}

	if (io_limit == 0) {
		struct request_queue *q;

		q = bdev_get_queue(dev->bdev);
		if (!q) {
			ti->error = "Can't get queue size";
			r = -ENXIO;
			goto release_dm_device;
		}
		dprintk(KERN_ERR "ioband_ctr nr_requests:%lu\n",
							q->nr_requests);
		io_limit = q->nr_requests;
	}

	if (io_limit < io_throttle)
		io_limit = io_throttle;
	dprintk(KERN_ERR "ioband_ctr io_throttle:%d io_limit:%d\n",
						io_throttle, io_limit);

	dp = alloc_ioband_device(argv[1], io_throttle, io_limit);
	if (!dp) {
		ti->error = "Cannot create ioband device";
		r = -EINVAL;
		goto release_dm_device;
	}

	mutex_lock(&dp->g_lock_device);
	r = policy_init(dp, argv[POLICY_PARAM_START - 1],
			argc - POLICY_PARAM_START, &argv[POLICY_PARAM_START]);
	if (r) {
		ti->error = "Invalid policy parameter";
		goto release_ioband_device;
	}

	gp = kzalloc(sizeof(struct ioband_group), GFP_KERNEL);
	if (!gp) {
		ti->error = "Cannot allocate memory for ioband group";
		r = -ENOMEM;
		goto release_ioband_device;
	}

	ti->private = gp;
	gp->c_target = ti;
	gp->c_dev = dev;

	/* Find a default group parameter */
	for (start = POLICY_PARAM_START; start < argc; start++)
		if (argv[start][0] == ':')
			break;
	param = (start < argc) ? &argv[start][1] : NULL;

	/* Create a default ioband group */
	r = ioband_group_init(gp, NULL, dp, IOBAND_ID_ANY, param);
	if (r) {
		kfree(gp);
		ti->error = "Cannot create default ioband group";
		goto release_ioband_device;
	}

	r = ioband_group_type_select(gp, argv[4]);
	if (r) {
		ti->error = "Cannot set ioband group type";
		goto release_ioband_group;
	}

	/* Create sub ioband groups */
	for (i = start + 1; i < argc; i++) {
		r = split_string(argv[i], &id, &param);
		if (r) {
			ti->error = "Invalid ioband group parameter";
			goto release_ioband_group;
		}
		r = ioband_group_attach(gp, id, param);
		if (r) {
			ti->error = "Cannot create ioband group";
			goto release_ioband_group;
		}
	}
	mutex_unlock(&dp->g_lock_device);
	return 0;

release_ioband_group:
	ioband_group_destroy_all(gp);
release_ioband_device:
	mutex_unlock(&dp->g_lock_device);
	release_ioband_device(dp);
release_dm_device:
	dm_put_device(ti, dev);
	return r;
}

static void ioband_dtr(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;

	mutex_lock(&dp->g_lock_device);
	ioband_group_stop_all(gp, 0);
	cancel_delayed_work_sync(&dp->g_conductor);
	dm_put_device(ti, gp->c_dev);
	ioband_group_destroy_all(gp);
	mutex_unlock(&dp->g_lock_device);
	release_ioband_device(dp);
}

static void ioband_hold_bio(struct ioband_group *gp, struct bio *bio)
{
	/* Todo: The list should be split into a read list and a write list */
	bio_list_add(&gp->c_blocked_bios, bio);
}

static struct bio *ioband_pop_bio(struct ioband_group *gp)
{
	return bio_list_pop(&gp->c_blocked_bios);
}

static int is_urgent_bio(struct bio *bio)
{
	struct page *page = bio_iovec_idx(bio, 0)->bv_page;
	/*
	 * ToDo: A new flag should be added to struct bio, which indicates
	 * 	it contains urgent I/O requests.
	 */
	if (!PageReclaim(page))
		return 0;
	if (PageSwapCache(page))
		return 2;
	return 1;
}

static inline int device_should_block(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;

	if (is_group_down(gp))
		return 0;
	if (is_device_blocked(dp))
		return 1;
	if (dp->g_blocked >= dp->g_io_limit[0] + dp->g_io_limit[1]) {
		set_device_blocked(dp);
		return 1;
	}
	return 0;
}

static inline int group_should_block(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;

	if (is_group_down(gp))
		return 0;
	if (is_group_blocked(gp))
		return 1;
	if (dp->g_should_block(gp)) {
		set_group_blocked(gp);
		return 1;
	}
	return 0;
}

static void prevent_burst_bios(struct ioband_group *gp, struct bio *bio)
{
	struct ioband_device *dp = gp->c_banddev;

	if (current->flags & PF_KTHREAD || is_urgent_bio(bio)) {
		/*
		 * Kernel threads shouldn't be blocked easily since each of
		 * them may handle BIOs for several groups on several
		 * partitions.
		 */
		wait_event_lock_irq(dp->g_waitq, !device_should_block(gp),
						dp->g_lock, do_nothing());
	} else {
		wait_event_lock_irq(gp->c_waitq, !group_should_block(gp),
						dp->g_lock, do_nothing());
	}
}

static inline int should_pushback_bio(struct ioband_group *gp)
{
	return is_group_suspended(gp) && dm_noflush_suspending(gp->c_target);
}

static inline int prepare_to_issue(struct ioband_group *gp, struct bio *bio)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_issued[bio_data_dir(bio)]++;
	return dp->g_prepare_bio(gp, bio, 0);
}

static inline int room_for_bio(struct ioband_device *dp)
{
	return dp->g_issued[0] < dp->g_io_limit[0]
		|| dp->g_issued[1] < dp->g_io_limit[1];
}

static void hold_bio(struct ioband_group *gp, struct bio *bio)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_blocked++;
	if (is_urgent_bio(bio)) {
		/*
		 * ToDo:
		 * When barrier mode is supported, write bios sharing the same
		 * file system with the currnt one would be all moved
		 * to g_urgent_bios list.
		 * You don't have to care about barrier handling if the bio
		 * is for swapping.
		 */
		dp->g_prepare_bio(gp, bio, IOBAND_URGENT);
		bio_list_add(&dp->g_urgent_bios, bio);
	} else {
		gp->c_blocked++;
		dp->g_hold_bio(gp, bio);
	}
}

static inline int room_for_bio_rw(struct ioband_device *dp, int direct)
{
	return dp->g_issued[direct] < dp->g_io_limit[direct];
}

static void push_prio_bio(struct ioband_group *gp, struct bio *bio, int direct)
{
	if (bio_list_empty(&gp->c_prio_bios))
		set_prio_queue(gp, direct);
	bio_list_add(&gp->c_prio_bios, bio);
	gp->c_prio_blocked++;
}

static struct bio *pop_prio_bio(struct ioband_group *gp)
{
	struct bio *bio = bio_list_pop(&gp->c_prio_bios);

	if (bio_list_empty(&gp->c_prio_bios))
		clear_prio_queue(gp);

	if (bio)
		gp->c_prio_blocked--;
	return bio;
}

static int make_issue_list(struct ioband_group *gp, struct bio *bio,
		 struct bio_list *issue_list, struct bio_list *pushback_list)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_blocked--;
	gp->c_blocked--;
	if (!gp->c_blocked && is_group_blocked(gp)) {
		clear_group_blocked(gp);
		wake_up_all(&gp->c_waitq);
	}
	if (should_pushback_bio(gp))
		bio_list_add(pushback_list, bio);
	else {
		int rw = bio_data_dir(bio);

		gp->c_stat[rw].deferred++;
		gp->c_stat[rw].sectors += bio_sectors(bio);
		bio_list_add(issue_list, bio);
	}
	return prepare_to_issue(gp, bio);
}

static void release_urgent_bios(struct ioband_device *dp,
		struct bio_list *issue_list, struct bio_list *pushback_list)
{
	struct bio *bio;

	if (bio_list_empty(&dp->g_urgent_bios))
		return;
	while (room_for_bio_rw(dp, 1)) {
		bio = bio_list_pop(&dp->g_urgent_bios);
		if (!bio)
			return;
		dp->g_blocked--;
		dp->g_issued[bio_data_dir(bio)]++;
		bio_list_add(issue_list, bio);
	}
}

static int release_prio_bios(struct ioband_group *gp,
		struct bio_list *issue_list, struct bio_list *pushback_list)
{
	struct ioband_device *dp = gp->c_banddev;
	struct bio *bio;
	int direct;
	int ret;

	if (bio_list_empty(&gp->c_prio_bios))
		return R_OK;
	direct = prio_queue_direct(gp);
	while (gp->c_prio_blocked) {
		if (!dp->g_can_submit(gp))
			return R_BLOCK;
		if (!room_for_bio_rw(dp, direct))
			return R_OK;
		bio = pop_prio_bio(gp);
		if (!bio)
			return R_OK;
		ret = make_issue_list(gp, bio, issue_list, pushback_list);
		if (ret)
			return ret;
	}
	return R_OK;
}

static int release_norm_bios(struct ioband_group *gp,
		struct bio_list *issue_list, struct bio_list *pushback_list)
{
	struct ioband_device *dp = gp->c_banddev;
	struct bio *bio;
	int direct;
	int ret;

	while (gp->c_blocked - gp->c_prio_blocked) {
		if (!dp->g_can_submit(gp))
			return R_BLOCK;
		if (!room_for_bio(dp))
			return R_OK;
		bio = dp->g_pop_bio(gp);
		if (!bio)
			return R_OK;

		direct = bio_data_dir(bio);
		if (!room_for_bio_rw(dp, direct)) {
			push_prio_bio(gp, bio, direct);
			continue;
		}
		ret = make_issue_list(gp, bio, issue_list, pushback_list);
		if (ret)
			return ret;
	}
	return R_OK;
}

static inline int release_bios(struct ioband_group *gp,
		struct bio_list *issue_list, struct bio_list *pushback_list)
{
	int ret = release_prio_bios(gp, issue_list, pushback_list);
	if (ret)
		return ret;
	return release_norm_bios(gp, issue_list, pushback_list);
}

static struct ioband_group *ioband_group_get(struct ioband_group *head,
							struct bio *bio)
{
	struct ioband_group *gp;

	if (!head->c_type->t_getid)
		return head;

	gp = ioband_group_find(head, head->c_type->t_getid(bio));

	if (!gp)
		gp = head;
	return gp;
}

/*
 * Start to control the bandwidth once the number of uncompleted BIOs
 * exceeds the value of "io_throttle".
 */
static int ioband_map(struct dm_target *ti, struct bio *bio,
						union map_info *map_context)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	unsigned long flags;
	int rw;

	spin_lock_irqsave(&dp->g_lock, flags);

	/*
	 * The device is suspended while some of the ioband device
	 * configurations are being changed.
	 */
	if (is_device_suspended(dp))
		wait_event_lock_irq(dp->g_waitq_suspend,
			!is_device_suspended(dp), dp->g_lock, do_nothing());

	gp = ioband_group_get(gp, bio);
	prevent_burst_bios(gp, bio);
	if (should_pushback_bio(gp)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return DM_MAPIO_REQUEUE;
	}

	bio->bi_bdev = gp->c_dev->bdev;
	bio->bi_sector -= ti->begin;
	rw = bio_data_dir(bio);

	if (!gp->c_blocked && room_for_bio_rw(dp, rw)) {
		if (dp->g_can_submit(gp)) {
			prepare_to_issue(gp, bio);
			gp->c_stat[rw].immediate++;
			gp->c_stat[rw].sectors += bio_sectors(bio);
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return DM_MAPIO_REMAPPED;
		} else if (!dp->g_blocked
				&& dp->g_issued[0] + dp->g_issued[1] == 0) {
			dprintk(KERN_ERR "ioband_map: token expired "
					"gp:%p bio:%p\n", gp, bio);
			queue_delayed_work(dp->g_ioband_wq,
							&dp->g_conductor, 1);
		}
	}
	hold_bio(gp, bio);
	spin_unlock_irqrestore(&dp->g_lock, flags);

	return DM_MAPIO_SUBMITTED;
}

/*
 * Select the best group to resubmit its BIOs.
 */
static struct ioband_group *choose_best_group(struct ioband_device *dp)
{
	struct ioband_group *gp;
	struct ioband_group *best = NULL;
	int	highest = 0;
	int	pri;

	/* Todo: The algorithm should be optimized.
	 *       It would be better to use rbtree.
	 */
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (!gp->c_blocked || !room_for_bio(dp))
			continue;
		if (gp->c_blocked == gp->c_prio_blocked
			&& !room_for_bio_rw(dp, prio_queue_direct(gp))) {
			continue;
		}
		pri = dp->g_can_submit(gp);
		if (pri > highest) {
			highest = pri;
			best = gp;
		}
	}

	return best;
}

/*
 * This function is called right after it becomes able to resubmit BIOs.
 * It selects the best BIOs and passes them to the underlying layer.
 */
static void ioband_conduct(struct work_struct *work)
{
	struct ioband_device *dp =
		container_of(work, struct ioband_device, g_conductor.work);
	struct ioband_group *gp = NULL;
	struct bio *bio;
	unsigned long flags;
	struct bio_list issue_list, pushback_list;

	bio_list_init(&issue_list);
	bio_list_init(&pushback_list);

	spin_lock_irqsave(&dp->g_lock, flags);
	release_urgent_bios(dp, &issue_list, &pushback_list);
	if (dp->g_blocked) {
		gp = choose_best_group(dp);
		if (gp && release_bios(gp, &issue_list, &pushback_list)
								== R_YIELD)
			queue_delayed_work(dp->g_ioband_wq,
							&dp->g_conductor, 0);
	}

	if (is_device_blocked(dp)
	    && dp->g_blocked < dp->g_io_limit[0]+dp->g_io_limit[1]) {
		clear_device_blocked(dp);
		wake_up_all(&dp->g_waitq);
	}

	if (dp->g_blocked && room_for_bio_rw(dp, 0) && room_for_bio_rw(dp, 1) &&
		bio_list_empty(&issue_list) && bio_list_empty(&pushback_list) &&
		dp->g_restart_bios(dp)) {
		dprintk(KERN_ERR "ioband_conduct: token expired dp:%p "
			"issued(%d,%d) g_blocked(%d)\n", dp,
			 dp->g_issued[0], dp->g_issued[1], dp->g_blocked);
		queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	}


	spin_unlock_irqrestore(&dp->g_lock, flags);

	while ((bio = bio_list_pop(&issue_list)))
		generic_make_request(bio);
	while ((bio = bio_list_pop(&pushback_list)))
		bio_endio(bio, -EIO);
}

static int ioband_end_io(struct dm_target *ti, struct bio *bio,
				int error, union map_info *map_context)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	unsigned long flags;
	int r = error;

	/*
	 *  XXX: A new error code for device mapper devices should be used
	 *       rather than EIO.
	 */
	if (error == -EIO && should_pushback_bio(gp)) {
		/* This ioband device is suspending */
		r = DM_ENDIO_REQUEUE;
	}
	/*
	 * Todo: The algorithm should be optimized to eliminate the spinlock.
	 */
	spin_lock_irqsave(&dp->g_lock, flags);
	dp->g_issued[bio_data_dir(bio)]--;

	/*
	 * Todo: It would be better to introduce high/low water marks here
	 * 	 not to kick the workqueues so often.
	 */
	if (dp->g_blocked)
		queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	else if (is_device_suspended(dp)
				&& dp->g_issued[0] + dp->g_issued[1] == 0)
		wake_up_all(&dp->g_waitq_flush);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static void ioband_presuspend(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;

	mutex_lock(&dp->g_lock_device);
	ioband_group_stop_all(gp, 1);
	mutex_unlock(&dp->g_lock_device);
}

static void ioband_resume(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;

	mutex_lock(&dp->g_lock_device);
	ioband_group_resume_all(gp);
	mutex_unlock(&dp->g_lock_device);
}


static void ioband_group_status(struct ioband_group *gp, int *szp,
					char *result, unsigned int maxlen)
{
	struct ioband_group_stat *stat;
	int i, sz = *szp; /* used in DMEMIT() */

	DMEMIT(" %d", gp->c_id);
	for (i = 0; i < 2; i++) {
		stat = &gp->c_stat[i];
		DMEMIT(" %lu %lu %lu",
			stat->immediate + stat->deferred, stat->deferred,
			stat->sectors);
	}
	*szp = sz;
}

static int ioband_status(struct dm_target *ti, status_type_t type,
					char *result, unsigned int maxlen)
{
	struct ioband_group *gp = ti->private, *p;
	struct ioband_device *dp = gp->c_banddev;
	struct rb_node *node;
	int sz = 0;	/* used in DMEMIT() */
	unsigned long flags;

	mutex_lock(&dp->g_lock_device);

	switch (type) {
	case STATUSTYPE_INFO:
		spin_lock_irqsave(&dp->g_lock, flags);
		DMEMIT("%s", dp->g_name);
		ioband_group_status(gp, &sz, result, maxlen);
		for (node = rb_first(&gp->c_group_root); node;
						node = rb_next(node)) {
			p = rb_entry(node, struct ioband_group, c_group_node);
			ioband_group_status(p, &sz, result, maxlen);
		}
		spin_unlock_irqrestore(&dp->g_lock, flags);
		break;

	case STATUSTYPE_TABLE:
		spin_lock_irqsave(&dp->g_lock, flags);
		DMEMIT("%s %s %d %d %s %s",
				gp->c_dev->name, dp->g_name,
				dp->g_io_throttle, dp->g_io_limit[0],
				gp->c_type->t_name, dp->g_policy->p_name);
		dp->g_show(gp, &sz, result, maxlen);
		spin_unlock_irqrestore(&dp->g_lock, flags);
		break;
	}

	mutex_unlock(&dp->g_lock_device);
	return 0;
}

static int ioband_group_type_select(struct ioband_group *gp, char *name)
{
	struct ioband_device *dp = gp->c_banddev;
	struct group_type *t;
	unsigned long flags;

	for (t = dm_ioband_group_type; (t->t_name); t++) {
		if (!strcmp(name, t->t_name))
			break;
	}
	if (!t->t_name) {
		DMWARN("ioband type select: %s isn't supported.", name);
		return -EINVAL;
	}
	spin_lock_irqsave(&dp->g_lock, flags);
	if (!RB_EMPTY_ROOT(&gp->c_group_root)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return -EBUSY;
	}
	gp->c_type = t;
	spin_unlock_irqrestore(&dp->g_lock, flags);

	return 0;
}

static int ioband_set_param(struct ioband_group *gp, char *cmd, char *value)
{
	struct ioband_device *dp = gp->c_banddev;
	char *val_str;
	long id;
	unsigned long flags;
	int r;

	r = split_string(value, &id, &val_str);
	if (r)
		return r;

	spin_lock_irqsave(&dp->g_lock, flags);
	if (id != IOBAND_ID_ANY) {
		gp = ioband_group_find(gp, id);
		if (!gp) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			DMWARN("ioband_set_param: id=%ld not found.", id);
			return -EINVAL;
		}
	}
	r = dp->g_set_param(gp, cmd, val_str);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static int ioband_group_attach(struct ioband_group *gp, int id, char *param)
{
	struct ioband_device *dp = gp->c_banddev;
	struct ioband_group *sub_gp;
	int r;

	if (id < 0) {
		DMWARN("ioband_group_attach: invalid id:%d", id);
		return -EINVAL;
	}
	if (!gp->c_type->t_getid) {
		DMWARN("ioband_group_attach: "
		       "no ioband group type is specified");
		return -EINVAL;
	}

	sub_gp = kzalloc(sizeof(struct ioband_group), GFP_KERNEL);
	if (!sub_gp)
		return -ENOMEM;

	r = ioband_group_init(sub_gp, gp, dp, id, param);
	if (r < 0) {
		kfree(sub_gp);
		return r;
	}
	return 0;
}

static int ioband_group_detach(struct ioband_group *gp, int id)
{
	struct ioband_device *dp = gp->c_banddev;
	struct ioband_group *sub_gp;
	unsigned long flags;

	if (id < 0) {
		DMWARN("ioband_group_detach: invalid id:%d", id);
		return -EINVAL;
	}
	spin_lock_irqsave(&dp->g_lock, flags);
	sub_gp = ioband_group_find(gp, id);
	if (!sub_gp) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		DMWARN("ioband_group_detach: invalid id:%d", id);
		return -EINVAL;
	}

	/*
	 * Todo: Calling suspend_ioband_device() before releasing the
	 *       ioband group has a large overhead. Need improvement.
	 */
	suspend_ioband_device(dp, flags, 0);
	ioband_group_release(gp, sub_gp);
	resume_ioband_device(dp);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return 0;
}

/*
 * Message parameters:
 *	"policy"      <name>
 *       ex)
 *		"policy" "weight"
 *	"type"        "none"|"pid"|"pgrp"|"node"|"cpuset"|"cgroup"|"user"|"gid"
 * 	"io_throttle" <value>
 * 	"io_limit"    <value>
 *	"attach"      <group id>
 *	"detach"      <group id>
 *	"any-command" <group id>:<value>
 *       ex)
 *		"weight" 0:<value>
 *		"token"  24:<value>
 */
static int __ioband_message(struct dm_target *ti,
					unsigned int argc, char **argv)
{
	struct ioband_group *gp = ti->private, *p;
	struct ioband_device *dp = gp->c_banddev;
	struct rb_node *node;
	long val;
	int r = 0;
	unsigned long flags;

	if (argc == 1 && !strcmp(argv[0], "reset")) {
		spin_lock_irqsave(&dp->g_lock, flags);
		memset(gp->c_stat, 0, sizeof(gp->c_stat));
		for (node = rb_first(&gp->c_group_root); node;
						 node = rb_next(node)) {
			p = rb_entry(node, struct ioband_group, c_group_node);
			memset(p->c_stat, 0, sizeof(p->c_stat));
		}
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return 0;
	}

	if (argc != 2) {
		DMWARN("Unrecognised band message received.");
		return -EINVAL;
	}
	if (!strcmp(argv[0], "debug")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r || val < 0)
			return -EINVAL;
		ioband_debug = val;
		return 0;
	} else if (!strcmp(argv[0], "io_throttle")) {
		r = strict_strtol(argv[1], 0, &val);
		spin_lock_irqsave(&dp->g_lock, flags);
		if (r || val < 0 ||
			val > dp->g_io_limit[0] || val > dp->g_io_limit[1]) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return -EINVAL;
		}
		dp->g_io_throttle = (val == 0) ? DEFAULT_IO_THROTTLE : val;
		spin_unlock_irqrestore(&dp->g_lock, flags);
		ioband_set_param(gp, argv[0], argv[1]);
		return 0;
	} else if (!strcmp(argv[0], "io_limit")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r || val < 0)
			return -EINVAL;
		spin_lock_irqsave(&dp->g_lock, flags);
		if (val == 0) {
			struct request_queue *q;

			q = bdev_get_queue(gp->c_dev->bdev);
			if (!q) {
				spin_unlock_irqrestore(&dp->g_lock, flags);
				return -ENXIO;
			}
			val = q->nr_requests;
		}
		if (val < dp->g_io_throttle) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return -EINVAL;
		}
		dp->g_io_limit[0] = dp->g_io_limit[1] = val;
		spin_unlock_irqrestore(&dp->g_lock, flags);
		ioband_set_param(gp, argv[0], argv[1]);
		return 0;
	} else if (!strcmp(argv[0], "type")) {
		return ioband_group_type_select(gp, argv[1]);
	} else if (!strcmp(argv[0], "attach")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r)
			return r;
		return ioband_group_attach(gp, val, NULL);
	} else if (!strcmp(argv[0], "detach")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r)
			return r;
		return ioband_group_detach(gp, val);
	} else if (!strcmp(argv[0], "policy")) {
		r = policy_init(dp, argv[1], 0, &argv[2]);
		return r;
	} else {
		/* message anycommand <group-id>:<value> */
		r = ioband_set_param(gp, argv[0], argv[1]);
		if (r < 0)
			DMWARN("Unrecognised band message received.");
		return r;
	}
	return 0;
}

static int ioband_message(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	int r;

	mutex_lock(&dp->g_lock_device);
	r = __ioband_message(ti, argc, argv);
	mutex_unlock(&dp->g_lock_device);
	return r;
}

static int ioband_merge(struct dm_target *ti, struct bvec_merge_data *bvm,
					struct bio_vec *biovec, int max_size)
{
	struct ioband_group *gp = ti->private;
	struct request_queue *q = bdev_get_queue(gp->c_dev->bdev);

	if (!q->merge_bvec_fn)
		return max_size;

	bvm->bi_bdev = gp->c_dev->bdev;
	bvm->bi_sector -= ti->begin;

	return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

static struct target_type ioband_target = {
	.name	     = "ioband",
	.module      = THIS_MODULE,
	.version     = {1, 8, 0},
	.ctr	     = ioband_ctr,
	.dtr	     = ioband_dtr,
	.map	     = ioband_map,
	.end_io	     = ioband_end_io,
	.presuspend  = ioband_presuspend,
	.resume	     = ioband_resume,
	.status	     = ioband_status,
	.message     = ioband_message,
	.merge       = ioband_merge,
};

static int __init dm_ioband_init(void)
{
	int r;

	r = dm_register_target(&ioband_target);
	if (r < 0) {
		DMERR("register failed %d", r);
		return r;
	}
	return r;
}

static void __exit dm_ioband_exit(void)
{
	int r;

	r = dm_unregister_target(&ioband_target);
	if (r < 0)
		DMERR("unregister failed %d", r);
}

module_init(dm_ioband_init);
module_exit(dm_ioband_exit);

MODULE_DESCRIPTION(DM_NAME " I/O bandwidth control");
MODULE_AUTHOR("Hirokazu Takahashi <taka@valinux.co.jp>, "
	      "Ryo Tsuruta <ryov@valinux.co.jp");
MODULE_LICENSE("GPL");
