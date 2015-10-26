/*
 * Copyright (C) 2008 VA Linux Systems Japan K.K.
 *
 *  I/O bandwidth control
 *
 * This file is released under the GPL.
 */

#include <linux/version.h>
#include <linux/wait.h>

#define DEFAULT_IO_THROTTLE	4
#define DEFAULT_IO_LIMIT	128
#define IOBAND_NAME_MAX 31
#define IOBAND_ID_ANY (-1)

struct ioband_group;

struct ioband_device {
	struct list_head	g_groups;
	struct delayed_work     g_conductor;
	struct workqueue_struct	*g_ioband_wq;
	struct	bio_list	g_urgent_bios;
	int	g_io_throttle;
	int	g_io_limit[2];
	int	g_issued[2];
	int	g_blocked;
	spinlock_t	g_lock;
	struct mutex	g_lock_device;
	wait_queue_head_t g_waitq;
	wait_queue_head_t g_waitq_suspend;
	wait_queue_head_t g_waitq_flush;

	int	g_ref;
	struct	list_head g_list;
	int	g_flags;
	char	g_name[IOBAND_NAME_MAX + 1];
	struct	policy_type *g_policy;

	/* policy dependent */
	int	(*g_can_submit)(struct ioband_group *);
	int	(*g_prepare_bio)(struct ioband_group *, struct bio *, int);
	int	(*g_restart_bios)(struct ioband_device *);
	void	(*g_hold_bio)(struct ioband_group *, struct bio *);
	struct bio * (*g_pop_bio)(struct ioband_group *);
	int	(*g_group_ctr)(struct ioband_group *, char *);
	void	(*g_group_dtr)(struct ioband_group *);
	int	(*g_set_param)(struct ioband_group *, char *cmd, char *value);
	int	(*g_should_block)(struct ioband_group *);
	void	(*g_show)(struct ioband_group *, int *, char *, unsigned int);

	/* members for weight balancing policy */
	int	g_epoch;
	int	g_weight_total;
		/* the number of tokens which can be used in every epoch */
	int	g_token_bucket;
		/* how many epochs tokens can be carried over */
	int	g_carryover;
		/* how many tokens should be used for one page-sized I/O */
	int	g_token_unit;
		/* the last group which used a token */
	struct ioband_group *g_current;
		/* give another group a chance to be scheduled when the rest
		   of tokens of the current group reaches this mark */
	int	g_yield_mark;
		/* the latest group which used up its tokens */
	struct ioband_group *g_expired;
		/* the group which has the largest number of tokens in the
		   active groups */
	struct ioband_group *g_dominant;
		/* the number of unused tokens in this epoch */
	int	g_token_left;
		/* left-over tokens from the previous epoch */
	int	g_token_extra;
};

struct ioband_group_stat {
	unsigned long	sectors;
	unsigned long	immediate;
	unsigned long	deferred;
};

struct ioband_group {
	struct	list_head c_list;
	struct ioband_device *c_banddev;
	struct dm_dev *c_dev;
	struct dm_target *c_target;
	struct	bio_list c_blocked_bios;
	struct	bio_list c_prio_bios;
	struct	rb_root c_group_root;
	struct  rb_node c_group_node;
	int	c_id;	/* should be unsigned long or unsigned long long */
	char	c_name[IOBAND_NAME_MAX + 1];	/* rfu */
	int	c_blocked;
	int	c_prio_blocked;
	wait_queue_head_t c_waitq;
	int	c_flags;
	struct	ioband_group_stat c_stat[2];	/* hold rd/wr status */
	struct	group_type *c_type;

	/* members for weight balancing policy */
	int	c_weight;
	int	c_my_epoch;
	int	c_token;
	int	c_token_initial;
	int	c_limit;
	int     c_consumed;

	/* rfu */
	/* struct bio_list	c_ordered_tag_bios; */
};

#define IOBAND_URGENT 1

#define DEV_BIO_BLOCKED		1
#define DEV_SUSPENDED		2

#define set_device_blocked(dp)		((dp)->g_flags |= DEV_BIO_BLOCKED)
#define clear_device_blocked(dp)	((dp)->g_flags &= ~DEV_BIO_BLOCKED)
#define is_device_blocked(dp)		((dp)->g_flags & DEV_BIO_BLOCKED)

#define set_device_suspended(dp)	((dp)->g_flags |= DEV_SUSPENDED)
#define clear_device_suspended(dp)	((dp)->g_flags &= ~DEV_SUSPENDED)
#define is_device_suspended(dp)		((dp)->g_flags & DEV_SUSPENDED)

#define IOG_PRIO_BIO_WRITE	1
#define IOG_PRIO_QUEUE		2
#define IOG_BIO_BLOCKED		4
#define IOG_GOING_DOWN		8
#define IOG_SUSPENDED		16
#define IOG_NEED_UP		32

#define R_OK		0
#define R_BLOCK		1
#define R_YIELD		2

#define set_group_blocked(gp)		((gp)->c_flags |= IOG_BIO_BLOCKED)
#define clear_group_blocked(gp)		((gp)->c_flags &= ~IOG_BIO_BLOCKED)
#define is_group_blocked(gp)		((gp)->c_flags & IOG_BIO_BLOCKED)

#define set_group_down(gp)		((gp)->c_flags |= IOG_GOING_DOWN)
#define clear_group_down(gp)		((gp)->c_flags &= ~IOG_GOING_DOWN)
#define is_group_down(gp)		((gp)->c_flags & IOG_GOING_DOWN)

#define set_group_suspended(gp)		((gp)->c_flags |= IOG_SUSPENDED)
#define clear_group_suspended(gp)	((gp)->c_flags &= ~IOG_SUSPENDED)
#define is_group_suspended(gp)		((gp)->c_flags & IOG_SUSPENDED)

#define set_group_need_up(gp)		((gp)->c_flags |= IOG_NEED_UP)
#define clear_group_need_up(gp)		((gp)->c_flags &= ~IOG_NEED_UP)
#define group_need_up(gp)		((gp)->c_flags & IOG_NEED_UP)

#define set_prio_read(gp)		((gp)->c_flags |= IOG_PRIO_QUEUE)
#define clear_prio_read(gp)		((gp)->c_flags &= ~IOG_PRIO_QUEUE)
#define is_prio_read(gp) \
	((gp)->c_flags & (IOG_PRIO_QUEUE|IOG_PRIO_BIO_WRITE) == IOG_PRIO_QUEUE)

#define set_prio_write(gp) \
	((gp)->c_flags |= (IOG_PRIO_QUEUE|IOG_PRIO_BIO_WRITE))
#define clear_prio_write(gp) \
	((gp)->c_flags &= ~(IOG_PRIO_QUEUE|IOG_PRIO_BIO_WRITE))
#define is_prio_write(gp) \
	((gp)->c_flags & (IOG_PRIO_QUEUE|IOG_PRIO_BIO_WRITE) == \
		(IOG_PRIO_QUEUE|IOG_PRIO_BIO_WRITE))

#define set_prio_queue(gp, direct) \
	((gp)->c_flags |= (IOG_PRIO_QUEUE|direct))
#define clear_prio_queue(gp)		clear_prio_write(gp)
#define is_prio_queue(gp)		((gp)->c_flags & IOG_PRIO_QUEUE)
#define prio_queue_direct(gp)		((gp)->c_flags & IOG_PRIO_BIO_WRITE)


struct policy_type {
	const char *p_name;
	int	  (*p_policy_init)(struct ioband_device *, int, char **);
};

extern struct policy_type dm_ioband_policy_type[];

struct group_type {
	const char *t_name;
	int	  (*t_getid)(struct bio *);
};

extern struct group_type dm_ioband_group_type[];

/* Just for debugging */
extern long ioband_debug;
#define dprintk(format, a...) \
	if (ioband_debug > 0) ioband_debug--, printk(format, ##a)
