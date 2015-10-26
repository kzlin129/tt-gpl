/*
 * tc_cgroup.c - traffic control cgroup subsystem
 *
 */

#include <linux/module.h>
#include <linux/cgroup.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cgroup_tc.h>

struct tc_cgroup {
	struct cgroup_subsys_state css;
	unsigned int classid;
};

struct cgroup_subsys tc_subsys;

static inline struct tc_cgroup *cgroup_to_tc(
		struct cgroup *cgroup)
{
	return container_of(cgroup_subsys_state(cgroup, tc_subsys_id),
			    struct tc_cgroup, css);
}

static unsigned int cgroup_tc_classid(struct task_struct *tsk)
{
	unsigned int tc_classid;

	rcu_read_lock();
	tc_classid = container_of(task_subsys_state(tsk, tc_subsys_id),
					 struct tc_cgroup, css)->classid;
	rcu_read_unlock();
	return tc_classid;
}

void cgroup_tc_set_sock_classid(struct sock *sk)
{
	if (sk)
		sk->sk_cgroup_classid = cgroup_tc_classid(current);
}

static struct cgroup_subsys_state *tc_create(struct cgroup_subsys *ss,
						struct cgroup *cgroup)
{
	struct tc_cgroup *tc_cgroup;

	tc_cgroup = kzalloc(sizeof(*tc_cgroup), GFP_KERNEL);

	if (!tc_cgroup)
		return ERR_PTR(-ENOMEM);

	/* Copy parent's class id if present */
	if (cgroup->parent)
		tc_cgroup->classid = cgroup_to_tc(cgroup->parent)->classid;

	return &tc_cgroup->css;
}

static void tc_destroy(struct cgroup_subsys *ss,
			struct cgroup *cgroup)
{
	kfree(cgroup_to_tc(cgroup));
}

static int tc_write_u64(struct cgroup *cgroup, struct cftype *cft, u64 val)
{
	struct tc_cgroup *tc = cgroup_to_tc(cgroup);

	if (!cgroup_lock_live_group(cgroup))
		return -ENODEV;

	tc->classid = (unsigned int) (val & 0xffffffff);
	cgroup_unlock();
	return 0;
}

static u64 tc_read_u64(struct cgroup *cgroup, struct cftype *cft)
{
	struct tc_cgroup *tc = cgroup_to_tc(cgroup);
	return tc->classid;
}

static struct cftype tc_files[] = {
	{
		.name = "classid",
		.read_u64 = tc_read_u64,
		.write_u64 = tc_write_u64,
	}
};

static int tc_populate(struct cgroup_subsys *ss, struct cgroup *cgroup)
{
	int err;
	err = cgroup_add_files(cgroup, ss, tc_files, ARRAY_SIZE(tc_files));
	return err;
}

struct cgroup_subsys tc_subsys = {
	.name = "tc",
	.create = tc_create,
	.destroy  = tc_destroy,
	.populate = tc_populate,
	.subsys_id = tc_subsys_id,
};
