/*
 * net/sched/cls_cgroup.c	Simple packet classifier which can filter
 * 				packets based on the cgroups they belong to.
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <net/pkt_cls.h>
#include <net/netlink.h>
#include <net/sock.h>

struct cgroup_head {
	struct list_head	flist;		/* Head of filter list */
};

struct cgroup_filter {
	u32			handle;		/* Unique filter handle */
	struct tcf_exts		exts;
	struct tcf_ematch_tree	ematches;
	struct tcf_result	res;
	struct list_head	link;
	u32			mask;
	u32			value;
};

static const struct tcf_ext_map cgroup_ext_map = {
	.action	= TCA_CGROUP_ACT,
	.police	= TCA_CGROUP_POLICE,
};

/* This function is called from the qdisc to classify a particular packet
 * contained in the skb to the appropriate sub-classes. It returns the
 * classid of the target class. This filter will match if the cgroup_classid
 * in the skb matches the value in the filter.
 */
static int cgroup_classify(struct sk_buff *skb, struct tcf_proto *tp,
			  struct tcf_result *res)
{
	struct cgroup_head *head = (struct cgroup_head *)tp->root;
	struct cgroup_filter *f;
	uint32_t cgroup_classid = 0;
	int r;

#ifdef CONFIG_CGROUP_TC
	if (skb->sk)
		cgroup_classid =  skb->sk->sk_cgroup_classid;
#endif

	list_for_each_entry(f, &head->flist, link) {

		if (!tcf_em_tree_match(skb, &f->ematches, NULL))
			continue;

		if ((cgroup_classid & f->mask) == f->value) {
			*res = f->res;
			r = tcf_exts_exec(skb, &f->exts, res);
			if (r < 0)
				continue;
			return r;
		}
	}
	return -1;
}

/* Returns pointer to filter matching the handle passed into the function.*/
static unsigned long cgroup_get(struct tcf_proto *tp, u32 handle)
{
	unsigned long l = 0UL;
	struct cgroup_head *head = (struct cgroup_head *) tp->root;
	struct cgroup_filter *f;

	if (head == NULL)
		return 0UL;

	list_for_each_entry(f, &head->flist, link)
		if (f->handle == handle)
			l = (unsigned long) f;

	return l;
}

/* Does not seem to be used for classifiers. */
static void cgroup_put(struct tcf_proto *tp, unsigned long f)
{
}

/* Initializer function called when tp is created. */
static int cgroup_init(struct tcf_proto *tp)
{
	struct cgroup_head *head = kzalloc(sizeof(*head), GFP_KERNEL);
	if (head == NULL)
		return -ENOBUFS;

	INIT_LIST_HEAD(&head->flist);
	tp->root = head;
	return 0;
}

/* Simple delete function called when filter is deleted */
static inline void cgroup_delete_filter(struct tcf_proto *tp,
				       struct cgroup_filter *f)
{
	tcf_unbind_filter(tp, &f->res);
	tcf_exts_destroy(tp, &f->exts);
	tcf_em_tree_destroy(tp, &f->ematches);
	kfree(f);
}

/* Destroy the entire tp structure.*/
static void cgroup_destroy(struct tcf_proto *tp)
{
	struct cgroup_head *head = (struct cgroup_head *) xchg(&tp->root, NULL);
	struct cgroup_filter *f, *n;

	list_for_each_entry_safe(f, n, &head->flist, link) {
		list_del(&f->link);
		cgroup_delete_filter(tp, f);
	}
	kfree(head);
}

/* Delete one filter entry */
static int cgroup_delete(struct tcf_proto *tp, unsigned long arg)
{
	struct cgroup_head *head = (struct cgroup_head *) tp->root;
	struct cgroup_filter *t, *f = (struct cgroup_filter *) arg;

	list_for_each_entry(t, &head->flist, link)
		if (t == f) {
			tcf_tree_lock(tp);
			list_del(&t->link);
			tcf_tree_unlock(tp);
			cgroup_delete_filter(tp, t);
			return 0;
		}

	return -ENOENT;
}

/* Set the mask and value parameters in the tp structure. */
static inline int cgroup_set_parms(struct tcf_proto *tp,
			unsigned long base,
			struct cgroup_filter *f, struct nlattr **tb)
{
	int err = -EINVAL;

	if (tb[TCA_CGROUP_MASK]) {
		if (nla_len(tb[TCA_CGROUP_MASK]) < sizeof(u32))
			return err;
		f->mask =  nla_get_u32(tb[TCA_CGROUP_MASK]);
	} else
		f->mask = UINT_MAX;

	if (tb[TCA_CGROUP_VALUE]) {
		if (nla_len(tb[TCA_CGROUP_VALUE]) < sizeof(u32))
			return err;
		f->value = nla_get_u32(tb[TCA_CGROUP_VALUE]);
	} else
		return err;

	if (tb[TCA_CGROUP_CLASSID]) {
		if (nla_len(tb[TCA_CGROUP_CLASSID]) < sizeof(u32))
			return err;
		f->res.classid = nla_get_u32(tb[TCA_CGROUP_CLASSID]);
		tcf_bind_filter(tp, &f->res, base);
	} else
		return err;

	return 0;
}

/* Change the mask and value parameters in the current settings. */
static int cgroup_change(struct tcf_proto *tp, unsigned long base, u32 handle,
				struct nlattr **tca, unsigned long *arg)
{
	int err = -EINVAL;
	struct cgroup_head *head = (struct cgroup_head *) tp->root;
	struct nlattr *tb[TCA_CGROUP_MAX + 1];
	struct cgroup_filter *f = (struct cgroup_filter *) *arg;
	struct tcf_exts e;
	struct tcf_ematch_tree t;

	if (tca[TCA_OPTIONS] == NULL)
		return -EINVAL;

	if (nla_parse_nested(tb, TCA_CGROUP_MAX, tca[TCA_OPTIONS], NULL) < 0)
		return -EINVAL;

	err = tcf_exts_validate(tp, tb, tca[TCA_RATE], &e, &cgroup_ext_map);
	if (err < 0)
		return err;

	err = tcf_em_tree_validate(tp, tb[TCA_CGROUP_EMATCHES], &t);
	if (err < 0)
		goto error1;

	if (f != NULL) {
		if (handle && f->handle != handle)
			goto error2;
	} else {
		if (!handle)
			goto error2;
		f = kzalloc(sizeof(*f), GFP_KERNEL);
		if (f == NULL)
			goto error2;
		f->handle = handle;
	}

	err = cgroup_set_parms(tp, base, f, tb);
	if (err < 0)
		goto error3;

	tcf_exts_change(tp, &f->exts, &e);
	tcf_em_tree_change(tp, &f->ematches, &t);

	if (*arg == 0) {
		tcf_tree_lock(tp);
		list_add(&f->link, &head->flist);
		tcf_tree_unlock(tp);
	}

	*arg = (unsigned long)f;
	return 0;

error3:
	if (*arg == 0)
		kfree(f);
error2:
	tcf_em_tree_destroy(tp, &t);
error1:
	tcf_exts_destroy(tp, &e);

	return err;
}

/* Walk the filter list for things like displaying contents.*/
static void cgroup_walk(struct tcf_proto *tp, struct tcf_walker *arg)
{
	struct cgroup_head *head = (struct cgroup_head *) tp->root;
	struct cgroup_filter *f;

	list_for_each_entry(f, &head->flist, link) {
		if (arg->count < arg->skip)
			goto skip;

		if (arg->fn(tp, (unsigned long) f, arg) < 0) {
			arg->stop = 1;
			break;
		}
skip:
		arg->count++;
	}
}

/* Retreive current settings in the filter */
static int cgroup_dump(struct tcf_proto *tp, unsigned long fh,
		      struct sk_buff *skb, struct tcmsg *t)
{
	struct cgroup_filter *f = (struct cgroup_filter *) fh;
	struct nlattr *nest;

	if (f == NULL)
		return skb->len;

	t->tcm_handle = f->handle;

	nest = nla_nest_start(skb, TCA_OPTIONS);
	if (nest == NULL)
		goto nla_put_failure;

	NLA_PUT_U32(skb, TCA_CGROUP_CLASSID, f->res.classid);
	NLA_PUT_U32(skb, TCA_CGROUP_MASK, f->mask);
	NLA_PUT_U32(skb, TCA_CGROUP_VALUE, f->value);

	if (tcf_exts_dump(skb, &f->exts, &cgroup_ext_map) < 0)
		goto nla_put_failure;

#ifdef CONFIG_NET_EMATCH
	if (f->ematches.hdr.nmatches &&
	  tcf_em_tree_dump(skb, &f->ematches, TCA_CGROUP_EMATCHES) < 0)
		goto nla_put_failure;
#endif

	if (tcf_exts_dump_stats(skb, &f->exts, &cgroup_ext_map) < 0)
		goto nla_put_failure;

	nla_nest_end(skb, nest);
	return skb->len;

nla_put_failure:
	nla_nest_cancel(skb, nest);
	return -1;
}

static struct tcf_proto_ops cls_cgroup_ops = {
	.kind		=	"cgroup",
	.classify	=	cgroup_classify,
	.init		=	cgroup_init,
	.destroy	=	cgroup_destroy,
	.get		=	cgroup_get,
	.put		=	cgroup_put,
	.change		=	cgroup_change,
	.delete		=	cgroup_delete,
	.walk		=	cgroup_walk,
	.dump		=	cgroup_dump,
	.owner		=	THIS_MODULE,
};

static int __init init_cgroup(void)
{
	return register_tcf_proto_ops(&cls_cgroup_ops);
}

static void __exit exit_cgroup(void)
{
	unregister_tcf_proto_ops(&cls_cgroup_ops);
}

module_init(init_cgroup)
module_exit(exit_cgroup)
MODULE_LICENSE("GPL");

