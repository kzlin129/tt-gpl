/*
 * Copyright (C) 2008 VA Linux Systems Japan K.K.
 *
 *  I/O bandwidth control
 *
 * This file is released under the GPL.
 */
#include <linux/bio.h>
#include <linux/biocontrol.h>
#include "dm.h"
#include "dm-bio-list.h"
#include "dm-ioband.h"

/*
 * Any I/O bandwidth can be divided into several bandwidth groups, each of which
 * has its own unique ID. The following functions are called to determine
 * which group a given BIO belongs to and return the ID of the group.
 */

/* ToDo: unsigned long value would be better for group ID */

static int ioband_process_id(struct bio *bio)
{
	/*
	 * This function will work for KVM and Xen.
	 */
	return (int)current->tgid;
}

static int ioband_process_group(struct bio *bio)
{
	return (int)task_pgrp_nr(current);
}

static int ioband_uid(struct bio *bio)
{
	return (int)current_uid();
}

static int ioband_gid(struct bio *bio)
{
	return (int)current_gid();
}

static int ioband_cpuset(struct bio *bio)
{
	return 0;	/* not implemented yet */
}

static int ioband_node(struct bio *bio)
{
	return 0;	/* not implemented yet */
}

static int ioband_cgroup(struct bio *bio)
{
	struct io_context *ioc = get_bio_cgroup_iocontext(bio);
	int id = 0;
	if (ioc) {
		id = ioc->id;
		put_io_context(ioc);
	}
	return id;
}

struct group_type dm_ioband_group_type[] = {
	{"none",   NULL},
	{"pgrp",   ioband_process_group},
	{"pid",    ioband_process_id},
	{"node",   ioband_node},
	{"cpuset", ioband_cpuset},
	{"cgroup", ioband_cgroup},
	{"user",   ioband_uid},
	{"uid",    ioband_uid},
	{"gid",    ioband_gid},
	{NULL,     NULL}
};
