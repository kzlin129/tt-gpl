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
 */
#ifndef LINUX_MEMRLIMITCGROUP_H
#define LINUX_MEMRLIMITCGROUP_H

#ifdef CONFIG_CGROUP_MEMRLIMIT_CTLR

int memrlimit_cgroup_charge_as(struct mm_struct *mm, unsigned long nr_pages);
void memrlimit_cgroup_uncharge_as(struct mm_struct *mm, unsigned long nr_pages);

#else /* !CONFIG_CGROUP_RLIMIT_CTLR */

static inline int
memrlimit_cgroup_charge_as(struct mm_struct *mm, unsigned long nr_pages)
{
	return 0;
}

static inline void
memrlimit_cgroup_uncharge_as(struct mm_struct *mm, unsigned long nr_pages)
{
}

#endif /* CONFIG_CGROUP_RLIMIT_CTLR */


#endif /* LINUX_MEMRLIMITCGROUP_H */
