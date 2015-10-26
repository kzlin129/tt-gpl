#ifndef __LINUX_CGROUP_TC_H
#define __LINUX_CGROUP_TC_H

/* Interface to obtain tasks cgroup identifier. */

#include <linux/cgroup.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#ifdef CONFIG_CGROUP_TC

void cgroup_tc_set_sock_classid(struct sock *sk);

#else

#define cgroup_tc_set_sock_classid(sk)

#endif /* CONFIG_CGROUP_TC */

#endif /* __LINUX_CGROUP_TC_H */
