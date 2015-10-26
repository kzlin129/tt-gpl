/*
 * File: linux/security/bsdjail.c
 * Author: Serge Hallyn (serue@us.ibm.com)
 * Date: Sep 1, 2004
 *
 * (See Documentation/bsdjail.txt for more information)
 *
 * Copyright (C) 2004 International Business Machines <serue@us.ibm.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 */

#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/namei.h>
#include <linux/nsproxy.h>
#include <linux/proc_fs.h>
#include <linux/in.h>
#include <linux/pagemap.h>
#include <linux/ip.h>
#include <linux/mount.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/seq_file.h>
#include <linux/un.h>
#include <linux/smp_lock.h>
#include <linux/kref.h>
#include <net/sock.h>
#include <net/inet_sock.h>

static int jail_debug = 0;
module_param(jail_debug, uint, 0600);
MODULE_PARM_DESC(jail_debug, "Print bsd jail debugging messages.\n");

#define BSDJAIL_DBG 0
#define BSDJAIL_WARN 1
#define bsdj_debug(how, fmt, arg... ) \
	do { \
		if ( how || jail_debug ) \
			printk(KERN_NOTICE "%s: %s: " fmt, \
				MY_NAME, __FUNCTION__, \
				## arg ); \
	} while ( 0 )

#define MY_NAME "bsdjail"

/*
 * The task structure holding jail information.
 * Taskp->security points to one of these (or is null).
 * There is exactly one jail_struct for each jail.  If >1 process
 * are in the same jail, they share the same jail_struct.
 */
struct jail_struct {
	struct kref		kref;

	/* these are set on writes to /proc/<pid>/attr/exec */
	char *root_pathname; /* char * containing path to use as jail / */
	char *ip_addr_name;  /* char * containing ip addr to use for jail */

	/* these are set when a jail becomes active */
	__u32 realaddr;      /* internal form of ip_addr_name */
	struct dentry *dentry;  /* dentry of fs root */
	struct vfsmount *mnt;   /* vfsmnt of fs root */

	/* Resource limits.  0 = no limit */
	int max_nrtask;		/* maximum number of tasks within this jail. */
	int cur_nrtask;	/* current number of tasks within this jail. */
	long maxtimeslice;      /* max timeslice in ms for procs in this jail */
	long nice;		/* nice level for processes in this jail */
	long max_data, max_memlock;  /* equivalent to RLIMIT_{DATA,MEMLOCK} */
/* values for the jail_flags field */
#define GOT_NETWORK 1    /* if not set, jail can use any valid net address */
#define IN_USE 2	 /* if 0, task is setting up jail, not yet in it */
	char jail_flags;
};

#define in_use(x) (x->jail_flags & IN_USE)
#define set_in_use(x) (x->jail_flags |= IN_USE)

#define got_network(x) (x->jail_flags & GOT_NETWORK)
#define set_got_network(x) (x->jail_flags |= GOT_NETWORK)
#define unset_got_network(x) (x->jail_flags &= ~GOT_NETWORK)

/*
 * structs, defines, and functions to cope with stacking
 */

#define get_task_security(task) (task->security)
#define get_inode_security(inode) (inode->i_security)
#define get_sock_security(sock) (sock->sk_security)
#define get_file_security(file) (file->f_security)
#define get_ipc_security(ipc)	(ipc->security)

#define jail_of(proc) (get_task_security(proc))

static void release_jail(struct kref *kref);

/*
 * disable_jail:  A jail which was in use, but has no references
 * left, is disabled - we free up the mountpoint and dentry, and
 * give up our reference on the module.
 *
 *   don't need to put namespace, it will be done automatically
 *     when the last process in jail is put.
 *   DO need to put the dentry and vfsmount
 */
static void
disable_jail(struct jail_struct *tsec)
{
	dput(tsec->dentry);
	mntput(tsec->mnt);
	module_put(THIS_MODULE);
}


static void free_jail(struct jail_struct *tsec)
{
	if (!tsec)
		return;

	if (tsec->root_pathname)
		kfree(tsec->root_pathname);
	if (tsec->ip_addr_name)
		kfree(tsec->ip_addr_name);
	kfree(tsec);
}

#define set_task_security(task,data) task->security = data
#define set_inode_security(inode,data) inode->i_security = data
#define set_sock_security(sock,data) sock->sk_security = data
#define set_file_security(file,data) file->f_security = data
#define set_ipc_security(ipc,data)   ipc.security = data

/*
 * jail_task_free_security: this is the callback hooked into LSM.
 * If there was no task->security field for bsdjail, do nothing.
 * If there was, but it was never put into use, free the jail.
 * If there was, and the jail is in use, then decrement the usage
 *  count, and disable and free the jail if the usage count hits 0.
 */
static void jail_task_free_security(struct task_struct *task)
{
	struct jail_struct *tsec;

	tsec = get_task_security(task);

	if (!tsec)
		return;

	if (!in_use(tsec)) {
		/*
		 * someone did 'echo -n x > /proc/<pid>/attr/exec' but
		 * then forked before execing.  Nuke the old info.
		 */
		free_jail(tsec);
		set_task_security(task,NULL);
		return;
	}
	tsec->cur_nrtask--;
	/* If this was the last process in the jail, delete the jail */
	kref_put(&tsec->kref, release_jail);
}

static struct jail_struct *
alloc_task_security(struct task_struct *tsk)
{
	struct jail_struct *tsec;
	tsec = kmalloc(sizeof(struct jail_struct), GFP_KERNEL);
	if (!tsec)
		return ERR_PTR(-ENOMEM);
	memset(tsec, 0, sizeof(struct jail_struct));
	set_task_security(tsk, tsec);
	return tsec;
}

static inline int
in_jail(struct task_struct *t)
{
	struct jail_struct *tsec = jail_of(t);

	if (tsec && in_use(tsec))
		return 1;

	return 0;
}

/*
 * If a network address was passed into /proc/<pid>/attr/exec,
 * then process in its jail will only be allowed to bind/listen
 * to that address.
 */
void
setup_netaddress(struct jail_struct *tsec)
{
	unsigned int a,b,c,d;

	unset_got_network(tsec);
	tsec->realaddr = 0;
	if (!tsec->ip_addr_name)
		return;

	if (sscanf(tsec->ip_addr_name,"%u.%u.%u.%u",&a,&b,&c,&d)!=4)
		return;
	if (a>255 || b>255 || c>255 || d>255)
		return;
	tsec->realaddr = htonl((a<<24)|(b<<16)|(c<<8)|d);
	set_got_network(tsec);
	bsdj_debug(BSDJAIL_DBG, "Network set up (%s)\n", tsec->ip_addr_name);
}

/* release_jail:
 * Callback for kref_put to use for releasing a jail when its
 * last user exits.
 */
static void release_jail(struct kref *kref)
{
	struct jail_struct *tsec;

	tsec = container_of(kref,struct jail_struct,kref);
	disable_jail(tsec);
	free_jail(tsec);
}

/*
 * enable_jail:
 * Called when a process is placed into a new jail to handle the
 * actual creation of the jail.
 *   Creates namespace
 *   Sets process root+pwd
 *   Stores the requested ip address
 *   Registers a unique pseudo-proc filesystem for this jail
 */
int enable_jail(struct task_struct *tsk)
{
	struct nameidata nd;
	struct jail_struct *tsec;
	int retval = -EFAULT;

	tsec = jail_of(tsk);
	if (!tsec || !tsec->root_pathname)
		goto out;

	/*
	 * USE_JAIL_NAMESPACE: could be useful, so that future mounts outside
	 * the jail don't affect the jail.  But it's not necessary, and
	 * requires exporting copy_namespace from fs/namespace.c
	 *
	 * Actually, it would also be useful for truly hiding
	 * information about mounts which do not exist in this jail.
	 * "#define USE_JAIL_NAMESPACE" has been replace with
	 * CONFIG_SECURITY_BSDJAIL_USE_NAMESPACE.
	 */
#ifdef CONFIG_SECURITY_BSDJAIL_USE_NAMESPACE
	bsdj_debug(BSDJAIL_DBG, "bsdjail: copying namespace.\n");
	retval = -EPERM;
	if (copy_namespaces(CLONE_NEWNS, tsk))
		goto out;
	bsdj_debug(BSDJAIL_DBG, "bsdjail: copied namespace.\n");
#endif

	/* find our new root directory */
	bsdj_debug(BSDJAIL_DBG, "bsdjail: looking up %s\n", tsec->root_pathname);
	retval = path_lookup(tsec->root_pathname, LOOKUP_FOLLOW | LOOKUP_DIRECTORY, &nd);
	if (retval)
		goto out;

	bsdj_debug(BSDJAIL_DBG, "bsdjail: got %s, setting root to it\n", tsec->root_pathname);

	/* and set the fsroot to it */
	set_fs_root(tsk->fs, &nd.path);
	set_fs_pwd(tsk->fs, &nd.path);

	bsdj_debug(BSDJAIL_DBG, "bsdjail: root has been set.  Have fun.\n");

	/* set up networking */
	if (tsec->ip_addr_name)
		setup_netaddress(tsec);

	tsec->cur_nrtask = 1;
	if (tsec->nice)
		set_user_nice(current, tsec->nice);
	if (tsec->max_data) {
		current->signal->rlim[RLIMIT_DATA].rlim_cur = tsec->max_data;
		current->signal->rlim[RLIMIT_DATA].rlim_max = tsec->max_data;
	}
	if (tsec->max_memlock) {
		current->signal->rlim[RLIMIT_MEMLOCK].rlim_cur = tsec->max_memlock;
		current->signal->rlim[RLIMIT_MEMLOCK].rlim_max = tsec->max_memlock;
	}
	if (tsec->maxtimeslice) {
		current->signal->rlim[RLIMIT_CPU].rlim_cur = tsec->maxtimeslice;
		current->signal->rlim[RLIMIT_CPU].rlim_max = tsec->maxtimeslice;
	}
	/* success and end */
	tsec->mnt = mntget(nd.path.mnt);
	tsec->dentry = dget(nd.path.dentry);
	path_put(&nd.path);
	kref_init(&tsec->kref);
	set_in_use(tsec);

	/* won't let ourselves be removed until this jail goes away */
	try_module_get(THIS_MODULE);

	return 0;

out:
	return retval;
}

/*
 * LSM /proc/<pid>/attr hooks.
 * You may write into /proc/<pid>/attr/exec:
 *    root /some/path
 *    ip 2.2.2.2
 * These values will be used on the next exec() to set up your jail
 *  (assuming you're not already in a jail)
 */
static int
jail_setprocattr(struct task_struct *p, char *name, void *value, size_t size)
{
	struct jail_struct *tsec = jail_of(current);
	long val;
	int start, len;

	if (tsec && in_use(tsec))
		return -EINVAL;  /* let them guess why */

	if (p != current || strcmp(name, "exec"))
		return -EPERM;

	if (strncmp(value, "root ", 5)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		if (tsec->root_pathname)
			kfree(tsec->root_pathname);
		start = 5;
		len = size-start;
		tsec->root_pathname = kmalloc(len+1, GFP_KERNEL);
		if (!tsec->root_pathname)
			return -ENOMEM;
		strncpy(tsec->root_pathname, value+start, len);
		tsec->root_pathname[len] = '\0';
	} else if (strncmp(value, "ip ", 3)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		if (tsec->ip_addr_name)
			kfree(tsec->ip_addr_name);
		start = 3;
		len = size-start;
		tsec->ip_addr_name = kmalloc(len+1, GFP_KERNEL);
		if (!tsec->ip_addr_name)
			return -ENOMEM;
		strncpy(tsec->ip_addr_name, value+start, len);
		tsec->ip_addr_name[len] = '\0';

	/* the next two are equivalent */
	} else if (strncmp(value, "slice ", 6)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = simple_strtoul(value+6, NULL, 0);
		tsec->maxtimeslice = val;
	} else if (strncmp(value, "timeslice ", 10)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = simple_strtoul(value+10, NULL, 0);
		tsec->maxtimeslice = val;
	} else if (strncmp(value, "nrtask ", 7)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = (int) simple_strtol(value+7, NULL, 0);
		if (val < 1)
			return -EINVAL;
		tsec->max_nrtask = val;
	} else if (strncmp(value, "memlock ", 8)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = simple_strtoul(value+8, NULL, 0);
		tsec->max_memlock = val;
	} else if (strncmp(value, "data ", 5)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = simple_strtoul(value+5, NULL, 0);
		tsec->max_data = val;
	} else if (strncmp(value, "nice ", 5)==0) {
		if (!tsec)
			tsec = alloc_task_security(current);
		if (IS_ERR(tsec))
			return -ENOMEM;

		val = simple_strtoul(value+5, NULL, 0);
		tsec->nice = val;
	} else
		return -EINVAL;

	return size;
}

static int print_jail_net_info(struct jail_struct *j, char *buf, int maxcnt)
{
	if (j->ip_addr_name)
		return snprintf(buf, maxcnt, "%s\n", j->ip_addr_name);

	return snprintf(buf, maxcnt, "No network information\n");
}

/*
 * LSM /proc/<pid>/attr read hook.
 *
 * /proc/$$/attr/current output:
 * If the reading process, say process 1001, is in a jail, then
 *   cat /proc/999/attr/current
 * will print networking information.
 * If the reading process, say process 1001, is not in a jail, then
 *   cat /proc/999/attr/current
 * will return
 *   root: (root of jail)
 *   ip:   (ip address of jail)
 * if 999 is in a jail, or
 *   -EINVAL
 * if 999 is not in a jail.
 *
 * /proc/$$/attr/exec output:
 * A process in a jail gets -EINVAL for /proc/$$/attr/exec.
 * A process not in a jail gets hints on starting a jail.
 */
#define JAIL_BUF_SIZE 512
static int
jail_getprocattr(struct task_struct *p, char *name, char **value)
{
	int err = 0;
	struct jail_struct *tsec;
	int size;
	char *cp;

	/* our caller (proc_pid_attr_read) will free this buffer */
	cp = kmalloc(JAIL_BUF_SIZE, GFP_ATOMIC);
	if (cp == NULL)
                return -ENOMEM;
	size = JAIL_BUF_SIZE;

	/* are we currently in jail ? */
	if (in_jail(current)) {
		if (strcmp(name, "current")==0) {
			/* provide network info */
			err = print_jail_net_info(jail_of(current), cp,
				size);
			*value = cp;
			return err;
		}
		*value = cp;
		return -EINVAL;  /* let them guess why */
	}

	/* not in jail */
	if (strcmp(name, "exec") == 0) {
		/* Print usage some help */
		err = snprintf(cp, size,
			"Valid keywords:\n"
			"root    <pathname>\n"
			"ip      <ip4-addr>\n"
			"nrtask  <max number of tasks in this jail>\n"
			"nice    <nice level for processes in this jail>\n"
			"slice   <max timeslice per process in msecs>\n"
			"data    <max data size per process in bytes>\n"
			"memlock <max lockable memory per process in bytes>\n");
		*value = cp;
		return err;
	}

	/* not in jail, and anything but "current" was asked for */
	if (strcmp(name, "current")) {
		*value = cp;
		return -EPERM;
	}

	/* asking about another task .. */
	tsec = jail_of(p);
	if (!tsec || !in_use(tsec)) {
		err = snprintf(cp, size, "Not Jailed\n");
	} else {
		err = snprintf(cp, size,
			"Root: %s\nIP: %s\n"
			"max_nrtask %d current nrtask %d max_timeslice %lu "
			"nice %lu\n"
			"max_memlock %lu max_data %lu\n",
			tsec->root_pathname,
			tsec->ip_addr_name ? tsec->ip_addr_name : "(none)",
			tsec->max_nrtask, tsec->cur_nrtask, tsec->maxtimeslice,
			tsec->nice, tsec->max_data, tsec->max_memlock);
	}

	*value = cp;
	return err;
}

/*
 * Forbid a process in a jail from sending a signal to a process in another
 * (or no) jail through file sigio.
 *
 * We consider the process which set the fowner to be the one sending the
 * signal, rather than the one writing to the file.  Therefore we store the
 * jail of a process during jail_file_set_fowner, then check that against
 * the jail of the process receiving the signal.
 */
static int
jail_file_send_sigiotask(struct task_struct *tsk, struct fown_struct *fown,
       int sig)
{
	struct file *file;
	struct jail_struct *tsec, *fsec;

	if (!in_jail(current))
		return 0;

	file = (struct file *)((long)fown - offsetof(struct file,f_owner));
	tsec = jail_of(tsk);
	fsec = get_file_security(file);

	if (fsec != tsec)
		return -EPERM;

	return 0;
}

static int
jail_file_set_fowner(struct file *file)
{
	struct jail_struct *tsec;

	tsec = jail_of(current);
	set_file_security(file, tsec);
	if (tsec)
		kref_get(&tsec->kref);

	return 0;
}

static void free_ipc_security(struct kern_ipc_perm *ipc)
{
	struct jail_struct *tsec;

	tsec = get_ipc_security(ipc);
	if (!tsec)
		return;
	kref_put(&tsec->kref, release_jail);
	set_ipc_security((*ipc), NULL);
}

static void free_file_security(struct file *file)
{
	struct jail_struct *tsec;

	tsec = get_file_security(file);
	if (!tsec)
		return;
	kref_put(&tsec->kref, release_jail);
	set_file_security(file, NULL);
}

static void free_inode_security(struct inode *inode)
{
	struct jail_struct *tsec;

	tsec = get_inode_security(inode);
	if (!tsec)
		return;
	kref_put(&tsec->kref, release_jail);
	set_inode_security(inode, NULL);
}

/*
 * LSM ptrace hook:
 * process in jail may not ptrace process not in the same jail
 */
/* static int */
/* jail_ptrace (struct task_struct *tracer, struct task_struct *tracee) */
/* { */
/* 	struct jail_struct *tsec = jail_of(tracer); */

/* 	if (tsec && (tsec->jail_flags & IN_USE)) { */
/* 		if (tsec == tracee->security) */
/* 			return 0; */
/* 		return -EPERM; */
/* 	} */
/* 	return 0; */
/* } */

/*
 * LSM ptrace hook:
 * process in jail may not ptrace process not in the same jail
 */
static int
jail_ptrace_may_access (struct task_struct *ctp, unsigned int mode)
{
	struct jail_struct *tsec = ctp->security;
	int rc;

	rc = cap_ptrace_may_access(ctp, mode);
	if (rc != 0)
		return rc;

	if (tsec && in_use(tsec)) {
		if (tsec == jail_of(current))
			return 0;
		return -EPERM;
	}
	return 0;
}


#define loopbackaddr htonl((127 << 24) | 1)

/*
 * process in jail may only use one (aliased) ip address.  If they try to
 * attach to 127.0.0.1, that is remapped to their own address.  If some
 * other address (and not their own), deny permission
 */
static int jail_socket_unix_bind(struct socket *sock, struct sockaddr *address,
		int addrlen);

static int
jail_socket_bind(struct socket *sock, struct sockaddr *address, int addrlen)
{
	struct jail_struct *tsec = jail_of(current);
	struct sockaddr_in *inaddr;
	__u32 sin_addr, jailaddr;

	if (!tsec || !in_use(tsec))
		return 0;

	if (sock->sk->sk_family == AF_UNIX)
		return jail_socket_unix_bind(sock, address, addrlen);

	if (address->sa_family != AF_INET)
		return 0;

	if (!got_network(tsec))
		/* If we want to be strict, we could just
		 * deny net access when lacking a pseudo ip.
		 * For now we just allow it. */
		return 0;

	inaddr = (struct sockaddr_in *)address;
	sin_addr = inaddr->sin_addr.s_addr;
	jailaddr = tsec->realaddr;

	if (sin_addr == jailaddr)
		return 0;

	if (sin_addr == loopbackaddr || !sin_addr) {
		bsdj_debug(BSDJAIL_DBG, "Got a loopback or 0 address\n");
		sin_addr = jailaddr;
		bsdj_debug(BSDJAIL_DBG, "Converted to: %u.%u.%u.%u\n",
			NIPQUAD(sin_addr));
		return 0;
	}

	return -EPERM;
}

static int
jail_socket_post_create(struct socket *sock, int family, int type,
	int protocol, int kern)
{
	int err = 0;
	struct inet_sock *inet;
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec) || kern)
		return err; // XXX
	if (!got_network(tsec))
		return err; // XXX

	if (sock->sk->sk_family != AF_INET)
		return err; // XXX

	inet = (struct inet_sock *)sock->sk;
	inet->saddr = tsec->realaddr;

	return err; // XXX
}

static int
jail_socket_listen(struct socket *sock, int backlog)
{
	struct inet_sock *inet;
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;

	if (!got_network(tsec))
		return 0;

	if (sock->sk->sk_family != AF_INET)
		return 0;

	inet = (struct inet_sock *)sock->sk;

	if (inet->saddr == tsec->realaddr)
		return 0;

	return -EPERM;
}

static void free_sock_security(struct sock *sk)
{
	struct jail_struct *tsec;

	tsec = get_sock_security(sk);
	if (!tsec)
		return;
	kref_put(&tsec->kref, release_jail);
	set_sock_security(sk, NULL);
}

/*
 * The next three (socket) hooks prevent a process in a jail from sending
 * data to a abstract unix domain socket which was bound outside the jail.
 */
static int
jail_socket_unix_bind(struct socket *sock, struct sockaddr *address,
	int addrlen)
{
	struct sockaddr_un *sunaddr;
	struct jail_struct *tsec;

	if (sock->sk->sk_family != AF_UNIX)
		return 0;

	sunaddr = (struct sockaddr_un *)address;
	if (sunaddr->sun_path[0] != 0)
		return 0;

	tsec = jail_of(current);
	set_sock_security(sock->sk, tsec);
	if (tsec)
		kref_get(&tsec->kref);
	return 0;
}

/*
 * Note - we deny sends  both from unjailed to jailed, and from jailed
 * to unjailed.  As well as, of course between different jails.
 */
static int
jail_socket_unix_may_send(struct socket *sock, struct socket *other)
{
	struct jail_struct *tsec, *ssec;

	tsec = jail_of(current);  /* jail of sending process */
	ssec = get_sock_security(other->sk);  /* jail of receiver */

	if (tsec != ssec)
		return -EPERM;

	return 0;
}

static int
jail_socket_unix_stream_connect(struct socket *sock,
	      struct socket *other, struct sock *newsk)
{
	struct jail_struct *tsec, *ssec;

	tsec = jail_of(current);  /* jail of sending process */
	ssec = get_sock_security(other->sk);  /* jail of receiver */

	if (tsec != ssec)
		return -EPERM;

	return 0;
}

static int
jail_mount(char * dev_name, struct path *path, char * type,
	   unsigned long flags, void * data)
{
	if (in_jail(current))
		return -EPERM;

	return 0;
}

static int
jail_umount(struct vfsmount *mnt, int flags)
{
	if (in_jail(current))
		return -EPERM;

	return 0;
}

/*
 * process in jail may not:
 *   use nice
 *   change network config
 *   load/unload modules
 */
static int
jail_capable (struct task_struct *tsk, int cap)
{
	if (in_jail(tsk)) {
		if (cap == CAP_SYS_NICE)
			return -EPERM;
		if (cap == CAP_NET_ADMIN)
			return -EPERM;
		if (cap == CAP_SYS_MODULE)
			return -EPERM;
		if (cap == CAP_SYS_RAWIO)
			return -EPERM;
	}

	if (cap_is_fs_cap (cap) ? tsk->fsuid == 0 : tsk->euid == 0)
		return 0;
	return -EPERM;
}

/*
 * jail_security_task_create:
 *
 * If the current process is ina a jail, and that jail is about to exceed a
 * maximum number of processes, then refuse to fork.  If the maximum number
 * of jails is listed as 0, then there is no limit for this jail, and we allow
 * all forks.
 */
static inline int
jail_security_task_create (unsigned long clone_flags)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;

	if (tsec->max_nrtask && tsec->cur_nrtask >= tsec->max_nrtask)
		return -EPERM;
	return 0;
}

/*
 * The child of a process in a jail belongs in the same jail
 */
static int
jail_task_alloc_security(struct task_struct *tsk)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;

	bsdj_debug(BSDJAIL_DBG,"jail task alloc security\n");
	set_task_security(tsk, tsec);
	kref_get(&tsec->kref);
	tsec->cur_nrtask++;
	if (tsec->maxtimeslice) {
		tsk->signal->rlim[RLIMIT_CPU].rlim_max = tsec->maxtimeslice;
		tsk->signal->rlim[RLIMIT_CPU].rlim_cur = tsec->maxtimeslice;
	}
	if (tsec->max_data) {
		tsk->signal->rlim[RLIMIT_CPU].rlim_max = tsec->max_data;
		tsk->signal->rlim[RLIMIT_CPU].rlim_cur = tsec->max_data;
	}
	if (tsec->max_memlock) {
		tsk->signal->rlim[RLIMIT_CPU].rlim_max = tsec->max_memlock;
		tsk->signal->rlim[RLIMIT_CPU].rlim_cur = tsec->max_memlock;
	}
	if (tsec->nice)
		set_user_nice(current, tsec->nice);

	return 0;
}

static int
jail_bprm_alloc_security(struct linux_binprm *bprm)
{
	struct jail_struct *tsec;
	int ret;

	tsec = jail_of(current);
	if (!tsec)
		return 0;

	if (in_use(tsec))
		return 0;

	if (tsec->root_pathname) {
		ret = enable_jail(current);
		if (ret) {
			/* if we failed, nix out the root/ip requests */
			jail_task_free_security(current);
			return ret;
		}
	}
	return 0;
}

/*
 * Process in jail may not create devices
 * Thanks to Brad Spender for pointing out fifos should be allowed.
 */
/* TODO: We may want to allow /dev/log, at least... */
static int
jail_inode_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	if (!in_jail(current))
		return 0;

	if (S_ISFIFO(mode))
		return 0;

	return -EPERM;
}

/* yanked from fs/proc/base.c */
static unsigned name_to_int(struct dentry *dentry)
{
	const char *name = dentry->d_name.name;
	int len = dentry->d_name.len;
	unsigned n = 0;

	if (len > 1 && *name == '0')
		goto out;
	while (len-- > 0) {
		unsigned c = *name++ - '0';
		if (c > 9)
			goto out;
		if (n >= (~0U-9)/10)
			goto out;
		n *= 10;
		n += c;
	}
	return n;
out:
	return ~0U;
}

/*
 * jail_proc_inode_permission:
 *   called only when current is in a jail, and is trying to reach
 *   /proc/<pid>.  We check whether <pid> is in the same jail as
 *   current.  If not, permission is denied.
 *
 * NOTE:  On the one hand, the task_to_inode(inode)->i_security
 * approach seems cleaner, but on the other, this prevents us
 * from unloading bsdjail for awhile...
 */
static int
jail_proc_inode_permission(struct inode *inode, int mask,
				    struct nameidata *nd)
{
	struct jail_struct *tsec = jail_of(current);
	struct dentry *dentry = nd->path.dentry;
	unsigned pid;

	pid = name_to_int(dentry);
	if (pid == ~0U) {
		struct qstr *dname = &dentry->d_name;
		if (strcmp(dname->name, "scsi")==0 ||
			strcmp(dname->name, "sys")==0 ||
			strcmp(dname->name, "ide")==0)
			return -EPERM;
		return 0;
	}

	if (dentry->d_parent != dentry->d_sb->s_root)
		return 0;
	if (get_inode_security(inode) != tsec)
		return -ENOENT;

	return 0;
}

/*
 * Here is our attempt to prevent chroot escapes.
 */
static int
is_jailroot_parent(struct dentry *candidate, struct dentry *root,
	struct vfsmount *rootmnt)
{
	if (candidate == root)
		return 0;

	/* simple case:  fs->root/.. == candidate */
	if (root->d_parent == candidate)
		return 1;

	/*
	 * now more complicated:  if fs->root is a mounted directory,
	 * then chdir(..) out of fs->root, at follow_dotdot, will follow
	 * the fs->root mount point. So we must check the parent dir of
	 * the fs->root mount point.
	 */
	if (rootmnt->mnt_root == root && rootmnt->mnt_mountpoint!=root) {
		root = rootmnt->mnt_mountpoint;
		rootmnt = rootmnt->mnt_parent;
		return is_jailroot_parent(candidate, root, rootmnt);
	}

	return 0;
}

/*
 * A process in a jail may not see that /proc/<pid> exists for
 * process not in its jail
 * Unfortunately we can't pretend that pid for the starting process
 * is 1, as vserver does.
 */
static int jail_task_lookup(struct task_struct *p)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec)
		return 0;
	if (tsec == jail_of(p))
		return 0;
	return -EPERM;
}
/*
 * security_task_to_inode:
 * Set inode->security = task's jail.
 */
static void jail_task_to_inode(struct task_struct *p, struct inode *inode)
{
	struct jail_struct *tsec = jail_of(p);

	if (!tsec || !in_use(tsec))
		return;
	if (get_inode_security(inode))
		return;
	kref_get(&tsec->kref);
	set_inode_security(inode, tsec);
}

/*
 * inode_permission:
 * If we are trying to look into certain /proc files from in a jail, we
 * 	may deny permission.
 * If we are trying to cd(..), but the cwd is the root of our jail, then
 * permission is denied.
 */
static int
jail_inode_permission(struct inode *inode, int mask )
{
	struct jail_struct *tsec = jail_of(current);
	if (!tsec || !in_use(tsec))
		return 0;

	if (!(mask&MAY_EXEC))
		return 0;
	if (!inode || !S_ISDIR(inode->i_mode))
		return 0;

	if (is_jailroot_parent(d_find_alias(inode), tsec->dentry, tsec->mnt)) {
		bsdj_debug(BSDJAIL_WARN,"Attempt to chdir(..) out of jail!\n"
				"(%s is a subdir of %s)\n",
				tsec->dentry->d_name.name,
				d_find_alias(inode)->d_name.name);
		return -EPERM;
	}
	return 0;
}

/*
 * A function which returns -ENOENT if dentry is the dentry for
 * a /proc/<pid> directory.  It returns 0 otherwise.
 */
static inline int
generic_procpid_check(struct dentry *dentry)
{
	struct jail_struct *jail = jail_of(current);
	unsigned pid = name_to_int(dentry);

	if (!jail || !in_use(jail))
		return 0;
	if (pid == ~0U)
		return 0;
	if (strcmp(dentry->d_sb->s_type->name, "proc")!=0)
		return 0;
	if (dentry->d_parent != dentry->d_sb->s_root)
		return 0;
	if (get_inode_security(dentry->d_inode) != jail)
		return -ENOENT;
	return 0;
}

/*
 * We want getattr to fail on /proc/<pid> to prevent leakage through, for
 * instance, ls -d.
 */
static int
jail_inode_getattr(struct vfsmount *mnt, struct dentry *dentry)
{
	return generic_procpid_check(dentry);
}

/* This probably is not necessary - /proc does not support xattrs? */
static int
jail_inode_getxattr(struct dentry *dentry, const char *name)
{
	return generic_procpid_check(dentry);
}

/* process in jail may not send signal to process not in the same jail */
static int
jail_task_kill(struct task_struct *p, struct siginfo *info, int sig, u32 secid)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;

	if (tsec == jail_of(p))
		return 0;

	if (sig==SIGCHLD)
		return 0;

	return -EPERM;
}

/*
 * LSM hooks to limit jailed process' abilities to muck with resource
 * limits
 */
static int jail_task_setrlimit (unsigned int resource, struct rlimit *new_rlim)
{
	if (!in_jail(current))
		return 0;

	return -EPERM;
}

static int jail_task_setscheduler (struct task_struct *p, int policy,
				    struct sched_param *lp)
{
	if (!in_jail(current))
		return 0;

	return -EPERM;
}

/*
 * LSM hooks to limit IPC access.
 */

static inline int
basic_ipc_security_check(struct kern_ipc_perm *p, struct task_struct *target)
{
	struct jail_struct *tsec = jail_of(target);

	if (!tsec || !in_use(tsec))
		return 0;

	if (get_ipc_security(p) != tsec)
		return -EPERM;

	return 0;
}

static int
jail_ipc_permission(struct kern_ipc_perm *ipcp, short flag)
{
	return basic_ipc_security_check(ipcp, current);
}

static int
jail_shm_alloc_security (struct shmid_kernel *shp)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;
	set_ipc_security(shp->shm_perm, tsec);
	kref_get(&tsec->kref);
	return 0;
}

static void
jail_shm_free_security (struct shmid_kernel *shp)
{
	free_ipc_security(&shp->shm_perm);
}

static int
jail_shm_associate (struct shmid_kernel *shp, int shmflg)
{
	return basic_ipc_security_check(&shp->shm_perm, current);
}

static int
jail_shm_shmctl(struct shmid_kernel *shp, int cmd)
{
	if (cmd == IPC_INFO || cmd == SHM_INFO)
		return 0;

	return basic_ipc_security_check(&shp->shm_perm, current);
}

static int
jail_shm_shmat(struct shmid_kernel *shp, char *shmaddr, int shmflg)
{
	return basic_ipc_security_check(&shp->shm_perm, current);
}

static int
jail_msg_queue_alloc(struct msg_queue *msq)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;
	set_ipc_security(msq->q_perm, tsec);
	kref_get(&tsec->kref);
	return 0;
}

static void
jail_msg_queue_free(struct msg_queue *msq)
{
	free_ipc_security(&msq->q_perm);
}

static int jail_msg_queue_associate(struct msg_queue *msq, int flag)
{
	return basic_ipc_security_check(&msq->q_perm, current);
}

static int
jail_msg_queue_msgctl(struct msg_queue *msq, int cmd)
{
	if (cmd == IPC_INFO || cmd == MSG_INFO)
		return 0;

	return basic_ipc_security_check(&msq->q_perm, current);
}

static int
jail_msg_queue_msgsnd(struct msg_queue *msq, struct msg_msg *msg, int msqflg)
{
	return basic_ipc_security_check(&msq->q_perm, current);
}

static int
jail_msg_queue_msgrcv(struct msg_queue *msq, struct msg_msg *msg,
		struct task_struct *target, long type, int mode)

{
	return basic_ipc_security_check(&msq->q_perm, target);
}

static int
jail_sem_alloc_security(struct sem_array *sma)
{
	struct jail_struct *tsec = jail_of(current);

	if (!tsec || !in_use(tsec))
		return 0;
	set_ipc_security(sma->sem_perm, tsec);
	kref_get(&tsec->kref);
	return 0;
}

static void
jail_sem_free_security(struct sem_array *sma)
{
	free_ipc_security(&sma->sem_perm);
}

static int
jail_sem_associate(struct sem_array *sma, int semflg)
{
	return basic_ipc_security_check(&sma->sem_perm, current);
}

static int
jail_sem_semctl(struct sem_array *sma, int cmd)
{
	if (cmd == IPC_INFO || cmd == SEM_INFO)
		return 0;
	return basic_ipc_security_check(&sma->sem_perm, current);
}

static int
jail_sem_semop(struct sem_array *sma, struct sembuf *sops, unsigned nsops,
	int alter)
{
	return basic_ipc_security_check(&sma->sem_perm, current);
}

static struct security_operations bsdjail_security_ops = {
	.ptrace_may_access =		jail_ptrace_may_access,
	.capable =			jail_capable,

	.task_kill =			jail_task_kill,
	.task_alloc_security =		jail_task_alloc_security,
	.task_free_security =		jail_task_free_security,
	.bprm_alloc_security =		jail_bprm_alloc_security,
	.task_create =			jail_security_task_create,
	.task_to_inode =		jail_task_to_inode,
	.task_lookup =			jail_task_lookup,

	.task_setrlimit =		jail_task_setrlimit,
	.task_setscheduler =		jail_task_setscheduler,

	.setprocattr =                  jail_setprocattr,
	.getprocattr =                  jail_getprocattr,

	.file_set_fowner =		jail_file_set_fowner,
	.file_send_sigiotask =		jail_file_send_sigiotask,
	.file_free_security =		free_file_security,

	.socket_bind =			jail_socket_bind,
	.socket_listen =		jail_socket_listen,
	.socket_post_create =		jail_socket_post_create,
	.unix_stream_connect =		jail_socket_unix_stream_connect,
	.unix_may_send =		jail_socket_unix_may_send,
	.sk_free_security =		free_sock_security,

	.inode_mknod =			jail_inode_mknod,
	.inode_permission =		jail_inode_permission,
	.inode_free_security =		free_inode_security,
	.inode_getattr =		jail_inode_getattr,
	.inode_getxattr =		jail_inode_getxattr,
	.sb_mount =			jail_mount,
	.sb_umount =			jail_umount,

	.ipc_permission =		jail_ipc_permission,
	.shm_alloc_security = 		jail_shm_alloc_security,
	.shm_free_security = 		jail_shm_free_security,
	.shm_associate =		jail_shm_associate,
	.shm_shmctl =			jail_shm_shmctl,
	.shm_shmat =			jail_shm_shmat,

	.msg_queue_alloc_security =	jail_msg_queue_alloc,
	.msg_queue_free_security =	jail_msg_queue_free,
	.msg_queue_associate =		jail_msg_queue_associate,
	.msg_queue_msgctl =		jail_msg_queue_msgctl,
	.msg_queue_msgsnd =		jail_msg_queue_msgsnd,
	.msg_queue_msgrcv =		jail_msg_queue_msgrcv,

	.sem_alloc_security = 		jail_sem_alloc_security,
	.sem_free_security =  		jail_sem_free_security,
	.sem_associate =		jail_sem_associate,
	.sem_semctl =			jail_sem_semctl,
	.sem_semop =			jail_sem_semop,
};

static int __init bsdjail_init (void)
{
	if (register_security (&bsdjail_security_ops)) {
		printk (KERN_INFO
			"Failure registering BSD Jail module with the kernel\n");
		return -EINVAL;
	}
	printk (KERN_INFO "BSD Jail module initialized.\n");

	return 0;
}

static void __exit bsdjail_exit (void)
{
     /* You cannot unload a LSM. This was removed in the 2.6.24
	timeframe */
#if 0
	if (unregister_security (&bsdjail_security_ops)) {
		printk (KERN_INFO "Failure unregistering BSD Jail "
			"module with the kernel\n");
		return;
	}

	printk (KERN_INFO "BSD Jail module removed\n");
#endif
}

security_initcall (bsdjail_init);
module_exit (bsdjail_exit);

MODULE_DESCRIPTION("BSD Jail LSM.");
MODULE_LICENSE("GPL");
