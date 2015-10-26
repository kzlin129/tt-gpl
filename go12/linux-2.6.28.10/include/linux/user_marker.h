#ifndef _LINUX_USER_MARKER_H
#define _LINUX_USER_MARKER_H

#include <linux/list.h>


#define MAX_USER_MARKER_NAME_LEN	128
#define MAX_USER_MARKER_FORMAT_LEN	128

struct user_marker {
	struct hlist_node hlist;
	char __user *state;
	char name[MAX_USER_MARKER_NAME_LEN];
	char format[MAX_USER_MARKER_FORMAT_LEN];
};

#endif
