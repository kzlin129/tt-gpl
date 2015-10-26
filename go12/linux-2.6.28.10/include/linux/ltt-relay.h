/*
 * linux/include/linux/ltt-relay.h
 *
 * Copyright (C) 2002, 2003 - Tom Zanussi (zanussi@us.ibm.com), IBM Corp
 * Copyright (C) 1999, 2000, 2001, 2002 - Karim Yaghmour (karim@opersys.com)
 * Copyright (C) 2008 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * CONFIG_RELAY definitions and declarations
 */

#ifndef _LINUX_LTT_RELAY_H
#define _LINUX_LTT_RELAY_H

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/kref.h>
#include <linux/mm.h>

/* Needs a _much_ better name... */
#define FIX_SIZE(x) ((((x) - 1) & PAGE_MASK) + PAGE_SIZE)

/*
 * Tracks changes to rchan/rchan_buf structs
 */
#define LTT_RELAY_CHANNEL_VERSION		8

struct rchan_buf;

struct buf_page {
	struct page *page;
	struct rchan_buf *buf;	/* buffer the page belongs to */
	size_t offset;		/* page offset in the buffer */
	struct list_head list;	/* buffer linked list */
};

/*
 * Per-cpu relay channel buffer
 */
struct rchan_buf {
	struct rchan *chan;		/* associated channel */
	wait_queue_head_t read_wait;	/* reader wait queue */
	struct timer_list timer; 	/* reader wake-up timer */
	struct dentry *dentry;		/* channel file dentry */
	struct kref kref;		/* channel buffer refcount */
	struct list_head pages;		/* list of buffer pages */
	struct buf_page *wpage;		/* current write page (cache) */
	struct buf_page *hpage[2];	/* current subbuf header page (cache) */
	struct buf_page *rpage;		/* current subbuf read page (cache) */
	unsigned int page_count;	/* number of current buffer pages */
	unsigned int finalized;		/* buffer has been finalized */
	unsigned int cpu;		/* this buf's cpu */
} ____cacheline_aligned;

/*
 * Relay channel data structure
 */
struct rchan {
	u32 version;			/* the version of this struct */
	size_t subbuf_size;		/* sub-buffer size */
	size_t n_subbufs;		/* number of sub-buffers per buffer */
	size_t alloc_size;		/* total buffer size allocated */
	struct rchan_callbacks *cb;	/* client callbacks */
	struct kref kref;		/* channel refcount */
	void *private_data;		/* for user-defined data */
	struct rchan_buf *buf[NR_CPUS]; /* per-cpu channel buffers */
	struct list_head list;		/* for channel list */
	struct dentry *parent;		/* parent dentry passed to open */
	int subbuf_size_order;		/* order of sub-buffer size */
	char base_filename[NAME_MAX];	/* saved base filename */
};

/*
 * Relay channel client callbacks
 */
struct rchan_callbacks {
	/*
	 * subbuf_start - called on buffer-switch to a new sub-buffer
	 * @buf: the channel buffer containing the new sub-buffer
	 * @subbuf: the start of the new sub-buffer
	 * @prev_subbuf: the start of the previous sub-buffer
	 * @prev_padding: unused space at the end of previous sub-buffer
	 *
	 * The client should return 1 to continue logging, 0 to stop
	 * logging.
	 *
	 * NOTE: subbuf_start will also be invoked when the buffer is
	 *       created, so that the first sub-buffer can be initialized
	 *       if necessary.  In this case, prev_subbuf will be NULL.
	 *
	 * NOTE: the client can reserve bytes at the beginning of the new
	 *       sub-buffer by calling subbuf_start_reserve() in this callback.
	 */
	int (*subbuf_start) (struct rchan_buf *buf,
			     void *subbuf,
			     void *prev_subbuf,
			     size_t prev_padding);

	/*
	 * create_buf_file - create file to represent a relay channel buffer
	 * @filename: the name of the file to create
	 * @parent: the parent of the file to create
	 * @mode: the mode of the file to create
	 * @buf: the channel buffer
	 *
	 * Called during relay_open(), once for each per-cpu buffer,
	 * to allow the client to create a file to be used to
	 * represent the corresponding channel buffer.  If the file is
	 * created outside of relay, the parent must also exist in
	 * that filesystem.
	 *
	 * The callback should return the dentry of the file created
	 * to represent the relay buffer.
	 *
	 * Setting the is_global outparam to a non-zero value will
	 * cause relay_open() to create a single global buffer rather
	 * than the default set of per-cpu buffers.
	 *
	 * See Documentation/filesystems/relayfs.txt for more info.
	 */
	struct dentry *(*create_buf_file)(const char *filename,
					  struct dentry *parent,
					  int mode,
					  struct rchan_buf *buf);

	/*
	 * remove_buf_file - remove file representing a relay channel buffer
	 * @dentry: the dentry of the file to remove
	 *
	 * Called during relay_close(), once for each per-cpu buffer,
	 * to allow the client to remove a file used to represent a
	 * channel buffer.
	 *
	 * The callback should return 0 if successful, negative if not.
	 */
	int (*remove_buf_file)(struct dentry *dentry);
};

extern int ltt_relay_write(struct rchan_buf *buf, size_t offset,
	const void *src, size_t len);

extern int ltt_relay_read(struct rchan_buf *buf, size_t offset,
	void *dest, size_t len);

extern struct buf_page *ltt_relay_read_get_page(struct rchan_buf *buf,
	size_t offset);

/*
 * Return the address where a given offset is located.
 * Should be used to get the current subbuffer header pointer. Given we know
 * it's never on a page boundary, it's safe to write directly to this address,
 * as long as the write is never bigger than a page size.
 */
extern void *ltt_relay_offset_address(struct rchan_buf *buf,
	size_t offset);

/*
 * CONFIG_LTT_RELAY kernel API, ltt/ltt-relay-alloc.c
 */

struct rchan *ltt_relay_open(const char *base_filename,
			 struct dentry *parent,
			 size_t subbuf_size,
			 size_t n_subbufs,
			 struct rchan_callbacks *cb,
			 void *private_data);
extern void ltt_relay_close(struct rchan *chan);

/*
 * exported ltt_relay file operations, ltt/ltt-relay-alloc.c
 */
extern const struct file_operations ltt_relay_file_operations;

#endif /* _LINUX_LTT_RELAY_H */

