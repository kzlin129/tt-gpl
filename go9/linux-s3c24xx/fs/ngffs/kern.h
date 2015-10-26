/* fs/ngffs/kern.h
 *
 * NGFFS, next generation flash file system
 * Defines and data structures for kernel space only.
 *
 * Copyright (C) 2004-2007 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FS_NGFFS_KERN_H
#define __FS_NGFFS_KERN_H

#include <linux/list.h>

//
// Defines
//

#define NGFFS_SUPER_MAGIC       0x476d4373

#define NGFFS_GARBAGE_NONE       0
#define NGFFS_GARBAGE_EMPTY      1
#define NGFFS_GARBAGE_ACTIVATE   2
#define NGFFS_GARBAGE_INVALIDATE 3
#define NGFFS_GARBAGE_IGNORE     4
#define NGFFS_GARBAGE_COLLECT    5

//
// Inline functions to convert metadata from ngffs to inode vv.
//

static inline void ngffs_copy_from_meta(struct inode *inode,struct ngffs_metadata *meta)
{
	inode->i_mode=meta->mode;
	inode->i_nlink=meta->link_count;
	inode->i_uid=meta->uid;
	inode->i_gid=meta->gid;
	inode->i_mtime.tv_sec=meta->mtime;
	inode->i_atime.tv_sec=meta->mtime;
	inode->i_ctime.tv_sec=meta->ctime;
	inode->i_mtime.tv_nsec=0;
	inode->i_atime.tv_nsec=0;
	inode->i_ctime.tv_nsec=0;
}

static inline void ngffs_copy_to_meta(struct inode *inode,struct ngffs_metadata *meta)
{
	meta->mode=inode->i_mode;
	meta->link_count=inode->i_nlink;
	meta->uid=inode->i_uid;
	meta->gid=inode->i_gid;
	meta->mtime=inode->i_mtime.tv_sec;
	meta->ctime=inode->i_ctime.tv_sec;
}

// not defined elsewhere, is in lib/string.c
//extern void bcopy(const void * srcp, void * destp, size_t count);

#define bcopy(src,dst,cnt) memcpy(dst,src,cnt)

//
// List data structures
//
struct dir_list_entry {
	struct list_head list;
	struct dir_chunk *chunk;
	__u32 ofs;
};

struct ngffs_data_cache {
	__u32 offset;
	__u32 modification;
};

struct file_list_entry {
	struct list_head list;
	struct file_chunk *chunk;
	__u32 ofs;
	struct ngffs_data_cache *cache;
};

struct ngffs_block {
	struct header_chunk   header;
	__u32                 offset;
	__u32                 size;
	__u32                 freespace;
	__u32                 reserved;       // used when locked, to determine if commit_write may
	// succeed
	__u32                 garbage;        // used for flags during garbage collection
};

struct ngffs_info {
	struct ngffs_block *block_list;
	__u32 max_blocks;             // max blocks in block_list
	__u32 offset;                 // offset from start of flash where flash fs starts
	__u32 blocks;                 // number of erase blocks used for fs
	__u32 erasesize;              // total size in bytes of erase blocks
	__u32 size;                   // total size in bytes of fs
	__u32 max_entry_id;           // highest ino in fs
	struct mtd_info *mtd;
	struct list_head dirs;
	struct list_head files;
	struct semaphore global_lock;	// locks the entire filesystem, but CANT be a spinlock or
					// rw_lock as these are empty placeholders on non SMP systems
	struct super_block *sb;       // superblock
};

#undef NGFFS_PARANOIA

#ifdef NGFFS_PARANOIA
#define CHECK_GLB_LOCK(lock) do { 				  \
		if (sema_count(&lock) > 0) { 		  \
			__backtrace(); 			  \
			printk("\nlock count = %d\n", sema_count(&lock)); \
			panic("ngffs global_lock open!"); \
		}					  \
	} while(0)
#else
#define CHECK_GLB_LOCK(lock) do {} while(0)
#endif

//
// Some convenient shorthands
//

// Get pointer to a certain cache item in a file list entry
static inline struct ngffs_data_cache *NGFFS_CACHE(struct file_list_entry *filele, int index)
{
#ifdef NGFFS_PARANOIA
	if (filele == NULL) {
		__backtrace();
		panic("%s: NULL filele\n", __func__);
	}
	if (filele->cache == NULL) {
		__backtrace();
		panic("%s: NULL filele->cache\n", __func__);
	}
	if (index < 0 || index >= NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE)) {
		__backtrace();
		panic("%s: Invalid cache item %d (max %d)\n", __func__, index, NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE));
	}
#endif /* NGFFS_PARANOIA */
	return &filele->cache[index];
}

// Extract the ngffs info from the superblock struct
static inline struct ngffs_info *NGFFS_INFO(struct super_block *sb)
{
#ifdef NGFFS_PARANOIA
	if (sb == NULL) {
		__backtrace();
		panic("%s: NULL sb\n", __func__);
	}
	if (sb->s_fs_info == NULL) {
		__backtrace();
		panic("%s: NULL sb->s_fs_info\n", __func__);
	}
#endif /* NGFFS_PARANOIA */
	return sb->s_fs_info;
}

// Get pointer to a certain block in the block list
static inline struct ngffs_block *NGFFS_BLOCK(struct ngffs_info *ni, int block)
{
#ifdef NGFFS_PARANOIA
	if (ni == NULL) {
		__backtrace();
		panic("%s: NULL ngffs_info\n", __func__);
	}
	if (ni->block_list == NULL) {
		__backtrace();
		panic("%s: NULL ngffs_info::block_list\n", __func__);
	}
	if (ni->max_blocks == 0 || ni->max_blocks > 1000) {
		__backtrace();
		panic("%s: Insane ngffs_info::max_blocks (%u)\n", __func__, ni->max_blocks);
	}
	if (ni->blocks == 0 || ni->blocks > 1000) {
		__backtrace();
		panic("%s: Insane ngffs_info::blocks (%u)\n", __func__, ni->blocks);
	}
	if (block < 0 || block >= ni->max_blocks || block >= ni->blocks) {
		__backtrace();
		panic("%s: Invalid block %d (max %u/%u)\n", __func__, block, ni->max_blocks, ni->blocks);
	}
#endif /* NGFFS_PARANOIA */
	return &ni->block_list[block];
}

// Shorthands to some inode data, not used anymore due to consistency probs
//#define INODE_DIR(x)    (((struct dir_list_entry *)(x->u.generic_ip))->chunk)
//#define INODE_FILE(x)   (((struct file_list_entry *)(x->u.generic_ip))->chunk)
//#define INODE_FILE_OFS(x) (((struct file_list_entry *)(x->u.generic_ip))->ofs)
//#define INODE_DIR_OFS(x) (((struct dir_list_entry *)(x->u.generic_ip))->ofs)

#endif /* __FS_NGFFS_KERN_H */

/* EOF */
