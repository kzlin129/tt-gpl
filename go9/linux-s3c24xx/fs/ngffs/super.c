/* fs/ngffs/super.c
 *
 * NGFFS, next generation flash file system
 *
 * Copyright (C) 2004-2007 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/statfs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/blkdev.h>
#include <linux/ctype.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/mtd/mtd.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include <asm/semaphore.h>
#include "kern.h"
#include "crc.h"
#include "flash.h"
#include "chunk.h"
#include "garbage.h"

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

extern void ngffs_read_inode(struct inode *);
extern void ngffs_clear_inode(struct inode *);

static int ngffs_statfs(struct super_block *sb, struct kstatfs *buf);
int ngffs_remount_fs(struct super_block *, int *, char *);
void ngffs_put_super(struct super_block *sb);
void ngffs_write_super(struct super_block *sb);

static struct super_operations ngffs_super_operations =
{
	.read_inode   = ngffs_read_inode,
//	.delete_inode = ngffs_delete_inode,
	.put_super    = ngffs_put_super,
	.write_super  = ngffs_write_super,
	.statfs       = ngffs_statfs,
	.remount_fs   = ngffs_remount_fs,
	.clear_inode  = ngffs_clear_inode,
};

#define NGFFS_PROC_DIR_NAME "fs/ngffs"
static struct proc_dir_entry *ngffs_dir = NULL;

static int ngffs_statfs(struct super_block *sb, struct kstatfs *buf)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	__u32 available=0;
	int i;

	PK_DBG("ngffs_statfs(), numblocks=%d\n",ngsb->blocks);

	buf->f_type = NGFFS_SUPER_MAGIC;
	buf->f_bsize = 1 << PAGE_SHIFT;
	buf->f_blocks = ngsb->size >> PAGE_SHIFT;
	buf->f_files = 0;
	buf->f_ffree = 0;
	buf->f_namelen = NGFFS_MAX_NAME_LEN;

	// obtain fs-wide read lock
	down(&ngsb->global_lock);

	// sum free space in available blocks, TODO: get _real_ free space
	// by counting garbage (see garbage.c)
	for(i=0;i<ngsb->blocks;i++) {
		//    if((NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_WRITTEN) &&
		if( IS_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header)) &&
		    (NGFFS_BLOCK(ngsb,i)->header.activated==0x00) ) {
			available += ngffs_block_garbage(ngsb,i) +
				     NGFFS_BLOCK(ngsb,i)->freespace;
		} else {
			available += NGFFS_BLOCK(ngsb,i)->size;
		}
	}

	// release lock
	up( &ngsb->global_lock );

#if 0
	// reserve one block
	available -= NGFFS_BLOCK(ngsb,i)->size;
#endif

	buf->f_bavail = buf->f_bfree = available >> PAGE_SHIFT; // available _blocks_

	return 0;
}

int ngffs_remount_fs (struct super_block *sb, int *flags, char *data)
{
	PK_DBG("ngffs_remount_fs()\n");
	return 0;
}

void ngffs_put_super (struct super_block *sb)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	PK_DBG("ngffs_put_super()\n");
	down(&ngsb->global_lock); 
	ngffs_cleanup(sb);
  	up(&ngsb->global_lock);
}

void ngffs_write_super (struct super_block *sb)
{
	PK_DBG("ngffs_write_super()\n");
	// write _what_ ?
}

static int ngffs_sb_compare(struct super_block *sb, void *data)
{
	struct ngffs_info *p=data;
	struct ngffs_info *n=NGFFS_INFO(sb);

	/* The superblocks are considered to be equivalent if the underlying MTD
	device is the same one */
	if (n->mtd == p->mtd) {
		PK_NOTICE("match on device %d (\"%s\")\n", p->mtd->index, p->mtd->name);
		return 1;
	} else {
		PK_NOTICE("No match, device %d (\"%s\"), device %d (\"%s\")\n",
			  n->mtd->index, n->mtd->name, p->mtd->index, p->mtd->name);
		return 0;
	}
}

static int ngffs_sb_set(struct super_block *sb, void *data)
{
	struct ngffs_info *n = data;

	/* For persistence of NFS exports etc. we use the same s_dev
	each time we mount the device, don't just use an anonymous
	device */
	sb->s_fs_info = n;
	sb->s_dev = MKDEV(MTD_BLOCK_MAJOR, n->mtd->index);

	return 0;
}

static int ngffs_read_proc(char *page, char **start, off_t off,
                           int count, int *eof, void *data)
{
	struct ngffs_info      *ngsb = NGFFS_INFO((struct super_block *)data);
	struct list_head       *curr;
	struct dir_list_entry  *dirle;
	struct file_list_entry *filele;
	unsigned int           index;

	int len = 0;
	
	len += sprintf(page + len, "Dumping directories\n\n");

	list_for_each(curr, &ngsb->dirs) {
		dirle = list_entry(curr, struct dir_list_entry, list);

		if (!dirle)
			break;
		if (!dirle->chunk)
			break;

		len += sprintf(page + len, "id: %04x ofs: %08x valid: %08x len %d namelen %d\n", 
					dirle->chunk->entry_id,
					dirle->ofs,
					dirle->chunk->valid,
					dirle->chunk->len,
					dirle->chunk->namelen);

	}

	len += sprintf(page + len, "\n\nDumping files\n\n");

	list_for_each(curr, &ngsb->files) {
		filele = list_entry(curr, struct file_list_entry, list);

		if (!filele)
			break;
		if (!filele->chunk)
			break;

		len += sprintf(page + len, "id: %04x ofs: %08x valid: %08x len %d namelen %d", 
					filele->chunk->entry_id,
					filele->ofs,
					filele->chunk->valid,
					filele->chunk->len,
					filele->chunk->namelen);


		for (index=0; index<NGFFS_NUMCHUNKS(filele->chunk->len); index++) {
			if (NGFFS_CACHE(filele, index)) {
				len += sprintf(page + len, " (ofs: %08x, mod %d)", 
						NGFFS_CACHE(filele, index)->offset,
						NGFFS_CACHE(filele, index)->modification);
			}
		}
		len += sprintf(page + len, "\n"); 
	}


	*eof = 1;
	return len;
}

static struct super_block *ngffs_get_sb_mtd(struct file_system_type *fs_type,
					    int flags, const char *dev_name,
					    void *data, struct mtd_info *mtd)
{
	struct super_block *sb;
	struct ngffs_info *ngsb;
	struct inode *root_i;
	int ret;

	PK_DBG("ngffs_get_sb_mtd()\n");

	// allocate the ngffs superblock info struct
	ngsb=kmalloc(sizeof(struct ngffs_info),GFP_KERNEL);
	if(ngsb==NULL) {
		PK_ERR("ngffs: not enough memory for ngffs_info\n");
		return ERR_PTR(-ENOMEM);
	}

	// initialize the global rw lock
	init_MUTEX(&ngsb->global_lock);

	// initialize the various values & pointers
	ngsb->offset=CONFIG_NGFFS_START_OFS;
	ngsb->erasesize=mtd->erasesize;
	ngsb->size=CONFIG_NGFFS_BLOCKS * 0x10000; //was: mtd->size - CONFIG_NGFFS_START_OFS; // was: mtd->erasesize * CONFIG_NGFFS_BLOCKS;
	ngsb->blocks=ngsb->size/ngsb->erasesize; //CONFIG_NGFFS_BLOCKS;
	ngsb->mtd=mtd;
	ngsb->max_entry_id=99;	// will be updated by ngffs_scan_flash()

	PK_WARN("size=0x%x\n", ngsb->size);
	if(mtd->erasesize != CONFIG_NGFFS_ERASE_SIZE) {
		PK_WARN("Warning! CONFIG_NGFFS_ERASE_SIZE (%u) != mtd->erasesize (%u)\n",CONFIG_NGFFS_ERASE_SIZE,mtd->erasesize);
	}

	// Initialize the dir & file lists
	INIT_LIST_HEAD(&ngsb->dirs);
	INIT_LIST_HEAD(&ngsb->files);

	down(&ngsb->global_lock);

	// initialize the block lists, array of ngffs_block strucs
	ngsb->max_blocks=mtd->size/mtd->erasesize;
	ngsb->block_list=kmalloc(sizeof(struct ngffs_block)*ngsb->max_blocks,GFP_KERNEL);
	if(ngsb->block_list==NULL) {
		PK_ERR("ngffs: not enough memory for block list\n");
		up(&ngsb->global_lock);
		return ERR_PTR(-ENOMEM);
	}
#ifdef NGFFS_PARANOIA
	memset(ngsb->block_list, 0xde, sizeof(struct ngffs_block)*ngsb->max_blocks);
#endif // NGFFS_PARANOIA

	// get superblock

	PK_DBG("get superblock\n");

	sb = sget(fs_type, ngffs_sb_compare, ngffs_sb_set, ngsb);

	ngsb->sb=sb;

	PK_DBG("got superblock at 0x%p (=0x%p)\n",sb,ngsb->sb);

	// now fill the file&dir lists and the block list
	// and set the max_entry_id, all done by ngffs_scan_flash()
	PK_DBG("scanning flash\n");
	if((ret = ngffs_scan_flash(ngsb)) < 0) {
		PK_ERR("ngffs: error scanning flash sectors (%d)\n", ret);
		up(&ngsb->global_lock);
		return ERR_PTR(ret);
	}

	if (IS_ERR(sb))
		goto out_put;

	if(sb->s_root) {
		PK_DBG("ngffs_get_sb_mtd(): Device %d (\"%s\") is already mounted\n",
		       mtd->index, mtd->name);
		goto out_put;
	}

#define KERNEL_HARDSECT_SIZE 512
	sb->s_blocksize = KERNEL_HARDSECT_SIZE;

	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;


	// more setup
	sb->s_magic = NGFFS_SUPER_MAGIC;
	sb->s_op = &ngffs_super_operations;
	sb->s_maxbytes = 0xFFFFFFFF;
	sb->s_flags = flags | MS_NOATIME ;

	root_i = iget(sb, 1);
	if (is_bad_inode(root_i)) {
		PK_ERR("get root inode failed\n");
		goto out_putcache;
	}

	PK_DBG("d_alloc_root()\n");
	sb->s_root = d_alloc_root(root_i);
	if (!sb->s_root)
		goto out_putroot;


	// debug cruft
	PK_DBG("ngffs reading super block\n");
	PK_DBG("blocksize is %d\n",KERNEL_HARDSECT_SIZE);
	PK_DBG("blocksize bits is %d\n",sb->s_blocksize_bits);
	PK_DBG("erase size is %d\n",mtd->erasesize);
	PK_DBG("size is %d\n",mtd->size);
	PK_DBG("blocks is %d\n",mtd->size/mtd->erasesize);
	PK_DBG("size of ngffs_block is %d\n",sizeof(struct ngffs_block));
	PK_DBG("alloced is %d\n",sizeof(struct ngffs_block)*(mtd->size/mtd->erasesize));


	// set the user-definable data of the superblock to our data
	PK_DBG("scanned flash\n");

	if (ngffs_dir) {
		create_proc_read_entry(sb->s_id , 
					S_IWUSR | S_IRUGO, 
					ngffs_dir, 
					ngffs_read_proc, 
					sb);
	}

	up(&ngsb->global_lock);
	return sb;

out_putroot:
	iput(root_i);
out_putcache:
	ngffs_cleanup(sb);
out_put:
	kfree(ngsb);
	put_mtd_device(mtd);

	up(&ngsb->global_lock);
	return sb;
}

static struct super_block *ngffs_get_sb_mtdnr(struct file_system_type *fs_type,
					      int flags, const char *dev_name,
					      void *data, int mtdnr)
{
	struct mtd_info *mtd;

	PK_DBG("ngffs_get_sb_mtdnr(...,%i)\n",mtdnr);

	mtd = get_mtd_device(NULL, mtdnr);
	if (!mtd) {
		PK_DBG("MTD device #%u doesn't appear to exist\n", mtdnr);
		return ERR_PTR(-EINVAL);
	}

	return ngffs_get_sb_mtd(fs_type, flags, dev_name, data, mtd);
}

//struct super_block * ngffs_read_super (struct super_block * sb, void * data,
//                                      int silent)
static struct super_block *ngffs_get_sb(struct file_system_type *fs_type,
					int flags, const char *dev_name,
					void *data)
{
	/*
	 */
	int err;
	struct nameidata nd;
	int mtdnr;

	PK_DBG("ngffs_get_sb()\n");

	if (dev_name[0] == 'm' && dev_name[1] == 't' && dev_name[2] == 'd') {
		/* Probably mounting without the blkdev crap */
		if (dev_name[3] == ':') {
			struct mtd_info *mtd;

			/* Mount by MTD device name */
			PK_DBG("mtd:%%s, name \"%s\"\n", dev_name+4);
			for (mtdnr = 0; mtdnr < MAX_MTD_DEVICES; mtdnr++) {
				mtd = get_mtd_device(NULL, mtdnr);
				if (mtd) {
					if (!strcmp(mtd->name, dev_name+4))
						return ngffs_get_sb_mtd(fs_type, flags, dev_name, data, mtd);
					put_mtd_device(mtd);
				}
			}
			PK_WARN("ngffs_get_sb(): MTD device with name \"%s\" not found.\n", dev_name+4);
		} else if (isdigit(dev_name[3])) {
			/* Mount by MTD device number name */
			char *endptr;

			mtdnr = simple_strtoul(dev_name+3, &endptr, 0);
			if (!*endptr) {
				/* It was a valid number */
				PK_DBG("ngffs_get_sb(): mtd%%d, mtdnr %d\n", mtdnr);
				return ngffs_get_sb_mtdnr(fs_type, flags, dev_name, data, mtdnr);
			}
		}
	}

	/* Try the old way - the hack where we allowed users to mount
	/dev/mtdblock$(n) but didn't actually _use_ the blkdev */

	err = path_lookup(dev_name, LOOKUP_FOLLOW, &nd);

	PK_DBG("_get_sb(): path_lookup() returned %d, inode %p\n",
	       err, nd.dentry->d_inode);

	if (err)
		return ERR_PTR(err);

	err = -EINVAL;

	if (!S_ISBLK(nd.dentry->d_inode->i_mode))
		goto out;

	if (nd.mnt->mnt_flags & MNT_NODEV) {
		err = -EACCES;
		goto out;
	}

	if (imajor(nd.dentry->d_inode) != MTD_BLOCK_MAJOR) {
		if (!(flags & MS_VERBOSE)) /* Yes I mean this. Strangely */
			PK_NOTICE("Attempt to mount non-MTD device \"%s\" as JFFS2\n",
				  dev_name);
		goto out;
	}

	mtdnr = iminor(nd.dentry->d_inode);
	path_release(&nd);

	return ngffs_get_sb_mtdnr(fs_type, flags, dev_name, data, mtdnr);

out:
	path_release(&nd);
	return ERR_PTR(err);
}

static void ngffs_kill_sb(struct super_block *sb)
{
	//struct ngffs_info *ngsb = NGFFS_INFO(sb);

	PK_DBG("ngffs_kill_sb()\n");

	if (ngffs_dir) 
		remove_proc_entry(sb->s_id, ngffs_dir);

	generic_shutdown_super(sb);
	//put_mtd_device(sb->mtd); uhoh


	//kfree(ngsb); // hmmmm, what about put_super??
}

//static DECLARE_FSTYPE_DEV(ngffs_fs_type, "ngffs", ngffs_read_super);

static struct file_system_type ngffs_fs_type = {
	.owner   = THIS_MODULE,
	.name    = "ngffs",
	.get_sb  = ngffs_get_sb,
	.kill_sb = ngffs_kill_sb,
};

static int __init init_ngffs_fs(void)
{
	int ec;

	PK_NOTICE("NGFFS initializing\n");
	ngffs_dir = proc_mkdir(NGFFS_PROC_DIR_NAME, NULL);

	ec=register_filesystem(&ngffs_fs_type);
	PK_DBG("register_filesystem()=ec\n");

	return ec;
}

static void __exit exit_ngffs_fs(void)
{
	if (ngffs_dir) 
		remove_proc_entry(NGFFS_PROC_DIR_NAME, NULL);

	unregister_filesystem(&ngffs_fs_type);
}

module_init(init_ngffs_fs)
module_exit(exit_ngffs_fs)

MODULE_DESCRIPTION("The 'next generation' flash file system");
MODULE_AUTHOR("Koen Martens <kmartens@sonologic.nl>");
MODULE_LICENSE("GPL");

/* EOF */
