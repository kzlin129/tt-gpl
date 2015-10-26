/* fs/ngffs/inode.c
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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/blkdev.h>
#include <linux/mtd/mtd.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include "crc.h"
#include "kern.h"
#include "file.h"
#include "chunk.h"
#include "flash.h"

#ifdef NGFFS_HAS_SYSFILE
#include "sysfile.h"
#include "sysfiles.h"
#endif

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

// in dir.c
extern struct file_operations ngffs_dir_operations;
extern struct inode_operations ngffs_dir_inode_operations;

/*
 ** read the node inode->i_ino
 */
void ngffs_read_inode(struct inode *inode)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	struct list_head *curr;

	PK_DBG("----ngffs_read_inode()\n");
	PK_DBG("ngffs_read_inode, ino=%lu\n",inode->i_ino);

#ifdef NGFFS_HAS_SYSFILE
	if(inode->i_ino==1 || inode->i_ino>99) {
		// normal inodes
#endif
		CHECK_GLB_LOCK(ngsb->global_lock);

		list_for_each(curr, &ngsb->files) {
			struct file_list_entry *file;
			file=list_entry(curr, struct file_list_entry, list);
			if(file->chunk->entry_id==inode->i_ino) {
				// got file match
				PK_DBG("matched file \n");
				inode->u.generic_ip=NULL; //(void*)file;	// INODEPTR

				ngffs_copy_from_meta(inode,&file->chunk->meta);
				inode->i_fop=&ngffs_file_operations;
				inode->i_op=&ngffs_file_inode_operations;
				inode->i_mapping->a_ops = &ngffs_file_address_operations;
				inode->i_mapping->nrpages = 0;
				inode->i_size=file->chunk->len;
			}
		}
		list_for_each(curr, &ngsb->dirs) {
			struct dir_list_entry *dir;
			dir=list_entry(curr, struct dir_list_entry, list);
			if(dir->chunk->entry_id==inode->i_ino) {
				PK_DBG("matched dir \n");
				// got directory match
				inode->u.generic_ip=NULL; //(void*)dir;	// INODEPTR

				ngffs_copy_from_meta(inode,&dir->chunk->meta);
				inode->i_fop=&ngffs_dir_operations;
				inode->i_op=&ngffs_dir_inode_operations;
			}
		}
		PK_DBG("end of scan\n");
#ifdef NGFFS_HAS_SYSFILE
	} else {
		// sysfile inodes
		// ino 2 is sysfile dir
		// ino x for x>3 && x<100 is sysfile entry x-3
		inode->i_nlink=1;
		inode->i_uid=0;
		inode->i_gid=0;
		inode->i_mtime.tv_sec=0;
		inode->i_atime.tv_sec=0;
		inode->i_ctime.tv_sec=0;
		inode->i_mtime.tv_nsec=0;
		inode->i_atime.tv_nsec=0;
		inode->i_ctime.tv_nsec=0;

		if(inode->i_ino==2) {
			inode->i_fop=&ngffs_dir_operations;
			inode->i_op=&ngffs_dir_inode_operations;

			inode->i_mode=S_IFDIR | 0555;
		} else {
			inode->i_fop=&ngffs_sysfile_operations;
			inode->i_op=&ngffs_sysfile_inode_operations;
			inode->i_mapping->a_ops = &ngffs_sysfile_address_operations;
			inode->i_mapping->nrpages = 0;
			inode->i_size=ngffs_sysfiles[inode->i_ino-3].length;

			inode->i_mode=S_IFREG | 0444;
		}
	}
#endif
}

void ngffs_clear_inode (struct inode *inode)
{
	PK_DBG("ngffs_clear_inode()\n");
	PK_DBG("ngffs_clear_inode, ino=%lu\n",inode->i_ino);
}

/*
 * Use only to create a root block on an empty
 * erase block!
 */
int ngffs_create_root (struct ngffs_info * ngsb , struct ngffs_block *block)
{
	struct mtd_info *mtd=ngsb->mtd;
	struct dir_chunk *dir;
	struct dir_list_entry *dirle;
	struct timeval tv;

	PK_DBG("creating root inode\n");

	dir=kmalloc(sizeof(struct dir_chunk),GFP_KERNEL);
	if(dir==NULL) {
		PK_ERR("ngffs_create_root(): Fatal, could not allocate root dir_chunk!\n");
		//TODO ngsb->status=NGFFS_STATUS_CORRUPT;
		return -ENOMEM;
	} else {
		dirle=kmalloc(sizeof(struct dir_list_entry),GFP_KERNEL);
		if(dirle==NULL) {
			PK_ERR("ngffs_create_root(): Fatal, could not allocate root dir_chunk!\n");
			kfree(dir);
			//TODO ngsb->status=NGFFS_STATUS_CORRUPT;
			return -ENOMEM;
		} else {
			do_gettimeofday(&tv);

			dir->type=NGFFS_DIR_CHUNK;
			dir->valid=-1;
			RESET_WRITTEN(dir);
			SET_VALID(dir);
			SET_CRC(dir);
			dir->entry_id=1;
			dir->len=0;
			dir->namelen=0;
			dir->modification=1;
			// set meta data
			// mode drwxr-xr-x
			dir->meta.mode=S_IFDIR | S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH;
			dir->meta.link_count=1;
			dir->meta.uid=dir->meta.gid=0;	// owned by root:wheel
			dir->meta.mtime=tv.tv_sec;
			dir->meta.ctime=tv.tv_sec;

			PK_DBG("writing root chunk, entry_id=%i, len=%i, namelen=%i, mod=%i\n",dir->entry_id,dir->len,dir->namelen,dir->modification);


			if(ngffs_write(mtd,block->offset + EMPTY_BYTES + sizeof(struct header_chunk),(u_char *)dir,sizeof(struct dir_chunk))) {
				PK_ERR("ngffs_create_root(): Fatal, could not write root dir_chunk!\n");
				kfree(dir); kfree(dirle);
				//TODO ngsb->status=NGFFS_STATUS_CORRUPT;
				return -EIO;
			}
			if(ngffs_written_chunk(ngsb->sb,block->offset + EMPTY_BYTES + sizeof(struct header_chunk))) {
				PK_ERR("Fatal, could not mark root dir_chunk as WRITTEN");
				return -EIO;
			}
			SET_WRITTEN(dir);
			//SET_CRC(dir);
			// update free space
			block->freespace-=sizeof(struct dir_chunk);

			// insert into dir list
			dirle->chunk=dir;
			CHECK_GLB_LOCK(ngsb->global_lock);

			list_add_tail(&dirle->list,&ngsb->dirs);
		}
	}
	return 0;
}

/* EOF */
