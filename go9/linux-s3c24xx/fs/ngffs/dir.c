/* fs/ngfss/dir.c
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
#include <asm/errno.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include <asm/semaphore.h>
#include "crc.h"
#include "kern.h"
#include "chunk.h"
#include "file.h"
#include "flash.h"

#ifdef NGFFS_HAS_SYSFILE
#include "sysfile.h"
#include "sysfiles.h"
#endif

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

/* Urgh. Please tell me there's a nicer way of doing these. (from jffs2) */
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,48)
typedef int mknod_arg_t;
#define NAMEI_COMPAT(x) ((void *)x)
#else
typedef dev_t mknod_arg_t;
#define NAMEI_COMPAT(x) (x)
#endif

static int ngffs_readdir(struct file *, void *, filldir_t);

static int ngffs_create(struct inode *,struct dentry *,int,struct nameidata *);
static struct dentry *ngffs_lookup(struct inode *,struct dentry *,struct nameidata *);
static int ngffs_link(struct dentry *,struct inode *,struct dentry *);
static int ngffs_unlink(struct inode *,struct dentry *);
static int ngffs_symlink(struct inode *,struct dentry *,const char *);
static int ngffs_mkdir(struct inode *,struct dentry *,int);
static int ngffs_rmdir(struct inode *,struct dentry *);
#ifdef USE_UNIMPLEMENTED_METHODS
static int ngffs_mknod(struct inode *,struct dentry *,int,mknod_arg_t);
static int ngffs_rename(struct inode *, struct dentry *,
			 struct inode *, struct dentry *);
#endif /* USE_UNIMPLEMENTED_METHODS */

struct file_operations ngffs_dir_operations = {
	.read    = generic_read_dir,
	.readdir = ngffs_readdir,
//	.ioctl   = ngffs_ioctl,		// returns -EINVAL always
//	.fsync   = ngffs_null_fsync,	// returns 0 always
};

struct inode_operations ngffs_dir_inode_operations = {
	.create         = ngffs_create,
	.lookup         = ngffs_lookup,
	.link           = ngffs_link,
	.unlink         = ngffs_unlink,
	.symlink        = ngffs_symlink,
	.mkdir          = ngffs_mkdir,
	.rmdir          = ngffs_rmdir,
#ifdef USE_UNIMPLEMENTED_METHODS
	.mknod          = ngffs_mknod,
	.rename         = ngffs_rename,
#endif /* USE_UNIMPLEMENTED_METHODS */
	.setattr        = ngffs_setattr,
};

static int ngffs_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	unsigned long offset=filp->f_pos;
	struct ngffs_info *ngsb=NGFFS_INFO(filp->f_dentry->d_inode->i_sb);
	struct dir_chunk *chunk;
	struct generic_chunk *entry_chunk;
	struct inode *inode=filp->f_dentry->d_inode;
	__u32 ino;
	__u32 *entries;
	int relofs=0;		// set to +1 if this is root, to account for SYSFILE_DIR
	int rv=0;

	PK_DBG("ngffs_readdir()\n");


	if(offset==0) {
		rv=filldir(dirent,".",1,0,inode->i_ino,DT_DIR);
		if(rv<0) goto out;
		offset++;
	}
	if(offset==1) {
		rv=filldir(dirent,"..",2,1,filp->f_dentry->d_parent->d_inode->i_ino,DT_DIR);
		if(rv<0) goto out;
		offset++;
	}
#ifdef NGFFS_HAS_SYSFILE
	if(offset==2 && inode->i_ino==1) {	// only if entry_id=1 --> root node
		rv=filldir(dirent,NGFFS_SYSFILE_DIR,strlen(NGFFS_SYSFILE_DIR),2,filp->f_dentry->d_parent->d_inode->i_ino,DT_DIR);
		if(rv<0) goto out;
		offset++;
	}
	if(inode->i_ino==2) {	// read sysfile dir
		while(offset-2 < NGFFS_SYSFILES) {
			rv=filldir(dirent,ngffs_sysfiles[offset-2].name,strlen(ngffs_sysfiles[offset-2].name),offset,2,DT_REG);
			if(rv<0) goto out;
			offset++;
		}
		goto out;
	} else {
		if(inode->i_ino==1) relofs=1;
#endif

		// at this point we need the fs-wide lock
		down(&ngsb->global_lock);

		chunk=(struct dir_chunk *)ngffs_get_chunk(inode->i_sb,inode->i_ino);
		if(chunk==NULL) {
			PK_ERR(KERN_ERR "Unable to get inode %lu from cache [1]\n",inode->i_ino);
			goto out_unlock;
		}
		if(chunk->type!=NGFFS_DIR_CHUNK) { rv=-ENOTDIR; goto out_unlock; }

		entries=(__u32 *)(((char *)chunk)+sizeof(struct dir_chunk)+chunk->namelen);
		for(;offset<(chunk->len+2+relofs);offset++) {
			ino=entries[offset-2-relofs];
			entry_chunk=ngffs_get_chunk(inode->i_sb,ino);
			if(entry_chunk==NULL) {
				PK_ERR(KERN_ERR "Unable to get inode %d from cache [2]\n",ino);
				rv=-ENXIO;
				goto out_unlock;
			}
			switch(entry_chunk->type) {
			case NGFFS_DIR_CHUNK:
				rv=filldir(dirent,((char *)entry_chunk)+sizeof(struct dir_chunk),
					   DIR_CHUNK(entry_chunk)->namelen,offset,
					   inode->i_ino,DT_DIR);
				if(rv<0) goto out_unlock;
				break;
			case NGFFS_FILE_CHUNK:
				rv=filldir(dirent,((char *)entry_chunk)+sizeof(struct file_chunk),
					   FILE_CHUNK(entry_chunk)->namelen,offset,
					   inode->i_ino,DT_REG);
				if(rv<0) goto out_unlock;
				break;
			default:
				rv=filldir(dirent,"?",1,offset,1,DT_DIR);
				if(rv<0) goto out_unlock;
				break;
			}
		}

#ifdef NGFFS_HAS_SYSFILE
	}
#endif

out_unlock:
	up(&ngsb->global_lock);
out:
	//  up(&f->sem);
	filp->f_pos=offset;
	return 0;
}

static int ngffs_create(struct inode *parent,struct dentry *dentry,int mode,struct nameidata *nd)
{
	struct file_chunk *file=0;
	struct file_list_entry *filele=0;
	struct inode *inode;
	struct ngffs_info *ngsb=NGFFS_INFO(parent->i_sb);
	//int block,ec,i;
	int block,i;
	struct dir_chunk *parent_chunk;
	__u32 *entries;
	__u32 oldofs,newofs;
	struct timeval tv;
	int rv=0;

	PK_DBG("ngffs_create()\n");
	PK_DBG("name [%s]\n",dentry->d_name.name);

#ifdef NGFFS_HAS_SYSFILE
	if(parent->i_ino==2) return -EACCES;
#endif

	// obtain the filesystem lock
	down(&ngsb->global_lock);

	// get parent chunk
	parent_chunk=ngffs_get_dirptr(parent->i_sb,parent->i_ino);
	if(parent_chunk==NULL) {
		PK_ERR("Unable to get chunk ptr for dir %lu\n",parent->i_ino);
		return -EIO;
	}

	// if the max number of dir entries would be exceeded, err
	if(parent_chunk->len>=NGFFS_MAXDIRENT) {
		up(&ngsb->global_lock);
		return -ENOSPC;
	}

	// check available space
	if((block=ngffs_contig_available(parent->i_sb,
					 sizeof(struct file_chunk) +
					 dentry->d_name.len +
					 sizeof(struct dir_chunk) +
					 parent_chunk->namelen +
					 ((parent_chunk->len+1)*sizeof(__u32))))==-1) {
		rv=-ENOSPC;
		goto create_error;
	}
	// reload chunk ptr's in case contig_available did GC
	parent_chunk=ngffs_get_dirptr(parent->i_sb,parent->i_ino);

	PK_DBG("got space in block %i\n",block);

	// alloc all memory
	file=kmalloc(sizeof(struct file_chunk)+dentry->d_name.len,GFP_KERNEL);
	if(file==NULL) {
		PK_ERR("Out of memory allocating file_chunk in ngffs_create()\n");
		rv=-ENOMEM;
		goto create_error;
	}

	filele=kmalloc(sizeof(struct file_list_entry),GFP_KERNEL);
	if(filele==NULL) {
		PK_ERR("Out of memory allocating file_list_entry in ngffs_create()\n");
		rv=-ENOMEM;
		goto create_error;
	}

	filele->cache=kmalloc(NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE)*sizeof(struct ngffs_data_cache),GFP_KERNEL);
	if(filele->cache==NULL) {
		PK_ERR("Out of memory allocating file_list_entry->cache in ngffs_create()\n");
		rv=-ENOMEM;
		goto create_error;
	}
#ifdef NGFFS_PARANOIA
	memset(filele->cache, 0xef, NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE)*sizeof(struct ngffs_data_cache));
#endif // NGFFS_PARANOIA

	for(i=0;i<NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE);i++) {
		NGFFS_CACHE(filele,i)->modification=0;
	}

	PK_DBG("got memory\n");

	inode=new_inode(parent->i_sb);
	if(inode==NULL) {
		PK_ERR("Out of memory allocating inode in create()\n");
		rv=-ENOMEM;
		goto create_error;
	}

	PK_DBG("got inode\n");

	// initialize the new inode
	inode->i_op = &ngffs_file_inode_operations;
	inode->i_fop = &ngffs_file_operations;
	inode->i_mapping->a_ops = &ngffs_file_address_operations;
	inode->i_mapping->nrpages = 0;

	inode->i_nlink=1;
	inode->i_mode=mode;
	inode->i_uid = current->fsuid;
	if (parent->i_mode & S_ISGID) {
		inode->i_gid = parent->i_gid;
		//    if (S_ISDIR(mode))		irrelevant
		//       inode->i_mode |= S_ISGID;
	} else {
		inode->i_gid = current->fsgid;
	}

	do_gettimeofday(&tv);
	inode->i_atime.tv_sec=inode->i_ctime.tv_sec=inode->i_mtime.tv_sec=tv.tv_sec;
	inode->i_atime.tv_nsec=inode->i_ctime.tv_nsec=inode->i_mtime.tv_nsec=0;

	parent->i_mtime.tv_sec=parent->i_atime.tv_sec=tv.tv_sec;

	inode->i_blksize=parent->i_sb->s_blocksize;
	inode->i_blocks=0;
	inode->i_size=0;

	inode->i_sb=parent->i_sb;

	//  inode->u.generic_ip=(void*)filele;	// INODEPTR
	inode->u.generic_ip=NULL;	// obsolete

	insert_inode_hash(inode);

	// initialize new file chunk
	file->type=NGFFS_FILE_CHUNK;
	file->valid=-1;
	RESET_WRITTEN(file);
	SET_VALID(file);
	SET_CRC(file);
	file->len=0;
	file->namelen=dentry->d_name.len;
	file->entry_id=++(ngsb->max_entry_id);
	file->modification=1;

	inode->i_ino=file->entry_id;

	// copy inode metadata to filechunk metadata
	ngffs_copy_to_meta(inode,&file->meta);

	// copy file name
	bcopy(dentry->d_name.name,((char *)file)+sizeof(struct file_chunk),dentry->d_name.len);

	// and point to chunk from file list
	filele->chunk=file;

	PK_DBG("before resize\n");
	// resize & modify parent
	parent_chunk=(struct dir_chunk *)ngffs_resize_chunk((struct generic_chunk *)parent_chunk,sizeof(__u32));
	if(parent_chunk==NULL) {
		rv=-ENOMEM;
		goto create_error;
	}

	// set modified, accesed and create time, also for parent
	inode->i_mtime.tv_sec=inode->i_atime.tv_sec=parent_chunk->meta.mtime=tv.tv_sec;

	parent_chunk->modification++;

	// add the new inode to the parent dir list, and replace it
	entries=(__u32 *)(((char *)parent_chunk)+sizeof(struct dir_chunk)+DIR_CHUNK(parent_chunk)->namelen);
	entries[parent_chunk->len]=inode->i_ino;
	PK_DBG("replacing chunk\n");
	ngffs_replace_chunk(parent->i_sb,(struct generic_chunk *)parent_chunk);
	parent_chunk->len+=1;

	PK_DBG("putting new chunk\n");
	// write new chunk
	if( !(filele->ofs=ngffs_put_chunk(parent->i_sb,(struct generic_chunk *)file,block)) ) {
		PK_ERR("Error putting chunk\n");
		rv=-EIO;
		goto create_error;
	}
	//  file->valid=NGFFS_CHUNK_WRITTEN;
	//  filele->ofs=ngffs_get_chunk_ofs(parent->i_sb,file->entry_id);
	PK_DBG("replacing chunk\n");

	// write parent
	oldofs=ngffs_get_chunk_ofs(parent->i_sb,parent->i_ino);

	PK_DBG("old offset %i\n",oldofs);

	if( !(newofs=ngffs_put_chunk(parent->i_sb,(struct generic_chunk *)parent_chunk,block)) ) {
		PK_ERR("Error putting (parent) chunk\n");
		rv=-EIO;
		goto create_error;
	}
	//  parent_chunk->valid=NGFFS_CHUNK_WRITTEN;

	PK_DBG("about to invalidate at oldofs\n");
	// invalidate parent
	ngffs_invalidate_chunk(parent->i_sb,oldofs);

	// get new ofs
	//  newofs=ngffs_get_chunk_ofs(parent->i_sb,parent->i_ino);
	PK_DBG("new ofs = %i\n",newofs);
	ngffs_update_chunk_ofs(parent->i_sb,parent->i_ino,newofs);

	// finalize
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_add_tail(&filele->list,&ngsb->files);
	d_instantiate(dentry, inode);

create_done:
	up(&ngsb->global_lock);
	return rv;

create_error:
	if(file) kfree(file);
	if(filele) {
		if(filele->cache) kfree(filele->cache);
		kfree(filele);
	}
	goto create_done;
}

static struct dentry *ngffs_lookup(struct inode *inode,struct dentry *target,struct nameidata *nd)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	struct generic_chunk *chunk;
	struct generic_chunk *entry_chunk;
	struct inode *newinode=NULL;
	__u32 *entries;
	int i,fd;
	struct dentry *rv=0;

	PK_DBG("ngffs_lookup()\n");
	PK_DBG("name [%s]\n",target->d_name.name);

#ifdef NGFFS_HAS_SYSFILE
	if(inode->i_ino==1 && !strcmp(target->d_name.name,NGFFS_SYSFILE_DIR)) {
		newinode=iget(inode->i_sb,2);
		if(!inode) {
			PK_WARN("iget() failed for ino 2\n");
			return (ERR_PTR(-EIO));
		}
		d_add(target,newinode);
		return NULL;
	} else if(inode->i_ino==2) {
		for(i=0;i<NGFFS_SYSFILES;i++) {
			if(!strcmp(ngffs_sysfiles[i].name,target->d_name.name)) {
				newinode=iget(inode->i_sb,i+3);
				if(!inode) {
					PK_WARN("iget() failed for ino %i\n",i+3);
					return (ERR_PTR(-EIO));
				}
				d_add(target,newinode);
				return NULL;
			}
		}
	}
#endif

	// if we get here, we are not in sysfile so we need the fs lock
	down(&ngsb->global_lock);

	// get the chunk from the cache
	chunk=ngffs_get_chunk(inode->i_sb,inode->i_ino);
	if(chunk==NULL) {
		PK_ERR("Unable to get inode %lu from cache [3]\n",inode->i_ino);
		rv=ERR_PTR(-EIO);
		goto out;
	}
	if(chunk->type!=NGFFS_DIR_CHUNK) { rv=ERR_PTR(-ENOTDIR); goto out; }

	// loop over all dir entries
	entries=(__u32 *)(((char *)chunk)+sizeof(struct dir_chunk)+DIR_CHUNK(chunk)->namelen);
	for(i=0,fd=0;i<DIR_CHUNK(chunk)->len;i++) {
		entry_chunk=ngffs_get_chunk(inode->i_sb,entries[i]);

		if(entry_chunk==NULL) {
			PK_ERR("Unable to get inode %i from cache - Flash rescan forced\n",entries[i]);
			ngffs_rescan_flash(inode->i_sb);
			entry_chunk=ngffs_get_chunk(inode->i_sb,entries[i]);

			if (entry_chunk==NULL) {
				PK_ERR("Unable to get inode %i from cache [4]\n",entries[i]);
				rv=ERR_PTR(-EIO);
				goto out;
			}
		}

		if(entry_chunk==NULL) {
			PK_ERR("Unable to get inode %i from cache [5]\n",entries[i]);
			rv=ERR_PTR(-EIO);
			goto out;
		}
		switch(entry_chunk->type) {
		case NGFFS_DIR_CHUNK:
			if( (DIR_CHUNK(entry_chunk)->namelen==target->d_name.len) &&
			    !strncmp( ((char *)DIR_CHUNK(entry_chunk))+sizeof(struct dir_chunk),target->d_name.name,target->d_name.len)) {
				fd=DIR_CHUNK(entry_chunk)->entry_id;
			}
			break;
		case NGFFS_FILE_CHUNK:
			if( (FILE_CHUNK(entry_chunk)->namelen==target->d_name.len) &&
			    !strncmp( ((char*)FILE_CHUNK(entry_chunk))+sizeof(struct file_chunk),target->d_name.name,target->d_name.len)) {
				fd=FILE_CHUNK(entry_chunk)->entry_id;
			}
			break;
		default:
			// ignore unknown (and therefore unnamed) entries
			break;
		}
	}

	if(fd) {
		newinode=iget(inode->i_sb, fd);
		if(!inode) {
			PK_WARN("iget() failed for ino #%u\n", fd);
			rv=ERR_PTR(-EIO);
			goto out;
		}
		d_add(target,newinode);
	}

out:
	up(&ngsb->global_lock);
	return rv;
}

static int ngffs_link(struct dentry *source,struct inode *inode,struct dentry *dest)
{
	PK_DBG("ngffs_link()\n");
	return -EACCES;
}

// delete a file
static int ngffs_unlink(struct inode *inode,struct dentry *dentry)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	struct file_chunk *file;
	struct dir_chunk *parent;
	//struct dir_chunk *newparent;
	__u32 fileofs;
	__u32 parentofs;
	__u32 newofs;
	__u32 *entries;
	int mode;
	int rv=0;
	int block,i;
	struct ngffs_data_cache *cached;

	PK_DBG("ngffs_unlink(), inode %li\n",inode->i_ino);

	PK_DBG("entry %li\n", dentry->d_inode->i_ino);
	PK_DBG("parent %li\n",dentry->d_parent->d_inode->i_ino);

#ifdef NGFFS_HAS_SYSFILE
	if((dentry->d_inode->i_ino>1) && (dentry->d_inode->i_ino<100)) return -EPERM;
#endif

	down(&ngsb->global_lock);

	// shorthands, can only be obtained if this is not the
	// sysfile directory
	file=ngffs_get_fileptr(inode->i_sb,dentry->d_inode->i_ino);
	parent=ngffs_get_dirptr(inode->i_sb,dentry->d_parent->d_inode->i_ino);
	if(file==NULL || parent==NULL) {
		PK_ERR("Error getting chunk ptrs, file=0x%p, parent=0x%p\n",file,parent);
		up(&ngsb->global_lock);
		return -EIO;
	}
	fileofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_inode->i_ino);
	parentofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino);

	// delete entry from parent chunk
	entries=(__u32 *)(((char *)parent)+sizeof(struct dir_chunk)+parent->namelen);
	for(i=0,mode=0;i<parent->len;i++) {
		if(mode==1) entries[i-1]=entries[i];
		if((mode==0) && (entries[i]==file->entry_id)) mode++;
	}
	// TODO: atime mtime

	// resize the allocated space (not strictly necesarry)
	//  newparent=ngffs_resize_chunk(GENERIC_CHUNK(parent),-sizeof(__u32));
	//  if(newparent==NULL) {
	//	  rv=-ENOMEM;
	//	  goto out;
	//  }
	//  INODE_DIR(dentry->d_parent->d_inode)=parent=newparent;

	// get some space
	if((block=ngffs_contig_available(dentry->d_sb,CHUNK_SIZE(parent,1)))==-1) {
		rv=-ENOSPC;
		goto out;
	}
	// reload chunk ptrs in case contig_available did GC
	file=ngffs_get_fileptr(inode->i_sb,dentry->d_inode->i_ino);
	parent=ngffs_get_dirptr(inode->i_sb,dentry->d_parent->d_inode->i_ino);
	fileofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_inode->i_ino);
	parentofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino);
	
  PK_DBG("parent->len %i\n",parent->len);
	parent->len--;
	parent->modification++;

	// write new parent chunk
	if(!(newofs=ngffs_put_chunk(inode->i_sb,(struct generic_chunk *)parent,block))) {
		rv=-EIO; goto out;
	}
	//parent->valid=NGFFS_CHUNK_WRITTEN;
	ngffs_set_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino,newofs);
	PK_DBG("newofs=%i, INODE_DIR_OFS=%i\n",newofs,ngffs_get_cached_chunk_ofs(inode->i_sb,
		dentry->d_parent->d_inode->i_ino));


	// invalidate file & old parent chunk
	ngffs_invalidate_chunk(dentry->d_sb,parentofs);
	ngffs_invalidate_chunk(dentry->d_sb,fileofs);

	// invalidate data chunks
	if(file->len) {
		for(i=0;i<(file->len+NGFFS_MAXCSIZE-1)/NGFFS_MAXCSIZE;i++) {
			cached=ngffs_get_cached_data_chunk(inode->i_sb,file->entry_id,i);
			if(!cached) {
				PK_WARN("no cache chunk %i for file %i!\n",i,file->entry_id);
			} else if (!cached->modification) {
				PK_WARN("cache chunk %i for file %i not modified!\n",i,file->entry_id);
			} else {
				ngffs_invalidate_chunk(dentry->d_sb,cached->offset);
			}
		}
	}

	// remove list entry
	ngffs_delete_file_chunk(inode->i_sb,file->entry_id);


	dentry->d_inode->i_nlink--;

out:
	if(rv) { // error->restore parent dir
		entries[parent->len]=file->entry_id;
		parent->len++;
		parent->modification--;
	}
	up(&ngsb->global_lock);
	return rv;
}

static int ngffs_symlink(struct inode *inode,struct dentry *dentry,const char *name)
{
	PK_DBG("ngffs_symblink()\n");
	return -EACCES;
	return 0;
}

static int ngffs_mkdir(struct inode *parent,struct dentry *dentry,int mode)
{
	struct dir_chunk *dir;
	struct dir_list_entry *dirle;
	struct inode *inode;
	struct ngffs_info *ngsb=NGFFS_INFO(parent->i_sb);
	int block; //,ec;
	struct dir_chunk *parent_chunk;
	__u32 *entries;
	__u32 oldofs,newofs,dirofs;
	struct timeval tv;

#ifdef NGFFS_HAS_SYSFILE
	if(parent->i_ino==2) return -EACCES;
#endif

	PK_DBG("ngffs_mkdir()\n");

	down(&ngsb->global_lock);

	// get parent chunk
	parent_chunk=ngffs_get_dirptr(parent->i_sb,parent->i_ino);

	// check if we are at NGFFS_MAXDIRENT already
	if(parent_chunk->len>=NGFFS_MAXDIRENT) {
		up(&ngsb->global_lock);
		return -ENOSPC;
	}

	// check available space
	if((block=ngffs_contig_available(parent->i_sb,
					 sizeof(struct dir_chunk) +
					 dentry->d_name.len +
					 sizeof(struct dir_chunk) +
					 parent_chunk->namelen +
					 ((parent_chunk->len+1)*sizeof(__u32))))==-1) {
		up(&ngsb->global_lock);
		return -ENOSPC;
	}
	// reload chunk ptrs in case contig_available did GC
	parent_chunk=ngffs_get_dirptr(parent->i_sb,parent->i_ino);

	// alloc all memory
	dir=kmalloc(sizeof(struct dir_chunk)+dentry->d_name.len,GFP_KERNEL);
	if(dir==NULL) {
		PK_WARN("Out of memory allocating dir_chunk in mkdir()\n");
		up(&ngsb->global_lock);
		return -ENOMEM;
	}

	dirle=kmalloc(sizeof(struct dir_list_entry),GFP_KERNEL);
	if(dirle==NULL) {
		PK_ERR("Out of memory allocating dir_list_entry in mkdir()\n");
		kfree(dir);
		up(&ngsb->global_lock);
		return -ENOMEM;
	}

	inode=new_inode(parent->i_sb);
	if(inode==NULL) {
		PK_ERR("Out of memory allocating inode in mkdir()\n");
		kfree(dir); kfree(dirle);
		up(&ngsb->global_lock);
		return -ENOMEM;
	}

	// initialize the new inode
	inode->i_op = &ngffs_dir_inode_operations;
	inode->i_fop = &ngffs_dir_operations;

	inode->i_nlink=1;
	inode->i_mode=mode|S_IFDIR;
	inode->i_uid = current->fsuid;
	if (parent->i_mode & S_ISGID) {
		inode->i_gid = parent->i_gid;
		if (S_ISDIR(mode))
			inode->i_mode |= S_ISGID;
	} else {
		inode->i_gid = current->fsgid;
	}

	do_gettimeofday(&tv);
	inode->i_atime.tv_sec=inode->i_ctime.tv_sec=inode->i_mtime.tv_sec=tv.tv_sec;

	parent->i_mtime.tv_sec=parent->i_atime.tv_sec=tv.tv_sec;

	inode->i_blksize=parent->i_sb->s_blocksize;
	inode->i_blocks=0;
	inode->i_size=0;

	inode->i_sb=parent->i_sb;

	//  inode->u.generic_ip=(void*)dirle;	// INODEPTR
	inode->u.generic_ip=NULL;	// obsoleted..

	insert_inode_hash(inode);

	// initialize new dir chunk
	dir->type=NGFFS_DIR_CHUNK;
	dir->valid=-1;
	SET_VALID(dir);
	RESET_WRITTEN(dir);
	SET_CRC(dir);
	dir->len=0;
	dir->namelen=dentry->d_name.len;
	dir->entry_id=++(ngsb->max_entry_id);
	dir->modification=1;

	PK_DBG("new entry id=%i\n",dir->entry_id);

	inode->i_ino=dir->entry_id;

	ngffs_copy_to_meta(inode,&dir->meta);

	bcopy(dentry->d_name.name,((char *)dir)+sizeof(struct dir_chunk),dentry->d_name.len);

	dirle->chunk=dir;

	// resize & modify parent
	parent_chunk=(struct dir_chunk *)ngffs_resize_chunk((struct generic_chunk *)parent_chunk,sizeof(__u32));
	if(parent_chunk==NULL) {
		kfree(dir); kfree(dirle); kfree(inode);
		up(&ngsb->global_lock);
		return -ENOMEM;
	}
	parent_chunk->meta.mtime=tv.tv_sec;
	parent_chunk->modification++;
	entries=(__u32 *)(((char *)parent_chunk)+sizeof(struct dir_chunk)+DIR_CHUNK(parent_chunk)->namelen);
	entries[parent_chunk->len]=inode->i_ino;
	ngffs_replace_chunk(parent->i_sb,(struct generic_chunk *)parent_chunk);
	parent_chunk->len+=1;

	// write new chunk
	if( !(dirofs=ngffs_put_chunk(parent->i_sb,(struct generic_chunk *)dir,block)) ) {
		PK_ERR("Error putting chunk\n");
		up(&ngsb->global_lock);
		return -EIO;
	}
	//  dir->valid=NGFFS_CHUNK_WRITTEN;

	// write parent
	oldofs=ngffs_get_chunk_ofs(parent->i_sb,parent->i_ino);
	PK_DBG("oldofs=%u\n",oldofs);

	if( !(newofs=ngffs_put_chunk(parent->i_sb,(struct generic_chunk *)parent_chunk,block)) ) {
		PK_ERR("Error putting (parent) chunk\n");
		up(&ngsb->global_lock);
		return -EIO;
	}
	//  parent_chunk->valid=NGFFS_CHUNK_WRITTEN;

	// invalidate parent
	ngffs_invalidate_chunk(parent->i_sb,oldofs);

	// get new ofs
	ngffs_update_chunk_ofs(parent->i_sb,parent->i_ino,newofs);


	// finalize
	dirle->ofs=dirofs;
	PK_DBG("set dirle->ofs to %u\n",dirle->ofs);
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_add_tail(&dirle->list,&ngsb->dirs);
	d_instantiate(dentry, inode);

	up(&ngsb->global_lock);
	return 0;
}

static int ngffs_rmdir(struct inode *inode,struct dentry *dentry)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	struct dir_chunk *dir;
	struct dir_chunk *parent;
	//struct dir_chunk *newparent;
	__u32 dirofs;
	__u32 parentofs;
	__u32 newofs;
	__u32 *entries=NULL;
	int mode;
	int rv=0;
	int block,i;

	PK_DBG("ngffs_rmdir(), inode %li\n",inode->i_ino);

	PK_DBG("entry %li\n", dentry->d_inode->i_ino);
	PK_DBG("parent %li\n",dentry->d_parent->d_inode->i_ino);

#ifdef NGFFS_HAS_SYSFILE
	if((dentry->d_inode->i_ino>1) && (dentry->d_inode->i_ino<100)) return -EPERM;
#endif

	down(&ngsb->global_lock);

	// shorthands, can only be obtained if this is not the
	// sysfile directory
	dir=ngffs_get_dirptr(inode->i_sb,dentry->d_inode->i_ino);
	parent=ngffs_get_dirptr(inode->i_sb,dentry->d_parent->d_inode->i_ino);
	if(dir==NULL || parent==NULL) {
		PK_ERR("Unable to get dir/parent chunk ptrs, dir=0x%p, parent=0x%p\n",dir,parent);
		up(&ngsb->global_lock);
		return -EIO;
	}
	dirofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_inode->i_ino);
	parentofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino);

	if(dir->len!=0) return -ENOTEMPTY;

	// delete entry from parent chunk
	entries=(__u32 *)(((char *)parent)+sizeof(struct dir_chunk)+parent->namelen);
	for(i=0,mode=0;i<parent->len;i++) {
		if(mode==1) entries[i-1]=entries[i];
		if((mode==0) && (entries[i]==dir->entry_id)) mode++;
	}
	parent->len--;
	parent->modification++;
	// TODO: atime mtime

	// resize the allocated space (not strictly necesarry)
	//  newparent=ngffs_resize_chunk(GENERIC_CHUNK(parent),-sizeof(__u32));
	//  if(newparent==NULL) {
	//	  rv=-ENOMEM;
	//	  goto out;
	//  }
	//  INODE_DIR(dentry->d_parent->d_inode)=parent=newparent;

	// get some space
	if((block=ngffs_contig_available(dentry->d_sb,CHUNK_SIZE(parent,1)))==-1) {
		rv=-ENOSPC;
		goto out;
	}
	// reset file/dir chunk ptr's as these might have changed
	// due to GC in contig_available
	dir=ngffs_get_dirptr(inode->i_sb,dentry->d_inode->i_ino);
	parent=ngffs_get_dirptr(inode->i_sb,dentry->d_parent->d_inode->i_ino);
	dirofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_inode->i_ino);
	parentofs=ngffs_get_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino);

	//
	PK_DBG("block is %i\n",block);
	PK_DBG("offset=%i, size=%i, freespace=%i\n",
	       ngsb->blocklist[block].offset,
	       ngsb->blocklist[block].size,
	       ngsb->blocklist[block].freespace);

	if(!(newofs=ngffs_put_chunk(inode->i_sb,(struct generic_chunk *)parent,block))) {
		rv=-EIO; goto out;
	}
	//  parent->valid=NGFFS_CHUNK_WRITTEN;
	PK_DBG("newofs = %i\n",newofs);
	ngffs_set_cached_chunk_ofs(inode->i_sb,dentry->d_parent->d_inode->i_ino,newofs);




	// invalidate old parent & chunk itself
	ngffs_invalidate_chunk(dentry->d_sb,dirofs);
	ngffs_invalidate_chunk(dentry->d_sb,parentofs);

	// remove list entry
	ngffs_delete_dir_chunk(inode->i_sb,dir->entry_id);

	dentry->d_inode->i_nlink--;

out:
	if(rv) { // error->restore parent dir
		entries[parent->len]=dir->entry_id;
		parent->len++;
		parent->modification--;
	}
	up(&ngsb->global_lock);
	return rv;
}

#ifdef USE_UNIMPLEMENTED_METHODS
static int ngffs_mknod(struct inode *inode,struct dentry *dentry,int mode, mknod_arg_t rdev)
{
	PK_DBG("ngffs_mknod()\n");
	return -EINVAL;
}

static int ngffs_rename(struct inode *old_inode, struct dentry *old_dentry,
			struct inode *new_inode, struct dentry *new_dentry)
{
	PK_DBG("ngffs_rename()\n");
	return 0;
}
#endif /* USE_UNIMPLEMENTED_METHODS */

/* EOF */
