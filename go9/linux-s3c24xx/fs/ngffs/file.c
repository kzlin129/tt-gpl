/* fs/ngffs/file.c
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
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/blkdev.h>
#include <linux/mtd/mtd.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/semaphore.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include "crc.h"
#include "kern.h"
#include "chunk.h"
#include "flash.h"
#include "garbage.h"

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

int ngffs_setattr(struct dentry *dentry, struct iattr *iattr);
int ngffs_readpage(struct file *filp, struct page *pg);
int ngffs_prepare_write(struct file *filp, struct page *pg, unsigned start, unsigned end);
int ngffs_commit_write(struct file *filp, struct page *pg, unsigned start, unsigned end);

int do_ngffs_commit_write(struct file *filp, void *page, pgoff_t pgindex, unsigned start, unsigned end, int lock);

struct file_operations ngffs_file_operations = {
	.llseek = generic_file_llseek,
	.open   = generic_file_open,
	.read   = generic_file_read,
	.write  = generic_file_write,
//	.ioctl  = ngffs_ioctl,
	.mmap   = generic_file_mmap,
//	.fsync  = ngffs_null_fsync,
};

/* ngffs_file_inode_operations */

struct inode_operations ngffs_file_inode_operations = {
	.setattr = ngffs_setattr
};


struct address_space_operations ngffs_file_address_operations = {
	.readpage      = ngffs_readpage,
	.prepare_write = ngffs_prepare_write,
	.commit_write  = ngffs_commit_write,
};


#ifdef DEBUG_BLOCKS
static inline void dump_blocks(struct ngffs_info *c,char *str)
{
	int i;
	PK_DBG("-----[ %s\n",str);
	for(i=0;i<c->blocks;i++) {
		PK_DBG("block %i: free=%8u, res=%8u\n",i,c->blocklist[i].freespace,c->blocklist[i].reserved);
	}
	mdelay(500);
}

static inline void dump_block(struct ngffs_info *c,char *str,int i)
{
	PK_DBG("%s block %i: free=%8u, res=%8u\n",str,i,c->blocklist[i].freespace,c->blocklist[i].reserved);
}
#else
static inline void dump_blocks(struct ngffs_info *c,char *str) {}
static inline void dump_block(struct ngffs_info *c,char *str,int i) {}
#endif

/*
struct iattr {
	unsigned int    ia_valid;
	umode_t         ia_mode;
	uid_t           ia_uid;
	gid_t           ia_gid;
	loff_t          ia_size;
	time_t          ia_atime;
	time_t          ia_mtime;
	time_t          ia_ctime;
	unsigned int    ia_attr_flags;
};

struct ngffs_metadata {
	__u32         link_count;
	umode_t       mode;
	uid_t         uid;
	gid_t         gid;
	//  time_t      atime;  // last accessed not used, mtime returned instead
	time_t        mtime;
	time_t        ctime;
};
*/

int ngffs_setattr(struct dentry *dentry, struct iattr *iattr)
{
	struct ngffs_info *ngsb=NGFFS_INFO(dentry->d_inode->i_sb);
	struct ngffs_metadata meta;
	struct generic_chunk *chunk;
	int change=0;
	int rv=0;
	int block;
	__u32 newofs,oldofs;
	int invalidate_chunks=0;
	__u32 inval_from=0,inval_to=0;

	PK_DBG("ngffs_setattr(0x%p,0x%p)\n",dentry,iattr);
	PK_DBG("ngsb=0x%p, inode=0x%p\n",ngsb,dentry->d_inode);

#ifdef NGFFS_HAS_SYSFILE
	if(dentry->d_inode->i_ino>1 && dentry->d_inode->i_ino<100) return -EPERM;
#endif

	PK_DBG("ia_valid = %i\n",iattr->ia_valid);

	down( &ngsb->global_lock );

	// obtain the
	if(dentry->d_inode->i_mode & S_IFDIR) {
		chunk=(struct generic_chunk *)ngffs_get_dirptr(ngsb->sb,dentry->d_inode->i_ino);
		meta=((struct dir_chunk *)chunk)->meta;
	} else {
		chunk=(struct generic_chunk *)ngffs_get_fileptr(ngsb->sb,dentry->d_inode->i_ino);
		meta=((struct file_chunk *)chunk)->meta;
	}
	oldofs=ngffs_get_cached_chunk_ofs(ngsb->sb,dentry->d_inode->i_ino);
	PK_DBG("oldofs=%u\n",oldofs);

	if(iattr->ia_valid & ATTR_MODE) {
		PK_DBG("ia_mode = %i\n",iattr->ia_mode);
		meta.mode=iattr->ia_mode;
		change++;
	}
	if(iattr->ia_valid & ATTR_UID) {
		PK_DBG("ia_uid = %i\n",iattr->ia_uid);
		meta.uid=iattr->ia_uid;
		change++;
	}
	if(iattr->ia_valid & ATTR_GID) {
		PK_DBG("ia_gid = %i\n",iattr->ia_gid);
		meta.gid=iattr->ia_gid;
		change++;
	}
	if(iattr->ia_valid & ATTR_ATIME) {
		PK_DBG("ia_atime = %ld\n",(unsigned long)iattr->ia_atime.tv_sec);
		PK_WARN("WARNING attempt to set access time, ngffs should be mounted NOATIME !\n");
		// access time ignored
	}
	if(iattr->ia_valid & ATTR_MTIME) {
		PK_DBG("ia_mtime = %ld\n",(unsigned long)iattr->ia_mtime.tv_sec);
		meta.mtime=iattr->ia_mtime.tv_sec;
		change++;
	}
	if(iattr->ia_valid & ATTR_CTIME) {
		PK_DBG("ia_ctime = %ld\n",(unsigned long)iattr->ia_ctime.tv_sec);
		meta.ctime=iattr->ia_ctime.tv_sec;
		change++;
	}
	PK_DBG("ia_attr_flags = %i\n",iattr->ia_attr_flags);


	if(change || (iattr->ia_valid & ATTR_SIZE )) {
		struct file_chunk *newchunk=0;
		struct file_chunk *oldchunk=0;
		int i;
		struct file_list_entry *filele=NULL;

		//    filele=ngffs_get_filele(dentry->d_inode->i_sb,dentry->d_inode->i_ino);
		//    cache=filele->cache;

		if(dentry->d_inode->i_mode & S_IFDIR) {
			((struct dir_chunk *)chunk)->meta=meta;
		} else {
			((struct file_chunk *)chunk)->meta=meta;
		}

		if(iattr->ia_valid & ATTR_SIZE) {
			// A
			PK_DBG("ia_size = %ld\n",(unsigned long)iattr->ia_size);

			if(dentry->d_inode->i_mode & S_IFDIR) {
				PK_WARN("WARNING set size called on directory..\n");
			} else {
				int curchunks=NGFFS_NUMCHUNKS(chunk->len);
				int newchunks=NGFFS_NUMCHUNKS(iattr->ia_size);

				PK_DBG("cursize=%d, newsize=%lld\n",chunk->len,iattr->ia_size);
				PK_DBG("curchunks=%i, newchunks=%i\n",curchunks,newchunks);

				if(iattr->ia_size>NGFFS_MAXFSIZE) {
					// better to quit with -ENOSPC ??
					PK_WARN("WARNING set size with size bigger than NGFFS_MAXFSIZE, truncating\n");
					iattr->ia_size=NGFFS_MAXFSIZE;
				}

				if(curchunks<newchunks) {	// START EXTENDING
					__u32 oldlen=chunk->len;
					__u32 left=iattr->ia_size - oldlen;
					struct file filp;
					void *page;
					__u32 page_size=(1<<PAGE_CACHE_SHIFT);
					__u32 writeofs=oldlen;
					__u32 writelen;

					PK_WARN("WARNING file expansion without writing untested\n");

					page=kmalloc(page_size,GFP_KERNEL);
					if(!page) { rv=-ENOMEM; goto out; }
					memset(page,0,page_size);

					filp.f_dentry=dentry;

					// loop while there is something to write
					while(left) {
						if(writeofs%page_size) {
							writelen=page_size-(writeofs%page_size);
							if(left<writelen) writelen=left;
						} else {
							writelen=(left<=page_size)?page_size:left;
						}

						rv=do_ngffs_commit_write (&filp, page, 0, writeofs, writeofs+writelen, 0);

						if(rv) {
							kfree(page);
							goto out;
						}

						left-=writelen;
						writeofs+=writelen;
					}

					// TODO: update any shorthands we may have set above
					kfree(page);

					// END EXTENDING
				} else if(curchunks>newchunks) {
					// decrease size, delete chunks    TODO inode file size!!
					// 1. change len in file chunk
					PK_DBG("old modification=%i\n",chunk->modification);
					newchunk=(struct file_chunk *)ngffs_resize_chunk((struct generic_chunk *)chunk,
						-((curchunks-newchunks)*sizeof(__u32)));
					if(newchunk) {
						newchunk->len=iattr->ia_size;

						PK_DBG("got new chunk at 0x%p, modification=%i\n",newchunk,newchunk->modification);

						invalidate_chunks=1;
						inval_from=newchunks; inval_to=curchunks;

					}
				} else if(iattr->ia_size!=chunk->len) {
					// same number of chunks, just change len and save file chunk
					chunk->len=iattr->ia_size;
				}
			}

			// /A
		}

		if(newchunk) {
			oldchunk=(struct file_chunk *)chunk;
			chunk=(struct generic_chunk *)newchunk;
		}

		chunk->modification++;

		block=ngffs_contig_available(dentry->d_inode->i_sb,CHUNK_SIZE(chunk,1));
		if(block==-1) { rv=-ENOSPC; goto out; }
		// TODO: what if contig_available did GC ??

		newofs=ngffs_put_chunk(dentry->d_inode->i_sb,chunk,block);
		if(!newofs) { PK_DBG("put_chunk err\n"); rv=-EIO; goto out; }
		//    chunk->valid=NGFFS_CHUNK_WRITTEN;

		rv=ngffs_invalidate_chunk(dentry->d_inode->i_sb,oldofs);
		if(rv) { PK_DBG("inval chunk err\n"); goto out_free_oldchunk; }

		//newofs=ngffs_get_chunk_ofs(dentry->d_inode->i_sb,dentry->d_inode->i_ino);
		//if(newofs==0) { PK_DBG("entry %ld notfound\n",dentry->d_inode->i_ino); rv=-EIO; goto out_free_oldchunk; }
		ngffs_set_cached_chunk_ofs(dentry->d_inode->i_sb,dentry->d_inode->i_ino,newofs);

		rv=ngffs_replace_chunk(dentry->d_inode->i_sb,chunk);
		if(!rv) { rv=-EIO; PK_DBG("replace chunk err\n"); goto out_free_oldchunk; }
		rv=0;

		ngffs_copy_from_meta(dentry->d_inode,&meta);

		if( (iattr->ia_valid & ATTR_SIZE) && !(dentry->d_inode->i_mode & S_IFDIR) ) {
			dentry->d_inode->i_size=iattr->ia_size;
		}

		PK_DBG("invalidating...\n");
		// 3. invalidate obsoleted chunks
		//     - get ptr to cache (only if indeed there is some invalidating to be done)
		if(inval_from<inval_to) {
			filele=ngffs_get_filele(dentry->d_inode->i_sb,dentry->d_inode->i_ino);
		}

		if (filele) {
			for(i=inval_from;i<inval_to;i++) {
				// 4. invalidate chunk[i]
				PK_DBG("invalidating chunk %i at 0x%08x\n",i,NGFFS_CACHE(filele,i)->offset);
				ngffs_invalidate_chunk(dentry->d_inode->i_sb,NGFFS_CACHE(filele,i)->offset);
				NGFFS_CACHE(filele,i)->modification=0;
			}
		}
		else {
			PK_DBG("Requested inode %i couldn't be found\n", dentry->d_inode->i_ino);
		}


out_free_oldchunk:
		if(oldchunk) kfree(oldchunk);
out:
		// TODO: check if this is indeed the right place to trunc
		if( (iattr->ia_valid & ATTR_SIZE) && !(dentry->d_inode->i_mode & S_IFDIR) && (dentry->d_inode->i_size<iattr->ia_size)) {
			vmtruncate(dentry->d_inode, iattr->ia_size);
		}
	}
	up( &ngsb->global_lock );

	PK_DBG("setattr done rv=%i\n",rv);
	return rv;
}

// we assume that pages always start on chunk boundaries (enforced in ngffs.h)
int ngffs_do_readpage_nolock(struct inode *inode, struct page *pg)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	__u32 pageofs = pg->index << PAGE_CACHE_SHIFT;
	__u32 chunk=pageofs/NGFFS_MAXCSIZE;
	__u32 startchunk=chunk;
	__u32 endchunks=chunk+(4096/NGFFS_MAXCSIZE);
	struct file_chunk *f = 0;
	struct file_list_entry *filele = 0;
	char dchunk[sizeof(struct data_chunk)+NGFFS_MAXCSIZE];
	int rv=0;

	down( &ngsb->global_lock );

	f = ngffs_get_fileptr(inode->i_sb,inode->i_ino);
	filele=ngffs_get_filele(inode->i_sb,f->entry_id);

	if(f==NULL) {
		PK_ERR("Unable to get file chunk %lu\n",inode->i_ino);
		up( &ngsb->global_lock );
		return -EIO;
	}

	PK_DBG("chunk %i, pageofs %i, endchunks=%i\n",chunk,pageofs,endchunks);
	PK_DBG("filele at %p, file at %p, inode_sb at %p, entry_id %u\n",filele,f,inode->i_sb,f->entry_id);
	if (!PageLocked(pg)) {
		/* PLEASECHECK Koen: PAGE_BUG has been removed as of 2.6.12 or so,
		 * no idea what it should be. */
		printk("page BUG for page at %p\n", pg);
		BUG();
	}

	for(;chunk<endchunks;chunk++) {
		if( (chunk*NGFFS_MAXCSIZE)<f->len ) {
			PK_DBG("getting chunk %i, cache-mod: %u, cache-ofs: %u\n",chunk,NGFFS_CACHE(filele,chunk)->modification,NGFFS_CACHE(filele,chunk)->offset);
			if(!ngffs_get_data_chunk(inode->i_sb,NGFFS_CACHE(filele,chunk)->offset,(struct data_chunk *)dchunk)) {
				rv=-EIO;
				goto readpage_fail;
			}

#if 0
			int i;
			char bla[2]="\0\0";
			char bla2[20]="\0";

			for(i=0;i<512;i++) {
				if(i && !(i%16)) {
					PK_DBG(" - %s\n",bla2);
					bla2[0]=0;
				}
				PK_DBG("%02x ",*(((unsigned char *)dchunk)+sizeof(struct data_chunk)+i));
				bla[0]=(*(((unsigned char *)dchunk)+sizeof(struct data_chunk)+i))>=32?
				       *(((unsigned char *)dchunk)+sizeof(struct data_chunk)+i):'.';
				strcat(bla2,bla);
			}
			PK_DBG(" - %s\n",bla2);
#endif
		} else {
			// zerofill ??
			goto readpage_succes;
		}
		// TODO: wcopy??
		if((pageofs+NGFFS_MAXCSIZE)>=f->len) {
			bcopy(dchunk+sizeof(struct data_chunk),page_address(pg)+((chunk-startchunk)*NGFFS_MAXCSIZE),
			      (f->len)%NGFFS_MAXCSIZE);
			pageofs+=(f->len-1)%NGFFS_MAXCSIZE;
			goto readpage_succes; // done, don't bother with remaining bytes (TODO: fill with zeroes?)
		} else {
			bcopy(dchunk+sizeof(struct data_chunk),page_address(pg)+((chunk-startchunk)*NGFFS_MAXCSIZE),
			      NGFFS_MAXCSIZE);
			pageofs+=NGFFS_MAXCSIZE;
		}
	}

readpage_succes:
	up( &ngsb->global_lock );
	SetPageUptodate(pg);
	ClearPageError(pg);
	flush_dcache_page(pg);
	kunmap(pg);
	PK_DBG("done\n");
	return 0;

readpage_fail:
	up( &ngsb->global_lock );
	ClearPageUptodate(pg);
	SetPageError(pg);
	kunmap(pg);
	PK_DBG("done\n");
	return rv;

}

int ngffs_readpage (struct file *filp, struct page *pg)
{
	//  struct inode *inode = filp->f_dentry->d_inode;

	PK_DBG("ngffs_readpage()\n");
	// do lock stuff?
	ngffs_do_readpage_nolock(filp->f_dentry->d_inode,pg);
	unlock_page(pg);
	// unlock
	PK_DBG("done\n");
	return 0;
}

int ngffs_prepare_write (struct file *filp, struct page *pg, unsigned start, unsigned end)
{
	struct inode *inode = filp->f_dentry->d_inode;

	//  PK_DBG("jffs2_prepare_write() nrpages %u at %u\n", inode->i_mapping->nrpages,pageofs);
	//  PK_DBG("from %u, to %u\n",start,end);

	/* Read in the page if it wasn't already present */
	if (!PageUptodate(pg) && (start || end < PAGE_SIZE)) {
		return ngffs_do_readpage_nolock(inode, pg);
	}

	return 0;
}

// wrapper to do_ngffs_commit_write for vfs layer
int ngffs_commit_write (struct file *filp, struct page *pg, unsigned start, unsigned end)
{
	return do_ngffs_commit_write( filp, page_address(pg), pg->index << PAGE_CACHE_SHIFT, start, end, 1);
}

// actual commit_write, used by:
// - vfs for actual writing
// - by setattr for extending when newsize>currsize
// If lock==0, the ngffs global lock will not be acquired (the caller must have set the lock, or problems WILL occur)
//
int do_ngffs_commit_write (struct file *filp, void *page, pgoff_t pgindex, unsigned start, unsigned end, int lock)
{
	struct inode *inode = filp->f_dentry->d_inode;
	struct ngffs_info *c = NGFFS_INFO(inode->i_sb);
	struct file_chunk *f;
	struct file_list_entry *filele;
	__u32 pageofs = pgindex;
	__u32 newsize = max_t(__u32, filp->f_dentry->d_inode->i_size, pgindex + end);
	__u32 startofs,endofs,ofs,oldofs,startchunk,endchunk,curchunk;
	__u32 numchunks;
	int blockidx;
	struct data_chunk **queue=0;
	int retval=0;
	int gc=0;			// number of times garbage collection done
	int invalidate=0;

	// TODO: is modification increased properly for each file/data chunk ?


	if(lock) down( &c->global_lock );

	// initialisation
	f = ngffs_get_fileptr(inode->i_sb,inode->i_ino);
	if(f==NULL) {
		PK_ERR("Unable to get file chunk %lu\n",inode->i_ino);
		if(lock) up( &c->global_lock );
		return -EIO;
	}
	filele=ngffs_get_filele(inode->i_sb,f->entry_id);

	PK_DBG("ino #%lu, page at 0x%lx, range %d-%d, nrpages %ld\n",
	       inode->i_ino, pgindex , start, end,
	       filp->f_dentry->d_inode->i_mapping->nrpages);
	PK_DBG("filele at %p, file at %p, inode_sb at %p, entry_id %u\n",filele,f,inode->i_sb,f->entry_id);
	PK_DBG("start=%u, end=%u\n",start,end);

	//dump_blocks(c,"commit start");

	// bail out if nothing to write
	if((end-start)==0) {
		if(lock) up( &c->global_lock );
		return 0;
	}

	// some shorthands
	// offset in the file from where we start and end writing:
	startofs=pageofs+start;
	endofs=pageofs+end;
	// first & last chunk we need to write:
	startchunk=(startofs>f->len)
		   ?(f->len/NGFFS_MAXCSIZE)
		   :(startofs/NGFFS_MAXCSIZE);
	endchunk=(endofs-1)/NGFFS_MAXCSIZE;

	PK_DBG("startofs=%i, endofs=%i\n",startofs,endofs);

	// prevent files from getting too big
	if(startofs>NGFFS_MAXFSIZE || endofs>NGFFS_MAXFSIZE)
	{
		if(lock) up( &c->global_lock );
		return -ENOSPC;
	}

	// now get number of chunks to write (to alloc chunk queue)
	numchunks=(endchunk-startchunk)+1;

	PK_DBG("startchunk=%i, endchunk=%i, numchunks=%i\n",startchunk,endchunk,numchunks);

	// alloc memory for temporary storage of modified data chunks
	queue=kmalloc(sizeof(struct data_chunk *)*numchunks,GFP_KERNEL);
	if(!queue) {
		PK_ERR("Unable to alloc mem for write queue\n");
		retval=-ENOMEM;
		goto write_done;
	}
	queue[0]=(struct data_chunk *)kmalloc((sizeof(struct data_chunk)+NGFFS_MAXCSIZE)*numchunks,GFP_KERNEL);
	if(!queue[0]) {
		PK_ERR("Unable to alloc mem for write queue entries\n");
		retval=-ENOMEM;
		goto write_done;
	}
	for(curchunk=0;curchunk<numchunks;curchunk++) {
		queue[curchunk]=(struct data_chunk *)(((char *)queue[0])+(curchunk*(sizeof(struct data_chunk)+NGFFS_MAXCSIZE)));
	}
	curchunk=0;		// is idx, real current chunk = startchunk+curchunk

	// setup first chunk (do padding if necesary)
	if( startofs > f->len ) {
		PK_DBG("startofs > f->len\n");
		// pad with zeroes
		if( (startofs/NGFFS_MAXCSIZE) == (f->len/NGFFS_MAXCSIZE) ) {
			// we start writing inside the current last chunk
			// need padding in between
			PK_DBG("write case 1: inside current last chunk, need padding inbetween\n");

			// 1. load current last
			//	if( f->len && (((startchunk+curchunk)*NGFFS_MAXCSIZE) < (f->len-1) )) {
			if(NGFFS_CACHE(filele,startchunk+curchunk)->modification) {
				if(!ngffs_get_data_chunk(inode->i_sb,NGFFS_CACHE(filele,startchunk+curchunk)->offset,queue[curchunk])) {
					retval=-EIO;
					goto write_done;
				}
				queue[curchunk]->modification++;
			} else {
				queue[curchunk]->type=NGFFS_DATA_CHUNK;
				queue[curchunk]->valid=-1;
				SET_VALID(queue[curchunk]);
				RESET_WRITTEN(queue[curchunk]);
				SET_CRC(queue[curchunk]);
				queue[curchunk]->modification=1;
				queue[curchunk]->entry_id=f->entry_id;
				queue[curchunk]->seq_id=startchunk+curchunk;
				NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
			}
			// 2. pad with zeroes till end
			for(ofs=f->len%NGFFS_MAXCSIZE;ofs<NGFFS_MAXCSIZE;ofs++) {
				*(((char *)queue[curchunk])+sizeof(struct data_chunk)+ofs)=0;
			}
			// now curchunk is still 0, points at first chunk to write
		} else {
			// we start writing after current last chunk,
			// pad current chunk with zeroes
			PK_DBG("write case 2: after current last chunk, pad current with zeroes\n");

			// 1. load first, pad
			//	if( f->len && ( ((startchunk+curchunk)*NGFFS_MAXCSIZE) < (f->len-1)) ) {
			if(NGFFS_CACHE(filele,startchunk+curchunk)->modification) {
				// chunk exists
				if(!ngffs_get_data_chunk(inode->i_sb,NGFFS_CACHE(filele,startchunk+curchunk)->offset,queue[curchunk])) {
					retval=-EIO;
					goto write_done;
				}
				queue[curchunk]->modification++;
			} else {
				// chunk does not exist
				queue[curchunk]->type=NGFFS_DATA_CHUNK;
				queue[curchunk]->valid=-1;
				SET_VALID(queue[curchunk]);
				RESET_WRITTEN(queue[curchunk]);
				SET_CRC(queue[curchunk]);
				queue[curchunk]->modification=1;
				queue[curchunk]->entry_id=f->entry_id;
				queue[curchunk]->seq_id=startchunk+curchunk;
				NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
			}
			for(ofs=(f->len)%NGFFS_MAXCSIZE;ofs<NGFFS_MAXCSIZE;ofs++) {
				*(((char *)queue[curchunk])+sizeof(struct data_chunk)+ofs)=0;
			}
			curchunk++;	// so curchunk==1

			// 2. while(not done padding)
			// 3. add chunk with all zeroes
			while( (curchunk+startchunk)<(startofs/NGFFS_MAXCSIZE) ) {
				for(ofs=0;ofs<NGFFS_MAXCSIZE;ofs++) {
					*(((char *)queue[curchunk])+sizeof(struct data_chunk)+ofs)=0;
				}
				queue[curchunk]->type=NGFFS_DATA_CHUNK;
				queue[curchunk]->valid=-1;
				SET_VALID(queue[curchunk]);
				RESET_WRITTEN(queue[curchunk]);
				SET_CRC(queue[curchunk]);
				queue[curchunk]->modification=1;
				queue[curchunk]->entry_id=f->entry_id;
				queue[curchunk]->seq_id=startchunk+curchunk;
				NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
				curchunk++;
			}
			// 5. create last (??)
			// 6. pad with zeroes till end
			for(ofs=0;ofs<NGFFS_MAXCSIZE;ofs++)
				*(((char *)queue[curchunk])+sizeof(struct data_chunk)+ofs)=0;
			queue[curchunk]->type=NGFFS_DATA_CHUNK;
			queue[curchunk]->valid=-1;
			SET_VALID(queue[curchunk]);
			RESET_WRITTEN(queue[curchunk]);
			SET_CRC(queue[curchunk]);
			queue[curchunk]->modification=1;
			queue[curchunk]->entry_id=f->entry_id;
			queue[curchunk]->seq_id=startchunk+curchunk;
			NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
			// now curchunk is the chunk in which startofs falls
		}
	} else {
		PK_DBG("startofs <= f->len\n");
		//	if( ((startchunk+curchunk)*NGFFS_MAXCSIZE) < f->len) {
		if(NGFFS_CACHE(filele,startchunk+curchunk)->modification) {
			if(!ngffs_get_data_chunk(inode->i_sb,NGFFS_CACHE(filele,startchunk+curchunk)->offset,queue[curchunk])) {
				retval=-EIO;
				goto write_done;
			}
			queue[curchunk]->modification++;
		} else {
			queue[curchunk]->type=NGFFS_DATA_CHUNK;
			queue[curchunk]->valid=-1;
			SET_VALID(queue[curchunk]);
			RESET_WRITTEN(queue[curchunk]);
			SET_CRC(queue[curchunk]);
			queue[curchunk]->modification=1;
			queue[curchunk]->entry_id=f->entry_id;
			queue[curchunk]->seq_id=startchunk+curchunk;
			NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
		}
	} // first chunk setup & padded if nec., curchunk==chunk with startofs

	ofs=startofs;
	while((curchunk+startchunk)<=endchunk) {

		// copy data to chunk
		if(ofs%NGFFS_MAXCSIZE) {			// not on chunk boundary so first chunk & unaligned
			PK_DBG("ofs=%u, chunk=%u, first&unaligned\n",ofs,curchunk);
			// bcopy(src,dst,len)
			bcopy(	page+(ofs-pageofs),
				  ((char *)queue[curchunk])+sizeof(struct data_chunk)+(ofs%NGFFS_MAXCSIZE) ,
				  ((endofs/NGFFS_MAXCSIZE)==(startchunk+curchunk))
				  ?(endofs-startofs)
				  :(NGFFS_MAXCSIZE-(ofs%NGFFS_MAXCSIZE)-1) );

			ofs+=((endofs/NGFFS_MAXCSIZE)==(startchunk+curchunk))
			     ?(endofs-startofs)
			     :(NGFFS_MAXCSIZE-(ofs%NGFFS_MAXCSIZE)) ;
			queue[curchunk]->len=((endofs/NGFFS_MAXCSIZE)==(startchunk+curchunk))
					     ?(endofs%NGFFS_MAXCSIZE)
					     :NGFFS_MAXCSIZE;
		} else if((ofs+NGFFS_MAXCSIZE)>=endofs) {	// last chunk
			PK_DBG("ofs=%u, chunk=%u, last\n",ofs,curchunk);
			bcopy(page+(ofs-pageofs), ((char *)queue[curchunk])+sizeof(struct data_chunk) ,
			      (endofs-ofs));
			queue[curchunk]->len=(endofs-ofs);
			ofs+=queue[curchunk]->len;
		} else {					// normal chunk
			PK_DBG("ofs=%u, chunk=%u, normal\n",ofs,curchunk);
			bcopy(page+(ofs-pageofs), ((char *)queue[curchunk])+sizeof(struct data_chunk) ,NGFFS_MAXCSIZE);
			queue[curchunk]->len=NGFFS_MAXCSIZE;
			ofs+=NGFFS_MAXCSIZE;
		}

		// advance to next chunk
		curchunk++;
		if((curchunk+startchunk)<=endchunk) {
			//      if(f->len && (curchunk+startchunk)<((f->len-1)/NGFFS_MAXCSIZE)) {
			if(NGFFS_CACHE(filele,startchunk+curchunk)->modification) {
				// get next chunk
				PK_DBG("loading next chunk\n");
				if(!ngffs_get_data_chunk(inode->i_sb,NGFFS_CACHE(filele,startchunk+curchunk)->offset,queue[curchunk])) {
					retval=-EIO;
					goto write_done;
				}
				queue[curchunk]->modification++;
			} else {
				// create next chunk
				PK_DBG("creating next chunk\n");
				queue[curchunk]->type=NGFFS_DATA_CHUNK;
				queue[curchunk]->valid=-1;
				SET_VALID(queue[curchunk]);
				RESET_WRITTEN(queue[curchunk]);
				SET_CRC(queue[curchunk]);
				queue[curchunk]->modification=1;
				queue[curchunk]->entry_id=f->entry_id;
				queue[curchunk]->seq_id=startchunk+curchunk;
				NGFFS_CACHE(filele,startchunk+curchunk)->modification=0;
			}
		}
	}
	// TODO start lock
	//

	PK_DBG("starting free space check\n");
recheck:
	/* we possibly arrive here from garbage collection, so
	file/dir chunk ptrs might be wrong.. */
	f = ngffs_get_fileptr(inode->i_sb,inode->i_ino);
	// check if there is enough free space
	curchunk=0; blockidx=0;
	NGFFS_BLOCK(c,blockidx)->reserved=NGFFS_BLOCK(c,blockidx)->freespace;
	// first check for all data chunks
	while((curchunk+startchunk)<=endchunk) {
		// if current block not valid & active, get first valid&active next one
		//while( (blockidx<c->blocks) && ( NGFFS_BLOCK(c,blockidx)->header.valid!=NGFFS_CHUNK_WRITTEN ||
		while( (blockidx<c->blocks) && ( !IS_WRITTEN(&(NGFFS_BLOCK(c,blockidx)->header)) ||
			!IS_VALID(&(NGFFS_BLOCK(c,blockidx)->header)) ||
			NGFFS_BLOCK(c,blockidx)->header.activated ) ) {
			if (++blockidx>=c->blocks) {
				PK_DBG("out of blocks, gonna try garbage collect\n");
				goto collect;
			}
			NGFFS_BLOCK(c,blockidx)->reserved=NGFFS_BLOCK(c,blockidx)->freespace;
			PK_DBG("iterating block=%i reserved=%u\n",blockidx,NGFFS_BLOCK(c,blockidx)->reserved);
			//dump_blocks(c,"block iterator");
		}
		//dump_block(c,"chunk iterator",blockidx);
		// if no more valid&active blocks, do garbage collection to free space
		if(blockidx>=c->blocks) {
			// TODO: garbage collect
			// TODO: restart free space checking
collect:
			PK_DBG("no free space, garbage collection\n");
			if(gc>c->blocks) {
				// stop garbage collecting, we tried x times and it failed
				PK_DBG("gc count exceeded max gc count\n");
				retval=-ENOSPC;
				goto write_done;
			}
			// do garbage collection, quit on error with 'no space'
			if(ngffs_collect_garbage(inode->i_sb)) {
				retval=-ENOSPC;
				goto write_done;
			}
			gc++;	// increase garbage collect count
			PK_DBG("gc done, now at count %i\n",gc);
			goto recheck;
		}
		// check if current chunk fits, but reserved field is unsigned so we check if
		// it wrapped, new reserved should be < than current reserved
		if( (NGFFS_BLOCK(c,blockidx)->reserved - CHUNK_SIZE(queue[curchunk],1))>NGFFS_BLOCK(c,blockidx)->reserved ) {
			if (++blockidx>=c->blocks) {
				PK_DBG("blockidx too large, gonna try garbage collect\n");
				goto collect;
			}
			NGFFS_BLOCK(c,blockidx)->reserved = NGFFS_BLOCK(c,blockidx)->freespace;
			PK_DBG("chunk didn't fit, new block %i\n",blockidx);
		} else {
			NGFFS_BLOCK(c,blockidx)->reserved -= CHUNK_SIZE(queue[curchunk],1);
			PK_DBG("chunk fits, new reserved is %u\n",NGFFS_BLOCK(c,blockidx)->reserved);
			curchunk++;
		}
		dump_blocks(c,"after fit");
	}

	// can we fit the file chunk in?
	for(blockidx=0;blockidx<c->blocks;blockidx++) {
		//dump_block(c,"file chunk fit",blockidx);
		//{ int dd=1000000; while(--dd) { dd*=2; dd=dd/(4-2); } }
		if( (NGFFS_BLOCK(c,blockidx)->reserved >= CHUNK_SIZE(f,1)) &&
		    IS_WRITTEN(&(NGFFS_BLOCK(c,blockidx)->header)) &&
		    IS_VALID(&(NGFFS_BLOCK(c,blockidx)->header)) &&
		    !(NGFFS_BLOCK(c,blockidx)->header.activated)) {
			//NGFFS_BLOCK(c,blockidx)->header.valid==NGFFS_CHUNK_WRITTEN && !(NGFFS_BLOCK(c,blockidx)->header.activated)) {
			goto freespace_ok;
		}
	}
	if(gc>c->blocks) {
		// stop garbage collecting, we tried x times and it failed
		retval=-ENOSPC;
		goto write_done;
	}

	// do garbage collection, quit on error with 'no space'
	if(ngffs_collect_garbage(inode->i_sb)) {
		retval=-ENOSPC;
		goto write_done;
	}
	gc++;	// increase garbage collect count
	goto recheck;

freespace_ok:
	dump_blocks(c,"freespace_ok");

	// write data chunks
	curchunk=0; blockidx=0;
	while((curchunk+startchunk)<=endchunk) {
		//while( (blockidx<c->blocks) && ( NGFFS_BLOCK(c,blockidx)->header.valid!=NGFFS_CHUNK_WRITTEN ||
		while( (blockidx<c->blocks) && ( !IS_VALID(&(NGFFS_BLOCK(c,blockidx)->header))
			|| !IS_WRITTEN(&(NGFFS_BLOCK(c,blockidx)->header))
			|| NGFFS_BLOCK(c,blockidx)->header.activated ) ) {
			blockidx++;
			NGFFS_BLOCK(c,blockidx)->reserved=NGFFS_BLOCK(c,blockidx)->freespace;
		}
		if(blockidx>=c->blocks) {
			// this should not happen, unless some other process
			// doesn't respect the write-lock
			retval=-EIO;
			goto write_done;
		}
		// again a check whether unsigned - unsigned < 0, new free space should be smaller than current
		if( (NGFFS_BLOCK(c,blockidx)->freespace - CHUNK_SIZE(queue[curchunk],1)) < NGFFS_BLOCK(c,blockidx)->freespace) {
			dump_block(c,"befor write",blockidx);
			//PK_DBG("writing curchunk=%u, blockidx=%i, ",curchunk,blockidx);
			//PK_DBG("ofs=%u, ",ofs);
			oldofs=NGFFS_CACHE(filele,startchunk+curchunk)->offset;

			PK_DBG("cached modification = %u\n",NGFFS_CACHE(filele,startchunk+curchunk)->modification);
			if(!NGFFS_CACHE(filele,startchunk+curchunk)->modification) {
				invalidate=0;
				NGFFS_CACHE(filele,startchunk+curchunk)->modification=1;
			} else {
				invalidate=1;
			}

			if(!(ofs=ngffs_put_chunk(inode->i_sb,(struct generic_chunk *)queue[curchunk],blockidx))) {
				PK_DBG("put_chunk failed for new chunk\n");
				retval=-EIO;
				goto write_done;
			}
			//      queue[curchunk]->valid=NGFFS_CHUNK_WRITTEN;
			PK_DBG("written to offs %u\n",ofs);

			// and invalidate old chunk
			if(invalidate)
				ngffs_invalidate_chunk(inode->i_sb,oldofs);

			// update data chunk offset cache
			NGFFS_CACHE(filele,startchunk+curchunk)->offset=ofs;
			NGFFS_CACHE(filele,startchunk+curchunk)->modification=queue[curchunk]->modification;
			PK_DBG("updated cache, cache[%i]=%u\n",startchunk+curchunk,ofs);
			//      NGFFS_BLOCK(c,blockidx)->freespace -= CHUNK_SIZE(queue[curchunk],1);
			PK_DBG("---after write (size=%i)",queue[curchunk]->len);
			curchunk++;
		} else {
			//PK_DBG("curchunk=%u won't fit in blockidx=%i\n",curchunk,blockidx);
			blockidx++;
		}

	}

	// write updated file chunk
	f->len=newsize;
	f->modification++;
	for(blockidx=0;blockidx<c->blocks;blockidx++) {
		dump_block(c,"file chunk write srch",blockidx);
		{ int dd=1000000; while(--dd) { dd*=2; dd=dd/(4-2); } }
		if( (NGFFS_BLOCK(c,blockidx)->freespace >= CHUNK_SIZE(f,1)) &&
		    //NGFFS_BLOCK(c,blockidx)->header.valid==NGFFS_CHUNK_WRITTEN &&
		    IS_VALID(&(NGFFS_BLOCK(c,blockidx)->header)) &&
		    IS_WRITTEN(&(NGFFS_BLOCK(c,blockidx)->header)) &&
		    !(NGFFS_BLOCK(c,blockidx)->header.activated)) {
			goto freespace_ok2;
		}
	}
	
	PK_DBG("can't find freespace, blockidx:%d c->blocks:%d chunksize:%u\n", blockidx, c->blocks, CHUNK_SIZE(f,1));
	PK_DBG("Garbage collect, is this possible at this stage, i think the check before has to detect it sooner\n");
	goto collect;	
freespace_ok2:
	if(!(ofs=ngffs_put_chunk(inode->i_sb,(struct generic_chunk *)f,blockidx))) {
		retval=-EIO; goto write_done;
	}
	PK_DBG("written file chunk to offs %u\n",ofs);
	//  f->valid=NGFFS_CHUNK_WRITTEN;

	// invalidate old file chunk
	ngffs_invalidate_chunk(inode->i_sb,ngffs_get_cached_chunk_ofs(inode->i_sb,inode->i_ino));

	// and update offset
	ngffs_set_cached_chunk_ofs(inode->i_sb,inode->i_ino,ofs);

	dump_block(c,"file chunk write subst",blockidx);

	// wrap up, update cache values etc

	inode->i_size=newsize;
	retval=0;
	goto write_succes;

write_done:	// exit point for errors only

write_succes:	// generic exit point
	if(lock) up( &c->global_lock );
	if(queue) {
		if(queue[0]) kfree(queue[0]);
		kfree(queue);
	}
	PK_DBG("commit_write retval=%i\n",retval);
	PK_DBG("done\n");
	return retval;
}

/* EOF */
