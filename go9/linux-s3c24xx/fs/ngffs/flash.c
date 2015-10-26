/* fs/ngffs/flash.c
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
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <asm/uaccess.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include "crc.h"
#include "kern.h"
#include "chunk.h"

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC
//#define PK_DBG PK_DBG_NONE

extern void ngffs_create_root(struct ngffs_info * ngsb , struct ngffs_block *block);

static u_long erase_done_flags=0;
static spinlock_t erase_flag_lock = SPIN_LOCK_UNLOCKED;

void hexdump(char *buff)
{
	int i;
	char bla[2]="\0\0";
	char bla2[20]="\0";
	mdelay(500);
	for(i=0;i<512;i++) {
		if(i && !(i%16)) {
			printk(" - %s\n",bla2);
			bla2[0]=0;
		}
		printk("%02x ",*( ((unsigned char*)buff)+i ));
		bla[0]=(*( ((unsigned char*)buff)+i))>=32?
		       *( ((unsigned char*)buff)+i):'.';
		strcat(bla2,bla);
	}
	printk(" - %s\n",bla2);
	mdelay(500);
}

// TODO what if non-word-aligned read/write?? will mtd handle this for us??
int ngffs_write(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len)
{
	int retlen;
	int ret;

	PK_DBG("write at ofs %u, len=%u\n",ofs,len);
#if 0
	//    if( ((struct generic_chunk *)buff)->type==NGFFS_DATA_CHUNK ) {
	hexdump(buff);
	//    }
#endif


	ret=mtd->write(mtd,ofs+CONFIG_NGFFS_START_OFS,len,&retlen,buff);
	if(ret) {
		PK_ERR("Write error1 at 0x%08x: %d\n", ofs, ret);
		return ret;
	}
	if (retlen != len) {
		PK_ERR("Short write: 0x%x bytes at 0x%08x instead of requested %x\n",
		       retlen, ofs, len);
		return -EIO;
	}
	return 0;
}

int ngffs_absolute_read(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len)
{
	int retlen;
	int ret;

	PK_DBG("read at ofs %u, len=%u\n",ofs-CONFIG_NGFFS_START_OFS,len);

	ret=mtd->read(mtd,ofs,len,&retlen,buff);
	if (ret) {
		PK_ERR("ngffs_read: Read error at 0x%08x: %d\n", ofs, ret);
		return ret;
	}
	if (retlen != len) {
		PK_ERR(KERN_NOTICE "Short read: 0x%x bytes at 0x%08x instead of requested %x\n",
		       retlen, ofs, len);
		return -EIO;
	}
#if 0
	//    if( ((struct generic_chunk *)buff)->type==NGFFS_DATA_CHUNK ) {
	hexdump(buff);
	//    }
#endif
	return 0;
}

int ngffs_read(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len)
{
	return ngffs_absolute_read(mtd,ofs+CONFIG_NGFFS_START_OFS,buff,len);
}

void ngffs_erase_callback(struct erase_info *instr)
{
	if(instr->state != MTD_ERASE_DONE) {
		PK_DBG("Erase completed, but state is not MTD_ERASE_DONE!\n");
	} else {
		unsigned long flags;
		spin_lock_irqsave(&erase_flag_lock, flags);
		erase_done_flags|=instr->priv;
		spin_unlock_irqrestore(&erase_flag_lock, flags);
	}

	kfree(instr);
}

int ngffs_erase_flash(struct mtd_info *mtd,int block,u_long mask)
{
	struct erase_info *instr;
	int ret;
	unsigned long flags;

	instr = kmalloc(sizeof(struct erase_info), GFP_KERNEL);

	memset(instr, 0, sizeof(instr));

	instr->mtd=mtd;
	instr->addr=(mtd->erasesize*block)+CONFIG_NGFFS_START_OFS;
	instr->len=mtd->erasesize;
	instr->callback=ngffs_erase_callback;
	instr->priv=mask;

	spin_lock_irqsave(&erase_flag_lock, flags);
	erase_done_flags&=~(mask);
	spin_unlock_irqrestore(&erase_flag_lock, flags);
eagain:
	ret = mtd->erase(mtd, instr);
	if (!ret) {
		return 0;
	}
	if(ret==-EROFS) {
		PK_ERR("ngffs: trying to erase read only sector %i\n",block);
		return -EROFS;
	}
	if(ret==-EAGAIN) {
		//schedule();
		goto eagain;
	}
	return ret;
}

int ngffs_erase_ready(u_long mask)
{
	return erase_done_flags&mask;
}

// replaces dir chunk in list for the entry_id only if dir->modification is
// larger (TODO: wrap)
//
// side-effect: invalidates the oldest of the chunks (cleanup), to keep the
// filesystem tidy when power-downs/reboots result in more than one valid
// chunk for the same entry_id
//
// returns 1 if replaced or older, 0 if not
int ngffs_replace_older_dir(struct ngffs_info *ngsb,struct dir_chunk *dir,__u32 ofs)
{
	struct list_head *curr;

	PK_DBG("entry_id %u\n",dir->entry_id);

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dirle=list_entry(curr, struct dir_list_entry, list);
		if(dirle->chunk->entry_id==dir->entry_id) {
			PK_DBG("old mod: %u, new mod: %u\n",dirle->chunk->modification,dir->modification);
			if(dirle->chunk->modification <= dir->modification) { // added for gc: equals
				// invalidate older chunk, but only if indeed newer
				if(dirle->chunk->modification != dir->modification) {
					ngffs_invalidate_chunk(ngsb->sb,dirle->ofs); // (cleanup)
				}
				// replace
				kfree(dirle->chunk);
				dirle->chunk=dir;
				dirle->ofs=ofs;
				return 1;
			} else {
				// older
				// invalidate this chunk
				ngffs_invalidate_chunk(ngsb->sb,ofs);  // (cleanup)
				kfree(dir);
				return 1;
			}
		}
	}
	return 0;
}

// replaces dir chunk in list for the entry_id only if dir->modification is
// larger (TODO: wrap)
//
// side-effect: invalidates the oldest of the chunks (cleanup), to keep the
// filesystem tidy when power-downs/reboots result in more than one valid
// chunk for the same entry_id
//
// returns 1 if replaced or older, 0 if not (ie. insert!)
int ngffs_replace_older_file(struct ngffs_info *ngsb,struct file_chunk *file,__u32 ofs)
{
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *filele=list_entry(curr, struct file_list_entry, list);
		if(filele->chunk->entry_id==file->entry_id) {
			if(filele->chunk->modification <= file->modification) { // added for gc: equals
				// invalidate older chunk, but only if indeed newer
				if(filele->chunk->modification != file->modification) {
					ngffs_invalidate_chunk(ngsb->sb,filele->ofs);	// (cleanup)
				}
				// drop old chunk
				kfree(filele->chunk);
				filele->chunk=file;
				filele->ofs=ofs;
				return 1;
			} else {
				// invalidate this chunk
				ngffs_invalidate_chunk(ngsb->sb,ofs);  // (cleanup)
				kfree(file);
				return 1;
			}
		}
	}
	return 0;
}

int ngffs_scan_block(struct ngffs_info *ngsb, int b)
{
	struct mtd_info *mtd=ngsb->mtd;
	__u32 currofs = EMPTY_BYTES + NGFFS_BLOCK(ngsb,b)->offset;
	__u32 maxofs= NGFFS_BLOCK(ngsb,b)->offset+NGFFS_BLOCK(ngsb,b)->size;
	__u32 freespace=NGFFS_BLOCK(ngsb,b)->size-EMPTY_BYTES;
	__u32 currsize;
	int done=0,firstchunk=0;
	int mod_init;
	struct generic_chunk prebuff;
	struct dir_list_entry *dir;
	struct file_list_entry *file;
	struct dir_chunk *tmpdir;
	struct file_chunk *tmpfile;
	int rv;

	while(!done) {
		// TODO: fix -1 to return valid error codes
		if((rv=ngffs_read(mtd,currofs,(u_char*)&prebuff,sizeof(struct generic_chunk)))) {
			PK_DBG("unable to read for block %i\n",b);
			return rv;
		}


		if(prebuff.type==NGFFS_UNUSED) {
			// this is the first chunk we read, we have at least one
			// chunk, so if it is not a valid type -> err
			if(!firstchunk) {
				return -EIO;
			}
			done=1;
		} else {
			currsize=CHUNK_SIZE(&prebuff,0);
			PK_DBG("inloop> currsize=%u, ofs=%u\n",currsize,currofs);
			if(currsize==-1) return -EIO;	// TODO: invalidate block/fs ?
			freespace-=currsize;
			PK_DBG("inloop> |-> freespace=%u, valid=%x, type=%i\n",freespace,prebuff.valid,prebuff.type);
			//      if(prebuff.valid==NGFFS_CHUNK_WRITTEN) {
			if(IS_WRITTEN(&prebuff)) {
				switch(prebuff.type) {
				case NGFFS_DIR_CHUNK:
					tmpdir=kmalloc(CHUNK_SIZE(&prebuff,0),GFP_KERNEL);
					if(tmpdir==NULL) {
						PK_DBG("ngffs_scan_block(sb,%d): unable to kmalloc dir chunk\n",b);
						return -ENOMEM;
					}
					if((rv=ngffs_read(mtd,currofs,(u_char *)(tmpdir),CHUNK_SIZE(&prebuff,0)))) {
						PK_DBG("ngffs_scan_block(sb,%d): unable to read dir chunk\n",b);
						return rv;
					}
					if(!ngffs_replace_older_dir(ngsb,tmpdir,currofs)) {
						dir=kmalloc(sizeof(struct dir_list_entry),GFP_KERNEL);
						if(dir==NULL) {
							PK_DBG("ngffs_scan_block(sb,%d): unable to kmalloc dir_list_entry\n",b);
							return -ENOMEM;
						}
						dir->ofs=currofs;
						dir->chunk=tmpdir;

						CHECK_GLB_LOCK(ngsb->global_lock);

						list_add_tail(&dir->list,&ngsb->dirs);
					}
					if(tmpdir->entry_id > ngsb->max_entry_id) ngsb->max_entry_id=tmpdir->entry_id;
					PK_DBG("ngffs_scan_block(sb,%d): read dir inode %d\n",b,tmpdir->entry_id);
					break;
				case NGFFS_FILE_CHUNK:
					// alloc mem and read the file chunk
					tmpfile=kmalloc(CHUNK_SIZE(&prebuff,0),GFP_KERNEL);
					if(tmpfile==NULL) {
						PK_ERR("ngffs_scan_block(sb,%d): unable to kmalloc file chunk\n",b);
						return -ENOMEM;
					}
					if((rv=ngffs_read(mtd,currofs,(u_char *)(tmpfile),CHUNK_SIZE(&prebuff,0)))) {
						PK_ERR("ngffs_scan_block(sb,%d): unable to read file chunk\n",b);
						return rv;
					}
					// is this a later version (this.modified > cached.modified?)
					if(!ngffs_replace_older_file(ngsb,tmpfile,currofs)) {
						// no, alloc new list head
						file=kmalloc(sizeof(struct file_list_entry),GFP_KERNEL);
						if(file==NULL) {
							PK_ERR("ngffs_scan_block(sb,%d): unable to kmalloc file_list_entry\n",b);
							return -ENOMEM;
						}
						file->cache=kmalloc(NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE)*sizeof(struct ngffs_data_cache),
							GFP_KERNEL);
						if(file->cache==NULL) {
							kfree(file);
							PK_ERR("ngffs_scan_block(sb,%d): unable to kmalloc file_list_entry.chunkofs\n",b);
							return -ENOMEM;
						}
#ifdef NGFFS_PARANOIA
						memset(file->cache, 0xef, NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE)*sizeof(struct ngffs_data_cache));
#endif // NGFFS_PARANOIA
						for(mod_init=0;mod_init<NGFFS_NUMCHUNKS(NGFFS_MAXFSIZE);mod_init++) {
							NGFFS_CACHE(file,mod_init)->modification=0;
						}
						file->ofs=currofs;
						file->chunk=tmpfile;

						CHECK_GLB_LOCK(ngsb->global_lock);

						list_add_tail(&file->list,&ngsb->files);
					}
					if(tmpfile->entry_id > ngsb->max_entry_id) ngsb->max_entry_id=tmpfile->entry_id;
					PK_DBG("ngffs_scan_block(sb,%d): read file inode %d (len=%d)\n",b,tmpfile->entry_id,
					       tmpfile->len);
					break;
				case NGFFS_HEADER_CHUNK:
				case NGFFS_DATA_CHUNK:
					// ignore all, assume CHUNK_SIZE already caught
					break;
				default:
					// invalid types
					PK_WARN("invalid type");
					return -EIO;
				}
			}
			currofs+=currsize;
			if(currofs>maxofs) {
				PK_WARN("Warning, chunk went outside erase block\n");
				return -EIO;
			}
			if(currofs==maxofs) done=1;	// end-of-block
			if(currofs>(maxofs-sizeof(struct generic_chunk))) done=1;	// end-of-block cause no space for next chunk
		}
		firstchunk=1;
	}
	NGFFS_BLOCK(ngsb,b)->freespace=freespace;
	PK_DBG("blocks[%d].freespace=%d\n",b,freespace);
	return 0;
}

/*
 * Does the second pass of flash scanning, storing offsets for
 * data chunks in the filelisthead's chunk offset cache
 */
int ngffs_build_data_cache(struct ngffs_info * ngsb)
{
	struct mtd_info *mtd=ngsb->mtd;
	int block;
	int ofs;
	struct generic_chunk prebuff;
	struct data_chunk data;

	mtd=ngsb->mtd;

	for(block=0;block<ngsb->blocks;block++) {
		PK_DBG("second pass scan block %i\n",block);
		if( (IS_WRITTEN(&(NGFFS_BLOCK(ngsb,block)->header))) && !(NGFFS_BLOCK(ngsb,block)->header.activated)) {
			ofs=EMPTY_BYTES+sizeof(struct header_chunk);
			while(ofs<NGFFS_BLOCK(ngsb,block)->size) {
				if(ngffs_read(mtd,NGFFS_BLOCK(ngsb,block)->offset+ofs,(u_char*)&prebuff,sizeof(struct generic_chunk))) {
					PK_WARN("ngffs_build_data_cache() unable to read from block %i\n",block);
				}
				PK_DBG("at %u found type %i\n",NGFFS_BLOCK(ngsb,block)->offset+ofs,prebuff.type);
				switch(prebuff.type) {
				case NGFFS_UNUSED: ofs=NGFFS_BLOCK(ngsb,block)->size; break; // abort while loop
				case NGFFS_DATA_CHUNK:
					if(IS_VALID(&prebuff) && IS_WRITTEN(&prebuff)) {          // only read&cache written chunks
						if(ngffs_read(mtd,NGFFS_BLOCK(ngsb,block)->offset+ofs,(u_char*)&data,sizeof(struct data_chunk))) {
							PK_WARN("ngffs_build_data_cache() unable to read from block %i\n",block);
							return -EIO;
						}
						ngffs_cache_data_chunk(ngsb,data.entry_id,data.seq_id,data.modification,NGFFS_BLOCK(ngsb,block)->offset+ofs);
						PK_DBG("data with entry_id=%u, seq_id=%u, mod=%u\n",data.entry_id,data.seq_id,data.modification);
					}
					ofs+=CHUNK_SIZE(&prebuff,0);
					break;
				default:
					ofs+=CHUNK_SIZE(&prebuff,0);
					break;
				}
				// abort loop if there is no space for a next chunk
				if(ofs>(NGFFS_BLOCK(ngsb,block)->size-sizeof(struct generic_chunk))) ofs=NGFFS_BLOCK(ngsb,block)->size;
			}
		} else {
			PK_DBG("second pass, block %i invalid or not activated\n",block);
		}
	}
	return 0;
}

int ngffs_scan_flash(struct ngffs_info *ngsb)
{
	int activeblocks=0,gcblocks=0,createroot=0;
	//int b;
	size_t retlen;
	int ret,i,j;
	int valid_block=0;
	u_char buff[EMPTY_BYTES+sizeof(struct header_chunk)];
	u_char empty[EMPTY_BYTES];
	struct header_chunk *header=(struct header_chunk *)(buff+EMPTY_BYTES);
	struct mtd_info *mtd;
	int numblocks;
	int rv;

	mtd=ngsb->mtd;
	numblocks=ngsb->blocks;
	// TODO memset?
	for(i=0;i<EMPTY_BYTES;i++) empty[i]=0;


	PK_DBG("getting block headers\n");
	activeblocks=0;
	for(i=0;i<numblocks;i++) {
		PK_DBG("block %i:\n",i);
		NGFFS_BLOCK(ngsb,i)->offset = mtd->erasesize * i;
		NGFFS_BLOCK(ngsb,i)->size = mtd->erasesize;
		// read in EMPTY_BYTES plus header chunk
		ret=mtd->read(mtd,(mtd->erasesize*i)+CONFIG_NGFFS_START_OFS,
			      EMPTY_BYTES+sizeof(struct header_chunk),&retlen,buff);
		if (ret) {
			PK_ERR("ngffs_scan_flash: Read error at 0x%08x: %d\n", mtd->erasesize*i, ret);
			return ret;
		}
		if (retlen != (sizeof(struct header_chunk)+EMPTY_BYTES)) {
			PK_ERR("Short read: 0x%x bytes at 0x%08x instead of requested %x\n",
			       retlen, mtd->erasesize*i, sizeof(struct header_chunk)+EMPTY_BYTES);
			return -EIO;
		}
		PK_DBG("header->valid = 0x%x, header->type = %i\n",header->valid,header->type);

		// innocent until proven guilty
		valid_block=1;
		// guilty if no header chunk
		if(header->type!=NGFFS_HEADER_CHUNK) {
			PK_DBG("UNUSED: first chunk type is not header chunk type\n");
			valid_block=0;
		}
		// guilty if not written or invalidated
		//    if(header->valid!=NGFFS_CHUNK_WRITTEN && header->valid!=NGFFS_CHUNK_INVALID) {
		if(!IS_WRITTEN(header) || !IS_VALID(header)) {
			valid_block=0;
			PK_DBG("UNUSED: first chunk is not valid|written\n");
		}
		// guilty if first EMPTY_BYTES bytes not all empty
		for(j=0;j<EMPTY_BYTES;j++) {
			if(buff[j]!=0) {
				PK_DBG("UNUSED: byte at offset %i is not zero\n",j);
				valid_block=0;
			}
		}
		//
		if(valid_block) {
			PK_DBG("block is ngffs formatted..\n");
			activeblocks++;
			bcopy(header,&(NGFFS_BLOCK(ngsb,i)->header),sizeof(struct header_chunk));
		} else {
			// mark guilty blocks so we can reformat them
			PK_DBG("block is NOT ngffs formatted..\n");
			//NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_VALID;
			RESET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
			RESET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
			SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
			NGFFS_BLOCK(ngsb,i)->header.type=-1;
		}
	}

	if(activeblocks==0) createroot=1;

	// count blocks that are to be overwritten by GC
	for(i=0;i<numblocks;i++) {
		PK_DBG("%i: type=%i 0x%02x\n",i,NGFFS_BLOCK(ngsb,i)->header.type,NGFFS_BLOCK(ngsb,i)->header.valid);
		if( (NGFFS_BLOCK(ngsb,i)->header.type==NGFFS_HEADER_CHUNK) &&
		    ( !IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header))  || NGFFS_BLOCK(ngsb,i)->header.activated) ) {
			//    (NGFFS_BLOCK(ngsb,i)->header.valid!=NGFFS_CHUNK_WRITTEN || NGFFS_BLOCK(ngsb,i)->header.activated) ) {
			gcblocks++;
		}
	}

	PK_DBG("activeblocks=%i, createroot=%i, gcblocks=%i\n",activeblocks,createroot,gcblocks);

	// now reformat blocks if needed, making sure that we have at least
	// one block ready to be overwritten by GC
	for(i=0;i<numblocks;i++) {
		//if(NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_VALID) {
		if(NGFFS_BLOCK(ngsb,i)->header.type==-1) {
			PK_DBG("formatting block %i\n",i);
			// doformat
			ngffs_erase_flash(mtd,i,1);
			while(!ngffs_erase_ready(1)) {
				// schedule();
			}

			NGFFS_BLOCK(ngsb,i)->header.type=NGFFS_HEADER_CHUNK;
			NGFFS_BLOCK(ngsb,i)->header.len=mtd->erasesize;
			if(gcblocks) {
				// already have GC blocks, make this a usable block
				SET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
				RESET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
				SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
				//NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_VALID;
			} else {
				// no GC blocks yet, make this GC-able
				RESET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
				RESET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
				SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
				//NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_INVALID;
				gcblocks++;
			}
			NGFFS_BLOCK(ngsb,i)->header.usage=1;
			NGFFS_BLOCK(ngsb,i)->header.activated=0x0;
			NGFFS_BLOCK(ngsb,i)->header.maxfsize=NGFFS_MAXFSIZE;
			NGFFS_BLOCK(ngsb,i)->header.maxcsize=NGFFS_MAXCSIZE;
			NGFFS_BLOCK(ngsb,i)->header.maxdirent=NGFFS_MAXDIRENT;

			//PK_DBG("block %i is %s\n",i,(NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_INVALID)?"GC":"usable");
			PK_DBG("block %i is %s\n",i,(IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header)))?"GC":"usable");

			// write emptybytes
			if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset,empty,EMPTY_BYTES)) {
				PK_ERR("PANIC! error writing new emptybytes chunk\n");
				return -EIO;
			}
			if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES,(u_char *)&(NGFFS_BLOCK(ngsb,i)->header),sizeof(struct header_chunk))) {
				PK_ERR("PANIC! error writing new header chunk\n");
				return -EIO;
			}
			/*
			if(NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_VALID) {
			if(ngffs_written_chunk(ngsb->sb,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES)) {
			PK_ERR("PANIC! error marking new header chunk WRITTEN\n");
			return -EIO;
			}
			NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_WRITTEN;
			}
*/
			if(IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header))) {
				if(ngffs_written_chunk(ngsb->sb,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES)) {
					PK_ERR("PANIC! error marking new header chunk WRITTEN\n");
					return -EIO;
				}
				SET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
				//SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
			}
		}
	}

	if(createroot) {
		PK_DBG("creating root inode\n");
		for(i=0;i<numblocks && createroot;i++) {
			//if( (!(NGFFS_BLOCK(ngsb,i)->header.activated)) && (NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_WRITTEN)) {
			if( (!(NGFFS_BLOCK(ngsb,i)->header.activated)) && IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header)) && IS_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header))) {
				PK_DBG("placing root inode in block %i\n",i);
				ngffs_create_root(ngsb,NGFFS_BLOCK(ngsb,i));
				ngsb->max_entry_id=99;	// start at 100, <100 is for sysfile subsystem
				createroot=0;
			}
		}
		if(createroot) {
			PK_ERR("should create root, but not created, panic");
			return -EIO;
		}
	}


	PK_DBG("ngffs: scanning %d blocks\n",numblocks);

	for(i=0;i<numblocks;i++) {
		//if( (!(NGFFS_BLOCK(ngsb,i)->header.activated)) && NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_WRITTEN) ) {
		if( (!(NGFFS_BLOCK(ngsb,i)->header.activated)) && IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header)) && IS_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header)) ) {
			PK_DBG("scanning block %i\n",i);
			rv=ngffs_scan_block(ngsb,i);
			if(rv) {
				PK_ERR("scan block err for block %i\n",i);
				return -EIO;
			}
		}
	}

	if((ret=ngffs_build_data_cache(ngsb))) {
		PK_ERR("unable to build data cache..\n");
		return -EIO;
	}

#ifdef BARCELONA_DEBUG
	for(i=0;i<numblocks;i++) {
		PK_DBG("%i: offset=0x%x (%i), size=0x%x (%i), freespace=0x%x (%i)\n",i,
		       NGFFS_BLOCK(ngsb,i)->offset,NGFFS_BLOCK(ngsb,i)->offset,
		       NGFFS_BLOCK(ngsb,i)->size,NGFFS_BLOCK(ngsb,i)->size,
		       NGFFS_BLOCK(ngsb,i)->freespace,NGFFS_BLOCK(ngsb,i)->freespace);
		PK_DBG("   valid=0x%08x, len=0x%x (%i), activated=0x%x, usage=%i\n",
		       NGFFS_BLOCK(ngsb,i)->header.valid,
		       NGFFS_BLOCK(ngsb,i)->header.len,NGFFS_BLOCK(ngsb,i)->header.len,
		       NGFFS_BLOCK(ngsb,i)->header.activated,
		       NGFFS_BLOCK(ngsb,i)->header.usage);
	}
#endif

	PK_DBG("ngffs: scan done\n");
	return 0;
}

/*
 **  Flash scan routine for already mounted filesystems (used after
 **  garbage collection to update the various caches)
 */
int ngffs_rescan_flash(struct super_block * sb)
{
	int b;
	size_t retlen;
	int ret;
	u_char buff[EMPTY_BYTES+sizeof(struct header_chunk)];
	struct header_chunk *header=(struct header_chunk *)(buff+EMPTY_BYTES);
	struct ngffs_info *ngsb;
	struct mtd_info *mtd;
	int numblocks;

	ngsb=NGFFS_INFO(sb);
	mtd=ngsb->mtd;
	numblocks=ngsb->blocks;

	for(b=0;b<numblocks;b++) {
		PK_DBG("ngffs: re-scanning block %d\n",b);
		ret=mtd->read(mtd,(mtd->erasesize*b)+CONFIG_NGFFS_START_OFS,EMPTY_BYTES+sizeof(struct header_chunk),&retlen,buff);
		if (ret) {
			PK_ERR(KERN_NOTICE "ngffs_scan_flash: Read error at 0x%08x: %d\n", mtd->erasesize*b, ret);
			return ret;
		}
		if (retlen != (sizeof(struct header_chunk)+EMPTY_BYTES)) {
			PK_ERR("Short read: 0x%x bytes at 0x%08x instead of requested %x\n",
			       retlen, mtd->erasesize*b, sizeof(struct header_chunk)+EMPTY_BYTES);
			return -EIO;
		}
		// overwrite current header for this block
		bcopy(header,&(NGFFS_BLOCK(ngsb,b)->header),sizeof(struct header_chunk));

		// is this a valid & active block?
		if(IS_WRITTEN(header) && IS_VALID(header) && !(header->activated)) {
			PK_DBG("block is valid & active\n");
			// yes => scan it
			// note that this will overwrite file & dir chunks but
			// leaves dir list entries and file list entries intact
			ngffs_scan_block(ngsb,b);
		} else {
			PK_DBG("block is NOT valid & active\n");
		}
	}

	if((ret=ngffs_build_data_cache(ngsb))) {
		PK_ERR("ngffs_build_data_cache failed..\n");
		return ret;
	}

	return 0;
}

/* EOF */
