/* fs/ngffs/garbage.c
 *
 * NGFFS, next generation flash file system
 * Garbage collection
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
#include <asm/uaccess.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include "crc.h"
#include "kern.h"
#include "flash.h"
#include "chunk.h"

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

/*
 **
 ** This does garbage collection for one block:
 **  - loop over the chunks in block 'from'
 **  - copy valid chunks to the new block 'to' (increase modification!)
 **
 ** Does not check whether there is enough free space in the to block, assumes
 ** this is already done by caller.
 **
 */
__u32 ngffs_collect_garbage_block (struct ngffs_info *ngsb, int from, int to)
{
	__u32 ofs=EMPTY_BYTES+sizeof(struct header_chunk);	// skip empty buffer + header
	__u32 newofs;
	struct generic_chunk chunk;
	u_char buffer[sizeof(struct data_chunk)+NGFFS_MAXCSIZE];	// see (i) in ngffs.h


	PK_DBG("collecting garbage for block %i (new block is %i)\n",from,to);
	while(ofs<(NGFFS_BLOCK(ngsb,from)->size-sizeof(struct generic_chunk))) {
		if(ngffs_read(ngsb->mtd, ofs+NGFFS_BLOCK(ngsb,from)->offset, (u_char*)&chunk, sizeof(struct generic_chunk))) {
			PK_WARN("ngffs_block_garbage(sb,%i): unable to read at %u \n",from,ofs+NGFFS_BLOCK(ngsb,from)->offset);
			return 0; // return 0 so garbage collection will leave this block alone
		}
		PK_DBG("ofs=%7u (size=%7u), type=%i, chunk size=%i\n",ofs,NGFFS_BLOCK(ngsb,from)->size,chunk.type,CHUNK_SIZE(&chunk,0));

		if(chunk.type==NGFFS_UNUSED) {
			return 0;
		}
		//if(chunk.valid==NGFFS_CHUNK_WRITTEN) {
		if(IS_WRITTEN(&chunk) && IS_VALID(&chunk)) {
			// chunk is valid -> copy
			//
			if(ngffs_read(ngsb->mtd, ofs+NGFFS_BLOCK(ngsb,from)->offset, buffer, CHUNK_SIZE(&chunk,0))) {
				PK_WARN("Unable to read during garbage collection at ofs %u\n",ofs+NGFFS_BLOCK(ngsb,from)->offset);
				return -EIO;
			}
			((struct generic_chunk *)buffer)->modification++;
			newofs=NGFFS_BLOCK(ngsb,to)->offset + ( NGFFS_BLOCK(ngsb,to)->size-NGFFS_BLOCK(ngsb,to)->freespace );
			// don't bother with valid/written, no chunk in the to block
			// is valid before the block itself is, and it isn't at this point
			// TODO: update old ngffs chunks to new ngffs chunks??
			if(ngffs_write(ngsb->mtd, newofs, buffer, CHUNK_SIZE(&chunk,1))) {
				PK_WARN("Unable to write during garbage collection at ofs %u\n",newofs);
				return -EIO;
			}
			NGFFS_BLOCK(ngsb,to)->freespace-=CHUNK_SIZE(&chunk,1);
			PK_DBG("copied to %u, %u\n",newofs,NGFFS_BLOCK(ngsb,to)->freespace);
		}
		ofs+=CHUNK_SIZE(&chunk,0);
	}

	// this condition implies no overrun, so return success
	if(ofs <= NGFFS_BLOCK(ngsb,from)->size) return 0;

	// else ofs>size, meaning chunk exceeds block, err
	PK_WARN("WARNING chunk exceeds erase block boundary!!!\n");
	return -EIO;
}

/*
 **
 ** Counts the collectable garbage in a given block
 **
 */
__u32 ngffs_block_garbage(struct ngffs_info *ngsb,int blockidx)
{
	__u32 ofs=EMPTY_BYTES+sizeof(struct header_chunk);	// skip empty buffer + header
	struct generic_chunk chunk;
	__u32 garbage=0;



	PK_DBG("counting garbage for block %i\n",blockidx);
	while(ofs<(NGFFS_BLOCK(ngsb,blockidx)->size-sizeof(struct generic_chunk))) {
		if(ngffs_read(ngsb->mtd, ofs+NGFFS_BLOCK(ngsb,blockidx)->offset, (u_char*)&chunk, sizeof(struct generic_chunk))) {
			PK_WARN("ngffs_block_garbage(sb,%i): unable to read\n",blockidx);
			return 0; // return 0 so garbage collection will leave this block alone
		}
		PK_DBG("ofs=%7u (size=%7u), type=%i, chunk size=%i\n",ofs,NGFFS_BLOCK(ngsb,blockidx)->size,chunk.type,CHUNK_SIZE(&chunk,0));
		if(chunk.type==NGFFS_UNUSED) {
			PK_DBG("garbage count: %u\n",garbage);
			return garbage;
		}
		if(CHUNK_SIZE(&chunk,0)==-1) {
			PK_ERR("got -1 from CHUNK_SIZE");
			return 0;	// getting -1 from CHUNK_SIZE indicates major troubles
		}
		//if(chunk.valid!=NGFFS_CHUNK_WRITTEN) {
		if( !IS_VALID(&chunk) || !IS_WRITTEN(&chunk) ) {
			garbage+=CHUNK_SIZE(&chunk,0);
		}
		ofs+=CHUNK_SIZE(&chunk,0);
	}
	if(ofs>NGFFS_BLOCK(ngsb,blockidx)->size)
		PK_WARN("WARNING chunk exceeds erase block boundary!!!\n");
	PK_DBG("garbage count: %u\n",garbage);
	return garbage;
}

// do actual garbage collection
// The fs is supposed to be locked already by caller
int ngffs_collect_garbage(struct super_block *sb)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	int i;
	int last_inactive=-1;
	int multi_inactive=0;
	int got_active=0;
	int curr=0;
	int ec;
	u_char empty[EMPTY_BYTES];	// all zeroes, needed to initialize a block

	// TODO: memset?
	for(i=0;i<EMPTY_BYTES;i++) empty[i]=0;

	// find out how much garbage is present in each block
	// and find last inactive block (should be only one
	// inactive block really!)
	for(i=0;i<ngsb->blocks;i++) {
		//if((NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_WRITTEN) && !(NGFFS_BLOCK(ngsb,i)->header.activated)) {
		if( IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header))
		    && IS_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header))
		    && !(NGFFS_BLOCK(ngsb,i)->header.activated)) {
			NGFFS_BLOCK(ngsb,i)->garbage=NGFFS_GARBAGE_COLLECT;
			NGFFS_BLOCK(ngsb,i)->reserved=ngffs_block_garbage(ngsb,i);
			got_active=1;
		} else {
			NGFFS_BLOCK(ngsb,i)->garbage=NGFFS_GARBAGE_IGNORE;
			if(last_inactive!=-1) {
				PK_DBG("more than one inactive block!!!\n");
				multi_inactive=1;
			}

			last_inactive=i;

			NGFFS_BLOCK(ngsb,i)->reserved=0;
		}
	}

	// now inactive contains the number of the last inactive block
	// and the ngsb->blocks[i].reserved contain the amount of collectable garbage
	PK_DBG("last_inactive=%i\n",last_inactive);

	if(last_inactive==-1) {
		PK_ERR("ngffs_collect_garbage(): no inactive blocks, garbage collection failed\n");
		return -ENOSPC;
	}

	// correct multiple inactive situation
	if(multi_inactive) {
		PK_DBG("got multi_inactive\n");
		for(i=0;i<ngsb->blocks;i++) {
			PK_DBG("|- checking block %i\n",i);
			if(NGFFS_BLOCK(ngsb,i)->garbage==NGFFS_GARBAGE_IGNORE && i!=last_inactive) {
				PK_DBG("erasing block %i\n",i);
				// erase & activate this block
				ngffs_erase_flash(ngsb->mtd,i,4);
				while(!ngffs_erase_ready(4)) {
					// schedule();
				}

				NGFFS_BLOCK(ngsb,i)->header.type=NGFFS_HEADER_CHUNK;
				NGFFS_BLOCK(ngsb,i)->header.valid=-1;
				SET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
				RESET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
				SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
				NGFFS_BLOCK(ngsb,i)->header.usage++;
				NGFFS_BLOCK(ngsb,i)->header.activated=0x0;
				NGFFS_BLOCK(ngsb,i)->header.maxfsize=NGFFS_MAXFSIZE;
				NGFFS_BLOCK(ngsb,i)->header.maxcsize=NGFFS_MAXCSIZE;
				NGFFS_BLOCK(ngsb,i)->header.maxdirent=NGFFS_MAXDIRENT;

				if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset,empty,EMPTY_BYTES)) {
					PK_ERR("PANIC! error writing empty bytes\n");
					return -EIO;
				}
				if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES,(u_char *)&(NGFFS_BLOCK(ngsb,i)->header),sizeof(struct header_chunk))) {
					PK_ERR("PANIC! error writing new header chunk\n");
					return -EIO;
				}
				if(ngffs_written_chunk(ngsb->sb,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES)) {
					PK_ERR("PANIC! error marking new header chunk WRITTEN\n");
					return -EIO;
				}
				SET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
				//SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
				//NGFFS_BLOCK(ngsb,i)->header.valid = NGFFS_CHUNK_WRITTEN;
				NGFFS_BLOCK(ngsb,i)->size = ngsb->mtd->erasesize;
				NGFFS_BLOCK(ngsb,i)->freespace = NGFFS_BLOCK(ngsb,i)->size - EMPTY_BYTES - sizeof(struct header_chunk);
				PK_DBG("block %i size=%i, freespace=%i\n",i,NGFFS_BLOCK(ngsb,i)->size,NGFFS_BLOCK(ngsb,i)->freespace);
			}
		}
		// file should now fit (unless max file size > chunk size of course)
		return 0;
	}

	// find first garbage-collectable block
	i=0;	// use i to count iterations, just to be sure
	curr=(last_inactive+1)%ngsb->blocks;
	while( (i<ngsb->blocks) &&
	       // !( (NGFFS_BLOCK(ngsb,curr)->header.valid==NGFFS_CHUNK_WRITTEN) &&
	       !( IS_VALID(&(NGFFS_BLOCK(ngsb,curr)->header)) && IS_WRITTEN(&(NGFFS_BLOCK(ngsb,curr)->header)) &&
		  !(NGFFS_BLOCK(ngsb,curr)->header.activated) ) ) {

		curr++;
		curr%=ngsb->blocks;
		i++;
	}
	if(i==ngsb->blocks) { // no suitable block found
		PK_ERR("ngffs_garbage_collect(): no valid+active block found!!\n");
		return -ENOSPC;
	}

	// erase last_inactive & mark as 'to be activated'
	ngffs_erase_flash(ngsb->mtd,last_inactive,2);
	while(!ngffs_erase_ready(2)) {
		// schedule();
	}
	NGFFS_BLOCK(ngsb,last_inactive)->garbage=NGFFS_GARBAGE_ACTIVATE;
	NGFFS_BLOCK(ngsb,last_inactive)->freespace=NGFFS_BLOCK(ngsb,curr)->size - EMPTY_BYTES - sizeof(struct header_chunk);

	i=0;	// use i to count iterations again
	curr=(last_inactive+1)%ngsb->blocks;
	while(i<(ngsb->blocks-1)) {
		__u32 notgarbage=NGFFS_BLOCK(ngsb,curr)->size - NGFFS_BLOCK(ngsb,curr)->reserved - EMPTY_BYTES - sizeof(struct header_chunk);
		if( (NGFFS_BLOCK(ngsb,curr)->garbage==NGFFS_GARBAGE_COLLECT) && (notgarbage<=NGFFS_BLOCK(ngsb,last_inactive)->freespace)) {
			if((ec=ngffs_collect_garbage_block(ngsb,curr,last_inactive))) {
				PK_DBG("ngffs_collect_garbage(): copy block from %i to %i failed\n",curr,last_inactive);
				return ec;
			}
			// mark block for invalidation
			NGFFS_BLOCK(ngsb,curr)->garbage=NGFFS_GARBAGE_INVALIDATE;
		}
		curr++;
		curr%=ngsb->blocks;
		i++;
	}

	// now first activate new block (write new header), then invalidate blocks
	// if we lose power here: we're screwed --> no we're not.. we have the
	// 'modified' counter right, we always use last chunk.. just as long as
	// we activate first

	for(i=0;i<ngsb->blocks;i++) {
		if(NGFFS_BLOCK(ngsb,i)->garbage==NGFFS_GARBAGE_ACTIVATE) {
			NGFFS_BLOCK(ngsb,i)->header.type=NGFFS_HEADER_CHUNK;
			NGFFS_BLOCK(ngsb,i)->header.valid=-1;
			SET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
			RESET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
			SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
			NGFFS_BLOCK(ngsb,i)->header.usage++;
			NGFFS_BLOCK(ngsb,i)->header.activated=0;
			NGFFS_BLOCK(ngsb,i)->header.maxfsize= NGFFS_MAXFSIZE;
			NGFFS_BLOCK(ngsb,i)->header.maxcsize= NGFFS_MAXCSIZE;
			NGFFS_BLOCK(ngsb,i)->header.maxdirent= NGFFS_MAXDIRENT;

			PK_DBG("activating block %i\n",i);

			if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset,empty,EMPTY_BYTES)) {
				PK_ERR("PANIC! error writing empty bytes\n");
				return -EIO;
			}
			if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES,(u_char *)&(NGFFS_BLOCK(ngsb,i)->header),sizeof(struct header_chunk))) {
				PK_ERR("PANIC! error writing new header chunk\n");
				return -EIO;
			}
			if(ngffs_written_chunk(ngsb->sb,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES)) {
				PK_ERR("PANIC! error marking new header chunk WRITTEN\n");
				return -EIO;
			}
			SET_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header));
			//SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));
			//NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_WRITTEN;
		}
	}
	for(i=0;i<ngsb->blocks;i++) {
		if(NGFFS_BLOCK(ngsb,i)->garbage==NGFFS_GARBAGE_INVALIDATE) {
			PK_DBG("invalidating block %i\n",i);

			//NGFFS_BLOCK(ngsb,i)->header.valid=NGFFS_CHUNK_INVALID;
			RESET_VALID(&(NGFFS_BLOCK(ngsb,i)->header));
			//SET_CRC(&(NGFFS_BLOCK(ngsb,i)->header));

			if(ngffs_write(ngsb->mtd,NGFFS_BLOCK(ngsb,i)->offset+EMPTY_BYTES,(u_char *)&(NGFFS_BLOCK(ngsb,i)->header),sizeof(struct header_chunk))) {
				PK_ERR("Error invalidating block in garbage collection, PANIC!\n");
				return -EIO;
			}
		}
	}

	// refresh cached offsets
	ngffs_rescan_flash(sb);

	// unlock
	return 0;
}

/* EOF */
