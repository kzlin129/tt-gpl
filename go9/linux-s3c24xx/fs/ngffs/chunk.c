/* fs/ngffs/chunk.c
 *
 * NGFFS, next generation flash file system
 *
 * Copyright (C) 2004-2007 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Chunks are the units in which access to the flash is organised,
 * there exist four types of chunks:
 *
 * - header chunks
 *     each erase-block of flash has as it's first chunk
 *     a header chunk which specifies activated and validity
 *     and more
 *
 * - directory chunks
 *     each (sub-)directory on the flash filesystem has a
 *     directory chunk, which contains the number of items,
 *     the name and pointers to the file/dir chunks of each
 *     entry in this directory
 *
 * - file chunks
 *     each file on the flash filesystem has a file chunk that
 *     defines name, size, permissions, etc...
 *
 * - data chunks
 *     contains actual file data, all data chunks contain MAX_CSIZE
 *     bytes, with the possible exception of the last chunk, which
 *     may contain 512 or less bytes
 *
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
#include "garbage.h"

#ifdef CONFIG_NGFFS_FLASH_DUMP
#include "linux/delay.h"

#define DUMP_SLEEP 20
#endif /* CONFIG_NGFFS_FLASH_DUMP */

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

int ngffs_invalidate_chunk(struct super_block *sb,__u32 ofs);
int ngffs_chunk_check_field_crc(__u32 w);
char ngffs_chunk_field_v2(__u32 w);

/*
 * Function that calculates for any type of chunk the total size
 *
 * When ignoreValid is not null, the valid field will be
 * ignored (this is usefull when CHUNK_SIZE is used in
 * the context of write functions
 */
__u32 CHUNK_SIZE(void *chunk, int ignoreValid)
{
	//  if(((struct generic_chunk *)chunk)->valid==NGFFS_CHUNK_WRITTEN ||
	//     ((struct generic_chunk *)chunk)->valid==NGFFS_CHUNK_INVALID ||
	if( IS_WRITTEN(GENERIC_CHUNK(chunk)) || ignoreValid ) {
		// Chunk has been written entirely (possibly invalidated), so we
		// can safely assume the chunk is consistent, so calculate length
		switch(((struct generic_chunk *)chunk)->type) {
		case NGFFS_HEADER_CHUNK:
			// header chunk has constant size
			return sizeof(struct header_chunk);
		case NGFFS_DIR_CHUNK:
			// size of dir chunk is constant + length of name +
			// length of array of entry_id's
			return sizeof(struct dir_chunk) +
					( ((struct generic_chunk *)chunk)->len*sizeof(__u32) ) +
					((struct generic_chunk *)chunk)->extra;  // namelen
		case NGFFS_FILE_CHUNK:
			// length of file chunk is constant + length of name
			return sizeof(struct file_chunk) +
					( ((struct generic_chunk *)chunk)->len/NGFFS_MAXCSIZE ) +
					(NGFFS_NUMCHUNKS(((struct generic_chunk *)chunk)->len)*sizeof(__u32)) +
					((struct generic_chunk *)chunk)->extra;  // namelen
		case NGFFS_DATA_CHUNK:
			// length of data chunk is constant + data size
			return sizeof(struct data_chunk) +
					((struct generic_chunk *)chunk)->len;
		default:
			// this should never happen (Hi, Dimitry ;)
			PK_DBG("Invalid chunk type %d\n",((struct generic_chunk *)chunk)->type);
			return -1;
		}
	} else {
		// Chunk has NOT been written entirely, so we give the standard
		// size := MAX(size(dirchunk w/ max entries),size(datachunk w/ max bytes))
		return NGFFS_LARGESTCHUNK;
	}
	return -1;
}

/*
 * get the pointer to the cached chunk with the given entry_id, so this
 * is not the pointer in the flash mem, it's a pointer to the chunk in
 * the dir chunk list or file chunk list
 */
struct generic_chunk *ngffs_get_chunk(struct super_block *sb,__u32 entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id)
			return (struct generic_chunk *)file->chunk;
	}
	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id)
			return (struct generic_chunk *)dir->chunk;
	}
	PK_DBG("return null for entry_id %u\n",entry_id);
	return NULL;
}

/*
 ** tries to read the data chunk with given offset (relative to start of flash mem) and
 ** puts it in the data_chunk struct pointed at by the provided pointer
 **
 ** RETURNS
 **   data	if success
 **   NULL	if read failed
 **
 */
struct data_chunk *ngffs_get_data_chunk(struct super_block *sb,__u32 ofs,struct data_chunk *data)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	int ec;

	PK_DBG("ngffs_get_data_chunk(0x%x,%u,0x%x)\n",(unsigned int)sb,ofs,(unsigned int)data);

	// check for out of bound reads
	if(ofs>ngsb->size) return NULL;

	if((ec=ngffs_read(ngsb->mtd,ofs,(u_char *)data,sizeof(struct generic_chunk)))) {
		PK_DBG("ngffs_get_data_chunk(..,%i): ngffs_read returned error %i for generic chunk\n",ofs,ec);
		return NULL;
	}
	PK_DBG("got generic chunk datalen=%u\n",data->len);

	// can we trust this chunk?
	//  if(data->valid!=NGFFS_CHUNK_WRITTEN) return NULL;
	if(!IS_WRITTEN(data) || !IS_VALID(data)) return NULL;

	// TODO: shouldn't that be CHUNKSIZE(data) in the 2nd argument..
	if((ec=ngffs_read(ngsb->mtd,ofs,(u_char *)data,sizeof(struct data_chunk)+data->len))) {
		PK_DBG("ngffs_get_data_chunk(..,%i): ngffs_read returned error %i for actual chunk\n",ofs,ec);
		return NULL;
	}
	return data;
}

/*
 ** tries to write out the provided chunk into the specified block of flash mem,
 **
 ** TODO: should probably return offset
 **
 ** RETURNS
 **  0	   on error
 **  offset on success
 */
__u32 ngffs_put_chunk(struct super_block *sb,struct generic_chunk *chunk,int block)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	__u32 size;
	__u32 ofs;
	//__u32 valid;

	PK_DBG("entering\n");

	size=CHUNK_SIZE(chunk,1);
	PK_DBG("size=%u\n",size);

	if(NGFFS_BLOCK(ngsb,block)->freespace<size) return 0;


	// valid should be CHUNK_NGFFS_VALID (0xff)...
	/*
	if(chunk->valid!=NGFFS_CHUNK_VALID) {
	PK_DBG("Warning, writing chunk with valid=0x%x, setting to 0xff\n",(int)chunk->valid);
	chunk->valid=NGFFS_CHUNK_VALID;
	}
*/

	if(IS_WRITTEN(chunk)) {
		PK_DBG("Warning, writing chunk with valid=0x%x, doing |=0xf00 \n",(int)chunk->valid);
		RESET_WRITTEN(chunk);
		SET_CRC(chunk);
	}

	ofs=(ngsb->erasesize*block)+(ngsb->erasesize-NGFFS_BLOCK(ngsb,block)->freespace);

	// write the chunk
	if(ngffs_write(ngsb->mtd,
		       ofs,
		       (u_char *)chunk,
		       size)) {
		// write failed, assume NGFFS_LARGESTCHUNK chunk size (see fn CHUNK_SIZE)
		if(NGFFS_BLOCK(ngsb,block)->freespace<NGFFS_LARGESTCHUNK)
			NGFFS_BLOCK(ngsb,block)->freespace=0;
		else
			NGFFS_BLOCK(ngsb,block)->freespace-=NGFFS_LARGESTCHUNK;

		return 0;
	}

	// write succeeded, mark chunk as WRITTEN
	SET_WRITTEN(chunk);
	//  SET_CRC(chunk);
	if(ngffs_write(ngsb->mtd,
		       ofs + (int)(&(((struct generic_chunk *)0)->valid)),
		       (u_char *)(&(chunk->valid)),
		       4)) {
		// write failed, assume NGFFS_LARGESTCHUNK chunk size (see fn CHUNK_SIZE)
		if(NGFFS_BLOCK(ngsb,block)->freespace<NGFFS_LARGESTCHUNK)
			NGFFS_BLOCK(ngsb,block)->freespace=0;
		else
			NGFFS_BLOCK(ngsb,block)->freespace-=NGFFS_LARGESTCHUNK;
		RESET_WRITTEN(chunk);
		SET_CRC(chunk);
		return 0;
	}

	// write ok, mark as WRITTEN ok, so decrease freespace and return
	NGFFS_BLOCK(ngsb,block)->freespace-=size;
	return ofs;
}

/*
 ** reallocates memory for the chunk, the new area is of size CHUNK_SIZE+delta
 **
 ** RETURNS
 **  NULL	out of memory
 **  pointer to new chunk in other cases
 */
struct generic_chunk *ngffs_resize_chunk(struct generic_chunk *chunk,int delta)
{
	int size=CHUNK_SIZE(chunk,1);
	struct generic_chunk *new;
	PK_DBG("size %i, delta %i\n",size,delta);
	new=kmalloc(size+delta,GFP_KERNEL);
	PK_DBG("kmalloced %i bytes at 0x%p\n",size+delta,new);
	if(new==NULL) {
		PK_ERR("ngffs_resize_chunk(): out of mem\n");
		return NULL;
	}
	bcopy(chunk,new,(delta>0)?size:(size+delta));
	PK_DBG("copied %i bytes..\n",(delta>0)?size:(size+delta));
	return new;
}

/*
 ** Scans the flash memory, searching for the valid chunk with entry_id
 **
 **
 ** RETURNS
 **  0 if the chunk could not be found, the offset else
 **
 */
__u32 ngffs_get_chunk_ofs(struct super_block *sb,unsigned long entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	int i;
	__u32 ofs;
	struct generic_chunk chunk;
	__u32 max_modification=0;
	__u32 max_ofs=0;

	PK_DBG("looking for %lu\n",entry_id);

	for(i=0;i<ngsb->blocks;i++) {			// loop over all blocks
		ofs= (i*ngsb->erasesize) + EMPTY_BYTES;
		while(ofs<(((i+1)*ngsb->erasesize)-sizeof(struct generic_chunk))) {	// scan block

			if(ngffs_read(ngsb->mtd,ofs,(u_char *)&chunk,sizeof(struct generic_chunk))) {
				return 0;				// read error
			}

			PK_DBG("entry_id=%i, valid=0x%x, type=%i, mod=%i\n",chunk.entry_id,chunk.valid,chunk.type,chunk.modification);

			// ngffs: ngffs_get_chunk_ofs: entry_id=1, valid=0xf0, type=2, mod=1


			//if(chunk.valid==NGFFS_CHUNK_WRITTEN && (chunk.type==NGFFS_DIR_CHUNK || chunk.type==NGFFS_FILE_CHUNK) && chunk.entry_id==entry_id) {
			if( IS_VALID(&chunk) && IS_WRITTEN(&chunk) && (chunk.type==NGFFS_DIR_CHUNK || chunk.type==NGFFS_FILE_CHUNK) && chunk.entry_id==entry_id) {
				PK_DBG("match 1\n");
				if(chunk.modification>max_modification) {
					PK_DBG("match 2\n");
					max_modification=chunk.modification;
					max_ofs=ofs;
				}
			}
			// reached end of block yet (type==0xff)?
			if(IS_VALID(&chunk) && chunk.type==NGFFS_UNUSED) break;
			else if(CHUNK_SIZE(&chunk,0)!=-1) ofs+=CHUNK_SIZE(&chunk,0);	// no=>next chunk
			else {
				PK_DBG("block %i corrupted\n",i);
				return 0;	// return error
			}
		}
	}
	if(max_ofs==0) PK_DBG("none found..\n");
	return max_ofs;
}

// scans the internal lists to find the offset
__u32 ngffs_get_cached_chunk_ofs(struct super_block *sb,unsigned long entry_id) {
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id)
			return file->ofs;
	}
	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id)
			return dir->ofs;
	}
	PK_DBG("returning 0 for entry_id %lu\n",entry_id);
	return 0;
}

// actually, isn't this the same as update_chunk_ofs ?!?!?
int ngffs_set_cached_chunk_ofs(struct super_block *sb,unsigned long entry_id,__u32 ofs)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	int updated=0;
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			file->ofs=ofs;
			updated++;
		}
	}
	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id) {
			dir->ofs=ofs;
			updated++;
		}
	}
	PK_DBG("updated %i occurences of entry_id %lu, newofs=%u\n",updated,entry_id,ofs);
	return updated;
}

/*
 ** Enter data chunk info in the data chunk cache
 ** This is not actually caching the data, merely the offset relative to the start of flash
 ** Each entry in the list of file_chunks has it's own personal cache (limited by NGGFS_MAXFSIZE)
 ** discards data which is already cached with a higher modification counter
 **
 ** INVALIDATES CHUNKS THAT ARE DISCARDED!
 **
 ** RETURNS
 **  1 for success
 **  0 for error
 **
 ** (does no checking for overflow or whatever)
 */
int ngffs_cache_data_chunk(struct ngffs_info *ngsb,__u32 entry_id,__u32 seq_id,__u32 modification,__u32 ofs)
{
	struct list_head *curr;
	struct file_list_entry *file;

	PK_DBG("ngffs_cache_data_chunk(..,%u,%u,%u,%u)\n",entry_id,seq_id,modification,ofs);
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {				// loop over all files
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {			// is it our file?
			PK_DBG("cache-mod=%ud, cache-ofs=%ud\n",NGFFS_CACHE(file,seq_id)->modification,NGFFS_CACHE(file,seq_id)->offset);
			if(NGFFS_CACHE(file,seq_id)->modification) {	// don't invalidate if there's nothing cached yet
				if(NGFFS_CACHE(file,seq_id)->modification < modification) { // info is newer, invalidate old chunk (cleanup)
					PK_DBG("invalidating old (cached) chunk\n");
					ngffs_invalidate_chunk(ngsb->sb,NGFFS_CACHE(file,seq_id)->offset);
				} else if(NGFFS_CACHE(file,seq_id)->modification > modification) { // cached is newer, invalidate old chunk (cleanup)
					PK_DBG("invalidating old (offered) chunk\n");
					ngffs_invalidate_chunk(ngsb->sb,ofs);
				}
			}

			if(NGFFS_CACHE(file,seq_id)->modification <= modification) {	// is the info newer?? (added for gc: the same)
				NGFFS_CACHE(file,seq_id)->offset=ofs;			// yes, store info
				NGFFS_CACHE(file,seq_id)->modification=modification;
			}
			return 1;
		}
	}
	return 0;
}

/*
 **
 ** get the cache entry for a given entry/sequence id pair. the returned pointer is to a struct
 ** containing the modification no. and the offset relative to flash start
 **
 ** RETURNS
 **  NULL	on error
 **  ptr		else
 */
struct ngffs_data_cache *ngffs_get_cached_data_chunk(struct super_block *sb,__u32 entry_id,__u32 seq_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	struct file_list_entry *file;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			return NGFFS_CACHE(file,seq_id);
		}
	}
	return NULL;
}

#if 0
/*
 ** gets the entire cache, which is an array of ngffs_data_cache structs
 **
 ** RETURNS
 **  NULL	on failure
 **  array(=ptr)	else
 */
struct ngffs_data_cache *ngffs_get_file_data_cache(struct super_block *sb,__u32 entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	struct file_list_entry *file;

	list_for_each(curr, &ngsb->files) {
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			return file->cache;
		}
	}
	return NULL;
}
#endif

/*
 **  Returns the pointer to the file list entry or NULL if not found
 */
struct file_list_entry *ngffs_get_filele(struct super_block *sb,unsigned long entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file;
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			return file;
		}
	}
	PK_DBG("return NULL for entry_id %lu\n",entry_id);
	return NULL;
}

/*
 **  Returns the pointer to the file list entry or NULL if not found
 */
struct dir_list_entry *ngffs_get_dirle(struct super_block *sb,unsigned long entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir;
		dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id) {
			return dir;
		}
	}
	PK_DBG("return NULL for entry_id %lu\n",entry_id);
	return NULL;
}

struct dir_chunk *ngffs_get_dirptr(struct super_block *sb,unsigned long entry_id)
{
	struct dir_list_entry *dirle=ngffs_get_dirle(sb,entry_id);

	if(dirle==NULL) {
		PK_DBG("return NULL for entry_id %lu\n",entry_id);
		return NULL;
	}
	return dirle->chunk;
}

struct file_chunk *ngffs_get_fileptr(struct super_block *sb,unsigned long entry_id)
{
	struct file_list_entry *filele=ngffs_get_filele(sb,entry_id);

	if(filele==NULL) {
		PK_DBG("return NULL for entry_id %lu\n",entry_id);
		return NULL;
	}
	return filele->chunk;
}

/*
 ** hmmmmm, isn't this the same as set_cached_chunk_ofs ?!?!?
 */
int ngffs_update_chunk_ofs(struct super_block *sb,unsigned long entry_id,__u32 ofs)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	PK_DBG("ngffs_update_chunk_ofs(..,%ld,%u)\n",entry_id,ofs);
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file;
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			file->ofs=ofs;
			return 1;
		}
	}
	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir;
		dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id) {
			dir->ofs=ofs;
			return 1;
		}
	}
	return 0;
}

#ifdef CONFIG_NGFFS_FLASH_DUMP

#define BUFFER_SRC     32
#define BUFFER_DST     2 * BUFFER_SRC + 1 // '\0' on the last position
#define WDT_THRESHOLD  128

const char ngffs_hex_tbl[16] = 
{
	'0', '1', '2', '3', '4', '5', '6', '7', 
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

#define NGGFS_ENC(Char) (ngffs_hex_tbl[(Char) & 0xf])
extern int s3c2410wdt_keepalive(void);

static void ngffs_encode(char *src, char *dst)
{
  unsigned int i = 0;

  while (i < BUFFER_SRC)  {
  	*dst++ = NGGFS_ENC(*src >> 4);
  	*dst++ = NGGFS_ENC(*src);
	src++;
	i++;
  }
}

static void ngffs_dump_flash(struct ngffs_info *ngsb)
{
	struct mtd_info *mtd = ngsb->mtd;
	unsigned int offset = CONFIG_NGFFS_START_OFS;
	unsigned int wdt = WDT_THRESHOLD;
	char buffer_src[BUFFER_SRC];
	char buffer_dst[BUFFER_DST];

	printk(KERN_ERR "Dumping flash\n");
	while (offset < CONFIG_NGFFS_START_OFS + CONFIG_NGFFS_BLOCKS * CONFIG_NGFFS_ERASE_SIZE)
	{
		/* Read less than 2 */
		ngffs_absolute_read(mtd, offset,  buffer_src, BUFFER_SRC);
		ngffs_encode(buffer_src, buffer_dst);
		buffer_dst[BUFFER_DST-1] = '\0';

		if (!wdt--) {
			s3c2410wdt_keepalive();
			wdt = WDT_THRESHOLD;
		}
		printk(KERN_ERR "%08x %s\n", offset, buffer_dst);
		msleep(DUMP_SLEEP);
		offset += BUFFER_SRC;
	}
}

static void ngffs_dump_dir_cache(struct ngffs_info *ngsb)
{
	struct list_head      *curr;
	struct dir_list_entry *dirle;

	list_for_each(curr, &ngsb->dirs) {
		dirle = list_entry(curr, struct dir_list_entry, list);

		if (!dirle)
			break;
		if (!dirle->chunk)
			break;

		printk(KERN_ERR "Dumping directories\n");
		printk(KERN_ERR "%08x %08x %08x %08x %08x %08x %08x\n", 
				dirle->ofs,
				dirle->chunk->type,
				dirle->chunk->valid,
				dirle->chunk->len,
				dirle->chunk->namelen,
				dirle->chunk->modification,
				dirle->chunk->entry_id);
	}
}

static void ngffs_dump_data_cache(struct file_list_entry *filele)
{
	unsigned int index;

	if (!filele) 
		return;

	for (index=0; index<NGFFS_NUMCHUNKS(filele->chunk->len); index++) {
		if (NGFFS_CACHE(filele, index)) {
			printk(KERN_ERR "Dumping data\n");
			printk(KERN_ERR "%08x %08x\n", 
					NGFFS_CACHE(filele, index)->offset,
					NGFFS_CACHE(filele, index)->modification);
		}
	}
}

static void ngffs_dump_file_cache(struct ngffs_info *ngsb)
{
	struct list_head       *curr;
	struct file_list_entry *filele;

	list_for_each(curr, &ngsb->files) {
		filele = list_entry(curr, struct file_list_entry, list);

		if (!filele)
			break;
		if (!filele->chunk)
			break;

		printk(KERN_ERR "Dumping files\n");
		printk(KERN_ERR "%08x %08x %08x %08x %08x %08x %08x\n", 
				filele->ofs,
				filele->chunk->type,
				filele->chunk->valid,
				filele->chunk->len,
				filele->chunk->namelen,
				filele->chunk->modification,
				filele->chunk->entry_id);

		ngffs_dump_data_cache(filele);
	}
}
#endif

/*
 ** Invalidates the chunk at ofs by writing 0 in the valid field. Does no
 ** checking what-so-ever, so better only call this with a valid ofs
 **
 ** RETURNS
 **  0	 on success
 **  -EIO on error
 */
int ngffs_invalidate_chunk(struct super_block *sb,__u32 ofs)
{
	struct ngffs_info *ngsb;
	struct generic_chunk c;

	PK_DBG("sb=0x%p, ofs=%u\n",sb,ofs);

	ngsb=NGFFS_INFO(sb);

	PK_DBG("ngsb=0x%p\n",ngsb);

#ifdef CONFIG_NGFFS_FLASH_DUMP
	{
		__u32 from = CONFIG_NGFFS_START_OFS + ofs + (int)(&(((struct generic_chunk *)0)->valid));
		__u32 last = from + 4 - 1;

		if (from >=0x00040000 && last <= 0x0007ffff) {
			// OK
		}
		else {
			dump_stack();
			ngffs_dump_dir_cache(ngsb);
			ngffs_dump_file_cache(ngsb);
			ngffs_dump_flash(ngsb);
		}
	}
#endif
	if(ngffs_read(ngsb->mtd,ofs + (int)(&(((struct generic_chunk *)0)->valid)) ,(u_char *)(&(c.valid)),4)) {
		PK_ERR("unable to get valid field at ofs 0x%x\n",ofs);
		return -EIO;
	}

	RESET_VALID(&c);
	//  SET_CRC(&c);

	if(ngffs_write(ngsb->mtd , ofs + (int)(&(((struct generic_chunk *)0)->valid)) , (u_char *)(&(c.valid)) , 4)) {
		PK_ERR("unable to invalidate chunk\n");
		return -EIO;
	}



	return 0;
}

/*
 **
 ** RETURNS
 **  0	 on success
 **  -EIO on error
 */
int ngffs_written_chunk(struct super_block *sb,__u32 ofs)
{
	struct ngffs_info *ngsb;
	struct generic_chunk c;

	PK_DBG("sb=0x%p, ofs=%ud\n",sb,ofs);

	ngsb=NGFFS_INFO(sb);

	PK_DBG("ngsb=0x%p\n",ngsb);

	if(ngffs_read(ngsb->mtd,ofs + (int)(&(((struct generic_chunk *)0)->valid)) ,(u_char *)(&(c.valid)),4)) {
		PK_ERR("unable to get valid field at ofs 0x%x\n",ofs);
		return -EIO;
	}

	SET_WRITTEN(&c);
	//  SET_CRC(&c);

	if(ngffs_write(ngsb->mtd , ofs + (int)(&(((struct generic_chunk *)0)->valid)),(u_char *)(&(c.valid)),4)) {
		PK_ERR("unable to mark chunk WRITTEN\n");
		return -EIO;
	}
	return 0;
}

/*
 ** Activates the chunk at ofs by writing 0 in the activated field. Does no
 ** checking what-so-ever, so better only call this with a valid ofs. Makes
 ** sense only for header chunks really..
 **
 ** RETURNS
 **  0	 on success
 **  -EIO on error
 */
int ngffs_activate_chunk(struct super_block *sb,__u32 ofs)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	char activated=0;	// 0x00

	if(ngffs_write(ngsb->mtd , ofs + (int)(&(((struct header_chunk *)0)->activated)) , &activated , 1)) {
		PK_WARN("unable to activate chunk\n");
		return -EIO;
	}
	return 0;
}

/*
 ** Finds the parent of the given entry
 */
__u32 ngffs_parent_chunk_ofs(struct super_block *sb,__u32 entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	struct dir_list_entry *dir;
	__u32 *entries;
	int i;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->dirs) {
		dir=list_entry(curr, struct dir_list_entry, list);

		entries=(__u32*)(((char *)dir->chunk)+sizeof(struct dir_list_entry)+dir->chunk->namelen);
		for(i=0;i<dir->chunk->len;i++) {
			if(entries[i]==entry_id) return dir->chunk->entry_id;
		}
	}
	return -1;
}

/*
 **
 **
 */
int ngffs_replace_chunk(struct super_block *sb,struct generic_chunk *chunk)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	PK_DBG("ngffs_replace_chunk(..,%i)\n",chunk->entry_id);
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr, &ngsb->files) {
		struct file_list_entry *file;
		file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==chunk->entry_id) {
			file->chunk=(struct file_chunk *)chunk;
			return 1;
		}
	}
	list_for_each(curr, &ngsb->dirs) {
		struct dir_list_entry *dir;
		dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==chunk->entry_id) {
			dir->chunk=(struct dir_chunk *)chunk;
			return 1;
		}
	}
	return 0;
}

// returns block no of first block with at least size bytes free
int ngffs_contig_available(struct super_block * sb, __u32 size)
{
	struct ngffs_info *ngsb;
	int i,retry;

	ngsb=NGFFS_INFO(sb);
	CHECK_GLB_LOCK(ngsb->global_lock);

	for(retry=0;retry<ngsb->blocks;retry++) {
		for(i=0;i<ngsb->blocks;i++) {
			PK_DBG("block %i, free space %i, wanted %i, valid=0x%x, act=0x%x\n",i,NGFFS_BLOCK(ngsb,i)->freespace,size,
			       NGFFS_BLOCK(ngsb,i)->header.valid, NGFFS_BLOCK(ngsb,i)->header.activated);
			//      if( (NGFFS_BLOCK(ngsb,i)->header.valid==NGFFS_CHUNK_WRITTEN) && (!NGFFS_BLOCK(ngsb,i)->header.activated) &&
			if( IS_VALID(&(NGFFS_BLOCK(ngsb,i)->header)) &&
			    IS_WRITTEN(&(NGFFS_BLOCK(ngsb,i)->header)) &&
			    (!NGFFS_BLOCK(ngsb,i)->header.activated) && (size<=NGFFS_BLOCK(ngsb,i)->freespace) ) {
				return i;
			}
		}

		if(ngffs_collect_garbage(sb)) {
			return -1;
		}
	}
	return -1;
}

void ngffs_delete_file_chunk(struct super_block *sb,__u32 entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr,&ngsb->files) {
		struct file_list_entry *file=list_entry(curr, struct file_list_entry, list);
		if(file->chunk->entry_id==entry_id) {
			kfree(file->chunk);
			kfree(file->cache);
			list_del(curr);
      kfree(file);
			return;
		}
	}
}

void ngffs_delete_dir_chunk(struct super_block *sb,__u32 entry_id)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;

	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each(curr,&ngsb->dirs) {
		struct dir_list_entry *dir=list_entry(curr, struct dir_list_entry, list);
		if(dir->chunk->entry_id==entry_id) {
			kfree(dir->chunk);
			list_del(curr);
      kfree(dir);
			return;
		}
	}
}

// clean up the mess we made
void ngffs_cleanup(struct super_block *sb)
{
	struct ngffs_info *ngsb=NGFFS_INFO(sb);
	struct list_head *curr;
	struct list_head *temp;

	PK_DBG("freeing block_list\n");
	kfree(ngsb->block_list);

	PK_DBG("freeing files\n");
	CHECK_GLB_LOCK(ngsb->global_lock);

	list_for_each_safe(curr, temp, &ngsb->files) {
		struct file_list_entry *file=list_entry(curr, struct file_list_entry, list);
		kfree(file->chunk);
		kfree(file->cache);
		list_del(curr);
    kfree(file);
	}
	PK_DBG("freeing dirs\n");
	list_for_each_safe(curr, temp, &ngsb->dirs) {
		struct dir_list_entry *dir=list_entry(curr, struct dir_list_entry, list);
		kfree(dir->chunk);
		list_del(curr);
    kfree(dir);
	}
	PK_DBG("cleanup done\n");
}

/* EOF */
