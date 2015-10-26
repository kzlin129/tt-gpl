/* fs/ngffs/chunk.h
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

#ifndef __FS_NGFFS_CHUNK_H
#define __FS_NGFFS_CHUNK_H

extern __u32 CHUNK_SIZE(void *chunk,int ignoreValid);
extern struct generic_chunk *ngffs_get_chunk(struct super_block *sb,__u32 entry_id);
extern __u32 ngffs_put_chunk(struct super_block *sb,struct generic_chunk *chunk,int block);
extern struct generic_chunk *ngffs_resize_chunk(struct generic_chunk *chunk,int delta);

extern __u32 ngffs_get_chunk_ofs(struct super_block *sb,unsigned long entry_id);

extern __u32 ngffs_get_cached_chunk_ofs(struct super_block *sb,unsigned long entry_id);
extern int ngffs_set_cached_chunk_ofs(struct super_block *sb,unsigned long entry_id,__u32 newofs);

extern int ngffs_invalidate_chunk(struct super_block *sb,__u32 ofs);
extern int ngffs_written_chunk(struct super_block *sb,__u32 ofs);
extern int ngffs_activate_chunk(struct super_block *sb,__u32 ofs);
extern int ngffs_replace_chunk(struct super_block *sb,struct generic_chunk *chunk);
extern int ngffs_contig_available(struct super_block * sb, __u32 size);
extern int ngffs_update_chunk_ofs(struct super_block * sb, unsigned long entry_id,__u32 ofs);
extern struct data_chunk *ngffs_get_data_chunk(struct super_block *sb,__u32 ofs,struct data_chunk *data);
extern int ngffs_cache_data_chunk(struct ngffs_info *ngsb,__u32 entry_id,__u32 seq_id,__u32 modification,__u32 ofs);
extern struct ngffs_data_cache *ngffs_get_cached_data_chunk(struct super_block *sb,__u32 entry_id,__u32 seq_id);
//extern struct ngffs_data_cache *ngffs_get_file_data_cache(struct super_block *sb,__u32 entry_id);

extern struct file_list_entry *ngffs_get_filele(struct super_block *sb,unsigned long entry_id);
extern struct dir_list_entry *ngffs_get_dirle(struct super_block *sb,unsigned long entry_id);

extern struct dir_chunk *ngffs_get_dirptr(struct super_block *sb,unsigned long entry_id);  // INODE_DIR
extern struct file_chunk *ngffs_get_fileptr(struct super_block *sb,unsigned long entry_id); // INODE_FILE

extern void ngffs_delete_file_chunk(struct super_block *sb,__u32 entry_id);
extern void ngffs_delete_dir_chunk(struct super_block *sb,__u32 entry_id);

extern void ngffs_cleanup(struct super_block *sb);

#endif /* __FS_NGFFS_CHUNK_H */

/* EOF */
