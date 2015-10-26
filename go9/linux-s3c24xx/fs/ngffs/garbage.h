/* fs/ngffs/garbage.h
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

#ifndef __FS_NGFFS_GARBAGE_H
#define __FS_NGFFS_GARBAGE_H

__u32 ngffs_block_garbage(struct ngffs_info *ngsb,int blockidx);
__u32 ngffs_garbage_collectable(struct super_block *sb);		// TODO: tbi

//
// the actual garbage collection, returns 0 on succes, error otherwise
int ngffs_collect_garbage(struct super_block *sb);

#endif /* __FS_NGFFS_GARBAGE_H */

/* EOF */
