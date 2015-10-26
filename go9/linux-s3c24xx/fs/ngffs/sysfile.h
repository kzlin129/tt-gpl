/* fs/ngffs/sysfile.h
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

#ifndef __FS_NGFFS_SYSFILE_H
#define __FS_NGFFS_SYSFILE_H

#define NGFFS_SYSFILE_DIR	"sysfile"

struct ngffs_sysfile {
	__u32 length;
	__u32 ofs;
	char *name;
};

#ifndef NGFFS_SYSFILE
extern struct file_operations ngffs_sysfile_operations;
extern struct inode_operations ngffs_sysfile_inode_operations;
extern struct address_space_operations ngffs_sysfile_address_operations;
#endif /* NGFFS_SYSFILE */

#endif /* __FS_NGFFS_SYSFILE_H */

/* EOF */
