/* fs/ngffs/file.h
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

#ifndef __FS_NGFFS_FILE_H
#define __FS_NGFFS_FILE_H

extern struct file_operations ngffs_file_operations;
extern struct inode_operations ngffs_file_inode_operations;
extern struct address_space_operations ngffs_file_address_operations;

int ngffs_setattr(struct dentry *dentry, struct iattr *iattr);

#endif /* __FS_NGFFS_FILE_H */

/* EOF */
