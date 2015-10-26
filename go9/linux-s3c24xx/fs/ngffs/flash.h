/* fs/ngffs/flash.h
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

#ifndef __FS_NGFFS_FLASH_H
#define __FS_NGFFS_FLASH_H

extern int ngffs_write(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len);
extern int ngffs_absolute_read(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len);
extern int ngffs_read(struct mtd_info *mtd,__u32 ofs,u_char *buff,int len);
extern int ngffs_erase_flash(struct mtd_info *mtd,int block,u_long mask);
extern int ngffs_erase_ready(u_long mask);
extern int ngffs_scan_flash(struct ngffs_info *ngffs);
extern int ngffs_rescan_flash(struct super_block * sb);

#endif /* __FS_NGFFS_FLASH_H */

/* EOF */
