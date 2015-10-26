/* fs/ngffs/crc.h
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

#ifndef __FS_NGFFS_CRC_H
#define __FS_NGFFS_CRC_H

extern unsigned short ngffs_do_crc(const char *message,int nBytes);
extern int ngffs_chunk_check_field_crc(__u32 w);
extern __u32 ngffs_chunk_make_field_crc(__u32 w);
extern char ngffs_chunk_field_v2(__u32 w);

#define IS_VALID(chunk)         (((chunk)->valid)&0xff)
#define IS_WRITTEN(chunk)       (!(ngffs_chunk_field_v2((chunk)->valid)))
#define SET_CRC(chunk)          ((chunk)->valid = ngffs_chunk_make_field_crc((chunk)->valid))

#endif /* __FS_NGFFS_CRC_H */

/* EOF */
