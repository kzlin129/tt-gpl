/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/




#ifndef VC_FILESERVICE_DEFS_H
#define VC_FILESERVICE_DEFS_H

/* Definitions (not used by API) */
#define FS_MAX_DATA 4096

/* Protocol (not used by API) version 1.2 */

enum {
   /* Over-the-wire file open flags */
   VC_O_RDONLY     = 0x01,
   VC_O_WRONLY     = 0x02,
   VC_O_RDWR            = 0x04,
   VC_O_APPEND     = 0x08,
   VC_O_CREAT           = 0x10,
   VC_O_TRUNC           = 0x20,
   VC_O_EXCL            = 0x40,

   /* Request Commands (VC->Host->VC) */

   /* These commands don't require a pathname */
   VC_FILESYS_RESET      = 64,
   VC_FILESYS_CLOSE      = 65,
   VC_FILESYS_CLOSEDIR   = 66,
   VC_FILESYS_LSEEK      = 67,
   VC_FILESYS_READ       = 68,
   VC_FILESYS_READDIR    = 69,
   VC_FILESYS_SETEND     = 70,
   VC_FILESYS_WRITE      = 71,

   /* These commands require a pathname */
   VC_FILESYS_FORMAT     = 72,
   VC_FILESYS_FREESPACE  = 73,
   VC_FILESYS_GET_ATTR   = 74,
   VC_FILESYS_MKDIR      = 75,
   VC_FILESYS_OPEN       = 76,
   VC_FILESYS_OPENDIR    = 77,
   VC_FILESYS_REMOVE     = 78,
   VC_FILESYS_RENAME     = 79,
   VC_FILESYS_SET_ATTR   = 80,
   VC_FILESYS_SCANDISK   = 81,
   VC_FILESYS_TOTALSPACE = 82,
   VC_FILESYS_DISKWRITABLE=83,
   VC_FILESYS_OPEN_DISK_RAW  = 84,
   VC_FILESYS_CLOSE_DISK_RAW = 85,
   VC_FILESYS_NUMSECTORS     = 86,
   VC_FILESYS_READ_SECTORS   = 87,
   VC_FILESYS_WRITE_SECTORS  = 88,

   VC_FILESYS_MOUNT      = 89,
   VC_FILESYS_UMOUNT     = 90,
   VC_FILESYS_FSTYPE     = 91,

   VC_FILESYS_READ_DIRECT = 92,
   
   VC_FILESYS_LSEEK64     = 93,
   VC_FILESYS_FREESPACE64 = 94,
   VC_FILESYS_TOTALSPACE64= 95
};

/* Parameters for lseek */

#define	VC_FILESYS_SEEK_SET  0    /* Set file pointer to "offset" */
#define	VC_FILESYS_SEEK_CUR  1    /* Set file pointer to current plus "offset" */
#define	VC_FILESYS_SEEK_END  2    /* Set file pointer to EOF plus "offset" */

/* Return values of vc_filesys_type */
#define VC_FILESYS_FS_UNKNOWN 0
#define VC_FILESYS_FS_FAT12 1
#define VC_FILESYS_FS_FAT16 2
#define VC_FILESYS_FS_FAT32 3

#endif
