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




#include "vchost_config.h"
#include "vcfilesys_defs.h"
#include "vc_fileservice_defs.h"

typedef struct DIR_tag DIR;

// Initialises the file system for use
int vc_filesys_init (void);

// Stop it to prevent the functions from trying to use it.
void vc_filesys_stop(void);

// Return the service number (-1 if not running).
int vc_filesys_inum(void);

// Low level file system functions equivalent to close(), lseek(), open(), read() and write()
int vc_filesys_close(int fildes);
long vc_filesys_lseek(int fildes, long offset, int whence);
int vc_filesys_open(const char *path, int vc_oflag);
int vc_filesys_read(int fildes, void *buf, unsigned int nbyte);
int vc_filesys_write(int fildes, const void *buf, unsigned int nbyte);

int vc_filesys_mount(const char *device, const char *mountpoint, const char *options);
int vc_filesys_umount(const char *mountpoint);


// Ends a directory listing iteration
int vc_filesys_closedir(void *dhandle);

// Formats the drive that contains the given path
int vc_filesys_format(const char *path);

// Returns the amount of free space on the drive that contains the given path
int vc_filesys_freespace(const char *path);

// Gets the attributes of the named file
int vc_filesys_get_attr(const char *path, fattributes_t *attr);

// Creates a new directory
int vc_filesys_mkdir(const char *path);

// Starts a directory listing iteration
void *vc_filesys_opendir(const char *dirname);

// Directory listing iterator
struct dirent *vc_filesys_readdir_r(void *dhandle, struct dirent *result);

// Deletes a file or (empty) directory
int vc_filesys_remove(const char *path);

// Renames a file, provided the new name is on the same file system as the old
int vc_filesys_rename(const char *oldfile, const char *newfile);

// Resets the co-processor side file system
int vc_filesys_reset(void);

// Sets the attributes of the named file
int vc_filesys_set_attr(const char *path, fattributes_t attr);

// Truncates a file at its current position
int vc_filesys_setend(int fildes);

// Checks whether there are any messages in the incoming message fifo and responds to any such messages
int vc_filesys_poll_message_fifo(void);

// Return the event used to wait for reads.
void *vc_filesys_read_event(void);

// Sends a command for VC01 to reset the file system
void vc_filesys_sendreset(void);

// Return the error code of the last file system error
int vc_filesys_errno(void);

// Invalidates any cluster chains in the FAT that are not referenced in any directory structures
void vc_filesys_scandisk(const char *path);

// Return whether a disk is writeable or not.
int vc_filesys_diskwritable(const char *path);

// Return file system type of a disk.
int vc_filesys_fstype(const char *path);

// Returns the toatl amount of space on the drive that contains the given path
int vc_filesys_totalspace(const char *path);

// Open disk for block level access
int vc_filesys_open_disk_raw(const char *path);

// Close disk from block level access mode
int vc_filesys_close_disk_raw(const char *path);

// Return number of sectors.
int vc_filesys_numsectors(const char *path);

// Begin reading sectors from VideoCore.
int vc_filesys_read_sectors_begin(const char *path, uint32_t sector, uint32_t count);

// Read the next sector.
int vc_filesys_read_sector(char *buf);

// End streaming sectors.
int vc_filesys_read_sectors_end(uint32_t *sectors_read);

// Begin writing sectors from VideoCore.
int vc_filesys_write_sectors_begin(const char *path, uint32_t sector, uint32_t count);

// Write the next sector.
int vc_filesys_write_sector(const char *buf);

// End streaming sectors.
int vc_filesys_write_sectors_end(uint32_t *sectors_written);
