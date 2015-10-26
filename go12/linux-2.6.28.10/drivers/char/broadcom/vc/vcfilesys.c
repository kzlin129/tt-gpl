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



#ifdef WIN32
// This is included to enable sleep(). A more permament solution is required.
#include <windows.h>
#endif

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

#include "vcfilesys.h"

/******************************************************************************
Global data.
******************************************************************************/

int gVcFileSysVerbose = 0;

#define VC_VERBOSE(fmt, args...) if ( gVcFileSysVerbose ) printf( "vcfilesys: " fmt, ## args )

/******************************************************************************
Local types and defines.
******************************************************************************/

struct file_service_msg_body {
   uint32_t params[4];
   uint8_t  data[FS_MAX_DATA];
};

typedef enum {
   VC_SECTOR_IO_NONE,
   VC_SECTOR_IO_READING,
   VC_SECTOR_IO_WRITING
} VC_SECTOR_IO_T;

struct vc_file_service_globals {
   int initted;
   int          inum;         /* fifo reference number */
   volatile unsigned int cur_xid;      /* Outstanding transaction's ID */

   /* Storage for requests from the host and their responses, and also
          requests from the co-processor and their responses*/
   struct file_service_msg_body host_body, copr_body;

   /* Copy of the header code from responses */
   int32_t      resp_code;

   /* The error code from the last response to return an error */
   int err_no;

   void *response_lock;
   void *in_ievent;
   void *out_ievent;

   VC_SECTOR_IO_T sector_io;
   uint32_t num_sectors;
};

/******************************************************************************
Static data.
******************************************************************************/

static void *filesys_lock = NULL;
static void *filesys_fifo_lock = NULL;

static struct vc_file_service_globals vc_fsg;

/******************************************************************************
Static functions.
******************************************************************************/

/* File Service Message FIFO functions */
static int vc_fs_stub(int verb, int ext_len);
static int vc_fs_message_handler(void);

//#define PRINTF 1
#ifdef PRINTF
//#define printf tprintf
static void showmsg(VC_MSGFIFO_CMD_HEADER_T const * head, 
                    struct file_service_msg_body const * body);
#endif
static int fs_host_direntbytestream_create(struct dirent *d, void *buffer);
static void fs_host_direntbytestream_interp(struct dirent *d, void *buffer);

static void memcpy_htov32(void*buffer,uint32_t h);

/*---------------------------------------------------------------------------*/

/******************************************************************************
NAME
   vc_filesys_init

SYNOPSIS
   int vc_filesys_init()

FUNCTION
   Initialise the file system for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

int vc_filesys_init () {
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (filesys_lock == NULL)
      filesys_lock = vc_lock_create();

   vc_lock_obtain(filesys_lock);

   vc_fsg.inum = -1;
   vc_fsg.cur_xid = 0;

   if (!vc_fsg.initted) {
      filesys_fifo_lock = vc_lock_create();
      vc_fsg.response_lock = vc_lock_create();
      vc_lock_obtain(vc_fsg.response_lock);
      vc_fsg.in_ievent = vc_event_create();
      vc_fsg.out_ievent = vc_event_create();
      vc_hostfs_init();
      vc_fsg.initted = 1;
      vc_fsg.sector_io = VC_SECTOR_IO_NONE;
   }

   // We simply loop through every interface that there is and look for one
   // that claims to be a file service.
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface,
                                              vc_interface_base+vc_sharedmem_header.iface[i],
                                                                  sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_FILESERVICE) {
            // Gotcha!
            vc_fsg.inum = i;
            vc_interface_register_event_int(vc_fsg.in_ievent, (1<<vc_fsg.inum));
            vc_interface_register_event_int(vc_fsg.out_ievent, (1<<vc_fsg.inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   vc_lock_release(filesys_lock);
   return vc_fsg.inum;
}

/******************************************************************************
NAME
   vc_filesys_stop

SYNOPSIS
   void vc_filesys_stop()

FUNCTION
   This tells us that the file system service has stopped, thereby preventing
   any of the functions from doing anything.

RETURNS
   void
******************************************************************************/

void vc_filesys_stop () {
   // Wait for any existing filesys user to finish before zapping it.
   vc_lock_obtain(filesys_lock);
   vc_lock_obtain(filesys_fifo_lock);
   vc_fsg.inum = -1;
   vc_lock_release(filesys_fifo_lock);
   vc_interface_register_event_int(vc_fsg.in_ievent, 0);
   vc_interface_register_event_int(vc_fsg.out_ievent, 0);
   vc_lock_release(filesys_lock);
}

/******************************************************************************
NAME
   vc_filesys_inum

SYNOPSIS
   int vc_filesys_inum()

FUNCTION
   Return the filesys service number (-1 if not running).

RETURNS
   int
******************************************************************************/

int vc_filesys_inum () {
   return vc_fsg.inum;
}

/* Standard UNIX low-level library functions (declared in unistd.h) */

/******************************************************************************
NAME
   vc_filesys_close

SYNOPSIS
   int vc_filesys_close(int fildes)

FUNCTION
   Deallocates the file descriptor to a file.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_close(int fildes)
{
   VC_VERBOSE( "Closing file descriptor: %d\n", fildes );

   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = fildes;
   if (vc_fs_stub(VC_FILESYS_CLOSE, 4) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_lseek

SYNOPSIS
   long vc_filesys_lseek(int fildes, long offset, int whence)

FUNCTION
   Sets the file pointer associated with the open file specified by fildes.

RETURNS
   Successful completion: offset
   Otherwise: -1
******************************************************************************/

long vc_filesys_lseek(int fildes, long offset, int whence)
{
   long set;

   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = fildes;
   vc_fsg.host_body.params[1] = offset;
   vc_fsg.host_body.params[2] = whence;

   if (vc_fs_stub(VC_FILESYS_LSEEK, 12) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   set = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return set;
}

/******************************************************************************
NAME
   vc_filesys_mount

SYNOPSIS
   int vc_filesys_mount(const char *device, const char *mountpoint, const char *options)

FUNCTION
   Mounts a filesystem at a given location

RETURNS
   Successful completion: 0
******************************************************************************/

int vc_filesys_mount(const char *device, const char *mountpoint, const char *options)
{
   int set, len;
   vc_lock_obtain(filesys_lock);

   strcpy((char*)vc_fsg.host_body.data, device);
   len = strlen(device)+1;
   strcpy((char*)vc_fsg.host_body.data+len, mountpoint);
   len += strlen(mountpoint)+1;
   strcpy((char*)vc_fsg.host_body.data+len, options);
   len += strlen(options)+1;
   len = ((len + 15) & ~15) + 16;

   if (vc_fs_stub(VC_FILESYS_MOUNT, len) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   set = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return set;
}

/******************************************************************************
NAME
   vc_filesys_umount

SYNOPSIS
   int vc_filesys_mount(const char *mountpoint)

FUNCTION
   Un-mounts a removable device from the location that it has been mounted
   to earlier in the session

RETURNS
   Successful completion: 0
******************************************************************************/

int vc_filesys_umount(const char *mountpoint)
{
   int set;
   vc_lock_obtain(filesys_lock);

   strcpy((char*)vc_fsg.host_body.data, mountpoint);

   if (vc_fs_stub(VC_FILESYS_UMOUNT, strlen(mountpoint)+1+16) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   set = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return set;
}

/******************************************************************************
NAME
   vc_filesys_open

SYNOPSIS
   int vc_filesys_open(const char *path, int vc_oflag)

FUNCTION
   Establishes a connection between a file and a file descriptor.

RETURNS
   Successful completion: file descriptor
   Otherwise: -1
******************************************************************************/

int vc_filesys_open(const char *path, int vc_oflag)
{
   int fd;

   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = vc_oflag;
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_OPEN, 16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);

      VC_VERBOSE( "Open of file: '%s' failed\n", path );

      return -1;
   }

   fd = vc_fsg.host_body.params[0];

   VC_VERBOSE( "Opened file: '%s' as descriptor %d\n", path, fd );

   vc_lock_release(filesys_lock);
   return fd;
}


/******************************************************************************
NAME
   vc_filesys_read

SYNOPSIS
   int vc_filesys_read(int fildes, void *buf, unsigned int nbyte)

FUNCTION
   Attempts to read nbyte bytes from the file associated with the file
   descriptor, fildes, into the buffer pointed to by buf.

RETURNS
   Successful completion: number of bytes read
   Otherwise: -1
******************************************************************************/

int vc_filesys_read(int fildes, void *buf, unsigned int nbyte)
{
   int num_read = 0;
   unsigned int num_req;
   uint8_t *ptr = (uint8_t*)buf;

   if (nbyte == 0) {
      return 0;
   }

   do {
      if (nbyte > FS_MAX_DATA) {
         num_req = FS_MAX_DATA;
         nbyte -= FS_MAX_DATA;
      } else {
         num_req = nbyte;
                 nbyte = 0;
      }

      vc_lock_obtain(filesys_lock);
      vc_fsg.host_body.params[0] = fildes;
      vc_fsg.host_body.params[1] = 1;
      vc_fsg.host_body.params[2] = num_req;

      if (vc_fs_stub(VC_FILESYS_READ, 12) != VC_RESP_OK) {
         vc_lock_release(filesys_lock);
         return -1;
      }
      memcpy(&ptr[num_read], vc_fsg.host_body.data,
             vc_fsg.host_body.params[0]);
      num_read += vc_fsg.host_body.params[0];

           if (vc_fsg.host_body.params[0] < num_req) {
         vc_lock_release(filesys_lock);
         return num_read;
           }

      vc_lock_release(filesys_lock);
   } while (nbyte > 0);

   return num_read;
}


/******************************************************************************
NAME
   vc_filesys_write

SYNOPSIS
   int vc_filesys_write(int fildes, const void *buf, unsigned int nbyte)

FUNCTION
   Attempts to write nbyte bytes from the buffer pointed to by buf to file
   associated with the file descriptor, fildes.

RETURNS
   Successful completion: number of bytes written
   Otherwise: -1
******************************************************************************/

int vc_filesys_write(int fildes, const void *buf, unsigned int nbyte)
{
   int num_wrt = 0;
   unsigned int num_req;
   uint8_t *ptr = (uint8_t*) buf;

   if (nbyte == 0) {
      return 0;
   }

   do {
      if (nbyte > FS_MAX_DATA) {
         num_req = FS_MAX_DATA;
         nbyte -= FS_MAX_DATA;
      } else {
         num_req = nbyte;
         nbyte = 0;
      }
      vc_lock_obtain(filesys_lock);
      vc_fsg.host_body.params[0] = fildes;
      vc_fsg.host_body.params[1] = 1;
      vc_fsg.host_body.params[2] = num_req;
      memcpy(vc_fsg.host_body.data, &ptr[num_wrt], num_req);

      if (vc_fs_stub(VC_FILESYS_WRITE, 16+num_req) != VC_RESP_OK) {
         vc_lock_release(filesys_lock);
         return -1;
      }

      num_wrt += vc_fsg.host_body.params[0];

      if (vc_fsg.host_body.params[0] != num_req) {
         vc_lock_release(filesys_lock);
         return num_wrt;
      }

      vc_lock_release(filesys_lock);
   } while (nbyte > 0);

   return num_wrt;
}


/* Directory management functions */

/******************************************************************************
NAME
   vc_filesys_closedir

SYNOPSIS
   int vc_filesys_closedir(void *dhandle)

FUNCTION
   Ends a directory list iteration.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_closedir(void *dhandle)
{
   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = (uint32_t)dhandle;
   if (vc_fs_stub(VC_FILESYS_CLOSEDIR, 4) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_format

SYNOPSIS
   int vc_filesys_format(const char *path)

FUNCTION
   Formats the physical file system that contains path.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_format(const char *path)
{
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_FORMAT,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_freespace

SYNOPSIS
   int vc_filesys_freespace(const char *path)

FUNCTION
   Returns the amount of free space on the physical file system that contains
   path.

RETURNS
   Successful completion: free space
   Otherwise: -1
******************************************************************************/

int vc_filesys_freespace(const char *path)
{
   int freespace;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_FREESPACE,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   freespace = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return freespace;
}


/******************************************************************************
NAME
   vc_filesys_get_attr

SYNOPSIS
   int vc_filesys_get_attr(const char *path, fattributes_t *attr)

FUNCTION
   Gets the file/directory attributes.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_get_attr(const char *path, fattributes_t *attr)
{
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_GET_ATTR,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   *attr = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_mkdir

SYNOPSIS
   int vc_filesys_mkdir(const char *path)

FUNCTION
   Creates a new directory named by the pathname pointed to by path.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_mkdir(const char *path)
{
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_MKDIR,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_opendir

SYNOPSIS
   void *vc_filesys_opendir(const char *dirname)

FUNCTION
   Starts a directory list iteration.

RETURNS
   Successful completion: dhandle (pointer)
   Otherwise: NULL
******************************************************************************/

void *vc_filesys_opendir(const char *dirname)
{
   void *dhandle;
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, dirname, FS_MAX_PATH);
   if (vc_fs_stub(VC_FILESYS_OPENDIR,
       16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return NULL;
   }
   dhandle = (void *)vc_fsg.host_body.params[0];

   vc_lock_release(filesys_lock);
   return dhandle;
}


/******************************************************************************
NAME
   vc_filesys_readdir_r

SYNOPSIS
   struct dirent *vc_filesys_readdir_r(void *dhandle, struct dirent *result)

FUNCTION
   Fills in the passed result structure with details of the directory entry
   at the current psition in the directory stream specified by the argument
   dhandle, and positions the directory stream at the next entry.

RETURNS
   Successful completion: result
   End of directory stream: NULL
******************************************************************************/

struct dirent *vc_filesys_readdir_r(void *dhandle, struct dirent *result)
{
   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = (uint32_t)dhandle;
   if (vc_fs_stub(VC_FILESYS_READDIR, 4) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return NULL;
   }
   fs_host_direntbytestream_interp(result, (void *)vc_fsg.host_body.data);
   vc_lock_release(filesys_lock);
   return result;
}


/******************************************************************************
NAME
   vc_filesys_remove

SYNOPSIS
   int vc_filesys_remove(const char *path)

FUNCTION
   Removes a file or a directory. A directory must be empty before it can be
   deleted.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_remove(const char *path)
{
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_REMOVE,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;

}


/******************************************************************************
NAME
   vc_filesys_rename

SYNOPSIS
   int vc_filesys_rename(const char *oldfile, const char *newfile)

FUNCTION
   Changes the name of a file. The old and new pathnames must be on the same
   physical file system.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_rename(const char *oldfile, const char *newfile)
{
   int a, b;

   // Ensure the pathnames aren't too long
   a = strlen(oldfile);
   b = strlen(newfile);
   if (a >= FS_MAX_PATH || b >= FS_MAX_PATH) {
      return -1;
   }

   vc_lock_obtain(filesys_lock);
   strcpy((char *)vc_fsg.host_body.data, oldfile);
   strcpy((char *)&vc_fsg.host_body.data[a+1], newfile);

   if (vc_fs_stub(VC_FILESYS_RENAME, 16+a+1+b+1) != VC_RESP_OK)
   {
      vc_lock_release(filesys_lock);
      return -1;
   }

   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_reset

SYNOPSIS
   int vc_filesys_reset()

FUNCTION
   Send a VC_FILESYS_RESET command. This will return immediately.

RETURNS
   Succesful completion: VC_RESP_OK
   Otherwise: -
******************************************************************************/

int vc_filesys_reset()
{
   vc_lock_obtain(filesys_lock);
   if (vc_fs_stub(VC_FILESYS_RESET, 0) == VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return 0;
   } else {
      vc_lock_release(filesys_lock);
      return -1;
   }
}


/******************************************************************************
NAME
   vc_filesys_set_attr

SYNOPSIS
   int vc_filesys_set_attr(const char *path, fattributes_t attr)

FUNCTION
   Sets file/directory attributes.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_set_attr(const char *path, fattributes_t attr)
{
   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = attr;
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_SET_ATTR,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_setend

SYNOPSIS
   int vc_filesys_setend(int fildes)

FUNCTION
   Truncates file at current position.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_setend(int fildes)
{
   vc_lock_obtain(filesys_lock);
   vc_fsg.host_body.params[0] = fildes;
   if (vc_fs_stub(VC_FILESYS_SETEND, 4) != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }
   vc_lock_release(filesys_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_filesys_scandisk

SYNOPSIS
   void vc_filesys_scandisk(const char *path)

FUNCTION
   Truncates file at current position.

RETURNS
   -
******************************************************************************/

void vc_filesys_scandisk(const char *path)
{
   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);
   vc_fs_stub(VC_FILESYS_SCANDISK, 16+strlen((char *)vc_fsg.host_body.data)+1);
   vc_lock_release(filesys_lock);
   return;
}


/******************************************************************************
NAME
   vc_filesys_totalspace

SYNOPSIS
   int vc_filesys_totalspace(const char *path)

FUNCTION
   Returns the total amount of space on the physical file system that contains
   path.

RETURNS
   Successful completion: total space
   Otherwise: -1
******************************************************************************/

int vc_filesys_totalspace(const char *path)
{
   int totalspace;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_TOTALSPACE,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   totalspace = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return totalspace;
}


/******************************************************************************
NAME
   vc_filesys_diskwritable

SYNOPSIS
   int vc_filesys_diskwritable(const char *path)

FUNCTION
   Return whether the named disk is writable.

RETURNS
   Successful completion: 1 (disk writable) or 0 (disk not writable)
   Otherwise: -1
******************************************************************************/

int vc_filesys_diskwritable(const char *path)
{
   int writable;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_DISKWRITABLE,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   writable = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return writable;
}

/******************************************************************************
NAME
   vc_filesys_fstype

SYNOPSIS
   int vc_filesys_fstype(const char *path)

FUNCTION
   Return the filesystem type of the named disk.

RETURNS
   Successful completion: disk type (see vc_fileservice_defs.h)
   Otherwise: -1
******************************************************************************/

int vc_filesys_fstype(const char *path)
{
   int fstype;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_FSTYPE,
                  16+strlen((char *)vc_fsg.host_body.data)+1)
       != VC_RESP_OK) {
      vc_lock_release(filesys_lock);
      return -1;
   }

   fstype = vc_fsg.host_body.params[0];
   vc_lock_release(filesys_lock);
   return fstype;
}

/******************************************************************************
NAME
   vc_filesys_open_disk_raw

SYNOPSIS
   int vc_filesys_open_disk_raw(const char *path)

FUNCTION
   Open disk for access in raw mode.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_open_disk_raw(const char *path)
{
   int retval = -1;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_OPEN_DISK_RAW, 16+strlen((char *)vc_fsg.host_body.data)+1) == VC_RESP_OK)
      retval = vc_fsg.host_body.params[0];

   vc_lock_release(filesys_lock);
   return retval;
}

/******************************************************************************
NAME
   vc_filesys_close_disk_raw

SYNOPSIS
   int vc_filesys_close_disk_raw(const char *path)

FUNCTION
   Close disk from access in raw mode.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_close_disk_raw(const char *path)
{
   int retval = -1;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_CLOSE_DISK_RAW, 16+strlen((char *)vc_fsg.host_body.data)+1) == VC_RESP_OK)
      retval = vc_fsg.host_body.params[0];

   vc_lock_release(filesys_lock);
   return retval;
}

/******************************************************************************
NAME
   vc_filesys_numsectors

SYNOPSIS
   int vc_filesys_numsectors(const char *path)

FUNCTION
   Return number of sectors on disk

RETURNS
   Successful completion: greater than 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_numsectors(const char *path)
{
   int retval = -1;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);

   if (vc_fs_stub(VC_FILESYS_NUMSECTORS, 16+strlen((char *)vc_fsg.host_body.data)+1) == VC_RESP_OK)
      retval = vc_fsg.host_body.params[0];

   vc_lock_release(filesys_lock);
   return retval;
}

/******************************************************************************
NAME
   vc_filesys_read_sectors_begin

SYNOPSIS
   int vc_filesys_read_sectors_begin(const char *path, uint32_t sector_num, uint32_t num_sectors)

FUNCTION
   Start streaming sectors from VC01

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_read_sectors_begin(const char *path, uint32_t sector_num, uint32_t num_sectors)
{
   int i;
   int ext_len, ael;
   VC_MSGFIFO_CMD_HEADER_T hdr;

   if (vc_fsg.sector_io != VC_SECTOR_IO_NONE)
      return -1;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);
   vc_fsg.host_body.params[0] = sector_num;
   vc_fsg.host_body.params[1] = num_sectors;
   ext_len = 16+strlen((char*)vc_fsg.host_body.data)+1;

   ael = (ext_len+15) & ~15;

   if (vc_fsg.inum < 0)
      return VC_RESP_ERROR;

   vc_fsg.resp_code = VC_RESP_ERROR;

   i = vc_fsg.cur_xid + 1;
   i &= 0x7fffffffUL;
   vc_fsg.cur_xid = i; /* XXX assume this is atomic */

   hdr.sync = VC_HTOV32(VC_CMD_SYNC);
   hdr.xid  = VC_HTOV32(vc_fsg.cur_xid);
   hdr.cmd_code = VC_HTOV32(VC_FILESYS_READ_SECTORS);
   hdr.ext_length = VC_HTOV16(ext_len);
   hdr.timestamp = VC_HTOV16(0);

   /* This lock stops the reading task from maybe writing to our fifos until the
      response has been read (e.g. to avoid contention with an incoming request
      that must be answered). */
   vc_lock_obtain(filesys_fifo_lock);

   vc_msgfifo_write_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.out_ievent);
   if (ael) {
      vc_msgfifo_write_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.out_ievent);
   }
   vc_msgfifo_write_flush(vc_fsg.inum);

   // We hang onto the locks as we don't want the message handler task to do anything.

   vc_fsg.sector_io = VC_SECTOR_IO_READING;
   vc_fsg.num_sectors = num_sectors;

   return 0;
}

/******************************************************************************
NAME
   vc_filesys_read_sector

SYNOPSIS
   int vc_filesys_read_sector(char *buffer)

FUNCTION
   Read a sector from VC01.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_read_sector(char *buffer)
{
   if (vc_fsg.sector_io != VC_SECTOR_IO_READING || vc_fsg.num_sectors == 0)
      return -1;
   vc_msgfifo_read_blocking(vc_fsg.inum, buffer, 512, vc_fsg.in_ievent);
   vc_msgfifo_read_flush(vc_fsg.inum);
   vc_fsg.num_sectors--;
   return 0;
}

/******************************************************************************
NAME
   vc_filesys_read_sectors_end

SYNOPSIS
   int vc_filesys_read_sectors_end(uint32_t *sectors_read)

FUNCTION
   Read a sector from VC01.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_read_sectors_end(uint32_t *sectors_read)
{
   VC_MSGFIFO_CMD_HEADER_T hdr;
   int ael;
   if (vc_fsg.sector_io != VC_SECTOR_IO_READING || vc_fsg.num_sectors)
      return -1;
   // Here, we actually read the final command header.
   vc_msgfifo_read_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.in_ievent);
   vc_assert(hdr.sync == VC_VTOH32(VC_CMD_SYNC));
   vc_assert(hdr.ext_length == 4);
   ael = (hdr.ext_length+15)&~15;
   vc_msgfifo_read_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.in_ievent);
   vc_msgfifo_read_flush(vc_fsg.inum);
   *sectors_read = VC_VTOH32(vc_fsg.host_body.params[0]);
   vc_fsg.sector_io = VC_SECTOR_IO_NONE;

   // Finally release the locks.
   vc_lock_release(filesys_fifo_lock);
   vc_lock_release(filesys_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_filesys_write_sectors_begin

SYNOPSIS
   int vc_filesys_write_sectors_begin(const char *path, uint32_t sector_num, uint32_t num_sectors)

FUNCTION
   Start streaming sectors from VC01

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_write_sectors_begin(const char *path, uint32_t sector_num, uint32_t num_sectors)
{
   int i;
   int ext_len, ael;
   VC_MSGFIFO_CMD_HEADER_T hdr;

   if (vc_fsg.sector_io != VC_SECTOR_IO_NONE)
      return -1;

   vc_lock_obtain(filesys_lock);
   strncpy((char *)vc_fsg.host_body.data, path, FS_MAX_PATH);
   vc_fsg.host_body.params[0] = sector_num;
   vc_fsg.host_body.params[1] = num_sectors;
   ext_len = 16+strlen((char*)vc_fsg.host_body.data)+1;

   ael = (ext_len+15) & ~15;

   if (vc_fsg.inum < 0)
      return VC_RESP_ERROR;

   vc_fsg.resp_code = VC_RESP_ERROR;

   i = vc_fsg.cur_xid + 1;
   i &= 0x7fffffffUL;
   vc_fsg.cur_xid = i; /* XXX assume this is atomic */

   hdr.sync = VC_HTOV32(VC_CMD_SYNC);
   hdr.xid  = VC_HTOV32(vc_fsg.cur_xid);
   hdr.cmd_code = VC_HTOV32(VC_FILESYS_WRITE_SECTORS);
   hdr.ext_length = VC_HTOV16(ext_len);
   hdr.timestamp = VC_HTOV16(0);

   /* This lock stops the reading task from maybe writing to our fifos until the
      response has been read (e.g. to avoid contention with an incoming request
      that must be answered). */
   vc_lock_obtain(filesys_fifo_lock);

   vc_msgfifo_write_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.out_ievent);
   if (ael) {
      vc_msgfifo_write_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.out_ievent);
   }
   vc_msgfifo_write_flush(vc_fsg.inum);

   // We hang onto the locks as we don't want the message handler task to do anything.

   vc_fsg.sector_io = VC_SECTOR_IO_WRITING;
   vc_fsg.num_sectors = num_sectors;

   return 0;
}

/******************************************************************************
NAME
   vc_filesys_write_sector

SYNOPSIS
   int vc_filesys_write_sector(const char *buffer)

FUNCTION
   Write a sector to VC01.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_write_sector(const char *buffer)
{
   if (vc_fsg.sector_io != VC_SECTOR_IO_WRITING || vc_fsg.num_sectors == 0)
      return -1;
   vc_msgfifo_write_blocking(vc_fsg.inum, (void *)buffer, 512, vc_fsg.out_ievent);
   vc_msgfifo_write_flush(vc_fsg.inum);
   vc_fsg.num_sectors--;
   return 0;
}

/******************************************************************************
NAME
   vc_filesys_write_sectors_end

SYNOPSIS
   int vc_filesys_write_sectors_end(uint32_t *sectors_written)

FUNCTION
   Read a sector from VC01.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_write_sectors_end(uint32_t *sectors_written)
{
   VC_MSGFIFO_CMD_HEADER_T hdr;
   int ael;
   if (vc_fsg.sector_io != VC_SECTOR_IO_WRITING || vc_fsg.num_sectors)
      return -1;
   // Here, we actually read the final command header.
   vc_msgfifo_read_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.in_ievent);
   vc_assert(hdr.sync == VC_VTOH32(VC_CMD_SYNC));
   vc_assert(hdr.ext_length == 4);
   ael = (hdr.ext_length+15)&~15;
   vc_msgfifo_read_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.in_ievent);
   vc_msgfifo_read_flush(vc_fsg.inum);
   *sectors_written = VC_VTOH32(vc_fsg.host_body.params[0]);
   vc_fsg.sector_io = VC_SECTOR_IO_NONE;

   // Finally release the locks.
   vc_lock_release(filesys_fifo_lock);
   vc_lock_release(filesys_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_filesys_errno

SYNOPSIS
   int vc_filesys_errno(void)

FUNCTION
   Returns the error code of the last file system error that occured.

RETURNS
   Error code
******************************************************************************/

int vc_filesys_errno(void)
{
   return vc_fsg.err_no;
}


/* File Service Message FIFO functions */

/******************************************************************************
NAME
   vc_fs_stub

SYNOPSIS
   static int vc_fs_stub(int verb, int ext_len)

FUNCTION
   Generates a request and sends it to the co-processor. It then suspends
   until it receives a reply from the host. The calling task must hold
   the filesys_lock.

RETURNS
   Successful completion: Response code of reply
   Otherwise: -
******************************************************************************/

static int vc_fs_stub(int verb, int ext_len)
{
   int i;
   VC_MSGFIFO_CMD_HEADER_T hdr;
   int ael = (ext_len+15) & ~15;

   if (vc_fsg.inum < 0)
      return VC_RESP_ERROR;

   vc_fsg.resp_code = VC_RESP_ERROR;

   i = vc_fsg.cur_xid + 1;
   i &= 0x7fffffffUL;
   vc_fsg.cur_xid = i; /* XXX assume this is atomic */

   hdr.sync = VC_HTOV32(VC_CMD_SYNC);
   hdr.xid  = VC_HTOV32(vc_fsg.cur_xid);
   hdr.cmd_code = VC_HTOV32(verb);
   hdr.ext_length = VC_HTOV16(ext_len);
   hdr.timestamp = VC_HTOV16(0);

#ifdef PRINTF
   printf("Request from the host\n");
   showmsg(&hdr, &vc_fsg.host_body);
#endif

   /* This lock stops the reading task from maybe writing to our fifos until the
      response has been read (e.g. to avoid contention with an incoming request
      that must be answered). */
   vc_lock_obtain(filesys_fifo_lock);

   vc_msgfifo_write_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.out_ievent);
   if (ael) {

#ifdef VC_HOST_IS_BIG_ENDIAN
          vc_fsg.host_body.params[0] = VC_HTOV32(vc_fsg.host_body.params[0]);
          vc_fsg.host_body.params[1] = VC_HTOV32(vc_fsg.host_body.params[1]);
          vc_fsg.host_body.params[2] = VC_HTOV32(vc_fsg.host_body.params[2]);
          vc_fsg.host_body.params[3] = VC_HTOV32(vc_fsg.host_body.params[3]);
#endif
      vc_msgfifo_write_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.out_ievent);
   }
   vc_msgfifo_write_flush(vc_fsg.inum);

   // Give this us so that the reading task can go.
   vc_lock_release(filesys_fifo_lock);

   /* Wait for our task to process the response and whang the lock */
   vc_lock_obtain(vc_fsg.response_lock);

   return vc_fsg.resp_code;
}

/******************************************************************************
NAME
   vc_fs_message_handler

SYNOPSIS
   static int vc_fs_message_handler()

FUNCTION
   Handle messages from the co-processor.

RETURNS
   0 - No message found.
   1 - Request received and actioned.
   2 - Reply received.
******************************************************************************/

static int vc_fs_message_handler(void)
{
  int rr = 0;

  vc_lock_obtain(filesys_fifo_lock);
  if (vc_fsg.inum < 0) {
    vc_lock_release(filesys_fifo_lock);
    return 0;
  }

  vc_msgfifo_read_refresh(vc_fsg.inum);
  while (vc_msgfifo_input_bytes_available(vc_fsg.inum) >=
         (int)sizeof(VC_MSGFIFO_CMD_HEADER_T)) {
    VC_MSGFIFO_CMD_HEADER_T hdr;
    uint32_t ael;

    rr = 1;

    // Read the request header
    vc_msgfifo_read_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.in_ievent);
    vc_assert(hdr.sync == VC_VTOH32(VC_CMD_SYNC));

    hdr.cmd_code = VC_VTOH32(hdr.cmd_code);
    hdr.ext_length = VC_VTOH16(hdr.ext_length);
    hdr.timestamp = VC_VTOH16(0);
    hdr.xid = VC_VTOH32(hdr.xid);

    ael = (hdr.ext_length + 15) & ~15;

    if (hdr.xid == vc_fsg.cur_xid) {
      /* Process the reply to the current request */

      /* Read in the message body */
      if (ael > 0) {
        vc_assert(ael <= sizeof(struct file_service_msg_body));

        // Read the body
        vc_msgfifo_read_blocking(vc_fsg.inum, &vc_fsg.host_body, ael, vc_fsg.in_ievent);

        vc_fsg.host_body.params[0] = VC_VTOH32(vc_fsg.host_body.params[0]);
        vc_fsg.host_body.params[1] = VC_VTOH32(vc_fsg.host_body.params[1]);
        vc_fsg.host_body.params[2] = VC_VTOH32(vc_fsg.host_body.params[2]);
        vc_fsg.host_body.params[3] = VC_VTOH32(vc_fsg.host_body.params[3]);
      }
      vc_msgfifo_read_flush(vc_fsg.inum);

#ifdef PRINTF
      printf("Reply from VC01\n");
      showmsg(&hdr, &vc_fsg.host_body);
#endif
      vc_fsg.resp_code = hdr.cmd_code;
      if (vc_fsg.resp_code == VC_RESP_ERROR) {
        vc_fsg.err_no = vc_fsg.host_body.params[0];
      }
      vc_lock_release(vc_fsg.response_lock);
      rr = 2;

    } else if ((hdr.xid & 0x80000000UL) == 0x80000000UL) {
      /* Process new requests from the co-processor */

      int retval = VC_RESP_OK;
      uint32_t rlen = 0;
      int i;

      /* Read in the message body */
      if (ael > 0) {
        vc_assert(ael <= sizeof(struct file_service_msg_body));

        // Read the body
        vc_msgfifo_read_blocking(vc_fsg.inum, &vc_fsg.copr_body, ael, vc_fsg.in_ievent);

        vc_fsg.copr_body.params[0] = VC_VTOH32(vc_fsg.copr_body.params[0]);
        vc_fsg.copr_body.params[1] = VC_VTOH32(vc_fsg.copr_body.params[1]);
        vc_fsg.copr_body.params[2] = VC_VTOH32(vc_fsg.copr_body.params[2]);
        vc_fsg.copr_body.params[3] = VC_VTOH32(vc_fsg.copr_body.params[3]);
      }
      vc_msgfifo_read_flush(vc_fsg.inum);

#ifdef PRINTF
      printf("Request from VC01\n");
      showmsg(&hdr, &vc_fsg.copr_body);
#endif

      switch (hdr.cmd_code) {

      case VC_FILESYS_CLOSE:

         i = vc_hostfs_close(vc_fsg.copr_body.params[0]);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_CLOSEDIR:

         i = vc_hostfs_closedir((void *)vc_fsg.copr_body.params[0]);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_FORMAT:

         i = vc_hostfs_format((const char *)vc_fsg.copr_body.data);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_FREESPACE:

         i = vc_hostfs_freespace((const char *)vc_fsg.copr_body.data);
         if (i < 0) {
           retval = VC_RESP_ERROR;
           rlen = 0;
         } else {
           vc_fsg.copr_body.params[0] = i;
           rlen = 4;
         }
         break;

      case VC_FILESYS_GET_ATTR:
        {
          fattributes_t attr;

          i = vc_hostfs_get_attr((const char *)vc_fsg.copr_body.data,
                                 &attr);
          if (i != 0) {
            retval = VC_RESP_ERROR;
            rlen = 0;
          } else {
            vc_fsg.copr_body.params[0] = attr;
            rlen = 4;
          }
        }
        break;
      case VC_FILESYS_LSEEK:

         i = vc_hostfs_lseek(vc_fsg.copr_body.params[0],
                             vc_fsg.copr_body.params[1],
                             vc_fsg.copr_body.params[2]);
         if (i < 0) {
           retval = VC_RESP_ERROR;
           rlen = 0;
         } else {
           vc_fsg.copr_body.params[0] = i;
           rlen = 4;
         }
         break;

      case VC_FILESYS_MKDIR:

         i = vc_hostfs_mkdir((const char *)vc_fsg.copr_body.data);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_OPEN:

   		    i = vc_hostfs_open((const char *)vc_fsg.copr_body.data,
   		                       vc_fsg.copr_body.params[0]);
        if (i < 0) {
          retval = VC_RESP_ERROR;
        } else {
          vc_fsg.copr_body.params[0] = i;
        }
        rlen = 4;
        break;

      case VC_FILESYS_OPENDIR:

         vc_fsg.copr_body.params[0] = (uint32_t)vc_hostfs_opendir(
                                                                  (const char *)vc_fsg.copr_body.data);
         if ((void *)vc_fsg.copr_body.params[0] == NULL) {
           retval = VC_RESP_ERROR;
         }
         rlen = 4;
         break;

      case VC_FILESYS_READ:

         i = vc_hostfs_read(vc_fsg.copr_body.params[0],
                            vc_fsg.copr_body.data,
                            vc_fsg.copr_body.params[1] *
                            vc_fsg.copr_body.params[2]);
         if (i < 0) {
           retval = VC_RESP_ERROR;
         } else {
           vc_fsg.copr_body.params[0] = (int)(i/vc_fsg.copr_body.params[1]);
         }
         rlen = 16+i;
         break;

      case VC_FILESYS_READ_DIRECT:
      {
         const unsigned alignment_addr = 16;
         const unsigned alignment_len = 16;
         uint8_t buffer[1024];
            
         uint32_t fd = vc_fsg.copr_body.params[0];
         uint32_t offset = vc_fsg.copr_body.params[1];
         uint32_t nbyte = vc_fsg.copr_body.params[2];
         uint32_t vc_buf = vc_fsg.copr_body.params[3];

         uint32_t num_resp_bufs = 0;
         uint32_t nbyte_read = 0;

         uint8_t* resp_buffer = vc_fsg.copr_body.data;

         assert(nbyte);

         if ( 0xffffffff != offset) 
         {
            i = vc_hostfs_lseek(fd,
                                offset,
                                VC_FILESYS_SEEK_SET);
               if( 0 > i)
               { 
               retval = VC_RESP_ERROR;
               rlen = 4;
               break;
            }
         }

         if(vc_buf % alignment_addr)
         {
            uint32_t start = 0;
            uint32_t len = alignment_addr - (vc_buf % alignment_addr);
            long used;
            assert(sizeof(buffer) >= len);
            assert(len);
            if(len > nbyte)
               len = nbyte;
            used = vc_hostfs_read(fd,buffer,len);
            if(used < 0){
               retval = VC_RESP_ERROR;
               rlen = 4;
               break;
            }
            assert(used <= (long)len);
            if(used < (long)len)
               nbyte = used;
            len = used;
            if(len){
               memcpy_htov32(resp_buffer,start);
               resp_buffer += sizeof(start);
               memcpy_htov32(resp_buffer,len);
               resp_buffer += sizeof(len);
               memcpy(resp_buffer,buffer,len);
               resp_buffer += len;
               ++num_resp_bufs;
               nbyte_read += len;
               nbyte -= len;
               vc_buf += len;
            }
         }

         while(nbyte){
            size_t available = (nbyte > sizeof(buffer)) ? sizeof(buffer) : nbyte;
            long used = vc_hostfs_read(fd,buffer,available);
            if(used < 0){
               retval = VC_RESP_ERROR;
               rlen = 4;
               break;
            }
            if(!used)
               break;

            assert(!(vc_buf % alignment_addr));
            assert((size_t)used <= (size_t)nbyte);
            if((size_t)used < (size_t)available)
               nbyte = used;

            if((used == (long)nbyte) && (used % alignment_len)){
               uint32_t len = (used % alignment_len);
               size_t buffer_start = used - len;
               uint32_t start = nbyte_read+buffer_start;
               assert(len);
               memcpy_htov32(resp_buffer,start);
               resp_buffer += sizeof(start);
               memcpy_htov32(resp_buffer,len);
               resp_buffer += sizeof(len);
               memcpy(resp_buffer,buffer+buffer_start,len);
               resp_buffer += len;
               ++num_resp_bufs;
               used -= len;
               nbyte_read += len;
               nbyte -= len;
            }

            assert(!(used % alignment_len));

            vc_host_write_consecutive(vc_buf, buffer, used, 0);
            vc_buf += used;
            nbyte_read += used;
            nbyte -= used;
         }
         vc_fsg.copr_body.params[0] = nbyte_read;
         vc_fsg.copr_body.params[1] = num_resp_bufs;
         rlen = sizeof(vc_fsg.copr_body.params) + resp_buffer-vc_fsg.copr_body.data;
         break;
      }

      case VC_FILESYS_READDIR:
        {
          struct dirent result;
          if (vc_hostfs_readdir_r((void *)vc_fsg.copr_body.params[0],
                                  &result) == NULL) {
            retval = VC_RESP_ERROR;
            rlen = 4;
          } else {
            rlen = 16+fs_host_direntbytestream_create(&result,
                                                      (void *)vc_fsg.copr_body.data);
          }
        }
        break;

      case VC_FILESYS_REMOVE:

         i = vc_hostfs_remove((const char *)vc_fsg.copr_body.data);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_RENAME:

         i = strlen((char *)vc_fsg.copr_body.data);
         if (vc_hostfs_rename((const char *)vc_fsg.copr_body.data,
                              (const char *)&vc_fsg.copr_body.data[i+1])
             != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_SETEND:

         i = vc_hostfs_setend(vc_fsg.copr_body.params[0]);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_SET_ATTR:

         i = vc_hostfs_set_attr((const char *)vc_fsg.copr_body.data,
                                vc_fsg.copr_body.params[0]);
         if (i != 0) {
           retval = VC_RESP_ERROR;
         }
         rlen = 0;
         break;

      case VC_FILESYS_TOTALSPACE:

         i = vc_hostfs_totalspace((const char *)vc_fsg.copr_body.data);
         if (i < 0) {
           retval = VC_RESP_ERROR;
           rlen = 0;
         } else {
           vc_fsg.copr_body.params[0] = i;
           rlen = 4;
         }
         break;

      case VC_FILESYS_WRITE:

         i = vc_hostfs_write(vc_fsg.copr_body.params[0],
                             vc_fsg.copr_body.data,
                             vc_fsg.copr_body.params[1] *
                             vc_fsg.copr_body.params[2]);
         if (i < 0) {
           retval = VC_RESP_ERROR;
         } else {
           vc_fsg.copr_body.params[0] = (int)(i/vc_fsg.copr_body.params[1]);
         }
         rlen = 4;
         break;

      default:
         vc_assert(0);
      }

      vc_fsg.copr_body.params[0] = VC_HTOV32(vc_fsg.copr_body.params[0]);
      vc_fsg.copr_body.params[1] = VC_HTOV32(vc_fsg.copr_body.params[1]);
      vc_fsg.copr_body.params[2] = VC_HTOV32(vc_fsg.copr_body.params[2]);
      vc_fsg.copr_body.params[3] = VC_HTOV32(vc_fsg.copr_body.params[3]);

      hdr.cmd_code = VC_HTOV32(retval);
      hdr.ext_length = VC_HTOV16(rlen);
      hdr.timestamp = VC_HTOV16(0);
      hdr.xid = VC_HTOV32(hdr.xid);
      ael = (rlen + 15) & ~15;

#ifdef PRINTF
      printf("Reply from the host\n");
      showmsg(&hdr, &vc_fsg.copr_body);
#endif

      vc_msgfifo_write_blocking(vc_fsg.inum, &hdr, sizeof(VC_MSGFIFO_CMD_HEADER_T), vc_fsg.out_ievent);
      if (ael) {
        vc_msgfifo_write_blocking(vc_fsg.inum, &vc_fsg.copr_body, ael, vc_fsg.out_ievent);
      }
      vc_msgfifo_write_flush(vc_fsg.inum);

    } else {
      /* A message has been left in the fifo and the host side has been reset.
         The message needs to be flushed. It would be better to do this by resetting
         the fifos. */
      if (ael > 0) {
        vc_assert(ael <= sizeof(struct file_service_msg_body));

        // Read the body
        vc_msgfifo_read_blocking(vc_fsg.inum, &vc_fsg.copr_body, ael, vc_fsg.in_ievent);
      }
      vc_msgfifo_read_flush(vc_fsg.inum);
#ifdef PRINTF
      printf("This message is spurious:\n");
      showmsg(&hdr, &vc_fsg.copr_body);
#endif
      /* Must continue to see if there is more stuff to obtain from the fifo */
      vc_msgfifo_read_refresh(vc_fsg.inum);
      continue;
    }
    break;
  }

  vc_lock_release(filesys_fifo_lock);
  return rr;
}


/******************************************************************************
NAME
   showmsg

SYNOPSIS
   static void showmsg(VC_MSGFIFO_CMD_HEADER_T const * head,
                       struct file_service_msg_body const * body)

FUNCTION
   De-bug tool: prints out fifo message.

RETURNS
   void
******************************************************************************/

#ifdef PRINTF
static void showmsg(VC_MSGFIFO_CMD_HEADER_T const * head,
                    struct file_service_msg_body const * body)
{
        unsigned int ael = (head->ext_length + 15) & ~15;
    printf("Sync=%08x XID=%08x Code=%08x Extlen=%08x (%d bytes follow)\n",
                   head->sync, head->xid, head->cmd_code, head->ext_length, ael);
    if (ael) {
                unsigned int i;
                printf("Content:");
                for(i = 0; i < 4 && i*4 < head->ext_length; ++i) printf(" %08x", body->params[i]);
                //for(i = 0; i+16 < head->ext_length; ++i) printf(" %02x", body->data[i]);
            if (head->ext_length > 16) printf(" plus %d bytes\n", head->ext_length);
    }
        printf("\n");
}
#endif


/******************************************************************************
NAME
   fs_host_direntbytestream_create

SYNOPSIS
   static int fs_host_direntbytestream_create(struct dirent *d, void *buffer)

FUNCTION
   Turns a variable of type struct dirent into a compiler independent byte
   stream, which is stored in buffer.

RETURNS
   Successful completion: The length of the byte stream
   Otherwise: -1
******************************************************************************/
static void write_bytestream(void *a, char *b, int n)
{
   int i;
   for (i=0;i<n;i++) {
      b[i] = ((char *)a)[i];
   }
   for (;i<4;i++) {
      b[i] = 0;
   }
}

static void memcpy_htov32(void*buffer,uint32_t h)
{
   uint32_t v = VC_HTOV32(h);
   memcpy(buffer,&v,sizeof(v));
}

static int fs_host_direntbytestream_create(struct dirent *d, void *buffer)
{
  char *buf = (char*)buffer;

   // Write d_name (D_NAME_MAX_SIZE chars)
   memcpy(buf, &d->d_name, D_NAME_MAX_SIZE);
        buf += D_NAME_MAX_SIZE;

        // Write d_size (int)
        write_bytestream((void *)&d->d_size, buf, sizeof(d->d_size));
        buf += 4;

        // Write d_attrib (int)
        write_bytestream((void *)&d->d_attrib, buf, sizeof(d->d_attrib));
        buf += 4;

        // Write d_modtime (time_t)
        write_bytestream((void *)&d->d_modtime, buf, sizeof(d->d_modtime));
        buf += 4;

        return (int)(buf-(char *)buffer);
}


/******************************************************************************
NAME
   fs_host_direntbytestream_interp

SYNOPSIS
   static void fs_host_direntbytestream_interp(struct dirent *d, void *buffer)

FUNCTION
   Turns a compiler independent byte stream back into a struct of type dirent.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/
static void read_bytestream(void *a, char *b, int n)
{
   int i;
   for (i=0;i<n;i++) {
      ((char *)a)[i] = b[i];
   }
}

static void fs_host_direntbytestream_interp(struct dirent *d, void *buffer)
{
  char *buf = (char*)buffer;

   // Read d_name (D_NAME_MAX_SIZE chars)
   memcpy(&d->d_name, buf, D_NAME_MAX_SIZE);
        buf += D_NAME_MAX_SIZE;

        // Read d_size (int)
        read_bytestream((void *)&d->d_size, buf, sizeof(d->d_size));
        d->d_size = VC_VTOH32(d->d_size);
        buf += 4;

        // Read d_attrib (int)
        read_bytestream((void *)&d->d_attrib, buf, sizeof(d->d_attrib));
        d->d_attrib = VC_VTOH32(d->d_attrib);
        buf += 4;

        // Read d_modtime (time_t)
        read_bytestream((void *)&d->d_modtime, buf, sizeof(d->d_modtime));
        d->d_modtime = VC_VTOH32(d->d_modtime);

        return;
}

/******************************************************************************
NAME
   vc_filesys_poll_message_fifo

SYNOPSIS
   int vc_filesys_poll_message_fifo(struct dirent *d, void *buffer)

FUNCTION
   Keep processing messages in the incoming fifo till none are left.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_filesys_poll_message_fifo(void)
{
   int r = 1;

   if (vc_fsg.inum < 0)
      return 0;

   while (r) {
      // this returns 0 when there is nothing more to do
      r = vc_fs_message_handler();
   }
   return r;
}

/******************************************************************************
NAME
   vc_filesys_read_event

SYNOPSIS
   void *vc_filesys_read_event(void)

FUNCTION
   Return the event for reading.

RETURNS
   void *
******************************************************************************/

void *vc_filesys_read_event(void) {
   return vc_fsg.in_ievent;
}

void vc_filesys_sendreset(void)
{
   vc_lock_obtain(filesys_lock);
   vc_fs_stub(VC_FILESYS_RESET, 0);
   vc_lock_release(filesys_lock);
}
