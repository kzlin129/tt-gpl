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



#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/unistd.h>
#include <linux/stat.h>
#include <linux/statfs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/limits.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/jiffies.h>
#include <linux/broadcom/vc.h>
#include "vchost.h"
#include "vcfilesys_defs.h"
#include "vc_fileservice_defs.h"

/******************************************************************************
Global data. 
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

#define DEBUG_LEVEL 1

#ifdef DEBUG_LEVEL
#if DEBUG_LEVEL > 1
#define DEBUG_MINOR(x) x  /* Debug level 2, enable major and minor debug */
#define DEBUG_MAJOR(x) x
#else
#define DEBUG_MAJOR(x) x  /* Debug level 1, enable only major debug stmts */
#define DEBUG_MINOR(x)
#endif
#else
#define DEBUG_MAJOR(x)    /* Debug level 0, do not compile debug statements */
#define DEBUG_MINOR(x)
#endif

/******************************************************************************
Static data.
******************************************************************************/

/******************************************************************************
Static functions.
******************************************************************************/

static void adjust_filename( const char *src, char *dst, size_t dstSize );

#define HOSTFS_STATS

#if defined(HOSTFS_STATS)

#define HOSTFS_STATS_MAX 16
typedef struct 
{
  uint64_t ts;
  uint64_t te;
  int writecount;
  int bytecount;
  int printcount;
} hostfs_stats_t;

static hostfs_stats_t hostfs_stats[HOSTFS_STATS_MAX];

#endif


/******************************************************************************
Global functions.
******************************************************************************/

/******************************************************************************
NAME
   vc_hostfs_init

SYNOPSIS
   void vc_hostfs_init(void)

FUNCTION
   Initialises the host to accept requests from VC02

RETURNS
   void
******************************************************************************/

void vc_hostfs_init(void)
{
   DEBUG_MINOR(printk("[vc_hostfs_init]\n"));
}

/******************************************************************************
NAME
   vc_hostfs_close

SYNOPSIS
   int vc_hostfs_close(int fildes)

FUNCTION
   Deallocates the file descriptor to a file.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_close(int fd)
{
   long ret = -1;
   int dt;
   
   DEBUG_MINOR(printk("[vc_hostfs_close(%d)]\n", fd));
   
   ret = sys_close(fd);

   //
#if defined(HOSTFS_STATS)
   // reset statistics for fd
   if((fd >=0) && (fd < HOSTFS_STATS_MAX))
     {
       hostfs_stats[fd].te = jiffies_to_msecs(jiffies);
       dt = (int) 
	 (hostfs_stats[fd].te > hostfs_stats[fd].ts) ?
	 (hostfs_stats[fd].te - hostfs_stats[fd].ts) : 
	 (~0 - hostfs_stats[fd].ts + hostfs_stats[fd].te);
       VC_DEBUG(Trace, "vc_hostfs_close: fd=%d ts=%llu te=%llu dt=%d count=%d bytecount=%d %d byte/msec\n", 
		fd, 
		hostfs_stats[fd].ts, 
		hostfs_stats[fd].te, 
		dt, 
		hostfs_stats[fd].writecount, 
		hostfs_stats[fd].bytecount,
		hostfs_stats[fd].bytecount / dt);
     }
#endif // HOSTFS_STATS
   
   return ret; 
}

/******************************************************************************
NAME
   vc_hostfs_lseek

SYNOPSIS
   long vc_hostfs_lseek(int fildes, long offset, int whence)

FUNCTION
   Sets the file pointer associated with the open file specified by fildes.  If
   the file is a FIFO (Linux does not support seeking on a FIFO) then, for the
   benefit of the VC02 streaming file handlers which do a number of null seeks,
   that is, seeks to the current position, the return value is faked without an 
   actual seek being done.

RETURNS
   Successful completion: offset
   Otherwise: -1
******************************************************************************/

long vc_hostfs_lseek(int fd, long offset, int origin)
{
  loff_t res;
  off_t retval;
  struct file * file;
  int fput_needed;
  
  DEBUG_MINOR(printk("[vc_hostfs_lseek(%d,%ld,%d)]\n", fd, offset, origin));
  retval = -EBADF;
  file = fget_light(fd, &fput_needed);
  if (!file)
    return -EBADF;
  if (origin <= SEEK_MAX) 
    {
      res = vfs_llseek(file, offset, origin);
      retval = res;
      if (res != (loff_t)retval)
	retval = -EOVERFLOW;       
    }
  fput_light(file, fput_needed);
  
  DEBUG_MINOR(printk("vc_hostfs_lseek returning %ld)\n", (long)retval));
  return retval;
}


/******************************************************************************
NAME
   vc_hostfs_lseek64

SYNOPSIS
   int64_t vc_hostfs_lseek64(int fildes, int64_t offset, int whence)

FUNCTION
   Sets the file pointer associated with the open file specified by fildes.  If
   the file is a FIFO (Linux does not support seeking on a FIFO) then, for the
   benefit of the VC02 streaming file handlers which do a number of null seeks,
   that is, seeks to the current position, the return value is faked without an 
   actual seek being done.


RETURNS
   Successful completion: offset
   Otherwise: -1
******************************************************************************/

int64_t vc_hostfs_lseek64(int fd, int64_t offset, int origin)
{
  off_t retval;
  struct file * file;
  int fput_needed;
   
  DEBUG_MINOR(printk("[vc_hostfs_lseek64(%d,%lld,%d)]\n", fd, offset, origin));
  retval = -EBADF;
  file = fget_light(fd, &fput_needed);
  if (!file)
    return -EBADF;;
  
  retval = -EINVAL;
  if (origin <= SEEK_MAX) 
    {
      loff_t res = vfs_llseek(file, offset, origin);
      retval = res;
      if (res != (loff_t)retval)
	retval = -EOVERFLOW;	
    }
  fput_light(file, fput_needed);
   
  DEBUG_MINOR(printk("[vc_hostfs_lseek64] returning %lld\n", (int64_t)retval));
  return retval;
}



/******************************************************************************
NAME
   vc_hostfs_open

SYNOPSIS
   int vc_hostfs_open(const char *path, int vc_oflag)

FUNCTION
   Establishes a connection between a file and a file descriptor.  For the benefit
   of faking out seeks on a FIFO, we will need to keep track of the read offset for
   all reads, and to facilitate this each opened file is given an entry in a local
   file info table.

RETURNS
   Successful completion: file descriptor
   Otherwise: -1

******************************************************************************/

int vc_hostfs_open(char *const_path, int vc_oflag)
{
   int flags = 0, ret=0;
   mm_segment_t old_fs;
   char path[FS_MAX_PATH];

   adjust_filename( const_path, path, sizeof( path ));
   
   flags = O_RDONLY;
   if (vc_oflag & VC_O_WRONLY)  flags =  O_WRONLY;
   if (vc_oflag & VC_O_RDWR)    flags =  O_RDWR;
   if (vc_oflag & VC_O_APPEND)  flags |= O_APPEND;
   if (vc_oflag & VC_O_CREAT)   flags |= O_CREAT;
   if (vc_oflag & VC_O_TRUNC)   flags |= O_TRUNC;
   if (vc_oflag & VC_O_EXCL)    flags |= O_EXCL;


   old_fs = get_fs();
   set_fs(get_ds());
   ret = sys_open(path, flags, 0777);
   set_fs(old_fs);
   
   VC_DEBUG(Trace, "vc_hostfs_open: path=%s ret=%d\n", path, ret);
   
   if (ret < 0 )
   {
     DEBUG_MINOR(printk("[vc_hostfs_open(%s,%d)] = %d\n", path, vc_oflag, ret));
   }
   else
     {
       DEBUG_MINOR(printk("[vc_hostfs_open(%s,%d)] = %d\n", path, vc_oflag, ret));
     }

#if defined(HOSTFS_STATS)
   // reset statistics for fd
   if((ret >= 0) && (ret < HOSTFS_STATS_MAX))
     {
       memset(hostfs_stats + ret, 0, sizeof(hostfs_stats[0]));
       hostfs_stats[ret].ts = jiffies_to_msecs(jiffies);
     }
#endif // HOSTFS_STATS
     
   return ret;
}


/******************************************************************************
NAME
   vc_hostfs_read

SYNOPSIS
   int vc_hostfs_read(int fildes, void *buf, unsigned int nbyte)

FUNCTION
   Attempts to read nbyte bytes from the file associated with the file
   descriptor, fildes, into the buffer pointed to by buf.  For the benefit
   of faking out seeks on a FIFO, we keep track of the read offset for all
   reads.

RETURNS
   Successful completion: number of bytes read
   Otherwise: -1
******************************************************************************/

int vc_hostfs_read(int fd, void *buf, unsigned int nbyte)
{
  mm_segment_t old_fs;
  ssize_t ret = -EBADF;

  //  DEBUG_MINOR(printk("[vc_hostfs_read(%d,%p,%u)]\n", fd, buf, nbyte));

  old_fs = get_fs();
  set_fs(get_ds());
  ret = sys_read(fd, (char __user*) buf, nbyte);
  set_fs(old_fs);

  DEBUG_MINOR(printk("[vc_hostfs_read(%d,%p,%u)] ret=%d\n", fd, buf, nbyte, ret));
  return ret;
}

/******************************************************************************
NAME
   vc_hostfs_write

SYNOPSIS
   int vc_hostfs_write(int fildes, const void *buf, unsigned int nbyte)

FUNCTION
   Attempts to write nbyte bytes from the buffer pointed to by buf to file
   associated with the file descriptor, fildes.

RETURNS
   Successful completion: number of bytes written
   Otherwise: -1 
******************************************************************************/


int vc_hostfs_write(int fd, const void *buf, unsigned int nbytes)
{
  mm_segment_t old_fs;
  struct file *file; 
  ssize_t ret;
  int fput_needed; 
  loff_t pos;
  uint64_t dt;

  ret = -EBADF;
  file = fget_light(fd, &fput_needed); 
  if (file) 
    { 
      pos = file->f_pos; 
      old_fs = get_fs();
      set_fs(get_ds());
      ret = vfs_write(file, (const char __user*)buf, nbytes, &pos); 
      set_fs(old_fs);
      file->f_pos = pos; 
      fput_light(file, fput_needed);
    } 
 
#if defined(HOSTFS_STATS)
   // reset statistics for fd
   if((fd >=0) && (fd < HOSTFS_STATS_MAX))
     {
       hostfs_stats[fd].te = jiffies_to_msecs(jiffies);
       dt = 
	 (hostfs_stats[fd].te >= hostfs_stats[fd].ts) ? 
	 (hostfs_stats[fd].te - hostfs_stats[fd].ts) :
	 (~0 - hostfs_stats[fd].ts + hostfs_stats[fd].te);
	 
       hostfs_stats[fd].writecount += 1;
       hostfs_stats[fd].bytecount += nbytes;
       if(0 == (hostfs_stats[fd].printcount % 1024))
	 {
	   VC_DEBUG(Trace, "fd=%d ts=%llu te=%llu dt=%llu count=%d bytecount=%d\n", 
		    fd, 
		    hostfs_stats[fd].ts, 
		    hostfs_stats[fd].te, 
		    hostfs_stats[fd].te - hostfs_stats[fd].ts, 
		    hostfs_stats[fd].writecount, 
		    hostfs_stats[fd].bytecount);
	 }
       hostfs_stats[fd].printcount += 1;
     }
#endif // HOSTFS_STATS

   DEBUG_MINOR(printk("[vc_hostfs_write] fd=%d nbytes=%d wrote=%d pos=%d\n", fd, nbytes, ret, (int)pos));
   return ret;
}

/******************************************************************************
NAME
   vc_hostfs_closedir

SYNOPSIS
   int vc_hostfs_closedir(void *dhandle)

FUNCTION
   Ends a directory list iteration.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_closedir(void *dhandle)
{
   (void)dhandle;
   printk( "vc_hostfs_closedir not implemented\n" );
   return -1;
}

/******************************************************************************
NAME
   vc_hostfs_format

SYNOPSIS
   int vc_hostfs_format(const char *path)

FUNCTION
   Formats the physical file system that contains path.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_format(const char *path)
{
   printk( "vc_hostfs_format: '%s' not implemented\n", path );
   return -1;
}

/******************************************************************************
NAME
   vc_hostfs_freespace

SYNOPSIS
   int vc_hostfs_freespace(const char *path)

FUNCTION
   Returns the amount of free space on the physical file system that contains
   path.

RETURNS
   Successful completion: free space
   Otherwise: -1
******************************************************************************/

int vc_hostfs_freespace(const char *const_path)
{

   int ret;
   struct statfs fsStat;
   mm_segment_t old_fs;
   char path[FS_MAX_PATH];

   adjust_filename( const_path, path, sizeof( path ));

   old_fs = get_fs();
   set_fs(get_ds());
   ret = sys_statfs( path, &fsStat );
   set_fs(old_fs);

   if (ret == 0)
   {
      uint64_t freeSpace = fsStat.f_bsize * fsStat.f_bavail;

      // Saturate return value (need this in case we have a large file system)
      if (freeSpace > (uint64_t) INT_MAX)
      {
         ret = INT_MAX;
      }
      else
      {
         ret = (int) freeSpace;
      }
   }
   else
   {
      ret = -1;
   }

   DEBUG_MINOR(printk( "[vc_hostfs_freespace] for '%s' returning %d\n", path, ret ));

   return ret;
}

int64_t vc_hostfs_freespace64(const char *path)
{
  return vc_hostfs_freespace(path);
}

/******************************************************************************
NAME
   vc_hostfs_get_attr

SYNOPSIS
   int vc_hostfs_get_attr(const char *path, fattributes_t *attr)

FUNCTION
   Gets the file/directory attributes.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_get_attr(const char *path, fattributes_t *attr)
{
   (void)path;
   (void)attr;
   printk( "vc_hostfs_get_attr not implemented\n" );
   return -1;
/*
    struct stat sb;

    printk( "vc_hostfs_get_attr: '%s'\n", path );


    *attr = 0;

    if ( sys_stat( path, &sb ) == 0 )
    {
        if ( S_ISDIR( sb.st_mode ))
        {
            *attr |= ATTR_DIRENT;
        }

        if (( sb.st_mode & S_IWUSR  ) == 0 )
        {
            *attr |= ATTR_RDONLY;
        }

        return 0;
    }
    return -1;
*/
}

/******************************************************************************
NAME
   vc_hostfs_mkdir

SYNOPSIS
   int vc_hostfs_mkdir(const char *path)

FUNCTION
   Creates a new directory named by the pathname pointed to by path.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_mkdir(const char *path)
{
    printk( "vc_hostfs_mkdir: '%s'is not implemented \n",  path );
    
    /*if ( mkdir( path, 0777 ) == 0 )
    {
        return 0;
    }*/
    return -1;
}

/******************************************************************************
NAME
   vc_hostfs_opendir

SYNOPSIS
   void *vc_hostfs_opendir(const char *dirname)

FUNCTION
   Starts a directory list iteration of sub-directories.

RETURNS
   Successful completion: dhandle (pointer)
   Otherwise: NULL
******************************************************************************/

void *vc_hostfs_opendir(const char *dirname)
{
   printk( "vc_hostfs_opendir: '%s' not implemented\n", dirname );
   return NULL;
}

/******************************************************************************
NAME
   vc_hostfs_readdir_r

SYNOPSIS
   struct dirent *vc_hostfs_readdir_r(void *dhandle, struct dirent *result)

FUNCTION
   Fills in the passed result structure with details of the directory entry
   at the current psition in the directory stream specified by the argument
   dhandle, and positions the directory stream at the next entry. If the last
   sub-directory has been reached it ends the iteration and begins a new one
   for files in the directory.

RETURNS
   Successful completion: result
   End of directory stream: NULL
******************************************************************************/

struct vc03_dirent *vc_hostfs_readdir_r(void *dhandle, struct vc03_dirent *result)
{
   (void)dhandle;
   (void)result;
   printk( "vc_hostfs_readdir_r not implemented\n" );
   return NULL;
}

/******************************************************************************
NAME
   vc_hostfs_remove

SYNOPSIS
   int vc_hostfs_remove(const char *path)

FUNCTION
   Removes a file or a directory. A directory must be empty before it can be
   deleted.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_remove(const char *path)
{
   (void)path;
   printk( "vc_hostfs_remove not implemented\n" );
   return -1;

   /* printk( "vc_hostfs_remove: '%s'\n", path );

    if ( unlink( path ) == 0 )
    {
        return 0;
    }
    return -1;
    */
}

/******************************************************************************
NAME
   vc_hostfs_rename

SYNOPSIS
   int vc_hostfs_rename(const char *old, const char *new)

FUNCTION
   Changes the name of a file. The old and new pathnames must be on the same
   physical file system.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_rename(const char *old, const char *new)
{
   (void)old;
   (void)new;
   printk( "vc_hostfs_rename not implemented\n" );
   return -1;

/*    printk( "vc_hostfs_rename: '%s' to '%s'\n", old, new );

    if ( rename( old, new ) == 0 )
    {
        return 0;
    }
*/
   return -1;
}

/******************************************************************************
NAME
   vc_hostfs_set_attr

SYNOPSIS
   int vc_hostfs_set_attr(const char *path, fattributes_t attr)

FUNCTION
   Sets file/directory attributes.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_set_attr(const char *path, fattributes_t attr)
{
   (void)path;
   (void)attr;
   printk( "vc_hostfs_set_attr not implemented\n" );
   return -1;

/*
    int mode = 0;
    struct stat sb;

    if ( sys_stat( path, &sb ) == 0 )
    {
        mode = sb.st_mode;

        if ( attr & ATTR_RDONLY )
        {
            mode &= ~S_IWUSR;
        }
        else
        {
            mode |= S_IWUSR;
        }

        if ( sys_chmod( path, mode ) == 0 )
        {
            return 0;
        }
    }
    
   return -1;
*/
}

/******************************************************************************
NAME
   vc_hostfs_setend

SYNOPSIS
   int vc_hostfs_setend(int fildes)

FUNCTION
   Truncates file at current position.

RETURNS
   Successful completion: 0
   Otherwise: -1
******************************************************************************/

int vc_hostfs_setend(int filedes)
{   
   (void)filedes;
   printk( "vc_hostfs_setend( not implemented\n" );
   return -1;

/*  off_t   currPosn;
                                 
    if (( currPosn = sys_lseek( filedes, 0, SEEK_CUR )) != (off_t)-1 )
    {
        if ( sys_ftruncate( filedes, currPosn ) == 0 )
        {
            return 0;
        }
    }
*/
   return -1;

}

/******************************************************************************
NAME
   vc_hostfs_totalspace

SYNOPSIS
   int vc_hostfs_totalspace(const char *path)

FUNCTION
   Returns the total amount of space on the physical file system that contains
   path.

RETURNS
   Successful completion: total space
   Otherwise: -1
******************************************************************************/

int vc_hostfs_totalspace(const char *const_path)
{
   int ret;
   struct statfs fsStat;
   mm_segment_t old_fs;
   char path[FS_MAX_PATH];

   adjust_filename( const_path, path, sizeof( path ));

   old_fs = get_fs();
   set_fs(get_ds());
   ret = sys_statfs( path, &fsStat );
   set_fs(old_fs);

   if (ret == 0)
   {
      uint64_t totalSpace = fsStat.f_bsize * fsStat.f_blocks;

      // Saturate return value (need this in case we have a large file system)
      if (totalSpace > (uint64_t) INT_MAX)
      {
         ret = INT_MAX;
      }
      else
      {
         ret = (int) totalSpace;
      }
   }
   else
   {
      ret = -1;
   }

   DEBUG_MINOR(printk( "[vc_hostfs_totalspace] for '%s' returning %d\n", path, ret ));

   return ret;
}

/******************************************************************************
NAME
   adjust_filename

SYNOPSIS
   void adjust_filename( const char *src, char *dst, size_t dstSize )
   void backslash_to_slash( char *s )

FUNCTION
   Convert all '\' in a string to '/', and strip off any leading /hfs

RETURNS
   None.
******************************************************************************/

static void adjust_filename( const char *src, char *dst, size_t dstSize )
{
    // We expect that all of the filenames we're dealing with to start
    // with /hfs. Strip it off.

    vc_assert( src != NULL );

    if (( strncmp( src, "/hfs",  4 ) == 0 )
    ||  ( strncmp( src, "\\hfs", 4 ) == 0 ))
    {
        src += 4;
    }
    else
    {
        DEBUG_MINOR( printk( "Invalid path, expected /hfs prefix: '%s'?\n", path ));
    }

    while ( --dstSize > 0 )
    {
        if ( *src == '\\' )
        {
            // Translate backslash to forward slash

            *dst++ = '/';
        }
        else
        {
            // Otherwise copy and terminate on null character

            if (( *dst++ = *src ) == '\0' )
            {
                break;
            }
        }
        src++;
    }
}





