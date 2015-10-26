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




#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>
#include <limits.h>

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
static void backslash_to_slash( char *s );

/*
 *  The media player on the VC02 may be asked to open a file on the Host that 
 *  is in fact a FIFO.  We need to note when a FIFO has been opened so that we 
 *  can fake out some FIFO seeks that the VC02 may perform, hence the following
 *  types and variables.
 */

typedef struct
{
   int is_fifo;            // non-zero if file is a FIFO
   long read_offset;       // read offset into file
} file_info_t;

static file_info_t *p_file_info_table = NULL;
static int file_info_table_len = 0;
#define FILE_INFO_TABLE_CHUNK_LEN   20

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
   DEBUG_MINOR(printf("vc_hostfs_init\n"));
   // Allocate memory for the file info table
   p_file_info_table = (file_info_t *)calloc( FILE_INFO_TABLE_CHUNK_LEN, sizeof( file_info_t ) );
   assert( p_file_info_table != NULL );
   if (p_file_info_table)
   {
      file_info_table_len = FILE_INFO_TABLE_CHUNK_LEN;
   }
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

int vc_hostfs_close(int fildes)
{
   DEBUG_MINOR(printf("vc_hostfs_close(%d)\n", fildes));
   return close(fildes);
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

long vc_hostfs_lseek(int fildes, long offset, int whence)
{
   DEBUG_MINOR(printf("vc_hostfs_lseek(%d,%ld,%d)\n", fildes, offset, whence));
   if (fildes >= file_info_table_len)
   {
      // File descriptor not in table, so this is an error
      DEBUG_MAJOR(printf("vc_hostfs_lseek: invalid fildes %d\n", fildes));
      return -1;
   }
   else
   {
      // There is entry in the file info table for this file descriptor, so go
      // ahead and handle the seek
      long read_offset = p_file_info_table[fildes].read_offset;

      if (p_file_info_table[fildes].is_fifo)
      {
         // The VC02 is attempting to seek on a FIFO.  FIFOs don't support seeking
         // but, for the benefit of certain VC02 "streaming" file handlers, we 
         // will fake limited FIFO seek functionality by computing where a seek
         // would take us to
         if (whence == SEEK_SET)
         {
            read_offset = offset;
         }
         else if (whence == SEEK_CUR)
         {
            read_offset += offset;         
         }
         else
         {
            // seeking to the end of FIFO makes no sense, so this is an error
            DEBUG_MAJOR(printf("vc_hostfs_lseek(%d,%ld,%d): SEEK_END not supported on FIFO\n", fildes, offset, whence));
            return -1;
         }
      }
      else
      {
         // File is not a FIFO, so do the seek
         read_offset = lseek(fildes, offset, whence);
      }
      p_file_info_table[fildes].read_offset = read_offset;
      DEBUG_MINOR(printf("vc_hostfs_lseek returning %ld)\n", read_offset));
      return read_offset;
   }
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

int vc_hostfs_open(const char *inPath, int vc_oflag)
{
   char *path = strdup( inPath );
   char *s;
   int flags = 0, ret=errno;
   struct stat fileStat;

   // Replace all '\' with '/'
   backslash_to_slash( path );

   s = path + strlen( path );
   if (( s - path ) >= 4 )
   {
      if ( strcasecmp( &s[ -4 ], ".vll" ) == 0 )
      {
         // The VC02 is asking for a .vll file. Since it isn't consistent with
         // the case, we convert .vll files to all lowercase.

         s--;	 // backup to the last character (*s is on the '\0')
         while (( s >= path ) && ( *s != '/' ))
         {
            *s = tolower( *s );
            s--;
         }
      }
   }

   printf( "vc_hostfs_open: '%s'\n", path );

   flags = O_RDONLY;
   if (vc_oflag & VC_O_WRONLY)  flags =  O_WRONLY;
   if (vc_oflag & VC_O_RDWR)    flags =  O_RDWR;
   if (vc_oflag & VC_O_APPEND)  flags |= O_APPEND;
   if (vc_oflag & VC_O_CREAT)   flags |= O_CREAT;
   if (vc_oflag & VC_O_TRUNC)   flags |= O_TRUNC;
   if (vc_oflag & VC_O_EXCL)    flags |= O_EXCL;

   //while (*path == '\\') path++; // do not want initial '\'
   if (flags & O_CREAT)
      ret = open(path, flags, S_IREAD | S_IWRITE );
   else
      ret = open(path, flags );

   if (ret < 0 )
   {
      DEBUG_MINOR(printf("vc_hostfs_open(%s,%d) = %d\n", path, vc_oflag, ret));
   }
   else
   {
      DEBUG_MINOR(printf("vc_hostfs_open(%s,%d) = %d\n", path, vc_oflag, ret));
      ;
   }

   // If the file was successfully open then initialize its entry in 
   // the file info table.  If necessary, we expand the size of the table
   if (ret >= 0)
   {
      // File was successfully opened
      if (ret >= file_info_table_len)
      {
         file_info_t *p_new_file_info_table = p_file_info_table;
         int new_file_info_table_len = file_info_table_len;
   
         // try and allocate a bigger buffer for the file info table
         new_file_info_table_len += FILE_INFO_TABLE_CHUNK_LEN;
         p_new_file_info_table = calloc( new_file_info_table_len, sizeof( file_info_t ) );
         if (p_new_file_info_table == NULL)
         {
            // calloc failed
            DEBUG_MAJOR(printf("vc_hostfs_open: file_info_table calloc failed\n"));
            assert( 0 );
         }
         else
         {
            // calloc successful, so copy data from previous buffer to new buffer,
            // free previous buffer and update ptr and len info
            memcpy( p_new_file_info_table, p_file_info_table, sizeof( file_info_t ) * file_info_table_len );
            free( p_file_info_table );
            p_file_info_table = p_new_file_info_table;
            file_info_table_len = new_file_info_table_len;
         }
      }
      assert( ret < file_info_table_len );
      {
         // initialize this file's entry in the file info table
         p_file_info_table[ret].is_fifo = 0;
         p_file_info_table[ret].read_offset = 0;
      }
   
      // Check whether the file is a FIFO.  A FIFO does not support seeking
      // but we will fake, to the extent supported by the buffered file system
      // on the VC02, limited FIFO seek functionality.  This is for the benefit
      // of certain VC02 "streaming" file handlers.
      fstat( ret, &fileStat );
      if (S_ISFIFO( fileStat.st_mode ))
      {
         // file is a FIFO, so note its fildes for future reference
         p_file_info_table[ret].is_fifo = 1;
         DEBUG_MINOR(printf("vc_hostfs_open: file with fildes %d is a FIFO\n", ret));
      }
   }

   free( path );

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

int vc_hostfs_read(int fildes, void *buf, unsigned int nbyte)
{
   if (fildes >= file_info_table_len)
   {
      // File descriptor not in table, so this is an error
      DEBUG_MAJOR(printf("vc_hostfs_read(%d,%p,%u): invalid fildes\n", fildes, buf, nbyte));
      return -1;
   }
   else
   {
      // There is entry in the file info table for this file descriptor, so go
      // ahead and handle the read
      int ret = read(fildes, buf, nbyte);
      DEBUG_MINOR(printf("vc_hostfs_read(%d,%p,%u) = %d\n", fildes, buf, nbyte, ret));
      if (ret > 0)
      {
         p_file_info_table[fildes].read_offset += (long) ret;
      }
      return ret;
   }
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

int vc_hostfs_write(int fildes, const void *buf, unsigned int nbyte)
{
   int ret = write(fildes, buf, nbyte);
   DEBUG_MINOR(printf("vc_hostfs_write(%d,%p,%u) = %d\n", fildes, buf, nbyte, ret));
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
   printf( "vc_hostfs_closedir not implemented\n" );
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
   printf( "vc_hostfs_format: '%s' not implemented\n", path );
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

int vc_hostfs_freespace(const char *inPath)
{
   char *path = strdup( inPath );
   int ret;
   struct statfs fsStat;

   // Replace all '\' with '/'
   backslash_to_slash( path );

   ret = statfs( path, &fsStat );

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

   DEBUG_MINOR(printf( "vc_hostfs_freespace for '%s' returning %d\n", path, ret ));

   free( path );
   return ret;
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
    struct stat sb;

    printf( "vc_hostfs_get_attr: '%s'\n", path );


    *attr = 0;

    if ( stat( path, &sb ) == 0 )
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
    printf( "vc_hostfs_mkdir: '%s'\n",  path );
    if ( mkdir( path, 0777 ) == 0 )
    {
        return 0;
    }
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
   printf( "vc_hostfs_opendir: '%s' not implemented\n", dirname );
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

struct dirent *vc_hostfs_readdir_r(void *dhandle, struct dirent *result)
{
   (void)dhandle;
   (void)result;
   printf( "vc_hostfs_readdir_r not implemented\n" );
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
    printf( "vc_hostfs_remove: '%s'\n", path );

    if ( unlink( path ) == 0 )
    {
        return 0;
    }
    return -1;
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
    printf( "vc_hostfs_rename: '%s' to '%s'\n", old, new );

    if ( rename( old, new ) == 0 )
    {
        return 0;
    }
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
    int mode = 0;
    struct stat sb;

    if ( stat( path, &sb ) == 0 )
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

        if ( chmod( path, mode ) == 0 )
        {
            return 0;
        }
    }
   return -1;
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
    off_t   currPosn;

    if (( currPosn = lseek( filedes, 0, SEEK_CUR )) != (off_t)-1 )
    {
        if ( ftruncate( filedes, currPosn ) == 0 )
        {
            return 0;
        }
    }
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

int vc_hostfs_totalspace(const char *inPath)
{
   char *path = strdup( inPath );
   int ret;
   struct statfs fsStat;

   // Replace all '\' with '/'
   backslash_to_slash( path );

   ret = statfs( path, &fsStat );

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

   DEBUG_MINOR(printf( "vc_hostfs_totalspace for '%s' returning %d\n", path, ret ));

   free( path );
   return ret;
}

/******************************************************************************
NAME
   backslash_to_slash

SYNOPSIS
   void backslash_to_slash( char *s )

FUNCTION
   Convert all '\' in a string to '/'.

RETURNS
   None.
******************************************************************************/

static void backslash_to_slash( char *s )
{
   while ( *s != '\0' )
   {
       if ( *s == '\\' )
       {
           *s = '/';
       }
       s++;
   }
}




