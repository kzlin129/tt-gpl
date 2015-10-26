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




#ifndef VCERRORS_H
#define VCERRORS_H

// Enum to aid interpreting errors returned by VideoCore in general command responses.
// Their meanings are indicated in the comments, but only a few of them are ever likely
// to be of interest to end users. The error number should be returned in
// a "error=<number>" field of the response.

typedef enum {
   VC_ERR_NONE = 0,    // no error has occurred

   VC_ERR_NO_CMD,      // no such command exists
   VC_ERR_BAD_PARAMS,  // command parameters were bad in some way
   VC_ERR_BAD_CMD,     // command is inappropriate in some way

   VC_ERR_NO_MEMORY,   // insufficient memory to continue with request
   VC_ERR_NO_VLL,      // a required VLL is missing
   VC_ERR_BAD_VLL,     // VLL is bad in some way

   VC_ERR_NO_SOURCE,     // a named file/url does not appear to exist
   VC_ERR_UNIMPL_SOURCE, // the data in the source is unsupported/unimplemented
   VC_ERR_BAD_SOURCE,    // the file/url is incomplete or invalid in some way

   VC_ERR_AUDIO_BUSY,  // VideoCore audio was required but is in use

   VC_ERR_INTERRUPTED, // some operation has been interrupted

   VC_ERR_NO_SPACE,    // insufficient space on filesystem or similar

   VC_ERR_NO_SIGNAL,   // used by DMB to indicate a transmission signal failure

   // New error types of error should be added right here:
   // ...

   VC_ERR_UNKNOWN=9999 // another kind of error has occurred
} VC_ERR_T;

#endif
