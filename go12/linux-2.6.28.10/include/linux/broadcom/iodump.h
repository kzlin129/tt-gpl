/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
*****************************************************************************
*
*  iodump.h
*
*  PURPOSE:
*
*     This file contains the interface to the iodump field debug tool
*
*  NOTES:
*
*****************************************************************************/

#if !defined( IODUMP_H )
#define IODUMP_H

/* ---- Include Files ---------------------------------------- */
#ifndef WIN32
#include <linux/spinlock.h>
#endif
/* ---- Constants and Types ---------------------------------- */


#define IODUMP_DEFAULT_BUFSIZE 0x10000     // Capture buffer in RAM default size
#define IODUMP_MAXMSGLEN        1024
#define IODUMP_IOCTL_LOGMSG     _IO( 'D', 0x80 )   // arg is char *

typedef enum
{
    IODUMP_LOG              =   1,
    IODUMP_NET_INGRESS      =   2,
    IODUMP_NET_EGRESS       =   3,
    IODUMP_TDM_INGRESS      =   4,
    IODUMP_TDM_EGRESS       =   5,
    IODUMP_TDM_INGRESS_WB   =   6,
    IODUMP_TDM_EGRESS_WB    =   7
}
IODUMP_TYPE;

#define IODUMP_MAGIC 0xBABEFACE

typedef struct
{
   int magic;           /* IODUMP_MAGIC */
   int timestamp;       /* system timestamp */
   short length;        /* payload length not including this header */
   char stream;         /* streamId for network traffic, 0 otherwise */
   char type;           /* IODUMP_TYPE */
}
IODUMP_HDR;

typedef struct
{
    int ingress;
    int egress;
}
IODUMP_CTRL;

#define IODUMP_MAX_STREAMS  4   // must match EPTCFG_MAX_STREAMS

typedef struct
{
    char *bufp;                                 // The ram capture buffer base
    char *bufendp;                              // The ram capture buffer end + 1
    int bufsize;                                // ram capture buffer size
    char *bufwritep;                            // next write point to capture buffer
    char *bufreadp;                             // next read point from capture buffer
    int logfile;                                // flag to log to 'filename'
    char filename[128];                         // filename to dump to
    struct file *file;                          // file handle
    char ipaddr[32];                            // ip address string "x.x.x.x"
    int port;                                   // port to send to
    int enable;                                 // turn dump on/off
    IODUMP_CTRL stream[IODUMP_MAX_STREAMS];     // individual control for each stream
    IODUMP_CTRL tdm;                            // individual control for tdm
    struct socket *sock;                        // socket to dump to (not in sysctl)
    int dumpstate;                              // flag to dump state information
    int dumpbuf;                                // flag to dump ram capture buffer
    int read_timeout;                           // thread wakeup time in jiffies
#ifndef WIN32
    spinlock_t netDumpLock;                     // spinlock used to prevent ingress and egress threads from interrupting each other
#endif
}
IODUMP_OBJ;

/* ---- Variable Externs ------------------------------------- */
extern void iodump_write(int streamId, IODUMP_TYPE type, short length, void *bufp);
extern void iodump_write2(int streamId, IODUMP_TYPE type, short length, volatile short *bufp);

/* ---- Function Prototypes ---------------------------------- */

#endif /* IODUMP_H */

