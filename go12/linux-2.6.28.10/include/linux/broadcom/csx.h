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
*  csx.h
*
*  PURPOSE:
*
*     This file contains the CSX interface debug tool
*
*  NOTES:
*
*****************************************************************************/

#if !defined( CSX_H )
#define CSX_H

/* ---- Include Files ---------------------------------------- */
/* ---- Constants and Types ---------------------------------- */

typedef int (*CSX_INJECT_FP)(char *buf, int len, void *data);
typedef int (*CSX_CAPTURE_FP)(char *buf, int len, void *data);

typedef struct csx_io_point_fncs
{
   CSX_INJECT_FP    csxInject;
   CSX_CAPTURE_FP   csxCapture;
} CSX_IO_POINT_FNCS;


/* ---- Variable Externs ------------------------------------- */
/* ---- Function Prototypes ---------------------------------- */

#endif /* CSX_H */

