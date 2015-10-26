/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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



/**
*
*  @file    iodump_stub.c
*
*  @brief   iodump stub implementation.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/broadcom/iodump.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
static IODUMP_OBJ IoDump;
/* ---- Functions -------------------------------------------------------- */

void iodump_write(int streamId, IODUMP_TYPE type, short length, void *bufp)
{
   (void)streamId;
   (void)type;
   (void)length;
   (void)bufp;
}

EXPORT_SYMBOL(iodump_write);

void iodump_write2(int streamId, IODUMP_TYPE type, short length, volatile short *bufp)
{
   (void)streamId;
   (void)type;
   (void)length;
   (void)bufp;
}

EXPORT_SYMBOL(iodump_write2);

unsigned int iodump_inuse(void)
{
   return 0;
}

unsigned int iodump_room(void)
{
   return 0;
}

int __init iodump_init(void)
{
   memset(&IoDump, 0, sizeof(IoDump));
   return 0;
}

/****************************************************************************/

module_init(iodump_init);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM116x IoDump Driver");
