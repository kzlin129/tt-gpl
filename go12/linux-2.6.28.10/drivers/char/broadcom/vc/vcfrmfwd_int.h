/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    vcfrmfwd_int.h
*
*  @brief   Contains function prototypes for internal functions needed for frame forwarding service
*
*
****************************************************************************/

#ifndef VCFRMFWD_INT_H
#define VCFRMFWD_INT_H

/* Initialize the frame forwarding service. */
extern int vc_frmfwd_init (void);

/* Stop the service from being used. */
extern void vc_frmfwd_stop(void);

/* Return the service number (-1 if not running). */
extern int vc_frmfwd_inum(void);

/* Poll host bound message fifo and process any messages there */
extern int vc_frmfwd_poll_message_fifo(void);

/* Return the event used to wait for reads. */
void *vc_frmfwd_read_event(void);

#endif
