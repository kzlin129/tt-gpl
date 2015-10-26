/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
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
*  bcm_platinfo.h
*
*  PURPOSE:
*
* This file contains platform data specific to the BCM476X architecture. It
* defines valid settings for the device IDs, device revisions, board IDs and
* board revisions.
*
* It is intended to assist in allowing a single binary to execute on different
* hardware platforms based off of BCM476x family devices. It is not intended
* to support platforms based off of widely different devices other than
* BCM476x family devices or related derivatives.
*
*****************************************************************************/


#if !defined( BCM_PLATINFO_H )
#define BCM_PLATINFO_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>

/* ---- Constants and Types ---------------------------------------------- */

typedef enum bcm_platid_e {
	BCM4760=10
} bcm_platid_t;

typedef enum bcm_board_e {
	CATALINA10=0,			/* P1 revision Catalina */
	CATALINA11=1,			/* P3 revision Catalina */
	CATALINA20=2,			/* P4 revision Catalina */
	NICOLAS10=10,			/* P1 revision Nicolas */
	NICOLAS11=11,			/* P3 revision Nicolas */
	ROSA10=20				/* P1 revision Rosa */
} bcm_board_t;

typedef enum bcm476x_board_id_e {
	
} bcm476x_board_id_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function and Macro Prototypes ------------------------------------ */


#endif  /* BCM_PLATINFO_H */
