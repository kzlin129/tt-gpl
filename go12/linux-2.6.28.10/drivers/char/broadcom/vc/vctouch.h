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




#ifndef VCTOUCH_H
#define VCTOUCH_H

/* Initialise touch service. Returns it's interface number. This initialises
   the host side of the interface, it does not send anything to VideoCore. */

int vc_touch_init(void);

/* Re-read VideoCore memory and return values of touch, xpos, ypos. */

void vc_touch_fetch(VC_TOUCH_PARAMS_T *params);

#endif
