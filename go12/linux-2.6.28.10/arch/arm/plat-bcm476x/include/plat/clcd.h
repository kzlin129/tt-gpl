/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
#ifndef __BCM476X_CLCD_H__
#define __BCM476X_CLCD_H__

typedef struct clcd_device *(*clcd_lookup_t) (int);

void    bcm476x_clcd_lookup_register(clcd_lookup_t lookup_func);
void	bcm476x_amba_lcd_init (void);

#endif