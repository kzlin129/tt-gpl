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
#ifndef __IRVINE_LCD_H__
#define __IRVINE_LCD_H__

struct clcd_paneldev {
	struct list_head	list;

	struct clcd_device	*panel;
	unsigned int		lcd_mdiv;
};

extern void irvine_clcd_register(struct clcd_paneldev *dev);
extern void irvine_clcd_unregister(struct clcd_paneldev *dev);
extern void irvine_clcd_init (void);

#endif /* __IRVINE_LCD_H__ */

