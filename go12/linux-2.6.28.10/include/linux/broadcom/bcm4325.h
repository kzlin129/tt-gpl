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



#ifndef __BCM4325_H
#define __BCM4325_H

void bcm4325_enable_wl(int wl);
void bcm4325_enable_bt(int bt);

#define BCM4325_WIRELESSLAN   0
#define BCM4325_BLUETOOTH     1

typedef struct
{
   unsigned int pin_num; /* GPIO pin */
   unsigned int disable_val; /* GPIO value for disable/init */
   unsigned int enable_val; /* GPIO value for enable */
   unsigned int disable_delay; /* disable time delay in ms */
   unsigned int enable_delay; /* enable time delay in ms */
} BCM4325_GPIO_PIN_MAP;

#endif /* __BCM4325_H */
