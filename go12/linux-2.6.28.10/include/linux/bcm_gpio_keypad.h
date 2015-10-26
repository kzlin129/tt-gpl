/*****************************************************************************
* Copyright 2007 - 2008 Broadcom Corporation.  All rights reserved.
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


#ifndef BCM_GPIO_KEYPAD_H
#define BCM_GPIO_KEYPAD_H

struct BCM_GPIO_KEYMAP {
    int     gpio_row;
    int     gpio_col;
    const char *name;
    int     key_code;
}; 

struct BCM_KEYPAD_PLATFORM_DATA {
    int      array_size;
    struct BCM_GPIO_KEYMAP     *keymap;
};

#endif
