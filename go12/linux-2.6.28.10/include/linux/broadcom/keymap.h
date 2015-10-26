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




#ifndef KEYMAP_H
#define KEYMAP_H


typedef struct
{
   unsigned int scancode;
   unsigned int keycode;
} KEYMAP;

typedef struct
{
    unsigned int    gpio;           // GPIO LED is connected to
    unsigned int    activeHigh;     // LED is on when GPIO has this value
    unsigned int    ledNum;         // LED number (0-16) in input driver
    const char     *label;          // Label for registering with gpiolib
} LEDMAP;

#endif
