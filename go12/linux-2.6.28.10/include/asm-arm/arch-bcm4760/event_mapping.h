/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 	Copy of this header file is in user space and kernel. Both copies should be in sync
 	If you modify this file please make sure to edit the file in user space
 	and vice-versa
 */

#ifndef __BCM47XX_UEVENT_MAPPING_H
#define __BCM47XX_UEVENT_MAPPING_H

typedef enum
{
	BCM47XX_UIN_LC_A = 'a',
	BCM47XX_UIN_LC_B = 'b',
	BCM47XX_UIN_LC_C = 'c',
	BCM47XX_UIN_LC_D = 'd',
	BCM47XX_UIN_LC_E = 'e',
	BCM47XX_UIN_LC_F = 'f',
	BCM47XX_UIN_LC_G = 'g',
	BCM47XX_UIN_LC_H = 'h',
	BCM47XX_UIN_LC_I = 'i',
	BCM47XX_UIN_LC_J = 'j',
	BCM47XX_UIN_LC_K = 'k',
	BCM47XX_UIN_LC_L = 'l',
	BCM47XX_UIN_LC_M = 'm',
	BCM47XX_UIN_LC_N = 'n',
	BCM47XX_UIN_LC_O = 'o',
	BCM47XX_UIN_LC_P = 'p',
	BCM47XX_UIN_LC_Q = 'q',
	BCM47XX_UIN_LC_R = 'r',	
	BCM47XX_UIN_LC_S = 's',
	BCM47XX_UIN_LC_T = 't',
	BCM47XX_UIN_LC_U = 'u',
	BCM47XX_UIN_LC_V = 'v',
	BCM47XX_UIN_LC_W = 'w',
	BCM47XX_UIN_LC_X = 'x',
	BCM47XX_UIN_LC_Y = 'y',
	BCM47XX_UIN_LC_Z = 'z',

	BCM47XX_UIN_UC_A = 'A',
	BCM47XX_UIN_UC_B = 'B',
	BCM47XX_UIN_UC_C = 'C',
	BCM47XX_UIN_UC_D = 'D',
	BCM47XX_UIN_UC_E = 'E',
	BCM47XX_UIN_UC_F = 'F',
	BCM47XX_UIN_UC_G = 'G',
	BCM47XX_UIN_UC_H = 'H',
	BCM47XX_UIN_UC_I = 'I',
	BCM47XX_UIN_UC_J = 'J',
	BCM47XX_UIN_UC_K = 'K',
	BCM47XX_UIN_UC_L = 'L',
	BCM47XX_UIN_UC_M = 'M',
	BCM47XX_UIN_UC_N = 'N',
	BCM47XX_UIN_UC_O = 'O',
	BCM47XX_UIN_UC_P = 'P',
	BCM47XX_UIN_UC_Q = 'Q',
	BCM47XX_UIN_UC_R = 'R',	
	BCM47XX_UIN_UC_S = 'S',
	BCM47XX_UIN_UC_T = 'T',
	BCM47XX_UIN_UC_U = 'U',
	BCM47XX_UIN_UC_V = 'V',
	BCM47XX_UIN_UC_W = 'W',
	BCM47XX_UIN_UC_X = 'X',
	BCM47XX_UIN_UC_Y = 'Y',
	BCM47XX_UIN_UC_Z = 'Z',

	BCM47XX_UIN_NUM_1 = '1',
	BCM47XX_UIN_NUM_2 = '2',
	BCM47XX_UIN_NUM_3 = '3',
	BCM47XX_UIN_NUM_4 = '4',
	BCM47XX_UIN_NUM_5 = '5',
	BCM47XX_UIN_NUM_6 = '6',
	BCM47XX_UIN_NUM_7 = '7',
	BCM47XX_UIN_NUM_8 = '8',
	BCM47XX_UIN_NUM_9 = '9',
	BCM47XX_UIN_NUM_0 = '0',

	BCM47XX_UIN_TILDA = '~',
	BCM47XX_UIN_UC_EXCL = '!',	
	BCM47XX_UIN_AT = '@',
	BCM47XX_UIN_UC_HASH = '#',
	BCM47XX_UIN_UC_DOLLAR = '$',
	BCM47XX_UIN_UC_PERCENT= '%',
	BCM47XX_UIN_UC_CARROT= '^',
	BCM47XX_UIN_UC_AMBERSEND = '&',
	BCM47XX_UIN_UC_ASTERIX = '*',

	BCM47XX_UIN_MENU_PRESS = 0xC5,
	BCM47XX_UIN_PLAY_PRESS = 0xC6,
	BCM47XX_UIN_PREV_PRESS = 0xC7,
	BCM47XX_UIN_NEXT_PRESS = 0xC8,

	BCM47XX_UIN_LEFT_PRESS = 0xC9,
	BCM47XX_UIN_RIGHT_PRESS = 0xCA,
	BCM47XX_UIN_UP_PRESS = 0xCB,
	BCM47XX_UIN_DOWN_PRESS = 0xCC,
	BCM47XX_UIN_MIDDLE_PRESS = 0xCD,

	BCM47XX_UIN_HOLD_PRESS = 0xCE,
	BCM47XX_UIN_RCRD_PRESS = 0xCF,

	BCM47XX_UIN_FWD = 0xD0,
	BCM47XX_UIN_BKWD = 0xD1,

	BCM47XX_UIN_MENU_RELEASE = 0xD2,
	BCM47XX_UIN_PLAY_RELEASE = 0xD3,
	BCM47XX_UIN_PREV_RELEASE = 0xD4,
	BCM47XX_UIN_NEXT_RELEASE = 0xD5,

	BCM47XX_UIN_LEFT_RELEASE = 0xD6,
	BCM47XX_UIN_RIGHT_RELEASE = 0xD7,
	BCM47XX_UIN_UP_RELEASE = 0xD8,
	BCM47XX_UIN_DOWN_RELEASE = 0xD9,
	BCM47XX_UIN_MIDDLE_RELEASE = 0xDA,

	BCM47XX_UIN_HOLD_RELEASE = 0xDB,
	BCM47XX_UIN_RCRD_RELEASE = 0xDC,

	BCM47XX_UIN_SHUTDOWN = 0xDD,
	BCM47XX_UIN_DISABLE = 0xDE,

	BCM47XX_BATTERY_OK = 0xE0,
	BCM47XX_BATTERY_FULL = 0xE1,
	BCM47XX_BATTERY_LOW = 0xE2,
	BCM47XX_BATTERY_CHARGER_INSERTED = 0xE3,
	BCM47XX_BATTERY_CHARGER_REMOVED = 0xE4,
	BCM47XX_BATTERY_PWR_ON_KEY_PRESSED = 0xE5,
	BCM47XX_BATTERY_PWR_ON_KEY_RELEASED = 0xE6,
	BCM47XX_BATTERY_PWR_ON_KEY_PRESSED_1S = 0xE7,
	BCM47XX_BATTERY_UNKNOWN = 0xE8
	

}BCM47XX_EVENT_MAPPING;

#endif /*__BCM47XX_UEVENT_MAPPING_H*/

