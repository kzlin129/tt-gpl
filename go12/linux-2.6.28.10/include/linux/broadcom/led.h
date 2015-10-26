/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  led.h
*
*  PURPOSE:
*
*  This file defines the interface to the LED driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_LED_H )
#define LINUX_LED_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

typedef enum
{
	LED_MODE_OFF,
	LED_MODE_SOLID_ON,
	LED_MODE_FLASH_PATTERN
}
led_mode_t;

typedef struct
{
   unsigned int led;       // led number (numbering from 1)
   led_mode_t mode;	   // flash pattern
   unsigned int period;	   // overall repetition period in 1/10 secs units
   unsigned int ontime;    // on time in msec
   unsigned int offtime;   // off time in msec
   unsigned int pulses;    // pulses per period
}
led_ctrl_t;

/* ---- Constants and Types ---------------------------------------------- */

#define LED_POWER_LIGHT    1     // Power LED (usually RED)
#define LED_KEYPAD_LIGHT   2     // Keypad backlight (usually GREEN)


#define LED_MAGIC   'l'

#define LED_CMD_FIRST               0x80
#define LED_CMD_CTRL                0x80
#define LED_CMD_LAST                0x80

// time is in UTC time format
#define LED_IOCTL_CTRL  _IOWR( LED_MAGIC, LED_CMD_CTRL, led_ctrl_t )

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_LED_H */
