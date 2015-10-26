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
*  vc_gpio.h
*
*  PURPOSE:
*
*     This file defines the function pototype for the GPIO driver of video core
*
*  NOTES:
*
*****************************************************************************/


#ifndef VC_GPIO_H
#define VC_GPIO_H

#define GPIO_VC02_MASK      (0x80)
#define GPIO_VC02_OFFSET    (GPIO_VC02_MASK)
#define VC_GPIO_NUM_GPIO    (52)

#define IS_VC_GPIO(pin)  ((pin & GPIO_VC02_MASK) > 0)

int  vc_gpio_get_pin_type(int pin);
void vc_gpio_set_pin_type(int pin, int val);
int  vc_gpio_get_pin_val(int pin);
void vc_gpio_set_pin_val(int pin, int val);

#if defined( CONFIG_HAVE_GPIO_LIB )
void vc_init_gpio( void );
#else
#define vc_init_gpio()
#endif

#endif  /* LINUX_GPIO_H  */
