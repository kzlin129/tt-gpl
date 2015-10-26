/*****************************************************************************
* Copyright 2002 - 2009 Broadcom Corporation.  All rights reserved.
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
*  gpio.h - Broadcom global functions for GPIO access.
*
*  PURPOSE:
*
*     This file contains function prototypes
*     for configuring GPIOs (for input or output) and
*     setting/clearing output GPIOs.
*
*  NOTES:
*
*****************************************************************************/

#if !defined( ASM_ARCH_GPIO_H )
#define ASM_ARCH_GPIO_H

#if defined( CONFIG_HAVE_GPIO_LIB ) && !defined( STANDALONE ) && !defined( USE_BCM_GPIO )
    #define   ARCH_NR_GPIOS   (96 * 2)

    #include <asm-generic/gpio.h>
    #define gpio_get_value(gpio)        __gpio_get_value(gpio)
    #define gpio_set_value(gpio,value)  __gpio_set_value(gpio,value)
    #define gpio_cansleep(gpio)         __gpio_cansleep(gpio)
    #define gpio_to_irq(gpio)			__gpio_to_irq(gpio)
#else

    // The else side of this definition provides a gpiolib emulation for versions
    // of the kernel which haven't got gpiolib ported to it yet.
    // 
    // These definitions occur in include/linux/broadcom/gpio.h

#endif

#endif  // ASM_ARCH_GPIO_H
