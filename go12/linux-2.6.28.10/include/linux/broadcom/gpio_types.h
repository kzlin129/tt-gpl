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
*  gpio_types.h
*
*  PURPOSE:
*
*     This file defines generic types used by the GPIO and GPIO IRQ driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_GPIO_TYPES_H )
#define LINUX_GPIO_TYPES_H

/* ---- Include Files ---------------------------------------------------- */
/* ---- Constants and Types ---------------------------------------------- */


typedef enum
{
    GPIO_NO_INTERRUPT                   = 0x00,
    GPIO_RISING_EDGE_INTERRUPT_TRIGGER  = 0x01,
    GPIO_FALLING_EDGE_INTERRUPT_TRIGGER = 0x02,
    GPIO_BOTH_EDGE_INTERRUPT_TRIGGER    = 0x03,
    GPIO_LOW_LEVEL_INTERRUPT_TRIGGER    = 0x04,
    GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER   = 0x05,
    GPIO_MAX_INTERRUPT_TYPES

} GPIO_INTERRUPT_TYPE;

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_GPIO_TYPES_H */
