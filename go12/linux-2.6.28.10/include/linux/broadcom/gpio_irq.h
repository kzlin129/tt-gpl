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
*  gpio_irq.h
*
*  PURPOSE:
*
*     This file defines the architecture-independent kernel API for the GPIO
*     IRQ driver. This allows clients to register ISR handlers that are
*     triggered by external interrupts on GPIO lines.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_GPIO_IRQ_H )
#define LINUX_GPIO_IRQ_H

#if defined( __KERNEL__ )

/* ---- Include Files ---------------------------------------------------- */

#include <linux/interrupt.h>
#include <linux/broadcom/gpio_types.h>


/* ---- Constants and Types ---------------------------------------------- */

typedef void *GPIO_IRQ_DATA;
typedef irqreturn_t (*GPIO_IRQ_HANDLER)( GPIO_IRQ_DATA );

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

int gpio_request_irq
(
   int                  pin,
   GPIO_INTERRUPT_TYPE  interruptType,
   GPIO_IRQ_HANDLER     irqHandler,
   void                *devId
);

int gpio_free_irq( int pin );
int gpio_enable_irq( int pin );
int gpio_disable_irq( int pin );

#endif   /* __KERNEL__ */
#endif  /* LINUX_GPIO_IRQ_H */
