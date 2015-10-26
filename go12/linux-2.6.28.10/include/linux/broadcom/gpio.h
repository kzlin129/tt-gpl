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
*  gpio.h
*
*  PURPOSE:
*
*     This file defines the architecture-independent interface for
*     the GPIO driver.
*
*  NOTES:
*
*****************************************************************************/


#ifndef LINUX_GPIO_H
#define LINUX_GPIO_H


#if defined( __KERNEL__ )

   #if defined( CONFIG_MIPS ) && ( CONFIG_MIPS )
      #include <asm/broadcom/gpio.h>
      #if !defined( STANDALONE )
        // #including linux/irq.h or asm/irq.h gets NR_IRQs, but not the gpio_to_irq
        // So we add this #include to ensure that this definition shows up
        #include <asm/broadcom/irq.h>
      #endif
   #elif defined( CONFIG_ARM ) && ( CONFIG_ARM )
      #include <asm/arch/reg_gpio.h>
      #include <asm/gpio.h>
   #else
      #error "Unknown ARCH!"
   #endif

   #if defined( USE_BCM_GPIO )
       #define gpio_get_value(gpio)                 bcm_gpio_get(gpio)
       #define gpio_set_value(gpio,value)           bcm_gpio_set(gpio,value)
       #define gpio_direction_input(gpio)           bcm_gpio_direction_input(gpio)
       #define gpio_direction_output(gpio,value)    bcm_gpio_direction_output(gpio,value)

       static inline int  gpio_request( unsigned gpio, const char *label ) {(void)gpio; (void)label; return 0;}
       static inline void gpio_free( unsigned gpio ) {(void)gpio;}
   #else
       #if defined( CONFIG_HAVE_GPIO_LIB ) && !defined( STANDALONE )
    
          // This side of the if is chip specific, so the definitions should occur
          // in one of the header files included above.
    
       #else
          // Compatability code so that code which uses the gpiolib calls will still
          // work on platforms which don't yet support it.
    
          #define gpio_get_value(gpio)        gpio_get_pin_val(gpio)
          #define gpio_set_value(gpio,value)  gpio_set_pin_val(gpio,value)
          static inline int gpio_direction_input( unsigned gpio) { gpio_set_pin_type(gpio,GPIO_PIN_TYPE_INPUT); return 0; }
          static inline int gpio_direction_output( unsigned gpio, int value) 
            { gpio_set_pin_val(gpio,value); gpio_set_pin_type(gpio,GPIO_PIN_TYPE_OUTPUT); return 0; }
          #define gpio_direction_is_output(gpio) ( gpio_get_pin_type(gpio) == GPIO_PIN_TYPE_OUTPUT )
          static inline int  gpio_request( unsigned gpio, const char *label ) {(void)gpio; (void)label; return 0;}
          static inline void gpio_free( unsigned gpio ) {(void)gpio;}
       #endif
   #endif
#endif


#endif  /* LINUX_GPIO_H  */
