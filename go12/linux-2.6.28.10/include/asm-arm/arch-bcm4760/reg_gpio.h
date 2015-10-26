/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
*  reg_gpio.h
*
*  PURPOSE:
*
*     This file contains definitions for the GPIO registers.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_GPIO_H )
#define __ASM_ARCH_REG_GPIO_H

/* ---- Include Files ---------------------------------------------------- */

#include <asm/types.h>
#include <asm/arch/hardware.h>
#include <linux/broadcom/gpio_types.h>
#include <linux/spinlock.h>
#if defined(CONFIG_BCM_VC02)
#include <linux/broadcom/vc_gpio.h>
#endif 

HW_EXTERN_SPINLOCK(Gpio)

#define	HW_GPIO0_PHY_BASE	GIO0_REG_BASE_ADDR
#define	HW_GPIO0_BASE	IO_ADDRESS(HW_GPIO0_PHY_BASE)
#define HW_GPIO1_PHY_BASE   GIO1_REG_BASE_ADDR
#define	HW_GPIO1_BASE	IO_ADDRESS(HW_GPIO1_PHY_BASE)

/* ---- Constants and Types ---------------------------------------------- */


/* --------------------------------------------------------------------------
** This defines the public GPIO API defined in /include/linux/broadcom/gpio.h.
*/

typedef enum
{
    GPIO_PIN_TYPE_INPUT                   = 0x00,
    GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT    = 0x01,
    GPIO_PIN_TYPE_OUTPUT                  = 0x02,
#if defined(CONFIG_BCM_VC02)
    /* following API is only used for video core gpio pin function */
    GPIO_PIN_TYPE_ALTERNATIVE_FUNC0       = 0x03,
    GPIO_PIN_TYPE_ALTERNATIVE_FUNC1       = 0x04,
#endif
} GPIO_PIN_TYPE;


#define gpio_get_pin_type  reg_gpio_iotr_get_pin_type
#define gpio_set_pin_type  reg_gpio_iotr_set_pin_type
#define gpio_set_pin_val   reg_gpio_set_pin
#define gpio_get_pin_val   reg_gpio_get_pin

/*
** End of public API
--------------------------------------------------------------------------*/

#define  GPIO_BANK_0_START          0
#define  GPIO_BANK_0_NUM            7
#define  GPIO_BANK_1_START          (GPIO_BANK_0_START + GPIO_BANK_0_NUM)
#define  GPIO_BANK_1_NUM            32
#define  GPIO_BANK_2_START          (GPIO_BANK_1_START + GPIO_BANK_1_NUM)
#define  GPIO_BANK_2_NUM            32
#define  GPIO_BANK_3_START          (GPIO_BANK_2_START + GPIO_BANK_2_NUM)
#define  GPIO_BANK_3_NUM            25

#define GPIO0_NUM                   (GPIO_BANK_0_NUM)
#define GPIO1_NUM                   (GPIO_BANK_1_NUM + GPIO_BANK_2_NUM + GPIO_BANK_3_NUM)

#define GIO1_R_GPCTR0_MEMADDR       GIO1_R_GPCTR7_MEMADDR

#define REG_GPIO_GPCTR(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_GPCTR0_MEMADDR + ((PIN) << 2)))

#define REG_GPIO_GPITR_NO_INTERRUPT                   GPIO_NO_INTERRUPT
#define REG_GPIO_GPITR_RISING_EDGE_INTERRUPT_TRIGGER  GPIO_RISING_EDGE_INTERRUPT_TRIGGER
#define REG_GPIO_GPITR_FALLING_EDGE_INTERRUPT_TRIGGER GPIO_FALLING_EDGE_INTERRUPT_TRIGGER
#define REG_GPIO_GPITR_BOTH_EDGE_INTERRUPT_TRIGGER    GPIO_BOTH_EDGE_INTERRUPT_TRIGGER
#define REG_GPIO_LOW_LEVEL_INTERRUPT_TRIGGER          GPIO_LOW_LEVEL_INTERRUPT_TRIGGER
#define REG_GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER         GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER

static inline int reg_gpio_iotr_get_pin_type( int pin )
{
 
#if defined(CONFIG_BCM_VC02)
   if(IS_VC_GPIO(pin))
   {
      return vc_gpio_get_pin_type(pin);
   }
#endif

      if(pin < GPIO_BANK_0_NUM)
      {
         if(REG_GPIO_GPCTR(0,pin) & GIO0_F_IOTR_MASK)
            if(REG_GPIO_GPCTR(0,pin) & GIO0_F_ITR_MASK)
               return GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT;
            else
               return GPIO_PIN_TYPE_INPUT;
         else
            return GPIO_PIN_TYPE_OUTPUT;
      }
      else
      {
         if(REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_IOTR_MASK)
            if(REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_ITR_MASK)
               return GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT;
            else
               return GPIO_PIN_TYPE_INPUT;
         else
            return GPIO_PIN_TYPE_OUTPUT;
      }
}

static inline void reg_gpio_iotr_set_pin_type( int pin, GPIO_PIN_TYPE pinType )
{
#if defined(CONFIG_BCM_VC02)
   if(IS_VC_GPIO(pin))
      {
      vc_gpio_set_pin_type(pin, pinType);
   }
   else
#endif
   {
      unsigned long flags,maskin;

      HW_IRQ_SAVE(Gpio, flags);
      if(pinType == GPIO_PIN_TYPE_OUTPUT)
         maskin = 0;
      else
         maskin = GIO0_F_IOTR_MASK;

      if(pin < GPIO_BANK_0_NUM)
      {
         REG_GPIO_GPCTR(0,pin) &= ~(GIO0_F_IOTR_MASK | GIO0_F_ITR_MASK);
         REG_GPIO_GPCTR(0,pin) |= maskin;
      }
      else
      {
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) &= ~(GIO0_F_IOTR_MASK | GIO0_F_ITR_MASK);
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) |= maskin;
      }
      HW_IRQ_RESTORE(Gpio, flags);
   }
}

#define REG_GPIO_GPORS(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_GPORS0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPORC(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_GPORC0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPORG(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_GPOR0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPOR_SHIFT(PIN)    ((PIN) & 0x1F)
#define REG_GPIO_GPOR_MASK(PIN)     (1 << REG_GPIO_GPOR_SHIFT(PIN))

static inline void reg_gpio_set_pin( int pin, int val )
{
#if defined(CONFIG_BCM_VC02)
   if(IS_VC_GPIO(pin))
   {
      return vc_gpio_set_pin_val(pin, val);
   }
   else
#endif
   {
      unsigned long flags;

      HW_IRQ_SAVE( Gpio, flags );
      if(pin < GPIO_BANK_0_NUM)
      {
         if ( val == 0 )
         {
            // Set the pin to zero
            REG_GPIO_GPORC(0,pin) = REG_GPIO_GPOR_MASK(pin);
         }
         else
         {
            // Set the pin to 1
            REG_GPIO_GPORS(0,pin) = REG_GPIO_GPOR_MASK(pin);
         }
      }
      else
      {
         if ( val == 0 )
         {
            // Set the pin to zero
            REG_GPIO_GPORC(1,pin-GPIO_BANK_0_NUM) = REG_GPIO_GPOR_MASK(pin-GPIO_BANK_0_NUM);
         }
         else
         {
            // Set the pin to 1
            REG_GPIO_GPORS(1,pin-GPIO_BANK_0_NUM) = REG_GPIO_GPOR_MASK(pin-GPIO_BANK_0_NUM);
         }
      }
      
      HW_IRQ_RESTORE( Gpio, flags );
   }
}

static inline int reg_gpio_get_pin_output( int pin )
{
      if(pin < GPIO_BANK_0_NUM)
         return ( REG_GPIO_GPORG(0,pin) & REG_GPIO_GPOR_MASK(pin) ) != 0;
      else
         return ( REG_GPIO_GPORG(1,pin-GPIO_BANK_0_NUM) & REG_GPIO_GPOR_MASK(pin-GPIO_BANK_0_NUM) ) != 0;
}

#define REG_GPIO_GPIPS(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_GPIR0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPIPS_SHIFT(PIN)   ((PIN) & 0x1F)
#define REG_GPIO_GPIPS_MASK(PIN)    (1 << REG_GPIO_GPIPS_SHIFT(PIN))

static inline int reg_gpio_get_pin( int pin )
{
#if defined(CONFIG_BCM_VC02)
   if(IS_VC_GPIO(pin))
   {
      return vc_gpio_get_pin_val(pin);
   }
#endif

   if(pin < GPIO_BANK_0_NUM)
      return ( REG_GPIO_GPIPS(0,pin) & REG_GPIO_GPIPS_MASK(pin) ) != 0;
   else
      return ( REG_GPIO_GPIPS(1,pin-GPIO_BANK_0_NUM) & REG_GPIO_GPIPS_MASK(pin-GPIO_BANK_0_NUM) ) != 0;

}

#define REG_GPIO_GPIMRS(IDX,PIN)    __REG32(IO_ADDRESS(GIO##IDX##_R_IMR0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPIMRC(IDX,PIN)    __REG32(IO_ADDRESS(GIO##IDX##_R_IMRC0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPIMR_SHIFT(PIN)   ((PIN) & 0x1F)
#define REG_GPIO_GPIMR_MASK(PIN)    (1 << REG_GPIO_GPIMR_SHIFT(PIN))

static inline int reg_gpio_is_interrupt_enable( int pin )
{
      if(pin < GPIO_BANK_0_NUM)
         return ( REG_GPIO_GPIMRS(0,pin) & REG_GPIO_GPIMR_MASK(pin) ) == 0;
      else
         return ( REG_GPIO_GPIMRS(1,pin-GPIO_BANK_0_NUM) & REG_GPIO_GPIMR_MASK(pin-GPIO_BANK_0_NUM) ) == 0;
}

static inline void reg_gpio_enable_interrupt( int pin )
{
   unsigned long flags;

   HW_IRQ_SAVE( Gpio, flags );
   if(pin < GPIO_BANK_0_NUM)
      REG_GPIO_GPIMRC(0,pin) = REG_GPIO_GPIMR_MASK(pin);
   else
      REG_GPIO_GPIMRC(1,pin-GPIO_BANK_0_NUM) = REG_GPIO_GPIMR_MASK(pin-GPIO_BANK_0_NUM);
   HW_IRQ_RESTORE( Gpio, flags );
}

static inline void reg_gpio_disable_interrupt( int pin )
{
   unsigned long flags;

   HW_IRQ_SAVE( Gpio, flags );
   if(pin < GPIO_BANK_0_NUM)
      REG_GPIO_GPIMRS(0,pin) = REG_GPIO_GPIMR_MASK(pin);
   else
      REG_GPIO_GPIMRS(1,pin-GPIO_BANK_0_NUM) = REG_GPIO_GPIMR_MASK(pin-GPIO_BANK_0_NUM);
   HW_IRQ_RESTORE( Gpio, flags );
}

#define REG_GPIO_GPISR(IDX,PIN)     __REG32(IO_ADDRESS(GIO##IDX##_R_ISR0_MEMADDR + (((PIN) & 0xE0) >> 3)))
#define REG_GPIO_GPISR_SHIFT(PIN)    ((PIN) & 0x1F)
#define REG_GPIO_GPISR_MASK(PIN)     (1 << REG_GPIO_GPISR_SHIFT(PIN))

static inline void reg_gpio_clear_interrupt( int pin )
{
   if(pin < GPIO_BANK_0_NUM)
      REG_GPIO_GPISR(0,pin) = REG_GPIO_GPISR_MASK(pin);
   else
      REG_GPIO_GPISR(1,pin-GPIO_BANK_0_NUM) = REG_GPIO_GPISR_MASK(pin-GPIO_BANK_0_NUM);
}

static inline int reg_gpio_get_interrupt_status( int pin )
{
      if(pin < GPIO_BANK_0_NUM)
         return ( REG_GPIO_GPISR(0,pin) & REG_GPIO_GPISR_MASK(pin) ) != 0;
      else
         return ( REG_GPIO_GPISR(1,pin-GPIO_BANK_0_NUM) & REG_GPIO_GPISR_MASK(pin-GPIO_BANK_0_NUM) ) != 0;
}

static inline int reg_gpio_itr_get_interrupt_type( int pin )
{
   if(pin < GPIO_BANK_0_NUM)
   {
      if(REG_GPIO_GPCTR(0,pin) & GIO0_F_LVLINT_MASK)
         return (((REG_GPIO_GPCTR(0,pin) & GIO0_F_LVLINT_MASK) >> (GIO0_F_LVLINT_R-2)) |
                 ((((REG_GPIO_GPCTR(0,pin) & GIO0_F_PMLINV_MASK) ^ GIO0_F_PMLINV_MASK) >> GIO0_F_PMLINV_R)));
      else
         return (( REG_GPIO_GPCTR(0,pin) & GIO0_F_ITR_MASK) >> GIO0_F_ITR_R);
   }
   else
   {
      if(REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_LVLINT_MASK)
         return (((REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_LVLINT_MASK) >> (GIO0_F_LVLINT_R-2)) |
                 ((((REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_PMLINV_MASK) ^ GIO0_F_PMLINV_MASK) >> GIO0_F_PMLINV_R)));
      else
         return (( REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) & GIO0_F_ITR_MASK) >> GIO0_F_ITR_R);
   }
}

static inline void reg_gpio_itr_set_interrupt_type( int pin, GPIO_INTERRUPT_TYPE interruptType )
{
   unsigned long flags;

   HW_IRQ_SAVE( Gpio, flags );
   if(pin < GPIO_BANK_0_NUM)
   {
      REG_GPIO_GPCTR(0,pin) &= ~(GIO0_F_ITR_MASK | GIO0_F_LVLINT_MASK | GIO0_F_PMLINV_MASK);
      if(interruptType < GPIO_LOW_LEVEL_INTERRUPT_TRIGGER)
         REG_GPIO_GPCTR(0,pin) |= (interruptType << GIO0_F_ITR_R);
      else
         REG_GPIO_GPCTR(0,pin) |= (GIO0_F_LVLINT_MASK |
                        (((interruptType-GPIO_LOW_LEVEL_INTERRUPT_TRIGGER) << GIO0_F_PMLINV_R) ^ GIO0_F_PMLINV_MASK));
   }
   else
   {
      REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) &= ~(GIO0_F_ITR_MASK | GIO0_F_LVLINT_MASK | GIO0_F_PMLINV_MASK);
      if(interruptType < GPIO_LOW_LEVEL_INTERRUPT_TRIGGER)
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) |= (interruptType << GIO0_F_ITR_R);
      else
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) |= (GIO0_F_LVLINT_MASK |
                        (((interruptType-GPIO_LOW_LEVEL_INTERRUPT_TRIGGER) << GIO0_F_PMLINV_R) ^ GIO0_F_PMLINV_MASK));
   }
   HW_IRQ_RESTORE( Gpio, flags );
}

static inline void reg_gpio_set_pull_up_down( int pin, int val )
{
   if(val)
   {
      if(pin < GPIO_BANK_0_NUM)
      {
         REG_GPIO_GPCTR(0,pin) |= GIO0_F_PUD_MASK;
      }
      else
      {
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) |= GIO0_F_PUD_MASK;
      }
   }
   else
   {
      if(pin < GPIO_BANK_0_NUM)
      {
         REG_GPIO_GPCTR(0,pin) &= ~GIO0_F_PUD_MASK;
      }
      else
      {
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) &= ~GIO0_F_PUD_MASK;
      }
   }
}

static inline void reg_gpio_set_pull_up_down_enable( int pin )
{
      if(pin < GPIO_BANK_0_NUM)
      {
         REG_GPIO_GPCTR(0,pin) |= GIO0_F_PEN_MASK;
      }
      else
      {
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) |= GIO0_F_PEN_MASK;
      }
}

static inline void reg_gpio_set_pull_up_down_disable( int pin )
{
      if(pin < GPIO_BANK_0_NUM)
      {
         REG_GPIO_GPCTR(0,pin) &= ~GIO0_F_PEN_MASK;
      }
      else
      {
         REG_GPIO_GPCTR(1,pin-GPIO_BANK_0_NUM) &= ~GIO0_F_PEN_MASK;
      }
}

/*
 *   gpio pinshare
 */
typedef enum
  {

    PIN_SHARE_GPIO_ALT0 = 0,
    PIN_SHARE_GPIO_ALT1 = 1,
    PIN_SHARE_GPIO_ALT2 = 2,
    PIN_SHARE_GPIO_ALT3 = 3,
    PIN_SHARE_GPIO_MAX,

  } PIN_SHARE_GPIO_FUNCTION;

#define GPIOPIN_MAX 95

#endif  /* __ASM_ARCH_REG_GPIO_H */
