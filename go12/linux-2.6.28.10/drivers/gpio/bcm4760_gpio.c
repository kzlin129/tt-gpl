/*****************************************************************************
* Copyright 2007 - 2009 Broadcom Corporation.  All rights reserved.
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
*  bcm4760_gpio.c
*
*  PURPOSE:
*
*     This implements the gpio driver.
*
*  NOTES:
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <asm/arch/hardware.h>
#include <asm/arch/reg_gpio.h>
#include <asm/arch/reg_irq.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/gpio_irq.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach/irq.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define  USE_NEW_IRQ 1

/* Wake up bit position for GPIO0 in PML_R_PML_WAKEUP_MASK0 */
#define PML_WAKEUP_BIT_GPIO_0	0

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
#define DBG_DEFAULT_LEVEL	DBG_TRACE2

#if DEBUG
#	define GPIO_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define GPIO_DEBUG(level,x)
#endif

HW_DECLARE_SPINLOCK(Gpio);

static char banner[] __initdata = KERN_INFO "BCM4760 GPIO Control Driver: 2.00 (built on "__DATE__" "__TIME__")\n";
static int gLevel = DBG_DEFAULT_LEVEL;

/* ---- Private Variables ------------------------------------------------ */

/*
 * Data structures for retaining GPIO pin programming during suspend
 *
 */

static uint32_t g_save_gpio0_gpor,
                g_save_gpio1_gpor[3];       /* output pin value save locations */

#ifdef CONFIG_PM
static uint32_t g_save_gpio0_imr,
                g_save_gpio1_imr[3];        /* interrupt mask register save locations */

static uint32_t g_save_gpio0_ctr[GPIO0_NUM],
                g_save_gpio1_ctr[GPIO1_NUM];/* control register register save locations */
#endif


/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  gpio_irq_ack
*
*     Called by the interrupt handler to acknowledge (i.e. clear)
*     the interrupt.
*
***************************************************************************/

static void gpio_irq_ack( unsigned irq )
{
   // Since this function is ONLY called with interrupts disabled, we don't
   // need to disable irqs around the following

   reg_gpio_clear_interrupt(IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_mask
*
*     Called to mask (i.e. disable) an interrupt.
*
***************************************************************************/

static void gpio_irq_mask( unsigned irq )
{
   // Since this function is ONLY called with interrupts disabled, we don't
   // need to disable irqs around the following

   reg_gpio_disable_interrupt(IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_unmask
*
*     Called to unmask (i.e. enable) an interrupt.
*
***************************************************************************/

static void gpio_irq_unmask( unsigned irq )
{
   // Since this function is ONLY called with interrupts disabled, we don't
   // need to disable irqs around the following

   reg_gpio_enable_interrupt(IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_type
*
*     Sets the type of the GPIO irq.
*
***************************************************************************/

static int gpio_irq_set_type( unsigned irq, unsigned type )
{
   int   gpio;

   gpio = IRQ_TO_GPIO( irq );

   // Since this function is ONLY called with interrupts disabled, we don't
   // need to disable irqs around the following

   GPIO_DEBUG(DBG_INFO,( KERN_INFO "IRQ%d (gpio%d): ", irq, gpio ));

   if ( type == IRQ_TYPE_PROBE )
   {
      // Don't mess GPIOs which already have interrupt handlers registered.

      if ( reg_gpio_is_interrupt_enable( gpio ))
      {
         GPIO_DEBUG(DBG_INFO,( "interrupts enabled already.\n" ));
         return 0;
      }

      if ( reg_gpio_itr_get_interrupt_type( gpio ) != GPIO_NO_INTERRUPT )
      {
         GPIO_DEBUG(DBG_INFO,( "interrupt type already set.\n" ));
         return 0;
      }

      if ( reg_gpio_iotr_get_pin_type( gpio ) == GPIO_PIN_TYPE_OUTPUT )
      {
         GPIO_DEBUG(DBG_INFO,( "pin programmed as output already.\n" ));
         return 0;
      }

//      type = IRQ_TYPE_EDGE_BOTH;
      type = IRQ_TYPE_NONE;
   }

   reg_gpio_disable_interrupt( gpio );
   reg_gpio_iotr_set_pin_type( gpio, GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT );
   reg_gpio_clear_interrupt( gpio );

   if ( type & IRQ_TYPE_EDGE_RISING )
   {
      if ( type & IRQ_TYPE_EDGE_FALLING )
      {
         GPIO_DEBUG(DBG_INFO,( "both edges\n" ));
         reg_gpio_itr_set_interrupt_type( gpio, GPIO_BOTH_EDGE_INTERRUPT_TRIGGER );
      }
      else
      {
         GPIO_DEBUG(DBG_INFO,( "rising edges\n" ));
         reg_gpio_itr_set_interrupt_type( gpio, GPIO_RISING_EDGE_INTERRUPT_TRIGGER );
      }
   }
   else if ( type & IRQ_TYPE_EDGE_FALLING )
   {
      GPIO_DEBUG(DBG_INFO,( "falling edges\n" ));
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_FALLING_EDGE_INTERRUPT_TRIGGER );
   }
   else if ( type & IRQ_TYPE_LEVEL_HIGH )
   {
      GPIO_DEBUG(DBG_INFO,( "high\n" ));
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER );
   }
   else if ( type & IRQ_TYPE_LEVEL_LOW )
   {
      GPIO_DEBUG(DBG_INFO,( "high\n" ));
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_LOW_LEVEL_INTERRUPT_TRIGGER );
   }
   else
   {
      GPIO_DEBUG(DBG_INFO,( "None.\n" ));
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_NO_INTERRUPT );
   }

   if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
      __set_irq_handler_unlocked(irq, handle_level_irq);
   else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
      __set_irq_handler_unlocked(irq, handle_edge_irq);

   return 0;

} // gpio_irq_set_type

/****************************************************************************
*
*  gpio_irq_set_wake
*
*     Enables or disables wake from suspend triggering for the specified GPIO.
*
***************************************************************************/

static int gpio_irq_set_wake( unsigned irq, unsigned on )
{
   int   gpio;

   gpio = IRQ_TO_GPIO( irq );

   if (gpio > 6)
      return -EINVAL;

   if ( on )
   {
      writel(readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK0_MEMADDR)) & ~(1 << (PML_WAKEUP_BIT_GPIO_0+gpio)),
             IO_ADDRESS(PML_R_PML_WAKEUP_MASK0_MEMADDR));
   }
   else
   {
      writel(readl(IO_ADDRESS(PML_R_PML_WAKEUP_MASK0_MEMADDR)) | (1 << (PML_WAKEUP_BIT_GPIO_0+gpio)),
             IO_ADDRESS(PML_R_PML_WAKEUP_MASK0_MEMADDR));
   }

   return 0;

} // gpio_irq_set_wake

/****************************************************************************
*
*  gpio_isr_handler
*
*     Figures out which GPIO caused the interrupt and calls the register
*     handler to deal with it.
*
*     The handler function will in all likelyhood be do_edge_IRQ.
*
***************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
void gpio_isr_handler( unsigned irq, struct irqdesc *desc, struct pt_regs *regs )
#else
void gpio_isr_handler( unsigned int irq, struct irq_desc *desc )
#endif
{
   int i;

    irq = GPIO_TO_IRQ( 0 );
    desc = irq_desc + irq;

    for( i=irq; i < (irq+NUM_GPIO_IRQS) ; i++,desc++ )
    {
        if( reg_gpio_get_interrupt_status( IRQ_TO_GPIO(i) ) && reg_gpio_is_interrupt_enable( IRQ_TO_GPIO(i) ))
        {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
            desc->handle( i, desc, regs );
#else
            desc->handle_irq( i, desc );
#endif
            continue;
        }
    }

} // gpio_isr_handler

#ifdef CONFIG_HAVE_GPIO_LIB
/****************************************************************************
*
*  Configure a GPIO pin as an input pin
*
*****************************************************************************/

static int gpio_476x_direction_input( struct gpio_chip *chip, unsigned offset )
{
    (void)chip;

    if(offset > GPIOPIN_MAX)
    {
        return -EINVAL;
    }
    else
    {
        GPIO_DEBUG(DBG_TRACE,("GPIO: Set input direction %d\n",offset));
        reg_gpio_iotr_set_pin_type( offset, GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT);
    }
    return 0;

} // gpio_476x_direction_input

/****************************************************************************
*
*  Configure a GPIO pin as an output pin and sets its initial value.
*
*****************************************************************************/

static int gpio_476x_direction_output( struct gpio_chip *chip, unsigned offset, int value )
{
    (void)chip;

    if(offset > GPIOPIN_MAX)
    {
        return -EINVAL;
    }
    else
    {
        if(value != 0)
            value=1;

        GPIO_DEBUG(DBG_TRACE,("GPIO: Set output direction %d,%d\n",offset,value));
        reg_gpio_iotr_set_pin_type( offset, GPIO_PIN_TYPE_OUTPUT );
        reg_gpio_set_pin( offset, value );
        
        /*
         * Save the values we set for output so we can
         * correctly reprogram them if we enter suspend and
         * then resume. Core power is lost so these register
         * values are not retained.
         * 
         * Note: The first 24 GPIOs have extra state retention logic
         *       maintaining the pin state during suspend. This is
         *       continued until the state retention is disabled. That
         *       is done during the pwr_seq.c final suspend routine
         *       processing. By then we will have restored the register
         *       values in the GPIO resume function so all should be
         *       well for those 24 GPIOs. Of course the remaining GPIO
         *       pins have no state retention but we'll return them
         *       to the correct state but during suspend and until we
         *       execute the GPIO resume function they will not be
         *       in the state they were prior to suspend.
         */
        if(offset < GPIO_BANK_0_NUM)
        {
            g_save_gpio0_gpor &= ~(1<<offset);
            g_save_gpio0_gpor |= (value<<offset);
        }
        else
        {
            g_save_gpio1_gpor[offset>>5] &= ~(1<<((offset-7)&31));
            g_save_gpio1_gpor[offset>>5] |= (value<<((offset-7)&31));
        }
    }
    return 0;

} // gpio_476x_direction_output

/****************************************************************************
*
*  Retrieve the value of a GPIO pin. Note that this returns zero or the raw
*   value.
*
*****************************************************************************/

static int gpio_476x_get( struct gpio_chip *chip, unsigned offset )
{
    (void)chip;

    if(offset > GPIOPIN_MAX)
    {
        return 0;
    }
    else
    {
        GPIO_DEBUG(DBG_TRACE,("GPIO: Get value %d\n",offset));
        return reg_gpio_get_pin( offset );
    }

} // gpio_476x_get

/****************************************************************************
*
*  Set the value of a GPIO pin
*
*****************************************************************************/

static void gpio_476x_set( struct gpio_chip *chip, unsigned offset, int value )
{
    (void)chip;

	GPIO_DEBUG(DBG_TRACE,("GPIO: Set value %d,%d\n",offset,value));
    if(offset > GPIOPIN_MAX)
    {
        return;
    }
    else
    {
        if(value != 0)
            value=1;

        GPIO_DEBUG(DBG_TRACE,("GPIO: Set output direction %d,%d\n",offset,value));
        reg_gpio_set_pin( offset, value );
        
        /*
         * Save the values we set for output so we can
         * correctly reprogram them if we enter suspend and
         * then resume. Core power is lost so these register
         * values are not retained.
         * 
         * Note: The first 24 GPIOs have extra state retention logic
         *       maintaining the pin state during suspend. This is
         *       continued until the state retention is disabled. That
         *       is done during the pwr_seq.c final suspend routine
         *       processing. By then we will have restored the register
         *       values in the GPIO resume function so all should be
         *       well for those 24 GPIOs. Of course the remaining GPIO
         *       pins have no state retention but we'll return them
         *       to the correct state but during suspend and until we
         *       execute the GPIO resume function they will not be
         *       in the state they were prior to suspend.
         */
        if(offset < GPIO_BANK_0_NUM)
        {
            g_save_gpio0_gpor &= ~(1<<offset);
            g_save_gpio0_gpor |= (value<<offset);
        }
        else
        {
            g_save_gpio1_gpor[offset>>5] &= ~(1<<((offset-7)&31));
            g_save_gpio1_gpor[offset>>5] |= (value<<((offset-7)&31));
        }
    }

} // gpio_476x_set

static int gpio_476x_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return GPIO_TO_IRQ(offset + chip->base);
}

/****************************************************************************
*
*  gpiolib chip description
*
*****************************************************************************/

// Note: If you need to add a field, like a register base, then create a new
//       structure which includes the gpio_chip as the first element and has
//       the custom fields after. See asm-arm/arch-pxa/gpio.c for an example.

static struct gpio_chip gpio_476x_chip =
{
    .label              = "476x",
    .direction_input    = gpio_476x_direction_input,
    .direction_output   = gpio_476x_direction_output,
    .get                = gpio_476x_get,
    .set                = gpio_476x_set,
    .to_irq             = gpio_476x_to_irq,
    .base               = 0,
    .ngpio              = NUM_GPIO_IRQS,
};

/****************************************************************************
*
*  brcm_init_gpio
*
*   Sets up gpiolib so that it's aware of how to manipulate our GPIOs
*
*****************************************************************************/

void  bcm_init_gpio( void )
{
    gpiochip_add( &gpio_476x_chip );

} // bcm_init_gpio

#endif //#ifdef CONFIG_HAVE_GPIO_LIB

/****************************************************************************
*
*  gpio_chip data structure.
*
***************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static struct irqchip gpio_chip =
{
   .ack     = gpio_irq_ack,
   .mask    = gpio_irq_mask,
   .unmask  = gpio_irq_unmask,
   .set_type    = gpio_irq_set_type,
};
#else
static struct irq_chip gpio_chip =
{
   .ack        = gpio_irq_ack,
   .mask       = gpio_irq_mask,
   .unmask     = gpio_irq_unmask,
   .set_type   = gpio_irq_set_type,
   .set_wake   = gpio_irq_set_wake,
};
#endif

/****************************************************************************
*
*  bcm4760_gpio_probe
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

//static int bcm4760_gpio_probe( void )
static int bcm4760_gpio_probe(struct platform_device *pdev)
{
   int   irq;

   GPIO_DEBUG(DBG_INFO,( "bcm4760_gpio_probe called\n" ));

   /* Start out with interrupts for all GPIOs disabled. */

   __REG32(IO_ADDRESS(GIO0_R_IMR0_MEMADDR)) = 0x0000007F;
   __REG32(IO_ADDRESS(GIO1_R_IMR0_MEMADDR)) = 0xFFFFFFFF;
   __REG32(IO_ADDRESS(GIO1_R_IMR1_MEMADDR)) = 0xFFFFFFFF;
   __REG32(IO_ADDRESS(GIO1_R_IMR2_MEMADDR)) = 0xFFFFFFFF;

#ifdef CONFIG_HAVE_GPIO_LIB 
   bcm_init_gpio( );
#endif
   GPIO_DEBUG(DBG_INFO,( banner ));

   for ( irq = GPIO_TO_IRQ( GPIO_BANK_0_START ); irq <= GPIO_TO_IRQ( GPIO_BANK_0_NUM -1 ); irq++ )
   {
      set_irq_chip( irq, &gpio_chip );
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
      set_irq_handler( irq, do_edge_IRQ );
#else
      set_irq_handler( irq, handle_edge_irq );
#endif
      set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
   }
   set_irq_chained_handler( IRQ_GPIO_0, gpio_isr_handler );

   for ( irq = GPIO_TO_IRQ( GPIO_BANK_1_START ); irq <= GPIO_TO_IRQ( (GPIO_BANK_1_START + GPIO_BANK_1_NUM) - 1 ); irq++ )
   {
      set_irq_chip( irq, &gpio_chip );
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
      set_irq_handler( irq, do_edge_IRQ );
#else
      set_irq_handler( irq, handle_edge_irq );
#endif
      set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
   }
   set_irq_chained_handler( IRQ_GPIO_1, gpio_isr_handler );

   for ( irq = GPIO_TO_IRQ( GPIO_BANK_2_START ); irq <= GPIO_TO_IRQ( (GPIO_BANK_2_START + GPIO_BANK_2_NUM) - 1 ); irq++ )
   {
      set_irq_chip( irq, &gpio_chip );
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
      set_irq_handler( irq, do_edge_IRQ );
#else
      set_irq_handler( irq, handle_edge_irq );
#endif
      set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
   }
   set_irq_chained_handler( IRQ_GPIO_2, gpio_isr_handler );

   for ( irq = GPIO_TO_IRQ( GPIO_BANK_3_START ); irq <= GPIO_TO_IRQ( (GPIO_BANK_3_START + GPIO_BANK_3_NUM) - 1 ); irq++ )
   {
      set_irq_chip( irq, &gpio_chip );
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
      set_irq_handler( irq, do_edge_IRQ );
#else
      set_irq_handler( irq, handle_edge_irq );
#endif
      set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
   }
   set_irq_chained_handler( IRQ_GPIO_3, gpio_isr_handler );

   return 0;
} /* bcm4760_gpio_probe */

/****************************************************************************
*
*  gpio_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
//static void bcm4760_gpio_remove( void )
static int bcm4760_gpio_remove(struct platform_device *pdev)
{
	GPIO_DEBUG(DBG_TRACE,("COMCTL - gpio_cleanup()\n"));

	/* unregister sysctl table */
	return 0;
} /* bcm4760_gpio_remove */

#ifdef CONFIG_PM
static int bcm4760_gpio_suspend(struct device *dev)
{
    int i;
    struct platform_device *pdev;

    pdev = to_platform_device(dev);
    GPIO_DEBUG(DBG_TRACE2,( "Suspend processed for %s.\n", pdev->name ));

//    i2c = platform_get_drvdata(pdev);

    /*
     * Preserve Interrupt Mask Register values.
     */

    g_save_gpio0_imr = readl(IO_ADDRESS(GIO0_R_IMR0_MEMADDR));
    g_save_gpio1_imr[0] = readl(IO_ADDRESS(GIO1_R_IMR0_MEMADDR));
    g_save_gpio1_imr[1] = readl(IO_ADDRESS(GIO1_R_IMR1_MEMADDR));
    g_save_gpio1_imr[2] = readl(IO_ADDRESS(GIO1_R_IMR2_MEMADDR));
    
    /*
     * Preserve GPCTR Register values.
     */

    for( i=0 ; i<GPIO0_NUM ; i++)
    {
        g_save_gpio0_ctr[i] = readl(IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4);

	/* See if we need to do the edge triggered wake-up workaround. */
	if( !(g_save_gpio0_ctr[i] & 0x8000) && (g_save_gpio0_ctr[i] & 0x0001) && !(readl( IO_ADDRESS( PML_R_PML_WAKEUP_MASK0_MEMADDR ) ) & (1 << i)) )
	{
	    switch( g_save_gpio0_ctr[i] & 0x0018 )
	    {
		case 0x0008 :
		    /* Rising edge. */
		    if( !gpio_get_value( i ) )
			writel( g_save_gpio0_ctr[i] & ~(1 << 14), IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4 );
		    break;
		case 0x0010 :
		    /* Falling edge. */
		    if( gpio_get_value( i ) )
			writel( g_save_gpio0_ctr[i] | (1 << 14), IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4 );
		    break;
		case 0x0018 :
		    /* Both edges. */
		    if( gpio_get_value( i ) )
			writel( g_save_gpio0_ctr[i] | (1 << 14), IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4 );
		    else
			writel( g_save_gpio0_ctr[i] & ~(1 << 14), IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4 );
		    break;
		default :
		    /* No interrupt. Do nothing. */
		    break;
	    }
	}
    }
    for( i=0 ; i<GPIO1_NUM ; i++)
    {
        g_save_gpio1_ctr[i] = readl(IO_ADDRESS(GIO1_R_GPCTR0_MEMADDR) + i*4);
    }

    return 0;
} /* bcm4760_gpio_suspend */

static int bcm4760_gpio_resume(struct device *dev)
{
    int i;
    struct platform_device *pdev;

    pdev = to_platform_device(dev);
    GPIO_DEBUG(DBG_TRACE2,( "Resume processed for %s.\n", pdev->name ));

    /*
     * Restore Interrupt Mask Register values.
     */

    writel(g_save_gpio0_imr, IO_ADDRESS(GIO0_R_IMR0_MEMADDR));
    writel(g_save_gpio1_imr[0], IO_ADDRESS(GIO1_R_IMR0_MEMADDR));
    writel(g_save_gpio1_imr[1], IO_ADDRESS(GIO1_R_IMR1_MEMADDR));
    writel(g_save_gpio1_imr[2], IO_ADDRESS(GIO1_R_IMR2_MEMADDR));
    
    /*
     * Restore GPIO output Register values.
     */

    writel(g_save_gpio0_gpor, IO_ADDRESS(GIO0_R_GPORS0_MEMADDR));
    writel((~g_save_gpio0_gpor) & GIO0_R_GPORC0_MASK,
           IO_ADDRESS(GIO0_R_GPORC0_MEMADDR));
    writel(g_save_gpio1_gpor[0], IO_ADDRESS(GIO1_R_GPORS0_MEMADDR));
    writel((~g_save_gpio1_gpor[0]) & GIO1_R_GPORC0_MASK,
           IO_ADDRESS(GIO1_R_GPORC0_MEMADDR));
    writel(g_save_gpio1_gpor[1], IO_ADDRESS(GIO1_R_GPORS1_MEMADDR));
    writel((~g_save_gpio1_gpor[1]) & GIO1_R_GPORC1_MASK,
           IO_ADDRESS(GIO1_R_GPORC1_MEMADDR));
    writel(g_save_gpio1_gpor[2], IO_ADDRESS(GIO1_R_GPORS2_MEMADDR));
    writel((~g_save_gpio1_gpor[2]) & GIO1_R_GPORC2_MASK,
           IO_ADDRESS(GIO1_R_GPORC2_MEMADDR));
    
    /*
     * Preserve GPCTR Register values.
     */

    for( i=0 ; i<GPIO0_NUM ; i++)
    {
        writel(g_save_gpio0_ctr[i], IO_ADDRESS(GIO0_R_GPCTR0_MEMADDR) + i*4);
    }
    for( i=0 ; i<GPIO1_NUM ; i++)
    {
        writel(g_save_gpio1_ctr[i],
               IO_ADDRESS(GIO1_R_GPCTR0_MEMADDR) + i*4);
    }

    return 0;
} /* bcm4760_gpio_resume */

#else
#define bcm4760_gpio_suspend NULL
#define bcm4760_gpio_resume NULL
#endif

static struct pm_ext_ops bcm4760_gpio_pm_ops =
{
    .base =
    {
        .suspend    = bcm4760_gpio_suspend,
        .resume     = bcm4760_gpio_resume
    }
};

/* device driver for platform bus bits */

static struct platform_driver bcm4760_gpio_driver = {
    .probe          = bcm4760_gpio_probe,
    .remove         = bcm4760_gpio_remove,
//    .suspend_late   = bcm4760_i2c_suspend_late,
//    .resume         = bcm4760_i2c_resume,
    .pm             = &bcm4760_gpio_pm_ops,
    .driver       =
    {
        .owner  = THIS_MODULE,
        .name   = "bcm4760-gpio"
    }
};


/****************************************************************************
*
*  bcm4760_gpio_init
*
*       Module initialization function registers GPIO platform driver.
*
***************************************************************************/
static int __init bcm4760_gpio_init(void)
{
    return platform_driver_register(&bcm4760_gpio_driver);
} /* bcm4760_gpio_init */

/****************************************************************************
*
*  bcm4760_gpio_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit bcm4760_gpio_exit( void )
{
	GPIO_DEBUG(DBG_INFO,( "gpio_exit called\n" ));

    platform_driver_unregister(&bcm4760_gpio_driver);
} /* bcm4760_gpio_exit */


core_initcall(bcm4760_gpio_init);
module_exit(bcm4760_gpio_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM4760 GPIO Control Driver");
