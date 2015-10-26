/* linux/arch/arm/mach-s5pc100/include/mach/system.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5PC100 - system implementation
 */

 #include <mach/idle.h>

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

void (*s5pc1xx_idle)(void);

void s5pc1xx_default_idle(void)
{
	printk("default idle function\n");
}

static void arch_idle(void)
{
	if(s5pc1xx_idle != NULL)
		(s5pc1xx_idle)();
	else
		s5pc1xx_default_idle();
}

static void arch_reset(char mode)
{
	/* nothing here yet */
}

#endif /* __ASM_ARCH_IRQ_H */
