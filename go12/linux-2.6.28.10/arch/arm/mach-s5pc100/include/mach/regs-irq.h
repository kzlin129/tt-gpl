/* linux/arch/arm/mach-s5pc100/include/mach/regs-irq.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S5PC1XX - IRQ register definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_REGS_IRQ_H
#define __ASM_ARCH_REGS_IRQ_H __FILE__

#include <asm/hardware/vic.h>

/* interrupt controller */
#define S5PC100_VIC0REG(x)          		((x) + S3C_VA_VIC0)
#define S5PC100_VIC1REG(x)          		((x) + S3C_VA_VIC1)
#define S5PC100_VIC2REG(x)         		((x) + S3C_VA_VIC2)

#endif /* __ASM_ARCH_REGS_IRQ_H */
