/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  linux/include/asm-arm/arch-bcm476x/hardware.h
 *
 *  This file contains the hardware definitions of the BCM476X.
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <asm/arch/platform.h>

//TODO: joyjit
// Macros to make managing spinlocks a bit more controlled in terms of naming.
// See reg_gpio.h, reg_irq.h, arch.c, gpio.c for example usage.
#if defined( __KERNEL__ )
#define HW_DECLARE_SPINLOCK(name)  spinlock_t g##name##RegLock = SPIN_LOCK_UNLOCKED;
#define HW_EXTERN_SPINLOCK(name)   extern spinlock_t g##name##RegLock;
#define HW_IRQ_SAVE(name, val)     spin_lock_irqsave(&g##name##RegLock,(val))
#define HW_IRQ_RESTORE(name, val)  spin_unlock_irqrestore(&g##name##RegLock,(val))
#else
#define HW_DECLARE_SPINLOCK(name)
#define HW_EXTERN_SPINLOCK(name)
#define HW_IRQ_SAVE(name, val)     {(void)(name);(void)(val);}
#define HW_IRQ_RESTORE(name, val)  {(void)(name);(void)(val);}
#endif


/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define IO_BASE			0xd0000000      // VA of IO 
#define IO_SIZE			0x2fffffff  	// How much?
#define IO_START		BCM47XX_ARM_PERIPH_BASE      // PA of IO

#define IO_ADDRESS(x)		(IO_BASE + (x))
#define IO_ADDRESS_TO_PHY(x)	((x) - IO_BASE)
#define __REG32(x)   (*((volatile u32 *)(x)))
#define __REG16(x)   (*((volatile u16 *)(x)))
#define __REG8(x) (*((volatile u8  *)(x)))

#define pcibios_assign_all_busses()	1


#endif

