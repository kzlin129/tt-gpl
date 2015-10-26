/*
 * Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
 * Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <mach/hardware.h>
#include <asm/arch/bcm4760_reg.h>

#if   ( CONFIG_DEBUG_BCM_UART == 0 )
	#define URT_REG_BASE_ADDR	URT0_REG_BASE_ADDR
#elif ( CONFIG_DEBUG_BCM_UART == 1 )
	#define URT_REG_BASE_ADDR	URT1_REG_BASE_ADDR
#else
	#define URT_REG_BASE_ADDR	URT2_REG_BASE_ADDR
#endif

	.macro enableuart rd, rx
		ldr	\rd, [ \rx, #URT2_R_UARTCR_SEL ]
		orr	\rd, \rd, #URT2_F_RXE_MASK
		orr	\rd, \rd, #URT2_F_TXE_MASK
		str	\rd, [ \rx, #URT2_R_UARTCR_SEL ]
	.endm

	.macro addruart, rx
#ifdef __BOOTSTRAP__
		ldr	\rx, =0xb2000
#else
		@ is MMU enabled?
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1
		ldreq	\rx, = URT2_REG_BASE_ADDR
		ldrne	\rx, = IO_ADDRESS(URT2_REG_BASE_ADDR)
#endif
	.endm

	.macro	senduart,rd,rx
		strb 	\rd, [\rx, #URT2_R_UARTDR_SEL ]
	.endm

	.macro	busyuart, rd, rx
		enableuart \rd, \rx
1001:
		ldr	\rd, [ \rx, #URT2_R_UARTFR_SEL ]
		tst	\rd, #URT2_F_BUSY_MASK
		bne	1001b
	.endm

	.macro	waituart,rd,rx
		enableuart \rd, \rx
1001:
		ldr	\rd, [ \rx, #URT2_R_UARTFR_SEL ]
		tst	\rd, #URT2_F_BUSY_MASK
		bne	1001b
	.endm
