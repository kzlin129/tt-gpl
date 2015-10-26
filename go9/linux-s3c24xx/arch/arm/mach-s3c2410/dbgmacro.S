/* arch/arm/mach-s3c2410/dbgmacro.S
 *
 * Some debug macros for use in low-level code.
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

	.macro waitabit, rx
	mov	\rx, #0x01000000	@ wait a bit...
1:	subs	\rx, \rx, #1
	bne	1b
	.endm

	.macro poke_rstcon_p, rd, rx
	mov	\rd, #S3C2410_PA_CLKPWR
	ldr	\rx, =0x0001ffff
	str	\rx, [ \rd, #0x64 ]	@ poke RSTCON
	.endm

	.macro poke_rstcon_v, rd, rx
	mov	\rd, #S3C24XX_VA_CLKPWR
	ldr	\rx, =0x0001ffff
	str	\rx, [ \rd, #0x64 ]	@ poke RSTCON
	.endm

	.macro backlight_off_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0x14 ]	@ read GPBDAT
	bic	\rx, \rx, #(1<<0)	@ turn off GPB0 (BACKLIGHT_PWM)
	orr	\rx, \rx, #(1<<1)	@ turn on  GPB1 (nBACKLIGHT_EN)
	str	\rx, [ \rd, #0x14 ]	@ write GPBDAT
	ldr	\rx, [ \rd, #0x10 ]	@ read GPBCON
	bic	\rx, \rx, #(3<<0)	@ set GPB0 (BACKLIGHT_PWM) ...
	orr	\rx, \rx, #(1<<0)	@ ... to output
	bic	\rx, \rx, #(3<<2)	@ set GPB1 (nBACKLIGHT_EN) ...
	orr	\rx, \rx, #(1<<2)	@ ... to output
	str	\rx, [ \rd, #0x10 ]	@ write GPBCON
	.endm

	.macro backlight_on_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0x14 ]	@ read GPBDAT
	orr	\rx, \rx, #(1<<0)	@ turn on  GPB0 (BACKLIGHT_PWM)
	bic	\rx, \rx, #(1<<1)	@ turn off GPB1 (nBACKLIGHT_EN)
	str	\rx, [ \rd, #0x14 ]	@ write GPBDAT
	ldr	\rx, [ \rd, #0x10 ]	@ read GPBCON
	bic	\rx, \rx, #(3<<0)	@ set GPB0 (BACKLIGHT_PWM) ...
	orr	\rx, \rx, #(1<<0)	@ ... to output
	bic	\rx, \rx, #(3<<2)	@ set GPB1 (nBACKLIGHT_EN) ...
	orr	\rx, \rx, #(1<<2)	@ ... to output
	str	\rx, [ \rd, #0x10 ]	@ write GPBCON
	.endm

	.macro dock_on_milan_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0xd4 ]	@ read GPJDAT
	bic	\rx, \rx, #(1<<12)	@ turn off GPJ12 (nDOCK_PWREN)
	str	\rx, [ \rd, #0xd4 ]	@ write GPJDAT
	ldr	\rx, [ \rd, #0xd0 ]	@ read GPJCON
	bic	\rx, \rx, #(3<<24)	@ set GPJ12 (nDOCK_PWREN) ...
	orr	\rx, \rx, #(1<<24)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPJCON
	.endm

	.macro dock_off_milan_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0xd4 ]	@ read GPJDAT
	orr	\rx, \rx, #(1<<12)	@ turn on GPJ12 (nDOCK_PWREN)
	str	\rx, [ \rd, #0xd4 ]	@ write GPJDAT
	ldr	\rx, [ \rd, #0xd0 ]	@ read GPJCON
	bic	\rx, \rx, #(3<<24)	@ set GPJ12 (nDOCK_PWREN) ...
	orr	\rx, \rx, #(1<<24)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPJCON
	.endm

	.macro led_on_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0x64 ]	@ read GPGDAT
	orr	\rx, \rx, #(1<<10)	@ turn on GPG10 (LX_EN)
	str	\rx, [ \rd, #0x64 ]	@ write GPGDAT
	ldr	\rx, [ \rd, #0x60 ]	@ read GPGCON
	bic	\rx, \rx, #(3<<20)	@ set GPG10 (LX_EN) ...
	orr	\rx, \rx, #(1<<20)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPGCON
	.endm

	.macro led_off_p, rd, rx
	mov	\rd, #S3C2410_PA_GPIO
	ldr	\rx, [ \rd, #0x64 ]	@ read GPGDAT
	bic	\rx, \rx, #(1<<10)	@ turn off GPG10 (LX_EN)
	str	\rx, [ \rd, #0x64 ]	@ write GPGDAT
	ldr	\rx, [ \rd, #0x60 ]	@ read GPGCON
	bic	\rx, \rx, #(3<<20)	@ set GPG10 (LX_EN) ...
	orr	\rx, \rx, #(1<<20)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPGCON
	.endm

	.macro led_on_v, rd, rx
	ldr	\rd, =S3C24XX_VA_GPIO
	ldr	\rx, [ \rd, #0x64 ]	@ read GPGDAT
	orr	\rx, \rx, #(1<<10)	@ turn on GPG10 (LX_EN)
	str	\rx, [ \rd, #0x64 ]	@ write GPGDAT
	ldr	\rx, [ \rd, #0x60 ]	@ read GPGCON
	bic	\rx, \rx, #(3<<20)	@ set GPG10 (LX_EN) ...
	orr	\rx, \rx, #(1<<20)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPGCON
	.endm

	.macro led_off_v, rd, rx
	ldr	\rd, =S3C24XX_VA_GPIO
	ldr	\rx, [ \rd, #0x64 ]	@ read GPGDAT
	bic	\rx, \rx, #(1<<10)	@ turn off GPG10 (LX_EN)
	str	\rx, [ \rd, #0x64 ]	@ write GPGDAT
	ldr	\rx, [ \rd, #0x60 ]	@ read GPGCON
	bic	\rx, \rx, #(3<<20)	@ set GPG10 (LX_EN) ...
	orr	\rx, \rx, #(1<<20)	@ ... to output
	str	\rx, [ \rd, #0xd0 ]	@ write GPGCON
	.endm

	.macro	led_blink_p, n, rx, ry, rz
	mov	\rx, #\n
2:	led_on_p \ry, \rz
	waitabit \ry
	led_off_p \ry, \rz
	waitabit \ry
	subs	\rx, \rx, #1
	bne	2b
	.endm

	.macro	led_blink_v, n, rx, ry, rz
	mov	\rx, #\n
2:	led_on_v \ry, \rz
	waitabit \ry
	led_off_v \ry, \rz
	waitabit \ry
	subs	\rx, \rx, #1
	bne	2b
	.endm

/* EOF */
