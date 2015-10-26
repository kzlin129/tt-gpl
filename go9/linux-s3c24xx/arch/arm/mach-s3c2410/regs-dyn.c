/* arch/arm/mach-s3c2410/regs-dyn.c
 *
 * S3C24xx dynamic register definitions.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/stddef.h>

#include <asm/arch/map.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-sdi.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-dyn.h>

unsigned s3c24xx_sdidat   = S3C2440_SDIDAT;
unsigned s3c24xx_sdiimsk  = S3C2440_SDIIMSK; /* All models != s3c2410 have the s3c2440 location */

#ifdef CONFIG_CPU_S3C2412
s3c24xx_reg_t s3c24xx_misccr   = S3C2410_MISCCR;
s3c24xx_reg_t s3c24xx_dclkcon  = S3C2410_DCLKCON;
s3c24xx_reg_t s3c24xx_mslcon   = S3C2410_MSLCON;
s3c24xx_reg_t s3c24xx_extint0  = S3C2410_EXTINT0;
s3c24xx_reg_t s3c24xx_extint1  = S3C2410_EXTINT1;
s3c24xx_reg_t s3c24xx_extint2  = S3C2410_EXTINT2;
s3c24xx_reg_t s3c24xx_einflt0  = S3C2410_EINFLT0;
s3c24xx_reg_t s3c24xx_einflt1  = S3C2410_EINFLT1;
s3c24xx_reg_t s3c24xx_einflt2  = S3C2410_EINFLT2;
s3c24xx_reg_t s3c24xx_einflt3  = S3C2410_EINFLT3;
s3c24xx_reg_t s3c24xx_eintmask = S3C2410_EINTMASK;
s3c24xx_reg_t s3c24xx_eintpend = S3C2410_EINTPEND;
s3c24xx_reg_t s3c24xx_gstatus0 = S3C2410_GSTATUS0;
s3c24xx_reg_t s3c24xx_gstatus1 = S3C2410_GSTATUS1;
s3c24xx_reg_t s3c24xx_gstatus2 = S3C2410_GSTATUS2;
s3c24xx_reg_t s3c24xx_gstatus3 = S3C2410_GSTATUS3;
s3c24xx_reg_t s3c24xx_gstatus4 = S3C2410_GSTATUS4;
s3c24xx_reg_t s3c24xx_gstatus5 = NULL;

void s3c2412_setregs(void)
{
	s3c24xx_misccr   = S3C2412_MISCCR;
	s3c24xx_dclkcon  = S3C2412_DCLKCON;
	s3c24xx_mslcon   = S3C2412_MSLCON;
	s3c24xx_extint0  = S3C2412_EXTINT0;
	s3c24xx_extint1  = S3C2412_EXTINT1;
	s3c24xx_extint2  = S3C2412_EXTINT2;
	s3c24xx_einflt0  = S3C2412_EINFLT0;
	s3c24xx_einflt1  = S3C2412_EINFLT1;
	s3c24xx_einflt2  = S3C2412_EINFLT2;
	s3c24xx_einflt3  = S3C2412_EINFLT3;
	s3c24xx_eintmask = S3C2412_EINTMASK;
	s3c24xx_eintpend = S3C2412_EINTPEND;
	s3c24xx_gstatus0 = S3C2412_GSTATUS0;
	s3c24xx_gstatus1 = S3C2412_GSTATUS1;
	s3c24xx_gstatus2 = S3C2412_GSTATUS2;
	s3c24xx_gstatus3 = S3C2412_GSTATUS3;
	s3c24xx_gstatus4 = S3C2412_GSTATUS4;
	s3c24xx_gstatus5 = S3C2412_GSTATUS5;

	/* See include/asm-arm/arch-s3c2410/entry-macro.S */
	s3c24xx_eintofs += S3C2412_EINTMASK - S3C2410_EINTMASK;
}
#endif /* CONFIG_CPU_S3C2412 */

/* EOF */
