/* linux/arch/arm/plat-s5p64xx/include/plat/regs-gpio.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5P64XX - GPIO register definitions
 */

#ifndef __ASM_PLAT_S5P64XX_REGS_GPIO_H
#define __ASM_PLAT_S5P64XX_REGS_GPIO_H __FILE__

#include <plat/gpio-bank-a.h>
#include <plat/gpio-bank-b.h>
#include <plat/gpio-bank-c.h>
#include <plat/gpio-bank-f.h>
#include <plat/gpio-bank-g.h>
#include <plat/gpio-bank-h.h>
#include <plat/gpio-bank-i.h>
#include <plat/gpio-bank-j.h>
#include <plat/gpio-bank-n.h>
#include <plat/gpio-bank-p.h>
#include <plat/gpio-bank-r.h>
#include <mach/map.h>

/* Base addresses for each of the banks */

#define S5P64XX_GPA_BASE	(S5P64XX_VA_GPIO + 0x0000)
#define S5P64XX_GPB_BASE	(S5P64XX_VA_GPIO + 0x0020)
#define S5P64XX_GPC_BASE	(S5P64XX_VA_GPIO + 0x0040)
#define S5P64XX_GPF_BASE	(S5P64XX_VA_GPIO + 0x00A0)
#define S5P64XX_GPG_BASE	(S5P64XX_VA_GPIO + 0x00C0)
#define S5P64XX_GPH_BASE	(S5P64XX_VA_GPIO + 0x00E0)
#define S5P64XX_GPI_BASE	(S5P64XX_VA_GPIO + 0x0100)
#define S5P64XX_GPJ_BASE	(S5P64XX_VA_GPIO + 0x0120)
#define S5P64XX_GPN_BASE	(S5P64XX_VA_GPIO + 0x0830)
#define S5P64XX_GPP_BASE	(S5P64XX_VA_GPIO + 0x0160)
#define S5P64XX_GPR_BASE	(S5P64XX_VA_GPIO + 0x0290)
#define S5P64XX_SPC_BASE	(S5P64XX_VA_GPIO + 0x01A0)
#define S5P64XX_SPC1_BASE	(S5P64XX_VA_GPIO + 0x02B0)

#define S5P64XX_EINT0CON0	(S5P64XX_VA_GPIO + 0x900)
#define S5P64XX_EINT0FLTCON0	(S5P64XX_VA_GPIO + 0x910)
#define S5P64XX_EINT0FLTCON1	(S5P64XX_VA_GPIO + 0x914)

#define S5P64XX_EINT0MASK	(S5P64XX_VA_GPIO + 0x920)
#define S5P64XX_EINT0PEND	(S5P64XX_VA_GPIO + 0x924)

#define S5P64XX_SLPEN		(S5P64XX_VA_GPIO + 0x930)

/* for lcd */
#define S5P64XX_SPCON_LCD_SEL_RGB	(1 << 0)
#define S5P64XX_SPCON_LCD_SEL_MASK	(3 << 0)

#endif /* __ASM_PLAT_S5P64XX_REGS_GPIO_H */

