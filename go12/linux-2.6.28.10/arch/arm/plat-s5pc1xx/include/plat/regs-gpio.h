/* linux/arch/arm/plat-s5pc1xx/include/plat/regs-gpio.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5PC1XX - GPIO register definitions
 */

#ifndef __ASM_PLAT_S5PC1XX_REGS_GPIO_H
#define __ASM_PLAT_S5PC1XX_REGS_GPIO_H __FILE__

/* Base addresses for each of the banks */
#if defined(CONFIG_CPU_S5PC100)

#include <plat/gpio-bank-a0.h>
#include <plat/gpio-bank-a1.h>
#include <plat/gpio-bank-b.h>
#include <plat/gpio-bank-c.h>
#include <plat/gpio-bank-d.h>
#include <plat/gpio-bank-e0.h>
#include <plat/gpio-bank-e1.h>
#include <plat/gpio-bank-f0.h>
#include <plat/gpio-bank-f1.h>
#include <plat/gpio-bank-f2.h>
#include <plat/gpio-bank-f3.h>
#include <plat/gpio-bank-g0.h>
#include <plat/gpio-bank-g1.h>
#include <plat/gpio-bank-g2.h>
#include <plat/gpio-bank-g3.h>
#include <plat/gpio-bank-h0.h>
#include <plat/gpio-bank-h1.h>
#include <plat/gpio-bank-h2.h>
#include <plat/gpio-bank-h3.h>
#include <plat/gpio-bank-i.h>
#include <plat/gpio-bank-j0.h>
#include <plat/gpio-bank-j1.h>
#include <plat/gpio-bank-j2.h>
#include <plat/gpio-bank-j3.h>
#include <plat/gpio-bank-j4.h>
#include <plat/gpio-bank-k0.h>
#include <plat/gpio-bank-k1.h>
#include <plat/gpio-bank-k2.h>
#include <plat/gpio-bank-k3.h>

#include <mach/map.h>

#define S5PC1XX_GPA0_BASE	(S5PC1XX_VA_GPIO + 0x0000)
#define S5PC1XX_GPA1_BASE	(S5PC1XX_VA_GPIO + 0x0020)
#define S5PC1XX_GPB_BASE	(S5PC1XX_VA_GPIO + 0x0040)
#define S5PC1XX_GPC_BASE	(S5PC1XX_VA_GPIO + 0x0060)
#define S5PC1XX_GPD_BASE	(S5PC1XX_VA_GPIO + 0x0080)
#define S5PC1XX_GPE0_BASE	(S5PC1XX_VA_GPIO + 0x00A0)
#define S5PC1XX_GPE1_BASE	(S5PC1XX_VA_GPIO + 0x00C0)
#define S5PC1XX_GPF0_BASE	(S5PC1XX_VA_GPIO + 0x00E0)
#define S5PC1XX_GPF1_BASE	(S5PC1XX_VA_GPIO + 0x0100)
#define S5PC1XX_GPF2_BASE	(S5PC1XX_VA_GPIO + 0x0120)
#define S5PC1XX_GPF3_BASE	(S5PC1XX_VA_GPIO + 0x0140)
#define S5PC1XX_GPG0_BASE	(S5PC1XX_VA_GPIO + 0x0160)
#define S5PC1XX_GPG1_BASE	(S5PC1XX_VA_GPIO + 0x0180)
#define S5PC1XX_GPG2_BASE	(S5PC1XX_VA_GPIO + 0x01A0)
#define S5PC1XX_GPG3_BASE	(S5PC1XX_VA_GPIO + 0x01C0)
#define S5PC1XX_GPH0_BASE	(S5PC1XX_VA_GPIO + 0x0C00)
#define S5PC1XX_GPH1_BASE	(S5PC1XX_VA_GPIO + 0x0C20)
#define S5PC1XX_GPH2_BASE	(S5PC1XX_VA_GPIO + 0x0C40)
#define S5PC1XX_GPH3_BASE	(S5PC1XX_VA_GPIO + 0x0C60)
#define S5PC1XX_GPI_BASE	(S5PC1XX_VA_GPIO + 0x01E0)
#define S5PC1XX_GPJ0_BASE	(S5PC1XX_VA_GPIO + 0x0200)
#define S5PC1XX_GPJ1_BASE	(S5PC1XX_VA_GPIO + 0x0220)
#define S5PC1XX_GPJ2_BASE	(S5PC1XX_VA_GPIO + 0x0240)
#define S5PC1XX_GPJ3_BASE	(S5PC1XX_VA_GPIO + 0x0260)
#define S5PC1XX_GPJ4_BASE	(S5PC1XX_VA_GPIO + 0x0280)
#define S5PC1XX_GPK0_BASE	(S5PC1XX_VA_GPIO + 0x02A0)
#define S5PC1XX_GPK1_BASE	(S5PC1XX_VA_GPIO + 0x02C0)
#define S5PC1XX_GPK2_BASE	(S5PC1XX_VA_GPIO + 0x02E0)
#define S5PC1XX_GPK3_BASE	(S5PC1XX_VA_GPIO + 0x0300)
#define S5PC1XX_MP00_BASE	(S5PC1XX_VA_GPIO + 0x0320)
#define S5PC1XX_MP01_BASE	(S5PC1XX_VA_GPIO + 0x0340)
#define S5PC1XX_MP02_BASE	(S5PC1XX_VA_GPIO + 0x0360)
#define S5PC1XX_MP03_BASE	(S5PC1XX_VA_GPIO + 0x0380)
#define S5PC1XX_MP04_BASE	(S5PC1XX_VA_GPIO + 0x03A0)


#define S5PC1XX_UHOST	(S5PC1XX_VA_GPIO + 0x0B68)
#define S5PC1XX_PDNEN	(S5PC1XX_VA_GPIO + 0x0F80)

#elif defined(CONFIG_CPU_S5PC110)

#include <plat/gpio-bank-a0.h>
#include <plat/gpio-bank-a1.h>
#include <plat/gpio-bank-b.h>
#include <plat/gpio-bank-c0.h>
#include <plat/gpio-bank-c1.h>
#include <plat/gpio-bank-d0.h>
#include <plat/gpio-bank-d1.h>
#include <plat/gpio-bank-e0.h>
#include <plat/gpio-bank-e1.h>
#include <plat/gpio-bank-f0.h>
#include <plat/gpio-bank-f1.h>
#include <plat/gpio-bank-f2.h>
#include <plat/gpio-bank-f3.h>
#include <plat/gpio-bank-g0.h>
#include <plat/gpio-bank-g1.h>
#include <plat/gpio-bank-g2.h>
#include <plat/gpio-bank-g3.h>
#include <plat/gpio-bank-h0.h>
#include <plat/gpio-bank-h1.h>
#include <plat/gpio-bank-h2.h>
#include <plat/gpio-bank-h3.h>
#include <plat/gpio-bank-i.h>
#include <plat/gpio-bank-j0.h>
#include <plat/gpio-bank-j1.h>
#include <plat/gpio-bank-j2.h>
#include <plat/gpio-bank-j3.h>
#include <plat/gpio-bank-j4.h>
#include <plat/gpio-bank-mp01.h>
#include <plat/gpio-bank-mp02.h>
#include <plat/gpio-bank-mp03.h>
#include <plat/gpio-bank-mp04.h>
#include <plat/gpio-bank-mp05.h>
#include <plat/gpio-bank-mp06.h>
#include <plat/gpio-bank-mp07.h>
#include <plat/gpio-bank-mp10.h>
#include <plat/gpio-bank-mp11.h>
#include <plat/gpio-bank-mp12.h>
#include <plat/gpio-bank-mp13.h>
#include <plat/gpio-bank-mp14.h>
#include <plat/gpio-bank-mp15.h>
#include <plat/gpio-bank-mp16.h>
#include <plat/gpio-bank-mp17.h>
#include <plat/gpio-bank-mp18.h>
#include <plat/gpio-bank-mp20.h>
#include <plat/gpio-bank-mp21.h>
#include <plat/gpio-bank-mp22.h>
#include <plat/gpio-bank-mp23.h>
#include <plat/gpio-bank-mp24.h>
#include <plat/gpio-bank-mp25.h>
#include <plat/gpio-bank-mp26.h>
#include <plat/gpio-bank-mp27.h>
#include <plat/gpio-bank-mp28.h>


#include <mach/map.h>

#define S5PC1XX_GPA0_BASE	(S5PC1XX_VA_GPIO + 0x0000)
#define S5PC1XX_GPA1_BASE	(S5PC1XX_VA_GPIO + 0x0020)
#define S5PC1XX_GPB_BASE	(S5PC1XX_VA_GPIO + 0x0040)
#define S5PC1XX_GPC0_BASE	(S5PC1XX_VA_GPIO + 0x0060)
#define S5PC1XX_GPC1_BASE	(S5PC1XX_VA_GPIO + 0x0080)
#define S5PC1XX_GPD0_BASE	(S5PC1XX_VA_GPIO + 0x00A0)
#define S5PC1XX_GPD1_BASE	(S5PC1XX_VA_GPIO + 0x00C0)
#define S5PC1XX_GPE0_BASE	(S5PC1XX_VA_GPIO + 0x00E0)
#define S5PC1XX_GPE1_BASE	(S5PC1XX_VA_GPIO + 0x0100)
#define S5PC1XX_GPF0_BASE	(S5PC1XX_VA_GPIO + 0x0120)
#define S5PC1XX_GPF1_BASE	(S5PC1XX_VA_GPIO + 0x0140)
#define S5PC1XX_GPF2_BASE	(S5PC1XX_VA_GPIO + 0x0160)
#define S5PC1XX_GPF3_BASE	(S5PC1XX_VA_GPIO + 0x0180)
#define S5PC1XX_GPG0_BASE	(S5PC1XX_VA_GPIO + 0x01A0)
#define S5PC1XX_GPG1_BASE	(S5PC1XX_VA_GPIO + 0x01C0)
#define S5PC1XX_GPG2_BASE	(S5PC1XX_VA_GPIO + 0x01E0)
#define S5PC1XX_GPG3_BASE	(S5PC1XX_VA_GPIO + 0x0200)
#define S5PC1XX_GPH0_BASE	(S5PC1XX_VA_GPIO + 0x0C00)
#define S5PC1XX_GPH1_BASE	(S5PC1XX_VA_GPIO + 0x0C20)
#define S5PC1XX_GPH2_BASE	(S5PC1XX_VA_GPIO + 0x0C40)
#define S5PC1XX_GPH3_BASE	(S5PC1XX_VA_GPIO + 0x0C60)
#define S5PC1XX_GPI_BASE	(S5PC1XX_VA_GPIO + 0x0220)
#define S5PC1XX_GPJ0_BASE	(S5PC1XX_VA_GPIO + 0x0240)
#define S5PC1XX_GPJ1_BASE	(S5PC1XX_VA_GPIO + 0x0260)
#define S5PC1XX_GPJ2_BASE	(S5PC1XX_VA_GPIO + 0x0280)
#define S5PC1XX_GPJ3_BASE	(S5PC1XX_VA_GPIO + 0x02A0)
#define S5PC1XX_GPJ4_BASE	(S5PC1XX_VA_GPIO + 0x02C0)
#define S5PC1XX_MP01_BASE	(S5PC1XX_VA_GPIO + 0x02E0)
#define S5PC1XX_MP02_BASE	(S5PC1XX_VA_GPIO + 0x0300)
#define S5PC1XX_MP03_BASE	(S5PC1XX_VA_GPIO + 0x0320)
#define S5PC1XX_MP04_BASE	(S5PC1XX_VA_GPIO + 0x0340)
#define S5PC1XX_MP05_BASE	(S5PC1XX_VA_GPIO + 0x0360)
#define S5PC1XX_MP06_BASE	(S5PC1XX_VA_GPIO + 0x0380)
#define S5PC1XX_MP07_BASE	(S5PC1XX_VA_GPIO + 0x03A0)
#define S5PC1XX_MP10_BASE	(S5PC1XX_VA_GPIO + 0x03C0)
#define S5PC1XX_MP11_BASE	(S5PC1XX_VA_GPIO + 0x03E0)
#define S5PC1XX_MP12_BASE	(S5PC1XX_VA_GPIO + 0x0400)
#define S5PC1XX_MP13_BASE	(S5PC1XX_VA_GPIO + 0x0420)
#define S5PC1XX_MP14_BASE	(S5PC1XX_VA_GPIO + 0x0440)
#define S5PC1XX_MP15_BASE	(S5PC1XX_VA_GPIO + 0x0460)
#define S5PC1XX_MP16_BASE	(S5PC1XX_VA_GPIO + 0x0480)
#define S5PC1XX_MP17_BASE	(S5PC1XX_VA_GPIO + 0x04A0)
#define S5PC1XX_MP18_BASE	(S5PC1XX_VA_GPIO + 0x04C0)
#define S5PC1XX_MP20_BASE	(S5PC1XX_VA_GPIO + 0x04E0)
#define S5PC1XX_MP21_BASE	(S5PC1XX_VA_GPIO + 0x0500)
#define S5PC1XX_MP22_BASE	(S5PC1XX_VA_GPIO + 0x0520)
#define S5PC1XX_MP23_BASE	(S5PC1XX_VA_GPIO + 0x0540)
#define S5PC1XX_MP24_BASE	(S5PC1XX_VA_GPIO + 0x0560)
#define S5PC1XX_MP25_BASE	(S5PC1XX_VA_GPIO + 0x0580)
#define S5PC1XX_MP26_BASE	(S5PC1XX_VA_GPIO + 0x05A0)
#define S5PC1XX_MP27_BASE	(S5PC1XX_VA_GPIO + 0x05C0)
#define S5PC1XX_MP28_BASE	(S5PC1XX_VA_GPIO + 0x05E0)
#else
#error "CPU type should be defined(S5PC100, S5PC110)"
#endif

#endif /* __ASM_PLAT_S5PC1XX_REGS_GPIO_H */

