/* arch/arm/mach-s3c2410/tomtomgo-iopins.h
 *
 * Macros for TomTom GO pin structure definitions.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_S3C2410_TOMTOMGO_IOPINS_H
#define __ARCH_ARM_MACH_S3C2410_TOMTOMGO_IOPINS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PORT_GPA        0x00
#define PORT_GPB        0x01
#define PORT_GPC        0x02
#define PORT_GPD        0x03
#define PORT_GPE        0x04
#define PORT_GPF        0x05
#define PORT_GPG        0x06
#define PORT_GPH        0x07
#define PORT_GPJ_2413   0x08
#define PORT_GPJ_2440   0x0d
#define PORT_GPK        0x0e
#define PORT_GPL        0x0f
#define PORT_GPM        0x10
#define PORT_GPIIC      0x09

#define PIN_AMOUNT              ((gopin_t)0x0200)           /* A maximum of 32*16=512 pins is possible */
#define PIN_MASK                ((gopin_t)0x01ff)           /* A maximum of 32*16=512 pins is possible */
#define PIN_PULL_DOWN           ((gopin_t)0x8000)           /* Enable pulldown */
#define PIN_INVERTED            ((gopin_t)0x4000)           /* Invert input or output */
#define PIN_PULL_UP             ((gopin_t)0x2000)           /* Enable pullup */
#define PIN_FLOAT               ((gopin_t)0x1000)           /* Make pin floating when deactivated */
#define PIN_OPEN_EMITTER        (PIN_FLOAT)                 /* Float pin when deactivated, high when activated */
#define PIN_OPEN_COLLECTOR      (PIN_FLOAT | PIN_INVERTED)  /* Float pin when deactivated, low when activated */
#define PIN_ACTIVATE_ON_SUSPEND   ((gopin_t)0x0800)           /* Activate pin on suspend */
#define PIN_DEACTIVATE_ON_SUSPEND ((gopin_t)0x0000)           /* Deactivate pin on suspend */
#define PIN_INPUT_ON_SUSPEND      ((gopin_t)0x0400)           /* Make input pin on suspend */
#define PIN_IGNORE_ON_SUSPEND     ((gopin_t)0x0200)           /* Ignore pin on suspend */

#define PIN_IS_NOT_USED(pin)   ((pin) == 0)
#define PIN_IS_USED(pin)       ((pin) != 0)
#define PIN_IS_INVERTED(pin)  (((pin) & PIN_INVERTED ) == PIN_INVERTED)
#define PIN_IS_FLOATING(pin)  (((pin) & PIN_FLOAT ) == PIN_FLOAT)

#define PIN_SET(pin,mask) do { gotype_current.pins.pin = mask; } while (0)
#define PIN_ADD_MASK(pin,mask) do { if (gotype_current.pins.pin) gotype_current.pins.pin |= mask; } while (0)
#define PIN_SET_MASK(pin,mask) do { if (gotype_current.pins.pin) gotype_current.pins.pin = (gotype_current.pins.pin & ~(PIN_ACTIVATE_ON_SUSPEND | PIN_INPUT_ON_SUSPEND | PIN_IGNORE_ON_SUSPEND)) | mask; } while (0)
#define VALUE_SET(destination,source) do { gotype_current.destination = source; } while (0)
#define STRING_SET(destination,source) do { strlcpy(gotype_current.destination,source,sizeof(gotype_current.destination)); } while (0)

#define PIN_STEP_SIZE_BITS 5
#define PIN_STEP_SIZE_MASK ((1 << PIN_STEP_SIZE_BITS) - 1)

#define PIN_NO(port,pinnr) ((port << PIN_STEP_SIZE_BITS) + (pinnr))

#define GET_PIN(pin)    ((pin) & PIN_MASK)                          /* Get combined pin & port number */
#define GET_PORTNR(pin) (GET_PIN(pin) >> PIN_STEP_SIZE_BITS)        /* Get port number */
#define GET_PINNR(pin)  ((pin) & PIN_STEP_SIZE_MASK)                /* Get pin number */

#define PIN_GPA0	PIN_NO(PORT_GPA,0)
#define PIN_GPA1	PIN_NO(PORT_GPA,1)
#define PIN_GPA2	PIN_NO(PORT_GPA,2)
#define PIN_GPA3	PIN_NO(PORT_GPA,3)
#define PIN_GPA4	PIN_NO(PORT_GPA,4)
#define PIN_GPA5	PIN_NO(PORT_GPA,5)
#define PIN_GPA6	PIN_NO(PORT_GPA,6)
#define PIN_GPA7	PIN_NO(PORT_GPA,7)
#define PIN_GPA8	PIN_NO(PORT_GPA,8)
#define PIN_GPA9	PIN_NO(PORT_GPA,9)
#define PIN_GPA10	PIN_NO(PORT_GPA,10)
#define PIN_GPA11	PIN_NO(PORT_GPA,11)
#define PIN_GPA12	PIN_NO(PORT_GPA,12)
#define PIN_GPA13	PIN_NO(PORT_GPA,13)
#define PIN_GPA14	PIN_NO(PORT_GPA,14)
#define PIN_GPA15	PIN_NO(PORT_GPA,15)
#define PIN_GPA16	PIN_NO(PORT_GPA,16)
#define PIN_GPA17	PIN_NO(PORT_GPA,17)
#define PIN_GPA18	PIN_NO(PORT_GPA,18)
#define PIN_GPA19	PIN_NO(PORT_GPA,19)
#define PIN_GPA20	PIN_NO(PORT_GPA,20)
#define PIN_GPA21	PIN_NO(PORT_GPA,21)
#define PIN_GPA22	PIN_NO(PORT_GPA,22)
#define PIN_GPA23	PIN_NO(PORT_GPA,23)
#define PIN_GPA24	PIN_NO(PORT_GPA,24)
#define PIN_GPA25	PIN_NO(PORT_GPA,25)
#define PIN_GPA26	PIN_NO(PORT_GPA,26)
#define PIN_GPA27	PIN_NO(PORT_GPA,27)
#define PIN_GPA28	PIN_NO(PORT_GPA,28)
#define PIN_GPA29	PIN_NO(PORT_GPA,29)
#define PIN_GPA30	PIN_NO(PORT_GPA,30)
#define PIN_GPA31	PIN_NO(PORT_GPA,31)

#define PIN_GPB0	PIN_NO(PORT_GPB,0)
#define PIN_GPB1	PIN_NO(PORT_GPB,1)
#define PIN_GPB2	PIN_NO(PORT_GPB,2)
#define PIN_GPB3	PIN_NO(PORT_GPB,3)
#define PIN_GPB4	PIN_NO(PORT_GPB,4)
#define PIN_GPB5	PIN_NO(PORT_GPB,5)
#define PIN_GPB6	PIN_NO(PORT_GPB,6)
#define PIN_GPB7	PIN_NO(PORT_GPB,7)
#define PIN_GPB8	PIN_NO(PORT_GPB,8)
#define PIN_GPB9	PIN_NO(PORT_GPB,9)
#define PIN_GPB10	PIN_NO(PORT_GPB,10)
#define PIN_GPB11	PIN_NO(PORT_GPB,11)
#define PIN_GPB12	PIN_NO(PORT_GPB,12)
#define PIN_GPB13	PIN_NO(PORT_GPB,13)
#define PIN_GPB14	PIN_NO(PORT_GPB,14)
#define PIN_GPB15	PIN_NO(PORT_GPB,15)

#define PIN_GPC0	PIN_NO(PORT_GPC,0)
#define PIN_GPC1	PIN_NO(PORT_GPC,1)
#define PIN_GPC2	PIN_NO(PORT_GPC,2)
#define PIN_GPC3	PIN_NO(PORT_GPC,3)
#define PIN_GPC4	PIN_NO(PORT_GPC,4)
#define PIN_GPC5	PIN_NO(PORT_GPC,5)
#define PIN_GPC6	PIN_NO(PORT_GPC,6)
#define PIN_GPC7	PIN_NO(PORT_GPC,7)
#define PIN_GPC8	PIN_NO(PORT_GPC,8)
#define PIN_GPC9	PIN_NO(PORT_GPC,9)
#define PIN_GPC10	PIN_NO(PORT_GPC,10)
#define PIN_GPC11	PIN_NO(PORT_GPC,11)
#define PIN_GPC12	PIN_NO(PORT_GPC,12)
#define PIN_GPC13	PIN_NO(PORT_GPC,13)
#define PIN_GPC14	PIN_NO(PORT_GPC,14)
#define PIN_GPC15	PIN_NO(PORT_GPC,15)

#define PIN_GPD0	PIN_NO(PORT_GPD,0)
#define PIN_GPD1	PIN_NO(PORT_GPD,1)
#define PIN_GPD2	PIN_NO(PORT_GPD,2)
#define PIN_GPD3	PIN_NO(PORT_GPD,3)
#define PIN_GPD4	PIN_NO(PORT_GPD,4)
#define PIN_GPD5	PIN_NO(PORT_GPD,5)
#define PIN_GPD6	PIN_NO(PORT_GPD,6)
#define PIN_GPD7	PIN_NO(PORT_GPD,7)
#define PIN_GPD8	PIN_NO(PORT_GPD,8)
#define PIN_GPD9	PIN_NO(PORT_GPD,9)
#define PIN_GPD10	PIN_NO(PORT_GPD,10)
#define PIN_GPD11	PIN_NO(PORT_GPD,11)
#define PIN_GPD12	PIN_NO(PORT_GPD,12)
#define PIN_GPD13	PIN_NO(PORT_GPD,13)
#define PIN_GPD14	PIN_NO(PORT_GPD,14)
#define PIN_GPD15	PIN_NO(PORT_GPD,15)

#define PIN_GPE0	PIN_NO(PORT_GPE,0)
#define PIN_GPE1	PIN_NO(PORT_GPE,1)
#define PIN_GPE2	PIN_NO(PORT_GPE,2)
#define PIN_GPE3	PIN_NO(PORT_GPE,3)
#define PIN_GPE4	PIN_NO(PORT_GPE,4)
#define PIN_GPE5	PIN_NO(PORT_GPE,5)
#define PIN_GPE6	PIN_NO(PORT_GPE,6)
#define PIN_GPE7	PIN_NO(PORT_GPE,7)
#define PIN_GPE8	PIN_NO(PORT_GPE,8)
#define PIN_GPE9	PIN_NO(PORT_GPE,9)
#define PIN_GPE10	PIN_NO(PORT_GPE,10)
#define PIN_GPE11	PIN_NO(PORT_GPE,11)
#define PIN_GPE12	PIN_NO(PORT_GPE,12)
#define PIN_GPE13	PIN_NO(PORT_GPE,13)
#define PIN_GPE14	PIN_NO(PORT_GPE,14)
#define PIN_GPE15	PIN_NO(PORT_GPE,15)

#define PIN_GPF0	PIN_NO(PORT_GPF,0)
#define PIN_GPF1	PIN_NO(PORT_GPF,1)
#define PIN_GPF2	PIN_NO(PORT_GPF,2)
#define PIN_GPF3	PIN_NO(PORT_GPF,3)
#define PIN_GPF4	PIN_NO(PORT_GPF,4)
#define PIN_GPF5	PIN_NO(PORT_GPF,5)
#define PIN_GPF6	PIN_NO(PORT_GPF,6)
#define PIN_GPF7	PIN_NO(PORT_GPF,7)
#define PIN_GPF8	PIN_NO(PORT_GPF,8)
#define PIN_GPF9	PIN_NO(PORT_GPF,9)
#define PIN_GPF10	PIN_NO(PORT_GPF,10)
#define PIN_GPF11	PIN_NO(PORT_GPF,11)
#define PIN_GPF12	PIN_NO(PORT_GPF,12)
#define PIN_GPF13	PIN_NO(PORT_GPF,13)
#define PIN_GPF14	PIN_NO(PORT_GPF,14)
#define PIN_GPF15	PIN_NO(PORT_GPF,15)

#define PIN_GPG0	PIN_NO(PORT_GPG,0)
#define PIN_GPG1	PIN_NO(PORT_GPG,1)
#define PIN_GPG2	PIN_NO(PORT_GPG,2)
#define PIN_GPG3	PIN_NO(PORT_GPG,3)
#define PIN_GPG4	PIN_NO(PORT_GPG,4)
#define PIN_GPG5	PIN_NO(PORT_GPG,5)
#define PIN_GPG6	PIN_NO(PORT_GPG,6)
#define PIN_GPG7	PIN_NO(PORT_GPG,7)
#define PIN_GPG8	PIN_NO(PORT_GPG,8)
#define PIN_GPG9	PIN_NO(PORT_GPG,9)
#define PIN_GPG10	PIN_NO(PORT_GPG,10)
#define PIN_GPG11	PIN_NO(PORT_GPG,11)
#define PIN_GPG12	PIN_NO(PORT_GPG,12)
#define PIN_GPG13	PIN_NO(PORT_GPG,13)
#define PIN_GPG14	PIN_NO(PORT_GPG,14)
#define PIN_GPG15	PIN_NO(PORT_GPG,15)

#define PIN_GPH0	PIN_NO(PORT_GPH,0)
#define PIN_GPH1	PIN_NO(PORT_GPH,1)
#define PIN_GPH2	PIN_NO(PORT_GPH,2)
#define PIN_GPH3	PIN_NO(PORT_GPH,3)
#define PIN_GPH4	PIN_NO(PORT_GPH,4)
#define PIN_GPH5	PIN_NO(PORT_GPH,5)
#define PIN_GPH6	PIN_NO(PORT_GPH,6)
#define PIN_GPH7	PIN_NO(PORT_GPH,7)
#define PIN_GPH8	PIN_NO(PORT_GPH,8)
#define PIN_GPH9	PIN_NO(PORT_GPH,9)
#define PIN_GPH10	PIN_NO(PORT_GPH,10)
#define PIN_GPH11	PIN_NO(PORT_GPH,11)
#define PIN_GPH12	PIN_NO(PORT_GPH,12)
#define PIN_GPH13	PIN_NO(PORT_GPH,13)
#define PIN_GPH14	PIN_NO(PORT_GPH,14)
#define PIN_GPH15	PIN_NO(PORT_GPH,15)

#define PIN_GPJ0	PIN_NO(PORT_GPJ_2440,0)
#define PIN_GPJ1	PIN_NO(PORT_GPJ_2440,1)
#define PIN_GPJ2	PIN_NO(PORT_GPJ_2440,2)
#define PIN_GPJ3	PIN_NO(PORT_GPJ_2440,3)
#define PIN_GPJ4	PIN_NO(PORT_GPJ_2440,4)
#define PIN_GPJ5	PIN_NO(PORT_GPJ_2440,5)
#define PIN_GPJ6	PIN_NO(PORT_GPJ_2440,6)
#define PIN_GPJ7	PIN_NO(PORT_GPJ_2440,7)
#define PIN_GPJ8	PIN_NO(PORT_GPJ_2440,8)
#define PIN_GPJ9	PIN_NO(PORT_GPJ_2440,9)
#define PIN_GPJ10	PIN_NO(PORT_GPJ_2440,10)
#define PIN_GPJ11	PIN_NO(PORT_GPJ_2440,11)
#define PIN_GPJ12	PIN_NO(PORT_GPJ_2440,12)
#define PIN_GPJ13	PIN_NO(PORT_GPJ_2440,13)
#define PIN_GPJ14	PIN_NO(PORT_GPJ_2440,14)
#define PIN_GPJ15	PIN_NO(PORT_GPJ_2440,15)

#define PIN_GPJ0_2413	PIN_NO(PORT_GPJ_2413,0)
#define PIN_GPJ1_2413	PIN_NO(PORT_GPJ_2413,1)
#define PIN_GPJ2_2413	PIN_NO(PORT_GPJ_2413,2)
#define PIN_GPJ3_2413	PIN_NO(PORT_GPJ_2413,3)
#define PIN_GPJ4_2413	PIN_NO(PORT_GPJ_2413,4)
#define PIN_GPJ5_2413	PIN_NO(PORT_GPJ_2413,5)
#define PIN_GPJ6_2413	PIN_NO(PORT_GPJ_2413,6)
#define PIN_GPJ7_2413	PIN_NO(PORT_GPJ_2413,7)
#define PIN_GPJ8_2413	PIN_NO(PORT_GPJ_2413,8)
#define PIN_GPJ9_2413	PIN_NO(PORT_GPJ_2413,9)
#define PIN_GPJ10_2413	PIN_NO(PORT_GPJ_2413,10)
#define PIN_GPJ11_2413	PIN_NO(PORT_GPJ_2413,11)
#define PIN_GPJ12_2413	PIN_NO(PORT_GPJ_2413,12)
#define PIN_GPJ13_2413	PIN_NO(PORT_GPJ_2413,13)
#define PIN_GPJ14_2413	PIN_NO(PORT_GPJ_2413,14)
#define PIN_GPJ15_2413	PIN_NO(PORT_GPJ_2413,15)

#define PIN_GPK0	PIN_NO(PORT_GPK,0)
#define PIN_GPK1	PIN_NO(PORT_GPK,1)
#define PIN_GPK2	PIN_NO(PORT_GPK,2)
#define PIN_GPK3	PIN_NO(PORT_GPK,3)
#define PIN_GPK4	PIN_NO(PORT_GPK,4)
#define PIN_GPK5	PIN_NO(PORT_GPK,5)
#define PIN_GPK6	PIN_NO(PORT_GPK,6)
#define PIN_GPK7	PIN_NO(PORT_GPK,7)
#define PIN_GPK8	PIN_NO(PORT_GPK,8)
#define PIN_GPK9	PIN_NO(PORT_GPK,9)
#define PIN_GPK10	PIN_NO(PORT_GPK,10)
#define PIN_GPK11	PIN_NO(PORT_GPK,11)
#define PIN_GPK12	PIN_NO(PORT_GPK,12)
#define PIN_GPK13	PIN_NO(PORT_GPK,13)
#define PIN_GPK14	PIN_NO(PORT_GPK,14)
#define PIN_GPK15	PIN_NO(PORT_GPK,15)

#define PIN_GPL0	PIN_NO(PORT_GPL,0)
#define PIN_GPL1	PIN_NO(PORT_GPL,1)
#define PIN_GPL2	PIN_NO(PORT_GPL,2)
#define PIN_GPL3	PIN_NO(PORT_GPL,3)
#define PIN_GPL4	PIN_NO(PORT_GPL,4)
#define PIN_GPL5	PIN_NO(PORT_GPL,5)
#define PIN_GPL6	PIN_NO(PORT_GPL,6)
#define PIN_GPL7	PIN_NO(PORT_GPL,7)
#define PIN_GPL8	PIN_NO(PORT_GPL,8)
#define PIN_GPL9	PIN_NO(PORT_GPL,9)
#define PIN_GPL10	PIN_NO(PORT_GPL,10)
#define PIN_GPL11	PIN_NO(PORT_GPL,11)
#define PIN_GPL12	PIN_NO(PORT_GPL,12)
#define PIN_GPL13	PIN_NO(PORT_GPL,13)
#define PIN_GPL14	PIN_NO(PORT_GPL,14)
#define PIN_GPL15	PIN_NO(PORT_GPL,15)

#define PIN_GPM0	PIN_NO(PORT_GPM,0)
#define PIN_GPM1	PIN_NO(PORT_GPM,1)

#define PIN_GPIIC0	PIN_NO(PORT_GPIIC,0)
#define PIN_GPIIC1	PIN_NO(PORT_GPIIC,1)
#define PIN_GPIIC2	PIN_NO(PORT_GPIIC,2)
#define PIN_GPIIC3	PIN_NO(PORT_GPIIC,3)
#define PIN_GPIIC4	PIN_NO(PORT_GPIIC,4)
#define PIN_GPIIC5	PIN_NO(PORT_GPIIC,5)
#define PIN_GPIIC6	PIN_NO(PORT_GPIIC,6)
#define PIN_GPIIC7	PIN_NO(PORT_GPIIC,7)
#define PIN_GPIIC8	PIN_NO(PORT_GPIIC,8)
#define PIN_GPIIC9	PIN_NO(PORT_GPIIC,9)
#define PIN_GPIIC10	PIN_NO(PORT_GPIIC,10)
#define PIN_GPIIC11	PIN_NO(PORT_GPIIC,11)
#define PIN_GPIIC12	PIN_NO(PORT_GPIIC,12)
#define PIN_GPIIC13	PIN_NO(PORT_GPIIC,13)
#define PIN_GPIIC14	PIN_NO(PORT_GPIIC,14)
#define PIN_GPIIC15	PIN_NO(PORT_GPIIC,15)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARCH_ARM_MACH_S3C2410_TOMTOMGO_IOPINS_H */

/* EOF */
