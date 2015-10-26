/* linux/arch/arm/mach-s5p6440/include/mach/gpio.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C6400 - GPIO lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#define gpio_to_irq	__gpio_to_irq

/* GPIO bank sizes */
#define S5P64XX_GPIO_A_NR	(6)
#define S5P64XX_GPIO_B_NR	(7)
#define S5P64XX_GPIO_C_NR	(8)
#define S5P64XX_GPIO_F_NR	(16)
#define S5P64XX_GPIO_G_NR	(7)
#define S5P64XX_GPIO_H_NR	(10)
#define S5P64XX_GPIO_I_NR	(16)
#define S5P64XX_GPIO_J_NR	(12)
#define S5P64XX_GPIO_N_NR	(16)
#define S5P64XX_GPIO_P_NR	(11)
#define S5P64XX_GPIO_R_NR	(15)

/* GPIO bank numbes */

/* CONFIG_S3C_GPIO_SPACE allows the user to select extra
 * space for debugging purposes so that any accidental
 * change from one gpio bank to another can be caught.
*/

#define S5P64XX_GPIO_NEXT(__gpio) \
	((__gpio##_START) + (__gpio##_NR) + CONFIG_S3C_GPIO_SPACE + 1)

enum s3c_gpio_number {
	S5P64XX_GPIO_A_START = 0,
	S5P64XX_GPIO_B_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_A),
	S5P64XX_GPIO_C_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_B),
	S5P64XX_GPIO_F_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_C),
	S5P64XX_GPIO_G_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_F),
	S5P64XX_GPIO_H_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_G),
	S5P64XX_GPIO_I_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_H),
	S5P64XX_GPIO_J_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_I),
	S5P64XX_GPIO_N_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_J),
	S5P64XX_GPIO_P_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_N),
	S5P64XX_GPIO_R_START = S5P64XX_GPIO_NEXT(S5P64XX_GPIO_P),
};

/* S5P64XX GPIO number definitions. */

#define S5P64XX_GPA(_nr)	(S5P64XX_GPIO_A_START + (_nr))
#define S5P64XX_GPB(_nr)	(S5P64XX_GPIO_B_START + (_nr))
#define S5P64XX_GPC(_nr)	(S5P64XX_GPIO_C_START + (_nr))
#define S5P64XX_GPF(_nr)	(S5P64XX_GPIO_F_START + (_nr))
#define S5P64XX_GPG(_nr)	(S5P64XX_GPIO_G_START + (_nr))
#define S5P64XX_GPH(_nr)	(S5P64XX_GPIO_H_START + (_nr))
#define S5P64XX_GPI(_nr)	(S5P64XX_GPIO_I_START + (_nr))
#define S5P64XX_GPJ(_nr)	(S5P64XX_GPIO_J_START + (_nr))
#define S5P64XX_GPN(_nr)	(S5P64XX_GPIO_N_START + (_nr))
#define S5P64XX_GPP(_nr)	(S5P64XX_GPIO_P_START + (_nr))
#define S5P64XX_GPR(_nr)	(S5P64XX_GPIO_R_START + (_nr))

/* the end of the S5P64XX specific gpios */
#define S5P64XX_GPIO_END	(S5P64XX_GPR(S5P64XX_GPIO_R_NR) + 1)
#define S3C_GPIO_END		S5P64XX_GPIO_END

/* define the number of gpios we need to the one after the GPR() range */
/* MAZ: Actually, dont do that. It prevents the use of any kind of external
 * GPIOs, either real (I2C GPIO expander), or virtual... */
#if 0
#define ARCH_NR_GPIOS	(S5P64XX_GPR(S5P64XX_GPIO_R_NR) + 1)
#endif

#include <asm-generic/gpio.h>
