/*
 * Configuration for Cordoba
 *
 * Copyright 2009 TomTom B.V.
 *      Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _PLAT_CORDOBA_H
#define _PLAT_CORDOBA_H

/* Temporary stuff, to be moved to each user... */
#include <plat/mendoza.h>

/* GPIO expander */
#define CORDOBA_GPIO_EXPANDER_BASE	140
#define CORDOBA_GPIO_EXPANDER(n)	(CORDOBA_GPIO_EXPANDER_BASE + (n))

#define CORDOBA_GPIO_EXP_IO0(x)		CORDOBA_GPIO_EXPANDER((x))
#define CORDOBA_GPIO_EXP_IO1(x)		CORDOBA_GPIO_EXPANDER(8 + (x))

void cordoba_spi_setup(void);
int  tomtom_bl_setup(void);

#endif 
