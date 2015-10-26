/* arch/arm/mach-s3c2410/tomtomgo-clocks.h
 *
 * * Additional clock type definitions
 *
 * Copyright (C) 2004,2005, 2006 TomTom BV <http://www.tomtom.com/>
 * Authors:  Mark Vels <Mark.Vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#ifndef __ARCH_ARM_MACH_TOMTOMGO_CLOCKS_H__
#define __ARCH_ARM_MACH_TOMTOMGO_CLOCKS_H__

/**
 * Setup additional clock devices that might be present on this TTGO 
 * hardware type 
 */  
extern int tomtomgo_init_clocks(void);

#endif //__ARCH_ARM_MACH_TOMTOMGO_CLOCKS_H__
