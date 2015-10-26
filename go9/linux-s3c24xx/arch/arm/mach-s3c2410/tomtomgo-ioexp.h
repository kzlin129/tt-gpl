/* arch/arm/mach-s3c2410/tomtomgo-ioexp.h
 *
 * Implementation of the IO expander driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_S3C2410_TOMTOMGO_IOEXP_H
#define __ARCH_ARM_MACH_S3C2410_TOMTOMGO_IOEXP_H

#ifdef __KERNEL__
#include "tomtomgo-iopins.h"
#endif

#ifdef __BOOTLOADER__
#include "tomtomgo-iopins.h"
#endif

unsigned int IOEXP_InputOutput(unsigned reqOutputValue);

#endif
