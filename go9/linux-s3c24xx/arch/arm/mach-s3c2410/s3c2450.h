/* arch/arm/mach-s3c2410/s3c2450.h
 *
 * Copyright (C) 2009 TomTom BV. All rights reserved.
 * Author 0: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author 1: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 * Header file for S3C2450 CPU support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *	25-Oct-2006 DA   Start of S3C2450 CPU support
 */

#ifdef CONFIG_CPU_S3C2450

extern  int s3c2450_init(void);
extern void s3c2450_map_io(struct map_desc *mach_desc, int size);
extern void s3c2450_init_uarts(struct s3c2410_uartcfg *cfg, int no);
extern void s3c2450_init_clocks(int xtal);

#else /* CONFIG_CPU_S3C2450 */

#define s3c2450_init_clocks NULL
#define s3c2450_init_uarts NULL
#define s3c2450_map_io NULL
#define s3c2450_init NULL

#endif /* CONFIG_CPU_S3C2450 */

/* EOF */
