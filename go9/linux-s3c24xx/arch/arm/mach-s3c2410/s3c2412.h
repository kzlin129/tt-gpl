/* arch/arm/mach-s3c2410/s3c2412.h
 *
 * Copyright (C) 2006 TomTom BV. All rights reserved.
 * Author 0: Koen Martens <kmartens@sonologic.nl>
 * Author 1: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * Header file for S3C2412 CPU support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *	27-Feb-2006 KM   Start of S3C2412 CPU support
 */

#ifdef CONFIG_CPU_S3C2412

extern  int s3c2412_init(void);
extern void s3c2412_map_io(struct map_desc *mach_desc, int size);
extern void s3c2412_init_uarts(struct s3c2410_uartcfg *cfg, int no);
extern void s3c2412_init_clocks(int xtal);

#else /* CONFIG_CPU_S3C2412 */

#define s3c2412_init_clocks NULL
#define s3c2412_init_uarts NULL
#define s3c2412_map_io NULL
#define s3c2412_init NULL

#endif /* CONFIG_CPU_S3C2412 */

/* EOF */
