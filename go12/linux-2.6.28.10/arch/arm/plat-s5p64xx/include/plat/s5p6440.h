/* arch/arm/plat-s5p64xx/include/plat/s5p6440.h
 *
 * Copyright 2008 Openmoko,  Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * Header file for s5p6440 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifdef CONFIG_CPU_S5P6440

extern void s5p6440_common_init_uarts(struct s3c2410_uartcfg *cfg, int no);
extern void s5p6440_register_clocks(void);
extern void s5p6440_setup_clocks(void);

extern  int s5p6440_init(void);
extern void s5p6440_init_irq(void);
extern void s5p6440_map_io(void);
extern void s5p6440_init_clocks(int xtal);

#define s5p6440_init_uarts s5p6440_common_init_uarts

#else
#define s5p6440_init_clocks NULL
#define s5p6440_init_uarts NULL
#define s5p6440_map_io NULL
#define s5p6440_init NULL
#endif
