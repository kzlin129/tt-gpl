/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <plat/gps.h>
#include <plat/tt_setup_handler.h>

#include <asm/io.h>

#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>


#define BCM4760_DEVNAME "bcm4760"
#define PFX	BCM4760_DEVNAME ": "

#define GPS_UNLOCK_SIGNATURE 0xBCBC4760

static int firsttime = 1;

static void bcm4760_set_power(int power)
{
	volatile unsigned long val = 0;

	if (power) {
		printk(KERN_INFO "bcm4760 gps switched ON\n");
		
		val = readl(IO_ADDRESS(PML_R_PML_SW_ISOLATE_MEMADDR));
		writel (val | PML_F_GPS_SW_STANDBY_N_3P3_MASK, IO_ADDRESS(PML_R_PML_SW_ISOLATE_MEMADDR));

		mdelay(80);
	} else {
		printk(KERN_INFO "bcm4760 gps switched OFF\n");

		val = readl(IO_ADDRESS(CMU_R_BUS_CLK_STOP_MEMADDR));
		writel ( val | CMU_F_CMU_BM5_PCLK_GTEN_MASK, IO_ADDRESS(CMU_R_BUS_CLK_STOP_MEMADDR));
        
		val = readl(IO_ADDRESS(PML_R_PML_SW_ISOLATE_MEMADDR));
		writel (val & ~PML_F_GPS_SW_STANDBY_N_3P3_MASK, IO_ADDRESS(PML_R_PML_SW_ISOLATE_MEMADDR));

		udelay(156);
	}
}

void bcm4760_reset(void)
{
	volatile unsigned long val = 0;

	printk(KERN_INFO "bcm4760 gps reset\n");

	writel (GPS_UNLOCK_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

	val = readl (IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
	writel(val & ~CMU_F_CMU_GPS_RESETN_MASK, IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
    
	mdelay(50);

	/* to the time when nSTANDBY is deasserted = 1/6 ms */
	val = readl(IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
	writel (val | CMU_F_CMU_GPS_RESETN_MASK, IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
	writel (0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
}	

static void bcm4760_init (void)
{
	volatile unsigned long val = 0;

	if (firsttime) {
		bcm4760_reset();
		firsttime = 0;
	} else {
		writel (GPS_UNLOCK_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
		val = readl (IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
		writel (val | CMU_F_CMU_GPS_RESETN_MASK, IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
		writel (0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
	}

	udelay (156);
	bcm4760_set_power (1);
	mdelay (80);

	bcm4760_set_power (0);
}

static struct generic_gps_info machinfo =
{
	.name		= BCM4760_DEVNAME,
	.gps_set_power	= bcm4760_set_power,
	.gps_reset	= bcm4760_reset,
	.gps_init	= bcm4760_init
};

static void bcm4760_dev_release(struct device *dev){}

static struct platform_device bcm4760_pdev =
{
	.name		= "tomtom-gps",
	.id		= -1,
	.dev = {
		.release	= bcm4760_dev_release,
	},
};

static __init int bcm4760_register(void)
{
	bcm4760_pdev.dev.platform_data = (void *) &machinfo;
	return platform_device_register(&bcm4760_pdev);
};


arch_initcall(bcm4760_register);
//TT_SETUP_CB(tt_setup_cb , "tomtom-gps-gps_dev");
