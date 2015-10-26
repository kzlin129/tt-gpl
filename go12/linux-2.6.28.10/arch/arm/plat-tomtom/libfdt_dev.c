/*
 * linux/arch/arm/mach-s3c6410/libfdt_dev.c
 *
 * libfdt platform device setup/initialization for TomTom platforms.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <plat/factorydata.h>
extern struct factorydata_buffer_info_t fdt_buffer_info;

struct resource tomtom_libfdt_resource[] ={
	{
		.name = "dtb-buffer",
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end = 0,
	},
};

struct platform_device tomtom_device_libfdt = {
	.name		= "libfdt",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tomtom_libfdt_resource),
	.resource	= tomtom_libfdt_resource,
};	
