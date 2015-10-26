/*
 * linux/arch/arm/plat-tomtom/fdt_dev.c
 *
 * device-tree export platform device setup/initialization for TomTom platforms.
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

static struct resource tomtom_fdtexport_resource[] ={
	{
		.name = "dtb-buffer",
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end = 0,
	},
};

struct platform_device tomtom_device_fdtexport = {
	.name		= "fdtexport",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tomtom_fdtexport_resource),
	.resource	= tomtom_fdtexport_resource,
};	
