/*
 * linux/arch/arm/mach-bcm4760/fdtexport_setup.c
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

extern struct platform_device tomtom_device_fdtexport; /* defined in plat-tomtom for reuse */

static int __init tomtom_fdtexport_init(void)
{
	tomtom_device_fdtexport.resource[0].start = fdt_buffer_info.address;
	tomtom_device_fdtexport.resource[0].end = fdt_buffer_info.address + fdt_buffer_info.size;

	platform_device_register(&tomtom_device_fdtexport);
	return 0;
}

arch_initcall(tomtom_fdtexport_init);
