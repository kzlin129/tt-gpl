/*
 * Dock driver for TomTom active docks
 *
 *
 *  Copyright (C) 2008 by TomTom International BV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 *
 * Description of dock driver
 * ===========================================================================
 * There are two abstractions in this driver. A dock detection id is an int
 * representing what was read from the dock GPIO pins to identify the dock.
 * This is then converted into a dock_t, which is an index into the global
 * list of dock types. The dock detection id -> dock_t mapping is platform
 * dependent, and therefore carried out by the platform code in
 *    arch/arm/plat-tomtom/tlv2platform/dock.c.
 * The resolved dock_t should *not* be platform dependent.
 *
 * The dock driver is split into two parts: The generic dock identification
 * part (drivers/tomtom/dock/dock-core.c) and what is referred to as
 * the "dock function driver" - the code relating to a specific dock (e.g.
 * windscreen dock with RDS TMC receiver, DOCK_WINDSCR_RDS - see
 *    drivers/tomtom/dock/dock_windscr_rds.c).
 * The dock function driver plugs into the generic dock driver and does any
 * dock-specific set up such as adding the correct platform drivers.
 * Therefore, the definition of what devices are present in a particular dock
 * is encoded in the dock function driver itself, and *not* in platform data,
 * or anywhere else.
 *
 * Dock driver userland interface
 * ============================================================================
 * There are two userland interfaces for the dock driver: one is the main
 * interface for an application to discover which dock is connected (if a dock
 * is at all connected) and the other is a debug / test interface, which is
 * enabled by the DOCK_DEBUG define below. These interfaces are present in the
 * sysfs directory of the dock driver (e.g. /sys/devices/platform/dock).
 *
 * Main userland interface
 * -----------------------
 * Attribute: dock_state (read-only, string)
 *  The value of this shows which dock is connected. This takes the string
 *  value of one of the enumerations defined by enum docks (see below), e.g.
 *  DOCK_SPARE1, DOCK_NONE etc. The value "DOCK_NONE" is returned if no dock
 *  is connected. If an unrecognized dock is connected, the value
 *  "DOCK_INVALID" is returned.
 * Uevent: KOBJ_CHANGE
 *  A uevent of type KOBJ_CHANGE is emitted when a dock is plugged / unplugged
 *  (also on removal of the module and suspend / resume). This uevent points
 *  to the dock device kobject.
 *
 * Debug / test userland interface
 * -------------------------------
 * This interface is enabled by defining the macro DOCK_DEBUG below
 *
 * Attribute: force_dock (write-only, hexadecimal integer)
 *  Write a hexadecmial integer representing the dock detection id (*not*
 *  dock_t representation) to force the driver to think that a particular dock
 *  has been inserted. e.g. echo 0xe > force_dock
 *
 * Attribute: gpio_* (read-only, decimal integer)
 *  Read the state of a gpio pin used by the dock.
 *
 */

#ifndef __ASM_PLAT_TOMTOM_DOCK_H
#define __ASM_PLAT_TOMTOM_DOCK_H

#include <linux/platform_device.h>
#include <linux/kobject.h>



/***** Enable this for debug / test userland interface & debug messages  *****/
#define DOCK_DEBUG



/* Dock types */
typedef uint dock_t;
/* Enumeration of all the values a dock_t is allowed to have */
enum docks {
	DOCK_INVALID = 0,
	DOCK_NONE,
	DOCK_WINDSCR_NO_RDS,
	DOCK_WINDSCR_RDS,
	DOCK_SPARE1,
	DOCK_SPARE2,
	DOCK_EXTENDED,
	NUM_DOCKS, /* Number of docks defined */
};

typedef int (*dock_function_connect)(void **);
typedef int (*dock_function_disconnect)(void **);
typedef int (*dock_function_reset_devices)(void *);
/* Operations that we want to perform on a dock function driver */
struct dock_operations {
	dock_function_connect connect;
	dock_function_disconnect disconnect;
	dock_function_reset_devices reset_devices;
};

struct dock_desc {
	char *name; /* This string is exported to userland */
	struct dock_operations d_ops;
	void *data; /* Data for use by the dock function driver */
};
#define MK_DOCK(ID, p_connect, p_disconnect, p_reset_devices) \
	{ \
		.name = # ID, \
		.d_ops = { p_connect, p_disconnect, p_reset_devices}, \
		.data = NULL, \
	}
#define MK_BORING_DOCK(ID) MK_DOCK(ID, NULL, NULL, NULL) /* yawn */

/* Global list of docks. The dock_t is an index into this table */
extern struct dock_desc dock_info[];

static inline struct dock_desc *get_dock_info(dock_t dock)
{
	if (dock > 0 && dock < (dock_t) NUM_DOCKS)
		return &dock_info[dock];
	else
		return &dock_info[0];
}

/* Map (potentially) device-specific detection masks to dock types. Each board
 * should have a known list of these mappings defined
 */
struct dock_map_el {
	int detid; /* Value read out by GPIO on dock */
	dock_t dock; /* DOCK_ ... ID */
};

/* Status of detection ID pins with dock power disabled (bit 3/2) */
#define DETID_NOPWR(b1, b0) (((b1) << 3) | ((b0) << 2))
/* Status of detection ID pins with dock power enabled (bit 1/0) */
#define DETID_PWR(b1, b0) (((b1) << 1) | ((b0) << 0))

/* Platform data for generic dock driver */
struct dock_platform_data {
	uint32_t	i2cbusnumber;	/* the I2C bus number */
	void (*gpio_reset)(int);
	void (*gpio_pwr_dock)(int);
	void (*gpio_set_pwr_det)(int);
	int (*gpio_get_pwr_det)(void);
	int (*gpio_sense0)(void);
	int (*gpio_sense1)(void);

	/* Callbacks to handle the gpio configuration */
	void (*config_gpio)(void);
	int (*request_gpio)(void);
	void (*free_gpio)(void);

	/* Platform specific stuff (namely: pin setup) to do on ... */
	void (*suspend)(void);	/* ... suspend */
	void (*resume)(void);	/* ... resume */
	dock_t (*dock_lookup)(int);	/* Machine specific way of deciding
	                                which dock has been plugged */
};

enum dock_irqs {
	DOCK_IRQ_SENSE0 = 0,
	DOCK_IRQ_SENSE1,
	DOCK_IRQ_I2C,
	NUM_DOCK_IRQS,
};

/* Interface between generic dock driver and dock function driver to get I2C
 * interrupt number of dock
 */
int dock_get_i2c_irq(void);

/* same, but then for index of dock i2c adapter driver */
int dock_get_i2c_adapter_index(void);

#endif /* __ASM_PLAT_TOMTOM_DOCK_H */

