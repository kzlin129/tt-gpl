/*
 * Synaptics embedded Touchpad driver for TomTom GO devices.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 *
 */

#ifndef DRIVERS_CHAR_MEP_MEP_BUS_H
#define DRIVERS_CHAR_MEP_MEP_BUS_H

#include <linux/types.h>
#include <linux/spinlock.h>

#include "mep_lib.h"

#define MEP_BUS_MAJOR		246
#define MEP_MAX_DEVICES		8
//#define MEP_MAX_PKTS		10			/* # of packets that we 'cache' per device */

/* Definition for the 'bus' device */
typedef struct _mep_bus {
	struct device*		dev;				/* 'bus' device */
	struct resource*	irq;				/* IRQ resource assigned to bus */
	spinlock_t		lock;				/* Lock protecting access to bus */
	struct _mep_dev*	devices[MEP_MAX_DEVICES];	/* Devices on chain */
} mep_bus_t;

/* Definition of current device state */
typedef struct _mep_state {
	short		x;
	short		y;
	unsigned char	w;
	unsigned char	z;
	unsigned char	buttons;
} mep_state;

/* Definition for a specific device */
typedef struct _mep_dev {
	mep_bus_t*		bus;			/* Bus this device is on */
	int			id;			/* ID of device */
	spinlock_t		lock;			/* lock for whole device */
	wait_queue_head_t	inq;			/* Wait queue for reading */

	mep_state		state;			/* Current state of the mep */
	unsigned char		changed;		/* Has the state changed since the last read? */
	unsigned char		curr_buttons;		/* Last seen buttonstate */
} mep_dev_t;

#endif /* DRIVERS_CHAR_MEP_MEP_BUS_H */

