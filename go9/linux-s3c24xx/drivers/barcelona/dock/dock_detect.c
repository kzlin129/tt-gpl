/* drivers/barcelona/dock/dock_detect.c
 *
 * Dock detect driver library
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Laurent Gregoire <laurent.gregoire@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <barcelona/gopins.h>

#include <barcelona/dock.h>
#include "dock_detect.h"

#define PFX	"dockdetect: "

static spinlock_t state_handler_lock = SPIN_LOCK_UNLOCKED;
static dock_state_handler state_handler = NULL;

void dock_set_dock_state_handler(dock_state_handler hnd)
{
	unsigned long flags;
	int failed = 0;
	spin_lock_irqsave(&state_handler_lock, flags);
	if (state_handler)
	{	failed = 1;
	} else
	{	state_handler = hnd;
	}
	spin_unlock_irqrestore(&state_handler_lock, flags);
	if (failed) {
		printk(KERN_ERR PFX
			"Attempting to set two dock state handler!");
	}
}

int DOCK_GetDockState(void)
{
	if (state_handler)
	{	return state_handler();
	}
	return -1;
}
EXPORT_SYMBOL(DOCK_GetDockState);

