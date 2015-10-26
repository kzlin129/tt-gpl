/* include/barcelona/Barc_buspower.h
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDE_BARCELONA_BARC_BUSPOWER_H
#define __INCLUDE_BARCELONA_BARC_BUSPOWER_H
#include<linux/notifier.h>

/* NHotifications interface to get notified of chagnes of the buspower 
funtions that are registered here will be called upon buspower status change
the callback function of the notifier_block will be called with as second parameter
the buspower status (1 is on 0 is off) */
int buspower_register_notifier(struct notifier_block *nb);
int buspower_unregister_notifier(struct notifier_block *nb);

/**
 * returns the current status of the buspower where 1 in on and 0 is off
 * if the module is not iniitalized yet a 0 is returned
 **/
int buspower_get_status(void);

/**
 * Forces single power_on or power_off event to be generated.
 * This is called by the usb driver when it enters state idle,
 * because it needs a thread-safe way of detecting the buspower status.
 **/
void buspower_request_state_change_event(void);


#endif /* __INCLUDE_BARCELONA_BARC_BUSPOWER_H */
