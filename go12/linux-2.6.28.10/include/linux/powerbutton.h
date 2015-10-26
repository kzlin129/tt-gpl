/*
 * Include file for the power button device.
 *
 * Author: Mark Vels <mark.vels@tomtom.com>
 *  * (C) Copyright 2008 TomTom International BV. 
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __LINUX_POWERBUTTON_H__
#define __LINUX_POWERBUTTON_H__

struct powerbutton_pdata {	
	int (*get_value)(void);
	int (*get_timed_value)(void);
	int resume_state;
};

#endif /* __LINUX_POWERBUTTON_H__ */
