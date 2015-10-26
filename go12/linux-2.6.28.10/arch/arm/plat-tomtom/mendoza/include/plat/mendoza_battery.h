/* arch/arm/mach-s3c6410/battery.h
 * 
 * Copyright (c) 2009 TomTom BV <http://www.tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.

 */

#ifndef ARCH_INCLUDE_LINUX_BATTERY_H
#define ARCH_INCLUDE_LINUX_BATTERY_H

#include <plat/adc.h>
#include <linux/semaphore.h>
#include <linux/completion.h>

struct battery_poll
{
        struct s3c_adc_client	*adc_client;
	unsigned int 		 channel;	
	unsigned int 		 samples;
	struct semaphore	 mutex;
	struct completion	 adc_done;
        u32			 current_value;
	unsigned long		 last_read;

};

void mendoza_battery_adc_setup(unsigned int voltage_channel, unsigned int current_channel);

#endif /* ARCH_INCLUDE_LINUX_BATTERY_H */
