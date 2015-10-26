/* drivers/barcelona/gprs/gprs.h
 *
 * GPRS driver for Murcia/Knock
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_BARCELONA_GPRS_GPRS_H
#define __DRIVERS_BARCELONA_GPRS_GPRS_H      __FILE__

#include <linux/kobject.h>
#include <linux/string.h>
#define MODPFX  "GPRS: "

enum gprs_status
{
	GPRS_STATUS_OFF=0,
	GPRS_STATUS_ON,
	GPRS_STATUS_SUSPEND,
};

struct gprs_modem
{
	char			*name;
	enum gprs_status	stat;

	int			(*detect)( struct gprs_modem *device );
	int			(*reset)( struct gprs_modem *device );
	int			(*off)( struct gprs_modem *device );
	int			(*on)( struct gprs_modem *device ); 
	int			(*status)( struct gprs_modem *device );
#ifdef CONFIG_PM
	int			(*suspend)( struct gprs_modem *device );
	int			(*resume)( struct gprs_modem *device );
#endif
};

extern struct gprs_modem	gprs_modem_list[];
extern struct device_attribute	gprs_attributes[];

#define GPRS_MODEM_ENTRIES      (sizeof( gprs_modem_list )/sizeof( gprs_modem_list[0] ))
#endif /* __DRIVERS_BARCELONA_GPRS_GPRS_H */
