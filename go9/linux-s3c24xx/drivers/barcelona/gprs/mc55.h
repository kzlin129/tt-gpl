/* drivers/barcelona/gprs/mc55.h
 *
 * GPRS driver for Siemens MC55
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_BARCELONA_GPRS_MC55_H
#define __DRIVERS_BARCELONA_GPRS_MC55_H		__FILE__

#include "gprs.h"
#define GPRS_MODEL_MC55       "mc55"

extern int siemens_mc55_detect( struct gprs_modem *device );
extern int siemens_mc55_reset( struct gprs_modem *device );
extern int siemens_mc55_off( struct gprs_modem *device );
extern int siemens_mc55_on( struct gprs_modem *device );
#ifdef CONFIG_PM
extern int siemens_mc55_suspend( struct gprs_modem *device );
extern int siemens_mc55_resume( struct gprs_modem *device );
#else
#define siemens_mc55_suspend    NULL
#define siemens_mc55_resume     NULL
#endif
#endif /* __DRIVERS_BARCELONA_GPRS_MC55_H */
