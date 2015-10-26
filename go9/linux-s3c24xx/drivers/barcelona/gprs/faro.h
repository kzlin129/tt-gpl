/* drivers/barcelona/gprs/faro.h
 *
 * GPRS driver for Foxlink FARO
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_BARCELONA_GPRS_FARO_H
#define __DRIVERS_BARCELONA_GPRS_FARO_H	__FILE__

#include "gprs.h"
#define GPRS_MODEL_FARO	"faro"

extern int faro_detect( struct gprs_modem *device );
extern int faro_reset( struct gprs_modem *device );
extern int faro_off( struct gprs_modem *device );
extern int faro_on( struct gprs_modem *device );
#ifdef CONFIG_PM
extern int faro_suspend( struct gprs_modem *device );
extern int faro_resume( struct gprs_modem *device );
#else
#define faro_suspend         NULL
#define faro_resume          NULL
#endif
#endif /* __DRIVERS_BARCELONA_GPRS_FARO_H */
