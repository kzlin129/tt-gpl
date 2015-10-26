/* drivers/barcelona/gprs/adi6720.h
 *
 * GPRS driver for ADI6720
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_BARCELONA_GPRS_ADI6720_H
#define __DRIVERS_BARCELONA_GPRS_ADI6720_H	__FILE__

#include "gprs.h"
#define GPRS_MODEL_ADI6720	"adi6720"

extern int adi6720_detect( struct gprs_modem *device );
extern int adi6720_reset( struct gprs_modem *device );
extern int adi6720_off( struct gprs_modem *device );
extern int adi6720_on( struct gprs_modem *device );
#ifdef CONFIG_PM
extern int adi6720_suspend( struct gprs_modem *device );
extern int adi6720_resume( struct gprs_modem *device );
#else
#define adi6720_suspend         NULL
#define adi6720_resume          NULL
#endif
#endif /* __DRIVERS_BARCELONA_GPRS_ADI6720_H */
