/* drivers/pmu/bcm59040_usbstat.h

Copyright (C) 2010 TomTom BV <http://www.tomtom.com/>
Authors: Rogier Stam <rogier.stam@tomtom.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

*/

#ifndef __DRIVERS_PMU_BCM59040_USBSTAT_H__
#define __DRIVERS_PMU_BCM59040_USBSTAT_H__
#define DRIVER_DESC_LONG "TomTom BCM59040 USBSTAT Emulation driver, (C) 2010 TomTom BV "
#define PFX "BCM59040_USBSTAT: "

struct bcm59040_usbstat_pdata
{
	uint32_t __iomem bcm4760_cmu_base;
	uint32_t __iomem bcm4760_usb_base;
	int		 bcm59040_usb_irq;
};
#endif /* __DRIVERS_PMU_BCM59040_USBSTAT_H__ */
