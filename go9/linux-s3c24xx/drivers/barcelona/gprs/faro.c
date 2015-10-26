/* drivers/barcelona/gprs/faro.c
 *
 * GPRS driver for Foxlink FARO
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Jaap-Jan Boor <jaap-jan.boor@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include "gprs.h"

int faro_detect(struct gprs_modem *device)
{
	int rc;

	rc = ((IO_GetModelId() == GOTYPE_CAGLIARI)  || 
              (IO_GetModelId() == GOTYPE_FLORENCE)  ||
              (IO_GetModelId() == GOTYPE_TREVISO)  ||
              (IO_GetModelId() == GOTYPE_DURBAN))   && 
              (IO_HaveGprsModem());
	if (rc)
		device->stat = GPRS_STATUS_OFF;

	return rc;
}

#ifdef CONFIG_PM
int faro_suspend(struct gprs_modem *device)
{
	return 0;
}

int faro_resume(struct gprs_modem *device)
{
	return 0;
}
#endif

int faro_reset(struct gprs_modem *device)
{
	if (device->stat == GPRS_STATUS_ON) {
		IO_Activate(GSM_SYS_EN);
		IO_Deactivate(GSM_SYS_RST);
		msleep(1);
		IO_Activate(GSM_SYS_RST);
		msleep(100);
		IO_Deactivate(GSM_SYS_RST);
		msleep(1000);
		IO_Deactivate(GSM_SYS_EN);
		return 0;
	} else
		return -1;
}

int faro_off(struct gprs_modem *device)
{
	if (device->stat == GPRS_STATUS_ON) {
		if (IO_GetModelId() != GOTYPE_CAGLIARI) {
			IO_Deactivate(GSM_SYS_EN);
			IO_Deactivate(GSM_SYS_RST);
			msleep(1);
			IO_Activate(GSM_SYS_RST);
			msleep(100);
			IO_Deactivate(GSM_SYS_RST);
		}
		device->stat = GPRS_STATUS_OFF;
		return 0;
	} else
		return -1;
}

int faro_on(struct gprs_modem *device)
{
	/* Set device->stat=ON first, otherwise /sys/.../power_on will return 0 until
	   this function completes. If we take a long time to return, kernel will
	   report modem as off when we're in the process of turning on.
	*/
  device->stat = GPRS_STATUS_ON;

	/* start with a "power off reset", otherwise Faro 1.1 and 2.0 might
	   start outputting debug information. */
	IO_Deactivate(GSM_SYS_EN);
	IO_Deactivate(GSM_SYS_RST);
	msleep(1);
	IO_Activate(GSM_SYS_RST);
	msleep(100);
	IO_Deactivate(GSM_SYS_RST);
	msleep(2000);

	/* power on sequence */
	IO_Activate(GSM_SYS_EN);
	msleep(2000);
	IO_Deactivate(GSM_SYS_EN);
	msleep(1200);

	IO_SetFunction(GSM_RXD);
	IO_SetFunction(GSM_TXD);
	IO_SetFunction(GSM_RTS);
	IO_SetFunction(GSM_CTS);

	return 0;
}
