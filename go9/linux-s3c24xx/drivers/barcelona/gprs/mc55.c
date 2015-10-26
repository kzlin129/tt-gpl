/* drivers/barcelona/gprs/mc55.c
 *
 * GPRS driver for Siemens MC55 chip 
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include "gprs.h"

int siemens_mc55_detect( struct gprs_modem *device )
{
	int	rc;
	int	model_id=IO_GetModelId( );

	rc=((model_id == GOTYPE_MURCIA500) || (model_id == GOTYPE_MURCIA700)) && (IO_HaveGprsModem( ));
	if( rc )
	{
		device->stat=GPRS_STATUS_OFF;
	}
	return rc;
}

int siemens_mc55_off( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_ON )
	{
		IO_Deactivate( GSM_ON );
		IO_Activate( GSM_OFF );
		if( IO_HasPin( GSM_OFF ) )
		{
			msleep( 4000 );
			IO_Deactivate( GSM_OFF );
		}
		device->stat=GPRS_STATUS_OFF;
		return 0;
	}
	else return -1;
}

int siemens_mc55_on( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_OFF )
	{
		IO_Deactivate( GSM_OFF );
		IO_Activate( GSM_ON );
		if( IO_HasPin( GSM_OFF ) )
		{
			msleep( 2000 );
			IO_Deactivate( GSM_ON );
		}
		device->stat=GPRS_STATUS_ON;
		return 0;
	}
	else return -1;
}

int siemens_mc55_suspend( struct gprs_modem *device )
{
	int	rc;

	if( device->stat == GPRS_STATUS_ON )
	{
		rc=siemens_mc55_off( device );
		if( rc ) return rc;
		device->stat=GPRS_STATUS_SUSPEND;
	}
	return 0;
}

int siemens_mc55_resume( struct gprs_modem *device )
{
	int	rc;

	if( device->stat == GPRS_STATUS_SUSPEND )
	{
		device->stat=GPRS_STATUS_OFF;
		rc=siemens_mc55_on( device );
		if( rc ) return rc;
	}
	return 0;
}

int siemens_mc55_reset( struct gprs_modem *device )
{
	int	rc;

	if( device->stat == GPRS_STATUS_ON )
	{
		rc=siemens_mc55_off( device );
		if( rc ) return rc;
		rc=siemens_mc55_on( device );
		if( rc ) return rc;
		return 0;
	}
	else return -1;
}
