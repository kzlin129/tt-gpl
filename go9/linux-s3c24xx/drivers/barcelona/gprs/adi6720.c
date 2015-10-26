/* drivers/barcelona/gprs/adi6720.c
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

#include <linux/delay.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include "gprs.h"

int adi6720_detect( struct gprs_modem *device )
{
	int	rc;

	rc=(IO_GetModelId( ) == GOTYPE_KNOCK) && (IO_HaveGprsModem( ));
	if( rc )
	{
		device->stat=GPRS_STATUS_OFF;
	}
	return rc;
}

#ifdef CONFIG_PM
int adi6720_suspend( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_ON )
	{
		device->stat=GPRS_STATUS_SUSPEND;
		IO_Activate( GSM_DTR );
	}
	return 0;
}

int adi6720_resume( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_SUSPEND )
	{
		device->stat=GPRS_STATUS_ON;
		IO_Deactivate( GSM_DTR );
	}
	return 0;
}
#endif

int adi6720_reset( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_ON )
	{
		IO_Deactivate( GSM_RESET );
		msleep( 100 );
		IO_Activate( GSM_RESET );
		msleep( 100 );
		IO_Deactivate( GSM_RESET );
		return 0;
	}
	else return -1;
}

int adi6720_off( struct gprs_modem *device )
{
	if( device->stat == GPRS_STATUS_ON )
	{
		IO_Deactivate( GSM_DTR );
		msleep( 100 );

		IO_Deactivate( GSM_ON );
		msleep( 100 );
		device->stat=GPRS_STATUS_OFF;
		return 0;
	}
	else return -1;
}
	
int adi6720_on( struct gprs_modem *device )
{
	int	rc;

	if( device->stat == GPRS_STATUS_OFF )
	{
		IO_SetFunction( GSM_RXD );
		IO_SetFunction( GSM_TXD );
		IO_SetFunction( GSM_RTS );
		IO_SetFunction( GSM_CTS );

		IO_Deactivate( GSM_DL_EN );

		IO_Activate( GSM_DTR );
		msleep( 100 );

		IO_Activate( GSM_ON );

		device->stat=GPRS_STATUS_ON;
		rc=adi6720_reset( device );
		if( rc != 0 ) return rc;

		msleep( 2500 );
		IO_Deactivate( GSM_ON );
		return 0;
	}
	else return -1;
}
