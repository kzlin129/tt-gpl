/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author: Vincent Dejouy <vincent.dejouy@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
 
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <plat/devs.h>
#include <plat/tt_setup_handler.h>


static int __init tt_setup_cb( char * identifier )
{

	platform_device_register( &s3c_device_ts );
	return 0;
}

TT_SETUP_CB(tt_setup_cb ,"s3c-ts");

