/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
#include <linux/platform_device.h>
#include <linux/broadcom/ts.h>

#include <plat/tt_setup_handler.h>
#include <plat/fdt.h>

static struct resource bcm4760_ts_resources[] = 
{
	{
		.start  = BCM4760_INTR_TSC,
		.end	= BCM4760_INTR_TSC,
		.flags  = IORESOURCE_IRQ,
	}
};

/* Touchscreen configuration variable */
static tsc_control_table default_ts_control_table =
{
	.sample_rate		= 976*1,
	.data_threshold		= 0,
	.debounce		= 512*1,
	.settling		= 2560,
	.data_point_average  	= 8,
	.wire_mode		= TSC_MODE_4WIRE,
	.xres			= 480,
	.yres			= 272,
	.tscMaxX		= 0xf9c0,	// max X touch value
	.tscMaxY		= 0xf150,	// max Y touch value
	.tscMinX		= 0x790,	// min X touch value
	.tscMinY		= 0xbe0,	// min Y touch value
	.ABSxy			= 1,		// Return abs value not LCD	
};

/* Touchscreen configuration variable for LMS606KF01 */
static tsc_control_table lms606kf01_ts_control_table =
{
	.sample_rate		= 976*1,
	.data_threshold		= 0,
	.debounce		= 512*1,
	.settling		= 2560,
	.data_point_average  	= 8,
	.wire_mode		= TSC_MODE_4WIRE,
	.xres			= 480,
	.yres			= 800,
	.tscMaxX		= 0xf150,	// max X touch value
	.tscMaxY		= 0xf9c0,	// max Y touch value
	.tscMinX		= 0xbe0,	// min X touch value
	.tscMinY		= 0x790,	// min Y touch value
	.ABSxy			= 1,		// Return abs value not LCD	
};

static struct platform_device bcm476x_device_ts =
{
	.name		= "bcm4760_ts",
	.id		= 0,
	.num_resources 	= ARRAY_SIZE(bcm4760_ts_resources),
	.resource 	= bcm4760_ts_resources,
	.dev = 
	{
		.platform_data = NULL
	},
};

struct ts_platform
{
	char	*name;
	void	*platform_data;
};

static struct ts_platform	bcm4760_ts_platform []=
{
	{"LMS606KF01",	(void *) &lms606kf01_ts_control_table},
	{NULL,			(void *) &default_ts_control_table},
};

static int __init tt_bcm4760_ts_setup (char *str)
{
	const char	*tft;
	int		index;

	tft = fdt_get_string("/features", "tft", NULL);
	if( tft )
	{
		for( index=0; bcm4760_ts_platform[index].name != NULL; index++ )
		{
			if( !strcmp(tft, bcm4760_ts_platform[index].name) )
				break;
		}
		bcm476x_device_ts.dev.platform_data=bcm4760_ts_platform[index].platform_data;
	}
	else
	{
		bcm476x_device_ts.dev.platform_data=&default_ts_control_table;
	}

	platform_device_register( &bcm476x_device_ts );
}

TT_SETUP_CB(tt_bcm4760_ts_setup, "tomtom-bcm-touchscreen-resistive");
