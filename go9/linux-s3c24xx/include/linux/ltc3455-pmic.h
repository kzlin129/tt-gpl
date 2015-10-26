/* ltc3455-pmic.h
 *
 * Control driver for LTC3455 PMIC.
 *
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_LTC3455_PMIC_H
#define INCLUDE_LINUX_LTC3455_PMIC_H
struct ltc3455_volt
{
	uint32_t	frequency;	/* Frequency in KHz. */
	uint32_t	voltage;	/* Voltage in milivolt. */
};

enum ltc3455_cmode
{
	LTC3455_PWM=0,
	LTC3455_BURST
};

struct ltc3455_pdata
{
	gopin_t				acpwr_pin;
	gopin_t				wall_pwr_pin;
	gopin_t				low_core_pin;
	gopin_t				pwr_mode_pin;

	struct ltc3455_volt		*power_setting;
	struct
	{
		uint32_t		high;
		uint32_t		low;
	} voltage;

	struct
	{
		enum ltc3455_cmode	suspend;
		enum ltc3455_cmode	batt;
		enum ltc3455_cmode	acadapt;
	} cmode;
};

#define LTC3455_DEVNAME			"ltc3455_pmic"
#endif
