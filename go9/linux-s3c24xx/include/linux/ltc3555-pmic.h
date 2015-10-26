/* ltc3555-pmic.h
 *
 * Control driver for LTC3555 PMIC.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_LTC3555_PMIC_H
#define INCLUDE_LINUX_LTC3555_PMIC_H
struct ltc3555_volt
{
	uint32_t	frequency;	/* Frequency in KHz. */
	uint32_t	voltage;	/* Voltage in milivolt. */
};

struct ltc3555_swreg
{
	/* R1 and R2 detail the register divider */
	/* settings for the feedback pins. */
	uint32_t		R1;
	uint32_t		R2;

	/* This array at which frequency the */
	/* voltage should be changed. Note:  */
	/* The higher voltage will be chosen */
	/* When the frequency is below the   */
	/* one in this array. Terminate by an*/
	/* entry with all fields set to 0.   */
	/* Voltages should be in ascending   */
	/* order. */
	struct ltc3555_volt	*power_setting;
};

enum ltc3555_swmode
{
	LTC3555_PULSE_SKIP=0,
	LTC3555_FORCE_BURST,
	LTC3555_LDO,
	LTC3555_BURST
};

struct ltc3555_platform
{
	/* Switching Regulator settings. */
	struct ltc3555_swreg	sw2;
	struct ltc3555_swreg	sw3;

	/* Pin definition. acpwr_pin shows current AC power */
	/* status, wall_pwr_pin can be used to set higher   */
	/* charging current when CLA is used. */
	gopin_t			acpwr_pin;
	gopin_t			wall_pwr_pin;

	/* Mode settings depending on which mode we run in. */
	/* For example: While running in suspend it might be*/
	/* wiser to put the regulator in a different, more  */
	/* efficient mode. The acpwr_pin denotes which pin  */
	/* can be polled for the current AC power status.   */
	struct
	{
		enum ltc3555_swmode	suspend;
		enum ltc3555_swmode	batt;
		enum ltc3555_swmode	acadapt;
	} swmode;
};

#define LTC3555_DEVNAME			"ltc3555_pmic"
#endif
