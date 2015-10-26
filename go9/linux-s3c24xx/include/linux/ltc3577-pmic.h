/* ltc3577-pmic.h
 *
 * Control driver for LTC3577 PMIC.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_LTC3577_PMIC_H
#define INCLUDE_LINUX_LTC3577_PMIC_H
struct ltc3577_volt
{
	uint32_t	frequency;	/* Frequency in KHz. */
	uint32_t	voltage;	/* Voltage in milivolt. */
};

struct ltc3577_swreg
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
	struct ltc3577_volt	*power_setting;
};

enum ltc3577_swmode
{
	LTC3577_PWM   = 0,
	LTC3577_BURST = 1
};

enum ltc3577_ledmode
{
	LTC3577_OFF = 0,
	LTC3577_ON  = 1
};

struct ltc3577_platform
{
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
		enum ltc3577_swmode	suspend;
		enum ltc3577_swmode	batt;
		enum ltc3577_swmode	acadapt;
	} swmode;
};

#define LTC3577_READ_SIZE               (1)
#define LTC3577_WRITE_SIZE              (2)
#define LTC3577_SUBADDR			(0)
#define LTC3577_DATA			(1)

typedef struct ltc3577_msg
{
	uint8_t commit;
	uint8_t data[LTC3577_WRITE_SIZE];
} ltc3577_msg_t;

int ltc3577_set_reg( ltc3577_msg_t *write_buf, uint8_t addr, uint8_t data);
int ltc3577_commit( ltc3577_msg_t *write_buf, ltc3577_msg_t *read_buf, uint8_t lock);

#define LTC3577_DEVNAME			"ltc3577_pmic"
#endif
