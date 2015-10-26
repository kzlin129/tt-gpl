/* include/barcelona/Barc_Battery.h
 *
 * Public interface for the battery driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDE_BARCELONA_BARC_BATTERY_H
#define __INCLUDE_BARCELONA_BARC_BATTERY_H

#ifndef __INCLUDE_BARCELONA_TYPES_H
#include <barcelona/types.h>
#endif /* __INCLUDE_BARCELONA_TYPES_H */

#include <linux/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BATTERY_DEVNAME				"battery"
#define BATTERY_MAJOR				241

typedef struct {
	UINT16 u16BatteryVoltage;		/* battery voltage */
	UINT16 u16ChargeCurrent;		/* charge current */
	UINT8 u8ChargeStatus;
} BATTERY_STATUS;

typedef struct {
	UINT16 u16BatteryRaw4000mVADCREF3300mV;
	UINT16 u16BatteryRaw3500mVADCREF3300mV;
} BATTERY_CALIBRATION;

#define CHARGE_STATE_NO_POWER		0	/* No power for charging */
#define CHARGE_STATE_COMPLETED		1	/* Charging completed */
#define CHARGE_STATE_CHARGING		2	/* Charging in progress */

#define VOLTAGE_SOURCE_BATTERY		0		/* Standard battery  */
#define VOLTAGE_SOURCE_BATTERY_PACK	1		/* AA batter pack    */

#define BATTERY_DRIVER_MAGIC	'B' /* Battery driver magic number */
#define IOR_BATTERY_STATUS	_IOR(BATTERY_DRIVER_MAGIC, 3, BATTERY_STATUS)
#define IO_ENABLE_CHARGING	_IO(BATTERY_DRIVER_MAGIC, 4)
#define IO_DISABLE_CHARGING	_IO(BATTERY_DRIVER_MAGIC, 5)
#define IOW_SET_CALIBRATION	_IOW(BATTERY_DRIVER_MAGIC, 6, BATTERY_CALIBRATION)

#define BASIC_BATTERY_ADC_CALIBRATION	1
#define BATT_ADC_CAL_LOW_VOLTAGE		3500 /* mV */
#define	BATT_ADC_CAL_HIGH_VOLTAGE		4000 /* mV */
#define BATT_ADC_CAL_DIFF_VOLTAGE		( BATT_ADC_CAL_HIGH_VOLTAGE - BATT_ADC_CAL_LOW_VOLTAGE )
#define CALIBRATE_SAMPLE(s, o, diff, ad, ov) \
  ((int)((int)s - (int)o) * (int)diff / (int)ad + ov)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDE_BARCELONA_BARC_BATTERY_H */

/* EOF */
