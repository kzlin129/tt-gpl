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

#ifndef DRIVERS_I2C_CHIPS_LTC3577_PMIC_H
#define DRIVERS_I2C_CHIPS_LTC3577_PMIC_H

struct ltc3577_i2c_driver
{
	struct ltc3577_platform		*pdata;
	struct i2c_driver		i2c_driver;
	struct device_driver		plat_driver;
	struct i2c_client		client;
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct notifier_block		freq_transition;
	struct notifier_block		freq_policy;
	unsigned int				max_freq;
	unsigned int				min_freq;
	unsigned int				last_freq;
#endif
	atomic_t					suspended;
};

#define ltc3577_set_sw2_voltage( client, voltage )		(ltc3577_set_voltage( (client), (voltage), ltc3577_sw2_volttable ))
#define ltc3577_set_sw3_voltage( client, voltage )		(ltc3577_set_voltage( (client), (voltage), ltc3577_sw3_volttable ))
#define ltc3577_set_sw2_volt_from_freq( client, freq, pdata )	(ltc3577_set_sw2_voltage( (client), ltc3577_get_match_volt( (freq), (pdata)->sw2.power_setting ) ))
#define ltc3577_set_sw3_volt_from_freq( client, freq, pdata )	(ltc3577_set_sw3_voltage( (client), ltc3577_get_match_volt( (freq), (pdata)->sw3.power_setting ) ))
#define ltc3577_get_sw2_voltage( ) )				(ltc3577_get_voltage( ltc3577_sw2_volttable ))
#define ltc3577_get_sw3_voltage( ) )				(ltc3577_get_voltage( ltc3577_sw3_volttable ))

#define LTC3577_VOLTTBL_SIZE            16	/* Maximum number of entries in the table. */
#define LTC3577_I2C_SLAVE_ADDR		0x09	/* Address LTC3577 can be found at. */
#define LTC3577_VOLT_PRECISION		1	/* In procent */
#endif
