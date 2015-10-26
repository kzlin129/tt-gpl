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

#ifndef DRIVERS_BARCELONA_PMIC_LTC3455_PMIC_H
#define DRIVERS_BARCELONA_PMIC_LTC3455_PMIC_H

struct ltc3455_driver
{
	struct device_driver		driver;
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct ltc3455_pdata		*pdata;
	struct notifier_block		freq_transition;
	struct notifier_block		freq_policy;
#endif
};

#endif
