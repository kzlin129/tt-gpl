/*
 *  pmic.h  -- PMIC driver for LTC3577
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_LTC3577_PMIC_H_
#define __LINUX_MFD_LTC3577_PMIC_H_

typedef enum {eCHARGING_500mA, eCHARGING_1A} charge_e;

struct ltc3577_pmic_data {
	uint8_t burst_mode;
	uint8_t slew_rate;
	void (*set_charge)(charge_e charge);
};

#define LTC3577_DEVNAME		"ltc3577-pmic"
#endif /*__LINUX_MFD_LTC3577_MFD_H_ */
