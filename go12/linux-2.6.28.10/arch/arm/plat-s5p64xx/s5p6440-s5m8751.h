/*
 * s5p6440-s5m8751.h  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef __S5P6440S5M8751_H_
#define __S5P6440S5M8751_H_


#define LDO_AUDIO_VOL_STEP	0
#define LDO1_VOL_STEP		1
#define LDO2_VOL_STEP		2
#define LDO3_VOL_STEP		3
#define LDO4_VOL_STEP		4
#define LDO_MEM_VOL_STEP	5

#define BUCK1_VOL_STEP		6
#define BUCK2_VOL_STEP		7

typedef struct {
	unsigned int pwr_type;
	unsigned int curr_voltage_ldo_audio;
	unsigned int curr_voltage_ldo1;
	unsigned int curr_voltage_ldo2;
	unsigned int curr_voltage_ldo3;
	unsigned int curr_voltage_ldo4;
	unsigned int curr_voltage_ldo_memory;

	unsigned int curr_voltage_buck1;
	unsigned int curr_voltage_buck2;
	unsigned int new_voltage;
} s5p6440_s5m8751_info;

#endif
