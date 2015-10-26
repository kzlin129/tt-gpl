/*
 * s5m8751_regulator.h  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#define PFM_MODE			0
#define PWM_MODE			1
#define AUTO_MODE			2

//static int s5m8751_ldo_val_to_mvolts(unsigned int val);
//tatic u8 s5m8751_ldo_mvolts_to_val(int mV);
//static int s5m8751_ldo3_4_val_to_mvolts(unsigned int val);
//tatic u8 s5m8751_ldo3_4_mvolts_to_val(int mV);
//static int s5m8751_buck1_val_to_mvolts(unsigned int val);
//static int s5m8751_buck2_val_to_mvolts(unsigned int val);
//static u8 s5m8751_buck1_mvolts_to_val(int mV);
//static u8 s5m8751_buck2_mvolts_to_val(int mV);

int s5m8751_ldo_set_voltage(int regulator, int mV);
int s5m8751_ldo_get_voltage(int regulator);
int s5m8751_ldo_enable(int regulator);
int s5m8751_ldo_disable(int regulator);

int s5m8751_buck_set_voltage(int regulator, int mV);
int s5m8751_buck_get_voltage(int regulator);
int s5m8751_buck_enable(int regulator);
int s5m8751_buck_disable(int regulator);
int s5m8751_buck_set_mode(int regulator, unsigned int mode);
unsigned int s5m8751_buck_get_mode(int regulator);

