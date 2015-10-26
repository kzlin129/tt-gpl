/*
 * s5m8751_regulator.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/s5m8751/s5m8751_core.h>
#include <linux/mfd/s5m8751/s5m8751_pmic.h>
//#include <linux/platform_device.h>
//#include <linux/regulator/driver.h>
//#include <linux/regulator/machine.h>
#include <linux/mfd/s5m8751/s5m8751_regulator.h>

extern struct s5m8751 *s5m8751;

static int s5m8751_ldo_val_to_mvolts(unsigned int val)
{
	return (val * 100) + 1800;
}

static u8 s5m8751_ldo_mvolts_to_val(int mV)
{
	return (mV - 1800) / 100;
}

static int s5m8751_ldo3_4_val_to_mvolts(unsigned int val)
{
	return (val * 100) + 800;
}

static u8 s5m8751_ldo3_4_mvolts_to_val(int mV)
{
	return (mV - 800) / 100;
}

static int s5m8751_buck1_val_to_mvolts(unsigned int val)
{
	return (val * 25) + 1800;
}

static int s5m8751_buck2_val_to_mvolts(unsigned int val)
{
	return (val * 25) + 500;
}

static u8 s5m8751_buck1_mvolts_to_val(int mV)
{
	return (mV - 1800) / 25;
}

static u8 s5m8751_buck2_mvolts_to_val(int mV)
{
	return (mV - 500) / 25;
}

int s5m8751_ldo_set_voltage(int regulator, int mV)
{
	int volt_reg;
	int ldo = regulator;
	int volt_value;
	int mask;
	u8 val;

	if (ldo < S5M8751_LDO_AUDIO || ldo > S5M8751_LDO_MEMORY)
		return -EINVAL;

	if (regulator == S5M8751_LDO3 || regulator == S5M8751_LDO4)
	{
		if (mV < 800 || mV > 2300)
			return -EINVAL;
		volt_value = s5m8751_ldo3_4_mvolts_to_val(mV);	
	}
	else

	{
		if (mV < 1800 || mV > 3300)
			return -EINVAL;
		volt_value = s5m8751_ldo_mvolts_to_val(mV);	
	}
	
	switch(ldo)
	{
		case S5M8751_LDO_AUDIO :
			volt_reg = S5M8751_LDO_AUDIO_VSET;
			mask = S5M8751_LDO_AUDIO_VSET_MASK;
			break;
		case S5M8751_LDO1 :
			volt_reg = S5M8751_LDO1_VSET;
			mask = S5M8751_LDO1_VSET_MASK;
			break;
		case S5M8751_LDO2 :
			volt_reg = S5M8751_LDO2_VSET;
			mask = S5M8751_LDO2_VSET_MASK;
			break;
		case S5M8751_LDO3 :
			volt_reg = S5M8751_LDO3_VSET;
			mask = S5M8751_LDO3_VSET_MASK;
			break;
		case S5M8751_LDO4 :
			volt_reg = S5M8751_LDO4_VSET;
			mask = S5M8751_LDO4_VSET_MASK;
			break;
		case S5M8751_LDO_MEMORY :
			volt_reg = S5M8751_LDO_MEMORY_VSET;
			mask = S5M8751_LDO_MEMORY_VSET_MASK;
			break;
		default:
			return -EINVAL;
	}

	/* Set the LDO voltage */
	val = s5m8751_reg_read(s5m8751, volt_reg)& ~mask;
	s5m8751_reg_write(s5m8751, volt_reg, val | volt_value);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_ldo_set_voltage);

int s5m8751_ldo_get_voltage(int regulator)
{
	int volt_reg;
	int ldo = regulator;
	int mask;
	u8 val;

	if (ldo < S5M8751_LDO_AUDIO || ldo > S5M8751_LDO_MEMORY)
		return -EINVAL;

	switch (ldo) {
		case S5M8751_LDO_AUDIO :
			volt_reg = S5M8751_LDO_AUDIO_VSET;
			mask = S5M8751_LDO_AUDIO_VSET_MASK;
			break;
		case S5M8751_LDO1 :
			volt_reg = S5M8751_LDO1_VSET;
			mask = S5M8751_LDO1_VSET_MASK;
			break;
		case S5M8751_LDO2 :
			volt_reg = S5M8751_LDO2_VSET;
			mask = S5M8751_LDO2_VSET_MASK;
			break;
		case S5M8751_LDO3 :
			volt_reg = S5M8751_LDO3_VSET;
			mask = S5M8751_LDO3_VSET_MASK;
			break;
		case S5M8751_LDO4 :
			volt_reg = S5M8751_LDO4_VSET;
			mask = S5M8751_LDO4_VSET_MASK;
			break;
		case S5M8751_LDO_MEMORY :
			volt_reg = S5M8751_LDO_MEMORY_VSET;
			mask = S5M8751_LDO_MEMORY_VSET_MASK;
			break;
		default:
			return -EINVAL;
	}

	/* Get the LDO voltage */
	val = s5m8751_reg_read(s5m8751, volt_reg) & mask;
	
	if(ldo == S5M8751_LDO3 || ldo == S5M8751_LDO4)
		return s5m8751_ldo3_4_val_to_mvolts(val);
	else 
		return s5m8751_ldo_val_to_mvolts(val);

}
EXPORT_SYMBOL_GPL(s5m8751_ldo_get_voltage);

int s5m8751_ldo_enable(int regulator)
{
	int ldo = regulator;
	int volt_reg;
	u8 shift;

	if (ldo < S5M8751_LDO_AUDIO || ldo > S5M8751_LDO_MEMORY)
		return -EINVAL;

	switch (ldo) {
		case S5M8751_LDO_AUDIO:
			volt_reg = S5M8751_ONOFF3;
			shift = S5M8751_LDO_AUDIO_SHIFT;
			break;
		case S5M8751_LDO1:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO1_SHIFT;
			break;
		case S5M8751_LDO2:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO2_SHIFT;
			break;
		case S5M8751_LDO3:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO3_SHIFT;
			break;
		case S5M8751_LDO4:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO4_SHIFT;
			break;
		case S5M8751_LDO_MEMORY:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO_MEMORY_SHIFT;
			break;
		default:
			return -EINVAL;
	}
	s5m8751_set_bits(s5m8751, volt_reg, 1 << shift);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_ldo_enable);

int s5m8751_ldo_disable(int regulator)
{
	int ldo = regulator;
	int volt_reg;
	u8 shift;

	if (ldo < S5M8751_LDO_AUDIO || ldo > S5M8751_LDO_MEMORY)
		return -EINVAL;

	switch (ldo) {
		case S5M8751_LDO_AUDIO:
			volt_reg = S5M8751_ONOFF3;
			shift = S5M8751_LDO_AUDIO_SHIFT;
			break;
		case S5M8751_LDO1:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO1_SHIFT;
			break;
		case S5M8751_LDO2:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO2_SHIFT;
			break;
		case S5M8751_LDO3:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO3_SHIFT;
			break;
		case S5M8751_LDO4:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO4_SHIFT;
			break;
		case S5M8751_LDO_MEMORY:
			volt_reg = S5M8751_ONOFF2;
			shift = S5M8751_LDO_MEMORY_SHIFT;
			break;
		default:
			return -EINVAL;
	}
	s5m8751_clear_bits(s5m8751, volt_reg, 1 << shift);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_ldo_disable);

int s5m8751_buck_set_voltage(int regulator, int mV)
{
	int volt_reg;
	int volt_value;
	int buck = regulator;
	int mask;
	int shift;
	u8 val;

	if (buck < S5M8751_BUCK1_1 || buck > S5M8751_BUCK2_2)
		return -EINVAL;

	if (buck ==  S5M8751_BUCK1_1 || buck == S5M8751_BUCK1_2)
	{
		if (mV < 1800 || mV > 3300)
			return -EINVAL;
	}
	else 
	{
		if (mV < 500 || mV > 1650)
			return -EINVAL;		
	}
	
	switch (buck)
	{
		case S5M8751_BUCK1_1:
			volt_reg = S5M8751_BUCK1_V1_SET;
			mask = S5M8751_BUCK1_V1_SET_MASK;
			shift = S5M8751_BUCK1_V1_SET_SHIFT;
			volt_value = s5m8751_buck1_mvolts_to_val(mV);	
			break;
		case S5M8751_BUCK1_2:
			volt_reg = S5M8751_BUCK1_V2_SET;
			mask = S5M8751_BUCK1_V2_SET_MASK;
			shift = S5M8751_BUCK1_V2_SET_SHIFT;
			volt_value = s5m8751_buck1_mvolts_to_val(mV);	
			break;
		case S5M8751_BUCK2_1:
			volt_reg = S5M8751_BUCK2_V1_SET;
			mask = S5M8751_BUCK2_V1_SET_MASK;
			shift = S5M8751_BUCK2_V1_SET_SHIFT;
			volt_value = s5m8751_buck2_mvolts_to_val(mV);	
			break;
		case S5M8751_BUCK2_2:
			volt_reg = S5M8751_BUCK2_V2_SET;
			mask = S5M8751_BUCK2_V2_SET_MASK;
			shift = S5M8751_BUCK2_V2_SET_SHIFT;
			volt_value = s5m8751_buck2_mvolts_to_val(mV);	
			break;
		default:
			return -EINVAL;
	}
	
	/* Set the BUCK voltage */
	val = s5m8751_reg_read(s5m8751, volt_reg)& ~mask;
	s5m8751_reg_write(s5m8751, volt_reg, val | volt_value);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_set_voltage);

int s5m8751_buck_get_voltage(int regulator)
{
	int volt_reg;
	int buck = regulator;
	int mask;
	u8 val;

	switch (buck)
	{
		case S5M8751_BUCK1_1:
			volt_reg = S5M8751_BUCK1_V1_SET;
			mask = S5M8751_BUCK1_V1_SET_MASK;
			break;
		case S5M8751_BUCK1_2:
			volt_reg = S5M8751_BUCK1_V2_SET;
			mask = S5M8751_BUCK1_V2_SET_MASK;
			break;
		case S5M8751_BUCK2_1:
			volt_reg = S5M8751_BUCK2_V1_SET;
			mask = S5M8751_BUCK2_V1_SET_MASK;
			break;
		case S5M8751_BUCK2_2:
			volt_reg = S5M8751_BUCK2_V2_SET;
			mask = S5M8751_BUCK2_V2_SET_MASK;
			break;
		default:
			return -EINVAL;
	}
	
	// Get the BUCK voltage 
	val = s5m8751_reg_read(s5m8751, volt_reg) & mask;
	if (buck == S5M8751_BUCK1_1 || buck == S5M8751_BUCK1_2)
		return s5m8751_buck1_val_to_mvolts(val);
	else if (buck == S5M8751_BUCK2_1 || buck == S5M8751_BUCK2_2)
		return s5m8751_buck2_val_to_mvolts(val);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_get_voltage);

int s5m8751_buck_enable(int regulator)
{
	int buck = regulator;
	int volt_reg;
	u8 shift;

	if (buck < S5M8751_BUCK1_1 || buck > S5M8751_BUCK2_2)
		return -EINVAL;
	
	volt_reg = S5M8751_ONOFF3;
	
	switch (buck)
	{
		case S5M8751_BUCK1_1:
		case S5M8751_BUCK1_2:
			shift = S5M8751_BUCK1_SHIFT;
			break;
		case S5M8751_BUCK2_1:
		case S5M8751_BUCK2_2:
			shift = S5M8751_BUCK2_SHIFT;
			break;
		default:
			return -EINVAL;
	}	
	s5m8751_set_bits(s5m8751, volt_reg, 1 << shift);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_enable);

int s5m8751_buck_disable(int regulator)
{
	int buck = regulator;
	int volt_reg;
	u8 shift;

	if (buck < S5M8751_BUCK1_1 || buck > S5M8751_BUCK2_2)
		return -EINVAL;

	volt_reg = S5M8751_ONOFF3;

	switch (buck)
	{
		case S5M8751_BUCK1_1:
		case S5M8751_BUCK1_2:
			shift = S5M8751_BUCK1_SHIFT;
			break;
		case S5M8751_BUCK2_1:
		case S5M8751_BUCK2_2:
			shift = S5M8751_BUCK2_SHIFT;
			break;
		default:
			return -EINVAL;
	}
	s5m8751_clear_bits(s5m8751, volt_reg, 1 << shift);
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_disable);

int s5m8751_buck_set_mode(int regulator, unsigned int mode)
{
	int buck = regulator;
	int volt_reg;
	int shift;
	int mask;
	int buck_mode;
	u8 val;	

	if (buck < S5M8751_BUCK1_1 || buck > S5M8751_BUCK2_2)
		return -EINVAL;

	switch (mode)
	{
		case PFM_MODE:
			buck_mode = S5M8751_BUCK_MODE_PFM;
			break;
		case PWM_MODE:
			buck_mode = S5M8751_BUCK_MODE_PWM;
			break;
		case AUTO_MODE:
			buck_mode = S5M8751_BUCK_MODE_AUTO;
			break;
		default:
			return -EINVAL;
	}

	switch (buck)
	{
		case S5M8751_BUCK1_1:
		case S5M8751_BUCK1_2:
			volt_reg = S5M8751_BUCK1_V2_SET;
			mask = S5M8751_BUCK1_MODE_MASK;
			shift = S5M8751_BUCK1_MODE_SHIFT;
			break;
		case S5M8751_BUCK2_1:
		case S5M8751_BUCK2_2:
			volt_reg = S5M8751_BUCK2_V2_SET;
			mask = S5M8751_BUCK2_MODE_MASK;
			shift = S5M8751_BUCK2_MODE_SHIFT;
			break;
		default:
			return -EINVAL;
	}

	val = s5m8751_reg_read(s5m8751, volt_reg) & ~mask;
	s5m8751_reg_write(s5m8751, volt_reg, val | (buck_mode << shift)); 
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_set_mode);

unsigned int s5m8751_buck_get_mode(int regulator)
{
	int buck = regulator;
	int volt_reg;
	int mask;
	int shift;
	u8 mode;
	
	if (buck < S5M8751_BUCK1_1 || buck > S5M8751_BUCK2_2)
		return -EINVAL;
	
	switch (buck)
	{
		case S5M8751_BUCK1_1:
		case S5M8751_BUCK1_2:
			volt_reg = S5M8751_BUCK1_V2_SET;
			mask = S5M8751_BUCK1_MODE_MASK;
			shift = S5M8751_BUCK1_MODE_SHIFT;
			break;
		case S5M8751_BUCK2_1:
		case S5M8751_BUCK2_2:
			volt_reg = S5M8751_BUCK2_V2_SET;
			mask = S5M8751_BUCK2_MODE_MASK;
			shift = S5M8751_BUCK2_MODE_SHIFT;
			break;
		default:
			return -EINVAL;
	}
	mode = s5m8751_reg_read(s5m8751, volt_reg) & mask;
	return mode >> shift;
}
EXPORT_SYMBOL_GPL(s5m8751_buck_get_mode);

/* Module Information */
MODULE_AUTHOR ("Won Jong Bin");
MODULE_DESCRIPTION("S5M8751 regulator driver");
MODULE_LICENSE("GPL");

