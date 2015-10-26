/* include/asm-arm/arch/bcm_gen_battery.h
 *
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author: Pepijn de Langen <pepijn.delangen@tomtom.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __INCLUDE_LINUX_BCM_GEN_BATTERY_H__
#define __INCLUDE_LINUX_BCM_GEN_BATTERY_H__

#define	BATT_POLL_AVG_COUNT	50		/* Number of samples per average value. */
#define BATT_POLL_INTERVAL	HZ/4		/* In jiffies, 4 times per second */
#define BCM_BATTERY_DRIVER	"bcm_battery"


#define	INIT_ATOMIC_ADC_VALUE( val )	\
{\
	.lock			= SPIN_LOCK_UNLOCKED,\
	.value			= val,\
}

struct atomic_adc_value
{
	spinlock_t		lock;
	u32				value;
};

typedef struct 
{
	u32				vbatt;				/* voltage in uV */
	u32				remaining_capacity;	/* capacity in % */
	u32				remaining_time;		/* capacity in minutes */
} batt_discharge_lut_t;

typedef struct
{
	u32				current_current;
	batt_discharge_lut_t *lut;
	u8				dclut;
} batt_discharge_curve_t;

typedef struct
{
	u32				vm_bat;		/* voltage in uV */
	u32				esr;		/* esr in ? */
} batt_esr_curve_t;

struct bcm_battery
{
	const char		*model;
	batt_discharge_lut_t	*discharge_curves;
	u8			discharge_curve_entries;

	batt_esr_curve_t	*esr_curves;
	u8			esr_curve_entries;
};

struct bcm_batt_info
{
	struct
	{
		u32		battery_voltage_resistor_1;
		u32		battery_voltage_resistor_2;
		u32		charge_current_resistor;
		u32		reference_voltage;
		u32		accuracy;
		s32		correction;
	} adc;

	struct bcm_battery *battery;
};

struct bcm_battery_platform_info
{
	int (*platform_init)( struct platform_device *pdevice );
	void (*platform_remove)( struct platform_device *pdevice );

	u32 (*get_charge_current_adc)( void );
	u32 (*adc_read)(int device, int channel);
	s16 (*get_fuel_gauge_sample)( void );

	struct bcm_batt_info      *battery_info;
};
#endif /* __INCLUDE_LINUX_BCM_GEN_BATTERY_H__ */
