/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/




/*
*
*****************************************************************************
*
*  pm_platforms.h
*
*  PURPOSE:
*
*     Platform defintions for Power Manager.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( _PM_PLATFORMS_H_ )
#define _PM_PLATFORMS_H_

#if defined( __KERNEL__ )

/* ---- Include Files ---------------------------------------------------- */

#include <linux/init.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>

/* ---- Constants and Types ---------------------------------------------- */

#define PM_MAX_REGULATORS_PER_PLATFORM 15
#define PM_PLATFORM_NAME_MAX_LEN 128
#define PM_MAX_BATTERY_LEVELS 5

/*
 * Component bitmap
 */

typedef uint32_t PM_Component_Bitmap;
#define NO_CONNECTED_COMPONENTS 0
#define CONNECT_1_COMPONENT(a) (1 << a)
#define CONNECT_2_COMPONENTS(a,b) ((1 << a) | (1 << b))
#define CONNECT_3_COMPONENTS(a,b,c) ((1 << a) | (1 << b) | (1 << c))
#define SET_COMPONENT(bitmap,id) (bitmap |= (1 << id))
#define CLEAR_COMPONENT(bitmap,id) (bitmap &= ~(1 << id))
#define IS_COMPONENT_SET(bitmap,id) (bitmap & (1 << id))
#define IS_COMPONENT_SUBSET(bitmap,sub_bitmap) ((bitmap & sub_bitmap) == sub_bitmap)

//#if (PM_NUM_COMPONENTS > (sizeof(PM_Component_Bitmap) * 8)))
//#error Too Many components to fit into bitmap.  Time to re-define bitmap.
//#endif

typedef struct
{
   int id;
   const char *name;
   uint32_t mV;
   BCM_PMU_Regulator_State_t initial_state;
   PM_Component_Bitmap connected_components;
} PM_Regulator_Map;

typedef struct
{
   PM_Component id;
   PM_CompPowerLevel compPowerLevel[PM_NUM_POWER_LEVELS];
   void (* extra_init) ( void );
   void (* extra_action) (PM_CompPowerLevel powerLevel);
} PM_Component_Map;

typedef struct
{
   int adc_output;
   int voltage; /*in mV*/
} PM_Battery_Level;

typedef struct
{
   const char *name;
   BCM_PMU_Chip_t pmu_chip;
   int battery_voltage_adc_channel;
   int battery_temperature_adc_channel;
   int battery_early_shutoff;
   int battery_charging_only;
   int battery_temp_high_limit;
   int battery_temp_low_limit;
   int (* detect) ( void );
   void (* extra_init) ( void );
   PM_Regulator_Map regulators[PM_MAX_REGULATORS_PER_PLATFORM];
   PM_Component_Map components[PM_NUM_COMPONENTS];
   PM_Battery_Level batteryleveltable[PM_MAX_BATTERY_LEVELS];
} PM_Platform;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

void __init pm_register_platform( const PM_Platform *platform );

#endif

#endif  /* _PM_PLATFORMS_H_ */

