/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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

#if CONFIG_BCM_POWER_MANAGER

// Power Management custom platform implementation

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/broadcom/pm_platforms.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Platform Definition Table ---------------------------------------- */
static PM_Platform __initdata bcm94760_platform =
{
   name:       "BCM94760",
   pmu_chip:   PMU_BCM59040,
   battery_voltage_adc_channel: 3,
   battery_temperature_adc_channel: 2,
   battery_early_shutoff: 0,
   battery_charging_only: 1,
   battery_temp_high_limit:178,
   battery_temp_low_limit:31,
   detect:     NULL,
   extra_init: NULL,
   components:
   {
	   	// Terminator
    	[0] = { id: PM_NUM_COMPONENTS }
   },
   regulators:
   {
	   	// Terminator
    	[0] = { id: -1 }
   },
   batteryleveltable:
   {
      {
        adc_output: 0xb0,
        voltage: 3500,
      },
      {
        adc_output: 0xb6,
        voltage: 3600,
      },
      {
        adc_output: 0xbc,
        voltage: 3720,
      },
      {
        adc_output: 0xc3,
        voltage: 3800,
      },
      {
        adc_output: 0xc9,
        voltage: 3950,
      },
   },
};

static int __init register_bcm94760_platform( void )
{
   pm_register_platform( &bcm94760_platform );

   return 0;
}

subsys_initcall( register_bcm94760_platform );

#endif
