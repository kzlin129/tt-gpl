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
*  pm_battmgr.h
*
*  PURPOSE:
*
*     Battery management defintions.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( _PM_BATTMGR_H_ )
#define _PM_BATTMGR_H_

#if defined( __KERNEL__ )

#if defined( CONFIG_BCM_BATTERY_MANAGER )

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pm_platforms.h>

/* ---- Constants and Types ---------------------------------------------- */
/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */


void pm_battMgrInit ( int tempAdcChannel,
                      int voltAdcChannel,
                      int earlyShutoff,
                      BCM_PMU_Operations_t *pmu,
                      PM_Status *status );

void pm_battMgrTask( void );

void pm_battMgrStartTurnOffTimer( void );
void pm_battMgrTurnOff( void );

void pm_battMgrBatteryLow( void );
void pm_battMgrBatteryFull( void );

void pm_battMgrChargerInserted( int chargerId );
void pm_battMgrChargerRemoved( int chargerId );

void pm_battMgrLogLevel(int level); 

void pm_battMgrInitVoltTable(PM_Battery_Level* table, int temp_high_limit,
                              int temp_low_limit);

#else

// Allow Battery Manager calls to compile but not do anything

#define pm_battMgrInit(tempAdcChannel, voltAdcChannel, earlyShutoff, pmu, status) \
   ((void)(tempAdcChannel), (void)(voltAdcChannel), (void)(earlyShutoff), (void)(pmu), (void)(status))

#define pm_battMgrTask()

#define pm_battMgrStartTurnOffTimer()
#define pm_battMgrTurnOff()

#define pm_battMgrBatteryLow()
#define pm_battMgrBatteryFull()

#define pm_battMgrChargerInserted(chargerId) ((void)(chargerId))
#define pm_battMgrChargerRemoved(chargerId) ((void)(chargerId))

#define pm_battMgrLogLevel(level) ((void)(level))
#define pm_battMgrInitVoltTable(table, temp_high_limit, temp_low_limit) \
   ((void) (table), (void) (temp_high_limit), (void) (temp_low_limit))

#endif

#endif

#endif  /* _PM_BATTMGR_H_ */

