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
/*
 *  pm_battmgr.c
 *
 *  PURPOSE:
 *
 *     Battery management for BCM116x based boards.  Part of Power Manager.
 *
 *  NOTES:
 *
 *****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <asm/arch/hw_cfg.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pm_battmgr.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/delay.h>
#include <linux/broadcom/pmu_bcm59040.h>

//#include <asm/arch/auxadc.h>


/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL  (DBG_ERROR)
//#define DBG_DEFAULT_LEVEL  (DBG_ERROR|DBG_INFO|DBG_TRACE)

#if DEBUG
#   define PM_DEBUG(level,x) {if (level & gPmBattData.logLevel) printk x;}
#else
#   define PM_DEBUG(level,x)
#endif

/*
 * Battery manangement
 */
#define ADC_RUNNING_AVG_SHIFT   5   // # of shifts used to divide the sum to get the average.
#define ADC_RUNNING_AVG_SIZE    (1 << ADC_RUNNING_AVG_SHIFT)    // # of samples to perform voltage running sum
#define NOT_READY               0xffff      // voltage or temperature is not available yet
#define TEMP_HIGH_LIMIT         48          // ADC output at 5C
#define TEMP_LOW_LIMIT          7           // ADC output at 60C
#define TEMP_INVALID            255           // ADC output at 60C
#define BATT_HYSTERESIS         4           // hysteresis in ADC units  declaring new level
#define BATT_HYSTERESIS_DELTA   0           // add on to BATT_HYSTERESIS

// ADC value = V * 51.72
#define RECHARGE_THRESH         197         // threshold to start charging battery if low (~3.8 Volts)
#define CHARGE_RESTART_TIME     100         // # of task intervals for re-starting charging

typedef struct
{
   int logLevel;   // Logging level

   BCM_PMU_Operations_t *pmu;  // PMU Chip handle
   PM_Status *status;              // Global status structure
   int chargerId;                  // Current Charger ID
   int earlyShutoff;               // Flag enabling early shutoff using voltage ADC

   int fastChargeOn;
   int fastChargeTimer;

   // for battery temperature control
   int TempAdcChannel;
   int TempReadings[ ADC_RUNNING_AVG_SIZE ];    // Saved temperature-voltage measurements
   int StartTempAvg;                            // 1, start averaging temp-volt measurements
   int RunningTempSum;                          // Running temperature-voltage sum
   int RunningTempAvg;                          // Running temperature-voltage average
   int TempReadingsIndex;                       // Next location to save measurment
   int TempHighLimit;                           // High limit for temperature
   int TempLowLimit;                            // Low limit for temperature

   // for battery voltage measurement
   int VoltAdcChannel;
   int VoltReadings[ ADC_RUNNING_AVG_SIZE ];    // Saved battery-voltage measurements
   int StartVoltAvg;                            // 1, start averaging volt measurements
   int RunningVoltSum;                          // Running voltage sum
   int RunningVoltAvg;                          // Running voltage average
   int VoltReadingsIndex;                       // Next location to save measurment
} PM_BATT_DATA;

/* ---- Private Variables ------------------------------------------------ */

static PM_BATT_DATA gPmBattData =
{
   logLevel: DBG_DEFAULT_LEVEL,
};

static int BatteryLevelTable[PM_MAX_BATTERY_LEVELS] = {
    184,            // 3.56 V
    189,            // 3.65 V
    194,            // 3.75 V
    199,            // 3.85 V
    204             // 3.95 V
};

/* ---- Private Function Prototypes -------------------------------------- */

static int pm_battMgrGetVoltage( void );
static int pm_battMgrGetTemperature( void );
static int pm_battMgrCalcBatteryLevel( int VoltAvg );

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  pm_battMgrInit
*
***************************************************************************/
void pm_battMgrInit ( int tempAdcChannel,
                      int voltAdcChannel,
                      int earlyShutoff,
                      BCM_PMU_Operations_t *pmu,
                      PM_Status *status )
{
   int i;
   int chargerId;

   // intialize global variables
   gPmBattData.pmu = pmu;
   gPmBattData.status = status;

   gPmBattData.earlyShutoff = earlyShutoff;

   gPmBattData.fastChargeOn = 0;
   gPmBattData.fastChargeTimer = 0;

   // for battery temperature measurement
   gPmBattData.TempAdcChannel = tempAdcChannel;
   gPmBattData.StartTempAvg = 0;                        // 1, start averaging temp-volt measurements
   gPmBattData.RunningTempSum = 0;                      // Running temperature-voltage sum
   gPmBattData.RunningTempAvg = NOT_READY;              // Running temperature-voltage average
   gPmBattData.TempReadingsIndex = 0;                   // Next location to save measurment
   for ( i = 0 ; i < ADC_RUNNING_AVG_SIZE ; i++ )       // Saved temperature-voltage measurements
   {
      gPmBattData.TempReadings[ i ] = 0;
   }

   // for battery voltage measurement
   gPmBattData.VoltAdcChannel = voltAdcChannel;
   gPmBattData.StartVoltAvg = 0;                        // 1, start averaging volt measurements
   gPmBattData.RunningVoltSum = 0;                      // Running voltage sum
   gPmBattData.RunningVoltAvg = NOT_READY;              // Running voltage average
   gPmBattData.VoltReadingsIndex = 0;                   // Next location to save measurment
   for ( i = 0 ; i < ADC_RUNNING_AVG_SIZE ; i++ )       // Saved battery-voltage measurements
   {
      gPmBattData.VoltReadings[ i ] = 0;
   }

   // Kick-start charging if charger is present
   if (PMU_charger_is_inserted(gPmBattData.pmu, &chargerId))
   {
      PM_DEBUG(DBG_INFO,("pmbatt - Charger %d detected on init.\n", chargerId));
      pm_battMgrChargerInserted(chargerId);
   }

   // Kick-start periodic battery management task
   pm_battMgrTask();
}


/****************************************************************************
*
*  pm_GetVoltageI2c
*
*  returns: battery voltage measurement in ADC units or -1 on error
*
***************************************************************************/
static int pm_GetVoltageI2c( int AdcChannel )
{
   //QP: Need to change bcm59040 reg#
	
   int adcOut = 0;
   int rc=0;
   u8 adcOutBytes[2];

	if (AdcChannel <= 0)
   	return -1;

   // Select ADC channel
   //rc = pmu_i2c_write(BCM59040_REG_PMUID, AdcChannel);
   if (rc < 0)
   {
      printk("pm_GetVoltageI2c: error writing ADC Control7 register.\n");
      return rc;
   }
	
   /* get voltage bytes (2 bytes)*/
   //rc = pmu_i2c_read_bytes(BCM59040_REG_PMUID, adcOutBytes, 2);
   if ( rc < 0)
   {
      printk("pm_GetVoltageI2c: failed to read adcOut bytes\n");
      return rc;
   }

	//QP: For testing
	adcOutBytes[0] = 0x0;	
	adcOutBytes[1] = AdcChannel;	
	adcOut = (adcOutBytes[0]<<8 | adcOutBytes[1]);
	//QP

	return adcOut;
}


/****************************************************************************
*
*  pm_battMgrGetVoltage
*
*  returns: battery voltage measurement in ADC units or -1 on error
*
***************************************************************************/
static int pm_battMgrGetVoltage( void )
{
   int adcOut;
   int rc;
   int i;

   if (gPmBattData.VoltAdcChannel <= 0)
      return -1;

   // get 10 bit ADC output
   //QP: rc = auxadc_access(gPmBattData.VoltAdcChannel);
	rc = pm_GetVoltageI2c(gPmBattData.VoltAdcChannel);
   
   if ( rc <= 0 )
   {
       printk("pm: error, battery voltage ADC read failed, rc = %d\n",rc);
       return -1;
   }
   adcOut = rc;

   // If it is the very first measurement taken, initialize the buffer elements to the same value
   if ( !gPmBattData.StartVoltAvg )
   {
      gPmBattData.StartVoltAvg = 1;
      gPmBattData.RunningVoltSum = 0;
      for ( i = 0 ; i < ADC_RUNNING_AVG_SIZE ; i++ )
      {
         gPmBattData.VoltReadings[ i ] = adcOut;
         gPmBattData.RunningVoltSum += adcOut;
      }
      gPmBattData.VoltReadingsIndex = 0;
   }

    // Keep the sum running forwards
    gPmBattData.RunningVoltSum -= gPmBattData.VoltReadings[ gPmBattData.VoltReadingsIndex ];
    gPmBattData.VoltReadings[ gPmBattData.VoltReadingsIndex ] = adcOut;
    gPmBattData.RunningVoltSum += adcOut;
    gPmBattData.VoltReadingsIndex++;

    // Wrap the index
    if ( gPmBattData.VoltReadingsIndex >= ADC_RUNNING_AVG_SIZE )
    {
        gPmBattData.VoltReadingsIndex = 0;
    }

    // Divide the running sum by number of measurements taken to get the average
    gPmBattData.RunningVoltAvg = ((gPmBattData.RunningVoltSum >> (ADC_RUNNING_AVG_SHIFT - 1)) + 1 ) >> 1;

    return gPmBattData.RunningVoltAvg;
}


/****************************************************************************
*
*  pm_battMgrGetTemperature
*
*  returns: temperature measurement in ADC units or -1 on error
*
***************************************************************************/
static int pm_battMgrGetTemperature( void )
{
   int adcOut;
   int rc;
   int i;

   if (gPmBattData.TempAdcChannel <= 0)
      return -1;

   // get 10 bit ADC output
   //QP: rc = auxadc_access(gPmBattData.TempAdcChannel);
   rc = 1;
   if ( rc <= 0 )
   {
       printk("pm: error, battery temperature ADC read failed, rc = %d\n",rc);
       return -1;
   }
   adcOut = rc;

   // If it is the very first measurement taken, initialize the buffer elements to the same value
   if ( !gPmBattData.StartTempAvg )
   {
       gPmBattData.StartTempAvg = 1;
       gPmBattData.RunningTempSum = 0;
       for ( i = 0 ; i < ADC_RUNNING_AVG_SIZE ; i++ )
       {
           gPmBattData.TempReadings[ i ] = adcOut;
           gPmBattData.RunningTempSum += adcOut;
       }
       gPmBattData.TempReadingsIndex = 0;
   }

   // Keep the sum running forwards
   gPmBattData.RunningTempSum -= gPmBattData.TempReadings[ gPmBattData.TempReadingsIndex ];
   gPmBattData.TempReadings[ gPmBattData.TempReadingsIndex ] = adcOut;
   gPmBattData.RunningTempSum += adcOut;
   gPmBattData.TempReadingsIndex++;

   // Wrap the index
   if ( gPmBattData.TempReadingsIndex >= ADC_RUNNING_AVG_SIZE )
   {
       gPmBattData.TempReadingsIndex = 0;
   }

   // Divide the running sum by number of measurements taken to get the average
   gPmBattData.RunningTempAvg = gPmBattData.RunningTempSum >> ADC_RUNNING_AVG_SHIFT;

   // Turn off the charging if the battery temperature is too high or too low
   if ((gPmBattData.RunningTempAvg >= gPmBattData.TempHighLimit) || (gPmBattData.RunningTempAvg <= gPmBattData.TempLowLimit))
   {
       PM_DEBUG(DBG_TRACE,("pmbatt: turning charger off due to temperature condition with battery\n"));

       PMU_charger_stop(gPmBattData.pmu, gPmBattData.chargerId);
   }

   return gPmBattData.RunningTempAvg;
}

/****************************************************************************
*
*  pm_battMgrCalcBatteryLevel
*
*  returns: battery level number
*
***************************************************************************/
static int pm_battMgrCalcBatteryLevel( int VoltAvg )
{
    int i;
    int new_level = 0;
    int in_prev_range = 0;

    //Check if current battery voltage is still in the same range
    if( gPmBattData.status->battLevel == 0)
    {
        /* currently at lowest battery level */
        if ( VoltAvg < ( BatteryLevelTable[0] + BATT_HYSTERESIS + BATT_HYSTERESIS_DELTA ) )
        {
            in_prev_range = 1;
        }
    }
    else if ( gPmBattData.status->battLevel == PM_BATT_MAX_LEVELS)
    {
        /* currently at highest battery level */
        if ( VoltAvg > ( BatteryLevelTable[PM_BATT_MAX_LEVELS - 1] - BATT_HYSTERESIS) )
        {
            in_prev_range = 1;
        }
    }
    else
    {
        /* currently in a middle range; neither highest nor lowest */
        if (( VoltAvg > ( BatteryLevelTable[gPmBattData.status->battLevel - 1] - BATT_HYSTERESIS )) &&
            ( VoltAvg < ( BatteryLevelTable[gPmBattData.status->battLevel ] + BATT_HYSTERESIS + BATT_HYSTERESIS_DELTA )))
        {
            in_prev_range = 1;
        }
    }

    // If the Battlevel changes to new range,  A new battery level will be calculated
    if( !in_prev_range )
    {
        /* battery voltage has changed appreciably, so calculate new level */
        if( VoltAvg <= BatteryLevelTable[0] )
        {
            /* lowest battery level */
            new_level = 0;
        }
        else if( VoltAvg >= BatteryLevelTable[ PM_BATT_MAX_LEVELS -1 ] )
        {
            /* highest battery level */
            new_level = PM_BATT_MAX_LEVELS;
        }
        else
        {
            /* neither highest nor lowest level, quantize the level */
            for( i = 0; i < (PM_BATT_MAX_LEVELS - 1); i++ )
            {
                if(( VoltAvg >  BatteryLevelTable[i] ) &&
                   ( VoltAvg <= BatteryLevelTable[i + 1] ))
                {
                    /* found the level, break out of loop */
                    new_level = i + 1;
                    break;
                }
            }
        }
        /* update the battery level */
        gPmBattData.status->battLevel = new_level;

        PM_DEBUG(DBG_TRACE,("pmbatt - battery level message, level = %d\n",gPmBattData.status->battLevel));
        pm_status_event();
    }
    else
    {
        /* battery voltage has not changed appreciably */
        new_level = gPmBattData.status->battLevel;
    }

    return new_level;
}

/****************************************************************************
*
*  pm_battBeginFastCharge
*
*  Begin the fast charging process.
*
***************************************************************************/
static void pm_battBeginFastCharge( void )
{
   PM_DEBUG(DBG_INFO,("pmbatt - Starting fast charge of battery.\n"));

   PMU_charger_start(gPmBattData.pmu, gPmBattData.chargerId);

   gPmBattData.fastChargeOn = 1;
   gPmBattData.fastChargeTimer = 0;
}

/****************************************************************************
*
*  pm_battMgrTask
*
*  A periodic task that performs the battery managment duties.
*
***************************************************************************/
void pm_battMgrTask( void )
{
   int battVoltage;
   int tempVoltage;
   int battLevel;
   static int counter = 0;

   /* get the battery voltage */
   battVoltage = pm_battMgrGetVoltage();
   if ( battVoltage < 0 )
   {
      PM_DEBUG(DBG_ERROR,("pmbatt  - error getting battery voltage\n"));
      return;
   }

   //PM_DEBUG(DBG_TRACE, ("pmbatt - batt voltage = %d\n", battVoltage));

   if (++counter == 10)
   {
      PM_DEBUG(DBG_TRACE,("pmuState - info, battVoltage = %d battlevel = %d chgPluggedIn = %d chgState = %d\n",
                        battVoltage, gPmBattData.status->battLevel,gPmBattData.status->chgPluggedIn, gPmBattData.status->chgState));
      PM_DEBUG(DBG_TRACE,("pmuData - info, volt = %d temp = %d fastChargeOn = %d fastChargeTimer =%d \n",
                        gPmBattData.RunningVoltAvg, gPmBattData.RunningTempAvg, gPmBattData.fastChargeOn, gPmBattData.fastChargeTimer));

      counter = 0;

      // Run interval timed pmu specific code, if any
      PMU_run(gPmBattData.pmu);
   }

   /* get the temperature if in fast charge mode */
   if ( gPmBattData.fastChargeOn )
   {
      tempVoltage = pm_battMgrGetTemperature();
      if ( tempVoltage < 0 )
      {
         PM_DEBUG(DBG_ERROR,("pmu - error getting battery temperature\n"));
         return;
      }
      //PM_DEBUG(DBG_TRACE2, ("pmbatt - batt temperature = %d\n",tempVoltage));
   }

   /* calculate the battery gauge level */
   battLevel = pm_battMgrCalcBatteryLevel( battVoltage );
   //PM_DEBUG(DBG_TRACE2, ("pmbatt - batt level = %d\n",battLevel));

   /* start charging if battery is low, charger is plugged in, and not already charging */
   if (( battVoltage < RECHARGE_THRESH ) && gPmBattData.status->chgPluggedIn && !gPmBattData.fastChargeOn)
   {
      pm_battBeginFastCharge();
   }

   /* decrement the fast charge re-start timer and re-start when expired */
   if ( gPmBattData.fastChargeTimer > 0 )
   {
      if (( --gPmBattData.fastChargeTimer == 0 ) && ( gPmBattData.status->chgPluggedIn ))
      {
         pm_battBeginFastCharge();
      }
   }

   // For some phones, if the battery level falls below 1 bar, then
   // power off the phone and force the use of a charger. This prevents
   // lockups and inability to startup or reprogram flash
   if ( gPmBattData.earlyShutoff && ( battVoltage < BatteryLevelTable[0] ) &&
        !gPmBattData.status->chgPluggedIn )
   {
      PM_DEBUG(DBG_ERROR,("Low battery, powering off now. Please insert wall charger.\n"));
      pm_battMgrTurnOff();
   }

   // Reschedule battery manager task every 2 seconds.
   pm_submit_delayed_event(PM_EVENT_BATTMGRTASK, msecs_to_jiffies(2000), 0, 0, 0);
}

/****************************************************************************
*
*  pm_battMgrTurnOff
*
*  Go into the off state.  If the charger is unplugged, then the pmu chip
*  is put into standby mode.  Otherwise, the 116x will reset and pmud will
*  enter charge only state.
*
***************************************************************************/
void pm_battMgrTurnOff( void )
{
   /* TODO: Properly shutdown Linux */

   /* go to standby */
   PMU_poweroff(gPmBattData.pmu);
   /* wait to die */
   while(1);
}


/****************************************************************************
*
*  pm_battMgrChargerInserted
*
***************************************************************************/
void pm_battMgrChargerInserted( int chargerId )
{
   if (gPmBattData.status->chgPluggedIn)
   {
      PM_DEBUG(DBG_INFO,("pmbatt - Charger %d already inserted.\n", gPmBattData.chargerId));
      return;
   }

   /* charger present, and voltage okay */
   gPmBattData.status->chgPluggedIn = 1;
   gPmBattData.status->chgId = chargerId;

   /* save charger ID */
   gPmBattData.chargerId = chargerId;

   /* reset the voltage measurement averages */
   gPmBattData.StartTempAvg = 0;
   gPmBattData.StartVoltAvg = 0;

   /* Start the fast charging process. */
   pm_battBeginFastCharge();

   /* update status showing charger is plugged in */
   pm_status_event();
   gPmBattData.fastChargeTimer = 0;
}


/****************************************************************************
*
*  pm_battMgrChargerRemoved
*
***************************************************************************/
void pm_battMgrChargerRemoved( int chargerId )
{
   if (!gPmBattData.status->chgPluggedIn)
   {
      PM_DEBUG(DBG_INFO,("pmbatt - No previous charger inserted.\n"));
      return;
   }
   if (gPmBattData.chargerId != chargerId)
   {
      PM_DEBUG(DBG_INFO,("pmbatt - Previous charger inserted was %d, not %d.\n",
                         gPmBattData.chargerId, chargerId));
      return;
   }

   /* charger no longer plugged in */
   gPmBattData.status->chgPluggedIn = 0;
   gPmBattData.fastChargeOn = 0;
   gPmBattData.fastChargeTimer = 0;
   gPmBattData.status->chgState = 0;

   PM_DEBUG(DBG_INFO,("pmbatt - Charger was removed.\n"));
   pm_status_event();
}


/**************************************************************************** *
*  pm_battMgrBatteryLow
*
*  Processing for battery low condition.
*
***************************************************************************/
void pm_battMgrBatteryLow( void )
{
   pm_battMgrTurnOff();
}


/**************************************************************************** *
*  pm_battMgrBatteryFull
*
*  Processing for battery charge completion.
*
***************************************************************************/
void pm_battMgrBatteryFull( void )
{
   /* Battery is fully charged */
   gPmBattData.status->battLevel = PM_BATT_MAX_LEVELS;
   gPmBattData.status->chgState = 1;

   PM_DEBUG(DBG_INFO, ("pmbatt - charging is complete\n"));
   PM_DEBUG(DBG_TRACE,("pmbatt - battery level message, level = %d\n",gPmBattData.status->battLevel));

   PMU_charger_stop(gPmBattData.pmu, gPmBattData.chargerId);

   gPmBattData.fastChargeOn = 0;
   gPmBattData.fastChargeTimer = CHARGE_RESTART_TIME;
   pm_status_event();
}

    
/**************************************************************************** *
*  pm_battMgrLogLevel
*
*  Set debugging log level.
*
***************************************************************************/
void pm_battMgrLogLevel( int logLevel )
{
   gPmBattData.logLevel = logLevel;
}

/**************************************************************************** *
*  pm_battMgrInitTable
*
*  Set the battery voltage tables and temperature thresholds.
*  Called from Power Manager to set platform specific data
*
***************************************************************************/
void pm_battMgrInitVoltTable(PM_Battery_Level* table, int temp_high_limit,
                              int temp_low_limit)
{
   int i;
   if (table != NULL) {
      for (i=0; i<PM_MAX_BATTERY_LEVELS; i++)
      {
         if (table->adc_output != 0)
            BatteryLevelTable[i] = table->adc_output;
         table++;
      }
   }
   if (temp_high_limit != 0)
      gPmBattData.TempHighLimit = temp_high_limit; 
   else
      gPmBattData.TempHighLimit = TEMP_HIGH_LIMIT;
   if (temp_low_limit != 0)
      gPmBattData.TempLowLimit = temp_low_limit;
   else 
      gPmBattData.TempLowLimit = TEMP_LOW_LIMIT;
}
