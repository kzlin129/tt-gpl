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
*  PowerManager.c
*
*  PURPOSE:
*
*     Kernel Mode Power management control for the BCM116x, including battery charging,
*     battery level monitoring, and power supply control.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pm_platforms.h>
#include <linux/broadcom/pm_battmgr.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/cpu_sleep.h>
#include <linux/broadcom/timer.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/broadcom/bcm_adc.h>

#include <asm/arch/hw_cfg.h>


/* ---- Public Variables ------------------------------------------------- */
extern bcm_adc_request_t* bcm59040_adc_current_req;
extern bcm_adc_handler_t bcm59040_adc_completion_handler;

extern bcm_evt_handler_t bcm476x_rtc_update_handler;

// @KP: 090922
extern bcm_evt_handler_t bcm476x_usb_event_handler;

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
//#define DBG_DEFAULT_LEVEL  (DBG_ERROR | DBG_INFO | DBG_TRACE)

#if DEBUG
#   define PM_DEBUG(level,fmt,args...) do { if (level & gPmData.logLevel) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#else
#   define PM_DEBUG(level,fmt,args...)
#endif


#define PM_NAME "powermanager"


/*
 * Work queue data structures
 */
typedef struct
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
   struct work_struct work;
#else
   struct delayed_work work;
#endif
   int synchronous;
   struct completion completion;
   PM_Event event;
   uint32_t data1;
   uint32_t data2;
   uint32_t data3;
} PM_Work;

/*
 * Internal Events
 */
typedef enum
{
   PM_REGISTER_COMPONENT = 0,
   PM_UNREGISTER_COMPONENT,
   PM_UPDATE_SYSTEM,
   PM_RELEASE_SLEEP_OVERRIDE,
} PM_Internal_Events;

/*
 * Component registry
 */
typedef struct
{
   int registered;               // Component regsistration flag
   PM_CompPowerLevel powerLevel; // Current power level
   PM_Comp_Ops_t *ops;           // Operations table
} PM_Component_Reg_Entry;

//
// User Event Queue
//
#define PM_USER_EVENT_QUEUE_LEN 16

/* structure to hold driver state */
typedef struct
{
   int            logLevel;            // Logging level
   int            battMgrLogLevel;     // Logging level for battery manager
   int            pmuLogLevel;         // Logging level for pmu chips
   PM_Status      status;     // Current status
   PM_PowerLevel  powerLevel; // Current power level

   unsigned int   maxWorkDuration; // Profiling counter

   int            pmuChipInit;     // PMU Chip initialization flag
   BCM_PMU_Operations_t *pmu;  // PMU Chip handle
   int            devReg;          // Device node registration flag

   struct ctl_table_header *sysCtlHeader; // sysctl table

   struct workqueue_struct *workQueue;    // Work Queue

   PM_Platform *platform;                 // Detected platform
   char platformName[PM_PLATFORM_NAME_MAX_LEN];     // Name of detected platform

   PM_Component_Reg_Entry components[PM_NUM_COMPONENTS];  // Components registry

   PM_Component_Bitmap offComponents; // Bitmap cache of components that are off

   wait_queue_head_t pollQueue; // Poll Queue

   // User event queue
   struct semaphore userEventSema;
   PM_UserEvent userEventQueue[PM_USER_EVENT_QUEUE_LEN];
   int userEventGetIndex;
   int userEventPutIndex;
   int numUserEvents;

} PM_DATA;


/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "Power Manager: 1.00 (built on "__DATE__" "__TIME__")\n";
static PM_DATA gPmData =
{
   logLevel: DBG_DEFAULT_LEVEL,
   battMgrLogLevel: DBG_DEFAULT_LEVEL,
   pmuLogLevel: DBG_DEFAULT_LEVEL,
   status:
   {
      battLevel:     0,
      chgPluggedIn:  0,
      chgState:      0,
      chgId:         0,
   },
   powerLevel:    PM_PWR_OFF,
   pmuChipInit:   0,
   pmu:           NULL,
   devReg:        0,
   workQueue:     NULL,
   platform:      NULL,
   platformName:  "Unknown",
   userEventGetIndex: 0,
   userEventPutIndex: 0,
   numUserEvents: 0,
   maxWorkDuration: 0,
};

// Lookup tables for symbolic printing,
// indexed byt the events themselves.
// Not done with a val/name lookup for
// simplicity and speed.
static const char * PM_PowerLevelTable[] =
{
   "PM_PWR_OFF",              // Table must match PM_PowerLevel enum
   "PM_PWR_CHARGING",
   "PM_PWR_STANDBY",
   "PM_PWR_HIGH",
   "PM_PWR_FULL"
};
static const char * PM_ComponentTable[] =
{
   "PM_COMP_LCD",             // Table must match PM_Component enum
   "PM_COMP_CAMERA",
   "PM_COMP_USB",
   "PM_COMP_ETHERNET",
   "PM_COMP_WIFI",
   "PM_COMP_AUDIO",
   "PM_COMP_BLUETOOTH",
   "PM_COMP_SIM"
};
static const char * PM_CompPowerLevelTable[] =
{
   "PM_COMP_PWR_OFF",         // Table must match PM_CompPowerLevel enum
   "PM_COMP_PWR_STANDBY",
   "PM_COMP_PWR_ON"
};

static const char * PMU_EventTable[] =
{
   "PMU_EVENT_ATTACHED",                     // Table must match BCM1160_PMU_Event_t enum
   "PMU_EVENT_BATTERY_LOW",
   "PMU_EVENT_BATTERY_FULL",
   "PMU_EVENT_BATTERY_TEMPERATURE_FAULT",
   "PMU_EVENT_BATTERY_TEMPERATURE_OK",
   "PMU_EVENT_ONKEY_RISE",
   "PMU_EVENT_ONKEY_FALL",
   "PMU_EVENT_ONKEY_1S_HOLD",
   "PMU_EVENT_HIGH_TEMPERATURE",
   "PMU_EVENT_CHARGER_INSERT",
   "PMU_EVENT_CHARGER_REMOVE",
   "PMU_EVENT_CHARGER_ERROR",
   "PMU_EVENT_RTC1S",
   "PMU_EVENT_RTC60S",
   "PMU_EVENT_RTCA1"
};

//
// sysctl
//
static int shadow_charging_only;
static int pm_proc_charging_only(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );
static int pm_proc_battmgr_logLevel(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );
static int pm_proc_pmu_logLevel(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );

#if defined( CONFIG_BCM_SLEEP_MODE )
static int shadow_cpu_sleep_override;
static int pm_proc_cpu_sleep_override(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );
#endif

static struct ctl_table gSysCtlLocalStatus[] = {
   {
      .ctl_name         = 1,
      .procname         = "battLevel",
      .data             = &gPmData.status.battLevel,
      .maxlen           = sizeof( int ),
      .mode             = 0444,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 2,
      .procname         = "chgPluggedIn",
      .data             = &gPmData.status.chgPluggedIn,
      .maxlen           = sizeof( int ),
      .mode             = 0444,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 3,
      .procname         = "chgState",
      .data             = &gPmData.status.chgState,
      .maxlen           = sizeof( int ),
      .mode             = 0444,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 4,
      .procname         = "chgId",
      .data             = &gPmData.status.chgId,
      .maxlen           = sizeof( int ),
      .mode             = 0444,
      .proc_handler     = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtlLocal[] = {
   {
      .ctl_name         = 1,
      .procname         = "platform",
      .data             = &gPmData.platformName,
      .maxlen           = sizeof(gPmData.platformName),
      .mode             = 0444,
      .proc_handler     = &proc_dostring
   },
   {
      .ctl_name         = 2,
      .procname         = "logLevel",
      .data             = &gPmData.logLevel,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 3,
      .procname         = "status",
      .child            = gSysCtlLocalStatus,
      .mode             = 0555
   },
   {
      .ctl_name         = 4,
      .procname         = "powerLevel",
      .data             = &gPmData.powerLevel,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 5,
      .procname         = "chargingOnly",
      .data             = &shadow_charging_only,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &pm_proc_charging_only
   },
   {
      .ctl_name         = 6,
      .procname         = "maxWorkDuration",
      .data             = &gPmData.maxWorkDuration,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
#if defined( CONFIG_BCM_SLEEP_MODE )
   {
      .ctl_name         = 7,
      .procname         = "sleepEnable",
      .data             = &cpu_sleep_enable,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = 8,
      .procname         = "sleepOverride",
      .data             = &shadow_cpu_sleep_override,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &pm_proc_cpu_sleep_override
   },
#endif
   {
      .ctl_name         = 9,
      .procname         = "battMgrLogLevel",
      .data             = &gPmData.battMgrLogLevel,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &pm_proc_battmgr_logLevel
   },
   {
      .ctl_name         = 10,
      .procname         = "pmuLogLevel",
      .data             = &gPmData.pmuLogLevel,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &pm_proc_pmu_logLevel
   },
   {}
};
static ctl_table gSysCtl[] = {
	{
		.ctl_name	= CTL_BCM_PM,
		.procname	= PM_NAME,
		.mode		   = 0555,
		.child		= gSysCtlLocal
	},
	{}
};


/* ---- Private Function Prototypes -------------------------------------- */

const static PM_Component_Map * pm_find_component_in_platform(PM_Component id);
static void pm_set_regulators_state(const PM_Component_Map *map, PM_CompPowerLevel level);
static void pm_update_component(PM_Component id);
static void pm_update_system(PM_PowerLevel powerLevel);
static void pm_remove_component(PM_Component id);
static void pm_cleanup( void );

static void pm_queue_user_event( PM_UserEvent event );
static int pm_dequeue_user_event( PM_UserEvent *event );

/* ---- Functions -------------------------------------------------------- */

#if defined( CONFIG_BCM_SLEEP_MODE )
/****************************************************************************
*
*  pm_proc_cpu_sleep_override
*
***************************************************************************/
static int pm_proc_cpu_sleep_override(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
       return -EINVAL;

   if ( write )
   {
      // use generic int handler to get input value
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

      if (rc < 0)
         return rc;

      // increment override count if input is 1, decrement if 0
      if (shadow_cpu_sleep_override == 1)
      {
         atomic_inc(&cpu_sleep_proc_override);
      }
      else if (shadow_cpu_sleep_override == 0)
      {
         if ( atomic_read(&cpu_sleep_proc_override) > 0 )
         {
            atomic_dec(&cpu_sleep_proc_override);
         }
         else
         {
            /* Decrementing sleepOverride to negative value causes unstable state */
            printk("Preventing unbalanced decrement of sleepOverride\n");
         }
      }
      return rc;
   }
   else
   {
      // nothing special for read, just update the shadow
      // variable and use generic int handler */
      //printk("cpu_sleep_override =%d, cpu_sleep_proc_override = %d\n", atomic_read(&cpu_sleep_override), atomic_read(&cpu_sleep_proc_override));
      shadow_cpu_sleep_override = atomic_read(&cpu_sleep_override) + atomic_read(&cpu_sleep_proc_override);
      return proc_dointvec( table, write, filp, buffer, lenp, ppos );
   }
}

#endif

/****************************************************************************
*
*  pm_proc_charging_only
*
***************************************************************************/
static int pm_proc_charging_only(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
       return -EINVAL;

   if ( write )
   {
      // use generic int handler to get input value
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

      if (rc < 0)
         return rc;

      // only allow input of 0 to leave charging only mode
      if ( (gPmData.powerLevel == PM_PWR_CHARGING) && (shadow_charging_only == 0) )
      {
         printk("Taking platform out of charging only mode\n");
         pm_submit_event_and_wait(PM_EVENT_INTERNAL, PM_UPDATE_SYSTEM, (uint32_t)PM_PWR_STANDBY, 0);
      }
      return rc;
   }
   else
   {
      // nothing special for read, just update the shadow
      // variable and use generic int handler */
      shadow_charging_only = (gPmData.powerLevel == PM_PWR_CHARGING) ? 1 : 0;
      return proc_dointvec( table, write, filp, buffer, lenp, ppos );
   }
}

/****************************************************************************
*
*  pm_proc_battmgr_logLevel
*
***************************************************************************/
static int pm_proc_battmgr_logLevel(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
       return -EINVAL;

   if ( write )
   {
      // use generic int handler to get input value
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
      pm_battMgrLogLevel(gPmData.battMgrLogLevel);
      return rc;
   }
   else
   {
      return proc_dointvec( table, write, filp, buffer, lenp, ppos );
   }
}
/****************************************************************************
*
*  pm_proc_pmu_logLevel
*
***************************************************************************/
static int pm_proc_pmu_logLevel(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
       return -EINVAL;

   if ( write )
   {
      // use generic int handler to get input value
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
      if (gPmData.pmu)
      {
         // In case not attached due to i2c error.
         PMU_logLevel(gPmData.pmu, gPmData.pmuLogLevel);
      }
      return rc;
   }
   else
   {
      return proc_dointvec( table, write, filp, buffer, lenp, ppos );
   }
}
/****************************************************************************
*
*  pm_detect_platform
*
***************************************************************************/
void __init pm_register_platform( const PM_Platform *platform )
{
   static int __initdata detected = 0;

   printk( KERN_INFO "Platform '%s' registered.\n", platform->name );
   if ( !detected )
   {
      // If no detection routine is supplied with the platform, assume it
      // is the only platform and use its definition.
      if (!platform->detect)
      {
         printk( KERN_INFO "Platform '%s' registered without detection routine.\n", platform->name );
         printk( KERN_INFO "Assuming running on platform '%s'.\n", platform->name );
         detected =  1;
      }
      else if (platform->detect())
      {
         printk( KERN_INFO "Platform '%s' detected.\n", platform->name );
         detected =  1;
      }
   }

   if ( detected )
   {
      // We found a match
      if ( gPmData.platform != NULL )
      {
         printk( KERN_ERR "*****\n" );
         printk( KERN_ERR "***** pm_register_platform: Duplicate platform (%s) detected (ignored)\n", platform->name );
         printk( KERN_ERR "*****\n" );
      }
      else
      {
         // Since the data being passed in is initdata, we allocate some memory
         // to copy it into.
         gPmData.platform = kmalloc( sizeof( *gPmData.platform ), GFP_KERNEL );
         *gPmData.platform = *platform;
      }
   }
}

/****************************************************************************
*
*  pm_ioctl
*
***************************************************************************/

static int pm_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    PM_DEBUG(DBG_INFO, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    switch ( cmd )
    {
        case PM_IOCTL_SETPOWER:
        {
            PM_PowerLevel powerLevel;
            if ( copy_from_user( &powerLevel, (PM_PowerLevel *)arg, sizeof( PM_PowerLevel )) != 0 )
            {
                printk("PM_IOCTL_SETPOWER copy_from_user() failed\n");
                return -EFAULT;
            }
    	    PM_DEBUG(DBG_INFO, "PM_IOCTL_SETPOWER: %s (%d)\n", PM_PowerLevelTable[powerLevel], powerLevel);
            pm_submit_event_and_wait(PM_EVENT_INTERNAL, PM_UPDATE_SYSTEM, (uint32_t)powerLevel, 0);
            break;
        }

        case PM_IOCTL_GETPOWER:
        {
            if ( copy_to_user( (PM_PowerLevel *)arg, &gPmData.powerLevel, sizeof( PM_PowerLevel )) != 0 )
            {
                printk("PM_IOCTL_GETPOWER copy_to_user() failed\n");
                return -EFAULT;
            }
    	    PM_DEBUG(DBG_INFO, "PM_IOCTL_GETPOWER: %s (%d)\n", PM_PowerLevelTable[gPmData.powerLevel], gPmData.powerLevel);
            break;
        }

        case PM_IOCTL_GETSTATUS:
        {
            if ( copy_to_user( (PM_Status *)arg, &gPmData.status, sizeof( PM_Status )) != 0 )
            {
                printk("PM_IOCTL_GETSTATUS copy_to_user() failed\n");
                return -EFAULT;
            }
    	    PM_DEBUG(DBG_INFO, "PM_IOCTL_GETSTATUS: battLevel=%d, chgPluggedIn=%d, chgState=%d, chgId=%d\n",
      				gPmData.status.battLevel, gPmData.status.chgPluggedIn, gPmData.status.chgState, gPmData.status.chgId);
            break;
        }

        case PM_IOCTL_GETEVENT:
        {
            PM_UserEvent event;
            if (pm_dequeue_user_event(&event) < 0)
            {
                printk("PM_IOCTL_GETEVENT no user event available\n");
                return -EFAULT;
            }
            if ( copy_to_user( (PM_Status *)arg, &event, sizeof( PM_UserEvent )) != 0 )
            {
                printk("PM_IOCTL_GETEVENT copy_to_user() failed\n");
                return -EFAULT;
            }
    	    PM_DEBUG(DBG_INFO, "PM_IOCTL_GETEVENT: %d\n", event);
            break;
        }

        default:
        {
            PM_DEBUG(DBG_ERROR, "Unrecognized ioctl: '0x%x'\n", cmd );
            return -ENOTTY;
        }
    }

    return 0;

} /* pm_ioctl */


/****************************************************************************
*
* pm_poll
*
* used to support the select system call
*
***************************************************************************/
static unsigned int pm_poll( struct file *file, struct poll_table_struct *poll_table )
{
    poll_wait( file, &gPmData.pollQueue, poll_table );

    // Indicate that data is currently available
    if (gPmData.numUserEvents)
    {
       return POLLIN | POLLRDNORM;
    }

    return 0;
} /* pm_poll */


/****************************************************************************
*
*  pm_open
*
***************************************************************************/
static int pm_open( struct inode *inode, struct file *file )
{
    PM_DEBUG(DBG_TRACE, "called\n" );

    return 0;

} /* pm_open */


/****************************************************************************
*
*  pm_release
*
***************************************************************************/
static int pm_release( struct inode *inode, struct file *file )
{
    PM_DEBUG(DBG_TRACE, "called\n" );
    return 0;

} /* pm_release */


/****************************************************************************
*
*  pm_local_power_off
*
***************************************************************************/
static void pm_local_power_off( void )
{
   if(gPmData.pmu)
   {
      PMU_poweroff(gPmData.pmu);
   }
}


/****************************************************************************
*
*  pm_find_component_in_platform
*
***************************************************************************/
const static PM_Component_Map * pm_find_component_in_platform(PM_Component id)
{
   int i;

   // Look for component in platform
   for (i = 0; i < PM_NUM_COMPONENTS; i++)
   {
      if (gPmData.platform->components[i].id == id)
         return &gPmData.platform->components[i];

      if (gPmData.platform->components[i].id == PM_NUM_COMPONENTS)
         return NULL;
   }
   return NULL;
}

/****************************************************************************
*
*  pm_set_regulators_state
*
***************************************************************************/
static void pm_set_regulators_state(const PM_Component_Map *map, PM_CompPowerLevel level)
{
   int i;
   const PM_Regulator_Map *regulator;
   BCM_PMU_Regulator_State_t regulator_state;

   regulator_state = ((level == PM_COMP_PWR_OFF) ? PMU_Regulator_Eco : PMU_Regulator_On);
   // Find all the regulators connected to this component
   for (i = 0; i < PM_MAX_REGULATORS_PER_PLATFORM; i++)
   {
      regulator = &gPmData.platform->regulators[i];

      if (regulator->id == -1)
         break;

      PM_DEBUG(DBG_TRACE2, "is %s (%d) <--> %s (%d) (bitmap 0x%08x)\n",
                          PM_ComponentTable[map->id], map->id, regulator->name, regulator->id, regulator->connected_components);
      if (IS_COMPONENT_SET(regulator->connected_components,map->id))
      {
         PM_DEBUG(DBG_INFO, "%s (%d) <--> %s (%d)\n",
                             PM_ComponentTable[map->id], map->id, regulator->name, regulator->id);
         // When turning off regulator, make sure all components connected
         // to this regulator are off.
         if ((level == PM_COMP_PWR_OFF) &&
             !IS_COMPONENT_SUBSET(gPmData.offComponents, regulator->connected_components))
         {
            PM_DEBUG(DBG_INFO, "not all comp's connected to %s (%d) are off (0x%08x, 0x%08x).\n",
                                regulator->name, regulator->id, gPmData.offComponents, regulator->connected_components);
            continue;
         }
         PMU_regulator_set_state(gPmData.pmu, regulator->id, regulator_state);
      }
   }
}


/****************************************************************************
*
*  pm_update_component
*
***************************************************************************/
static void pm_update_component(PM_Component id)
{
   PM_Component_Reg_Entry *reg_entry = &gPmData.components[id];
   const PM_Component_Map *map;
   PM_CompPowerLevel compPowerLevel;
   int powerLevelChanged = 0;

   // Don't update the component until the PMU chip is ready
   if (!gPmData.pmu)
   {
      PM_DEBUG(DBG_INFO, "pmu not attached yet.\n");
      return;
   }

   if (!reg_entry->registered)
   {
      return;
   }

   // Map system power level to component power level
   map = pm_find_component_in_platform(id);
   if (!map)
   {
      return;
   }
   compPowerLevel = map->compPowerLevel[gPmData.powerLevel];
   if (compPowerLevel != reg_entry->powerLevel)
   {
      powerLevelChanged = 1;
      reg_entry->powerLevel = compPowerLevel;
      if (compPowerLevel == PM_COMP_PWR_OFF)
      {
         SET_COMPONENT(gPmData.offComponents,id);
      }
      else
      {
         CLEAR_COMPONENT(gPmData.offComponents,id);
      }
   }

   // Turn on regulators if necessary
   if (powerLevelChanged && (compPowerLevel != PM_COMP_PWR_OFF))
   {
      pm_set_regulators_state(map, compPowerLevel);
      if (map->extra_action)
         map->extra_action(compPowerLevel);
   }

   // Notify component via callback
   PM_DEBUG(DBG_INFO, "%s (%d) --> %s (%d)\n",
      PM_ComponentTable[id],
      id,
      PM_CompPowerLevelTable[reg_entry->powerLevel],
      reg_entry->powerLevel);

   reg_entry->ops->update_power_level(reg_entry->powerLevel, gPmData.powerLevel);

   // Turn off regulators if necessary
   if (powerLevelChanged && (compPowerLevel == PM_COMP_PWR_OFF))
   {
      pm_set_regulators_state(map, compPowerLevel);
      if (map->extra_action)
         map->extra_action(compPowerLevel);
   }
}


/****************************************************************************
*
*  pm_remove_component
*
***************************************************************************/
static void pm_remove_component(PM_Component id)
{
   PM_Component_Reg_Entry *reg_entry = &gPmData.components[id];
   const PM_Component_Map *map;

   // Don't update the component until the PMU chip is ready
   if (!gPmData.pmu)
   {
      PM_DEBUG(DBG_INFO, "pmu not attached yet.\n");
      return;
   }

   if (!reg_entry->registered)
   {
      return;
   }

   // Find regulator connection map
   map = pm_find_component_in_platform(id);
   if (!map)
   {
      return;
   }

   // Turn off component
   if (reg_entry->powerLevel != PM_COMP_PWR_OFF)
   {
      SET_COMPONENT(gPmData.offComponents,id);
      reg_entry->powerLevel = PM_COMP_PWR_OFF;

      // Notify component via callback
      PM_DEBUG(DBG_INFO, "update component %d power level to %d.\n", id, reg_entry->powerLevel);
      reg_entry->ops->update_power_level(reg_entry->powerLevel, gPmData.powerLevel);

      // Turn off regulators if necessary
      pm_set_regulators_state(map, PM_COMP_PWR_OFF);
      if (map->extra_action)
         map->extra_action(PM_COMP_PWR_OFF);
   }
}


/****************************************************************************
*
*  pm_update_system
*
***************************************************************************/
static void pm_update_system(PM_PowerLevel powerLevel)
{
   int i;

   // Nothing to do if level hasn't changed
   if (powerLevel == gPmData.powerLevel)
      return;
   gPmData.powerLevel = powerLevel;

#ifdef CONFIG_BCM_SLEEP_MODE
   // Allow CPU sleep mode only when in standby or lower power states
   switch (gPmData.powerLevel)
   {
      case PM_PWR_OFF:
      case PM_PWR_CHARGING:
      case PM_PWR_STANDBY:
         cpu_sleep_enable = 1;
         break;
      case PM_PWR_HIGH:
      case PM_PWR_FULL:
      default:
         cpu_sleep_enable = 0;
         break;
   }
#endif

   // Update power levels of all components
   for (i = 0; i < PM_NUM_COMPONENTS; i++)
   {
      pm_update_component(i);
   }

   // Shutoff system if requested
   if (gPmData.powerLevel == PM_PWR_OFF)
   {
      // TBD
   }
}


/****************************************************************************
*
*  pm_handle_internal_event
*
***************************************************************************/
static void pm_handle_internal_event ( PM_Internal_Events event, uint32_t data, uint32_t data2 )
{
   switch (event)
   {
      case PM_REGISTER_COMPONENT:
         {
            PM_Component component = (PM_Component) data;
            PM_Comp_Ops_t *ops = (PM_Comp_Ops_t *) data2;

            if (gPmData.components[component].registered )
            {
               printk("PM_REGISTER_COMPONENT %s (%d) already registered.\n", PM_ComponentTable[component], component);
               return;
            }

            gPmData.components[component].ops = ops;
            gPmData.components[component].registered = 1;
            PM_DEBUG(DBG_INFO, "PM_REGISTER_COMPONENT %s (%d) registered.\n", PM_ComponentTable[component], component);

            pm_update_component(component);
         }
         break;

      case PM_UNREGISTER_COMPONENT:
         {
            PM_Component component = (PM_Component) data;
            wait_queue_head_t *unregister_wait_queue = (wait_queue_head_t *) data2;

            if (!gPmData.components[component].registered )
            {
               printk("pm: component %s (%d) not registered.\n", PM_ComponentTable[component], component);
            }
            else
            {
               // Remove component from system
               pm_remove_component(component);
               gPmData.components[component].registered = 0;
               gPmData.components[component].ops = NULL;
            }

            // Notify the caller the component has been removed
            PM_DEBUG(DBG_INFO, "component %s (%d) unregistered.\n", PM_ComponentTable[component], component);
            wake_up_interruptible( unregister_wait_queue );
         }
         break;

      case PM_UPDATE_SYSTEM:
         {
            PM_PowerLevel powerLevel = (PM_PowerLevel) data;

            PM_DEBUG(DBG_INFO, "PM_UPDATE_SYSTEM to %s\n", PM_PowerLevelTable[powerLevel]);
            pm_update_system(powerLevel);
         }
         break;

      case PM_RELEASE_SLEEP_OVERRIDE:
         {
#if defined( CONFIG_BCM_SLEEP_MODE )
            atomic_dec(&cpu_sleep_override);
#endif
         }
      default:
         break;
   }
}


/****************************************************************************
*
*  pm_init_system
*
***************************************************************************/
static void pm_init_system ( void )
{
   int i;

   // Perform extra initialization of platform
   if (gPmData.platform->extra_init)
      gPmData.platform->extra_init();

   // Initialize all regulators defined in platform
   for (i = 0; i < PM_MAX_REGULATORS_PER_PLATFORM; i++)
   {
      const PM_Regulator_Map *regulator = &gPmData.platform->regulators[i];

      if (regulator->id < 0)
         break;

      // Set initial voltage if specified
      if (regulator->mV > 0)
         PMU_regulator_set_voltage(gPmData.pmu, regulator->id, regulator->mV);

      PMU_regulator_set_state(gPmData.pmu, regulator->id, regulator->initial_state);
   }

   // Perform any extra initialization for each component in platform
   for (i = 0; i < PM_NUM_COMPONENTS; i++)
   {
      if (gPmData.platform->components[i].id == PM_NUM_COMPONENTS)
         break;

      if (gPmData.platform->components[i].extra_init)
         gPmData.platform->components[i].extra_init();
   }
}


/****************************************************************************
*
*  pm_handle_pmu_chip_event
*
***************************************************************************/
static void pm_handle_pmu_chip_event ( BCM_PMU_Event_t event, uint32_t data, uint32_t data3 )
{
   int statusUpdate = 0;
   PM_PowerLevel initialSystemLevel = PM_PWR_STANDBY;

   PM_DEBUG(DBG_TRACE, "%s\n", PMU_EventTable[event]);

   switch (event)
   {
      case PMU_EVENT_ATTACHED:
         gPmData.pmu = (BCM_PMU_Operations_t *)data;

         pm_init_system();

         if ( gPmData.platform->pmu_chip != PMU_NONE )
         {
            pm_battMgrInitVoltTable(gPmData.platform->batteryleveltable,
                           gPmData.platform->battery_temp_high_limit,
                           gPmData.platform->battery_temp_low_limit);
            pm_battMgrInit(gPmData.platform->battery_temperature_adc_channel,
                           gPmData.platform->battery_voltage_adc_channel,
                           gPmData.platform->battery_early_shutoff,
                           gPmData.pmu,
                           &gPmData.status);
            pm_battMgrLogLevel(gPmData.battMgrLogLevel);
            PMU_logLevel(gPmData.pmu, gPmData.pmuLogLevel);
         }
         else
         {
            // For NULL PMU, mark battery level and charger status as invalid
            gPmData.status.battLevel = -1;
            gPmData.status.chgState = -1;
            gPmData.status.chgPluggedIn = -1;
         }

         switch (PMU_get_power_on_state(gPmData.pmu))
         {
            case PMU_Power_On_By_On_Button:
               initialSystemLevel = PM_PWR_STANDBY;
               break;
            case PMU_Power_On_By_Charger:
               if ( gPmData.platform->battery_charging_only )
               {
                  printk("pm: enter charging only mode\n");
                  initialSystemLevel = PM_PWR_CHARGING;
               }
               else
               {
                  initialSystemLevel = PM_PWR_STANDBY;
               }
               break;
            case PMU_Power_On_By_Restart:
               initialSystemLevel = PM_PWR_STANDBY;
               break;
            default:
               break;
         }
         pm_update_system(initialSystemLevel);

         // Re-allow CPU sleep mode to be enbaled after 1 minute
         pm_submit_delayed_event(PM_EVENT_INTERNAL, msecs_to_jiffies(60 * 1000), PM_RELEASE_SLEEP_OVERRIDE, 0, 0);
         break;

      case PMU_EVENT_BATTERY_LOW:
         pm_battMgrBatteryLow();
         break;

      case PMU_EVENT_BATTERY_FULL:
         pm_battMgrBatteryFull();
         break;

      case PMU_EVENT_BATTERY_TEMPERATURE_FAULT:
      case PMU_EVENT_BATTERY_TEMPERATURE_OK:
      case PMU_EVENT_HIGH_TEMPERATURE:
         break;

      case PMU_EVENT_VBUS_VLS_R:
      case PMU_EVENT_B_SESS_END_F:
         // by default, dwc-core usb softdisconnect is enable; we use VBusValid as the first trigger event to delay 500 ms more
         // (to avoid conflict between dwc-core and BC_Detect both driving the same DP signal) before disabling softdisconnect 
         // to allow enumeration to take place;
         // we use bSessEnd to re-enable the dwc-core usb softdisconnect
         if (bcm476x_usb_event_handler != NULL)
            (*bcm476x_usb_event_handler)(data);
         break;

      case PMU_EVENT_ONKEY_RISE:
         break;
      case PMU_EVENT_ONKEY_FALL:
         pm_queue_user_event(PM_EVENT_ON_KEY_PRESSED);
         break;
      case PMU_EVENT_ONKEY_1S_HOLD:
         pm_queue_user_event(PM_EVENT_ON_KEY_HELD_FOR_1S);
         break;

      case PMU_EVENT_CHARGER_INSERT:
         pm_battMgrChargerInserted(data);
         break;
      case PMU_EVENT_CHARGER_REMOVE:
         pm_battMgrChargerRemoved(data);
         break;

      case PMU_EVENT_CHARGER_START:
         if(gPmData.pmu)
         {
             PMU_charger_start(gPmData.pmu, data);
         }
         break;

      case PMU_EVENT_CHARGER_STOP:
         if(gPmData.pmu)
         {
             PMU_charger_stop(gPmData.pmu, data);
         }
         break;

      case PMU_EVENT_CHARGER_SET_CURR_LMT:
         if(gPmData.pmu)
         {
             PMU_charger_set_current_limit(gPmData.pmu, data, data3);
         }
         break;

      case PMU_EVENT_SARCONVEND:   //ADC request succeed
         if (bcm59040_adc_completion_handler != NULL)
		 {
			 bcm_adc_request_t* req;

			 req = bcm59040_adc_current_req;
			 bcm59040_adc_current_req = NULL;

			 if ((*bcm59040_adc_completion_handler)(req, data, 0))
			 {
				 printk("PMU_EVENT_SARCONVEND: Invalid completion call\n");
			 }
		 }
         break;

      case PMU_EVENT_SARASYNREQFAIL:   //ADC request failed
		 if (bcm59040_adc_completion_handler != NULL)
		 {
			 bcm_adc_request_t* req;

			 req = bcm59040_adc_current_req;
			 bcm59040_adc_current_req = NULL;

			 if ((*bcm59040_adc_completion_handler)(req, 0, -1))
			 {
				 printk("PMU_EVENT_SARASYNREQFAIL: Invalid completion call\n");
			 }
		 }
         break;

      case  PMU_EVENT_RTC1S:
      case  PMU_EVENT_RTC60S:
      case  PMU_EVENT_RTCA1:
       	  if (bcm476x_rtc_update_handler != NULL)
              (*bcm476x_rtc_update_handler)(data);

    	  break;

      case PMU_EVENT_CHARGER_ERROR:
      default:
         break;
   }
   if (statusUpdate)
   {
      pm_status_event();
   }
}


/****************************************************************************
*
*  pm_enable_early_shutoff
*
***************************************************************************/
void pm_enable_early_shutoff( void)
{
   gPmData.platform->battery_early_shutoff = 1;
}


/****************************************************************************
*
*  pm_status_event
*
***************************************************************************/
void pm_status_event( void )
{
   pm_queue_user_event(PM_EVENT_STATUS_UPDATE);
}


/****************************************************************************
*
*  pm_queue_user_event
*
***************************************************************************/
static void pm_queue_user_event( PM_UserEvent event )
{
   down( &gPmData.userEventSema );

   gPmData.userEventQueue[ gPmData.userEventPutIndex++ ] = event;
   if ( gPmData.userEventPutIndex >= PM_USER_EVENT_QUEUE_LEN )
   {
      gPmData.userEventPutIndex = 0;
   }
   gPmData.numUserEvents++;

   up( &gPmData.userEventSema );
   wake_up_interruptible( &gPmData.pollQueue );
}


/****************************************************************************
*
*  pm_dequeue_user_event
*
***************************************************************************/
static int pm_dequeue_user_event( PM_UserEvent *event )
{
   down( &gPmData.userEventSema );

   if (gPmData.numUserEvents == 0)
   {
      up( &gPmData.userEventSema );
      return -1;
   }

   *event = gPmData.userEventQueue[ gPmData.userEventGetIndex++ ];
   if ( gPmData.userEventGetIndex >= PM_USER_EVENT_QUEUE_LEN )
   {
      gPmData.userEventGetIndex = 0;
   }
   gPmData.numUserEvents--;

   up( &gPmData.userEventSema );
   return 0;
}


/****************************************************************************
*
*  pm_do_work
*
***************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void pm_do_work ( void * data )
#else
void pm_do_work ( struct work_struct * data )
#endif
{
   u32 in_clk, duration;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
   PM_Work *work = (PM_Work *)data;
#else
   PM_Work *work = container_of(data, PM_Work, work.work);
#endif

   in_clk = timer_get_tick_count();
   switch (work->event)
   {
      case PM_EVENT_INTERNAL:
         PM_DEBUG(DBG_INFO, "PM_EVENT_INTERNAL (0x%08x, 0x%08x, 0x%08x)\n",
                            work->data1, work->data2, work->data3);
         pm_handle_internal_event(work->data1, work->data2, work->data3);
         break;

      case PM_EVENT_PMU_CHIP:
         PM_DEBUG(DBG_INFO, "PM_EVENT_PMU_CHIP (0x%08x, 0x%08x, 0x%08x)\n",
                            work->data1, work->data2, work->data3);
         pm_handle_pmu_chip_event(work->data1, work->data2, work->data3);
         break;

      case PM_EVENT_BATTMGRTASK:
         pm_battMgrTask();
         break;

      default:
         PM_DEBUG( DBG_ERROR, "Unknown event %d (0x%08x, 0x%08x, 0x%08x)\n",
                work->event, work->data1, work->data2, work->data3);
         break;
   }

   duration = timer_get_tick_count() - in_clk;
   if (duration > gPmData.maxWorkDuration)
   {
      gPmData.maxWorkDuration = duration;
   }

   if (work->synchronous)
   {
      complete(&work->completion);
   }
   else
   {
      /* any risk? leo */
      kfree(data);
   }
} /* pm_do_work */


/****************************************************************************
*
*  pm_submit_event
*
***************************************************************************/
int pm_submit_event( PM_Event event, uint32_t data1, uint32_t data2, uint32_t data3 )
{
   PM_Work *work;

   if (!gPmData.workQueue)
   {
      panic("pm: Work Queue not initialized yet.\n"
            "    Make sure the Power Manager is linked in before other drivers.\n");
      return -ENODEV;
   }

   work = kmalloc(sizeof(PM_Work), GFP_KERNEL);
   if (!work)
   {
      printk("pm: Failed to allocate memory for work.\n");
      return -ENOMEM;
   }

   work->synchronous = 0;
   work->event = event;
   work->data1 = data1;
   work->data2 = data2;
   work->data3 = data3;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
   INIT_WORK( &(work->work), pm_do_work, (void *)work );
   if ( queue_work( gPmData.workQueue, &(work->work) ) == 0 )
#else
   INIT_DELAYED_WORK( &(work->work), pm_do_work );
   if ( queue_delayed_work( gPmData.workQueue, &(work->work), 0 ) == 0 )
#endif
   {
      printk("pm: Work previously queued.\n");
      return -EINVAL;
   }

   return 0;
} /* pm_submit_event */


/****************************************************************************
*
*  pm_submit_event_and_wait
*
***************************************************************************/
int pm_submit_event_and_wait( PM_Event event, uint32_t data1, uint32_t data2, uint32_t data3 )
{
   PM_Work *work;

   if (!gPmData.workQueue)
   {
      panic("pm: Work Queue not initialized yet.\n"
            "    Make sure the Power Manager is linked in before other drivers.\n");
      return -ENODEV;
   }

   work = kmalloc(sizeof(PM_Work), GFP_KERNEL);
   if (!work)
   {
      printk("pm: Failed to allocate memory for work.\n");
      return -ENOMEM;
   }

   work->synchronous = 1;
   init_completion(&work->completion);
   work->event = event;
   work->data1 = data1;
   work->data2 = data2;
   work->data3 = data3;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
   INIT_WORK( &(work->work), pm_do_work, (void *)work );
   if ( queue_work( gPmData.workQueue, &(work->work) ) == 0 )
#else
   INIT_DELAYED_WORK( &(work->work), pm_do_work );
   if ( queue_delayed_work( gPmData.workQueue, &(work->work), 0 ) == 0 )
#endif
   {
      printk("pm: Work previously queued.\n");
      return -EINVAL;
   }

   wait_for_completion(&work->completion);
   PM_DEBUG(DBG_INFO, "work done.\n" );
   kfree (work);

   return 0;
} /* pm_submit_event_and_wait */


/****************************************************************************
*
*  pm_submit_delayed_event
*
***************************************************************************/
int pm_submit_delayed_event( PM_Event event, uint32_t delay, uint32_t data1, uint32_t data2, uint32_t data3 )
{
   PM_Work *work;

   if (!gPmData.workQueue)
   {
      panic("pm: Work Queue not initialized yet.\n"
            "    Make sure the Power Manager is linked in before other drivers.\n");
      return -ENODEV;
   }

   work = kmalloc(sizeof(PM_Work), GFP_KERNEL);
   if (!work)
   {
      printk("pm: Failed to allocate memory for work.\n");
      return -ENOMEM;
   }

   work->synchronous = 0;
   work->event = event;
   work->data1 = data1;
   work->data2 = data2;
   work->data3 = data3;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
   INIT_WORK( &(work->work), pm_do_work, (void *)work );
#else
   INIT_DELAYED_WORK( &(work->work), pm_do_work );
#endif
   if ( queue_delayed_work( gPmData.workQueue, &(work->work), delay ) == 0 )
   {
      printk("pm: Work previously queued.\n");
      return -EINVAL;
   }

   return 0;
} /* pm_submit_delayed_event */


/****************************************************************************
*
*   pm_register_component
*
***************************************************************************/
int pm_register_component( PM_Component component, PM_Comp_Ops_t *ops )
{
   PM_DEBUG(DBG_INFO, "%s (%d)\n", PM_ComponentTable[component], component);
   return pm_submit_event(PM_EVENT_INTERNAL, PM_REGISTER_COMPONENT, (uint32_t)component, (uint32_t)ops);
}


/****************************************************************************
*
*   pm_unregister_component
*
***************************************************************************/
void pm_unregister_component( PM_Component component, PM_Comp_Ops_t *ops )
{
   DECLARE_WAIT_QUEUE_HEAD(wait_queue);

   PM_DEBUG(DBG_INFO, "%s (%d)\n", PM_ComponentTable[component], component);

   if (!gPmData.components[component].registered)
   {
      printk("pm: component %s (%d) not registered.\n", PM_ComponentTable[component], component);
      return;
   }

   if (gPmData.components[component].ops != ops)
   {
      printk("pm: component %s (%d) registered by someone else.\n", PM_ComponentTable[component], component);
      return;
   }

   // Submit event to be processed by power manager, and wait for device to be fully
   // removed before returning.
   pm_submit_event(PM_EVENT_INTERNAL, PM_UNREGISTER_COMPONENT, (uint32_t)component, (uint32_t)&wait_queue);
   do
   {
      wait_event_interruptible(wait_queue, !gPmData.components[component].registered);
   } while (gPmData.components[component].registered);
}


/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/
struct file_operations pm_fops =
{
    owner:      THIS_MODULE,
    open:       pm_open,
    release:    pm_release,
    ioctl:      pm_ioctl,
    poll:       pm_poll,
};


/****************************************************************************
*
*  pm_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void pm_cleanup( void )
{
   PM_DEBUG(DBG_TRACE, "called\n" );

   if (gPmData.workQueue)
   {
      flush_workqueue(gPmData.workQueue);
      destroy_workqueue(gPmData.workQueue);
   }
   gPmData.workQueue = NULL;

   if (gPmData.devReg)
   {
      unregister_chrdev( BCM_PM_MAJOR, "PowerManager" );
   }
   gPmData.devReg = 0;

   if (gPmData.pmuChipInit)
   {
      pmu_chip_exit();
   }
   gPmData.pmuChipInit = 0;

   // unregister sysctl table
   if ( gPmData.sysCtlHeader != NULL )
   {
      unregister_sysctl_table( gPmData.sysCtlHeader );
   }
   gPmData.sysCtlHeader = NULL;

   if ( gPmData.platform != NULL )
   {
      kfree( gPmData.platform );
      gPmData.platform = NULL;
   }
}

/****************************************************************************
*
*  pm_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init pm_init( void )
{
   int rc = 0;
   int i;

   PM_DEBUG(DBG_TRACE, "called\n" );

   printk( banner );

   // Detect platform

   if ( gPmData.platform == NULL )
   {
      printk( KERN_ERR "pm_init: No PMU platform detected\n" );
      rc = -ENODEV;
      goto fail;
   }
   strncpy(gPmData.platformName, gPmData.platform->name, sizeof( gPmData.platformName ));
   gPmData.platformName[ sizeof( gPmData.platformName ) - 1 ] = '\0';

   printk( KERN_INFO "Platform '%s' detected.\n", gPmData.platformName );

   // Initialize PMU chip support
   if (( rc = pmu_chip_init(gPmData.platform->pmu_chip)) < 0 )
   {
      printk( KERN_ERR "pm: error %d initializing chip support driver\n", rc );
      goto fail;
   }
   gPmData.pmuChipInit = 1;
   gPmData.pmu = NULL;

   if (( rc = register_chrdev( BCM_PM_MAJOR, "PowerManager", &pm_fops )) < 0 )
   {
      printk( "pm: register_chrdev failed for major %d\n", BCM_PM_MAJOR );
      goto fail;
   }
   gPmData.devReg = 1;

   // Initialize component power level bitmap cache
   gPmData.offComponents = 0;

   // Initialize component registry
   for (i = 0; i < PM_NUM_COMPONENTS; i++)
   {
      gPmData.components[i].registered = 0;
      gPmData.components[i].ops = NULL;
      gPmData.components[i].powerLevel = PM_COMP_PWR_OFF;
      SET_COMPONENT(gPmData.offComponents,i);
   }

   // Initialize current state
   gPmData.powerLevel = PM_PWR_OFF;

#if defined( CONFIG_BCM_SLEEP_MODE )
   // Override CPU sleep mode during bootup
   // The override will expire when the PM_RELEASE_SLEEP_OVERRIDE event is received
   atomic_inc(&cpu_sleep_override);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   // register sysctl table
   gPmData.sysCtlHeader = register_sysctl_table( gSysCtl, 0 );
   if ( gPmData.sysCtlHeader != NULL )
   {
      gPmData.sysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   gPmData.sysCtlHeader = register_sysctl_table( gSysCtl );
#endif

   // Initialize poll queue
   init_waitqueue_head(&gPmData.pollQueue);
   sema_init(&gPmData.userEventSema, 1);

   // Create main power manager work queue
   gPmData.workQueue = create_singlethread_workqueue("pm_work");
   if (!gPmData.workQueue)
   {
      printk( "pm: error creating work queue\n" );
      goto fail;
   }

   // Register power off function
   pm_power_off = pm_local_power_off;


   // For the NULL PMU - simulate the attachment and poweron

   if ( gPmData.platform->pmu_chip == PMU_NONE )
   {
      pmu_start_null_pmu();
   }

   return 0;

fail:
   pm_cleanup();
   return rc;

} /* pm_init */


/****************************************************************************
*
*  pm_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void __exit pm_exit( void )
{
   PM_DEBUG(DBG_TRACE,  "called\n" );

   pm_cleanup();

} /* pm_exit */



/* Changed from module_init to fs_initcall.
 * PMU drivers were changed to fs_initcall so that they are loaded before
 * most of the other drivers. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */


fs_initcall(pm_init);
module_exit(pm_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Power Manager");

EXPORT_SYMBOL (pm_submit_event);
EXPORT_SYMBOL (pm_register_component);
EXPORT_SYMBOL (pm_unregister_component);

