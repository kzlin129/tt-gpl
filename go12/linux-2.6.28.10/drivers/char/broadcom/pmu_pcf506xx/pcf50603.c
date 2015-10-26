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
*  pcf50603.c
*
*  PURPOSE:
*
*     This implements the driver for the PMU chip (Philips pfc50603).
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
#include <linux/i2c.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/rtc.h>
#include <linux/broadcom/headset.h>
#include <linux/broadcom/led.h>
#include <linux/pm.h>
#include <linux/delay.h>


/* Include external interfaces */
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_pcf506xx.h>

/* Include internal interfaces */
#include "pcf506xx.h"
#include "headset.h"
#include "led.h"
#include "rtc.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

/* charging constants */
#define INIT_CHGC2_VALUE    (PCF506XX_BIT_VCHGCON420|PCF506XX_BIT_CURRAT20)
#define FAST_CHARGE_CHGC1   (PCF506XX_BIT_CHGAPE|PCF506XX_BIT_CHGMODFST|PCF506XX_BIT_WDTIME|PCF506XX_BIT_BATMAXHYST)
#define NO_CHARGE_CHGC1     (0)
#define CHARGER_STAT_NORMAL (PCF506XX_BIT_VBAT_MID|PCF506XX_BIT_VCHGSTAT_OK|PCF506XX_BIT_CHGCUR_MID)

/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "PCF50603 Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

//
// Regulators mapping
//

static pcf506xx_regulator_map_t pcf50603_regulator_map[PCF506XX_NUM_REGULATORS] =
{
   // PCF506XX_REGULATOR_D1
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50603_REG_D1C,
      min_mV:    1200,
      max_mV:    3200,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_D2
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50603_REG_D2C,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_D3
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50603_REG_D3C,
      min_mV:    1200,
      max_mV:    3200,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_HC
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50603_REG_HCC,
      min_mV:    2600,
      max_mV:    3200,
      mV_step:   200,
   },

   // PCF506XX_REGULATOR_CP
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50603_REG_CPC,
      min_mV:    3500,
      max_mV:    5000,
      mV_step:   500,
   },

   // PCF506XX_REGULATOR_IO
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50603_REG_IOC,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_LP
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50603_REG_LPC,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_RF1
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50603_REG_RF1C,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_RF2
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50603_REG_RF2C,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_DCD
   {
      available: 0,
   },

   // PCF506XX_REGULATOR_LC
   {
      available: 0,
   },
};

static BCM_PMU_Power_On_State_t gPowerOnState = PMU_Power_On_By_On_Button;

/* ---- Private Function Prototypes -------------------------------------- */

/* Module functions */
static int pcf50603_module_init( void );
static void pcf50603_module_exit( void );
static void pcf50603_module_cleanup( void );

/* Initialization function */
static int pcf50603_init( void );

/* Interrupt service routine */
static irqreturn_t pcf50603_isr(void *dev_id);

/* Get power on condition */
static BCM_PMU_Power_On_State_t pcf50603_get_power_on_state( void );

/* IOCTL handler */
static int pcf50603_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
static void pcf50603_poweroff(void);

/* Power regulator control */
static int pcf50603_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state);
static BCM_PMU_Regulator_State_t pcf50603_regulator_get_state(int regulatorID);
static int pcf50603_regulator_set_voltage(int regulatorID, u32 mV);
static u32 pcf50603_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);

/* Charger control */
static void pcf50603_charger_start(int chargerID);
static void pcf50603_charger_stop(int chargerID);
static int pcf50603_charger_is_inserted(int *chargerID);

/* Event dispatcher */
static void pcf50603_event_notify(PCF506XX_InterruptId_t irq_id);

/* I2C client address definitions */
static unsigned short normal_i2c[] = {PCF50603_I2C_BASE_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]       = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data pcf50603_i2c_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

/* PMU device operations */
static BCM_PMU_Operations_t pcf50603_ops =
{
   init: pcf50603_init,
   isr: pcf50603_isr,
   get_power_on_state: pcf50603_get_power_on_state,
   ioctl: pcf50603_ioctl,
   poweroff: pcf50603_poweroff,
   regulator:
   {
      set_state: pcf50603_regulator_set_state,
      get_state: pcf50603_regulator_get_state,
      set_voltage: pcf50603_regulator_set_voltage,
      get_voltage: pcf50603_regulator_get_voltage,
   },
   charger:
   {
      start: pcf50603_charger_start,
      stop: pcf50603_charger_stop,
      is_inserted: pcf50603_charger_is_inserted,
   },
   i2c_data: &pcf50603_i2c_data,
};

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  pcf50603_init
*
*     Called to initialize the PCF50603 device.
*
***************************************************************************/
static int pcf50603_init ( void )
{
   u8 int_status[PCF506XX_NUM_INT_REGS(PMU_PCF50603)];
   int rc;
   int i;

   printk("pcf50603_init\n");

   /* Initialize commonly used registers */
   pcf506xx_common_init(PMU_PCF50603);

   /* Initialize IRQ handler */
   pcf506xx_irq_init(PMU_PCF50603, int_status);

   /* Clear watchdog */
   pcf506xx_reset8Second(PMU_PCF50603);

   /* Save power on condition */
   if (int_status[0] & PCF506XX_BIT_ONKEYF)
   {
      /* power up through on key */
      gPowerOnState = PMU_Power_On_By_On_Button;
   }
   else if (int_status[2] & PCF506XX_BIT_CHGINS)
   {
      /* power up by charger insertion */
      gPowerOnState = PMU_Power_On_By_Charger;
   }
   else
   {
      /* power up by reboot, or pmu re-run */
      gPowerOnState = PMU_Power_On_By_Restart;
   }

   /* Initialize voltages for mask programmed regulators */
   for (i = 0; i < PCF506XX_NUM_REGULATORS; i++)
   {
      if (pcf50603_regulator_map[i].available &&
          !pcf50603_regulator_map[i].programmable)
      {
         printk("pcf50603: retrieving reset value for regulator %d\n", i);
         rc = pmu_i2c_read(pcf50603_regulator_map[i].reg_addr);
         if (rc < 0)
         {
            printk("pcf50603: error reading regulator control register.\n");
            return rc;
         }
         pcf506xx_vout_to_mV(i, (u8)rc, &pcf50603_regulator_map[i].min_mV);
         pcf50603_regulator_map[i].max_mV = pcf50603_regulator_map[i].min_mV;
      }
   }

   /* Register IRQ handler */
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT1LOWBAT, pcf50603_event_notify);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT1ONKEYR, pcf50603_event_notify);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT1ONKEYF, pcf50603_event_notify);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT1ONKEY1S, pcf50603_event_notify);

   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT2CHGEVT, pcf50603_event_notify);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT2CHGWD, pcf50603_event_notify);

   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT3CHGINS, pcf50603_event_notify);
   pcf506xx_irq_register(PMU_PCF50603, PCF506XX_IRQID_INT3CHGRM, pcf50603_event_notify);

   /* Initialize auxiliary interfaces */
   rtc506xx_init(PMU_PCF50603);
   led506xx_init(PMU_PCF50603);
   headset506xx_init(PMU_PCF50603);

   /* determine if the charger is plugged in */
   if (pcf50603_charger_is_inserted(NULL))
   {
      pcf50603_charger_start(PCF506XX_CHARGER_MAIN);
   }
   return 0;
} /* pcf50603_init */


/****************************************************************************
*
*  pcf50603_isr
*
***************************************************************************/
static irqreturn_t pcf50603_isr( void *dev_id )
{
   (void)dev_id;

   /* change status to show interrupt was received */
   return pcf506xx_isr(PMU_PCF50603);

} /*pcf50603_isr */


/****************************************************************************
*
*  pcf50603_get_power_on_state
*
***************************************************************************/
static BCM_PMU_Power_On_State_t pcf50603_get_power_on_state( void )
{
   return gPowerOnState;
}


/****************************************************************************
*
*  pcf50603_ioctl
*
***************************************************************************/
static int pcf50603_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   return pcf506xx_ioctl(PMU_PCF50603, inode, file, cmd, arg);

} /* pcf50603_ioctl */


/****************************************************************************
*
*  pcf50603_poweroff
*
***************************************************************************/
static void pcf50603_poweroff (void)
{
   pcf506xx_poweroff(PMU_PCF50603);
} /* pcf50603_poweroff */


/****************************************************************************
*
*  pcf50603_regulator_get_state
*
*  Set the state of a regulator
*
***************************************************************************/
static int pcf50603_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;

   //printk("pcf50603_regulator_set_state regId=%d, state=%d\n", regulatorID, state);
   if (!pcf50603_regulator_map[regulatorID].available)
   {
      printk("pcf50603_set_state: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = pcf506xx_state_to_opmod(regulatorID, state, &opmod);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error converting state %d.\n", state);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_i2c_read(pcf50603_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;

   // update register
   val &= ~(PCF506XX_BIT_REG_OPMOD_MASK);
   val |= opmod;

   // write settings
   rc = pmu_i2c_write(pcf50603_regulator_map[regulatorID].reg_addr, val);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error writing regulator control register.\n");
      return rc;
   }
   //printk("pcf50603_regulator_set_state addr=0x%x, val=0x%x\n", pcf50603_regulator_map[regulatorID].reg_addr, val);

   return 0;
} /* pcf50603_regulator_set_state */


/****************************************************************************
*
*  pcf50603_regulator_get_state
*
*  Retrieve the current state of a regulator
*
***************************************************************************/
static BCM_PMU_Regulator_State_t pcf50603_regulator_get_state(int regulatorID)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if (!pcf50603_regulator_map[regulatorID].available)
   {
      printk("pcf50603_get_state: regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_i2c_read(pcf50603_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50603_get_voltage: error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = pcf506xx_opmod_to_state(regulatorID, (u8)rc, &state) ;
   if (rc < 0)
   {
      printk("pcf50603_get_voltage: error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* pcf50603_regulator_get_state */


/****************************************************************************
*
*  pcf50603_regulator_set_voltage
*
*  Set the current voltage level
*
***************************************************************************/
static int pcf50603_regulator_set_voltage(int regulatorID, u32 mV)
{
   int rc;
   u8 val;
   u8 vout;

   if (!pcf50603_regulator_map[regulatorID].available)
   {
      printk("pcf50603_set_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert voltage
   rc = pcf506xx_mV_to_vout(regulatorID, mV, &vout,
                            pcf50603_regulator_map[regulatorID].min_mV,
                            pcf50603_regulator_map[regulatorID].max_mV,
                            pcf50603_regulator_map[regulatorID].mV_step);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error converting %d mV.\n", mV);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_i2c_read(pcf50603_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;

   // update register
   val &= ~(PCF506XX_BIT_REG_VOUT_MASK);
   val |= vout;

   // write settings
   rc = pmu_i2c_write(pcf50603_regulator_map[regulatorID].reg_addr, val);
   if (rc < 0)
   {
      printk("pcf50603_set_voltage: error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* pcf50603_regulator_set_voltage */


/****************************************************************************
*
*  pcf50603_regulator_get_voltage
*
*  Retrieve the current voltage level and optionally the valid range of settings.
*
***************************************************************************/
static u32 pcf50603_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step)
{
   int rc;
   u32 mV;

   if (!pcf50603_regulator_map[regulatorID].available)
   {
      printk("pcf50603_get_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   rc = pmu_i2c_read(pcf50603_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50603_get_voltage: error reading regulator control register.\n");
      return rc;
   }

   rc = pcf506xx_vout_to_mV(regulatorID, (u8)rc, &mV) ;
   if (rc < 0)
   {
      printk("pcf50603_get_voltage: error converting voltage.\n");
      return rc;
   }

   if (min_mV)
      *min_mV = pcf50603_regulator_map[regulatorID].min_mV;
   if (max_mV)
      *max_mV = pcf50603_regulator_map[regulatorID].max_mV;
   if (mV_step)
      *mV_step = pcf50603_regulator_map[regulatorID].mV_step;

   return mV;
} /* pcf50603_regulator_get_voltage */


/****************************************************************************
*
*  pcf50603_charger_start
*
*  Start the charging process to avoid phone turning off and rebooting when
*  battery is low (and charger is inserted).
*
***************************************************************************/
static void pcf50603_charger_start(int chargerID)
{
   int rc;
   uint8_t oocs1, oocs2;

   /* The PCF50603 only supports the main charger */
   if (chargerID != PCF506XX_CHARGER_MAIN)
   {
      return;
   }

   /* start fast charge if charger is plugged in */
   rc = pmu_i2c_read(PCF50603_REG_OOCS1);
   if ( rc != -1 )
   {
       oocs1 = (uint8_t)rc;
       oocs2 = pmu_i2c_read(PCF50603_REG_OOCS2);
       if ( oocs1 & PCF506XX_BIT_CHG_OK )
       {
          /* charger is present, check to see if voltage is in range */
          rc = pmu_i2c_read(PCF50603_REG_CHGS1);
          if ( rc != 1 )
          {
             if ( rc & PCF506XX_BIT_VCHGSTAT_OK )
             {
                /* Start the fast charge (CCCV) */
                rc = pmu_i2c_write( PCF50603_REG_CHGC2, INIT_CHGC2_VALUE | PCF506XX_BIT_WDRST );

                if ( rc == 0 )
                {
                   /* write command to PCF50603 to start fast charge process */
                   rc = pmu_i2c_write( PCF50603_REG_CHGC1, FAST_CHARGE_CHGC1 );
                }
             }
          }
       }
   }
}


/****************************************************************************
*
*  pcf50603_charger_stop
*
*  Stop the charging process.
*
***************************************************************************/
static void pcf50603_charger_stop(int chargerID)
{
   int rc;

   /* The PCF50603 only supports the main charger */
   if (chargerID != PCF506XX_CHARGER_MAIN)
   {
      return;
   }

   rc = pmu_i2c_write( PCF50603_REG_CHGC1, NO_CHARGE_CHGC1 );

   return;

} /* pcf50603_charger_stop */


/****************************************************************************
*
*  pcf50603_charger_is_inserted
*
*  Check if charger is inserted
*
***************************************************************************/
static int pcf50603_charger_is_inserted(int *chargerID)
{
   int rc;
   rc = pmu_i2c_read(PCF50603_REG_OOCS1);
   if ( ( rc >= 0 ) && ( (u8)rc & PCF506XX_BIT_CHG_OK ) )
   {
      /* charger is present, check to see if voltage is in range */
      rc = pmu_i2c_read(PCF50603_REG_CHGS1);
      if ( ( rc >= 0 ) && ( (u8)rc & PCF506XX_BIT_VCHGSTAT_OK ) )
      {
         if (chargerID)
         {
            *chargerID = PCF506XX_CHARGER_MAIN;
         }
         return 1;
      }
   }
   return 0;
} /* pcf50603_charger_is_inserted */


/****************************************************************************
*
*  pcf50603_event_notify
*
*     Called to dispatch interrupt into PMU events
*
***************************************************************************/
static void pcf50603_event_notify(PCF506XX_InterruptId_t irq_id)
{
   int rc;
   BCM_PMU_Event_t event = PMU_NUM_EVENTS;
   void *data = NULL;

   switch (irq_id)
   {
      // Onkey events
      case PCF506XX_IRQID_INT1ONKEYR:
         event = PMU_EVENT_ONKEY_RISE;
         break;
      case PCF506XX_IRQID_INT1ONKEYF:
         event = PMU_EVENT_ONKEY_FALL;
         break;

      case PCF506XX_IRQID_INT1ONKEY1S:
         pcf506xx_reset8Second(PMU_PCF50603);
         event = PMU_EVENT_ONKEY_1S_HOLD;
         break;

      // Battery and charger events
      case PCF506XX_IRQID_INT1LOWBAT:
         /* read battery status in on/off control status reg 1 */
         rc = pmu_i2c_read(PCF50603_REG_OOCS1);
         if ( rc != -1 )
         {
            if ( rc & PCF506XX_BIT_BAT_OK )
            {
               /* battery voltage is okay, reset the TOT */
               pcf506xx_reset8Second(PMU_PCF50603);
            }
            else
            {
               // Notify low battery event
               event = PMU_EVENT_BATTERY_LOW;
            }
         }
         break;

      case PCF506XX_IRQID_INT3CHGINS:
         //printk("pm: PCF506XX_IRQID_INT3CHGINS\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)PCF506XX_CHARGER_MAIN;
         break;

      case PCF506XX_IRQID_INT3CHGRM:
         //printk("pm: PCF506XX_IRQID_INT3CHGRM");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)PCF506XX_CHARGER_MAIN;
         break;

      // Generic charger event from 50603, need to parse status bits
      case PCF506XX_IRQID_INT2CHGEVT:

         // read the charger status
         rc = pmu_i2c_read( PCF50603_REG_CHGS1 );

         if ( rc == -1 )
         {
             printk("pcf50603: failed to read charger status\n");
             break;
         }
         //printk("pcf50603: charger status = 0x%x\n",rc);

         /* Check the end of charge voltage condition */
         if ((rc & PCF506XX_BIT_VBAT_MASK) == PCF506XX_BIT_VBAT_HI )
         {
             printk("pcf50603 - battery voltage is higher than preset voltage\n");
         }

         /* check over-voltage condition on charger */
         if ((rc & PCF506XX_BIT_VCHG_MASK) == PCF506XX_BIT_VCHGSTAT_HI)
         {
             printk("pcf50603 - charger voltage was too high\n");
         }

         /* check charger current to see if charging is finished */
         if ((rc & PCF506XX_BIT_CHGCUR_MASK) == PCF506XX_BIT_CHGCUR_LO )
         {
             event = PMU_EVENT_BATTERY_FULL;
             break;
         }

         break;

      // Reset charger watchdog without sending event up
      case PCF506XX_IRQID_INT2CHGWD:
         rc = pmu_i2c_write( PCF50603_REG_CHGC2, INIT_CHGC2_VALUE | PCF506XX_BIT_WDRST );
         break;

      default:
         break;
   }

   // Notify PMU
   if (event != PMU_NUM_EVENTS)
   {
      pmu_event_notify(PMU_PCF50603, event, data);
   }

} /* pcf50603_event_notify */


/****************************************************************************
*
*  pcf50603_module_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init pcf50603_module_init( void )
{
   printk( banner );

   printk("pcf50603: register with PMU module\n");
   return pmu_register_device(PMU_PCF50603, &pcf50603_ops);

} /* pcf50603_module_init */


/****************************************************************************
*
*  pcf50603_module_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit pcf50603_module_exit( void )
{
   printk("pcf50603: module_exit called\n");

   pcf50603_module_cleanup();

} /* pcf50603_module_exit */


/****************************************************************************
*
*  pcf50603_module_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void pcf50603_module_cleanup( void )
{
   printk("pcf50603: module_cleanup\n");

} /* pcf50603_module_cleanup */


fs_initcall(pcf50603_module_init);
module_exit(pcf50603_module_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("PCF50603 Driver");

