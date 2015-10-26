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
*  pcf50611.c
*
*  PURPOSE:
*
*     This implements the driver for the PMU chip (Philips pfc50611).
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

#define INIT_CBCC2_VALUE_N1C    (PCF506XX_BIT_CBCC2_VBATCOND_3_0|PCF506XX_BIT_CBCC2_VMAX_4_65V)
#define INIT_CBCC2_VALUE_N2A    (PCF506XX_BIT_CBCC2_VBATCOND_3_0|PCF506XX_BIT_CBCC2_VMAX_4_10V)
#define FAST_CHARGE_CBCC1   (PCF506XX_BIT_CBCC1_CHGENA|PCF506XX_BIT_CBCC1_USBENA| PCF506XX_BIT_CBCC1_AUTOCC| \
                             PCF506XX_BIT_CBCC1_WDRST| \
                             PCF506XX_BIT_CBCC1_WDTIME_NOLIM| \
                             PCF506XX_BIT_CBCC1_AUTORES_6_0 )
#define FAST_CHARGE_CBCC3   (PCF506XX_BIT_CBCC3_VAL)
#define FAST_CHARGE_CBCC4   (PCF506XX_BIT_CBCC4_VAL)
#define FAST_CHARGE_CBCC5_TRICKLE_N2A   0xF
#define NO_CHARGE_CBCC1     (0)

static char banner[] __initdata = KERN_INFO "PCF50611 Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

//
// Regulators mapping
//

static pcf506xx_regulator_map_t pcf50611_regulator_map[PCF506XX_NUM_REGULATORS] =
{
   // PCF506XX_REGULATOR_D1
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_D1C,
      min_mV:    1200,
      max_mV:    3300,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_D2
   {
      available: 1,
      programmable: 0,
      reg_addr:  PCF50611_REG_D2C,
      mV_step:   0,
      // min_mV and max_mV are set during init for mask programmable regulators
   },

   // PCF506XX_REGULATOR_D3
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_D3C,
      min_mV:    1200,
      max_mV:    3300,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_HC
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_HCC,
      min_mV:    2600,
      max_mV:    3200,
      mV_step:   200,
   },

   // PCF506XX_REGULATOR_CP
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_CPC,
      min_mV:    3500,
      max_mV:    5000,
      mV_step:   500,
   },

   // PCF506XX_REGULATOR_IO
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_IOC,
      min_mV:    1500,
      max_mV:    3300,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_LP
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_LPC,
      min_mV:    1200,
      max_mV:    3300,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_RF1
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_RF1C,
      min_mV:    1200,
      max_mV:    3000,
      mV_step:   100,
   },

   // PCF506XX_REGULATOR_RF2
   {
      available: 1,
      programmable: 1,
      reg_addr:  PCF50611_REG_RF2C,
      min_mV:    1200,
      max_mV:    3000,
      mV_step:   100,
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
static int pcf50611_module_init( void );
static void pcf50611_module_exit( void );
static void pcf50611_module_cleanup( void );

/* Initialization function */
static int pcf50611_init( void );

/* Interrupt service routine */
static irqreturn_t pcf50611_isr(void *dev_id);

/* Get power on condition */
static BCM_PMU_Power_On_State_t pcf50611_get_power_on_state( void );

/* IOCTL handler */
static int pcf50611_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
static void pcf50611_poweroff(void);

/* Power regulator control */
static int pcf50611_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state);
static BCM_PMU_Regulator_State_t pcf50611_regulator_get_state(int regulatorID);
static int pcf50611_regulator_set_voltage(int regulatorID, u32 mV);
static u32 pcf50611_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);

/* Charger control */
static void pcf50611_charger_start(int chargerID);
static void pcf50611_charger_stop(int chargerID);
static int pcf50611_charger_is_inserted(int *chargerID);

/* Event dispatcher */
static void pcf50611_event_notify(PCF506XX_InterruptId_t irq_id);

/* I2C client address definitions */
static unsigned short normal_i2c[] = {PCF50611_I2C_BASE_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]       = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data pcf50611_i2c_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

/* PMU device operations */
static BCM_PMU_Operations_t pcf50611_ops =
{
   init: pcf50611_init,
   isr: pcf50611_isr,
   get_power_on_state: pcf50611_get_power_on_state,
   ioctl: pcf50611_ioctl,
   poweroff: pcf50611_poweroff,
   regulator:
   {
      set_state: pcf50611_regulator_set_state,
      get_state: pcf50611_regulator_get_state,
      set_voltage: pcf50611_regulator_set_voltage,
      get_voltage: pcf50611_regulator_get_voltage,
   },
   charger:
   {
      start: pcf50611_charger_start,
      stop: pcf50611_charger_stop,
      is_inserted: pcf50611_charger_is_inserted,
   },
   i2c_data: &pcf50611_i2c_data,
};

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  pcf50611_init
*
*     Called to initialize the PCF50611 device.
*
***************************************************************************/
static int pcf50611_init ( void )
{
   u8 int_status[PCF506XX_NUM_INT_REGS(PMU_PCF50611)];
   int rc;
   int i;

   printk("pcf50611_init\n");

   /* Initialize commonly used registers */
   pcf506xx_common_init(PMU_PCF50611);

   /* Initialize IRQ handler */
   pcf506xx_irq_init(PMU_PCF50611, int_status);

   /* Clear watchdog */
//   pcf506xx_reset8Second(PMU_PCF50611);

   /* Save power on condition */
   if (int_status[0] & PCF506XX_BIT_ONKEYF)
   {
      /* power up through on key */
      gPowerOnState = PMU_Power_On_By_On_Button;
   }
   else if ((int_status[2] & PCF506XX_BIT_CHGINS) ||
            (int_status[3] & PCF506XX_BIT_UCHGINS))
   {
      /* power up by charger insertion */
      gPowerOnState = PMU_Power_On_By_Charger;
   }
   else
   {
      /* power up by reboot, or pmu re-run */
      gPowerOnState = PMU_Power_On_By_Restart;

   }

   /* If a battery is reinserted, a false LOWBAT interrupt is being generated
    * THis code would clear that interrupt  */

   if (int_status[0] & PCF506XX_BIT_LOWBAT)
   {
      /* read battery status in on/off control status reg 1 */
      rc = pmu_i2c_read(PCF50611_REG_OOCS1);
      if ( rc != -1 )
      {
         if ( rc & PCF506XX_BIT_BAT_OK )
         {
            /* battery voltage is okay, reset the TOT */
            pcf506xx_reset8Second(PMU_PCF50611);
            printk("PMU: False LOWBAT Interrupt, Reset Timer\n");
         }
         else
         {
            printk("PMU: Low BAT Interrupt, Shutting down in 8 sec\n");
         }
      }
   }
   /* Initialize voltages for mask programmed regulators */
   for (i = 0; i < PCF506XX_NUM_REGULATORS; i++)
   {
      if (pcf50611_regulator_map[i].available &&
          !pcf50611_regulator_map[i].programmable)
      {
         printk("pcf50611: retrieving reset value for regulator %d\n", i);
         rc = pmu_i2c_read(pcf50611_regulator_map[i].reg_addr);
         if (rc < 0)
         {
            printk("pcf50611: error reading regulator control register.\n");
            return rc;
         }
         pcf506xx_vout_to_mV(i, (u8)rc, &pcf50611_regulator_map[i].min_mV);
         pcf50611_regulator_map[i].max_mV = pcf50611_regulator_map[i].min_mV;
      }
   }

   /* Register IRQ handler */
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT1LOWBAT, pcf50611_event_notify);
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT1ONKEYR, pcf50611_event_notify);
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT1ONKEYF, pcf50611_event_notify);
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT1ONKEY1S, pcf50611_event_notify);

   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT2CHGEVT, pcf50611_event_notify);      // VMAX on 50611
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT2CHGWD, pcf50611_event_notify);

   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT3CHGINS, pcf50611_event_notify);
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT3CHGRM, pcf50611_event_notify);

   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT4BATFUL, pcf50611_event_notify);    // Vbat > Vmax
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT4CHGRES, pcf50611_event_notify);    // Vbat < resume
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT4UCHGINS, pcf50611_event_notify);   // usb charger in
   pcf506xx_irq_register(PMU_PCF50611, PCF506XX_IRQID_INT4UCHGRM, pcf50611_event_notify);    // usb charger out

   /* Initialize auxiliary interfaces */
   rtc506xx_init(PMU_PCF50611);
   led506xx_init(PMU_PCF50611);
   headset506xx_init(PMU_PCF50611);

   /* determine if the charger is plugged in */
   {
      int chargerId;
      if (pcf50611_charger_is_inserted(&chargerId))
      {
         pcf50611_charger_start(chargerId);
      }
   }

   return 0;

} /* pcf50611_init */


/****************************************************************************
*
*  pcf50611_isr
*
***************************************************************************/
static irqreturn_t pcf50611_isr( void *dev_id )
{
   (void)dev_id;

   /* change status to show interrupt was received */
   return pcf506xx_isr(PMU_PCF50611);

} /*pcf50611_isr */

/****************************************************************************
*
*  pcf50611_get_power_on_state
*
***************************************************************************/
static BCM_PMU_Power_On_State_t pcf50611_get_power_on_state( void )
{
   return gPowerOnState;
}


/****************************************************************************
*
*  pcf50611_ioctl
*
***************************************************************************/
static int pcf50611_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   return pcf506xx_ioctl(PMU_PCF50611, inode, file, cmd, arg);

} /* pcf50611_ioctl */


/****************************************************************************
*
*  pcf50611_poweroff
*
***************************************************************************/
static void pcf50611_poweroff (void)
{
   pcf506xx_poweroff(PMU_PCF50611);
} /* pcf50611_poweroff */


/****************************************************************************
*
*  pcf50611_regulator_get_state
*
*  Set the state of a regulator
*
***************************************************************************/
static int pcf50611_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;

   if (!pcf50611_regulator_map[regulatorID].available)
   {
      printk("pcf50611_set_state: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = pcf506xx_state_to_opmod(regulatorID, state, &opmod);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error converting state %d.\n", state);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_i2c_read(pcf50611_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;

   // update register
   val &= ~(PCF506XX_BIT_REG_OPMOD_MASK);
   val |= opmod;

   // write settings
   rc = pmu_i2c_write(pcf50611_regulator_map[regulatorID].reg_addr, val);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* pcf50611_regulator_set_state */


/****************************************************************************
*
*  pcf50611_regulator_get_state
*
*  Retrieve the current state of a regulator
*
***************************************************************************/
static BCM_PMU_Regulator_State_t pcf50611_regulator_get_state(int regulatorID)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if (!pcf50611_regulator_map[regulatorID].available)
   {
      printk("pcf50611_get_state: regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_i2c_read(pcf50611_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50611_get_voltage: error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = pcf506xx_opmod_to_state(regulatorID, (u8)rc, &state) ;
   if (rc < 0)
   {
      printk("pcf50611_get_voltage: error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* pcf50611_regulator_get_state */


/****************************************************************************
*
*  pcf50611_regulator_set_voltage
*
*  Set the current voltage level
*
***************************************************************************/
static int pcf50611_regulator_set_voltage(int regulatorID, u32 mV)
{
   int rc;
   u8 val;
   u8 vout;

   if (!pcf50611_regulator_map[regulatorID].available)
   {
      printk("pcf50611_set_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert voltage
   rc = pcf506xx_mV_to_vout(regulatorID, mV, &vout,
                            pcf50611_regulator_map[regulatorID].min_mV,
                            pcf50611_regulator_map[regulatorID].max_mV,
                            pcf50611_regulator_map[regulatorID].mV_step);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error converting %d mV.\n", mV);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_i2c_read(pcf50611_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;

   // update register
   val &= ~(PCF506XX_BIT_REG_VOUT_MASK);
   val |= vout;

   // write settings
   rc = pmu_i2c_write(pcf50611_regulator_map[regulatorID].reg_addr, val);
   if (rc < 0)
   {
      printk("pcf50611_set_voltage: error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* pcf50611_regulator_set_voltage */


/****************************************************************************
*
*  pcf50611_regulator_get_voltage
*
*  Retrieve the current voltage level and optionally the valid range of settings.
*
***************************************************************************/
static u32 pcf50611_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step)
{
   int rc;
   u32 mV;

   if (!pcf50611_regulator_map[regulatorID].available)
   {
      printk("pcf50611_get_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   rc = pmu_i2c_read(pcf50611_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      printk("pcf50611_get_voltage: error reading regulator control register.\n");
      return rc;
   }

   rc = pcf506xx_vout_to_mV(regulatorID, (u8)rc, &mV) ;
   if (rc < 0)
   {
      printk("pcf50611_get_voltage: error converting voltage.\n");
      return rc;
   }

   if (min_mV)
      *min_mV = pcf50611_regulator_map[regulatorID].min_mV;
   if (max_mV)
      *max_mV = pcf50611_regulator_map[regulatorID].max_mV;
   if (mV_step)
      *mV_step = pcf50611_regulator_map[regulatorID].mV_step;

   return mV;
} /* pcf50611_regulator_get_voltage */


/****************************************************************************
*
*  pcf50611_charger_start
*
*  Start the charging process to avoid phone turning off and rebooting when
*  battery is low (and charger is inserted).
*
***************************************************************************/
static void pcf50611_charger_start(int chargerID)
{
   int rc;
   int intmask;
   uint8_t oocs1, oocs2, cbcc5;
   uint8_t version ;

   /* The PCF50611 only supports both the main and USB charger */
   if ((chargerID != PCF506XX_CHARGER_MAIN) && (chargerID != PCF506XX_CHARGER_USB))
   {
      return;
   }

   /* start fast charge if charger is plugged in */

   /* Read the PMU version  */
   rc = pmu_i2c_read(PCF50611_REG_ID);
   if (rc != -1)
   {
      version = (uint8_t) rc;

      rc = pmu_i2c_read(PCF50611_REG_OOCS1);
      if ( rc != -1 )
      {
         oocs1 = (uint8_t)rc;
         oocs2 = pmu_i2c_read(PCF50611_REG_OOCS2);
         if ((oocs1 & PCF506XX_BIT_CHG_OK ) || (oocs2 & PCF506XX_BIT_UCHG_OK))
         {
            if (version == PCF50611_REG_ID_VAL_N1C)
            {
               /* Start the fast charge (CCCV) */
               rc = pmu_i2c_write( PCF50611_REG_CBCC2, INIT_CBCC2_VALUE_N1C );

            }
            else
            if (version == PCF50611_REG_ID_VAL_N2A )
            {
               rc = pmu_i2c_write( PCF50611_REG_CBCC2, INIT_CBCC2_VALUE_N2A );
               rc = pmu_i2c_read(PCF50611_REG_CBCC5);
               if ( rc != -1 )
               {
                  cbcc5 = (uint8_t) rc;
                  cbcc5 = (cbcc5 & ~PCF506XX_BIT_CBCC5_TRICKLE_MASK) | (FAST_CHARGE_CBCC5_TRICKLE_N2A << PCF506XX_BIT_CBCC5_TRICKLE_SHIFT);
                  rc = pmu_i2c_write( PCF50611_REG_CBCC5, cbcc5 );
               }
            }
            else
            {
               /* Bad version number */
               /* report error  */
               printk("PMU - Incorrect version number %d\n", version);
               return;

            }
            if ( rc == 0 )
            {
               /* write command to PCF50611 to start fast charge process */
               rc = pmu_i2c_write( PCF50611_REG_CBCC1, FAST_CHARGE_CBCC1 );
            }
            if ( chargerID == PCF506XX_CHARGER_USB )
            {
               /* Set current to maximum for USB charger */
               rc = pmu_i2c_write( PCF50611_REG_CBCC4, FAST_CHARGE_CBCC4 );
            }
#if 0 // Not sure if we need this
            if ( chargerID == PCF506XX_CHARGER_MAIN )
            {
               /* Set current to maximum for Main charger */
               rc = pmu_i2c_write( PCF50611_REG_CBCC3, FAST_CHARGE_CBCC3 );
            }
#endif
         }
      }
   }
   /* unmask the battery full interrupt */
   intmask = pmu_i2c_read(PCF50611_REG_INT4M);
   rc = pmu_i2c_write(PCF50611_REG_INT4M, intmask & ~PCF506XX_BIT_BATFUL );
   if ( rc != 0 )
   {
      printk("PMU - Unable to unmask battery full interrupt\n");
   }

   /* Unmask the vmax interrupt */
   intmask = pmu_i2c_read(PCF50611_REG_INT2M);
   rc = pmu_i2c_write(PCF50611_REG_INT2M, intmask & ~PCF506XX_BIT_VMAX );
   if ( rc != 0 )
   {
      printk("PMU - Unable to unmask vmax interrupt\n");
   }
}

/****************************************************************************
*
*  pcf50611_charger_stop
*
*  Stop the charging process.
*
***************************************************************************/
static void pcf50611_charger_stop(int chargerID)
{
   int rc;

   /* The PCF50611 only supports the main charger */
   if ((chargerID != PCF506XX_CHARGER_MAIN) && (chargerID != PCF506XX_CHARGER_USB))
   {
      return;
   }

   rc = pmu_i2c_write( PCF50611_REG_CBCC1, NO_CHARGE_CBCC1 );

   return;

} /* pcf50611_charger_stop */


/****************************************************************************
*
*  pcf50611_charger_is_inserted
*
*  Check if charger is inserted
*
***************************************************************************/
static int pcf50611_charger_is_inserted(int *chargerID)
{
   int oocs1,oocs2;

   oocs1 = pmu_i2c_read(PCF50611_REG_OOCS1);
   oocs2 = pmu_i2c_read(PCF50611_REG_OOCS2);
   if ( ( oocs1 >= 0 ) && ( (u8)oocs1 & PCF506XX_BIT_CHG_OK ) )
   {
      if (chargerID)
      {
         *chargerID = PCF506XX_CHARGER_MAIN;
      }
      return 1;
   }
   if ( ( oocs2 >= 0 ) && ( (u8)oocs2 & PCF506XX_BIT_UCHG_OK ) )
   {
      if (chargerID)
      {
         *chargerID = PCF506XX_CHARGER_USB;
      }
      return 1;
   }
   return 0;
} /* pcf50611_charger_is_inserted */


/****************************************************************************
*
*  pcf50611_event_notify
*
*     Called to dispatch interrupt into PMU events
*
***************************************************************************/
static void pcf50611_event_notify(PCF506XX_InterruptId_t irq_id)
{
   int rc;
   BCM_PMU_Event_t event = PMU_NUM_EVENTS;
   //int chargerID;
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
         pcf506xx_reset8Second(PMU_PCF50611);
         event = PMU_EVENT_ONKEY_1S_HOLD;
         break;

      // Battery and charger events
      case PCF506XX_IRQID_INT1LOWBAT:
         /* read battery status in on/off control status reg 1 */
         rc = pmu_i2c_read(PCF50611_REG_OOCS1);
         if ( rc != -1 )
         {
            if ( rc & PCF506XX_BIT_BAT_OK )
            {
               /* battery voltage is okay, reset the TOT */
               printk("PMU: False Low BAT Interrupt, Reset timer\n");
               pcf506xx_reset8Second(PMU_PCF50611);
            }
            else
            {
               printk("PMU: Low BAT Interrupt, Shutting down in 8 sec\n");
               // Notify low battery event
               event = PMU_EVENT_BATTERY_LOW;
            }
         }
         break;

      case PCF506XX_IRQID_INT3CHGINS:
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)PCF506XX_CHARGER_MAIN;
         break;
      case PCF506XX_IRQID_INT3CHGRM:
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)PCF506XX_CHARGER_MAIN;
         break;

      // This is really the VMAX event on the 611

      case PCF506XX_IRQID_INT2CHGEVT:
         {
            int intmask;
            rc = pmu_i2c_read( PCF50611_REG_CBCS1 );

            /* Ignore interrupt if no switch to contant voltage detected */
            if ( rc & PCF506XX_BIT_VLMT_MASK )
            {
               /*printk("pcf50611: Charger has changed from CC to CV, CBCS1 = 0x%x\n", rc);*/

               /* Mask the vmax interrupt as we no longer need to recieve interrupt*/
               intmask = pmu_i2c_read(PCF50611_REG_INT2M);
               rc = pmu_i2c_write(PCF50611_REG_INT2M, (intmask | PCF506XX_BIT_VMAX) );
               if ( rc != 0 )
               {
                  printk("PMU - Unable to mask vmax interrupt\n");
               }
            }
         }
         break;



      // Reset charger watchdog without sending event up
      case PCF506XX_IRQID_INT2CHGWD:
         {
            int cbcc1;
            cbcc1 = pmu_i2c_read( PCF50611_REG_CBCC1);
            rc = pmu_i2c_write( PCF50611_REG_CBCC1, cbcc1 | PCF506XX_BIT_CBCC1_WDRST );
            if ( rc != 0 )
            {
               printk("PMU - Unable to reset charger watchdog\n");
            }
         }
         break;

      case PCF506XX_IRQID_INT4BATFUL:
          /* check the batful status bit as a double-check */
          rc = pmu_i2c_read( PCF50611_REG_CBCS1 );
          if ( rc == -1 )
          {
              printk("PMU - failed to read charger status\n");
              return;
          }
          else if ( rc & PCF506XX_BIT_BATFUL_MASK )
          {
              int intmask;
              /* batful bit is set */
              /*printk("PMU - Charger: battery is full, CBCS1 = 0x%x\n",rc);*/

              /* Mask the battery full interrupt */
              intmask = pmu_i2c_read(PCF50611_REG_INT4M);
              rc = pmu_i2c_write(PCF50611_REG_INT4M, (intmask | PCF506XX_BIT_BATFUL) );
              if ( rc != 0 )
              {
                 printk("PMU - Unable to mask battery full interrupt\n");
              }

              event = PMU_EVENT_BATTERY_FULL;
          }
          else
          {
              /*printk("PMU - spurious BATFUL interrupt, CBCS1 = 0x%x\n",rc);*/
          }
          break;

      case PCF506XX_IRQID_INT4CHGRES:
         printk("pcf50611: Vbat < resume\n");
         break;

      case PCF506XX_IRQID_INT4UCHGINS:
         printk("pcf50611: USB Charger Inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         //   rc = pmu_i2c_read(PCF50611_REG_BVMC);
         //   pmu_i2c_write(PCF50611_REG_BVMC, ((rc & ~(0x0e)) |(0x06)));
         data = (void *)PCF506XX_CHARGER_USB;
         break;

      case PCF506XX_IRQID_INT4UCHGRM:
         printk("pcf50611: USB Charger Removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         //   rc = pmu_i2c_read(PCF50611_REG_BVMC);
         //   pmu_i2c_write(PCF50611_REG_BVMC, ((rc & ~(0x0e)) |(0x0c)));
         data = (void *)PCF506XX_CHARGER_USB;
         break;

      default:
         break;
   }


   // Notify PMU
   if (event != PMU_NUM_EVENTS)
   {
      pmu_event_notify(PMU_PCF50611, event, data);
   }

} /* pcf50611_event_notify */


/****************************************************************************
*
*  pcf50611_module_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init pcf50611_module_init( void )
{
   printk( banner );

   printk("pcf50611: register with PMU module\n");
   return pmu_register_device(PMU_PCF50611, &pcf50611_ops);

} /* pcf50611_module_init */


/****************************************************************************
*
*  pcf50611_module_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit pcf50611_module_exit( void )
{
   printk("pcf50611: module_exit called\n");

   pcf50611_module_cleanup();

} /* pcf50611_module_exit */


/****************************************************************************
*
*  pcf50611_module_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void pcf50611_module_cleanup( void )
{
   printk("pcf50611: module_cleanup\n");

} /* pcf50611_module_cleanup */


/* Changed from module_init to fs_initcall so that GPIO driver
 * is loaded before the any of the PowerMgr and PMU drivers are loaded.
 * PMU drivers were also changed to fs_initcall so that they are loaded before
 * most of the other drivers. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */

fs_initcall(pcf50611_module_init);
module_exit(pcf50611_module_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("PCF50611 Driver");

