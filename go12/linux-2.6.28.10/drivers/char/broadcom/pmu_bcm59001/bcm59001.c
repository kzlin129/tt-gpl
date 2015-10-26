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
*  bcm59001.c
*
*  PURPOSE:
*
*     This implements the common portion of the drivers for the BCM59001 chip.
*
*  NOTES:
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>

/* Include external interfaces */
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59001.h>
#include <linux/broadcom/timer.h>

/* Include internal interfaces */
#include "bcm59001.h"
#include "headset.h"
#include "led.h"
#include "rtc.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define TEST_FG   0 // Simple test of coulomb counter fuel gauge

// INT2M Setting - handle EOC interrupts separately based on charger state
#define INT2M_DEFAULT \
              ( BCM59001_BIT_INT2_CHGINS | \
                BCM59001_BIT_INT2_CHGRM |  \
                BCM59001_BIT_INT2_CHGERR | \
                BCM59001_BIT_INT2_USBINS | \
                BCM59001_BIT_INT2_USBRM |  \
                BCM59001_BIT_INT2_MBCCHGERR )

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL	(DBG_ERROR)
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR|DBG_INFO|DBG_TRACE|DBG_TRACE2)

#define PMU_DEBUG(level,fmt,args...) do { if (level & logLevel) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
//#define PMU_DEBUG(level,fmt,args...)


enum 
{
   CHG_INS  = 1,  // Must not start from 0
   CHG_RM   = 2
};

/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "BCM59001 Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

DECLARE_WAIT_QUEUE_HEAD(gChgEventQ);        
static int gChgEvent;
static long gChgThreadPid = 0;
static struct completion gChgExited;

static long gIsrThreadPid = 0;
static struct completion gIsrExited;
static struct semaphore gIsrSem;

static int logLevel = DBG_DEFAULT_LEVEL;

static bcm59001_isr_t bcm59001_IRQVectorTable[BCM59001_NUM_IRQ];

static bcm59001_regulator_map_t bcm59001_regulator_map[BCM59001_NUM_REGULATORS] =
{
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_A1OPMODCTRL,     // BCM59001_REGULATOR_ALDO1
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_A2OPMODCTRL,     // BCM59001_REGULATOR_ALDO2
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_R1OPMODCTRL,     // BCM59001_REGULATOR_RFLDO1
      mV_step:       0,
   },
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_R2OPMODCTRL,     // BCM59001_REGULATOR_RFLDO2
      mV_step:       0,
   },
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_HOPMODCTRL,      // BCM59001_REGULATOR_HCLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_UOPMODCTRL,      // BCM59001_REGULATOR_USBLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_IOPMODCTRL,      // BCM59001_REGULATOR_IOLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_MOPMODCTRL,      // BCM59001_REGULATOR_MSLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_LOPMODCTRL,      // BCM59001_REGULATOR_LCLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_SOPMODCTRL,      // BCM59001_REGULATOR_SIMLDO
      mV_step:       0,     
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59001_REG_IOSROPMODCTRL,   // BCM59001_REGULATOR_IOSR
      mV_step:       0,
   }
};

static BCM_PMU_Power_On_State_t gPowerOnState = PMU_Power_On_By_On_Button;


// This section is present to allow a guard time around charger
// insertion removal transients.
#define CHGRM_MSEC   100                     // Charger removal guard time
#define CHGINS_MSEC   20                     // Charger insertion guard time
static int wallChargerEOC = 0;               // wall charger eoc detected flag
static int usbChargerEOC = 0;                // usb charger eoc detected flag
static int isWallChargerPresent = 0;         // wall charger present flag
static int isUSBChargerPresent = 0;          // usb charger present flag
static struct timer_list eoc_chgrm_timer;    // removal guard timer
static struct timer_list eoc_chgins_timer;   // insertion guard timer

      
/* ---- Private Function Prototypes -------------------------------------- */

static void eoc_chgrm_timer_cback(unsigned long dummy);
static void eoc_chgins_timer_cback(unsigned long dummy);

static int bcm59001_chg_thread(void *data);
static int bcm59001_isr_thread(void *data);
static int bcm59001_process_interrupt( void );

/* Module functions */
static int bcm59001_module_init( void );
static void bcm59001_module_exit( void );

/* Common initialization routine */
static int bcm59001_init( void );

/* Interrupt handling functions */
static int bcm59001_irq_init(u8 *initial_int_status);

/* Interrupt service routine */
static irqreturn_t bcm59001_isr( void *dev_id );

/* Reset 8 second watchdog timer */
//static void bcm59001_reset8Second( void );

/* Get power on condition */
static BCM_PMU_Power_On_State_t bcm59001_get_power_on_state( void );

/* IOCTL handler */
static int bcm59001_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
static void bcm59001_poweroff( void );

/* Custom timed run function */
static void bcm59001_run( void );

/* Set log level for pmu debugging */
static void bcm59001_logLevel( int level );

/* Power regulator control */
static int bcm59001_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state);
static BCM_PMU_Regulator_State_t bcm59001_regulator_get_state(int regulatorID);

//static int bcm59001_regulator_set_voltage(int regulatorID, u32 mV);
//static u32 bcm59001_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);

static int bcm59001_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod);
static int bcm59001_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state);

//static int bcm59001_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step);
//static int bcm59001_vout_to_mV(int regulatorID, u8 vout, u32 *mV);

/* Charger control */
static void bcm59001_charger_start(int chargerID);
static void bcm59001_charger_stop(int chargerID);
static int bcm59001_charger_is_inserted(int *chargerID);

/* Event dispatcher */
static void bcm59001_event_notify(BCM59001_InterruptId_t irq_id);

/* I2C client address definitions */
static unsigned short normal_i2c[] = {BCM59001_I2C_BASE_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]       = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data bcm59001_i2c_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

/* PMU device operations */
static BCM_PMU_Operations_t bcm59001_ops =
{
   init: bcm59001_init,
   isr: bcm59001_isr,
   get_power_on_state: bcm59001_get_power_on_state,
   ioctl: bcm59001_ioctl,
   poweroff: bcm59001_poweroff,
   run: bcm59001_run,
   logLevel: bcm59001_logLevel,
   regulator:
   {
      set_state:     bcm59001_regulator_set_state,
      get_state:     bcm59001_regulator_get_state,
      set_voltage:   NULL,
      get_voltage:   NULL,
   },
   charger:
   {
      start: bcm59001_charger_start,
      stop: bcm59001_charger_stop,
      is_inserted: bcm59001_charger_is_inserted,
   },
   i2c_data: &bcm59001_i2c_data,
};

/* ---- Functions -------------------------------------------------------- */

static int pmu_write(u8 regAddr, u8 value)
{
   PMU_DEBUG(DBG_TRACE2, "Reg 0x%02x = 0x%02x\n", regAddr, value);
   return pmu_i2c_write(regAddr, value);
}
static int pmu_read(u8 regAddr)
{
   int value = pmu_i2c_read(regAddr);
   PMU_DEBUG(DBG_TRACE2,"Reg 0x%02x = 0x%02x\n", regAddr, value);
   return value;
}
/****************************************************************************
*
*  bcm59001_init
*
***************************************************************************/
static int bcm59001_init( void )
{
   int rc;
   u8 int_status[BCM59001_NUM_INT_REG];

   PMU_DEBUG(DBG_TRACE, "\n");

   /* Initialize IRQ handler */
   bcm59001_irq_init(int_status);

   /* change RFLD01 to 2.9V and RFLD02 to 2.5V */
   rc = pmu_write( BCM59001_REG_RFDOCTRL, (BCM59001_VAL_RFLD01_2_9V | BCM59001_VAL_RFLD02_2_5V) );

   /* disable watchdog timer, needed for A0 silicon only */
   // watchdog defaults to off on C1 silicon
   //rc |= pmu_write( BCM59001_REG_HOSTACT, BCM59001_BIT_WATCHDOG_DISABLE );

   /* properly trim the battery charging circuit, needed for A0 silicon bug work around */
   // Not needed for C1 silicon?
   //rc |= pmu_write( BCM59001_REG_OTPMBCTRM, 0x20 );

   /* set the charging current to be 500mA, trickle charge 50 mA */
   rc |= pmu_write( BCM59001_REG_MBCCTRL4, ( BCM59001_VAL_TC1_50MA | BCM59001_VAL_RC1_500MA ) );

   /* configure the HCLDO rail to 3.2V for WLAN_3.2 */
   rc |= pmu_write( BCM59001_REG_HCUSBDOCTRL, (BCM59001_VAL_HCLDO_3_2V | BCM59001_VAL_USBLDO_3_4V) );

   /* Set the power-on key delay */
   if (rc == 0)
   {
      int val;

      val = pmu_read(BCM59001_REG_ACDDB);
      if (val != -1)
      {
         val &= ~(BCM59001_VAL_PONKEYDEL_MASK << BCM59001_VAL_PONKEYDEL_SHIFT);
         val |= BCM59001_VAL_PONKEYDEL_8s;
         rc = pmu_write( BCM59001_REG_ACDDB, (u8)val);
      }
   }

   /* Save power on condition */
   if( int_status[0] & BCM59001_BIT_INT1_PONKEYF  )
   {
      gPowerOnState = PMU_Power_On_By_On_Button;
   }
   else if( int_status[1] & BCM59001_BIT_INT2_CHGINS )
   {
      gPowerOnState = PMU_Power_On_By_Charger;
   }
   else
   {
      gPowerOnState = PMU_Power_On_By_Restart;
   }

   /* Register IRQ handler */
   bcm59001_irq_register(BCM59001_IRQID_INT2_CHGINS,    bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_CHGRM,     bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_CHGERR,    bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_CHGEOC,    bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_USBINS,    bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_USBRM,     bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_USBEOC,    bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT2_MBCCHGERR, bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT1_PONKEYR,   bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT1_PONKEYF,   bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT1_PONKEYH,   bcm59001_event_notify);
   bcm59001_irq_register(BCM59001_IRQID_INT3_LOWBAT,    bcm59001_event_notify);
   
   /* Initialize auxiliary interfaces */
   rtc59001_init();
   led59001_init();
   headset59001_init();
    
   {
      BCM59001_chargers_t charger;
      rc = pmu_read(BCM59001_REG_ENV1);
      if ( rc != -1 )
      {
         if( (u8)rc & BCM59001_BIT_ENV1_CGPD )
         {
            PMU_DEBUG(DBG_TRACE, "wall charger detected\n");
            isWallChargerPresent = 1;
            charger = BCM59001_CHARGER_MAIN;
            bcm59001_charger_start(charger);
         }
         if ((u8)rc & BCM59001_BIT_ENV1_UBPD) 
         {
            PMU_DEBUG(DBG_TRACE, "USB charger detected\n");
            isUSBChargerPresent = 1;
            charger = BCM59001_CHARGER_USB;
            bcm59001_charger_start(charger);
         }
      }
   }

#if TEST_FG
   rc = pmu_read( BCM59001_REG_FGCTRL1 ); 
   rc |= BCM59001_REG_FGCTRL1_FGHOSTEN;
   pmu_write( BCM59001_REG_FGCTRL1, rc );    // Enable fuel gauge.
#endif



   init_timer(&eoc_chgrm_timer);
   eoc_chgrm_timer.function = eoc_chgrm_timer_cback;
   
   init_timer(&eoc_chgins_timer);
   eoc_chgins_timer.function = eoc_chgins_timer_cback;

   return 0;
}
      
/****************************************************************************
*
*  eoc_chgrm_timer - ignore EOC interrupts near charger remove events
*
***************************************************************************/
static void eoc_chgrm_timer_cback(unsigned long dummy)
{
   (void)dummy;
   gChgEvent = CHG_RM;
   wake_up_interruptible(&gChgEventQ);
}
/****************************************************************************
*
*  setupEOCIntMask - setup EOC interrupt mask bits based on state of chargers
*
***************************************************************************/
static void setupEOCIntMask(int isWallChargerPresent, int isUSBChargerPresent)
{
   int readval;
   int writeval;
   PMU_DEBUG(DBG_TRACE, "isWallChargerPresent=%d, isUSBChargerPresent=%d\n", isWallChargerPresent, isUSBChargerPresent);
   readval = pmu_read(BCM59001_REG_INT2M);
   if (readval == -1)
   {
      PMU_DEBUG(DBG_ERROR, "error reading INT2M\n");
      return;
   }
   writeval = readval;
   if (isWallChargerPresent)
   {
      writeval &= (~BCM59001_BIT_INT2_CHGEOC);
   }
   else
   {
      writeval |= BCM59001_BIT_INT2_CHGEOC;
   }
   if (isUSBChargerPresent)
   {
      writeval &= (~BCM59001_BIT_INT2_USBEOC);
   }
   else
   {
      writeval |= BCM59001_BIT_INT2_USBEOC;
   }
   if (readval != writeval) 
   {
      int rc = pmu_write( BCM59001_REG_INT2M, writeval);
      if (rc != 0) 
      {
         PMU_DEBUG(DBG_ERROR, "error writing to INT2M register\n");
      }
   }
}
/****************************************************************************
*
*  eoc_chgins_timer - timer callback to unmask EOC interrupts
*
***************************************************************************/
static void eoc_chgins_timer_cback(unsigned long dummy)
{
   (void) dummy;

   gChgEvent = CHG_INS;
   wake_up_interruptible(&gChgEventQ);
}

/****************************************************************************
*
*  bcm59001_irq_init
*
***************************************************************************/
static int bcm59001_irq_init(u8 *initial_int_status)
{
   int rc;
   int i;
   u8 int_status[BCM59001_NUM_INT_REG];

   /* get initial interrupt bits */
   for ( i = 0; i < BCM59001_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59001_REG_INT1 + i);
      if (rc == -1)
      {
         PMU_DEBUG(DBG_ERROR, "error reading interrupt registers - Init \n");
         return -EINVAL;
      }
      else
      {
         int_status[i] = (u8)rc;
      }
   }
   /* setup the interrupt mask for the wall adapter charger */
   rc |= pmu_write( BCM59001_REG_INT1M, 
         (u8)~( BCM59001_BIT_INT1_PONKEYR | BCM59001_BIT_INT1_PONKEYF |
                   BCM59001_BIT_INT1_PONKEYH ) );

   rc = pmu_write( BCM59001_REG_INT2M, (u8)~( INT2M_DEFAULT ) ); // Don't enable EOC ints until charger state tested

   rc |= pmu_write( BCM59001_REG_INT3M, 
         (u8)~( BCM59001_BIT_INT3_LOWBAT ) );

   if( rc != 0 )
   {
      PMU_DEBUG(DBG_ERROR, "error writing interrupt register -Init\n");
      return -EINVAL;
   }
   /* Fill IRQ function table with empty functions and
    * build the table that has bit positions for interrupts based on priority
    */
   for ( i = 0; i < BCM59001_NUM_IRQ; i++ )
   {
      bcm59001_IRQVectorTable[i] = NULL;
   }

   /* save initial int status */
   if (initial_int_status)
   {
      for (i = 0; i < BCM59001_NUM_INT_REG; i++)
      {
         initial_int_status[i] = int_status[i];
      }
   }

   /* Create ISR thread */
   sema_init(&gIsrSem, 0);
   init_completion(&gIsrExited);
   
   gIsrThreadPid = kernel_thread(bcm59001_isr_thread, (void *)PMU_BCM59001, 0);
   PMU_DEBUG(DBG_TRACE, "isr_thread started %lu\n", gIsrThreadPid);
   
   init_completion(&gChgExited);
   gChgThreadPid = kernel_thread(bcm59001_chg_thread, (void *)PMU_BCM59001, 0);
   PMU_DEBUG(DBG_TRACE, "chg_thread started %lu\n", gChgThreadPid);
   
   return 0;
}


/****************************************************************************
*
*  bcm59001_irq_register
*
*  irqId: ID of the IRQ to be registered
*  isrFunction: function to run for the particular IRQ
*
***************************************************************************/
int bcm59001_irq_register(BCM59001_InterruptId_t irqId, bcm59001_isr_t isrFunction)
{
   if (irqId >= BCM59001_NUM_IRQ)
   {
      PMU_DEBUG(DBG_ERROR, "irqId %d out of range\n", irqId);
      return -EINVAL;     
   }
   bcm59001_IRQVectorTable[irqId] = isrFunction;
   
   return 0;
}

/****************************************************************************
*
*  adjustThreadPriority
*
*  Adjust the thread priority to the specified level
*
***************************************************************************/
static void adjustThreadPriority( int requestedPriority )
{
   int rc;

   if (( current->policy != SCHED_FIFO ) || ( current->rt_priority != requestedPriority ))
   {
      struct sched_param param;

      param.sched_priority = requestedPriority;

      if (( rc = sched_setscheduler( current, SCHED_FIFO, &param )) == 0 )
      {
         PMU_DEBUG(DBG_TRACE, "%s priority set to %lu\n", current->comm, (unsigned long)current->rt_priority );
      }
      else
      {
         PMU_DEBUG(DBG_ERROR, "sched_setscheduler failed: %d\n", rc );
      }
   }

} /* adjustThreadPriority */

/****************************************************************************
*
*  bcm59001_chg_thread - charger state thread
*
***************************************************************************/
static int bcm59001_chg_thread(void *data)
{
   int rc;

   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   (void)data;
   daemonize("bcm59001_chg");
   PMU_DEBUG(DBG_TRACE, "\n");

   /* Adjust priority to be higher than any user mode threads but
    * lower than any network threads */
   adjustThreadPriority(1);

   while(1)
   {
      if (0 == wait_event_interruptible(gChgEventQ, gChgEvent))
      {
         switch (gChgEvent)
         {
            case CHG_INS:
               PMU_DEBUG(DBG_TRACE2, "called isWallChargerPresent=%d, isUSBChargerPresent=%d\n", 
                         isWallChargerPresent, isUSBChargerPresent);
            
               // read INT2 to clear out existing EOC interrupts
               rc = pmu_read(BCM59001_REG_INT2);
               if (rc == -1)
               {
                  PMU_DEBUG(DBG_ERROR, "error reading INT2\n");
               }
               setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);
               break;
               
            case CHG_RM:
            {
               BCM_PMU_Event_t event = PMU_NUM_EVENTS;
            
               if ((usbChargerEOC && !isUSBChargerPresent) || 
                   (wallChargerEOC && !isWallChargerPresent))
               {
                  PMU_DEBUG(DBG_TRACE2, "EOC ignored\n");
               }
               else
               {
                  PMU_DEBUG(DBG_TRACE, "EOC detected\n");
            
                  //Inform that the battery is fully charged.
                  if (wallChargerEOC && isWallChargerPresent) 
                  {
                     PMU_DEBUG(DBG_INFO,"Main charger full battery\n");
                     data = (void *)BCM59001_CHARGER_MAIN;
                     event = PMU_EVENT_BATTERY_FULL;
                     pmu_event_notify(PMU_BCM59001, event, data);
                  }
                  else if (usbChargerEOC && isUSBChargerPresent) 
                  {
                     PMU_DEBUG(DBG_INFO,"USB charger full battery\n");
                     event = PMU_EVENT_BATTERY_FULL;
                     data = (void *)BCM59001_CHARGER_USB;
                     pmu_event_notify(PMU_BCM59001, event, data);
                  }
               }
               wallChargerEOC = 0;
               usbChargerEOC = 0;
               break;
            }
            default:
               PMU_DEBUG(DBG_ERROR,"Bad gChgEvent state %d\n", gChgEvent);
               break;
         }
         gChgEvent = 0;
      }
   }

   PMU_DEBUG(DBG_ERROR, "Fatal. Thread should never exit.\n");

   complete_and_exit(&gChgExited, 0);

} /* bcm59001_isr_thread */

/****************************************************************************
*
*  bcm59001_isr_thread
*
***************************************************************************/
static int bcm59001_isr_thread(void *data)
{
   int rc;

   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   (void)data;
   daemonize("bcm59001_isr");
   PMU_DEBUG(DBG_TRACE, "\n");

   /* Adjust priority to be higher than any user mode threads but
    * lower than any network threads */
   adjustThreadPriority(1);

   while(1)
   {
      if ( down_interruptible (&gIsrSem) == 0 )
      {
         rc = bcm59001_process_interrupt();
         if (rc < 0)
         {
            PMU_DEBUG(DBG_ERROR, "Error %d processing interrupt.\n", rc);
         }
      }
      else
         break; //leave while
   }

   PMU_DEBUG(DBG_ERROR, "Fatal. Thread should never exit.\n");

   complete_and_exit(&gIsrExited, 0);

} /* bcm59001_isr_thread */

/****************************************************************************
*
*  bcm59001_process_interrupt
*
***************************************************************************/
static int bcm59001_process_interrupt( void )
{
   int rc;
   u8 intBits[BCM59001_NUM_INT_REG];
   u8 maskBits[BCM59001_NUM_INT_REG];
   u8 intStatus;
   u8 intMask[8];
   int i, k;
   u32 clk = timer_get_tick_count();

   /* read the interrupt status registers */
   for ( i = 0; i < BCM59001_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59001_REG_INT1 + i);
      if ( rc == -1 )
      {
         PMU_DEBUG(DBG_ERROR,"error reading interrupt registers - service.\n");
         return -EINVAL;
      }
      else
      {
         intBits[i] = (u8)rc;
      }
   }
   /* read the interrupt mask bit registers */
   for ( i = 0; i < BCM59001_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59001_REG_INT1M + i);
      if ( rc == -1 )
      {
         PMU_DEBUG(DBG_ERROR,"error reading interrupt mask registers - service.\n");
         return -EINVAL;
      }
      else
      {
         maskBits[i] = (u8)rc;
      }
   }

   intMask[0] = 0x01;
   intMask[1] = 0x02;
   intMask[2] = 0x04;
   intMask[3] = 0x08;
   intMask[4] = 0x10;
   intMask[5] = 0x20;
   intMask[6] = 0x40;
   intMask[7] = 0x80;

   for( k =0; k < BCM59001_NUM_INT_REG; k++ )
   {
      intStatus = (u8)(intBits[k] & (~maskBits[k]));
      for( i=0; i < 8; i++ )
      {
         if( intStatus & intMask[i] )
         {
            if( bcm59001_IRQVectorTable[ (k<<3)+i ] != NULL )
            {
               (*bcm59001_IRQVectorTable[ (k<<3)+i ]) ((k<<3)+i);
            }
         }
      }
   }

   clk = timer_get_tick_count() - clk;
   if (clk > pmu_max_isr_clk)
   {
      pmu_max_isr_clk = clk;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59001_isr
*
***************************************************************************/
static irqreturn_t bcm59001_isr( void *dev_id )
{
   (void)dev_id;

   up( &gIsrSem );
   return IRQ_HANDLED;
} /* bcm59001_isr */


/****************************************************************************
*
*  bcm59001_get_power_on_state
*
***************************************************************************/
static BCM_PMU_Power_On_State_t bcm59001_get_power_on_state( void )
{
   return gPowerOnState;
}


/****************************************************************************
*
*  bcm59001_ioctl
*
***************************************************************************/
static int bcm59001_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   switch ( cmd )
   {
       default:
       {
           return -ENOTTY;
       }
   }

   return 0;

} /* bcm59001_ioctl */

/****************************************************************************
*
*  bcm59001_poweroff
*
***************************************************************************/
static void bcm59001_poweroff( void )
{
   int rc = 0;
   uint8_t hostact;

   rc = pmu_read( BCM59001_REG_HOSTACT );
   if( rc != -1 )
   {
      hostact = (uint8_t)rc;
      hostact |= BCM59001_BIT_HOSTACT_HOSTDICOFF;
      rc = pmu_write(BCM59001_REG_HOSTACT, hostact );
   }
}

/****************************************************************************
*
*  bcm59001_run - platform specific run function, called from battmgr.
*  We are using the fuel gauge for demonstration purposes only. This code does
*  NOT account for charge cycle history, battery condition, temperature, learning
*  cycles, aging, voltage measurement for replacement batteries, etc.
*
***************************************************************************/
#if TEST_FG
// BCM59001
// AU = Accumulator units
// BR = Battery Rating in mAh
// AU = BR * 7200 * 8192 * 0.020 / 80
// AU = BR * 14745.6
// For our 950 mAh battery, we have
// AU = 950 * 14745.6 = 14008320
#define BATTERY_AU 14008320
#define BATTERY_CAPACITY ((70 * BATTERY_AU)/100)  // 70% for aging, charge cycles, temp, etc.    
static int battery_capacity = BATTERY_CAPACITY;
#endif

static void bcm59001_run( void )
{
#if TEST_FG
   int fgacc;
   int battery_remain_percent;


   fgacc = ((pmu_read(BCM59001_REG_FGACCM1) << 0) | 
            (pmu_read(BCM59001_REG_FGACCM2) << 8) |
            (pmu_read(BCM59001_REG_FGACCM3) << 16) |
            (pmu_read(BCM59001_REG_FGACCM4) << 24));


   pmu_write( BCM59001_REG_FGCTRL3, BCM59001_REG_FGCTRL3_FGSNAPSHOT );  // Freeze accum next time.

   // sign extend to 32 bits
   fgacc <<= 6;
   fgacc >>= 6;
   
   battery_capacity += fgacc;  // grows when charging, shrinks when draining

   if (battery_capacity > BATTERY_CAPACITY) 
   {
      battery_capacity = BATTERY_CAPACITY;
   }
   if (battery_capacity < 0) 
   {
      battery_capacity = 0;
   }
   battery_remain_percent = 100 - ((BATTERY_CAPACITY - battery_capacity) * 100 / BATTERY_CAPACITY);
   PMU_DEBUG(DBG_TRACE2,"capacity = %d = %d%% left, accum = %d = 0x%x\n",
      battery_capacity, 
      battery_remain_percent, 
      fgacc, (unsigned int)fgacc);
#endif      
}

/****************************************************************************
*
*  bcm59001_logLevel
*
***************************************************************************/
static void bcm59001_logLevel( int level )
{
   logLevel = level;
}

/****************************************************************************
*
*  bcm59001_regulator_get_state
*
*  Set the state of a regulator
*
***************************************************************************/
static int bcm59001_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;

   if (!bcm59001_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = bcm59001_state_to_opmod(regulatorID, state, &opmod);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state %d.\n", state);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_read(bcm59001_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return rc;
   }

   // update register
   val = opmod;

   if ( val != (u8)rc )
   {
      // write settings only if a change in value is detected
      rc = pmu_write(bcm59001_regulator_map[regulatorID].reg_addr, val);
   }
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* bcm59001_regulator_set_state */


/****************************************************************************
*
*  bcm59001_regulator_get_state
*
*  Retrieve the current state of a regulator
*
***************************************************************************/
static BCM_PMU_Regulator_State_t bcm59001_regulator_get_state(int regulatorID)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if (!bcm59001_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_read(bcm59001_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = bcm59001_opmod_to_state(regulatorID, (u8)rc, &state) ;
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* bcm59001_regulator_get_state */

/****************************************************************************
*
*  bcm59001_charger_start
*
***************************************************************************/
static void bcm59001_charger_start(int chargerID)
{
   int rc;
   
   PMU_DEBUG(DBG_INFO, "%d\n", chargerID);
   if ((chargerID != BCM59001_CHARGER_MAIN) && (chargerID != BCM59001_CHARGER_USB))
   {
      PMU_DEBUG(DBG_ERROR, "Bad chargerID %d\n", chargerID);
      return;
   }
   if (chargerID == BCM59001_CHARGER_MAIN)
   {
      /* set the trickle charge threshold to 3.6V (default 3.2) to ensure we can power up properly */
      rc = pmu_read( BCM59001_REG_MBCCTRL3 );
      rc = ( BCM59001_VAL_TRICKLE_3_2V | (unsigned char)(rc & 0x0F) );
      rc = pmu_write( BCM59001_REG_MBCCTRL3, (uint8_t)rc );
   }
   else if (chargerID == BCM59001_CHARGER_USB)
   {
      /* set the trickle charge threshold to 3.6V (default 3.2) to ensure we can power up properly */
      rc = pmu_read( BCM59001_REG_MBCCTRL5 );
      rc = ( BCM59001_VAL_TRICKLE_3_6V | (unsigned char)(rc & 0x0F) );
      pmu_write( BCM59001_REG_MBCCTRL5, (uint8_t)rc );

      // Rapid charge at 500 mA, trickle charge is set by OTP (50 mA)
      rc = pmu_read( BCM59001_REG_MBCCTRL6 );
      rc &= 0x0f;
      rc |= BCM59001_VAL_RC1_500MA;
      pmu_write( BCM59001_REG_MBCCTRL6, (uint8_t)rc);
   }
   
   // Bit 6 can clear by itself when charging complete, so don't trust the read value
   rc = pmu_read( BCM59001_REG_MBCCTRL2 );  // Get timeout settings in lower bits
   rc &= 0x3f; // mask out bits that can't be read reliably
   pmu_write( BCM59001_REG_MBCCTRL2, (uint8_t)rc );   // Reset state machine
   rc |= (BCM59001_BIT_MBCCTRL2_MBCHOSTRC + BCM59001_BIT_MBCCTRL2_MBCHOSTEN);
   pmu_write( BCM59001_REG_MBCCTRL2, (uint8_t)rc );   // Enable rapid charge mode and enable charger

   // Enable the battery full interupts here 
   setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);
}

/****************************************************************************
*
*  bcm59001_charger_stop
*
***************************************************************************/
static void bcm59001_charger_stop(int chargerID)
{
   int rc;

   PMU_DEBUG(DBG_INFO, "%d\n", chargerID);
   
   if ((chargerID != BCM59001_CHARGER_MAIN) && (chargerID != BCM59001_CHARGER_USB))
   {
      PMU_DEBUG(DBG_ERROR,"Error: Bad chargerID %d\n", chargerID);
      return;
   }
   rc = pmu_read( BCM59001_REG_MBCCTRL2 );  // Get timeout settings in lower bits
   rc &= ~(BCM59001_BIT_MBCCTRL2_MBCHOSTRC + BCM59001_BIT_MBCCTRL2_MBCHOSTEN);
   pmu_write( BCM59001_REG_MBCCTRL2, (uint8_t)rc );   // Disable charging
}

/****************************************************************************
*
*  bcm59001_charger_is_inserted
*
***************************************************************************/
static int bcm59001_charger_is_inserted(int *chargerID)
{
   int rc;

   rc = pmu_read(BCM59001_REG_ENV1);
   if ( rc != -1 )
   {
      if( (u8)rc & BCM59001_BIT_ENV1_CGPD )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59001_CHARGER_MAIN;
            return 1;
         }
      }
      if( (u8)rc & BCM59001_BIT_ENV1_UBPD )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59001_CHARGER_USB;
            return 1;
         }
      }
   }
   return 0;
}

/****************************************************************************
*
*  bcm59001_event_notify
*
***************************************************************************/
static void bcm59001_event_notify(BCM59001_InterruptId_t irq_id)
{
   BCM_PMU_Event_t event = PMU_NUM_EVENTS;
   void *data = NULL;
   int val;

   //PMU_DEBUG("irq_id=%d\n", irq_id);
   switch (irq_id)
   {
      // Onkey events
      case BCM59001_IRQID_INT1_PONKEYR:
         // Clear key-lock bit to allow auto shutoff in case software stops running
         val = pmu_read(BCM59001_REG_PONKEYBDB);
         if (val != -1)
            pmu_write(BCM59001_REG_PONKEYBDB, (u8)(val & ~BCM59001_VAL_KEYLOCK));
         event = PMU_EVENT_ONKEY_RISE;
         break;
      case BCM59001_IRQID_INT1_PONKEYF:
         // Set key-lock bit to prevent auto shutoff after power-on key delay
         val = pmu_read(BCM59001_REG_PONKEYBDB);
         if (val != -1)
            pmu_write(BCM59001_REG_PONKEYBDB, (u8)(val | BCM59001_VAL_KEYLOCK));
         event = PMU_EVENT_ONKEY_FALL;
         break;
      case BCM59001_IRQID_INT1_PONKEYH:
         event = PMU_EVENT_ONKEY_1S_HOLD;
         break;

      // Battery and charger events
      case BCM59001_IRQID_INT3_LOWBAT:
         event = PMU_EVENT_BATTERY_LOW;
         break;

      case BCM59001_IRQID_INT2_USBINS:
         isUSBChargerPresent = 1;
         PMU_DEBUG(DBG_INFO,"USB charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59001_CHARGER_USB;

         // start a one-shot timer set to expire later to enable EOC interrupt
         PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgins timer now\n", CHGINS_MSEC);
         mod_timer(&eoc_chgins_timer, jiffies + msecs_to_jiffies(CHGINS_MSEC));
         isUSBChargerPresent = 1;
         break;
      case BCM59001_IRQID_INT2_USBRM:
         PMU_DEBUG(DBG_INFO,"USB charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59001_CHARGER_USB;
         isUSBChargerPresent = 0;
         break;
      case BCM59001_IRQID_INT2_USBEOC:
         if(!isUSBChargerPresent)
         {
            PMU_DEBUG(DBG_TRACE2,"USB charger EOC ignored\n");
         }
         else
         {
            // start a one-shot timer set to expire later to see if there is a 
            // charger removal event
            PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgrm timer now\n", CHGRM_MSEC);
            mod_timer(&eoc_chgrm_timer, jiffies + msecs_to_jiffies(CHGRM_MSEC));
            usbChargerEOC = 1;
         }
         // Disable the EOC interrupt here to avoid nested batful interrupts 
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);

#if TEST_FG
         pmu_write( BCM59001_REG_FGCTRL1, BCM59001_REG_FGCTRL1_FGHOSTEN );    // Enable fuel gauge.
         pmu_write( BCM59001_REG_FGCTRL3, BCM59001_REG_FGCTRL3_FGCAL );       // Calibrate on full battery.
         pmu_write( BCM59001_REG_FGCTRL3, BCM59001_REG_FGCTRL3_FGSNAPSHOT );  // Freeze accum next time.
         battery_capacity = BATTERY_CAPACITY;
#endif         
         break;
         
      case BCM59001_IRQID_INT2_CHGINS:
         PMU_DEBUG(DBG_INFO,"Main charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59001_CHARGER_MAIN;
         
         
         // start a one-shot timer set to expire later to enable EOC interrupt
         PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgins timer now\n", CHGINS_MSEC);
         mod_timer(&eoc_chgins_timer, jiffies + msecs_to_jiffies(CHGINS_MSEC));
         isWallChargerPresent = 1;
         break;
      case BCM59001_IRQID_INT2_CHGRM:
         PMU_DEBUG(DBG_INFO,"Main charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59001_CHARGER_MAIN;
         isWallChargerPresent = 0;
         break;
      case BCM59001_IRQID_INT2_CHGEOC:
         if(!isWallChargerPresent)
         {
            PMU_DEBUG(DBG_TRACE2,"Main charger EOC ignored\n");
         }
         else
         {
            // start a one-shot timer set to expire later to see if there is a 
            // charger removal event
            PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgrm timer now\n", CHGRM_MSEC);
            mod_timer(&eoc_chgrm_timer, jiffies + msecs_to_jiffies(CHGRM_MSEC));
            wallChargerEOC = 1;
         }
         // Disable the EOC interrupt here to avoid nested batful interrupts 
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);
          
#if TEST_FG
         pmu_write( BCM59001_REG_FGCTRL1, BCM59001_REG_FGCTRL1_FGHOSTEN );    // Enable fuel gauge.
         pmu_write( BCM59001_REG_FGCTRL3, BCM59001_REG_FGCTRL3_FGCAL );       // Calibrate on full battery.
         pmu_write( BCM59001_REG_FGCTRL3, BCM59001_REG_FGCTRL3_FGSNAPSHOT );  // Freeze accum next time.
         battery_capacity = BATTERY_CAPACITY;
#endif         
         break;
         
      case BCM59001_IRQID_INT2_CHGERR:
         PMU_DEBUG(DBG_ERROR,"Charger voltage is greater than over-voltage threshold\n");
         /* We indicate full battery when error to stop charging */
         event = PMU_EVENT_BATTERY_FULL;
         break;
      case BCM59001_IRQID_INT2_MBCCHGERR:
         PMU_DEBUG(DBG_ERROR,"Main battery charging timeout error\n");
         /* We indicate full battery when error to stop charging */
         event = PMU_EVENT_BATTERY_FULL;
         break;

      default:
         break;
   }

#if 1
   // Notify PMU
   if (event != PMU_NUM_EVENTS)
   {
      //PMU_DEBUG("event=%d data=%p\n", event, data);
      pmu_event_notify(PMU_BCM59001, event, data);
   }
#endif
}

/****************************************************************************
*
*  bcm59001_state_to_opmod
*
***************************************************************************/
static int bcm59001_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod)
{
   if (!opmod)
      return -1;

   switch (state)
   {
      case PMU_Regulator_Off:
         *opmod = BCM59001_BIT_REG_OFF;
         break;
      case PMU_Regulator_On:
         *opmod = BCM59001_BIT_REG_ON;
         break;
      case PMU_Regulator_Eco:
         *opmod = BCM59001_BIT_REG_ECO;
         break;
      default:
         return -1;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59001_opmod_to_state
*
***************************************************************************/
static int bcm59001_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state)
{
   if (!state)
      return -1;

   switch (opmod)
   {
      case BCM59001_BIT_REG_OFF:
         *state = PMU_Regulator_Off;
         break;
      case BCM59001_BIT_REG_ON:
         *state = PMU_Regulator_On;
         break;
      case BCM59001_BIT_REG_ECO:
         *state = PMU_Regulator_Eco;
         break;
      default:
         return -1;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59001_module_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init bcm59001_module_init( void )
{
   printk( banner );

   PMU_DEBUG(DBG_TRACE,"register with PMU module\n");
   return pmu_register_device(PMU_BCM59001, &bcm59001_ops);
}


/****************************************************************************
*
*  bcm59001_module_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit bcm59001_module_exit( void )
{
   PMU_DEBUG(DBG_TRACE,"module_exit called\n");
   del_timer(&eoc_chgrm_timer);
   del_timer(&eoc_chgins_timer);
}


fs_initcall(bcm59001_module_init);
module_exit(bcm59001_module_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM59001 Driver");


