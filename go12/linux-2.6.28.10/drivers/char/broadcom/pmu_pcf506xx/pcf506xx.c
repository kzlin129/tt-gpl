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
*  pcf506xx.c
*
*  PURPOSE:
*
*     This implements the common portion of the drivers for the Philips PFC506xx chips.
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

/* Include external interfaces */
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_pcf506xx.h>
#include <linux/broadcom/timer.h>

/* Include internal interfaces */
#include "pcf506xx.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

/* Backup battery charger default value */
#define BBCE_INIT_VALUE (PCF506XX_BIT_BBCE|PCF506XX_BIT_BBCR|PCF506XX_BIT_BBCC400|PCF506XX_BIT_BBCV)

/* Interrupt Masks: 0 for enabled */
#define PCF506XX_DEFAULT_INT1M ((u8)~(PCF506XX_BIT_ONKEY1S | \
                                      PCF506XX_BIT_ONKEYF | \
                                      PCF506XX_BIT_ONKEYR | \
                                      PCF506XX_BIT_LOWBAT) )

#define PCF506XX_DEFAULT_INT2M ((u8)~(PCF506XX_BIT_CHGWD | \
                                      PCF506XX_BIT_CHGEVT | \
                                      PCF506XX_BIT_REC2HF | \
                                      PCF506XX_BIT_REC2LF | \
                                      PCF506XX_BIT_REC2HR))

#define PCF506XX_DEFAULT_INT3M ((u8)~(PCF506XX_BIT_CHGRM | \
                                      PCF506XX_BIT_CHGINS))

/* INT4M does not exist for PCF50603 */
#define PCF50611_DEFAULT_INT4M ((u8)~(PCF506XX_BIT_CHGRES | \
                                      PCF506XX_BIT_UCHGRM | \
                                      PCF506XX_BIT_UCHGINS | \
                                      PCF506XX_BIT_BATFUL ))

/* ---- Private Variables ------------------------------------------------ */

static long gIsrThreadPid = 0;
struct completion gIsrExited;
struct semaphore gIsrSem;

static pcf506xx_isr_t pmuIRQVectorTable[PCF506XX_MAX_NUM_IRQS];
static uint32_t pmu_intPriorityBit[PCF506XX_MAX_NUM_IRQS];

/* table of interrupt IDs (0 to 31) ordered based on priority */
static const PCF506XX_InterruptId_t pmu_intPriority[] =
{
    PCF506XX_IRQID_INT1LOWBAT,   // First interrupt served
    PCF506XX_IRQID_INT1SECOND,
    PCF506XX_IRQID_INT1MINUTE,
    PCF506XX_IRQID_INT1ALARM,
    PCF506XX_IRQID_INT1ONKEYF,   // Three ONKEY int process order:
    PCF506XX_IRQID_INT1ONKEY1S,  // Fall, 1S, Rise
    PCF506XX_IRQID_INT1ONKEYR,   // Do not change the order.
    PCF506XX_IRQID_INT1THS,

    PCF506XX_IRQID_INT2REC1R,
    PCF506XX_IRQID_INT2REC1F,
    PCF506XX_IRQID_INT2REC2HF,
    PCF506XX_IRQID_INT2REC2LF,
    PCF506XX_IRQID_INT2REC2LR,
    PCF506XX_IRQID_INT2REC2HR,
    PCF506XX_IRQID_INT2CHGEVT,
    PCF506XX_IRQID_INT2CHGWD,

    PCF506XX_IRQID_INT3SIMUV,
    PCF506XX_IRQID_INT3INSERT,
    PCF506XX_IRQID_INT3EXTRACT,
    PCF506XX_IRQID_INT3MUTE,
    PCF506XX_IRQID_INT3EARLY,
    PCF506XX_IRQID_INT3SIMRDY,
    PCF506XX_IRQID_INT3CHGINS,
    PCF506XX_IRQID_INT3CHGRM,

    PCF506XX_IRQID_INT4THLIMON,
    PCF506XX_IRQID_INT4THLIMOFF,
    PCF506XX_IRQID_INT4BATFUL,
    PCF506XX_IRQID_INT4CHGRES,
    PCF506XX_IRQID_INT4BATTMFLT,
    PCF506XX_IRQID_INT4BATTMOK,
    PCF506XX_IRQID_INT4UCHGRM,
    PCF506XX_IRQID_INT4UCHGINS,

};

/* output voltage mapping table */
static u32 pcf506xx_vout_to_mV_map[] =
{
   0,
   0,
   0,
   1200,
   1300,
   1400,
   1500,
   1600,
   1700,
   1800,
   1900,
   2000,
   2100,
   2200,
   2300,
   2400,
   2500,
   2600,
   2700,
   2800,
   2900,
   3000,
   3100,
   3200,
};

/* charge pump output voltage mapping table */
static u32 pcf506xx_cpv_to_mV_map[] =
{
   3500,
   4000,
   4500,
   5000,
};

/* ---- Private Function Prototypes -------------------------------------- */

static int pcf506xx_isr_thread(void *data);
static int pcf506xx_process_interrupt(BCM_PMU_Chip_t chip);

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  pcf506xx_common_init
*
***************************************************************************/
int pcf506xx_common_init(BCM_PMU_Chip_t chip)
{
   int rc;

   /* turn on backup battery charger */
   rc = pmu_i2c_write(PCF506XX_REG_BBCC(chip), BBCE_INIT_VALUE);

   /* change turn off threshold voltage to 3.3 V */
   rc |= pmu_i2c_write(PCF506XX_REG_BVMC(chip), PCF506XX_BIT_THRSHLD_330);

   /* initialize the on button debounce time */
   rc |= pmu_i2c_write(PCF506XX_REG_DEBC(chip), PCF506XX_BIT_ONKEYDB62 );

   if (rc != 0)
   {
      return -EINVAL;
   }

   return 0;
} /* pcf506xx_common_init */


/****************************************************************************
*
*  pcf506xx_irq_init
*
***************************************************************************/
int pcf506xx_irq_init(BCM_PMU_Chip_t chip, u8 *initial_int_status)
{
   int rc;
   int i;
   u8 int_status[PCF506XX_MAX_NUM_INT_REGS];

   /* get initial interrupt bits */
   rc = pmu_i2c_read_bytes(PCF506XX_REG_INT1(chip), int_status, PCF506XX_NUM_INT_REGS(chip));
   if ( rc < 0)
   {
      printk("pcf506xx_irq_init: failed to read int status\n");
      return rc;
   }
   else
   {
      for ( i = 0; i < PCF506XX_NUM_INT_REGS(chip); i++ )
         printk("pcf506xx_irq_init: initial interrupt bits[%d] = 0x%x\n", i, int_status[i]);
   }

   /* Initialize interrupt masks */
   rc = pmu_i2c_write(PCF506XX_REG_INT1M(chip), PCF506XX_DEFAULT_INT1M);
   rc |= pmu_i2c_write(PCF506XX_REG_INT2M(chip), PCF506XX_DEFAULT_INT2M);
   rc |= pmu_i2c_write(PCF506XX_REG_INT3M(chip), PCF506XX_DEFAULT_INT3M);
   if ( chip == PMU_PCF50611 )
   {
      rc |= pmu_i2c_write(PCF50611_REG_INT4M, PCF50611_DEFAULT_INT4M);
   }
   if (rc != 0)
   {
      printk("pcf506xx_irq_init: failed to write int mask\n");
      return -EINVAL;
   }

   /* Fill IRQ function table with empty functions and
    * build the table that has bit positions for interrupts based on priority
    */
   for ( i = 0; i < PCF506XX_MAX_NUM_IRQS; i++ )
   {
      pmuIRQVectorTable[i] = NULL;
      pmu_intPriorityBit[i] = 1 << pmu_intPriority[i];
   }

   /* save initial int status */
   if (initial_int_status)
   {
      for (i = 0; i < PCF506XX_NUM_INT_REGS(chip); i++)
      {
         initial_int_status[i] = int_status[i];
      }
   }

   /* Create ISR thread */
   sema_init(&gIsrSem, 0);
   init_completion(&gIsrExited);
   gIsrThreadPid = kernel_thread(pcf506xx_isr_thread, (void *)chip, 0);
   printk("pcf506xx_irq_init: isr_thread started %lu\n", gIsrThreadPid);

   return 0;

} /* pcf506xx_irq_init */


/****************************************************************************
*
*  pcf506xx_irq_register
*
***************************************************************************/
int pcf506xx_irq_register(BCM_PMU_Chip_t chip, PCF506XX_InterruptId_t irqId, pcf506xx_isr_t isrFunction)
{
   if (irqId >= PCF506XX_NUM_IRQS(chip))
   {
      printk("pcf506xx_IRQRegister: irqId %d out of range\n", irqId);
      return -EINVAL;
   }
   pmuIRQVectorTable[irqId] = isrFunction;

   return 0;
} /* pcf506xx_irq_register */


/****************************************************************************
*
*  pcf506xx_reset8Second
*
*  Reset the 8 second time-out timer in the on-off control module
*
***************************************************************************/
void pcf506xx_reset8Second(BCM_PMU_Chip_t chip)
{
   int rc;
   u8 oocc;

   rc = pmu_i2c_read(PCF506XX_REG_OOCC(chip));
   if ( rc != -1 )
   {
      oocc = (u8)rc;
      /* set the tot_rst bit to turn off the timer */
      oocc |= PCF506XX_BIT_TOT_RST;
      pmu_i2c_write(PCF506XX_REG_OOCC(chip), oocc);
   }
} /* pcf506xx_reset8Second */


/****************************************************************************
*
*  adjustThreadPriority
*
*  Reset the 8 second time-out timer in the on-off control module
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
         printk( "%s priority set to %lu\n", current->comm, (unsigned long)current->rt_priority );
      }
      else
      {
         printk( "sched_setscheduler failed: %d\n", rc );
      }
   }

} // AdjustThreadPriority

/****************************************************************************
*
*  pcf506xx_isr_thread
*
***************************************************************************/
static int pcf506xx_isr_thread(void *data)
{
   int rc;

   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   daemonize("pcf506xx_isr");
   printk("pcf506xx_isr_thread\n");

   /* Adjust priority to be higher than any user mode threads but
    * lower than the network threads */
   adjustThreadPriority(1);

   while(1) {
      if ( down_interruptible (&gIsrSem) == 0 )
      {
         rc = pcf506xx_process_interrupt((BCM_PMU_Chip_t)data);
         if (rc < 0)
         {
            printk("pcf506xx_isr_thread: Error %d processing interrupt.\n", rc);
         }
      }
      else
         break; //leave while
   }

   printk("pcf506xx_isr_thread: Fatal. Thread should never exit.\n");

   complete_and_exit(&gIsrExited, 0);

} /* pcf506xx_isr_thread */


/****************************************************************************
*
*  pcf506xx_process_interrupt
*
***************************************************************************/
static int pcf506xx_process_interrupt(BCM_PMU_Chip_t chip)
{
   int rc;
   u8 intBits[PCF506XX_MAX_NUM_INT_REGS];
   u8 maskBits[PCF506XX_MAX_NUM_INT_REGS];
   u32 intStatus;
   u32 intMask;
   int i;
   u32 clk = timer_get_tick_count();

   memset(intBits, 0, sizeof(intBits));
   memset(maskBits, 0, sizeof(maskBits));

   /* read the interrupt status registers */
   rc = pmu_i2c_read_bytes(PCF506XX_REG_INT1(chip), intBits, PCF506XX_NUM_INT_REGS(chip));
   if ( rc < 0)
   {
      printk("pcf506xx_isr - error %d reading interrupt registers.\n", rc);
      return -EINVAL;
   }

   /* read the interrupt mask bit registers */
   rc = pmu_i2c_read_bytes(PCF506XX_REG_INT1M(chip), maskBits, PCF506XX_NUM_INT_REGS(chip));
   if ( rc < 0)
   {
      printk("pcf506xx_isr - error %d reading interrupt registers.\n", rc);
      return -EINVAL;
   }

   /* make one big interrupt status register */
   intStatus = intBits[0] | (intBits[1] << 8) | (intBits[2] << 16) | (intBits[3] << 24);

   /* make one big interrupt mask register */
   intMask = maskBits[0] | (maskBits[1] << 8) | (maskBits[2] << 16) | (maskBits[3] << 24);

   /* apply mask, note that 0 means enabled in the mask */
   intStatus &= ~intMask;

   /* run ISR's for each interrupt that is not masked out */
   for ( i = 0; i < PCF506XX_NUM_IRQS(chip); i++ )
   {
      if ( intStatus & pmu_intPriorityBit[i] )
      {
         if ( pmuIRQVectorTable[pmu_intPriority[i]] != NULL )
         {
            (*pmuIRQVectorTable[pmu_intPriority[i]])(pmu_intPriority[i]);
         }
      }
   }

#if 0
   ONE_LOG(ONE_LOG_TRACE,("PMU - intStatus 1 = 0x%x\n",intBits[0]));
   ONE_LOG(ONE_LOG_TRACE,("PMU - intStatus 2 = 0x%x\n",intBits[1]));
   ONE_LOG(ONE_LOG_TRACE,("PMU - intStatus 3 = 0x%x\n",intBits[2]));
   ONE_LOG(ONE_LOG_TRACE,("PMU - intStatus 4 = 0x%x\n",intBits[3]));
#endif

   clk = timer_get_tick_count() - clk;
   if (clk > pmu_max_isr_clk)
   {
      pmu_max_isr_clk = clk;
   }

   return 0;

} /*pcf506xx_process_interrupt */


/****************************************************************************
*
*  pcf506xx_isr
*
***************************************************************************/
irqreturn_t pcf506xx_isr(BCM_PMU_Chip_t chip)
{
   up( &gIsrSem );
   return IRQ_HANDLED;
} /* pcf506xx_isr */


/****************************************************************************
*
*  pcf506xx_ioctl
*
***************************************************************************/
int pcf506xx_ioctl( BCM_PMU_Chip_t chip, struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   switch ( cmd )
   {
       default:
       {
           return -ENOTTY;
       }
   }

   return 0;

} /* pcf506xx_ioctl */


/****************************************************************************
*
*  pcf506xx_poweroff
*
***************************************************************************/
void pcf506xx_poweroff ( BCM_PMU_Chip_t chip )
{
   int rc = 0;
   u8 oocc;

   rc = pmu_i2c_read(PCF506XX_REG_OOCC(chip));
   if( rc != -1 )
   {
      oocc = (u8)rc;
      oocc |= PCF506XX_BIT_GO_STDBY;
      rc = pmu_i2c_write(PCF506XX_REG_OOCC(chip), oocc);
   }

} /* pcf506xx_poweroff */


/****************************************************************************
*
*  pcf506xx_state_to_opmod
*
***************************************************************************/
int pcf506xx_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod)
{
   if (!opmod)
      return -1;

   switch (state)
   {
      case PMU_Regulator_Off:
         *opmod = PCF506XX_BIT_REG_OFF;
         break;
      case PMU_Regulator_On:
         *opmod = PCF506XX_BIT_REG_ON;
         break;
      case PMU_Regulator_Eco:
         *opmod = PCF506XX_BIT_REG_ECO;
         break;
      default:
         return -1;
   }
   return 0;
}


/****************************************************************************
*
*  pcf506xx_opmod_to_state
*
***************************************************************************/
int pcf506xx_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state)
{
   if (!state)
      return -1;

   switch (opmod & PCF506XX_BIT_REG_OPMOD_MASK)
   {
      case PCF506XX_BIT_REG_OFF:
         *state = PMU_Regulator_Off;
         break;
      case PCF506XX_BIT_REG_ON:
         *state = PMU_Regulator_On;
         break;
      case PCF506XX_BIT_REG_ECO:
         *state = PMU_Regulator_Eco;
         break;
      default:
         return -1;
   }
   return 0;
}


/****************************************************************************
*
*  pcf506xx_mV_to_vout
*
***************************************************************************/
int pcf506xx_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step)
{
   u32 *vout_to_mV_map;
   int map_size;
   int i;

   if (!vout)
      return -1;

   // Validate input mV
   if ((mV < min_mV) || (mV > max_mV) || ((mV - min_mV) % mV_step))
   {
      printk("pcf506xx: invalid %d mV setting for regulator %d.\n", mV, regulatorID);
      return -1;
   }

   // Set mapping tables
   if (regulatorID == PCF506XX_REGULATOR_CP)
   {
      vout_to_mV_map = pcf506xx_cpv_to_mV_map;
      map_size = sizeof(pcf506xx_cpv_to_mV_map)/sizeof(pcf506xx_cpv_to_mV_map[0]);
   }
   else
   {
      vout_to_mV_map = pcf506xx_vout_to_mV_map;
      map_size = sizeof(pcf506xx_vout_to_mV_map)/sizeof(pcf506xx_vout_to_mV_map[0]);
   }

   // Find matching voltage in table
   for (i = 0; i < map_size; i++)
   {
      if (vout_to_mV_map[i] == mV)
      {
         *vout = i;
         return 0;
      }
   }

   printk("pcf506xx: corrupt mapping table.\n");
   return -1;
}


/****************************************************************************
*
*  pcf506xx_vout_to_mV
*
***************************************************************************/
int pcf506xx_vout_to_mV(int regulatorID, u8 vout, u32 *mV)
{
   u32 *vout_to_mV_map;
   int map_size;
   u8 val;

   if (!mV)
      return -1;

   // Set mapping tables
   if (regulatorID == PCF506XX_REGULATOR_CP)
   {
      vout_to_mV_map = pcf506xx_cpv_to_mV_map;
      map_size = sizeof(pcf506xx_cpv_to_mV_map)/sizeof(pcf506xx_cpv_to_mV_map[0]);
      val = (vout & PCF506XX_BIT_REG_CPV_MASK) >> PCF506XX_BIT_REG_CPV_SHIFT;
   }
   else
   {
      vout_to_mV_map = pcf506xx_vout_to_mV_map;
      map_size = sizeof(pcf506xx_vout_to_mV_map)/sizeof(pcf506xx_vout_to_mV_map[0]);
      val = (vout & PCF506XX_BIT_REG_VOUT_MASK) >> PCF506XX_BIT_REG_VOUT_SHIFT;
   }

   // Mapping register value to voltage
   if (val >= map_size)
   {
      printk("pcf506xx: vout out of range\n");
      *mV = 0;
      return -1;
   }

   *mV = vout_to_mV_map[vout];
   return 0;
}

