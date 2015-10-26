/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
*  bcm59040.c
*
*  PURPOSE:
*
*     This implements the BCM59040 chip specific portion of the pmu driver.
*
*  NOTES:
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>

/* Include external interfaces */
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>
#include <linux/broadcom/timer.h>
#include <linux/broadcom/bcm_adc.h>

#include <asm/arch/hw_cfg.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/atomic.h>
#include <linux/power_supply.h>

/* Include internal interfaces */
#include <asm/arch/bcm59040.h>
#include "rtc.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

// INT2M Setting - handle EOC interrupts separately based on charger state
#define INT2M_DEFAULT 0

#define DISABLE_ALL_INTS 0xFF

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 0

#define DBG_ERROR  0x01
#define DBG_INFO   0x02
#define DBG_TRACE  0x04
#define DBG_TRACE2 0x08
#define DBG_DATA   0x10
#define DBG_DATA2  0x20

//#define DBG_DEFAULT_LEVEL  (DBG_ERROR)
#define DBG_DEFAULT_LEVEL  (DBG_ERROR|DBG_INFO|DBG_TRACE|DBG_TRACE2)

#if DEBUG
   #define PMU_DEBUG(level,fmt,args...) do { if (level & logLevel) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#else
   #define PMU_DEBUG(level,fmt,args...)
#endif


/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "BCM59040 Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

static long gIsrThreadPid = 0;
static struct completion gIsrExited;
static struct semaphore gIsrSem;

static int isWallChargerPresent = 0;         // wall charger present flag
static int isUSBChargerPresent = 0;          // usb charger present flag

static int TempKeyLockBitSet = 0;          // KEYLOCK bit set flag
static int PonkeyONOFF = 0;            // PONKEYONOFF bit set flag
static int PonkeyKeylockOTP = 0;        // OTP value of keylock bit
atomic_t bcm59040_batt_status=ATOMIC_INIT( POWER_SUPPLY_HEALTH_GOOD );		// Battery status.

static int logLevel = DBG_DEFAULT_LEVEL;

static unsigned int bcm59040_module_gpio = 0;

static bcm59040_isr_t bcm59040_IRQVectorTable[BCM59040_NUM_IRQ];

void bcm476x_rtc_update(int intType);

static int bcm59040_init_charging_mode(void);

/* interrupt masks to be initialized */
static u8 bcm59040_IntMask[BCM59040_NUM_INT_REG] =
{
#ifndef CONFIG_BCM_PMU_BCM59040_B0
    0xC0, // int1M: PMUTOOWARM, reserved
#else
    0x4F,	// int1M: PMUTOOWARM, bit7 is RESTARTON for B0 + Mask PONKEY-related interrupts
		// since we don't get the powerbutton interrupt via the PMU. 
#endif
    0x0,
#ifndef CONFIG_BCM_PMU_BCM59040_B0
    0x28,  // int3M: bit 3: VCHGRCLSP is reserved for A0; bit 5: CHGERRDIS also needs mask due to bug in A0
#else
    0x00,  // int3M: bit 3: CHGWDTALARM for B0;
#endif
    0xC0,  // int4M: BBLOW, FGC
#if defined(CONFIG_RTC_DRV_BCM59040)
    0x00,  // int5M: RTC1S/60S, RTCA1
#else
	0x60,
#endif
#ifndef CONFIG_BCM_PMU_BCM59040_B0
    0x10,  // bug in A0: so mask MBOV interrupt
#else
    0X00,  // Can use for B0
#endif
    0x0,
    0xFF, // int8M: individual SS channel interrupts
    0x03  // int9M: individual SS channel interrupts
};

struct pmu_platform_map	*pmu_platform_map;
#define bcm59040_regulator_map ((bcm59040_regulator_map_t *)pmu_platform_map->regulator_map)

typedef enum {
    OTG_BOOST_STAT,        // otg boost regulator status
    OTG_BLOCK_STAT         // otg block status
} OTG_Stat_t;

typedef struct
{
    int hw_otg_ctrl;     // need to initialize this;
    int float_legacy;   // if true, float_legacy is 1; else, 0
    BCM_PMU_OTG_Role_t otg_role;   // need to initialize this during pmu_init
    int otg_block_enabled;
    int otg_boost_enabled;
} OTG_Maintain_t;

typedef struct
{
    int chargerEOC;
    int in_maintchrg_mode;
    BCM_PMU_USB_Charger_Type_t  charger_type;
} MBC_Maintain_t;

static OTG_Maintain_t otg_maintain;
static MBC_Maintain_t mbc_maintain;

#ifdef CONFIG_BCM_PMU_BCM59040_B0
static int pwmled_sw_ctrl_enable = 0;   /* initialize to disabled; only matters for pwmled 2 & 3 */
#endif

/* fuel gauge */
typedef struct
{
    BCM59040_FuelGauge_State_t fg_state;
    int fg_calibrate_done;
    unsigned long fg_calibrate_done_time;
    u16 fg_offset;      // absolute value of offset;  in 59040, offset should always be negative
    int fg_cont_mode;   // 1 for continuous mode; 0 for sync mode
    short fg_sample;   // in PMU register, bit 13 is sign bit.  We shift it to bit 15
} FG_Maintain_t;  // in the future, can also add timer (used to dump FG registers periodically, e.g. 2 min)
                  // and timer1 (used to delay for at least 50 msec after FGCAL to get FG offset registers)

static FG_Maintain_t fg_maintain;

/* GPIO init table - change this as necessary */
static BCM_PMU_gpioInit_t gpioInitTable[BCM59040_MAX_GPIO] = {
   {
      gpioDir: GPIO_OUTPUT,
      gpioData: 0,
      gpioMode: BCM59040_GPIOX_MODE_NORMAL,
   },
   {
      gpioDir: GPIO_OUTPUT,
      gpioData: 0,
      gpioMode: BCM59040_GPIOX_MODE_NORMAL,
   },
   {
      gpioDir: GPIO_OUTPUT,
      gpioData: 0,
      gpioMode: BCM59040_GPIOX_MODE_NORMAL,
   },
   {
      gpioDir: GPIO_OUTPUT,
      gpioData: 0,
      gpioMode: BCM59040_GPIOX_MODE_NORMAL,
   },
   {
      gpioDir: GPIO_OUTPUT,
      gpioData: 0,
      gpioMode: BCM59040_GPIOX_MODE_NORMAL,
   },
};

static BCM_PMU_Power_On_State_t gPowerOnState = PMU_Power_On_By_On_Button;


/* ---- Private Function Prototypes -------------------------------------- */
static int bcm59040_isr_thread(void *data);
static int bcm59040_process_interrupt( void );

/* Module functions */
static int bcm59040_module_init( void );
static void bcm59040_module_exit( void );

/* Common initialization routine */
static int bcm59040_init( void );

/* Interrupt handling functions */
static int bcm59040_irq_init(u8 *initial_int_status);

/* Interrupt service routine */
static irqreturn_t bcm59040_isr( void *dev_id );

/* Get power on condition */
static BCM_PMU_Power_On_State_t bcm59040_get_power_on_state( void );

/* IOCTL handler */
static int bcm59040_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
static void bcm59040_poweroff( void );

/* Custom timed run function */
static void bcm59040_run( void );

/* Set log level for pmu debugging */
static void bcm59040_logLevel( int level );

/* Suspend and resume PMU handlers */
static int bcm59040_suspend( int mode );
static int bcm59040_resume( int mode );

/* Power regulator control */
int bcm59040_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state);
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state(int regulatorID);
int bcm59040_regulator_set_state_for_pm(int regulatorID, int pmState, BCM_PMU_Regulator_State_t state);
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state_for_pm(int regulatorID, int pmState);

int bcm59040_regulator_set_voltage(int regulatorID, u32 mV);
u32 bcm59040_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);

static int bcm59040_state_to_opmod(BCM_PMU_Regulator_State_t state, u8 *opmod);
static int bcm59040_opmod_to_state(u8 opmod, BCM_PMU_Regulator_State_t *state);

static int bcm59040_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step);
static int bcm59040_vout_to_mV(int regulatorID, u8 vout, u32 *mV);

/* Charger control */
static int bcm59040_charger_init(void);
static int bcm59040_set_charging_current(int mA);
int bcm59040_get_charging_current( void );
static int bcm59040_charger_wd_control(int enable);
static int bcm59040_charger_wd_clear(void);

static int bcm59040_charger_is_inserted(int *chargerID);

/* otg control */
static int bcm59040_otg_init( void );
/*static void bcm59040_check_otg_stat( void ); */
static void bcm59040_update_otg_stat(OTG_Stat_t OTG_Stat, int enable);
static BCM_PMU_OTG_Role_t bcm59040_get_otg_role(void);
static void bcm59040_check_id_from_env(unsigned char env_byte);
static int bcm59040_id_insert(void);
static int bcm59040_id_change(void);
static int bcm59040_id_remove(void);

/* Event dispatcher */
static void bcm59040_event_notify(BCM59040_InterruptId_t irq_id);

/* DVS APIs for Core switching regulator */
int bcm59040_is_dvs_enabled(void);
static int bcm59040_CSR_PC_DVS_init(void);
#ifndef CONFIG_BCM_PMU_BCM59040_B0
void bcm59040_enable_dvs(void);
void bcm59040_disable_dvs(void);
#else 
static int bcm59040_update_reg_addr_volt(void);
#endif

/* fuel gauge */
static void bcm59040_fuelgauge_init( void );
static void bcm59040_fuelgauge_enable( int enable );
static int bcm59040_fuelgauge_clear_acc_cnt( void );
static void bcm59040_fuelgauge_calibrate( void );
static int bcm59040_get_fuelgauge_sample_reg( short *fgsmpl, int fast );
static int bcm59040_fuelgauge_sample_to_integer(u8 *smpl);

/* I2C client address definitions */
static unsigned short normal_i2c[] = {BCM59040_I2C_BASE_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]     = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]    = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data bcm59040_i2c_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

/* PMU device operations */
static BCM_PMU_Operations_t bcm59040_ops =
{
   init: bcm59040_init,
   isr: bcm59040_isr,
   get_power_on_state: bcm59040_get_power_on_state,
   ioctl: bcm59040_ioctl,
   poweroff: bcm59040_poweroff,
   run: bcm59040_run,
   logLevel: bcm59040_logLevel,
   suspend: bcm59040_suspend,
   resume: bcm59040_resume,
   regulator:
   {
      set_state:        bcm59040_regulator_set_state,
      get_state:        bcm59040_regulator_get_state,
      set_state_for_pm: bcm59040_regulator_set_state_for_pm,
      get_state_for_pm: bcm59040_regulator_get_state_for_pm,
      set_voltage:      bcm59040_regulator_set_voltage,
      get_voltage:      bcm59040_regulator_get_voltage,
   },
   charger:
   {
      start: NULL,
      stop: NULL,
      is_inserted: bcm59040_charger_is_inserted,
      set_current_limit: NULL,
      get_current: bcm59040_get_charging_current,
   },
   fuelgauge:
   {
      get_FG_sample: bcm59040_get_fuelgauge_sample_reg,
      FG_enable: bcm59040_fuelgauge_enable,
   },
   i2c_data: &bcm59040_i2c_data,
};

static int bcm59040_pwmled_init(void);
static int bcm59040_set_pwm_hiper(BCM_PMU_PWM_hi_per_t hiper) ;
static int bcm59040_set_pwm_loper(BCM_PMU_PWM_lo_per_t loper) ;
static int bcm59040_set_pwm_pwr_ctrl(BCM_PMU_PWM_pwr_ctrl_t pwrctrl) ;
static int bcm59040_set_pwm_ctrl(BCM_PMU_PWM_ctrl_t pwmctrl) ;
#ifdef CONFIG_BCM_PMU_BCM59040_B0
static int bcm59040_set_pwm_sw_ctrl(int sw_enable) ;
#endif

/* power on key functions */
static int bcm59040_ponkey_onoff_init(void);
static int bcm59040_UserPonkeyLock(int enable);

/* adc functions */
static int bcm59040_adc_init(void);
static int bcm59040_adc_continuous_mode_select(int normal_mode);
static int bcm59040_adc_reset_count(int reset_count);
static int bcm59040_adc_async_mode_control( void );
static int bcm59040_adc_latch_data( int channel );
static int bcm59040_adc_read_data( void );

/* gpio functions */
static int bcm59040_gpio_init(void);
static int bcm59040_gpio_read_data(int gpioNum, int *ioData);
static int bcm59040_gpio_write_data(int gpioNum, int ioData);
static int bcm59040_gpio_set_direction(int gpioNum, BCM_PMU_gpio_dir_t ioDir);
static int bcm59040_gpio_set_mode(int gpioNum, int ioMode);

/* power mode functions */
static int bcm59040_powermode_init(void);
static int bcm59040_powermode_pc_i2c_control( int pc1, int pc2 );
static int bcm59040_hibernate_mode_enable(int enable);

/* RTC functions*/
static int bcm59040_RTC_GetTime(BCM_PMU_RTC_time_t *pRtc);
static int bcm59040_RTC_SetTime(BCM_PMU_RTC_time_t *pRtc);
static int bcm59040_RTC_GetAlarmTime(BCM_PMU_RTC_time_t *pRtc);
static int bcm59040_RTC_SetAlarmTime(BCM_PMU_RTC_time_t *pRtc);

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

/* ------variable and functions for ADC request--------------------------- */
bcm_adc_request_t* bcm59040_adc_current_req			= NULL;
bcm_adc_handler_t bcm59040_adc_completion_handler	= NULL;
bcm_evt_handler_t bcm476x_rtc_update_handler		= NULL;

// @KP: added for TOMTOM_OTAVALO
bcm_evt_handler_t bcm476x_usb_event_handler		    = NULL;

int bcm59040_adc_request(bcm_adc_request_t* req)
{
	int rc;

	rc = pmu_write(BCM59040_REG_ADCCTRL1, req->channel);

	if (rc < 0)
    {
       PMU_DEBUG(DBG_ERROR, "error writing to ADCCTRL1 register\n");
	   return rc;
    }

	bcm59040_adc_current_req = req;

	bcm59040_adc_async_mode_control();

	return 0;
}

int bcm59040_adc_complete_register(bcm_adc_handler_t adcCompletion)
{
   bcm59040_adc_completion_handler = adcCompletion;

   return 0;
}

int bcm59040_rtc_complete_register(bcm_evt_handler_t rtcCompletion)
{
   bcm476x_rtc_update_handler = rtcCompletion;

   return 0;
}

// @KP: added for TOMTOM_OTAVALO
int bcm59040_usb_event_register(bcm_evt_handler_t usbEventHandler)
{
   bcm476x_usb_event_handler = usbEventHandler;

   return 0;
}

/****************************************************************************
*
*  bcm59040_init
*
***************************************************************************/
static int bcm59040_init( void )
{
   int rc = 0;
   int version = 0xFF;
   u8 int_status[BCM59040_NUM_INT_REG];

#ifdef   CONFIG_RTC_DRV_BCM59040
   bcm476x_rtc_update_handler = &bcm476x_rtc_update;
#endif

   PMU_DEBUG(DBG_TRACE, "\n");

   /* Note: PMUID is on page 1 !  So switch to page 1 first, read it, then switch back to page 0 */
   rc = pmu_write ( BCM59040_REG_PAGESEL, 0x1);
   version = pmu_read(BCM59040_REG_PMUID);
   rc = pmu_write ( BCM59040_REG_PAGESEL, 0);
   if (version == -1)
   {
      PMU_DEBUG(DBG_ERROR, "Error reading Chip Version\n");
   }
#ifdef CONFIG_BCM_PMU_BCM59040_B0
   else if ((version != PMU_59040_B0_REV) &&
            (version != PMU_59040_B1_REV))
   {
       PMU_DEBUG(DBG_ERROR, "PMU Chip Version and software does not match\n");
       return 1;
   }
#endif
   else
   {
      printk("BCM59040: Chip Version [0x%x]\n", version);
   }

   /* initialize power mode functions */
   bcm59040_powermode_init();

   /* Initialize IRQ handler */
   bcm59040_irq_init(int_status);

   /* Register IRQ handler */
   bcm59040_irq_register(BCM59040_IRQID_INT1_PONKEYR,   bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_PONKEYF,   bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_PONKEYH,   bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_PONKEYBHD, bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_RESTARTH,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_HBINT,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT1_PMUTOOWARM,bcm59040_event_notify); // temp related

   bcm59040_irq_register(BCM59040_IRQID_INT2_CHGINS,    bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_CHGRM,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_CHGOVERV,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_EOC,       bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_USBINS,    bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_USBRM,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_USBOVERV,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT2_CHGDET,    bcm59040_event_notify);

   bcm59040_irq_register(BCM59040_IRQID_INT3_VSROVERV,  bcm59040_event_notify); // vsr over voltage
   bcm59040_irq_register(BCM59040_IRQID_INT3_VSROVERI,  bcm59040_event_notify); // vsr over current
   bcm59040_irq_register(BCM59040_IRQID_INT3_VCHGRNOTOK,bcm59040_event_notify); // VCHGR short to ground (not OK)
#ifdef CONFIG_BCM_PMU_BCM59040_B0
   bcm59040_irq_register(BCM59040_IRQID_INT3_CHG_WDT_ALARM, bcm59040_event_notify); // charger watchdog early warning
#else
   bcm59040_irq_register(BCM59040_IRQID_INT3_VCHGRCLSP, bcm59040_event_notify); // VCHGR collapse
#endif
   bcm59040_irq_register(BCM59040_IRQID_INT3_VBUSLOWBND,bcm59040_event_notify); // charging current reset to 100 mA
   bcm59040_irq_register(BCM59040_IRQID_INT3_CHGERRDIS, bcm59040_event_notify); // MBC charging error goes away
   bcm59040_irq_register(BCM59040_IRQID_INT3_CHGWDTEXP, bcm59040_event_notify); // charger watch dog timer expire
   bcm59040_irq_register(BCM59040_IRQID_INT3_IDOVERI,   bcm59040_event_notify); // ideal diode over current

   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO1OVRI,  bcm59040_event_notify); // LDO over current
   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO20VRI,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO3OVRI,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO4OVRI,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO5OVRI,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT4_LDO6OVRI,  bcm59040_event_notify);

   bcm59040_irq_register(BCM59040_IRQID_INT5_IOSROVRI,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_CSROVRI,   bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_IOSROVRV,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_CSROVRV,   bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_RTCADJ,    bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_RTC1S,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_RTC60S,    bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT5_RTCA1,     bcm59040_event_notify);

   bcm59040_irq_register(BCM59040_IRQID_INT6_MBTEMPFAULT,bcm59040_event_notify); // temp related
   bcm59040_irq_register(BCM59040_IRQID_INT6_MBTEMPLOW,  bcm59040_event_notify); // temp related
   bcm59040_irq_register(BCM59040_IRQID_INT6_MBTEMPHIGH, bcm59040_event_notify); // temp related
   bcm59040_irq_register(BCM59040_IRQID_INT6_MBRM,       bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT6_MBOV,       bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT6_BATINS,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT6_LOWBAT,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT6_VERYLOWBAT, bcm59040_event_notify);

   bcm59040_irq_register(BCM59040_IRQID_INT7_ID_INSRT,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_ID_RMV,       bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_VBUS_VALID_F, bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_A_SESSVALID_F,bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_B_SESSEND_F,  bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_VBUS_VALID_R, bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_A_SESSVALID_R,bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT7_B_SESSEND_R,  bcm59040_event_notify);

   bcm59040_irq_register(BCM59040_IRQID_INT9_SARCONVEND,     bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT9_SARCONTCONVFAIL,bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT9_SARASYNCONVOFF, bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT9_SARASYNREQFAIL, bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT9_RESUME_VBUS,    bcm59040_event_notify);
   bcm59040_irq_register(BCM59040_IRQID_INT9_ID_CHNG,        bcm59040_event_notify);

   /* initialize GPIO */
   bcm59040_gpio_init();

   /* initialize charger function */
   bcm59040_charger_init();

   /* initialize ponkey onoff functions */
   bcm59040_ponkey_onoff_init();

   /* initialize otg functions */
   bcm59040_otg_init();

   /* initialize adc functions */
   bcm59040_adc_init();

   /* Temp disable now so that we don't have a conflict between the old rtc and BCM4670 new rtcN driver */
   //rtc59040_init();

   bcm59040_pwmled_init();

   bcm59040_fuelgauge_init();

   /* If Battery manager is not present, initialize charger settings */
#ifndef CONFIG_BCM_BATTERY_MANAGER
   rc = pmu_read(BCM59040_REG_ENV1);
   if ( rc != -1 )
   {
      if( (u8)rc & (1<<1) )
      {
         PMU_DEBUG(DBG_TRACE, "wall charger detected\n");
         isWallChargerPresent = 1;
      }
      if ((u8)rc & (1<<2) )
      {
         PMU_DEBUG(DBG_TRACE, "USB charger detected\n");
         isUSBChargerPresent = 1;
      }
   }
#endif

/* QP: Temp disable otgshutdown */
/*   rc = 0x0;
   rc = pmu_write( BCM59040_REG_OTGCTRL2, (unsigned char)rc ); */

   /* configure gPowerOnState based on initial INT1 and INT2 status */
   if (int_status[1] & (BCM59040_INT2_CHGINS | BCM59040_INT2_USBINS))
   {
       gPowerOnState = PMU_Power_On_By_Charger;
   }
#ifdef CONFIG_BCM_PMU_BCM59040_B0 // enable for B0 chip
   else if (int_status[0] & BCM59040_INT1_PONKEYH)
   {
       gPowerOnState = PMU_Power_On_By_On_Button;
   }
   /* restart does not have PONKEY Hold interrupt */
   else if (int_status[0] & BCM59040_INT1_RESTARTON) 
   {
       gPowerOnState = PMU_Power_On_By_Restart;
   }
#endif

   /* Set the CLK32 output halted bit for PM3. */
   rc = pmu_read(BCM59040_REG_CLKHALTCTRL);
   rc = pmu_write( BCM59040_REG_CLKHALTCTRL, rc | BCM59040_CLKHALCTRL_CLK_HALT_EN_PM3 );
   return 0;
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
   readval = pmu_read(BCM59040_REG_INT2M);
   if (readval == -1)
   {
      PMU_DEBUG(DBG_ERROR, "error reading INT2M\n");
      return;
   }
   writeval = readval;
   if (isWallChargerPresent || isUSBChargerPresent)
   {
      writeval &= (~BCM59040_INT2_EOC);
   }
   else
   {
      writeval |= BCM59040_INT2_EOC;
   }

   if (readval != writeval)
   {
      int rc = pmu_write( BCM59040_REG_INT2M, writeval);
      if (rc != 0)
      {
         PMU_DEBUG(DBG_ERROR, "error writing to INT2M register\n");
      }
   }
}

/****************************************************************************
*
*  bcm59040_irq_init
*
***************************************************************************/
static int bcm59040_irq_init(u8 *initial_int_status)
{
   int rc;
   int i;
   u8 int_status[BCM59040_NUM_INT_REG];
   u8 reg_addr;

   if(gpio_request(bcm59040_module_gpio, "PMU I2C Interrupt (GPIO1)"))
   {
      return -EINVAL;                                 // GPIO already claimed by someone else!
   }

   /* for initializing the mask registers */
   for (reg_addr = BCM59040_REG_INT1M; reg_addr <= BCM59040_REG_INT9M; reg_addr++)
   {
      rc = pmu_write(reg_addr, bcm59040_IntMask[reg_addr - BCM59040_REG_INT1M]);
      if( rc != 0 )
      {
         PMU_DEBUG(DBG_ERROR, "error writing interrupt register [0x%x]\n", reg_addr);
         //return -EINVAL;
      }
   }
   pmu_write(BCM59040_REG_INT5M, 0x60);

   /* get initial interrupt bits */
   for ( i = 0; i < BCM59040_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59040_REG_INT1 + i);
      if (rc == -1)
      {
         PMU_DEBUG(DBG_ERROR, "error reading interrupt registers - Init \n");
         return -EINVAL;
      }
      else
      {
         int_status[i] = (u8)rc;
         PMU_DEBUG(DBG_TRACE2,"Reg 0x%02x = 0x%02x\n", BCM59040_REG_INT1 + i, rc);
      }
   }

   /* Fill IRQ function table with empty functions and
    * build the table that has bit positions for interrupts based on priority
    */
   for ( i = 0; i < BCM59040_NUM_IRQ; i++ )
   {
      bcm59040_IRQVectorTable[i] = NULL;
   }

   /* save initial int status */
   if (initial_int_status)
   {
      for (i = 0; i < BCM59040_NUM_INT_REG; i++)
      {
         initial_int_status[i] = int_status[i];
      }
   }

   /* Create ISR thread */
   sema_init(&gIsrSem, 0);
   init_completion(&gIsrExited);

   gIsrThreadPid = kernel_thread(bcm59040_isr_thread, (void *)PMU_BCM59040, 0);
   PMU_DEBUG(DBG_TRACE, "isr_thread started %lu\n", gIsrThreadPid);

   return 0;
}

/****************************************************************************
*
*  bcm59040_irq_register
*
*  irqId: ID of the IRQ to be registered
*  isrFunction: function to run for the particular IRQ
*
***************************************************************************/
int bcm59040_irq_register(BCM59040_InterruptId_t irqId, bcm59040_isr_t isrFunction)
{
   if (irqId >= BCM59040_NUM_IRQ)
   {
      PMU_DEBUG(DBG_ERROR, "irqId %d out of range\n", irqId);
      return -EINVAL;
   }
   bcm59040_IRQVectorTable[irqId] = isrFunction;

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
*  bcm59040_isr_thread
*
***************************************************************************/
static int bcm59040_isr_thread(void *data)
{
   int rc;

   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   (void)data;
   daemonize("bcm59040_isr");
   PMU_DEBUG(DBG_TRACE, "\n");

   /* Adjust priority to be higher than any user mode threads but
    * lower than any network threads */
   adjustThreadPriority(1);

   while(1)
   {
      if ( down_interruptible (&gIsrSem) == 0 )
      {
         rc = bcm59040_process_interrupt();
         if (rc < 0)
         {
            PMU_DEBUG(DBG_ERROR, "Error %d processing interrupt.\n", rc);
         }
         IF_BCM4760_B0
            enable_irq(gpio_to_irq(bcm59040_module_gpio));
      }
      else
         break; //leave while
   }

   PMU_DEBUG(DBG_ERROR, "Fatal. Thread should never exit.\n");

   complete_and_exit(&gIsrExited, 0);

} /* bcm59040_isr_thread */

/****************************************************************************
*
*  bcm59040_process_interrupt
*
***************************************************************************/
static int bcm59040_process_interrupt( void )
{
   int rc;
   u8 intBits[BCM59040_NUM_INT_REG];
   u8 intStatus;
   u8 intMask[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
   int i, k;
   u32 clk = timer_get_tick_count();

do
{
   /* read the interrupt status registers */
   rc = pmu_i2c_read_bytes(BCM59040_REG_INT1, intBits, BCM59040_NUM_INT_REG);
   if ( rc < 0)
   {
      printk("error reading interrupt registers\n");
      return -EINVAL;
   }

   for( k =0; k < BCM59040_NUM_INT_REG; k++ )
   {
      intStatus = (u8)(intBits[k] & (~bcm59040_IntMask[k]));
      for( i=0; i < 8; i++ )
      {
         if( intStatus & intMask[i] )
         {
            if( bcm59040_IRQVectorTable[ (k<<3)+i ] != NULL )
            {
               (*bcm59040_IRQVectorTable[ (k<<3)+i ]) ((k<<3)+i);
            }
         }
      }
   }
//   IF_BCM4760_B0
//      break;
}
while ( !gpio_get_value (bcm59040_module_gpio) );  // if GPIO line is still low, then there are more pending interurpts

   clk = timer_get_tick_count() - clk;
   if (clk > pmu_max_isr_clk)
   {
      pmu_max_isr_clk = clk;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_isr
*
***************************************************************************/
static irqreturn_t bcm59040_isr( void *dev_id )
{
   (void)dev_id;

   IF_BCM4760_B0
      disable_irq_nosync(gpio_to_irq(bcm59040_module_gpio));
   up( &gIsrSem );
   return IRQ_HANDLED;
} /* bcm59040_isr */

/****************************************************************************
*
*  bcm59040_get_power_on_state
*
***************************************************************************/
static BCM_PMU_Power_On_State_t bcm59040_get_power_on_state( void )
{
   return gPowerOnState;
}


/****************************************************************************
*
*  bcm59040_ioctl
*
***************************************************************************/
static int bcm59040_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   int rc = 0;
   int temp;

   PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

   switch ( cmd )
   {
      case BCM_PMU_IOCTL_ACTIVATESIM:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: ACTIVATE_SIM not supported\n");
         return -EFAULT;
      }
      break;

      case BCM_PMU_IOCTL_DEACTIVATESIM:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: DEACTIVATE_SIM not supported\n");
         return -EFAULT;
      }
      break;

      case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
      {
         BCM_PMU_Regulator_t regulator;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_REGULATOR_STATE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_regulator_set_state(regulator.regulatorID,regulator.state);
      }
      break;

      case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
      {
         BCM_PMU_Regulator_t regulator;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_REGULATOR_STATE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         regulator.state = bcm59040_regulator_get_state(regulator.regulatorID);
         if ( copy_to_user( (BCM_PMU_Regulator_t *)arg, &regulator, sizeof( regulator )) != 0 )
         {
            return -EFAULT;
         }

         rc = 0;
      }
      break;

      case BCM_PMU_IOCTL_SET_CHARGING_CURRENT:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_CHARGING_CURRENT\n");

         if ( copy_from_user( &temp, (u32 *)arg, sizeof( temp )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_set_charging_current(temp);
      }
      break;

      case BCM_PMU_IOCTL_GET_CHARGING_CURRENT:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_CHARGING_CURRENT\n");

         temp = bcm59040_get_charging_current();
         if ( copy_to_user( (int *)arg, &temp, sizeof( temp )) != 0 )
         {
            return -EFAULT;
         }
         rc = 0;
      }
      break;

      case BCM_PMU_IOCTL_SET_CHARGER_WDT_CTRL:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_CHARGER_WDT_CTRL\n");

         if ( copy_from_user( &temp, (int *)arg, sizeof( temp )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_charger_wd_control(temp);
      }
      break;

      case BCM_PMU_IOCTL_SET_CHARGER_WDT_CLEAR:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_CHARGER_WDT_CLEAR\n");

         rc = bcm59040_charger_wd_clear();
      }
      break;

      case BCM_PMU_IOCTL_SET_ADC_CONT_MODE_SEL:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_ADC_CONT_MODE_SEL\n");

         if ( copy_from_user( &temp, (int *)arg, sizeof( temp )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_adc_continuous_mode_select(temp);
      }
      break;

      case BCM_PMU_IOCTL_SET_ADC_RESET_COUNT:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_ADC_RESET_COUNT\n");

         if ( copy_from_user( &temp, (int *)arg, sizeof( temp )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_adc_reset_count(temp);
      }
      break;

      case BCM_PMU_IOCTL_SET_ADC_ASYN_MODE_CTRL:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_ADC_ASYN_MODE_CTRL\n");

         rc = bcm59040_adc_async_mode_control();
      }
      break;

      case BCM_PMU_IOCTL_SET_ADC_LATCH_DATA:
      {
         BCM_PMU_adc_data_t adc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_ADC_LATCH_DATA\n");

         if ( copy_from_user( &adc, (int *)arg, sizeof( adc )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_adc_latch_data(adc.channel);
      }
      break;

      case BCM_PMU_IOCTL_GET_ADC_READ_DATA:
      {
         BCM_PMU_adc_data_t adc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_ADC_READ_DATA\n");

         adc.data = bcm59040_adc_read_data();
         if ( copy_to_user( (BCM_PMU_adc_data_t *)arg, &adc, sizeof( adc )) != 0 )
         {
            return -EFAULT;
         }

         rc = 0;
      }
      break;

      case BCM_PMU_IOCTL_SET_POWERMODE_PC_I2C_CTRL:
      {
         BCM_PMU_pc1_pc2_t pc_control;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_POWERMODE_PC_I2C_CTRL\n");

         if ( copy_from_user( &pc_control, (BCM_PMU_pc1_pc2_t *)arg, sizeof( pc_control )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59040_powermode_pc_i2c_control(pc_control.pc1, pc_control.pc2);
      }
      break;

      case BCM_PMU_IOCTL_GET_VOLTAGE:
      {
         BCM_PMU_Regulator_Volt_t reg;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_VOLTAGE\n");

         if ( copy_from_user( &reg, (BCM_PMU_Regulator_Volt_t *)arg, sizeof( reg )) != 0 )
         {
             return -EFAULT;
         }
         reg.voltage = bcm59040_regulator_get_voltage(reg.regulatorID,&reg.min,&reg.max,&reg.step);
         if ( copy_to_user( (BCM_PMU_Regulator_Volt_t *)arg, &reg, sizeof( reg )) != 0 )
         {
            return -EFAULT;
         }
         rc = 0;
      }
      break;

      case BCM_PMU_IOCTL_SET_VOLTAGE:
      {
         BCM_PMU_Regulator_Volt_t regulator;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_VOLTAGE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_Volt_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_regulator_set_voltage(regulator.regulatorID,regulator.voltage);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_LED_CTRL :
      {
         // how to use from user space :
         // #include <linux/broadcom/pmu_chip.h>
         // BCM_PMU_PWM_ctrl_t reg ;
         // reg.pwmled_ctrl = 2 ;
         // reg.pwmdiv = 0 ;
         //
         // if (ioctl (fd, BCM_PMU_IOCTL_SET_PWM_LED_CTRL, (unsigned long) &reg) != 0)
         // {
         //    perror( "ioctl call failed" );
         //    return(-1);
         // }

         BCM_PMU_PWM_ctrl_t pwmctrl ;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_PWM_LED_CTRL \n");

         if ( copy_from_user( &pwmctrl, (BCM_PMU_PWM_ctrl_t *)arg, sizeof( pwmctrl )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_set_pwm_ctrl(pwmctrl);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_HI_PER :
      {
         BCM_PMU_PWM_hi_per_t hiper ;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_PWM_HI_PER \n");

         if ( copy_from_user( &hiper, (BCM_PMU_PWM_hi_per_t *)arg, sizeof( hiper )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_set_pwm_hiper(hiper);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_LO_PER :
      {
         BCM_PMU_PWM_lo_per_t loper ;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_PWM_LO_PER \n");

         if ( copy_from_user( &loper, (BCM_PMU_PWM_lo_per_t *)arg, sizeof( loper )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_set_pwm_loper(loper);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_PWR_CTRL :
      {
         BCM_PMU_PWM_pwr_ctrl_t pwrctrl ;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_PWM_PWR_CTRL \n");

         if ( copy_from_user( &pwrctrl, (BCM_PMU_PWM_pwr_ctrl_t *)arg, sizeof( pwrctrl )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_set_pwm_pwr_ctrl(pwrctrl);
      }
      break;

      case BCM_PMU_IOCTL_GPIO_WRITE_DATA :
      {
         BCM_PMU_gpioData_t data;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GPIO_WRITE_DATA \n");

         if ( copy_from_user( &data, (BCM_PMU_gpioData_t *)arg, sizeof( data )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_gpio_write_data(data.gpioNum, data.gpioData);
      }
      break;

      case BCM_PMU_IOCTL_GPIO_READ_DATA :
      {
         BCM_PMU_gpioData_t data;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GPIO_READ_DATA \n");

         if ( copy_from_user( &data, (BCM_PMU_gpioData_t *)arg, sizeof( data )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_gpio_read_data(data.gpioNum, &data.gpioData);

         if (rc < 0)
         {
            return -EFAULT;
         }

         if ( copy_to_user( (BCM_PMU_gpioData_t *)arg, &data, sizeof( data )) != 0 )
         {
            return -EFAULT;
         }
      }
      break;

      case BCM_PMU_IOCTL_GPIO_SET_DIRECTION :
      {
         BCM_PMU_gpioDir_t data;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GPIO_SET_DIRECTION \n");

         if ( copy_from_user( &data, (BCM_PMU_gpioDir_t *)arg, sizeof( data )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_gpio_set_direction(data.gpioNum, data.gpioDir);
      }
      break;

      case BCM_PMU_IOCTL_GPIO_SET_MODE :
      {
         BCM_PMU_gpioMode_t data;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GPIO_SET_DIRECTION \n");

         if ( copy_from_user( &data, (BCM_PMU_gpioMode_t *)arg, sizeof( data )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_gpio_set_mode(data.gpioNum, data.gpioMode);
      }
      break;

      case BCM_PMU_IOCTL_GET_OTG_ROLE:
      {
         BCM_PMU_OTG_Role_t otg_role;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_OTG_ROLE\n");

         otg_role = bcm59040_get_otg_role();
         if ( copy_to_user( (BCM_PMU_OTG_Role_t *)arg, &otg_role, sizeof( otg_role )) != 0 )
         {
            return -EFAULT;
         }

         rc = 0;
      }
      break;

      case BCM_PMU_IOCTL_SET_KEYLOCK:
      {
         int enable = 0;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_KEYLOCK\n");

         if ( copy_from_user( &enable, (int *)arg, sizeof( enable )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_UserPonkeyLock(enable);
      }
      break;

      case BCM_PMU_IOCTL_GET_RTC_TIME:
      {

         BCM_PMU_RTC_time_t rtc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_RTC_TIME\n");

         rc = bcm59040_RTC_GetTime(&rtc);
         if ( copy_to_user( (BCM_PMU_RTC_time_t *)arg, &rtc, sizeof( rtc )) != 0 )
         {
            return -EFAULT;
         }

      }
      break;

      case BCM_PMU_IOCTL_SET_RTC_TIME:
      {
         BCM_PMU_RTC_time_t rtc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_RTC_TIME\n");

         if ( copy_from_user( &rtc, (BCM_PMU_RTC_time_t *)arg, sizeof( rtc )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_RTC_SetTime(&rtc);
      }
      break;

      case BCM_PMU_IOCTL_GET_RTC_ALARM_TIME:
      {

         BCM_PMU_RTC_time_t rtc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_RTC_ALARM_TIME\n");

         rc = bcm59040_RTC_GetAlarmTime(&rtc);
         if ( copy_to_user( (BCM_PMU_RTC_time_t *)arg, &rtc, sizeof( rtc )) != 0 )
         {
            return -EFAULT;
         }

      }
      break;

      case BCM_PMU_IOCTL_SET_RTC_ALARM_TIME:
      {
         BCM_PMU_RTC_time_t rtc;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_SET_RTC_ALARM_TIME\n");

         if ( copy_from_user( &rtc, (BCM_PMU_RTC_time_t *)arg, sizeof( rtc )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59040_RTC_SetAlarmTime(&rtc);
      }
      break;

      case BCM_PMU_IOCTL_GET_FG_SAMPLE:
      {
         BCM_PMU_FuelGauge_t fg;
         PMU_DEBUG(DBG_TRACE, "bcm59040_ioctl: BCM_PMU_IOCTL_GET_FG_SAMPLE\n");

         if ( copy_from_user( &fg, (BCM_PMU_FuelGauge_t *)arg, sizeof( fg )) != 0 )
         {
             return -EFAULT;
         }
         /* read FGSMPL,slow mode*/
         if (bcm59040_get_fuelgauge_sample_reg(&(fg.fgsmpl), fg.mode) == 0)
         {
            if ( copy_to_user( (BCM_PMU_FuelGauge_t *)arg, &fg, sizeof( fg )) != 0 )
            {
               return -EFAULT;
            }
         }
         else
             return -EFAULT;

         rc = 0;
      }
      break;


      default:
      {
          PMU_DEBUG(DBG_ERROR, "bcm59040_ioctl: UNSUPPORTED CMD\n");
          rc = -ENOTTY;
      }
      break;
   }

   return rc;

} /* bcm59040_ioctl */

/****************************************************************************
*
*  bcm59040_poweroff
*
***************************************************************************/
static void bcm59040_poweroff( void )
{
   int rc = 0;
   uint8_t hostact;

   rc = pmu_read( BCM59040_REG_HOSTACT );
   if( rc != -1 )
   {
      hostact = (uint8_t)rc;
      hostact |= BCM59040_HOSTACT_HOSTDICOFF;
      rc = pmu_write(BCM59040_REG_HOSTACT, hostact );
   }
}

/****************************************************************************
*
*  bcm59040_run - platform specific run function, called from battmgr.
*  We are using the fuel gauge for demonstration purposes only. This code does
*  NOT account for charge cycle history, battery condition, temperature, learning
*  cycles, aging, voltage measurement for replacement batteries, etc.
*
***************************************************************************/
static void bcm59040_run( void )
{
    return;
}

/****************************************************************************
*
*  bcm59040_logLevel
*
***************************************************************************/
static void bcm59040_logLevel( int level )
{
   logLevel = level;
}

/*
 * needed to store PMU interrupt mask registers during suspend/resume.
 */

static uint8_t pmu_mask_regs[9];

/****************************************************************************
*
*  bcm59040_suspend
*
***************************************************************************/

static int bcm59040_suspend( int mode )
{
    int i;
    uint8_t read_val;

#ifdef CONFIG_BCM4760_PMSTR_BBL59040_LUKEWARMSTART
    pmu_write(BCM59040_REG_FGGNLR1, 0x6A);
#endif

    /*
     * save current interrupt mask registers. Then mask all PMU events
     * that are uninteresting during suspend.
     */
    for( i=0 ; i<9 ; i++)
    {
        pmu_mask_regs[i] = pmu_read(BCM59040_REG_INT1M+i);
        (void)pmu_write(BCM59040_REG_INT1M+i,0xFF);
    }
    (void)pmu_write(BCM59040_REG_INT1M, 0x80 | BCM59040_INT1_PMUTOOWARM);
    (void)pmu_write(BCM59040_REG_INT5M,
                    pmu_read(BCM59040_REG_INT5M) & ~BCM59040_INT5_RTCA1);

    /*
     * Clear any pending PMU irq
     */
	
    read_val = pmu_read(BCM59040_REG_INT1);
    read_val = pmu_read(BCM59040_REG_INT2);
    read_val = pmu_read(BCM59040_REG_INT3);
    read_val = pmu_read(BCM59040_REG_INT4);
    read_val = pmu_read(BCM59040_REG_INT5);
    read_val = pmu_read(BCM59040_REG_INT6);
    read_val = pmu_read(BCM59040_REG_INT7);
    read_val = pmu_read(BCM59040_REG_INT8);
    read_val = pmu_read(BCM59040_REG_INT9);


    pmu_write(BCM59040_REG_OTGCTRL1, (bcm59040_otg_suspend_regs[0] & 0x7F) );
    pmu_write(BCM59040_REG_OTGCTRL2, (bcm59040_otg_suspend_regs[1] & 0x3F) );
    return 0;
}

/****************************************************************************
*
*  bcm59040_resume
*
***************************************************************************/
static int bcm59040_resume( int mode )
{
    int i;

    for( i=0 ; i<9 ; i++)
        pmu_write(BCM59040_REG_INT1M+i, pmu_mask_regs[i]);

	return 0;
}

/****************************************************************************
*
*  bcm59040_pwmled_init(void)
*
***************************************************************************/
static int bcm59040_pwmled_init(void)
{
#ifdef CONFIG_BCM_PMU_BCM59040_B0
   int rc;

   // read current settings
   rc = pmu_read(BCM59040_REG_PWMLEDCTRL1);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading PWMLEDCTRL1 register.\n");
      return rc;
   }

   if ( rc & BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL ) 
   {
      pwmled_sw_ctrl_enable = 1;
   }
   else
   {
      pwmled_sw_ctrl_enable = 0;
   }
#endif

   return 0;
}

/****************************************************************************
*
*  bcm59040_set_pwm_hiper
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59040_set_pwm_hiper(BCM_PMU_PWM_hi_per_t hiper)
{
   int rc;
   u8 regAddr;

   if ((hiper.pwmledNum > 3) || (hiper.pwmledNum < 1))
   {
      printk("bcm59040_set_pwm_hiper: PWMLED %d not available.\n", hiper.pwmledNum);
      return -EINVAL;
   }

   if ( hiper.hi_per > 0x3f )
   {
      printk("bcm59040_set_pwm_hiper: High per out of range %d-0x%x, given val = 0x%x\n", 0, 0x3f, hiper.hi_per);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_PLD1CTRL2 + (3 * (hiper.pwmledNum - 1));
   rc = pmu_read( regAddr );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading PLDxCTRL2 register.\n");
      return rc;
   }

   // set high period value.
   rc = pmu_write(regAddr, (u8)(hiper.hi_per));

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing PLDxCTRL2 register.\n");
      return rc;
   }

#ifdef CONFIG_BCM_PMU_BCM59040_B0
   if (hiper.pwmledNum != 1) 
   {      
       bcm59040_set_pwm_sw_ctrl(1);
   }
#endif

   return 0;
} /* bcm59040_set_pwm_hiper */




/****************************************************************************
*
*  bcm59040_set_pwm_loper
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59040_set_pwm_loper(BCM_PMU_PWM_lo_per_t loper)
{
   int rc;
   u8 regAddr;

   if ((loper.pwmledNum > 3) || (loper.pwmledNum < 1))
   {
      printk("bcm59040_set_pwm_loper: PWMLED %d not available.\n", loper.pwmledNum);
      return -EINVAL;
   }

   if ( loper.lo_per > 0x3f )
   {
      PMU_DEBUG(DBG_ERROR, "Low per out of range %d-0x%x, given val = 0x%x\n", 0, 0x3f, loper.lo_per);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_PLD1CTRL3 + (3 * (loper.pwmledNum - 1));
   rc = pmu_read( regAddr );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading PLDxCTRL3 register.\n");
      return rc;
   }

   // set low period value.
   rc = pmu_write(regAddr, (u8)(loper.lo_per));

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing PLDxCTRL3 register.\n");
      return rc;
   }

#ifdef CONFIG_BCM_PMU_BCM59040_B0
   if (loper.pwmledNum != 1) 
   {      
       bcm59040_set_pwm_sw_ctrl(1);
   }
#endif

   return 0;
} /* bcm59040_set_pwm_loper */


/****************************************************************************
*
*  bcm59040_set_pwm_pwr_ctrl
*
*  Set the pwm/led controls to either power down or power up the pwmled section
*
***************************************************************************/
static int bcm59040_set_pwm_pwr_ctrl(BCM_PMU_PWM_pwr_ctrl_t pwrctrl)
{
   u8 val = 0 ;
   int rc;

   if ( pwrctrl.pwr_ctrl > 1 )
   {
      printk("bcm59040_set_pwm_pwr_ctrl: Power contrl value can be either 0 or 1, given val = 0x%x\n", pwrctrl.pwr_ctrl);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_read(BCM59040_REG_PWMLEDCTRL1);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading PWMLEDCTRL1 register.\n");
      return rc;
   }

   if ( pwrctrl.pwr_ctrl == 0 )
   {
      val = rc & (~(BCM59040_PWMLEDCTRL1_PWMLED_PDN)) ; // Disable bit 7, set to 0.
   }
   else
   {
      val = rc | (BCM59040_PWMLEDCTRL1_PWMLED_PDN) ; // Enable bit 7, set to 1.
   }

   // update register
   rc = pmu_write(BCM59040_REG_PWMLEDCTRL1, val);

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing PWMLEDCTRL1 register.\n");
      return rc;
   }

   return 0;
} /* bcm59040_set_pwm_pwr_ctrl */


#ifdef CONFIG_BCM_PMU_BCM59040_B0
/****************************************************************************
*
*  bcm59040_set_pwm_sw_ctrl
*
*  Set the pwm/led SW controls for pwmled 2 & 3
*
***************************************************************************/
static int bcm59040_set_pwm_sw_ctrl(int sw_enable) 
{
   int rc;
   u8 val = 0;

   if ( (sw_enable && (pwmled_sw_ctrl_enable == 0)) || ((sw_enable == 0) && (pwmled_sw_ctrl_enable))  )
   {   
       // read current settings
       rc = pmu_read(BCM59040_REG_PWMLEDCTRL1);
       if (rc < 0)
       {
           PMU_DEBUG(DBG_ERROR, "error reading PWMLEDCTRL1 register.\n");
           return rc;
       }

       if (sw_enable && (pwmled_sw_ctrl_enable == 0)) 
       {
           val = rc | BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL;
       }
       else
       {
           val = rc & (~BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL) ;
       }

       rc = pmu_write( BCM59040_REG_PWMLEDCTRL1, val );

       if (rc < 0)
       {
           PMU_DEBUG(DBG_ERROR, "error writing to PLDxCTRL1 register.\n");
           return rc;
       }
   }

   return 0;
} /* bcm59040_set_pwm_sw_ctrl */
#endif

/****************************************************************************
*
*  bcm59040_set_pwm_ctrl
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59040_set_pwm_ctrl(BCM_PMU_PWM_ctrl_t pwmctrl)
{
   int rc;
   u8 regAddr;

   if ((pwmctrl.pwmledNum > 3) || (pwmctrl.pwmledNum < 1))
   {
      printk("bcm59040_set_pwm_ctrl: PWMLED %d not available.\n", pwmctrl.pwmledNum);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_PLD1CTRL1 + (3 * (pwmctrl.pwmledNum - 1));
   rc = pmu_read( regAddr );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading PLDxCTRL1 register.\n");
      return rc;
   }

   if(pwmctrl.pwmledNum == 1)
   {
       /* preserve the OTP value of the current field */
       rc = (BCM59040_PLD1CTRL1_LCTRL1_MASK & (unsigned char)(rc));
   }
   else
   {
#ifdef CONFIG_BCM_PMU_BCM59040_B0
       bcm59040_set_pwm_sw_ctrl(1);
#endif
       /* preserve the OTP value of the current field; pwmled2&3 have same format */
       rc = (BCM59040_PLD2CTRL1_LCTRL2_MASK & (unsigned char)(rc));
   }

   rc = ((pwmctrl.pwmdiv << 2) | (pwmctrl.pwmled_ctrl << 1) | (pwmctrl.pwmledOn) | rc);

   rc = pmu_write( regAddr, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing to PLDxCTRL1 register.\n");
      return rc;
   }

   return 0;
} /* bcm59040_set_pwm_ctrl */

/****************************************************************************
*
*  bcm59040_regulator_set_state_for_pm
*
*  Set the state (NM, LPM, OFF) of a regulator
*
***************************************************************************/
int bcm59040_regulator_set_state_for_pm(int regulatorID, int pmState, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;
   u8 mask = 0;

   if (!bcm59040_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = bcm59040_state_to_opmod(state, &opmod);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state %d.\n", state);
      return -EINVAL;
   }

   switch (pmState)
   {
     case 0:
       mask = BCM59040_PMCTRL_PM0_MASK;
       break;
     case 1:
       opmod = opmod << 2;
       mask = BCM59040_PMCTRL_PM1_MASK;
       break;
     case 2:
       opmod = opmod << 4;
       mask = BCM59040_PMCTRL_PM2_MASK;
       break;
     case 3:
       opmod = opmod << 6;
       mask = BCM59040_PMCTRL_PM3_MASK;
       break;
   }

   // read current settings
   rc = pmu_read(bcm59040_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return rc;
   }

   // update register
   val = opmod;

   if ( val != ((u8)rc & mask) )
   {
      // write settings only if a change in value is detected
      rc = pmu_write(bcm59040_regulator_map[regulatorID].reg_addr, ((u8)rc & ~mask) | val);
   }

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* bcm59040_regulator_set_state_for_pm */

/****************************************************************************
*
*  bcm59040_regulator_set_state
*
*  Set the state (NM, LPM, OFF) of a regulator
*
***************************************************************************/
int bcm59040_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;
   u8 mask = 0;

   if (!bcm59040_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = bcm59040_state_to_opmod(state, &opmod);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state %d.\n", state);
      return -EINVAL;
   }

   rc = pmu_read( BCM59040_REG_HBCTRL );
   rc = (rc & BCM59040_HBCTRL_PC_I2C_MASK) >> 6; // rc has the value of PMx

   switch (rc)
   {
     case 0:
       mask = BCM59040_PMCTRL_PM0_MASK;
       break;
     case 1:
       opmod = opmod << 2;
       mask = BCM59040_PMCTRL_PM1_MASK;
       break;
     case 2:
       opmod = opmod << 4;
       mask = BCM59040_PMCTRL_PM2_MASK;
       break;
     case 3:
       opmod = opmod << 6;
       mask = BCM59040_PMCTRL_PM3_MASK;
       break;
   }

   // read current settings
   rc = pmu_read(bcm59040_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return rc;
   }

   // update register
   val = opmod;

   if ( val != ((u8)rc & mask) )
   {
      // write settings only if a change in value is detected
      rc = pmu_write(bcm59040_regulator_map[regulatorID].reg_addr, ((u8)rc & ~mask) | val);
   }

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* bcm59040_regulator_set_state */

/****************************************************************************
*
*  bcm59040_mV_to_vout
*
*  vout is the array index, which is also the binary value written into register
*
***************************************************************************/
int bcm59040_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step)
{
   u32 *vout_to_mV_map;
   int map_size;
   int i;

   if (!vout)
      return -1;

   /* Validate input mV */
   if ((mV < min_mV) || (mV > max_mV))
   {
      printk(KERN_ERR "bcm59040: invalid %d mV setting for regulator %d.\n", mV, regulatorID);
      return -1;
   }
   if (mV_step != -1)
   {
      if ((mV - min_mV) % mV_step)
      {
         printk(KERN_ERR "bcm59040: invalid %d mV setting for regulator %d.\n", mV, regulatorID);
         return -1;
      }
   }

   if (bcm59040_regulator_map[regulatorID].vout_to_mV_map)
   {
      vout_to_mV_map = bcm59040_regulator_map[regulatorID].vout_to_mV_map;
      map_size = bcm59040_regulator_map[regulatorID].map_size;
   }
   else
   {
      printk(KERN_ERR "Not supported\n");
      return -1;
   }

   /* Find matching voltage in table */
   for (i = 0; i < map_size; i++)
   {
      if (vout_to_mV_map[i] == mV)
      {
         *vout = i;
         return 0;
      }
   }

   printk("bcm59040: corrupt mapping table.\n");
   return -1;
}

/****************************************************************************
*
*  bcm59040_vout_to_mV
*
*  vout is the array index
*
***************************************************************************/
int bcm59040_vout_to_mV(int regulatorID, u8 vout, u32 *mV)
{
   u32 *vout_to_mV_map;
   int map_size;

   if (!mV)
      return -1;

   if (bcm59040_regulator_map[regulatorID].vout_to_mV_map)
   {
      vout_to_mV_map = bcm59040_regulator_map[regulatorID].vout_to_mV_map;
      map_size = bcm59040_regulator_map[regulatorID].map_size;
   }
   else
   {
      printk(KERN_ERR "Not supported\n");
      return -1;
   }
   // Mapping register value to voltage
   if (vout >= map_size)
   {
      printk("bcm59040: vout out of range\n");
      *mV = 0;
      return -1;
   }

   *mV = vout_to_mV_map[vout];
   return 0;
}

/****************************************************************************
*
*  bcm59040_regulator_get_state_for_pm
*
*  Retrieve the current state of a regulator
*
*  state is normal, econ, or off.
*
***************************************************************************/
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state_for_pm(int regulatorID, int pmState)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if((regulatorID > 7) || (regulatorID < 0))
   {
      printk("bcm59040_get_state: regulatorID %d is out of boundary.\n", regulatorID);
      return -EINVAL;
   }

   if (!bcm59040_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_read(bcm59040_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = rc >> (pmState *2);
   rc = bcm59040_opmod_to_state((u8)rc, &state) ;
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* bcm59040_regulator_get_state_for_pm */

/****************************************************************************
*
*  bcm59040_regulator_get_state
*
*  Retrieve the current state of a regulator
*
*  state is normal, econ, or off.
*
***************************************************************************/
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state(int regulatorID)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if((regulatorID > 7) || (regulatorID < 0))
   {
      printk("bcm59040_get_state: regulatorID %d is out of boundary.\n", regulatorID);
      return -EINVAL;
   }

   if (!bcm59040_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_read(bcm59040_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = bcm59040_opmod_to_state((u8)rc, &state) ;
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* bcm59040_regulator_get_state */

/****************************************************************************
*
*  bcm59040_set_voltage
*
*  Set the current voltage level
*
****************************************************************************/
int bcm59040_regulator_set_voltage(int regulatorID, u32 mV)
{
   int rc;
   u8 val;
   u8 vout;
#ifndef CONFIG_BCM_PMU_BCM59040_B0 
   int mask,shift;
   BCM_PMU_Regulator_State_t state;
#endif


   if (!bcm59040_regulator_map[regulatorID].available)
   {
      printk("bcm59040_regulator_set_voltage:regulator %d not available\n", regulatorID);
      return -EINVAL;
   }

   /* Convert voltage */
   rc = bcm59040_mV_to_vout(regulatorID, mV, &vout,
                            bcm59040_regulator_map[regulatorID].min_mV,
                            bcm59040_regulator_map[regulatorID].max_mV,
                            bcm59040_regulator_map[regulatorID].mV_step);

   if (rc < 0)
   {
      printk("bcm59040_regulator_set_voltage: error converting %d mV.\n", mV);
      return -EINVAL;
   }
#ifndef CONFIG_BCM_PMU_BCM59040_B0
   if(regulatorID == 6)  //for CSR only, LDOs and IOSR don't need run it
   {
      if( bcm59040_is_dvs_enabled() )
      {
         state = bcm59040_regulator_get_state(6);
         if(state == PMU_Regulator_On)  //NM
            bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRCTRL10;
         else  //LPM
            bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRCTRL1;
      }
      else
         bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRCTRL2;
   }

   // read current settings
   rc = pmu_i2c_read(bcm59040_regulator_map[regulatorID].reg_addr_volt);
   if (rc < 0)
   {
      printk(KERN_ERR "bcm59040_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;
   mask = bcm59040_regulator_map[regulatorID].vout_mask;
   shift = bcm59040_regulator_map[regulatorID].vout_shift;

   // update register
   val &= ~(mask<<shift);  // zero out the mask bits
   val |= (vout<<shift);   // use array index, which is also the binary value of the register
#else
   if((regulatorID == 6) || (regulatorID == 7))
   {
      bcm59040_update_reg_addr_volt();
   }
   val=vout;
#endif
   // write settings
   rc = pmu_i2c_write(bcm59040_regulator_map[regulatorID].reg_addr_volt, val);
   if (rc < 0)
   {
      printk("bcm59040_set_voltage: error writing regulator control register.\n");
      return rc;
   }
   return 0;
}

#ifdef CONFIG_BCM_PMU_BCM59040_B0
/****************************************************************************
*
*  bcm59040_update_reg_addr_volt
*
*  for B0, update the control reg  for SR output voltage according to current PMx 
*
***************************************************************************/
static int bcm59040_update_reg_addr_volt(void)
{
   int rc;
   rc = pmu_read( BCM59040_REG_HBCTRL );
   rc = (rc & BCM59040_HBCTRL_PC_I2C_MASK) >> 6; // rc has the value of PMx

   switch (rc) 
   {
      case 0:
         bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS0;
         bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS0;
         break;
      case 1:
         bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS1;
         bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS1;
         break;
      case 2:
         bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS2;
         bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS2;
         break;
      case 3:
         bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS3;
         bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS3;
         break;
   }

   return 0;
}
#endif


/****************************************************************************
*
*  bcm59040_regulator_get_voltage
*
*  Retrieve the current voltage level and optionally the valid range of settings.
*
***************************************************************************/
u32 bcm59040_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step)
{
   int rc;
   u32 mV;
   BCM_PMU_Regulator_State_t state;
#ifndef CONFIG_BCM_PMU_BCM59040_B0
   int mask,shift;
#endif

   if((regulatorID > 7) || (regulatorID < 0))
   {
      printk("bcm59040_get_voltage: regulatorID %d is out of boundary.\n", regulatorID);
      return -EINVAL;
   }

   if (!bcm59040_regulator_map[regulatorID].available)
   {
      printk("bcm59040_get_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   state = bcm59040_regulator_get_state(regulatorID);
   if(state == PMU_Regulator_Off)
   {
      mV = 0;
   }
   else //NM or LPM
   {
#ifndef CONFIG_BCM_PMU_BCM59040_B0
      if(regulatorID == 6)  //CSR is special
      {
         if( bcm59040_is_dvs_enabled() )
         {
            if(state == PMU_Regulator_On)  //NM
               rc = pmu_read( BCM59040_REG_CSRCTRL10 );
            else  //LPM
               rc = pmu_read( BCM59040_REG_CSRCTRL1 );
         }
         else
            rc = pmu_read( BCM59040_REG_CSRCTRL2 );
      }
      else  //LDO or IOSR
      {
         rc = pmu_i2c_read(bcm59040_regulator_map[regulatorID].reg_addr_volt);
      }

      if (rc < 0)
      {
         printk("bcm59040_get_voltage: error reading regulator control register.\n");
         return rc;
      }
      mask = bcm59040_regulator_map[regulatorID].vout_mask;
      shift = bcm59040_regulator_map[regulatorID].vout_shift;
      rc = (rc>>shift) & mask;
#else
      if((regulatorID == 6) || (regulatorID == 7))
      {
         bcm59040_update_reg_addr_volt();
      }
      rc = pmu_read(bcm59040_regulator_map[regulatorID].reg_addr_volt);
#endif
      rc = bcm59040_vout_to_mV(regulatorID, (u8)rc, &mV) ;
      if (rc < 0)
      {
         printk("bcm59040_get_voltage: error converting voltage.\n");
         return rc;
      }
   }

   if (min_mV)
      *min_mV = bcm59040_regulator_map[regulatorID].min_mV;
   if (max_mV)
      *max_mV = bcm59040_regulator_map[regulatorID].max_mV;
   if (mV_step)
      *mV_step = bcm59040_regulator_map[regulatorID].mV_step;

   return mV;
} /* bcm59040_regulator_get_voltage */

/****************************************************************************
*
*  bcm59040_charger_init(void)
*
***************************************************************************/
static int bcm59040_charger_init(void)
{
   int rc;

   mbc_maintain.in_maintchrg_mode = 0;
   mbc_maintain.chargerEOC = 0;

	rc = bcm59040_init_charging_mode(); 
	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, "charging mode initialization failure\n");
		return rc;
	}

#ifndef CONFIG_BCM_PMU_BCM59040_B0
   /* disable charger watchdog for 59040 A0.  TODO: re-enable in 59040 B0 */
   bcm59040_charger_wd_control(0);
#endif

   /* configure NTCCTRLx registers for initializing some non-OTPable fields */
   rc = BCM59040_PMU_REG_NTCCTRL1_VAL;
   rc = pmu_write( BCM59040_REG_NTCCTRL1, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_NTCCTRL1 register.\n");
      return rc;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_ponkey_onoff_init(void)
*
***************************************************************************/
static int bcm59040_ponkey_onoff_init(void)
{
   int rc;

   /* configure PONKEYBCNTRL1 */
   rc = pmu_read( BCM59040_REG_PONKEYBCNTRL1 );
   if((unsigned char)rc & BCM59040_PONKEYBCNTRL1_ONOFF)
   {
      PonkeyONOFF=1;
   }
   if((unsigned char)rc & BCM59040_PONKEYBCNTRL1_KEYLOCK)
   {
      PonkeyKeylockOTP = 1;
   }

   rc = ((BCM59040_PMU_REG_PONKEYBCNTRL1_VAL & (~BCM59040_PONKEYBCNTRL1_OTP_MASK))
           | ((unsigned char)(rc) & BCM59040_PONKEYBCNTRL1_OTP_MASK));  // preserve OTP fields
   rc = pmu_write( BCM59040_REG_PONKEYBCNTRL1, (uint8_t)rc );

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_PONKEYBCNTRL1 register.\n");
      return rc;
   }

   /* configure PONKEYBCNTRL2: there's no OTPable fields in this register */
   rc = BCM59040_PMU_REG_PONKEYBCNTRL2_VAL & (BCM59040_PONKEYBCNTRL2_OFFHOLD_MASK | BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK);
   rc = pmu_write( BCM59040_REG_PONKEYBCNTRL2, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n");
      return rc;
   }

   /* configure RSTRTCNTRL, preserve OTP fields */
   rc = pmu_read( BCM59040_REG_RSTRTCNTRL );
   rc = ((BCM59040_PMU_REG_RSTRTCNTRL_VAL & (~BCM59040_RSTRTCNTRL_OTP_MASK)) | ((unsigned char)(rc) & BCM59040_RSTRTCNTRL_OTP_MASK) );
   rc = pmu_write( BCM59040_REG_RSTRTCNTRL, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_RSTRTCNTRL register.\n");
      return rc;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_UserPonkeyLock(int enable)
*
***************************************************************************/
static int bcm59040_UserPonkeyLock(int enable)
{
   int rc;

   if (PonkeyKeylockOTP == 1) // OTP value is 1 for B0 and TomTom case, otherwise is 0 (A0 or B0)
   {
       TempKeyLockBitSet = 0;
   }
   else
   {
   if(enable) //make sure the shutdown delay is not 0
   {
      rc = pmu_read( BCM59040_REG_PONKEYBCNTRL2 );
      if((rc & BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK) == 0)
      {
         rc = rc | 0x08;  // set to 500 msec
         rc = pmu_write( BCM59040_REG_PONKEYBCNTRL2, (uint8_t)rc );
         if (rc < 0)
         {
            PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n");
            return rc;
         }
      }
   }
   else
// restore default values
   {

       rc = BCM59040_PMU_REG_PONKEYBCNTRL2_VAL & (BCM59040_PONKEYBCNTRL2_OFFHOLD_MASK | BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK);
       rc = pmu_write( BCM59040_REG_PONKEYBCNTRL2, (uint8_t)rc );
       if (rc < 0)
       {
           PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n");
           return rc;
       }
   }

      TempKeyLockBitSet = (enable > 0) ? 1 : 0;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_adc_init(void)
*
***************************************************************************/
static int bcm59040_adc_init(void)
{
   int rc;

   /* configure ADCCTRL1~5 for any priority assignment for async conversion */
   /* Default value has been changed to 0xFF */
   /* Do we have a burst write? */
   rc = BCM59040_PMU_REG_ADCCTRL1_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL1, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL1 register.\n");
      return rc;
   }

   rc = BCM59040_PMU_REG_ADCCTRL2_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL2, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL2 register.\n");
      return rc;
   }

   rc = BCM59040_PMU_REG_ADCCTRL3_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL3, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL3 register.\n");
      return rc;
   }

   rc = BCM59040_PMU_REG_ADCCTRL4_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL4, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL4 register.\n");
      return rc;
   }

   rc = BCM59040_PMU_REG_ADCCTRL5_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL5, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL5 register.\n");
      return rc;
   }

   /* configure other ADC registers */
   /* Be careful,SS_MASK bit enabled means PMU will IGNORE all SS measurement requests*/
   rc = BCM59040_PMU_REG_ADCCTRL6_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL6, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL6 register.\n");
      return rc;
   }

   /* Default value has been changed to 0x0F: no channel selection */
   rc = BCM59040_PMU_REG_ADCCTRL7_VAL;
   rc = pmu_write( BCM59040_REG_ADCCTRL7, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL7 register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_adc_continuous_mode_select(int normal_mode )
*
*  Select either normal mode or GSM-burst mode for continuous mode operation
*
***************************************************************************/
static int bcm59040_adc_continuous_mode_select(int normal_mode)
{
   int rc;

   /* configure ADCCTRL6 */
   rc = pmu_read( BCM59040_REG_ADCCTRL6 );

   if (normal_mode)
   {
       rc = (~BCM59040_ADCCTRL6_GSM_DEBOUNCE_EN & (unsigned char)(rc) );
   }
   else  // GSM burst mode
   {
       rc = (BCM59040_ADCCTRL6_GSM_DEBOUNCE_EN | (unsigned char)(rc) );
   }

   rc = pmu_write( BCM59040_REG_ADCCTRL6, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL6 register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_adc_reset_count(int reset_couunt )
*
*  Configure the reset count of 20ms timer
*
***************************************************************************/
static int bcm59040_adc_reset_count(int reset_count)
{
   int rc;
   unsigned char val;

   switch(reset_count)
   {
       case 1:
           val = 0x00;
           break;
       case 3:
           val = 0x10;
           break;
       case 5:
           val = 0x20;
           break;
       case 7:
           val = 0x30;
           break;
       default:
           val = 0x10; /* By default*/
           PMU_DEBUG(DBG_ERROR, "bcm59040_adc_reset_count: UNSUPPORTED VALUE\n");
           break;
   }

   /* configure ADCCTRL6 */
   rc = pmu_read( BCM59040_REG_ADCCTRL6 );
   rc = (~BCM59040_ADCCTRL6_RESET_COUNT_MASK & (unsigned char)(rc) & val);

   rc = pmu_write( BCM59040_REG_ADCCTRL6, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL6 register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_adc_async_mode_control( void )
*
*  start async conversion via i2c register
*
***************************************************************************/
static int bcm59040_adc_async_mode_control( void )
{
   int rc;

   /* configure ADCCTRL6 to make sure SS_MASK is disabled so SS requests can be responded */
   rc = pmu_read( BCM59040_REG_ADCCTRL6 );
   rc = (~BCM59040_ADCCTRL6_SS_MASK_EN & (unsigned char)(rc) );
   rc = pmu_write( BCM59040_REG_ADCCTRL6, (uint8_t)rc );

   /* configure ADCCTRL7 */
   rc = pmu_read( BCM59040_REG_ADCCTRL7 );
   rc = (BCM59040_ADCCTRL7_I2CSS_EN | (unsigned char)(rc) );
   rc = pmu_write( BCM59040_REG_ADCCTRL7, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL7 register.\n");
      return rc;
   }

   return 0;
}


/****************************************************************************
*
*  bcm59040_adc_latch_data( int channel )
*
*  latch raw data to ADC_DATA register
*
***************************************************************************/
static int bcm59040_adc_latch_data( int channel )
{
   int rc;

   rc = (BCM59040_ADCCTRL7_CHAN_SNAPSHOT_MASK & channel);
   rc = pmu_write( BCM59040_REG_ADCCTRL7, (uint8_t)rc );

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_ADCCTRL7 register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_adc_read_data( void )
*
*  read latched raw data from ADC_DATA registers, which was latched by a prior
*  i2c command
*
***************************************************************************/
static int bcm59040_adc_read_data( void )
{
   int combined_data;
   u8 adc_data[2];

   pmu_i2c_read_bytes(BCM59040_REG_ADCCTRL8, adc_data, 2);
   combined_data = (u16) (((adc_data[0] & 0x03) << 8) | adc_data[1]);

   return combined_data;
}



/****************************************************************************
*
*  bcm59040_gpio_init(void)
*
***************************************************************************/
static int bcm59040_gpio_init(void)
{
   int i = 0;
   /* For now, set all the GPIOs to normal mode and as input   *
    * This is to be decided by the application. Change the gpioInitTable *
    * accordingly  *
    */
   for(i=1; i <= BCM59040_MAX_GPIO; i++)
   {
      bcm59040_gpio_set_mode(i, gpioInitTable[i].gpioMode);
      if(gpioInitTable[i].gpioMode == BCM59040_GPIOX_MODE_NORMAL)
      {
         bcm59040_gpio_set_direction(i, gpioInitTable[i].gpioDir);
         if(gpioInitTable[i].gpioDir == GPIO_INPUT)
            bcm59040_gpio_read_data(i, &gpioInitTable[i].gpioData);
         else if(gpioInitTable[i].gpioDir == GPIO_OUTPUT)
            bcm59040_gpio_write_data(i, gpioInitTable[i].gpioData);
      }
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_gpio_read_data(int gpioNum, int *ioData)
*
***************************************************************************/
static int bcm59040_gpio_read_data(int gpioNum, int *ioData)
{
   int rc;
   u8 regAddr;

   if ((gpioNum > 5) || (gpioNum < 1))
   {
      printk("bcm59040_gpio_read_data: gpio %d not available.\n", gpioNum);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum - 1;
   rc = pmu_read( regAddr );

   *ioData = (BCM59040_GPIOCTRLX_GPIO_DATA & (unsigned char)(rc)) >> 2;

   return 0;
}

/****************************************************************************
*
*  bcm59040_gpio_write_data(int gpioNum, int ioData)
*
*  Note: gpioNum = 1~5
***************************************************************************/
static int bcm59040_gpio_write_data(int gpioNum, int ioData)
{
   int rc;
   u8 regAddr;

   if ((gpioNum > 5) || (gpioNum < 1))
   {
      printk("bcm59040_gpio_write_data: gpio %d not available.\n", gpioNum);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum - 1;
   rc = pmu_read( regAddr );

   /* clear it */
   rc = (~BCM59040_GPIOCTRLX_GPIO_DATA & (unsigned char)(rc));

   rc = ((ioData << 2) | rc);

   rc = pmu_write( regAddr, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing to gpio register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_gpio_set_direction(int gpioNum, BCM_PMU_gpio_dir_t ioDir)
*
***************************************************************************/
static int bcm59040_gpio_set_direction(int gpioNum, BCM_PMU_gpio_dir_t ioDir)
{
   int rc;
   u8 regAddr;

   if ((gpioNum > 5) || (gpioNum < 1))
   {
      printk("bcm59040_gpio_set_direction: gpio %d not available.\n", gpioNum);
      return -EINVAL;
   }

   /* If HW based OTG is used, GPIO1 and GPIO2 cannot be configured as output */
   rc = pmu_read(BCM59040_REG_OTGCTRL2);
   if ((BCM59040_OTGCTRL2_HWVSSW_MASK & (unsigned char) rc) >> 4)
   {
      if(gpioNum < 3)
      {
         printk("bcm59040_gpio_set_direction: gpio %d cannot be configured in HW OTG CTRL mode\n", gpioNum);
         return -EINVAL;
      }
   }

   regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum - 1;
   rc = pmu_read( regAddr );

   rc = (~BCM59040_GPIOCTRLX_GPIODIR_MASK & (unsigned char)rc) | ioDir;

   rc = pmu_write( regAddr, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing to gpio register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_gpio_set_mode(int gpioNum, int ioMode)
*
***************************************************************************/
static int bcm59040_gpio_set_mode(int gpioNum, int ioMode)
{
   int rc;
   u8 regAddr;
#ifdef CONFIG_BCM_PMU_BCM59040_B0
   if (gpioNum < 2)
#else
   if (gpioNum < 3)
#endif
   {
      printk("bcm59040_gpio_set_mode: gpio %d is not available for special mode.\n", gpioNum);
      return -EINVAL;
   }

   regAddr = BCM59040_REG_GPIOCTRL1 + gpioNum - 1;
   rc = pmu_read( regAddr );

   rc = (~BCM59040_GPIOCTRLX_GPIO_MODE_MASK & (unsigned char)rc) | (ioMode << 3);
   rc = pmu_write( regAddr, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing to gpio register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_powermode_init(void)
*
***************************************************************************/
static int bcm59040_powermode_init(void)
{
   int rc;
   int i,j;
   u8 opmod[32];


   j=0;
   for(i=0;i<=3;i++)
   {
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo1,&(opmod[j]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo2,&(opmod[j+1]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo3,&(opmod[j+2]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo4,&(opmod[j+3]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo5,&(opmod[j+4]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].ldo6,&(opmod[j+5]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].csr.state,&(opmod[j+6]));
      bcm59040_state_to_opmod(pmu_platform_map->PMx_map[i].iosr,&(opmod[j+7]));
      j+=8;
   }
   for(j=0;j<=7;j++)
   {
      opmod[j] = opmod[j] | (opmod[j+8]<<2) | (opmod[j+16]<<4) | (opmod[j+24]<<6);
      pmu_write( BCM59040_REG_L1PMCTRL+j, opmod[j] );
   }

   rc = pmu_read( BCM59040_REG_HBCTRL );
   rc = pmu_write( BCM59040_REG_HBCTRL, (rc & 0xC0) | (BCM59040_PMU_REG_HBCTRL_VAL & 0x3F) );

   bcm59040_hibernate_mode_enable(1);

#ifndef CONFIG_BCM_PMU_BCM59040_B0
   if(pmu_platform_map->PMx_map[0].csr.DVSenabled)
      bcm59040_enable_dvs(); //for TomTom, DVS always enabled
   else
      bcm59040_disable_dvs();
#else
   rc = pmu_read(BCM59040_REG_CSRCTRL1);    //check DVS_EN bit which is read only and OTPed
   if( (rc & BCM59040_CSRCTRL1_DVS_ENABLE) == 0 )  //if DVS = 0, CSR output voltage can up to 2.5V
   {
      bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_to_mV_map	= pmu_platform_map->map_nodvs_config.vout_to_mV_map;
      bcm59040_regulator_map[BCM59040_REGULATOR_CSR].map_size		= pmu_platform_map->map_nodvs_config.map_size;
      bcm59040_regulator_map[BCM59040_REGULATOR_CSR].min_mV		= pmu_platform_map->map_nodvs_config.min_mV;
      bcm59040_regulator_map[BCM59040_REGULATOR_CSR].max_mV		= pmu_platform_map->map_nodvs_config.max_mV;
      bcm59040_regulator_map[BCM59040_REGULATOR_CSR].mV_step		= pmu_platform_map->map_nodvs_config.mV_step;
   }
#endif

   rc = bcm59040_CSR_PC_DVS_init();
   return 0;
}

/****************************************************************************
*
*  bcm59040_powermode_pc_i2c_control( int pc1, int pc2 )
*
*  use i2c to program pc1/pc2 values
*
***************************************************************************/
static int bcm59040_powermode_pc_i2c_control( int pc1, int pc2 )
{
   int rc;

   /* configure HBCTRL p2_i2c bits */
   rc = pmu_read( BCM59040_REG_HBCTRL );
   rc = (~BCM59040_HBCTRL_PC_I2C_MASK & (unsigned char)(rc) );

   if ( pc1 && pc2 )
   {
      rc = (BCM59040_HBCTRL_PC2_PC1_11 | (unsigned char)(rc) );
#ifndef CONFIG_BCM_PMU_BCM59040_B0
      if(pmu_platform_map->PMx_map[3].csr.DVSenabled)
         bcm59040_enable_dvs();
      else
         bcm59040_disable_dvs();
#endif
   }
   else if ( !pc1 && !pc2 )
   {
      rc = (BCM59040_HBCTRL_PC2_PC1_00 | (unsigned char)(rc) );
#ifndef CONFIG_BCM_PMU_BCM59040_B0
      if(pmu_platform_map->PMx_map[0].csr.DVSenabled)
         bcm59040_enable_dvs();
      else
         bcm59040_disable_dvs();
#endif
   }
   else if ( !pc1 && pc2 )
   {
      rc = (BCM59040_HBCTRL_PC2_PC1_10 | (unsigned char)(rc) );
#ifndef CONFIG_BCM_PMU_BCM59040_B0
      if(pmu_platform_map->PMx_map[2].csr.DVSenabled)
         bcm59040_enable_dvs();
      else
         bcm59040_disable_dvs();
#endif
   }
   else
   {
      rc = (BCM59040_HBCTRL_PC2_PC1_01 | (unsigned char)(rc) );
#ifndef CONFIG_BCM_PMU_BCM59040_B0 
      if(pmu_platform_map->PMx_map[1].csr.DVSenabled)
         bcm59040_enable_dvs();
      else
         bcm59040_disable_dvs();
#endif
   }

   rc = pmu_write( BCM59040_REG_HBCTRL, (uint8_t)rc );
   return 0;
}


/****************************************************************************
*
*  bcm59040_hibernate_mode_enable(int enable)
*
*  enabel or disable hibernate mode
*
***************************************************************************/
static int bcm59040_hibernate_mode_enable(int enable)
{
   int rc;

   rc = pmu_read( BCM59040_REG_HBCTRL );

   if(enable)
   {
      rc = rc | BCM59040_HBCTRL_HB_EN;
   }
   else
   {
      rc = rc & (~BCM59040_HBCTRL_PC_I2C_MASK) ;
   }

   rc = pmu_write( BCM59040_REG_HBCTRL, (uint8_t)rc );

   return 0;
}


/****************************************************************************
*
*  bcm59040_RTC_GetTime(RTCTime_t *pRtc)
*
*  get current date and time from RTC registers
*
***************************************************************************/
static int bcm59040_RTC_GetTime(BCM_PMU_RTC_time_t *pRtc)
{
   int rc;
   u8 regBits[BCM59040_NUM_RTC_REG];

   /* read the RTC registers */
   rc = pmu_i2c_read_bytes(BCM59040_REG_RTCSC, regBits, BCM59040_NUM_RTC_REG);
   if ( rc < 0)
   {
      printk("error reading RTC registers\n");
      return -EINVAL;
   }

   pRtc->Sec = regBits[0];
   pRtc->Min = regBits[1];
   pRtc->Hour = regBits[2];
   pRtc->Week = regBits[3];
   pRtc->Day = regBits[4];
   pRtc->Month = regBits[5];
   pRtc->Year = regBits[6];

   return 0;
}




/****************************************************************************
*
*  bcm59040_RTC_SetTime(RTCTime_t *pRtc)
*
*  get current date and time from RTC registers
*
***************************************************************************/
static int bcm59040_RTC_SetTime(BCM_PMU_RTC_time_t *pRtc)
{
   int rc;
   u8 i,regBits[BCM59040_NUM_RTC_REG];

   regBits[0] = pRtc->Sec;
   regBits[1] = pRtc->Min;
   regBits[2] = pRtc->Hour;
   regBits[3] = pRtc->Week;
   regBits[4] = pRtc->Day;
   regBits[5] = pRtc->Month;
   regBits[6] = pRtc->Year;

   /* wite the RTC registers */
   for(i=0;i<BCM59040_NUM_RTC_REG;i++)
   {
      rc = pmu_write(BCM59040_REG_RTCSC+i, regBits[i]);
      if ( rc < 0)
      {
         printk("error reading RTC registers\n");
         return -EINVAL;
      }
   }

   return 0;
}


/****************************************************************************
*
*  bcm59040_RTC_GetAlarmTime(RTCTime_t *pRtc)
*
*  get current date and time from RTC registers
*
***************************************************************************/
static int bcm59040_RTC_GetAlarmTime(BCM_PMU_RTC_time_t *pRtc)
{
   int rc;
   u8 regBits[BCM59040_NUM_RTC_REG];

   /* read the RTC registers */
   rc = pmu_i2c_read_bytes(BCM59040_REG_RTCSC_A1, regBits, BCM59040_NUM_RTC_REG);
   if ( rc < 0)
   {
      printk("error reading RTC_A1 registers\n");
      return -EINVAL;
   }

   pRtc->Sec = regBits[0];
   pRtc->Min = regBits[1];
   pRtc->Hour = regBits[2];
   pRtc->Week = regBits[3];
   pRtc->Day = regBits[4];
   pRtc->Month = regBits[5];
   pRtc->Year = regBits[6];

   return 0;
}




/****************************************************************************
*
*  bcm59040_RTC_SetAlarmTime(RTCTime_t *pRtc)
*
*  get current date and time from RTC registers
*
***************************************************************************/
static int bcm59040_RTC_SetAlarmTime(BCM_PMU_RTC_time_t *pRtc)
{
   int rc;
   u8 i,regBits[BCM59040_NUM_RTC_REG];

   regBits[0] = pRtc->Sec;
   regBits[1] = pRtc->Min;
   regBits[2] = pRtc->Hour;
   regBits[3] = pRtc->Week;
   regBits[4] = pRtc->Day;
   regBits[5] = pRtc->Month;
   regBits[6] = pRtc->Year;

   /* wite the RTC registers */
   for(i=0;i<BCM59040_NUM_RTC_REG;i++)
   {
      rc = pmu_write(BCM59040_REG_RTCSC_A1+i, regBits[i]);
      if ( rc < 0)
      {
         printk("error reading RTC_A1 registers\n");
         return -EINVAL;
      }
   }

   return 0;
}


/****************************************************************************
*
*  bcm59040_set_charging_current
*
*  Set the charging current (current into the battery) for charger
*
****************************************************************************/
static int bcm59040_set_charging_current(int mA)
{
   int rc, i;
   u32 curr_index = 0xff;

   /* Find matching current in table. curr_index is the array index,
     which is also the binary value written into register */
   for (i = 0; i <= BCM59040_MBCCTRL6_FC_CC_1500MA; i++)
   {
      if (pmu_platform_map->charging_curr_to_mA_map[i] == mA)
      {
         curr_index = i;
         break;
      }
   }

   if (curr_index == 0xff)
   {
       PMU_DEBUG(DBG_ERROR, "bcm59040_set_charging_current: error converting %d mA.\n", mA);
       return -EINVAL;
   }

   /* configure MBCCTRL6 */
   rc = pmu_write( BCM59040_REG_MBCCTRL6, BCM59040_MBCCTRL6_FC_CC_MASK & (unsigned char)curr_index );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_set_charging_current: error writing BCM59040_REG_MBCCTRL6 register.\n");
      return rc;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_get_charging_current
*
*  Get the charging current (current into the battery) for charger
*
***************************************************************************/
int bcm59040_get_charging_current( void )
{
   int rc;
   u32 curr_index;

   rc = pmu_read(BCM59040_REG_MBCCTRL6);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_get_charging_current: error reading MBCCTRL6 register.\n");
      return rc;
   }

   curr_index = BCM59040_MBCCTRL6_FC_CC_MASK & (unsigned char)rc;

   if (curr_index <= BCM59040_MBCCTRL6_FC_CC_1500MA)
   {
       return pmu_platform_map->charging_curr_to_mA_map[curr_index];
   }
   else
   {
       PMU_DEBUG(DBG_ERROR, "bcm59040_get_charging_current: error: current exceed legal range\n");
       return -EINVAL;
   }
} /* bcm59040_get_charging_current */

static int bcm59040_init_charging_mode(void) 
{
	int ret = 0;

	ret = pmu_write(BCM59040_REG_MBCCTRL7, BCM59040_MBCCTRL7_50MA);
	if (ret < 0) {
		pr_err("bcm59040_init_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL7 register.\n");
		return ret;
	}

	ret = pmu_write(BCM59040_REG_MBCCTRL8,  pmu_read(BCM59040_REG_MBCCTRL8)
					|	BCM59040_MBCCTRL8_SYS_TYP
					|	BCM59040_MBCCTRL8_IDDIODE_NORMAL_MODE );
	if (ret < 0) {
		pr_err("bcm59040_init_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL8 register.\n");
		return ret;
	}

	ret = pmu_write(BCM59040_REG_MBCCTRL10,	
						BCM59040_MBCCTRL10_DIS_AUTO_RESUME	
					|	BCM59040_MBCCTRL10_DIS_PWRUP_FROM_IDCTRL
					|	BCM59040_MBCCTRL10_BTEMP_FAULT_CNT);
	if (ret < 0) {
		pr_err("bcm59040_init_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL8 register.\n");
		return ret;
	}

	return ret;
}

static const uint16_t	charge_current[]=
{
	0,	10,	20,	30,	40,	50,	60,	70,
	80,	90,	100,	110,	120,	130,	140,	150,
	0,	100,	200,	300,	400,	500,	600,	700,
	800,	900,	1000,	1100,	1200,	1300,	1400,	1500
};
 
static u8 bcm59040_get_max_charging_current( u32 max_current )
{
	u8	index;
	u8	retval=0;

	for( index=0; index < (sizeof( charge_current )/sizeof( charge_current[0] )); index++ )
	{
		if( (charge_current[index] < max_current) && (charge_current[index] > charge_current[retval]) )
			retval=index;
	}
	return retval;
}

int bcm59040_set_charging_mode(bcm59040_usb_charger_t charger)
{
	int ret = 0;

	unsigned char	charger_mode		= 0;
	unsigned char	charging_current	= 0;
	unsigned char	usb_i_limits		= 0;
	u32		max_charge_current	= 0;

	/* Determine what the maximum is we can charge at. */
	max_charge_current=bcm59040_get_max_charging_current( pmu_platform_map->battery_capacity/2 );

	switch(charger)
	{
		case BCM59040_USB_CLA:

		  charger_mode	=		BCM59040_MBCCTRL3_MBCHOSTEN
							|	BCM59040_MBCCTRL3_VUBGR_FC2_EN
							|	BCM59040_MBCCTRL3_MAINGCHRG
							|	BCM59040_MBCCTRL3_USB0_FC_OPTION;

		  charging_current	=	(BCM59040_MBCCTRL6_FC_CC_1000MA < max_charge_current ? BCM59040_MBCCTRL6_FC_CC_1000MA : max_charge_current);

		  usb_i_limits	=		BCM59040_MBCCTRL9_USB_ILIMIT_1200MA
							|	BCM59040_MBCCTRL9_WALL_ILIMIT_1200MA
							|	BCM59040_MBCCTRL9_USB_VSR_EN
							|	BCM59040_MBCCTRL9_CHGTYP_SHP;

		  break;
		case BCM59040_USB_DEV:

		  charger_mode	=		BCM59040_MBCCTRL3_MBCHOSTEN
							|	BCM59040_MBCCTRL3_VUBGR_FC2_EN
							|	BCM59040_MBCCTRL3_MAINGCHRG
							|	BCM59040_MBCCTRL3_USB0_FC_OPTION;

		  charging_current	=	(BCM59040_MBCCTRL6_FC_CC_500MA < max_charge_current ? BCM59040_MBCCTRL6_FC_CC_500MA : max_charge_current);

		  usb_i_limits =		BCM59040_MBCCTRL9_USB_ILIMIT_500MA
							|	BCM59040_MBCCTRL9_WALL_ILIMIT_1200MA
							|	BCM59040_MBCCTRL9_USB_VSR_EN
							|	BCM59040_MBCCTRL9_CHGTYP_SHP;

		  break;

		case BCM59040_USB_NONE:

		  charging_current	=	(BCM59040_MBCCTRL6_FC_CC_500MA < max_charge_current ? BCM59040_MBCCTRL6_FC_CC_500MA : max_charge_current);

		  usb_i_limits =		BCM59040_MBCCTRL9_USB_ILIMIT_500MA
							|	BCM59040_MBCCTRL9_WALL_ILIMIT_1200MA; 

		  break;

		default:
		  pr_err("bcm59040_set_charging_mode "
					"[%d: UNKNOWN CHARGER_TYPE]", charger);
		  break;
	}

	ret = pmu_write(BCM59040_REG_MBCCTRL3, charger_mode);
	if (ret < 0) {
		pr_err("bcm59040_set_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL3 register.\n");
		return ret;
	}


	ret = pmu_write(BCM59040_REG_MBCCTRL6, charging_current);
	if (ret < 0) {
		pr_err("bcm59040_set_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL6 register.\n");
		return ret;
	}

	ret = pmu_write(BCM59040_REG_MBCCTRL9, usb_i_limits);
	if (ret < 0) {
		pr_err("bcm59040_set_charging_mode: "
				"error writing BCM59040_REG_MBCCTRL9 register.\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(bcm59040_set_charging_mode);

/****************************************************************************
*
*  bcm59040_get_vsr_current_limit
*
*  Get the VSR current limit based for given type of charger.  Allowable mA
*  to be returned are 100, 500, 900, and 1200 mA.
*
***************************************************************************/
int bcm59040_get_vsr_current_limit(BCM59040_chargers_t charger_type)
{
   int rc;

   rc = pmu_read(BCM59040_REG_MBCCTRL9);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_get_vsr_current_limit: error reading MBCCTRL9 register.\n");
      return rc;
   }

   if (charger_type == BCM59040_CHARGER_MAIN)
   {
       rc = (BCM59040_MBCCTRL9_WALL_ILIM_MASK & (unsigned char)rc) >> BCM59040_MBCCTRL9_WALL_ILIM_SHIFT;
   }
   else if (charger_type == BCM59040_CHARGER_USB)
   {
       rc = (BCM59040_MBCCTRL9_USB_ILIM_MASK & (unsigned char)rc);
   }
   else
   {
       PMU_DEBUG(DBG_ERROR, "bcm59040_get_vsr_current_limit: wrong charger type.\n");
       return -EINVAL;
   }

   return pmu_platform_map->vsr_curr_limit_to_mA_map[rc];
} /* bcm59040_get_vsr_current_limit */
EXPORT_SYMBOL(bcm59040_get_vsr_current_limit);


/****************************************************************************
*
*  bcm59040_charger_wd_control(int enable)
*
*  enable or disable charger watchdog.  Since mbcctrl1.0 is OTPed to be enabled
*  and it cannot be changed (read only), we have to individually enable/disable
*  watchdog for each charging phase.  For now, we will keep QC phase always
*  enabled and CV phase always disabled (same as OTP), and just enable/disable
*  cc1 and cc2 phases in mbcctrl2.
*
***************************************************************************/
static int bcm59040_charger_wd_control(int enable)
{
   int rc;

   rc = pmu_read(BCM59040_REG_MBCCTRL1);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_charger_wd_control: error reading MBCCTRL1 register.\n");
      return rc;
   }

   if (enable)
   {
       /* simply load sysparm which has the OTP values: CV mode WDT disabled */
       rc = ( (BCM59040_PMU_REG_MBCCTRL1_VAL & BCM59040_MBCCTRL1_CHGWDT_DV_DIS)
             | ((unsigned char)(rc) & ~BCM59040_MBCCTRL1_CHGWDT_DV_DIS) );
       rc = pmu_write( BCM59040_REG_MBCCTRL1, (uint8_t)rc );

       /* simply load sysparm which has the OTP values: cc1 = disabled, cc2 = 10 min */
       rc = pmu_write( BCM59040_REG_MBCCTRL2, (uint8_t)BCM59040_PMU_REG_MBCCTRL2_VAL );
       if (rc < 0)
       {
          PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_MBCCTRL2 register.\n");
          return rc;
       }
   }
   else  //disable
   {
       rc = ( BCM59040_MBCCTRL1_CHGWDT_DV_DIS | (unsigned char)(rc) );
       rc = pmu_write( BCM59040_REG_MBCCTRL1, (uint8_t)rc );

       rc = pmu_write( BCM59040_REG_MBCCTRL2, 0 );
       if (rc < 0)
       {
          PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_MBCCTRL2 register.\n");
          return rc;
       }
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_charger_wd_clear(void)
*
*  clear charger watchdog
*
***************************************************************************/
static int bcm59040_charger_wd_clear(void)
{
   int rc;

   /* read MBCCTRL1 */
   rc = pmu_read( BCM59040_REG_MBCCTRL1 );

   rc = (BCM59040_MBCCTRL1_CHGWDT_CLR | (unsigned char)(rc) );
   rc = pmu_write( BCM59040_REG_MBCCTRL1, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_MBCCTRL1 register.\n");
      return rc;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_charger_is_inserted
*
*  This function will return TRUE if either charger is inserted
*
***************************************************************************/
static int bcm59040_charger_is_inserted(int *chargerID)
{
   int rc;

   rc = pmu_read(BCM59040_REG_ENV1);
   if ( rc != -1 )
   {
      if( (u8)rc & (1<<1) )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59040_CHARGER_MAIN;
            return 1;
         }
      }
      if( (u8)rc & (1<<2) )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59040_CHARGER_USB;
            return 1;
         }
      }
   }
   return 0;
}
/****************************************************************************
*
*  bcm59040_event_notify
*
***************************************************************************/
static void bcm59040_event_notify(BCM59040_InterruptId_t irq_id)
{
   BCM_PMU_Event_t event = PMU_NUM_EVENTS;
   void *data = NULL;
   int val;

   PMU_DEBUG(DBG_ERROR,"irq_id=%d\n", irq_id);
   switch (irq_id)
   {
      // Onkey events
      case BCM59040_IRQID_INT1_PONKEYR:
         if(TempKeyLockBitSet)
         {
            if(PonkeyONOFF)
            {
               val = pmu_read(BCM59040_REG_PONKEYBCNTRL1);
               if (val != -1)
                  pmu_write(BCM59040_REG_PONKEYBCNTRL1, (u8)(val | BCM59040_PONKEYBCNTRL1_KEYLOCK));
               val = pmu_read(BCM59040_REG_PONKEYBCNTRL1);  // read back before set it off
               pmu_write(BCM59040_REG_PONKEYBCNTRL1, (u8)(val & 0xBF));
            }
         }
         event = PMU_EVENT_ONKEY_RISE;
         break;

      case BCM59040_IRQID_INT1_PONKEYF:
         event = PMU_EVENT_ONKEY_FALL;
         break;

      case BCM59040_IRQID_INT1_PONKEYH:
         if(TempKeyLockBitSet)
         {
            if(!PonkeyONOFF)
            {
               val = pmu_read(BCM59040_REG_PONKEYBCNTRL1);
               if (val != -1)
                  pmu_write(BCM59040_REG_PONKEYBCNTRL1, (u8)(val | BCM59040_PONKEYBCNTRL1_KEYLOCK));
               val = pmu_read(BCM59040_REG_PONKEYBCNTRL1);  // read back before set it off
               pmu_write(BCM59040_REG_PONKEYBCNTRL1, (u8)(val & 0xBF));
            }
         }
         event = PMU_EVENT_ONKEY_1S_HOLD;
         break;

      case BCM59040_IRQID_INT1_PONKEYBHD:
         event = PMU_EVENT_ONKEY_STANDBY;		// [JLH] Go into standby on this event
         break;

      case BCM59040_IRQID_INT1_RESTARTH:
         event = PMU_EVENT_ONKEY_RESTART;		// [JLH] Handle this if we want to prevent restart, hardware will power cycle
         break;

      case BCM59040_IRQID_INT2_CHGDET:
         event = PMU_EVENT_CHARGER_DETECTED;	// [JLH] charger type field in MBCCTRL9
         										// [JLH] need hook from USB stack to set current to 500 mA after enumeration
         										// [JLH] CHGTYP not set correctly for dedicated charger currently, PMU bug
         break;

      case BCM59040_IRQID_INT3_VSROVERV:
         event = PMU_EVENT_VSR_OVER_V;
         break;

      case BCM59040_IRQID_INT3_VSROVERI:
         event = PMU_EVENT_VSR_OVER_I;
         break;

      case BCM59040_IRQID_INT3_VCHGRNOTOK:
         event = PMU_EVENT_CHARGER_NOT_OK;
         break;
#ifdef CONFIG_BCM_PMU_BCM59040_B0
      case BCM59040_IRQID_INT3_CHG_WDT_ALARM:
         bcm59040_charger_wd_clear();
         event = PMU_EVENT_CHG_WDT_ALARM;
         break;
#endif

      case BCM59040_IRQID_INT3_VBUSLOWBND:
         event = PMU_EVENT_VBUSLOWBND;
         break;

      case BCM59040_IRQID_INT3_CHGERRDIS:
         atomic_set( &bcm59040_batt_status, POWER_SUPPLY_HEALTH_GOOD );
         event = PMU_EVENT_CHARGER_ERR_GONE;
         break;

      case BCM59040_IRQID_INT3_CHGWDTEXP:
         event = PMU_EVENT_CHGWDEXP;
         break;

      case BCM59040_IRQID_INT4_LDO1OVRI:
         event = PMU_EVENT_LDO1OVRI;
         break;

      case BCM59040_IRQID_INT4_LDO20VRI:
         event = PMU_EVENT_LDO2OVRI;
         break;

      case BCM59040_IRQID_INT4_LDO3OVRI:
         event = PMU_EVENT_LDO3OVRI;
         break;

      case BCM59040_IRQID_INT4_LDO4OVRI:
         event = PMU_EVENT_LDO4OVRI;
         break;

      case BCM59040_IRQID_INT4_LDO5OVRI:
         event = PMU_EVENT_LDO5OVRI;
         break;

      case BCM59040_IRQID_INT4_LDO6OVRI:
         event = PMU_EVENT_LDO6OVRI;
         break;

      case BCM59040_IRQID_INT5_IOSROVRI:
         event = PMU_EVENT_IOSROVRI;
         break;

      case BCM59040_IRQID_INT5_CSROVRI:
         event = PMU_EVENT_CSROVRI;
         break;

      case BCM59040_IRQID_INT5_IOSROVRV:
         event = PMU_EVENT_IOSROVRV;
         break;

      case BCM59040_IRQID_INT5_CSROVRV:
         event = PMU_EVENT_CSROVRV;
         break;

      case BCM59040_IRQID_INT5_RTC1S:
         event = PMU_EVENT_RTC1S;
         data = (void *)BCM59040_INT5_RTC1S;
         break;

      case BCM59040_IRQID_INT5_RTC60S:
         event = PMU_EVENT_RTC60S;
        data = (void *)BCM59040_INT5_RTC60S;
         break;

      case BCM59040_IRQID_INT5_RTCA1:
         event = PMU_EVENT_RTCA1;
         data = (void *)BCM59040_INT5_RTCA1;
         break;

      case BCM59040_IRQID_INT9_SARCONVEND:
         if(bcm59040_adc_current_req != NULL)
         {
            bcm59040_adc_latch_data(bcm59040_adc_current_req->channel);     //latch the requested channel
            data = (void *)bcm59040_adc_read_data();
         }
         event = PMU_EVENT_SARCONVEND;
         break;

      case BCM59040_IRQID_INT9_SARCONTCONVFAIL:
         event = PMU_EVENT_SARCONTCONVFAIL;
         break;

      case BCM59040_IRQID_INT9_SARASYNCONVOFF:
         event = PMU_EVENT_SARASYNCONVOFF;
         break;

      case BCM59040_IRQID_INT9_SARASYNREQFAIL:
         event = PMU_EVENT_SARASYNREQFAIL;
         break;

      case BCM59040_IRQID_INT1_PMUTOOWARM:			// [JLH] figure out handling so it doesn't overheat
         event = PMU_EVENT_PMUTOOWARM;
         break;

      case BCM59040_IRQID_INT1_HBINT:
         event = PMU_EVENT_HBINT;
         break;

      case BCM59040_IRQID_INT6_MBTEMPFAULT:
         atomic_set( &bcm59040_batt_status, POWER_SUPPLY_HEALTH_UNKNOWN );
         event = PMU_EVENT_MBTEMPFAULT;
         break;

      case BCM59040_IRQID_INT6_MBTEMPLOW:
         atomic_set( &bcm59040_batt_status, POWER_SUPPLY_HEALTH_UNSPEC_FAILURE );
         event = PMU_EVENT_MBTEMPLOW;
         break;

      case BCM59040_IRQID_INT6_MBTEMPHIGH:
         atomic_set( &bcm59040_batt_status, POWER_SUPPLY_HEALTH_OVERHEAT );
         event = PMU_EVENT_MBTEMPHIGH;
         break;

      // Battery and charger events
      case BCM59040_IRQID_INT6_MBRM:
         event = PMU_EVENT_BATTERY_REMOVED;
         break;

      case BCM59040_IRQID_INT6_MBOV:
         atomic_set( &bcm59040_batt_status, POWER_SUPPLY_HEALTH_OVERVOLTAGE );
         event = PMU_EVENT_BATTERY_OVER_VOLT;
         break;

      case BCM59040_IRQID_INT6_BATINS:
         event = PMU_EVENT_BATTERY_INSERTED;
         break;

      case BCM59040_IRQID_INT6_LOWBAT:
         event = PMU_EVENT_BATTERY_LOW;
         break;

      case BCM59040_IRQID_INT6_VERYLOWBAT:
         event = PMU_EVENT_BATTERY_VERY_LOW;
         break;

      case BCM59040_IRQID_INT7_ID_INSRT:		// [JLH] USB A connector inserted
         bcm59040_id_insert();
         event = PMU_EVENT_ID_INSERT;
         break;

      case BCM59040_IRQID_INT7_ID_RMV:			// [JLH] USB A connector removed
         bcm59040_id_remove();
         event = PMU_EVENT_ID_REMOVE;
         break;

      /* Useful for knowing when Vbus is not valid any more */
      case BCM59040_IRQID_INT7_VBUS_VALID_F:	// [JLh] probably only needed if we do software OTG, VBUS, B_SESS, A_SESS
         event = PMU_EVENT_VBUS_VLD_F;
         break;

      case BCM59040_IRQID_INT7_A_SESSVALID_F:
         event = PMU_EVENT_A_SESS_VLD_F;
         break;

      /* useful for knowing B session has ended, and discharge is almost complete
       (Vbus <= 0.5V)*/
      case BCM59040_IRQID_INT7_B_SESSEND_F:
         // @KP: 090902: we use this usb event to disconnect phy from the usb
         event = PMU_EVENT_B_SESS_END_F;
         data = (void *) BCM59040_INT7_B_SESSEND_F;
         break;

      /* useful for knowing Vbus is now valid */
      case BCM59040_IRQID_INT7_VBUS_VALID_R:
         // @KP: 090902: we use this usb event to start delay timer before allowing enumeration to occur
         event = PMU_EVENT_VBUS_VLS_R;
         data = (void *) BCM59040_INT7_VBUS_VALID_R;
         break;

      /* useful for knowing A session is valid.  If we are A device, then
         this could mean that B device is signally SRP */
      case BCM59040_IRQID_INT7_A_SESSVALID_R:
         event = PMU_EVENT_A_SESS_VLD_R;
         break;

      /* useful for knowing Vbus has risen above B session end level */
      case BCM59040_IRQID_INT7_B_SESSEND_R:
         event = PMU_EVENT_B_SESS_END_R;
         break;

         /* This will tell baseband start SRP so that recharge can begin  */
      case BCM59040_IRQID_INT9_RESUME_VBUS:
         val = pmu_read( BCM59040_REG_ENV4 );
         if (val & BCM59040_ENV4_B_SESS_END)
         {
            /* no voltage on Vbus, could mean no cable or vbus is off. Upper layer needs to do SRP.
               After SRP is done, if Vbus comes back, it means cable is still connected.  Upper layer
               might want to check if vbus is in suspend.  If so, it might want to un-suspend it */
             event = PMU_EVENT_RESUME_VBUS_ZERO;
         }
         else if (val & BCM59040_ENV4_VBUS_VALID)
         {
             /* Valid voltage on vbus.  We could be in suspend, only USB stack knows. If in suspend,
                USB stack needs to un-suspend it. */
             event = PMU_EVENT_RESUME_VBUS_VLD;
         }
         else /* vbus is between 4.5V and 0.5V.  Most likely, Vbus is transitioning.  Best is for USB
                stack to wait for a bit and then query for Vbus voltage again, and then decide what to do
                next */
         {
             event = PMU_EVENT_RESUME_VBUS;
         }
         break;

      case BCM59040_IRQID_INT9_ID_CHNG:					// [JLH] ACA Support: this must be handled
         bcm59040_id_change();
         event = PMU_EVENT_ID_CHANGE;
         break;

      case BCM59040_IRQID_INT2_USBINS:
         isUSBChargerPresent = 1;
         PMU_DEBUG(DBG_INFO,"USB charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59040_CHARGER_USB;

         isUSBChargerPresent = 1;
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent); // make sure EOC interrupt is enabled
         break;

      case BCM59040_IRQID_INT2_USBRM:
         PMU_DEBUG(DBG_INFO,"USB charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59040_CHARGER_USB;

         isUSBChargerPresent = 0;
         mbc_maintain.chargerEOC = 0;
         break;

      case BCM59040_IRQID_INT2_CHGINS:
         PMU_DEBUG(DBG_INFO,"Main charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59040_CHARGER_MAIN;

         isWallChargerPresent = 1;
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent); // make sure EOC interrupt is enabled
         break;

      case BCM59040_IRQID_INT2_CHGRM:
         PMU_DEBUG(DBG_INFO,"Main charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59040_CHARGER_MAIN;

         isWallChargerPresent = 0;
         mbc_maintain.chargerEOC = 0;
         break;

      case BCM59040_IRQID_INT2_EOC:
         if ((!isUSBChargerPresent) && (!isWallChargerPresent))
         {
            PMU_DEBUG(DBG_TRACE2,"Charger EOC ignored\n");
         }
         else
         {
            mbc_maintain.chargerEOC = 1;
         }
         // Disable the EOC interrupt here to avoid nested batful interrupts
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);
         val = pmu_read( BCM59040_REG_INT2M );
         val |= BCM59040_INT2_EOC;
         pmu_write( BCM59040_REG_INT2M, val);
         event = PMU_EVENT_BATTERY_FULL;
         break;

      case BCM59040_IRQID_INT2_CHGOVERV:
      case BCM59040_IRQID_INT2_USBOVERV:
         PMU_DEBUG(DBG_ERROR,"Charger voltage is greater than over-voltage threshold\n");
         event = PMU_EVENT_CHARGER_ERROR;
         break;

      default:
         break;
   }

#if 1
   // Notify PMU
   if (event != PMU_NUM_EVENTS)
   {
      //PMU_DEBUG("event=%d data=%p\n", event, data);
      pmu_event_notify(PMU_BCM59040, event, data);
   }
#endif
}

/****************************************************************************
*
*  bcm59040_state_to_opmod
*
***************************************************************************/
static int bcm59040_state_to_opmod(BCM_PMU_Regulator_State_t state, u8 *opmod)
{
   if (!opmod)
      return -1;

   switch (state)
   {
      case PMU_Regulator_Off:
         *opmod = BCM59040_BIT_REG_OFF;
         break;
      case PMU_Regulator_On:
         *opmod = BCM59040_BIT_REG_ON;
         break;
      case PMU_Regulator_Eco:
         *opmod = BCM59040_BIT_REG_ECO;
         break;
      default:
         return -1;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_opmod_to_state
*
*  need to know the currene power mode and then get the state accordingly
***************************************************************************/
static int bcm59040_opmod_to_state(u8 opmod, BCM_PMU_Regulator_State_t *state)
{
   int rc;

   if (!state)
      return -1;

   rc = pmu_read( BCM59040_REG_HBCTRL );
   rc = (rc & BCM59040_HBCTRL_PC_I2C_MASK) >> 6; // rc has the value of PMx

   switch (rc)
   {
     case 0:
       opmod = opmod & BCM59040_PMCTRL_PM0_MASK;
       break;
     case 1:
       opmod = (opmod & BCM59040_PMCTRL_PM1_MASK) >> 2;
       break;
     case 2:
       opmod = (opmod & BCM59040_PMCTRL_PM2_MASK) >> 4;
       break;
     case 3:
       opmod = (opmod & BCM59040_PMCTRL_PM3_MASK) >> 6;
       break;
   }

   switch (opmod)
   {
      case BCM59040_BIT_REG_OFF:
         *state = PMU_Regulator_Off;
         break;
      case BCM59040_BIT_REG_ON:
      case BCM59040_BIT_REG_ON_ALT:
         *state = PMU_Regulator_On;
         break;
      case BCM59040_BIT_REG_ECO:
         *state = PMU_Regulator_Eco;
         break;
      default:
         return -1;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59040_is_dvs_enabled
*
*       Called to check if dvs for csr is enabled.
*
***************************************************************************/
int bcm59040_is_dvs_enabled()
{
   int value;
   value = pmu_read(BCM59040_REG_CSRCTRL1);
   value &= BCM59040_CSRCTRL1_DVS_ENABLE;
   if (value)
      return 1;
   else
      return 0;
}

#ifndef CONFIG_BCM_PMU_BCM59040_B0 
/****************************************************************************
*
*  bcm59040_enable_dvs
*
*       Called to enable dvs
*
***************************************************************************/
void bcm59040_enable_dvs(void)
{
   int value;
   value = pmu_read(BCM59040_REG_CSRCTRL1);
   value |= BCM59040_CSRCTRL1_DVS_ENABLE;
   pmu_write(BCM59040_REG_CSRCTRL1,value);
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_to_mV_map	= pmu_platform_map->map_dvs_config.vout_to_mV_map;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].map_size		= pmu_platform_map->map_dvs_config.map_size;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].reg_addr_volt		= pmu_platform_map->map_dvs_config.reg_addr_volt;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].min_mV		= pmu_platform_map->map_dvs_config.min_mV;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].max_mV		= pmu_platform_map->map_dvs_config.max_mV;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].mV_step		= pmu_platform_map->map_dvs_config.mV_step;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_mask		= pmu_platform_map->map_dvs_config.vout_mask;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_shift		= pmu_platform_map->map_dvs_config.vout_shift;
}

/****************************************************************************
*
*  bcm59040_disable_dvs
*
*       Called to disable dvs
*
***************************************************************************/
void bcm59040_disable_dvs(void)
{
   int value;
   value = pmu_read(BCM59040_REG_CSRCTRL1);
   value &= ~(BCM59040_CSRCTRL1_DVS_ENABLE);
   pmu_write(BCM59040_REG_CSRCTRL1,value);
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_to_mV_map	= pmu_platform_map->map_nodvs_config.vout_to_mV_map;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].map_size		= pmu_platform_map->map_nodvs_config.map_size;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].reg_addr_volt		= pmu_platform_map->map_nodvs_config.reg_addr_volt;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].min_mV		= pmu_platform_map->map_nodvs_config.min_mV;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].max_mV		= pmu_platform_map->map_nodvs_config.max_mV;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].mV_step		= pmu_platform_map->map_nodvs_config.mV_step;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_mask		= pmu_platform_map->map_nodvs_config.vout_mask;
   bcm59040_regulator_map[BCM59040_REGULATOR_CSR].vout_shift		= pmu_platform_map->map_nodvs_config.vout_shift;
}
#endif


/****************************************************************************
*
*  bcm59040_CSR_PC_DVS_init
*
*  CSR_PC_DVSx and IOSR_PC_DVSx initialization
*
***************************************************************************/

static int bcm59040_CSR_PC_DVS_init(void)
{
   int i;
   int rc = 0;
   u8 regAddr;
#ifdef CONFIG_BCM_PMU_BCM59040_B0
   u8 csrVout[4],iosrVout[4];

   //for B0, init the CSR/IOSR voltage in PMx mode by sysparam value
   csrVout[0] = BCM59040_PMU_REG_CSRPCDVS0_VAL;
   csrVout[1] = BCM59040_PMU_REG_CSRPCDVS1_VAL;
   csrVout[2] = BCM59040_PMU_REG_CSRPCDVS2_VAL;
   csrVout[3] = BCM59040_PMU_REG_CSRPCDVS3_VAL;
   iosrVout[0] = BCM59040_PMU_REG_IOSRPCDVS0_VAL;
   iosrVout[1] = BCM59040_PMU_REG_IOSRPCDVS1_VAL;
   iosrVout[2] = BCM59040_PMU_REG_IOSRPCDVS2_VAL;
   iosrVout[3] = BCM59040_PMU_REG_IOSRPCDVS3_VAL;
#endif


   regAddr = BCM59040_REG_CSRPCDVS0;

   for(i=0;i<=3;i++)
   {
#ifndef CONFIG_BCM_PMU_BCM59040_B0
      //init CSR_PC_DVSx
      if( pmu_platform_map->PMx_map[i].csr.DVSenabled ) //DVS enabled
      {
         if(pmu_platform_map->PMx_map[i].csr.state == PMU_Regulator_On)  //NM
         {
            rc = pmu_read( BCM59040_REG_CSRCTRL10 );
            rc &= BCM59040_CSR_VOUT_MASK;
            rc = pmu_write(regAddr, (u8)rc);
         }
         else if(pmu_platform_map->PMx_map[i].csr.state == PMU_Regulator_Eco)  //LPM
         {
            rc = pmu_read( BCM59040_REG_CSRCTRL1 );
            rc &= BCM59040_CSR_VOUT_MASK;
            rc = pmu_write(regAddr, (u8)rc);
         }
         else  //CSR OFF
         {
            rc = pmu_write(regAddr, 0x1F);  //no voltage
         }
      }
      else  //DVS disabled
      {
         rc = pmu_read( BCM59040_REG_CSRCTRL2 );
         rc = (rc >> 2) & BCM59040_CSR_VOUT_MASK;
         rc = pmu_write(regAddr, (u8)rc);
      }

      //init IOSR_PC_DVSx
      if(pmu_platform_map->PMx_map[i].iosr == PMU_Regulator_Off)
      {
         rc = pmu_write(regAddr+5, 0x1F); //no voltage
      }
      else
      {
         rc = pmu_read( BCM59040_REG_IOSRCTRL2 );
         rc = (rc >> 2) & BCM59040_IOSRCTRL2_VOUT_MASK;
         rc = pmu_write(regAddr+5, (u8)rc);
      }

#else
      rc = pmu_write(regAddr,csrVout[i]);
      rc = pmu_write(regAddr+5,iosrVout[i]);
#endif

      regAddr++;

      if (rc < 0)
      {
         PMU_DEBUG(DBG_ERROR, "error writing xSR_PC_DVS%d register.\n", i);
         return rc;
      }
   }

   return 0;
}



/****************************************************************************
*
*  bcm59040_otg_init(void)
*
*  Notes: If hardware control is selected, PMU automatically cause
*         GPIO1 pin be selected as OTGSHD pin, and GPIO2 pin as vbus_pulse pin
*         Both GPIO 1 & 2 are input pins regardless of GPIO_DIR values
*
***************************************************************************/
static int bcm59040_otg_init( void )
{
   int rc;

   /* check for float vs float_legacy */
   rc = pmu_read( BCM59040_REG_OTGCTRL1 );
   if (BCM59040_OTGCTRL1_FLOATLEGACY & rc )
   {
       otg_maintain.float_legacy = 1;
   }
   else
   {
       otg_maintain.float_legacy = 0;
   }

   /* configure otg related registers */
   rc = BCM59040_PMU_REG_OTGCTRL2_VAL & 0xEF;
   rc = pmu_write( BCM59040_REG_OTGCTRL2, (unsigned char)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_OTGCTRL2 register.\n");
      return rc;
   }

   if (BCM59040_PMU_REG_OTGCTRL2_VAL & BCM59040_OTGCTRL2_HWVSSW)
   {
       otg_maintain.hw_otg_ctrl = 1;
   }
   else
   {
       otg_maintain.hw_otg_ctrl = 0;
   }


   otg_maintain.otg_block_enabled = 1; // init to true. TODO: make sure Host's GPIO1 pin is set to hi on board
   otg_maintain.otg_boost_enabled = 0; // TODO: once Host mode is supported, if ID = ground, this should be 1

   rc = pmu_read( BCM59040_REG_ENV4 );
   bcm59040_check_id_from_env((unsigned char)rc);

   return 0;
}

#if 0
/*****************************************************************************
*
*      bcm59040_check_otg_stat
*
*      Checks for PMU OTG status, just a debug function
*
*
*****************************************************************************/
static void bcm59040_check_otg_stat( void )
{
   PMU_DEBUG(DBG_INFO, "bcm59040_check_otg_stat: hw_otg_ctrl is %d\n\r", otg_maintain.hw_otg_ctrl);
   PMU_DEBUG(DBG_INFO, "bcm59040_check_otg_stat: float_legacy is %d\n\r", otg_maintain.float_legacy);
   PMU_DEBUG(DBG_INFO, "bcm59040_check_otg_stat: otg_role is %d\n\r", otg_maintain.otg_role);
   PMU_DEBUG(DBG_INFO, "bcm59040_check_otg_stat: otg_block_enabled is %d\n\r", otg_maintain.otg_block_enabled);
   PMU_DEBUG(DBG_INFO, "bcm59040_check_otg_stat: otg_boost_enabled is 0x%x\n\r", otg_maintain.otg_boost_enabled);
}
#endif

/****************************************************************************
*
*  bcm59040_update_otg_stat
*
*  For host to set OTG status when using hardware based OTG control
*
***************************************************************************/
static void bcm59040_update_otg_stat(OTG_Stat_t OTG_Stat, int enable)
{
    switch (OTG_Stat)
    {
        case OTG_BOOST_STAT:
            if (enable)
                otg_maintain.otg_boost_enabled = 1;
            else
                otg_maintain.otg_boost_enabled = 0;
            PMU_DEBUG(DBG_INFO, "bcm59040_update_otg_stat: boost regulator = %d\n", enable );
            break;

        case OTG_BLOCK_STAT:
            if (enable)
                otg_maintain.otg_block_enabled = 1;
            else
                otg_maintain.otg_block_enabled = 0;
            PMU_DEBUG(DBG_INFO, "bcm59040_update_otg_stat: otg block = %d\n", enable );
            break;
    }
}

/******************************************************************************
*
*   bcm59040_get_otg_role
*
*   return OTG_Role_t from otg_maintain structure, used by upper layer to find
*   out which otg role (e.g. A dev, RID_A, etc) we are in
*
******************************************************************************/
static BCM_PMU_OTG_Role_t bcm59040_get_otg_role(void)
{
    return otg_maintain.otg_role;
}

/******************************************************************************
*
*   bcm59040_check_id_from_env
*
*   given env4 byte as input, check id pin status and update otg_maintain
*
*******************************************************************************/
static void bcm59040_check_id_from_env(unsigned char env_byte)
{
   switch (BCM59040_ENV4_ID_IN_MASK & env_byte)
   {
      case BCM59040_ENV4_A_DEV:
         otg_maintain.otg_role = OTG_A_DEV;
         break;

      case BCM59040_ENV4_B_DEV_LEGACY:
         otg_maintain.otg_role = OTG_B_DEV_LEGACY;
         break;

      case BCM59040_ENV4_RID_A:
         otg_maintain.otg_role = OTG_RID_A;
         break;

      case BCM59040_ENV4_RID_B:
         otg_maintain.otg_role = OTG_RID_B;
         break;

      case BCM59040_ENV4_RID_C:
         otg_maintain.otg_role = OTG_RID_C;
         break;

      case BCM59040_ENV4_B_DEV_FLOAT:
         otg_maintain.otg_role = OTG_B_DEV_FLOAT;
         break;
   }
}

/******************************************************************************
*
*   bcm59040_id_insert
*
*   ISR for ID insertion detection, occurring when ID_OUT changes from hi to lo
*
*******************************************************************************/
static int bcm59040_id_insert(void)
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_id_insert\n");

   rc = pmu_read( BCM59040_REG_ENV4 );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_id_insert: error reading ENV4 register.\n");
      return rc;
   }

   /* find out otg role: should be either A dev or RID_A */
   bcm59040_check_id_from_env((unsigned char)rc);

   /* For 59035 C0 and later, USB charger automatically disabled if vbus is turned on */
   bcm59040_update_otg_stat( OTG_BOOST_STAT, (rc & BCM59040_ENV4_OFFVBUSb)? 1:0 );
   bcm59040_update_otg_stat( OTG_BLOCK_STAT, (rc & BCM59040_ENV4_OTGSHDNb)? 1:0 );

   /* enable boost (we only support HW based otg control) */
   if (otg_maintain.hw_otg_ctrl)
   {
      /* Just need to make sure OTG block is enabled on the hardware by BB; nothing can be done by the software.
       Bringup board's GPIO2 is connected to PMU's OTGSHDN pin(PMU GPIO1).  Make sure this pin is set to hi. */
   }

   return 0;
}

/******************************************************************************
*
*   bcm59040_id_change
*
*   ISR for ID change, occurring when ID changes value.  Host needs to react
*   to ID change based on new ID.  E.g. Was in RID_A mode, then user suddenly
*   removes USB charger, ID will change to gnd and int9.7 will be issued.
*   Host needs to enable external boost immediately.
*   E.g. ID changes from A to RID_A, host needs to disable external boost immediately
*   (via HW based control)
*
*******************************************************************************/
static int bcm59040_id_change(void)
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_id_change\n");

   rc = pmu_read( BCM59040_REG_ENV4 );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_id_change: error reading ENV4 register.\n");
      return rc;
   }

   /* find out otg role */
   bcm59040_check_id_from_env((unsigned char)rc);

   /* For 59040 B0 and later, USB external FET automatically open if external boost is turned on */
   bcm59040_update_otg_stat( OTG_BOOST_STAT, (rc & BCM59040_ENV4_OFFVBUSb)? 1:0 );
   bcm59040_update_otg_stat( OTG_BLOCK_STAT, (rc & BCM59040_ENV4_OTGSHDNb)? 1:0 );

   return 0;
}

/******************************************************************************
*
*   bcm59040_id_remove
*
*   ISR for ID removal detection, occurring when ID_OUT changes from lo to hi
*
*******************************************************************************/
static int bcm59040_id_remove(void)
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_id_remove\n");

   rc = pmu_read( BCM59040_REG_ENV4 );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "bcm59040_id_remove: error reading ENV4 register.\n");
      return rc;
   }

   /* find out otg role: should be either OTG_B_DEV_FLOAT, OTG_B_DEV_LEGACY, RID_B, orRID_C  */
   bcm59040_check_id_from_env((unsigned char)rc);

   /* For 59035 C0 and later, USB charger automatically disabled if vbus is turned on */
   bcm59040_update_otg_stat( OTG_BOOST_STAT, (rc & BCM59040_ENV4_OFFVBUSb)? 1:0 );
   bcm59040_update_otg_stat( OTG_BLOCK_STAT, (rc & BCM59040_ENV4_OTGSHDNb)? 1:0 );

   /* turn off boost and OTG block, assming hw based otg control is used */
   if (otg_maintain.hw_otg_ctrl)
   {
       /* Currently, Host's gpio 2 is driving PMU's OTG block.
         So the OTG block has to be turned on by setting this gpio to high.
         TODO: make sure this gpio is set to high

         disable boost regulator by calling stack API to write '0' to bit 12 (port power bit) of
         HPRT register to reset vbus.  This causes voffbus signal to go low */
   }

   return 0;
}

//******************************************************************************
//
// Function Name:   void bcm59040_fuelgauge_init( void )
//
// Description:     Fuel Gauge Initialization routine
//
// Notes:
//
//******************************************************************************
static void bcm59040_fuelgauge_init( void )
{
   int rc;

   /* configure FGCTRL registers for initializing some non-OTPable fields */
   rc = BCM59040_PMU_REG_FGCTRL1_VAL;
   rc = pmu_write( BCM59040_REG_FGCTRL1, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_FGCTRL1 register.\n");
      return;
   }

   rc = BCM59040_PMU_REG_FGCTRL3_VAL;
   rc = pmu_write( BCM59040_REG_FGCTRL3, (uint8_t)rc );
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_FGCTRL3 register.\n");
      return;
   }

   /* initialize FG data structure */
   fg_maintain.fg_state = BCM59040_FG_OFF;
   if (BCM59040_PMU_REG_FGCTRL3_VAL & BCM59040_FGCTRL3_FGSYNCMODE)
       fg_maintain.fg_cont_mode = 0;
   else
       fg_maintain.fg_cont_mode = 1;
   fg_maintain.fg_sample = 0;

   fg_maintain.fg_calibrate_done = 0;

   bcm59040_fuelgauge_enable(1);
   bcm59040_fuelgauge_clear_acc_cnt();  // clears Accum and Counter registers before starting
   bcm59040_fuelgauge_calibrate();
}

//******************************************************************************
//
// Function Name:   void bcm59040_fuelgauge_enable( int enable )
//
// Description:     Start or stop Fuel Gauge
//
// Notes:
//
//******************************************************************************
static void bcm59040_fuelgauge_enable( int enable )
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_fuelgauge_enable\n");

   /* read FGCTRL1 */
   rc = pmu_read(BCM59040_REG_FGCTRL1);

   if (enable)
   {
       rc |= BCM59040_FGCTRL1_FGHOSTEN;
       if (fg_maintain.fg_cont_mode)
       {
           fg_maintain.fg_state = BCM59040_FG_ON_CONT_MODE;
       }
       else
       {
           fg_maintain.fg_state = BCM59040_FG_ON_SYNC_MODE;
       }
   }
   else
   {
       rc &= ~BCM59040_FGCTRL1_FGHOSTEN;
       fg_maintain.fg_state = BCM59040_FG_OFF;
   }

   rc = pmu_write( BCM59040_REG_FGCTRL1, (uint8_t)rc );
}

/****************************************************************************
*
*  bcm59040_fuelgauge_clear_acc_cnt( void )
*
*  Clears FG accum and Counter registers before starting Fuel Gauge
*
*  Notes: Clearing acc. instead of calibrating. Testing shows that
*         calibration is effective only when the system is up and running.
*
***************************************************************************/
static int bcm59040_fuelgauge_clear_acc_cnt( void )
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_fuelgauge_clear_acc_cnt.\n");

   rc = pmu_read( BCM59040_REG_FGCTRL3 );
   rc = BCM59040_FGCTRL3_FGFRZREAD | (unsigned char)rc;
   rc = pmu_write( BCM59040_REG_FGCTRL3, (unsigned char)rc );

   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing BCM59040_REG_FGCTRL3 register.\n");
      return rc;
   }

   return 0;
}

/****************************************************************************
*
*  bcm59040_fuelgauge_calibrate( void )
*
*  Host to calibrate FG.  Calibration time is about 50 msec. Once it's
*  done, FGCAL bit will clear itself
*
*  Notes:  added this routine for abbas.  Calibration takes about 50 msec,
*          so need to wait for that amount of time before accessing FG
***************************************************************************/
static void bcm59040_fuelgauge_calibrate( void )
{
   int rc;

   PMU_DEBUG(DBG_INFO, "bcm59040_fuelgauge_calibrate.\n");

   rc = pmu_read( BCM59040_REG_FGCTRL3 );
   rc = BCM59040_FGCTRL3_FGCAL | (unsigned char)rc;
   pmu_write( BCM59040_REG_FGCTRL3, (unsigned char)rc );

   fg_maintain.fg_calibrate_done = 0;
   fg_maintain.fg_calibrate_done_time = jiffies + HZ/14; // wait for 70 msec before checking calibration offset
}

/******************************************************************************
*
*  int bcm59040_fuelgauge_sample_to_integer( u8 *smpl )
*
*  Translate 2 bytes FG sample value to a 2's complement integer
*
******************************************************************************/
static int bcm59040_fuelgauge_sample_to_integer(u8 *smpl)
{
   int rc;
   u16 tmp;

   tmp = (smpl[0] | (smpl[1] << 8)) & BCM59040_FGSAMPLE_MASK;

   if(tmp & BCM59040_FGSAMPLE_SIGN_BIT) //negative sample value
   {
      rc = -((~tmp + 1) & BCM59040_FGSAMPLE_MASK);
   }
   else
   {
      rc = tmp;
   }

   return rc;

}


/******************************************************************************
*
*  int bcm59040_get_fuelgauge_sample_reg( short *fgsmpl )
*
*  Read either FGSMPL1~2 sample A registers (slower update: every 500 msec) or
*  FGSMPLB1~2 sample B registers (faster update).Sample registers are 14-bit signed, 
*  with bit-13 being the sign bit.
*
******************************************************************************/
static int bcm59040_get_fuelgauge_sample_reg( short *fgsmpl, int fast )
{
   int rc;
   int smpl1,smpl2;
   u8 sample[2], fgsmpl_reg;
   u16 tmp;

   PMU_DEBUG(DBG_INFO, "bcm59040_get_fuelgauge_sample_reg\n");

   if (!(fg_maintain.fg_calibrate_done))
   {
       // check time elapsed since calibration; need to have at least 50 msec
       if(time_after(jiffies,fg_maintain.fg_calibrate_done_time))
       {
           rc = pmu_i2c_read_bytes(BCM59040_REG_FGTRIMGN1_1, sample, 2);

           if ( rc < 0)
           {
              printk("error reading FG offset registers\n");
              return -EINVAL;
           }

           tmp = (sample[0] | (sample[1] << 8)) & BCM59040_FGOFFSET_MASK;
           tmp = (~tmp + 1) & BCM59040_FGOFFSET_MASK;  //absolute value, notice that always negative value in FGOFFSET
           fg_maintain.fg_offset = tmp;
           fg_maintain.fg_calibrate_done = 1; // calibration done
       }
       else
       {
           printk("PMU Fuel gauge calibration is not yet done.\n");
           return -EINVAL;
       }
   }

   if(fast)
   {
      fgsmpl_reg = BCM59040_REG_FGSMPLB1;
   }
   else
   {
      fgsmpl_reg = BCM59040_REG_FGSMPL1;
   }

   rc = pmu_i2c_read_bytes(fgsmpl_reg, sample, 2);
   if ( rc < 0)
   {
      printk("error reading FG sample registers\n");
      return -EINVAL;
   }
   smpl1 = bcm59040_fuelgauge_sample_to_integer(sample);


   rc = pmu_i2c_read_bytes(fgsmpl_reg, sample, 2);
   if ( rc < 0)
   {
      printk("error reading FG sample registers\n");
      return -EINVAL;
   }
   smpl2 = bcm59040_fuelgauge_sample_to_integer(sample);

   if ( ((fast) && ((smpl2-smpl1)>50 || (smpl2-smpl1)<-50))  ||
        ((!fast) && ((smpl2-smpl1)>2 || (smpl2-smpl1)<-2))   )
   {
      rc = pmu_i2c_read_bytes(fgsmpl_reg, sample, 2);

      if ( rc < 0)
      {
         printk("error reading FG sample registers\n");
         return -EINVAL;
      }
      smpl2 = bcm59040_fuelgauge_sample_to_integer(sample);
   }

   if(smpl2 < 0) //negative sample value
   {
      fg_maintain.fg_sample = smpl2 + fg_maintain.fg_offset;
#ifdef PMU_59040_FG_LAYOUT
      smpl1 = fg_maintain.fg_sample;
      smpl1 = smpl1 * 50 / 57;   //1.14
      fg_maintain.fg_sample = smpl1;
#endif
   }
   else
   {
      fg_maintain.fg_sample = smpl2 - fg_maintain.fg_offset;
#ifdef PMU_59040_FG_LAYOUT
      smpl1 = fg_maintain.fg_sample;
      smpl1 = smpl1 * 20 / 23;   //1.15
      fg_maintain.fg_sample = smpl1;
#endif
   }

   *fgsmpl = fg_maintain.fg_sample;

   PMU_DEBUG(DBG_INFO, "bcm59040_get_fuelgauge_sample_reg: sample = %d\n", fg_maintain.fg_sample);

   return 0;
}

static struct platform_driver bcm59040_module_drv = {
   .driver = {
              .name = "bcm59040-pmu",
              .owner = THIS_MODULE,
              },
};

static int bcm59040_module_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res) {
		printk(KERN_ERR 
			"bcm59040_module: platform_get_resource "
			"IORESOURCE_IO error.\n");
		return -ENODEV;
	}

	bcm59040_module_gpio	= res->start;

	pmu_platform_map	= pdev->dev.platform_data;

	PMU_DEBUG(DBG_TRACE,"register with PMU module\n");
   	return pmu_register_device(PMU_BCM59040, &bcm59040_ops, pdev->resource);
}

/****************************************************************************
*
*  bcm59040_module_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init bcm59040_module_init( void )
{
	int ret;

	printk( banner );

	ret = platform_driver_probe(&bcm59040_module_drv, bcm59040_module_probe);
	return ret;
}


/****************************************************************************
*
*  bcm59040_module_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit bcm59040_module_exit( void )
{
	platform_driver_unregister(&bcm59040_module_drv);
	PMU_DEBUG(DBG_TRACE,"module_exit called\n");
}


fs_initcall(bcm59040_module_init);
module_exit(bcm59040_module_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM59040 Driver");


EXPORT_SYMBOL( bcm59040_usb_event_register );
EXPORT_SYMBOL( bcm59040_batt_status );
