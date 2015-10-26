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
*  pmu_chip.h
*
*  PURPOSE:
*
*  This file defines the common interface to a PMU chip.
*
*  NOTES:
*
*****************************************************************************/

#if !defined( BCM_PMU_CHIP_H )
#define BCM_PMU_CHIP_H

#if 0
/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>
#include <linux/fs.h>

#ifdef __KERNEL__ 
#include <linux/i2c.h>
#include <linux/interrupt.h>
#endif

#include <linux/broadcom/PowerManager.h>

/* ---- Constants and Types ---------------------------------------------- */

#define BCM_PMU_MAGIC   'P'

#define BCM_PMU_CMD_FIRST                       0x80

#define BCM_PMU_CMD_ENABLE_INTS                 0x80
#define BCM_PMU_CMD_DISABLE_INTS                0x81
#define BCM_PMU_CMD_READ_BYTES_REG              0x82
#define BCM_PMU_CMD_READ_REG                    0x83
#define BCM_PMU_CMD_WRITE_REG                   0x84
#define BCM_PMU_CMD_ACTIVATESIM                 0x85
#define BCM_PMU_CMD_DEACTIVATESIM               0x86
#define BCM_PMU_CMD_GET_REGULATOR_STATE         0x87
#define BCM_PMU_CMD_SET_REGULATOR_STATE         0x88
#define BCM_PMU_CMD_SET_PWM_LED_CTRL            0x89
#define BCM_PMU_CMD_SET_PWM_HI_PER              0x8a
#define BCM_PMU_CMD_SET_PWM_LO_PER              0x8b
#define BCM_PMU_CMD_SET_PWM_PWR_CTRL            0x8c
#define BCM_PMU_CMD_GET_VOLTAGE                 0x8d
#define BCM_PMU_CMD_SET_VOLTAGE                 0x8e
#define BCM_PMU_CMD_SET_ADC_CONT_MODE_SEL       0x8f
#define BCM_PMU_CMD_SET_ADC_ASYN_MODE_CTRL      0x90
#define BCM_PMU_CMD_SET_ADC_LATCH_DATA          0x91
#define BCM_PMU_CMD_GET_ADC_READ_DATA           0x92
#define BCM_PMU_CMD_SET_ADC_RESET_COUNT         0x93
#define BCM_PMU_CMD_SET_POWERMODE_PC_I2C_CTRL   0x95
#define BCM_PMU_CMD_SET_CHARGING_CURRENT        0x96 
#define BCM_PMU_CMD_GET_CHARGING_CURRENT        0x97 
#define BCM_PMU_CMD_SET_VSR_CURRENT_LMT         0x98 
#define BCM_PMU_CMD_GET_VSR_CURRENT_LMT         0x99 
#define BCM_PMU_CMD_SET_CHARGER_WDT_CTRL        0x9a 
#define BCM_PMU_CMD_SET_CHARGER_WDT_CLEAR       0x9b 
#define BCM_PMU_CMD_SET_CHARGER_FC_OPTION       0x9c 
#define BCM_PMU_CMD_CHECK_IF_STANDARD_HOST_PORT 0x9d 
#define BCM_PMU_CMD_SET_USB_CHARGER_TYPE        0x9e 
#define BCM_PMU_CMD_GET_OTG_ROLE                0x9f
#define BCM_PMU_CMD_SET_GPIO_READ_DATA          0xa0
#define BCM_PMU_CMD_SET_GPIO_WRITE_DATA         0xa1
#define BCM_PMU_CMD_SET_GPIO_SET_DIRECTION      0xa2
#define BCM_PMU_CMD_SET_GPIO_SET_MODE           0xa3
#define BCM_PMU_CMD_SET_PONKEY_KEYLOCK          0xa4
#define BCM_PMU_CMD_GET_RTC_TIME                0xa5
#define BCM_PMU_CMD_SET_RTC_TIME                0xa6
#define BCM_PMU_CMD_GET_RTC_ALARM_TIME          0xa7
#define BCM_PMU_CMD_SET_RTC_ALARM_TIME          0xa8
#define BCM_PMU_CMD_WRITE_BYTES_REG             0xa9
#define BCM_PMU_CMD_GET_FG_SAMPLE               0xaa 

#define BCM_PMU_CMD_LAST                        0xaa 

#define BCM_PMU_IOCTL_ENABLE_INTS                  _IO(  BCM_PMU_MAGIC, BCM_PMU_CMD_ENABLE_INTS )       // arg is unused
#define BCM_PMU_IOCTL_DISABLE_INTS                 _IO(  BCM_PMU_MAGIC, BCM_PMU_CMD_DISABLE_INTS )      // arg is unused
#define BCM_PMU_IOCTL_READ_REG                     _IOWR(BCM_PMU_MAGIC, BCM_PMU_CMD_READ_REG, BCM_PMU_Reg_t )
#define BCM_PMU_IOCTL_READ_BYTES_REG               _IOWR(BCM_PMU_MAGIC, BCM_PMU_CMD_READ_BYTES_REG, BCM_PMU_Reg_bytes_t )
#define BCM_PMU_IOCTL_WRITE_REG                    _IOW( BCM_PMU_MAGIC, BCM_PMU_CMD_WRITE_REG, BCM_PMU_Reg_t )
#define BCM_PMU_IOCTL_WRITE_BYTES_REG              _IOW( BCM_PMU_MAGIC, BCM_PMU_CMD_WRITE_BYTES_REG, BCM_PMU_Reg_bytes_t )
#define BCM_PMU_IOCTL_ACTIVATESIM                  _IOW( BCM_PMU_MAGIC, BCM_PMU_CMD_ACTIVATESIM, PM_SimVoltage_t )
#define BCM_PMU_IOCTL_DEACTIVATESIM                _IO( BCM_PMU_MAGIC, BCM_PMU_CMD_DEACTIVATESIM )  // arg is unused
#define BCM_PMU_IOCTL_GET_REGULATOR_STATE          _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_REGULATOR_STATE, BCM_PMU_Regulator_t)
#define BCM_PMU_IOCTL_SET_REGULATOR_STATE          _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_REGULATOR_STATE, BCM_PMU_Regulator_t)
#define BCM_PMU_IOCTL_GET_VOLTAGE                  _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_VOLTAGE, BCM_PMU_Regulator_Volt_t)
#define BCM_PMU_IOCTL_SET_VOLTAGE                  _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_VOLTAGE, BCM_PMU_Regulator_Volt_t)
#define BCM_PMU_IOCTL_SET_PWM_LED_CTRL             _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LED_CTRL, BCM_PMU_PWM_ctrl_t)
#define BCM_PMU_IOCTL_SET_PWM_HI_PER               _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_HI_PER, BCM_PMU_PWM_hi_per_t)
#define BCM_PMU_IOCTL_SET_PWM_LO_PER               _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LO_PER, BCM_PMU_PWM_lo_per_t)
#define BCM_PMU_IOCTL_SET_PWM_PWR_CTRL             _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_PWR_CTRL, BCM_PMU_PWM_pwr_ctrl_t)
#define BCM_PMU_IOCTL_SET_CHARGING_CURRENT         _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_CHARGING_CURRENT, int)
#define BCM_PMU_IOCTL_GET_CHARGING_CURRENT         _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_CHARGING_CURRENT, int)
#define BCM_PMU_IOCTL_SET_VSR_CURRENT_LMT          _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_VSR_CURRENT_LMT, BCM_PMU_Charger_Current_t)
#define BCM_PMU_IOCTL_GET_VSR_CURRENT_LMT          _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_VSR_CURRENT_LMT, BCM_PMU_Charger_Current_t)
#define BCM_PMU_IOCTL_SET_CHARGER_WDT_CTRL         _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_CHARGER_WDT_CTRL, int)
#define BCM_PMU_IOCTL_SET_CHARGER_WDT_CLEAR        _IO(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_CHARGER_WDT_CLEAR)   // arg is unused
#define BCM_PMU_IOCTL_SET_CHARGER_FC_OPTION        _IO(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_CHARGER_FC_OPTION)   // arg is unused
#define BCM_PMU_IOCTL_CHECK_IF_STANDARD_HOST_PORT  _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_CHECK_IF_STANDARD_HOST_PORT, int)
#define BCM_PMU_IOCTL_SET_USB_CHARGER_TYPE         _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_USB_CHARGER_TYPE, BCM_PMU_USB_Charger_Type_t)
#define BCM_PMU_IOCTL_SET_ADC_CONT_MODE_SEL        _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_ADC_CONT_MODE_SEL, int)
#define BCM_PMU_IOCTL_SET_ADC_RESET_COUNT          _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_ADC_RESET_COUNT, int)
#define BCM_PMU_IOCTL_SET_ADC_ASYN_MODE_CTRL       _IO(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_ADC_ASYN_MODE_CTRL)   // arg is unused
#define BCM_PMU_IOCTL_SET_ADC_LATCH_DATA           _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_ADC_LATCH_DATA, BCM_PMU_adc_data_t)
#define BCM_PMU_IOCTL_GET_ADC_READ_DATA            _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_ADC_READ_DATA, BCM_PMU_adc_data_t)
#define BCM_PMU_IOCTL_SET_POWERMODE_PC_I2C_CTRL    _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_POWERMODE_PC_I2C_CTRL, BCM_PMU_pc1_pc2_t)
#define BCM_PMU_IOCTL_GET_OTG_ROLE                 _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_OTG_ROLE, BCM_PMU_OTG_Role_t)
#define BCM_PMU_IOCTL_GPIO_READ_DATA               _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_GPIO_READ_DATA, int)
#define BCM_PMU_IOCTL_GPIO_WRITE_DATA              _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_GPIO_WRITE_DATA, int)
#define BCM_PMU_IOCTL_GPIO_SET_DIRECTION           _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_GPIO_SET_DIRECTION, int)
#define BCM_PMU_IOCTL_GPIO_SET_MODE                _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_GPIO_SET_MODE, int)
#define BCM_PMU_IOCTL_SET_KEYLOCK                  _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PONKEY_KEYLOCK, int)
#define BCM_PMU_IOCTL_GET_RTC_TIME                 _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_RTC_TIME, BCM_PMU_RTC_time_t)
#define BCM_PMU_IOCTL_SET_RTC_TIME                 _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_RTC_TIME, BCM_PMU_RTC_time_t)
#define BCM_PMU_IOCTL_GET_RTC_ALARM_TIME           _IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_RTC_ALARM_TIME, BCM_PMU_RTC_time_t)
#define BCM_PMU_IOCTL_SET_RTC_ALARM_TIME           _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_RTC_ALARM_TIME, BCM_PMU_RTC_time_t)

#define BCM_PMU_IOCTL_GET_FG_SAMPLE                _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_FG_SAMPLE, BCM_PMU_FuelGauge_t)

typedef enum
{
    SIM_3POINT0VOLT = 0,
    SIM_2POINT5VOLT,
    SIM_3POINT1VOLT,
    SIM_1POINT8VOLT,
    SIM_MAX_VOLTAGE    
} PM_SimVoltage_t;

typedef struct
{
    unsigned char  reg;
    unsigned char  val;
} BCM_PMU_Reg_t;

typedef struct
{
    unsigned char  reg;
    unsigned char  num;
    unsigned char  val[10];
} BCM_PMU_Reg_bytes_t;

typedef struct
{
   int regulatorID;
   int voltage;
   int min;
   int max;
   int step;
} BCM_PMU_Regulator_Volt_t;

typedef struct
{
   int charger_type;
   int mA;
} BCM_PMU_Charger_Current_t;  

typedef enum
{
   VWALL = 0,
   VBUS,
   VSYS,
   VMBAT,
   VBBAT,
   VIDIN,
   ADC1,
   ADC2,
   ADC3,
   ADC4
} BCM_PMU_ADC_Channel_t;

typedef enum
{
   PMU_PCF50603 = 0,
   PMU_PCF50611,
   PMU_BCM59001,
   PMU_BCM59035,
   PMU_BCM59040,
   PMU_NONE,
   PMU_NUM_CHIPS,
} BCM_PMU_Chip_t;

typedef enum
{
   PMU_Regulator_Off,
   PMU_Regulator_On,
   PMU_Regulator_Eco
} BCM_PMU_Regulator_State_t;

typedef enum
{
   PMU_Charging_Host_Port,
   PMU_Dedicated_Charger,
   PMU_Standard_Host_Port
} BCM_PMU_USB_Charger_Type_t;

/* OTG related data structure */
typedef enum {
    OTG_A_DEV,          // otg A device
    OTG_B_DEV_LEGACY,    // otg B legacy device or no cable
    OTG_RID_A,          // otg RID A (has ACA)
    OTG_RID_B,          // otg RID B (has ACA)
    OTG_RID_C,          // otg RID C (has ACA)
    OTG_B_DEV_FLOAT     // otg B device (has ACA)
} BCM_PMU_OTG_Role_t;

typedef enum
{
   PMU_Power_Mode_Hibernate_Enable,
   PMU_Power_Mode_Hibernate_Disable,

} BCM_PMU_Power_Mode_Control_t;

typedef struct
{
   int regulatorID;
   BCM_PMU_Regulator_State_t state;
} BCM_PMU_Regulator_t;

typedef struct
{
   int mode;
   short fgsmpl;
} BCM_PMU_FuelGauge_t;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int pwmledOn;
    unsigned int pwmled_ctrl ;
    unsigned int pwmdiv ; // divider value. fsys/x value.    
} BCM_PMU_PWM_ctrl_t ;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int hi_per ;
} BCM_PMU_PWM_hi_per_t ;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int lo_per ;
} BCM_PMU_PWM_lo_per_t ;

typedef struct 
{
    unsigned int pwr_ctrl ;
} BCM_PMU_PWM_pwr_ctrl_t ;

typedef struct
{
   int pc1;
   int pc2;
} BCM_PMU_pc1_pc2_t;

typedef struct
{
   int channel;
   int data;
} BCM_PMU_adc_data_t;


typedef struct{
   unsigned char  Sec;
   unsigned char  Min;
   unsigned char  Hour;
   unsigned char  Week;
   unsigned char  Day;
   unsigned char  Month;
   unsigned char  Year;
} BCM_PMU_RTC_time_t;

typedef enum
{
   PMU_Power_On_By_On_Button,      // on button
   PMU_Power_On_By_Charger,        // charger insertion and no on button
   PMU_Power_On_By_Restart         // re-started while power on
} BCM_PMU_Power_On_State_t;

typedef enum                     // If you change this, please update PM_EventTable as well
{
   PMU_EVENT_ATTACHED = 0,

   PMU_EVENT_BATTERY_LOW,
   PMU_EVENT_BATTERY_FULL,
   PMU_EVENT_BATTERY_REMOVED,  // new for 59040
   PMU_EVENT_BATTERY_OVER_VOLT,  // new for 59040
   PMU_EVENT_BATTERY_INSERTED,  // new for 59040
   PMU_EVENT_BATTERY_VERY_LOW,  // new for 59040
   PMU_EVENT_ID_INSERT, // new for 59040
   PMU_EVENT_ID_REMOVE, // new for 59040                    
   PMU_EVENT_ID_CHANGE, // new for 59040                        
   PMU_EVENT_VBUS_VLD_F, // new for 59040
   PMU_EVENT_A_SESS_VLD_F, // new for 59040
   PMU_EVENT_B_SESS_END_F, // new for 59040
   PMU_EVENT_VBUS_VLS_R, // new for 59040
   PMU_EVENT_A_SESS_VLD_R, // new for 59040
   PMU_EVENT_B_SESS_END_R, // new for 59040
   PMU_EVENT_RESUME_VBUS_VLD, // new for 59040: vbus valid
   PMU_EVENT_RESUME_VBUS_ZERO, // new for 59040: vbus 0
   PMU_EVENT_RESUME_VBUS, // new for 59040: vbus transitioning

   PMU_EVENT_BATTERY_TEMPERATURE_FAULT,
   PMU_EVENT_BATTERY_TEMPERATURE_OK,

   PMU_EVENT_ONKEY_RISE,
   PMU_EVENT_ONKEY_FALL,
   PMU_EVENT_ONKEY_1S_HOLD,
   PMU_EVENT_ONKEY_STANDBY, // new for 59040
   PMU_EVENT_ONKEY_RESTART, // new for 59040
   PMU_EVENT_CHARGER_DETECTED, // new for 59040

   PMU_EVENT_VSR_OVER_V,     // new for 59040
   PMU_EVENT_VSR_OVER_I,     // new for 59040
   PMU_EVENT_CHARGER_NOT_OK, // new for 59040                            
   PMU_EVENT_CHG_WDT_ALARM, // new for 59040
   PMU_EVENT_VBUSLOWBND,     // new for 59040
   PMU_EVENT_CHARGER_ERR_GONE, // new for 59040
   PMU_EVENT_CHGWDEXP,       // new for 59040  
                          
   PMU_EVENT_SARCONVEND,     // new for 59040
   PMU_EVENT_SARCONTCONVFAIL,// new for 59040
   PMU_EVENT_SARASYNCONVOFF, // new for 59040
   PMU_EVENT_SARASYNREQFAIL, // new for 59040

   PMU_EVENT_PMUTOOWARM,  // new for 59040
   PMU_EVENT_MBTEMPFAULT, // new for 59040
   PMU_EVENT_MBTEMPLOW,   // new for 59040
   PMU_EVENT_MBTEMPHIGH,  // new for 59040
                            
   PMU_EVENT_HBINT, // new for 59040

   PMU_EVENT_HIGH_TEMPERATURE,  

   PMU_EVENT_CHARGER_INSERT,
   PMU_EVENT_CHARGER_REMOVE,
   PMU_EVENT_CHARGER_ERROR,

   PMU_EVENT_CHARGER_START,  // For USB Vbus resume
   PMU_EVENT_CHARGER_STOP,   // For USB Vbus suspend
   PMU_EVENT_CHARGER_SET_CURR_LMT,  // For USB set current limit after enumeration

   PMU_EVENT_LDO1OVRI,
   PMU_EVENT_LDO2OVRI,
   PMU_EVENT_LDO3OVRI,
   PMU_EVENT_LDO4OVRI,
   PMU_EVENT_LDO5OVRI,
   PMU_EVENT_LDO6OVRI,
   PMU_EVENT_IOSROVRI,
   PMU_EVENT_CSROVRI,
   PMU_EVENT_IOSROVRV,
   PMU_EVENT_CSROVRV,

   PMU_EVENT_RTC1S,
   PMU_EVENT_RTC60S,
   PMU_EVENT_RTCA1,

   PMU_NUM_EVENTS,
} BCM_PMU_Event_t;


typedef enum
{
   GPIO_HIGH_IMPEDANCE = 0,
   GPIO_OUTPUT,
   GPIO_INPUT
}BCM_PMU_gpio_dir_t;

typedef struct
{
   int gpioNum;
   BCM_PMU_gpio_dir_t	gpioDir;
}BCM_PMU_gpioDir_t;

typedef struct
{
   int gpioNum;
   int gpioData;
}BCM_PMU_gpioData_t;

typedef struct
{
   int gpioNum;
   int gpioMode;
}BCM_PMU_gpioMode_t;

typedef struct
{
   BCM_PMU_gpio_dir_t gpioDir;
   int	gpioData;
   int gpioMode;
}BCM_PMU_gpioInit_t;


/* ---- Variable Externs ------------------------------------------------- */

#ifdef __KERNEL__
extern u32 pmu_max_isr_clk;      /* ISR profiling counter */
#endif

/* ---- Function Prototypes ---------------------------------------------- */

#ifdef __KERNEL__

#if defined( CONFIG_BCM_PMU )

/* Initialization function */
typedef int (*pmu_init_fnc)(void);
#define PMU_init(pmu) ((pmu)->init?(pmu)->init():0)

/* Interrupt service routine */
typedef irqreturn_t (*pmu_isr_fnc)(void *dev_id);
#define PMU_isr(pmu,dev_id) ((pmu)->isr?(pmu)->isr(dev_id):IRQ_NONE)

/* Get power on condition */
typedef BCM_PMU_Power_On_State_t (*pmu_get_power_on_state_fnc)(void);
#define PMU_get_power_on_state(pmu) ((pmu)->get_power_on_state?(pmu)->get_power_on_state():PMU_Power_On_By_On_Button)

/* IOCTL handler */
typedef int (*pmu_ioctl_fnc) (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#define PMU_ioctl(pmu,inode,file,cmd,arg) ((pmu)->ioctl?(pmu)->ioctl(inode,file,cmd,arg):-EINVAL)
 
/* Power off function */
typedef void (*pmu_poweroff_fnc)(void);
#define PMU_poweroff(pmu) ((pmu)->poweroff?(pmu)->poweroff():0)

/* Platform specific timed run function */
typedef void (*pmu_run_fnc)(void);
#define PMU_run(pmu) ((pmu)->run?(pmu)->run():0)

/* Platform specific logLevel setting */
typedef void (*pmu_logLevel_fnc)(int level);
#define PMU_logLevel(pmu, level) ((pmu)->logLevel?(pmu)->logLevel(level):0)

/* Platform specific suspend handler */
typedef int (*pmu_suspend_fnc)(int mode);
#define PMU_suspend(pmu, mode) ((pmu)->suspend?(pmu)->suspend(mode):0)

/* Platform specific resume handler */
typedef int (*pmu_resume_fnc)(int mode);
#define PMU_resume(pmu, mode) ((pmu)->resume?(pmu)->resume(mode):0)

/* Power regulator control */
typedef int (*pmu_regulator_set_state_fnc)(int regulatorID, BCM_PMU_Regulator_State_t state);
#define PMU_regulator_set_state(pmu,regulatorID,state) \
   ((pmu)->regulator.set_state?(pmu)->regulator.set_state(regulatorID,state):0)

typedef BCM_PMU_Regulator_State_t (*pmu_regulator_get_state_fnc)(int regulatorID);
#define PMU_regulator_get_state(pmu,regulatorID) \
   ((pmu)->regulator.get_state?(pmu)->regulator.get_state(regulatorID):PMU_Regulator_On)

typedef int (*pmu_regulator_set_state_for_pm_fnc)(int regulatorID, int pmState, BCM_PMU_Regulator_State_t regulatorState);
#define PMU_regulator_set_state_for_pm(pmu,regulatorID,pmState,regulatorState) \
   ((pmu)->regulator.set_state_for_pm?(pmu)->regulator.set_state_for_pm(regulatorID,pmState,regulatorState):0)

typedef BCM_PMU_Regulator_State_t (*pmu_regulator_get_state_for_pm_fnc)(int regulatorID, int pmState);
#define PMU_regulator_get_state_for_pm(pmu,regulatorID,pmState) \
   ((pmu)->regulator.get_state_for_pm?(pmu)->regulator.get_state_for_pm(regulatorID,pmState):PMU_Regulator_On)

typedef int (*pmu_regulator_set_voltage_fnc)(int regulatorID, u32 mV);
#define PMU_regulator_set_voltage(pmu,regulatorID,mV) \
   ((pmu)->regulator.set_voltage?(pmu)->regulator.set_voltage(regulatorID,mV):0)

typedef u32 (*pmu_regulator_get_voltage_fnc)(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);
#define PMU_regulator_get_voltage(pmu,regulatorID,min_mV,max_mV,mV_step) \
   ((pmu)->regulator.get_voltage?(pmu)->regulator.get_voltage(regulatorID,min_mV,max_mV,mV_step):0)

/* Charger control */
typedef void (*pmu_charger_start_fnc)(int chargerID);
#define PMU_charger_start(pmu,chargerID) \
   ((pmu)->charger.start?(pmu)->charger.start(chargerID):0)

typedef void (*pmu_charger_stop_fnc)(int chargerID);
#define PMU_charger_stop(pmu,chargerID) \
   ((pmu)->charger.stop?(pmu)->charger.stop(chargerID):0)

typedef int (*pmu_charger_is_inserted_fnc)(int *chargerID);
#define PMU_charger_is_inserted(pmu,chargerID) \
   ((pmu)->charger.is_inserted?(pmu)->charger.is_inserted(chargerID):0)

typedef int (*pmu_charger_set_current_limit_fnc)(int charger_type, u32 mA);
#define PMU_charger_set_current_limit(pmu,charger_type,mA) \
   ((pmu)->charger.set_current_limit?(pmu)->charger.set_current_limit(charger_type,mA):0)

typedef int (*pmu_charger_get_current_fnc)(void);
#define PMU_charger_get_current(pmu) \
   ((pmu)->charger.get_current?(pmu)->charger.get_current():0)

/*Fuel gauge control*/
typedef int (*pmu_fuelgauge_get_sample_fnc)(short *fgsmpl, int mode);
#define PMU_fuelgauge_get_sample(pmu,fgsmpl,mode) \
   ((pmu)->fuelgauge.get_FG_sample?(pmu)->fuelgauge.get_FG_sample(fgsmpl,mode):0)

typedef void (*pmu_fuelgauge_enable_fnc)(int enable);
#define PMU_fuelgauge_enable(pmu,enable) \
   ((pmu)->fuelgauge.FG_enable?(pmu)->fuelgauge.FG_enable(enable):0)

typedef struct
{
   pmu_init_fnc init;
   pmu_isr_fnc isr;
   pmu_get_power_on_state_fnc get_power_on_state;
   pmu_ioctl_fnc ioctl;
   pmu_poweroff_fnc poweroff;
   pmu_run_fnc run;
   pmu_logLevel_fnc logLevel;
   pmu_suspend_fnc suspend;
   pmu_resume_fnc resume;
   struct
   {
      pmu_regulator_set_state_fnc set_state;
      pmu_regulator_get_state_fnc get_state;
      pmu_regulator_set_state_for_pm_fnc set_state_for_pm;
      pmu_regulator_get_state_for_pm_fnc get_state_for_pm;
      pmu_regulator_set_voltage_fnc set_voltage;
      pmu_regulator_get_voltage_fnc get_voltage;
   } regulator;
   struct
   {
      pmu_charger_start_fnc start;
      pmu_charger_stop_fnc stop;
      pmu_charger_is_inserted_fnc is_inserted;
      pmu_charger_set_current_limit_fnc set_current_limit;
      pmu_charger_get_current_fnc get_current;
   } charger;
   struct
   {
      pmu_fuelgauge_get_sample_fnc get_FG_sample;
      pmu_fuelgauge_enable_fnc FG_enable;
   } fuelgauge;
   struct i2c_client_address_data *i2c_data;
   char driver_name[ 20 ];
   struct i2c_driver driver;
} BCM_PMU_Operations_t;

// @KP: added for TOMTOM_OTAVALO
extern BCM_PMU_Operations_t *current_pmu;

int pmu_register_device(BCM_PMU_Chip_t chip, BCM_PMU_Operations_t *pmu_ops, void *pdata);

int pmu_i2c_read(u8 regAddr);
int pmu_i2c_write(u8 regAddr, u8 value);

int pmu_i2c_read_bytes(u8 regAddr, u8 *values, int num);
int pmu_i2c_write_bytes(u8 reg, u8 *values, u8 num);

void pmu_enable_ints( void );
void pmu_disable_ints( void );

void pmu_event_notify(BCM_PMU_Chip_t chip, BCM_PMU_Event_t event, void *data);

int pmu_chip_init(BCM_PMU_Chip_t chip);
void pmu_chip_exit( void );

void pmu_start_null_pmu( void );

#else

// Allow PMU calls to compile but not do anything

#define pmu_register_device(chip, pmu_ops, i2c_data) ((void)(chip), (void)(pmu_ops), (void)(i2c_data), -1)

#ifndef CONFIG_PMU_DEVICE_BCM59040
#define pmu_i2c_read(regAddr)        ((void)(regAddr), -1)
#define pmu_i2c_write(regAddr, val)  ((void)(regAddr), (void)(val), -1)
#else
int pmu_i2c_read(u8 regAddr);
int pmu_i2c_write(u8 regAddr, u8 value);

int pmu_i2c_read_bytes(u8 regAddr, u8 *values, int num);
int pmu_i2c_write_bytes(u8 reg, u8 *values, u8 num);
#endif

#define pmu_enable_ints() (-1)
#define pmu_disable_ints() (-1)
#define pmu_event_notify(chip, event, data) ((void)(chip), (void)(event), (void)(data), -1)
#define pmu_chip_init(chip) ((void)(chip), -1)
#define pmu_chip_exit() (-1)

#endif

#endif
#endif

int pmu_i2c_read(u8 regAddr);
int pmu_i2c_write(u8 regAddr, u8 value);

int pmu_i2c_read_bytes(u8 regAddr, u8 *values, int num);
int pmu_i2c_write_bytes(u8 reg, u8 *values, u8 num);
#endif  /* BCM_PMU_CHIP_H */
