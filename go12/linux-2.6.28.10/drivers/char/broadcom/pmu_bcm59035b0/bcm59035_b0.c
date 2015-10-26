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
*  bcm59035_b0.c
*
*  PURPOSE:
*
*     This implements the BCM59035 chip specific portion of the pmu driver.
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
#include <linux/sysctl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>

/* Include external interfaces */
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59035_b0.h>
#include <linux/broadcom/timer.h>

/* Include internal interfaces */
#include "bcm59035_b0.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

// INT2M Setting - handle EOC interrupts separately based on charger state
#define INT2M_DEFAULT 0

#define DISABLE_ALL_INTS 0xFF

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL	(DBG_ERROR)
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR|DBG_INFO|DBG_TRACE|DBG_TRACE2)

#ifdef DEBUG
   #define PMU_DEBUG(level,fmt,args...) do { if (level & logLevel) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#else
   #define PMU_DEBUG(level,fmt,args...)
#endif


/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "BCM59035 B0 Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

static long gIsrThreadPid = 0;
static struct completion gIsrExited;
static struct semaphore gIsrSem;

static int chargerEOC = 0;                // charger eoc detected flag
static int isWallChargerPresent = 0;         // wall charger present flag
static int isUSBChargerPresent = 0;          // usb charger present flag

static int logLevel = DBG_DEFAULT_LEVEL;

static bcm59035_isr_t bcm59035_IRQVectorTable[BCM59035_NUM_IRQ];

/* output voltage mapping table */
/* THE csr voltage output code currently assumes that csr is always in normal mode. Also, the functions to enable/disable dvs and change voltage are not re-entrant - for example, there could be problems if dvfs in disabled when voltage is being changed. It is the responsibility of higher layers to make sure this does not happen */
static u32 bcm59035_csr_dvs_vout_to_mV_map[] =
{
   1500,
   1480,
   1460,
   1440,
   1420,
   1400,
   1380,
   1360,
   1340,
   1320,
   1300,
   1280,
   1260,
   1240,
   1220,
   1200,
   1180,
   1160,
   1140,
   1120,
   1100,
   1080,
   1060,
   1040,
   1020,
   1000,
   980,
   960,
   940,
   920,
   900,
};

/* output voltage mapping table */
static u32 bcm59035_ax1_ldo_vout_to_mV_map[] =
{
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
   3300,
   3400,
   2400,
   2400,
   2400,
   2600,
   2600,
   2800,
   3000,
   3200,
   2800,
};

/* output voltage mapping table */
static u32 bcm59035_ax2_ldo_vout_to_mV_map[] =
{
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
   3300,
   3400,
   2400,
   2400,
   2400,
   2600,
   2600,
   2800,
   3000,
   3200,
   1800,
};


/* output voltage mapping table */
static u32 bcm59035_io_ldo_vout_to_mV_map[] =
{
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
   3300,
   3400,
   2400,
   2400,
   2400,
   2600,
   2800,
   2800,
   3000,
   3200,
   3000,
};

/* output voltage mapping table */
static u32 bcm59035_ms_ldo_vout_to_mV_map[] =
{
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
   3300,
   3400,
   2400,
   2400,
   2400,
   2600,
   2600,
   2800,
   3000,
   3200,
   2800,
};

/* output voltage mapping table */
static u32 bcm59035_lc_ldo_vout_to_mV_map[] =
{
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
   3300,
   3400,
   2400,
   2400,
   2400,
   2600,
   2600,
   2800,
   3000,
   3200,
   2700,
};



/* output voltage mapping table */
static u32 bcm59035_a_hc_ldo_vout_to_mV_map[] =
{
   2500,
   2600,
   2700,
   2800,
   2900,
   3200,
   3100,
   3000,
};

/* output voltage mapping table */
static u32 bcm59035_rfldo1_vout_to_mV_map[] =
{
   2500, /*000*/
   2600, /*001*/
   2800, /*010*/
   3200, /*011*/
   2900, /*100*/
   3000, /*101*/
   3100, /*110*/
   2700, /*111*/
};

/* output voltage mapping table */
static u32 bcm59035_rfldo2_vout_to_mV_map[] =
{
   3200, /*000*/
   2600, /*001*/
   2700, /*010*/
   2800, /*011*/
   2900, /*100*/
   3000, /*101*/
   3100, /*110*/
   2500, /*111*/
};

/* output voltage mapping table */
static u32 bcm59035_ms2_ldo_vout_to_mV_map[] =
{
   2800,
   2900,
   3000,
   3100,
   3200,
   3300,
   3400,
};

/* output voltage mapping table */
static u32 bcm59035_sim_ldo_vout_to_mV_map[] =
{
   3000,
   2500,
   3100,
   1800,
};

/* output voltage mapping table */
static u32 bcm59035_lv1_ldo_vout_to_mV_map[] =
{
   1200,
   1225,
   1250,
   1275,
   1300,
   1325,
   1350,
   1375,
   1400,
   0,
   0,
   0,
   0,
   0,
   0,
   1500,
};

/* output voltage mapping table */
static u32 bcm59035_lv2_ldo_vout_to_mV_map[] =
{
   1200,
   1225,
   1250,
   1275,
   1325,
   1350,
   1375,
   1400,
   1500,
   0,
   0,
   0,
   0,
   0,
   0,
   1300,
};

/* output voltage mapping table */
static u32 bcm59035_csr_nodvs_vout_to_mV_map[] =
{
   2500, /*00000*/
   2400,
   2300,
   2200,
   2100,
   2000,
   1900,
   1800,
   1700,
   1600,
   1500,
   1400,
   1300,
   1200,
   1100,
   1000,
   900,  /*10000*/
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,
   0,   /*11111*/
};

/* output voltage mapping table */
static u32 bcm59035_iosr_vout_to_mV_map[] =
{
   2500, /*00000*/
   2400,
   2300,
   2200,
   2100,
   2000,
   1900,
   1800,
   1700,
   1600,
   1500,
   1400,
   1300,
   1200,
   1100,
   1000,
   900,
};

static bcm59035_regulator_map_t bcm59035_regulator_map[BCM59035_NUM_REGULATORS] =                       
{                                                                                                       
   {                                                                                                    
      available:     1,                                                                                 
      programmable:  0,                                                                                 
      reg_addr:      BCM59035_REG_A1OPMODCTRL,     // BCM59035_REGULATOR_ALDO1                          
      reg_addr_volt: BCM59035_REG_ALDOCTRL,     // BCM59035_REGULATOR_ALDO1                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_a_hc_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_a_hc_ldo_vout_to_mV_map)/sizeof(bcm59035_a_hc_ldo_vout_to_mV_map[0])),
   },                                                                                                  
   {                                                                                                   
      available:     1,                                                                                
      programmable:  0,                                                                                
      reg_addr:      BCM59035_REG_A2OPMODCTRL,     // BCM59035_REGULATOR_ALDO2                         
      reg_addr_volt: BCM59035_REG_ALDOCTRL,     // BCM59035_REGULATOR_ALDO2                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    4,
      vout_to_mV_map: bcm59035_a_hc_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_a_hc_ldo_vout_to_mV_map)/sizeof(bcm59035_a_hc_ldo_vout_to_mV_map[0])),
   },                                                                                                  
   {                                                                                                   
      available:     1,
      programmable:  0,
      reg_addr:      BCM59035_REG_R1OPMODCTRL,     // BCM59035_REGULATOR_RFLDO1
      reg_addr_volt: BCM59035_REG_RFDOCTRL,     // BCM59035_REGULATOR_RFLDO1                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_rfldo1_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_rfldo1_vout_to_mV_map)/sizeof(bcm59035_rfldo1_vout_to_mV_map[0])),
   },
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59035_REG_R2OPMODCTRL,     // BCM59035_REGULATOR_RFLDO2
      reg_addr_volt: BCM59035_REG_RFDOCTRL,     // BCM59035_REGULATOR_RFLDO2                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    4,
      vout_to_mV_map: bcm59035_rfldo2_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_rfldo2_vout_to_mV_map)/sizeof(bcm59035_rfldo2_vout_to_mV_map[0])),
   },
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59035_REG_H1OPMODCTRL,      // BCM59035_REGULATOR_HCLDO1
      reg_addr_volt: BCM59035_REG_HCLDOCTRL,     // BCM59035_REGULATOR_HCLDO1                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_a_hc_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_a_hc_ldo_vout_to_mV_map)/sizeof(bcm59035_a_hc_ldo_vout_to_mV_map[0])),
   },   
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59035_REG_H2OPMODCTRL,      // BCM59035_REGULATOR_HCLDO2
      reg_addr_volt: BCM59035_REG_HCLDOCTRL,     // BCM59035_REGULATOR_HCLDO2                          
      min_mV:        2500,
      max_mV:        3200,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    4,
      vout_to_mV_map: bcm59035_a_hc_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_a_hc_ldo_vout_to_mV_map)/sizeof(bcm59035_a_hc_ldo_vout_to_mV_map[0])),
   },   
   {                                                                                                      
      available:     1,                                                                                   
      programmable:  0,                                                                                   
      reg_addr:      BCM59035_REG_IOPMODCTRL,      // BCM59035_REGULATOR_IOLDO                            
      reg_addr_volt: BCM59035_REG_IOLDOCTRL,     // BCM59035_REGULATOR_IOLDO                          
      min_mV:        1200,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_io_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_io_ldo_vout_to_mV_map)/sizeof(bcm59035_io_ldo_vout_to_mV_map[0])),
   },                                                                                                     
   {                                                                                                      
      available:     1,                                                                                   
      programmable:  0,                                                                                   
      reg_addr:      BCM59035_REG_M1OPMODCTRL,      // BCM59035_REGULATOR_MSLDO                           
      reg_addr_volt: BCM59035_REG_MSLDOCTRL,     // BCM59035_REGULATOR_MSLDO                          
      min_mV:        1200,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_ms_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_ms_ldo_vout_to_mV_map)/sizeof(bcm59035_ms_ldo_vout_to_mV_map[0])),
   },                                                                                                     
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_LOPMODCTRL,      // BCM59035_REGULATOR_LCLDO
      reg_addr_volt: BCM59035_REG_LCSIMDOCTRL,     // BCM59035_REGULATOR_LCLDO                          
      min_mV:        1200,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_lc_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_lc_ldo_vout_to_mV_map)/sizeof(bcm59035_lc_ldo_vout_to_mV_map[0])),
   },   
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_LV1OPMODCTRL,      // BCM59035_REGULATOR_LV1LDO                          
      reg_addr_volt: BCM59035_REG_LVLDOCTRL,     // BCM59035_REGULATOR_LV1LDO                          
      min_mV:        1200,
      max_mV:        1500,
      mV_step:       25,
      vout_mask:     0xF,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_lv1_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_lv1_ldo_vout_to_mV_map)/sizeof(bcm59035_lv1_ldo_vout_to_mV_map[0])),
   },                                                                                                    
   {
      available:     1,
      programmable:  0,
      reg_addr:      BCM59035_REG_SOPMODCTRL,      // BCM59035_REGULATOR_SIMLDO
      reg_addr_volt: BCM59035_REG_LCSIMDOCTRL,     // BCM59035_REGULATOR_SIMLDO                          
      min_mV:        1800,
      max_mV:        3100,
      mV_step:       -1,
      vout_mask:     0x3,
      vout_shift:    6,
      vout_to_mV_map: bcm59035_sim_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_sim_ldo_vout_to_mV_map)/sizeof(bcm59035_sim_ldo_vout_to_mV_map[0])),
   },   
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_LV2OPMODCTRL,      // BCM59035_REGULATOR_LV2LDO                          
      reg_addr_volt: BCM59035_REG_LVLDOCTRL,     // BCM59035_REGULATOR_LV2LDO                          
      min_mV:        1200,
      max_mV:        1500,
      mV_step:       25,
      vout_mask:     0xF,
      vout_shift:    4,
      vout_to_mV_map: bcm59035_lv2_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_lv2_ldo_vout_to_mV_map)/sizeof(bcm59035_lv2_ldo_vout_to_mV_map[0])),
   },                                                                                                    
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_M2OPMODCTRL,      // BCM59035_REGULATOR_MS2LDO                          
      reg_addr_volt: BCM59035_REG_MSLDOCTRL,     // BCM59035_REGULATOR_MSLDO2                         
      min_mV:        2800,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x7,
      vout_shift:    5,
      vout_to_mV_map: bcm59035_ms2_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_ms2_ldo_vout_to_mV_map)/sizeof(bcm59035_ms2_ldo_vout_to_mV_map[0])),
   },                                                                                                    
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_AX1OPMODCTRL,      // BCM59035_REGULATOR_AX1LDO                          
      reg_addr_volt: BCM59035_REG_AX1LDOCTRL,     // BCM59035_REGULATOR_AX1LDO                         
      min_mV:        1200,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_ax1_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_ax1_ldo_vout_to_mV_map)/sizeof(bcm59035_ax1_ldo_vout_to_mV_map[0])),
   },                                                                                                    
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_AX2OPMODCTRL,      // BCM59035_REGULATOR_AX2LDO                          
      reg_addr_volt: BCM59035_REG_AX2LDOCTRL,     // BCM59035_REGULATOR_AX2LDO                         
      min_mV:        1200,
      max_mV:        3400,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_ax2_ldo_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_ax2_ldo_vout_to_mV_map)/sizeof(bcm59035_ax2_ldo_vout_to_mV_map[0])),
   },
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  1,                                                                                  
      reg_addr:      BCM59035_REG_CSROPMODCTRL,   // CSR Mode control
      reg_addr_volt: BCM59035_REG_CSRCTRL10,      // CSR Voltage control (in normal mode with dvs enabled) 
      min_mV:        900,
      max_mV:        1500,
      mV_step:       20,
      vout_mask:     0x1F,
      vout_shift:    0,
      vout_to_mV_map: bcm59035_csr_dvs_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_csr_dvs_vout_to_mV_map)/sizeof(bcm59035_csr_dvs_vout_to_mV_map[0])),
   },
   {                                                                                                     
      available:     1,                                                                                  
      programmable:  0,                                                                                  
      reg_addr:      BCM59035_REG_IOSROPMODCTRL,  // BCM59035_REGULATOR_IOSR
      reg_addr_volt: BCM59035_REG_IOSRCTRL2,     // BCM59035_REGULATOR_IOLDO                         
      min_mV:        900,
      max_mV:        2500,
      mV_step:       100,
      vout_mask:     0x1F,
      vout_shift:    2,
      vout_to_mV_map: bcm59035_iosr_vout_to_mV_map,
      map_size:      (sizeof(bcm59035_iosr_vout_to_mV_map)/sizeof(bcm59035_iosr_vout_to_mV_map[0])),
   },
    

};

static BCM_PMU_Power_On_State_t gPowerOnState = PMU_Power_On_By_On_Button;

    
/* ---- Private Function Prototypes -------------------------------------- */
static int bcm59035_isr_thread(void *data);
static int bcm59035_process_interrupt( void );

/* Module functions */
static int bcm59035_module_init( void );
static void bcm59035_module_exit( void );

/* Common initialization routine */
static int bcm59035_init( void );

/* Interrupt handling functions */
static int bcm59035_irq_init(u8 *initial_int_status);

/* Interrupt service routine */
static irqreturn_t bcm59035_isr( void *dev_id );

/* Get power on condition */
static BCM_PMU_Power_On_State_t bcm59035_get_power_on_state( void );

/* IOCTL handler */
static int bcm59035_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/* Power off function */
static void bcm59035_poweroff( void );

/* Custom timed run function */
static void bcm59035_run( void );

/* Set log level for pmu debugging */
static void bcm59035_logLevel( int level );

/* Power regulator control */
static int bcm59035_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state);
static BCM_PMU_Regulator_State_t bcm59035_regulator_get_state(int regulatorID);

static int bcm59035_regulator_set_voltage(int regulatorID, u32 mV);
static u32 bcm59035_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);

static int bcm59035_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod);
static int bcm59035_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state);

static int bcm59035_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step);
static int bcm59035_vout_to_mV(int regulatorID, u8 vout, u32 *mV);

/* Charger control */
static void bcm59035_charger_start(int chargerID);
static void bcm59035_charger_stop(int chargerID);
static int bcm59035_charger_is_inserted(int *chargerID);

/* Event dispatcher */
static void bcm59035_event_notify(BCM59035_InterruptId_t irq_id);

//SIM related APIs
static int bcm59035_deactivate_sim(void);
static int bcm59035_activate_sim(BCM59035_sim_voltage_t sim_voltage);

/* DVS APIs for Core switching regulator */
int bcm59035_is_dvs_enabled(void);
void bcm59035_enable_dvs(void);
void bcm59035_disable_dvs(void);

/* I2C client address definitions */
static unsigned short normal_i2c[] = {BCM59035_I2C_BASE_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]     = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]    = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data bcm59035_i2c_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

/* PMU device operations */
static BCM_PMU_Operations_t bcm59035_ops =
{
   init: bcm59035_init,
   isr: bcm59035_isr,
   get_power_on_state: bcm59035_get_power_on_state,
   ioctl: bcm59035_ioctl,
   poweroff: bcm59035_poweroff,
   run: bcm59035_run,
   logLevel: bcm59035_logLevel,
   regulator:
   {
      set_state:     bcm59035_regulator_set_state,
      get_state:     bcm59035_regulator_get_state,
      set_voltage:   bcm59035_regulator_set_voltage,
      get_voltage:   bcm59035_regulator_get_voltage,
   },
   charger:
   {
      start: bcm59035_charger_start,
      stop: bcm59035_charger_stop,
      is_inserted: bcm59035_charger_is_inserted,
   },

   i2c_data: &bcm59035_i2c_data,
};

static int bcm59035_set_pwm_hiper(BCM_PMU_PWM_hi_per_t hiper) ;
static int bcm59035_set_pwm_loper(BCM_PMU_PWM_lo_per_t loper) ;
static int bcm59035_set_pwm_pwr_ctrl(BCM_PMU_PWM_pwr_ctrl_t pwrctrl) ;
static int bcm59035_set_pwm_ctrl(BCM_PMU_PWM_ctrl_t pwmctrl) ;

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
*  bcm59035_init
*
***************************************************************************/
static int bcm59035_init( void )
{
   int rc = 0;
   int version = 0xFF;
   u8 int_status[BCM59035_NUM_INT_REG];
#ifndef CONFIG_BCM_BATTERY_MANAGER
   BCM59035_chargers_t charger;
#endif

   PMU_DEBUG(DBG_TRACE, "\n");

   /* Initialize IRQ handler */
   bcm59035_irq_init(int_status);

   /* Register IRQ handler */
   bcm59035_irq_register(BCM59035_IRQID_INT2_CHGINS,    bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_CHGRM,     bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_CHGERR,    bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_CHGEOC,    bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_USBINS,    bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_USBRM,     bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_USBERR,    bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT2_MBCCHGERR, bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT1_PONKEYR,   bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT1_PONKEYF,   bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT1_PONKEYH,   bcm59035_event_notify);
   bcm59035_irq_register(BCM59035_IRQID_INT3_LOWBAT,    bcm59035_event_notify);

         
   version = pmu_read(BCM59035_REG_PMUID);
   if (version == -1)
   {
      PMU_DEBUG(DBG_ERROR, "Error reading Chip Version\n");
   }
   else
   {
      printk("BCM59035: Chip Version [0x%x]\n", version);
   }
   /* Disable USB charging timer */
   rc = pmu_read( BCM59035_REG_MBCCTRL2 );
   rc |= BCM59035_MBCCTRL2_USBTIMER_DIS | BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_DIS;
   rc |= pmu_write ( BCM59035_REG_MBCCTRL2, rc); 

   /* For USB, set TC2 to 3.6V and RC2 to 4.2V */
   rc |= pmu_write( BCM59035_REG_MBCCTRL5, (BCM59035_MBCCTRL5_USB_TC2_3_60V | BCM59035_MBCCTRL5_USB_RC2_4_2V) );
   /* Set charging current for USB to 950ma */
   rc |= pmu_write( BCM59035_REG_MBCCTRL6, BCM59035_MBCCTRL6_USB_RC_950MA );

   /* For USB, set TC2 to 3.2V and RC2 to 4.2V */
   rc |= pmu_write( BCM59035_REG_MBCCTRL3, (BCM59035_MBCCTRL3_WAC_RC2_4_2V | BCM59035_MBCCTRL3_WAC_TC2_3_2V)  );
   /* Set charging current for wall to 950ma */
   rc |= pmu_write( BCM59035_REG_MBCCTRL4, (BCM59035_MBCCTRL4_WAC_RC_950MA | (1<<5) | (1<<6)));

   /* Set EOC current */
   rc |= pmu_write( BCM59035_REG_MBCCTRL7, (BCM59035_MBCCTRL7_OV_DISABLE | BCM59035_MBCCTRL7_200MA) );
   rc |= pmu_write( BCM59035_REG_MBCCTRL11,(BCM59035_MBCCTRL11_WACTC1_100MA | BCM59035_MBCCTRL11_USBTC1_950MA) );
   rc |= pmu_write( BCM59035_REG_MBCCTRL1, (BCM59035_MBCCTRL1_WACTIMER_DIS | BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_DIS) );
   rc |= pmu_write( BCM59035_REG_LOWBATCVS, BCM59035_LOWBATCVS_LOWBATCVS_3_6V );

   /* Enable HCLDO1 and HCLDO2 during bootup */
   rc |= pmu_write( BCM59035_REG_H1OPMODCTRL, 0x0);
   rc |= pmu_write( BCM59035_REG_H2OPMODCTRL, 0x0);

   /* Modify dvs voltage register before enabling dvs so that there is no voltage change */
   rc |= pmu_write( BCM59035_REG_CSRCTRL10, 0xF); 
   /* Enable dvs during bootup */
   bcm59035_enable_dvs();

   /* If Battery manager is not present, initialize charger settings */
#ifndef CONFIG_BCM_BATTERY_MANAGER
   rc = pmu_read(BCM59035_REG_ENV1);
   if ( rc != -1 )
   {
      if( (u8)rc & (1<<1) )
      {
         PMU_DEBUG(DBG_TRACE, "wall charger detected\n");
         isWallChargerPresent = 1;
         charger = BCM59035_CHARGER_MAIN;
         bcm59035_charger_start(charger);
      }
      if ((u8)rc & (1<<2) )
      {
         PMU_DEBUG(DBG_TRACE, "USB charger detected\n");
         isUSBChargerPresent = 1;
         charger = BCM59035_CHARGER_USB;
         bcm59035_charger_start(charger);
      }
   }
#endif

#if 0
   //Set SimVoltage - TESTING
   rc = bcm59035_activate_sim(ESIM_3Point0Volt);
   if (rc != 0)
   {
      printk("BCM59035: Failed to activate SIM ret[0x%x]\n", rc);
   }
#endif

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
   readval = pmu_read(BCM59035_REG_INT2M);
   if (readval == -1)
   {
      PMU_DEBUG(DBG_ERROR, "error reading INT2M\n");
      return;
   }
   writeval = readval;
   if (isWallChargerPresent || isUSBChargerPresent)
   {
      writeval &= (~BCM59035_INT2_EOC);
   }
   else
   {
      writeval |= BCM59035_INT2_EOC;
   }

   if (readval != writeval) 
   {
      int rc = pmu_write( BCM59035_REG_INT2M, writeval);
      if (rc != 0) 
      {
         PMU_DEBUG(DBG_ERROR, "error writing to INT2M register\n");
      }
   }
}

/****************************************************************************
*
*  bcm59035_irq_init
*
***************************************************************************/
static int bcm59035_irq_init(u8 *initial_int_status)
{
   int rc;
   int i;
   int writeval;
   u8 int_status[BCM59035_NUM_INT_REG];
   u8 reg_addr;

   /* get initial interrupt bits */
   for ( i = 0; i < BCM59035_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59035_REG_INT1 + i);
      if (rc == -1)
      {
         PMU_DEBUG(DBG_ERROR, "error reading interrupt registers - Init \n");
         return -EINVAL;
      }
      else
      {
         int_status[i] = (u8)rc;
         PMU_DEBUG(DBG_TRACE2,"Reg 0x%02x = 0x%02x\n", BCM59035_REG_INT1 + i, rc);
      }
   }

   for (reg_addr = BCM59035_REG_INT1M; reg_addr <= BCM59035_REG_INT10M; reg_addr++)
   {
      rc = pmu_write(reg_addr, DISABLE_ALL_INTS);
      if( rc != 0 )
      {
         PMU_DEBUG(DBG_ERROR, "error writing interrupt register [0x%x]\n", reg_addr);
         //return -EINVAL;
      }
   }

   /* Fill IRQ function table with empty functions and
    * build the table that has bit positions for interrupts based on priority
    */
   for ( i = 0; i < BCM59035_NUM_IRQ; i++ )
   {
      bcm59035_IRQVectorTable[i] = NULL;
   }

   /* save initial int status */
   if (initial_int_status)
   {
      for (i = 0; i < BCM59035_NUM_INT_REG; i++)
      {
         initial_int_status[i] = int_status[i];
      }
   }

   writeval = 0xFF;
   writeval &= ~(BCM59035_INT2_CHGINS | BCM59035_INT2_CHGRM | BCM59035_INT2_CHGERR | BCM59035_INT2_USBINS | BCM59035_INT2_USBRM | BCM59035_INT2_USBERR | BCM59035_INT2_MBCCHGERR); 
   rc = pmu_write( BCM59035_REG_INT2M, writeval);

   /* Create ISR thread */
   sema_init(&gIsrSem, 0);
   init_completion(&gIsrExited);
   
   gIsrThreadPid = kernel_thread(bcm59035_isr_thread, (void *)PMU_BCM59035, 0);
   PMU_DEBUG(DBG_TRACE, "isr_thread started %lu\n", gIsrThreadPid);
      
   return 0;
}


/****************************************************************************
*
*  bcm59035_irq_register
*
*  irqId: ID of the IRQ to be registered
*  isrFunction: function to run for the particular IRQ
*
***************************************************************************/
int bcm59035_irq_register(BCM59035_InterruptId_t irqId, bcm59035_isr_t isrFunction)
{
   if (irqId >= BCM59035_NUM_IRQ)
   {
      PMU_DEBUG(DBG_ERROR, "irqId %d out of range\n", irqId);
      return -EINVAL;     
   }
   bcm59035_IRQVectorTable[irqId] = isrFunction;
   
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
*  bcm59035_isr_thread
*
***************************************************************************/
static int bcm59035_isr_thread(void *data)
{
   int rc;

   /* This thread doesn't need any user-level access,
    * so get rid of all our resources
    */
   (void)data;
   daemonize("bcm59035_isr");
   PMU_DEBUG(DBG_TRACE, "\n");

   /* Adjust priority to be higher than any user mode threads but
    * lower than any network threads */
   adjustThreadPriority(1);

   while(1)
   {
      if ( down_interruptible (&gIsrSem) == 0 )
      {
         rc = bcm59035_process_interrupt();
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

} /* bcm59035_isr_thread */

/****************************************************************************
*
*  bcm59035_process_interrupt
*
***************************************************************************/
static int bcm59035_process_interrupt( void )
{
   int rc;
   u8 intBits[BCM59035_NUM_INT_REG];
   u8 maskBits[BCM59035_NUM_INT_REG];
   u8 intStatus;
   u8 intMask[8];
   int i, k;
   u32 clk = timer_get_tick_count();

   /* read the interrupt status registers */
   for ( i = 0; i < BCM59035_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59035_REG_INT1 + i);
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
   for ( i = 0; i < BCM59035_NUM_INT_REG; i++ )
   {
      rc = pmu_read(BCM59035_REG_INT1M + i);
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

   for( k =0; k < BCM59035_NUM_INT_REG; k++ )
   {
      intStatus = (u8)(intBits[k] & (~maskBits[k]));
      for( i=0; i < 8; i++ )
      {
         if( intStatus & intMask[i] )
         {
            if( bcm59035_IRQVectorTable[ (k<<3)+i ] != NULL )
            {
               (*bcm59035_IRQVectorTable[ (k<<3)+i ]) ((k<<3)+i);
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
*  bcm59035_isr
*
***************************************************************************/
static irqreturn_t bcm59035_isr( void *dev_id )
{
   (void)dev_id;

   up( &gIsrSem );
   return IRQ_HANDLED;
} /* bcm59035_isr */


/****************************************************************************
*
*  bcm59035_get_power_on_state
*
***************************************************************************/
static BCM_PMU_Power_On_State_t bcm59035_get_power_on_state( void )
{
   return gPowerOnState;
}


/****************************************************************************
*
*  bcm59035_ioctl
*
***************************************************************************/
static int bcm59035_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   int rc = 0;

   PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

   switch ( cmd )
   {
      case BCM_PMU_IOCTL_ACTIVATESIM:
      {
         PM_SimVoltage_t voltage;

         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: ACTIVATE_SIM\n");

         if ( copy_from_user( &voltage, (PM_SimVoltage_t *)arg, sizeof( voltage )) != 0 )
         {
             return -EFAULT;
         }

         rc = bcm59035_activate_sim((int)voltage);         
      }
      break;

      case BCM_PMU_IOCTL_DEACTIVATESIM:
      {
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: DEACTIVATE_SIM\n");
         rc = bcm59035_deactivate_sim();
      }
      break;

      case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
      {
         BCM_PMU_Regulator_t regulator;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_REGULATOR_STATE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_regulator_set_state(regulator.regulatorID,regulator.state);
      }
      break;
      
      case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
      {
         BCM_PMU_Regulator_t regulator;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_GET_REGULATOR_STATE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         regulator.state = bcm59035_regulator_get_state(regulator.regulatorID);
         if ( copy_to_user( (BCM_PMU_Regulator_t *)arg, &regulator, sizeof( regulator )) != 0 )
         {
            return -EFAULT;
         }

         rc = 0;
      }
      break;
      
      case BCM_PMU_IOCTL_GET_VOLTAGE:
      {
         BCM_PMU_Regulator_Volt_t reg;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_GET_VOLTAGE\n");

         if ( copy_from_user( &reg, (BCM_PMU_Regulator_Volt_t *)arg, sizeof( reg )) != 0 )
         {
             return -EFAULT;
         }
         reg.voltage = bcm59035_regulator_get_voltage(reg.regulatorID,&reg.min,
						&reg.max,&reg.step);
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
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_VOLTAGE\n");

         if ( copy_from_user( &regulator, (BCM_PMU_Regulator_Volt_t *)arg, sizeof( regulator )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_regulator_set_voltage(regulator.regulatorID,regulator.voltage);
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
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_PWM_LED_CTRL \n");

         if ( copy_from_user( &pwmctrl, (BCM_PMU_PWM_ctrl_t *)arg, sizeof( pwmctrl )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_set_pwm_ctrl(pwmctrl);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_HI_PER :
      {
         BCM_PMU_PWM_hi_per_t hiper ;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_PWM_HI_PER \n");

         if ( copy_from_user( &hiper, (BCM_PMU_PWM_hi_per_t *)arg, sizeof( hiper )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_set_pwm_hiper(hiper);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_LO_PER :
      {
         BCM_PMU_PWM_lo_per_t loper ;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_PWM_LO_PER \n");

         if ( copy_from_user( &loper, (BCM_PMU_PWM_lo_per_t *)arg, sizeof( loper )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_set_pwm_loper(loper);
      }
      break;

      case BCM_PMU_IOCTL_SET_PWM_PWR_CTRL :
      {
         BCM_PMU_PWM_pwr_ctrl_t pwrctrl ;
         PMU_DEBUG(DBG_TRACE, "bcm59035_ioctl: BCM_PMU_IOCTL_SET_PWM_PWR_CTRL \n");

         if ( copy_from_user( &pwrctrl, (BCM_PMU_PWM_pwr_ctrl_t *)arg, sizeof( pwrctrl )) != 0 )
         {
             return -EFAULT;
         }
         rc = bcm59035_set_pwm_pwr_ctrl(pwrctrl);
      }
      break;

      default:
      {
          PMU_DEBUG(DBG_ERROR, "bcm59035_ioctl: UNSUPPORTED CMD\n");
          rc = -ENOTTY;
      }
      break;
   }

   return rc;

} /* bcm59035_ioctl */

/****************************************************************************
*
*  bcm59035_poweroff
*
***************************************************************************/
static void bcm59035_poweroff( void )
{
   int rc = 0;
   uint8_t hostact;

   rc = pmu_read( BCM59035_REG_HOSTACT );
   if( rc != -1 )
   {
      hostact = (uint8_t)rc;
      hostact |= BCM59035_HOSTACT_HOSTDICOFF;
      rc = pmu_write(BCM59035_REG_HOSTACT, hostact );
   }
}

/****************************************************************************
*
*  bcm59035_run - platform specific run function, called from battmgr.
*  We are using the fuel gauge for demonstration purposes only. This code does
*  NOT account for charge cycle history, battery condition, temperature, learning
*  cycles, aging, voltage measurement for replacement batteries, etc.
*
***************************************************************************/
static void bcm59035_run( void )
{
    return;
}

/****************************************************************************
*
*  bcm59035_logLevel
*
***************************************************************************/
static void bcm59035_logLevel( int level )
{
   logLevel = level;
}

/****************************************************************************
*
*  bcm59035_set_pwm_hiper
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59035_set_pwm_hiper(BCM_PMU_PWM_hi_per_t hiper)
{
   // u8 val = 0 ;
   int rc;

   if ( hiper.hi_per > 0x3f ) 
   {
      PMU_DEBUG(DBG_ERROR, "High per out of range %d-0x%x, given val = 0x%x\n", 0, 0x3f, hiper.hi_per);
      return -EINVAL;
   }

   // set high period value.
   // val = hiper.hi_per ;

   // update register
   rc = pmu_write(BCM59035_REG_PWMLEDCTRL3, (u8)(hiper.hi_per));
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing pwmctrl register.\n");
      return rc;
   }

   return 0;
} /* bcm59035_set_pwm_hiper */

/****************************************************************************
*
*  bcm59035_set_pwm_loper
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59035_set_pwm_loper(BCM_PMU_PWM_lo_per_t loper)
{
   // u8 val = 0 ;
   int rc;

   if ( loper.lo_per > 0x3f ) 
   {
      PMU_DEBUG(DBG_ERROR, "High per out of range %d-0x%x, given val = 0x%x\n", 0, 0x3f, loper.lo_per);
      return -EINVAL;
   }

   // set low period value.
   // val = loper.lo_per ;

   // update register
   rc = pmu_write(BCM59035_REG_PWMLEDCTRL4, (u8)(loper.lo_per));
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing pwmctrl register.\n");
      return rc;
   }

   return 0;
} /* bcm59035_set_pwm_loper */

/****************************************************************************
*
*  bcm59035_set_pwm_pwr_ctrl
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59035_set_pwm_pwr_ctrl(BCM_PMU_PWM_pwr_ctrl_t pwrctrl)
{
   u8 val = 0 ;
   int rc;

   if ( pwrctrl.pwr_ctrl > 1 ) 
   {
      PMU_DEBUG(DBG_ERROR, "Power contrl value can be either 0 or 1, given val = 0x%x\n", pwrctrl.pwr_ctrl);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_read(BCM59035_REG_PWMLEDCTRL5);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return rc;
   }

   if ( pwrctrl.pwr_ctrl == 0 ) 
   {
      val = rc & (~(BCM59035_PWMLED_PDN)) ; // Disable bit 6, set to 0.
   }
   else
   {
      val = rc | (BCM59035_PWMLED_PDN) ; // Enable bit 6, set to 1.
   }

   // update register
   rc = pmu_write(BCM59035_REG_PWMLEDCTRL5, val);
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing pwmctrl register.\n");
      return rc;
   }

   return 0;
} /* bcm59035_set_pwm_pwr_ctrl */


/****************************************************************************
*
*  bcm59035_set_pwm_ctrl
*
*  Set the pwm/led controls
*
***************************************************************************/
static int bcm59035_set_pwm_ctrl(BCM_PMU_PWM_ctrl_t pwmctrl)
{
   u8 val = 0 ;
   int rc;


   if ( ( pwmctrl.pwmled_ctrl > BCM59035_LEDON_MASK ) || ( pwmctrl.pwmdiv > BCM59035_LEDON_MASK ) )
   {
      PMU_DEBUG(DBG_ERROR, "pwmled ctrl or pwmdiv is out of range. ctrl = %d, div = %d \n", pwmctrl.pwmled_ctrl, pwmctrl.pwmdiv );
      return -EINVAL;
   }

   // set divider and pwmled control.
   val = ( ( val | ( pwmctrl.pwmdiv << 2 ) ) | ( pwmctrl.pwmled_ctrl ) ); 

   // update register
   rc = pmu_write(BCM59035_REG_PWMLEDCTRL1, val );
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing pwmctrl register.\n");
      return rc;
   }

   return 0;
} /* bcm59035_set_pwm_ctrl */

/****************************************************************************
*
*  bcm59035_regulator_get_state
*
*  Set the state of a regulator
*
***************************************************************************/
static int bcm59035_regulator_set_state(int regulatorID, BCM_PMU_Regulator_State_t state)
{
   int rc;
   u8 val;
   u8 opmod;

   if (!bcm59035_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   // convert state
   rc = bcm59035_state_to_opmod(regulatorID, state, &opmod);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state %d.\n", state);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_read(bcm59035_regulator_map[regulatorID].reg_addr);
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
      rc = pmu_write(bcm59035_regulator_map[regulatorID].reg_addr, val);
   }
   
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error writing regulator control register.\n");
      return rc;
   }

   return 0;
} /* bcm59035_regulator_set_state */

/****************************************************************************
*
*  bcm59035_mV_to_vout
*
***************************************************************************/
int bcm59035_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step)
{
   u32 *vout_to_mV_map;
   int map_size;
   int i;

   if (!vout)
      return -1;

   /* Validate input mV */
   if ((mV < min_mV) || (mV > max_mV))
   {
      printk(KERN_ERR "bcm59035: invalid %d mV setting for regulator %d.\n", mV, regulatorID);
      return -1;
   }
   if (mV_step != -1)
   {
      if ((mV - min_mV) % mV_step)
      {
         printk(KERN_ERR "bcm59035: invalid %d mV setting for regulator %d.\n", mV, regulatorID);
         return -1;
      }
   }

   if (bcm59035_regulator_map[regulatorID].vout_to_mV_map)
   {
      vout_to_mV_map = bcm59035_regulator_map[regulatorID].vout_to_mV_map;
      map_size = bcm59035_regulator_map[regulatorID].map_size;
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

   printk("bcm59035: corrupt mapping table.\n");
   return -1;
}

/****************************************************************************
*
*  bcm59035_vout_to_mV
*
***************************************************************************/
int bcm59035_vout_to_mV(int regulatorID, u8 vout, u32 *mV)
{
   u32 *vout_to_mV_map;
   int map_size;

   if (!mV)
      return -1;

   if (bcm59035_regulator_map[regulatorID].vout_to_mV_map)
   {
      vout_to_mV_map = bcm59035_regulator_map[regulatorID].vout_to_mV_map;
      map_size = bcm59035_regulator_map[regulatorID].map_size;
   }
   else
   {
      printk(KERN_ERR "Not supported\n");
      return -1;
   }
   // Mapping register value to voltage
   if (vout >= map_size)
   {
      printk("bcm59035: vout out of range\n");
      *mV = 0;
      return -1;
   }

   *mV = vout_to_mV_map[vout];
   return 0;
}



/****************************************************************************
*
*  bcm59035_regulator_get_state
*
*  Retrieve the current state of a regulator
*
***************************************************************************/
static BCM_PMU_Regulator_State_t bcm59035_regulator_get_state(int regulatorID)
{
   int rc;
   BCM_PMU_Regulator_State_t state;

   if (!bcm59035_regulator_map[regulatorID].available)
   {
      PMU_DEBUG(DBG_ERROR, "regulator %d not available.\n", regulatorID);
      return PMU_Regulator_Off;
   }

   rc = pmu_read(bcm59035_regulator_map[regulatorID].reg_addr);
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error reading regulator control register.\n");
      return PMU_Regulator_Off;
   }

   rc = bcm59035_opmod_to_state(regulatorID, (u8)rc, &state) ;
   if (rc < 0)
   {
      PMU_DEBUG(DBG_ERROR, "error converting state.\n");
      return PMU_Regulator_Off;
   }

   return state;
} /* bcm59035_regulator_get_state */

/****************************************************************************
*
*  bcm59035_set_voltage
*
* Set the current voltage levle
*
****************************************************************************/
static int bcm59035_regulator_set_voltage(int regulatorID, u32 mV)
{
   int rc,mask,shift;
   u8 val;
   u8 vout;

   if (!bcm59035_regulator_map[regulatorID].available)
   {
      printk("bcm59035_regulator_set_voltage:regulaor %d not available\n", regulatorID);
      return -EINVAL;
   }
   
   /* Convert voltage */
   rc = bcm59035_mV_to_vout(regulatorID, mV, &vout,
                            bcm59035_regulator_map[regulatorID].min_mV,
                            bcm59035_regulator_map[regulatorID].max_mV,
                            bcm59035_regulator_map[regulatorID].mV_step);

   if (rc < 0)
   {
      printk("bcm59035_regulator_set_voltage: error converting %d mV.\n", mV);
      return -EINVAL;
   }

   // read current settings
   rc = pmu_i2c_read(bcm59035_regulator_map[regulatorID].reg_addr_volt);
   if (rc < 0)
   {
      printk(KERN_ERR "bcm59035_set_voltage: error reading regulator control register.\n");
      return rc;
   }
   val = (u8)rc;
   mask = bcm59035_regulator_map[regulatorID].vout_mask;
   shift = bcm59035_regulator_map[regulatorID].vout_shift;

   // update register
   val &= ~(mask<<shift);
   val |= (vout<<shift);

   // write settings
   rc = pmu_i2c_write(bcm59035_regulator_map[regulatorID].reg_addr_volt, val);
   if (rc < 0)
   {
      printk("bcm59035_set_voltage: error writing regulator control register.\n");
      return rc;
   }
   return 0;

}

/****************************************************************************
*
*  bcm59035_regulator_get_voltage
*
*  Retrieve the current voltage level and optionally the valid range of settings.
*
***************************************************************************/
static u32 bcm59035_regulator_get_voltage(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step)
{
   int rc,mask,shift;
   u32 mV;

   if (!bcm59035_regulator_map[regulatorID].available)
   {
      printk("bcm59035_get_voltage: regulator %d not available.\n", regulatorID);
      return -EINVAL;
   }

   rc = pmu_i2c_read(bcm59035_regulator_map[regulatorID].reg_addr_volt);
   if (rc < 0)
   {
      printk("bcm59035_get_voltage: error reading regulator control register.\n");
      return rc;
   }
   mask = bcm59035_regulator_map[regulatorID].vout_mask;
   shift = bcm59035_regulator_map[regulatorID].vout_shift;
   rc = (rc>>shift) & mask;

   rc = bcm59035_vout_to_mV(regulatorID, (u8)rc, &mV) ;
   if (rc < 0)
   {
      printk("bcm59035_get_voltage: error converting voltage.\n");
      return rc;
   }

   if (min_mV)
      *min_mV = bcm59035_regulator_map[regulatorID].min_mV;
   if (max_mV)
      *max_mV = bcm59035_regulator_map[regulatorID].max_mV;
   if (mV_step)
      *mV_step = bcm59035_regulator_map[regulatorID].mV_step;

   return mV;
} /* bcm59035_regulator_get_voltage */

/****************************************************************************
*
*  bcm59035_charger_start
*
***************************************************************************/
static void bcm59035_charger_start(int chargerID)
{
   int rc;
   PMU_DEBUG(DBG_INFO, "%d\n", chargerID);
   if ((chargerID != BCM59035_CHARGER_MAIN) && (chargerID != BCM59035_CHARGER_USB))
   {
      PMU_DEBUG(DBG_ERROR, "Bad chargerID %d\n", chargerID);
      return;
   }
   if (chargerID == BCM59035_CHARGER_MAIN)
   {
      /* set the trickle charge threshold to 3.6V (default 3.2) to ensure we can power up properly */
      rc = pmu_read( BCM59035_REG_MBCCTRL3 );
      rc = ( BCM59035_MBCCTRL3_WAC_TC2_3_6V | (unsigned char)(rc & 0x0F) );
      rc = pmu_write( BCM59035_REG_MBCCTRL3, (uint8_t)rc );
   }
   else if (chargerID == BCM59035_CHARGER_USB)
   {
      /* set the trickle charge threshold to 3.6V (default 3.2) to ensure we can power up properly */
      rc = pmu_read( BCM59035_REG_MBCCTRL5 );
      rc = ( BCM59035_MBCCTRL5_USB_TC2_3_60V | (unsigned char)(rc & 0x0F) );
      pmu_write( BCM59035_REG_MBCCTRL5, (uint8_t)rc );

      // Rapid charge at 950 mA, trickle charge is set by OTP (50 mA)
      rc = pmu_read( BCM59035_REG_MBCCTRL6 );
      rc &= ~0x0f;
      rc |= BCM59035_MBCCTRL6_USB_RC_950MA;
      pmu_write( BCM59035_REG_MBCCTRL6, (uint8_t)rc);

      rc = pmu_read( BCM59035_REG_MBCCTRL8 );  // Get timeout settings in lower bits
      rc |= BCM59035_MBCCTRL8_VUBGRRC;
      pmu_write( BCM59035_REG_MBCCTRL8, (uint8_t)rc );   // Enable rapid charge mode and enable charger
   }

  // Bit 6 can clear by itself when charging complete, so don't trust the read value
   rc = pmu_read( BCM59035_REG_MBCCTRL2 );  // Get timeout settings in lower bits
   rc &= 0x3f; // mask out bits that can't be read reliably
   pmu_write( BCM59035_REG_MBCCTRL2, (uint8_t)rc );   // Reset state machine
   rc |= BCM59035_MBCCTRL2_VCHGRRC | BCM59035_MBCCTRL2_MBCHOSTEN;
   pmu_write( BCM59035_REG_MBCCTRL2, (uint8_t)rc );   // Enable rapid charge mode and enable charger

   rc = pmu_read( BCM59035_REG_MBCCTRL9 );
   rc |= BCM59035_MBCCTRL9_MAINTCHRG | BCM59035_MBCCTRL9_LOWCOST_WAC_EN | BCM59035_MBCCTRL9_LOWCOST_USB_EN;
   pmu_write( BCM59035_REG_MBCCTRL9, (uint8_t)rc );
   /* Enable EOC interrupt */
   rc = pmu_read( BCM59035_REG_INT2M );
   rc &= (~BCM59035_INT2_EOC);
   pmu_write( BCM59035_REG_INT2M, rc);
}

/****************************************************************************
*
*  bcm59035_charger_stop
*
***************************************************************************/
static void bcm59035_charger_stop(int chargerID)
{
   int rc;

   PMU_DEBUG(DBG_INFO, "%d\n", chargerID);

   if ((chargerID != BCM59035_CHARGER_MAIN) && (chargerID != BCM59035_CHARGER_USB))
   {
      PMU_DEBUG(DBG_ERROR,"Error: Bad chargerID %d\n", chargerID);
      return;
   }
   rc = pmu_read( BCM59035_REG_MBCCTRL2 );  // Get timeout settings in lower bits
   rc &= ~(BCM59035_MBCCTRL2_MBCHOSTEN | BCM59035_MBCCTRL2_VCHGRRC);
   pmu_write( BCM59035_REG_MBCCTRL2, (uint8_t)rc );   // Disable charging
   if (chargerID == BCM59035_CHARGER_USB)
   {
      rc = pmu_read( BCM59035_REG_MBCCTRL8 );  // Get timeout settings in lower bits
      rc &= ~(BCM59035_MBCCTRL8_VUBGRRC);
      pmu_write( BCM59035_REG_MBCCTRL8, (uint8_t)rc );   // Disable chargin
   }
}
/****************************************************************************
*
*  bcm59035_charger_is_inserted
*
***************************************************************************/
static int bcm59035_charger_is_inserted(int *chargerID)
{
   int rc;

   rc = pmu_read(BCM59035_REG_ENV1);
   if ( rc != -1 )
   {
      if( (u8)rc & (1<<1) )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59035_CHARGER_MAIN;
            return 1;
         }
      }
      if( (u8)rc & (1<<2) )
      {
         if (chargerID != NULL)
         {
            *chargerID = BCM59035_CHARGER_USB;
            return 1;
         }
      }
   }
   return 0;
}
/****************************************************************************
*
*  bcm59035_event_notify
*
***************************************************************************/
static void bcm59035_event_notify(BCM59035_InterruptId_t irq_id)
{
   BCM_PMU_Event_t event = PMU_NUM_EVENTS;
   void *data = NULL;
   int val;

   //PMU_DEBUG("irq_id=%d\n", irq_id);
   switch (irq_id)
   {
      // Onkey events
      case BCM59035_IRQID_INT1_PONKEYR:
         // Clear key-lock bit to allow auto shutoff in case software stops running
         val = pmu_read(BCM59035_REG_PONKEYBDB);
         if (val != -1)
            pmu_write(BCM59035_REG_PONKEYBDB, (u8)(val & ~BCM59035_PONKEYBDB_KEYLOCK));
         event = PMU_EVENT_ONKEY_RISE;
         break;

      case BCM59035_IRQID_INT1_PONKEYF:
         // Set key-lock bit to prevent auto shutoff after power-on key delay
         val = pmu_read(BCM59035_REG_PONKEYBDB);
         if (val != -1)
            pmu_write(BCM59035_REG_PONKEYBDB, (u8)(val | BCM59035_PONKEYBDB_KEYLOCK));
         event = PMU_EVENT_ONKEY_FALL;
         break;

      case BCM59035_IRQID_INT1_PONKEYH:
         event = PMU_EVENT_ONKEY_1S_HOLD;
         break;

      // Battery and charger events
      case BCM59035_IRQID_INT3_LOWBAT:
         event = PMU_EVENT_BATTERY_LOW;
         break;

      case BCM59035_IRQID_INT2_USBINS:
         isUSBChargerPresent = 1;
         PMU_DEBUG(DBG_INFO,"USB charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59035_CHARGER_USB;

         // start a one-shot timer set to expire later to enable EOC interrupt
         //PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgins timer now\n", CHGINS_MSEC);
         // If battery manger is present, battery manager API will get called
         // Else, call pmu driver directly
#ifndef CONFIG_BCM_BATTERY_MANAGER
	 bcm59035_charger_start(BCM59035_CHARGER_USB);
#endif
         isUSBChargerPresent = 1;
         break;

      case BCM59035_IRQID_INT2_USBRM:
         PMU_DEBUG(DBG_INFO,"USB charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59035_CHARGER_USB;
         // If battery manger is present, battery manager API will get called
         // Else, call pmu driver directly
#ifndef CONFIG_BCM_BATTERY_MANAGER
	 bcm59035_charger_stop(BCM59035_CHARGER_USB);
#endif
         isUSBChargerPresent = 0;
         break;
        
      case BCM59035_IRQID_INT2_CHGINS:
         PMU_DEBUG(DBG_INFO,"Main charger inserted\n");
         event = PMU_EVENT_CHARGER_INSERT;
         data = (void *)BCM59035_CHARGER_MAIN;
         
         // start a one-shot timer set to expire later to enable EOC interrupt
         //PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgins timer now\n", CHGINS_MSEC);
         // If battery manger is present, battery manager API will get called
         // Else, call pmu driver directly
#ifndef CONFIG_BCM_BATTERY_MANAGER
	 bcm59035_charger_start(BCM59035_CHARGER_MAIN);
#endif
         isWallChargerPresent = 1;
         break;

      case BCM59035_IRQID_INT2_CHGRM:
         PMU_DEBUG(DBG_INFO,"Main charger removed\n");
         event = PMU_EVENT_CHARGER_REMOVE;
         data = (void *)BCM59035_CHARGER_MAIN;
         // If battery manger is present, battery manager API will get called
         // Else, call pmu driver directly
#ifndef CONFIG_BCM_BATTERY_MANAGER
	 bcm59035_charger_stop(BCM59035_CHARGER_MAIN);
#endif
         isWallChargerPresent = 0;
         break;

      case BCM59035_IRQID_INT2_CHGEOC:
         if ((!isUSBChargerPresent) && (!isWallChargerPresent))
         {
            PMU_DEBUG(DBG_TRACE2,"Charger EOC ignored\n");
         }
         else
         {
            // start a one-shot timer set to expire later to see if there is a 
            // charger removal event
            //PMU_DEBUG(DBG_TRACE2,"Starting %d msec chgrm timer now\n", CHGRM_MSEC);
            //mod_timer(&eoc_chgrm_timer, jiffies + msecs_to_jiffies(CHGRM_MSEC));
            chargerEOC = 1;
         }
         // Disable the EOC interrupt here to avoid nested batful interrupts 
         setupEOCIntMask(isWallChargerPresent, isUSBChargerPresent);         
         val = pmu_read( BCM59035_REG_INT2M );
         val |= BCM59035_INT2_EOC;
         pmu_write( BCM59035_REG_INT2M, val);
         event = PMU_EVENT_BATTERY_FULL;
         break;
         
      case BCM59035_IRQID_INT2_CHGERR:
      case BCM59035_IRQID_INT2_USBERR:
         PMU_DEBUG(DBG_ERROR,"Charger voltage is greater than over-voltage threshold\n");
         /* We indicate full battery when error to stop charging */
         event = PMU_EVENT_BATTERY_FULL;
         break;

      case BCM59035_IRQID_INT2_MBCCHGERR:
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
      pmu_event_notify(PMU_BCM59035, event, data);
   }
#endif
}

/****************************************************************************
*
*  bcm59035_state_to_opmod
*
***************************************************************************/
static int bcm59035_state_to_opmod(int regulatorID, BCM_PMU_Regulator_State_t state, u8 *opmod)
{
   if (!opmod)
      return -1;

   switch (state)
   {
      case PMU_Regulator_Off:
         *opmod = BCM59035_BIT_REG_OFF;
         break;
      case PMU_Regulator_On:
         *opmod = BCM59035_BIT_REG_ON;
         break;
      case PMU_Regulator_Eco:
         *opmod = BCM59035_BIT_REG_ECO;
         break;
      default:
         return -1;
   }
   return 0;
}

/****************************************************************************
*
*  bcm59035_opmod_to_state
*
***************************************************************************/
static int bcm59035_opmod_to_state(int regulatorID, u8 opmod, BCM_PMU_Regulator_State_t *state)
{
   if (!state)
      return -1;

   switch (opmod)
   {
      case BCM59035_BIT_REG_OFF:
         *state = PMU_Regulator_Off;
         break;
      case BCM59035_BIT_REG_ON:
         *state = PMU_Regulator_On;
         break;
      case BCM59035_BIT_REG_ECO:
         *state = PMU_Regulator_Eco;
         break;
      default:
         return -1;
   }
   return 0;
}


/****************************************************************************
*
*  bcm59035_set_sim_voltage
*
*       Called to set sim voltage.
*
***************************************************************************/
static int bcm59035_config_ldo_mode(int ldo, int mode)
{
   int rc = 0;

   PMU_DEBUG(DBG_INFO, "BCM59035: ldo[0x%0x] mode[0x%0x]\n", ldo, mode);

   rc = pmu_write( ldo, mode );
   if (rc != 0) 
   {
      PMU_DEBUG(DBG_ERROR, "error writing to BCM59035_REG_SOPMODCTRL register\n");
      return(-EIO);
   }

   return(0);
}

/****************************************************************************
*
*  bcm59035_set_sim_voltage
*
*       Called to set sim voltage.
*
***************************************************************************/
static int bcm59035_set_sim_voltage(int voltage)
{
   int rc = 0;
   int new_voltage = 0;

   int current_voltage = pmu_read( BCM59035_REG_LCSIMDOCTRL );
   if (current_voltage < 0)
   {
      PMU_DEBUG(DBG_ERROR, "Error reading BCM59035_REG_LCSIMDOCTRL\n");
      return(-EIO);
   }

   new_voltage = current_voltage & ~BCM59035_LCSIMLDOCTRL_SIMMASK;
   new_voltage = new_voltage | (voltage << BCM59035_LCSIMLDOCTRL_SIM_BIT_SHIFT);

   rc = pmu_write( BCM59035_REG_LCSIMDOCTRL, new_voltage );
   if (rc != 0) 
   {
      PMU_DEBUG(DBG_ERROR, "error writing to BCM59035_REG_LCSIMDOCTRL register\n");
      return(-EIO);
   }

   PMU_DEBUG(DBG_INFO, "BCM59035: current_voltage[0x%0x] req_voltage[0x%0x] new_voltage[0x%0x]\n", current_voltage, voltage, new_voltage);

   return(rc);
}

/****************************************************************************
*
*  bcm59035_activate_sim
*
*       Called to activate SIM with the specified voltage.
*
***************************************************************************/
static int bcm59035_activate_sim(BCM59035_sim_voltage_t sim_voltage)
{
   int rc = 0;
   
   if (sim_voltage >= ESIM_Max_Voltage)
   {
      return(-EINVAL);
   }

   if ((rc = bcm59035_set_sim_voltage( sim_voltage )) != 0)
   {
      return(rc);
   }

   if ((rc = bcm59035_config_ldo_mode( BCM59035_REG_SOPMODCTRL, BCM59035_BIT_REG_ON )) != 0)
   {
      return(rc);
   }

   return(0);
}

/****************************************************************************
*
*  bcm59035_deactivate_sim
*
*       Called to deactivate SIM with the specified voltage.
*
***************************************************************************/
static int bcm59035_deactivate_sim(void)
{
   int rc = 0;

   if ((rc = bcm59035_config_ldo_mode( BCM59035_REG_SOPMODCTRL, BCM59035_BIT_REG_OFF )) != 0)
   {
      return(rc);
   }

   return(0);
}

/****************************************************************************
*
*  bcm59035_is_dvs_enabled
*
*       Called to check if dvs for csr is enabled.
*
***************************************************************************/
int bcm59035_is_dvs_enabled()
{
   int value;
   value = pmu_read(BCM59035_REG_CSRCTRL1);
   value &= BCM59035_CSRCTRL1_DVS_ENABLE;
   if (value)
      return 1;
   else
      return 0; 
}
 
/****************************************************************************
*
*  bcm59035_enable_dvs
*
*       Called to enable dvs 
*
***************************************************************************/
void bcm59035_enable_dvs(void)
{
   int value;
   value = pmu_read(BCM59035_REG_CSRCTRL1);
   value |= BCM59035_CSRCTRL1_DVS_ENABLE;
   pmu_write(BCM59035_REG_CSRCTRL1,value);
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_to_mV_map = bcm59035_csr_dvs_vout_to_mV_map;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].map_size = (sizeof(bcm59035_csr_dvs_vout_to_mV_map)/sizeof(bcm59035_csr_dvs_vout_to_mV_map[0]));
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].reg_addr_volt = BCM59035_REG_CSRCTRL10; /* Assuming normal mode */ 
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].min_mV = 900;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].max_mV = 1500;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].mV_step = 20;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_mask = 0x1F;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_shift = 0;
}

/****************************************************************************
*
*  bcm59035_disable_dvs
*
*       Called to disable dvs 
*
***************************************************************************/
void bcm59035_disable_dvs(void)
{
   int value;
   value = pmu_read(BCM59035_REG_CSRCTRL1);
   value &= ~(BCM59035_CSRCTRL1_DVS_ENABLE);
   pmu_write(BCM59035_REG_CSRCTRL1,value);
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_to_mV_map = bcm59035_csr_nodvs_vout_to_mV_map;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].map_size = (sizeof(bcm59035_csr_nodvs_vout_to_mV_map)/sizeof(bcm59035_csr_nodvs_vout_to_mV_map[0]));
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].reg_addr_volt = BCM59035_REG_CSRCTRL2;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].min_mV = 900;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].max_mV = 2500;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].mV_step = 100;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_mask = 0x1F;
   bcm59035_regulator_map[BCM59035_REGULATOR_CSR].vout_shift = 2;
}

/****************************************************************************
*
*  bcm59035_module_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init bcm59035_module_init( void )
{
   printk( banner );

   PMU_DEBUG(DBG_TRACE,"register with PMU module\n");
   return pmu_register_device(PMU_BCM59035, &bcm59035_ops);
}


/****************************************************************************
*
*  bcm59035_module_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit bcm59035_module_exit( void )
{
   PMU_DEBUG(DBG_TRACE,"module_exit called\n");
}


fs_initcall(bcm59035_module_init);
module_exit(bcm59035_module_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM59035 Driver");


