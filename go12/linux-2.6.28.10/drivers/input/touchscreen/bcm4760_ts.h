/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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


#ifndef BCM4760_TS
#error This file is meant to be included by a chip-dependent wrapper file such as bcm4760_ts.c
#endif

#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm/mach/irq.h>


static int __init bcm4760_ts_init(void);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t bcm4760_ts_isr( int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t bcm4760_ts_isr( int irq, void *dev_id );
#endif
static int proc_do_ts_intvec_set_param (ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );

#define BCM4760_TS_MODULE_DESC	         "Touchscreen driver for Broadcom 4760"
#define BCM4760_TS_MODULE_VERSION         "1.2"

#define ISR_NAME                          "bcm4760_tsc_isr"
#define BCM_TSDEV_NAME                    "bcm4760_ts"

#define TRUE	 1
#define FALSE    0 





typedef struct bcm4760_ts_dev
{
	struct input_dev *input;

   TSC_EVENT touch_data;

   int data_available;
   int pen_transition;

   int irq;
} bcm4760_ts_dev;

static DEFINE_SPINLOCK(lock);



typedef struct
{
   int32_t settling_timeout;             /* Setttling Timeout */
   int32_t register_value;               /* Register Value */
} settling_table_t;

typedef struct
{
   int32_t data_average;                 /* Number of data points to average */
   int32_t register_value;               /* Register Value */
} dataaverage_table_t;

/****************************************************************************
** Touch Screen Controller (TSC) def 
****************************************************************************/
#define TSC_R_CNTL1_OFFSET                                                          0x00
#define     TSCHW_CNTL1_SCAN_PERIOD                 (0xFF << 24)
#define     TSCHW_CNTL1_DEBOUNCE_TIMEOUT            (0x0FF << 16)
#define     TSCHW_CNTL1_SETTLING_TIMEOUT            (0x0F << 8)
#define     TSCHW_CNTL1_TOUCH_TIMEOUT               (0xFF << 0 )


#define TSC_R_CNTL2_OFFSET                                                          0x04
#define     TSCHW_CNTL2_CON_EN                 		( 1 << 16)
#define     TSCHW_CNTL2_AVERAGE_DATA_NUM        	( 0x7<< 8)
#define     TSCHW_CNTL2_REG2P5_PWRDN            	( 1<< 6)
#define     TSCHW_CNTL2_LDO_PWRDN               	( 1<< 5)
#define     TSCHW_CNTL2_ADC_PWRDN               	( 1<< 4)
#define     TSCHW_CNTL2_BGP_PWRDN               	( 1<< 3)
#define     TSCHW_CNTL2_WIRE_PWRDN              	( 1<< 2)
#define     TSCHW_CNTL2_WIRE_MODE_CNTL          	( 1<< 1)
#define     TSCHW_CNTL2_AUXIN_SCAN_EN           	( 1<< 0)

#define TSC_R_FIFO_THRESH_OFFSET                                                    0x08
#define     TSCHW_FIFO_THRESH_MASK              	(0x1F << 0)

#define TSC_R_INT_MASK_OFFSET                                                       0x0C
#define   TSCHW_INT_MASK_ADC_RDY_EDGE               ( 1<< 5)
#define   TSCHW_INT_MASK_ADC_STRT_EDGE              ( 1<< 4)
#define   TSCHW_INT_MASK_AUXIN_AVAIL                ( 1<< 3)
#define   TSCHW_INT_MASK_FIFO_THRESHOLD             ( 1<< 2)
#define   TSCHW_INT_MASK_FIFO_OVL                   ( 1<< 1)
#define   TSCHW_INT_MASK_PEN_DOWN_IRQ               ( 1<< 0)

#define TSC_R_INT_STAT_OFFSET                                                       0x10
#define     TSCHW_INT_STAT_ADC_RDY_EDGE             ( 1<< 5)
#define     TSCHW_INT_STAT_ADC_STRT_EDGE            ( 1<< 4)
#define     TSCHW_INT_STAT_AUXIN_AVAIL              ( 1<< 3)
#define     TSCHW_INT_STAT_FIFO_THRESHOLD           ( 1<< 2)
#define     TSCHW_INT_STAT_FIFO_OVL                 ( 1<< 1)
#define     TSCHW_INT_STAT_PEN_DOWN_IRQ             ( 1<< 0)

#define TSC_R_CON_STAT_OFFSET                                                       0x14
#define     TSCHW_CON_STAT_DIG_FSM_STAT             ( 0xF<< 12)
#define     TSCHW_CON_STAT_ANA_FSM_STAT             ( 0x7<< 8)
#define     TSCHW_CON_STAT_ADC_RDY                  ( 1<< 5)
#define     TSCHW_CON_STAT_ADC_STRT                 ( 1<< 4)
#define     TSCHW_CON_STAT_TOUCH_DET                ( 1<< 2)
#define     TSCHW_CON_STAT_FIFO_EMPTY               ( 1<< 1)
#define     TSCHW_CON_STAT_PEN_DOWN                 ( 1<< 0)

#define TSC_R_FIFO_ACCESS_OFFSET                                                    0x18
#define     TSCHW_FIFO_ACCESS_MASK              	(0xFFFFFFFF)

#define TSC_R_ANACNTL_OFFSET                                                        0x1C
#define     TSCHW_ANACNTL_MASK                   	(0x0FFFFFFF)

#define TSC_R_AUXIN_STAT_OFFSET                                                     0x20
#define     TSCHW_AUXIN_STAT_MASK                   (0x3F)

#define TSC_R_DEBOUNCE_CNTR_STAT_OFFSET                                             0x24
#define     TSCHW_DEBOUNCE_CNTR_STAT_MASK           (0x1FFF) 

#define TSC_R_SCAN_CNTR_STAT_OFFSET                                                 0x28
#define     TSCHW_SCAN_CNTR_STAT_MASK               (0x3FFF)

#define TSC_R_REM_CNTR_STAT_OFFSET                                                  0x2C
#define     TSCHW_REM_CNTR_STAT_MASK                (0xFF)

#define TSC_R_SETTLING_TIMER_STAT_OFFSET                                            0x30
#define     TSCHW_SETTLING_TIMER_STAT_MASK          (0x3FF) 

#define TSC_R_SPARE_REG_OFFSET                                                      0x34


#define TSCHW_MODE_4WIRE	4
#define TSCHW_MODE_5WIRE	5


#define    cntl1_SETTLING_TIMEOUT_10240USEC    11  
#define    cntl1_SETTLING_TIMEOUT_05120USEC    10
#define    cntl1_SETTLING_TIMEOUT_02560USEC    9
#define    cntl1_SETTLING_TIMEOUT_01280USEC    8
#define    cntl1_SETTLING_TIMEOUT_00640USEC    7
#define    cntl1_SETTLING_TIMEOUT_00320USEC    6
#define    cntl1_SETTLING_TIMEOUT_00160USEC    5
#define    cntl1_SETTLING_TIMEOUT_00080USEC    4
#define    cntl1_SETTLING_TIMEOUT_00040USEC    3
#define    cntl1_SETTLING_TIMEOUT_00020USEC    2
#define    cntl1_SETTLING_TIMEOUT_00010USEC    1
#define    cntl1_SETTLING_TIMEOUT_00008USEC    0

#define    cntl2_AVG_64     6
#define    cntl2_AVG_32     5
#define    cntl2_AVG_16     4
#define    cntl2_AVG_8      3
#define    cntl2_AVG_4      2
#define    cntl2_AVG_2      1
#define    cntl2_AVG_1      0

#define		TSC_CTL1_SCAN_PERIOD_SHIFT				24
#define 	TSC_CTL1_DEBOUNCE_TIMEOUT_SHIFT      	16
#define 	TSC_CTL1_DEBOUNCE_TIMEOUT_MASK 			(0xff << TSC_CTL1_DEBOUNCE_TIMEOUT_SHIFT)
#define 	TSC_CTL2_ENABLE  	                	(1 << 16)
#define 	TSC_CTL1_DEBOUNCE_TIMEOUT_MASK       	(0xff << TSC_CTL1_DEBOUNCE_TIMEOUT_SHIFT)
#define 	TSC_CTL1_SETTLING_TIMEOUT_SHIFT      	8
#define 	TSC_CTL1_SETTLING_TIMEOUT_MASK       	(0xf << TSC_CTL1_SETTLING_TIMEOUT_SHIFT)
#define 	TSC_CTL2_AVG_SHIFT                  	8
#define 	TSC_CTL2_AVG_MASK                   	(0x7 << TSC_CTL2_AVG_SHIFT)
#define 	TSC_CTL2_4WIRE  	                	(1 << 2)



#define MMR1103_TSC_CTL2_PWRDN_ANALOG_WCTL          (1 << 1)
#define MMR1103_TSC_STATUS_FIFO_EMPTY               (1 << 1)
#define MMR1103_TSC_STATUS_PEN_DOWN                 (1 << 0)
#define MMR1103_TSC_INTR_PEN_CHANGE                 (1 << 0)
#define MMR1103_TSC_INTR_FIFO_THRESH                (1 << 2)
#define MMR1103_TSC_INTR_FIFO_OVERFLOW              (1 << 1)
 
/* ---- Private Constants and Types --------------------------------------- */
#define  REG_TSC (volatile TSC_REG_BASE_ADDR)      /* Touchscreen controller registers */

#define SCAN_PERIOD_MAX   64             			/* Maximum supported scan period 64ms */
#define DEBOUNCE_TIMEOUT_UNIT_USECS 512  			/* Debounce timeout configured in units of 0.512 msec */



#define MAXx		0xffff
#define MAXy		0xffff

/* ---- Functions ------------------------------------------------- */
static int bcm4760_tsc_enable(void);
static int bcm4760_tsc_is_irq(void);
static int bcm4760_tsc_get_sample_rate(void);
static int bcm4760_tsc_set_sample_rate(uint32_t sample_rate);
static int bcm4760_tsc_set_data_threshold(uint32_t threshold);
static int bcm4760_tsc_get_data_threshold(void);
static tsc_wire_mode bcm4760_tsc_get_mode ( void );
static tsc_wire_mode bcm4760_tsc_set_mode ( tsc_wire_mode mode );
static int bcm4760_tsc_set_data_point_average(uint32_t average);
static int bcm4760_tsc_get_data_point_average(void);
static int bcm4760_tsc_set_settling(uint32_t timeout);
static int bcm4760_tsc_get_settling(void);
static int bcm4760_tsc_set_debounce(uint32_t timeout);
static int bcm4760_tsc_get_debounce(void);
static int bcm4760_tsc_is_pen_down(void);
static int bcm4760_tsc_isr_data( uint32_t *xtouch, uint32_t *ytouch, uint32_t *pd );
static int bcm4760_tsc_isr_pen (void);

