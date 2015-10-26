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

//#define DEBUG            /* define this to enable printk messages */

/*
 * BCM4760 touchscreen driver
 *
 * This driver can handle Broadcom 4760 touchscreen controller.
 * It has been tested with the Catalina-4760 board using 4-wire control to the
 * CHILIN touchscreen.
 *
 * The driver fits the input driver model and provide evdev interface.
 * User can access the touchscreen data through the /dev/input/event0 node.
 *
 * It is recommended to use tslib for user space programs to access the
 * touchscreen raw data.
 * Tslib can be used to calibrate the touchscreen raw data.
 *
 */

#include <linux/version.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>              /* For request_irq */

#include <linux/platform_device.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <asm/arch/bcm4760_reg.h>
#include <linux/broadcom/gpio_irq.h>
#include <asm/gpio.h>
#include <asm/mach/irq.h>

#include <linux/broadcom/ts.h>

#define BCM4760_TS
#define ANACNTL_THRESHOLD 0x3

#include "bcm4760_ts.h"


static unsigned int panel_type = 1;       /* 0 = qvga, 1 = wqvga, 2 = vga, 3 = wvga */
module_param(panel_type, uint, 0644);
static int g_set_param = 0;
static int maxdifx;
static int maxdify;
static struct ctl_table_header *g_sysctl_header;

static unsigned int  RegTouch;


static struct tsc_control_table *gp_control_table;
#define g_control_table		(*gp_control_table)

struct tsc_panel_params
{
   unsigned int xres;
   unsigned int yres;
   unsigned int tscMaxX;                  // max X touch value
   unsigned int tscMaxY;                  // max Y touch value
   unsigned int tscMinX;                  // min X touch value
   unsigned int tscMinY;                  // min Y touch value
};

struct tsc_panel_params PanelParamTable[] =
{
   {
      .xres    = 320,
      .yres    = 240,
   },
   {
      .xres    = 480,
      .yres    = 272,
   },
   {
      .xres    = 640,
      .yres    = 480,
   },
   {
      .xres    = 800,
      .yres    = 480,
   }
};

static struct ctl_table g_sysctl_tsc[] =
{
   {
      .ctl_name      = 1,
      .procname      = "set_param",
      .data          = &g_set_param,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_ts_intvec_set_param
   },
   {
      .ctl_name      = 2,
      .procname      = "sample_rate",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 3,
      .procname      = "data_threshold",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 4,
      .procname      = "debounce",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 5,
      .procname      = "settling",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 6,
      .procname      = "data_point_average",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 7,
      .procname      = "wire_mode",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 8,
      .procname      = "MaxX",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 9,
      .procname      = "MinX",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 10,
      .procname      = "MaxY",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 11,
      .procname      = "MinY",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 1,
      .procname      = "ABSxy",
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};

static struct ctl_table g_sysctl[] =
{
   {
      .ctl_name   = CTL_BCM_TSC,
      .procname   = "tsc",
      .mode       = 0555,
      .child      = g_sysctl_tsc
   },
   {}
};

/* Table for settling timeout to a register value */
static const settling_table_t settlingTable[] =
{
   { 10240,  cntl1_SETTLING_TIMEOUT_10240USEC},
   {  5120,  cntl1_SETTLING_TIMEOUT_05120USEC},
   {  2560,  cntl1_SETTLING_TIMEOUT_02560USEC},
   {  1280,  cntl1_SETTLING_TIMEOUT_01280USEC},
   {   640,  cntl1_SETTLING_TIMEOUT_00640USEC},
   {   320,  cntl1_SETTLING_TIMEOUT_00320USEC},
   {   160,  cntl1_SETTLING_TIMEOUT_00160USEC},
   {    80,  cntl1_SETTLING_TIMEOUT_00080USEC},
   {    40,  cntl1_SETTLING_TIMEOUT_00040USEC},
   {    20,  cntl1_SETTLING_TIMEOUT_00020USEC},
   {    10,  cntl1_SETTLING_TIMEOUT_00010USEC},
   {     8,  cntl1_SETTLING_TIMEOUT_00008USEC},
   {    -1, -1 }
};

/* Table for data point average to a register value */
static const dataaverage_table_t dataAverageTable[] =
{
   {    64,  cntl2_AVG_64 },
   {    32,  cntl2_AVG_32 },
   {    16,  cntl2_AVG_16 },
   {     8,  cntl2_AVG_8  },
   {     4,  cntl2_AVG_4  },
   {     2,  cntl2_AVG_2  },
   {     1,  cntl2_AVG_1  },
   {    -1, -1 }
};




/*--------------------------------------------------------------------------*/

//#define BCM4760_TS

/*****************************************************************************/
/*
 * Initialize the touchscreen controller, set default configuration
 */
static int bcm4760_tsc_drv_init(void)
{
   int rc;

#ifdef DEBUG
   printk("BEG-bcm4760 touchscreen controller initialized!\n");
#endif

   RegTouch = (unsigned int )ioremap(TSC_REG_BASE_ADDR, 0x60);
#ifdef DEBUG
   printk("RegTouch:%X\n",RegTouch);
#endif
   if (!RegTouch) {
#ifdef DEBUG
      printk(KERN_ERR "TOUCHSCREEN: Unable to remap registers\n");
#endif
      return 1;
   }


   rc = bcm4760_tsc_enable();

   /* Set default configuration */
   bcm4760_tsc_set_sample_rate(g_control_table.sample_rate);
   bcm4760_tsc_set_data_threshold(g_control_table.data_threshold);
   bcm4760_tsc_set_debounce(g_control_table.debounce);
   bcm4760_tsc_set_settling(g_control_table.settling);
   bcm4760_tsc_set_data_point_average(g_control_table.data_point_average);
//   bcm4760_tsc_set_mode(g_control_table.wire_mode);

   /* Set pendown detect threshold to 600mV */
   writel( ANACNTL_THRESHOLD, RegTouch + TSC_R_ANACNTL_OFFSET);

   /* Enable the touch screen controller */
//#if CFG_GLOBAL_DISPLAY_PANEL_TYPE == 2
//   writel( 0x00300000, RegTouch + TSC_R_ANACNTL_OFFSET);
//#endif
   writel( readl(RegTouch + TSC_R_CNTL2_OFFSET) | TSCHW_CNTL2_CON_EN | TSCHW_CNTL2_AUXIN_SCAN_EN | TSCHW_CNTL2_WIRE_MODE_CNTL, RegTouch + TSC_R_CNTL2_OFFSET);

   writel( 0xffff,RegTouch + TSC_R_INT_STAT_OFFSET);		// clr all interrupts
   writel(TSCHW_INT_MASK_PEN_DOWN_IRQ | TSCHW_INT_MASK_FIFO_OVL | TSCHW_INT_MASK_FIFO_THRESHOLD,RegTouch + TSC_R_INT_MASK_OFFSET);		// enable interrupts

#ifdef DEBUG
   printk( "TSC CNTL1: %X\n", readl(RegTouch + TSC_R_CNTL1_OFFSET) );
   printk( "TSC CNTL2: %X\n", readl(RegTouch + TSC_R_CNTL2_OFFSET) );
   printk( "TSC MASK : %X\n", readl(RegTouch + TSC_R_INT_MASK_OFFSET) );

   printk("END-bcm4760 touchscreen controller initialized!\n");
#endif

   return 0;


}

/*
 * Disable the touchscreen controller, disable interrupt
 */
static void bcm4760_tsc_drv_exit(void)
{
//   int rc;

//   rc = bcm1103_tsc_irq_pen_disable();
//   rc = bcm1103_tsc_irq_data_disable();
//   rc = bcm1103_tsc_disable();

#ifdef DEBUG
   printk("bcm4760 touchscreen controller exit!\n");
#endif

   return;
}

/*
 * Enable the touchscreen block
 */
static int bcm4760_tsc_enable(void)
{

   /* Enable analog block */

#ifdef DEBUG
   printk("from bcm4760_tsc_enable:%p\n",(void *)(RegTouch + TSC_R_CNTL2_OFFSET));
#endif

   /* Disable power down of the analog state machine */
   writel( readl(RegTouch + TSC_R_CNTL2_OFFSET) & ~TSCHW_CNTL2_REG2P5_PWRDN,RegTouch + TSC_R_CNTL2_OFFSET);

   msleep(2);              //2msec delay

   /* Disable power down of the block */
   writel( readl(RegTouch + TSC_R_CNTL2_OFFSET) & ~TSCHW_CNTL2_LDO_PWRDN,RegTouch + TSC_R_CNTL2_OFFSET);


   /* Enable the touch screen controller */
 //  writel( readl(RegTouch + TSC_R_CNTL2_OFFSET) | TSC_CTL2_ENABLE, RegTouch + TSC_R_CNTL2_OFFSET);

   /* Set touch removed counter to 0 to avoid data outlyer when pen up occurs */
   writel(readl(RegTouch + TSC_R_CNTL2_OFFSET) &
          ~(TSCHW_CNTL2_ADC_PWRDN      |
            TSCHW_CNTL2_BGP_PWRDN      |
            TSCHW_CNTL2_WIRE_PWRDN),
          RegTouch + TSC_R_CNTL2_OFFSET );

   return 1;
//   return( bcm4760_tsc_is_enable() );
}

/*
 * Check if touchscreen interrupt is triggered
 */
static int bcm4760_tsc_is_irq(void)
{

   if(readl(RegTouch + TSC_R_INT_STAT_OFFSET) &
      (TSCHW_INT_STAT_FIFO_THRESHOLD | TSCHW_INT_STAT_FIFO_OVL | TSCHW_INT_STAT_PEN_DOWN_IRQ))
      return 1;
   else
      return 0;
}

/*
 *  Get the sample rate used to control the tsc
 *
 *  This function will get the sample used to control the touch screen
 *  logic.  The sample rate is the frequency which data samples
 *  are generated.
 *
 *  @return
 *     Configured sample rate for the touch screen in Hz
 *
 *  @note
 *     The sample rate can be configured from 16 to 1000 Hz
 *     The actual sample rate may be quantized based on allowable
 *     inter-sample durations of 1ms to 64ms.
 */
static int bcm4760_tsc_get_sample_rate(void)
{
   uint32_t scan_period_msec;

   /* Get scan period in msec.  Each period count corresonds to 1 ms */
   scan_period_msec = ( readl(RegTouch + TSC_R_CNTL1_OFFSET) & TSCHW_CNTL1_SCAN_PERIOD ) >> TSC_CTL1_SCAN_PERIOD_SHIFT;
   if ( scan_period_msec <= 0) {
      scan_period_msec = 1;
   }

   /* Convert scan period (ms) to Hz*/
   return ( MSEC_PER_SEC/scan_period_msec );
}

/*
 *  Set the sample rate
 *
 *  This function will set the sample used to control the touch screen
 *  logic.  The sample rate is the frequency which data samples
 *  are generated.
 *
 *  @param   period - [IN] sample rate in Hz
 *
 *  @return
 *     Configured sample rate for the touch screen in Hz
 *
 *  @note
 *     The sample rate can be configured from 15 to 1000 Hz
 *     The actual sample rate may be quantized based on allowable
 *     inter-sample durations of 1ms to 64ms and will result
 *     in sample rates rounded up to the next valid value
 */
static int bcm4760_tsc_set_sample_rate(uint32_t sample_rate)
{
   uint32_t scan_period_msec;


#ifdef DEBUG
   printk("set_sample_rate:%X\n",sample_rate);
#endif

   if ( sample_rate <= 0) {
      return (bcm4760_tsc_get_sample_rate());
   }

   scan_period_msec = MSEC_PER_SEC/sample_rate;

   scan_period_msec = (scan_period_msec >= SCAN_PERIOD_MAX) ? SCAN_PERIOD_MAX : scan_period_msec;

   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) & ~TSCHW_CNTL1_SCAN_PERIOD, RegTouch + TSC_R_CNTL1_OFFSET);

   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) | ((scan_period_msec<<TSC_CTL1_SCAN_PERIOD_SHIFT) & TSCHW_CNTL1_SCAN_PERIOD), RegTouch + TSC_R_CNTL1_OFFSET);

   /* Return the sample rate in Hz */
   return (bcm4760_tsc_get_sample_rate());
}




/*
 * Set data threshold
 * The value is 0-31
 */
static int bcm4760_tsc_set_data_threshold(uint32_t threshold)
{
#ifdef DEBUG
   printk("set_data_threshold:%X\n",threshold);
#endif
   writel( threshold, RegTouch + TSC_R_FIFO_THRESH_OFFSET);

   /* Return the threshold */
   return ( bcm4760_tsc_get_data_threshold( ) );
}

/*
 * Get data threshold
 *
 * This function will get the data threshold.  The threshold
 * is the number of entries of type (x,y) that are present before an
 * data ISR is raised.
 */
static int bcm4760_tsc_get_data_threshold(void)
{
   /* Return the threshold */
   return  readl(RegTouch + TSC_R_FIFO_THRESH_OFFSET );
}

/**
*  Get debounce timeout
*
*  This function will get the timeout used to control the touch screen debounce
*  logic.  The debounce timeout is the duration which touch is
*  detected continously before touch screen interrupt is generated
*
*  @return
*     Debounce timeout for the touch screen in usec
*
*/
static int bcm4760_tsc_get_debounce(void)
{
   uint32_t timeout_count;


   timeout_count = (readl(RegTouch + TSC_R_CNTL1_OFFSET) & TSC_CTL1_DEBOUNCE_TIMEOUT_MASK) >> TSC_CTL1_DEBOUNCE_TIMEOUT_SHIFT;

   /* Return the timeout in usec.  Each timeout count corresonds to 0.512ms */
   return (timeout_count * DEBOUNCE_TIMEOUT_UNIT_USECS );
}

/**
*  Set debounce timeout
*
*  This function will set the timeout used to control the touch screen debounce
*  logic.  The debounce timeout is the duration which touch is
*  detected continously before touch screen interrupt is generated
*
*  @param   timeout - [IN] Debounce timeout in usec
*
*  @return
*     Configured debounce timeout for the touch screen in usec
*
*  @note
*     Debounce timeout will be set in multiple of 512usec
*     Non-multiples of 512us will be rounded down.
*/
static int bcm4760_tsc_set_debounce(uint32_t timeout)
{
   uint32_t timeout_count;

#ifdef DEBUG
   printk("set_debounce:%X\n",timeout);
#endif

   timeout_count = timeout / DEBOUNCE_TIMEOUT_UNIT_USECS;

   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) & ~TSCHW_CNTL1_DEBOUNCE_TIMEOUT, RegTouch + TSC_R_CNTL1_OFFSET);

   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) | ((timeout_count<<TSC_CTL1_DEBOUNCE_TIMEOUT_SHIFT) & TSC_CTL1_DEBOUNCE_TIMEOUT_MASK), RegTouch + TSC_R_CNTL1_OFFSET);

   /* Return the timeout in usec.  Each timeout count corresonds to 0.512ms */
   return (bcm4760_tsc_get_debounce());
}

/**
*  Get settling timeout
*
*  This function will get the timeout used to control the settling duration.
*  The settling duration is amount of time the touch screen controller
*  waits to allow the voltage to settle after turning on the drivers
*  in detection mode.  This allows for screen and decoupling
*  capacitances
*
*  @return
*     Settling timeout for the touch screen in usec
*
*/
static int bcm4760_tsc_get_settling(void)
{
   const settling_table_t *tablep;
   int32_t settling_value;

   settling_value = (readl(RegTouch + TSC_R_CNTL1_OFFSET) & TSC_CTL1_SETTLING_TIMEOUT_MASK) >> 8;
   tablep = &settlingTable[0];

   while( tablep->register_value != -1 )
   {
      if( settling_value == tablep->register_value )
      {
         return( tablep->settling_timeout );
      }
      tablep++;
   }

   return( 0 );
}

/**
*  Set settling timeout
*
*  This function will set the timeout used to control the settling duration.
*  The settling duration is amount of time the touch screen controller
*  waits to allow the voltage to settle after turning on the drivers
*  in detection mode.  This allows for screen and decoupling
*  capacitances
*
*  @param   timeout - [IN] Settling timeout in usec
*
*  @return
*     Settling timeout for the touch screen in usec
*
*  @note
*     The timeout can only be configure in quantized amounts
*     The BCM4760 support configurations of
*     8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384 usec
*     Configurations specified between the quantized amounts will be set
*     to the lower quantized value.
*/
static int bcm4760_tsc_set_settling(uint32_t timeout)
{
   const settling_table_t *tablep;
   int32_t settling_value;

#ifdef DEBUG
   printk("set_settling:%X\n",timeout);
#endif
   settling_value = -1;
   tablep = &settlingTable[0];

   while( tablep->settling_timeout != -1 )
   {
      if( (int32_t)timeout >= tablep->settling_timeout )
      {
         settling_value = tablep->register_value;
         break;
      }
      tablep++;
   }

   if( settling_value == -1 )
   {
      tablep--;
      settling_value = tablep->register_value;
   }

   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) & ~TSC_CTL1_SETTLING_TIMEOUT_MASK, RegTouch + TSC_R_CNTL1_OFFSET);
   writel( readl(RegTouch + TSC_R_CNTL1_OFFSET) | (settling_value << 8), RegTouch + TSC_R_CNTL1_OFFSET);

   return( bcm4760_tsc_get_settling( ) );
}

/**
*  Get number of data points to average
*
*  This function will get the configured number of data samples which are
*  averaged before a final data point is placed into the FIFO
*
*  @return
*     Number of data points that are averaged
*
*/
static int bcm4760_tsc_get_data_point_average(void)
{
   const dataaverage_table_t *tablep;
   int32_t data_average_value;

   data_average_value = (readl(RegTouch + TSC_R_CNTL2_OFFSET ) & TSC_CTL2_AVG_MASK) >> 8;
   tablep = &dataAverageTable[0];

   while( tablep->register_value != -1 )
   {
      if( data_average_value == tablep->register_value )
      {
         return( tablep->data_average );
      }
      tablep++;
   }

   return( 1 );
}

/**
*  Set number of data points to average
*
*  This function will set the configured number of data samples which are
*  averaged before a final data point is placed into the FIFO
*
*  average - [IN] Number of data points to average
*
*  @return
*     Number of data points that are averaged
*
*  @note
*     The data average can only be configure in quantized amounts
*     The BCM4760 support configurations of
*     1, 2, 4, 8, 16, 32, 64, 128 samples
*     Configurations specified between the quantized amounts will be set
*     to the lower quantized value.
*/
static int bcm4760_tsc_set_data_point_average(uint32_t average)
{
   const dataaverage_table_t *tablep;
   int32_t data_average_value;

#ifdef DEBUG
   printk("set_data_point_average:%X\n",average);
#endif
   data_average_value = -1;
   tablep = &dataAverageTable[0];

   while( tablep->data_average != -1 )
   {
      if( (int32_t)average >= tablep->data_average )
      {
         data_average_value = tablep->register_value;
         break;
      }
      tablep++;
   }

   if( data_average_value == -1 )
   {
      tablep--;
      data_average_value = tablep->register_value;
   }

   writel(readl(RegTouch + TSC_R_CNTL2_OFFSET ) & ~TSC_CTL2_AVG_MASK, RegTouch + TSC_R_CNTL2_OFFSET);
   writel(readl(RegTouch + TSC_R_CNTL2_OFFSET ) | ((data_average_value << TSC_CTL2_AVG_SHIFT) & TSC_CTL2_AVG_MASK), RegTouch + TSC_R_CNTL2_OFFSET);

   return bcm4760_tsc_get_data_point_average( );
}

/*
 * Set the wire mode of the touchscreen
 */
static tsc_wire_mode bcm4760_tsc_set_mode ( tsc_wire_mode mode )
{
   if ( mode == TSC_MODE_4WIRE )
   {
      writel(readl(RegTouch + TSC_R_CNTL2_OFFSET ) | TSC_CTL2_4WIRE, RegTouch + TSC_R_CNTL2_OFFSET);
   }
   else if ( mode == TSC_MODE_5WIRE )
   {
      writel(readl(RegTouch + TSC_R_CNTL2_OFFSET ) & ~TSC_CTL2_4WIRE, RegTouch + TSC_R_CNTL2_OFFSET);
   }

   return bcm4760_tsc_get_mode ();
}

/*
 * Return the wire mode of the touchscreen
 */
static tsc_wire_mode bcm4760_tsc_get_mode ( void )
{
   if ( readl(RegTouch + TSC_R_CNTL2_OFFSET ) & TSC_CTL2_4WIRE)
   {
      return TSC_MODE_4WIRE;
   }
   else
   {
      return TSC_MODE_5WIRE;
   }
}

/*
 * Check if the pen is down
 */
static int bcm4760_tsc_is_pen_down(void)
{
   if ( readl(RegTouch + TSC_R_CON_STAT_OFFSET ) & MMR1103_TSC_STATUS_PEN_DOWN )
   {
      /* Pen is currently down */
      return 1;
   }
   else
   {
      /* Pen is currently up */
      return 0;
   }
}


/**
*
*  This function will service the data interrupt
*
*  @return
*     1 if the data interrupt was serviced
*     0 if no data interrupt was serviced
*/
/****************************************************************************/
static int bcm4760_tsc_isr_data( uint32_t *xtouch, uint32_t *ytouch, uint32_t *pd )
{
   int x, y;
   static int pendown=0, penup_count=0, down_counter=0;
   static int lastx=0, lasty=0, avx=0, avy=0;
   int difx, dify;
   uint32_t Fdata;

   if( readl(RegTouch + TSC_R_INT_STAT_OFFSET) & MMR1103_TSC_INTR_FIFO_THRESH )
   {
      
        while (!(readl(RegTouch + TSC_R_CON_STAT_OFFSET) & TSCHW_CON_STAT_FIFO_EMPTY)){
	      Fdata = readl(RegTouch + TSC_R_FIFO_ACCESS_OFFSET);
 	}

        x = (Fdata & 0x0000FFFF);
        y = (Fdata & 0xFFFF0000) >> 16;

	if ((x < gp_control_table->tscMaxX) && (y< gp_control_table->tscMaxY)){	
		if( down_counter > 3 ){		         	
              		difx = lastx - x;
			dify = lasty - y;
 			
			if(difx < 0){
				difx = -difx;
			} 	

			if(dify < 0){
				dify = -dify;
			}	

			if (difx < maxdifx && dify < maxdify){
                 		avx = (lastx + x) >>1;
				avy = (lasty + y) >>1;
                  
				*xtouch = avx;
				*ytouch = avy;

				lastx = x;
				lasty = y;

				pendown=1;
				*pd = pendown;
				penup_count=3;

				/* Clear interrupt status */
				writel( MMR1103_TSC_INTR_FIFO_THRESH, RegTouch + TSC_R_INT_STAT_OFFSET);
				return( 1 );
			} else {  	
				lastx = x;
				lasty = y;
			}
		} else  down_counter++;          
      } else {//pen up
	     lastx = 0xffff;
	     lasty = 0xffff;	
             if( pendown ){
                  if(penup_count ==0){
                      *xtouch = avx;
		      *ytouch = avy;
                      pendown=0;
		      *pd = pendown;	
		      down_counter=0;
                      /* Clear interrupt status */
                      writel( MMR1103_TSC_INTR_FIFO_THRESH, RegTouch + TSC_R_INT_STAT_OFFSET);
                      return( 1 );
                  }else penup_count--; 
	    }	
      }
      /* Clear interrupt status */
      writel( MMR1103_TSC_INTR_FIFO_THRESH, RegTouch + TSC_R_INT_STAT_OFFSET);
      return( 0 );	    
   }

   return( 0 );
}

/**
*  This function will service the pen change interrupt
*
*  @return
*     1 if the interrupt was serviced
*     0 if no interrupt was serviced
*/
/****************************************************************************/
static int bcm4760_tsc_isr_pen ( void )
{
   if( readl(RegTouch + TSC_R_INT_STAT_OFFSET) & MMR1103_TSC_INTR_PEN_CHANGE ) {
      /* Pen transition has occurred */

     /* Clear interrupt status */
      writel(  MMR1103_TSC_INTR_PEN_CHANGE, RegTouch + TSC_R_INT_STAT_OFFSET);

      return( 1 );
   }

   return( 0 );
}


/*
 * Callback function when /dev/sys/tsc/set_param read/write
 */
static int proc_do_ts_intvec_set_param (ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   /* call the default proc function to read/write g_set_param */
   rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );

   if (rc)
   {
#ifdef DEBUG
      printk("proc_dointvec return ERR:%X\n",rc,g_set_param);
#endif
      return -EINVAL;
   }

#ifdef DEBUG
   printk("rc=%x  g_set_param=%x  g_control_table.ABSxy=%x \n",rc,g_set_param,g_control_table.ABSxy);
#endif

   /* user write to /dev/sys/tsc/ */
   if ( write )
   {
      switch (g_set_param)
      {
         case 1: /* reset the parameter values to default values */
            g_control_table.sample_rate = 100;
            g_control_table.data_threshold = 1;
            g_control_table.debounce = 512*40;
            g_control_table.settling = 1024;
            g_control_table.data_point_average = 32;
            g_control_table.wire_mode = TSC_MODE_4WIRE;
            break;
         case 2: /* set the parameter values to current parameter values */
            bcm4760_tsc_set_sample_rate(g_control_table.sample_rate);
            bcm4760_tsc_set_data_threshold(g_control_table.data_threshold);
            bcm4760_tsc_set_debounce(g_control_table.debounce);
            bcm4760_tsc_set_settling(g_control_table.settling);
            bcm4760_tsc_set_data_point_average(g_control_table.data_point_average);
            bcm4760_tsc_set_mode(g_control_table.wire_mode);
            break;
         case 3:
//            printk("Update MAX,MIN  xy   ABSxy=%d\n",g_control_table.ABSxy);
            if (g_control_table.ABSxy)
            {
               g_control_table.tscMaxX = MAXx;
               g_control_table.tscMaxY = MAXy;
               g_control_table.tscMinX = 0;
               g_control_table.tscMinY = 0;

            }
            else
            {
               g_control_table.tscMaxX = PanelParamTable[panel_type].tscMaxX;
               g_control_table.tscMaxY = PanelParamTable[panel_type].tscMaxY;
               g_control_table.tscMinX = PanelParamTable[panel_type].tscMinX;
               g_control_table.tscMinY = PanelParamTable[panel_type].tscMinY;
            }

            break;
         case 0: /* display the current parameter values */

            printk( "TSC sample_rate: %u\n", g_control_table.sample_rate );
            printk( "TSC data_threshold: %u\n", g_control_table.data_threshold );
            printk( "TSC debounce: %u\n", g_control_table.debounce );
            printk( "TSC settling: %u\n", g_control_table.settling );
            printk( "TSC data_point_average: %u\n", g_control_table.data_point_average );
            printk( "TSC wire_mode: %s\n", g_control_table.wire_mode == TSC_MODE_4WIRE ? "4wire" : "5wire" );

            g_set_param = 0;
            break;
         default:
            return -EINVAL;
      }
   }

   return rc;
}

/*
 * It is called when driver is registered
 */
static int __init bcm4760_ts_probe(struct platform_device *pdev)
{
	struct bcm4760_ts_dev * bcm_ts_dev;
	struct input_dev *input_dev;
	struct resource *res;
	int err = -ENOMEM, i;

	gp_control_table	= (struct tsc_control_table *) pdev->dev.platform_data;
	BUG_ON(!gp_control_table);

	g_sysctl_tsc[1].data	= &g_control_table.sample_rate;
	g_sysctl_tsc[2].data	= &g_control_table.data_threshold;
	g_sysctl_tsc[3].data	= &g_control_table.debounce;
	g_sysctl_tsc[4].data	= &g_control_table.settling;
	g_sysctl_tsc[5].data	= &g_control_table.data_point_average;
	g_sysctl_tsc[6].data	= &g_control_table.wire_mode;
	g_sysctl_tsc[7].data	= &g_control_table.tscMaxX;
	g_sysctl_tsc[8].data	= &g_control_table.tscMinX;
	g_sysctl_tsc[9].data	= &g_control_table.tscMaxY;
	g_sysctl_tsc[10].data	= &g_control_table.tscMinY;
	g_sysctl_tsc[11].data	= &g_control_table.ABSxy;

        maxdifx = (((int)g_control_table.tscMaxX - (int)g_control_table.tscMinX) / 10);
        maxdify = (((int)g_control_table.tscMaxY - (int)g_control_table.tscMinY) / 10);

	for (i=0; i<ARRAY_SIZE(PanelParamTable); i++) {
		PanelParamTable[i].tscMinX	= g_control_table.tscMinX;
		PanelParamTable[i].tscMaxX	= g_control_table.tscMaxX;
		PanelParamTable[i].tscMinY	= g_control_table.tscMinY;
		PanelParamTable[i].tscMaxY	= g_control_table.tscMaxY;
	}

   bcm_ts_dev = kzalloc(sizeof(struct bcm4760_ts_dev), GFP_KERNEL);
   input_dev = input_allocate_device();

   if ( !bcm_ts_dev || !input_dev )
   {
      input_free_device(input_dev);
      kfree(bcm_ts_dev);
#ifdef DEBUG
      printk(">>>>>>>>> BCM4760-TSC ERROR:%X<<<<<<<<<<<<<<<<<\n",err);
#endif
      return err;
   }

   g_control_table.xres = PanelParamTable[panel_type].xres;
   g_control_table.yres = PanelParamTable[panel_type].yres;
   g_control_table.tscMaxX = PanelParamTable[panel_type].tscMaxX;
   g_control_table.tscMaxY = PanelParamTable[panel_type].tscMaxY;
   g_control_table.tscMinX = PanelParamTable[panel_type].tscMinX;
   g_control_table.tscMinY = PanelParamTable[panel_type].tscMinY;

   platform_set_drvdata(pdev, bcm_ts_dev);

   bcm_ts_dev->input = input_dev;

   /*
    * Set input device info.
    */
   input_dev->name = BCM_TSDEV_NAME;
   input_dev->phys = "bcm4760ts/input0";
   input_dev->id.bustype = BUS_HOST;
   input_dev->id.vendor = 0x0001;
   input_dev->id.product = 0x4760;
   input_dev->id.version = 0x0000;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   input_dev->dev.parent = &pdev->dev;
#else
   input_dev->cdev.dev = &pdev->dev;
//   input_dev->private = bcm_ts_dev;		// [jlh-removed] may not be needed on either kernel version
#endif

   /* Enable event bits */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
   input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#else
   input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
   input_dev->keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);
   input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);	
#endif

   if (g_control_table.ABSxy)
   {
      input_set_abs_params(input_dev, ABS_X, 0, MAXx, 0, 0);
      input_set_abs_params(input_dev, ABS_Y, 0, MAXy, 0, 0);
      input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFFFF, 0, 0);
   }
   else
   {
      input_set_abs_params(input_dev, ABS_X, 0, PanelParamTable[panel_type].xres, 0, 0);
      input_set_abs_params(input_dev, ABS_Y, 0, PanelParamTable[panel_type].yres, 0, 0);
      input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFFFF, 0, 0);
   }

   err = input_register_device(bcm_ts_dev->input);
   if (err)
	 return -1;

   /* Init the touchscreen controller block */
   bcm4760_tsc_drv_init();


#ifdef DEBUG
   printk("status: %x\n", readl(RegTouch + TSC_R_CON_STAT_OFFSET ));
   printk("get_sample_rate: %d\n", bcm4760_tsc_get_sample_rate());
   printk("get_data_threshold: %d\n", bcm4760_tsc_get_data_threshold());
   printk("get_debounce: %d\n", bcm4760_tsc_get_debounce());
   printk("get_settling: %d\n", bcm4760_tsc_get_settling());
   printk("get_data_point_average: %d\n", bcm4760_tsc_get_data_point_average());
   printk("get_mode: %d\n", bcm4760_tsc_get_mode());
   printk("get_mask: %d\n", readl(RegTouch + TSC_R_INT_MASK_OFFSET ));
#endif

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		err = -ENODEV;
		printk(KERN_ERR 
			"bcm4760_ts: platform_get_resource IORESOURCE_IRQ error.\n");
		input_free_device(input_dev);
		kfree(bcm_ts_dev);
	}

	bcm_ts_dev->irq = res->start;




   /* Install interrupt handler */
   if (request_irq( bcm_ts_dev->irq , bcm4760_ts_isr, 0, ISR_NAME, bcm_ts_dev ))
   {
#ifdef DEBUG
      	printk(KERN_ERR "bcm4760_ts.c: Can't allocate irq %d\n", bcm_ts_dev->irq );
#endif
      err = -EBUSY;
      input_free_device(input_dev);
      kfree(bcm_ts_dev);
#ifdef DEBUG
      printk(">>>>>>>>> BCM4760-TSC ERROR:%X<<<<<<<<<<<<<<<<<\n",err);
#endif
      return err;
   }

   return 0;

}

/*
 * It is called when the device is removed, i.e. module unloaded
 */
static int bcm4760_ts_remove(struct platform_device *pdev)
{
   struct bcm4760_ts_dev * bcm_ts_dev = platform_get_drvdata(pdev);

   bcm4760_tsc_drv_exit();
   free_irq( bcm_ts_dev->irq , bcm_ts_dev );
   input_unregister_device(bcm_ts_dev->input);
   kfree(bcm_ts_dev);

   return 0;
}

/**********************************************************
 * The interrupt handler for the touchscreen controller
 *********************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t bcm4760_ts_isr( int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t bcm4760_ts_isr( int irq, void *dev_id )
#endif
{
   unsigned long flags;
   uint32_t xx,yy,pendown=0;

   struct bcm4760_ts_dev *bcm_ts_dev = dev_id;

   spin_lock_irqsave(&lock, flags);

#ifdef DEBUG
   printk("FROM 4760 TSC INT\n");
#endif

   if ( !bcm4760_tsc_is_irq() )
   {
#ifdef DEBUG
      printk("Non-handled interrupt fired in TSC ISR!\n");
#endif
      spin_unlock_irqrestore(&lock, flags);
      return IRQ_NONE;
   }

   bcm_ts_dev->data_available = FALSE;
   bcm_ts_dev->pen_transition = FALSE;

   if (bcm4760_tsc_isr_data (&xx, &yy, &pendown ))		// get tsc x,y coorditation
   {
#ifdef DEBUG
      printk("MaxX:%X  MaxY:%X  MinX:%X  MinY:%X\n",g_control_table.tscMaxX, g_control_table.tscMaxY, g_control_table.tscMinX, g_control_table.tscMinY);
#endif
      if (g_control_table.ABSxy)
      {

         // make abs coordinates
         bcm_ts_dev->touch_data.yData = (MAXy - yy) / 16;		//12 bit only		
         bcm_ts_dev->touch_data.xData = (MAXx - xx) / 16;		//12 bit only		
      }
      else
      {
         // make screen coordinates
         bcm_ts_dev->touch_data.yData = ((g_control_table.tscMaxY - yy) * g_control_table.yres) / (g_control_table.tscMaxY - g_control_table.tscMinY);
         bcm_ts_dev->touch_data.xData = ((g_control_table.tscMaxX - xx) * g_control_table.xres) / (g_control_table.tscMaxX - g_control_table.tscMinX);
      }
      bcm_ts_dev->data_available = TRUE;
   }


   if ( bcm4760_tsc_isr_pen ( ) )
   {
      /* Set flag indicating that pen transition occurred */
      bcm_ts_dev->pen_transition = TRUE;
   }

   /* Check the pen status */
   if ( pendown )
   {
      bcm_ts_dev->touch_data.state = TSC_TOUCH_DOWN;       /* touch down event */
      /* Artifical pressure data */
      bcm_ts_dev->touch_data.pressure = 0x8888;
   } else
   {
      bcm_ts_dev->touch_data.state = TSC_TOUCH_UP;         /* touch up event */
      bcm_ts_dev->touch_data.pressure = 0;
   }


#ifdef DEBUG
   printk("x: %x y: %x pen %s\n", bcm_ts_dev->touch_data.xData, bcm_ts_dev->touch_data.yData, (bcm_ts_dev->touch_data.state == TSC_TOUCH_UP) ? "Up" : "Down");
#endif

   /* Report data to input layer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
   input_regs(bcm_ts_dev->input, regs);
#endif
   if ((bcm_ts_dev->touch_data.state == TSC_TOUCH_DOWN) && ( bcm_ts_dev->data_available ))
   {
      input_report_abs(bcm_ts_dev->input, ABS_X, bcm_ts_dev->touch_data.xData);
      input_report_abs(bcm_ts_dev->input, ABS_Y, bcm_ts_dev->touch_data.yData);
      input_report_abs(bcm_ts_dev->input, ABS_PRESSURE, bcm_ts_dev->touch_data.pressure);
      input_report_key(bcm_ts_dev->input, BTN_TOUCH, 1 );
   }
   else if (bcm_ts_dev->touch_data.state == TSC_TOUCH_UP)
   {
      bcm_ts_dev->data_available = FALSE;
      input_report_abs(bcm_ts_dev->input, ABS_X, bcm_ts_dev->touch_data.xData);
      input_report_abs(bcm_ts_dev->input, ABS_Y, bcm_ts_dev->touch_data.yData);
      input_report_abs(bcm_ts_dev->input, ABS_PRESSURE, 0);
      input_report_key(bcm_ts_dev->input, BTN_TOUCH, 0 );    
   }
   else
   {
      spin_unlock_irqrestore(&lock, flags);
      return IRQ_HANDLED;
   }

   input_sync(bcm_ts_dev->input);
   spin_unlock_irqrestore(&lock, flags);
   return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int bcm4760_ts_suspend(struct platform_device *pdev,
                              pm_message_t state)
{
   return 0;
}

static int bcm4760_ts_resume(struct platform_device *pdev)
{
   /* Init the touchscreen controller block */
   bcm4760_tsc_drv_init();

   return 0;
}
#else
#define bcm4760_ts_suspend    NULL
#define bcm4760_ts_resume     NULL
#endif

static struct platform_driver bcm4760_ts_driver =
{
   .driver  =
   {
      .name    = BCM_TSDEV_NAME,
   },
   .probe   = bcm4760_ts_probe,
   .remove  = bcm4760_ts_remove,
   .suspend = bcm4760_ts_suspend,
   .resume  = bcm4760_ts_resume,
};

/*
 * The functions for inserting/removing us as a module.
 */
static int __devinit bcm4760_ts_init(void)
{
   int rc;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   /* Register sysctl table */
   g_sysctl_header = register_sysctl_table( g_sysctl, 0 );
   if ( g_sysctl_header != NULL )
   {
      g_sysctl_header->ctl_table->child->de->owner = THIS_MODULE;
   }
#else
   g_sysctl_header = register_sysctl_table( g_sysctl );
#endif
	BUG_ON(!g_sysctl_header);

	rc = platform_driver_register(&bcm4760_ts_driver);

   return rc;
}

module_init( bcm4760_ts_init );

#if defined( MODULE )

static void __exit bcm4760_ts_exit(void)
{
   if ( g_sysctl_header != NULL )
   {
      unregister_sysctl_table( g_sysctl_header );
   }

	platform_driver_unregister(&bcm4760_ts_driver);
}

module_exit( bcm4760_ts_exit );
#endif


MODULE_AUTHOR("Broadcom Corporation.");
MODULE_DESCRIPTION( BCM4760_TS_MODULE_DESC );
MODULE_VERSION( BCM4760_TS_MODULE_VERSION );
MODULE_LICENSE( "GPL" );
