/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  ts.h
*
*  PURPOSE:
*
*  Definitions for the touchscreen driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_TS_H )
#define LINUX_TS_H


/**
*  The touch screen state enumeration is used report whether the user
*  is currently touching the screen (down) or not (up)
*/
typedef enum
{
   TSC_TOUCH_DOWN = 0,     /**< Down state/event */
   TSC_TOUCH_UP            /**< Up state/event */
} TSC_TOUCH;

/**
*  The touch screen event structure is used to report a touch screen event.  When an
*  event occurs, the x and y coordinates are reported, along with the touch
*  screen state.  The pixel at x=0, y=0 is located at top left corner of the
*  LCD.
*/
typedef struct
{
   int xData;                  /**< Event x coordinate */
   int yData;                  /**< Event y coordinate */
   int pressure;               /**< Pressure (n/a)     */
   TSC_TOUCH state;            /**< Touch screen state */
} TSC_EVENT;

typedef enum tsc_wire_mode {
   TSC_MODE_4WIRE = 0,
   TSC_MODE_5WIRE
} tsc_wire_mode;

typedef struct tsc_control_table {
   int sample_rate;
   int data_threshold;
   int debounce;
   int settling;
   int data_point_average;
   tsc_wire_mode wire_mode;
#if defined( CONFIG_BCM4760_TOUCHSCREEN ) || defined( CONFIG_BCM4760_TOUCHSCREEN_MODULE )
   unsigned int xres;
   unsigned int yres; 
   unsigned int tscMaxX;			// max X touch value
   unsigned int tscMaxY;			// max Y touch value
   unsigned int tscMinX;			// min X touch value
   unsigned int tscMinY;			// min Y touch value
   unsigned int ABSxy;				// return abs not LCD x,y
#endif
} tsc_control_table;

#endif
