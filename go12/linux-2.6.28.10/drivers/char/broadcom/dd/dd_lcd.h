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
 * Description: Header of the Display Director LCD sub module
 */


#ifndef DD_LCD_H
#define DD_LCD_H

#include <linux/broadcom/dd/dd.h>
#include <csp/clcdHw.h>

#define DD_CLD_MAX_NUM_PANEL 4 /* max number of panels supported */

/*
 * LCD screen type definition
 */
typedef enum dd_lcd_type {
   DD_LCD_TYPE_TFT = CLCDHW_LCDTFT,
   DD_LCD_TYPE_STN = CLCDHW_LCDSTNCOLOR,
   DD_LCD_TYPE_INVALID
} DD_LCD_TYPE_T;

/*
 * LCD panel configurations
 */
typedef struct dd_lcd_panel {
   DD_LCD_MODEL_T model; /* panel model */
   const char *name; /* panel name */
   uint32_t refresh; /* refresh rate in Hz */
	uint32_t xres; /* x resolution in pixels */
	uint32_t yres; /* y resolution in pixels */
	uint32_t pixel_clk; /* pixel clock */
	uint32_t left_margin; /* horizontal back porch */
	uint32_t right_margin; /* horizontal front porch */
	uint32_t upper_margin; /* vertical back porch */
	uint32_t lower_margin; /* vertical front porch */
	uint32_t hsync_len; /* horizontal pulse width */
	uint32_t vsync_len; /* vertical pulse width */
   uint32_t pixel_clk_div; /* pixel clock divider */
   uint32_t inv_out_enable; /* invert output enable */
   uint32_t inv_pixel_clk; /* invert pixel clock */
   uint32_t inv_hsync; /* invert horizontal sync */
   uint32_t inv_vsync; /* invert certical sync */
   DD_LCD_TYPE_T type; /* panel type */
   uint32_t bpp; /* bits per pixel */
} DD_LCD_PANEL_T;

/*
 * Initialize the LCD, load the corresponding panel parameters based on
 * 'model' supplied by the caller, reserve its GPIO pins, and register the
 * LCD ISR
 *
 * This routine needs to be called once (and only once) before LCD can
 * be used
 */
extern DD_STATUS_T dd_lcd_init(DD_LCD_MODEL_T model);

/*
 * Terminate the LCD
 */
extern DD_STATUS_T dd_lcd_term(void);

/*
 * Enable the LCD and power up the panel
 */
extern DD_STATUS_T dd_lcd_panel_enable(void);

/*
 * Disable the LCD and power down the panel
 */
extern DD_STATUS_T dd_lcd_panel_disable(void);

/*
 * Reset the LCD panel by reloading all the LCD parameters. This routine
 * should be called before 'dd_lcd_enable'
 */
extern DD_STATUS_T dd_lcd_panel_reset(void);

/*
 * Update the LCD. Load the bitmap of a buffer of starting address 'addr' to
 * the LCD. This routine blocks wait until the update finishes
 */
extern DD_STATUS_T dd_lcd_update(uint32_t addr);

/*
 * Obtain informations about the LCD including width, height, bpp, pitch, and
 * required bitmap buffer size in bytes
 */
extern void dd_lcd_info_get(DD_LCD_INFO_T *info);

#endif /* DD_LCD_H */
