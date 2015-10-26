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




#ifndef VDM_PARAMS_H
#define VDM_PARAMS_H

// All the settings below may change depending on the nature of the display
// that is connected. The available parameters are:

// VC_DISPMAN_SIZE_X - horizontal size of the display, in client coordinate frame, in pixels.
// VC_DISPMAN_SIZE_Y - vertical size of the display, in client coordinate frame, in pixels.
// VC_DISPMAN_MAX_SIZE - should be set to the larger of VC_DISPMAN_SIZE_X and VC_DISPMAN_SIZE_Y.
// VC_DISPMAN_LOCAL_DISPLAY_DIFFERENT - set if display is driven from a secondary frame buffer.
// VC_DISPMAN_LOCAL_SIZE_X - size of local display, if VC_DISPMAN_LOCAL_DISPLAY_DIFFERENT is set.
// VC_DISPMAN_LOCAL_SIZE_Y - size of local display, if VC_DISPMAN_LOCAL_DISPLAY_DIFFERENT is set.
// VC_DISPMAN_MPU_LCD - display is to be driven in MPU rather than DMA mode.
// VC_DISPMAN_24BPP - use 24bpp rather than 16bpp graphics throughout the display manager.
// VC_DISPMAN_EXT_FRAME_BUFFER - external frame buffer to be used (only for MPU display).

// To make things convenient the settings for a variety of known displays are
// given below. Exactly ONE primary LCD should be selected, and ONE or ZERO secondary LCD.

// Primary LCD:

#ifdef VC_DISPMAN_PSP
#define VC_DISPMAN_SIZE_X 272
#define VC_DISPMAN_SIZE_Y 480
#define VC_DISPMAN_MAX_SIZE 480
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_SHARP_LQ057Q3DC02_LANDSCAPE
#define VC_DISPMAN_SIZE_X 320
#define VC_DISPMAN_SIZE_Y 240
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_SHARP_LQ057Q3DC02
#define VC_DISPMAN_SIZE_X 320
#define VC_DISPMAN_SIZE_Y 240
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_SHARP_LQ080V3DG01
#define VC_DISPMAN_SIZE_X 640
#define VC_DISPMAN_SIZE_Y 480
#define VC_DISPMAN_MAX_SIZE 640
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_SHARP_LQ080V3DG01
#define VC_DISPMAN_SIZE_X 640
#define VC_DISPMAN_SIZE_Y 480
#define VC_DISPMAN_MAX_SIZE 640
#define VC_DISPMAN_NUM_DISPLAYS 1
#if 0
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif
#endif

#ifdef VC_DISPMAN_RITDISPLAYCORP_OLED
#define VC_DISPMAN_SIZE_X 96
#define VC_DISPMAN_SIZE_Y 64
#define VC_DISPMAN_MAX_SIZE 96
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_TOSHIBA_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_TOSHIBA_LCD_2
#define VC_DISPMAN_SIZE_X 320
#define VC_DISPMAN_SIZE_Y 240
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_NUM_DISPLAYS 1
#define VC_DISPMAN_NO_ROTATION
#endif /* VC_DISPMAN_TOSHIBA_LCD_2 */

#ifdef VC_DISPMAN_SONY_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

#ifdef VC_DISPMAN_AM_LCD
#error "VC_DISPMAN_AM_LCD no longer supported"
#endif

#ifdef VC_DISPMAN_EPSON_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif    // of #ifdef VC_DISPMAN_EPSON_LCD

#ifdef VC_DISPMAN_CASIO_RGB_LCD
#define VC_DISPMAN_SIZE_X 320
#define VC_DISPMAN_SIZE_Y 240
#define VC_DISPMAN_MAX_SIZE 240
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
//#define VC_DISPMAN_DMA_32BIT
//#define VC_DISPMAN_LCD_PIXEL_FORMAT VC_IMAGE_RGB666
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif    // of #ifdef VC_DISPMAN_EPSON_RGB_LCD




#ifdef VC_DISPMAN_EPSON_50297_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_BYTESWAP16
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_EPSON_50300_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_TOMATO_TL1766_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_TOSHIBA_MPU_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_HYUNDAI_MPU_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_HYUNDAI_MPU_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_NEC_MPU_LCD
#define VC_DISPMAN_SIZE_X 128
#define VC_DISPMAN_SIZE_Y 160
#define VC_DISPMAN_MAX_SIZE 160
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_RENESAS_MPU_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define VC_DISPMAN_16BPP
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif

#ifdef VC_DISPMAN_L2F50304_MPU_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w)   (w)
#endif



#ifdef VC_DISPMAN_AM_MPU_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_16BPP_READ
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) ((x) & ~3)
#define MPU_LCD_PAD_W(w)   (((w)+3) & ~3)
#endif /* VC_DISPMAN_AM_MPU_LCD */

#ifdef VC_DISPMAN_HD66793_LCD
#define VC_DISPMAN_SIZE_X 176
#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) ((x) & ~3)
#define MPU_LCD_PAD_W(w)   (((w)+3) & ~3)
#endif

#ifdef VC_DISPMAN_S6D0110A_LCD
#define VC_DISPMAN_SIZE_X 128
#define VC_DISPMAN_SIZE_Y 160
#define VC_DISPMAN_MAX_SIZE 160
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP (for now assume 16bit)
//#define VC_DISPMAN_16BPP_READ
#define VC_DISPMAN_EXT_FRAME_BUFFER
// Currently do not insert padding for high speed mode as the driver chip has a bug
//#define MPU_LCD_ALIGN_X(x) ((x) & ~3)
//#define MPU_LCD_PAD_W(w)   (((w)+3) & ~3)
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w) (w)
#define VC_DISPMAN_DMA_32BIT
#define VC_DISPMAN_LCD_PIXEL_FORMAT VC_IMAGE_RGB2X9
#endif


#ifdef VC_DISPMAN_HD66787_LCD
#define VC_DISPMAN_SIZE_X 176

#define VC_DISPMAN_SIZE_Y 220
#define VC_DISPMAN_MAX_SIZE 220
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_16BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define MPU_LCD_ALIGN_X(x) (x)
// ((x) & ~2)
#define MPU_LCD_PAD_W(w) (w)
//   (((w)+2) & ~2)
#endif


#ifdef VC_DISPMAN_HD66781_LCD
#define VC_DISPMAN_SIZE_X 240
#define VC_DISPMAN_SIZE_Y 320
#define VC_DISPMAN_MAX_SIZE 320
#define VC_DISPMAN_MPU_LCD
#define VC_DISPMAN_24BPP
#define VC_DISPMAN_EXT_FRAME_BUFFER
#define VC_DISPMAN_DMA_32BIT
#define VC_DISPMAN_LCD_PIXEL_FORMAT VC_IMAGE_RGB666
#define MPU_LCD_ALIGN_X(x) (x)
#define MPU_LCD_PAD_W(w) (w)
#endif


// Secondary LCD:

#ifdef VC_DISPMAN_LEADIS_2ND_LCD
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 96
#define VC_DISPMAN_SECOND_SIZE_Y 64

#elif defined(VC_DISPMAN_EPSON_2ND_LCD)
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 128
#define VC_DISPMAN_SECOND_SIZE_Y 128

#elif defined(VC_DISPMAN_AM_2ND_LCD)
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 128
#define VC_DISPMAN_SECOND_SIZE_Y 96

#elif defined(VC_DISPMAN_SSD1770_2ND_LCD)
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 96
#define VC_DISPMAN_SECOND_SIZE_Y 64


#elif defined(VC_DISPMAN_HD66773_2ND_LCD)
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 128
#define VC_DISPMAN_SECOND_SIZE_Y 96

#elif defined(VC_DISPMAN_HD66773_BIG_LCD)
#define VC_DISPMAN_NUM_DISPLAYS 2
#define VC_DISPMAN_SECOND_SIZE_X 132
#define VC_DISPMAN_SECOND_SIZE_Y 176


#else
#define VC_DISPMAN_NUM_DISPLAYS 1
#endif

// Finally some general settings and parameters that should not be changed.

// Maximum allowed number of VDM clients.
#define VC_DISPMAN_NUM_CLIENTS 8

// Cursor size (applies to both X and Y).
#define VC_DISPMAN_CURSOR_SIZE 16

#endif
