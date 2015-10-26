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
 * Description: Public header of the BCM framebuffer driver
 */

#ifndef BCM_FB_H_
#define BCM_FB_H_

#include <linux/ioctl.h>
#include <linux/fb.h>
#include <linux/broadcom/dd/dd.h>

#define BCM_FB_MAGIC    'B'

#define BCM_FB_CMD_GET_GUI_IMAGE    0x10

#define BCM_FB_IOCTL_GET_GUI_IMAGE  _IOR( BCM_FB_MAGIC, BCM_FB_CMD_GET_GUI_IMAGE, DD_IMAGE_T )

#if defined( __KERNEL__ )
/*
 * BCM framebuffer data structure
 */
struct bcm_fb {
   /* Linux framebuffer info */
   struct fb_info info;

   /* LCD settings/configurations */
   DD_LCD_INFO_T lcd_info;

   /* GUI image that maps to the framebuffer in user space */
   DD_IMAGE_T gui_img;

   /*
    * Buffered image that contains the "buffered" bitmap of the GUI. This 
    * image is used by the Display Director and the user needs to call a
    * Display Director API to "lock" it before touching it
    */
   DD_IMAGE_T buffer_img;

   DD_UPDATE_HANDLE_T update_handle;

   /* element handle of the buffered image */
   DD_ELEMENT_HANDLE_T element_handle;
   
   /* color palette */
   uint32_t palette[16];
};

#endif

#endif /* BCM_FB_H_ */
