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




#ifndef DISPMAN_EX_H
#define DISPMAN_EX_H

/* Draw pixel data directly to an image */
int vc_dispman_draw_pixels(uint32_t dest, uint32_t d_x_offset, uint32_t d_y_offset, uint32_t width, uint32_t height, void *data, int stride, int format);

/* BLT between two displays */
int vc_dispman_blt2(uint32_t dest, uint32_t d_x_offset, uint32_t d_y_offset, uint32_t width, uint32_t height, uint32_t src, uint32_t s_x_offset, uint32_t s_y_offset, VC_DISPMAN_TRANSFORM_T transform);

/* Create an off-screen image */
int vc_dispman_create_image(uint32_t w, uint32_t h, uint8_t format, uint32_t *pHandle);

/* Destroy a previously-created off-screen image */
int vc_dispman_destroy_image(uint32_t handle);

/* Synchronise with VideoCore */
int vc_dispman_sync();

/* Extended text command. */
int vc_dispman_text2(uint32_t display_num, uint32_t f_x_offset, uint32_t f_y_offset, int anchor, int font, char *text, uint32_t fg_colour, uint32_t bg_colour);

#endif
