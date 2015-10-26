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
 * Description: Private header of the Display Director driver
 */

#ifndef DD_PRIV_H
#define DD_PRIV_H

#include <linux/broadcom/dd/dd.h>

/*
 * Return pointer to the head of the elements in the display list
 */
extern DD_ELEMENT_T *dd_element_display_get(void);

#if DD_VIDEO_DEMO
extern DD_STATUS_T dd_video_move_start(unsigned int xsize, unsigned int ysize,
      unsigned int xstep, unsigned int nregions);
#endif

#if DD_VIDEO_DEMO
void dd_video_move_stop(void);
#endif

#endif /* DD_PRIV_H */
