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
 
/*
 * Description: Header of the Display Director mislleneous functions
 */

#ifndef DD_MISC_H
#define DD_MISC_H

#include <linux/broadcom/dd/dd.h>

#if DD_VIDEO_DEMO
typedef struct
{
   int xsize;        // max x dimension that rectangle can move  e.g. 800-352 = 448 for cif on wvga
   int ysize;        // max t dimension that rectangle can move  e.g. 480-288 = 192 for cif on wvga
   int xstep;        // number of pixels to step in x direction (more is faster but more grainy)
   int nregions;     // number of times the rectangle will bounce on the wvga display
   int regionsize;   // x size of each bounce region
   int curregion;    // current region
   int curx;         // current x value (output to caller for object coordinates)
   int cury;         // current y value (output to caller for object coordinates)
   int curysize;     // current attenuated value of ysize (each bounce gets lower)
   int k;            // parabola focus
   int base;         // base x value of each region
   int plarge;       // p coefficient of parabola
   int initialized;
} BOUNCE_STATE;
#endif

#if DD_VIDEO_DEMO
/*
 * CIF = 352x288 QCIF = 176X220 QVGA = 320x240 VGA = 640x480, WVGA=800x480
 * Round the corners of a rectangular 32-bit RBG bitmap
 */
extern int round_corners(uint32_t *bufp, uint16_t width, uint16_t height,
      uint16_t radius);
#endif

#if DD_VIDEO_DEMO
// Initialize the bounce logic.
void bounce_init(BOUNCE_STATE *statep, int xsize, int ysize, int xstep, int nregions);
#endif

#if DD_VIDEO_DEMO
// Get the next pair of curx/cury coordinates. This can be called continuously and will
// just wrap back to the beginning when necessary.
int bounce_next(BOUNCE_STATE *statep);
#endif

#endif /* DD_MISC_H */
