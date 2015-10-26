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
 * Description: Display Director mislleneous stuff
 */

#include "dd_misc.h"

#if DD_VIDEO_DEMO
static inline int square(int x)
{
   return x*x;
}
#endif

#if DD_VIDEO_DEMO
/*
 * CIF = 352x288 QCIF = 176X220 QVGA = 320x240 VGA = 640x480, WVGA=800x480
 * Round the corners of a rectangular 32-bit RBG bitmap
 */
int round_corners(uint32_t *bufp, uint16_t width, uint16_t height, uint16_t radius)
{
   unsigned int i, pixel_len;
   int x,y;
   if ((width == 0) || (height == 0))
   {
      return -1;
   }
   if ((radius > width/2) || (radius > height/2))
   {
      return -1;
   }

   pixel_len = width * height;
   for (i = 0; i < pixel_len; i++) {
      bufp[i] &= 0x00FFFFFF;
   }

   // For each pixel, check if it is within the circle. Run this algorithm
   // for the to left corner, and then reflect the result to the other 3 corners.
   for (y=0; y<radius; y++)
   {
      if (y > radius)
      {
         break;   // only check top left corner within radius distance of y direction
      }
      for (x=0; x<radius; x++)
      {
         int deltax = radius - x; // x distance from pixel to center of rounding circle
         int deltay = radius - y; // x distance from pixel to center of rounding circle

         if (x > radius)
         {
            break;   // only check top left corner within radius distance of x dimension
         }
         if ((deltax > 0) && (deltay > 0))
         {
            // We are somewhere left or above the rounding circle center.
            // Check if we are beyond the radius or not.
            if (square(deltax)+square(deltay) >= square(radius))
            {
               // We are beyond the radius, make the pixel opaque
               bufp[y*width + x] |= 0xff000000;

               // Set other 3 corresponding coordinates to opaque as well
               bufp[y*width + (width-1-x)] |= 0xff000000;
               bufp[(height-1-y)*width + x] |= 0xff000000;
               bufp[(height-1-y)*width + (width-1-x)] |= 0xff000000;
            }
         }
      }
   }
   return 0;
}
#endif

#if DD_VIDEO_DEMO
// Initialize the bounce logic.
void bounce_init(BOUNCE_STATE *statep, int xsize, int ysize, int xstep, int nregions)
{
   statep->xsize = xsize;
   statep->ysize = ysize;
   statep->xstep = xstep;
   statep->nregions = nregions;
   statep->regionsize = xsize / nregions;
   statep->curregion = 0;
   statep->curx = 0;
   statep->cury = ysize-1;
   statep->base = statep->curregion*statep->regionsize;
   statep->curysize = statep->ysize;
   statep->k = statep->base + statep->regionsize/2;
   statep->plarge = 1000*(statep->regionsize/2)*(statep->regionsize/2)/4/(statep->curysize-1);
   statep->initialized = 1;
}
#endif

#if DD_VIDEO_DEMO
// Returns next coordinates in statep->curx and statep->cury
int bounce_next(BOUNCE_STATE *statep)
{
   if (!statep->initialized)
   {
      return -1;
   }
   // This is the current y height for the x step
   statep->cury = 1000*(statep->curx-statep->k)*(statep->curx-statep->k)/4/statep->plarge;

   // Add an offset so that the bases of the parabolas are on the same line. The y offset is actually negative
   // from normal coordinates. +y goes down...
   statep->cury += (statep->ysize - statep->curysize);

   statep->curx += statep->xstep;
   if (statep->curx >= statep->regionsize + statep->base)
   {
      if (++statep->curregion >= statep->nregions)
      {
         // restart from the left margin
         bounce_init(statep, statep->xsize, statep->ysize, statep->xstep, statep->nregions);
      }
      else
      {
         // initialize next region
         statep->base = statep->curregion*statep->regionsize;  // x base of next region
         statep->k = statep->base + statep->regionsize/2;      // x center of next region
         statep->curysize = statep->curysize*2/3;              // attenuate each bounce
         statep->plarge = 1000*(statep->regionsize/2)*(statep->regionsize/2)/4/(statep->curysize-1);  // scaled p parameter
      }
   }
   return 0;
}
#endif
