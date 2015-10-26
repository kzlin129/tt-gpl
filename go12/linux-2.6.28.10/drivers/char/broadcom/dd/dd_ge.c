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
 * Description: The Display Director generic APIs for the Graphic Engine (GE)
 */

#include <linux/dma-mapping.h>

#include "dd_ge.h"
#include "dd_util.h"

static DD_GE_FUNCS_T ge_funcp;

static DD_STATUS_T sw_copy_area(DD_GE_PARAM_T *param)
{
   uint8_t *src, *dst;
   uint32_t bytes_per_line, bytes_per_pixel;
   uint32_t sx, sy, dx, dy;
   uint32_t width, height;
   uint32_t row;

   /* the source and destination formats have to match */
   if (param->src_format != param->dst_format)
      return DD_FAIL;
   if (param->src_pitch != param->dst_pitch)
      return DD_FAIL;

   bytes_per_pixel = dd_util_pixel_bytes_calc(param->src_format);
   bytes_per_line = param->src_pitch;

   src = (uint8_t *)dma_to_virt(NULL, param->src_addr);
   dst = (uint8_t *)dma_to_virt(NULL, param->dst_addr);

   sx = param->sx;
   sy = param->sy;
   dx = param->dx;
   dy = param->dy;
   width = param->width;
   height = param->height;

   if (sy > dy) {
      /* copy from top to bottom */
      src += sy * bytes_per_line + sx * bytes_per_pixel;
      dst += dy * bytes_per_line + dx * bytes_per_pixel;

      for (row = sy; row < sy + height; row++) {
         memcpy(dst, src, width * bytes_per_pixel);
         src += bytes_per_line;
         dst += bytes_per_line;
      }
   } else if (sy < dy) {
      /* copy from bottom to top */
      src += (sy + height - 1) * bytes_per_line + sx * bytes_per_pixel;
      dst += (dy + height - 1) * bytes_per_line + dx * bytes_per_pixel;

      for (row = sy + height; row > sy; row--) {
         memcpy(dst, src, width * bytes_per_pixel);
         src -= bytes_per_line;
         dst -= bytes_per_line;
      }
   } else { /* sy == dy */
      /* move on the same horizontal line */
      src += sy * bytes_per_line + sx * bytes_per_pixel;
      dst += dy * bytes_per_line + dx * bytes_per_pixel;

      for (row = sy; row < sy + height; row++) {
         memmove(dst, src, width * bytes_per_pixel);
         src += bytes_per_line;
         dst += bytes_per_line;
      }
   }
   return DD_SUCCESS;
}

static DD_STATUS_T sw_fill_color(DD_GE_PARAM_T *param)
{
   uint8_t *img_base;
   uint16_t *img_16;
   uint32_t *img_32;
   uint32_t bytes_per_line, bytes_per_pixel, pixels_per_line;
   uint32_t dx, dy;
   uint32_t width, height;
   uint32_t row, col;
   uint32_t color;

   img_base = (uint8_t *)dma_to_virt(NULL, param->dst_addr);
   bytes_per_pixel = dd_util_pixel_bytes_calc(param->dst_format);
   bytes_per_line = param->dst_pitch;
   pixels_per_line = bytes_per_line / bytes_per_pixel;
   dx = param->dx;
   dy = param->dy;
   width = param->width;
   height = param->height;
   color = param->option.color;
   
   switch (bytes_per_pixel) {
      case 2:
         for (row = dy; row < dy + height; row++) {
            img_16 = row * pixels_per_line + dx + (uint16_t *)img_base;
            for (col = dx; col < dx + width; col++) {
               *img_16 = color;
               img_16++;
            }
         }
         break;

      case 4:
         for (row = dy; row < dy + height; row++) {
            img_32 = row * pixels_per_line + dx + (uint32_t *)img_base;
            for (col = dx; col < dx + width; col++) {
               *img_32 = color;
               img_32++;
            }
         }
         break;

      default:
         break;
   }
   return DD_SUCCESS;
}

void dd_ge_register(DD_GE_FUNCS_T *ge_funcs)
{
   ge_funcp.dd_ge_alpha_blend = ge_funcs->dd_ge_alpha_blend;
   ge_funcp.dd_ge_copy_area = ge_funcs->dd_ge_copy_area;
   ge_funcp.dd_ge_fill_color = ge_funcs->dd_ge_fill_color;
}
EXPORT_SYMBOL(dd_ge_register);

void dd_ge_unregister(void)
{
   ge_funcp.dd_ge_alpha_blend = NULL;
   ge_funcp.dd_ge_copy_area = NULL;
   ge_funcp.dd_ge_fill_color = NULL;
}
EXPORT_SYMBOL(dd_ge_unregister);

DD_STATUS_T dd_ge_alphablend(DD_GE_PARAM_T *param)
{
   if (ge_funcp.dd_ge_alpha_blend)
      return ge_funcp.dd_ge_alpha_blend(param);
   else
      return DD_FAIL;
}

/*
 * TODO: In the future the SW copyarea and fillcolor can be moved to another
 * GE module that runs on a platform w/o the Graphic Engine
 */

DD_STATUS_T dd_ge_copyarea(DD_GE_PARAM_T *param)
{
   if (ge_funcp.dd_ge_copy_area)
      return ge_funcp.dd_ge_copy_area(param);
   else
      return sw_copy_area(param);
}

DD_STATUS_T dd_ge_fillcolor(DD_GE_PARAM_T *param)
{
   if (ge_funcp.dd_ge_fill_color)
      return ge_funcp.dd_ge_fill_color(param);
   else
      return sw_fill_color(param);
}
