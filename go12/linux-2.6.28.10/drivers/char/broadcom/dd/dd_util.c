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
 * Description: Display Director utilities
 */

#include "dd_util.h"

DD_STATUS_T dd_util_data_validate(void *src, void *dst,
      uint32_t len, unsigned int skip_cnt)
{
   uint32_t i;
   uint32_t *src_32;
   uint32_t *dst_32;
   unsigned int incr = skip_cnt + 1;

   len /= 4;

   src_32 = (uint32_t *)src;
   dst_32 = (uint32_t *)dst;

   for (i = 0; i < len; i += incr) {
      if (src_32[i] != dst_32[i])
         return DD_FAIL;
   }
   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_util_data_validate);

uint32_t dd_util_pixel_bytes_calc(DD_FORMAT_T format)
{
   switch (format) {
      case DD_FORMAT_ARGB888:
      case DD_FORMAT_URGB888:
         return 4;

      default:
         return 4;
   }
}
EXPORT_SYMBOL(dd_util_pixel_bytes_calc);
