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
 * Description: Header of the Display Director utilities. This header should
 * only be used by Display Director and its plug-ins
 */

#ifndef DD_UTIL_H
#define DD_UTIL_H

#include <linux/broadcom/dd/dd.h>

/*
 * Perform bit-exact comparison of the 'src' array and the 'dst' array up to
 * the length of 'len' bytes. The optional 'skip_cnt' parameter specifies the
 * number of iterations to skip during the comparison. It can be used to
 * reduce the CPU load
 */
extern DD_STATUS_T dd_util_data_validate(void *src, void *dst,
      uint32_t len, unsigned int skip_cnt);

/*
 * This routine calculates and returns the corresponding bytes per pixel based
 * on the color format supplied by the caller
 */
extern uint32_t dd_util_pixel_bytes_calc(DD_FORMAT_T format);

#endif /* DD_UTIL_H */
