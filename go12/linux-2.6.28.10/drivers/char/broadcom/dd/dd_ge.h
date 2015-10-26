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
 * Description: Public header that contains the Display Director generic APIs
 * for the Graphic Engine (GE). This header should only be used by Display
 * Director and its plug-ins
 */

#ifndef DD_GE_H
#define DD_GE_H

#include <linux/broadcom/dd/dd.h>

/*
 * Define various GE operations including alpha blending, copy area, fill
 * color, etc.
 */
typedef enum dd_ge_operation {
   DD_GE_OP_ALPHA_BLEND = 0,
   DD_GE_OP_COPY_AREA,
   DD_GE_OP_FILL_COLOR,
   DD_GE_OP_INVALID
} DD_GE_OPERATION_T;

/*
 * GE parameters
 */
typedef struct dd_ge_param {
   DD_FORMAT_T src_format;
   DD_FORMAT_T dst_format;
   uint32_t sx; /* x offset to the src starting point (address) */
   uint32_t sy; /* y offset to the src starting point (address) */
   uint32_t dx; /* x offset to the dst starting point (address) */
   uint32_t dy; /* y offset to the dst starting point (address) */
   uint32_t width; /* in pixels */
   uint32_t height; /* in pixels */
   uint32_t src_pitch; /* bytes per line */
   uint32_t dst_pitch;
   uint32_t src_addr; /* physical source address */
   uint32_t dst_addr; /* physical destination address */
   union {
      uint32_t color; /* pixel information for fill color */
      u8 alpha; /* optional global alpha for alpha blending */
   } option;
} DD_GE_PARAM_T;

/*
 * GE function pointers
 */
typedef struct dd_ge_funcs {
   DD_STATUS_T (*dd_ge_alpha_blend) (DD_GE_PARAM_T *param);
   DD_STATUS_T (*dd_ge_copy_area) (DD_GE_PARAM_T *param);
   DD_STATUS_T (*dd_ge_fill_color) (DD_GE_PARAM_T *param);
} DD_GE_FUNCS_T;

/*
 * Routine called by the GE module to register its functions
 */
extern void dd_ge_register(DD_GE_FUNCS_T *ge_funcs);

/*
 * Routine called by the GE module to unregister its functions
 */
extern void dd_ge_unregister(void);

/*
 * Wrappers of the GE functions to be called by Displayer Director
 */
extern DD_STATUS_T dd_ge_alphablend(DD_GE_PARAM_T *param);
extern DD_STATUS_T dd_ge_copyarea(DD_GE_PARAM_T *param);
extern DD_STATUS_T dd_ge_fillcolor(DD_GE_PARAM_T *param);

#endif /* DD_GE_H */
