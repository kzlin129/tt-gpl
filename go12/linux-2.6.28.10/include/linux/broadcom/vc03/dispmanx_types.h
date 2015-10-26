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




/*=============================================================================

File     :  $RCSfile: dispmanx.h,v $
Revision :  $Revision: # $

FILE DESCRIPTION
Typedefs and enums for the VideoCore III Display Manager.
=============================================================================*/

#ifndef _DISPMANX_TYPES_H
#define _DISPMANX_TYPES_H

#include "vcos.h"
#include "vc_image_types.h"

/*** Typedefs ***/

/* Return codes. Nonzero ones indicate failure. */
typedef enum tag_DISPMANX_STATUS_T {
   DISPMANX_SUCCESS      = 0,
   DISPMANX_INVALID      = -1
   /* XXX others TBA */
} DISPMANX_STATUS_T;

/* Flags (for alpha source and scaling quality) passed to dispmanx_element_add() */
typedef enum tag_DISPMANX_FLAGS_T {
   DISPMANX_FLAGS_ALPHA_NONE      = 0,   /* there is no alpha plane                  */
   DISPMANX_FLAGS_ALPHA_SOURCE    = 2,   /* use source image alpha (where present)   */
   DISPMANX_FLAGS_ALPHA_MASK      = 4,   /* use mask alpha (must supply mask image)  */

   DISPMANX_FLAGS_ALPHA_FIXED     = 1,   /* addin: also apply global opacity factor  */
   DISPMANX_FLAGS_ALPHA_THRESH    = 8    /* addin: treat values above 0x07 as opaque */
} DISPMANX_FLAGS_T;

/* Opaque handles */
typedef uint32_t DISPMANX_DISPLAY_HANDLE_T;
typedef uint32_t DISPMANX_UPDATE_HANDLE_T;
typedef uint32_t DISPMANX_ELEMENT_HANDLE_T;
typedef uint32_t DISPMANX_RESOURCE_HANDLE_T;

#define DISPMANX_NO_HANDLE 0

/* Update callback. */
typedef void (*DISPMANX_CALLBACK_FUNC_T)(DISPMANX_UPDATE_HANDLE_T u, void * arg);


/* Display information structure. Display driver should report the physical
   dimensions and set transform to 0. When queried by the user, all values
   will be suitably mangled to correspond to the logical coordinate system,
   and transform will reflect the actual post-transform to be applied.      */

typedef struct tag_DISPMANX_MODEINFO_T {
   const char * description;
   int32_t      width;
   int32_t      height;
   int32_t      aspect_pixwidth;
   int32_t      aspect_pixheight;
   int32_t      fieldrate_num;
   int32_t      fieldrate_denom;
   int32_t      fields_per_frame;
   VC_IMAGE_TRANSFORM_T transform;
} DISPMANX_MODEINFO_T;


/* Pluggable display interface */

typedef struct tag_DISPMANX_DISPLAY_FUNCS_T {
   // Get essential HVS configuration to be passed to the HVS driver. Options
   // is any combination of the following flags: HVS_ONESHOT, HVS_FIFOREG,
   // HVS_FIFO32, HVS_AUTOHSTART, HVS_INTLACE; and if HVS_FIFOREG, one of;
   // { HVS_FMT_RGB888, HVS_FMT_RGB565, HVS_FMT_RGB666, HVS_FMT_YUV }.
   int32_t (*get_hvs_config)(void * instance, uint32_t * pchan, 
                             uint32_t * poptions, uint32_t * pwidth, uint32_t * pheight);
   
   // Get optional HVS configuration for gamma tables, OLED matrix and dither controls.
   // Set these function pointers to NULL if the relevant features are not required.
   int32_t (*get_gamma_params)(void * instance,
                               int32_t gain[3], int32_t offset[3], int32_t gamma[3]);
   int32_t (*get_oled_params)(void * instance, uint32_t * poffsets,
                              uint32_t coeffs[3]);
   int32_t (*get_dither)(void * instance, uint32_t * dither_type);
   
   // Get mode information, which may be returned to the applications as a courtesy.
   // Transform should be set to 0, and {width,height} should be final dimensions.
   int32_t (*get_info)(void * instance, DISPMANX_MODEINFO_T * info);
   
   // Inform driver that the application refcount has become nonzero / zero
   // These callbacks might perhaps be used for backlight and power management.
   int32_t (*open)(void * instance);
   int32_t (*close)(void * instance);
   
   // Display list updated callback. Primarily of use to a "one-shot" display.
   // For convenience of the driver, we pass the register address of the HVS FIFO.
   void (*dlist_updated)(void * instance, volatile uint32_t * fifo_reg);
   
   // End-of-field callback. This may occur in an interrupt context.
   void (*eof_callback)(void * instance);
} DISPMANX_DISPLAY_FUNCS_T;

#endif /* ifndef _DISPMANX_TYPES_H */
