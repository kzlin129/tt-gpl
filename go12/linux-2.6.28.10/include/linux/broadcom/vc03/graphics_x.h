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

Project  :  VMCS Host Apps
Module   :  Framework
File     :  $RCSfile: graphics.h,v $
Revision :  $Revision: #4 $

FILE DESCRIPTION
Graphics definitions for the platform independant VMCS host apps.
=============================================================================*/

#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "vc_image_types.h"
#include "vchost.h"
#include "vc_dispservice_x_defs.h"

/******************************************************************************
Global typedefs, macros and constants
******************************************************************************/

//Magic flag which can be used instead of passing in the exact length of a string
#define GRAPHICS_STRING_IS_NULL_TERMINATED   0

//The height of the text which is rendered by the graphics layer
#define  GRAPHICS_TEXT_HEIGHT                16

//Colour which implies transparency
#define GRAPHICS_TRANSPARENT_COLOUR          0x00000001UL

//resource defs
typedef enum
{
   GRAPHICS_RESOURCE_HANDLE_TYPE_MIN,

   GRAPHICS_RESOURCE_RGB565,
   GRAPHICS_RESOURCE_RGB888, /*  888 format is ONLY used when loading bitmaps
                                 - you can't create or delete bitmaps with this format */
   GRAPHICS_RESOURCE_RGBA32,
   GRAPHICS_RESOURCE_TF_RGB32A,
   GRAPHICS_RESOURCE_TF_RGB565,

   GRAPHICS_RESOURCE_HANDLE_TYPE_MAX

} GRAPHICS_RESOURCE_TYPE_T;

//local resource definition - just a void ptr, as it is platform specific
typedef void *GRAPHICS_RESOURCE_HANDLE;

//Definitions which in certain functions can be used to mean the actual width and height of a resource, without
//having to know the data implicitly.
#define GRAPHICS_RESOURCE_WIDTH  0xFFFF
#define GRAPHICS_RESOURCE_HEIGHT 0xFFFF

//graphics macro's
#define R_555_MASK (0x7600)
#define G_555_MASK (0x03E0)
#define B_555_MASK (0x001F)

#define R_565_MASK (0xF800)
#define G_565_MASK (0x07E0)
#define B_565_MASK (0x001F)

#define R_888_MASK      (0x00FF0000)
#define G_888_MASK      (0x0000FF00)
#define B_888_MASK      (0x000000FF)
#define ALPHA_888_MASK  (0xFF000000)

#define GRAPHICS_RGB565( r, g, b ) ( ((((r) >> 3) << (5+6)) & R_565_MASK) | ((((g) >> 2) << 5) & G_565_MASK) | (((b) >> 3) & B_565_MASK) )

#define GRAPHICS_RGBA32( r, g, b, a ) GRAPHICS_RGBA888( r, g, b, a )

#define GRAPHICS_RGB555_TO_RGB565( rgb555 ) ((((rgb555) & (R_555_MASK | G_555_MASK)) << 1) | ((rgb555) & B_555_MASK) )

#if defined(RGB888)

   #define GRAPHICS_RGB888_TO_RGB565( rgb ) GRAPHICS_RGB565((rgb >> 0) & 0xff, (rgb >> 8) & 0xff, (rgb >> 16) & 0xff)

   #define GRAPHICS_RGBA888( r, g, b, a ) ( (((a) << (8+8+8)) & ALPHA_888_MASK) | (((b) << (8+8)) & R_888_MASK) | (((g) << 8) & G_888_MASK) | ((r) & B_888_MASK) )

   #define GRAPHICS_RGB888_TO_RGBA32( r, g, b, a ) (     (((a) & 0xFF) << 24) \
                                                      |  (((b) & 0xFF) << 16) \
                                                      |  (((g) & 0xFF) << 8) \
                                                      |  (((r) & 0xFF) << 0) )

   #define GRAPHICS_RGBA888_TO_BGRA888( rgb888 ) ( (rgb888) & (ALPHA_888_MASK | G_888_MASK | R_888_MASK | B_888_MASK) )
#else

   #define GRAPHICS_RGB888_TO_RGB565( rgb ) GRAPHICS_RGB565((rgb >> 16) & 0xff, (rgb >> 8) & 0xff, (rgb >> 0) & 0xff)

   #define GRAPHICS_RGBA888( r, g, b, a ) ( (((a) << (8+8+8)) & ALPHA_888_MASK) | (((r) << (8+8)) & R_888_MASK) | (((g) << 8) & G_888_MASK) | ((b) & B_888_MASK) )

   #define GRAPHICS_RGB888_TO_RGBA32( r, g, b, a ) (     (((a) & 0xFF) << 24) \
                                                      |  (((r) & 0xFF) << 16) \
                                                      |  (((g) & 0xFF) << 8) \
                                                      |  (((b) & 0xFF) << 0) )

   #define GRAPHICS_RGBA888_TO_BGRA888( rgb888 ) ( ((rgb888) & ALPHA_888_MASK) | ((rgb888) & G_888_MASK) | (((rgb888) & R_888_MASK) >> 16) | (((rgb888) & B_888_MASK) << 16) )
#endif

/******************************************************************************
Global Functions
******************************************************************************/

/* The graphics initialise routine - called by the platform layer! Do not use manually */
VCHPRE_ int32_t VCHPOST_ graphics_initialise( void );

// The graphics unnitialise routine - called by the platform layer! Do not use manually
VCHPRE_ int32_t VCHPOST_ graphics_uninitialise( void );

// Routine to create a new resource
VCHPRE_ int32_t VCHPOST_ graphics_create_resource( const uint32_t size_x,
                                                   const uint32_t size_y,
                                                   const GRAPHICS_RESOURCE_TYPE_T resource_type,
                                                   GRAPHICS_RESOURCE_HANDLE *resource_handle );

/* Routine to create a new videocore side resource */
VCHPRE_ int32_t VCHPOST_ graphics_create_videocore_resource(   const uint32_t size_x,
                                                               const uint32_t size_y,
                                                               const GRAPHICS_RESOURCE_TYPE_T image_type,
                                                               GRAPHICS_RESOURCE_HANDLE *resource_handle );


/* Routine to delete a resource */
VCHPRE_ int32_t VCHPOST_ graphics_delete_resource( GRAPHICS_RESOURCE_HANDLE resource_handle );


/* Routine to load a resource from a local file */
VCHPRE_ int32_t VCHPOST_ graphics_load_resource_local(   const char *file_name,
                                                         GRAPHICS_RESOURCE_HANDLE *resource_handle );

/* Routine to load a resource from a vmcs file - this is a host-side resource*/
VCHPRE_ int32_t VCHPOST_ graphics_load_resource_vmcs( const char *file_name,
                                                      GRAPHICS_RESOURCE_HANDLE *resource_handle );
/* As above, but make it a videocore side resource */
VCHPRE_ int32_t VCHPOST_ graphics_load_videocore_resource_vmcs( const char *file_name,
                                                      GRAPHICS_RESOURCE_HANDLE *resource_handle );

/* Routine to copy a local resource to a vmcs host based resource */
VCHPRE_ int32_t VCHPOST_ graphics_copy_local_resource_to_vmcs_resource( const GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                                        const uint32_t vmcs_dispman_resource );

/* routine to get the resource details */
VCHPRE_ int32_t VCHPOST_ graphics_get_resource_details(  const GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                         uint32_t *width,
                                                         uint32_t *height,
                                                         uint32_t *pitch,
                                                         GRAPHICS_RESOURCE_TYPE_T *type,
                                                         void **memory_pointer,
                                                         uint32_t *object_id,
                                                         uint32_t *resource_id,
                                                         uint32_t *native_image_handle);

/* routines to bitblt data between bitmaps */
VCHPRE_ int32_t VCHPOST_ graphics_bitblt( const GRAPHICS_RESOURCE_HANDLE src_resource_handle,
                                          const uint32_t x,
                                          const uint32_t y,
                                          const uint32_t width,
                                          const uint32_t height,
                                          GRAPHICS_RESOURCE_HANDLE dest_resource_handle,
                                          const uint32_t x_pos,
                                          const uint32_t y_pos );

VCHPRE_ int32_t VCHPOST_ graphics_stretch_bitblt(  const GRAPHICS_RESOURCE_HANDLE src_resource_handle,
                                                   const uint32_t x,
                                                   const uint32_t y,
                                                   const uint32_t width,
                                                   const uint32_t height,
                                                   GRAPHICS_RESOURCE_HANDLE dest_resource_handle,
                                                   const uint32_t dest_x,
                                                   const uint32_t dest_y,
                                                   const uint32_t dest_width,
                                                   const uint32_t dest_height );

/* Routine display a resource on videocore */
VCHPRE_ int32_t VCHPOST_ graphics_display_resource(   GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                      const uint16_t display_number,
                                                      const int16_t z_order,
                                                      const uint16_t offset_x,
                                                      const uint16_t offset_y,
                                                      const uint16_t dest_width,
                                                      const uint16_t dest_height,
                                                      const VC_DISPMAN_TRANSFORM_T transform,
                                                      const uint8_t display );

/* routine to update a resource already being displayed */
VCHPRE_ int32_t VCHPOST_ graphics_update_displayed_resource(GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                            const uint32_t x_offset,
                                                            const uint32_t y_offset,
                                                            const uint32_t width,
                                                            const uint32_t height );

/* routine to move a resource already being displayed */
VCHPRE_ int32_t VCHPOST_ graphics_move_displayed_resource(  GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                            const uint16_t display_number,
                                                            const int16_t z_order,
                                                            const uint16_t offset_x,
                                                            const uint16_t offset_y,
                                                            const uint16_t dest_height,
                                                            const uint16_t dest_width,
                                                            const VC_DISPMAN_TRANSFORM_T transform );

/* routine to fill part of a resource */
VCHPRE_ int32_t VCHPOST_ graphics_resource_fill(GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                const uint32_t x,
                                                const uint32_t y,
                                                const uint32_t width,
                                                const uint32_t height,
                                                const uint32_t fill_colour );

/* routine to vertically flip an image in place */
VCHPRE_ int32_t VCHPOST_ graphics_resource_vflip_in_place(GRAPHICS_RESOURCE_HANDLE resource_handle);

/* routine to set the alpha level of a resource */
VCHPRE_ int32_t VCHPOST_ graphics_resource_set_alpha( GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                      const uint32_t x,
                                                      const uint32_t y,
                                                      const uint32_t width,
                                                      const uint32_t height,
                                                      const uint8_t alpha );


/* routine to set the alpha level of a resource for a particular colour */
VCHPRE_ int32_t VCHPOST_ graphics_resource_set_alpha_per_colour(  GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                                  const uint32_t colour,
                                                                  const uint8_t alpha );


/* routine to render text into a resource */
VCHPRE_ int32_t VCHPOST_ graphics_resource_render_text(  GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                         const uint32_t x,
                                                         const uint32_t y,
                                                         const uint32_t width, /* this can be GRAPHICS_RESOURCE_HANDLE_WIDTH for no clipping */
                                                         const uint32_t height, /* this can be GRAPHICS_RESOURCE_HANDLE_HEIGHT for no clipping */
                                                         const uint32_t text_foreground_colour,
                                                         const uint32_t text_background_colour,
                                                         const char *text,
                                                         const uint32_t text_length);

/* routine to render text into a resource with chosen size */
VCHPRE_ int32_t VCHPOST_ graphics_resource_render_text_ext( GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                            const uint32_t x,
                                                            const uint32_t y,
                                                            const uint32_t width, /* this can be GRAPHICS_RESOURCE_HANDLE_WIDTH for no clipping */
                                                            const uint32_t height, /* this can be GRAPHICS_RESOURCE_HANDLE_HEIGHT for no clipping */
                                                            const uint32_t text_foreground_colour,
                                                            const uint32_t text_background_colour,
                                                            const char *text,
                                                            const uint32_t text_length,
                                                            const uint32_t text_size );

/* routine to calculate text width */
VCHPRE_ int32_t VCHPOST_ graphics_resource_text_dimensions(    GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                               const char *text,
                                                               const uint32_t text_length,
                                                               uint32_t *width,
                                                               uint32_t *height );

/* routine to calculate text width with chosen size */
VCHPRE_ int32_t VCHPOST_ graphics_resource_text_dimensions_ext(   GRAPHICS_RESOURCE_HANDLE resource_handle,
                                                                  const char *text,
                                                                  const uint32_t text_length,
                                                                  uint32_t *width,
                                                                  uint32_t *height,
                                                                  const uint32_t text_size );

/******************************************************************************
These functions below are used to manually control when VMCS actually performs any requested
display operations. If you call graphics_update_start() then you MUST call graphics_update_end()
before exiting your function.
Calling graphics_update_start increments a counter, whereas graphics_update_end decrements a counter.
If the counter is 0 (initial value) when graphics_update_start is called, an actual dispman_update_start is called.
When graphics_update_end is called, the counter is decremented. If it is then 0, an actual dispman_update_end is called.
******************************************************************************/

VCHPRE_ int32_t VCHPOST_ graphics_update_start( void );

VCHPRE_ int32_t VCHPOST_ graphics_update_end( void );

/******************************************************************************
Functions which are common to all graphics.c implementations
Some of these are usually by the framework ports of the graphics.c library.
******************************************************************************/

// return size of a display
VCHPRE_ int32_t VCHPOST_ graphics_get_display_size( const uint16_t display_number,
                                                    uint32_t *width,
                                                    uint32_t *height);

//Routine to read a bitmap header and extract the buffer size
VCHPRE_ int32_t VCHPOST_ graphics_common_read_bitmap_header_via_vmcs(const char *file_name,
                                                                     uint32_t *width,
                                                                     uint32_t *height,
                                                                     uint32_t *pitch,
                                                                     GRAPHICS_RESOURCE_TYPE_T *resource_type,
                                                                     uint32_t *image_data_size );

//Routine to read a tga header and extract the buffer size
VCHPRE_ int32_t VCHPOST_ graphics_common_read_tga_header_via_vmcs(   const char *file_name,
                                                                     uint32_t *width,
                                                                     uint32_t *height,
                                                                     uint32_t *pitch,
                                                                     GRAPHICS_RESOURCE_TYPE_T *resource_type,
                                                                     uint32_t *image_data_size );


//Routine to read a bitmap file
VCHPRE_ int32_t VCHPOST_ graphics_common_read_bitmap_file_from_vmcs( const char *file_name,
                                                                     void *image_data,
                                                                     const uint32_t image_data_size );

//Routine to read a tga file
VCHPRE_ int32_t VCHPOST_ graphics_common_read_tga_file_from_vmcs( const char *file_name,
                                                                  void *image_data,
                                                                  const uint32_t image_data_size );

//Routine to copy a HOST resource onto a vmcs resource
//VCHPRE_ int32_t VCHPOST_ graphics_copy_resource_to_vmcs_resource( const uint32_t source_resource,
//                                                                  const uint32_t source_resource_width,
//                                                                  const uint32_t source_resource_height,
//                                                                  const uint32_t dest_resource );

#endif /* GRAPHICS_H */
