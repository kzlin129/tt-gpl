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




#ifndef _VC_DISPMAN2_H_
#define _VC_DISPMAN2_H_

#include "vcinterface.h"
#include "vc_dispservice_defs.h"

/* Initialise display manager service. Returns its interface number. This initialises
   the host side of the interface, it does not send anything to VideoCore. */

int vc_dispman_init(void);

/* Stop the service from being used. */

void vc_dispman_stop(void);

/* Return the service number (-1 if not running). */
int vc_dispman_inum(void);

// Lock/unlock the displays so that they can be modified in place.
int vc_dispman_lock (uint32_t state, int32_t *response);
int vc_dispman_unlock (uint32_t *state);

/******************************************************************************
Send commands to VideoCore.
These all return 0 for success.
******************************************************************************/

// Set the destination of the display. Only works for the "memory" display.
extern int vc_dispman_display_set_destination(uint32_t display, uint32_t resource, int32_t *response);

// Snapshot the given display into the resource buffer. USE WITH CARE.
extern int vc_dispman_display_snapshot(uint32_t display, uint32_t resource, int32_t *response);

// Return the composite buffer address of a display. Caveat emptor.
extern int vc_dispman_display_get_composite(uint32_t display, uint32_t *composite_addr);

// Change the apparent shape of a display. May not make a display bigger than it was
// initialised to.
extern int vc_dispman_display_reconfigure(uint32_t display,
                                          uint16_t width, uint16_t height, int32_t *response);

/* function to apply an effects instance to a display */

extern int vc_dispman_display_apply_effects_instance(uint32_t display, const uint32_t effects_instance, int32_t *response);

/* function to create a new effects instance */

extern int vc_dispman_create_effects_instance( uint32_t *effects_instance, int32_t *response);

/* function to delete a new effects instance */

extern int vc_dispman_delete_effects_instance( const uint32_t effects_instance, int32_t *response);

/* function to set an effect */

extern int vc_dispman_set_effect( const uint32_t effect_instance, const char *effect_name, int32_t *response);

// Functions to start or end an atomic update to a display.

// Brackets the start of any display modifications.
extern int vc_dispman_update_start(int32_t *response);

// Brackets the end of a set of display modifications.
extern int vc_dispman_update_end(int32_t *response);

//  Functions for manipulating display objects. All return non-zero for failure.

// Add a display object so that it gets shown.
extern int vc_dispman_object_add(uint32_t *object,
                                 uint32_t display,
                                 int32_t layer,
                                 uint16_t dest_x_offset,
                                 uint16_t dest_y_offset,
                                 uint16_t width,
                                 uint16_t height,
                                 uint32_t resource,
                                 uint16_t src_x_offset,
                                 uint16_t src_y_offset,
                                 VC_DISPMAN_TRANSFORM_T transform);

// Delete a display object. Does not actually deallocate it (that's the caller's job)
// but tells the display manager that it will not be needed any more once the next
// actual display update begins.
extern int vc_dispman_object_remove(uint32_t object, int32_t *response);

// Tell the Display Manager that the given region of an object has been modified in
// place. It is preferable to use this when possible.
extern int vc_dispman_object_modify(uint32_t object,
                                    uint16_t x_offset,
                                    uint16_t y_offset,
                                    uint16_t width,
                                    uint16_t height,
                                    int32_t *response);

// Functions for manipulating resources.

// Create a new resource.
// Set transparent colour to -1 if it is a non-transparent RGB565 bitmap.
extern int vc_dispman_resource_create(uint32_t *resource, VC_IMAGE_PARAM_T *image, VC_RESOURCE_TYPE_T type);

// Delete a resource.
extern int vc_dispman_resource_delete(uint32_t resource, int32_t *response);

extern int vc_dispman_resource_set_alpha(uint32_t resource, uint32_t alpha, int32_t *response);
//Some utility functions to help with image creation

//Create an image.  Does not allocate storage for the data
extern int vc_dispman_image_create( VC_IMAGE_PARAM_T *image,
                                    VC_IMAGE_FORMAT_T type,
                                    uint32_t width,           // width in pixels
                                    uint32_t height,          // height in pixels
                                    uint32_t pitch,           // pitch in *bytes* - can be left 0 for automatic calculation
                                    void *pointer             // image data if on host side, or NULL if on VideoCore side
                                    );

// Function to query what image formats are support by the VMCS host
extern int vc_dispman_query_image_formats( uint32_t *support_formats );

extern int vc_dispman_copy_dirty_rows( VC_IMAGE_PARAM_T   *srcImage, 
                                       VC_IMAGE_PARAM_T   *dstImage,
                                       int                 numRows,
                                       uint32_t           *dirtyRowBits,
                                       int32_t            *response );

extern int vc_dispman_copyarea( VC_IMAGE_PARAM_T    *image,
                                uint32_t             bytes_per_line,
                                uint32_t             bytes_per_pixel,
                                uint32_t             max_width,
                                uint32_t             max_height,
                                uint32_t             sx,
                                uint32_t             sy,
                                uint32_t             dx,
                                uint32_t             dy,
                                uint32_t             width,
                                uint32_t             height,
                                int32_t             *response );

extern int vc_dispman_fillrect( VC_IMAGE_PARAM_T    *image,
                                uint32_t             bytes_per_line,
                                uint32_t             bytes_per_pixel,
                                uint32_t             max_width,
                                uint32_t             max_height,
                                uint32_t             dx,
                                uint32_t             dy,
                                uint32_t             width,
                                uint32_t             height,
                                uint32_t             color,
                                int32_t             *response );
#endif //_VC_DISPMAN2_H_
