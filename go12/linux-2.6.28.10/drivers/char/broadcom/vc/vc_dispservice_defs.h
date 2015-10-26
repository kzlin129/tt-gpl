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




#ifndef VC_DISPSERVICE_DEFS_H
#define VC_DISPSERVICE_DEFS_H

#define  HOST_PITCH_ALIGNMENT    4

//Round up to the nearest multiple of 16
#define  PAD16(x) (((x) + (VC_INTERFACE_BLOCK_SIZE-1)) & ~(VC_INTERFACE_BLOCK_SIZE-1))

//The max length for an effect name
#define DISPMAN_MAX_EFFECT_NAME  (28)

typedef enum {
   // Values chosen to match VC_IMAGE_TYPE_T to aid debugging (does not affect
   // correct functioning of code however).
   VC_FORMAT_RGB565 = 1,
   VC_FORMAT_RGB888 = 5,
   VC_FORMAT_RGBA32 = 15,
   VC_FORMAT_RGBA565 = 17,
   VC_FORMAT_RGBA16 = 18
} VC_IMAGE_FORMAT_T;

// Transforms.
/* Image transformations. These must match the DISPMAN and Media Player versions */
#define TRANSFORM_HFLIP     (1<<0)
#define TRANSFORM_VFLIP     (1<<1)
#define TRANSFORM_TRANSPOSE (1<<2)

typedef enum {
   VC_DISPMAN_ROT0           = 0,
   VC_DISPMAN_MIRROR_ROT0    = TRANSFORM_HFLIP,
   VC_DISPMAN_MIRROR_ROT180  = TRANSFORM_VFLIP,
   VC_DISPMAN_ROT180         = TRANSFORM_HFLIP|TRANSFORM_VFLIP,
   VC_DISPMAN_MIRROR_ROT90   = TRANSFORM_TRANSPOSE,
   VC_DISPMAN_ROT270         = TRANSFORM_TRANSPOSE|TRANSFORM_HFLIP,
   VC_DISPMAN_ROT90          = TRANSFORM_TRANSPOSE|TRANSFORM_VFLIP,
   VC_DISPMAN_MIRROR_ROT270  = TRANSFORM_TRANSPOSE|TRANSFORM_HFLIP|TRANSFORM_VFLIP,
} VC_DISPMAN_TRANSFORM_T;

typedef enum {
   VC_RESOURCE_TYPE_HOST,
   VC_RESOURCE_TYPE_VIDEOCORE,
} VC_RESOURCE_TYPE_T;

typedef struct {
   uint8_t  type;            // VC_IMAGE_FORMAT_T
   uint32_t width;           // width in pixels
   uint32_t height;          // height in pixels
   uint32_t pitch;           // pitch of image_data array in *bytes*
   uint32_t size;            // number of *bytes* available in the image_data arry
   uint32_t pointer;         // pointer for image_data - this allows the object to be cast to a VC_IMAGE_T on the VIDEOCORE side
} VC_IMAGE_PARAM_T;

typedef struct {
    uint32_t    src_ptr;    // Pointer to source data
    uint32_t    dst_ptr;    // Pointer to destination data
    uint32_t    pitch;      // Pitch of image array (in bytes)
    uint32_t    numRows;    // Number of rows to look at.
    uint32_t    dirtyRowBits[ 16 ];    // Allows upto 496 lines (used 16 to round structure out to a multiple of 16 bytes)

} VC_COPY_DIRTY_ROWS_PARAM_T;

typedef struct {
   uint32_t    img_ptr;         // Pointer to the image data
   uint32_t    bytes_per_line;  // bytes per horizontal line
   uint32_t    bytes_per_pixel; // bytes per pixel
   uint32_t    max_width;       // maximum width
   uint32_t    max_height;      // maximum height
   uint32_t    sx;              // source X in pixels
   uint32_t    sy;              // source Y in pixels
   uint32_t    dx;              // destination X in pixels
   uint32_t    dy;              // destination Y in pixels
   uint32_t    width;           // width in pixels
   uint32_t    height;          // height in pixels
   uint32_t    dummy[1];        // pad to multiple of 16 bytes
} VC_COPYAREA_PARAM_T;

typedef struct {
   uint32_t    img_ptr;         // Pointer to the image data
   uint32_t    bytes_per_line;  // bytes per horizontal line
   uint32_t    bytes_per_pixel; // bytes per pixel
   uint32_t    max_width;       // maximum width
   uint32_t    max_height;      // maximum height
   uint32_t    dx;              // destination X in pixels
   uint32_t    dy;              // destination Y in pixels
   uint32_t    width;           // width in pixels
   uint32_t    height;          // height in pixels
   uint32_t    color;           // raw color
   uint32_t    dummy[2];        // pad to multiple of 16 bytes
} VC_FILLRECT_PARAM_T;

typedef enum {
   VC_DISPMAN_DISPLAY_SET_DESTINATION = 0,
   VC_DISPMAN_DISPLAY_UPDATE_START,
   VC_DISPMAN_DISPLAY_UPDATE_END,
   VC_DISPMAN_DISPLAY_OBJECT_ADD,
   VC_DISPMAN_DISPLAY_OBJECT_REMOVE,
   VC_DISPMAN_DISPLAY_OBJECT_MODIFY,
   VC_DISPMAN_DISPLAY_LOCK,
   VC_DISPMAN_DISPLAY_UNLOCK,
   VC_DISPMAN_DISPLAY_RESOURCE_CREATE,
   VC_DISPMAN_DISPLAY_RESOURCE_DELETE,
   VC_DISPMAN_DISPLAY_GET_COMPOSITE,
   VC_DISPMAN_DISPLAY_APPLY_EFFECT_INSTANCE,
   VC_DISPMAN_DISPLAY_RECONFIGURE,
   VC_DISPMAN_DISPLAY_CREATE_EFFECTS_INSTANCE,
   VC_DISPMAN_DISPLAY_DELETE_EFFECTS_INSTANCE,
   VC_DISPMAN_DISPLAY_SET_EFFECT,
   VC_DISPMAN_DISPLAY_RESOURCE_SET_ALPHA,
   VC_DISPMAN_DISPLAY_SNAPSHOT,
   VC_DISPMAN_DISPLAY_QUERY_IMAGE_FORMATS,
   VC_DISPMAN_DISPLAY_COPY_DIRTY_ROWS,
   VC_DISPMAN_DISPLAY_COPYAREA,
   VC_DISPMAN_DISPLAY_FILLRECT,
   VC_CMD_END_OF_LIST
} VC_CMD_CODE_T;

/* The table of functions executed for each command. */

typedef void (*INTERFACE_EXECUTE_FN_T)(int, int);

extern INTERFACE_EXECUTE_FN_T interface_execute_fn[];

//Parameter sets for dispservice commands
typedef struct {
   uint32_t state;
   uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_LOCK_PARAM_T;

typedef struct {
   uint32_t display;
   uint32_t resource;
   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_SET_DEST_PARAM_T;

typedef struct {
   uint32_t display;
   uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_GET_COMPOSITE_PARAM_T;

typedef struct
{
   uint32_t display;
   uint32_t effects_instance;

   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_APPLY_EFFECTS_INSTANCE_PARAM_T;

typedef struct
{
   uint32_t read_response;
   uint32_t effects_instance;
} DISPMAN_CREATE_EFFECTS_INSTANCE_RESPONSE_T;

typedef struct
{
   uint32_t effects_instance;
   uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_DELETE_EFFECTS_INSTANCE_PARAM_T;

typedef struct
{
   uint32_t effects_instance;
   char effect_name[ DISPMAN_MAX_EFFECT_NAME ];
   //no need to pad as long as DISPMAN_MAX_EFFECT_NAME +sizeof(uint32) = 32
} DISPMAN_SET_EFFECT_PARAM_T;

typedef struct {
   uint32_t display;
   uint16_t width;
   uint16_t height;
   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_RECONFIGURE_PARAM_T;

typedef struct {
   uint32_t display;
   uint32_t transparent_colour;
   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_SET_TRANSPARENT_COLOUR_PARAM_T;

typedef struct {
   //uint32_t object;
   uint32_t display;
   int32_t layer;
   uint32_t transform;
   uint32_t resource;
   uint16_t dest_x;
   uint16_t dest_y;
   uint16_t src_x;
   uint16_t src_y;
   uint16_t width;
   uint16_t height;
   uint32_t dummy[1];
} DISPMAN_OBJECT_ADD_PARAM_T;

typedef struct {
   uint32_t object;
   uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_OBJECT_REMOVE_PARAM_T;

typedef struct {
   uint32_t object;
   uint16_t src_x;
   uint16_t src_y;
   uint16_t width;
   uint16_t height;
   uint32_t dummy[1];   // Pad to multiple of 16 bytes
} DISPMAN_OBJECT_MODIFY_PARAM_T;

typedef struct {
   uint32_t *resource;
   VC_IMAGE_PARAM_T image;
   uint8_t  type;   // VC_RESOURCE_TYPE_T
   //Removed padding.  VC_IMAGE_T may change in size, so handle the size in the code that sends and receives the commands
   //uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_RESOURCE_CREATE_PARAM_T;

typedef struct {
   uint32_t resource;
   uint32_t dummy[3];   //Pad to multiple of 16 bytes
} DISPMAN_RESOURCE_DELETE_PARAM_T;

typedef struct {
   uint32_t resource;
   uint32_t alpha;
   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_RESOURCE_SET_ALPHA_PARAM_T;

typedef struct {
   uint32_t display;
   uint32_t resource;
   uint32_t dummy[2];   //Pad to multiple of 16 bytes
} DISPMAN_DISPLAY_SNAPSHOT_PARAM_T;


#endif   //VC_DISPSERVICE_DEFS_H
