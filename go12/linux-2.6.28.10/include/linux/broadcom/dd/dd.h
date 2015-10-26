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
 * Description: Public header of the Display Director driver. Users of the
 * Display Director should only include this header.
 *
 * NOTE: The Display Director is a kernel driver. In the future a user space
 * library will be provided with APIs very similar to the kernel functions
 * 
 * NOTE: The APIs in this header file are not final.  Work is still ongoing to
 * optimize the DD and this may necessitate changes in this API.  Return codes
 * still need to be finalized as well...
 */

/*
 * The Display Director uses the concepts of Image and Element. An Image is a
 * piece of memory that contains the bitmap information. An Element
 * represents a rectangular region of an Image. Each Element has its associated
 * layer that follows the z-order (ascending order with the lowest number as the
 * background), parameters that govern the post-processing operations
 * (cropping, rotation, etc) and an optional global alpha.
 *
 * The Display Director has no knowledge of the so-called background or
 * foreground graphics, merely the notion of graphics being on different
 * layers
 *
 * Before making any changes to the display including addition or deletion of
 * Elements or changes to the bitmaps of an Image, dd_update_start must be
 * called, and these changes are committed with a call to dd_update_submit_sync.
 *
 * The Display Director maintains a linked list of the Elements. An Image can
 * be shared among various Elements. Also, an Element can be an entire Image.
 * When dd_update_submit_sync is called, the Display Director goes through all
 * Elements in the linked link in an ascending order of its associated layer
 * number and performs specified operations (alpha blending, scaling,
 * rotation), then merges all the Elements to produce the final displayed image.
 */

#ifndef DD_H
#define DD_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <linux/broadcom/knllog.h>
#else
#include <stdint.h>
#endif

/*
 * Turn on mislleneous support functions used for DEMO purposes only
 */
#define DD_VIDEO_DEMO 1

#define DD_MAX_WIDTH            854
#define DD_MAX_HEIGHT           480
#define DD_MAX_BUFFER_SIZE      (DD_MAX_WIDTH * DD_MAX_HEIGHT * 4)
#define DD_INVALID_HANDLE       10000

/* max number of updates allowed */
#define DD_MAX_UPDATES    6

/* max number of elements allowed */
#define DD_MAX_ELEMENTS   12

#define DD_MAX_ELEM_NAME_LEN   8

#define DD_MAGIC               'D'
#define DD_CMD_INIT            0x00
#define DD_CMD_TERM            0x01
#define DD_CMD_IS_INIT         0x02
#define DD_CMD_LCD_GET         0x10
#define DD_CMD_IMG_CREATE      0x20
#define DD_CMD_IMG_DEL         0x21
#define DD_CMD_ELEMENT_ADD     0x30
#define DD_CMD_ELEMENT_RMV     0x31
#define DD_CMD_ELEMENT_MOD     0x32
#define DD_CMD_UPDATE_START    0x40
#define DD_CMD_UPDATE_SYNC     0x41
#define DD_CMD_COPYAREA        0x50

#define DD_IOCTL_INIT          _IOW(DD_MAGIC, DD_CMD_INIT, DD_LCD_MODEL_T)
#define DD_IOCTL_TERM          _IO(DD_MAGIC, DD_CMD_TERM)
#define DD_IOCTL_IS_INIT       _IO(DD_MAGIC, DD_CMD_IS_INIT)
#define DD_IOCTL_LCD_GET       _IOR(DD_MAGIC, DD_CMD_LCD_GET, DD_LCD_INFO_T)
#define DD_IOCTL_IMG_CREATE    _IOWR(DD_MAGIC, DD_CMD_IMG_CREATE, DD_IMAGE_T)
#define DD_IOCTL_IMG_DEL       _IOW(DD_MAGIC, DD_CMD_IMG_DEL, DD_IMAGE_T)
#define DD_IOCTL_ELEMENT_ADD   _IOW(DD_MAGIC, DD_CMD_ELEMENT_ADD, DD_ELEMENT_T)
#define DD_IOCTL_ELEMENT_MOD   _IOWR(DD_MAGIC, DD_CMD_ELEMENT_MOD, DD_ELEMENT_T)
#define DD_IOCTL_ELEMENT_RMV   _IOW(DD_MAGIC, DD_CMD_ELEMENT_RMV, DD_ELEMENT_HANDLE_T)
#define DD_IOCTL_UPDATE_START  _IOW(DD_MAGIC, DD_CMD_UPDATE_START, int32_t)
#define DD_IOCTL_UPDATE_SYNC   _IOW(DD_MAGIC, DD_CMD_UPDATE_SYNC, DD_UPDATE_HANDLE_T)
#define DD_IOCTL_COPYAREA      _IOW(DD_MAGIC, DD_CMD_COPYAREA, DD_COPYAREA_T)

#ifdef __KERNEL__
#define DD_LOG_ENABLED    0
#if DD_LOG_ENABLED
#define DD_LOG            KNLLOG
#else
#define DD_LOG(c,args...)
#endif
#endif

typedef uint32_t DD_DISPLAY_HANDLE_T;
typedef uint32_t DD_UPDATE_HANDLE_T;
typedef uint32_t DD_ELEMENT_HANDLE_T;

typedef enum dd_status {
   DD_SUCCESS = 0,
   DD_FAIL = -1,
} DD_STATUS_T;

/*
 * Color format
 */
typedef enum dd_format {
   DD_FORMAT_ARGB888 = 0, /* 32-bit ARGB 8888 */
   DD_FORMAT_URGB888, /* 32-bit RGB 0888 */
   /* to be added ... */
   DD_FORMAT_INVALID
} DD_FORMAT_T;

/*
 * LCD panel model definition
 */
typedef enum dd_lcd_model {
   DD_LCD_MODEL_SEIKO_RA167Z = 0,
   DD_LCD_MODEL_SEIKO_RA169Z,
   DD_LCD_MODEL_SAMSUNG_LMS700KF01,
   DD_LCD_MODEL_INVALID
} DD_LCD_MODEL_T;

/*
 * Rotation angle of a rectangular region 
 */
typedef enum dd_transform {
   DD_ROT0 = 0,
   DD_ROT90,
   DD_ROT180,
   DD_ROT270,
   DD_VFLIP,
   DD_HFLIP,
   /* to be added ... */
} DD_TRANSFORM_T;

/*
 * Alpha blending config flags
 */
typedef enum dd_flags {
   DD_FLAGS_ALPHA_NONE = 0, /* NO alpha blending required */
   DD_FLAGS_ALPHA_SOURCE, /* use in-pixel alpha in source image */
   DD_FLAGS_ALPHA_FIXED, /* use global alpha (opacity) factor */
} DD_FLAGS_T;

/*
 * Element state
 */
typedef enum dd_element_state {
   /* free and ready to be used */
   DD_ELEMENT_STATE_FREE,
   /* belonged to an update and about to be loaded to display */
   DD_ELEMENT_STATE_UPDATE,
   /* currently being displayed */
   DD_ELEMENT_STATE_DISPLAY 
} DD_ELEMENT_STATE_T;

/*
 * Generic LCD display information
 */
typedef struct dd_lcd_info {
   uint32_t width; /* display width in pixels */
   uint32_t height; /* display height in pixels */
   uint32_t bpp; /* bits per pixel */
   uint32_t pitch; /* bytes per line */
   uint32_t size; /* required bitmap buffer size in bytes */
   uint32_t pixel_clk; /* pixel clock */
	uint32_t left_margin; /* horizontal back porch */
	uint32_t right_margin; /* horizontal front porch */
	uint32_t upper_margin; /* vertical back porch */
	uint32_t lower_margin; /* vertical front porch */
	uint32_t hsync_len; /* horizontal pulse width */
	uint32_t vsync_len; /* vertical pulse width */
} DD_LCD_INFO_T;

/*
 * The data structure that defines a rectangular region on an image
 */
typedef struct dd_rect
{
   unsigned int x; /* X offset (pixels) from the start of the image */
   unsigned int y; /* Y offset (pixels) from the start of the image */
   unsigned int width; /* width of the rectangle (pixels) */
   unsigned int height; /* height of the rectangle (pixels) */
} DD_RECT_T;

/*
 * Image data structure that contains information of the image dimension and
 * a pointer to the memory that contains the image bitmap
 */
typedef struct dd_image {
   DD_FORMAT_T format; /* color format */
   unsigned short width; /* width in pixels */
   unsigned short height; /* height in pixels */
   int pitch; /* bytes per line */
   int size; /* size (in bytes) of the image_data array */
   void *image_data; /* pointer to the bitmap (framebuffer) */
   uint32_t addr; /* physical address of the bitmap */
} DD_IMAGE_T;

/*
 * Element data structure
 */

typedef struct dd_element DD_ELEMENT_T;

struct dd_element {
   DD_ELEMENT_HANDLE_T handle;
   /* name of the owner of the element */
   char name[DD_MAX_ELEM_NAME_LEN];
#ifdef __KERNEL__
   /* active/inactive */
   volatile atomic_t state;
#else
   int reserved;
#endif
   DD_FLAGS_T flags; /* alpha blending flags */
   uint8_t opacity; /* global alpha */
   DD_TRANSFORM_T transform;
   int32_t layer; /* layer that follows the z-order */
   DD_RECT_T dest_rect; /* destination rectangle */
   DD_IMAGE_T src_img; /* source image, bitmap */
   DD_RECT_T src_rect; /* source rectangle */
   /* pointer to the next element in the update list */
   DD_ELEMENT_T *update_next; 
   /* pointer to the next element in the display list */
   DD_ELEMENT_T *display_next; 
};

/*
 * Parameters for DD_IOCTL_COPYAREA
 */

typedef struct
{
    DD_RECT_T   srcRect;
    DD_IMAGE_T  srcImage;

    DD_RECT_T   dstRect;
    DD_IMAGE_T  dstImage;

} DD_COPYAREA_T;

#ifdef __KERNEL__

/*
 * Create a new image with specified width and height (in pixels) and allocate
 * its associated bitmap buffer
 */
extern DD_STATUS_T dd_image_create(DD_IMAGE_T *img, unsigned short width,
      unsigned short height, DD_FORMAT_T format);

/*
 * Delete an image and free up its bitmap buffer memory
 */
extern DD_STATUS_T dd_image_delete(DD_IMAGE_T *img);

/*
 * Copy a rectangular region of an image to another rectangular region on 
 * another image (or the same image)
 *
 * NOTE:
 * 1. The width and height of the source and destination rectangles MUST be
 * the same
 */
extern DD_STATUS_T dd_image_copy_area(DD_IMAGE_T *src_img, DD_IMAGE_T *dst_img,
      DD_RECT_T *src_rect, DD_RECT_T *dst_rect);

/*
 * Fill color on a rectangular region of an image
 */
extern DD_STATUS_T dd_image_fill_rect(DD_IMAGE_T *image, DD_RECT_T *rect,
      uint32_t color);

/*
 * Perform alpha blending on an overlapped rectangular region of two images,
 * with the result stored on the destination image
 */
extern DD_STATUS_T dd_image_alpha_blend(DD_IMAGE_T *src_img,
      DD_IMAGE_T *dst_img, DD_RECT_T *src_rect, DD_RECT_T *dst_rect,
      DD_FLAGS_T flag, uint32_t alpha);

/*
 * Get the current LCD configuration information
 */
extern DD_STATUS_T dd_lcd_get(DD_LCD_INFO_T *info);

/*
 * Initialize the Display Director. This function must be called before
 * everything else can be called
 */
extern DD_STATUS_T dd_init(DD_LCD_MODEL_T panel_model);

/*
 * Terminate the Display Director. All associated memory will be released
 * upon return of this function
 */
extern DD_STATUS_T dd_term(void);

/* 
 * Call to check if the Display Director has been initialized
 */
extern int dd_is_initialized(void);

/*
 * Set the fields of a DD_RECT_T data structure that is used to describe a
 * source or destination rectangle
 *
 * Note the caller of this function keeps the DD_RECT_T object
 */
extern DD_STATUS_T dd_rect_set(DD_RECT_T *rect, int32_t x, int32_t y,
      int32_t width, int32_t height);

/*
 * Start a new update and return the associated update handle. There are a
 * maximum number of DD_MAX_UPDATES updates that can be started concurrently.
 * If the Display Director runs out of updates, DD_INVALID_HANDLE will be
 * returned
 * 
 * priority is NOT supported at this point, resevred for future async update
 * support
 */
extern DD_UPDATE_HANDLE_T dd_update_start(int32_t priority);

/*
 * Add an element to the display
 *
 * update - update handle
 * name - name of the owner of the element
 * layer - z-order of this element
 * dest_rect - coordinates of the rectangle on the display where the element
 * will appear
 * src_img - source image bitmap
 * src_rect - coordinates of the rectangle taken from the source image. Must
 * lie within the bounds of the source image
 * flags - alpha blending options
 * opacity - a global alpha value between 0 and 255, only meaningful when
 * flags include DD_FLAGS_ALPHA_FIXED
 * transform - a transform to be applied on the image (to be supported in the
 * future)
 *
 * NOTE:
 * 1. By specifying different widths and heights in the rectangles, a scaling
 * operation will be performed (To be supported in the future)
 *
 * 2. src_rect and dest_rect are copied into the element to be displayed;
 *    therefore, temporary variables on the stack can be used
 */
extern DD_ELEMENT_HANDLE_T dd_element_add(DD_UPDATE_HANDLE_T update,
      const char *name, int32_t layer, const DD_RECT_T *dest_rect,
      const DD_IMAGE_T *src, const DD_RECT_T *src_rect, DD_FLAGS_T flags,
      uint8_t opacity, DD_TRANSFORM_T transform);

/*
 * Remove an element from the display
 *
 * The element handle is invalidated after this call. Note that all the
 * associated image buffers must remain valid until an update has been
 * completed
 */
extern DD_STATUS_T dd_element_remove(
      DD_UPDATE_HANDLE_T update, DD_ELEMENT_HANDLE_T element);

/*
 * Submit an update and wait for it to complete before returning
 *
 * All removed images can safely be overwritten or freed after this routine
 * returns.
 */
extern DD_STATUS_T dd_update_submit_sync(
      DD_UPDATE_HANDLE_T update);

/*
 * This routine kicks off DMA data transfer from 'src_addr' to 'dst_addr' with
 * size of 'len' bytes. Launching DMA transfers with this API allows the user to
 * use the dedicated DMA channel reserved for Display Director
 *
 * Note this routine may block so it should NOT be called from an ISR or
 * softirqs
 */
extern DD_STATUS_T dd_dma_transfer(dma_addr_t src_addr, dma_addr_t dst_addr,
      uint32_t len);

#if 0

TBD

/*
 * Color expansion of an image, limited to RGB only
 */
DD_STATUS_T dd_img_color_expand(...);

/*
 * Perform raster operation such as (~SRC)&(~DST), (~SRC)&DST, ~SRC,
 * (SRC) & (~DST), etc. between the soruce and destination images and store
 * the result on the destination image
 */
DD_STATUS_T dd_img_raster(...);

/*
 * Convert from YUV to RGB color space from the source image and store the
 * result in the destination image
 *
 * BT.601-5, BT.709 with user conversion coefficients
 */
DD_STATUS_T dd_img_yuv_rgb(...);


/*
 * Put the source image in the destination image (Picture-in-picture)
 */
DD_STATUS_T dd_img_pip(...);

/*
 * Adjust contrast, brightness and color saturation of an image
 */
DD_STATUS_T dd_img_adjust(...);

/*
 * Rotate a rectangle from the source image and store the transformed
 * rectangle in the destination image (source and destination image
 * can be the same).
 */
DD_STATUS_T dd_img_rotate(...);

/*
 * Scale a rectangle from the source image and store the result in the
 * destination image (source and destination image can be the same).
 */
DD_STATUS_T dd_img_scale(...);

/*
 * Crop a JPEG image
 */
DD_STATUS_T dd_img_crop(...);

#endif

#endif /* __KERNEL__ */

#endif /* DD_H */
