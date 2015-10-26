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



#include <stdlib.h>
#include <string.h>
#include <stdlib.h>

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcdispman.h"
#include "vcdispman_ex.h"
#include "vcutil.h"

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

/******************************************************************************
Static data.
******************************************************************************/

/******************************************************************************
Static functions.
******************************************************************************/

#define CHECK_SIZE16(obj) assert((sizeof(obj)&15)==0)

// Extensions to vc_msgfifo library...

static int vc_msgfifo_read_reply_blocking(uint32_t inum, void *reply, uint32_t reply_size, void *event)
{
   VC_MSGFIFO_RESP_HEADER_T header;
   uint32_t read_bytes = (reply_size + 15) & ~15;

   if (read_bytes != reply_size) {
      assert(0);
   }

   vc_msgfifo_read_blocking(inum, &header, sizeof(header), event);
   assert(reply_size == header.ext_length);

   if (reply_size) {
      vc_msgfifo_read_blocking(inum, reply, read_bytes, event);
   }
   vc_msgfifo_read_flush(inum);

   return VC_VTOH32(header.resp_code);
}


/* Synchronise with VideoCore: used for debugging */

int vc_dispman_sync()
{
   if (_dispman_inum < 0) {
      return -1;
   }

   vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_SYNC, 0, NULL, _dispman_ievent);

   return vc_msgfifo_read_reply_blocking(_dispman_inum, 0, 0, _dispman_ievent);
}

/* Create an off-screen image and return a handle for it */

typedef struct {
        uint16_t w;
        uint16_t h;
        uint8_t format;
   uint8_t pad[11];
} VC_DISPMAN_CREATE_IMAGE_PARAMS_T;

typedef struct {
        uint32_t handle;
   uint8_t pad[12];
} VC_DISPMAN_CREATE_IMAGE_RESULT_T;

int vc_dispman_create_image(uint32_t w, uint32_t h, uint8_t format, uint32_t *pHandle)
{
   int rc;
   VC_DISPMAN_CREATE_IMAGE_PARAMS_T params;
   VC_DISPMAN_CREATE_IMAGE_RESULT_T result;

   CHECK_SIZE16(params);
   CHECK_SIZE16(result);

   if (_dispman_inum < 0) {
      return 1; // ERROR_NOT_INTIALISED
   }

   if (pHandle == 0) {
      return 1; // ERROR_BAD_POINTER
   }

   params.w = (uint16_t)w;
   params.h = (uint16_t)h;
   params.format = format;

   vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_CREATE_IMAGE, sizeof(params), &params, _dispman_ievent);

   rc = vc_msgfifo_read_reply_blocking(_dispman_inum, &result, sizeof(result), _dispman_ievent);
   if (rc) {
      return rc;
   }

   *pHandle = result.handle;

   return 0;
}

/* Destroy a previously-created image */

typedef struct {
   uint32_t handle;
   uint8_t pad[12];
} VC_DISPMAN_DESTROY_IMAGE_PARAMS_T;

int vc_dispman_destroy_image(uint32_t handle)
{
   VC_DISPMAN_DESTROY_IMAGE_PARAMS_T params;

   CHECK_SIZE16(params);

   if (_dispman_inum < 0) {
      return 1; // ERROR_NOT_INTIALISED
   }

   params.handle = handle;

   vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_DESTROY_IMAGE, sizeof(params), &params, _dispman_ievent);

   return 0;
}


/* Copy a rectangle between two images with optional transformation */

typedef struct {
   uint32_t dst_img;
   int16_t dst_x;
   int16_t dst_y;
   uint16_t width;
   uint16_t height;
   uint32_t src_img;
   int16_t src_x;
   int16_t src_y;
   uint32_t transform;
   uint8_t pad[8];
} VC_DISPMAN_BLT2_PARAMS_T;

int vc_dispman_blt2(uint32_t dest, uint32_t d_x_offset, uint32_t d_y_offset, uint32_t width, uint32_t height, uint32_t src, uint32_t s_x_offset, uint32_t s_y_offset, VC_DISPMAN_TRANSFORM_T transform)
{
   VC_DISPMAN_BLT2_PARAMS_T params;

   CHECK_SIZE16(params);

   if (_dispman_inum < 0) {
      return 1; // ERROR_NOT_INTIALISED
   }

   params.dst_img = dest;
   params.dst_x = VC_HTOV16((int16_t)d_x_offset);
   params.dst_y = VC_HTOV16((int16_t)d_y_offset);
   params.width = VC_HTOV16((uint16_t)width);
   params.height = VC_HTOV16((uint16_t)height);
   params.src_img = src;
   params.src_x = VC_HTOV16((int16_t)s_x_offset);
   params.src_y = VC_HTOV16((int16_t)s_y_offset);
   params.transform = transform;

   vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_BLT2, sizeof(params), &params, _dispman_ievent);

   return 0;
}

/* Copy raw pixel data into an image */
typedef struct {
   uint32_t img;
   int16_t x;
   int16_t y;
   uint16_t w;
   uint16_t h;
   uint32_t format;
// uint8_t data[sizeof(format)*area.h*round32(area.h)]
} VC_DISPMAN_DRAW_PIXELS_PARAMS_T;

static void host_data_write(uint8_t * data, int size, int last)
{
   struct {
      uint32_t block_size;
      uint32_t last_block;
      uint32_t pad1;
      uint32_t pad2;
   } header;

   header.block_size = size;
   header.last_block = last;
   header.pad1 = 0xFF00FF00;
   header.pad2 = 0x00FF00FF;

   vc_msgfifo_write_blocking(_dispman_inum, &header, sizeof(header), _dispman_ievent);
   vc_msgfifo_write_blocking(_dispman_inum, data, size, _dispman_ievent);
}

static int pixel_bits(int format)
{
   switch (format) {
   default:
      return 8;
   case 1: // VC_IMAGE_RGB565
      return 16;
   case 2: // VC_IMAGE_1BPP
      return 1;
   case 5: // VC_IMAGE_RGB888
      return 24;
   }
}

int vc_dispman_draw_pixels(uint32_t dest, uint32_t d_x_offset, uint32_t d_y_offset, uint32_t width, uint32_t height, void *data, int stride, int format)
{
   VC_DISPMAN_DRAW_PIXELS_PARAMS_T params;
   int line_bytes = (pixel_bits(format)*width + 7) / 8;
   int data_size = (line_bytes*height + 15) & ~15;

   CHECK_SIZE16(params);

   if (_dispman_inum < 0) {
      return 1; // ERROR_NOT_INTIALISED
   }

   params.img = VC_HTOV32(dest);
   params.x = VC_HTOV16((int16_t)d_x_offset);
   params.y = VC_HTOV16((int16_t)d_y_offset);
   params.w = VC_HTOV16((uint16_t)width);
   params.h = VC_HTOV16((uint16_t)height);
   params.format = format;

   vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_DRAW_PIXELS, sizeof(params), &params, _dispman_ievent);

   if (line_bytes == stride || height == 1) {
// Send direct from image
     host_data_write((unsigned char*)data, data_size, 1);
   } else {
// Copy to temporary area
      unsigned char *buffer = (unsigned char*)malloc(data_size);
      unsigned char *pBuff = buffer;
      char *pData = (char*)data;
      uint32_t y;
      if (!buffer) {
         return 1; // ERRROR_OUT_OF_MEMORY
      }

      for (y = 0; y < height; y++) {
         memcpy(pBuff, pData, line_bytes);
         pBuff += line_bytes;
         pData += stride;
      }

      host_data_write(buffer, data_size, 1);
      free(buffer);
   }

   return 0;
}

typedef struct {
   uint32_t display_num;
   uint32_t f_x_offset;
   uint32_t f_y_offset;
   int16_t anchor;
   int16_t font;
   uint32_t fg_colour;
   uint32_t bg_colour;
   char text[256];
} VC_DISPMAN_TEXT2_PARAMS_T;

int vc_dispman_text2(uint32_t display_num, uint32_t f_x_offset, uint32_t f_y_offset, int anchor, int font, char *text, uint32_t fg_colour, uint32_t bg_colour)
{
   // The text parameter means we can't just call send_command, so we write the
   // code out again with suitable modifications (sigh).
   VC_DISPMAN_TEXT2_PARAMS_T params;
   params.display_num = VC_HTOV32(display_num);
   params.f_x_offset = VC_HTOV32(f_x_offset);
   params.f_y_offset = VC_HTOV32(f_y_offset);
   params.anchor = VC_HTOV32(anchor);
   params.font = VC_HTOV32(font);
   params.fg_colour = VC_HTOV32(fg_colour);
   params.bg_colour = VC_HTOV32(bg_colour);
   params.text[0] = '\0';
   strncat(params.text, text, 255);
   vc_lock_obtain(_dispman_lock);
   if (_dispman_inum >= 0) {
      int data_len = sizeof(VC_DISPMAN_TEXT2_PARAMS_T) - sizeof(params.text) + strlen(params.text) + 1;
      vc_msgfifo_send_command_blocking(_dispman_inum, VC_DISPMAN_TEXT2, data_len, &params, _dispman_ievent);
   }
   vc_lock_release(_dispman_lock);
   return 0;
}
