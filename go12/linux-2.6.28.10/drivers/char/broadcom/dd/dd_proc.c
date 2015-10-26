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
 * Description: Display Director procfs
 */

#include <linux/proc_fs.h>
#include <asm/sizes.h>
#include <asm/uaccess.h>

#include "dd_priv.h"
#include "dd_proc.h"
#include "dd_misc.h"

#define MAX_PROC_BUF_SIZE    256

/*
 * Procfs directory structure
 */
typedef struct proc_dir {
   struct proc_dir_entry *parent;
#if DD_VIDEO_DEMO
   struct proc_dir_entry *vdemo;
#endif
} PROC_DIR_T;

/* procfs */
static PROC_DIR_T proc_dir;
#if DD_VIDEO_DEMO
static DD_IMAGE_T mask_img;
static DD_ELEMENT_HANDLE_T element_handle;
#endif

static int proc_setup_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Display Director is %s\n",
         dd_is_initialized() ? "initialized" : "NOT initialized");
 
   return len;
}

static int proc_setup_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   unsigned int enable, model;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "DD_PROC: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u %u", &enable, &model) != 2) {
      printk(KERN_WARNING "DD_PROC: proc write syntax error\n");
      return count;
   }

   if (enable != 0) {
      if (!dd_is_initialized())
         dd_init(model);
   } else {
      if (dd_is_initialized())
         dd_term();
   }

   return count;
}

static int proc_lcd_info_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   unsigned int len = 0;
   DD_LCD_INFO_T info;

   if (off > 0)
      return 0;

   dd_lcd_get(&info);

   len += sprintf(buffer + len, "Display Director LCD configuration is:\n");
   len += sprintf(buffer + len, "resolution = %u x %u\n", info.width,
         info.height);
   len += sprintf(buffer + len, "bits per pixel = %u\n", info.bpp);
   len += sprintf(buffer + len, "pitch = %u bytes\n", info.pitch);
   len += sprintf(buffer + len, "max buffer size = %u bytes\n", info.size);
   len += sprintf(buffer + len, "pixel clock = %u Hz\n", info.pixel_clk);
   len += sprintf(buffer + len, "left margin = %u\n", info.left_margin);
   len += sprintf(buffer + len, "right margin = %u\n", info.right_margin);
   len += sprintf(buffer + len, "upper margin = %u\n", info.upper_margin);
   len += sprintf(buffer + len, "lower margin = %u\n", info.lower_margin);
   len += sprintf(buffer + len, "hsync length = %u\n", info.hsync_len);
   len += sprintf(buffer + len, "vsync length = %u\n", info.vsync_len);

   return len;
}

static int proc_element_disp_list_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   unsigned int len = 0;
   DD_ELEMENT_T *element;
   DD_IMAGE_T *image;

   if (off > 0)
      return 0;

   if (dd_is_initialized() == 0) {
      printk(KERN_ERR "Display Director is not initialized\n");
      return len;
   }

   len += sprintf(buffer + len, "Current elements are:\n");

   element = dd_element_display_get();
   while (element != NULL) {
      image = &element->src_img;

      len += sprintf(buffer + len, "Element %u:", element->handle);
      len += sprintf(buffer + len, " owner=%s state=%d flags=0x%x opacity=%u "
            "layer=%u\n", element->name, atomic_read(&element->state),
            element->flags, element->opacity, element->layer);
      len += sprintf(buffer + len, "source image format=%d width=%d "
            "height=%d pitch=%d size=%d image_data=0x%08x addr=0x%08x\n",
            image->format, image->width, image->height, image->pitch,
            image->size, (unsigned int)image->image_data, image->addr);
      len += sprintf(buffer + len, "source rect x=%u y=%u width=%u height=%u\n",
            element->src_rect.x, element->src_rect.y, element->src_rect.width,
            element->src_rect.height);
      len += sprintf(buffer + len, "dest rect x=%u y=%u width=%u height=%u\n\n",
            element->dest_rect.x, element->dest_rect.y,
            element->dest_rect.width, element->dest_rect.height);

      /* next element on the display list */
      element = element->display_next;
   }

   return len;
}
#if DD_VIDEO_DEMO
static int proc_mask_img_remove_write(struct file *file,
      const char __user *buffer,
      unsigned long count, void *data)
{
   unsigned int remove;
   DD_UPDATE_HANDLE_T update_handle;

   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "DD_PROC: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &remove) != 1) {
      printk(KERN_WARNING "maskImageRemove: Syntax -> echo 1 > maskImageRemove\n");
      return count;
   }

   if (remove == 0)
      return count;

   if (dd_is_initialized() == 0) {
      printk(KERN_ERR "maskImageRemove: Display Director is not initialized\n");
      return count;
   }

   if (mask_img.image_data == NULL) {
      printk(KERN_WARNING "maskImageRemove: Masking image does not exist\n");
      return count;
   }

   /* now remove the masking image from the display list */
   update_handle = dd_update_start(1);
   dd_element_remove(update_handle, element_handle);
   dd_update_submit_sync(update_handle);

   /* free the image buffer */
   dd_image_delete(&mask_img);

   return count;
}
#endif

#if DD_VIDEO_DEMO
static int proc_mask_img_add_write(struct file *file,
      const char __user *buffer,
      unsigned long count, void *data)
{
   unsigned int x, y, width, height, radius;
   DD_ELEMENT_T *element;
   DD_RECT_T rect;
   int layer;
   DD_UPDATE_HANDLE_T update_handle;

   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "DD_PROC: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u %u %u %u %u %d", &x, &y, &width, &height,
            &radius, &layer) != 6) {
      printk(KERN_WARNING "maskImageAdd: Syntax -> echo <x> <y> <width> "
            "<height> <radius> <layer> > maskImageAdd\n");
      return count;
   }

   if (dd_is_initialized() == 0) {
      printk(KERN_ERR "maskImageAdd: Display Director is not initialized\n");
      return count;
   }

   /* parameter validation */
   if (width == 0 || height == 0) {
      printk(KERN_WARNING "maskImageAdd: Invalid parameters\n");
      return count;
   }

   if ((radius > width / 2) || (radius > height / 2)) {
      printk(KERN_WARNING "maskImageAdd: Invalid parameters\n");
      return count;
   }

   /* TODO: mask sure GUI buffer is already there */
   element = dd_element_display_get();
   if (element == NULL) {
      printk(KERN_WARNING "maskImageAdd: Missing bottom layer\n");
      return count;
   }

   if (mask_img.image_data != NULL) {
      printk(KERN_WARNING "maskImageAdd: Masking image already exists\n");
      return count;
   }

   /* allocate masking image buffer */
   if (dd_image_create(&mask_img, width, height, DD_FORMAT_ARGB888)
         != DD_SUCCESS) {
      printk(KERN_WARNING "maskImageAdd: Unable to allocate masking image\n");
      return count;
   }

   /* copy defined rectangular region of GUI into the masking buffer */
   rect.x = x;
   rect.y = y;
   rect.width = width;
   rect.height = height;
   if (dd_image_copy_area(&element->src_img, &mask_img, &rect, &rect)
         != DD_SUCCESS) {
      printk(KERN_WARNING "maskImageAdd: GUI -> Masking image copyarea failed\n");
      return count;
   }
   
   /* fill in alpha pixels in the masking image */
   if (round_corners((uint32_t *)mask_img.image_data, width, height, radius)
         != 0) {
      printk(KERN_WARNING "maskImageAdd: Masking image alpha embedding failed\n");
      return count;
   }

   /* now add the masking image to the display list */
   rect.x = 0;
   rect.y = 0;
   update_handle = dd_update_start(1);
   element_handle = dd_element_add(update_handle, "mask", layer, &rect,
      &mask_img, &rect, DD_FLAGS_ALPHA_SOURCE, 0, 0);
   dd_update_submit_sync(update_handle);

   return count;
}
#endif

#if DD_VIDEO_DEMO
static int proc_video_move_start_write(struct file *file,
      const char __user *buffer,
      unsigned long count, void *data)
{
   unsigned int xsize, ysize, xstep, nregions;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "DD_PROC: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u %u %u %u",
            &xsize, &ysize, &xstep, &nregions) != 4) {
      printk(KERN_WARNING "videoMoveStart: Syntax -> echo <xsize> <ysize> "
            "<xstep> <nregions> > videoMoveStart\n");
      return count;
   }

   dd_video_move_start(xsize, ysize, xstep, nregions);

   return count;
}
#endif

#if DD_VIDEO_DEMO
static int proc_video_move_stop_write(struct file *file,
      const char __user *buffer,
      unsigned long count, void *data)
{
   unsigned int stop;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "DD_PROC: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &stop) != 1) {
      printk(KERN_WARNING "videoMoveStop: Syntax -> echo 1 > videoMoveStop\n");
      return count;
   }

   if (stop > 0)
      dd_video_move_stop();

   return count;
}
#endif

/*
 * Set up proc entries
 */
DD_STATUS_T dd_proc_init(void)
{
   struct proc_dir_entry *setup;
   struct proc_dir_entry *lcd_info;
   struct proc_dir_entry *element_disp_list;
#if DD_VIDEO_DEMO
   struct proc_dir_entry *mask_img_add;
   struct proc_dir_entry *mask_img_remove;
   struct proc_dir_entry *video_move_start;
   struct proc_dir_entry *video_move_stop;
#endif

   proc_dir.parent = proc_mkdir("dd", NULL);
#if DD_VIDEO_DEMO
   proc_dir.vdemo = proc_mkdir("vdemo", proc_dir.parent);
#endif

   setup = create_proc_entry("setup", 0644, proc_dir.parent);
   if (setup == NULL) {
      return DD_FAIL;
   }
   setup->read_proc = proc_setup_read;
   setup->write_proc = proc_setup_write;
   setup->data = NULL;

   lcd_info = create_proc_entry("lcdInfo", 0644, proc_dir.parent);
   if (lcd_info == NULL) {
      return DD_FAIL;
   }
   lcd_info->read_proc = proc_lcd_info_read;
   lcd_info->write_proc = NULL;
   lcd_info->data = NULL;

   element_disp_list = create_proc_entry("elementDisplayList", 0644,
         proc_dir.parent);
   if (element_disp_list == NULL) {
      return DD_FAIL;
   }
   element_disp_list->read_proc = proc_element_disp_list_read;
   element_disp_list->write_proc = NULL;
   element_disp_list->data = NULL;

#if DD_VIDEO_DEMO
   mask_img_add = create_proc_entry("maskImageAdd", 0644,
         proc_dir.vdemo);
   if (mask_img_add == NULL) {
      return DD_FAIL;
   }
   mask_img_add->read_proc = NULL;
   mask_img_add->write_proc = proc_mask_img_add_write;
   mask_img_add->data = NULL;

   mask_img_remove = create_proc_entry("maskImageRemove", 0644,
         proc_dir.vdemo);
   if (mask_img_remove == NULL) {
      return DD_FAIL;
   }
   mask_img_remove->read_proc = NULL;
   mask_img_remove->write_proc = proc_mask_img_remove_write;
   mask_img_remove->data = NULL;

   video_move_start = create_proc_entry("videoMoveStart", 0644,
         proc_dir.vdemo);
   if (video_move_start == NULL) {
      return DD_FAIL;
   }
   video_move_start->read_proc = NULL;
   video_move_start->write_proc = proc_video_move_start_write;
   video_move_start->data = NULL;

   video_move_stop = create_proc_entry("videoMoveStop", 0644,
         proc_dir.vdemo);
   if (video_move_stop == NULL) {
      return DD_FAIL;
   }
   video_move_stop->read_proc = NULL;
   video_move_stop->write_proc = proc_video_move_stop_write;
   video_move_stop->data = NULL;
#endif

   return DD_SUCCESS;
}

DD_STATUS_T dd_proc_term(void)
{
#if DD_VIDEO_DEMO
   remove_proc_entry("maskImageAdd", proc_dir.vdemo);
   remove_proc_entry("maskImageRemove", proc_dir.vdemo);
   remove_proc_entry("videoMoveStart", proc_dir.vdemo);
   remove_proc_entry("videoMoveStop", proc_dir.vdemo);
   remove_proc_entry("vdemo", proc_dir.parent);
#endif
   remove_proc_entry("setup", proc_dir.parent);
   remove_proc_entry("lcdInfo", proc_dir.parent);
   remove_proc_entry("elementList", proc_dir.parent);
   remove_proc_entry("dd", NULL);

   return DD_SUCCESS;
}
