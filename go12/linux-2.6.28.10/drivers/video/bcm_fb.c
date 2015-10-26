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
 * Description: The BCM framebuffer driver that interfaces with the Display
 * Director
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <asm/sizes.h>
#include <asm/uaccess.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/bcm_fb.h>

#define CMAP_SIZE 256

static const char *fb_name = "BCM_FB";
static struct bcm_fb *gfb = NULL;

/* LCD panel type */
#if defined(CONFIG_FB_BCM_ALL_PANEL) || defined(CONFIG_FB_BCM_SEIKO_RA169Z)
static unsigned int gpanel = DD_LCD_MODEL_SEIKO_RA169Z;
#elif defined(CONFIG_FB_BCM_SEIKO_RA169Z)
static unsigned int gpanel = DD_LCD_MODEL_SEIKO_RA167Z;
#elif defined(CONFIG_FB_BCM_SAMSUNG_LMS700KF01)
static unsigned int gpanel = DD_LCD_MODEL_SAMSUNG_LMS700KF01;
#else
static unsigned int gpanel = DD_LCD_MODEL_SEIKO_RA169Z;
#endif
module_param(gpanel, uint, 0644);

/*
 * Draw some rectangles with different colors in 'data' of 'size' bytes
 */
static void draw_something(void *data, uint32_t size)
{
   uint32_t i, len;
   uint32_t *tmp_ptr;

   if (data == NULL)
      return;

   tmp_ptr = (uint32_t *)data;
   len = size / 4;

   /*
    * Assuming 32 bpp. Fill up 4 sections of the buffer with different
    * colors by default
    */

   /* section 1 */
   for (i = 0; i < len / 4; i++)
      tmp_ptr[i] = 0xFFFF0000; /* RED */

   /* section 2 */
   for (i = len / 4; i < len / 2; i++)
      tmp_ptr[i] = 0xF000FF00; /* GREEN */

   /* section 3 */
   for (i = len / 2; i < len * 3 / 4; i++)
      tmp_ptr[i] = 0x700000FF; /* BLUE */

   /* section 4 */
   for (i = len * 3 / 4; i < len; i++)
      tmp_ptr[i] = 0xFFFFFFFF; /* WHITE */
}

/*
 * Set the pixel bitfields
 */
static int bitfields_set(struct bcm_fb *fb, struct fb_var_screeninfo *var)
{
   int rc = 0;

   var->transp.msb_right = 0;
   var->red.msb_right = 0;
   var->green.msb_right = 0;
   var->blue.msb_right = 0;

   switch (var->bits_per_pixel) {
      case 1:
      case 2:
      case 4:
      case 8:
      case 16:
         rc = -EINVAL;
         break;

      case 32:
         var->transp.offset = 24;
         var->transp.length = 8;
         var->red.offset = 16;
         var->red.length = 8;
         var->green.offset = 8;
         var->green.length = 8;
         var->blue.offset = 0;
         var->blue.length = 8;
         break;

      default:
         rc = -EINVAL;
         break;
   }

   return rc;
}

/*
 * Validate some fb_info parameters
 */
static int check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
   struct bcm_fb *fb = container_of(info, struct bcm_fb, info);
   int rc = 0;

   if (var->xres != fb->lcd_info.width || var->yres != fb->lcd_info.height ||
         var->xres_virtual != fb->lcd_info.width ||
         var->yres_virtual != fb->lcd_info.height) {
      return -EINVAL;
   }

   rc = bitfields_set(fb, var);
   if (rc != 0) {
      return rc;
   }

   return 0;
}

/*
 * Set the line length (in bytes)
 */
static int set_par(struct fb_info *info)
{
   info->fix.line_length = info->var.xres_virtual *
      info->var.bits_per_pixel / 8;

   return 0;
}

static unsigned int bitfield_extract(struct fb_bitfield *bf,
      unsigned int color, unsigned int bpp)
{
   switch (bpp) {
      case 32:
         break;

      default: /* only support 32 bpp for now */
         return 0;
   }

   color = color >> bf->offset;
   color = color << (bpp - bf->length);

   return color;
}

static unsigned int bitfield_convert(struct fb_bitfield *bf, unsigned int val,
      unsigned int bpp)
{
   switch (bpp) {
      case 32:
         break;

      default: /* only support 32 bpp for now */
         return 0;
   }

   val >>= bpp - bf->length;
   return val << bf->offset;
}

static int setcolreg(unsigned int regno, unsigned int red, unsigned int green,
       unsigned int blue, unsigned int transp, struct fb_info *info)
{
   struct bcm_fb *fb = container_of(info, struct bcm_fb, info);
   unsigned int bpp = info->var.bits_per_pixel;

   /*
	 * If greyscale is true, convert the RGB value to greyscale no mater what
    * visual is used.
	 */
   if (info->var.grayscale) {
      red = green = blue = ((19595 * red) + (38470 * green) + (7471 * blue))
         >> 16;
    }

   switch (info->fix.visual) {
      case FB_VISUAL_TRUECOLOR:
      case FB_VISUAL_DIRECTCOLOR:
         if (regno < 16) {
            unsigned int val;

            val = bitfield_convert(&fb->info.var.transp, transp, bpp);
            val |= bitfield_convert(&fb->info.var.red, red, bpp);
            val |= bitfield_convert(&fb->info.var.green, green, bpp);
            val |= bitfield_convert(&fb->info.var.blue, blue, bpp);
            
            fb->palette[regno] = val;
            
            return 0;
         }

      default:
         return -EINVAL;
   } 
}

/*
 * Map the GUI image to the framebuffer in the user space
 */
static int mmap(struct fb_info *info, struct vm_area_struct *vma)
{
   return dma_mmap_writecombine(NULL, vma, info->screen_base,
         info->fix.smem_start, info->fix.smem_len);
}

static int blank(int blank_mode, struct fb_info *info)
{
   return 0;
}

static void lcd_info_get(LCD_Info_t *lcd_info, struct fb_info *info)
{
   lcd_info->width = info->var.xres;
   lcd_info->height = info->var.yres;
   lcd_info->bitsPerPixel = info->var.bits_per_pixel;
}

static void copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
   struct bcm_fb *fb = container_of(info, struct bcm_fb, info);
   uint32_t width, height;
   DD_RECT_T src_rect, dst_rect;

   if (area->width == 0 || area->height == 0 ||
       area->sx > info->var.xres || area->dx > info->var.xres ||
       area->sy > info->var.yres || area->dy > info->var.yres) {
      printk(KERN_ERR "BCM_FB: Invalid copyarea parameters "
            "sx=%u sy=%u dx=%u dy=%u width=%u height=%u\n", area->sx,
            area->sy, area->dx, area->dy, area->width, area->height);
      return;
   }

   /* chop off some out of bound area if there's any */
   width = area->width;
   height = area->height;
   if (area->sx + width > info->var.xres) {
      width = info->var.xres - area->sx;
   }
   if (area->dx + width > info->var.xres) {
      uint32_t new_width = info->var.xres - area->dx;
      width = width < new_width ? width : new_width;
   }
   if (area->sy + height > info->var.yres) {
      height = info->var.yres - area->sy;
   }
   if (area->dy + height > info->var.yres) {
      uint32_t new_height = info->var.yres - area->dy;
      height = height < new_height ? height : new_height;
   }
   src_rect.x = area->sx;
   src_rect.y = area->sy;
   src_rect.width = width;
   src_rect.height = height;

   dst_rect.x = area->dx;
   dst_rect.y = area->dy;
   dst_rect.width = width;
   dst_rect.height = height;

   dd_image_copy_area(&fb->gui_img, &fb->gui_img, &src_rect, &dst_rect);
}

static void fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
   struct bcm_fb *fb = container_of(info, struct bcm_fb, info);
   uint32_t width, height, color;
   DD_RECT_T dst_rect;

   if (rect->width == 0 || rect->height == 0 ||
       rect->dx > info->var.xres || rect->dy > info->var.yres ||
       rect->color >= 16 || rect->rop != ROP_COPY) {
      printk(KERN_ERR "BCM_FB: Invalid fillrect parameters dx=%u dy=%u "
            "width=%u height=%u, color=%u, rop=%u\n", rect->dx, rect->dy,
            rect->width, rect->height, rect->color, rect->rop);
      return;
   }

   /* chop off some out of bound area if there's any */
   width = rect->width;
   height = rect->height;
   if (rect->dx + width > info->var.xres) {
      width = info->var.xres - rect->dx;
   }
   if (rect->dy + height > info->var.yres) {
      height = info->var.yres - rect->dy;
   }

   dst_rect.x = rect->dx;
   dst_rect.y = rect->dy;
   dst_rect.width = width;
   dst_rect.height = height;

   if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
      /* get color from the palette array */
      color = ((uint32_t *)info->pseudo_palette)[rect->color];
   }
   else {
      /* use color directly */
      color = rect->color;
   }

   dd_image_fill_rect(&fb->gui_img, &dst_rect, color);
}

/*
 * Wrapper of clcdfb_fillrect that allows the user to use the raw color
 * code directly, since clcdfb_fillrect uses the palette interface in
 * true/direct color mode
 */
static void fillrect_color(struct fb_info *info,
      LCD_FillRectColor_t *rect_c)
{
   uint32_t red, green, blue, transp;
   uint32_t bpp = info->var.bits_per_pixel;
   struct fb_fillrect rect;

   rect.dx = rect_c->dx;
   rect.dy = rect_c->dy;
   rect.width = rect_c->width;
   rect.height = rect_c->height;
   rect.rop = ROP_COPY; /* support copy only */

   if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
      /* extract color information */
      transp = bitfield_extract(&info->var.transp, rect_c->rawColor, bpp);
      red = bitfield_extract(&info->var.red, rect_c->rawColor, bpp);
      green = bitfield_extract(&info->var.green, rect_c->rawColor, bpp);
      blue = bitfield_extract(&info->var.blue, rect_c->rawColor, bpp);

      /*
       * Update color information stored in the palette, use the reserved
       * array index of 15
       */
      setcolreg(15, red, green, blue, transp, info);
      rect.color = 15;
   } else { /* use rawColor directly for other color modes */
      rect.color = rect_c->rawColor;
   }
   fillrect(info, &rect);
}

/*
 * Copy the bitmap of the GUI image to the buffered image and then signal
 * the Display Director to update the LCD
 */
static int fb_update(struct bcm_fb *fb, LCD_DirtyRows_t *dirty_rows)
{
   /* signal the Display Director that we are starting an update */
   fb->update_handle = dd_update_start(10);

   /* DMA the bitmap from the GUI image to the buffered image */
   if (dd_dma_transfer(fb->gui_img.addr, fb->buffer_img.addr,
            fb->buffer_img.size) != DD_SUCCESS)
      return -EFAULT;

   /* start the update so it can be seen on the LCD */
   if (dd_update_submit_sync(fb->update_handle) != DD_SUCCESS)
      return -EFAULT;
   
   return 0;
}

/*
 * IOCTL command processing
 */
static int ioctl_handler(struct fb_info *info, unsigned int cmd,
      unsigned long arg)
{
   int rc = 0;
   struct bcm_fb *fb = container_of(info, struct bcm_fb, info);

   switch (cmd) {
      case LCD_IOCTL_INFO:
      {
         LCD_Info_t lcd_info;

         lcd_info_get(&lcd_info, info);
         rc = copy_to_user((void *)arg, &lcd_info, sizeof(LCD_Info_t));
         if (rc != 0) {
            printk(KERN_ERR "BCM_FB: copy_to_user failed\n");
            return -EFAULT;
         }
         return 0;
      }

      case LCDFB_IOCTL_UPDATE_LCD:
      {
         return fb_update(fb, NULL);
      }

      case LCD_IOCTL_COPYAREA:
      {
         struct fb_copyarea area;

         if (copy_from_user(&area, (struct fb_copyarea *)arg,
                  sizeof(area)) != 0) {
            return -EFAULT;
         }

         copyarea(info, &area);
         return 0;
      }

      case LCD_IOCTL_FILLRECT_COLOR:
      {
         LCD_FillRectColor_t rect_c;

         if (copy_from_user(&rect_c, (LCD_FillRectColor_t *)arg,
                    sizeof(rect_c)) != 0) {
            return -EFAULT;
         }

         fillrect_color(info, &rect_c);
         return 0;
      }

      case LCD_IOCTL_IS_DIRTY_ROW_UPDATE_SUPPORTED:
      {
         return 1;
      }

      case LCD_IOCTL_DIRTY_ROW_BITS:
      {
         return fb_update(fb, NULL);
      }

      case BCM_FB_IOCTL_GET_GUI_IMAGE:
      {
          if ( copy_to_user( (void *)arg, &fb->gui_img, sizeof( fb->gui_img )) != 0 )
          {
              return -EFAULT;
          }
          break;
      }

      default:
         return -EINVAL;
   }
   return 0;
}

static struct fb_ops fb_ops = {
   .owner = THIS_MODULE,
   .fb_check_var = check_var,
   .fb_set_par = set_par,
   .fb_setcolreg = setcolreg,
   .fb_fillrect = fillrect,
   .fb_copyarea = copyarea,
   .fb_imageblit = cfb_imageblit,
   .fb_ioctl = ioctl_handler,
   .fb_mmap = mmap,
   .fb_blank = blank,
};

/*
 * Set Linux 'fb_info' parameters and register it
 */
static int fbinfo_set_register(struct bcm_fb *fb)
{
   struct fb_info *info = &fb->info;
   DD_LCD_INFO_T *lcd_info = &fb->lcd_info;

   /* zero everything */
   memset(info, 0, sizeof(struct fb_info));

   info->fbops = &fb_ops;
   info->flags = FBINFO_FLAG_DEFAULT | FBINFO_HWACCEL_COPYAREA |
      FBINFO_HWACCEL_FILLRECT;
   info->pseudo_palette = fb->palette;

   strncpy(info->fix.id, fb_name, sizeof(info->fix.id));
   info->fix.type = FB_TYPE_PACKED_PIXELS;
   info->fix.visual = FB_VISUAL_TRUECOLOR;
   info->fix.accel = FB_ACCEL_NONE;
   info->screen_base = fb->gui_img.image_data;
   info->fix.smem_start = fb->gui_img.addr;
   info->fix.smem_len = fb->gui_img.size;

   info->var.xres = lcd_info->width;
   info->var.yres = lcd_info->height;
   info->var.xres_virtual = lcd_info->width;
   info->var.yres_virtual = lcd_info->height;
   info->var.bits_per_pixel = lcd_info->bpp;
   info->var.pixclock = lcd_info->pixel_clk;
   info->var.left_margin = lcd_info->left_margin;
   info->var.right_margin = lcd_info->right_margin;
   info->var.upper_margin = lcd_info->upper_margin;
   info->var.lower_margin = lcd_info->lower_margin;
   info->var.hsync_len = lcd_info->hsync_len;
   info->var.vsync_len = lcd_info->vsync_len;
   info->var.vmode = FB_VMODE_NONINTERLACED;
   info->var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
   info->var.height = -1;
   info->var.width = -1;

   info->fix.line_length = info->var.xres_virtual *
      info->var.bits_per_pixel / 8;

   /* make sure that the bitfields are set appropriately */
   bitfields_set(fb, &info->var);

   /* allocate color map */
   fb_alloc_cmap(&info->cmap, CMAP_SIZE, 0);

   fb_set_var(info, &info->var);

   return register_framebuffer(info);
}

static int __init fb_init(void)
{
   int rc;
   struct bcm_fb *fb;

   printk(KERN_NOTICE "BCM_FB: Framebuffer driver initializing ....\n");

   fb = kzalloc(sizeof(struct bcm_fb), GFP_KERNEL);
   if (!fb) {
      printk(KERN_ERR "BCM_FB: Unable to allocate new bcm_fb struct\n");
      rc = -ENOMEM;
      goto err_out;
   }

   /* initialize the Display Director with a desired LCD panel */
   if (dd_init(gpanel) != DD_SUCCESS) {
      printk(KERN_ERR "BCM_FB: Display Director init failed\n");
      rc = -EFAULT;
      goto err_free_fb;
   }
   gfb = fb;

   /* get LCD information */
   if (dd_lcd_get(&fb->lcd_info) != DD_SUCCESS) {
      printk(KERN_ERR "BCM_FB: Unable to get LCD information\n");
      rc = -EFAULT;
      goto err_dd_term;
   }
   printk(KERN_NOTICE "BCM_FB: width=%u height=%u bpp=%u pitch=%u size=%u\n",
      fb->lcd_info.width, fb->lcd_info.height, fb->lcd_info.bpp,
      fb->lcd_info.pitch, fb->lcd_info.size);

   /* allocate the GUI image memory */
   if (dd_image_create(&fb->gui_img, fb->lcd_info.width,
         fb->lcd_info.height, DD_FORMAT_URGB888) != DD_SUCCESS) {
      printk(KERN_ERR "BCM_FB: Unable to allocate the GUI image\n");
      rc = -ENOMEM;
      goto err_dd_term;
   }

   /* allocate the buffer image memory */
   if (dd_image_create(&fb->buffer_img, fb->lcd_info.width,
         fb->lcd_info.height, DD_FORMAT_URGB888) != DD_SUCCESS) {
      printk(KERN_ERR "BCM_FB: Unable to allocate the Buffer image\n");
      rc = -ENOMEM;
      goto err_free_gui;
   }

   rc = fbinfo_set_register(fb);
   if (rc != 0) {
      printk(KERN_ERR "BCM_FB: Unable to set/configure fb_info\n");
      goto err_free_buffer;
   }

   /* add the buffered image to the Display Director */
   {
      int layer = 0; /* add it to the bottom layer */
      DD_RECT_T src_rect, dst_rect;

      /* make the element the entire LCD screen */
      src_rect.x = 0;
      src_rect.y = 0;
      src_rect.width = fb->lcd_info.width;
      src_rect.height = fb->lcd_info.height;

      memcpy(&dst_rect, &src_rect, sizeof(DD_RECT_T));
      
      fb->update_handle = dd_update_start(10);
      fb->element_handle = dd_element_add(0, "fb", layer, &dst_rect,
            &fb->buffer_img, &src_rect, 0, 0, 0);
      if (fb->element_handle == DD_INVALID_HANDLE) {
         printk(KERN_ERR "BCM_FB: Unable to add the buffered image to "
               "Display Director\n");
         dd_update_submit_sync(0);
         rc = -EFAULT;
         goto err_free_buffer;
      }
      dd_update_submit_sync(fb->update_handle);

      /* draw something on the GUI image */
      draw_something(fb->gui_img.image_data, fb->gui_img.size);

      /* update the LCD */
      fb_update(fb, NULL);
   }

   return 0;

err_free_buffer:
   dd_image_delete(&fb->buffer_img);
err_free_gui:
   dd_image_delete(&fb->gui_img);
err_dd_term:
   dd_term();
err_free_fb:
   kfree(fb);
   gfb = NULL;
err_out:
   return rc;
}
module_init(fb_init);

static void __exit fb_exit(void)
{
   struct bcm_fb *fb = gfb;

   unregister_framebuffer(&fb->info);
   framebuffer_release(&fb->info);
   dd_image_delete(&fb->buffer_img);
   dd_image_delete(&fb->gui_img);
   dd_term();
   kfree(fb);

   return;
}
module_exit(fb_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Framebuffer Driver");
MODULE_LICENSE("GPL");
