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
 * Description: The Display Director driver
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/string.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/dd/dd.h>

#include "dd_priv.h"
#include "dd_util.h"
#include "dd_dma.h"
#include "dd_ge.h"
#include "dd_lcd.h"
#include "dd_proc.h"
#include "dd_misc.h"

/*
 * Update state
 */
typedef enum update_state {
   /* free and ready to be used */
   UPDATE_STATE_FREE,
   /* currently being used */
   UPDATE_STATE_USED 
} UPDATE_STATE_T;

typedef struct element_cfg {
   /* pointer to the pool that stores all elements */
   DD_ELEMENT_T *pool;
   /* list of elements currently being displayed */
   DD_ELEMENT_T *display;
} ELEMENT_CFG_T;

/*
 * The update data structure
 */
typedef struct update UPDATE_T;

struct update {
   DD_UPDATE_HANDLE_T handle;
   int32_t priority;
   /* update state, used/free */
   volatile atomic_t state;
   /* pointer to new elements to be added in this update */
   DD_ELEMENT_T *elements_new; 
   /* list of elements to be removed in this update */
   DD_ELEMENT_HANDLE_T elements_rm_list[DD_MAX_ELEMENTS];
};

typedef struct update_cfg {
   /* pointer to the update pool */
   UPDATE_T *pool;  
} UPDATE_CFG_T;

typedef struct dd_cfg {
   /* flag to indicate if the Display Director has been initialized */
   atomic_t is_initialized;
   struct semaphore lock;
   DD_LCD_INFO_T lcd_info;
   /* active bitmap image being displayed */
   DD_IMAGE_T active_img;
   /* inactive image to prevent the "tearing" effect */
   DD_IMAGE_T inactive_img;
   UPDATE_CFG_T update;
   ELEMENT_CFG_T element;
} DD_CFG_T;

// The DD_FILE_DATA_T struct is stored in the per-file private_data.
typedef struct
{
    DD_IMAGE_T  lastImage;

} DD_FILE_DATA_T;

static DD_CFG_T g_dd;
static volatile atomic_t video_is_on;

#if DD_VIDEO_DEMO
static volatile atomic_t move_video;
static BOUNCE_STATE move_pattern;
#endif

/*
 * Swap the active/inactive images and get the new active image to be
 * displayed on the LCD
 */
static DD_STATUS_T display_update(DD_CFG_T *dd)
{
   void *tmp_ptr;
   uint32_t tmp_addr;

   /* swap image addresses and display it on the LCD */
   tmp_ptr = dd->active_img.image_data;
   tmp_addr = dd->active_img.addr;
   dd->active_img.image_data = dd->inactive_img.image_data;
   dd->active_img.addr = dd->inactive_img.addr;
   dd->inactive_img.image_data = tmp_ptr;
   dd->inactive_img.addr = tmp_addr;

   return dd_lcd_update(dd->active_img.addr);
}

/*
 * Engage the Graphic Engine
 */
static inline DD_STATUS_T ge_engage(const DD_IMAGE_T *src_img,
      const DD_IMAGE_T *dst_img, const DD_RECT_T *src_rect,
      const DD_RECT_T *dst_rect, DD_GE_OPERATION_T operation,
      uint32_t opt1, int32_t opt2)
{
   DD_GE_PARAM_T ge_param;

   memset(&ge_param, 0, sizeof(ge_param));

   if (operation == DD_GE_OP_ALPHA_BLEND) {
      if (opt2 == DD_FLAGS_ALPHA_SOURCE) {
         ge_param.src_format = DD_FORMAT_ARGB888;
         ge_param.dst_format = DD_FORMAT_ARGB888;
      } else if (opt2 == DD_FLAGS_ALPHA_FIXED) {
         ge_param.src_format = DD_FORMAT_URGB888;
         ge_param.dst_format = DD_FORMAT_URGB888;
      } else {
         ge_param.src_format = src_img->format;
         ge_param.dst_format = dst_img->format;
      }
   } else {
      ge_param.src_format = src_img->format;
      ge_param.dst_format = dst_img->format;
   }

   ge_param.sx = src_rect->x;
   ge_param.sy = src_rect->y;
   ge_param.dx = dst_rect->x;
   ge_param.dy = dst_rect->y;
   ge_param.width = dst_rect->width;
   ge_param.height = dst_rect->height;
   ge_param.src_pitch = src_img->pitch;
   ge_param.dst_pitch = dst_img->pitch;
   ge_param.src_addr = src_img->addr;
   ge_param.dst_addr = dst_img->addr;

   switch (operation) {
      case DD_GE_OP_ALPHA_BLEND:
         ge_param.option.alpha = opt1;
         return dd_ge_alphablend(&ge_param);

      case DD_GE_OP_COPY_AREA:
         return dd_ge_copyarea(&ge_param);

      case DD_GE_OP_FILL_COLOR:
         ge_param.option.color = opt1;
         return dd_ge_fillcolor(&ge_param);

      default:
         return DD_FAIL;
   }

   return DD_SUCCESS;
}

/*
 * Initialize the update confgiuration and allocate memories for the update
 * pool
 */
static DD_STATUS_T update_init(UPDATE_CFG_T *update)
{
   unsigned int i, index;

   update->pool = kzalloc(sizeof(UPDATE_T) * DD_MAX_UPDATES,
         GFP_KERNEL);
   if (!update->pool) {
      printk(KERN_INFO "DD: Unable allocate memories for the update pool\n");
      return DD_FAIL;
   }

   for (index = 0; index < DD_MAX_UPDATES; index++) {
      update->pool[index].handle = index;
      update->pool[index].priority = 0;
      atomic_set(&update->pool[index].state, UPDATE_STATE_FREE);
      update->pool[index].elements_new = NULL;
      for (i = 0; i < DD_MAX_ELEMENTS; i++) {
         update->pool[index].elements_rm_list[i] = DD_INVALID_HANDLE;
      }
   }

   return DD_SUCCESS;
}

/*
 * Release the update pool memory
 */
static DD_STATUS_T update_term(UPDATE_CFG_T *update)
{
   if (update->pool) {
      kfree(update->pool);
      update->pool = NULL;
   }
   return DD_SUCCESS;
}

/*
 * Find a free update from the pool, reserve it, and return its handle. If
 * nothing free can be found, return an invalid handle
 */
static DD_UPDATE_HANDLE_T update_reserve(UPDATE_CFG_T *update,
      int32_t priority)
{
   unsigned int i, index;

   /* go through the update pool and find a free update */
   for (index = 0; index < DD_MAX_UPDATES; index++) {
      if (atomic_read(&update->pool[index].state) == UPDATE_STATE_FREE) {
         atomic_set(&update->pool[index].state, UPDATE_STATE_USED);
         break;
      }
   }

   /* no free updates can be found */
   if (index == DD_MAX_UPDATES)
      return DD_INVALID_HANDLE;

   update->pool[index].priority = priority;
   update->pool[index].elements_new = NULL;
   for (i = 0; i < DD_MAX_ELEMENTS; i++) {
      update->pool[index].elements_rm_list[i] = DD_INVALID_HANDLE;
   }

   return update->pool[index].handle;
}

/*
 * Release an update and return it back to the pool
 */
static DD_STATUS_T update_release(UPDATE_T *update)
{
   atomic_set(&update->state, UPDATE_STATE_FREE);
   
   return DD_SUCCESS;
}

/*
 * Find an update using the update handle and return its address. If nothing
 * can be found, return NULL
 */
static UPDATE_T *update_find(UPDATE_CFG_T *update, DD_UPDATE_HANDLE_T handle)
{
   if (handle >= DD_MAX_UPDATES)
      return NULL;

   if (update->pool[handle].handle != handle)
      return NULL;

   return &update->pool[handle];
}

/*
 * Add an element to the update list
 */
static DD_STATUS_T update_add_element(UPDATE_T *update, DD_ELEMENT_T *element)
{
   /* always add to the head of the list */
   element->update_next = update->elements_new;
   update->elements_new = element;

   return DD_SUCCESS;
}

/*
 * Add an element to the update removal list
 */
static DD_STATUS_T update_remove_element(UPDATE_T *update,
      DD_ELEMENT_HANDLE_T element_handle)
{
   unsigned int index;

   for (index = 0; index < DD_MAX_ELEMENTS; index++) {
      if (update->elements_rm_list[index] == DD_INVALID_HANDLE)
         break;
   }

   if (index >= DD_MAX_ELEMENTS)
      return DD_FAIL;

   update->elements_rm_list[index] = element_handle;
   return DD_SUCCESS;
}

/*
 * Initialize the element confgiuration and allocate memories for the element
 * pool
 */
static DD_STATUS_T element_init(ELEMENT_CFG_T *element)
{
   unsigned int index;

   element->pool = kzalloc(sizeof(DD_ELEMENT_T) * DD_MAX_ELEMENTS,
         GFP_KERNEL);
   if (!element->pool) {
      printk(KERN_INFO "DD: Could not allocate element pool\n");
      return DD_FAIL;
   }

   element->display = NULL;

   for (index = 0; index < DD_MAX_ELEMENTS; index++) {
      element->pool[index].handle = index;
      atomic_set(&element->pool[index].state, DD_ELEMENT_STATE_FREE);

   }

   return DD_SUCCESS;
}

/*
 * Free the element pool memory
 */
static DD_STATUS_T element_term(ELEMENT_CFG_T *element)
{
   if (element->pool) {
      kfree(element->pool);
      element->pool = NULL;
   }
   element->display = NULL;
   return DD_SUCCESS;
}

/*
 * Find and reserve a free element from the pool. If nothing can be found
 * return NULL otherwise return the address of the element
 */
static DD_ELEMENT_T *element_reserve(ELEMENT_CFG_T *element)
{
   unsigned int index;

   for (index = 0; index < DD_MAX_ELEMENTS; index++) {
      if (atomic_read(&element->pool[index].state) == DD_ELEMENT_STATE_FREE)
         break;
   }

   /* no free element can be found */
   if (index >= DD_MAX_ELEMENTS)
      return NULL;

   atomic_set(&element->pool[index].state, DD_ELEMENT_STATE_UPDATE);

   element->pool[index].update_next = NULL; 
   element->pool[index].display_next = NULL;
   memset(element->pool[index].name, 0, sizeof(element->pool[index].name));

   return &element->pool[index];
}

/*
 * Release an element and return it back to the pool
 */
static DD_STATUS_T element_release(DD_ELEMENT_T *element)
{
   atomic_set(&element->state, DD_ELEMENT_STATE_FREE);
   element->update_next = NULL; 
   element->display_next = NULL;
   
   if (strncmp(element->name, "vdec", 4) == 0) {
      atomic_set(&video_is_on, 0);
   }
   
   return DD_SUCCESS;
}

/*
 * Add an element to the display list. The display list is sorted
 * according to the z-order
 */
static void element_add_to_display_single(ELEMENT_CFG_T *cfg,
      DD_ELEMENT_T *element)
{
   DD_ELEMENT_T *curr;
   DD_ELEMENT_T *prev;

   atomic_set(&element->state, DD_ELEMENT_STATE_DISPLAY);

   /* Case 1: Nothing on the display list */
   if (cfg->display == NULL) {
      cfg->display = element;
      element->display_next= NULL;
      return;
   }

   /* Case 2: The new element has the lowest z-order, add it to the head */
   if (element->layer < cfg->display->layer) {
      element->display_next = cfg->display;
      cfg->display = element;
      return;
   }

   /*
    * Case 3: The general case, the active list has at least one element and
    * the new element doesn't have the lowest z-order
    */
   prev = cfg->display;
   curr = prev->display_next;
   while (1) {
      if (curr == NULL)
         break;
      if (element->layer < curr->layer)
         break;

      curr = curr->display_next;
      prev = prev->display_next;
   }

   prev->display_next = element;
   element->display_next = curr;
}

/*
 * Go through the new element list of a given update and add all of them to
 * the display list
 */
static void element_add_to_display(UPDATE_T *update, ELEMENT_CFG_T *cfg)
{
   DD_ELEMENT_T *cur_element = update->elements_new;

   /* go through the new element list and add them to the display list */
   while (cur_element != NULL) {
      element_add_to_display_single(cfg, cur_element);
      cur_element = cur_element->update_next;
   }
}

/*
 * Remove an element from the display list
 */
static void element_remove_from_display_single(ELEMENT_CFG_T *cfg,
      DD_ELEMENT_HANDLE_T handle)
{
   DD_ELEMENT_T *curr = cfg->display;
   DD_ELEMENT_T *prev = NULL;

   /* go through the display list to find a matched handle */
   while (curr != NULL) {
      if (curr->handle == handle)
         break;
      prev = curr;
      curr = curr->display_next;
   }

   /* nothing can be found */
   if (curr == NULL)
      return;

   /* got it, now remove it from the list */
   prev->display_next = curr->display_next;
   element_release(curr);
}

/*
 * Go through the element removal list of a given update and remove them from
 * the display
 */
static void element_remove_from_display(UPDATE_T *update, ELEMENT_CFG_T *cfg)
{
   unsigned int index;

   /* go through the element removal list and remove them from the display */
   for (index = 0; index < DD_MAX_ELEMENTS; index++) {
      if (update->elements_rm_list[index] == DD_INVALID_HANDLE)
         break;
      element_remove_from_display_single(cfg,
            update->elements_rm_list[index]);
   }
}

static int dd_open( struct inode *inode, struct file *file )
{
    DD_FILE_DATA_T  *fileData;

    // Allocate a per-open data structure

    if (( fileData = kcalloc( 1, sizeof( *fileData ), GFP_KERNEL )) == NULL )
    {
        return -ENOMEM;
    }

    file->private_data = fileData;

    return 0;
}

static int dd_release( struct inode *inode, struct file *file )
{
    DD_FILE_DATA_T  *fileData = file->private_data;

    kfree( fileData );

    return 0;
}

static int dd_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
   DD_STATUS_T dd_status;

   switch (cmd) {
      case DD_IOCTL_INIT:
      {
         dd_status = dd_init(arg);
         if (dd_status != DD_SUCCESS)
            return DD_FAIL;

         return DD_SUCCESS;
      }

      case DD_IOCTL_TERM:
      {
         dd_status = dd_term();
         if (dd_status != DD_SUCCESS)
            return DD_FAIL;

         return DD_SUCCESS;
      }

      case DD_IOCTL_IS_INIT:
      {
         return dd_is_initialized();
      }

      case DD_IOCTL_LCD_GET:
      {
         DD_LCD_INFO_T lcd_info;
   
         dd_lcd_info_get(&lcd_info);

         if (copy_to_user((void *)arg, &lcd_info, sizeof(lcd_info)) != 0) {
            printk(KERN_ERR "DD: copy_to_user failed for "
                  "ioctl=DD_IOCTL_LCD_GET\n");
            return DD_FAIL;
         }

         return DD_SUCCESS;
      }

      case DD_IOCTL_IMG_CREATE:
      {
         DD_IMAGE_T      img;
         DD_FILE_DATA_T *fileData = filp->private_data;

         if (copy_from_user(&img, (DD_IMAGE_T *)arg,
                  sizeof(DD_IMAGE_T)) != 0) {
            printk(KERN_ERR "DD: copy_from_user failed for "
                  "ioctl=DD_IOCTL_IMG_CREATE\n");
            return DD_FAIL;
         }

         if (dd_image_create(&img, img.width, img.height, img.format)
               != DD_SUCCESS) {
            return DD_FAIL;
         }

         // Save away the image information into the file private area.
         // We do this so that the user can mmap the image data into 
         // user space. Currently, we only allow the last created image
         // to be mmaped.

         fileData->lastImage = img;
   
         if (copy_to_user((void *)arg, &img, sizeof(DD_IMAGE_T)) != 0) {
            printk(KERN_ERR "DD: copy_to_user failed for "
                  "ioctl=DD_IOCTL_IMG_CREATE\n");
            return DD_FAIL;
         }

         return DD_SUCCESS;
      }

      case DD_IOCTL_IMG_DEL:
      {
         DD_IMAGE_T img;
         DD_FILE_DATA_T *fileData = filp->private_data;

         if (copy_from_user(&img, (DD_IMAGE_T *)arg,
                  sizeof(DD_IMAGE_T)) != 0) {
            printk(KERN_ERR "DD: copy_from_user failed for "
                  "ioctl=DD_IOCTL_IMG_DEL\n");
            return DD_FAIL;
         }

         if (dd_image_delete(&img) != DD_SUCCESS) {
            return DD_FAIL;
         }

         if ( fileData->lastImage.image_data == img.image_data )
         {
             // If this image corresponds to last image that was created, 
             // then remove our reference to that mmap will fail.
             
             memset( &fileData->lastImage, 0, sizeof( fileData->lastImage ));
         }
         return DD_SUCCESS;
      }

      case DD_IOCTL_ELEMENT_ADD:
      {
         DD_ELEMENT_T element;

         if (copy_from_user(&element, (DD_ELEMENT_T *)arg,
                  sizeof(DD_ELEMENT_T)) != 0) {
            printk(KERN_ERR "DD: copy_from_user failed for "
                  "ioctl=DD_IOCTL_ELEMENT_ADD\n");
            return DD_INVALID_HANDLE;
         }

         return dd_element_add(0, element.name, element.layer,
               &element.dest_rect, &element.src_img, &element.src_rect,
               element.flags, element.opacity, element.transform);
      }

      case DD_IOCTL_ELEMENT_RMV:
      {
         DD_ELEMENT_HANDLE_T handle = arg;

         if (dd_element_remove(0, handle) != DD_SUCCESS) {
            return DD_FAIL;
         }

         return DD_SUCCESS;
      }

      case DD_IOCTL_UPDATE_START:
      {
         int32_t priority = arg;

         return dd_update_start(priority);
      }

      case DD_IOCTL_UPDATE_SYNC:
      {
         DD_UPDATE_HANDLE_T update_handle = arg;
         return dd_update_submit_sync(update_handle);
      }

      case DD_IOCTL_COPYAREA:
      {
          DD_COPYAREA_T copyArea;

          if ( copy_from_user( &copyArea, (void *)arg, sizeof( copyArea )) != 0 )
          {
              return -EFAULT;
          }

          return dd_image_copy_area( &copyArea.srcImage, &copyArea.dstImage, &copyArea.srcRect, &copyArea.dstRect );
      }

      default:
         return -ENOTTY;
   }
   return 0;
}

int dd_mmap( struct file *file, struct vm_area_struct * vma )
{
    DD_FILE_DATA_T  *fileData = file->private_data;
    DD_IMAGE_T      *lastImage = &fileData->lastImage;

    if ( lastImage->image_data == NULL )
    {
        return -EINVAL;
    }

    return dma_mmap_writecombine( NULL, vma, lastImage->image_data, lastImage->addr, lastImage->size );

} // dd_mmap

struct file_operations dd_drv_fops =
{
   owner:   THIS_MODULE,
   ioctl:   dd_ioctl,
   open:    dd_open,
   release: dd_release,
   mmap:    dd_mmap
};

static int __init dd_driver_init(void)
{
   printk(KERN_NOTICE "DD: Display Director initializing...\n");

   if (register_chrdev(BCM_DD_MAJOR, "dd", &dd_drv_fops) < 0) {
      printk(KERN_ERR "DD: register_chrdev failed for major %u\n",
            BCM_DD_MAJOR);
   }

   if (dd_proc_init() != DD_SUCCESS) {
      printk(KERN_ERR "DD: procfs initialization failed\n");
   }

   return 0;
}

static void __exit dd_driver_exit(void)
{
   dd_proc_term();
   unregister_chrdev(BCM_DD_MAJOR, "dd");
}

module_init(dd_driver_init);
module_exit(dd_driver_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Display Director");
MODULE_LICENSE("GPL");

/*
 * Public Functions within DD
 */

DD_ELEMENT_T *dd_element_display_get(void)
{
   DD_CFG_T *dd = &g_dd;

   return dd->element.display;
}

#if DD_VIDEO_DEMO
DD_STATUS_T dd_video_move_start(unsigned int xsize, unsigned int ysize,
      unsigned int xstep, unsigned int nregions)
{
   bounce_init(&move_pattern, xsize, ysize, xstep, nregions);
   atomic_set(&move_video, 1);

   return DD_SUCCESS;
}
#endif

#if DD_VIDEO_DEMO
void dd_video_move_stop(void)
{
   atomic_set(&move_video, 0);
}
#endif

/*
 * Public Functions outside DD
 */

DD_STATUS_T dd_image_create(DD_IMAGE_T *img, unsigned short width,
      unsigned short height, DD_FORMAT_T format)
{
   uint32_t bytes_per_pixel = dd_util_pixel_bytes_calc(format);

   img->format = format;
   img->width = width;
   img->height = height;
   img->pitch = width * bytes_per_pixel;
   img->size = width * height * bytes_per_pixel;

   /* allocate memory for the bitmap of the image */
   img->image_data = dma_alloc_writecombine(NULL, img->size, &img->addr,
         GFP_KERNEL);
	if (img->image_data == NULL) {
		printk(KERN_ERR "DD: Unable to allocate memory for a new image\n");
      return DD_FAIL;
   }

   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_image_create);

DD_STATUS_T dd_image_delete(DD_IMAGE_T *img)
{
   if (img->image_data != NULL) {
      dma_free_writecombine(NULL, img->size, img->image_data, img->addr);
      img->image_data = NULL;
   }

   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_image_delete);

DD_STATUS_T dd_image_copy_area(DD_IMAGE_T *src_img, DD_IMAGE_T *dst_img,
      DD_RECT_T *src_rect, DD_RECT_T *dst_rect)
{
   return ge_engage(src_img, dst_img, src_rect, dst_rect, DD_GE_OP_COPY_AREA,
         0, 0);
}
EXPORT_SYMBOL(dd_image_copy_area);

DD_STATUS_T dd_image_fill_rect(DD_IMAGE_T *image, DD_RECT_T *rect,
      uint32_t color)
{
    return ge_engage(image, image, rect, rect, DD_GE_OP_FILL_COLOR, color, 0);
}
EXPORT_SYMBOL(dd_image_fill_rect);

DD_STATUS_T dd_image_alpha_blend(DD_IMAGE_T *src_img,
      DD_IMAGE_T *dst_img, DD_RECT_T *src_rect, DD_RECT_T *dst_rect,
      DD_FLAGS_T flag, uint32_t alpha)
{
   return ge_engage(src_img, dst_img, src_rect, dst_rect,
          DD_GE_OP_ALPHA_BLEND, alpha, flag);
}
EXPORT_SYMBOL(dd_image_alpha_blend);

DD_STATUS_T dd_lcd_get(DD_LCD_INFO_T *info)
{
   DD_CFG_T *dd = &g_dd;

   memcpy(info, &dd->lcd_info, sizeof(DD_LCD_INFO_T));

   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_lcd_get);

DD_STATUS_T dd_rect_set(DD_RECT_T *rect, int32_t x, int32_t y,
                  int32_t width, int32_t height)
{
   rect->x = x;
   rect->y = y;
   rect->width = width;
   rect->height = height;
   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_rect_set);

DD_STATUS_T dd_init(DD_LCD_MODEL_T panel_model)
{
   DD_CFG_T *dd = &g_dd;

   atomic_set(&video_is_on, 0);

   atomic_set(&dd->is_initialized, 1);
   init_MUTEX(&dd->lock); /* unlocked */

   if (dd_dma_init() != DD_SUCCESS) {
      printk(KERN_ERR "DD: DMA initialization failed\n");
      return DD_FAIL;
   }

   if (dd_lcd_init(panel_model) != DD_SUCCESS) {
      printk(KERN_ERR "DD: LCD initialization failed\n");
      goto dd_err_dma;
   }
   dd_lcd_panel_reset();
   dd_lcd_panel_enable();

   dd_lcd_info_get(&dd->lcd_info);
   if (dd->lcd_info.size > DD_MAX_BUFFER_SIZE) {
      printk(KERN_ERR "DD: Bitmap size = %u bytes > max allowed size = %u\n",
            dd->lcd_info.size, DD_MAX_BUFFER_SIZE);
      goto dd_err_lcd;
   }

   if (update_init(&dd->update) != DD_SUCCESS) {
      printk(KERN_ERR "DD: Update initialization failed\n");
      goto dd_err_lcd;
   }

   if (element_init(&dd->element) != DD_SUCCESS) {
      printk(KERN_ERR "DD: Element initialization failed\n");
      goto dd_err_update;
   }

   if (dd_image_create(&dd->active_img, dd->lcd_info.width,
            dd->lcd_info.height, DD_FORMAT_URGB888) != DD_SUCCESS) {
		printk(KERN_ERR "DD: Unable to create the Active image\n");
		goto dd_err_element;
	}

   if (dd_image_create(&dd->inactive_img, dd->lcd_info.width,
            dd->lcd_info.height, DD_FORMAT_URGB888) != DD_SUCCESS) {
		printk(KERN_ERR "DD: Unable to create the Inactive image\n");
		goto dd_err_active;
	}
   
   return DD_SUCCESS;

//dd_err_inactive:
   dd_image_delete(&dd->inactive_img);
dd_err_active:
   dd_image_delete(&dd->active_img);
dd_err_element:
   element_term(&dd->element);
dd_err_update:
   update_term(&dd->update);
dd_err_lcd:
   dd_lcd_term();
dd_err_dma:
   dd_dma_term();
   return DD_FAIL;
}
EXPORT_SYMBOL(dd_init);

DD_STATUS_T dd_term(void)
{
   DD_CFG_T *dd = &g_dd;

   dd_dma_term();
   dd_lcd_term();
   element_term(&dd->element);
   update_term(&dd->update);

   dd_image_delete(&dd->inactive_img);
   dd_image_delete(&dd->active_img);

   atomic_set(&dd->is_initialized, 0);

   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_term);

int dd_is_initialized(void)
{
   DD_CFG_T *dd = &g_dd;

   return atomic_read(&dd->is_initialized);
}

DD_ELEMENT_HANDLE_T dd_element_add(DD_UPDATE_HANDLE_T update_handle,
      const char *name, int32_t layer, const DD_RECT_T *dest_rect,
      const DD_IMAGE_T *src_img, const DD_RECT_T *src_rect, DD_FLAGS_T flags,
      uint8_t opacity, DD_TRANSFORM_T transform)
{
   DD_CFG_T *dd = &g_dd;
   UPDATE_T *update;
   DD_ELEMENT_T *element;

   /* find the update using its handle */
   update = update_find(&dd->update, update_handle);
   if (update == NULL) {
      printk(KERN_ERR "DD: Update handle is invalid\n");
      return DD_INVALID_HANDLE;
   }
      
   /* TODO: add code validate element parameters */

   /* find a free element to use */
   element = element_reserve(&dd->element);
   if (element == NULL) {
      printk(KERN_ERR "DD: No free element is available\n");
      return DD_INVALID_HANDLE;
   }

   /* copy information into the element */
   if (name != NULL)
      snprintf(element->name, sizeof(element->name), "%s", name);

   /* video playback mode */
   if (strncmp(element->name, "vdec", 4) == 0) {
      atomic_set(&video_is_on, 1);
   }

   element->flags = flags;
   element->opacity = opacity;
   element->layer = layer;
   memcpy(&element->src_img, src_img, sizeof(DD_IMAGE_T));
   
   element->dest_rect.x = dest_rect->x;
   element->dest_rect.y = dest_rect->y;
   element->dest_rect.width = dest_rect->width;
   element->dest_rect.height = dest_rect->height;

   element->src_rect.x = src_rect->x;
   element->src_rect.y = src_rect->y;
   element->src_rect.width = src_rect->width;
   element->src_rect.height = src_rect->height;

   /* add the element to the update list */
   update_add_element(update, element);

   return element->handle;
}
EXPORT_SYMBOL(dd_element_add);

DD_STATUS_T dd_element_remove(DD_UPDATE_HANDLE_T update_handle,
      DD_ELEMENT_HANDLE_T element_handle)
{
   DD_CFG_T *dd = &g_dd;
   UPDATE_T *update;

   update = update_find(&dd->update, update_handle);
   if (update == NULL) {
      printk(KERN_ERR "DD: Update handle is invalid\n");
      return DD_FAIL;
   }

   return update_remove_element(update, element_handle);
}
EXPORT_SYMBOL(dd_element_remove);

DD_UPDATE_HANDLE_T dd_update_start(int32_t priority)
{
   DD_CFG_T *dd = &g_dd;
   DD_UPDATE_HANDLE_T update_handle;
      
   DD_LOG("tstart [DD dd_update_start]\n");
   
   update_handle = update_reserve(&dd->update, priority);

   DD_LOG("tstop [DD dd_update_start]\n");

   return update_handle;
}
EXPORT_SYMBOL(dd_update_start);

DD_STATUS_T dd_update_submit_sync(DD_UPDATE_HANDLE_T update_handle)
{
   DD_CFG_T *dd = &g_dd;
   UPDATE_T *update;
   DD_ELEMENT_T *curr;

   update = update_find(&dd->update, update_handle);
   if (update == NULL) {
      printk(KERN_ERR "DD: Update handle is invalid\n");
      return DD_FAIL;
   }

   down(&dd->lock);

   DD_LOG("tstart [DD dd_update_submit_sync]\n");

   /* add new elements in the update list to the display list */
   element_add_to_display(update, &dd->element);
   /* remove elements in the update removal list from the display list */
   element_remove_from_display(update, &dd->element);

   /* skip the GUI update in the video playback mode */
   if (update->priority == 10 && atomic_read(&video_is_on)) {
      goto update_finish;
   }

   /* start with the element on the bottom */
   curr = dd->element.display;
   /*
    * If the rectangular region covers the entire LCD (which indicates the
    * bitmap to be copied is contiguous), use DMA. Otherwise, use GE copy area
    */
   if (curr->dest_rect.width == dd->lcd_info.width &&
         curr->dest_rect.height == dd->lcd_info.height) {
      dd_dma_transfer(curr->src_img.addr, dd->inactive_img.addr,
            dd->lcd_info.size);
   } else {
      ge_engage(&curr->src_img, &dd->inactive_img, &curr->src_rect,
            &curr->dest_rect, DD_GE_OP_COPY_AREA, 0, 0);
   }

   /* go through the display list */
   curr = curr->display_next;
   while (curr != NULL) {

#if DD_VIDEO_DEMO
      /*
       * If it's the video frame and move_video flag is turned on, move
       * videos on the fly
       */
      if (strncmp(curr->name, "vdec", 4) == 0) {
         if (atomic_read(&move_video)) {
            bounce_next(&move_pattern);
            curr->dest_rect.x = move_pattern.curx;
            curr->dest_rect.y = move_pattern.cury;
         }
      }
#endif
      
      /* if alpha blending is required */
      if (curr->flags == DD_FLAGS_ALPHA_SOURCE ||
            curr->flags == DD_FLAGS_ALPHA_FIXED) {
         ge_engage(&curr->src_img, &dd->inactive_img, &curr->src_rect,
               &curr->dest_rect, DD_GE_OP_ALPHA_BLEND, curr->opacity,
               curr->flags);
      } else { /* no alpha blending, just copy it over */
         if (curr->dest_rect.width == dd->lcd_info.width &&
            curr->dest_rect.height == dd->lcd_info.height) {
            dd_dma_transfer(curr->src_img.addr, dd->inactive_img.addr,
                  dd->lcd_info.size);
         } else {
            ge_engage(&curr->src_img, &dd->inactive_img, &curr->src_rect,
                  &curr->dest_rect, DD_GE_OP_COPY_AREA, 0, 0);
         }
      }

      /* advance to the next element */
      curr = curr->display_next;
   }
      
   /* swap active/inactive images and display it on the LCD */
   display_update(dd);

update_finish:
   /* release the update and return it back to the pool */
   update_release(update);

   DD_LOG("tstop [DD dd_update_submit_sync]\n");

   up(&dd->lock);

   return DD_SUCCESS;
}
EXPORT_SYMBOL(dd_update_submit_sync);
