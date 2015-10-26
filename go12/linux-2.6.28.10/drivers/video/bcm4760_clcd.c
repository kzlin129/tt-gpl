/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  linux/drivers/video/bcm4760_clcd.c
 *
 * Copyright (C) 2001 ARM Limited, by David A Rusling
 * Updated to 2.5, Deep Blue Solutions Ltd.
 *
 *  Modified ARM PrimeCell PL110 Color LCD Controller
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#include <asm/sizes.h>
#include <asm/uaccess.h>

#define ASM_STOPHERE    __asm("1: nop; b 1b")

#define CLCD_DBG 1

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/knllog.h>


#define BCM4760_HACK					// Hack to make LCD driver build/work on BCM4760

#ifdef BCM4760_HACK
#undef CLCD_GE_RASTER_WORKAROUND
//#define CONFIG_ARCH_SMEAGOL
#endif

#ifndef BCM4760_HACK
//#include <csp/include/intcHw.h>
//#include <asm/arch/csp/chipcHw_inline.h>
//#include <asm/arch/csp/geHw_reg.h>
//#include <csp/geHw.h>
//#include <csp/gev3Hw.h>
#endif

#define MAX_PROC_BUF_SIZE (256)
/* TODO: change to new values after measurement */
#define TIMEOUT_VSYNC     (10 * HZ)
#define TIMEOUT_GE        (HZ)

#define CMAP_SIZE 256

#ifdef CONFIG_ARCH_SMEAGOL
#define TIME_SCALE 60 /* FPGA is 30 times slower than a real system */
#else
#define TIME_SCALE 1
#endif

#define to_clcd(info)     container_of(info, struct clcd_fb, fb)

#define CLCD_TIMER_START(timer_ptr, tval_ptr) \
   if (clcdfb_timer_is_enabled(timer_ptr)) { \
   clcdfb_timer_start(tval_ptr);}
#define CLCD_TIMER_END(timer_ptr, tval_ptr) \
   if (clcdfb_timer_is_enabled(timer_ptr)) { \
   clcdfb_timer_end(tval_ptr);}

struct clcd_proc_dirs {
   struct proc_dir_entry *parent_dir;
#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   struct proc_dir_entry *video_test_dir;
   struct proc_dir_entry *gui_test_dir;
#endif
};

/*
 * Private Function Prototypes
 */
static int clcdfb_update(struct fb_info *info,
      LCD_DirtyRows_t *dirty_rows_ptr);
static void clcdfb_copyarea(struct fb_info *info,
      const struct fb_copyarea *area);
static int clcdfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);

/* This is limited to 16 characters when displayed by X startup */
static const char *clcd_name = "CLCD_FB";

/* Panel index to select the LCD panel type */
#if defined(CONFIG_FB_ARMCLCD_ALL_PANEL) || defined(CONFIG_FB_ARMCLCD_SEIKO_RA169Z)
static unsigned int gpanel = SEIKO_RA169Z;
#elif defined(FB_ARMCLCD_SEIKO_RA167Z)
static unsigned int gpanel = SEIKO_RA167Z;
#elif defined(CONFIG_FB_BCM476X_CLCD)
static unsigned int gpanel = 3;
#else
static unsigned int gpanel = SEIKO_RA169Z;
#endif
module_param(gpanel, uint, 0644);

/*
 * Address of the CLCD frame buffer data structure, only used for public
 * functions
 */
static struct clcd_fb *gfb = NULL;

/* flag to indicate if the CLCD driver has been initialized or not */
static int clcd_is_initialized = 0;

/* procfs */
static struct clcd_proc_dirs clcd_proc;

/*
 * Private Function Prototypes
 */
static inline void clcdfb_dma_memcpy_complete(struct clcd_dma *dma);

static void clcdfb_default_draw(struct clcd_buf *buf)
{
   u32 i, len;
   u32 *tmp_buf;

   if (buf == NULL)
      return;

   if (buf->virt_ptr == NULL)
      return;

   tmp_buf = (u32 *)buf->virt_ptr;
   len = buf->len / 4;

   /*
    * TODO: Assuming 32 bpp. Fill up 4 sections of the FB with different
    * colors by default
    */

   /* section 1 */
   for (i = 0; i < len / 4; i++)
      tmp_buf[i] = 0xFFFF0000; /* RED */

   /* section 2 */
   for (i = len / 4; i < len / 2; i++)
      tmp_buf[i] = 0xF000FF00; /* GREEN */

   /* section 3 */
   for (i = len / 2; i < len * 3 / 4; i++)
      tmp_buf[i] = 0x700000FF; /* BLUE */

   /* section 4 */
   for (i = len * 3 / 4; i < len; i++)
      tmp_buf[i] = 0xFFFFFFFF; /* WHITE */
}

/*
 * Compare the src and dst of len size
 */
static inline int clcdfb_fb_validation(void *src, void *dst, u32 len, u32 bpp, u32 skip_count)
{
   u32 i;
   u32 *src_32;
   u32 *dst_32;
   u32 incr = skip_count + 1;

   if (bpp == 16) {
      len = len / 4 / 2; /* convert 8 bits to 32 bits, and cut len by half */
   } else if (bpp == 32) {
      len = len / 4; /* convert 8 bits to 32 bits */
   } else { /* bpp other than 16 or 32 not supported */
      return -1;
   }

   src_32 = (u32 *)src;
   dst_32 = (u32 *)dst;

   for (i = 0; i < len; i+=incr) {
      if (src_32[i] != dst_32[i]) {
         printk(KERN_ERR "CLCD: FB src[%u]=0x%08x != dst[%u]=0x%08x\n",
               i, src_32[i], i, dst_32[i]);
         return -1;
      }
   }
   return 0;
}

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_vthread_init(struct clcd_vthread *vthread)
{
   u32 width, height;

   BUG_ON(vthread == NULL);

   /* check if the CLCD driver has been initialized */
   if (!clcdfb_is_initialized()) {
      printk(KERN_ERR "CLCD: Driver is NOT initialized\n");
      return -EFAULT;
   }

	vthread->sig_id = 0;
	vthread->proc_id = 0;

	init_completion(&vthread->sig_exit); /* incomplete */
   init_completion(&vthread->proc_exit); /* incomplete */
   init_completion(&vthread->wait_to_start); /* incomplete */
	init_completion(&vthread->wake_up); /* incomplete */

   /* disable vthread by default */
   atomic_set(&vthread->enable, 0);
   /* set default frame rate */
   atomic_set(&vthread->rate, CLCD_VTHREAD_RATE);
	atomic_set(&vthread->in_process, 0);
   /* turn on frame overrun warning messages by default */
   atomic_set(&vthread->warning, 1);

   /* set default single frame processing threshold */
   atomic_set(&vthread->thresh, CLCD_VIDEO_THRESH);

   /* allocate the max possible buffer */
   vthread->buf.len = 800 * 480 * 4;

   /* allocate the Video FB */
   vthread->buf.virt_ptr = dma_alloc_writecombine(NULL, vthread->buf.len,
			&vthread->buf.phys_addr, GFP_KERNEL);
   if (!vthread->buf.virt_ptr) {
      printk(KERN_ERR "CLCD: Unable to allocate the Video FB\n");
      return -ENOMEM;
   }
   vthread->buf.phys_addr = virt_to_phys(vthread->buf.virt_ptr);

   memset(vthread->buf.virt_ptr, 0, vthread->buf.len);

#ifndef BCM4760_HACK
   /* Video FB size */
   switch (gpanel) {
      case SEIKO_RA167Z:
         width = 240;
         height = 136;
         break;

      case SEIKO_RA169Z:
         width = 352;
         height = 288;
         break;

      default:
         width = 100;
         height = 100;
   }
#else
     width = 320;
     height = 240;
#endif

   /* pass the Video FB info to the CLCD driver */
   if (clcdfb_set_video_fb(vthread->buf.phys_addr,
                           CLCD_VTHREAD_XOFFSET,
                           CLCD_VTHREAD_YOFFSET,
                           width,
                           height,
                           32) != 0) {
      printk(KERN_ERR "CLCD: clcdfb_set_video_fb failed\n");
      return -EFAULT;
   }
   printk(KERN_INFO "CLCD: Default video FB width=%u height=%u\n",
         width, height);

   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_vthread_proc(void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   struct clcd_fb *fb = container_of(vthread, struct clcd_fb, vthread);
   unsigned msec;
   timer_tick_count_t tmr_val;
   struct sched_param param = { .sched_priority = 90 };
   int rval;

   sched_setscheduler(current, SCHED_FIFO, &param);

   daemonize("CLCD_VThread_Process");

   /* tell the signaling thread to start */
   complete(&vthread->wait_to_start);

   while (1) {
      if (!atomic_read(&vthread->enable))
         break;

      rval = wait_for_completion_interruptible(&vthread->wake_up);
      if (rval < 0)
         break;

      atomic_set(&vthread->in_process, 1);

      tmr_val = timer_get_tick_count();

      clcdfb_video_decode_start();
      clcdfb_video_decode_end();

      tmr_val = timer_get_tick_count() - tmr_val;
      msec = tmr_val / (timer_get_tick_rate() / 1000);
      msec = msec / TIME_SCALE;
      if (msec > atomic_read(&vthread->thresh)) {
         fb->stats.video_long++;
         if (atomic_read(&vthread->warning)) {
            printk(KERN_ERR "CLCD: VThread - frame processing is taking too "
                  "long: %u ms\n", msec);
         }
      }

      atomic_set(&vthread->in_process, 0);
   }

   atomic_set(&vthread->in_process, 0);
   complete_and_exit(&vthread->proc_exit, 0);
   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_vthread_signal(void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   struct clcd_fb *fb = container_of(vthread, struct clcd_fb, vthread);
   struct sched_param param = { .sched_priority = 99 };
   int rval;

   sched_setscheduler(current, SCHED_FIFO, &param);

   daemonize("CLCD_VThread_Signal");

   /* wait for the frame processing thread to start before continuing */
   rval = wait_for_completion_interruptible(&vthread->wait_to_start);

   while (1) {
      if (!atomic_read(&vthread->enable))
         break;

      /* frame processing is still in process, this is an overrun */
      if (atomic_read(&vthread->in_process)) {
         fb->stats.video_overrun++;
         if (atomic_read(&vthread->warning)) {
            printk(KERN_ERR "CLCD: VThread - frame overrun occurred\n");
         }

         /*
          * Since this is in test mode, we wait for frame processing to
          * finish before continuing
          */
         while (atomic_read(&vthread->in_process)) {
            msleep(1);
         }
      }

      /* signal the frame processing thread to start processing the frame */
      complete(&vthread->wake_up);

      /* convert fps to ms */
      msleep(1000 / atomic_read(&vthread->rate));
   }

   complete(&vthread->wake_up);
   complete_and_exit(&vthread->sig_exit, 0);
   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_vthread_start(struct clcd_vthread *vthread)
{
   BUG_ON(vthread == NULL);

   /* already enabled */
   if (atomic_read(&vthread->enable)) {
      return;
   }

   /* mark proc_exit as incomplete */
   INIT_COMPLETION(vthread->proc_exit);
	/* mark sig_exit as incomplete */
   INIT_COMPLETION(vthread->sig_exit);
   /* mark wait_to_start as incomplete */
   INIT_COMPLETION(vthread->wait_to_start);
	/* mark wake_up as incomplete */
   INIT_COMPLETION(vthread->wake_up);

   atomic_set(&vthread->enable, 1);

   vthread->proc_id = kernel_thread(clcdfb_vthread_proc,
			(void *)vthread, 0);
	vthread->sig_id = kernel_thread(clcdfb_vthread_signal,
			(void *)vthread, 0);
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_vthread_stop(struct clcd_vthread *vthread)
{
   BUG_ON(vthread == NULL);

   atomic_set(&vthread->enable, 0);

   if (vthread->proc_id > 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
      kill_pid(find_vpid(vthread->proc_id), SIGTERM, 1);
#else
      kill_proc(vthread->proc_id, SIGTERM, 1);
#endif
      wait_for_completion(&vthread->proc_exit);
   }

   if (vthread->sig_id > 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
      kill_pid(find_vpid(vthread->sig_id), SIGTERM, 1);
#else
      kill_proc(vthread->sig_id, SIGTERM, 1);
#endif
      wait_for_completion(&vthread->sig_exit);
   }

   vthread->sig_id = 0;
   vthread->proc_id = 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_vthread_term(struct clcd_vthread *vthread)
{
   BUG_ON(vthread == NULL);

   clcdfb_vthread_stop(vthread);

   /* release Video FB memory */
   if (vthread->buf.virt_ptr != NULL) {
      dma_free_writecombine(NULL, vthread->buf.len,
			vthread->buf.virt_ptr, vthread->buf.phys_addr);
   }

   vthread->buf.virt_ptr = NULL;
   vthread->buf.phys_addr = 0;
   vthread->buf.len = 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void
clcdfb_vthread_modify_rate(struct clcd_vthread *vthread, unsigned int rate)
{
   BUG_ON(vthread == NULL);

   atomic_set(&vthread->rate, rate);
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_gthread_init(struct clcd_gthread *gthread)
{
   BUG_ON(gthread == NULL);

   /* check if the CLCD driver has been initialized */
   if (!clcdfb_is_initialized()) {
      printk(KERN_ERR "CLCD: Driver is NOT initialized\n");
      return -EFAULT;
   }

   gthread->sig_id = 0;
	gthread->proc_id = 0;

	init_completion(&gthread->sig_exit); /* incomplete */
   init_completion(&gthread->proc_exit); /* incomplete */
   init_completion(&gthread->wait_to_start); /* incomplete */
	init_completion(&gthread->wake_up); /* incomplete */

   /* disable gthread by default */
   atomic_set(&gthread->enable, 0);
   /* set default frame rate */
   atomic_set(&gthread->rate, CLCD_GTHREAD_RATE);
	atomic_set(&gthread->in_process, 0);
   /* turn on frame overrun warning messages by default */
   atomic_set(&gthread->warning, 1);

   /* set default single frame processing threshold */
   atomic_set(&gthread->thresh, CLCD_GUI_THRESH);

   /*
    * By default use CIF size for copyarea and purposely make 1) src and dst
    * areas overlap; 2) bottom-right corner of the src falls in the dst area
    * to simulate the worst case
    */
   atomic_set(&gthread->enable_copy, 0);
   atomic_set(&gthread->copy_rate, 1);
   gthread->area.sx = 0;
   gthread->area.sy = 0;
   gthread->area.width = 352;
   gthread->area.height = 288;
   gthread->area.dx = gthread->area.width / 2;
   gthread->area.dy = gthread->area.height / 2;

   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_gthread_proc(void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   struct clcd_fb *fb = container_of(gthread, struct clcd_fb, gthread);
   unsigned msec;
   timer_tick_count_t tmr_val;
   struct sched_param param = { .sched_priority = 90 };
   LCD_DirtyRows_t dirty_rows;
   int rval;

   dirty_rows.top = 0;
   dirty_rows.bottom = 0;

   sched_setscheduler(current, SCHED_FIFO, &param);

   daemonize("CLCD_GThread_Process");

   /* tell the signaling thread to start */
   complete(&gthread->wait_to_start);

   while (1) {
      if (!atomic_read(&gthread->enable))
         break;

      rval = wait_for_completion_interruptible(&gthread->wake_up);
      if (rval < 0)
         break;

      atomic_set(&gthread->in_process, 1);

      tmr_val = timer_get_tick_count();

      if (atomic_read(&gthread->enable_copy)) {
         unsigned int i;

         for (i = 0; i < atomic_read(&gthread->copy_rate); i++) {
            clcdfb_copyarea(&fb->fb, &gthread->area);
         }
      }
      clcdfb_update(&fb->fb, &dirty_rows);

      tmr_val = timer_get_tick_count() - tmr_val;
      msec = tmr_val / (timer_get_tick_rate() / 1000);
      msec = msec / TIME_SCALE;
      if (msec > atomic_read(&gthread->thresh)) {
         fb->stats.gui_long++;
         if (atomic_read(&gthread->warning)) {
            printk(KERN_ERR "CLCD: GThread - frame processing is taking too "
                  "long: %u ms\n", msec);
         }
      }

      atomic_set(&gthread->in_process, 0);
   }

   atomic_set(&gthread->in_process, 0);
   complete_and_exit(&gthread->proc_exit, 0);
   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int clcdfb_gthread_signal(void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   struct clcd_fb *fb = container_of(gthread, struct clcd_fb, gthread);
   struct sched_param param = { .sched_priority = 99 };
   int rval;

   sched_setscheduler(current, SCHED_FIFO, &param);

   daemonize("CLCD_GThread_Signal");

   /* wait for the frame processing thread to start before continuing */
   rval = wait_for_completion_interruptible(&gthread->wait_to_start);

   while (1) {
      if (!atomic_read(&gthread->enable))
         break;

      /* frame processing is still in process, this is an overrun */
      if (atomic_read(&gthread->in_process)) {
         fb->stats.gui_overrun++;
         if (atomic_read(&gthread->warning)) {
            printk(KERN_ERR "CLCD: GThread - frame overrun occurred\n");
         }
         /*
          * Since this is in test mode, we wait for frame processing to
          * finish before continuing
          */
         while (atomic_read(&gthread->in_process)) {
            msleep(1);
         }
      }

      /* signal the frame processing thread to start processing the frame */
      complete(&gthread->wake_up);

      /* convert fps to ms */
      msleep(1000 / atomic_read(&gthread->rate));
   }

   complete(&gthread->wake_up);
   complete_and_exit(&gthread->sig_exit, 0);
   return 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_gthread_start(struct clcd_gthread *gthread)
{
   BUG_ON(gthread == NULL);

   /* already enabled */
   if (atomic_read(&gthread->enable)) {
      return;
   }

   /* mark proc_exit as incomplete */
   INIT_COMPLETION(gthread->proc_exit);
	/* mark sig_exit as incomplete */
   INIT_COMPLETION(gthread->sig_exit);
   /* mark wait_to_start as incomplete */
   INIT_COMPLETION(gthread->wait_to_start);
	/* mark wake_up as incomplete */
   INIT_COMPLETION(gthread->wake_up);

   atomic_set(&gthread->enable, 1);

   gthread->proc_id = kernel_thread(clcdfb_gthread_proc,
			(void *)gthread, 0);
	gthread->sig_id = kernel_thread(clcdfb_gthread_signal,
			(void *)gthread, 0);
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_gthread_stop(struct clcd_gthread *gthread)
{
   BUG_ON(gthread == NULL);

   atomic_set(&gthread->enable, 0);

   if (gthread->proc_id > 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
      kill_pid(find_vpid(gthread->proc_id), SIGTERM, 1);
#else
      kill_proc(gthread->proc_id, SIGTERM, 1);
#endif
      wait_for_completion(&gthread->proc_exit);
   }

   if (gthread->sig_id > 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
      kill_pid(find_vpid(gthread->sig_id), SIGTERM, 1);
#else
      kill_proc(gthread->sig_id, SIGTERM, 1);
#endif
      wait_for_completion(&gthread->sig_exit);
   }

   gthread->sig_id = 0;
   gthread->proc_id = 0;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void clcdfb_gthread_term(struct clcd_gthread *gthread)
{
   BUG_ON(gthread == NULL);

   clcdfb_gthread_stop(gthread);
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static void
clcdfb_gthread_modify_rate(struct clcd_gthread *gthread, unsigned int rate)
{
   BUG_ON(gthread == NULL);

   atomic_set(&gthread->rate, rate);
}
#endif

/*
 * Reset all stats counters
 */
static inline void clcdfb_stats_reset(struct clcd_stats *stats)
{
   memset(stats, 0, sizeof(struct clcd_stats));
}

/*
 * Initialize all stats counters
 */
static inline void clcdfb_stats_init(struct clcd_stats *stats)
{
   clcdfb_stats_reset(stats);
}

/*
 * Enable the performance timers
 */
static inline void clcdfb_timer_enable(struct clcd_timer *timer)
{
   atomic_set(&timer->enable, 1);
}

/*
 * Disable the performance timers
 */
static inline void clcdfb_timer_disable(struct clcd_timer *timer)
{
   atomic_set(&timer->enable, 0);
}

/*
 * Initialize the performance timers
 */
static inline void clcdfb_timer_init(struct clcd_timer *timer)
{
   /* enable timer by default */
   clcdfb_timer_enable(timer);

   memset(&timer->tot_update, 0, sizeof(timer->tot_update));

#ifdef CONFIG_FB_ARMCLCD_VIDEO
   memset(&timer->tot_video, 0, sizeof(timer->tot_video));
   memset(&timer->g2h, 0, sizeof(timer->g2h));
   memset(&timer->h2i, 0, sizeof(timer->h2i));
#endif
#ifdef CLCD_GE_RASTER_WORKAROUND
   memset(&timer->g2r, 0, sizeof(timer->g2r));
   memset(&timer->r2g, 0, sizeof(timer->r2g));
#endif
   memset(&timer->g2i, 0, sizeof(timer->g2i));
   memset(&timer->ge_alpha, 0, sizeof(timer->ge_alpha));
   memset(&timer->ge_copyarea, 0, sizeof(timer->ge_copyarea));
   memset(&timer->ge_fillrect, 0, sizeof(timer->ge_fillrect));
   memset(&timer->swap, 0, sizeof(timer->swap));
}

/*
 * Check if the performance timer is enabled
 */
static inline int clcdfb_timer_is_enabled(struct clcd_timer *timer)
{
   if (atomic_read(&timer->enable) == 0)
      return 0;
   else
      return 1;
}

/*
 * Start a performance timer
 */
static inline void clcdfb_timer_start(struct clcd_tval *tval)
{
   tval->start = timer_get_tick_count();
}

/*
 * End a performance timer
 */
static inline void clcdfb_timer_end(struct clcd_tval *tval)
{
   tval->delta = timer_get_tick_count() - tval->start;
}

#ifdef BCM4760_HACK
/*
 * IRQ handler called when DMA finishes
 */
static void clcdfb_irq_dma(u32 data, u32 err)
{
   struct clcd_dma *dma = (struct clcd_dma *)data;
   clcdfb_dma_memcpy_complete(dma);
}

/*
 * Initialize the DMA lock and register its IRQ handler
 */
static inline int clcdfb_dma_init(struct clcd_dma *dma)
{
   /* initialize semaphore */
   init_MUTEX(&dma->lock.lock);
   init_completion(&dma->lock.complete);
   return 0;
}

/*
 * Acquire a DMA channel. Perform a DMA data transfer and block wait for it
 * to finish. After DMA finishes release the channel
 */
static inline void
clcdfb_dma_memcpy(struct clcd_dma *dma, dma_addr_t src_addr,
      dma_addr_t dst_addr, u32 len)
{
   int rval;
   int ch;
   u32 tf_size[1], src_list[1], dest_list[1];
   DMAC_CTRL_REG in;
   DMAC_CTRL_REG out;
   DMAC_CFG_REG  cfg;

   rval = down_interruptible(&dma->lock.lock);

   tf_size[0] = len/4; // Divided by 4 because transfer width is 32 bits
   src_list[0] = src_addr;
   dest_list[0] = dst_addr;
   //prepare the DMA register data
   in.burst_sz = BCM476X_DMAC_BURST_SIZE_1; // 4 byte burst
   in.incr = 1;
   in.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   in.addr = src_list;
   in.tfr_size = tf_size;
   in.n_addr = 1;
   in.flags = BCM476X_EN_DMA_LAST_TC_INT;

   out.burst_sz = BCM476X_DMAC_BURST_SIZE_1;
   out.incr = 1;
   out.width = BCM476X_DMAC_TRANSFER_WIDTH_32BIT;
   out.addr = dest_list;
   out.tfr_size = tf_size;
   out.n_addr = 1;

   cfg.src_id = 0; 
   cfg.dst_id = 0;
   cfg.trans_type = BCM476X_DMAC_MEM2MEM_CTL;

   /* request the DMA channel */
   ch = bcm476x_request_dma_channel();
   if (ch < 0) {
      printk(KERN_ERR "CLCD: clcdfb_dma_memcpy: dma_request_channel "
            "failed\n");
   }
   else {
      rval = bcm476x_setup_dma_chain(ch, &in, &out, &cfg);
      if (rval == 0) {
         printk(KERN_ERR "CLCD: clcdfb_dma_memcpy: bcm476x_setup_dma_chain failed\n");
      }
      else {
         bcm476x_register_dma_handler(ch,
                                      (u32) dma,
                                      (void (*)(unsigned long, unsigned long))clcdfb_irq_dma);
         /* Start the dma */
         bcm476x_enable_dma_channel(ch);
         /* wait for DMA to complete */
         rval = wait_for_completion_interruptible(&dma->lock.complete);
         if (rval != 0) {
            printk(KERN_ERR "CLCD: clcdfb_dma_memcpy: Completion interrupted or"
                  " failed\n");
         }
      }
      bcm476x_release_dma_channel(ch);
   }
   up(&dma->lock.lock);
}

#endif

/*
 * To signal the completion of DMA
 */
static inline void clcdfb_dma_memcpy_complete(struct clcd_dma *dma)
{
   complete(&dma->lock.complete);
}

/*
 * Initialize an FB lock
 */
static void clcdfb_fb_lock_init(struct semaphore *lock)
{
   init_MUTEX(lock);
}

/*
 * Acquire an FB lock. An FB lock should be acquired before an
 * operation is performed on that FB
 */
static int clcdfb_fb_lock_acquire(struct semaphore *lock)
{
   return down_interruptible(lock);
}

/*
 *  Check if an FB lock is in the "locked" state
 */
static int clcdfb_fb_is_locked(struct semaphore *lock)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   unsigned long flags;
   unsigned int count;
   spin_lock_irqsave(&lock->lock, flags);
   count = lock->count;
   spin_unlock_irqrestore(&lock->lock, flags);
   return (count <= 0);
#else
   return atomic_read(&lock->count) <= 0;
#endif
}

/*
 * Release an FB lock. An FB lock can be released after an
 * opration is done on that FB
 */
static void clcdfb_fb_lock_release(struct semaphore *lock)
{
   BUG_ON(!clcdfb_fb_is_locked(lock));
   up(lock);
}

/*
 * Initialize the Vsync locking mechanism
 */
static void clcdfb_vsync_lock_init(struct clcd_fb *fb)
{
   init_MUTEX(&fb->vsync_lock.lock); /* unlocked */
   init_completion(&fb->vsync_lock.complete); /* incomplete */
}

/*
 * Check if the Vsync is in the "locked" state
 */
static int clcdfb_vsync_is_locked(struct clcd_fb *fb)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   unsigned long flags;
   unsigned int count;
   spin_lock_irqsave(&fb->vsync_lock.lock.lock, flags);
   count = fb->vsync_lock.lock.count;
   spin_unlock_irqrestore(&fb->vsync_lock.lock.lock, flags);
   return (count <= 0);
#else
   return atomic_read(&fb->vsync_lock.lock.count) <= 0;
#endif
}

/*
 * Acquire the Vsync lock. The Vsync lock should be acquired before an
 * operation is perform on either the Active FB or the Inactive FB
 */
static int clcdfb_vsync_lock_acquire(struct clcd_fb *fb)
{
   return down_interruptible(&fb->vsync_lock.lock);
}

/*
 * Release the Vsync lock. The Vsync lock can released after operations are
 * done on BOTH the Active FB and the Inactive FB and the hardware vsync has
 * completed
 */
static void clcdfb_vsync_lock_release(struct clcd_fb *fb)
{
   BUG_ON(!clcdfb_vsync_is_locked(fb));
   up(&fb->vsync_lock.lock);
}

/*
 * Reset the Vsync lock
 */
static void clcdfb_vsync_lock_reset(struct clcd_fb *fb)
{
   /* mark as incomplete */
   INIT_COMPLETION(fb->vsync_lock.complete);
}

/*
 * Wait on the Vsync lock until it's complete
 */
static int clcdfb_vsync_lock_wait(struct clcd_fb *fb, unsigned long timeout)
{
   int rval;

   rval = wait_for_completion_interruptible_timeout(&fb->vsync_lock.complete,
         timeout);
   if (rval < 0) {
      return -EFAULT;
   }
   return 0;
}

/*
 * Signal for complete of the Vsync
 */
static void clcdfb_vsync_lock_complete(struct clcd_fb *fb)
{
   complete(&fb->vsync_lock.complete);
}

#ifndef BCM4760_HACK
/*
 * Initialize the GE locking mechanism
 */
static void clcdfb_ge_lock_init(struct clcd_fb *fb)
{
   init_MUTEX(&fb->ge_lock.lock); /* unlocked */
   init_completion(&fb->ge_lock.complete); /* incomplete */
}

/*
 * Check if the GE is in the "locked" state
 */
static int clcdfb_ge_is_locked(struct clcd_fb *fb)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   unsigned long flags;
   unsigned int count;
   spin_lock_irqsave(&fb->ge_lock.lock, flags);
   count = fb->ge_lock.lock.count;
   spin_unlock_irqrestore(&fb->ge_lock.lock, flags);
   return (count <= 0);
#else
   return atomic_read(&fb->ge_lock.lock.count) <= 0;
#endif
}

/*
 * Acquire the GE lock. The GE lock should be acquired before engaging the GE
 */
static int clcdfb_ge_lock_acquire(struct clcd_fb *fb)
{
   return down_interruptible(&fb->ge_lock.lock);
}

/*
 * Release the GE lock. The GE lock can released after the GE hardware has
 * completed its operation
 */
static void clcdfb_ge_lock_release(struct clcd_fb *fb)
{
   BUG_ON(!clcdfb_ge_is_locked(fb));
   up(&fb->ge_lock.lock);
}

/*
 * Reset the GE lock
 */
static void clcdfb_ge_lock_reset(struct clcd_fb *fb)
{
   /* mark as incomplete */
   INIT_COMPLETION(fb->ge_lock.complete);
}

/*
 * Wait on the GE lock until it's complete
 */
static int clcdfb_ge_lock_wait(struct clcd_fb *fb, unsigned long timeout)
{
   int rval;

   rval = wait_for_completion_interruptible_timeout(&fb->ge_lock.complete,
         timeout);
   if (rval < 0) {
      return -EFAULT;
   }
   return 0;
}

/*
 * Signal for complete of the GE
 */
static void clcdfb_ge_lock_complete(struct clcd_fb *fb)
{
   complete(&fb->ge_lock.complete);
}

#endif

/*
 * Unfortunately, the enable/disable functions may be called either from
 * process or IRQ context, and we _need_ to delay.  This is _not_ good.
 */
static inline void clcdfb_sleep(unsigned int ms)
{
   if (in_atomic()) {
      mdelay(ms);
   } else {
      msleep(ms);
   }
}

/*
 * Sets the starting addrees of the frame buffer to be displayed on the LCD
 */
static inline void clcdfb_set_start(struct clcd_fb *fb)
{
#ifdef DUAL_BUFFERS
	unsigned long ustart = fb->active_fb.phys_addr;
#else
	unsigned long ustart = fb->fb.fix.smem_start;
#endif
   unsigned long lstart;

   BUG_ON(ustart == 0);

   ustart += fb->fb.var.yoffset * fb->fb.fix.line_length;
   BUG_ON(ustart & 0x03);
   lstart = ustart + fb->fb.var.yres * fb->fb.fix.line_length / 2;

   /* program the new Active FB address */
   writel(ustart, fb->regs + CLCD_UBAS);
   writel(lstart, fb->regs + CLCD_LBAS);
}

static void clcdfb_disable(struct clcd_fb *fb)
{
   u32 val;

   if (fb->board->panel_disable)
      fb->board->panel_disable(fb->clk);

   val = readl(fb->regs + CLCD_CNTL);
   if (val & CNTL_LCDPWR) {
      val &= ~CNTL_LCDPWR;
      writel(val, fb->regs + CLCD_CNTL);

      clcdfb_sleep(20);
   }
   if (val & CNTL_LCDEN) {
      val &= ~CNTL_LCDEN;
      writel(val, fb->regs + CLCD_CNTL);
   }

   /*
    * Disable CLCD clock source.
    */
   clk_disable(fb->clk);
}

static void clcdfb_enable(struct clcd_fb *fb, u32 cntl)
{
   /*
    * Enable the CLCD clock source.
    */
   clk_enable(fb->clk);

   /*
    * Bring up by first enabling..
    */
   cntl |= CNTL_LCDEN;
   writel(cntl, fb->regs + CLCD_CNTL);

   clcdfb_sleep(20);

   /*
    * and now apply power.
    */
   cntl |= CNTL_LCDPWR;
   writel(cntl, fb->regs + CLCD_CNTL);

   /*
    * finally, enable the interface.
    */
   if (fb->board->panel_enable)
      fb->board->panel_enable(fb->clk);
}

/* Swap the Active/Inactive FB and get the new Active FB to be displayed on
 * the LCD
 */
static void clcdfb_swap_fb(struct clcd_fb *fb)
{
   u32 val;
   struct clcd_buf tmp_fb;

   CLCD_TIMER_START(&fb->timer, &fb->timer.swap);

   /* swap Active/Inactive FBs */
   tmp_fb = fb->active_fb;
   fb->active_fb = fb->inactive_fb;
   fb->inactive_fb = tmp_fb;

   clcdfb_vsync_lock_reset(fb);

   /* enable Vsync interrupt */
   val = readl(fb->regs + CLCD_IENB);
   writel(val | INTR_LNBU, fb->regs + CLCD_IENB);

   /* get the LCD to display the new Active FB */
   clcdfb_set_start(fb);

   /* wait for Vsync to finish so the new Acive FB address is loaded */
   clcdfb_vsync_lock_wait(fb, TIMEOUT_VSYNC);

#if 0
   /* TODO: not needed until dirty row tracking is added */
   /* data transfer from the new Active FB to the new Inactive FB */
   memcpy(fb->inactive_fb.virt_ptr, fb->active_fb.virt_ptr,
         fb->active_fb.len);
#endif

   CLCD_TIMER_END(&fb->timer, &fb->timer.swap);
}

static int
clcdfb_set_bitfields(struct clcd_fb *fb, struct fb_var_screeninfo *var)
{
   int ret = 0;

   memset(&var->transp, 0, sizeof(var->transp));

   var->red.msb_right = 0;
   var->green.msb_right = 0;
   var->blue.msb_right = 0;

   switch (var->bits_per_pixel) {
   case 1:
   case 2:
   case 4:
   case 8:
      var->red.length      = var->bits_per_pixel;
      var->red.offset      = 0;
      var->green.length = var->bits_per_pixel;
      var->green.offset = 0;
      var->blue.length  = var->bits_per_pixel;
      var->blue.offset  = 0;
      break;
   case 16:
      var->red.length = 5;
      var->blue.length = 5;
      /*
       * Green length can be 5 or 6 depending whether
       * we're operating in RGB555 or RGB565 mode.
       */
      if (var->green.length != 5 && var->green.length != 6)
         var->green.length = 6;
      break;
   case 32:
      if (fb->panel->cntl & CNTL_LCDTFT) {
         var->red.length      = 8;
         var->green.length = 8;
         var->blue.length  = 8;
         break;
      }
   default:
      ret = -EINVAL;
      break;
   }

   /*
    * >= 16bpp displays have separate colour component bitfields
    * encoded in the pixel data.  Calculate their position from
    * the bitfield length defined above.
    */
   if (ret == 0 && var->bits_per_pixel >= 16) {
      if (fb->panel->cntl & CNTL_BGR) {
         var->blue.offset = 0;
         var->green.offset = var->blue.offset + var->blue.length;
         var->red.offset = var->green.offset + var->green.length;
      } else {
         var->red.offset = 0;
         var->green.offset = var->red.offset + var->red.length;
         var->blue.offset = var->green.offset + var->green.length;
      }
   }

   return ret;
}

static int clcdfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
   struct clcd_fb *fb = to_clcd(info);
   int ret = -EINVAL;

   if (fb->board->check)
      ret = fb->board->check(fb, var);

   if (ret == 0 &&
       var->xres_virtual * var->bits_per_pixel / 8 *
       var->yres_virtual > fb->fb.fix.smem_len)
      ret = -EINVAL;

   if (ret == 0)
      ret = clcdfb_set_bitfields(fb, var);

   return ret;
}

static int clcdfb_set_par(struct fb_info *info)
{
   struct clcd_fb *fb = to_clcd(info);
   struct clcd_regs regs;

   fb->fb.fix.line_length = fb->fb.var.xres_virtual *
             fb->fb.var.bits_per_pixel / 8;

   if (fb->fb.var.bits_per_pixel <= 8)
      fb->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
   else
      fb->fb.fix.visual = FB_VISUAL_TRUECOLOR;

   fb->board->decode(fb, &regs);

   clcdfb_disable(fb);

   writel(regs.tim0, fb->regs + CLCD_TIM0);
   writel(regs.tim1, fb->regs + CLCD_TIM1);
   writel(regs.tim2, fb->regs + CLCD_TIM2);
   writel(regs.tim3, fb->regs + CLCD_TIM3);

   clcdfb_vsync_lock_acquire(fb);
   clcdfb_set_start(fb);
   clcdfb_vsync_lock_release(fb);

   clk_set_rate(fb->clk, (1000000000 / regs.pixclock) * 1000);

   fb->clcd_cntl = regs.cntl;

   clcdfb_enable(fb, regs.cntl);

   CLCD_DEBUG(KERN_INFO "CLCD: Registers set to\n"
         KERN_INFO "  %08x %08x %08x %08x\n"
         KERN_INFO "  %08x %08x %08x %08x\n",
         readl(fb->regs + CLCD_TIM0), readl(fb->regs + CLCD_TIM1),
         readl(fb->regs + CLCD_TIM2), readl(fb->regs + CLCD_TIM3),
         readl(fb->regs + CLCD_UBAS), readl(fb->regs + CLCD_LBAS),
         readl(fb->regs + CLCD_IENB), readl(fb->regs + CLCD_CNTL));

   return 0;
}

static inline unsigned int extract_bitfield(unsigned int color,
      struct fb_bitfield *bf, unsigned int bpp)
{
   switch (bpp) {
      case 16:
         color &= 0xFFFF;
         break;

      case 32:
         break;

      default: /* only support 16 or 32 bpp */
         return 0;
   }

   color = color >> bf->offset;
   color = color << (bpp - bf->length);

   return color;
}

static inline unsigned int convert_bitfield(unsigned int val,
      struct fb_bitfield *bf, unsigned int bpp)
{
   switch (bpp) {
      case 16:
         val &= 0xFFFF;
         break;

      case 32:
         break;

      default: /* only support 16 or 32 bpp */
         return 0;
   }

   val >>= bpp - bf->length;
   return val << bf->offset;
}

/*
 *  Set a single color register. The values supplied have a 16 bit
 *  magnitude.  Return != 0 for invalid regno.
 */
static int
clcdfb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
       unsigned int blue, unsigned int transp, struct fb_info *info)
{
   unsigned int bpp = info->var.bits_per_pixel;
   struct clcd_fb *fb = to_clcd(info);

   if (regno < 16)
      fb->cmap[regno] = convert_bitfield(transp, &fb->fb.var.transp, bpp) |
              convert_bitfield(blue, &fb->fb.var.blue, bpp) |
              convert_bitfield(green, &fb->fb.var.green, bpp) |
              convert_bitfield(red, &fb->fb.var.red, bpp);

   if (fb->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < CMAP_SIZE) {
      int hw_reg = CLCD_PALETTE + ((regno * 2) & ~3);
      u32 val, mask, newval;

      newval  = (red >> 11)  & 0x001f;
      newval |= (green >> 6) & 0x03e0;
      newval |= (blue >> 1)  & 0x7c00;

      /*
       * 3.2.11: if we're configured for big endian
       * byte order, the palette entries are swapped.
       */
      if (fb->clcd_cntl & CNTL_BEBO)
         regno ^= 1;

      if (regno & 1) {
         newval <<= 16;
         mask = 0x0000ffff;
      } else {
         mask = 0xffff0000;
      }

      val = readl(fb->regs + hw_reg) & mask;
      writel(val | newval, fb->regs + hw_reg);
   }

   return regno > 255;
}

/*
 *  Blank the screen if blank_mode != 0, else unblank. If blank == NULL
 *  then the caller blanks by setting the CLUT (Color Look Up Table) to all
 *  black. Return 0 if blanking succeeded, != 0 if un-/blanking failed due
 *  to e.g. a video mode which doesn't support it. Implements VESA suspend
 *  and powerdown modes on hardware that supports disabling hsync/vsync:
 *    blank_mode == 2: suspend vsync
 *    blank_mode == 3: suspend hsync
 *    blank_mode == 4: powerdown
 */
static int clcdfb_blank(int blank_mode, struct fb_info *info)
{
   struct clcd_fb *fb = to_clcd(info);

   if (blank_mode != 0) {
      clcdfb_disable(fb);
   } else {
      clcdfb_enable(fb, fb->clcd_cntl);
   }
   return 0;
}

static int
clcdfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
   struct clcd_fb *fb = to_clcd(info);
   unsigned long len;
   //unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
   int ret = -EINVAL;

   len = info->fix.smem_len;

   //if (off <= len && vma->vm_end - vma->vm_start <= len - off &&
     //  fb->board->mmap) {
      ret = fb->board->mmap(fb, vma);
// }

   return ret;
}

#ifdef BCM4760_HACK
/*
 * copyarea that supports 2-D hardware acceleration
 */
static void
clcdfb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
   /* TODO: use standard Linux copyarea for now */
   cfb_copyarea(info, area);
}
#else
#ifdef CLCD_GE_RASTER_WORKAROUND
/*
 * Check the copyarea src and dst coordinates to see if data overwritten can
 * occur with the Graphic Engine. If yes return 1 if no return 0
 */
static int clcdfb_ge_check_coord(const struct fb_copyarea *area)
{
   /* bottom-right corner of the source area */
   unsigned int br_x, br_y;
   /* destination area */
   unsigned int left, right, top, bottom;

   br_x = area->sx + area->width;
   br_y = area->sy + area->height;

   left = area->dx;
   right = area->dx + area->width;
   top = area->dy;
   bottom = area->dy + area->height;

   /*
    * If the bottom right corner of the source falls into the destination area,
    * return 1. else return 0
    */
   if (br_x >= left && br_x <= right && br_y >= top && br_y <= bottom)
      return 1;
   else
      return 0;
}
#endif

static int
clcdfb_ge_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
   struct clcd_fb *fb = to_clcd(info);
   geHw_ROP_TRANSPARENT_PAR_t rop;
#ifdef CLCD_GE_RASTER_WORKAROUND
   /* flag to indicate whether a tmp raster buffer is needed */
   int raster_buff_flag = 0;
#endif

   CLCD_TIMER_START(&fb->timer, &fb->timer.ge_copyarea);
   KNLLOG("tstart [GE copy area]\n");

#ifdef CLCD_GE_RASTER_WORKAROUND
   raster_buff_flag = clcdfb_ge_check_coord(area);
#endif

   /* must zero the rop params */
   memset(&rop, 0, sizeof(rop));

   /* assume it's either RGB565 or URGB888 */
   if (fb->fb.var.bits_per_pixel == 32) {
      rop.srcformat = GEV3HW_COLOR_URGB888;
      rop.dstformat = GEV3HW_COLOR_URGB888;
   } else { /* RGB565 */
      rop.srcformat = GEV3HW_COLOR_RGB565;
      rop.dstformat = GEV3HW_COLOR_RGB565;
   }

   rop.src_endianess = GEV3HW_LITTLE_ENDIN;
   rop.dst_endianess = GEV3HW_LITTLE_ENDIN;
   rop.rop = GEV3HW_RASTER_S;
   rop.width = area->width;
   rop.height = area->height;
   rop.src1Pitch = fb->fb.var.xres;
   rop.dstPitch = fb->fb.var.xres;
   rop.src1Addr = (void *) fb->gui_fb.phys_addr +
      (area->sy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (area->sx * fb->fb.var.bits_per_pixel / 8);
   rop.dstAddr = (void *) fb->gui_fb.phys_addr +
      (area->dy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (area->dx * fb->fb.var.bits_per_pixel / 8);

#ifdef CLCD_GE_RASTER_WORKAROUND
   /*
    * If a tmp raster buffer is needed, copy the entire GUI FB to the Raster
    * FB. Then copyarea from the src on the GUI FB to the dest on the Raster FB
    *
    * TODO: Possible future optimization is to copy ONLY the dirty rows
    */
   if (raster_buff_flag) {
      if (clcdfb_fb_lock_acquire(&fb->raster_lock) != 0) {
         printk(KERN_WARNING "CLCD: Raster lock acquire interrupted or "
               "failed\n");
         return -EFAULT;
      }

      /* data transfer from the GUI FB to the Raster FB */
      CLCD_TIMER_START(&fb->timer, &fb->timer.g2r);
      KNLLOG("tstart [DMA from GUI to Raster FB]\n");
      clcdfb_dma_memcpy(&fb->dma, fb->gui_fb.phys_addr,
         fb->raster_fb.phys_addr, fb->gui_fb.len);
      KNLLOG("tstop [DMA from GUI to Raster FB]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.g2r);

      if (fb->validation_level & CLCD_VALIDATE_DMA) {
         if (clcdfb_fb_validation(fb->gui_fb.virt_ptr,
                  fb->raster_fb.virt_ptr, fb->gui_fb.len,
                  fb->panel->bpp, fb->skip_count) != 0) {
            printk(KERN_ERR "CLCD: GUI -> Raster FB DMA validation failed\n");
         }
      }

      /* dest needs to be on the Raster FB */
      rop.dstAddr = (void *) fb->raster_fb.phys_addr +
      (area->dy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (area->dx * fb->fb.var.bits_per_pixel / 8);
   }
#endif

   if (clcdfb_ge_lock_acquire(fb) != 0) {
      KNLLOG("tstop [GE copy area]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.ge_copyarea);
      return -EFAULT;
   }

   /* copy data to the GUI Validation FB so it can be validated later */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      clcdfb_dma_memcpy(&fb->dma, fb->gui_fb.phys_addr,
            fb->gui_validation_fb.phys_addr, fb->gui_fb.len);
   }

   /* select GE3 for copy area */
   chipcHw_selectGE3();

   clcdfb_ge_lock_reset(fb);

   /* enable GE interrupt */
   intcHw_irq_enable((void *)INTCHW_INTC0, INTCHW_INTC0_GE);

   /* perform copy area on the GUI FB */
   gev3Hw_PowerOn(1);
   gev3Hw_rasterOperation(&rop, 1); /* use 1 to enable GE irq */

   /* wait for GE to finish */
   clcdfb_ge_lock_wait(fb, TIMEOUT_GE);
   gev3Hw_PowerOn(0);

   /* perform a software copy area on the GUI Validation FB */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      /* TODO: need a dummy Src 2 address */
      rop.src2Addr = (void *) fb->gui_validation_fb.virt_ptr;

      /*
       * Need to use virtual address instead of physical address. Also,
       * put the result on the Validation FB, so it can be compared with
       * the GUI FB where the real HW fillrect was performed
       */
      rop.src1Addr = (void *) fb->gui_fb.virt_ptr +
      (area->sy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (area->sx * fb->fb.var.bits_per_pixel / 8);
      rop.dstAddr = (void *) fb->gui_validation_fb.virt_ptr +
      (area->dy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (area->dx * fb->fb.var.bits_per_pixel / 8);

      /* software-based copy area */
      GE_Rop_Transparent_op(&rop);
   }

   clcdfb_ge_lock_release(fb);

#ifdef CLCD_GE_RASTER_WORKAROUND
   if (raster_buff_flag) {
      /* data transfer from the Raster FB back to the GUI FB */
      CLCD_TIMER_START(&fb->timer, &fb->timer.r2g);
      KNLLOG("tstart [DMA from Raster to GUI FB]\n");
      clcdfb_dma_memcpy(&fb->dma, fb->raster_fb.phys_addr,
         fb->gui_fb.phys_addr, fb->raster_fb.len);
      KNLLOG("tstop [DMA from Raster to GUI FB]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.r2g);
      if (fb->validation_level & CLCD_VALIDATE_DMA) {
         if (clcdfb_fb_validation(fb->raster_fb.virt_ptr,
                  fb->gui_fb.virt_ptr, fb->raster_fb.len,
                  fb->panel->bpp, fb->skip_count) != 0) {
            printk(KERN_ERR "CLCD: Raster -> GUI FB DMA validation failed\n");
         }
      }
      clcdfb_fb_lock_release(&fb->raster_lock);
   }
#endif

   if (fb->validation_level & CLCD_VALIDATE_GE) {
      /* validate the HW results */
      if (clcdfb_fb_validation(fb->gui_fb.virt_ptr,
               fb->gui_validation_fb.virt_ptr, fb->gui_fb.len,
               fb->panel->bpp, fb->skip_count) != 0) {
         printk(KERN_ERR "CLCD: GE copyarea validation failed\n");
      }
   }

   KNLLOG("tstop [GE copy area]\n");
   CLCD_TIMER_END(&fb->timer, &fb->timer.ge_copyarea);

   return 0;
}

static int
clcdfb_ge_fillrect(struct fb_info *info, LCD_FillRectColor_t *rect)
{
   struct clcd_fb *fb = to_clcd(info);
   geHw_COLORFILLING_PAR_t params;

   CLCD_TIMER_START(&fb->timer, &fb->timer.ge_fillrect);
   KNLLOG("tstart [GE fill rect]\n");

   /* must zero the params */
   memset(&params, 0, sizeof(params));

   /* assume it's either RGB565 or URGB888 */
   if (fb->fb.var.bits_per_pixel == 32)
      params.color_format = GEV3HW_COLOR_URGB888;
   else /* RGB565 */
      params.color_format = GEV3HW_COLOR_RGB565;
   params.dst_endianess = GEV3HW_LITTLE_ENDIN;
   params.color_key = rect->rawColor;
   params.width = rect->width;
   params.height = rect->height;
   params.dstPitch = fb->fb.var.xres;
   params.dstAddr = (void *) fb->gui_fb.phys_addr +
      (rect->dy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (rect->dx * fb->fb.var.bits_per_pixel / 8);

   if (clcdfb_ge_lock_acquire(fb) != 0) {
      KNLLOG("tstop [GE fill rect]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.ge_fillrect);
      return -EFAULT;
   }

   /* copy data to the GUI Validation FB so it can be validated later */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      clcdfb_dma_memcpy(&fb->dma, fb->gui_fb.phys_addr,
            fb->gui_validation_fb.phys_addr, fb->gui_fb.len);
   }

   /* select GE3 for fill rect */
   chipcHw_selectGE3();

   clcdfb_ge_lock_reset(fb);

   /* enable GE interrupt */
   intcHw_irq_enable((void *)INTCHW_INTC0, INTCHW_INTC0_GE);

   /* perform fill rect on the GUI FB */
   gev3Hw_PowerOn(1);
   gev3Hw_colorFill(&params, 1); /* use 1 to enable GE irq */
   /* wait for GE to finish */
   clcdfb_ge_lock_wait(fb, TIMEOUT_GE);
   gev3Hw_PowerOn(0);

   /* use a software fill rect to validate the HW results */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      /*
       * Need to use virtual address instead of physical address. Also,
       * put the result on the GUI Validation FB, so it can be compared with
       * the GUI FB where the real HW fillrect was performed
       */
      params.dstAddr = fb->gui_validation_fb.virt_ptr +
      (rect->dy * info->var.yres * fb->fb.var.bits_per_pixel / 8) +
      (rect->dx * fb->fb.var.bits_per_pixel / 8);

      /* software-based fill rect */
      Ge_ColorFilling_op(&params);

      /* validate the HW results */
      if (clcdfb_fb_validation(fb->gui_fb.virt_ptr,
               fb->gui_validation_fb.virt_ptr, fb->gui_fb.len,
               fb->panel->bpp, fb->skip_count) != 0) {
         printk(KERN_ERR "CLCD: GE fillrect validation failed\n");
      }
   }

   clcdfb_ge_lock_release(fb);

   KNLLOG("tstop [GE fill rect]\n");
   CLCD_TIMER_END(&fb->timer, &fb->timer.ge_fillrect);

   return 0;
}

#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Software alpha blending (URGB888 -> URGB888 only)
 */
static inline void clcdfb_sw_alpha_blend(void *src, void *dst, int width,
      int height, int src_pitch, int dst_pitch, u32 alpha, u32 skip_count)
{
   unsigned int i, j, src_row, dst_row;
   u32 incr = skip_count + 1;
   u32 area = width*height;

   u32 r_src, r_dst, r_result;
   u32 g_src, g_dst, g_result;
   u32 b_src, b_dst, b_result;

   u32 *src_32 = (u32 *) src;
   u32 *dst_32 = (u32 *) dst;
   u32 alpha_com = 256 - alpha;

   if (!skip_count)
   {
      // when skip count is 0, revert to use full algorithm that supports cases
      // where width != pitch
      src_row = 0;
      dst_row = 0;
      for (j = 0; j < height; j++) {
         for (i = 0; i < width; i++) {
            r_src = (src_32[i + src_row] & 0x00FF0000) >> 16;
            g_src = (src_32[i + src_row] & 0x0000FF00) >> 8;
            b_src = (src_32[i + src_row] & 0x000000FF);
            r_dst = (dst_32[i + dst_row] & 0x00FF0000) >> 16;
            g_dst = (dst_32[i + dst_row] & 0x0000FF00) >> 8;
            b_dst = (dst_32[i + dst_row] & 0x000000FF);

            r_result = ((alpha * r_src) + (alpha_com * r_dst)) / 256;
            g_result = ((alpha * g_src) + (alpha_com * g_dst)) / 256;
            b_result = ((alpha * b_src) + (alpha_com * b_dst)) / 256;

            dst_32[i + dst_row] = (dst_32[i + dst_row] & 0xFF000000) |
               ((r_result & 0x000000FF) << 16) |
               ((g_result & 0x000000FF) << 8) |
               (b_result & 0x000000FF);
         }
         src_row += src_pitch;
         dst_row += dst_pitch;
      }
   }
   else
   {
      for (i = 0; i < area; i+=incr) {
         r_src = (src_32[i] & 0x00FF0000) >> 16;
         g_src = (src_32[i] & 0x0000FF00) >> 8;
         b_src = (src_32[i] & 0x000000FF);
         r_dst = (dst_32[i] & 0x00FF0000) >> 16;
         g_dst = (dst_32[i] & 0x0000FF00) >> 8;
         b_dst = (dst_32[i] & 0x000000FF);

         r_result = ((alpha * r_src) + (alpha_com * r_dst)) / 256;
         g_result = ((alpha * g_src) + (alpha_com * g_dst)) / 256;
         b_result = ((alpha * b_src) + (alpha_com * b_dst)) / 256;

         dst_32[i] = (dst_32[i] & 0xFF000000) |
            ((r_result & 0x000000FF) << 16) |
            ((g_result & 0x000000FF) << 8) |
            (b_result & 0x000000FF);
      }
   }
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
#ifndef BCM4760_HACK
/*
 * Engage the GE to perform alpha blending on the Video FB and the Inactive FB
 */
static int clcdfb_ge_alpha_blend(struct clcd_fb *fb)
{
   geHw_ALPHABLEND_PAR_t alpha_info;

   CLCD_TIMER_START(&fb->timer, &fb->timer.ge_alpha);
   KNLLOG("tstart [GE alpha blending]\n");

   /* set up alpha blending parameters */
   alpha_info.srcFormat = GEHW_COLOR_URGB888;
   alpha_info.dstFormat = GEHW_COLOR_URGB888;
   alpha_info.srcAddr = fb->vfb.phys_addr;
   alpha_info.dstAddr = fb->inactive_fb.phys_addr + fb->vfb.offset;
   alpha_info.width = fb->vfb.width;
   alpha_info.height = fb->vfb.height;
   alpha_info.srcPitch = fb->vfb.pitch;
   alpha_info.dstPitch = fb->fb.var.xres;
   /*
    * TODO: Cannot support in-pixel alpha until a new GE driver comes out.
    * Use the alpha value from the first pixel for now
    */
   alpha_info.alpha = 255 - ((*(u32 *)fb->inactive_fb.virt_ptr) >> 24);
   alpha_info.option = 0;
   alpha_info.colorSpace = GEHW_COLOR_SPACE_RGB;

   if (clcdfb_ge_lock_acquire(fb) != 0) {
      KNLLOG("tstop [GE alpha blending]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.ge_alpha);
      return -EFAULT;
   }

   /* copy data to the Video Validation FB so it can be validated later */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      clcdfb_dma_memcpy(&fb->dma, fb->inactive_fb.phys_addr,
            fb->video_validation_fb.phys_addr, fb->inactive_fb.len);
   }

   /* select GE2 for alpha blending */
   chipcHw_selectGE2();

   clcdfb_ge_lock_reset(fb);

   /* enable GE interrupt */
   intcHw_irq_enable((void *)INTCHW_INTC0, INTCHW_INTC0_GE);

   /* perform alpha blending on the Video FB and the Inactive FB */
   geHw_alphablend(&alpha_info, 1); /* use 1 to enable GE irq */

   /* wait for GE to finish */
   clcdfb_ge_lock_wait(fb, TIMEOUT_GE);

   /* use a software alpha blend to validate the HW results */
   if (fb->validation_level & CLCD_VALIDATE_GE) {
      void *video_ptr = NULL;

      video_ptr = dma_to_virt(NULL, fb->vfb.phys_addr);

      /* software-based alpha blending */
      clcdfb_sw_alpha_blend(video_ptr,
            fb->video_validation_fb.virt_ptr + fb->vfb.offset,
            fb->vfb.width,
            fb->vfb.height,
            fb->vfb.pitch,
            fb->fb.var.xres,
            alpha_info.alpha,
            fb->skip_count);

      /* validate the HW results */
      if (clcdfb_fb_validation(fb->inactive_fb.virt_ptr,
               fb->video_validation_fb.virt_ptr, fb->inactive_fb.len,
               fb->panel->bpp, fb->skip_count) != 0) {
         printk(KERN_ERR "CLCD: GE alpha blending validation failed\n");
      }
   }

   clcdfb_ge_lock_release(fb);

   KNLLOG("tstop [GE alpha blending]\n");
   CLCD_TIMER_END(&fb->timer, &fb->timer.ge_alpha);

   return 0;
}
#endif
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * This routine performs required actions in Video On mode:
 * 1. Data transfer from the Holding FB to the Inactive FB
 * 2. Engage the GE engine for alpha blending on the Inactive FB and the Video
 *    FB. The result is stored in the Inactive FB
 * 3. Swap the Active/Inactive FBs and display the new Active FB on the LCD
 * 4. Data transfer from the new Active FB to the new Inactive FB (with dirty
 *    row tracking)
 */
static int clcdfb_video_process(struct clcd_fb *fb)
{
   if (!atomic_read(&fb->video_on)) { /* video is turned off on LCD */
      return 0;
   }

   KNLLOG("tstart [clcd total video time]\n");
   CLCD_TIMER_START(&fb->timer, &fb->timer.tot_video);

   if (clcdfb_fb_lock_acquire(&fb->holding_lock) != 0) {
   	KNLLOG("tstop [clcd total video time]\n");
      printk(KERN_WARNING "CLCD: Holding lock acquire interrupted or "
            "failed\n");
      return -EFAULT;
   }
   if (clcdfb_vsync_lock_acquire(fb) != 0) {
   	KNLLOG("tstop [clcd total video time]\n");
      printk(KERN_WARNING "CLCD: Vsync lock acquire interrupted or failed\n");
      return -EFAULT;
   }

   /* data transfer from the Holding FB to the Inactive FB */
   CLCD_TIMER_START(&fb->timer, &fb->timer.h2i);
   KNLLOG("tstart [DMA from Holding to Inactive FB]\n");
   clcdfb_dma_memcpy(&fb->dma, fb->holding_fb.phys_addr,
         fb->inactive_fb.phys_addr, fb->holding_fb.len);
   KNLLOG("tstop [DMA from Holding to Inactive FB]\n");
   CLCD_TIMER_END(&fb->timer, &fb->timer.h2i);

   if (fb->validation_level & CLCD_VALIDATE_DMA) {
      if (clcdfb_fb_validation(fb->holding_fb.virt_ptr,
               fb->inactive_fb.virt_ptr, fb->holding_fb.len,
               fb->panel->bpp, fb->skip_count) != 0) {
         printk(KERN_ERR "CLCD: Holding -> Inactive FB DMA validation "
               "failed\n");
      }
   }

   clcdfb_fb_lock_release(&fb->holding_lock);
   clcdfb_vsync_lock_release(fb);

   if (clcdfb_vsync_lock_acquire(fb) != 0) {
   	KNLLOG("tstop [clcd total video time]\n");
      printk(KERN_WARNING "CLCD: Vsync lock acquire interrupted or failed\n");
		CLCD_TIMER_END(&fb->timer, &fb->timer.tot_video);
      return -EFAULT;
   }
   if (clcdfb_fb_lock_acquire(&fb->video_lock) != 0) {
   	KNLLOG("tstop [clcd total video time]\n");
      printk(KERN_WARNING "CLCD: Video lock acquire interrupted or failed\n");
		CLCD_TIMER_END(&fb->timer, &fb->timer.tot_video);
      return -EFAULT;
   }

#ifndef BCM4760_HACK
   /* engage the GE for alpha blending */
   if (clcdfb_ge_alpha_blend(fb) != 0) {
   	KNLLOG("tstop [clcd total video time]\n");
      printk(KERN_WARNING "CLCD: Alpha blending timeout or failed\n");
		CLCD_TIMER_END(&fb->timer, &fb->timer.tot_video);
      return -EFAULT;
   }
#endif

   clcdfb_fb_lock_release(&fb->video_lock);

   /* swap the Active/Inactive FBs */
   clcdfb_swap_fb(fb);

   clcdfb_vsync_lock_release(fb);

   CLCD_TIMER_END(&fb->timer, &fb->timer.tot_video);
   KNLLOG("tstop [clcd total video time]\n");

   return 0;
}
#endif

/*
 * Get LCD information including width, height, and bits per pixel
 */
static void clcdfb_get_info(LCD_Info_t *lcd_info_ptr, struct fb_info *info)
{
   struct clcd_fb *fb = to_clcd(info);

   lcd_info_ptr->width = fb->fb.var.xres;
   lcd_info_ptr->height = fb->fb.var.yres;
   lcd_info_ptr->bitsPerPixel = fb->fb.var.bits_per_pixel;
}

/*
 * Get the currently displayed row on the LCD
 */
static void clcdfb_get_vblank(struct fb_vblank *vblank, struct fb_info *info)
{
    unsigned long address;
   struct clcd_fb *fb = to_clcd(info);

   memset(vblank, 0, sizeof(*vblank));
   vblank->flags = FB_VBLANK_HAVE_VCOUNT;
   // read currently DMA address and convert to vcount 
   // vcount is relative to start of *unpanned* buffer i.e. 0 <= vcount < yres_virtual
   address = readl(fb->regs + CLCD_UCUR) - fb->fb.fix.smem_start;
   vblank->vcount = address / fb->fb.fix.line_length;
   // vblank->hcount = address % fb->fb.fix.line_length;
}

/*
 * Update the frame displayed on the LCD panel with the frame from the GUI FB
 */
static int
clcdfb_update(struct fb_info *info, LCD_DirtyRows_t *dirty_rows_ptr)
{
   struct clcd_fb *fb = to_clcd(info);

   KNLLOG("tstart [lcdfb update]\n");
   CLCD_TIMER_START(&fb->timer, &fb->timer.tot_update);

#ifdef CONFIG_FB_ARMCLCD_VIDEO
   if (atomic_read(&fb->video_on)) {
      if (clcdfb_fb_lock_acquire(&fb->holding_lock) != 0) {
         printk(KERN_WARNING "CLCD: Holding lock acquire interrupted or "
               "failed\n");
         return -EFAULT;
      }

      /* data transfer from the GUI FB to the Holding FB */
      CLCD_TIMER_START(&fb->timer, &fb->timer.g2h);
      KNLLOG("tstart [DMA from GUI to Holding FB]\n");
      clcdfb_dma_memcpy(&fb->dma, fb->gui_fb.phys_addr,
            fb->holding_fb.phys_addr, fb->gui_fb.len);
      KNLLOG("tstop [DMA from GUI to Holding FB]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.g2h);

      if (fb->validation_level & CLCD_VALIDATE_DMA) {
         if (clcdfb_fb_validation(fb->gui_fb.virt_ptr,
                  fb->holding_fb.virt_ptr, fb->gui_fb.len,
                  fb->panel->bpp, fb->skip_count) != 0) {
            printk(KERN_ERR "CLCD: GUI -> Holding FB DMA validation failed\n");
         }
      }

      clcdfb_fb_lock_release(&fb->holding_lock);

      /* video related process and get the result onto the LCD */
      if (clcdfb_video_process(fb) != 0) {
         printk(KERN_WARNING "CLCD: clcdfb_video_process failed\n");
         return -EFAULT;
      }

      KNLLOG("tstop [lcdfb update]\n");
      CLCD_TIMER_END(&fb->timer, &fb->timer.tot_update);
      /* video mode code ends here */
      return 0;
   }
#endif

   /*
    * The following code is for Video Off mode or when CONFIG_FB_ARMCLCD_VIDEO is
    * NOT defined
    */
   if (clcdfb_vsync_lock_acquire(fb) != 0) {
      printk(KERN_WARNING "CLCD: Vsync lock acquire interrupted or failed\n");
      return -EFAULT;
   }

   /* data transfer from the GUI FB to the Inactive FB */
   CLCD_TIMER_START(&fb->timer, &fb->timer.g2i);
   KNLLOG("tstart [DMA from GUI to Inactive FB]\n");
   clcdfb_dma_memcpy(&fb->dma, fb->gui_fb.phys_addr,
            fb->inactive_fb.phys_addr, fb->gui_fb.len);
   KNLLOG("tstop [DMA from GUI to Inactive FB]\n");
   CLCD_TIMER_END(&fb->timer, &fb->timer.g2i);

   if (fb->validation_level & CLCD_VALIDATE_DMA) {
      if (clcdfb_fb_validation(fb->gui_fb.virt_ptr,
            fb->inactive_fb.virt_ptr, fb->gui_fb.len,
            fb->panel->bpp, fb->skip_count) != 0) {
         printk(KERN_ERR "CLCD: GUI -> Inactive FB DMA validation failed\n");
      }
   }

   /* swap the Active/Inactive FBs */
   clcdfb_swap_fb(fb);
   clcdfb_vsync_lock_release(fb);

   CLCD_TIMER_END(&fb->timer, &fb->timer.tot_update);

   return 0;
}

#ifndef BCM4760_HACK
/*
 * copyarea wrapper that validates the parameters and invokes the real copyarea
 * function
 */
static void
clcdfb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
   struct fb_copyarea new_area;
   u32 width, height;

   if (area->width == 0 || area->height == 0 ||
       area->sx > info->var.xres || area->dx > info->var.xres ||
       area->sy > info->var.yres || area->dy > info->var.yres)
   {
      printk(KERN_WARNING "CLCD: Invalid copyarea parameters "
            "sx=%u sy=%u dx=%u dy=%u width=%u height=%u\n", area->sx, area->sy,
            area->dx, area->dy, area->width, area->height);
      return;
   }

   memcpy(&new_area, area, sizeof(new_area));

   /* chop off some out of bound area if there's any */
   width = area->width;
   height = area->height;
   if (area->sx + width > info->var.xres)
   {
      width = info->var.xres - area->sx;
   }
   if (area->dx + width > info->var.xres)
   {
      u32 new_width = info->var.xres - area->dx;
      width = width < new_width ? width : new_width;
   }
   if (area->sy + height > info->var.yres)
   {
      height = info->var.yres - area->sy;
   }
   if (area->dy + height > info->var.yres)
   {
      u32 new_height = info->var.yres - area->dy;
      height = height < new_height ? height : new_height;
   }
   new_area.width = width;
   new_area.height = height;

   clcdfb_ge_copyarea(info, &new_area);
}

/*
 * fillrect wrapper that validates the parameters and invokes the real fillrect
 * function
 */
static void
clcdfb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
   struct fb_fillrect new_rect;
   LCD_FillRectColor_t rect_c;
   u32 width, height;

   if (rect->width == 0 || rect->height == 0 ||
       rect->dx > info->var.xres || rect->dy > info->var.yres ||
       rect->color >= 16 || rect->rop != ROP_COPY) {
      printk(KERN_WARNING "CLCD: Invalid fillrect parameters dx=%u dy=%u "
            "width=%u height=%u, color=%u, rop=%u\n", rect->dx, rect->dy,
            rect->width, rect->height, rect->color, rect->rop);
      return;
   }

   memcpy(&new_rect, rect, sizeof(new_rect));

   /* chop off some out of bound area if there's any */
   width = rect->width;
   height = rect->height;
   if (rect->dx + width > info->var.xres) {
      width = info->var.xres - rect->dx;
   }
   if (rect->dy + height > info->var.yres) {
      height = info->var.yres - rect->dy;
   }
   new_rect.width = width;
   new_rect.height = height;

   /* construct the rect_c structure that uses the color directly */
   rect_c.dx = new_rect.dx;
   rect_c.dy = new_rect.dy;
   rect_c.width = new_rect.width;
   rect_c.height = new_rect.height;

   if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
      /* get color from the palette array */
      rect_c.rawColor = ((u32 *)info->pseudo_palette)[new_rect.color];
   }
   else {
      /* use color directly */
      rect_c.rawColor = new_rect.color;
   }

   clcdfb_ge_fillrect(info, &rect_c);
}

/*
 * Wrapper of clcdfb_fillrect that allows the user to use the raw color
 * code directly, since clcdfb_fillrect uses the palette interface in
 * true/direct color mode
 */
static void clcdfb_fillrect_color(struct fb_info *info,
      LCD_FillRectColor_t *rect_c)
{
   u32 red, green, blue, transp;
   u32 bpp = info->var.bits_per_pixel;
   struct fb_fillrect rect;

   if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
      /* extract color information */
      red = extract_bitfield(rect_c->rawColor, &info->var.red, bpp);
      green = extract_bitfield(rect_c->rawColor, &info->var.green, bpp);
      blue = extract_bitfield(rect_c->rawColor, &info->var.blue, bpp);
      transp = extract_bitfield(rect_c->rawColor, &info->var.transp, bpp);

      /*
       * Update color information stored in the palette, use the reserved
       * array index of 15
       */
      clcdfb_setcolreg(15, red, green, blue, transp, info);

      /* fillrect */
      rect.dx = rect_c->dx;
      rect.dy = rect_c->dy;
      rect.width = rect_c->width;
      rect.height = rect_c->height;
      rect.color = 15;
      rect.rop = ROP_COPY; /* support copy only */
   } else { /* use rawColor directly for other color modes */
      /* fillrect */
      rect.dx = rect_c->dx;
      rect.dy = rect_c->dy;
      rect.width = rect_c->width;
      rect.height = rect_c->height;
      rect.color = rect_c->rawColor;
      rect.rop = ROP_COPY; /* support copy only */
   }
   clcdfb_fillrect(info, &rect);
}

#endif

/*
 * Handle ioctols from user space
 */
static int
clcdfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
   int rval = 0;

   switch (cmd) {
      case LCD_IOCTL_INFO:
      {
         LCD_Info_t lcd_info;

         clcdfb_get_info(&lcd_info, info);
         rval = copy_to_user((void *)arg, &lcd_info, sizeof(LCD_Info_t));
         if (rval != 0) {
            printk(KERN_ERR "CLCD: copy_to_user failed\n");
            return -EFAULT;
         }
         return 0;
      }

      case LCDFB_IOCTL_UPDATE_LCD:
      {
         LCD_DirtyRows_t dirty_rows;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
         struct clcd_fb *fb = to_clcd(info);

         if (atomic_read(&fb->gthread.enable)) {
            printk(KERN_ERR "CLCD: GThread is currently running. GUI update "
                  "abandoned\n");
            return -EFAULT;
         }
#endif
         /*
          * TODO: Update full frame for now, but leave room for adding dirty
          * rows update in the future
          */
         if (copy_from_user(&dirty_rows, (LCD_DirtyRows_t *)arg,
                  sizeof(dirty_rows)) != 0) {
            printk(KERN_ERR "CLCD: copy_from_user failed\n");
            return -EFAULT;
         }

         rval = clcdfb_update(info, &dirty_rows);

         return rval;
      }

      case LCD_IOCTL_COPYAREA:
      {
         struct fb_copyarea area;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
         struct clcd_fb *fb = to_clcd(info);

         if (atomic_read(&fb->gthread.enable)) {
            printk(KERN_ERR "CLCD: GThread is currently running. copyarea "
                  "abandoned\n");
            return -EFAULT;
         }
#endif

         if (copy_from_user(&area, (struct fb_copyarea *)arg,
                  sizeof(area)) != 0) {
            return -EFAULT;
         }

         clcdfb_copyarea(info, &area);
         return 0;
      }

#ifndef BCM4760_HACK
      case LCD_IOCTL_FILLRECT_COLOR:
      {
         LCD_FillRectColor_t rect_c;
         struct clcd_fb *fb = to_clcd(info);

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
         if (atomic_read(&fb->gthread.enable)) {
            printk(KERN_ERR "CLCD: GThread is currently running. fillrect "
                  "abandoned\n");
            return -EFAULT;
         }
#endif

         if (copy_from_user(&rect_c, (LCD_FillRectColor_t *)arg,
                    sizeof(rect_c)) != 0) {
            return -EFAULT;
         }

         clcdfb_fillrect_color(info, &rect_c);
         return 0;
      }
#endif

      case FBIOGET_VBLANK:
      {
         struct fb_vblank vblank;
         clcdfb_get_vblank(&vblank, info);
         
         if (copy_to_user((void *)arg, &vblank, sizeof(vblank))) {
             return -EFAULT;
      }
      break;
      }
      
      default:
		 printk(KERN_ERR "CLCD-IOCTL Not supp:%X\n",cmd);
         return -EINVAL;
   }
   return 0;
}

#ifdef CONFIG_FB_ARMCLCD_VIDEO
static int
clcdfb_proc_video_ctrl_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Video Mode is %s\n",
         atomic_read(&fb->video_on) ? "On" : "Off");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
static int
clcdfb_proc_video_ctrl_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   int video_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &video_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (video_enable != 0)
      atomic_set(&fb->video_on, 1);
   else
      atomic_set(&fb->video_on, 0);

   return count;
}
#endif

static int
clcdfb_proc_timer_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int usec;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Performance timer is %s\n",
          clcdfb_timer_is_enabled(&fb->timer) ? "On" : "Off");

   usec = fb->timer.tot_update.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "Total Update Time = "
         "%u.%03u ms\n",
         usec / 1000, usec % 1000);
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   usec = fb->timer.tot_video.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "Total Video Update Time = "
         "%u.%03u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.g2h.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GUI FB -> Holding FB Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.h2i.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "Holding FB -> Inactive FB Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
#endif
#ifdef CLCD_GE_RASTER_WORKAROUND
   usec = fb->timer.g2r.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GUI FB -> Raster FB Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.r2g.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "Raster FB -> GUI FB Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
#endif
   usec = fb->timer.g2i.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GUI FB -> Inactive FB Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.ge_alpha.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GE Alpha Blending Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.ge_copyarea.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GE Copy Area Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.ge_fillrect.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "GE Fill Rect Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);
   usec = fb->timer.swap.delta / (timer_get_tick_rate() / 1000000) /
      TIME_SCALE;
   len += sprintf(buffer + len, "Active/Inactive FB Swap Time = "
         "%u.%06u ms\n",
         usec / 1000, usec % 1000);

   return len;
}

static int
clcdfb_proc_timer_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   int timer_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &timer_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (timer_enable != 0)
       clcdfb_timer_enable(&fb->timer);
   else
      clcdfb_timer_disable(&fb->timer);

   return count;
}

static int
clcdfb_proc_stats_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "\nStatistics Counters\n\n");
   len += sprintf(buffer + len, "GE IRQ:                      %u\n",
         fb->stats.irq_ge);
   len += sprintf(buffer + len, "CLCD IRQ - Total:            %u\n",
         fb->stats.irq_clcd);
   len += sprintf(buffer + len, "CLCD IRQ - VSync:            %u\n",
         fb->stats.irq_clcd_vsync);
   len += sprintf(buffer + len, "CLCD IRQ - FIFO Underflow:   %u\n",
         fb->stats.irq_clcd_fifo_under);
   len += sprintf(buffer + len, "CLCD IRQ - AHB Bus Error:    %u\n",
         fb->stats.irq_clcd_ahb_err);
#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   len += sprintf(buffer + len, "Video too long:              %u\n",
         fb->stats.video_long);
	len += sprintf(buffer + len, "Video overrun:               %u\n",
         fb->stats.video_overrun);
#endif
#ifdef CLCD_GE_RASTER_WORKAROUND
	len += sprintf(buffer + len, "GUI too long:                %u\n",
         fb->stats.gui_long);
	len += sprintf(buffer + len, "GUI overrun:                 %u\n",
         fb->stats.gui_overrun);
#endif
   len += sprintf(buffer + len, "\n");

   return len;
}

static int clcdfb_proc_stats_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;

   clcdfb_stats_reset(&fb->stats);

   return count;
}

static int
clcdfb_proc_validation_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "\nCurrent Data Validation Setting:\n\n");
   len += sprintf(buffer + len, "DMA (after each FB copy): %s\n",
         (fb->validation_level & CLCD_VALIDATE_DMA) ? "Enabled" : "Disabled");
   len += sprintf(buffer + len, "GE: %s\n",
         (fb->validation_level & CLCD_VALIDATE_GE) ? "Enabled" : "Disabled");

   return len;
}

static int clcdfb_proc_validation_write(struct file *file,
      const char __user *buffer, unsigned long count, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int validation_level;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &validation_level) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   fb->validation_level = validation_level;

   return count;
}

static int
clcdfb_proc_skip_count_read(char *buffer, char **start, off_t off, int count,
      int *eof, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Skip count is: %d\n", fb->skip_count);

   return len;
}

static int clcdfb_proc_skip_count_write(struct file *file,
      const char __user *buffer, unsigned long count, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;
   unsigned int skip_count;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &skip_count) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   fb->skip_count = skip_count;

   return count;
}

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_enable_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "VThread is %s\n",
         atomic_read(&vthread->enable) ? "Running" : "NOT Running");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_enable_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   int vthread_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &vthread_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (vthread_enable != 0)
      clcdfb_vthread_start(vthread);
   else
      clcdfb_vthread_stop(vthread);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_warning_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Warning messages for frame overrun / frame "
         "processing time is %s\n",
         atomic_read(&vthread->warning) ? "Enabled" : "Disabled");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_warning_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   int warning_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &warning_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (warning_enable != 0)
      atomic_set(&vthread->warning, 1);
   else
      atomic_set(&vthread->warning, 0);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_rate_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "VThread rate is %u fps\n",
         atomic_read(&vthread->rate));
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_rate_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int rate;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &rate) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   clcdfb_vthread_modify_rate(vthread, rate);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_thresh_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Threshold is %u ms\n",
         atomic_read(&vthread->thresh));
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_thresh_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int threshold;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &threshold) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

	atomic_set(&vthread->thresh, threshold);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_dimension_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Current video dimension is %u x %u\n",
         gfb->vfb.width, gfb->vfb.height);
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_vtest_dimension_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_vthread *vthread = (struct clcd_vthread *)data;
   unsigned int width, height;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u %u", &width, &height) != 2) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (clcdfb_set_video_fb(vthread->buf.phys_addr,
                           CLCD_VTHREAD_XOFFSET,
                           CLCD_VTHREAD_YOFFSET,
                           width,
                           height,
                           32) != 0) {
      printk(KERN_ERR "CLCD: video dimension incorrect\n");
      return -EFAULT;
   }

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_enable_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GThread is %s\n",
         atomic_read(&gthread->enable) ? "Running" : "NOT Running");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_enable_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   int gthread_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &gthread_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (gthread_enable != 0)
      clcdfb_gthread_start(gthread);
   else
      clcdfb_gthread_stop(gthread);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_warning_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Warning messages for frame overrun / frame "
         "processing time is %s\n",
         atomic_read(&gthread->warning) ? "Enabled" : "Disabled");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_warning_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   int warning_enable;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &warning_enable) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (warning_enable != 0)
      atomic_set(&gthread->warning, 1);
   else
      atomic_set(&gthread->warning, 0);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_rate_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GThread rate is %u fps\n",
         atomic_read(&gthread->rate));
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_rate_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int rate;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &rate) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   clcdfb_gthread_modify_rate(gthread, rate);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_thresh_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Threshold is %u ms\n",
         atomic_read(&gthread->thresh));
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_thresh_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int threshold;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &threshold) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

	atomic_set(&gthread->thresh, threshold);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_enable_copy_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Copyarea is %s in GThread\n",
         atomic_read(&gthread->enable_copy) ? "Enabled" : "NOT Enabled");
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_enable_copy_write(struct file *file,
      const char __user *buffer, unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   int enable_copy;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%d", &enable_copy) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   if (enable_copy != 0)
      atomic_set(&gthread->enable_copy, 1);
   else
      atomic_set(&gthread->enable_copy, 0);

   return count;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_copy_rate_read(char *buffer, char **start, off_t off,
      int count, int *eof, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GThread copyarea rate is %u copies "
         "per update\n", atomic_read(&gthread->copy_rate));
   return len;
}
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
static int
clcdfb_proc_gtest_copy_rate_write(struct file *file, const char __user *buffer,
      unsigned long count, void *data)
{
   struct clcd_gthread *gthread = (struct clcd_gthread *)data;
   unsigned int copy_rate;
   unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   if (copy_from_user(kernel_buffer, buffer, count)) {
      printk(KERN_WARNING "CLCD: copy_from_user failed\n");
      return -EFAULT;
   }

   if (sscanf(kernel_buffer, "%u", &copy_rate) != 1) {
      printk(KERN_WARNING "CLCD: proc write syntax error\n");
      return count;
   }

   atomic_set(&gthread->copy_rate, copy_rate);

   return count;
}
#endif

/*
 * Set up CLCD proc entries
 */
static int clcdfb_proc_setup(struct clcd_fb *fb)
{
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   struct proc_dir_entry *proc_video_ctrl;
#endif
   struct proc_dir_entry *proc_timer;
   struct proc_dir_entry *proc_stats;
   struct proc_dir_entry *proc_validation;
   struct proc_dir_entry *proc_skip_count;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   struct proc_dir_entry *proc_vtest_enable;
   struct proc_dir_entry *proc_vtest_warning;
   struct proc_dir_entry *proc_vtest_rate;
   struct proc_dir_entry *proc_vtest_thresh;
   struct proc_dir_entry *proc_vtest_dimension;

   struct proc_dir_entry *proc_gtest_enable;
   struct proc_dir_entry *proc_gtest_warning;
   struct proc_dir_entry *proc_gtest_rate;
   struct proc_dir_entry *proc_gtest_thresh;
   struct proc_dir_entry *proc_gtest_enable_copy;
   struct proc_dir_entry *proc_gtest_copy_rate;
#endif

   clcd_proc.parent_dir = proc_mkdir("clcd", NULL);
#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   clcd_proc.video_test_dir = proc_mkdir("dummyVideo", clcd_proc.parent_dir);
   clcd_proc.gui_test_dir = proc_mkdir("dummyGUI", clcd_proc.parent_dir);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
   proc_video_ctrl = create_proc_entry("videoCtrl", 0644,
         clcd_proc.parent_dir);
   if (proc_video_ctrl == NULL) {
      return -ENOMEM;
   }
   proc_video_ctrl->read_proc = clcdfb_proc_video_ctrl_read;
   proc_video_ctrl->write_proc = clcdfb_proc_video_ctrl_write;
   proc_video_ctrl->data = fb;
#endif

   proc_timer = create_proc_entry("perfTimer", 0644,
         clcd_proc.parent_dir);
   if (proc_timer == NULL) {
      return -ENOMEM;
   }
   proc_timer->read_proc = clcdfb_proc_timer_read;
   proc_timer->write_proc = clcdfb_proc_timer_write;
   proc_timer->data = fb;

   proc_stats = create_proc_entry("stats", 0644, clcd_proc.parent_dir);
   if (proc_stats == NULL) {
      return -ENOMEM;
   }
   proc_stats->read_proc = clcdfb_proc_stats_read;
   proc_stats->write_proc = clcdfb_proc_stats_write;
   proc_stats->data = fb;

   proc_validation = create_proc_entry("setValidation", 0644, clcd_proc.parent_dir);
   if (proc_stats == NULL) {
      return -ENOMEM;
   }
   proc_validation->read_proc = clcdfb_proc_validation_read;
   proc_validation->write_proc = clcdfb_proc_validation_write;
   proc_validation->data = fb;

   proc_skip_count = create_proc_entry("skipCount", 0644, clcd_proc.parent_dir);
   if (proc_stats == NULL) {
      return -ENOMEM;
   }
   proc_skip_count->read_proc = clcdfb_proc_skip_count_read;
   proc_skip_count->write_proc = clcdfb_proc_skip_count_write;
   proc_skip_count->data = fb;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   proc_vtest_enable = create_proc_entry("enable", 0644, clcd_proc.video_test_dir);
   if (proc_vtest_enable == NULL) {
      return -ENOMEM;
   }
   proc_vtest_enable->read_proc = clcdfb_proc_vtest_enable_read;
   proc_vtest_enable->write_proc = clcdfb_proc_vtest_enable_write;
   proc_vtest_enable->data = &fb->vthread;

   proc_vtest_warning = create_proc_entry("warning", 0644, clcd_proc.video_test_dir);
   if (proc_vtest_warning == NULL) {
      return -ENOMEM;
   }
   proc_vtest_warning->read_proc = clcdfb_proc_vtest_warning_read;
   proc_vtest_warning->write_proc = clcdfb_proc_vtest_warning_write;
   proc_vtest_warning->data = &fb->vthread;

   proc_vtest_rate = create_proc_entry("rate", 0644, clcd_proc.video_test_dir);
   if (proc_vtest_rate == NULL) {
      return -ENOMEM;
   }
   proc_vtest_rate->read_proc = clcdfb_proc_vtest_rate_read;
   proc_vtest_rate->write_proc = clcdfb_proc_vtest_rate_write;
   proc_vtest_rate->data = &fb->vthread;

   proc_vtest_thresh = create_proc_entry("threshold", 0644,
			clcd_proc.video_test_dir);
   if (proc_vtest_thresh == NULL) {
      return -ENOMEM;
   }
   proc_vtest_thresh->read_proc = clcdfb_proc_vtest_thresh_read;
   proc_vtest_thresh->write_proc = clcdfb_proc_vtest_thresh_write;
   proc_vtest_thresh->data = &fb->vthread;

   proc_vtest_dimension = create_proc_entry("dimension", 0644,
			clcd_proc.video_test_dir);
   if (proc_vtest_dimension == NULL) {
      return -ENOMEM;
   }
   proc_vtest_dimension->read_proc = clcdfb_proc_vtest_dimension_read;
   proc_vtest_dimension->write_proc = clcdfb_proc_vtest_dimension_write;
   proc_vtest_dimension->data = &fb->vthread;

   proc_gtest_enable = create_proc_entry("enable", 0644, clcd_proc.gui_test_dir);
   if (proc_gtest_enable == NULL) {
      return -ENOMEM;
   }
   proc_gtest_enable->read_proc = clcdfb_proc_gtest_enable_read;
   proc_gtest_enable->write_proc = clcdfb_proc_gtest_enable_write;
   proc_gtest_enable->data = &fb->gthread;

   proc_gtest_warning = create_proc_entry("warning", 0644, clcd_proc.gui_test_dir);
   if (proc_gtest_warning == NULL) {
      return -ENOMEM;
   }
   proc_gtest_warning->read_proc = clcdfb_proc_gtest_warning_read;
   proc_gtest_warning->write_proc = clcdfb_proc_gtest_warning_write;
   proc_gtest_warning->data = &fb->gthread;

   proc_gtest_rate = create_proc_entry("rate", 0644, clcd_proc.gui_test_dir);
   if (proc_gtest_rate == NULL) {
      return -ENOMEM;
   }
   proc_gtest_rate->read_proc = clcdfb_proc_gtest_rate_read;
   proc_gtest_rate->write_proc = clcdfb_proc_gtest_rate_write;
   proc_gtest_rate->data = &fb->gthread;

   proc_gtest_thresh = create_proc_entry("threshold", 0644,
			clcd_proc.gui_test_dir);
   if (proc_gtest_thresh == NULL) {
      return -ENOMEM;
   }
   proc_gtest_thresh->read_proc = clcdfb_proc_gtest_thresh_read;
   proc_gtest_thresh->write_proc = clcdfb_proc_gtest_thresh_write;
   proc_gtest_thresh->data = &fb->gthread;

   proc_gtest_enable_copy = create_proc_entry("enableCopy", 0644,
			clcd_proc.gui_test_dir);
   if (proc_gtest_enable_copy == NULL) {
      return -ENOMEM;
   }
   proc_gtest_enable_copy->read_proc = clcdfb_proc_gtest_enable_copy_read;
   proc_gtest_enable_copy->write_proc = clcdfb_proc_gtest_enable_copy_write;
   proc_gtest_enable_copy->data = &fb->gthread;

   proc_gtest_copy_rate = create_proc_entry("copyRate", 0644,
			clcd_proc.gui_test_dir);
   if (proc_gtest_copy_rate == NULL) {
      return -ENOMEM;
   }
   proc_gtest_copy_rate->read_proc = clcdfb_proc_gtest_copy_rate_read;
   proc_gtest_copy_rate->write_proc = clcdfb_proc_gtest_copy_rate_write;
   proc_gtest_copy_rate->data = &fb->gthread;
#endif

   return 0;
}

static int clcdfb_proc_remove(void)
{
   clcd_proc.parent_dir = proc_mkdir("clcd", NULL);
#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   clcd_proc.video_test_dir = proc_mkdir("dummyVideo", clcd_proc.parent_dir);
   clcd_proc.gui_test_dir = proc_mkdir("dummyGUI", clcd_proc.parent_dir);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   remove_proc_entry("enable", clcd_proc.video_test_dir);
   remove_proc_entry("warning", clcd_proc.video_test_dir);
   remove_proc_entry("rate", clcd_proc.video_test_dir);
   remove_proc_entry("threshold", clcd_proc.video_test_dir);
   remove_proc_entry("dimension", clcd_proc.video_test_dir);
   remove_proc_entry("dummyVideo", clcd_proc.parent_dir);

   remove_proc_entry("enable", clcd_proc.gui_test_dir);
   remove_proc_entry("warning", clcd_proc.gui_test_dir);
   remove_proc_entry("rate", clcd_proc.gui_test_dir);
   remove_proc_entry("threshold", clcd_proc.gui_test_dir);
   remove_proc_entry("dummyGUI", clcd_proc.parent_dir);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
   remove_proc_entry("videoCtrl", clcd_proc.parent_dir);
#endif

   remove_proc_entry("perfTimer", clcd_proc.parent_dir);
   remove_proc_entry("stats", clcd_proc.parent_dir);
   remove_proc_entry("setValidation", clcd_proc.parent_dir);
   remove_proc_entry("skipCount", clcd_proc.parent_dir);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
   remove_proc_entry("clcd", NULL);
#else
   remove_proc_entry("clcd", &proc_root);
#endif

   return 0;
}

static struct fb_ops clcdfb_ops = {
   .owner = THIS_MODULE,
   .fb_check_var = clcdfb_check_var,
   .fb_set_par = clcdfb_set_par,
   .fb_setcolreg = clcdfb_setcolreg,
   .fb_blank = clcdfb_blank,
   .fb_pan_display = clcdfb_pan_display,
#ifdef BCM4760_HACK
   .fb_fillrect = cfb_fillrect,
#else
   .fb_fillrect = clcdfb_fillrect,
#endif
   .fb_copyarea = clcdfb_copyarea,
   .fb_imageblit = cfb_imageblit,
   .fb_ioctl = clcdfb_ioctl,
   .fb_mmap = clcdfb_mmap,
};

static int clcdfb_register(struct clcd_fb *fb)
{
   int ret;

   fb->clk = clk_get(&fb->dev->dev, "CLCDCLK");
   if (IS_ERR(fb->clk)) {
      ret = PTR_ERR(fb->clk);
      goto out;
   }

   fb->fb.fix.mmio_start = fb->dev->res.start;
   fb->fb.fix.mmio_len = SZ_4K;

   fb->regs = ioremap(fb->fb.fix.mmio_start, fb->fb.fix.mmio_len);
   if (!fb->regs) {
      printk(KERN_ERR "CLCD: Unable to remap registers\n");
      ret = -ENOMEM;
      goto free_clk;
   }

   /* disable all LCD interrupts */
   writel(readl(fb->regs + CLCD_IENB) & ~(INTR_VCOMP | INTR_LNBU | INTR_FUF | INTR_MBERROR), fb->regs + CLCD_IENB);

   fb->fb.fbops = &clcdfb_ops;
   fb->fb.flags = FBINFO_FLAG_DEFAULT;
   fb->fb.pseudo_palette = fb->cmap;

   strncpy(fb->fb.fix.id, clcd_name, sizeof(fb->fb.fix.id));
   fb->fb.fix.type = FB_TYPE_PACKED_PIXELS;
   fb->fb.fix.type_aux = 0;
   fb->fb.fix.xpanstep = 0;
   fb->fb.fix.ypanstep = 1;
   fb->fb.fix.ywrapstep = 0;
   fb->fb.fix.accel = FB_ACCEL_NONE;

   fb->fb.var.xres = fb->panel->mode.xres;
   fb->fb.var.yres = fb->panel->mode.yres;
   fb->fb.var.xres_virtual = fb->panel->mode.xres;
   fb->fb.var.yres_virtual = fb->panel->mode.yres*2;

   /* TODO: only support RGB565 and URGB888 for now */
   if (fb->panel->bpp != 16 && fb->panel->bpp != 32) {
      printk(KERN_ERR "CLCD: Unsupported pixel depth = %u bpp\n",
            fb->panel->bpp);
      ret = -EFAULT;
      goto free_clk;
   }
   fb->fb.var.bits_per_pixel = fb->panel->bpp;

   fb->fb.var.grayscale = fb->panel->grayscale;
   fb->fb.var.pixclock = fb->panel->mode.pixclock;
   fb->fb.var.left_margin = fb->panel->mode.left_margin;
   fb->fb.var.right_margin = fb->panel->mode.right_margin;
   fb->fb.var.upper_margin = fb->panel->mode.upper_margin;
   fb->fb.var.lower_margin = fb->panel->mode.lower_margin;
   fb->fb.var.hsync_len = fb->panel->mode.hsync_len;
   fb->fb.var.vsync_len = fb->panel->mode.vsync_len;
   fb->fb.var.sync = fb->panel->mode.sync;
   fb->fb.var.vmode = fb->panel->mode.vmode;
   fb->fb.var.activate = FB_ACTIVATE_NOW;
   fb->fb.var.nonstd = 0;
   fb->fb.var.height = fb->panel->height;
   fb->fb.var.width = fb->panel->width;
   fb->fb.var.accel_flags = 0;

   fb->fb.monspecs.hfmin = 0;
   fb->fb.monspecs.hfmax = 100000;
   fb->fb.monspecs.vfmin = 0;
   fb->fb.monspecs.vfmax = 400;
   fb->fb.monspecs.dclkmin = 1000000;
   fb->fb.monspecs.dclkmax = 100000000;

   /* make sure that the bitfields are set appropriately */
   clcdfb_set_bitfields(fb, &fb->fb.var);

   /* allocate color map */
   fb_alloc_cmap(&fb->fb.cmap, CMAP_SIZE, 0);

   /* turn on LCD underflow and AHB bus error interrupts */
   writel(INTR_FUF | INTR_MBERROR, fb->regs + CLCD_IENB);

   fb_set_var(&fb->fb, &fb->fb.var);

   printk(KERN_INFO "BCM4760_CLCD: %s hardware, %s display\n",
         fb->board->name, fb->panel->mode.name);

   ret = register_framebuffer(&fb->fb);

   clcdfb_set_par(&fb->fb);

   if (ret == 0)
      goto out;

   printk(KERN_ERR "CLCD: Cannot register framebuffer (%d)\n", ret);

   iounmap(fb->regs);
 free_clk:
   clk_put(fb->clk);
 out:
   return ret;
}

/*
 * IRQ handler for the CLCD controller
 */
static irqreturn_t clcdfb_irq_clcd(int irq, void *data)
{
   u32 val, irq_status;
   struct clcd_fb *fb = (struct clcd_fb *)data;

   BUG_ON(!fb);

   fb->stats.irq_clcd++;

   /* get masked CLCD status */
   irq_status = readl(fb->regs + CLCD_STAT);

   /* ack to clear all CLCD interrupts */
   writel(irq_status, fb->regs + CLCD_INTR);

   /* new Active FB address loaded (Vsync) */
   if (irq_status & INTR_LNBU) {
      /* disable this interrupt */
      val = readl(fb->regs + CLCD_IENB);
      writel(val & ~INTR_LNBU, fb->regs + CLCD_IENB);

      /* signal for the completion of Vsync */
      clcdfb_vsync_lock_complete(fb);

      fb->stats.irq_clcd_vsync++;
   }

   /* FIFO underflow */
   if (irq_status & INTR_FUF) {
      fb->stats.irq_clcd_fifo_under++;
   }

   /* AHB master bus error */
   if (irq_status & INTR_MBERROR) {
      fb->stats.irq_clcd_ahb_err++;
      printk(KERN_ERR "CLCD: AHB Bus Error\n");
   }

   /* Clear VIC0 interrupt */
   writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));

   return IRQ_HANDLED;
}

#ifndef BCM4760_HACK
/*
 * IRQ handler for the GE
 */
static irqreturn_t clcdfb_irq_ge(int irq, void *data)
{
   struct clcd_fb *fb = (struct clcd_fb *)data;

   BUG_ON(!fb);
   fb->stats.irq_ge++;

   /* TODO: see revised GE driver to modify this sequence */
   /* disable GE interrupt */
   intcHw_irq_disable((void *)INTCHW_INTC0, INTCHW_INTC0_GE);
   /* clear interrupt*/
   geHw_clearInterrupt();
   /* signal the completion of the GE operation */
   clcdfb_ge_lock_complete(fb);

   return IRQ_HANDLED;
}
#endif

static int clcdfb_probe(struct amba_device *dev, void *id)
{
	struct clcd_board *board;
	struct clcd_fb *fb;
	int ret;

	board = dev->dev.platform_data;
	if (!board) {
		return -EINVAL;
	}

	ret = amba_request_regions(dev, NULL);
	if (ret) {
		printk(KERN_ERR "CLCD: Unable to reserve regs region\n");
		goto out;
	}

   fb = kzalloc(sizeof(struct clcd_fb), GFP_KERNEL);
   if (!fb) {
      printk(KERN_INFO "CLCD: Could not allocate new clcd_fb struct\n");
      ret = -ENOMEM;
      goto free_region;
   }

   fb->dev = dev;
   fb->board = board;

   /* select LCD panel type before setup */
// [JLH]   if (gpanel >= CLCD_MAX_PANEL)
// [JLH]     gpanel = SEIKO_RA169Z;

   /* BELR: By setting the panel_index field to '5', we select the panel */
   /* "wvga565_800x480_WithPLL" which is defined in                      */
   /* mach-bcm476x/core.c.                                               */
   /* This gets the big screen to work.                                  */

#ifdef CONFIG_FB_BCM4760_CLCD_5_1
   fb->panel_index = 5;
#else
   fb->panel_index = gpanel;
#endif

	ret = fb->board->setup(fb);
	if (ret)
		goto free_fb;

	if (fb->board->panel_setup)
		fb->board->panel_setup(fb->clk);

   CLCD_DEBUG("CLCD: GUI FB virt=0x%lx phys=0x%lx len=%u\n",
         (unsigned long)fb->gui_fb.virt_ptr,
         (unsigned long)fb->gui_fb.phys_addr,
         fb->gui_fb.len);
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   CLCD_DEBUG("CLCD: Holding FB virt=0x%lx phys=0x%lx len=%u\n",
         (unsigned long)fb->holding_fb.virt_ptr,
         (unsigned long)fb->holding_fb.phys_addr,
         fb->holding_fb.len);
#endif
   CLCD_DEBUG("CLCD: Active FB virt=0x%lx phys=0x%lx len=%u\n",
         (unsigned long)fb->active_fb.virt_ptr,
         (unsigned long)fb->active_fb.phys_addr,
         fb->active_fb.len);
   CLCD_DEBUG("CLCD: Inactive FB virt=0x%lx phys=0x%lx len=%u\n",
         (unsigned long)fb->inactive_fb.virt_ptr,
         (unsigned long)fb->inactive_fb.phys_addr,
         fb->inactive_fb.len);
#ifdef CLCD_GE_RASTER_WORKAROUND
   CLCD_DEBUG("CLCD: Raster FB virt=0x%lx phys=0x%lx len=%u\n",
         (unsigned long)fb->raster_fb.virt_ptr,
         (unsigned long)fb->raster_fb.phys_addr,
         fb->raster_fb.len);
#endif

   /* initialize locking mechanisms */
#ifdef CLCD_GE_RASTER_WORKAROUND
   clcdfb_fb_lock_init(&fb->raster_lock);
#endif
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   clcdfb_fb_lock_init(&fb->holding_lock);
   clcdfb_fb_lock_init(&fb->video_lock);
#endif
   clcdfb_vsync_lock_init(fb);
#ifndef BCM4760_HACK
   clcdfb_ge_lock_init(fb);
#endif

   gfb = fb;
   clcd_is_initialized = 1;

   /* initialize DMA */
   ret = clcdfb_dma_init(&fb->dma);
   if (ret != 0) {
      printk(KERN_ERR "CLCD: DMA initialization failed\n");
      goto free_fb;
   }

   /* initialize performance timers */
   clcdfb_timer_init(&fb->timer);

   /* initialize stats counters */
   clcdfb_stats_init(&fb->stats);

   /* set up procfs */
   ret = clcdfb_proc_setup(fb);
   if (ret != 0) {
      printk(KERN_ERR "CLCD: procfs setup error\n");
      goto remove_procfs;
   }

   ret = clcdfb_register(fb);
   if (ret)
      goto remove_procfs;

   /* reserve the CLCD controller IRQ line */
   ret = request_irq(dev->irq[0], clcdfb_irq_clcd, IRQF_SHARED, fb->fb.fix.id, fb);
   if (ret < 0) {
      printk(KERN_ERR "CLCD: IRQ request for CLCD failed\n");
      goto remove_board;
   }

#ifndef BCM4760_HACK
	/* reserve the GE IRQ line */
	ret = request_irq(IRQ_GE,clcdfb_irq_ge, IRQF_SHARED, "CLCD_GE", fb);
	if (ret < 0) {
		printk(KERN_ERR "CLCD: IRQ request for GE failed\n");
		goto remove_irq_clcd;
	}
#endif

	if (fb->board->panel_init)
		fb->board->panel_init(fb->clk);

	ret = clcdfb_register(fb);
	if (ret)
		goto remove_irqs;

	amba_set_drvdata(dev, fb);

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   ret = clcdfb_vthread_init(&fb->vthread);
   if (ret < 0) {
      printk(KERN_ERR "CLCD: Vthread init failed\n");
      goto remove_vthread;
   }

   ret = clcdfb_gthread_init(&fb->gthread);
   if (ret < 0) {
      printk(KERN_ERR "CLCD: Gthread init failed\n");
      goto remove_gthread;
   }
#endif

   fb->validation_level = 0;

   fb->skip_count = 0;

   /* put some default drawings on the GUI FB */
   clcdfb_default_draw(&fb->gui_fb);

   goto out;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
 remove_gthread:
   clcdfb_gthread_term(&fb->gthread);
 remove_vthread:
   clcdfb_vthread_term(&fb->vthread);
#endif
remove_irqs:
#ifndef BCM4760_HACK
   free_irq(IRQ_GE, 0);
#endif
   free_irq(dev->irq[0], 0);
 remove_board:
   fb->board->remove(fb);
 remove_procfs:
   clcdfb_proc_remove();
 free_fb:
   kfree(fb);
 free_region:
   amba_release_regions(dev);
 out:
   return ret;
}

static int clcdfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
   struct clcd_fb *fb = to_clcd(info);
   u32 xoffset, yoffset;
   unsigned long ustart = fb->fb.fix.smem_start;
   unsigned long lstart;


#ifdef VERBOSE
   DBG_MSG("clcdfb_pan_display\n");
#endif

//   xoffset = ROUND_DOWN_TO(var->xoffset, 2);
   xoffset = 0;			/* X panning not implemented */
   yoffset = var->yoffset;

   if ((xoffset + var->xres > var->xres_virtual) ||
       (yoffset + var->yres > var->yres_virtual))
      return -EINVAL;

   BUG_ON(ustart == 0);

   ustart += (yoffset * fb->fb.fix.line_length) + xoffset;
//   BufferSizeRoundedUp = var_info.xres * var_info.yres*(var_info.bits_per_pixel>>3);
//   if ( (ustart % PAGE_SIZE) != 0 )
//   {
//      ustart += (PAGE_SIZE - ((var->xres * var->yres*(var->bits_per_pixel>>3)) % PAGE_SIZE));
//   }

   BUG_ON(ustart & 0x03);
   lstart = ustart + fb->fb.var.yres * fb->fb.fix.line_length / 2;

   /* program the new Active FB address */
   writel(ustart, fb->regs + CLCD_UBAS);
   writel(lstart, fb->regs + CLCD_LBAS);

//   dinfo->vsync.pan_offset = offset;
//   if ((var->activate & FB_ACTIVATE_VBL) &&
//       !intelfbhw_enable_irq(dinfo))
//      dinfo->vsync.pan_display = 1;
//   else {
//      dinfo->vsync.pan_display = 0;
//      OUTREG(DSPABASE, offset);
//   }

   return 0;
}

static int clcdfb_remove(struct amba_device *dev)
{
   struct clcd_fb *fb = amba_get_drvdata(dev);

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   clcdfb_gthread_term(&fb->gthread);
   clcdfb_vthread_term(&fb->vthread);
#endif

   clcd_is_initialized = 0;
   gfb = NULL;

#ifndef BCM4760_HACK
   free_irq(IRQ_GE, 0);
#endif
   free_irq(dev->irq[0], 0);

   clcdfb_proc_remove();

   amba_set_drvdata(dev, NULL);

   clcdfb_disable(fb);
   unregister_framebuffer(&fb->fb);
   iounmap(fb->regs);
   clk_put(fb->clk);

   fb->board->remove(fb);

   kfree(fb);

   amba_release_regions(dev);

   return 0;
}

static int clcdfb_suspend(struct amba_device *dev, pm_message_t pm)
{
	struct clcd_fb *fb = amba_get_drvdata(dev);

	printk(KERN_INFO "Disabling bcm4760_clcd for suspend.\n");

	if (fb->board->panel_suspend)
		fb->board->panel_suspend(fb->clk);

	return 0;
}

static int clcdfb_resume(struct amba_device *dev)
{
	struct clcd_fb *fb = amba_get_drvdata(dev);

	printk(KERN_INFO "Enabling bcm4760_clcd after resume.\n");

	/* disable all LCD interrupts */
	writel(readl(fb->regs + CLCD_IENB) & ~(INTR_VCOMP | INTR_LNBU | INTR_FUF | INTR_MBERROR), fb->regs + CLCD_IENB);

	/* turn on LCD underflow and AHB bus error interrupts */
	writel(INTR_FUF | INTR_MBERROR, fb->regs + CLCD_IENB);

	if (fb->board->panel_resume)
		fb->board->panel_resume(fb->clk);

	fb->fb.var.activate = FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW;

	/* Setup LCD controller based on VAR */
	fb_set_var(&fb->fb, &fb->fb.var);

	return 0;
}


static struct amba_id clcdfb_id_table[] = {
   {
      .id   = 0x00041110,
      .mask = 0x000ffffe,
   },
   { 0, 0 },
};

static struct amba_driver clcd_driver = {
   .drv     = {
      .name = "clcd-pl11x",
   },
   .probe      = clcdfb_probe,
   .remove     = clcdfb_remove,
   .suspend    = clcdfb_suspend,
   .resume     = clcdfb_resume,
   .id_table   = clcdfb_id_table,
};

static int __init amba_clcdfb_init(void)
{
   if (fb_get_options("ambafb", NULL))
      return -ENODEV;

   return amba_driver_register(&clcd_driver);
}

module_init(amba_clcdfb_init);

static void __exit amba_clcdfb_exit(void)
{
   amba_driver_unregister(&clcd_driver);
}

module_exit(amba_clcdfb_exit);

MODULE_DESCRIPTION("ARM PrimeCell PL110 CLCD core driver");
MODULE_LICENSE("GPL");

/*
 * Public Functions
 */

/*
 * Check to see if the CLCD driver has been initialized
 */
int clcdfb_is_initialized(void)
{
   return clcd_is_initialized;
}
EXPORT_SYMBOL(clcdfb_is_initialized);

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * The caller supplies the Video FB related parameters including the virtual
 * address, physical address, frame size, xoffset, yoffset, width, height,
 * and bpp (bits per pixel) to the CLCD driver by callling this function.
 *
 * This function should only be called after the CLCD driver has been
 * initialized and after the Video FB has been allocated by the caller
 */
int clcdfb_set_video_fb(dma_addr_t phys_addr,
                        unsigned int xoffset,
                        unsigned int yoffset,
                        unsigned int width,
                        unsigned int height,
                        unsigned int bpp)
{
   if (clcdfb_is_initialized() == 0 || gfb == NULL) {
      printk(KERN_ERR "CLCD: Driver is not initialized\n");
      return -EFAULT;
   }

   if (phys_addr == 0 ||
         xoffset + width > gfb->fb.var.xres ||
         yoffset + height > gfb->fb.var.yres ||
         bpp != 32) {
      printk(KERN_ERR "CLCD: Invalid Video FB parameters\n");
      return -EINVAL;
   }

   gfb->vfb.phys_addr = phys_addr;
   gfb->vfb.width = width;
   gfb->vfb.height = height;
   gfb->vfb.pitch = width;
   gfb->vfb.offset = (yoffset * width * bpp / 8) + (xoffset * bpp / 8);
   return 0;
}
EXPORT_SYMBOL(clcdfb_set_video_fb);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Call before starting video decoding. The caller needs to make sure the CLCD
 * driver has been been initialized and the Video FB has been set
 * (clcdfb_set_video_fb) before calling this function
 */
int clcdfb_video_decode_start(void)
{
   /* acquire the video lock */
   if (clcdfb_fb_lock_acquire(&gfb->video_lock) != 0) {
      printk(KERN_WARNING "CLCD: Video lock acquire interrupted or failed\n");
      return -EFAULT;
   }

   return 0;
}
EXPORT_SYMBOL(clcdfb_video_decode_start);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Call before ending video decoding. The caller needs to make sure the CLCD
 * driver has been been initialized and the Video FB has been set
 * (clcdfb_set_video_fb) before calling this function
 */
int clcdfb_video_decode_end(void)
{
   clcdfb_fb_lock_release(&gfb->video_lock);
   if (clcdfb_video_process(gfb) != 0) {
      printk(KERN_WARNING "CLCD: clcdfb_video_process failed\n");
      return -EFAULT;
   }
   return 0;
}
EXPORT_SYMBOL(clcdfb_video_decode_end);
#endif
