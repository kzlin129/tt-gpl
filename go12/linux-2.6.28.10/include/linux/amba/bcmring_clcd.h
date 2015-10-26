/*
 * linux/include/linux/amba/bcmring_clcd.h -- Integrator LCD panel.
 *
 * David A Rusling
 *
 * Copyright (C) 2001 ARM Limited
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/fb.h>
#include <asm/arch/dma.h>
#include <linux/broadcom/timer.h>

#undef CLCD_DBG
#define CLCD_DBG 0

/*
 * The current GE does not support raster operations in address-decrementing
 * mode. Therefore, in some cases of the copyarea, when src and dest overlap
 * with each other, and the dest has a higher address than the src, data
 * overwritten will occur.
 *
 * The workaround covers those cases and uses a tmp buffer to store the
 * result of the copied area, and move it back to the FB where the copyarea
 * was originally performed
 */
#define CLCD_GE_RASTER_WORKAROUND

#if (CLCD_DBG == 1)
#define CLCD_DEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define CLCD_DEBUG(fmt, args...) 
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
#define CLCD_VIDEO_THRESH       40 /* default to 40 ms */
#define CLCD_GUI_THRESH         40 /* default to 40 ms */

#define CLCD_VTHREAD_RATE       30 /* default to 30 fps */
#define CLCD_VTHREAD_XOFFSET    0
#define CLCD_VTHREAD_YOFFSET    0
#define CLCD_GTHREAD_RATE       30 /* default to 30 fps */
#endif

#define CLCD_VALIDATE_DMA       0x1
#define CLCD_VALIDATE_GE        0x2

/*
 * CLCD Controller Internal Register addresses
 */
#define CLCD_TIM0       0x00000000
#define CLCD_TIM1       0x00000004
#define CLCD_TIM2       0x00000008
#define CLCD_TIM3       0x0000000c
#define CLCD_UBAS       0x00000010
#define CLCD_LBAS       0x00000014

#if !defined(CONFIG_ARCH_VERSATILE) && !defined(CONFIG_ARCH_REALVIEW) && !defined(CONFIG_ARCH_BCMRING) && !defined(CONFIG_PLAT_BCM476X)
#define CLCD_IENB       0x00000018
#define CLCD_CNTL       0x0000001c
#else
/*
 * Someone rearranged these two registers on the Versatile
 * platform...
 */
#define CLCD_IENB       0x0000001c
#define CLCD_CNTL       0x00000018
#endif

#define CLCD_STAT       0x00000024
#define CLCD_INTR       0x00000028
#define CLCD_UCUR       0x0000002C
#define CLCD_LCUR       0x00000030
#define CLCD_PALL       0x00000200
#define CLCD_PALETTE    0x00000200

#define TIM2_CLKSEL     (1 << 5)
#define TIM2_IVS        (1 << 11)
#define TIM2_IHS        (1 << 12)
#define TIM2_IPC        (1 << 13)
#define TIM2_IOE        (1 << 14)
#define TIM2_BCD        (1 << 26)

#define INTR_FUF        (1 << 1)
#define INTR_LNBU       (1 << 2)
#define INTR_VCOMP      (1 << 3)
#define INTR_MBERROR    (1 << 4)

#define CNTL_LCDEN      (1 << 0)
#define CNTL_LCDBPP1    (0 << 1)
#define CNTL_LCDBPP2    (1 << 1)
#define CNTL_LCDBPP4    (2 << 1)
#define CNTL_LCDBPP8    (3 << 1)
#define CNTL_LCDBPP16   (4 << 1)
#define CNTL_LCDBPP16_565  (6 << 1)
#define CNTL_LCDBPP24      (5 << 1)
#define CNTL_LCDBW      (1 << 4)
#define CNTL_LCDTFT     (1 << 5)
#define CNTL_LCDMONO8   (1 << 6)
#define CNTL_LCDDUAL    (1 << 7)
#define CNTL_BGR        (1 << 8)
#define CNTL_BEBO       (1 << 9)
#define CNTL_BEPO       (1 << 10)
#define CNTL_LCDPWR     (1 << 11)
#define CNTL_LCDVCOMP(x)   ((x) << 12)
#define CNTL_LDMAFIFOTIME  (1 << 15)
#define CNTL_WATERMARK     (1 << 16)

struct clcd_panel {
   struct fb_videomode mode;
   signed short width; /* width in mm */
   signed short height; /* height in mm */
   u32 tim2;
   u32 tim3;
   u32 cntl;
   unsigned int bpp:8,
                fixedtimings:1,
                grayscale:1;
   unsigned int connector;
};

struct clcd_regs {
   u32 tim0;
   u32 tim1;
   u32 tim2;
   u32 tim3;
   u32 cntl;
   unsigned long pixclock;
};

struct clcd_fb;
struct clk;

struct clcd_panel_cntrl {
	void (*enable)  (struct clk *clk);
	void (*disable) (struct clk *clk);
	void (*suspend) (struct clk *clk);
	void (*resume)  (struct clk *clk);
	void (*init)    (struct clk *clk);
	void (*setup)   (struct clk *clk);
};

/* the board-type specific routines */
struct clcd_board {
	const char *name;

	/* Optional.  Check whether the var structure is acceptable
	 * for this display. */
	int (*check)(struct clcd_fb *fb, struct fb_var_screeninfo *var);

	/* Compulsory. Decode fb->fb.var into regs->*.  In the case of
	 * fixed timing, set regs->* to the register values required. */
	void (*decode)(struct clcd_fb *fb, struct clcd_regs *regs);

	/* Setup platform specific parts of CLCD driver */
	int (*setup)(struct clcd_fb *);
   
	/* mmap the framebuffer memory */
	int (*mmap)(struct clcd_fb *, struct vm_area_struct *);

	/* Remove platform specific parts of CLCD driver */
	void (*remove)(struct clcd_fb *);

	/********************** Panel management **************/
	void (*panel_enable)  (struct clk *clk);
	void (*panel_disable) (struct clk *clk);
	void (*panel_suspend) (struct clk *clk);
	void (*panel_resume)  (struct clk *clk);
	void (*panel_init)    (struct clk *clk);
	void (*panel_setup)   (struct clk *clk);
};

struct clcd_device {
	struct clcd_panel       *timings;
	struct clcd_panel_cntrl *cntrl;
};

/*
 * Data structure of the generic locking mechanism
 */
struct clcd_lock {
   struct completion complete;
   struct semaphore lock;
};

/*
 * Data structure of the frame buffer
 */
struct clcd_buf {
   void *virt_ptr;
   dma_addr_t phys_addr;
   u32 len;
};

/* Data structure for DMA */
struct clcd_dma {
   DMA_Handle_t handle;
   struct clcd_lock lock;
};

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
/*
 * Data structure of the dummy video thread for testing
 */
struct clcd_vthread {      
   volatile atomic_t enable; /* flag to turn on or turn off the threads */
   volatile atomic_t rate; /* frame rate in frames per second */
   volatile atomic_t warning; /* flag to turn on/off warning msgs */

   struct clcd_buf buf;

   long sig_id; /* ID of the signaling thread */
   long proc_id; /* ID of the thread that does frame processing */
   struct completion proc_exit; /* exit sync of the frame processing thread */
   struct completion sig_exit; /* exit sync of the signaling thread */
   struct completion wait_to_start; /* sync between the two threads */

   struct completion wake_up; /* to wake up the process thread */
   volatile atomic_t in_process; /* flag to indicate the process thread is running */

   /*
    * Threshold in ms to detect if a single frame processing has been taking
    * too long
    */
   volatile atomic_t thresh;
};
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
/*
 * Data structure of the dummy GUI thread (animation) for testing
 */
struct clcd_gthread {
   volatile atomic_t enable; /* flag to turn on or turn off the threads */
   volatile atomic_t rate; /* frame rate in frames per second */
   volatile atomic_t warning; /* flag to turn on/off warning msgs */

   long sig_id; /* ID of the signaling thread */
   long proc_id; /* ID of the thread that does frame processing */
   struct completion proc_exit; /* exit sync of the frame processing thread */
   struct completion sig_exit; /* exit sync of the signaling thread */
   struct completion wait_to_start; /* sync between the two threads */

   struct completion wake_up; /* to wake up the process thread */
   volatile atomic_t in_process; /* flag to indicate the process thread is running */

   /*
    * Threshold in ms to detect if a single frame processing has been taking
    * too long
    */
   volatile atomic_t thresh;

   volatile atomic_t enable_copy; /* flag to turn on/off copyarea */
   volatile atomic_t copy_rate; /* how many copies per update */
   struct fb_copyarea area; /* copyarea to be performed on GThread */
};
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Data structure of the Video FB including some alpha blending parameters
 */
struct clcd_vfb {
   dma_addr_t phys_addr;
   /*
    * Offset in bytes from the starting address of the Inactive FB where alpha
    * blending is performed
    */
   u32 offset; 
   u32 width;
   u32 height;
   u32 pitch;
};
#endif

struct clcd_tval {
   timer_tick_count_t start; /* start time */
   timer_tick_count_t delta; /* time span */
};

/*
 * Performance timer data structure
 */
struct clcd_timer {
   atomic_t enable;
   struct clcd_tval tot_update; /* total time of the entire update */
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   struct clcd_tval tot_video; /* total time of the entire video update */
   struct clcd_tval g2h; /* from GUI FB to Holding FB */
#ifdef CLCD_GE_RASTER_WORKAROUND
   struct clcd_tval g2r; /* from GUI FB to Raster FB */
   struct clcd_tval r2g; /* from Raster FB to GUI FB */
#endif
   struct clcd_tval h2i; /* from Holding FB to Inactive FB */
#endif
   struct clcd_tval g2i; /* from GUI FB to Inactive FB */
   struct clcd_tval ge_alpha; /* GE alpha blending */
   struct clcd_tval ge_copyarea; /* GE copy area */
   struct clcd_tval ge_fillrect; /* GE fill rect */
   struct clcd_tval swap; /* Active/Inactive FB swap */
};

/*
 * Statistics counter data structure
 */
struct clcd_stats {
   u32 irq_ge; /* GE IRQ counts */
   u32 irq_clcd; /* CLCD IRQ counts */
   u32 irq_clcd_vsync; /* CLCD Vsync counts */
   u32 irq_clcd_fifo_under; /* CLCD FIFO underflow counts */
   u32 irq_clcd_ahb_err; /* AHB master bus error */
#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   u32 video_long;
   u32 video_overrun;
   u32 gui_long;
   u32 gui_overrun;
#endif
};

/* LCD panel type */
enum clcd_panel_type {
   SEIKO_RA167Z = 0,
   SEIKO_RA169Z,
   SAMSUNG_LMS700KF01,
   CLCD_MAX_PANEL
};

struct amba_device;

/* this data structure describes each frame buffer device we find */
struct clcd_fb {
   struct fb_info fb;
   struct amba_device *dev;
   struct clk *clk;
   struct clcd_panel *panel;
   struct clcd_board *board;
   void *board_data;
   void __iomem *regs;
   u32 clcd_cntl;
   u32 cmap[16];

/* [JLH] make the following conditional since we're doing this different. */
#ifdef CONFIG_FB_BCM476X_CLCD
   int panel_index;
#else
   enum clcd_panel_type panel_index;
#endif
   struct clcd_dma dma;

#ifdef CONFIG_FB_ARMCLCD_VIDEO
   /* flag to indicate video is on/off */
   atomic_t video_on;
#endif

   int skip_count;

   /* GUI FB, mapped to the Linux frame buffer */
   struct clcd_buf gui_fb;
#ifdef CLCD_GE_RASTER_WORKAROUND
   /*
    * Raster FB, serves as a tmp FB to store the result of a copyarea in
    * some cases where memory overwritten occurs
    */
   struct clcd_buf raster_fb;
#endif
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   /* Holding FB, holds updated frame for alpha blending */
   struct clcd_buf holding_fb;
   /* Video FB, stores the video frame, allocated in the video driver */
   struct clcd_vfb vfb;
#endif
   /* Active FB, stores the frame currently displayed on the LCD panel */
   struct clcd_buf active_fb;
   /* Inactive FB, stores the frame to be swapped with the Active FB */
   struct clcd_buf inactive_fb;

#ifdef CONFIG_BCMRING_GEV3_CMODEL
   /*
    * GUI Validation FB, stores the result of software-modeled GE operations 
    * related to GUI operations and updates including copy area and fill
    * color, etc. to be compared with the HW result
    */
   struct clcd_buf gui_validation_fb;
#endif

   /*
    * Video Validation FB, stores the result of software-modeled alpha
    * blending, to be compared with the HW result
    */
   struct clcd_buf video_validation_fb;

#ifdef CLCD_GE_RASTER_WORKAROUND
   /* lock that protects the Raster FB */
   struct semaphore raster_lock;
#endif
#ifdef CONFIG_FB_ARMCLCD_VIDEO
   /* lock that protects the Holding FB */
   struct semaphore holding_lock;
   /* lock that protects the Video FB */
   struct semaphore video_lock;
#endif
   struct clcd_lock vsync_lock;
   /* lock that protects all GE-related operations */
   struct clcd_lock ge_lock;

   /* performance timer */
   struct clcd_timer timer;

   /* statistics counter */
   struct clcd_stats stats;

   volatile u32 validation_level;

#ifdef CONFIG_FB_ARMCLCD_VIDEO_TEST
   struct clcd_vthread vthread;
   struct clcd_gthread gthread;
#endif
};

static inline void clcdfb_decode(struct clcd_fb *fb, struct clcd_regs *regs)
{
   u32 val, cpl;

   /*
    * Program the CLCD controller registers and start the CLCD
    */
   val = ((fb->fb.var.xres / 16) - 1) << 2;
   val |= (fb->fb.var.hsync_len - 1) << 8;
   val |= (fb->fb.var.right_margin - 1) << 16;
   val |= (fb->fb.var.left_margin - 1) << 24;
   regs->tim0 = val;

   val = fb->fb.var.yres;
   if (fb->panel->cntl & CNTL_LCDDUAL)
      val /= 2;
   val -= 1;
   val |= (fb->fb.var.vsync_len - 1) << 10;
   val |= fb->fb.var.lower_margin << 16;
   val |= fb->fb.var.upper_margin << 24;
   regs->tim1 = val;

   val = fb->panel->tim2;
   val |= fb->fb.var.sync & FB_SYNC_HOR_HIGH_ACT  ? 0 : TIM2_IHS;
   val |= fb->fb.var.sync & FB_SYNC_VERT_HIGH_ACT ? 0 : TIM2_IVS;

   cpl = fb->fb.var.xres_virtual;
   if (fb->panel->cntl & CNTL_LCDTFT) /* TFT */
      /* / 1 */;
   else if (!fb->fb.var.grayscale) /* STN color */
      cpl = cpl * 8 / 3;
   else if (fb->panel->cntl & CNTL_LCDMONO8) /* STN monochrome, 8bit */
      cpl /= 8;
   else /* STN monochrome, 4bit */
      cpl /= 4;

   regs->tim2 = val | ((cpl - 1) << 16);

   regs->tim3 = fb->panel->tim3;

   val = fb->panel->cntl;
   if (fb->fb.var.grayscale)
      val |= CNTL_LCDBW;

   switch (fb->fb.var.bits_per_pixel) {
   case 1:
      val |= CNTL_LCDBPP1;
      break;
   case 2:
      val |= CNTL_LCDBPP2;
      break;
   case 4:
      val |= CNTL_LCDBPP4;
      break;
   case 8:
      val |= CNTL_LCDBPP8;
      break;
   case 16:
      /*
       * PL110 cannot choose between 5551 and 565 modes in
       * its control register
       */
      if ((fb->dev->periphid & 0x000fffff) == 0x00041110)
         val |= CNTL_LCDBPP16;
      else if (fb->fb.var.green.length == 5)
         val |= CNTL_LCDBPP16;
      else
         val |= CNTL_LCDBPP16_565;
      break;
   case 32:
      val |= CNTL_LCDBPP24;
      break;
   }

   regs->cntl = val;
   regs->pixclock = fb->fb.var.pixclock;
}

static inline int clcdfb_check(struct clcd_fb *fb, struct fb_var_screeninfo *var)
{
   var->xres = (var->xres + 15) & ~15;
   var->yres = (var->yres + 1) & ~1;

#define CHECK(e,l,h) (var->e < l || var->e > h)
#if defined(CONFIG_ARCH_BCMRING) || defined(CONFIG_PLAT_BCM476X)
   if (CHECK(right_margin, (1), 256) ||	/* back porch */
       CHECK(left_margin, (1), 256) ||	/* front porch */
       CHECK(hsync_len, (1), 256) ||
#else
	if (CHECK(right_margin, (1+5), 256) ||	/* back porch */
       CHECK(left_margin, (1+5), 256) ||	/* front porch */
       CHECK(hsync_len, (1+5), 256) ||
#endif
       var->xres > 4096 ||
       var->lower_margin > 255 ||		/* back porch */
       var->upper_margin > 255 ||		/* front porch */
       var->vsync_len > 32 ||
       var->yres > 1024)
   return -EINVAL;
#undef CHECK

   /* single panel mode: PCD = max(PCD, 1) */
   /* dual panel mode: PCD = max(PCD, 5) */

   /*
    * You can't change the grayscale setting, and
    * we can only do non-interlaced video.
    */
   if (var->grayscale != fb->fb.var.grayscale ||
         (var->vmode & FB_VMODE_MASK) != FB_VMODE_NONINTERLACED)
      return -EINVAL;

#define CHECK(e) (var->e != fb->fb.var.e)
   if (fb->panel->fixedtimings &&
      (CHECK(xres) ||
       CHECK(yres) ||
       CHECK(bits_per_pixel) ||
       CHECK(pixclock) ||
       CHECK(left_margin) ||
       CHECK(right_margin) ||
       CHECK(upper_margin) ||
       CHECK(lower_margin) ||
       CHECK(hsync_len) ||
       CHECK(vsync_len) ||
       CHECK(sync)))
      return -EINVAL;
#undef CHECK

   var->nonstd = 0;
   var->accel_flags = 0;

   return 0;
}

/*
 * Public Function Prototypes
 */

/*
 * Check to see if the CLCD driver has been initialized
 */
extern int clcdfb_is_initialized(void);

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
      unsigned int bpp);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Call before starting video decoding. The caller needs to make sure the CLCD
 * driver has been been initialized and the Video FB has been set
 * (clcdfb_set_video_fb) before calling this function
 */
extern int clcdfb_video_decode_start(void);
#endif

#ifdef CONFIG_FB_ARMCLCD_VIDEO
/*
 * Call before ending video decoding. The caller needs to make sure the CLCD
 * driver has been been initialized and the Video FB has been set
 * (clcdfb_set_video_fb) before calling this function
 */
extern int clcdfb_video_decode_end(void);
#endif
