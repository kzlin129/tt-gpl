/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*
*****************************************************************************
*
*  vclcd.c
*
*  PURPOSE:
*
*   This implements the VideoCore LCD driver for video reference
*   designs. Contrary to the voice reference design, the LCD is connected to
*   VideoCore processor, instead of being connected directly to the host processor.
*   This driver maintains a virtual frame buffer, and communicates with the
*   VideoCore processor to copy the contents of the virtual frame buffer
*   to the real frame buffer (which exists on the VideoCore processor).
*
*   This driver is largely based upon /drivers/char/bcm116x/lcd.c.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/string.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/hardirq.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/dma-mapping.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/lcd_backlight.h>
#include <linux/broadcom/timer.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/knllog.h>

#include <cfg_global.h>	//ljc

#if defined( CONFIG_BCM_LCD_PERF ) && defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
#include <broadcom/linux/idle_prof.h>
#endif

#include <linux/broadcom/lcd.h>
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
#include <linux/broadcom/idle_prof.h>
#endif

#include <linux/broadcom/videocore_settings.h>

#include <vchost_config.h>
#include "vcdispman_ex.h"
#include "vcgencmd.h"
#include "linux/broadcom/vc.h"
#include "vchost.h"
#include "vchost_int.h"
#include "vcgencmd.h"
#include "vc_dispmanx.h"
#include "graphics_x.h"

#if (CFG_GLOBAL_CPU == MIPS32)
extern  timer_tick_count_t  gLastKeypadIrqTick;
extern  timer_tick_count_t  gLastKeypadReadTick;
#endif

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

// gLcdPrintStats is controlled via the /proc/sys/vc/display-stats entry
// which is controlled through the vc_drv.c file.

int   gLcdPrintStats = 0;

void lcd_backlight_enable( LCD_BACKLIGHT_LEVEL level );
void lcd_backlight_init( void );
void lcd_backlight_deinit( void );
void lcd_control(int dispnum, int power, int backlight);
int lcd_undisplay(void);


/*
** Debug logging macros.
*/

#if 1
#   define LCD_PUTS(str)           printk( "%s: %s", __FUNCTION__, str )
#   define LCD_DEBUG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args )
#else
#   define LCD_PUTS(str)
#   define LCD_DEBUG(fmt, args...)
#endif

#if 0
#   define LCD_DEBUG_REG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args )
#else
#   define LCD_DEBUG_REG(fmt, args...)
#endif

#ifdef CONFIG_BCM_KNLLOG_IRQ
    #define FB_KNLLOG( fmt, args... )   if (gKnllogIrqSchedEnable & KNLLOG_FB) KNLLOG( fmt, ## args )
#else
    #define FB_KNLLOG( fmt, args... )
#endif


/*
** VideoCore display number.
*/
#define DISPMANX_FRAMEBUF_SCREEN_DISPLAY     0
#define DISPMANX_FRAMEBUF_BITMAP_TYPE        VC_IMAGE_RGBA32


/*
** There is a double-buffered frame-buffer on the VideoCore, which are cached
** versions of the shadow frame-buffer that exists on the host. Double-buffering
** is used to avoid "tearing" effects.
*/
#define VC_NUM_FRAME_BUFS  2

#if !defined( CFG_GLOBAL_EXTRA_FRAME_BUFFERS )
#   define  CFG_GLOBAL_EXTRA_FRAME_BUFFERS  0
#endif

/*
** Controls whether dirty row tracking is done to minimize the amount of data
** which needs to be sent to the VideoCore
 */
#define VC_USE_DIRTY_ROWS  0

#define VC_LCD_NO_YIELD 0
#define VC_LCD_YIELD    1

/* Align value to the specified boundary. */
#define ALIGN_UP(value, alignment) (((value)+((alignment)-1)) & ~((alignment)-1))

#define BITS_TO_UINT32S( bits )  ( ALIGN_UP( bits, 32 ) / 32 )
#define BITS_TO_BYTES( bits )    ( BITS_TO_UINT32S( bits ) * sizeof( uint32_t ))

/* ---- Private Variables ------------------------------------------------ */

static char gBanner[] __initdata = KERN_INFO "Broadcom VC03 LCD Driver: 0.01\n";

//---------------------------------------------------------------------------
// We maintain one framebuffer on the host. This is simply referred to as
// gFrameBuffer. This is typically mmap'd into user space so that the
// user mode program can access it.

typedef struct
{
    struct semaphore    mutex;      // Lock for access the framebuffer/dirtyrows
    struct semaphore    dirty;      // Semaphore that indicates the buffer is dirty

    long                updatePid;  // PID of worker thread
    struct completion   exited;     // For when the worker thread exits

    void               *virtPtr;
    dma_addr_t          physPtr;

#if VC_USE_DIRTY_ROWS
    uint32_t           *dirtyRowBits;
#endif

    char                name[ 20 ];

} HOST_FrameBuffer_t;

//---------------------------------------------------------------------------
// We maintain two framebuffers on the VC02, and active one and an inactive one.
// We always copy the host framebuffer to the inactive framebuffer on the VC02
// and then tell the VC02 to swap.

typedef struct
{
   // Each of the bitmaps on the VC02 is really an image from the VC02's
   // perspective. The image parameter stores information about these
   // images.

   VC_IMAGE_PARAM_T  image;

   // The VC02 uses the notion of a resource as a wrapper for the image.

   uint32_t          imageHndl;

#if VC_USE_DIRTY_ROWS
   // Array of bits to keep track of which rows are dirty.

   uint32_t         *dirtyRowBits;
#endif


} VC_FB_t;

//---------------------------------------------------------------------------
// Finally, the FrameBuffer_t holds all of the information related to
// frame buffers together in a single place.

typedef struct
{
   // The ioctl_dirtyRowBits is allocated when the host frame buffer is 
   // allocated, and is intended to be used by the lcd_ioctl function, but
   // it can also be used by anybody who holds the lock.

   uint32_t            *ioctl_dirtyRowBits;

   // We maintain one framebuffer on the host. This is mmap'd into user space
   // so that the user-mode program can access it.

   HOST_FrameBuffer_t   hostFB;

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )
   // Temporary frame buffers used to reduce waiting for transfers down to 
   // the VideoCore

   HOST_FrameBuffer_t   tempFB[ CFG_GLOBAL_EXTRA_FRAME_BUFFERS ];

   spinlock_t           timerLock;
   struct timer_list    timer;

#endif

   uint32_t             hostPitch;          // bytes per line
   size_t               hostSizeInBytes;    // bytes for the entire frame buffer

   int                  hostWidth;          // Width of the Host FB in pixels
   int                  hostHeight;         // Height of the Host FB in pixels
   int                  hostBitsPerPixel;
   int                  hostBytesPerPixel;
   int                  hostFrameBufferBytes;

   // We maintain two framebuffers on the VC02, and active one and an 
   // inactive one. We always copy the host framebuffer to the inactive 
   // framebuffer on the VC02 and then tell the VC02 to swap.

   VC_FB_t              vcFrameBufferObj[ VC_NUM_FRAME_BUFS ];

   // vcFb is a pointer to the current vcFrameBufferObj which is being used
   // by the host, and vcPrevFB is a pointer to the one we used last time.

   VC_FB_t             *vcFB;
   VC_FB_t             *vcPrevFB;

   // The pitch of the vc02 frame buffer is the number of bytes/scanline and
   // must be a multiple of 16.

   uint32_t             vcPitch;

   // The vcHndl describes a VC02 data structure used to represent the 
   // framebuffer (for dispman). Each time we want to draw a new frame 
   // buffer we destroy the old framebuffer object and construct a new one 
   // which conatins just the image referred to by the imageHndl.

   DISPMANX_ELEMENT_HANDLE_T             vcHndl;

   // VideoCore layer number used for the framebuffer. Video windows should
   // use a negative layer number (which means that the GUI would obscure it)
   // and the GUI should use transparent pixels to allow the video window
   // data to show through.

   int                  vcLayerNum;

   //
   DISPMANX_DISPLAY_HANDLE_T hdisp;
   VC_IMAGE_FORMAT_T    imageType;

} FrameBuffer_t;

/* The following is a definition of a structure that maintains a thread for lcd bit map transfer
 * a thread is maintained so lcd update can be run at a slightly higher priority than
 * the console threads.  This option avoids the lcd update being suspended */
typedef struct VC_XFER_THD_t
{
   /* This update thread is used when only 1 frame buffer is maintained */
   /* when only 1 frame buffer is involved, it is important that only 1 bit map transfer 
    * can be handled at the same time to avoid tearing.  
    * 2 semaphores are being used to maintain the synchronization of the update process */
   struct semaphore    xfer_start;     // semaphore for the start of the transfer
   struct semaphore    xfer_complete;  // semaphore for the completion of the transfer

   long                threadPid;      // PID of worker thread
   struct completion   exited;         // For when the worker thread exits

   // parameters needed for bitmap transfer
   uint32_t    vcAddr;
   void       *hostAddr;
   int         x;
   int         y;
   int         width;
   int         height;
   int         vcPitch;
   int         hostPitch;
   int         yieldRequired;

} VC_XFER_THD_t;

static   FrameBuffer_t  gFB;
static   VC_XFER_THD_t  gXferThreadStruct;

static  int     gEnabled = 0;

#define  DIRTY_WORD_IDX( row )   ( (row) >> 5 )             // Divide by 32
#define  DIRTY_WORD_MASK( row )  ( 1 << ( (row) & 0x1f ))   // Module 32

#define  IS_ROW_DIRTY( bits, row )   ( (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] &   DIRTY_WORD_MASK( row )) != 0 )
#define  MARK_ROW_DIRTY( bits, row )   (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] |=  DIRTY_WORD_MASK( row ))
#define  MARK_ROW_CLEAN( bits, row )   (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] &= ~DIRTY_WORD_MASK( row ))

#if defined( CONFIG_BCM_LCD_PERF )

static  int                gLcdPerf          = 1;
static  int                gLcdPerfFreq      = 50;
static  struct timer_list  gLcdPerfTimer;

static  long               gPerfMonThreadId  = 0;
static  struct completion  gPerfMonExited;
static  struct semaphore   gPerfMonSem;

typedef struct
{
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    unsigned int    jiffies;
    unsigned int    user;
    unsigned int    nice;
    unsigned int    system;
    unsigned int    busy;
#else
    idle_handle_t   idle_handle;
    u32 idle;
    u32 total;
#endif
} LcdPerfData;

static  LcdPerfData                 gLcdPerfCurrData;
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
static  LcdPerfData                 gLcdPerfPrevData;
#endif
static  unsigned int                gLcdPerfPeakBusy = 0;

static int lcd_proc_sysctl_perf(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );


#endif  /* CONFIG_BCM_LCD_PERF */


#if (CFG_GLOBAL_CPU == MIPS32)
static  timer_tick_count_t   gLastUpdateTick = 0;
#endif

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )
static  int gVcFrameBufferUpdateDelayMsec = -1; // -1 = use synchronous updates
static  int gVcFrameBufferUpdateDelayRows = -1; // initialized to gFB.hostHeight in lcd_init
static  int gVcPriorityUpdateLcd = 3;
static  int gVcCompareDirty = 0;
#endif

// Since raising the priority of the MMI thread can't be undone, we set the 
// default to 0. If a customer really wants the mmi thread priority to be 
// elevated then they should raise it by adding a line to the /etc/rcS file.

static  int gVcPriorityMMI = 0;
//static  int gVcCopyImage = 0; GLS
static  int gVcCopyImage = 0;
static  int gVcMonitorRow = -1;
static  int gVcDispmanStats = 0;

#if !defined( CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE )
#   define  CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE  0
#endif
#if CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE
// KLUGE FIX for Qtopia!!!
// Override alpha bits in Host Frame buffer (on 32 bit displays only) with gVcAlphaBits
int gVcAlphaBitsOverrideOn = 0; 
int gVcAlphaBits = 0xFF;
#endif

#if VC_USE_DIRTY_ROWS
static  int gVcDebugDirtyRows = 1;
#endif

/* default to use the LCD update thread for bitmap transfer to VC02 */
static int gVcLcdUseUpdateThd = 0;

static  struct ctl_table_header    *gSysCtlHeader;

static struct ctl_table gSysCtlLcd[] = {
#if defined( CONFIG_BCM_LCD_PERF )
   {
      .ctl_name      = BCM_SYSCTL_LCD_PERF,
      .procname      = "perf",
      .data          = &gLcdPerf,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = BCM_SYSCTL_LCD_PERF_FREQ,
      .procname      = "perf_freq",
      .data          = &gLcdPerfFreq,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = BCM_SYSCTL_LCD_PERF_PEAKBUSY,
      .procname      = "perf_perk_busy",
      .data          = &gLcdPerfPeakBusy,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#endif
   {
      .ctl_name      = 100,
      .procname      = "display_layer",
      .data          = &gFB.vcLayerNum,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#if VC_USE_DIRTY_ROWS
   {
      .ctl_name      = 101,
      .procname      = "debug-dirtyRows",
      .data          = &gVcDebugDirtyRows,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#endif
#if CFG_GLOBAL_EXTRA_FRAME_BUFFERS
   {
      .ctl_name      = 110,
      .procname      = "update-delay",
      .data          = &gVcFrameBufferUpdateDelayMsec,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 111,
      .procname      = "update-delay-rows",
      .data          = &gVcFrameBufferUpdateDelayRows,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 112,
      .procname      = "priority-update-lcd",
      .data          = &gVcPriorityUpdateLcd,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 113,
      .procname      = "compare-dirty",
      .data          = &gVcCompareDirty,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#endif
   {
      .ctl_name      = 140,
      .procname      = "priority-mmi",
      .data          = &gVcPriorityMMI,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 141,
      .procname      = "vc02-copy",
      .data          = &gVcCopyImage,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 142,
      .procname      = "monitor-row",
      .data          = &gVcMonitorRow,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 143,
      .procname      = "dispman-stats",
      .data          = &gVcDispmanStats,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#if CFG_GLOBAL_FRAME_BUFFER_ALPHA_OVERRIDE
   {
      .ctl_name      = 144,
      .procname      = "alpha-override-on",
      .data          = &gVcAlphaBitsOverrideOn,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 145,
      .procname      = "alpha-bits",
      .data          = &gVcAlphaBits,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#endif
   {
      .ctl_name      = 146,
      .procname      = "use-update-thread",
      .data          = &gVcLcdUseUpdateThd,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};


static struct ctl_table gSysCtl[] = {
   {
      .ctl_name = CTL_BCM_LCD,
      .procname = "lcd",
      .mode     = 0555,
      .child    = gSysCtlLcd
   },
   {}
};


/* ---- Private Function Prototypes -------------------------------------- */

void lcd_colorbars(int num);

#if defined( CONFIG_BCM_LCD_PERF )
static void     lcd_draw_perf( void );
static void     lcd_clear_perf( void );
static void     lcd_perf_timer( unsigned long dummy );
#endif

static void     lcd_exit( void );
static int      lcd_init( void );
static void     lcd_pwr_off_controller( void );
static int      lcd_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static int      lcd_mmap( struct file *file, struct vm_area_struct * vma );
static int      lcd_open( struct inode *inode, struct file *file );

static void     lcd_fb_lock_init( HOST_FrameBuffer_t *fb );
static int      lcd_fb_lock_acquire( HOST_FrameBuffer_t *fb );
static void     lcd_fb_lock_release( HOST_FrameBuffer_t *fb );
static int      lcd_fb_is_locked( HOST_FrameBuffer_t *fb );

#if VC_USE_DIRTY_ROWS
static uint32_t   *lcd_alloc_dirtyRowBits( int numRows );
#endif
static void        lcd_dirty_row_bits( LCD_DirtyRowBits_t *dirtyRowBits );

int lcd_dispman_update_framebuffer( void );
static int lcd_dispman_init( void );
static int lcd_dispman_deinit( void );

int lcd_dispman_write_bitmap
(
   uint32_t    vcAddr,
   void       *hostAddr,
   int         x,
   int         y,
   int         width,
   int         height,
   int         vcPitch,
   int         hostPitch,
   int         yieldRequired
);

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations lcd_fops =
{
    owner:      THIS_MODULE,
    ioctl:      lcd_ioctl,
    mmap:       lcd_mmap,
    open:       lcd_open,
};

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  Calculates an rgb value from individual components.
*
*  This function assumes that R G & B values are specified in the range
*  0-255 and will scale appropriately for non 8-bit pixels.
*
***************************************************************************/

static unsigned rgb_val( unsigned red, unsigned green, unsigned blue )
{
   if ( gFB.hostBitsPerPixel == 16 )
   {
      // 16 bpp implies 565 format

      return (( red & 0xf8 ) << 8 ) | (( green & 0xfc ) << 3 ) | (( blue & 0xf8 ) >> 3 );
   }

   // Otherwise assume 32 bpp, which is ARGB (0xff in the transparency means
   // 100% opaque)

   return 0xff000000 | (( red & 0xff ) << 16 ) | (( green & 0xff ) << 8 ) | ( blue & 0xff );

} // rgb_val

/****************************************************************************
*
*   Calculates the location in the frame buffer for a given xy position
*
***************************************************************************/

static void *xyPtr( unsigned x, unsigned y )
{
   unsigned char *fbPtr = gFB.hostFB.virtPtr;

   return fbPtr + ((( y * gFB.hostWidth ) + x ) * gFB.hostBytesPerPixel );

} // xyPtr

/****************************************************************************
*
*   Calculates the location in the frame buffer for a given xy position
*
***************************************************************************/

static void *setPixels( unsigned x, unsigned y, unsigned numPixels, unsigned color )
{
  if ( gFB.hostBitsPerPixel == 16 )
	{
      u16 *fbPtr = gFB.hostFB.virtPtr;
	  
      fbPtr += (( y * gFB.hostWidth ) + x );
	  
      while ( numPixels-- > 0 )
		{
		  *fbPtr++ = color;
		}
	}
  else if (gFB.hostBitsPerPixel == 24)
	{
      uint8_t* ptr = (uint8_t*)gFB.hostFB.virtPtr;
      uint8_t* colptr = (uint8_t*)&color;
	  
      ptr += (( y * gFB.hostPitch ) + x * 3 );
	  
      while ( numPixels-- > 0 )
		{
		  ptr[0]= colptr[0];
		  ptr[1]= colptr[1];
		  ptr[2]= colptr[2];
		  ptr += 3;
		}
	}

  else if (gFB.hostBitsPerPixel == 32)
	{
      uint8_t* ptr = (uint8_t*)gFB.hostFB.virtPtr;
      uint8_t* colptr = (uint8_t*)&color;
	  
      ptr += (( y * gFB.hostPitch ) + x * 4 );
	  
      while ( numPixels-- > 0 )
		{
		  ptr[0]= colptr[0];
		  ptr[1]= colptr[1];
		  ptr[2]= colptr[2];
		  ptr[3]= colptr[3];
		  ptr += 4;
		}
	}

  return 0;
  
} // setPixels

/****************************************************************************
*
*  lcd_color_test
*
*   Sends some predefined pattern to the LCD
*
***************************************************************************/
void lcd_colorbars(int num)
{
  unsigned int    y;

#define R   rgb_val( 8, 0, 0 )  // 256 / 32 = 8
#define G   rgb_val( 0, 8, 0 )
#define B   rgb_val( 0, 0, 8 )
		  
#define RIGHT_BORDER_WIDTH   8
#define NUM_INCREMENTS       32 // RGB is 565 format, 5^2 = 32
		  
  unsigned pattern[ 7 ];
  unsigned *p;
  
  int patternIdx;
  int numPatterns = sizeof( pattern ) / sizeof( pattern[ 0 ] );
  int numRowsPerPattern = gFB.hostHeight / numPatterns;
  int incrWidth = gFB.hostWidth / NUM_INCREMENTS;
  int border = ( gFB.hostWidth - ( incrWidth * NUM_INCREMENTS )) / 2;
  LCD_DirtyRows_t dirtyRows;
  
  LCD_DEBUG( "lcd_colorbars num=%d\n", num );
  
  pattern[ 0 ] = R;
  pattern[ 1 ] = G;
  pattern[ 2 ] = B;
  pattern[ 3 ] = R | G;
  pattern[ 4 ] = R | B;
  pattern[ 5 ] = G | B;
  pattern[ 6 ] = R | G | B;
  
  
  // Start off by clearing the whole frame buffer to black
  
  memset( gFB.hostFB.virtPtr, 0, gFB.hostSizeInBytes );
  dirtyRows.top = 0;
  dirtyRows.bottom = gFB.hostHeight - 1;
  

  if(num == 0)
  	{
  	 lcd_undisplay();
	 return;
  	}
  y = 0;
  for ( patternIdx = 0; patternIdx < numPatterns; patternIdx++ )
	{
	  unsigned int r;
	  p = pattern +  (patternIdx + num) % 7;
	  
	  for ( r = 0; r < numRowsPerPattern; r++, y++ )
		{
		  unsigned x = border; 
		  unsigned color = (*p) * NUM_INCREMENTS;
		  int      col1;
		  
		  for ( col1 = 0; col1 < NUM_INCREMENTS; col1++ )
			{
			  setPixels( x, y, incrWidth, color );
			  
			  x += incrWidth;
			  color -= *p;
			}
		}
	}


  lcd_dirty_rows( &dirtyRows );

}  
  
// static
void lcd_color_test(int num)
{
  unsigned int    x;
  unsigned int    y;
  LCD_DirtyRows_t dirtyRows;
  
  LCD_DEBUG( "%d\n", num );
  
  dirtyRows.top = 0;
  dirtyRows.bottom = gFB.hostHeight - 1;
  
  switch ( num )
	{
      case 0: // Clear to black
		{
		  if ( gFB.hostFB.virtPtr != NULL )
			{
			  memset( gFB.hostFB.virtPtr, 0, gFB.hostSizeInBytes );
			}
		  break;
		}
		
      case 1: // Color Wedges
		{
		  lcd_colorbars(0);
		  break;
		}
		
      case 3: // Bit Test
		{
		  
		  return;
		}
		
      case 4: // Dump Some memory
		{
		  printk( "fbPtr = 0x%08lx\n", (long)xyPtr( 0, 0 ));
		  
		  for ( y = 0; y < 4; y++ )
			{
			  printk( "Line: %d ", y );
			  
			  if ( gFB.hostBitsPerPixel == 16 )
				{
				  u16   *fbPtr = xyPtr( 0, y );
				  
				  for ( x = 0; x < 14; x++ )
					{
					  printk( "%04x ", fbPtr[x] );
					}
				}
			  else
				{
				  u32   *fbPtr = xyPtr( 0, y );
				  
				  for ( x = 0; x < 7; x++ )
					{
					  printk( "%08x ", fbPtr[x] );
					}
				}
			  printk( "\n" );
			}
		  break;
		}

		
	}
  
  lcd_dirty_rows( &dirtyRows );
  
} // lcd_color_test

/****************************************************************************
*
*  Allocates enough uint32_t's to contain numRows + 1 bits. The +1 is so 
*  that an imaginary line beyond the end of the buffer can be marked clean.
*
*  This simplifies some of the algorithims.
*
***************************************************************************/

#if VC_USE_DIRTY_ROWS
static uint32_t *lcd_alloc_dirtyRowBits( int numRows )
{
   return kcalloc( BITS_TO_UINT32S( numRows + 1 ), sizeof( uint32_t ), GFP_KERNEL );

} // lcd_alloc_dirtyRowBits
#endif

/****************************************************************************
*
*  Dumps out (via printk) regions of bits from a dirtyRowBits bitmask.
*
***************************************************************************/

void vc_dump_dirtyRowBits( const uint32_t *dirtyRowBits, unsigned numRows );

void vc_dump_dirtyRowBits( const uint32_t *dirtyRowBits, unsigned numRows )
{
    int     row;
    int     dirty               = 0;
    int     firstDirtyRow       = 0;
    int     somethingPrinted    = 0;

    // Note: This routine may be called in situations where we can't take
    //       take the shortcut of marking the row indicated by 'numRows' as 
    //       clean, so we do extra check after the for loop.

    for ( row = 0; row < numRows; row++ ) 
    {
        if ( IS_ROW_DIRTY( dirtyRowBits, row ))
        {
            if ( !dirty )
            {
                // We've hit the first dirty row after some clean ones.

                firstDirtyRow = row;
                dirty = 1;
            }
        }
        else
        {
            if ( dirty )
            {
                // We've hit a clean row after a section of dirty rows.

                if ( somethingPrinted )
                {
                    printk( ", " );
                }

                printk( "%d-%d", firstDirtyRow, row - 1 );
                somethingPrinted = 1;
                dirty = 0;
            }
        }
    }

    if ( dirty )
    {
        // There is a dirty region which includes the last row

        if ( somethingPrinted )
        {
            printk( ", " );
        }

        printk( "%d-%d", firstDirtyRow, row - 1 );
        somethingPrinted = 1;
        dirty = 0;
    }

    if ( somethingPrinted )
    {
        printk( "\n" );
    }
    else
    {
        printk( "No dirty rows\n" );
    }

} // vc_dump_dirtyRowBits


/****************************************************************************
*
*  Initializes the lock for using a FrameBuffer_t structure.
*
***************************************************************************/

static void lcd_fb_lock_init( HOST_FrameBuffer_t *fb )
{
   init_MUTEX( &fb->mutex );

} // lcd_fb_lock_init

/****************************************************************************
*
*  Acquires the lock for using the gFB data structure. This function uses
*  an interruptible wait, so the caller should check the return code
*  to verify that the acquisition was actually successful.
*
***************************************************************************/

static int lcd_fb_lock_acquire( HOST_FrameBuffer_t *fb )
{
   return down_interruptible( &fb->mutex );
   
} // lcd_fb_lock_acquire

/****************************************************************************
*
*  Releases the lock associated with 
*  an interruptible wait, so the caller should check the return code
*  to verify that the acquisition was actually successful.
*
***************************************************************************/

static void lcd_fb_lock_release( HOST_FrameBuffer_t *fb )
{
   BUG_ON( !lcd_fb_is_locked( fb ));

   up( &fb->mutex );

} // lcd_fb_lock_release

/****************************************************************************
*
*  Returns non-zero (TRUE) if the mutex is currently held, zero otherwise.
*
***************************************************************************/

//static
int lcd_fb_is_locked( HOST_FrameBuffer_t *fb )
{
   // The mutex is really just a semaphore initialized with a count of 1.
   // If the count is 1 then the semaphore is available to be taken.
   // If the count is 0 then the semaphore is taken.
   // If the count is negative, then somebody is waiting.

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)

   unsigned long flags;
   unsigned int count;

   spin_lock_irqsave(&fb->mutex.lock, flags);
   count = fb->mutex.count;
   spin_unlock_irqrestore(&fb->mutex.lock, flags);
   return (count <= 0);

#else

   return atomic_read( &fb->mutex.count ) <= 0;

#endif
      
} // lcd_fb_is_locked

/****************************************************************************
*
*  lcd_dirty_rows
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

void lcd_dirty_rows( LCD_DirtyRows_t *dirtyRows )
{
  int                 row;
  LCD_DirtyRowBits_t  dirtyRowBits;
  
  // We can only be run from thread context.
  
  BUG_ON( in_irq() );
  BUG_ON( in_softirq() );
  
  if ( !vc_is_initialized() )
    {
	  LCD_DEBUG( "VC not initialized\n" );
	  return;
    }
  
  if (( dirtyRows->top > dirtyRows->bottom )
	  ||  ( dirtyRows->bottom >= gFB.hostHeight ))
    {
	  LCD_DEBUG( "invalid params - ignoring\n" );
	  LCD_DEBUG( "top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom );
	  
	  return;
    }
  
  if ( lcd_fb_lock_acquire( &gFB.hostFB ) != 0 )
    {
	  LCD_DEBUG( "Unable to acquire FB lock\n" );
	  return;
    }
  
#if VC_USE_DIRTY_ROWS
  dirtyRowBits.numRows = gFB.hostHeight;
  dirtyRowBits.bits    = gFB.ioctl_dirtyRowBits;

  if ( dirtyRowBits.numRows > gFB.hostHeight )
    {
	  dirtyRowBits.numRows = gFB.hostHeight;
    }
  memset( dirtyRowBits.bits, 0, BITS_TO_BYTES( dirtyRowBits.numRows ));
  
  for ( row = dirtyRows->top; row <= dirtyRows->bottom; row++ )
    {
	  MARK_ROW_DIRTY( dirtyRowBits.bits, row );
    }
#else
  (void)row;
#endif
  
  lcd_dirty_row_bits( &dirtyRowBits );
  lcd_fb_lock_release( &gFB.hostFB );

} // lcd_dirty_rows

/****************************************************************************
*
*  lcd_dirty_row_bits
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

void lcd_dirty_row_bits( LCD_DirtyRowBits_t *dirtyRowBits )
{
   int   i;
   int   numWords;

   BUG_ON( !lcd_fb_is_locked( &gFB.hostFB ));

#if VC_USE_DIRTY_ROWS

   if ( gVcDebugDirtyRows )
   {
      printk( "lcd_dirty_row_bits\n" );
      vc_dump_dirtyRowBits( dirtyRowBits->bits, dirtyRowBits->numRows );
   }

   if ( gVcMonitorRow >= 0 )
   {

       if (( gVcMonitorRow < dirtyRowBits->numRows ) && IS_ROW_DIRTY( dirtyRowBits->bits, gVcMonitorRow ))
       {
           int        col;
           uint32_t  *pixPtr = (uint32_t *)( (uint8_t *)gFB.hostFB.virtPtr + ( gVcMonitorRow * gFB.hostPitch ));

           printk( "Row %3d:", gVcMonitorRow );

           for ( col = 0; col < gFB.hostWidth; col += 30, pixPtr += 30 ) 
           {
               printk( " %08x", *pixPtr );
           }
           printk( "\n" );
       }
   }

   // Mark all of the indicated rows as dirty, and then update the videocore.

   if ( dirtyRowBits->numRows > gFB.hostHeight )
   {
      dirtyRowBits->numRows = gFB.hostHeight;
   }
   numWords = BITS_TO_UINT32S( dirtyRowBits->numRows );

   for ( i = 0; i < numWords; i++ ) 
   {
      gFB.hostFB.dirtyRowBits[ i ] |= dirtyRowBits->bits[ i ];
   }

   lcd_dispman_update_framebuffer();
   
#else // #ifdef  VC_USE_DIRTY_ROWS

   (void)dirtyRowBits;
   (void)i;
   (void)numWords;

   lcd_dispman_update_framebuffer();

#endif
   

} // lcd_dirty_row_bits

/****************************************************************************
*
*   Allocates a frame buffer
*
****************************************************************************/

static void *lcd_alloc_framebuffer( size_t numBytes, dma_addr_t *physAddrPtr )
{
    void    *virtAddr;

#if defined( CONFIG_ARM )
    virtAddr = dma_alloc_writecombine( NULL, numBytes, physAddrPtr, GFP_KERNEL );
#elif defined( CONFIG_MIPS )
    virtAddr = dma_alloc_coherent( NULL, numBytes, physAddrPtr, GFP_KERNEL );

    // Convert to cached ptr

    virtAddr = (void *)( (unsigned long)virtAddr & ~0x20000000uL );
#else
   #error "Unknown architecture type!"
#endif

    printk( "%s: framebuffer DMA: virtAddr = 0x%08lx physAddr = 0x%08x numBytes=%d\n",
            __FUNCTION__, (unsigned long)virtAddr, *physAddrPtr, numBytes );

    return virtAddr;

} // lcd_alloc_framebuffer

/****************************************************************************
*
*  lcd_perfmon_thread
*
*   Worker thread to mark the bottom two lines as dirty. A thread is 
*   required so that the frame buffer lock can be acquired.
*
****************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

static int lcd_perfmon_thread( void *data )
{
    // This thread doesn't need any user-level access, so get rid of 
    // all our resources

    daemonize( "lcdPerfMon" );

    while ( down_interruptible( &gPerfMonSem ) == 0 )
    {
       LCD_DirtyRows_t  dirtyRows;

       dirtyRows.top = gFB.hostHeight - 2;
       dirtyRows.bottom = gFB.hostHeight - 1;

       lcd_dirty_rows( &dirtyRows );
    }

    complete_and_exit( &gPerfMonExited, 0 );

} // lcd_perfmon_thread

#endif

/****************************************************************************
*
*  lcd_draw_perf
*
*   Draws the CPU performance data at the bottom of the LCD
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

// static
void lcd_draw_perf( void )
{
    unsigned int    total;
    unsigned int    accum;
    unsigned int    pos;
    unsigned int    end;
    void           *fbPtr;

    // Put a black line in for separation

    fbPtr = gFB.hostFB.virtPtr;
    fbPtr += gFB.hostWidth * ( gFB.hostHeight - 2 );
    for ( pos = 0; pos < gFB.hostWidth; pos++ )
	  {
        *fbPtr++ = 0;
	  }
	
    // Now that we have the data, figure out the delta, and draw a line
    // along the bottom

#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    total = gLcdPerfCurrData.jiffies - gLcdPerfPrevData.jiffies;
    accum = gLcdPerfCurrData.system  - gLcdPerfPrevData.system;
#else
    total = gLcdPerfCurrData.total;
    accum = total - gLcdPerfCurrData.idle;
#endif

    // Draw a black separation line

    setPixels( 0, gFB.hostHeight - 2, gFB.hostWidth, 0 );

    pos = 0;
    end = accum * gFB.hostWidth / total;

    setPixels( pos, gFB.hostHeight - 1, end - pos, rgb_val( 255, 0, 0 ));  // Red
    pos += end;

#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    accum += ( gLcdPerfCurrData.user - gLcdPerfPrevData.user );
    end = accum * gFB.hostWidth / total;

    setPixels( pos, gFB.hostHeight - 1, end - pos, rgb_val( 0, 355, 0 ));  // Green
    pos = end;

    accum += ( gLcdPerfCurrData.nice - gLcdPerfPrevData.nice );
    end = accum * gFB.hostWidth / total;

    setPixels( pos, gFB.hostHeight - 1, end - pos, rgb_val( 255, 255, 128 )); // Yellow (100% R, 100% G, 50% B)
    pos = end;
#endif

    if ( accum > gLcdPerfPeakBusy )
    {
        gLcdPerfPeakBusy = accum;
    }
    end = gLcdPerfPeakBusy * gFB.hostWidth / total;

    setPixels( pos, gFB.hostHeight - 1, end - pos, rgb_val( 0, 0, 255 ));   // Blue
    pos = end;

    end = gFB.hostWidth;

    setPixels( pos, gFB.hostHeight - 1, end - pos, rgb_val( 64, 64, 64 ));   // Dark Gray (25% R, 25% G, 25% B)

    up( &gPerfMonSem );

} /* lcd_draw_perf */

#endif /* CONFIG_BCM_LCD_PERF */

/****************************************************************************
*
*  lcd_clear_perf
*
*   Clears the CPU performance data at the bottom of the LCD
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

// static
void lcd_clear_perf( void )
{
    LCD_DirtyRows_t dirtyRows;

    // Put 2 black lines to clear performance meter

    setPixels( 0, gFB.hostHeight - 2, gFB.hostWidth, 0 );
    setPixels( 0, gFB.hostHeight - 1, gFB.hostWidth, 0 );

    up( &gPerfMonSem );

} /* lcd_clear_perf */

#endif

/****************************************************************************
*
*  lcd_fb_timer_callback
*
***************************************************************************/

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )

// static
void lcd_fb_timer_callback( unsigned long arg )
{
    HOST_FrameBuffer_t  *fb = (HOST_FrameBuffer_t *)arg;

    up( &fb->dirty );

} // lcd_fb_timer_callback

#endif

/****************************************************************************
*
*  lcd_fb_copy_thread
*
***************************************************************************/

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )

// static
int lcd_fb_copy_thread( void *data )
{
    int bufNum = (int)data;

    char    threadName[ 20 ];

    snprintf( threadName, sizeof( threadName ), "lcdUpdate-%d", bufNum );

    daemonize( threadName );

    printk( "%s: Starting...\n", threadName );

    while ( 1 )
    {
        if ( down_interruptible( &gFB.tempFB[ bufNum ].dirty ) != 0 )
        {
            printk( KERN_ERR "%s: down failed\n", threadName );
            break;
        }

        FB_KNLLOG( "%s: Working...\n", threadName );

        if ( gVcPriorityUpdateLcd != 0 )
        {
	  //            vc_adjust_thread_priority( &gVcPriorityUpdateLcd );
        }

        if ( lcd_fb_lock_acquire( &gFB.tempFB[ bufNum ]) != 0 )
        {
            printk( KERN_ERR "%s: Unable to acquire fb lock %d\n", threadName, bufNum );
            break;
        }

        if ( bufNum == ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS - 1 ))
        {
            // We're the last thread in the chain. Copy the data out to the
            // VideoCore

            FB_KNLLOG( "%s: copy fb to vc02\n", threadName );
            lcd_dispman_copy_fb_to_videocore( &gFB.tempFB[ bufNum ], VC_LCD_NO_YIELD );
            FB_KNLLOG( "%s: copy fb done\n", threadName );
        }
        else
        {
            // Copy the data to the next frame buffer in the list.

            FB_KNLLOG( "%s: copy fb to fb\n", threadName );
            lcd_copy_fb_to_fb( &gFB.tempFB[ bufNum ], &gFB.tempFB[ bufNum + 1 ] );
            FB_KNLLOG( "%s: copy fb done\n", threadName );

            // And wake up the worker thread for that frame buffer

            up( &gFB.tempFB[ bufNum + 1 ].dirty );
        }

        lcd_fb_lock_release( &gFB.tempFB[ bufNum ]);

        FB_KNLLOG( "%s: Done\n", threadName );
    }

    printk( "%s: Exiting\n", threadName );
    complete_and_exit( &gFB.tempFB[ bufNum ].exited, 0 );

} // lcd_fb_copy_thread

#endif

#if 0   // not called by anybody
static int lcd_dispmanxfer_thread( void *unused )
{
   struct sched_param sparm;
   int rc;

   daemonize( "lcd_disp_xfer" );

   /* set real time priority, note that the priority is just slightly higher than the default (0) */
   sparm.sched_priority = 2;
   if( (rc = sched_setscheduler( current, SCHED_FIFO, &sparm )) < 0 )
	 {
	   printk( KERN_ERR "failed to set real time priority for lcd transfer thread\n");
	 }
   while( 1 )
	 {
	   if( down_interruptible( &gXferThreadStruct.xfer_start ) == 0 )
		 {
		   lcd_dispman_write_bitmap(gXferThreadStruct.vcAddr,
					    gXferThreadStruct.hostAddr,
					    gXferThreadStruct.x,
					    gXferThreadStruct.y,
					    gXferThreadStruct.width,
					    gXferThreadStruct.height,
					    gXferThreadStruct.vcPitch,
					    gXferThreadStruct.hostPitch,
					    gXferThreadStruct.yieldRequired );
		 }
	   
	   up( &gXferThreadStruct.xfer_complete );
	 }
   
   complete_and_exit( &gXferThreadStruct.exited, 0 );
   
   return 0;
}
#endif

/****************************************************************************
*
*  lcd_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

// static
void __exit lcd_exit( void )
{
    LCD_PUTS( "called\n" );

    lcd_dispman_deinit();
    lcd_backlight_deinit();

#if defined( CONFIG_BCM_LCD_PERF )

    del_timer( &gLcdPerfTimer );

    if ( gPerfMonThreadId > 0 )
    {
       kill_proc( gPerfMonThreadId, SIGTERM, 1 );
       wait_for_completion( &gPerfMonExited );
    }

#endif

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )
    {
        int i;

        for ( i = 0; i < CFG_GLOBAL_EXTRA_FRAME_BUFFERS; i++ ) 
        {
            if ( gFB.tempFB[ i ].updatePid > 0 )
            {
                kill_proc( gFB.tempFB[ i ].updatePid, SIGTERM, 1 );
                wait_for_completion( &gFB.tempFB[ i ].exited );
            }
        }
    }
#endif

    if( gXferThreadStruct.threadPid > 0 )
    {
       kill_proc_info( SIGTERM, SEND_SIG_PRIV, gXferThreadStruct.threadPid );
       wait_for_completion( &gXferThreadStruct.exited );
    }

    if ( gSysCtlHeader != NULL )
    {
        unregister_sysctl_table( gSysCtlHeader );
    }

} // lcd_exit

/****************************************************************************
*
*  lcd_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int lcd_init_2( void )
{
    if ( lcd_dispman_init() != 0 )
    {
       printk( KERN_ERR "lcd_init: lcd_dispman_init failed\n" );
       return -ENOMEM;
    }

    /* The height of the allocated frame buffer must be a multiple of 16.
    ** The image is fetched from the host in 'stripes'. (A stripe is generally
    ** 16 rows high and the width of the image). The VideoCore display manager
    ** will ignore the any rows of the last stripe that are off the bottom of
    ** the actual display. */

    lcd_fb_lock_init( &gFB.hostFB );
    snprintf( gFB.hostFB.name, sizeof( gFB.hostFB.name ), "hostFB" );

#if VC_USE_DIRTY_ROWS
    if (( gFB.ioctl_dirtyRowBits = lcd_alloc_dirtyRowBits( gFB.hostHeight )) == NULL )
	  {
		return -ENOMEM;
    }
	
    if (( gFB.hostFB.dirtyRowBits = lcd_alloc_dirtyRowBits( gFB.hostHeight )) == NULL )
      {
        return -ENOMEM;
	  }
#endif //#if VC_USE_DIRTY_ROWS

//    lcd_control( DISPMANX_FRAMEBUF_SCREEN_DISPLAY, 2, 2); // lcd_control(dispnum, power, backlight)
//lcd_colorbars(2);


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
    gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
    if ( gSysCtlHeader != NULL )
    {
        gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
    }
#else
    gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif

#if defined( CONFIG_BCM_LCD_PERF )

    init_timer( &gLcdPerfTimer );
    gLcdPerfTimer.function = lcd_perf_timer;
    if (gLcdPerf)
    {
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
        arch_init_idle_profile(&gLcdPerfCurrData.idle_handle);
#endif
        gLcdPerfTimer.expires = jiffies + gLcdPerfFreq;
        add_timer( &gLcdPerfTimer );
    }

#endif /* CONFIG_BCM_LCD_PERF */

#if defined( CONFIG_BCM_LCD_PERF )
    sema_init( &gPerfMonSem, 0 );
    init_completion( &gPerfMonExited );
    gPerfMonThreadId = kernel_thread( lcd_perfmon_thread, 0, 0 );
#endif

    lcd_backlight_init();

    return 0;
}

// static
int __init lcd_init( void )
{
    int rc;
    printk( gBanner );

    // Register our device with Linux
    if (( rc = register_chrdev( BCM_LCD_MAJOR, "lcd", &lcd_fops )) < 0 )
    {
        printk( KERN_WARNING "lcd: register_chrdev failed for major %d\n", BCM_LCD_MAJOR );
        return rc;
    }

    return 0;

} // lcd_init


// static
void lcd_control(int dispnum, int power, int backlight)
{
#if 0
    VC_GenCmd_t      gencmd;

    /*
    ** Send "generic" command type to VideoCore to power on the LCD controller.
    **
    ** (Parameter value of 2 means to turn on the LCD, 1 means to turn off
    ** the LCD controller but still allow register access, and 0 means
    ** to turn off the controller).
    */
    sprintf( &gencmd.cmd[0], "display_control %d power=%d backlight=%d", dispnum, power, backlight);
    vc_gencmd( gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );
#endif
#if defined(HW_LCD_BACKLIGHT_PIN)
	//turn on backlight from arm side
	if (backlight>1) backlight=1;
    vchost_gpio_pintype_set(HW_LCD_BACKLIGHT_PIN, GPIO_PIN_TYPE_OUTPUT, GPIO_NO_INTERRUPT);
	vchost_gpio_pinval_set(HW_LCD_BACKLIGHT_PIN, backlight );
	printk (KERN_INFO "set HW_LCD_BACKLIGHT_PIN %d to %d\n",HW_LCD_BACKLIGHT_PIN,backlight);
#endif	
	//#endif

} //


/****************************************************************************
*
*  lcd_pwr_off_controller
*
*   Power off the LCD controller.
*
***************************************************************************/

// static
void lcd_pwr_off_controller( void )
{
#if 0
    VC_GenCmd_t gencmd;

    /*
    ** Send "generic" command type to VideoCore to power off the LCD controller.
    **
    ** (Parameter value of 2 means to turn on the LCD, 1 means to turn off
    ** the LCD controller but still allow register access, and 0 means
    ** to turn off the controller).
    */
    sprintf( &gencmd.cmd[0], "power_control lcd0 0" );
    vc_gencmd( gencmd.response.str, sizeof( gencmd.response.str ), "%s", &gencmd.cmd[0] );
#endif
    gEnabled = 0;

} // lcd_pwr_off_controller

/****************************************************************************
*
*  lcd_ioctl
*
***************************************************************************/

// static
int lcd_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    int err;

    // LCD_DEBUG( "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    if (( _IOC_TYPE( cmd ) != LCD_MAGIC )
    ||  ( _IOC_NR( cmd ) < LCD_CMD_FIRST )
    ||  ( _IOC_NR( cmd ) > LCD_CMD_LAST ))
    {
        return -ENOTTY;
    }

    // Note that Read/Write is from the perspective of userland. access_ok
    // is from the perspective of kernelland.

    err = 0;
    if (( _IOC_DIR( cmd ) & _IOC_READ ) != 0 )
    {
        err |= !access_ok( VERIFY_WRITE, (void *)arg, _IOC_SIZE( cmd ));
    }
    if (( _IOC_DIR( cmd ) & _IOC_WRITE ) != 0 )
    {
        err |= !access_ok( VERIFY_READ, (void *)arg, _IOC_SIZE( cmd ));
    }
    if ( err )
    {
        LCD_PUTS( "arg pointer is invalid\n" );
        return -EFAULT;
    }

    switch ( cmd )
	  {
		
        case LCD_IOCTL_ENABLE_BACKLIGHT:
		  {
            lcd_backlight_enable( (int)arg );
            break;
		  }
		  
		  
        case LCD_IOCTL_INIT:
		  {
			lcd_control(0, 2, 1); // lcd_control(dispnum, power, backlight)
            break;
		  }
		  

        case LCD_IOCTL_INIT_ALL:
		  {
			lcd_control(0, 2, 1); // lcd_control(dispnum, power, backlight)
            break;
		  }
		  

        case LCD_IOCTL_COLOR_TEST:
		  {
            lcd_color_test( (int)arg );
            break;
		  }
		  
		case LCD_IOCTL_DIRTY_ROWS:
		  {
			LCD_DirtyRows_t dirtyRows;
			
			if ( copy_from_user( &dirtyRows, (LCD_DirtyRows_t *)arg, sizeof( dirtyRows )) != 0 )
			  {
				err = -EFAULT;
                break;
			  }
			
            if ( dirtyRows.bottom > gFB.hostHeight )
			  {
				dirtyRows.bottom = gFB.hostHeight;
			  }
#if defined( CONFIG_BCM_LCD_PERF )
            else
			  if ( gLcdPerf )
				{
				  if ( dirtyRows.bottom > gFB.hostHeight - 3 )
					{
					  dirtyRows.bottom = gFB.hostHeight - 3;
					}
				}
#endif
            lcd_dirty_rows(&dirtyRows);
            break;
		  }
		  
        case LCD_IOCTL_DIRTY_ROW_BITS:
		  {
			err = lcd_ioctl_dirty_row_bits( arg );
			break;
		  }
		  
        case LCD_IOCTL_PWR_OFF:
		  {
            lcd_pwr_off_controller();
            break;
		  }
		  
        case LCD_IOCTL_INFO:
		  {
            LCD_Info_t lcdInfo;
			
            lcd_get_info( &lcdInfo );
            err = copy_to_user( (void *)arg, &lcdInfo, sizeof( LCD_Info_t ));
			
            break;
		  }
		  
        case LCD_IOCTL_SET_DISPLAY_LAYER_NUM:
		  {
            gFB.vcLayerNum = (int) arg;
            break;
		  }
		  
		  /*
		  ** These commands are not supported by this LCD driver.
		  */
        case LCD_IOCTL_ENABLE_CS:
        case LCD_IOCTL_ENABLE_SUB_BACKLIGHT:
        case LCD_IOCTL_HOLD:
        case LCD_IOCTL_PRINT_DATA:
        case LCD_IOCTL_PRINT_REGS:
        case LCD_IOCTL_PULSE:
        case LCD_IOCTL_REG:
        case LCD_IOCTL_RESET:
        case LCD_IOCTL_SCOPE_TIMEOUT:
        case LCD_IOCTL_SETUP:
        case LCD_IOCTL_RECT:
		  {
			printk( "Unsupported ioctl: '0x%x'\n", cmd );
			break;
		  }
		  
        default:
		  {
            LCD_DEBUG( "Unrecognized ioctl: '0x%x'\n", cmd );
            err = -ENOTTY;
		  }
	  }
	
    return err;
	
} /* lcd_ioctl */

/****************************************************************************
*
*  Helper function for dealing with the LCD_IOCTL_DIRTY_ROW_BITS ioctl.
*
*  This function is used both from lcdfb.c and this file.
*
***************************************************************************/

int lcd_ioctl_dirty_row_bits( unsigned long arg )
{
   int                err = 0;
   LCD_DirtyRowBits_t dirtyRowBits;

   if ( copy_from_user( &dirtyRowBits, (LCD_DirtyRowBits_t *)arg, sizeof( dirtyRowBits )) != 0 )
   {
      return -EFAULT;
   }
   if ( dirtyRowBits.numRows == 0 )
   {
      return 0;
   }
   if ( dirtyRowBits.numRows > gFB.hostHeight )
   {
       dirtyRowBits.numRows = gFB.hostHeight;
   }

   if ( lcd_fb_lock_acquire( &gFB.hostFB ) != 0 )
   {
      return -ERESTARTSYS;
   }

   if ( copy_from_user( gFB.ioctl_dirtyRowBits, dirtyRowBits.bits, BITS_TO_BYTES( dirtyRowBits.numRows )) != 0 )
   {
      err = -EFAULT;
   }
   else
   {
      dirtyRowBits.bits = gFB.ioctl_dirtyRowBits;

#if defined( CONFIG_BCM_LCD_PERF )
      if ( gLcdPerf )
      {
          MARK_ROW_CLEAN( dirtyRowBits.bits, gFB.hostHeight - 1 );
          MARK_ROW_CLEAN( dirtyRowBits.bits, gFB.hostHeight - 2 );
      }
#endif

      if ( gVcPriorityMMI != 0 )
      {
	//          vc_adjust_thread_priority( &gVcPriorityMMI );
      }

      lcd_dirty_row_bits( &dirtyRowBits );
   }
   lcd_fb_lock_release( &gFB.hostFB );

   return err;

} // lcd_ioctl_dirty_row_bits

/****************************************************************************
*
*  lcd_get_info
*
***************************************************************************/

//static
void lcd_get_info( LCD_Info_t *lcdInfo )
{
    lcdInfo->bitsPerPixel  = gFB.hostBitsPerPixel;
    lcdInfo->height        = gFB.hostHeight;
    lcdInfo->width         = gFB.hostWidth;

    VC_DEBUG(Trace, "%d x %d %d bpp\n", lcdInfo->width, lcdInfo->height, lcdInfo->bitsPerPixel );
}


/****************************************************************************
*
*  lcd_mmap
*
*   Note that the bulk of this code came from the fb_mmap routine found in
*   drivers/video/fbmem.c
*
***************************************************************************/

// static
int lcd_mmap( struct file *file, struct vm_area_struct * vma )
{
    unsigned long   offset;
    unsigned long   start;
    unsigned long   len;

    // vma->vm_start    is the start of the memory region, in user space
    // vma->vm_end      is one byte beyond the end of the memory region, in user space
    // vma->vm_pgoff    is the offset (in pages) within the vm_start to vm_end region

    LCD_PUTS( "called\n" );

    if ( vma->vm_pgoff > ( ~0UL >> PAGE_SHIFT ))
    {
        LCD_DEBUG( "vm_pgoff is out of range\n" );

        // the value is outside our memory range

        return -EINVAL;
    }

    // Convert offset into a byte offset, rather than a page offset

    offset = vma->vm_pgoff << PAGE_SHIFT;

    start = (unsigned long)gFB.hostFB.physPtr; // already page-aligned

    len = PAGE_ALIGN( start + gFB.hostSizeInBytes );

    if ( offset > len )
    {
        LCD_DEBUG( "offset is too large, offset = %lu, len = %lu\n", offset, len );

        // The pointer requested by the user isn't inside of our frame buffer

        return -EINVAL;
    }


#if defined( CONFIG_ARM )
   vma->vm_page_prot = pgprot_writecombine( vma->vm_page_prot );
#elif defined( CONFIG_MIPS )
   vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#elif
   #error "Unknown architecture type!"
#endif

    offset += start;

    vma->vm_pgoff = offset >> PAGE_SHIFT;

    if ( 0 != io_remap_pfn_range( vma,
                                  vma->vm_start,
                                  offset >> PAGE_SHIFT,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot ))
    {
        LCD_DEBUG( "remap_page_range failed\n" );

        return -EAGAIN;
    }

    return 0;

} // lcd_mmap

/****************************************************************************
*
*  lcd_open
*
***************************************************************************/

// static
int lcd_open( struct inode *inode, struct file *file )
{

    // LCD_DEBUG( "major = %d, minor = %d\n", MAJOR( inode->i_rdev ),  MINOR( inode->i_rdev ));

    return 0;

} /* lcd_open */

/****************************************************************************
*
*  lcd_perf_timer
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

// static
void lcd_perf_timer( unsigned long dummy )
{
    (void)dummy;

    if ( gLcdPerf )
    {
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
        gLcdPerfPrevData = gLcdPerfCurrData;

        gLcdPerfCurrData.nice = kstat_this_cpu.cpustat.nice;
        gLcdPerfCurrData.user = kstat_this_cpu.cpustat.user;
        gLcdPerfCurrData.system = kstat_this_cpu.cpustat.system +
            kstat_this_cpu.cpustat.softirq +
            kstat_this_cpu.cpustat.irq +
            kstat_this_cpu.cpustat.steal +
            kstat_this_cpu.cpustat.iowait;
        gLcdPerfCurrData.jiffies = jiffies;
#else
        arch_get_idle_profile( &gLcdPerfCurrData.idle_handle, &gLcdPerfCurrData.idle, &gLcdPerfCurrData.total );
#endif

        lcd_draw_perf();

        gLcdPerfTimer.expires = jiffies + gLcdPerfFreq + 1;
        add_timer( &gLcdPerfTimer );
    }
    else
    {
        lcd_clear_perf();
    }

} /* lcd_perf_timer */

/****************************************************************************
*
*  lcd_proc_sysctl_perf
*
*   Handler for lcd perf sysctl
*
***************************************************************************/
static int lcd_proc_sysctl_perf(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
{
    int rc;

    if ( !table || !table->data )
        return -EINVAL;

    if ( write )
    {
        int lastLcdPerf;

        lastLcdPerf = gLcdPerf;
        rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );

        if (rc < 0)
            return rc;

        /* Start timer if enabling.  The timer will be stopped in the timer function
         * if disabling.
         */
        if (gLcdPerf != lastLcdPerf)
        {
            if (gLcdPerf)
            {
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
                arch_init_idle_profile(&gLcdPerfCurrData.idle_handle);
#endif
                /* Use mod-timer to avoid starting a timer twice */
                mod_timer( &gLcdPerfTimer, jiffies + gLcdPerfFreq );
            }
        }

        return rc;
    }
    else
    {
        /* nothing special for read, just use generic int handler */
        return proc_dointvec( table, write, filp, buffer, lenp, ppos );
    }
}

#endif /* CONFIG_BCM_LCD_PERF */

/****************************************************************************
*
*  lcd_get_framebuffer_addr
*
*   Gets the address of the frame buffer
*
***************************************************************************/

void *lcd_get_framebuffer_addr( int *frame_size, dma_addr_t *dma_addr )
{
    // We only work properly if lcd_init was called first.

    if ( gFB.hostFB.virtPtr == NULL )
    {
        // Print a warning, and return NULL, so that the kernel keeps booting.
        // The link order may need to be adjusted to ensure that lcd_init
        // is called before this function is called.

        printk( KERN_ERR "*** lcd_get_framebuffer_addr: Frame Buffer has been allocated yet ***\n" );
    }

    if ( dma_addr != NULL )
    {
        *dma_addr = gFB.hostFB.physPtr;
    }
    if ( frame_size != NULL )
    {
        *frame_size = gFB.hostSizeInBytes;
    }

    return gFB.hostFB.virtPtr;

} // lcd_get_framebuffer_addr

/******************************************************************************
** FUNCTION:   lcd_is_display_regions_supported
**
** PURPOSE:
**
** PARAMETERS:
**
** RETURNS:
**
** NOTE:
******************************************************************************/
int lcd_is_display_regions_supported( void )
{
   return ( 1 );
}

/******************************************************************************
** FUNCTION:   lcd_is_dirty_row_update_supported
**
** PURPOSE:    Indicates if LCD driver supports dirty-row logic.
**
** PARAMETERS: None.
**
** RETURNS:    True (1) or False (0).
**
** NOTE:
******************************************************************************/
int lcd_is_dirty_row_update_supported( void )
{
   return VC_USE_DIRTY_ROWS;
}

/******************************************************************************
** FUNCTION:   lcd_dispman_init
**
** PURPOSE:    Create display mananger resource objects, associated with the
**             shadow frame buffer and display it to the LCD.
**
** PARAMETERS: None
**
** RETURNS:    int - < 0 is fail
**
** NOTE:
******************************************************************************/

static int lcd_dispman_init( void )
{
   int      success              = 0;  // Success by default
   //   int32_t  response             = 0;
   int      frameBufIdx;
   uint32_t native_image_ptr;
   DISPMANX_MODEINFO_T modeinfo;

   //
   gFB.hdisp = vc_dispmanx_display_open(DISPMANX_FRAMEBUF_SCREEN_DISPLAY);
  
   vc_dispmanx_display_get_info(gFB.hdisp, &modeinfo );

   gFB.hostWidth = modeinfo.width;
   gFB.hostHeight = modeinfo.height;

#if !defined( CFG_GLOBAL_LCD_BPP )
#define CFG_GLOBAL_LCD_BPP  16
#endif

   gFB.hostBitsPerPixel = CFG_GLOBAL_LCD_BPP;
#if ( CFG_GLOBAL_LCD_BPP == 16 )
   gFB.imageType = VC_FORMAT_RGBA565;
#else
   gFB.imageType = VC_FORMAT_ARGB8888;
#endif

   gFB.hostBytesPerPixel = gFB.hostBitsPerPixel / 8;
   gFB.vcPitch = ALIGN_UP( gFB.hostWidth * gFB.hostBytesPerPixel, 16 );
   gFB.hostPitch = ALIGN_UP( gFB.hostWidth * gFB.hostBytesPerPixel, 4 );
   gFB.hostSizeInBytes = gFB.hostHeight * gFB.hostPitch;
    //gFB.imageType = VC_FORMAT_RGB565;
   VC_DEBUG(Trace, "disp: %d x %d %d bpp\n", gFB.hostWidth, gFB.hostHeight,  gFB.hostBitsPerPixel);   
   // Allocate memory for the framebuffer.

   gFB.hostFB.virtPtr = lcd_alloc_framebuffer( gFB.hostSizeInBytes, &gFB.hostFB.physPtr );
   VC_DEBUG(Trace, "width=%d  h=%d bpp=%d pitch=%d sz=%d vptr=0x%08X phyptr=0x%08X\n",
			   gFB.hostWidth, gFB.hostHeight, gFB.hostBytesPerPixel, gFB.hostPitch,gFB.hostSizeInBytes,
			   (int)gFB.hostFB.virtPtr, (int)gFB.hostFB.physPtr);
	memset( gFB.hostFB.virtPtr, 0xff, gFB.hostSizeInBytes );
   
	if (( gFB.hostFB.physPtr & ~PAGE_MASK ) != 0 )
	   {
		   printk( KERN_ERR "lcd_init: We didn't get a page aligned buffer" );
		   return -ENOMEM;
	   }
   
	if ( gFB.hostFB.virtPtr == NULL )
		 {
		   return -ENOMEM;
		 }

   /* Create double-buffered frame-buffers on the VideoCore. These are cached
   ** versions of the shadow frame-buffer that exists on the host. Double buffers
   ** are used to avoid "tearing" effects, i.e. prevents the host from
   ** writing to the frame buffer at the same time that the VideoCore is
   ** reading from the frame buffer to display to the LCD. */
   for ( frameBufIdx = 0; frameBufIdx < VC_NUM_FRAME_BUFS; frameBufIdx++ )
	 {
	   VC_FB_t  *frameBuffer = &gFB.vcFrameBufferObj[ frameBufIdx ];
	   
	   if ( gFB.hostFB.virtPtr != NULL )
		 {

#if VC_USE_DIRTY_ROWS
		   
		   if (( frameBuffer->dirtyRowBits = lcd_alloc_dirtyRowBits( gFB.hostHeight )) == NULL )
			 {
			   success += -1;
			 }
#endif
		   /* Create a VideoCore-side resource. Note that the resource type is
		   ** VC_RESOURCE_TYPE_VIDEOCORE and imageParams.pointer is automatically
		   ** filled in with the Videocore memory address location of where the
		   ** resource has been created. */
		   frameBuffer->imageHndl = vc_dispmanx_resource_create(gFB.imageType,
									gFB.hostWidth,
									gFB.hostHeight,
									&native_image_ptr);	
	       VC_DEBUG(Trace, "imageHndl=0x%08X  w=%d h=%d t=%d\n", 
		   				    frameBuffer->imageHndl, gFB.hostWidth, gFB.hostHeight, gFB.imageType); 
		 }
	 }
   
   gFB.vcFB     = &gFB.vcFrameBufferObj[ 0 ];
   gFB.vcPrevFB = &gFB.vcFrameBufferObj[ 1 ];
   gFB.vcHndl = 0;

   return success;

} // lcd_dispman_init

/******************************************************************************
** FUNCTION:   lcd_dispman_deinit
**
** PURPOSE:    Deinit resources associated with VideoCore display manager.
**
** PARAMETERS: None
**
** RETURNS:    int - < 0 is fail
**
** NOTE:
******************************************************************************/

static int lcd_dispman_deinit( void )
{
   int      success = 0; // success by default
   int      frameBufIdx;


   for ( frameBufIdx = 0; frameBufIdx < VC_NUM_FRAME_BUFS; frameBufIdx++ )
   {
      VC_FB_t  *frameBuffer = &gFB.vcFrameBufferObj[ frameBufIdx ];
#if VC_USE_DIRTY_ROWS
      kfree( frameBuffer->dirtyRowBits );
      frameBuffer->dirtyRowBits = NULL;
#endif

      success += vc_dispmanx_resource_delete(frameBuffer->imageHndl);
   }

   return ( success );
}


/****************************************************************************
*
*  lcd_copy_fb_to_fb
*
***************************************************************************/

#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )
static int lcd_copy_fb_to_fb( HOST_FrameBuffer_t *srcFb, HOST_FrameBuffer_t *dstFb )
{
    // Make sure that the mutex for the source buffer has already been acquired.

    BUG_ON( !lcd_fb_is_locked( srcFb ));

    // Acquire the lock for the destination frame buffer

    if ( lcd_fb_lock_acquire( dstFb ) != 0 )
    {
        return -ERESTARTSYS;
    }

#if VC_USE_DIRTY_ROWS
    {
        int         row;
        uint8_t    *srcPtr;
        uint8_t    *dstPtr;
        int         dirty = 0;
        int         firstDirtyRow = 0;

        if ( gVcDebugDirtyRows )
        {
            printk( "lcd_copy_fb_to_fb: %s: ", srcFb->name );
            vc_dump_dirtyRowBits( srcFb->dirtyRowBits, gFB.hostHeight );
        }

        srcPtr = srcFb->virtPtr;
        dstPtr = dstFb->virtPtr;

        for ( row = 0; row < gFB.hostHeight; row++ ) 
        {
            int thisRowDirty = 0;

            if ( IS_ROW_DIRTY( srcFb->dirtyRowBits, row ))
            {
                MARK_ROW_CLEAN( srcFb->dirtyRowBits, row );

                if ( gVcCompareDirty && ( memcmp( dstPtr, srcPtr, gFB.hostPitch ) == 0 ))
                {
                    // The lines are identical - nothing to do
                }
                else
                {
                    thisRowDirty = 1;
                }
            }

            if ( thisRowDirty )
            {
                if ( !dirty )
                {
                    dirty = 1;
                    firstDirtyRow = row;
                }
                memcpy( dstPtr, srcPtr, gFB.hostPitch );

                MARK_ROW_DIRTY( dstFb->dirtyRowBits, row );
            }
            else
            {
                if ( dirty )
                {
                    dirty = 0;
                    FB_KNLLOG( "FB-FB Dirty: %d-%d\n", firstDirtyRow, row - 1 );
                }
            }

            srcPtr += gFB.hostPitch;
            dstPtr += gFB.hostPitch;
        }

        if ( dirty )
        {
            dirty = 0;
            FB_KNLLOG( "Dirty: %d-%d\n", firstDirtyRow, row - 1 );
        }
    }
#else
    memcpy( dstFb->virtPtr, srcFb->virtPtr, gFB.hostSizeInBytes );
#endif

    lcd_fb_lock_release( dstFb );

    return 0;

} // lcd_copy_fb_to_fb
#endif

//	Frame buffer layer number. Set this to be below the video layer to allow video to be displayed on the lcd when
//	playback is initiated from the menu.
#define	K_FB_LAYER_NUMBER	-1

/****************************************************************************
*
*  lcd_copy_fb_to_videocore
*
***************************************************************************/

int lcd_undisplay(void)
	{
	     int      success  = 0; // success by default
         DISPMANX_UPDATE_HANDLE_T update;

		  if ( gFB.vcHndl == 0 )
	    	return 0;
		  
	      /* Remove the VideoCore object from the inactive frame, and re-add it
	      ** to the newly active frame. */
	      update = vc_dispmanx_update_start(10);
	      success =  vc_dispmanx_element_remove(update, gFB.vcHndl);
	      gFB.vcHndl = 0;
		  success += vc_dispmanx_update_submit_sync(update);

		  return 0;
	 }


int lcd_dispman_copy_fb_to_videocore( HOST_FrameBuffer_t *fb, int yieldRequired )
{
  int      success  = 0; // success by default
  timer_tick_count_t   t1, t2, t3, t4, t5, t6;
  DISPMANX_UPDATE_HANDLE_T update;
  VC_RECT_T src_rect, dest_rect;
  
#if VC_USE_DIRTY_ROWS
  int      row;
  int      dirty = 0;
  int      firstDirtyRow = 0;
  
  BUG_ON( !lcd_fb_is_locked( fb ));
  
  if ( gVcDebugDirtyRows )
	{
	  printk( "lcd_dispman_copy_fb_to_videocore: %s: ", fb->name );
	  vc_dump_dirtyRowBits( fb->dirtyRowBits, gFB.hostHeight );
	}
  
  memcpy( gFB.vcFB->dirtyRowBits, fb->dirtyRowBits, BITS_TO_BYTES( gFB.hostHeight ));
  memset( fb->dirtyRowBits, 0, BITS_TO_BYTES( gFB.hostHeight ));
  
  // Mark the imaginary row just beyond the bottom of the LCD as clean. This
  // allows us to use a <= in the for loop and simplify the logic since we'll 
  // be guaranteed to hit a clean row.
  
  MARK_ROW_CLEAN( gFB.vcFB->dirtyRowBits, gFB.hostHeight );
  MARK_ROW_CLEAN( gFB.vcPrevFB->dirtyRowBits, gFB.hostHeight );

  // Walk through the dirty rows, and transfer any rows which are marked
  // dirty. Note that since there are two buffers on the videocore, and
  // we're tranferring into the "old" one, we need to transfer any rows
   // which are marked dirty in either the "new" or the "old" one.
  
  for ( row = 0; row <= gFB.hostHeight; row++ ) 
	{
	  if ( IS_ROW_DIRTY( gFB.vcFB->dirtyRowBits, row )
		   ||   ( !gVcCopyImage && IS_ROW_DIRTY( gFB.vcPrevFB->dirtyRowBits, row )))
		{
		  // Only mark the previous one a clean, since a swap will
		  // happen and it will become the current
		  
		  MARK_ROW_CLEAN( gFB.vcPrevFB->dirtyRowBits, row );
		  
		  if ( !dirty )
			{
			  dirty = 1;
			  firstDirtyRow = row;
			}
		}
	  else
		{
		  if ( dirty )
			{
			  // We've hit a clean row after a bunch of dirty ones, send the 
			  // dirty rows from the host frame buffer down to the video core.
			  
			  FB_KNLLOG( "FB-VC Dirty: %d-%d\n", firstDirtyRow, row - 1 );
			  VC_DEBUG(Trace, "FB-VC Dirty: %d-%d\n", firstDirtyRow, row - 1 );
			  			  
			  if( gVcLcdUseUpdateThd == 0 )
				{
				  lcd_dispman_write_bitmap( gFB.vcFB->image.pointer,
							    fb->virtPtr,
							    0,                    // x
							    firstDirtyRow,        // y
							    gFB.hostWidth,        // width
							    row - firstDirtyRow,  // height
							    gFB.vcPitch,
							    gFB.hostPitch,
							    yieldRequired );
				}
			  else
				{
				  /* use a higher priority thread to do the update */
				  gXferThreadStruct.vcAddr = gFB.vcFB->image.pointer;
				  gXferThreadStruct.hostAddr = fb->virtPtr;
				  gXferThreadStruct.x = 0;
				  gXferThreadStruct.y = firstDirtyRow;
				  gXferThreadStruct.width = gFB.hostWidth;
				  gXferThreadStruct.height = row - firstDirtyRow;
				  gXferThreadStruct.vcPitch = gFB.vcPitch;
				  gXferThreadStruct.hostPitch = gFB.hostPitch;
				  gXferThreadStruct.yieldRequired = yieldRequired;
				  
				  up( &gXferThreadStruct.xfer_start );
				  
				  if( down_interruptible( &gXferThreadStruct.xfer_complete ) == 0 )
					{
					   FB_KNLLOG( "transfer from thread compelte\n");
					}
				}
			  
			  dirty = 0;
			}
		}
	}
#else
  // We're not using dirty rows, so transfer the whole buffer.
  
  vc_dispmanx_rect_set( &dest_rect, 0, 0, gFB.hostWidth, gFB.hostHeight);
  vc_dispmanx_resource_write_data(gFB.vcFB->imageHndl,
				  gFB.imageType,
				  gFB.hostPitch,
				  fb->virtPtr,
				  &dest_rect );
#endif
  
  // We've now updated the bitmap on the videocore to match our copy, 
  // tell the videocore to use it.
  
  t1 = timer_get_tick_count();
  update = vc_dispmanx_update_start(10);
  if(update)
	{
	  t2 = timer_get_tick_count();
	  
	  if ( gFB.vcHndl != 0 )
	    {
	      /* Remove the VideoCore object from the inactive frame, and re-add it
	      ** to the newly active frame. */
	      
	      success =  vc_dispmanx_element_remove(update, gFB.vcHndl);
	      gFB.vcHndl = 0;
	    }
	  
	  t3 = timer_get_tick_count();
	  
	  vc_dispmanx_rect_set( &dest_rect, 0, 0, gFB.hostWidth, gFB.hostHeight);
	  vc_dispmanx_rect_set(&src_rect, 0, 0, gFB.hostWidth << 16, gFB.hostHeight << 16 );
	  gFB.vcHndl = vc_dispmanx_element_add(update,
					      gFB.hdisp,
					      K_FB_LAYER_NUMBER,
					      &dest_rect,
					      gFB.vcFB->imageHndl,
					      &src_rect,
					      DISPMANX_FLAGS_ALPHA_SOURCE|DISPMANX_FLAGS_ALPHA_FIXED,
					      255,
					      0,
					      VC_IMAGE_ROT0);
					      
	  t4 = timer_get_tick_count();
	  
	  /* This call blocks until the update is done. */
	  success += vc_dispmanx_update_submit_sync(update);
	  
	  t5 = timer_get_tick_count();
	  
	  if (( success == 0 ) && gVcCopyImage )
		{
		  
		  //		  success += vc_dispman_copy_dirty_rows( &gFB.vcFB->image, &gFB.vcPrevFB->image, 
		  //						 gFB.hostHeight, gFB.vcFB->dirtyRowBits, &response );
		}
	  t6 = timer_get_tick_count();
	  
	  if ( success == 0 )
		{
		  // We've just sent down a framebuffer. Time to swap.
		  
		  gFB.vcPrevFB = gFB.vcFB;
		  gFB.vcFB++;
		  if ( gFB.vcFB >= &gFB.vcFrameBufferObj[ VC_NUM_FRAME_BUFS ])
			{
			  gFB.vcFB = &gFB.vcFrameBufferObj[ 0 ];
			}
		}
	  
	  if ( gVcDispmanStats )
		{
		  uint32_t  t1u, t2u, t3u, t4u, t5u;
		  
		  t1u = ( t2 - t1 ) / ( timer_get_tick_rate() / 1000000 );
		  t2u = ( t3 - t2 ) / ( timer_get_tick_rate() / 1000000 );
		  t3u = ( t4 - t3 ) / ( timer_get_tick_rate() / 1000000 );
		  t4u = ( t5 - t4 ) / ( timer_get_tick_rate() / 1000000 );
		  t5u = ( t6 - t5 ) / ( timer_get_tick_rate() / 1000000 );
		  
		  printk( "dispman-stats: s:%6u r:%6u a:%6u e:%6u c:%6u\n",
				  t1u, t2u, t3u, t4u, t5u );
		}
	}
   
  return ( success );
}

/******************************************************************************
** FUNCTION:   lcd_dispman_update_framebuffer
**
** PURPOSE:    Routine to update an area of the framebuffer on VideoCore.
**
** RETURNS:    int - < 0 is fail
**
** NOTE:
******************************************************************************/

int lcd_dispman_update_framebuffer( void )
{
    int rc;
#if ( CFG_GLOBAL_EXTRA_FRAME_BUFFERS > 0 )
    static int firstUpdate = 1;
    int        timerDelay;
#endif

    BUG_ON( !lcd_fb_is_locked( &gFB.hostFB ));

#if VC_USE_DIRTY_ROWS
    if ( gVcDebugDirtyRows )
    {
        printk( "lcd_dispmap_update_framebuffer: hostFb: " );
        vc_dump_dirtyRowBits(  gFB.hostFB.dirtyRowBits, gFB.hostHeight );
    }
#endif

    {
        // In this case we've only got one framebuffer and we're doing synchronous
        // updates. Since this path may be through an ioctl call we request
        // that a yield be used so we don't hold the BKL (big-kernel-lock) for
        // a long period of time.

        rc = lcd_dispman_copy_fb_to_videocore( &gFB.hostFB, VC_LCD_YIELD );

    }


    return rc;

} // lcd_dispman_update_framebuffer

/******************************************************************************	
*	
* FUNCTION:   lcd_dispman_write_bitmap
*
******************************************************************************/

void lcd_backlight_enable( LCD_BACKLIGHT_LEVEL level )
{
#if defined(HW_LCD_BACKLIGHT_PIN)
	//turn on backlight from arm side
	if (level != LCD_BACKLIGHT_OFF) level = 1;
#if 1
    vchost_gpio_direction_output( HW_LCD_BACKLIGHT_PIN, level );
#else
    vchost_gpio_pintype_set(HW_LCD_BACKLIGHT_PIN, GPIO_PIN_TYPE_OUTPUT, GPIO_NO_INTERRUPT);
	vchost_gpio_pinval_set(HW_LCD_BACKLIGHT_PIN, level );
#endif
	printk (KERN_INFO "set HW_LCD_BACKLIGHT_PIN %d to %d\n",HW_LCD_BACKLIGHT_PIN,level);
#else
	FB_KNLLOG( "BACKLIGHT NOT IMPLEMENTED\n" );
#endif	
}

void lcd_backlight_init( void )
{
#if defined(HW_LCD_BACKLIGHT_PIN)
    vchost_gpio_request( HW_LCD_BACKLIGHT_PIN, "backlight" );
#endif
}
void lcd_backlight_deinit( void )
{
}

/******************************************************************************
** FUNCTION:   lcd_dispman_write_bitmap
**
** PURPOSE:    Copy bitmap from host memory to VideoCore memory.
**
** PARAMETERS: dest     (in)  VideoCore destination address.
**             src      (in)  Host source address.
**             x, y     (in)  Offset start copy location.
**             width    (in)  Width to copy in pixels.
**             height   (in)  Height to copy in pixels.
**             vcPitch  (in)  Width in bytes of one scanline on the VideoCore.
**             hostPitch(in)  Width in bytes of one scanline on the host.
**
**
** RETURNS:    0 on success, else error code.
**
** NOTE:
******************************************************************************/

int lcd_dispman_write_bitmap(uint32_t vcAddr, void* hostAddr, int x, int y,
			     int width, int height, int vcPitch, int hostPitch,
			     int yieldRequired)
{
   int         status = 0;
   int         h;
   int         transferWidthBytes;
   uint8_t    *hostBitmap  = (uint8_t *) hostAddr;
   uint32_t    vcBitmap    = vcAddr;

   timer_tick_count_t   t1 = 0;
   timer_tick_count_t   t2 = 0;


   hostBitmap += ( ( y * hostPitch ) + x * gFB.hostBytesPerPixel );
   vcBitmap   += ( ( y * vcPitch )   + x * gFB.hostBytesPerPixel );

   transferWidthBytes = width * gFB.hostBytesPerPixel;

   if ( gLcdPrintStats )
     {
       t1 = timer_get_tick_count();
     }

   if ( gFB.hostBitsPerPixel == 16 )
   {
      for ( h = 0; h < height; h++ )
      {
         if ( yieldRequired && (( h % 20  ) == 0 ))
         {
            yield();
         }
         VC_HOST_WRITE_BYTESWAPPED_16( vcBitmap, hostBitmap, transferWidthBytes, 1 );

         hostBitmap  += hostPitch;
         vcBitmap    += vcPitch;
      }
   }
   else
   {
      for ( h = 0; h < height; h++ )
      {
         if ( yieldRequired && (( h % 20  ) == 0 ))
         {
            yield();
         }
         VC_HOST_WRITE_BYTESWAPPED_32( vcBitmap, hostBitmap, transferWidthBytes, 1 );

         hostBitmap  += hostPitch;
         vcBitmap    += vcPitch;
      }
   }
   if ( gLcdPrintStats )
   {
      uint32_t kBytesPerSec;
      uint32_t useconds;

      t2 = timer_get_tick_count();

      useconds = ( t2 - t1 ) / ( timer_get_tick_rate() / 1000000 );

      if ( useconds == 0 )
      {
         kBytesPerSec = 0;
      }
      else
      {
         // Since we need to print in integer, we need take kbytes/sec * 1000
         //
         // kBytes/sec = ( bytes / 1024 ) / ( usec / 1000000 )
         //            = ( bytes / 1024 ) * ( 1000000 / usec )

         kBytesPerSec = (((( height * transferWidthBytes ) * 1000 ) / useconds ) * 1000 ) / 1024;
      }
#if (CFG_GLOBAL_CPU == MIPS32) && 0
      if ( irqTick == 0 )
      {
          uint32_t updateUsec = ( t1 - updateTick ) / ( timer_get_tick_rate() / 1000000 );

          printk( "lcd_write: Last Update: %u.%06u secs\n",
                  updateUsec / 1000000u, updateUsec % 1000000 );

      }
      else
      {
         uint32_t irqUsec    = ( t1 - irqTick )    / ( timer_get_tick_rate() / 1000000 );
         uint32_t readUsec   = ( t1 - readTick )   / ( timer_get_tick_rate() / 1000000 );

         printk( "lcd_write: Last Keypad IRQ: %u.%06u secs, Read: %u.%06u secs\n",
                 irqUsec    / 1000000u, irqUsec    % 1000000, 
                 readUsec   / 1000000u, readUsec   % 1000000 );

      }
#endif

      printk( "lcd_write: y:%d h:%d %u.%06u sec %d kbytes/sec\n", 
              y, h, useconds / 1000000u, useconds % 1000000u, kBytesPerSec );
   }
#if (CFG_GLOBAL_CPU == MIPS32)
   gLastUpdateTick = timer_get_tick_count();
#endif

   return status;

} // lcd_dispman_write_bitmap

/****************************************************************************/

module_init(lcd_init);
module_exit(lcd_exit);

EXPORT_SYMBOL (lcd_colorbars);
EXPORT_SYMBOL (lcd_dispman_write_bitmap);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom LCD Driver");
