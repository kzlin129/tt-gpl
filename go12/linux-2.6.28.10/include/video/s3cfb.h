/*
 * include/video/s3cfb.h
 *
 *
 * Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Interface
 */

#ifndef _VIDEO_S3CFB_H_
#define _VIDEO_S3CFB_H_

#ifdef KERNEL
#warning This file should not be included from within the kernel!
#else
/*
 *  ioctls
 */
#define S3C_FB_GET_BRIGHTNESS		_IOR ('F', 1,  unsigned int)
#define S3C_FB_SET_BRIGHTNESS		_IOW ('F', 2,  unsigned int)
#define S3C_FB_WIN_ON			_IOW ('F', 10, unsigned int)
#define S3C_FB_WIN_OFF			_IOW ('F', 11, unsigned int)
#define FBIO_WAITFORVSYNC		_IOW ('F', 32, unsigned int)

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
#define S3C_FB_VS_START			_IO  ('F', 103)
#define S3C_FB_VS_STOP			_IO  ('F', 104)
#define S3C_FB_VS_SET_INFO		_IOW ('F', 105, s3c_vs_info_t)
#define S3C_FB_VS_MOVE			_IOW ('F', 106, unsigned int)
#endif

#define S3C_FB_OSD_START		_IO  ('F', 201)
#define S3C_FB_OSD_STOP			_IO  ('F', 202)
#define S3C_FB_OSD_ALPHA_UP		_IO  ('F', 203)
#define S3C_FB_OSD_ALPHA_DOWN		_IO  ('F', 204)
#define S3C_FB_OSD_MOVE_LEFT		_IO  ('F', 205)
#define S3C_FB_OSD_MOVE_RIGHT		_IO  ('F', 206)
#define S3C_FB_OSD_MOVE_UP		_IO  ('F', 207)
#define S3C_FB_OSD_MOVE_DOWN		_IO  ('F', 208)
#define S3C_FB_OSD_SET_INFO		_IOW ('F', 209, s3c_win_info_t)
#define S3C_FB_OSD_ALPHA_SET		_IOW ('F', 210, unsigned int)
#define S3C_FB_OSD_ALPHA0_SET		_IOW ('F', 211, unsigned int)
#define S3C_FB_OSD_ALPHA_MODE		_IOW ('F', 212, unsigned int)

#define S3C_FB_COLOR_KEY_START		_IO  ('F', 300)
#define S3C_FB_COLOR_KEY_STOP		_IO  ('F', 301)
#define S3C_FB_COLOR_KEY_ALPHA_START	_IO  ('F', 302)
#define S3C_FB_COLOR_KEY_ALPHA_STOP	_IO  ('F', 303)
#define S3C_FB_COLOR_KEY_SET_INFO	_IOW ('F', 304, s3c_color_key_info_t)
#define S3C_FB_COLOR_KEY_VALUE		_IOW ('F', 305, s3c_color_val_info_t)

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
#define S3C_FB_GET_NUM			_IOWR('F', 306, unsigned int)
#endif

//#define S3C_FB_GET_INFO			_IOR ('F', 307, s3c_fb_dma_info_t)
#define S3C_FB_CHANGE_REQ		_IOW ('F', 308, int)
#define S3C_FB_SET_VSYNC_INT		_IOW ('F', 309, int)
#define S3C_FB_SET_NEXT_FB_INFO		_IOW ('F', 320, s3c_fb_next_info_t)
#define S3C_FB_GET_CURR_FB_INFO		_IOR ('F', 321, s3c_fb_next_info_t)

#define S3C_FB_RELEASE_DEFAULT_FB	_IO  ('F', 322)

/*
 *  Definitions
 */
#define S3C_FB_PALETTE_BUFF_CLEAR	(0x80000000)	/* entry is clear/invalid */
#define S3C_FB_COLOR_KEY_DIR_BG 	0
#define S3C_FB_COLOR_KEY_DIR_FG 	1
#define S3C_FB_DEFAULT_BACKLIGHT_LEVEL	2
#define S3C_FB_MAX_DISPLAY_OFFSET	200
#define S3C_FB_DEFAULT_DISPLAY_OFFSET	100
#define S3C_FB_MAX_ALPHA_LEVEL		0xf
#define S3C_FB_MAX_BRIGHTNESS		9
#define S3C_FB_DEFAULT_BRIGHTNESS	4
#define S3C_FB_VS_SET 			12
#define S3C_FB_VS_MOVE_LEFT		15
#define S3C_FB_VS_MOVE_RIGHT		16
#define S3C_FB_VS_MOVE_UP		17
#define S3C_FB_VS_MOVE_DOWN		18


/*
 *  structures
 */
typedef struct {
	int bpp;
	int left_x;
	int top_y;
	int width;
	int height;
} s3c_win_info_t;

typedef struct {
	int width;
	int height;
	int bpp;
	int offset;
	int v_width;
	int v_height;
} s3c_vs_info_t;

typedef struct {
	int direction;
	unsigned int compkey_red;
	unsigned int compkey_green;
	unsigned int compkey_blue;
} s3c_color_key_info_t;

typedef struct {
	unsigned int colval_red;
	unsigned int colval_green;
	unsigned int colval_blue;
} s3c_color_val_info_t;

typedef struct {
	__u32 phy_start_addr;
	__u32 xres;		/* visible resolution*/
	__u32 yres;
	__u32 xres_virtual;	/* virtual resolution*/
	__u32 yres_virtual;
	__u32 xoffset;		/* offset from virtual to visible */
	__u32 yoffset;		/* resolution	*/
	__u32 lcd_offset_x;
	__u32 lcd_offset_y;
} s3c_fb_next_info_t;

#endif /*KERNEL*/

#endif

