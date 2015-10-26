#warning "TomTom Temporary file"

/*
 * drivers/video/s3c/s3cfb_lms430wqv.c
 *
 * Copyright (C) 2008 Jeroen Taverne <jeroen.taverne@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>

#include <video/s3cfb.h>
#include "s3cfb.h"

#define S3C_FB_HFP		8	/* front porch */
#define S3C_FB_HSW		41	/* hsync width */
#define S3C_FB_HBP		45	/* back porch */

#define S3C_FB_VFP		4	/* front porch */
#define S3C_FB_VSW		1	/* vsync width */
#define S3C_FB_VBP		12	/* back porch */

#define S3C_FB_HRES		480	/* horizon pixel  x resolition */
#define S3C_FB_VRES		272	/* line cnt       y resolution */

#define S3C_FB_HRES_VIRTUAL	S3C_FB_HRES	/* horizon pixel  x resolition */
#define S3C_FB_VRES_VIRTUAL	S3C_FB_VRES*2	/* line cnt       y resolution */

#define S3C_FB_HRES_OSD		S3C_FB_HRES	/* horizon pixel  x resolition */
#define S3C_FB_VRES_OSD		S3C_FB_VRES	/* line cnt       y resolution */

#define S3C_FB_VFRAME_FREQ     	75	/* frame rate freq */

#define S3C_FB_PIXEL_CLOCK	(S3C_FB_VFRAME_FREQ * (S3C_FB_HFP + S3C_FB_HSW + S3C_FB_HBP + S3C_FB_HRES) * (S3C_FB_VFP + S3C_FB_VSW + S3C_FB_VBP + S3C_FB_VRES))

void s3cfb_set_fimd_info(void)
{
	s3cfb_fimd.vidcon1 = S3C_VIDCON1_IHSYNC_INVERT | S3C_VIDCON1_IVSYNC_INVERT | S3C_VIDCON1_IVDEN_NORMAL;
	s3cfb_fimd.vidtcon0 = S3C_VIDTCON0_VBPD(S3C_FB_VBP - 1) | S3C_VIDTCON0_VFPD(S3C_FB_VFP - 1) | S3C_VIDTCON0_VSPW(S3C_FB_VSW - 1);
	s3cfb_fimd.vidtcon1 = S3C_VIDTCON1_HBPD(S3C_FB_HBP - 1) | S3C_VIDTCON1_HFPD(S3C_FB_HFP - 1) | S3C_VIDTCON1_HSPW(S3C_FB_HSW - 1);
	s3cfb_fimd.vidtcon2 = S3C_VIDTCON2_LINEVAL(S3C_FB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3C_FB_HRES - 1);

	s3cfb_fimd.vidosd0a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd0b = S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES - 1);

	s3cfb_fimd.vidosd1a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd1b = S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES_OSD - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES_OSD - 1);

	s3cfb_fimd.width = S3C_FB_HRES;
	s3cfb_fimd.height = S3C_FB_VRES;
	s3cfb_fimd.xres = S3C_FB_HRES;
	s3cfb_fimd.yres = S3C_FB_VRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3cfb_fimd.xres_virtual = S3C_FB_HRES_VIRTUAL;
	s3cfb_fimd.yres_virtual = S3C_FB_VRES_VIRTUAL;
#else
	s3cfb_fimd.xres_virtual = S3C_FB_HRES;
	s3cfb_fimd.yres_virtual = S3C_FB_VRES;
#endif

	s3cfb_fimd.osd_width = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_height = S3C_FB_VRES_OSD;
	s3cfb_fimd.osd_xres = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_yres = S3C_FB_VRES_OSD;

	s3cfb_fimd.osd_xres_virtual = S3C_FB_HRES_OSD;
	s3cfb_fimd.osd_yres_virtual = S3C_FB_VRES_OSD;

  	s3cfb_fimd.pixclock = S3C_FB_PIXEL_CLOCK;

	s3cfb_fimd.hsync_len = S3C_FB_HSW;
	s3cfb_fimd.vsync_len = S3C_FB_VSW;
	s3cfb_fimd.left_margin = S3C_FB_HFP;
	s3cfb_fimd.upper_margin = S3C_FB_VFP;
	s3cfb_fimd.right_margin = S3C_FB_HBP;
	s3cfb_fimd.lower_margin = S3C_FB_VBP;
}

void s3cfb_init_hw(void)
{
	printk(KERN_INFO "LCD TYPE :: LMS430WQV will be initialized\n");

	s3cfb_set_fimd_info();
	s3cfb_set_gpio();
}

