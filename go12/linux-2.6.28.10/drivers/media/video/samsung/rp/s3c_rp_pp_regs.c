/* linux/drivers/media/video/samsung/rp/s3c_rp_pp_regs.c
 *
 * Register interface file for Samsung S3C6410 Renderer pipeline driver 
 *
 * Jonghun Han, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <mach/map.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-lcd.h>
#include <plat/regs-pp.h>

#include "s3c_rp.h"

void s3c_rp_pp_set_envid(struct s3c_rp_control *ctrl, unsigned int onoff)
{
	unsigned int regs = 0;

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_POSTENVID);

	regs &= ~S3C_POSTENVID_ENABLE;
	regs |= S3C_PP_POSTENVID(onoff);

	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_POSTENVID);
}

int s3c_rp_pp_set_pixelformat(struct s3c_rp_control *ctrl)
{
	unsigned int regs = 0;
	unsigned int src_pixelformat = 0, dst_pixelformat = 0;

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE);

	regs &= ~S3C_PP_MODE_NARROW;
	regs |= S3C_PP_MODE_WIDE;
	regs |= S3C_PP_MODE_R2YSEL_WIDE;

	regs &= ~S3C_PP_MODE_FORMAT_MASK;

	/* Set source pixel format */
	switch (ctrl->v4l2.video_out_fmt.pixelformat) {
	case V4L2_PIX_FMT_YUV420:
		src_pixelformat	|= (0x1<<8)|(0x1<<1);
		break;
	case V4L2_PIX_FMT_RGB32:
		src_pixelformat	|= (0x1<<3)|(0x1<<2)|(0x1<<1);
		break;
	default:
		err("Invalid pixelformat !\n");
		return -EINVAL;
	}

	/* Set destination pixel format */
	dst_pixelformat = (0x1<<18)|(0x1<<13)|(0x1<<4);/* RGB24 via FIFO */

	regs |= src_pixelformat;
	regs |= dst_pixelformat;	
	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);	

	return 0;
}


static int s3c_rp_pp_check_skip_line(unsigned int src_height, unsigned int dst_height)
{
	unsigned int ret = 1;

	/* If vertical shrink ratio is bigger than 2, post processor skips some lines.*/
	if (src_height >= 2*dst_height)
		ret = src_height / dst_height;

	return ret;
}

int s3c_rp_pp_set_scaler(struct s3c_rp_control *ctrl)
{
	struct s3c_rp_scaler_cfg scaler_cfg;
	struct v4l2_rect src_rect, dst_rect;
	unsigned int	pre_ratio, pre_dst, src_size, dst_size;
	int 		ret = 0;

	src_rect.left	= 0;
	src_rect.top	= 0;
	dst_rect.left	= 0;
	dst_rect.top	= 0;

	if ((ctrl->rot.degree != ROT_90) && (ctrl->rot.degree != ROT_270)) {	/* Portrait mode */
		src_rect.width	= ctrl->v4l2.video_out_fmt.width;
		src_rect.height	= ctrl->v4l2.video_out_fmt.height; 
		dst_rect.width	= ctrl->v4l2.video_overlay_fmt.w.width;
		dst_rect.height	= ctrl->v4l2.video_overlay_fmt.w.height; 
	} else {								/* Landscape mode */
		src_rect.width	= ctrl->v4l2.video_out_fmt.height;
		src_rect.height	= ctrl->v4l2.video_out_fmt.width;
		dst_rect.width	= ctrl->v4l2.video_overlay_fmt.w.height;
		dst_rect.height	= ctrl->v4l2.video_overlay_fmt.w.width; 
	}

 	ret = s3c_rp_pp_check_skip_line(src_rect.height, dst_rect.height);
	if (ret > 1) {
		src_rect.height /= ret;
		info("Vertical shrink ratio is too big.\n");
		info("So renderer pipeline will skip %d lines like interlace.\n", (ret-1));
	}

 	ret = s3c_rp_pp_query_scaler(&src_rect, &dst_rect, &scaler_cfg);
	if (ret < 0) {
		err("The renderer pipeline cannot support the video output argument which you set.\n");
		err("Or the renderer pipeline cannot support the overlay argument which you set.\n");
		return -1;
	}

	pre_ratio	= S3C_PP_PRE_RATIO_V(scaler_cfg.prescale_v_ratio)	
				| S3C_PP_PRE_RATIO_H(scaler_cfg.prescale_h_ratio);
	pre_dst		= S3C_PP_PRE_DST_H(scaler_cfg.prescale_dst_height)	
				| S3C_PP_PRE_DST_W(scaler_cfg.prescale_dst_width);
	src_size	= S3C_PP_SRC_H(src_rect.height) 			
				| S3C_PP_SRC_W(src_rect.width);
	dst_size	= S3C_PP_DST_H(dst_rect.height)				
				| S3C_PP_DST_W(dst_rect.width);

	__raw_writel(pre_ratio,			ctrl->pp.regs + S3C_VPP_PRESCALE_RATIO);
	__raw_writel(pre_dst,			ctrl->pp.regs + S3C_VPP_PRESCALEIMGSIZE);
	__raw_writel(scaler_cfg.sh_factor,	ctrl->pp.regs + S3C_VPP_PRESCALE_SHFACTOR);
	__raw_writel(scaler_cfg.dx,		ctrl->pp.regs + S3C_VPP_MAINSCALE_H_RATIO);
	__raw_writel(scaler_cfg.dy,		ctrl->pp.regs + S3C_VPP_MAINSCALE_V_RATIO);
	__raw_writel(src_size,			ctrl->pp.regs + S3C_VPP_SRCIMGSIZE);
	__raw_writel(dst_size,			ctrl->pp.regs + S3C_VPP_DSTIMGSIZE);

	return 0;
}

void s3c_rp_pp_set_int_enable(struct s3c_rp_control *ctrl, unsigned int onoff)
{
	unsigned int regs = 0;

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE);
	
	regs |= S3C_MODE_IRQ_LEVEL;
	regs &= ~S3C_MODE_POST_INT_ENABLE;
	regs |= S3C_PP_INT_ENABLE(onoff);

	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);
}

void s3c_rp_pp_set_clock(struct s3c_rp_control *ctrl){
	unsigned int regs = 0;

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE);

	regs &= ~(S3C_PP_MODE_CLKSEL_F_MASK | S3C_PP_MODE_CLKDIR_MASK | S3C_PP_MODE_CLKVAL_F_MASK);
	regs |= S3C_PP_CLKSEL_F(0x0);		/* set clock source : hclk */
	regs |= S3C_PP_CLKDIR(0x1);		/* set direction : CLKVAL_F */
	regs |= S3C_PP_CLKVAL_F(0x1);		/* set clock divide value : CLKVAL_F = 1 */

	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);
}

static int s3c_rp_pp_fifo_open(void *param)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *)param;
	unsigned int regs;

	__raw_writel(S3C_PP_POSTENVID_DISABLE, ctrl->pp.regs + S3C_VPP_POSTENVID);

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE);  
	regs &= ~(0x1<<31);					/* Must be '0' bit */
	regs &= ~S3C_PP_MODE_INTERLACE;				/* 0: progressive mode */
	regs |= (S3C_PP_MODE_FIFO | S3C_PP_MODE_FREERUN);	/* FIFO & FREERUN */
	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);

	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE_2);  
	regs &= ~(0x3<<0);					/* Must be '0' bit */
	regs |= S3C_MODE2_CHANGE_AT_FRAME_END;			/* Address change at FRAME end */
	regs |= S3C_MODE2_ADDR_CHANGE_DISABLE;	
	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE_2);

	__raw_writel(S3C_PP_POSTENVID_ENABLE, ctrl->pp.regs + S3C_VPP_POSTENVID);	/*  PP start */

	return 0;
}

static int s3c_rp_pp_fimd_rect(const struct s3c_rp_control *ctrl, struct v4l2_rect *fimd_rect)
{
	int ret = 0;

	switch (ctrl->rot.degree) {
	case ROT_0:
		fimd_rect->left		= ctrl->v4l2.video_overlay_fmt.w.left;
		fimd_rect->top		= ctrl->v4l2.video_overlay_fmt.w.top;
		fimd_rect->width	= ctrl->v4l2.video_overlay_fmt.w.width;
		fimd_rect->height	= ctrl->v4l2.video_overlay_fmt.w.height;

		break;

	case ROT_90:
		fimd_rect->left		= ctrl->fimd.h_res - 
					(ctrl->v4l2.video_overlay_fmt.w.top \
						+ ctrl->v4l2.video_overlay_fmt.w.height);
		fimd_rect->top		= ctrl->v4l2.video_overlay_fmt.w.left;
		fimd_rect->width	= ctrl->v4l2.video_overlay_fmt.w.height;
		fimd_rect->height	= ctrl->v4l2.video_overlay_fmt.w.width;

		break;

	case ROT_180:
		fimd_rect->left		= ctrl->fimd.h_res - 
					(ctrl->v4l2.video_overlay_fmt.w.left \
						+ ctrl->v4l2.video_overlay_fmt.w.width);
		fimd_rect->top		= ctrl->fimd.v_res - 
					(ctrl->v4l2.video_overlay_fmt.w.top \
						+ ctrl->v4l2.video_overlay_fmt.w.height);
		fimd_rect->width	= ctrl->v4l2.video_overlay_fmt.w.width;
		fimd_rect->height	= ctrl->v4l2.video_overlay_fmt.w.height;

		break;

	case ROT_270:
		fimd_rect->left		= ctrl->v4l2.video_overlay_fmt.w.top;
		fimd_rect->top		= ctrl->fimd.v_res - 
					(ctrl->v4l2.video_overlay_fmt.w.left \
						+ ctrl->v4l2.video_overlay_fmt.w.width);
		fimd_rect->width	= ctrl->v4l2.video_overlay_fmt.w.height;
		fimd_rect->height	= ctrl->v4l2.video_overlay_fmt.w.width;

		break;

	default:
		ret = -1;
		err("rot degree is inavlid.\n");

		break;
	}

	return ret;
}

void s3c_rp_pp_fifo_start(struct s3c_rp_control *ctrl)
{
	struct v4l2_rect		fimd_rect;
	struct fb_var_screeninfo	var;	
	struct s3cfb_user_window	window;
	int				ret = -1;

	memset(&fimd_rect, 0, sizeof(struct v4l2_rect));

	ret = s3c_rp_pp_fimd_rect(ctrl, &fimd_rect);
	if (ret < 0)
		err("s3c_rp_pp_fimd_rect fail\n");

	/* Get WIN0 var_screeninfo  */
	ret = s3cfb_direct_ioctl(0, FBIOGET_VSCREENINFO, (unsigned long)&var);
	if (ret < 0)
		err("s3cfb_direct_ioctl(FBIOGET_VSCREENINFO) fail\n");

	/* Update WIN0 size  */
	var.xres = fimd_rect.width;
	var.yres = fimd_rect.height;
	ret = s3cfb_direct_ioctl(0, FBIOPUT_VSCREENINFO, (unsigned long)&var);
	if (ret < 0)
		err("s3cfb_direct_ioctl(FBIOPUT_VSCREENINFO) fail\n");

	/* Update WIN0 position */
	window.x = fimd_rect.left;
	window.y = fimd_rect.top;
	ret = s3cfb_direct_ioctl(0, S3CFB_WIN_POSITION, (unsigned long)&window);
	if (ret < 0)
		err("s3cfb_direct_ioctl(S3CFB_WIN_POSITION) fail\n");

	/* Open WIN0 FIFO */
	ret = ctrl->fimd.open_fifo(0, 0, s3c_rp_pp_fifo_open, (void *)ctrl);
	if (ret < 0)
		err("FIMD FIFO close fail\n");
}

static int s3c_rp_pp_fifo_close(void *param)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *)param;
	unsigned int regs;

	/* set Per Frame mode */
	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE) & ~S3C_MODE_AUTOLOAD_ENABLE;
	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);

	/* Post Processor stop */
	__raw_writel(S3C_POSTENVID_DISABLE, ctrl->pp.regs + S3C_VPP_POSTENVID);

	return 0;
}

void s3c_rp_pp_fifo_stop(struct s3c_rp_control *ctrl, int sleep)
{
	int		ret = -1;

	ret = ctrl->fimd.close_fifo(0, s3c_rp_pp_fifo_close, (void *)ctrl, sleep);
	if (ret < 0)
		err("FIMD FIFO close fail\n");
}

static void s3c_rp_pp_make_addr(struct s3c_rp_control *ctrl, unsigned int index, struct s3c_rp_pp_addr *pp_addr)
{
	unsigned int 	ret = 0;
	unsigned int	width, height, dst_height;

	if ((ctrl->rot.degree == ROT_0) || (ctrl->rot.degree == ROT_180)) {
		width		= ctrl->v4l2.video_out_fmt.width;
		height		= ctrl->v4l2.video_out_fmt.height;
		dst_height	= ctrl->v4l2.video_overlay_fmt.w.height;

		pp_addr->phys_y_start	= ctrl->user_buf[index].buf_addr.phys_y;
	} else {
		width		= ctrl->v4l2.video_out_fmt.height;
		height		= ctrl->v4l2.video_out_fmt.width;
		dst_height	= ctrl->v4l2.video_overlay_fmt.w.width;
	
		pp_addr->phys_y_start	= ctrl->driver_buf[index].buf_addr.phys_y;
	}

	pp_addr->phys_cb_start	= pp_addr->phys_y_start		+ (width * height);
	pp_addr->phys_cr_start	= pp_addr->phys_cb_start	+ ((width * height)>>2);

 	ret = s3c_rp_pp_check_skip_line(height, dst_height);
	if (ret > 1) {
		pp_addr->phys_y_offset	= width * (ret - 1);	/* Number of bytes */
		height /= ret;
	} else {
		pp_addr->phys_y_offset	= 0;			/* Number of bytes */
	}
	
	pp_addr->phys_cb_offset	= pp_addr->phys_y_offset / 2;
	pp_addr->phys_cr_offset	= pp_addr->phys_y_offset / 2;

	pp_addr->phys_y_end	= pp_addr->phys_y_start		
					+ ((width * height)	
						+ pp_addr->phys_y_offset * (height - 1));
	pp_addr->phys_cb_end	= pp_addr->phys_cb_start	
					+ (((width * height)>>2)	
						+ pp_addr->phys_cb_offset * (height/2 - 1));
	pp_addr->phys_cr_end	= pp_addr->phys_cr_start	
					+ (((width * height)>>2)	
						+ pp_addr->phys_cr_offset * (height/2 - 1));
}

void s3c_rp_pp_set_addr(struct s3c_rp_control *ctrl, unsigned int index, enum s3c_pp_addr addr_pos)
{
	struct s3c_rp_pp_addr	pp_addr;
	unsigned int		regs, width, height;
	unsigned int		phys_rgb_start, phys_rgb_end;

	width = ctrl->v4l2.video_out_fmt.width;
	height = ctrl->v4l2.video_out_fmt.height;

	if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_YUV420) {
		s3c_rp_pp_make_addr(ctrl, index, &pp_addr);
		
		if (addr_pos == PP_CURRENT) {
			__raw_writel(pp_addr.phys_y_start,	ctrl->pp.regs + S3C_VPP_ADDRSTART_Y);
			__raw_writel(pp_addr.phys_cr_start,	ctrl->pp.regs + S3C_VPP_ADDRSTART_CR);
			__raw_writel(pp_addr.phys_cb_start,	ctrl->pp.regs + S3C_VPP_ADDRSTART_CB);

			__raw_writel(pp_addr.phys_y_end,	ctrl->pp.regs + S3C_VPP_ADDREND_Y);
			__raw_writel(pp_addr.phys_cb_end,	ctrl->pp.regs + S3C_VPP_ADDREND_CB);
			__raw_writel(pp_addr.phys_cr_end,	ctrl->pp.regs + S3C_VPP_ADDREND_CR);
			
			__raw_writel(pp_addr.phys_y_offset,	ctrl->pp.regs + S3C_VPP_OFFSET_Y);
			__raw_writel(pp_addr.phys_cb_offset,	ctrl->pp.regs + S3C_VPP_OFFSET_CB);
			__raw_writel(pp_addr.phys_cr_offset,	ctrl->pp.regs + S3C_VPP_OFFSET_CR);
		} else {
			/* set the protection bit */
			regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE_2);
			regs |= S3C_MODE2_ADDR_CHANGE_DISABLE;
			__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE_2);

			/* set the next address */
			__raw_writel(pp_addr.phys_y_start,	ctrl->pp.regs + S3C_VPP_NXTADDRSTART_Y);
			__raw_writel(pp_addr.phys_cb_start,	ctrl->pp.regs + S3C_VPP_NXTADDRSTART_CB);
			__raw_writel(pp_addr.phys_cr_start,	ctrl->pp.regs + S3C_VPP_NXTADDRSTART_CR);

			__raw_writel(pp_addr.phys_y_end,	ctrl->pp.regs + S3C_VPP_NXTADDREND_Y);
			__raw_writel(pp_addr.phys_cb_end,	ctrl->pp.regs + S3C_VPP_NXTADDREND_CB);
			__raw_writel(pp_addr.phys_cr_end,	ctrl->pp.regs + S3C_VPP_NXTADDREND_CR);

			/* clear the protection bit */
			regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE_2);
			regs &= ~S3C_MODE2_ADDR_CHANGE_DISABLE;
			__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE_2);
		}
	} else {
		if (ctrl->rot.degree == ROT_0)
			phys_rgb_start	= ctrl->user_buf[index].buf_addr.phys_rgb;
		else
			phys_rgb_start	= ctrl->driver_buf[index].buf_addr.phys_rgb;
	
		phys_rgb_end	= phys_rgb_start + ((width * height)<<2);
	
		if (addr_pos == PP_CURRENT) {
			__raw_writel(phys_rgb_start,	ctrl->pp.regs + S3C_VPP_ADDRSTART_Y);
			__raw_writel(phys_rgb_end,	ctrl->pp.regs + S3C_VPP_ADDREND_Y);
		} else {
			/* set the protection bit */
			regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE_2);
			regs |= S3C_MODE2_ADDR_CHANGE_DISABLE;
			__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE_2);

			/* set the next address */
			__raw_writel(phys_rgb_start,	ctrl->pp.regs + S3C_VPP_NXTADDRSTART_Y);
			__raw_writel(phys_rgb_end,	ctrl->pp.regs + S3C_VPP_NXTADDREND_Y);

			/* clear the protection bit */
			regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE_2);
			regs &= ~S3C_MODE2_ADDR_CHANGE_DISABLE;
			__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE_2);
		}
	}
}

