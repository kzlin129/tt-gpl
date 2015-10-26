/* linux/drivers/media/video/samsung/rp/s3c_rp_rot_regs.c
 *
 * Register interface file for Samsung Renderer pipeline driver
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
#include <asm/io.h>
#include <mach/map.h>
#include <plat/regs-rotator.h>

#include "s3c_rp.h"

int s3c_rp_rot_set_param(struct s3c_rp_control *ctrl)
{
	unsigned int regs = 0;
	unsigned int pixelformat;

	if((ctrl->v4l2.video_out_fmt.pixelformat != V4L2_PIX_FMT_YUV420) && (ctrl->v4l2.video_out_fmt.pixelformat != V4L2_PIX_FMT_RGB32)) {
		err("V4L2_PIX_FMT_YUV420 and V4L2_PIX_FMT_RGB32 are only supported.");
		return -1;
	}

	/* Rotator size setting */
	regs = S3C_ROT_SRC_WIDTH(ctrl->v4l2.video_out_fmt.width) 
		| S3C_ROT_SRC_HEIGHT(ctrl->v4l2.video_out_fmt.height);
	__raw_writel(regs, ctrl->rot.regs + S3C_ROTATOR_SRCSIZEREG);

	regs = __raw_readl(ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);
	regs &= ~S3C_ROTATOR_CTRLREG_MASK;

	switch (ctrl->v4l2.video_out_fmt.pixelformat) {
	case V4L2_PIX_FMT_YUV420:
		pixelformat = S3C_ROTATOR_CTRLCFG_INPUT_YUV420;
		break;
	case V4L2_PIX_FMT_RGB32:
		pixelformat = S3C_ROTATOR_CTRLCFG_INPUT_RGB888;
		break;
	default:
		err("Invalid pixelformat !\n");
		return -EINVAL;
	}

	regs |= pixelformat;
	regs |= S3C_ROT_ROT_DEGREE(ctrl->rot.degree);
	regs |= S3C_ROTATOR_CTRLCFG_ENABLE_INT;

	__raw_writel(regs, ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);

	return 0;
}


void s3c_rp_rot_set_addr(struct s3c_rp_control *ctrl, unsigned int index)
{
	unsigned int	src_phys_y, src_phys_cb, src_phys_cr, src_phys_rgb;
	unsigned int	dst_phys_y, dst_phys_cb, dst_phys_cr, dst_phys_rgb;	

	if(ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_YUV420) {
		src_phys_y	= ctrl->user_buf[index].buf_addr.phys_y;
		src_phys_cb	= src_phys_y + ctrl->v4l2.video_out_fmt.width * ctrl->v4l2.video_out_fmt.height;
		src_phys_cr	= src_phys_cb + ((ctrl->v4l2.video_out_fmt.width * ctrl->v4l2.video_out_fmt.height)>>2);

		dst_phys_y	= ctrl->driver_buf[index].buf_addr.phys_y;
		dst_phys_cb	= dst_phys_y + ctrl->v4l2.video_out_fmt.width * ctrl->v4l2.video_out_fmt.height;
		dst_phys_cr	= dst_phys_cb + ((ctrl->v4l2.video_out_fmt.width * ctrl->v4l2.video_out_fmt.height)>>2);

		/* set source address */
		__raw_writel(src_phys_y,	ctrl->rot.regs + S3C_ROTATOR_SRCADDRREG0);
		__raw_writel(src_phys_cb,	ctrl->rot.regs + S3C_ROTATOR_SRCADDRREG1);
		__raw_writel(src_phys_cr,	ctrl->rot.regs + S3C_ROTATOR_SRCADDRREG2);    

		/* set destination address */
		__raw_writel(dst_phys_y,	ctrl->rot.regs + S3C_ROTATOR_DESTADDRREG0);
		__raw_writel(dst_phys_cb,	ctrl->rot.regs + S3C_ROTATOR_DESTADDRREG1);
		__raw_writel(dst_phys_cr,	ctrl->rot.regs + S3C_ROTATOR_DESTADDRREG2);
	} else {
		src_phys_rgb	= ctrl->user_buf[index].buf_addr.phys_rgb;
		dst_phys_rgb	= ctrl->driver_buf[index].buf_addr.phys_rgb;

		/* set source & destination address */
		__raw_writel(src_phys_rgb,	ctrl->rot.regs + S3C_ROTATOR_SRCADDRREG0);
		__raw_writel(dst_phys_rgb,	ctrl->rot.regs + S3C_ROTATOR_DESTADDRREG0);

	}
}

void s3c_rp_rot_start(struct s3c_rp_control *ctrl)
{
	unsigned int regs = 0;

	regs = __raw_readl(ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);
	__raw_writel(regs | S3C_ROTATOR_CTRLCFG_START_ROTATE, ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);
}

void s3c_rp_rot_enable_int(struct s3c_rp_control *ctrl, unsigned int onoff)
{
	unsigned int regs;

	regs = __raw_readl(ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);

	if(onoff == TRUE) {
		regs |= S3C_ROTATOR_CTRLCFG_ENABLE_INT;
	} else {
		regs &= ~S3C_ROTATOR_CTRLCFG_ENABLE_INT;
	}

	__raw_writel(regs, ctrl->rot.regs + S3C_ROTATOR_CTRLCFG);
}

