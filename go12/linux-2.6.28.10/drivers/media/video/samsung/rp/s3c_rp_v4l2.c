/* linux/drivers/media/video/samsung/rp/s3c_rp_v4l2.c
 *
 * V4L2 interface support file for Samsung Renderer pipeline driver
 *
 * Jonghun Han, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>

#include "s3c_rp.h"

static int s3c_rp_v4l2_querycap(struct file *filp, void *fh, struct v4l2_capability *cap)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;

	strcpy(cap->driver, "Samsung Renderer pipeline Driver");
	strlcpy(cap->card, ctrl->vd->name, sizeof(cap->card));
	sprintf(cap->bus_info, "Renderer pipeline AHB-bus");

	cap->version = 0;
	cap->capabilities = (V4L2_CAP_VIDEO_OVERLAY | \
				V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING);

	return 0;
}

static int s3c_rp_v4l2_g_fmt_vid_out(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	
	f->fmt.pix = ctrl->v4l2.video_out_fmt;

	return 0;
}

static int s3c_rp_v4l2_try_fmt_vid_out(struct file *filp, void *fh, struct v4l2_format *f)
{
	unsigned int	fixup_width;
	int		ret = 0;

	/* Check pixel format */
	if ((f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) && (f->fmt.pix.pixelformat != V4L2_PIX_FMT_RGB32)) {
		info("The pixelformat of V4L2_BUF_TYPE_VIDEO_OUTPUT must be V4L2_PIX_FMT_YUV420 or V4L2_PIX_FMT_RGB32.\n");
		info("The renderer pipeline driver will change the pixelformat to V4L2_PIX_FMT_YUV420.\n");
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;	/* Default value */
		ret = -1;
	}

	if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		/* Check pixel width : pixel width must be 8's multiple. */
		if (f->fmt.pix.width > S3C_RP_YUV_SRC_MAX_WIDTH) {
			info("The width of V4L2_BUF_TYPE_VIDEO_OUTPUT must be up to %d pixels.\n",
				S3C_RP_YUV_SRC_MAX_WIDTH);
			info("The renderer pipeline driver will change the width from %d to %d.\n",
				f->fmt.pix.width, S3C_RP_YUV_SRC_MAX_WIDTH);
			f->fmt.pix.width = S3C_RP_YUV_SRC_MAX_WIDTH;
			ret = -1;		
		}	
	} else if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32) {
		/* fall through : We cannot check max size. */
		/* Because rotation will be called after VIDIOC_S_FMT. */
	}

	/* Check pixel width : pixel width must be 8's multiple. */
	if ((f->fmt.pix.width)%8 != 0) {
		info("The width of V4L2_BUF_TYPE_VIDEO_OUTPUT must be 8's multiple.\n");
		fixup_width = (f->fmt.pix.width) & 0xFF1;
		info("The renderer pipeline driver will change the width from %d to %d.\n",
			f->fmt.pix.width, fixup_width);
		f->fmt.pix.width = fixup_width;
		ret = -1;		
	}

	/* Fill the return value. */
	if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
		/* YUV420 format uses 1.5 bytes per pixel. */
		f->fmt.pix.bytesperline	= (f->fmt.pix.width * 3)>>1;
	} else {
		/* ARGBX888 format uses 4 bytes per pixel. */
		f->fmt.pix.bytesperline	= f->fmt.pix.width<<2;
	}

	f->fmt.pix.sizeimage	= f->fmt.pix.bytesperline * f->fmt.pix.height;
	f->fmt.pix.colorspace	= V4L2_COLORSPACE_SMPTE170M;

	return ret;
}

static int s3c_rp_v4l2_s_fmt_vid_out(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	int		ret;

	/* Check stream status */
	if (ctrl->stream_status != RP_STREAMOFF) {
		err("The data format cannot be changed at this time.\n	\
			Because the streaming is already in progress.\n");
		return -EBUSY;
	}

	ret = s3c_rp_v4l2_try_fmt_vid_out(filp, fh, f);
	if (ret == -1) {
		err("Ther renderer pipeline cannot support the output argument which you set.\n");
		return -1;
	}

	ctrl->v4l2.video_out_fmt = f->fmt.pix;

	return 0;
}


static int s3c_rp_v4l2_g_fmt_vid_overlay(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;

	f->fmt.win = ctrl->v4l2.video_overlay_fmt;

	return 0;
}

static int s3c_rp_v4l2_s_fmt_vid_overlay(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	struct s3c_rp_scaler_cfg scaler_cfg;
	struct v4l2_rect src_rect;
	int ret;

	src_rect.left	= 0;
	src_rect.top	= 0;

	/* Check stream status */
	if (ctrl->stream_status != RP_STREAMOFF) {
		err("The data format cannot be changed at this time.\n	\
			Because the streaming is already in progress.\n");
		return -EBUSY;
	}

	/* Check Overlay Size : Overlay size must be smaller than LCD size. */
	if ((ctrl->rot.degree != ROT_90) && (ctrl->rot.degree != ROT_270)) {	/* Portrait mode */
		if (f->fmt.win.w.width > ctrl->fimd.h_res) {
			info("The width of V4L2_BUF_TYPE_VIDEO_OVERLAY must be up to %d pixels.\n",
				ctrl->fimd.h_res);
			info("The renderer pipeline driver will change the width from %d to %d.\n",
				f->fmt.win.w.width, ctrl->fimd.h_res);
			f->fmt.win.w.width = ctrl->fimd.h_res;
		}
		
		if (f->fmt.win.w.height > ctrl->fimd.v_res) {
			info("The height of V4L2_BUF_TYPE_VIDEO_OVERLAY must be up to %d pixels.\n",
				ctrl->fimd.v_res);
			info("The renderer pipeline driver will change the height from %d to %d.\n",
				f->fmt.win.w.height, ctrl->fimd.v_res);
			f->fmt.win.w.height = ctrl->fimd.v_res;
		}

		src_rect.width	= ctrl->v4l2.video_out_fmt.width;
		src_rect.height	= ctrl->v4l2.video_out_fmt.height; 
	} else {				/* Landscape mode */
		if (f->fmt.win.w.width > ctrl->fimd.v_res) {
			info("The width of V4L2_BUF_TYPE_VIDEO_OVERLAY must be up to %d pixels.\n",
				ctrl->fimd.v_res);
			info("The renderer pipeline driver will change the width from %d to %d.\n",
				f->fmt.win.w.width, ctrl->fimd.v_res);
			f->fmt.win.w.width = ctrl->fimd.v_res;
		}
		
		if (f->fmt.win.w.height > ctrl->fimd.h_res) {
			info("The height of V4L2_BUF_TYPE_VIDEO_OVERLAY must be up to %d pixels.\n",
				ctrl->fimd.h_res);
			info("The renderer pipeline driver will change the height from %d to %d.\n",
				f->fmt.win.w.height, ctrl->fimd.h_res);
			f->fmt.win.w.height = ctrl->fimd.h_res;
		}
		src_rect.width	= ctrl->v4l2.video_out_fmt.height;
		src_rect.height	= ctrl->v4l2.video_out_fmt.width;
	}

 	ret = s3c_rp_pp_query_scaler(&src_rect, &f->fmt.win.w, &scaler_cfg);
	if (ret == -1) {
		err("The renderer pipeline cannot support the overlay argument which you set.\n");
		return -1;
	}

	ctrl->v4l2.video_overlay_fmt = f->fmt.win;

	return 0;
}


static int s3c_rp_v4l2_s_ctrl(struct file *filp, void *fh, struct v4l2_control *c)
{
	struct s3c_rp_control	*ctrl = (struct s3c_rp_control *) fh;

	dev_dbg(ctrl->dev, "[%s] called. CID = %d\n", __FUNCTION__, c->id);

	switch (c->id) {
	case V4L2_CID_ROTATION:
		s3c_rp_mapping_rot(ctrl, c->value);
		break;

	default:
		err("invalid control id: %d\n", c->id);
		return -EINVAL;
	}

	return 0;
}

static int s3c_rp_v4l2_streamon(struct file *filp, void *fh, enum v4l2_buf_type i)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	int	ret=0;

	dev_dbg(ctrl->dev, "[%s] called\n", __FUNCTION__);

	if (i != V4L2_BUF_TYPE_VIDEO_OUTPUT ) {
		err("V4L2_BUF_TYPE_VIDEO_OUTPUT is only supported\n");
		return -EINVAL;
	}

	ret = s3c_rp_check_param(ctrl);
	if (ret < 0) {
		err("s3c_rp_check_param failed.\n");
		return -1;
	}

	ctrl->stream_status = RP_READY_ON;

	if ( ctrl->rot.degree != 0) {
		ret = s3c_rp_rot_set_param(ctrl);
		if (ret < 0) {
			err("s3c_rp_rot_set_param failed.\n");
			return -1;
		}
	}

	ret = s3c_rp_pp_set_param(ctrl);
	if (ret < 0) {
		err("s3c_rp_pp_set_param failed.\n");
		return -1;
	}

	return 0;
}

static int s3c_rp_v4l2_streamoff(struct file *filp, void *fh, enum v4l2_buf_type i)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	unsigned int 	loop_i, dummy_idx;
	int 	ret;	

	dev_dbg(ctrl->dev, "[%s] called\n", __FUNCTION__);

	if (i != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		err("V4L2_BUF_TYPE_VIDEO_OUTPUT is only supported\n");
		return -EINVAL;
	}

	ctrl->stream_status = RP_READY_OFF;

	if ((ctrl->rot.degree != ROT_0) && (ctrl->rot.status != ROT_IDLE)) {
		interruptible_sleep_on_timeout(&ctrl->waitq.rot, S3C_RP_ROT_TIMEOUT);
		ctrl->rot.status = ROT_IDLE;
	} else {
		/* Fall through : There is nothing to do. */
	}

	/* Waiting for stop post processor */
	s3c_rp_pp_fifo_stop(ctrl, FIFO_CLOSE);

	/* detach all buffers from incoming queue. */
	for (i = 0; i < S3C_RP_BUFF_NUM; i++) {
		ret = s3c_rp_detach_in_queue(ctrl, &dummy_idx);
		if (ret < 0)
			break;	/* There is no buffer in incoming queue. */
	}
	
	/* detach all buffers from inside queue. */
	for (i = 0; i < S3C_RP_BUFF_NUM; i++) {
		ret = s3c_rp_detach_inside_queue(ctrl, &dummy_idx);
		if (ret < 0)
			break;	/* There is no buffer in incoming queue. */
	}

	/* Move all buffer to outgoing queue. */
	for(loop_i = 0; loop_i < S3C_RP_BUFF_NUM; loop_i++) {
		ret = s3c_rp_attach_out_queue(ctrl, loop_i);
		if (ret < 0) {
			err("Failed : s3c_rp_attach_out_queue.\n");
		}
	}

	ctrl->stream_status = RP_STREAMOFF;

	return 0;
}

static int s3c_rp_v4l2_reqbufs(struct file *filp, void *fh, struct v4l2_requestbuffers *b)
{
	struct s3c_rp_control	*ctrl = (struct s3c_rp_control *) fh;
	unsigned int		i;
	int			ret = 0;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ) {
		err("V4L2_BUF_TYPE_VIDEO_OUTPUT is only supported\n");
		return -EINVAL;
	}

	if (b->memory != V4L2_MEMORY_MMAP) {
		err("V4L2_MEMORY_MMAP is only supported\n");
		return -EINVAL;
	}

	if (ctrl->stream_status != RP_STREAMOFF) {
		err("Renderer pipeline is running.\n");
		return -EBUSY;
	}

	if (ctrl->buf_info.requested == TRUE && b->count != 0 ) {
		err("Buffers were already requested.\n");
		return -EBUSY;
	}

	/* control user input */
	if (b->count > S3C_RP_BUFF_NUM) {
		info("The buffer count is modified by driver from %d to %d.\n", 
			b->count, S3C_RP_BUFF_NUM);
		b->count = S3C_RP_BUFF_NUM;
	} 

	/* Initialize all buffers */
	ret = s3c_rp_check_buf(ctrl, b->count);
	if (ret) {
		err("Reserved memory is not enough to allocate buffers.\n");
		return -1;
	}
	
	ret = s3c_rp_init_buf(ctrl);
	if (ret) {
		err("Cannot initialize the buffers\n");
		return -1;
	}

	if (b->count != 0) {	/* allocate buffers */
		ctrl->buf_info.requested = TRUE;
		
		for(i = 0; i < b->count; i++) {
			ctrl->user_buf[i].buf_state = BUF_DQUEUED;
		}
	} else {
		/* fall through */
		/* All buffers are initialized.  */
	}

	ctrl->buf_info.num = b->count;

	return 0;
}

static int s3c_rp_v4l2_querybuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ) {
		err("V4L2_BUF_TYPE_VIDEO_OUTPUT is only supported.\n");
		return -EINVAL;
	}

	if (b->memory != V4L2_MEMORY_MMAP ) {
		err("V4L2_MEMORY_MMAP is only supported.\n");
		return -EINVAL;
	}

	if (b->index > ctrl->buf_info.num ) {
		err("The index is out of bounds. You requested %d buffers. \
			But you set the index as %d.\n", ctrl->buf_info.num, b->index);
		return -EINVAL;
	}
	
	b->flags	= ctrl->user_buf[b->index].buf_flag;
	b->m.offset	= b->index * PAGE_SIZE;
 	b->length	= ctrl->user_buf[b->index].buf_length;

	return 0;
}

static int s3c_rp_v4l2_qbuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	unsigned int		index = 0;
	int			ret = 0;

	dev_dbg(ctrl->dev, "[%s] enqueued index = %d\n", __FUNCTION__, b->index);

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ) {
		err("V4L2_BUF_TYPE_VIDEO_OUTPUT is only supported.\n");
		return -EINVAL;
	}

	if (b->memory != V4L2_MEMORY_MMAP ) {
		err("V4L2_MEMORY_MMAP is only supported.\n");
		return -EINVAL;
	}

	if (b->index > ctrl->buf_info.num ) {
		err("The index is out of bounds. You requested %d buffers. \
			But you set the index as %d.\n", ctrl->buf_info.num, b->index);
		return -EINVAL;
	}

	/* Check the buffer state if the state is BUF_DQUEUED. */
	if (ctrl->user_buf[b->index].buf_state != BUF_DQUEUED) {
		err("The index(%d) buffer must be dequeued state(%d).\n", b->index, ctrl->user_buf[b->index].buf_state);
		return -EINVAL;
	}

	/* Attach the buffer to the incoming queue. */
	ret =  s3c_rp_attach_in_queue(ctrl, b->index);
	if (ret < 0) {
		err("Failed :  s3c_rp_attach_in_queue.\n");
		return -1;
	}

	if (ctrl->stream_status == RP_READY_ON) {
		ret =  s3c_rp_detach_in_queue(ctrl, &index);
		if (ret < 0) {
			err("Failed :  s3c_rp_attach_in_queue.\n");
			return -1;
		}
		
		if (ctrl->rot.degree != ROT_0) {
			ret = s3c_rp_rot_run(ctrl, index);
			if (ret < 0) {
				err("Failed : s3c_rp_run_rot().\n");
				return -1;
			}
			
			ret =  s3c_rp_detach_inside_queue(ctrl, &index);
			if (ret < 0) {
				err("Failed : s3c_rp_detach_inside_queue().\n");
				return -1;
			}		
		} 

		ret = s3c_rp_pp_start(ctrl, index);
		if (ret < 0) {
			err("Failed : s3c_rp_pp_start().\n");
			return -1;
		}

		ctrl->stream_status = RP_STREAMON;
	} else if ((ctrl->rot.degree != ROT_0) && (ctrl->rot.status == ROT_IDLE)) {
		ret =  s3c_rp_detach_in_queue(ctrl, &index);
		if (ret < 0) {
			err("Failed :  s3c_rp_detach_in_queue.\n");
			return -1;
		}
		
		ret = s3c_rp_rot_run(ctrl, index);
		if (ret < 0) {
			err("Failed : s3c_rp_run_rot().\n");
			return -1;
		}
	}

	return 0;
}

static int s3c_rp_v4l2_dqbuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
	struct s3c_rp_control	*ctrl = (struct s3c_rp_control *) fh;
	int			index = -1;
	int			ret = -1;

	ret = s3c_rp_detach_out_queue(ctrl, &index);
	if (ret < 0) {
		ret = interruptible_sleep_on_timeout(&ctrl->waitq.pp, S3C_RP_DQUEUE_TIMEOUT);
		if (ret == 0) {
			err("[0]There is no buffer in outgoing queue.\n");
			return -1;
		} else {
			ret = s3c_rp_detach_out_queue(ctrl, &index);
			if (ret < 0) {
				err("[1]There is no buffer in outgoing queue.\n");
				return -1;
			}
		}
	}

	b->index = index;
	dev_dbg(ctrl->dev, "[%s] dqueued index = %d\n", __FUNCTION__, b->index);

	return 0;
}

static int s3c_rp_v4l2_cropcap(struct file *filp, void *fh, struct v4l2_cropcap *a)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	unsigned int	max_width = 0, max_height = 0;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		err("The buffer type must be V4L2_BUF_TYPE_VIDEO_OUTPUT.\n");
		return -EINVAL;
	}

	if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_YUV420) {
		max_width	= S3C_RP_YUV_SRC_MAX_WIDTH;
		max_height	= S3C_RP_YUV_SRC_MAX_HEIGHT;
	} else if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_RGB32) {
		if ((ctrl->rot.degree == ROT_0) || (ctrl->rot.degree == ROT_180)) {
			max_width	= ctrl->fimd.h_res;
			max_height	= ctrl->fimd.v_res;
		} else {
			max_width	= ctrl->fimd.v_res;
			max_height	= ctrl->fimd.h_res;
		}
	}

	/* crop bounds */
	ctrl->v4l2.crop_bounds.left	= 0;
	ctrl->v4l2.crop_bounds.top	= 0;
	ctrl->v4l2.crop_bounds.width	= max_width;
	ctrl->v4l2.crop_bounds.height	= max_height;

	/* crop default values */
	ctrl->v4l2.crop_defrect.left	= 0;
	ctrl->v4l2.crop_defrect.top	= 0;
	ctrl->v4l2.crop_defrect.width	= max_width;
	ctrl->v4l2.crop_defrect.height	= max_height;

	/* crop pixel aspec values */
	/* To Do : Have to modify but I don't know the meaning. */
	ctrl->v4l2.pixelaspect.numerator	= 5;
	ctrl->v4l2.pixelaspect.denominator	= 3;

	a->bounds	= ctrl->v4l2.crop_bounds;
	a->defrect	= ctrl->v4l2.crop_defrect;
	a->pixelaspect	= ctrl->v4l2.pixelaspect;

	return 0;
}


static int s3c_rp_v4l2_s_crop(struct file *filp, void *fh, struct v4l2_crop *a)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) fh;
	unsigned int	max_width = 0, max_height = 0;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		err("The buffer type must be V4L2_BUF_TYPE_VIDEO_OUTPUT.\n");
		return -1;
	}

	/* Check arguments : widht and height */
	if ((a->c.width < 0) || (a->c.height < 0)) {
		err("The crop rect width and height must be bigger than zero.\n");
		return -1;
	}

	if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_YUV420) {
		max_width	= S3C_RP_YUV_SRC_MAX_WIDTH;
		max_height	= S3C_RP_YUV_SRC_MAX_HEIGHT;
	} else if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_RGB32) {
		if ((ctrl->rot.degree == ROT_0) || (ctrl->rot.degree == ROT_180)) {
			max_width	= ctrl->fimd.h_res;
			max_height	= ctrl->fimd.v_res;
		} else {
			max_width	= ctrl->fimd.v_res;
			max_height	= ctrl->fimd.h_res;
		}	
	}

	if ((a->c.width > max_width) || (a->c.height > max_height)) {
		err("The crop rect width and height must be smaller than %d and %d.\n", 
			max_width, max_height);
		return -1;
	}

	/* Check arguments : left and top */
	if ((a->c.left < 0) || (a->c.top < 0)) {
		err("The crop rect left and top must be bigger than zero.\n");
		return -1;
	}
	
	if ((a->c.left > max_width) || (a->c.top > max_height)) {
		err("The crop rect left and top must be smaller than %d, %d.\n", 
			max_width, max_height);
		return -1;
	}

	if ((a->c.left + a->c.width) > max_width) {
		err("The crop rect must be in bound rect.\n");
		return -1;
	}
	
	if ((a->c.top + a->c.height) > max_height) {
		err("The crop rect must be in bound rect.\n");
		return -1;
	}

	ctrl->v4l2.crop_rect.left	= a->c.left;
	ctrl->v4l2.crop_rect.top	= a->c.top;
	ctrl->v4l2.crop_rect.width	= a->c.width;
	ctrl->v4l2.crop_rect.height	= a->c.height;

	return 0;
}

const struct v4l2_ioctl_ops s3c_rp_v4l2_ops = {
	.vidioc_querycap		= s3c_rp_v4l2_querycap,

	.vidioc_reqbufs			= s3c_rp_v4l2_reqbufs,
	.vidioc_querybuf		= s3c_rp_v4l2_querybuf,

	.vidioc_qbuf			= s3c_rp_v4l2_qbuf,
	.vidioc_dqbuf			= s3c_rp_v4l2_dqbuf,

	.vidioc_streamon		= s3c_rp_v4l2_streamon,
	.vidioc_streamoff		= s3c_rp_v4l2_streamoff,

	.vidioc_s_ctrl			= s3c_rp_v4l2_s_ctrl,

	.vidioc_cropcap			= s3c_rp_v4l2_cropcap,
	.vidioc_s_crop			= s3c_rp_v4l2_s_crop,	

	.vidioc_g_fmt_vid_out		= s3c_rp_v4l2_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out		= s3c_rp_v4l2_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out		= s3c_rp_v4l2_try_fmt_vid_out,
	
	.vidioc_g_fmt_vid_overlay	= s3c_rp_v4l2_g_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay	= s3c_rp_v4l2_s_fmt_vid_overlay,
};

