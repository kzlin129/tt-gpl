/* Video For Linux 2 Interface for camera driver.
 * (C) 2006 TomTom BV.
 *   
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *	
 * Author: Ithamar R. Adema, <ithamar.adema@tomtom.com>
 */

#include <linux/config.h>
#include <linux/dma-mapping.h>
#include <asm/arch/regs-camif.h>
#include <asm/io.h>
#include <media/ovcamchip.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/delay.h>
#include <asm/page.h>
#include <asm/system.h>
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#include <asm/arch/regs-clock.h>
#endif
#include "camera.h"

extern void disable_cam_irq( struct s3c_cam *cam );
extern void enable_cam_irq( struct s3c_cam *cam );

/* Local Prototypes */
int cam_stream_off(struct s3c_cam *cam);
	
/* --- Functions handling the Video For Linux 2 ioctl() interface --- */
static int
cam_query_capabilities(struct s3c_cam *cam, struct v4l2_capability *cap)
{
	if (!cap) return -EFAULT;

	memset(cap, 0, sizeof(*cap));
	strcpy(cap->driver, CAMERA_DRIVER_NAME);
	strlcpy(cap->card, cam->vdev->name, sizeof(cap->card));
	strcpy(cap->bus_info, CAMERA_DRIVER_NAME);
	cap->bus_info[0] = 0;
	cap->version = CAMERA_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE;

	return 0;
}

static int
cam_enumerate_inputs(struct s3c_cam *cam, struct v4l2_input *input)
{
	if (!input) return -EFAULT;
	if (input->index) return -EINVAL;

	memset(input, 0, sizeof(*input));
	strlcpy(input->name, "default", sizeof(input->name));

	return 0;
}

static int
cam_get_input(struct s3c_cam *cam, int *input)
{
	if (!input) return -EFAULT;

	*input = 0; /* Only one input supported */
	return 0;
}

static int
cam_set_input(struct s3c_cam *cam, int *input)
{
	if (!input) return -EFAULT;
	if (*input) return -EINVAL; /* Only one input supported */

	return 0;
}

static int
cam_query_control(struct s3c_cam *cam, struct v4l2_control *ctrl)
{
	if (!ctrl) return -EFAULT;

	/* As we don't define controls ourselfs, just pass it onto the ovcamchip module, so sensor module can handle it */
        if (cam->ovclient && cam->ovclient->driver->command)
		return cam->ovclient->driver->command(cam->ovclient, VIDIOC_QUERYCTRL, ctrl);

	return -EINVAL;
}

static int
cam_get_control(struct s3c_cam *cam, struct v4l2_control *ctrl)
{
	if (!ctrl) return -EFAULT;

	switch( ctrl->id )
	{
		case V4L2_CID_HFLIP :
			ctrl->value=(cam->vCICOTRGFMT & S3C2413_CICOTRGFMT_FLIPMD_XMIRROR) ? 1 : 0;
			break;

		case V4L2_CID_VFLIP :
			ctrl->value=(cam->vCICOTRGFMT & S3C2413_CICOTRGFMT_FLIPMD_YMIRROR) ? 1 : 0;
			break;

		case OVCAMCHIP_CID_FREQ :
			/* Workaround for framecorruption. To solve this problem we use LASTIRQ. However, this will */
			/* cause the actual framerate to be half of what we program. For this we need to modify what*/
			/* is specified. */
                        if (cam->ovclient && cam->ovclient->driver->command)
                                return cam->ovclient->driver->command(cam->ovclient, VIDIOC_G_CTRL, ctrl);
			else
				return -EINVAL;

			/* Correct the value found. */
			if( cam->slots_in_use == 1 )
			{
				/* Workaround is in use. Modify what is set. */
				ctrl->value/=2;
			}
			break;

                default :
                        /* As we don't define other controls ourselfs, just pass it onto the ovcamchip module */
                        /* and see if the sensor module can handle it */
                        if (cam->ovclient && cam->ovclient->driver->command)
                                return cam->ovclient->driver->command(cam->ovclient, VIDIOC_G_CTRL, ctrl);
			else
				return -EINVAL;
                        break;
        }

	/* Done. */
	return 0;
}

static int
cam_set_control(struct s3c_cam *cam, struct v4l2_control *ctrl)
{
	if (!ctrl) return -EFAULT;

	switch( ctrl->id )
	{
		case V4L2_CID_HFLIP :
			if( ctrl->value ) cam->vCICOTRGFMT|=S3C2413_CICOTRGFMT_FLIPMD_XMIRROR;
			else cam->vCICOTRGFMT&=~(S3C2413_CICOTRGFMT_FLIPMD_XMIRROR);
			writel( cam->vCICOTRGFMT, S3C2413_CICOTRGFMT );
			break;

		case V4L2_CID_VFLIP :
			if( ctrl->value ) cam->vCICOTRGFMT|=S3C2413_CICOTRGFMT_FLIPMD_YMIRROR;
			else cam->vCICOTRGFMT&=~(S3C2413_CICOTRGFMT_FLIPMD_YMIRROR);
			writel( cam->vCICOTRGFMT, S3C2413_CICOTRGFMT );
			break;

		case OVCAMCHIP_CID_FREQ :
			/* Workaround for framecorruption. To solve this problem we use LASTIRQ. However, this will */
			/* cause the actual framerate to be half of what we program. For this we need to modify what*/
			/* is specified. */
			if( cam->slots_in_use == 1 )
			{
				/* Workaround is in use. Modify what is set. */
				ctrl->value*=2;
			}

		default :
		        /* As we don't define other controls ourselfs, just pass it onto the ovcamchip module */
			/* and see if the sensor module can handle it */
			if (cam->ovclient && cam->ovclient->driver->command)
				return cam->ovclient->driver->command(cam->ovclient, VIDIOC_S_CTRL, ctrl);
			else
				return -EINVAL;
			break;
	}

	/* Done. */
	return 0;
}

static int
cam_enumerate_formats(struct s3c_cam *cam, struct v4l2_fmtdesc *f)
{
	int index;
	
	if (!f) return -EFAULT;
	
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	index = f->index;
	
	memset(f,0,sizeof(struct v4l2_fmtdesc));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->index = index;
	f->flags = 0;


	switch(index)
	{
		case 0:	/* V4L2_PIX_FMT_YUV420 - Most memory-friendly format */
			f->pixelformat = V4L2_PIX_FMT_YUV420;
			strcpy(f->description, "YUV 4:2:0 (planar)");
			break;

		case 1: /* V4L2_PIX_FMT_RGB24 - Most efficient for preview */
			f->pixelformat = V4L2_PIX_FMT_RGB24;
			strcpy(f->description, "24 bits RGB");
			break;

		case 2:	/* V4L2_PIX_FMT_RGB565 - Less resource hungry, but more processor intensive */
			f->pixelformat = V4L2_PIX_FMT_RGB565;
			strcpy(f->description, "5-6-5 bits RGB");
			break;

		case 3: /* V4L2_PIX_FMT_YUV422P - Highest quality YUV */
			f->pixelformat=V4L2_PIX_FMT_YUV422P;
			strcpy(f->description, "YUV 4:2:2 (planar)" );
			break;

		case 4:	/* V4L2_PIX_FMT_YUYV - YUV422 (YCbYCr) non planar. */
			f->pixelformat=V4L2_PIX_FMT_YUYV;
			strcpy(f->description, "YUV 4:2:2" );
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int
cam_get_format(struct s3c_cam *cam, struct v4l2_format *fmt)
{
	if (!fmt) return -EFAULT;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Ignore what the app selected, we don't do interlaced */
	fmt->fmt.pix.field = V4L2_FIELD_NONE;

	/* CAMIF wants a target width as a multiple of 8 */
	if (fmt->fmt.pix.width % 8)
		fmt->fmt.pix.width += 8 - (fmt->fmt.pix.width % 8);

	/* Default to max of camera interface if dimensions are out of range */
	if (fmt->fmt.pix.width == 0 || fmt->fmt.pix.width > CAMI_MAX_WIDTH)
		fmt->fmt.pix.width = CAMI_MAX_WIDTH;
        if (fmt->fmt.pix.height == 0 || fmt->fmt.pix.height > CAMI_MAX_HEIGHT)
		fmt->fmt.pix.width = CAMI_MAX_HEIGHT;

	/* Calculate size of image/scanline */
	switch(fmt->fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_YUV420:
			/* YUV4:2:0 Ysize=widthxheight, Usize=widthxheight/4, Vsize=widthxheight/4 */
			fmt->fmt.pix.bytesperline = fmt->fmt.pix.width;		/* U and V bytesperline=width/2, height/2. */
			fmt->fmt.pix.sizeimage=fmt->fmt.pix.width * fmt->fmt.pix.height * 3/2;
			break;

		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YUV422P:
			/* YUV4:2:2 Ysize=widthxheight, Usize=widthxheight/2, Vsize=widthxheight/2 */
			/* Doesn't matter whether we output planar or non planar for the size. */
			fmt->fmt.pix.bytesperline = fmt->fmt.pix.width;		/* U and V bytesperline=width/2, height/1. */
			fmt->fmt.pix.sizeimage=fmt->fmt.pix.width * fmt->fmt.pix.height * 2;
			break;

		case V4L2_PIX_FMT_RGB24:
			fmt->fmt.pix.bytesperline = fmt->fmt.pix.width * sizeof(__u32);
			fmt->fmt.pix.sizeimage = fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
			break;

		case V4L2_PIX_FMT_RGB565:
			fmt->fmt.pix.bytesperline = fmt->fmt.pix.width * sizeof(__u16);
			fmt->fmt.pix.sizeimage = fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
			break;

		default:
			return -EINVAL;
	}


	return 0;
}

static int
cam_try_format(struct s3c_cam *cam, struct v4l2_format *fmt)
{
	/* Check and adjust format, but don't set it yet */
	return cam_get_format(cam,fmt);
}

static int cam_set_scaler( struct s3c_cam *cam )
{
	short winHorOfst2, winVerOfst2;
	int src_width, src_height, dst_width, dst_height, h_shift, v_shift;
	int PreHorRatio_xx, PreVerRatio_xx, PreDstHeight_xx, 
	    PreDstWidth_xx, MainVerRatio_xx, MainHorRatio_xx, SHfactor_xx;
	long val;

	/* Verify the size is correct. */
	if( ((cam->croprect.width+cam->croprect.top) > cam->defres.width) ||
	    ((cam->croprect.height+cam->croprect.left) > cam->defres.height) )
		return -EINVAL;

	/* First set the registers correctly for the cropping/offset. */
	if( (cam->croprect.top != 0) || (cam->croprect.left != 0) ||
	    (cam->croprect.width < cam->defres.width) ||
	    (cam->croprect.height < cam->defres.height) )
	{
		cam->vCIWDOFST = S3C2413_CIWDOFST_WINOFSEN |
			(cam->croprect.left << S3C2413_CIWDOFST_WINHOROFST_SHIFT) |
			(cam->croprect.top << S3C2413_CIWDOFST_WINVEROFST_SHIFT);
		writel( cam->vCIWDOFST, S3C2413_CIWDOFST );

		winHorOfst2 = cam->defres.width - (cam->croprect.left + cam->croprect.width);
		winVerOfst2 = cam->defres.height - (cam->croprect.top + cam->croprect.height);
		cam->vCIDOWSFT2 = (winHorOfst2 << S3C2413_CIDOWSFT2_WINHOROFST2_SHIFT) |
			(winVerOfst2 << S3C2413_CIDOWSFT2_WINVEROFST2_SHIFT);
		writel( cam->vCIDOWSFT2, S3C2413_CIDOWSFT2 );
	}	
	else
	{
		/* Disable cropping. */
		cam->vCIWDOFST&=~(S3C2413_CIWDOFST_WINOFSEN | S3C2413_CIWDOFST_WINVEROFST_MASK | S3C2413_CIWDOFST_WINHOROFST_MASK);
		writel( cam->vCIDOWSFT2, S3C2413_CIDOWSFT2 );
	}

	/* Setup CAMIF scaling. Note that we enable the scaler regardless of if we actually scale. This is because otherwise */
	/* we cannot output YUV420 in YUV mode, since the conversion is done by the scaler codec. */
	src_width = cam->croprect.width;
	src_height = cam->croprect.height;
	dst_width = cam->currfmt.fmt.pix.width;
	dst_height = cam->currfmt.fmt.pix.height;

	if (src_width >= 64 * dst_width)	{ return -EINVAL; /* Out Of Horizontal Scale Range */ }
	else if (src_width >= 32 * dst_width)	{ PreHorRatio_xx = 32; h_shift = 5; }
	else if (src_width >= 16 * dst_width)	{ PreHorRatio_xx = 16; h_shift = 4; }
	else if (src_width >= 8 * dst_width)	{ PreHorRatio_xx = 8;  h_shift = 3; }
	else if (src_width >= 4 * dst_width)	{ PreHorRatio_xx = 4;  h_shift = 2; }
	else if (src_width >= 2 * dst_width)	{ PreHorRatio_xx = 2;  h_shift = 1; }
	else					{ PreHorRatio_xx = 1;  h_shift = 0; }

	PreDstWidth_xx  = src_width / PreHorRatio_xx;
	MainHorRatio_xx = (src_width << 8) / (dst_width << h_shift);

	if	(src_height >= 64 * dst_height)		{ return -EINVAL; /* Out Of Vertical Scale Range */ }
	else if (src_height >= 32 * dst_height)		{ PreVerRatio_xx = 32; v_shift = 5; }
	else if (src_height >= 16 * dst_height)		{ PreVerRatio_xx = 16; v_shift = 4; }
	else if (src_height >= 8  * dst_height)		{ PreVerRatio_xx = 8;  v_shift = 3; }
	else if (src_height >= 4  * dst_height)		{ PreVerRatio_xx = 4;  v_shift = 2; }
	else if (src_height >= 2  * dst_height)		{ PreVerRatio_xx = 2;  v_shift = 1; }
	else						{ PreVerRatio_xx = 1;  v_shift = 0; }

	PreDstHeight_xx = src_height / PreVerRatio_xx;
	MainVerRatio_xx = (src_height << 8) / (dst_height << v_shift);
	SHfactor_xx     = 10 - (h_shift + v_shift);

	/* Make sure the scaler can handle this zoom factor */
	if (dst_width > src_width && (src_width / PreHorRatio_xx) > 320)
		return -EINVAL;

	/* If we get here, we've got valid settings, so program scaler */
	cam->vCICOSCPRERATIO = (PreHorRatio_xx << S3C2413_CICOSCPRERATIO_PREHORRATIO_SHIFT) |
		(PreVerRatio_xx << S3C2413_CICOSCPRERATIO_PREVERRATIO_SHIFT) |
		(SHfactor_xx << S3C2413_CICOSCPRERATIO_SHFACTOR_SHIFT);
	writel(cam->vCICOSCPRERATIO, S3C2413_CICOSCPRERATIO);

	cam->vCICOSCPREDST = (PreDstWidth_xx << S3C2413_CICOSCPREDST_PREDSTWIDTH_SHIFT) |
		(PreDstHeight_xx << S3C2413_CICOSCPREDST_PREDSTHEIGHT_SHIFT);
	writel(cam->vCICOSCPREDST, S3C2413_CICOSCPREDST);

	val =	(MainHorRatio_xx << S3C2413_CICOSCCTRL_MAINHORRATIO_SHIFT) |
		(MainVerRatio_xx << S3C2413_CICOSCCTRL_MAINVERRATIO_SHIFT) |
		S3C2413_CICOSCCTRL_COSCALERSTART;				/* XXX No docs on this flag?? */

	/* Check if we're scaling up (H and V) */
	if (dst_width >= src_width) val |= S3C2413_CICOSCCTRL_SCALEUPH;
	if (dst_height >= src_height) val |= S3C2413_CICOSCCTRL_SCALEUPV;

	/* Set the scaler bit if used. */
	cam->vCIIMGCPT |= S3C2413_CIIMGCPT_IMGCPTEN_COSC;

	/* Write the remaining values. */
	cam->vCICOSCCTRL = val;
	writel(val, S3C2413_CICOSCCTRL);
	writel( cam->vCIIMGCPT, S3C2413_CIIMGCPT ); 
	return 0;
}

static int
cam_set_format(struct s3c_cam *cam, struct v4l2_format *fmt)
{
	int rc;

	/* Check this is capture. */
	if( fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	/* Try format, and exit if fails */
	rc = cam_get_format(cam,fmt);
	if (rc) return rc;

	/* Store selected format for future reference */
	cam->currfmt = *fmt;

	/* Program the scaler/cropping function. */
	return cam_set_scaler( cam );
}

int ProgramBurstSize( struct s3c_cam *cam )
{
	unsigned short int	nr_datawordsY=0;
	unsigned short int	nr_datawordsUV=0;
	unsigned short int	MainBurstY=16;
	unsigned short int	RemBurstY=0;
	unsigned short int	MainBurstUV=16;
	unsigned short int	RemBurstUV=0;
	int			done=0;
	int			is_interleaved=0;

	/* Determine the linesize. */
	switch( cam->currfmt.fmt.pix.pixelformat )
	{
		case V4L2_PIX_FMT_RGB24:
			/* 32 bits / pixel */
			is_interleaved=1;
			nr_datawordsY=cam->currfmt.fmt.pix.width;
			nr_datawordsUV=0;
			break;

		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_YUYV:
			/* 32 bits / 2 pixel */
			is_interleaved=1;
			nr_datawordsY=cam->currfmt.fmt.pix.width/2;
			nr_datawordsUV=0;
			break;

		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YUV422P:
			is_interleaved=0;
			/* 32 bits / 4 pixel (planar, count only Y plane) */
			nr_datawordsY=cam->currfmt.fmt.pix.width/4;

			/* 32 bits / 8 pixel (planar, count only U or V plane) */
			nr_datawordsUV=cam->currfmt.fmt.pix.width/8;
			break;

		default:
			/* Shouldn't get here. */
			return -1;
	}

	/* Now try out which burst we can use. Keep in account that Main burst ought to be */
	/* 4,8,16 and remained burst ought to be 2,4,8,16. */
	done=0;
	do
	{
		/* Calculate what the remained burst would be. */
		RemBurstY=nr_datawordsY % MainBurstY;
		if( !RemBurstY ) RemBurstY=MainBurstY;

		/* Check if it's a valid value. */
		switch( RemBurstY )
		{
			case 2 :
			case 4 :
			case 8 :
			case 16:
				/* Valid result found. */
				done=1;
				break;

			default:
				/* Result is invalid. Try again. */
				MainBurstY>>=1;
				done=0;
				break;
		}
	} while( (MainBurstY >= 4) && !done );

	/* Check if the result is valid. */
	if( (MainBurstY < 4) || (RemBurstY < 2) )
		return -2;

	/* If this is a planar format, calculate for the U/V planes. For interleaved, there is a fixed ratio. */
	if( !is_interleaved )
	{
		/* Now do the same for the U/V planes. */
		done=0;
		do
		{
			/* Calculate what the remained burst would be. */
			RemBurstUV=nr_datawordsUV % MainBurstUV;
			if( !RemBurstUV ) RemBurstUV=MainBurstUV;

			/* Check if it's a valid value. */
			switch( RemBurstUV )
			{
				case 2 :
				case 4 :
				case 8 :
				case 16:
					/* Valid result found. */
					done=1;
					break;

				default:
					/* Result is invalid. Try again. */
					MainBurstUV>>=1;
					done=0;
					break;
			}
		} while( (MainBurstUV >= 4) && !done );

		/* Check if the result is valid. */
		if( (MainBurstUV < 4) || (RemBurstUV < 2) )
			return -3;
	}

	/* When we get here, we got valid values. */
	if( is_interleaved )
	{
		cam->vCICOCTRL=((MainBurstY/2) << S3C2413_CICOCTRL_YBURST1_SHIFT) |
		               ((RemBurstY/2) << S3C2413_CICOCTRL_YBURST2_SHIFT) |
		               ((MainBurstY/4) << S3C2413_CICOCTRL_CBURST1_SHIFT) |
		               ((RemBurstY/4) << S3C2413_CICOCTRL_CBURST2_SHIFT) |
		               (cam->vCICOCTRL & S3C2413_CICOCTRL_LASTIRQEN);
	}
	else
	{
		cam->vCICOCTRL=(MainBurstY << S3C2413_CICOCTRL_YBURST1_SHIFT) |
		               (RemBurstY << S3C2413_CICOCTRL_YBURST2_SHIFT) |
		               (MainBurstUV << S3C2413_CICOCTRL_CBURST1_SHIFT) |
		               (RemBurstUV << S3C2413_CICOCTRL_CBURST2_SHIFT) |
		               (cam->vCICOCTRL & S3C2413_CICOCTRL_LASTIRQEN);
	}

	/* Done. Now write. */
	writel( cam->vCICOCTRL, S3C2413_CICOCTRL );
	return 0;
}

int
cam_stream_on(struct s3c_cam *cam)
{
	unsigned long int	val;
	unsigned long int	count=0;
	unsigned long int	index=0;
	
	/* Lock. */
	spin_lock(&cam->lock);

	/* First reset camera interface, since we might have changed mode or format. */
	cam->vCIGCTRL=readl( S3C2413_CIGCTRL );
	writel( cam->vCIGCTRL | S3C2413_CIGCTRL_SWRST, S3C2413_CIGCTRL );

	/* Wait for 1ms for the interface to reset. Note: We do not reset the camera */
	/* itself. It is still in the correct mode/settings. */
	mdelay( 1 );
	writel( cam->vCIGCTRL & ~(S3C2413_CIGCTRL_SWRST), S3C2413_CIGCTRL ); 

	/* Reprogram the scaler, as this one will have lost its settings. */
	if( cam_set_scaler( cam ) != 0 )
	{
		spin_unlock( &cam->lock );
		return -EINVAL;
	}

	/* Switch off any possible test modes. Make sure VSYNC polarity inversion is enabled. */
	val = readl(S3C2413_CIGCTRL) & ~S3C2413_CIGCTRL_TEST_MASK;
	val |= S3C2413_CIGCTRL_TEST_NORMAL;
	writel(val, S3C2413_CIGCTRL);
	cam->vCIGCTRL = val;
	
	/* Clear any overflows */
	cam->vCIWDOFST|=S3C2413_CIWDOFST_CLROVCOFIY | S3C2413_CIWDOFST_CLROVCOFICB | S3C2413_CIWDOFST_CLROVCOFICR;
	writel( cam->vCIWDOFST, S3C2413_CIWDOFST );
	cam->vCIWDOFST&=~(S3C2413_CIWDOFST_CLROVCOFIY | S3C2413_CIWDOFST_CLROVCOFICB | S3C2413_CIWDOFST_CLROVCOFICR);
	writel( cam->vCIWDOFST, S3C2413_CIWDOFST );
	
	/* Set source format for codec */
	cam->vCISRCFMT = S3C2413_CISRCFMT_FMT_BT601 | /* OV sensor feeds us this */
		S3C2413_CISRCFMT_UVOFF_NONE | /* It has "proper' YUV, not the +128 offsetted kind */
		(cam->defres.width << S3C2413_CISRCFMT_SRCHSZ_SHIFT) |
		(cam->defres.height << S3C2413_CISRCFMT_SRCVSZ_SHIFT);
	writel(cam->vCISRCFMT, S3C2413_CISRCFMT);

	/* Set output format for codec. Ensure the FLIP bits are copied from previous settings, to ensure these */
	/* pass between consecutive calls of stream on/stream off. */
	val = S3C2413_CICOTRGFMT_IN422 | /* Input for codec is YUV 4:2:2 */
		(cam->currfmt.fmt.pix.width << S3C2413_CICOTRGFMT_TRGHSZ_SHIFT) |
		(cam->currfmt.fmt.pix.height << S3C2413_CICOTRGFMT_TRGVSZ_SHIFT) |
		(cam->vCICOTRGFMT & S3C2413_CICOTRGFMT_FLIPMD_MASK);

	switch(cam->currfmt.fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_RGB565:	val |= S3C2413_CICOTRGFMT_OUT422 | S3C2413_CICOTRGFMT_INTERLEAVE;       break;
		case V4L2_PIX_FMT_RGB555:	val |= S3C2413_CICOTRGFMT_OUT422 | S3C2413_CICOTRGFMT_INTERLEAVE;       break;
		case V4L2_PIX_FMT_RGB24:	val |= S3C2413_CICOTRGFMT_OUT422 | S3C2413_CICOTRGFMT_INTERLEAVE;	break;
		case V4L2_PIX_FMT_YUV422P:	val |= S3C2413_CICOTRGFMT_OUT422;					break;
		case V4L2_PIX_FMT_YUV420:	val &= ~(S3C2413_CICOTRGFMT_OUT422);					break;
		case V4L2_PIX_FMT_YUYV:		val |= S3C2413_CICOTRGFMT_OUT422 | S3C2413_CICOTRGFMT_INTERLEAVE;	break;
        	default:			spin_unlock(&cam->lock); return -EINVAL;
	}
	cam->vCICOTRGFMT = val;
	writel(val, S3C2413_CICOTRGFMT);

	/* Setup CAMIF DMA */
	if( ProgramBurstSize( cam ) )
		return -EINVAL;

	cam->vCICOTAREA = cam->currfmt.fmt.pix.width * cam->currfmt.fmt.pix.height;
	writel(cam->vCICOTAREA, S3C2413_CICOTAREA );

	/* Select the correct output. Note: val is used as bitmask, and the ImgCptEn_Cosc bit is copied from the register. */
	val=cam->vCIIMGCPT & S3C2413_CIIMGCPT_IMGCPTEN_COSC;
	switch(cam->currfmt.fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_RGB24:	val |= S3C2413_CIIMGCPT_RGB | S3C2413_CIIMGCPT_RGBFMT_24BIT; break;
                case V4L2_PIX_FMT_RGB565:	val |= S3C2413_CIIMGCPT_RGB | S3C2413_CIIMGCPT_RGBFMT_16BIT; break;
                case V4L2_PIX_FMT_RGB555:	val |= S3C2413_CIIMGCPT_RGB | S3C2413_CIIMGCPT_RGBFMT_16BIT; break;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YUV422P:	val |= S3C2413_CIIMGCPT_YUV | S3C2413_CIIMGCPT_RGBFMT_16BIT; break;
		default:			spin_unlock( &cam->lock ); return -EINVAL;
	}
	cam->vCIIMGCPT=val;

	/* Determine how many buffers we can use for this resolution. */
	cam->slots_in_use=(__u16) (CAMI_BUFFER_SIZE/cam->currfmt.fmt.pix.sizeimage);
	switch( cam->slots_in_use )
	{
		case 0 :
			/* Should not happen, but just in case. */
			spin_unlock( &cam->lock );
			return -EINVAL;
			
		case 1 :
		case 2 :
		case 4 :
			/* No need to do anything. */
			break;

		case 3 :
			/* For sake of simplicity, we don't do 3 buffers, we do 2 then. */
			cam->slots_in_use=2;
			break;

		default :
			/* More than 4 buffers are possible. However, our maximum is 4. */
			cam->slots_in_use=4;
			break;
	}

	/* Start the stream. */
	for( count=0; count < 16; count+=4 )
	{
		/* Which buffer index? */
		index=((count/4) % cam->slots_in_use) * ((unsigned long int) cam->currfmt.fmt.pix.sizeimage);

		/* Write Y. */
		writel( cam->dma_ptr + index, S3C2413_CICOYSA1 + count );
		switch( cam->currfmt.fmt.pix.pixelformat )
		{
			case V4L2_PIX_FMT_YUV422P:
			{
				unsigned long int	uoffset=cam->currfmt.fmt.pix.width*cam->currfmt.fmt.pix.height;
				/* 1 byte Y, 4 bits U, 4 bits V */
				writel( cam->dma_ptr + index + uoffset, S3C2413_CICOCBSA1 + count );
				writel( cam->dma_ptr + index + ((uoffset*3)/2), S3C2413_CICOCRSA1 + count );
				break;
			}

			case V4L2_PIX_FMT_YUV420:
			{
				unsigned long int	uoffset=cam->currfmt.fmt.pix.width*cam->currfmt.fmt.pix.height;
				/* 1byte Y, 2 bits U, 2 bits V */
				writel( cam->dma_ptr + index + uoffset, S3C2413_CICOCBSA1 + count );
				writel( cam->dma_ptr + index + ((uoffset*5)/4), S3C2413_CICOCRSA1 + count );
				break;
			}

			case V4L2_PIX_FMT_RGB565:
			case V4L2_PIX_FMT_RGB555:
			case V4L2_PIX_FMT_RGB24:
			case V4L2_PIX_FMT_YUYV:
			{
				/* Not used registers. Set to 0. */
				writel( 0, S3C2413_CICOCBSA1 + count );
				writel( 0, S3C2413_CICOCRSA1 + count );
				break;
			}
		}
	}

	/* Flag that we're streaming. Make sure the stopped flag is cleared (just in case). */
	/* Note that we need to set these flags before the camera is started, other wise the*/
	/* camera might be stopped again when the interrupt fires before these flags are set*/
	cam->flags|=S3C_CAM_STREAMING;
	cam->flags|=S3C_CAM_STATECHANGE;

	/* Flag the LASTIRQ bit if we only have space for one buffer. This to ensure that enough time is left */
	/* to copy the frame. This is to workaround a problem that the camera interface starts streaming the  */
	/* data for the next frame while we are still copying the current frame. */
	if( cam->slots_in_use == 1 )
		writel( cam->vCICOCTRL | S3C2413_CICOCTRL_LASTIRQEN, S3C2413_CICOCTRL );

	/* Now start the camera. */
	val=cam->vCIIMGCPT | S3C2413_CIIMGCPT_IMGCPTEN | S3C2413_CIIMGCPT_DMAEN;
	writel( val,  S3C2413_CIIMGCPT );
	cam->vCIIMGCPT=val;

	/* Restore IRQs. */
        spin_unlock(&cam->lock);
	return 0;
}

int
cam_stream_off(struct s3c_cam *cam)
{
	/* Lock */
        spin_lock(&cam->lock);
	disable_cam_irq( cam );

	/* Flag we stopped streaming. */
	cam->flags&=~(S3C_CAM_STREAMING);

	/* The camera interface will generate at least one more IRQ before it's stopped so wait for the interface */
	/* to stop. */
	while( !(cam->flags & S3C_CAM_STATECHANGE) )
	{
		/* Ensure IRQs can happen again. */
		enable_cam_irq( cam );
		spin_unlock( &cam->lock ); 

		/* Wait for the camera to be stopped. */
		if( wait_event_interruptible(cam->inq, (cam->flags & S3C_CAM_STATECHANGE) ) )
                        return -ERESTARTSYS;

		/* Event happened or condition reached. Get the lock and disable IRQs. */
		spin_lock( &cam->lock );
		disable_cam_irq( cam );
	}

	/* Clear the flag. It's stopped. */
	cam->flags&=~(S3C_CAM_STATECHANGE);

	/* Done. */
	enable_cam_irq( cam );
        spin_unlock(&cam->lock);
	return 0;
}

static int
cam_crop_capabilities(struct s3c_cam *cam, struct v4l2_cropcap* cap)
{
	if (!cap) return -EFAULT;
	if (cap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EINVAL;

	/* Set the bounds; area we can capture from */
	cap->bounds.top = 0;
	cap->bounds.left = 0;
	cap->bounds.width = cam->defres.width;;
	cap->bounds.height = cam->defres.height;

	/* Default rectangle */
	cap->defrect = cam->croprect;

	/* set the pixelratio; as we're doing all, set 1:1 */
	cap->pixelaspect.numerator = 1; 
	cap->pixelaspect.denominator = 1;

	return 0;
}

static int
cam_get_crop(struct s3c_cam *cam, struct v4l2_crop* crop)
{
	if (!crop) return -EFAULT;
	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EINVAL;

	crop->c = cam->croprect;

	return 0;
}

static int
cam_set_crop(struct s3c_cam *cam, struct v4l2_crop* crop)
{
	__s32			tmp_res;
	__s32			offperc;

	if (!crop) return -EFAULT;
	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EINVAL;

	/* Ensure that the format specified in fmt has resolutions that are multiples */
	/* of 8. This is a requirement for the camera interface. */

	/* First try to get the width as close as possible. */
	if( crop->c.width % 8 )
	{
		/* First calculate how far the window is offset into the total image, we need to */
		/* keep this similar so that the zoomed in picture doesn't change too much. */
		offperc=1000*crop->c.left/(cam->defres.width - crop->c.width);

		/* Calculate the new width. */
		tmp_res=8*(crop->c.width/8);
		if( (crop->c.width - tmp_res) > 4 )
			tmp_res+=8;

		/* Calculate the new horizontal offset. */
		crop->c.left=offperc * (cam->defres.width - tmp_res) / 1000;
		crop->c.width=tmp_res;
	}

	/* Now try to get the height as close as possible. */
	if( crop->c.height % 8 )
	{
		/* First calculate how far the window is offset into the total image, we need to */
		/* keep this similar so that the zoomed in picture doesn't change too much. */
		offperc=1000*crop->c.top/(cam->defres.height - crop->c.height);

		/* Calculate the new height. */
		tmp_res=8*(crop->c.height/8);
		if( (crop->c.height - tmp_res) > 4 )
			tmp_res+=8;

		/* Calculate the new vertical offset. */
		crop->c.top=offperc * (cam->defres.height - tmp_res) / 1000;
		crop->c.height=tmp_res;
	}

	/* Now make sure the offsets are also aligned on 8 pixel boundary. */
	if( crop->c.left % 8 )
		crop->c.left=8*(crop->c.left/8);
		
	if( crop->c.top % 8 )
		crop->c.top=8*(crop->c.top/8);

	cam->croprect = crop->c;

	/* Program the scaler/cropping function. */
	return cam_set_scaler( cam );
}

int 
cam_set_defres( struct s3c_cam *cam, struct v4l2_rect *res )
{
	struct ovcamchip_window		ovwin;

	/* Check if we are streaming. No changing of this while streaming. */
	if( cam->flags & S3C_CAM_STREAMING  )
		return -EBUSY;

	/* Offsets are always 0. */
	if( res->left || res->top )
		return -EINVAL;

	/* Ensure the right resolution is set. */
	switch( res->width )
	{
		case 1280 :
			if( res->height == 1024 ) break;
			else return -EINVAL;
		case 640 :
			if( res->height == 480 ) break;
			else return -EINVAL;
		case 320 :
			if( res->height == 240 ) break; 
			else return -EINVAL;
		case 160 :
			if( res->height == 120 ) break; 
			else return -EINVAL;
		case 352 :
			if( res->height == 288 ) break;
			else return -EINVAL;
		case 176 :
			if( res->height == 144 ) break; 
			else return -EINVAL;
		default :
			/* Illegal resolution. */
			return -EINVAL;
	}

	/* Set the default resolution. */
	memcpy( &(cam->defres), res, sizeof( cam->defres ) );
	
	/* Need to do the ovcamchip also. */
	ovwin.width=cam->defres.width;
	ovwin.height=cam->defres.height;
        ovwin.x=cam->defres.left;
	ovwin.y=cam->defres.top;
        ovwin.quarter = 0;
        ovwin.clockdiv = 4;
	switch( cam->currfmt.fmt.pix.pixelformat )
	{
               case V4L2_PIX_FMT_YUV422P:
               case V4L2_PIX_FMT_YUV420:
	       case V4L2_PIX_FMT_YUYV:
               case V4L2_PIX_FMT_RGB24:
               case V4L2_PIX_FMT_RGB565:
			/* Note that we program YUV422 for all output formats. The S3C2413 does the conversion from */
			/* YUV422 to the designated formats. It doesn't accept any other form of input data. */
			ovwin.format=VIDEO_PALETTE_YUV422;
                        break;

		default:
			return -EINVAL;
	}

	/* Reinitialize the mode also for the Camera itself (the I2C registers). */
        if( cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CMD_S_MODE, &ovwin ) != 0 )
	{
                printk( "Unable to reset OVCAMCHIP sensor's mode!\n");
		return -EINVAL;
	}
	
	/* Now make sure the crop region is within boundaries. */
	cam->croprect.top=0;
	cam->croprect.left=0;
	cam->croprect.width=cam->defres.width;
	cam->croprect.height=cam->defres.height;
	cam->currfmt.fmt.pix.width=cam->defres.width;
	cam->currfmt.fmt.pix.height=cam->defres.height;
	if( cam_set_scaler( cam ) != 0 )
	{
		printk( "Unable to reset cropping settings.\n" );
		return -EINVAL;
	}
	return 0;
}

static int
cam_get_defres( struct s3c_cam *cam, struct v4l2_rect *res )
{
	/* Copy default res setting. */
	memcpy( res, &(cam->defres), sizeof( *res ) );
	return 0;
}

int
s3c_cam_do_ioctl(struct inode *inode, struct file *file, 
		 unsigned int cmd, void *arg)
{
	struct video_device	*vdev = video_devdata(file);
	struct s3c_cam		*cam;
	struct v4l2_format	*fmt=(struct v4l2_format *) arg;

	if (vdev == NULL) return -EFAULT;
	cam = video_get_drvdata(vdev);
	if (cam == NULL) return -EFAULT;

	switch(cmd) {
		/* we support Video For Linux _2_ only! */
		case VIDIOC_QUERYCAP:	return cam_query_capabilities(cam,arg);

		case VIDIOC_CROPCAP:	return cam_crop_capabilities(cam,arg);
		case VIDIOC_G_CROP:	return cam_get_crop(cam,arg);
		case VIDIOC_S_CROP:	return cam_set_crop(cam,arg);

		case VIDIOC_ENUMINPUT:	return cam_enumerate_inputs(cam,arg);
		case VIDIOC_G_INPUT:	return cam_get_input(cam,arg);
		case VIDIOC_S_INPUT:	return cam_set_input(cam,arg);

		case VIDIOC_QUERYCTRL:	return cam_query_control(cam,arg);
		case VIDIOC_G_CTRL:	return cam_get_control(cam,arg);
		case VIDIOC_S_CTRL:	return cam_set_control(cam,arg);

		case VIDIOC_ENUM_FMT:	return cam_enumerate_formats(cam,arg);
		case VIDIOC_G_FMT:
			/* Make sure the right format is set. Otherwise the function fails. It should not. */
			if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			{
				memcpy( arg, &(cam->currfmt), sizeof( cam->currfmt ) );
				return 0;
			}
			else return -EINVAL;

		case VIDIOC_TRY_FMT:	return cam_try_format(cam,arg);
		case VIDIOC_S_FMT:	return cam_set_format(cam,arg);

		case VIDIOC_STREAMON:	return cam_stream_on(cam);
		case VIDIOC_STREAMOFF:	return cam_stream_off(cam);
		case VIDIOC_PRIV_SETDEFRES:
					return cam_set_defres( cam, arg );
		case VIDIOC_PRIV_GETDEFRES:
					return cam_get_defres( cam, arg );
		default:
			/* Not supported by the S3C2413 part of the driver. Pass on to OV9655. */
			if (cam->ovclient && cam->ovclient->driver->command)
				return cam->ovclient->driver->command(cam->ovclient, cmd, arg);
	}

	return -ENOSYS;
}

void
s3c_cam_setup_defaults(struct s3c_cam* cam)
{
	cam->flags				= 0;
	cam->currfmt.type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->currfmt.fmt.pix.pixelformat	= V4L2_PIX_FMT_RGB565;
	cam->currfmt.fmt.pix.field		= V4L2_FIELD_NONE;
	cam->currfmt.fmt.pix.bytesperline	= OVCC_MAX_WIDTH*2;
	cam->currfmt.fmt.pix.sizeimage		= cam->currfmt.fmt.pix.bytesperline * OVCC_MAX_HEIGHT;
	cam->currfmt.fmt.pix.colorspace		= V4L2_COLORSPACE_SMPTE170M;
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cam->curr_divider=((readl( S3C2410_CLKDIVN ) & S3C2412_CLKDIVN_CAMCLKDIV_MASK) >> 16) + 1;
#endif

	/* Done. */
	return;
}
