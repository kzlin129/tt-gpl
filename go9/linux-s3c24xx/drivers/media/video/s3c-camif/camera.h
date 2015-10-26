/*
 * Camera driver for s3c241x camera interface, with ov camchip.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 */

#ifndef __DRIVERS_BARCELONA_CAMERA_CAMERA_H
#define __DRIVERS_BARCELONA_CAMERA_CAMERA_H

#include <linux/videodev.h>
#include <linux/i2c.h>
#include <asm/hardware/clock.h>

#define CAMERA_DRIVER_NAME	"s3c241x-cam"
#define CAMERA_VERSION_CODE	KERNEL_VERSION(1,0,0)

#define DEF_CAM_I2C_ADDR	60

/*
 * These IOCTLs allow the default viewing window (see chapter 1.10 cropping and scaling) to be set.
 * Default V4L2 expects this to be a fixed number from which all cropping/scaling is done.
 * Note that even though the parameter is a struct v4l2_rect, the top and left fields should ALWAYS
 * be 0.
 */
#define VIDIOC_PRIV_SETDEFRES	_IOWR  ('V', 192, struct v4l2_rect)
#define VIDIOC_PRIV_GETDEFRES	_IOR  ('V', 193, struct v4l2_rect)

/*
 * Resolution limits of the CAMI on the s3c2413. Please note that the attached camera
 * could have a (much) lower limit.
 */

#define CAMI_MAX_WIDTH		1600
#define CAMI_MAX_HEIGHT		1200

/*
 * Number of slots available in the camera interface.
 */
#define CAMI_MAX_SLOTS		4

/*
 * Resolution limits of the OV module used. This should be queried from the ovcamchip module,
 * but alas, it doesn't have an interface for that (yet).
 */

#define OVCC_MAX_WIDTH		1280
#define OVCC_MAX_HEIGHT		1024

/* Buffer size of temporary buffer. */
#define CAMI_BUFFER_SIZE	(OVCC_MAX_HEIGHT * OVCC_MAX_WIDTH * sizeof( __u16 ))

/* Flags. */
#define S3C_CAM_STREAMING	0x00000001	/* Set to 1 when the camera is streaming (stream_on) or 0 if not. */
#define S3C_CAM_STATECHANGE	0x00000002	/* Set by the IRQ handler when the camera has just been stopped or*/
						/* started, and the IRQ handler needs to take action on this. */
/* Clock frequency settings. */
#define S3C_CLOCKPOLICY_MIN_FREQ	24000000
						/* Minimum frequency necessary for allowing the camera to work */
#define S3C_CLOCKPOLICY_MAX_FREQ	27000000
						/* Maximum frequency necessary for allowing the camera to work */

/* General device information */
struct s3c_cam {
	struct video_device* vdev;		/* V4L2 device */
	struct platform_device* pdev;		/* CAMIF device */
	struct clk* clk;			/* CAMCLK */
	struct i2c_client* ovclient;		/* I2C client for ovcamchip device */
	__u16 num_buffers;			/* Number of allocated buffers at this time */
	struct v4l2_format currfmt;		/* Current format as selected by app */
	struct v4l2_rect croprect;		/* Currently set croprect */
	struct v4l2_rect defres;		/* For private VIDIOC_PRIV_SETDEFRES/VIDIOC_PRIV_GETDEFRES IOCTL. */
						/* This sets the default resolution the camera outputs to the cpu. */
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	struct notifier_block cpu_policy_notifier;
	struct notifier_block cpu_transition_notifier;
						/* Notifiers for the CPU speed changes. */
	__u32 curr_divider;			/* Current PCLK prescaler value. */
#endif
        wait_queue_head_t inq;          	/* Wait queue for reading */
	spinlock_t lock;			/* For protecting in-interrupt used portions of this structure */
	__u16 num_done_buffers;			/* Number of filled buffers ready to be dequeued by userland */
	__u16 open;				/* open count => should never be allowed to go above 1! */
	__u16 irq_num;				/* IRQ# of CAMI interrupt */
	__u16 slots_in_use;			/* Number of slots in use */
	__u32 flags;				/* Flags in use for the driver. */
	unsigned char	*kernel_ptr;		/* Base kernel buffer. */
	dma_addr_t	dma_ptr;		/* Base DMA address. */
	__u32	vCIWDOFST,			/* Contents of registers determined at 'stream on' time. required */
		vCIGCTRL,			/*	for reprogramming registers at interrupt time */
		vCIDOWSFT2,
		vCICOSCPRERATIO,
		vCICOSCPREDST,
		vCICOSCCTRL,
		vCISRCFMT,
		vCICOTAREA,
		vCICOCTRL,
		vCICOTRGFMT,
		vCIIMGCPT;
	struct
	{
		struct page	**page_array;	/* Array to pass along the list of userpages from the read routine. */
						/* This array is always allocated to the maximum number of pages possible */
						/* for the maximum resolution of the camera. */
		int		nr_pages;	/* Number of pages currently in the page_array. */
		void __user	*user_ptr;	/* Pointer to the buffer in user space. E.G A USERSPACE POINTER. */ 
	} userspace;
};

/* Camera file operations (from driver.c) */
extern struct file_operations s3c_camif_fops;

/* Video4Linux centered functions (from cam-v4l.c) */
int s3c_cam_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *arg);
void s3c_cam_setup_defaults(struct s3c_cam* cam);

/* I2C access routines for sensor */
int cam_read_reg(struct s3c_cam* cam, int reg);
void cam_write_reg(struct s3c_cam* cam, int reg, int val);

/* Camera default resolution reprogram routine. */
int cam_set_defres( struct s3c_cam *cam, struct v4l2_rect *res );
#endif /* __DRIVERS_BARCELONA_CAMERA_CAMERA_H */

