/* linux/drivers/media/video/samsung/rp/s3c_rp.h
 *
 * Header file for Samsung S3C6410 Renderer pipeline driver 
 *
 * Jonghun Han, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _S3C_RP_H
#define _S3C_RP_H

#ifdef __KERNEL__
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#endif

/*
 * C O M M O N   D E F I N I T I O N S
 *
*/
#define S3C_RP_NAME				"s3c-rp"
#define S3C_RP_MINOR				10

#define S3C_RP_PP_CLK_NAME			"rp_pp"
#define S3C_RP_ROT_CLK_NAME			"rp_rot"

#define	S3C_RP_YUV_SRC_MAX_WIDTH		720
#define	S3C_RP_YUV_SRC_MAX_HEIGHT		576

#define S3C_RP_RESOURCE_PP			0
#define S3C_RP_RESOURCE_ROT			1

#define S3C_RP_BUFF_NUM				3

#define RP_RESERVED_MEM_ADDR_PHY		(unsigned int)s3c_get_media_memory(S3C_MDEV_RP)
#define RP_RESERVED_MEM_SIZE			(unsigned int)s3c_get_media_memsize(S3C_MDEV_RP)

#define S3C_RP_CROP_DEF_WIDTH			S3C_RP_SRC_MAX_WIDTH
#define S3C_RP_CROP_DEF_HEIGHT			S3C_RP_SRC_MAX_HEIGHT

#define S3C_RP_PP_TIMEOUT			100
#define S3C_RP_ROT_TIMEOUT			100	// normally 800 * 480 * 2 rotation takes about 20ms
#define S3C_RP_DQUEUE_TIMEOUT			200

/* S flag: global status flags */
#define S3C_RP_FLAG_READY_TO_STREAMON		0x0001
#define S3C_RP_FLAG_STREAMON			0x0002
#define S3C_RP_FLAG_READY_TO_STREAMOFF		0x0004
#define S3C_RP_FLAG_STREAMOFF			0x0008

#define info(args...)	do { printk(KERN_INFO S3C_RP_NAME ": " args); } while (0)
#define err(args...)	do { printk(KERN_ERR  S3C_RP_NAME ": " args); } while (0)

#define TRUE 1
#define FALSE 0

/*
 * E N U M E R A T I O N S
 *
*/
enum s3c_rp_status {
	RP_STREAMOFF,
	RP_READY_ON,
	RP_STREAMON,
	RP_READY_OFF,
};

enum s3c_pp_addr {
	PP_CURRENT,
	PP_NEXT,		
};

enum s3c_rot_status_t {
	ROT_IDLE,
	ROT_RUNNING,
	ROT_ABORT,	
};

enum s3c_rp_rot_t {
	ROT_X_FLIP = 0x2,
	ROT_Y_FLIP = 0x3,
	ROT_0 = 0x0,
	ROT_90 = 0x4,
	ROT_180 = 0x8,
	ROT_270 = 0xC,
};

enum s3c_rp_format_t {
	FORMAT_RGB888,
	FORMAT_YCBCR420,
	FORMAT_YCBCR422,
};

enum s3c_rp_buf_state {
	BUF_DQUEUED,
	BUF_QUEUED,
	BUF_RUNNING,
	BUF_DONE,
};

enum s3c_rp_fifo_state {
	FIFO_CLOSE,
	FIFO_SLEEP,		
};

/*
 * Renderer Pipeline   S T R U C T U R E S
 *
*/

struct s3c_rp_scaler_cfg {
	unsigned int prescale_h_ratio;
	unsigned int prescale_v_ratio;
	unsigned int h_shift;
	unsigned int v_shift;
	unsigned int sh_factor;
	unsigned int prescale_dst_width;
	unsigned int prescale_dst_height;
	unsigned int dx;
	unsigned int dy;
};

/*
 * struct s3c_rp_addr
 * @phys_rgb:	physical start address of rgb buffer
 * @phys_y:	physical start address of y buffer
 * @phys_cb:	physical start address of u buffer
 * @phys_cr:	physical start address of v buffer 
 * @virt_y:	virtual start address of y buffer
 * @virt_rgb:	virtual start address of rgb buffer
 * @virt_cb:	virtual start address of u buffer
 * @virt_cr:	virtual start address of v buffer 
*/
struct s3c_rp_addr {
	union {
		dma_addr_t	phys_rgb;
		dma_addr_t	phys_y;		
	};
	dma_addr_t		phys_cb;
	dma_addr_t		phys_cr;
};

struct s3c_rp_pp_addr {
	unsigned int	phys_y_start;
	unsigned int	phys_cb_start;
	unsigned int	phys_cr_start;
	
	unsigned int	phys_y_end;
	unsigned int	phys_cb_end;
	unsigned int	phys_cr_end;

	unsigned int	phys_y_offset;
	unsigned int	phys_cb_offset;
	unsigned int	phys_cr_offset;
};

struct s3c_rp_buf {
	struct s3c_rp_addr	buf_addr;
	unsigned int		buf_length;	
	enum s3c_rp_buf_state	buf_state;
	unsigned int		buf_flag;
 };

struct s3c_rp_buf_index {
	unsigned int		index;
	struct s3c_rp_buf_index	*next_buf_idx;
};

struct s3c_rp_v4l2 {
	struct v4l2_pix_format	video_out_fmt;
	struct v4l2_window	video_overlay_fmt;	
	struct v4l2_rect	crop_bounds;
	struct v4l2_rect	crop_defrect;
	struct v4l2_fract	pixelaspect;	
	struct v4l2_rect	crop_rect;
};


struct s3c_rp_fimd_info	{
	unsigned int		h_res;
	unsigned int		v_res;
	int			(*open_fifo)(int id, int ch, int (*do_priv)(void *), void *param);
	int			(*close_fifo)(int id, int (*do_priv)(void *), void *param, int sleep);
	wait_queue_head_t	wq;	
};

struct s3c_rp_pp_idx {
	int	prev;
	int	run;
	int	next;
};

struct s3c_rp_pp {
	char			clk_name[16];
	struct clk		*clock;
	void __iomem		*regs;
	int			irq;

	struct s3c_rp_pp_idx	buf_idx;
};

struct s3c_rp_rot {
	char			clk_name[16];
	struct clk		*clock;
	void __iomem		*regs;
	int			irq;	

	int			run_idx;
	enum s3c_rp_rot_t	degree;
	enum s3c_rot_status_t	status;
};

struct s3c_rp_spin {
	spinlock_t			lock_in;
	spinlock_t			lock_inside;
	spinlock_t			lock_out;	
};

struct s3c_rp_waitq {
	wait_queue_head_t		pp;
	wait_queue_head_t		rot;	
};

struct s3c_rp_buf_info {
	dma_addr_t			reserved_mem_start;
	unsigned int			num;
	unsigned int			requested;
};
/*
 * struct s3c_rp_control: abstraction for Renderer Pipeline
 * @name:		name for video_device

 * @lock:		mutex lock
 * @waitq:		waitqueue
 * @in_use:		1 when resource is occupied

 * @dev:		platform data
 * @vd:			video_device
 * @v4l2:		v4l2 info

 * @stream_status:	renderer pipeline stream status
 * @rot_degree:		renderer pipeline rotation degree
 * @rot_status:

 * @reserved_mem_start:	start address of reserved memory which can be used for src/dst buffer
 * @video_out_buf_num:	requested buffer number by VIDIOC_REQBUFS
 * @buf_requested:	whether VIDIOC_REQBUFS called or not
 * @pp_run_buf_idx:	buffer index which is running on post processor
 * @rot_run_buf_idx:	buffer index which is running on rotator
 * @user_buf:		buffer for rotator or pp input
 * @driver_buf:		buffer for rotator output
 * @incoming_queue:	incoming queue of pipeline

 * @clk_name_pp:	post processor clock name
 * @clk_name_rot:	rotator clock name
 * @clock_pp:		post processor clock
 * @clock_rot:		rotator clock 
 * @regs_pp:		post processor virtual address of SFR
 * @regs_rot:		rotator virtual address of SFR 
 * @irq_pp:		post processor irq number
 * @irq_rot:		rotator irq number
*/
struct s3c_rp_control {
	char				name[16];

	struct mutex			lock;
	struct s3c_rp_spin		spin;
	struct s3c_rp_waitq		waitq;
	atomic_t			in_use;

	struct device			*dev;
	struct video_device		*vd;
	
	struct s3c_rp_v4l2		v4l2;
	struct s3c_rp_fimd_info		fimd;
	struct s3c_rp_pp		pp;
	struct s3c_rp_rot		rot;

	enum s3c_rp_status		stream_status;

	/* Buffer */
	struct s3c_rp_buf_info		buf_info;
	struct s3c_rp_buf		user_buf[S3C_RP_BUFF_NUM];		/* buffer for incoming, outgoging */
	struct s3c_rp_buf		driver_buf[S3C_RP_BUFF_NUM];		/* buffer for rotator output, user cannot access this buffer*/
	int				incoming_queue[S3C_RP_BUFF_NUM];	/* buffer state = BUF_QUEUED */
	int				inside_queue[S3C_RP_BUFF_NUM];		/* buffer state = BUF_QUEUED */	
	int				outgoing_queue[S3C_RP_BUFF_NUM];	/* buffer state = BUF_DQUEUED */	
};

/*
 * V 4 L 2   F I M C   E X T E N S I O N S
 *
*/

/* CID extensions */
#define V4L2_CID_ROTATION		(V4L2_CID_PRIVATE_BASE + 0)

/* FIMD */
struct s3cfb_user_window {
	int x;
	int y;
};

#define S3CFB_WIN_OFF_ALL		_IO  ('F', 202)
#define S3CFB_WIN_POSITION		_IOW ('F', 203, struct s3cfb_user_window)

/*
 * E X T E R N S
 *
*/
extern struct s3c_rp_control s3c_rp;
extern const struct v4l2_ioctl_ops s3c_rp_v4l2_ops;
extern struct video_device s3c_rp_video_device;

extern int s3c_rp_check_buf(struct s3c_rp_control *ctrl, unsigned int num);
extern int s3c_rp_init_buf(struct s3c_rp_control *ctrl);
extern int s3c_rp_check_param(struct s3c_rp_control *ctrl);
	
extern void s3c_rp_pp_set_int_enable(struct s3c_rp_control *ctrl, unsigned int onoff);
extern void s3c_rp_pp_set_clock(struct s3c_rp_control *ctrl);
extern int s3c_rp_pp_set_param(struct s3c_rp_control *ctrl);
extern int s3c_rp_pp_set_pixelformat(struct s3c_rp_control *ctrl);
extern int s3c_rp_pp_set_scaler(struct s3c_rp_control *ctrl);
extern int s3c_rp_pp_query_scaler(const struct v4l2_rect *src_rect, const struct v4l2_rect *dst_rect, struct s3c_rp_scaler_cfg *scaler_cfg);
extern void s3c_rp_pp_set_addr(struct s3c_rp_control *ctrl, unsigned int index, enum s3c_pp_addr addr_pos);
extern int s3c_rp_pp_start(struct s3c_rp_control *ctrl, unsigned int index);
extern void s3c_rp_pp_set_envid(struct s3c_rp_control *ctrl, unsigned int onoff);
extern void s3c_rp_pp_fifo_start(struct s3c_rp_control *ctrl);
extern void s3c_rp_pp_fifo_stop(struct s3c_rp_control *ctrl, int sleep);

extern void s3c_rp_mapping_rot(struct s3c_rp_control *ctrl, int degree);
extern int s3c_rp_rot_set_param(struct s3c_rp_control *ctrl);
extern void s3c_rp_rot_set_addr(struct s3c_rp_control *ctrl, unsigned int index);
extern void s3c_rp_rot_start(struct s3c_rp_control *ctrl);
extern int s3c_rp_rot_run(struct s3c_rp_control *ctrl, unsigned int index);
extern void s3c_rp_rot_enable_int(struct s3c_rp_control *ctrl, unsigned int onoff);

extern int s3c_rp_attach_in_queue(struct s3c_rp_control *ctrl, unsigned int index);
extern int s3c_rp_detach_in_queue(struct s3c_rp_control *ctrl, int *index);
extern int s3c_rp_attach_inside_queue(struct s3c_rp_control *ctrl, unsigned int index);
extern int s3c_rp_detach_inside_queue(struct s3c_rp_control *ctrl, int *index);
extern int s3c_rp_attach_out_queue(struct s3c_rp_control *ctrl, unsigned int index);
extern int s3c_rp_detach_out_queue(struct s3c_rp_control *ctrl, int *index);

extern int s3cfb_direct_ioctl(int id, unsigned int cmd, unsigned long arg);
extern int s3cfb_open_fifo(int id, int ch, int (*do_priv)(void *param), void *param);
extern int s3cfb_close_fifo(int id, int (*do_priv)(void *param), void *param, int sleep);

#endif /* _S3C_RP_H */
