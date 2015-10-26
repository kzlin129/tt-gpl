/* linux/drivers/media/video/samsung/rp/s3c_rp_core.c
 *
 * Driver file for Samsung S3C6410 Renderer pipeline driver
 *
 * Jonghun Han, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <plat/clock.h>
#include <plat/media.h>
#include <plat/regs-pp.h>
#include <plat/regs-rotator.h>
#include <plat/regs-lcd.h>

#include "s3c_rp.h"

struct s3c_rp_control s3c_rp;

static irqreturn_t s3c_rp_pp_irq(int irq, void *dev_id)
{
	struct s3c_rp_control *ctrl = (struct s3c_rp_control *) dev_id;
	unsigned int	regs;
	unsigned int	prev_idx, next_index;
	int		ret = -1, wakeup = 0;

	/* Interrupt pendding clear */
	regs = __raw_readl(ctrl->pp.regs + S3C_VPP_MODE);
	regs &= ~(S3C_MODE_POST_PENDING);
	__raw_writel(regs, ctrl->pp.regs + S3C_VPP_MODE);

	if (ctrl->stream_status != RP_READY_OFF) {
		/* Attache done buffer to outgoing queue. */
		if (ctrl->pp.buf_idx.prev != -1) {
			ret = s3c_rp_attach_out_queue(ctrl, ctrl->pp.buf_idx.prev);
			if (ret < 0) {
				err("Failed : s3c_rp_attach_out_queue.\n");
			} else {
				ctrl->pp.buf_idx.prev = -1;
				wakeup = 1;	/* wake up s3c_rp_v4l2_dqbuf(). */
			}
		}

		/* Update pp structure. */
		if (ctrl->pp.buf_idx.next != -1) {
			ctrl->pp.buf_idx.run	= ctrl->pp.buf_idx.next;
			ctrl->pp.buf_idx.next	= -1;
		}

		if (ctrl->rot.degree == ROT_0)
			ret =  s3c_rp_detach_in_queue(ctrl, &next_index);
	 	else
			ret =  s3c_rp_detach_inside_queue(ctrl, &next_index);

		if (ret == 0) {		/* There is a buffer in pp input queue. */
			prev_idx	= ctrl->pp.buf_idx.run;
			ctrl->pp.buf_idx.prev = prev_idx;
			ctrl->pp.buf_idx.next = next_index;

			s3c_rp_pp_set_addr(ctrl, next_index, PP_NEXT);
		}
	}

	if (wakeup == 1)
		wake_up_interruptible(&ctrl->waitq.pp);
	
	return IRQ_HANDLED;
}

static irqreturn_t s3c_rp_rot_irq(int irq, void *dev_id)
{
	struct s3c_rp_control	*ctrl = (struct s3c_rp_control *) dev_id;
	unsigned int		rot_prev_idx, rot_next_idx;
	int			ret = -1;

	/* Interrupt pendding clear */
	__raw_readl(ctrl->rot.regs + S3C_ROTATOR_STATCFG);

	if (ctrl->stream_status != RP_READY_OFF) {
		rot_prev_idx		= ctrl->rot.run_idx;
		if (ctrl->stream_status == RP_READY_ON) {
			ret = s3c_rp_attach_inside_queue(ctrl, rot_prev_idx);
			if (ret < 0) {
				err("Failed : s3c_rp_attach_inside_queue.\n");
			}
		} else {
			if (ctrl->pp.buf_idx.next == -1) {
				ctrl->pp.buf_idx.prev = ctrl->pp.buf_idx.run;
				ctrl->pp.buf_idx.next = rot_prev_idx;

				s3c_rp_pp_set_addr(ctrl, rot_prev_idx, PP_NEXT);
			} else {
				ret = s3c_rp_attach_inside_queue(ctrl, rot_prev_idx);
				if (ret < 0) {
					err("Failed : s3c_rp_attach_inside_queue.\n");
				}
			}
		}

	 	ret = s3c_rp_detach_in_queue(ctrl, &rot_next_idx);
		if (ret == 0) {
			ctrl->rot.run_idx	= rot_next_idx;
			s3c_rp_rot_set_addr(ctrl, rot_next_idx);
			s3c_rp_rot_start(ctrl);
		} else {
			ctrl->rot.status	= ROT_IDLE;
			ctrl->rot.run_idx	= -1;
			//info("[%s] : There is no buffer in incoming queue.\n", __FUNCTION__);
		}
	}

	wake_up_interruptible(&ctrl->waitq.rot);

	return IRQ_HANDLED;
}

static int s3c_rp_open(struct inode *inode, struct file *filp)
{

	struct s3c_rp_control 	*ctrl;
	unsigned int 		regs;
	int 			ret;

	ctrl = &s3c_rp;

	info("Renderer pipeline Ver. 1.0.2\n");
	mutex_lock(&ctrl->lock);

	if (atomic_read(&ctrl->in_use)) {
		ret = -EBUSY;
		goto resource_busy;
	} else {
		atomic_inc(&ctrl->in_use);
		filp->private_data = ctrl;
	}

	/* Initialize the ctrl structure */
	regs = __raw_readl ( S3C_VIDTCON2 );
#if 0	
	ctrl->fimd.h_res = ((regs) & 0x7FF) + 1;
	ctrl->fimd.v_res = ((regs>>11) & 0x7FF) + 1;
#else
	ctrl->fimd.h_res = 240;
	ctrl->fimd.v_res = 400;
#endif
	ctrl->fimd.open_fifo	= s3cfb_open_fifo;
	ctrl->fimd.close_fifo	= s3cfb_close_fifo;

#if 0
	ret = s3cfb_direct_ioctl(0, S3CFB_GET_WQ, (unsigned long)&ctrl->fimd.wq);
	if (ret < 0)
		err("s3cfb_direct_ioctl(S3CFB_GET_WQ) fail\n");
#endif

	ctrl->stream_status	= RP_STREAMOFF;

	ctrl->buf_info.requested	= FALSE;
	
	ctrl->pp.buf_idx.prev	= -1;
	ctrl->pp.buf_idx.run	= -1;
	ctrl->pp.buf_idx.next	= -1;

	ctrl->rot.degree	= ROT_0;
	ctrl->rot.run_idx	= -1;

	mutex_unlock(&ctrl->lock);

	return 0;

resource_busy:
	mutex_unlock(&ctrl->lock);
	return ret;
}


static int s3c_rp_release(struct inode *inode, struct file *filp)
{
	struct s3c_rp_control *ctrl;

	ctrl = &s3c_rp;

	mutex_lock(&ctrl->lock);

	atomic_dec(&ctrl->in_use);
	filp->private_data = NULL;

	mutex_unlock(&ctrl->lock);

	return 0;
}

static int s3c_rp_mmap(struct file* filp, struct vm_area_struct *vma)
{
	struct s3c_rp_control *ctrl = filp->private_data;
	unsigned int	start_phy_addr = 0;
	unsigned int	size = vma->vm_end - vma->vm_start;
	unsigned int	pfn, index;

	/* This assumed that all buffers have same size. */
	index = vma->vm_pgoff;

	if (size > ctrl->user_buf[index].buf_length) {
		err("The size of mapping is too big.\n");
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED;

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		err("writable mapping must be shared\n");
		return -EINVAL;
	}

	if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_YUV420)
		start_phy_addr = ctrl->user_buf[index].buf_addr.phys_y;
	else if (ctrl->v4l2.video_out_fmt.pixelformat == V4L2_PIX_FMT_RGB32)
		start_phy_addr = ctrl->user_buf[index].buf_addr.phys_rgb;
	pfn = __phys_to_pfn(start_phy_addr);

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		err("mmap fail\n");
		return -EINVAL;
	}

	ctrl->user_buf[index].buf_flag |= V4L2_BUF_FLAG_MAPPED;

	return 0;
}

struct file_operations s3c_rp_fops = {
	.owner		= THIS_MODULE,
	.open		= s3c_rp_open,
	.release	= s3c_rp_release,
	.mmap		= s3c_rp_mmap,	
	.ioctl		= video_ioctl2,
};

static struct s3c_rp_control *s3c_rp_register_controller(struct platform_device *pdev)
{
	struct s3c_rp_control *ctrl;
	struct resource *res;

	ctrl = &s3c_rp;

	sprintf(ctrl->name, "%s", S3C_RP_NAME);
	sprintf(ctrl->pp.clk_name, "%s", S3C_RP_PP_CLK_NAME);
	sprintf(ctrl->rot.clk_name, "%s", S3C_RP_ROT_CLK_NAME);
	
	ctrl->dev = &pdev->dev;

	ctrl->vd = &s3c_rp_video_device;
	ctrl->vd->minor = S3C_RP_MINOR;
	strcpy(ctrl->vd->name, ctrl->name);

	ctrl->stream_status = RP_STREAMOFF;
	ctrl->rot.degree = ROT_0;

	atomic_set(&ctrl->in_use, 0);
	mutex_init(&ctrl->lock);

	spin_lock_init(&ctrl->spin.lock_in);
	spin_lock_init(&ctrl->spin.lock_inside);
	spin_lock_init(&ctrl->spin.lock_out);

	init_waitqueue_head(&ctrl->waitq.pp);
	init_waitqueue_head(&ctrl->waitq.rot);

	/* get rp(post processor) resource for io memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, S3C_RP_RESOURCE_PP);
	if (!res) {
		err("failed to get io memory region\n");
		return NULL;
	}

	/* request mem region */
	res = request_mem_region(res->start, res->end - res->start + 1, pdev->name);
	if (!res) {
		err("failed to request io memory region\n");
		return NULL;
	}

	/* ioremap for register block */
	ctrl->pp.regs = ioremap(res->start, res->end - res->start + 1);
	if (!ctrl->pp.regs) {
		err("failed to remap rp(post processor) io region\n");
		return NULL;
	}

	/* get rp(rotator) resource for io memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, S3C_RP_RESOURCE_ROT);
	if (!res) {
		err("failed to get io memory region\n");
		return NULL;
	}

	/* request mem region */
	res = request_mem_region(res->start, res->end - res->start + 1, pdev->name);
	if (!res) {
		err("failed to request io memory region\n");
		return NULL;
	}
	
	ctrl->rot.regs = ioremap(res->start, res->end - res->start + 1);
	if (!ctrl->rot.regs) {
		err("failed to remap rp(rotator) io region\n");
		return NULL;
	}

	/* irq */

	ctrl->pp.irq = platform_get_irq(pdev, S3C_RP_RESOURCE_PP);
	if (request_irq(ctrl->pp.irq, s3c_rp_pp_irq, IRQF_DISABLED, ctrl->name, ctrl))
		err("request_irq rp(post processor) failed\n");

	ctrl->rot.irq = platform_get_irq(pdev, S3C_RP_RESOURCE_ROT);
	if (request_irq(ctrl->rot.irq, s3c_rp_rot_irq, IRQF_DISABLED, ctrl->name, ctrl))
		err("request_irq rp(rotator) failed\n");
	
	
	return ctrl;
}

static int s3c_rp_unregister_controller(struct platform_device *pdev)
{
	struct s3c_rp_control *ctrl;

	ctrl = &s3c_rp;

	/* To Do : free memory buffer */

	iounmap(ctrl->pp.regs);
	iounmap(ctrl->rot.regs);

	memset(ctrl, 0, sizeof(*ctrl));
	
	return 0;
}

static int s3c_rp_probe(struct platform_device *pdev)
{
	struct s3c_rp_control *ctrl;
	int ret;

	info("s3c_rp_probe called\n");

	ctrl = s3c_rp_register_controller(pdev);
	if (!ctrl) {
		err("cannot register renderer pipeline controller\n");
		goto err_rp;
	}

	ctrl->pp.clock = clk_get(&pdev->dev, ctrl->pp.clk_name);
	if (IS_ERR(ctrl->pp.clock)) {
		err("failed to get rp(post processor) clock source\n");
		goto err_clk_io;
	}

	ctrl->rot.clock = clk_get(&pdev->dev, ctrl->rot.clk_name);
	if (IS_ERR(ctrl->rot.clock)) {
		err("failed to get rp(rotator) clock source\n");
		goto err_clk_io;
	}

	clk_enable(ctrl->pp.clock);
	clk_enable(ctrl->rot.clock);	
	s3c_rp_pp_set_clock(ctrl);

	ret = video_register_device(ctrl->vd, VFL_TYPE_GRABBER, S3C_RP_MINOR);
	if (ret) {
		err("cannot register video driver\n");
		goto err_video;
	}

	info("s3c_rp_probe successfully\n");

	return 0;

err_video:
	clk_disable(ctrl->pp.clock);
	clk_disable(ctrl->rot.clock);	
	clk_put(ctrl->pp.clock);
	clk_put(ctrl->rot.clock);	
	
err_clk_io:
	s3c_rp_unregister_controller(pdev);

err_rp:
	return -EINVAL;

	return 0;  
}



static void s3c_rp_vdev_release(struct video_device *vdev)
{
	kfree(vdev);
}

struct video_device s3c_rp_video_device = {
		.fops = &s3c_rp_fops,
		.ioctl_ops = &s3c_rp_v4l2_ops,
		.release  = s3c_rp_vdev_release,
};

static int s3c_rp_remove(struct platform_device *pdev)
{
	s3c_rp_unregister_controller(pdev);
	info("s3c_rp_remove success\n");

	return 0;
}

static int s3c_rp_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}


static int s3c_rp_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver s3c_rp_driver = {
       .probe		= s3c_rp_probe,
       .remove		= s3c_rp_remove,
       .suspend		= s3c_rp_suspend,
       .resume		= s3c_rp_resume,
       .driver		= {
		    .owner	= THIS_MODULE,
		    .name	= "s3c-rp",
	},
};

static char banner[] __initdata = KERN_INFO "S3C Renderer pipeline Driver, (c) 2009 Samsung Electronics\n";

static int __init s3c_rp_init(void)
{
 	unsigned int ret;
	printk(banner);
	
	ret = platform_driver_register(&s3c_rp_driver);
	if ( ret != 0) {
		err("s3c_rp_driver platform device register failed\n");
		return -1;
	}

	return 0;
}


static void s3c_rp_exit(void)
{
	platform_driver_unregister(&s3c_rp_driver);
	
	info("s3c renderer pipeline module exit\n");
}


module_init(s3c_rp_init);
module_exit(s3c_rp_exit);

MODULE_AUTHOR("Jonghun Han <jonghun.han@samsung.com>");
MODULE_DESCRIPTION("S3C Renderer pipeline Device Driver");
MODULE_LICENSE("GPL");
