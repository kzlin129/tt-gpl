/* linux/drivers/media/video/samsung/tv20/s5p_tv_base.c
 *
 * Entry file for Samsung TVOut driver
 *
 * SangPil Moon, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <mach/map.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "s5p_tv.h"

static struct mutex	*hMutex = NULL;

static struct resource	*tvout_mem;
void __iomem		*tvout_base;

static struct resource	*tvout_mem_clk;
void __iomem		*tvout_base_clk;

s5p_tv_status 	s5ptv_status;
s5p_tv_vo 	s5ptv_overlay[2];

#define TVOUT_CLK_INIT(dev, clk, name) 							\
	clk= clk_get(dev, name);							\
	if(clk == NULL) { 								\
		printk(KERN_ERR  "failed to find %s clock source\n", name);		\
		return -ENOENT;								\
	}										\
	clk_enable(clk)

#define TVOUT_IRQ_INIT(x, ret, dev, num, jump, ftn, m_name) 				\
	x = platform_get_irq(dev, num); 						\
	if (x <0 ) {									\
		printk(KERN_ERR  "failed to get %s irq resource\n", m_name);		\
		ret = -ENOENT; 								\
		goto jump;								\
	}										\
	ret = request_irq(x, ftn, IRQF_DISABLED, dev->name, dev) ;			\
	if (ret != 0) {									\
		printk(KERN_ERR  "failed to install %s irq (%d)\n", m_name, ret);	\
		goto jump;								\
	}										\
	while(0)


const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = {(S5P_HDCP_I2C_ADDR >> 1), I2C_CLIENT_END };
const static u16 *forces[] = { NULL };

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};

static struct i2c_driver s5p_hdcp_i2c_driver;

/*
 * ftn for irq 
 */
static irqreturn_t s5p_tvenc_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static irqreturn_t s5p_hpd_irq(int irq, void *dev_id)
{
	if(s5ptv_status.hpd_status){
		s5ptv_status.hpd_status = false;
		__s5p_set_hpd_detection(false);
		set_irq_type(IRQ_EINT5, IRQ_TYPE_EDGE_RISING);
	}else{
		s5ptv_status.hpd_status = true;
		__s5p_set_hpd_detection(true);
		set_irq_type(IRQ_EINT5, IRQ_TYPE_EDGE_FALLING);
	}

	printk("%s : hpd_status = %d\n", __FUNCTION__, s5ptv_status.hpd_status);
		
	return IRQ_HANDLED;
}

/*
 * ftn for video
 */
static int s5p_tv_v_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	mutex_lock(hMutex);

	if (s5ptv_status.tvout_output_enable) {
		printk(KERN_INFO "tvout drv. already used !!\n");
		ret =  -EBUSY;
		goto drv_used;
	}

	_s5p_tv_if_init_param();

	s5p_tv_v4l2_init_param();

	mutex_unlock(hMutex);
	return 0;

drv_used:
	mutex_unlock(hMutex);
	return ret;
}

int s5p_tv_v_read(struct file *filp, char *buf, size_t count,
		  loff_t *f_pos)
{
	return 0;
}

int s5p_tv_v_write(struct file *filp, const char *buf, size_t
		   count, loff_t *f_pos)
{
	return 0;
}

int s5p_tv_v_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return 0;
}

int s5p_tv_v_release(struct inode *inode, struct file *filp)
{
	_s5p_vlayer_stop();
	_s5p_tv_if_stop();

	s5ptv_status.tvout_output_enable = false;

	return 0;
}

/*
 * ftn for graphic(video output overlay)
 */
static int check_layer(dev_t dev)
{
	int id = 0;
	int layer = 0;

	id = MINOR(dev);

	switch (id) {
		// GRP0

	case TVOUT_MINOR_GRP0:
		layer = 0;
		break;
		// GRP1

	case TVOUT_MINOR_GRP1:
		layer = 1;
		break;

	default:
		break;
	}

	return layer;

}

static int s5p_tv_vo_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	int layer = 0;

	mutex_lock(hMutex);

	// check tvout path available!!
	if (!s5ptv_status.tvout_output_enable) {
		printk(KERN_INFO "check tvout start !!\n");
		ret =  -EACCES;
		goto resource_busy;
	}

	// check graphic layer !!
	layer = check_layer(inode->i_rdev);

	if (s5ptv_status.grp_layer_enable[layer]) {
		printk(KERN_INFO "grp %d layer is busy!!\n", layer);
		ret =  -EBUSY;
		goto resource_busy;
	}

	// set layer info.!!
	s5ptv_overlay[layer].index = layer;

	// set file private data.!!
	file->private_data = &s5ptv_overlay[layer];

	mutex_unlock(hMutex);

	return 0;

resource_busy:
	mutex_unlock(hMutex);

	return ret;
}

int s5p_tv_vo_release(struct inode *inode, struct file *filp)
{
	int layer = 0;

	layer = check_layer(inode->i_rdev);

	_s5p_grp_stop(layer);

	return 0;
}

/*
 * struct for video
 */
static struct file_operations s5p_tv_v_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_v_open,
	.read		= s5p_tv_v_read,
	.write		= s5p_tv_v_write,
	.ioctl		= s5p_tv_v_ioctl,
	.mmap		= s5p_tv_v_mmap,
	.release	= s5p_tv_v_release
};

/*
 * struct for graphic
 */
static struct file_operations s5p_tv_vo_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_vo_open,
	.ioctl		= s5p_tv_vo_ioctl,
	.release	= s5p_tv_vo_release
};


void s5p_tv_vdev_release(struct video_device *vdev)
{
	kfree(vdev);
}

struct video_device s5p_tvout[S5P_TVMAX_CTRLS] = {
	[0] = {
		.name = "S5PC1xx TVOUT for Video",
		.type2 = V4L2_CAP_VIDEO_OUTPUT,
		.fops = &s5p_tv_v_fops,
		.ioctl_ops = &s5p_tv_v4l2_v_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_VIDEO,
		.tvnorms = V4L2_STD_ALL_HD,
	},
	[1] = {
		.name = "S5PC1xx TVOUT Overlay0",
		.type2 = V4L2_CAP_VIDEO_OUTPUT_OVERLAY,
		.fops = &s5p_tv_vo_fops,
		.ioctl_ops = &s5p_tv_v4l2_vo_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_GRP0,
		.tvnorms = V4L2_STD_ALL_HD,
	},
	[2] = {
		.name = "S5PC1xx TVOUT Overlay1",
		.type2 = V4L2_CAP_VIDEO_OUTPUT_OVERLAY,
		.fops = &s5p_tv_vo_fops,
		.ioctl_ops = &s5p_tv_v4l2_vo_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_GRP1,
		.tvnorms = V4L2_STD_ALL_HD,
	},
};

/*
 * i2c client drv.  - register client drv
 */
static int s5p_hdcp_i2c_attach(struct i2c_adapter *adap, int addr, int kind)
{

	struct i2c_client *c;

	c = kzalloc(sizeof(*c), GFP_KERNEL);

	if (!c)
		return -ENOMEM;

	strcpy(c->name, "s5p_ddc_port");

	c->addr = addr;

	c->adapter = adap;

	c->driver = &s5p_hdcp_i2c_driver;

	s5ptv_status.hdcp_i2c_client = c;

	dev_info(&adap->dev, "s5p_hdcp_port attached successfully\n");

	return i2c_attach_client(c);
}

static int s5p_hdcp_i2c_attach_adapter(struct i2c_adapter *adap)
{
	int ret = 0;

	ret = i2c_probe(adap, &addr_data, s5p_hdcp_i2c_attach);

	if (ret) {
		dev_err(&adap->dev, 
			"failed to attach s5p_hdcp_port driver\n");
		ret = -ENODEV;
	}

	return ret;
}

static int s5p_hdcp_i2c_detach(struct i2c_client *client)
{
	i2c_detach_client(client);

	return 0;
}

static struct i2c_driver s5p_hdcp_i2c_driver = {
	.driver = {
		.name = "s5p_ddc_port",
	},
	.id = I2C_DRIVERID_S5P_HDCP,
	.attach_adapter = s5p_hdcp_i2c_attach_adapter,
	.detach_client = s5p_hdcp_i2c_detach,
};


/*
 *  Probe
 */
static int __init s5p_tv_probe(struct platform_device *pdev)
{
	struct resource *res;
	int 	irq_num;
	size_t	size;
	int 	ret;
	int 	i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
		dev_err(&pdev->dev, 
			"failed to get memory region resource\n");
		ret = -ENOENT;
		goto out;
	}

	size = (res->end - res->start) + 1;

	tvout_mem = request_mem_region(res->start, size, pdev->name);

	if (tvout_mem == NULL) {
		dev_err(&pdev->dev,  
			"failed to get memory region\n");
		ret = -ENOENT;
		goto out;
	}

	tvout_base = ioremap(res->start, size);

	if (tvout_base == NULL) {
		dev_err(&pdev->dev,  
			"failed to ioremap address region\n");
		ret = -ENOENT;
		goto out;

	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (res == NULL) {
		dev_err(&pdev->dev,  
			"failed to get memory region resource\n");
		ret = -ENOENT;
		goto out;
	}

	size = (res->end - res->start) + 1;

	tvout_mem_clk = request_mem_region(res->start, size, pdev->name);

	if (tvout_mem_clk == NULL) {
		dev_err(&pdev->dev,  
			"failed to get memory region\n");
		ret = -ENOENT;
		goto out;
	}

	tvout_base_clk = ioremap(res->start, size);

	if (tvout_base_clk == NULL) {
		dev_err(&pdev->dev, 
			"failed to ioremap address region\n");
		ret = -ENOENT;
		goto out;
	}

	// clock
	TVOUT_CLK_INIT(&pdev->dev, s5ptv_status.tvenc_clk, "tv");

	TVOUT_CLK_INIT(&pdev->dev, s5ptv_status.vp_clk, "vp");

	TVOUT_CLK_INIT(&pdev->dev, s5ptv_status.mixer_clk, "mixer");

	TVOUT_CLK_INIT(&pdev->dev, s5ptv_status.hdmi_clk, "hdmi");

#ifdef FIX_27M_UNSTABLE_ISSUE
	writel(0x1, S5PC1XX_GPA0_BASE + 0x56c);

#endif

	// irq
	TVOUT_IRQ_INIT(irq_num, ret, pdev, 0,  out, __s5p_mixer_irq, "mixer");

	TVOUT_IRQ_INIT(irq_num, ret, pdev, 1,  out_hdmi_irq, __s5p_hdmi_irq , "hdmi");

	TVOUT_IRQ_INIT(irq_num, ret, pdev, 2,  out_tvenc_irq, s5p_tvenc_irq, "tvenc");

	TVOUT_IRQ_INIT(irq_num, ret, pdev, 3,  out_hpd_irq, s5p_hpd_irq, "hpd");

	// v4l2 video device registration
	for (i = 0;i < S5P_TVMAX_CTRLS;i++) {
		s5ptv_status.video_dev[i] = &s5p_tvout[i];

		if (video_register_device(s5ptv_status.video_dev[i],
				VFL_TYPE_GRABBER, s5p_tvout[i].minor) != 0) {
				
			dev_err(&pdev->dev, 
				"Couldn't register tvout driver.\n");
			return 0;
		}
	}

	// v4l2 video device registration
	hMutex = (struct mutex *)kmalloc(sizeof(struct mutex), GFP_KERNEL);

	if (hMutex == NULL) {
		dev_err(&pdev->dev, 
			"failed to create mutex handle\n");
		goto out;
	}

	mutex_init(hMutex);

	// for ddc(hdcp port)
	if (i2c_add_driver(&s5p_hdcp_i2c_driver))
		dev_info(&pdev->dev, 
			"DDC port driver(HDCP port) add failed\n");

	// for i2c probing
	udelay(100);

	s3c_gpio_cfgpin(S5PC1XX_GPH0(5), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S5PC1XX_GPH0(5), S3C_GPIO_PULL_UP);
	set_irq_type(IRQ_EINT5, IRQ_TYPE_EDGE_RISING);


	return 0;

out_hpd_irq:
	free_irq(IRQ_TVENC, pdev);

out_tvenc_irq:
	free_irq(IRQ_HDMI, pdev);

out_hdmi_irq:
	free_irq(IRQ_MIXER, pdev);

out:
	printk(KERN_ERR "not found (%d). \n", ret);

	return ret;
}

/*
 *  Remove
 */
static int s5p_tv_remove(struct platform_device *pdev)
{
	i2c_del_driver(&s5p_hdcp_i2c_driver);

	iounmap(tvout_base);

	/* remove memory region */
	if (tvout_mem != NULL) {
		if (release_resource(tvout_mem))
			dev_err(&pdev->dev,
				"Can't remove tvout drv !!\n");

		kfree(tvout_mem);

		tvout_mem = NULL;
	}

	iounmap(tvout_base_clk);

	/* remove memory region */
	if (tvout_mem_clk != NULL) {
		if (release_resource(tvout_mem_clk))
			dev_err(&pdev->dev, 
			"Can't remove tvout drv !!\n");

		kfree(tvout_mem_clk);

		tvout_mem_clk = NULL;
	}

	clk_disable(s5ptv_status.tvenc_clk);
	clk_disable(s5ptv_status.vp_clk);
	clk_disable(s5ptv_status.mixer_clk);
	clk_disable(s5ptv_status.hdmi_clk);

	free_irq(IRQ_MIXER, pdev);
	free_irq(IRQ_HDMI, pdev);
	free_irq(IRQ_TVENC, pdev);
	free_irq(IRQ_EINT5, pdev);

	mutex_destroy(hMutex);

	return 0;
}


/*
 *  Suspend
 */
int s5p_tv_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

/*
 *  Resume
 */
int s5p_tv_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver s5p_tv_driver = {
	.probe		= s5p_tv_probe,
	.remove		= s5p_tv_remove,
	.suspend	= s5p_tv_suspend,
	.resume		= s5p_tv_resume,
	.driver		= {
		.name	= "s5p-tvout",
		.owner	= THIS_MODULE,
	},
};

static char banner[] __initdata = KERN_INFO "S5PC1XX TVOUT Driver, (c) 2009 Samsung Electronics\n";

int __init s5p_tv_init(void)
{
	int ret;

	printk(banner);

	ret = platform_driver_register(&s5p_tv_driver);

	if (ret) {
		printk(KERN_ERR "Platform Device Register Failed %d\n", ret);
		return -1;
	}

	return 0;
}

static void __exit s5p_tv_exit(void)
{
	platform_driver_unregister(&s5p_tv_driver);
}

module_init(s5p_tv_init);
module_exit(s5p_tv_exit);

MODULE_AUTHOR("SangPil Moon");
MODULE_DESCRIPTION("SS5PC1XX TVOUT driver");
MODULE_LICENSE("GPL");
