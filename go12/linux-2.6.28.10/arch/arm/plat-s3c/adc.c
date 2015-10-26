/* arch/arm/plat-s3c24xx/adc.c
 *
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>, <ben-linux@fluff.org>
 *
 * S3C24XX ADC device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <mach/irqs.h>

/* This driver is designed to control the usage of the ADC block between
 * the touchscreen and any other drivers that may need to use it, such as
 * the hwmon driver.
 *
 * Priority will be given to the touchscreen driver, but as this itself is
 * rate limited it should not starve other requests which are processed in
 * order that they are received.
 *
 * Each user registers to get a client block which uniquely identifies it
 * and stores information such as the necessary functions to callback when
 * action is required.
 */

struct s3c_adc_client {
	struct platform_device	*pdev;
	struct list_head	 pend;

	unsigned int		 nr_samples;
	unsigned char		 is_ts;
	unsigned char		 channel;

	select_func		 select_cb;
	convert_func		 convert_cb;

	void 			*data;
};

struct adc_device {
	struct platform_device		*pdev;
	struct platform_device		*owner;
	struct clk			*clk;
	struct s3c_adc_client		*cur;
	struct s3c_adc_client		*ts_pend;
	void __iomem			*regs;

	unsigned int 		 	data_mask;

	int			 	irq;

	struct s3c_adc_mach_info	*platform_data;
};

static struct adc_device *adc_dev;

static DEFINE_SPINLOCK(adc_lock);
static LIST_HEAD(adc_pending);

static int adc_suspended;

#define adc_dbg(_adc, msg...) dev_dbg(&(_adc)->pdev->dev, msg)

static inline void s3c_adc_convert(struct adc_device *adc, struct s3c_adc_client *client)
{
	unsigned con;

	client->select_cb(SELECTED, client->data);

	con = readl(adc->regs + S3C_ADCCON);
	con |= S3C_ADCCON_ENABLE_START;
	writel(con, adc->regs + S3C_ADCCON);
}

static inline void s3c_adc_select(struct adc_device *adc,
				  struct s3c_adc_client *client)
{
	unsigned con = readl(adc->regs + S3C_ADCCON);

	con &= ~S3C_ADCCON_MUXMASK;
	con &= ~S3C_ADCCON_STDBM;
	con &= ~S3C_ADCCON_STARTMASK;

	if (!client->is_ts){
#ifdef CONFIG_CPU_S5P6440
		/* this is different between sp56440 and s3c6410!
		on the s3c6410 the mux bits are in the con reg but on s5p
		there is a separate register for the mux*/		
		writel(client->channel, adc->regs + S3C_ADCMUX);
#else
		con |= S3C_ADCCON_SELMUX(client->channel);
#endif
	}

	writel(con, adc->regs + S3C_ADCCON);
}

static void s3c_adc_dbgshow(struct adc_device *adc)
{
	adc_dbg(adc, "CON=%08x, TSC=%08x, DLY=%08x\n",
		readl(adc->regs + S3C_ADCCON),
		readl(adc->regs + S3C_ADCTSC),
		readl(adc->regs + S3C_ADCDLY));
}

int s3c_adc_set_ts_control (struct s3c_adc_client *client, unsigned int port, unsigned int value)
{
	struct adc_device *adc = adc_dev;

	if (!client->is_ts)
		return -EINVAL;	

	if (port != S3C_ADCTSC && port != S3C_ADCCLRWK)
		return -EINVAL;

	writel (value, adc->regs + port);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c_adc_set_ts_control);


unsigned int s3c_adc_get_ts_control (struct s3c_adc_client *client, unsigned int port)
{
	struct adc_device *adc = adc_dev;

	if (client == NULL)
		return -EINVAL;

	if (!client->is_ts)
		return -EINVAL;

	return (readl (adc->regs + port));
}
EXPORT_SYMBOL_GPL(s3c_adc_get_ts_control);


static void s3c_adc_try(struct adc_device *adc)
{
	struct s3c_adc_client *next = adc->ts_pend;
	unsigned long flags;

	/* If suspend(ed/ing), don't let any client bug us,
	 * it just confuses the hell out the ADC... */
	if (unlikely(adc_suspended))
		return;

	spin_lock_irqsave(&adc_lock, flags);

	if (!next && !list_empty(&adc_pending)) {
		next = list_first_entry(&adc_pending, struct s3c_adc_client, pend);
		list_del(&next->pend);
	} else
		adc->ts_pend = NULL;

	spin_unlock_irqrestore(&adc_lock, flags);

	if (next) {
		adc->cur = next;
		s3c_adc_select(adc, next);

		next->select_cb(INIT, next->data);

		s3c_adc_convert(adc, next);

		s3c_adc_dbgshow(adc);
	}
}

int s3c_adc_start(struct s3c_adc_client *client,
		  unsigned int channel, unsigned int nr_samples)
{
	struct adc_device *adc = adc_dev;
	unsigned long flags;

	/* If suspend(ed/ing), don't let any client bug us,
	 * it just confuses the hell out the ADC... */
	if (unlikely(adc_suspended))
		return -EAGAIN;

	if (!adc) {
		printk(KERN_ERR "%s: failed to find adc\n", __func__);
		return -EINVAL;
	}

	if (client->is_ts && adc->ts_pend)
		return -EAGAIN;

	spin_lock_irqsave(&adc_lock, flags);

	client->channel = channel;
	client->nr_samples = nr_samples;

	if (client->is_ts)
		adc->ts_pend = client;
	else {
		/* Don't allow a client to be added twice,
		 * this is just a recipe for disaster... */
		struct list_head *lh;
		int listed = 0;

		list_for_each(lh, &adc_pending) {
			if (list_entry(lh, struct s3c_adc_client, pend) == client) {
				listed = 1;
				break;
			}
		}

		if (!listed)
			list_add_tail(&client->pend, &adc_pending);
		
	}

	spin_unlock_irqrestore(&adc_lock, flags);

	if (!adc->cur) 
		s3c_adc_try(adc);

	

	return 0;
}
EXPORT_SYMBOL_GPL(s3c_adc_start);

static void s3c_adc_default_select(unsigned select, void *data)
{
}

struct s3c_adc_client *s3c_adc_register(struct platform_device *pdev,
					select_func select,
					convert_func conv, 
					void *data,
					unsigned int is_ts)
{
	struct s3c_adc_client *client;

	WARN_ON(!pdev);
	WARN_ON(!conv);

	if (!select)
		select = s3c_adc_default_select;

	if (!conv || !pdev)
		return ERR_PTR(-EINVAL);

	client = kzalloc(sizeof(struct s3c_adc_client), GFP_KERNEL);
	if (!client) {
		dev_err(&pdev->dev, "no memory for adc client\n");
		return ERR_PTR(-ENOMEM);
	}

	client->pdev = pdev;
	client->is_ts = is_ts;
	client->select_cb = select;
	client->convert_cb = conv;
	client->data = data;

	return client;
}
EXPORT_SYMBOL_GPL(s3c_adc_register);

void s3c_adc_release(struct s3c_adc_client *client)
{
	/* We should really check that nothing is in progress. */
	kfree(client);
}
EXPORT_SYMBOL_GPL(s3c_adc_release);

static irqreturn_t s3c_adc_irq(int irq, void *pw)
{
	struct adc_device *adc = pw;
	struct s3c_adc_client *client = adc->cur;
	unsigned long flags;
	unsigned data0, data1;

	if (!client) {
		dev_warn(&adc->pdev->dev, "%s: no adc pending\n", __func__);
		return IRQ_HANDLED;
	}

	data0 = readl(adc->regs + S3C_ADCDAT0);
	data1 = readl(adc->regs + S3C_ADCDAT1);

	local_irq_save (flags);

	if (client->is_ts)
		(client->convert_cb)(data0, data1, client->data);
	else
		(client->convert_cb)(data0 & adc->data_mask, data1 & adc->data_mask, client->data);

	__raw_writel(0x0, adc->regs+S3C_ADCCLRINT);

	if (--client->nr_samples > 0) {
		/* fire another conversion for this */
		s3c_adc_convert(adc, client);

	} else {
		adc->cur = NULL;

		(client->select_cb)(FINISHED, client->data);
	
		s3c_adc_try(adc);
	}

	local_irq_restore(flags);

	return IRQ_HANDLED;
}

static inline resource_size_t device_resource_size(struct resource *res)
{
        return res->end - res->start + 1;
}

static inline const char *device_name(const struct device *dev)
{
        /* will be changed into kobject_name(&dev->kobj) in the near future */
        return dev->bus_id;
}

struct s3c_adc_mach_info s3c_adc_default_cfg = {
	.delay = 1000,
	.resolution = 12,
	.presc=49,
}; 

static struct s3c_adc_mach_info *s3c_adc_get_platdata(struct device *dev)
{
        if(dev->platform_data != NULL)
        {
                printk(KERN_INFO "ADC platform data read\n");
                return (struct s3c_adc_mach_info*)dev->platform_data;
        }

        return &s3c_adc_default_cfg;
}


static inline unsigned int s3c_adc_get_adccon (struct adc_device *adc)
{
	struct s3c_adc_mach_info *cfg = adc->platform_data;
	unsigned int adccon = 0;

	if (cfg->resolution == 12) {
		adccon |= S3C_ADCCON_RESSEL_12BIT;
		adc->data_mask = S3C_ADCDAT0_XPDATA_MASK_12BIT;
	} else {
		adc->data_mask = S3C_ADCDAT0_XPDATA_MASK;
	}

	if (cfg->presc != 0)
		adccon |= S3C_ADCCON_PRSCEN | S3C_ADCCON_PRSCVL(cfg->presc);

	return adccon;
}

static int s3c_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adc_device *adc;
	struct s3c_adc_mach_info *cfg;
	struct resource *regs;
	unsigned int adccon;
	int ret;

	adc = kzalloc(sizeof(struct adc_device), GFP_KERNEL);
	if (adc == NULL) {
		dev_err(dev, "failed to allocate adc_device\n");
		return -ENOMEM;
	}

	adccon 	  = 0;
	adc->pdev = pdev;

	adc->irq = platform_get_irq(pdev, 0);
	if (adc->irq <= 0) {
		dev_err(dev, "failed to get adc irq\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(adc->irq, s3c_adc_irq, 0, dev->bus_id, adc);
	if (ret < 0) {
		dev_err(dev, "failed to attach adc irq\n");
		goto err_irq;
	}

	adc->clk = clk_get(dev, "adc");
	if (IS_ERR(adc->clk)) {
		dev_err(dev, "failed to get adc clock\n");
		ret = PTR_ERR(adc->clk);
		goto err_irq;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENXIO;
		goto err_clk;
	}

	adc->regs = ioremap(regs->start, device_resource_size(regs));
	if (!adc->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_ioregion;
	}

	cfg = s3c_adc_get_platdata (&pdev->dev);
	if (cfg == NULL) {
		dev_err(dev, "failed to get platform data\n");
		ret = -ENOENT;
		goto err_plat_data;
	}

	adc->platform_data = cfg;

	clk_enable(adc->clk);

	adccon = s3c_adc_get_adccon (adc);

        writel(adccon,  adc->regs + S3C_ADCCON);
	writel(adc->platform_data->delay & 0xffff, adc->regs + S3C_ADCDLY);
	
	dev_info(dev, "attached adc driver\n");

	platform_set_drvdata(pdev, adc);
	adc_dev = adc;

	return 0;

 err_plat_data:
	iounmap(adc->regs);

 err_ioregion:
	clk_put(adc->clk);
 err_clk:
	free_irq(adc->irq, adc);
 err_irq:
	kfree(adc);
	return ret;
}

static int s3c_adc_remove(struct platform_device *pdev)
{
	struct adc_device *adc = platform_get_drvdata(pdev);

	iounmap(adc->regs);
	free_irq(adc->irq, adc);
	clk_disable(adc->clk);
	clk_put(adc->clk);
	kfree(adc);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct adc_device *adc = platform_get_drvdata(pdev);
	u32 con;

	printk(KERN_INFO"S5P6440 ADC suspending.\n");	
	adc_suspended = 1;

	con = readl(adc->regs + S3C_ADCCON);
	con |= S3C_ADCCON_STDBM;
	writel(con, adc->regs + S3C_ADCCON);

	adc->cur = NULL;
	adc->ts_pend = NULL;

	disable_irq(adc->irq);
	clk_disable(adc->clk);

	return 0;
}

static int s3c_adc_resume(struct platform_device *pdev)
{
	struct adc_device *adc = platform_get_drvdata(pdev);
	struct s3c_adc_mach_info *cfg = adc->platform_data;
	unsigned int adccon;

	printk(KERN_INFO"S5P6440 ADC resuming.\n");	
	clk_enable(adc->clk);

	adccon = s3c_adc_get_adccon(adc);

        writel(adccon,              adc->regs + S3C_ADCCON);
        writel(cfg->delay & 0xffff, adc->regs + S3C_ADCDLY);

#if 0
	/* The touchscreen gets resumed before the ADC does, this means that
	 * the ADCTSC values will not be set. This is a walk-around this this issue
         */
	writel(S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | 
	       S3C_ADCTSC_XY_PST(3), adc->regs + S3C_ADCTSC);
#endif

	enable_irq(adc->irq);

	adc_suspended = 0;

	return 0;
}

#else
#define s3c_adc_suspend NULL
#define s3c_adc_resume NULL
#endif

static struct platform_driver s3c_adc_driver = {
	.driver		= {
		.name	= "s3c-adc",
		.owner	= THIS_MODULE,
	},
	.probe		= s3c_adc_probe,
	.remove		= __devexit_p(s3c_adc_remove),
	.suspend	= s3c_adc_suspend,
	.resume		= s3c_adc_resume,
};

static int __init adc_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_adc_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add adc driver\n", __func__);

	printk(KERN_INFO "S5P6440 ADC driver successfully initialized !\n");

	return ret;
}

arch_initcall(adc_init);
