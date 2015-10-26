/* arch/arm/mach-bcm476x/pwm_bcm4760.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/arch/reg_gpio.h>
#include <linux/pwm.h>
#include <linux/broadcom/gpio.h>
#include <asm/arch/bcm4760_reg.h>

#define  PWM_OUT_POLARITY_00	0x100
#define  PWM_OUT_POLARITY_01	0x200
#define  PWM_OPENDRAIN_00	0x8000
#define  PWM_OPENDRAIN_01	0x10000
#define  PWM_ENABLE_00		(0x01 | PWM_OUT_POLARITY_00)
#define  PWM_ENABLE_01		(0x02 | PWM_OUT_POLARITY_01)

struct pwm_device {
	struct list_head	 list;
	struct platform_device	*pdev;

	const char		*label;

	unsigned int		 period_ns;
	unsigned int		 duty_ns;
	unsigned int		 prescale;
	unsigned int		 poparity;
	unsigned char		 running;
	unsigned char		 use_count;
	unsigned char		 pwm_id;
};

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

/////////////////////////////////////////////////////////////

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, list) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count = 1;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}

EXPORT_SYMBOL(pwm_request);

/////////////////////////////////////////////////////////////
void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		printk(KERN_ERR "PWM%d device already freed\n", pwm->pwm_id);

	mutex_unlock(&pwm_lock);
}

EXPORT_SYMBOL(pwm_free);

/////////////////////////////////////////////////////////////
int pwm_enable(struct pwm_device *pwm)
{
	unsigned long flags;
	unsigned long tcon;

	local_irq_save(flags);

	tcon = readl(IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
#ifdef PWM_CHANEL_00
	tcon |= PWM_ENABLE_00;
#else
	tcon |= PWM_ENABLE_01;
#endif

	writel( tcon,IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
	local_irq_restore(flags);

	pwm->running = 1;
	return 0;
}

EXPORT_SYMBOL(pwm_enable);

/////////////////////////////////////////////////////////////
void pwm_disable(struct pwm_device *pwm)
{
	unsigned long flags;
	unsigned long tcon;

	local_irq_save(flags);

	tcon = readl(IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
#ifdef PWM_CHANEL_00
	tcon &= ~PWM_ENABLE_00;
#else
	tcon &= ~PWM_ENABLE_01;
#endif
	writel( tcon,IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));

	local_irq_restore(flags);

	pwm->running = 0;

}

EXPORT_SYMBOL(pwm_disable);

/////////////////////////////////////////////////////////////
int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long flags;
	unsigned long tmp;

	if (duty_ns > period_ns)
		return -EINVAL;

	if (period_ns == pwm->period_ns &&
	    duty_ns == pwm->duty_ns)
		return 0;

#ifdef PWM_CHANEL_00
	tmp = readl(IO_ADDRESS(CMU_R_CHIP_PIN_MUX5_MEMADDR));
	tmp |= 0x1 << CMU_F_GPIO_12_MXSEL_R;
	writel( tmp,IO_ADDRESS(CMU_R_CHIP_PIN_MUX5_MEMADDR));
#else
	tmp = readl(IO_ADDRESS(CMU_R_CHIP_PIN_MUX5_MEMADDR));
	tmp |= 0x1 << CMU_F_GPIO_13_MXSEL_R;
	writel( tmp,IO_ADDRESS(CMU_R_CHIP_PIN_MUX5_MEMADDR));
#endif

	local_irq_save(flags);

	IF_BCM4760_A0
	{
	tmp = readl(IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
#ifdef PWM_CHANEL_00
	tmp &= ~PWM_ENABLE_00;
#else
	tmp &= ~PWM_ENABLE_01;
#endif
	writel( tmp,IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
	

#ifdef PWM_CHANEL_00
	writel( period_ns,IO_ADDRESS(PWM_R_PERIOD_CNT_0_MEMADDR));
	writel( duty_ns,IO_ADDRESS(PWM_R_DUTY_CNT_0_MEMADDR));
#else
	writel( period_ns,IO_ADDRESS(PWM_R_PERIOD_CNT_1_MEMADDR));
	writel( duty_ns,IO_ADDRESS(PWM_R_DUTY_CNT_1_MEMADDR));

#endif
	writel( pwm->prescale << 18,IO_ADDRESS(PWM_R_PRESCALE_MEMADDR));
	}
	else
	{
		writel( (pwm->prescale << 18) | 0xC0000000,IO_ADDRESS(PWM_R_PRESCALE_MEMADDR));	
#ifdef PWM_CHANEL_00
		writel( period_ns, IO_ADDRESS(PWM_R_PERIOD_CNT_0_MEMADDR));
		writel( period_ns - duty_ns,IO_ADDRESS(PWM_R_DUTY_CNT_0_MEMADDR));
		tmp = readl(IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
		tmp |= 0x040;
#else
		writel( period_ns, IO_ADDRESS(PWM_R_PERIOD_CNT_1_MEMADDR));
		writel( period_ns - duty_ns,IO_ADDRESS(PWM_R_DUTY_CNT_1_MEMADDR));
		tmp = readl(IO_ADDRESS(PWM_R_PWMCTL_MEMADDR));
		tmp |= 0x080;
#endif
		writel( tmp, IO_ADDRESS(PWM_R_PWMCTL_MEMADDR)); 
	}

	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(pwm_config);

/////////////////////////////////////////////////////////////
static int pwm_register(struct pwm_device *pwm)
{
	pwm->duty_ns = -1;
	pwm->period_ns = -1;

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->list, &pwm_list);
	mutex_unlock(&pwm_lock);

	return 0;
}

/////////////////////////////////////////////////////////////
static int bcm4760_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_device *pwm;

	unsigned int id = pdev->id;
	int ret;

	if (id == 4) {
		dev_err(dev, "TIMER4 is currently not supported\n");
		return -ENXIO;
	}

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(dev, "failed to allocate pwm_device\n");
		return -ENOMEM;
	}

	pwm->pdev = pdev;
	pwm->pwm_id = id;
	pwm->period_ns = 0x25;
	pwm->poparity = 0x100;
	pwm->prescale = 1;



	ret = pwm_register(pwm);
	if (ret) {
		dev_err(dev, "failed to register pwm\n");
		kfree(pwm);
		return ret;
	}

	platform_set_drvdata(pdev, pwm);

	return 0;
}

static struct platform_driver bcm_pwm_driver = {
	.driver		= {
		.name	= "bcm4760-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= bcm4760_pwm_probe,
};

/////////////////////////////////////////////////////////////
static int __init pwm_init(void)
{
	int ret;

	ret = platform_driver_register(&bcm_pwm_driver);
	if (ret) {
		printk(KERN_ERR "%s: failed to add pwm driver\n", __func__);
	}

	return ret;
}

arch_initcall(pwm_init);
