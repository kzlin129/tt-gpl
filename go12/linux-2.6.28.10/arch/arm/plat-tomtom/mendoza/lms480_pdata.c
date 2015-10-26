#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <plat/map.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>
#include <plat/mendoza.h>
#include <mach/gpio.h>

#include <plat/lms480_pdata.h>

static void lms480wv_suspend(void)
{
}

static void lms480wv_resume(void)
{
}

static int lms480wv_gpio_request(void)
{
	int err = 0;

	err = gpio_request(TT_VGPIO_LCM_RESET, "LCM_RESET");

	if (err){
		printk(KERN_ERR "error requesting LCM_RESET %d\n",TT_VGPIO_LCM_RESET);
		return err;
	}

	err = gpio_request(TT_VGPIO_LCM_PWR_EN, "LCM_PWR_EN");

	if (err){
		printk(KERN_ERR"error requesting LCM_PWR_EN %d\n",TT_VGPIO_LCM_PWR_EN);
		return err;
	}

	return err;
}

static void lms480wv_gpio_free(void)
{
	gpio_free(TT_VGPIO_LCM_PWR_EN);
	gpio_free(TT_VGPIO_LCM_RESET);
	gpio_free(TT_VGPIO_BACKLIGHT_EN);
	//gpio_free(TT_VGPIO_BACKLIGHT_PWM);
}

static void lms480wv_config_gpio(void)
{
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_LCM_PWR_EN), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_LCM_RESET), S3C_GPIO_PULL_NONE);

	gpio_direction_output(TT_VGPIO_LCM_PWR_EN, 1);/* low active */
	gpio_direction_output(TT_VGPIO_LCM_RESET,  1);/* low active */
}

static void lms480wv_enable_lcm (void)
{
	gpio_set_value(TT_VGPIO_LCM_PWR_EN, 0);	/* enable power to the LCM */
}

static void lms480wv_reset_lcm (void)
{
	mdelay(100);
	gpio_set_value(TT_VGPIO_LCM_RESET, 0);
	mdelay(10);

	gpio_set_value(TT_VGPIO_LCM_RESET, 1);
	mdelay(10);
}

static int lms480wv_simple_backlight_on(void)
{
	int err =0;
#if 0
	printk(KERN_INFO"***INFO: switching backlight on.. not using PWM\n");

	err = gpio_request(TT_VGPIO_BACKLIGHT_EN, "BACKLIGHT_EN");

	if (err) {
		printk(KERN_ERR "error requesting BACKLIGHT_EN %d\n",TT_VGPIO_BACKLIGHT_EN);
		return err;
	}

	err = gpio_request(TT_VGPIO_BACKLIGHT_PWM, "BACKLIGHT_PWM");

	if (err) {
		printk(KERN_ERR "error requesting BACKLIGHT_PWM %d\n",TT_VGPIO_BACKLIGHT_PWM);
		return err;
	}

	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_BACKLIGHT_EN), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_BACKLIGHT_PWM), S3C_GPIO_PULL_NONE);

	gpio_direction_output(TT_VGPIO_BACKLIGHT_PWM, 0);/* 0 == PWM max output */
	gpio_direction_output(TT_VGPIO_BACKLIGHT_EN, 1);/* high active */
#else
	pr_err("lms480wv_simple_backlight_on: disabled!!!!!!!\n");
#endif

	return err;
}


static struct lms480wv_platform_data lcm_pdata = {
	.suspend = lms480wv_suspend,
	.resume  = lms480wv_resume,
	.reset_lcm = lms480wv_reset_lcm,
	.enable_lcm = lms480wv_enable_lcm,
	.config_gpio = lms480wv_config_gpio,
	.request_gpio = lms480wv_gpio_request,
	.free_gpio = lms480wv_gpio_free,
	.simple_backlight_on = lms480wv_simple_backlight_on,
};

struct lms480wv_platform_data * setup_lms480wv_pdata(void)
{
	return &lcm_pdata;
}

