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

#include <linux/a043fw05v4_pdata.h>


static int a043fw05v4_gpio_request(void)
{
	int err = 0;
   
	err = gpio_request(TT_VGPIO_LCM_CS, "TT_VGPIO_LCM_CS");

	if (err){
		printk(KERN_ERR "error requesting TT_VGPIO_LCM_CS %d\n",TT_VGPIO_LCM_CS);
		return err;
	}

	err = gpio_request(TT_VGPIO_LCM_PWR_EN, "TT_VGPIO_LCM_PWR_EN");

	if (err){
		printk(KERN_ERR"error requesting TT_VGPIO_LCM_PWR_EN %d\n",TT_VGPIO_LCM_PWR_EN);
		return err;
	}

	return err;
}

static void a043fw05v4_gpio_free(void)
{
	gpio_free(TT_VGPIO_LCM_CS);
	gpio_free(TT_VGPIO_LCM_PWR_EN);

}

static void a043fw05v4_config_gpio(void)
{
  //give power but no disp on yet.
  gpio_direction_output(TT_VGPIO_LCM_PWR_EN, 1);
  gpio_direction_output(TT_VGPIO_LCM_CS, 0);
  	
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_LCM_PWR_EN), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_LCM_CS), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin_slp(vgpio_to_gpio(TT_VGPIO_LCM_PWR_EN), S3C_GPIO_SLEEP_OUTPUT1);
  s3c_gpio_cfgpin_slp(vgpio_to_gpio(TT_VGPIO_LCM_CS), S3C_GPIO_SLEEP_OUTPUT0);  
}

static void a043fw05v4_power_on(void)
{
	gpio_set_value(TT_VGPIO_LCM_PWR_EN, 1);
}

static void a043fw05v4_power_off(void)
{
	gpio_set_value(TT_VGPIO_LCM_PWR_EN, 0);
}

static void a043fw05v4_disp_on(void)
{
	gpio_set_value(TT_VGPIO_LCM_CS, 1);
}

static void a043fw05v4_disp_off (void)
{
	gpio_set_value(TT_VGPIO_LCM_CS, 0);	
}


static struct a043fw05v4_platform_data lcm_pdata = {
	.config_gpio = a043fw05v4_config_gpio,
	.request_gpio = a043fw05v4_gpio_request,
	.free_gpio = a043fw05v4_gpio_free,
	.disp_on = a043fw05v4_disp_on,
	.disp_off = a043fw05v4_disp_off,
	.power_on = a043fw05v4_power_on,
	.power_off = a043fw05v4_power_off,	
};

struct a043fw05v4_platform_data * setup_a043fw05v4_pdata(void)
{
	return &lcm_pdata;
}

