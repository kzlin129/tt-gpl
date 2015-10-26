#include <linux/device.h>
#include <linux/pwm_backlight.h>
#include <linux/broadcom/lcd_backlight.h>

//#include <mach/lcd-dev.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <linux/vgpio.h>

#include <plat/irvine.h>

static int backlight_notify (int brightness);

struct platform_pwm_backlight_data bcm4760_pwm_data =
{
	.max_brightness = 60,
	/* default brightness is zero so just a short wakeup (from powerbutton or USB insert) does not flash the screen. */
	.dft_brightness = 0,
	.pwm_period_ns  = 37,
	.notify         = backlight_notify,
};

static struct resource lcd_backlight_resources[] = 
{
        {
                .start  = TT_VGPIO_LCD_BACKLIGHT_ENABLE,
                .end    = TT_VGPIO_LCD_BACKLIGHT_ENABLE,
                .flags  = IORESOURCE_IO,
        },
        {
                .start  = TT_VGPIO_LCD_BACKLIGHT_PWM,
                .end    = TT_VGPIO_LCD_BACKLIGHT_PWM,
                .flags  = IORESOURCE_IO,
        }
};

struct platform_device bcm_backlight_device = 
{
	.name		= "pwm-backlight",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lcd_backlight_resources),
	.resource	= lcd_backlight_resources,
	.dev = {
		.platform_data = &bcm4760_pwm_data, /* Passed to driver */
	},
};

static int backlight_notify (int brightness)
{
	if (brightness > 0) {
		gpio_set_value (TT_VGPIO_LCD_BACKLIGHT_ENABLE, 1);
	} else {
		gpio_set_value (TT_VGPIO_LCD_BACKLIGHT_ENABLE, 0);
	}

	/* XXX - doesn't even seem to be necessary
		INTEG: check this is true! 
		lcd_backlight_enable(brightness); 
	 */

	return brightness;
}

static int __init backlight_init (void)
{
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_BACKLIGHT_ENABLE), BCM4760_PIN_MUX_GPIO);
	
	gpio_request (TT_VGPIO_LCD_BACKLIGHT_ENABLE, "");
	gpio_request (TT_VGPIO_LCD_BACKLIGHT_PWM, "");

	gpio_direction_output (TT_VGPIO_LCD_BACKLIGHT_PWM, 1);
	/*initial backlight disable in the beginning for T3*/
	gpio_direction_output (TT_VGPIO_LCD_BACKLIGHT_ENABLE, 0);

	return platform_device_register(&bcm_backlight_device);
}

/*
lower backlight device register, so that backlight resume function will be called after lcd resume function.

Device suspend sequence:
...
backlight->suspend();
lcd->suspend();
...

Device resume sequence:
...
lcd->resume();
backlight->resume();
...
*/
late_initcall (backlight_init);
