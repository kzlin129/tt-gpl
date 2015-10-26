/* linux/arch/arm/plat-tomtom/mendoza/mendoza-fb.c
 *
 * Copyright 2009 TomTom B.V.
 *      Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>

#include <mach/gpio.h>
#include <plat/mendoza.h>

struct lcm_desc {
	char			*lcm_name;
	struct s3cfb_lcd	*lcm_info;
	int			(*gpio_setup)(void);
	int			(*backlight_on)(struct platform_device *pdev);
	int			(*reset_lcd)(struct platform_device *pdev);
	int			(*shutdown)(struct notifier_block *block, unsigned long code, void *null);
};
	

#define DBG(fmt,args...) do {} while(0)
static struct s3cfb_lcd a043fw05v4 = {
	.width = 480,
	.height = 272,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = (4+1),
		.h_bp = (38+1),
		.h_sw = (0+1),
		.v_fp = (7+1),
		.v_fpe = 1,
		.v_bp = (6+1),
		.v_bpe = 1,
		.v_sw = (0+1),
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};
	
/* lms500hf04 */
static struct s3cfb_lcd lms500hf04 = {
	.width = 480,
	.height = 272,
	.bpp = 24,
	.freq = 40,

	.timing = {
		.h_fp = 8,
		.h_bp = 45,
		.h_sw = 41,
		.v_fp = 7,
		.v_fpe = 1,
		.v_bp = 11,
		.v_bpe = 1,
		.v_sw = 1,
	},

	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* lms430wqv */
static struct s3cfb_lcd lms430wqv = {
	.width = 480,
	.height = 272,
	.bpp = 24,
	.freq = 40,

	.timing = {
		.h_fp = 8,
		.h_bp = 45,
		.h_sw = 41,
		.v_fp = 7,
		.v_fpe = 1,
		.v_bp = 11,
		.v_bpe = 1,
		.v_sw = 1,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static int mendoza_fb_reboot(struct notifier_block *block, unsigned long code, void *null)
{
	      pr_emerg("Shutting off LCM\n");
	      gpio_direction_output(TT_VGPIO_LCM_CS, 0);
	      gpio_direction_output(TT_VGPIO_LCM_PWR_EN, 0);
	      msleep(100);
	      return 0;
}

static int mendoza_backlight_on(struct platform_device *pdev)
{
		    return 0;
}

static int mendoza_reset_lcd(struct platform_device *pdev)
{
	      gpio_direction_output(TT_VGPIO_LCM_PWR_EN, 1);

        mdelay(100);
        gpio_direction_output(TT_VGPIO_LCM_CS, 0);
        mdelay(10);
        gpio_direction_output(TT_VGPIO_LCM_CS, 1);
        mdelay(10);

	return 0;
}

static int mendoza_gpio_setup(void)
{
        if (gpio_request(TT_VGPIO_LCM_PWR_EN, "TT_VGPIO_LCM_PWR_EN")) {
                pr_err("Failed to request TT_VGPIO_LCM_PWR_EN\n");
        }	
        //this is actual DISP_ON
        if (gpio_request(TT_VGPIO_LCM_CS, "TT_VGPIO_LCM_CS")) {
                pr_err("Failed to request TT_VGPIO_LCM_CS\n");
        } else {
                gpio_direction_output(TT_VGPIO_LCM_CS, 1);
        }

        return 0;
}

/* lms480wv */
static struct s3cfb_lcd lms480wv = {
	.width = 800,
	.height = 480,
	.bpp = 24,
	.freq = 30,

	.timing = {
		.h_fp = 16,
		.h_bp = 86,
		.h_sw = 13,
		.v_fp = 8,
		.v_fpe = 1,
		.v_bp = 19,
		.v_bpe = 1,
		.v_sw = 1,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};


static int mendoza_reset_spi_lcd(struct platform_device *pdev)
{
	return 0;
}

static int mendoza_gpio_spi_setup(void)
{
	return 0;
}

static struct lcm_desc mendoza_lcm[] __initdata = {
	{
		.lcm_name	= "lms430wqv",
		.lcm_info	= &lms430wqv,
		.gpio_setup	= mendoza_gpio_setup,
		.backlight_on	= mendoza_backlight_on,
		.reset_lcd	= mendoza_reset_lcd,
		.shutdown	= mendoza_fb_reboot,
	},
	{
		.lcm_name	= "lms480wv",
		.lcm_info	= &lms480wv,
		.gpio_setup	= mendoza_gpio_spi_setup,
		.backlight_on	= mendoza_backlight_on,
		.reset_lcd	= mendoza_reset_spi_lcd,
	},
	{
		.lcm_name	= "lms500hf04",
		.lcm_info	= &lms500hf04,
		.gpio_setup	= mendoza_gpio_setup,
		.backlight_on	= mendoza_backlight_on,
		.reset_lcd	= mendoza_reset_lcd,
		.shutdown	= mendoza_fb_reboot,
	},
	{
		.lcm_name	= "a043fw05v4",
		.lcm_info	= &a043fw05v4,
	},	
};

#ifdef CONFIG_CPU_S3C6410
static struct s3c_platform_fb mendoza_fb_data __initdata = {
	.hw_ver		= 0x40,
	.clk_name	= "lcd",
	.nr_wins	= 5,
	.default_win	= 0,
	.swap		= FB_SWAP_HWORD,
};
#endif

#ifdef CONFIG_CPU_S5P6440
static struct s3c_platform_fb mendoza_fb_data __initdata = {
        .hw_ver		= 0x50,
        .clk_name	= "lcd",
        .nr_wins	= 3,
        .default_win	= 0,
        .swap		= FB_SWAP_WORD | FB_SWAP_HWORD,
};
#endif

static struct notifier_block mendoza_fb_reboot_block;

void __init mendoza_s3cfb_setup(char *lcm_id)
{
	int i;

	pr_info("mendoza_s3cfb_setup: setting up %s\n", lcm_id);

	for (i = 0; i < ARRAY_SIZE(mendoza_lcm); i++) {
		if (!strcmp(mendoza_lcm[i].lcm_name, lcm_id)) {
			mendoza_fb_data.backlight_on = mendoza_lcm[i].backlight_on;
			mendoza_fb_data.reset_lcd = mendoza_lcm[i].reset_lcd;
			mendoza_fb_data.lcd = mendoza_lcm[i].lcm_info;

			if (mendoza_lcm[i].shutdown) {
				mendoza_fb_reboot_block.notifier_call = mendoza_lcm[i].shutdown;
				register_reboot_notifier(&mendoza_fb_reboot_block);
			}

			if (mendoza_lcm[i].gpio_setup)
				mendoza_lcm[i].gpio_setup();

			break;
		}
	}

	if (i >= ARRAY_SIZE(mendoza_lcm))
		pr_err("mendoza_s3cfb_setup: Could not get timings for LCM %s\n", lcm_id);

	s3cfb_set_platdata(&mendoza_fb_data);
}

