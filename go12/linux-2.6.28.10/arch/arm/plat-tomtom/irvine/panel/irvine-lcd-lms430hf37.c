#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>

#include <mach/pinmux.h>

static inline void panel_sleep(unsigned int ms)
{
	if (in_atomic()) {
		mdelay(ms);
	} else {
		msleep(ms);
	}
}


static void lms430hf37_enable (struct clk *clk)
{
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
}

static void lms430hf37_disable (struct clk *clk)
{
	gpio_set_value(TT_VGPIO_LCD_ON, 0);
	panel_sleep(150);				/* T5-T4 should be greater than 6 frames */
}

static void lms430hf37_suspend (struct clk *clk)
{
	u32 val;
	
	gpio_set_value(TT_VGPIO_LCD_ON, 0);
	gpio_direction_input(TT_VGPIO_LCD_ON);
	panel_sleep(150);			/* T5-T4 should be greater than 6 frames */

	/* disabling clk, CNTL_LCDPWR and CNTL_LCDEN before suspend 
	 * doesn't happen in bcm4760_clcd.c that's why it's added
	 * here to be compliant with LCM power sequence specs
	 */

	val = readl(IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
	if (val & CNTL_LCDPWR){
		val &= ~CNTL_LCDPWR;
		writel(val, IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
		panel_sleep(20);
	}
	if (val & CNTL_LCDEN) {
		val &= ~CNTL_LCDEN;
		writel(val, IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
	}
	clk_disable( clk);
}

static void lms430hf37_resume (struct clk *clk)
{
	gpio_direction_output(TT_VGPIO_LCD_ON, 0);
	panel_sleep(200);			/* T3 should be greater than 11 frames */

	/* enabling clk, CNTL_LCDPWR and CNTL_LCDEN after resume happens in bcm4760_clcd.c */
}

static void lms430hf37_init (struct clk *clk)
{
	gpio_set_value(TT_VGPIO_LCD_ON, 0);	/* Initially LCD_OFF in the begining of T2 */
}

static void lms430hf37_setup (struct clk *clk)
{
	printk (KERN_INFO "Panel: LMS430HF37\n");

	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_ON), BCM4760_PIN_MUX_GPIO);

	gpio_request (TT_VGPIO_LCD_ON, "");
	gpio_direction_output (TT_VGPIO_LCD_ON, 0);
}

static struct clcd_panel_cntrl cntrl = {
	.enable		= lms430hf37_enable,
	.disable	= lms430hf37_disable,
	.suspend	= lms430hf37_suspend,
	.resume		= lms430hf37_resume,
	.init		= lms430hf37_init,
	.setup		= lms430hf37_setup,
};

static struct clcd_panel timings =
{
	.mode = {
		.name   	= "LMS430HF37",
		.refresh	= 60,   /* probably ignored, as it is bogus for all listed panels */
		.xres   	= 480,
		.yres   	= 272,
		.pixclock       = 90909,
	
		.left_margin    = 46,   /* horizontal back porch */
		.right_margin   = 8,    /* horizontal front porch */
		.upper_margin   = 1,    /* vertical back porch */
		.lower_margin   = 3,    /* vertical front porch */

		.hsync_len      = 42,   /* horizontal pulse width */
		.vsync_len      = 11,   /* vertical pulse width */
		.sync   	= 0,    /* horizontal and vertical both active low */
		.vmode  	= FB_VMODE_NONINTERLACED,
	},
	
	.width  = -1,
	.height = -1,
	.tim2   = 0x01df18c3,
	.tim3   = 0,
	.cntl   = CNTL_LCDPWR | CNTL_LCDVCOMP(1) | CNTL_LCDTFT  | CNTL_LCDBPP16_565 | CNTL_BGR,
	.bpp    = 16,
};

static struct clcd_device lms430hf37 = {
	.timings = &timings,
	.cntrl 	 = &cntrl,
};

struct clcd_paneldev LMS430HF37_WithPLL =
{
	.panel 		= &lms430hf37,
	.lcd_mdiv	= 0x00000018,
};

//arch_initcall(irvine_lcd_init);

