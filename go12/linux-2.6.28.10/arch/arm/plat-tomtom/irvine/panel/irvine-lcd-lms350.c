#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>

#include <mach/pinmux.h>

#define LMS350GF20_HBP  (0x21)
#define LMS350GF20_VBP  (0xf)

#define DEVICE_ID       (0x74)

static void lms350_write (unsigned char data)
{
	int i;
	
        /*
          According to LMS350 data-sheet, LCM driver IC latches SDI input data 
          at rising edge. Thus SPI data from SoC must be ready at falling edge.
        */
        
	for (i = 0; i < 8; i++) {
		gpio_set_value (TT_VGPIO_LCD_SCLK, 1);

		if (data & 0x80)
			gpio_set_value (TT_VGPIO_LCD_SDI, 1);
		else
			gpio_set_value (TT_VGPIO_LCD_SDI, 0);

		gpio_set_value (TT_VGPIO_LCD_SCLK, 0);
		data <<= 1;
	}

	gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
}

static void lms350_tx_data (unsigned char indx, unsigned short data)
{
	unsigned char dev_id = DEVICE_ID;
	unsigned char lcd_index = (unsigned char)(indx & 0xff);
	unsigned char lcd_data0 = (unsigned char)((data >> 8) & 0xff);
	unsigned char lcd_data1 = (unsigned char)(data & 0xff);

	gpio_set_value(TT_VGPIO_LCD_SCLK, 1);
	gpio_set_value(TT_VGPIO_LCD_CS, 1);
	
	gpio_set_value(TT_VGPIO_LCD_CS, 0);
	lms350_write (dev_id | 0x00);
	lms350_write (0x00);
	lms350_write (lcd_index);
	gpio_set_value (TT_VGPIO_LCD_CS, 1);

	mdelay(1);	

	gpio_set_value (TT_VGPIO_LCD_CS, 0);
	lms350_write(dev_id | 0x02);
	lms350_write(lcd_data0);
	lms350_write(lcd_data1);
	gpio_set_value (TT_VGPIO_LCD_CS, 1);
	mdelay(1);
}

static void lms350_display_on(void)
{
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
	mdelay(20);
	gpio_set_value(TT_VGPIO_LCD_ON, 0);
	mdelay(10);
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
	mdelay(10);

	lms350_tx_data (0x07, 0x0000);

	mdelay(15);
	lms350_tx_data (0x12,0x1018);
	lms350_tx_data (0x11,0x222a);
	lms350_tx_data (0x13,0x3dc5);
	lms350_tx_data (0x76,0x2213);
	lms350_tx_data (0x74,0x0001);
	lms350_tx_data (0x76,0x0000);
	lms350_tx_data (0x10,0x3504);
	mdelay(100);
	lms350_tx_data (0x12,0x1058);
	mdelay(80);
	lms350_tx_data (0x01,0x2b1d);
	lms350_tx_data (0x02,0x0300);
	lms350_tx_data (0x03,0xd000);
	lms350_tx_data (0x08,LMS350GF20_VBP);
	lms350_tx_data (0x09,LMS350GF20_HBP);

	lms350_tx_data (0x76,0x2213);
	lms350_tx_data (0x0b,0x37e0);
	lms350_tx_data (0x0c,0x0020);
	lms350_tx_data (0x1c,0x6650);
	lms350_tx_data (0x76,0x0000);
	lms350_tx_data (0x0d,0x0005);
	lms350_tx_data (0x0e,0x0000);
	lms350_tx_data (0x14,0x0000);
	lms350_tx_data (0x15,0x0803);
	lms350_tx_data (0x16,0x000a);
	lms350_tx_data (0x30,0x0200);
	lms350_tx_data (0x31,0x0707);
	lms350_tx_data (0x32,0x0204);
	lms350_tx_data (0x33,0x0602);
	lms350_tx_data (0x34,0x0707);
	lms350_tx_data (0x35,0x0708);
	lms350_tx_data (0x36,0x0006);
	lms350_tx_data (0x37,0x0206);
	lms350_tx_data (0x38,0x0f06);
	lms350_tx_data (0x39,0x0611);

	lms350_tx_data (0x07,0x0001);
	mdelay(30);
	lms350_tx_data (0x07,0x0101);
	mdelay(30);
	lms350_tx_data (0x76,0x2213);
	lms350_tx_data (0x1c,0x6650);

        /* R0b = 0x33e0 is gone in Rev.005 data-sheet. */
	/* lms350_tx_data (0x0b,0x33e0); */

	lms350_tx_data (0x76,0x0000);
	lms350_tx_data (0x07,0x0103);
}

static void lms350_display_off(void)
{
	lms350_tx_data (0x0b,0x3000);
	lms350_tx_data (0x07,0x0102);
	mdelay(30);
	lms350_tx_data (0x07,0x0000);
	mdelay(30);
	lms350_tx_data (0x12,0x0000);
	lms350_tx_data (0x10,0x0100);

        /* R07 = 0x0000 is gone in Rev.005 data-sheet. */
	/* lms350_tx_data (0x07,0x0000); */                

	lms350_tx_data (0x10,0x0001);
	lms350_tx_data (0x11,0x0000);
	lms350_tx_data (0x12,0x0000);

	gpio_set_value (TT_VGPIO_LCD_ON, 1);
	gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
	gpio_set_value (TT_VGPIO_LCD_SDI, 1);
	gpio_set_value (TT_VGPIO_LCD_CS, 1); 
}

static void lms350_enable (struct clk *clk)
{
}

static void lms350_disable (struct clk *clk)
{
}

static void lms350_suspend (struct clk *clk)
{
        lms350_display_off();
}

static void lms350_resume (struct clk *clk)
{
        lms350_display_on();
}

static void lms350_init (struct clk *clk)
{
        lms350_display_on();
}

static void lms350_setup (struct clk *clk)
{
	printk (KERN_INFO "Panel: LMS350\n");

	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_ON),   BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_SCLK), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_SDI),  BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_CS),   BCM4760_PIN_MUX_GPIO);

	gpio_request (TT_VGPIO_LCD_ON, "");
	gpio_request (TT_VGPIO_LCD_SCLK, "");
	gpio_request (TT_VGPIO_LCD_SDI, "");
	gpio_request (TT_VGPIO_LCD_CS, "");

	gpio_direction_output (TT_VGPIO_LCD_ON, 0);
	gpio_direction_output (TT_VGPIO_LCD_SCLK, 1);
	gpio_direction_output (TT_VGPIO_LCD_SDI, 1);
	gpio_direction_output (TT_VGPIO_LCD_CS, 1);
}

static struct clcd_panel_cntrl cntrl = {
	.enable		= lms350_enable,
	.disable	= lms350_disable,
	.suspend	= lms350_suspend,
	.resume		= lms350_resume,
	.init		= lms350_init,
	.setup		= lms350_setup
};

static struct clcd_panel timings = 
{
	.mode = {
		.name   	= "LMS350",
		.refresh	= 60,   /* probably ignored, as it is bogus for all listed panels */
		.xres   	= 320,
		.yres   	= 240,
		.pixclock       = 111000, /* was 111000 */
	
		.left_margin    = 19,   /* horizontal back porch */
		.right_margin   = 17,    /* horizontal front porch */
		.upper_margin   = 14,    /* vertical back porch */
		.lower_margin   = 7,    /* vertical front porch */

		.hsync_len      = 14,   /* horizontal pulse width */
		.vsync_len      = 3,   /* vertical pulse width */
		.sync   	= 0,    /* horizontal and vertical both active low */
		.vmode  = FB_VMODE_NONINTERLACED,
	},
	
	.width  = -1,
	.height = -1,
	.tim2   = 0x013f18c5,
	.tim3   = 0,
	.cntl   = CNTL_LCDPWR | CNTL_LCDVCOMP(1) | CNTL_LCDTFT  | CNTL_LCDBPP16_565 | CNTL_BGR,
	.bpp    = 16,
};

static struct clcd_device lms350 = {
	.timings= &timings,
	.cntrl	= &cntrl,
};

struct clcd_paneldev LMS350_WithPLL =
{
	.panel 		= &lms350,
	.lcd_mdiv	= 0x00000018,
};

//arch_initcall(irvine_lcd_init);

