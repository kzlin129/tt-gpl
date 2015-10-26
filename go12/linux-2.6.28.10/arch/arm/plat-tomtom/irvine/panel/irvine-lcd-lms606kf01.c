#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>
#include <plat/lcd.h>

#include <mach/pinmux.h>

#define LCM_MODEL_NAME  "LMS606KF01"

#define HBP     (8)
#define HFP     (8)
#define HSW     (10)
#define PPL     (480)

#define VBP     (8)
#define VFP     (8)
#define VSW     (2)
#define LPP     (800)

#define PCD_HI  (0)
#define BCD     (0)
#define CPL     (PPL-1)
#define IOE     (0)
#define IPC     (1)
#define IHS     (1)
#define IVS     (1)
#define ACB     (0)
#define CLKSEL  (0)
#define PCD_LO  (3)



static void lms606kf01_enable (struct clk *clk)
{
}

static void lms606kf01_disable (struct clk *clk)
{
}

static void lms606kf01_suspend (struct clk *clk)
{
}

static void lms606kf01_resume (struct clk *clk)
{
}

static void lms606kf01_init (struct clk *clk)
{
}

static void lms606kf01_lcd_on(int v)
{
	gpio_direction_output (TT_VGPIO_LCD_ON, v);		
}

static struct lcd_info lms606kf01_lcd_info = {
	.lcd_on = lms606kf01_lcd_on,
};

static struct spi_board_info spi_board_info[] = {
	[0]= {
		.modalias       = "lcm-lms606kf01",
		.max_speed_hz   = 1333333, /* 1.333333 MHz */
		.bus_num        = 0,
		.chip_select    = 0,
		.mode		= SPI_MODE_3 ,
		.platform_data = &lms606kf01_lcd_info,
	},
};

static void lms606kf01_setup (struct clk *clk)
{
	printk (KERN_INFO "Panel: LMS606KF01\n");
	
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_ON),   BCM4760_PIN_MUX_GPIO);
	gpio_request (TT_VGPIO_LCD_ON, "");		
	
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static struct clcd_panel_cntrl cntrl = {
	.enable		= lms606kf01_enable,
	.disable	= lms606kf01_disable,
	.suspend	= lms606kf01_suspend,
	.resume		= lms606kf01_resume,
	.init		= lms606kf01_init,
	.setup		= lms606kf01_setup
};

static struct clcd_panel timings = 
{
	.mode = {
		.name   	= LCM_MODEL_NAME,
		.refresh	= 60,   /* probably ignored, as it is bogus for all listed panels */
		.xres   	= PPL,
		.yres   	= LPP,
		.pixclock       = 90909,
	
		.left_margin    = HBP,  /* horizontal back porch */
		.right_margin   = HFP,  /* horizontal front porch */
		.upper_margin   = VBP,  /* vertical back porch */
		.lower_margin   = VFP,  /* vertical front porch */

		.hsync_len      = HSW,  /* horizontal pulse width */
		.vsync_len      = VSW,  /* vertical pulse width */
		.sync   	= 0,    /* horizontal and vertical both active low */
		.vmode  	= FB_VMODE_NONINTERLACED,
	},
	
	.width  = -1,
	.height = -1,
	.tim2   = PCD_HI << 27 | BCD << 26 | CPL << 16 | IOE << 14 | IPC << 13 | IHS << 12 | \
                  IVS << 11 | ACB << 6 | CLKSEL << 5 | PCD_LO << 0,
	.tim3   = 0,
	.cntl   = CNTL_LCDPWR | CNTL_LCDVCOMP(1) | CNTL_LCDTFT  | CNTL_LCDBPP16_565 | CNTL_BGR,
	.bpp    = 16,
};

static struct clcd_device lms606kf01 = {
	.timings= &timings,
	.cntrl	= &cntrl,
};

struct clcd_paneldev LMS606KF01_WithPLL =
{
	.panel 		= &lms606kf01,
	.lcd_mdiv	= 0x0000000A,
};


