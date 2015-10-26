#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>

#include <mach/pinmux.h>

#define LCM_MODEL_NAME  "LD050WQ1"

#define HBP     (39)
#define HFP     (5)
#define HSW     (1)
#define PPL     (480)

#define VBP     (7)
#define VFP     (6)
#define VSW     (1)
#define LPP     (272)

#define PCD_HI  (0)
#define BCD     (0)
#define CPL     (PPL-1)
#define IOE     (0)
#define IPC     (0)
#define IHS     (1)
#define IVS     (1)
#define ACB     (0)
#define CLKSEL  (0)
#define PCD_LO  (3)

static inline void panel_sleep(unsigned int ms)
{
	if (in_atomic()) {
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

static void ld050wq1_display_on(void)
{
        panel_sleep(20);
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
}

static void ld050wq1_display_off(void)
{
	u32 val;

	gpio_set_value(TT_VGPIO_LCD_ON, 0);
	panel_sleep(200);
	val = readl(IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
	if (val & CNTL_LCDPWR) {
		val &= ~CNTL_LCDPWR;
		writel(val, IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
		panel_sleep(20);
	}
	if (val & CNTL_LCDEN) {
		val &= ~CNTL_LCDEN;
		writel(val, IO_ADDRESS(LCD_REG_BASE_ADDR + CLCD_CNTL));
	}
}

static void ld050wq1_enable (struct clk *clk)
{
	ld050wq1_display_on();
}

static void ld050wq1_disable (struct clk *clk)
{
}

static void ld050wq1_suspend (struct clk *clk)
{
        ld050wq1_display_off();
}

static void ld050wq1_resume (struct clk *clk)
{        
}

static void ld050wq1_init (struct clk *clk)
{
        ld050wq1_display_on();
}

static void ld050wq1_setup (struct clk *clk)
{
	printk ("Panel: %s\n", LCM_MODEL_NAME);

 	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_LCD_ON),   BCM4760_PIN_MUX_GPIO);
	gpio_request (TT_VGPIO_LCD_ON, "");
	gpio_direction_output (TT_VGPIO_LCD_ON, 0);
}

static struct clcd_panel_cntrl cntrl = {
	.enable		= ld050wq1_enable,
	.disable	= ld050wq1_disable,
	.suspend	= ld050wq1_suspend,
	.resume		= ld050wq1_resume,
	.init		= ld050wq1_init,
	.setup		= ld050wq1_setup,
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

static struct clcd_device ld050wq1 = {
	.timings = &timings,
	.cntrl 	 = &cntrl,
};

struct clcd_paneldev LD050WQ1_WithPLL =
{
	.panel 		= &ld050wq1,
	.lcd_mdiv	= 0x00000018,
};

//arch_initcall(irvine_lcd_init);

