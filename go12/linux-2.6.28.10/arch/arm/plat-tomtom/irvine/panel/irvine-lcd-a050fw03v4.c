#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>

#include <mach/pinmux.h>

#define LCM_MODEL_NAME  "A050FW03V4"

#define HBP     (39)
#define HFP     (5)
#define HSW     (1)
#define PPL     (480)

#define VBP     (7)
#define VFP     (8)
#define VSW     (1)
#define LPP     (272)

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

static inline void panel_sleep(unsigned int ms)
{
	if (in_atomic()) {
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

static void a050fw03v4_serial_write (unsigned short data)
{
	int i;
	        
	for (i = 0; i < 16; i++) {
		gpio_set_value (TT_VGPIO_LCD_SCLK, 0);

		if (data & 0x8000)
			gpio_set_value (TT_VGPIO_LCD_SDI, 1);
		else
			gpio_set_value (TT_VGPIO_LCD_SDI, 0);

                data <<= 1;
		gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
		gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
	}
}

static void a050fw03v4_tx_data (unsigned char indx, unsigned char data)
{
        unsigned short value;

        gpio_set_value(TT_VGPIO_LCD_SCLK, 1);
	gpio_set_value(TT_VGPIO_LCD_CS, 1);
        
        value = indx << 1 | 0x0 << 0;
        value = value << 8 | data;

        gpio_set_value(TT_VGPIO_LCD_CS, 0);
        a050fw03v4_serial_write(value);
        gpio_set_value(TT_VGPIO_LCD_CS, 1);

        panel_sleep(1);
}

static void a050fw03v4_display_on(void)
{
	panel_sleep(5); /* Delay more than 2 ms */

        a050fw03v4_tx_data(5, 0x1E); /* Reset / Standby */
        a050fw03v4_tx_data(5, 0x5E); /* Reset / Standby */
        a050fw03v4_tx_data(5, 0x5F); /* Reset / Standby */
        a050fw03v4_tx_data(2, 0x1F); /* Contrast (default) */
        a050fw03v4_tx_data(3, 0x40); /* Brightness (default) */
        a050fw03v4_tx_data(6, 0x08); /* HBLK_EN / VBLK (default) */
        a050fw03v4_tx_data(7, 0x28); /* HBLK (default) */
        a050fw03v4_tx_data(8, 0x4);  /* DE mode */
        a050fw03v4_tx_data(12, 0 << 3 | 1 << 2 | 1 << 1 | 1 << 0);

	panel_sleep(25); /* Delay more than 20 ms*/

	gpio_set_value(TT_VGPIO_LCD_ON, 1); /* Pull DISP high*/
}

static void a050fw03v4_display_off(void)
{
	u32 val;

        a050fw03v4_tx_data(5, 0x5E);    /* Reset / Standby */
        panel_sleep(20);                /* > 1 frame */
        gpio_set_value(TT_VGPIO_LCD_ON, 0);
	panel_sleep(200);               /* > 10 frames */
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

static void a050fw03v4_enable (struct clk *clk)
{
	a050fw03v4_display_on();
}

static void a050fw03v4_disable (struct clk *clk)
{
}

static void a050fw03v4_suspend (struct clk *clk)
{
        a050fw03v4_display_off();
}

static void a050fw03v4_resume (struct clk *clk)
{
}

static void a050fw03v4_init (struct clk *clk)
{
        a050fw03v4_display_on();
}

static void a050fw03v4_setup (struct clk *clk)
{
	printk ("Panel: %s\n", LCM_MODEL_NAME);

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
	.enable		= a050fw03v4_enable,
	.disable	= a050fw03v4_disable,
	.suspend	= a050fw03v4_suspend,
	.resume		= a050fw03v4_resume,
	.init		= a050fw03v4_init,
	.setup		= a050fw03v4_setup,
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

static struct clcd_device a050fw03v4 = {
	.timings = &timings,
	.cntrl 	 = &cntrl,
};

struct clcd_paneldev A050FW03V4_WithPLL =
{
	.panel 		= &a050fw03v4,
	.lcd_mdiv	= 0x00000018,
};

//arch_initcall(irvine_lcd_init);

