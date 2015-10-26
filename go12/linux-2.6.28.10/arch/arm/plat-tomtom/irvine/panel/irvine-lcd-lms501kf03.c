#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/delay.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/irvine.h>
#include <plat/irvine-lcd.h>

#include <mach/pinmux.h>

#define LCM_MODEL_NAME  "LMS501KF03"

#define HBP     (8)
#define HFP     (8)
#define HSW     (6)
#define PPL     (480)

#define VBP     (6)
#define VFP     (6)
#define VSW     (4)
#define LPP     (800)

#define PCD_HI  (0)
#define BCD     (0)
#define CPL     (PPL-1)
#define IOE     (0)
#define IPC     (0)
#define IHS     (1)
#define IVS     (1)
#define ACB     (0)
#define CLKSEL  (0)
#define PCD_LO  (1)



/* Set password */
static unsigned char set_passwd[] = 
		{ 
			0xB9, 0xFF, 0x83, 0x69 
		};
#define SIZE_SET_PASSWD		(sizeof(set_passwd) / sizeof(char))

/* Set power */
static unsigned char set_power[] = 
		{ 
			0xB1, 0x01, 0x00, 0x34, 0x06, 0x00, 0x14, 0x14, 0x20, 0x28, 
			0x12, 0x12, 0x17, 0x0A, 0x01, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6 
		};
#define SIZE_SET_POWER		(sizeof(set_power) / sizeof(char))

/* Set display */
static unsigned char set_display[] = 
		{ 
			0xB2, 0x00, 0x2B, 0x03, 0x03, 0x70, 0x00, 0xFF, 0x00, 0x00,
			0x00, 0x00, 0x03, 0x03, 0x00, 0x01
		};
#define SIZE_SET_DISPLAY	(sizeof(set_display) / sizeof(char))

/* Set RGBIF */
static unsigned char set_rgbif[] =
		{
			0xB3, 0x09
		};
#define SIZE_SET_RGBIF	(sizeof(set_rgbif) / sizeof(char))

/* Set Disp Inv. */
static unsigned char set_dispinv[] = 
		{
			0xB4, 0x01, 0x08, 0x77, 0x0E, 0x06
		};
#define SIZE_SET_DISPINV	(sizeof(set_dispinv) / sizeof(char))

/* Set Vcom */
static unsigned char set_vcom[] =
		{
			0xB6, 0x4C, 0x2E
		};
#define SIZE_SET_VCOM	(sizeof(set_vcom) / sizeof(char))

/* Set Gate */
static unsigned char set_gate[] =
		{
			0xD5, 0x00, 0x05, 0x03, 0x29, 0x01, 0x07, 0x17, 0x68, 0x13,
			0x37, 0x20, 0x31, 0x8A, 0x46, 0x9B, 0x57, 0x13, 0x02, 0x75,
			0xB9, 0x64, 0xA8, 0x07, 0x0F, 0x04, 0x07
		};
#define SIZE_SET_GATE	(sizeof(set_gate) / sizeof(char))

/* Set Panel */
static unsigned char set_panel[] =
		{
			0xCC, 0x0A
		};
#define SIZE_SET_PANEL	(sizeof(set_panel) / sizeof(char))

/* Set COLMOD */
static unsigned char set_colmod[] =
		{
			0x3A, 0x77
		};
#define SIZE_SET_COLMOD	(sizeof(set_colmod) / sizeof(char))

/* Set W Gamma */
static unsigned char set_w_gamma[] =
		{
			0xE0, 0x00, 0x04, 0x09, 0x0F, 0x1F, 0x3F, 0x1F, 0x2F, 0x0A,
			0x0F, 0x10, 0x16, 0x18, 0x16, 0x17, 0x0D, 0x15, 0x00, 0x04,
			0x09, 0x0F, 0x38, 0x3F, 0x20, 0x39, 0x0A, 0x0F, 0x10, 0x16, 
			0x18, 0x16, 0x17, 0x0D, 0x15 
		};
#define SIZE_SET_W_GAMMA	(sizeof(set_w_gamma) / sizeof(char))

/* Set RGB Gamma */
static unsigned char set_rgb_gamma[] =
		{
			0xC1, 0x01, 0x03, 0x07, 0x0F, 0x1A, 0x22, 0x2C, 0x33, 0x3C, 
			0x46, 0x4F, 0x58, 0x60, 0x69, 0x71, 0x79, 0x82, 0x89, 0x92, 
			0x9A, 0xA1, 0xA9, 0xB1, 0xB9, 0xC1, 0xC9, 0xCF, 0xD6, 0xDE, 
			0xE5, 0xEC, 0xF3, 0xF9, 0xFF, 0xDD, 0x39, 0x07, 0x1C, 0xCB, 
			0xAB, 0x5F, 0x49, 0x80, 0x03, 0x07, 0x0F, 0x19, 0x20, 0x2A, 
			0x31, 0x39, 0x42, 0x4B, 0x53, 0x5B, 0x63, 0x6B, 0x73, 0x7B, 
			0x83, 0x8A, 0x92, 0x9B, 0xA2, 0xAA, 0xB2, 0xBA, 0xC2, 0xCA, 
			0xD0, 0xD8, 0xE1, 0xE8, 0xF0, 0xF8, 0xFF, 0xF7, 0xD8, 0xBE, 
			0xA7, 0x39, 0x40, 0x85, 0x8C, 0xC0, 0x04, 0x07, 0x0C, 0x17, 
			0x1C, 0x23, 0x2B, 0x34, 0x3B, 0x43, 0x4C, 0x54, 0x5B, 0x63, 
			0x6A, 0x73, 0x7A, 0x82, 0x8A, 0x91, 0x98, 0xA1, 0xA8, 0xB0, 
			0xB7, 0xC1, 0xC9, 0xCF, 0xD9, 0xE3, 0xEA, 0xF4, 0xFF, 0x00, 
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		};
#define SIZE_SET_RGB_GAMMA	(sizeof(set_rgb_gamma) / sizeof(char))

/* Set UPDN */
static unsigned char set_updn[] =
		{
			0x36, 0x10
		};
#define SIZE_SET_UPDN	(sizeof(set_updn) / sizeof(char))

/* Sleep Out */
static unsigned char set_sleep_out[] =  { 0x11 };
#define SIZE_SLEEP_OUT	(sizeof(set_sleep_out) / sizeof(char))

/* Disp On */
static unsigned char set_disp_on[] = { 0x29 };
#define SIZE_DISP_ON	(sizeof(set_disp_on) / sizeof(char))

/* Sleep in */
static unsigned char set_sleep_in[] = { 0x10 };
#define SIZE_SLEEP_IN	(sizeof(set_sleep_in) / sizeof(char))

#define SPI_COMMAND	(0x00)
#define SPI_DATA	(0x01)

static inline void spi_write(unsigned char is_data, unsigned char data)
{
	int i;
	
	gpio_set_value(TT_VGPIO_LCD_CS, 0);
	
    if(is_data)
    {
        /* control bit is 1 */
        gpio_set_value (TT_VGPIO_LCD_SCLK, 0);
        gpio_set_value (TT_VGPIO_LCD_SDI, 1);
        gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
    }
    else
    {    
        /* control bit is 0 */
        gpio_set_value (TT_VGPIO_LCD_SCLK, 0);
        gpio_set_value (TT_VGPIO_LCD_SDI, 0);
        gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
    }
            
	for (i = 0; i < 8; i++) {
		gpio_set_value (TT_VGPIO_LCD_SCLK, 0);

		if (data & 0x80)
			gpio_set_value (TT_VGPIO_LCD_SDI, 1);
		else
			gpio_set_value (TT_VGPIO_LCD_SDI, 0);

		gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
		data <<= 1;
	}

	//gpio_set_value (TT_VGPIO_LCD_SCLK, 0);
	gpio_set_value(TT_VGPIO_LCD_CS, 1);
    udelay(10);
    gpio_set_value(TT_VGPIO_LCD_SDI, 1);
}

#define spi_write_command(x) spi_write(0,x)
#define spi_write_data(x)  spi_write(1,x)

static void spi_write_set(unsigned char * data, int len)
{
	int i;	
    spi_write_command(data[0]);

	for (i = 1; i < len; i++) {
		spi_write_data(data[i]);
	}
}



static void lms501kf03_display_on(void)
{
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
	mdelay(10);
	gpio_set_value(TT_VGPIO_LCD_ON, 0);
	mdelay(50);
	gpio_set_value(TT_VGPIO_LCD_ON, 1);
	mdelay(20);

	spi_write_set(set_passwd, SIZE_SET_PASSWD);
	spi_write_set(set_power, SIZE_SET_POWER);
	spi_write_set(set_display, SIZE_SET_DISPLAY);
	spi_write_set(set_rgbif, SIZE_SET_RGBIF);
	spi_write_set(set_dispinv, SIZE_SET_DISPINV);
	spi_write_set(set_vcom, SIZE_SET_VCOM);
	spi_write_set(set_gate, SIZE_SET_GATE);
	spi_write_set(set_panel, SIZE_SET_PANEL);
	spi_write_set(set_colmod, SIZE_SET_COLMOD);
	spi_write_set(set_w_gamma, SIZE_SET_W_GAMMA);
	spi_write_set(set_rgb_gamma, SIZE_SET_RGB_GAMMA);
	spi_write_set(set_updn, SIZE_SET_UPDN);
	spi_write_set(set_sleep_out, SIZE_SLEEP_OUT);

	mdelay(120);

	spi_write_set(set_disp_on, SIZE_DISP_ON);    
}

static void lms501kf03_display_off(void)
{
	/* Sleep in */
	spi_write_set(set_sleep_in, SIZE_SLEEP_IN);

	mdelay(120);

	gpio_set_value (TT_VGPIO_LCD_ON, 1);
	gpio_set_value (TT_VGPIO_LCD_SCLK, 1);
	gpio_set_value (TT_VGPIO_LCD_SDI, 1);
	gpio_set_value (TT_VGPIO_LCD_CS, 1); 
}

static void lms501kf03_enable (struct clk *clk)
{
}

static void lms501kf03_disable (struct clk *clk)
{
}

static void lms501kf03_suspend (struct clk *clk)
{
        lms501kf03_display_off();
}

static void lms501kf03_resume (struct clk *clk)
{
        lms501kf03_display_on();
}

static void lms501kf03_init (struct clk *clk)
{
        lms501kf03_display_on();
}

static void lms501kf03_setup (struct clk *clk)
{
	printk (KERN_INFO "Panel: LMS501KF03\n");

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
	.enable		= lms501kf03_enable,
	.disable	= lms501kf03_disable,
	.suspend	= lms501kf03_suspend,
	.resume		= lms501kf03_resume,
	.init		= lms501kf03_init,
	.setup		= lms501kf03_setup
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

static struct clcd_device lms501kf03 = {
	.timings= &timings,
	.cntrl	= &cntrl,
};

struct clcd_paneldev LMS501KF03_WithPLL =
{
	.panel 		= &lms501kf03,
	.lcd_mdiv	= 0x00000011,
};

//arch_initcall(irvine_lcd_init);

