/* linux/arch/arm/mach-s5p6440/cordoba.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 * Copyright 2009 TomTom B.V.
 *	Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/pwm_backlight.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/vgpio.h>
#include <linux/mfd/s5m8751/s5m8751_core.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-mem.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>

#include <plat/s5p6440.h>
#include <plat/clock.h>
#include <plat/regs-clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/pm.h>
#include <plat/pll.h>

#include <plat/regs-rtc.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/regs-otg.h>
#include <linux/usb/ch9.h>
#include <linux/usb/s3c-otg.h>

#include <plat/mendoza.h>
#include <plat/mendoza_s5m8751.h>
#include <plat/adau1761_pdata.h>
#include <mach/cordoba.h>

#include <plat/fdt.h>
#include <plat/tt_setup_handler.h>

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

extern void s3c64xx_reserve_bootmem(void);

static struct s3c2410_uartcfg cordoba_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = S3C64XX_UCON_DEFAULT,
		.ulcon	     = S3C64XX_ULCON_DEFAULT,
		.ufcon	     = S3C64XX_UFCON_DEFAULT,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = S3C64XX_UCON_DEFAULT,
		.ulcon	     = S3C64XX_ULCON_DEFAULT,
		.ufcon	     = S3C64XX_UFCON_DEFAULT,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = S3C64XX_UCON_DEFAULT,
		.ulcon	     = S3C64XX_ULCON_DEFAULT,
		.ufcon	     = S3C64XX_UFCON_DEFAULT,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = S3C64XX_UCON_DEFAULT,
		.ulcon	     = S3C64XX_ULCON_DEFAULT,
		.ufcon	     = S3C64XX_UFCON_DEFAULT,
	},	
};

struct map_desc cordoba_iodesc[] = {};

static int cordoba_vgpio1_setup(struct platform_device *dev, void *context);

// Some macros, just to save some typing...
#define TT_DEF_PIN0(p,t)         VGPIO_DEF_PIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_INVPIN0(p,t)      VGPIO_DEF_INVPIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_NCPIN0(p)         VGPIO_DEF_NCPIN(TT_VGPIO0_BASE, TT_VGPIO_##p)
#define TT_DEF_PIN1(p,t)         VGPIO_DEF_PIN(TT_VGPIO1_BASE, TT_VGPIO_##p, t)
#define TT_DEF_INVPIN1(p,t)      VGPIO_DEF_INVPIN(TT_VGPIO1_BASE, TT_VGPIO_##p, t)

// Pin tables
static struct vgpio_pin cordoba_vgpio0_pins[] = {
	TT_DEF_PIN0(PU_I2C0,		S5P64XX_GPP(4)),
	TT_DEF_PIN0(PU_I2C1,		S5P64XX_GPP(8)),
	TT_DEF_PIN0(BOOT_DEV0,		S5P64XX_GPN(13)),
	TT_DEF_PIN0(SD0_PWR_EN,		S5P64XX_GPN(11)),
	TT_DEF_PIN0(BARRACUDA_CS,	S5P64XX_GPC(3)),
	TT_DEF_PIN0(BACKLIGHT_EN,	S5P64XX_GPR(1)),
	TT_DEF_PIN0(LCM_RESET,		S5P64XX_GPR(0)),
	TT_DEF_PIN0(PWR_BUTTON,		S5P64XX_GPN(1)),
	TT_DEF_PIN0(RFS_BOOT_CLK,	S5P64XX_GPB(4)),
	TT_DEF_PIN0(RFS_BOOT_Q,		S5P64XX_GPR(3)),
	TT_DEF_PIN0(USB_HOST_DETECT,	S5P64XX_GPN(6)),
	TT_DEF_INVPIN0(GPS_STANDBY, 	S5P64XX_GPP(7)),
	TT_DEF_INVPIN0(GPS_RESET, 	S5P64XX_GPR(2)),
	TT_DEF_INVPIN0(BT_RST,		S5P64XX_GPC(7)),
	TT_DEF_PIN0(WALL_ON,		S5P64XX_GPP(3)),
	TT_DEF_INVPIN0(SD_CD,		S5P64XX_GPG(6)),
	TT_DEF_PIN0(SD_BASE,		S5P64XX_GPG(0)),
	TT_DEF_PIN0(SD_BOOT_BASE,	S5P64XX_GPH(0)),
	TT_DEF_PIN0(PWR_KILL,		S5P64XX_GPR(5)),
	TT_DEF_PIN0(DOCK_DET0,		S5P64XX_GPN(3)),
	TT_DEF_PIN0(DOCK_DET1,		S5P64XX_GPN(2)),
	TT_DEF_NCPIN0(MIC_STBY),
};

static struct vgpio_pin cordoba_vgpio1_pins[] = {
	TT_DEF_PIN1(GSM_SYS_EN,		CORDOBA_GPIO_EXP_IO1(3)),
	TT_DEF_PIN1(GSM_SYS_RST,	CORDOBA_GPIO_EXP_IO1(1)),
	TT_DEF_PIN1(LCM_CS,		CORDOBA_GPIO_EXP_IO1(5)),
	TT_DEF_PIN1(LCM_PWR_EN,		CORDOBA_GPIO_EXP_IO1(2)),
	TT_DEF_PIN1(CODEC_PWR_EN,	CORDOBA_GPIO_EXP_IO1(4)),
	TT_DEF_PIN1(AMP_PWR_EN,		CORDOBA_GPIO_EXP_IO1(7)),
};

// Platform data & device
static struct vgpio_platform_data cordoba_vgpio_pdata[] = {
	[0] = {
        	.gpio_base      = TT_VGPIO0_BASE,
        	.gpio_number    = ARRAY_SIZE(cordoba_vgpio0_pins),
        	.pins           = cordoba_vgpio0_pins,
	},
	[1] = {
        	.gpio_base      = TT_VGPIO1_BASE,
        	.gpio_number    = ARRAY_SIZE(cordoba_vgpio1_pins),
        	.pins           = cordoba_vgpio1_pins,
		.setup		= cordoba_vgpio1_setup,
	},
};

static struct platform_device cordoba_vgpio[] = {
	[0] = {
        	.name           = "vgpio",
        	.id             = 0,
        	.dev            = {
                	.platform_data  = &cordoba_vgpio_pdata[0],
        	},
	},
	[1] = {
        	.name           = "vgpio",
        	.id             = 1,
        	.dev            = {
                	.platform_data  = &cordoba_vgpio_pdata[1],
        	},
	},
};

/* First setup whatever oddity the HW requires... */
static void cordoba_quirk_setup(void)
{
	/* Design error. SD0 power is connected to both GPN(11) and
	 * IO1(6) on the GPIO expander. Bad.
	 * Set IO1(6) as input to avoid the mess... */
	if (gpio_request(CORDOBA_GPIO_EXP_IO1(6), "SD0 PWR EN bug"))
		pr_err("Can't request SD0 PWR EN bug\n");
	else
		gpio_direction_input(CORDOBA_GPIO_EXP_IO1(6));

	/* Switch off any pull-resistors on ttySAC0 (FARO) */
	s3c_gpio_setpull(S5P64XX_GPA(0), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPA(1), S3C_GPIO_PULL_NONE);
}

static void cordoba_wakeup_irq(int irq)
{
	pr_info("Woke-up from IRQ#%d\n", irq);
}

/* Convoluted setup code:
 * Once the I2C GPIO expander is up and running,
 * - register the associated vgpio
 * - init the SDHCI driver (which depends on vgpio for SD0 power)
 * - Probably needs some other driver setup here.
 */
static int cordoba_gpio_exp_setup(struct i2c_client *client,
				  unsigned gpio, unsigned ngpio,
				  void *context)
{
	cordoba_quirk_setup();
	cordoba_vgpio[1].dev.parent = &client->dev;
	platform_device_register(&cordoba_vgpio[1]);

	return 0;
}

static struct pca953x_platform_data gpio_exp = {
        .gpio_base      = CORDOBA_GPIO_EXPANDER_BASE,
        .setup          = cordoba_gpio_exp_setup,
};

static struct i2c_board_info cordoba_i2c_devs0[] __initdata = {
        {
                I2C_BOARD_INFO("pca9539", 0x74),
                .platform_data  = &gpio_exp,
                //.irq          = gpio_to_irq(S5P64XX_GPP(10)),
        },
        { I2C_BOARD_INFO("24c01", 0x50), },
};

static struct i2c_board_info cordoba_i2c_devs1[] __initdata = {
};

static struct platform_device *cordoba_devices[] /* __initdata */ = {
#if defined(CONFIG_TOMTOM_FDT)
	&tomtom_device_libfdt,
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_fb,
	&s3c_device_usbgadget,
	&s3c_device_usb_otghcd,
	&s3c_device_gvg,	
	&s3c_device_adc,
	&s3c_device_ts,

#ifdef CONFIG_SND_S5P64XX_SOC_I2S
	&s5p64xx_device_iisv4,
#endif

#ifdef CONFIG_S3C_PWM
	&s3c_device_timer[0],
	&s3c_device_timer[1],
#endif

#ifdef CONFIG_TOMTOM_BATTERY
	&mendoza_device_battery,
#endif
#ifdef CONFIG_TOMTOM_VBUSMON
	&mendoza_device_vbus,
#endif
#if defined(CONFIG_TOMTOM_TT_SETUP)
	&tomtom_device_ttsetup,
#endif
};

/* ADC */
struct s3c_adc_mach_info s3c_adc_platform={
	/* s5p6440 support 12-bit resolution */
	.delay	= 10000,
	.presc 	= 49,
	.resolution = 12,
};

/* TS */
struct s3c_ts_mach_info s3c_ts_platform = {
	.delay = 10000,
	.presc = 49,
	.resol_bit = 12,
};

#define OTGH_PHY_CLK_VALUE      (0x02)  /* UTMI Interface, Cristal, 12Mhz clk for PLL */

/* Initializes OTG Phy. */
static void cordoba_otg_phy_init(void)
{
	int err;

	if ((err = gpio_request(S5P64XX_GPB(2), "USB_3V3_PWR_EN"))) {
		pr_err("Failed to request USB_3V3_PWR_EN\n");
		return;
	}
	gpio_direction_output(S5P64XX_GPB(2), 0);

	if ((err = gpio_request(S5P64XX_GPB(3), "USB_1V2_PWR_EN"))) {
		pr_err("Failed to request USB_1V2_PWR_EN\n");
		return;
	}
	gpio_direction_output(S5P64XX_GPB(3), 0);
	mdelay(100);
	gpio_set_value(S5P64XX_GPB(3), 1);
	mdelay(50);
	gpio_set_value(S5P64XX_GPB(2), 1);
	mdelay(50);

	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
	writel(0x0, S3C_USBOTG_PHYPWR);		/* Power up */
        writel(OTGH_PHY_CLK_VALUE, S3C_USBOTG_PHYCLK);
	writel(0x1, S3C_USBOTG_RSTCON);

	udelay(50);
	writel(0x0, S3C_USBOTG_RSTCON);
	udelay(50);
}

/* OTG PHY Power Off */
static void cordoba_otg_phy_off(void) {
	writel(readl(S3C_USBOTG_PHYPWR)|(0x1F<<1), S3C_USBOTG_PHYPWR);
	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);

	gpio_free(S5P64XX_GPB(2));
	gpio_free(S5P64XX_GPB(3));
}

static struct s3c_otg_platform_data cordoba_otg = {
	.otg_phy_init	= cordoba_otg_phy_init,
	.otg_phy_off	= cordoba_otg_phy_off,
};

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(8)));
EXPORT_SYMBOL(usb_ctrl);

static void __init cordoba_setup_clocks(void)
{
	/* Setup CLK_OUT as the RTC clock, to be supplied to
	 * the Barracuda chip */
	s3c_gpio_cfgpin(S5P64XX_GPF(14), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S5P64XX_GPF(14), S3C_GPIO_PULL_NONE);
	writel(5 << 12 | /* Select RTC as source */
	       2 << 4  | /* Set divider to 2 */
	       1 << 0,   /* Output enable */
	       S3C_CLK_OUT);
}

static void __init cordoba_map_io(void)
{
	s5p64xx_init_io(cordoba_iodesc, ARRAY_SIZE(cordoba_iodesc));
	s3c24xx_init_clocks(XTAL_FREQ);
	s3c24xx_init_uarts(cordoba_uartcfgs, ARRAY_SIZE(cordoba_uartcfgs));

	s3c64xx_reserve_bootmem();
}

static int cordoba_vgpio1_setup(struct platform_device *dev, void *context)
{
	/* At this point, I2C MUST BE UP AND RUNNING!!!!!
	 * If not, then vgpio1 is not available, and that will
	 * screw you over. You've been warned
	 */

	pm_cpu_wakeup_irq = cordoba_wakeup_irq;

	s3c_adc_set_platdata(&s3c_adc_platform);
	s3c_ts_set_platdata(&s3c_ts_platform);

	mendoza_s3cfb_setup("lms480wv");
	mendoza_sdhci_setup();
	cordoba_spi_setup();

	s3c_device_usbgadget.dev.platform_data = &cordoba_otg;
	s3c_device_usb_otghcd.dev.platform_data = &cordoba_otg;

	platform_add_devices(cordoba_devices, ARRAY_SIZE(cordoba_devices));

	tomtom_bl_setup();

	s5p6440_pm_init();

	cordoba_setup_clocks();

	gprs_init();

	return 0;
}

static void __init cordoba_machine_init(void)
{
	/* First register the first vgpio, as it is used to power-on
	 * the i2c bus, and in turn the second vgpio */
	platform_device_register(&cordoba_vgpio[0]);
	mendoza_s5m8751_pmic_setup(	S5M8751_LDO1_EN |
					S5M8751_LDO2_EN |
					S5M8751_LDO3_EN |
					S5M8751_LDO4_EN |
					S5M8751_LDO5_EN |
					S5M8751_LDO6_EN |
					S5M8751_BCK1_EN);

	mendoza_s5m8751_i2c_init(0);
	i2c_register_board_info(0, cordoba_i2c_devs0, ARRAY_SIZE(cordoba_i2c_devs0));
	i2c_register_board_info(1, cordoba_i2c_devs1, ARRAY_SIZE(cordoba_i2c_devs1));
	mendoza_i2c_setup(3);
}

MACHINE_START(CORDOBA, "Cordoba")
	/* Maintainer: TomTom B.V. */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P64XX_PA_SDRAM + 0x100,

	.init_irq	= s5p6440_init_irq,
	.map_io		= cordoba_map_io,
	.init_machine	= cordoba_machine_init,
	.timer		= &s3c64xx_timer,
MACHINE_END

#if defined(CONFIG_RTC_DRV_S3C)
/* RTC common Function for samsung APs*/
unsigned int s3c_rtc_set_bit_byte(void __iomem *base, uint offset, uint val)
{
	writeb(val, base + offset);

	return 0;
}

unsigned int s3c_rtc_read_alarm_status(void __iomem *base)
{
	return 1;
}

void s3c_rtc_set_pie(void __iomem *base, uint to)
{
	unsigned int tmp;

	tmp = readw(base + S3C2410_RTCCON) & ~S3C_RTCCON_TICEN;

        if (to)
                tmp |= S3C_RTCCON_TICEN;

        writew(tmp, base + S3C2410_RTCCON);
}

void s3c_rtc_set_freq_regs(void __iomem *base, uint freq, uint s3c_freq)
{
	unsigned int tmp;

        tmp = readw(base + S3C2410_RTCCON) & (S3C_RTCCON_TICEN | S3C2410_RTCCON_RTCEN );
        writew(tmp, base + S3C2410_RTCCON);
        s3c_freq = freq;
        tmp = (32768 / freq)-1;
        writel(tmp, base + S3C2410_TICNT);
}

void s3c_rtc_enable_set(struct platform_device *pdev,void __iomem *base, int en)
{
	unsigned int tmp;

	if (!en) {
		tmp = readw(base + S3C2410_RTCCON);
		writew(tmp & ~ (S3C2410_RTCCON_RTCEN | S3C_RTCCON_TICEN), base + S3C2410_RTCCON);
	} else {
		/* re-enable the device, and check it is ok */
		if ((readw(base+S3C2410_RTCCON) & S3C2410_RTCCON_RTCEN) == 0){
			dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp|S3C2410_RTCCON_RTCEN, base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)){
			dev_info(&pdev->dev, "removing RTCCON_CNTSEL\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp& ~S3C2410_RTCCON_CNTSEL, base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)){
			dev_info(&pdev->dev, "removing RTCCON_CLKRST\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp & ~S3C2410_RTCCON_CLKRST, base+S3C2410_RTCCON);
		}
	}
}
#endif

