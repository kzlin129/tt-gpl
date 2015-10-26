/* linux/arch/arm/mach-s3c6410/beaumont_11.c
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
#include <linux/vgpio.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-mem.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>

#include <plat/s3c6410.h>
#include <plat/clock.h>
#include <plat/regs-clock.h>
#include <plat/regs-iis.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/fb.h>
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
#include <plat/mendoza_battery.h>

#include <plat/fdt.h>
#include <plat/tt_setup_handler.h>

#define OTGH_PHY_CLK_VALUE      (0x10)

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

extern void s3c64xx_reserve_bootmem(void);

static struct s3c2410_uartcfg beaumont_11_uartcfgs[] __initdata = {
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

struct map_desc beaumont_11_iodesc[] = {};

// Some macros, just to save some typing...
#define TT_DEF_PIN0(p,t)         VGPIO_DEF_PIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_INVPIN0(p,t)      VGPIO_DEF_INVPIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_NCPIN0(p)         VGPIO_DEF_NCPIN(TT_VGPIO0_BASE, TT_VGPIO_##p)

// Pin tables
static struct vgpio_pin beaumont_11_vgpio0_pins[] = {
	TT_DEF_PIN0(BACKLIGHT_EN,		S3C64XX_GPL(4)),
	TT_DEF_PIN0(CODEC_PWR_EN,		S3C64XX_GPD(1)),
	TT_DEF_PIN0(AMP_PWR_EN,			S3C64XX_GPQ(2)),
	TT_DEF_PIN0(GPS_1PPS,			S3C64XX_GPN(4)),
	TT_DEF_INVPIN0(GPS_RESET,		S3C64XX_GPP(11)),
	TT_DEF_INVPIN0(GPS_STANDBY,		S3C64XX_GPP(10)),
//	TT_DEF_INVPIN0(GPS_CS,			S3C64XX_GPC(7)),
	TT_DEF_PIN0(PWR_BUTTON,			S3C64XX_GPN(12)),
	TT_DEF_PIN0(PWR_KILL,			S3C64XX_GPO(13)),
	TT_DEF_PIN0(GSM_SYS_EN,			S3C64XX_GPF(1)),
	TT_DEF_PIN0(GSM_SYS_RST,		S3C64XX_GPF(5)),
	TT_DEF_PIN0(LCM_CS,			S3C64XX_GPL(3)),
	TT_DEF_PIN0(LCM_PWR_EN,			S3C64XX_GPL(1)),
	TT_DEF_INVPIN0(BT_RST,			S3C64XX_GPP(7)),
	TT_DEF_PIN0(PU_I2C0,			S3C64XX_GPF(3)),
	TT_DEF_PIN0(USB_3V3_ON,			S3C64XX_GPK(1)),
	TT_DEF_PIN0(USB_1V2_ON,			S3C64XX_GPP(3)),
	TT_DEF_PIN0(USB_HOST_DETECT,		S3C64XX_GPN(8)),
//	TT_DEF_PIN0(RFS_BOOT_Q,			S3C64XX_GPB(3)),
//	TT_DEF_PIN0(RFS_BOOT_CLK,		S3C64XX_GPB(4)),
	TT_DEF_INVPIN0(DOCK_RESET,		S3C64XX_GPL(10)),
	TT_DEF_PIN0(DOCK_DET0,			S3C64XX_GPL(14)),
	TT_DEF_PIN0(DOCK_DET1,			S3C64XX_GPL(9)),
	TT_DEF_PIN0(ACCESSORY_PWR_EN,		S3C64XX_GPP(2)),
	TT_DEF_PIN0(DOCK_DET_PWR,		S3C64XX_GPF(8)),
	TT_DEF_PIN0(WALL_ON,			S3C64XX_GPO(15)),
	TT_DEF_PIN0(SD_CD,			S3C64XX_GPG(6)),
	TT_DEF_PIN0(PU_I2C1,			S3C64XX_GPK(7)),
	TT_DEF_INVPIN0(DOCK_I2C_EN,		S3C64XX_GPK(8)),
	TT_DEF_PIN0(HPDETECT,			S3C64XX_GPM(1)),
	TT_DEF_PIN0(L3_MODE,			S3C64XX_GPO(8)),
	TT_DEF_INVPIN0(DOCK_RESET,		S3C64XX_GPL(10)),
	TT_DEF_PIN0(DOCK_DET0,			S3C64XX_GPL(14)),
	TT_DEF_PIN0(DOCK_DET1,			S3C64XX_GPL(9)),
	TT_DEF_PIN0(ACCESSORY_PWR_EN,		S3C64XX_GPP(2)),
	TT_DEF_PIN0(BOOT_DEV0,			S3C64XX_GPN(15)),
	TT_DEF_INVPIN0(SD0_PWR_EN,		S3C64XX_GPP(8)),
	TT_DEF_PIN0(SD_BASE,			S3C64XX_GPG(0)),
	TT_DEF_PIN0(SD_BOOT_BASE,		S3C64XX_GPH(0)),
	TT_DEF_NCPIN0(BARRACUDA_CS),
	TT_DEF_NCPIN0(LCM_RESET),
	TT_DEF_NCPIN0(MIC_STBY),

};

// Platform data & device
static struct vgpio_platform_data beaumont_11_vgpio_pdata[] = {
	[0] = {
        	.gpio_base      = TT_VGPIO0_BASE,
        	.gpio_number    = ARRAY_SIZE(beaumont_11_vgpio0_pins),
        	.pins           = beaumont_11_vgpio0_pins,
	},
};

static struct platform_device beaumont_11_vgpio[] = {
	[0] = {
        	.name           = "vgpio",
        	.id             = 0,
        	.dev            = {
                	.platform_data  = &beaumont_11_vgpio_pdata[0],
        	},
	},
};


/* First setup whatever oddity the HW requires... */
static void beaumont_11_quirk_setup(void)
{
	/* Use I2C1 PU to keep serial console alive when undocking... */
	if (gpio_request(TT_VGPIO_PU_I2C1, "PU I2C1") ||
	    gpio_direction_output(TT_VGPIO_PU_I2C1, 1))
		pr_err("Can't configure PU_I2C1, console will die when undocking\n");

	/* Wire flow control to UART-0 for GPS */
	s3c_gpio_cfgpin(S3C64XX_GPA(2), S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(S3C64XX_GPA(3), S3C_GPIO_SFN(2));
}

static struct i2c_board_info beaumont_11_i2c_devs0[] __initdata = {
};

static struct platform_device *beaumont_11_devices[] /* __initdata */ = {
#if defined(CONFIG_TOMTOM_FDT)
	&tomtom_device_libfdt,
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_fb,
	&s3c_device_usbgadget,
	&s3c_device_usb_otghcd,
	&s3c_device_adc,
	&s3c_device_ts,
	&s3c_device_timer[0],
	&s3c_device_timer[1],

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

/* Initializes OTG Phy. */
static void beaumont_11_otg_phy_init(void)
{
	int err;

	if ((err = gpio_request(TT_VGPIO_USB_3V3_ON, "USB_3V3_ON"))) {
		pr_err("Failed to request USB_3V3_ON\n");
		return;
	}

	if ((err = gpio_request(TT_VGPIO_USB_1V2_ON, "USB_1V2_ON"))) {
		pr_err("Failed to request USB_1V2_ON\n");
		return;
	}

	gpio_direction_output(TT_VGPIO_USB_3V3_ON, 0);
	gpio_direction_output(TT_VGPIO_USB_1V2_ON, 0);
	mdelay(100);
	gpio_set_value(TT_VGPIO_USB_1V2_ON, 1);
	mdelay(50);
	gpio_set_value(TT_VGPIO_USB_3V3_ON, 1);
	mdelay(50);

	writel(readl(S3C_OTHERS) | S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
	writel(0x0, S3C_USBOTG_PHYPWR);		/* Power up */
        writel(OTGH_PHY_CLK_VALUE, S3C_USBOTG_PHYCLK);
	writel(0x1, S3C_USBOTG_RSTCON);

	udelay(50);
	writel(0x0, S3C_USBOTG_RSTCON);
	udelay(50);
}

/* OTG PHY Power Off */
static void beaumont_11_otg_phy_off(void) {
	writel(readl(S3C_USBOTG_PHYPWR)|(0x1F<<1), S3C_USBOTG_PHYPWR);
	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);

	gpio_set_value(TT_VGPIO_USB_1V2_ON, 0);
	gpio_set_value(TT_VGPIO_USB_3V3_ON, 0);

	gpio_free(TT_VGPIO_USB_3V3_ON);
	gpio_free(TT_VGPIO_USB_1V2_ON);
}

static struct s3c_otg_platform_data beaumont_11_otg = {
	.otg_phy_init   = beaumont_11_otg_phy_init,
	.otg_phy_off    = beaumont_11_otg_phy_off,
};

static void __init beaumont_11_setup_clocks(void)
{
#if 0
	void  __iomem	* regs = NULL;

	/* Setup CLK_OUT as the RTC clock, to be supplied to
	 * the Barracuda chip */
	s3c_gpio_cfgpin(S5P64XX_GPF(14), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S5P64XX_GPF(14), S3C_GPIO_PULL_NONE);
	writel(5 << 12 | /* Select RTC as source */
	       2 << 4  | /* Set divider to 2 */
	       1 << 0,   /* Output enable */
	       S3C_CLK_OUT);

	/* Output IIS CDCLK clock */
	regs = ioremap(S5P64XX_PA_IIS_V40, 0x100);
	BUG_ON(!regs);
	writel(readl(regs + S5P64XX_IISMOD) | S5P64XX_IISMOD_MSTCLKAUDIO,
		regs + S5P64XX_IISMOD);
	iounmap(regs);
	writel(readl(S3C_CLK_SRC0) & ~(S3C_CLKSRC_EPLL_CLKSEL), S3C_CLK_SRC0);
	writel((readl(S3C_CLK_SRC1) & ~(S3C_CLKSRC1_AUDIO2_MASK)) |
		(S3C_CLKSRC1_AUDIO2_SEL_MOUT), S3C_CLK_SRC1);
	writel(readl(S3C_CLK_GATE_SCLK0) | S3C_CLKCON_SCLK0_AUDIO2,
		S3C_CLK_GATE_SCLK0);
	writel(readl(S3C_CLK_GATE_PCLK) | S3C_CLKCON_PCLK_IIS2,
		S3C_CLK_GATE_PCLK);
	s3c_gpio_setpull(S5P64XX_GPR(14), S3C_GPIO_PULL_DOWN);
	s3c_gpio_cfgpin(S5P64XX_GPR(14), S3C_GPIO_SFN(5));
#endif
}

static void __init beaumont_11_map_io(void)
{
	s3c64xx_init_io(beaumont_11_iodesc, ARRAY_SIZE(beaumont_11_iodesc));
	s3c24xx_init_clocks(XTAL_FREQ);
	s3c24xx_init_uarts(beaumont_11_uartcfgs, ARRAY_SIZE(beaumont_11_uartcfgs));

	s3c64xx_reserve_bootmem();
}

static void __init beaumont_11_machine_init(void)
{
	/* First register VGPIO. Nothing should come before it... */
	platform_device_register(&beaumont_11_vgpio[0]);

	beaumont_11_quirk_setup();

	s3c_ts_set_platdata(&s3c_ts_platform);
	s3c_adc_set_platdata(&s3c_adc_platform);
	mendoza_battery_adc_setup(AIN1, AIN2);
	s3cfb_set_platdata(NULL);

	s3c_device_usbgadget.dev.platform_data = &beaumont_11_otg;
	s3c_device_usb_otghcd.dev.platform_data = &beaumont_11_otg;

	mendoza_s3cfb_setup("lms430wqv");
	mendoza_sdhci_setup();
	i2c_register_board_info(0, beaumont_11_i2c_devs0, ARRAY_SIZE(beaumont_11_i2c_devs0));
	mendoza_i2c_setup(1);
	platform_add_devices(beaumont_11_devices, ARRAY_SIZE(beaumont_11_devices));
	tomtom_bl_setup();

	s3c6410_pm_init();

	beaumont_11_setup_clocks();
}

MACHINE_START(SEOUL, "Beaumont-11")
	/* Maintainer: TomTom B.V. */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= beaumont_11_map_io,
	.init_machine	= beaumont_11_machine_init,
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

