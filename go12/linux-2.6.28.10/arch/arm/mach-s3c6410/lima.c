/* linux/arch/arm/mach-s3c6410/lima.c
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
#include <plat/mendoza_ltc3577.h>

#include <plat/fdt.h>
#include <plat/tt_setup_handler.h>

#define OTGH_PHY_CLK_VALUE      (0x10)

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

extern void s3c64xx_reserve_bootmem(void);

static struct s3c2410_uartcfg lima_uartcfgs[] __initdata = {
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

struct map_desc lima_iodesc[] = {};

// Some macros, just to save some typing...
#define TT_DEF_PIN0(p,t)         VGPIO_DEF_PIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_INVPIN0(p,t)      VGPIO_DEF_INVPIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_NCPIN0(p)         VGPIO_DEF_NCPIN(TT_VGPIO0_BASE, TT_VGPIO_##p)

// Pin tables
static struct vgpio_pin lima_vgpio0_pins[] = {
	TT_DEF_PIN0(PU_I2C0,		S3C64XX_GPK(8)),
	TT_DEF_PIN0(PU_I2C1,		S3C64XX_GPP(8)),
	TT_DEF_PIN0(BOOT_DEV0,		S3C64XX_GPN(13)),
	TT_DEF_PIN0(SD0_PWR_EN,		S3C64XX_GPN(11)),
	TT_DEF_PIN0(BARRACUDA_CS,	S3C64XX_GPC(3)),
	TT_DEF_PIN0(BACKLIGHT_EN,	S3C64XX_GPF(13)),
	TT_DEF_PIN0(LCM_RESET,		S3C64XX_GPL(3)),
	TT_DEF_PIN0(PWR_BUTTON,		S3C64XX_GPN(4)),
	TT_DEF_PIN0(RFS_BOOT_CLK,	S3C64XX_GPB(4)),
	TT_DEF_PIN0(RFS_BOOT_Q,		S3C64XX_GPM(1)),
	TT_DEF_PIN0(USB_HOST_DETECT,	S3C64XX_GPN(6)),
	TT_DEF_INVPIN0(GPS_STANDBY,	S3C64XX_GPK(0)),
	TT_DEF_INVPIN0(GPS_RESET, 	S3C64XX_GPK(3)),
	TT_DEF_INVPIN0(BT_RST,		S3C64XX_GPC(7)),
	TT_DEF_PIN0(WALL_ON,		S3C64XX_GPK(7)),
	TT_DEF_INVPIN0(SD_CD,		S3C64XX_GPG(6)),
	TT_DEF_PIN0(SD_BASE,		S3C64XX_GPG(0)),
	TT_DEF_PIN0(SD_BOOT_BASE,	S3C64XX_GPH(0)),
	TT_DEF_PIN0(GSM_SYS_EN,		S3C64XX_GPL(0)),
	TT_DEF_PIN0(LCM_CS,		S3C64XX_GPM(3)),
	TT_DEF_INVPIN0(LCM_PWR_EN,		S3C64XX_GPL(2)),
	TT_DEF_PIN0(CODEC_PWR_EN,	S3C64XX_GPK(10)),
	TT_DEF_PIN0(AMP_PWR_EN,		S3C64XX_GPK(9)),
	TT_DEF_NCPIN0(MIC_STBY),
	TT_DEF_INVPIN0(DOCK_RESET,	S3C64XX_GPN(12)),
	TT_DEF_PIN0(DOCK_DET0,		S3C64XX_GPN(3)),
	TT_DEF_PIN0(DOCK_DET1,		S3C64XX_GPN(2)),
	TT_DEF_PIN0(ACCESSORY_PWR_EN,	S3C64XX_GPN(9)),
	TT_DEF_NCPIN0(DOCK_DET_PWR),
	TT_DEF_INVPIN0(HPDETECT,	S3C64XX_GPN(5)),
};

// Platform data & device
static struct vgpio_platform_data lima_vgpio_pdata[] = {
	[0] = {
        	.gpio_base      = TT_VGPIO0_BASE,
        	.gpio_number    = ARRAY_SIZE(lima_vgpio0_pins),
        	.pins           = lima_vgpio0_pins,
	},
};

static struct platform_device lima_vgpio[] = {
	[0] = {
        	.name           = "vgpio",
        	.id             = 0,
        	.dev            = {
                	.platform_data  = &lima_vgpio_pdata[0],
        	},
	},
};

/* Initializes OTG Phy. */
static void lima_otg_phy_init(void)
{
	int err;

	if ((err = gpio_request(S3C64XX_GPL(4), "USB_3V3_PWR_EN"))) {
		pr_err("Failed to request USB_3V3_PWR_EN\n");
		return;
	}

	if ((err = gpio_request(S3C64XX_GPL(5), "USB_1V2_PWR_EN"))) {
		pr_err("Failed to request USB_1V2_PWR_EN\n");
		return;
	}

	gpio_direction_output(S3C64XX_GPL(4), 0);
	gpio_direction_output(S3C64XX_GPL(5), 0);
	mdelay(100);
	gpio_set_value(S3C64XX_GPL(5), 1);
	mdelay(50);
	gpio_set_value(S3C64XX_GPL(4), 1);
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
static void lima_otg_phy_off(void) {
	writel(readl(S3C_USBOTG_PHYPWR)|(0x1F<<1), S3C_USBOTG_PHYPWR);
	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);

	gpio_free(S3C64XX_GPL(4));
	gpio_free(S3C64XX_GPL(5));
}

static struct s3c_otg_platform_data lima_otg = {
	.otg_phy_init   = lima_otg_phy_init,
	.otg_phy_off    = lima_otg_phy_off,
};

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(8)));
EXPORT_SYMBOL(usb_ctrl);

/* First setup whatever oddity the HW requires... */
static void lima_quirk_setup(void)
{
	u32 cfg;

	/* Lower drive strenght for video down to 4mA */
	cfg = readl(S3C64XX_SPC_BASE);
	cfg &= ~(3 << 24);
	cfg |= (1 << 24);
	writel(cfg, S3C64XX_SPC_BASE);
}

static struct i2c_board_info lima_i2c_devs0[] __initdata = {
};

static struct i2c_board_info lima_i2c_devs1[] __initdata = {
	{ I2C_BOARD_INFO("24c01", 0x50), },
};

static struct platform_device *lima_devices[] /* __initdata */ = {
#if defined(CONFIG_TOMTOM_FDT)
	&tomtom_device_libfdt,
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	//&s3c_device_lcd,
	&s3c_device_fb,
	&s3c_device_usbgadget,
	&s3c_device_usb_otghcd,
	&s3c_device_adc,
	&s3c_device_ts,
	&s3c_device_timer[1],
	&s3c_device_g3d,

#ifdef CONFIG_SND_S3C64XX_SOC_I2S
	&s3c64xx_device_iis0,
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
	/* s3c6410 support 12-bit resolution */
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

static void __init lima_setup_clocks(void)
{
	/* Setup CLK_OUT as the RTC clock, to be supplied to
	 * the Barracuda chip */
	s3c_gpio_cfgpin(S3C64XX_GPF(14), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S3C64XX_GPF(14), S3C_GPIO_PULL_NONE);
	writel(5 << 12 | /* Select RTC as source */
	       2 << 4  | /* Set divider to 2 */
	       1 << 0,   /* Output enable */
	       S3C_CLK_OUT);
}

/* QoS settings, same as SMDK... */
static void lima_set_qos(void)
{
	u32 reg;

	/* AXI sfr */
	reg = (u32) ioremap((unsigned long) 0x7e003000, SZ_4K);

	/* QoS override: FIMD min. latency */
	writel(0x2, S3C_VA_SYS + 0x128);

	/* AXI QoS */
	writel(0x7, reg + 0x460);       /* (8 - MFC ch.) */
	writel(0x7ff7, reg + 0x464);

	/* Bus cacheable */
	writel(0x8ff, S3C_VA_SYS + 0x838);
}


static void __init lima_map_io(void)
{
	s3c64xx_init_io(lima_iodesc, ARRAY_SIZE(lima_iodesc));
	s3c24xx_init_clocks(XTAL_FREQ);
	s3c24xx_init_uarts(lima_uartcfgs, ARRAY_SIZE(lima_uartcfgs));

	s3c64xx_reserve_bootmem();
}

void lima_spi_setup(void);

static void __init lima_machine_init(void)
{
	lima_quirk_setup();
	platform_device_register(&lima_vgpio[0]);

	s3c_ts_set_platdata(&s3c_ts_platform);
	s3c_adc_set_platdata(&s3c_adc_platform);

	mendoza_s3cfb_setup("lms480wv");
	mendoza_sdhci_setup();
	lima_spi_setup();

	s3c_device_usbgadget.dev.platform_data = &lima_otg;
	s3c_device_usb_otghcd.dev.platform_data = &lima_otg;

	mendoza_ltc3577_i2c_init();
	i2c_register_board_info(0, lima_i2c_devs0, ARRAY_SIZE(lima_i2c_devs0));
	i2c_register_board_info(1, lima_i2c_devs1, ARRAY_SIZE(lima_i2c_devs1));
	mendoza_i2c_setup(3);
	platform_add_devices(lima_devices, ARRAY_SIZE(lima_devices));
	tomtom_bl_setup();

	s3c6410_pm_init();

	lima_setup_clocks();
	lima_set_qos();
}

MACHINE_START(LIMA, "Lima")
	/* Maintainer: TomTom B.V. */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= lima_map_io,
	.init_machine	= lima_machine_init,
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

