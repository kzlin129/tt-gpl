/* linux/arch/arm/mach-s5p6440/havana.c
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
#include <linux/spi/spi.h>
#include <plat/adau1761_pdata.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
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
#include <plat/mendoza_battery.h>
#include <mach/cordoba.h>

#include <plat/fdt.h>
#include <plat/tt_setup_handler.h>

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

extern void s3c64xx_reserve_bootmem(void);

static struct s3c2410_uartcfg havana_uartcfgs[] __initdata = {
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

struct map_desc havana_iodesc[] = {};


// Some macros, just to save some typing...
#define TT_DEF_PIN0(p,t)         VGPIO_DEF_PIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_INVPIN0(p,t)      VGPIO_DEF_INVPIN(TT_VGPIO0_BASE, TT_VGPIO_##p, t)
#define TT_DEF_NCPIN0(p)         VGPIO_DEF_NCPIN(TT_VGPIO0_BASE, TT_VGPIO_##p)

// Pin tables
static struct vgpio_pin havana_vgpio0_pins[] = {
	TT_DEF_PIN0(PU_I2C0,		S5P64XX_GPN(12)),
	TT_DEF_PIN0(PU_I2C1,		S5P64XX_GPP(8)),
	TT_DEF_PIN0(WALL_ON,		S5P64XX_GPP(3)),
	TT_DEF_INVPIN0(HRESET,		S5P64XX_GPP(7)),
	TT_DEF_PIN0(PWR_KILL,		S5P64XX_GPP(5)),
	TT_DEF_PIN0(BOOT_DEV0,		S5P64XX_GPN(13)),
	TT_DEF_PIN0(SD0_PWR_EN,		S5P64XX_GPN(11)),
	TT_DEF_PIN0(MUTE,		S5P64XX_GPC(3)),
	TT_DEF_INVPIN0(LCM_PWR_EN,	S5P64XX_GPC(6)),
	TT_DEF_PIN0(LCM_CS,		S5P64XX_GPN(10)),
	TT_DEF_INVPIN0(CHARGING,	S5P64XX_GPN(0)),
	TT_DEF_PIN0(PWR_BUTTON,		S5P64XX_GPN(4)),
	TT_DEF_PIN0(DOCK_RESET, 	S5P64XX_GPR(3)),
	TT_DEF_PIN0(DOCK_DET0,		S5P64XX_GPN(3)),
	TT_DEF_PIN0(DOCK_DET1,		S5P64XX_GPN(2)),
	TT_DEF_PIN0(DOCK_I2C_EN,	S5P64XX_GPN(7)),
	TT_DEF_PIN0(DOCK_FM_INT,	S5P64XX_GPC(2)),
	TT_DEF_PIN0(RFS_BOOT_CLK,	S5P64XX_GPB(4)),
	TT_DEF_PIN0(RFS_BOOT_Q,		S5P64XX_GPR(11)),
	TT_DEF_PIN0(USB_HOST_DETECT,	S5P64XX_GPN(6)),
	TT_DEF_PIN0(GSM_SYS_EN,		S5P64XX_GPP(10)),
	TT_DEF_PIN0(GSM_SYS_RST,	S5P64XX_GPC(7)),
	TT_DEF_PIN0(AMP_PWR_EN,		S5P64XX_GPR(0)),
	TT_DEF_PIN0(ACCESSORY_PWR_EN,	S5P64XX_GPR(1)),
	TT_DEF_INVPIN0(GPS_RESET, 	S5P64XX_GPR(2)),
	TT_DEF_INVPIN0(GPS_1PPS, 	S5P64XX_GPN(9)),
	TT_DEF_INVPIN0(BT_RST,		S5P64XX_GPC(5)),
	TT_DEF_INVPIN0(SD_CD,		S5P64XX_GPG(6)),
	TT_DEF_PIN0(SD_BASE,		S5P64XX_GPG(0)),
	TT_DEF_PIN0(SD_BOOT_BASE,	S5P64XX_GPH(0)),
	TT_DEF_INVPIN0(MIC_RESET,	S5P64XX_GPF(15)),
	TT_DEF_NCPIN0(GPS_STANDBY),
	TT_DEF_NCPIN0(MIC_STBY),
	TT_DEF_NCPIN0(DOCK_DET_PWR),
};


// Platform data & device
static struct vgpio_platform_data havana_vgpio_pdata[] = {
	[0] = {
        	.gpio_base      = TT_VGPIO0_BASE,
        	.gpio_number    = ARRAY_SIZE(havana_vgpio0_pins),
        	.pins           = havana_vgpio0_pins,
	},
};

static struct platform_device havana_vgpio[] = {
	[0] = {
        	.name           = "vgpio",
        	.id             = 0,
        	.dev            = {
                	.platform_data  = &havana_vgpio_pdata[0],
        	},
	},
};

/* First setup whatever oddity the HW requires... */
static void havana_quirk_setup(void)
{
	/* Switch off any pull-resistors on ttySAC0 (FARO) */
	s3c_gpio_setpull(S5P64XX_GPA(0), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPA(1), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPA(4), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPA(5), S3C_GPIO_PULL_NONE);
	/* Configure uart3 for Bluetooth */	
	s3c_gpio_cfgpin(S5P64XX_GPB(2), S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(S5P64XX_GPB(3), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S5P64XX_GPB(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPB(3), S3C_GPIO_PULL_NONE);

	/* I2S V40 drive strength 00 = 2 mA, 01 = 4mA, 10 = 8 mA, 11 = 12 mA */
	writel(((readl(S5P64XX_SPC1_BASE) & ~(0x03<<6)) | (0x00 << 6)), S5P64XX_SPC1_BASE);
}

static struct i2c_board_info havana_i2c_devs0[] __initdata = {
};

static struct i2c_board_info havana_i2c_devs1[] __initdata = {
	{ I2C_BOARD_INFO("24c01", 0x50), },
};

static struct platform_device *havana_devices[] /* __initdata */ = {
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
#ifdef CONFIG_TOUCHSCREEN_S3C
	&s3c_device_ts,
#endif

#ifdef CONFIG_SND_S5P64XX_SOC_I2S
	&s5p64xx_device_iisv4,
#endif

#ifdef CONFIG_SND_S3C64XX_SOC_PCM
	&s5p64xx_device_pcm,
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

#ifdef CONFIG_TOUCHSCREEN_S3C
/* TS */
struct s3c_ts_mach_info s3c_ts_platform = {
	.delay = 10000,
	.presc = 49,
	.resol_bit = 12,
};
#endif

#define OTGH_PHY_CLK_VALUE      (0x02)  /* UTMI Interface, Cristal, 12Mhz clk for PLL */

/* Initializes OTG Phy. */
static void havana_otg_phy_init(void)
{
	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
	writel(0x0, S3C_USBOTG_PHYPWR);		/* Power up */
        writel(OTGH_PHY_CLK_VALUE, S3C_USBOTG_PHYCLK);
	writel(0x1, S3C_USBOTG_RSTCON);
	udelay(50);
	writel(0x0, S3C_USBOTG_RSTCON);
	udelay(50);
}

/* OTG PHY Power Off */
static void havana_otg_phy_off(void)
{
	writel(0x4, S3C_USBOTG_RSTCON);
	udelay(50);
	writel(0x0, S3C_USBOTG_RSTCON);
	udelay(50);
	writel(0x19, S3C_USBOTG_PHYPWR);
	writel(readl(S3C_OTHERS)|S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
}

static struct s3c_otg_platform_data havana_otg = {
	.otg_phy_init	= havana_otg_phy_init,
	.otg_phy_off	= havana_otg_phy_off,
};

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(8)));
EXPORT_SYMBOL(usb_ctrl);

static void __init havana_setup_clocks(void)
{
	/* Setup CLK_OUT as the RTC clock, to be supplied to
	 * the GPS chip */
	s3c_gpio_cfgpin(S5P64XX_GPF(14), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S5P64XX_GPF(14), S3C_GPIO_PULL_NONE);
	writel(5 << 12 | /* Select RTC as source */
	       2 << 4  | /* Set divider to 2 */
	       1 << 0,   /* Output enable */
	       S3C_CLK_OUT);
}

static void havana_wakeup_irq(int irq)
{
	pr_info("Woke-up from IRQ#%d\n", irq);
}

static void __init havana_map_io(void)
{
	s5p64xx_init_io(havana_iodesc, ARRAY_SIZE(havana_iodesc));
	s3c24xx_init_clocks(XTAL_FREQ);
	s3c24xx_init_uarts(havana_uartcfgs, ARRAY_SIZE(havana_uartcfgs));

	s3c64xx_reserve_bootmem();
}

static void __init havana_machine_init(void)
{
	platform_device_register(&havana_vgpio[0]);
	havana_quirk_setup();

	pm_cpu_wakeup_irq = havana_wakeup_irq;

	i2c_register_board_info(0, havana_i2c_devs0, ARRAY_SIZE(havana_i2c_devs0));
	i2c_register_board_info(1, havana_i2c_devs1, ARRAY_SIZE(havana_i2c_devs1));
	mendoza_s5m8751_i2c_init(1);
	mendoza_i2c_setup(3);

	s3c_adc_set_platdata(&s3c_adc_platform);
#ifdef CONFIG_TOMTOM_BATTERY
	mendoza_battery_adc_setup(AIN2, AIN4);
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C
	s3c_ts_set_platdata(&s3c_ts_platform);
#endif

	mendoza_s3cfb_setup("lms430wqv");
	mendoza_sdhci_setup();

	s3c_device_usbgadget.dev.platform_data = &havana_otg;
	s3c_device_usb_otghcd.dev.platform_data = &havana_otg;

	platform_add_devices(havana_devices, ARRAY_SIZE(havana_devices));

	s5p6440_pm_init();

	havana_setup_clocks();
#ifdef CONFIG_TOMTOM_GPRS
	gprs_init();
#endif
}

MACHINE_START(HAVANA, "havana")
	/* Maintainer: TomTom B.V. */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P64XX_PA_SDRAM + 0x100,

	.init_irq	= s5p6440_init_irq,
	.map_io		= havana_map_io,
	.init_machine	= havana_machine_init,
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

