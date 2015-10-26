/* linux/arch/arm/plat-tomtom/mendoza/mendoza-sdhci.c
 *
 * Copyright 2009 TomTom B.V.
 *	Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/vgpio.h>
#include <linux/mmc/host.h>

#include <plat/devs.h>
#include <plat/sdhci.h>
#include <plat/regs-sdhci.h>
#include <plat/gpio-cfg.h>

#include <mach/gpio.h>
#include <plat/mendoza.h>

/* SDHCI stuff. SMDK uses a scheme with some hardcoded names, so we basically
 * don't use that. Instead, we define our complete platform here and register
 * with the s3c-sdhci driver. The code is mostly similar to the SMDK stuff, though. */

// Channel we want to wire the card-detect to.
#define MENDOZA_CD_CHANNEL	0

/* clock sources for the mmc bus clock, order as for the ctrl2[5..4] */


static char *mendoza_hsmmc_clksrcs[4] = {
	[0] = "hsmmc",
	[1] = "hsmmc",
	[2] = "mmc_bus",
	/* [3] = "48m", - note not succesfully used yet */
};

static void mendoza_setup_card_detect(int channel)
{
	int gpio;

	/* 64xx only has one card detect, so check if channel
	 * matches MENDOZA_CD_CHANNEL, and do the right thing (tm).
	 * Don't touch anything if channel is #2...
	 */

	gpio = vgpio_to_gpio(TT_VGPIO_SD_CD);

	if (channel == MENDOZA_CD_CHANNEL && channel != 2) {
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN((2 + channel)));
	}
}

static void mendoza_setup_sdhci0_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;

	/* Power-on socket 0, inverted... */
	gpio_set_value(TT_VGPIO_SD0_PWR_EN, !width);

	/* GPIO should be set on 4bit though 1-bit setting is comming. */
	if (width == 1)
		width = 4;
	gpio = vgpio_to_gpio(TT_VGPIO_SD_BASE);
	end = gpio + width + 2;

	/* Set all the necessary GPG pins to special-function 2 */
	for (; gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	mendoza_setup_card_detect(0);
}

static void mendoza_setup_sdhci_cfg_card(struct platform_device *dev,
					 void __iomem *r,
			 		 struct mmc_ios *ios,
					 struct mmc_card *card)
{
	u32 ctrl2, ctrl3;

	/* don't need to alter anything acording to card-type */

	writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);

	ctrl2 = readl(r + S3C_SDHCI_CONTROL2);
	ctrl2 &= S3C_SDHCI_CTRL2_SELBASECLK_MASK;
	ctrl2 |= (S3C64XX_SDHCI_CTRL2_ENSTAASYNCCLR |
		  S3C64XX_SDHCI_CTRL2_ENCMDCNFMSK |
		  S3C_SDHCI_CTRL2_ENFBCLKRX |
		  S3C_SDHCI_CTRL2_DFCNT_NONE |
		  S3C_SDHCI_CTRL2_ENCLKOUTHOLD);

	if (ios->clock < 25 * 1000000)
		ctrl3 = (S3C_SDHCI_CTRL3_FCSEL3 |
			 S3C_SDHCI_CTRL3_FCSEL2 |
			 S3C_SDHCI_CTRL3_FCSEL1 |
			 S3C_SDHCI_CTRL3_FCSEL0);
	else
		ctrl3 = (S3C_SDHCI_CTRL3_FCSEL1 | S3C_SDHCI_CTRL3_FCSEL0);

	writel(ctrl2, r + S3C_SDHCI_CONTROL2);
	writel(ctrl3, r + S3C_SDHCI_CONTROL3);
}

static void mendoza_setup_sdhci1_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;

	/* GPIO should be set on 4bit though 1-bit setting is comming. */
	if (width == 1)
		width = 4;
	gpio = vgpio_to_gpio(TT_VGPIO_SD_BOOT_BASE);
	end = gpio + width + 2;

	/* Set all the necessary GPH pins to special-function 2 */
	for (; gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	mendoza_setup_card_detect(1);
}

/* Fake card-detect interrupt for channel 1 (and maybe 2) */
static void mendoza_sdhci1_cfg_ext_cd(void)
{
}

static unsigned int mendoza_sdhci1_detect_ext_cd(void)
{
        return 1;
}

static struct s3c_sdhci_platdata mendoza_hsmmc0_platdata __initdata = {
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED |
			   MMC_CAP_SD_HIGHSPEED | MMC_CAP_BOOT_ONTHEFLY),
	.clocks		= mendoza_hsmmc_clksrcs,
	.cfg_gpio	= mendoza_setup_sdhci0_cfg_gpio,
	.cfg_card	= mendoza_setup_sdhci_cfg_card,
};

static struct s3c_sdhci_platdata mendoza_hsmmc1_platdata __initdata = {
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED |
			   MMC_CAP_SD_HIGHSPEED | MMC_CAP_BOOT_ONTHEFLY),
	.clocks		= mendoza_hsmmc_clksrcs,
	.cfg_gpio	= mendoza_setup_sdhci1_cfg_gpio,
	.cfg_card	= mendoza_setup_sdhci_cfg_card,
#if MENDOZA_CD_CHANNEL != 1
	.cfg_ext_cd	= mendoza_sdhci1_cfg_ext_cd,
	.detect_ext_cd	= mendoza_sdhci1_detect_ext_cd,
#endif
};

static struct platform_device *mendoza_sdhci_devices[2][2] __initdata = {
	[0] = {
		&s3c_device_hsmmc0,
		&s3c_device_hsmmc1,
	},
	[1] = {
		&s3c_device_hsmmc1,
		&s3c_device_hsmmc0,
	},
};

void __init mendoza_sdhci_setup(void)
{
	int err;
	int boot_slot = 1;

	/* Configure SD/MMC CH0 power enable, default off (signal is active low) */
	if ((err = gpio_request(TT_VGPIO_SD0_PWR_EN, "SD0n_PWR_EN")))
		pr_err("Cant't request SD0n_PWR_EN (%d): %d\n", TT_VGPIO_SD0_PWR_EN, err);
	else {
		s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_SD0_PWR_EN),S3C_GPIO_PULL_NONE);
		gpio_direction_output(TT_VGPIO_SD0_PWR_EN, 1);
	}

	s3c_sdhci0_set_platdata(&mendoza_hsmmc0_platdata);
	s3c_sdhci1_set_platdata(&mendoza_hsmmc1_platdata);

	if ((err = gpio_request(TT_VGPIO_BOOT_DEV0, "Boot source")))
		pr_err("Can't determine boot MMC slot, assuming 1\n");
	else {
		gpio_direction_input(TT_VGPIO_BOOT_DEV0);
		boot_slot = gpio_get_value(TT_VGPIO_BOOT_DEV0);
		pr_info("Boot MMC on channel %d\n", boot_slot);
		gpio_free(TT_VGPIO_BOOT_DEV0);
	}

	/* We only ask for two channels at the moment */
	platform_add_devices(mendoza_sdhci_devices[boot_slot], 2);
}

