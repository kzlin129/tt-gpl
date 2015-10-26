#ifndef _PLAT_MENDOZA_H
#define _PLAT_MENDOZA_H

#include <linux/platform_device.h>

/* Define the virtual gpio base after the last of the physical gpios, */
/* for the transition to work out smoothly.                           */
/* Out of some anthropic arbitrary principle, I rounded up to 100.    */
#define BCM4760_VGPIO_BASE	96

enum bcm476x_vgpio_pins {
	TT_VGPIO_BASE	= BCM4760_VGPIO_BASE,

	/* Backlight */
	TT_VGPIO_LCD_BACKLIGHT_PWM = TT_VGPIO_BASE,	/* Catalina GPIO[13] */
	TT_VGPIO_LCD_BACKLIGHT_ENABLE,			/* Catalina GPIO[12] */
	TT_VGPIO_LCD_ON,				/* Catalina GPIO[14] */

	/* LCD */
	TT_VGPIO_LCD_ID,
        TT_VGPIO_LCD_CS,
        TT_VGPIO_LCD_SCLK,
        TT_VGPIO_LCD_SDI,
        TT_VGPIO_LCD_SDO,
        
	/* PMU */
	TT_VGPIO_PMU_IRQ,

	/* Dock */
	TT_VGPIO_DOCK_RESET,
	TT_VGPIO_ACCESSORY_PWR_EN,
	TT_VGPIO_DOCK_DET_PWR,
	TT_VGPIO_DOCK_DET0,
	TT_VGPIO_DOCK_DET1,
	TT_VGPIO_DOCK_I2C_PWR,
	TT_VGPIO_DOCK_FM_INT,
	TT_VGPIO_DOCK_HPDETECT,
	TT_VGPIO_DOCK_MUTE,

	/* Bluetooth */
	TT_VGPIO_BT_EN,
	TT_VGPIO_BT_RST,
	TT_VGPIO_WL_RST,
	TT_VGPIO_REG_ON,
	TT_VGPIO_BT_UART_RTS,
	TT_VGPIO_BT_UART_CTS,

	/* USB */
	TT_VGPIO_USB_HOST_DETECT,

	/* Suicide */
	TT_VGPIO_KILL_PWRn,

	/* Power Button */
	TT_VGPIO_PB,

	/* GPRS */
	TT_VGPIO_GSM_POWER,
	TT_VGPIO_GSM_RESET,

	/* MMC */
	TT_VGPIO_MMC0_CD,
	TT_VGPIO_MMC1_CD,
	TT_VGPIO_MMC2_CD,
	TT_VGPIO_SD_PWR_EN,

	/* Audio */
	TT_VGPIO_AMP_PWR_EN,

	/* I2C EEPROM */
	TT_VGPIO_EEPROM_WP,

	/* Touch Screen Panel */
	TT_VGPIO_TSP_ATTN,
	TT_VGPIO_TSP_CE,
	TT_VGPIO_TSP_PWR,

	/* Low dc vcc detector */
	TT_VGPIO_LOW_DC_VCC,

	/* Tilt Sensor */
	TT_VGPIO_TILT_PWR,
	TT_VGPIO_TILT_OUT,
	TT_VGPIO_TILT_OUT2,

	/* PPS */
	TT_VGPIO_PPS
};

extern struct platform_device bcm476x_device_ts;
void irvine_init_machine (void);

#endif
