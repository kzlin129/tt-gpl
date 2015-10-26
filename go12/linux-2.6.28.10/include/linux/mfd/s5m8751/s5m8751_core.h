/*
 * s5m8751_core.h  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef __LINUX_MFD_S5M8751_CORE_H_
#define __LINUX_MFD_S5M8751_CORE_H_

#include <linux/kernel.h>

#include <linux/mfd/s5m8751/s5m8751_pmic.h>
#include <linux/mfd/s5m8751/s5m8751_audio.h>
#include <linux/mfd/s5m8751/s5m8751_backlight.h>

/*
 * Register values.
 */
#define S5M8751_IRQB_EVENT1			0x00
#define	S5M8751_IRQB_EVENT2			0x01
#define S5M8751_IRQB_MASK1			0x02
#define S5M8751_IRQB_MASK2			0x03

#define S5M8751_UVLO				0x09

#define S5M8751_CHG_TEST_WR			0x3C
#define S5M8751_CHG_TRIM			0x3D
#define S5M8571_BUCK_TEST1			0x3E
#define S5M8751_VREF_TEST			0x3F
#define S5M8751_BUCK_TEST2			0x40
#define S5M8751_LDO_OCPEN			0x42
#define S5M8751_CHIP_ID				0x43
#define S5M8751_STATUS				0x44
#define S5M8751_AUDIO_STATUS			0x45
#define S5M8751_CHG_TEST_R			0x46

#define S5M8751_MAS_REGISTER			0xFF


/*
 * Field Definitions
 */

/*
 * R0 (0x00) - IRQB_EVENT1
 */
#define S5M8751_IRQB_EVENT1_MASK		0x0F
#define S5M8751_PWRKEY1B			0x08
#define S5M8751_PWRKEY2B			0x04
#define S5M8751_PWRKEY3				0x02
#define S5M8751_PWRKEY4				0x01

/*
 * R1 (0x01) - IRQB_EVENT2
 */
#define S5M8751_IRQB_EVENT2_MASK		0x1F
#define S5M8751_VCHG_DET			0x10
#define S5M8751_VCHG_REM			0x08
#define S5M8751_CHG_T_OUT			0x04
#define S5M8751_CHG_BATT_DET			0x02
#define S5M8751_CHG_EOC				0x01

/*
 * R2 (0x02) - IRQB_MASK1
 */
#define S5M8751_IRQB_MASK1_MASK			0x0F
#define S5M8751_MASK_PWRKEY1B			0x08
#define S5M8751_MASK_PWRKEY2B			0x04
#define S5M8751_MASK_PWRKEY3			0x02
#define S5M8751_MASK_PWRKEY4			0x01

/*
 * R3 (0x03) - IRQB_MASK2
 */
#define S5M8751_IRQB_MASK2_MASK			0x1F
#define S5M8751_MASK_VCHG_DET			0x10
#define S5M8751_MASK_VCHG_REM			0x08
#define S5M8751_MASK_CHG_T_OUT			0x04
#define S5M8751_MASK_CHG_BATT_DET		0x02
#define S5M8751_CHG_EOC				0x01

/*
 * R9 (0x09) - UVLO
 * value (Max:111[3.7V]  ~  Min:000[3.0V])
 */
#define S5M8751_UVLO_MASK			0x07

/*
 * R60 (0x3C) - CHG_TEST_WR
 */
#define S5M8751_CHG_TEST_WR_MASK		0x1F
#define S5M8751_TEST_FURST			0x10
#define S5M8751_NOBATT_ENB			0x08
#define S5M8751_TMFUNC				0x04
#define S5M8751_TIMER				0x02
#define S5M8751_TIMER_SEL_OUT			0x01

/*
 * R61 (0x3D) - CHG_TRIM
 */
#define S5M8751_CHG_TRIM_MASK			0x3F
#define S5M8751_V_TRIM_MASK			0x38
#define S5M8751_V_TRIM_SHIFT			3
#define S5M8751_I_TRIM_MASK			0x07
#define S5M8751_I_TRIM_SHIFT			0

/*
 * R62 (0x3E) - BUCK_TEST1
 */
#define S5M8751_BUCK_TEST1_MASK			0xFF
#define S5M8751_LEB1_MASK			0xC0
#define S5M8751_LEB1_SHIFT			6
#define S5M8751_BUCK1_DEADTIME_MASK		0x30
#define S5M8751_BUCK1_DEADTIME_SHIFT		4
#define S5M8751_LEB2_MASK			0x0C
#define S5M8751_LEB2_SHIFT			2
#define S5M8751_BUCK2_DEADTIME_MASK		0x03
#define S5M8751_BUCK2_DEADTIME_SHIFT		0

/*
 * R63 (0x3F) - VREF_TEST
 */
#define S5M8751_VREF_TRIM_MASK			0x1F
#define S5M8751_VREF_TRIM_SHIFT			0

/*
 * R64 (0x40) - BUCK_TEST2
 */
#define S5M8751_BUCK_TEST2_MASK			0x0F
#define S5M8751_BUCK2_PFM_TH_MASK		0x0C
#define S5M8751_BUCK2_PFM_TH_SHIFT		2
#define S5M8751_BUCK1_PFM_TH_MASK		0x03
#define S5M8751_BUCK1_PFM_TH_SHIFT		0

/*
 * R66 (0x42) - LDO_OCPEN
 */
#define S5M8751_LDO_OCPEN_MASK			0xFF
#define S5M8751_LDO_OCPEN_SHIFT			0

/*
 * R67 (0x43) - CHIP_ID
 */
#define S5M8751_CHIP_ID_MASK			0xFF
#define S5M8751_CHIP_ID_SHIFT			0

/*
 * R68 (0x44) - STATUS
 */
#define S5M8751_STATUS_MASK			0x1F
#define S5M8751_STATUS_PWRKEY1B			0x10
#define S5M8751_STATUS_PWRKEY2B			0x08
#define S5M8751_STATUS_PWRKEY3			0x04
#define S5M8751_STATUS_PWRKEY4			0x02
#define S5M8751_STATUS_CHG_STATEB		0x01

/*
 * R69 (0x45) - AUDIO_STATUS
 */
#define S5M8751_AUDIO_STATUS_MASK		0x07
#define S5M8751_STATUS_OCPP			0x04
#define S5M8751_STATUS_OCPN			0x02
#define S5M8751_STATUS_SPK_DOWN			0x01

/*
 * R70 (0x46) - CHG_TEST_R
 */
#define S5M8751_CHG_TEST_R_MASK			0xFF
#define S5M8751_NTCDOWNB_TEST			0x80
#define S5M8751_PRE_CHG_TEST			0x40
#define S5M8751_AUTO_RECHG_TEST			0x20
#define S5M8751_ENCHG_TEST			0x10
#define S5M8751_BATDET_TEST			0x08
#define S5M8751_EOC_TEST			0x04
#define S5M8751_CHGDETDEL_TEST			0x02
#define S5M8751_CHGDET_TEST			0x01


/*
 * S5M8751 Interrupts
 */
#define S5M8751_IRQ_PWRKEY1B			0
#define S5M8751_IRQ_PWRKEY2B			1
#define S5M8751_IRQ_PWRKEY3			2
#define S5M8751_IRQ_PWRKEY4			3
#define S5M8751_IRQ_CHARGER_REMOVAL		4
#define S5M8751_IRQ_USB_DEVICE_REMOVAL		5
#define S5M8751_IRQ_CHARGER_TIMEOUT		6
#define S5M8751_IRQ_BATTERY_DETECTION		7
#define S5M8751_IRQ_CHARGE_COMPLETION		8

#define S5M8751_NUM_IRQ				9
#define S5M8751_MAX_REGISTER			0xFF

#define S5M8751_NUMREGS				0x3C	

struct s5m8751;

struct s5m8751_irq {
	void (*handler) (struct s5m8751 *, int, void *);
	void *data;
};

struct s5m8751 {
	int rev;		/* chip revision */
	struct device *dev;
	struct i2c_client *i2c_client;
	
	int (*read_dev) (struct s5m8751 *s5m8751, char reg, int size, void *dest);
	int (*write_dev) (struct s5m8751 *s5m8751, char reg, int size, void *src);
	u16 *reg_cache;

	struct mutex irq_mutex;
	struct s5m8751_irq irq[S5M8751_NUM_IRQ];
	int chip_irq;
};

struct s5m8751_platform_data {
	int (*init)(struct s5m8751 *s5m8751);
};

static inline struct s5m8751 *dev_to_s5m8751(struct device *dev)
{
	return dev_get_drvdata(dev->parent);
}

int s5m8751_client_register(struct s5m8751 *s5m8751, struct platform_device *pdev);

void s5m8751_device_exit(struct s5m8751 *s5m8751);

int s5m8751_clear_bits(struct s5m8751 *s5m8751, u8 reg, u8 mask);
int s5m8751_set_bits(struct s5m8751 *s5m8751, u8 reg, u8 mask);

u8 s5m8751_reg_read(struct s5m8751 *s5m8751, int reg);
int s5m8751_reg_write(struct s5m8751 *s5m8751, int reg, u8 val);

int s5m8751_mask_irq(struct s5m8751 *s5m8751, int irq);
int s5m8751_unmask_irq(struct s5m8751 *s5m8751, int irq);

int s5m8751_i2c_read_device(struct s5m8751 *s5m8751, char reg, int bytes, void *dest);

int s5m8751_i2c_write_device(struct s5m8751 *s5m8751, char reg, int bytes, void *src);

#endif


