/*
 *  LTC3577 Regulator PLATFORM DEVICE 
 *
 *  Copyright (c) 2004-2009 Benoit Leffray
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <barcelona/gopins.h>
#include <linux/ltc3577-pmic.h>
#include <linux/ltc3577-regulator.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/semaphore.h>

#define LTC3577_REG_PFX		LTC3577_REG_DEVNAME " Dev: "

/*
 * LED Control Register
 */
#define EN			(0x01)
#define GR1			(0x02)
#define GR2			(0x04)
#define MD1			(0x08)
#define MD2			(0x10)
#define PWMC1			(0x20)
#define PWMC2			(0x40)
#define SLEWLED			(0x80)

/*
 * Gradations
 */
#define GR_15ms			(0x00)
#define GR_460ms		(GR1)
#define GR_930ms		(GR2)
#define GR_1_85s		(GR1 | GR2)


/*
 * Configurations
 */
#define DISABLE			(0x00 )
#define DISABLE_GR		(GR_460ms)
#define CC			(EN | GR_460ms)
#define PWM			(EN | MD2 | GR_460ms)

#define PWMC_8_77kHz		(PWM)
#define PWMC_4_39kHz		(PWM | PWMC1)
#define PWMC_2_92kHz		(PWM | PWMC2)
#define PWMC_2_19kHz		(PWM | PWMC1 | PWMC2)

#define DAC_MIN_VALUE		(43)
#define DAC_MAX_VALUE		(63)

#define PWM_DUTYC_OFF		(0x01)

/*
 * i2c channel
 */

/* i2c message layout */
#define I2C_MSG_SIZE		(2)
#define I2C_MSG_SUBADDR_IDX	(0)
#define I2C_MSG_PAYLOAD_IDX	(1)

/* i2c register sub-addresses */
#define LTC3577_ADDR_BUCK_CTRL	(0x0)
#define LTC3577_ADDR_LED_CTRL	(0x1)
#define LTC3577_ADDR_LED_DAC	(0x2)
#define LTC3577_ADDR_LED_PWM	(0x3)
#define LTC3577_ADDR_MAX	(0x4)
	
/*
 * Lookup Table
 */

typedef struct {
	unsigned char intensity;
	unsigned char control_reg;
	unsigned char dac_reg;
	unsigned char pwm_reg;
} ltc3577_reg_calibration_t;

#define PERC_2_LTC3577_INT(x)  ((LTC3577_REG_POWER_MAX - LTC3577_REG_POWER_MIN) * x / 100)

DECLARE_MUTEX(ltc3577_reg_mutex);
static ltc3577_msg_t ltc3577_write[LTC3577_ADDR_MAX];

#define CALIBRATION_ENTRY_3577(i,c,d,p) \
	{	.intensity		= i, \
		.control_reg		= c, \
		.dac_reg		= d, \
		.pwm_reg		= p \
	} 
static ltc3577_reg_calibration_t ltc3577_reg_calibration_table[] = {
	/*		      intensity	control_reg	dac_reg	pwm_reg */
	CALIBRATION_ENTRY_3577(0,	DISABLE,	0x19,	0xFF),
	CALIBRATION_ENTRY_3577(5,	EN,		0x24,	0xFF),
	CALIBRATION_ENTRY_3577(10,	EN,		0x2A,	0xFF),
	CALIBRATION_ENTRY_3577(15,	EN,		0x2D,	0xFF),
	CALIBRATION_ENTRY_3577(20,	EN,		0x30,	0xFF),
	CALIBRATION_ENTRY_3577(25,	EN | MD2,	0x3F,	0x4F),
	CALIBRATION_ENTRY_3577(30,	EN,		0x33,	0xFF),
	CALIBRATION_ENTRY_3577(35,	EN | MD2,	0x3E,	0x6F),
	CALIBRATION_ENTRY_3577(40,	EN,		0x36,	0xFF),
	CALIBRATION_ENTRY_3577(45,	EN | MD2,	0x3F,	0x7F),
	CALIBRATION_ENTRY_3577(50,	EN,		0x38,	0xFF),
	CALIBRATION_ENTRY_3577(55,	EN,		0x39,	0xFF),
	CALIBRATION_ENTRY_3577(60,	EN | MD2,	0x3F,	0x9F),
	CALIBRATION_ENTRY_3577(65,	EN | MD2,	0x3F,	0xAF),
	CALIBRATION_ENTRY_3577(70,	EN,		0x3B,	0xFF),
	CALIBRATION_ENTRY_3577(75,	EN | MD2,	0x3F,	0xBF),
	CALIBRATION_ENTRY_3577(80,	EN,		0x3C,	0xFF),
	CALIBRATION_ENTRY_3577(85,	EN | MD2,	0x3F,	0xCF),
	CALIBRATION_ENTRY_3577(90,	EN | MD2,	0x3F,	0xDF),
	CALIBRATION_ENTRY_3577(95,	EN,		0x3E,	0xFF),
	CALIBRATION_ENTRY_3577(100,	EN,		0x3F,	0xFF),
};
static ltc3577_reg_calibration_t ltc3577_reg_calibration_table_cc[] = {
	/*		      intensity	control_reg	dac_reg	pwm_reg */
	CALIBRATION_ENTRY_3577(0,	DISABLE,	0,	0xFF),
	CALIBRATION_ENTRY_3577(5,	EN | SLEWLED,	13,	0xFF),
	CALIBRATION_ENTRY_3577(10,	EN | SLEWLED,	16,	0xFF),
	CALIBRATION_ENTRY_3577(15,	EN | SLEWLED,	19,	0xFF),
	CALIBRATION_ENTRY_3577(20,	EN | SLEWLED,	22,	0xFF),
	CALIBRATION_ENTRY_3577(25,	EN | SLEWLED,	25,	0xFF),
	CALIBRATION_ENTRY_3577(30,	EN | SLEWLED,	28,	0xFF),
	CALIBRATION_ENTRY_3577(35,	EN | SLEWLED,	31,	0xFF),
	CALIBRATION_ENTRY_3577(40,	EN | SLEWLED,	34,	0xFF),
	CALIBRATION_ENTRY_3577(45,	EN | SLEWLED,	37,	0xFF),
	CALIBRATION_ENTRY_3577(50,	EN | SLEWLED,	40,	0xFF),
	CALIBRATION_ENTRY_3577(55,	EN | SLEWLED,	43,	0xFF),
	CALIBRATION_ENTRY_3577(60,	EN | SLEWLED,	46,	0xFF),
	CALIBRATION_ENTRY_3577(65,	EN | SLEWLED,	49,	0xFF),
	CALIBRATION_ENTRY_3577(70,	EN | SLEWLED,	51,	0xFF),
	CALIBRATION_ENTRY_3577(75,	EN | SLEWLED,	53,	0xFF),
	CALIBRATION_ENTRY_3577(80,	EN | SLEWLED,	55,	0xFF),
	CALIBRATION_ENTRY_3577(85,	EN | SLEWLED,	57,	0xFF),
	CALIBRATION_ENTRY_3577(90,	EN | SLEWLED,	59,	0xFF),
	CALIBRATION_ENTRY_3577(95,	EN | SLEWLED,	61,	0xFF),
	CALIBRATION_ENTRY_3577(100,	EN | SLEWLED,	63,	0xFF),
};
#undef CALIBRATION_ENTRY_3577

static inline int nb_calibration_item(void)
{
	return ARRAY_SIZE(ltc3577_reg_calibration_table);
}

static int ltc3577_reg_get_calibration(int intensity, ltc3577_reg_calibration_t **calibration)
{
	int ret = -EINVAL;
	int i;
	static int init = 0;

	BUG_ON(!calibration);

	if (!init)
	{
		if (IO_GetBacklightCCMode())
			memcpy(ltc3577_reg_calibration_table,ltc3577_reg_calibration_table_cc,sizeof(ltc3577_reg_calibration_table));
		init = 1;
	}

	if (intensity < LTC3577_REG_POWER_MIN)
		intensity = LTC3577_REG_POWER_MIN;

	if (intensity > LTC3577_REG_POWER_MAX)
		intensity = LTC3577_REG_POWER_MAX;

	for (	i = 0 ; 
			   (i < nb_calibration_item())
			&& (intensity > ltc3577_reg_calibration_table[i].intensity) ;  
			i ++);

	if (i < nb_calibration_item()) {
		*calibration = &ltc3577_reg_calibration_table[i];
		ret = 0;
	} else {
		ret = -ENODEV;
	}

	return ret;
}

static int ltc3577_reg_send_calibration(int intensity, ltc3577_reg_calibration_t *calibration)
{
	BUG_ON(!calibration);

	down(&ltc3577_reg_mutex);
	ltc3577_set_reg(ltc3577_write, LTC3577_ADDR_LED_CTRL,
			calibration->control_reg);
	ltc3577_set_reg(ltc3577_write, LTC3577_ADDR_LED_DAC,
			calibration->dac_reg);
	ltc3577_set_reg(ltc3577_write, LTC3577_ADDR_LED_PWM,
			calibration->pwm_reg);
	ltc3577_commit(ltc3577_write, NULL, 0);
	up(&ltc3577_reg_mutex);
	return 0;
}

/*
 * LTC3577 Regulator Device
 */
int ltc3577_regulator_set_power(int power)
{
	int ret = 0;
	ltc3577_reg_calibration_t *calibration = NULL;

	ret = ltc3577_reg_get_calibration(power, &calibration);
	if (0 == ret)
		ret = ltc3577_reg_send_calibration(power, calibration);

	return ret;
}
EXPORT_SYMBOL(ltc3577_regulator_set_power);

