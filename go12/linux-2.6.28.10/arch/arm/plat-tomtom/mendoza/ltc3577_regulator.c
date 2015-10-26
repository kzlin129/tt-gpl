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
#include <linux/platform_device.h>
#include <linux/ltc3577-pmic.h>
#include <linux/ltc3577-regulator.h>
#include <linux/err.h>
#include <plat/tt_setup_handler.h>

#define LTC3577_REG_PFX		LTC3577_REG_DEVNAME " Dev: "

/*
 * LED Control Register
 */
#define EN		0x01
#define GR1		0x02
#define GR2		0x04
#define MD1		0x08
#define MD2		0x10
#define PWMC1	0x20
#define PWMC2	0x40


/*
 * Gradations
 */
#define GR_15ms			0x00
#define GR_460ms		GR1
#define GR_930ms		GR2
#define GR_1_85s		(GR1 | GR2)


/*
 * Configurations
 */
#define DISABLE			0x00 
#define CC				(EN | GR_1_85s)
#define PWM				(EN | MD2 |GR_1_85s)

#define PWMC_8_77kHz	PWM
#define PWMC_4_39kHz	(PWM | PWMC1)
#define PWMC_2_92kHz	(PWM | PWMC2)
#define PWMC_2_19kHz	(PWM | PWMC1 | PWMC2)

#define DAC_MIN_VALUE	43
#define DAC_MAX_VALUE	63

#define PWM_DUTYC_OFF	0x01

/*
 * i2c channel
 */

/* i2c message layout */
#define I2C_MSG_SIZE		2
#define I2C_MSG_SUBADDR_IDX	0
#define I2C_MSG_PAYLOAD_IDX	1

/* i2c register sub-addresses */
#define CTRL_REG_SUBADDR	0x01
#define DAC_REG_SUBADDR		0x02
#define PWM_REG_SUBADDR		0x03

typedef enum 
{
	eDEACTIVATE, eACTIVATE
} activate_e;

/*
 * Lookup Table
 */

typedef struct {
	unsigned char intensity;
	unsigned char control_reg;
	unsigned char dac_reg;
	unsigned char pwm_reg;
} ltc3577_reg_calibration_t;

static struct semaphore ltc3577_reg_mutex;

static ltc3577_reg_calibration_t ltc3577_reg_calibration_table[] = {
	{
		.intensity		= LTC3577_REG_POWER_MIN,
		.control_reg	= DISABLE,
		.dac_reg		= DAC_MIN_VALUE,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 5,
		.control_reg	= CC,
		.dac_reg		= 45,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 10,
		.control_reg	= CC,
		.dac_reg		= 48,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 15,
		.control_reg	= CC,
		.dac_reg		= 50,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 20,
		.control_reg	= CC,
		.dac_reg		= 51,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 25,
		.control_reg	= CC,
		.dac_reg		= 52,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 30,
		.control_reg	= CC,
		.dac_reg		= 53,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 35,
		.control_reg	= CC,
		.dac_reg		= 55,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 40,
		.control_reg	= CC,
		.dac_reg		= 56,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 45,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 126,
	},
	{
		.intensity		= 50,
		.control_reg	= CC,
		.dac_reg		= 57,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 55,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 159,
	},
	{
		.intensity		= 60,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 158,
	},
	{
		.intensity		= 65,
		.control_reg	= CC,
		.dac_reg		= 59,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 70,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 191,
	},
	{
		.intensity		= 75,
		.control_reg	= CC,
		.dac_reg		= 60,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
	{
		.intensity		= 80,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 155,
	},
	{
		.intensity		= 85,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 223,
	},
	{
		.intensity		= 90,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 188,
	},
	{
		.intensity		= 95,
		.control_reg	= PWMC_8_77kHz,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= 239,
	},
	{
		.intensity		= LTC3577_REG_POWER_MAX,
		.control_reg	= CC,
		.dac_reg		= DAC_MAX_VALUE,
		.pwm_reg		= PWM_DUTYC_OFF,
	},
};

static inline int nb_calibration_item(void)
{
	return (sizeof(ltc3577_reg_calibration_table)/sizeof(ltc3577_reg_calibration_t));
}

static int ltc3577_reg_get_calibration(int intensity, ltc3577_reg_calibration_t **calibration)
{
	int ret = -EINVAL;
	int i;

	BUG_ON(!calibration);

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

static void ltc3577_reg_free_msgs(ltc3577_msg_t *msgs)
{
	up(&ltc3577_reg_mutex);
}


static int ltc3577_reg_send_calibration(int intensity, ltc3577_reg_calibration_t *calibration)
{
	static activate_e current_state  = eACTIVATE;

	static ltc3577_msg_t msgs[3];
	static unsigned char control[I2C_MSG_SIZE] ;
	static unsigned char dac[I2C_MSG_SIZE] ;
	static unsigned char pwm[I2C_MSG_SIZE] ;

	int sleep = 1;

	BUG_ON(!calibration);

	if (down_interruptible(&ltc3577_reg_mutex))
		return -ERESTARTSYS;

	control[I2C_MSG_SUBADDR_IDX] = CTRL_REG_SUBADDR;
	control[I2C_MSG_PAYLOAD_IDX] = calibration->control_reg;

	dac[I2C_MSG_SUBADDR_IDX] = DAC_REG_SUBADDR;
	dac[I2C_MSG_PAYLOAD_IDX] = calibration->dac_reg;

	pwm[I2C_MSG_SUBADDR_IDX] = PWM_REG_SUBADDR;
	pwm[I2C_MSG_PAYLOAD_IDX] = calibration->pwm_reg;

	msgs[0].buf	= control;
	msgs[0].len	= I2C_MSG_SIZE;
	msgs[1].buf	= dac;
	msgs[1].len	= I2C_MSG_SIZE;
	msgs[2].buf	= pwm;
	msgs[2].len	= I2C_MSG_SIZE;

	if (0 == intensity) {
		if (eACTIVATE == current_state) {
			current_state = eDEACTIVATE;
			sleep = 0;
		}
	} else if (eDEACTIVATE == current_state) {
			current_state = eACTIVATE;
			sleep = 0;
	}

	if (unlikely(0 == sleep))
		ltc3577_i2c_transfer_async(msgs, 3, ltc3577_reg_free_msgs);
	else {
		ltc3577_i2c_transfer(msgs, 3);
		up(&ltc3577_reg_mutex);
	}

	return 0;
}

/*
 * LTC3577 Regulator Device
 */

static int ltc3577_reg_set_power(int power)
{
	int ret = 0;
	ltc3577_reg_calibration_t *calibration = NULL;

	ret = ltc3577_reg_get_calibration(power, &calibration);
	if (0 == ret)
		ret = ltc3577_reg_send_calibration(power, calibration);

	return ret;
}

int ltc3577_bl_init(void);

static ltc3577_reg_pdata_t ltc3577_reg_pdata = {
	.init		= ltc3577_bl_init,
	.set_power	= ltc3577_reg_set_power,
};
		
static struct platform_device ltc3577_reg_device = {
	.name		= LTC3577_REG_DEVNAME,
	.dev		= {
		.platform_data	= &ltc3577_reg_pdata,
	},
	.id		= -1,
};

static int __init ltc3577_reg_init(void)
{
    int ret = 0;

	init_MUTEX(&ltc3577_reg_mutex);

	if (0 == (ret = platform_device_register(&ltc3577_reg_device))) {
		printk(KERN_INFO LTC3577_REG_PFX "Registered\n");
	} else {
		printk(KERN_ERR LTC3577_REG_PFX "Registration FAILURE\n");
	}

	return ret;
}



static int __init ltc3577_bl_setup_cb(char * identifier)
{
	return ltc3577_reg_init();
};

TT_SETUP_CB(ltc3577_bl_setup_cb, LTC3577_REG_DEVNAME);

