/*
 * s5m8751_pmic.h  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef __LINUX_MFD_S5M8751_PMIC_H_
#define __LINUX_MFD_S5M8751_PMIC_H_

/*
 * Register values
 */

#define S5M8751_ONOFF1				0x04
#define S5M8751_ONOFF2				0x05
#define S5M8751_ONOFF3				0x06

#define S5M8751_SLEEP_CNTL1			0x07
#define S5M8751_SLEEP_CNTL2			0x08

#define S5M8751_LDO_AUDIO_VSET			0x0A
#define S5M8751_LDO1_VSET			0x0B
#define S5M8751_LDO2_VSET			0x0C
#define S5M8751_LDO3_VSET			0x0D
#define S5M8751_LDO4_VSET			0x0E
#define S5M8751_LDO_MEMORY_VSET			0x0F

#define S5M8751_BUCK1_V1_SET			0x10
#define S5M8751_BUCK1_V2_SET			0x11
#define S5M8751_BUCK2_V1_SET			0x12
#define S5M8751_BUCK2_V2_SET			0x13
#define S5M8751_CHG_IV_SET			0x15

#define S5M8751_LDO_RTC_VSET			0x41

/*
 * R4 (0x04) - ONOFF1
 */
#define S5M8751_ONOFF1_MASK			0x03
#define S5M8751_SLEEPB_PIN_EN			0x02
#define S5M8751_SLEEP_MODE			0x01

/*
 * R5 (0x05) - ONOFF2
 */
#define S5M8751_ONOFF2_MASK			0x1F
#define S5M8751_LDO4_ENA			0x10
#define S5M8751_LDO4_SHIFT			4
#define S5M8751_LDO3_ENA			0x08
#define S5M8751_LDO3_SHIFT			3
#define S5M8751_LDO2_ENA			0x04
#define S5M8751_LDO2_SHIFT			2
#define S5M8751_LDO1_ENA			0x02
#define S5M8751_LDO1_SHIFT			1
#define S5M8751_LDO_MEMORY_ENA			0x01
#define S5M8751_LDO_MEMORY_SHIFT		0

/*
 * R6 (0x06) - ONOFF3
 */
#define S5M8751_ONOFF3_MASK			0x07
#define S5M8751_BUCK2_ENA			0x04
#define S5M8751_BUCK2_SHIFT			2
#define S5M8751_BUCK1_ENA			0x02
#define S5M8751_BUCK1_SHIFT			1
#define S5M8751_LDO_AUDIO_ENA			0x01
#define S5M8751_LDO_AUDIO_SHIFT			0

/*
 * R7 (0x07) - SLEEP_CNTL1
 */
#define S5M8751_SLEEP_CNTL1_MASK		0x1F
#define S5M8751_SLEEP_LDO4_ENA			0x10
#define S5M8751_SLEEP_LDO3_ENA			0x08
#define S5M8751_SLEEP_LDO2_ENA			0x04
#define S5M8751_SLEEP_LDO1_ENA			0x02
#define S5M8751_SLEEP_LDO_MEMORY_ENA		0x01

/*
 * R8 (0x08) - SLEEP_CNTL2
 */
#define S5M8751_SLEEP_CNTL2_MASK		0x07
#define S5M8751_SLEEP_BUCK2_ENA			0x04
#define S5M8751_SLEEP_BUCK1_ENA			0x02
#define S5M8751_SLEEP_LDO_AUDIO_ENA		0x01

/*
 * R10 (0x0A) - LDO_AUDIO_VSET
 */
#define S5M8751_LDO_AUDIO_VSET_MASK		0x0F
#define S5M8751_LDO_AUDIO_VSET_SHIFT		0

/*
 * R11 (0x0B) - LDO1_VSET
 */
#define S5M8751_LDO1_VSET_MASK			0x0F
#define S5M8751_LDO1_VSET_SHIFT			0

/*
 * R12 (0x0C) - LDO2_VSET
 */
#define S5M8751_LDO2_VSET_MASK			0x0F
#define S5M8751_LDO2_VSET_SHIFT			0

/*
 * R13 (0x0D) - LDO3_VSET
 */
#define S5M8751_LDO3_VSET_MASK			0x0F
#define S5M8751_LDO3_VSET_SHIFT			0

/*
 * R14 (0x0E) - LDO4_VSET
 */
#define S5M8751_LDO4_VSET_MASK			0x0F
#define S5M8751_LDO4_VSET_SHIFT			0

/*
 * R15 (0x0F) - LDO_MEMORY_VSET
 */
#define S5M8751_LDO_MEMORY_VSET_MASK		0x0F
#define S5M8751_LDO_MEMORY_VSET_SHIFT		0

/*
 * R16 (0x10) - BUCK1_V1_SET
 */
#define S5M8751_BUCK1_RAMP_TIME_MASK		0xC0
#define S5M8751_BUCK1_RAMP_TIME_SHIFT		6

#define S5M8751_BUCK1_V1_SET_MASK		0x3F
#define S5M8751_BUCK1_V1_SET_SHIFT		0

/*
 * R17 (0x11) - BUCK1_V2_SET
 */
#define S5M8751_BUCK1_MODE_MASK			0xC0
#define S5M8751_BUCK1_MODE_SHIFT		6

#define S5M8751_BUCK1_V2_SET_MASK		0x3F
#define S5M8751_BUCK1_V2_SET_SHIFT		0

/*
 * R18 (0x12) - BUCK2_V1_SET
 */
#define S5M8751_BUCK2_RAMP_TIME_MASK		0xC0
#define S5M8751_BUCK2_RAMP_TIME_SHIFT		6

#define S5M8751_BUCK2_V1_SET_MASK		0x3F
#define S5M8751_BUCK2_V1_SET_SHIFT		0

/*
 * R19 (0x13) - BUCK2_V2_SET
 */
#define S5M8751_BUCK2_MODE_MASK			0xC0
#define S5M8751_BUCK2_MODE_SHIFT		6

#define S5M8751_BUCK2_V2_SET_MASK		0x3F
#define S5M8751_BUCK2_V2_SET_SHIFT		0

/*
 * R19 (0x15) - CHG_IV_SET
 */
#define S5M8751_REGVTGSET_MASK			0xe0
#define S5M8751_REGVTGSET_SHIFT			5

#define S5M8751_AUTOMODE_DISABLE_MASK		0x10
#define S5M8751_AUTOMODE_DISABLE_SHIFT		4

#define S5M8751_FCHG_CSET_MASK			0x0f
#define S5M8751_FCHG_CSET_SHIFT			0

/*
 * S5M8751 BUCK RAMP TIME
 */

#define S5M8751_BUCK_RAMP_TIME_64US		0
#define S5M8751_BUCK_RAMP_TIME_128US		1
#define S5M8751_BUCK_RAMP_TIME_256US		2
#define S5M8751_BUCK_RAMP_TIME_512US		3

/*
 * S5M8751 BUCK MODE
 */
#define S5M8751_BUCK_MODE_PFM			0
#define S5M8751_BUCK_MODE_PWM			1
#define S5M8751_BUCK_MODE_AUTO			2
#define S5M8751_BUCK_MODE_UNUSED		3


/*
 * R65 (0x41) - LDO_RTC_VSET
 */
#define S5M8751_LDO_RTC_VSET_MASK		0x0F
#define S5M8751_LDO_RTC_VSET_SHIFT		0


/*
 * S5M8751 LDOs
 */
#define S5M8751_LDO_AUDIO			0
#define S5M8751_LDO1				1
#define S5M8751_LDO2				2
#define S5M8751_LDO3				3
#define S5M8751_LDO4				4
#define S5M8751_LDO_MEMORY			5

/*
 * S5M8751 BUCKs
 */
#define S5M8751_BUCK1_1				6
#define S5M8751_BUCK1_2				7
#define S5M8751_BUCK2_1				8
#define S5M8751_BUCK2_2				9

#define NUM_S5M8751_REGULATORS			10

/*
 * Sleep/Active mode state of LDOs and Bucks
 */
#define S5M8751_SLPB_EN				(1<<0)
#define S5M8751_LDO1_EN				(1<<1)
#define S5M8751_LDO2_EN				(1<<2)
#define S5M8751_LDO3_EN				(1<<3)
#define S5M8751_LDO4_EN				(1<<4)
#define S5M8751_LDO5_EN				(1<<5)
#define S5M8751_LDO6_EN				(1<<6)
#define S5M8751_BCK1_EN				(1<<7)
#define S5M8751_BCK2_EN				(1<<8)

/*
 * Wall charger current
 */
#define S5M8751_WALLADAPTER_050			(0x0)
#define S5M8751_WALLADAPTER_100			(0x1)
#define S5M8751_WALLADAPTER_150			(0x2)
#define S5M8751_WALLADAPTER_200			(0x3)
#define S5M8751_WALLADAPTER_250			(0x4)
#define S5M8751_WALLADAPTER_300			(0x5)
#define S5M8751_WALLADAPTER_350			(0x6)
#define S5M8751_WALLADAPTER_400			(0x7)
#define S5M8751_WALLADAPTER_450			(0x8)
#define S5M8751_WALLADAPTER_500			(0x9)
#define S5M8751_WALLADAPTER_550			(0xa)
#define S5M8751_WALLADAPTER_600			(0xb)
#define S5M8751_WALLADAPTER_650			(0xc)
#define S5M8751_WALLADAPTER_700			(0xd)
#define S5M8751_WALLADAPTER_750			(0xe)
#define S5M8751_WALLADAPTER_800			(0xf)

struct platform_s5m8751_pmic_data
{
	uint32_t	sleep_mode_state;
	uint32_t	wall_pwr_pin;
	uint8_t		wall_pwr_current;

	/* Callbacks to handle the gpio configuration */
	int  (*request_gpio)(void);
	void (*config_gpio)(void);
	void (*free_gpio)(void);
	uint32_t (*get_charging)(void);
};
#endif
