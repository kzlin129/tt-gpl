/*****************************************************************************
 * Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 *****************************************************************************/

/*
 *
 *****************************************************************************
 *
 *  pmu_device_bcm59002.h
 *
 *  PURPOSE:
 *
 *  This file defines the interface to the Broadcom BCM59002 PMU chip.
 *
 *  NOTES:
 *
 *****************************************************************************/


#if !defined( PMU_DEVICE_BCM59002_H )
#define PMU_DEVICE_BCM59002_H

#include <linux/types.h>
#include <asm/plat-bcm/pmu_device_bcm.h>

#define BCM59040_DEVNAME "bcm59040"

typedef enum bcm59040_led_pattern_e {
	BCMPMU_LED_PAT_50_OFF=0,
	BCMPMU_LED_PAT_100_OFF,
	BCMPMU_LED_PAT_200_OFF,
	BCMPMU_LED_PAT_500_OFF,
	BCMPMU_LED_PAT_50_50_50_OFF,
	BCMPMU_LED_PAT_100_100_100_OFF,
	BCMPMU_LED_PAT_200_200_200_OFF,
	BCMPMU_LED_PAT_ON
} bcm59040_led_pattern_t;

struct pmu_bcm59040_platform_data
{
    struct pmu_board_info                   *pmu_board_info;
    int                                     pmu_board_info_entries;
};

struct bcm59040_batt_defaults
{
	uint8_t	fgctrl1;
	uint8_t fgctrl2;
	uint8_t fgctrl3;
};

struct bcm59040_charger_defaults
{
	uint8_t mbcctrl1;
	uint8_t mbcctrl2;
	uint8_t mbcctrl3;
	uint8_t mbcctrl4;
	uint8_t mbcctrl5;
	uint8_t mbcctrl6;
	uint8_t mbcctrl7;
	uint8_t mbcctrl8;
	uint8_t mbcctrl9;
	uint8_t mbcctrl10;

	uint8_t	ntcctrl1;
	uint8_t ntcctrl2;
	uint8_t	mbtemphys;
	uint32_t battery_capacity;
};

struct bcm59040_control_defaults
{
	uint8_t poncntrl1;
	uint8_t poncntrl2;
	uint8_t rstrtcntrl;
};

typedef struct bcm59040_client_platform_data_struct {;
	bcmpmu_client_platform_data_t	base;		/* basic information standard */
							/* for all Broadcom PMUs */
	void				*extra;		/* extra data */
	void				*defaults;
} bcm59040_client_platform_data_t;

/*
 * Function declarations.
 */

extern struct pmu_client *bcm_get_pclient_from_client_type(struct pmu_client *pclient,
														   bcmpmu_clients_t client_type);

extern int bcm59040_get_charger_status(struct pmu_client *pclient);
extern int bcm59040_charger_is_charging(struct pmu_client *pclient);
extern int bcm59040_get_batt_voltage(struct pmu_client *pclient);
extern int bcm59040_get_avg_batt_voltage(struct pmu_client *pclient);

#endif /* PMU_DEVICE_BCM59040_H */
