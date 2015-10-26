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
*  pmu_device_bcm.h
*
*  PURPOSE:
*
* This file contains information specific to Broadcom PMU devices but is
* intended to be of generic use to different Broadcom PMU devices.
*
*****************************************************************************/


#if !defined( PMU_DEVICE_BCM_H )
#define PMU_DEVICE_BCM_H

typedef enum bcmpmu_clients_e {
	BCMPMU_ADC=0,
	BCMPMU_BATTERY,
	BCMPMU_CHARGER,
	BCMPMU_CONTROL,
	BCMPMU_GPIO,
	BCMPMU_REGULATOR,
	BCMPMU_RTC,
	BCMPMU_USBSTAT,
	BCMPMU_MAX_CLIENT_TYPES
} bcmpmu_clients_t;

typedef struct bcmpmu_client_platform_data_struct {
	bcmpmu_clients_t	client_type;		/* type of PMU client */
	int					index;				/* -1 if only 1, 0, 1, 2, etc. */
											/* if multiple clients of same type */
} bcmpmu_client_platform_data_t;

typedef enum bcmpmu_charger_status_e {
	BCM_CHRGR_REMOVED=0,
	BCM_CHRGR_INSERTED,
	BCM_CHRGR_CHARGING,
	BCM_CHRGR_TRICKLE,
	BCM_CHRGR_RAPID,
	BCM_CHRGR_PAUSED,
	BCM_CHRGR_STOPPED,
	BCM_CHRGR_COMPLETE,
	BCM_CHRGR_ERROR
} bcmpmu_charger_status_t;

typedef enum bcmpmu_usb_event_e {
	BCMPMU_USB_CONNECTEVENT=0,
	BCMPMU_USB_DISCONNECTEVENT,
	BCMPMU_USB_IDCHANGEEVENT,
} bcmpmu_usb_event_t;

typedef void (*bcmpmu_usb_callback_t)(void *cookie, bcmpmu_usb_event_t event, int arg);

/*
 * Global function definitions.
 */

//int bcmpmu_usb_callback_register(bcmpmu_usb_callback_t callback, void *magic);
int bcmpmu_usb_callback_unregister(bcmpmu_usb_callback_t callback, void *magic);

#endif /* PMU_DEVICE_BCM_H */
