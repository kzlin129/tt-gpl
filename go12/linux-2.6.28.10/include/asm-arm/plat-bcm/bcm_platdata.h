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
*  bcm_platdata.h
*
*  PURPOSE:
*
* This file defines the data structure and access functions for the Broadcom
* platform data configuration structure. This structure defines functions and
* data that defines the active hardware platform. Thus this structure is a
* combination of statically and dynamically configured data. Some of the
* platform data is determined by build-time configuration but much of it
* may be updated during boot time when the system configuration is detected.
*
* The concept is that this detection process could identify hardware platform
* unique information and other drivers/applications could use this information
* to enable, disable or appropriately configure themselves to the running
* hardware platform at run-time instead of having to be exclusively built
* for a particular platform and having multiple unique and incompatible
* kernel binaries.
*
*****************************************************************************/


#if !defined( BCM_PLATDATA_H )
#define BCM_PLATDATA_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>

#include <asm/arch/bcm_gen_battery.h>

#include <asm/arch/bcm_platinfo.h>

/* ---- Constants and Types ---------------------------------------------- */

#define BCM_RES_ENTRY_NAME_MAX		32			/* maximum size of a resource */
												/* entry name */

typedef enum bcm_platid_e {
	BCMRING
	BCMMIPS,
	BCM116x,
	BCM282x,
	BCM2850,
	BCM476x,
	BCM5892,
} bcm_platid_t;

typedef struct bcm_res_entry {
	struct list_head			links;			/* linked list maint */

	char	name[BCM_RES_ENTRY_NAME_MAX];		/* name of resource */
	union {
		uint8_t		ubyte;
		uint16_t	uhalfword;
		uint32_t	uword;
		uint64_t	udword;
		int8_t		sbyte;
		int16_t		shalfword;
		int32_t		sword;
		int64_t		sdword;
		void		*ptr;
	} data;										/* data associated with item */
} bcm_res_entry_t;

typedef struct bcm_platform_data_struct {
	struct {
		bcm_platid_t				device_id;	/* device ID */
		uint32_t					device_rev;	/* device revision # */
		bcm_board_t					board_id;	/* board ID */
	} plat_id;									/* platform ID structure */

	struct bcm_batt_info		*batt_info;		/* pointer to platforms battery */
												/* information structure */

	struct list_head			plat_resources;	/* linked list of platform */
												/* unique resources */

} bcm_platform_data_t;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function and Macro Prototypes ------------------------------------ */


#endif  /* BCM_PLATDATA_H */
