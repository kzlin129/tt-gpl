/*****************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
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
*  adc.h
*
*  PURPOSE:
*
*  This file defines the user mode interface to the ADC driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_BROADCOM_ADC_H )
#define LINUX_BROADCOM_ADC_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

typedef struct brcm_adc_req {
	unsigned short	device;
	unsigned short	channel;
	unsigned short	sample;
} brcm_adc_req_t;

#define BRCM_ADC_MAGIC   'r'

#define BRCM_ADC_CMD_FIRST              0x80
#define BRCM_ADC_CMD_GET_NUM_DEVICES    0x81
#define BRCM_ADC_CMD_GET_NUM_CHANNELS   0x82
#define BRCM_ADC_CMD_GET_CAPS           0x83
#define BRCM_ADC_CMD_REQUEST            0x84
#define BRCM_ADC_CMD_CANCEL_REQUEST     0x85
#define BRCM_ADC_CMD_LAST               0x86

// time is in UTC time format
#define BRCM_ADC_IOCTL_GET_NUM_DEVICES _IO( BRCM_ADC_MAGIC, BRCM_ADC_CMD_GET_NUM_DEVICES )
#define BRCM_ADC_IOCTL_GET_NUM_CHANNELS _IOR( BRCM_ADC_MAGIC, BRCM_ADC_CMD_GET_NUM_CHANNELS, int )
#define BRCM_ADC_IOCTL_GET_CAPS _IOWR( BRCM_ADC_MAGIC, BRCM_ADC_CMD_GET_CAPS, brcm_adc_req_t* )
#define BRCM_ADC_IOCTL_REQUEST _IOWR( BRCM_ADC_MAGIC, BRCM_ADC_CMD_REQUEST, brcm_adc_req_t* )
#define BRCM_ADC_IOCTL_CANCEL_REQUEST _IOWR( BRCM_ADC_MAGIC, BRCM_ADC_CMD_CANCEL_REQUEST, brcm_adc_req_t* )

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_BROADCOM_ADC_H */
