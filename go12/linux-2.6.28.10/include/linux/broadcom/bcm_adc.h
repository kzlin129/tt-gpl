/*****************************************************************************
* Copyright 2009 - 2009 Broadcom Corporation.  All rights reserved.
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

#ifndef BCM_ADC_H
#define BCM_ADC_H

#include <linux/list.h>

/*
 * Device and Channel information
 */
#if defined(CONFIG_PMU_DEVICE_BCM59040) || defined(CONFIG_BCM_PMU_BCM59040)
#define BCM_ADC_DEVICES				2
#else
#define BCM_ADC_DEVICES				1
#endif

#define BCM4760_ADC_DEVICE			0x0000
#define BCM4760_ADC_MIN_CHANNEL		0x0000
#define BCM4760_ADC_MAX_CHANNEL		0x0003

#if defined(CONFIG_PMU_DEVICE_BCM59040) || defined(CONFIG_BCM_PMU_BCM59040)
#define BCM59040_ADC_DEVICE			0x0001
#define BCM59040_ADC_MIN_CHANNEL	0x0000
#define BCM59040_ADC_MAX_CHANNEL	0x0009
#endif

struct bcm_adc_info_s;

/*
 * ADC Request structure used to send a request to the
 * bcm_adc_request routine to select the device/channel
 * of the request along with information to get the
 * response back to the caller.
 *
 * The lifetime of this request is from _request() until the handler is
 * called or until _cancel() is used. Note that, especially in the case
 * of _cancel() it is quite possible that the lifetime of a request ends
 * before the ADC driver has finished getting the data.
 */
typedef struct bcm_adc_request_s
{
	int				device;
	int				channel;
	unsigned long int		param;
	void (*callback)(struct bcm_adc_request_s* req,
			unsigned short sample,
			int error);
} bcm_adc_request_t;

/** Entry in the list of requests.
 * The channel from the request is saved so ADC drivers can access that always.
 *
 * A request_entry is valid as long as it is waiting for its turn and until
 * the ADC driver calls the handler to indicate it was done.
 */
typedef struct bcm_adc_request_entry
{
	struct list_head		list;
	struct bcm_adc_request_s	*request;
	struct bcm_adc_info_s		*dev_info;
	int				channel;
} bcm_adc_request_entry_t;

/*
 * completion handler definition
 */
typedef int (*bcm_adc_handler_t)(bcm_adc_request_entry_t* req,
		unsigned short sample,
		int error);


/*
 * external interface prototypes
 */
extern int bcm4760_adc_request(bcm_adc_request_entry_t*);
extern int bcm4760_adc_complete_register(bcm_adc_handler_t);

#if defined(CONFIG_PMU_DEVICE_BCM59040) || defined(CONFIG_BCM_PMU_BCM59040)
extern int bcm59040_adc_request(bcm_adc_request_entry_t *);
extern int bcm59040_adc_complete_register(bcm_adc_handler_t);
#endif

extern u32 bcm_adc_read(int device, int channel);

/*
 * bcm_adc interface prototype
 */
extern int bcm_adc_request(bcm_adc_request_t* req);
extern int bcm_adc_cancel_request(bcm_adc_request_t* req);
#endif
