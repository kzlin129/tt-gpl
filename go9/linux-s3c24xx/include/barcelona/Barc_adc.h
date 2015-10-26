/* include/barcelona/Barc_adc.h
 *
 * Public interface for the ADC driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDE_BARCELONA_BARC_ADC_H
#define __INCLUDE_BARCELONA_BARC_ADC_H

#include <linux/config.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define ADC_DEVNAME		"adc"
#define ADC_MAJOR		121

#define ADC_CHANNELS	16
#define ADC_BUFSIZE		(ADC_CHANNELS * sizeof(short))
#define ADC_RATE		100

/* Touchscreen needs to be controlled one period earlier because of capacitors on the touchscreen signals. TSW channels are dummy to determine switching moment in ADC driver. */
/* Amount of virtual channels seems to be a lot, but only a subset per device is used. */ 

#define ADC_BAT		0
#define ADC_CHRG	1
#define ADC_REF		2
#define ADC_ACC_X	3
#define ADC_ACC_Y	4
#define ADC_ACC_Z_TEMP	5
#define ADC_GYRO	6
#define ADC_GYRO_X	7
#define ADC_GYRO_Y	8
#define ADC_LX_OUT	9
#define ADC_TSW_DOWN	10
#define ADC_TS_DOWN	11
#define ADC_TSW_X	12
#define ADC_TS_X	13
#define ADC_TSW_Y	14
#define ADC_TS_Y	15

/* Sample rate for touchscreen was reduced to 50 Hz because of bigger capacitors on the touchscreen signals on Bergamo/Boston. Please don't change this. */

#define ADC_LRATE		10
#define ADC_MRATE		50
#define ADC_HRATE		100
#define ADC_LTRESHOLD		(ADC_HRATE/ADC_LRATE - 1) 

#define ADC_RATE_BAT		ADC_LRATE
#define ADC_RATE_ALCALINE	ADC_LRATE
#define ADC_RATE_CHRG		ADC_LRATE
#define ADC_RATE_REF		ADC_LRATE
#define ADC_RATE_ACC		ADC_HRATE
#define ADC_RATE_GYRO		ADC_HRATE
#define ADC_RATE_TS		ADC_MRATE
#define ADC_RATE_LX_OUT		ADC_LRATE
#define ADC_RATE_DOCK		ADC_LRATE

/**
 * Events are only sent if the pressure below this value
 * The pressure value goes down when the user presses harder
 **/
#define TSINPUT_PRESSURE_TRIGGER 100

#ifdef __KERNEL__
extern void adc_get_buffer(short *buf);
extern short adc_get_channel(int channel);
typedef void (*adc_pollfunc_t)(short buf[ADC_CHANNELS], void* arg);
extern int adc_register_poll(adc_pollfunc_t func, void* arg, unsigned int rate);
extern int adc_unregister_poll(adc_pollfunc_t func);

struct adc_channels 
{
	char  * name ;
	short   number ;
};
#endif /* __KERNEL__ */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDE_BARCELONA_BARC_ADC_H */

/* EOF */
