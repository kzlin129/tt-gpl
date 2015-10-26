/* drivers/barcelona/acc/mxm9301.c
 *
 * Implementation of the accelerometer MXM9301 driver.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <barcelona/Barc_acc.h>
#include <barcelona/Barc_adc.h>

void mxr9500_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data)
{
	acc_data->u32xData = buf[ADC_ACC_X];
	acc_data->u32yData = buf[ADC_ACC_Y];
	acc_data->u32zData = buf[ADC_ACC_Z_TEMP];
	acc_data->u32Temperature = 0;
}
