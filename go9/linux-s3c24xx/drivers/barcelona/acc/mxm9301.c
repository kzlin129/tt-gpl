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
#include "acciic.h"

#define ACC_ADDRESS 0x20

void mxm9301_Read(short buf[ADC_CHANNELS],ACCMETER_DATA *acc_data)
{
	unsigned short x,y;

	ACCIIC_Start();
	ACCIIC_Transmit(ACC_ADDRESS);
	ACCIIC_Transmit(0);
	ACCIIC_Transmit(0);
	ACCIIC_Transmit(0);
	ACCIIC_Start();
	ACCIIC_Transmit(ACC_ADDRESS);
	ACCIIC_Transmit(1);
	ACCIIC_Start();
	ACCIIC_Transmit(ACC_ADDRESS + 1);
	x  = ACCIIC_Receive(1) << 8;
	x |= ACCIIC_Receive(1);
	y  = ACCIIC_Receive(1) << 8;
	y |= ACCIIC_Receive(0);
	ACCIIC_Stop();

	acc_data->u32xData = x;
	acc_data->u32yData = y;
	acc_data->u32zData = buf[ADC_ACC_Z_TEMP];
	acc_data->u32Temperature = 0;
}
