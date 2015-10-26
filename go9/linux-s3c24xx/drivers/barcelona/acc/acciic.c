/* drivers/barcelona/acc/acciic.c
 *
 * Implementation of the accelerometer IIC driver.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <barcelona/gopins.h>

#define IIC_SDALow() IO_Deactivate(HW_IIC_SDA)
#define IIC_SDAHigh() IO_SetInput(HW_IIC_SDA)
#define IIC_GetSDA() IO_GetInput(HW_IIC_SDA)
#define IIC_SCLLow() IO_Deactivate(HW_IIC_SCL)
#define IIC_SCLHigh() IO_SetInput(HW_IIC_SCL)

void ACCIIC_Start(void)
{
	// Make SDA low while SCL is high
	IIC_SDAHigh();
	IIC_SCLHigh();
	IIC_SDALow();
	IIC_SCLLow();
}

void ACCIIC_Stop(void)
{
	// Make SDA high while SCL is high
	IIC_SCLLow();
	IIC_SDALow();
	IIC_SCLHigh();
	IIC_SDAHigh();
}

int ACCIIC_Transmit(unsigned char data)
{
	int bit;
	int result;
	
	for (bit=0;bit<8;bit++)
	{
		if (data & 0x80) IIC_SDAHigh(); else IIC_SDALow();
		IIC_SCLHigh();
		data <<= 1;
		IIC_SCLLow();
	}
	IIC_SDAHigh();
	IIC_SCLHigh();
	if (IIC_GetSDA() == 0) result = 1; else result = 0;
	IIC_SCLLow();

	return result;
}

unsigned char ACCIIC_Receive(int generateAck)
{
	int bit;
	unsigned char data = 0;
	
	IIC_SDAHigh();
	for (bit=0;bit<8;bit++)
	{
		data <<= 1; 
		IIC_SCLHigh();
		if (IIC_GetSDA()) data |= 1;
		IIC_SCLLow();
	}
	if (generateAck)
	{
		IIC_SDALow(); /* ack = low */
		IIC_SCLHigh();
		IIC_SCLLow();
	} else {
		IIC_SDAHigh(); /* no ack = high, but still a clock */
		IIC_SCLHigh();
		IIC_SCLLow();
	}
	
	return data;
}
