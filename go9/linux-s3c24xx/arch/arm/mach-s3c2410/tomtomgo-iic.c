/* arch/arm/mach-s3c2410/tomtomgo-iic.c
 *
 * Implementation of the GPIO IIC driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DONT_USE_IOEXPANDER
#ifdef __KERNEL__
#include <asm/uaccess.h>
#include <barcelona/gopins.h>
#endif /* __KERNEL__ */

#ifdef __BOOTLOADER__
#include "compiler.h"
#include "irqsave.h"
#include "timer.h"
#include "gopins.h"
#define EXPORT_SYMBOL(x)
#endif  /* __BOOTLOADER__ */

#ifdef __KERNEL__
/* approximate GCC compiler timings */
#define IIC_SPEED_400 5
#define IIC_SPEED_100 45
#endif
#ifdef __BOOTLOADER__
/* approximate ARM compiler timings */
#define IIC_SPEED_400 5
#define IIC_SPEED_100 45
#endif

#define IIC_SPEED IIC_SPEED_100

#define IIC_GetSDA() IO_GetInput(SW_SDA)

__inline void IIC_SDALow(void)
{
	IO_Deactivate(SW_SDA);
}

__inline void IIC_SDAHigh(void)
{
	IO_SetInput(SW_SDA);
}

__inline void IIC_SCLLow(void)
{
	IO_Deactivate(SW_SCL);
}

__inline void IIC_SCLHigh(void)
{
	IO_SetInput(SW_SCL);
}

/* approximate timings for RVCT2.0 incl call overhead for S3C2440/2442 @ 376.8 Mhz */
static void	IIC_Wait1uSec(void) {
	volatile int i;
	for (i=0; i<(IIC_SPEED); i++);
}
static void	IIC_Wait2uSec(void) {
	volatile int i;
	for (i=0; i<(45 + IIC_SPEED); i++);
}
static void	IIC_Wait3uSec(void) {
	volatile int i;
	for (i=0; i<(70 + IIC_SPEED); i++);
}

void IOIIC_Start(void)
{
	// Make SDA low while SCL is high
	IIC_SDAHigh();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait1uSec();
	IIC_SDALow();
	IIC_Wait1uSec();
	IIC_SCLLow();
	IIC_Wait1uSec();
}

void IOIIC_Stop(void)
{
	// Make SDA high while SCL is high
	IIC_SCLLow();
	IIC_Wait1uSec();
	IIC_SDALow();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait1uSec();
	IIC_SDAHigh();
}

int IOIIC_Transmit(unsigned char data)
{
	int bit;
	int result;
	
	for (bit=0;bit<8;bit++)
	{
		if (data & 0x80) IIC_SDAHigh(); else IIC_SDALow();
		IIC_Wait1uSec();
		IIC_SCLHigh();
		IIC_Wait3uSec();
		data <<= 1;
		IIC_SCLLow();
		IIC_Wait1uSec();
	}
	IIC_SDAHigh();
	IIC_Wait1uSec();
	IIC_SCLHigh();
	IIC_Wait1uSec();
	if (IIC_GetSDA() == 0) result = 1; else result = 0;
	IIC_Wait1uSec();
	IIC_SCLLow();
	IIC_Wait1uSec();

	return result;
}

unsigned char IOIIC_Receive(int generateAck)
{
	int bit;
	unsigned char data = 0;
	
	IIC_SDAHigh();
	for (bit=0;bit<8;bit++)
	{
		data <<= 1; 
		IIC_SCLHigh();
		IIC_Wait2uSec();
		if (IIC_GetSDA()) data |= 1;
		IIC_Wait2uSec();
		IIC_SCLLow();
		IIC_Wait2uSec();
	}
	if (generateAck)
	{
		IIC_SDALow(); /* ack = low */
		IIC_Wait2uSec();
		IIC_SCLHigh();
		IIC_Wait2uSec();
		IIC_SCLLow();
		IIC_Wait2uSec();
	} 
	
	return data;
}
#endif //DONT_USE_IOEXPANDER
